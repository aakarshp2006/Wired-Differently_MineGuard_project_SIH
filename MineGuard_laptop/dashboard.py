"""
dashboard.py
-------------
MineGuard - Smart Mine Vehicle Safety System
Phase 9: Web Dashboard

Runs the exact same sense -> decide loop as integration_controller.py
(camera -> ai_detector -> read_telemetry -> risk_engine), but instead
of only printing to the terminal, it also publishes the latest
decision over a small local Flask web server -- so it can be watched
live in a browser, on the laptop itself or on any other device on the
same MineGuard WiFi network (handy for a demo: point a judge's phone
at the URL).

Architecture:
  - A background thread ("sensor_loop") runs the sense -> decide loop
    forever, same as integration_controller.main(), and writes the
    latest result into a small shared dict (_latest_result), guarded
    by a lock since two threads touch it.
  - The Flask app's main thread just serves two routes:
      "/"            -> the dashboard page (HTML + a little JS)
      "/api/status"  -> the latest result as JSON
  - The page polls "/api/status" once a second and updates the
    numbers/colours in place -- no page reloads.

Install (one-time):
    pip install flask

Run:
    python dashboard.py

Then open in a browser:
    http://localhost:5000
    (or http://<this laptop's IP on the MineGuard WiFi>:5000 from
    another device on the same network, e.g. for a demo)
"""

import threading
import time

from flask import Flask, Response, jsonify, render_template_string

import cv2

from ai_detector import MineGuardVision, ESP32_CAM_URL, YOLO_MODEL, CONFIDENCE_THRESHOLD
from read_telemetry import get_telemetry, ESP32_URL as TELEMETRY_URL
from risk_engine import calculate_risk
from integration_controller import _extract_object_type, _extract_fog_level, LOOP_INTERVAL_SECONDS

# --------------------------------------------------------------------
# SHARED STATE (written by the sensor thread, read by the Flask thread)
# --------------------------------------------------------------------

_state_lock = threading.Lock()

# Shared JPEG-encoded latest camera frame (bytes, or None until the first
# frame arrives). Written by capture_loop, read by the /video_feed route.
_frame_lock = threading.Lock()
_latest_frame = None

# Shared raw (numpy) latest camera frame -- written by capture_loop,
# read by sensor_loop for YOLO detection. Kept separate from the JPEG
# bytes above so encoding only happens once per captured frame.
_raw_frame_lock = threading.Lock()
_latest_raw_frame = None

_latest_result = {
    "risk": "WAITING",
    "action": "-",
    "object": "-",
    "distance": 0.0,
    "speed": 0.0,
    "fog_level": 0.0,
    "recommended_speed": 0.0,
    "reason": "Waiting for the first sensor reading...",
    "operation_recommended": True,
    "connected": False,
}


def _update_state(new_values):
    global _latest_result
    with _state_lock:
        _latest_result = new_values


def _read_state():
    with _state_lock:
        return dict(_latest_result)


def _update_frame(jpeg_bytes):
    global _latest_frame
    with _frame_lock:
        _latest_frame = jpeg_bytes


def _read_frame():
    with _frame_lock:
        return _latest_frame


def _update_raw_frame(frame):
    global _latest_raw_frame
    with _raw_frame_lock:
        _latest_raw_frame = frame


def _read_raw_frame():
    with _raw_frame_lock:
        return _latest_raw_frame


# --------------------------------------------------------------------
# CAPTURE LOOP (background thread) -- reads the camera stream as fast
# as possible, so the underlying network/stream buffer never builds up.
# sensor_loop and /video_feed both just read whatever this last wrote.
# --------------------------------------------------------------------

_capture_status = {"ok": None, "reason": ""}  # ok: None=not started yet, True/False after first attempt
_capture_status_lock = threading.Lock()


def _set_capture_status(ok, reason=""):
    global _capture_status
    with _capture_status_lock:
        _capture_status = {"ok": ok, "reason": reason}


def get_capture_status():
    with _capture_status_lock:
        return dict(_capture_status)


def capture_loop():
    cap = cv2.VideoCapture(ESP32_CAM_URL)
    # Best-effort: on backends that honour this, keeps the internal
    # buffer at 1 frame so read() always returns the newest one rather
    # than the oldest queued one. Harmless no-op on backends that don't
    # support it (e.g. many MJPEG-over-HTTP streams via FFMPEG).
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        _set_capture_status(False, f"Could not open camera stream at {ESP32_CAM_URL}")
        return

    _set_capture_status(True)
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                # Transient read failure -- don't spin a hot loop on it.
                time.sleep(0.05)
                continue

            _update_raw_frame(frame)
            ok, jpeg = cv2.imencode(".jpg", frame)
            if ok:
                _update_frame(jpeg.tobytes())
    finally:
        cap.release()


# --------------------------------------------------------------------
# SENSOR LOOP (background thread) -- mirrors integration_controller.main()
# --------------------------------------------------------------------

def sensor_loop():
    try:
        vision = MineGuardVision(model_path=YOLO_MODEL, confidence_threshold=CONFIDENCE_THRESHOLD)
    except RuntimeError as exc:
        _update_state({
            "risk": "ERROR", "action": "-", "object": "-", "distance": 0.0,
            "speed": 0.0, "fog_level": 0.0, "recommended_speed": 0.0,
            "reason": f"Vision init failed: {exc}",
            "operation_recommended": False, "connected": False,
        })
        return

    # Wait briefly for capture_loop to open the stream and confirm status,
    # rather than racing it.
    for _ in range(50):  # up to ~5s
        status = get_capture_status()
        if status["ok"] is not None:
            break
        time.sleep(0.1)
    else:
        status = get_capture_status()

    if status["ok"] is False:
        _update_state({
            "risk": "ERROR", "action": "-", "object": "-", "distance": 0.0,
            "speed": 0.0, "fog_level": 0.0, "recommended_speed": 0.0,
            "reason": status["reason"] or f"Could not open camera stream at {ESP32_CAM_URL}",
            "operation_recommended": False, "connected": False,
        })
        return

    while True:
        frame = _read_raw_frame()  # latest frame captured by capture_loop; may be None briefly at startup
        vision_result = vision.process_frame(frame)

        telemetry = get_telemetry()
        if telemetry is None:
            stale = _read_state()
            stale["connected"] = False
            stale["reason"] = "No telemetry from ESP32 this cycle -- skipping."
            _update_state(stale)
            time.sleep(LOOP_INTERVAL_SECONDS)
            continue

        distance = telemetry.get("distance_m", 0.0)
        speed = telemetry.get("speed_mps", 0.0)
        object_type = _extract_object_type(vision_result)
        fog_level = _extract_fog_level(vision_result)

        risk_result = calculate_risk(object_type, distance, speed, fog_level)
        risk_result["connected"] = True

        _update_state(risk_result)
        time.sleep(LOOP_INTERVAL_SECONDS)


# --------------------------------------------------------------------
# FLASK APP
# --------------------------------------------------------------------

app = Flask(__name__)

# Colour per risk tier -- kept in one place so the page and any future
# view (e.g. a status LED) can share the same mapping.
RISK_COLORS = {
    "SAFE": "#22c55e",
    "CAUTION": "#eab308",
    "HIGH RISK": "#f97316",
    "CRITICAL": "#ef4444",
    "WAITING": "#6b7280",
    "ERROR": "#6b7280",
}

# --------------------------------------------------------------------
# Design notes (for future reference / next redesign pass):
#   - HUD / perception-sweep aesthetic, grounded in the fact that this
#     is a vehicle sensor readout, not a generic SaaS panel.
#   - Palette: near-black navy (#0a0e14) base, cyan (#38bdf8) reserved
#     ONLY for the sensor cone (sensing itself, not a risk signal), so
#     it never gets confused with the SAFE/CAUTION/HIGH RISK/CRITICAL
#     colour coding.
#   - Type: monospace for live numbers (instrument-panel readout feel),
#     system sans for labels. No external font loading -- this may run
#     with no internet access underground.
#   - Layout: sweep visual on top, flat divider-separated stat rows
#     (deliberately NOT individual bordered "cards" -- keeps the HUD
#     feel and matches the reference brief) then a single risk banner.
#   - Object's position in the sweep is driven by real distance data
#     only; there is no lateral/left-right sensor yet, so the object
#     marker stays on the centreline (see SWEEP_MAX_DISTANCE_M) rather
#     than fabricating a direction.
# --------------------------------------------------------------------

DASHBOARD_HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>MineGuard Dashboard</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  :root {
    color-scheme: dark;
    --mono: ui-monospace, SFMono-Regular, "SF Mono", Menlo, Consolas, "Liberation Mono", monospace;
    --sans: -apple-system, "Segoe UI", Roboto, sans-serif;
    --bg: #0a0e14;
    --panel: #0d1420;
    --line: #1e293b;
    --text: #e2e8f0;
    --muted: #64748b;
    --cyan: #38bdf8;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; padding: 20px;
    background: var(--bg); color: var(--text);
    font-family: var(--sans);
  }
  .wrap { max-width: 440px; margin: 0 auto; }

  .topbar {
    display: flex; align-items: center; justify-content: space-between;
    margin-bottom: 14px;
  }
  .topbar h1 { font-size: 1rem; font-weight: 600; margin: 0; letter-spacing: 0.3px; }
  .conn { font-size: 0.75rem; color: var(--muted); display: flex; align-items: center; gap: 6px; }
  .dot { width: 7px; height: 7px; border-radius: 50%; background: var(--muted); transition: background-color 0.3s; }

  /* --- Perception sweep --- */
  .sweep {
    position: relative;
    height: 190px;
    border-radius: 18px 18px 0 0;
    overflow: hidden;
    background: linear-gradient(180deg, #0e1b2b 0%, var(--panel) 100%);
    border: 1px solid var(--line);
    border-bottom: none;
  }
  .sweep-eyebrow {
    position: absolute; top: 12px; left: 16px;
    font-size: 0.68rem; letter-spacing: 1.5px; color: var(--muted);
    text-transform: uppercase;
  }
  .sweep-cone {
    position: absolute; top: 0; left: 50%;
    width: 320px; height: 190px;
    transform: translateX(-50%);
    clip-path: polygon(44% 100%, 56% 100%, 96% 6%, 4% 6%);
    background: radial-gradient(ellipse at 50% 100%, rgba(56,189,248,0.30) 0%, rgba(56,189,248,0.07) 55%, transparent 78%);
  }
  .sweep-scanline {
    position: absolute; left: 8%; right: 8%; height: 2px;
    background: linear-gradient(90deg, transparent, rgba(56,189,248,0.55), transparent);
    animation: scan 3.2s ease-in-out infinite;
  }
  @keyframes scan {
    0%   { top: 90%; opacity: 0; }
    10%  { opacity: 0.9; }
    50%  { top: 15%; opacity: 0.5; }
    90%  { opacity: 0.9; }
    100% { top: 90%; opacity: 0; }
  }
  @media (prefers-reduced-motion: reduce) {
    .sweep-scanline { animation: none; top: 50%; opacity: 0.3; }
  }
  .sweep-centerline {
    position: absolute; top: 6%; bottom: 0; left: 50%;
    border-left: 1px dashed rgba(148,163,184,0.3);
  }
  .sweep-vehicle {
    position: absolute; bottom: 8px; left: 50%; transform: translateX(-50%);
    font-size: 1.3rem; filter: drop-shadow(0 0 6px rgba(56,189,248,0.5));
  }
  .sweep-badge {
    position: absolute; left: 50%;
    transform: translate(-50%, -50%);
    background: rgba(10,14,20,0.9);
    border: 1px solid var(--line);
    border-radius: 999px;
    padding: 5px 12px;
    font-family: var(--mono);
    font-size: 0.78rem;
    white-space: nowrap;
    transition: top 0.4s ease, border-color 0.3s ease, color 0.3s ease;
  }

  /* --- Stat rows (flat, divider-based -- not cards) --- */
  .stats {
    background: var(--panel);
    border: 1px solid var(--line);
    border-top: none;
    padding: 4px 18px;
  }
  .row {
    display: flex; justify-content: space-between; align-items: baseline;
    padding: 13px 0;
    border-bottom: 1px solid var(--line);
  }
  .row:last-child { border-bottom: none; }
  .row .k {
    font-size: 0.7rem; letter-spacing: 1px; color: var(--muted);
    text-transform: uppercase;
  }
  .row .v {
    font-family: var(--mono); font-size: 1rem; font-weight: 600;
  }

  /* --- Risk banner --- */
  .banner {
    display: flex; align-items: center; justify-content: space-between;
    gap: 12px;
    border: 1px solid var(--line);
    border-radius: 0 0 18px 18px;
    padding: 14px 18px;
    background: var(--panel);
    transition: background-color 0.3s ease, border-color 0.3s ease;
  }
  .pill {
    font-family: var(--mono); font-size: 0.78rem; font-weight: 700;
    letter-spacing: 0.5px;
    padding: 6px 14px; border-radius: 999px;
    border: 1px solid currentColor;
  }
  .banner .verdict { font-weight: 700; font-size: 0.95rem; }

  .not-recommended {
    margin-top: 10px;
    background: rgba(127,29,29,0.35);
    border: 1px solid #ef4444;
    color: #fecaca;
    border-radius: 10px;
    padding: 9px 14px;
    font-size: 0.82rem; font-weight: 600; text-align: center;
    display: none;
  }
  .reason {
    margin-top: 10px;
    font-size: 0.8rem; color: var(--muted);
    text-align: center;
  }
</style>
</head>
<body>
<div class="wrap">

  <div class="topbar">
    <h1>MineGuard</h1>
    <div class="conn"><span class="dot" id="conn-dot"></span><span id="conn-text">Connecting...</span></div>
  </div>

  <div class="sweep">
    <div class="sweep-eyebrow">Perception sweep</div>
    <div class="sweep-cone"></div>
    <div class="sweep-scanline"></div>
    <div class="sweep-centerline"></div>
    <div class="sweep-badge" id="sweep-badge" style="top:170px;">-- m</div>
    <div class="sweep-vehicle">&#9650;</div>
  </div>

  <div class="stats">
    <div class="row"><span class="k">Object detected</span><span class="v" id="v-object">-</span></div>
    <div class="row"><span class="k">Distance</span><span class="v" id="v-distance">-</span></div>
    <div class="row"><span class="k">Time to collision</span><span class="v" id="v-ttc">-</span></div>
    <div class="row"><span class="k">Visibility</span><span class="v" id="v-fog">-</span></div>
    <div class="row"><span class="k">Current speed</span><span class="v" id="v-speed">-</span></div>
    <div class="row"><span class="k">Recommended</span><span class="v" id="v-recspeed">-</span></div>
  </div>

  <div class="banner" id="banner">
    <span class="pill" id="risk-pill">WAITING</span>
    <span class="verdict" id="verdict">--</span>
  </div>

  <div class="not-recommended" id="not-recommended">OPERATION NOT RECOMMENDED</div>
  <div class="reason" id="v-reason">Waiting for the first reading...</div>

</div>

<script>
const COLORS = {
  "SAFE": "#22c55e", "CAUTION": "#eab308", "HIGH RISK": "#f97316",
  "CRITICAL": "#ef4444", "WAITING": "#64748b", "ERROR": "#64748b"
};
const VERDICTS = {
  "SAFE": "Proceed", "CAUTION": "Ease off speed", "HIGH RISK": "Reduce speed",
  "CRITICAL": "Stop now", "WAITING": "--", "ERROR": "Sensor error"
};
// Visual scale for the sweep: how far (metres) counts as "far" on screen.
// Tuned for the current prototype-scale safety distances, not real
// industrial mine distances -- see risk_engine.py's SAFETY_LIMITS.
const SWEEP_MAX_DISTANCE_M = 2.0;

function visibilityLabel(fogPct) {
  if (fogPct > 90) return "VERY LOW";
  if (fogPct > 70) return "LOW";
  if (fogPct > 40) return "MODERATE";
  return "CLEAR";
}

function fmt(n, digits) {
  return Number(n).toFixed(digits);
}

async function poll() {
  try {
    const res = await fetch("/api/status");
    const data = await res.json();
    const color = COLORS[data.risk] || "#64748b";

    // Sweep badge: position by real distance, no fabricated direction.
    const frac = Math.min(data.distance / SWEEP_MAX_DISTANCE_M, 1);
    const topPx = 168 - frac * 148;
    const badge = document.getElementById("sweep-badge");
    badge.style.top = topPx + "px";
    badge.style.borderColor = color;
    badge.style.color = color;
    badge.textContent = (data.object === "none" ? "clear" : data.object) + " \\u2022 " + fmt(data.distance, 2) + " m";

    document.getElementById("v-object").textContent = data.object;
    document.getElementById("v-distance").textContent = fmt(data.distance, 2) + " m";
    document.getElementById("v-ttc").textContent = (data.ttc === null) ? "\\u221e" : fmt(data.ttc, 1) + " s";
    document.getElementById("v-fog").textContent = fmt(data.fog_level, 0) + "% \\u2014 " + visibilityLabel(data.fog_level);
    document.getElementById("v-speed").textContent = fmt(data.speed * 3.6, 0) + " km/h";

    const recEl = document.getElementById("v-recspeed");
    recEl.textContent = fmt(data.recommended_speed * 3.6, 0) + " km/h";
    recEl.style.color = color;

    document.getElementById("risk-pill").textContent = data.risk;
    document.getElementById("risk-pill").style.color = color;
    document.getElementById("verdict").textContent = VERDICTS[data.risk] || data.action;
    document.getElementById("verdict").style.color = color;
    document.getElementById("banner").style.borderColor = color;

    document.getElementById("not-recommended").style.display = data.operation_recommended ? "none" : "block";
    document.getElementById("v-reason").textContent = data.reason;

    const dot = document.getElementById("conn-dot");
    const text = document.getElementById("conn-text");
    if (data.connected) {
      dot.style.backgroundColor = "#22c55e";
      text.textContent = "Connected";
    } else {
      dot.style.backgroundColor = "#ef4444";
      text.textContent = "No telemetry";
    }
  } catch (err) {
    document.getElementById("conn-text").textContent = "Dashboard server unreachable";
  }
}

poll();
setInterval(poll, 1000);
</script>
</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(DASHBOARD_HTML)


def _frame_generator():
    """Yields the latest JPEG frame as a multipart MJPEG stream.

    Re-sends the same frame if sensor_loop hasn't produced a newer one
    yet (e.g. camera briefly stalls) rather than blocking, so the
    connection stays alive.
    """
    boundary = b"--frame"
    while True:
        jpeg_bytes = _read_frame()
        if jpeg_bytes is not None:
            yield (
                boundary + b"\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: " + str(len(jpeg_bytes)).encode() + b"\r\n\r\n"
                + jpeg_bytes + b"\r\n"
            )
        time.sleep(0.1)  # ~10 fps to the browser, independent of sensor_loop's pace


@app.route("/video_feed")
def video_feed():
    """Raw MJPEG stream of the camera feed.

    Intentionally NOT embedded in the driver-facing dashboard page ("/")
    -- a continuously-playing video panel is a distraction risk for a
    driver in-cab. Kept as its own route so it can still be opened
    directly (e.g. on a separate demo screen for judges, or a future
    control-room/monitoring view) without being on the driver's HUD.
    """
    return Response(
        _frame_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/api/status")
def api_status():
    return jsonify(_read_state())


# --------------------------------------------------------------------
# MAIN
# --------------------------------------------------------------------

def main():
    capture_thread = threading.Thread(target=capture_loop, daemon=True)
    capture_thread.start()

    sensor_thread = threading.Thread(target=sensor_loop, daemon=True)
    sensor_thread.start()

    print("=" * 60)
    print("MineGuard - Web Dashboard")
    print("=" * 60)
    print("Open in a browser: http://localhost:5000")
    print("(Or http://<this-laptop-IP-on-MineGuard-WiFi>:5000 from another device)")
    print("Ctrl+C to stop")
    print("=" * 60)

    app.run(host="0.0.0.0", port=5000, debug=False)


if __name__ == "__main__":
    main()
