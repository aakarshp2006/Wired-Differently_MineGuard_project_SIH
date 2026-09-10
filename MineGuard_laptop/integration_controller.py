"""
integration_controller.py
--------------------------
MineGuard - Smart Mine Vehicle Safety System
Integration Controller

Wires together the three modules built so far:

  1. ai_detector.py     -> MineGuardVision.process_frame(frame)
                            gives object type + fog reading from the
                            ESP32-CAM stream (YOLO + fog fusion).
  2. read_telemetry.py  -> get_telemetry()
                            gives distance_m + speed_mps from the
                            Normal ESP32 (HC-SR04 + LM393 encoder).
  3. risk_engine.py     -> calculate_risk(object_type, distance,
                            speed, fog_level)
                            gives the final SAFE / WARNING / DANGER
                            decision.

Current behaviour (by design, confirmed with the team):
  - This controller only READS sensors and PRINTS the decision to the
    terminal every cycle. It does NOT send any command back to the
    ESP32 -- no automatic "/emergency_stop" call.
  - When risk_result["action"] == "STOP", the terminal prints a clear
    recommendation; the driver is expected to act on it manually from
    the dashboard.
  - The one line that WOULD send an automatic stop command is present
    but commented out in _maybe_send_emergency_stop(), so switching to
    automatic intervention later is a one-line change, not a rewrite.

Edge-case handling (see _extract_object_type / _extract_fog_level):
  - No object detected ("none") or ai_status == "ERROR"
        -> treated as "obstacle" (never silently treated as "no risk").
  - camera_status/fog_status == "ERROR" (camera/fog pipeline broken)
        -> fog_level forced to 100 (worst case), so risk_engine.py's
           fog > 90% hard cutoff fires automatically. A broken camera
           must never be read as "clear weather".
"""

import time

import cv2

from ai_detector import MineGuardVision, ESP32_CAM_URL, YOLO_MODEL, CONFIDENCE_THRESHOLD
from read_telemetry import get_telemetry, ESP32_URL as TELEMETRY_URL
from risk_engine import calculate_risk

# How often the main loop runs (seconds): one camera frame + one
# telemetry poll per cycle.
LOOP_INTERVAL_SECONDS = 1.0

# Fog level assumed when the camera/AI/fog pipeline itself is broken.
# 100 guarantees risk_engine.py's FOG_CRITICAL_PERCENT cutoff fires.
FOG_LEVEL_ON_VISION_FAILURE = 100.0


# --------------------------------------------------------------------
# TRANSLATION HELPERS: ai_detector output -> risk_engine input
# --------------------------------------------------------------------

def _extract_object_type(vision_result):
    """
    Map ai_detector.py's "object" field onto what risk_engine.py
    expects. "none" (nothing detected) and a failed ai_status both
    fall back to "obstacle" -- an absent detection is never read as
    "definitely nothing there", since it may just mean YOLO missed it.
    """
    if vision_result.get("ai_status") != "OK":
        return "obstacle"

    detected = vision_result.get("object", "obstacle")
    return "obstacle" if detected == "none" else detected


def _extract_fog_level(vision_result):
    """
    Map ai_detector.py's fog_percentage onto the 0-100 fog_level
    risk_engine.py expects. Any camera/fog failure is treated as
    worst-case fog rather than "no fog", per the module's own
    "never report failure as clear weather" principle.
    """
    if vision_result.get("camera_status") != "OK":
        return FOG_LEVEL_ON_VISION_FAILURE
    if vision_result.get("fog_status") != "OK":
        return FOG_LEVEL_ON_VISION_FAILURE

    fog_percentage = vision_result.get("fog_percentage")
    if fog_percentage is None:
        return FOG_LEVEL_ON_VISION_FAILURE

    return float(fog_percentage)


# --------------------------------------------------------------------
# ACTION HANDLING
# --------------------------------------------------------------------

def _maybe_send_emergency_stop(risk_result):
    """
    Placeholder for automatic intervention -- DISABLED for now.

    Current design: the driver decides from the dashboard when to
    actually stop the vehicle. To switch to automatic intervention
    later, uncomment the requests.post() call below. Nothing else in
    this file needs to change.
    """
    if risk_result["action"] != "STOP":
        return

    # --- Uncomment to enable automatic emergency stop -------------
    # import requests
    # try:
    #     requests.post(f"{TELEMETRY_URL.rsplit('/', 1)[0]}/emergency_stop", timeout=2)
    # except requests.exceptions.RequestException as error:
    #     print(f"[integration_controller] Failed to send emergency stop: {error}")


def _print_decision(risk_result):
    """One clean, readable status line per cycle, plus a loud banner
    when the operation itself isn't recommended (e.g. fog cutoff)."""
    icon = {"SAFE": "\u2705", "CAUTION": "\U0001f7e1", "HIGH RISK": "\U0001f7e0", "CRITICAL": "\U0001f6a8"}.get(
        risk_result["risk"], ""
    )

    print(
        f"{icon} {risk_result['risk']:<10} | "
        f"object={risk_result['object']:<10} "
        f"dist={risk_result['distance']}m "
        f"speed={risk_result['speed']}m/s "
        f"fog={risk_result['fog_level']}% "
        f"| action={risk_result['action']} "
        f"rec_speed={risk_result['recommended_speed']}m/s "
        f"| {risk_result['reason']}"
    )

    if not risk_result["operation_recommended"]:
        print("   >>> OPERATION NOT RECOMMENDED -- do not proceed. <<<")


# --------------------------------------------------------------------
# MAIN LOOP
# --------------------------------------------------------------------

def main():
    print("=" * 60)
    print("MineGuard - Integration Controller")
    print("=" * 60)
    print(f"Camera stream : {ESP32_CAM_URL}")
    print(f"Telemetry     : {TELEMETRY_URL}")
    print("Ctrl+C to stop")
    print("=" * 60)

    try:
        vision = MineGuardVision(
            model_path=YOLO_MODEL, confidence_threshold=CONFIDENCE_THRESHOLD
        )
    except RuntimeError as exc:
        print(exc)
        raise SystemExit(1)

    cap = cv2.VideoCapture(ESP32_CAM_URL)
    if not cap.isOpened():
        print(f"[integration_controller] ERROR: could not open camera stream at {ESP32_CAM_URL}")
        print("Check that the ESP32-CAM is powered on and connected to Wi-Fi.")
        raise SystemExit(1)

    try:
        while True:
            ret, frame = cap.read()

            # Feed even a failed read through process_frame() so the
            # health flags (and therefore fog_level=100 fallback)
            # reflect the real camera state instead of skipping silently.
            vision_result = vision.process_frame(frame if ret else None)

            telemetry = get_telemetry()
            if telemetry is None:
                print("[integration_controller] No telemetry from ESP32 this cycle -- skipping.")
                time.sleep(LOOP_INTERVAL_SECONDS)
                continue

            distance = telemetry.get("distance_m", 0.0)
            speed = telemetry.get("speed_mps", 0.0)

            object_type = _extract_object_type(vision_result)
            fog_level = _extract_fog_level(vision_result)

            risk_result = calculate_risk(object_type, distance, speed, fog_level)

            _print_decision(risk_result)
            _maybe_send_emergency_stop(risk_result)

            time.sleep(LOOP_INTERVAL_SECONDS)

    except KeyboardInterrupt:
        print("\n[integration_controller] Interrupted by user, shutting down.")
    finally:
        cap.release()
        print("[integration_controller] Camera released cleanly.")


if __name__ == "__main__":
    main()
