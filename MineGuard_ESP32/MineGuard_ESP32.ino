/*
 * ============================================================
 *  MineGuard — Member 1
 *  Onboard Dumper Control & Vehicle Telemetry System
 * ============================================================
 *
 *  Hardware:
 *    - ESP32 DevKit V1
 *    - L298N Motor Driver
 *    - 2x DC Motors
 *    - LM393 Encoder / Slot Sensor
 *
 *  Control Philosophy:
 *    SAFE     -> Driver operates normally via the web console.
 *    WARNING  -> Risk Engine (Member 4) may report a recommended
 *                 safe speed; this ESP32 NEVER auto-adjusts
 *                 throttlePWM in response. The driver manually
 *                 reduces throttle.
 *    DANGER   -> Member 5 (Integration Controller) calls
 *                 /emergency_stop. Motors are forced OFF
 *                 immediately and ALL movement commands are
 *                 blocked until /clear_emergency is called.
 *
 *  This module is strictly driver-operated. It never drives
 *  autonomously, never auto-avoids obstacles, and never
 *  auto-adjusts throttle. Only emergency stop is automatic.
 * ============================================================
 *
 *  v3 CHANGE NOTES (engineering hardening pass, same architecture):
 *    1. updateTelemetry() now uses the actual measured elapsed
 *       time (millis()-based, non-blocking) instead of assuming
 *       the nominal SPEED_UPDATE_INTERVAL_MS.
 *    2. An exponential moving average (EMA) low-pass filter has
 *       been added to smooth LM393 encoder noise. Raw speed is
 *       still computed each cycle; the FILTERED value is what is
 *       exposed via currentSpeedMps / currentSpeedKmh.
 *    3. Defense-in-depth: moveForward()/moveBackward()/turnLeft()/
 *       turnRight() now also check currentSafetyState directly, so
 *       emergency stop cannot be bypassed even by a future internal
 *       caller that skips the HTTP handlers.
 *    4. /telemetry now reports "speed_source":"LM393_ENCODER".
 *    5. Filtered speed is snapped to exactly 0 once it decays below
 *       a small epsilon, so a stopped vehicle reads 0.00, not a
 *       lingering fractional value from the smoothing filter.
 *
 *  FINAL CHANGE NOTES (targeted refinement pass, same architecture):
 *    6. Zero-speed detection is now based on CONSECUTIVE zero-pulse
 *       telemetry cycles (ZERO_SPEED_CONSECUTIVE_CYCLES) rather than
 *       a single zero-pulse reading, so speed/RPM reach a confident
 *       hard zero shortly after the dumper physically stops, without
 *       false-zeroing on one missed/noisy encoder cycle.
 *    7. Added lastCommand tracking (e.g. MANUAL_FORWARD, PWM_UPDATED,
 *       EMERGENCY_STOP), exposed via /telemetry and /status as
 *       "last_command" for diagnostics only — it does not affect
 *       control logic.
 *    8. Emergency stop still cuts motors and sets state immediately,
 *       but no longer artificially zeroes encoder-derived speed —
 *       actual wheel coast-down is reflected honestly via the
 *       consecutive-zero-pulse logic in updateTelemetry().
 * ============================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include "distance_sensor.h"   // Member 2's distance sensor, converted to non-blocking

// ==============================================================
// SECTION 1: CONFIGURATION
// ==============================================================
const char *AP_SSID = "......";
const char *AP_PASSWORD = ".....";

const int PWM_FREQ_HZ = 5000;
const int PWM_RESOLUTION_BITS = 8; // 0-255 duty range

const unsigned long SPEED_UPDATE_INTERVAL_MS = 500;

// ---- CALIBRATION VALUES (must be verified on the physical robot) ----
const float PULSES_PER_REVOLUTION = 20.0f;
const float WHEEL_DIAMETER_METERS = 0.065f;

// ---- SPEED FILTER CONFIGURATION ----
// Exponential moving average: filtered = ALPHA*raw + (1-ALPHA)*filtered.
// ALPHA = 0.35 gives a reasonably quick response (settles in roughly
// 4-5 telemetry cycles, i.e. ~2s at the default 500ms interval) while
// still smoothing out LM393 slot-sensor jitter. Raise ALPHA for a
// snappier (noisier) reading, lower it for heavier smoothing.
const float SPEED_EMA_ALPHA = 0.35f;

// Below this, filtered speed is snapped to exactly 0.0 so a stopped
// vehicle doesn't display a lingering fractional speed while the
// EMA asymptotically decays toward zero.
const float ZERO_SPEED_EPSILON_MPS = 0.01f;

// Number of CONSECUTIVE zero-pulse telemetry cycles required before
// speed/RPM are forced to hard zero. At the default 500ms telemetry
// interval, 3 cycles = ~1.5s of no encoder pulses. This is long enough
// to ignore a single missed/noisy encoder cycle, but short enough that
// the dashboard doesn't show a lingering non-zero speed for long after
// the dumper has physically stopped.
const int ZERO_SPEED_CONSECUTIVE_CYCLES = 3;

const char *SPEED_SOURCE_LABEL = "LM393_ENCODER";

// ==============================================================
// SECTION 2: PIN DEFINITIONS
// ==============================================================
const int ENA = 27;   // Left motor PWM (speed)
const int IN1 = 26;   // Left motor direction pin A
const int IN2 = 25;   // Left motor direction pin B

const int ENB = 14;   // Right motor PWM (speed)
const int IN3 = 33;   // Right motor direction pin A
const int IN4 = 32;   // Right motor direction pin B

// GPIO 34 is input-only on the ESP32 and has no internal pull-up/pull-down.
// The LM393 encoder module must provide a clean digital logic signal
// (its own onboard comparator/pull-up) directly to this pin.
const int ENCODER_PIN = 34;

// ==============================================================
// SECTION 3: VEHICLE STATE DEFINITIONS
// ==============================================================
// Movement state and safety state are kept strictly separate so
// telemetry consumers never have to disambiguate a mixed enum.
enum MovementState {
  MOVEMENT_STOPPED,
  MOVEMENT_FORWARD,
  MOVEMENT_BACKWARD,
  MOVEMENT_LEFT,
  MOVEMENT_RIGHT
};

enum SafetyState {
  SAFETY_NORMAL,
  SAFETY_EMERGENCY_STOP_ACTIVE
};

MovementState currentMovementState = MOVEMENT_STOPPED;
SafetyState currentSafetyState = SAFETY_NORMAL;

// ==============================================================
// SECTION 4: GLOBAL TELEMETRY VARIABLES
// ==============================================================
int throttlePWM = 180; // Driver-controlled throttle, range 0-255

volatile unsigned long encoderPulseCount = 0;

float currentSpeedMpsRaw = 0.0f;  // last raw (unfiltered) calculated speed
float currentSpeedMps = 0.0f;     // EMA-filtered speed (used by telemetry)
float currentSpeedKmh = 0.0f;     // derived from filtered speed
float currentRPM = 0.0f;

unsigned long lastTelemetryUpdate = 0;
unsigned long lastSpeedCalcMs = 0; // for real elapsed-time speed math

// Consecutive telemetry cycles with zero encoder pulses. Used to decide
// when to force speed/RPM to a hard zero (see ZERO_SPEED_CONSECUTIVE_CYCLES).
int consecutiveZeroPulseCycles = 0;

// Most recent command/event received, exposed via /telemetry and /status
// for diagnostics (e.g. distinguishing a driver STOP from an EMERGENCY_STOP
// that both result in MOVEMENT_STOPPED). Purely informational — it does
// not influence any control logic.
String lastCommand = "NONE";

WebServer server(80);

// ==============================================================
// FUNCTION PROTOTYPES
// ==============================================================
const char *movementStateToString(MovementState state);
const char *safetyStateToString(SafetyState state);

void setupPWM();
void applyThrottle();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMotors();

void IRAM_ATTR encoderISR();
void updateTelemetry();

void handleRoot();
void handleForward();
void handleBackward();
void handleLeft();
void handleRight();
void handleStop();
void handleSetPWM();
void handleTelemetry();
void handleStatus();
void handleEmergencyStop();
void handleClearEmergency();
void handleNotFound();

String buildMovementResponse(bool success, const char *errorReason);

// ==============================================================
// EMBEDDED WEB DASHBOARD (HTML + CSS + JS, no external dependency)
// ==============================================================
const char DASHBOARD_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>MineGuard - Onboard Dumper Control</title>
<style>
  * { box-sizing: border-box; }
  body {
    margin: 0;
    font-family: 'Segoe UI', Arial, sans-serif;
    background: #0d1117;
    color: #e6edf3;
    padding: 16px;
  }
  h1 {
    text-align: center;
    letter-spacing: 3px;
    color: #ffb020;
    margin-bottom: 2px;
  }
  h2 {
    text-align: center;
    font-weight: normal;
    color: #9fb1c1;
    margin-top: 0;
    font-size: 13px;
    letter-spacing: 2px;
  }
  .panel {
    background: #161b22;
    border: 1px solid #2a3742;
    border-radius: 10px;
    padding: 16px;
    max-width: 480px;
    margin: 16px auto;
  }
  .panel h3 {
    margin-top: 0;
    color: #ffb020;
    border-bottom: 1px solid #2a3742;
    padding-bottom: 8px;
    font-size: 14px;
    letter-spacing: 1px;
  }
  .status-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
  }
  .status-item {
    background: #0d1117;
    border-radius: 8px;
    padding: 10px;
    text-align: center;
  }
  .status-item .label {
    font-size: 11px;
    color: #9fb1c1;
    letter-spacing: 1px;
  }
  .status-item .value {
    font-size: 18px;
    font-weight: bold;
    color: #4fd1c5;
    margin-top: 4px;
  }
  .value.safety-normal { color: #4fd1c5; }
  .value.safety-emergency {
    color: #ff4d4d;
    animation: blink 1s infinite;
  }
  @keyframes blink {
    0%, 49% { opacity: 1; }
    50%, 100% { opacity: 0.35; }
  }
  .controls {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 8px;
    margin-top: 10px;
  }
  button {
    padding: 14px 8px;
    border: none;
    border-radius: 8px;
    background: #21262d;
    color: #e6edf3;
    font-size: 14px;
    cursor: pointer;
    transition: background 0.15s;
  }
  button:hover:not(:disabled) { background: #2d333b; }
  button:disabled {
    opacity: 0.35;
    cursor: not-allowed;
  }
  .btn-forward { grid-column: 2; }
  .btn-backward { grid-column: 2; }
  .btn-stop { background: #3a2b16; color: #ffb020; }
  .btn-stop:hover:not(:disabled) { background: #4a3820; }
  .btn-emergency {
    width: 100%;
    margin-top: 12px;
    padding: 16px;
    background: #5c1414;
    color: #ff8080;
    font-weight: bold;
    font-size: 16px;
    letter-spacing: 1px;
    border-radius: 8px;
  }
  .btn-emergency:hover { background: #711a1a; }
  .btn-clear {
    width: 100%;
    margin-top: 8px;
    padding: 10px;
    background: #17301f;
    color: #6fd18a;
  }
  .btn-clear:hover { background: #1e3d28; }
  input[type=range] {
    width: 100%;
    margin-top: 8px;
  }
  .pwm-value {
    text-align: center;
    font-size: 18px;
    color: #4fd1c5;
    margin-top: 4px;
  }
  .footer-note {
    text-align: center;
    font-size: 11px;
    color: #5c6b78;
    margin-top: 20px;
  }
</style>
</head>
<body>

<h1>MINEGUARD</h1>
<h2>ONBOARD DUMPER CONTROL</h2>

<div class="panel">
  <h3>VEHICLE STATUS</h3>
  <div class="status-grid">
    <div class="status-item">
      <div class="label">MOVEMENT</div>
      <div class="value" id="movementValue">--</div>
    </div>
    <div class="status-item">
      <div class="label">SAFETY</div>
      <div class="value safety-normal" id="safetyValue">--</div>
    </div>
  </div>
</div>

<div class="panel">
  <h3>LIVE TELEMETRY</h3>
  <div class="status-grid">
    <div class="status-item">
      <div class="label">SPEED</div>
      <div class="value" id="speedMpsValue">-- m/s</div>
    </div>
    <div class="status-item">
      <div class="label">SPEED</div>
      <div class="value" id="speedKmhValue">-- km/h</div>
    </div>
    <div class="status-item">
      <div class="label">RPM</div>
      <div class="value" id="rpmValue">--</div>
    </div>
    <div class="status-item">
      <div class="label">THROTTLE</div>
      <div class="value" id="pwmStatusValue">-- PWM</div>
    </div>
  </div>
</div>

<div class="panel">
  <h3>MANUAL VEHICLE CONTROL</h3>
  <div class="controls">
    <div></div>
    <button class="btn-forward movement-btn"
            onmousedown="startMove('/forward')" onmouseup="stopMove()" onmouseleave="stopMove()"
            ontouchstart="event.preventDefault(); startMove('/forward')" ontouchend="stopMove()" ontouchcancel="stopMove()">FORWARD</button>
    <div></div>

    <button class="movement-btn"
            onmousedown="startMove('/left')" onmouseup="stopMove()" onmouseleave="stopMove()"
            ontouchstart="event.preventDefault(); startMove('/left')" ontouchend="stopMove()" ontouchcancel="stopMove()">LEFT</button>
    <button class="btn-stop" onclick="sendCommand('/stop')">STOP</button>
    <button class="movement-btn"
            onmousedown="startMove('/right')" onmouseup="stopMove()" onmouseleave="stopMove()"
            ontouchstart="event.preventDefault(); startMove('/right')" ontouchend="stopMove()" ontouchcancel="stopMove()">RIGHT</button>

    <div></div>
    <button class="btn-backward movement-btn"
            onmousedown="startMove('/backward')" onmouseup="stopMove()" onmouseleave="stopMove()"
            ontouchstart="event.preventDefault(); startMove('/backward')" ontouchend="stopMove()" ontouchcancel="stopMove()">BACKWARD</button>
    <div></div>
  </div>
</div>

<div class="panel">
  <h3>MANUAL THROTTLE</h3>
  <input type="range" id="pwmSlider" min="0" max="255" value="180"
         oninput="document.getElementById('pwmLiveValue').innerText = this.value"
         onchange="setPWM(this.value)">
  <div class="pwm-value">Current: <span id="pwmLiveValue">180</span></div>
</div>

<div class="panel">
  <h3>EMERGENCY CONTROL</h3>
  <button class="btn-emergency" onclick="sendCommand('/emergency_stop')">EMERGENCY STOP</button>
  <button class="btn-clear" onclick="sendCommand('/clear_emergency')">CLEAR EMERGENCY</button>
</div>

<div class="footer-note">MineGuard Prototype &mdash; Member 1 Onboard Control</div>

<script>
function sendCommand(path) {
  fetch(path).catch(function(err) { console.error(err); });
}

// Press-and-hold movement: startMove() fires when the button is
// pressed down (mouse or touch), stopMove() fires the moment it is
// released, dragged off, or the touch is cancelled -- so the robot
// only moves while the button is actually held.
function startMove(path) {
  sendCommand(path);
}

function stopMove() {
  sendCommand('/stop');
}

function setPWM(value) {
  fetch('/set_pwm?value=' + value).catch(function(err) { console.error(err); });
}

function setMovementButtonsEnabled(enabled) {
  var buttons = document.getElementsByClassName('movement-btn');
  for (var i = 0; i < buttons.length; i++) {
    buttons[i].disabled = !enabled;
  }
}

function refreshTelemetry() {
  fetch('/telemetry')
    .then(function(res) { return res.json(); })
    .then(function(data) {
      document.getElementById('movementValue').innerText = data.movement_state;

      var safetyEl = document.getElementById('safetyValue');
      safetyEl.innerText = data.safety_state;
      if (data.safety_state === 'EMERGENCY_STOP_ACTIVE') {
        safetyEl.classList.remove('safety-normal');
        safetyEl.classList.add('safety-emergency');
        setMovementButtonsEnabled(false);
      } else {
        safetyEl.classList.remove('safety-emergency');
        safetyEl.classList.add('safety-normal');
        setMovementButtonsEnabled(true);
      }

      document.getElementById('speedMpsValue').innerText = data.speed_mps.toFixed(2) + ' m/s';
      document.getElementById('speedKmhValue').innerText = data.speed_kmh.toFixed(2) + ' km/h';
      document.getElementById('rpmValue').innerText = data.rpm.toFixed(1);
      document.getElementById('pwmStatusValue').innerText = data.throttle_pwm + ' PWM';
    })
    .catch(function(err) { console.error(err); });
}

setInterval(refreshTelemetry, 500);
refreshTelemetry();
</script>

</body>
</html>
)HTMLPAGE";

// ==============================================================
// SETUP
// ==============================================================
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  setupPWM();

  // GPIO 34 has no internal pull resistor; the LM393 module supplies
  // its own clean digital output, so plain INPUT mode is used here.
  pinMode(ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);

  beginDistanceSensor();   // Member 2: sets up TRIG/ECHO pins for HC-SR04

  stopMotors();
  currentMovementState = MOVEMENT_STOPPED;
  currentSafetyState = SAFETY_NORMAL;

  WiFi.softAP(AP_SSID, AP_PASSWORD);
  IPAddress apIP = WiFi.softAPIP();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/forward", HTTP_GET, handleForward);
  server.on("/backward", HTTP_GET, handleBackward);
  server.on("/left", HTTP_GET, handleLeft);
  server.on("/right", HTTP_GET, handleRight);
  server.on("/stop", HTTP_GET, handleStop);
  server.on("/set_pwm", HTTP_GET, handleSetPWM);
  server.on("/telemetry", HTTP_GET, handleTelemetry);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/emergency_stop", HTTP_GET, handleEmergencyStop);
  server.on("/clear_emergency", HTTP_GET, handleClearEmergency);
  server.onNotFound(handleNotFound);

  server.begin();

  Serial.println("========================================");
  Serial.println("MINEGUARD ONBOARD VEHICLE SYSTEM");
  Serial.println("========================================");
  Serial.print("WiFi AP: ");
  Serial.println(AP_SSID);
  Serial.println("Controller URL:");
  Serial.print("http://");
  Serial.println(apIP);

  lastTelemetryUpdate = millis();
  lastSpeedCalcMs = millis();
}

// ==============================================================
// MAIN LOOP (non-blocking)
// ==============================================================
void loop() {
  server.handleClient();

  if (millis() - lastTelemetryUpdate >= SPEED_UPDATE_INTERVAL_MS) {
    updateTelemetry();
    lastTelemetryUpdate = millis();
  }

  updateDistanceSensor();   // Member 2: takes one HC-SR04 ping every ~50ms, never blocks
}

// ==============================================================
// SECTION 5: MOTOR / PWM FUNCTIONS
// ==============================================================
void setupPWM() {
  // Pin-based LEDC API (ESP32 Arduino Core 3.x) — no manual channel
  // bookkeeping required.
  ledcAttach(ENA, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttach(ENB, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}

// Central helper: applies the current throttlePWM to both motor
// channels. Direction pins are controlled separately by the
// movement functions below.
void applyThrottle() {
  ledcWrite(ENA, throttlePWM);
  ledcWrite(ENB, throttlePWM);
}

// ---- Defense-in-depth note ----
// Each movement function below re-checks currentSafetyState directly,
// in addition to the checks already performed in the HTTP handlers.
// This guarantees emergency stop cannot be bypassed even if these
// functions are ever invoked from somewhere other than the HTTP
// layer (e.g. a future internal caller). HTTP handler-level checks
// are left completely unchanged.
void moveForward() {
  if (currentSafetyState == SAFETY_EMERGENCY_STOP_ACTIVE) {
    stopMotors();
    currentMovementState = MOVEMENT_STOPPED;
    return;
  }
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  applyThrottle();
}

void moveBackward() {
  if (currentSafetyState == SAFETY_EMERGENCY_STOP_ACTIVE) {
    stopMotors();
    currentMovementState = MOVEMENT_STOPPED;
    return;
  }
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  applyThrottle();
}

void turnLeft() {
  if (currentSafetyState == SAFETY_EMERGENCY_STOP_ACTIVE) {
    stopMotors();
    currentMovementState = MOVEMENT_STOPPED;
    return;
  }
  // Left motor reverse, right motor forward -> pivot left
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  applyThrottle();
}

void turnRight() {
  if (currentSafetyState == SAFETY_EMERGENCY_STOP_ACTIVE) {
    stopMotors();
    currentMovementState = MOVEMENT_STOPPED;
    return;
  }
  // Right motor reverse, left motor forward -> pivot right
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  applyThrottle();
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}

// ==============================================================
// SECTION 6: ENCODER INTERRUPT
// ==============================================================
// LM393 module produces a clean digital pulse per encoder slot.
// RISING edge is used; the ISR performs no work beyond counting.
void IRAM_ATTR encoderISR() {
  encoderPulseCount++;
}

// ==============================================================
// SECTION 7: SPEED CALCULATION
// ==============================================================
void updateTelemetry() {
  // Atomically read and reset the interrupt-shared pulse counter.
  noInterrupts();
  unsigned long pulses = encoderPulseCount;
  encoderPulseCount = 0;
  interrupts();

  // Use the ACTUAL measured elapsed time rather than assuming the
  // nominal interval. loop() is non-blocking (server.handleClient()
  // can occasionally delay the next call slightly), so this keeps
  // RPM/speed accurate even if a cycle runs a little long or short.
  unsigned long nowMs = millis();
  unsigned long elapsedMs = nowMs - lastSpeedCalcMs;
  lastSpeedCalcMs = nowMs;

  // Guard against a zero/garbage interval (e.g. millis() rollover or
  // two calls landing on the same millisecond) to avoid a divide-by-zero.
  if (elapsedMs == 0) {
    elapsedMs = SPEED_UPDATE_INTERVAL_MS;
  }

  float elapsedMsF = (float)elapsedMs;

  currentRPM = (pulses / PULSES_PER_REVOLUTION) * (60000.0f / elapsedMsF);

  float wheelCircumferenceMeters = PI * WHEEL_DIAMETER_METERS;
  currentSpeedMpsRaw = (currentRPM * wheelCircumferenceMeters) / 60.0f;

  // ---- Low-pass filter (EMA) to smooth LM393 encoder noise ----
  // filtered = ALPHA*raw + (1-ALPHA)*filtered  (see SPEED_EMA_ALPHA above)
  currentSpeedMps = (SPEED_EMA_ALPHA * currentSpeedMpsRaw) +
                     ((1.0f - SPEED_EMA_ALPHA) * currentSpeedMps);

  // ---- Zero-speed handling ----
  // An EMA never reaches exactly zero on its own, it only decays
  // toward it. Track how many CONSECUTIVE telemetry cycles have seen
  // zero encoder pulses: a single zero-pulse cycle is not trusted on
  // its own (it could be a momentary/missed encoder read), but once
  // ZERO_SPEED_CONSECUTIVE_CYCLES in a row have seen no pulses, the
  // vehicle is confidently stopped and speed/RPM are forced to hard
  // zero immediately, instead of waiting for the EMA to decay below
  // ZERO_SPEED_EPSILON_MPS on its own.
  if (pulses == 0) {
    consecutiveZeroPulseCycles++;
  } else {
    consecutiveZeroPulseCycles = 0;
  }

  if (consecutiveZeroPulseCycles >= ZERO_SPEED_CONSECUTIVE_CYCLES) {
    currentSpeedMps = 0.0f;
    currentRPM = 0.0f;
  } else if (pulses == 0 && currentSpeedMps < ZERO_SPEED_EPSILON_MPS) {
    // Not yet at the consecutive-cycle threshold, but the filtered
    // value has already decayed to a negligible amount — snap it to
    // exactly zero rather than displaying a tiny fractional residue.
    currentSpeedMps = 0.0f;
  }

  currentSpeedKmh = currentSpeedMps * 3.6f;

  Serial.print("[TELEMETRY] Speed(raw): ");
  Serial.print(currentSpeedMpsRaw, 2);
  Serial.print(" m/s | Speed(filtered): ");
  Serial.print(currentSpeedMps, 2);
  Serial.print(" m/s | RPM: ");
  Serial.print(currentRPM, 1);
  Serial.print(" | elapsedMs: ");
  Serial.println(elapsedMs);
}

// ==============================================================
// SECTION 8/9: STATE HELPERS
// ==============================================================
const char *movementStateToString(MovementState state) {
  switch (state) {
    case MOVEMENT_STOPPED:   return "STOPPED";
    case MOVEMENT_FORWARD:   return "FORWARD";
    case MOVEMENT_BACKWARD:  return "BACKWARD";
    case MOVEMENT_LEFT:      return "LEFT";
    case MOVEMENT_RIGHT:     return "RIGHT";
    default:                 return "UNKNOWN";
  }
}

const char *safetyStateToString(SafetyState state) {
  switch (state) {
    case SAFETY_NORMAL:                  return "NORMAL";
    case SAFETY_EMERGENCY_STOP_ACTIVE:   return "EMERGENCY_STOP_ACTIVE";
    default:                             return "UNKNOWN";
  }
}

// ==============================================================
// SECTION 11: HTTP API HANDLERS
// ==============================================================

// ---- Dashboard ----
void handleRoot() {
  server.send_P(200, "text/html", DASHBOARD_HTML);
}

// ---- Shared response builder for movement endpoints ----
String buildMovementResponse(bool success, const char *errorReason) {
  String json = "{";
  json += "\"success\":" + String(success ? "true" : "false");
  if (success) {
    json += ",\"movement_state\":\"" + String(movementStateToString(currentMovementState)) + "\"";
    json += ",\"throttle_pwm\":" + String(throttlePWM);
  } else {
    json += ",\"error\":\"" + String(errorReason) + "\"";
  }
  json += "}";
  return json;
}

// ---- Movement commands (blocked during emergency stop) ----
void handleForward() {
  if (currentSafetyState == SAFETY_EMERGENCY_STOP_ACTIVE) {
    server.send(423, "application/json", buildMovementResponse(false, "EMERGENCY_STOP_ACTIVE"));
    return;
  }
  moveForward();
  currentMovementState = MOVEMENT_FORWARD;
  lastCommand = "MANUAL_FORWARD";
  Serial.println("[VEHICLE] Movement: FORWARD");
  server.send(200, "application/json", buildMovementResponse(true, nullptr));
}

void handleBackward() {
  if (currentSafetyState == SAFETY_EMERGENCY_STOP_ACTIVE) {
    server.send(423, "application/json", buildMovementResponse(false, "EMERGENCY_STOP_ACTIVE"));
    return;
  }
  moveBackward();
  currentMovementState = MOVEMENT_BACKWARD;
  lastCommand = "MANUAL_BACKWARD";
  Serial.println("[VEHICLE] Movement: BACKWARD");
  server.send(200, "application/json", buildMovementResponse(true, nullptr));
}

void handleLeft() {
  if (currentSafetyState == SAFETY_EMERGENCY_STOP_ACTIVE) {
    server.send(423, "application/json", buildMovementResponse(false, "EMERGENCY_STOP_ACTIVE"));
    return;
  }
  turnLeft();
  currentMovementState = MOVEMENT_LEFT;
  lastCommand = "MANUAL_LEFT";
  Serial.println("[VEHICLE] Movement: LEFT");
  server.send(200, "application/json", buildMovementResponse(true, nullptr));
}

void handleRight() {
  if (currentSafetyState == SAFETY_EMERGENCY_STOP_ACTIVE) {
    server.send(423, "application/json", buildMovementResponse(false, "EMERGENCY_STOP_ACTIVE"));
    return;
  }
  turnRight();
  currentMovementState = MOVEMENT_RIGHT;
  lastCommand = "MANUAL_RIGHT";
  Serial.println("[VEHICLE] Movement: RIGHT");
  server.send(200, "application/json", buildMovementResponse(true, nullptr));
}

void handleStop() {
  // STOP is always allowed, even during emergency (it cannot
  // re-enable movement by itself and simply keeps motors off).
  stopMotors();
  currentMovementState = MOVEMENT_STOPPED;
  lastCommand = "MANUAL_STOP";
  Serial.println("[VEHICLE] Movement: STOPPED");
  server.send(200, "application/json", buildMovementResponse(true, nullptr));
}

// ---- Throttle control ----
void handleSetPWM() {
  if (!server.hasArg("value")) {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"missing_value\"}");
    return;
  }

  String rawValue = server.arg("value");

  // Basic numeric validation: reject non-numeric input.
  bool isNumeric = rawValue.length() > 0;
  for (unsigned int i = 0; i < rawValue.length(); i++) {
    if (!isDigit(rawValue.charAt(i))) {
      isNumeric = false;
      break;
    }
  }

  if (!isNumeric) {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"invalid_number\"}");
    return;
  }

  int requestedPWM = rawValue.toInt();

  if (requestedPWM < 0 || requestedPWM > 255) {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"pwm_out_of_range\"}");
    return;
  }

  throttlePWM = requestedPWM;
  lastCommand = "PWM_UPDATED";

  // Apply immediately only if the vehicle is actively moving and not
  // under emergency stop. Direction pins are left untouched here;
  // this never causes the vehicle to start or stop moving.
  bool isMoving = (currentMovementState != MOVEMENT_STOPPED);
  if (isMoving && currentSafetyState == SAFETY_NORMAL) {
    applyThrottle();
  }

  Serial.print("[VEHICLE] Throttle PWM: ");
  Serial.println(throttlePWM);

  String response = "{\"success\":true,\"throttle_pwm\":" + String(throttlePWM) + "}";
  server.send(200, "application/json", response);
}

// ---- Telemetry ----
void handleTelemetry() {
  String json = "{";
  json += "\"movement_state\":\"" + String(movementStateToString(currentMovementState)) + "\",";
  json += "\"safety_state\":\"" + String(safetyStateToString(currentSafetyState)) + "\",";
  json += "\"last_command\":\"" + lastCommand + "\",";
  json += "\"speed_mps\":" + String(currentSpeedMps, 2) + ",";
  json += "\"speed_kmh\":" + String(currentSpeedKmh, 2) + ",";
  json += "\"rpm\":" + String(currentRPM, 1) + ",";
  json += "\"throttle_pwm\":" + String(throttlePWM) + ",";
  json += "\"speed_source\":\"" + String(SPEED_SOURCE_LABEL) + "\",";
  json += "\"distance_m\":" + String(getDistanceMeters(), 2) + ",";
  json += "\"distance_status\":\"" + String(distanceStatusToString(getDistanceStatus())) + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

// ---- Status ----
void handleStatus() {
  String json = "{";
  json += "\"movement_state\":\"" + String(movementStateToString(currentMovementState)) + "\",";
  json += "\"safety_state\":\"" + String(safetyStateToString(currentSafetyState)) + "\",";
  json += "\"last_command\":\"" + lastCommand + "\",";
  json += "\"throttle_pwm\":" + String(throttlePWM);
  json += "}";

  server.send(200, "application/json", json);
}

// ---- Emergency stop (highest priority) ----
void handleEmergencyStop() {
  currentSafetyState = SAFETY_EMERGENCY_STOP_ACTIVE;
  stopMotors();
  currentMovementState = MOVEMENT_STOPPED;
  lastCommand = "EMERGENCY_STOP";

  // NOTE: currentSpeedMps/currentRPM are intentionally NOT zeroed here.
  // Motors are cut immediately, but the physical wheels/dumper may still
  // coast briefly. The encoder-driven consecutive-zero-pulse logic in
  // updateTelemetry() is what determines when actual speed has reached
  // zero — telemetry should reflect real motion, not a faked value.
  Serial.println("[SAFETY] !!! EMERGENCY STOP ACTIVATED !!!");

  String json = "{";
  json += "\"success\":true,";
  json += "\"event\":\"EMERGENCY_STOP\",";
  json += "\"movement_state\":\"" + String(movementStateToString(currentMovementState)) + "\",";
  json += "\"safety_state\":\"" + String(safetyStateToString(currentSafetyState)) + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

// ---- Clear emergency ----
void handleClearEmergency() {
  currentSafetyState = SAFETY_NORMAL;
  stopMotors();
  currentMovementState = MOVEMENT_STOPPED;
  lastCommand = "EMERGENCY_CLEARED";

  Serial.println("[SAFETY] Emergency cleared. Vehicle remains stopped.");

  String json = "{";
  json += "\"success\":true,";
  json += "\"event\":\"EMERGENCY_CLEARED\",";
  json += "\"movement_state\":\"" + String(movementStateToString(currentMovementState)) + "\",";
  json += "\"safety_state\":\"" + String(safetyStateToString(currentSafetyState)) + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

// ---- 404 ----
void handleNotFound() {
  server.send(404, "application/json", "{\"success\":false,\"error\":\"not_found\"}