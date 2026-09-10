/*
 * MineGuard — Member 2's distance sensor logic (NON-BLOCKING VERSION)
 * Hardware: HC-SR04 Ultrasonic Sensor
 *
 * WHY THIS FILE IS DIFFERENT FROM THE ORIGINAL ultrasonic_sensor.ino:
 * -----------------------------------------------------------------
 * The original code called delay(50) five times in a row (once
 * between each of its 5 samples) plus a further delay(50) at the
 * end of every loop -- about 250ms of the chip doing NOTHING else,
 * every single cycle.
 *
 * On its own that was fine. But now this ESP32 ALSO runs Member 1's
 * motor web server in the same loop(). If we kept those delay()
 * calls, the web server would freeze for 250ms at a time, and any
 * browser button press (forward/stop/etc) could be delayed or even
 * dropped during that window.
 *
 * The fix: instead of "take 5 samples right now, one every 50ms,
 * blocking the whole time," we take ONE sample every time
 * updateDistanceSensor() is called from loop(), but only act on it
 * if at least 50ms have passed since the last sample. Once we've
 * collected 5 samples this way (spread out naturally as loop() keeps
 * running), we compute the median exactly like the original code did.
 *
 * The math (median filtering, valid range, OK/WARNING/ERROR status)
 * is UNCHANGED from the original -- only the timing/scheduling changed.
 */

#include "distance_sensor.h"

// ---- Pins (same as the original Member 2 code) ----
static const int TRIG_PIN = 5;
static const int ECHO_PIN = 18;

// ---- Timing ----
static const unsigned long TRIGGER_PULSE_US   = 10;
static const unsigned long ECHO_TIMEOUT_US    = 30000; // ~5m max range timeout
static const unsigned long SAMPLE_INTERVAL_MS = 50;    // gap between individual pings

// ---- Range / filtering constants (same as the original) ----
static const float SOUND_SPEED_CM_PER_US       = 0.0343f / 2.0f;
static const float MIN_VALID_DISTANCE_CM       = 2.0f;
static const float MAX_VALID_DISTANCE_CM       = 400.0f;
static const int   NUM_SAMPLES                 = 5;
static const int   MIN_VALID_SAMPLES_FOR_OK    = 5;
static const int   MIN_VALID_SAMPLES_FOR_WARNING = 3;
static const float INVALID_DISTANCE            = -1.0f;

// ---- State for the "collect samples over time" state machine ----
static float sampleBuffer[NUM_SAMPLES];
static int samplesCollected = 0;
static unsigned long lastSampleMs = 0;

// ---- Most recent published result ----
static float latestDistanceMeters = 0.0f;
static SensorStatus latestStatus = STATUS_ERROR;

// Sends one trigger pulse and measures the echo.
// NOTE: pulseIn() here can still block for up to ECHO_TIMEOUT_US
// (30ms worst case, only when nothing reflects the ping back). That
// is a MUCH smaller and rarer block than the old 250ms/cycle, so it
// is left as-is -- this is a normal, accepted tradeoff for HC-SR04.
static float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(TRIGGER_PULSE_US);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long durationUS = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);

  if (durationUS == 0) {
    return INVALID_DISTANCE; // no echo -- timeout
  }

  float distanceCM = durationUS * SOUND_SPEED_CM_PER_US;

  if (distanceCM < MIN_VALID_DISTANCE_CM || distanceCM > MAX_VALID_DISTANCE_CM) {
    return INVALID_DISTANCE; // outside HC-SR04's reliable range
  }

  return distanceCM;
}

static void sortArray(float arr[], int size) {
  for (int i = 1; i < size; i++) {
    float key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
}

// Called once we've collected NUM_SAMPLES readings: filters out
// invalid ones, takes the median of what's left, and updates the
// status (OK / WARNING / ERROR) -- same logic as the original code.
static void finishBatch() {
  float validSamples[NUM_SAMPLES];
  int validCount = 0;

  for (int i = 0; i < samplesCollected; i++) {
    if (sampleBuffer[i] != INVALID_DISTANCE) {
      validSamples[validCount] = sampleBuffer[i];
      validCount++;
    }
  }

  if (validCount < MIN_VALID_SAMPLES_FOR_WARNING) {
    // Not enough good readings this batch -- report ERROR, but keep
    // the last known good distance rather than overwriting it with
    // garbage. The laptop should check distance_status, not just
    // distance_m, before trusting the number.
    latestStatus = STATUS_ERROR;
  } else {
    sortArray(validSamples, validCount);

    float medianCM;
    if (validCount % 2 == 1) {
      medianCM = validSamples[validCount / 2];
    } else {
      int mid = validCount / 2;
      medianCM = (validSamples[mid - 1] + validSamples[mid]) / 2.0f;
    }

    latestDistanceMeters = medianCM / 100.0f;
    latestStatus = (validCount >= MIN_VALID_SAMPLES_FOR_OK) ? STATUS_OK : STATUS_WARNING;
  }

  samplesCollected = 0; // start collecting the next batch
}

void beginDistanceSensor() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  lastSampleMs = millis();
}

void updateDistanceSensor() {
  unsigned long now = millis();

  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) {
    return; // not time for the next ping yet -- do nothing, don't block
  }
  lastSampleMs = now;

  sampleBuffer[samplesCollected] = readDistanceCM();
  samplesCollected++;

  if (samplesCollected >= NUM_SAMPLES) {
    finishBatch();
  }
}

float getDistanceMeters() {
  return latestDistanceMeters;
}

SensorStatus getDistanceStatus() {
  return latestStatus;
}

const char *distanceStatusToString(SensorStatus status) {
  switch (status) {
    case STATUS_OK:      return "OK";
    case STATUS_WARNING: return "WARNING";
    case STATUS_ERROR:   return "ERROR";
    default:              return "UNKNOWN";
  }
}
