# MineGuard – Testing

This document records what has been demonstrated and what has not yet been formally, quantitatively tested. Where a measurement or success rate has not been recorded, it is explicitly marked as such rather than estimated or assumed.

## 1. What "demonstrated" means here

Several parts of MineGuard have been run and shown working (see [`assets/screenshots/`](../assets/screenshots/) for evidence) — this confirms the feature **functions**. It does not, by itself, establish a measured **accuracy**, **range**, or **success rate**. Those are tracked separately below.

## 2. Distance sensing (HC-SR04)

- **Status:** Hardware tested and debugged; sensor produces live distance readings, shown on the dashboard (e.g. `ss_dashboard_safe.png`, `ss_dashboard_warning.png`, `ss_dashboard_danger.png` in `assets/screenshots/`).
- **Quantitative accuracy at known distances (10 cm / 50 cm / 1 m, etc.):** `[NOT YET MEASURED]`
- No formal accuracy testing against a reference measurement (e.g. tape measure) has been documented yet.

## 3. Speed measurement (LM393 encoder)

- **Status:** Encoder is wired and integrated; the dashboard displays live speed/RPM values (see `ss_telemetry.png`).
- **Quantitative accuracy vs. a known/reference speed:** `[NOT YET QUANTITATIVELY VERIFIED]`
- No formal comparison against a reference speed measurement has been documented yet.

## 4. Vision / YOLO object detection

- **Status:** YOLO object detection is implemented and demonstrated — see `ss_yolo_detection.png`, which shows a live detection with a bounding box, object label, and confidence score.
- **Reliable detection range:** `[NOT YET QUANTITATIVELY MEASURED]`
- No formal range testing (e.g. detection success rate at increasing distances) has been documented yet. The screenshot demonstrates that detection works at the distance shown at the time it was captured — it should not be read as a general range claim.

## 5. Emergency stop

- **Status:** Emergency-stop behavior is implemented in code (`MineGuard_ESP32.ino` / `/emergency_stop` and `/clear_emergency` endpoints — see `pin_connections.md`) and triggers when the risk engine reaches a critical decision (see `ss_dashboard_danger.png`, showing the "CRITICAL — Stop now" state).
- **Number of test runs / success rate:** `[TEST COUNT NOT RECORDED]`
- No formal test log (number of trials, pass/fail count) has been documented yet.

## 6. Fog / low-visibility detection

- **Status:** Fog detection is implemented and produces a live visibility percentage and classification on the dashboard (see `ss_yolo_detection.png`, showing `Fog: 53.7% (MODERATE_FOG)`).
- **Formal real-fog testing has not yet been conducted.**
- No claim is made here about testing under actual fog conditions or a controlled obstruction-based simulation, since neither has been confirmed as formally performed.

## 7. Summary table

| Component | Functionally demonstrated | Quantitative test result |
|---|---|---|
| Distance sensing (HC-SR04) | Yes | `[NOT YET MEASURED]` |
| Speed sensing (LM393) | Yes | `[NOT YET QUANTITATIVELY VERIFIED]` |
| YOLO object detection | Yes | `[NOT YET QUANTITATIVELY MEASURED]` |
| Emergency stop | Yes (behavior implemented, triggers on critical risk) | `[TEST COUNT NOT RECORDED]` |
| Fog detection | Yes (live output implemented) | Formal real-fog testing not yet conducted |

## 8. Planned formal testing (before final submission, if time permits)

- [ ] Measure HC-SR04 readings against a tape measure at fixed distances (e.g. 10 cm, 50 cm, 1 m, 2 m) and record error.
- [ ] Compare encoder-derived speed against a manually timed reference speed over a known distance.
- [ ] Record YOLO detection success/failure at a set of fixed distances to establish a reliable detection range.
- [ ] Run a documented number of emergency-stop trials and record the success count.
- [ ] If possible, test fog detection under an actual or simulated low-visibility condition and record the result.
