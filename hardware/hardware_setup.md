# MineGuard – Hardware Setup Guide

## 1. Overview
MineGuard is a prototype smart mine vehicle safety system. It uses a Main ESP32 (DevKit V1) for motor control and obstacle sensing, and a separate ESP32-CAM for live video streaming used in fog/low-visibility object detection (YOLO, run on a connected laptop). The vehicle is built on an **RC car chassis**, fitted with the sensors and controllers described below.

## 2. Components
| Component | Qty | Purpose |
|---|---|---|
| RC car chassis | 1 | Physical vehicle base |
| ESP32 DevKit V1 (Main) | 1 | Motor control, ultrasonic sensing, encoder reading, Wi-Fi AP host |
| AI Thinker ESP32-CAM (on ESP32-CAM-MB/HW-381) | 1 | Video streaming (Wi-Fi station) |
| L298N Motor Driver | 1 | Drives left/right DC motors |
| HC-SR04 Ultrasonic Sensor | 1 | Obstacle distance sensing |
| LM393 Encoder Module | 1 | Speed/distance measurement |
| DC Motors + Wheels | 2 (assumed) | Vehicle drive |
| 2× 18650 Li-ion Battery Pack (in series) | 1 pack | Motor supply (via L298N) |
| Power Bank (Main ESP32) | 1 | Powers Main ESP32 via USB |
| Power Bank (ESP32-CAM) | 1 | Powers ESP32-CAM independently |
| Laptop/PC | 1 | Runs dashboard.py, integration_controller.py, YOLO detection |
| Resistors: 1kΩ, 2.2kΩ | 1 each | HC-SR04 ECHO voltage divider |

## 3. Network Architecture
- Main ESP32 hosts a Wi-Fi Access Point (`WiFi.softAP`).
- ESP32-CAM joins that same network as a station/client (`WIFI_STA`) — it does not host its own AP.
- The laptop also joins the Main ESP32's AP.
- Communication between Main ESP32 and ESP32-CAM is wireless only — there is no electrical wire between the two boards.

```
Main ESP32 AP
      ↕ Wi-Fi
ESP32-CAM + Laptop
```

- AP IP: 192.168.4.1 is the default AP IP, but it is not guaranteed because the code uses `WiFi.softAPIP()` and does not explicitly configure the IP. **[TO CONFIRM]**
- ESP32-CAM IP: The ESP32-CAM gets its IP dynamically through DHCP from the Main ESP32 AP, so the IP may vary between boots/reconnections. **[TO CONFIRM]**

## 4. Pin Connections
See `pin_connections.md` for the full table. Summary:
- L298N: ENA=27, IN1=26, IN2=25 (left); ENB=14, IN3=33, IN4=32 (right); OUT1+OUT2 and OUT3+OUT4 to the two motor sides.
- HC-SR04: VCC→5V/VIN, GND→GND, TRIG=5, ECHO=18 through a 1kΩ/2.2kΩ voltage divider.
- LM393: VCC→3.3V, GND→GND, Signal→GPIO 34.
- Common ground shared across Main ESP32, L298N, HC-SR04, and LM393.

## 5. Power Architecture

```
2×18650 Li-ion pack (in series, 3.7V per cell — EASTAR ICR18650, confirmed from cell label)
        ↓
      L298N
        ↓
    DC Motors

Power bank
        ↓ USB
   Main ESP32
      ├── HC-SR04
      └── LM393

Separate power bank
        ↓
 ESP32-CAM-MB / HW-381
        ↓
 AI Thinker ESP32-CAM
```

- The 2× 18650 pack supplies only the L298N's motor output stage. Cells are **EASTAR ICR18650, 3.7V, 2000mAh** each (confirmed from the cell label), connected **in series**, giving a combined nominal pack voltage of approximately **7.4V**. Do NOT call it a 12V battery.
- Main ESP32 is USB-powered from its own power bank, and in turn powers HC-SR04 (5V/VIN) and LM393 (3.3V).
- ESP32-CAM is powered from a separate power bank, entirely independent of the Main ESP32's power supply.

## 6. Wi-Fi Configuration Before Flashing
1. Open `MineGuard_ESP32.ino`.
2. Replace the placeholder `AP_SSID` and `AP_PASSWORD` values with your chosen network name/password.
3. Open `esp32_cam_stream.ino` and set `WIFI_SSID`/`WIFI_PASSWORD` to match the Main ESP32's AP credentials exactly.
4. Flash both boards in this order: Main ESP32 first (so its AP is live), then ESP32-CAM.

## 7. Physical Assembly
1. Mount the ESP32 DevKit V1 and L298N motor driver onto the RC car chassis.
2. Attach/verify the motors and wheels are correctly connected to the L298N outputs (OUT1+OUT2 to one side, OUT3+OUT4 to the other).
3. Mount the HC-SR04 ultrasonic sensor at the front of the chassis, facing forward, with a clear unobstructed line of sight.
4. Mount the ESP32-CAM at the front of the chassis (near the HC-SR04), with a clear line of sight for video streaming.
5. Mount the LM393 encoder module near a wheel so it can read wheel rotation correctly.
6. Wire the L298N, HC-SR04, and LM393 to the Main ESP32 exactly per `pin_connections.md`.
7. Connect the 2× 18650 pack (in series) to the L298N motor supply input — verify polarity before powering on.
8. Connect the separate power bank to the ESP32-CAM.
9. Do a final visual check: confirm common ground is shared correctly across Main ESP32, L298N, HC-SR04, and LM393, and that the ESP32-CAM's power circuit is fully independent.

## 8. First Power-On / Verification
- [ ] Main ESP32 boots and creates its Wi-Fi AP (check on phone/laptop Wi-Fi list).
- [ ] ESP32-CAM connects to that AP as a client (check serial monitor for its DHCP-assigned IP).
- [ ] Laptop connects to the same AP.
- [ ] `/status` and `/telemetry` endpoints on Main ESP32 respond.
- [ ] `/stream` endpoint on ESP32-CAM shows live video.
- [ ] Ultrasonic sensor readings look sane at known distances (e.g., 10 cm, 50 cm, 1 m).
- [ ] Encoder pulses register correctly when wheels turn.

## 9. Known Limitations
See README Section 12 (ultrasonic range, fog/detection accuracy, day/night testing, single-vehicle-only notes).

## 10. Troubleshooting
| Symptom | Likely Cause |
|---|---|
| ESP32-CAM won't connect to AP | SSID/password mismatch between the two `.ino` files |
| No `/telemetry` data | LM393 wiring or GPIO 34 issue (input-only pin, check for pull resistor needs) |
| Ultrasonic gives erratic readings | Check the 1kΩ/2.2kΩ divider connections on ECHO |
| Motors don't respond but Wi-Fi works | 18650 pack disconnected/low, or `/set_pwm` values too low |
| Motors run but weak/inconsistent | Check 18650 pack charge level and its connection to the L298N |
