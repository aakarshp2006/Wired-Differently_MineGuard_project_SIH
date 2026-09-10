# MineGuard – Pin Connections

> Confirmed from code (`MineGuard_ESP32.ino`, `esp32_cam_stream.ino`, `distance_sensor.cpp/h`) and physical hardware confirmation.
> Only two items remain unresolved (marked **[TO CONFIRM]**) — see bottom of this document.

## Main ESP32 (DevKit V1) — Motor Control + Ultrasonic + Encoder + Wi-Fi AP

**Board power:** USB, from a power bank.

| Component | Pin / Function | ESP32 GPIO | Power | Ground | Notes |
|---|---|---|---|---|---|
| L298N Motor Driver | ENA (left, PWM enable) | GPIO 27 | — | — | Left motor speed control |
| L298N Motor Driver | IN1 (left, direction) | GPIO 26 | — | — | Paired with IN2 |
| L298N Motor Driver | IN2 (left, direction) | GPIO 25 | — | — | Paired with IN1 |
| L298N Motor Driver | ENB (right, PWM enable) | GPIO 14 | — | — | Right motor speed control |
| L298N Motor Driver | IN3 (right, direction) | GPIO 33 | — | — | Paired with IN4 |
| L298N Motor Driver | IN4 (right, direction) | GPIO 32 | — | — | Paired with IN3 |
| L298N Motor Driver | OUT1 + OUT2 | — | 2× 18650 Li-ion pack (series) | Common ground | Drives one motor side |
| L298N Motor Driver | OUT3 + OUT4 | — | 2× 18650 Li-ion pack (series) | Common ground | Drives other motor side |
| HC-SR04 Ultrasonic Sensor | VCC | — | ESP32 5V/VIN | ESP32 GND | |
| HC-SR04 Ultrasonic Sensor | GND | — | — | ESP32 GND | |
| HC-SR04 Ultrasonic Sensor | TRIG | GPIO 5 | — | — | Sends trigger pulse |
| HC-SR04 Ultrasonic Sensor | ECHO | GPIO 18 | — | — | Through 1kΩ/2.2kΩ voltage divider (below) — protects ESP32's 3.3V-only GPIO from the sensor's 5V ECHO output |
| LM393 Encoder Module | VCC | — | ESP32 3.3V | — | |
| LM393 Encoder Module | GND | — | — | ESP32 GND | |
| LM393 Encoder Module | Signal | GPIO 34 | — | — | Input-only pin; PULSES_PER_REVOLUTION = 20, WHEEL_DIAMETER = 0.065 m used for speed calc |

**HC-SR04 ECHO voltage divider (confirmed):**
```
HC-SR04 ECHO
     │
    1kΩ
     │
     ├──────→ GPIO 18
     │
   2.2kΩ
     │
    GND
```

**Common ground:** Main ESP32, L298N, HC-SR04, and LM393 all use the required common ground on the vehicle-side electronics.

**Power sources on the vehicle side:**
- Main ESP32: USB power from a power bank.
- L298N motor supply: 2× 18650 Li-ion battery pack (EASTAR ICR18650, 3.7V, 2000mAh each — confirmed from cell label), connected **in series**, combined nominal pack voltage ≈ 7.4V.
- HC-SR04 and LM393: powered from the Main ESP32 (5V/VIN and 3.3V rails respectively).

## ESP32-CAM (AI Thinker, on ESP32-CAM-MB / HW-381) — Camera Streaming

| Component | Power | Ground | Notes |
|---|---|---|---|
| ESP32-CAM (AI Thinker, on ESP32-CAM-MB/HW-381) | Separate power bank (independent from Main ESP32's power) | Own board ground (not part of the vehicle-side common ground) | Communicates with the Main ESP32 wirelessly, as a Wi-Fi station/client — no electrical wire runs between ESP32-CAM and Main ESP32 |

## Network Section (wireless — not a physical/electrical connection)

| Device | Wi-Fi Role | Port | Notes |
|---|---|---|---|
| Main ESP32 | Access Point (AP) | 80 | Endpoints: `/`, `/forward`, `/backward`, `/left`, `/right`, `/stop`, `/set_pwm`, `/telemetry`, `/status`, `/emergency_stop`, `/clear_emergency` |
| ESP32-CAM | Station (client) | 80 (HTTP), 32768 (ctrl_port) | Endpoints: `/`, `/capture`, `/stream`. Connects to the Main ESP32's AP; does not host its own AP. |
| Laptop / Dashboard PC | Station (client) | — | Connects to the Main ESP32's AP to reach both devices |

**AP IP:** 192.168.4.1 is the default AP IP, but it is not guaranteed because the code uses `WiFi.softAPIP()` and does not explicitly configure the IP.

## Remaining Unresolved Items — [TO CONFIRM]

1. **Main ESP32 AP exact IP** — 192.168.4.1 is the default AP IP, but it is not guaranteed because the code uses `WiFi.softAPIP()` and does not explicitly configure the IP.
2. **ESP32-CAM exact IP** — The ESP32-CAM gets its IP dynamically through DHCP from the Main ESP32 AP, so the IP may vary between boots/reconnections.

*(Battery pack: EASTAR ICR18650, 3.7V, 2000mAh × 2 cells in series, combined ≈ 7.4V — confirmed from the physical cell label.)*
