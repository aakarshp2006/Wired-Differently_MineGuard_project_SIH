# MineGuard – Smart Mine Vehicle Safety System

**Smart India Hackathon 2026**

| | |
|---|---|
| **PS ID** | 26007 |
| **PS Title** | Safe and Efficient Operation of Mine Vehicles in Fog and Low-Visibility Conditions in Open Cast Iron Ore Mines |
| **Team Name** | Wired Differently |
| **College** | NSUT |

---

## 1. What is MineGuard?

MineGuard is a smart safety-assist system for vehicles operating in open cast iron ore mines, built specifically to handle **fog and low-visibility conditions**. It combines ultrasonic distance sensing, AI-based vision (YOLO object detection), and a real-time risk-scoring engine to warn the driver of danger and trigger an emergency stop when conditions become critical — without taking away the driver's normal control of the vehicle.

## 2. The Problem

NMDC Limited is India's largest iron ore producer, operating fully mechanized mining complexes including the Bailadila region, which alone contributes nearly 37 MTPA of iron ore production.

The Bailadila mining region experiences severe monsoon conditions from June to October — heavy rainfall, strong winds, high humidity, dense clouds, and thick fog. During this period, visibility on mine haul roads, particularly in hilltop mining areas, often reduces to as low as **3–5 metres**. This significantly affects the safe and efficient movement of Heavy Earth Moving Machinery (HEMM), especially dumpers engaged in ore transportation.

Dense fog and extremely low visibility during the monsoon season create major operational and safety challenges:
- Poor visibility forces operators to reduce speed or halt operations, increasing haul-cycle times and reducing fleet productivity.
- The risk of vehicle collisions, road accidents, and operational disruptions rises substantially.
- Existing visibility aids and operational controls have limited effectiveness in dense fog.

There is a need for an intelligent, technology-driven solution that enables safe and efficient movement of mine vehicles under low-visibility conditions, while maintaining production continuity.

## 3. Our Proposed Solution (Prototype)

MineGuard assists the driver with real-time hazard detection and risk-based recommendations:

- A vehicle-mounted **ESP32 controller** measures distance (ultrasonic) and vehicle speed (encoder), and manages manual motor control.
- An **ESP32-CAM** streams live video of the surroundings.
- A **laptop-side AI system** runs YOLO object detection on that video feed to identify people, vehicles, and obstacles, and also detects fog/low-visibility conditions.
- A **risk engine** combines object type, distance, speed, and time-to-collision (TTC) to calculate a risk score and recommend **MOVE / SLOW / STOP**, along with a recommended speed.
- The driver stays in control of normal driving at all times. The system does **not** automatically control regular throttle. It only recommends action, and triggers an **automatic emergency stop** if the situation becomes critical.
- A **live dashboard** displays all this information so the safety status is always visible.

This prototype demonstrates the core safety logic on affordable hardware. See [Section 13 — Industrial Vision](#13-industrial-vision--scalability) for how this scales to a full mine-wide deployment.

## 4. Key Features

- Real-time distance measurement (ultrasonic)
- Real-time speed measurement (encoder-based)
- Live video streaming from ESP32-CAM
- YOLO-based object detection (person / vehicle / obstacle)
- Fog / low-visibility detection
- Risk scoring using distance, speed, and time-to-collision (TTC)
- SAFE / WARNING / DANGER risk states
- MOVE / SLOW / STOP recommendations with suggested speed
- Automatic emergency stop on critical risk (hard safety override)
- Live safety dashboard for driver/operator visibility

## 5. System Architecture (Prototype)

```text
                     ┌─────────────────────┐
                     │   ESP32-CAM          │
                     │   (video stream)      │
                     └──────────┬───────────┘
                                │ Wi-Fi (video stream)
                                ▼
┌──────────────────┐   Wi-Fi    ┌─────────────────────────────┐
│  Main ESP32       │◄─────────►│        LAPTOP (Python)        │
│  - Distance (HC-SR04)         │  ai_detector.py (YOLO)        │
│  - Speed (LM393 encoder)      │  fog_detection.py             │
│  - Motor control (L298N)      │  risk_engine.py               │
│  - Emergency stop              │  integration_controller.py    │
└──────────────────┘            │  read_telemetry.py            │
                                 │  dashboard.py                 │
                                 └─────────────────────────────┘
```

- **Distance + Speed** → Main ESP32 → sent to laptop over Wi-Fi
- **Video** → ESP32-CAM → sent to laptop over Wi-Fi
- **Laptop** runs YOLO + fog detection + risk engine → combines everything in `integration_controller.py`
- **Dashboard** shows the live safety state to the operator
- The driver always controls normal vehicle speed manually. Emergency stop is automatic only when a hard safety condition is met.

*(A more detailed hardware wiring diagram and data-flow diagram are available in [`hardware/`](hardware/) and [`docs/architecture.md`](docs/architecture.md).)*

## 6. Hardware

| Component | Role |
|---|---|
| ESP32 DevKit V1 | Main vehicle controller |
| L298N Motor Driver | Drives the DC motors |
| DC Motors | Vehicle movement |
| LM393 Speed Encoder | Measures wheel speed |
| HC-SR04 Ultrasonic Sensor | Measures obstacle distance |
| AI Thinker ESP32-CAM (OV2640) | Captures live video |
| ESP32-CAM-MB / HW-381 | USB programmer for ESP32-CAM |

## 7. Software / Technology Stack

| Layer | Technology |
|---|---|
| Vehicle firmware | Arduino (C++), ESP32 |
| Camera firmware | Arduino (C++), ESP32-CAM |
| AI vision | Python, OpenCV, Ultralytics YOLO (yolo11n.pt) |
| Risk logic | Python |
| Dashboard | Python |
| Communication | Wi-Fi |

## 8. Repository Structure

```text
MineGuard_project_SIH/
├── README.md
├── LICENSE
├── .gitignore
│
├── MineGuard_ESP32/          # Main vehicle controller firmware
│   ├── MineGuard_ESP32.ino
│   ├── distance_sensor.cpp
│   └── distance_sensor.h
│
├── esp32_cam_stream/          # ESP32-CAM video streaming firmware
│   └── esp32_cam_stream.ino
│
├── MineGuard_laptop/          # Laptop-side AI + safety software
│   ├── ai_detector.py
│   ├── dashboard.py
│   ├── fog_detection.py
│   ├── integration_controller.py
│   ├── read_telemetry.py
│   ├── risk_engine.py
│   ├── requirements.txt
│   └── yolo11n.pt
│
├── hardware/                  # Wiring + circuit documentation
│   ├── circuit_diagram/
│   ├── hardware_setup.md
│   └── pin_connections.md
│
├── docs/                      # Technical documentation
│   ├── architecture.md
│   ├── integration.md
│   └── testing.md
│
├── assets/screenshots/        # Evidence: photos + screenshots
│
└── submission/                # PPT, demo video link, etc.
    |── PRESENTATION.md
```

## 9. Installation & Setup

### 9.1 Main ESP32 (vehicle controller)
1. Open `MineGuard_ESP32/MineGuard_ESP32.ino` in Arduino IDE.
2. Set your own Wi-Fi Access Point name/password in place of the placeholders.
3. Select the correct ESP32 board and COM port.
4. Upload the code.

### 9.2 ESP32-CAM (video stream)
1. Open `esp32_cam_stream/esp32_cam_stream.ino` in Arduino IDE.
2. Set your own Wi-Fi Access Point name/password in place of the placeholders.
3. Connect ESP32-CAM using the HW-381/ESP32-CAM-MB programmer.
4. Upload the code.

### 9.3 Laptop software
```bash
cd MineGuard_laptop
pip install -r requirements.txt
```

## 10. Running the System

1. Power on the Main ESP32 and ESP32-CAM. They will start their own Wi-Fi Access Point(s).
2. Connect your laptop to the ESP32's Wi-Fi network.
3. Run the integration controller:
   ```bash
   python integration_controller.py
   ```
4. Run the dashboard:
   ```bash
   python dashboard.py
   ```
5. The dashboard will display live distance, speed, YOLO detections, risk state (SAFE/WARNING/DANGER), and recommended action (MOVE/SLOW/STOP).

*(Exact IP addresses / connection steps are documented in [`hardware/hardware_setup.md`](hardware/hardware_setup.md).)*

## 11. Testing

See [`docs/testing.md`](docs/testing.md) for details on what was tested and the results observed.

## 12. Limitations

This is a proof-of-concept prototype, and the following limitations are known:

- **Distance sensing:** The HC-SR04 has a rated range of approximately 2 cm–4 m under suitable conditions, but the full range is not guaranteed in all conditions. Readings can be affected by object surface, angle, and environmental factors.
- **Fog/low-visibility detection:** Detection performance may degrade under dense fog, dust and very low visibility. Quantitative degradation has not yet been measured.
- **Lighting conditions:** The prototype has primarily been tested under normal/daylight conditions. Night-time and very low-light performance have not been fully validated.
- **Single-vehicle scope:** This is a single-vehicle proof of concept. It does not currently implement multi-vehicle coordination, vehicle-to-vehicle communication, or fleet-level collision avoidance.
- **Wi-Fi dependency:** Communication between the ESP32/ESP32-CAM and the laptop relies on local Wi-Fi. Performance depends on network availability, range, congestion, and latency; no formal range or latency characterization has been done.
- **Compute dependency:** The current prototype requires a laptop for YOLO/AI processing.
- **Detection accuracy:** Camera-based detection can produce false positives or false negatives; this has not been formally quantified.
- **Hardware grade:** Prototype hardware (HC-SR04, ESP32-CAM, consumer battery) is not industrial-grade or safety-certified.
- **Real-world validation:** The prototype has not been extensively tested under real mine conditions — heavy dust, dense fog, rain, strong vibration, uneven terrain, and night operation.
- **Battery/runtime and end-to-end latency** have not been formally characterized.

## 13. Industrial Vision & Scalability

The prototype validates the **core safety logic** — sensing, risk calculation, warning, and emergency stop. The long-term vision is to scale this into a complete mine-wide safety platform:

| Prototype (built) | Industrial version (future) |
|---|---|
| HC-SR04 ultrasonic sensor | Radar / LiDAR-based ranging |
| ESP32-CAM | Industrial-grade / thermal cameras |
| Single vehicle, manual driving | Fleet-wide vehicle-to-vehicle (V2V) awareness |
| Laptop as edge compute | Industrial edge computer + safety-rated controller |
| Local dashboard | Central mine safety control room dashboard |
| No location awareness | GNSS/DGPS-based tracking + geo-fencing of mine zones |
| Manual review | Event logging + near-miss analytics for preventive safety |

**Planned scaling stages:** single-vehicle safety assist → multi-sensor fusion → multi-vehicle awareness → worker (pedestrian) safety integration → mine-wide central monitoring → predictive safety analytics.

This roadmap directly addresses the full scope of the NMDC problem statement (Section 2), while the current repository demonstrates a working, testable proof-of-concept of its core decision-making logic.

## 14. Future Scope

The immediate next phase focuses on **improving and hardening the existing prototype**, before scaling toward the industrial vision in Section 13:

**Sensing & Detection**
- Upgrade distance sensing beyond HC-SR04 (better ultrasonic, ToF, or short-range LiDAR) for improved reliability and range
- Improve fog/visibility detection (contrast-based estimation, testing across varying fog/dust/haze densities)
- Add multiple camera angles (front/rear/side) to reduce blind spots
- Extend to low-light/night operation (IR-capable camera, low-light image enhancement)
- Add object tracking across frames to estimate approach speed and improve collision prediction

**System & Reliability**
- Move AI processing from laptop to a compact edge device (Raspberry Pi- or Jetson-class) for a more portable prototype
- Strengthen Wi-Fi communication (auto-reconnect, timestamped data, stale-data handling, latency measurement)
- Extend the risk model with object movement, relative velocity, and direction of travel
- Add a physical driver alert system (buzzer, warning LEDs, directional alerts) alongside the dashboard

**Testing & Validation**
- Structured data logging (timestamp, distance, speed, TTC, risk score, action) for performance analysis
- Automated, repeatable test scenarios (SAFE/WARNING/DANGER, detection classes, emergency stop)
- Quantitative performance evaluation (detection accuracy, false positive/negative rate, latency, e-stop response time)
- Field validation in a controlled, authorized mining-like environment (dust, uneven terrain, vibration, realistic distances)

**Goal:** make the prototype more accurate, reliable, portable, and quantitatively testable — before moving toward the industrial-grade hardware described in Section 13.

## 15. Team — Wired Differently

| Name | Role |
|---|---|
| Aakarsh Pandey | Team Lead / Vehicle & ESP32 Integration |
| Ekasdeep Kaur | Distance Sensing / Hardware |
| Aditya Thakor | Computer Vision / ESP32-CAM |
| Karan Kumar | Risk Engine / Safety Logic |
| Pranav Bhasin | System Integration / Communication |
| Asya Gupta | Dashboard / Documentation & Presentation |

## 16. Presentation

- Presentation: see [`submission/PRESENTATION.md`](submission/PRESENTATION.md)

## 17. Important Note

This repository does not contain any passwords, API keys, or private credentials. Wi-Fi credentials in the firmware are placeholders — replace them with your own before flashing the hardware.
