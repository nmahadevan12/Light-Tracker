# Dual-Axis Solar Tracker & Motor Control System

An embedded AVR C application for automated solar tracking using LDR sensor data, Kalman filtering, and Hall-effect safety limits, paired with IoT telemetry logging and custom PCB hardware designs.

## 📹 Video Demo

Watch the project in action on YouTube:

[![Project Demo](https://img.youtube.com/vi/PQEl34fqnAk/0.jpg)](https://www.youtube.com/shorts/PQEl34fqnAk)

*(Click the image above or [click here](https://www.youtube.com/shorts/PQEl34fqnAk) to watch the video)*

---

## 📌 Features

* **Real-time Kalman Filtering:** Smooths noise on raw ADC readings from dual LDRs, potentiometers, and battery monitoring.
* **Closed-loop Motor Control:** Drives base and panel DC motors via directional output signals based on thresholded LDR differentials.
* **Auto-Calibration Routine:** Performs startup sweep using Hall-effect limit switches to establish min/max potentiometer ADC limits.
* **Hardware Limit Protection:** Uses Hall-effect sensors and latched software states to prevent motor over-rotation.
* **IoT Telemetry:** UART serial communication interface for real-time status updates, logged via Python scripts.

---

## 📁 Repository Structure

```text
.
├── .vscode/                     # VS Code workspace tasks and setup
├── ATmega328P Datasheet/        # Hardware datasheet reference files
├── Images/                      # Project photos and KiCad screenshots
├── Kicad Files/                 # KiCad schematics and PCB layout designs
├── src/                         # Application source code
│   ├── IoT Logging/             # Python scripts for UART data logging
│   │   └── main.py
│   └── Source Code/             # AVR C firmware
│       └── main.c
├── .gitignore                   # Files excluded from Git tracking
└── README.md                    # Project documentation