# 2-Axis Light Tracker

Solo **ELEE 4230** project: a battery-powered dual-axis light tracker on an **ATmega328P**. Bare-metal C drives **DC motors** (not servos) through MOSFET H-bridges, with LDR sensing, fixed-point Kalman filtering, Hall-effect tilt limits, and UART telemetry to a Python logger.

## Demo

[YouTube short](https://www.youtube.com/shorts/PQEl34fqnAk)

## Features

- **Bare-metal dual-axis drive:** ATmega328P C firmware driving motors with MOSFET H-bridges.
- **Tilt safety:** LDR array with Hall-effect and potentiometer feedback to keep tilt within safe limits.
- **Fixed-point Kalman + UART:** cuts ADC noise from 3 LSBs to < 1 LSB without an FPU; streams results over a register-level 115.2 kbps UART driver.
- **4-layer KiCad PCB:** separate power, ground, and signal layers; widened power traces to reduce voltage drop.

## Repo layout

```text
.
├── ATmega328P Datasheet/
├── Images/
├── Kicad Files/          # Schematics + PCB
├── src/
│   ├── IoT Logging/      # Python UART logger (folder name is historical)
│   └── Source Code/      # AVR C firmware
└── README.md
```

## More detail

Design notes, gallery, and interactive PCB: [nmahadevan12.github.io/projects/light-tracker/](https://nmahadevan12.github.io/projects/light-tracker/)
