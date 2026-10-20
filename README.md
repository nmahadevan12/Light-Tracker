# 2-Axis Light Tracker

Solo **ELEE 4230** project: a battery-powered dual-axis light tracker on an **ATmega328P**. Bare-metal C drives **DC motors** (not servos) through MOSFET H-bridges, with LDR sensing, fixed-point Kalman filtering, Hall-effect tilt limits, and UART telemetry to a Python logger.

## Demo

[YouTube short](https://www.youtube.com/shorts/PQEl34fqnAk)

## Features

- **Kalman on ADC:** smooths LDR, potentiometer, and battery channels (peak noise cut from ~3 LSBs to under 1 LSB on the portfolio write-up measurements).
- **Closed-loop DC motor control:** base and panel axes via H-bridge direction outputs from thresholded LDR differentials.
- **Auto-calibration:** startup sweep into Hall-effect limits to learn pot min/max.
- **Hardware limit protection:** Hall sensors + latched software states stop over-rotation.
- **UART telemetry:** register-level UART (9.216 MHz crystal) streaming status to Python logging scripts — local serial logging, not a cloud “IoT” stack.

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
