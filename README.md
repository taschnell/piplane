# PiPlane/PiDrone:
A DIY FPV *DRONE/PLANE* with Raspberry Pi, ELRS/CRSF, Video, & Autopilot Features

A Personal Project to create a simplified FC software for a Raspberry Pi. Inspired by INAV and Betaflight

## Hardware Requirements

- **Radio Protocol:** ELRS & CRSF (Crossfire) for low-latency, reliable communication
  - Bandit BR3 915/868MHz Dual High Sensitivity T Antenna ELRS Receiver
- **Flight Controller:** Raspberry Pi 5 FC
  - Raspberry Pi Pico, Motor Control
- **Motors:** DSHOT600 ESC and Motor System
  - Speedybee 50A F4 128K BLHeli_32
  - Hyperlite 2807.5, 1122KV
- **VTX:** TBD
- **Sensors:** 
  - GPS: GEP-10 DQ
    - UBLOX
      - M10 Chip
    - 12C
      - *QMC5883L* | MAG
      - *DPS310* | BAR
  - IMU | 12C: BO085
    - *Accelerometer*, *Gyroscope* 
    - Mag (disabled)

## Software Requirements
- Ubuntu 24.04LTS
- ROS2 Jazzy
- MicroROS
- Adafruit Libaries
