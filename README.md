# PiPlane:
A DIY FPV Plane with Raspberry Pi, ELRS/CRSF, MSD Video, IC12 Servo Control, Limited Autopilot Features

## Project Overview

A Personal Project to create a simplified FC software for a Raspberry Pi. Inspired by INAV and Betaflight.

- **Radio Protocol:** ELRS or CRSF (Crossfire) for low-latency, reliable communication
- **Flight Controller:** Raspberry Pi 5 FC
- **Servos:** IC12 servos and motors for precise control of the aircraft's surfaces
- **VTX:** MSD VTX Support, potentially using Open VTX
- **Sensors:** Serial GPS, Accelerometer, Magnetometer (Compass), ETC 

## Hardware Requirements
- Raspberry Pi-5
- ELRS TX and RX
  - Run at 100 HZ to save CPU Time
- Bo085
  -IMU
  -MAG <-- Proably to garbled by nearby em interference.
- Raspberry Pi Pico
- etc.

## Software Requirements
- Ubuntu 24.04LTS
- ROS2 Jazzy
- MicroROS
- Adafruit Libaries
