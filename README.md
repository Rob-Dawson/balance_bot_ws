<h1 align="center">BalanceBot</h1>
<h3>Cross-platform self-balancing robot exploring embedded firmware,
robotics software architecture and modern control systems.</h3>

---

## About

BalanceBot is a project which began as a ROS 2 and Gazebo simulation and has gradually evolved into a physical self-balancing robot. The project investigates how different control architectures such as Cascading PID, LQR and MPC perform while developing portable embedded firmware that can be reused across simulation and multiple hardware platforms.

---

## Technologies

| Firmware | Simulation | Analysis |
| --- | --- | --- |
| C++ | ROS 2 Jazzy | Python |
| Arduino | Gazebo Harmonic | PlotJuggler |
| ESP-IDF *(planned)* | | Streamlit |

---

## Getting Started

BalanceBot currently provides two independent development workflows.

### ROS 2 Simulation

Develop, test and evaluate balancing controllers within a simulated environment using ROS 2 Jazzy and Gazebo Harmonic.

**[Simulation Guide](simulation/README.md)**


### Firmware

Deploy the balancing controller to the physical robot using PlatformIO and the Arduino framework.

**[Firmware Guide](firmware/README.md)**

Native ESP-IDF support is currently under development.

---

## Hardware

### Current Hardware

- Arduino UNO
- MPU6050 IMU
- L298N Motor Driver
- 12 V 300 RPM Brushed DC Motors with Integrated Encoders

### Planned Hardware

- ESP32-C5
- TB6612FNG Motor Driver

---

## Current Development

Current work focuses on improving the firmware architecture through hardware abstraction, allowing the same control logic to be reused across simulation and embedded platforms.

Development is also underway on an offline telemetry dashboard for analysing controller behaviour and comparing control algorithms.

Future work includes native ESP-IDF support, wireless telemetry and the evaluation of additional control architectures.

---
## Credits

The original mechanical design is based on:

- *(https://makerworld.com/en/models/691734-fall-e-the-self-balancing-robot?from=search#profileId-620442)*

---


&#xa0;

<a href="#top">Back to top</a>
