# Firmware

## Overview

The firmware controls the physical BalanceBot platform and is responsible for sensor acquisition, state estimation and motor control.

The project is currently developed using PlatformIO with the Arduino framework, while being structured to support additional embedded platforms in the future.

Current development is focused on improving code portability, separating hardware-specific implementations from reusable control logic, and evaluating different control architectures.

---

## Supported Hardware

### Current

- Arduino UNO
- MPU6050 IMU
- L298N Motor Driver
- 12 V 300 RPM Brushed DC Motors with Quadrature Encoders

### Planned

- ESP32-C5
- TB6612FNG Motor Driver

---

## Project Structure

```text
firmware/
├── include/
├── src/
├── lib/
├── test/
└── platformio.ini
```

The project follows the standard PlatformIO layout.

The `include` and `src` directories are organised into subsystems such as controllers, IMU, motor drivers and encoder interfaces to keep platform-independent logic separated from hardware-specific implementations where possible.

---

## Building

Open the `firmware` directory as a PlatformIO project.

Build the firmware using either the PlatformIO toolbar within VSCode or:

```bash
pio run
```

---

## Uploading

Connect the target board and upload using either the PlatformIO toolbar within VSCode or:

```bash
pio run --target upload
```

---

## Current Development

Current work includes:

- Refactoring the firmware architecture
- Improving hardware abstraction
- Expanding telemetry support
- Preparing for additional embedded platforms
- Improving controller portability

---

## Future Work

Planned work includes:

- ESP-IDF support
- Additional motor driver implementations
- Improved hardware abstraction
- Shared interfaces across embedded platforms
- Integration with the telemetry tools
