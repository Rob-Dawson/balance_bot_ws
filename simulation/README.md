# Simulation

## Overview

The simulation environment provides a ROS 2 Jazzy and Gazebo Harmonic implementation of BalanceBot.

The simulator is used to develop and evaluate controllers before deploying them to physical hardware. It provides a repeatable environment for testing controller behaviour, collecting telemetry and validating changes without risking damage to the robot.

---

## Packages

The simulation workspace currently contains:

| Package | Description |
|----------|-------------|
| balance_bot_description | Robot description, URDF and Gazebo configuration |
| balance_bot_controller | Controller implementation |
| balance_bot_bringup | Launch files and simulation startup |

---

## Requirements

Install:

- ROS 2 Jazzy
- Gazebo Harmonic

Additional packages required by this project:

```bash
sudo apt install ros-jazzy-ros2-control
sudo apt install ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-ros-gz
sudo apt install ros-jazzy-ros-gz-sim
sudo apt install ros-jazzy-gz-ros2-control
sudo apt install ros-jazzy-imu-filter-madgwick
```

---

## Workspace Layout

```text
simulation/
└── balance_bot_ws/
    └── src/
```

The repository already contains the ROS workspace source tree.

Create or use an existing ROS 2 workspace and place the contents of this repository into the workspace before building.

---

## Building

From the workspace root:

```bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

---

## Launching

Start the simulator with:

```bash
ros2 launch balance_bot_bringup gazebo.launch.py
```

---

## Visualisation and Analysis

Useful tools include:

- PlotJuggler
- RViz2
- rqt

Install PlotJuggler:

```bash
sudo apt install ros-jazzy-plotjuggler-ros
```

---

## Purpose

The simulation is used to:

- Develop new controllers
- Compare controller architectures
- Validate changes before hardware deployment
- Record telemetry for offline analysis
- Experiment with new robotics software architecture

As the project develops, new control approaches will be evaluated and compared within the simulation before being transferred to the physical platform.
