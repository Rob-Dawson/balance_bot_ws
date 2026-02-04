<h1 align="center">Balance_bot</h1>

## About

The project features a ROS2 and Gazebo Harmonic balancing robot which is being continuously updated. The purpose is to explore different control algorithms and investigate how they perform and the differences between them.
 
## Technologies

The following tools were used in this project:

ROS2 - Jazzy
Python
Gazebo Harmonic

## Requirements

Before starting, you need to have [ROS2-Jazzy](https://docs.ros.org/en/jazzy/Installation.html) and [Gazebo Harmonic](https://gazebosim.org/docs/latest/ros_installation/) installed. I tend to install ros2 using the desktop-full for a smooth experience. Once ROS2 Jazzy is installed, you then need ros2_control and the Gazebo to ros2_control bridge
```bash
# Install ros 2 control ros2 controllers and the gazebo bridge
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-sim
```

## Starting

```bash
# Clone this project
git clone https://github.com/Rob-Dawson/balance_bot_ws

# Access
cd balance_bot_ws

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash

# Launch the robot
# I will update this when a functioning launch file is present
```


&#xa0;

<a href="#top">Back to top</a>
