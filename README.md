# Lab 1 – Introduction to Robotics and Intelligent Systems

## Project Overview

This repository contains the implementation of Lab 1 using ROS 2 Humble and Gazebo. The exercise covers robot simulation, sensors, ROS 2 nodes, parameters, custom interfaces, services, actions, and autonomous movement.

## Repository Structure

| Package | Description |
|---|---|
| `diffdrive_bot` | Robot model, simulation, sensors, and differential-drive control |
| `lab_interfaces` | Custom message, service, and action interfaces |
| `my_robot_interfaces` | Custom interfaces developed for the lab |
| `my_robot_nodes` | Python nodes implementing the lab exercises |

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Harmonic
- Python 3
- colcon

## Build Instructions

Open a terminal in the ROS 2 environment and run:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

For every new terminal, source the environment again:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```


## Lab Implementation

### Step 1 – Robot Sensors

Extended the robot simulation with additional sensors, including a depth camera and an IMU. Sensor topics can be inspected using:

```bash
ros2 topic list
```

Sensor data can be visualized using RViz.

### Step 2 – Turn Away Node

Implemented the `turn_away` node to control the robot's turning behavior using sensor information.

### Step 3 – ROS 2 Parameters

Added parameters to configure the node's behavior at runtime without modifying its source code.


### Step 4 – Custom Robot Status Message

Created a custom `RobotStatus` message to represent the robot's status information.

### Step 5 – Services

Implemented service-based control, including enabling and disabling turning behavior and setting the turning speed.

### Step 6 – Rotate Action Server

Implemented the `rotate_server` action server to rotate the robot by a specified angle. The server provides feedback during execution and supports cancellation.


### Step 7 – Square Movement

Implemented `square_driver` to make the robot follow a square path by sending movement and rotation goals in sequence.

Run the square movement using:

```bash
ros2 launch my_robot_nodes square.launch.py side_length:=0.8
```

The `side_length` argument controls the length of each side.

## Useful ROS 2 Commands

List active nodes:

    ros2 node list

List active topics:

    ros2 topic list

List services:

    ros2 service list

List actions:

    ros2 action list

List package executables:

    ros2 pkg executables my_robot_nodes

## Screenshots and Evidence

The following screenshots will be included in the `screenshots/` directory:

| Evidence | File |
|---|---|
| Step 1 – Depth camera visualization in RViz | `screenshots/depth_image.png` |
| Step 1 – Point cloud visualization in RViz | `screenshots/point_cloud.png` |
| Step 7 – ROS node communication using rqt_graph | `screenshots/rqt_graph.png` |

## Challenges and Observations

The simulation and robot behavior depend on the relevant ROS 2 nodes and topic publishers being active. The rotation action server relies on odometry data to measure the robot rotation. If the simulation or odometry publisher stops, the server cannot receive the updates needed to calculate progress.

Obstacle detection can prevent movement when an obstacle is within the configured safety distance. This is expected safety behavior.

## Conclusion

This lab provided practical experience with ROS 2 and robot simulation, covering sensor integration, node development, parameters, custom interfaces, services, actions, and coordinated robot movement.
