# Quadruped Robot with YOLO-Based Object Tracking & Autonomous Motion

A fully functional quadruped robot inspired by Boston Dynamics' Spot. This robot is capable of stable walking, balance correction using sensor fusion, and autonomous human/object following using real-time vision and depth sensing.

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Hardware Architecture](#hardware-architecture)
- [Software Stack](#software-stack)
- [Tracking & Following System](#tracking--following-system)
- [Motion Control Pipeline](#motion-control-pipeline)
- [Power System](#power-system)
- [Getting Started](#getting-started)
- [Future Plans](#future-plans)

---

## Overview

This project is a modular quadruped robot developed by a student team from IIT Kharagpur. Designed and 3D-modeled from scratch, the robot uses powerful servos, embedded computing platforms, and a rich set of sensors for mobility, balance, and object interaction in real-time.

---

## Features

- Modular leg design with 3 DOF per leg
- Real-time human/object tracking using YOLO and depth camera
- Sensor fusion using IMU and pressure sensors for dynamic balance
- Dual onboard processing with Jetson Nano and Raspberry Pi 4
- ROS-compatible software architecture
- 3D-printed frame and open-source hardware

---

## Hardware Architecture

### Controllers
- **Jetson Nano**: Vision processing, object tracking, and motion planning
- **Raspberry Pi 5**: Sensor interfacing, depth processing, and ROS nodes

### Actuators
- **DS5160 (60kg·cm)**: Hip actuation
- **DS3225 (25kg·cm)**: Knee and lateral leg movement

### Sensors
- **10-axis IMU** (ROS-compatible)
- **Pressure sensors** (1 per foot)
- **GPS module**
- **LiDAR**
- **Depth Camera**: Sipeed MaixSense
- **2x 8MP CSI Cameras** (120° FOV) for stereo vision

---

## Software Stack

- **Operating System**: Ubuntu 22.04 (Jetson & Pi)
- **Languages**: Python, C++
- **Frameworks**:
  - [ROS 2] (Robot Operating System)
  - YOLOv5 or YOLOv8 (on Jetson Nano)
  - OpenCV for preprocessing
  - Sensor fusion filters
  - PID controllers for balance

---

## Tracking & Following System

- **Camera Input**: One CSI camera to Jetson Nano, one to Pi
- **Detection**: YOLO detects humans/objects
- **Depth Mapping**: Depth at bounding box center is fetched from MaixSense
- **Control**: Robot follows based on offset from center & target distance

### YOLO + Depth Workflow:
1. YOLO detects object → gives bounding box
2. Center pixel is extracted
3. Depth value at pixel is fetched
4. Control logic aligns and moves robot toward object

---

## Motion Control Pipeline

1. **Sensor Data Input**:
   - IMU (roll, pitch, yaw)
   - Pressure sensors (ground contact)
2. **Sensor Fusion**:
   - Orientation estimation
   - Foot contact validation
3. **Balance Control**:
   - PID feedback on lateral & pitch axes
4. **Gait Planning**:
   - FSM for trot/walk
   - Weight shifting before leg lift
5. **Motion Execution**:
   - Servo commands via PWM or motor driver

---

## Power System

- **Batteries**:
  - 2 × 5200mAh 11.4V 3S LiPo
  - 2 × 2200mAh 11.4V 3S LiPo
  - 2S 5P 7.4V Li-ion (13Ah total)
- **Power Management**:
  - 3S 40A BMS for safety
  - Buck converters for all servos
  - Buck-boost converter for Jetson Nano

---

## Future Plans

Add SLAM & obstacle avoidance

Dynamic gait learning using reinforcement learning

Fully autonomous navigation

Remote control via app or voice


---

### Authors

Team Proboticists, IIT Kharagpur
Designed & developed by students passionate about robotics.
