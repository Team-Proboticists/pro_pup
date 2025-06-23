# Quadruped Robot Dog: Gait-Controlled Walking Robot  
*A modular quadruped platform for research and development*

## Table of Contents
- [Overview](#overview)  
- [Features](#features)  
- [Hardware Architecture](#hardware-architecture)  
- [Software Stack](#software-stack)  
- [Motion Control Pipeline](#motion-control-pipeline)  
- [Power System](#power-system)  
- [Current Limitations](#current-limitations)  
- [Future Roadmap](#future-roadmap)  
- [Authors](#authors)

---

## Overview

This project is a custom-designed quadruped robot developed by undergraduate students at IIT Kharagpur as part of the Inter-Hall General Championship. Designed and 3D-modeled from scratch, the robot demonstrates stable walking using a predefined gait cycle and is controlled using a wireless gamepad. The current version focuses on mechanical design, embedded control, and basic motion execution, serving as a foundation for future autonomous functionality.

---

## Features

- **3 DOF per Leg**: Each leg has hip yaw, hip pitch, and knee pitch degrees of freedom  
- **Predefined Gait Execution**: Robot walks in a fixed direction using a static gait sequence  
- **Wireless Gamepad Control**: Controlled via USB receiver plugged into Raspberry Pi  
- **Modular & Repairable**: 3D-printed frame and standardized leg modules  
- **ROS 2 Ready**: System design is ROS 2-compatible for future integration

---

## Hardware Architecture

**Controllers:**
- **Raspberry Pi 4 / 5**: Handles motor control logic, sensor input, and ROS 2 nodes  


**Actuators:**
- **DS5160 (60kg·cm) servo** – Hip pitch  
- **DS3225 (25kg·cm) servo** – Knee and hip yaw

**Sensors:**
- **10-axis IMU** – Orientation sensing  
- *(Future support planned for pressure sensors, depth camera, LIDAR, and GPS)*

---

## Software Stack

- **Operating System:** Raspberry Pi OS / Ubuntu 22.04  
- **Programming Languages:** Python, C++  
- **Middleware:** ROS 2 (Foxy)  
- **Motion Logic:** Predefined gait states stored and replayed  
- **User Input:** Wireless gamepad via USB dongle (plugged into Pi)

---

## Motion Control Pipeline

1. **Gait Cycle**: Manually triggered cyclic joint trajectories for all legs  
2. **Leg Coordination**: FSM-based gait synchronizes foot contacts and swing phases  
3. **Control Strategy**: Feedforward joint angle sequences  
4. **Execution**: Servo angles sent via PWM using RPi bridge

---

## Power System

- **Power Sources:**
  - 2 × 5200mAh 11.1V LiPo for actuators  
  - 2S 5P 7.4V Li-ion (13Ah total) as separate supply for computing units (5V stepped down)

- **Power Management:**
  - 3S 40A BMS for safety
  - Buck converters for servo rail isolation  
  - Basic thermal & voltage safety integrated

---

## Current Limitations

- No active balance or feedback control  
- No vision, tracking, or sensor-based navigation  

---

## Future Roadmap

We aim to evolve this platform into a fully autonomous quadruped research robot. Planned improvements include:

- **Dynamic Gait Generation** with IMU feedback  
- **SLAM and Obstacle Avoidance** using LiDAR and stereo cameras  
- **YOLO-Based Object Tracking** with real-time depth mapping  
- **Reinforcement Learning for Gait Adaptation**  
- **Jetson Nano** for all the media processing and mapping 
- **Fully Wireless Operation**  
- **Custom App or Voice Command Interface**

---

## Authors

**Team Proboticists**  
IIT Kharagpur  
*Designed, built, and tested by a student team passionate about robotics.*
