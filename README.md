Stewart Platform U-joint Digital Shadow 


https://github.com/user-attachments/assets/c09ef7ee-dcb9-4c92-a32a-1affe8adb6ae

https://youtu.be/XMISqJ7-U6o

# Stewart Platform Joint Digital Shadow

This repository contains MATLAB code and supporting files for the joint-level digital shadow framework presented in the IEEE Access paper:

**Real-Time Validation of Stewart Platform U-Joint Kinematics Through a Digital Shadow**  
Juliana Danesi Ruiz, Skylar Halley, Casey Harwood, and Rachel V. Vitali  
IEEE Access, 2026  
DOI: `10.1109/ACCESS.2026.3682958`

## Overview

Universal joints in Stewart platforms can experience kinematic limit violations and mechanical collisions that are not always captured by platform-level inverse kinematics or offline simulation alone. This repository provides tools for modeling, validating, and monitoring the U-joint behavior of a 6-UPU Stewart platform using a joint-level digital shadow.

The framework uses:

- Stewart platform inverse kinematics
- U-joint angle calculations for base and platform joints
- MATLAB collision mesh analysis
- A precomputed U-joint collision lookup table
- Low-cost IMU measurements
- A MATLAB App Designer interface for static and dynamic validation

The digital shadow continuously compares measured joint behavior against calculated kinematic predictions and evaluates the joint state against a collision and limit lookup table.

## Research Context

This code supports the development of a digital shadow for a six-degree-of-freedom Stewart platform, also referred to as a hexapod. The physical platform uses a 6-UPU architecture, where each actuator is connected to the base and moving platform through universal joints.

The research demonstrates that joint-level monitoring can identify unsafe configurations and potential U-joint collisions in real time. This provides an operational safety layer that complements traditional inverse kinematic models and offline simulation tools.

## Repository Contents

| File | Description |
|---|---|
| `StewartPlatform_JointKinematics.mlapp` | MATLAB App Designer GUI for initializing the hexapod, validating U-joint kinematics, and running the digital shadow interface. |
| `InitializeHexapodObject.m` | Initializes the Stewart platform geometry and model parameters. |
| `InverseKinematics_hexapod.m` | Computes actuator lengths from the commanded platform pose. |
| `InverseKinematicsEfficient.m` | Efficient inverse kinematics implementation for repeated calculations. |
| `InverseKinematicsEfficient2.m` | Alternative efficient inverse kinematics implementation. |
| `GatherArduinoData_BaseJoint.m` | Collects IMU data for base U-joint validation. |
| `GatherArduinoData_Plat.m` | Collects IMU data for platform U-joint validation. |
| `checkYokeCollision.m` | Evaluates U-joint yoke collision conditions. |
| `MadgwickAHRS.m` | Madgwick filter implementation for IMU orientation estimation. |
| `quaternProd.m` | Quaternion product utility function. |
| `E2R.m` | Converts Euler angles to a rotation matrix. |
| `lookupTableInterpolant.mat` | Precomputed collision lookup table for U-joint minimum distance estimation. |
| `yoke_meshes.mat` | Mesh data for U-joint/yoke collision modeling. |
| `README.md` | Project documentation. |
| `LICENSE` | MIT license. |

## Methodology

The workflow implemented in this repository follows four main steps:

1. **Hexapod Initialization**  
   Define the geometry of the 6-UPU Stewart platform, including base points, platform points, actuator geometry, and U-joint mesh information.

2. **Inverse Kinematics**  
   Convert a commanded platform pose into actuator link vectors and actuator lengths.

3. **U-Joint Kinematics**  
   Compute the two rotational angles of each universal joint using the actuator direction vectors and known joint geometry.

4. **Digital Shadow Validation**  
   Compare calculated U-joint angles against measured IMU-based joint angles and use the lookup table to estimate minimum yoke clearance in real time.

## Digital Shadow Architecture

The digital shadow operates in parallel with the actuator control system. It does not directly control the platform; instead, it monitors and validates joint behavior during operation.

The system includes:

- A physical Stewart platform with six linear actuators
- IMUs mounted on the platform and selected U-joints
- Arduino-based IMU data acquisition
- MATLAB-based orientation filtering and kinematic processing
- Real-time visualization and collision checking through the GUI

Future work may extend this architecture toward a full digital twin by adding bidirectional communication between the digital model and the physical control system.

## Requirements

This project was developed using MATLAB and requires the following toolboxes or capabilities:

- MATLAB
- MATLAB App Designer
- Robotics System Toolbox or collision mesh support
- Arduino support package, if collecting live IMU data
- Access to IMU sensor data from TDK ICM-20948 sensors or equivalent
- Precomputed lookup table and yoke mesh files included in this repository

## Hardware Used in the Paper

The experimental platform described in the paper used:

- 6 Progressive Automations PA-14P linear actuators
- Arduino Due for actuator control
- Arduino Uno for IMU data acquisition
- SparkFun Qwiic Mux breakout board
- TDK ICM-20948 9-DOF IMU sensors
- 3D-printed base, platform, actuator end caps, and U-joint components
- MATLAB host computer for digital shadow processing

## Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/juliana-lab/Stewart-Platform-Joint-Digital-Shadow.git
cd Stewart-Platform-Joint-Digital-Shadow
