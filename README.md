# 🤖 6894 Crescendo Swerve
# ⛄☕ Iced Java 6894 repository for 2024 swerve drive robot.

<div style="display: flex;">
  <img src="./img/img1.gif" style="width: 50%;padding:2px" alt="Image 1">
</div>

<br>**Table Of Contents:**
- Features
- Hardware
- Source Code

## ✨ Features
Our swerve drive robot has the following features:
- Swerve drive
- Slow roatiational speed for precise turning
- Field centric drive (Via button input)
- Gyro correction
- PID control

## 🔧 Hardware
Our version of the source code has been modified to work with our hardware. 

We use the following hardware for our swerve drive robot:
- Swerve Module: MK4i L2 from Swerve Drive Specialties
- 8x [SPARK MAX](https://www.revrobotics.com/rev-11-2158/) motor controllers (Two per module, drive & turn)
- 8x [NEO Brushless](https://www.revrobotics.com/rev-21-1650/) motors (Two per module, drive & turn)
- 4x [CTRE CANcoder](https://store.ctr-electronics.com/cancoder/) (One per module) acts as absolute encoder for the turn motor
- 1x [CTRE Pigeon 2.0](https://store.ctr-electronics.com/blog/pigeon-20-hardware-update/) (One per robot) acts as gyro for the robot to determine heading
- 1x [Extreme 3D Pro Joystick](https://www.logitechg.com/en-us/products/space/extreme-3d-pro-joystick.963290-0403.html) (One per robot) acts as controller for the driver to control the robot
