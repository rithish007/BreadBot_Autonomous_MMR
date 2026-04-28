# BreadBot_Autonomous_MMR
An autonomous mobile robot that navigates, positions itself, and draws using a marker with the help of encoder feedback and a 2-DOF robotic arm. The system is designed for simplicity using encoder feedback for motion control and an ultrasonic sensor for distance based positioning.

<img width="400" height=auto alt="Robot" src="https://github.com/user-attachments/assets/74c93476-ec4b-43ad-8bbc-e57d73bf3574" />

## System Components
- Microcontroller: Arduino Mega 2560
- Drive System: Differential drive with encoder DC motors
- Manipulator: 2-DOF robotic arm (servo actuated)
- Sensor: HC-SR04 ultrasonic sensor
- Motor Driver: TB6612FNG

## Features
- Autonomous navigation using encoder feedback
- Distance-based stopping with ultrasonic sensing
- Repeatable marker placement using predefined arm positions
- Lightweight finite-state control logic

## How It Works
The robot follows a sequence of actions:
- Moves forward until a set distance is reached
- Executes turns using encoder targets
- Aligns near the target location
- Activates the arm to release the marker

## Contributors
- [Rithish Ramamoorthy Sathya](https://github.com/rithish007)
- [Sofia Tostado Puente](https://github.com/CSTostadoP)
- [Ibim Duopama-Obomanu](https://github.com/Ibim12)
