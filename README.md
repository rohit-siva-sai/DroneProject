# DroneProject


1. Project Title

Autonomous Drone Navigation with Return-to-Home (Low Battery)

2. Project Description

This project implements an autonomous navigation system for a quadrotor drone using the Cyberbotics Webots simulator and C programming language.
The drone autonomously takes off, navigates through predefined waypoints using a Turn-and-Move strategy, continuously monitors battery status, and automatically returns to the home position when the battery level drops below a critical threshold.

3. Objectives

To design a fully autonomous drone navigation system.

To implement accurate waypoint tracking without lateral drift.

To ensure safe flight using PID-based altitude control.

To provide an automatic Return-to-Home (RTH) feature during low-battery conditions.

To visualize drone movement and navigation states in real time.

4. Key Features

Autonomous Waypoint Navigation:
The drone follows a predefined set of waypoints without manual control.

Turn-and-Move Navigation Logic:
The drone first aligns its yaw toward the target waypoint and then moves forward, ensuring precise navigation.

PID-Based Altitude Control:
Maintains stable altitude during takeoff, hover, movement, and landing.

Low-Battery Detection:
Battery level is continuously monitored during flight.

Automatic Return-to-Home (RTH):
When battery level falls below 20%, the drone automatically returns to the home position.

Visual Alert System:
Onboard LEDs blink to indicate a low-battery warning.

Flight Path Visualization:
Waypoints, home position, and drone trail are visualized using Webots Supervisor API.

5. System Architecture

Simulation Environment: Webots 3D environment

Control System: Finite State Machine (FSM)

Sensors Used: GPS, IMU, Gyroscope

Actuators: Four propeller motors

Safety Module: Battery monitoring and RTH logic

6. Tools and Technologies Used

Simulator: Cyberbotics Webots (R2023b or newer)

Programming Language: C

Libraries / APIs:

Webots C Controller API

Standard C Libraries (math.h, stdio.h, stdbool.h)

Robot Model: DJI Mavic 2 Pro

7. Dependencies

Webots Simulator

C Compiler (included with Webots)

8. How to Run the Project

Open Webots Simulator.

Load the drone world file (.wbt).

Create or select a C controller.

Copy the provided controller.c code into the controller folder.

Assign the controller to the drone model.

Start the simulation by clicking Run.

Observe autonomous takeoff, navigation, RTH, and landing behavior.

9. Files Included

controller.c – Main autonomous navigation and control code

README.md – Project documentation

Demo Video (link provided separately)

10. Output and Results

Successful autonomous takeoff and landing.

Accurate waypoint navigation without sideways drift.

Automatic return-to-home on low battery.

Real-time visual indicators for navigation and safety states.

11. Future Enhancements

Dynamic obstacle avoidance.

Real-time battery discharge modeling.

Integration of vision-based navigation.

Multi-drone coordination.

12. Authors / Team

Reddy Rohit Siva Sai (206125022) – Autonomous Navigation & Control Logic

Sai Kiran (206125015) – Safety System & Return-to-Home
