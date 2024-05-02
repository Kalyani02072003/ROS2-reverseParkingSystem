# Reverse Car Parking System

This project implements a Reverse Car Parking System using ROS 2 and Arduino. It utilizes ultrasonic and infrared (IR) sensors to detect obstacles and aid in parking.

## Overview

The system consists of two main components:

1. **Serial Server (Arduino)**: This component runs on an Arduino board connected to ultrasonic and IR sensors. It collects sensor data and publishes it over serial communication.

2. **Serial Client (ROS 2 Node)**: This component is a ROS 2 node that subscribes to the sensor data published by the Serial Server. It processes the sensor data and provides feedback on the parking status.

## Usage

1. Connect the Arduino board with the ultrasonic and IR sensors.
2. Upload the `serial_server.ino` sketch to the Arduino board.
3. Run the Serial Server:
    ```
    ros2 run UltrasonicData server.py
    ```
4. Run the Serial Client:
    ```
    ros2 run UltrasonicData client.py
    ```
4. Run the Serial Plotter for real-time plotting:
    ```
    ros2 run UltrasonicData serialPlotter.py
    ```
    
## Features
- Real-time monitoring of ultrasonic and IR sensor data.
- Detection of obstacles and human presence during parking.
- Feedback on parking status, including alerts for potential collisions.

