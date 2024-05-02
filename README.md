# Reverse Car Parking System

This project implements a Reverse Car Parking System using ROS 2 and Arduino. It utilizes ultrasonic and infrared (IR) sensors to detect obstacles and aid in parking.

## Overview

The system consists of two main components:

1. **Serial Server (Arduino)**: This component runs on an Arduino board connected to ultrasonic and IR sensors. It collects sensor data and publishes it over serial communication.

2. **Serial Client (ROS 2 Node)**: This component is a ROS 2 node that subscribes to the sensor data published by the Serial Server. It processes the sensor data and provides feedback on the parking status.

## ROS2 Setup 
1. Install ROS 2 and required dependencies:

2. Set up ROS 2 workspace:
    ```
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

3. Clone the project repository into the `src` directory:
    ```
    git clone https://github.com/Kalyani02072003/ROS2-reverseParkingSystem.git
    ```

4. Build the workspace:
    ```
    cd ~/ros2_ws
    colcon build
    ```

5. Source the workspace:
    ```
    source ~/ros2_ws/install/setup.bash
    ```

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

