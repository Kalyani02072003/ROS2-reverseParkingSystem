#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import matplotlib.pyplot as plt
from collections import deque

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.serial_port = None
        self.init_serial()

        self.ir_data1 = deque(maxlen=100)  # Deque to store IR sensor 1 data
        self.ir_data2 = deque(maxlen=100)  # Deque to store IR sensor 2 data
        self.ultrasonic_data = deque(maxlen=100)  # Deque to store ultrasonic sensor data

        self.timer = self.create_timer(0.1, self.timer_callback)

        plt.ion()  # Turn on interactive mode for live plotting

    def init_serial(self):
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info("Serial port opened successfully")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")

    def timer_callback(self):
        try:
            if self.serial_port and self.serial_port.in_waiting:
                line = self.serial_port.readline().decode('utf-8', 'ignore').rstrip()
                self.get_logger().info(f"Received data from Arduino: {line}")
                
                ultrasonic_distance, ir_distance1, ir_distance2 = self.parse_data(line)
                
                if ultrasonic_distance is not None and ir_distance1 is not None and ir_distance2 is not None:
                    self.ir_data1.append(ir_distance1)
                    self.ir_data2.append(ir_distance2)
                    self.ultrasonic_data.append(ultrasonic_distance)
                    
                    self.plot_data()
                
        except serial.SerialException as e:
            self.get_logger().error(f"Serial port error: {e}")

    def parse_data(self, data):
        parts = data.split('|')
        
        # Debug print
        self.get_logger().info(f"Received data parts: {parts}")
        
        if len(parts) >= 3:
            try:
                ultrasonic_distance = float(parts[0].split(':')[1].strip().split(' ')[0])
                ir_distance1 = float(parts[1].split(':')[1].strip().split(' ')[0])
                ir_distance2 = float(parts[2].split(':')[1].strip().split(' ')[0])
                return ultrasonic_distance, ir_distance1, ir_distance2
            except IndexError as e:
                self.get_logger().error(f"Error parsing data: {e}")
                return None, None, None
        else:
            self.get_logger().error("Invalid data format")
            return None, None, None

    def plot_data(self):
        plt.clf()  # Clear previous plot
        plt.plot(self.ir_data1, label='IR Distance 1 (cm)')
        plt.plot(self.ir_data2, label='IR Distance 2 (cm)')
        plt.plot(self.ultrasonic_data, label='Ultrasonic Distance (cm)')
        plt.title('IR and Ultrasonic Sensor Data')
        plt.xlabel('Time')
        plt.ylabel('Distance (cm)')
        plt.legend()
        plt.pause(0.01)

    def stop_node(self):
        self.get_logger().info("Stopping serial node")
        print("Car is parked now!")
        print("You have stopped the Reverse Car Parking System")
        if self.serial_port:
            self.serial_port.close()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    try:
        rclpy.spin(serial_node)
    except KeyboardInterrupt:
        serial_node.stop_node()

if __name__ == '__main__':
    main()

