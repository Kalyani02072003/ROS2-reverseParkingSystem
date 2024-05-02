#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info("Serial port opened successfully")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Timer callback triggered")
        try:
            if self.serial_port.in_waiting:
                line = self.serial_port.readline().decode('utf-8').rstrip()
                self.get_logger().info(f"Read data from serial port: {line}")
                ultrasonic_distance, ir_sensor_value = self.parse_data(line)
                self.process_data(ultrasonic_distance, ir_sensor_value)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial port error: {e}")

    def parse_data(self, data):
        parts = data.split(',')
        ultrasonic_distance = int(parts[0].split(':')[1].strip())
        ir_sensor_value = int(parts[1].split(':')[1].strip())
        return ultrasonic_distance, ir_sensor_value

    def process_data(self, ultrasonic_distance, ir_sensor_value):
        if ultrasonic_distance < 30:
            self.get_logger().info("Car is near obstacle. Drive Carefully!")
        else:
            self.get_logger().info("Continue Parking")
        
        if ir_sensor_value > 700:
            self.get_logger().info("Living object Detected. Drive Carefully!")
        else:
            self.get_logger().info("Continue parking")

    def stop_node(self):
        self.get_logger().info("Car Parked!")
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
