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
                msg = String()
                msg.data = line
                self.publisher_.publish(msg)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial port error: {e}")

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

