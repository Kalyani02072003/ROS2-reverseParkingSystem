#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.serial_port = None
        self.init_serial()

        self.timer = self.create_timer(0.1, self.timer_callback)

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
        except serial.SerialException as e:
            self.get_logger().error(f"Serial port error: {e}")

    def stop_node(self):
        self.get_logger().info("Stopping serial node")
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

