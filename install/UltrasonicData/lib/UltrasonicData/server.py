#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialServer(Node):
    def __init__(self):
        super().__init__('serial_server')
        self.serial_port = None
        self.publisher_ = self.create_publisher(String, 'serial_data', 10)
        self.init_serial()

    def init_serial(self):
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info("Serial port opened successfully")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")

    def publish_serial_data(self):
        if self.serial_port and self.serial_port.in_waiting:
            line = self.serial_port.readline().decode('utf-8', 'ignore').rstrip()
            msg = String()
            msg.data = line
            self.get_logger().info(f"Publishing data: {line}")
            self.publisher_.publish(msg)

    def stop_node(self):
        self.get_logger().info("Stopping serial server")
        if self.serial_port:
            self.serial_port.close()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    serial_server = SerialServer()
    try:
        while rclpy.ok():
            serial_server.publish_serial_data()
            rclpy.spin_once(serial_server, timeout_sec=0.1)
    except KeyboardInterrupt:
        serial_server.stop_node()

if __name__ == '__main__':
    main()

