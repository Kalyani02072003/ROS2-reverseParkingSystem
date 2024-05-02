#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SerialClient(Node):
    def __init__(self):
        super().__init__('serial_client')
        self.subscription = self.create_subscription(
            String,
            'serial_data',
            self.serial_callback,
            10
        )
        self.is_parked = False

    def serial_callback(self, msg):
        data = msg.data
        ultrasonic_distance, ir_distance_1, ir_distance_2 = self.parse_data(data)
        
        if int(ir_distance_1) <= 500 :
            print("Human presence detected")
        if int(ultrasonic_distance) <= 10:
            print("Car is about to hit the back side! Buzzer on!")
        if "parked" in data:
            if not self.is_parked:
                print("Car is parked!")
                self.is_parked = True
        else:
            print("Continue parking")

    def parse_data(self, data):
        parts = data.split('|')
    
        if len(parts) >= 3:
            try:
                ultrasonic_distance = int(parts[0].split(':')[1].strip().split(' ')[0])
                ir_distance_1 = int(parts[1].split(':')[1].strip().split(' ')[0])
                ir_distance_2 = int(parts[2].split(':')[1].strip())
                return ultrasonic_distance, ir_distance_1, ir_distance_2
            except (ValueError, IndexError) as e:
                self.get_logger().error(f"Error parsing data: {e}")
                return None, None, None
        else:
            self.get_logger().error("Invalid data format")
            return None, None, None


def main(args=None):
    rclpy.init(args=args)
    serial_client = SerialClient()
    try:
        rclpy.spin(serial_client)
    except KeyboardInterrupt:
        print("Car is parked now")
    finally:
        serial_client.destroy_node()
        rclpy.shutdown()

    serial_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

