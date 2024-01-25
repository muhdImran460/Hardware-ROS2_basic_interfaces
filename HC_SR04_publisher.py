#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64

import serial

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        self.publisher_ = self.create_publisher(String, 'ultrasonic_distance', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        # Serial configuration
        self.serial_port = '/dev/ttyACM0'  # Change this to your Arduino's serial port
        self.serial_baudrate = 9600
        self.serial_timeout = 5  # Timeout in seconds
        self.ser = serial.Serial(self.serial_port, self.serial_baudrate, timeout=self.serial_timeout)

    def timer_callback(self):
        # Read serial data from Arduino
        serial_data = self.ser.readline().decode('utf-8').strip()

        # Publish data to ROS2 topic
        msg = String()
        msg.data = str(serial_data)
        self.publisher_.publish(msg)
        #self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    ultrasonic_node = UltrasonicNode()
    rclpy.spin(ultrasonic_node)
    ultrasonic_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
