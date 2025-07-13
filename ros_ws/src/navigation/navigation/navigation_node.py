#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import serial
import time

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('use_arduino', True)
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.use_arduino = self.get_parameter('use_arduino').value
        
        # Initialize serial connection if using Arduino
        if self.use_arduino:
            try:
                self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
                self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
            except serial.SerialException:
                self.get_logger().error(f"Failed to connect to Arduino on {self.serial_port}")
                self.ser = None
        
        # Subscribe to egg detection topic
        self.egg_detection_sub = self.create_subscription(
            Bool,
            'egg_detected',
            self.egg_detection_callback,
            10)
            
        self.get_logger().info("Navigation node initialized")
        
        # Robot state
        self.is_moving = True
    
    def egg_detection_callback(self, msg):
        if msg.data:
            self.get_logger().info("Egg detected! Stopping robot")
            self.stop_robot()
    
    def stop_robot(self):
        if not self.is_moving:
            return
            
        self.is_moving = False
        
        if self.use_arduino and self.ser:
            # Send stop command to Arduino
            self.ser.write(b'STOP\n')
            self.get_logger().info("Sent STOP command to Arduino")
        else:
            self.get_logger().info("Robot would stop now (no Arduino connected)")
    
    def resume_robot(self):
        if self.is_moving:
            return
            
        self.is_moving = True
        
        if self.use_arduino and self.ser:
            # Send resume command to Arduino
            self.ser.write(b'RESUME\n')
            self.get_logger().info("Sent RESUME command to Arduino")
        else:
            self.get_logger().info("Robot would resume now (no Arduino connected)")

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()