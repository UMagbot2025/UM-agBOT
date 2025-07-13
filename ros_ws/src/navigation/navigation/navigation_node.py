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
        self.declare_parameter('use_mock', False)
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.use_arduino = self.get_parameter('use_arduino').value
        self.use_mock = self.get_parameter('use_mock').value
        
        self.ser = None
        
        # Initialize serial connection
        if self.use_arduino:
            if self.use_mock:
                self.setup_mock_serial()
            else:
                self.setup_real_serial()
        
        # Subscribe to egg detection topic
        self.subscription = self.create_subscription(
            Bool,
            'egg_detected',
            self.egg_detection_callback,
            10
        )
            
        self.get_logger().info("Navigation node initialized")
    
    def setup_mock_serial(self):
        try:
            from navigation.mock_serial import MockSerial
            self.ser = MockSerial(self.serial_port, self.baud_rate, timeout=1, node=self)
            self.get_logger().info(f"Using mock serial on {self.serial_port}")
        except ImportError as e:
            self.get_logger().error(f"Could not import mock_serial: {e}")
            self.ser = None
            
    def setup_real_serial(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino on {self.serial_port}: {e}")
            self.ser = None
    
    def egg_detection_callback(self, msg):
        if msg.data:
            self.get_logger().info("Egg detected! Stopping robot")
            self.stop_robot()
    
    def stop_robot(self):
        if self.ser:
            try:
                # Send stop command to Arduino
                self.ser.write(b'STOP\n')
                self.get_logger().info("Sent STOP command to Arduino")
            except Exception as e:
                self.get_logger().error(f"Failed to send command: {e}")
        else:
            self.get_logger().info("SIMULATION: Robot would stop now (no Arduino connected)")

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()