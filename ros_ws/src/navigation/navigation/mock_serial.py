#!/usr/bin/env python3
import time
from rclpy.node import Node

class MockSerial:
    """A mock implementation of the serial port for testing"""
    
    def __init__(self, port, baudrate, timeout, node: Node):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.node = node
        self.is_open = True
        self.received_data = []
        self.node.get_logger().info(f"Mock serial port {port} opened with baudrate {baudrate}")
    
    def write(self, data):
        """Simulate writing to the serial port"""
        if isinstance(data, bytes):
            command = data.decode('utf-8').strip()
        else:
            command = str(data).strip()
            
        self.node.get_logger().info(f"MOCK ARDUINO: Received command: {command}")
        self.received_data.append(command)
        
        # Simulate Arduino response
        if command == "STOP":
            self.node.get_logger().info("MOCK ARDUINO: Motors stopped")
            return len(data)
        elif command == "RESUME":
            self.node.get_logger().info("MOCK ARDUINO: Motors resumed")
            return len(data)
        
        return len(data)
    
    def read(self, size=1):
        """Simulate reading from the serial port"""
        # Simulate delay
        time.sleep(0.01)
        
        # Return mock response based on most recent command
        if self.received_data:
            last_command = self.received_data[-1]
            if last_command == "STOP":
                return b"Robot stopped\r\n"[:size]
            elif last_command == "RESUME":
                return b"Robot resumed\r\n"[:size]
        
        # Default - no data
        return b""
    
    def close(self):
        """Close the mock serial port"""
        self.is_open = False
        self.node.get_logger().info(f"Mock serial port {self.port} closed")