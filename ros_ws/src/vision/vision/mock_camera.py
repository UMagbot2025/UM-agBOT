#!/usr/bin/env python3
import cv2
import numpy as np
import os
import time
from rclpy.node import Node

class MockBaslerCamera:
    """A mock implementation of the Basler camera for testing"""
    
    def __init__(self, node: Node, test_images_dir=None):
        self.node = node
        self.is_grabbing = True
        
        # Use test images if provided, otherwise generate synthetic images
        self.test_images_dir = test_images_dir
        self.test_images = []
        self.current_image_idx = 0
        
        # Create or load test images
        if test_images_dir and os.path.exists(test_images_dir):
            self.node.get_logger().info(f"Loading test images from {test_images_dir}")
            image_files = [f for f in os.listdir(test_images_dir) 
                          if f.endswith(('.jpg', '.jpeg', '.png'))]
            self.test_images = [os.path.join(test_images_dir, f) for f in image_files]
            if not self.test_images:
                self.node.get_logger().warn("No test images found, will generate synthetic images")
        
        if not self.test_images:
            self.node.get_logger().info("Using synthetic test images")
        
        self.node.get_logger().info("Mock Basler camera initialized")
    
    def Open(self):
        self.node.get_logger().info("Mock Basler camera opened")
    
    def StartGrabbing(self, strategy):
        self.is_grabbing = True
        self.node.get_logger().info("Mock Basler camera started grabbing")
    
    def IsGrabbing(self):
        return self.is_grabbing
    
    def RetrieveResult(self, timeout, exception_handling):
        # Wait a bit to simulate camera frame rate
        time.sleep(0.05)
        return MockGrabResult(self)
    
    def Close(self):
        self.is_grabbing = False
        self.node.get_logger().info("Mock Basler camera closed")
    
    def _get_next_image(self):
        """Get the next image, either from test files or generate one"""
        if self.test_images:
            # Cycle through test images
            image_path = self.test_images[self.current_image_idx]
            self.current_image_idx = (self.current_image_idx + 1) % len(self.test_images)
            return cv2.imread(image_path)
        else:
            # Generate a synthetic image with a potential egg
            return self._generate_synthetic_image()
    
    def _generate_synthetic_image(self):
        """Generate a synthetic test image, sometimes with an egg"""
        # Create empty image (640x480)
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Add some background texture
        img += np.random.randint(30, 60, img.shape, dtype=np.uint8)
        
        # Randomly decide if this frame has an egg (30% chance)
        if np.random.random() < 0.3:
            # Add a circular egg-like object
            center_x = np.random.randint(100, 540)
            center_y = np.random.randint(100, 380)
            radius = np.random.randint(20, 40)
            
            # Draw egg (yellowish-white circle)
            cv2.circle(img, (center_x, center_y), radius, (200, 230, 255), -1)
            
            # Add some texture to the egg
            noise = np.random.randint(0, 30, (2*radius, 2*radius, 3), dtype=np.uint8)
            roi = img[center_y-radius:center_y+radius, center_x-radius:center_x+radius]
            roi_shape = roi.shape
            noise_resized = cv2.resize(noise, (roi_shape[1], roi_shape[0]))
            roi += noise_resized
        
        return img

class MockGrabResult:
    def __init__(self, camera):
        self.camera = camera
        self.image = camera._get_next_image()
    
    def GrabSucceeded(self):
        return True
    
    def Release(self):
        pass

class MockImageConverter:
    def __init__(self):
        self.OutputPixelFormat = None
        self.OutputBitAlignment = None
    
    def Convert(self, grab_result):
        return MockImage(grab_result.image)

class MockImage:
    def __init__(self, image):
        self.image = image
    
    def GetArray(self):
        return self.image