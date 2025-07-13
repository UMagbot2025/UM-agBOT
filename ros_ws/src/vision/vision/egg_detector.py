#!/usr/bin/env python3
import os
import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pypylon import pylon

class EggDetectionNode(Node):
    def __init__(self):
        super().__init__('egg_detection_node')
        
        # Create publisher for egg detections
        self.detection_pub = self.create_publisher(Bool, 'egg_detected', 10)
        
        # Parameter declarations
        self.declare_parameter('model_path', '/workspace/models/agbot_yolo_nano.onnx')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('min_box_area', 500.0)
        self.declare_parameter('input_size', 640)
        self.declare_parameter('use_basler', True)
        self.declare_parameter('use_mock', False)
        self.declare_parameter('test_images_dir', '')
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.min_box_area = self.get_parameter('min_box_area').value
        self.input_size = self.get_parameter('input_size').value
        self.use_basler = self.get_parameter('use_basler').value
        self.use_mock = self.get_parameter('use_mock').value
        self.test_images_dir = self.get_parameter('test_images_dir').value
        
        # Load ONNX model
        self.get_logger().info(f"Loading model from {self.model_path}")
        self.session = ort.InferenceSession(self.model_path)
        
        # CV Bridge for ROS image conversion
        self.bridge = CvBridge()
        
        # Setup camera based on parameter
        if self.use_mock:
            self.setup_mock_camera()
        elif self.use_basler:
            self.setup_basler_camera()
        else:
            self.setup_webcam()
            
        # Create timer for processing frames
        self.timer = self.create_timer(0.1, self.process_frame)
    
    def setup_mock_camera(self):
        # Import the mock camera module
        try:
            from vision.mock_camera import MockBaslerCamera, MockImageConverter
            self.get_logger().info("Setting up mock Basler camera")
            
            # Create mock Basler camera
            self.camera = MockBaslerCamera(self, self.test_images_dir)
            self.camera.Open()
            
            # Set up mock image converter
            self.converter = MockImageConverter()
            
            # Start grabbing images
            self.camera.StartGrabbing("LatestImageOnly")
            self.get_logger().info("Mock Basler camera initialized")
            self.camera_type = "basler"
            
        except ImportError as e:
            self.get_logger().error(f"Failed to import mock camera: {e}")
            self.setup_webcam()
            
    def setup_basler_camera(self):
        try:
            from pypylon import pylon
            # Connect to the first available Basler camera
            self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
            self.camera.Open()
            
            # Set up the image converter for OpenCV
            self.converter = pylon.ImageFormatConverter()
            self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
            self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
            
            # Start grabbing images continuously
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
            self.get_logger().info("Basler camera initialized")
            self.camera_type = "basler"
            
        except ImportError:
            self.get_logger().error("pypylon is not installed. Falling back to webcam.")
            self.setup_webcam()
    
    def setup_webcam(self):
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam")
            return
        self.get_logger().info("Webcam initialized")
        self.camera_type = "webcam"
    
    def process_frame(self):
        if self.camera_type == "basler":
            if not self.camera.IsGrabbing():
                return
            
            grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if grabResult.GrabSucceeded():
                # Convert to OpenCV BGR format
                image = self.converter.Convert(grabResult)
                frame = image.GetArray()
                self.process_detection(frame)
                grabResult.Release()
        else:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to grab frame")
                return
            self.process_detection(frame)
    
    def process_detection(self, frame):
        # Run detection
        num, xywh_list, scores = self.get_detections(frame)
        
        # Draw boxes (optional - for visualization)
        for xywh, score in zip(xywh_list, scores):
            x, y, w, h = map(int, xywh)
            x1, y1 = int(x - w/2), int(y - h/2)
            x2, y2 = int(x + w/2), int(y + h/2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            
        # Publish detection result
        if num > 0:
            self.get_logger().info(f"Egg detected with confidence: {scores[0]}")
            msg = Bool()
            msg.data = True
            self.detection_pub.publish(msg)
    
    def letterbox(self, img, new_shape=(640, 640), color=(114, 114, 114)):
        shape = img.shape[:2]  # current shape [height, width]
        ratio = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        new_unpad = (int(round(shape[1] * ratio)), int(round(shape[0] * ratio)))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
        dw /= 2
        dh /= 2
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
        return img, ratio, (dw, dh)
    
    def preprocess_image(self, frame):
        img, ratio, dwdh = self.letterbox(frame, (self.input_size, self.input_size))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC to CHW
        img = np.expand_dims(img, axis=0)   # Add batch dimension
        return img, ratio, dwdh
    
    def run_inference(self, img):
        ort_inputs = {self.session.get_inputs()[0].name: img}
        outputs = self.session.run(None, ort_inputs)
        return outputs[0]
    
    def nms(self, boxes, scores, iou_threshold=0.5):
        boxes_xyxy = []
        for x, y, w, h in boxes:
            x1 = x - w / 2
            y1 = y - h / 2
            x2 = x + w / 2
            y2 = y + h / 2
            boxes_xyxy.append([x1, y1, x2, y2])
        indices = cv2.dnn.NMSBoxes(
            bboxes=boxes_xyxy,
            scores=scores,
            score_threshold=0.0,
            nms_threshold=iou_threshold
        )
        indices = indices.flatten() if len(indices) > 0 else []
        return indices
    
    def postprocess_detections(self, output, ratio, dwdh):
        if output.shape[1] == 5:
            output = np.transpose(output, (0, 2, 1))
        detections = output[0]

        boxes = []
        scores = []
        for det in detections:
            x, y, w, h, conf = det[:5]
            if conf >= self.confidence_threshold:
                # Undo letterbox and resize to get original image coordinates
                x = (x - dwdh[0]) / ratio
                y = (y - dwdh[1]) / ratio
                w = w / ratio
                h = h / ratio
                area = w * h
                if area >= self.min_box_area:
                    boxes.append([x, y, w, h])
                    scores.append(float(conf))

        if boxes:
            indices = self.nms(boxes, scores, iou_threshold=0.5)
            boxes = [boxes[i] for i in indices]
            scores = [scores[i] for i in indices]
        else:
            boxes = []
            scores = []

        num_detections = len(boxes)
        return num_detections, boxes, scores
    
    def get_detections(self, frame):
        img, ratio, dwdh = self.preprocess_image(frame)
        output = self.run_inference(img)
        return self.postprocess_detections(output, ratio, dwdh)

def main(args=None):
    rclpy.init(args=args)
    node = EggDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()