#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from sklearn import linear_model
from typing import Tuple, List

class GreenLineDetector(Node):
    def __init__(self):
        super().__init__('green_line_detector')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Subscriber for the input image
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/pure_image',
            self.image_callback,
            10)
        
        # Publisher for the output image with lines
        self.publisher = self.create_publisher(
            Image,
            '/camera/depth/lines',
            10)
        
        self.get_logger().info('Green Line Detector node has been initialized')

    def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect green pixels
            green_pixels = self.detect_green_pixels(cv_image)
            
            if green_pixels is None or len(green_pixels) == 0:
                self.get_logger().info('No green pixels detected')
                return
            
            # Split green pixels into left and right groups relative to center
            height, width = cv_image.shape[:2]
            center_x = width // 2
            center_y = height * 0.75
            left_pixels = [p for p in green_pixels if (p[0] < center_x and p[1] < center_y) ]
            right_pixels = [p for p in green_pixels if (p[0] >= center_x and p[1] < center_y)]
            
            # Perform RANSAC regression on both groups
            lines = []
            if len(left_pixels) > 1:
                left_line = self.ransac_line_fit(left_pixels)
                lines.append(('left', left_line))
            
            if len(right_pixels) > 1:
                right_line = self.ransac_line_fit(right_pixels)
                lines.append(('right', right_line))
            
            # Draw the detected lines on the original image
            result_image = self.draw_lines(cv_image, lines, center_x)
            
            # Convert back to ROS image message and publish
            output_msg = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')
            self.publisher.publish(output_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def detect_green_pixels(self, image: np.ndarray) -> List[Tuple[int, int]]:
        """Detect green pixels in the image and return their coordinates."""
        # Convert to HSV color space for better color segmentation
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define range for green color in HSV
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])
        
        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Find coordinates of all non-zero pixels (green pixels)
        y_coords, x_coords = np.where(mask > 0)
        green_pixels = list(zip(x_coords, y_coords))
        
        return green_pixels

    def ransac_line_fit(self, pixels: List[Tuple[int, int]]) -> Tuple[float, float]:
        """Fit a line to the pixels using RANSAC algorithm."""
        # Convert to numpy array
        X = np.array([p[0] for p in pixels]).reshape(-1, 1)
        y = np.array([p[1] for p in pixels])
        
        # Robustly fit linear model with RANSAC algorithm
        ransac = linear_model.RANSACRegressor()
        ransac.fit(X, y)
        
        # Get line parameters (slope and intercept)
        slope = ransac.estimator_.coef_[0]
        intercept = ransac.estimator_.intercept_
        
        return (slope, intercept)

    def draw_lines(self, image: np.ndarray, lines: List[Tuple[str, Tuple[float, float]]], center_x: int) -> np.ndarray:
        """Draw the detected lines on the image."""
        result = image.copy()
        height, width = image.shape[:2]
        
        for side, (slope, intercept) in lines:
            # Choose color based on side
            color = (0, 0, 255) if side == 'left' else (255, 0, 0)  # Red for left, Blue for right
            
            # Calculate two points for the line
            if side == 'left':
                x1 = 0
                x2 = center_x
            else:
                x1 = center_x
                x2 = width
            
            y1 = int(slope * x1 + intercept)
            y2 = int(slope * x2 + intercept)
            
            # Draw the line
            cv2.line(result, (x1, y1), (x2, y2), color, 2)
        
        return result

def main(args=None):
    rclpy.init(args=args)
    node = GreenLineDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()