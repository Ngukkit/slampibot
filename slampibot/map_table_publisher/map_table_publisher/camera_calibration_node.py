#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from apriltag_msgs.msg import AprilTagDetectionArray
import tf_transformations

class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_calibration_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/ceiling_camera/image_raw',
            self.image_callback,
            10
        )
        
        self.detections_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detections_callback,
            10
        )
        
        # Publishers
        self.calibrated_image_pub = self.create_publisher(
            Image,
            '/ceiling_camera/calibrated_image',
            10
        )
        
        # Calibration parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.calibration_data = []
        
        self.get_logger().info('Camera Calibration Node started')
        
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Apply calibration if available
            if self.camera_matrix is not None and self.dist_coeffs is not None:
                calibrated_image = cv2.undistort(cv_image, self.camera_matrix, self.dist_coeffs)
            else:
                calibrated_image = cv_image
            
            # Publish calibrated image
            calibrated_msg = self.bridge.cv2_to_imgmsg(calibrated_image, "bgr8")
            calibrated_msg.header = msg.header
            self.calibrated_image_pub.publish(calibrated_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def detections_callback(self, msg):
        for detection in msg.detections:
            tag_id = detection.id
            corners = detection.corners
            
            # Log detection information for calibration
            self.get_logger().info(f'Tag {tag_id} detected at center: ({detection.centre.x:.2f}, {detection.centre.y:.2f})')
            
            # Store calibration data
            self.calibration_data.append({
                'tag_id': tag_id,
                'corners': corners,
                'centre': detection.centre
            })
    
    def calibrate_camera(self, object_points, image_points, image_size):
        """Perform camera calibration using detected AprilTags"""
        try:
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                object_points, image_points, image_size, None, None
            )
            
            if ret:
                self.camera_matrix = mtx
                self.dist_coeffs = dist
                self.get_logger().info('Camera calibration successful')
                self.get_logger().info(f'Camera matrix:\n{mtx}')
                self.get_logger().info(f'Distortion coefficients: {dist.flatten()}')
            else:
                self.get_logger().warn('Camera calibration failed')
                
        except Exception as e:
            self.get_logger().error(f'Error during calibration: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 