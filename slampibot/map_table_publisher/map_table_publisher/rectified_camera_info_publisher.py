import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import numpy as np

class RectifiedCameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('rectified_camera_info_publisher')
        self.subscription = self.create_subscription(
            CameraInfo,
            '/ceiling_camera/camera_info',
            self.camera_info_callback,
            10)
        self.publisher_ = self.create_publisher(
            CameraInfo,
            '/rectify/camera_info',
            10)
        self.get_logger().info('Rectified Camera Info Publisher started.')

    def camera_info_callback(self, msg):
        rectified_info = CameraInfo()
        rectified_info.header = msg.header
        rectified_info.width = msg.width
        rectified_info.height = msg.height

        # Explicitly set frame_id to match the image frame_id
        rectified_info.header.frame_id = 'ceiling_camera_link' 
        rectified_info.distortion_model = msg.distortion_model # Copy distortion_model
        rectified_info.d = msg.d # Copy distortion coefficients

        # The P matrix (projection matrix) from the original CameraInfo
        # contains the new camera matrix (K') for the rectified image.
        # We extract the 3x3 part of P for the K matrix.
        rectified_info.k = [
            msg.p[0], msg.p[1], msg.p[2],
            msg.p[4], msg.p[5], msg.p[6],
            msg.p[8], msg.p[9], msg.p[10]
        ]

        # Rectification matrix (R) should be identity for a rectified image.
        rectified_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # The P matrix itself is copied directly.
        rectified_info.p = msg.p

        self.publisher_.publish(rectified_info)

def main(args=None):
    rclpy.init(args=args)
    node = RectifiedCameraInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
