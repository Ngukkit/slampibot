import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo

class QosRelayNode(Node):
    def __init__(self):
        super().__init__('qos_relay_node')
        self.get_logger().info('Initializing QoS Relay Node...')

        # Define QoS profiles
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100  # Match the publisher's depth
        )
        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10  # Match the subscriber's expected depth
        )

        self.get_logger().info('Creating relay for Image topic...')
        # Relay for Image topic
        self.image_sub = self.create_subscription(
            Image,
            '/ceiling_camera/image_raw',
            self.image_callback,
            qos_profile=qos_reliable
        )
        self.image_pub = self.create_publisher(
            Image,
            '/image_relay/image_raw',
            qos_profile=qos_reliable
        )
        self.get_logger().info('Relay for Image topic created.')

        self.get_logger().info('Creating relay for CameraInfo topic...')
        # Relay for CameraInfo topic
        self.camerainfo_sub = self.create_subscription(
            CameraInfo,
            '/ceiling_camera/camera_info',
            self.camerainfo_callback,
            qos_profile=qos_reliable
        )
        self.camerainfo_pub = self.create_publisher(
            CameraInfo,
            '/image_relay/camera_info',
            qos_profile=qos_reliable
        )
        self.get_logger().info('Relay for CameraInfo topic created.')

        self.get_logger().info('QoS Relay Node has been started.')
        self.get_logger().info('Relaying /ceiling_camera/image_raw (RELIABLE) -> /image_relay/image_raw (RELIABLE)')
        self.get_logger().info('Relaying /ceiling_camera/camera_info (RELIABLE) -> /image_relay/camera_info (RELIABLE)')


    def image_callback(self, msg):
        self.get_logger().info(f'Received image message, relaying... (size: {len(msg.data)} bytes)')
        self.image_pub.publish(msg)

    def camerainfo_callback(self, msg):
        self.get_logger().info('Received camera_info message, relaying...')
        self.camerainfo_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = QosRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
