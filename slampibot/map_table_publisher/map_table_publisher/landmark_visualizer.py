#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor

class LandmarkVisualizer(Node):
    def __init__(self):
        super().__init__('landmark_visualizer')
        self.get_logger().info('Landmark Visualizer node started.')

        # Declare parameters for landmark tags
        declared_tag_ids = [100, 200, 300, 400]
        for tag_id in declared_tag_ids:
            self.declare_parameter(f'landmark_tags.{tag_id}.x', 0.0)
            self.declare_parameter(f'landmark_tags.{tag_id}.y', 0.0)

        self.landmark_tags = {}
        for tag_id in declared_tag_ids:
            try:
                x = self.get_parameter(f'landmark_tags.{tag_id}.x').get_parameter_value().double_value
                y = self.get_parameter(f'landmark_tags.{tag_id}.y').get_parameter_value().double_value
                self.landmark_tags[tag_id] = {'x': x, 'y': y}
            except Exception as e:
                self.get_logger().warn(f"Landmark parameters for tag {tag_id} not fully defined or other error: {e}")

        if not self.landmark_tags:
            self.get_logger().error("Landmark tags parameter is not set or empty!")
            return

        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, '/landmark_markers', latching_qos)
        self.timer = self.create_timer(1.0, self.publish_markers)

    def publish_markers(self):
        if not self.landmark_tags:
            self.get_logger().warn('No landmark tags found. Retrying...')
            return

        marker_array = MarkerArray()
        for tag_id, coords in self.landmark_tags.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "ideal_landmarks"
            marker.id = tag_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = coords['x']
            marker.pose.position.y = coords['y']
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 0.8
            marker.color.r = 1.0
            marker.color.g = 0.7
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(marker_array.markers)} landmark markers.')
        self.timer.cancel() # Publish only once

def main(args=None):
    rclpy.init(args=args)
    node = LandmarkVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()