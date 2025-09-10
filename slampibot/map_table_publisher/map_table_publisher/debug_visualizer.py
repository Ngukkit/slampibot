#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA
from apriltag_msgs.msg import AprilTagDetectionArray
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import tf_transformations

class DebugVisualizer(Node):
    def __init__(self):
        super().__init__('debug_visualizer')
        
        # Subscribers
        self.detections_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detections_callback,
            10
        )
        
        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/debug_markers',
            10
        )
        
        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Marker array for visualization
        self.marker_array = MarkerArray()
        
        self.get_logger().info('Debug Visualizer started')
        
    def detections_callback(self, msg):
        self.marker_array.markers.clear()
        
        for i, detection in enumerate(msg.detections):
            # Create marker for detected tag
            marker = Marker()
            marker.header.frame_id = msg.header.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"tag_{detection.id}"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Set position
            marker.pose.position.x = detection.centre.x
            marker.pose.position.y = detection.centre.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Set scale and color
            marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            
            self.marker_array.markers.append(marker)
            
            # Try to transform to map frame if possible
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', 
                    msg.header.frame_id, 
                    rclpy.time.Time()
                )
                
                # Create pose stamped for transformation
                pose_stamped = self.create_pose_stamped(detection.centre, msg.header)
                
                # Manually perform the transformation using tf_transformations
                # Convert PoseStamped to a 4x4 matrix
                translation = [pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z]
                rotation = [pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w]
                pose_matrix = tf_transformations.quaternion_matrix(rotation)
                pose_matrix[0:3, 3] = translation

                # Convert TransformStamped to a 4x4 matrix
                transform_translation = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
                transform_rotation = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
                transform_matrix = tf_transformations.quaternion_matrix(transform_rotation)
                transform_matrix[0:3, 3] = transform_translation

                # Apply the transformation: T_map_tag = T_map_camera * T_camera_tag
                transformed_matrix = np.dot(transform_matrix, pose_matrix)

                # Extract transformed position
                transformed_x = transformed_matrix[0, 3]
                transformed_y = transformed_matrix[1, 3]
                transformed_z = transformed_matrix[2, 3]
                
                # Create marker in map frame
                map_marker = Marker()
                map_marker.header.frame_id = 'map'
                map_marker.header.stamp = self.get_clock().now().to_msg()
                map_marker.ns = f"tag_{detection.id}_map"
                map_marker.id = i + 1000  # Different ID range
                map_marker.type = Marker.SPHERE
                map_marker.action = Marker.ADD
                
                map_marker.pose.position.x = transformed_x
                map_marker.pose.position.y = transformed_y
                map_marker.pose.position.z = transformed_z
                map_marker.pose.orientation.w = 1.0 # Assuming no rotation for sphere marker
                
                map_marker.scale = Vector3(x=0.15, y=0.15, z=0.15)
                map_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
                
                self.marker_array.markers.append(map_marker)
                
                self.get_logger().info(f'Tag {detection.id}: Camera frame ({detection.centre.x:.3f}, {detection.centre.y:.3f}) -> Map frame ({transformed_x:.3f}, {transformed_y:.3f})')
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f'Could not transform tag {detection.id} to map frame: {e}')
        
        # Publish markers
        self.marker_pub.publish(self.marker_array)
    
    def create_pose_stamped(self, point, header):
        from geometry_msgs.msg import PoseStamped, Pose
        pose = Pose()
        pose.position.x = point.x
        pose.position.y = point.y
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        
        pose_stamped = PoseStamped()
        pose_stamped.header = header
        pose_stamped.pose = pose
        
        return pose_stamped

def main(args=None):
    rclpy.init(args=args)
    node = DebugVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 