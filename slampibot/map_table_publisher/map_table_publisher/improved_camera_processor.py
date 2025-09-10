#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
from scipy.optimize import least_squares

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32, TransformStamped, Quaternion, Pose
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import tf_transformations

class ImprovedCameraProcessor(Node):
    def __init__(self):
        super().__init__('improved_camera_processor')

        # Declare and get parameters
        self.declare_parameter('robot_tag_id', 0)
        self.declare_parameter('table_tag_ids', [1, 2])
        self.declare_parameter('landmark_tags.100.x', 0.0)
        self.declare_parameter('landmark_tags.100.y', 0.0)
        self.declare_parameter('landmark_tags.200.x', 5.0)
        self.declare_parameter('landmark_tags.200.y', 0.0)
        self.declare_parameter('landmark_tags.300.x', 2.5)
        self.declare_parameter('landmark_tags.300.y', 4.0)

        self.robot_tag_id = self.get_parameter('robot_tag_id').get_parameter_value().integer_value
        self.table_tag_ids = self.get_parameter('table_tag_ids').get_parameter_value().integer_array_value
        
        self.landmark_tags = {}
        self.landmark_tags['100'] = (self.get_parameter('landmark_tags.100.x').get_parameter_value().double_value, self.get_parameter('landmark_tags.100.y').get_parameter_value().double_value)
        self.landmark_tags['200'] = (self.get_parameter('landmark_tags.200.x').get_parameter_value().double_value, self.get_parameter('landmark_tags.200.y').get_parameter_value().double_value)
        self.landmark_tags['300'] = (self.get_parameter('landmark_tags.300.x').get_parameter_value().double_value, self.get_parameter('landmark_tags.300.y').get_parameter_value().double_value)

        # TF components
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Subscribers and Publishers
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detections_callback,
            10)
        self.table_pub = self.create_publisher(PolygonStamped, '/table_obstacle', 10)

        # Calibration state
        self.map_to_camera_transform = None
        self.calibration_points = []
        self.min_calibration_points = 3
        
        self.get_logger().info('Improved Camera Processor started')
        self.get_logger().info(f'Landmark tags configuration: {self.landmark_tags}')

    def detections_callback(self, msg):
        header = msg.header
        detected_landmarks = {}
        detected_robot_pose = None
        detected_table_tags = {}

        # Sort detected tags
        for detection in msg.detections:
            tag_id = str(detection.id)

            # Create pose from detection
            current_pose = Pose()
            current_pose.position.x = detection.centre.x
            current_pose.position.y = detection.centre.y
            current_pose.position.z = 0.0
            current_pose.orientation.w = 1.0

            if tag_id in self.landmark_tags:
                detected_landmarks[tag_id] = current_pose
                self.get_logger().info(f"Landmark {tag_id} detected at: ({current_pose.position.x:.3f}, {current_pose.position.y:.3f})")
            elif detection.id == self.robot_tag_id:
                detected_robot_pose = current_pose
                self.get_logger().info(f"Robot tag {detection.id} detected at: ({current_pose.position.x:.3f}, {current_pose.position.y:.3f})")
            elif detection.id in self.table_tag_ids:
                detected_table_tags[detection.id] = current_pose
                self.get_logger().info(f"Table tag {detection.id} detected at: ({current_pose.position.x:.3f}, {current_pose.position.y:.3f})")

        # Update calibration data
        self.update_calibration_data(detected_landmarks, header)

        # Establish map -> camera transform using multiple landmarks
        if self.map_to_camera_transform is None and len(self.calibration_points) >= self.min_calibration_points:
            self.calculate_improved_transform()

        # Process robot and table poses
        if self.map_to_camera_transform is not None:
            if detected_robot_pose:
                self.publish_robot_transform(detected_robot_pose, header)
            
            if len(detected_table_tags) == 2:
                self.publish_table_obstacle(detected_table_tags, header)

    def update_calibration_data(self, detected_landmarks, header):
        """Update calibration data with new landmark detections"""
        for tag_id, pose_in_cam in detected_landmarks.items():
            if tag_id in self.landmark_tags:
                known_pos = self.landmark_tags[tag_id]
                
                # Check if we already have this tag in calibration data
                existing = False
                for point in self.calibration_points:
                    if point['tag_id'] == tag_id:
                        # Update existing point
                        point['camera_pos'] = (pose_in_cam.position.x, pose_in_cam.position.y)
                        point['timestamp'] = header.stamp
                        existing = True
                        break
                
                if not existing:
                    # Add new calibration point
                    self.calibration_points.append({
                        'tag_id': tag_id,
                        'map_pos': known_pos,
                        'camera_pos': (pose_in_cam.position.x, pose_in_cam.position.y),
                        'timestamp': header.stamp
                    })
                
                self.get_logger().info(f"Calibration point {tag_id}: Map({known_pos[0]:.3f}, {known_pos[1]:.3f}) -> Camera({pose_in_cam.position.x:.3f}, {pose_in_cam.position.y:.3f})")

    def calculate_improved_transform(self):
        """Calculate transform using least squares optimization with multiple landmarks"""
        if len(self.calibration_points) < self.min_calibration_points:
            self.get_logger().warn(f"Need at least {self.min_calibration_points} calibration points, have {len(self.calibration_points)}")
            return

        # Extract camera and map positions
        camera_positions = np.array([point['camera_pos'] for point in self.calibration_points])
        map_positions = np.array([point['map_pos'] for point in self.calibration_points])

        # Initial guess for transform parameters [tx, ty, scale, rotation]
        initial_params = [0.0, 0.0, 1.0, 0.0]

        # Optimize transform parameters
        result = least_squares(self.transform_error, initial_params, 
                             args=(camera_positions, map_positions))

        if result.success:
            tx, ty, scale, rotation = result.x
            
            # Create transform matrix
            cos_r = math.cos(rotation)
            sin_r = math.sin(rotation)
            
            transform_matrix = np.array([
                [scale * cos_r, -scale * sin_r, tx],
                [scale * sin_r, scale * cos_r, ty],
                [0, 0, 1]
            ])

            # Create transform message
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'ceiling_camera_link'
            
            t.transform.translation.x = tx
            t.transform.translation.y = ty
            t.transform.translation.z = 0.0
            
            # Convert rotation to quaternion
            quat = tf_transformations.quaternion_from_euler(0, 0, rotation)
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]

            self.map_to_camera_transform = t
            self.static_tf_broadcaster.sendTransform(self.map_to_camera_transform)
            
            self.get_logger().info(f"Improved transform calculated:")
            self.get_logger().info(f"  Translation: ({tx:.3f}, {ty:.3f})")
            self.get_logger().info(f"  Scale: {scale:.3f}")
            self.get_logger().info(f"  Rotation: {math.degrees(rotation):.1f} degrees")
            
            # Verify transform
            self.verify_transform()
        else:
            self.get_logger().error("Failed to calculate improved transform")

    def transform_error(self, params, camera_positions, map_positions):
        """Error function for least squares optimization"""
        tx, ty, scale, rotation = params
        
        cos_r = math.cos(rotation)
        sin_r = math.sin(rotation)
        
        # Apply transform to camera positions
        transformed = np.zeros_like(camera_positions)
        transformed[:, 0] = scale * (cos_r * camera_positions[:, 0] - sin_r * camera_positions[:, 1]) + tx
        transformed[:, 1] = scale * (sin_r * camera_positions[:, 0] + cos_r * camera_positions[:, 1]) + ty
        
        # Return error (difference between transformed and map positions)
        return (transformed - map_positions).flatten()

    def verify_transform(self):
        """Verify the calculated transform by checking all calibration points"""
        if self.map_to_camera_transform is None:
            return
            
        total_error = 0.0
        for point in self.calibration_points:
            # Transform camera position to map
            camera_pos = point['camera_pos']
            map_pos = point['map_pos']
            
            # Apply transform
            transformed_x = (self.map_to_camera_transform.transform.translation.x + 
                           camera_pos[0] * math.cos(0) - camera_pos[1] * math.sin(0))
            transformed_y = (self.map_to_camera_transform.transform.translation.y + 
                           camera_pos[0] * math.sin(0) + camera_pos[1] * math.cos(0))
            
            error = math.sqrt((transformed_x - map_pos[0])**2 + (transformed_y - map_pos[1])**2)
            total_error += error
            
            self.get_logger().info(f"Verification {point['tag_id']}: Expected({map_pos[0]:.3f}, {map_pos[1]:.3f}) -> Calculated({transformed_x:.3f}, {transformed_y:.3f}) -> Error: {error:.3f}m")
        
        avg_error = total_error / len(self.calibration_points)
        self.get_logger().info(f"Average transformation error: {avg_error:.3f}m")

    def publish_robot_transform(self, robot_pose_in_cam, header):
        """Publish robot transform using improved calibration"""
        if self.map_to_camera_transform is None:
            return
            
        # Transform robot pose to map frame
        robot_pos = (robot_pose_in_cam.position.x, robot_pose_in_cam.position.y)
        
        # Apply transform
        map_x = (self.map_to_camera_transform.transform.translation.x + robot_pos[0])
        map_y = (self.map_to_camera_transform.transform.translation.y + robot_pos[1])
        
        self.get_logger().info(f"Robot in map frame: ({map_x:.3f}, {map_y:.3f})")
        
        # Publish transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'dummy_link'
        
        t.transform.translation.x = map_x
        t.transform.translation.y = map_y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)

    def publish_table_obstacle(self, table_tags, header):
        """Publish table obstacle using improved calibration"""
        if self.map_to_camera_transform is None:
            return
            
        poses_in_map = {}
        for tag_id, pose_in_cam in table_tags.items():
            # Transform table tag to map frame
            tag_pos = (pose_in_cam.position.x, pose_in_cam.position.y)
            
            map_x = (self.map_to_camera_transform.transform.translation.x + tag_pos[0])
            map_y = (self.map_to_camera_transform.transform.translation.y + tag_pos[1])
            
            poses_in_map[tag_id] = (map_x, map_y)
            self.get_logger().info(f"Table tag {tag_id} in map frame: ({map_x:.3f}, {map_y:.3f})")

        # Create polygon
        tag_ids = list(poses_in_map.keys())
        (x1, y1) = poses_in_map[tag_ids[0]]
        (x2, y2) = poses_in_map[tag_ids[1]]

        # Calculate table properties
        table_width = math.sqrt((x1 - x2)**2 + (y1 - y2)**2) / math.sqrt(2)
        table_height = table_width

        poly = PolygonStamped()
        poly.header.frame_id = 'map'
        poly.header.stamp = self.get_clock().now().to_msg()

        dx = table_width / 2
        dy = table_height / 2

        # Calculate center and orientation
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        yaw = math.atan2(y2 - y1, x2 - x1)

        self.get_logger().info(f"Table center: ({cx:.3f}, {cy:.3f}), size: {table_width:.3f}x{table_height:.3f}, yaw: {math.degrees(yaw):.1f}°")

        # Create polygon corners
        corners = [(-dx, -dy), (dx, -dy), (dx, dy), (-dx, dy)]
        for px, py in corners:
            rx = px * math.cos(yaw) - py * math.sin(yaw)
            ry = px * math.sin(yaw) + py * math.cos(yaw)
            poly.polygon.points.append(Point32(x=cx + rx, y=cy + ry, z=0.0))

        self.table_pub.publish(poly)

def main(args=None):
    rclpy.init(args=args)
    node = ImprovedCameraProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 