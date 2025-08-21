#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
import cv2
from scipy.optimize import least_squares

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32, TransformStamped, Quaternion, Pose
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import tf_transformations

class CalibratedCameraProcessor(Node):
    def __init__(self):
        super().__init__('calibrated_camera_processor')

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

        # Camera calibration parameters (from camera_info.yaml)
        self.camera_matrix = np.array([
            [877.027206, 0.000000, 308.821392],
            [0.000000, 872.797374, 251.479926],
            [0.000000, 0.000000, 1.000000]
        ])
        
        self.dist_coeffs = np.array([-0.031664, 0.650382, 0.004510, 0.000736, 0.000000])
        
        # Camera parameters
        self.image_width = 640
        self.image_height = 480
        
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
        self.min_calibration_points = 2  # Reduced since we have good camera calibration
        self.scale_factor = 0.013  # Default scale factor (calculated from landmarks)
        
        self.get_logger().info('Calibrated Camera Processor started')
        self.get_logger().info(f'Landmark tags configuration: {self.landmark_tags}')
        self.get_logger().info(f'Using calibrated camera matrix: {self.camera_matrix}')

    def detections_callback(self, msg):
        header = msg.header
        detected_landmarks = {}
        detected_robot_pose = None
        detected_table_tags = {}

        # Sort detected tags
        for detection in msg.detections:
            tag_id = str(detection.id)

            # Prefer metric pose from AprilTag (meters in camera frame)
            current_pose = Pose()
            try:
                # apriltag_msgs: PoseWithCovarianceStamped -> .pose.pose
                current_pose = detection.pose.pose.pose
            except Exception:
                # Fallback to pixel centre if pose not available
                current_pose.position.x = float(detection.centre.x)
                current_pose.position.y = float(detection.centre.y)
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

        # Establish map -> camera transform using calibrated camera
        if self.map_to_camera_transform is None and len(self.calibration_points) >= self.min_calibration_points:
            self.calculate_calibrated_transform(header) # Pass header

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

    def calculate_calibrated_transform(self, header):
        """Calculate transform using calibrated camera parameters"""
        if len(self.calibration_points) < self.min_calibration_points:
            self.get_logger().warn(f"Need at least {self.min_calibration_points} calibration points, have {len(self.calibration_points)}")
            return

        # Use the most reliable landmark (usually tag 100) for initial transform
        primary_landmark = None
        for point in self.calibration_points:
            if point['tag_id'] == '100':
                primary_landmark = point
                break
        
        if primary_landmark is None:
            primary_landmark = self.calibration_points[0]

        # Calculate initial transform using primary landmark
        camera_pos = primary_landmark['camera_pos']
        map_pos = primary_landmark['map_pos']
        
        # Calculate scale factor based on camera calibration
        # Using the distance between landmarks to calculate proper scale
        if len(self.calibration_points) >= 2:
            # Find two landmarks to calculate scale
            landmark1 = None
            landmark2 = None
            for point in self.calibration_points:
                if point['tag_id'] == '100':
                    landmark1 = point
                elif point['tag_id'] == '200':
                    landmark2 = point
            
            if landmark1 and landmark2:
                # Calculate distance between landmarks in camera frame
                cam_dist = math.sqrt((landmark2['camera_pos'][0] - landmark1['camera_pos'][0])**2 + 
                                   (landmark2['camera_pos'][1] - landmark1['camera_pos'][1])**2)
                # Calculate real distance between landmarks in map frame
                map_dist = math.sqrt((landmark2['map_pos'][0] - landmark1['map_pos'][0])**2 + 
                                   (landmark2['map_pos'][1] - landmark1['map_pos'][1])**2)
                
                if cam_dist > 1e-6:
                    # Calculate scale factor
                    scale_factor = map_dist / cam_dist
                    self.get_logger().info(f"Calculated scale factor: {scale_factor:.6f} (camera dist: {cam_dist:.3f}m, map dist: {map_dist:.3f}m)")
                else:
                    self.get_logger().warn("Landmarks have zero distance in camera frame, cannot calculate scale. Using fallback.")
                    scale_factor = 1.0 # Fallback for metric poses
            else:
                self.get_logger().warn("Could not find landmarks 100 and 200 for scaling. Using fallback.")
                scale_factor = 1.0  # Fallback for metric poses
        else:
            scale_factor = 1.0  # Fallback for metric poses
        
        # Calculate transform with scale
        tx = map_pos[0] - camera_pos[0] * scale_factor
        ty = map_pos[1] - camera_pos[1] * scale_factor
        
        # Create transform message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = header.frame_id or 'default_cam'
        
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = 0.0
        
        # Identity rotation (assuming camera is mounted straight down)
        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0

        self.map_to_camera_transform = t
        self.static_tf_broadcaster.sendTransform(self.map_to_camera_transform)
        
        self.get_logger().info(f"Calibrated transform calculated:")
        self.get_logger().info(f"  Translation: ({tx:.3f}, {ty:.3f})")
        self.get_logger().info(f"  Scale factor: {scale_factor}")
        
        # Store scale factor for later use
        self.scale_factor = scale_factor
        
        # Verify transform with all calibration points
        self.verify_calibrated_transform()

    def verify_calibrated_transform(self):
        """Verify the calculated transform by checking all calibration points"""
        if self.map_to_camera_transform is None:
            return
            
        total_error = 0.0
        for point in self.calibration_points:
            camera_pos = point['camera_pos']
            map_pos = point['map_pos']
            
            # Apply transform
            transformed_x = (camera_pos[0] * self.scale_factor) + self.map_to_camera_transform.transform.translation.x
            transformed_y = (camera_pos[1] * self.scale_factor) + self.map_to_camera_transform.transform.translation.y
            
            error = math.sqrt((transformed_x - map_pos[0])**2 + (transformed_y - map_pos[1])**2)
            total_error += error
            
            self.get_logger().info(f"Verification {point['tag_id']}: Expected({map_pos[0]:.3f}, {map_pos[1]:.3f}) -> Calculated({transformed_x:.3f}, {transformed_y:.3f}) -> Error: {error:.3f}m")
        
        avg_error = total_error / len(self.calibration_points)
        self.get_logger().info(f"Average transformation error: {avg_error:.3f}m")
        
        if avg_error > 0.1:  # If error is more than 10cm
            self.get_logger().warn(f"High transformation error detected: {avg_error:.3f}m. Consider recalibrating landmarks.")

    def publish_robot_transform(self, robot_pose_in_cam, header):
        """Publish robot transform using calibrated camera"""
        if self.map_to_camera_transform is None:
            return
            
        # Transform robot pose to map frame
        robot_pos = (robot_pose_in_cam.position.x, robot_pose_in_cam.position.y)
        
        # Apply calibrated transform (robot_pos already in meters if detection.pose is present)
        map_x = (robot_pos[0] * self.scale_factor) + self.map_to_camera_transform.transform.translation.x
        map_y = (robot_pos[1] * self.scale_factor) + self.map_to_camera_transform.transform.translation.y
        
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
        """Publish table obstacle using calibrated camera"""
        if self.map_to_camera_transform is None:
            return
            
        poses_in_map = {}
        for tag_id, pose_in_cam in table_tags.items():
            # Transform table tag to map frame (pose_in_cam expected in meters)
            tag_pos = (pose_in_cam.position.x, pose_in_cam.position.y)
            
            map_x = (tag_pos[0] * self.scale_factor) + self.map_to_camera_transform.transform.translation.x
            map_y = (tag_pos[1] * self.scale_factor) + self.map_to_camera_transform.transform.translation.y
            
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
    node = CalibratedCameraProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 