#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
import cv2
import tf2_ros
# from tf2_geometry_msgs import do_transform_pose
import tf_transformations

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose

class MapAligner(Node):
    def __init__(self):
        super().__init__('map_aligner')

        self.get_logger().info("MapAligner node started.")

        # Declare parameters for landmark tags (actual positions in Nav2 map frame)
        # These should match the landmark_tags.yaml values
        self.declare_parameter('landmark_tags.100.x', 0.0)
        self.declare_parameter('landmark_tags.100.y', 0.0)
        self.declare_parameter('landmark_tags.200.x', 0.0)
        self.declare_parameter('landmark_tags.200.y', 0.0)
        self.declare_parameter('landmark_tags.300.x', 0.0)
        self.declare_parameter('landmark_tags.300.y', 0.0)
        self.declare_parameter('landmark_tags.400.x', 0.0)
        self.declare_parameter('landmark_tags.400.y', 0.0)

        # Get landmark tags parameters (actual positions in Nav2 map frame)
        self.actual_landmark_pos = {}
        landmark_params = self.get_parameters_by_prefix('landmark_tags')
        for param_name, param_value in landmark_params.items():
            parts = param_name.split('.')
            if len(parts) == 2:
                tag_id_str, coord = parts
                try:
                    tag_id = int(tag_id_str)
                except ValueError:
                    self.get_logger().warn(f"Invalid landmark tag ID format: {tag_id_str}")
                    continue

                if tag_id not in self.actual_landmark_pos:
                    self.actual_landmark_pos[tag_id] = [0.0, 0.0]
                
                if coord == 'x':
                    self.actual_landmark_pos[tag_id][0] = param_value.value
                elif coord == 'y':
                    self.actual_landmark_pos[tag_id][1] = param_value.value
        
        # Convert list to tuple for consistency
        for tag_id in self.actual_landmark_pos:
            self.actual_landmark_pos[tag_id] = tuple(self.actual_landmark_pos[tag_id])

        self.get_logger().info(f"Actual landmark positions (Nav2 map): {self.actual_landmark_pos}")

        # Camera calibration parameters (hardcoded for now, ideally from camera_info.yaml)
        self.camera_matrix = np.array([
            [877.027206, 0.000000, 308.821392],
            [0.000000, 872.797374, 251.479926],
            [0.000000, 0.000000, 1.000000]
        ])
        self.tag_size = 0.05  # From apriltag_node parameter
        self.declare_parameter('camera_height', 2.0)
        self.camera_height = self.get_parameter('camera_height').get_parameter_value().double_value

        # TF components
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Subscriber for AprilTag detections
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.detections_callback,
            10)
        
        self.detected_landmark_poses = {} # Store latest detected poses in ceiling_camera_link
        self.min_landmarks_for_alignment = 4 # Need all 4 landmarks for homography
        self.alignment_published = False # Flag to publish transform only once

    def detections_callback(self, msg):
        if self.alignment_published:
            return # Only align once

        current_detected_landmarks = {}
        for detection in msg.detections:
            tag_id = detection.id
            if tag_id in self.actual_landmark_pos: # Only consider our landmark tags
                current_pose = None
                # Try to get the 3D pose from the detection's pose field
                if hasattr(detection, 'pose') and detection.pose is not None and hasattr(detection.pose, 'pose') and detection.pose.pose is not None:
                    current_pose = detection.pose.pose
                else:
                    # If pose is not available, try to compute it from homography
                    if hasattr(detection, 'homography') and detection.homography is not None:
                        try:
                            fx = self.camera_matrix[0, 0]
                            fy = self.camera_matrix[1, 1]
                            cx = self.camera_matrix[0, 2]
                            cy = self.camera_matrix[1, 2]

                            u = detection.centre.x
                            v = detection.centre.y

                            x_norm = (u - cx) / fx
                            y_norm = (v - cy) / fy

                            # This is a very rough approximation, assuming a fixed Z distance
                            z_est = self.camera_height  # Assume fixed camera height
                            x_est = x_norm * z_est
                            y_est = y_norm * z_est

                            current_pose = Pose()
                            current_pose.position.x = x_est
                            current_pose.position.y = y_est
                            current_pose.position.z = 0.0
                            current_pose.orientation.w = 1.0  # Identity quaternion
                            self.get_logger().info(
                                f"Computed approximate 3D pose from homography for tag {tag_id}: ({x_est:.3f}, {y_est:.3f}, 0.0)")

                        except Exception as e:
                            self.get_logger().warn(f"Failed to compute pose from homography for tag {tag_id}: {e}")
                            current_pose = None  # Ensure it's None if computation fails
                    else:
                        self.get_logger().warn(f"Detection for tag {tag_id} has no pose or homography. Skipping.")
                        current_pose = None # Ensure it's None if neither is available

                if current_pose is not None:
                    current_detected_landmarks[tag_id] = current_pose
                else:
                    self.get_logger().warn(f"Detection for tag {tag_id} has no valid pose after all attempts. Skipping.")
                    continue
        
        self.detected_landmark_poses.update(current_detected_landmarks)

        if len(self.detected_landmark_poses) >= self.min_landmarks_for_alignment:
            self.get_logger().info("All required landmarks detected. Attempting to align maps...")
            self.align_maps()

    def align_maps(self):
        # Get the transform from ceiling_camera_link to april_map
        # This transform is published by calibrated_camera_processor
        try:
            # Add a wait for transform
            if not self.tf_buffer.can_transform('april_map', 'ceiling_camera_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
                self.get_logger().warn('Waiting for transform from ceiling_camera_link to april_map...')
                return

            transform_camera_to_april_map = self.tf_buffer.lookup_transform(
                'april_map', 'ceiling_camera_link', rclpy.time.Time())
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'Could not transform ceiling_camera_link to april_map: {ex}')
            return

        src_points_april_map = []  # Detected landmark positions in april_map frame
        dst_points_nav2_map = []  # Actual landmark positions in Nav2 map frame

        self.get_logger().info("--- MAP ALIGNMENT COORDINATES ---")
        for tag_id, pose_in_camera in self.detected_landmark_poses.items():
            # Create a PoseStamped object from pose_in_camera
            pose_stamped_camera = PoseStamped()
            pose_stamped_camera.header.frame_id = 'ceiling_camera_link'
            pose_stamped_camera.header.stamp = self.get_clock().now().to_msg()
            pose_stamped_camera.pose = pose_in_camera  # Assign the Pose object

            # Get the rotation quaternion from the transform
            transform_quat = [
                transform_camera_to_april_map.transform.rotation.x,
                transform_camera_to_april_map.transform.rotation.y,
                transform_camera_to_april_map.transform.rotation.z,
                transform_camera_to_april_map.transform.rotation.w
            ]

            # Get the orientation quaternion from the pose
            pose_quat = [
                pose_stamped_camera.pose.orientation.x,
                pose_stamped_camera.pose.orientation.y,
                pose_stamped_camera.pose.orientation.z,
                pose_stamped_camera.pose.orientation.w
            ]

            # Perform manual transformation from ceiling_camera_link to april_map
            # This replaces do_transform_pose
            transformed_position = tf_transformations.quaternion_multiply(
                tf_transformations.quaternion_multiply(
                    transform_quat,
                    [pose_stamped_camera.pose.position.x, pose_stamped_camera.pose.position.y,
                     pose_stamped_camera.pose.position.z, 0.0]
                ),
                tf_transformations.quaternion_conjugate(transform_quat)
            )

            pose_in_april_map = PoseStamped()
            pose_in_april_map.pose.position.x = transformed_position[0] + \
                transform_camera_to_april_map.transform.translation.x
            pose_in_april_map.pose.position.y = transformed_position[1] + \
                transform_camera_to_april_map.transform.translation.y

            src_point = [pose_in_april_map.pose.position.x,
                         pose_in_april_map.pose.position.y]
            dst_point = [self.actual_landmark_pos[tag_id]
                         [0], self.actual_landmark_pos[tag_id][1]]
            self.get_logger().info(
                f"  Landmark {tag_id}: AprilMap Coords [{src_point[0]:.3f}, {src_point[1]:.3f}] -> Nav2 Map Coords {dst_point}")

            src_points_april_map.append(src_point)
            dst_points_nav2_map.append(dst_point)
        self.get_logger().info("---------------------------------")

        self.get_logger().info(f"DEBUG: Final len(src_points_april_map): {len(src_points_april_map)}")
        if len(src_points_april_map) < self.min_landmarks_for_alignment:
            self.get_logger().warn("Not enough transformed landmark points for alignment.")
            return

        src_pts = np.array(src_points_april_map, dtype=np.float32)
        dst_pts = np.array(dst_points_nav2_map, dtype=np.float32)

        try:
            # Calculate the affine transformation matrix from april_map to Nav2 map
            # M maps points from src_pts (april_map) to dst_pts (Nav2 map)
            M, _ = cv2.estimateAffine2D(src_pts, dst_pts)
            
            if M is None:
                self.get_logger().error("Failed to estimate affine transform.")
                return

            # Extract translation and rotation from the affine matrix
            # M = [[cos(theta), -sin(theta), tx], [sin(theta), cos(theta), ty]]
            tx = M[0, 2]
            ty = M[1, 2]
            
            # Rotation matrix part
            R = M[0:2, 0:2]
            theta = math.atan2(R[1, 0], R[0, 0]) # Yaw angle

            q = tf_transformations.quaternion_from_euler(0, 0, theta)

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map' # Nav2 map frame
            t.child_frame_id = 'april_map' # Our AprilTag map frame
            
            t.transform.translation.x = tx
            t.transform.translation.y = ty
            t.transform.translation.z = 0.0 # Assume alignment is on the ground plane
            
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.static_tf_broadcaster.sendTransform(t)
            self.alignment_published = True
            self.get_logger().info("Successfully published map to april_map transform!")
            self.get_logger().info(f"  Translation: ({tx:.3f}, {ty:.3f}, 0.0)")
            self.get_logger().info(f"  Rotation (Yaw): {math.degrees(theta):.1f}°")

        except Exception as e:
            self.get_logger().error(f"Error during map alignment: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MapAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
