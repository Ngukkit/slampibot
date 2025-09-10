#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
import cv2
from scipy.optimize import least_squares

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32, TransformStamped, Quaternion, Pose
from std_msgs.msg import Header
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import tf_transformations
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class CalibratedCameraProcessor(Node):
    def __init__(self):
        super().__init__('calibrated_camera_processor')
        
        # Declare parameters
        self.declare_parameter('robot_tag_id', 0)
        self.declare_parameter('table_tag_pairs', [1, 2, 3, 4, 5, 6, 7, 8])
        self.declare_parameter('landmark_tags.100.x', 0.0)
        self.declare_parameter('landmark_tags.100.y', 0.0)
        self.declare_parameter('landmark_tags.100.size', 0.05)  # Default size in meters
        self.declare_parameter('landmark_tags.200.x', 5.0)
        self.declare_parameter('landmark_tags.200.y', 0.0)
        self.declare_parameter('landmark_tags.200.size', 0.05)  # Default size in meters
        self.declare_parameter('landmark_tags.300.x', 2.5)
        self.declare_parameter('landmark_tags.300.y', 4.0)
        self.declare_parameter('landmark_tags.300.size', 0.05)  # Default size in meters
        self.declare_parameter('landmark_tags.400.x', 0.0)
        self.declare_parameter('landmark_tags.400.y', 0.0)
        self.declare_parameter('landmark_tags.400.size', 0.05)  # Default size in meters

        # Get parameters
        self.robot_tag_id = self.get_parameter('robot_tag_id').get_parameter_value().integer_value
        self.table_tag_pairs = self.get_parameter('table_tag_pairs').get_parameter_value().integer_array_value
        
        # 테이블 태그 쌍을 딕셔너리로 변환 [(1,2), (3,4), (5,6), (7,8)]
        self.table_pairs = []
        for i in range(0, len(self.table_tag_pairs), 2):
            if i + 1 < len(self.table_tag_pairs):
                self.table_pairs.append((self.table_tag_pairs[i], self.table_tag_pairs[i + 1]))
        
        # Get landmark tags parameters
        self.landmark_tags = {}
        self.landmark_tag_sizes = {}  # Dictionary to store tag sizes
        # Manually parse landmark_tags parameters
        all_params = self._parameters
        temp_landmarks = {}
        for param_name, param_value in all_params.items():
            if param_name.startswith('landmark_tags.'):
                parts = param_name.split('.')
                if len(parts) == 3:
                    tag_id_str, coord = parts[1], parts[2]
                    try:
                        tag_id = int(tag_id_str)
                        if tag_id not in temp_landmarks:
                            temp_landmarks[tag_id] = {}
                        temp_landmarks[tag_id][coord] = float(param_value.value)
                    except ValueError:
                        self.get_logger().warn(f"Invalid tag ID format: {tag_id_str}")

        for tag_id, coords in temp_landmarks.items():
            if 'x' in coords and 'y' in coords:
                self.landmark_tags[tag_id] = (coords['x'], coords['y'])
            if 'size' in coords:
                # Validate tag size
                tag_size = coords['size']
                if tag_size <= 0:
                    self.get_logger().warn(f"Invalid tag size {tag_size} for tag {tag_id}. Using default size 0.05m.")
                    self.landmark_tag_sizes[tag_id] = 0.05
                elif tag_size > 1.0:
                    self.get_logger().warn(f"Unusually large tag size {tag_size} for tag {tag_id}. Please verify.")
                    self.landmark_tag_sizes[tag_id] = tag_size
                else:
                    self.landmark_tag_sizes[tag_id] = tag_size

        if not self.landmark_tags:
            self.get_logger().error("Landmark tags parameter is not set or empty!")
        else:
            self.get_logger().info(f"Loaded {len(self.landmark_tags)} landmark(s).")

        # Camera calibration parameters (from camera_info.yaml)
        self.camera_matrix = np.array([
            [876.51626, 0.0, 311.97008],
            [0.0, 872.29481, 237.83805],
            [0.0, 0.0, 1.0]
        ])
        
        # Real distortion coefficients from camera calibration
        self.dist_coeffs = np.array([-0.015725, 0.324479, -0.002707, 0.001885, 0.000000])
        self.declare_parameter('camera_height', 1.79)
        self.camera_height = self.get_parameter('camera_height').get_parameter_value().double_value
        
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
            '/tag_detections',
            self.detections_callback,
            10)
            
        # 테이블 감지를 위한 타이머 (1초 주기)
        self.table_update_timer = self.create_timer(1.0, self.update_table_obstacles)
        self.latest_detections = None
        
        self.table_pub = self.create_publisher(PolygonStamped, '/table_obstacle', 10)
        
        # Publisher for calculated landmark markers
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.calculated_marker_pub = self.create_publisher(MarkerArray, '/landmark_markers', latching_qos)
        
        # 테이블별 퍼블리셔 생성
        self.table_pubs = {}
        for i, pair in enumerate(self.table_pairs):
            table_number = i + 1
            topic_name = f'/table_obstacles/table_{pair[0]}_{pair[1]}'
            self.table_pubs[pair] = self.create_publisher(PolygonStamped, topic_name, 10)
            self.get_logger().info(f'Created publisher for table {table_number} (tags {pair[0]}-{pair[1]}) on topic {topic_name}')

        # Persistence variables
        self.last_known_tag_positions = {}  # {tag_id: PoseStamped}
        self.last_detection_timestamps = {} # {tag_id: rclpy.time.Time}
        self.persistence_timeout_sec = 30.0

        # Calibration state
        self.map_to_camera_transform = None
        self.calibration_points = []
        self.min_calibration_points = 2  # Reduced since we have good camera calibration
        self.scale_factor = 0.013  # Default scale factor (calculated from landmarks)
        self.use_homography = False  # homography 사용 여부
        self.homography_matrix = None  # homography matrix

    def detections_callback(self, msg):
        """Store latest detections for periodic processing"""
        self.latest_detections = msg

    def update_table_obstacles(self):
        """Periodically update table obstacles based on latest detections"""
        if self.latest_detections is None:
            return
            
        current_time = self.get_clock().now() # Get current time for persistence

        # Process the latest detections
        msg = self.latest_detections
        detected_robot_pose = None
        detected_table_tags = {}
        detected_landmarks = {} # This will hold currently detected landmarks

        header = msg.header

        # Parse detections and update last known positions/timestamps
        current_frame_detected_ids = set() # Initialize here
        for detection in msg.detections:
            tag_id = detection.id
            current_frame_detected_ids.add(tag_id) # Add detected tag_id to the set
            
            # Only calculate 3D poses for tags we're interested in (landmarks, robot, or table tags)
            is_landmark = tag_id in self.landmark_tags
            is_robot = tag_id == self.robot_tag_id
            is_table = tag_id in self.table_tag_pairs
            
            # Skip tags we don't need
            if not (is_landmark or is_robot or is_table):
                continue
                
            # Convert 3D pose to 2D position (in camera frame, meters)
            # Try to get the 3D pose from the detection
            current_pose = None
            try:
                # Try the standard pose field
                current_pose = detection.pose.pose.pose
            except AttributeError:
                # If pose is not available, try to compute it from homography using cv2.solvePnP
                try:
                    # Get camera parameters
                    fx = self.camera_matrix[0, 0]
                    fy = self.camera_matrix[1, 1]
                    cx = self.camera_matrix[0, 2]
                    cy = self.camera_matrix[1, 2]
                    
                    # Use actual tag size if available, otherwise default to 5cm
                    tag_size = self.landmark_tag_sizes.get(tag_id, 0.05) if tag_id in self.landmark_tags else 0.05
                    
                    # Validate tag size
                    if tag_size <= 0:
                        self.get_logger().warn(f"Invalid tag size {tag_size} for tag {tag_id}. Using default size 0.05m.")
                        tag_size = 0.05
                    elif tag_size > 1.0:
                        self.get_logger().warn(f"Unusually large tag size {tag_size} for tag {tag_id}. Please verify.")
                    
                    # Define the 3D points of the tag corners in the tag's coordinate system
                    # The tag is centered at (0,0,0) with corners at (+-tag_size/2, +-tag_size/2, 0)
                    half_size = tag_size / 2.0
                    object_points = np.array([
                        [-half_size, -half_size, 0],  # Bottom-left
                        [ half_size, -half_size, 0],  # Bottom-right
                        [ half_size,  half_size, 0],  # Top-right
                        [-half_size,  half_size, 0]   # Top-left
                    ], dtype=np.float32)
                    
                    # Get the 2D image points of the tag corners
                    # Assuming corners are in the same order as object_points
                    image_points = np.array([
                        [detection.corners[0].x, detection.corners[0].y],  # Bottom-left
                        [detection.corners[1].x, detection.corners[1].y],  # Bottom-right
                        [detection.corners[2].x, detection.corners[2].y],  # Top-right
                        [detection.corners[3].x, detection.corners[3].y]   # Top-left
                    ], dtype=np.float32)
                    
                    # Camera intrinsic matrix
                    camera_matrix = np.array([
                        [fx, 0, cx],
                        [0, fy, cy],
                        [0, 0, 1]
                    ], dtype=np.float32)
                    
                    # Use actual distortion coefficients
                    dist_coeffs = self.dist_coeffs
                    
                    # Solve for the 3D pose using cv2.solvePnP with actual distortion coefficients
                    success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
                    
                    if success:
                        # Convert rotation vector to rotation matrix
                        rotation_matrix, _ = cv2.Rodrigues(rvec)
                        
                        # Account for camera orientation (ceiling-mounted, looking down)
                        # Flip Y and Z coordinates to match world coordinate system
                        corrected_x = tvec[0][0]
                        corrected_y = -tvec[1][0]  # Flip Y axis
                        corrected_z = 0.0  # Tags are on the ground, so Z=0
                        
                        # Create a mock pose object with the computed position and orientation
                        class MockPose:
                            def __init__(self, x, y, z, qx, qy, qz, qw):
                                self.position = type('Position', (), {'x': x, 'y': y, 'z': z})()
                                self.orientation = type('Orientation', (), {'x': qx, 'y': qy, 'z': qz, 'w': qw})()
                        
                        # For tags on the ground, we want Z=0 in world coordinates
                        current_pose = MockPose(corrected_x, corrected_y, corrected_z, 
                                               0.0, 0.0, 0.0, 1.0)  # Simple identity quaternion for now
                        
                        self.get_logger().info(f"Computed accurate 3D pose for tag {tag_id}: "
                                             f"({corrected_x:.3f}, {corrected_y:.3f}, {corrected_z:.3f})")
                    else:
                        # Log detailed information about why solvePnP failed
                        self.get_logger().warn(f"solvePnP failed for tag {tag_id}. "
                                             f"Object points: {object_points.shape}, "
                                             f"Image points: {image_points.shape}, "
                                             f"Camera matrix: {camera_matrix.shape}")
                        
                        # Fallback to the simplified approximation if solvePnP fails
                        self.get_logger().warn(f"solvePnP failed for tag {tag_id}, using simplified approximation")
                        
                        # Compute 3D position from homography and camera parameters
                        # This is a simplified approximation
                        u = detection.centre.x
                        v = detection.centre.y
                        
                        # Convert pixel coordinates to normalized camera coordinates
                        x_norm = (u - cx) / fx
                        y_norm = (v - cy) / fy
                        
                        # For a tag on the ground plane (z=0), we can estimate x,y position
                        # This is a very rough approximation
                        z_est = 1.0  # Assume 1 meter distance for now
                        x_est = x_norm * z_est
                        y_est = y_norm * z_est
                        
                        # Create a mock pose object
                        class MockPose:
                            def __init__(self, x, y, z):
                                self.position = type('Position', (), {'x': x, 'y': y, 'z': z})()
                                self.orientation = type('Orientation', (), {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0})()
                        
                        current_pose = MockPose(x_est, y_est, 0.0)
                        
                        self.get_logger().info(f"Computed approximate 3D pose for tag {tag_id}: ({x_est:.3f}, {y_est:.3f}, 0.0)")
                except cv2.error as e:
                    # Handle OpenCV specific errors
                    self.get_logger().error(f"OpenCV error in pose computation for tag {tag_id}: {e}")
                    continue
                except np.linalg.LinAlgError as e:
                    # Handle linear algebra errors
                    self.get_logger().error(f"Linear algebra error in pose computation for tag {tag_id}: {e}")
                    continue
                except Exception as e:
                    # If pose is not available, skip this detection
                    self.get_logger().warn(f"Detection for tag {tag_id} has no 3D pose and couldn't compute one: {e}")
                    continue

            # Check if this is a landmark tag
            if tag_id in self.landmark_tags:
                detected_landmarks[tag_id] = current_pose
                self.get_logger().info(f"Landmark {tag_id} detected at: ({current_pose.position.x:.3f}, {current_pose.position.y:.3f})")
                
            # Check if this is the robot tag
            elif tag_id == self.robot_tag_id:
                detected_robot_pose = current_pose
                self.get_logger().info(f"Robot tag {tag_id} detected at: ({current_pose.position.x:.3f}, {current_pose.position.y:.3f})")
                
            # Check if this is a table tag
            elif tag_id in self.table_tag_pairs:
                detected_table_tags[tag_id] = current_pose
                detected_landmarks[tag_id] = current_pose # Add to landmarks for visualization
                self.get_logger().info(f"Table tag {tag_id} detected at: ({current_pose.position.x:.3f}, {current_pose.position.y:.3f})")

            # Update last known position and timestamp for all currently detected tags
            self.last_known_tag_positions[tag_id] = current_pose
            self.last_detection_timestamps[tag_id] = current_time

        # --- Persistence Logic ---
        # Combine all known tags (landmarks and table tags) for persistence check
        all_known_tags = set(self.landmark_tags.keys())
        for pair in self.table_pairs:
            all_known_tags.add(pair[0])
            all_known_tags.add(pair[1])

        processed_landmarks_for_calibration = {} # For update_calibration_data
        processed_landmarks_for_visualization = {} # For publish_calculated_landmark_markers
        processed_table_tags_for_obstacle = {} # For publish_table_obstacle

        for tag_id in all_known_tags:
            if tag_id in current_frame_detected_ids:
                # Tag was detected in the current frame, use its fresh data
                if tag_id in detected_landmarks:
                    processed_landmarks_for_calibration[tag_id] = detected_landmarks[tag_id]
                    processed_landmarks_for_visualization[tag_id] = detected_landmarks[tag_id]
                if tag_id in detected_table_tags:
                    processed_table_tags_for_obstacle[tag_id] = detected_table_tags[tag_id]
            else:
                # Tag was NOT detected in the current frame, check persistence
                if tag_id in self.last_detection_timestamps:
                    time_since_last_detection = (current_time - self.last_detection_timestamps[tag_id]).nanoseconds / 1e9
                    if time_since_last_detection <= self.persistence_timeout_sec:
                        # Use last known position
                        persistent_pose = self.last_known_tag_positions[tag_id]
                        if tag_id in self.landmark_tags: # Check if it's a landmark
                            processed_landmarks_for_calibration[tag_id] = persistent_pose
                            processed_landmarks_for_visualization[tag_id] = persistent_pose
                        # Check if it's a table tag (need to ensure it's part of a pair for obstacle publishing)
                        if tag_id in [item for sublist in self.table_pairs for item in sublist]:
                            processed_table_tags_for_obstacle[tag_id] = persistent_pose
                        self.get_logger().info(f"Tag {tag_id} not detected, using persistent position. Time since last: {time_since_last_detection:.2f}s")
                    else:
                        self.get_logger().info(f"Tag {tag_id} lost (timeout). Time since last: {time_since_last_detection:.2f}s")
                # If tag_id was never seen or timeout, it's simply not added to processed_... dictionaries

        # Update calibration data (only when landmarks are detected)
        if processed_landmarks_for_calibration:
            self.update_calibration_data(processed_landmarks_for_calibration, header)
        if processed_landmarks_for_visualization: # Use for visualization
            self.publish_calculated_landmark_markers(processed_landmarks_for_visualization, header)

        # Establish map -> camera transform using calibrated camera
        if len(self.calibration_points) >= self.min_calibration_points:
            self.calculate_calibrated_transform(header)

        # Process robot and table poses
        if self.map_to_camera_transform is not None:
            if detected_robot_pose:
                self.publish_robot_transform(detected_robot_pose, header)
            
            # Process table tags in pairs (only when both tags of a pair are available in processed_table_tags_for_obstacle)
            for pair in self.table_pairs:
                tag1_id, tag2_id = pair
                self.get_logger().info(f"Checking table pair {pair}: tag1_id={tag1_id} in processed={tag1_id in processed_table_tags_for_obstacle}, tag2_id={tag2_id} in processed={tag2_id in processed_table_tags_for_obstacle}")
                if tag1_id in processed_table_tags_for_obstacle and tag2_id in processed_table_tags_for_obstacle:
                    table_tags = {tag1_id: processed_table_tags_for_obstacle[tag1_id], tag2_id: processed_table_tags_for_obstacle[tag2_id]}
                    self.publish_table_obstacle(table_tags, header)
                elif tag1_id in processed_table_tags_for_obstacle:
                    # Only tag1 is available, use it to estimate the table
                    self.get_logger().warn(f"Only tag {tag1_id} available for table pair {pair}, estimating table position")
                    table_tags = {tag1_id: processed_table_tags_for_obstacle[tag1_id]}
                    self.publish_table_obstacle(table_tags, header)
                elif tag2_id in processed_table_tags_for_obstacle:
                    # Only tag2 is available, use it to estimate the table
                    self.get_logger().warn(f"Only tag {tag2_id} available for table pair {pair}, estimating table position")
                    table_tags = {tag2_id: processed_table_tags_for_obstacle[tag2_id]}
                    self.publish_table_obstacle(table_tags, header)
                else:
                    self.get_logger().warn(f"Neither tag {tag1_id} nor {tag2_id} available for table pair {pair}")

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
                
                # Debug: Compare expected vs actual positions
                self.get_logger().debug(f"DEBUG - Tag {tag_id}: Expected YAML position ({known_pos[0]:.3f}, {known_pos[1]:.3f})")
                self.get_logger().debug(f"DEBUG - Tag {tag_id}: Detected camera position ({pose_in_cam.position.x:.3f}, {pose_in_cam.position.y:.3f})")
                
                # Calculate difference between expected and detected positions
                diff_x = known_pos[0] - pose_in_cam.position.x
                diff_y = known_pos[1] - pose_in_cam.position.y
                distance = math.sqrt(diff_x**2 + diff_y**2)
                self.get_logger().debug(f"DEBUG - Tag {tag_id}: Difference ({diff_x:.3f}, {diff_y:.3f}), Distance: {distance:.3f}m")

    def calculate_calibrated_transform(self, header):
        """Calculate transform using homography-based calibration with 3D pose information"""
        if len(self.calibration_points) < self.min_calibration_points:
            self.get_logger().warn(f"Need at least {self.min_calibration_points} calibration points, have {len(self.calibration_points)}")
            return

        landmark_points = []
        for point in self.calibration_points:
            tag_id = point['tag_id']
            if tag_id in self.landmark_tags:
                landmark_points.append({
                    'tag_id': tag_id,
                    'actual_pos': self.landmark_tags[tag_id],  # 실제 위치 (맵 좌표계)
                    'detected_pos': point['camera_pos']       # 감지된 위치 (카메라 좌표계, 미터 단위)
                })

        # 4개 이상의 랜드마크 태그가 있으면 homography 사용
        if len(landmark_points) >= 4:
            self.get_logger().info(f"Using homography with {len(landmark_points)} landmark tags")
            
            src_points = []  # 카메라 좌표계 점들 (detected_pos)
            dst_points = []  # 맵 좌표계 점들 (actual_pos)
            
            for point in landmark_points:
                src_points.append([point['detected_pos'][0], point['detected_pos'][1]])
                dst_points.append([point['actual_pos'][0], point['actual_pos'][1]])

            try:
                import cv2
                import numpy as np
                
                src_pts = np.array(src_points, dtype=np.float32)
                dst_pts = np.array(dst_points, dtype=np.float32)
                
                # Check if we have enough points for homography calculation
                if len(src_pts) < 4 or len(dst_pts) < 4:
                    self.get_logger().warn(f"Not enough points for homography calculation. "
                                         f"Need at least 4, have {len(src_pts)}")
                    self.use_homography = False
                    return
                
                H, status = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                
                if H is not None:
                    self.get_logger().info("Homography matrix calculated successfully")
                    self.homography_matrix = H
                    self.use_homography = True
                    
                    # Extract translation and rotation from homography for TransformStamped
                    # Assuming a 2D rigid transformation (translation and rotation)
                    # H = [[cos(theta), -sin(theta), Tx], [sin(theta), cos(theta), Ty], [0, 0, 1]]
                    
                    # Extract translation
                    tx = H[0, 2]
                    ty = H[1, 2]
                    
                    # Extract rotation (yaw)
                    # theta = atan2(H[1,0], H[0,0]) or atan2(-H[0,1], H[1,1])
                    # Using average of two ways to be robust
                    theta = math.atan2((H[1,0] - H[0,1])/2, (H[0,0] + H[1,1])/2)
                    
                    # Account for camera orientation (ceiling-mounted, looking down)
                    # The camera coordinate system has Y pointing down, but we want Y pointing up
                    # So we need to flip the Y axis
                    ty = -ty
                    
                    q = tf_transformations.quaternion_from_euler(0, 0, theta)

                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'map'
                    t.child_frame_id = 'default_cam'
                    
                    t.transform.translation.x = tx
                    t.transform.translation.y = ty
                    t.transform.translation.z = 0.0 # Ensure Z is 0
                    
                    t.transform.rotation.x = q[0]
                    t.transform.rotation.y = q[1]
                    t.transform.rotation.z = q[2]
                    t.transform.rotation.w = q[3]

                    self.map_to_camera_transform = t
                    self.tf_broadcaster.sendTransform(self.map_to_camera_transform)
                    
                    self.get_logger().info(f"Calibrated transform (Homography) calculated:")
                    self.get_logger().info(f"  Translation: ({tx:.3f}, {ty:.3f})")
                    self.get_logger().info(f"  Rotation (Yaw): {math.degrees(theta):.1f}°")
                    
                    self.verify_homography_calibrated_transform()
                    return # IMPORTANT: Exit after successful homography
                else:
                    self.get_logger().warn("Failed to calculate homography matrix")
                    self.use_homography = False # Fallback to old method if homography fails
                    
            except cv2.error as e:
                self.get_logger().error(f"OpenCV error in homography calculation: {e}")
                self.use_homography = False # Fallback to old method if homography fails
            except np.linalg.LinAlgError as e:
                self.get_logger().error(f"Linear algebra error in homography calculation: {e}")
                self.use_homography = False # Fallback to old method if homography fails
            except Exception as e:
                self.get_logger().warn(f"Failed to calculate homography: {e}")
                self.use_homography = False # Fallback to old method if homography fails

        # If homography failed or not enough points, use the affine transformation (old method)
        self.get_logger().info(f"Using affine transformation with {len(landmark_points)} landmark tags (Homography not used or failed)")
        
        scales = []
        translations_x = []
        translations_y = []
        
        for point in landmark_points:
            actual_x, actual_y = point['actual_pos']
            detected_x, detected_y = point['detected_pos']
            
            scales.append(1.0) # 3D pose is already in meters
            
            tx = actual_x - detected_x
            ty = actual_y - detected_y
            
            translations_x.append(tx)
            translations_y.append(ty)

        avg_scale = 1.0 if len(scales) == 0 else sum(scales) / len(scales)
        avg_tx = 0.0 if len(translations_x) == 0 else sum(translations_x) / len(translations_x)
        avg_ty = 0.0 if len(translations_y) == 0 else sum(translations_y) / len(translations_y)
        avg_ty = -avg_ty  # Flip Y to account for camera orientation

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'default_cam'
        
        t.transform.translation.x = avg_tx
        t.transform.translation.y = avg_ty
        t.transform.translation.z = 0.0
        
        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0

        self.map_to_camera_transform = t
        self.static_tf_broadcaster.sendTransform(self.map_to_camera_transform)
        
        self.get_logger().info(f"Calibrated transform calculated with 3D pose alignment (Fallback):")
        self.get_logger().info(f"  Translation: ({avg_tx:.3f}, {avg_ty:.3f})")
        self.get_logger().info(f"  Scale factor: {avg_scale}")
        
        self.scale_factor = avg_scale
        self.use_homography = False # Ensure this is false for fallback
        
        self.verify_calibrated_transform()

    def verify_calibrated_transform(self):
        """Verify the calculated transform by checking all calibration points"""
        if self.map_to_camera_transform is None:
            return
            
        total_error = 0.0
        verified_points = 0
        
        for point in self.calibration_points:
            tag_id = point['tag_id']
            if tag_id in self.landmark_tags:
                # 실제 위치
                actual_pos = self.landmark_tags[tag_id]
                # 감지된 위치
                detected_pos = point['camera_pos']
                
                # 변환 적용
                transformed_x = (detected_pos[0] * self.scale_factor) + self.map_to_camera_transform.transform.translation.x
                transformed_y = (detected_pos[1] * self.scale_factor) + self.map_to_camera_transform.transform.translation.y
                
                error = math.sqrt((transformed_x - actual_pos[0])**2 + (transformed_y - actual_pos[1])**2)
                total_error += error
                verified_points += 1
                
                self.get_logger().info(f"Verification {tag_id}: Expected({actual_pos[0]:.3f}, {actual_pos[1]:.3f}) -> Calculated({transformed_x:.3f}, {transformed_y:.3f}) -> Error: {error:.3f}m")
                
                # Log debug comparison - YAML 정의 위치 vs 변환된 위치
                self.log_debug_comparison(tag_id, actual_pos, detected_pos, (transformed_x, transformed_y), "Affine")
        
        if verified_points > 0:
            avg_error = total_error / verified_points
            self.get_logger().info(f"Average transformation error: {avg_error:.3f}m")
            
            if avg_error > 0.1:  # If error is more than 10cm
                self.get_logger().warn(f"High transformation error detected: {avg_error:.3f}m. Consider recalibrating landmarks.")

    def verify_homography_calibrated_transform(self):
        """Verify the homography-based transform by checking all calibration points"""
        if not hasattr(self, 'homography_matrix'):
            return
            
        import numpy as np
        total_error = 0.0
        verified_points = 0
        
        for point in self.calibration_points:
            tag_id = point['tag_id']
            if tag_id in self.landmark_tags:
                # 실제 위치
                actual_pos = self.landmark_tags[tag_id]
                # 감지된 위치
                detected_pos = point['camera_pos']
                
                # homography를 사용하여 변환 적용
                src_point = np.array([[[detected_pos[0], detected_pos[1]]]], dtype=np.float32)
                dst_point = cv2.perspectiveTransform(src_point, self.homography_matrix)
                transformed_x, transformed_y = dst_point[0][0]
                
                error = math.sqrt((transformed_x - actual_pos[0])**2 + (transformed_y - actual_pos[1])**2)
                total_error += error
                verified_points += 1
                
                self.get_logger().info(f"Verification {tag_id}: Expected({actual_pos[0]:.3f}, {actual_pos[1]:.3f}) -> Calculated({transformed_x:.3f}, {transformed_y:.3f}) -> Error: {error:.3f}m")
                
                # Log debug comparison - YAML 정의 위치 vs 변환된 위치
                self.log_debug_comparison(tag_id, actual_pos, detected_pos, (transformed_x, transformed_y), "Homography")
        
        if verified_points > 0:
            avg_error = total_error / verified_points
            self.get_logger().info(f"Average homography transformation error: {avg_error:.3f}m")
            
            if avg_error > 0.1:  # If error is more than 10cm
                self.get_logger().warn(f"High homography transformation error detected: {avg_error:.3f}m.")
        

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
        """Publish table obstacle, using homography if available for precision."""
        poses_in_map = {}

        # Prioritize homography for coordinate transformation if available
        if self.use_homography and self.homography_matrix is not None:
            for tag_id, pose_in_cam in table_tags.items():
                try:
                    tag_pos = (pose_in_cam.position.x, pose_in_cam.position.y)
                    src_point = np.array([[[tag_pos[0], tag_pos[1]]]], dtype=np.float32)
                    dst_point = cv2.perspectiveTransform(src_point, self.homography_matrix)
                    
                    if np.isnan(dst_point).any() or np.isinf(dst_point).any():
                        self.get_logger().warn(f'Homography transformation resulted in NaN/Inf for table tag {tag_id}. Skipping.')
                        continue

                    poses_in_map[tag_id] = (float(dst_point[0][0][0]), float(dst_point[0][0][1]))
                except Exception as e:
                    self.get_logger().warn(f'Could not transform table tag {tag_id} pose using homography: {e}')
                    continue
        # Fallback to TF transform if homography is not in use
        elif self.map_to_camera_transform is not None:
            for tag_id, pose_in_cam in table_tags.items():
                pose_stamped_in = PoseStamped()
                pose_stamped_in.header.frame_id = 'default_cam'
                pose_stamped_in.header.stamp = header.stamp
                if not isinstance(pose_in_cam, Pose):
                    temp_pose = Pose()
                    temp_pose.position.x = pose_in_cam.position.x
                    temp_pose.position.y = pose_in_cam.position.y
                    temp_pose.position.z = pose_in_cam.position.z
                    temp_pose.orientation.w = 1.0
                    pose_stamped_in.pose = temp_pose
                else:
                    pose_stamped_in.pose = pose_in_cam
                
                try:
                    transformed_pose_stamped = self.tf_buffer.transform(pose_stamped_in, 'map', rclpy.duration.Duration(seconds=0.1))
                    poses_in_map[tag_id] = (transformed_pose_stamped.pose.position.x, transformed_pose_stamped.pose.position.y)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    self.get_logger().warn(f'Could not transform table tag {tag_id} pose using TF: {e}')
                    return
        else:
            self.get_logger().warn('No transformation available for table tags.')
            return

        if len(poses_in_map) < 2:
            return

        tag_ids = list(poses_in_map.keys())
        (x1, y1) = poses_in_map[tag_ids[0]]
        (x2, y2) = poses_in_map[tag_ids[1]]

        # Calculate yaw from the vector connecting the two tags, assuming they are on a diagonal
        diagonal_yaw = math.atan2(y2 - y1, x2 - x1)
        yaw = diagonal_yaw + math.pi / 4.0  # Add 45 degrees to align the sides

        # Calculate table properties assuming a square table
        side_length = math.sqrt((x1 - x2)**2 + (y1 - y2)**2) / math.sqrt(2)
        table_width = side_length
        table_height = side_length

        poly = PolygonStamped()
        poly.header.frame_id = 'map'
        poly.header.stamp = self.get_clock().now().to_msg()

        dx = table_width / 2
        dy = table_height / 2

        # Calculate center and apply rotation
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        
        self.get_logger().info(f"Table center: ({cx:.3f}, {cy:.3f}), size: {table_width:.3f}x{table_height:.3f}, yaw: {math.degrees(yaw):.1f}° (from tag positions)")

        # Create polygon corners and rotate them
        corners = [(-dx, -dy), (dx, -dy), (dx, dy), (-dx, dy)]
        for px, py in corners:
            rx = px * math.cos(yaw) - py * math.sin(yaw)
            ry = px * math.sin(yaw) + py * math.cos(yaw)
            poly.polygon.points.append(Point32(x=cx + rx, y=cy + ry, z=0.0))

        # Publish to the correct topic for the table pair
        current_pair = tuple(sorted(tag_ids))
        found_publisher = False
        for pair_key, publisher in self.table_pubs.items():
            if tuple(sorted(pair_key)) == current_pair:
                publisher.publish(poly)
                found_publisher = True
                break
        
        if not found_publisher:
            self.table_pub.publish(poly)


    def publish_calculated_landmark_markers(self, detected_landmarks, header):
        if self.map_to_camera_transform is None:
            return

        marker_array = MarkerArray()
        for tag_id, pose_in_cam in detected_landmarks.items():
            # Create a PoseStamped message for the transformation
            pose_stamped_in = PoseStamped()
            pose_stamped_in.header.frame_id = 'default_cam'
            pose_stamped_in.header.stamp = header.stamp

            # Ensure pose_in_cam is a geometry_msgs/Pose
            if not isinstance(pose_in_cam, Pose):
                # Convert MockPose to geometry_msgs/Pose
                temp_pose = Pose()
                temp_pose.position.x = pose_in_cam.position.x
                temp_pose.position.y = pose_in_cam.position.y
                temp_pose.position.z = pose_in_cam.position.z
                temp_pose.orientation.x = pose_in_cam.orientation.x
                temp_pose.orientation.y = pose_in_cam.orientation.y
                temp_pose.orientation.z = pose_in_cam.orientation.z
                temp_pose.orientation.w = pose_in_cam.orientation.w
                pose_stamped_in.pose = temp_pose
            else:
                pose_stamped_in.pose = pose_in_cam

            # Transform the pose to the map frame
            transformed_pose = Pose()
            if self.use_homography and self.homography_matrix is not None:
                try:
                    # Use cv2.perspectiveTransform for homography
                    src_point = np.array([[[pose_stamped_in.pose.position.x, pose_stamped_in.pose.position.y]]], dtype=np.float32)
                    dst_point = cv2.perspectiveTransform(src_point, self.homography_matrix)
                    
                    # Check for NaN or Inf values
                    if np.isnan(dst_point).any() or np.isinf(dst_point).any():
                        self.get_logger().warn(f'Homography transformation resulted in NaN/Inf for landmark {tag_id}. Skipping.')
                        continue

                    transformed_pose.position.x = float(dst_point[0][0][0])
                    transformed_pose.position.y = float(dst_point[0][0][1])
                    transformed_pose.position.z = float(0.0) # Homography is 2D, set Z to 0
                    transformed_pose.orientation.w = float(1.0) # Identity quaternion for 2D points
                    transformed_pose.orientation.x = float(0.0)
                    transformed_pose.orientation.y = float(0.0)
                    transformed_pose.orientation.z = float(0.0)
                except Exception as e:
                    self.get_logger().warn(f'Could not transform landmark {tag_id} pose using homography: {e}')
                    continue
            elif self.map_to_camera_transform is not None:
                try:
                    # Existing matrix transformation for affine
                    pose_matrix = tf_transformations.quaternion_matrix([
                        pose_stamped_in.pose.orientation.x,
                        pose_stamped_in.pose.orientation.y,
                        pose_stamped_in.pose.orientation.z,
                        pose_stamped_in.pose.orientation.w
                    ])
                    pose_matrix[0, 3] = pose_stamped_in.pose.position.x
                    pose_matrix[1, 3] = pose_stamped_in.pose.position.y
                    pose_matrix[2, 3] = pose_stamped_in.pose.position.z

                    transform_matrix = tf_transformations.quaternion_matrix([
                        self.map_to_camera_transform.transform.rotation.x,
                        self.map_to_camera_transform.transform.rotation.y,
                        self.map_to_camera_transform.transform.rotation.z,
                        self.map_to_camera_transform.transform.rotation.w
                    ])
                    transform_matrix[0, 3] = self.map_to_camera_transform.transform.translation.x
                    transform_matrix[1, 3] = self.map_to_camera_transform.transform.translation.y
                    transform_matrix[2, 3] = self.map_to_camera_transform.transform.translation.z

                    transformed_matrix = np.dot(transform_matrix, pose_matrix)

                    # Check for NaN or Inf values
                    if np.isnan(transformed_matrix).any() or np.isinf(transformed_matrix).any():
                        self.get_logger().warn(f'Affine transformation resulted in NaN/Inf for landmark {tag_id}. Skipping.')
                        continue

                    transformed_pose.position.x = float(transformed_matrix[0, 3])
                    transformed_pose.position.y = float(transformed_matrix[1, 3])
                    transformed_pose.position.z = float(transformed_matrix[2, 3])
                    
                    quat = tf_transformations.quaternion_from_matrix(transformed_matrix)
                    transformed_pose.orientation.x = float(quat[0])
                    transformed_pose.orientation.y = float(quat[1])
                    transformed_pose.orientation.z = float(quat[2])
                    transformed_pose.orientation.w = float(quat[3])

                except Exception as e:
                    self.get_logger().warn(f'Could not transform landmark {tag_id} pose using affine: {e}')
                    continue
            else:
                self.get_logger().warn(f'No transformation available for landmark {tag_id}. Skipping.')
                continue

            # Create a marker for the calculated position
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "calculated_landmark_tags"
            marker.id = tag_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = transformed_pose # Assign the manually transformed pose
            # Set Z-coordinate to 0 since tags are on the ground
            marker.pose.position.z = 0.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 0.8
            if tag_id in self.table_tag_pairs:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0 # Blue
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0 # Green
            marker_array.markers.append(marker)

        if marker_array.markers:
            self.calculated_marker_pub.publish(marker_array)

    def log_debug_comparison(self, tag_id, expected_pos, detected_pos, calculated_pos, transform_type="N/A"):
        """Log debug information comparing expected, detected, and calculated positions"""
        self.get_logger().debug(f"DEBUG - Tag {tag_id} Position Analysis:")
        self.get_logger().debug(f"  YAML Expected position: ({expected_pos[0]:.3f}, {expected_pos[1]:.3f})")
        self.get_logger().debug(f"  Camera Detected position: ({detected_pos[0]:.3f}, {detected_pos[1]:.3f})")
        self.get_logger().debug(f"  {transform_type} Transformed position: ({calculated_pos[0]:.3f}, {calculated_pos[1]:.3f})")
        
        # Calculate differences
        # Difference between YAML expected and camera detected
        expected_detected_diff_x = expected_pos[0] - detected_pos[0]
        expected_detected_diff_y = expected_pos[1] - detected_pos[1]
        expected_detected_distance = math.sqrt(expected_detected_diff_x**2 + expected_detected_diff_y**2)
        
        # Difference between YAML expected and transformed
        expected_transformed_diff_x = expected_pos[0] - calculated_pos[0]
        expected_transformed_diff_y = expected_pos[1] - calculated_pos[1]
        expected_transformed_distance = math.sqrt(expected_transformed_diff_x**2 + expected_transformed_diff_y**2)
        
        self.get_logger().debug(f"  YAML vs Camera Diff: ({expected_detected_diff_x:.3f}, {expected_detected_diff_y:.3f}), Distance: {expected_detected_distance:.3f}m")
        self.get_logger().debug(f"  YAML vs Transformed Diff: ({expected_transformed_diff_x:.3f}, {expected_transformed_diff_y:.3f}), Distance: {expected_transformed_distance:.3f}m")

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