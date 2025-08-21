import rclpy
from rclpy.node import Node
import numpy as np
import math

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32, TransformStamped, Quaternion, Pose
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import tf_transformations

class CeilingCameraProcessor(Node):
    def __init__(self):
        super().__init__('ceiling_camera_processor')

        # --- Parameter Declarations ---
        self.declare_parameter('robot_tag_id', 0)
        # Hardcoded table definitions (due to rclpy parameter parsing issues)
        self.tables = {
            "table1": [1, 2],
            "table2": [3, 4]
        }
        self.all_table_tags = [1, 2, 3, 4] # Manually populate all_table_tags
        self.declare_parameter('landmark_tags.100.x', 0.0)
        self.declare_parameter('landmark_tags.100.y', 0.0)
        self.declare_parameter('landmark_tags.200.x', 5.0)
        self.declare_parameter('landmark_tags.200.y', 0.0)
        self.declare_parameter('landmark_tags.300.x', 2.5)
        self.declare_parameter('landmark_tags.300.y', 4.0)

        # --- Getting Parameters ---
        self.robot_tag_id = self.get_parameter('robot_tag_id').get_parameter_value().integer_value
        
        self.landmark_tags = {}
        self.landmark_tags['100'] = (self.get_parameter('landmark_tags.100.x').get_parameter_value().double_value, self.get_parameter('landmark_tags.100.y').get_parameter_value().double_value)
        self.landmark_tags['200'] = (self.get_parameter('landmark_tags.200.x').get_parameter_value().double_value, self.get_parameter('landmark_tags.200.y').get_parameter_value().double_value)
        self.landmark_tags['300'] = (self.get_parameter('landmark_tags.300.x').get_parameter_value().double_value, self.get_parameter('landmark_tags.300.y').get_parameter_value().double_value)

        # --- TF Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # --- Subscribers and Publishers ---
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detections_callback,
            10)
        
        self.table_publishers = {}
        for table_name in self.tables.keys():
            topic = f'/table_obstacles/{table_name}'
            self.table_publishers[table_name] = self.create_publisher(PolygonStamped, topic, 10)
            self.get_logger().info(f'Created obstacle publisher for {table_name} on topic {topic}')

        self.map_to_camera_transform = None
        self.get_logger().info('Ceiling Camera Processor node started. Waiting for landmark tags to establish map frame...')

    def detections_callback(self, msg):
        header = msg.header
        detected_landmarks = {}
        detected_robot_pose = None
        detected_table_tags = {}

        for detection in msg.detections:
            tag_id = detection.id # Keep as integer for lookups

            # Prefer metric pose from AprilTag (meters in camera frame)
            current_pose = Pose()
            try:
                current_pose = detection.pose.pose.pose
            except AttributeError:
                self.get_logger().warn(f"AprilTag {tag_id} detected but 3D pose not available. Skipping.")
                continue # Skip this detection if 3D pose is not available

            if str(tag_id) in self.landmark_tags:
                detected_landmarks[str(tag_id)] = current_pose
            elif tag_id == self.robot_tag_id:
                detected_robot_pose = current_pose
            elif tag_id in self.all_table_tags:
                detected_table_tags[tag_id] = current_pose

        # 1. Establish map -> camera transform using landmarks
        if self.map_to_camera_transform is None and len(detected_landmarks) > 0:
            self.calculate_map_to_camera_transform(detected_landmarks, header)

        # 2. Process robot and table poses only after the map frame is established
        if self.map_to_camera_transform is not None:
            if detected_robot_pose:
                self.publish_robot_transform(detected_robot_pose, header)
            
            for table_name, tag_pair in self.tables.items():
                tag1_id, tag2_id = tag_pair
                if tag1_id in detected_table_tags and tag2_id in detected_table_tags:
                    self.get_logger().info(f'Found complete tag pair for {table_name} ({tag1_id}, {tag2_id}).')
                    table_corners = {
                        tag1_id: detected_table_tags[tag1_id],
                        tag2_id: detected_table_tags[tag2_id]
                    }
                    publisher = self.table_publishers[table_name]
                    self.publish_table_obstacle(table_corners, header, publisher)

    def publish_table_obstacle(self, table_tags, header, publisher):
        poses_in_map = {}
        for tag_id, pose_in_cam in table_tags.items():
            m_tag_in_cam = self.pose_to_matrix(pose_in_cam)
            m_map_to_camera = self.transform_to_matrix(self.map_to_camera_transform.transform)
            m_tag_in_map = np.dot(m_map_to_camera, m_tag_in_cam)
            translation = tf_transformations.translation_from_matrix(m_tag_in_map)
            poses_in_map[tag_id] = (translation[0], translation[1])

        tag_ids = list(poses_in_map.keys())
        (x1, y1) = poses_in_map[tag_ids[0]]
        (x2, y2) = poses_in_map[tag_ids[1]]

        table_width = math.sqrt((x1 - x2)**2 + (y1 - y2)**2) # This is the diagonal distance
        table_height = self.table_obstacle_default_height # Use parameter for height

        poly = PolygonStamped()
        poly.header.frame_id = 'map'
        poly.header.stamp = self.get_clock().now().to_msg()

        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        yaw = math.atan2(y2 - y1, x2 - x1)

        # Simplified rectangle generation
        dx = table_width / 2
        dy = table_height / 2
        corners = [(-dx, -dy), (dx, -dy), (dx, dy), (-dx, dy)]
        for px, py in corners:
            rx = px * math.cos(yaw) - py * math.sin(yaw)
            ry = px * math.sin(yaw) + py * math.cos(yaw)
            poly.polygon.points.append(Point32(x=cx + rx, y=cy + ry, z=0.0))

        publisher.publish(poly)
        self.get_logger().info(f"Published obstacle for topic: {publisher.topic_name}")

    def calculate_map_to_camera_transform(self, detected_landmarks, header):
        # This function can be expanded with more robust logic (e.g., using multiple landmarks)
        # For now, using the first detected landmark
        tag_id = list(detected_landmarks.keys())[0]
        tag_pose_in_cam = detected_landmarks[tag_id]
        
        landmark_pos_in_map = self.landmark_tags[tag_id]
        
        m_cam_landmark = self.pose_to_matrix(tag_pose_in_cam)
        m_landmark_cam = np.linalg.inv(m_cam_landmark)
        m_map_landmark = tf_transformations.translation_matrix((landmark_pos_in_map[0], landmark_pos_in_map[1], 0.0))
        m_map_cam = np.dot(m_map_landmark, m_landmark_cam)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = header.frame_id
        
        translation = tf_transformations.translation_from_matrix(m_map_cam)
        rotation = tf_transformations.quaternion_from_matrix(m_map_cam)
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation = Quaternion(x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3])

        self.map_to_camera_transform = t
        self.static_tf_broadcaster.sendTransform(self.map_to_camera_transform)
        self.get_logger().info(f"Published static transform from 'map' to '{header.frame_id}'")

    def publish_robot_transform(self, robot_pose_in_cam, header):
        # This function can be expanded to publish map->odom transform
        pass # Placeholder

    def pose_to_matrix(self, pose):
        return np.dot(tf_transformations.translation_matrix((pose.position.x, pose.position.y, pose.position.z)),
                        tf_transformations.quaternion_matrix((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)))

    def transform_to_matrix(self, transform):
        return np.dot(tf_transformations.translation_matrix((transform.translation.x, transform.translation.y, transform.translation.z)),
                        tf_transformations.quaternion_matrix((transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)))

def main(args=None):
    rclpy.init(args=args)
    node = CeilingCameraProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
