#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import tf2_ros
import tf_transformations
from geometry_msgs.msg import PolygonStamped, Point32, TransformStamped, Quaternion

class CeilingCameraProcessorTF(Node):
    def __init__(self):
        super().__init__('ceiling_camera_processor_tf')

        # --- Parameters ---
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value

        self.declare_parameter('robot_tag_id', 0)
        self.robot_tag_id = self.get_parameter('robot_tag_id').get_parameter_value().integer_value

        self.landmark_tags = {'100', '200', '300'}
        self.landmark_map_coords = {
            '100': tuple(self.declare_parameter('landmark_coords.100', [0.0, 0.0]).get_parameter_value().double_array_value),
            '200': tuple(self.declare_parameter('landmark_coords.200', [5.0, 0.0]).get_parameter_value().double_array_value),
            '300': tuple(self.declare_parameter('landmark_coords.300', [2.5, 4.0]).get_parameter_value().double_array_value),
        }

        self.tables = {f"table{i+1}": (2*i+1, 2*i+2) for i in range(6)}
        self.table_pubs = {name: self.create_publisher(PolygonStamped, f'/table_obstacles/{name}', 10) for name in self.tables.keys()}

        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # --- State ---
        self.T_map_from_cam = None  # 3x3 homogeneous transform matrix

        # --- Main Loop ---
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        self.get_logger().info(f"CeilingCameraProcessor (TF version) started.")
        self.get_logger().info(f"Waiting for transforms from camera_frame: '{self.camera_frame}'")


    def timer_callback(self):
        if self.T_map_from_cam is None:
            self.try_to_calibrate()
        else:
            self.publish_table_obstacles()

    def try_to_calibrate(self):
        detected_landmarks = {}
        for tag_id_str in self.landmark_tags:
            try:
                # apriltag_ros publishes tag frames relative to the camera frame
                trans = self.tf_buffer.lookup_transform(self.camera_frame, f'tag36h11:{tag_id_str}', rclpy.time.Time())
                detected_landmarks[tag_id_str] = (trans.transform.translation.x, trans.transform.translation.y)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                pass # Tag not visible, just skip

        if len(detected_landmarks) < 2:
            self.get_logger().warn(f"Not enough landmark TFs visible yet. Found: {list(detected_landmarks.keys())}", throttle_duration_sec=5)
            return

        # --- We have enough landmarks, let's calculate the transform ---
        self.get_logger().info(f"Calibrating with landmarks: {list(detected_landmarks.keys())}")
        
        cam_pts = [detected_landmarks[k] for k in detected_landmarks.keys()]
        map_pts = [self.landmark_map_coords[k] for k in detected_landmarks.keys()]

        src = np.asarray(cam_pts, dtype=float)
        dst = np.asarray(map_pts, dtype=float)
        
        mu_src = src.mean(axis=0)
        mu_dst = dst.mean(axis=0)
        src_c = src - mu_src
        
        var_src = (src_c**2).sum()
        if var_src < 1e-9:
            self.get_logger().error("Landmarks in camera frame are too close; cannot compute a stable transform.")
            return

        cov = (dst_c.T @ src_c) / src.shape[0]
        U, S, Vt = np.linalg.svd(cov)
        R = U @ Vt
        if np.linalg.det(R) < 0:
            R[:, -1] *= -1
        
        scale = np.trace(np.diag(S)) / var_src
        t = mu_dst - scale * (R @ mu_src)

        T = np.eye(3)
        T[0:2, 0:2] = scale * R
        T[0:2, 2] = t
        self.T_map_from_cam = T

        yaw = math.atan2(R[1, 0], R[0, 0])
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)

        tmsg = TransformStamped()
        tmsg.header.stamp = self.get_clock().now().to_msg()
        tmsg.header.frame_id = 'map'
        tmsg.child_frame_id = self.camera_frame
        tmsg.transform.translation.x = float(t[0])
        tmsg.transform.translation.y = float(t[1])
        tmsg.transform.translation.z = 0.0  # Assuming a 2D plane
        tmsg.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.static_tf_broadcaster.sendTransform(tmsg)
        self.get_logger().info(f"SUCCESS! Published static transform 'map' -> '{self.camera_frame}'")
        self.get_logger().info(f"Transform params: scale={scale:.4f}, yaw={math.degrees(yaw):.2f}°, t=({t[0]:.3f}, {t[1]:.3f})")
        
        # Stop this timer as we are now calibrated
        self.timer.cancel()
        # Start the obstacle publishing timer
        self.obstacle_timer = self.create_timer(0.2, self.publish_table_obstacles)


    def publish_table_obstacles(self):
        if self.T_map_from_cam is None: return

        for name, (t1_id, t2_id) in self.tables.items():
            try:
                # Get table tag poses from TF
                p1_cam = self.tf_buffer.lookup_transform(self.camera_frame, f'tag36h11:{t1_id}', rclpy.time.Time())
                p2_cam = self.tf_buffer.lookup_transform(self.camera_frame, f'tag36h11:{t2_id}', rclpy.time.Time())

                # Transform to map frame
                x1, y1 = self.cam_to_map_xy(p1_cam.transform.translation.x, p1_cam.transform.translation.y)
                x2, y2 = self.cam_to_map_xy(p2_cam.transform.translation.x, p2_cam.transform.translation.y)

                x_left, x_right = (x1, x2) if x1 <= x2 else (x2, x1)
                y_bottom, y_top = (y2, y1) if y1 >= y2 else (y1, y2)

                poly = PolygonStamped()
                poly.header.frame_id = 'map'
                poly.header.stamp = self.get_clock().now().to_msg()
                corners = [(x_left, y_top), (x_right, y_top), (x_right, y_bottom), (x_left, y_bottom)]
                for px, py in corners:
                    poly.polygon.points.append(Point32(x=px, y=py, z=0.0))
                self.table_pubs[name].publish(poly)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass # Table not visible

    def cam_to_map_xy(self, x, y):
        w = self.T_map_from_cam @ np.array([x, y, 1.0])
        return float(w[0]), float(w[1])

def main(args=None):
    rclpy.init(args=args)
    node = CeilingCameraProcessorTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()