import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32
from apriltag_msgs.msg import AprilTagDetectionArray
import tf2_ros
import tf2_geometry_msgs
import math

class TableObstacleNode(Node):
    def __init__(self):
        super().__init__('table_obstacle_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.tag_callback,
            10
        )

        self.pub = self.create_publisher(PolygonStamped, '/table_obstacle', 10)

        # Declare parameters for tag IDs
        self.declare_parameter('tag_ids', [1, 2])

        self.tag_ids = self.get_parameter('tag_ids').get_parameter_value().integer_array_value

    def tag_callback(self, msg):
        positions = {}
        for det in msg.detections:
            if det.id in self.tag_ids:
                try:
                    # Construct the tag's frame ID
                    tag_frame_id = f'tag_{det.id}'

                    # Create a PoseStamped for the tag's frame (identity pose)
                    # This assumes the tag's origin is at its center
                    tag_pose_stamped = PoseStamped()
                    tag_pose_stamped.header.frame_id = tag_frame_id
                    tag_pose_stamped.header.stamp = msg.header.stamp # Use the timestamp from the detection array

                    # Transform the tag's pose from its own frame to the 'map' frame
                    map_pose = self.tf_buffer.transform(
                        tag_pose_stamped, 'map', timeout=rclpy.duration.Duration(seconds=0.1))
                    
                    positions[det.id] = (map_pose.pose.position.x, map_pose.pose.position.y)

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    self.get_logger().warn(f'Could not transform tag {det.id} to map frame: {e}')

        if len(positions) == 2:
            detected_tag_ids = list(positions.keys())
            (x1, y1) = positions[detected_tag_ids[0]]
            (x2, y2) = positions[detected_tag_ids[1]]

            # Calculate measured width and height based on diagonal tags
            measured_width = abs(x1 - x2)
            measured_height = abs(y1 - y2)

            self.get_logger().info(f'Detected tags {detected_tag_ids[0]} and {detected_tag_ids[1]}. Calculated table width: {measured_width:.3f} m, height: {measured_height:.3f} m')

            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            yaw = math.atan2(y2 - y1, x2 - x1)

            self.publish_obstacle(cx, cy, yaw, measured_width, measured_height)

    def publish_obstacle(self, cx, cy, yaw, table_width, table_height):
        poly = PolygonStamped()
        poly.header.frame_id = 'map'
        poly.header.stamp = self.get_clock().now().to_msg()

        dx = table_width / 2
        dy = table_height / 2

        # 회전 적용
        corners = [
            (-dx, -dy),
            ( dx, -dy),
            ( dx,  dy),
            (-dx,  dy)
        ]

        for px, py in corners:
            rx = px * math.cos(yaw) - py * math.sin(yaw)
            ry = px * math.sin(yaw) + py * math.cos(yaw)
            poly.polygon.points.append(Point32(x=cx+rx, y=cy+ry, z=0.0))

        self.pub.publish(poly)

def main():
    rclpy.init()
    node = TableObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
