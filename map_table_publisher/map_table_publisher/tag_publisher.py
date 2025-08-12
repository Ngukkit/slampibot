import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32
from apriltag_msgs.msg import AprilTagDetectionArray
import tf2_ros
import math

class TableObstacleNode(Node):
    def __init__(self):
        super().__init__('table_obstacle_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.tag_callback,
            10
        )

        self.pub = self.create_publisher(PolygonStamped, '/table_obstacle', 10)

        # 태그 ID와 테이블 크기 (단위 m)
        self.tag_ids = [1, 2]
        self.table_width = 0.6
        self.table_height = 1.0

    def tag_callback(self, msg):
        positions = {}
        for det in msg.detections:
            if det.id[0] in self.tag_ids:
                try:
                    trans = self.tf_buffer.lookup_transform(
                        'map',
                        det.pose.header.frame_id,
                        rclpy.time.Time()
                    )
                    x = det.pose.pose.pose.position.x + trans.transform.translation.x
                    y = det.pose.pose.pose.position.y + trans.transform.translation.y
                    positions[det.id[0]] = (x, y)
                except Exception as e:
                    self.get_logger().warn(str(e))

        if len(positions) == 2:
            (x1, y1) = positions[self.tag_ids[0]]
            (x2, y2) = positions[self.tag_ids[1]]

            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            yaw = math.atan2(y2 - y1, x2 - x1)

            self.publish_obstacle(cx, cy, yaw)

    def publish_obstacle(self, cx, cy, yaw):
        poly = PolygonStamped()
        poly.header.frame_id = 'map'
        poly.header.stamp = self.get_clock().now().to_msg()

        dx = self.table_width / 2
        dy = self.table_height / 2

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
