import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Point32

class PolygonPublisher(Node):
    def __init__(self):
        super().__init__('polygon_publisher')
        self.publisher_ = self.create_publisher(PolygonStamped, 'table_polygon', 10)
        timer_period = 1.0  # 1초마다 발행
        self.timer = self.create_timer(timer_period, self.publish_polygon)

    def publish_polygon(self):
        msg = PolygonStamped()
        msg.header.frame_id = 'map'  # 적절한 좌표계로 변경
        msg.header.stamp = self.get_clock().now().to_msg()

        # 예시: 사각형 테이블 꼭짓점 4개 (x,y,z)
        points = [
            Point32(x=0.0, y=0.0, z=0.0),
            Point32(x=1.0, y=0.0, z=0.0),
            Point32(x=1.0, y=1.0, z=0.0),
            Point32(x=0.0, y=1.0, z=0.0),
        ]
        msg.polygon.points = points
        self.publisher_.publish(msg)
        self.get_logger().info('Published polygon')

def main(args=None):
    rclpy.init(args=args)
    node = PolygonPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
