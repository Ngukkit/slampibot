#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, PoseStamped, Point
import math

class TableCenterPublisher(Node):
    def __init__(self):
        super().__init__('table_center_publisher')
        
        # 테이블 토픽 구독자들
        self._subscriptions = []
        self.table_topics = [
            '/table_obstacles/table_1_2',
            '/table_obstacles/table_3_4',
            '/table_obstacles/table_5_6',
            '/table_obstacles/table_7_8'
        ]
        
        # 테이블 번호 매핑
        self.table_numbers = {
            '/table_obstacles/table_1_2': 1,
            '/table_obstacles/table_3_4': 2,
            '/table_obstacles/table_5_6': 3,
            '/table_obstacles/table_7_8': 4
        }
        
        # 퍼블리셔들
        self.publishers_ = {}
        for i in range(1, 5):
            self.publishers_[i] = self.create_publisher(PoseStamped, f'/table_centers/table_{i}', 10)
        
        # 구독자들 생성
        for topic in self.table_topics:
            sub = self.create_subscription(
                PolygonStamped,
                topic,
                lambda msg, t=topic: self.polygon_callback(msg, t),
                10
            )
            self._subscriptions.append(sub)
            
        self.get_logger().info('Table Center Publisher started')

    def polygon_callback(self, msg, topic):
        table_number = self.table_numbers[topic]
        points = msg.polygon.points
        
        if len(points) != 4:
            self.get_logger().warn(f'Table {table_number} polygon does not have 4 points. Skipping.')
            return
            
        # 중심점 계산
        cx = sum(point.x for point in points) / 4.0
        cy = sum(point.y for point in points) / 4.0
        cz = sum(point.z for point in points) / 4.0
        
        # 방향 계산 (첫 번째 점과 두 번째 점을 이용)
        # 테이블의 긴 변을 따라 있는 것으로 가정
        dx = points[1].x - points[0].x
        dy = points[1].y - points[0].y
        yaw = math.atan2(dy, dx)
        
        # PoseStamped 메시지 생성
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = msg.header.frame_id  # 'map'으로 가정
        
        pose_msg.pose.position.x = cx
        pose_msg.pose.position.y = cy
        pose_msg.pose.position.z = cz
        
        # 오일러 각을 쿼터니언으로 변환
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, yaw)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        
        # 퍼블리시
        self.publishers_[table_number].publish(pose_msg)
        self.get_logger().debug(f'Published center for table {table_number}: ({cx:.2f}, {cy:.2f}), yaw: {math.degrees(yaw):.1f}°')

def main(args=None):
    rclpy.init(args=args)
    node = TableCenterPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()