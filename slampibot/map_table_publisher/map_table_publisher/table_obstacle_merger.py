#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped

class TableObstacleMerger(Node):
    def __init__(self):
        super().__init__('table_obstacle_merger')
        
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
        
        # 각 테이블별로 최신 폴리곤 저장
        self.latest_polygons = {}
        
        for topic in self.table_topics:
            # 토픽 이름에서 테이블 식별자를 추출
            self.latest_polygons[topic] = None
            sub = self.create_subscription(
                PolygonStamped,
                topic,
                lambda msg, t=topic: self.polygon_callback(msg, t),
                10
            )
            self._subscriptions.append(sub)
        
        # 통합 토픽 발행자
        self.publisher = self.create_publisher(
            PolygonStamped,
            '/merged_table_obstacles',
            10
        )
        
        # 개별 테이블 퍼블리셔들
        self.individual_publishers = {}
        for i in range(1, 5):
            self.individual_publishers[i] = self.create_publisher(
                PolygonStamped,
                f'/individual_table_obstacles/table_{i}',
                10
            )
        
        # 타이머를 사용하여 주기적으로 병합된 폴리곤 발행
        self.timer = self.create_timer(0.1, self.publish_merged_polygons)
        
        self.get_logger().info('Table Obstacle Merger started')

    def polygon_callback(self, msg, topic):
        # 각 테이블의 최신 폴리곤 업데이트
        self.latest_polygons[topic] = msg
        self.get_logger().debug(f'Received polygon from {topic}')

    def publish_merged_polygons(self):
        # 개별 테이블 폴리곤 발행
        for topic, polygon in self.latest_polygons.items():
            if polygon is not None:
                # 테이블 번호 가져오기
                table_number = self.table_numbers[topic]
                
                # 개별 폴리곤을 발행
                individual_polygon = PolygonStamped()
                individual_polygon.header.stamp = self.get_clock().now().to_msg()
                individual_polygon.header.frame_id = "map"
                individual_polygon.polygon.points = polygon.polygon.points
                
                # 개별 폴리곤 발행
                self.individual_publishers[table_number].publish(individual_polygon)
                self.get_logger().debug(f'Published individual polygon for table {table_number}')
        
        # 유효한 폴리곤만 필터링
        valid_polygons = [poly for poly in self.latest_polygons.values() if poly is not None]
        
        if not valid_polygons:
            return
            
        # 모든 폴리곤을 병합
        merged_polygon = PolygonStamped()
        # 헤더는 현재 시간 사용
        merged_polygon.header.stamp = self.get_clock().now().to_msg()
        merged_polygon.header.frame_id = "map"
        
        # 모든 점들을 병합 (각 테이블은 4개의 점으로 구성됨)
        total_points = 0
        for polygon in valid_polygons:
            merged_polygon.polygon.points.extend(polygon.polygon.points)
            total_points += len(polygon.polygon.points)
            
        # 병합된 폴리곤 발행
        self.publisher.publish(merged_polygon)
        self.get_logger().info(f'Published merged polygon with {total_points} total points from {len(valid_polygons)} tables')

def main(args=None):
    rclpy.init(args=args)
    node = TableObstacleMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()