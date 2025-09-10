#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
import time

class AprilTagPersistenceNode(Node):
    def __init__(self):
        super().__init__('apriltag_persistence_node')

        # 구독: 기존 AprilTag detection 토픽
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.detection_callback,
            10
        )

        # 퍼블리시: Nav2가 읽을 토픽
        self.publisher = self.create_publisher(
            AprilTagDetectionArray,
            '/persistent_tag_detections',
            10
        )

        # persistence dictionary
        # tag_id -> (AprilTagDetection, last_seen_time)
        self.persisted_tags = {}
        self.persist_time = 10.0  # 초 단위 유지 시간

        # timer: 주기적으로 publish
        self.timer = self.create_timer(0.05, self.publish_persistent_tags)

    def detection_callback(self, msg: AprilTagDetectionArray):
        now = time.time()
        for det in msg.detections:
            tag_id = det.id  # det.id는 이미 int형입니다.
            self.persisted_tags[tag_id] = (det, now)

    def publish_persistent_tags(self):
        now = time.time()
        persistent_msg = AprilTagDetectionArray()
        persistent_msg.header.stamp = self.get_clock().now().to_msg()
        to_delete = []

        for tag_id, (det, last_seen) in self.persisted_tags.items():
            if now - last_seen <= self.persist_time:
                persistent_msg.detections.append(det)
            else:
                to_delete.append(tag_id)

        # 오래된 tag 제거
        for tag_id in to_delete:
            del self.persisted_tags[tag_id]

        self.publisher.publish(persistent_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPersistenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
