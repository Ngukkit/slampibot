#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import time
import numpy as np

class AprilTagTrackerNode(Node):
    def __init__(self):
        super().__init__('apriltag_tracker_node')

        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.detection_callback,
            10
        )
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            AprilTagDetectionArray,
            '/persistent_tag_detections',
            10
        )

        self.bridge = CvBridge()

        # tracker dictionary: tag_id -> cv2.Tracker instance
        self.trackers = {}
        self.tag_data = {}  # tag_id -> AprilTagDetection
        self.last_seen = {} # tag_id -> last_seen_time
        self.persist_time = 1.0  # seconds

        self.timer = self.create_timer(0.05, self.publish_tracked_tags)

        self.current_frame = None

    def image_callback(self, msg: Image):
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def detection_callback(self, msg: AprilTagDetectionArray):
        now = time.time()
        if self.current_frame is None:
            return

        for det in msg.detections:
            tag_id = det.id[0]

            # detection 기록
            self.tag_data[tag_id] = det
            self.last_seen[tag_id] = now

            # tracker 초기화
            if tag_id not in self.trackers:
                x, y, w, h = self.get_bbox(det)
                tracker = cv2.TrackerCSRT_create()
                tracker.init(self.current_frame, (x, y, w, h))
                self.trackers[tag_id] = tracker

    def get_bbox(self, det: AprilTagDetection):
        # 간단히 tag 코너 기준 bounding box
        corners = det.corners
        x_min = min(c[0] for c in corners)
        y_min = min(c[1] for c in corners)
        x_max = max(c[0] for c in corners)
        y_max = max(c[1] for c in corners)
        return int(x_min), int(y_min), int(x_max - x_min), int(y_max - y_min)

    def publish_tracked_tags(self):
        if self.current_frame is None:
            return

        now = time.time()
        persistent_msg = AprilTagDetectionArray()
        persistent_msg.header.stamp = self.get_clock().now().to_msg()

        to_delete = []

        for tag_id, tracker in self.trackers.items():
            if tag_id in self.tag_data:
                last_det_time = self.last_seen[tag_id]
                if now - last_det_time > self.persist_time:
                    # detection 없으면 tracker로 bbox 업데이트
                    success, bbox = tracker.update(self.current_frame)
                    if success:
                        # bbox를 기반으로 AprilTagDetection 객체 좌표 업데이트
                        det = self.tag_data[tag_id]
                        x, y, w, h = bbox
                        # bbox 중앙으로 대체 (간단화)
                        cx = x + w / 2
                        cy = y + h / 2
                        for corner in det.corners:
                            corner[0] += (cx - sum(c[0] for c in det.corners)/4)
                            corner[1] += (cy - sum(c[1] for c in det.corners)/4)
                        persistent_msg.detections.append(det)
                    else:
                        to_delete.append(tag_id)
                else:
                    # detection 존재 -> 그대로 append
                    persistent_msg.detections.append(self.tag_data[tag_id])
            else:
                to_delete.append(tag_id)

        # 오래된 tracker 제거
        for tag_id in to_delete:
            del self.trackers[tag_id]
            if tag_id in self.tag_data:
                del self.tag_data[tag_id]
            if tag_id in self.last_seen:
                del self.last_seen[tag_id]

        self.publisher.publish(persistent_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
