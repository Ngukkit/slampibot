#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImagePreprocessor(Node):
    def __init__(self):
        super().__init__('image_preprocessor')
        self.bridge = CvBridge()

        # 전처리된 이미지를 발행할 퍼블리셔
        self.publisher = self.create_publisher(Image, '/ceiling_camera/image_processed', 10)

        # 원본 카메라 이미지를 구독할 서브스크라이버
        self.subscription = self.create_subscription(
            Image,
            '/ceiling_camera/image_raw',  # usb_cam_node가 발행하는 원본 토픽
            self.image_callback,
            10
        )

        self.get_logger().info('Image Preprocessor Node (Hybrid Otsu+Adaptive) started.')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # --- Step 1: BGR → HSV 변환 ---
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # --- Step 2: 반사광 마스크 생성 ---
            # 채도(S)가 낮고 밝기(V)가 높은 영역을 반사광으로 간주
            lower_bound = np.array([0, 0, 240])
            upper_bound = np.array([255, 30, 255])
            mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

            # --- Step 3: Gray 변환 ---
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # --- Step 4: 반사광 영역은 흰색(255)으로 채우기 ---
            gray_image[mask > 0] = 255

            # --- Step 5: Gamma 보정 (밝기/대비 강화, 두께 불변) ---
            gamma = 1.2
            look_up_table = np.array([((i / 255.0) ** (1.0 / gamma)) * 255
                                      for i in np.arange(256)]).astype("uint8")
            corrected = cv2.LUT(gray_image, look_up_table)

            # --- Step 6: Gaussian Blur (노이즈 제거) ---
            blurred = cv2.GaussianBlur(corrected, (5, 5), 0)

            # --- Step 7a: Otsu 전역 threshold ---
            _, otsu = cv2.threshold(
                blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
            )

            # --- Step 7b: Adaptive threshold (국소 대비 보강) ---
            adaptive = cv2.adaptiveThreshold(
                blurred, 255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,
                21, 5  # blockSize=21, C=5 → 필요시 조정
            )

            # --- Step 7c: 두 결과를 AND 결합 (태그 강조, 배경 억제) ---
            final_processed_image = cv2.bitwise_and(otsu, adaptive)

            # --- Step 8: ROS 메시지로 변환 및 발행 ---
            processed_msg = self.bridge.cv2_to_imgmsg(final_processed_image, "mono8")
            processed_msg.header = msg.header  # 원본 메시지의 타임스탬프와 frame_id 유지
            self.publisher.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImagePreprocessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
