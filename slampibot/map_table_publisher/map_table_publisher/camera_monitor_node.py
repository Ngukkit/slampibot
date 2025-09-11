#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import threading
import time
from apriltag_msgs.msg import AprilTagDetectionArray

class CameraMonitor(Node):
    def __init__(self):
        super().__init__('camera_monitor')
        
        # AprilTag detections subscription
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.detection_callback,
            10)
        
        # Timer for checking camera status
        self.timer = self.create_timer(2.0, self.check_camera_status)
        
        # Status variables
        self.last_detection_time = time.time()
        self.camera_restarting = False
        
        self.get_logger().info('Camera Monitor Node started')
        
    def detection_callback(self, msg):
        # Update last detection time when any detection is received
        if len(msg.detections) > 0:
            self.last_detection_time = time.time()
            self.get_logger().info(f'Received {len(msg.detections)} detections')
            
    def check_camera_status(self):
        # Check if no detections received for more than 2 seconds
        current_time = time.time()
        time_since_last_detection = current_time - self.last_detection_time
        
        if time_since_last_detection > 2.0 and not self.camera_restarting:
            self.get_logger().warn(f'No detections for {time_since_last_detection:.1f} seconds. Restarting camera...')
            self.restart_camera()
            
    def restart_camera(self):
        self.camera_restarting = True
        
        # Run camera restart in a separate thread to avoid blocking
        restart_thread = threading.Thread(target=self._restart_camera_process)
        restart_thread.start()
        
    def _restart_camera_process(self):
        try:
            # Kill existing camera process
            subprocess.run(['pkill', '-f', 'usb_cam_node_exe'], check=True)
            self.get_logger().info('Killed existing camera process')
            
            # Wait a moment - reduced from 2 seconds to 0.5 seconds
            time.sleep(0.5)
            
            # Restart camera with the same parameters as in calibrated_persistent.launch.py
            cmd = [
                'ros2', 'run', 'usb_cam', 'usb_cam_node_exe',
                '--ros-args',
                '-r', '__node:=ceiling_camera',
                '-p', 'video_device:=/dev/video0',
                '-p', 'image_width:=1280',
                '-p', 'image_height:=720',
                '-p', 'framerate:=10.0',
                '-p', 'pixel_format:=yuyv2rgb',
                '-p', 'camera_name:=ceiling_camera',
                '-p', 'camera_info_url:=package://map_table_publisher/config/camera_info.yaml',
                '-p', 'io_method:=mmap',
                '-p', 'camera_frame_id:=default_cam',
                '-r', 'image_raw:=/ceiling_camera/image_raw',
                '-r', 'camera_info:=/ceiling_camera/camera_info'
            ]
            
            subprocess.Popen(cmd)
            self.get_logger().info('Restarted camera process')
            
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Error restarting camera: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error restarting camera: {e}')
        finally:
            # Reset the flag after some time to allow future restarts - reduced from 30 seconds to 5 seconds
            time.sleep(5)
            self.camera_restarting = False

def main(args=None):
    rclpy.init(args=args)
    node = CameraMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()