
import rclpy
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState # 배터리 상태 메시지 임포트
from turtlebot3_msgs.msg import SensorState # 터틀봇3 센서 상태 메시지 임포트 (필요시 설치)
import yaml # YAML 파일 파싱용
import os # 파일 경로 처리용
import time # 대기 시간용

from robot_commander_interfaces.srv import SendWaypoint # Import the custom service

class WaypointCommander(Node):
    def __init__(self):
        super().__init__('waypoint_commander')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.get_logger().info('Waypoint Commander Node started.')

        # 웨이포인트 YAML 파일 경로를 파라미터로 받도록 수정
        self.declare_parameter('waypoints_file', '')
        waypoints_file_path = self.get_parameter('waypoints_file').get_parameter_value().string_value

        if not waypoints_file_path:
            self.get_logger().error('waypoints_file parameter is not set.')
            return

        self.waypoints_data = self.load_waypoints_from_yaml(waypoints_file_path)
        if not self.waypoints_data:
            self.get_logger().error(f'Failed to load waypoints from {waypoints_file_path}')
            return

        # 1번 목적지 (베이스) 찾기
        self.base_waypoint = None
        for wp in self.waypoints_data:
            if wp['id'] == 1:
                self.base_waypoint = wp
                break
        
        if not self.base_waypoint:
            self.get_logger().error("Waypoint ID 1 (base) not found in waypoints.yaml. Please define it.")
            return

        # --- 센서 토픽 구독 --- 
        self.battery_state = None
        self.sensor_state = None

        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )
        self.sensor_sub = self.create_subscription(
            SensorState,
            '/sensor_state',
            self.sensor_callback,
            10
        )
        self.get_logger().info('Subscribing to /battery_state and /sensor_state')

        # Create the service server
        self.send_waypoint_service = self.create_service(
            SendWaypoint,
            'send_waypoint',
            self.send_waypoint_callback
        )
        self.get_logger().info('SendWaypoint service is ready.')

    def load_waypoints_from_yaml(self, yaml_file_path):
        if not os.path.exists(yaml_file_path):
            self.get_logger().error(f'Waypoint file not found: {yaml_file_path}')
            return None

        try:
            with open(yaml_file_path, 'r') as file:
                data = yaml.safe_load(file)
                if 'waypoints' in data:
                    return data['waypoints']
                else:
                    self.get_logger().error("YAML file does not contain a 'waypoints' key.")
                    return None
        except Exception as e:
            self.get_logger().error(f'Error loading waypoints YAML: {e}')
            return None

    def battery_callback(self, msg):
        self.battery_state = msg
        # --- 센서 정보 표시 --- 
        if self.battery_state:
            self.get_logger().info(f"[로봇 상태] 배터리 전압: {self.battery_state.voltage:.2f}V, 퍼센트: {self.battery_state.percentage:.1f}%")

    def sensor_callback(self, msg):
        self.sensor_state = msg
        if self.sensor_state:
            self.get_logger().info(f"[로봇 상태] 모터 토크: {self.sensor_state.torque}, 왼쪽 엔코더: {self.sensor_state.left_encoder}, 오른쪽 엔코더: {self.sensor_state.right_encoder}")

    def _send_single_waypoint_goal(self, waypoint_data):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map' # 지도의 프레임 ID
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        
        pose_stamped.pose.position.x = float(waypoint_data['pose']['position']['x'])
        pose_stamped.pose.position.y = float(waypoint_data['pose']['position']['y'])
        pose_stamped.pose.position.z = float(waypoint_data['pose']['position']['z'])
        
        pose_stamped.pose.orientation.x = float(waypoint_data['pose']['orientation']['x'])
        pose_stamped.pose.orientation.y = float(waypoint_data['pose']['orientation']['y'])
        pose_stamped.pose.orientation.z = float(waypoint_data['pose']['orientation']['z'])
        pose_stamped.pose.orientation.w = float(waypoint_data['pose']['orientation']['w'])
        
        goal_msg = FollowWaypoints.Goal()
        goal_msg.waypoints = [pose_stamped] # 단일 웨이포인트 전송
        
        self.get_logger().info(f'Sending goal to waypoint ID {waypoint_data['id']}: {waypoint_data['name']}')
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        rclpy.spin_until_future_complete(self, self._send_goal_future)
        goal_handle = self._send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return False
        
        self._get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self._get_result_future)
        
        result = self._get_result_future.result().result
        if result.status == FollowWaypoints.Result.SUCCEEDED:
            self.get_logger().info(f'Waypoint navigation to ID {waypoint_data['id']} succeeded!')
            return True
        else:
            self.get_logger().error(f'Waypoint navigation to ID {waypoint_data['id']} failed with status: {result.status}')
            return False

    def send_waypoint_callback(self, request, response):
        selected_id = request.waypoint_id
        self.get_logger().info(f'Received request to navigate to waypoint ID: {selected_id}')

        if selected_id == 1:
            self.get_logger().info("1번 목적지는 베이스입니다. 다른 목적지를 선택해주세요.")
            response.success = False
            return response

        selected_waypoint = None
        for wp in self.waypoints_data:
            if wp['id'] == selected_id:
                selected_waypoint = wp
                break
        
        if selected_waypoint:
            # 1. 선택된 목적지로 이동
            if self._send_single_waypoint_goal(selected_waypoint):
                self.get_logger().info(f"목적지 {selected_id}에 도착했습니다. 3초간 대기합니다.")
                time.sleep(3) # 3초 대기

                # 2. 베이스로 복귀
                self.get_logger().info(f"베이스 목적지 (ID 1: {self.base_waypoint['name']})로 복귀합니다.")
                if self._send_single_waypoint_goal(self.base_waypoint):
                    self.get_logger().info("베이스로 성공적으로 복귀했습니다.")
                    response.success = True
                else:
                    self.get_logger().error("베이스로 복귀하는 데 실패했습니다.")
                    response.success = False
            else:
                self.get_logger().error(f"목적지 {selected_id}로 이동하는 데 실패했습니다.")
                response.success = False
        else:
            self.get_logger().warn(f'Waypoint ID {selected_id} not found.')
            response.success = False
        
        return response

def main(args=None):
    rclpy.init(args=args)
    waypoint_commander = WaypointCommander()
    rclpy.spin(waypoint_commander)
    waypoint_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
