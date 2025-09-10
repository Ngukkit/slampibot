
import tf_transformations # For quaternion to euler conversion
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
import math # 수학 함수 사용을 위해 추가

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

        # Find the Kitchen waypoint (ID 1) to use as return destination
        self.kitchen_waypoint = None
        for wp in self.waypoints_data:
            if wp['id'] == 1:
                self.kitchen_waypoint = wp
                break
        
        if not self.kitchen_waypoint:
            self.get_logger().error("Waypoint ID 1 (kitchen) not found in waypoints.yaml. Please define it.")
            return

        # Find the Base waypoint (ID 2) 
        self.base_waypoint = None
        for wp in self.waypoints_data:
            if wp['id'] == 2:
                self.base_waypoint = wp
                break
        
        if not self.base_waypoint:
            self.get_logger().warn("Waypoint ID 2 (base) not found in waypoints.yaml. This is optional.")

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

        # Subscribers for table centers
        self.table_poses = {}
        self.table_center_subs = {}
        for i in range(1, 5): # For table 1 to table 4
            topic_name = f'/table_centers/table_{i}'
            self.table_center_subs[i] = self.create_subscription(
                PoseStamped,
                topic_name,
                lambda msg, table_num=i: self.table_center_callback(msg, table_num),
                10
            )
            self.get_logger().info(f'Subscribing to {topic_name} for table {i} center.')

        # Create the service server
        self.send_waypoint_service = self.create_service(
            SendWaypoint,
            'send_waypoint',
            self.send_waypoint_callback
        )
        self.get_logger().info('SendWaypoint service is ready.')

    def table_center_callback(self, msg, table_num):
        self.table_poses[table_num] = msg
        # self.get_logger().info(f"Received table {table_num} center: {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}")

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
        # if self.battery_state:
        #     self.get_logger().info(f"[로봇 상태] 배터리 전압: {self.battery_state.voltage:.2f}V, 퍼센트: {self.battery_state.percentage:.1f}%")

    def sensor_callback(self, msg):
        self.sensor_state = msg
        # if self.sensor_state:
        #     self.get_logger().info(f"[로봇 상태] 모터 토크: {self.sensor_state.torque}, 왼쪽 엔코더: {self.sensor_state.left_encoder}, 오른쪽 엔코더: {self.sensor_state.right_encoder}")

    def _send_single_waypoint_goal(self, goal_pose_stamped, goal_name=""):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [goal_pose_stamped] # 단일 웨이포인트 전송
        
        self.get_logger().info(f"Sending goal to {goal_name}: ({goal_pose_stamped.pose.position.x:.2f}, {goal_pose_stamped.pose.position.y:.2f})")
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
            self.get_logger().info(f"Navigation to {goal_name} succeeded!")
            return True
        else:
            self.get_logger().error(f"Navigation to {goal_name} failed with status: {result.status}")
            return False

    def send_waypoint_callback(self, request, response):
        selected_id = request.waypoint_id
        self.get_logger().info(f'Received request to navigate to waypoint ID: {selected_id}')

        # Handle table waypoints (IDs 3-6 map to tables 1-4)
        if selected_id >= 3 and selected_id <= 6:
            table_num = selected_id - 2  # ID 3 maps to table 1, ID 4 to table 2, etc.
            goal_name = f"Table {table_num}"
            if table_num in self.table_poses:
                table_center_pose = self.table_poses[table_num]
                # Calculate offset goal pose
                offset_distance = 0.4 # meters, adjust as needed (was 0.5)
                
                # Get table orientation (yaw)
                q = table_center_pose.pose.orientation
                (roll, pitch, yaw) = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
                
                # Calculate position offset away from the table center
                # This means offsetting along the direction opposite to the table's yaw
                approach_yaw = yaw + math.pi # Opposite direction
                
                offset_x = offset_distance * math.cos(approach_yaw)
                offset_y = offset_distance * math.sin(approach_yaw)
                
                goal_pose_stamped = PoseStamped()
                goal_pose_stamped.header.frame_id = 'map'
                goal_pose_stamped.header.stamp = self.get_clock().now().to_msg()
                
                goal_pose_stamped.pose.position.x = float(table_center_pose.pose.position.x + offset_x)
                goal_pose_stamped.pose.position.y = float(table_center_pose.pose.position.y + offset_y)
                goal_pose_stamped.pose.position.z = float(0.0) # Ensure Z is 0
                
                # Keep the robot's orientation facing the table (or aligned with table's yaw)
                goal_pose_stamped.pose.orientation = table_center_pose.pose.orientation
                
            else:
                self.get_logger().warn(f'Table {table_num} center not received yet.')
                response.success = False
                return response
        # Handle Kitchen waypoint (ID 1)
        elif selected_id == 1:
            # Navigate to Kitchen
            goal_name = self.kitchen_waypoint['name']
            goal_pose_stamped = PoseStamped()
            goal_pose_stamped.header.frame_id = 'map'
            goal_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            goal_pose_stamped.pose.position.x = float(self.kitchen_waypoint['pose']['position']['x'])
            goal_pose_stamped.pose.position.y = float(self.kitchen_waypoint['pose']['position']['y'])
            goal_pose_stamped.pose.position.z = float(self.kitchen_waypoint['pose']['position']['z'])
            goal_pose_stamped.pose.orientation.x = float(self.kitchen_waypoint['pose']['orientation']['x'])
            goal_pose_stamped.pose.orientation.y = float(self.kitchen_waypoint['pose']['orientation']['y'])
            goal_pose_stamped.pose.orientation.z = float(self.kitchen_waypoint['pose']['orientation']['z'])
            goal_pose_stamped.pose.orientation.w = float(self.kitchen_waypoint['pose']['orientation']['w'])
        # Handle Base waypoint (ID 2)
        elif selected_id == 2:
            if self.base_waypoint:
                goal_name = self.base_waypoint['name']
                goal_pose_stamped = PoseStamped()
                goal_pose_stamped.header.frame_id = 'map'
                goal_pose_stamped.header.stamp = self.get_clock().now().to_msg()
                goal_pose_stamped.pose.position.x = float(self.base_waypoint['pose']['position']['x'])
                goal_pose_stamped.pose.position.y = float(self.base_waypoint['pose']['position']['y'])
                goal_pose_stamped.pose.position.z = float(self.base_waypoint['pose']['position']['z'])
                goal_pose_stamped.pose.orientation.x = float(self.base_waypoint['pose']['orientation']['x'])
                goal_pose_stamped.pose.orientation.y = float(self.base_waypoint['pose']['orientation']['y'])
                goal_pose_stamped.pose.orientation.z = float(self.base_waypoint['pose']['orientation']['z'])
                goal_pose_stamped.pose.orientation.w = float(self.base_waypoint['pose']['orientation']['w'])
            else:
                self.get_logger().warn("Base waypoint not defined in waypoints.yaml")
                response.success = False
                return response
        else: # Existing waypoint logic for IDs > 6
            selected_waypoint = None
            goal_name = ""
            for wp in self.waypoints_data:
                if wp['id'] == selected_id:
                    selected_waypoint = wp
                    goal_name = selected_waypoint['name']
                    break
            
            if selected_waypoint:
                goal_pose_stamped = PoseStamped()
                goal_pose_stamped.header.frame_id = 'map'
                goal_pose_stamped.header.stamp = self.get_clock().now().to_msg()
                
                goal_pose_stamped.pose.position.x = float(selected_waypoint['pose']['position']['x'])
                goal_pose_stamped.pose.position.y = float(selected_waypoint['pose']['position']['y'])
                goal_pose_stamped.pose.position.z = float(selected_waypoint['pose']['position']['z'])
                
                goal_pose_stamped.pose.orientation.x = float(selected_waypoint['pose']['orientation']['x'])
                goal_pose_stamped.pose.orientation.y = float(selected_waypoint['pose']['orientation']['y'])
                goal_pose_stamped.pose.orientation.z = float(selected_waypoint['pose']['orientation']['z'])
                goal_pose_stamped.pose.orientation.w = float(selected_waypoint['pose']['orientation']['w'])
            else:
                self.get_logger().warn(f'Waypoint ID {selected_id} not found.')
                response.success = False
                return response
        
        if goal_pose_stamped:
            # Send the goal
            if self._send_single_waypoint_goal(goal_pose_stamped, goal_name):
                # If we went to a table (IDs 3-6), wait and then return to kitchen
                if selected_id >= 3 and selected_id <= 6:
                    self.get_logger().info(f"목적지 {goal_name}에 도착했습니다. 3초간 대기합니다.")
                    time.sleep(3) # 3초 대기

                    # Return to Kitchen instead of base
                    self.get_logger().info(f"주방 목적지 (ID 1: {self.kitchen_waypoint['name']})로 복귀합니다.")
                    kitchen_pose_stamped = PoseStamped()
                    kitchen_pose_stamped.header.frame_id = 'map'
                    kitchen_pose_stamped.header.stamp = self.get_clock().now().to_msg()
                    kitchen_pose_stamped.pose.position.x = float(self.kitchen_waypoint['pose']['position']['x'])
                    kitchen_pose_stamped.pose.position.y = float(self.kitchen_waypoint['pose']['position']['y'])
                    kitchen_pose_stamped.pose.position.z = float(self.kitchen_waypoint['pose']['position']['z'])
                    kitchen_pose_stamped.pose.orientation.x = float(self.kitchen_waypoint['pose']['orientation']['x'])
                    kitchen_pose_stamped.pose.orientation.y = float(self.kitchen_waypoint['pose']['orientation']['y'])
                    kitchen_pose_stamped.pose.orientation.z = float(self.kitchen_waypoint['pose']['orientation']['z'])
                    kitchen_pose_stamped.pose.orientation.w = float(self.kitchen_waypoint['pose']['orientation']['w'])

                    if self._send_single_waypoint_goal(kitchen_pose_stamped, self.kitchen_waypoint['name']):
                        self.get_logger().info("주방으로 성공적으로 복귀했습니다.")
                        response.success = True
                    else:
                        self.get_logger().error("주방으로 복귀하는 데 실패했습니다.")
                        response.success = False # Update response if return to kitchen fails
                else:
                    # For kitchen, base or other waypoints, just report success
                    self.get_logger().info(f"Successfully reached {goal_name}.")
                    response.success = True
            else:
                self.get_logger().error(f"목적지 {goal_name}로 이동하는 데 실패했습니다.")
                response.success = False
        else:
            self.get_logger().error("No valid goal pose could be determined.")
            response.success = False
        
        return response
        
        return response
        
        return response

def main(args=None):
    rclpy.init(args=args)
    waypoint_commander = WaypointCommander()
    rclpy.spin(waypoint_commander)
    waypoint_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
