# slampibot_gazebo - 로봇 구동 및 시뮬레이션 패키지

## 버전: v0.18

이 패키지는 4륜 구동 로봇의 구동 및 시뮬레이션을 위한 ROS2 드라이버와 설정 파일을 포함합니다. v0.18에서는 TurtleBot3 펌웨어를 4바퀴 지원으로 수정하여, 모든 바퀴가 동기화되어 안정적으로 동작하도록 개선했습니다.

### v0.18: TurtleBot3 4바퀴 펌웨어 수정

*   **문제점 발견**: 기본 TurtleBot3 펌웨어는 2바퀴만 지원하여 4바퀴 로봇에서 뒷바퀴가 동작하지 않는 문제가 발생했습니다. 또한 Sync Write/Read 방식으로 인한 통신 충돌과 관성으로 인한 추가 회전 문제가 있었습니다.
*   **원인 분석**:
    *   **2바퀴 펌웨어**: 기본 펌웨어는 모터 ID 1, 2번만 제어하여 뒷바퀴(3, 4번)가 동작하지 않음
    *   **Sync Write 충돌**: 4바퀴 동시 제어 시 통신 충돌로 인한 불안정성
    *   **관성 회전**: 정지 명령 후 뒷바퀴가 관성으로 추가 회전하는 문제
*   **해결 방안**:
    *   **개별 제어 방식 도입**: Sync Write/Read에서 개별 Raw Write/Read로 변경하여 통신 충돌 해결
    *   **Profile Acceleration 조정**: 100ms로 설정하여 적절한 가속/감속 구현
    *   **BUS_WATCHDOG 활성화**: 100ms로 설정하여 통신 문제 시 자동 정지
    *   **토크 해제 순서 최적화**: 뒷바퀴 먼저 정지하여 관성 회전 방지
*   **현재 상태**: 4바퀴 모두 동일한 제어 방식으로 동작하며, 안전 기능이 강화되어 통신 문제 시 자동 정지됩니다. 모든 바퀴가 동기화되어 안정적인 주행이 가능합니다.

### v0.17: AprilTag 정적 감지 안정성 확보

*   **문제점 발견**: 천장 카메라를 이용한 AprilTag 기반 위치 추정 시, 로봇이나 태그가 정지해 있는 상황에서 `apriltag_node`가 태그를 지속적으로 감지하지 못하는 문제가 발생했습니다. 이로 인해 `/tf` 및 `/detections` 토픽 발행이 중단되어 Nav2의 위치 추정이 불안정해졌습니다.
*   **원인 분석**:
    *   **환경적 요인**: 가장 근본적인 원인은 검은색 바닥과 같이 대비가 낮은 환경이었습니다. 정적인 이미지에서 AprilTag의 엣지(edge)를 감지하기 어려워 알고리즘이 실패하는 경우가 많았습니다.
    *   **TF 트리 및 파라미터 오류**: 디버깅 과정에서 `tf2_monitor`를 통해 `camera` vs `ceiling_camera`와 같은 프레임 이름 불일치 문제를 발견하고 수정했습니다. `apriltag_node`의 `camera_frame` 파라미터 설정이 중요했습니다.
*   **해결 방안**:
    *   **환경 개선 (근본 해결책)**: 태그가 잘 인식될 수 있도록 바닥의 대비를 높이는 등 환경을 개선하는 것이 가장 효과적인 해결책임을 확인했습니다.
    *   **`apriltag_persistence_node` 도입 (워크어라운드)**: 환경 개선 후에도 `apriltag_node`의 내부 최적화로 인해 발생할 수 있는 간헐적 감지 문제를 해결하기 위해 `apriltag_persistence_node`를 도입했습니다. 이 노드는 마지막으로 감지된 태그의 위치를 일정 시간 동안 유지(cache)하여 Nav2에 안정적이고 지속적인 위치 정보를 제공합니다. 이를 통해 로봇이 정지 상태에서도 자신의 위치를 잃지 않게 됩니다.
*   **현재 상태**: 시스템은 이제 환경적 요인에 대응하고, `apriltag_persistence_node`를 통해 `apriltag_node`로부터 안정적인 태그 데이터를 Nav2에 제공하도록 구성되었습니다. 더 고급 대안으로 `apriltag_tracker_node`도 고려할 수 있습니다.

### v0.16: AprilTag 기반 테이블 인식 기능

*   **기능**: `map_table_publisher`와 `table_obstacle_layer` 패키지를 이용하여 테이블과 같은 동적 장애물을 인식하고, 내비게이션 맵에 실시간으로 반영합니다.
*   **작동 방식**:
    1.  **AprilTag 검출**: `apriltag_ros` 노드가 카메라 영상에서 AprilTag를 검출합니다. (`launch/my_apriltag.launch.py`)
    2.  **테이블 영역 계산**: `map_table_publisher` 패키지의 `tag_publisher_node`가 검출된 태그 2개의 위치를 기반으로 테이블의 영역(Polygon)을 계산하여 `/table_obstacle` 토픽으로 발행합니다.
    3.  **Costmap 업데이트**: `table_obstacle_layer` (Costmap 플러그인)가 `/table_obstacle` 토픽을 구독하여, Nav2의 Costmap에 해당 영역을 장애물로 추가합니다. 이를 통해 로봇은 테이블을 회피하여 주행할 수 있습니다.
*   **주요 가정 및 설정**:
    *   `tag_publisher_node`는 두 개의 AprilTag가 테이블의 **대각선 모서리**에 위치한다고 가정하고 테이블의 크기를 계산합니다.
    *   카메라 토픽은 `launch/my_apriltag.launch.py` 파일 내에서 `/camera1/image_raw`로 지정되어 있습니다.
    *   AprilTag의 실제 크기는 `config/table_params.yaml` 파일에서 설정합니다 (기본값: 5cm).
*   **실행 방법**:
    1.  AprilTag 검출 노드를 실행합니다: `ros2 launch map_table_publisher my_apriltag.launch.py`
    2.  테이블 인식 및 발행 노드를 실행합니다: `ros2 run map_table_publisher tag_publisher_node`
    3.  Nav2 스택을 `table_obstacle_layer` 플러그인이 활성화된 설정으로 실행합니다.


## 1. 프로젝트 목표

*   **실제 로봇 구동**: Raspberry Pi 5, OpenCR 1.0 보드, Dynamixel XL430 모터, 라이다, 카메라를 사용하여 4륜 구동 로봇을 제어합니다.
*   **시뮬레이션**: Gazebo 환경에서 로봇 모델을 시뮬레이션하고 제어합니다.
*   **기구학 지원**: 4륜 독립 구동 로봇의 역기구학 및 순기구학 계산을 지원하여 다양한 움직임(전진, 횡이동, 회전)을 구현합니다.
*   **서빙 로봇 기능**: 웨이포인트 내비게이션을 통해 특정 목적지(숫자로 선택)로 이동하고 복귀하는 기능을 구현합니다.

## 2. 주요 구성 요소

*   **하드웨어**: Raspberry Pi 5, OpenCR 1.0, Dynamixel XL430 (4개), 라이다, 카메라
*   **통신**: USB-Serial 통신 (rosserial_python 기반)
*   **ROS2 배포판**: Humble (예상)

## 3. 파일 구조 및 현재 상태 (v0.15 기준)

### 3.1. 로봇 모델 정의 (URDF)

*   **`myCar/real_BMW_core.xacro`**: 실제 로봇의 물리적 치수(몸통, 바퀴)와 센서 위치를 반영한 핵심 URDF 파일입니다. 시뮬레이션 전용 태그는 모두 제거되었습니다.
*   **`myCar/real_BMW.urdf.xacro`**: `real_BMW_core.xacro`를 포함하는 실제 로봇의 메인 URDF 파일입니다.
*   **`myCar/real_BMW_core.urdf`**: `real_BMW_core.xacro`를 `xacro` 명령으로 처리하여 생성된 순수 URDF 파일입니다. FreeCAD와 같은 외부 툴에서 로봇 모델을 시각적으로 확인할 때 사용됩니다.

### 3.2. ROS2 Launch 파일

*   **`launch/real_robot.launch.py`**: 실제 로봇 구동을 위한 Launch 파일입니다. `rosserial_python`을 사용하여 OpenCR 보드와 통신하고, `robot_state_publisher`, 라이다/카메라 드라이버를 실행합니다.
*   **`launch/real_nav.launch.py`**: 실제 로봇에서 내비게이션 스택(Nav2)을 구동하기 위한 Launch 파일입니다. `rosserial_python`을 포함하며, 웨이포인트 내비게이션 기능을 위한 `WaypointCommander` 노드를 포함합니다.
*   **`launch/spb_spawn_space.launch.py`**: Gazebo 시뮬레이션 구동을 위한 Launch 파일입니다. `gazebo_ros_diff_drive` 플러그인을 사용하여 4륜 스키드 스티어 로봇을 시뮬레이션합니다.

### 3.3. 로봇 드라이버 코드

*   **`src/sim_drive_node.cpp`**: Gazebo 없이 RViz에서 로봇의 움직임을 시뮬레이션하기 위한 간단한 차동 구동 노드입니다. (현재는 `spb_spawn_space.launch.py`에서 사용되지 않음)
*   **`src/EduDrive.h`, `src/EduDrive.cpp` 등 EduDrive 시스템 관련 파일**: `real_driver_node`와 `ros2_control` 기반의 `DynamixelHardwareInterface` 플러그인 개발 과정에서 사용되었으나, 최종적으로 `turtlebot3_core.ino` 펌웨어와 `rosserial_python` 방식으로 전환하면서 **더 이상 사용되지 않습니다.** (파일은 삭제됨)
*   **`src/hardware_interface` 디렉토리**: `ros2_control` 기반의 `DynamixelHardwareInterface` 플러그인 개발 과정에서 사용되었으나, 최종적으로 `turtlebot3_core.ino` 펌웨어와 `rosserial_python` 방식으로 전환하면서 **더 이상 사용되지 않습니다.** (디렉토리 삭제됨)
*   **`src/dynamixel` 디렉토리**: `DynamixelSerialPort` 클래스 개발 과정에서 사용되었으나, 최종적으로 `turtlebot3_core.ino` 펌웨어와 `rosserial_python` 방식으로 전환하면서 **더 이상 사용되지 않습니다.** (디렉토리 삭제됨)

### 3.4. 빌드 시스템 및 파라미터

*   **`real_CMakeLists.txt`**: `rosserial_python` 방식에서는 별도의 C++ 노드를 빌드하지 않으므로, 최소한의 설정만 포함합니다.
*   **`CMakeLists.txt`**: Gazebo 시뮬레이션 구동을 위한 CMake 파일입니다.
*   **`parameter/edu_drive_edu_bot.yaml`**: `real_driver_node` (EduDrive 기반)에 전달되는 파라미터들을 정의합니다. (현재 `rosserial_python` 방식에서는 직접 사용되지 않음)
*   **`parameter/ros_controllers.yaml`**: `ros2_control` 기반의 `diff_drive_controller` 설정을 정의합니다. (현재 `rosserial_python` 방식에서는 직접 사용되지 않음)
*   **`parameter/spb_nav2_params.yaml`**: 내비게이션 스택(Nav2)에 전달되는 파라미터들을 정의합니다. 차동 구동 로봇에 적합한 설정으로 시작합니다.
*   **`parameter/waypoints.yaml`**: 서빙 로봇의 목적지(웨이포인트)들을 정의하는 파일입니다. 각 웨이포인트는 ID, 이름, 로봇의 위치 및 방향 정보를 포함합니다.
*   **`parameter/README.md`**: `edu_drive_edu_bot.yaml` 파일의 내용과 각 파라미터의 의미를 설명하는 한국어 문서입니다.

### 3.5. 기타 문서

*   **`README.md` (현재 디렉토리)**: Raspberry Pi 5에서 Docker를 사용하여 CAN 통신을 설정하는 방법에 대한 한국어 가이드입니다. (현재 프로젝트의 통신 방식과는 다름을 유의)
*   **`Robot_Make_Order_Readme.md`**: 실제 로봇 제작 주문 전에 준비해야 할 소프트웨어 관련 사항들을 요약한 문서입니다.

## 4. 프로젝트 개발 여정 및 버전별 변경 이력

이 프로젝트는 4륜 구동 로봇의 제어를 목표로 여러 접근 방식을 시도하며 발전해 왔습니다.

### v0.16 (이전 버전)

*   **동적 장애물(테이블) 인식 기능 추가**: 천장 카메라와 AprilTag를 이용하여 테이블과 같은 동적 장애물을 인식하고, 이를 내비게이션 Costmap에 실시간으로 반영하는 기능이 추가되었습니다.
    *   **`apriltag_ros`**: 카메라 영상에서 AprilTag를 검출합니다.
    *   **`map_table_publisher`**: 검출된 태그 위치를 기반으로 테이블의 영역(Polygon)을 계산하여 `/table_obstacle` 토픽으로 발행합니다.
    *   **`table_obstacle_layer`**: `/table_obstacle` 토픽을 구독하여 Nav2 Costmap에 장애물 레이어를 추가하는 플러그인입니다.
*   **관련 파라미터 및 런치 파일**: `table_params.yaml` (태그 크기 설정), `my_apriltag.launch.py` (AprilTag 노드 실행) 등이 추가되었습니다.

### v0.15 (이전 버전)

*   **서빙 로봇 기능 구현**: `nav2_waypoint_follower`를 활용한 웨이포인트 내비게이션 기능이 추가되었습니다. 사용자가 터미널에서 목적지 번호를 선택하면, 로봇이 해당 목적지로 이동하고 3초간 대기한 후 베이스 위치(ID 1)로 복귀하는 코스를 수행합니다.
    *   **`program/src/robot_commander/robot_commander/waypoint_commander_node.py`**: 사용자 인터페이스 및 웨이포인트 관리 노드.
    *   **`parameter/waypoints.yaml`**: 로봇의 목적지(웨이포인트)들을 정의하는 파일.
    *   **`launch/real_nav.launch.py`**: `WaypointCommander` 노드를 포함하도록 업데이트되었습니다.
*   **로봇 상태 표시**: `WaypointCommander` 노드에 배터리 전압, 모터 토크, 엔코더 값 등 로봇의 현재 상태를 터미널에 주기적으로 표시하는 기능이 추가되었습니다.
*   **웹 기반 제어 인터페이스 추가**: `robot_web_interface` 패키지를 통해 웹 브라우저에서 로봇의 웨이포인트를 제어할 수 있는 인터페이스가 추가되었습니다.
    *   `robot_web_interface` 패키지는 Flask 웹 서버를 사용하여 `index.html`을 제공하고, ROS2 서비스를 통해 `waypoint_commander_node`와 통신합니다.
    *   이제 웨이포인트 1번(복귀 위치)을 포함하여 웹 인터페이스에서 직접 웨이포인트를 선택하여 로봇을 제어할 수 있습니다.

### v0.14 (이전 버전)

*   **실제 로봇 구동 방식 확정**: `turtlebot3_core.ino` 펌웨어와 `rosserial_python`을 사용하는 방식으로 최종 확정되었습니다. 이에 따라 `real_CMakeLists.txt` 및 관련 C++ 드라이버 코드(EduDrive, ros2_control 하드웨어 인터페이스)는 제거되었습니다.
*   **시뮬레이션 환경 안정화**: `gazebo_ros_diff_drive` 플러그인을 사용하여 Gazebo에서 4륜 스키드 스티어 로봇의 움직임을 성공적으로 시뮬레이션하고 RViz와 연동하는 것이 확인되었습니다.
*   **Nav2 런처 추가**: 실제 로봇에서 Nav2 스택을 구동하기 위한 `launch/real_nav.launch.py` 파일이 추가되었습니다.
*   **파라미터 파일 업데이트**: `parameter/edu_drive_edu_bot.yaml`이 차동 구동 로봇의 기구학에 맞게 업데이트되었으며, `parameter/spb_nav2_params.yaml`이 Nav2 스택의 기본 파라미터로 추가되었습니다.

### v0.13 (이전 버전)

*   **초기 접근 (EduDrive 시스템)**:
    *   기존 `EduDrive` 시스템을 활용하여 로봇을 제어하고자 했습니다. 이 시스템은 CAN 통신을 기반으로 하며, `kinematics` 파라미터를 통해 4륜 독립 구동을 지원하는 구조를 가지고 있었습니다.
    *   하지만, 실제 하드웨어(Dynamixel XL430, OpenCR 1.0)가 CAN HAT 없이 USB-Serial 통신을 사용해야 하는 상황에 직면했습니다.
*   **USB-Serial 통신 전환 및 `usb_to_dxl` 시도**:
    *   CAN 통신 대신 USB-Serial 통신을 사용하기 위해 `EduDrive` 시스템을 `DynamixelSerialPort` 기반으로 수정하고 `Dynamixel SDK`를 통합했습니다.
    *   OpenCR 보드에는 `usb_to_dxl.ino` 펌웨어를 사용하고자 했습니다. 이 펌웨어는 OpenCR을 투명한 USB-Serial 브릿지로 작동시켜 라즈베리파이에서 Dynamixel 모터를 직접 제어할 수 있게 합니다.
    *   이 단계에서 `ros2_control` 기반의 `DynamixelHardwareInterface` 플러그인 구현도 시도했습니다. 이는 `ros2_control`의 표준 방식을 따르기 위함이었습니다.
*   **기구학적 이해의 전환**:
    *   4륜 로봇의 물리적 구조(일반 바퀴)로는 `linear.y`를 통한 횡이동이 불가능하며, `diff` (차동 구동) 방식처럼 동작한다는 것을 확인했습니다. 즉, 4륜의 개별 구동이 물리적으로 의미가 없다는 것을 깨달았습니다. 이로 인해 `ros2_control`의 `diff_drive_controller`를 사용하는 것이 합리적이라는 결론에 도달했습니다.
*   **OpenCR 센서 데이터의 필요성 및 펌웨어 전환**:
    *   `usb_to_dxl` 펌웨어는 모터 제어만 가능하고 OpenCR 보드 자체의 IMU, 전압, 전류 등 중요한 센서 데이터를 노출하지 않는다는 한계에 직면했습니다.
    *   로봇의 정확한 오도메트리, 내비게이션, 상태 모니터링을 위해 이러한 센서 데이터가 필수적임을 인지했습니다.
    *   **최종 결정**: OpenCR 보드 자체의 센서 데이터를 ROS2로 가져오기 위해 `turtlebot3_core.ino` 펌웨어를 사용하기로 결정했습니다. 이 펌웨어는 OpenCR을 ROS 노드처럼 작동시켜 USB-Serial 통신을 통해 다양한 센서 데이터(IMU, 전압, 전류, 오도메트리, 조인트 상태 등)를 직접 발행합니다.
*   **상태 (v0.13)**:
    *   **실제 로봇 구동**: OpenCR 보드에 `turtlebot3_core.ino` 펌웨어를 업로드하고, 라즈베리파이에서는 `rosserial_python` 노드를 사용하여 OpenCR과 통신합니다. OpenCR이 ROS 토픽을 직접 발행/구독하므로, 라즈베리파이 측의 복잡한 C++ 드라이버 코드(EduDrive, ros2_control 하드웨어 인터페이스)는 더 이상 필요 없습니다.
    *   **시뮬레이션**: `gazebo_ros_diff_drive` 플러그인을 사용하여 4륜 스키드 스티어 로봇의 움직임을 Gazebo에서 성공적으로 시뮬레이션하고 제어할 수 있습니다. RViz에서도 로봇 모델이 올바르게 표시되고 움직입니다.


## 5. 다음 단계

1.  **`rosserial_python` 설치**: Raspberry Pi에 `rosserial_python` 패키지를 설치합니다.
2.  **OpenCR 1.0에 `turtlebot3_core.ino` 펌웨어 업로드**: OpenCR 보드에 이 펌웨어를 업로드합니다.
3.  **라이다 및 카메라 드라이버 설치**: Raspberry Pi에 라이다 및 카메라 모델에 맞는 ROS2 드라이버를 설치합니다.
4.  **빌드 및 실행**: `real_CMakeLists.txt`는 이제 빌드할 C++ 코드가 없으므로, `colcon build`는 패키지를 빌드하지 않을 것입니다. 이후 다음 런처를 실행합니다.
    *   기본 구동: `ros2 launch slampibot_gazebo ros_robot.launch.py`
    *   내비게이션: `ros2 launch slampibot_gazebo real_nav.launch.py`

이러한 준비가 완료되면, 실제 로봇을 구동하고 센서 데이터를 받아보며 내비게이션 및 SLAM 개발을 시작할 수 있을 것입니다.

## 6. 웹 인터페이스 실행

로봇의 웹 기반 제어 인터페이스를 사용하려면 다음 단계를 따르세요:

1.  **서비스 노드 실행**: `waypoint_commander_node`를 실행하여 `send_waypoint` 서비스를 활성화합니다.
    ```bash
    ros2 launch robot_commander waypoint_commander.launch.py
    ```
    이 노드는 백그라운드에서 실행되어야 합니다.

2.  **웹 애플리케이션 실행**: `robot_web_interface` 패키지의 Flask 웹 서버를 실행합니다.
    ```bash
    source /home/pi/ros2_ws/install/setup.bash && python3 /home/pi/ros2_ws/src/robot_web_interface/robot_web_interface/app.py &
    ```
    이 명령도 백그라운드에서 실행되어야 합니다.

3.  **웹 인터페이스 접속**: 웹 브라우저를 열고 다음 주소로 접속합니다.
    ```
    http://<로봇_IP_주소>:5000
    ```
    (예: `http://192.168.1.100:5000` 또는 로컬에서 테스트 시 `http://localhost:5000`)

## 7. TurtleBot3 4바퀴 펌웨어 수정 (v0.18)

### 개요
기본 TurtleBot3 펌웨어는 2바퀴만 지원하므로, 4바퀴 로봇을 위해 펌웨어를 수정했습니다.

### 주요 수정 사항

#### 4바퀴 동기화 제어 개선
- **모든 바퀴를 개별 Raw Write/Read로 통일**
  - 앞바퀴(1,2번)와 뒷바퀴(3,4번) 모두 동일한 제어 방식 사용
  - Sync Write/Read에서 개별 Raw 통신으로 변경하여 동기화 문제 해결

#### 모터 제어 설정 최적화
- **Profile Acceleration**: 100ms로 설정 (적절한 가속/감속)
- **BUS_WATCHDOG**: 100ms로 활성화 (통신 문제 시 자동 정지)
- **토크 해제 순서**: 뒷바퀴 먼저 정지 → 앞바퀴 정지 (관성 회전 방지)

#### 안전 기능 강화
- **토크 비활성화 전 속도 0 설정**: 모든 바퀴를 먼저 정지 후 토크 해제
- **4바퀴 연결 상태 확인**: 각 바퀴별 연결 상태 출력
- **에러 로깅**: 모든 모터 명령에 에러 체크 및 로그 출력

#### 제거된 기능
- **매니퓰레이터 관련 코드**: Burger 모델 전용으로 정리
- **부저 소리**: 모든 알림음 비활성화
- **Sync Write/Read**: 4바퀴 동기화 문제로 인한 제거

### 개발 과정 및 문제 해결

#### 초기 문제점
- **2바퀴 펌웨어**: 기본 TurtleBot3 펌웨어는 2바퀴만 지원하여 뒷바퀴가 동작하지 않음
- **Sync Write 충돌**: 4바퀴 동시 제어 시 통신 충돌로 인한 불안정성
- **관성 회전**: 정지 명령 후 뒷바퀴가 관성으로 추가 회전하는 문제

#### 해결 과정
1. **개별 제어 방식 도입**: Sync Write/Read에서 개별 Raw Write/Read로 변경
2. **Profile Acceleration 조정**: 0 → 100ms로 설정하여 적절한 감속 구현
3. **BUS_WATCHDOG 활성화**: 100ms로 설정하여 통신 문제 시 자동 정지
4. **토크 해제 순서 최적화**: 뒷바퀴 먼저 정지하여 관성 회전 방지

#### 핵심 개선사항
- **Raw 통신 함수**: `wr8()`, `wr16()`, `wr32()`, `rd8()`, `rd16()`, `rd32()` 사용
- **에러 로깅**: 모든 모터 명령에 실패 시 로그 출력
- **4바퀴 동기화**: 앞바퀴(1,2번)와 뒷바퀴(3,4번) 동일한 제어 방식
- **안전 기능**: 토크 비활성화 전 속도 0 설정으로 안전한 정지

#### 테스트 결과
- **4바퀴 동시 구동**: 모든 바퀴가 동일한 속도로 회전
- **제자리 회전**: 왼쪽(1,3번)과 오른쪽(2,4번) 바퀴 반대 방향 회전
- **즉시 정지**: 브레이크 명령 시 모든 바퀴 동시 정지
- **연결 상태 모니터링**: 각 바퀴별 연결 상태 실시간 확인

### 설치 방법

#### 1. 필요한 파일 복사
다음 파일들을 Arduino 라이브러리 폴더에 복사하세요:

```
src/turtlebot3/turtlebot3_motor_driver.cpp
include/turtlebot3/turtlebot3_motor_driver.h
src/turtlebot3/turtlebot3.cpp
include/turtlebot3/turtlebot3.h
```

#### 2. 복사 위치
- **Linux**: `/home/[사용자명]/.arduino15/packages/OpenCR/hardware/OpenCR/1.5.3/libraries/turtlebot3_ros2/`
- **Windows**: `C:\Users\[사용자명]\AppData\Local\Arduino15\packages\OpenCR\hardware\OpenCR\1.5.3\libraries\turtlebot3_ros2\`
- **macOS**: `~/Library/Arduino15/packages/OpenCR/hardware/OpenCR/1.5.3/libraries/turtlebot3_ros2/`

#### 3. 실행 방법
1. Arduino IDE에서 `examples/turtlebot3_burger/turtlebot3_burger.ino` 열기
2. OpenCR 보드 선택
3. 컴파일 및 업로드
4. 시리얼 모니터에서 모터 연결 상태 확인

### 하드웨어 요구사항
- OpenCR 보드
- Dynamixel XM430-W210-T 모터 4개 (ID: 1, 2, 3, 4)
- 4바퀴 TurtleBot3 Burger 모델

### 주의사항
- 이 버전은 Burger 모델 전용입니다 (Waffle 모델 미지원)
- 4바퀴 모두 동일한 제어 방식으로 동작합니다
- BUS_WATCHDOG가 활성화되어 있어 통신 문제 시 자동 정지됩니다
