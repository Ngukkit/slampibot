# slampibot_gazebo - 로봇 구동 및 시뮬레이션 패키지

## 버전: v0.11

이 패키지는 4륜 구동 로봇의 구동 및 시뮬레이션을 위한 ROS2 드라이버와 설정 파일을 포함합니다. 특히, Dynamixel XL430 모터와 OpenCR 1.0 보드를 사용하는 실제 로봇 구동에 초점을 맞춰 개발되었습니다.

## 1. 프로젝트 목표

*   **실제 로봇 구동**: Raspberry Pi 5, OpenCR 1.0 보드, Dynamixel XL430 모터, 라이다, 카메라를 사용하여 4륜 구동 로봇을 제어합니다.
*   **시뮬레이션**: Gazebo 환경에서 로봇 모델을 시뮬레이션하고 제어합니다.
*   **기구학 지원**: 4륜 독립 구동 로봇의 역기구학 및 순기구학 계산을 지원하여 다양한 움직임(전진, 횡이동, 회전)을 구현합니다.

## 2. 주요 구성 요소

*   **하드웨어**: Raspberry Pi 5, OpenCR 1.0, Dynamixel XL430 (4개), 라이다, 카메라
*   **통신**: USB-Serial 통신 (Dynamixel SDK 기반)
*   **ROS2 배포판**: Humble (예상)

## 3. 파일 구조 및 주요 변경 사항 (v0.11 기준)

### 3.1. 로봇 모델 정의 (URDF)

*   **`myCar/real_BMW_core.xacro`**: 실제 로봇의 물리적 치수(몸통, 바퀴)와 센서 위치를 반영한 핵심 URDF 파일입니다. 시뮬레이션 전용 태그는 모두 제거되었습니다.
*   **`myCar/real_BMW.urdf.xacro`**: `real_BMW_core.xacro`를 포함하는 실제 로봇의 메인 URDF 파일입니다.
*   **`myCar/real_BMW_core.urdf`**: `real_BMW_core.xacro`를 `xacro` 명령으로 처리하여 생성된 순수 URDF 파일입니다. FreeCAD와 같은 외부 툴에서 로봇 모델을 시각적으로 확인할 때 사용됩니다.

### 3.2. ROS2 Launch 파일

*   **`launch/real_robot.launch.py`**: 실제 로봇 구동을 위한 Launch 파일입니다. Gazebo 및 시뮬레이션 관련 노드를 포함하지 않으며, `robot_state_publisher`, `real_driver_node`, 라이다/카메라 드라이버를 실행합니다.
*   **`launch/spb_spawn_space.launch.py`**: Gazebo 시뮬레이션 구동을 위한 Launch 파일입니다. `ExecuteProcess`를 사용하여 Gazebo를 ROS 파라미터로부터 격리하여 실행합니다.

### 3.3. 로봇 드라이버 코드

*   **`src/real_driver_node.cpp`**: `EduDrive` 시스템을 초기화하고 실행하는 ROS2 노드의 메인 진입점입니다. 실제 로봇 구동 시 사용됩니다.
*   **`src/EduDrive.h`, `src/EduDrive.cpp`**: 로봇의 핵심 구동 로직을 포함하는 클래스입니다. `cmd_vel` 명령 처리, 오도메트리 계산, 센서 데이터 발행 등을 담당합니다. `SocketCAN` 대신 `DynamixelSerialPort`를 사용하도록 수정되었습니다.
*   **`src/MotorController.h`, `src/MotorController.cpp`**: Dynamixel XL430 모터를 직접 제어하는 클래스입니다. CAN 통신 로직이 Dynamixel SDK 기반의 USB-Serial 통신 로직으로 교체되었습니다. 모터의 토크 제어, 속도 설정, 현재 속도 읽기 등을 수행합니다.
*   **`src/dynamixel/DynamixelSerialPort.h`, `src/dynamixel/DynamixelSerialPort.cpp`**: Dynamixel SDK를 사용하여 USB-Serial 포트 통신을 처리하는 기본 클래스입니다. `SocketCAN`을 대체합니다.
*   **`src/RPiAdapterBoard.h/.cpp`, `src/RPiExtensionBoard.h/.cpp`, `src/PowerManagementBoard.h/.cpp`**: OpenCR 보드의 센서(IMU, 전압, 전류) 및 GPIO/서보 제어 인터페이스 클래스입니다. `usb_to_dxl` 펌웨어의 한계로 인해 현재는 해당 기능들이 직접 지원되지 않으며, 더미 값을 반환하도록 수정되었습니다.

### 3.4. 빌드 시스템 및 파라미터

*   **`real_CMakeLists.txt`**: 실제 로봇 구동을 위한 `real_driver_node` 및 관련 소스 파일들을 빌드하도록 설정된 CMake 파일입니다. `DynamixelSDK` 라이브러리에 링크됩니다.
*   **`CMakeLists.txt`**: Gazebo 시뮬레이션 구동을 위한 CMake 파일입니다.
*   **`parameter/edu_drive_edu_bot.yaml`**: `real_driver_node`에 전달되는 ROS2 파라미터들을 정의합니다. 로봇의 하드웨어 설정, 통신 방식, 모터 제어 특성, 그리고 가장 중요한 로봇의 기구학 모델(`kinematics`)을 포함합니다.
*   **`parameter/README.md`**: `edu_drive_edu_bot.yaml` 파일의 내용과 각 파라미터의 의미를 설명하는 한국어 문서입니다.

### 3.5. 기타 문서

*   **`README.md` (현재 디렉토리)**: Raspberry Pi 5에서 Docker를 사용하여 CAN 통신을 설정하는 방법에 대한 한국어 가이드입니다. (현재 프로젝트의 통신 방식과는 다름을 유의)
*   **`Robot_Make_Order_Readme.md`**: 실제 로봇 제작 주문 전에 준비해야 할 소프트웨어 관련 사항들을 요약한 문서입니다.

## 4. 현재 상태 및 제한 사항 (v0.11)

*   **모터 제어**: `cmd_vel` 명령을 받아 Dynamixel XL430 모터의 속도를 제어하고, 현재 RPM을 읽어와 오도메트리를 업데이트하는 기능은 구현되었습니다.
*   **OpenCR 센서/제어**: `usb_to_dxl.ino` 펌웨어의 특성상, OpenCR 보드 자체의 IMU, 전압, 전류, GPIO/서보 제어 기능은 이 통신 채널을 통해 직접 지원되지 않습니다. 관련 클래스들은 현재 더미 값을 반환합니다.
*   **기구학**: `EduDrive` 시스템은 4륜 독립 구동 로봇의 기구학 계산을 지원하지만, `parameter/edu_drive_edu_bot.yaml` 파일의 `kinematics` 파라미터는 로봇의 실제 물리적 특성에 맞게 정확히 튜닝되어야 합니다.

## 5. 다음 단계

1.  **OpenCR 1.0에 `usb_to_dxl.ino` 펌웨어 업로드**: OpenCR 보드에 이 펌웨어를 업로드합니다.
2.  **Dynamixel SDK 설치**: Raspberry Pi에 Dynamixel SDK를 설치합니다.
3.  **파라미터 파일 튜닝**: `parameter/edu_drive_edu_bot.yaml` 파일의 `kinematics` 및 모터 관련 파라미터들을 당신의 로봇에 맞게 정확히 설정합니다.
4.  **빌드 및 실행**: `real_CMakeLists.txt`를 활성화하고 `colcon build` 후 `ros2 launch slampibot_gazebo real_robot.launch.py`를 실행합니다.
5.  **OpenCR 센서 데이터 필요 시**: OpenCR 보드 자체의 센서 데이터(IMU, 전압 등)가 필요하다면, `usb_to_dxl` 펌웨어 대신 해당 데이터를 Dynamixel Protocol로 노출하는 다른 OpenCR 펌웨어(예: TurtleBot3의 OpenCR 펌웨어)를 사용하거나, 직접 펌웨어를 수정해야 합니다.
