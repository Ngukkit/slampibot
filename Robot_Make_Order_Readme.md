# 로봇 제작 주문 전 준비 사항 (소프트웨어)

이 문서는 실제 로봇(4륜 구동, Dynamixel XL430 모터, OpenCR 1.0 보드, 라즈베리파이, 라이다, 카메라)이 도착하기 전에 소프트웨어적으로 준비해야 할 핵심 사항들을 요약합니다.

## 1. 로봇 모델 정의 (URDF)

*   **파일**: `myCar/real_BMW_core.xacro` 및 이를 통해 생성된 `myCar/real_BMW_core.urdf`
*   **내용**: 로봇의 몸통, 바퀴, 센서(라이다, 카메라)의 실제 측정된 크기와 위치를 정확하게 반영합니다. 시뮬레이션 전용 태그는 모두 제거되었습니다.
*   **목적**: RViz에서 로봇 모델을 정확하게 시각화하고, `robot_state_publisher`가 로봇의 TF(변환)를 올바르게 계산하도록 합니다.
*   **상태**: 준비 완료.

## 2. 실제 로봇 구동을 위한 ROS2 Launch 파일

*   **파일**: `launch/real_robot.launch.py` (기본 구동용), `launch/real_nav.launch.py` (내비게이션용)
*   **내용**: Gazebo와 같은 시뮬레이션 관련 노드를 모두 제거하고, 실제 하드웨어 드라이버 노드(`real_driver_node`), `robot_state_publisher`, 그리고 라이다/카메라 드라이버를 실행하기 위한 구조를 갖추고 있습니다.
*   **목적**: 실제 로봇 하드웨어를 ROS2 환경에서 구동합니다.
*   **상태**: 준비 완료.

## 3. 실제 로봇 드라이버 노드 (C++ 템플릿)

*   **파일**: `src/real_driver_node.cpp`
*   **내용**: `EduDrive` 시스템을 기반으로 하는 ROS2 노드의 뼈대입니다. USB-Serial 통신을 통해 OpenCR 보드 및 Dynamixel 모터와 통신하고, 센서 데이터를 읽어와 ROS 토픽으로 발행하는 기본적인 구조를 갖추고 있습니다.
*   **목적**: 라즈베리파이와 OpenCR 보드 및 모터, 센서 간의 통신을 담당하고 로봇의 움직임을 제어합니다.
*   **상태**: **구조는 준비 완료. 핵심 로직 구현 필요.**
    *   **OpenCR 통신 방식**: `usb_to_dxl` 펌웨어를 사용하는 USB-Serial 통신 기반으로 코드가 수정되었습니다.
    *   **핵심 로직 구현**: `src/real_driver_node.cpp` 내의 `EduDrive::receiveDynamixelData()` 함수와 `RPiAdapterBoard`, `RPiExtensionBoard`, `PowerManagementBoard` 클래스들의 실제 구현을 채워 넣어야 합니다. 이들은 `usb_to_dxl` 펌웨어의 통신 프로토콜에 맞춰 OpenCR 보드로부터 센서 데이터를 읽어오고 제어 명령을 보내야 합니다.

## 4. 빌드 시스템 설정

*   **파일**: `real_CMakeLists.txt`
*   **내용**: `real_driver_node`와 그 의존성들(Dynamixel SDK 포함)을 올바르게 빌드하도록 설정되었습니다.
*   **목적**: `real_driver_node` 실행 파일을 생성합니다.
*   **상태**: 준비 완료.

## 5. CAN 통신 환경 설정 가이드

*   **파일**: `README.md` (현재 디렉토리 내)
*   **내용**: 라즈베리파이 5에서 Docker를 사용하여 CAN 통신을 설정하는 방법에 대한 한국어 가이드입니다.
*   **목적**: 라즈베리파이 호스트 OS와 Docker 컨테이너 간의 CAN 통신 환경을 구축합니다.
*   **상태**: 준비 완료. (단, 현재 로봇은 USB-Serial 통신을 사용하므로 직접적인 관련은 적음)

## 요약 및 실제 장비 도착 시 해야 할 일

실제 로봇이 도착하기 전에 소프트웨어의 **기본적인 구조와 틀은 모두 준비되었습니다.**

로봇이 도착하면, 다음의 **핵심 하드웨어 설정 및 소프트웨어 구현/튜닝 작업**에 집중해야 합니다.

### 1. 하드웨어 조립 및 연결

*   **라즈베리파이 5 초기 설정**: Raspberry Pi OS 설치 및 Docker 설치를 완료합니다.
*   **OpenCR 1.0 보드 연결**: OpenCR 1.0 보드를 라즈베리파이와 USB 케이블로 연결합니다.
*   **Dynamixel XL430 모터 연결**: 4개의 Dynamixel XL430 모터를 OpenCR 보드에 연결합니다.
*   **라이다 및 카메라 연결**: 라이다와 카메라를 라즈베리파이에 연결합니다.

### 2. OpenCR 1.0 펌웨어 업로드

*   **`usb_to_dxl.ino` 펌웨어 업로드**: OpenCR 보드에 `usb_to_dxl.ino` 펌웨어를 업로드합니다. 이 펌웨어는 OpenCR을 USB-Serial 브릿지로 작동시킵니다.

### 3. 라즈베리파이 소프트웨어 설정 및 빌드

*   **Dynamixel SDK 설치**: 라즈베리파이에 Dynamixel SDK를 설치합니다. (ROS2 패키지 형태로 설치하는 것이 가장 편리합니다: `sudo apt install ros-humble-dynamixel-sdk`)
*   **라이다 및 카메라 드라이버 설치**: 당신의 라이다 및 카메라 모델에 맞는 ROS2 드라이버를 설치합니다. (예: `sudo apt install ros-humble-rplidar-ros`)
*   **`real_CMakeLists.txt` 활성화**: `real_CMakeLists.txt`를 활성화하고 `colcon build`를 실행하여 `real_driver_node`를 빌드합니다.
    ```bash
    # 프로젝트 루트 디렉토리에서
    mv CMakeLists.txt CMakeLists.txt.sim # 시뮬레이션용 백업
    mv real_CMakeLists.txt CMakeLists.txt # 실제 로봇용 활성화
    colcon build --packages-select slampibot_gazebo
    ```

### 4. `real_driver_node.cpp` 핵심 로직 완성

*   **OpenCR 센서 데이터 읽기**: `EduDrive::receiveDynamixelData()` 함수와 `RPiAdapterBoard`, `PowerManagementBoard` 클래스 내에서 OpenCR 보드로부터 IMU, 전압, 전류 등의 센서 데이터를 읽어오는 로직을 구현해야 합니다. 이는 OpenCR 펌웨어의 Dynamixel Control Table 문서를 참조하여 해당 데이터가 저장된 Dynamixel ID와 레지스터 주소를 파악한 후, Dynamixel SDK 함수를 사용하여 구현합니다.
*   **서보 제어 구현**: `RPiExtensionBoard` 클래스의 `setServos()` 함수를 구현하여 서보 모터를 제어합니다. (만약 Dynamixel 서보라면 Dynamixel SDK 사용, 일반 서보라면 OpenCR 펌웨어의 해당 제어 방식 파악)

### 5. 파라미터 파일 튜닝

*   **`parameter/edu_drive_edu_bot.yaml` 파라미터 튜닝**: 로봇의 실제 `kinematics` 값, `canID` (이제 Dynamixel ID), `gearRatio`, `encoderRatio`, `rpmMax` 등을 당신의 로봇에 맞게 정확히 입력해야 합니다.

### 6. 로봇 구동 및 내비게이션 시작

*   **기본 구동**: `ros2 launch slampibot_gazebo real_robot.launch.py`를 실행하여 로봇이 `rqt_robot_steering`으로 제어되는지 확인합니다.
*   **내비게이션**: `parameter/spb_nav2_params.yaml`을 튜닝하고 `ros2 launch slampibot_gazebo real_nav.launch.py`를 실행하여 내비게이션 기능을 테스트합니다.

이러한 준비가 완료되면, 실제 로봇을 구동하고 센서 데이터를 받아보며 내비게이션 및 SLAM 개발을 시작할 수 있을 것입니다.