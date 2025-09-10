# Orange v0.26 Release Notes

이 문서는 AprilTag 기반 테이블 장애물 시스템과 웹 인터페이스의 안정성 및 사용성을 향상시킨 `Orange v0.26` 버전의 주요 변경 사항을 기록합니다.

### 주요 개선 사항 및 기능 추가

*   **테이블 장애물 통합 시스템 개선**:
    *   `map_table_publisher` 패키지에 테이블 장애물 병합 노드(`table_obstacle_merger`) 추가로 여러 테이블 장애물을 하나의 토픽으로 통합
    *   `polygon_to_pointcloud_node`를 통해 폴리곤 형식의 테이블 장애물을 Nav2에서 사용 가능한 포인트클라우드 형식으로 변환
    *   테이블 장애물 레이어(`table_obstacle_layer`)를 Nav2 코스트맵에 직접 통합하여 장애물 인식 정확도 향상

*   **웹 인터페이스 개선**:
    *   웹 페이지에 주방(Kitchen) 이동 버튼 추가
    *   테이블 이동 버튼 레이블 명확화 (Table 1-4)
    *   웹 인터페이스에서 테이블 장애물 실시간 시각화 기능 강화

*   **로봇 내비게이션 로직 개선**:
    *   `waypoint_commander_node.py`에서 테이블 접근 후 주방으로 자동 복귀하는 기능 구현
    *   테이블 접근 시 로봇과 테이블 사이의 안전 거리(40cm)로 조정
    *   테이블 중심 인식 정확도 향상을 위한 `table_center_publisher` 개선

*   **시스템 안정성 향상**:
    *   디버그 로그 레벨 조정으로 불필요한 로그 메시지 감소
    *   노드 간 통신 안정성을 위한 타이머 기반 처리 방식 적용
    *   테이블 감지 지속성을 위한 퍼시스턴스 노드(`apriltag_persistence_node`) 활용

### 현재 상태

*   4개 테이블(1-4)의 장애물 정보가 `/merged_table_obstacles` 토픽을 통해 통합 관리됨
*   통합된 테이블 장애물이 `/obstacle_point_cloud`를 통해 Nav2 코스트맵에 실시간 반영
*   웹 인터페이스에서 주방 및 테이블(1-4)로의 직접 이동 가능
*   테이블 접근 후 자동으로 주방 위치로 복귀하는 완전한 서빙 로봇 기능 구현
*   테이블 중심 인식과 장애물 표시의 정확도 및 안정성이 향상됨

### 시스템 실행 방법

`Orange v0.26` 시스템을 실행하고 테스트하는 방법은 다음과 같습니다.

1.  **작업 공간 빌드**: 변경된 코드를 적용합니다.
    ```bash
    colcon build
    ```

2.  **모든 ROS2 노드 실행**:
    *   **Nav2 스택 실행**:
        ```bash
        ros2 launch nav2_bringup navigation_launch.py use_sim_time:=<true_or_false> map:=<지도_파일_경로>
        ```
    *   **AprilTag 및 테이블 장애물 처리 시스템 실행**:
        ```bash
        ros2 launch map_table_publisher calibrated_persistent.launch.py
        ```
    *   **로봇 커맨더 및 테이블 중심 퍼블리셔 실행**:
        ```bash
        ros2 launch robot_commander waypoint_commander.launch.py
        ```
    *   **웹 인터페이스 실행**:
        ```bash
        ros2 run robot_web_interface web_app
        ```

3.  **RViz 및 웹 인터페이스 확인**:
    *   RViz에서 `/landmark_markers`, `/merged_table_obstacles` 토픽을 구독하여 랜드마크와 테이블 장애물이 정상적으로 표시되는지 확인
    *   웹 브라우저에서 `http://<로봇_IP_주소>:5000` 접속하여 주방 및 테이블 버튼을 통해 내비게이션 테스트

4.  **테이블 내비게이션 테스트**:
    *   웹 인터페이스에서 "Go to Kitchen", "Go to Table 1-4" 버튼을 클릭하여 로봇 내비게이션 테스트
    *   터미널에서 ROS2 서비스 호출:
        ```bash
        ros2 service call /send_waypoint robot_commander_interfaces/srv/SendWaypoint "{waypoint_id: 1}"
        ```
        (waypoint_id: 1=주방, 3=테이블1, 4=테이블2, 5=테이블3, 6=테이블4)

# Orange v0.27 Release Notes

이 문서는 AprilTag 기반 시스템의 안정성과 신뢰성을 더욱 향상시킨 `Orange v0.27` 버전의 주요 변경 사항을 기록합니다. 특히 카메라 연결 문제와 감지 안정성을 개선하여 시스템의 지속적인 작동을 보장합니다.

### 주요 개선 사항 및 기능 추가

*   **자동 카메라 모니터링 및 재시작 기능 추가**:
    *   새로운 `camera_monitor_node`를 개발하여 AprilTag 감지를 지속적으로 모니터링합니다.
    *   5초 이상 AprilTag 감지가 없을 경우 자동으로 카메라를 재시작하여 연결 문제를 해결합니다.
    *   카메라 상태를 5초 간격으로 확인하여 빠른 문제 감지 및 대응이 가능합니다.
    *   `calibrated_persistent.launch.py`에 카메라 모니터 노드를 통합하여 시스템 시작 시 함께 실행됩니다.

*   **카메라 드라이버 설정 최적화**:
    *   카메라 픽셀 형식을 `yuyv2rgb`로 변경하여 AprilTag 노드와의 호환성을 향상시켰습니다.
    *   해상도를 1280x720로 유지하면서도 안정적인 이미지 전송을 구현했습니다.
    *   카메라와 AprilTag 노드 간의 동기화 문제를 해결하여 감지 안정성을 높였습니다.

*   **시스템 안정성 향상**:
    *   카메라 모니터링을 통해 시스템이 장시간 안정적으로 작동할 수 있도록 개선되었습니다.
    *   일시적인 카메라 연결 문제나 감지 중단 상황에서도 자동 복구가 가능합니다.
    *   사용자는 시스템을 지속적으로 모니터링하지 않고도 안정적인 작동을 기대할 수 있습니다.

### 현재 상태

*   카메라 모니터 노드가 AprilTag 감지를 5초 간격으로 모니터링합니다.
*   5초 이상 감지가 없을 경우 자동으로 카메라를 재시작합니다.
*   카메라와 AprilTag 노드 간의 동기화 문제를 해결하여 안정적인 감지가 이루어집니다.
*   시스템이 장시간 안정적으로 작동할 수 있는 기반을 마련했습니다.

### 시스템 실행 방법

`Orange v0.27` 시스템을 실행하는 방법은 다음과 같습니다.

1.  **작업 공간 빌드**: 변경된 코드를 적용합니다.
    ```bash
    colcon build
    ```

2.  **모든 ROS2 노드 실행**:
    *   **Nav2 스택 실행**:
        ```bash
        ros2 launch nav2_bringup navigation_launch.py use_sim_time:=<true_or_false> map:=<지도_파일_경로>
        ```
    *   **AprilTag 및 테이블 장애물 처리 시스템 실행**:
        ```bash
        ros2 launch map_table_publisher calibrated_persistent.launch.py
        ```
        이 명령어로 실행 시 카메라 모니터 노드도 함께 실행됩니다.

    *   **로봇 커맨더 및 테이블 중심 퍼블리셔 실행**:
        ```bash
        ros2 launch robot_commander waypoint_commander.launch.py
        ```
    *   **웹 인터페이스 실행**:
        ```bash
        ros2 run robot_web_interface web_app
        ```

3.  **카메라 모니터링 확인**:
    *   ROS2 노드 목록에서 `/camera_monitor` 노드가 실행 중인지 확인합니다:
        ```bash
        ros2 node list | grep camera_monitor
        ```
    *   로그를 확인하여 카메라 모니터가 정상적으로 AprilTag 감지를 받고 있는지 확인합니다:
        ```bash
        ros2 topic echo /rosout | grep "Received.*detections"
        ```

4.  **RViz 및 웹 인터페이스 확인**:
    *   RViz에서 `/landmark_markers`, `/merged_table_obstacles` 토픽을 구독하여 랜드마크와 테이블 장애물이 정상적으로 표시되는지 확인
    *   웹 브라우저에서 `http://<로봇_IP_주소>:5000` 접속하여 주방 및 테이블 버튼을 통해 내비게이션 테스트

5.  **테이블 내비게이션 테스트**:
    *   웹 인터페이스에서 "Go to Kitchen", "Go to Table 1-4" 버튼을 클릭하여 로봇 내비게이션 테스트
    *   터미널에서 ROS2 서비스 호출:
        ```bash
        ros2 service call /send_waypoint robot_commander_interfaces/srv/SendWaypoint "{waypoint_id: 1}"
        ```
        (waypoint_id: 1=주방, 3=테이블1, 4=테이블2, 5=테이블3, 6=테이블4)

# Orange v0.28 Release Notes

이 문서는 AprilTag 기반 테이블 장애물 시스템의 안정성과 정확성을 더욱 향상시킨 `Orange v0.28` 버전의 주요 변경 사항을 기록합니다. 이번 업데이트에서는 코드 안정성과 시스템 신뢰성에 중점을 두었습니다.

### 주요 개선 사항 및 버그 수정

*   **코드 안정성 향상**:
    *   `calibrated_camera_processor.py`에서 직접 `_parameters` 접근하는 부분을 수정하여 ROS2 표준 파라미터 API를 사용하도록 변경
    *   `list_parameters` 메서드 사용 시 반환값 처리 오류를 수정하여 노드 충돌 문제 해결
    *   여러 예외 처리를 추가하여 시스템 안정성 향상

*   **테이블 위치 추정 정확도 향상**:
    *   테이블 태그 중 하나만 감지되었을 때도 위치를 추정할 수 있도록 개선
    *   누락된 테이블 태그 위치를 추정하는 알고리즘을 구현하여 시스템 견고성 강화
    *   테이블 폴리곤 생성 시 좌표계 변환 정확도 향상

*   **카메라 이미지 처리 개선**:
    *   천장 카메라의 물리적 방향을 고려한 좌표계 변환을 정확하게 처리
    *   동차변환(Homography)을 사용할 때 Y축 반전 문제를 수정
    *   이미지 전처리 노드와의 호환성 개선

*   **자동 카메라 재시작 기능 최적화**:
    *   카메라 감지 모니터링 주기를 10초에서 5초로 단축하여 빠른 문제 감지
    *   재시작 후 재시도 방지를 위한 쿨다운 타임 설정
    *   카메라 재시작 시 더 안정적인 프로세스 관리

### 현재 상태

*   ROS2 표준 파라미터 API를 사용하여 코드 안정성 향상
*   테이블 태그 중 하나만 감지되었을 때도 위치 추정이 가능하여 시스템 신뢰성 강화
*   천장 카메라의 물리적 방향을 정확히 고려한 좌표계 변환 구현
*   5초 간격으로 카메라 감지를 모니터링하고 필요시 자동 재시작
*   시스템이 다양한 상황에서도 안정적으로 작동할 수 있도록 개선됨

### 시스템 실행 방법

`Orange v0.28` 시스템을 실행하는 방법은 다음과 같습니다.

1.  **작업 공간 빌드**: 변경된 코드를 적용합니다.
    ```bash
    colcon build
    ```

2.  **모든 ROS2 노드 실행**:
    *   **Nav2 스택 실행**:
        ```bash
        ros2 launch nav2_bringup navigation_launch.py use_sim_time:=<true_or_false> map:=<지도_파일_경로>
        ```
    *   **AprilTag 및 테이블 장애물 처리 시스템 실행**:
        ```bash
        ros2 launch map_table_publisher calibrated_persistent.launch.py
        ```
        이 명령어로 실행 시 모든 개선된 기능이 함께 실행됩니다.

    *   **로봇 커맨더 및 테이블 중심 퍼블리셔 실행**:
        ```bash
        ros2 launch robot_commander waypoint_commander.launch.py
        ```
    *   **웹 인터페이스 실행**:
        ```bash
        ros2 run robot_web_interface web_app
        ```

3.  **시스템 상태 확인**:
    *   ROS2 노드 목록에서 모든 노드가 정상적으로 실행 중인지 확인:
        ```bash
        ros2 node list
        ```
    *   로그를 확인하여 시스템이 정상적으로 작동하고 있는지 확인:
        ```bash
        ros2 topic echo /rosout | grep "Received\|detections\|Camera Monitor"
        ```

4.  **RViz 및 웹 인터페이스 확인**:
    *   RViz에서 `/landmark_markers`, `/merged_table_obstacles` 토픽을 구독하여 랜드마크와 테이블 장애물이 정상적으로 표시되는지 확인
    *   웹 브라우저에서 `http://<로봇_IP_주소>:5000` 접속하여 주방 및 테이블 버튼을 통해 내비게이션 테스트

5.  **테이블 내비게이션 테스트**:
    *   웹 인터페이스에서 "Go to Kitchen", "Go to Table 1-4" 버튼을 클릭하여 로봇 내비게이션 테스트
    *   터미널에서 ROS2 서비스 호출:
        ```bash
        ros2 service call /send_waypoint robot_commander_interfaces/srv/SendWaypoint "{waypoint_id: 1}"
        ```
        (waypoint_id: 1=주방, 3=테이블1, 4=테이블2, 5=테이블3, 6=테이블4)