# Orange v0.22 Release Notes

이 문서는 천장 카메라 기반의 AprilTag 감지 및 랜드마크 위치 추정 시스템의 안정성과 정확성을 크게 향상시킨 `Orange v0.22` 버전의 주요 변경 사항 및 버그 수정 내역을 기록합니다. 이전 버전에서 발생했던 여러 문제를 해결하여, `map` 좌표계에서 랜드마크와 테이블 장애물이 정확하게 표시되도록 개선했습니다.

### 주요 개선 사항 및 버그 수정

*   **랜드마크 마커 발행 문제 해결**:
    *   **문제**: `calibrated_camera_processor.py` 노드가 계산된 랜드마크 마커를 `/landmark_markers` 토픽에 발행하도록 설정되어 있었으나, 실제로는 해당 함수가 호출되지 않아 RViz에 마커가 표시되지 않았습니다.
    *   **해결**: `calibrated_camera_processor.py`의 `update_table_obstacles` 함수 내에 `self.publish_calculated_landmark_markers` 호출을 추가하여 계산된 랜드마크 마커가 정상적으로 발행되도록 수정했습니다.

*   **AprilTag TF 발행 문제 해결**:
    *   **문제**: `apriltag_ros` 노드가 AprilTag를 성공적으로 감지했음에도 불구하고, 해당 태그들의 TF(Transform)가 `map` 좌표계에 발행되지 않아 TF 트리가 끊어지는 문제가 발생했습니다.
    *   **해결**: `map_table_publisher/config/landmark_tags.yaml` 파일에서 `apriltag_ceiling_camera` 노드의 `publish_tf` 파라미터 값을 `false`에서 `true`로 변경하여 AprilTag TF가 정상적으로 발행되도록 수정했습니다.

*   **`do_transform_pose` 관련 `AttributeError` 해결**:
    *   **문제**: `calibrated_camera_processor.py`에서 `tf2_geometry_msgs.do_transform_pose` 함수 사용 시 `AttributeError: 'PoseStamped' object has no attribute 'position'` 오류가 반복적으로 발생했습니다. 이는 `tf2_geometry_msgs` 라이브러리 버전 또는 환경 문제로 추정되었습니다.
    *   **해결**: `do_transform_pose` 함수 대신 `tf_transformations` 라이브러리를 사용하여 포즈 변환을 수동으로 구현하는 방식으로 변경했습니다. 이로써 `PoseStamped` 객체 타입 불일치로 인한 오류를 우회하고 안정적인 변환을 확보했습니다.

*   **`SyntaxError: unterminated string literal` 해결**:
    *   **문제**: `calibrated_camera_processor.py` 파일의 `marker.type = Marker.SPHERE"` 라인에 불필요한 큰따옴표(`"`)가 포함되어 빌드 시 구문 오류가 발생했습니다.
    *   **해결**: 해당 라인에서 `Marker.SPHERE` 뒤의 큰따옴표를 제거하여 `marker.type = Marker.SPHERE`로 수정했습니다.

### 현재 상태

*   `calibrated_camera_processor` 노드에서 계산된 랜드마크 마커(녹색 구)가 `/landmark_markers` 토픽에 정상적으로 발행됩니다.
*   `landmark_visualizer` 노드에서 발행하는 이상적인 랜드마크 마커(주황색 큐브)와 함께 RViz에서 두 종류의 마커가 모두 올바르게 시각화됩니다.
*   AprilTag TF가 `map` 좌표계에 정상적으로 연결되어 TF 트리가 올바르게 구성됩니다.
*   전반적인 시스템의 안정성과 정확성이 향상되었습니다。

### 시스템 실행 방법

1.  **작업 공간 빌드**: 변경된 코드를 적용합니다.
    ```bash
colcon build
    ```
2.  **런치 파일 실행**: 랜드마크 시각화 및 처리를 시작합니다.
    ```bash
ros2 launch map_table_publisher calibrated_persistent.launch.py
    ```
3.  **RViz 확인**: RViz에서 `/landmark_markers` 토픽을 구독하고 TF 트리를 확인하여 모든 마커와 TF 링크가 정상적으로 표시되는지 검증합니다.

# Orange v0.23 Release Notes

이 문서는 `Orange v0.23` 버전에서 개선된 AprilTag 감지 및 위치 추정 시스템의 정확성과 신뢰성을 더욱 향상시킨 주요 변경 사항을 기록합니다. 카메라의 물리적 방향을 고려한 좌표계 변환과 정밀한 3D 포즈 계산을 통해 시스템의 전반적인 성능을 개선했습니다.

### 주요 개선 사항

*   **카메라 좌표계와 월드 좌표계의 올바른 변환**:
    *   **문제**: 천장에 설치된 카메라의 물리적 방향(아래를 내려다보는)을 고려하지 않아 Y축이 뒤집히는 문제가 발생했습니다.
    *   **해결**: 카메라 좌표계와 월드 좌표계 사이의 올바른 변환을 정의하여, 카메라가 감지한 태그 위치가 월드 좌표계에서 정확하게 표현되도록 수정했습니다.

*   **정확한 3D 포즈 계산**:
    *   **문제**: 이전 버전에서는 단순화된 근사치를 사용하여 3D 포즈를 계산했기 때문에 정확도가 떨어졌습니다.
    *   **해결**: `cv2.solvePnP` 함수를 사용하여 실제 AprilTag 크기와 카메라 내재 파라미터를 기반으로 정확한 3D 포즈를 계산하도록 개선했습니다.

*   **실제 카메라 왜곡 계수 적용**:
    *   **문제**: 카메라 왜 distortion을 무시하고 계산하여 정밀도가 떨어졌습니다.
    *   **해결**: 카메라 교정 파일에서 실제 왜곡 계수를 읽어와 `cv2.solvePnP` 함수에 적용하여 더욱 정확한 포즈 계산을实现했습니다.

*   **성능 최적화**:
    *   **문제**: 모든 태그에 대해 3D 포즈 계산을 수행하여 불필요한 계산이 발생했습니다.
    *   **해결**: 랜드마크 태그, 로봇 태그, 테이블 태그에 대해서만 3D 포즈 계산을 수행하도록 최적화하여 시스템 성능을 향상시켰습니다.

*   **향상된 디버깅 기능**:
    *   **문제**: YAML 파일에 정의된 기준 위치와 실제 감지된 태그 위치 사이의 차이를 확인하기 어려웠습니다.
    *   **해결**: YAML 기준 위치와 카메라 감지 위치를 비교하여 차이를 보여주는 디버그 기능을 추가하여, 사용자가 시스템을 조정하는 데 도움을 주도록 개선했습니다.

### 현재 상태

*   카메라가 천장에 설치되어 아래를 내려다보는 물리적 방향을 올바르게 고려하여 좌표계 변환을 수행합니다.
*   `cv2.solvePnP`를 사용하여 실제 AprilTag 크기와 카메라 왜 distortion 계수를 기반으로 정확한 3D 포즈를 계산합니다.
*   랜드마크 태그, 로봇 태그, 테이블 태그에 대해서만 필요한 경우에 3D 포즈 계산을 수행하여 성능을 최적화합니다.
*   디버그 모드에서 YAML 기준 위치와 실제 감지 위치 사이의 차이를 확인할 수 있어 시스템 조정이 용이합니다.
*   RViz에서 `/landmark_markers` 토픽을 통해 계산된 태그 위치를 시각적으로 확인할 수 있습니다.

### 시스템 실행 방법

1.  **작업 공간 빌드**: 변경된 코드를 적용합니다.
    ```bash
colcon build
    ```
2.  **런치 파일 실행**: 랜드마크 시각화 및 처리를 시작합니다.
    ```bash
ros2 launch map_table_publisher calibrated_persistent.launch.py
    ```
3.  **디버그 정보 확인**: 디버그 레벨 로그를 통해 YAML 기준 위치와 실제 감지 위치 사이의 차이를 확인합니다.
4.  **RViz 확인**: RViz에서 `/landmark_markers` 토픽을 구독하여 계산된 태그 위치를 시각적으로 검증합니다.

# Orange v0.24 Release Notes

이 문서는 AprilTag 기반 시스템의 안정성, 정확성 및 Nav2 통합 기능을 향상시킨 `Orange v0.24` 버전의 주요 변경 사항을 기록합니다.

### 주요 개선 사항 및 기능 추가

*   **랜드마크 및 테이블 위치 일관성 강화**:
    *   `calibrated_camera_processor.py`에서 계산된 랜드마크 위치(`Verification` 로그)와 `/landmark_markers` 토픽에 게시되는 위치 간의 불일치를 해결했습니다. 이제 두 위치가 동일한 호모그래피 변환을 사용하여 정확히 일치합니다.
    *   변환된 포즈에서 `NaN`(Not a Number) 또는 `Inf`(Infinity) 값이 발생할 경우를 처리하여 노드 충돌을 방지하고 시스템 견고성을 높였습니다.
    *   NumPy float 값을 표준 Python float으로 명시적으로 변환하여 `geometry_msgs` 메시지 필드 할당 시 발생하던 `PyFloat_Check` 어설션 오류를 해결했습니다.
    *   테이블 장애물 폴리곤(`PolygonStamped`)도 랜드마크와 동일한 호모그래피 변환을 사용하여 정확한 위치에 표시되도록 `publish_table_obstacle` 함수를 수정했습니다.

*   **동적 테이블 내비게이션 기능 추가**:
    *   `calibrated_camera_processor.py`가 각 테이블의 중심 위치를 `/table_centers/table_1`부터 `/table_centers/table_4` 토픽으로 `PoseStamped` 메시지로 게시하도록 기능을 추가했습니다.
    *   `robot_commander/waypoint_commander_node.py`가 이 테이블 중심 토픽들을 구독하도록 수정했습니다.
    *   `waypoint_commander_node`의 `send_waypoint` 서비스가 이제 테이블 ID(1~4)를 직접 받아 처리할 수 있도록 개선했습니다.
    *   로봇이 테이블 안으로 들어가지 않고 근처에 접근하도록 목표 위치에 오프셋을 자동으로 적용하는 로직을 추가했습니다.
    *   테이블 ID를 101-104에서 1-4로 변경하여 사용 편의성을 높였습니다.

*   **Nav2 코스트맵 통합**:
    *   테이블 폴리곤이 Nav2 코스트맵에 장애물로 인식되도록 `nav2_params.yaml` 파일에 `table_polygons` 관측 소스를 추가하는 설정을 적용했습니다.

### 시스템 실행 방법

`Orange v0.24` 시스템을 실행하고 테스트하는 방법은 다음과 같습니다.

1.  **작업 공간 빌드**: 변경된 코드를 적용합니다.
    ```bash
    colcon build
    ```

2.  **Nav2 파라미터 파일 설정**:
    *   수정된 `nav2_params.yaml` 파일이 Nav2 스택에 의해 로드되는지 확인합니다. (기본 경로에 복사했거나, 런치 파일에서 해당 경로를 지정해야 합니다.)
    *   `nav2_params.yaml` 파일은 `/home/dong/turtlebot3_ws/src/nav2_params.yaml`에 있습니다. 이 파일을 `/opt/ros/jazzy/share/nav2_bringup/params/` 경로에 복사해 넣었다면 별도의 설정 없이 Nav2가 로드할 것입니다.

3.  **모든 ROS2 노드 실행**:
    *   **Nav2 스택 실행**:
        ```bash
        ros2 launch nav2_bringup navigation_launch.py use_sim_time:=<true_or_false> # 시뮬레이션 환경에 따라 설정
        ```
        (만약 `navigation_launch.py`를 수정하여 `params_file` 경로를 변경했다면, 해당 런치 파일을 실행하세요.)
    *   **AprilTag 및 테이블 퍼블리셔 실행**:
        ```bash
        ros2 launch map_table_publisher calibrated_persistent.launch.py
        ```
    *   **로봇 커맨더 실행**:
        ```bash
        ros2 run robot_commander waypoint_commander_node
        ```
    *   **웹 인터페이스 실행**:
        ```bash
        ros2 run robot_web_interface app
        ```

4.  **RViz 확인**:
    *   RViz에서 `/landmark_markers` 토픽을 구독하고 TF 트리를 확인하여 모든 마커와 TF 링크가 정상적으로 표시되는지 검증합니다.
    *   Nav2 코스트맵에서 테이블 폴리곤이 장애물로 표시되는지 확인합니다.

5.  **테이블 내비게이션 테스트**:
    *   **웹 인터페이스 사용**: 웹 브라우저에서 `http://<로봇_IP_주소>:5000` (또는 `http://localhost:5000`)에 접속하여 테이블 ID(1~4) 버튼을 클릭하여 로봇을 해당 테이블로 내비게이션합니다.
    *   **ROS2 서비스 호출**: 터미널에서 직접 서비스를 호출하여 내비게이션을 테스트할 수 있습니다.
        ```bash
        ros2 service call /send_waypoint robot_commander_interfaces/srv/SendWaypoint "{waypoint_id: 1}"
        ```
        (`waypoint_id`를 `1`, `2`, `3`, `4`로 변경하여 각 테이블로 이동을 테스트합니다.)

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
