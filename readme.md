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
*   전반적인 시스템의 안정성과 정확성이 향상되었습니다.

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