# LiDAR 지도와 천장 카메라 연동 운영 가이드

이 문서는 LiDAR를 이용해 제작한 지도와, 천장 카메라를 이용한 동적 장애물(테이블) 인식을 연동하는 절차를 설명합니다. 이 방식은 LiDAR 지도를 기준으로, 카메라가 검출한 장애물의 위치를 정확하게 일치시키는 것을 목표로 합니다.

---

## 1단계: LiDAR로 기준 지도 제작 (로봇 완성 후)

가장 먼저, 로봇의 LiDAR를 이용해 전체 공간에 대한 기준 지도를 만듭니다.

1.  천장 카메라 시스템은 잠시 꺼둡니다.
2.  터틀봇을 조종하여 공간 전체를 돌아다니며, `slam_toolbox` 같은 SLAM 노드를 실행하여 지도를 작성합니다.
3.  지도가 완성되면, `map_saver_cli`를 이용해 `my_map.pgm`(이미지)과 `my_map.yaml`(설정) 파일로 저장합니다.

    ```bash
    # SLAM으로 지도를 만든 후, 원하는 위치에 저장
    ros2 run nav2_map_server map_saver_cli -f ~/my_map
    ```

---

## 2단계: 기준 지도에서 랜드마크 태그의 좌표 획득

이제 저장된 지도 안에서, 우리가 현실에 붙여놓은 랜드마크 태그(100, 200, 300번)가 어디에 위치하는지 정확한 좌표를 알아내야 합니다.

1.  `map_server`로 방금 저장한 지도를 RViz에 띄웁니다.

    ```bash
    # 저장된 지도를 불러오는 예시
    ros2 launch nav2_bringup bringup_launch.py map:=~/my_map.yaml
    ```

2.  RViz 상단 툴바에서 **"Publish Point"** 툴을 선택합니다.
3.  RViz에 표시된 지도 위에서, **랜드마크 태그 100번이 실제로 있는 위치**를 마우스로 클릭합니다.
4.  새 터미널을 열고, 아래 명령어로 `/clicked_point` 토픽을 확인하면 방금 클릭한 위치의 x, y 좌표가 나타납니다. 이 좌표를 기록해 둡니다.

    ```bash
    ros2 topic echo /clicked_point
    ```
    *(출력 예시)*
    ```yaml
    header:
      stamp:
        sec: 1755655000
        nanosec: 123456789
      frame_id: map
    point:
      x: 1.25
      y: -0.50
      z: 0.0
    ```

5.  랜드마크 태그 200번, 300번에 대해서도 위 과정을 반복하여 지도상의 정확한 x, y 좌표를 모두 얻습니다.

---

## 3단계: 천장 카메라 설정 파일 업데이트

2단계에서 얻은 좌표를 천장 카메라 시스템에 알려주어 두 시스템의 기준을 일치시킵니다.

1.  `src/map_table_publisher/config/ceiling_cam_params.yaml` 파일을 엽니다.
2.  `landmark_tags` 섹션의 x, y 값을 2단계에서 얻은 **LiDAR 지도의 좌표값**으로 각각 수정합니다.

    *(수정 예시)*
    ```yaml
    landmark_tags:
      "100":
        x: 1.25   # 2단계에서 얻은 x 좌표
        y: -0.50  # 2단계에서 얻은 y 좌표
        z: 0.0
      "200":
        x: 4.80
        y: -0.55
        z: 0.0
      "300":
        x: 2.50
        y: 3.10
        z: 0.0
    ```

---

## 4단계: 최종 시스템 실행

이제 모든 준비가 끝났습니다. 최종적으로 Nav2와 안정화된 천장 카메라 시스템을 함께 실행합니다.

1.  SLAM 노드 대신, `map_server`와 `amcl` (위치 추정) 노드를 포함한 Nav2 스택을 실행합니다. (예: `ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=~/my_map.yaml`)

2.  안정성이 개선된 천장 카메라 시스템을 동시에 실행합니다. 이 런치 파일은 `apriltag_persistence_node`를 포함하여, 정지 상태에서도 AprilTag 감지가 끊기지 않도록 보장해줍니다.
    ```bash
    ros2 launch map_table_publisher calibrated_persistent.launch.py
    ```

이제 `amcl`이 LiDAR 지도를 기준으로 로봇의 위치를 추정하고, 천장 카메라는 `ceiling_cam_params.yaml`에 업데이트된 좌표를 기준으로 테이블의 위치를 계산합니다. 두 시스템이 동일한 `map` 좌표계를 공유하기 때문에, **카메라가 발견한 테이블 장애물이 Nav2의 LiDAR 지도 위에 정확한 위치에 표시**되게 됩니다.