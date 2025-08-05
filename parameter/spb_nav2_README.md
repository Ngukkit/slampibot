# `parameter/spb_nav2_params.yaml` 튜닝 가이드

이 문서는 `parameter/spb_nav2_params.yaml` 파일의 파라미터들을 로봇의 성능을 최적화하고 실제 환경에서 안정적으로 작동하도록 튜닝하는 방법을 안내합니다.

## 1. Nav2 파라미터 튜닝의 기반

Nav2 파라미터 튜닝은 주로 다음 세 가지를 기반으로 합니다.

1.  **로봇의 물리적 특성 (가장 중요)**:
    *   **크기**: 로봇의 실제 폭, 길이, 높이. (URDF에서 가져옴)
    *   **바퀴**: 바퀴 반지름, 바퀴 간 거리. (URDF 및 `parameter/edu_drive_edu_bot.yaml`에서 가져옴)
    *   **최대 속도/가속도**: 로봇이 낼 수 있는 최대 선형 속도, 최대 각속도, 최대 선형 가속도, 최대 각 가속도. (모터 사양, 기어비, 배터리 출력 등을 고려)
    *   **회전 반경**: 로봇이 회전할 수 있는 최소 반경. (차동 구동 로봇의 경우 제자리 회전 가능)

2.  **센서의 특성**:
    *   **라이다**: 최소/최대 감지 거리, 스캔 각도, 노이즈 특성.
    *   **IMU**: 노이즈 특성, 드리프트 특성.

3.  **환경의 특성**:
    *   **지형**: 평평한지, 경사가 있는지, 험한지.
    *   **장애물**: 장애물의 크기, 밀도, 움직이는지 여부.
    *   **공간**: 좁은 통로, 넓은 공간 등.

4.  **원하는 로봇의 동작 특성**:
    *   **공격적인 움직임 vs. 보수적인 움직임**: 장애물에 얼마나 가깝게 접근할 것인지, 얼마나 빠르게 움직일 것인지.
    *   **부드러운 움직임 vs. 빠른 움직임**: 경로를 얼마나 부드럽게 따라갈 것인지, 얼마나 빠르게 목표에 도달할 것인지.

## 2. `parameter/spb_nav2_params.yaml` 튜닝 가이드

`parameter/spb_nav2_params.yaml` 파일에는 Nav2의 다양한 노드에 대한 파라미터가 포함되어 있습니다. 주요 튜닝 대상은 다음과 같습니다.

### 2.1. `controller_server` (로컬 플래너 - DWBLocalPlanner)

로봇이 경로를 따라가고 장애물을 회피하는 방식을 결정합니다.

*   **`max_vel_x`, `max_vel_theta`**: 로봇의 최대 선형/각속도. **로봇의 실제 물리적 한계에 맞춰야 합니다.**
*   **`acc_lim_x`, `acc_lim_theta`**: 로봇의 최대 선형/각 가속도. **로봇의 실제 물리적 한계에 맞춰야 합니다.**
*   **`wheel_separation`, `wheel_diameter`**: `diff_drive_controller`가 사용하는 값과 일치해야 합니다. (현재 `0.167`, `0.066`으로 설정되어 있음)
*   **`min_vel_x`, `min_vel_theta`**: 로봇이 움직일 수 있는 최소 속도.
*   **`sim_time`**: 시뮬레이션 시간. `false`로 설정되어 있는지 확인합니다.
*   **`critics`**: DWB 플래너의 평가 함수들입니다. 각 critic의 `scale` 값을 조절하여 로봇의 동작 특성을 변경합니다.
    *   `PathAlign`, `GoalAlign`: 경로 및 목표에 얼마나 잘 정렬할 것인지.
    *   `PathDist`, `GoalDist`: 경로 및 목표로부터 얼마나 떨어져 있을 것인지.
    *   `BaseObstacle`: 장애물로부터 얼마나 떨어져 있을 것인지. (이 값이 너무 작으면 충돌, 너무 크면 회피를 너무 일찍 시작)
    *   `RotateToGoal`: 목표 지점에서 얼마나 정확하게 회전할 것인지.

### 2.2. `planner_server` (글로벌 플래너)

로봇이 지도 상에서 목표 지점까지의 전체 경로를 계획하는 방식을 결정합니다.

*   **`tolerance`**: 목표 지점으로부터 얼마나 가까이 도달해야 목표에 도착했다고 판단할지.
*   **`allow_unknown`**: 미지의 공간으로 경로를 계획할지 여부.
*   **`use_astar`**: A* 알고리즘을 사용할지 여부.

### 2.3. `local_costmap` / `global_costmap` (코스트맵)

로봇 주변 환경의 장애물 정보를 나타내는 지도입니다.

*   **`resolution`**: 코스트맵의 해상도.
*   **`robot_radius`**: 로봇의 반지름. **로봇의 실제 크기보다 약간 크게 설정하여 안전 마진을 줍니다.**
*   **`inflation_radius`**: 장애물 주변에 생성되는 팽창 영역의 반지름. 이 값이 클수록 로봇이 장애물로부터 더 멀리 떨어져서 이동합니다.
*   **`plugins`**: 코스트맵에 사용할 플러그인들 (예: `static_layer`, `obstacle_layer`, `inflation_layer`).
*   **`scan` (obstacle_layer 아래)**: 라이다 센서의 설정.
    *   `topic`: 라이다 토픽 이름 (예: `/scan`).
    *   `max_obstacle_height`: 장애물로 인식할 최대 높이.
    *   `raytrace_max_range`, `obstacle_max_range`: 라이다가 장애물을 감지할 최대 거리.

### 2.4. `amcl` (Adaptive Monte Carlo Localization)

로봇의 현재 위치를 추정하는 데 사용됩니다.

*   **`use_sim_time`**: `false`로 설정되어 있는지 확인합니다.
*   **`initial_pose_x`, `initial_pose_y`, `initial_pose_a`**: 로봇의 초기 위치 및 방향.
*   **`min_particles`, `max_particles`**: AMCL이 사용하는 파티클 필터의 파티클 수.
*   **`update_min_a`, `update_min_d`**: 로봇이 얼마나 움직여야 AMCL이 위치 추정을 업데이트할지.
*   **`odom_frame_id`, `base_frame_id`, `global_frame_id`**: TF 프레임 이름.

## 3. 튜닝 과정 (일반적인 접근)

1.  **기본 설정 확인**: `use_sim_time`이 `false`인지, `robot_radius`, `wheel_separation`, `wheel_diameter` 등 로봇의 물리적 특성이 정확한지 확인합니다.
2.  **속도/가속도 제한 설정**: 로봇의 모터 사양과 안전을 고려하여 `max_vel_x`, `max_vel_theta`, `acc_lim_x`, `acc_lim_theta` 등을 설정합니다.
3.  **코스트맵 튜닝**: `robot_radius`, `inflation_radius`, `scan` 설정 등을 조절하여 로봇이 장애물을 얼마나 잘 피하고, 얼마나 가까이 접근할 수 있는지 테스트합니다.
4.  **플래너 튜닝**: `critics`의 `scale` 값들을 조절하여 로봇의 경로 추종 및 장애물 회피 동작을 최적화합니다.
5.  **AMCL 튜닝**: 로봇의 위치 추정 정확도와 안정성을 위해 파티클 수, 업데이트 임계값 등을 조절합니다.

**튜닝은 반복적인 과정입니다.** 파라미터를 변경할 때마다 로봇을 실제 환경에서(또는 시뮬레이션에서) 실행하여 동작을 관찰하고, 로그 메시지를 확인하며, RViz에서 로봇의 경로, 코스트맵, 센서 데이터 등을 시각적으로 확인해야 합니다.

Nav2 공식 문서의 튜닝 가이드를 참고하는 것이 매우 도움이 될 것입니다.
