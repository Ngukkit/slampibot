# 테이블 장애물 레이어 통합 가이드

## 개요
이 문서는 AprilTag를 사용하여 감지된 테이블을 ROS 2 Navigation Stack의 costmap에 동적 장애물로 통합하는 방법을 설명합니다.

## 구성 요소
1. AprilTag 감지 노드
2. 캘리브레이션된 카메라 프로세서
3. 테이블 장애물 레이어

## 설정 방법

### 1. Launch 파일 실행
```bash
ros2 launch map_table_publisher calibrated_persistent.launch.py
```

### 2. Navigation Stack 설정
Navigation Stack의 costmap 설정에 테이블 장애물 레이어를 추가합니다.

### 3. costmap 설정
`global_costmap_params.yaml` 및 `local_costmap_params.yaml` 파일에 다음과 같이 테이블 레이어를 추가합니다:

```yaml
plugins:
  - {name: static_layer, type: "costmap_2d::StaticLayer"}
  - {name: table_layer, type: "polygon_layer::PolygonLayer"}
  - {name: inflation_layer, type: "costmap_2d::InflationLayer"}

table_layer:
  enabled: true
  polygon_topic: "/table_obstacle"
  footprint_clearing_enabled: true
```

## 작동 원리
1. 천장 카메라에서 AprilTag 감지
2. 감지된 태그 위치를 지도 좌표계로 변환
3. 두 개의 태그를 사용하여 테이블의 사각형 장애물 생성
4. 생성된 다각형을 `/table_obstacle` 토픽으로 발행
5. costmap의 테이블 레이어가 이 다각형을 받아 장애물로 추가

## 주의사항
- AprilTag가 정확하게 감지되어야 정확한 테이블 위치가 계산됩니다.
- 카메라 캘리브레이션이 정확해야 정확한 좌표 변환이 가능합니다.
- 테이블 레이어는 ROS 2 Navigation Stack의 polygon_layer 패키지에 의존합니다.