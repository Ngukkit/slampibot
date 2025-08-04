# `edu_drive_edu_bot.yaml` 파일 설명

이 YAML 파일은 `edu_drive_node` (현재 `real_driver_node`)에 전달되는 ROS2 파라미터들을 정의합니다. 이 파라미터들은 로봇의 하드웨어 설정, 통신 방식, 모터 제어 특성, 그리고 가장 중요한 로봇의 기구학 모델을 구성합니다.

## 파일 구조 및 파라미터 설명

```yaml
/**:
  ros__parameters:
```
*   `/**: ros__parameters:`: 이 부분은 ROS2의 파라미터 로딩 규칙을 따릅니다. `/**`는 모든 노드에 적용되는 파라미터임을 의미하며, `ros__parameters`는 ROS2 파라미터 서버에 로드될 실제 파라미터들을 포함하는 키워드입니다.

---

### 시스템 파라미터 (System Parameters)

이들은 `EduDrive` 노드 전체에 적용되는 일반적인 설정들입니다.

```yaml
    usingPowerManagementBoard: true
```
*   `usingPowerManagementBoard`: `true`로 설정되어 있으므로, `EduDrive` 노드가 전력 관리 보드(`PowerManagementBoard` 클래스)와 통신하여 전압 및 전류 센싱 데이터를 읽어올 것임을 나타냅니다.

```yaml
    verbosity: false
```
*   `verbosity`: `false`로 설정되어 있으므로, 노드의 디버그 메시지 출력을 비활성화합니다. `true`로 설정하면 더 많은 정보가 콘솔에 출력됩니다.

```yaml
    canInterface: eduart-can2
```
*   `canInterface`: CAN 통신에 사용할 인터페이스의 이름입니다. 라즈베리파이에서 CAN HAT를 설정했을 때의 인터페이스 이름(예: `can0`)과 일치해야 합니다.

```yaml
    frequencyScale: 32
```
*   `frequencyScale`: 모터 컨트롤러의 PWM(펄스 폭 변조) 주파수 스케일링 파라미터입니다. 모터의 제어 주파수를 조절합니다.

```yaml
    inputWeight: 0.2
```
*   `inputWeight`: 모터 제어 입력에 대한 필터링 가중치입니다. 값이 0에 가까울수록 입력 변화에 더 민감하게 반응하고, 1에 가까울수록 더 부드럽게 반응합니다.

```yaml
    maxPulseWidth: 100
```
*   `maxPulseWidth`: 모터에 인가할 수 있는 최대 PWM 펄스 폭을 제한합니다. 모터의 최대 속도나 토크를 제한하는 데 사용될 수 있습니다.

```yaml
    timeout: 300
```
*   `timeout`: 모터 컨트롤러가 명령을 받지 못했을 때 모터를 비활성화하는 시간(밀리초)입니다. 통신이 끊겼을 때 로봇이 폭주하는 것을 방지합니다.

```yaml
    kp: 0.5
    ki: 7.0
    kd: 0.0
```
*   `kp`, `ki`, `kd`: 모터 컨트롤러 내부의 PID 제어기(Proportional-Integral-Derivative controller) 게인 값입니다. 모터의 속도 제어 성능(정확성, 안정성, 반응성)을 튜닝하는 데 사용됩니다.

```yaml
    antiWindup: 1
```
*   `antiWindup`: PID 제어기의 안티-와인드업(Anti-windup) 기능 활성화 여부입니다. 제어기가 포화 상태에 있을 때 적분 항이 과도하게 커지는 것을 방지하여 제어 성능을 향상시킵니다.

```yaml
    responseMode: 0
```
*   `responseMode`: 모터 컨트롤러가 어떤 종류의 응답을 보낼지 설정합니다.
    *   `0`: RPM (분당 회전수) 응답
    *   `1`: POS (위치, 엔코더 틱) 응답

---

### 컨트롤러 파라미터 (Controller Parameters)

이 섹션은 로봇에 연결된 개별 모터 컨트롤러에 대한 설정을 정의합니다.

```yaml
    controllers: 2
```
*   `controllers`: 로봇에 연결된 모터 컨트롤러의 총 개수입니다. 이 예시에서는 2개의 컨트롤러가 사용됩니다. (각 컨트롤러가 2개의 모터를 제어하므로 총 4개의 모터가 됩니다.)

```yaml
    controller0:
      canID: 0
      drive0: #front right
        # ... (모터 파라미터) ...
      drive1: #rear right
        # ... (모터 파라미터) ...

    controller1:
      canID: 1
      drive0: #front left
        # ... (모터 파라미터) ...
      drive1: #rear left
        # ... (모터 파라미터) ...
```
*   `controller0`, `controller1`: 각 모터 컨트롤러의 고유한 설정 블록입니다.
*   `canID`: 각 모터 컨트롤러의 CAN 버스 ID입니다. 이 ID는 실제 하드웨어에 설정된 CAN ID와 일치해야 합니다.
*   `drive0`, `drive1`: 각 컨트롤러에 연결된 개별 모터(드라이브)에 대한 설정입니다. 이 예시에서는 `controller0`이 "front right"와 "rear right" 모터를, `controller1`이 "front left"와 "rear left" 모터를 제어하는 것으로 주석에 명시되어 있습니다.

---

### 모터 파라미터 (Motor Parameters)

각 `drive` 블록 내에는 해당 모터에 대한 상세 설정이 포함됩니다.

```yaml
        channel: 0
```
*   `channel`: 모터 컨트롤러 내에서 해당 모터가 연결된 채널 번호입니다.

```yaml
        kinematics: [20.0, 20.0, 6.1]
```
*   `kinematics`: **이것이 4륜 독립 구동 로봇의 움직임을 정의하는 핵심 파라미터입니다.**
    *   이 벡터는 `[kx, ky, kw]`의 형태를 가집니다.
    *   `kx`: 로봇의 전진 속도(`linear.x`)가 이 모터의 회전 속도에 미치는 영향.
    *   `ky`: 로봇의 횡이동 속도(`linear.y`)가 이 모터의 회전 속도에 미치는 영향.
    *   `kw`: 로봇의 회전 속도(`angular.z`)가 이 모터의 회전 속도에 미치는 영향.
    *   `EduDrive` 노드는 이 `kinematics` 벡터들을 모아서 로봇의 전체 기구학 행렬을 구성하고, 이를 사용하여 `/cmd_vel` 명령(로봇의 전진, 횡이동, 회전 속도)을 각 바퀴의 개별 RPM으로 변환합니다.
    *   **이 값들은 로봇의 바퀴 배치, 바퀴 종류(옴니휠/메카넘휠), 그리고 로봇의 크기에 따라 정확하게 계산되어야 합니다.** 현재 값들은 예시이며, 당신의 로봇에 맞게 튜닝해야 합니다.

```yaml
        gearRatio: 89.0
```
*   `gearRatio`: 모터의 기어비입니다. 모터의 회전수와 바퀴의 회전수 사이의 비율을 나타냅니다.

```yaml
        encoderRatio: 2048.0
```
*   `encoderRatio`: 모터 엔코더의 해상도입니다. 모터가 한 바퀴 회전할 때 엔코더가 생성하는 틱(tick) 수입니다. 오도메트리 계산에 사용됩니다.

```yaml
        rpmMax: 60.0
```
*   `rpmMax`: 해당 모터의 최대 RPM(분당 회전수)입니다. 로봇의 최대 속도를 제한하는 데 사용될 수 있습니다.

```yaml
        invertEnc: 1
```
*   `invertEnc`: 엔코더의 방향을 반전시킬지 여부입니다. `1`은 반전, `0`은 반전하지 않음을 의미합니다. 엔코더가 로봇의 움직임 방향과 반대로 값을 줄 때 사용합니다.
