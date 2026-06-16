# NN Controller 시스템 설계 계획 (v2)

## Context

현재 ibvs_controller는 align + approach 단계까지 동작하지만, 물체가 일정 거리 이내로 좁혀지면 카메라 기반 감지가 불안정해져 grip까지 이어지지 못한다. 이를 해결하기 위해:

1. ibvs_controller의 align + approach 전체 시퀀스(DONE 조건 충족까지)와 그 이후 human free-drive + grip 시퀀스를 하나의 에피소드로 연속 기록
2. 기록된 데이터로 NN controller 학습

에피소드 구성 (하나의 episode 디렉토리):

```
[PHASE 1] IBVS align + approach  (visual_servo_bag_recorder가 기록)
  ↓ ibvs_done 시그널
[PHASE 2] Human free-drive 정밀 조정  (human_interaction_recorder가 기록)
  ↓ 사용자 [G] 입력
[PHASE 3] Grip 실행
  ↓ 사용자 [S]/[F] 입력
[PHASE 4] Success/Fail 레이블 기록
```

## 확정된 설계 결정 (코드베이스 검토 반영)

- **D1. NN 대체 범위는 단계적**: 1단계에서는 ibvs_done 이후의 fine-tune + grip 구간만 NN이 담당한다. align + approach는 검증된 기존 IBVS를 유지한다. 단, 데이터는 전 구간(align부터 grip까지)을 기록하여 추후 전체 파이프라인 대체로 확장 가능하게 한다.
- **D2. controller phase 토픽 발행**: ibvs_controller가 자신의 내부 Phase(ALIGN_JAC_*, RUN, AREA_JAC_*, APPROACH_WAIT, DONE, ERROR 등)를 토픽으로 발행하고, 두 recorder가 샘플에 기록한다. 학습 시 Jacobian probing 구간(perturbation wiggle)을 필터링할 수 있고, ERROR 발생 시 recorder가 에피소드를 폐기할 수 있어 데드락도 방지된다.
- **D3. human phase의 detection은 동결값 사용**: IBVS는 DONE까지 detection이 안정적이도록 설계되어 있으므로, IBVS DONE 시점의 마지막 detection 값을 human interaction phase 동안 동일하게 유지한다. human_interaction_recorder는 detection을 구독하지 않으며, 동결 처리는 학습 스크립트에서 IBVS bag의 마지막 유효 샘플을 읽어 수행한다.
- **D4. event-driven 기록**: 고정 10 Hz 타이머 대신 의미 있는 이벤트 기준으로 샘플을 기록한다.
  - visual_servo_bag_recorder: target_pose command 수신 시점에 (state, action) 쌍으로 기록
  - human_interaction_recorder: status 도착 시점에 관절 변화량이 임계값을 넘을 때 기록 (+ grip/result 이벤트는 항상 기록)
  - 이유: status 폴링이 시리얼 read 7회를 동반하여 실효 폴링 속도가 2~5 Hz로 제한되며, 고정 10 Hz 기록 시 delta_angles 라벨이 대부분 0이 되어 학습 품질이 저하됨
- **D5. 에피소드 디렉토리 통합**: 두 recorder는 동일한 `episode_id`를 공유하고 `~/rosbags/raw/episode_{id}/` 아래에 `ibvs/`, `human/` 서브 bag으로 기록한다. 결과 확정 시 에피소드 디렉토리 전체를 `success/` 또는 `fail/`로 이동한다. (기존 계획은 IBVS bag이 success/fail 분류에서 빠져 policy 학습 데이터와 연결이 끊기는 문제가 있었음)

---
## Task 1: VisualServoSample.msg 확장

파일: `src/just_pick_it/just_pick_it_interfaces/msg/VisualServoSample.msg`

추가할 필드:

```
# 에피소드 연결용 (두 recorder가 동일 값 공유)
string episode_id

# ibvs_controller 내부 phase (ibvs_phase 토픽의 마지막 수신값, 미수신 시 -1)
# 값은 ibvs_controller_node.py의 Phase enum value를 그대로 사용
int32 controller_phase

# Commanded joint angles (ibvs_controller가 발행한 target_pose에서 파싱)
float64[6] commanded_angles
float64[6] commanded_delta   # commanded_angles - previous_commanded_angles
bool has_command             # 이 샘플이 command 수신 이벤트로 생성되었는지 여부
```

이유: 현재 VisualServoSample은 robot state(joint_angles)만 기록하고 action(ibvs_controller가 보낸 명령)을 기록하지 않음. imitation learning에는 (state, action) 쌍이 필요. controller_phase가 있어야 학습 시 Jacobian probing 샘플(ALIGN_JAC_*, AREA_JAC_* 구간)을 제외할 수 있음.

---
## Task 2: HumanInteractionSample.msg 신규 생성

파일: `src/just_pick_it/just_pick_it_interfaces/msg/HumanInteractionSample.msg`

```
std_msgs/Header header
string episode_id

int32 sample_index
int32 phase
# 0=WAITING, 1=FREE_DRIVE, 2=GRIPPING, 3=RESULT

float64[6] joint_angles        # 현재 관절 각도 (get_angles)
float64[6] delta_angles        # 이전 기록 샘플 대비 delta
float64 time_since_prev_sample # event-driven이라 dt가 가변이므로 명시 기록 (초)

float64 gripper_value
bool gripper_closed

bool grip_triggered            # 사용자가 grip 명령을 내린 시점 true
bool result_recorded           # 에피소드 종료 샘플
bool grip_success              # result_recorded=true일 때만 유효

float64 time_since_ibvs_done   # ibvs DONE 후 경과 시간(초)
```

detection 필드는 두지 않는다 (D3: 학습 시 IBVS DONE 시점 값으로 동결).

---
## Task 3: ibvs_controller_node.py 수정

파일: `src/just_pick_it/just_pick_it_perception/just_pick_it_perception/ibvs_controller_node.py`

변경 내용:

1. Publisher 추가:
```python
self.ibvs_done_pub = self.create_publisher(Empty, f"{self.ns}/ibvs_done", 1)
self.ibvs_phase_pub = self.create_publisher(Int32, f"{self.ns}/ibvs_phase", 10)
```

2. `set_phase()`에서 phase 변경 시마다 `ibvs_phase` publish (Phase enum의 value를 Int32로):
```python
def set_phase(self, new_phase):
    self.phase = new_phase
    self.phase_start_time = self.get_clock().now()
    self.ibvs_phase_pub.publish(Int32(data=int(new_phase.value)))
    self.get_logger().info(f"Phase -> {self.phase.name}")
```

3. Phase.DONE 진입 시 한 번만 ibvs_done publish:
```python
elif self.phase == Phase.DONE:
    if not self._ibvs_done_published:
        self.ibvs_done_pub.publish(Empty())
        self._ibvs_done_published = True
    return
```

4. `__init__`에 `self._ibvs_done_published = False` 추가

ERROR는 별도 시그널 없이 ibvs_phase 토픽(value=99)으로 전파된다. recorder들이 이를 보고 에피소드를 폐기한다.

---
## Task 4: visual_servo_bag_recorder_node.py 수정

파일: `src/just_pick_it/just_pick_it_perception/just_pick_it_perception/visual_servo_bag_recorder_node.py`

### 변경 1: event-driven 기록으로 전환 (D4)

고정 10 Hz 타이머 기록 대신, `/{robot_name}/target_pose` command 수신 시점에 샘플을 작성한다. command_callback에서 (직전 status + 직전 detection + 이번 command)를 하나의 (state, action) 샘플로 기록.

```python
self.command_sub = self.create_subscription(
    Float64MultiArray,
    f"/{self.robot_name}/target_pose",
    self.command_callback,
    10,
)

def command_callback(self, msg):
    data = list(msg.data)
    if len(data) >= 7 and int(data[0]) == 0:  # CMD_JOINT만 처리
        commanded = [float(v) for v in data[1:7]]
        # prev와의 delta 계산 후 build_sample + 기록
```

status 폴링 타이머(request_status)는 유지하되 기록 트리거로는 쓰지 않는다.

### 변경 2: ibvs_phase 구독 (D2)

```python
self.ibvs_phase_sub = self.create_subscription(
    Int32, f"/{self.robot_name}/ibvs_phase", self.ibvs_phase_callback, 10
)
```

- 수신값을 `latest_controller_phase`로 저장하여 샘플에 기록
- value=99(ERROR) 수신 시: bag 닫고 episode 디렉토리에 `ABORTED` 마커 파일 생성 후 종료

### 변경 3: ibvs_done 구독 — 수신 시 bag을 닫아 IBVS 구간 기록 종료

```python
self.ibvs_done_sub = self.create_subscription(
    Empty, f"/{self.robot_name}/ibvs_done", self.ibvs_done_callback, 1
)

def ibvs_done_callback(self, _):
    self.get_logger().info("ibvs_done received. Closing IBVS bag.")
    self.recording = False
    self.close_bag_writer()
    if self.shutdown_on_stop:
        rclpy.shutdown()
```

### 변경 4: 배경 쓰기 스레드 신규 구현

`queue.Queue` + `threading.Thread`로 rosbag SQLite I/O를 callback에서 분리한다. (현재 코드는 동기 write이며 기존 구현이 없으므로 신규 작성)

### 변경 5: 저장 경로 (D5)

`bag_uri = {bag_base_dir}/raw/episode_{episode_id}/ibvs` (episode_id는 launch에서 주입)

---
## Task 5: human_interaction_recorder_node.py 신규 생성

파일: `src/just_pick_it/just_pick_it_perception/just_pick_it_perception/human_interaction_recorder_node.py`

역할: ibvs DONE 이후 free-drive 구간의 데이터를 기록하고 grip success/fail을 레이블링

구독:
- `/{robot_name}/ibvs_done` (Empty): 시작 트리거
- `/{robot_name}/ibvs_phase` (Int32): ERROR(99) 수신 시 에피소드 폐기 후 종료
- `/{robot_name}/status` (Float64MultiArray): 현재 각도 수신 (data[14:20]=angles, data[26]=gripper)

발행:
- `/{robot_name}/request_status` (Empty): FREE_DRIVE 진입 후 폴링 시작 (status_poll_rate_hz, default 5 Hz; 시리얼 부하 고려)
- `/{robot_name}/set_arm` (Float64MultiArray): `[0]` = release_all_servos
- `/{robot_name}/set_gripper` (Float64MultiArray): `[0, 50]` = grip close

### 기록 조건 (D4, event-driven)

status 수신 시마다 평가:
- `norm(delta_angles) >= movement_eps_deg` (parameter, default 0.5)이면 샘플 기록
- grip_triggered, result_recorded 이벤트 샘플은 변화량과 무관하게 항상 기록
- `time_since_prev_sample`을 함께 기록 (dt 가변)

### State Machine

```
WAITING_FOR_IBVS_DONE
  ibvs_done 수신 시 FREE_DRIVE 진입
    release_all_servos() 호출 (set_arm [0])
    status 폴링 시작, event-driven으로 HumanInteractionSample 기록
    터미널에서 [G] 입력 대기

  [G] 입력 시 GRIPPING 진입
    set_gripper([0, 50]) 발행
    0.5초 대기

  GRIPPING 후 WAITING_RESULT 진입
    터미널에서 [S]=성공 / [F]=실패 입력 대기

  WAITING_RESULT 후 RECORDING_DONE 진입
    result 샘플 기록 (grip_success 필드 설정)
    bag 닫기
    episode 디렉토리 전체를 success/ 또는 fail/ 폴더로 이동
```

### 터미널 UI (비동기 stdin, select 또는 별도 Thread)

```
=== Human Interaction Recorder ===
[G] Grip   [S] Success   [F] Fail   [Q] Quit
Phase: FREE_DRIVE | t=3.2s | angles: [107.5, 28.9, ...]
```

### Rosbag 저장 경로 (D5)

```
{bag_base_dir}/raw/episode_{episode_id}/
  ibvs/    (visual_servo_bag_recorder가 기록)
  human/   (human_interaction_recorder가 기록)
```

결과 확정 시 `raw/episode_{episode_id}/` 디렉토리 전체를 `{bag_base_dir}/success/` 또는 `{bag_base_dir}/fail/`로 이동. ERROR/Quit 시 `ABORTED` 마커를 남기고 raw/에 잔류 (학습에서 제외).

배경 쓰기 스레드: Task 4와 동일하게 queue.Queue + threading.Thread로 SQLite I/O를 callback에서 분리

---
## Task 6: Launch 파일 신규 생성

파일: `src/just_pick_it/just_pick_it_perception/launch/nn_data_collection.launch.py`

세 노드를 한 번에 기동:
1. ibvs_controller (기존 ibvs_controller.launch.py 파라미터 재사용)
2. visual_servo_bag_recorder (IBVS align+approach 구간 기록, ibvs_done 수신 시 bag 닫기)
3. human_interaction_recorder (ibvs_done 수신 후 free-drive + grip 구간 기록)

두 recorder는 동일한 episode_id를 공유하여 학습 시 하나의 에피소드로 연결.

파라미터:
- `bag_base_dir` (default: `~/rosbags`)
- `robot_name`, `detection_topic` 공유 인자
- `episode_id` (launch 시 타임스탬프로 자동 생성, 두 recorder에 동일 주입)
- `movement_eps_deg`, `status_poll_rate_hz` (human recorder용)

---
## Task 7: NN 학습 스크립트 (1단계: fine-tune + grip policy)

파일: `src/just_pick_it/just_pick_it_perception/scripts/train_nn_controller.py`

### 범위 (D1)

1단계 policy는 human phase(FREE_DRIVE) 데이터로만 학습한다. window seed로 IBVS 종단(DONE 직전) 샘플을 사용한다. 전 구간 데이터는 기록되어 있으므로 2단계(전체 파이프라인 대체) 확장 시 동일 데이터셋 재사용 가능.

### 전처리

- `success/` 에피소드만 사용 (policy)
- IBVS bag에서 `controller_phase`가 RUN(20), APPROACH_WAIT(40), DONE(90)이 아닌 샘플(Jacobian probing 구간) 제외
- human phase 샘플의 detection feature는 IBVS bag의 마지막 유효 detection 값으로 동결 채움 (D3)
- event-driven이므로 timestep 간격이 가변. `time_since_prev_sample`을 feature에 포함하거나 고정 dt로 리샘플링 (구현 시 결정, 우선 feature 포함 방식)

### 데이터셋 구성

입력 (sliding window, 5 timesteps), timestep당 feature:

| 그룹 | Feature | 크기 |
|---|---|---|
| Phase | 0=align, 1=approach, 2=human (1단계 학습에선 상수 2) | 1 |
| Detection | cx_norm, cy_norm, area_norm, confidence, detected (human 구간은 동결값) | 5 |
| Joint angles | q1~q6 (JOINT_LIMITS로 정규화) | 6 |
| Delta angles | dq1~dq6 | 6 |
| Gripper | gripper_value / 100 | 1 |

timestep당 19 features × 5 steps = 95 input features

출력:

| Feature | 크기 |
|---|---|
| delta_angles (q1~q6) | 6 |
| gripper cmd (0=close, 1=open) | 1 |

총 7 outputs. gripper head는 inference에서 Task 7-B predictor가 go/no-go를 담당하므로 auxiliary supervision 역할.

### MLP 아키텍처

```
Input: [batch, 95]
  FC(95→256) + BatchNorm + ReLU + Dropout(0.2)
  FC(256→256) + BatchNorm + ReLU + Dropout(0.2)
  FC(256→128) + BatchNorm + ReLU
  FC(128→7)
    head_angles: FC(7→6) + tanh × max_delta_deg  (각도 clamp)
    head_gripper: FC(7→1) + sigmoid              (0~1)
```

Loss:
- angles: MSE
- gripper: BCE
- total: 0.8 × loss_angles + 0.2 × loss_gripper

저장: PyTorch `.pt` + ONNX export (`nn_controller_policy.onnx`)

---
## Task 7-B: Grip Success Predictor 학습 (별도 모델)

목적: 현재 state(95 features)를 입력받아 지금 grip하면 성공할 확률 P(success)를 예측. Policy network와 독립적으로 학습.

효용성:
- Policy network는 "어떻게 움직일지"를 결정하지만, "지금 grip해도 되는지"를 판단하지 못함
- P(success) < threshold이면 fine-tuning을 더 진행하고, P(success) >= threshold이면 grip 명령을 허용
- "go/no-go" 안전 게이트 역할로 불확실한 상태에서의 grip 실패를 사전에 방지

데이터: `success/` + `fail/` 에피소드 모두 사용.
각 에피소드에서 grip_triggered=true 시점의 5-frame window 1개를 추출, label 0(fail) or 1(success)

아키텍처:

```
Input: [batch, 95]
  FC(95→128) + ReLU + Dropout(0.3)
  FC(128→64) + ReLU + Dropout(0.3)
  FC(64→1) + sigmoid
Output: P(success) in [0, 1]
```

Loss: BCEWithLogitsLoss with pos_weight
success/fail 비율 불균형 보정 (success=positive 기준 pos_weight = N_fail / N_success):

```python
pos_weight = torch.tensor([n_fail / n_success])
criterion = nn.BCEWithLogitsLoss(pos_weight=pos_weight)
```

에피소드 수가 늘어나면 자동으로 weight 재계산.

저장: `grip_success_predictor.pt` + ONNX export (`grip_success_predictor.onnx`)

---
## Task 8: nn_controller_node.py 신규 생성 (1단계: fine-tune + grip 전담)

파일: `src/just_pick_it/just_pick_it_perception/just_pick_it_perception/nn_controller_node.py`

역할 (D1): align + approach는 기존 ibvs_controller가 수행하고, **ibvs_done 수신 후 활성화**되어 fine-tune + grip을 담당한다.

구독:
- `/{robot_name}/ibvs_done` (Empty): 활성화 트리거
- `detection_topic` (TrackedObjectArray): 활성화 직전 마지막 유효 detection을 동결값으로 캡처 (D3와 동일 규칙)
- `/{robot_name}/status` (Float64MultiArray)

발행:
- `/{robot_name}/target_pose` (Float64MultiArray): send_angles 명령
- `/{robot_name}/set_gripper` (Float64MultiArray)
- `/{robot_name}/request_status` (Empty)

내부 상태:
- `deque(maxlen=5)` sliding window buffer (활성화 시 IBVS 종단 상태로 seed)
- 데이터 수집 때와 달리 inference 중 arm은 powered 상태를 유지 (release_all_servos 호출하지 않음)

추론 루프 (status 수신 기반, 폴링 status_poll_rate_hz):
1. 현재 state 벡터 구성 후 window에 push (detection은 동결값)
2. window가 5개 찼으면 Policy forward, delta_angles를 target_pose(CMD_JOINT)로 발행
3. Grip success predictor forward, P(success) 계산
   - P(success) >= grip_confidence_threshold (default 0.8): set_gripper([0, 50]) 발행
   - P(success) < threshold: grip 보류, fine-tuning 계속
4. max_fine_tune_steps 초과 시 P(success)와 무관하게 강제 grip 또는 ERROR

---
## 구현 순서 (의존성 고려)

```
Task 2  HumanInteractionSample.msg 신규
Task 1  VisualServoSample.msg 확장 (interfaces 함께 rebuild)
  ↓
Task 3  ibvs_controller_node.py에 ibvs_done + ibvs_phase publisher 추가
Task 4  visual_servo_bag_recorder_node.py event-driven 전환 + 구독 추가
Task 5  human_interaction_recorder_node.py 신규 작성
  ↓
Task 6  nn_data_collection.launch.py 신규 작성
  ↓
[데이터 수집 반복 ~30+ 에피소드]
  ↓
Task 7 / 7-B  학습 스크립트 작성 및 학습
  ↓
Task 8  nn_controller_node.py 작성 및 테스트
```

---
## 검증 방법

- Task 1-2 (메시지): `colcon build --packages-select just_pick_it_interfaces` 후 `ros2 interface show`로 필드 확인
- Task 3 (ibvs 수정): 실제 로봇에서 ibvs 구동 후 `ros2 topic echo /{robot_name}/ibvs_done`, `/{robot_name}/ibvs_phase`로 시그널 확인
- Task 4 (recorder): rosbag에서 commanded_angles, controller_phase, episode_id 필드 확인. command 발행 횟수와 샘플 수 일치 확인. ERROR phase 주입 시 ABORTED 마커 생성 확인
- Task 5 (human recorder): 로봇 없이 mock status/ibvs_done topic을 publish하며 state machine 전환, movement_eps 필터, bag 생성 및 디렉토리 이동 확인
- Task 6 (launch): `ros2 launch just_pick_it_perception nn_data_collection.launch.py`로 세 노드 동시 기동, 두 recorder의 episode_id 일치 확인
- Task 7 (학습): 소량 에피소드로 loss 하강 확인, delta_angles 분포 sanity check, Jacobian probing 필터 동작 확인
- Task 8 (NN 노드): 정지 상태에서 detection 주고 출력 delta_angles가 0에 가까운지 확인, 과도한 명령 clamp 동작 확인, ibvs_done 전에는 명령을 발행하지 않는지 확인
