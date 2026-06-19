#!/usr/bin/env python3

"""
NN Controller 학습 스크립트 (PLAN.md Task 7 / Task 7-B, 1단계).

수집된 에피소드(ibvs + human rosbag)에서 (state, action) 데이터를 추출해
두 개의 모델을 학습한다.

  Task 7   : Policy network  (fine-tune delta_angles + gripper cmd)
             - success/ 에피소드의 human FREE_DRIVE 구간으로 학습
             - 5-step sliding window. window seed는 IBVS 종단(DONE 직전) 샘플 사용
  Task 7-B : Grip success predictor  (현재 state에서 grip 성공 확률)
             - success/ + fail/ 모두 사용
             - grip_triggered 시점의 5-frame window 1개 / 에피소드, label=성공여부

1단계 설계 (IBVS는 그대로 두고 human 정밀제어 구간만 NN으로 대체):
  - IBVS bag에서 controller_phase가 RUN(20)/APPROACH_WAIT(40)/DONE(90)이 아닌
    샘플(Jacobian probing 구간)은 제외한다.
  - anchor(window당 1회 동결) = [cx, cy, area, sin2θ, cos2θ].
      위치(cx,cy,area)는 IBVS 종단 마지막 유효값. orientation은 search 시점(첫 유효
      detection) OBB 각도 — J6(gripper rotation)는 IBVS가 안 건드리고 human이 물체
      방향에 맞춰 수동 설정하는데, 근접 시야는 OBB 각도가 부정확하기 때문이다.
  - phase / confidence / detected / gripper 는 human 구간 내내 상수라 입력에서 제외.
  - 시계열 입력은 관절각(6) + delta(6) 만. 입력 = anchor(5) + 12 × window = 65.
  - Policy 출력은 delta_angles(6)만. grip(0/1)은 별도 Grip Success Predictor가 전담한다.

실행 (ROS2 환경 source 필요 — rosbag2_py / 메시지 타입 사용):
  source install/setup.bash
  python3 src/just_pick_it/just_pick_it_perception/scripts/train_nn_controller.py \
      --data-dir ~/rosbags \
      --out-dir src/just_pick_it/just_pick_it_perception/result/nn_controller/pick

  place 학습은 place 수집 데이터와 place 출력 경로를 지정한다:
      --data-dir ~/rosbags_place \
      --out-dir src/just_pick_it/just_pick_it_perception/result/nn_controller/place
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

from rclpy.serialization import deserialize_message
import rosbag2_py

from just_pick_it_interfaces.msg import HumanInteractionSample, VisualServoSample


# ============================================================
# 상수 / 데이터 규약
# ============================================================

# jetcobot_bringup/jetcobot_joint_subscriber.py 의 JOINT_LIMITS 와 동일.
JOINT_LIMITS = [
    (-168.0, 168.0),
    (-135.0, 135.0),
    (-150.0, 150.0),
    (-145.0, 145.0),
    (-155.0, 160.0),
    (-180.0, 180.0),
]

GRIPPER_MAX = 100.0

# 학습에 사용할 IBVS controller_phase (Jacobian probing 구간 제외).
# Phase enum: RUN=20, APPROACH_WAIT=40, DONE=90.
IBVS_KEEP_PHASES = {20, 40, 90}

DEFAULT_IMAGE_W = 640.0
DEFAULT_IMAGE_H = 480.0

# 1단계 설계 (IBVS는 그대로 두고 human 정밀제어만 NN으로 대체):
#   - J6는 NN에서 완전히 제외한다. IBVS 수렴 후 ibvs_controller가 OBB 장축으로 J6를
#     결정론적으로 정렬하고(j6 += sign*obb + offset), 이후 고정한다. 따라서 NN은
#     J1~J5(=CTRL_IDX)만 제어하고, orientation/J6 관련 feature도 두지 않는다.
#   - phase / confidence / detected / gripper 도 human 구간 내내 상수라 제외.
#   - detection은 IBVS 종단 마지막 유효값(cx, cy, area)을 anchor로 window당 1회만.
#   - 시계열은 제어 관절각(J1~J5) + delta(dJ1~dJ5) 만.
#   - grip(0/1) 판단은 Policy가 아니라 별도 Grip Success Predictor가 전담.
N_JOINTS = 6              # 전체 로봇 관절 수
CTRL_IDX = [0, 1, 2, 3, 4]  # NN이 제어하는 관절 (J1~J5). J6(5) 제외.
N_CTRL = len(CTRL_IDX)
ANCHOR_DIM = 3            # anchor: cx_norm, cy_norm, area_norm
# 진행도(progress): 활성화(시작) 자세 대비 제어 관절 누적 변위 Δq (현재값 1개).
# "시작에서 얼마나 움직였나"를 정책에 직접 제공해 정지 학습을 돕는다.
PROGRESS_DIM = N_CTRL    # 5
PROGRESS_SCALE_DEG = 90.0  # Δq 정규화 스케일
# policy 입력 구조 (--policy-input, main에서 설정):
#   memoryless: step = joint(5)만, window=1. 입력 = 3+5+5 = 13.
#     delta 이력은 "직전 움직임 복사" 지름길 학습을 유발해 런타임에서 중력 처짐 등
#     측정 바이어스를 증폭하는 것이 확인되어(ablation: closed-loop 도달 거리
#     memoryless 4.3deg vs window 7.7deg, 처짐 주입 시 6.3 vs 12.0) 기본값으로 한다.
#   window: step = joint(5)+delta(5), window=5. 입력 = 3+5+50 = 58 (기존 구조).
USE_DELTA_FEATURES = False
# progress(시작 자세 대비 Δq) 사용 여부. to_goal에서 target은 q_final-q로 q_start와
# 무관한데, progress는 q_start 의존성을 입력에 주입해 비전형 q_start에서 OOD 언더슈트를
# 유발하는 것이 확인되어(같은 q에서 progress만 바꿔도 출력 방향이 뒤집힘) 기본 비활성.
USE_PROGRESS = False
FEATURES_PER_STEP = N_CTRL
WINDOW = 1
INPUT_DIM = ANCHOR_DIM + PROGRESS_DIM + FEATURES_PER_STEP * WINDOW


# ============================================================
# Rosbag 읽기
# ============================================================
def _open_reader(uri, storage_id="sqlite3"):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(uri), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def normalize_joints(angles):
    # 제어 관절(J1~J5)만 정규화한다.
    out = np.zeros(N_CTRL, dtype=np.float32)
    for k, i in enumerate(CTRL_IDX):
        lo, hi = JOINT_LIMITS[i]
        span = max(hi - lo, 1e-6)
        out[k] = float(np.clip(2.0 * (float(angles[i]) - lo) / span - 1.0, -1.0, 1.0))
    return out


def scale_delta(delta, max_delta_deg):
    # 제어 관절(J1~J5)의 delta만 스케일한다.
    out = np.zeros(N_CTRL, dtype=np.float32)
    for k, i in enumerate(CTRL_IDX):
        out[k] = float(np.clip(float(delta[i]) / max_delta_deg, -1.0, 1.0))
    return out


def scale_progress(dq_from_start):
    # 시작 자세 대비 누적 변위(제어 관절)를 PROGRESS_SCALE_DEG로 정규화.
    out = np.zeros(N_CTRL, dtype=np.float32)
    for k, i in enumerate(CTRL_IDX):
        out[k] = float(np.clip(float(dq_from_start[i]) / PROGRESS_SCALE_DEG, -1.0, 1.0))
    return out


class Frame:
    """
    combined sequence의 한 timestep.
    ts_feat = [J1~J5 정규화(+delta 스케일, window 모드)]. 학습 target용 raw 값도 보관.
    """

    __slots__ = ("ts_feat", "raw_delta_deg", "progress", "is_human",
                 "grip_triggered", "raw_joints")

    def __init__(self, ts_feat, raw_delta_deg, progress, is_human,
                 grip_triggered, raw_joints=None):
        self.ts_feat = ts_feat              # np.float32 (5 or 10,)
        self.raw_delta_deg = raw_delta_deg  # np.float32 (6,) degrees (전체 관절)
        self.progress = progress            # np.float32 (5,) 시작 자세 대비 Δq(scaled)
        self.is_human = is_human            # bool
        self.grip_triggered = grip_triggered  # bool
        self.raw_joints = raw_joints        # np.float32 (6,) degrees (to_goal target용)


def _make_ts_feat(joints, delta, max_delta_deg):
    if not USE_DELTA_FEATURES:
        # memoryless: 관절각만. delta 이력 채널을 제거해 관성 복사 학습을 차단한다.
        return normalize_joints(joints).astype(np.float32)
    return np.concatenate([
        normalize_joints(joints),
        scale_delta(delta, max_delta_deg),
    ]).astype(np.float32)


def _downsample_human(human_raw, target_control_hz):
    """
    human FREE_DRIVE 샘플을 target_control_hz 주기로 다운샘플하고, 선택된 인접 샘플
    간 joint 차이로 delta를 재계산한다. target_control_hz<=0이면 기록 그대로 사용하되
    delta는 인접 차이로 재계산(기록 rate가 학습 timestep).
    grip_triggered 샘플은 항상 포함한다.
    """
    if len(human_raw) <= 1:
        for i, r in enumerate(human_raw):
            r["delta"] = (np.zeros(6, dtype=np.float32) if i == 0
                          else human_raw[i]["joints"] - human_raw[i - 1]["joints"])
        return human_raw

    if target_control_hz and target_control_hz > 0.0:
        min_dt = 1.0 / float(target_control_hz)
        selected = [human_raw[0]]
        last_t = human_raw[0]["t"]
        for r in human_raw[1:]:
            if r["grip_triggered"] or (r["t"] - last_t) >= (min_dt - 1e-3):
                selected.append(r)
                last_t = r["t"]
        # 마지막 샘플(끝점)이 빠졌으면 포함.
        if selected[-1] is not human_raw[-1]:
            selected.append(human_raw[-1])
    else:
        selected = list(human_raw)

    # delta 재계산 (다운샘플된 인접 간 차이).
    for i, r in enumerate(selected):
        if i == 0:
            r["delta"] = np.zeros(6, dtype=np.float32)
        else:
            r["delta"] = (selected[i]["joints"] - selected[i - 1]["joints"]).astype(np.float32)
    return selected


def build_episode_sequence(episode_dir, max_delta_deg, seed_len, target_control_hz=0.0):
    """
    한 에피소드를 (seq, anchor)로 변환한다.
    seq = [IBVS 종단 seed frames] + [human FREE_DRIVE frames].
    anchor = [cx_norm, cy_norm, area_norm] (window당 1회 동결).
      - IBVS 종단의 마지막 유효 detection (정밀제어 시작 시 화면 목표).
      - J6는 ibvs_controller가 결정론적으로 정렬하므로 orientation feature는 두지 않는다.

    target_control_hz > 0 이면 human FREE_DRIVE 샘플을 그 주기로 다운샘플한다.
    기록은 고속(예: 10Hz)으로 충실히 하고, 학습/추론 timestep은 더 낮게(예: 5Hz)
    분리하기 위함. delta는 다운샘플된 인접 샘플 간 joint 차이로 재계산한다.
    """
    episode_dir = Path(episode_dir)
    ibvs_uri = episode_dir / "ibvs"
    human_uri = episode_dir / "human"
    if not ibvs_uri.exists() or not human_uri.exists():
        return None, None

    # --- IBVS 읽기 ---
    ibvs_raw = []
    pos_anchor = None  # (cx_norm, cy_norm, area_norm), 마지막 유효값
    reader = _open_reader(ibvs_uri)
    while reader.has_next():
        _topic, data, _t = reader.read_next()
        try:
            m = deserialize_message(data, VisualServoSample)
        except Exception:
            # 구버전 메시지로 기록된 bag. 재수집 필요.
            print(f"  [skip] {episode_dir.name}: VisualServoSample 역직렬화 실패 "
                  f"(구버전 bag). 재수집 필요.")
            return None, None

        img_w = float(m.image_width) if float(m.image_width) > 0 else DEFAULT_IMAGE_W
        img_h = float(m.image_height) if float(m.image_height) > 0 else DEFAULT_IMAGE_H

        # 위치 anchor는 마지막 유효 detection으로 갱신 (last_valid_*).
        if float(m.last_valid_area_norm) > 0.0:
            pos_anchor = (
                float(m.last_valid_cx) / img_w,
                float(m.last_valid_cy) / img_h,
                float(m.last_valid_area_norm),
            )

        if int(m.controller_phase) not in IBVS_KEEP_PHASES:
            continue
        if not bool(m.has_command):
            continue

        ibvs_raw.append({
            "joints": np.array(m.joint_angles, dtype=np.float32),
            "delta": np.array(m.commanded_delta, dtype=np.float32),
        })

    # --- human 읽기 ---
    human_raw = []
    reader = _open_reader(human_uri)
    while reader.has_next():
        _topic, data, _t = reader.read_next()
        m = deserialize_message(data, HumanInteractionSample)
        # FREE_DRIVE(1) 구간 + grip 이벤트만 사용. result(3) 등 종료 샘플은 제외.
        if int(m.phase) != 1:
            continue
        human_raw.append({
            "t": float(_t) * 1e-9,
            "joints": np.array(m.joint_angles, dtype=np.float32),
            "delta": np.array(m.delta_angles, dtype=np.float32),
            "grip_triggered": bool(m.grip_triggered),
        })

    if len(human_raw) == 0:
        return None, None

    # 다운샘플: 기록(고속)을 학습 timestep(target_control_hz)으로 솎고 delta 재계산.
    human_raw = _downsample_human(human_raw, target_control_hz)

    if pos_anchor is None:
        pos_anchor = (0.0, 0.0, 0.0)

    anchor = np.array(
        [pos_anchor[0], pos_anchor[1], pos_anchor[2]],
        dtype=np.float32,
    )

    # 진행도 기준점 = 활성화(시작) 자세 = 첫 human FREE_DRIVE 샘플의 관절각.
    start_joints = human_raw[0]["joints"].astype(np.float32)

    seq = []
    # IBVS seed frames (종단 seed_len 개). 시작 자세 근처라 progress ≈ 0.
    for r in (ibvs_raw[-seed_len:] if seed_len > 0 else []):
        seq.append(Frame(
            _make_ts_feat(r["joints"], r["delta"], max_delta_deg),
            r["delta"].astype(np.float32),
            scale_progress(r["joints"] - start_joints),
            False, False,
            r["joints"].astype(np.float32),
        ))
    # human frames.
    for r in human_raw:
        seq.append(Frame(
            _make_ts_feat(r["joints"], r["delta"], max_delta_deg),
            r["delta"].astype(np.float32),
            scale_progress(r["joints"] - start_joints),
            True, r["grip_triggered"],
            r["joints"].astype(np.float32),
        ))

    return seq, anchor


# ============================================================
# 데이터셋 구성
# ============================================================
def _flatten_window(anchor, window):
    """
    입력 = [anchor(3)] + [progress(5)] + [window 각 frame의 ts_feat(10)].
    progress는 window의 마지막(가장 최근) frame 기준(현재 진행도).
    USE_PROGRESS=False면 progress 자리를 0으로 채운다(차원은 유지).
    """
    prog = window[-1].progress if USE_PROGRESS else np.zeros_like(window[-1].progress)
    return np.concatenate(
        [anchor, prog] + [f.ts_feat for f in window]
    ).astype(np.float32)


def make_policy_samples(seq, anchor, max_delta_deg):
    """
    Policy 학습 쌍 생성.
    window = seq[t-WINDOW : t] (5 frame), target = seq[t]의 delta_angles.
    target은 human frame일 때만 생성. 첫 human frame(delta=0)은 제외.
    """
    X, Yd = [], []
    for t in range(WINDOW, len(seq)):
        frame = seq[t]
        if not frame.is_human:
            continue
        # 직전 frame이 human이 아니면 첫 human transition(delta=0 garbage) → skip.
        if not seq[t - 1].is_human:
            continue
        window = seq[t - WINDOW:t]
        X.append(_flatten_window(anchor, window))
        Yd.append(scale_delta(frame.raw_delta_deg, max_delta_deg))
    return X, Yd


def make_policy_samples_to_goal(seq, anchor, max_delta_deg,
                                noise_deg, n_aug, rng):
    """
    목표 지향 재라벨링: target = 현재 자세에서 q_final(grip 자세)로 향하는 방향.
      - 방향 보존 캡: vec * min(1, max_delta/max|성분|). 관절별 독립 클리핑과 달리
        방향 비율이 왜곡되지 않고, q_final 근처에서 크기가 줄어 정지가 학습된다.
      - 모든 frame이 같은 끝점을 가리키므로 수렴장(attractor)이 만들어진다.
      - 노이즈 증강(memoryless 전용): label을 임의 자세에서 해석적으로 계산할 수
        있으므로(q_final - q), 시연 궤적 주변 noise_deg 이내 상태를 n_aug개씩
        합성해 궤적 이탈 시 복귀를 학습시킨다.
    """
    q_final = None
    for f in seq:
        if f.grip_triggered and f.raw_joints is not None:
            q_final = f.raw_joints
    if q_final is None:
        hum = [f for f in seq if f.is_human and f.raw_joints is not None]
        if not hum:
            return [], []
        q_final = hum[-1].raw_joints

    hum_idx = [i for i, f in enumerate(seq) if f.is_human]
    if not hum_idx:
        return [], []
    start_joints = seq[hum_idx[0]].raw_joints

    def goal_label(q):
        vec = (q_final - q).astype(np.float32)
        m = max(abs(float(vec[i])) for i in CTRL_IDX)
        if m > max_delta_deg:
            vec = vec * (max_delta_deg / m)
        return scale_delta(vec, max_delta_deg)

    X, Y = [], []
    for t in range(WINDOW, len(seq) + 1):
        window = seq[t - WINDOW:t]
        f = window[-1]
        if f.raw_joints is None:
            continue
        X.append(_flatten_window(anchor, window))
        Y.append(goal_label(f.raw_joints))

        # 노이즈 증강은 이력이 없는 memoryless(window=1)에서만 일관되게 가능.
        if n_aug > 0 and WINDOW == 1:
            for _ in range(n_aug):
                noise = rng.uniform(-noise_deg, noise_deg, size=6).astype(np.float32)
                noise[5] = 0.0
                qn = f.raw_joints + noise
                fr = Frame(
                    _make_ts_feat(qn, np.zeros(6, np.float32), max_delta_deg),
                    np.zeros(6, np.float32),
                    scale_progress(qn - start_joints),
                    f.is_human, False, qn,
                )
                X.append(_flatten_window(anchor, [fr]))
                Y.append(goal_label(qn))
    return X, Y


def make_grip_sample(seq, anchor):
    """grip_triggered 시점에서 끝나는 5-frame window 1개를 반환. 없으면 None."""
    grip_idx = None
    for i, f in enumerate(seq):
        if f.grip_triggered:
            grip_idx = i
            break
    if grip_idx is None:
        return None
    start = grip_idx - WINDOW + 1
    if start < 0:
        return None
    window = seq[start:grip_idx + 1]
    if len(window) != WINDOW:
        return None
    return _flatten_window(anchor, window)


# ============================================================
# 모델
# ============================================================
class PolicyNet(nn.Module):
    """
    Policy MLP. delta_angles만 출력한다(gripper head 없음).
    grip 판단은 GripSuccessNet이 전담한다.
    """

    def __init__(self, max_delta_deg):
        super().__init__()
        self.max_delta_deg = max_delta_deg
        self.backbone = nn.Sequential(
            nn.Linear(INPUT_DIM, 256), nn.BatchNorm1d(256), nn.ReLU(), nn.Dropout(0.2),
            nn.Linear(256, 256), nn.BatchNorm1d(256), nn.ReLU(), nn.Dropout(0.2),
            nn.Linear(256, 128), nn.BatchNorm1d(128), nn.ReLU(),
        )
        self.head_angles = nn.Linear(128, N_CTRL)

    def forward(self, x):
        h = self.backbone(x)
        # 학습/추론 일관성을 위해 정규화된 delta([-1,1])를 출력한다.
        # 추론 노드에서 max_delta_deg를 곱해 실제 각도로 변환한다.
        return torch.tanh(self.head_angles(h))


class GripSuccessNet(nn.Module):
    """PLAN.md Task 7-B. logit 출력(BCEWithLogitsLoss)."""

    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(INPUT_DIM, 128), nn.ReLU(), nn.Dropout(0.3),
            nn.Linear(128, 64), nn.ReLU(), nn.Dropout(0.3),
            nn.Linear(64, 1),
        )

    def forward(self, x):
        return self.net(x)


# ============================================================
# 학습 루프
# ============================================================
def split_indices(n, val_ratio, generator):
    idx = torch.randperm(n, generator=generator).tolist()
    n_val = int(round(n * val_ratio))
    if n < 8:
        n_val = 0
    return idx[n_val:], idx[:n_val]


def train_policy(X, Yd, args, device):
    n = len(X)
    if n == 0:
        print("[policy] 학습 샘플이 없습니다. 건너뜀.")
        return None

    X = torch.tensor(np.stack(X), dtype=torch.float32)
    Yd = torch.tensor(np.stack(Yd), dtype=torch.float32)

    gen = torch.Generator().manual_seed(args.seed)
    train_idx, val_idx = split_indices(n, args.val_ratio, gen)

    ds_train = TensorDataset(X[train_idx], Yd[train_idx])
    loader = DataLoader(ds_train, batch_size=min(args.batch_size, len(ds_train)),
                        shuffle=True, drop_last=len(ds_train) >= args.batch_size)

    model = PolicyNet(args.max_delta_deg).to(device)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr,
                           weight_decay=args.weight_decay)
    mse = nn.MSELoss()

    print(f"[policy] samples={n} (train={len(train_idx)}, val={len(val_idx)}), "
          f"input_dim={INPUT_DIM}, weight_decay={args.weight_decay}, "
          f"patience={args.patience}")

    has_val = len(val_idx) > 0
    Xv = X[val_idx].to(device) if has_val else None
    Yv = Yd[val_idx].to(device) if has_val else None

    best_val = float("inf")
    best_state = None
    best_epoch = -1
    no_improve = 0
    print_every = max(1, args.epochs // 20)

    for epoch in range(1, args.epochs + 1):
        model.train()
        tot = 0.0
        nb = 0
        for xb, ydb in loader:
            xb, ydb = xb.to(device), ydb.to(device)
            opt.zero_grad()
            loss = mse(model(xb), ydb)
            loss.backward()
            opt.step()
            tot += float(loss.item())
            nb += 1

        vloss = None
        if has_val:
            model.eval()
            with torch.no_grad():
                vloss = float(mse(model(Xv), Yv))
            # 과적합 방지: val 최저 모델을 보존(마지막 epoch 모델이 아니라 best를 저장).
            if vloss < best_val - 1e-6:
                best_val = vloss
                best_epoch = epoch
                best_state = {k: v.detach().cpu().clone()
                              for k, v in model.state_dict().items()}
                no_improve = 0
            else:
                no_improve += 1

        if epoch % print_every == 0 or epoch == 1:
            msg = f"[policy] epoch {epoch:5d}  train_loss={tot / max(nb,1):.5f}"
            if vloss is not None:
                msg += f"  val_loss={vloss:.5f}  best={best_val:.5f}@{best_epoch}"
            print(msg)

        # early stopping: val 개선이 patience epoch 동안 없으면 중단.
        if args.patience > 0 and has_val and no_improve >= args.patience:
            print(f"[policy] early stop at epoch {epoch} "
                  f"(no val improvement for {args.patience} epochs). "
                  f"best val_loss={best_val:.5f} @ epoch {best_epoch}")
            break

    # 과적합 직전(val 최저) 모델로 복원.
    if best_state is not None:
        model.load_state_dict(best_state)
        print(f"[policy] restored best model: val_loss={best_val:.5f} @ epoch {best_epoch}")

    # sanity: 출력 delta 분포
    model.eval()
    with torch.no_grad():
        pd_deg = (model(X.to(device)) * args.max_delta_deg).cpu().numpy()
    print(f"[policy] pred delta(deg) mean|.|={np.mean(np.abs(pd_deg)):.3f}, "
          f"max|.|={np.max(np.abs(pd_deg)):.3f}")
    return model


def train_grip_predictor(Xg, Yg, args, device):
    n = len(Xg)
    if n == 0:
        print("[grip] 학습 샘플이 없습니다. 건너뜀.")
        return None

    X = torch.tensor(np.stack(Xg), dtype=torch.float32)
    Y = torch.tensor(np.array(Yg, dtype=np.float32).reshape(-1, 1))

    n_pos = float(Y.sum().item())          # success
    n_neg = float(n - n_pos)               # fail
    pos_weight = torch.tensor(
        [n_neg / n_pos if n_pos > 0 else 1.0], dtype=torch.float32
    ).to(device)

    gen = torch.Generator().manual_seed(args.seed)
    train_idx, val_idx = split_indices(n, args.val_ratio, gen)

    ds_train = TensorDataset(X[train_idx], Y[train_idx])
    loader = DataLoader(ds_train, batch_size=min(args.batch_size, len(ds_train)),
                        shuffle=True)

    model = GripSuccessNet().to(device)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr,
                           weight_decay=args.weight_decay)
    crit = nn.BCEWithLogitsLoss(pos_weight=pos_weight)

    print(f"[grip] samples={n} (success={int(n_pos)}, fail={int(n_neg)}), "
          f"pos_weight={float(pos_weight):.3f}, train={len(train_idx)}, val={len(val_idx)}")

    for epoch in range(1, args.epochs + 1):
        model.train()
        tot = 0.0
        nb = 0
        for xb, yb in loader:
            xb, yb = xb.to(device), yb.to(device)
            opt.zero_grad()
            loss = crit(model(xb), yb)
            loss.backward()
            opt.step()
            tot += float(loss.item())
            nb += 1
        if epoch % max(1, args.epochs // 10) == 0 or epoch == 1:
            print(f"[grip] epoch {epoch:4d}  train_loss={tot / max(nb,1):.5f}")

    model.eval()
    with torch.no_grad():
        p = torch.sigmoid(model(X.to(device))).cpu().numpy().reshape(-1)
    for i in range(n):
        print(f"[grip] sample {i}: label={int(Yg[i])} P(success)={p[i]:.3f}")
    return model


# ============================================================
# 저장 / ONNX export
# ============================================================
def export_onnx(model, path, device):
    try:
        dummy = torch.zeros(1, INPUT_DIM, dtype=torch.float32, device=device)
        model.eval()
        # torch 2.x 기본 dynamo exporter는 onnxscript를 요구하므로
        # 추가 의존성이 없는 legacy TorchScript exporter(dynamo=False)를 사용한다.
        torch.onnx.export(
            model, dummy, str(path),
            input_names=["state_window"],
            output_names=(["delta_norm"]
                          if isinstance(model, PolicyNet) else ["success_logit"]),
            dynamic_axes={"state_window": {0: "batch"}},
            opset_version=17,
            dynamo=False,
        )
        print(f"  ONNX export 완료: {path}")
    except Exception as exc:
        print(f"  ONNX export 실패(무시): {exc}")


# ============================================================
# main
# ============================================================
def collect_episode_dirs(base):
    out = []
    for sub in ("success", "fail"):
        d = base / sub
        if not d.is_dir():
            continue
        for ep in sorted(d.iterdir()):
            if ep.is_dir() and (ep / "ibvs").exists() and (ep / "human").exists():
                out.append((sub, ep))
    return out


def main():
    global WINDOW, INPUT_DIM, FEATURES_PER_STEP, USE_DELTA_FEATURES, USE_PROGRESS

    parser = argparse.ArgumentParser(description="NN controller 학습 (Task 7 / 7-B)")
    parser.add_argument("--data-dir", default="~/rosbags")
    # 기본은 픽 학습 산출물 경로. place 학습 시 --out-dir 을 result/nn_controller/place 로 지정.
    parser.add_argument("--out-dir",
                        default="src/just_pick_it/just_pick_it_perception/result/nn_controller/pick")
    parser.add_argument("--policy-input", choices=["memoryless", "window"],
                        default="memoryless",
                        help="memoryless(기본): anchor+progress+현재 관절각(13차원). "
                             "window: 기존 q+delta 5-frame window(58차원)로 되돌리기.")
    parser.add_argument("--policy-target", choices=["to_goal", "next_delta"],
                        default="to_goal",
                        help="to_goal(기본): target = q_final로 향하는 방향(수렴장 학습). "
                             "next_delta: 기존 사람 다음 step 모방으로 되돌리기.")
    parser.add_argument("--anchor-mode", choices=["zero", "frozen"],
                        default="zero",
                        help="zero(기본): anchor 3차원을 학습/추론 모두 0으로 고정. "
                             "anchor는 물체별 정보가 없는 반면(LOO 검증) 런타임 area가 "
                             "학습 분포 밖(0.7~0.8)이라 수렴장을 왜곡하는 부채로 확인됨. "
                             "frozen: 기존 방식(IBVS 종단 detection 동결)으로 되돌리기.")
    parser.add_argument("--progress-mode", choices=["zero", "use"], default="zero",
                        help="zero(기본): progress(q_start 대비 Δq) 입력을 0으로. "
                             "to_goal에서 잉여이며 비전형 q_start OOD 언더슈트를 유발. "
                             "use: 기존처럼 progress 사용.")
    parser.add_argument("--augment-noise-deg", type=float, default=3.0,
                        help="to_goal 노이즈 증강 폭(deg). memoryless에서만 적용.")
    parser.add_argument("--augment-copies", type=int, default=5,
                        help="frame당 노이즈 증강 샘플 수. 0이면 비활성.")
    parser.add_argument("--window", type=int, default=5,
                        help="--policy-input window 일 때의 window 크기")
    parser.add_argument("--seed-len", type=int, default=5,
                        help="IBVS 종단 seed frame 개수")
    parser.add_argument("--target-control-hz", type=float, default=3.0,
                        help="학습/추론 timestep(Hz). 기록(고속)을 이 주기로 다운샘플. "
                             "3Hz 권장(샘플 수 vs delta 크기 균형). 0이면 기록 rate 그대로.")
    parser.add_argument("--max-delta-deg", type=float, default=5.0)
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--weight-decay", type=float, default=1e-4,
                        help="L2 정규화(과적합 완화). 데이터 적을수록 키워볼 것(예: 1e-3)")
    parser.add_argument("--patience", type=int, default=1000,
                        help="val 개선 없이 이 epoch 지나면 early stop(best 모델 저장). 0=비활성")
    parser.add_argument("--val-ratio", type=float, default=0.15)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--skip-onnx", action="store_true")
    args = parser.parse_args()

    if args.policy_input == "memoryless":
        USE_DELTA_FEATURES = False
        FEATURES_PER_STEP = N_CTRL
        WINDOW = 1
    else:
        USE_DELTA_FEATURES = True
        FEATURES_PER_STEP = 2 * N_CTRL
        WINDOW = args.window
    USE_PROGRESS = (args.progress_mode == "use")
    INPUT_DIM = ANCHOR_DIM + PROGRESS_DIM + FEATURES_PER_STEP * WINDOW
    print(f"policy_input={args.policy_input} (window={WINDOW}, "
          f"features_per_step={FEATURES_PER_STEP}, input_dim={INPUT_DIM}), "
          f"progress_mode={args.progress_mode}")

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    data_dir = Path(args.data_dir).expanduser()
    out_dir = Path(args.out_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    device = torch.device(args.device)

    episodes = collect_episode_dirs(data_dir)
    if not episodes:
        raise SystemExit(f"에피소드를 찾지 못했습니다: {data_dir}/{{success,fail}}")

    print(f"device={device}, episodes={len(episodes)} "
          f"(success={sum(1 for s,_ in episodes if s=='success')}, "
          f"fail={sum(1 for s,_ in episodes if s=='fail')})")

    # --- 데이터 추출 ---
    pX, pYd = [], []              # policy (success only)
    gX, gY = [], []               # grip predictor (success + fail)
    aug_rng = np.random.default_rng(args.seed)

    for label, ep in episodes:
        seq, anchor = build_episode_sequence(
            ep, args.max_delta_deg, args.seed_len, args.target_control_hz
        )
        if seq is None:
            print(f"  [skip] {ep.name}: 시퀀스 비어있음")
            continue

        if args.anchor_mode == "zero":
            anchor = np.zeros(ANCHOR_DIM, dtype=np.float32)

        print(f"  [{label}] {ep.name}: anchor(cx,cy,area)="
              f"({anchor[0]:.3f},{anchor[1]:.3f},{anchor[2]:.3f}), "
              f"frames={len(seq)}")

        if label == "success":
            if args.policy_target == "to_goal":
                x, yd = make_policy_samples_to_goal(
                    seq, anchor, args.max_delta_deg,
                    args.augment_noise_deg, args.augment_copies, aug_rng,
                )
            else:
                x, yd = make_policy_samples(seq, anchor, args.max_delta_deg)
            pX += x
            pYd += yd

        grip_win = make_grip_sample(seq, anchor)
        if grip_win is not None:
            gX.append(grip_win)
            gY.append(1.0 if label == "success" else 0.0)
        else:
            print(f"  [warn] {ep.name}: grip window 추출 실패")

    # --- 정규화/메타 설정 저장 (Task 8 추론 노드에서 재사용) ---
    config = {
        "window": WINDOW,
        "anchor_dim": ANCHOR_DIM,
        "features_per_step": FEATURES_PER_STEP,
        # 추론 노드(FeatureBuilder)가 이 플래그로 step feature 구성을 분기한다.
        "policy_input": args.policy_input,
        "policy_target": args.policy_target,
        # zero: 추론 노드도 anchor를 0으로 넣고 detection 대기를 생략한다.
        "anchor_mode": args.anchor_mode,
        # zero: 추론 노드도 progress를 0으로 넣는다(FeatureBuilder가 처리).
        "progress_mode": args.progress_mode,
        "use_delta_features": USE_DELTA_FEATURES,
        "input_dim": INPUT_DIM,
        "max_delta_deg": args.max_delta_deg,
        # 학습/추론 timestep(Hz). nn_controller는 이 주기로 동작해야 일관성 유지.
        "target_control_hz": args.target_control_hz,
        # 진행도(시작 자세 대비 Δq) feature.
        "progress_dim": PROGRESS_DIM,
        "progress_scale_deg": PROGRESS_SCALE_DEG,
        "joint_limits": JOINT_LIMITS,
        "controlled_joints": CTRL_IDX,   # NN이 제어하는 관절 (J1~J5). J6 제외.
        "gripper_max": GRIPPER_MAX,
        "default_image_w": DEFAULT_IMAGE_W,
        "default_image_h": DEFAULT_IMAGE_H,
        "ibvs_keep_phases": sorted(IBVS_KEEP_PHASES),
        # 입력 = anchor(3) + progress(5) + window 각 step의 ts_feat(10).
        "anchor_layout": ["anchor_cx_norm", "anchor_cy_norm", "anchor_area_norm"],
        "progress_layout": ["prog_q1", "prog_q2", "prog_q3", "prog_q4", "prog_q5"],
        "step_layout": (
            ["q1n", "q2n", "q3n", "q4n", "q5n"]
            + (["dq1s", "dq2s", "dq3s", "dq4s", "dq5s"]
               if USE_DELTA_FEATURES else [])
        ),
        "policy_output": ["dq1..dq5 (tanh, ×max_delta_deg for degrees). J6 excluded."],
        "grip_decision": "GripSuccessPredictor P(success) gate (policy has no gripper head)",
    }
    with open(out_dir / "nn_controller_config.json", "w") as f:
        json.dump(config, f, indent=2)
    print(f"config 저장: {out_dir / 'nn_controller_config.json'}")

    # --- Task 7: Policy ---
    print("\n=== Task 7: Policy network ===")
    policy = train_policy(pX, pYd, args, device)
    if policy is not None:
        pt_path = out_dir / "nn_controller_policy.pt"
        torch.save({"state_dict": policy.state_dict(), "config": config}, pt_path)
        print(f"  저장: {pt_path}")
        if not args.skip_onnx:
            export_onnx(policy, out_dir / "nn_controller_policy.onnx", device)

    # --- Task 7-B: Grip success predictor ---
    print("\n=== Task 7-B: Grip success predictor ===")
    grip = train_grip_predictor(gX, gY, args, device)
    if grip is not None:
        pt_path = out_dir / "grip_success_predictor.pt"
        torch.save({"state_dict": grip.state_dict(), "config": config}, pt_path)
        print(f"  저장: {pt_path}")
        if not args.skip_onnx:
            export_onnx(grip, out_dir / "grip_success_predictor.onnx", device)

    print("\n완료.")


if __name__ == "__main__":
    main()
