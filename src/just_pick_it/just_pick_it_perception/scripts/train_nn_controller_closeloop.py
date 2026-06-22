#!/usr/bin/env python3

"""
Closed-loop NN controller 학습 (방식 1 재수집 데이터 전용).

기존 train_nn_controller.py 와 별개 파일이다(롤백/참조용으로 기존 파일 보존).
핵심 차이: 입력을 zeroed anchor/progress 대신 live 시각오차로 교체한다.

입력 (8차원, --include-area=false 면 7차원):
    [e_u, e_v, (area_norm), q1n..q5n]
      e_u = (det_cx - target_cx) / image_w
      e_v = (det_cy - target_cy) / image_h
      area_norm = det_area_norm
      q1n..q5n = J1~J5 정규화 [-1,1]
    target_cx/cy = off-center 그립 목표점. 성공 에피소드의 last-visible(grip 직전
      마지막 유효 detection) 프레임 물체 위치의 median 으로 데이터에서 추출한다
      (화면 정중앙이면 못 잡으므로 중앙이 아니다).
    det_cx/cy 는 recorder 가 이미 live-or-frozen(검출 소실 시 마지막 유효값 유지)으로
      저장하므로 학습 입력도 자동으로 closed/open 일관이 된다. valid 여부는
      det_age_sec 로 --det-valid-timeout 기준 재계산한다(녹화값 박제 아님).

출력: tanh(delta) × max_delta_deg, J1~J5 (J6 제외, ibvs_controller 가 정렬).
target: to_goal(현재 q 에서 q_final 로 향하는 방향, 방향보존 캡). 시연 궤적의
    불필요한 왔다갔다는 to_goal 재라벨링이 흡수하므로 그대로 둔다.
    closed-loop 입력에서는 synthetic q-noise 증강이 시각오차와 불일치하므로 쓰지 않는다.

grip predictor: last-visible 프레임 state(= open-loop 전환점)에서 P(success). 입력은
    policy 와 동일 레이아웃. label 은 에피소드 성공/실패.

데이터: human bag 만 사용(HumanInteractionSample det_* 필드 필요). ibvs bag 불필요.

실행:
  source install/setup.bash
  python3 src/just_pick_it/just_pick_it_perception/scripts/train_nn_controller_closeloop.py \
      --data-dir ~/rosbags_closeloop \
      --out-dir src/just_pick_it/just_pick_it_perception/result/nn_controller/pick
"""

import argparse
import json
from pathlib import Path

import numpy as np

import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

from rclpy.serialization import deserialize_message
import rosbag2_py

from just_pick_it_interfaces.msg import HumanInteractionSample


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

N_JOINTS = 6
CTRL_IDX = [0, 1, 2, 3, 4]   # NN 이 제어하는 관절(J1~J5). J6 제외.
N_CTRL = len(CTRL_IDX)

DEFAULT_IMAGE_W = 640.0
DEFAULT_IMAGE_H = 480.0

# HumanInteractionSample.phase 의 FREE_DRIVE.
PHASE_FREE_DRIVE = 1


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


def read_human_episode(human_uri):
    """human bag 의 FREE_DRIVE 샘플을 시간순 list 로 반환. det 필드 없으면 None."""
    frames = []
    reader = _open_reader(human_uri)
    while reader.has_next():
        _topic, data, _t = reader.read_next()
        try:
            m = deserialize_message(data, HumanInteractionSample)
        except Exception:
            # det_age_sec 추가 이전 메시지로 기록된 구버전 bag. 재수집 필요.
            return None
        if not hasattr(m, "det_age_sec"):
            return None
        if int(m.phase) != PHASE_FREE_DRIVE:
            continue
        iw = float(m.det_image_width) if float(m.det_image_width) > 0 else DEFAULT_IMAGE_W
        ih = float(m.det_image_height) if float(m.det_image_height) > 0 else DEFAULT_IMAGE_H
        frames.append({
            "joints": np.array(m.joint_angles, dtype=np.float32),
            "grip_triggered": bool(m.grip_triggered),
            "det_cx": float(m.det_cx),
            "det_cy": float(m.det_cy),
            "det_area": float(m.det_area_norm),
            "det_age": float(m.det_age_sec),
            "iw": iw,
            "ih": ih,
        })
    return frames if frames else None


def is_valid_det(frame, timeout_sec):
    """det_age_sec 로 freshness 재계산. age<0(미수신)이면 invalid."""
    age = frame["det_age"]
    return age >= 0.0 and age <= timeout_sec


# ============================================================
# Feature / target 구성
# ============================================================
def normalize_joints(angles):
    out = np.zeros(N_CTRL, dtype=np.float32)
    for k, i in enumerate(CTRL_IDX):
        lo, hi = JOINT_LIMITS[i]
        span = max(hi - lo, 1e-6)
        out[k] = float(np.clip(2.0 * (float(angles[i]) - lo) / span - 1.0, -1.0, 1.0))
    return out


def scale_delta(delta, max_delta_deg):
    out = np.zeros(N_CTRL, dtype=np.float32)
    for k, i in enumerate(CTRL_IDX):
        out[k] = float(np.clip(float(delta[i]) / max_delta_deg, -1.0, 1.0))
    return out


def visual_error(frame, target_cx, target_cy):
    """[e_u, e_v, area_norm]. det_cx/cy 는 recorder 가 저장한 live-or-frozen 값."""
    e_u = (frame["det_cx"] - target_cx) / max(frame["iw"], 1.0)
    e_v = (frame["det_cy"] - target_cy) / max(frame["ih"], 1.0)
    return np.array([e_u, e_v, frame["det_area"]], dtype=np.float32)


def build_input(frame, target_cx, target_cy, include_area):
    vis = visual_error(frame, target_cx, target_cy)
    if not include_area:
        vis = vis[:2]
    return np.concatenate([vis, normalize_joints(frame["joints"])]).astype(np.float32)


def find_q_final(frames):
    """grip_triggered 샘플의 관절각(grip 자세). 없으면 마지막 프레임."""
    q_final = None
    for f in frames:
        if f["grip_triggered"]:
            q_final = f["joints"]
    if q_final is None:
        q_final = frames[-1]["joints"]
    return q_final


def last_visible_frame(frames, timeout_sec):
    """grip 직전(또는 마지막) 유효 detection 프레임 index. = open-loop 전환점."""
    grip_i = None
    for i, f in enumerate(frames):
        if f["grip_triggered"]:
            grip_i = i
            break
    upto = grip_i if grip_i is not None else len(frames) - 1
    last = None
    for i in range(upto + 1):
        if is_valid_det(frames[i], timeout_sec):
            last = i
    return last


def goal_label(q, q_final, max_delta_deg):
    vec = (q_final - q).astype(np.float32)
    m = max(abs(float(vec[i])) for i in CTRL_IDX)
    if m > max_delta_deg:
        vec = vec * (max_delta_deg / m)
    return scale_delta(vec, max_delta_deg)


# ============================================================
# 모델
# ============================================================
class PolicyNet(nn.Module):
    def __init__(self, input_dim, n_ctrl):
        super().__init__()
        self.backbone = nn.Sequential(
            nn.Linear(input_dim, 256), nn.BatchNorm1d(256), nn.ReLU(), nn.Dropout(0.2),
            nn.Linear(256, 256), nn.BatchNorm1d(256), nn.ReLU(), nn.Dropout(0.2),
            nn.Linear(256, 128), nn.BatchNorm1d(128), nn.ReLU(),
        )
        self.head_angles = nn.Linear(128, n_ctrl)

    def forward(self, x):
        return torch.tanh(self.head_angles(self.backbone(x)))


class GripSuccessNet(nn.Module):
    def __init__(self, input_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, 128), nn.ReLU(), nn.Dropout(0.3),
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


def train_policy(X, Y, input_dim, args, device):
    n = len(X)
    if n == 0:
        print("[policy] 학습 샘플이 없습니다. 건너뜀.")
        return None

    X = torch.tensor(np.stack(X), dtype=torch.float32)
    Y = torch.tensor(np.stack(Y), dtype=torch.float32)

    gen = torch.Generator().manual_seed(args.seed)
    train_idx, val_idx = split_indices(n, args.val_ratio, gen)

    ds_train = TensorDataset(X[train_idx], Y[train_idx])
    loader = DataLoader(ds_train, batch_size=min(args.batch_size, len(ds_train)),
                        shuffle=True, drop_last=len(ds_train) >= args.batch_size)

    model = PolicyNet(input_dim, N_CTRL).to(device)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    mse = nn.MSELoss()

    print(f"[policy] samples={n} (train={len(train_idx)}, val={len(val_idx)}), "
          f"input_dim={input_dim}, weight_decay={args.weight_decay}, patience={args.patience}")

    has_val = len(val_idx) > 0
    Xv = X[val_idx].to(device) if has_val else None
    Yv = Y[val_idx].to(device) if has_val else None

    best_val, best_state, best_epoch, no_improve = float("inf"), None, -1, 0
    print_every = max(1, args.epochs // 20)

    for epoch in range(1, args.epochs + 1):
        model.train()
        tot, nb = 0.0, 0
        for xb, yb in loader:
            xb, yb = xb.to(device), yb.to(device)
            opt.zero_grad()
            loss = mse(model(xb), yb)
            loss.backward()
            opt.step()
            tot += float(loss.item())
            nb += 1

        vloss = None
        if has_val:
            model.eval()
            with torch.no_grad():
                vloss = float(mse(model(Xv), Yv))
            if vloss < best_val - 1e-6:
                best_val, best_epoch = vloss, epoch
                best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}
                no_improve = 0
            else:
                no_improve += 1

        if epoch % print_every == 0 or epoch == 1:
            msg = f"[policy] epoch {epoch:5d}  train_loss={tot/max(nb,1):.5f}"
            if vloss is not None:
                msg += f"  val_loss={vloss:.5f}  best={best_val:.5f}@{best_epoch}"
            print(msg)

        if args.patience > 0 and has_val and no_improve >= args.patience:
            print(f"[policy] early stop at epoch {epoch} "
                  f"(no val improvement for {args.patience}). best={best_val:.5f}@{best_epoch}")
            break

    if best_state is not None:
        model.load_state_dict(best_state)
        print(f"[policy] restored best: val_loss={best_val:.5f}@{best_epoch}")

    model.eval()
    with torch.no_grad():
        pd_deg = (model(X.to(device)) * args.max_delta_deg).cpu().numpy()
    print(f"[policy] pred delta(deg) mean|.|={np.mean(np.abs(pd_deg)):.3f}, "
          f"max|.|={np.max(np.abs(pd_deg)):.3f}")
    return model


def train_grip_predictor(Xg, Yg, input_dim, args, device):
    n = len(Xg)
    if n == 0:
        print("[grip] 학습 샘플이 없습니다. 건너뜀.")
        return None

    X = torch.tensor(np.stack(Xg), dtype=torch.float32)
    Y = torch.tensor(np.array(Yg, dtype=np.float32).reshape(-1, 1))

    n_pos = float(Y.sum().item())
    n_neg = float(n - n_pos)
    pos_weight = torch.tensor(
        [n_neg / n_pos if n_pos > 0 else 1.0], dtype=torch.float32).to(device)

    gen = torch.Generator().manual_seed(args.seed)
    train_idx, val_idx = split_indices(n, args.val_ratio, gen)

    ds_train = TensorDataset(X[train_idx], Y[train_idx])
    loader = DataLoader(ds_train, batch_size=min(args.batch_size, len(ds_train)), shuffle=True)

    model = GripSuccessNet(input_dim).to(device)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    crit = nn.BCEWithLogitsLoss(pos_weight=pos_weight)

    print(f"[grip] samples={n} (success={int(n_pos)}, fail={int(n_neg)}), "
          f"pos_weight={float(pos_weight):.3f}, train={len(train_idx)}, val={len(val_idx)}")

    for epoch in range(1, args.epochs + 1):
        model.train()
        for xb, yb in loader:
            xb, yb = xb.to(device), yb.to(device)
            opt.zero_grad()
            loss = crit(model(xb), yb)
            loss.backward()
            opt.step()

    model.eval()
    with torch.no_grad():
        p = torch.sigmoid(model(X.to(device))).cpu().numpy().reshape(-1)
    for i in range(n):
        print(f"[grip] sample {i}: label={int(Yg[i])} P(success)={p[i]:.3f}")
    return model


# ============================================================
# 저장 / ONNX export
# ============================================================
def export_onnx(model, path, input_dim, device, is_policy):
    try:
        dummy = torch.zeros(1, input_dim, dtype=torch.float32, device=device)
        model.eval()
        torch.onnx.export(
            model, dummy, str(path),
            input_names=["state"],
            output_names=(["delta_norm"] if is_policy else ["success_logit"]),
            dynamic_axes={"state": {0: "batch"}},
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
            if ep.is_dir() and (ep / "human").exists():
                out.append((sub, ep))
    return out


def main():
    parser = argparse.ArgumentParser(description="Closed-loop NN controller 학습")
    parser.add_argument("--data-dir", default="~/rosbags_closeloop")
    parser.add_argument("--out-dir",
                        default="src/just_pick_it/just_pick_it_perception/result/nn_controller/pick")
    parser.add_argument("--det-valid-timeout", type=float, default=0.3,
                        help="det_age_sec 가 이 값(sec) 이하이면 유효 detection 으로 본다. "
                             "녹화 시점 박제가 아니라 det_age_sec 로 재계산하므로 자유롭게 조정.")
    parser.add_argument("--include-area", choices=["true", "false"], default="true",
                        help="입력에 area_norm 포함 여부. false 면 입력 7차원.")
    parser.add_argument("--target-cx", type=float, default=-1.0,
                        help="off-center 그립 목표점 cx(px). 음수면 데이터에서 자동 추출.")
    parser.add_argument("--target-cy", type=float, default=-1.0,
                        help="off-center 그립 목표점 cy(px). 음수면 데이터에서 자동 추출.")
    parser.add_argument("--max-delta-deg", type=float, default=5.0)
    parser.add_argument("--epochs", type=int, default=3000)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--weight-decay", type=float, default=1e-3)
    parser.add_argument("--patience", type=int, default=800)
    parser.add_argument("--val-ratio", type=float, default=0.15)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--skip-onnx", action="store_true")
    args = parser.parse_args()

    include_area = (args.include_area == "true")
    input_dim = (3 if include_area else 2) + N_CTRL

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    data_dir = Path(args.data_dir).expanduser()
    out_dir = Path(args.out_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    device = torch.device(args.device)

    episodes = collect_episode_dirs(data_dir)
    if not episodes:
        raise SystemExit(f"에피소드를 찾지 못했습니다: {data_dir}/{{success,fail}}/*/human")

    # --- Pass 1: 에피소드 로드 + last-visible 추출(목표점 산출용) ---
    loaded = []   # (label, frames, q_final, last_vis_idx)
    last_vis_cx, last_vis_cy = [], []
    blind_tails = []
    skipped = 0
    for label, ep in episodes:
        frames = read_human_episode(ep / "human")
        if frames is None:
            print(f"  [skip] {label}/{ep.name}: det 필드 없음 또는 빈 bag(재수집 필요)")
            skipped += 1
            continue
        q_final = find_q_final(frames)
        lv = last_visible_frame(frames, args.det_valid_timeout)
        loaded.append((label, frames, q_final, lv))
        if label == "success" and lv is not None:
            last_vis_cx.append(frames[lv]["det_cx"])
            last_vis_cy.append(frames[lv]["det_cy"])
            # blind tail = grip 까지 lv 이후 프레임 수.
            grip_i = next((i for i, f in enumerate(frames) if f["grip_triggered"]),
                          len(frames) - 1)
            blind_tails.append(max(0, grip_i - lv))

    if not loaded:
        raise SystemExit("유효 에피소드가 없습니다(전부 구버전 bag).")

    # off-center 그립 목표점.
    if args.target_cx >= 0.0 and args.target_cy >= 0.0:
        target_cx, target_cy = args.target_cx, args.target_cy
        print(f"target(cx,cy) = ({target_cx:.1f}, {target_cy:.1f}) [수동 지정]")
    elif last_vis_cx:
        target_cx = float(np.median(last_vis_cx))
        target_cy = float(np.median(last_vis_cy))
        print(f"target(cx,cy) = ({target_cx:.1f}, {target_cy:.1f}) "
              f"[success {len(last_vis_cx)}개 last-visible median]")
    else:
        target_cx, target_cy = DEFAULT_IMAGE_W * 0.5, DEFAULT_IMAGE_H * 0.5
        print(f"target(cx,cy) = ({target_cx:.1f}, {target_cy:.1f}) [fallback: 화면중앙]")

    if blind_tails:
        bt = np.array(blind_tails)
        print(f"blind tail 프레임 수: mean={bt.mean():.1f} min={bt.min()} max={bt.max()} "
              f"-> open_loop_lost_frames 권장 N≈{max(2, int(np.median(bt)//2))}")

    # --- Pass 2: 샘플 생성 ---
    pX, pY = [], []          # policy (success only, to_goal)
    gX, gY = [], []          # grip predictor (success + fail), last-visible 1개/에피소드
    n_open_frames = 0
    for label, frames, q_final, lv in loaded:
        if label == "success":
            seen_valid = False
            for f in frames:
                valid = is_valid_det(f, args.det_valid_timeout)
                # 첫 유효 detection 이전(시각 컨텍스트 없음)은 제외.
                if valid:
                    seen_valid = True
                if not seen_valid:
                    continue
                if not valid:
                    n_open_frames += 1
                pX.append(build_input(f, target_cx, target_cy, include_area))
                pY.append(goal_label(f["joints"], q_final, args.max_delta_deg))

        # grip predictor: last-visible 프레임 state.
        if lv is not None:
            gX.append(build_input(frames[lv], target_cx, target_cy, include_area))
            gY.append(1.0 if label == "success" else 0.0)

    print(f"\n에피소드: {len(loaded)} (success="
          f"{sum(1 for l,_,_,_ in loaded if l=='success')}, "
          f"fail={sum(1 for l,_,_,_ in loaded if l=='fail')}), skipped={skipped}")
    print(f"policy 샘플={len(pX)} (그중 open-loop frozen frame={n_open_frames}), "
          f"grip 샘플={len(gX)}")

    # --- config 저장 (추론 노드가 재사용) ---
    config = {
        "model_kind": "closeloop",
        "input_dim": input_dim,
        "include_area": include_area,
        "input_layout": (["e_u", "e_v"] + (["area_norm"] if include_area else [])
                         + ["q1n", "q2n", "q3n", "q4n", "q5n"]),
        "target_cx": target_cx,
        "target_cy": target_cy,
        "det_valid_timeout": args.det_valid_timeout,
        "max_delta_deg": args.max_delta_deg,
        "joint_limits": JOINT_LIMITS,
        "controlled_joints": CTRL_IDX,
        "default_image_w": DEFAULT_IMAGE_W,
        "default_image_h": DEFAULT_IMAGE_H,
        # 추론 핸드오프 권장값(데이터의 jitter 0~2 vs blind tail 4~8 기준).
        "open_loop_lost_frames": 3,
        "policy_output": ["dq1..dq5 (tanh, x max_delta_deg). J6 excluded."],
        "grip_decision": "GripSuccessPredictor at last-visible frame (open-loop 전환점)",
    }
    with open(out_dir / "nn_controller_config.json", "w") as f:
        json.dump(config, f, indent=2)
    print(f"config 저장: {out_dir / 'nn_controller_config.json'}")

    # --- Policy ---
    print("\n=== Policy network ===")
    policy = train_policy(pX, pY, input_dim, args, device)
    if policy is not None:
        pt = out_dir / "nn_controller_policy.pt"
        torch.save({"state_dict": policy.state_dict(), "config": config}, pt)
        print(f"  저장: {pt}")
        if not args.skip_onnx:
            export_onnx(policy, out_dir / "nn_controller_policy.onnx", input_dim, device, True)

    # --- Grip predictor ---
    print("\n=== Grip success predictor ===")
    grip = train_grip_predictor(gX, gY, input_dim, args, device)
    if grip is not None:
        pt = out_dir / "grip_success_predictor.pt"
        torch.save({"state_dict": grip.state_dict(), "config": config}, pt)
        print(f"  저장: {pt}")
        if not args.skip_onnx:
            export_onnx(grip, out_dir / "grip_success_predictor.onnx", input_dim, device, False)

    print("\n완료.")


if __name__ == "__main__":
    main()
