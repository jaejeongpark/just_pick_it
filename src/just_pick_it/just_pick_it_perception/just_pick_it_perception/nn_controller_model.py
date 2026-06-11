#!/usr/bin/env python3

"""
NN Controller 추론용 모델/전처리 모듈 (Task 8 추론 노드가 사용).

train_nn_controller.py 의 PolicyNet / GripSuccessNet 과 아키텍처가 정확히 동일해야
state_dict 로드가 가능하다. submodule 이름(backbone, head_angles, net)도 동일하게 유지한다.

전처리(정규화/스케일/feature layout)도 학습과 동일해야 train/inference 일관성이 유지된다.
모든 차원/정규화 상수는 학습 시 저장한 nn_controller_config.json 에서 읽는다.
"""

import json
from pathlib import Path

import numpy as np

import torch
import torch.nn as nn


N_JOINTS = 6


class PolicyNet(nn.Module):
    """train_nn_controller.PolicyNet 과 동일. 제어 관절 delta_norm(n_ctrl) 출력."""

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
    """train_nn_controller.GripSuccessNet 과 동일. success logit(1) 출력."""

    def __init__(self, input_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, 128), nn.ReLU(), nn.Dropout(0.3),
            nn.Linear(128, 64), nn.ReLU(), nn.Dropout(0.3),
            nn.Linear(64, 1),
        )

    def forward(self, x):
        return self.net(x)


class FeatureBuilder:
    """학습과 동일한 정규화/feature layout으로 입력 벡터를 구성한다."""

    def __init__(self, config: dict):
        self.window = int(config["window"])
        self.anchor_dim = int(config["anchor_dim"])
        self.features_per_step = int(config["features_per_step"])
        self.input_dim = int(config["input_dim"])
        self.max_delta_deg = float(config["max_delta_deg"])
        self.joint_limits = [tuple(v) for v in config["joint_limits"]]
        # NN이 제어하는 관절 (J1~J5). J6는 ibvs_controller가 결정론적으로 정렬·고정.
        self.controlled_joints = list(config.get("controlled_joints", [0, 1, 2, 3, 4]))
        self.n_ctrl = len(self.controlled_joints)
        self.image_w = float(config.get("default_image_w", 640.0))
        self.image_h = float(config.get("default_image_h", 480.0))

    def normalize_joints(self, angles):
        out = np.zeros(self.n_ctrl, dtype=np.float32)
        for k, i in enumerate(self.controlled_joints):
            lo, hi = self.joint_limits[i]
            span = max(hi - lo, 1e-6)
            out[k] = float(np.clip(2.0 * (float(angles[i]) - lo) / span - 1.0, -1.0, 1.0))
        return out

    def scale_delta(self, delta):
        out = np.zeros(self.n_ctrl, dtype=np.float32)
        for k, i in enumerate(self.controlled_joints):
            out[k] = float(np.clip(float(delta[i]) / self.max_delta_deg, -1.0, 1.0))
        return out

    def step_feat(self, joints, delta):
        return np.concatenate([
            self.normalize_joints(joints),
            self.scale_delta(delta),
        ]).astype(np.float32)

    def anchor_vec(self, cx, cy, area_norm, image_w=None, image_h=None):
        iw = float(image_w) if image_w else self.image_w
        ih = float(image_h) if image_h else self.image_h
        return np.array([
            float(cx) / iw,
            float(cy) / ih,
            float(area_norm),
        ], dtype=np.float32)

    def build_input(self, anchor_vec, window_step_feats):
        """입력 = [anchor(5)] + [step_feat(12) × window]. window는 oldest→newest 순."""
        return np.concatenate([anchor_vec] + list(window_step_feats)).astype(np.float32)


def load_config(model_dir):
    path = Path(model_dir).expanduser() / "nn_controller_config.json"
    with open(path) as f:
        return json.load(f)


def load_policy(model_dir, config, device="cpu"):
    n_ctrl = len(config.get("controlled_joints", [0, 1, 2, 3, 4]))
    model = PolicyNet(int(config["input_dim"]), n_ctrl)
    ckpt = torch.load(
        str(Path(model_dir).expanduser() / "nn_controller_policy.pt"),
        map_location=device,
    )
    model.load_state_dict(ckpt["state_dict"])
    model.to(device).eval()
    return model


def load_grip_predictor(model_dir, config, device="cpu"):
    model = GripSuccessNet(int(config["input_dim"]))
    ckpt = torch.load(
        str(Path(model_dir).expanduser() / "grip_success_predictor.pt"),
        map_location=device,
    )
    model.load_state_dict(ckpt["state_dict"])
    model.to(device).eval()
    return model
