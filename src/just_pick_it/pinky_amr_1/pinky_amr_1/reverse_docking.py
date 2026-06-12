#!/usr/bin/env python3
"""
Reverse Docking - 마커 깊이 + 노란 주차라인 정밀 정렬 후진 도킹

설계 (좁은 도크 정밀 도킹용):
  역할 분담
    - ArUco 마커(상시 가시): 깊이(정지 시점) + 시작 coarse 정렬 + 도킹 후 /initialpose 보정
    - 노란 주차라인:        lateral + yaw 정밀 정렬. 두 라인의 대칭선(채널 중심)을
                            이미지 중심·수직(법선)으로 PID. 후진할수록 더 잘 보인다.
    - 라인 검출 신뢰도(conf)로 "마커 coarse → 라인 정밀" 블렌딩.
      시작(라인 일부만 보임)=마커 위주, 삽입(라인 충분)=라인 위주. ω 는 항상 한 값으로
      합쳐 매끄럽게 움직인다(lateral+yaw 합산).
    - 정지: 마커까지 거리로 추정한 로봇 world y 가 dock_y 에 도달하면.

  좌표/부호 전제 (실차 브링업에서 검증·튜닝 대상):
    - 로봇은 헤드(+x_body, 전방 카메라)가 +y_world(마커 쪽)를 보며 -y 로 후진해 도크에 들어간다.
    - solvePnP tvec: 카메라(optical) 기준. tvec[0]>0 = 마커가 우측, tvec[2] = 카메라→마커 거리.
    - 후진 시 조향 부호는 전진과 반대(마커가 우측이면 좌회전).

마커 월드 좌표는 state_manager 가 아니라 이 노드 설정(marker_id 별)에 둔다.
reverse_dock(marker_id, dock_map_x, dock_map_y, dock_map_yaw) 를 state_manager 가 호출.
"""

import math
import os
import time
import threading

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image


class PID:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp, self.ki, self.kd = kp, ki, kd
        self._integral = 0.0
        self._prev_err = 0.0
        self._prev_t = time.time()

    def reset(self):
        self._integral = 0.0
        self._prev_err = 0.0
        self._prev_t = time.time()

    def compute(self, err: float) -> float:
        now = time.time()
        dt = max(now - self._prev_t, 1e-4)
        self._integral += err * dt
        d = (err - self._prev_err) / dt
        out = self.kp * err + self.ki * self._integral + self.kd * d
        self._prev_err = err
        self._prev_t = now
        return out


class LaneResult:
    """노란 주차라인 검출 결과.

    lateral_px : 채널 중심선의 이미지 중심 대비 수평 오프셋(px). +면 중심이 우측.
    yaw_rad    : 채널 중심선의 수직 대비 기울기(rad). +면 위로 갈수록 우측으로 기욺.
    conf       : 검출 신뢰도 0..1 (양쪽 라인 존재 + 픽셀 수 기반).
    """
    __slots__ = ("lateral_px", "yaw_rad", "conf")

    def __init__(self, lateral_px: float, yaw_rad: float, conf: float):
        self.lateral_px = lateral_px
        self.yaw_rad = yaw_rad
        self.conf = conf


class ReverseDocking(Node):
    def __init__(self, emergency_latch=None):
        super().__init__("reverse_docking")

        # 비상 정지 래치. state_manager 가 공유 인스턴스를 주입한다.
        if emergency_latch is None:
            from pinky_amr_1.emergency_latch import EmergencyLatch
            emergency_latch = EmergencyLatch()
        self._emergency = emergency_latch

        # ── ArUco ────────────────────────────────────────────────────────
        # cv2.aruco dictionary 이름. 팀 perception 표준은 AprilTag 36h11.
        self.declare_parameter("aruco_marker_dict", "DICT_APRILTAG_36h11")
        self.declare_parameter("marker_size_m", 0.05)

        # 마커 월드 좌표 (marker_id 별 병렬 배열). 가로벽에 도크를 바라보게 부착.
        # 법선은 -y(원점 쪽) 고정이라 yaw 따로 안 받는다.
        self.declare_parameter("marker_ids", [0, 1])
        self.declare_parameter("marker_world_x", [0.11, 0.28])
        self.declare_parameter("marker_world_y", [0.655, 0.655])

        # ── 카메라 (비전팀 캘리브레이션 실측값, 1280x720) ────────────────
        self.declare_parameter("camera_matrix", [
            777.0, 0.0, 960.0,
            0.0, 777.0, 540.0,
            0.0, 0.0, 1.0,
        ])
        self.declare_parameter("dist_coeffs", [0.0, 0.0, 0.0, 0.0, 0.0])
        # ROS camera_info yaml 경로. 지정되면 camera_matrix/dist_coeffs 대신 이 파일에서
        # 직접 읽는다(캘리브레이션 단일 출처). package:// URI 도 지원(install share 로 해석).
        # 비어 있거나 읽기 실패면 위 camera_matrix/dist_coeffs 파라미터를 fallback 으로 쓴다.
        self.declare_parameter(
            "calibration_yaml",
            "package://just_pick_it_perception/result/camera_calibration.yaml",
        )
        # 카메라 소스: 'picamera2'(보드 직접, 기본) 또는 'ros_topic'(sim/테스트 폴백).
        # 직접 모드는 도킹 중에만 카메라를 열어 ROS Image pub/sub 오버헤드를 없앤다.
        self.declare_parameter("camera_source", "picamera2")
        self.declare_parameter("camera_width", 1280)
        self.declare_parameter("camera_height", 720)
        # 카메라가 물리적으로 거꾸로(180°) 장착됨 + 캘리브레이션도 flip 된 이미지 기준
        # (just_pick_it_perception/apriltag_detector_real 와 동일). 검출 전 cv2.flip(-1) 필요.
        self.declare_parameter("flip_camera_180", True)
        # base_link(중심) 에서 카메라가 전방(+x_body=+y_world)으로 떨어진 거리(m).
        # 마커 거리로 로봇 base 의 world y 를 추정할 때 보정에 쓴다(URDF 기준 근사).
        self.declare_parameter("camera_forward_offset_m", 0.05)
        # solvePnP tvec 이 실측보다 짧게 나옴(캘리브/검출 규약). 그런데 그 비율이 거리에
        # 따라 달라(가까움 ~1.48, 도크 원거리 ~1.36) 깊이정지(원거리)와 횡측정(가까움)을
        # 분리한다. depth_scale: robot_y(도크 정지)용, lateral_scale: Δx(횡 측정)용.
        self.declare_parameter("depth_scale", 1.36)
        self.declare_parameter("lateral_scale", 1.48)
        # 마커 좌표 횡 바이어스 보정(m). 정렬 후 x 가 일정하게 +쪽(동)으로 남으면 +값으로
        # Δx 를 키워 더 서쪽으로 보낸다(실측 final_x - 0.11 만큼).
        self.declare_parameter("marker_lat_offset_m", 0.0)
        # 마커 정면 정렬 시 rvec[1]≈π 인데, 마커 장착 미세 기울기 등으로 도크-정렬 헤딩과
        # 몇 도 어긋날 수 있다. yaw 목표를 이만큼 보정(도). +면 더 회전(rvec[1] 목표를 +방향).
        self.declare_parameter("marker_yaw_offset_deg", 0.0)

        # ── 시작 coarse 정렬 (마커 상대) ────────────────────────────────
        self.declare_parameter("acquire_rotate_speed", 0.3)   # 마커 탐색 회전(rad/s)
        self.declare_parameter("marker_lat_kp", 1.0)          # tvec[0] coarse 횡 게인
        self.declare_parameter("marker_yaw_kp", 0.8)          # rvec[1] coarse yaw 게인
        # 채널 안에서 마커 상실 시: 후진을 멈추고 제자리 회전으로 마커를 카메라 중앙에
        # 되돌리는 재중심 단계. kp=중심 정렬 회전 게인, tol=중앙 허용 bearing(rad).
        self.declare_parameter("recenter_kp", 0.8)
        self.declare_parameter("recenter_tol_rad", 0.05)
        self.declare_parameter("recenter_timeout_sec", 6.0)
        # 시퀀스1 = 마커로 법선 횡오차 1회 측정 → 부드러운 후진 arc 로 진입.
        #   arc_omega: arc 회전율(rad/s, 작을수록 큰 반경=회전 적고 후진 많음)
        #   arc_lat_tol_m: odom 횡이동 종료 허용오차, arc_timeout_sec: arc 최대 시간
        #   measure_frames: 법선 횡오차 1회 측정 평균 프레임수
        self.declare_parameter("arc_omega", 0.25)
        self.declare_parameter("arc_lat_tol_m", 0.005)
        self.declare_parameter("arc_timeout_sec", 20.0)
        self.declare_parameter("measure_frames", 10)
        # 측정 Δx 가 이보다 작으면 이미 정렬된 것으로 보고 arc 생략·반복 종료(노이즈/phantom
        # 추종 방지). rvec 잔여 yaw 로 인한 측정 phantom 바닥(~2~3cm)보다 약간 크게.
        self.declare_parameter("align_conv_tol_m", 0.03)
        # 정렬 보정 게인: 1차는 first_pass_gain, 2차+ 는 arc_refine_gain 비율만 이동
        # (과보정으로 법선 지나침 방지).
        self.declare_parameter("first_pass_gain", 0.8)
        self.declare_parameter("arc_refine_gain", 0.3)
        # arc 1회 최대 횡이동(m). phantom 으로 큰 Δx 가 나와도 벽으로 돌진 못하게 캡.
        self.declare_parameter("arc_max_travel_m", 0.06)
        # 시퀀스2 라인검출 횡조향 사용 여부. 기본 off → arc+recenter 정렬 믿고 직진 후진,
        # 마커는 깊이 정지에만 사용. 라인검출이 이 거리에서 불안정해 끄는 것이 안전.
        self.declare_parameter("use_lane_steering", False)
        # [측정→arc→재정렬] 반복 횟수(잔여 횡오차 수렴).
        self.declare_parameter("align_passes", 2)
        # yaw 정렬 완료 허용오차(rad). 마커 fronto-parallel(법선 정면)까지 제자리 회전.
        self.declare_parameter("yaw_align_tol_rad", 0.04)   # ~2.3deg
        # 후진 중 odom yaw 헤딩 유지 P게인. θ_ref(법선)에서 벗어나면 되돌린다.
        self.declare_parameter("yaw_hold_kp", 1.0)

        # ── 정밀 정렬 (노란 라인) ───────────────────────────────────────
        self.declare_parameter("lane_lat_kp", 0.004)          # lateral_px 게인
        self.declare_parameter("lane_yaw_kp", 0.8)            # yaw_rad 게인
        self.declare_parameter("lane_yellow_lower", [20, 100, 100])  # HSV
        self.declare_parameter("lane_yellow_upper", [35, 255, 255])
        self.declare_parameter("lane_min_pixels", 150)        # 한쪽 라인 최소 픽셀(conf 0)
        self.declare_parameter("lane_full_pixels", 1200)      # 이 이상이면 conf 1
        # 노란선이 3줄(왼|중앙(공유)|오)이라 컬럼 히스토그램으로 라인을 분리한 뒤
        # dock1=왼쪽 2줄, dock2=오른쪽 2줄을 채널로 쓴다. 컬럼이 라인인지 판정하는 임계
        # (ROI 세로 대비 그 컬럼의 노란픽셀 비율).
        self.declare_parameter("lane_col_active_ratio", 0.25)

        # ── 후진/정지 ───────────────────────────────────────────────────
        self.declare_parameter("reverse_speed", 0.04)         # 후진 속도(m/s)
        self.declare_parameter("stop_y_tolerance_m", 0.01)    # dock_y 도달 허용오차
        self.declare_parameter("settle_sec", 0.5)             # 정지 후 안정화 대기

        # ── 공통 ────────────────────────────────────────────────────────
        self.declare_parameter("max_angular_vel", 0.4)
        self.declare_parameter("acquire_timeout_sec", 30.0)
        self.declare_parameter("insert_timeout_sec", 40.0)
        self.declare_parameter("camera_topic", "camera/image_raw")

        # ── 파라미터 로드 ───────────────────────────────────────────────
        # dictionary 이름(예: DICT_APRILTAG_36h11) 또는 정수 enum 둘 다 허용.
        _dict_param = self.get_parameter("aruco_marker_dict").value
        dict_id = (
            getattr(cv2.aruco, _dict_param)
            if isinstance(_dict_param, str) else int(_dict_param)
        )
        self._marker_size = self.get_parameter("marker_size_m").value

        ids = list(self.get_parameter("marker_ids").value)
        mwx = list(self.get_parameter("marker_world_x").value)
        mwy = list(self.get_parameter("marker_world_y").value)
        # marker_id -> (world_x, world_y)
        self._marker_world = {int(i): (float(x), float(y))
                              for i, x, y in zip(ids, mwx, mwy)}

        # 캘리브레이션: yaml 경로가 주어지면 거기서 직접 읽고, 아니면 파라미터 fallback.
        calib = self._load_calibration(self.get_parameter("calibration_yaml").value)
        if calib is not None:
            self._cam_matrix, self._dist_coeffs = calib
        else:
            self._cam_matrix = np.array(
                self.get_parameter("camera_matrix").value, dtype=np.float64
            ).reshape(3, 3)
            self._dist_coeffs = np.array(
                self.get_parameter("dist_coeffs").value, dtype=np.float64
            )
        self._cam_fwd = self.get_parameter("camera_forward_offset_m").value
        self._depth_scale = self.get_parameter("depth_scale").value
        self._lateral_scale = self.get_parameter("lateral_scale").value
        self._marker_lat_offset = self.get_parameter("marker_lat_offset_m").value
        self._marker_yaw_offset = math.radians(self.get_parameter("marker_yaw_offset_deg").value)

        self._camera_source = self.get_parameter("camera_source").value
        self._cam_w = int(self.get_parameter("camera_width").value)
        self._cam_h = int(self.get_parameter("camera_height").value)
        self._flip_180 = bool(self.get_parameter("flip_camera_180").value)

        self._acquire_rot   = self.get_parameter("acquire_rotate_speed").value
        self._marker_lat_kp = self.get_parameter("marker_lat_kp").value
        self._marker_yaw_kp = self.get_parameter("marker_yaw_kp").value
        self._recenter_kp  = self.get_parameter("recenter_kp").value
        self._recenter_tol = self.get_parameter("recenter_tol_rad").value
        self._recenter_to  = self.get_parameter("recenter_timeout_sec").value
        self._arc_omega     = self.get_parameter("arc_omega").value
        self._arc_lat_tol   = self.get_parameter("arc_lat_tol_m").value
        self._arc_to        = self.get_parameter("arc_timeout_sec").value
        self._measure_frames = int(self.get_parameter("measure_frames").value)
        self._use_lane      = bool(self.get_parameter("use_lane_steering").value)
        self._align_passes  = int(self.get_parameter("align_passes").value)
        self._align_conv_tol = self.get_parameter("align_conv_tol_m").value
        self._first_pass_gain = self.get_parameter("first_pass_gain").value
        self._arc_refine_gain = self.get_parameter("arc_refine_gain").value
        self._arc_max_travel = self.get_parameter("arc_max_travel_m").value
        self._yaw_align_tol = self.get_parameter("yaw_align_tol_rad").value
        self._yaw_hold_kp   = self.get_parameter("yaw_hold_kp").value

        self._lane_lat_kp = self.get_parameter("lane_lat_kp").value
        self._lane_yaw_kp = self.get_parameter("lane_yaw_kp").value
        self._yellow_lo = tuple(int(v) for v in self.get_parameter("lane_yellow_lower").value)
        self._yellow_hi = tuple(int(v) for v in self.get_parameter("lane_yellow_upper").value)
        self._lane_min_px  = int(self.get_parameter("lane_min_pixels").value)
        self._lane_full_px = int(self.get_parameter("lane_full_pixels").value)
        self._lane_col_ratio = self.get_parameter("lane_col_active_ratio").value
        self._dock_left = True       # reverse_dock 에서 marker_id 로 설정(dock1=좌/dock2=우)
        self._dbg_line_xs: list[int] = []   # 디버그: 마지막 검출 라인 x 들

        self._reverse_speed = self.get_parameter("reverse_speed").value
        self._stop_tol      = self.get_parameter("stop_y_tolerance_m").value
        self._settle        = self.get_parameter("settle_sec").value

        self._max_ang        = self.get_parameter("max_angular_vel").value
        self._acquire_to     = self.get_parameter("acquire_timeout_sec").value
        self._insert_to      = self.get_parameter("insert_timeout_sec").value

        # ── ArUco 검출기 / solvePnP 기준점 ──────────────────────────────
        aruco_dict   = cv2.aruco.getPredefinedDictionary(dict_id)
        aruco_params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        h = self._marker_size / 2.0
        # IPPE_SQUARE 규약 순서(좌상,우상,우하,좌하)에 맞춘 마커 평면 3D 점.
        self._marker_obj_pts = np.array([
            [-h,  h, 0.0],
            [ h,  h, 0.0],
            [ h, -h, 0.0],
            [-h, -h, 0.0],
        ], dtype=np.float64)
        # ±π 평면 모호성 해소용: 직전에 채택한 psi(시간연속성으로 8° 튐 거름). 도킹 시작 시 리셋.
        self._last_psi = None

        # ── PID (정밀 정렬용 라인 lateral/yaw) ──────────────────────────
        self._lane_lat_pid = PID(self._lane_lat_kp, 0.0, 0.0005)
        self._lane_yaw_pid = PID(self._lane_yaw_kp, 0.0, 0.05)

        # ── 카메라/퍼블리셔 ─────────────────────────────────────────────
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_frame = None
        self._picam2 = None  # picamera2 모드: reverse_dock() 시작 시 lazy open

        # 오도메트리: open-loop arc 의 상대 횡이동량 측정용(전역 누적오차와 무관, 상대만 사용).
        self._odom_lock = threading.Lock()
        self._odom = None    # (x, y, yaw)
        self.create_subscription(Odometry, "odom", self._odom_cb, 10)

        # picamera2(기본)는 도킹 중에만 직접 연다. ros_topic 폴백만 토픽을 구독한다.
        if self._camera_source == "ros_topic":
            cam_topic = self.get_parameter("camera_topic").value
            self.create_subscription(Image, cam_topic, self._image_cb, 10)

        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "initialpose", 10
        )

        self.get_logger().info(
            f"ReverseDocking ready (camera={self._camera_source}, fx={self._cam_matrix[0, 0]:.1f})."
        )

    # ====================================================================== #
    # 외부 인터페이스
    # ====================================================================== #

    def reverse_dock(
        self,
        marker_id: int,
        dock_map_x: float,
        dock_map_y: float,
        dock_map_yaw: float,
    ) -> bool:
        """마커 깊이 + 노란 라인 정밀 정렬 후진 도킹. 성공 시 True."""
        if marker_id not in self._marker_world:
            self.get_logger().error(
                f"reverse_dock: marker_id={marker_id} 월드좌표 설정 없음"
            )
            return False
        marker_x, marker_y = self._marker_world[marker_id]
        # dock1(marker 0, world x 작음) = 왼쪽 채널(왼+중앙 라인), dock2 = 오른쪽(중앙+오).
        self._dock_left = (marker_id == 0)

        self.get_logger().info(
            f"reverse_dock: marker={marker_id}@world({marker_x:.3f},{marker_y:.3f}), "
            f"dock=({dock_map_x:.3f},{dock_map_y:.3f},"
            f"{math.degrees(dock_map_yaw):.1f}deg)"
        )
        self._lane_lat_pid.reset()
        self._lane_yaw_pid.reset()
        self._last_psi = None   # 모호성 해소 시간연속성 초기화(첫 검출은 재투영오차로)

        # picamera2 모드는 도킹 동안만 카메라를 열고, 끝나면(성공/실패 무관) 반납한다.
        self._open_camera()
        try:
            # 1) 마커 획득 (보일 때까지 탐색 회전)
            if not self._acquire_marker(marker_id):
                self._stop()
                self.get_logger().error("reverse_dock: FAILED — 마커 미획득")
                return False

            # 2) 1차 법선정렬: [recenter(마커 정면) → 측정 → arc] × N → 최종 yaw_align.
            #    측정은 반드시 '마커 정면(recenter)' 자세에서 한다 — off-line 일 때 법선 정면
            #    자세는 마커가 화각을 벗어나 검출 실패하기 때문. 마커 정면에선 tx≈0 이라 횡오차가
            #    tz·sin(psi) 로 잡힌다(measure 가 디커플링 전체식 사용). 후진은 reverse_insert
            #    가 정렬된 헤딩을 odom 으로 앵커링·유지(후진하면 마커가 화각 이탈 → odom 필수).
            move_dx = 0.0
            for p in range(self._align_passes):
                if not self._recenter_on_marker(marker_id, move_dx):
                    self._stop()
                    self.get_logger().error("reverse_dock: FAILED — 마커 재정렬")
                    return False
                dx = self._measure_lateral_offset(marker_id)
                if dx is None:
                    self._stop()
                    self.get_logger().error("reverse_dock: FAILED — 횡오차 측정 실패")
                    return False
                self.get_logger().info(
                    f"정렬 {p + 1}/{self._align_passes}차: Δx={dx:+.3f}m"
                )
                if abs(dx) < self._align_conv_tol:
                    self.get_logger().info(
                        f"정렬 수렴 (|Δx|={abs(dx):.3f} < {self._align_conv_tol}) → 반복 종료"
                    )
                    break
                gain = self._first_pass_gain if p == 0 else self._arc_refine_gain
                move_dx = dx * gain
                self.get_logger().info(
                    f"  보정: Δx {dx:+.3f} 의 {gain}배 = {move_dx:+.3f}m 이동"
                )
                if not self._arc_into_line(move_dx):
                    self._stop()
                    self.get_logger().error("reverse_dock: FAILED — arc 진입")
                    return False

            # 3) 후진 전 최종 법선 정렬(on-line 이라 법선에서도 마커 화각 안).
            #    reverse_insert 가 이 헤딩을 odom 으로 앵커링·유지.
            self._align_yaw_to_normal(marker_id)
            if not self._reverse_insert(marker_id, marker_y, dock_map_y):
                self._stop()
                self.get_logger().error("reverse_dock: FAILED — 후진 도킹")
                return False

            # 6) 정지 + 안정화 + 위치 보정
            self._stop()
            time.sleep(self._settle)
            self.get_logger().info("reverse_dock: SUCCESS")
            self._publish_pose_correction(dock_map_x, dock_map_y, dock_map_yaw)
            return True
        finally:
            self._close_camera()

    # ====================================================================== #
    # 단계 A: 마커 획득
    # ====================================================================== #

    def _acquire_marker(self, marker_id: int) -> bool:
        """마커가 검출될 때까지 제자리 탐색 회전. 검출되면 True."""
        deadline = time.time() + self._acquire_to
        last_log = 0.0
        while time.time() < deadline:
            if self._emergency.is_stopped():
                self._stop()
                deadline += self._wait_if_paused()
                continue

            frame = self._get_latest_frame()
            if frame is None:
                if time.time() - last_log > 2.0:
                    self.get_logger().warn("Acquire: 프레임 없음 (카메라 확인)")
                    last_log = time.time()
                time.sleep(0.05)
                continue

            if self._detect_aruco(frame, marker_id) is not None:
                self._stop()
                self.get_logger().info("Acquire: 마커 검출")
                return True

            # 디버그(2s): 검출된 id 목록으로 dict/id/FOV 원인을 구분한다.
            # 빈 목록=dict 불일치/FOV밖/너무 멀음, 다른 id=id 불일치.
            if time.time() - last_log > 2.0:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                _, dbg_ids, _ = self._detector.detectMarkers(gray)
                seen = [] if dbg_ids is None else dbg_ids.flatten().tolist()
                self.get_logger().info(
                    f"Acquire: 탐색 중 — 검출 id={seen}, 목표 id={marker_id}"
                )
                last_log = time.time()

            twist = Twist()
            twist.angular.z = self._acquire_rot
            self._cmd_pub.publish(twist)
            time.sleep(0.05)

        self._stop()
        self.get_logger().warn("Acquire: timeout")
        return False

    # ====================================================================== #
    # 단계 C: 후진 삽입 (라인 정밀 + 마커 깊이)
    # ====================================================================== #

    def _reverse_insert(
        self,
        marker_id: int,
        marker_world_y: float,
        dock_y: float,
        theta_ref: float = None,
    ) -> bool:
        """후진 도킹 + 마커 깊이 정지. 진입 시 로봇은 이미 법선 정렬·마커 정면 상태.

        헤딩 유지: 진입 직후 odom yaw 를 θ_ref(=법선 헤딩)로 앵커링하고, 후진 내내
        odom yaw=θ_ref 로 P제어해 헤딩을 정밀하게 고정한다(psi 노이즈 안 씀). 작은 yaw
        오차도 후진하며 x 드리프트로 누적되므로, 흔들림 없이 일정 헤딩으로 곧게 내려간다.
        라인은 use_lane_steering=true 일 때만 횡조향(기본 off). 마커 상실 시 재중심.
        정지: 로봇 world y(= marker_y - 마커거리·scale - 카메라 전방오프셋) <= dock_y
        """
        deadline = time.time() + self._insert_to
        last_tx = 0.0
        last_log = 0.0
        # 후진 헤딩(법선): 방법B 가 캡처한 θ_ref 를 받으면 그걸 유지(측정·정렬과 동일 기준).
        # 못 받으면(None) 진입 직후 odom yaw 로 앵커링(하위호환).
        if theta_ref is None:
            od0 = self._get_odom()
            theta_ref = od0[2] if od0 is not None else None
        if theta_ref is not None:
            self.get_logger().info(f"Insert: odom 헤딩 θ_ref={math.degrees(theta_ref):+.1f}deg 유지")

        while time.time() < deadline:
            if self._emergency.is_stopped():
                self._stop()
                deadline += self._wait_if_paused()
                continue

            frame = self._get_latest_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            marker = self._detect_aruco(frame, marker_id)
            if marker is None:
                # 마커가 화각을 벗어나면 후진 멈추고 제자리 회전으로 다시 중앙에 잡는다.
                self._stop()
                if self._recenter_on_marker(marker_id, last_tx):
                    continue
                self.get_logger().warn("Insert: 마커 재중심 실패 → 도킹 실패")
                return False

            tvec, rvec = marker
            marker_dist = float(tvec[2])
            last_tx = float(tvec[0])

            # 깊이 기반 정지
            robot_y = marker_world_y - marker_dist * self._depth_scale - self._cam_fwd
            if robot_y <= dock_y + self._stop_tol:
                self._stop()
                self.get_logger().info(
                    f"Insert: 정지 — robot_y≈{robot_y:.3f} <= dock_y {dock_y:.3f}"
                )
                return True

            # 헤딩 유지: odom yaw 를 θ_ref 로 P제어(노이즈 없는 정밀 직진). odom yaw 증가=CCW
            # 이므로 (yaw-θ_ref)>0 이면 CW(-)로 보정. use_lane_steering=true 면 라인 PID 우선.
            lane = self._detect_lane(frame)
            yaw_drift = 0.0
            if self._use_lane and lane is not None and lane.conf > 0.0:
                omega = -(self._lane_lat_pid.compute(lane.lateral_px)
                          + self._lane_yaw_pid.compute(lane.yaw_rad))
            elif theta_ref is not None:
                od = self._get_odom()
                if od is not None:
                    yaw_drift = math.atan2(math.sin(od[2] - theta_ref),
                                           math.cos(od[2] - theta_ref))
                omega = -self._yaw_hold_kp * yaw_drift
            else:
                omega = 0.0

            if time.time() - last_log > 0.5:
                lane_s = (
                    f"conf={lane.conf:.2f} lat={lane.lateral_px:.0f}px yaw={lane.yaw_rad:.3f}"
                    if lane is not None else "none"
                )
                self.get_logger().info(
                    f"Insert: dist={marker_dist:.3f} robot_y={robot_y:.3f}/dock={dock_y:.3f} "
                    f"yaw_drift={math.degrees(yaw_drift):+.1f}deg lane[{lane_s}] "
                    f"lines={self._dbg_line_xs} omega={omega:.3f}"
                )
                last_log = time.time()

            twist = Twist()
            twist.linear.x = -self._reverse_speed
            twist.angular.z = self._clamp(omega)
            self._cmd_pub.publish(twist)
            time.sleep(0.05)

        self._stop()
        self.get_logger().warn("Insert: timeout")
        return False

    def _recenter_on_marker(self, marker_id: int, last_tx: float) -> bool:
        """마커 상실/시퀀스 전환 시: 후진을 멈추고 제자리 회전으로 마커를 카메라 중앙에
        되돌린다(헤딩=법선 복귀, 포지션 확정). 중앙 정렬되면 True.

        회전 부호(정적검증 테스트3 기준): 마커가 왼쪽(tvec[0]<0)이면 CCW(ω>0)로 돌려야
        중앙에 온다 → omega = -recenter_kp·bearing. 마커가 안 보이면 마지막에 보이던
        쪽으로 탐색 회전(오른쪽이었으면 CW). last_tx 부호로 방향 결정.
        """
        deadline = time.time() + self._recenter_to
        search_dir = 1.0 if last_tx >= 0.0 else -1.0
        last_log = 0.0
        while time.time() < deadline:
            if self._emergency.is_stopped():
                self._stop()
                deadline += self._wait_if_paused()
                continue

            frame = self._get_latest_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            marker = self._detect_aruco(frame, marker_id)
            twist = Twist()   # 회전만(후진 정지)
            if marker is not None:
                tvec, _ = marker
                dist = max(float(tvec[2]), 0.1)
                bearing = float(tvec[0]) / dist     # 카메라 광축 대비 마커 횡(rad 근사)
                # 마커가 왼쪽(bearing<0)이면 CCW 로 탐색해야 하므로 다음 상실 시 방향힌트.
                search_dir = -1.0 if bearing >= 0.0 else 1.0
                if abs(bearing) < self._recenter_tol:
                    self._stop()
                    self.get_logger().info(
                        f"Recenter: 마커 중앙 복귀 완료 (bearing={bearing:.3f})"
                    )
                    return True
                # 마커 왼쪽(bearing<0) → CCW(ω>0) 로 돌려야 중앙(정적검증 테스트3). 부호 -.
                twist.angular.z = self._clamp(-self._recenter_kp * bearing)
            else:
                twist.angular.z = self._clamp(search_dir * self._acquire_rot)

            self._cmd_pub.publish(twist)
            if time.time() - last_log > 0.5:
                seen = "detect" if marker is not None else "search"
                self.get_logger().info(
                    f"Recenter: 마커 중앙 정렬 중 ({seen}, dir={search_dir:+.0f})"
                )
                last_log = time.time()
            time.sleep(0.05)

        self._stop()
        self.get_logger().warn("Recenter: timeout — 마커 재중심 실패")
        return False

    def _align_yaw_to_normal(self, marker_id: int) -> bool:
        """마커가 fronto-parallel(법선 정면, psi≈0)이 되도록 제자리 yaw 정렬. best-effort.

        psi = atan2(R[0,2], -R[2,2]) (정면 정렬 시 ~0). 정면 근처에서 psi 가 ±부호로
        튀므로(rvec ±π 모호성) 매 판정마다 몇 프레임 평균낸다. d(psi)/d(omega)<0 이라
        omega = recenter_kp·psi (psi<0→CW). |psi| 가 커지면 부호 자동 반전(오류 강건).
        실패해도(마커 상실/timeout) 현재 정렬 유지하고 True(도킹 진행).
        """
        deadline = time.time() + self._recenter_to
        omega_sign = 1.0
        prev_abs = None
        last_log = 0.0
        while time.time() < deadline:
            if self._emergency.is_stopped():
                self._stop()
                deadline += self._wait_if_paused()
                continue
            psis = []
            for _ in range(15):   # 프레임별 psi 수집 후 median → flip(±π) outlier 제거.
                f = self._get_latest_frame()
                if f is not None:
                    m = self._detect_aruco(f, marker_id)
                    if m is not None:
                        R, _ = cv2.Rodrigues(
                            np.asarray(m[1], dtype=np.float64).reshape(3, 1))
                        psis.append(math.atan2(float(R[0, 2]), -float(R[2, 2])))
                time.sleep(0.02)
            if not psis:
                self._stop()
                self.get_logger().info("YawAlign: 마커 미검출 → 현 정렬 유지")
                return True
            # median: 평균은 flip 한두개에 편향되지만 중앙값은 강건(θ_ref 정확도 핵심).
            psis.sort()
            psi = psis[len(psis) // 2]
            # 마커 장착 미세 기울기 보정: 실제 법선 정면은 psi=marker_yaw_offset 일 때이므로
            # 그 기준으로 정렬(psi_err→0). (마커 fronto-parallel psi=0 이 실제 법선이 아님)
            psi_err = psi - self._marker_yaw_offset
            if abs(psi_err) < self._yaw_align_tol:
                self._stop()
                self.get_logger().info(
                    f"YawAlign: 완료 (psi={math.degrees(psi):+.1f}deg, "
                    f"법선기준 err={math.degrees(psi_err):+.1f}deg)"
                )
                return True
            if prev_abs is not None and abs(psi_err) > prev_abs + 0.03:
                omega_sign = -omega_sign
                self.get_logger().warn("YawAlign: 방향 반대 감지 → 부호 반전")
            prev_abs = abs(psi_err)
            twist = Twist()
            twist.angular.z = self._clamp(omega_sign * self._recenter_kp * psi_err)
            self._cmd_pub.publish(twist)
            if time.time() - last_log > 0.5:
                self.get_logger().info(f"YawAlign: psi={math.degrees(psi):+.1f}deg")
                last_log = time.time()
            time.sleep(0.05)
        self._stop()
        self.get_logger().info("YawAlign: timeout → 현 정렬 유지")
        return True

    def _measure_lateral_offset(self, marker_id: int):
        """마커를 여러 프레임 평균내어 법선(마커 x선)까지의 횡오차 Δx 를 1회 계산.

        Δx = robot_x - marker_x = -(tx·cos(psi) + tz·sin(psi)).  (정적검증: dec-)
        psi 는 정면(rvec_y≈±π)에서 ±부호 모호성이 있어, 프레임별 psi 대신 법선벡터
        성분(R[0,2], R[2,2])을 평균해 한 번만 psi 를 구한다(노이즈·flip 완화).
        Δx>0 이면 로봇이 법선보다 +x(동)쪽 → 서쪽으로 이동해야 함. 실패 시 None.
        """
        txs, tzs, psis = [], [], []
        deadline = time.time() + 3.0
        while len(txs) < self._measure_frames and time.time() < deadline:
            frame = self._get_latest_frame()
            if frame is None:
                time.sleep(0.03)
                continue
            m = self._detect_aruco(frame, marker_id)
            if m is None:
                time.sleep(0.03)
                continue
            tvec, rvec = m
            R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
            txs.append(float(tvec[0]))
            tzs.append(float(tvec[2]))
            psis.append(math.atan2(float(R[0, 2]), -float(R[2, 2])))
            time.sleep(0.03)
        if len(txs) < 3:
            self.get_logger().warn("Measure: 마커 샘플 부족")
            return None
        # median: flip(±π) outlier 제거(평균은 flip 한두개에 편향).
        tx = sorted(txs)[len(txs) // 2]
        tz = sorted(tzs)[len(tzs) // 2]
        psis.sort()
        psi = psis[len(psis) // 2]
        psi_n = psi - self._marker_yaw_offset
        # 마커 정면(recenter) 자세에서 측정 → tx≈0, 횡오차는 tz·sin(psi_n) 에 담긴다.
        # 법선 자세는 off-line 시 마커가 화각을 벗어나 검출 불가라 부득이 마커 정면에서 측정.
        # 그래서 디커플링 전체식 사용(psi 노이즈는 median + 모호성해소로 완화).
        dx = (-(tx * math.cos(psi_n) + tz * math.sin(psi_n)) * self._lateral_scale
              + self._marker_lat_offset)
        self.get_logger().info(
            f"Measure: Δx={dx:+.3f}m (tx={tx:.3f} tz={tz:.3f} "
            f"psi={math.degrees(psi):+.1f}deg(법선기준 {math.degrees(psi_n):+.1f}), "
            f"lat_scale={self._lateral_scale}, offset={self._marker_lat_offset}, n={len(txs)})"
        )
        return dx

    def _arc_into_line(self, dx: float) -> bool:
        """부드러운 후진 arc 로 법선(마커 x선)까지 횡이동 |Δx|. odom 상대 횡변위로 종료 판단.

        Δx>0(로봇이 법선보다 +x=동쪽) → 서쪽(로봇 왼쪽)으로 이동해야 하므로 후진+CW(ω<0).
        odom 으로 시작점 대비 '오른쪽 방향' 변위를 적분해 |Δx| 만큼 가면 정지(단일 매끄러운 호).
        부호가 반대로 가면 odom 으로 감지해 ω 를 한 번 뒤집는다(부호 오류에 강건).
        """
        if abs(dx) < self._arc_lat_tol:
            self.get_logger().info(f"Arc: 이미 법선 근처(Δx={dx:+.3f}m), 생략")
            return True
        odom0 = self._get_odom()
        if odom0 is None:
            self.get_logger().error("Arc: odom 없음 → 실패")
            return False
        ox0, oy0, oyaw0 = odom0
        # 시작 시 로봇 오른쪽(≈world +x) 단위벡터. 변위를 여기 투영해 횡이동량 측정.
        rdx, rdy = math.sin(oyaw0), -math.cos(oyaw0)
        omega = -math.copysign(self._arc_omega, dx)   # Δx>0 → ω<0(CW)=왼쪽(서)
        move_sign = -math.copysign(1.0, dx)           # 법선 쪽으로 가는 오른쪽-변위 부호
        # phantom 으로 큰 Δx 가 나와도 벽으로 돌진 못하게 이동량 캡.
        need = min(abs(dx), self._arc_max_travel)
        if need < abs(dx):
            self.get_logger().warn(
                f"Arc: |Δx|={abs(dx):.3f} 가 상한 {self._arc_max_travel} 초과 → {need} 로 제한"
            )
        deadline = time.time() + self._arc_to
        start = time.time()
        flipped = False
        progress = 0.0
        last_log = 0.0
        while time.time() < deadline:
            if self._emergency.is_stopped():
                self._stop()
                deadline += self._wait_if_paused()
                continue
            od = self._get_odom()
            if od is None:
                time.sleep(0.05)
                continue
            ox, oy, _ = od
            lat = (ox - ox0) * rdx + (oy - oy0) * rdy   # 시작프레임 기준 오른쪽 변위
            progress = move_sign * lat                  # 법선 쪽으로 간 거리(양수면 정방향)
            if progress >= need:
                self._stop()
                self.get_logger().info(f"Arc: 법선 도달 (이동 {progress:.3f}/{need:.3f}m)")
                return True
            # 방향 반대 감지(1.5s 지나도 음의 진행) → ω 한 번 반전.
            if not flipped and (time.time() - start) > 1.5 and progress < -self._arc_lat_tol:
                omega = -omega
                flipped = True
                self.get_logger().warn("Arc: 방향 반대 감지 → omega 부호 반전")
            if time.time() - last_log > 0.5:
                self.get_logger().info(
                    f"Arc: 진행 {progress:.3f}/{need:.3f}m (lat={lat:+.3f})"
                )
                last_log = time.time()
            twist = Twist()
            twist.linear.x = -self._reverse_speed
            twist.angular.z = self._clamp(omega)
            self._cmd_pub.publish(twist)
            time.sleep(0.05)
        self._stop()
        self.get_logger().warn(f"Arc: timeout (이동 {progress:.3f}/{need:.3f}m)")
        return False

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._odom_lock:
            self._odom = (p.x, p.y, yaw)

    def _get_odom(self):
        with self._odom_lock:
            return self._odom

    def _rotate_to_odom_yaw(self, theta_target: float,
                            tol: float = 0.01, timeout: float = 4.0) -> bool:
        """odom yaw 를 theta_target 으로 제자리 회전(P제어).

        부호가 표준이라 psi 의 ±π/부호 혼란이 없다: odom yaw 는 CCW 로 증가하고
        angular.z>0 도 CCW 이므로 err=target-cur>0 이면 ω>0(CCW)로 좁힌다. odom 은
        단기적으로 정밀해 헤딩오차 δ 를 ~0 으로 만든다(횡 측정의 D·sin(δ) 누설 차단).
        odom 없으면 즉시 True(best-effort, 현 헤딩 유지).
        """
        deadline = time.time() + timeout
        last_log = 0.0
        while time.time() < deadline:
            if self._emergency.is_stopped():
                self._stop()
                deadline += self._wait_if_paused()
                continue
            od = self._get_odom()
            if od is None:
                self._stop()
                self.get_logger().warn("OdomYaw: odom 없음 → 현 헤딩 유지")
                return True
            err = math.atan2(math.sin(theta_target - od[2]),
                             math.cos(theta_target - od[2]))
            if abs(err) < tol:
                self._stop()
                return True
            twist = Twist()
            twist.angular.z = self._clamp(self._yaw_hold_kp * err)   # err>0 → CCW(+)
            self._cmd_pub.publish(twist)
            if time.time() - last_log > 0.5:
                self.get_logger().info(
                    f"OdomYaw: 법선 복귀 중 err={math.degrees(err):+.1f}deg "
                    f"(target={math.degrees(theta_target):+.1f}deg)"
                )
                last_log = time.time()
            time.sleep(0.05)
        self._stop()
        self.get_logger().info("OdomYaw: timeout → 현 헤딩 유지")
        return True

    # ====================================================================== #
    # 검출
    # ====================================================================== #

    def _detect_aruco(self, frame, target_id: int):
        """IPPE_SQUARE 기반 ArUco pose. 평면 ±π 모호성을 두 해 중 직전 psi 와 가장
        가까운 해(시간연속성)로 골라 해소한다. 첫 검출은 재투영오차 작은 해.
        (tvec[3], rvec[3]) 또는 None."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)
        if ids is None:
            return None

        for i, mid in enumerate(ids.flatten()):
            if mid != target_id:
                continue
            n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
                self._marker_obj_pts,
                corners[i][0].astype(np.float64),
                self._cam_matrix,
                self._dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,   # 평면 정사각 마커: 최대 2해 반환
            )
            if n < 1:
                return None
            # 각 해의 psi(법선기준 헤딩) 계산
            cands = []
            for si in range(n):
                R, _ = cv2.Rodrigues(rvecs[si])
                psi = math.atan2(float(R[0, 2]), -float(R[2, 2]))
                e = float(errs[si][0]) if errs is not None else 0.0
                cands.append((psi, e, rvecs[si], tvecs[si]))
            # ±π 모호성 해소: 직전 psi 가 있으면 그와 가장 가까운 해(8° 튐 거름),
            # 없으면(첫 검출) 재투영오차 작은 해. 정면 근처서 두 오차가 비슷해도
            # 시간연속성으로 한 군집에 고정된다.
            if self._last_psi is not None and n > 1:
                psi, _, rvec, tvec = min(
                    cands, key=lambda c: abs(c[0] - self._last_psi))
            else:
                psi, _, rvec, tvec = min(cands, key=lambda c: c[1])
            self._last_psi = psi
            return tvec.flatten(), rvec.flatten()

        return None

    def _detect_lane(self, frame) -> LaneResult | None:
        """노란 주차라인(3줄: 왼|중앙(공유)|오) 중 도크 채널 2줄의 대칭선을 낸다.

        하단 ROI 에서 컬럼 히스토그램으로 라인 클러스터를 분리해 x 순 정렬한 뒤,
        dock1(좌)은 가장 왼쪽 2줄(왼+중앙), dock2(우)는 가장 오른쪽 2줄(중앙+오)을
        채널로 선택한다. 그 중심선(대칭선)의 수평 오프셋·기울기·신뢰도를 반환.
        """
        h, w = frame.shape[:2]
        roi = frame[h // 2:, :]              # 하단 절반(바닥)
        rh = roi.shape[0]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        yellow = cv2.inRange(hsv, self._yellow_lo, self._yellow_hi)

        lines = self._find_lines(yellow, rh)   # [(x_bottom, ang, px), ...] x 오름차순
        self._dbg_line_xs = [round(x) for (x, _, _) in lines]
        if len(lines) < 2:
            return None
        # 중앙선은 dock1·dock2 공유. dock1=왼쪽 2줄, dock2=오른쪽 2줄을 채널로 선택.
        (lx, l_ang, l_px), (rx, r_ang, r_px) = (
            (lines[0], lines[1]) if self._dock_left else (lines[-2], lines[-1])
        )

        center_x = (lx + rx) / 2.0
        center_ang = (l_ang + r_ang) / 2.0
        lateral_px = center_x - w / 2.0          # +면 중심이 우측
        yaw_rad = center_ang                     # 수직 대비 기울기(우측+)

        m = min(l_px, r_px)
        conf = (m - self._lane_min_px) / max(self._lane_full_px - self._lane_min_px, 1)
        conf = float(max(0.0, min(1.0, conf)))
        if conf <= 0.0:
            return None
        return LaneResult(lateral_px, yaw_rad, conf)

    def _find_lines(self, yellow, rh: int):
        """노란 마스크에서 수직 라인 클러스터를 x 오름차순으로 반환한다.

        컬럼별 노란 픽셀이 ROI 세로의 lane_col_active_ratio 이상이면 라인 컬럼으로 보고,
        인접 라인 컬럼을 묶어 한 라인으로 피팅한다. 반환: [(x_bottom, ang, px), ...].
        """
        col_count = (yellow > 0).sum(axis=0)
        active = col_count > (rh * self._lane_col_ratio)
        w = yellow.shape[1]
        lines = []
        c = 0
        while c < w:
            if not active[c]:
                c += 1
                continue
            c0 = c
            while c < w and active[c]:
                c += 1
            fit = self._fit_line(yellow[:, c0:c], c0)
            if fit is not None:
                lines.append(fit)
        lines.sort(key=lambda t: t[0])
        return lines

    def _fit_line(self, mask, x_offset: int):
        """마스크 비영 픽셀을 직선 피팅. (하단 x, 수직대비각rad, 픽셀수) 또는 None."""
        pts = cv2.findNonZero(mask)
        if pts is None or len(pts) < self._lane_min_px:
            return None
        pts = pts.reshape(-1, 2).astype(np.float32)
        vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
        if abs(vy) < 1e-6:
            return None
        rh = mask.shape[0]
        # 하단 행(y=rh-1)에서의 x (로봇에 가장 가까운 지점)
        x_bottom = x0 + (rh - 1 - y0) * (vx / vy) + x_offset
        # 수직(이미지 아래방향) 대비 기울기. vx/vy 가 0이면 완전 수직.
        ang = math.atan2(vx, abs(vy))   # +면 위로 갈수록 우측
        return float(x_bottom), float(ang), int(len(pts))

    # ====================================================================== #
    # 위치 보정
    # ====================================================================== #

    def _publish_pose_correction(self, x: float, y: float, yaw: float):
        """도크 절대 좌표를 /initialpose 로 발행해 AMCL 재초기화."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        half = yaw / 2.0
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)
        msg.pose.covariance[0]  = 0.01
        msg.pose.covariance[7]  = 0.01
        msg.pose.covariance[35] = 0.005
        self._init_pose_pub.publish(msg)
        self.get_logger().info(
            f"Pose correction: ({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f}deg)"
        )

    # ====================================================================== #
    # 유틸
    # ====================================================================== #

    def _clamp(self, val: float) -> float:
        return max(min(val, self._max_ang), -self._max_ang)

    def _image_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._lock:
                self._latest_frame = frame
        except Exception as e:
            self.get_logger().warn(f"Image conversion error: {e}")

    def _get_latest_frame(self):
        if self._camera_source == "picamera2":
            if self._picam2 is None:
                return None
            try:
                # RGB888 배열은 BGR 바이트 순서 → 마커/HSV 검출(bgr8 기준)과 일치.
                frame = self._picam2.capture_array()
            except Exception as e:
                self.get_logger().warn(f"Picamera2 capture error: {e}")
                return None
        else:
            with self._lock:
                frame = self._latest_frame.copy() if self._latest_frame is not None else None
        if frame is None:
            return None
        # 카메라가 거꾸로(180°) 장착됨 + 캘리브레이션도 flip 된 이미지 기준이므로, 검출 전
        # 180° flip 으로 상하좌우를 바로잡는다. 없으면 마커 횡/yaw·라인 좌우가 전부 반전된다.
        if self._flip_180:
            frame = cv2.flip(frame, -1)
        return frame

    def _open_camera(self) -> None:
        """picamera2 모드: 도킹 시작 시 카메라를 직접 연다(워밍업 포함)."""
        if self._camera_source != "picamera2" or self._picam2 is not None:
            return
        try:
            from picamera2 import Picamera2  # 보드 전용, 지연 import
            picam = Picamera2()
            picam.configure(picam.create_video_configuration(
                main={"size": (self._cam_w, self._cam_h), "format": "RGB888"}
            ))
            picam.start()
            time.sleep(0.5)  # 노출 워밍업
            self._picam2 = picam
            self.get_logger().info(
                f"ReverseDocking: Picamera2 open ({self._cam_w}x{self._cam_h})"
            )
        except Exception as e:
            self.get_logger().error(f"Picamera2 open 실패: {e}")
            self._picam2 = None

    def _close_camera(self) -> None:
        """picamera2 모드: 도킹 종료 시 카메라를 반납한다(다음 도킹/뷰어가 쓰게)."""
        if self._picam2 is None:
            return
        try:
            self._picam2.stop()
        except Exception:
            pass
        try:
            self._picam2.close()
        except Exception:
            pass
        self._picam2 = None
        self.get_logger().info("ReverseDocking: Picamera2 close")

    @staticmethod
    def _resolve_pkg_path(path):
        """package://<pkg>/<rel> URI 를 install share 절대경로로 변환한다."""
        if path and path.startswith("package://"):
            from ament_index_python.packages import get_package_share_directory
            pkg, _, rel = path[len("package://"):].partition("/")
            try:
                return os.path.join(get_package_share_directory(pkg), rel)
            except Exception:
                return path
        return path

    def _load_calibration(self, path):
        """ROS camera_info yaml 에서 camera_matrix(3x3)·dist(5) 를 직접 읽는다.

        경로가 비었거나 읽기 실패면 None 을 반환해 파라미터 fallback 을 쓰게 한다.
        """
        path = self._resolve_pkg_path(path)
        if not path or not os.path.exists(path):
            if path:
                self.get_logger().warn(
                    f"캘리브레이션 yaml 없음: {path} → 파라미터 fallback"
                )
            return None
        try:
            with open(path) as f:
                data = yaml.safe_load(f)
            K = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
            dist = np.array(
                data["distortion_coefficients"]["data"], dtype=np.float64
            )
            self.get_logger().info(f"캘리브레이션 로드: {path} (fx={K[0, 0]:.1f})")
            return K, dist
        except Exception as e:
            self.get_logger().error(f"캘리브레이션 로드 실패({path}): {e} → fallback")
            return None

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _wait_if_paused(self) -> float:
        """비상 정지 중이면 재개될 때까지 제자리에서 대기(pause-continue).

        대기 동안 0 속도 명령을 재발행해 도킹 중 로봇을 확실히 멈춰둔다.
        반환값은 대기 시간(초)으로 호출부 deadline 을 그만큼 미룬다.
        """
        if not self._emergency.is_stopped():
            return 0.0
        start = time.time()
        self.get_logger().warn(
            f"[비상정지] 도킹 일시정지 — reason={self._emergency.reason}"
        )
        while self._emergency.is_stopped() and rclpy.ok():
            self._stop()
            time.sleep(0.1)
        waited = time.time() - start
        self.get_logger().info(f"[비상정지] 도킹 재개 ({waited:.1f}s 정지)")
        return waited


def main(args=None):
    rclpy.init(args=args)
    node = ReverseDocking()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
