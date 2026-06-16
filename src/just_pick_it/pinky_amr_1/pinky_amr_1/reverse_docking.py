#!/usr/bin/env python3
"""
Reverse Docking - 두 ArUco 마커 localization + odom 정밀 곡선 이동 후진 도킹

설계 (좁은 도크 정밀 도킹용, 2026-06-15 최종):
  단일 평면 마커 rvec(yaw)는 pose-flip 으로 ±5~8° 노이즈라 헤딩 병목. 도크의 마커 2개
  (id0,id1, 같은 벽)의 translation 만으로(회전 rvec 미사용) 강체정합해 로봇 (x,y)·벽기준
  yaw 를 ambiguity 없이 1회 측정하고, 실제 이동은 noise 없는 odom 으로 한다.

  6단계 시퀀스 (reverse_dock):
    1. 주마커(id0) 향해 정렬 (acquire + recenter)
    2. 법선 yaw 정렬 (단일 psi, 대략)
    3. 법선방향 converged_reverse_m 직진 후진 (두-마커 검출 영역 진입)
    4. 쌍마커 검출 + 월드 pose 측정 (마커 사이로 회전 → 둘 다 검출 → robot_x,y·psi_two;
       가까워 partner 화각밖이면 2cm 씩 추가 후진 재시도)
    5. odom 정밀 곡선 이동 (dock_x, approach_y, 법선 yaw 로 후진 곡선 + 종료자세 회전)
    6. dock_y 까지 직진 후진 + 마커 깊이로 정지 → /initialpose 보정

  좌표 전제:
    - 로봇은 헤드(+x_body, 전방 카메라)가 +y_world(마커 쪽)를 보며 -y 로 후진해 도크에 들어간다.
    - solvePnP tvec: 카메라(optical) 기준. tvec[0]>0 = 마커 우측, tvec[2] = 카메라→마커 거리.
    - tx 는 fx 무관(정확), tz 는 fx 비례(영상모드 crop 으로 짧게 → fx_scale 보정).

마커 월드 좌표는 이 노드 설정(marker_id 별)에 둔다.
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
        # 캘리브 fx 가 실제 도킹 영상모드(picamera2 crop)보다 작게 잡혀 tz(깊이)·회전이
        # 어긋난다(실측 D/tz=1.48). fx,fy 에 이 배율을 곱해 보정한다. tx(횡)는 fx 무관이라
        # 영향 없음. 제대로는 도킹 영상모드 그대로 재캘리브가 정답(이건 stopgap).
        self.declare_parameter("fx_scale", 1.0)
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
        # solvePnP tz(깊이)가 fx(영상모드 crop)로 짧게 나오는 걸 보정(robot_y 정지용).
        # tx(횡)는 fx 무관이라 별도 스케일 불필요.
        self.declare_parameter("depth_scale", 1.36)
        # 마커 정면 정렬 시 rvec[1]≈π 인데, 마커 장착 미세 기울기 등으로 도크-정렬 헤딩과
        # 몇 도 어긋날 수 있다. yaw 목표를 이만큼 보정(도). +면 더 회전(rvec[1] 목표를 +방향).
        self.declare_parameter("marker_yaw_offset_deg", 0.0)

        # ── 마커 탐색/재중심 ────────────────────────────────────────────
        self.declare_parameter("acquire_rotate_speed", 0.3)   # 마커 탐색 회전(rad/s)
        # 채널 안에서 마커 상실 시: 후진을 멈추고 제자리 회전으로 마커를 카메라 중앙에
        # 되돌리는 재중심 단계. kp=중심 정렬 회전 게인, tol=중앙 허용 bearing(rad).
        self.declare_parameter("recenter_kp", 0.8)
        self.declare_parameter("recenter_tol_rad", 0.05)
        self.declare_parameter("recenter_timeout_sec", 6.0)
        # 후진/측정 공통: 후진 직진 timeout, 두-마커 측정 평균 프레임수.
        self.declare_parameter("arc_timeout_sec", 20.0)
        self.declare_parameter("measure_frames", 10)
        # 법선 정렬 후 두-마커 검출 영역으로 진입하기 위한 법선방향 직진 후진량(m).
        self.declare_parameter("converged_reverse_m", 0.10)
        # 후진 직전 법선 헤딩을 두-마커 yaw 로 정밀 정렬(단일 psi 노이즈 회피). 두 마커를
        # 보려면 마커 사이를 조준해야 하므로 partner 쪽으로 제자리 회전해 둘 다 잡은 뒤
        # psi_two 만큼 되돌려 법선 정면을 만든다. 실패 시 단일 psi(_align_yaw_to_normal) fallback.
        self.declare_parameter("use_two_marker_normal", True)
        # psi_two 의 tz 차에만 적용하는 로컬 fx 보정(전역 cam_matrix·depth 경로는 안 건드림).
        # 실측 D/tz=1.48. tx 는 fx 무관이라 보정 안 함.
        self.declare_parameter("two_marker_fx_scale", 1.48)
        # partner 마커를 화각에 넣기 위한 최대 제자리 회전(rad). 초과하면 못 찾은 것으로 보고 fallback.
        self.declare_parameter("two_marker_max_turn_rad", 0.6)
        # partner 탐색 회전 속도(rad/s). acquire_rot(0.3)은 너무 빨라 동시검출 구간을
        # 지나치고 모션블러로 검출 실패 → 느리게. 첫 동시검출에 멈춰 정착 후 확인한다.
        self.declare_parameter("two_marker_search_omega", 0.08)
        # 두-마커 검출 후 곡선 이동 목표 접근 깊이(월드 y, m). 여기서 dock_y 까지 직진 후진.
        self.declare_parameter("approach_y_m", 0.18)
        # 곡선 이동 도달 허용오차(m)와 후진 선속 게인(v=-kv·dist, reverse_speed 캡).
        self.declare_parameter("curve_pos_tol_m", 0.012)
        self.declare_parameter("curve_kv", 0.6)
        # 곡선 이동의 횡(델타x) 게인. 약간 서(왼)쪽으로 지나쳐 멈추면 <1.0 으로 덜 이동.
        # 카메라 offset 대신 odom 이동량만 줄여 다른 경로엔 영향 없음.
        self.declare_parameter("curve_lat_gain", 0.9)
        # 두-마커 검출 실패 시(너무 가까워 partner 화각밖): 법선 복귀 후 이만큼 추가 후진하고
        # 재시도. 멀어질수록 두 마커 각이 좁아져 화각에 들어온다. 최대 재시도 횟수.
        self.declare_parameter("two_marker_retry_reverse_m", 0.02)
        self.declare_parameter("two_marker_max_retries", 5)
        # yaw 정렬 완료 허용오차(rad). 마커 fronto-parallel(법선 정면)까지 제자리 회전.
        self.declare_parameter("yaw_align_tol_rad", 0.04)   # ~2.3deg
        # 후진 중 odom yaw 헤딩 유지 P게인. θ_ref(법선)에서 벗어나면 되돌린다.
        self.declare_parameter("yaw_hold_kp", 1.0)

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
        # fx,fy 보정(영상모드 crop 으로 캘리브 fx 가 작게 잡힘). tx 는 fx 무관이라 영향 없고
        # tz(깊이)·회전만 보정된다. 기본 1.0 이면 무동작.
        self._fx_scale = float(self.get_parameter("fx_scale").value)
        if self._fx_scale != 1.0:
            self._cam_matrix = self._cam_matrix.copy()
            self._cam_matrix[0, 0] *= self._fx_scale
            self._cam_matrix[1, 1] *= self._fx_scale
            self.get_logger().info(
                f"fx_scale={self._fx_scale} 적용 → fx={self._cam_matrix[0, 0]:.1f}"
            )
        self._cam_fwd = self.get_parameter("camera_forward_offset_m").value
        self._depth_scale = self.get_parameter("depth_scale").value
        self._marker_yaw_offset = math.radians(self.get_parameter("marker_yaw_offset_deg").value)

        self._camera_source = self.get_parameter("camera_source").value
        self._cam_w = int(self.get_parameter("camera_width").value)
        self._cam_h = int(self.get_parameter("camera_height").value)
        self._flip_180 = bool(self.get_parameter("flip_camera_180").value)

        self._acquire_rot   = self.get_parameter("acquire_rotate_speed").value
        self._recenter_kp  = self.get_parameter("recenter_kp").value
        self._recenter_tol = self.get_parameter("recenter_tol_rad").value
        self._recenter_to  = self.get_parameter("recenter_timeout_sec").value
        self._arc_to        = self.get_parameter("arc_timeout_sec").value
        self._measure_frames = int(self.get_parameter("measure_frames").value)
        self._converged_reverse_m = self.get_parameter("converged_reverse_m").value
        self._use_two_marker_normal = bool(
            self.get_parameter("use_two_marker_normal").value)
        self._two_marker_fx_scale = float(
            self.get_parameter("two_marker_fx_scale").value)
        self._two_marker_max_turn = float(
            self.get_parameter("two_marker_max_turn_rad").value)
        self._two_marker_search_omega = float(
            self.get_parameter("two_marker_search_omega").value)
        self._approach_y = float(self.get_parameter("approach_y_m").value)
        self._curve_pos_tol = float(self.get_parameter("curve_pos_tol_m").value)
        self._curve_kv = float(self.get_parameter("curve_kv").value)
        self._curve_lat_gain = float(self.get_parameter("curve_lat_gain").value)
        self._two_marker_retry_reverse = float(
            self.get_parameter("two_marker_retry_reverse_m").value)
        self._two_marker_max_retries = int(
            self.get_parameter("two_marker_max_retries").value)
        self._yaw_align_tol = self.get_parameter("yaw_align_tol_rad").value
        self._yaw_hold_kp   = self.get_parameter("yaw_hold_kp").value

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

        self.get_logger().info(
            f"reverse_dock: marker={marker_id}@world({marker_x:.3f},{marker_y:.3f}), "
            f"dock=({dock_map_x:.3f},{dock_map_y:.3f},"
            f"{math.degrees(dock_map_yaw):.1f}deg)"
        )

        # picamera2 모드는 도킹 동안만 카메라를 열고, 끝나면(성공/실패 무관) 반납한다.
        self._open_camera()
        try:
            # 1) 마커 획득 (보일 때까지 탐색 회전)
            if not self._acquire_marker(marker_id):
                self._stop()
                self.get_logger().error("reverse_dock: FAILED — 마커 미획득")
                return False

            # 2) 주마커 정면 정렬 후 법선 yaw 정렬(단일 psi).
            if not self._recenter_on_marker(marker_id, 0.0):
                self._stop()
                self.get_logger().error("reverse_dock: FAILED — 초기 재정렬")
                return False
            self._align_yaw_to_normal(marker_id)

            # 3) 법선방향 10cm 후진(두-마커 검출 영역 진입).
            self._reverse_straight_odom(self._converged_reverse_m)

            # 4) 쌍마커 검출 + 월드 pose 측정(마커 사이를 조준해 둘 다 검출).
            #    너무 가까우면 partner 가 화각밖이라 실패 → 법선 복귀 후 2cm 더 후진하고
            #    재시도(멀어질수록 두 마커 각이 좁아져 화각에 들어온다).
            od_n = self._get_odom()
            normal_yaw = od_n[2] if od_n is not None else None
            pose = self._measure_two_marker_pose(marker_id)
            retries = 0
            while pose is None and retries < self._two_marker_max_retries:
                retries += 1
                if normal_yaw is not None:
                    self._rotate_to_odom_yaw(normal_yaw, "재시도 법선복귀")
                self.get_logger().warn(
                    f"reverse_dock: 두-마커 실패 → {self._two_marker_retry_reverse:.2f}m "
                    f"추가 후진 후 재시도({retries}/{self._two_marker_max_retries})")
                self._reverse_straight_odom(self._two_marker_retry_reverse)
                od_n = self._get_odom()
                normal_yaw = od_n[2] if od_n is not None else normal_yaw
                pose = self._measure_two_marker_pose(marker_id)
            if pose is None:
                self.get_logger().warn(
                    "reverse_dock: 두-마커 측정 실패(재시도 소진) → 단일 psi 법선정렬 fallback")
                self._align_yaw_to_normal(marker_id)
            else:
                robot_x, robot_y, psi_two, (ox_m, oy_m, oth_m) = pose
                # 5) (dock_x, approach_y, 법선 월드yaw=π/2) 로 odom 정밀 곡선 이동.
                #    world->odom 회전 α 로 목표 월드점을 odom 으로 변환.
                alpha = oth_m - (math.pi / 2.0 - psi_two)
                ca, sa = math.cos(alpha), math.sin(alpha)
                # 횡(델타x)만 curve_lat_gain(0.9) 배 — 약간 서쪽으로 지나치는 경향 완화.
                wx = (dock_map_x - robot_x) * self._curve_lat_gain
                wy = self._approach_y - robot_y
                to_x = ox_m + ca * wx - sa * wy
                to_y = oy_m + sa * wx + ca * wy
                to_yaw = self._wrap(oth_m + psi_two)
                self.get_logger().info(
                    f"reverse_dock: 두-마커 robot=({robot_x:.3f},{robot_y:.3f}) "
                    f"psi_two={math.degrees(psi_two):+.1f}deg → 곡선목표 "
                    f"odom=({to_x:.3f},{to_y:.3f},{math.degrees(to_yaw):+.1f}deg)")
                self._curve_to_pose_odom(to_x, to_y, to_yaw)

            # 6) dock_y 까지 직진 후진 + 마커 깊이로 정지.
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
    # 단계 C: 후진 삽입 (마커 깊이 정지)
    # ====================================================================== #

    def _reverse_insert(
        self,
        marker_id: int,
        marker_world_y: float,
        dock_y: float,
    ) -> bool:
        """후진 도킹 + 마커 깊이 정지. 진입 시 로봇은 이미 법선 정렬·마커 정면 상태.

        헤딩 유지: 진입 직후 odom yaw 를 θ_ref(=법선 헤딩)로 앵커링하고, 후진 내내
        odom yaw=θ_ref 로 P제어해 헤딩을 정밀하게 고정한다(psi 노이즈 안 씀). 작은 yaw
        오차도 후진하며 x 드리프트로 누적되므로, 흔들림 없이 일정 헤딩으로 곧게 내려간다.
        마커 상실 시 제자리 회전으로 재중심. 정지: 로봇 world y(= marker_y - 마커거리·scale
        - 카메라 전방오프셋) <= dock_y.
        """
        deadline = time.time() + self._insert_to
        last_tx = 0.0
        last_log = 0.0
        # 후진 시작 헤딩(법선)을 odom 으로 앵커링 → 이 헤딩을 유지한다.
        # 헤딩 미세보정: 정렬이 일정하게 좌로 약간 틀어지므로 앵커값을 marker_yaw_offset 만큼
        # 우(CW=yaw 감소)로 옮긴다. 측정·정렬이 모두 끝난 뒤 odom 앵커값만 바꾸는 것이라
        # x(횡)에는 영향이 없다. marker_yaw_offset_deg>0 = 우회전, 더 좌면 음수로.
        od0 = self._get_odom()
        theta_ref = od0[2] - self._marker_yaw_offset if od0 is not None else None
        if theta_ref is not None:
            self.get_logger().info(
                f"Insert: odom 헤딩 앵커 θ_ref={math.degrees(theta_ref):+.1f}deg "
                f"(헤딩보정 {math.degrees(self._marker_yaw_offset):+.1f}deg 우)"
            )

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
            # 이므로 (yaw-θ_ref)>0 이면 CW(-)로 보정.
            yaw_drift = 0.0
            if theta_ref is not None:
                od = self._get_odom()
                if od is not None:
                    yaw_drift = math.atan2(math.sin(od[2] - theta_ref),
                                           math.cos(od[2] - theta_ref))
                omega = -self._yaw_hold_kp * yaw_drift
            else:
                omega = 0.0

            if time.time() - last_log > 0.5:
                self.get_logger().info(
                    f"Insert: dist={marker_dist:.3f} robot_y={robot_y:.3f}/dock={dock_y:.3f} "
                    f"yaw_drift={math.degrees(yaw_drift):+.1f}deg omega={omega:.3f}"
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
            nxs, nzs = [], []
            for _ in range(3):
                f = self._get_latest_frame()
                if f is not None:
                    m = self._detect_aruco(f, marker_id)
                    if m is not None:
                        R, _ = cv2.Rodrigues(
                            np.asarray(m[1], dtype=np.float64).reshape(3, 1))
                        nxs.append(float(R[0, 2]))
                        nzs.append(float(R[2, 2]))
                time.sleep(0.02)
            if not nxs:
                self._stop()
                self.get_logger().info("YawAlign: 마커 미검출 → 현 정렬 유지")
                return True
            psi = math.atan2(sum(nxs) / len(nxs), -sum(nzs) / len(nzs))
            if abs(psi) < self._yaw_align_tol:
                self._stop()
                self.get_logger().info(f"YawAlign: 완료 (psi={math.degrees(psi):+.1f}deg)")
                return True
            if prev_abs is not None and abs(psi) > prev_abs + 0.03:
                omega_sign = -omega_sign
                self.get_logger().warn("YawAlign: 방향 반대 감지 → 부호 반전")
            prev_abs = abs(psi)
            twist = Twist()
            twist.angular.z = self._clamp(omega_sign * self._recenter_kp * psi)
            self._cmd_pub.publish(twist)
            if time.time() - last_log > 0.5:
                self.get_logger().info(f"YawAlign: psi={math.degrees(psi):+.1f}deg")
                last_log = time.time()
            time.sleep(0.05)
        self._stop()
        self.get_logger().info("YawAlign: timeout → 현 정렬 유지")
        return True

    def _detect_two_markers(self, frame, id_a: int, id_b: int):
        """한 프레임에서 두 마커 id 의 (tvec,rvec) 를 한 번의 검출로 얻는다.

        반환: {id: (tvec, rvec)} (검출된 것만). 두-마커 헤딩 정렬 전용.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)
        out = {}
        if ids is None:
            return out
        for i, mid in enumerate(ids.flatten()):
            mid = int(mid)
            if mid != id_a and mid != id_b:
                continue
            ok, rvec, tvec = cv2.solvePnP(
                self._marker_obj_pts,
                corners[i][0].astype(np.float64),
                self._cam_matrix, self._dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if ok:
                out[mid] = (tvec.flatten(), rvec.flatten())
        return out

    def _measure_two_marker_pose(self, marker_id: int):
        """두 마커로 로봇 월드 pose 를 측정한다. (robot_x, robot_y, psi_two, (ox,oy,oθ)) 또는 None.

        법선 정면에선 마커 1개만 보이므로 partner 쪽으로 느리게 회전해 둘 다 잡은 뒤,
        translation 만으로(회전 rvec 안 씀) 강체정합해 로봇중심 월드 (x,y) 와 벽 기준
        yaw(psi_two)를 낸다. psi_two=atan2((tz1-tz0)·fx, tx1-tx0) (월드 x 작은쪽=0,큰쪽=1).
        tz 만 로컬 fx 보정(전역 depth 경로 무영향). 같은 순간의 odom 도 함께 반환해
        호출부가 world->odom 변환에 쓴다. 실패 시 None.
        """
        if not self._use_two_marker_normal:
            return None
        others = [i for i in self._marker_world if i != marker_id]
        if not others:
            return None
        partner = others[0]
        lo_id, hi_id = sorted(
            (marker_id, partner), key=lambda i: self._marker_world[i][0])
        # partner 가 오른쪽(+x)이면 CW(omega<0)로 돌려 화각에 넣는다.
        turn_dir = -1.0 if self._marker_world[partner][0] > self._marker_world[marker_id][0] else 1.0

        od0 = self._get_odom()
        if od0 is None:
            self.get_logger().warn("TwoMarker: odom 없음")
            return None
        start_yaw = od0[2]

        # Phase 1: partner 가 보일 때까지 느리게 제자리 회전. 동시검출 구간이 좁고 보드
        # 검출이 느려 빠르면 지나친다 → search_omega(느림)로 돌고, 첫 동시검출 즉시 멈춰
        # 정착(모션블러 제거) 후 정지상태에서 재확인되면 채택.
        deadline = time.time() + max(self._recenter_to,
                                     self._two_marker_max_turn / max(self._two_marker_search_omega, 0.01) + 4.0)
        found = False
        last_log = 0.0
        while time.time() < deadline:
            if self._emergency.is_stopped():
                self._stop()
                deadline += self._wait_if_paused()
                continue
            frame = self._get_latest_frame()
            if frame is not None:
                dets = self._detect_two_markers(frame, lo_id, hi_id)
                if lo_id in dets and hi_id in dets:
                    self._stop()
                    time.sleep(0.25)
                    ok = 0
                    for _ in range(3):
                        f2 = self._get_latest_frame()
                        if f2 is not None:
                            d2 = self._detect_two_markers(f2, lo_id, hi_id)
                            if lo_id in d2 and hi_id in d2:
                                ok += 1
                        time.sleep(0.05)
                    if ok >= 2:
                        self.get_logger().info("TwoMarker: 양마커 동시검출 확보(정지 확인)")
                        found = True
                        break
            od = self._get_odom()
            if od is not None and abs(self._angdiff(od[2], start_yaw)) > self._two_marker_max_turn:
                self._stop()
                self.get_logger().warn("TwoMarker: 최대회전 초과로 양마커 미검출")
                return None
            twist = Twist()
            twist.angular.z = self._clamp(turn_dir * self._two_marker_search_omega)
            self._cmd_pub.publish(twist)
            if time.time() - last_log > 0.5:
                self.get_logger().info(
                    f"TwoMarker: partner 탐색 회전(dir={turn_dir:+.0f}, ω={self._two_marker_search_omega})")
                last_log = time.time()
            time.sleep(0.05)
        if not found:
            self.get_logger().warn("TwoMarker: partner 탐색 timeout")
            return None

        # Phase 2: 두 마커 tx,tz + odom(x,y,yaw) 윈도우 평균.
        txs_lo, tzs_lo, txs_hi, tzs_hi = [], [], [], []
        oxs, oys, oys_yaw = [], [], []
        t_end = time.time() + 2.0
        while len(txs_lo) < self._measure_frames and time.time() < t_end:
            frame = self._get_latest_frame()
            if frame is None:
                time.sleep(0.03)
                continue
            dets = self._detect_two_markers(frame, lo_id, hi_id)
            if lo_id in dets and hi_id in dets:
                txs_lo.append(float(dets[lo_id][0][0]))
                tzs_lo.append(float(dets[lo_id][0][2]))
                txs_hi.append(float(dets[hi_id][0][0]))
                tzs_hi.append(float(dets[hi_id][0][2]))
                od = self._get_odom()
                if od is not None:
                    oxs.append(od[0])
                    oys.append(od[1])
                    oys_yaw.append(od[2])
            time.sleep(0.03)
        if len(txs_lo) < 3 or not oys_yaw:
            self.get_logger().warn("TwoMarker: 샘플 부족")
            return None

        # tz 만 로컬 fx 보정. psi_two = 벽 기준 카메라 yaw(strong, translation 기반).
        mtx_lo = sum(txs_lo) / len(txs_lo)
        mtz_lo = sum(tzs_lo) / len(tzs_lo) * self._two_marker_fx_scale
        mtx_hi = sum(txs_hi) / len(txs_hi)
        mtz_hi = sum(tzs_hi) / len(tzs_hi) * self._two_marker_fx_scale
        psi_two = math.atan2(mtz_hi - mtz_lo, mtx_hi - mtx_lo)
        if abs(psi_two) > self._two_marker_max_turn + 0.2:
            self.get_logger().warn(
                f"TwoMarker: psi_two={math.degrees(psi_two):+.1f}deg 과대")
            return None

        # 강체정합 카메라 월드 (x,y) → 로봇중심(카메라 전방오프셋 역산).
        c, s = math.cos(psi_two), math.sin(psi_two)
        cx_lo = self._marker_world[lo_id][0] - (c * mtx_lo + s * mtz_lo)
        cx_hi = self._marker_world[hi_id][0] - (c * mtx_hi + s * mtz_hi)
        cy_lo = self._marker_world[lo_id][1] - (-s * mtx_lo + c * mtz_lo)
        cy_hi = self._marker_world[hi_id][1] - (-s * mtx_hi + c * mtz_hi)
        cam_x = 0.5 * (cx_lo + cx_hi)
        cam_y = 0.5 * (cy_lo + cy_hi)
        robot_x = cam_x - self._cam_fwd * math.sin(psi_two)
        robot_y = cam_y - self._cam_fwd * math.cos(psi_two)
        odom_at = (sum(oxs) / len(oxs), sum(oys) / len(oys),
                   sum(oys_yaw) / len(oys_yaw))
        self.get_logger().info(
            f"TwoMarker: robot=({robot_x:.3f},{robot_y:.3f}) "
            f"psi_two={math.degrees(psi_two):+.1f}deg "
            f"정합잔차x={abs(cx_lo - cx_hi)*1000:.0f}mm")
        return robot_x, robot_y, psi_two, odom_at

    def _curve_to_pose_odom(self, tx: float, ty: float, tyaw: float) -> bool:
        """odom 기준 (tx,ty,tyaw) 로 후진 곡선 이동(정밀 정차). best-effort True.

        후진으로 목표점에 접근하므로 '로봇 뒤쪽이 목표를 향하도록' 조향한다:
        목표 desired heading = atan2(-ey,-ex)(rear 가 목표를 가리키는 body yaw). v<0 로
        후진하며 ω=kp·(desired-cur). 목표점 도달(거리<tol) 후 tyaw 로 제자리 회전.
        """
        deadline = time.time() + self._arc_to * 2.0
        start = self._get_odom()
        start_dist = math.hypot(tx - start[0], ty - start[1]) if start else 0.0
        min_dist = start_dist
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
            cx, cy, cth = od
            ex, ey = tx - cx, ty - cy
            dist = math.hypot(ex, ey)
            min_dist = min(min_dist, dist)
            if dist < self._curve_pos_tol:
                # 목표점 도달 → 종료 자세(법선)로 제자리 회전.
                self._stop()
                self.get_logger().info(f"Curve: 목표점 도달(dist={dist:.3f}) → 법선 회전")
                return self._rotate_to_odom_yaw(tyaw, "곡선 종료자세")
            theta_des = math.atan2(-ey, -ex)   # rear 가 목표를 향하는 body yaw
            herr = self._angdiff(theta_des, cth)
            v = -min(self._reverse_speed, max(0.02, self._curve_kv * dist))
            omega = self._clamp(self._recenter_kp * herr)
            # 발산 가드: 목표에서 멀어지기만 하면(조향 부호 의심) 정지하고 종료자세만 시도.
            if dist > min_dist + 0.05:
                self._stop()
                self.get_logger().warn("Curve: 목표서 멀어짐(조향 의심) → 곡선 중단, 법선 회전")
                return self._rotate_to_odom_yaw(tyaw, "곡선 종료자세")
            twist = Twist()
            twist.linear.x = v
            twist.angular.z = omega
            self._cmd_pub.publish(twist)
            if time.time() - last_log > 0.5:
                self.get_logger().info(
                    f"Curve: dist={dist:.3f} herr={math.degrees(herr):+.1f}deg v={v:.3f}")
                last_log = time.time()
            time.sleep(0.05)
        self._stop()
        self.get_logger().warn("Curve: timeout → 법선 회전")
        return self._rotate_to_odom_yaw(tyaw, "곡선 종료자세")

    def _rotate_to_odom_yaw(self, target_yaw: float, label: str = "") -> bool:
        """제자리 회전으로 odom yaw 를 target_yaw 로 맞춘다(P제어). best-effort True."""
        deadline = time.time() + self._recenter_to
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
            err = self._angdiff(target_yaw, od[2])
            if abs(err) < self._yaw_align_tol:
                self._stop()
                self.get_logger().info(
                    f"TwoMarkerNormal: {label} 완료(잔차={math.degrees(err):+.1f}deg)")
                return True
            twist = Twist()
            twist.angular.z = self._clamp(self._recenter_kp * err)
            self._cmd_pub.publish(twist)
            if time.time() - last_log > 0.5:
                self.get_logger().info(
                    f"TwoMarkerNormal: {label} 중(err={math.degrees(err):+.1f}deg)")
                last_log = time.time()
            time.sleep(0.05)
        self._stop()
        self.get_logger().warn(f"TwoMarkerNormal: {label} timeout")
        return True

    @staticmethod
    def _angdiff(a: float, b: float) -> float:
        """a-b 를 [-pi,pi] 로 정규화."""
        return math.atan2(math.sin(a - b), math.cos(a - b))

    @staticmethod
    def _wrap(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    def _reverse_straight_odom(self, dist: float) -> bool:
        """현재 헤딩(법선)을 odom 으로 유지하며 dist(m) 만큼 직진 후진. 거리 도달 시 True.

        수렴(횡오차 작음) 시 arc 를 건너뛰면 standby 에 머물러 두-마커가 안 잡히므로,
        법선방향으로 일정 거리 들어가 두-마커 검출 영역으로 진입할 때 쓴다. 시작 odom
        yaw 를 θ_ref 로 잡고 yaw_hold 로 곧게 후진한다(reverse_insert 와 동일 원리).
        """
        if dist <= 0.0:
            return True
        od0 = self._get_odom()
        if od0 is None:
            self.get_logger().error("ReverseStraight: odom 없음 → 생략")
            return False
        ox0, oy0, theta_ref = od0
        deadline = time.time() + self._arc_to
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
            ox, oy, oyaw = od
            traveled = math.hypot(ox - ox0, oy - oy0)
            if traveled >= dist:
                self._stop()
                self.get_logger().info(f"ReverseStraight: {traveled:.3f}/{dist:.3f}m 도달")
                return True
            yaw_drift = math.atan2(math.sin(oyaw - theta_ref),
                                   math.cos(oyaw - theta_ref))
            omega = -self._yaw_hold_kp * yaw_drift
            if time.time() - last_log > 0.5:
                self.get_logger().info(
                    f"ReverseStraight: {traveled:.3f}/{dist:.3f}m "
                    f"yaw_drift={math.degrees(yaw_drift):+.1f}deg"
                )
                last_log = time.time()
            twist = Twist()
            twist.linear.x = -self._reverse_speed
            twist.angular.z = self._clamp(omega)
            self._cmd_pub.publish(twist)
            time.sleep(0.05)
        self._stop()
        self.get_logger().warn(f"ReverseStraight: timeout ({dist:.3f}m 미달)")
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

    # ====================================================================== #
    # 검출
    # ====================================================================== #

    def _detect_aruco(self, frame, target_id: int):
        """IPPE_SQUARE solvePnP 기반 ArUco pose. (tvec[3], rvec[3]) 또는 None."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)
        if ids is None:
            return None

        for i, mid in enumerate(ids.flatten()):
            if mid != target_id:
                continue
            ok, rvec, tvec = cv2.solvePnP(
                self._marker_obj_pts,
                corners[i][0].astype(np.float64),
                self._cam_matrix,
                self._dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,   # 평면 정사각 마커 ambiguity 처리
            )
            if ok:
                return tvec.flatten(), rvec.flatten()

        return None

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
