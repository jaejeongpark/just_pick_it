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
import time
import threading

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
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
        self.declare_parameter("aruco_marker_dict", 0)        # DICT_4X4_50
        self.declare_parameter("marker_size_m", 0.10)

        # 마커 월드 좌표 (marker_id 별 병렬 배열). 가로벽에 도크를 바라보게 부착.
        # 법선은 -y(원점 쪽) 고정이라 yaw 따로 안 받는다.
        self.declare_parameter("marker_ids", [0, 1])
        self.declare_parameter("marker_world_x", [0.07, 0.28])
        self.declare_parameter("marker_world_y", [0.655, 0.655])

        # ── 카메라 (비전팀 캘리브레이션 실측값, 1280x720) ────────────────
        self.declare_parameter("camera_matrix", [
            777.0, 0.0, 960.0,
            0.0, 777.0, 540.0,
            0.0, 0.0, 1.0,
        ])
        self.declare_parameter("dist_coeffs", [0.0, 0.0, 0.0, 0.0, 0.0])
        # base_link(중심) 에서 카메라가 전방(+x_body=+y_world)으로 떨어진 거리(m).
        # 마커 거리로 로봇 base 의 world y 를 추정할 때 보정에 쓴다(URDF 기준 근사).
        self.declare_parameter("camera_forward_offset_m", 0.05)

        # ── 시작 coarse 정렬 (마커 상대) ────────────────────────────────
        self.declare_parameter("acquire_rotate_speed", 0.3)   # 마커 탐색 회전(rad/s)
        self.declare_parameter("marker_lat_kp", 1.0)          # tvec[0] coarse 횡 게인
        self.declare_parameter("marker_yaw_kp", 0.8)          # rvec[1] coarse yaw 게인

        # ── 정밀 정렬 (노란 라인) ───────────────────────────────────────
        self.declare_parameter("lane_lat_kp", 0.004)          # lateral_px 게인
        self.declare_parameter("lane_yaw_kp", 0.8)            # yaw_rad 게인
        self.declare_parameter("lane_yellow_lower", [20, 100, 100])  # HSV
        self.declare_parameter("lane_yellow_upper", [35, 255, 255])
        self.declare_parameter("lane_min_pixels", 150)        # 한쪽 라인 최소 픽셀(conf 0)
        self.declare_parameter("lane_full_pixels", 1200)      # 이 이상이면 conf 1

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
        dict_id = self.get_parameter("aruco_marker_dict").value
        self._marker_size = self.get_parameter("marker_size_m").value

        ids = list(self.get_parameter("marker_ids").value)
        mwx = list(self.get_parameter("marker_world_x").value)
        mwy = list(self.get_parameter("marker_world_y").value)
        # marker_id -> (world_x, world_y)
        self._marker_world = {int(i): (float(x), float(y))
                              for i, x, y in zip(ids, mwx, mwy)}

        cam = self.get_parameter("camera_matrix").value
        self._cam_matrix = np.array(cam, dtype=np.float64).reshape(3, 3)
        self._dist_coeffs = np.array(
            self.get_parameter("dist_coeffs").value, dtype=np.float64
        )
        self._cam_fwd = self.get_parameter("camera_forward_offset_m").value

        self._acquire_rot   = self.get_parameter("acquire_rotate_speed").value
        self._marker_lat_kp = self.get_parameter("marker_lat_kp").value
        self._marker_yaw_kp = self.get_parameter("marker_yaw_kp").value

        self._lane_lat_kp = self.get_parameter("lane_lat_kp").value
        self._lane_yaw_kp = self.get_parameter("lane_yaw_kp").value
        self._yellow_lo = tuple(int(v) for v in self.get_parameter("lane_yellow_lower").value)
        self._yellow_hi = tuple(int(v) for v in self.get_parameter("lane_yellow_upper").value)
        self._lane_min_px  = int(self.get_parameter("lane_min_pixels").value)
        self._lane_full_px = int(self.get_parameter("lane_full_pixels").value)

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

        # ── PID (정밀 정렬용 라인 lateral/yaw) ──────────────────────────
        self._lane_lat_pid = PID(self._lane_lat_kp, 0.0, 0.0005)
        self._lane_yaw_pid = PID(self._lane_yaw_kp, 0.0, 0.05)

        # ── 카메라/퍼블리셔 ─────────────────────────────────────────────
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_frame = None

        cam_topic = self.get_parameter("camera_topic").value
        self.create_subscription(Image, cam_topic, self._image_cb, 10)

        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "initialpose", 10
        )

        self.get_logger().info("ReverseDocking ready (marker-depth + yellow-lane).")

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
        self._lane_lat_pid.reset()
        self._lane_yaw_pid.reset()

        # 1) 마커 획득 (보일 때까지 탐색 회전)
        if not self._acquire_marker(marker_id):
            self._stop()
            self.get_logger().error("reverse_dock: FAILED — 마커 미획득")
            return False

        # 2) 후진 삽입 (라인 정밀 정렬 블렌딩, 마커 깊이로 정지)
        #    마커가 도크 바로 위가 아닐 수 있어(예: dock1 마커x 0.07 vs 도크x 0.11),
        #    coarse 단계의 횡 목표는 (marker_x - dock_x) 만큼 오프셋을 준다.
        lat_offset = marker_x - dock_map_x
        if not self._reverse_insert(marker_id, marker_y, dock_map_y, lat_offset):
            self._stop()
            self.get_logger().error("reverse_dock: FAILED — 삽입 단계")
            return False

        # 3) 정지 + 안정화 + 위치 보정
        self._stop()
        time.sleep(self._settle)
        self.get_logger().info("reverse_dock: SUCCESS")
        self._publish_pose_correction(dock_map_x, dock_map_y, dock_map_yaw)
        return True

    # ====================================================================== #
    # 단계 A: 마커 획득
    # ====================================================================== #

    def _acquire_marker(self, marker_id: int) -> bool:
        """마커가 검출될 때까지 제자리 탐색 회전. 검출되면 True."""
        deadline = time.time() + self._acquire_to
        while time.time() < deadline:
            if self._emergency.is_stopped():
                self._stop()
                deadline += self._wait_if_paused()
                continue

            frame = self._get_latest_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            if self._detect_aruco(frame, marker_id) is not None:
                self._stop()
                self.get_logger().info("Acquire: 마커 검출")
                return True

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
        lat_offset: float,
    ) -> bool:
        """노란 라인으로 lateral+yaw 정밀 정렬하며 후진. 마커 깊이가 dock_y 도달 시 정지.

        ω = conf·(라인 PID) + (1-conf)·(마커 coarse 정렬)
          - conf 0(라인 부족, 시작): 마커 tvec[0]/rvec[1] coarse
          - conf 1(라인 충분, 삽입): 라인 대칭선 lateral+yaw 정밀
        v = -reverse_speed (느린 후진)
        정지: 로봇 world y(= marker_y - 마커거리 - 카메라 전방오프셋) <= dock_y
        """
        deadline = time.time() + self._insert_to
        lost = 0

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
                # 마커는 상시 가시 전제. 잠깐 놓치면 저속 직진 후진 유지, 길면 실패.
                lost += 1
                if lost > 40:   # 약 2s
                    self._stop()
                    self.get_logger().warn("Insert: 마커 장기 미검출")
                    return False
                twist = Twist()
                twist.linear.x = -self._reverse_speed * 0.5
                self._cmd_pub.publish(twist)
                time.sleep(0.05)
                continue
            lost = 0

            tvec, rvec = marker
            marker_dist = float(tvec[2])

            # 깊이 기반 정지: 로봇 base 의 world y 추정
            robot_y = marker_world_y - marker_dist - self._cam_fwd
            if robot_y <= dock_y + self._stop_tol:
                self._stop()
                self.get_logger().info(
                    f"Insert: 정지 — robot_y≈{robot_y:.3f} <= dock_y {dock_y:.3f}"
                )
                return True

            # ── 정렬 ω 계산 (라인 신뢰도로 블렌딩) ──────────────────────
            lane = self._detect_lane(frame)
            marker_omega = self._marker_coarse_omega(tvec, rvec, lat_offset)

            if lane is not None and lane.conf > 0.0:
                lane_omega = -(self._lane_lat_pid.compute(lane.lateral_px)
                               + self._lane_yaw_pid.compute(lane.yaw_rad))
                w = lane.conf
                omega = w * lane_omega + (1.0 - w) * marker_omega
            else:
                omega = marker_omega

            twist = Twist()
            twist.linear.x = -self._reverse_speed
            twist.angular.z = self._clamp(omega)
            self._cmd_pub.publish(twist)
            time.sleep(0.05)

        self._stop()
        self.get_logger().warn("Insert: timeout")
        return False

    def _marker_coarse_omega(self, tvec, rvec, lat_offset: float) -> float:
        """마커 상대 coarse 정렬 ω. 라인이 안 보이는 시작 구간용.

        후진 부호 반전: 마커가 (목표보다) 우측이면 좌회전.
        횡 목표는 lat_offset(= marker_x - dock_x)/거리 만큼 카메라 횡으로 둔다.
        """
        dist = max(float(tvec[2]), 0.1)
        # 도크에 맞춰 정렬됐을 때 마커가 카메라 광축에서 떨어져 보여야 하는 횡(rad 근사).
        target_lat = lat_offset / dist
        lat_err = float(tvec[0]) / dist - target_lat   # 카메라 기준 횡 오차(rad 근사)
        yaw_err = float(rvec[1])                        # 마커 평면 yaw
        return -(self._marker_lat_kp * lat_err + self._marker_yaw_kp * yaw_err)

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

    def _detect_lane(self, frame) -> LaneResult | None:
        """노란 주차라인 2개 → 채널 대칭선의 (lateral_px, yaw_rad, conf).

        하단 ROI 에서 좌/우 절반의 노란 픽셀을 각각 직선 피팅해 두 라인을 얻고,
        그 중심선(대칭선)의 이미지 중심 대비 수평 오프셋과 수직 대비 기울기를 낸다.
        """
        h, w = frame.shape[:2]
        roi = frame[h // 2:, :]              # 하단 절반(바닥)
        rh = roi.shape[0]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        yellow = cv2.inRange(hsv, self._yellow_lo, self._yellow_hi)

        left = self._fit_line(yellow[:, : w // 2], 0)
        right = self._fit_line(yellow[:, w // 2:], w // 2)
        if left is None or right is None:
            return None
        (lx_bottom, l_ang, l_px) = left
        (rx_bottom, r_ang, r_px) = right

        # 대칭선: 두 라인의 평균(중심선). 하단(로봇 가까운 쪽) 기준점 + 평균 각.
        center_x_bottom = (lx_bottom + rx_bottom) / 2.0
        center_ang = (l_ang + r_ang) / 2.0
        lateral_px = center_x_bottom - w / 2.0   # +면 중심이 우측
        yaw_rad = center_ang                     # 수직 대비 기울기(우측+)

        # 신뢰도: 양쪽 라인 픽셀 수가 충분할수록 1 에 가까움.
        m = min(l_px, r_px)
        conf = (m - self._lane_min_px) / max(self._lane_full_px - self._lane_min_px, 1)
        conf = float(max(0.0, min(1.0, conf)))
        if conf <= 0.0:
            return None
        return LaneResult(lateral_px, yaw_rad, conf)

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
        with self._lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

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
