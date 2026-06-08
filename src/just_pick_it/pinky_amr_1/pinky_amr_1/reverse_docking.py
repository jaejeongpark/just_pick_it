#!/usr/bin/env python3
"""
Reverse Docking - ArUco 시각 서보 후진 도킹

4단계:
  0단계: 마커 탐색 및 rvec[1] 기반 yaw 정렬
  1단계: tvec[0] 기반 횡방향 pre-alignment (전진 아크)
  2단계: tvec + rvec PID 후진 (부호 반전), tvec[2] <= dock_switch_distance 시 종료
  3단계: 노란 주차라인 중심 추종 후진 + 파란 정지선 감지 시 정지

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


class ReverseDocking(Node):
    def __init__(self, emergency_latch=None):
        super().__init__("reverse_docking")

        # 비상 정지 래치. state_manager 가 공유 인스턴스를 주입한다.
        # 단독 실행(__main__)이면 자체 생성한다.
        if emergency_latch is None:
            from pinky_amr_1.emergency_latch import EmergencyLatch
            emergency_latch = EmergencyLatch()
        self._emergency = emergency_latch

        # ArUco
        self.declare_parameter("aruco_marker_dict", 0)
        self.declare_parameter("marker_size_m", 0.10)

        # 카메라 (비전 담당 캘리브레이션 후 채워줌, wide lens 1080p placeholder)
        self.declare_parameter("camera_matrix", [
            777.0, 0.0, 960.0,
            0.0, 777.0, 540.0,
            0.0, 0.0, 1.0,
        ])
        self.declare_parameter("dist_coeffs", [0.0, 0.0, 0.0, 0.0, 0.0])

        # 거리 임계값
        self.declare_parameter("dock_switch_distance", 0.20)   # Phase 2→3 전환 (m)

        # Phase 0: yaw 정렬
        self.declare_parameter("rotate_kp", 1.5)
        self.declare_parameter("rotate_ki", 0.0)
        self.declare_parameter("rotate_kd", 0.1)
        self.declare_parameter("rotate_thresh_rad", 0.05)

        # Phase 1: 횡방향 pre-alignment
        self.declare_parameter("prealign_kp", 1.2)
        self.declare_parameter("prealign_thresh_m", 0.03)

        # Phase 2: ArUco 후진
        self.declare_parameter("lat_kp", 1.0)
        self.declare_parameter("lat_ki", 0.0)
        self.declare_parameter("lat_kd", 0.05)
        self.declare_parameter("yaw_kp", 0.8)
        self.declare_parameter("yaw_ki", 0.0)
        self.declare_parameter("yaw_kd", 0.05)
        self.declare_parameter("reverse_speed", 0.05)

        # Phase 3: 주차라인 + 정지선
        self.declare_parameter("lane_kp", 0.003)
        self.declare_parameter("final_speed", 0.03)

        # 파란 정지선 (HSV)
        self.declare_parameter("tape_hsv_lower", [100, 100, 50])
        self.declare_parameter("tape_hsv_upper", [130, 255, 255])
        self.declare_parameter("tape_coverage_thresh", 0.5)

        # 공통
        self.declare_parameter("max_angular_vel", 0.4)
        self.declare_parameter("aruco_timeout_sec", 30.0)
        self.declare_parameter("camera_topic", "camera/image_raw")

        # 파라미터 로드
        dict_id = self.get_parameter("aruco_marker_dict").value
        self._marker_size = self.get_parameter("marker_size_m").value

        cam = self.get_parameter("camera_matrix").value
        self._cam_matrix = np.array(cam, dtype=np.float64).reshape(3, 3)
        self._dist_coeffs = np.array(
            self.get_parameter("dist_coeffs").value, dtype=np.float64
        )

        self._switch_dist     = self.get_parameter("dock_switch_distance").value
        self._rotate_thresh   = self.get_parameter("rotate_thresh_rad").value
        self._prealign_kp     = self.get_parameter("prealign_kp").value
        self._prealign_thresh = self.get_parameter("prealign_thresh_m").value
        self._reverse_speed   = self.get_parameter("reverse_speed").value
        self._lane_kp         = self.get_parameter("lane_kp").value
        self._final_speed     = self.get_parameter("final_speed").value
        self._tape_lower      = tuple(int(v) for v in self.get_parameter("tape_hsv_lower").value)
        self._tape_upper      = tuple(int(v) for v in self.get_parameter("tape_hsv_upper").value)
        self._tape_thresh     = self.get_parameter("tape_coverage_thresh").value
        self._max_ang         = self.get_parameter("max_angular_vel").value
        self._timeout         = self.get_parameter("aruco_timeout_sec").value

        # ArUco 검출기
        aruco_dict   = cv2.aruco.getPredefinedDictionary(dict_id)
        aruco_params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        # solvePnP 용 마커 3D 기준점 (마커 중심 원점, XY 평면)
        h = self._marker_size / 2.0
        self._marker_obj_pts = np.array([
            [-h,  h, 0.0],
            [ h,  h, 0.0],
            [ h, -h, 0.0],
            [-h, -h, 0.0],
        ], dtype=np.float64)

        # PID
        self._rotate_pid = PID(
            self.get_parameter("rotate_kp").value,
            self.get_parameter("rotate_ki").value,
            self.get_parameter("rotate_kd").value,
        )
        self._lat_pid = PID(
            self.get_parameter("lat_kp").value,
            self.get_parameter("lat_ki").value,
            self.get_parameter("lat_kd").value,
        )
        self._yaw_pid = PID(
            self.get_parameter("yaw_kp").value,
            self.get_parameter("yaw_ki").value,
            self.get_parameter("yaw_kd").value,
        )

        # 카메라
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_frame = None

        cam_topic = self.get_parameter("camera_topic").value
        self.create_subscription(Image, cam_topic, self._image_cb, 10)

        # 퍼블리셔. 노드 namespace 가 'picky1' 이면 자동으로 /picky1/cmd_vel,
        # /picky1/initialpose 가 된다 (AMCL 도 같은 namespace 안에서 띄운다는 가정).
        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "initialpose", 10
        )

        self.get_logger().info("ReverseDocking ready.")

    # ------------------------------------------------------------------ #
    # 외부 인터페이스
    # ------------------------------------------------------------------ #

    def reverse_dock(
        self,
        marker_id: int,
        dock_map_x: float,
        dock_map_y: float,
        dock_map_yaw: float,
    ) -> bool:
        """ArUco 기반 4단계 후진 도킹. 성공 시 True."""
        self.get_logger().info(
            f"reverse_dock: marker={marker_id}, "
            f"target=({dock_map_x:.3f}, {dock_map_y:.3f}, "
            f"{math.degrees(dock_map_yaw):.1f}deg)"
        )
        self._rotate_pid.reset()
        self._lat_pid.reset()
        self._yaw_pid.reset()

        phases = [
            ("Phase 0", lambda: self._phase0_rotate_to_marker(marker_id)),
            ("Phase 1", lambda: self._phase1_lateral_prealign(marker_id)),
            ("Phase 2", lambda: self._phase2_aruco_reverse(marker_id)),
            ("Phase 3", self._phase3_final_approach),
        ]
        for name, fn in phases:
            if not fn():
                self._stop()
                self.get_logger().error(f"reverse_dock: FAILED at {name}")
                return False

        self._stop()
        self.get_logger().info("reverse_dock: SUCCESS")
        self._publish_pose_correction(dock_map_x, dock_map_y, dock_map_yaw)
        return True

    # ------------------------------------------------------------------ #
    # 0단계: 마커 탐색 + yaw 정렬
    # ------------------------------------------------------------------ #

    def _phase0_rotate_to_marker(self, marker_id: int) -> bool:
        """마커 미검출 시 탐색 회전, 검출 후 rvec[1] 기반 yaw 정렬."""
        deadline = time.time() + self._timeout

        while time.time() < deadline:
            if self._emergency.is_stopped():
                self._stop()
                deadline += self._wait_if_paused()
                continue

            frame = self._get_latest_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            result = self._detect_aruco(frame, marker_id)

            if result is None:
                twist = Twist()
                twist.angular.z = 0.3
                self._cmd_pub.publish(twist)
                time.sleep(0.05)
                continue

            _, rvec = result
            yaw_err = float(rvec[1])

            if abs(yaw_err) <= self._rotate_thresh:
                self._stop()
                self.get_logger().info(f"Phase 0: done — rvec[1]={yaw_err:.3f} rad")
                return True

            twist = Twist()
            twist.angular.z = self._clamp(self._rotate_pid.compute(-yaw_err))
            self._cmd_pub.publish(twist)
            time.sleep(0.05)

        self._stop()
        self.get_logger().warn("Phase 0: timeout")
        return False

    # ------------------------------------------------------------------ #
    # 1단계: 횡방향 pre-alignment
    # ------------------------------------------------------------------ #

    def _phase1_lateral_prealign(self, marker_id: int) -> bool:
        """tvec[0] 허용 범위 이내가 될 때까지 아크로 횡방향 보정.

        기본은 후진 아크. tvec[2] <= dock_switch_distance 에 도달하면
        전진 아크로 전환하여 벽 충돌 없이 보정을 완료한다.
        """
        deadline = time.time() + 10.0

        while time.time() < deadline:
            if self._emergency.is_stopped():
                self._stop()
                deadline += self._wait_if_paused()
                continue

            frame = self._get_latest_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            result = self._detect_aruco(frame, marker_id)
            if result is None:
                time.sleep(0.05)
                continue

            tvec, _ = result
            lat_err = float(tvec[0])
            dist    = float(tvec[2])

            if abs(lat_err) <= self._prealign_thresh:
                self._stop()
                self.get_logger().info(f"Phase 1: done — tvec[0]={lat_err:.3f} m")
                return True

            if dist <= self._switch_dist:
                # 너무 가까워졌으면 전진 아크로 전환 (벽 충돌 방지)
                lin = 0.04
                ang = self._clamp(self._prealign_kp * lat_err)
            else:
                # 기본: 후진 아크 (부호 반전)
                lin = -0.04
                ang = self._clamp(-self._prealign_kp * lat_err)

            twist = Twist()
            twist.linear.x  = lin
            twist.angular.z = ang
            self._cmd_pub.publish(twist)
            time.sleep(0.05)

        self._stop()
        self.get_logger().warn("Phase 1: timeout")
        return False

    # ------------------------------------------------------------------ #
    # 2단계: ArUco 기반 후진
    # ------------------------------------------------------------------ #

    def _phase2_aruco_reverse(self, marker_id: int) -> bool:
        """tvec[0] + rvec[1] PID 후진. tvec[2] <= dock_switch_distance 시 종료."""
        deadline = time.time() + 30.0

        while time.time() < deadline:
            if self._emergency.is_stopped():
                self._stop()
                deadline += self._wait_if_paused()
                continue

            frame = self._get_latest_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            result = self._detect_aruco(frame, marker_id)

            if result is None:
                # 검출 실패: 각도 보정 없이 저속 직진 후진
                twist = Twist()
                twist.linear.x = -self._reverse_speed * 0.5
                self._cmd_pub.publish(twist)
                time.sleep(0.05)
                continue

            tvec, rvec = result
            dist = float(tvec[2])

            if dist <= self._switch_dist:
                self._stop()
                self.get_logger().info(f"Phase 2: done — dist={dist:.3f} m")
                return True

            # 후진 시 부호 반전: 마커가 우측(tvec[0] > 0)이면 좌회전(-angular.z)
            ang_cmd = self._clamp(
                -(self._lat_pid.compute(float(tvec[0]))
                  + self._yaw_pid.compute(float(rvec[1])))
            )
            speed = min(self._reverse_speed, dist * 0.15 + 0.02)

            twist = Twist()
            twist.linear.x = -speed
            twist.angular.z = ang_cmd
            self._cmd_pub.publish(twist)
            time.sleep(0.05)

        self._stop()
        self.get_logger().warn("Phase 2: timeout")
        return False

    # ------------------------------------------------------------------ #
    # 3단계: 주차라인 기반 최종 접근
    # ------------------------------------------------------------------ #

    def _phase3_final_approach(self) -> bool:
        """노란 주차라인 중심 추종 후진 + 파란 정지선 감지 시 정지."""
        deadline = time.time() + 15.0

        while time.time() < deadline:
            if self._emergency.is_stopped():
                self._stop()
                deadline += self._wait_if_paused()
                continue

            frame = self._get_latest_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            if self._detect_tape(frame):
                self._stop()
                self.get_logger().info("Phase 3: stop line detected — docking complete")
                return True

            lane_err = self._detect_lane_center(frame)
            ang_cmd = self._clamp(-self._lane_kp * lane_err) if lane_err is not None else 0.0

            twist = Twist()
            twist.linear.x = -self._final_speed
            twist.angular.z = ang_cmd
            self._cmd_pub.publish(twist)
            time.sleep(0.05)

        self._stop()
        self.get_logger().warn("Phase 3: timeout")
        return False

    # ------------------------------------------------------------------ #
    # 검출
    # ------------------------------------------------------------------ #

    def _detect_aruco(self, frame, target_id: int):
        """solvePnP 기반 ArUco pose 추정. (tvec[3], rvec[3]) 또는 None."""
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
            )
            if ok:
                return tvec.flatten(), rvec.flatten()

        return None

    def _detect_lane_center(self, frame) -> float | None:
        """노란 주차라인 좌/우 중심 평균으로 횡방향 오차(px) 반환."""
        h, w = frame.shape[:2]
        roi = frame[h * 2 // 3 :, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        yellow = cv2.inRange(hsv, (20, 100, 100), (35, 255, 255))

        left_pts  = cv2.findNonZero(yellow[:, : w // 2])
        right_pts = cv2.findNonZero(yellow[:, w // 2 :])
        if left_pts is None or right_pts is None:
            return None

        left_cx  = float(np.mean(left_pts[:, 0, 0]))
        right_cx = float(np.mean(right_pts[:, 0, 0])) + w // 2
        return (left_cx + right_cx) / 2.0 - w / 2.0

    def _detect_tape(self, frame) -> bool:
        """파란 정지선이 하단 ROI 가로의 tape_coverage_thresh 이상이면 True."""
        h, w = frame.shape[:2]
        roi = frame[h * 2 // 3 :, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._tape_lower, self._tape_upper)
        row_coverage = np.sum(mask > 0, axis=1) / w
        return bool(np.any(row_coverage > self._tape_thresh))

    # ------------------------------------------------------------------ #
    # 위치 보정
    # ------------------------------------------------------------------ #

    def _publish_pose_correction(self, x: float, y: float, yaw: float):
        """알려진 dock 절대 좌표를 /initialpose 로 발행해 AMCL 재초기화."""
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

    # ------------------------------------------------------------------ #
    # 유틸
    # ------------------------------------------------------------------ #

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
        """비상 정지 중이면 재개될 때까지 제자리에서 대기한다(pause-continue).

        대기 동안 0 속도 명령을 재발행해 도킹 중 로봇을 확실히 멈춰둔다.
        반환값은 대기 시간(초)으로, 각 phase 의 deadline 을 그만큼 미뤄
        비상 정지 시간이 phase timeout 을 잡아먹지 않게 한다.
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
