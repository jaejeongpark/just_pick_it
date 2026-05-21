#!/usr/bin/env python3
"""
Aruco Parking
배치: [주차 지점] --- [Standby Zone] --- [마커]

주차 3단계:
  0단계: 마커 방향으로 제자리 회전 (주차 축 정렬)
  1단계: 주차 지점 approach_distance(6cm) 앞 보정점까지 후진 코너링
  2단계: 마커 각도 보정하며 주차 지점까지 저속 직진 후진

aruco_dock(marker_id, parking_x, parking_y) 메서드를 task_manager가 직접 호출.
"""

import math
import time
import threading

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener


def quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


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


class ArucoParking(Node):
    def __init__(self):
        super().__init__("aruco_parking")

        self.declare_parameter("aruco_marker_dict", 0)          # DICT_4X4_50
        self.declare_parameter("approach_distance", 0.06)       # 보정점: 주차 지점에서 6cm
        self.declare_parameter("stop_distance", 0.05)           # 최종 정지 거리 (m)
        self.declare_parameter("centering_threshold_px", 20)    # 마커 중앙 정렬 허용 오차 (px)
        self.declare_parameter("angle_kp", 0.8)
        self.declare_parameter("angle_ki", 0.0)
        self.declare_parameter("angle_kd", 0.1)
        self.declare_parameter("max_linear_vel", 0.08)
        self.declare_parameter("max_angular_vel", 0.4)
        self.declare_parameter("aruco_timeout_sec", 30.0)
        self.declare_parameter("camera_topic", "/camera/image_raw")

        dict_id = self.get_parameter("aruco_marker_dict").value
        self._approach_dist = self.get_parameter("approach_distance").value
        self._stop_dist = self.get_parameter("stop_distance").value
        self._center_thresh = self.get_parameter("centering_threshold_px").value
        self._max_lin = self.get_parameter("max_linear_vel").value
        self._max_ang = self.get_parameter("max_angular_vel").value
        self._timeout = self.get_parameter("aruco_timeout_sec").value

        self._aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self._aruco_params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._aruco_params)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_frame = None
        self._cur_x = 0.0
        self._cur_y = 0.0
        self._cur_yaw = 0.0

        cam_topic = self.get_parameter("camera_topic").value
        self.create_subscription(Image, cam_topic, self._image_cb, 10)

        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self.create_timer(0.05, self._update_pose)

        self._angle_pid = PID(
            self.get_parameter("angle_kp").value,
            self.get_parameter("angle_ki").value,
            self.get_parameter("angle_kd").value,
        )

        self.get_logger().info("ArucoParking ready.")

    # ------------------------------------------------------------------ #
    # 외부 인터페이스
    # ------------------------------------------------------------------ #

    def aruco_dock(
        self,
        marker_id: int,
        parking_x: float,
        parking_y: float,
        stop_distance: float | None = None,
    ) -> bool:
        """
        [주차 지점 parking_x/y] --- [현재 위치] --- [marker_id 마커]
        배치에서 3단계 후진 주차 수행. 성공 시 True.
        """
        if stop_distance is None:
            stop_distance = self._stop_dist

        self.get_logger().info(
            f"aruco_dock: marker={marker_id}, target=({parking_x:.3f},{parking_y:.3f})"
        )
        self._angle_pid.reset()

        if not self._phase0_rotate_to_marker(marker_id):
            self._stop_robot()
            return False

        if not self._phase1_curved_reverse(parking_x, parking_y):
            self._stop_robot()
            return False

        if not self._phase2_straight_reverse(marker_id, parking_x, parking_y, stop_distance):
            self._stop_robot()
            return False

        self.get_logger().info("aruco_dock: SUCCESS")
        self._publish_position_correction()
        return True

    # ------------------------------------------------------------------ #
    # 0단계: 마커 방향 제자리 회전
    # ------------------------------------------------------------------ #

    def _phase0_rotate_to_marker(self, marker_id: int) -> bool:
        """마커가 화면 중앙 centering_threshold_px 이내에 올 때까지 회전."""
        deadline = time.time() + self._timeout
        no_detect_streak = 0

        while time.time() < deadline:
            frame = self._get_latest_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            result = self._detect_marker(frame, marker_id)

            if result is None:
                no_detect_streak += 1
                # 마커 탐색: 천천히 회전
                twist = Twist()
                twist.angular.z = 0.3
                self._cmd_pub.publish(twist)
                time.sleep(0.05)
                continue

            no_detect_streak = 0
            cx_err, _ = result

            if abs(cx_err) <= self._center_thresh:
                self._stop_robot()
                self.get_logger().info("Phase 0: marker centered")
                return True

            ang_cmd = self._angle_pid.compute(-cx_err)
            ang_cmd = max(min(ang_cmd, self._max_ang), -self._max_ang)
            twist = Twist()
            twist.angular.z = ang_cmd
            self._cmd_pub.publish(twist)
            time.sleep(0.05)

        self._stop_robot()
        self.get_logger().warn("Phase 0: timeout")
        return False

    # ------------------------------------------------------------------ #
    # 1단계: 보정점까지 후진 코너링
    # ------------------------------------------------------------------ #

    def _phase1_curved_reverse(self, parking_x: float, parking_y: float) -> bool:
        """
        주차 지점에서 approach_distance 앞의 보정점까지
        로봇 후방이 보정점을 향하도록 조향하며 후진.
        """
        deadline = time.time() + 30.0
        KP = 1.2
        ARRIVE_THRESH = 0.02  # 2cm 이내면 도달 판정

        while time.time() < deadline:
            with self._lock:
                cur_x = self._cur_x
                cur_y = self._cur_y
                cur_yaw = self._cur_yaw

            dist_to_park = math.hypot(parking_x - cur_x, parking_y - cur_y)

            # 이미 보정점 이내면 완료
            if dist_to_park <= self._approach_dist + ARRIVE_THRESH:
                self._stop_robot()
                self.get_logger().info("Phase 1: alignment point reached")
                return True

            # 보정점 = 주차 지점에서 approach_dist만큼 로봇 방향으로 이격
            dx_rp = (parking_x - cur_x) / dist_to_park
            dy_rp = (parking_y - cur_y) / dist_to_park
            align_x = parking_x - self._approach_dist * dx_rp
            align_y = parking_y - self._approach_dist * dy_rp

            dx_a = align_x - cur_x
            dy_a = align_y - cur_y
            dist_to_align = math.hypot(dx_a, dy_a)

            if dist_to_align <= ARRIVE_THRESH:
                self._stop_robot()
                self.get_logger().info("Phase 1: alignment point reached")
                return True

            # 후진 조향: 로봇 후방(cur_yaw + π)이 보정점을 향하도록
            target_dir = math.atan2(dy_a, dx_a)
            angle_err = normalize_angle(target_dir - cur_yaw - math.pi)

            ang_cmd = max(min(KP * angle_err, self._max_ang), -self._max_ang)
            speed = min(self._max_lin, dist_to_align * 0.5 + 0.03)

            twist = Twist()
            twist.linear.x = -speed
            twist.angular.z = ang_cmd
            self._cmd_pub.publish(twist)
            time.sleep(0.05)

        self._stop_robot()
        self.get_logger().warn("Phase 1: timeout")
        return False

    # ------------------------------------------------------------------ #
    # 2단계: 직진 후진 주차
    # ------------------------------------------------------------------ #

    def _phase2_straight_reverse(
        self,
        marker_id: int,
        parking_x: float,
        parking_y: float,
        stop_distance: float,
    ) -> bool:
        """마커 cx_err로 각도 보정하며 주차 지점까지 저속 직진 후진."""
        deadline = time.time() + 15.0

        while time.time() < deadline:
            with self._lock:
                cur_x = self._cur_x
                cur_y = self._cur_y

            dist = math.hypot(parking_x - cur_x, parking_y - cur_y)
            if dist <= stop_distance:
                self._stop_robot()
                self.get_logger().info(f"Phase 2: parked — dist={dist:.3f}m")
                return True

            # 마커로 각도 보정
            ang_cmd = 0.0
            frame = self._get_latest_frame()
            if frame is not None:
                result = self._detect_marker(frame, marker_id)
                if result is not None:
                    cx_err, _ = result
                    ang_cmd = self._angle_pid.compute(-cx_err)
                    ang_cmd = max(min(ang_cmd, self._max_ang), -self._max_ang)

            speed = min(self._max_lin * 0.6, dist * 0.4 + 0.02)
            twist = Twist()
            twist.linear.x = -speed
            twist.angular.z = ang_cmd
            self._cmd_pub.publish(twist)
            time.sleep(0.05)

        self._stop_robot()
        self.get_logger().warn("Phase 2: timeout")
        return False

    # ------------------------------------------------------------------ #
    # 마커 검출
    # ------------------------------------------------------------------ #

    def _detect_marker(self, frame, target_id: int):
        """반환값: (cx_err_px, distance_m) 또는 None"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)

        if ids is None:
            return None

        for i, mid in enumerate(ids.flatten()):
            if mid != target_id:
                continue
            c = corners[i][0]
            cx = float(np.mean(c[:, 0]))
            img_cx = frame.shape[1] / 2.0
            cx_err = cx - img_cx
            side_px = float(np.linalg.norm(c[0] - c[1]))
            # 초점거리 약 500px, 마커 크기 0.05m 기준 휴리스틱
            distance_m = (500.0 * 0.05) / max(side_px, 1.0)
            return cx_err, distance_m

        return None

    # ------------------------------------------------------------------ #
    # 위치 보정
    # ------------------------------------------------------------------ #

    def _publish_position_correction(self):
        try:
            trans = self._tf_buffer.lookup_transform("map", "base_link", Time())
            t = trans.transform
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.pose.position.x = t.translation.x
            msg.pose.pose.position.y = t.translation.y
            msg.pose.pose.orientation = t.rotation
            msg.pose.covariance[0] = 0.01
            msg.pose.covariance[7] = 0.01
            msg.pose.covariance[35] = 0.005
            self._init_pose_pub.publish(msg)
            self.get_logger().info("Position correction published to /initialpose")
        except Exception as e:
            self.get_logger().warn(f"Position correction failed: {e}")

    # ------------------------------------------------------------------ #
    # 유틸
    # ------------------------------------------------------------------ #

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

    def _update_pose(self):
        try:
            trans = self._tf_buffer.lookup_transform("map", "base_link", Time())
            t = trans.transform
            with self._lock:
                self._cur_x = t.translation.x
                self._cur_y = t.translation.y
                self._cur_yaw = quat_to_yaw(t.rotation)
        except Exception:
            pass

    def _stop_robot(self):
        self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ArucoParking()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
