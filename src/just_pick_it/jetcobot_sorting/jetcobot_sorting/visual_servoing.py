#!/usr/bin/env python3

import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from pymycobot.mycobot280 import MyCobot280


@dataclass
class Detection:
    cx: float
    cy: float
    w: float
    h: float
    score: float
    stamp: float


class JetcobotPixelServo(Node):
    def __init__(self):
        super().__init__("jetcobot_pixel_servo")

        # -------------------------
        # Robot parameters
        # -------------------------
        self.declare_parameter("port", "/dev/ttyJETCOBOT")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("speed", 15)
        self.declare_parameter("mode", 1)

        # -------------------------
        # Image parameters
        # -------------------------
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)

        # -------------------------
        # Servo parameters
        # -------------------------
        self.declare_parameter("pixel_deadband", 15.0)
        self.declare_parameter("max_step_mm", 8.0)
        self.declare_parameter("kx_mm_per_px", 0.04)
        self.declare_parameter("ky_mm_per_px", 0.04)
        self.declare_parameter("control_period", 0.15)
        self.declare_parameter("detection_timeout", 0.5)
        self.declare_parameter("motion_timeout", 2.0)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.speed = int(self.get_parameter("speed").value)
        self.mode = int(self.get_parameter("mode").value)

        self.image_width = float(self.get_parameter("image_width").value)
        self.image_height = float(self.get_parameter("image_height").value)

        self.img_cx = self.image_width / 2.0
        self.img_cy = self.image_height / 2.0

        self.pixel_deadband = float(self.get_parameter("pixel_deadband").value)
        self.max_step_mm = float(self.get_parameter("max_step_mm").value)
        self.kx = float(self.get_parameter("kx_mm_per_px").value)
        self.ky = float(self.get_parameter("ky_mm_per_px").value)
        self.control_period = float(self.get_parameter("control_period").value)
        self.detection_timeout = float(self.get_parameter("detection_timeout").value)
        self.motion_timeout = float(self.get_parameter("motion_timeout").value)

        # -------------------------
        # Robot init
        # -------------------------
        self.mc = MyCobot280(self.port, self.baudrate)
        self.mc.thread_lock = True

        try:
            # 최신 명령 우선 모드.
            # visual servoing에서는 오래된 명령 queue가 쌓이는 것보다 최신 명령 우선이 유리함.
            self.mc.set_fresh_mode(1)
        except Exception as e:
            self.get_logger().warn(f"set_fresh_mode failed: {e}")

        try:
            # 가능하면 vision tracking mode도 사용.
            self.mc.set_vision_mode(1)
        except Exception as e:
            self.get_logger().warn(f"set_vision_mode failed: {e}")

        # -------------------------
        # Internal states
        # -------------------------
        self.latest_detection: Optional[Detection] = None

        self.command_active = False
        self.last_command_time: Optional[float] = None

        self.servo_enabled = True
        self.centered_count = 0
        self.required_centered_count = 5

        # detection topic:
        # Float32MultiArray data = [cx, cy, bbox_w, bbox_h, score]
        self.det_sub = self.create_subscription(
            Float32MultiArray,
            "/target_bbox",
            self.detection_callback,
            10,
        )

        self.timer = self.create_timer(
            self.control_period,
            self.control_loop,
        )

        self.get_logger().info("Jetcobot pixel visual servo node started")
        self.get_logger().info(
            f"image center=({self.img_cx:.1f}, {self.img_cy:.1f}), "
            f"kx={self.kx}, ky={self.ky}, max_step={self.max_step_mm} mm"
        )

    def detection_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 5:
            self.get_logger().warn("Invalid detection msg. Expected [cx, cy, w, h, score]")
            return

        cx, cy, w, h, score = msg.data[:5]

        self.latest_detection = Detection(
            cx=float(cx),
            cy=float(cy),
            w=float(w),
            h=float(h),
            score=float(score),
            stamp=time.time(),
        )

    def control_loop(self):
        if not self.servo_enabled:
            return

        now = time.time()

        # 1. detection 유효성 확인
        det = self.latest_detection
        if det is None:
            self.get_logger().debug("No detection yet")
            return

        if now - det.stamp > self.detection_timeout:
            self.get_logger().warn("Detection timeout")
            return

        # 2. 로봇 이동 상태 확인
        try:
            moving = self.mc.is_moving()
        except Exception as e:
            self.get_logger().error(f"is_moving failed: {e}")
            self.safe_stop()
            return

        if moving == -1:
            self.get_logger().error("Robot moving state error")
            self.safe_stop()
            return

        # 3. 이전 명령이 아직 수행 중이면 새 명령을 보내지 않음
        if moving == 1:
            if self.last_command_time is not None:
                if now - self.last_command_time > self.motion_timeout:
                    self.get_logger().warn("Motion timeout during servoing")
                    self.command_active = False
            return

        # 여기부터 moving == 0, 즉 로봇이 정지 상태

        # 4. 픽셀 오차 계산
        error_u = det.cx - self.img_cx
        error_v = det.cy - self.img_cy

        abs_u = abs(error_u)
        abs_v = abs(error_v)

        self.get_logger().info(
            f"pixel error: u={error_u:.1f}, v={error_v:.1f}, "
            f"bbox=({det.w:.1f}, {det.h:.1f}), score={det.score:.2f}"
        )

        # 5. 중심에 충분히 들어왔으면 정지
        if abs_u < self.pixel_deadband and abs_v < self.pixel_deadband:
            self.centered_count += 1
            self.get_logger().info(
                f"Target centered: {self.centered_count}/{self.required_centered_count}"
            )

            if self.centered_count >= self.required_centered_count:
                self.get_logger().info("Visual servoing converged")
                self.servo_enabled = False
            return

        self.centered_count = 0

        # 6. 현재 로봇 좌표 읽기
        try:
            current_coords = self.mc.get_coords()
        except Exception as e:
            self.get_logger().error(f"get_coords failed: {e}")
            self.safe_stop()
            return

        if current_coords is None or len(current_coords) < 6:
            self.get_logger().error(f"Invalid current coords: {current_coords}")
            self.safe_stop()
            return

        x, y, z, rx, ry, rz = [float(v) for v in current_coords[:6]]

        # 7. 픽셀 오차 → mm 보정량 변환
        #
        # 주의:
        # 이 부호는 카메라 장착 방향에 따라 반드시 실험으로 맞춰야 함.
        #
        # 일반 이미지 좌표:
        # u: 오른쪽 +
        # v: 아래쪽 +
        #
        # 아래 mapping은 예시.
        # 만약 반대로 움직이면 dx, dy 부호를 각각 뒤집으면 됨.
        dx = -self.ky * error_v
        dy = -self.kx * error_u

        dx = self.clamp(dx, -self.max_step_mm, self.max_step_mm)
        dy = self.clamp(dy, -self.max_step_mm, self.max_step_mm)

        target_coords = [
            x + dx,
            y + dy,
            z,
            rx,
            ry,
            rz,
        ]

        self.get_logger().info(
            f"servo step: dx={dx:.2f} mm, dy={dy:.2f} mm -> target={target_coords}"
        )

        # 8. 비동기 이동 명령 전송
        try:
            self.mc.send_coords(
                target_coords,
                self.speed,
                mode=self.mode,
                _async=True,
            )
            self.command_active = True
            self.last_command_time = now
        except Exception as e:
            self.get_logger().error(f"send_coords failed: {e}")
            self.safe_stop()

    def safe_stop(self):
        self.get_logger().warn("Safe stop called")

        try:
            self.mc.stop()
        except Exception as e:
            self.get_logger().error(f"mc.stop failed: {e}")

        self.servo_enabled = False

    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min_value, min(value, max_value))


def main(args=None):
    rclpy.init(args=args)

    node = JetcobotPixelServo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt")
        node.safe_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()