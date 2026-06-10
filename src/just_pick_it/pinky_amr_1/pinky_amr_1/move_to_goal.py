#!/usr/bin/env python3
"""
Move to Goal
- 1단계: Nav2 NavigateToPose로 목표 근처까지 이동
- 2단계: precision_approach_distance 이내 도달 시 Nav2 취소 후
         cmd_vel 직접 제어로 저속 정밀 접근
- 3단계: TF 기반 최종 yaw 보정 후 완료 보고

task_manager가 move_to_goal() 메서드를 직접 호출하는 방식으로 사용.
"""

import math
import time
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
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


# 최종 목적지 정지 자세(사방향 90° 스냅) 회전 파라미터
STOP_ROTATE_SPEED = 0.8        # [rad/s]
STOP_ROTATE_TOL = 0.05         # [rad] 약 3도
STOP_ROTATE_TIMEOUT = 8.0      # [s]

# 최종 목적지 정지 자세 모드. state_manager 가 task_type 에 따라 지정한다.
STOP_NEAREST_90 = "NEAREST_90"  # 도착 heading 기준 가장 가까운 90° (기본)
STOP_NEAREST_Y = "NEAREST_Y"    # +y/-y(법선) 중 회전이 적은 쪽 (MOVE_TO_PRODUCT/DISPLAY)
STOP_PLUS_Y = "PLUS_Y"          # 월드 +y 고정 (RETURN_HOME, standby)
STOP_PLUS_X = "PLUS_X"          # 월드 +x 고정 (MOVE_TO_STOCK)
STOP_MINUS_X = "MINUS_X"        # 월드 -x 고정 (MOVE_TO_PICKUP)

# _nav2_navigate 가 비상 정지로 중단됐음을 알리는 sentinel.
# True(성공) / False(실패) 와 구분하기 위해 별도 객체를 쓴다.
NAV_PAUSED = object()


class MoveToGoal(Node):
    def __init__(self, emergency_latch=None):
        super().__init__("move_to_goal")

        # 비상 정지 래치. state_manager 가 공유 인스턴스를 주입한다.
        # 단독 실행(__main__)이면 자체 생성해 항상 정상 동작하게 한다.
        if emergency_latch is None:
            from pinky_amr_1.emergency_latch import EmergencyLatch
            emergency_latch = EmergencyLatch()
        self._emergency = emergency_latch

        self.declare_parameter("precision_approach_distance", 0.3)
        self.declare_parameter("xy_goal_tolerance", 0.02)
        self.declare_parameter("yaw_goal_tolerance", 0.05)
        self.declare_parameter("nav_timeout_sec", 120.0)

        self._prec_dist = self.get_parameter("precision_approach_distance").value
        self._xy_tol = self.get_parameter("xy_goal_tolerance").value
        self._yaw_tol = self.get_parameter("yaw_goal_tolerance").value
        self._nav_timeout = self.get_parameter("nav_timeout_sec").value

        self._lock = threading.Lock()
        self._cur_x = 0.0
        self._cur_y = 0.0
        self._cur_yaw = 0.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self.create_timer(0.05, self._update_pose)

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.get_logger().info("MoveToGoal ready.")

    # ------------------------------------------------------------------ #
    # 외부 인터페이스 (blocking, executor 스레드에서 호출)
    # ------------------------------------------------------------------ #

    def move_to_goal(
        self, x: float, y: float, final: bool = True, final_mode: str = STOP_NEAREST_90
    ) -> bool:
        """목표 (x, y) '위치'까지 이동(도착)만 한다. 정지 자세(회전)는 State Machine 담당.

        zone 의 theta 는 사용하지 않는다.
        - 중간 경유지(final=False): nav2 로 근처까지 통과만 한다(정밀접근 생략).
        - 목적지(final=True): 정밀접근으로 위치 오차 이내까지 도달한다(회전 없음).
        task_manager의 daemon 스레드에서 호출.
        """
        self.get_logger().info(f"move_to_goal: target=({x:.3f},{y:.3f}) final={final}")

        # Nav2 phase: 비상 정지로 중단되면(NAV_PAUSED) 재개될 때까지 대기한 뒤,
        # 현재 위치 기준으로 bearing 을 다시 계산해 같은 목표로 재시도한다(pause-continue).
        while True:
            # Nav2 목표 헤딩을 "현재→목표 진행 방향" bearing 으로 준다. yaw=0(동쪽) 하드코딩 시
            # use_rotate_to_heading 컨트롤러가 매 목표마다 로봇을 동쪽으로 돌려(불필요한 90°)
            # 축이 틀어졌다. 최종 정지 자세는 도착 후 _rotate_to_nearest_90 이 따로 잡는다.
            with self._lock:
                cur_x, cur_y = self._cur_x, self._cur_y
            bearing = math.atan2(y - cur_y, x - cur_x)

            nav_result = self._nav2_navigate(x, y, bearing)
            if nav_result is NAV_PAUSED:
                self._wait_if_paused()
                continue
            if not nav_result:
                return False
            break

        if not final:
            self.get_logger().info("move_to_goal: 경유지 통과")
            return True

        if not self._precision_approach(x, y):
            return False

        # 최종 목적지(goal zone)에서만 정지 자세 회전(중간 경유지는 회전 안 함).
        # final_mode 에 따라 정지 yaw 를 정한다(task_type 별 정책은 state_manager). zone theta 는 안 씀.
        self._rotate_to_stop_pose(final_mode)

        self.get_logger().info("move_to_goal: 위치 도착")
        return True

    def cancel_navigation(self):
        """로봇을 즉시 정지. Nav2 액션 목표는 별도 취소 필요."""
        self._stop_robot()

    # ------------------------------------------------------------------ #
    # 비상 정지(pause-continue)
    # ------------------------------------------------------------------ #

    def _wait_if_paused(self) -> float:
        """비상 정지 중이면 재개될 때까지 제자리에서 대기한다.

        대기 동안 0 속도 명령을 주기적으로 재발행해 로봇을 확실히 멈춰둔다.
        반환값은 대기한 시간(초)으로, 호출부가 자신의 timeout deadline 을
        그만큼 미뤄 비상 정지 시간이 주행 timeout 을 잡아먹지 않게 한다.
        """
        if not self._emergency.is_stopped():
            return 0.0

        start = time.time()
        self.get_logger().warn(
            f"[비상정지] move 일시정지 — reason={self._emergency.reason}"
        )
        while self._emergency.is_stopped() and rclpy.ok():
            self._stop_robot()
            time.sleep(0.1)
        waited = time.time() - start
        self.get_logger().info(f"[비상정지] move 재개 ({waited:.1f}s 정지)")
        return waited

    # ------------------------------------------------------------------ #
    # 내부 단계
    # ------------------------------------------------------------------ #

    def _nav2_navigate(self, x: float, y: float, yaw: float) -> bool:
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose action server unavailable")
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send_future = self._nav_client.send_goal_async(goal)

        # MultiThreadedExecutor가 콜백을 처리하므로 future 완료를 polling으로 대기
        # (spin_until_future_complete은 이미 스피닝 중인 노드와 충돌)
        send_deadline = time.time() + 5.0
        while not send_future.done():
            if time.time() > send_deadline:
                self.get_logger().error("Nav2 goal send timeout")
                return False
            time.sleep(0.05)

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Nav2 goal rejected")
            return False

        deadline = time.time() + self._nav_timeout
        while time.time() < deadline:
            # 비상 정지: Nav2 목표를 취소하고 로봇을 멈춘 뒤 NAV_PAUSED 로 빠져나간다.
            # 재개 후 move_to_goal 이 같은 목표로 다시 _nav2_navigate 를 호출한다.
            if self._emergency.is_stopped():
                goal_handle.cancel_goal_async()
                self._stop_robot()
                self.get_logger().warn("[비상정지] Nav2 주행 중단 — 목표 취소")
                return NAV_PAUSED

            with self._lock:
                dx = x - self._cur_x
                dy = y - self._cur_y
                dist = math.hypot(dx, dy)

            if dist <= self._prec_dist:
                # 정밀 접근 전환: Nav2 취소
                cancel_future = goal_handle.cancel_goal_async()
                cancel_deadline = time.time() + 3.0
                while not cancel_future.done():
                    if time.time() > cancel_deadline:
                        break
                    time.sleep(0.05)
                self._stop_robot()
                self.get_logger().info(
                    f"Nav2 phase done — dist={dist:.3f}m, switching to precision"
                )
                return True

            time.sleep(0.1)

        # 타임아웃 — Nav2 취소
        goal_handle.cancel_goal_async()
        self.get_logger().warn("Nav2 navigation timeout")
        return False

    def _precision_approach(self, tx: float, ty: float) -> bool:
        """저속 직진으로 목표 xy 오차 이하까지 접근."""
        deadline = time.time() + 15.0
        KP = 0.5

        while time.time() < deadline:
            if self._emergency.is_stopped():
                self._stop_robot()
                deadline += self._wait_if_paused()
                continue

            with self._lock:
                dx = tx - self._cur_x
                dy = ty - self._cur_y
                cur_yaw = self._cur_yaw

            dist = math.hypot(dx, dy)
            if dist <= self._xy_tol:
                self._stop_robot()
                return True

            twist = Twist()
            twist.linear.x = min(KP * dist, 0.12)
            # 목표에 가까우면 atan2(dy,dx)(목표 방향)가 노이즈에 민감해 헤딩이 요동친다(wobble).
            # 진입 시 이미 정렬돼 있으므로 가까운 구간(<0.03m)에선 직진만 하고,
            # 충분히 멀 때만 조향한다. angular.z 도 클램프해 급격한 스윙을 막는다.
            if dist > 0.03:
                target_heading = math.atan2(dy, dx)
                angle_err = normalize_angle(target_heading - cur_yaw)
                twist.angular.z = max(-0.6, min(0.6, 1.0 * angle_err))
            else:
                twist.angular.z = 0.0
            self._cmd_pub.publish(twist)
            time.sleep(0.05)

        self._stop_robot()
        self.get_logger().warn("Precision approach timeout")
        return False

    def _yaw_correction(self, target_yaw: float) -> bool:
        """제자리 회전으로 최종 yaw 보정."""
        deadline = time.time() + 8.0
        KP = 1.2

        while time.time() < deadline:
            with self._lock:
                err = normalize_angle(target_yaw - self._cur_yaw)

            if abs(err) <= self._yaw_tol:
                self._stop_robot()
                return True

            twist = Twist()
            twist.angular.z = max(min(KP * err, 0.3), -0.3)
            self._cmd_pub.publish(twist)
            time.sleep(0.05)

        self._stop_robot()
        self.get_logger().warn("Yaw correction timeout")
        return False

    def _stop_robot(self):
        self._cmd_pub.publish(Twist())

    def _rotate_to_stop_pose(self, mode: str = STOP_NEAREST_90) -> None:
        """최종 목적지 정지 자세로 제자리 회전한다(중간 경유지 제외).

        mode 에 따라 목표 yaw(월드 프레임)를 정한다. cmd_vel 로 직접 회전한다.
        - STOP_PLUS_Y / STOP_PLUS_X / STOP_MINUS_X: 해당 축 방향으로 고정.
        - STOP_NEAREST_Y: +y/-y(법선) 중 도착 heading 에서 회전이 적은 쪽.
        - STOP_NEAREST_90(기본): 가장 가까운 90°(축 정렬) 스냅.
        """
        half_pi = math.pi / 2.0
        with self._lock:
            cur = self._cur_yaw
        if mode == STOP_PLUS_Y:
            target = half_pi
        elif mode == STOP_PLUS_X:
            target = 0.0
        elif mode == STOP_MINUS_X:
            target = math.pi
        elif mode == STOP_NEAREST_Y:
            target = half_pi if abs(normalize_angle(half_pi - cur)) <= abs(
                normalize_angle(-half_pi - cur)
            ) else -half_pi
        else:
            target = round(cur / half_pi) * half_pi
        target = normalize_angle(target)
        self.get_logger().info(f"move_to_goal: 정지 자세 회전 {cur:.2f} -> {target:.2f} rad")
        deadline = time.time() + STOP_ROTATE_TIMEOUT
        while time.time() < deadline:
            if self._emergency.is_stopped():
                self._cmd_pub.publish(Twist())
                deadline += self._wait_if_paused()
                continue

            with self._lock:
                err = normalize_angle(target - self._cur_yaw)
            if abs(err) < STOP_ROTATE_TOL:
                break
            twist = Twist()
            twist.angular.z = STOP_ROTATE_SPEED if err > 0 else -STOP_ROTATE_SPEED
            self._cmd_pub.publish(twist)
            time.sleep(0.05)
        self._cmd_pub.publish(Twist())

    # ------------------------------------------------------------------ #
    # TF 위치 업데이트
    # ------------------------------------------------------------------ #

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


def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoal()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
