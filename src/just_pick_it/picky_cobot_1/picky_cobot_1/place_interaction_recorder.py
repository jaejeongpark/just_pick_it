#!/usr/bin/env python3
"""DISPLAY_PLACE 데이터 수집용 human recorder.

픽 human_interaction_recorder 를 상속해 '그리퍼 의미'만 반전한다. perception 코드는
수정하지 않고 상속/override 만 한다.

픽 vs place 차이:
  픽   : 빈 그리퍼로 시작 -> [R] 에서 그리퍼 open + 서보 해제 -> free-drive 로 물체에 접근 ->
         [G] 에서 그리퍼 close(잡기)로 종단.
  place: 물건을 쥔 채 시작 -> [R] 에서 팔 서보만 해제(그리퍼는 닫힌 채 물건 쥠) ->
         free-drive 로 쥔 물건을 빈자리 위로 이동 -> [G] 에서 그리퍼 open(놓기)으로 종단.

기록 포맷/학습 파이프라인은 픽과 완전히 동일하다. grip_triggered=True 샘플이 '발화 시점'
(픽=잡는 순간, place=놓는 순간)을 표시하므로, 같은 train 파이프라인으로 place 전용
nn_controller(policy + grip_success_predictor)를 학습할 수 있다.

detection(빈자리 cx/cy/area)은 이 노드가 기록하지 않는다. IBVS 구간의 detection 은
visual_servo_bag_recorder 가 /place/tracked_objects 에서 기록한다(launch 참고).
"""
import math
import threading
import tkinter as tk

import rclpy

from std_msgs.msg import Float64MultiArray

from just_pick_it_perception.human_interaction_recorder_node import (
    HumanInteractionRecorderNode,
    InteractionPhase,
    RecorderGUI,
    _spin_node,
)


class PlaceInteractionRecorderNode(HumanInteractionRecorderNode):
    """픽 recorder 상속 — 그리퍼 방향만 place 의미로 반전한다."""

    def __init__(self):
        super().__init__()

        # place 종단(놓기) 시 그리퍼 개방값. DISPLAY_PLACE 의 부분개방(70)과 일치시킨다.
        self.declare_parameter('place_open_value', 70.0)
        self.declare_parameter('place_open_speed', 50)
        # RELEASING 확인 방향 반전: 픽은 그리퍼 open(>=open_confirm)을 기다리지만,
        # place 는 닫힘(<=close_confirm, 물건 쥠 유지)을 확인하고 free-drive 로 넘어간다.
        self.declare_parameter('gripper_close_confirm_value', 20.0)

        self.place_open_value = float(self.get_parameter('place_open_value').value)
        self.place_open_speed = int(self.get_parameter('place_open_speed').value)
        self.gripper_close_confirm_value = float(
            self.get_parameter('gripper_close_confirm_value').value
        )

        self.get_logger().info(
            'PlaceInteractionRecorderNode 시작 — 물건을 쥔 채 시작. '
            '[R] 팔 서보만 해제(그리퍼 닫힘 유지), [G] 놓기(그리퍼 개방).'
        )

    # ── 그리퍼 닫힘(물건 쥠) 발행 ─────────────────────────────────────────
    def _publish_gripper_hold(self):
        msg = Float64MultiArray()
        msg.data = [0.0, float(self.gripper_open_speed)]
        self.set_gripper_pub.publish(msg)

    # ── [R]: 팔 서보만 해제, 그리퍼는 닫힌 채 유지 ────────────────────────
    def _trigger_release(self):
        self.get_logger().info(
            'Release [R]: 팔 서보 해제(그리퍼 닫힘 유지). '
            'free-drive 로 쥔 물건을 빈자리 위로 옮긴 뒤 [G] 로 놓으세요.'
        )
        # 서보가 살아있을 때 그리퍼를 먼저 0(close)으로 확실히 쥔다.
        self._publish_gripper_hold()

        # 팔 release. set_arm [0] 은 그리퍼 서보까지 함께 푼다.
        arm_msg = Float64MultiArray()
        arm_msg.data = [0.0]
        self.set_arm_pub.publish(arm_msg)

        # release 가 시리얼에서 먼저 처리되도록 짧은 지연 후 그리퍼 close 를 1회 재발행한다.
        # 그러면 그리퍼 서보만 다시 잡혀 0(close, 물건 쥠)이 유지되고 팔 관절은 풀린 채 남는다.
        # 풀린 동안 물건이 흘러내리지 않도록 release_gripper_reopen_delay_sec 를 짧게 둘 것.
        self._schedule_gripper_reopen()

        self.phase = InteractionPhase.RELEASING
        self._release_request_sec = self._now_sec()
        self._release_status_poll_sec = 0.0

    # super._schedule_gripper_reopen 타이머가 호출. place 는 닫힘(0)을 재발행한다.
    def _gripper_reopen_cb(self):
        if self._gripper_reopen_timer is not None:
            self._gripper_reopen_timer.cancel()
            self._gripper_reopen_timer = None
        if self._gripper_reopen_done:
            return
        self._gripper_reopen_done = True
        self._publish_gripper_hold()
        self.get_logger().info(
            '서보 해제 후 그리퍼 close(0) 재발행 — 물건 쥠 유지.'
        )

    # ── RELEASING 확인: 그리퍼가 닫힘(<=confirm)인지 본다 ─────────────────
    def _update_releasing(self):
        elapsed = self._now_sec() - self._release_request_sec

        self._poll_release_status()

        if elapsed < self.release_gripper_reopen_delay_sec + self.release_settle_margin_sec:
            return

        gripper_ok = (
            math.isfinite(self.latest_gripper_value)
            and self.latest_gripper_value <= self.gripper_close_confirm_value
        )
        timed_out = elapsed >= self.release_confirm_timeout_sec

        if gripper_ok or timed_out:
            if timed_out and not gripper_ok:
                self.get_logger().warn(
                    f'Release confirm timeout ({self.release_confirm_timeout_sec:.1f}s). '
                    f'gripper_value={self.latest_gripper_value}. Starting FREE_DRIVE anyway.'
                )
            self._enter_free_drive(elapsed)

    # ── [G]: 놓을 자세 기록 후 그리퍼 open(release) ───────────────────────
    def _do_grip_after_capture(self):
        # fresh status(또는 timeout) 이후 호출. 놓는 순간의 자세를 grip_triggered 로 기록한 뒤
        # 그리퍼를 열어(release) 물건을 내려놓고 GRIPPING(종단 대기)으로 전이한다.
        self._grip_capture_pending = False
        self._commit_sample(grip_triggered=True, result_recorded=False)

        msg = Float64MultiArray()
        msg.data = [self.place_open_value, float(self.place_open_speed)]
        self.set_gripper_pub.publish(msg)

        self.phase = InteractionPhase.GRIPPING
        self.gripping_start_ros_time = self._now_sec()
        self.get_logger().info(
            f'Place pose captured. Opening gripper to {self.place_open_value:.0f} (release).'
        )

    # ── GUI 안내문구를 place 의미로 교체 ──────────────────────────────────
    def get_action_hint(self) -> str:
        if self.phase == InteractionPhase.READY_TO_RELEASE:
            return 'Hold the arm FIRMLY, then [R] to release servos (gripper STAYS closed)'
        if self.phase == InteractionPhase.RELEASING:
            return 'Releasing servos / keeping grip closed... please wait'
        if self.phase == InteractionPhase.FREE_DRIVE:
            return 'Move the held object over the empty slot, then [G] to PLACE (release)'
        if self.phase == InteractionPhase.GRIPPING:
            return 'Placing (opening gripper)...'
        return super().get_action_hint()


def main(args=None):
    rclpy.init(args=args)
    node = PlaceInteractionRecorderNode()

    spin_thread = threading.Thread(target=_spin_node, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    root.title('Place Interaction Recorder')
    RecorderGUI(root, node)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node._close_bag_writer()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
