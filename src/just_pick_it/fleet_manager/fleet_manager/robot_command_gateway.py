from __future__ import annotations

import math
from typing import Any, Callable

from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from just_pick_it_interfaces.action import DockCommand, MoveCommand
from just_pick_it_interfaces.srv import EmergencyControl

try:
    from just_pick_it_interfaces.action import ExecuteTask
except ImportError:
    ExecuteTask = None


FeedbackCallback = Callable[[str, int, int], None]
CobotFeedbackCallback = Callable[[dict[str, Any]], None]
ResultCallback = Callable[[dict[str, Any]], None]


class RobotCommandGateway:
    """Fleet task를 ROS2 Action/Service 명령으로 변환하는 출력 adapter.

    역할:
    - TaskManager는 ROS2 topic/action 이름과 message 구조를 직접 알지 않는다.
    - PICKY 이동 task는 MoveCommand.action goal로 변환해 State Manager에 보낸다.
    - COBOT 작업 task는 ExecuteTask.action이 생성되면 goal로 변환해 State Manager에 보낸다.
    - Action feedback/result를 TaskManager callback으로 다시 전달한다.

    현재 지원:
    - PICKY MoveCommand.action
    - PICKY DockCommand.action
    - COBOT ExecuteTask.action client hook (메시지 정의 대기)
    - PICKY/COBOT EmergencyControl service
    """

    def __init__(
        self,
        node: Node,
        *,
        # 크로스머신(PC<->보드) 첫 주문 시 action 서버 DDS discovery 가 2초보다
        # 오래 걸려 MoveCommand 가 실패하던 문제로 여유를 둔다. 첫 명령만 대기하고
        # 이후엔 이미 discovery 된 상태라 즉시 반환된다.
        action_wait_timeout_sec: float = 8.0,
        service_wait_timeout_sec: float = 1.0,
        map_frame: str = "map",
    ) -> None:
        """RobotCommandGateway를 초기화한다."""
        self._node = node
        self._action_wait_timeout_sec = action_wait_timeout_sec
        self._service_wait_timeout_sec = service_wait_timeout_sec
        self._map_frame = map_frame
        # action 클라이언트 전용 ReentrantCallbackGroup. wait_for_server 를 task
        # dispatch 콜백 안에서 호출하는데, 클라이언트가 노드 기본(mutually-exclusive)
        # 그룹을 쓰면 그 콜백이 블록된 동안 같은 그룹의 discovery 콜백이 못 돌아
        # 서버 매칭이 영영 안 된다(콜백그룹 데드락). 별도 reentrant 그룹으로 푼다.
        self._cb_group = ReentrantCallbackGroup()
        self._move_clients: dict[str, ActionClient] = {}
        self._dock_clients: dict[str, ActionClient] = {}
        self._cobot_clients: dict[str, ActionClient] = {}
        self._emergency_clients: dict[str, Any] = {}
        self._active_move_goals: dict[int, Any] = {}
        self._active_dock_goals: dict[int, Any] = {}
        self._active_cobot_goals: dict[int, Any] = {}

    def prewarm(self, robot_names: list[str]) -> None:
        """PICKY action 클라이언트를 미리 생성해 discovery 를 startup 에 끝낸다.

        wait_for_server 를 task dispatch 콜백 안에서 동기 호출하면, 첫 주문 때
        해당 콜백이 블록되는 동안 클라이언트 discovery 가 제때 완료되지 못해
        timeout(첫 MoveCommand 실패) 나는 문제가 있었다. node spin 전에 클라이언트를
        만들어 두면, executor 가 자유롭게 도는 기동 구간에 discovery 가 완료되어
        첫 주문 시 wait_for_server 가 즉시 반환한다.
        """
        for name in robot_names:
            self._get_move_client(name)
            self._get_dock_client(name)
        self._node.get_logger().info(
            f"[RobotCommandGateway] action 클라이언트 prewarm: {list(robot_names)}"
        )

    # ==================================================================
    # PICKY MoveCommand
    # ==================================================================

    def send_move_task(
        self,
        *,
        robot_name: str,
        task_id: int,
        task_type: str,
        waypoints: tuple[str, ...] | list[str],
        zone_map: dict[str, dict[str, Any]],
        feedback_callback: FeedbackCallback | None = None,
        result_callback: ResultCallback | None = None,
    ) -> bool:
        """PICKY 이동 task를 MoveCommand.action goal로 전송한다.

        MoveCommand goal에는 task_id 필드가 없으므로 task_id는 Gateway 내부
        callback 매핑에만 사용한다.
        """
        client = self._get_move_client(robot_name)
        if not client.wait_for_server(timeout_sec=self._action_wait_timeout_sec):
            self._node.get_logger().warn(
                f"[RobotCommandGateway] {robot_name} MoveCommand action server 없음"
            )
            return False

        poses = self._build_pose_waypoints(waypoints, zone_map)
        if not poses:
            self._node.get_logger().warn(
                f"[RobotCommandGateway] task_id={task_id} waypoint pose 변환 실패"
            )
            return False

        goal = MoveCommand.Goal()
        goal.task_type = task_type
        goal.waypoints = poses

        self._node.get_logger().info(
            f"[RobotCommandGateway] {robot_name} task_id={task_id} {task_type} 전송, "
            f"waypoints={len(poses)}"
        )
        self._node.get_logger().info(
            f"[PATHTRACE][Gateway->StateMachine] task_id={task_id} zone={list(waypoints)} "
            "poses=[" + ", ".join(
                f"({p.pose.position.x:.3f},{p.pose.position.y:.3f})" for p in poses
            ) + "]"
        )

        send_future = client.send_goal_async(
            goal,
            feedback_callback=lambda feedback_msg: self._on_move_feedback(
                robot_name,
                task_id,
                feedback_msg,
                feedback_callback,
            ),
        )
        send_future.add_done_callback(
            lambda future: self._on_move_goal_response(
                robot_name,
                task_id,
                task_type,
                future,
                result_callback,
            )
        )
        return True

    def cancel_task(self, robot_name: str, task_id: int) -> bool:
        """진행 중인 PICKY action goal 취소를 요청한다."""
        goal_handle = self._active_move_goals.get(task_id)
        if goal_handle is None:
            goal_handle = self._active_dock_goals.get(task_id)
        if goal_handle is None:
            goal_handle = self._active_cobot_goals.get(task_id)
        if goal_handle is None:
            self._node.get_logger().warn(
                f"[RobotCommandGateway] task_id={task_id} 취소 실패: active goal 없음"
            )
            return False

        goal_handle.cancel_goal_async()
        self._node.get_logger().info(
            f"[RobotCommandGateway] {robot_name} task_id={task_id} 취소 요청"
        )
        return True

    def _get_move_client(self, robot_name: str) -> ActionClient:
        """robot_name에 대응되는 MoveCommand ActionClient를 lazy 생성한다."""
        client = self._move_clients.get(robot_name)
        if client is not None:
            return client

        namespace = self._robot_name_to_namespace(robot_name)
        action_name = f"/{namespace}/move_command"
        client = ActionClient(
            self._node, MoveCommand, action_name, callback_group=self._cb_group
        )
        self._move_clients[robot_name] = client
        return client

    def _on_move_goal_response(
        self,
        robot_name: str,
        task_id: int,
        task_type: str,
        future: Any,
        result_callback: ResultCallback | None,
    ) -> None:
        """MoveCommand goal accept/reject 결과를 처리한다."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().warn(
                f"[RobotCommandGateway] {robot_name} task_id={task_id} goal rejected"
            )
            if result_callback is not None:
                result_callback(
                    {
                        "task_id": task_id,
                        "robot_name": robot_name,
                        "task_type": task_type,
                        "success": False,
                        "message": "goal rejected",
                    }
                )
            return

        self._active_move_goals[task_id] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done: self._on_move_result(
                robot_name,
                task_id,
                task_type,
                goal_handle,
                done,
                result_callback,
            )
        )

    def _on_move_feedback(
        self,
        robot_name: str,
        task_id: int,
        feedback_msg: Any,
        feedback_callback: FeedbackCallback | None,
    ) -> None:
        """MoveCommand feedback을 TaskManager callback으로 전달한다."""
        feedback = feedback_msg.feedback
        current_index = int(feedback.current_waypoint_index)

        if feedback_callback is not None:
            feedback_callback(robot_name, task_id, current_index)

    def _on_move_result(
        self,
        robot_name: str,
        task_id: int,
        task_type: str,
        goal_handle: Any,
        future: Any,
        result_callback: ResultCallback | None,
    ) -> None:
        """MoveCommand result를 TaskManager가 이해하는 dict로 변환한다."""
        if self._active_move_goals.get(task_id) is goal_handle:
            self._active_move_goals.pop(task_id, None)

        result_wrapper = future.result()
        result = result_wrapper.result
        payload = {
            "task_id": task_id,
            "robot_name": robot_name,
            "task_type": task_type,
            "success": bool(result.success),
            "message": result.message,
        }

        self._node.get_logger().info(
            f"[RobotCommandGateway] {robot_name} task_id={task_id} result: "
            f"success={payload['success']}, message={payload['message']}"
        )

        if result_callback is not None:
            result_callback(payload)

    # ==================================================================
    # PICKY DockCommand
    # ==================================================================

    def send_dock_task(
        self,
        *,
        robot_name: str,
        task_id: int,
        dock_name: str,
        start_zone_name: str,
        result_callback: ResultCallback | None = None,
    ) -> bool:
        """PICKY 도킹 task를 DockCommand.action goal로 전송한다.

        DockCommand는 Nav2 waypoint 이동이 아니라 State Manager 내부의
        AprilTag/ArUco 정렬 + odom 거리 기반 정밀 도킹 루틴을 실행하기 위한 action이다.

        TrafficManager는 STANDBY_ZONE까지의 교통/점유만 관여한다.
        CHARGING_DOCK은 DB zone pose가 아니라 State Manager 내부 도킹 루틴이
        해석하는 논리 도크 이름으로만 전달한다.
        """
        client = self._get_dock_client(robot_name)
        if not client.wait_for_server(timeout_sec=self._action_wait_timeout_sec):
            self._node.get_logger().warn(
                f"[RobotCommandGateway] {robot_name} DockCommand action server 없음"
            )
            return False

        goal = DockCommand.Goal()
        goal.task_id = int(task_id)
        goal.dock_name = dock_name
        goal.start_zone_name = start_zone_name

        self._node.get_logger().info(
            f"[RobotCommandGateway] {robot_name} task_id={task_id} DOCK_IN 전송, "
            f"dock={dock_name}, start={start_zone_name}"
        )

        send_future = client.send_goal_async(
            goal,
            feedback_callback=lambda feedback_msg: self._on_dock_feedback(
                robot_name,
                task_id,
                feedback_msg,
            ),
        )
        send_future.add_done_callback(
            lambda future: self._on_dock_goal_response(
                robot_name,
                task_id,
                future,
                result_callback,
            )
        )
        return True

    def _get_dock_client(self, robot_name: str) -> ActionClient:
        """robot_name에 대응되는 DockCommand ActionClient를 lazy 생성한다."""
        client = self._dock_clients.get(robot_name)
        if client is not None:
            return client

        namespace = self._robot_name_to_namespace(robot_name)
        action_name = f"/{namespace}/dock_command"
        client = ActionClient(
            self._node, DockCommand, action_name, callback_group=self._cb_group
        )
        self._dock_clients[robot_name] = client
        return client

    def _on_dock_goal_response(
        self,
        robot_name: str,
        task_id: int,
        future: Any,
        result_callback: ResultCallback | None,
    ) -> None:
        """DockCommand goal accept/reject 결과를 처리한다."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().warn(
                f"[RobotCommandGateway] {robot_name} task_id={task_id} dock goal rejected"
            )
            if result_callback is not None:
                result_callback(
                    {
                        "task_id": task_id,
                        "robot_name": robot_name,
                        "task_type": "DOCK_IN",
                        "success": False,
                        "message": "dock goal rejected",
                    }
                )
            return

        self._active_dock_goals[task_id] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done: self._on_dock_result(
                robot_name,
                task_id,
                goal_handle,
                done,
                result_callback,
            )
        )

    def _on_dock_feedback(self, robot_name: str, task_id: int, feedback_msg: Any) -> None:
        """DockCommand feedback을 로그로 남긴다."""
        feedback = feedback_msg.feedback
        self._node.get_logger().debug(
            f"[RobotCommandGateway] {robot_name} task_id={task_id} dock feedback: "
            f"phase={feedback.phase}, progress={feedback.progress:.2f}, message={feedback.message}"
        )

    def _on_dock_result(
        self,
        robot_name: str,
        task_id: int,
        goal_handle: Any,
        future: Any,
        result_callback: ResultCallback | None,
    ) -> None:
        """DockCommand result를 TaskManager가 이해하는 dict로 변환한다."""
        if self._active_dock_goals.get(task_id) is goal_handle:
            self._active_dock_goals.pop(task_id, None)

        result_wrapper = future.result()
        result = result_wrapper.result
        payload = {
            "task_id": task_id,
            "robot_name": robot_name,
            "task_type": "DOCK_IN",
            "success": bool(result.success),
            "message": result.message,
        }

        self._node.get_logger().info(
            f"[RobotCommandGateway] {robot_name} task_id={task_id} dock result: "
            f"success={payload['success']}, message={payload['message']}"
        )

        if result_callback is not None:
            result_callback(payload)

    # ==================================================================
    # COBOT ExecuteTask
    # ==================================================================

    def send_cobot_task(
        self,
        *,
        robot_name: str,
        task: dict[str, Any],
        feedback_callback: CobotFeedbackCallback | None = None,
        result_callback: ResultCallback | None = None,
    ) -> bool:
        """COBOT task를 ExecuteTask.action goal로 전송한다."""
        if ExecuteTask is None:
            self._node.get_logger().warn(
                f"[RobotCommandGateway] {robot_name} ExecuteTask.action 미생성: "
                f"task_id={task.get('task_id')}, task_type={task.get('task_type')}"
            )
            return False

        client = self._get_cobot_client(robot_name)
        if not client.wait_for_server(timeout_sec=self._action_wait_timeout_sec):
            self._node.get_logger().warn(
                f"[RobotCommandGateway] {robot_name} ExecuteTask action server 없음"
            )
            return False

        goal = self._build_cobot_goal(task)
        task_id = int(goal.task_id)
        task_type = str(goal.task_type)

        self._node.get_logger().info(
            f"[RobotCommandGateway] {robot_name} task_id={task_id} {task_type} 전송"
        )

        send_future = client.send_goal_async(
            goal,
            feedback_callback=lambda feedback_msg: self._on_cobot_feedback(
                robot_name,
                task_id,
                task_type,
                feedback_msg,
                feedback_callback,
            ),
        )
        send_future.add_done_callback(
            lambda future: self._on_cobot_goal_response(
                robot_name,
                task_id,
                task_type,
                future,
                result_callback,
            )
        )
        return True

    def _get_cobot_client(self, robot_name: str) -> ActionClient:
        """robot_name에 대응되는 ExecuteTask ActionClient를 lazy 생성한다."""
        client = self._cobot_clients.get(robot_name)
        if client is not None:
            return client

        namespace = self._robot_name_to_namespace(robot_name)
        action_name = f"/{namespace}/execute_task"
        client = ActionClient(
            self._node, ExecuteTask, action_name, callback_group=self._cb_group
        )
        self._cobot_clients[robot_name] = client
        return client

    def _build_cobot_goal(self, task: dict[str, Any]) -> ExecuteTask.Goal:
        """Fleet task summary를 ExecuteTask goal로 변환한다."""
        goal = ExecuteTask.Goal()

        quantity = self._int_or_zero(
            task.get("product_quantity")
            or task.get("quantity")
            or task.get("requested_quantity")
            or task.get("processed_quantity")
        )

        self._set_goal_field(goal, "task_id", self._int_or_zero(task.get("task_id")))
        self._set_goal_field(goal, "task_type", str(task.get("task_type") or ""))
        self._set_goal_field(goal, "order_id", self._int_or_zero(task.get("order_id")))
        self._set_goal_field(goal, "display_item_id", self._int_or_zero(task.get("display_item_id")))
        self._set_goal_field(goal, "product_name", str(task.get("product_name") or ""))
        self._set_goal_field(goal, "quantity", quantity)
        self._set_goal_field(goal, "target_zone_name", str(task.get("target_zone_name") or ""))
        return goal

    def _on_cobot_goal_response(
        self,
        robot_name: str,
        task_id: int,
        task_type: str,
        future: Any,
        result_callback: ResultCallback | None,
    ) -> None:
        """ExecuteTask goal accept/reject 결과를 처리한다."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().warn(
                f"[RobotCommandGateway] {robot_name} task_id={task_id} cobot goal rejected"
            )
            if result_callback is not None:
                result_callback(
                    {
                        "task_id": task_id,
                        "robot_name": robot_name,
                        "task_type": task_type,
                        "success": False,
                        "message": "cobot goal rejected",
                    }
                )
            return

        self._active_cobot_goals[task_id] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done: self._on_cobot_result(
                robot_name,
                task_id,
                task_type,
                goal_handle,
                done,
                result_callback,
            )
        )

    def _on_cobot_feedback(
        self,
        robot_name: str,
        task_id: int,
        task_type: str,
        feedback_msg: Any,
        feedback_callback: CobotFeedbackCallback | None,
    ) -> None:
        """ExecuteTask feedback을 TaskManager callback으로 전달한다."""
        feedback = feedback_msg.feedback
        status = (
            self._get_message_field(feedback, "status")
            or self._get_message_field(feedback, "state")
            or ""
        )
        payload = {
            "task_id": task_id,
            "robot_name": robot_name,
            "task_type": task_type,
            "status": str(status),
            "state": str(status),
            "message": str(self._get_message_field(feedback, "message") or ""),
            "progress": float(self._get_message_field(feedback, "progress") or 0.0),
            "processed_quantity": self._quantity_from_message(feedback),
        }

        if feedback_callback is not None:
            feedback_callback(payload)

    def _on_cobot_result(
        self,
        robot_name: str,
        task_id: int,
        task_type: str,
        goal_handle: Any,
        future: Any,
        result_callback: ResultCallback | None,
    ) -> None:
        """ExecuteTask result를 TaskManager가 이해하는 dict로 변환한다."""
        if self._active_cobot_goals.get(task_id) is goal_handle:
            self._active_cobot_goals.pop(task_id, None)

        result_wrapper = future.result()
        result = result_wrapper.result
        status = str(self._get_message_field(result, "status") or "")
        message = (
            self._get_message_field(result, "result_message")
            or self._get_message_field(result, "message")
            or status
        )
        payload = {
            "task_id": task_id,
            "robot_name": robot_name,
            "task_type": task_type,
            "success": bool(self._get_message_field(result, "success")),
            "status": status,
            "message": str(message),
            "processed_quantity": self._quantity_from_message(result),
            "stock_delta": self._int_or_zero(self._get_message_field(result, "stock_delta")),
        }

        self._node.get_logger().info(
            f"[RobotCommandGateway] {robot_name} task_id={task_id} cobot result: "
            f"success={payload['success']}, status={payload['status']}, "
            f"message={payload['message']}"
        )

        if result_callback is not None:
            result_callback(payload)

    # ==================================================================
    # Emergency Stop Service
    # ==================================================================

    def set_emergency_stop(
        self,
        robot_names: list[str] | tuple[str, ...],
        enabled: bool,
        *,
        reason: str = "ADMIN",
        task_id: int = 0,
        request_id: str = "",
    ) -> dict[str, bool]:
        """PICKY/COBOT State Manager의 EmergencyControl service를 호출한다.

        Args:
            robot_names: `PICKY1`, `COBOT1` 같은 DB robot_name 목록.
            enabled: True면 emergency stop, False면 resume.
            reason: emergency/resume 요청 사유.
            task_id: 관련 task가 있으면 task_id, 없으면 0.
            request_id: Fleet API/Fleet Manager 로그 추적용 id.

        Returns:
            robot_name별 service 호출 요청 성공 여부.
            실제 service response accepted는 done callback에서 로그로 남긴다.
        """
        results: dict[str, bool] = {}

        for robot_name in robot_names:
            results[robot_name] = self._send_emergency_stop(
                robot_name,
                enabled,
                reason=reason,
                task_id=task_id,
                request_id=request_id,
            )

        return results

    def _send_emergency_stop(
        self,
        robot_name: str,
        enabled: bool,
        *,
        reason: str,
        task_id: int,
        request_id: str,
    ) -> bool:
        """robot 1대에 EmergencyControl 요청을 보낸다."""
        client = self._get_emergency_client(robot_name)
        if not client.wait_for_service(timeout_sec=self._service_wait_timeout_sec):
            self._node.get_logger().warn(
                f"[RobotCommandGateway] {robot_name} emergency_control service 없음"
            )
            return False

        request = EmergencyControl.Request()
        request.emergency_stop = enabled
        request.reason = reason
        request.task_id = int(task_id or 0)
        request.request_id = request_id
        future = client.call_async(request)
        future.add_done_callback(
            lambda done: self._on_emergency_response(robot_name, enabled, done)
        )
        self._node.get_logger().info(
            f"[RobotCommandGateway] {robot_name} emergency_control={enabled} "
            f"reason={reason} task_id={request.task_id} 요청"
        )
        return True

    def _get_emergency_client(self, robot_name: str) -> Any:
        """robot_name에 대응되는 EmergencyFleet API Client를 lazy 생성한다."""
        client = self._emergency_clients.get(robot_name)
        if client is not None:
            return client

        namespace = self._robot_name_to_namespace(robot_name)
        service_name = f"/{namespace}/emergency_control"
        client = self._node.create_client(EmergencyControl, service_name)
        self._emergency_clients[robot_name] = client
        return client

    def _on_emergency_response(self, robot_name: str, enabled: bool, future: Any) -> None:
        """EmergencyControl service response를 로그로 남긴다."""
        try:
            response = future.result()
        except Exception as exc:
            self._node.get_logger().warn(
                f"[RobotCommandGateway] {robot_name} emergency_control={enabled} 응답 오류: {exc}"
            )
            return

        if response.accepted:
            self._node.get_logger().info(
                f"[RobotCommandGateway] {robot_name} emergency_control={enabled} "
                f"accepted status={response.status}: {response.message}"
            )
        else:
            self._node.get_logger().warn(
                f"[RobotCommandGateway] {robot_name} emergency_control={enabled} "
                f"rejected status={response.status}: {response.message}"
            )

    # ==================================================================
    # Pose 변환 helpers
    # ==================================================================

    def _build_pose_waypoints(
        self,
        waypoints: tuple[str, ...] | list[str],
        zone_map: dict[str, dict[str, Any]],
    ) -> list[PoseStamped]:
        """zone_name waypoint 목록을 PoseStamped 목록으로 변환한다."""
        poses: list[PoseStamped] = []

        for zone_name in waypoints:
            zone = zone_map.get(zone_name)
            pose = (zone or {}).get("pose") or {}

            x = pose.get("x")
            y = pose.get("y")
            theta = pose.get("theta") or 0.0
            if x is None or y is None:
                self._node.get_logger().warn(
                    f"[RobotCommandGateway] zone pose 없음: {zone_name}"
                )
                return []

            poses.append(self._pose_stamped(float(x), float(y), float(theta)))

        return poses

    def _pose_for_zone(
        self,
        zone_name: str,
        zone_map: dict[str, dict[str, Any]],
    ) -> PoseStamped | None:
        """zone_name 하나를 PoseStamped로 변환한다."""
        zone = zone_map.get(zone_name)
        pose = (zone or {}).get("pose") or {}

        x = pose.get("x")
        y = pose.get("y")
        theta = pose.get("theta") or 0.0
        if x is None or y is None:
            return None

        return self._pose_stamped(float(x), float(y), float(theta))

    def _pose_stamped(self, x: float, y: float, theta: float) -> PoseStamped:
        """2D map pose를 PoseStamped로 변환한다."""
        msg = PoseStamped()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)
        return msg

    def _robot_name_to_namespace(self, robot_name: str) -> str:
        """DB robot_name을 ROS namespace로 변환한다."""
        return robot_name.lower()

    def _int_or_zero(self, value: Any) -> int:
        """ROS action에서 optional int를 표현하기 위한 0 변환 helper."""
        if value is None:
            return 0
        try:
            return int(value)
        except (TypeError, ValueError):
            return 0

    def _set_goal_field(self, goal: Any, field_name: str, value: Any) -> None:
        """최종 action 필드가 확정되기 전까지 존재하는 필드에만 값을 채운다."""
        if hasattr(goal, field_name):
            setattr(goal, field_name, value)

    def _get_message_field(self, message: Any, field_name: str) -> Any:
        """ROS message/action object에서 optional field를 안전하게 읽는다."""
        if hasattr(message, field_name):
            return getattr(message, field_name)
        return None

    def _quantity_from_message(self, message: Any) -> int:
        """ROS message/action object에서 처리 수량을 읽는다."""
        return self._int_or_zero(self._get_message_field(message, "processed_quantity"))
