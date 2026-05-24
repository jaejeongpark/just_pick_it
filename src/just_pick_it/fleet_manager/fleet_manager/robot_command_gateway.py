from __future__ import annotations

import math
from typing import Any, Callable

from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node

from just_pick_it_interfaces.action import DockCommand, MoveCommand
from just_pick_it_interfaces.srv import EmergencyControl


FeedbackCallback = Callable[[str, int, int], None]
ResultCallback = Callable[[dict[str, Any]], None]


class RobotCommandGateway:
    """Fleet task를 ROS2 Action/Service 명령으로 변환하는 출력 adapter.

    역할:
    - TaskManager는 ROS2 topic/action 이름과 message 구조를 직접 알지 않는다.
    - PICKY 이동 task는 MoveCommand.action goal로 변환해 State Manager에 보낸다.
    - Action feedback/result를 TaskManager callback으로 다시 전달한다.

    현재 지원:
    - PICKY MoveCommand.action
    - PICKY DockCommand.action
    - PICKY/COBOT EmergencyControl service

    추후 확장:
    - COBOT ExecuteTask.action
    """

    def __init__(
        self,
        node: Node,
        *,
        action_wait_timeout_sec: float = 2.0,
        service_wait_timeout_sec: float = 1.0,
        map_frame: str = "map",
    ) -> None:
        """RobotCommandGateway를 초기화한다."""
        self._node = node
        self._action_wait_timeout_sec = action_wait_timeout_sec
        self._service_wait_timeout_sec = service_wait_timeout_sec
        self._map_frame = map_frame
        self._move_clients: dict[str, ActionClient] = {}
        self._dock_clients: dict[str, ActionClient] = {}
        self._emergency_clients: dict[str, Any] = {}
        self._active_move_goals: dict[int, Any] = {}
        self._active_dock_goals: dict[int, Any] = {}

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
        client = ActionClient(self._node, MoveCommand, action_name)
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
        future: Any,
        result_callback: ResultCallback | None,
    ) -> None:
        """MoveCommand result를 TaskManager가 이해하는 dict로 변환한다."""
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
        ArUco/라인 기반 정밀 도킹 루틴을 실행하기 위한 action이다.

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
        client = ActionClient(self._node, DockCommand, action_name)
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
        future: Any,
        result_callback: ResultCallback | None,
    ) -> None:
        """DockCommand result를 TaskManager가 이해하는 dict로 변환한다."""
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
        result_callback: ResultCallback | None = None,
    ) -> bool:
        """COBOT task 전송 인터페이스 대기 함수.

        현재 저장소에는 ExecuteTask.action이 아직 정의되어 있지 않다.
        COBOT 담당 인터페이스가 확정되면 이 함수에서 ActionClient를 붙인다.
        """
        self._node.get_logger().warn(
            f"[RobotCommandGateway] {robot_name} COBOT ExecuteTask.action 정의 대기: "
            f"task_id={task.get('task_id')}, task_type={task.get('task_type')}"
        )
        return False

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
            request_id: Control Server/Fleet Manager 로그 추적용 id.

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
        """robot_name에 대응되는 EmergencyControl Service Client를 lazy 생성한다."""
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
