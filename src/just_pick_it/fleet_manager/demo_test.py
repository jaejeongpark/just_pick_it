#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import json
import subprocess
import sys
import time
import types
from dataclasses import dataclass
from pathlib import Path
import threading
from typing import Any, Callable

import requests


REPO_ROOT = Path(__file__).resolve().parents[3]
FLEET_PACKAGE_ROOT = Path(__file__).resolve().parent


def _ensure_rclpy_stub() -> None:
    """web venv에서도 Fleet Manager 클래스를 import할 수 있게 rclpy 최소 stub을 넣는다.

    이 demo는 실제 ROS2 노드를 spin하지 않는다. Control Server API와 TaskManager
    로직만 검증하고, 로봇 실행 결과는 DemoRobotGateway가 가짜로 만든다.
    """
    if "rclpy.node" in sys.modules:
        return

    try:
        import rclpy.node  # noqa: F401
        return
    except ModuleNotFoundError:
        pass

    rclpy_mod = types.ModuleType("rclpy")
    rclpy_node_mod = types.ModuleType("rclpy.node")

    class Node:
        pass

    rclpy_node_mod.Node = Node
    sys.modules.setdefault("rclpy", rclpy_mod)
    sys.modules.setdefault("rclpy.node", rclpy_node_mod)


_ensure_rclpy_stub()
sys.path.insert(0, str(FLEET_PACKAGE_ROOT))

from fleet_manager.control_server_client import ControlServerClient  # noqa: E402
from fleet_manager.task_manager import COBOT_TASK_TYPES, TaskManager  # noqa: E402
from fleet_manager.traffic_manager import TrafficManager  # noqa: E402


HOUSEKEEPING_TASK_TYPES = {"RETURN_HOME", "DOCK_IN", "CHARGE"}
FINAL_TASK_STATUSES = {"SUCCESS", "FAILED", "CANCELLED"}


class DemoLogger:
    """Fleet Manager 클래스가 기대하는 node logger의 최소 구현."""

    def info(self, message: str) -> None:
        print(f"[info] {message}")

    def warn(self, message: str) -> None:
        print(f"[warn] {message}")

    def warning(self, message: str) -> None:
        print(f"[warn] {message}")

    def debug(self, message: str) -> None:
        print(f"[debug] {message}")

    def error(self, message: str) -> None:
        print(f"[error] {message}")


class DemoNode:
    """실제 rclpy Node 대신 logger만 제공하는 테스트용 객체."""

    def __init__(self) -> None:
        self._logger = DemoLogger()

    def get_logger(self) -> DemoLogger:
        return self._logger


@dataclass
class PendingTaskResult:
    """DemoRobotGateway가 나중에 SUCCESS 처리할 pending task."""

    robot_name: str
    task_id: int
    task_type: str
    callback: Callable[[dict[str, Any]], None]


class DemoRobotGateway:
    """실제 로봇 대신 task 실행 결과를 지연 성공 처리하는 gateway.

    TaskManager는 이 객체를 실제 RobotCommandGateway처럼 호출한다.
    차이점은 ROS2 Action을 보내지 않고 pending queue에 넣어둔 뒤,
    DemoRunner가 delay_sec 뒤에 SUCCESS callback을 호출한다는 점이다.
    """

    def __init__(self) -> None:
        self.pending: list[PendingTaskResult] = []
        self.move_goals: list[dict[str, Any]] = []
        self.dock_goals: list[dict[str, Any]] = []
        self.cobot_goals: list[dict[str, Any]] = []
        self.cancel_requests: list[dict[str, Any]] = []
        self.emergency_requests: list[dict[str, Any]] = []

    def send_move_task(
        self,
        *,
        robot_name: str,
        task_id: int,
        task_type: str,
        waypoints: tuple[str, ...],
        zone_map: dict[str, dict[str, Any]],
        feedback_callback: Callable[[str, int, int], None],
        result_callback: Callable[[dict[str, Any]], None],
    ) -> bool:
        """PICKY 이동 task를 fake 실행 queue에 넣는다."""
        self.move_goals.append(
            {
                "robot_name": robot_name,
                "task_id": task_id,
                "task_type": task_type,
                "waypoints": list(waypoints),
            }
        )

        # 첫 waypoint 통과 feedback만 흉내내서 TrafficManager progress도 한 번 갱신한다.
        if len(waypoints) > 1:
            feedback_callback(robot_name, task_id, 1)

        self.pending.append(
            PendingTaskResult(
                robot_name=robot_name,
                task_id=task_id,
                task_type=task_type,
                callback=result_callback,
            )
        )
        return True

    def send_dock_task(
        self,
        *,
        robot_name: str,
        task_id: int,
        dock_name: str,
        start_zone_name: str,
        result_callback: Callable[[dict[str, Any]], None],
    ) -> bool:
        """PICKY DOCK_IN task를 fake 실행 queue에 넣는다."""
        self.dock_goals.append(
            {
                "robot_name": robot_name,
                "task_id": task_id,
                "task_type": "DOCK_IN",
                "dock_name": dock_name,
                "start_zone_name": start_zone_name,
            }
        )
        self.pending.append(
            PendingTaskResult(
                robot_name=robot_name,
                task_id=task_id,
                task_type="DOCK_IN",
                callback=result_callback,
            )
        )
        return True

    def send_cobot_task(
        self,
        *,
        robot_name: str,
        task: dict[str, Any],
        result_callback: Callable[[dict[str, Any]], None],
    ) -> bool:
        """COBOT 작업 task를 fake 실행 queue에 넣는다."""
        task_id = int(task["task_id"])
        task_type = str(task["task_type"])
        self.cobot_goals.append(
            {
                "robot_name": robot_name,
                "task_id": task_id,
                "task_type": task_type,
            }
        )
        self.pending.append(
            PendingTaskResult(
                robot_name=robot_name,
                task_id=task_id,
                task_type=task_type,
                callback=result_callback,
            )
        )
        return True

    def cancel_task(self, robot_name: str, task_id: int) -> bool:
        """TaskManager cancel 요청을 fake로 기록한다."""
        self.cancel_requests.append({"robot_name": robot_name, "task_id": task_id})
        self.pending = [
            pending for pending in self.pending
            if not (pending.robot_name == robot_name and pending.task_id == task_id)
        ]
        return True

    def set_emergency_stop(
        self,
        robot_names: list[str] | tuple[str, ...],
        enabled: bool,
        *,
        reason: str = "DEMO",
        task_id: int = 0,
        request_id: str = "",
    ) -> dict[str, bool]:
        """EmergencyControl service 전파를 fake로 기록한다."""
        request = {
            "enabled": enabled,
            "reason": reason,
            "task_id": task_id,
            "request_id": request_id,
            "robot_names": list(robot_names),
        }
        self.emergency_requests.append(request)
        print(f"[demo] EmergencyControl fake request: {request}")
        return {robot_name: True for robot_name in robot_names}

    def pop_all_pending(self) -> list[PendingTaskResult]:
        """현재 dispatch된 pending task를 모두 꺼낸다."""
        pending = self.pending
        self.pending = []
        return pending


class DemoFleetEventBridge:
    """Control Server fleet event WebSocket을 fake gateway로 연결한다.

    실제 FleetManagerNode의 WebSocket listener를 완전히 띄우지는 않는다.
    대신 demo 안에서 `/api/fleet/ws/events`를 구독하고, `EMERGENCY_STOP`/`RESUME`
    event를 받으면 DemoRobotGateway.set_emergency_stop()을 호출한다.
    """

    def __init__(
        self,
        *,
        base_url: str,
        gateway: DemoRobotGateway,
        task_manager: TaskManager,
        robot_names: list[str],
        reconnect_sec: float = 0.5,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.gateway = gateway
        self.task_manager = task_manager
        self.robot_names = robot_names
        self.reconnect_sec = reconnect_sec
        self.events: list[dict[str, Any]] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        """background thread에서 fleet event listener를 시작한다."""
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="demo_fleet_event_bridge", daemon=True)
        self._thread.start()
        self._ready.wait(timeout=2.0)

    def stop(self) -> None:
        """listener 종료를 요청한다."""
        self._stop.set()

    def count(self) -> int:
        """수신한 fleet event 개수를 반환한다."""
        with self._lock:
            return len(self.events)

    def wait_for_event(self, event_name: str, *, after: int = 0, timeout_sec: float = 5.0) -> dict[str, Any]:
        """특정 event가 들어올 때까지 대기한다."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self._lock:
                for event in self.events[after:]:
                    if event.get("event") == event_name:
                        return event
            time.sleep(0.05)
        raise RuntimeError(f"fleet event timeout: {event_name}")

    def _run(self) -> None:
        """async websocket loop 실행 wrapper."""
        try:
            asyncio.run(self._loop())
        except Exception as exc:
            print(f"[demo][warn] fleet event bridge 종료: {exc}")

    async def _loop(self) -> None:
        """Control Server fleet event를 계속 수신한다."""
        import websockets

        ws_url = self._to_ws_url(self.base_url)
        while not self._stop.is_set():
            try:
                async with websockets.connect(ws_url) as websocket:
                    self._ready.set()
                    print(f"[demo] fleet event bridge 연결: {ws_url}")
                    while not self._stop.is_set():
                        try:
                            raw = await asyncio.wait_for(websocket.recv(), timeout=0.5)
                        except asyncio.TimeoutError:
                            continue
                        self._handle_raw_event(raw)
            except Exception as exc:
                if not self._stop.is_set():
                    print(f"[demo][warn] fleet event bridge 재연결 대기: {exc}")
                    await asyncio.sleep(self.reconnect_sec)

    def _handle_raw_event(self, raw_event: str) -> None:
        """fleet event JSON을 fake gateway 호출로 변환한다."""
        event = json.loads(raw_event)
        event_name = event.get("event")

        if event_name in ("EMERGENCY_STOP", "RESUME"):
            self.gateway.set_emergency_stop(
                self.robot_names,
                event_name == "EMERGENCY_STOP",
                reason=str(event.get("reason") or event_name),
                task_id=int(event.get("task_id") or 0),
                request_id=str(event.get("request_id") or event.get("event_id") or ""),
            )
            if event_name == "EMERGENCY_STOP":
                self.task_manager.handle_emergency_stop()
            else:
                self.task_manager.handle_resume()

        with self._lock:
            self.events.append(event)

    def _to_ws_url(self, base_url: str) -> str:
        """HTTP base URL을 fleet event WebSocket URL로 변환한다."""
        if base_url.startswith("https://"):
            base_url = "wss://" + base_url[len("https://"):]
        elif base_url.startswith("http://"):
            base_url = "ws://" + base_url[len("http://"):]
        return f"{base_url}/api/fleet/ws/events"


class DemoRunner:
    """Control Server API + Fleet Manager task logic demo runner."""

    def __init__(self, *, base_url: str, delay_sec: float, stow_delay_sec: float) -> None:
        self.base_url = base_url.rstrip("/")
        self.delay_sec = delay_sec
        self.stow_delay_sec = stow_delay_sec
        self.node = DemoNode()
        self.control = ControlServerClient(self.node, self.base_url)
        self.gateway = DemoRobotGateway()
        self.traffic = TrafficManager(
            self.node,
            ["PICKY1", "PICKY2"],
            zone_coords=self.control.fetch_zone_coords(),
        )
        self.manager = TaskManager(
            node=self.node,
            control_server=self.control,
            traffic_manager=self.traffic,
            robot_gateway=self.gateway,
        )
        self.event_bridge: DemoFleetEventBridge | None = None
        self._policy_checked_orders: set[int] = set()

    # ==================================================================
    # HTTP helpers
    # ==================================================================

    def api(self, method: str, path: str, **kwargs: Any) -> Any:
        """Control Server API를 호출하고 JSON 응답을 반환한다."""
        response = requests.request(
            method,
            f"{self.base_url}{path}",
            timeout=5,
            **kwargs,
        )
        response.raise_for_status()
        if not response.content:
            return {}
        return response.json()

    def ensure_server_ready(self) -> None:
        """Control Server가 떠 있고 기본 API가 응답하는지 확인한다."""
        products = self.api("GET", "/api/products")
        if not isinstance(products, list):
            raise RuntimeError("/api/products response is not a list")
        print(f"[demo] Control Server 연결 확인: products={len(products)}")

    # ==================================================================
    # Demo scenarios
    # ==================================================================

    def create_order(self, product_ids: list[int], quantity: int) -> int:
        """주문을 생성하고 order_id를 반환한다."""
        payload = {
            "items": [
                {"product_id": product_id, "quantity": quantity}
                for product_id in product_ids
            ]
        }
        order = self.api("POST", "/api/orders", json=payload)
        order_id = int(order["order_id"])
        print(f"[demo] 주문 생성: order_id={order_id}, product_ids={product_ids}, quantity={quantity}")
        return order_id

    def create_orders(self, *, count: int, products_per_order: int, quantity: int) -> list[int]:
        """여러 주문을 만들고 order_id 목록을 반환한다."""
        products = self.api("GET", "/api/products")
        if not products:
            raise RuntimeError("product list is empty")

        product_ids = [int(product["product_id"]) for product in products]
        available_stock = {
            int(product["product_id"]): int(product.get("stock_qty") or 0)
            for product in products
        }
        order_ids: list[int] = []

        for index in range(count):
            ids: list[int] = []
            attempts = 0
            cursor = index

            while len(ids) < products_per_order and attempts < len(product_ids) * 3:
                product_id = product_ids[cursor % len(product_ids)]
                cursor += 1
                attempts += 1

                if product_id in ids:
                    continue
                if available_stock.get(product_id, 0) < quantity:
                    continue

                ids.append(product_id)
                available_stock[product_id] -= quantity

            if len(ids) < products_per_order:
                raise RuntimeError(
                    "not enough product stock for multi-order demo: "
                    f"order_index={index}, requested_products={products_per_order}, "
                    f"remaining_stock={available_stock}"
                )

            order_ids.append(self.create_order(ids, quantity))

        return order_ids

    def create_stocking_item(self, product_id: int, quantity: int) -> int:
        """입고 요청을 생성하고 stocking_item_id를 반환한다."""
        item = self.api(
            "POST",
            "/api/fleet/stocking-items",
            json={
                "product_id": product_id,
                "requested_quantity": quantity,
                "stocking_policy": "REQUESTED_QUANTITY",
                "status": "REQUESTED",
            },
        )
        stocking_item_id = int(item["stocking_item_id"])
        print(
            f"[demo] 입고 요청 생성: stocking_item_id={stocking_item_id}, "
            f"product_id={product_id}, quantity={quantity}"
        )
        return stocking_item_id

    def poll_waiting_work_if_picky_idle(self) -> None:
        """FleetManagerNode timer와 같은 조건으로 대기 작업 확인을 수행한다."""
        self._simulate_charge_battery_updates()
        if not self.manager.has_idle_picky_for_waiting_work():
            return
        self.manager.check_waiting_work()

    def _simulate_charge_battery_updates(self) -> None:
        """데모에서 RobotStateMonitor의 battery update hook을 대체한다."""
        for task in self.control.list_tasks(status="RUNNING", task_type="CHARGE"):
            robot_name = task.get("assigned_robot_name")
            if not robot_name:
                continue
            robot = next(
                (item for item in self.control.list_robots() if item.get("robot_name") == robot_name),
                None,
            )
            if robot is None or robot.get("battery_level") is None:
                continue
            self.manager.handle_battery_update(str(robot_name), int(robot["battery_level"]))

    def run_until_order_ready(self, order_id: int, *, max_cycles: int) -> None:
        """주문이 PICKUP_READY가 될 때까지 대기 작업 확인과 fake result를 반복한다."""
        self.run_until_orders_ready([order_id], max_cycles=max_cycles, auto_complete=False)

    def run_until_orders_ready(
        self,
        order_ids: list[int],
        *,
        max_cycles: int,
        auto_complete: bool,
    ) -> None:
        """여러 주문이 PICKUP_READY 또는 COMPLETED가 될 때까지 반복한다."""
        completed: set[int] = set()

        for cycle in range(1, max_cycles + 1):
            self.poll_waiting_work_if_picky_idle()
            self._finish_pending_tasks_if_any()
            self.print_multi_order_state(order_ids, cycle=cycle)

            for order_id in order_ids:
                detail = self.api("GET", f"/api/orders/{order_id}")
                status = detail.get("status")

                if status == "PICKUP_READY" and order_id not in self._policy_checked_orders:
                    remaining = [
                        item_id for item_id in order_ids
                        if item_id != order_id and item_id not in completed
                    ]
                    self.run_pickup_ready_policy_check(
                        order_id,
                        final_order=not remaining,
                    )
                    self._policy_checked_orders.add(order_id)
                    detail = self.api("GET", f"/api/orders/{order_id}")
                    status = detail.get("status")

                if status == "PICKUP_READY" and auto_complete and order_id not in completed:
                    self.api("POST", f"/api/orders/{order_id}/complete")
                    completed.add(order_id)
                    print(f"[demo] 픽업 완료 처리: order_id={order_id}, pickup slot release")
                elif status in ("PICKUP_READY", "COMPLETED"):
                    completed.add(order_id)

            if len(completed) == len(order_ids):
                print(f"[demo] 주문 묶음 완료: order_ids={order_ids}")
                return

        raise RuntimeError(f"orders={order_ids} did not finish within {max_cycles} cycles")

    def run_emergency_resume_demo(
        self,
        *,
        order_ids: list[int],
        max_cycles: int,
        hold_sec: float,
        auto_complete: bool,
    ) -> None:
        """RUNNING task가 있는 상태에서 emergency stop/resume을 검증한다."""
        self.start_fleet_event_bridge()

        self.print_policy_snapshot("시작 전 주문/배터리 체크")

        self.poll_waiting_work_if_picky_idle()
        if not self.gateway.pending:
            raise RuntimeError("emergency demo requires at least one RUNNING/pending task")

        print("[demo] ===== EMERGENCY STOP / RESUME TEST START =====")
        print(f"[demo] emergency 전 robots=[{self.format_robot_states()}]")
        before_event_count = self.event_bridge.count() if self.event_bridge else 0
        before_request_count = len(self.gateway.emergency_requests)

        response = self.api("POST", "/api/admin/emergency-stop")
        print(f"[demo] emergency-stop 응답: {response}")
        if self.event_bridge is not None:
            self.event_bridge.wait_for_event("EMERGENCY_STOP", after=before_event_count)
        self._assert_emergency_applied(before_request_count=before_request_count)

        print(f"[demo] EMERGENCY_STOP 유지 {hold_sec:.1f}s: robots=[{self.format_robot_states()}]")
        time.sleep(hold_sec)

        before_event_count = self.event_bridge.count() if self.event_bridge else 0
        before_request_count = len(self.gateway.emergency_requests)
        response = self.api("POST", "/api/admin/resume")
        print(f"[demo] resume 응답: {response}")
        if self.event_bridge is not None:
            self.event_bridge.wait_for_event("RESUME", after=before_event_count)
        self._assert_resume_applied(response, before_request_count=before_request_count)

        self._finish_pending_tasks_if_any()
        self.run_until_orders_ready(
            order_ids,
            max_cycles=max_cycles,
            auto_complete=auto_complete,
        )
        print("[demo] ===== EMERGENCY STOP / RESUME TEST END =====")

    def run_until_stocking_completed(self, stocking_item_id: int, *, max_cycles: int) -> None:
        """입고 요청이 COMPLETED가 될 때까지 대기 작업 확인과 fake result를 반복한다."""
        for cycle in range(1, max_cycles + 1):
            self.poll_waiting_work_if_picky_idle()
            self._finish_pending_tasks_if_any()
            self.print_stocking_state(stocking_item_id, cycle=cycle)

            item = self.get_stocking_item(stocking_item_id)
            if item.get("status") == "COMPLETED":
                print(f"[demo] 입고 완료: stocking_item_id={stocking_item_id}, status=COMPLETED")
                return

        raise RuntimeError(
            f"stocking_item_id={stocking_item_id} did not reach COMPLETED within {max_cycles} cycles"
        )

    # ==================================================================
    # State display / fake result
    # ==================================================================

    def _finish_pending_tasks_if_any(self) -> None:
        """pending task 묶음을 delay_sec 동안 RUNNING으로 보여준 뒤 SUCCESS 처리한다."""
        pending_tasks = self.gateway.pop_all_pending()
        if not pending_tasks:
            return

        summary = ", ".join(
            f"task_id={pending.task_id}/{pending.robot_name}/{pending.task_type}"
            for pending in pending_tasks
        )
        print(
            f"[demo] RUNNING 유지 {self.delay_sec:.1f}s: "
            f"{summary}"
        )
        print(f"[demo] robots(before sleep)=[{self.format_robot_states()}]")
        time.sleep(self.delay_sec)

        cobot_tasks = [pending for pending in pending_tasks if pending.task_type in COBOT_TASK_TYPES]
        if cobot_tasks:
            self._show_cobot_stowing_arm(cobot_tasks)

        for pending in pending_tasks:
            pending.callback(
                {
                    "task_id": pending.task_id,
                    "robot_name": pending.robot_name,
                    "task_type": pending.task_type,
                    "success": True,
                    "message": f"{pending.task_type} completed by demo_test.py",
                }
            )
            print(f"[demo] SUCCESS 처리: task_id={pending.task_id}, type={pending.task_type}")

        print(f"[demo] robots(after success)=[{self.format_robot_states()}]")

    def run_pickup_ready_policy_check(self, order_id: int, *, final_order: bool) -> None:
        """PICKUP_READY 직후 다음 주문/배터리 판단과 후속 동작을 보여준다."""
        label = f"order_id={order_id} PICKUP_READY 직후"
        self.print_policy_snapshot(label)

        steps = 4 if final_order else 1
        for step in range(1, steps + 1):
            print(
                f"[demo][policy] {label}: 판단 cycle {step}/{steps} "
                f"({'마지막 주문이라 복귀/도킹/충전 확인' if final_order else '다음 주문 있으면 복귀 안 함 확인'})"
            )
            self.poll_waiting_work_if_picky_idle()
            self._finish_pending_tasks_if_any()
            self.print_housekeeping_state(order_id)

    def print_policy_snapshot(self, label: str) -> None:
        """주문/입고 대기열과 PICKY 배터리 기준을 출력한다."""
        waiting_orders = self.control.list_waiting_orders()
        requested_stocking = self.control.list_requested_stocking_items()
        robots = self.control.list_robots()
        picky_states = []

        for robot in robots:
            if robot.get("robot_type") != "PICKY":
                continue
            battery_level = robot.get("battery_level")
            battery_text = "unknown" if battery_level is None else f"{battery_level}%"
            can_work = battery_level is None or int(battery_level) > 40
            picky_states.append(
                f"{robot.get('robot_name')}:status={robot.get('robot_status')} "
                f"battery={battery_text} can_work={can_work}"
            )

        print(
            f"[demo][policy] {label}: "
            f"ORDER_WAIT={[order.get('order_id') for order in waiting_orders]} "
            f"REQUESTED_STOCKING={[item.get('stocking_item_id') for item in requested_stocking]} "
            f"PICKY=[{'; '.join(picky_states)}]"
        )

    def print_housekeeping_state(self, order_id: int) -> None:
        """주문에 붙은 RETURN_HOME/DOCK_IN/CHARGE 상태를 출력한다."""
        tasks = self.control.list_order_tasks(order_id)
        housekeeping = [
            task for task in tasks
            if task.get("task_type") in HOUSEKEEPING_TASK_TYPES
        ]
        if not housekeeping:
            print(f"[demo][policy] order_id={order_id} housekeeping task 없음")
            return

        summary = [
            f"#{task['sequence_no']}:{task['task_type']}={task['status']}:{task.get('result_message')}"
            for task in housekeeping
        ]
        print(f"[demo][policy] order_id={order_id} housekeeping=[{', '.join(summary)}]")

    def _show_cobot_stowing_arm(self, pending_tasks: list[PendingTaskResult]) -> None:
        """COBOT task 완료 직전 STOWING_ARM 상태를 UI에 노출한다.

        실제 운용에서는 COBOT State Manager가 팔을 기본 자세로 복귀한 뒤에만
        action result SUCCESS를 보내는 것이 가장 단순하다. 이 demo는 그 사이
        상태를 눈으로 확인하기 위해 Control Server robot 상태만 STOWING_ARM으로
        잠깐 바꾼 뒤 최종 SUCCESS callback을 호출한다.
        """
        for pending in pending_tasks:
            self.control.update_robot_state(
                pending.robot_name,
                robot_status="BUSY",
                cobot_state="STOWING_ARM",
                current_task_id=pending.task_id,
            )

        summary = ", ".join(
            f"task_id={pending.task_id}/{pending.robot_name}/{pending.task_type}"
            for pending in pending_tasks
        )
        print(
            f"[demo] STOWING_ARM 유지 {self.stow_delay_sec:.1f}s: "
            f"{summary}"
        )
        print(f"[demo] robots(stowing arm)=[{self.format_robot_states()}]")

        for pending in pending_tasks:
            preplanned = self.manager.preplan_after_cobot_stowing(pending.task_id)
            if preplanned:
                print(f"[demo] STOWING_ARM 중 다음 이동 task 선계획 완료: trigger_task_id={pending.task_id}")

        time.sleep(self.stow_delay_sec)

    def start_fleet_event_bridge(self) -> None:
        """Emergency/Resume event 확인용 demo bridge를 시작한다."""
        if self.event_bridge is not None:
            return
        self.event_bridge = DemoFleetEventBridge(
            base_url=self.base_url,
            gateway=self.gateway,
            task_manager=self.manager,
            robot_names=["PICKY1", "PICKY2", "COBOT1", "COBOT2"],
        )
        self.event_bridge.start()

    def stop(self) -> None:
        """demo runner 종료 정리."""
        if self.event_bridge is not None:
            self.event_bridge.stop()

    def _assert_emergency_applied(self, *, before_request_count: int) -> None:
        """Control Server와 fake EmergencyControl 전파 결과를 검증한다."""
        robots = self.control.list_robots()
        non_emergency = [robot for robot in robots if robot.get("robot_status") != "EMERGENCY_STOP"]
        if non_emergency:
            raise RuntimeError(f"robots not in EMERGENCY_STOP: {non_emergency}")

        paused_tasks = [task for task in self.control.list_tasks(status="PAUSED")]
        if not paused_tasks:
            raise RuntimeError("no PAUSED task after emergency-stop")

        if len(self.gateway.emergency_requests) <= before_request_count:
            raise RuntimeError("EmergencyControl fake request was not propagated")
        last_request = self.gateway.emergency_requests[-1]
        if not last_request.get("enabled"):
            raise RuntimeError(f"last emergency request is not stop: {last_request}")

        print(
            f"[demo] emergency 검증 통과: paused_tasks="
            f"{[task['task_id'] for task in paused_tasks]} robots=[{self.format_robot_states()}]"
        )

    def _assert_resume_applied(self, response: dict[str, Any], *, before_request_count: int) -> None:
        """Control Server와 fake EmergencyControl resume 결과를 검증한다."""
        emergency_robots = [
            robot for robot in self.control.list_robots()
            if robot.get("robot_status") == "EMERGENCY_STOP"
        ]
        if emergency_robots:
            raise RuntimeError(f"robots still in EMERGENCY_STOP: {emergency_robots}")

        resumed_task_ids = response.get("resumed_task_ids") or []
        for task_id in resumed_task_ids:
            task = self._find_task(int(task_id))
            if task.get("status") != "RUNNING":
                raise RuntimeError(f"resumed task is not RUNNING: {task}")

        if len(self.gateway.emergency_requests) <= before_request_count:
            raise RuntimeError("EmergencyControl fake resume request was not propagated")
        last_request = self.gateway.emergency_requests[-1]
        if last_request.get("enabled"):
            raise RuntimeError(f"last emergency request is not resume: {last_request}")

        print(f"[demo] resume 검증 통과: robots=[{self.format_robot_states()}]")

    def _find_task(self, task_id: int) -> dict[str, Any]:
        """task_id로 task summary를 찾는다."""
        for task in self.control.list_tasks():
            if int(task.get("task_id") or 0) == task_id:
                return task
        raise RuntimeError(f"task_id={task_id} not found")

    def format_robot_states(self) -> str:
        """UI에서 봐야 할 핵심 로봇 상태를 한 줄 문자열로 만든다."""
        robots = self.control.list_robots()
        parts: list[str] = []

        for robot in sorted(robots, key=lambda item: int(item.get("robot_id") or 0)):
            robot_name = robot.get("robot_name")
            robot_status = robot.get("robot_status")
            detail_state = robot.get("picky_state") or robot.get("cobot_state") or "-"
            current_task_id = robot.get("current_task_id")
            battery_level = robot.get("battery_level")
            task_text = f" task={current_task_id}" if current_task_id is not None else ""
            battery_text = f" bat={battery_level}%" if battery_level is not None else ""
            parts.append(f"{robot_name} {robot_status}/{detail_state}{task_text}{battery_text}")

        return "; ".join(parts)

    def print_order_state(self, order_id: int, *, cycle: int) -> None:
        """현재 주문/task/order_item 상태를 한 줄로 출력한다."""
        detail = self.api("GET", f"/api/orders/{order_id}")
        tasks = self.control.list_order_tasks(order_id)
        task_summary = [
            f"#{task['sequence_no']}:{task['task_type']}={task['status']}"
            for task in tasks
        ]
        item_summary = [
            f"item{item['item_id']}={item['status']}"
            for item in detail.get("items", [])
        ]
        print(
            f"[demo][cycle={cycle}] order={detail['status']} "
            f"slot={detail.get('pickup_slot_name')} "
            f"tasks=[{', '.join(task_summary)}] "
            f"items=[{', '.join(item_summary)}] "
            f"robots=[{self.format_robot_states()}]"
        )

    def print_multi_order_state(self, order_ids: list[int], *, cycle: int) -> None:
        """여러 주문 상태를 한 줄로 출력한다."""
        summaries: list[str] = []
        for order_id in order_ids:
            detail = self.api("GET", f"/api/orders/{order_id}")
            tasks = self.control.list_order_tasks(order_id)
            open_tasks = [task for task in tasks if task.get("status") not in {"SUCCESS", "FAILED", "CANCELLED"}]
            current = open_tasks[0] if open_tasks else None
            if current:
                current_text = f"{current['task_type']}={current['status']}"
            else:
                current_text = "no-open-task"
            summaries.append(
                f"order{order_id}:{detail['status']}/slot={detail.get('pickup_slot_name')}/current={current_text}"
            )

        print(
            f"[demo][cycle={cycle}] orders=[{'; '.join(summaries)}] "
            f"robots=[{self.format_robot_states()}]"
        )

    def print_stocking_state(self, stocking_item_id: int, *, cycle: int) -> None:
        """현재 입고/task 상태를 한 줄로 출력한다."""
        item = self.get_stocking_item(stocking_item_id)
        tasks = [
            task for task in self.control.list_tasks()
            if task.get("stocking_item_id") == stocking_item_id
        ]
        tasks.sort(key=lambda task: int(task.get("sequence_no") or 0))
        task_summary = [
            f"#{task['sequence_no']}:{task['task_type']}={task['status']}"
            for task in tasks
        ]
        print(
            f"[demo][cycle={cycle}] stocking={item['status']} "
            f"tasks=[{', '.join(task_summary)}] "
            f"robots=[{self.format_robot_states()}]"
        )

    def get_stocking_item(self, stocking_item_id: int) -> dict[str, Any]:
        """stocking_item 하나를 조회한다."""
        items = self.api("GET", "/api/fleet/stocking-items", params={"include_completed": True})
        for item in items:
            if int(item["stocking_item_id"]) == stocking_item_id:
                return item
        raise RuntimeError(f"stocking_item_id={stocking_item_id} not found")


def reset_demo_data() -> None:
    """web/scripts/reset_demo_data.sh를 실행해 데모 DB를 초기화한다."""
    script = REPO_ROOT / "web" / "scripts" / "reset_demo_data.sh"
    subprocess.run([str(script)], cwd=REPO_ROOT, check=True)


def parse_product_ids(value: str | None, products: list[dict[str, Any]], count: int) -> list[int]:
    """CLI product id 문자열을 int list로 변환한다."""
    if value:
        return [int(part.strip()) for part in value.split(",") if part.strip()]

    return [int(product["product_id"]) for product in products[:count]]


def build_parser() -> argparse.ArgumentParser:
    """CLI argument parser를 만든다."""
    parser = argparse.ArgumentParser(
        description="Fleet Manager demo driver. 실제 ROS2 로봇 대신 task를 지연 성공 처리한다.",
    )
    parser.add_argument("--base-url", default="http://localhost:8000", help="Control Server base URL")
    parser.add_argument("--delay-sec", type=float, default=3.0, help="각 task RUNNING 상태 유지 시간")
    parser.add_argument("--stow-delay-sec", type=float, default=3.0, help="COBOT STOWING_ARM 상태 유지 시간")
    parser.add_argument(
        "--scenario",
        choices=("order", "multi-order", "stocking", "both", "emergency", "full"),
        default="order",
        help="실행할 데모 시나리오",
    )
    parser.add_argument("--reset", action="store_true", help="실행 전 demo DB를 초기화한다")
    parser.add_argument("--order-product-ids", default=None, help="주문 상품 ID 목록. 예: 1,2,3")
    parser.add_argument("--order-product-count", type=int, default=3, help="상품 ID 미지정 시 앞에서 몇 개 상품을 주문할지")
    parser.add_argument("--order-quantity", type=int, default=1, help="주문 상품별 수량")
    parser.add_argument("--order-count", type=int, default=3, help="multi-order/emergency/full 시 생성할 주문 수")
    parser.add_argument("--products-per-order", type=int, default=3, help="multi-order/emergency/full 시 주문 1건당 상품 종류 수")
    parser.add_argument("--emergency-hold-sec", type=float, default=3.0, help="emergency 상태를 유지할 시간")
    parser.add_argument(
        "--no-auto-complete-ready-orders",
        action="store_false",
        dest="auto_complete_ready_orders",
        help="multi-order/full에서 PICKUP_READY 주문을 자동 수령 완료 처리하지 않는다",
    )
    parser.add_argument("--stocking-product-id", type=int, default=1, help="입고 요청 상품 ID")
    parser.add_argument("--stocking-quantity", type=int, default=5, help="입고 요청 수량")
    parser.add_argument("--max-cycles", type=int, default=30, help="시나리오별 최대 대기 작업 확인 횟수")
    parser.set_defaults(auto_complete_ready_orders=True)
    return parser


def main() -> int:
    """demo_test.py 진입점."""
    args = build_parser().parse_args()

    if args.reset:
        print("[demo] DB 초기화 실행")
        reset_demo_data()

    runner = DemoRunner(
        base_url=args.base_url,
        delay_sec=args.delay_sec,
        stow_delay_sec=args.stow_delay_sec,
    )
    try:
        runner.ensure_server_ready()

        products = runner.api("GET", "/api/products")
        if args.scenario == "order":
            product_ids = parse_product_ids(args.order_product_ids, products, args.order_product_count)
            order_id = runner.create_order(product_ids, args.order_quantity)
            runner.run_until_order_ready(order_id, max_cycles=args.max_cycles)

        elif args.scenario == "multi-order":
            order_ids = runner.create_orders(
                count=args.order_count,
                products_per_order=args.products_per_order,
                quantity=args.order_quantity,
            )
            runner.run_until_orders_ready(
                order_ids,
                max_cycles=args.max_cycles,
                auto_complete=args.auto_complete_ready_orders,
            )

        elif args.scenario == "emergency":
            order_ids = runner.create_orders(
                count=args.order_count,
                products_per_order=args.products_per_order,
                quantity=args.order_quantity,
            )
            runner.run_emergency_resume_demo(
                order_ids=order_ids,
                max_cycles=args.max_cycles,
                hold_sec=args.emergency_hold_sec,
                auto_complete=args.auto_complete_ready_orders,
            )

        elif args.scenario == "stocking":
            stocking_item_id = runner.create_stocking_item(args.stocking_product_id, args.stocking_quantity)
            runner.run_until_stocking_completed(stocking_item_id, max_cycles=args.max_cycles)

        elif args.scenario == "both":
            product_ids = parse_product_ids(args.order_product_ids, products, args.order_product_count)
            order_id = runner.create_order(product_ids, args.order_quantity)
            runner.run_until_order_ready(order_id, max_cycles=args.max_cycles)

            stocking_item_id = runner.create_stocking_item(args.stocking_product_id, args.stocking_quantity)
            runner.run_until_stocking_completed(stocking_item_id, max_cycles=args.max_cycles)

        elif args.scenario == "full":
            order_ids = runner.create_orders(
                count=args.order_count,
                products_per_order=args.products_per_order,
                quantity=args.order_quantity,
            )
            runner.run_emergency_resume_demo(
                order_ids=order_ids,
                max_cycles=args.max_cycles,
                hold_sec=args.emergency_hold_sec,
                auto_complete=args.auto_complete_ready_orders,
            )

            stocking_item_id = runner.create_stocking_item(args.stocking_product_id, args.stocking_quantity)
            runner.run_until_stocking_completed(stocking_item_id, max_cycles=args.max_cycles)

        print("[demo] 완료")
        return 0
    finally:
        runner.stop()


if __name__ == "__main__":
    raise SystemExit(main())
