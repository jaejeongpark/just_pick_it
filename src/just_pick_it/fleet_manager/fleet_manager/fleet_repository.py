from __future__ import annotations

from datetime import UTC, datetime
from typing import Any

from rclpy.node import Node

from just_pick_it_db.models import (
    ExceptionLog,
    Order,
    OrderItem,
    PickupSlot,
    Product,
    Robot,
    RobotUnit,
    StockingItem,
    Task,
    TaskEvent,
    Zone,
)
from just_pick_it_db.services.product_images import resolve_product_image_url
from just_pick_it_db.services.robot_runtime_policy import FINAL_TASK_STATUSES, TASK_ROBOT_TYPE
from just_pick_it_db.services.status_service import (
    build_admin_status,
    build_product_summary,
    build_robot_summary,
    build_task_summary,
    build_zone_pose,
)
from just_pick_it_db.services.stocking_service import (
    build_stocking_item_summary,
    create_stocking_item_record,
    resolve_stocking_policy,
)
from just_pick_it_db.services.workflow_service import apply_task_runtime_state
from just_pick_it_db.session import session_scope


class _RepoError(Exception):
    """Repository 내부에서 not-found / 검증 실패를 표현하는 예외.

    이전 HTTP 구현에서 Control Server 가 4xx 를 돌려주면 ControlServerClient 는 None 을
    반환했다. 같은 계약을 지키기 위해 이 예외는 public 메서드 경계에서 잡혀 경고 로그 +
    None(또는 빈 list)로 변환된다.
    """


class FleetRepository:
    """Fleet Manager 의 단일 DB 접근 계층(Repository).

    역할:
    - Fleet Manager 내부 모듈(TaskManager 등)이 주문/작업/로봇/zone/입고 데이터를
      읽고 쓰는 단일 진입점이다.
    - 이전에는 Control Server HTTP API 를 호출했으나, 통합(Phase 2) 이후에는
      just_pick_it_db 를 통해 PostgreSQL 에 직접 접근한다.
    - 비즈니스 로직(상태 전이, 스냅샷 빌드)은 just_pick_it_db.services 를 재사용한다.

    계약:
    - public 메서드의 이름/시그니처/반환 형태는 이전 ControlServerClient 와 동일하다.
      not-found / 검증 실패 시 None(또는 빈 list)을 반환한다.
    - 각 메서드는 session_scope() 로 스레드 안전한 Session 을 열고 닫는다.
      (MultiThreadedExecutor 환경에서 안전)

    이 클래스가 하지 않는 일:
    - task 순서 결정, robot unit 선택, 경로 선택. 그 판단은 TaskManager/TrafficManager 책임이다.
    """

    def __init__(self, node: Node) -> None:
        """FleetRepository 를 초기화한다.

        Args:
            node: 로그 출력을 위해 공유받는 FleetManagerNode.
        """
        self._node = node

    def _log(self):
        return self._node.get_logger()

    # ==================================================================
    # 내부 helper (이전 fleet_router 의 helper 를 DB 세션 기반으로 이식)
    # ==================================================================

    def _get_robot_by_identifier(self, db, robot_identifier: int | str | None) -> Robot | None:
        if robot_identifier is None:
            return None

        if isinstance(robot_identifier, int):
            return db.get(Robot, robot_identifier)

        if robot_identifier.isdigit():
            robot = db.get(Robot, int(robot_identifier))
            if robot:
                return robot

        return db.query(Robot).filter(Robot.robot_name == robot_identifier).first()

    def _resolve_robot(
        self,
        db,
        robot_id: int | str | None = None,
        robot_name: str | None = None,
    ) -> Robot | None:
        """payload 의 robot_id/robot_name 으로 Robot 을 찾는다.

        id 나 name 이 주어졌는데 없으면 _RepoError 를 던진다(이전 HTTP 404 에 대응).
        둘 다 None 이면 None 을 반환한다.
        """
        if robot_id is not None:
            robot = self._get_robot_by_identifier(db, robot_id)
            if not robot:
                raise _RepoError("robot not found")
            return robot

        if robot_name is not None:
            robot = db.query(Robot).filter(Robot.robot_name == robot_name).first()
            if not robot:
                raise _RepoError("robot not found")
            return robot

        return None

    def _build_task_event_response(self, db, task_event: TaskEvent) -> dict:
        robot = db.get(Robot, task_event.robot_id) if task_event.robot_id else None
        return {
            "event_id": task_event.event_id,
            "task_id": task_event.task_id,
            "robot_id": task_event.robot_id,
            "robot_name": robot.robot_name if robot else None,
            "from_status": task_event.from_status,
            "to_status": task_event.to_status,
            "event_name": task_event.event_name,
            "reason": task_event.reason,
            "created_at": task_event.created_at.isoformat() if task_event.created_at else None,
        }

    def _build_order_summary_response(self, db, order: Order) -> dict:
        pickup_slot = db.get(PickupSlot, order.pickup_slot_id) if order.pickup_slot_id else None
        current_task = (
            db.query(Task)
            .filter(
                Task.order_id == order.order_id,
                Task.status.notin_(FINAL_TASK_STATUSES),
            )
            .order_by(Task.sequence_no, Task.task_id)
            .first()
        )
        robot = (
            db.get(Robot, current_task.assigned_robot_id)
            if current_task and current_task.assigned_robot_id
            else None
        )
        return {
            "order_id": order.order_id,
            "order_no": order.order_no,
            "status": order.status,
            "priority": order.priority,
            "pickup_slot_id": order.pickup_slot_id,
            "pickup_slot_name": pickup_slot.slot_name if pickup_slot else None,
            "assigned_unit_id": order.assigned_unit_id,
            "current_task_id": current_task.task_id if current_task else None,
            "current_task_type": current_task.task_type if current_task else None,
            "current_task_status": current_task.status if current_task else None,
            "assigned_robot_id": current_task.assigned_robot_id if current_task else None,
            "assigned_robot_name": robot.robot_name if robot else None,
        }

    def _build_pickup_slot_response(self, db, pickup_slot: PickupSlot) -> dict:
        active_order = (
            db.query(Order)
            .filter(Order.pickup_slot_id == pickup_slot.slot_id)
            .filter(Order.status != "COMPLETED")
            .order_by(Order.order_id.desc())
            .first()
        )
        return {
            "slot_id": pickup_slot.slot_id,
            "slot_name": pickup_slot.slot_name,
            "status": pickup_slot.status,
            "order_id": active_order.order_id if active_order else None,
            "order_no": active_order.order_no if active_order else None,
        }

    def _release_pickup_slot_if_unused(self, db, slot_id: int | None) -> None:
        if slot_id is None:
            return
        has_active_order = (
            db.query(Order)
            .filter(Order.pickup_slot_id == slot_id)
            .filter(Order.status != "COMPLETED")
            .first()
            is not None
        )
        if has_active_order:
            return
        pickup_slot = db.get(PickupSlot, slot_id)
        if pickup_slot and pickup_slot.status != "BLOCKED":
            pickup_slot.status = "EMPTY"

    def _sync_order_pickup_slot(self, db, order: Order, previous_slot_id: int | None) -> None:
        if previous_slot_id != order.pickup_slot_id:
            self._release_pickup_slot_if_unused(db, previous_slot_id)
        if order.pickup_slot_id is None:
            return
        pickup_slot = db.get(PickupSlot, order.pickup_slot_id)
        if not pickup_slot or pickup_slot.status == "BLOCKED":
            return
        if order.status == "COMPLETED":
            pickup_slot.status = "EMPTY"
        elif order.status == "PICKUP_READY":
            pickup_slot.status = "OCCUPIED"
        else:
            pickup_slot.status = "RESERVED"

    def _validate_robot_unit(self, db, unit_id: int | None) -> None:
        if unit_id is not None and not db.get(RobotUnit, unit_id):
            raise _RepoError("robot unit not found")

    def _validate_task_robot_type(self, robot: Robot | None, task_type: str) -> None:
        if robot is None:
            return
        expected_robot_type = TASK_ROBOT_TYPE.get(task_type)
        if expected_robot_type and robot.robot_type != expected_robot_type:
            raise _RepoError(f"{task_type} task must be assigned to {expected_robot_type}")

    def _validate_task_refs(self, db, task: dict) -> Robot | None:
        stocking_item_id = task.get("stocking_item_id")
        order_id = task.get("order_id")
        order_item_id = task.get("order_item_id")

        if stocking_item_id is not None:
            if order_id is not None or order_item_id is not None:
                raise _RepoError("stocking task cannot reference order or order_item")
            if not db.get(StockingItem, stocking_item_id):
                raise _RepoError("stocking item not found")

        if order_id is not None and not db.get(Order, order_id):
            raise _RepoError("order not found")

        order_item = db.get(OrderItem, order_item_id) if order_item_id is not None else None
        if order_item_id is not None and not order_item:
            raise _RepoError("order item not found")
        if order_item and order_id is not None and order_item.order_id != order_id:
            raise _RepoError("order item does not belong to order")

        if task.get("source_zone_id") is not None and not db.get(Zone, task["source_zone_id"]):
            raise _RepoError("source zone not found")
        if task.get("target_zone_id") is not None and not db.get(Zone, task["target_zone_id"]):
            raise _RepoError("target zone not found")

        robot = self._resolve_robot(
            db,
            robot_id=task.get("assigned_robot_id"),
            robot_name=task.get("assigned_robot_name"),
        )
        self._validate_task_robot_type(robot, task["task_type"])
        return robot

    def _resolve_task_sequence_no(self, db, task: dict, robot: Robot | None) -> int:
        if task.get("sequence_no") is not None:
            return task["sequence_no"]

        task_query = db.query(Task)
        if task.get("stocking_item_id") is not None:
            task_query = task_query.filter(Task.stocking_item_id == task["stocking_item_id"])
        elif task.get("order_id") is not None:
            task_query = task_query.filter(Task.order_id == task["order_id"])
        elif task.get("order_item_id") is not None:
            task_query = task_query.filter(Task.order_item_id == task["order_item_id"])
        else:
            task_query = task_query.filter(
                Task.order_id.is_(None),
                Task.order_item_id.is_(None),
                Task.stocking_item_id.is_(None),
            )
            if robot is not None:
                task_query = task_query.filter(Task.assigned_robot_id == robot.robot_id)

        previous_task = task_query.order_by(Task.sequence_no.desc(), Task.task_id.desc()).first()
        return (previous_task.sequence_no if previous_task else 0) + 1

    # ==================================================================
    # Zone / Product 조회
    # ==================================================================

    def get_snapshot(self) -> dict[str, Any] | None:
        """Fleet Manager 용 전체 상태 snapshot 을 조회한다."""
        with session_scope() as db:
            return build_admin_status(db)

    def list_robots(self) -> list[dict[str, Any]]:
        """robot 목록을 조회한다."""
        with session_scope() as db:
            robots = db.query(Robot).order_by(Robot.unit_id, Robot.robot_id).all()
            return [build_robot_summary(db, robot) for robot in robots]

    def list_zones(self, zone_type: str = "ALL") -> list[dict[str, Any]]:
        """zone 목록을 조회한다."""
        with session_scope() as db:
            return self._query_zones(db, zone_type)

    def _query_zones(self, db, zone_type: str) -> list[dict[str, Any]]:
        zone_query = db.query(Zone)
        normalized = zone_type.upper() if zone_type else None
        if normalized and normalized != "ALL":
            zone_query = zone_query.filter(Zone.zone_type == normalized)
        zones = zone_query.order_by(Zone.zone_type, Zone.zone_name).all()
        return [
            {
                "zone_id": zone.zone_id,
                "zone_name": zone.zone_name,
                "zone_type": zone.zone_type,
                "pose": build_zone_pose(zone),
            }
            for zone in zones
        ]

    def fetch_zone_coords(self) -> dict[str, tuple[float, float]]:
        """전체 zone 의 좌표를 TrafficManager 용 dict 로 변환한다."""
        zones = self.list_zones(zone_type="ALL")
        coords: dict[str, tuple[float, float]] = {}
        for zone in zones:
            pose = zone.get("pose") or {}
            x = pose.get("x")
            y = pose.get("y")
            if x is None or y is None:
                continue
            zone_name = zone.get("zone_name")
            if not zone_name:
                continue
            coords[str(zone_name)] = (float(x), float(y))
        self._log().info(f"[FleetRepository] zone 좌표 조회 완료: {len(coords)}개")
        return coords

    def get_zone_map(self) -> dict[str, dict[str, Any]]:
        """zone_name 을 key 로 하는 zone map 을 만든다."""
        with session_scope() as db:
            return self._zone_map(db)

    def _zone_map(self, db) -> dict[str, dict[str, Any]]:
        zones = self._query_zones(db, "ALL")
        return {str(zone["zone_name"]): zone for zone in zones if zone.get("zone_name") is not None}

    def list_products(self) -> list[dict[str, Any]]:
        """상품 목록을 조회한다."""
        with session_scope() as db:
            return self._products(db)

    def _products(self, db) -> list[dict[str, Any]]:
        products = db.query(Product).order_by(Product.product_id).all()
        return [build_product_summary(db, product) for product in products]

    def get_product_map(self) -> dict[int, dict[str, Any]]:
        """product_id 를 key 로 하는 product map 을 만든다."""
        with session_scope() as db:
            return self._product_map(db)

    def _product_map(self, db) -> dict[int, dict[str, Any]]:
        result: dict[int, dict[str, Any]] = {}
        for product in self._products(db):
            product_id = product.get("product_id")
            if product_id is None:
                continue
            result[int(product_id)] = product
        return result

    # ==================================================================
    # Order 조회
    # ==================================================================

    def list_orders(
        self,
        *,
        status: str | None = None,
        include_completed: bool = False,
    ) -> list[dict[str, Any]]:
        """Fleet Manager 가 처리할 주문 목록을 조회한다."""
        with session_scope() as db:
            order_query = db.query(Order)
            if status is not None:
                order_query = order_query.filter(Order.status == status)
            elif not include_completed:
                order_query = order_query.filter(Order.status.notin_(("COMPLETED", "ERROR")))
            orders = order_query.order_by(Order.priority, Order.order_id).limit(50).all()
            return [self._build_order_summary_response(db, order) for order in orders]

    def list_waiting_orders(self) -> list[dict[str, Any]]:
        """ORDER_WAIT 상태의 주문 목록을 조회한다."""
        return self.list_orders(status="ORDER_WAIT")

    def list_order_tasks(self, order_id: int) -> list[dict[str, Any]]:
        """특정 주문에 생성된 task 목록을 조회한다."""
        with session_scope() as db:
            if not db.get(Order, order_id):
                return []
            tasks = (
                db.query(Task)
                .filter(Task.order_id == order_id)
                .order_by(Task.sequence_no, Task.task_id)
                .all()
            )
            return [build_task_summary(db, task) for task in tasks]

    def list_tasks(
        self,
        *,
        status: str | None = None,
        robot_name: str | None = None,
        task_type: str | None = None,
        order_id: int | None = None,
    ) -> list[dict[str, Any]]:
        """Fleet task 목록을 조회한다."""
        with session_scope() as db:
            task_query = db.query(Task)

            if robot_name is not None:
                robot = db.query(Robot).filter(Robot.robot_name == robot_name).first()
                if not robot:
                    return []
                task_query = task_query.filter(Task.assigned_robot_id == robot.robot_id)

            if status is not None:
                task_query = task_query.filter(Task.status == status)
            if task_type is not None:
                task_query = task_query.filter(Task.task_type == task_type)
            if order_id is not None:
                task_query = task_query.filter(Task.order_id == order_id)

            tasks = task_query.order_by(Task.priority, Task.sequence_no, Task.task_id).all()
            return [build_task_summary(db, task) for task in tasks]

    def get_order_detail(self, order_id: int) -> dict[str, Any] | None:
        """주문 상세를 조회한다(이전 /api/orders/{id} 응답과 동형)."""
        with session_scope() as db:
            return self._order_detail(db, order_id)

    def _order_detail(self, db, order_id: int) -> dict[str, Any] | None:
        order = db.get(Order, order_id)
        if not order:
            return None

        pickup_slot_name = None
        if order.pickup_slot_id:
            pickup_slot = db.get(PickupSlot, order.pickup_slot_id)
            if pickup_slot:
                pickup_slot_name = pickup_slot.slot_name

        order_items = (
            db.query(OrderItem, Product)
            .join(Product, OrderItem.product_id == Product.product_id)
            .filter(OrderItem.order_id == order.order_id)
            .order_by(OrderItem.item_id)
            .all()
        )

        return {
            "order_id": order.order_id,
            "order_no": order.order_no,
            "status": order.status,
            "priority": order.priority,
            "pickup_slot_id": order.pickup_slot_id,
            "pickup_slot_name": pickup_slot_name,
            "assigned_unit_id": order.assigned_unit_id,
            "items": [
                {
                    "item_id": item.item_id,
                    "product_id": item.product_id,
                    "product_name": product.name,
                    "image_url": resolve_product_image_url(product),
                    "quantity": item.quantity,
                    "status": item.status,
                }
                for item, product in order_items
            ],
        }

    # ==================================================================
    # Order / Robot / Task 상태 변경
    # ==================================================================

    def update_order_status(
        self,
        order_id: int,
        *,
        status: str | None = None,
        assigned_unit_id: int | None = None,
        pickup_slot_id: int | None = None,
    ) -> dict[str, Any] | None:
        """주문 상태, 담당 robot_unit, pickup_slot 배정을 갱신한다."""
        if status is None and assigned_unit_id is None and pickup_slot_id is None:
            self._log().warn(f"[FleetRepository] order_id={order_id} 상태 변경 인자 없음")
            return None

        try:
            with session_scope() as db:
                order = db.get(Order, order_id)
                if not order:
                    raise _RepoError("order not found")

                previous_slot_id = order.pickup_slot_id

                if status is not None:
                    order.status = status

                if pickup_slot_id is not None:
                    if not db.get(PickupSlot, pickup_slot_id):
                        raise _RepoError("pickup slot not found")
                    order.pickup_slot_id = pickup_slot_id

                if assigned_unit_id is not None:
                    order.assigned_unit_id = assigned_unit_id

                self._sync_order_pickup_slot(db, order, previous_slot_id)
                return {"status": "ok"}
        except _RepoError as exc:
            self._log().warn(f"[FleetRepository] order_id={order_id} 상태 변경 실패: {exc}")
            return None

    def update_robot_state(
        self,
        robot_name: str,
        *,
        robot_status: str | None = None,
        picky_state: str | None = None,
        cobot_state: str | None = None,
        current_task_id: int | None = None,
        battery_level: int | None = None,
        pos_x: float | None = None,
        pos_y: float | None = None,
        pos_theta: float | None = None,
    ) -> dict[str, Any] | None:
        """로봇의 런타임 상태를 갱신한다."""
        try:
            with session_scope() as db:
                robot = self._get_robot_by_identifier(db, robot_name)
                if not robot:
                    raise _RepoError("robot not found")

                if robot_status is not None:
                    robot.robot_status = robot_status

                if picky_state is not None:
                    if robot.robot_type != "PICKY":
                        raise _RepoError("picky_state is only for PICKY")
                    robot.picky_state = picky_state

                if cobot_state is not None:
                    if robot.robot_type != "COBOT":
                        raise _RepoError("cobot_state is only for COBOT")
                    robot.cobot_state = cobot_state

                if current_task_id is not None:
                    if not db.get(Task, current_task_id):
                        raise _RepoError("task not found")
                    robot.current_task_id = current_task_id

                if battery_level is not None:
                    robot.battery_level = battery_level
                if pos_x is not None:
                    robot.pos_x = pos_x
                if pos_y is not None:
                    robot.pos_y = pos_y
                if pos_theta is not None:
                    robot.pos_theta = pos_theta

                return {"status": "ok"}
        except _RepoError as exc:
            self._log().warn(f"[FleetRepository] robot={robot_name} 상태 변경 실패: {exc}")
            return None

    def update_task_status(
        self,
        task_id: int,
        *,
        status: str,
        current_status: str | None = None,
        assigned_robot_id: int | str | None = None,
        assigned_robot_name: str | None = None,
        result_message: str | None = None,
    ) -> dict[str, Any] | None:
        """task 상태를 갱신한다(상태 전이는 apply_task_runtime_state 를 거친다)."""
        try:
            with session_scope() as db:
                task = (
                    db.query(Task)
                    .filter(Task.task_id == task_id)
                    .with_for_update()
                    .one_or_none()
                )
                if not task:
                    raise _RepoError("task not found")

                if current_status is not None and task.status != current_status:
                    raise _RepoError(
                        f"task status conflict (expected={current_status}, current={task.status})"
                    )

                previous_status = task.status
                task.status = status

                robot = self._resolve_robot(
                    db,
                    robot_id=assigned_robot_id,
                    robot_name=assigned_robot_name,
                )
                if robot is not None:
                    task.assigned_robot_id = robot.robot_id

                if result_message is not None:
                    task.result_message = result_message

                if previous_status != task.status:
                    db.add(
                        TaskEvent(
                            task_id=task.task_id,
                            robot_id=task.assigned_robot_id,
                            from_status=previous_status,
                            to_status=task.status,
                            event_name=f"TASK_{task.status}",
                            reason=task.result_message,
                            created_at=datetime.now(UTC),
                        )
                    )

                apply_task_runtime_state(db, task, previous_status=previous_status)
                return {
                    "status": "ok",
                    "previous_status": previous_status,
                    "current_status": task.status,
                }
        except _RepoError as exc:
            self._log().warn(f"[FleetRepository] task_id={task_id} 상태 변경 실패: {exc}")
            return None

    def create_task_event(
        self,
        task_id: int,
        *,
        to_status: str,
        from_status: str | None = None,
        event_name: str | None = None,
        reason: str | None = None,
        robot_id: int | str | None = None,
        robot_name: str | None = None,
        update_task_status: bool = True,
    ) -> dict[str, Any] | None:
        """task_event 를 기록한다(update_task_status=True 면 task 상태도 전이한다)."""
        try:
            with session_scope() as db:
                task = (
                    db.query(Task)
                    .filter(Task.task_id == task_id)
                    .with_for_update()
                    .one_or_none()
                )
                if not task:
                    raise _RepoError("task not found")

                robot = self._resolve_robot(db, robot_id=robot_id, robot_name=robot_name)

                if (
                    update_task_status
                    and from_status is not None
                    and task.status != from_status
                ):
                    raise _RepoError(
                        f"task status conflict (expected={from_status}, current={task.status})"
                    )

                effective_from = from_status or task.status
                task_event = TaskEvent(
                    task_id=task.task_id,
                    robot_id=robot.robot_id if robot else task.assigned_robot_id,
                    from_status=effective_from,
                    to_status=to_status,
                    event_name=event_name,
                    reason=reason,
                    created_at=datetime.now(UTC),
                )
                db.add(task_event)

                if update_task_status:
                    previous_status = task.status
                    task.status = to_status
                    apply_task_runtime_state(db, task, previous_status=previous_status)

                db.flush()
                return self._build_task_event_response(db, task_event)
        except _RepoError as exc:
            self._log().warn(f"[FleetRepository] task_id={task_id} event 생성 실패: {exc}")
            return None

    # ==================================================================
    # Pickup Slot
    # ==================================================================

    def list_pickup_slots(self, status: str | None = None) -> list[dict[str, Any]]:
        """pickup slot 목록을 조회한다."""
        with session_scope() as db:
            slot_query = db.query(PickupSlot)
            if status is not None:
                slot_query = slot_query.filter(PickupSlot.status == status)
            pickup_slots = slot_query.order_by(PickupSlot.slot_id).all()
            return [self._build_pickup_slot_response(db, slot) for slot in pickup_slots]

    def assign_pickup_slot(self, order_id: int, slot_id: int) -> dict[str, Any] | None:
        """주문에 특정 pickup slot 을 배정한다."""
        return self.update_order_status(order_id, pickup_slot_id=slot_id)

    def update_pickup_slot_status(self, slot_id: int, status: str) -> dict[str, Any] | None:
        """pickup slot 상태를 직접 변경한다."""
        try:
            with session_scope() as db:
                pickup_slot = db.get(PickupSlot, slot_id)
                if not pickup_slot:
                    raise _RepoError("pickup slot not found")
                pickup_slot.status = status
                return {"status": "ok"}
        except _RepoError as exc:
            self._log().warn(f"[FleetRepository] pickup_slot_id={slot_id} 상태 변경 실패: {exc}")
            return None

    # ==================================================================
    # Task 생성
    # ==================================================================

    def create_tasks_bulk(self, tasks: list[dict[str, Any]]) -> dict[str, Any] | None:
        """task 여러 개를 한 번에 생성한다."""
        if not tasks:
            self._log().warn("[FleetRepository] create_tasks_bulk 호출에 빈 task 목록 전달")
            return None

        try:
            with session_scope() as db:
                created_task_ids: list[int] = []
                for task in tasks:
                    robot = self._validate_task_refs(db, task)
                    new_task = Task(
                        order_id=task.get("order_id"),
                        order_item_id=task.get("order_item_id"),
                        stocking_item_id=task.get("stocking_item_id"),
                        sequence_no=self._resolve_task_sequence_no(db, task, robot),
                        assigned_robot_id=robot.robot_id if robot else None,
                        task_type=task["task_type"],
                        status=task.get("status", "QUEUED"),
                        priority=task.get("priority", 2),
                        source_zone_id=task.get("source_zone_id"),
                        target_zone_id=task.get("target_zone_id"),
                        result_message=task.get("result_message"),
                    )
                    db.add(new_task)
                    db.flush()
                    created_task_ids.append(new_task.task_id)

                result = {
                    "status": "ok",
                    "task_ids": created_task_ids,
                    "created_count": len(created_task_ids),
                }
        except _RepoError as exc:
            self._log().warn(f"[FleetRepository] task 일괄 생성 실패: {exc}")
            return None

        if len(result["task_ids"]) != len(tasks):
            self._log().warn(
                "[FleetRepository] task 생성 요청 개수와 생성 task_ids 개수 불일치: "
                f"requested={len(tasks)}, created={len(result['task_ids'])}"
            )
        return result

    # ==================================================================
    # Exception
    # ==================================================================

    def create_exception(
        self,
        *,
        exception_type: str,
        robot_id: int | None = None,
        robot_name: str | None = None,
        task_id: int | None = None,
        order_id: int | None = None,
        detail: str | None = None,
    ) -> dict[str, Any] | None:
        """exception_log 를 생성한다."""
        try:
            with session_scope() as db:
                robot = self._resolve_robot(db, robot_id=robot_id, robot_name=robot_name)

                if task_id is not None and not db.get(Task, task_id):
                    raise _RepoError("task not found")
                if order_id is not None and not db.get(Order, order_id):
                    raise _RepoError("order not found")

                exception = ExceptionLog(
                    robot_id=robot.robot_id if robot else None,
                    task_id=task_id,
                    order_id=order_id,
                    exception_type=exception_type,
                    detail=detail,
                    is_resolved=False,
                    created_at=datetime.now(UTC),
                )
                db.add(exception)
                db.flush()
                return {"status": "ok", "exception_id": exception.exception_id}
        except _RepoError as exc:
            self._log().warn(f"[FleetRepository] exception 생성 실패: {exc}")
            return None

    # ==================================================================
    # Stocking
    # ==================================================================

    def list_requested_stocking_items(self) -> list[dict[str, Any]]:
        """REQUESTED 상태의 stocking_item 목록을 조회한다."""
        with session_scope() as db:
            stocking_items = (
                db.query(StockingItem)
                .filter(StockingItem.status == "REQUESTED")
                .order_by(StockingItem.stocking_item_id.desc())
                .limit(50)
                .all()
            )
            return [build_stocking_item_summary(db, item) for item in stocking_items]

    def update_stocking_item(
        self,
        stocking_item_id: int,
        *,
        status: str | None = None,
        assigned_unit_id: int | None = None,
        detected_quantity: int | None = None,
        stock_delta: int | None = None,
    ) -> dict[str, Any] | None:
        """stocking_item 상태나 입고 수량 정보를 갱신한다."""
        if (
            status is None
            and assigned_unit_id is None
            and detected_quantity is None
            and stock_delta is None
        ):
            self._log().warn(f"[FleetRepository] stocking_item_id={stocking_item_id} 변경 인자 없음")
            return None

        try:
            with session_scope() as db:
                stocking_item = (
                    db.query(StockingItem)
                    .filter(StockingItem.stocking_item_id == stocking_item_id)
                    .with_for_update()
                    .one_or_none()
                )
                if not stocking_item:
                    raise _RepoError("stocking item not found")

                if assigned_unit_id is not None:
                    self._validate_robot_unit(db, assigned_unit_id)
                    stocking_item.assigned_unit_id = assigned_unit_id

                if detected_quantity is not None:
                    stocking_item.detected_quantity = detected_quantity
                if stock_delta is not None:
                    stocking_item.stock_delta = stock_delta
                if status is not None:
                    stocking_item.status = status

                db.flush()
                return build_stocking_item_summary(db, stocking_item)
        except _RepoError as exc:
            self._log().warn(
                f"[FleetRepository] stocking_item_id={stocking_item_id} 상태 변경 실패: {exc}"
            )
            return None

    def complete_stocking(
        self,
        *,
        task_id: int,
        detected_quantity: int,
        stock_delta: int,
        result_message: str | None = None,
    ) -> dict[str, Any] | None:
        """입고 완료를 기록한다(다중 테이블 갱신을 단일 트랜잭션으로 처리)."""
        try:
            with session_scope() as db:
                task = (
                    db.query(Task)
                    .filter(Task.task_id == task_id)
                    .with_for_update()
                    .one_or_none()
                )
                if not task:
                    raise _RepoError("task not found")
                if task.task_type != "STOCKING_PLACE":
                    raise _RepoError("task is not STOCKING_PLACE")
                if task.stocking_item_id is None:
                    raise _RepoError("stocking item is not linked to task")

                stocking_item = (
                    db.query(StockingItem)
                    .filter(StockingItem.stocking_item_id == task.stocking_item_id)
                    .with_for_update()
                    .one_or_none()
                )
                if not stocking_item:
                    raise _RepoError("stocking item not found")

                product = db.get(Product, stocking_item.product_id)
                if not product:
                    raise _RepoError("product not found")

                if detected_quantity is not None:
                    stocking_item.detected_quantity = detected_quantity
                if stock_delta is not None:
                    stocking_item.stock_delta = stock_delta

                previous_status = task.status
                task.status = "SUCCESS"
                if result_message is not None:
                    task.result_message = result_message

                apply_task_runtime_state(db, task, previous_status=previous_status)
                applied_stock_delta = stocking_item.stock_delta

                db.add(
                    TaskEvent(
                        task_id=task.task_id,
                        robot_id=task.assigned_robot_id,
                        from_status=previous_status,
                        to_status=task.status,
                        event_name="STOCKING_COMPLETED",
                        reason=task.result_message,
                        created_at=datetime.now(UTC),
                    )
                )
                return {
                    "status": "ok",
                    "task_id": task.task_id,
                    "stocking_item_id": stocking_item.stocking_item_id,
                    "product_id": product.product_id,
                    "stock_delta": applied_stock_delta,
                    "stock_qty": product.stock_qty,
                }
        except _RepoError as exc:
            self._log().warn(f"[FleetRepository] task_id={task_id} 입고 완료 실패: {exc}")
            return None

    # ==================================================================
    # 정규화 helpers (이전 ControlServerClient 와 동일, 저수준 조회만 DB 기반)
    # ==================================================================

    def get_order_work(self, order_id: int) -> dict[str, Any] | None:
        """주문 상세를 TaskManager 가 쓰기 좋은 dict 로 정규화한다(단일 트랜잭션).

        주문 상세/상품맵/zone맵을 하나의 session_scope 안에서 일관되게 읽고,
        이후 dict 가공은 세션 밖 순수 파이썬으로 처리한다.
        """
        with session_scope() as db:
            order = self._order_detail(db, order_id)
            if order is None:
                return None
            product_map = self._product_map(db)
            zone_map = self._zone_map(db)

        items: list[dict[str, Any]] = []
        raw_items = order.get("items") or []
        if not isinstance(raw_items, list):
            self._log().warn(f"[FleetRepository] order_id={order_id} items 응답이 list가 아님")
            return None

        for raw_item in raw_items:
            if not isinstance(raw_item, dict):
                continue
            product_id = raw_item.get("product_id")
            if product_id is None:
                self._log().warn(
                    f"[FleetRepository] order_id={order_id} item에 product_id 없음: {raw_item}"
                )
                continue

            product = product_map.get(int(product_id), {})
            product_slot_name = (
                raw_item.get("storage_zone_name")
                or raw_item.get("product_slot_name")
                or product.get("storage_zone_name")
            )
            product_zone_name = self._slot_name_to_zone_name(product_slot_name)

            if product_slot_name is None or product_zone_name is None:
                self._log().warn(
                    f"[FleetRepository] product_id={product_id} zone/slot 매핑 실패: "
                    f"product_slot_name={product_slot_name}"
                )
                continue

            product_slot = zone_map.get(product_slot_name, {})
            product_zone = zone_map.get(product_zone_name, {})

            items.append(
                {
                    "order_item_id": raw_item.get("order_item_id") or raw_item.get("item_id"),
                    "product_id": int(product_id),
                    "product_name": (raw_item.get("product_name") or raw_item.get("name") or product.get("name")),
                    "quantity": int(raw_item.get("quantity") or 0),
                    "product_zone_id": product_zone.get("zone_id"),
                    "product_zone_name": product_zone_name,
                    "product_slot_id": product_slot.get("zone_id"),
                    "product_slot_name": product_slot_name,
                    "status": raw_item.get("status") or "WAITING",
                }
            )

        if not items:
            self._log().warn(f"[FleetRepository] order_id={order_id} 정규화 결과 item 없음")
            return None

        assigned_unit_id = order.get("assigned_unit_id")
        picky_name, cobot_name = self._unit_id_to_robot_names(assigned_unit_id)

        pickup_slot_name = order.get("pickup_slot_name")
        pickup_zone_name = self._slot_name_to_zone_name(pickup_slot_name)

        return {
            "order_id": order.get("order_id"),
            "order_no": order.get("order_no"),
            "priority": order.get("priority") or 2,
            "assigned_unit_id": assigned_unit_id,
            "picky_name": picky_name,
            "cobot_name": cobot_name,
            "pickup_slot_id": order.get("pickup_slot_id"),
            "pickup_slot_name": pickup_slot_name,
            "pickup_zone_name": pickup_zone_name,
            "items": items,
        }

    def get_stocking_work(self, stocking_item: dict[str, Any]) -> dict[str, Any] | None:
        """stocking_item 을 TaskManager 가 쓰기 좋은 dict 로 정규화한다."""
        product_id = stocking_item.get("product_id")
        if product_id is None:
            self._log().warn(f"[FleetRepository] stocking_item에 product_id 없음: {stocking_item}")
            return None

        with session_scope() as db:
            product_map = self._product_map(db)
            zone_map = self._zone_map(db)
        product = product_map.get(int(product_id), {})

        product_slot_name = stocking_item.get("storage_zone_name") or product.get("storage_zone_name")
        product_zone_name = self._slot_name_to_zone_name(product_slot_name)

        if product_slot_name is None or product_zone_name is None:
            self._log().warn(
                f"[FleetRepository] stocking_item product_id={product_id} zone/slot 매핑 실패"
            )
            return None

        assigned_unit_id = stocking_item.get("assigned_unit_id")
        picky_name, cobot_name = self._unit_id_to_robot_names(assigned_unit_id)

        product_slot = zone_map.get(product_slot_name, {})
        product_zone = zone_map.get(product_zone_name, {})
        stock_zone = zone_map.get("STOCK_ZONE", {})
        stock_slot = zone_map.get("STOCK_SLOT", {})

        return {
            "stocking_item_id": stocking_item.get("stocking_item_id"),
            "product_id": int(product_id),
            "product_name": (stocking_item.get("product_name") or product.get("name")),
            "requested_quantity": stocking_item.get("requested_quantity"),
            "detected_quantity": stocking_item.get("detected_quantity"),
            "stock_delta": stocking_item.get("stock_delta"),
            "stocking_policy": stocking_item.get("stocking_policy"),
            "priority": stocking_item.get("priority") or 2,
            "assigned_unit_id": assigned_unit_id,
            "picky_name": picky_name,
            "cobot_name": cobot_name,
            "stock_zone_id": stock_zone.get("zone_id"),
            "stock_zone_name": "STOCK_ZONE",
            "stock_slot_id": stock_slot.get("zone_id"),
            "stock_slot_name": "STOCK_SLOT",
            "product_zone_id": product_zone.get("zone_id"),
            "product_zone_name": product_zone_name,
            "product_slot_id": product_slot.get("zone_id"),
            "product_slot_name": product_slot_name,
        }

    def _slot_name_to_zone_name(self, slot_name: str | None) -> str | None:
        """SLOT 이름을 대응되는 PICKY 정차 ZONE 이름으로 변환한다."""
        if slot_name is None:
            return None
        if slot_name.startswith("PRODUCT_SLOT_"):
            return slot_name.replace("PRODUCT_SLOT_", "PRODUCT_ZONE_", 1)
        if slot_name.startswith("PICKUP_SLOT_"):
            return slot_name.replace("PICKUP_SLOT_", "PICKUP_ZONE_", 1)
        if slot_name == "STOCK_SLOT":
            return "STOCK_ZONE"
        if slot_name.endswith("_ZONE") or "_ZONE_" in slot_name:
            return slot_name
        return None

    def _unit_id_to_robot_names(
        self,
        assigned_unit_id: int | None,
    ) -> tuple[str | None, str | None]:
        """robot_unit id 를 PICKY/COBOT 이름으로 변환한다."""
        if assigned_unit_id == 1:
            return "PICKY1", "COBOT1"
        if assigned_unit_id == 2:
            return "PICKY2", "COBOT2"
        return None, None
