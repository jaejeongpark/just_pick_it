from __future__ import annotations

from typing import Any

import requests
from rclpy.node import Node


class ControlServerClient:
    """Control Server HTTP API 클라이언트.

    역할:
    - Fleet Manager 내부에서 Control Server와 통신하는 단일 진입점이다.
    - TaskManager가 HTTP URL, JSON 응답 구조, timeout, status code 처리를 직접 알지 않게 한다.
    - Control Server 응답을 TaskManager가 쓰기 좋은 dict/list 형태로 정리한다.

    주의:
    - 이 클래스는 task 순서 결정, robot unit 선택, 경로 선택을 하지 않는다.
    - 그런 판단은 TaskManager 또는 TrafficManager 책임이다.
    """

    def __init__(
        self,
        node: Node,
        base_url: str,
        timeout_sec: float = 5.0,
    ) -> None:
        """ControlServerClient를 초기화한다.

        Args:
            node: 로그 출력을 위해 공유받는 FleetManagerNode.
            base_url: Control Server base URL. 예: http://192.168.4.1:8000
            timeout_sec: HTTP 요청 timeout 초.
        """
        self._node = node
        self._base_url = base_url.rstrip("/")
        self._timeout = timeout_sec

    # ==================================================================
    # HTTP 공통 처리
    # ==================================================================

    def _url(self, path: str) -> str:
        """상대 API path를 Control Server 전체 URL로 변환한다."""
        if not path.startswith("/"):
            path = f"/{path}"
        return f"{self._base_url}{path}"

    def _request_json(
        self,
        method: str,
        path: str,
        *,
        params: dict[str, Any] | None = None,
        payload: dict[str, Any] | None = None,
        ok_statuses: tuple[int, ...] = (200,),
    ) -> Any | None:
        """공통 HTTP 요청 처리 함수.

        모든 GET/POST/PATCH 요청은 이 함수를 통과한다.

        처리:
        - timeout 적용
        - 네트워크 예외 로그
        - HTTP status code 검증
        - JSON 파싱
        - 실패 시 None 반환

        Args:
            method: HTTP method. GET, POST, PATCH.
            path: API path. 예: /api/fleet/orders
            params: query parameter.
            payload: JSON body.
            ok_statuses: 성공으로 인정할 HTTP status code 목록.

        Returns:
            성공 시 JSON 응답 객체.
            실패 시 None.
        """
        url = self._url(path)

        try:
            resp = requests.request(
                method,
                url,
                params=params,
                json=payload,
                timeout=self._timeout,
            )
        except requests.exceptions.RequestException as exc:
            self._node.get_logger().warn(f"[ControlServerClient] {method} {path} 요청 오류: {exc}")
            return None

        if resp.status_code not in ok_statuses:
            body = resp.text[:300]
            self._node.get_logger().warn(
                f"[ControlServerClient] {method} {path} 실패: "
                f"HTTP {resp.status_code}, body={body}, payload={payload}"
            )
            return None

        if not resp.content:
            return {}

        try:
            return resp.json()
        except ValueError as exc:
            self._node.get_logger().warn(f"[ControlServerClient] {method} {path} JSON 파싱 실패: {exc}")
            return None

    def _get_json(
        self,
        path: str,
        params: dict[str, Any] | None = None,
    ) -> Any | None:
        """GET 요청을 보내고 JSON 응답을 반환한다."""
        return self._request_json("GET", path, params=params)

    def _post_json(
        self,
        path: str,
        payload: dict[str, Any] | None = None,
        *,
        ok_statuses: tuple[int, ...] = (200, 201),
    ) -> Any | None:
        """POST 요청을 보내고 JSON 응답을 반환한다."""
        return self._request_json(
            "POST",
            path,
            payload=payload,
            ok_statuses=ok_statuses,
        )

    def _patch_json(
        self,
        path: str,
        payload: dict[str, Any],
    ) -> Any | None:
        """PATCH 요청을 보내고 JSON 응답을 반환한다."""
        return self._request_json("PATCH", path, payload=payload)

    def _expect_list(self, value: Any, context: str) -> list[dict[str, Any]]:
        """응답이 list인지 확인하고, dict item만 남긴다.

        Control Server 응답 schema가 예상과 다르면 빈 list를 반환한다.
        """
        if isinstance(value, list):
            return [item for item in value if isinstance(item, dict)]

        self._node.get_logger().warn(f"[ControlServerClient] {context} 응답이 list가 아님: {type(value).__name__}")
        return []

    def _expect_dict(self, value: Any, context: str) -> dict[str, Any] | None:
        """응답이 dict인지 확인한다.

        Control Server 응답 schema가 예상과 다르면 None을 반환한다.
        """
        if isinstance(value, dict):
            return value

        self._node.get_logger().warn(f"[ControlServerClient] {context} 응답이 dict가 아님: {type(value).__name__}")
        return None

    # ==================================================================
    # Zone / Product 조회
    # ==================================================================

    def get_snapshot(self) -> dict[str, Any] | None:
        """Fleet Manager용 전체 상태 snapshot을 조회한다.

        TaskManager가 robot unit 배정, robot 상태 확인, 현재 task 확인을 할 때
        사용한다. Control Server의 admin status와 유사한 구조를 반환한다.
        """
        data = self._get_json("/api/fleet/snapshot")
        return self._expect_dict(data, "Fleet snapshot 조회")

    def list_robots(self) -> list[dict[str, Any]]:
        """snapshot에서 robot 목록만 추출한다."""
        snapshot = self.get_snapshot()
        if snapshot is None:
            return []
        return self._expect_list(snapshot.get("robots"), "snapshot robot 목록")

    def list_zones(self, zone_type: str = "ALL") -> list[dict[str, Any]]:
        """zone 목록을 조회한다.

        Args:
            zone_type:
                PRODUCT, PRODUCT_SLOT, PICKUP, PICKUP_SLOT, STOCK, STOCK_SLOT,
                STANDBY, ALL 등 Control Server가 지원하는 zone_type.

        Returns:
            zone dict list.
        """
        data = self._get_json(
            "/api/fleet/zones",
            params={"zone_type": zone_type},
        )
        return self._expect_list(data, "zone 목록 조회")

    def fetch_zone_coords(self) -> dict[str, tuple[float, float]]:
        """전체 zone의 좌표를 TrafficManager용 dict로 변환한다.

        Returns:
            {
                "PRODUCT_ZONE_1": (x, y),
                "PICKUP_ZONE_1": (x, y),
                ...
            }

        실패 시:
            빈 dict를 반환한다. TrafficManager는 기본 graph 좌표를 유지할 수 있다.
        """
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

        self._node.get_logger().info(f"[ControlServerClient] zone 좌표 조회 완료: {len(coords)}개")
        return coords

    def get_zone_map(self) -> dict[str, dict[str, Any]]:
        """zone_name을 key로 하는 zone map을 만든다.

        Task payload 생성 시 zone_name -> zone_id 변환에 사용한다.
        """
        zones = self.list_zones(zone_type="ALL")
        return {str(zone["zone_name"]): zone for zone in zones if zone.get("zone_name") is not None}

    def list_products(self) -> list[dict[str, Any]]:
        """상품 목록을 조회한다.

        상품의 storage_zone_name을 이용해 PRODUCT_SLOT/PRODUCT_ZONE 매핑에 사용한다.
        """
        data = self._get_json("/api/products")
        return self._expect_list(data, "상품 목록 조회")

    def get_product_map(self) -> dict[int, dict[str, Any]]:
        """product_id를 key로 하는 product map을 만든다."""
        products = self.list_products()
        result: dict[int, dict[str, Any]] = {}

        for product in products:
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
        """Fleet Manager가 처리해야 할 주문 목록을 조회한다.

        Args:
            status: 특정 주문 상태만 조회할 때 사용한다.
            include_completed: 완료/에러 주문까지 포함할지 여부.

        Returns:
            주문 summary dict list.
        """
        params: dict[str, Any] = {"include_completed": include_completed}
        if status is not None:
            params["status"] = status

        data = self._get_json("/api/fleet/orders", params=params)
        return self._expect_list(data, "Fleet 주문 목록 조회")

    def list_waiting_orders(self) -> list[dict[str, Any]]:
        """ORDER_WAIT 상태의 주문 목록을 조회한다.

        TaskManager.check_waiting_work()에서 polling 대상으로 사용한다.
        """
        data = self._get_json(
            "/api/fleet/orders",
            params={"status": "ORDER_WAIT"},
        )
        return self._expect_list(data, "ORDER_WAIT 주문 조회")

    def list_order_tasks(self, order_id: int) -> list[dict[str, Any]]:
        """특정 주문에 이미 생성된 task 목록을 조회한다.

        중복 task 생성을 막기 위해 사용한다.
        """
        data = self._get_json(f"/api/fleet/orders/{order_id}/tasks")
        return self._expect_list(data, f"order_id={order_id} task 조회")

    def list_tasks(
        self,
        *,
        status: str | None = None,
        robot_name: str | None = None,
        task_type: str | None = None,
        order_id: int | None = None,
    ) -> list[dict[str, Any]]:
        """Fleet task 목록을 조회한다.

        TaskManager 실행 큐에서 ASSIGNED/RUNNING task를 찾을 때 사용한다.
        Control Server API가 지원하는 query parameter만 전달한다.
        """
        params: dict[str, Any] = {}

        if status is not None:
            params["status"] = status
        if robot_name is not None:
            params["robot_name"] = robot_name
        if task_type is not None:
            params["task_type"] = task_type
        if order_id is not None:
            params["order_id"] = order_id

        data = self._get_json("/api/fleet/tasks", params=params or None)
        return self._expect_list(data, "Fleet task 목록 조회")

    def get_order_detail(self, order_id: int) -> dict[str, Any] | None:
        """주문 상세를 조회한다.

        주문 item 목록을 가져와 get_order_work()에서 정규화한다.
        """
        data = self._get_json(f"/api/orders/{order_id}")
        return self._expect_dict(data, f"order_id={order_id} 상세 조회")

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
        """주문 상태, 담당 robot_unit, pickup_slot 배정을 갱신한다.

        사용 예:
        - robot unit 배정 후 assigned_unit_id 기록
        - 상품 상차 완료 후 pickup_slot_id 기록
        - 주문 상태를 SORTING, DELIVERING, ERROR 등으로 변경
        """
        payload: dict[str, Any] = {}

        if status is not None:
            payload["status"] = status
        if assigned_unit_id is not None:
            payload["assigned_unit_id"] = assigned_unit_id
        if pickup_slot_id is not None:
            payload["pickup_slot_id"] = pickup_slot_id

        if not payload:
            self._node.get_logger().warn(f"[ControlServerClient] order_id={order_id} 상태 변경 payload 없음")
            return None

        data = self._patch_json(f"/api/fleet/orders/{order_id}", payload)
        return self._expect_dict(data, f"order_id={order_id} 상태 변경")

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
        """로봇의 런타임 상태를 Control Server에 보고한다.

        PICKY는 picky_state, battery, pose를 주로 보고한다.
        COBOT은 cobot_state를 주로 보고한다.
        """
        payload: dict[str, Any] = {}

        optional_fields = {
            "robot_status": robot_status,
            "picky_state": picky_state,
            "cobot_state": cobot_state,
            "current_task_id": current_task_id,
            "battery_level": battery_level,
            "pos_x": pos_x,
            "pos_y": pos_y,
            "pos_theta": pos_theta,
        }

        for key, value in optional_fields.items():
            if value is not None:
                payload[key] = value

        if not payload:
            self._node.get_logger().warn(f"[ControlServerClient] robot={robot_name} 상태 변경 payload 없음")
            return None

        data = self._patch_json(f"/api/fleet/robots/{robot_name}", payload)
        return self._expect_dict(data, f"robot={robot_name} 상태 변경")

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
        """task 상태를 갱신한다.

        사용 예:
        - ASSIGNED -> RUNNING
        - RUNNING -> SUCCESS
        - RUNNING -> FAILED

        current_status를 넣으면 Control Server에서 상태 충돌을 감지할 수 있다.
        """
        payload: dict[str, Any] = {"status": status}

        if current_status is not None:
            payload["current_status"] = current_status
        if assigned_robot_id is not None:
            payload["assigned_robot_id"] = assigned_robot_id
        if assigned_robot_name is not None:
            payload["assigned_robot_name"] = assigned_robot_name
        if result_message is not None:
            payload["result_message"] = result_message

        data = self._patch_json(f"/api/fleet/tasks/{task_id}", payload)
        return self._expect_dict(data, f"task_id={task_id} 상태 변경")

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
        """task_event를 기록한다.

        task 상태 변경 이력, 실패 사유, 실행 시작/완료 이벤트를 남기는 데 사용한다.

        update_task_status=True이면 Control Server가 task 상태도 함께 변경한다.
        """
        payload: dict[str, Any] = {
            "to_status": to_status,
            "update_task_status": update_task_status,
        }

        optional_fields = {
            "from_status": from_status,
            "event_name": event_name,
            "reason": reason,
            "robot_id": robot_id,
            "robot_name": robot_name,
        }

        for key, value in optional_fields.items():
            if value is not None:
                payload[key] = value

        data = self._post_json(f"/api/fleet/tasks/{task_id}/events", payload)
        return self._expect_dict(data, f"task_id={task_id} event 생성")

    # ==================================================================
    # Pickup Slot
    # ==================================================================

    def list_pickup_slots(
        self,
        status: str | None = None,
    ) -> list[dict[str, Any]]:
        """pickup slot 목록을 조회한다.

        MOVE_TO_PICKUP 생성 직전에 EMPTY slot 후보를 가져오는 데 사용한다.
        """
        params = {"status": status} if status is not None else None
        data = self._get_json("/api/fleet/pickup-slots", params=params)
        return self._expect_list(data, "pickup slot 조회")

    def assign_pickup_slot(
        self,
        order_id: int,
        slot_id: int,
    ) -> dict[str, Any] | None:
        """주문에 특정 pickup slot을 배정한다.

        TrafficManager가 PICKUP_ZONE_2를 선택했다면,
        TaskManager는 대응되는 PICKUP_SLOT_2의 slot_id를 이 함수로 배정한다.
        """
        return self.update_order_status(order_id, pickup_slot_id=slot_id)

    def update_pickup_slot_status(
        self,
        slot_id: int,
        status: str,
    ) -> dict[str, Any] | None:
        """pickup slot 상태를 직접 변경한다.

        일반적으로 주문에 pickup_slot_id를 배정하면 Control Server가 RESERVED로 맞춘다.
        수동 보정이나 예외 처리 시 사용한다.
        """
        data = self._patch_json(
            f"/api/fleet/pickup-slots/{slot_id}",
            {"status": status},
        )
        return self._expect_dict(data, f"pickup_slot_id={slot_id} 상태 변경")

    # ==================================================================
    # Task 생성
    # ==================================================================

    def create_tasks_bulk(
        self,
        tasks: list[dict[str, Any]],
    ) -> dict[str, Any] | None:
        """task 여러 개를 한 번에 생성한다.

        주문 task:
        - MOVE_TO_PRODUCT
        - SORTING_AND_LOAD
        - MOVE_TO_PICKUP
        - INSPECTION
        - UNLOAD

        입고 task:
        - MOVE_TO_STOCK
        - STOCKING_PICK
        - MOVE_TO_STORAGE
        - STOCKING_PLACE

        Returns:
            {
                "status": "ok",
                "task_ids": [1, 2, ...],
                "created_count": n
            }
        """
        if not tasks:
            self._node.get_logger().warn("[ControlServerClient] create_tasks_bulk 호출에 빈 task 목록 전달")
            return None

        data = self._post_json(
            "/api/fleet/tasks/bulk",
            {"tasks": tasks},
            ok_statuses=(200, 201),
        )
        result = self._expect_dict(data, "task 일괄 생성")

        if result is None:
            return None

        task_ids = result.get("task_ids") or []
        if len(task_ids) != len(tasks):
            self._node.get_logger().warn(
                "[ControlServerClient] task 생성 요청 개수와 응답 task_ids 개수 불일치: "
                f"requested={len(tasks)}, created={len(task_ids)}"
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
        """exception_log를 생성한다.

        이동 실패, sorting 실패, 검수 실패, timeout, safety event 등을 기록한다.
        """
        payload: dict[str, Any] = {"exception_type": exception_type}

        optional_fields = {
            "robot_id": robot_id,
            "robot_name": robot_name,
            "task_id": task_id,
            "order_id": order_id,
            "detail": detail,
        }

        for key, value in optional_fields.items():
            if value is not None:
                payload[key] = value

        data = self._post_json("/api/fleet/exceptions", payload)
        return self._expect_dict(data, "exception 생성")

    # ==================================================================
    # Stocking
    # ==================================================================

    def list_requested_stocking_items(self) -> list[dict[str, Any]]:
        """REQUESTED 상태의 stocking_item 목록을 조회한다.

        LLM 담당 모듈이 만든 입고 요청을 TaskManager가 polling으로 감지할 때 사용한다.
        """
        data = self._get_json(
            "/api/fleet/stocking-items",
            params={"status": "REQUESTED"},
        )
        return self._expect_list(data, "REQUESTED stocking_item 조회")

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
        payload: dict[str, Any] = {}

        optional_fields = {
            "status": status,
            "assigned_unit_id": assigned_unit_id,
            "detected_quantity": detected_quantity,
            "stock_delta": stock_delta,
        }

        for key, value in optional_fields.items():
            if value is not None:
                payload[key] = value

        if not payload:
            self._node.get_logger().warn(f"[ControlServerClient] stocking_item_id={stocking_item_id} 변경 payload 없음")
            return None

        data = self._patch_json(
            f"/api/fleet/stocking-items/{stocking_item_id}",
            payload,
        )
        return self._expect_dict(
            data,
            f"stocking_item_id={stocking_item_id} 상태 변경",
        )

    def complete_stocking(
        self,
        *,
        task_id: int,
        detected_quantity: int,
        stock_delta: int,
        result_message: str | None = None,
    ) -> dict[str, Any] | None:
        """입고 완료를 Control Server에 보고한다.

        STOCKING_PLACE 성공 후 호출한다.
        Control Server는 이 요청을 바탕으로 product.stock_qty와 stocking_item 상태를 반영한다.
        """
        payload: dict[str, Any] = {
            "task_id": task_id,
            "detected_quantity": detected_quantity,
            "stock_delta": stock_delta,
        }

        if result_message is not None:
            payload["result_message"] = result_message

        data = self._post_json("/api/fleet/stocking/complete", payload)
        return self._expect_dict(data, f"task_id={task_id} 입고 완료")

    # ==================================================================
    # 정규화 helpers
    # ==================================================================

    def get_order_work(self, order_id: int) -> dict[str, Any] | None:
        """주문 상세를 TaskManager가 쓰기 좋은 dict로 정규화한다.

        이 함수가 하는 일:
        - 주문 상세 조회
        - 상품 목록 조회
        - zone 목록 조회
        - order_item의 PRODUCT_SLOT을 PICKY 정차용 PRODUCT_ZONE으로 매핑
        - assigned_unit_id가 있으면 PICKY/COBOT 이름 계산

        이 함수가 하지 않는 일:
        - 상품 방문 순서 결정
        - 경로 선택
        - task 생성
        - robot unit 신규 배정

        Returns:
            TaskManager용 order_work dict.
        """
        order = self.get_order_detail(order_id)

        if order is None:
            return None

        product_map = self.get_product_map()
        zone_map = self.get_zone_map()

        items: list[dict[str, Any]] = []
        raw_items = order.get("items") or []

        if not isinstance(raw_items, list):
            self._node.get_logger().warn(f"[ControlServerClient] order_id={order_id} items 응답이 list가 아님")
            return None

        for raw_item in raw_items:
            if not isinstance(raw_item, dict):
                continue

            product_id = raw_item.get("product_id")
            if product_id is None:
                self._node.get_logger().warn(
                    f"[ControlServerClient] order_id={order_id} item에 product_id 없음: {raw_item}"
                )
                continue

            product = product_map.get(int(product_id), {})

            # 상품 실제 보관 위치는 PRODUCT_SLOT_*이고,
            # PICKY가 정차할 위치는 같은 번호의 PRODUCT_ZONE_*이다.
            product_slot_name = (
                raw_item.get("storage_zone_name")
                or raw_item.get("product_slot_name")
                or product.get("storage_zone_name")
            )
            product_zone_name = self._slot_name_to_zone_name(product_slot_name)

            if product_slot_name is None or product_zone_name is None:
                self._node.get_logger().warn(
                    f"[ControlServerClient] product_id={product_id} zone/slot 매핑 실패: "
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
            self._node.get_logger().warn(f"[ControlServerClient] order_id={order_id} 정규화 결과 item 없음")
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

    def get_stocking_work(
        self,
        stocking_item: dict[str, Any],
    ) -> dict[str, Any] | None:
        """stocking_item을 TaskManager가 쓰기 좋은 dict로 정규화한다.

        이 함수가 하는 일:
        - stocking_item의 product_id 확인
        - product storage slot 조회
        - PRODUCT_SLOT -> PRODUCT_ZONE 매핑
        - STOCK_SLOT/STOCK_ZONE 정보 추가
        - assigned_unit_id가 있으면 PICKY/COBOT 이름 계산

        이 함수가 하지 않는 일:
        - 입고 task 4개 생성
        - robot unit 신규 배정
        - 경로 선택
        """
        product_id = stocking_item.get("product_id")
        if product_id is None:
            self._node.get_logger().warn(f"[ControlServerClient] stocking_item에 product_id 없음: {stocking_item}")
            return None

        product_map = self.get_product_map()
        zone_map = self.get_zone_map()
        product = product_map.get(int(product_id), {})

        product_slot_name = stocking_item.get("storage_zone_name") or product.get("storage_zone_name")
        product_zone_name = self._slot_name_to_zone_name(product_slot_name)

        if product_slot_name is None or product_zone_name is None:
            self._node.get_logger().warn(
                f"[ControlServerClient] stocking_item product_id={product_id} zone/slot 매핑 실패"
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
        """SLOT 이름을 대응되는 PICKY 정차 ZONE 이름으로 변환한다.

        예:
        - PRODUCT_SLOT_3 -> PRODUCT_ZONE_3
        - PICKUP_SLOT_2 -> PICKUP_ZONE_2
        - STOCK_SLOT -> STOCK_ZONE
        """
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
        """robot_unit id를 PICKY/COBOT 이름으로 변환한다.

        현재 seed 기준:
        - 1 -> PICKY1 / COBOT1
        - 2 -> PICKY2 / COBOT2
        """
        if assigned_unit_id == 1:
            return "PICKY1", "COBOT1"

        if assigned_unit_id == 2:
            return "PICKY2", "COBOT2"

        return None, None
