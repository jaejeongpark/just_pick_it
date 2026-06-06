from __future__ import annotations

import asyncio
import threading
from collections.abc import Callable
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from rclpy.node import Node

from fleet_manager.fleet_api_schemas import (
    DebugTaskSuccessIn,
    FleetOrderStateUpdateIn,
    FleetRobotStateUpdateIn,
    FleetTaskBulkCreateIn,
    FleetTaskStateUpdateIn,
    OrderCreateIn,
    PickupSlotCreateIn,
    PickupSlotStateUpdateIn,
    ProductCreateIn,
    ProductStockUpdateIn,
    ProductUpdateIn,
    DisplayItemCreateIn,
)
from fleet_manager.fleet_repository import FleetRepository, RepoError
from just_pick_it_db.session import check_database_connection


# =====================================
# WebSocket manager
# =====================================

class _WsManager:
    """채널별(admin/customer) WebSocket 연결 집합을 관리한다."""

    def __init__(self) -> None:
        self._conns: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        async with self._lock:
            self._conns.add(websocket)

    async def disconnect(self, websocket: WebSocket) -> None:
        async with self._lock:
            self._conns.discard(websocket)

    async def broadcast(self, payload) -> None:
        async with self._lock:
            conns = list(self._conns)

        dead = []
        for websocket in conns:
            try:
                await websocket.send_json(payload)
            except Exception:  # noqa: BLE001 - 끊긴 연결은 정리 대상
                dead.append(websocket)

        if dead:
            async with self._lock:
                for websocket in dead:
                    self._conns.discard(websocket)

    def count(self) -> int:
        return len(self._conns)


# =====================================
# Fleet API server
# =====================================

class FleetApiServer:
    """Fleet Manager 가 웹 프런트에 노출하는 HTTP/REST + WebSocket API 서버.

    ROS2 executor와 uvicorn asyncio loop는 같은 프로세스 안에서 다른 스레드로 돈다.
    DB 접근은 모두 FleetRepository를 통해 수행하고, 로봇 전파가 필요한 명령만
    FleetManagerNode로 위임한다.
    """

    def __init__(
        self,
        node: Node,
        fleet_repo: FleetRepository,
        host: str = "0.0.0.0",
        port: int = 8100,
        push_interval_sec: float = 1.0,
        admin_snapshot_provider: Callable[[], dict | None] | None = None,
        debug_task_success_injector: Callable[..., dict | None] | None = None,
    ) -> None:
        self._node = node
        self._repo = fleet_repo
        self._host = host
        self._port = port
        self._push_interval = push_interval_sec
        self._admin_snapshot_provider = admin_snapshot_provider or self._repo.get_snapshot
        self._debug_task_success_injector = debug_task_success_injector
        self._admin_ws = _WsManager()
        self._customer_ws = _WsManager()
        self._app = self._build_app()
        self._server: uvicorn.Server | None = None
        self._thread: threading.Thread | None = None

    # =====================================
    # App construction
    # =====================================

    def _build_app(self) -> FastAPI:
        app = FastAPI(
            title="Just Pick It Fleet Manager API",
            lifespan=self._lifespan(),
        )
        self._register_health_routes(app)
        self._register_read_routes(app)
        self._register_command_routes(app)
        self._register_robot_control_routes(app)
        self._register_websocket_routes(app)
        return app

    def _lifespan(self):
        @asynccontextmanager
        async def lifespan(_app: FastAPI):
            push_task = asyncio.create_task(self._push_loop())
            self._node.get_logger().info(
                f"[FleetApiServer] status push loop 시작 (interval={self._push_interval:.1f}s)"
            )
            try:
                yield
            finally:
                push_task.cancel()
                try:
                    await push_task
                except asyncio.CancelledError:
                    pass

        return lifespan

    # =====================================
    # Read routes
    # =====================================

    def _register_health_routes(self, app: FastAPI) -> None:
        @app.get("/api/health/db")
        def health_db():
            try:
                check_database_connection()
            except Exception as exc:  # noqa: BLE001 - 연결 실패 원인을 그대로 노출
                raise HTTPException(status_code=503, detail=str(exc)) from exc
            return {"status": "ok"}

    def _register_read_routes(self, app: FastAPI) -> None:
        repo = self._repo

        @app.get("/api/admin/status")
        def admin_status():
            return self._admin_snapshot()

        @app.get("/api/customer/status")
        def customer_status():
            return repo.get_customer_snapshot()

        @app.get("/api/products")
        def list_products():
            return repo.list_products()

        @app.get("/api/orders")
        def list_orders():
            return repo.list_orders()

        @app.get("/api/orders/{order_id}")
        def get_order(order_id: int):
            order = repo.get_order_detail(order_id)
            if order is None:
                raise HTTPException(status_code=404, detail="order not found")
            return order

        @app.get("/api/fleet/snapshot")
        def fleet_snapshot():
            return self._admin_snapshot()

        @app.get("/api/fleet/zones")
        def list_fleet_zones(zone_type: str = "ALL"):
            return repo.list_zones(zone_type=zone_type)

        @app.get("/api/fleet/tasks")
        def list_fleet_tasks(
            status: str | None = None,
            robot_name: str | None = None,
            task_type: str | None = None,
            order_id: int | None = None,
        ):
            return repo.list_tasks(
                status=status,
                robot_name=robot_name,
                task_type=task_type,
                order_id=order_id,
            )

        @app.get("/api/fleet/orders")
        def list_fleet_orders(status: str | None = None, include_completed: bool = False):
            return repo.list_orders(status=status, include_completed=include_completed)

        @app.get("/api/fleet/orders/{order_id}/tasks")
        def list_fleet_order_tasks(order_id: int):
            return repo.list_order_tasks(order_id)

        @app.get("/api/fleet/pickup-slots")
        def list_fleet_pickup_slots(status: str | None = None):
            return repo.list_pickup_slots(status=status)

    # =====================================
    # Command routes
    # =====================================

    def _register_command_routes(self, app: FastAPI) -> None:
        repo = self._repo

        @app.post("/api/orders", status_code=201)
        def create_order(body: OrderCreateIn):
            items = [self._model_dump(item) for item in body.items]
            return self._guard(lambda: repo.create_order(items))

        @app.post("/api/orders/{order_id}/complete")
        def complete_order(order_id: int):
            return self._guard(lambda: repo.complete_order(order_id))

        @app.post("/api/admin/products", status_code=201)
        def create_product(body: ProductCreateIn):
            return self._guard(lambda: repo.create_product(**self._model_dump(body)))

        @app.patch("/api/admin/products/{product_id}")
        def update_product(product_id: int, body: ProductUpdateIn):
            return self._guard(lambda: repo.update_product(product_id, **self._model_dump(body)))

        @app.patch("/api/admin/products/{product_id}/stock")
        def update_product_stock(product_id: int, body: ProductStockUpdateIn):
            return self._guard(lambda: repo.update_product_stock(product_id, body.stock_qty))

        @app.post("/api/admin/pickup-slots", status_code=201)
        def create_pickup_slot(body: PickupSlotCreateIn):
            return self._guard(lambda: repo.create_pickup_slot(**self._model_dump(body)))

        @app.patch("/api/fleet/pickup-slots/{slot_id}")
        def update_fleet_pickup_slot(slot_id: int, body: PickupSlotStateUpdateIn):
            if body.status is None:
                raise HTTPException(status_code=400, detail="status is required")
            return self._require(
                repo.update_pickup_slot_status(slot_id, body.status),
                "pickup slot update failed",
            )

        @app.post("/api/admin/exceptions/{exception_id}/resolve")
        def resolve_exception(exception_id: int):
            return self._guard(lambda: repo.resolve_exception(exception_id))

        @app.post("/api/admin/display-items", status_code=201)
        def create_display_item(body: DisplayItemCreateIn):
            return self._guard(lambda: repo.create_display_item(**self._model_dump(body)))

        @app.post("/api/fleet/tasks/bulk", status_code=201)
        def create_fleet_tasks(body: FleetTaskBulkCreateIn):
            tasks = [self._model_dump(task) for task in body.tasks]
            return self._require(repo.create_tasks_bulk(tasks), "task create failed")

        @app.patch("/api/fleet/tasks/{task_id}")
        def update_fleet_task(task_id: int, body: FleetTaskStateUpdateIn):
            return self._require(
                repo.update_task_status(task_id, **self._task_update_kwargs(body)),
                "task update failed",
            )

        @app.delete("/api/fleet/tasks/{task_id}")
        def delete_fleet_task(task_id: int, force: bool = False):
            return self._guard(lambda: repo.delete_task(task_id, force=force))

        @app.post("/api/admin/debug/robots/{robot_identifier}/running-task/success")
        def debug_complete_running_robot_task(
            robot_identifier: str,
            body: DebugTaskSuccessIn | None = None,
        ):
            if self._debug_task_success_injector is None:
                raise HTTPException(status_code=404, detail="debug task injection unavailable")

            payload = self._model_dump(body) if body is not None else {}
            return self._require(
                self._debug_task_success_injector(robot_identifier, **payload),
                "debug task success injection failed",
            )

        @app.patch("/api/fleet/orders/{order_id}")
        def update_fleet_order(order_id: int, body: FleetOrderStateUpdateIn):
            return self._require(
                repo.update_order_status(order_id, **self._order_update_kwargs(body)),
                "order update failed",
            )

        @app.patch("/api/fleet/robots/{robot_identifier}")
        def update_fleet_robot(robot_identifier: str, body: FleetRobotStateUpdateIn):
            return self._require(
                repo.update_robot_state(robot_identifier, **self._robot_update_kwargs(body)),
                "robot update failed",
            )

    # =====================================
    # Robot control routes
    # =====================================

    def _register_robot_control_routes(self, app: FastAPI) -> None:
        @app.post("/api/admin/emergency-stop")
        def emergency_stop():
            return self._node.trigger_emergency_stop(True)

        @app.post("/api/admin/resume")
        def resume():
            return self._node.trigger_emergency_stop(False)

    # =====================================
    # WebSocket routes
    # =====================================

    def _register_websocket_routes(self, app: FastAPI) -> None:
        @app.websocket("/api/admin/ws/status")
        async def admin_ws(websocket: WebSocket):
            await self._serve_status_ws(websocket, self._admin_ws, self._admin_snapshot)

        @app.websocket("/api/customer/ws/status")
        async def customer_ws(websocket: WebSocket):
            await self._serve_status_ws(websocket, self._customer_ws, self._repo.get_customer_snapshot)

    # =====================================
    # Request helpers
    # =====================================

    @staticmethod
    def _model_dump(model) -> dict:
        """Pydantic v1 모델을 dict로 변환한다."""
        return model.dict()

    @staticmethod
    def _provided_fields(model) -> set[str]:
        """PATCH 요청에 실제 포함된 필드명을 얻는다."""
        return set(model.__fields_set__)

    def _task_update_kwargs(self, body: FleetTaskStateUpdateIn) -> dict:
        fields = self._provided_fields(body)
        kwargs = {}
        if "current_status" in fields:
            kwargs["current_status"] = body.current_status
        if "status" in fields:
            kwargs["status"] = body.status
        if "assigned_robot_id" in fields:
            kwargs["assigned_robot_id"] = body.assigned_robot_id
        if "assigned_robot_name" in fields:
            kwargs["assigned_robot_name"] = body.assigned_robot_name
        if "result_message" in fields:
            kwargs["result_message"] = body.result_message
        return kwargs

    def _order_update_kwargs(self, body: FleetOrderStateUpdateIn) -> dict:
        fields = self._provided_fields(body)
        kwargs = {}
        if "status" in fields:
            kwargs["status"] = body.status
        if "pickup_slot_id" in fields:
            kwargs["pickup_slot_id"] = body.pickup_slot_id
        if "assigned_unit_id" in fields:
            kwargs["assigned_unit_id"] = body.assigned_unit_id
        if "item_quantities" in fields:
            kwargs["item_quantities"] = (
                [self._model_dump(item) for item in body.item_quantities]
                if body.item_quantities is not None
                else []
            )
        return kwargs

    def _robot_update_kwargs(self, body: FleetRobotStateUpdateIn) -> dict:
        fields = self._provided_fields(body)
        kwargs = {}
        if "robot_status" in fields or "status" in fields:
            kwargs["robot_status"] = body.robot_status or body.status
        if "picky_state" in fields:
            kwargs["picky_state"] = body.picky_state
        if "cobot_state" in fields:
            kwargs["cobot_state"] = body.cobot_state
        if "current_task_id" in fields:
            kwargs["current_task_id"] = body.current_task_id
        if "battery_level" in fields:
            kwargs["battery_level"] = body.battery_level
        if "pos_x" in fields:
            kwargs["pos_x"] = body.pos_x
        if "pos_y" in fields:
            kwargs["pos_y"] = body.pos_y
        if "pos_theta" in fields:
            kwargs["pos_theta"] = body.pos_theta
        return kwargs

    # =====================================
    # Error helpers
    # =====================================

    @staticmethod
    def _guard(action):
        """명령 실행을 감싸 RepoError 를 적절한 HTTP 상태로 변환한다."""
        try:
            return action()
        except RepoError as exc:
            raise HTTPException(status_code=exc.status_code, detail=exc.detail) from exc

    @staticmethod
    def _require(result, detail: str):
        """Repository 의 None 반환을 HTTP 오류로 변환한다."""
        if result is None:
            raise HTTPException(status_code=400, detail=detail)
        return result

    def _admin_snapshot(self) -> dict | None:
        """관리자용 snapshot provider를 단일 경로로 호출한다."""
        return self._admin_snapshot_provider()

    # =====================================
    # WebSocket helpers
    # =====================================

    async def _serve_status_ws(self, websocket: WebSocket, manager: _WsManager, snapshot_fn) -> None:
        """상태 WebSocket 연결 1건을 처리한다."""
        await manager.connect(websocket)
        loop = asyncio.get_running_loop()
        try:
            snapshot = await loop.run_in_executor(None, snapshot_fn)
            await websocket.send_json(snapshot)
            while True:
                await websocket.receive_text()
        except WebSocketDisconnect:
            pass
        finally:
            await manager.disconnect(websocket)

    async def _push_loop(self) -> None:
        """연결된 클라이언트에게 주기적으로 상태 스냅샷을 push 한다."""
        loop = asyncio.get_running_loop()
        while True:
            await asyncio.sleep(self._push_interval)
            try:
                if self._admin_ws.count():
                    snapshot = await loop.run_in_executor(None, self._admin_snapshot)
                    await self._admin_ws.broadcast(snapshot)
                if self._customer_ws.count():
                    snapshot = await loop.run_in_executor(None, self._repo.get_customer_snapshot)
                    await self._customer_ws.broadcast(snapshot)
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # noqa: BLE001 - push 실패가 루프를 죽이지 않게 한다
                self._node.get_logger().warn(f"[FleetApiServer] status push 오류: {exc}")

    # =====================================
    # Lifecycle
    # =====================================

    @property
    def app(self) -> FastAPI:
        """테스트(TestClient)나 외부 마운트를 위해 FastAPI app 을 노출한다."""
        return self._app

    def start(self) -> None:
        """uvicorn 서버를 데몬 스레드에서 기동한다."""
        config = uvicorn.Config(
            self._app,
            host=self._host,
            port=self._port,
            log_level="warning",
        )
        server = uvicorn.Server(config)
        server.install_signal_handlers = lambda: None
        self._server = server
        self._thread = threading.Thread(
            target=server.run,
            name="fleet_api_server",
            daemon=True,
        )
        self._thread.start()
        self._node.get_logger().info(
            f"[FleetApiServer] HTTP API 서버 시작: http://{self._host}:{self._port}"
        )

    def stop(self) -> None:
        """uvicorn 서버를 정지한다."""
        if self._server is not None:
            self._server.should_exit = True
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        self._node.get_logger().info("[FleetApiServer] HTTP API 서버 정지")
