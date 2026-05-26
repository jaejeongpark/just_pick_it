from __future__ import annotations

import asyncio
import threading
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from rclpy.node import Node

from fleet_manager.fleet_api_schemas import (
    OrderCreateIn,
    PickupSlotCreateIn,
    ProductCreateIn,
    ProductStockUpdateIn,
    ProductUpdateIn,
)
from fleet_manager.fleet_repository import FleetRepository, RepoError
from just_pick_it_db.session import check_database_connection


class _WsManager:
    """채널별(admin/customer) WebSocket 연결 집합을 관리한다.

    asyncio 이벤트 루프(uvicorn) 위에서만 호출되는 것을 전제로 한다.
    """

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


class FleetApiServer:
    """Fleet Manager 가 웹 프런트에 노출하는 HTTP/REST + WebSocket API 서버.

    설계(통합 계획 2.3 / 3.4):
    - ROS2 노드 프로세스 안에서 uvicorn 을 **별도 데몬 스레드**로 띄운다.
      rclpy executor(메인 스레드)와 asyncio(uvicorn 스레드)가 한 프로세스에서 공존한다.
    - 라우트 핸들러는 DB 접근만 하며, FleetRepository 를 통해 처리한다.
      FleetRepository 의 각 메서드는 session_scope() 로 스레드 로컬 Session 을 열고 닫으므로
      uvicorn 워커 스레드에서 호출해도 안전하다.
    - 로봇을 실제로 움직이는 동작(emergency 전파 등)은 노드의 trigger_emergency_stop 으로 위임한다.

    실시간 push:
    - admin/customer WebSocket 으로 상태 스냅샷을 흘려보낸다.
    - 상태 변경은 여러 스레드에서 일어나므로(API/TaskManager), 쓰기 지점마다 훅을 거는 대신
      이벤트 루프에서 도는 **주기적 push 루프**를 둔다. 연결된 클라이언트가 있을 때만,
      DB 조회는 run_in_executor 로 루프 밖에서 수행해 이벤트 루프 블로킹을 피한다.
    - 연결 직후 1회 즉시 스냅샷을 보낸다.
    """

    def __init__(
        self,
        node: Node,
        fleet_repo: FleetRepository,
        host: str = "0.0.0.0",
        port: int = 8100,
        push_interval_sec: float = 1.0,
    ) -> None:
        self._node = node
        self._repo = fleet_repo
        self._host = host
        self._port = port
        self._push_interval = push_interval_sec
        self._admin_ws = _WsManager()
        self._customer_ws = _WsManager()
        self._app = self._build_app()
        self._server: uvicorn.Server | None = None
        self._thread: threading.Thread | None = None

    # ==================================================================
    # FastAPI app
    # ==================================================================

    def _build_app(self) -> FastAPI:
        repo = self._repo

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

        app = FastAPI(title="Just Pick It Fleet Manager API", lifespan=lifespan)

        @app.get("/api/health/db")
        def health_db():
            try:
                check_database_connection()
            except Exception as exc:  # noqa: BLE001 - 연결 실패 원인을 그대로 노출
                raise HTTPException(status_code=503, detail=str(exc)) from exc
            return {"status": "ok"}

        @app.get("/api/admin/status")
        def admin_status():
            return repo.get_snapshot()

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

        # ----- 명령 (POST/PATCH) -----
        # RepoError 는 status_code 를 들고 있으므로 HTTPException 으로 매핑한다.

        @app.post("/api/orders", status_code=201)
        def create_order(body: OrderCreateIn):
            items = [item.model_dump() for item in body.items]
            return self._guard(lambda: repo.create_order(items))

        @app.post("/api/orders/{order_id}/complete")
        def complete_order(order_id: int):
            return self._guard(lambda: repo.complete_order(order_id))

        @app.post("/api/admin/products", status_code=201)
        def create_product(body: ProductCreateIn):
            return self._guard(lambda: repo.create_product(**body.model_dump()))

        @app.patch("/api/admin/products/{product_id}")
        def update_product(product_id: int, body: ProductUpdateIn):
            return self._guard(lambda: repo.update_product(product_id, **body.model_dump()))

        @app.patch("/api/admin/products/{product_id}/stock")
        def update_product_stock(product_id: int, body: ProductStockUpdateIn):
            return self._guard(lambda: repo.update_product_stock(product_id, body.stock_qty))

        @app.post("/api/admin/pickup-slots", status_code=201)
        def create_pickup_slot(body: PickupSlotCreateIn):
            return self._guard(lambda: repo.create_pickup_slot(**body.model_dump()))

        @app.post("/api/admin/exceptions/{exception_id}/resolve")
        def resolve_exception(exception_id: int):
            return self._guard(lambda: repo.resolve_exception(exception_id))

        # ----- 로봇 제어 명령 (DB 전이 + executor 측 로봇 전파) -----

        @app.post("/api/admin/emergency-stop")
        def emergency_stop():
            return self._node.trigger_emergency_stop(True)

        @app.post("/api/admin/resume")
        def resume():
            return self._node.trigger_emergency_stop(False)

        # ----- 실시간 상태 WebSocket -----

        @app.websocket("/api/admin/ws/status")
        async def admin_ws(websocket: WebSocket):
            await self._serve_status_ws(websocket, self._admin_ws, repo.get_snapshot)

        @app.websocket("/api/customer/ws/status")
        async def customer_ws(websocket: WebSocket):
            await self._serve_status_ws(websocket, self._customer_ws, repo.get_customer_snapshot)

        return app

    @staticmethod
    def _guard(action):
        """명령 실행을 감싸 RepoError 를 적절한 HTTP 상태로 변환한다."""
        try:
            return action()
        except RepoError as exc:
            raise HTTPException(status_code=exc.status_code, detail=exc.detail) from exc

    async def _serve_status_ws(self, websocket: WebSocket, manager: _WsManager, snapshot_fn) -> None:
        """상태 WebSocket 연결 1건을 처리한다.

        연결 직후 스냅샷 1회를 보내고, 이후 주기 push 루프가 갱신을 보낸다.
        클라이언트 메시지는 keep-alive 용으로 받아 흘린다.
        """
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
                    snapshot = await loop.run_in_executor(None, self._repo.get_snapshot)
                    await self._admin_ws.broadcast(snapshot)
                if self._customer_ws.count():
                    snapshot = await loop.run_in_executor(None, self._repo.get_customer_snapshot)
                    await self._customer_ws.broadcast(snapshot)
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # noqa: BLE001 - push 실패가 루프를 죽이지 않게 한다
                self._node.get_logger().warn(f"[FleetApiServer] status push 오류: {exc}")

    @property
    def app(self) -> FastAPI:
        """테스트(TestClient)나 외부 마운트를 위해 FastAPI app 을 노출한다."""
        return self._app

    # ==================================================================
    # 수명주기
    # ==================================================================

    def start(self) -> None:
        """uvicorn 서버를 데몬 스레드에서 기동한다."""
        config = uvicorn.Config(
            self._app,
            host=self._host,
            port=self._port,
            log_level="warning",
        )
        server = uvicorn.Server(config)
        # 데몬 스레드에서는 OS signal 핸들러를 설치할 수 없으므로 비활성화한다.
        # (signal 은 메인 스레드에서만 동작)
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
