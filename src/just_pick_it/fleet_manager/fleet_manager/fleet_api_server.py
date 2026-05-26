from __future__ import annotations

import threading

import uvicorn
from fastapi import FastAPI, HTTPException
from rclpy.node import Node

from fleet_manager.fleet_repository import FleetRepository
from just_pick_it_db.session import check_database_connection


class FleetApiServer:
    """Fleet Manager 가 웹 프런트에 노출하는 HTTP/REST API 서버.

    설계(통합 계획 2.3 / 3.4):
    - ROS2 노드 프로세스 안에서 uvicorn 을 **별도 데몬 스레드**로 띄운다.
      rclpy executor(메인 스레드)와 asyncio(uvicorn 스레드)가 한 프로세스에서 공존한다.
    - 라우트 핸들러는 DB 접근만 하며, FleetRepository 를 통해 처리한다.
      FleetRepository 의 각 메서드는 session_scope() 로 스레드 로컬 Session 을 열고 닫으므로
      uvicorn 워커 스레드에서 호출해도 안전하다.
    - 로봇을 실제로 움직이는 동작(emergency 전파 등)은 이 스레드에서 rclpy 를 직접 호출하지 않고
      추후 executor 로 위임한다(명령 엔드포인트 증분에서 도입).

    현재 골격 범위:
    - health, 대표 읽기 엔드포인트(admin/customer status, products, orders).
    - 명령(POST/PATCH)과 실시간 WebSocket push 는 다음 증분에서 추가한다.
    """

    def __init__(
        self,
        node: Node,
        fleet_repo: FleetRepository,
        host: str = "0.0.0.0",
        port: int = 8100,
    ) -> None:
        self._node = node
        self._repo = fleet_repo
        self._host = host
        self._port = port
        self._app = self._build_app()
        self._server: uvicorn.Server | None = None
        self._thread: threading.Thread | None = None

    # ==================================================================
    # FastAPI app
    # ==================================================================

    def _build_app(self) -> FastAPI:
        app = FastAPI(title="Just Pick It Fleet Manager API")
        repo = self._repo

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

        return app

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
