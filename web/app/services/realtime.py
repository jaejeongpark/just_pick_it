import asyncio

from fastapi import WebSocket

from app.database import SessionLocal
from app.services.status_service import build_admin_status, build_customer_status


class WebSocketManager:
    def __init__(self):
        self._connections: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()

        async with self._lock:
            self._connections.add(websocket)

    async def disconnect(self, websocket: WebSocket) -> None:
        async with self._lock:
            self._connections.discard(websocket)

    async def broadcast(self, payload) -> None:
        async with self._lock:
            connections = list(self._connections)

        disconnected = []

        for websocket in connections:
            try:
                await websocket.send_json(payload)
            except RuntimeError:
                disconnected.append(websocket)

        if disconnected:
            async with self._lock:
                for websocket in disconnected:
                    self._connections.discard(websocket)


admin_websockets = WebSocketManager()
customer_websockets = WebSocketManager()


def get_admin_snapshot():
    db = SessionLocal()
    try:
        return build_admin_status(db)
    finally:
        db.close()


def get_customer_snapshot():
    db = SessionLocal()
    try:
        return build_customer_status(db)
    finally:
        db.close()


async def broadcast_admin_status() -> None:
    await admin_websockets.broadcast(get_admin_snapshot())


async def broadcast_customer_status() -> None:
    await customer_websockets.broadcast(get_customer_snapshot())


async def broadcast_all_status() -> None:
    await asyncio.gather(
        broadcast_admin_status(),
        broadcast_customer_status(),
    )
