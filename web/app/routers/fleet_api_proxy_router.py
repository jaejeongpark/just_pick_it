from __future__ import annotations

import asyncio
from urllib.parse import urljoin

import httpx
import websockets
from fastapi import APIRouter, Request, Response, WebSocket, WebSocketDisconnect

from app.config import FLEET_API_BASE_URL, FLEET_API_WS_BASE_URL


# =====================================
# Router
# =====================================

router = APIRouter(tags=["fleet-api-proxy"])


# =====================================
# Constants
# =====================================

HOP_BY_HOP_HEADERS = {
    "connection",
    "keep-alive",
    "proxy-authenticate",
    "proxy-authorization",
    "te",
    "trailers",
    "transfer-encoding",
    "upgrade",
    "host",
    "content-length",
    "content-encoding",
}


# =====================================
# URL/header helpers
# =====================================

def _target_http_url(path: str, query: str) -> str:
    base = FLEET_API_BASE_URL.rstrip("/") + "/"
    url = urljoin(base, f"api/{path}")
    return f"{url}?{query}" if query else url


def _target_ws_url(path: str, query: str) -> str:
    base = FLEET_API_WS_BASE_URL.rstrip("/") + "/"
    url = urljoin(base, f"api/{path}")
    return f"{url}?{query}" if query else url


def _forward_headers(headers) -> dict[str, str]:
    return {
        key: value
        for key, value in headers.items()
        if key.lower() not in HOP_BY_HOP_HEADERS
    }


# =====================================
# HTTP proxy
# =====================================

@router.api_route(
    "/api/{path:path}",
    methods=["GET", "POST", "PATCH", "PUT", "DELETE", "OPTIONS"],
)
async def proxy_http_api(path: str, request: Request) -> Response:
    """브라우저의 same-origin /api 요청을 Fleet Manager API 로 전달한다."""
    target_url = _target_http_url(path, request.url.query)
    body = await request.body()

    try:
        async with httpx.AsyncClient(timeout=30.0) as client:
            upstream = await client.request(
                request.method,
                target_url,
                content=body,
                headers=_forward_headers(request.headers),
            )
    except httpx.RequestError as exc:
        return Response(
            content=f"Fleet API 연결 실패: {exc}".encode(),
            status_code=503,
            media_type="text/plain; charset=utf-8",
        )

    return Response(
        content=upstream.content,
        status_code=upstream.status_code,
        headers=_forward_headers(upstream.headers),
    )


# =====================================
# WebSocket proxy
# =====================================

@router.websocket("/api/{path:path}")
async def proxy_websocket_api(path: str, websocket: WebSocket) -> None:
    """상태 WebSocket 을 Fleet Manager API 로 양방향 프록시한다."""
    await websocket.accept()
    target_url = _target_ws_url(path, websocket.url.query)

    try:
        async with websockets.connect(target_url) as upstream:
            async def client_to_upstream() -> None:
                while True:
                    message = await websocket.receive_text()
                    await upstream.send(message)

            async def upstream_to_client() -> None:
                async for message in upstream:
                    await websocket.send_text(message)

            tasks = [
                asyncio.create_task(client_to_upstream()),
                asyncio.create_task(upstream_to_client()),
            ]
            done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in pending:
                task.cancel()
            for task in done:
                task.result()
    except (OSError, WebSocketDisconnect, websockets.ConnectionClosed):
        return
