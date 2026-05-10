from fastapi import APIRouter, Depends, WebSocket, WebSocketDisconnect
from sqlalchemy.orm import Session

from app.database import get_db
from app.services.realtime import customer_websockets, get_customer_snapshot
from app.services.status_service import build_customer_status


router = APIRouter(prefix="/api/customer", tags=["customer"])


@router.get("/status")
def customer_status(db: Session = Depends(get_db)):
    return build_customer_status(db)


@router.websocket("/ws/status")
async def customer_status_websocket(websocket: WebSocket):
    await customer_websockets.connect(websocket)

    try:
        while True:
            await websocket.send_json(get_customer_snapshot())
            await websocket.receive_text()
    except WebSocketDisconnect:
        await customer_websockets.disconnect(websocket)
