from fastapi import APIRouter, HTTPException

from app.database import check_database_connection


router = APIRouter(prefix="/api/health", tags=["health"])


@router.get("/db")
def database_health():
    try:
        check_database_connection()
    except Exception as exc:
        raise HTTPException(status_code=503, detail="database unavailable") from exc

    return {"database": "ok"}

