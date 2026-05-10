from pathlib import Path

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

from app.routers import (
    admin_router,
    customer_router,
    fleet_router,
    health_router,
    order_router,
    page_router,
    product_router,
)


BASE_DIR = Path(__file__).resolve().parent

app = FastAPI(title="Just Pick It Control Server")

app.mount("/static", StaticFiles(directory=BASE_DIR / "static"), name="static")
app.include_router(page_router.router)
app.include_router(health_router.router)
app.include_router(product_router.router)
app.include_router(order_router.router)
app.include_router(admin_router.router)
app.include_router(customer_router.router)
app.include_router(fleet_router.router)
