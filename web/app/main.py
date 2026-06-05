from pathlib import Path

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

from app.routers import fleet_api_proxy_router, llm_router, page_router


# =====================================
# Paths
# =====================================

BASE_DIR = Path(__file__).resolve().parent


# =====================================
# App
# =====================================

app = FastAPI(title="Just Pick It Web Gateway")


# =====================================
# Static files
# =====================================

app.mount("/static", StaticFiles(directory=BASE_DIR / "static"), name="static")


# =====================================
# Routers
# =====================================

app.include_router(page_router.router)
app.include_router(llm_router.router)
app.include_router(fleet_api_proxy_router.router)
