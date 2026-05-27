import os
from pathlib import Path

from dotenv import load_dotenv


# =====================================
# Paths
# =====================================

WEB_DIR = Path(__file__).resolve().parents[1]
APP_DIR = Path(__file__).resolve().parent


# =====================================
# Environment
# =====================================

load_dotenv(WEB_DIR / ".env")


# =====================================
# Web Gateway
# =====================================

APP_HOST = os.getenv("APP_HOST", "0.0.0.0")
APP_PORT = int(os.getenv("APP_PORT", "8000"))


# =====================================
# Fleet API upstream
# =====================================

# Web Gateway는 DB를 직접 만지지 않고 Fleet Manager API 앞단 프록시로 동작한다.
FLEET_API_BASE_URL = os.getenv("FLEET_API_BASE_URL", "http://localhost:8100")
FLEET_API_WS_BASE_URL = os.getenv("FLEET_API_WS_BASE_URL", "ws://localhost:8100")
