import os
from pathlib import Path

from dotenv import load_dotenv


WEB_DIR = Path(__file__).resolve().parents[1]
load_dotenv(WEB_DIR / ".env")

DATABASE_URL = os.getenv(
    "DATABASE_URL",
    "postgresql://just_pick_it_user:just_pick_it_pw@localhost:5432/just_pick_it",
)
APP_HOST = os.getenv("APP_HOST", "0.0.0.0")
APP_PORT = int(os.getenv("APP_PORT", "8000"))
