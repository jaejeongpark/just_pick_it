import os
from pathlib import Path

from dotenv import load_dotenv


WEB_DIR = Path(__file__).resolve().parents[1]
APP_DIR = Path(__file__).resolve().parent
load_dotenv(WEB_DIR / ".env")

DATABASE_URL = os.getenv(
    "DATABASE_URL",
    "postgresql://just_pick_it_user:just_pick_it_pw@localhost:5432/just_pick_it",
)

# 공용 패키지의 product_images 가 정적 이미지 존재 여부를 확인할 디렉터리를 웹 static 경로로 지정한다.
os.environ.setdefault("JUST_PICK_IT_STATIC_IMG_DIR", str(APP_DIR / "static" / "img"))
APP_HOST = os.getenv("APP_HOST", "0.0.0.0")
APP_PORT = int(os.getenv("APP_PORT", "8000"))
