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

# Claude API 직접 연동용.
# 키/모델이 비어 있으면 관리자 AI 명령은 mock 응답으로 동작한다.
ANTHROPIC_API_KEY = os.getenv("ANTHROPIC_API_KEY", "")
CLAUDE_MODEL = os.getenv("CLAUDE_MODEL", "claude-sonnet-4-6")
CLAUDE_MAX_TOKENS = int(os.getenv("CLAUDE_MAX_TOKENS") or "512")
CLAUDE_TIMEOUT_SECONDS = float(os.getenv("CLAUDE_TIMEOUT_SECONDS") or "10")
