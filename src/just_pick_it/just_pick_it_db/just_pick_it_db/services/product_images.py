import os
from pathlib import Path

from just_pick_it_db.models import Product


STATIC_IMG_URL_PREFIX = "/static/img/"
OLD_STATIC_IMG_URL_PREFIX = "/static/images/"
# 정적 이미지가 실제로 있는 디렉터리. 웹은 자신의 static/img 경로를
# JUST_PICK_IT_STATIC_IMG_DIR 환경변수로 지정한다. Fleet Manager 처럼 정적 파일이
# 없는 환경에서는 미설정 상태로 두며, 파일 존재 확인이 실패하면 기본 URL 후보를 그대로 반환한다.
STATIC_IMG_DIR = Path(
    os.getenv(
        "JUST_PICK_IT_STATIC_IMG_DIR",
        str(Path(__file__).resolve().parent / "static" / "img"),
    )
)
PRODUCT_IMAGE_FILENAMES = {
    "우유": "milk.png",
    "시리얼": "cereal.png",
    "바나나 우유": "banana_milk.png",
    "식빵": "bread.png",
    "투게더": "together.png",
    "바나나": "banana.png",
}
STATIC_IMAGE_FILENAME_ALIASES = {
    alias_filename: filename
    for name, filename in PRODUCT_IMAGE_FILENAMES.items()
    for alias_filename in (name, f"{name}.png")
}


def resolve_product_image_url(product: Product) -> str | None:
    candidates = []
    image_url = product.image_url.strip() if product.image_url else ""

    if image_url.startswith(OLD_STATIC_IMG_URL_PREFIX):
        candidates.append(
            normalize_static_image_url(
                image_url.replace(
                    OLD_STATIC_IMG_URL_PREFIX, STATIC_IMG_URL_PREFIX, 1
                )
            )
        )
    elif image_url.startswith(STATIC_IMG_URL_PREFIX):
        candidates.append(normalize_static_image_url(image_url))
    elif image_url:
        return image_url

    filename = PRODUCT_IMAGE_FILENAMES.get(product.name, f"{product.name}.png")
    candidates.append(f"{STATIC_IMG_URL_PREFIX}{filename}")

    for candidate in candidates:
        resolved_candidate = resolve_static_image_candidate(candidate)

        if resolved_candidate:
            return resolved_candidate

    return candidates[0] if candidates else None


def normalize_static_image_url(image_url: str) -> str:
    if not image_url.startswith(STATIC_IMG_URL_PREFIX):
        return image_url

    relative_path = image_url.removeprefix(STATIC_IMG_URL_PREFIX)
    normalized_filename = STATIC_IMAGE_FILENAME_ALIASES.get(relative_path)

    if normalized_filename:
        return f"{STATIC_IMG_URL_PREFIX}{normalized_filename}"

    return image_url


def resolve_static_image_candidate(image_url: str) -> str | None:
    if not image_url.startswith(STATIC_IMG_URL_PREFIX):
        return None

    relative_path = image_url.removeprefix(STATIC_IMG_URL_PREFIX)
    image_path = STATIC_IMG_DIR / relative_path

    if image_path.exists():
        return image_url

    if image_path.suffix:
        return None

    if image_path.with_suffix(".png").exists():
        return f"{image_url}.png"

    return None
