from pathlib import Path

from app.models import Product


STATIC_IMG_URL_PREFIX = "/static/img/"
LEGACY_STATIC_IMG_URL_PREFIX = "/static/images/"
STATIC_IMG_DIR = Path(__file__).resolve().parents[1] / "static" / "img"
PRODUCT_IMAGE_FILENAMES = {
    "우유": "milk.png",
    "시리얼": "cereal.png",
    "바나나 우유": "banana_milk.png",
    "식빵": "bread.png",
    "투게더": "together.png",
    "바나나": "banana.png",
}
STATIC_IMAGE_FILENAME_ALIASES = {
    legacy_filename: filename
    for name, filename in PRODUCT_IMAGE_FILENAMES.items()
    for legacy_filename in (name, f"{name}.png")
}


def resolve_product_image_url(product: Product) -> str | None:
    candidates = []
    image_url = product.image_url.strip() if product.image_url else ""

    if image_url.startswith(LEGACY_STATIC_IMG_URL_PREFIX):
        candidates.append(
            normalize_static_image_url(
                image_url.replace(
                    LEGACY_STATIC_IMG_URL_PREFIX, STATIC_IMG_URL_PREFIX, 1
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
