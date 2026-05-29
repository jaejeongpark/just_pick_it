from pathlib import Path

from fastapi import APIRouter, Request
from fastapi.responses import RedirectResponse
from fastapi.templating import Jinja2Templates


BASE_DIR = Path(__file__).resolve().parents[1]
templates = Jinja2Templates(directory=BASE_DIR / "templates")

router = APIRouter()


# =====================================
# Template helpers
# =====================================

def render_template(request: Request, template_name: str, context: dict | None = None):
    """TemplateResponse helper. 최신 Starlette 시그니처(request 를 첫 인자로) 사용.
    구식 (name, context) 호출은 최신 Starlette 에서 name 자리에 dict 가 들어가
    'unhashable type: dict' 로 깨진다."""
    template_context = {"request": request}
    if context:
        template_context.update(context)
    return templates.TemplateResponse(request, template_name, template_context)


# =====================================
# Customer pages
# =====================================

@router.get("/")
def index():
    return RedirectResponse(url="/customer")


@router.get("/customer")
def customer_page(request: Request):
    return render_template(request, "customer.html")


# =====================================
# Admin pages
# =====================================

@router.get("/admin")
def admin_page(request: Request):
    return render_template(
        request,
        "admin.html",
        {
            "page": "dashboard",
            "page_title": "대시보드",
            "page_description": "로봇, 주문, 예외, 재고 상태를 한 화면에서 확인합니다.",
        },
    )


@router.get("/admin/map")
def admin_map_page(request: Request):
    return render_template(
        request,
        "admin.html",
        {
            "page": "map",
            "page_title": "미니맵",
            "page_description": "로봇 위치와 작업 구역을 확인합니다.",
        },
    )


@router.get("/admin/robots")
def admin_robots_page(request: Request):
    return render_template(
        request,
        "admin.html",
        {
            "page": "robots",
            "page_title": "로봇 관리",
            "page_description": "로봇 상태, 배터리, 현재 작업을 관리합니다.",
        },
    )


@router.get("/admin/orders")
def admin_orders_page(request: Request):
    return render_template(
        request,
        "admin.html",
        {
            "page": "orders",
            "page_title": "작업/주문 관리",
            "page_description": "주문, 작업, 픽업 슬롯 상태를 관리합니다.",
        },
    )


@router.get("/admin/exceptions")
def admin_exceptions_page(request: Request):
    return render_template(
        request,
        "admin.html",
        {
            "page": "exceptions",
            "page_title": "예외/알람 관리",
            "page_description": "미처리 예외와 알람 이력을 확인합니다.",
        },
    )


@router.get("/admin/inventory")
def admin_inventory_page(request: Request):
    return render_template(
        request,
        "admin.html",
        {
            "page": "inventory",
            "page_title": "재고 관리",
            "page_description": "상품 재고, 보관 위치, 부족 상태를 관리합니다.",
        },
    )
