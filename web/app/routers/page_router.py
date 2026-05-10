from pathlib import Path

from fastapi import APIRouter, Request
from fastapi.responses import RedirectResponse
from fastapi.templating import Jinja2Templates


BASE_DIR = Path(__file__).resolve().parents[1]
templates = Jinja2Templates(directory=BASE_DIR / "templates")

router = APIRouter()


@router.get("/")
def index():
    return RedirectResponse(url="/customer")


@router.get("/customer")
def customer_page(request: Request):
    return templates.TemplateResponse(request, "customer.html")


@router.get("/admin")
def admin_page(request: Request):
    return templates.TemplateResponse(
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
    return templates.TemplateResponse(
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
    return templates.TemplateResponse(
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
    return templates.TemplateResponse(
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
    return templates.TemplateResponse(
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
    return templates.TemplateResponse(
        request,
        "admin.html",
        {
            "page": "inventory",
            "page_title": "재고 관리",
            "page_description": "상품 재고, 보관 위치, 부족 상태를 관리합니다.",
        },
    )
