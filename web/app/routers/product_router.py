from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session

from app.database import get_db
from app.models import Product
from app.schemas import ProductRead
from app.services.product_images import resolve_product_image_url


router = APIRouter(prefix="/api/products", tags=["products"])


@router.get("", response_model=list[ProductRead])
def list_products(db: Session = Depends(get_db)):
    products = db.query(Product).order_by(Product.product_id).all()
    return [build_product_response(product) for product in products]


def build_product_response(product: Product) -> dict:
    return {
        "product_id": product.product_id,
        "name": product.name,
        "image_url": resolve_product_image_url(product),
        "stock_qty": product.stock_qty,
        "storage_location": product.storage_location,
    }
