from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session

from app.database import get_db
from app.models import Product
from app.schemas import ProductRead
from app.services.status_service import build_product_summary


router = APIRouter(prefix="/api/products", tags=["products"])


@router.get("", response_model=list[ProductRead])
def list_products(db: Session = Depends(get_db)):
    products = db.query(Product).order_by(Product.product_id).all()
    return [build_product_summary(db, product) for product in products]
