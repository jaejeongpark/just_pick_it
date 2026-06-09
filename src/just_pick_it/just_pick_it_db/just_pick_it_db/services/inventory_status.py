AUTO_DISPLAY_REQUEST_QTY = 2
LOW_STOCK_MAX = 2
WARNING_STOCK_QTY = 3
NORMAL_STOCK_MIN = 4


def stock_level(stock_qty: int) -> str:
    if stock_qty <= LOW_STOCK_MAX:
        return "low"

    if stock_qty == WARNING_STOCK_QTY:
        return "warning"

    if stock_qty >= NORMAL_STOCK_MIN:
        return "normal"

    return "low"


def is_low_stock(stock_qty: int) -> bool:
    return stock_level(stock_qty) == "low"
