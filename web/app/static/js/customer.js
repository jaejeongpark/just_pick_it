const orderStatusList = document.querySelector("#order-status-list");
const orderButton = document.querySelector("#order-button");
const productList = document.querySelector("#product-list");
const productCount = document.querySelector("#product-count");
const cartList = document.querySelector("#cart-list");
const clearCartButton = document.querySelector("#clear-cart-button");
const orderCountPill = document.querySelector("#order-count-pill");

let productsById = new Map();
let cart = new Map();
let activeOrders = new Map();
let customerSocket = null;
let fallbackTimer = null;
let failedOrderKey = 0;

const orderStatusText = {
  ORDER_RECEIVED: "주문 접수",
  ORDER_WAIT: "주문 대기",
  SORTING: "상품 선별 중",
  DELIVERING: "상품 운반 중",
  INSPECTING: "상품 검수 중",
  PICKUP_READY: "픽업 준비 완료",
  COMPLETED: "수령 완료",
  ERROR: "예외 발생",
};

function getCartTotal() {
  return Array.from(cart.values()).reduce((total, quantity) => total + quantity, 0);
}

function updateOrderButton() {
  if (!orderButton) {
    return;
  }

  orderButton.disabled = cart.size === 0;
}

function updateOrderCount() {
  if (orderCountPill) {
    orderCountPill.textContent = `${activeOrders.size}건`;
  }
}

function removeEmptyOrderState() {
  orderStatusList?.querySelector(".empty-state")?.remove();
}

function renderEmptyOrderStateIfNeeded() {
  if (!orderStatusList || activeOrders.size > 0) {
    return;
  }

  orderStatusList.innerHTML = '<div class="empty-state">진행 중인 주문이 없습니다</div>';
}

function removeOrderCard(orderId) {
  const card = orderStatusList?.querySelector(`[data-order-id="${orderId}"]`);
  card?.remove();
  activeOrders.delete(orderId);
  updateOrderCount();
  renderEmptyOrderStateIfNeeded();
}

function formatPickupSlotName(slotName) {
  if (!slotName) {
    return "";
  }

  const numberMatch = slotName.match(/\d+$/);
  return numberMatch ? `${numberMatch[0]}번` : slotName;
}

function createOrderCard(order) {
  const card = document.createElement("article");
  card.className = "status-line order-status-card";
  card.dataset.orderId = String(order.order_id);
  orderStatusList.prepend(card);
  return card;
}

function renderOrderCard(order) {
  if (!orderStatusList) {
    return;
  }

  if (order.status === "COMPLETED") {
    removeOrderCard(order.order_id);
    return;
  }

  removeEmptyOrderState();

  const statusText = orderStatusText[order.status] || order.status;
  const pickupName = formatPickupSlotName(order.pickup_slot_name);
  const pickupSlot = order.status === "PICKUP_READY" && pickupName
    ? `<div class="pickup-line"><span>픽업 칸</span><strong>${pickupName}</strong></div>`
    : "";
  const itemList = (order.items || [])
    .map((item) => `<span>${item.product_name} ${item.quantity}개</span>`)
    .join("");
  const completeButton = order.status === "PICKUP_READY"
    ? `<button class="small-action-button" type="button" data-complete-order="${order.order_id}">픽업 완료</button>`
    : "";

  let card = orderStatusList.querySelector(`[data-order-id="${order.order_id}"]`);

  if (!card) {
    card = createOrderCard(order);
  }

  card.innerHTML = `
    <div class="order-card-main">
      <strong>${order.order_no}</strong>
      <span>주문 ID #${order.order_id}</span>
    </div>
    <div class="order-item-list">
      ${itemList || "<span>상품 정보 없음</span>"}
    </div>
    <div class="order-card-actions">
      <div class="order-status-chip">${statusText}</div>
      ${pickupSlot}
      ${completeButton}
    </div>
  `;
}

function updateOrderStatus(order) {
  if (order.status === "COMPLETED") {
    removeOrderCard(order.order_id);
    return;
  }

  const statusText = orderStatusText[order.status] || order.status;
  activeOrders.set(order.order_id, { ...order, status_text: statusText });
  renderOrderCard(order);
  updateOrderCount();
}

async function loadOrders() {
  if (!orderStatusList) {
    return;
  }

  try {
    const response = await fetch("/api/orders");
    if (!response.ok) {
      throw new Error("failed to load orders");
    }

    const orders = await response.json();
    activeOrders.clear();
    orderStatusList.innerHTML = "";

    if (orders.length === 0) {
      orderStatusList.innerHTML = '<div class="empty-state">진행 중인 주문이 없습니다</div>';
      updateOrderCount();
      return;
    }

    orders.reverse().forEach((order) => {
      activeOrders.set(order.order_id, order);
      renderOrderCard(order);
    });
    updateOrderCount();
  } catch (error) {
    orderStatusList.innerHTML = '<div class="empty-state">주문 상태를 불러오지 못했습니다</div>';
  }
}

function syncCartWithProducts() {
  for (const [productId, quantity] of cart.entries()) {
    const product = productsById.get(productId);

    if (!product || product.stock_qty <= 0) {
      cart.delete(productId);
    } else if (quantity > product.stock_qty) {
      cart.set(productId, product.stock_qty);
    }
  }
}

function renderOrderSnapshot(orders) {
  if (!orderStatusList) {
    return;
  }

  activeOrders.clear();
  orderStatusList.innerHTML = "";

  if (!orders || orders.length === 0) {
    renderEmptyOrderStateIfNeeded();
    updateOrderCount();
    return;
  }

  [...orders].reverse().forEach((order) => {
    activeOrders.set(order.order_id, order);
    renderOrderCard(order);
  });
  updateOrderCount();
}

function renderCustomerSnapshot(data) {
  const products = data.products || [];
  productsById = new Map(products.map((product) => [product.product_id, product]));

  if (productCount) {
    productCount.textContent = `${products.length}개`;
  }

  syncCartWithProducts();
  renderProducts(products);
  renderCart();
  renderOrderSnapshot(data.orders || []);
}

async function loadCustomerStatus() {
  try {
    const response = await fetch("/api/customer/status");
    if (!response.ok) {
      throw new Error("failed to load customer status");
    }

    renderCustomerSnapshot(await response.json());
  } catch (error) {
    if (productList) {
      productList.innerHTML = '<p class="muted">상태 정보를 불러오지 못했습니다</p>';
    }
  }
}

function startFallbackPolling() {
  if (fallbackTimer) {
    return;
  }

  fallbackTimer = setInterval(loadCustomerStatus, 3000);
}

function stopFallbackPolling() {
  if (!fallbackTimer) {
    return;
  }

  clearInterval(fallbackTimer);
  fallbackTimer = null;
}

function connectCustomerSocket() {
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  customerSocket = new WebSocket(`${protocol}://${window.location.host}/api/customer/ws/status`);

  customerSocket.addEventListener("open", () => {
    stopFallbackPolling();
  });

  customerSocket.addEventListener("message", (event) => {
    renderCustomerSnapshot(JSON.parse(event.data));
  });

  customerSocket.addEventListener("close", () => {
    startFallbackPolling();
    setTimeout(connectCustomerSocket, 3000);
  });

  customerSocket.addEventListener("error", () => {
    customerSocket.close();
  });
}

function renderCart() {
  if (!cartList) {
    return;
  }

  const items = Array.from(cart.entries());

  if (items.length === 0) {
    cartList.innerHTML = '<p class="muted">선택한 상품이 없습니다</p>';
    updateOrderButton();
    return;
  }

  cartList.innerHTML = items
    .map(([productId, quantity]) => {
      const product = productsById.get(productId);
      const toneClass = `product-tone-${((product.product_id - 1) % 6) + 1}`;
      const imageText = product.name.replace("Test ", "").slice(0, 2).toUpperCase();
      const remainingQuantity = product.stock_qty - quantity;
      return `
        <div class="cart-row ${toneClass}">
          <div class="cart-item-main">
            <div class="cart-image">${imageText}</div>
            <div>
              <strong>${product.name}</strong>
              <span>남은 수량 ${remainingQuantity}개</span>
            </div>
          </div>
          <div class="quantity-control">
            <button class="icon-button" type="button" data-cart-decrease="${productId}" aria-label="Decrease ${product.name}">-</button>
            <output>${quantity}</output>
            <button class="icon-button" type="button" data-cart-increase="${productId}" aria-label="Increase ${product.name}">+</button>
          </div>
        </div>
      `;
    })
    .join("");

  updateOrderButton();
}

function setCartQuantity(productId, quantity) {
  const product = productsById.get(productId);

  if (!product) {
    return;
  }

  if (quantity <= 0) {
    cart.delete(productId);
  } else {
    cart.set(productId, Math.min(quantity, product.stock_qty));
  }

  renderProducts(Array.from(productsById.values()));
  renderCart();
}

function renderProducts(products) {
  if (!productList) {
    return;
  }

  if (products.length === 0) {
    productList.innerHTML = '<p class="muted">등록된 상품이 없습니다</p>';
    return;
  }

  productList.innerHTML = products
    .map((product) => {
      const selectedQuantity = cart.get(product.product_id) || 0;
      const remainingQuantity = product.stock_qty - selectedQuantity;
      const isSoldOut = remainingQuantity <= 0;
      return `
        <article class="product-card product-tone-${((product.product_id - 1) % 6) + 1}">
          <div class="product-image">${product.name.replace("Test ", "").slice(0, 2).toUpperCase()}</div>
          <div class="product-main">
            <h3>${product.name}</h3>
          </div>
          <div class="product-meta">
            <span>남은 수량 ${remainingQuantity}개</span>
            <div class="quantity-control">
              <button class="icon-button" type="button" data-decrease="${product.product_id}" aria-label="Decrease ${product.name}">-</button>
              <output>${selectedQuantity}</output>
              <button class="icon-button" type="button" data-increase="${product.product_id}" aria-label="Increase ${product.name}" ${isSoldOut ? "disabled" : ""}>+</button>
            </div>
          </div>
        </article>
      `;
    })
    .join("");
}

async function loadProducts() {
  if (!productList) {
    return;
  }

  try {
    const response = await fetch("/api/products");
    if (!response.ok) {
      throw new Error("failed to load products");
    }
    const products = await response.json();
    productsById = new Map(products.map((product) => [product.product_id, product]));
    if (productCount) {
      productCount.textContent = `${products.length}개`;
    }
    renderProducts(products);
  } catch (error) {
    productList.innerHTML = '<p class="muted">상품 정보를 불러오지 못했습니다</p>';
  }
}

orderButton?.addEventListener("click", async () => {
  const items = Array.from(cart.entries()).map(([productId, quantity]) => ({
    product_id: productId,
    quantity,
  }));

  orderButton.disabled = true;

  try {
    const response = await fetch("/api/orders", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ items }),
    });

    if (!response.ok) {
      throw new Error("failed to create order");
    }

    const order = await response.json();
    cart.clear();
    renderCart();
    updateOrderStatus(order);
  } catch (error) {
    failedOrderKey += 1;
    const failedOrder = {
      order_id: `failed-${failedOrderKey}`,
      order_no: "주문 실패",
      status: "ERROR",
      pickup_slot_name: null,
    };
    activeOrders.set(failedOrder.order_id, failedOrder);
    renderOrderCard(failedOrder);
    updateOrderCount();
    updateOrderButton();
  }
});

clearCartButton?.addEventListener("click", () => {
  cart.clear();
  renderProducts(Array.from(productsById.values()));
  renderCart();
});

productList?.addEventListener("click", (event) => {
  const button = event.target.closest("button");

  if (!button) {
    return;
  }

  const increaseId = button.dataset.increase;
  const decreaseId = button.dataset.decrease;

  if (increaseId) {
    const productId = Number(increaseId);
    setCartQuantity(productId, (cart.get(productId) || 0) + 1);
  }

  if (decreaseId) {
    const productId = Number(decreaseId);
    setCartQuantity(productId, (cart.get(productId) || 0) - 1);
  }
});

cartList?.addEventListener("click", (event) => {
  const button = event.target.closest("button");

  if (!button) {
    return;
  }

  const increaseId = button.dataset.cartIncrease;
  const decreaseId = button.dataset.cartDecrease;

  if (increaseId) {
    const productId = Number(increaseId);
    setCartQuantity(productId, (cart.get(productId) || 0) + 1);
  }

  if (decreaseId) {
    const productId = Number(decreaseId);
    setCartQuantity(productId, (cart.get(productId) || 0) - 1);
  }
});

orderStatusList?.addEventListener("click", async (event) => {
  const button = event.target.closest("button[data-complete-order]");

  if (!button) {
    return;
  }

  const orderId = Number(button.dataset.completeOrder);
  button.disabled = true;

  try {
    const response = await fetch(`/api/orders/${orderId}/complete`, {
      method: "POST",
    });

    if (!response.ok) {
      throw new Error("failed to complete order");
    }

    const order = await response.json();
    updateOrderStatus(order);
  } catch (error) {
    button.disabled = false;
  }
});

loadProducts();
loadOrders();
connectCustomerSocket();
