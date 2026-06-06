// =====================================
// DOM references / state
// =====================================

const socketState = document.querySelector("#socket-state");
const summaryRobots = document.querySelector("#summary-robots");
const summaryActiveRobots = document.querySelector("#summary-active-robots");
const summaryRunningRobots = document.querySelector("#summary-running-robots");
const summaryIdleRobots = document.querySelector("#summary-idle-robots");
const summaryWorkingRobots = document.querySelector("#summary-working-robots");
const summaryErrorRobots = document.querySelector("#summary-error-robots");
const robotDonut = document.querySelector("#robot-donut");
const summaryOrders = document.querySelector("#summary-orders");
const summaryExceptions = document.querySelector("#summary-exceptions");
const summaryTasks = document.querySelector("#summary-tasks");
const robotStatus = document.querySelector("#robot-status");
const mapRobotLayer = document.querySelector("#map-robot-layer");
const dashboardMap = document.querySelector(".dashboard-grid .warehouse-map");
const robotDetailPanel = document.querySelector("#robot-detail-panel");
const robotSearchInput = document.querySelector("#robot-search-input");
const robotStatusFilter = document.querySelector("#robot-status-filter");
const robotTypeFilter = document.querySelector("#robot-type-filter");
const orderList = document.querySelector("#order-list");
const orderWorkDetailPanel = document.querySelector("#order-work-detail-panel");
const pickupSlotList = document.querySelector("#pickup-slot-list");
const pickupSummaryList = document.querySelector("#pickup-summary-list");
const inventoryList = document.querySelector("#inventory-list");
const taskList = document.querySelector("#task-list");
const exceptionList = document.querySelector("#exception-list");
const emergencyStopButton = document.querySelector("#emergency-stop-button");
const resumeButton = document.querySelector("#resume-button");
const orderHistoryButton = document.querySelector("#order-history-button");
const exceptionHistoryButton = document.querySelector(
  "#exception-history-button",
);
const robotManageButton = document.querySelector("#robot-manage-button");
const pickupSlotManageButton = document.querySelector(
  "#pickup-slot-manage-button",
);
const inventoryManageButton = document.querySelector(
  "#inventory-manage-button",
);
const taskCreateButton = document.querySelector("#task-create-button");
const taskViewButton = document.querySelector("#task-view-button");
const modalBackdrop = document.querySelector("#modal-backdrop");
const modalPanel = document.querySelector(".modal-panel");
const modalTitle = document.querySelector("#modal-title");
const modalBody = document.querySelector("#modal-body");
const modalCloseButton = document.querySelector("#modal-close-button");
const llmPanel = document.querySelector("#llm-panel");
const llmOpenButton = document.querySelector("#llm-open-button");
const llmCloseButton = document.querySelector("#llm-close-button");
const llmMessages = document.querySelector("#llm-messages");
const llmForm = document.querySelector("#llm-form");
const llmInput = document.querySelector("#llm-input");
const dashboardLlmForm = document.querySelector("#dashboard-llm-form");
const dashboardLlmInput = document.querySelector("#dashboard-llm-input");
const dashboardLlmStatus = document.querySelector("#dashboard-llm-status");
const dashboardLlmResult = document.querySelector("#dashboard-llm-result");
const adminPage = document.body.dataset.adminPage || "dashboard";
let adminSocket = null;
let fallbackTimer = null;
let latestAdminStatus = null;
let selectedRobotId = null;
let selectedOrderId = null;
let selectedDisplayItemId = null;
let selectedTaskId = null;
let zoneOptionsCache = null;
let mapZonesLoading = false;
let modalReturnStack = [];
const mapRobotMovingState = new Map();
const mapRobotArrivalFlashUntil = new Map();

// =====================================
// Domain constants
// =====================================

const MAP_WIDTH_METERS = 2.0;
const MAP_HEIGHT_METERS = 1.0;
const MAP_ASPECT_RATIO = MAP_WIDTH_METERS / MAP_HEIGHT_METERS;
const MAP_LAYER_PADDING_PX = 10;
const STOCK_LEVELS = new Set(["low", "warning", "normal"]);
const ORDER_STATUSES = [
  "ORDER_RECEIVED",
  "ORDER_WAIT",
  "SORTING",
  "DELIVERING",
  "INSPECTING",
  "PICKUP_READY",
  "COMPLETED",
  "ERROR",
];
const TASK_STATUSES = [
  "QUEUED",
  "ASSIGNED",
  "RUNNING",
  "PAUSED",
  "SUCCESS",
  "FAILED",
  "CANCELLED",
];
const FINAL_TASK_STATUSES = new Set(["SUCCESS", "FAILED", "CANCELLED"]);
const FINAL_DISPLAY_ITEM_STATUSES = new Set(["COMPLETED", "FAILED", "CANCELLED"]);
const ACTIVE_TASK_STATUSES = new Set([
  "QUEUED",
  "ASSIGNED",
  "RUNNING",
  "PAUSED",
]);
const TASK_TYPE_SEQUENCE = [
  "MOVE_TO_PRODUCT",
  "SORTING_AND_LOAD",
  "MOVE_TO_PICKUP",
  "INSPECTION",
  "UNLOAD",
  "MOVE_TO_STOCK",
  "MOVE_TO_DISPLAY",
  "DISPLAY_SCAN",
  "DISPLAY_PLACE",
  "RETURN_HOME",
  "DOCK_IN",
  "CHARGE",
];
const DISPLAY_TASK_TYPES = new Set([
  "MOVE_TO_STOCK",
  "MOVE_TO_DISPLAY",
  "DISPLAY_SCAN",
  "DISPLAY_PLACE",
]);
const ROBOT_DISPLAY_NAMES = {
  PICKY1: "PICKY 1",
  PICKY2: "PICKY 2",
  COBOT1: "COBOT 1",
  COBOT2: "COBOT 2",
};
const ROBOT_TYPES = ["PICKY", "COBOT"];
const ROBOT_STATUSES = [
  "OFFLINE",
  "IDLE",
  "BUSY",
  "CHARGING",
  "EMERGENCY_STOP",
  "ERROR",
];
const PICKY_STATES = [
  "CHARGING",
  "STANDBY",
  "MOVING_TO_PRODUCT",
  "WAITING_FOR_COBOT",
  "MOVING_TO_PICKUP",
  "MOVING_TO_STOCK",
  "MOVING_TO_DISPLAY",
  "RETURNING",
  "DOCKING",
  "ERROR_RECOVERY",
];
const MAP_MOVING_STATES = new Set([
  "MOVING_TO_PRODUCT",
  "MOVING_TO_PICKUP",
  "MOVING_TO_STOCK",
  "MOVING_TO_DISPLAY",
  "RETURNING",
  "DOCKING",
]);
const MAP_MOVING_TASK_TYPES = new Set([
  "MOVE_TO_PRODUCT",
  "MOVE_TO_PICKUP",
  "MOVE_TO_STOCK",
  "MOVE_TO_DISPLAY",
  "RETURN_HOME",
  "DOCK_IN",
]);
const COBOT_STATES = [
  "STANDBY",
  "SORTING",
  "LOADING",
  "INSPECTING",
  "UNLOADING",
  "SCANNING",
  "PLACING",
  "STOWING_ARM",
  "SAFETY_STOPPED",
];
const PICKUP_SLOT_STATUSES = ["EMPTY", "RESERVED", "OCCUPIED", "BLOCKED"];
const STOCK_LEVEL_LABELS = {
  low: "부족",
  warning: "부족 임박",
  normal: "정상",
};
const STOCK_LOW_MAX = 0;
const STOCK_WARNING_QTY = 2;
const STOCK_NORMAL_MIN = 3;
const STOCK_LEVEL_CLASSES = {
  low: "table-danger",
  warning: "table-warning",
  normal: "table-ok",
};

// =====================================
// Display labels
// =====================================

const statusText = {
  ORDER_RECEIVED: "주문 접수",
  ORDER_WAIT: "주문 대기",
  REQUESTED: "요청됨",
  ASSIGNED: "배정됨",
  IN_PROGRESS: "진행 중",
  SORTING: "선별/상차 중",
  MOVE_TO_PRODUCT: "상품 위치 이동",
  SORTING_AND_LOAD: "선별/상차",
  MOVE_TO_PICKUP: "픽업존 이동",
  INSPECTION: "검수",
  UNLOAD: "하차",
  MOVE_TO_STOCK: "창고존 이동",
  MOVE_TO_DISPLAY: "진열 구역 이동",
  DISPLAY_SCAN: "진열대 스캔",
  DISPLAY_PLACE: "상품 진열",
  DOCK_IN: "도킹",
  CHARGE: "충전",
  RETURN_HOME: "복귀",
  DELIVERING: "운반 중",
  INSPECTING: "검수 중",
  PICKUP_READY: "픽업 준비",
  COMPLETED: "완료",
  ERROR: "예외",
  PICKY: "PICKY",
  COBOT: "COBOT",
  IDLE: "대기",
  BUSY: "작업 중",
  MOVING_TO_PRODUCT: "상품 위치 이동 중",
  WAITING_FOR_COBOT: "코봇 작업 대기",
  MOVING_TO_PICKUP: "픽업존 이동 중",
  MOVING_TO_STOCK: "창고존 이동 중",
  MOVING_TO_DISPLAY: "진열 구역 이동 중",
  STANDBY: "대기",
  LOADING: "상차 중",
  UNLOADING: "하차 중",
  SCANNING: "진열대 스캔 중",
  PLACING: "상품 진열 중",
  STOWING_ARM: "팔 기본 자세 복귀",
  SAFETY_STOPPED: "안전 정지",
  CHARGING: "충전",
  RETURNING: "복귀",
  DOCKING: "도킹 중",
  ERROR_RECOVERY: "오류 복구",
  EMERGENCY_STOP: "긴급정지",
  OFFLINE: "오프라인",
  EMPTY: "비어 있음",
  RESERVED: "예약됨",
  OCCUPIED: "픽업 대기",
  BLOCKED: "사용 불가",
  QUEUED: "큐 대기",
  ASSIGNED: "할당됨",
  RUNNING: "진행 중",
  PAUSED: "일시정지",
  SUCCESS: "성공",
  FAILED: "실패",
  CANCELLED: "취소",
};

const robotStateText = {
  SORTING: "상품 선별",
  LOADING: "상품 상차",
  INSPECTING: "상품 검수",
  UNLOADING: "상품 하차",
  SCANNING: "진열대 스캔",
  PLACING: "상품 진열 중",
  STOWING_ARM: "팔 기본 자세 복귀",
};

// =====================================
// Common helpers
// =====================================

function setSocketState(text) {
  if (socketState) {
    socketState.textContent = text;
    socketState.classList.toggle("danger", text !== "online");
  }
}

setSocketState("offline");

function label(value) {
  return statusText[value] || value || "-";
}

function escapeAttribute(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll('"', "&quot;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;");
}

function robotStateTextLabel(value) {
  return robotStateText[value] || label(value);
}

function normalizeId(value) {
  return value === null || value === undefined ? "" : String(value);
}

function sameId(left, right) {
  return normalizeId(left) === normalizeId(right);
}

// =====================================
// Robot helpers
// =====================================

function findRobotById(robotId) {
  if (robotId && typeof robotId === "object") {
    return robotId;
  }

  const normalizedRobotId = normalizeId(robotId);

  return (latestAdminStatus?.robots || []).find(
    (robot) =>
      sameId(robot.robot_id, normalizedRobotId) ||
      robot.robot_name === normalizedRobotId,
  );
}

function robotStatusValue(robot) {
  return robot?.robot_status || robot?.status || "OFFLINE";
}

function robotStateValue(robot) {
  if (!robot) {
    return null;
  }

  return robotType(robot) === "PICKY" ? robot.picky_state : robot.cobot_state;
}

function isRobotMovingOnMap(robot) {
  const state = robotStateValue(robot);
  if (MAP_MOVING_STATES.has(state)) {
    return true;
  }

  const currentTaskType = robot?.current_task_type || robot?.current_task?.task_type;
  const currentTaskStatus =
    robot?.current_task_status || robot?.current_task?.status;

  return (
    currentTaskStatus === "RUNNING" &&
    MAP_MOVING_TASK_TYPES.has(currentTaskType)
  );
}

function mapRobotKey(robot) {
  return normalizeId(robot?.robot_name || robot?.robot_id || "UNKNOWN");
}

function shouldFlashMapArrival(robot, moving, now) {
  const key = mapRobotKey(robot);
  const wasMoving = mapRobotMovingState.get(key) === true;

  if (moving) {
    mapRobotArrivalFlashUntil.delete(key);
  } else if (wasMoving) {
    mapRobotArrivalFlashUntil.set(key, now + 2400);
  }

  mapRobotMovingState.set(key, moving);
  return !moving && (mapRobotArrivalFlashUntil.get(key) || 0) > now;
}

function robotStateLabel(robot) {
  return robotStateValue(robot)
    ? robotStateTextLabel(robotStateValue(robot))
    : "-";
}

function assignedRobotLabel(task) {
  if (!task?.assigned_robot_id && !task?.assigned_robot_name) {
    return "로봇 미배정";
  }

  return task.assigned_robot_name || robotDisplayName(task.assigned_robot_id);
}

// =====================================
// Task helpers
// =====================================

function findTaskOrderItem(task) {
  if (!task?.order_item_id) {
    return null;
  }

  const orders = [
    ...(latestAdminStatus?.orders || []),
    ...(latestAdminStatus?.order_history || []),
  ];
  const order = orders.find(
    (candidate) =>
      sameId(candidate.order_id, task.order_id) ||
      candidate.order_no === task.order_no,
  );

  return (order?.items || []).find((item) =>
    sameId(item.item_id, task.order_item_id),
  );
}

function taskProductName(task) {
  const orderItem = findTaskOrderItem(task);

  return task?.product_name || orderItem?.product_name || null;
}

function isDisplayTaskType(taskType) {
  return DISPLAY_TASK_TYPES.has(taskType);
}

function taskDisplayTitle(task) {
  const productName = taskProductName(task);

  if (!productName) {
    return label(task?.task_type);
  }

  const productTaskLabels = {
    MOVE_TO_PRODUCT: `${productName} 위치 이동`,
    SORTING_AND_LOAD: `${productName} 선별/상차`,
    MOVE_TO_STOCK: `${productName} 창고존 이동`,
    MOVE_TO_DISPLAY: `${productName} 진열 구역 이동`,
    DISPLAY_SCAN: `${productName} 진열대 스캔`,
    DISPLAY_PLACE: `${productName} 진열`,
  };

  return productTaskLabels[task.task_type] || label(task.task_type);
}

function taskTargetLabel(task) {
  const orderNumber = taskOrderNumber(task);
  if (orderNumber) {
    return orderNumber;
  }

  if (task?.display_item_id) {
    return `진열 #${task.display_item_id}`;
  }

  return `작업 #${task?.task_id ?? "-"}`;
}

function taskReferenceLabel(task) {
  if (task?.order_item_id) {
    return `주문 상품 #${task.order_item_id}`;
  }

  if (task?.display_item_id) {
    return `진열 요청 #${task.display_item_id}`;
  }

  return "단독 작업";
}

function taskRouteLabel(task) {
  return `${task?.source_zone_name || "출발 미정"} → ${task?.target_zone_name || "목표 미정"}`;
}

function taskQuantityLabel(task) {
  const productName = taskProductName(task);
  const quantity =
    task?.product_quantity ??
    task?.processed_quantity;

  if (productName && quantity !== null && quantity !== undefined) {
    return `${productName} ${quantity}개`;
  }

  if (productName) {
    return productName;
  }

  if (quantity !== null && quantity !== undefined) {
    return `${quantity}개`;
  }

  return "-";
}

function recommendedTaskPriority(taskType, displayItemId = null) {
  return isDisplayTaskType(taskType) || Boolean(displayItemId) ? 1 : 2;
}

// =====================================
// Formatting helpers
// =====================================

function productStorageLabel(product) {
  return (
    product.storage_zone_name ||
    product.storage_location ||
    (product.storage_zone_id ? `Zone #${product.storage_zone_id}` : "-")
  );
}

function formatSlotName(slotName) {
  if (!slotName) {
    return "-";
  }

  const numberMatch = slotName.match(/\d+$/);
  return numberMatch ? `${numberMatch[0]}번` : slotName;
}

function formatPickupSlot(slotName) {
  return slotName ? formatSlotName(slotName) : "배정 전";
}

function renderOrderItems(items) {
  if (!items || items.length === 0) {
    return '<div class="empty-state">주문 상품 정보가 없습니다</div>';
  }

  return `
    <div class="modal-item-list">
      ${items
        .map((item) => {
          const product = {
            product_id: item.product_id,
            name: item.product_name,
            image_url: item.image_url,
          };

          return `
          <div class="cart-row ${productToneClass(item.product_id)}">
            <div class="cart-item-main">
              ${productImageMarkup(product, "cart-image")}
              <div>
                <strong>${item.product_name}</strong>
                <span>${label(item.status)}</span>
              </div>
            </div>
            <div class="metric">${item.quantity}개</div>
          </div>
        `;
        })
        .join("")}
    </div>
  `;
}

function canEditOrderItemQuantities(order) {
  return (
    ["ORDER_RECEIVED", "ORDER_WAIT"].includes(order.status) &&
    orderTasks(order).length === 0
  );
}

function renderOrderItemQuantityEditor(order) {
  if (!order.items || order.items.length === 0) {
    return '<div class="empty-state">주문 상품 정보가 없습니다</div>';
  }

  const editable = canEditOrderItemQuantities(order);
  const disabled = editable ? "" : "disabled";
  const helpText = editable
    ? "Fleet 작업 생성 전 주문 상품 수량을 수정할 수 있습니다."
    : "진행 중이거나 작업이 생성된 주문은 상품 수량을 변경할 수 없습니다.";

  return `
    <div class="modal-subsection">
      <h3>상품 수량</h3>
      <div class="order-item-editor-list">
        ${order.items
          .map((item) => {
            const product = {
              product_id: item.product_id,
              name: item.product_name,
              image_url: item.image_url,
            };

            return `
            <div class="cart-row order-item-editor-row ${productToneClass(item.product_id)}">
              <div class="cart-item-main">
                ${productImageMarkup(product, "cart-image")}
                <div>
                  <strong>${item.product_name}</strong>
                  <span>${label(item.status)}</span>
                </div>
              </div>
              <div class="order-item-quantity-field">
                <label for="order-item-qty-${item.item_id}">수량</label>
                <input id="order-item-qty-${item.item_id}" data-order-item-quantity="${item.item_id}" data-current-quantity="${item.quantity}" type="number" min="1" value="${item.quantity}" ${disabled}>
              </div>
            </div>
          `;
          })
          .join("")}
      </div>
      <p class="muted">${helpText}</p>
    </div>
  `;
}

function productToneClass(productId) {
  return `product-tone-${((productId - 1) % 6) + 1}`;
}

function productImageText(product) {
  if (!product) {
    return "-";
  }

  return product.name.replace("Test ", "").slice(0, 2).toUpperCase();
}

function productImageMarkup(product, className = "inventory-product-image") {
  const fallbackText = productImageText(product);
  const image = product.image_url
    ? `<img src="${product.image_url}" alt="" onerror="this.remove(); this.parentElement.dataset.fallback='true';">`
    : "";

  return `
    <span class="${className} ${productToneClass(product.product_id)}" data-fallback="${product.image_url ? "false" : "true"}">
      ${image}
      <b>${fallbackText}</b>
    </span>
  `;
}

function formatDateTime(value) {
  if (!value) {
    return "-";
  }

  return new Date(value).toLocaleString("ko-KR", {
    month: "2-digit",
    day: "2-digit",
    hour: "2-digit",
    minute: "2-digit",
  });
}

function escapeHtml(value) {
  return String(value ?? "")
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;")
    .replace(/'/g, "&#039;");
}

function openModal(title, body, options = {}) {
  if (!options.keepReturnState) {
    modalReturnStack = [];
  }

  resetModalHeaderActions();
  modalTitle.textContent = title;
  modalBody.innerHTML = body;
  modalPanel?.classList.toggle("modal-compact", options.size === "compact");
  modalBackdrop.hidden = false;
}

function closeModal() {
  const returnState = modalReturnStack.pop();

  if (returnState) {
    restoreModal(returnState);
    return;
  }

  modalBackdrop.hidden = true;
  modalPanel?.classList.remove("modal-compact");
  resetModalHeaderActions();
}

function captureModalState() {
  if (!modalBackdrop || modalBackdrop.hidden) {
    return null;
  }

  return {
    title: modalTitle.textContent,
    body: modalBody.innerHTML,
    compact: modalPanel?.classList.contains("modal-compact") || false,
    bodyScrollTop: modalBody.scrollTop,
    historyScrollTop: [...modalBody.querySelectorAll(".completed-work-history-list")].map(
      (list) => list.scrollTop,
    ),
  };
}

function restoreModal(state) {
  modalTitle.textContent = state.title;
  modalBody.innerHTML = state.body;
  modalPanel?.classList.toggle("modal-compact", state.compact);
  modalBackdrop.hidden = false;
  resetModalHeaderActions();

  requestAnimationFrame(() => {
    modalBody.scrollTop = state.bodyScrollTop || 0;
    modalBody
      .querySelectorAll(".completed-work-history-list")
      .forEach((list, index) => {
        list.scrollTop = state.historyScrollTop?.[index] || 0;
      });
  });
}

function resetModalHeaderActions() {
  const headerActions = modalPanel?.querySelector(".modal-header-actions");

  if (!headerActions) {
    return;
  }

  headerActions.innerHTML = "";
  headerActions.appendChild(modalCloseButton);
}

function setModalHeaderActions(actionsMarkup) {
  const headerActions = modalPanel?.querySelector(".modal-header-actions");

  if (!headerActions) {
    return;
  }

  headerActions.innerHTML = actionsMarkup;
  headerActions.appendChild(modalCloseButton);
}

function renderOptions(
  values,
  selectedValue,
  emptyLabel = null,
  labelFormatter = label,
) {
  const isEmptySelected =
    selectedValue === null ||
    selectedValue === undefined ||
    selectedValue === "";
  const emptyOption =
    emptyLabel === null
      ? ""
      : `<option value="" ${isEmptySelected ? "selected" : ""}>${emptyLabel}</option>`;

  return `${emptyOption}${values
    .map(
      (value) => `
      <option value="${value}" ${sameId(value, selectedValue) ? "selected" : ""}>${labelFormatter(value)}</option>
    `,
    )
    .join("")}`;
}

function renderRobotOptions(selectedRobotId) {
  const robots = sortedRobots(latestAdminStatus?.robots || []);

  const emptySelected =
    selectedRobotId === null ||
    selectedRobotId === undefined ||
    selectedRobotId === "";

  return `
    <option value="" ${emptySelected ? "selected" : ""}>미배정</option>
    ${robots
      .map(
        (robot) => `
        <option value="${robot.robot_id}" ${sameId(robot.robot_id, selectedRobotId) ? "selected" : ""}>
          ${robotDisplayName(robot)} · ${label(robot.robot_type)}
        </option>
      `,
      )
      .join("")}
  `;
}

function renderRobotStateOptions(robot) {
  const states = robotType(robot) === "PICKY" ? PICKY_STATES : COBOT_STATES;

  return renderOptions(
    states,
    robotStateValue(robot),
    "세부 상태 없음",
    robotStateTextLabel,
  );
}

function renderTaskOptions(selectedTaskId) {
  const tasks = latestAdminStatus?.tasks || [];
  const emptySelected = selectedTaskId === null || selectedTaskId === undefined;

  return `
    <option value="" ${emptySelected ? "selected" : ""}>작업 없음</option>
    ${tasks
      .map(
        (task) => `
        <option value="${task.task_id}" ${sameId(task.task_id, selectedTaskId) ? "selected" : ""}>
          #${task.task_id} ${taskDisplayTitle(task)} · ${taskTargetLabel(task)}
        </option>
      `,
      )
      .join("")}
  `;
}

function renderPickupSlotOptions(selectedSlotId) {
  const slots = latestAdminStatus?.pickup_slots || [];
  const emptySelected = selectedSlotId === null || selectedSlotId === undefined;

  return `
    <option value="" ${emptySelected ? "selected" : ""}>배정 전</option>
    ${slots
      .map(
        (slot) => `
        <option value="${slot.slot_id}" ${sameId(slot.slot_id, selectedSlotId) ? "selected" : ""}>
          ${formatSlotName(slot.slot_name)} · ${label(slot.status)}
        </option>
      `,
      )
      .join("")}
  `;
}

function renderZoneOptions(zones, selectedZoneId, emptyLabel = "미지정") {
  const emptySelected =
    selectedZoneId === null ||
    selectedZoneId === undefined ||
    selectedZoneId === "";

  return `
    <option value="" ${emptySelected ? "selected" : ""}>${emptyLabel}</option>
    ${zones
      .map(
        (zone) => `
        <option value="${zone.zone_id}" ${sameId(zone.zone_id, selectedZoneId) ? "selected" : ""}>
          ${zone.zone_name} · ${label(zone.zone_type)}
        </option>
      `,
      )
      .join("")}
  `;
}

// =====================================
// Order/display work helpers
// =====================================

function isDisplayWork(work) {
  return work?.work_kind === "DISPLAY" || Boolean(work?.display_item_id);
}

function displayWorkNumber(displayItem) {
  return `진열 #${displayItem?.display_item_id ?? "-"}`;
}

function orderWorkNumber(order) {
  return `주문 #${order?.order_id ?? "-"}`;
}

function taskOrderNumber(task) {
  if (task?.order_id) {
    return `주문 #${task.order_id}`;
  }

  if (task?.order_no) {
    const match = String(task.order_no).match(/(\d+)$/);
    if (match) {
      return `주문 #${Number(match[1])}`;
    }
    return task.order_no;
  }

  return null;
}

function displayWorkQuantity(displayItem) {
  return (
    displayItem?.requested_quantity ??
    displayItem?.processed_quantity ??
    displayItem?.stock_delta ??
    0
  );
}

function displayWorkItem(displayItem) {
  const quantity = displayWorkQuantity(displayItem);

  return {
    ...displayItem,
    work_kind: "DISPLAY",
    work_key: `display:${displayItem.display_item_id}`,
    order_no: displayWorkNumber(displayItem),
    priority: 1,
    items: [
      {
        item_id: `display-${displayItem.display_item_id}`,
        product_id: displayItem.product_id,
        product_name: displayItem.product_name,
        image_url: displayItem.image_url,
        quantity,
        status: displayItem.status,
      },
    ],
  };
}

function orderWorkItem(order) {
  return {
    ...order,
    work_kind: "ORDER",
    work_key: `order:${order.order_id}`,
  };
}

function workKey(work) {
  if (!work) {
    return null;
  }

  return isDisplayWork(work)
    ? `display:${work.display_item_id}`
    : `order:${work.order_id}`;
}

function activeWorkItems(data = latestAdminStatus) {
  const orderWorks = (data?.orders || []).map(orderWorkItem);
  const displayWorks = (data?.display_items || []).map(displayWorkItem);

  return [...displayWorks, ...orderWorks].sort(sortWorkItems);
}

function historyWorkItems(data = latestAdminStatus) {
  const orderWorks = (data?.order_history || []).map(orderWorkItem);
  const displayWorks = (data?.display_item_history || []).map(displayWorkItem);

  return [...displayWorks, ...orderWorks].sort(sortWorkItems);
}

function historyWorkGroups(data = latestAdminStatus) {
  return {
    orders: (data?.order_history || [])
      .map(orderWorkItem)
      .sort(sortWorkItemsNewestFirst),
    displays: (data?.display_item_history || [])
      .map(displayWorkItem)
      .sort(sortWorkItemsNewestFirst),
  };
}

function sortWorkItems(a, b) {
  const aIsDisplay = isDisplayWork(a);
  const bIsDisplay = isDisplayWork(b);

  if (aIsDisplay !== bIsDisplay) {
    return aIsDisplay ? -1 : 1;
  }

  const idDiff = Number(workId(a)) - Number(workId(b));
  if (idDiff !== 0) {
    return idDiff;
  }

  const aTaskId = latestTaskIdForWork(a);
  const bTaskId = latestTaskIdForWork(b);

  if (aTaskId !== bTaskId) {
    return bTaskId - aTaskId;
  }

  return 0;
}

function sortWorkItemsNewestFirst(a, b) {
  const idDiff = Number(workId(b)) - Number(workId(a));
  if (idDiff !== 0) {
    return idDiff;
  }

  const aTaskId = latestTaskIdForWork(a);
  const bTaskId = latestTaskIdForWork(b);

  return bTaskId - aTaskId;
}

function latestTaskIdForWork(work) {
  return Math.max(0, ...orderTasks(work).map((task) => Number(task.task_id) || 0));
}

function workId(work) {
  return isDisplayWork(work) ? work.display_item_id : work.order_id;
}

function workDisplayTitle(work) {
  return isDisplayWork(work) ? displayWorkNumber(work) : orderWorkNumber(work);
}

function workKindLabel(work) {
  return isDisplayWork(work) ? "진열" : "주문";
}

function findDisplayWork(displayItemId, { includeHistory = false } = {}) {
  const source = includeHistory
    ? [...(latestAdminStatus?.display_items || []), ...(latestAdminStatus?.display_item_history || [])]
    : latestAdminStatus?.display_items || [];
  const displayItem = source.find((item) =>
    sameId(item.display_item_id, displayItemId),
  );

  return displayItem ? displayWorkItem(displayItem) : null;
}

function findWorkByKey(key, { includeHistory = false } = {}) {
  const source = includeHistory
    ? [...activeWorkItems(), ...historyWorkItems()]
    : activeWorkItems();

  return source.find((work) => work.work_key === key) || null;
}

function findWorkByTask(task, { includeHistory = false } = {}) {
  if (!task) {
    return null;
  }

  if (task.display_item_id) {
    return findDisplayWork(task.display_item_id, { includeHistory });
  }

  if (task.order_id) {
    const order = findOrder(task.order_id, { includeHistory });
    return order ? orderWorkItem(order) : null;
  }

  return null;
}

function selectedWork() {
  if (selectedDisplayItemId !== null) {
    return findDisplayWork(selectedDisplayItemId);
  }

  if (selectedOrderId !== null) {
    const order = findOrder(selectedOrderId);
    return order ? orderWorkItem(order) : null;
  }

  return null;
}

function selectWork(work) {
  if (!work) {
    selectedOrderId = null;
    selectedDisplayItemId = null;
    return;
  }

  if (isDisplayWork(work)) {
    selectedDisplayItemId = Number(work.display_item_id);
    selectedOrderId = null;
    return;
  }

  selectedOrderId = Number(work.order_id);
  selectedDisplayItemId = null;
}

// =====================================
// Work detail renderers
// =====================================

function renderOrderDetail(order) {
  if (isDisplayWork(order)) {
    return `
      <div class="modal-summary">
        <div>
          <span>진열번호</span>
          <strong>${displayWorkNumber(order)}</strong>
        </div>
        <div>
          <span>상태</span>
          <strong>${label(order.status)}</strong>
        </div>
        <div>
          <span>정책</span>
          <strong>${label(order.display_policy)}</strong>
        </div>
      </div>
      ${renderOrderItems(order.items)}
      ${renderOrderTasks(order)}
    `;
  }

  return `
    <div class="modal-summary">
      <div>
        <span>주문번호</span>
        <strong>${order.order_no}</strong>
      </div>
      <div>
        <span>상태</span>
        <strong>${label(order.status)}</strong>
      </div>
      <div>
        <span>픽업 칸</span>
        <strong>${formatPickupSlot(order.pickup_slot_name)}</strong>
      </div>
    </div>
    ${renderOrderItemQuantityEditor(order)}
    ${renderOrderTasks(order)}
    <div class="state-editor-form">
      <div>
        <label for="order-status-select">주문 상태</label>
        <select id="order-status-select">${renderOptions(ORDER_STATUSES, order.status)}</select>
      </div>
      <div>
        <label for="order-pickup-slot-select">픽업 칸</label>
        <select id="order-pickup-slot-select">${renderPickupSlotOptions(order.pickup_slot_id)}</select>
      </div>
      <button class="small-action-button" type="button" data-save-order-state="${order.order_id}">상태 저장</button>
    </div>
  `;
}

function renderOrderTasks(order) {
  const tasks = orderTasks(order);

  if (tasks.length === 0) {
    return '<div class="empty-state">연결된 작업이 없습니다</div>';
  }

  return `
    <div class="modal-subsection">
      <h3>연결된 작업</h3>
      <div class="task-queue-list">
        ${tasks
          .map(
            (task) => `
            <button class="task-queue-row data-button" type="button" data-task-detail="${task.task_id}">
              <div class="queue-rank">#${task.task_id}</div>
              <div class="task-main">
                <div class="task-title-line">
                  <strong>${taskDisplayTitle(task)}</strong>
                  <span>${assignedRobotLabel(task)}</span>
                </div>
                <span>${task.result_message || "결과 메시지 없음"}</span>
              </div>
              <div class="task-side">
                <div class="state-badge ${statusClass(task.status)}">${label(task.status)}</div>
              </div>
            </button>
          `,
          )
          .join("")}
      </div>
    </div>
  `;
}

function findOrder(orderId, { includeHistory = false } = {}) {
  const source = includeHistory
    ? [...(latestAdminStatus?.orders || []), ...(latestAdminStatus?.order_history || [])]
    : latestAdminStatus?.orders || [];

  return source.find((order) =>
    sameId(order.order_id, orderId),
  );
}

function findTask(taskId) {
  return (latestAdminStatus?.tasks || []).find((task) =>
    sameId(task.task_id, taskId),
  );
}

function orderTasks(order) {
  if (!order) {
    return [];
  }

  return (latestAdminStatus?.tasks || [])
    .filter((task) => {
      if (isDisplayWork(order)) {
        return sameId(task.display_item_id, order.display_item_id);
      }

      return (
        task.order_id === order.order_id || task.order_no === order.order_no
      );
    })
    .sort(
      (a, b) =>
        (a.sequence_no ?? taskTypeOrder(a.task_type)) -
          (b.sequence_no ?? taskTypeOrder(b.task_type)) ||
        taskTypeOrder(a.task_type) - taskTypeOrder(b.task_type) ||
        a.task_id - b.task_id,
    );
}

function taskTypeOrder(taskType) {
  const index = TASK_TYPE_SEQUENCE.indexOf(taskType);
  return index === -1 ? TASK_TYPE_SEQUENCE.length : index;
}

function taskQueueForOrder(order) {
  const statusOrder = {
    RUNNING: 0,
    PAUSED: 1,
    ASSIGNED: 2,
    QUEUED: 3,
    SUCCESS: 4,
    FAILED: 5,
    CANCELLED: 6,
  };

  return orderTasks(order).sort(
    (a, b) =>
      (statusOrder[a.status] ?? 99) - (statusOrder[b.status] ?? 99) ||
      (a.sequence_no ?? taskTypeOrder(a.task_type)) -
        (b.sequence_no ?? taskTypeOrder(b.task_type)) ||
      taskTypeOrder(a.task_type) - taskTypeOrder(b.task_type) ||
      a.task_id - b.task_id,
  );
}

function workTaskStatusLabel(status) {
  if (status === "SUCCESS") {
    return "성공";
  }

  if (status === "RUNNING") {
    return "수행 중";
  }

  if (["QUEUED", "ASSIGNED"].includes(status)) {
    return "대기 중";
  }

  return label(status);
}

function orderProductSummaryMarkup(order) {
  if (isDisplayWork(order)) {
    const product = (latestAdminStatus?.products || []).find(
      (candidate) => candidate.product_id === order.product_id,
    ) || {
      product_id: order.product_id,
      name: order.product_name,
      image_url: order.image_url,
    };

    return `
      <span class="order-product-cell">
        ${productImageMarkup(product, "order-product-image")}
        <strong>${orderProductSummary(order)}</strong>
      </span>
    `;
  }

  if (!order.items || order.items.length === 0) {
    return '<span class="muted">상품 없음</span>';
  }

  const firstItem = order.items[0];
  const product = (latestAdminStatus?.products || []).find(
    (candidate) => candidate.product_id === firstItem.product_id,
  ) || {
    product_id: firstItem.product_id,
    name: firstItem.product_name,
    image_url: firstItem.image_url,
  };

  return `
    <span class="order-product-cell">
      ${productImageMarkup(product, "order-product-image")}
      <strong>${orderProductSummary(order)}</strong>
    </span>
  `;
}

function normalizeOrderWorkSelection(data) {
  if (adminPage !== "orders") {
    return;
  }

  const works = activeWorkItems(data);
  const tasks = data.tasks || [];

  if (
    selectedTaskId !== null &&
    !tasks.some((task) => sameId(task.task_id, selectedTaskId))
  ) {
    selectedTaskId = null;
  }

  if (
    selectedOrderId !== null &&
    !(data.orders || []).some((order) => sameId(order.order_id, selectedOrderId))
  ) {
    selectedOrderId = null;
  }

  if (
    selectedDisplayItemId !== null &&
    !(data.display_items || []).some((item) =>
      sameId(item.display_item_id, selectedDisplayItemId),
    )
  ) {
    selectedDisplayItemId = null;
  }

  if (selectedOrderId === null && selectedDisplayItemId === null && works.length > 0) {
    selectWork(works[0]);
  }
}

function orderItemCard(item) {
  const product = (latestAdminStatus?.products || []).find(
    (candidate) => candidate.product_id === item.product_id,
  ) || {
    product_id: item.product_id,
    name: item.product_name,
    image_url: item.image_url,
  };

  return `
    <div class="work-detail-item">
      ${productImageMarkup(product, "work-detail-image")}
      <div>
        <strong>${item.product_name}</strong>
        <span>${item.quantity}개 · ${label(item.status)}</span>
      </div>
    </div>
  `;
}

function renderOrderWorkDetail() {
  if (!orderWorkDetailPanel || !latestAdminStatus) {
    return;
  }

  const selectedTask =
    selectedTaskId === null ? null : findTask(selectedTaskId);
  const order = selectedTask ? findWorkByTask(selectedTask) : selectedWork();

  if (!order) {
    orderWorkDetailPanel.innerHTML =
      '<div class="empty-state">선택된 작업 요청이 없습니다</div>';
    return;
  }

  const tasks = taskQueueForOrder(order);
  const displayWork = isDisplayWork(order);
  const activeTask =
    selectedTask &&
    (
      (displayWork && sameId(selectedTask.display_item_id, order.display_item_id)) ||
      (!displayWork && (selectedTask.order_id === order.order_id ||
        selectedTask.order_no === order.order_no))
    )
      ? selectedTask
      : tasks.find((task) => ["RUNNING", "ASSIGNED"].includes(task.status)) ||
        tasks[0] ||
        null;

  orderWorkDetailPanel.innerHTML = `
    <div class="work-detail-header">
      <div class="work-detail-title">
        <span>${workKindLabel(order)} 상세</span>
        <strong>${workDisplayTitle(order)}</strong>
        ${activeTask ? `<small>현재 작업 · #${activeTask.task_id} ${taskDisplayTitle(activeTask)}</small>` : ""}
      </div>
      <div class="work-detail-header-actions">
        ${
          displayWork
            ? ""
            : `<button class="ghost-button work-detail-edit-button" type="button" data-open-order-modal="${order.order_id}">주문 수정</button>`
        }
      </div>
    </div>
    <div class="work-detail-grid">
      <div class="work-detail-block work-detail-info-block">
        <h3>${workKindLabel(order)} 정보</h3>
        <div class="work-detail-info-content">
          <dl class="work-detail-info-list">
            ${
              displayWork
                ? `
                  <div><dt>진열번호</dt><dd>${displayWorkNumber(order)}</dd></div>
                  <div><dt>진열 상태</dt><dd>${label(order.status)}</dd></div>
                  <div><dt>진열 정책</dt><dd>${label(order.display_policy)}</dd></div>
                  <div><dt>요청 수량</dt><dd>${order.requested_quantity ?? "전체 처리"}개</dd></div>
                  <div><dt>처리 수량</dt><dd>${order.processed_quantity ?? "-"}개</dd></div>
                `
                : `
                  <div><dt>주문번호</dt><dd>${order.order_no}</dd></div>
                  <div><dt>주문 상태</dt><dd>${label(order.status)}</dd></div>
                  <div><dt>픽업칸</dt><dd>${formatPickupSlot(order.pickup_slot_name)}</dd></div>
                `
            }
          </dl>
          <div class="work-detail-info-side">
            ${renderMiniProgress(workProgress(order), order.status)}
            <span class="state-badge ${statusClass(activeTask?.status || order.status)}">${label(activeTask?.status || order.status)}</span>
          </div>
        </div>
      </div>
      <div class="work-detail-block work-detail-items-block">
        <h3>${displayWork ? "진열 상품" : "상품"}</h3>
        <div class="work-detail-items">
          ${order.items.length === 0 ? '<span class="muted">상품 없음</span>' : order.items.map(orderItemCard).join("")}
        </div>
      </div>
      <div class="work-detail-block work-detail-tasks-block">
        <h3>Fleet 작업</h3>
        <div class="work-detail-task-list">
          ${
            tasks.length === 0
              ? '<span class="muted">Fleet Manager가 생성한 작업 없음</span>'
              : tasks
                  .map(
                    (task) => `
              <button class="work-detail-task ${sameId(task.task_id, activeTask?.task_id) ? "is-selected" : ""}" type="button" data-work-task="${task.task_id}">
                <span>#${task.task_id}</span>
                <strong>${taskDisplayTitle(task)}</strong>
                <em>${assignedRobotLabel(task)}</em>
                <i class="state-badge ${statusClass(task.status)}">${workTaskStatusLabel(task.status)}</i>
              </button>
            `,
                  )
                  .join("")
          }
        </div>
      </div>
    </div>
  `;
}

function renderEmpty(target, text) {
  if (!target) {
    return;
  }

  target.innerHTML = `<div class="empty-state">${text}</div>`;
}

// =====================================
// Dashboard/page renderers
// =====================================

function findRobotTask(robot) {
  const tasks = getRobotTasks(robot.robot_id);

  return (
    tasks.find((task) => sameId(task.task_id, robot.current_task_id)) ||
    tasks.find((task) => task.status === "RUNNING") ||
    null
  );
}

function getRobotTasks(robotId) {
  const tasks = latestAdminStatus?.tasks || [];
  const statusOrder = {
    RUNNING: 1,
    ASSIGNED: 2,
    QUEUED: 3,
    PAUSED: 4,
    SUCCESS: 5,
    FAILED: 6,
    CANCELLED: 7,
  };

  return tasks
    .filter((task) => sameId(task.assigned_robot_id, robotId))
    .sort(
      (a, b) =>
        (statusOrder[a.status] || 99) - (statusOrder[b.status] || 99) ||
        (a.sequence_no ?? 999) - (b.sequence_no ?? 999) ||
        a.task_id - b.task_id,
    );
}

function statusClass(status) {
  return `status-${String(status || "")
    .toLowerCase()
    .replaceAll("_", "-")}`;
}

function batteryClass(level) {
  if (level === null) {
    return "battery-plugged";
  }

  if (level <= 20) {
    return "battery-low";
  }

  if (level <= 50) {
    return "battery-medium";
  }

  return "battery-high";
}

function robotCategory(robot) {
  const status = robotStatusValue(robot);

  if (["ERROR", "EMERGENCY_STOP", "OFFLINE"].includes(status)) {
    return "error";
  }

  if (["IDLE", "CHARGING"].includes(status)) {
    return "idle";
  }

  return "working";
}

function robotLocationText(robot) {
  const hasPosition = robot.pos_x !== null && robot.pos_y !== null;

  if (!hasPosition) {
    return "-";
  }

  const x = Number(robot.pos_x).toFixed(1);
  const y = Number(robot.pos_y).toFixed(1);
  const theta =
    robot.pos_theta === null ? null : Number(robot.pos_theta).toFixed(1);

  return theta === null ? `(${x}, ${y})` : `(${x}, ${y}, ${theta})`;
}

function renderBatteryMeter(level) {
  if (level === null || level === undefined) {
    return `
      <div class="battery-meter plugged">
        <span>전원 연결</span>
        <div class="meter-track"><div class="meter-fill" style="width: 100%"></div></div>
      </div>
    `;
  }

  return `
    <div class="battery-meter ${batteryClass(level)}">
      <span>${level}%</span>
      <div class="meter-track"><div class="meter-fill" style="width: ${level}%"></div></div>
    </div>
  `;
}

function clampNumber(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function updateDashboardMapViewport() {
  if (!dashboardMap) {
    return;
  }

  const rect = dashboardMap.getBoundingClientRect();
  if (!rect.width || !rect.height) {
    return;
  }

  const maxWidth = Math.max(0, rect.width - MAP_LAYER_PADDING_PX);
  const maxHeight = Math.max(0, rect.height - MAP_LAYER_PADDING_PX);
  let height = maxHeight;
  let width = height * MAP_ASPECT_RATIO;

  if (width > maxWidth) {
    width = maxWidth;
    height = width / MAP_ASPECT_RATIO;
  }

  dashboardMap.style.setProperty("--map-visual-width", `${width}px`);
  dashboardMap.style.setProperty("--map-visual-height", `${height}px`);
}

function mapPosePosition(pose) {
  if (
    !pose ||
    pose.x === null ||
    pose.x === undefined ||
    pose.y === null ||
    pose.y === undefined
  ) {
    return null;
  }

  const x = (Number(pose.x) / MAP_WIDTH_METERS) * 100;
  const y = 100 - (Number(pose.y) / MAP_HEIGHT_METERS) * 100;

  return {
    x: clampNumber(x, 0, 100),
    y: clampNumber(y, 0, 100),
  };
}

function zoneMapByName() {
  if (!zoneOptionsCache) {
    return null;
  }

  return new Map(
    zoneOptionsCache.map((zone) => [zone.zone_name, zone]),
  );
}

function ensureMapZonesLoaded() {
  if (zoneOptionsCache || mapZonesLoading) {
    return;
  }

  mapZonesLoading = true;
  loadZoneOptions()
    .then(() => {
      mapZonesLoading = false;
      renderMapRobots(latestAdminStatus?.robots || []);
    })
    .catch(() => {
      mapZonesLoading = false;
    });
}

function currentMoveTask(robot) {
  const task = robot?.current_task;
  const taskType = robot?.current_task_type || task?.task_type;
  const taskStatus = robot?.current_task_status || task?.status;

  if (taskStatus !== "RUNNING" || !MAP_MOVING_TASK_TYPES.has(taskType)) {
    return null;
  }

  return task || {
    task_type: taskType,
    source_zone_name: null,
    target_zone_name: null,
  };
}

function plannedWaypointNamesForRobot(robot) {
  const waypoints =
    robot?.planned_waypoints || robot?.current_task?.planned_waypoints;
  if (!Array.isArray(waypoints)) {
    return [];
  }

  return waypoints.filter(Boolean).map(String);
}

function mapRouteForRobot(robot, zonesByName) {
  const task = currentMoveTask(robot);
  if (!task || !zonesByName) {
    return null;
  }

  const route = plannedWaypointNamesForRobot(robot);
  if (route.length < 2) {
    return null;
  }

  const points = route
    .map((zoneName) => mapPosePosition(zonesByName.get(zoneName)?.pose))
    .filter(Boolean);

  return points.length >= 2 ? points : null;
}

function renderMapRoute(robot, zonesByName) {
  const points = mapRouteForRobot(robot, zonesByName);
  if (!points) {
    return "";
  }

  const pointText = points
    .map((point) => `${point.x.toFixed(2)},${point.y.toFixed(2)}`)
    .join(" ");
  const waypointDots = points
    .slice(1, -1)
    .map(
      (point) => `
        <circle class="map-route-waypoint" cx="${point.x.toFixed(2)}" cy="${point.y.toFixed(2)}" r="1.1"></circle>
      `,
    )
    .join("");

  return `
    <svg class="map-route-overlay ${robotColorClass(robot.robot_id)}" viewBox="0 0 100 100" preserveAspectRatio="none" aria-hidden="true">
      <polyline class="map-route-line" points="${pointText}"></polyline>
      ${waypointDots}
      <circle class="map-route-target" cx="${points[points.length - 1].x.toFixed(2)}" cy="${points[points.length - 1].y.toFixed(2)}" r="1.5"></circle>
    </svg>
  `;
}

function mapRobotPosition(robot) {
  if (robot.pos_x !== null && robot.pos_y !== null) {
    const x = (Number(robot.pos_x) / MAP_WIDTH_METERS) * 100;
    const y = 100 - (Number(robot.pos_y) / MAP_HEIGHT_METERS) * 100;

    return {
      x: clampNumber(x, 4, 96),
      y: clampNumber(y, 4, 96),
    };
  }

  if (robot.robot_name === "PICKY1") {
    return { x: 50, y: 24 };
  }

  if (robot.robot_name === "PICKY2") {
    return { x: 50, y: 78 };
  }

  return { x: 50, y: 50 };
}

function robotHeadingDeg(robot) {
  if (robot.pos_theta === null || robot.pos_theta === undefined) {
    return 0;
  }

  return (Number(robot.pos_theta) * 180) / Math.PI;
}

function renderMapRobots(robots) {
  if (!mapRobotLayer) {
    return;
  }

  updateDashboardMapViewport();
  ensureMapZonesLoaded();

  const pickyRobots = robots.filter(
    (robot) =>
      robotType(robot) === "PICKY" &&
      !normalizeId(robot.robot_name).startsWith("UI_"),
  );

  const now = Date.now();
  const zonesByName = zoneMapByName();
  const routesHtml = pickyRobots
    .map((robot) => renderMapRoute(robot, zonesByName))
    .join("");
  const markersHtml = pickyRobots
    .map((robot) => {
      const position = mapRobotPosition(robot);
      const markerClass = "map-marker-amr";
      const moving = isRobotMovingOnMap(robot);
      const movingClass = moving ? "is-moving" : "";
      const arrivedClass = shouldFlashMapArrival(robot, moving, now) ? "is-arrived" : "";
      const displayName = robotDisplayName(robot);
      const status = robotStatusValue(robot);

      return `
        <div class="robot-map-marker ${markerClass} ${robotColorClass(robot.robot_id)} ${movingClass} ${arrivedClass}"
          style="--marker-x: ${position.x}%; --marker-y: ${position.y}%; --heading: ${robotHeadingDeg(robot)}deg"
          title="${displayName} · ${label(status)}">
          <i class="marker-heading"></i>
          <span>${displayName}</span>
        </div>
      `;
    })
    .join("");

  mapRobotLayer.innerHTML = `${routesHtml}${markersHtml}`;
}

function taskProgress(status) {
  const progressByStatus = {
    QUEUED: 8,
    ASSIGNED: 22,
    RUNNING: 72,
    PAUSED: 42,
    SUCCESS: 100,
    FAILED: 100,
    CANCELLED: 100,
  };

  return progressByStatus[status] || 0;
}

function orderProgress(status) {
  const progressByStatus = {
    ORDER_RECEIVED: 10,
    ORDER_WAIT: 15,
    SORTING: 35,
    DELIVERING: 55,
    INSPECTING: 75,
    PICKUP_READY: 100,
    COMPLETED: 100,
    ERROR: 100,
  };

  return progressByStatus[status] || 0;
}

function displayProgress(status) {
  const progressByStatus = {
    REQUESTED: 15,
    ASSIGNED: 25,
    IN_PROGRESS: 65,
    COMPLETED: 100,
    FAILED: 100,
    CANCELLED: 100,
  };

  return progressByStatus[status] || 0;
}

function workProgress(work) {
  return isDisplayWork(work)
    ? displayProgress(work.status)
    : orderProgress(work.status);
}

function orderProductSummary(order) {
  if (!order.items || order.items.length === 0) {
    return "상품 없음";
  }

  const firstItem = order.items[0];

  if (order.items.length === 1) {
    return `${firstItem.product_name || firstItem.name} ${firstItem.quantity}개`;
  }

  return `${firstItem.product_name || firstItem.name} 외 ${order.items.length - 1}종`;
}

function stockLevel(product) {
  if (STOCK_LEVELS.has(product.stock_level)) {
    return product.stock_level;
  }

  const stockQty = Number(product.stock_qty);

  if (stockQty <= STOCK_LOW_MAX) {
    return "low";
  }

  if (stockQty === STOCK_WARNING_QTY) {
    return "warning";
  }

  if (stockQty >= STOCK_NORMAL_MIN) {
    return "normal";
  }

  return "low";
}

function stockLevelLabel(level) {
  return STOCK_LEVEL_LABELS[level] || STOCK_LEVEL_LABELS.normal;
}

function stockLevelClass(level) {
  return STOCK_LEVEL_CLASSES[level] || STOCK_LEVEL_CLASSES.normal;
}

function countStockLevels(products) {
  return products.reduce(
    (counts, product) => {
      counts[stockLevel(product)] += 1;
      return counts;
    },
    { low: 0, warning: 0, normal: 0 },
  );
}

function sortedExceptions(exceptions) {
  return [...exceptions].sort(
    (a, b) => new Date(b.created_at) - new Date(a.created_at),
  );
}

function allExceptionsFromStatus(data) {
  return sortedExceptions([
    ...(data.exceptions || []),
    ...(data.exception_history || []),
  ]);
}

function findExceptionById(exceptionId) {
  if (!latestAdminStatus) {
    return null;
  }

  return allExceptionsFromStatus(latestAdminStatus).find((exception) =>
    sameId(exception.exception_id, exceptionId),
  );
}

function renderExceptionDetail(exception) {
  const robotLabel = exception.robot_name || exception.robot_id || "-";
  const taskLabel = exception.task_id ? `#${exception.task_id}` : "-";
  const orderLabel = exception.order_id ? `#${exception.order_id}` : "-";
  const detail = exception.detail || "상세 메시지가 없습니다.";

  return `
    <div class="modal-summary exception-detail-summary">
      <div>
        <span>상태</span>
        <strong>${exception.is_resolved ? "처리 완료" : "미조치"}</strong>
      </div>
      <div>
        <span>로봇</span>
        <strong>${escapeHtml(robotLabel)}</strong>
      </div>
      <div>
        <span>발생 시간</span>
        <strong>${formatDateTime(exception.created_at)}</strong>
      </div>
      <div>
        <span>Task</span>
        <strong>${escapeHtml(taskLabel)}</strong>
      </div>
      <div>
        <span>Order</span>
        <strong>${escapeHtml(orderLabel)}</strong>
      </div>
      <div>
        <span>예외 타입</span>
        <strong>${escapeHtml(exception.exception_type || "-")}</strong>
      </div>
    </div>
    <div class="modal-subsection">
      <h3>상세 메시지</h3>
      <pre class="exception-detail-message">${escapeHtml(detail)}</pre>
    </div>
  `;
}

function exceptionDetailHeaderActions(exception) {
  if (exception.is_resolved) {
    return "";
  }

  return `
    <button class="small-action-button" type="button" data-resolve-exception-detail="${exception.exception_id}">
      미조치
    </button>
  `;
}

function openExceptionDetail(exceptionId) {
  const exception = findExceptionById(exceptionId);

  if (!exception) {
    alert("예외 정보를 찾을 수 없습니다.");
    return;
  }

  openModal(`예외/알람 #${exception.exception_id}`, renderExceptionDetail(exception));
  setModalHeaderActions(exceptionDetailHeaderActions(exception));
}

function handleExceptionDetailKeydown(event) {
  if (!["Enter", " "].includes(event.key)) {
    return;
  }

  const row = event.target.closest("[data-open-exception-modal]");
  if (!row) {
    return;
  }

  event.preventDefault();
  openExceptionDetail(row.dataset.openExceptionModal);
}

async function resolveException(exceptionId, button = null) {
  if (button) {
    button.disabled = true;
  }

  try {
    await postAdminAction(`/api/admin/exceptions/${exceptionId}/resolve`);
  } finally {
    if (button) {
      button.disabled = false;
    }
  }
}

function renderMiniProgress(value, status) {
  const progress = Math.max(0, Math.min(100, Number(value) || 0));

  return `
    <div class="mini-progress ${statusClass(status)}" style="--progress: ${progress}%"><span>${progress}%</span></div>
  `;
}

function robotColorClass(robotId) {
  const robot = findRobotById(robotId);
  const robotName = robot?.robot_name || normalizeId(robotId);

  if (
    robot?.unit_id === 1 ||
    robotName === "PICKY1" ||
    robotName === "COBOT1"
  ) {
    return "robot-dot-amr1";
  }

  if (
    robot?.unit_id === 2 ||
    robotName === "PICKY2" ||
    robotName === "COBOT2"
  ) {
    return "robot-dot-amr2";
  }

  return "robot-dot-neutral";
}

function robotDisplayName(robotOrId) {
  const robot = findRobotById(robotOrId);
  const robotId = robot?.robot_name || normalizeId(robotOrId);

  return ROBOT_DISPLAY_NAMES[robotId] || robotId || "-";
}

function robotType(robot) {
  return robot?.robot_type || "COBOT";
}

function robotTypeOrder(robot) {
  return robotType(robot) === "PICKY" ? 0 : 1;
}

function sortedRobots(robots) {
  return [...(robots || [])].sort(
    (a, b) =>
      (a.unit_id ?? 999) - (b.unit_id ?? 999) ||
      robotTypeOrder(a) - robotTypeOrder(b) ||
      normalizeId(a.robot_name).localeCompare(normalizeId(b.robot_name)) ||
      Number(a.robot_id || 0) - Number(b.robot_id || 0),
  );
}

function robotImageUrl(robot) {
  return robotType(robot) === "PICKY"
    ? "/static/img/pinky.png"
    : "/static/img/jetcobot.png";
}

function hydrateRobotFilters() {
  if (robotStatusFilter) {
    robotStatusFilter.innerHTML = renderOptions(
      ROBOT_STATUSES,
      robotStatusFilter.value,
      "상태 전체",
    );
  }

  if (robotTypeFilter) {
    robotTypeFilter.innerHTML = renderOptions(
      ROBOT_TYPES,
      robotTypeFilter.value,
      "유형 전체",
    );
  }
}

function robotFilterValue(element) {
  return element?.value || "";
}

function filterRobotsForManagement(robots) {
  const search = (robotSearchInput?.value || "").trim().toLowerCase();
  const statusFilter = robotFilterValue(robotStatusFilter);
  const type = robotFilterValue(robotTypeFilter);

  return robots.filter((robot) => {
    const task = findRobotTask(robot);
    const status = robotStatusValue(robot);
    const searchable = [
      robot.robot_name,
      `#${robot.robot_id}`,
      robotDisplayName(robot),
      robotType(robot),
      label(status),
      robotStateLabel(robot),
      task?.order_no,
      task ? taskDisplayTitle(task) : "",
    ]
      .filter(Boolean)
      .join(" ")
      .toLowerCase();

    return (
      (!search || searchable.includes(search)) &&
      (!statusFilter || robotStatusValue(robot) === statusFilter) &&
      (!type || robotType(robot) === type)
    );
  });
}

function renderRobotManagementDetail(robot) {
  if (!robotDetailPanel) {
    return;
  }

  if (!robot) {
    robotDetailPanel.innerHTML =
      '<div class="empty-state">선택된 로봇이 없습니다</div>';
    return;
  }

  const task = findRobotTask(robot);
  const type = robotType(robot);
  const imageUrl = robotImageUrl(robot);
  const displayName = robotDisplayName(robot);
  const status = robotStatusValue(robot);
  const currentTask = task
    ? `${taskTargetLabel(task)} · ${taskDisplayTitle(task)}`
    : "작업 없음";

  robotDetailPanel.innerHTML = `
    <div class="robot-detail-visual">
      <div class="robot-detail-title">
        <span class="${robotColorClass(robot.robot_id)}"></span>
        <strong title="#${robot.robot_id}">${displayName}</strong>
        <span class="state-badge ${statusClass(status)}">${label(status)}</span>
      </div>
      <span>${label(type)}</span>
      <img src="${imageUrl}" alt="${displayName} ${type}">
    </div>
    <div class="robot-detail-metrics">
      <div>
        <span>배터리</span>
        <strong>${robot.battery_level === null ? "전원 연결" : `${robot.battery_level}%`}</strong>
      </div>
      <div>
        <span>현재 작업</span>
        <strong>${currentTask}</strong>
      </div>
      <div>
        <span>위치</span>
        <strong>${robotLocationText(robot)}</strong>
      </div>
      <div>
        <span>상태</span>
        <strong>${label(status)}</strong>
      </div>
      <div>
        <span>세부 상태</span>
        <strong>${robotStateLabel(robot)}</strong>
      </div>
    </div>
    <div class="robot-inline-editor">
      <div>
        <label for="robot-panel-status-select">로봇 상태</label>
        <select id="robot-panel-status-select">${renderOptions(ROBOT_STATUSES, status)}</select>
      </div>
      <div>
        <label for="robot-panel-state-select">세부 상태</label>
        <select id="robot-panel-state-select">${renderRobotStateOptions(robot)}</select>
      </div>
      <div>
        <label for="robot-panel-current-task-select">현재 작업</label>
        <select id="robot-panel-current-task-select">${renderTaskOptions(robot.current_task_id)}</select>
      </div>
      <button class="small-action-button" type="button" data-save-robot-panel="${robot.robot_id}">상태 저장</button>
    </div>
  `;
}

function renderRobotManagement(robots) {
  const filteredRobots = filterRobotsForManagement(robots);

  if (
    !selectedRobotId ||
    !robots.some((robot) => sameId(robot.robot_id, selectedRobotId))
  ) {
    selectedRobotId =
      filteredRobots[0]?.robot_id || robots[0]?.robot_id || null;
  }

  const selectedRobot =
    robots.find((robot) => sameId(robot.robot_id, selectedRobotId)) ||
    filteredRobots[0] ||
    null;

  if (selectedRobot) {
    selectedRobotId = selectedRobot.robot_id;
  }

  if (filteredRobots.length === 0) {
    renderEmpty(robotStatus, "조건에 맞는 로봇이 없습니다");
    renderRobotManagementDetail(selectedRobot);
    return;
  }

  robotStatus.innerHTML = `
    <div class="admin-table robot-management-table">
      <div class="admin-table-head">
        <span>로봇</span>
        <span>유형</span>
        <span>상태</span>
        <span>세부 상태</span>
        <span>배터리</span>
        <span>현재 작업</span>
        <span>위치</span>
      </div>
      ${filteredRobots
        .map((robot) => {
          const task = findRobotTask(robot);
          const isSelected = sameId(robot.robot_id, selectedRobotId);
          const displayName = robotDisplayName(robot);
          const status = robotStatusValue(robot);

          return `
            <div class="admin-table-row robot-management-row ${isSelected ? "selected" : ""}" data-robot-select="${robot.robot_id}">
              <span class="robot-name-cell" title="#${robot.robot_id}"><i class="${robotColorClass(robot.robot_id)}"></i><strong>${displayName}</strong></span>
              <span>${label(robotType(robot))}</span>
              <span><span class="state-badge ${statusClass(status)}">${label(status)}</span></span>
              <span>${robotStateLabel(robot)}</span>
              <span>${renderBatteryMeter(robot.battery_level)}</span>
              <span class="task-cell">${task ? `${taskTargetLabel(task)} · ${taskDisplayTitle(task)}` : "-"}</span>
              <span class="location-cell">${robotLocationText(robot)}</span>
            </div>
          `;
        })
        .join("")}
    </div>
  `;

  renderRobotManagementDetail(selectedRobot);
}

function renderRobots(robots) {
  if (!robotStatus) {
    return;
  }

  const robotsToRender = sortedRobots(robots);

  if (adminPage === "robots") {
    renderRobotManagement(robotsToRender);
    return;
  }

  if (robotsToRender.length === 0) {
    renderEmpty(robotStatus, "등록된 로봇이 없습니다");
    return;
  }

  robotStatus.innerHTML = `
    <div class="admin-table robot-table">
      <div class="admin-table-head">
        <span>로봇</span>
        <span>상태</span>
        <span>세부 상태</span>
        <span>현재 작업</span>
        <span>배터리</span>
      </div>
      ${robotsToRender
        .map((robot) => {
          const task = findRobotTask(robot);
          const robotTypeClass = robotColorClass(robot.robot_id);
          const displayName = robotDisplayName(robot);
          const status = robotStatusValue(robot);

          return `
            <button class="admin-table-row robot-table-row" type="button" data-robot-detail="${robot.robot_id}">
              <span class="robot-name-cell" title="#${robot.robot_id}"><i class="${robotTypeClass}"></i>${displayName}</span>
              <span><span class="state-badge ${statusClass(status)}">${label(status)}</span></span>
              <span>${robotStateLabel(robot)}</span>
              <span class="task-cell">${task ? `${taskTargetLabel(task)} · ${taskDisplayTitle(task)}` : "-"}</span>
              <span>${renderBatteryMeter(robot.battery_level)}</span>
            </button>
          `;
        })
        .join("")}
    </div>
  `;
}

function renderOrders(orders) {
  if (!orderList) {
    return;
  }

  const works = activeWorkItems(
    latestAdminStatus
      ? { ...latestAdminStatus, orders: orders || latestAdminStatus.orders || [] }
      : { orders: orders || [], display_items: [] },
  );

  if (works.length === 0) {
    renderEmpty(orderList, "진행 중인 주문/진열 요청이 없습니다");
    return;
  }

  orderList.innerHTML = `
    <div class="admin-table order-table">
      <div class="admin-table-head">
        <span>요청번호</span>
        <span>상품</span>
        <span>상태</span>
        <span>대상</span>
        <span>진행률</span>
      </div>
      ${works
        .map((work) => {
          const linkedTaskSelected =
            selectedTaskId !== null &&
            orderTasks(work).some((task) =>
              sameId(task.task_id, selectedTaskId),
            );
          const isSelected =
            adminPage === "orders" &&
            (
              (isDisplayWork(work) && sameId(work.display_item_id, selectedDisplayItemId)) ||
              (!isDisplayWork(work) && sameId(work.order_id, selectedOrderId)) ||
              linkedTaskSelected
            );

          return `
            <button class="admin-table-row order-table-row ${isSelected ? "is-selected" : ""}" type="button" data-work-detail="${work.work_key}">
              <span class="task-cell-stack">
                <strong>${workDisplayTitle(work)}</strong>
                <small>${workKindLabel(work)}</small>
              </span>
              <span>${orderProductSummary(work)}</span>
              <span><span class="state-badge ${statusClass(work.status)}">${label(work.status)}</span></span>
              <span>${isDisplayWork(work) ? "진열대" : formatPickupSlot(work.pickup_slot_name)}</span>
              <span>${renderMiniProgress(workProgress(work), work.status)}</span>
            </button>
          `;
        })
        .join("")}
    </div>
  `;
}

function renderPickupSlots(slots) {
  if (pickupSummaryList) {
    pickupSummaryList.innerHTML =
      slots.length === 0
        ? "-"
        : slots
            .map(
              (slot) => `
            <button class="pickup-summary-item slot-${slot.status.toLowerCase()}" type="button" data-pickup-slot-detail="${slot.slot_id}">
              <strong>${formatSlotName(slot.slot_name)}</strong>
              <span>${label(slot.status)}</span>
            </button>
          `,
            )
            .join("");
  }

  if (!pickupSlotList) {
    return;
  }

  if (slots.length === 0) {
    renderEmpty(pickupSlotList, "픽업 칸이 없습니다");
    return;
  }

  pickupSlotList.innerHTML = slots
    .map(
      (slot) => `
      <button class="data-row data-button slot-row slot-${slot.status.toLowerCase()}" type="button" data-pickup-slot-detail="${slot.slot_id}">
        <div>
          <strong>${formatSlotName(slot.slot_name)}</strong>
          <span>픽업 슬롯</span>
        </div>
        <div class="slot-state">
          <div class="slot-state-dot" title="${label(slot.status)}"></div>
          <span>${label(slot.status)}</span>
        </div>
      </button>
    `,
    )
    .join("");
}

function renderInventory(products) {
  if (!inventoryList) {
    return;
  }

  if (!products || products.length === 0) {
    renderEmpty(inventoryList, "등록된 상품이 없습니다");
    return;
  }

  const sortedProducts = [...products].sort(
    (a, b) => a.stock_qty - b.stock_qty || a.product_id - b.product_id,
  );

  if (adminPage === "dashboard") {
    const stockCounts = countStockLevels(products);
    const totalProducts = products.length || 1;
    const normalEnd = (stockCounts.normal / totalProducts) * 100;
    const warningEnd = normalEnd + (stockCounts.warning / totalProducts) * 100;

    inventoryList.innerHTML = `
      <div class="inventory-donut-summary">
        <div class="inventory-donut" style="background: conic-gradient(#14b8a6 0 ${normalEnd}%, #f59e0b ${normalEnd}% ${warningEnd}%, #ef4444 ${warningEnd}% 100%)">
          <div>
            <span>총</span>
            <strong>${products.length}종</strong>
          </div>
        </div>
        <div class="inventory-legend">
          <span><i class="legend-teal"></i>정상 ${stockCounts.normal}</span>
          <span><i class="legend-yellow"></i>부족 임박 ${stockCounts.warning}</span>
          <span><i class="legend-red"></i>부족 ${stockCounts.low}</span>
        </div>
      </div>
    `;
    return;
  }

  const productsToRender =
    adminPage === "inventory" ? sortedProducts : sortedProducts.slice(0, 5);

  inventoryList.innerHTML = `
    <div class="admin-table inventory-table">
      <div class="admin-table-head">
        <span>품목</span>
        <span>재고</span>
        <span>상태</span>
        <span>위치</span>
      </div>
      ${productsToRender
        .map((product) => {
          const level = stockLevel(product);

          return `
          <button class="admin-table-row inventory-table-row ${level !== "normal" ? "warning-row" : ""}" type="button" data-product-detail="${product.product_id}">
            <span class="inventory-product-cell">
              ${productImageMarkup(product)}
              <strong>${product.name.replace("Test ", "")}</strong>
            </span>
            <span>${product.stock_qty}</span>
            <span class="${stockLevelClass(level)}">${stockLevelLabel(level)}</span>
            <span>${productStorageLabel(product)}</span>
          </button>
        `;
        })
        .join("")}
    </div>
  `;
}

function renderTaskSnapshot(tasks) {
  if (!taskList) {
    return;
  }

  if (adminPage === "dashboard") {
    renderOrderSnapshot(activeWorkItems(latestAdminStatus));
    return;
  }

  if (!tasks || tasks.length === 0) {
    renderEmpty(taskList, "작업이 없습니다");
    return;
  }

  const tasksToRender = tasks
    .filter((task) => ACTIVE_TASK_STATUSES.has(task.status))
    .sort(sortMainTaskSnapshot);

  if (tasksToRender.length === 0) {
    renderEmpty(taskList, "대기 중이거나 수행 중인 작업이 없습니다");
    return;
  }

  const currentTasks = tasksToRender.filter((task) =>
    ["RUNNING", "PAUSED"].includes(task.status),
  );
  const waitingTasks = tasksToRender.filter((task) =>
    ["ASSIGNED", "QUEUED"].includes(task.status),
  );

  taskList.innerHTML = `
    <div class="admin-table task-table active-task-table">
      <div class="admin-table-head">
        <span>작업 ID</span>
        <span>유형</span>
        <span>주문/대상</span>
        <span>로봇</span>
        <span>상태</span>
      </div>
      ${renderTaskTableSection("현재 작업", currentTasks, "진행 중이거나 일시정지된 작업 없음", "current")}
      ${renderTaskTableSection("대기 작업", waitingTasks, "다음 수행 대기 작업 없음", "waiting")}
    </div>
  `;
}

function sortMainTaskSnapshot(a, b) {
  const statusOrder = {
    RUNNING: 0,
    PAUSED: 1,
    ASSIGNED: 2,
    QUEUED: 3,
  };

  return (
    (statusOrder[a.status] ?? 99) - (statusOrder[b.status] ?? 99) ||
    (a.priority ?? 999) - (b.priority ?? 999) ||
    (a.sequence_no ?? 999) - (b.sequence_no ?? 999) ||
    a.task_id - b.task_id
  );
}

function renderTaskTableSection(title, tasks, emptyText, sectionType) {
  return `
    <div class="task-table-section-row task-section-${sectionType}">
      <i aria-hidden="true"></i>
      <strong>${title}</strong>
      <span>${tasks.length}개</span>
    </div>
    ${
      tasks.length === 0
        ? `<div class="task-table-empty-row">${emptyText}</div>`
        : tasks.map(renderMainTaskTableRow).join("")
    }
  `;
}

function renderMainTaskTableRow(task) {
  const rowStateClass = ["RUNNING", "PAUSED"].includes(task.status)
    ? "task-row-current"
    : "task-row-waiting";

  return `
    <button class="admin-table-row task-table-row ${rowStateClass} ${adminPage === "orders" && sameId(task.task_id, selectedTaskId) ? "is-selected" : ""}" type="button" data-task-detail="${task.task_id}">
      <span>#${task.task_id}</span>
      <span class="task-cell-stack">
        <strong>${taskDisplayTitle(task)}</strong>
        <small>${taskRouteLabel(task)}</small>
      </span>
      <span class="task-cell-stack">
        <strong>${taskTargetLabel(task)}</strong>
        <small>${taskReferenceLabel(task)}</small>
      </span>
      <span>${assignedRobotLabel(task)}</span>
      <span><i class="state-badge ${statusClass(task.status)}">${label(task.status)}</i></span>
    </button>
  `;
}

function renderOrderSnapshot(orders) {
  if (!taskList) {
    return;
  }

  const activeOrders = orders
    .filter((order) =>
      isDisplayWork(order)
        ? !FINAL_DISPLAY_ITEM_STATUSES.has(order.status)
        : !["COMPLETED", "ERROR"].includes(order.status),
    )
    .slice(0, 5);

  if (activeOrders.length === 0) {
    renderEmpty(taskList, "진행 중인 주문/진열이 없습니다");
    return;
  }

  taskList.innerHTML = `
    <div class="admin-table dashboard-order-table">
      <div class="admin-table-head">
        <span>요청번호</span>
        <span>상품</span>
        <span>현재 단계</span>
        <span>진행률</span>
      </div>
      ${activeOrders
        .map(
          (order) => `
          <button class="admin-table-row dashboard-order-row" type="button" data-work-detail="${order.work_key}">
            <span class="task-cell-stack">
              <strong>${workDisplayTitle(order)}</strong>
              <small>${workKindLabel(order)}</small>
            </span>
            <span>${orderProductSummaryMarkup(order)}</span>
            <span><span class="state-badge ${statusClass(order.status)}">${label(order.status)}</span></span>
            <span>${renderMiniProgress(workProgress(order), order.status)}</span>
          </button>
        `,
        )
        .join("")}
    </div>
  `;
}

function renderExceptions(exceptions) {
  if (!exceptionList) {
    return;
  }

  if (exceptions.length === 0) {
    renderEmpty(exceptionList, "예외/알람이 없습니다");
    return;
  }

  exceptionList.innerHTML = `
    <div class="admin-table exception-table">
      <div class="admin-table-head">
        <span>시간</span>
        <span>로봇</span>
        <span>내용</span>
        <span>상태</span>
      </div>
      ${exceptions
        .map(
          (exception) => `
          <div class="admin-table-row exception-table-row ${exception.is_resolved ? "" : "danger-row"}" data-open-exception-modal="${exception.exception_id}" role="button" tabindex="0">
            <span>${formatDateTime(exception.created_at)}</span>
            <span>${escapeHtml(exception.robot_name || exception.robot_id || "-")}</span>
            <span class="${exception.is_resolved ? "" : "table-danger"}">${escapeHtml(exception.detail || exception.exception_type)}</span>
            <span>${
              exception.is_resolved
                ? '<span class="state-badge">처리</span>'
                : `<button class="small-action-button" type="button" data-resolve-exception="${exception.exception_id}">미조치</button>`
            }</span>
          </div>
        `,
        )
        .join("")}
    </div>
  `;
}

function exceptionMatches(exception, query) {
  if (!query) {
    return true;
  }

  const haystack = [
    exception.exception_type,
    exception.detail,
    exception.robot_name,
    exception.robot_id,
    exception.task_id ? `task ${exception.task_id}` : "",
    exception.order_id ? `order ${exception.order_id}` : "",
    exception.created_at,
    formatDateTime(exception.created_at),
    exception.is_resolved ? "처리 완료 resolved" : "미처리 unresolved",
  ]
    .join(" ")
    .toLowerCase();

  return haystack.includes(query.toLowerCase());
}

function renderExceptionHistoryList(exceptions, query = "") {
  const target = document.querySelector("#exception-history-list");

  if (!target) {
    return;
  }

  const filteredExceptions = exceptions.filter((exception) =>
    exceptionMatches(exception, query),
  );

  if (filteredExceptions.length === 0) {
    target.innerHTML = '<div class="empty-state">검색 결과가 없습니다</div>';
    return;
  }

  target.innerHTML = filteredExceptions
    .map(
      (exception) => `
      <div class="history-row ${exception.is_resolved ? "" : "danger-row"}" data-open-exception-modal="${exception.exception_id}" role="button" tabindex="0">
        <div>
          <strong>${escapeHtml(exception.exception_type)}</strong>
          <span>${escapeHtml(exception.detail || "상세 없음")}</span>
          <span>${formatDateTime(exception.created_at)} · ${escapeHtml(exception.robot_name || exception.robot_id || "로봇 미지정")}</span>
        </div>
        <div class="metric">${exception.is_resolved ? "처리" : "미처리"}</div>
      </div>
    `,
    )
    .join("");
}

function renderInventoryManager(products) {
  return `
    <div class="inventory-create-form">
      <div>
        <label for="new-product-name">상품명</label>
        <input id="new-product-name" type="text" autocomplete="off" placeholder="예: Cola">
      </div>
      <div>
        <label for="new-product-stock">수량</label>
        <input id="new-product-stock" type="number" min="0" value="0">
      </div>
      <div>
        <label for="new-product-location">보관 위치</label>
        <input id="new-product-location" type="text" autocomplete="off" placeholder="예: Zone A-1">
      </div>
      <div>
        <label for="new-product-image">이미지 URL</label>
        <input id="new-product-image" type="text" autocomplete="off" placeholder="선택">
      </div>
      <button class="small-action-button" type="button" data-create-product>상품 추가</button>
    </div>
    <div class="inventory-editor-list">
      ${
        !products || products.length === 0
          ? '<div class="empty-state">등록된 상품이 없습니다</div>'
          : products
              .map(
                (product) => `
          <div class="inventory-editor-row ${productToneClass(product.product_id)}">
            <div class="cart-item-main">
              ${productImageMarkup(product, "cart-image")}
              <div>
                <strong>${product.name}</strong>
                <span>상품 #${product.product_id}</span>
              </div>
            </div>
            <div class="inventory-product-editor">
              <input type="text" value="${product.name}" data-product-name-input="${product.product_id}" aria-label="${product.name} name">
              <input type="number" min="0" value="${product.stock_qty}" data-stock-input="${product.product_id}" aria-label="${product.name} stock">
              <input type="text" value="${productStorageLabel(product)}" data-product-location-input="${product.product_id}" aria-label="${product.name} location">
              <input type="text" value="${product.image_url || ""}" data-product-image-input="${product.product_id}" aria-label="${product.name} image url" placeholder="이미지 URL">
              <button class="small-action-button" type="button" data-save-product="${product.product_id}">저장</button>
            </div>
          </div>
        `,
              )
              .join("")
      }
    </div>
  `;
}

// =====================================
// Modal renderers
// =====================================

function openInventoryManager() {
  if (!latestAdminStatus) {
    return;
  }

  openModal(
    "Inventory Management",
    renderInventoryManager(latestAdminStatus.products || []),
  );
}

function renderProductDetail(product) {
  const level = stockLevel(product);

  return `
    <div class="product-detail-editor">
      <div class="product-detail-preview">
        ${productImageMarkup(product, "cart-image")}
        <div>
          <strong>${product.name}</strong>
          <span>상품 #${product.product_id}</span>
          <span class="${stockLevelClass(level)}">${stockLevelLabel(level)}</span>
        </div>
      </div>
      <div class="state-editor-form">
        <div>
          <label for="product-detail-name">상품명</label>
          <input id="product-detail-name" type="text" value="${product.name}" data-product-name-input="${product.product_id}">
        </div>
        <div>
          <label for="product-detail-stock">수량</label>
          <input id="product-detail-stock" type="number" min="0" value="${product.stock_qty}" data-stock-input="${product.product_id}">
        </div>
        <div>
          <label for="product-detail-location">보관 위치</label>
          <input id="product-detail-location" type="text" value="${productStorageLabel(product)}" data-product-location-input="${product.product_id}">
        </div>
        <div>
          <label for="product-detail-image">이미지 URL</label>
          <input id="product-detail-image" type="text" value="${product.image_url || ""}" data-product-image-input="${product.product_id}" placeholder="선택">
        </div>
        <button class="small-action-button" type="button" data-save-product-detail="${product.product_id}">상품 저장</button>
      </div>
    </div>
  `;
}

function openProductDetail(productId) {
  if (!latestAdminStatus) {
    return;
  }

  const product = (latestAdminStatus.products || []).find(
    (item) => item.product_id === productId,
  );

  if (!product) {
    return;
  }

  openModal(`${product.name} 수정`, renderProductDetail(product));
}

function renderRobotManager(robots) {
  return `
    <div class="task-queue-list">
      ${
        robots.length === 0
          ? '<div class="empty-state">등록된 로봇이 없습니다</div>'
          : robots
              .map(
                (robot) => `
          <button class="task-queue-row data-button" type="button" data-robot-detail="${robot.robot_id}">
            <div class="queue-rank" title="#${robot.robot_id}">${robotDisplayName(robot)}</div>
            <div class="task-main">
              <div class="task-title-line">
                <strong>${label(robotStatusValue(robot))}</strong>
                <span>${robot.current_task_id ? `작업 #${robot.current_task_id}` : "작업 없음"}</span>
              </div>
              <span>${robot.battery_level === null ? "전원 연결" : `${robot.battery_level}%`}</span>
            </div>
          </button>
        `,
              )
              .join("")
      }
    </div>
  `;
}

function openRobotManager() {
  if (!latestAdminStatus) {
    return;
  }

  openModal(
    "Robot Management",
    renderRobotManager(sortedRobots(latestAdminStatus.robots || [])),
  );
}

function renderPickupSlotManager(slots) {
  return `
    <div class="state-editor-form">
      <div>
        <label for="new-pickup-slot-name">픽업 칸 이름</label>
        <input id="new-pickup-slot-name" type="text" placeholder="예: Pickup_slot_3">
      </div>
      <div>
        <label for="new-pickup-slot-status">상태</label>
        <select id="new-pickup-slot-status">${renderOptions(PICKUP_SLOT_STATUSES, "EMPTY")}</select>
      </div>
      <button class="small-action-button" type="button" data-create-pickup-slot>픽업 칸 추가</button>
    </div>
    <div class="task-queue-list">
      ${
        slots.length === 0
          ? '<div class="empty-state">등록된 픽업 칸이 없습니다</div>'
          : slots
              .map(
                (slot) => `
          <button class="task-queue-row data-button" type="button" data-pickup-slot-detail="${slot.slot_id}">
            <div class="queue-rank">${formatSlotName(slot.slot_name)}</div>
            <div class="task-main">
              <div class="task-title-line">
                <strong>${label(slot.status)}</strong>
                <span>Slot #${slot.slot_id}</span>
              </div>
            </div>
          </button>
        `,
              )
              .join("")
      }
    </div>
  `;
}

function openPickupSlotManager() {
  if (!latestAdminStatus) {
    return;
  }

  openModal(
    "Pickup Slot Management",
    renderPickupSlotManager(latestAdminStatus.pickup_slots || []),
  );
}

function renderTaskDetail(task) {
  const deleteBlocked = ["RUNNING", "PAUSED"].includes(task.status);

  return `
    <div class="modal-summary">
      <div>
        <span>작업</span>
        <strong>${taskDisplayTitle(task)}</strong>
      </div>
      <div>
        <span>상태/우선순위</span>
        <strong>${label(task.status)} / ${task.priority ?? "-"}순위</strong>
      </div>
      <div>
        <span>로봇</span>
        <strong>${assignedRobotLabel(task)}</strong>
      </div>
    </div>
    <div class="modal-summary">
      <div>
        <span>대상</span>
        <strong>${taskTargetLabel(task)}</strong>
      </div>
      <div>
        <span>상품/수량</span>
        <strong>${taskQuantityLabel(task)}</strong>
      </div>
      <div>
        <span>경로</span>
        <strong>${taskRouteLabel(task)}</strong>
      </div>
    </div>
    <div class="state-editor-form">
      <div>
        <label for="task-robot-select">할당 로봇</label>
        <select id="task-robot-select">${renderRobotOptions(task.assigned_robot_id)}</select>
      </div>
      <button class="small-action-button" type="button" data-save-task-state="${task.task_id}">할당 저장</button>
    </div>
    ${deleteBlocked ? '<p class="muted">RUNNING/PAUSED 작업은 실행 중이므로 삭제할 수 없습니다.</p>' : ""}
  `;
}

function taskDeleteButtonMarkup(task) {
  const deleteBlocked = ["RUNNING", "PAUSED"].includes(task.status);

  return `
    <button class="danger-button modal-header-danger" type="button" data-delete-task="${task.task_id}" ${deleteBlocked ? "disabled" : ""}>
      작업 삭제
    </button>
  `;
}

function renderTaskCreateForm(zones) {
  return `
    <div class="state-editor-form task-create-form">
      <div>
        <label for="new-task-type">작업 유형</label>
        <select id="new-task-type">${renderOptions(TASK_TYPE_SEQUENCE, "MOVE_TO_PRODUCT")}</select>
      </div>
      <div>
        <label for="new-task-status">작업 상태</label>
        <select id="new-task-status">${renderOptions(TASK_STATUSES, "ASSIGNED")}</select>
      </div>
      <div>
        <label for="new-task-robot-select">할당 로봇</label>
        <select id="new-task-robot-select">${renderRobotOptions(null)}</select>
      </div>
      <div>
        <label for="new-task-order-id">주문 ID</label>
        <input id="new-task-order-id" type="number" min="1" placeholder="없으면 비움">
      </div>
      <div>
        <label for="new-task-order-item-id">주문 상품 ID</label>
        <input id="new-task-order-item-id" type="number" min="1" placeholder="없으면 비움">
      </div>
      <div>
        <label for="new-task-display-item-id">진열 요청 ID</label>
        <input id="new-task-display-item-id" type="number" min="1" placeholder="없으면 비움">
      </div>
      <div>
        <label for="new-task-priority">우선순위</label>
        <input id="new-task-priority" type="number" min="1" value="${recommendedTaskPriority("MOVE_TO_PRODUCT")}">
      </div>
      <div>
        <label for="new-task-source-zone-id">출발 구역</label>
        <select id="new-task-source-zone-id">${renderZoneOptions(zones, null)}</select>
      </div>
      <div>
        <label for="new-task-target-zone-id">목표 구역</label>
        <select id="new-task-target-zone-id">${renderZoneOptions(zones, null)}</select>
      </div>
      <div class="state-editor-wide">
        <label for="new-task-result-message">결과 메시지</label>
        <input id="new-task-result-message" type="text" placeholder="선택 입력">
      </div>
      <button class="small-action-button" type="button" data-create-task>작업 생성</button>
    </div>
  `;
}

function syncTaskCreatePriorityDefault({ force = false } = {}) {
  const taskType = modalBody?.querySelector("#new-task-type")?.value;
  const displayItemId = modalBody
    ?.querySelector("#new-task-display-item-id")
    ?.value.trim();
  const priorityInput = modalBody?.querySelector("#new-task-priority");

  if (!priorityInput) {
    return;
  }

  const recommended = String(recommendedTaskPriority(taskType, displayItemId));
  const current = priorityInput.value;

  if (force || current === "" || current === "1" || current === "2") {
    priorityInput.value = recommended;
  }
}

function renderTaskManager(tasks) {
  const sortedTasks = [...(tasks || [])].sort((a, b) => b.task_id - a.task_id);

  return `
    <div class="task-history-toolbar">
      <div class="task-history-filters">
        <input data-task-history-filter type="search" placeholder="작업 ID, 유형, 주문, 로봇, 상태, 상품, zone 검색">
        <select data-task-history-status-filter>
          ${renderOptions(TASK_STATUSES, "", "상태 전체")}
        </select>
      </div>
      <button class="small-action-button" type="button" data-open-task-create>작업 생성</button>
    </div>
    <div class="task-queue-list">
      ${
        sortedTasks.length === 0
          ? '<div class="empty-state">작업이 없습니다</div>'
          : sortedTasks
              .map(
                (task) => `
          <div class="task-queue-row task-history-row" data-task-history-row data-task-detail="${task.task_id}" data-task-status="${task.status}" data-task-search="${escapeAttribute(taskSearchText(task))}">
            <div class="queue-rank">#${task.task_id}</div>
            <div class="task-main">
              <div class="task-title-line">
                <strong>${taskDisplayTitle(task)}</strong>
                <span>${taskTargetLabel(task)}</span>
              </div>
              <span>${assignedRobotLabel(task)} · ${taskReferenceLabel(task)}</span>
            </div>
            <div class="task-side">
              ${renderTaskHistoryStatusControl(task)}
            </div>
          </div>
        `,
              )
              .join("")
      }
      <div class="empty-state" data-task-history-empty hidden>검색 결과가 없습니다</div>
    </div>
  `;
}

function renderTaskHistoryStatusControl(task) {
  return `<div class="state-badge ${statusClass(task.status)}">${label(task.status)}</div>`;
}

function taskSearchText(task) {
  return [
    `#${task.task_id}`,
    task.task_id,
    taskDisplayTitle(task),
    task.task_type,
    taskTargetLabel(task),
    taskReferenceLabel(task),
    task.order_no,
    task.order_id,
    task.order_item_id,
    task.display_item_id,
    assignedRobotLabel(task),
    task.status,
    label(task.status),
    task.product_name,
    task.product_quantity,
    task.processed_quantity,
    task.stock_delta,
    task.priority,
    task.sequence_no,
    task.source_zone_name,
    task.target_zone_name,
  ]
    .filter((value) => value !== null && value !== undefined && value !== "")
    .join(" ")
    .toLowerCase();
}

function applyTaskHistoryFilter() {
  const needle = (
    modalBody.querySelector("[data-task-history-filter]")?.value || ""
  )
    .trim()
    .toLowerCase();
  const statusFilter =
    modalBody.querySelector("[data-task-history-status-filter]")?.value || "";
  const rows = [...modalBody.querySelectorAll("[data-task-history-row]")];
  let visibleCount = 0;

  rows.forEach((row) => {
    const isVisible =
      (!needle || row.dataset.taskSearch.includes(needle)) &&
      (!statusFilter || row.dataset.taskStatus === statusFilter);
    row.hidden = !isVisible;

    if (isVisible) {
      visibleCount += 1;
    }
  });

  const emptyState = modalBody.querySelector("[data-task-history-empty]");

  if (emptyState) {
    emptyState.hidden = rows.length === 0 || visibleCount > 0;
  }
}

async function loadFleetTasks() {
  const response = await fetch("/api/fleet/tasks");

  if (!response.ok) {
    throw await errorFromResponse(response, "task list load failed");
  }

  return response.json();
}

async function openTaskManager() {
  if (!latestAdminStatus) {
    return;
  }

  openModal(
    "Task Management",
    '<div class="empty-state">작업 목록을 불러오는 중입니다</div>',
  );

  try {
    const tasks = await loadFleetTasks();
    latestAdminStatus.tasks = tasks;
    modalBody.innerHTML = renderTaskManager(tasks);
  } catch (error) {
    console.error(error);
    modalBody.innerHTML = renderTaskManager(latestAdminStatus.tasks || []);
  }
}

async function openTaskCreate() {
  resetModalHeaderActions();
  const zones = await loadZoneOptions();
  openModal("작업 생성", renderTaskCreateForm(zones));
  syncTaskCreatePriorityDefault({ force: true });
}

function openTaskDetail(
  taskId,
  { returnToCurrentModal = false, keepReturnStack = false } = {},
) {
  if (!latestAdminStatus) {
    return;
  }

  const task = (latestAdminStatus.tasks || []).find(
    (item) => item.task_id === taskId,
  );

  if (!task) {
    return;
  }

  const returnState = returnToCurrentModal ? captureModalState() : null;

  openModal(`작업 #${task.task_id}`, renderTaskDetail(task), {
    keepReturnState: returnToCurrentModal || keepReturnStack,
  });
  setModalHeaderActions(taskDeleteButtonMarkup(task));

  if (returnState) {
    modalReturnStack.push(returnState);
  }
}

function renderAdminStatus(data) {
  latestAdminStatus = data;
  normalizeOrderWorkSelection(data);
  const robots = sortedRobots(data.robots || []);
  const robotCounts = robots.reduce(
    (counts, robot) => {
      counts.total += 1;
      counts[robotCategory(robot)] += 1;
      return counts;
    },
    { total: 0, idle: 0, working: 0, error: 0 },
  );
  const activeRobots =
    robotCounts.total -
    robots.filter((robot) => robotStatusValue(robot) === "OFFLINE").length;

  if (summaryRobots) {
    summaryRobots.textContent = String(robotCounts.total);
  }

  if (summaryActiveRobots) {
    summaryActiveRobots.textContent = String(activeRobots);
  }

  if (summaryRunningRobots) {
    summaryRunningRobots.textContent = String(activeRobots);
  }

  if (summaryIdleRobots) {
    summaryIdleRobots.textContent = String(robotCounts.idle);
  }

  if (summaryWorkingRobots) {
    summaryWorkingRobots.textContent = String(robotCounts.working);
  }

  if (summaryErrorRobots) {
    summaryErrorRobots.textContent = String(robotCounts.error);
  }

  if (robotDonut) {
    const statusTotal =
      robotCounts.idle + robotCounts.working + robotCounts.error || 1;
    const idleEnd = (robotCounts.idle / statusTotal) * 100;
    const workingEnd = idleEnd + (robotCounts.working / statusTotal) * 100;
    const errorEnd = workingEnd + (robotCounts.error / statusTotal) * 100;
    robotDonut.style.background = `conic-gradient(#94a3b8 0 ${idleEnd}%, #f59e0b ${idleEnd}% ${workingEnd}%, #ef4444 ${workingEnd}% ${errorEnd}%, #334155 ${errorEnd}% 100%)`;
  }

  if (summaryOrders) {
    summaryOrders.textContent = String(activeWorkItems(data).length);
  }

  if (summaryExceptions) {
    summaryExceptions.textContent = String(
      data.unresolved_exception_count ?? data.exceptions.length,
    );
  }

  if (summaryTasks) {
    summaryTasks.textContent = String(activeWorkItems(data).length);
  }

  renderRobots(robots);
  renderMapRobots(robots);
  renderOrders(data.orders);
  renderPickupSlots(data.pickup_slots);
  renderInventory(data.products || []);
  renderTaskSnapshot(data.tasks || []);
  renderOrderWorkDetail();
  renderExceptions(
    adminPage === "dashboard"
      ? allExceptionsFromStatus(data).slice(0, 5)
      : data.exceptions,
  );
}

function renderRobotTaskQueue(robot) {
  const tasks = getRobotTasks(robot.robot_id);
  const status = robotStatusValue(robot);

  return `
    <div class="state-editor-form">
      <div>
        <label for="robot-status-select">로봇 상태</label>
        <select id="robot-status-select">${renderOptions(ROBOT_STATUSES, status)}</select>
      </div>
      <div>
        <label for="robot-state-select">세부 상태</label>
        <select id="robot-state-select">${renderRobotStateOptions(robot)}</select>
      </div>
      <div>
        <label for="robot-current-task-select">현재 작업</label>
        <select id="robot-current-task-select">${renderTaskOptions(robot.current_task_id)}</select>
      </div>
      <button class="small-action-button" type="button" data-save-robot-state="${robot.robot_id}">상태 저장</button>
    </div>
    ${
      tasks.length === 0
        ? '<div class="empty-state">할당된 작업이 없습니다</div>'
        : `
    <div class="task-queue-list">
      ${tasks
        .map(
          (task, index) => `
          <button class="task-queue-row data-button" type="button" data-task-detail="${task.task_id}">
            <div class="queue-rank">${index + 1}</div>
            <div class="task-main">
              <div class="task-title-line">
                <strong>${taskDisplayTitle(task)}</strong>
                <span>작업 #${task.task_id}</span>
              </div>
              <span>${taskTargetLabel(task)} · ${taskReferenceLabel(task)}</span>
            </div>
            <div class="task-side">
              <div class="state-badge ${statusClass(task.status)}">${label(task.status)}</div>
            </div>
          </button>
        `,
        )
        .join("")}
    </div>
    `
    }
  `;
}

function renderPickupSlotDetail(slot) {
  return `
    <div class="state-editor-form">
      <div>
        <label for="pickup-slot-status-select">현재 상태</label>
        <select id="pickup-slot-status-select">${renderOptions(PICKUP_SLOT_STATUSES, slot.status)}</select>
      </div>
      <button class="small-action-button" type="button" data-save-pickup-slot-state="${slot.slot_id}">상태 저장</button>
    </div>
  `;
}

function openRobotDetail(robotId) {
  if (!latestAdminStatus) {
    return;
  }

  const robot = latestAdminStatus.robots.find(
    (item) => sameId(item.robot_id, robotId) || item.robot_name === robotId,
  );

  if (!robot) {
    return;
  }

  openModal(
    `${robotDisplayName(robot)} Task Queue`,
    renderRobotTaskQueue(robot),
  );
}

function openPickupSlotDetail(slotId) {
  if (!latestAdminStatus) {
    return;
  }

  const slot = latestAdminStatus.pickup_slots.find(
    (item) => item.slot_id === slotId,
  );

  if (!slot) {
    return;
  }

  openModal(
    `${formatSlotName(slot.slot_name)} 픽업 슬롯`,
    renderPickupSlotDetail(slot),
    { size: "compact" },
  );
}

function openOrderDetail(
  orderId,
  { returnToCurrentModal = false, keepReturnStack = false } = {},
) {
  if (!latestAdminStatus) {
    return;
  }

  const order =
    typeof orderId === "string" && orderId.includes(":")
      ? findWorkByKey(orderId, { includeHistory: true })
      : (() => {
          const item = findOrder(orderId, { includeHistory: true });
          return item ? orderWorkItem(item) : null;
        })();

  if (!order) {
    return;
  }

  const returnState = returnToCurrentModal ? captureModalState() : null;

  openModal(workDisplayTitle(order), renderOrderDetail(order), {
    keepReturnState: returnToCurrentModal || keepReturnStack,
  });

  if (returnState) {
    modalReturnStack.push(returnState);
  }
}

function openOrderHistory() {
  if (!latestAdminStatus) {
    return;
  }

  const historyGroups = historyWorkGroups(latestAdminStatus);
  const hasHistory = historyGroups.orders.length > 0 || historyGroups.displays.length > 0;

  const body =
    !hasHistory
      ? '<div class="empty-state">완료된 주문/진열 작업이 없습니다</div>'
      : renderCompletedWorkHistory(historyGroups);

  openModal("완료 작업 이력", body);
}

function renderCompletedWorkHistory({ orders, displays }) {
  return `
    <div class="completed-work-history-grid">
      ${renderCompletedWorkHistoryColumn("주문 완료 이력", orders, "완료된 주문이 없습니다")}
      ${renderCompletedWorkHistoryColumn("진열 완료 이력", displays, "완료된 진열이 없습니다")}
    </div>
  `;
}

function renderCompletedWorkHistoryColumn(title, works, emptyText) {
  return `
    <section class="completed-work-history-column">
      <div class="completed-work-history-heading">
        <h3>${title}</h3>
        <span>${works.length}건</span>
      </div>
      ${
        works.length === 0
          ? `<div class="empty-state compact-empty-state">${emptyText}</div>`
          : `
            <div class="history-list completed-work-history-list">
              ${works.map(renderCompletedWorkHistoryRow).join("")}
            </div>
          `
      }
    </section>
  `;
}

function renderCompletedWorkHistoryRow(work) {
  return `
    <button class="history-row" type="button" data-work-history-detail="${work.work_key}">
      <div>
        <strong>${workDisplayTitle(work)}</strong>
        <span>${workKindLabel(work)} · ${orderProductSummary(work)}</span>
      </div>
      <div class="history-status-large">${label(work.status)}</div>
    </button>
  `;
}

function openExceptionHistory() {
  if (!latestAdminStatus) {
    return;
  }

  const exceptions = allExceptionsFromStatus(latestAdminStatus);

  const body =
    exceptions.length === 0
      ? '<div class="empty-state">예외/알람이 없습니다</div>'
      : `
      <div class="history-search">
        <input id="exception-history-search" type="search" placeholder="예외 타입, 상세, 로봇, 처리 상태, 시간(예: 17:31, 05.07) 검색">
      </div>
      <div id="exception-history-list" class="history-list"></div>
    `;

  openModal("예외/알람 이력", body);
  renderExceptionHistoryList(exceptions);
}

// =====================================
// Status loading / websocket
// =====================================

async function loadAdminStatus() {
  try {
    const response = await fetch("/api/admin/status");
    if (!response.ok) {
      throw new Error("관리자 상태 조회 실패");
    }

    const data = await response.json();
    renderAdminStatus(data);
  } catch (error) {
    renderEmpty(robotStatus, "관리자 상태를 불러오지 못했습니다");
  }
}

function startFallbackPolling() {
  if (fallbackTimer) {
    return;
  }

  fallbackTimer = setInterval(loadAdminStatus, 3000);
}

function stopFallbackPolling() {
  if (!fallbackTimer) {
    return;
  }

  clearInterval(fallbackTimer);
  fallbackTimer = null;
}

function connectAdminSocket() {
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  adminSocket = new WebSocket(
    `${protocol}://${window.location.host}/api/admin/ws/status`,
  );

  adminSocket.addEventListener("open", () => {
    setSocketState("online");
    stopFallbackPolling();
  });

  adminSocket.addEventListener("message", (event) => {
    renderAdminStatus(JSON.parse(event.data));
  });

  adminSocket.addEventListener("close", () => {
    setSocketState("offline");
    startFallbackPolling();
    setTimeout(connectAdminSocket, 3000);
  });

  adminSocket.addEventListener("error", () => {
    setSocketState("offline");
    adminSocket.close();
  });
}

hydrateRobotFilters();
updateDashboardMapViewport();
loadAdminStatus();
connectAdminSocket();
window.addEventListener("resize", updateDashboardMapViewport);

// =====================================
// API helpers
// =====================================

async function postAdminAction(path) {
  const response = await fetch(path, { method: "POST" });

  if (!response.ok) {
    throw new Error("관리자 작업 요청 실패");
  }

  await loadAdminStatus();
}

async function errorFromResponse(response, fallbackMessage) {
  try {
    const body = await response.json();
    const detail = body.detail;

    if (typeof detail === "string") {
      return new Error(detail);
    }

    if (detail?.message) {
      return new Error(detail.message);
    }
  } catch (error) {
    // JSON body가 없는 응답이면 기본 메시지를 사용한다.
  }

  return new Error(fallbackMessage);
}

async function postJson(path, body) {
  const response = await fetch(path, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify(body),
  });

  if (!response.ok) {
    throw await errorFromResponse(response, "생성 요청 실패");
  }

  const data = await response.json();
  await loadAdminStatus();
  return data;
}

async function patchJson(path, body) {
  const response = await fetch(path, {
    method: "PATCH",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify(body),
  });

  if (!response.ok) {
    throw await errorFromResponse(response, "상태 변경 실패");
  }

  await loadAdminStatus();
}

async function deleteJson(path) {
  const response = await fetch(path, { method: "DELETE" });

  if (!response.ok) {
    throw await errorFromResponse(response, "삭제 요청 실패");
  }

  await loadAdminStatus();
}

async function loadZoneOptions() {
  if (zoneOptionsCache) {
    return zoneOptionsCache;
  }

  const response = await fetch("/api/fleet/zones?zone_type=ALL");

  if (!response.ok) {
    throw await errorFromResponse(response, "구역 목록 조회 실패");
  }

  zoneOptionsCache = await response.json();
  return zoneOptionsCache;
}

function selectNumberOrNull(selector) {
  const value = modalBody.querySelector(selector)?.value;
  return value ? Number(value) : null;
}

function inputNumberOrNull(selector) {
  const value = modalBody.querySelector(selector)?.value;
  return value === "" || value === undefined ? null : Number(value);
}

function taskIntegerOrNull(
  selector,
  fieldName,
  { required = false, min = null } = {},
) {
  const rawValue = modalBody.querySelector(selector)?.value.trim() || "";

  if (!rawValue) {
    if (required) {
      throw new Error(`${fieldName} 값을 입력해주세요.`);
    }

    return null;
  }

  const value = Number(rawValue);

  if (!Number.isInteger(value) || (min !== null && value < min)) {
    throw new Error(`${fieldName} 값이 올바르지 않습니다.`);
  }

  return value;
}

async function createTask() {
  syncTaskCreatePriorityDefault();

  const resultMessage = modalBody
    .querySelector("#new-task-result-message")
    ?.value.trim();
  const task = {
    task_type: modalBody.querySelector("#new-task-type")?.value,
    status: modalBody.querySelector("#new-task-status")?.value,
    assigned_robot_id: selectNumberOrNull("#new-task-robot-select"),
    order_id: taskIntegerOrNull("#new-task-order-id", "주문 ID", { min: 1 }),
    order_item_id: taskIntegerOrNull(
      "#new-task-order-item-id",
      "주문 상품 ID",
      { min: 1 },
    ),
    display_item_id: taskIntegerOrNull(
      "#new-task-display-item-id",
      "진열 요청 ID",
      { min: 1 },
    ),
    priority: taskIntegerOrNull("#new-task-priority", "우선순위", {
      required: true,
      min: 1,
    }),
    source_zone_id: selectNumberOrNull("#new-task-source-zone-id"),
    target_zone_id: selectNumberOrNull("#new-task-target-zone-id"),
    result_message: resultMessage || null,
  };

  if (task.display_item_id && (task.order_id || task.order_item_id)) {
    throw new Error(
      "진열 작업은 주문 ID/주문 상품 ID와 같이 만들 수 없습니다.",
    );
  }

  await postJson("/api/fleet/tasks/bulk", { tasks: [task] });
  openTaskManager();
}

async function deleteTask(taskId) {
  const task = findTask(taskId);

  if (!task) {
    return false;
  }

  if (
    !confirm(
      `작업 #${task.task_id} ${taskDisplayTitle(task)} 작업을 삭제할까요?`,
    )
  ) {
    return false;
  }

  await deleteJson(`/api/fleet/tasks/${taskId}`);
  selectedTaskId = null;
  openTaskManager();
  return true;
}

async function updateOrderState(orderId) {
  const payload = {
    status: modalBody.querySelector("#order-status-select")?.value,
    pickup_slot_id: selectNumberOrNull("#order-pickup-slot-select"),
  };
  const itemQuantities = [
    ...modalBody.querySelectorAll("[data-order-item-quantity]"),
  ]
    .filter((input) => !input.disabled)
    .map((input) => {
      const quantity = Number(input.value);

      if (!Number.isInteger(quantity) || quantity < 1) {
        throw new Error("상품 수량은 1 이상 정수로 입력해주세요.");
      }

      return {
        item_id: Number(input.dataset.orderItemQuantity),
        quantity,
        current_quantity: Number(input.dataset.currentQuantity),
      };
    })
    .filter((item) => item.quantity !== item.current_quantity)
    .map(({ item_id, quantity }) => ({ item_id, quantity }));

  if (itemQuantities.length > 0) {
    payload.item_quantities = itemQuantities;
  }

  await patchJson(`/api/fleet/orders/${orderId}`, payload);
  openOrderDetail(orderId, { keepReturnStack: true });
}

async function updateTaskState(taskId) {
  const assignedRobotId =
    modalBody.querySelector("#task-robot-select")?.value || null;

  await patchJson(`/api/fleet/tasks/${taskId}`, {
    assigned_robot_id: assignedRobotId ? Number(assignedRobotId) : null,
  });
  openTaskDetail(taskId, { keepReturnStack: true });
}

function robotStateUpdatePayload(robotId, statusSelector, stateSelector) {
  const robot = findRobotById(robotId);
  const payload = {
    robot_status: document.querySelector(statusSelector)?.value,
  };
  const stateValue = document.querySelector(stateSelector)?.value || null;

  if (robotType(robot) === "PICKY") {
    payload.picky_state = stateValue;
  } else if (robotType(robot) === "COBOT") {
    payload.cobot_state = stateValue;
  }

  return payload;
}

async function updateRobotState(robotId) {
  const encodedRobotId = encodeURIComponent(robotId);
  await patchJson(`/api/fleet/robots/${encodedRobotId}`, {
    ...robotStateUpdatePayload(
      robotId,
      "#robot-status-select",
      "#robot-state-select",
    ),
    current_task_id: selectNumberOrNull("#robot-current-task-select"),
  });
  openRobotDetail(robotId);
}

async function updatePickupSlotState(slotId) {
  await patchJson(`/api/fleet/pickup-slots/${slotId}`, {
    status: modalBody.querySelector("#pickup-slot-status-select")?.value,
  });
  openPickupSlotDetail(slotId);
}

async function updateRobotPanelState(robotId) {
  const encodedRobotId = encodeURIComponent(robotId);
  await patchJson(`/api/fleet/robots/${encodedRobotId}`, {
    ...robotStateUpdatePayload(
      robotId,
      "#robot-panel-status-select",
      "#robot-panel-state-select",
    ),
    current_task_id: (() => {
      const value = document.querySelector(
        "#robot-panel-current-task-select",
      )?.value;
      return value ? Number(value) : null;
    })(),
  });
  selectedRobotId = robotId;
}

async function updateProductStock(productId, stockQty) {
  const response = await fetch(`/api/admin/products/${productId}/stock`, {
    method: "PATCH",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({ stock_qty: stockQty }),
  });

  if (!response.ok) {
    throw new Error("재고 변경 실패");
  }

  await loadAdminStatus();
  openInventoryManager();
}

async function updateProduct(productId, reopenMode = "manager") {
  const name = modalBody
    .querySelector(`input[data-product-name-input="${productId}"]`)
    ?.value.trim();
  const stockQty = Number(
    modalBody.querySelector(`input[data-stock-input="${productId}"]`)?.value,
  );
  const storageLocation = modalBody
    .querySelector(`input[data-product-location-input="${productId}"]`)
    ?.value.trim();
  const imageUrl = modalBody
    .querySelector(`input[data-product-image-input="${productId}"]`)
    ?.value.trim();

  if (
    !name ||
    !storageLocation ||
    !Number.isInteger(stockQty) ||
    stockQty < 0
  ) {
    alert("상품명, 수량, 보관 위치를 확인해주세요.");
    return;
  }

  const response = await fetch(`/api/admin/products/${productId}`, {
    method: "PATCH",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      name,
      stock_qty: stockQty,
      storage_location: storageLocation,
      image_url: imageUrl || null,
    }),
  });

  if (!response.ok) {
    throw new Error("상품 정보 변경 실패");
  }

  await loadAdminStatus();

  if (reopenMode === "detail") {
    openProductDetail(productId);
    return;
  }

  openInventoryManager();
}

async function createProduct() {
  const nameInput = modalBody.querySelector("#new-product-name");
  const stockInput = modalBody.querySelector("#new-product-stock");
  const locationInput = modalBody.querySelector("#new-product-location");
  const imageInput = modalBody.querySelector("#new-product-image");

  const name = nameInput?.value.trim();
  const stockQty = Number(stockInput?.value);
  const storageLocation = locationInput?.value.trim();
  const imageUrl = imageInput?.value.trim();

  if (
    !name ||
    !storageLocation ||
    !Number.isInteger(stockQty) ||
    stockQty < 0
  ) {
    alert("상품명, 수량, 보관 위치를 확인해주세요.");
    return;
  }

  const response = await fetch("/api/admin/products", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      name,
      stock_qty: stockQty,
      storage_location: storageLocation,
      image_url: imageUrl || null,
    }),
  });

  if (!response.ok) {
    throw new Error("상품 생성 실패");
  }

  await loadAdminStatus();
  openInventoryManager();
}

async function createPickupSlot() {
  const slotName = modalBody
    .querySelector("#new-pickup-slot-name")
    ?.value.trim();
  const status = modalBody.querySelector("#new-pickup-slot-status")?.value;

  if (!slotName) {
    alert("픽업 칸 이름을 입력해주세요.");
    return;
  }

  const response = await fetch("/api/admin/pickup-slots", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      slot_name: slotName,
      status,
    }),
  });

  if (!response.ok) {
    throw new Error("픽업 칸 생성 실패");
  }

  await loadAdminStatus();
  openPickupSlotManager();
}

function appendLlmMessage(role, text) {
  if (!llmMessages) {
    return;
  }

  const message = document.createElement("div");
  message.className = `llm-message ${role}`;
  message.textContent = text;
  llmMessages.appendChild(message);
  llmMessages.scrollTop = llmMessages.scrollHeight;
}

function buildLlmFailureReply(command) {
  const lowerCommand = command.toLowerCase();

  if (
    lowerCommand.includes("진열") ||
    lowerCommand.includes("display") ||
    lowerCommand.includes("place")
  ) {
    return "진열 명령을 처리하지 못했습니다. AI 메시지 API와 Fleet Manager 상태를 확인해주세요.";
  }

  if (lowerCommand.includes("재고") || lowerCommand.includes("stock")) {
    return `현재 재고 부족 상품은 ${latestAdminStatus?.low_stock_count ?? 0}개입니다. Inventory 관리에서 수량을 조정할 수 있습니다.`;
  }

  if (lowerCommand.includes("예외") || lowerCommand.includes("exception")) {
    return `미처리 예외는 ${latestAdminStatus?.unresolved_exception_count ?? 0}건입니다. Exceptions 영역에서 처리할 수 있습니다.`;
  }

  return "AI 메시지 API 호출에 실패했습니다. 서버 상태를 확인해주세요.";
}

async function sendLlmMessage(message) {
  const response = await fetch("/api/admin/llm/messages", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({ message }),
  });

  if (!response.ok) {
    throw new Error("AI 메시지 전송 실패");
  }

  return response.json();
}

function setDashboardLlmFeedback(state, statusText, resultText) {
  if (dashboardLlmStatus) {
    dashboardLlmStatus.className = `ai-command-status ${state}`;
    dashboardLlmStatus.textContent = statusText;
  }

  if (dashboardLlmResult) {
    dashboardLlmResult.textContent = resultText;
  }
}

function refreshOrderManagementView() {
  renderOrders(latestAdminStatus?.orders || []);
  renderTaskSnapshot(latestAdminStatus?.tasks || []);
  renderOrderWorkDetail();
}

function selectWorkInOrderManagement(work) {
  selectWork(work);
  selectedTaskId = null;
  refreshOrderManagementView();
}

function selectOrderInOrderManagement(orderId) {
  selectedOrderId = Number(orderId);
  selectedDisplayItemId = null;
  selectedTaskId = null;
  refreshOrderManagementView();
}

function selectTaskInOrderManagement(taskId, { openDetail = false } = {}) {
  const normalizedTaskId = Number(taskId);
  const task = findTask(normalizedTaskId);
  const work = findWorkByTask(task);

  selectedTaskId = normalizedTaskId;
  selectWork(work || selectedWork());
  refreshOrderManagementView();

  if (openDetail) {
    openTaskDetail(normalizedTaskId);
  }
}

function handleWorkDetailButton(button) {
  if (!button) {
    return false;
  }

  const work = findWorkByKey(button.dataset.workDetail);

  if (!work) {
    return true;
  }

  if (adminPage === "orders") {
    selectWorkInOrderManagement(work);
    return true;
  }

  openOrderDetail(workKey(work));
  return true;
}

function handleOrderDetailButton(button) {
  if (!button) {
    return false;
  }

  if (adminPage === "orders") {
    selectOrderInOrderManagement(button.dataset.orderDetail);
    return true;
  }

  openOrderDetail(Number(button.dataset.orderDetail));
  return true;
}

// =====================================
// Event bindings
// =====================================

emergencyStopButton?.addEventListener("click", async () => {
  emergencyStopButton.disabled = true;
  try {
    await postAdminAction("/api/admin/emergency-stop");
  } finally {
    emergencyStopButton.disabled = false;
  }
});

resumeButton?.addEventListener("click", async () => {
  resumeButton.disabled = true;
  try {
    await postAdminAction("/api/admin/resume");
  } finally {
    resumeButton.disabled = false;
  }
});

exceptionList?.addEventListener("click", async (event) => {
  const button = event.target.closest("button[data-resolve-exception]");

  if (button) {
    const exceptionId = button.dataset.resolveException;

    try {
      await resolveException(exceptionId, button);
    } catch (error) {
      alert(error.message);
    }
    return;
  }

  const row = event.target.closest("[data-open-exception-modal]");

  if (row) {
    openExceptionDetail(row.dataset.openExceptionModal);
  }
});

exceptionList?.addEventListener("keydown", handleExceptionDetailKeydown);

modalPanel?.addEventListener("click", async (event) => {
  const button = event.target.closest("button[data-resolve-exception-detail]");

  if (!button) {
    return;
  }

  const exceptionId = button.dataset.resolveExceptionDetail;

  try {
    await resolveException(exceptionId, button);
    openExceptionDetail(exceptionId);
  } catch (error) {
    alert(error.message);
  }
});

orderList?.addEventListener("click", (event) => {
  if (handleWorkDetailButton(event.target.closest("button[data-work-detail]"))) {
    return;
  }

  handleOrderDetailButton(event.target.closest("button[data-order-detail]"));
});

robotStatus?.addEventListener("click", (event) => {
  const row = event.target.closest("[data-robot-select]");

  if (adminPage === "robots") {
    if (!row) {
      return;
    }

    selectedRobotId = row.dataset.robotSelect;
    renderRobots(latestAdminStatus?.robots || []);
    return;
  }

  const button = event.target.closest("button[data-robot-detail]");

  if (!button) {
    return;
  }

  openRobotDetail(button.dataset.robotDetail);
});

robotDetailPanel?.addEventListener("click", (event) => {
  const button = event.target.closest("button[data-save-robot-panel]");

  if (!button) {
    return;
  }

  button.disabled = true;
  updateRobotPanelState(button.dataset.saveRobotPanel).catch((error) => {
    alert(error.message);
    button.disabled = false;
  });
});

[robotSearchInput, robotStatusFilter, robotTypeFilter].forEach((element) => {
  element?.addEventListener("input", () => {
    renderRobots(latestAdminStatus?.robots || []);
  });

  element?.addEventListener("change", () => {
    renderRobots(latestAdminStatus?.robots || []);
  });
});

pickupSlotList?.addEventListener("click", (event) => {
  const button = event.target.closest("button[data-pickup-slot-detail]");

  if (!button) {
    return;
  }

  openPickupSlotDetail(Number(button.dataset.pickupSlotDetail));
});

pickupSummaryList?.addEventListener("click", (event) => {
  const button = event.target.closest("button[data-pickup-slot-detail]");

  if (!button) {
    return;
  }

  openPickupSlotDetail(Number(button.dataset.pickupSlotDetail));
});

inventoryList?.addEventListener("click", (event) => {
  const button = event.target.closest("button[data-product-detail]");

  if (!button) {
    return;
  }

  if (adminPage === "inventory") {
    openProductDetail(Number(button.dataset.productDetail));
    return;
  }

  openInventoryManager();
});

taskList?.addEventListener("click", (event) => {
  if (handleWorkDetailButton(event.target.closest("button[data-work-detail]"))) {
    return;
  }

  if (handleOrderDetailButton(event.target.closest("button[data-order-detail]"))) {
    return;
  }

  const button = event.target.closest("button[data-task-detail]");

  if (!button) {
    return;
  }

  if (adminPage === "orders") {
    selectTaskInOrderManagement(button.dataset.taskDetail, { openDetail: true });
    return;
  }

  return;
});

orderWorkDetailPanel?.addEventListener("click", (event) => {
  const taskButton = event.target.closest("button[data-work-task]");

  if (taskButton) {
    selectTaskInOrderManagement(taskButton.dataset.workTask);
    return;
  }

  const orderModalButton = event.target.closest(
    "button[data-open-order-modal]",
  );

  if (orderModalButton) {
    openOrderDetail(Number(orderModalButton.dataset.openOrderModal));
    return;
  }

  const taskModalButton = event.target.closest("button[data-open-task-modal]");

  if (taskModalButton) {
    openTaskDetail(Number(taskModalButton.dataset.openTaskModal));
  }
});

orderHistoryButton?.addEventListener("click", openOrderHistory);
exceptionHistoryButton?.addEventListener("click", openExceptionHistory);
robotManageButton?.addEventListener("click", openRobotManager);
pickupSlotManageButton?.addEventListener("click", openPickupSlotManager);
inventoryManageButton?.addEventListener("click", openInventoryManager);
taskCreateButton?.addEventListener("click", () => {
  openTaskCreate().catch((error) => {
    alert(error.message);
  });
});
taskViewButton?.addEventListener("click", openTaskManager);
modalBody?.addEventListener("input", (event) => {
  if (event.target.id === "new-task-display-item-id") {
    syncTaskCreatePriorityDefault();
    return;
  }

  const input = event.target.closest("[data-task-history-filter]");

  if (!input) {
    return;
  }

  applyTaskHistoryFilter();
});

modalBody?.addEventListener("change", (event) => {
  if (event.target.id === "new-task-type") {
    syncTaskCreatePriorityDefault({ force: true });
    return;
  }

  const statusFilter = event.target.closest(
    "[data-task-history-status-filter]",
  );

  if (statusFilter) {
    event.stopPropagation();
    applyTaskHistoryFilter();
  }
});
llmOpenButton?.addEventListener("click", () => {
  if (llmPanel) {
    llmPanel.hidden = false;
    llmInput?.focus();
  }
});
llmCloseButton?.addEventListener("click", () => {
  if (llmPanel) {
    llmPanel.hidden = true;
  }
});
llmForm?.addEventListener("submit", async (event) => {
  event.preventDefault();

  const command = llmInput?.value.trim();

  if (!command) {
    return;
  }

  appendLlmMessage("user", command);
  llmInput.value = "";

  try {
    const response = await sendLlmMessage(command);
    appendLlmMessage("bot", response.message);
  } catch (error) {
    appendLlmMessage("bot", buildLlmFailureReply(command));
  }
});
dashboardLlmForm?.addEventListener("submit", async (event) => {
  event.preventDefault();

  const command = dashboardLlmInput?.value.trim();

  if (!command) {
    return;
  }

  const submitButton = dashboardLlmForm.querySelector("button[type='submit']");
  if (submitButton) {
    submitButton.disabled = true;
  }
  dashboardLlmInput.value = "";
  setDashboardLlmFeedback(
    "running",
    "명령 전송 중",
    `"${command}" 명령을 AI 메시지 API로 보내는 중입니다.`,
  );

  try {
    const response = await sendLlmMessage(command);
    const isError = response.result === "error";
    setDashboardLlmFeedback(
      isError ? "error" : "success",
      isError ? "응답 실패" : "응답 완료",
      response.message || "AI 응답이 도착했습니다.",
    );
  } catch (error) {
    setDashboardLlmFeedback(
      "error",
      "응답 실패",
      "AI 메시지 API 호출에 실패했습니다. 서버 상태를 확인해주세요.",
    );
  } finally {
    if (submitButton) {
      submitButton.disabled = false;
    }
    dashboardLlmInput?.focus();
  }
});
modalCloseButton?.addEventListener("click", closeModal);

document.addEventListener("keydown", (event) => {
  if (event.key === "Escape" && modalBackdrop && !modalBackdrop.hidden) {
    closeModal();
  }
});

modalBody?.addEventListener("click", (event) => {
  const exceptionRow = event.target.closest("[data-open-exception-modal]");

  if (exceptionRow) {
    openExceptionDetail(exceptionRow.dataset.openExceptionModal);
    return;
  }

  const createPickupSlotButton = event.target.closest(
    "button[data-create-pickup-slot]",
  );

  if (createPickupSlotButton) {
    createPickupSlotButton.disabled = true;
    createPickupSlot().catch((error) => {
      alert(error.message);
      createPickupSlotButton.disabled = false;
    });
    return;
  }

  const openTaskCreateButton = event.target.closest(
    "button[data-open-task-create]",
  );

  if (openTaskCreateButton) {
    openTaskCreate().catch((error) => {
      alert(error.message);
    });
    return;
  }

  const createTaskButton = event.target.closest("button[data-create-task]");

  if (createTaskButton) {
    createTaskButton.disabled = true;
    createTask().catch((error) => {
      alert(error.message);
      createTaskButton.disabled = false;
    });
    return;
  }

  const deleteTaskButton = event.target.closest("button[data-delete-task]");

  if (deleteTaskButton) {
    deleteTaskButton.disabled = true;
    deleteTask(Number(deleteTaskButton.dataset.deleteTask))
      .then((deleted) => {
        if (!deleted) {
          deleteTaskButton.disabled = false;
        }
      })
      .catch((error) => {
        alert(error.message);
        deleteTaskButton.disabled = false;
      });
    return;
  }

  const saveOrderButton = event.target.closest("button[data-save-order-state]");

  if (saveOrderButton) {
    saveOrderButton.disabled = true;
    updateOrderState(Number(saveOrderButton.dataset.saveOrderState)).catch(
      (error) => {
        alert(error.message);
        saveOrderButton.disabled = false;
      },
    );
    return;
  }

  const saveTaskButton = event.target.closest("button[data-save-task-state]");

  if (saveTaskButton) {
    saveTaskButton.disabled = true;
    updateTaskState(Number(saveTaskButton.dataset.saveTaskState)).catch(
      (error) => {
        alert(error.message);
        saveTaskButton.disabled = false;
      },
    );
    return;
  }

  const saveRobotButton = event.target.closest("button[data-save-robot-state]");

  if (saveRobotButton) {
    saveRobotButton.disabled = true;
    updateRobotState(saveRobotButton.dataset.saveRobotState).catch((error) => {
      alert(error.message);
      saveRobotButton.disabled = false;
    });
    return;
  }

  const savePickupSlotButton = event.target.closest(
    "button[data-save-pickup-slot-state]",
  );

  if (savePickupSlotButton) {
    savePickupSlotButton.disabled = true;
    updatePickupSlotState(
      Number(savePickupSlotButton.dataset.savePickupSlotState),
    ).catch((error) => {
      alert(error.message);
      savePickupSlotButton.disabled = false;
    });
    return;
  }

  const saveProductButton = event.target.closest("button[data-save-product]");

  if (saveProductButton) {
    saveProductButton.disabled = true;
    updateProduct(Number(saveProductButton.dataset.saveProduct)).catch(
      (error) => {
        alert(error.message);
        saveProductButton.disabled = false;
      },
    );
    return;
  }

  const saveProductDetailButton = event.target.closest(
    "button[data-save-product-detail]",
  );

  if (saveProductDetailButton) {
    saveProductDetailButton.disabled = true;
    updateProduct(
      Number(saveProductDetailButton.dataset.saveProductDetail),
      "detail",
    ).catch((error) => {
      alert(error.message);
      saveProductDetailButton.disabled = false;
    });
    return;
  }

  const createProductButton = event.target.closest(
    "button[data-create-product]",
  );

  if (createProductButton) {
    createProductButton.disabled = true;
    createProduct().catch((error) => {
      alert(error.message);
      createProductButton.disabled = false;
    });
    return;
  }

  const stockButton = event.target.closest("button[data-save-stock]");

  if (stockButton) {
    const productId = Number(stockButton.dataset.saveStock);
    const stockInput = modalBody.querySelector(
      `input[data-stock-input="${productId}"]`,
    );
    const stockQty = Number(stockInput?.value);

    if (!Number.isInteger(stockQty) || stockQty < 0) {
      return;
    }

    stockButton.disabled = true;
    updateProductStock(productId, stockQty).catch(() => {
      stockButton.disabled = false;
    });
    return;
  }

  if (
    event.target.closest(
      "[data-task-history-status-filter]",
    )
  ) {
    return;
  }

  const button = event.target.closest("button[data-order-detail]");

  if (button) {
    openOrderDetail(Number(button.dataset.orderDetail));
    return;
  }

  const workButton = event.target.closest("button[data-work-history-detail]");

  if (workButton) {
    openOrderDetail(workButton.dataset.workHistoryDetail, {
      returnToCurrentModal: true,
    });
    return;
  }

  const taskButton = event.target.closest("[data-task-detail]");

  if (taskButton) {
    openTaskDetail(Number(taskButton.dataset.taskDetail), {
      returnToCurrentModal: true,
    });
    return;
  }
});

modalBody?.addEventListener("keydown", handleExceptionDetailKeydown);

modalBody?.addEventListener("input", (event) => {
  if (event.target.id !== "exception-history-search" || !latestAdminStatus) {
    return;
  }

  renderExceptionHistoryList(
    allExceptionsFromStatus(latestAdminStatus),
    event.target.value.trim(),
  );
});
