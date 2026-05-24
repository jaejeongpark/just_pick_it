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
let selectedTaskId = null;
let zoneOptionsCache = null;
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
const TASK_TYPE_SEQUENCE = [
  "MOVE_TO_PRODUCT",
  "SORTING_AND_LOAD",
  "MOVE_TO_PICKUP",
  "INSPECTION",
  "UNLOAD",
  "MOVE_TO_STOCK",
  "STOCKING_PICK",
  "MOVE_TO_STORAGE",
  "STOCKING_PLACE",
  "RETURN_HOME",
  "DOCK_IN",
  "CHARGE",
];
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
  "MOVING_TO_STORAGE",
  "RETURNING",
  "DOCKING",
  "ERROR_RECOVERY",
];
const COBOT_STATES = [
  "STANDBY",
  "SORTING",
  "LOADING",
  "INSPECTING",
  "UNLOADING",
  "STOCKING_SORTING",
  "STOCKING_LOADING",
  "STOCKING_PLACING",
  "STOWING_ARM",
  "SAFETY_STOPPED",
];
const PICKUP_SLOT_STATUSES = ["EMPTY", "RESERVED", "OCCUPIED", "BLOCKED"];
const STOCK_LEVEL_LABELS = {
  low: "부족",
  warning: "부족 임박",
  normal: "정상",
};
const STOCK_LEVEL_CLASSES = {
  low: "table-danger",
  warning: "table-warning",
  normal: "table-ok",
};

const statusText = {
  ORDER_RECEIVED: "주문 접수",
  ORDER_WAIT: "주문 대기",
  SORTING: "선별/상차 중",
  MOVE_TO_PRODUCT: "상품 위치 이동",
  SORTING_AND_LOAD: "선별/상차",
  MOVE_TO_PICKUP: "픽업존 이동",
  INSPECTION: "검수",
  UNLOAD: "하차",
  MOVE_TO_STOCK: "입고존 이동",
  STOCKING_PICK: "입고 선별/상차",
  MOVE_TO_STORAGE: "적재 위치 이동",
  STOCKING_PLACE: "입고상품 적재",
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
  MOVING_TO_STOCK: "입고존 이동 중",
  MOVING_TO_STORAGE: "적재 위치 이동 중",
  STANDBY: "대기",
  LOADING: "상차 중",
  UNLOADING: "하차 중",
  STOCKING_SORTING: "입고상품 선별 중",
  STOCKING_LOADING: "입고상품 상차 중",
  STOCKING_PLACING: "입고상품 적재 중",
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
  STOCKING_SORTING: "입고 상품 선별",
  STOCKING_LOADING: "입고 상품 상차",
  STOCKING_PLACING: "입고상품 적재 중",
  STOWING_ARM: "팔 기본 자세 복귀",
};

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

function robotStateTextLabel(value) {
  return robotStateText[value] || label(value);
}

function normalizeId(value) {
  return value === null || value === undefined ? "" : String(value);
}

function sameId(left, right) {
  return normalizeId(left) === normalizeId(right);
}

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

  return (
    task?.product_name ||
    orderItem?.product_name ||
    null
  );
}

function taskDisplayTitle(task) {
  const productName = taskProductName(task);

  if (!productName) {
    return label(task?.task_type);
  }

  const productTaskLabels = {
    MOVE_TO_PRODUCT: `${productName} 위치 이동`,
    SORTING_AND_LOAD: `${productName} 선별/상차`,
    MOVE_TO_STOCK: `${productName} 입고존 이동`,
    STOCKING_PICK: `${productName} 입고 선별/상차`,
    MOVE_TO_STORAGE: `${productName} 적재 위치 이동`,
    STOCKING_PLACE: `${productName} 입고상품 적재`,
  };

  return productTaskLabels[task.task_type] || label(task.task_type);
}

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

function openModal(title, body, options = {}) {
  modalTitle.textContent = title;
  modalBody.innerHTML = body;
  modalPanel?.classList.toggle("modal-compact", options.size === "compact");
  modalBackdrop.hidden = false;
}

function closeModal() {
  modalBackdrop.hidden = true;
  modalPanel?.classList.remove("modal-compact");
}

function renderOptions(
  values,
  selectedValue,
  emptyLabel = null,
  labelFormatter = label,
) {
  const isEmptySelected =
    selectedValue === null || selectedValue === undefined || selectedValue === "";
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
          #${task.task_id} ${taskDisplayTitle(task)}
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

function renderOrderDetail(order) {
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
    ${renderOrderItems(order.items)}
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

function findOrder(orderId) {
  return (latestAdminStatus?.orders || []).find(
    (order) => sameId(order.order_id, orderId),
  );
}

function findTask(taskId) {
  return (latestAdminStatus?.tasks || []).find(
    (task) => sameId(task.task_id, taskId),
  );
}

function orderTasks(order) {
  if (!order) {
    return [];
  }

  return (latestAdminStatus?.tasks || [])
    .filter(
      (task) =>
        task.order_id === order.order_id || task.order_no === order.order_no,
    )
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
  return orderTasks(order);
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

  const orders = data.orders || [];
  const tasks = data.tasks || [];

  if (
    selectedTaskId !== null &&
    !tasks.some((task) => sameId(task.task_id, selectedTaskId))
  ) {
    selectedTaskId = null;
  }

  if (
    selectedOrderId !== null &&
    !orders.some((order) => sameId(order.order_id, selectedOrderId))
  ) {
    selectedOrderId = null;
  }

  if (selectedOrderId === null && orders.length > 0) {
    selectedOrderId = orders[0].order_id;
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
  const order = selectedTask?.order_id
    ? findOrder(selectedTask.order_id)
    : findOrder(selectedOrderId);

  if (!order) {
    orderWorkDetailPanel.innerHTML =
      '<div class="empty-state">선택된 주문이 없습니다</div>';
    return;
  }

  const tasks = taskQueueForOrder(order);
  const activeTask =
    selectedTask &&
    (selectedTask.order_id === order.order_id ||
      selectedTask.order_no === order.order_no)
      ? selectedTask
      : tasks.find((task) => ["RUNNING", "ASSIGNED"].includes(task.status)) ||
        tasks[0] ||
        null;

  orderWorkDetailPanel.innerHTML = `
    <div class="work-detail-header">
      <div>
        <span>${activeTask?.task_id ? `Task #${activeTask.task_id}` : "주문 상세"}</span>
        <strong>${order.order_no}${activeTask ? ` · ${taskDisplayTitle(activeTask)}` : ""}</strong>
      </div>
      <div class="work-detail-status">
        <span class="state-badge ${statusClass(activeTask?.status || order.status)}">${label(activeTask?.status || order.status)}</span>
        ${renderMiniProgress(orderProgress(order.status), order.status)}
      </div>
    </div>
    <div class="work-detail-grid">
      <div class="work-detail-block">
        <h3>주문 정보</h3>
        <dl>
          <div><dt>주문번호</dt><dd>${order.order_no}</dd></div>
          <div><dt>주문 상태</dt><dd>${label(order.status)}</dd></div>
          <div><dt>픽업칸</dt><dd>${formatPickupSlot(order.pickup_slot_name)}</dd></div>
        </dl>
      </div>
      <div class="work-detail-block">
        <h3>상품</h3>
        <div class="work-detail-items">
          ${order.items.length === 0 ? '<span class="muted">상품 없음</span>' : order.items.map(orderItemCard).join("")}
        </div>
      </div>
      <div class="work-detail-block">
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
      <div class="work-detail-actions">
        <button class="ghost-button" type="button" data-open-order-modal="${order.order_id}">주문 수정</button>
        ${activeTask?.task_id ? `<button class="ghost-button" type="button" data-open-task-modal="${activeTask.task_id}">작업 수정</button>` : ""}
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

function mapRobotPosition(robot) {
  if (robot.pos_x !== null && robot.pos_y !== null) {
    const x = 15 + (Number(robot.pos_x) / 1.8) * 70;
    const y = 78 - (Number(robot.pos_y) / 1.0) * 60;
    const offset = robot.robot_name === "PICKY1" ? -1.8 : robot.robot_name === "PICKY2" ? 1.8 : 0;

    return {
      x: clampNumber(x + offset, 10, 90),
      y: clampNumber(y + offset, 12, 88),
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

  const pickyRobots = robots.filter(
    (robot) =>
      robotType(robot) === "PICKY" &&
      !normalizeId(robot.robot_name).startsWith("UI_"),
  );

  mapRobotLayer.innerHTML = pickyRobots
    .map((robot) => {
      const position = mapRobotPosition(robot);
      const markerClass = "map-marker-amr";
      const displayName = robotDisplayName(robot);
      const status = robotStatusValue(robot);

      return `
        <div class="robot-map-marker ${markerClass} ${robotColorClass(robot.robot_id)}"
          style="--marker-x: ${position.x}%; --marker-y: ${position.y}%; --heading: ${robotHeadingDeg(robot)}deg"
          title="${displayName} · ${label(status)}">
          <i class="marker-heading"></i>
          <span>${displayName}</span>
        </div>
      `;
    })
    .join("");
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

  return "normal";
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

function renderMiniProgress(value, status) {
  const progress = Math.max(0, Math.min(100, Number(value) || 0));

  return `
    <div class="mini-progress ${statusClass(status)}" style="--progress: ${progress}%"></div>
  `;
}

function robotColorClass(robotId) {
  const robot = findRobotById(robotId);
  const robotName = robot?.robot_name || normalizeId(robotId);

  if (robot?.unit_id === 1 || robotName === "PICKY1" || robotName === "COBOT1") {
    return "robot-dot-amr1";
  }

  if (robot?.unit_id === 2 || robotName === "PICKY2" || robotName === "COBOT2") {
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
    ? `${task.order_no || `Task #${task.task_id}`} · ${taskDisplayTitle(task)}`
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
              <span class="task-cell">${task ? `${task.order_no || `Task #${task.task_id}`} · ${taskDisplayTitle(task)}` : "-"}</span>
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
              <span class="task-cell">${task ? `${task.order_no || `Task #${task.task_id}`} · ${taskDisplayTitle(task)}` : "-"}</span>
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

  if (orders.length === 0) {
    renderEmpty(orderList, "주문이 없습니다");
    return;
  }

  orderList.innerHTML = `
    <div class="admin-table order-table">
      <div class="admin-table-head">
        <span>주문번호</span>
        <span>상품</span>
        <span>상태</span>
        <span>픽업칸</span>
        <span>진행률</span>
      </div>
      ${orders
        .map((order) => {
          const linkedTaskSelected =
            selectedTaskId !== null &&
            orderTasks(order).some((task) => sameId(task.task_id, selectedTaskId));
          const isSelected =
            adminPage === "orders" &&
            (sameId(order.order_id, selectedOrderId) || linkedTaskSelected);

          return `
            <button class="admin-table-row order-table-row ${isSelected ? "is-selected" : ""}" type="button" data-order-detail="${order.order_id}">
              <span><strong>${order.order_no}</strong></span>
              <span>${orderProductSummary(order)}</span>
              <span><span class="state-badge ${statusClass(order.status)}">${label(order.status)}</span></span>
              <span>${formatPickupSlot(order.pickup_slot_name)}</span>
              <span>${renderMiniProgress(orderProgress(order.status), order.status)}</span>
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
    const warningEnd =
      normalEnd + (stockCounts.warning / totalProducts) * 100;

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
    renderOrderSnapshot(latestAdminStatus?.orders || []);
    return;
  }

  if (!tasks || tasks.length === 0) {
    renderEmpty(taskList, "작업이 없습니다");
    return;
  }

  const tasksToRender = tasks;

  taskList.innerHTML = `
    <div class="admin-table task-table">
      <div class="admin-table-head">
        <span>작업 ID</span>
        <span>유형</span>
        <span>주문</span>
        <span>로봇</span>
        <span>상태</span>
      </div>
      ${tasksToRender
        .map(
          (task) => `
          <button class="admin-table-row task-table-row ${adminPage === "orders" && sameId(task.task_id, selectedTaskId) ? "is-selected" : ""}" type="button" data-task-detail="${task.task_id}">
            <span>#${task.task_id}</span>
            <span>${taskDisplayTitle(task)}</span>
            <span>${task.order_no || "주문 없음"}</span>
            <span>${assignedRobotLabel(task)}</span>
            <span><span class="state-badge ${statusClass(task.status)}">${label(task.status)}</span></span>
          </button>
        `,
        )
        .join("")}
    </div>
  `;
}

function renderOrderSnapshot(orders) {
  if (!taskList) {
    return;
  }

  const activeOrders = orders
    .filter((order) => !["COMPLETED", "ERROR"].includes(order.status))
    .slice(0, 5);

  if (activeOrders.length === 0) {
    renderEmpty(taskList, "진행 중인 주문이 없습니다");
    return;
  }

  taskList.innerHTML = `
    <div class="admin-table dashboard-order-table">
      <div class="admin-table-head">
        <span>주문번호</span>
        <span>상품</span>
        <span>현재 단계</span>
        <span>진행률</span>
      </div>
      ${activeOrders
        .map(
          (order) => `
          <button class="admin-table-row dashboard-order-row" type="button" data-order-detail="${order.order_id}">
            <span><strong>${order.order_no}</strong></span>
            <span>${orderProductSummaryMarkup(order)}</span>
            <span><span class="state-badge ${statusClass(order.status)}">${label(order.status)}</span></span>
            <span>${renderMiniProgress(orderProgress(order.status), order.status)}</span>
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
          <div class="admin-table-row exception-table-row ${exception.is_resolved ? "" : "danger-row"}">
            <span>${formatDateTime(exception.created_at)}</span>
            <span>${exception.robot_name || exception.robot_id || "-"}</span>
            <span class="${exception.is_resolved ? "" : "table-danger"}">${exception.detail || exception.exception_type}</span>
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
      <div class="history-row ${exception.is_resolved ? "" : "danger-row"}">
        <div>
          <strong>${exception.exception_type}</strong>
          <span>${exception.detail || "상세 없음"}</span>
          <span>${formatDateTime(exception.created_at)} · ${exception.robot_name || exception.robot_id || "로봇 미지정"}</span>
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
                <span>${robot.current_task_id ? `Task #${robot.current_task_id}` : "작업 없음"}</span>
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
        <span>Task</span>
        <strong>#${task.task_id}</strong>
      </div>
      <div>
        <span>작업</span>
        <strong>${taskDisplayTitle(task)}</strong>
      </div>
      <div>
        <span>상태</span>
        <strong>${label(task.status)}</strong>
      </div>
    </div>
    <div class="modal-summary">
      <div>
        <span>주문</span>
        <strong>${task.order_no || "주문 없음"}</strong>
      </div>
      <div>
        <span>로봇</span>
        <strong>${assignedRobotLabel(task)}</strong>
      </div>
      <div>
        <span>결과</span>
        <strong>${task.result_message || "-"}</strong>
      </div>
    </div>
    <div class="state-editor-form">
      <div>
        <label for="task-status-select">작업 상태</label>
        <select id="task-status-select">${renderOptions(TASK_STATUSES, task.status)}</select>
      </div>
      <div>
        <label for="task-robot-select">할당 로봇</label>
        <select id="task-robot-select">${renderRobotOptions(task.assigned_robot_id)}</select>
      </div>
      <button class="small-action-button" type="button" data-save-task-state="${task.task_id}">상태 저장</button>
    </div>
    <div class="modal-actions">
      <button
        class="danger-button"
        type="button"
        data-delete-task="${task.task_id}"
        ${deleteBlocked ? "disabled" : ""}
      >
        작업 삭제
      </button>
      ${deleteBlocked ? '<span class="muted">RUNNING/PAUSED 작업은 먼저 상태를 바꾼 뒤 삭제합니다.</span>' : ""}
    </div>
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
        <label for="new-task-order-id">order_id</label>
        <input id="new-task-order-id" type="number" min="1" placeholder="없으면 비움">
      </div>
      <div>
        <label for="new-task-order-item-id">order_item_id</label>
        <input id="new-task-order-item-id" type="number" min="1" placeholder="없으면 비움">
      </div>
      <div>
        <label for="new-task-stocking-item-id">stocking_item_id</label>
        <input id="new-task-stocking-item-id" type="number" min="1" placeholder="없으면 비움">
      </div>
      <div>
        <label for="new-task-priority">priority</label>
        <input id="new-task-priority" type="number" min="1" value="2">
      </div>
      <div>
        <label for="new-task-source-zone-id">출발 zone</label>
        <select id="new-task-source-zone-id">${renderZoneOptions(zones, null)}</select>
      </div>
      <div>
        <label for="new-task-target-zone-id">목표 zone</label>
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

function renderTaskManager(tasks) {
  return `
    <div class="modal-actions">
      <button class="small-action-button" type="button" data-open-task-create>작업 생성</button>
    </div>
    <div class="task-queue-list">
      ${
        !tasks || tasks.length === 0
          ? '<div class="empty-state">작업이 없습니다</div>'
          : tasks
              .map(
                (task) => `
          <button class="task-queue-row data-button" type="button" data-task-detail="${task.task_id}">
            <div class="queue-rank">#${task.task_id}</div>
            <div class="task-main">
              <div class="task-title-line">
                <strong>${taskDisplayTitle(task)}</strong>
                <span>${task.order_no || "주문 없음"}</span>
              </div>
              <span>${assignedRobotLabel(task)}</span>
            </div>
            <div class="task-side">
              <div class="state-badge ${statusClass(task.status)}">${label(task.status)}</div>
            </div>
          </button>
        `,
              )
              .join("")
      }
    </div>
  `;
}

function openTaskManager() {
  if (!latestAdminStatus) {
    return;
  }

  openModal(
    "Task Management",
    renderTaskManager(latestAdminStatus.tasks || []),
  );
}

async function openTaskCreate() {
  const zones = await loadZoneOptions();
  openModal("Task 생성", renderTaskCreateForm(zones));
}

function openTaskDetail(taskId) {
  if (!latestAdminStatus) {
    return;
  }

  const task = (latestAdminStatus.tasks || []).find(
    (item) => item.task_id === taskId,
  );

  if (!task) {
    return;
  }

  openModal(`Task #${task.task_id}`, renderTaskDetail(task));
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
    summaryOrders.textContent = String(data.orders.length);
  }

  if (summaryExceptions) {
    summaryExceptions.textContent = String(
      data.unresolved_exception_count ?? data.exceptions.length,
    );
  }

  if (summaryTasks) {
    summaryTasks.textContent = String(
      data.orders.filter(
        (order) => !["COMPLETED", "ERROR"].includes(order.status),
      ).length,
    );
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
                <span>Task #${task.task_id}</span>
              </div>
              <span>${task.order_no || "주문 없음"}</span>
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

  openModal(`${robotDisplayName(robot)} Task Queue`, renderRobotTaskQueue(robot));
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

function openOrderDetail(orderId) {
  if (!latestAdminStatus) {
    return;
  }

  const order = [
    ...latestAdminStatus.orders,
    ...latestAdminStatus.order_history,
  ].find((item) => item.order_id === orderId);

  if (!order) {
    return;
  }

  openModal(order.order_no, renderOrderDetail(order));
}

function openOrderHistory() {
  if (!latestAdminStatus) {
    return;
  }

  const orders = latestAdminStatus.order_history;

  const body =
    orders.length === 0
      ? '<div class="empty-state">완료된 주문이 없습니다</div>'
      : `
      <div class="history-list">
        ${orders
          .map(
            (order) => `
            <button class="history-row" type="button" data-order-detail="${order.order_id}">
              <div>
                <strong>${order.order_no}</strong>
                <span>상품 ${order.items.length}종 · ${formatPickupSlot(order.pickup_slot_name)}</span>
              </div>
              <div class="history-status-large">${label(order.status)}</div>
            </button>
          `,
          )
          .join("")}
      </div>
    `;

  openModal("Order History", body);
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

  openModal("Exception History", body);
  renderExceptionHistoryList(exceptions);
}

async function loadAdminStatus() {
  try {
    const response = await fetch("/api/admin/status");
    if (!response.ok) {
      throw new Error("failed to load admin status");
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
loadAdminStatus();
connectAdminSocket();

async function postAdminAction(path) {
  const response = await fetch(path, { method: "POST" });

  if (!response.ok) {
    throw new Error("admin action failed");
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
    throw await errorFromResponse(response, "create request failed");
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
    throw await errorFromResponse(response, "state update failed");
  }

  await loadAdminStatus();
}

async function deleteJson(path) {
  const response = await fetch(path, { method: "DELETE" });

  if (!response.ok) {
    throw await errorFromResponse(response, "delete request failed");
  }

  await loadAdminStatus();
}

async function loadZoneOptions() {
  if (zoneOptionsCache) {
    return zoneOptionsCache;
  }

  const response = await fetch("/api/fleet/zones?zone_type=ALL");

  if (!response.ok) {
    throw await errorFromResponse(response, "zone list load failed");
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

function taskIntegerOrNull(selector, fieldName, { required = false, min = null } = {}) {
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
  const resultMessage = modalBody
    .querySelector("#new-task-result-message")
    ?.value.trim();
  const task = {
    task_type: modalBody.querySelector("#new-task-type")?.value,
    status: modalBody.querySelector("#new-task-status")?.value,
    assigned_robot_id: selectNumberOrNull("#new-task-robot-select"),
    order_id: taskIntegerOrNull("#new-task-order-id", "order_id", { min: 1 }),
    order_item_id: taskIntegerOrNull("#new-task-order-item-id", "order_item_id", { min: 1 }),
    stocking_item_id: taskIntegerOrNull("#new-task-stocking-item-id", "stocking_item_id", { min: 1 }),
    priority: taskIntegerOrNull("#new-task-priority", "priority", {
      required: true,
      min: 1,
    }),
    source_zone_id: selectNumberOrNull("#new-task-source-zone-id"),
    target_zone_id: selectNumberOrNull("#new-task-target-zone-id"),
    result_message: resultMessage || null,
  };

  if (task.stocking_item_id && (task.order_id || task.order_item_id)) {
    throw new Error("입고 작업은 order_id/order_item_id와 같이 만들 수 없습니다.");
  }

  await postJson("/api/fleet/tasks/bulk", { tasks: [task] });
  openTaskManager();
}

async function deleteTask(taskId) {
  const task = findTask(taskId);

  if (!task) {
    return false;
  }

  if (!confirm(`Task #${task.task_id} ${taskDisplayTitle(task)} 작업을 삭제할까요?`)) {
    return false;
  }

  await deleteJson(`/api/fleet/tasks/${taskId}`);
  selectedTaskId = null;
  openTaskManager();
  return true;
}

async function updateOrderState(orderId) {
  await patchJson(`/api/fleet/orders/${orderId}`, {
    status: modalBody.querySelector("#order-status-select")?.value,
    pickup_slot_id: selectNumberOrNull("#order-pickup-slot-select"),
  });
  openOrderDetail(orderId);
}

async function updateTaskState(taskId) {
  const assignedRobotId =
    modalBody.querySelector("#task-robot-select")?.value || null;

  await patchJson(`/api/fleet/tasks/${taskId}`, {
    status: modalBody.querySelector("#task-status-select")?.value,
    assigned_robot_id: assignedRobotId ? Number(assignedRobotId) : null,
  });
  openTaskDetail(taskId);
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
    throw new Error("failed to update stock");
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
    throw new Error("failed to update product");
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
    throw new Error("failed to create product");
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
    throw new Error("failed to create pickup slot");
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
    lowerCommand.includes("입고") ||
    lowerCommand.includes("stocking") ||
    lowerCommand.includes("stock in")
  ) {
    return "입고 명령을 처리하지 못했습니다. AI 메시지 API와 Fleet Manager 상태를 확인해주세요.";
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
    throw new Error("failed to send llm message");
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

  if (!button) {
    return;
  }

  button.disabled = true;
  const exceptionId = button.dataset.resolveException;

  try {
    await postAdminAction(`/api/admin/exceptions/${exceptionId}/resolve`);
  } finally {
    button.disabled = false;
  }
});

orderList?.addEventListener("click", (event) => {
  const button = event.target.closest("button[data-order-detail]");

  if (!button) {
    return;
  }

  if (adminPage === "orders") {
    selectedOrderId = Number(button.dataset.orderDetail);
    selectedTaskId = null;
    renderOrders(latestAdminStatus?.orders || []);
    renderTaskSnapshot(latestAdminStatus?.tasks || []);
    renderOrderWorkDetail();
  }

  openOrderDetail(Number(button.dataset.orderDetail));
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
  const orderButton = event.target.closest("button[data-order-detail]");

  if (orderButton) {
    if (adminPage === "orders") {
      selectedOrderId = Number(orderButton.dataset.orderDetail);
      selectedTaskId = null;
      renderOrders(latestAdminStatus?.orders || []);
      renderTaskSnapshot(latestAdminStatus?.tasks || []);
      renderOrderWorkDetail();
      return;
    }

    openOrderDetail(Number(orderButton.dataset.orderDetail));
    return;
  }

  const button = event.target.closest("button[data-task-detail]");

  if (!button) {
    return;
  }

  if (adminPage === "orders") {
    const taskId = Number(button.dataset.taskDetail);
    const task = findTask(taskId);
    const order = task?.order_id
      ? findOrder(task.order_id)
      : (latestAdminStatus?.orders || []).find(
          (item) => item.order_no === task?.order_no,
        );
    selectedTaskId = taskId;
    selectedOrderId = order?.order_id || selectedOrderId;
    renderOrders(latestAdminStatus?.orders || []);
    renderTaskSnapshot(latestAdminStatus?.tasks || []);
    renderOrderWorkDetail();
    return;
  }

  openTaskDetail(Number(button.dataset.taskDetail));
});

orderWorkDetailPanel?.addEventListener("click", (event) => {
  const taskButton = event.target.closest("button[data-work-task]");

  if (taskButton) {
    const taskId = Number(taskButton.dataset.workTask);
    const task = findTask(taskId);
    const order = task?.order_id
      ? findOrder(task.order_id)
      : (latestAdminStatus?.orders || []).find(
          (item) => item.order_no === task?.order_no,
        );
    selectedTaskId = taskId;
    selectedOrderId = order?.order_id || selectedOrderId;
    renderOrders(latestAdminStatus?.orders || []);
    renderTaskSnapshot(latestAdminStatus?.tasks || []);
    renderOrderWorkDetail();
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

  const openTaskCreateButton = event.target.closest("button[data-open-task-create]");

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

  const button = event.target.closest("button[data-order-detail]");

  if (button) {
    openOrderDetail(Number(button.dataset.orderDetail));
    return;
  }

  const taskButton = event.target.closest("button[data-task-detail]");

  if (taskButton) {
    openTaskDetail(Number(taskButton.dataset.taskDetail));
  }
});

modalBody?.addEventListener("input", (event) => {
  if (event.target.id !== "exception-history-search" || !latestAdminStatus) {
    return;
  }

  renderExceptionHistoryList(
    allExceptionsFromStatus(latestAdminStatus),
    event.target.value.trim(),
  );
});
