CREATE TYPE order_status AS ENUM (
    'ORDER_RECEIVED',
    'ORDER_WAIT',
    'SORTING',
    'DELIVERING',
    'INSPECTING',
    'PICKUP_READY',
    'COMPLETED',
    'ERROR'
);

CREATE TYPE order_item_status AS ENUM (
    'WAITING',
    'SORTED',
    'INSPECTED',
    'MISSING',
    'EXCESS',
    'MISMATCH'
);

CREATE TYPE pickup_slot_status AS ENUM (
    'EMPTY',
    'RESERVED',
    'OCCUPIED',
    'BLOCKED'
);

CREATE TYPE robot_type AS ENUM (
    'PICKY',
    'COBOT'
);

CREATE TYPE robot_status AS ENUM (
    'OFFLINE',
    'IDLE',
    'BUSY',
    'CHARGING',
    'EMERGENCY_STOP',
    'ERROR'
);

CREATE TYPE picky_state AS ENUM (
    'CHARGING',
    'STANDBY',
    'MOVING_TO_PRODUCT',
    'WAITING_FOR_COBOT',
    'MOVING_TO_PICKUP',
    'MOVING_TO_STOCK',
    'MOVING_TO_DISPLAY',
    'RETURNING',
    'DOCKING',
    'ERROR_RECOVERY'
);

CREATE TYPE cobot_state AS ENUM (
    'STANDBY',
    'SORTING',
    'LOADING',
    'INSPECTING',
    'UNLOADING',
    'SCANNING',
    'PLACING',
    'STOWING_ARM',
    'SAFETY_STOPPED'
);

CREATE TYPE task_type AS ENUM (
    'MOVE_TO_PRODUCT',
    'SORTING_AND_LOAD',
    'MOVE_TO_PICKUP',
    'INSPECTION',
    'UNLOAD',
    'MOVE_TO_STOCK',
    'MOVE_TO_DISPLAY',
    'DISPLAY_SCAN',
    'DISPLAY_PLACE',
    'RETURN_HOME',
    'DOCK_IN',
    'CHARGE'
);

CREATE TYPE task_status AS ENUM (
    'QUEUED',
    'ASSIGNED',
    'RUNNING',
    'PAUSED',
    'SUCCESS',
    'FAILED',
    'CANCELLED'
);

CREATE TYPE exception_type AS ENUM (
    'OBSTACLE_DETECTED',
    'LOW_BATTERY',
    'NAVIGATION_FAILED',
    'HARDWARE_ERROR',
    'TIMEOUT',
    'SORTING_FAIL',
    'INSPECTION_FAIL',
    'HUMAN_DETECTED',
    'SYSTEM_ERROR'
);

CREATE TYPE display_policy AS ENUM (
    'REQUESTED_QUANTITY',
    'ALL_PROCESSED'
);

CREATE TYPE display_item_status AS ENUM (
    'REQUESTED',
    'ASSIGNED',
    'IN_PROGRESS',
    'COMPLETED',
    'FAILED',
    'CANCELLED'
);

CREATE TABLE zone (
    zone_id SERIAL PRIMARY KEY,
    zone_name VARCHAR(50) NOT NULL,
    zone_type VARCHAR(30) NOT NULL,
    pos_x FLOAT NOT NULL,
    pos_y FLOAT NOT NULL,
    pos_z FLOAT NOT NULL,
    pos_theta FLOAT
);

CREATE TABLE product (
    product_id SERIAL PRIMARY KEY,
    name VARCHAR(100) NOT NULL,
    image_url TEXT,
    stock_qty INT NOT NULL DEFAULT 0 CHECK (stock_qty >= 0),
    storage_zone_id INT NOT NULL REFERENCES zone(zone_id)
);

CREATE TABLE pickup_slot (
    slot_id SERIAL PRIMARY KEY,
    slot_name VARCHAR(50),
    status pickup_slot_status NOT NULL DEFAULT 'EMPTY'
);

CREATE TABLE robot_unit (
    unit_id SERIAL PRIMARY KEY,
    unit_name VARCHAR(50) NOT NULL,
    description TEXT
);

CREATE TABLE orders (
    order_id SERIAL PRIMARY KEY,
    order_no VARCHAR(30) UNIQUE,
    status order_status NOT NULL DEFAULT 'ORDER_RECEIVED',
    priority INT NOT NULL DEFAULT 2,
    pickup_slot_id INT REFERENCES pickup_slot(slot_id),
    assigned_unit_id INT REFERENCES robot_unit(unit_id)
);

CREATE TABLE order_item (
    item_id SERIAL PRIMARY KEY,
    order_id INT NOT NULL REFERENCES orders(order_id) ON DELETE CASCADE,
    product_id INT NOT NULL REFERENCES product(product_id),
    quantity INT NOT NULL CHECK (quantity > 0),
    status order_item_status NOT NULL DEFAULT 'WAITING'
);

CREATE TABLE display_item (
    display_item_id SERIAL PRIMARY KEY,
    product_id INT NOT NULL REFERENCES product(product_id),
    requested_quantity INT CHECK (requested_quantity IS NULL OR requested_quantity > 0),
    processed_quantity INT CHECK (processed_quantity IS NULL OR processed_quantity >= 0),
    stock_delta INT CHECK (stock_delta IS NULL OR stock_delta >= 0),
    display_policy display_policy NOT NULL,
    status display_item_status NOT NULL DEFAULT 'REQUESTED',
    assigned_unit_id INT REFERENCES robot_unit(unit_id),
    CHECK (
        (display_policy = 'REQUESTED_QUANTITY' AND requested_quantity IS NOT NULL)
        OR
        (display_policy = 'ALL_PROCESSED' AND requested_quantity IS NULL)
    )
);

CREATE TABLE robot (
    robot_id SERIAL PRIMARY KEY,
    robot_name VARCHAR(30) UNIQUE NOT NULL,
    unit_id INT REFERENCES robot_unit(unit_id),
    robot_type robot_type NOT NULL,
    robot_status robot_status NOT NULL DEFAULT 'IDLE',
    picky_state picky_state,
    cobot_state cobot_state,
    current_task_id INT,
    ros_namespace VARCHAR(50),
    battery_level INT CHECK (battery_level BETWEEN 0 AND 100),
    pos_x FLOAT,
    pos_y FLOAT,
    pos_theta FLOAT,
    CHECK (
        (robot_type = 'PICKY' AND cobot_state IS NULL)
        OR
        (robot_type = 'COBOT' AND picky_state IS NULL)
    )
);

CREATE TABLE task (
    task_id SERIAL PRIMARY KEY,
    order_id INT REFERENCES orders(order_id) ON DELETE CASCADE,
    order_item_id INT REFERENCES order_item(item_id) ON DELETE CASCADE,
    display_item_id INT REFERENCES display_item(display_item_id) ON DELETE CASCADE,
    sequence_no INT NOT NULL,
    assigned_robot_id INT REFERENCES robot(robot_id),
    task_type task_type NOT NULL,
    status task_status NOT NULL DEFAULT 'QUEUED',
    priority INT NOT NULL DEFAULT 2,
    source_zone_id INT REFERENCES zone(zone_id),
    target_zone_id INT REFERENCES zone(zone_id),
    result_message TEXT,
    CHECK (
        NOT (order_item_id IS NOT NULL AND display_item_id IS NOT NULL)
    ),
    CHECK (
        display_item_id IS NULL
        OR
        (order_id IS NULL AND order_item_id IS NULL)
    )
);

ALTER TABLE robot
    ADD CONSTRAINT fk_robot_current_task
    FOREIGN KEY (current_task_id) REFERENCES task(task_id);

CREATE TABLE task_event (
    event_id SERIAL PRIMARY KEY,
    task_id INT NOT NULL REFERENCES task(task_id) ON DELETE CASCADE,
    robot_id INT REFERENCES robot(robot_id),
    from_status task_status,
    to_status task_status NOT NULL,
    event_name VARCHAR(50),
    reason TEXT,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE TABLE exception_log (
    exception_id SERIAL PRIMARY KEY,
    robot_id INT REFERENCES robot(robot_id),
    task_id INT REFERENCES task(task_id),
    order_id INT REFERENCES orders(order_id),
    exception_type exception_type NOT NULL,
    detail TEXT,
    is_resolved BOOLEAN NOT NULL DEFAULT FALSE,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_orders_status
ON orders(status);

CREATE INDEX idx_orders_assigned_unit_id
ON orders(assigned_unit_id);

CREATE INDEX idx_order_item_order_id
ON order_item(order_id);

CREATE INDEX idx_order_item_product_id
ON order_item(product_id);

CREATE INDEX idx_display_item_product_id
ON display_item(product_id);

CREATE INDEX idx_display_item_status
ON display_item(status);

CREATE INDEX idx_display_item_assigned_unit_id
ON display_item(assigned_unit_id);

CREATE INDEX idx_robot_unit_id
ON robot(unit_id);

CREATE INDEX idx_robot_type_status
ON robot(robot_type, robot_status);

CREATE INDEX idx_robot_current_task_id
ON robot(current_task_id);

CREATE INDEX idx_task_order_id
ON task(order_id);

CREATE INDEX idx_task_order_item_id
ON task(order_item_id);

CREATE INDEX idx_task_display_item_id
ON task(display_item_id);

CREATE INDEX idx_task_assigned_robot_id
ON task(assigned_robot_id);

CREATE INDEX idx_task_status_priority
ON task(status, priority);

CREATE INDEX idx_task_event_task_id
ON task_event(task_id);

CREATE INDEX idx_exception_log_robot_id
ON exception_log(robot_id);

CREATE INDEX idx_exception_log_task_id
ON exception_log(task_id);

CREATE INDEX idx_exception_log_order_id
ON exception_log(order_id);
