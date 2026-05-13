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

CREATE TYPE robot_status AS ENUM (
    'IDLE',
    'MOVING',
    'WAITING',
    'STANDBY',
    'SORTING',
    'LOADING',
    'PARKING',
    'INSPECTING',
    'UNLOADING',
    'PATROLLING',
    'CHARGING',
    'RETURNING',
    'DOCKING',
    'EMERGENCY_STOP',
    'ERROR',
    'OFFLINE'
);

CREATE TYPE task_type AS ENUM (
    'STANDBY_LOAD',
    'STANDBY_UNLOAD',
    'SORTING',
    'LOAD',
    'INSPECTION',
    'UNLOAD',
    'PATROL',
    'CHARGE',
    'RETURN_HOME'
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
    'SYSTEM_ERROR',
    'FIRE_DETECTED'
);

CREATE TABLE zone (
    zone_id SERIAL PRIMARY KEY,
    zone_name VARCHAR(50) NOT NULL,
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
    storage_location VARCHAR(50) NOT NULL
);

CREATE TABLE pickup_slot (
    slot_id SERIAL PRIMARY KEY,
    slot_name VARCHAR(50),
    status pickup_slot_status NOT NULL DEFAULT 'EMPTY'
);

CREATE TABLE orders (
    order_id SERIAL PRIMARY KEY,
    order_no VARCHAR(30) UNIQUE,
    status order_status NOT NULL DEFAULT 'ORDER_RECEIVED',
    priority INT NOT NULL DEFAULT 2,
    pickup_slot_id INT REFERENCES pickup_slot(slot_id)
);

CREATE TABLE order_item (
    item_id SERIAL PRIMARY KEY,
    order_id INT NOT NULL REFERENCES orders(order_id) ON DELETE CASCADE,
    product_id INT NOT NULL REFERENCES product(product_id),
    quantity INT NOT NULL CHECK (quantity > 0),
    status order_item_status NOT NULL DEFAULT 'WAITING'
);

CREATE TABLE robot (
    robot_id VARCHAR(30) PRIMARY KEY,
    status robot_status NOT NULL DEFAULT 'IDLE',
    current_task_id INT,
    ros_namespace VARCHAR(50),
    battery_level INT CHECK (battery_level BETWEEN 0 AND 100),
    pos_x FLOAT,
    pos_y FLOAT,
    pos_theta FLOAT
);

CREATE TABLE task (
    task_id SERIAL PRIMARY KEY,
    order_id INT REFERENCES orders(order_id) ON DELETE CASCADE,
    assigned_robot_id VARCHAR(30) REFERENCES robot(robot_id),
    task_type task_type NOT NULL,
    status task_status NOT NULL DEFAULT 'QUEUED',
    priority INT NOT NULL DEFAULT 1,
    source_zone_id INT REFERENCES zone(zone_id),
    target_zone_id INT REFERENCES zone(zone_id),
    result_message TEXT
);

ALTER TABLE robot
    ADD CONSTRAINT fk_robot_current_task
    FOREIGN KEY (current_task_id) REFERENCES task(task_id);

CREATE TABLE task_event (
    event_id SERIAL PRIMARY KEY,
    task_id INT NOT NULL REFERENCES task(task_id) ON DELETE CASCADE,
    robot_id VARCHAR(30) REFERENCES robot(robot_id),
    from_status task_status,
    to_status task_status NOT NULL,
    event_name VARCHAR(50),
    reason TEXT,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE TABLE exception_log (
    exception_id SERIAL PRIMARY KEY,
    robot_id VARCHAR(30) REFERENCES robot(robot_id),
    task_id INT REFERENCES task(task_id),
    order_id INT REFERENCES orders(order_id),
    exception_type exception_type NOT NULL,
    detail TEXT,
    is_resolved BOOLEAN NOT NULL DEFAULT FALSE,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_orders_status
ON orders(status);

CREATE INDEX idx_order_item_order_id
ON order_item(order_id);

CREATE INDEX idx_robot_status
ON robot(status);

CREATE INDEX idx_robot_current_task_id
ON robot(current_task_id);

CREATE INDEX idx_task_order_id
ON task(order_id);

CREATE INDEX idx_task_assigned_robot_id
ON task(assigned_robot_id);

CREATE INDEX idx_task_status
ON task(status);

CREATE INDEX idx_task_priority
ON task(priority);

CREATE INDEX idx_task_event_task_id
ON task_event(task_id);

CREATE INDEX idx_exception_log_robot_id
ON exception_log(robot_id);

CREATE INDEX idx_exception_log_task_id
ON exception_log(task_id);

CREATE INDEX idx_exception_log_order_id
ON exception_log(order_id);
