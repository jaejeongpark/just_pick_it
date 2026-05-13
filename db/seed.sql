-- Temporary development seed data. Replace coordinates and products after map/product details are fixed.

INSERT INTO zone (zone_name, pos_x, pos_y, pos_z, pos_theta) VALUES
    ('A_ZONE', 0.50, 0.70, 0.00, 0.00),
    ('LOADING_ZONE', 0.30, 0.40, 0.00, 0.00),
    ('STANDBY_LOADING_ZONE', 0.85, 0.40, 0.00, 0.00),
    ('STANDBY_UNLOADING_ZONE', 1.45, 0.40, 0.00, 3.14),
    ('UNLOADING_ZONE', 1.70, 0.40, 0.00, 3.14),
    ('HOME', 1.00, 0.50, 0.00, 0.00),
    ('CHARGING_ZONE', 1.00, 0.50, 0.00, 0.00),
    ('PRODUCT_ZONE', 0.20, 0.80, 0.10, NULL);

INSERT INTO product (name, image_url, stock_qty, storage_location) VALUES
    ('우유', '/static/images/우유.png', 2, 'PRODUCT_ZONE'),
    ('시리얼', '/static/images/시리얼.png', 2, 'PRODUCT_ZONE'),
    ('바나나 우유', '/static/images/바나나 우유.png', 2, 'PRODUCT_ZONE'),
    ('식빵', '/static/images/식빵.png', 2, 'PRODUCT_ZONE'),
    ('투게더', '/static/images/투게더.png', 2, 'PRODUCT_ZONE'),
    ('바나나', '/static/images/바나나', 2, 'PRODUCT_ZONE');

INSERT INTO pickup_slot (slot_name, status) VALUES
    ('Pickup_slot_1', 'EMPTY'),
    ('Pickup_slot_2', 'EMPTY');

INSERT INTO robot (robot_id, status, ros_namespace, battery_level, pos_x, pos_y, pos_theta) VALUES
    ('SORTING_COBOT', 'IDLE', '/sorting_cobot', NULL, NULL, NULL, NULL),
    ('INSPECTION_COBOT', 'IDLE', '/inspection_cobot', NULL, NULL, NULL, NULL),
    ('AMR_1', 'IDLE', '/amr_1', 100, 0.90, 0.82, 0.00),
    ('AMR_2', 'IDLE', '/amr_2', 100, 0.90, 0.08, 0.00);
