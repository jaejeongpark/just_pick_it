-- Temporary development seed data. Replace coordinates and products after map/product details are fixed.

INSERT INTO zone (zone_name, pos_x, pos_y, pos_z, pos_theta) VALUES
    ('LOADING_ZONE', 0.30, 0.40, 0.00, 0.00),
    ('LOADING_WAIT_ZONE', 0.85, 0.40, 0.00, 0.00),
    ('UNLOADING_ZONE', 1.70, 0.40, 0.00, 3.14),
    ('HOME', 1.00, 0.50, 0.00, 0.00),
    ('CHARGING_ZONE', 1.00, 0.50, 0.00, 0.00),
    ('PRODUCT_ZONE', 0.20, 0.80, 0.10, NULL);

INSERT INTO product (name, image_url, stock_qty, storage_location) VALUES
    ('Test Cola', '/static/images/test-cola.png', 5, 'PRODUCT_ZONE'),
    ('Test Snack', '/static/images/test-snack.png', 5, 'PRODUCT_ZONE'),
    ('Test Water', '/static/images/test-water.png', 5, 'PRODUCT_ZONE'),
    ('Test Candy', '/static/images/test-candy.png', 5, 'PRODUCT_ZONE'),
    ('Test Juice', '/static/images/test-juice.png', 5, 'PRODUCT_ZONE'),
    ('Test Cookie', '/static/images/test-cookie.png', 5, 'PRODUCT_ZONE');

INSERT INTO pickup_slot (slot_name, status) VALUES
    ('Pickup_slot_1', 'EMPTY'),
    ('Pickup_slot_2', 'EMPTY');

INSERT INTO robot (robot_id, status, ros_namespace, battery_level, pos_x, pos_y, pos_theta) VALUES
    ('COBOT1', 'IDLE', '/cobot1', NULL, NULL, NULL, NULL),
    ('COBOT2', 'IDLE', '/cobot2', NULL, NULL, NULL, NULL),
    ('AMR1', 'IDLE', '/amr1', 100, 0.90, 0.82, 0.00),
    ('AMR2', 'IDLE', '/amr2', 100, 0.90, 0.08, 0.00);
