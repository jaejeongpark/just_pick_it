-- Temporary development seed data. Replace coordinates after the final map is fixed.
-- *_ZONE_* means a PICKY navigation/parking pose.
-- *_SLOT_* means a physical product/pickup pose for COBOT work.

INSERT INTO zone (zone_name, zone_type, pos_x, pos_y, pos_z, pos_theta) VALUES
    ('STANDBY_ZONE_1', 'STANDBY', 0.90, 0.82, 0.00, 0.00),
    ('STANDBY_ZONE_2', 'STANDBY', 0.90, 0.08, 0.00, 0.00),
    ('STOCK_ZONE', 'STOCK', 0.15, 0.45, 0.00, 0.00),
    ('STOCK_SLOT', 'STOCK_SLOT', 0.08, 0.45, 0.35, 0.00),
    ('PRODUCT_ZONE_1', 'PRODUCT', 0.20, 0.80, 0.00, 0.00),
    ('PRODUCT_ZONE_2', 'PRODUCT', 0.32, 0.80, 0.00, 0.00),
    ('PRODUCT_ZONE_3', 'PRODUCT', 0.44, 0.80, 0.00, 0.00),
    ('PRODUCT_ZONE_4', 'PRODUCT', 0.56, 0.80, 0.00, 0.00),
    ('PRODUCT_ZONE_5', 'PRODUCT', 0.68, 0.80, 0.00, 0.00),
    ('PRODUCT_ZONE_6', 'PRODUCT', 0.80, 0.80, 0.00, 0.00),
    ('PRODUCT_SLOT_1', 'PRODUCT_SLOT', 0.20, 0.92, 0.35, 0.00),
    ('PRODUCT_SLOT_2', 'PRODUCT_SLOT', 0.32, 0.92, 0.35, 0.00),
    ('PRODUCT_SLOT_3', 'PRODUCT_SLOT', 0.44, 0.92, 0.35, 0.00),
    ('PRODUCT_SLOT_4', 'PRODUCT_SLOT', 0.56, 0.92, 0.35, 0.00),
    ('PRODUCT_SLOT_5', 'PRODUCT_SLOT', 0.68, 0.92, 0.35, 0.00),
    ('PRODUCT_SLOT_6', 'PRODUCT_SLOT', 0.80, 0.92, 0.35, 0.00),
    ('PICKUP_ZONE_1', 'PICKUP', 1.70, 0.75, 0.00, 3.14),
    ('PICKUP_ZONE_2', 'PICKUP', 1.70, 0.55, 0.00, 3.14),
    ('PICKUP_ZONE_3', 'PICKUP', 1.70, 0.35, 0.00, 3.14),
    ('PICKUP_ZONE_4', 'PICKUP', 1.70, 0.15, 0.00, 3.14),
    ('PICKUP_SLOT_1', 'PICKUP_SLOT', 1.88, 0.75, 0.35, 3.14),
    ('PICKUP_SLOT_2', 'PICKUP_SLOT', 1.88, 0.55, 0.35, 3.14),
    ('PICKUP_SLOT_3', 'PICKUP_SLOT', 1.88, 0.35, 0.35, 3.14),
    ('PICKUP_SLOT_4', 'PICKUP_SLOT', 1.88, 0.15, 0.35, 3.14);

INSERT INTO product (name, image_url, stock_qty, storage_zone_id) VALUES
    ('우유', '/static/img/milk.png', 2, (SELECT zone_id FROM zone WHERE zone_name = 'PRODUCT_SLOT_1')),
    ('시리얼', '/static/img/cereal.png', 2, (SELECT zone_id FROM zone WHERE zone_name = 'PRODUCT_SLOT_2')),
    ('바나나 우유', '/static/img/banana_milk.png', 2, (SELECT zone_id FROM zone WHERE zone_name = 'PRODUCT_SLOT_3')),
    ('식빵', '/static/img/bread.png', 2, (SELECT zone_id FROM zone WHERE zone_name = 'PRODUCT_SLOT_4')),
    ('투게더', '/static/img/together.png', 2, (SELECT zone_id FROM zone WHERE zone_name = 'PRODUCT_SLOT_5')),
    ('바나나', '/static/img/banana.png', 2, (SELECT zone_id FROM zone WHERE zone_name = 'PRODUCT_SLOT_6'));

INSERT INTO pickup_slot (slot_name, status) VALUES
    ('Pickup_slot_1', 'EMPTY'),
    ('Pickup_slot_2', 'EMPTY'),
    ('Pickup_slot_3', 'EMPTY'),
    ('Pickup_slot_4', 'EMPTY');

INSERT INTO robot_unit (unit_name, description) VALUES
    ('PICKY_UNIT_1', 'PICKY1 and COBOT1 pair'),
    ('PICKY_UNIT_2', 'PICKY2 and COBOT2 pair');

INSERT INTO robot (
    robot_name,
    unit_id,
    robot_type,
    robot_status,
    picky_state,
    cobot_state,
    ros_namespace,
    battery_level,
    pos_x,
    pos_y,
    pos_theta
) VALUES
    ('PICKY1', 1, 'PICKY', 'IDLE', 'STANDBY', NULL, '/picky1', 100, 0.90, 0.82, 0.00),
    ('COBOT1', 1, 'COBOT', 'IDLE', NULL, 'STANDBY', '/cobot1', NULL, NULL, NULL, NULL),
    ('PICKY2', 2, 'PICKY', 'IDLE', 'STANDBY', NULL, '/picky2', 100, 0.90, 0.08, 0.00),
    ('COBOT2', 2, 'COBOT', 'IDLE', NULL, 'STANDBY', '/cobot2', NULL, NULL, NULL, NULL);
