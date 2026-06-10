-- Zone poses. map frame, 좌하단 원점(+x 오른쪽, +y 위쪽), 약 2.0m x 1.0m arena.
-- docs/Traffic_node_2.1.jpg 토폴로지 기준이며 ZONE 좌표는 traffic_manager.py 의
-- DEFAULT_ZONE_COORDS 와 일치시킨다. 대략값이므로 실측 후 미세조정 필요.
-- *_ZONE_* = PICKY 주행/주차 pose. *_SLOT_* = COBOT 작업용 물리 pose(z=0.35 선반/슬롯 높이).
-- TRAFFIC_*/CHARGING_DOCK_* 그래프 노드 좌표는 DB 가 아니라 DEFAULT_ZONE_COORDS 에 있다.
INSERT INTO
    zone (
        zone_name,
        zone_type,
        pos_x,
        pos_y,
        pos_z,
        pos_theta
    )
VALUES (
        'STANDBY_ZONE_1',
        'STANDBY',
        0.11,
        0.38,
        0.00,
        1.5708
    ),
    (
        'STANDBY_ZONE_2',
        'STANDBY',
        0.28,
        0.38,
        0.00,
        1.5708
    ),
    (
        'STOCK_ZONE',
        'STOCK',
        0.20,
        0.85,
        0.00,
        3.1416
    ),
    (
        'STOCK_SLOT',
        'STOCK_SLOT',
        0.05,
        0.85,
        0.35,
        0.00
    ),
    (
        'PRODUCT_ZONE_1',
        'PRODUCT',
        0.64,
        0.60,
        0.00,
        0.00
    ),
    (
        'PRODUCT_ZONE_2',
        'PRODUCT',
        1.05,
        0.60,
        0.00,
        0.00
    ),
    (
        'PRODUCT_ZONE_3',
        'PRODUCT',
        1.48,
        0.60,
        0.00,
        0.00
    ),
    (
        'PRODUCT_ZONE_4',
        'PRODUCT',
        0.64,
        0.36,
        0.00,
        0.00
    ),
    (
        'PRODUCT_ZONE_5',
        'PRODUCT',
        1.05,
        0.36,
        0.00,
        0.00
    ),
    (
        'PRODUCT_ZONE_6',
        'PRODUCT',
        1.48,
        0.36,
        0.00,
        0.00
    ),
    (
        'PRODUCT_SLOT_1',
        'PRODUCT_SLOT',
        0.82,
        0.60,
        0.35,
        0.00
    ),
    (
        'PRODUCT_SLOT_2',
        'PRODUCT_SLOT',
        1.17,
        0.60,
        0.35,
        0.00
    ),
    (
        'PRODUCT_SLOT_3',
        'PRODUCT_SLOT',
        1.52,
        0.60,
        0.35,
        0.00
    ),
    (
        'PRODUCT_SLOT_4',
        'PRODUCT_SLOT',
        0.82,
        0.35,
        0.35,
        0.00
    ),
    (
        'PRODUCT_SLOT_5',
        'PRODUCT_SLOT',
        1.17,
        0.35,
        0.35,
        0.00
    ),
    (
        'PRODUCT_SLOT_6',
        'PRODUCT_SLOT',
        1.52,
        0.35,
        0.35,
        0.00
    ),
    (
        'PICKUP_ZONE_1',
        'PICKUP',
        1.80,
        0.85,
        0.00,
        3.14
    ),
    (
        'PICKUP_ZONE_2',
        'PICKUP',
        1.80,
        0.15,
        0.00,
        3.14
    ),
    (
        'PICKUP_SLOT_1',
        'PICKUP_SLOT',
        1.80,
        0.80,
        0.35,
        3.14
    ),
    (
        'PICKUP_SLOT_2',
        'PICKUP_SLOT',
        1.80,
        0.20,
        0.35,
        3.14
    ),
    -- 충전 도크 + 교통(라우팅) 노드. TrafficManager 그래프 경로를 MoveCommand waypoint
    -- 리스트로 로봇에 전송하려면 중간 경유지도 DB pose 가 있어야 한다.
    -- 좌표는 traffic_manager.py DEFAULT_ZONE_COORDS 와 일치(좌하단 원점).
    (
        'CHARGING_DOCK_1',
        'CHARGING',
        0.11,
        0.08,
        0.00,
        1.5708
    ),
    (
        'CHARGING_DOCK_2',
        'CHARGING',
        0.28,
        0.08,
        0.00,
        1.5708
    ),
    (
        'TRAFFIC_T1',
        'TRAFFIC',
        0.64,
        0.85,
        0.00,
        0.00
    ),
    (
        'TRAFFIC_T2',
        'TRAFFIC',
        1.05,
        0.85,
        0.00,
        0.00
    ),
    (
        'TRAFFIC_T3',
        'TRAFFIC',
        1.48,
        0.85,
        0.00,
        0.00
    ),
    (
        'TRAFFIC_B1',
        'TRAFFIC',
        0.64,
        0.15,
        0.00,
        0.00
    ),
    (
        'TRAFFIC_B2',
        'TRAFFIC',
        1.05,
        0.15,
        0.00,
        0.00
    ),
    (
        'TRAFFIC_B3',
        'TRAFFIC',
        1.48,
        0.15,
        0.00,
        0.00
    );

INSERT INTO
    product (
        name,
        image_url,
        stock_qty,
        storage_zone_id
    )
VALUES (
        '수박',
        '/static/img/watermelon.png',
        3,
        (
            SELECT zone_id
            FROM zone
            WHERE
                zone_name = 'PRODUCT_SLOT_1'
        )
    ),
    (
        '식빵',
        '/static/img/sliced_bread.png',
        5,
        (
            SELECT zone_id
            FROM zone
            WHERE
                zone_name = 'PRODUCT_SLOT_2'
        )
    ),
    (
        '환타',
        '/static/img/fanta.png',
        5,
        (
            SELECT zone_id
            FROM zone
            WHERE
                zone_name = 'PRODUCT_SLOT_3'
        )
    ),
    (
        '크림빵',
        '/static/img/cream_filled_bread.png',
        5,
        (
            SELECT zone_id
            FROM zone
            WHERE
                zone_name = 'PRODUCT_SLOT_4'
        )
    ),
    (
        '초코파이',
        '/static/img/choco_pie.png',
        5,
        (
            SELECT zone_id
            FROM zone
            WHERE
                zone_name = 'PRODUCT_SLOT_5'
        )
    ),
    (
        '생수',
        '/static/img/bottled_water.png',
        3,
        (
            SELECT zone_id
            FROM zone
            WHERE
                zone_name = 'PRODUCT_SLOT_6'
        )
    );

INSERT INTO
    pickup_slot (slot_name, status)
VALUES ('PICKUP_SLOT_1', 'EMPTY'),
    ('PICKUP_SLOT_2', 'EMPTY');

INSERT INTO
    robot_unit (unit_name, description)
VALUES (
        'PICKY_UNIT_1',
        'PICKY1 and COBOT1 pair'
    ),
    (
        'PICKY_UNIT_2',
        'PICKY2 and COBOT2 pair'
    );

INSERT INTO
    robot (
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
    )
VALUES (
        'PICKY1',
        1,
        'PICKY',
        'IDLE',
        'STANDBY',
        NULL,
        '/picky1',
        100,
        0.11,
        0.08,
        1.5708
    ),
    (
        'COBOT1',
        1,
        'COBOT',
        'IDLE',
        NULL,
        'STANDBY',
        '/cobot1',
        NULL,
        NULL,
        NULL,
        NULL
    ),
    (
        'PICKY2',
        2,
        'PICKY',
        'IDLE',
        'STANDBY',
        NULL,
        '/picky2',
        100,
        0.28,
        0.08,
        1.5708
    ),
    (
        'COBOT2',
        2,
        'COBOT',
        'IDLE',
        NULL,
        'STANDBY',
        '/cobot2',
        NULL,
        NULL,
        NULL,
        NULL
    );
