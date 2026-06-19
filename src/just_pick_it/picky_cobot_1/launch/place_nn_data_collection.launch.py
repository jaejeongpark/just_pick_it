#!/usr/bin/env python3
"""DISPLAY_PLACE 데이터 수집 launch (픽 nn_data_collection.launch.py 의 place 버전).

픽 수집과의 차이:
  1. detection 소스: 잡을 객체가 없고 타깃이 '빈 슬롯'이라, csrt_place_tracker 가
     latched /place/target_bbox 로부터 /place/tracked_objects(empty_slot)를 합성한다.
     IBVS 와 visual_servo_bag_recorder 를 이 토픽/라벨로 가리킨다.
  2. 시작 그리퍼 open 없음: place 는 물건을 쥔 채 시작하므로 launch 시작 시
     gripper open 을 발행하지 않는다(픽 launch 의 gripper_open TimerAction 제거).
  3. human recorder 반전: place_interaction_recorder 가 [R] 에서 그리퍼를 닫은 채
     서보만 풀고, [G] 에서 그리퍼를 열어(놓기) 종단한다.

구성 노드:
  - csrt_place_tracker        (just_pick_it_perception)  : 빈자리 bbox 추적 -> detection 합성
  - ibvs_controller           (just_pick_it_perception)  : 빈자리 bbox 에 IBVS 수렴 -> ibvs_done
  - visual_servo_bag_recorder (just_pick_it_perception)  : IBVS 구간(detection+관절) 기록
  - place_interaction_recorder(picky_cobot_1)            : ibvs_done 후 free-drive+놓기 기록

수집 절차(요약):
  0. 물건을 그리퍼에 쥐고, 팔을 진열대 관측 pregrasp 자세 부근에 둔다.
  1. 빈자리 bbox 를 latched 로 1회 발행해 CSRT init (아래 예시 명령 참고).
     ros2 topic pub --once /place/target_bbox std_msgs/msg/Float64MultiArray \
       "{data: [320.0, 240.0, 60.0, 40.0, 0.0]}"
     (cx, cy, w, h, angle_deg — 카메라 image space 기준)
  2. IBVS 가 빈자리로 수렴해 ibvs_done 발행 -> GUI 가 [R] 대기로 전환.
  3. [R] 팔 서보 해제(그리퍼 닫힘 유지) -> 손으로 물건을 빈자리 위로 이동 -> [G] 놓기 ->
     [S]/[F] 결과 레이블. loop_episodes=true 면 다음 episode 로 자동 진행.

학습: 수집된 bag(bag_base_dir 의 success/)을 픽과 동일한 train 파이프라인에 넣어
result/nn_controller/place 로 학습 산출물을 만든다(train_nn_controller.py --out-dir).
place_nn_servo.launch.py 의 model_dir 기본값이 이미 result/nn_controller/place 이므로,
이 디렉터리 내용만 교체하면 별도 인자 없이 새 place 모델이 적용된다.

주의: 실제 로봇을 움직인다. 작업공간을 비우고 실행할 것.
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    detection_topic = LaunchConfiguration('detection_topic').perform(context)
    target_class_label = LaunchConfiguration('target_class_label').perform(context)
    min_confidence = LaunchConfiguration('min_confidence').perform(context)
    image_width = LaunchConfiguration('image_width').perform(context)
    image_height = LaunchConfiguration('image_height').perform(context)
    bag_base_dir = LaunchConfiguration('bag_base_dir').perform(context)
    episode_id = LaunchConfiguration('episode_id').perform(context)
    record_rate_hz = LaunchConfiguration('record_rate_hz').perform(context)
    record_mode = LaunchConfiguration('record_mode').perform(context)
    displacement_threshold_deg = LaunchConfiguration('displacement_threshold_deg').perform(context)
    loop_episodes = LaunchConfiguration('loop_episodes').perform(context)
    done_status_poll_rate_hz = LaunchConfiguration('done_status_poll_rate_hz').perform(context)
    desired_area_norm = LaunchConfiguration('desired_area_norm').perform(context)
    search_timeout_sec = LaunchConfiguration('search_timeout_sec').perform(context)
    place_pregrasp = LaunchConfiguration('place_pregrasp_angles').perform(context)
    launch_detection = LaunchConfiguration('launch_detection').perform(context)
    udp_port = LaunchConfiguration('udp_port').perform(context)
    detection_model_path = LaunchConfiguration('detection_model_path').perform(context)
    csrt_debugger = LaunchConfiguration('csrt_debugger').perform(context)
    csrt_stale_sec = LaunchConfiguration('csrt_stale_sec').perform(context)

    if not episode_id:
        episode_id = datetime.now().strftime('%Y%m%d_%H%M%S')

    print(f'[place_nn_data_collection] episode_id = {episode_id}')
    print(f'[place_nn_data_collection] bag_base_dir = {bag_base_dir}')
    print(f'[place_nn_data_collection] detection_topic = {detection_topic}')

    perception_share = get_package_share_directory('just_pick_it_perception')
    ibvs_launch_path = os.path.join(perception_share, 'launch', 'ibvs_controller.launch.py')
    yolo_launch_path = os.path.join(perception_share, 'launch', 'yolo_seg_infer.launch.xml')

    # 0. 카메라 영상 소스 + YOLO detection. csrt_place_tracker 가 /infer/image_raw 를 추적
    #    대상으로 쓰므로 영상 스트림이 반드시 필요하다(place 는 YOLO detection 자체는 쓰지
    #    않지만 영상은 yolo_seg_infer 가 발행한다). 이미 따로 띄웠다면 launch_detection:=false
    #    로 꺼서 udp_port 중복 바인딩 충돌을 피한다.
    detection = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(yolo_launch_path),
        launch_arguments={
            'udp_port': udp_port,
            'model_path': detection_model_path,
        }.items(),
    )

    # 1. 빈자리 bbox(latched /place/target_bbox)를 CSRT 로 추적 -> /place/tracked_objects(empty_slot).
    csrt = Node(
        package='just_pick_it_perception',
        executable='csrt_place_tracker',
        name='csrt_place_tracker_node',
        output='screen',
        parameters=[{'class_label': target_class_label}],
    )

    # 2. IBVS 수렴(빈자리 bbox 로). 멀리 search 하지 않도록 pregrasp 를 진열대 관측 자세로 고정.
    ibvs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ibvs_launch_path),
        launch_arguments={
            'robot_name': robot_name,
            'detection_topic': detection_topic,
            'target_class_label': target_class_label,
            'min_confidence': min_confidence,
            'image_width': image_width,
            'image_height': image_height,
            'desired_area_norm': desired_area_norm,
            'search_timeout_sec': search_timeout_sec,
            'center_pregrasp_angles': place_pregrasp,
            'left_pregrasp_angles': place_pregrasp,
            'right_pregrasp_angles': place_pregrasp,
            'done_status_poll_rate_hz': done_status_poll_rate_hz,
        }.items(),
    )

    # 3. IBVS align+approach 구간(detection+관절) 기록. ibvs_done 시 bag 만 닫고 노드는 유지.
    visual_servo_recorder = Node(
        package='just_pick_it_perception',
        executable='visual_servo_bag_recorder',
        name='visual_servo_bag_recorder',
        output='screen',
        parameters=[
            {
                'robot_name': robot_name,
                'detection_topic': detection_topic,
                'target_class_label': target_class_label,
                'min_confidence': float(min_confidence),
                'image_width': float(image_width),
                'image_height': float(image_height),
                'bag_base_dir': bag_base_dir,
                'episode_id': episode_id,
                'shutdown_on_stop': False,
            }
        ],
    )

    # 4. ibvs_done 이후 free-drive + 놓기(release) 구간 기록(picky_cobot_1, 그리퍼 의미 반전).
    #    결과 확정(S/F) 시 episode 디렉터리 이동. 키 입력은 tkinter GUI 로 받는다.
    place_recorder = Node(
        package='picky_cobot_1',
        executable='place_interaction_recorder',
        name='place_interaction_recorder',
        output='screen',
        parameters=[
            {
                'robot_name': robot_name,
                'bag_base_dir': bag_base_dir,
                'episode_id': episode_id,
                'record_rate_hz': float(record_rate_hz),
                'record_mode': record_mode,
                'displacement_threshold_deg': float(displacement_threshold_deg),
                'loop_episodes': (loop_episodes.lower() in ('true', '1', 'yes')),
                'shutdown_on_done': True,
            }
        ],
    )

    # 한 episode = 한 launch 실행. place_recorder 가 결과 기록 후 종료되면 전체 launch 종료.
    shutdown_on_recorder_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=place_recorder,
            on_exit=[EmitEvent(event=Shutdown(reason='place_interaction_recorder finished'))],
        )
    )

    # CSRT 추적 시각화 overlay(디버그 전용). bbox 시드가 제대로 들어가 추적이 되는지
    # /place/csrt_overlay(Image)를 rqt_image_view 로 확인한다(ibvs_nn_place_agent 와 동일).
    csrt_debug = Node(
        package='picky_cobot_1',
        executable='csrt_overlay_viz',
        name='csrt_overlay_viz',
        output='screen',
        parameters=[{'track_stale_sec': float(csrt_stale_sec)}],
    )

    # 픽 launch 와 달리 시작 시 gripper open 을 발행하지 않는다(물건을 쥔 채 시작).
    nodes = [
        csrt,
        ibvs,
        visual_servo_recorder,
        place_recorder,
        shutdown_on_recorder_exit,
    ]
    # 영상 소스(yolo_seg_infer)를 같이 띄운다. 이미 따로 실행 중이면 launch_detection:=false.
    if launch_detection.lower() in ('true', '1', 'yes'):
        nodes.insert(0, detection)
    # CSRT 추적 시각화. csrt_debugger:=true 일 때만.
    if csrt_debugger.lower() in ('true', '1', 'yes'):
        nodes.append(csrt_debug)
    return nodes


def generate_launch_description():
    args = [
        DeclareLaunchArgument('robot_name', default_value='jetcobot1'),
        # csrt_place_tracker 가 합성하는 빈자리 detection 토픽/라벨.
        DeclareLaunchArgument('detection_topic', default_value='/place/tracked_objects'),
        DeclareLaunchArgument('target_class_label', default_value='empty_slot'),
        DeclareLaunchArgument('min_confidence', default_value='0.5'),
        DeclareLaunchArgument('image_width', default_value='640.0'),
        DeclareLaunchArgument('image_height', default_value='480.0'),
        # 픽 데이터와 섞이지 않도록 place 전용 디렉터리를 기본값으로 둔다.
        DeclareLaunchArgument('bag_base_dir', default_value='~/rosbags_place'),
        # 비워두면 launch 시점 타임스탬프로 자동 생성된다.
        DeclareLaunchArgument('episode_id', default_value=''),
        DeclareLaunchArgument('record_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('record_mode', default_value='displacement'),
        DeclareLaunchArgument('displacement_threshold_deg', default_value='2.0'),
        DeclareLaunchArgument('loop_episodes', default_value='true'),
        DeclareLaunchArgument('done_status_poll_rate_hz', default_value='10.0'),
        # IBVS area DONE(-> ibvs_done) 게이트. place 는 빈자리(작은 슬롯)라 낮은 값을 쓴다.
        # place_nn_servo.launch.py 의 desired_area_norm 과 맞춰 둔다.
        DeclareLaunchArgument('desired_area_norm', default_value='0.017'),
        DeclareLaunchArgument('search_timeout_sec', default_value='1.0'),
        # 멀리 search 하지 않도록 진열대 관측 자세로 둠(placeholder, 실측 보정 필요).
        DeclareLaunchArgument(
            'place_pregrasp_angles',
            default_value='[114.78,-5.09,-9.05,-75.49,9.05,-107.31]'),
        # 영상 소스(yolo_seg_infer) 동시 기동. csrt_place_tracker 가 /infer/image_raw 를
        # 추적 대상으로 쓰므로 영상 스트림이 필수다. 이미 따로 띄웠다면 false 로 끈다
        # (udp_port 중복 바인딩 충돌 방지).
        DeclareLaunchArgument('launch_detection', default_value='true'),
        # 카메라 UDP 수신 포트. 카메라 송신 포트와 일치해야 한다(운영 기본 5003).
        DeclareLaunchArgument('udp_port', default_value='5003'),
        # YOLO-seg 모델 경로. yolo_seg_infer.launch.xml 에 넘기던 값과 동일.
        DeclareLaunchArgument(
            'detection_model_path',
            default_value=os.path.join(
                os.path.expanduser('~'),
                'just_pick_it/src/just_pick_it/just_pick_it_perception',
                'result/jetcobot_1/best.pt')),
        # CSRT 추적 시각화(csrt_overlay_viz). bbox 시드/추적 확인용 디버그.
        # /place/csrt_overlay 를 rqt_image_view 로 본다(ibvs_nn_place_agent 와 동일).
        DeclareLaunchArgument('csrt_debugger', default_value='false'),
        DeclareLaunchArgument('csrt_stale_sec', default_value='0.5'),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
