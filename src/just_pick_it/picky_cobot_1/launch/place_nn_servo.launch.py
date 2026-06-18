#!/usr/bin/env python3
"""
DISPLAY_PLACE 서보 스택 (display_place_agent 가 on-demand 로 기동) — 픽 NN weight 공유 버전.

픽(nn_inference.launch.py)과 동일한 ibvs + nn_controller 를 그대로 재사용하되,
빈자리에는 yolo 로 잡을 객체가 없으므로 앞단에 csrt_place_tracker 를 둬서 빈자리 bbox 를
추적 detection 으로 합성해 공급한다. release(놓기)는 perception 을 수정하지 않기 위해
display_place_agent 가 담당한다(아래 'release 반전' 참고).

구성:
  1. csrt_place_tracker : /place/target_bbox(빈자리 bbox, latched)로 CSRT init 후
                          /infer/image_raw 추적 -> /place/tracked_objects(class_label=empty_slot).
  2. ibvs_controller    : detection_topic=/place/tracked_objects, target_class_label=empty_slot
                          로 빈자리 bbox 에 image-based 수렴 -> ibvs_done.
  3. nn_controller      : ibvs_done 후 활성화. 픽과 동일 정책/예측기(model_dir 공유)로 J1~J5
                          정밀 보정 후 grip 예측기가 발화하면 set_gripper [0](close) 발행.

release 반전(perception 무수정 핵심):
  - nn_controller 는 grip 을 [0](close)로 하드코딩한다(픽 전용). place 는 물건을 놓아야 하므로
    이 close 명령을 'NN 정렬 완료 = 놓을 타이밍' 신호로만 쓴다. display_place_agent 가 그 close
    를 관측하면 자기가 set_gripper [70](open) 을 발행해 release 하고 완료로 보고한다.
  - 따라서 이 launch 는 픽의 nn_inference 와 달리 '시작 시 gripper open' TimerAction 을 두지
    않는다(물건을 쥔 채 시작하므로 시작 시 열면 안 됨).

주의:
  - 실제 로봇을 움직인다. 작업공간을 비우고 실행.
  - 픽 weight 공유는 bootstrap 이다. grip 예측기는 객체 파지 장면으로 학습돼 빈자리 release
    타이밍 정확도는 검증 대상이다(필요 시 place 전용 재학습).
  - place IBVS 가 멀리 search 하지 않도록 pregrasp 자세를 진열대 관측 자세로 둔다
    (place_pregrasp_angles 기본값은 placeholder, 실측 보정 필요).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def _f(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _i(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def generate_launch_description():
    perception_share = get_package_share_directory('just_pick_it_perception')
    ibvs_launch = os.path.join(perception_share, 'launch', 'ibvs_controller.launch.py')
    home = os.path.expanduser('~')
    default_model_dir = os.path.join(
        home,
        'just_pick_it/src/just_pick_it/just_pick_it_perception/result/nn_controller',
    )

    robot_name = LaunchConfiguration('robot_name')
    detection_topic = LaunchConfiguration('detection_topic')
    target_class_label = LaunchConfiguration('target_class_label')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    desired_area_norm = LaunchConfiguration('desired_area_norm')
    place_pregrasp = LaunchConfiguration('place_pregrasp_angles')
    search_timeout_sec = LaunchConfiguration('search_timeout_sec')
    model_dir = LaunchConfiguration('model_dir')

    args = [
        DeclareLaunchArgument('robot_name', default_value='jetcobot1'),
        # csrt_place_tracker 가 합성하는 빈자리 detection 토픽/라벨.
        DeclareLaunchArgument('detection_topic', default_value='/place/tracked_objects'),
        DeclareLaunchArgument('target_class_label', default_value='empty_slot'),
        DeclareLaunchArgument('image_width', default_value='640.0'),
        DeclareLaunchArgument('image_height', default_value='480.0'),
        # 배치 높이에서 멈추도록 면적 종료 임계(선반면+상품 높이 고려, 실측 보정).
        DeclareLaunchArgument('desired_area_norm', default_value='0.23'),
        # 멀리 search 하지 않도록 진열대 관측 자세로 둠(placeholder, 실측 보정 필요).
        DeclareLaunchArgument(
            'place_pregrasp_angles',
            default_value='[114.78,-5.09,-9.05,-75.49,9.05,-107.31]'),
        DeclareLaunchArgument('search_timeout_sec', default_value='1.0'),
        # 픽과 공유하는 NN weight 디렉터리(policy + grip_success_predictor + config).
        DeclareLaunchArgument('model_dir', default_value=default_model_dir),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('min_confidence', default_value='0.5'),
        # NN grip(=정렬 완료) 게이트. 픽 nn_inference 기본값과 동일.
        DeclareLaunchArgument('grip_confidence_threshold', default_value='0.8'),
        DeclareLaunchArgument('grip_consecutive_required', default_value='3'),
        DeclareLaunchArgument('max_fine_tune_steps', default_value='100'),
        DeclareLaunchArgument('nn_command_speed', default_value='10'),
        DeclareLaunchArgument('nn_control_rate_hz', default_value='0.0'),
        DeclareLaunchArgument('nn_delta_scale', default_value='1.0'),
        DeclareLaunchArgument('nn_delta_smooth_alpha', default_value='0.5'),
        DeclareLaunchArgument('nn_settle_delta_deg', default_value='0.8'),
        DeclareLaunchArgument('nn_status_poll_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('nn_command_leash_deg', default_value='8.0'),
    ]

    # 1. 빈자리 bbox 를 CSRT 로 추적 -> /place/tracked_objects(empty_slot).
    csrt = Node(
        package='just_pick_it_perception',
        executable='csrt_place_tracker',
        name='csrt_place_tracker_node',
        output='screen',
        parameters=[{'class_label': 'empty_slot'}],
    )

    # 2. IBVS 수렴(빈자리 bbox 로). 픽처럼 멀리 search 하지 않도록 pregrasp 를 관측 자세로 고정.
    ibvs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ibvs_launch),
        launch_arguments={
            'robot_name': robot_name,
            'detection_topic': detection_topic,
            'target_class_label': target_class_label,
            'image_width': image_width,
            'image_height': image_height,
            'desired_area_norm': desired_area_norm,
            'search_timeout_sec': search_timeout_sec,
            'center_pregrasp_angles': place_pregrasp,
            'left_pregrasp_angles': place_pregrasp,
            'right_pregrasp_angles': place_pregrasp,
        }.items(),
    )

    # 3. 픽과 동일한 nn_controller(weight 공유). grip 발화 시 close 발행(=agent 가 release 트리거로 사용).
    #    [중요] 픽 nn_inference 의 '시작 시 gripper open' TimerAction 은 두지 않는다(물건을 쥔 채 시작).
    nn_controller = Node(
        package='just_pick_it_perception',
        executable='nn_controller',
        name='nn_controller',
        output='screen',
        parameters=[{
            'robot_name': robot_name,
            'detection_topic': detection_topic,
            'target_class_label': target_class_label,
            'min_confidence': _f('min_confidence'),
            'image_width': _f('image_width'),
            'image_height': _f('image_height'),
            'model_dir': model_dir,
            'device': LaunchConfiguration('device'),
            'grip_confidence_threshold': _f('grip_confidence_threshold'),
            'grip_consecutive_required': _i('grip_consecutive_required'),
            'max_fine_tune_steps': _i('max_fine_tune_steps'),
            'control_rate_hz': _f('nn_control_rate_hz'),
            'command_speed': _i('nn_command_speed'),
            'delta_scale': _f('nn_delta_scale'),
            'delta_smooth_alpha': _f('nn_delta_smooth_alpha'),
            'settle_delta_deg': _f('nn_settle_delta_deg'),
            'status_poll_rate_hz': _f('nn_status_poll_rate_hz'),
            'command_leash_deg': _f('nn_command_leash_deg'),
        }],
    )

    return LaunchDescription(args + [csrt, ibvs, nn_controller])
