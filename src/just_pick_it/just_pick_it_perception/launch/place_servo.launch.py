#!/usr/bin/env python3

"""DISPLAY_PLACE 서보 스택 (display_place_agent 가 on-demand 로 기동).

구성:
  1. csrt_place_tracker : /place/target_bbox(빈자리 bbox, latched)로 CSRT init 후
                          /infer/image_raw 추적 -> 합성 TrackedObjectArray(/place/tracked_objects,
                          class_label=empty_slot) 발행.
  2. ibvs_controller    : detection_topic=/place/tracked_objects, target_class_label=empty_slot
                          로 빈자리 bbox 에 image-based 수렴 -> ibvs_done.
  3. release            : ibvs_done 후 종단 처리.
                          use_place_nn=false(기본) -> place_finalizer(결정론적 하강+open, C-1).
                          use_place_nn=true        -> place_nn_controller(학습 모델, C-2, Phase D).

주의:
  - 실제 로봇을 움직인다. 작업공간을 비우고 실행.
  - 빈 공간에는 servo 할 객체가 없으므로 CSRT 가 빈자리 bbox 를 추적해 IBVS 의 타깃을
    공급한다. eye-in-hand 하강으로 빈자리가 가려져 추적이 끊겨도, NN(C-2)은 anchor frozen
    으로 detection 없이 동작한다(C-1 finalizer 는 ibvs_done 시점에 종단 처리).
  - place IBVS 는 픽 pregrasp 로 멀리 search 하지 않도록 pregrasp 자세를 진열대 관측 자세로
    맞춰 둔다(아래 *_pregrasp_angles 기본값은 placeholder, 실측 보정 필요).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perception_share = get_package_share_directory('just_pick_it_perception')
    ibvs_launch = os.path.join(perception_share, 'launch', 'ibvs_controller.launch.py')

    robot_name = LaunchConfiguration('robot_name')
    detection_topic = LaunchConfiguration('detection_topic')
    target_class_label = LaunchConfiguration('target_class_label')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    desired_area_norm = LaunchConfiguration('desired_area_norm')
    use_place_nn = LaunchConfiguration('use_place_nn')

    args = [
        DeclareLaunchArgument('robot_name', default_value='jetcobot1'),
        DeclareLaunchArgument('detection_topic', default_value='/place/tracked_objects'),
        DeclareLaunchArgument('target_class_label', default_value='empty_slot'),
        DeclareLaunchArgument('image_width', default_value='640.0'),
        DeclareLaunchArgument('image_height', default_value='480.0'),
        # 배치 높이에서 멈추도록 면적 종료 임계 재튜닝(선반면+상품 높이 고려, 실측 보정).
        DeclareLaunchArgument('desired_area_norm', default_value='0.23'),
        # 멀리 search 하지 않도록 진열대 관측 자세로 둠(placeholder, 실측 보정 필요).
        DeclareLaunchArgument(
            'place_pregrasp_angles',
            default_value='[114.78,-5.09,-9.05,-75.49,9.05,-107.31]'),
        DeclareLaunchArgument('search_timeout_sec', default_value='1.0'),
        # release 단계 선택. 기본 false = C-1 결정론적 finalizer.
        DeclareLaunchArgument('use_place_nn', default_value='false'),
        DeclareLaunchArgument('place_descent_mm', default_value='0.0'),
        DeclareLaunchArgument(
            'place_nn_model_dir',
            default_value=os.path.join(
                perception_share, 'result', 'place_nn_controller')),
    ]

    place_pregrasp = LaunchConfiguration('place_pregrasp_angles')

    csrt = Node(
        package='just_pick_it_perception',
        executable='csrt_place_tracker',
        name='csrt_place_tracker_node',
        output='screen',
        parameters=[{'class_label': 'empty_slot'}],
    )

    ibvs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ibvs_launch),
        launch_arguments={
            'robot_name': robot_name,
            'detection_topic': detection_topic,
            'target_class_label': target_class_label,
            'image_width': image_width,
            'image_height': image_height,
            'desired_area_norm': desired_area_norm,
            'search_timeout_sec': LaunchConfiguration('search_timeout_sec'),
            'center_pregrasp_angles': place_pregrasp,
            'left_pregrasp_angles': place_pregrasp,
            'right_pregrasp_angles': place_pregrasp,
        }.items(),
    )

    # C-1: 결정론적 종단(하강 + open). use_place_nn=false 일 때.
    finalizer = Node(
        package='just_pick_it_perception',
        executable='place_finalizer',
        name='place_finalizer_node',
        output='screen',
        condition=UnlessCondition(use_place_nn),
        parameters=[{
            'robot_name': robot_name,
            'descent_mm': LaunchConfiguration('place_descent_mm'),
        }],
    )

    # C-2: 학습된 place release NN(Phase D). use_place_nn=true 일 때.
    place_nn = Node(
        package='just_pick_it_perception',
        executable='place_nn_controller',
        name='place_nn_controller_node',
        output='screen',
        condition=IfCondition(use_place_nn),
        parameters=[{
            'robot_name': robot_name,
            'detection_topic': detection_topic,
            'target_class_label': target_class_label,
            'image_width': image_width,
            'image_height': image_height,
            'model_dir': LaunchConfiguration('place_nn_model_dir'),
        }],
    )

    return LaunchDescription(args + [csrt, ibvs, finalizer, place_nn])
