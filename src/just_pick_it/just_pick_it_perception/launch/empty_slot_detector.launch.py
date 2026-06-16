#!/usr/bin/env python3

"""빈자리 detector (DISPLAY_SCAN). AI 컴퓨터에서 상시 가동.

cobot_controller(run_scanning)가 /place/reset -> /place/capture_view(자세별) -> /place/plan
을 발행하면, 최신 YOLO-seg/이미지로 image-space 빈자리 후보를 누적·최적화해 /place/scan_result
를 발행한다. 카메라 calibration/메트릭 투영 없이 이미지 픽셀로만 동작한다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('just_pick_it_perception')
    default_params = os.path.join(pkg_share, 'config', 'empty_slot_detector_params.yaml')

    params_arg = DeclareLaunchArgument('params_file', default_value=default_params)

    node = Node(
        package='just_pick_it_perception',
        executable='empty_slot_detector',
        name='empty_slot_detector_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([params_arg, node])
