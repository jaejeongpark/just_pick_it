#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('just_pick_it_perception')
    default_params = os.path.join(pkg_share, 'config', 'place_planner_params.yaml')

    params_arg = DeclareLaunchArgument('params_file', default_value=default_params)

    node = Node(
        package='just_pick_it_perception',
        executable='shelf_place_planner',
        name='shelf_place_planner_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([params_arg, node])
