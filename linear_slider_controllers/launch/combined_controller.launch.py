#!/usr/bin/env python3
"""
Launch file to test the functionality of the combined controller
Author: Luke Strohbehn
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    linear_slider_controller_config = os.path.join(
        get_package_share_directory("linear_slider_bringup"), "config", "linear_slider_controllers.yaml"
    )

    # ur_controller_config = os.path.join(
    #     get_package_share_directory("ur_controllers"),
    #     "config",

    # )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[linear_slider_controller_config],
        output="screen",
    )

    ld = LaunchDescription([controller_manager])

    return controller_manager
