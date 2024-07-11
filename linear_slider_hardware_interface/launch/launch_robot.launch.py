import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

"""
Reference:
https://articulatedrobotics.xyz/mobile-robot-13-ros2-control-real/
"""


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    use_sim = LaunchConfiguration("use_sim")

    package_dir = get_package_share_directory("linear_slider_hardware_interface")
    controller_params = os.path.join(
        get_package_share_directory("linear_slider_hardware_interface"), "config", "controller_params.yaml"
    )

    robot_ip_arg = DeclareLaunchArgument(
        name="robot_ip",
        default_value="169.254.97.178",
        description="IP address of the ClearCore controller for the Teknic ClearPath-MC servo motor.",
    )

    controller_manager_node = Node(
        package="linear_slider_hardware_interface",
        executable="ros2_control_node",
        parameters=[{"robot_description": ...}, controller_params],
        condition=UnlessCondition(use_sim),
    )

    return LaunchDescription([robot_ip_arg, controller_manager_node])
