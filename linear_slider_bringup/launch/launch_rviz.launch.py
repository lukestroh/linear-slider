from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    description_package_arg = DeclareLaunchArgument(
        "description_package",
        default_value = "linear_slider_description",
        description = "Description package with the robot URDF/xacro files. Usually, the argument is not sset; it enables the use of a custom setup."
    )
    description_package = LaunchConfiguration("description_package")
    rviz_config_file = os.path.join(
        FindPackageShare("linear_slider_description").find("linear_slider_description"),
        "rviz",
        "linear_slider.rviz"
    )

    rviz_node = Node(
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        output = "screen",
        arguments = ["-d", rviz_config_file]
    )

    return LaunchDescription([
        description_package_arg,
        # rviz_config_file,
        rviz_node
    ])