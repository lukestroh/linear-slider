#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro
import os


def generate_launch_description():

    # Declare arguments
    declared_args = []
    declared_args.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value = "linear_slider_bringup",
            description = 'Package with the controller\'s configuration in the "config" folder. Usually, the argument is not set; it enables the use of a custom setup.'
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value = "linear_slider_velocity_controller.yaml",
            description = "YAML file with the controllers description."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "description_package",
            default_value = "linear_slider_description",
            description = "Description package with the robot URDF/xacro files. Usually, the argument is not sset; it enables the use of a custom setup."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "description_file",
            default_value = "linear_slider_velocity.urdf.xacro",
            description = "URDF/xacro description file of the robot."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "prefix",
            default_value = '""',
            description = "Prefix of the joint names, useful for multi-robot setup. If changed, then you need to update the joint names in the controllers' description."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value = "true",
            description = "Start robot with fake hardware mirroring command to its states."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value = "false",
            description = "Enable fake command interfaces for sensors for simple simulation. Use only if `use_mock_hardware` parameter is true."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value = "joint_trajectory_controller",
            choices = ["velocity_controllers", "joint_trajectory_controller"], # add another here if we want to switch between different controllers
            description = "Robot controller"
        )
    )

    # Initialize args
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    robot_controller = LaunchConfiguration("robot_controller")

    # Get URDF from xacro
    urdf_file = os.path.join(
        FindPackageShare('linear_slider_description').find('linear_slider_description'),
        "urdf",
        "linear_slider_velocity.urdf.xacro"
    )
    xacro_file = xacro.process_file(
        urdf_file,
        prefix=prefix,
        use_mock_hardware=use_mock_hardware,
        mock_sensor_commands=mock_sensor_commands
    )
    robot_description_content = xacro_file.toprettyxml()


    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution([
        FindPackageShare(runtime_config_package),
        "config",
        controllers_file
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(description_package),
        "rviz",
        "linear_slider.rviz"
    ])

    _log = LogInfo(msg=urdf_file)

    control_node = Node(
        package = "controller_manager",
        executable = "ros2_control_node",
        output = "both",
        parameters = [
            # robot_description, # Deprecated: Automatically subscribes to "/robot_description" topic from the /controller_manager node
            robot_controllers
        ]
    )

    robot_state_pub_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        output = "both",
        parameters = [robot_description]
    )

    gazebo_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "linear_slider"
        ],
        output="screen"
    )

    rviz_node = Node(
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        output = "log",
        arguments = ["-d", rviz_config_file]
    )

    joint_state_broadcaster_spawner = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = [
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ]
    )


    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )



    return LaunchDescription(
        declared_args
        + [
            _log,
            robot_state_pub_node,
            # control_node,
            gazebo_node,
            gazebo,
            joint_state_broadcaster_spawner
        ]
    )
