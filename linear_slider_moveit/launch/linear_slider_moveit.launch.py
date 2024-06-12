#!/usr/bin/env python3


"""
This launch file adapted from: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_moveit_config/launch/ur_moveit.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution
)
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from linear_slider_moveit.launch_common import load_yaml

import rclpy
import rclpy.logging

import os
import yaml

logger = rclpy.logging.get_logger("launch_logger")


def launch_setup(context, *args, **kwargs):
    """Callback function for launch setup using runtime context for evaluation and debugging."""
    moveit_runtime_config_pkg = LaunchConfiguration("moveit_runtime_config_pkg")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    description_pkg = LaunchConfiguration("description_pkg")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")

    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")

    # Get URDF from xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_pkg), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
        ]
    )


    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare(moveit_runtime_config_pkg),
                "srdf",
                moveit_config_file
            ]),
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )

    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics_file = PathJoinSubstitution([
        FindPackageShare(moveit_runtime_config_pkg),
        "config",
        "kinematics.yaml"
    ])

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            package_name=str(moveit_runtime_config_pkg.perform(context=context)),
            file_path=os.path.join("config", "kinematics.yaml")
        )
    }

    moveit_joint_limits_path = PathJoinSubstitution([
        FindPackageShare(moveit_runtime_config_pkg),
        "config",
        moveit_joint_limits_file
    ])

    moveit_joint_limits_evaluated_file = ParameterFile(moveit_joint_limits_path, allow_substs=True)
    moveit_joint_limits_evaluated_file.evaluate(context=context)

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            package_name=str(moveit_runtime_config_pkg.perform(context=context)),
            file_path=os.path.join("config", str(moveit_joint_limits_evaluated_file.param_file)) # .param_file gets name of temp yaml file created by ROS2
        )
    }

    """ USEFUL SCRIPT FOR CHECKING IF YAML FILE SUBSTITUTIONS WORK
        # file = ParameterFile(moveit_joint_limits, allow_substs=True)
        # file.evaluate(context=context)
        # # logger.warn(f"{file.param_file}")

        # with open(file.param_file, 'r') as _file:
        #     __file = _file.read()

        # logger.warn(f"{__file}") """

    # Planning configuration
    ompl_planning_pipeline_config = dict(
        move_group=dict(
            planning_plugins=["ompl_interface/OMPLPlanner"],
            request_adapters=[
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            response_adapters=[
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
        )
    )    

    ompl_planning_content = load_yaml(
        package_name=str(moveit_runtime_config_pkg.perform(context=context)),
        file_path=os.path.join("config", "ompl_planning.yaml")
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_content)

    # Trajectory execution configuration
    controllers_file = PathJoinSubstitution([
        FindPackageShare(moveit_runtime_config_pkg),
        "config",
        "controllers.yaml"
    ])
    controllers_yaml_evaluated_file = ParameterFile(controllers_file, allow_substs=True)
    controllers_yaml_evaluated_file.evaluate(context=context)
    controllers_content = load_yaml(
        package_name=str(moveit_runtime_config_pkg.perform(context=context)),
        file_path=os.path.join("config", str(controllers_yaml_evaluated_file.param_file))
    )

    # The scaled_joint_trajectory_controller does not work on mock_hardware, switch to regular joint_trajectory_controller
    change_controllers = context.perform_substitution(use_mock_hardware)
    if change_controllers == "true":
        controllers_content["scaled_joint_trajectory_controller"]["default"] = False
        controllers_content["joint_trajectory_controller"]["default"] = True

    moveit_controllers_content = dict(
        moveit_simple_controller_manager=controllers_content,
        moveit_controller_manager="moveit_simple_controller_manager/MoveItSimpleControllerManager"
    )

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_during_scaling": 1.2, # Dear UR, isn't assigning class attributes within a python launch string a bit hacky????
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01
    }

    planning_scene_monitor_parameters = dict(
        publish_planning_scene=True,
        publish_geometry_updates=True,
        publish_state_updates=True,
        publish_transform_updates=True
    )

    warehouse_ros_config = dict(
        warehouse_plugin="warehouse_ros_sqlite::DatabaseConnection",
        warehouse_host=warehouse_sqlite_path
    )


    # logger.warn(f"{moveit_controllers_content}")

    # Start the actual move_group node/action server
    node_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics_file,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers_content,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            warehouse_ros_config
        ]
    )

    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare(moveit_runtime_config_pkg), "rviz", "view_robot.rviz"]
    # )

    # node_rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2_moveit",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     parameters=[
    #         robot_description,
    #         robot_description_semantic,
    #         robot_description_kinematics,
    #         robot_description_planning,
    #         ompl_planning_pipeline_config,
    #         warehouse_ros_config
    #     ],
    #     condition=IfCondition(launch_rviz)
    # )

    # servo_yaml = load_yaml(package_name="linear_slider_moveit", file_path="config/ur_servo.yaml")

    # servo_params = dict(moveit_servo=servo_yaml)
    # servo_node = Node(
    #     package="moveit_servo",
    #     executable="servo_node_main",
    #     output="screen",
    #     parameters=[
    #         servo_params,
    #         robot_description,
    #         robot_description_semantic
    #     ],
    #     condition=IfCondition(launch_servo)
    # )

    # nodes_to_start = [node_move_group, node_rviz, servo_node]

    nodes_to_start = [node_move_group]

    return nodes_to_start

def generate_launch_description():
    declared_args = []
    declared_args.append(
        DeclareLaunchArgument(
            "moveit_runtime_config_pkg",
            default_value="linear_slider_moveit",
            description="MoveIt config package with robot SRDF/xacro files. Usually, the argument is not set; it enables the use of a custom setup."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="linear_slider.srdf.xacro", # TODO: Get srdf.xacro like UR drivers
            description="MoveIt SRDF/xacro description file with the robot."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "moveit_joint_limits_file",
            default_value="joint_limits.yaml",
            description="MoveIt joint limits that augment or override the values from the URDF robot description."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "description_pkg",
            default_value="linear_slider_description",
            description="Description package with the robot URDF/xacro files. Usually, the argument is not set; it enables the use of a custom setup."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "description_file",
            default_value = "linear_slider.urdf.xacro",
            description = "URDF/xacro description file of the robot."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint names. Useful for multi-robot setup. If changed, joint names in the controllers' configuration need to be updated, or dynamically configured in the launch file."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            choices=['true', 'false'],
            description = "Start robot with fake hardware mirroring command to its states."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value = "false",
            choices=["true", "false"],
            description = "Enable fake command interfaces for sensors for simple simulation. Use only if `use_mock_hardware` parameter is true."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path to where the warehouse database should be stored."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz window."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "launch_servo",
            default_value="true",
            description="Launch servoing node."
        )
    )

    ld = LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])

    return ld