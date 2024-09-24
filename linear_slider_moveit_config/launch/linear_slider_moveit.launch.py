#!/usr/bin/env python3
"""
This launch file adapted from: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_moveit_config/launch/ur_moveit.launch.py

    USEFUL SCRIPT FOR CHECKING IF YAML FILE SUBSTITUTIONS WORK
        # file = ParameterFile(moveit_joint_limits, allow_substs=True)
        # file.evaluate(context=context)
        # # logger.warn(f"{file.param_file}")

        # with open(file.param_file, 'r') as _file:
        #     __file = _file.read()

        # logger.warn(f"{__file}")

"""
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch.utilities import perform_substitutions
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from linear_slider_moveit_config.launch_common import load_yaml
from moveit_configs_utils import MoveItConfigsBuilder

import rclpy
import rclpy.logging

import os
import yaml
import json

logger = rclpy.logging.get_logger("linear_slider_moveit_config.launch_logger")


def launch_setup(context: LaunchContext, *args, **kwargs):
    """Callback function for launch setup using runtime context for evaluation and debugging."""

    # Launch configurations
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")
    description_file = LaunchConfiguration("description_file")
    description_pkg = LaunchConfiguration("description_pkg")
    joint_limits_file = LaunchConfiguration("joint_limits_file")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    moveit_semantic_description_file = LaunchConfiguration(
        "moveit_semantic_description_file"
    )
    moveit_runtime_config_pkg = LaunchConfiguration("moveit_runtime_config_pkg")
    parent = LaunchConfiguration("parent")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_sim_time = LaunchConfiguration("use_sim_time")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")

    # MoveIt Kinematics
    robot_description_kinematics_path = PathJoinSubstitution(
        [
            FindPackageShare(moveit_runtime_config_pkg),
            "config",
            "kinematics.yaml",
        ]
    )
    robot_description_kinematics_evaluated_file = ParameterFile(
        robot_description_kinematics_path, allow_substs=True
    )
    robot_description_kinematics_evaluated_file.evaluate(context=context)

    # MoveIt Joint Limits
    moveit_joint_limits_path = PathJoinSubstitution(
        [
            FindPackageShare(description_pkg),
            "config",
            joint_limits_file,
        ]
    )
    moveit_joint_limits_evaluated_file = ParameterFile(
        moveit_joint_limits_path, allow_substs=True
    )
    moveit_joint_limits_evaluated_file.evaluate(context=context)

    # Planning configuration
    ompl_planning_pipeline_config = dict(
        move_group=dict(
            planning_plugins=["ompl_interface/OMPLPlanner"],
            request_adapters="default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/AddRuckigTrajectorySmoothing default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixStartStatePathConstraints default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/Empty",
            response_adapters=[
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
        )
    )
    ompl_planning_content = load_yaml(
        package_name=str(moveit_runtime_config_pkg.perform(context=context)),
        file_path=os.path.join("config", "ompl_planning.yaml"),
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_content)

    # Controller configuration
    controllers_file = PathJoinSubstitution([
        FindPackageShare(moveit_runtime_config_pkg),
        "config",
        "controllers.yaml",
    ])
    controllers_yaml_evaluated_file = ParameterFile(
        controllers_file, allow_substs=True
    )
    controllers_yaml_evaluated_file.evaluate(context=context)
    controllers_content = load_yaml(
        package_name=str(moveit_runtime_config_pkg.perform(context=context)),
        file_path=os.path.join(
            "config", str(controllers_yaml_evaluated_file.param_file)
        ),
    )

    # The scaled_joint_trajectory_controller does not work on mock_hardware, switch to regular joint_trajectory_controller TODO: don't need all of this for the slider, just the UR
    change_controllers = context.perform_substitution(use_mock_hardware)
    if change_controllers.lower() == "true":
        if controllers_content is not None:
            controllers_content["scaled_joint_trajectory_controller"][
                "default"
            ] = False
            controllers_content["linear_slider_controller"]["default"] = True
    moveit_controllers_content = dict(
        moveit_simple_controller_manager=controllers_content,
        moveit_controller_manager="moveit_simple_controller_manager/MoveItSimpleControllerManager",
    )

    warehouse_ros_config = dict(
        warehouse_plugin="warehouse_ros_sqlite::DatabaseConnection",
        warehouse_host=warehouse_sqlite_path,
    )

    # MoveIt Configs Builder
    mcb = MoveItConfigsBuilder(
        robot_name="linear_slider", package_name="linear_slider_moveit_config"
    )
    mcb.robot_description(
        file_path=os.path.join(
            get_package_share_directory("linear_slider_description"),
            "urdf/linear_slider.urdf.xacro",
        ),
        mappings={
            "mock_sensor_commands": mock_sensor_commands,
            "parent": parent,
            "prefix": prefix,
            "use_mock_hardware": use_mock_hardware,
            # "sim_gazebo": sim_gazebo,
            # "sim_gazebo_classic": sim_gazebo_classic,
        },
    )
    mcb.robot_description_semantic(
        file_path=os.path.join(
            get_package_share_directory("linear_slider_moveit_config"),
            "srdf/linear_slider.srdf.xacro",
        ),
        mappings={"prefix": prefix},
    )
    mcb.robot_description_kinematics(
        file_path=robot_description_kinematics_evaluated_file.param_file
    )
    mcb.joint_limits(file_path=moveit_joint_limits_evaluated_file.param_file)
    mcb.trajectory_execution(
        file_path=os.path.join(
            get_package_share_directory("linear_slider_moveit_config"),
            "config/trajectory.yaml",
        ),
        moveit_manage_controllers=False,
    )
    mcb.planning_pipelines(
        default_planning_pipeline="ompl",
        pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"],
    ) 
    mcb.planning_scene_monitor()
    mcb.pilz_cartesian_limits(
        file_path=os.path.join(
            get_package_share_directory("linear_slider_moveit_config"),
            "config/pilz_cartesian_limits.yaml",
        )
    )

    moveit_config = mcb.to_moveit_configs()
    # logger.warn(f"{moveit_config.planning_scene_monitor}")
    # logger.warn(f"{moveit_config.robot_description['robot_description'].value[0].perform(context)}")

    # Start the actual move_group node/action server
    node_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            # robot_description_planning,
            moveit_config.joint_limits,
            moveit_config.trajectory_execution,
            # moveit_config.planning_pipelines,
            ompl_planning_pipeline_config,
            # trajectory_execution,
            moveit_config.pilz_cartesian_limits,
            moveit_controllers_content,
            # planning_scene_monitor_parameters,
            moveit_config.planning_scene_monitor,
            warehouse_ros_config,
        ],
        # remappings=[
        #     ("joint_states", f"{prefix.perform(context)}joint_states")
        # ],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [
            get_package_share_directory("linear_slider_moveit_config"),
            "rviz",
            "linear_slider_moveit.rviz",
        ]
    )
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            ompl_planning_pipeline_config,
            # moveit_config.planning_pipelines,
            warehouse_ros_config,
        ],
        condition=IfCondition(launch_rviz),
    )

    # MoveIt Servo
    servo_yaml_path = PathJoinSubstitution(
        [
            FindPackageShare(moveit_runtime_config_pkg),
            "config",
            "linear_slider_servo.yaml",
        ]
    )
    servo_yaml_evaluated_file = ParameterFile(
        servo_yaml_path, allow_substs=True
    )
    servo_yaml_evaluated_file.evaluate(context=context)
    servo_yaml_content = load_yaml(
        package_name=str(moveit_runtime_config_pkg.perform(context=context)),
        file_path=os.path.join(
            "config", str(servo_yaml_evaluated_file.param_file)
        ),
    )
    # logger.warn(f"{servo_yaml_content}")
    servo_params = dict(moveit_servo=servo_yaml_content)
    node_servo = Node(
        package="moveit_servo",
        executable="servo_node_main",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits
        ],
        condition=IfCondition(launch_servo),
    )

    # https://github.com/moveit/moveit2/blob/main/moveit_ros/moveit_servo/launch/demo_ros_api.launch.py
    # node_servo = ComposableNodeContainer(
    #     name="servo_node_container",
    #     namespace='/',
    #     package='rclcpp_components',
    #     executable='component_container_mt',
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package='moveit_servo',
    #             plugin="moveit_servo::ServoNode",
    #             name="servo_node",
    #             parameters=[
    #                 servo_params,
    #                 moveit_config.robot_description,
    #                 moveit_config.robot_description_semantic,
    #                 moveit_config.robot_description_kinematics,
    #                 moveit_config.joint_limits,
    #                 {"reliability": "reliable", "history": 'keep_last', "history_depth": 1}
    #             ],
    #             extra_arguments=[{'use_intra_process_comms': True}]
    #         ),
    #         # ComposableNode(
    #         #     package="joy",
    #         #     plugin="joy::Joy",
    #         #     name="joy_node",
    #         # ),
    #     ]
    # )

    nodes_to_start = [node_move_group, node_servo, node_rviz]

    return nodes_to_start


def generate_launch_description():
    declared_args = []
    declared_args.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="linear_slider.urdf.xacro",
            description="URDF/xacro description file of the robot.",
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "description_pkg",
            default_value="linear_slider_description",
            description="Description package with the robot URDF/xacro files. Usually, the argument is not set; it enables the use of a custom setup.",
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz window.",
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "launch_servo",
            default_value="true",
            description="Launch servoing node.",
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            choices=["true", "false"],
            description="Enable fake command interfaces for sensors for simple simulation. Use only if `use_mock_hardware` parameter is true.",
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "moveit_semantic_description_file",
            default_value="linear_slider.srdf.xacro",  # TODO: Get srdf.xacro like UR drivers
            description="MoveIt SRDF/xacro description file with the robot.",
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "moveit_runtime_config_pkg",
            default_value="linear_slider_moveit_config",
            description="MoveIt config package with robot SRDF/xacro files. Usually, the argument is not set; it enables the use of a custom setup.",
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "joint_limits_file",
            default_value="joint_limits.yaml",
            description="Joint limits that augment URDF robot description.",
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "parent",
            default_value="world",
            description="Parent joint of the robot. Typically the world, unless mounting to a mobile platform."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="linear_slider__",
            description="Prefix of the joint names. Useful for multi-robot setup. If changed, joint names in the controllers' configuration need to be updated, or dynamically configured in the launch file.",
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            choices=["true", "false"],
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path to where the warehouse database should be stored.",
        )
    )

    ld = LaunchDescription(
        declared_args + [OpaqueFunction(function=launch_setup)]
    )

    return ld
