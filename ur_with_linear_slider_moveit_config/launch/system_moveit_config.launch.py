#!/usr/bin/env python3
"""
# file = ParameterFile(moveit_joint_limits, allow_substs=True)
# file.evaluate(context=context)
# # logger.warn(f"{file.param_file}")

# with open(file.param_file, 'r') as _file:
#     __file = _file.read()

# logger.warn(f"{__file}")
"""
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
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

logger = rclpy.logging.get_logger("ur_with_linear_slider_moveit_config.launch")

def launch_setup(context: LaunchContext, *args, **kwargs):
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    prefix = LaunchConfiguration("prefix")
    system_description_file = LaunchConfiguration("system_description_file")
    system_description_package = LaunchConfiguration("system_description_package")
    system_moveit_joint_limits_file = LaunchConfiguration('system_moveit_joint_limits_file')
    system_moveit_config_package = LaunchConfiguration("system_moveit_config_package")
    system_semantic_description_file = LaunchConfiguration("system_semantic_description_file")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_sim_time = LaunchConfiguration("use_sim_time")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")

    # UR robot config
    ur_type = LaunchConfiguration("ur_type")
    tf_prefix = LaunchConfiguration("tf_prefix")
    ur_parent = LaunchConfiguration("ur_parent")
    ur_robot_ip = LaunchConfiguration("ur_robot_ip")
    ur_runtime_package = LaunchConfiguration("ur_runtime_package")
    ur_controllers_file = LaunchConfiguration("ur_controllers_file")

    # Get URDF from xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(system_description_package), "urdf", system_description_file]),
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
            "tf_prefix:=",
            tf_prefix,
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "ur_parent:=",
            ur_parent,
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Get SRDF from xacro
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(system_moveit_config_package), "srdf", system_semantic_description_file]),
            " ",
            "prefix:=",
            prefix,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
        ]
    )
    # logger.warn(f"{robot_description_semantic_content.perform(context)}")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    # MoveIt Kinematics
    filepath_system_kinematics_path = PathJoinSubstitution(
        [FindPackageShare(system_moveit_config_package), "config", "kinematics.yaml"]
    )
    parameterfile_moveit_kinematics = ParameterFile(filepath_system_kinematics_path, allow_substs=True)
    parameterfile_moveit_kinematics.evaluate(context=context)
    yamlcontent_system_kinematics = load_yaml(
        package_name=str(system_moveit_config_package.perform(context=context)),
        file_path=os.path.join("config", str(parameterfile_moveit_kinematics.param_file)),
    )
    filepath_tmp_kinematics = PathJoinSubstitution(
        [  # For some reason, this temp file gets deleted pretty quickly, so rewrite the contents of that temp file to a slightly-more-permanent temp file in /linear_slider_moveit/config/tmp/
            FindPackageShare(system_moveit_config_package),
            "config",
            "tmp",
            "tmp_kinematics.yaml",
        ]
    )
    _sv_path = filepath_tmp_kinematics.perform(context=context)
    with open(_sv_path, "w") as file:
        yaml.safe_dump(yamlcontent_system_kinematics, file)

    # MoveIt Joint Limits
    filepath_moveit_joint_limits = PathJoinSubstitution(
        [FindPackageShare(system_moveit_config_package), "config", system_moveit_joint_limits_file]
    )
    parameterfile_moveit_joint_limits = ParameterFile(filepath_moveit_joint_limits, allow_substs=True)
    parameterfile_moveit_joint_limits.evaluate(context=context)
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            package_name=str(system_moveit_config_package.perform(context=context)),
            file_path=os.path.join(
                "config", str(parameterfile_moveit_joint_limits.param_file)
            ),  # .param_file gets name of temp yaml file created by ROS2
        )
    }

    # Planning configuration
    ompl_planning_pipeline_config = dict(
        move_group=dict(
            planning_plugins=["ompl_interface/OMPLPlanner"],
            request_adapters= "default_planner_request_adapters/AddRuckigTrajectorySmoothing default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/Empty default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/ResolveConstraintFrames",

            response_adapters=[
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
        )
    )
    filepath_ompl_planning = PathJoinSubstitution([
        FindPackageShare(system_moveit_config_package),
        "config",
        "ompl_planning.yaml"
    ])
    parameterfile_ompl_planning = ParameterFile(filepath_ompl_planning, allow_substs=True)
    parameterfile_ompl_planning.evaluate(context=context)
    ompl_planning_content = load_yaml(
        package_name=str(system_moveit_config_package.perform(context=context)),
        file_path=os.path.join("config", parameterfile_ompl_planning.param_file),
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_content)

    # MoveIt Trajectory controllers
    filepath_moveit_controllers = PathJoinSubstitution([
        FindPackageShare(system_moveit_config_package),
        "config",
        "moveit_controllers.yaml"
    ])
    parameterfile_moveit_controllers = ParameterFile(filepath_moveit_controllers, allow_substs=True)
    parameterfile_moveit_controllers.evaluate(context=context)
    yamlcontent_moveit_controllers = load_yaml(
        package_name=str(system_moveit_config_package.perform(context=context)),
        file_path=os.path.join("config", str(parameterfile_moveit_controllers.param_file)),
    )

    # The scaled_joint_trajectory_controller does not work on mock_hardware, switch to regular joint_trajectory_controller
    change_controllers = context.perform_substitution(use_mock_hardware)
    if change_controllers.lower() == "true":
        yamlcontent_moveit_controllers["scaled_joint_trajectory_controller"]["default"] = False
        yamlcontent_moveit_controllers["joint_trajectory_controller"]["default"] = True
        logger.warn("GOT HERE")

    moveit_controllers = dict(
        moveit_simple_controller_manager=yamlcontent_moveit_controllers,
        moveit_controller_manager="moveit_simple_controller_manager/MoveItSimpleControllerManager",
    )
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_during_scaling": 1.2, # TODO: Dynamically assign values to these!!
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_parameters = dict(
        publish_planning_scene=True,
        publish_geometry_updates=True,
        publish_state_updates=True,
        publish_transform_updates=True,
    )
    warehouse_ros_config = dict(
        warehouse_plugin="warehouse_ros_sqlite::DatabaseConnection", warehouse_host=warehouse_sqlite_path
    )

    mcb = MoveItConfigsBuilder(robot_name="ur_with_linear_slider", package_name="ur_with_linear_slider_moveit_config")
    mcb.robot_description(
        file_path=os.path.join(
            FindPackageShare(system_description_package).perform(context=context),
            "urdf",
            system_description_file.perform(context=context)
        ),
        mappings=robot_description
    )
    mcb.robot_description_semantic(
        file_path=os.path.join(
            FindPackageShare(system_moveit_config_package).perform(context=context),
            "srdf",
            system_semantic_description_file.perform(context=context)
        ),
        mappings=robot_description_semantic
    )
    mcb.robot_description_kinematics(
        file_path=os.path.join(
            FindPackageShare(system_moveit_config_package).perform(context=context),
            "config/tmp/tmp_kinematics.yaml"
        ),
    )
    mcb.planning_pipelines(
        default_planning_pipeline="ompl",
        pipelines=[
            "ompl", "pilz_industrial_motion_planner", "chomp"
        ]
    )
    moveit_config = mcb.to_moveit_configs()

    # Start the actual move_group node/action server
    node_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            # # robot_description_kinematics_path,
            moveit_config.robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_parameters,
            # {"use_sim_time": use_sim_time},
            warehouse_ros_config,
        ],
    )

    # RViz
    filepath_rviz_config = PathJoinSubstitution(
        [FindPackageShare(system_moveit_config_package), "rviz", "ur_with_linear_slider_moveit.rviz"]
    )
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", filepath_rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            moveit_config.robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            warehouse_ros_config,
        ],
        condition=IfCondition(launch_rviz),
    )

    # MoveIt Servo
    filepath_servo_config = PathJoinSubstitution(
        [FindPackageShare(system_moveit_config_package), "config", "servo.yaml"]
    )
    parameterfile_servo_config = ParameterFile(filepath_servo_config, allow_substs=True)
    parameterfile_servo_config.evaluate(context=context)
    yamlcontent_servo_config = load_yaml(
        package_name=str(system_moveit_config_package.perform(context=context)),
        file_path=os.path.join("config", str(parameterfile_servo_config.param_file)),
    )
    servo_params = dict(moveit_servo=yamlcontent_servo_config)
    node_servo = Node(
        package="moveit_servo",
        executable="servo_node_main",
        output="screen",
        parameters=[servo_params, robot_description, robot_description_semantic, moveit_config.robot_description_kinematics],
        condition=IfCondition(launch_servo)
    )

    nodes_to_run = [
        node_move_group,
        node_rviz,
        node_servo
    ]

    return nodes_to_run


def generate_launch_description():
    # Declare args
    declared_args = []
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
            "prefix",
            default_value="linear_slider__",
            description="Prefix of the joint names, useful for multi-robot setup. Joint name parameters should be updated in URDF and YAML files."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "system_description_file",
            default_value="ur_with_linear_slider.urdf.xacro",
            description="URDF/xacro description file of the robot."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "system_description_package",
            default_value="ur_with_linear_slider_bringup",
            description="Description package with the robot URDF/xacro files. Usually, the argument is not set; it enables the use of a custom setup."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "system_moveit_joint_limits_file",
            default_value="joint_limits.yaml",
            description="MoveIt joint limits that augment or override the values from the URDF robot description.",
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "system_moveit_config_package",
            default_value="ur_with_linear_slider_moveit_config",
            description="MoveIt config package with robot SRDF/xacro files. Usually, the argument is not set; it enables the use of a custom setup."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "system_semantic_description_file",
            default_value="ur_with_linear_slider.srdf.xacro",  # TODO: Get srdf.xacro like UR drivers
            description="MoveIt SRDF/xacro description file with the robot.",
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='ur_robot__',
            description="Prefix of the UR robot joint names. Useful for multi-robot setup."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="UR robot type."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "ur_parent",
            default_value="linear_slider__tool0", # TODO: dynamically configure
            description="Parent link of the linear slider for joint attachment. Requires changing the URDF of the UR5 to accept a parent command. Otherwise, the UR5 gets fixed to the world by default."
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
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path to where the warehouse database should be stored.",
        )
    )


    ld = LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])

    return ld
