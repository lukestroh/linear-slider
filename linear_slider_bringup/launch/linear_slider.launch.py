#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, TimerAction
from launch.conditions import UnlessCondition, IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart, OnExecutionComplete
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command, FindExecutable
from launch_ros.actions import Node, LifecycleNode, ComposableNodeContainer
from launch_ros.event_handlers import OnStateTransition
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

import rclpy.logging

logger = rclpy.logging.get_logger("linear_slider_bringup.logger")


def generate_launch_description():
    # Declare arguments
    declared_args = []
    declared_args.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="linear_slider_bringup",
            description='Package with the controller\'s configuration in the "config" folder. Usually, the argument is not set; it enables the use of a custom setup.',
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="linear_slider_controllers.yaml",
            description="YAML file with the controllers description.",
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="linear_slider_description",
            description="Description package with the robot URDF/xacro files. Usually, the argument is not set; it enables the use of a custom setup.",
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="linear_slider.urdf.xacro",
            description="URDF/xacro description file of the robot.",
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="linear_slider_moveit_config",
            description="MoveIt2 configuration package for the linear slider.",
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "use_moveit", default_value="true", description="Use MoveIt2. Determines which RViz instance to run."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="linear_slider__",
            description="Prefix of the joint names, useful for multi-robot setup. Joint name parameters should be updated in URDF and YAML files.",
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
            "mock_sensor_commands",
            default_value="false",
            choices=["true", "false"],
            description="Enable fake command interfaces for sensors for simple simulation. Use only if `use_mock_hardware` parameter is true.",
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="linear_slider_controller",
            choices=[
                "linear_slider_controller",
                "joint_trajectory_controller"
            ],  # add another here if we want to switch between different controllers
            description="Robot controller",
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time. Default false.",
            choices=["true", "false"]
        )
    )

    # Initialize args
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    prefix = LaunchConfiguration("prefix")
    robot_controller = LaunchConfiguration("robot_controller")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_moveit = LaunchConfiguration("use_moveit")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Get URDF from xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
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

    robot_controllers = PathJoinSubstitution([FindPackageShare(runtime_config_package), "config", controllers_file])

    rviz_config_file = PathJoinSubstitution([FindPackageShare(description_package), "rviz", "linear_slider.rviz"])

    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            ParameterFile(robot_controllers, allow_substs=True),
            robot_description,  # Says it's deprecated, but only works if this is provided!
        ],
    )

    node_robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=UnlessCondition(use_moveit),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    limit_switch_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "limit_switch_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ]
    )

    robot_controllers = [robot_controller]
    robot_controller_spawners = []
    for controller in robot_controllers:
        robot_controller_spawners.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"]
            )
        )

    # node_lim_switch_broadcaster = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "limit_switch_state_broadcaster",
    #         "--controller_manager",
    #         "/controller_manager"
    #     ]
    # )
    # robot_controller_spawners.append(node_lim_switch_broadcaster)

    lifecycle_node_delay_jsb = LifecycleNode(
        name="delay_jsb_node_spawner",
        namespace="",
        package="linear_slider_bringup",
        executable="delay_joint_state_broadcaster_lifecycle_node.py",
        output="both",
    )

    register_event_for_slider_on_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lifecycle_node_delay_jsb,
            goal_state="finalized",
            entities=[joint_state_broadcaster_spawner, limit_switch_state_broadcaster_spawner],
        )
    )

    # Delay rviz start after joint_state_broadcaster to avoid unnecessary warning output
    register_event_delay_rviz_after_JSB_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(target_action=joint_state_broadcaster_spawner, on_start=[node_rviz])
    )

    # Delay loading and activation of robot_controller after 'joint_state_broadcaster'
    register_events_delay_robot_controller_spawners_after_JSB_spawner = []
    for controller in robot_controller_spawners:
        register_events_delay_robot_controller_spawners_after_JSB_spawner.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[controller])
            )
        )

    # MoveIt launch
    filepath_moveit_config_package = PathJoinSubstitution(
        [get_package_share_directory("linear_slider_moveit_config"), "launch", "linear_slider_moveit.launch.py"] # TODO: For some reason this wasn't working with FindPackageShare. Why ?
    )
    launch_linear_slider_moveit = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(filepath_moveit_config_package),
        launch_arguments=[
            ("use_mock_hardware", use_mock_hardware),
            ("mock_sensor_commands", mock_sensor_commands),
            ("use_sim_time", use_sim_time),
        ],
        condition=IfCondition(use_moveit)
    )

    return LaunchDescription(
        declared_args
        + [
            # node_sim_time_publisher,
            lifecycle_node_delay_jsb,
            node_controller_manager,
            node_robot_state_pub,
            register_event_delay_rviz_after_JSB_spawner,
            register_event_for_slider_on_activate,
            launch_linear_slider_moveit,
        ]
        + register_events_delay_robot_controller_spawners_after_JSB_spawner
    )


if __name__ == "__main__":
    generate_launch_description()
