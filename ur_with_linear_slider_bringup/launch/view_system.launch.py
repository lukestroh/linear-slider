from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, OpaqueFunction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import rclpy.logging

logger = rclpy.logging.get_logger("ur_with_linear_slider_bringup.view_system_launch")

def launch_setup(context: LaunchContext, *args, **kwargs):
    # Initialize Args
    package_description = LaunchConfiguration("ur_with_linear_slider_pkg")
    prefix = LaunchConfiguration("prefix")
    tf_prefix = LaunchConfiguration("tf_prefix")
    ur_type = LaunchConfiguration("ur_type")
    ur_parent = LaunchConfiguration("ur_parent")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")

    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(package=package_description), "urdf", "ur_with_linear_slider.urdf.xacro"]),
        " ",
        "prefix:=",
        prefix,
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
        "use_mock_hardware:=",
        use_mock_hardware,
        " ",
        "mock_sensor_commands:=",
        mock_sensor_commands,
        " ",
    ])

    robot_description = {"robot_description": robot_description_content}

    # logger.warn(f"{robot_description_content.perform(context=context)}")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package=package_description), "rviz", "ur_with_linear_slider.rviz"]
    )

    node_joint_state_publisher = Node(
        package = "joint_state_publisher_gui",
        executable = "joint_state_publisher_gui",
        name = "joint_state_publisher_gui"
    )

    node_robot_state_publisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        output = "both",
        parameters = [robot_description]
    )

    node_rviz = Node(
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        output = "both",
        arguments = ["-d", rviz_config_file]
    )

    event_handler_delay_rviz_after_joint_state_publisher_node = RegisterEventHandler(
        event_handler = OnProcessStart(
            target_action = node_joint_state_publisher,
            on_start = [node_rviz]
        )
    )

    nodes_to_run = [
        node_joint_state_publisher,
        node_robot_state_publisher,
        event_handler_delay_rviz_after_joint_state_publisher_node
    ]
    return nodes_to_run

def generate_launch_description():
    # Declare args
    declared_args: list = []
    declared_args.append(
        DeclareLaunchArgument(
            "ur_with_linear_slider_pkg",
            default_value="ur_with_linear_slider_bringup",
            description="Description package of the linear slider. Enables the use of a custom robot description."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value = "true",
            description = "Enable fake command interfaces for sensors for simple simulation. Use only if `use_mock_hardware` parameter is true."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='linear_slider__',
            description="Prefix of the linear slider joint names. Useful for multi-robot setup. If changed, joint names in the controllers' configuration need to be updated."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='ur_robot__',
            description="Prefix of the UR robot joint names. Useful for multi-robot setup. If changed, joint names in the controllers' configuration need to be updated."
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

    ld = LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])

    return ld
