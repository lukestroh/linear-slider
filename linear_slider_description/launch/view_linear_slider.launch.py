from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, LogInfo
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare args
    declared_args: list = []

    declared_args.append(
        DeclareLaunchArgument(
            "linear_slider_description_pkg",
            default_value="linear_slider_description",
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
            default_value = "false",
            description = "Enable fake command interfaces for sensors for simple simulation. Use only if `use_mock_hardware` parameter is true."
        )
    )

    declared_args.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names. Useful for multi-robot setup. If changed, joint names in the controllers' configuration need to be updated."
        )
    )

    # Initialize Args
    package_description = LaunchConfiguration("linear_slider_description_pkg")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")

    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(package=package_description), "urdf", "linear_slider.urdf.xacro"]),
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
    ])

    log0 = LogInfo(msg=robot_description_content)

    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package=package_description), "rviz", "linear_slider.rviz"]
    )

    joint_state_publisher_node = Node(
        package = "joint_state_publisher_gui",
        executable = "joint_state_publisher_gui"
    )

    robot_state_publisher_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        output = "both",
        parameters = [robot_description]
    )

    rviz_node = Node(
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        output = "both",
        arguments = ["-d", rviz_config_file]
    )

    delay_rviz_after_joint_state_publisher_node = RegisterEventHandler(
        event_handler = OnProcessStart(
            target_action = joint_state_publisher_node,
            on_start = [
                TimerAction(
                    period = 2.0,
                    actions = [rviz_node]
                )
            ]
        )
    )

    return LaunchDescription(
        declared_args
        + [
            log0,
            joint_state_publisher_node,
            robot_state_publisher_node,
            delay_rviz_after_joint_state_publisher_node
        ]
    )