#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
    LogInfo
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    Command,
    FindExecutable
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



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
            default_value = "linear_slider_controllers.yaml",
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
            default_value = "linear_slider.urdf.xacro",
            description = "URDF/xacro description file of the robot."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "prefix",
            default_value = "''",
            description = "Prefix of the joint names, useful for multi-robot setup. If changed, then you need to update the joint names in the controllers' description."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value = "false",
            choices=['true', 'false'],
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
            choices = ["velocity_controller", "joint_trajectory_controller"], # add another here if we want to switch between different controllers
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
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
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

    _log0 = LogInfo(msg=robot_description_content)

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

    control_node = Node(
        package = "controller_manager",
        executable = "ros2_control_node",
        output = "both",
        parameters = [
            robot_controllers,
            robot_description # Says it's deprecated, but only works if this is provided!
        ]
    )

    robot_state_pub_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        output = "both",
        parameters = [robot_description]
    )

    rviz_node = Node(
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        output = "log",
        arguments = ["-d", rviz_config_file]
    )

    timeout_duration = DeclareLaunchArgument(
        'timeout',
        default_value='120',  # Default timeout value (in seconds)
        description='Timeout duration for joint_state_broadcaster_spawner (in seconds)'
    )

    joint_state_broadcaster_spawner = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = [
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[
            {"timeout": LaunchConfiguration("timeout")}
        ]
    )

    

    robot_controllers = [robot_controller]
    robot_controller_spawners = []
    for controller in robot_controllers:
        robot_controller_spawners.append(
            Node(
                package = "controller_manager",
                executable = "spawner",
                arguments = [controller, "-c", "/controller_manager"]
            )
        )

    # Delay loading and activation of 'joint_state_broadcaster' after the start of ros2_control_node
    delay_joint_state_broadcaster_after_ros2_control_node = (
        RegisterEventHandler(
            event_handler = OnProcessStart(
                target_action = control_node,
                on_start=[joint_state_broadcaster_spawner]
            )
        )
    )

    # Delay rviz start after joint_state_broadcaster to avoid unnecessary warning output
    delay_rviz_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler = OnProcessStart(
                target_action = joint_state_broadcaster_spawner,
                on_start = [rviz_node]
            )
        )
    )

    # Delay loading and activation of robot_controller after 'joint_state_broadcaster'
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for controller in robot_controller_spawners:
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner.append(
            RegisterEventHandler(
                event_handler = OnProcessExit(
                    target_action = joint_state_broadcaster_spawner,
                    on_exit = [controller]
                )
            )
        )

    return LaunchDescription(
        
        declared_args
        + [
            # _log0,\
            timeout_duration,
            control_node,
            robot_state_pub_node,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_joint_state_broadcaster_after_ros2_control_node
          ]
        + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner
    )

if __name__ == "__main__":
    generate_launch_description()