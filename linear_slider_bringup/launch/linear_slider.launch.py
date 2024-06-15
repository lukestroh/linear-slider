#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, TimerAction, LogInfo
from launch.conditions import UnlessCondition, IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart, OnExecutionComplete
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command, FindExecutable
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


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
            default_value="linear_slider_moveit",
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
            description="Prefix of the joint names, useful for multi-robot setup. If changed, then you need to update the joint names in the controllers' description.",
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
            default_value="joint_trajectory_controller",
            choices=[
                "joint_trajectory_controller"
            ],  # add another here if we want to switch between different controllers
            description="Robot controller",
        )
    )

    # Initialize args
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    use_moveit = LaunchConfiguration("use_moveit")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    robot_controller = LaunchConfiguration("robot_controller")

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

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            ParameterFile(robot_controllers, allow_substs=True),
            robot_description,  # Says it's deprecated, but only works if this is provided!
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=UnlessCondition(use_moveit),
    )

    # timeout_duration = DeclareLaunchArgument(
    #     'timeout',
    #     default_value='120',  # Default timeout value (in seconds)
    #     description='Timeout duration for joint_state_broadcaster_spawner (in seconds)'
    # )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controllers = [robot_controller]
    robot_controller_spawners = []
    for controller in robot_controllers:
        robot_controller_spawners.append(
            Node(
                package="controller_manager", executable="spawner", arguments=[controller, "-c", "/controller_manager"]
            )
        )

    delay_lifecycle_node = LifecycleNode(
        name="delay_jsb_node_spawner",
        namespace="",
        package="linear_slider_bringup",
        executable="delay_joint_state_broadcaster_lifecycle_node.py",
        output="both",
    )

    # # Delay loading and activation of 'joint_state_broadcaster' after the start of ros2_control_node
    # delay_joint_state_broadcaster_after_ros2_control_node = (
    #     RegisterEventHandler(
    #         event_handler = OnExecutionComplete(
    #             target_action = delay_lifecycle_node,
    #             on_completion=[joint_state_broadcaster_spawner]
    #         )
    #     )
    # )

    register_event_for_slider_on_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=delay_lifecycle_node,
            goal_state="finalized",
            entities=[joint_state_broadcaster_spawner],
        )
    )

    # Delay rviz start after joint_state_broadcaster to avoid unnecessary warning output
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(target_action=joint_state_broadcaster_spawner, on_start=[rviz_node])
    )

    # MoveIt launch
    moveit_config_package_path = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "launch", "linear_slider_moveit.launch.py"]
    )
    launch_linear_slider_moveit = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(moveit_config_package_path),
        launch_arguments=[
            ("use_mock_hardware", use_mock_hardware),
            ("mock_sensor_commands", mock_sensor_commands),
            ("use_sim_time", "true"),
        ],
        condition=IfCondition(use_moveit)
    )

    # Delay loading and activation of robot_controller after 'joint_state_broadcaster'
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for controller in robot_controller_spawners:
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[controller])
            )
        )

    return LaunchDescription(
        declared_args
        + [
            delay_lifecycle_node,
            control_node,
            robot_state_pub_node,
            delay_rviz_after_joint_state_broadcaster_spawner,
            # delay_joint_state_broadcaster_after_ros2_control_node,
            register_event_for_slider_on_activate,
            launch_linear_slider_moveit,
        ]
        + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner
    )


if __name__ == "__main__":
    generate_launch_description()
