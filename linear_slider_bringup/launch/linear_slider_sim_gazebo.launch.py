#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    Command,
    FindExecutable,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():

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
            default_value = '""',
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
    declared_args.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="false",
            description="Simulate within the Gazebo Ignition environment"
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "sim_gazebo_classic",
            default_value="false",
            description="Simulate within the Gazebo Classic environment."
        )
    )

    # Initialize args
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    robot_controller = LaunchConfiguration("robot_controller")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    sim_gazebo_classic = LaunchConfiguration("sim_gazebo_classic")

    # robot_controllers = PathJoinSubstitution(
    #     [FindPackageShare(runtime_config_package), "config", controllers_file]
    # )

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
            "sim_gazebo:=", 
            sim_gazebo,
            " ",
            "sim_gazebo_classic:=",
            sim_gazebo_classic,
            " ",
            # "simulation_controllers:=",
            # robot_controllers,
            # " ",

        ]
    )

    robot_description = {"robot_description": robot_description_content}

    _log0 = LogInfo(msg=robot_description_content)
    _log1 = LogInfo(msg=sim_gazebo)
    _log2 = LogInfo(msg=sim_gazebo_classic)

    robot_state_pub_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        output = "both",
        parameters = [robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = [
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
        ],
    )

    robot_controllers = [robot_controller]
    robot_controller_spawners = []
    for controller in robot_controllers:
        robot_controller_spawners.append(
            Node(
                package = "controller_manager",
                executable = "spawner",
                arguments = [controller, "-c", "/controller_manager"],
            )
        )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros_ign_gazebo"),
            "/launch",
            "/ign_gazebo.launch.py"
        ]),
        launch_arguments={"ign_args": " -r -v 3 empty.sdf"}.items(),
        condition=IfCondition(sim_gazebo)
    )

    gazebo_classic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_ros"),
            "/launch",
            "/gazebo.launch.py"
        ]),
        condition=IfCondition(sim_gazebo_classic)
    )

    gazebo_node_spawner = Node(
        package="ros_ign_gazebo", #"ros_gz_sim"
        executable="create",
        name="spawn_linear_slider",
        arguments=["-name", "linear_slider", "-topic", "robot_description"],
        condition=IfCondition(sim_gazebo),
        output="screen"
    )

    gazebo_classic_node_spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_linear_slider",
        arguments=["-entity", "linear_slider", "-topic", "robot_description"],
        condition=IfCondition(sim_gazebo_classic),
        output="screen"
    )

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(description_package),
        "rviz",
        "linear_slider.rviz"
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    delay_joint_state_broadcaster_spawner_after_gazebo_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gazebo_node_spawner,
            on_start=[joint_state_broadcaster_spawner],
        ),
        condition=IfCondition(sim_gazebo)
    )

    delay_joint_state_broadcaster_spawner_after_gazebo_classic_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_classic_node_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        ),
        condition=IfCondition(sim_gazebo_classic)
    )

    delay_rviz_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for controller in robot_controller_spawners:
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[controller]
                )
            )
        ]


    return LaunchDescription(
        declared_args
        + [
            _log0,
            _log1,
            _log2,
            robot_state_pub_node,
            gazebo_launch,
            gazebo_classic_launch,
            gazebo_node_spawner,
            gazebo_classic_node_spawner,
            delay_joint_state_broadcaster_spawner_after_gazebo_classic_spawner,
            delay_joint_state_broadcaster_spawner_after_gazebo_spawner,
            delay_rviz_after_joint_state_broadcaster
            
        ]
        + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner
    )