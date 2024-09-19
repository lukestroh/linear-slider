#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    SetLaunchConfiguration,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.conditions import UnlessCondition, IfCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
    PythonExpression
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


from linear_slider_moveit_config.launch_common import load_yaml

from moveit_configs_utils import MoveItConfigsBuilder


import rclpy.logging

import os

logger = rclpy.logging.get_logger("ur_with_linear_slider_bringup.launch")

def launch_setup(context, *args, **kwargs):
    # Launch configurations
    # System config
    system_controllers_file = LaunchConfiguration("system_controllers_file")
    system_description_file = LaunchConfiguration("system_description_file")
    system_runtime_package = LaunchConfiguration("system_runtime_package")
    system_moveit_config_package = LaunchConfiguration("system_moveit_config_package")

    # Linear slider config
    linear_slider_bringup_package = LaunchConfiguration("linear_slider_bringup_package")
    linear_slider_controllers_package = LaunchConfiguration("linear_slider_controllers_package")
    linear_slider_controllers_file = LaunchConfiguration("linear_slider_controllers_file")
    linear_slider_controller = LaunchConfiguration("linear_slider_controller")
    linear_slider_parent = LaunchConfiguration("linear_slider_parent")
    linear_slider_robot_ip = LaunchConfiguration("linear_slider_robot_ip")
    prefix = LaunchConfiguration("prefix")

    # UR robot config
    headless_mode = LaunchConfiguration("headless_mode")
    ur_type = LaunchConfiguration("ur_type")
    tf_prefix = LaunchConfiguration("tf_prefix")
    ur_parent = LaunchConfiguration("ur_parent")
    ur_robot_ip = LaunchConfiguration("ur_robot_ip")
    ur_runtime_package = LaunchConfiguration("ur_runtime_package")
    ur_controllers_file = LaunchConfiguration("ur_controllers_file")

    # General parameters
    use_sim = LaunchConfiguration("use_sim")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    use_moveit = LaunchConfiguration("use_moveit")

    # # Get URDF from xacro
    # robot_description_content = Command([
    #     PathJoinSubstitution([FindExecutable(name="xacro")]),
    #     " ",
    #     PathJoinSubstitution(
    #         [FindPackageShare(system_runtime_package), "urdf", system_description_file]
    #     ),
    #     " ",
    #     "linear_slider_parent:=",
    #     linear_slider_parent,
    #     " ",
    #     "linear_slider_robot_ip:=",
    #     linear_slider_robot_ip,
    #     " ",
    #     "prefix:=",
    #     prefix,
    #     " ",
    #     "tf_prefix:=",
    #     tf_prefix,
    #     " ",
    #     "use_mock_hardware:=",
    #     use_mock_hardware,
    #     " ",
    #     "mock_sensor_commands:=",
    #     mock_sensor_commands,
    #     " ",
    #     "ur_robot_ip:=",
    #     ur_robot_ip,
    #     " ",
    #     "ur_type:=",
    #     ur_type,
    #     " ",
    #     "ur_parent:=",
    #     ur_parent,
    #     " ",
    # ])

    # robot_description = {"robot_description": robot_description_content}

    # filepath_linear_slider_controllers = PathJoinSubstitution([
    #     FindPackageShare(linear_slider_controllers_package),
    #     "config",
    #     linear_slider_controllers_file
    # ])

    # filepath_ur_controllers = PathJoinSubstitution([
    #     FindPackageShare(ur_runtime_package),
    #     "config",
    #     ur_controllers_file
    # ])

    filepath_system_controllers = PathJoinSubstitution([
        FindPackageShare(system_runtime_package), # TODO: refactor as system_bringup_package
        "config",
        system_controllers_file
    ])

    # define update rate for the UR robot
    filepath_update_rate_config = PathJoinSubstitution(
        [
            FindPackageShare(ur_runtime_package),
            "config",
            ur_type.perform(context=context) + "_update_rate.yaml"
        ]
    )
    
    
    script_filename = PathJoinSubstitution(
        [
            FindPackageShare("ur_client_library"),
            "resources",
            "external_control.urscript",
        ]
    )
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )

    mcb = MoveItConfigsBuilder(robot_name="ur_with_linear_slider", package_name="ur_with_linear_slider_moveit_config")
    mcb.robot_description(
        file_path=os.path.join(
            FindPackageShare(system_runtime_package).perform(context=context),
            "urdf",
            system_description_file.perform(context=context)
        ),
        mappings={
            "headless_mode": headless_mode,
            "mock_sensor_commands": mock_sensor_commands,
            "linear_slider_parent": linear_slider_parent,
            "linear_slider_robot_ip": linear_slider_robot_ip,
            "prefix": prefix, # TODO: Change system launch to linear_slider_prefix
            "tf_prefix": tf_prefix, # TODO: change to ur_prefix
            "ur_parent": ur_parent,
            "ur_robot_ip": ur_robot_ip,
            "ur_type": ur_type,
            "use_mock_hardware": use_mock_hardware,

            "script_filename": script_filename,
            "input_recipe_filename": input_recipe_filename,
            "output_recipe_filename": output_recipe_filename,
        }
    )
    mcb.robot_description_semantic(
        file_path=os.path.join(
            get_package_share_directory("ur_with_linear_slider_moveit_config"),
            "srdf",
            "ur_with_linear_slider.srdf.xacro"
        ),
        mappings={
            "prefix": prefix,
            "tf_prefix": tf_prefix
        }
    )
    moveit_configs = mcb.to_moveit_configs()
    # logger.warn(f"{moveit_configs.robot_description['robot_description'].value[0].perform(context)}")

    node_mock_hardware_control = Node(
        package = "controller_manager",
        executable = "ros2_control_node",
        output = "screen",
        parameters = [
            moveit_configs.robot_description, # Says it's deprecated, but only works if this is provided!
            filepath_update_rate_config,
            # ParameterFile(filepath_ur_controllers, allow_substs=True), # substitute joint names with prefix values
            # ParameterFile(filepath_linear_slider_controllers, allow_substs=True),
            ParameterFile(filepath_system_controllers, allow_substs=True)
        ],
        condition=IfCondition(use_mock_hardware)
    )

    # parameterfile_ur_controllers = ParameterFile(filepath_ur_controllers, allow_substs=True)
    # parameterfile_ur_controllers.evaluate(context=context)

    # logger.warn(f"{parameterfile_ur_controllers.param_file}")

    node_ur_control = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            moveit_configs.robot_description,
            filepath_update_rate_config,
            # ParameterFile(filepath_ur_controllers, allow_substs=True), # TODO: try this method out. Reduces the need for a custom one.
            # ParameterFile(filepath_linear_slider_controllers, allow_substs=True)
            ParameterFile(filepath_system_controllers, allow_substs=True)
        ],
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
    )

    node_robot_state_pub = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        output = "both",
        parameters = [moveit_configs.robot_description]
    )

    filepath_rviz_config = PathJoinSubstitution([
        FindPackageShare(package=system_runtime_package),
        "rviz",
        "ur_with_linear_slider.rviz"
    ])

    node_rviz = Node(
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        output = "log",
        arguments = ["-d", filepath_rviz_config],
        condition=UnlessCondition(use_moveit)
    )

    node_joint_state_broadcaster_spawner = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = [
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ]
    )

    # Detect when controller manager has published all services by utilizing a lifecycle node.
    # When the slider needs to calibrate, the spawner times-out if the moving base is at a far distance
    # from the negative limit switch.
    lifecyclenode_delay_jsb_lifecycle = LifecycleNode(
        name="delay_jsb_node_spawner",
        namespace="",
        package="linear_slider_bringup",
        executable="delay_joint_state_broadcaster_lifecycle_node.py",
        output="both"
    )
    register_event_for_slider_on_activate = RegisterEventHandler( # TODO: change the name of this variable.
        OnStateTransition(
            target_lifecycle_node=lifecyclenode_delay_jsb_lifecycle,
            goal_state="finalized",
            entities=[
                node_joint_state_broadcaster_spawner
            ]
        )
    )

    # Delay rviz start after joint_state_broadcaster to avoid unnecessary warning output
    register_event_delay_rviz_after_JSB_spawner = RegisterEventHandler(
        event_handler = OnProcessStart(target_action=node_joint_state_broadcaster_spawner, on_start=[node_rviz])
    )

    ###############################################################################
    # Robot controller spawners
    ###############################################################################
    robot_controllers = [linear_slider_controller]
    robot_controller_spawners = []
    for controller in robot_controllers:
        robot_controller_spawners.append(
            Node(
                package = "controller_manager",
                executable = "spawner",
                arguments = [controller, "-c", "/controller_manager"]
            )
        )
    # Delay loading and activation of robot_controller after 'joint_state_broadcaster'
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for controller in robot_controller_spawners:
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner.append(
            RegisterEventHandler(
                event_handler = OnProcessExit(
                    target_action = node_joint_state_broadcaster_spawner,
                    on_exit = [controller]
                )
            )
        )
    # Spawn controllers
    def controller_spawner(name, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                name,
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "120",
            ]
              + inactive_flags,
        )
    controller_spawner_names = [
        'linear_slider_controller',
        'scaled_joint_trajectory_controller',
    ]
    controller_spawner_inactive_names = ["forward_position_controller", 'joint_trajectory_controller']
    controller_spawners = [controller_spawner(name) for name in controller_spawner_names] + [
        controller_spawner(name, active=False) for name in controller_spawner_inactive_names
    ]



    # ur_launch = IncludeLaunchDescription( TODO: CHeck to see if this can be launched by removing it from URDF?
    #     AnyLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory("ur_robot_driver"), "launch", "ur_control.launch.py"),
    #     ),
    #     launch_arguments=[
    #         ("robot_ip", ur_robot_ip),
    #         ("ur_type", ur_type),
    #         ("use_fake_hardware", use_mock_hardware),
    #         ("fake_sensor_commands", mock_sensor_commands)
    #     ],
    # )

    filepath_system_moveit_config = PathJoinSubstitution([
        FindPackageShare(system_moveit_config_package), "launch", "system_moveit_config.launch.py"
    ]) # TODO: For some reason this wasn't working with FindPackageShare. Why ?


    launch_system_moveit = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(filepath_system_moveit_config),
        launch_arguments=[
            ('use_mock_hardware', use_mock_hardware),
            ('ur_parent', ur_parent),
            ('ur_robot_ip', ur_robot_ip),
            ('ur_type', ur_type),
            ("mock_sensor_commands", mock_sensor_commands),
            # ("use_sim_time", use_sim_time),
        ],
        condition=IfCondition(use_moveit)
    )

    register_event_delay_moveit_after_JSB_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(target_action=node_joint_state_broadcaster_spawner, on_start=launch_system_moveit)
    )


    nodes_to_start = [
            node_mock_hardware_control,
            node_ur_control,
            node_robot_state_pub,
            lifecyclenode_delay_jsb_lifecycle,
            register_event_for_slider_on_activate,
            register_event_delay_rviz_after_JSB_spawner,
            # launch_system_moveit,
            register_event_delay_moveit_after_JSB_spawner,
            # linear_slider_launch,
            # ur_launch,
        ] + controller_spawners
    return nodes_to_start


def generate_launch_description():
    # Declare arguments
    declared_args = []
    declared_args.append(
        DeclareLaunchArgument(
            "headless_mode"
            default_value="true",
            description="Run the UR robot in headless mode. Default true, as we typically want to run algorithms from host computer rather than UR pendant."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "linear_slider_controller",
            default_value = "linear_slider_controller",
            choices = ["linear_slider_controller", "scaled_joint_trajectory_controller"], # add another here if we want to switch between different controllers
            description = "Linear slider controller"
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "linear_slider_controllers_file",
            default_value = "linear_slider_controllers.yaml",
            description = "YAML file with the linear slider controllers description."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "linear_slider_controllers_package",
            default_value="linear_slider_controllers",
            description="Package for the controllers for the UR/linear slider combination."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "linear_slider_parent",
            default_value="world",
            description="Parent link for the linear_slider base. Default is 'world'."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "linear_slider_robot_ip",
            default_value="169.254.57.177",
            description="IP Address for the linear slider."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "linear_slider_bringup_package",
            default_value = "linear_slider_bringup",
            description = 'Package with the controller\'s configuration in the "config" folder. Usually, the argument is not set; it enables the use of a custom setup.'
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
            default_value = "linear_slider__",
            description = "Prefix of the linear slider joint names, useful for multi-robot setup. If changed, then you need to update the joint names in the controllers' description."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "system_description_file",
            default_value="ur_with_linear_slider.urdf.xacro",
            description="URDF/xacro description file of the entire system."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "system_controllers_file",
            default_value="system_controllers.yaml",
            description="YAML file with the system controllers description."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "system_moveit_config_package",
            default_value="ur_with_linear_slider_moveit_config",
            description="MoveIt2 configuration package for the ur arm with the linear slider."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "system_runtime_package",
            default_value="ur_with_linear_slider_bringup",
            description="Package with the complete system (linear slider + UR robot)."
        )
    )
    declared_args.append( # TODO: change variable name to 'ur_tf_prefix' in higher level launch files. Pass to 'tf_prefix'
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='ur_robot__',
            description="Prefix of the UR robot joint names. Useful for multi-robot setup. If changed, joint names in the controllers' configuration need to be updated."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "ur_controllers_file",
            default_value="ur_controllers.yaml",
            description="YAML file with the UR robot controllers description"
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "ur_parent",
            default_value="linear_slider__tool0",
            description="Parent link of the linear slider for joint attachment. Requires changing the URDF of the UR5 to accept a parent command. Otherwise, the UR5 gets fixed to the world by default."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "ur_robot_ip",
            default_value="169.254.177.232", # Cindy's UR5e
            description="IP Address for the UR robot."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "ur_runtime_package",
            default_value="ur_robot_driver",
            description="UR runtime package."
        )
    )
    declared_args.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="UR robot type.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
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
            "use_moveit", default_value="true", description="Use MoveIt2. Determines which RViz instance to run."
        )
    )


    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])


if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
