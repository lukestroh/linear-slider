from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    velocity_goals = PathJoinSubstitution(
        [FindPackageShare("linear_slider_bringup"), "config", "test_velocity_goal_publishers_config.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_velocity_controller",
                name="publisher_velocity_controller",
                parameters=[velocity_goals],
                output="both",
            )
        ]
    )