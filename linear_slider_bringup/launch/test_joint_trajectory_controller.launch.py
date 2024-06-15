from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    state_goals = PathJoinSubstitution(
        [FindPackageShare("linear_slider_bringup"), "config", "test_goal_publishers_config.yaml"]
    )

    joint_trajectory_controller_test_node = Node(
        package="ros2_controllers_test_nodes",
        executable="publisher_joint_trajectory_controller",
        name="publisher_joint_trajectory_controller",
        parameters=[state_goals],
        output="both",
    )

    return LaunchDescription([joint_trajectory_controller_test_node])
