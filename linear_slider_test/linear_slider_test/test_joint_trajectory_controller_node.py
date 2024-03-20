#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TestJoinTrajectoryControllerNode(Node):
    def __init__(self):
        super().__init__(node_name="test_joint_trajectory_controller_node")
        self.jt_client = ActionClient(
            node=self,
            action_type=FollowJointTrajectory,
            action_name='/joint_trajectory_controller/follow_joint_trajectory',
        )
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = ['joint1']

        point = JointTrajectoryPoint()
        point.positions

        return
    
    def send_goal(self, goal = 0.3):
        self.info("Waiting for server...")
        self.jt_client.wait_for_server()

        goal_msg = self.create_msg(goal=goal)

        self.info(f"Sending trajectory goal request.")
        self._send_goal_future = self.jt_client.send_goal_async(
            goal=goal_msg,
            feedback_callback=self._jt_feedback_cb
        )

        self._send_goal_future.add_done_callback(self._goal_response_cb)
        return
    
    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.info("Goal request was rejected by the server.")
            return

        self._goal_handle = goal_handle

        self.info("Goal request was accepted by the server")

        # Start a timer to cancel the goal
        self._timer = self.create_timer(timer_period_sec=10.0, callback=self.timer_callback)
        return

    def timer_callback(self) -> None:
        """A timer callback for """
        self.info("Canceling goal...")
        # Cancel the goal
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_request_done)

        # Cancel the timer
        self._timer.cancel()
        return
    
    def cancel_request_done(self, future) -> None:
        """Handle the result from the cancel request to the server"""
        cancel_response = future.result()

        if len(cancel_response.goals_canceling) > 0:
            self.info("Goal successfully canceled.")
        else:
            self.info("Goal failed to cancel.")

        rclpy.shutdown()
        return
    

    def _jt_feedback_cb(self, feedback):
        self.info(f"Launch server feedback: {feedback}")
        return


    def create_msg(self, goal):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = ['joint1']

        point = JointTrajectoryPoint()
        point.positions = [0.4] # This is the position that we're sending the linear slider to.
        point.time_from_start = Duration(seconds=2).to_msg()

        goal_msg.trajectory.points.append(point)
        return goal_msg

    def info(self, msg):
        return self.get_logger().info(message=msg)


def main():
    rclpy.init()
    test_joint_trajectory_controller_node = TestJoinTrajectoryControllerNode()
    test_joint_trajectory_controller_node.send_goal()
    executor = MultiThreadedExecutor()
    rclpy.spin(test_joint_trajectory_controller_node, executor=executor)
    test_joint_trajectory_controller_node.destroy_node()
    rclpy.shutdown()
    return

if __name__ == '__main__':
    main()