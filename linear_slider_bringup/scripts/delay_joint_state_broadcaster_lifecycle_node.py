#!/usr/bin/env python3

"""
Queries controller_manager services until they're usable, launches the joint_state_broadcaster, then quits
Author: Luke Strohbehn
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle.node import LifecycleState
from controller_manager_msgs.srv import ListControllers


class DelayJointStateBroadcasterNode(LifecycleNode):
    def __init__(self, node_name) -> None:
        super().__init__(node_name=node_name)

        # self._timer_ping_CM_srv = self.create_timer(timer_period_sec=0.1, callback=self._timer_ping_CM_srv_cb)
        srv = "/controller_manager/list_controllers"
        self.client = self.create_client(
            srv_name=srv,
            srv_type=ListControllers,
        )
        self.trigger_configure()
        while not self.client.wait_for_service(timeout_sec=5):
            self.get_logger().info(f"Service {srv} not available, waiting again...")
        self.trigger_shutdown()
        return

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        return super().on_configure(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        return super().on_deactivate(state)

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node shutdown")
        return super().on_shutdown(state)


def main(args=None) -> None:
    rclpy.init(args=args)
    lifecycle_node = DelayJointStateBroadcasterNode("delay_jsb_node_spawner")
    try:
        rclpy.spin(lifecycle_node)
    finally:
        lifecycle_node.destroy_node()


if __name__ == "__main__":
    main()
