#!/usr/bin/env python3

"""
Queries controller_manager services until they're usable, launches the joint_state_broadcaster, then quits
Author: Luke Strohbehn
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle.node import LifecycleState
from rclpy.time import Duration

from controller_manager_msgs.srv import ListControllers


class DelayJointStateBroadcasterNode(LifecycleNode):
    def __init__(self, node_name) -> None:
        super().__init__(node_name=node_name)
        self.trigger_configure()
        self.trigger_activate()
        return
    
    def _timer_cb_service_response(self) -> None:
        """Ping the controller manager for when its services are available"""
        # if not self.client.wait_for_service(timeout_sec=0.5):
        if not self.client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info(f"Service {self.srv} not available, waiting again...")
        else:
            self.timer_get_service_response.cancel() # cancel the timer so multiple requests aren't sent
            self.get_logger().info(f"Found service {self.srv}.")
            self.trigger_deactivate()
            self.trigger_shutdown()
        return

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring...")
        self.srv = "/controller_manager/list_controllers"
        self.client = self.create_client(
            srv_name=self.srv,
            srv_type=ListControllers,
        )
        return super().on_configure(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating...")
        self.timer_get_service_response = self.create_timer(timer_period_sec=1.0, callback=self._timer_cb_service_response)
        return super().on_activate(state)
    
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up...")
        return super().on_cleanup(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating...")
        return super().on_deactivate(state)

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Shutting down...")
        self.destroy_timer(self._timer_cb_service_response)
        return super().on_shutdown(state)


def main(args=None) -> None:
    rclpy.init(args=args)
    lifecycle_node = DelayJointStateBroadcasterNode("delay_jsb_node_spawner")
    try:
        rclpy.spin(lifecycle_node)
    finally:
        lifecycle_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
