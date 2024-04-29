#!/usr/bin/env python3

"""
Queries controller_manager services until they're usable, launches the joint_state_broadcaster, then quits
Author: Luke Strohbehn
"""

import rclpy
from rclpy.lifecycle import Node, TransitionCallbackReturn
from rclpy.lifecycle.node import LifecycleState

class DelayJointStateBroadcasterNode(Node):
    def __init__(self, node_name, **kwargs) -> None:
        super().__init__(node_name=node_name, kwargs=kwargs)

        self._timer_ping_CM_srv = self.create_timer(timer_period_sec=0.1, callback=self._timer_ping_CM_srv_cb)

        return
    
    def _timer_ping_CM_srv_cv(self) -> None:
        self.get_logger().info("hello world")
        return

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring: {state}")
        return super().on_configure(state)
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating: {state}")
        return super().on_activate(state)
    
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        return super().on_deactivate(state)
    
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        return super().on_shutdown(state)
    


