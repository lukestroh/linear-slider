#!/usr/bin/env python3

"""
This script subscribes to the /joint_states topic and publishes 


TODO:
0. If mock_hardware, disable original publisher?
1. Set min/max parameters from file
2. Read in /joint_states values
3. Run timer checks?
4. Publish limit switch state.

"""

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle.node import LifecycleState


class MockHardwareLimitSwitchStatePublisher(LifecycleNode):
    def __init__(self, node_name) -> None:
        super().__init__(node_name=node_name)
        self.trigger_configure()

        return
    
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring...")
        return super().on_configure(state)