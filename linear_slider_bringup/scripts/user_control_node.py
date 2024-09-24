#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState



from functools import partial
import os
import yaml


class UserControlNode(Node):
    def __init__(self):
        super().__init__(node_name="user_control_node")
        # Loggers
        self.info = lambda x: self.get_logger().info(f"{x}")
        self.warn = lambda x: self.get_logger().warn(f"{x}")
        self.err = lambda x: self.get_logger().error(f"{x}")

        

        # ROS Parameters
        self.linear_slider_prefix = self.declare_parameter("prefix", value=Parameter.Type.STRING).get_parameter_value().string_value

        # Callback groups
        self.high_priority_cb_group = ReentrantCallbackGroup()

        # Subscriptions
        self._sub_joy_state = self.create_subscription(
            msg_type=Joy,
            topic="/joy",
            callback=self._sub_cb_joy_state,
            qos_profile=1,
            callback_group=self.high_priority_cb_group
        )

        # Publishers
        self._pub_slider_servo = self.create_publisher(
            msg_type=TwistStamped,
            topic="/servo_node/delta_twist_cmds",
            qos_profile=10,
        )

        #Timers
        self._filtered_timer = self.create_timer(
            timer_period_sec=0.005,
            callback=self._timer_cb
        )

        self.axes = {
            "LT": 2,
            "RT": 5
        }
        self.remapped_vals = [0, 0]

        self.data: TwistStamped = None
        return
    
    def _sub_cb_joy_state(self, msg: Joy):

        axes_states = msg.axes

        # Remap the trigger values to positive and negative velocities. Unpressed triggers should have a velocity of 0. 
        for i, (key, value) in enumerate(self.axes.items()):
            _scaled = self._scale(axes_states[value], 1, -1, 0, 0.08333333333333334) # joy input range [1,-1] TODO: get max vel from param file
            if key == "LT":
                self.remapped_vals[i] = -_scaled
            elif key == "RT":
                self.remapped_vals[i] = _scaled
        
        # sum both trigger values
        vel_cmd = sum(self.remapped_vals)

        # publish twist message
        twist = TwistStamped()
        twist.header.frame_id = f'{self.linear_slider_prefix}base_link'
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = vel_cmd
        self.data = twist
        # self._pub_slider_servo.publish(twist)

        return 
    
    def _timer_cb(self):
        if self.data is not None:
            self._pub_slider_servo.publish(self.data)
            pass
        return

    # def process_axis(self, axes_states):
    #     """TODO: replace the for loop above (similar to alex's?)"""
        
    #     return
    
    def _scale(self, val, amin, amax, bmin, bmax):
        """Scale a value from one range to a another"""
        if amin < amax:
            if val > amax or val < amin:
                raise ValueError(f"Input value {val} is out of bounds of input range {amin} to {amax}")
        else:
            if val < amax or val > amin:
                raise ValueError(f"Input value {val} is out of bounds of input range {amin} to {amax}")
            
        arange = amax - amin
        brange = bmax - bmin
        new_val = (((val - amin) * brange) / arange) + bmin
        return new_val
    

def main(args=None):
    rclpy.init(args=args)
    node = UserControlNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    return


if __name__ == "__main__":
    main()