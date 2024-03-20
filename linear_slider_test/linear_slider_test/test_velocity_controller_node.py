#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64


class TestVelocityControllerNode(Node):
    def __init__(self):
        super().__init__(node_name="test_velocity_controller_node")
        self.vel_cmd_pub = self.create_publisher(msg_type=Float64MultiArray, topic="/velocity_controller/commands", qos_profile=1)

        self.pub_timer = self.create_timer(timer_period_sec=0.01, callback=self.pub_timer_callback)
        return
    
    def pub_timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [-0.05]
        msg.layout.data_offset = 0
        msg.get_fields_and_field_types()
        self.vel_cmd_pub.publish(msg)
        self.get_logger().info(f"Publishing: {msg}")
        return

def main():
    rclpy.init()
    test_velocity_controller_node = TestVelocityControllerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(test_velocity_controller_node, executor=executor)
    test_velocity_controller_node.destroy_node()
    rclpy.shutdown()
    return

if __name__ == '__main__':
    main()