#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

class SimTimePublisher(Node):
    def __init__(self):
        super().__init__('sim_time_publisher')
        self.publisher_ = self.create_publisher(Clock, '/clock', 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        current_time = self.get_clock().now() - self.start_time
        clock_msg = Clock()
        duration = Duration(seconds=current_time.nanoseconds / 1e9)
        clock_msg.clock.sec = duration.nanoseconds // 1_000_000_000
        clock_msg.clock.nanosec = duration.nanoseconds % 1_000_000_000
        self.publisher_.publish(clock_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimTimePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()