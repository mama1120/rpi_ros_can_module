#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class DummyJointStatePublisher(Node):
    def __init__(self):
        super().__init__('dummy_joint_state_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.start_time = time.time()

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        t = time.time() - self.start_time
        js = JointState()
        js.header.stamp = now
        js.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        js.position = [
            math.sin(t),
            math.sin(t + 1),
            math.sin(t + 2),
            math.sin(t + 3),
            math.sin(t + 4)
        ]
        self.publisher.publish(js)
        self.get_logger().info(f'Publishing: {js.position}')

def main(args=None):
    rclpy.init(args=args)
    node = DummyJointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()