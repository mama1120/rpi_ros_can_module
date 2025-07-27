#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import canopen

class SDOJointWriter(Node):
    def __init__(self):
        super().__init__('sdo_joint_writer')
        self.bus = canopen.Network()
        self.bus.connect(bustype='socketcan', channel='can0', bitrate=500000)

        self.node_id = 0x04
        self.node = self.bus.add_node(self.node_id, '/home/yannik/CANopenDemo/demo/demoDevice.eds')

        self.sub = self.create_subscription(JointState, '/joint_states', self.callback, 10)
        self.get_logger().info("SDO Joint Writer läuft.")

    def callback(self, msg):
        scale = 1000000
        try:
            if len(msg.position) >= 5:
                for i in range(5):
                    value = int(msg.position[i] * scale)
                    self.node.sdo[0x2110][i + 1].raw = value
                self.get_logger().info(f"SDOs geschrieben: {[round(p, 3) for p in msg.position[:5]]}")
            else:
                self.get_logger().warn("Nicht genug Positionen in joint_states.")
        except Exception as e:
            self.get_logger().error(f"Fehler beim SDO-Schreiben: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SDOJointWriter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
