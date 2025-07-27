#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import can
import time
import struct

class JointStateToPDO(Node):
    def __init__(self):
        super().__init__('joint_state_to_pdo')
        self.scale = 1_000_000  # Skalierung: µrad
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=125000)
        self.bus.set_filters([{"can_id": 0x702, "can_mask": 0x7FF}])

        self.sub = self.create_subscription(JointState, '/joint_states', self.callback, 10)
        self.get_logger().info("PDO Sender aktiv.")
        

    def send_pdo(self, cob_id, value):
        #self.get_logger().info(f"PDO {hex(cob_id)} gesendet: {int(value * self.scale)}")
        #data = struct.pack(int(value * self.scale)) #struct.pack("<i", int(value * self.scale))
        int_value = int(value * self.scale)
        data = int_value.to_bytes(4, byteorder="little", signed=True)
        msg = can.Message(arbitration_id=cob_id, data=data, is_extended_id=False)
        self.bus.send(msg)
        #self.get_logger().info(f"PDO {hex(cob_id)} gesendet: {round(value, 4)}")

    def callback(self, msg):
        if len(msg.position) < 5:
            self.get_logger().warn("Zu wenig Positionen.")
            return

        try:
            # Gelenke 1 & 2 → µC 1
            self.send_pdo(0x202, msg.position[2])
            self.send_pdo(0x203, msg.position[0])

            # Gelenke 3 & 4 → µC 2
            self.send_pdo(0x204, msg.position[1])
            self.send_pdo(0x205, msg.position[3])

            # Gelenk 5 → µC 3
            self.send_pdo(0x206, msg.position[4])

        except Exception as e:
            self.get_logger().error(f"Fehler beim Senden: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateToPDO()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
