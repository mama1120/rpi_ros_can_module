#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import can
import struct

class JointStateToPDO(Node):
    def __init__(self):
        super().__init__('joint_state_to_pdo')
        self.scale = 1_000_000  # Skalierung: µrad
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=125000)

        self.sub = self.create_subscription(JointState, '/joint_states', self.callback, 10)
        self.get_logger().info("PDO Sender aktiv.")

    def send_pdo(self, cob_id, values):
        data = b''.join([struct.pack("<i", int(v * self.scale)) for v in values]) #
        data = data.ljust(8, b'\x00')  # auf 8 Byte auffüllen
        msg = can.Message(arbitration_id=cob_id, data=data, is_extended_id=False)
        self.bus.send(msg)
        #self.get_logger().info(f"PDO {hex(cob_id)} gesendet: {[round(v, 4) for v in values]}")

    def callback(self, msg):
        if len(msg.position) < 5:
            self.get_logger().warn("Zu wenig Positionen.")
            return

        try:
            # Gelenke 1 & 2 → µC 1
            #self.send_pdo(0x201, msg.position[0:2])

            # Gelenke 3 & 4 → µC 2
            self.send_pdo(0x202, msg.position[2:4])

            # Gelenk 5 → µC 3
            #self.send_pdo(0x203, msg.position[4:5])

        except Exception as e:
            self.get_logger().error(f"Fehler beim Senden: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateToPDO()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
