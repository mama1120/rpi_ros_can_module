#!/usr/bin/env python3
#
# ROS 2‑Node, der JointStates in CAN‑PDOs übersetzt,
# aber nur, wenn die zugehörigen Nodes im Operational‑Modus sind.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import can
import struct
import time
from threading import Lock

# ─────────────────────────────────────────────
NODE_IDS      = [2, 3, 4]        # CANopen‑Node‑IDs deiner 3 Motion‑Controller
HEART_TIMEOUT = 10.0              # s – länger als 2×Heart‑Beat‑Periode
OPERATIONAL   = 0x05             # NMT‑Statusbyte für "Operational"
SCALE         = 1_000_000        # rad  →  µrad (int32)

# Mapping Gelenkindex ➜ (PDO‑COB‑ID, verantwortliche Node‑ID)
JOINT_MAP = {
    0: (0x203, 1),   # Gelenk 1   → Node 1, PDO 0x203
    1: (0x204, 2),   # Gelenk 2   → Node 2, PDO 0x204
    2: (0x202, 1),   # Gelenk 3   → Node 1, PDO 0x202
    3: (0x205, 2),   # Gelenk 4   → Node 2, PDO 0x205
    4: (0x206, 3),   # Gelenk 5   → Node 3, PDO 0x206
}
# ─────────────────────────────────────────────


class HeartbeatMonitor(can.Listener):
    """Empfängt Heartbeats und hält den letzten Status/TimeStamp je Node."""
    def __init__(self):
        super().__init__()
        self.last_state = {nid: None for nid in NODE_IDS}
        self.last_seen  = {nid: 0.0  for nid in NODE_IDS}
        self._lock = Lock()

    def on_message_received(self, msg):
        if 0x700 <= msg.arbitration_id <= 0x77F and len(msg.data):
            nid = msg.arbitration_id - 0x700
            if nid in NODE_IDS:
                with self._lock:
                    self.last_state[nid] = msg.data[0]
                    self.last_seen[nid]  = time.time()

    # ---------- Abfragen -----------------------------------------
    def node_operational(self, nid):
        with self._lock:
            return (self.last_state[nid] == OPERATIONAL and
                    (time.time() - self.last_seen[nid]) < HEART_TIMEOUT)

    def all_operational(self):
        return all(self.node_operational(nid) for nid in NODE_IDS)


class JointStateToPDO(Node):

    def __init__(self):
        super().__init__('joint_state_to_pdo')
        self.get_logger().info("Starte PDO‑Sender mit Heartbeat‑Überwachung …")

        # CAN‑Bus + Heartbeat‑Filter
        self.bus = can.ThreadSafeBus(channel='can0',
                                     bustype='socketcan',
                                     bitrate=125000)
        hb_filters = [{"can_id": 0x700 + nid, "can_mask": 0x7FF}
                      for nid in NODE_IDS]
        self.bus.set_filters(hb_filters)

        # Heartbeat‑Listener asynchron starten
        self.hb_monitor = HeartbeatMonitor()
        self.notifier   = can.Notifier(self.bus, [self.hb_monitor])

        # ROS‑Subscriber
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_cb,
            10)

    # ---------- CAN‑Utility ---------------------------------------
    def send_pdo(self, cob_id: int, value: float):
        int_val = int(value * SCALE)
        data = int_val.to_bytes(4, byteorder='little', signed=True)
        msg = can.Message(arbitration_id=cob_id,
                          data=data,
                          is_extended_id=False)
        self.bus.send(msg, timeout=0.01)

    # ---------- ROS‑Callback --------------------------------------
    def joint_cb(self, msg: JointState):
        if len(msg.position) < 5:
            self.get_logger().warn("Zu wenig Positionen im JointState.")
            return

        # Erst senden, wenn ALLE Controller ›Operational‹ sind
        if not self.hb_monitor.all_operational():
            self.get_logger().debug("Noch nicht alle Controller im Operational‑Modus.")
            return

        try:
            if self.hb_monitor.all_operational():
                # Gelenke 1 & 2 → µC 1
                self.send_pdo(0x202, msg.position[2])
                self.send_pdo(0x203, msg.position[0])

                # Gelenke 3 & 4 → µC 2
                self.send_pdo(0x204, msg.position[1])
                self.send_pdo(0x205, msg.position[3])

                # Gelenk 5 → µC 3
                self.send_pdo(0x206, msg.position[4])
            else:
                self.get_logger().warn_once("Nicht alle Nodes sind im Operational‑Modus.")
        
        except Exception as e:
            self.get_logger().error(f"Fehler beim PDO‑Senden: {e}")

    # ---------- Aufräumen -----------------------------------------
    def destroy_node(self):
        self.notifier.stop()
        self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JointStateToPDO()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
