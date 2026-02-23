#!/usr/bin/env python3
import os
import time

import rclpy
from rclpy.node import Node

from quest2ros2_msg.msg import OVR2ROSInputs
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher

from inspire_sdkpy import inspire_dds, inspire_hand_defaut


class SimpleQuestHandBridge(Node):
    def __init__(self):
        super().__init__("simple_quest_hand_bridge")

        dds_domain = int(os.getenv("HANDS_DDS_DOMAIN", "0"))
        dds_iface = os.getenv("HANDS_DDS_INTERFACE", "enp39s0").strip()
        threshold = float(os.getenv("HANDS_TRIGGER_THRESHOLD", "0.2"))

        self._close_angles = [900, 900, 900, 900, 900, 900]
        self._open_angles = [0, 0, 0, 0, 0, 0]
        self._threshold = threshold

        if dds_iface:
            ChannelFactoryInitialize(dds_domain, dds_iface)
            self.get_logger().info(
                f"DDS init domain={dds_domain} iface={dds_iface} (simple hand bridge)"
            )
        else:
            ChannelFactoryInitialize(dds_domain)
            self.get_logger().info(f"DDS init domain={dds_domain} iface=<auto>")

        self._pub_l = ChannelPublisher("rt/inspire_hand/ctrl/l", inspire_dds.inspire_hand_ctrl)
        self._pub_r = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)
        self._pub_l.Init()
        self._pub_r.Init()

        self._left_close = False
        self._right_close = False
        self._last_left = None
        self._last_right = None

        self.create_subscription(
            OVR2ROSInputs, "/q2r_left_hand_inputs", self._on_left_inputs, 10
        )
        self.create_subscription(
            OVR2ROSInputs, "/q2r_right_hand_inputs", self._on_right_inputs, 10
        )
        self.create_timer(0.04, self._publish_loop)  # 25 Hz

    def _should_close(self, msg: OVR2ROSInputs) -> bool:
        return (
            msg.press_middle > self._threshold
            or msg.press_index > self._threshold
            or msg.button_upper
            or msg.button_lower
        )

    def _on_left_inputs(self, msg: OVR2ROSInputs):
        self._left_close = self._should_close(msg)

    def _on_right_inputs(self, msg: OVR2ROSInputs):
        self._right_close = self._should_close(msg)

    def _make_cmd(self, close: bool):
        cmd = inspire_hand_defaut.get_inspire_hand_ctrl()
        cmd.mode = 0b0001  # angle mode
        cmd.angle_set = self._close_angles if close else self._open_angles
        return cmd

    def _publish_loop(self):
        # Keep publishing at a steady rate so drivers always have a fresh command.
        cmd_l = self._make_cmd(self._left_close)
        cmd_r = self._make_cmd(self._right_close)
        self._pub_l.Write(cmd_l)
        self._pub_r.Write(cmd_r)

        now_left = self._left_close
        now_right = self._right_close
        if self._last_left != now_left or self._last_right != now_right:
            self.get_logger().info(
                f"hand state changed: left={'close' if now_left else 'open'}, "
                f"right={'close' if now_right else 'open'}"
            )
            self._last_left = now_left
            self._last_right = now_right


def main():
    rclpy.init()
    node = SimpleQuestHandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
