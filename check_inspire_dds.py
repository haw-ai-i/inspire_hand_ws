#!/usr/bin/env python3
import argparse
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber

from inspire_sdkpy import inspire_dds


class Counter:
    def __init__(self):
        self.total = 0
        self.last_total = 0

    def cb(self, _msg):
        self.total += 1

    def per_sec(self):
        d = self.total - self.last_total
        self.last_total = self.total
        return d


def main():
    p = argparse.ArgumentParser(description="Headless Inspire DDS topic validator")
    p.add_argument("--domain", type=int, default=0)
    p.add_argument("--interface", type=str, default="", help="DDS nic, e.g. enp3s0")
    p.add_argument("--seconds", type=int, default=10)
    args = p.parse_args()

    if args.interface:
        ChannelFactoryInitialize(args.domain, args.interface)
        print(f"[init] domain={args.domain} interface={args.interface}")
    else:
        ChannelFactoryInitialize(args.domain)
        print(f"[init] domain={args.domain} interface=<default>")

    left_ctr = Counter()
    right_ctr = Counter()

    sub_l = ChannelSubscriber("rt/inspire_hand/state/l", inspire_dds.inspire_hand_state)
    sub_r = ChannelSubscriber("rt/inspire_hand/state/r", inspire_dds.inspire_hand_state)
    sub_l.Init(left_ctr.cb, 10)
    sub_r.Init(right_ctr.cb, 10)

    pub_l = ChannelPublisher("rt/inspire_hand/ctrl/l", inspire_dds.inspire_hand_ctrl)
    pub_r = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)
    pub_l.Init()
    pub_r.Init()
    print("[probe] Subscribed: rt/inspire_hand/state/l|r")
    print("[probe] Publisher ready: rt/inspire_hand/ctrl/l|r")

    t0 = time.time()
    while time.time() - t0 < args.seconds:
        time.sleep(1.0)
        print(
            f"[rate] left={left_ctr.per_sec():3d}/s total={left_ctr.total:5d} | "
            f"right={right_ctr.per_sec():3d}/s total={right_ctr.total:5d}"
        )

    ok = left_ctr.total > 0 or right_ctr.total > 0
    if ok:
        print("[result] PASS: received Inspire hand state traffic")
        return 0

    print("[result] FAIL: no Inspire hand state traffic received")
    print("         Check: hand driver running, hand IPs, DDS interface/domain.")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
