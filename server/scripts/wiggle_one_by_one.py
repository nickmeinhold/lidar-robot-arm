"""Wiggle each responding servo individually so the user can watch which
physical joint moves. For each servo: print the ID, move +50 ticks (~4°),
hold, move -50 ticks, hold, return to home, then pause before the next one.
"""
from __future__ import annotations

import sys
import time

from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

ADDR_TORQUE_ENABLE = 0x28
ADDR_GOAL_ACC = 0x29
ADDR_GOAL_POSITION = 0x2A
ADDR_GOAL_SPEED = 0x2E
ADDR_PRESENT_POSITION = 0x38

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem5A4B0468521"
BAUD = 1_000_000
WIGGLE_TICKS = 80     # ~7° peak — enough to see clearly
WIGGLE_SPEED = 300    # gentle
WIGGLE_ACC = 30
HOLD_S = 0.7
GAP_S = 1.2


def main() -> int:
    port = PortHandler(PORT)
    if not port.openPort() or not port.setBaudRate(BAUD):
        print(f"Failed to open {PORT}")
        return 1
    packet = PacketHandler(0)

    # Discover responders
    ids = []
    for sid in range(1, 254):
        _, comm, _ = packet.ping(port, sid)
        if comm == COMM_SUCCESS:
            ids.append(sid)
    print(f"Responding servos: {ids}\n")
    if not ids:
        return 1

    for sid in ids:
        pos, comm, _ = packet.read2ByteTxRx(port, sid, ADDR_PRESENT_POSITION)
        if comm != COMM_SUCCESS:
            print(f"ID {sid}: failed to read position, skipping")
            continue
        home = pos
        print(f"--- ID {sid} ---  home={home}")

        packet.write1ByteTxRx(port, sid, ADDR_GOAL_ACC, WIGGLE_ACC)
        packet.write2ByteTxRx(port, sid, ADDR_GOAL_SPEED, WIGGLE_SPEED)
        packet.write1ByteTxRx(port, sid, ADDR_TORQUE_ENABLE, 1)

        try:
            for offset in (+WIGGLE_TICKS, -WIGGLE_TICKS, 0):
                target = max(0, min(4095, home + offset))
                print(f"  → {target} (offset {offset:+d})")
                packet.write2ByteTxRx(port, sid, ADDR_GOAL_POSITION, target)
                time.sleep(HOLD_S)
        finally:
            packet.write1ByteTxRx(port, sid, ADDR_TORQUE_ENABLE, 0)

        time.sleep(GAP_S)

    port.closePort()
    print("\nDone.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
