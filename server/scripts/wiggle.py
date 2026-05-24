"""Gentle wiggle test for Feetech STS3215 servos.

Reads each servo's current position, applies a small random offset (~4° peak)
at low speed, holds for a beat, then returns to the original position and
disables torque. Designed to confirm "the bus can drive motion" without
swinging anything hard.
"""
from __future__ import annotations

import random
import sys
import time

from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# STS3215 register addresses
ADDR_TORQUE_ENABLE = 0x28      # 1 byte
ADDR_GOAL_ACC = 0x29           # 1 byte (acceleration limit)
ADDR_GOAL_POSITION = 0x2A      # 2 bytes, 0-4095 = 0-360°
ADDR_GOAL_SPEED = 0x2E         # 2 bytes (steps/sec, 0 = max)
ADDR_PRESENT_POSITION = 0x38   # 2 bytes (read-only)

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem5A4B0468521"
BAUD = 1_000_000
IDS = [3, 4, 6]               # discovered by scan_bus.py
WIGGLE_TICKS = 50             # ~4° peak (4096 ticks / 360°)
WIGGLE_SPEED = 200            # very slow — steps/sec
WIGGLE_ACC = 20               # gentle acceleration
HOLD_SECONDS = 1.0


def read_position(packet, port, sid: int) -> int | None:
    val, comm, _ = packet.read2ByteTxRx(port, sid, ADDR_PRESENT_POSITION)
    return val if comm == COMM_SUCCESS else None


def set_torque(packet, port, sid: int, on: bool) -> None:
    packet.write1ByteTxRx(port, sid, ADDR_TORQUE_ENABLE, 1 if on else 0)


def write_goal(packet, port, sid: int, position: int, speed: int, acc: int) -> None:
    packet.write1ByteTxRx(port, sid, ADDR_GOAL_ACC, acc)
    packet.write2ByteTxRx(port, sid, ADDR_GOAL_SPEED, speed)
    packet.write2ByteTxRx(port, sid, ADDR_GOAL_POSITION, position)


def main() -> int:
    port = PortHandler(PORT)
    if not port.openPort() or not port.setBaudRate(BAUD):
        print(f"Failed to open {PORT} @ {BAUD}")
        return 1
    packet = PacketHandler(0)

    # 1. Read home positions
    home: dict[int, int] = {}
    for sid in IDS:
        pos = read_position(packet, port, sid)
        if pos is None:
            print(f"ID {sid}: no response, skipping")
            continue
        home[sid] = pos
        print(f"ID {sid}: home = {pos} ({pos * 360 / 4095:.1f}°)")
    if not home:
        print("No servos responding — aborting.")
        return 1

    # 2. Pick small random offset targets
    targets: dict[int, int] = {}
    for sid, pos in home.items():
        offset = random.randint(-WIGGLE_TICKS, WIGGLE_TICKS)
        targets[sid] = max(0, min(4095, pos + offset))
        print(f"ID {sid}: target = {targets[sid]} (offset {offset:+d} ticks)")

    # 3. Enable torque
    for sid in home:
        set_torque(packet, port, sid, True)

    try:
        # 4. Move to wiggle target
        print("Moving to wiggle target...")
        for sid, target in targets.items():
            write_goal(packet, port, sid, target, WIGGLE_SPEED, WIGGLE_ACC)
        time.sleep(HOLD_SECONDS)

        # 5. Move back home
        print("Moving back to home...")
        for sid, pos in home.items():
            write_goal(packet, port, sid, pos, WIGGLE_SPEED, WIGGLE_ACC)
        time.sleep(HOLD_SECONDS)
    finally:
        # 6. Disable torque so the arm goes limp again (safe state)
        for sid in home:
            set_torque(packet, port, sid, False)
        port.closePort()

    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
