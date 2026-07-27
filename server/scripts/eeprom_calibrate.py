"""Firmware-aware limit calibration.

The servos' EEPROM Min/Max Angle Limit registers were clamping most joints to
tiny windows (shoulders to ~5-11 deg, gripper locked). That firmware cap sits
*inside* the physical range, so a current-based sweep can never reach the real
mechanical stops. This tool, per joint:

  1. reads + records the original angle limits (restore point)
  2. unlocks EEPROM, widens limits to full (0-4095), verifies the widen took
  3. current-based sweep -> finds the TRUE mechanical stops (high load now)
  4. writes the measured range (+ safety margin) back into EEPROM
  5. re-locks EEPROM
  6. records everything to calibration.json

One joint at a time: a joint is never left with widened limits — it's measured
and re-clamped before we move on. Abort-on-verify-mismatch throughout.

Usage:
  python -m server.scripts.eeprom_calibrate --ids 4            # canary
  python -m server.scripts.eeprom_calibrate --ids 1,2,3,4,5,6  # full arm
"""
from __future__ import annotations

import argparse
import json
import time

from scservo_sdk import PortHandler, PacketHandler

from .calibrate_limits import (
    Bus, sweep_direction, BACKOFF_TICKS, PORT, BAUD, ARM_JOINT_IDS,
)

EEPROM_SETTLE_S = 0.08  # let each EEPROM write cycle commit before reading back

# EEPROM registers
ADDR_LOCK = 0x37        # 1 = locked (writes ignored), 0 = unlocked
ADDR_MIN_ANGLE = 0x09   # 2 bytes
ADDR_MAX_ANGLE = 0x0B   # 2 bytes

FULL_MIN, FULL_MAX = 0, 4095


def w1(bus: Bus, sid: int, addr: int, val: int) -> None:
    bus.pk.write1ByteTxRx(bus.port, sid, addr, val)
    time.sleep(EEPROM_SETTLE_S)

def w2(bus: Bus, sid: int, addr: int, val: int) -> None:
    bus.pk.write2ByteTxRx(bus.port, sid, addr, val)
    time.sleep(EEPROM_SETTLE_S)

def read_limits(bus: Bus, sid: int) -> tuple[int | None, int | None]:
    return bus.read2(sid, ADDR_MIN_ANGLE), bus.read2(sid, ADDR_MAX_ANGLE)

def set_limits(bus: Bus, sid: int, mn: int, mx: int) -> bool:
    """Unlock, write both angle limits, verify readback. Leaves EEPROM UNLOCKED."""
    w1(bus, sid, ADDR_LOCK, 0)
    w2(bus, sid, ADDR_MIN_ANGLE, mn)
    w2(bus, sid, ADDR_MAX_ANGLE, mx)
    got = read_limits(bus, sid)
    return got == (mn, mx)


def calibrate_joint(bus: Bus, sid: int) -> dict | None:
    orig_min, orig_max = read_limits(bus, sid)
    home = bus.pos(sid)
    print(f"\n=== ID {sid}: home={home}  orig limits=[{orig_min},{orig_max}] ===")
    if home is None:
        print("  no position read — skipping"); return None

    # 1) widen to full range so the servo can reach physical stops
    if not set_limits(bus, sid, FULL_MIN, FULL_MAX):
        w1(bus, sid, ADDR_LOCK, 1)
        print("  !! widen verify FAILED — EEPROM not writable? aborting this joint")
        return None
    print(f"  widened to [{FULL_MIN},{FULL_MAX}] (verified)")

    # 2) sweep to the true mechanical stops
    bus.configure_speed(sid)
    bus.torque(sid, True)
    try:
        raw_max, r_up = sweep_direction(bus, sid, home, +1)
        bus.set_goal(sid, home); import time; time.sleep(0.6)
        raw_min, r_dn = sweep_direction(bus, sid, home, -1)
        bus.set_goal(sid, home); time.sleep(0.6)
    finally:
        bus.torque(sid, False)
    lo, hi = min(raw_min, raw_max), max(raw_min, raw_max)
    print(f"  swept: +stop {raw_max} ({r_up})   -stop {raw_min} ({r_dn})")

    # 3) re-clamp EEPROM to the measured range (+ margin), then lock
    safe_min, safe_max = lo + BACKOFF_TICKS, hi - BACKOFF_TICKS
    clamped_ok = set_limits(bus, sid, safe_min, safe_max)
    w1(bus, sid, ADDR_LOCK, 1)
    span = (safe_max - safe_min) / 4096 * 360
    print(f"  re-clamped EEPROM to [{safe_min},{safe_max}] (~{span:.0f} deg) "
          f"{'verified' if clamped_ok else '!! VERIFY FAILED'}")

    return {
        "home": home,
        "orig_min": orig_min, "orig_max": orig_max,
        "raw_min": lo, "raw_max": hi,
        "min_tick": safe_min, "max_tick": safe_max,
        "up_reason": r_up, "down_reason": r_dn,
        "eeprom_write_ok": clamped_ok,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", default=",".join(map(str, ARM_JOINT_IDS)))
    ap.add_argument("--out", default="server/calibration.json")
    args = ap.parse_args()
    ids = [int(x) for x in args.ids.split(",") if x.strip()]

    port = PortHandler(PORT)
    if not port.openPort() or not port.setBaudRate(BAUD):
        raise SystemExit(f"failed to open {PORT}")
    bus = Bus(port, PacketHandler(0))

    v = bus.volt(ids[0])
    if v is not None and (v < 60 or v > 79):
        port.closePort()
        raise SystemExit(f"voltage {v/10}V outside safe 6.0-7.9V window — check supply")

    results = {}
    try:
        for sid in ids:
            r = calibrate_joint(bus, sid)
            if r:
                results[str(sid)] = r
    finally:
        port.closePort()

    if results:
        with open(args.out, "w") as f:
            json.dump(results, f, indent=2)
        print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
