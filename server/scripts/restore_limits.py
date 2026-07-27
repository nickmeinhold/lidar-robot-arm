"""Restore STS3215 EEPROM angle limits from a backup file.

Safety net for calibration mishaps: puts a servo's Min/Max Angle Limit back to
a known-good snapshot. Read-verify each write, re-lock EEPROM when done.

Usage:
  python -m server.scripts.restore_limits --ids 2
  python -m server.scripts.restore_limits --ids 2,5 --backup server/servo_limits_backup_2026-07-27.json
"""
from __future__ import annotations

import argparse
import json
import time

from scservo_sdk import PortHandler, PacketHandler

from .calibrate_limits import Bus, PORT, BAUD
from .eeprom_calibrate import ADDR_LOCK, ADDR_MIN_ANGLE, ADDR_MAX_ANGLE, w1, w2


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", required=True)
    ap.add_argument("--backup", default="server/servo_limits_backup_2026-07-27.json")
    args = ap.parse_args()
    ids = [int(x) for x in args.ids.split(",") if x.strip()]

    with open(args.backup) as f:
        backup = json.load(f)["servos"]

    port = PortHandler(PORT)
    if not port.openPort() or not port.setBaudRate(BAUD):
        raise SystemExit(f"failed to open {PORT}")
    bus = Bus(port, PacketHandler(0))

    try:
        for sid in ids:
            key = str(sid)
            if key not in backup:
                print(f"ID {sid}: not in backup — skipping"); continue
            mn, mx = backup[key]["min_angle"], backup[key]["max_angle"]
            name = backup[key].get("name", "")
            cur = (bus.read2(sid, ADDR_MIN_ANGLE), bus.read2(sid, ADDR_MAX_ANGLE))
            w1(bus, sid, ADDR_LOCK, 0)
            w2(bus, sid, ADDR_MIN_ANGLE, mn)
            w2(bus, sid, ADDR_MAX_ANGLE, mx)
            got = (bus.read2(sid, ADDR_MIN_ANGLE), bus.read2(sid, ADDR_MAX_ANGLE))
            w1(bus, sid, ADDR_LOCK, 1)
            ok = got == (mn, mx)
            print(f"ID {sid} ({name}): {cur} -> [{mn},{mx}]  "
                  f"{'verified' if ok else '!! VERIFY FAILED'}  (lock re-engaged)")
    finally:
        port.closePort()


if __name__ == "__main__":
    main()
