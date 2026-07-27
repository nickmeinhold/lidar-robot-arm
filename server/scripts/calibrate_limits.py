"""Current-based joint-limit calibration for the SO-100 / STS3215 arm.

The arm feels for its own mechanical stops. For each joint we drive slowly
outward in small steps while watching two signals:

  1. Position-tracking error — goal keeps advancing but Present_Position
     stops following (the joint physically can't move further). This is the
     *gentle* early detector: it fires before the servo strains.
  2. Present_Load (reg 0x3C) — magnitude in the low 10 bits (0-1000 = 0-100%
     of rated torque), bit 10 is a direction sign. When it crosses a
     threshold the joint is pushing against something. This is the backstop.

When either fires, we've found a stop. We record that tick, back off by a
safety margin, and move on. The result is a per-joint, asymmetric, *measured*
envelope that replaces the hardcoded SOFT_LIMIT_TICKS guesses.

SAFETY:
  - One joint moves at a time.
  - Low speed + low acceleration + small steps → the arm creeps, never slams.
  - Conservative load threshold stops well before rated torque.
  - Hard excursion cap so a mis-read can't drive a joint into a wrap-around.
  - --dry-run reads only, enables no torque, moves nothing.

Usage:
  python -m server.scripts.calibrate_limits --dry-run      # read-only sanity
  python -m server.scripts.calibrate_limits --ids 1,2,3,4,5
  python -m server.scripts.calibrate_limits                # all arm joints
"""
from __future__ import annotations

import argparse
import json
import os
import time

from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# --- register map (confirmed live 2026-07-25) ---------------------------
ADDR_TORQUE_ENABLE = 0x28
ADDR_GOAL_ACC = 0x29
ADDR_GOAL_POSITION = 0x2A
ADDR_GOAL_SPEED = 0x2E
ADDR_PRESENT_POSITION = 0x38
ADDR_PRESENT_LOAD = 0x3C
ADDR_PRESENT_VOLTAGE = 0x3E

PORT = os.environ.get("ARM_PORT", "/dev/cu.usbmodem5B3D0449951")
BAUD = 1_000_000

# --- sweep tuning (conservative on purpose) -----------------------------
STEP_TICKS = 15          # ~1.3 deg per step — creep, don't lunge
SETTLE_S = 0.15          # let the joint reach the step before re-reading
SWEEP_SPEED = 300        # ticks/s — slow
SWEEP_ACC = 20           # gentle ramp
LOAD_STOP = 250          # magnitude (of 1000) — ~25% torque; tolerates the
                         # elevated baseline of a joint lifting against gravity,
                         # still far below a hard-wall spike (500+)
STALL_EPS = 5            # ticks: a step moving < this counts as "didn't move"
STALL_CONFIRM = 3        # need this many CONSECUTIVE non-moves = a real wall
                         # (one gravity-lag step alone must not end the sweep)
MAX_EXCURSION = 2048     # ticks (~180 deg) per direction — generous so an
                         # off-centre home can still reach both real walls;
                         # load-stop + TICK_FLOOR/CEIL are the true backstops
BACKOFF_TICKS = 40       # margin: record the limit this far inside the stop
TICK_FLOOR, TICK_CEIL = 40, 4055   # never command outside this (wrap guard)

ARM_JOINT_IDS = [1, 2, 3, 4, 5]    # gripper (6) has different mechanics


def load_magnitude(raw: int) -> int:
    """Low 10 bits = magnitude; bit 10 is a direction sign."""
    return raw & 0x03FF


class Bus:
    def __init__(self, port: PortHandler, packet: PacketHandler):
        self.port, self.pk = port, packet

    # The Feetech SDK throws IndexError (not a clean timeout) when a servo
    # returns an empty packet — which happens during EEPROM write cycles and
    # transient bus hiccups under load. Retry with a short delay.
    def read2(self, sid: int, addr: int, tries: int = 5) -> int | None:
        for _ in range(tries):
            try:
                v, comm, _ = self.pk.read2ByteTxRx(self.port, sid, addr)
                if comm == COMM_SUCCESS:
                    return v
            except IndexError:
                pass
            time.sleep(0.03)
        return None

    def read1(self, sid: int, addr: int, tries: int = 5) -> int | None:
        for _ in range(tries):
            try:
                v, comm, _ = self.pk.read1ByteTxRx(self.port, sid, addr)
                if comm == COMM_SUCCESS:
                    return v
            except IndexError:
                pass
            time.sleep(0.03)
        return None

    def pos(self, sid: int) -> int | None:
        return self.read2(sid, ADDR_PRESENT_POSITION)

    def load(self, sid: int) -> int | None:
        v = self.read2(sid, ADDR_PRESENT_LOAD)
        return load_magnitude(v) if v is not None else None

    def volt(self, sid: int) -> int | None:
        return self.read1(sid, ADDR_PRESENT_VOLTAGE)

    def set_goal(self, sid: int, tick: int) -> None:
        self.pk.write2ByteTxRx(self.port, sid, ADDR_GOAL_POSITION, tick)

    def torque(self, sid: int, on: bool) -> None:
        self.pk.write1ByteTxRx(self.port, sid, ADDR_TORQUE_ENABLE, 1 if on else 0)

    def configure_speed(self, sid: int) -> None:
        self.pk.write1ByteTxRx(self.port, sid, ADDR_GOAL_ACC, SWEEP_ACC)
        self.pk.write2ByteTxRx(self.port, sid, ADDR_GOAL_SPEED, SWEEP_SPEED)


def sweep_direction(bus: Bus, sid: int, home: int, direction: int) -> tuple[int, str]:
    """Creep outward from home until a stop is detected. Returns (limit_tick, reason)."""
    goal = home
    last_present = home
    stuck = 0  # consecutive non-moving reads
    while True:
        goal += STEP_TICKS * direction
        if abs(goal - home) > MAX_EXCURSION or not (TICK_FLOOR <= goal <= TICK_CEIL):
            return last_present, "excursion-cap"
        bus.set_goal(sid, goal)
        time.sleep(SETTLE_S)
        present = bus.pos(sid)
        load = bus.load(sid)
        if present is None or load is None:
            return last_present, "read-fail"
        moved = abs(present - last_present)
        if load > LOAD_STOP:
            return present, f"load>{LOAD_STOP} ({load})"
        if moved < STALL_EPS:
            stuck += 1
            if stuck >= STALL_CONFIRM:
                return present, f"stall (stuck x{stuck}, load {load})"
        else:
            stuck = 0  # it moved — not a wall, keep going
        last_present = present


def calibrate(bus: Bus, ids: list[int], dry_run: bool) -> dict:
    results: dict[str, dict] = {}
    for sid in ids:
        home = bus.pos(sid)
        v = bus.volt(sid)
        print(f"\n=== ID {sid}: home={home}  volt={v/10 if v else '?'}V ===")
        if home is None:
            print("  no position read — skipping")
            continue
        if dry_run:
            print(f"  [dry-run] load={bus.load(sid)}  (no motion)")
            continue

        bus.configure_speed(sid)
        bus.torque(sid, True)
        try:
            max_tick, r_max = sweep_direction(bus, sid, home, +1)
            print(f"  + limit: {max_tick}  ({r_max})")
            bus.set_goal(sid, home)          # return to home before other dir
            time.sleep(0.5)
            min_tick, r_min = sweep_direction(bus, sid, home, -1)
            print(f"  - limit: {min_tick}  ({r_min})")
            bus.set_goal(sid, home)          # park at home
            time.sleep(0.5)
        finally:
            bus.torque(sid, False)           # limp when done — safe

        safe_min = min(min_tick, max_tick) + BACKOFF_TICKS
        safe_max = max(min_tick, max_tick) - BACKOFF_TICKS
        results[str(sid)] = {
            "home": home,
            "min_tick": safe_min,
            "max_tick": safe_max,
            "raw_min": min(min_tick, max_tick),
            "raw_max": max(min_tick, max_tick),
        }
        span_deg = (safe_max - safe_min) / 4096 * 360
        print(f"  -> safe envelope [{safe_min}, {safe_max}]  (~{span_deg:.0f} deg)")
    return results


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", default=",".join(map(str, ARM_JOINT_IDS)))
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--out", default="server/calibration.json")
    args = ap.parse_args()
    ids = [int(x) for x in args.ids.split(",") if x.strip()]

    port = PortHandler(PORT)
    if not port.openPort() or not port.setBaudRate(BAUD):
        raise SystemExit(f"failed to open {PORT} @ {BAUD}")
    bus = Bus(port, PacketHandler(0))

    # Power-rail guard: refuse to sweep on a browned-out bus.
    v = bus.volt(ids[0])
    if v is not None and v < 60 and not args.dry_run:
        port.closePort()
        raise SystemExit(f"voltage {v/10}V below 6V floor — check supply before sweeping")

    try:
        results = calibrate(bus, ids, args.dry_run)
    finally:
        port.closePort()

    if results and not args.dry_run:
        with open(args.out, "w") as f:
            json.dump(results, f, indent=2)
        print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
