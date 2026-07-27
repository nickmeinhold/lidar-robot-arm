"""Live all-joint load monitor for the SO-100 / STS3215 arm.

The first step toward reactive self-collision avoidance: before we can react
to a collision we have to *see* one. This drives NO servo. Torque stays OFF on
every joint the whole time. You back-drive joints BY HAND toward known-bad
configs (e.g. wrist down while the shoulder rotates) and watch the readout for
a load spike blooming on a joint you AREN'T touching — the signature of two
links coming into contact.

What it shows, per joint, refreshed ~10x/s:
  - signed load   : magnitude (0-1000 = 0-100% rated torque) with direction
                    sign (bit 10 of reg 0x3C). Contact between two links tends
                    to show OPPOSITE-signed load on the pair — that correlated
                    pair is a cleaner collision signal than either magnitude.
  - bar           : quick visual of magnitude
  - peak          : peak-hold with slow decay, so a fast spike stays readable
  - baseline      : slow EMA — "ordinary" load (e.g. lifting the arm's weight)

Every rising-edge spike snapshots ALL joints (signed load + position) with a
timestamp to a capture file. That capture is the seed data for every threshold
decision in the reactive guard: it tells us how big a real-contact spike is,
which joints couple, and their direction relationship.

SAFETY:
  - Torque is written OFF on all IDs at start and never turned on.
  - No GOAL_POSITION is ever written. Reads only.

Usage:
  python -m server.scripts.load_monitor                 # all 6 joints
  python -m server.scripts.load_monitor --ids 2,3,4     # subset
  python -m server.scripts.load_monitor --once          # single snapshot, exit
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

from server.scripts.calibrate_limits import (
    Bus,
    ADDR_PRESENT_POSITION,
    ADDR_PRESENT_LOAD,
    ADDR_PRESENT_VOLTAGE,
    PORT,
    BAUD,
)

ALL_IDS = [1, 2, 3, 4, 5, 6]
JOINT_NAMES = {
    1: "shoulder_yaw",
    2: "shoulder_pitch",
    3: "elbow",
    4: "wrist_pitch",
    5: "wrist_roll",
    6: "gripper",
}

# --- spike detection tuning --------------------------------------------------
SPIKE_FLOOR = 60         # ignore anything below this magnitude as not-a-contact
SPIKE_DELTA = 40         # must exceed baseline by this much to count as a spike
BASELINE_ALPHA = 0.03    # EMA weight — slow, so a sustained push becomes baseline
PEAK_DECAY = 0.90        # peak-hold multiplier per frame (~visible for ~1s @10Hz)
POLL_HZ = 10.0


class CameraGrabber:
    """Best-effort still capture from the iPhone Continuity Camera on a spike.

    Deliberately a side-channel: it NEVER blocks the load loop. A grab is
    fired as a detached ffmpeg process; if a prior grab is still running or we
    grabbed within `cooldown_s`, we skip (avfoundation allows one capture
    session at a time, and the hand-driven pose persists for seconds anyway).
    The frame lands ~warmup/30 s after the trigger — fine while Nick moves the
    arm slowly by hand; this is for design-time understanding, not the runtime
    guard.
    """

    def __init__(self, device: str, out_dir: str, warmup: int = 18,
                 cooldown_s: float = 1.5):
        self.device = device
        self.out_dir = out_dir
        self.warmup = warmup
        self.cooldown_s = cooldown_s
        self._last = 0.0
        self._proc: subprocess.Popen | None = None
        os.makedirs(out_dir, exist_ok=True)

    def maybe_grab(self, tag: str) -> str | None:
        now = time.time()
        if now - self._last < self.cooldown_s:
            return None
        if self._proc is not None and self._proc.poll() is None:
            return None  # a grab is still in flight — don't contend for the camera
        path = os.path.join(self.out_dir, f"spike_{tag}.jpg")
        self._proc = subprocess.Popen(
            ["ffmpeg", "-loglevel", "error", "-f", "avfoundation",
             "-framerate", "30", "-i", self.device,
             "-frames:v", str(self.warmup), "-update", "1", "-y", path],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        self._last = now
        return path


def decode_load(raw: int | None) -> tuple[int, int]:
    """Return (signed_load, magnitude). direction sign = bit 10 (0x400)."""
    if raw is None:
        return 0, 0
    mag = raw & 0x03FF
    sign = -1 if (raw & 0x0400) else 1
    return sign * mag, mag


def bar(mag: int, width: int = 20, full: int = 500) -> str:
    n = min(width, int(mag / full * width))
    return "#" * n + "-" * (width - n)


def snapshot(bus: Bus, ids: list[int]) -> dict[int, dict]:
    """One read pass over all joints. Returns per-id {pos, signed, mag}."""
    out: dict[int, dict] = {}
    for sid in ids:
        pos = bus.read2(sid, ADDR_PRESENT_POSITION)
        raw = bus.read2(sid, ADDR_PRESENT_LOAD)
        signed, mag = decode_load(raw)
        out[sid] = {"pos": pos, "signed": signed, "mag": mag}
    return out


def run(bus: Bus, ids: list[int], once: bool, capture_path: str,
        grabber: CameraGrabber | None = None) -> None:
    # Belt-and-suspenders: force torque OFF. We never turn it back on.
    for sid in ids:
        bus.torque(sid, False)

    baseline = {sid: 0.0 for sid in ids}
    peak = {sid: 0 for sid in ids}
    in_spike = {sid: False for sid in ids}
    captures: list[dict] = []
    t0 = time.time()

    n_rows = len(ids)
    printed_block = False
    period = 1.0 / POLL_HZ

    try:
        while True:
            snap = snapshot(bus, ids)

            # --- spike detection (rising-edge, per joint) --------------------
            event_ids = []
            for sid in ids:
                mag = snap[sid]["mag"]
                base = baseline[sid]
                is_spike = mag >= SPIKE_FLOOR and mag > base + SPIKE_DELTA
                if is_spike and not in_spike[sid]:
                    event_ids.append(sid)
                in_spike[sid] = is_spike
                # baseline only tracks non-spiking samples so a real contact
                # doesn't get absorbed into "ordinary"
                if not is_spike:
                    baseline[sid] = base + BASELINE_ALPHA * (mag - base)
                peak[sid] = max(int(peak[sid] * PEAK_DECAY), mag)

            if event_ids:
                rec = {
                    "t": round(time.time() - t0, 3),
                    "triggered_by": event_ids,
                    "joints": {
                        str(sid): {
                            "name": JOINT_NAMES.get(sid, "?"),
                            "pos": snap[sid]["pos"],
                            "load": snap[sid]["signed"],
                        }
                        for sid in ids
                    },
                }
                if grabber is not None:
                    rec["frame"] = grabber.maybe_grab(f"{len(captures):03d}")
                captures.append(rec)

            # --- render ------------------------------------------------------
            lines = []
            hdr = f"{'ID':>2} {'joint':<15}{'pos':>6}{'load':>7}  {'bar':<20} {'peak':>5}{'base':>6}"
            lines.append(hdr)
            for sid in ids:
                s = snap[sid]
                spike_mark = " <== SPIKE" if in_spike[sid] else ""
                lines.append(
                    f"{sid:>2} {JOINT_NAMES.get(sid,'?'):<15}"
                    f"{s['pos']!s:>6}{s['signed']:>7}  "
                    f"{bar(s['mag']):<20} {peak[sid]:>5}{int(baseline[sid]):>6}{spike_mark}"
                )
            lines.append(f"captures: {len(captures)}   (Ctrl-C to stop & save)")

            block = "\n".join(lines)
            if printed_block and not once:
                # move cursor up to overwrite the previous block
                sys.stdout.write(f"\x1b[{len(lines)}A")
            sys.stdout.write("\x1b[2K" + block.replace("\n", "\n\x1b[2K") + "\n")
            sys.stdout.flush()
            printed_block = True

            if once:
                break
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        if captures:
            with open(capture_path, "w") as f:
                json.dump(captures, f, indent=2)
            print(f"\nwrote {len(captures)} spike captures -> {capture_path}")
        else:
            print("\nno spikes captured")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", default=",".join(map(str, ALL_IDS)))
    ap.add_argument("--once", action="store_true", help="single snapshot then exit")
    ap.add_argument("--out", default="server/load_captures.json")
    ap.add_argument("--camera", default="2",
                    help="avfoundation device index for spike snapshots (default 2 = iPhone)")
    ap.add_argument("--no-camera", action="store_true", help="disable spike snapshots")
    ap.add_argument("--frames-dir", default="server/spike_frames")
    args = ap.parse_args()
    ids = [int(x) for x in args.ids.split(",") if x.strip()]

    grabber = None
    if not args.no_camera and not args.once:
        grabber = CameraGrabber(args.camera, args.frames_dir)

    port = PortHandler(PORT)
    if not port.openPort() or not port.setBaudRate(BAUD):
        raise SystemExit(f"failed to open {PORT} @ {BAUD}")
    bus = Bus(port, PacketHandler(0))

    # Power-rail guard: refuse to run on a browned-out bus (masquerades as a wall).
    v = bus.volt(ids[0])
    if v is not None and v < 60:
        port.closePort()
        raise SystemExit(f"voltage {v/10}V below 6V floor — check supply")

    try:
        run(bus, ids, args.once, args.out, grabber)
    finally:
        port.closePort()


if __name__ == "__main__":
    main()
