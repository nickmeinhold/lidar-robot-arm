"""Feetech STS3215 arm controller — drives the SO-100/SO-101 follower arm.

Maps iPhone-sourced joint angles (radians, plus 0–1 gripper) to servo tick
positions via a per-joint ``JointConfig`` (center tick, ticks-per-radian,
direction, soft min/max). Writes are rate-limited and run in an executor
thread so the WebSocket event loop never blocks on serial I/O.

Servos that don't respond at connect time are skipped silently — that
means a partially-populated arm still works for whatever channels are
present.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from dataclasses import dataclass

from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

from .arm_controller import ArmController
from .protocol import ArmAngles, TrackingStatus

log = logging.getLogger(__name__)

# STS3215 register addresses
ADDR_MIN_ANGLE_LIMIT = 0x09  # EEPROM — the servo's own absolute lower bound
ADDR_MAX_ANGLE_LIMIT = 0x0B  # EEPROM — the servo's own absolute upper bound
ADDR_TORQUE_ENABLE = 0x28
ADDR_GOAL_ACC = 0x29
ADDR_GOAL_POSITION = 0x2A
ADDR_GOAL_SPEED = 0x2E
ADDR_PRESENT_POSITION = 0x38

DEFAULT_BAUD = 1_000_000
SOFT_LIMIT_TICKS = 1200  # default ±~105° from home — full useful range now that
                          # we've verified nothing will slam

# Per-joint overrides for the soft-limit envelope, in ticks. Joints not
# listed fall back to SOFT_LIMIT_TICKS.
SOFT_LIMIT_OVERRIDES: dict[str, int] = {
    "elbow_pitch": 1024,  # ±90°
}
TICKS_PER_REV = 4096
TICKS_PER_RAD = TICKS_PER_REV / (2 * math.pi)


def intersect_safe_window(
    home: int,
    soft_limit: int,
    eeprom_min: int | None,
    eeprom_max: int | None,
) -> tuple[int, int]:
    """Intersect the relative soft box with the servo's own EEPROM window.

    The soft box (±soft_limit ticks around the startup *home*) is RELATIVE —
    restart the server with the arm in a strange pose and the box moves with
    it. The EEPROM Min/Max Angle Limit registers are ABSOLUTE, live in the
    servo itself, and survive restarts, so they anchor the final window in
    reality. A missing/garbage EEPROM reading (comm failure, or a degenerate
    window that doesn't even contain home) falls back to the soft box alone —
    fail toward the behavior we've always had, never toward a wider range.
    """
    lo = max(0, home - soft_limit)
    hi = min(TICKS_PER_REV - 1, home + soft_limit)
    if (
        eeprom_min is not None
        and eeprom_max is not None
        and eeprom_min < eeprom_max
        and eeprom_min <= home <= eeprom_max
    ):
        lo = max(lo, eeprom_min)
        hi = min(hi, eeprom_max)
    return lo, hi


@dataclass(frozen=True, slots=True)
class JointConfig:
    """How a single iPhone channel maps to a single servo.

    ``center_tick`` is the servo position when the input angle is zero.
    ``direction`` is +1 or -1 to flip rotation sense.
    ``min_tick`` / ``max_tick`` clamp the final commanded position to
    keep the arm inside safe joint limits regardless of input.
    """

    servo_id: int
    center_tick: int
    direction: int = 1
    min_tick: int = 0
    max_tick: int = TICKS_PER_REV - 1

    def angle_to_tick(self, angle_rad: float) -> int:
        raw = self.center_tick + int(self.direction * angle_rad * TICKS_PER_RAD)
        return max(self.min_tick, min(self.max_tick, raw))


# Default SO-100 mapping. Center ticks are placeholders — for a real arm
# you'd calibrate by reading the "home" position of each servo. Soft limits
# are conservative (±60° from center) and should be tightened per joint.
DEFAULT_JOINTS: dict[str, JointConfig] = {
    "shoulder_yaw":   JointConfig(servo_id=1, center_tick=2048, direction=+1),
    "shoulder_pitch": JointConfig(servo_id=2, center_tick=2048, direction=+1),
    "elbow_pitch":    JointConfig(servo_id=3, center_tick=2048, direction=+1),
    "wrist_pitch":    JointConfig(servo_id=4, center_tick=2048, direction=+1),
    "wrist_roll":     JointConfig(servo_id=5, center_tick=2048, direction=+1),
}

# Gripper is special: 0-1 fraction, not radians. Maps linearly between
# closed_tick and open_tick.
GRIPPER_SERVO_ID = 6
GRIPPER_CLOSED_TICK = 2048
GRIPPER_OPEN_TICK = 2700

WRITE_INTERVAL_S = 0.033  # cap servo write rate at 30 Hz (matches iPhone rate)
GOAL_SPEED = 3000         # ticks/sec — snappier; ~260°/sec
GOAL_ACC = 80             # quicker ramp without slamming


class FeetechArmController(ArmController):
    """Drives Feetech STS3215 servos from streamed iPhone joint angles."""

    def __init__(
        self,
        port_name: str,
        baud: int = DEFAULT_BAUD,
        joints: dict[str, JointConfig] | None = None,
    ) -> None:
        self._port_name = port_name
        self._baud = baud
        self._joints = joints or DEFAULT_JOINTS
        self._port: PortHandler | None = None
        self._packet: PacketHandler | None = None
        self._present_ids: set[int] = set()
        self._last_write: float = 0.0
        self._last_debug: float = 0.0
        self._reference: ArmAngles | None = None  # locked on first body:OK frame
        # Gripper's absolute EEPROM window, read at connect (None = unknown,
        # fall back to the hardcoded closed/open ticks alone).
        self._gripper_window: tuple[int, int] | None = None

    # --- lifecycle -----------------------------------------------------

    def connect(self) -> None:
        """Open the serial port and discover which servos respond."""
        port = PortHandler(self._port_name)
        if not port.openPort():
            raise RuntimeError(f"Failed to open {self._port_name}")
        if not port.setBaudRate(self._baud):
            port.closePort()
            raise RuntimeError(f"Failed to set baud {self._baud}")
        packet = PacketHandler(0)

        # Ping every joint's servo + the gripper
        all_ids = {cfg.servo_id for cfg in self._joints.values()} | {GRIPPER_SERVO_ID}
        present: set[int] = set()
        for sid in sorted(all_ids):
            _, comm, _ = packet.ping(port, sid)
            if comm == COMM_SUCCESS:
                present.add(sid)
        log.info("Found servos: %s (expected %s)", sorted(present), sorted(all_ids))

        # Read live home positions and rewrite each joint's center so
        # iPhone "zero angle" maps to "stay where you are now" instead of
        # snapping to a hardcoded tick. The commanded window is the soft box
        # around home INTERSECTED with the servo's own EEPROM angle limits —
        # the EEPROM window is absolute and survives restarts, so a server
        # that wakes with the arm in a strange pose can't slide its "safe"
        # box into territory the hardware calibration forbids.
        for name, cfg in list(self._joints.items()):
            if cfg.servo_id not in present:
                continue
            pos, comm, _ = packet.read2ByteTxRx(port, cfg.servo_id, ADDR_PRESENT_POSITION)
            if comm != COMM_SUCCESS:
                continue
            emin, comm_min, _ = packet.read2ByteTxRx(port, cfg.servo_id, ADDR_MIN_ANGLE_LIMIT)
            emax, comm_max, _ = packet.read2ByteTxRx(port, cfg.servo_id, ADDR_MAX_ANGLE_LIMIT)
            eeprom_min = emin if comm_min == COMM_SUCCESS else None
            eeprom_max = emax if comm_max == COMM_SUCCESS else None
            limit = SOFT_LIMIT_OVERRIDES.get(name, SOFT_LIMIT_TICKS)
            lo, hi = intersect_safe_window(pos, limit, eeprom_min, eeprom_max)
            self._joints[name] = JointConfig(
                servo_id=cfg.servo_id,
                center_tick=pos,
                direction=cfg.direction,
                min_tick=lo,
                max_tick=hi,
            )
            log.info("ID %d (%s): home=%d, eeprom=[%s,%s], window=[%d,%d]",
                     cfg.servo_id, name, pos, eeprom_min, eeprom_max, lo, hi)

        if GRIPPER_SERVO_ID in present:
            emin, comm_min, _ = packet.read2ByteTxRx(port, GRIPPER_SERVO_ID, ADDR_MIN_ANGLE_LIMIT)
            emax, comm_max, _ = packet.read2ByteTxRx(port, GRIPPER_SERVO_ID, ADDR_MAX_ANGLE_LIMIT)
            if comm_min == COMM_SUCCESS and comm_max == COMM_SUCCESS:
                if emax - emin >= 100:
                    self._gripper_window = (emin, emax)
                    log.info("ID %d (gripper): eeprom=[%d,%d]", GRIPPER_SERVO_ID, emin, emax)
                else:
                    # The as-shipped gripper EEPROM is a degenerate ~1-tick lock
                    # ([2046,2047] observed live 2026-08-10) that the servo has
                    # NOT been enforcing on goal writes — adopting it verbatim
                    # would freeze the gripper in software. Ignore it and flag
                    # for a proper EEPROM calibration pass.
                    log.warning(
                        "ID %d (gripper): EEPROM window [%d,%d] is degenerate — "
                        "ignoring; gripper EEPROM needs calibration",
                        GRIPPER_SERVO_ID, emin, emax)

        # Enable torque on responders, set speed/acc limits
        for sid in present:
            packet.write1ByteTxRx(port, sid, ADDR_GOAL_ACC, GOAL_ACC)
            packet.write2ByteTxRx(port, sid, ADDR_GOAL_SPEED, GOAL_SPEED)
            packet.write1ByteTxRx(port, sid, ADDR_TORQUE_ENABLE, 1)

        self._port = port
        self._packet = packet
        self._present_ids = present

    def disconnect(self) -> None:
        """Disable torque on all servos and close the port (safe limp state)."""
        if self._port and self._packet:
            for sid in self._present_ids:
                try:
                    self._packet.write1ByteTxRx(self._port, sid, ADDR_TORQUE_ENABLE, 0)
                except Exception as exc:  # noqa: BLE001
                    log.warning("Torque-off failed for ID %d: %s", sid, exc)
            self._port.closePort()
        self._port = None
        self._packet = None
        self._present_ids = set()

    # --- ArmController interface --------------------------------------

    async def update(self, angles: ArmAngles, tracking: TrackingStatus) -> None:
        now = time.monotonic()
        # Periodic debug ping so we can see whether updates are arriving
        # and why we might be skipping them.
        if now - self._last_debug > 1.0:
            log.info("rx: %s | %s", angles.degrees_str(), tracking)
            self._last_debug = now
            # Also dump current target ticks so we can see whether soft-limit
            # clamping is freezing a joint at the boundary.
            if tracking.body or tracking.hand:
                relative_for_dbg = self._relative(angles) if self._reference else angles
                dbg_targets = self._build_targets(
                    relative_for_dbg,
                    drive_arm=tracking.body,
                    drive_gripper=tracking.hand,
                )
                if dbg_targets:
                    log.info("tx: %s", " ".join(f"ID{s}={t}" for s, t in dbg_targets))

        if now - self._last_write < WRITE_INTERVAL_S:
            return
        self._last_write = now

        # Lock a reference pose on the first body:OK frame so subsequent
        # angles are sent as deltas — your "neutral" body posture becomes
        # the arm's home position.
        if tracking.body and self._reference is None:
            self._reference = angles
            log.info("Reference pose locked: %s", angles.degrees_str())

        relative = self._relative(angles) if self._reference else angles

        # Body tracking drives the arm joints; hand tracking drives the
        # gripper. Either can be active independently.
        targets = self._build_targets(relative, drive_arm=tracking.body, drive_gripper=tracking.hand)
        if targets:
            await asyncio.to_thread(self._write_targets, targets)

    async def stop(self) -> None:
        await asyncio.to_thread(self.disconnect)

    # --- teach / manual helpers ---------------------------------------

    def set_torque(self, on: bool) -> None:
        """Enable (hold) or disable (limp) torque on all present servos.

        Limp is the safe state and is what lets you hand-pose the arm for a
        teach capture. Support the arm before releasing if it's raised — it
        will drop under gravity.
        """
        if self._port is None or self._packet is None:
            return
        for sid in self._present_ids:
            self._packet.write1ByteTxRx(self._port, sid, ADDR_TORQUE_ENABLE, 1 if on else 0)
        log.info("Torque %s on %s", "ON" if on else "OFF", sorted(self._present_ids))

    def _read_tick(self, sid: int, tries: int = 5) -> int | None:
        """Read PRESENT_POSITION with retry — the SDK throws IndexError on an
        empty packet during bus hiccups, so a bare read can crash."""
        for _ in range(tries):
            try:
                v, comm, _ = self._packet.read2ByteTxRx(self._port, sid, ADDR_PRESENT_POSITION)
                if comm == COMM_SUCCESS:
                    return v
            except IndexError:
                pass
            time.sleep(0.02)
        return None

    def read_present_ticks(self) -> dict[str, int]:
        """Read live PRESENT_POSITION for every present joint, keyed by name."""
        out: dict[str, int] = {}
        if self._port is None or self._packet is None:
            return out
        for name, cfg in self._joints.items():
            if cfg.servo_id not in self._present_ids:
                continue
            tick = self._read_tick(cfg.servo_id)
            if tick is not None:
                out[name] = tick
        if GRIPPER_SERVO_ID in self._present_ids:
            tick = self._read_tick(GRIPPER_SERVO_ID)
            if tick is not None:
                out["gripper"] = tick
        return out

    # --- internals -----------------------------------------------------

    def _relative(self, angles: ArmAngles) -> ArmAngles:
        """Subtract the reference pose from a fresh frame. Gripper passes through."""
        r = self._reference
        assert r is not None  # caller checked
        return ArmAngles(
            shoulder_yaw=angles.shoulder_yaw - r.shoulder_yaw,
            shoulder_pitch=angles.shoulder_pitch - r.shoulder_pitch,
            elbow_pitch=angles.elbow_pitch - r.elbow_pitch,
            wrist_pitch=angles.wrist_pitch - r.wrist_pitch,
            wrist_roll=angles.wrist_roll - r.wrist_roll,
            gripper=angles.gripper,
        )

    def _build_targets(
        self,
        angles: ArmAngles,
        drive_arm: bool = True,
        drive_gripper: bool = True,
    ) -> list[tuple[int, int]]:
        """Convert an ArmAngles into (servo_id, goal_tick) pairs."""
        out: list[tuple[int, int]] = []
        if drive_arm:
            channels = {
                "shoulder_yaw": angles.shoulder_yaw,
                "shoulder_pitch": angles.shoulder_pitch,
                "elbow_pitch": angles.elbow_pitch,
                "wrist_pitch": angles.wrist_pitch,
                "wrist_roll": angles.wrist_roll,
            }
            for name, angle in channels.items():
                cfg = self._joints.get(name)
                if cfg is None or cfg.servo_id not in self._present_ids:
                    continue
                out.append((cfg.servo_id, cfg.angle_to_tick(angle)))

        if drive_gripper and GRIPPER_SERVO_ID in self._present_ids:
            g = max(0.0, min(1.0, angles.gripper))
            tick = int(GRIPPER_CLOSED_TICK + g * (GRIPPER_OPEN_TICK - GRIPPER_CLOSED_TICK))
            if self._gripper_window is not None:
                tick = max(self._gripper_window[0], min(self._gripper_window[1], tick))
            out.append((GRIPPER_SERVO_ID, tick))
        return out

    def _write_targets(self, targets: list[tuple[int, int]]) -> None:
        if self._port is None or self._packet is None:
            return
        for sid, tick in targets:
            self._packet.write2ByteTxRx(self._port, sid, ADDR_GOAL_POSITION, tick)
