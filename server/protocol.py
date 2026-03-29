"""Message protocol for iPhone ↔ server communication.

The iPhone sends ``arm_state`` messages at ~30 Hz containing joint angles
(radians) and tracking status.  The server echoes a ``pong`` with the
original timestamp so the phone can measure round-trip latency.
"""

from __future__ import annotations

import json
import logging
import math
from dataclasses import dataclass

log = logging.getLogger(__name__)


@dataclass(frozen=True, slots=True)
class ArmAngles:
    """Six joint angles in radians (gripper is 0.0–1.0 fraction)."""

    shoulder_yaw: float
    shoulder_pitch: float
    elbow_pitch: float
    wrist_pitch: float
    wrist_roll: float
    gripper: float  # 1.0 = fully open, 0.0 = closed

    def degrees_str(self) -> str:
        """Format angles as a compact human-readable string (degrees + grip %)."""
        sy = math.degrees(self.shoulder_yaw)
        sp = math.degrees(self.shoulder_pitch)
        ep = math.degrees(self.elbow_pitch)
        wp = math.degrees(self.wrist_pitch)
        wr = math.degrees(self.wrist_roll)
        grip_pct = self.gripper * 100
        return (
            f"SY:{sy:6.1f}° SP:{sp:6.1f}° EP:{ep:6.1f}° "
            f"WP:{wp:6.1f}° WR:{wr:6.1f}° G:{grip_pct:4.0f}%"
        )


@dataclass(frozen=True, slots=True)
class TrackingStatus:
    """Whether ARKit body and Vision hand tracking are active."""

    body: bool
    hand: bool

    def __str__(self) -> str:
        b = "OK" if self.body else "--"
        h = "OK" if self.hand else "--"
        return f"body:{b} hand:{h}"


@dataclass(frozen=True, slots=True)
class ArmStateMessage:
    """A parsed ``arm_state`` message from the iPhone."""

    timestamp: float
    angles: ArmAngles
    tracking: TrackingStatus


def parse_message(raw: str) -> ArmStateMessage | None:
    """Parse a JSON ``arm_state`` message, returning *None* on any error."""
    try:
        data = json.loads(raw)

        if data.get("type") != "arm_state":
            log.debug("Ignoring message type: %s", data.get("type"))
            return None

        angles_data = data["angles"]
        angles = ArmAngles(
            shoulder_yaw=float(angles_data["shoulder_yaw"]),
            shoulder_pitch=float(angles_data["shoulder_pitch"]),
            elbow_pitch=float(angles_data["elbow_pitch"]),
            wrist_pitch=float(angles_data["wrist_pitch"]),
            wrist_roll=float(angles_data["wrist_roll"]),
            gripper=float(angles_data["gripper"]),
        )

        tracking_data = data["tracking"]
        tracking = TrackingStatus(
            body=bool(tracking_data["body"]),
            hand=bool(tracking_data["hand"]),
        )

        return ArmStateMessage(
            timestamp=float(data["timestamp"]),
            angles=angles,
            tracking=tracking,
        )
    except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
        log.warning("Malformed message: %s", exc)
        return None


def make_pong(timestamp: float) -> str:
    """Create a ``pong`` response echoing the original *timestamp*."""
    return json.dumps({"type": "pong", "timestamp": timestamp})
