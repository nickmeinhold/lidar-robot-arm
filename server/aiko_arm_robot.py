#!/usr/bin/env python3
"""SO-100 arm as an Aiko Services *robot* Actor — drive the arm from aiko chat.

This makes the physical arm show up on the Aiko Services bus as a discoverable
``type=robot`` Actor, exactly the shape the ``aiko-chat-bridge`` already knows
how to route to (see ``aiko-chat-bridge/examples/reference_robot.py`` — the
contract this file satisfies). Once running, a chat user types::

    @@armbot ready
    @@armbot joint elbow_pitch 30
    @@armbot open
    @@armbot wave

and the bridge turns each line into a typed Aiko call (``(joint elbow_pitch 30)``)
on this Actor's ``{topic}/in``. No bridge changes are needed — the bridge reads
our command vocabulary from the ``commands=`` registration tag.

Design — one door into the hardware
-----------------------------------
We do NOT touch the servos directly. Each command is compiled into an
``arm_state`` message (the very same JSON the iPhone streams) and sent over the
WebSocket to the running ``server.py`` on ``/viewer``. That reuses the server's
identical clamp / (future) collision-safety pipeline, so a chat-driven move is
subject to the same limits as a body-tracked one. The Actor holds no serial
port and can run anywhere on the bus; the arm server stays the single mutator.

    aiko chat ─▶ aiko-chat-bridge ─(MQTT)▶ THIS Actor ─(WebSocket arm_state)▶ server.py ─▶ SO-100

Run
---
    # start the arm server first (it owns the serial port):
    python -m server --port 8765 ...
    # then, on the aiko bus:
    python -m server.aiko_arm_robot                 # name "armbot", ws to localhost:8765
    python -m server.aiko_arm_robot armbot --url ws://192.168.1.20:8765/viewer
"""
from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import sys
import threading
import time

import websockets

import aiko_services as aiko
from aiko_services.main import (
    Actor,
    Interface,
    SERVICE_PROTOCOL_AIKO,
    actor_args,
    compose_instance,
)
# The process facade carries the live MQTT publisher used to reply on topic_out.
from aiko_services.main import aiko as aiko_process
from aiko_services.main.utilities import generate, get_logger

log = get_logger(__name__)

_VERSION = 0
ACTOR_TYPE = "armbot"
PROTOCOL = f"{SERVICE_PROTOCOL_AIKO}/robot:{_VERSION}"

# The five rotational joints the server's ``arm_state`` expects (radians), in
# addition to the 0..1 ``gripper`` fraction. Names MUST match protocol.ArmAngles.
JOINT_NAMES = (
    "shoulder_yaw",
    "shoulder_pitch",
    "elbow_pitch",
    "wrist_pitch",
    "wrist_roll",
)

# Named poses, expressed in the same joint-angle (radians) space the iPhone
# streams — 0.0 == each servo's calibrated centre. The server clamps every value
# to its safe tick range, so these presets can be gentle and human-legible.
HOME_POSE = {name: 0.0 for name in JOINT_NAMES}
HOME_GRIP = 0.5
READY_POSE = {
    "shoulder_yaw": 0.0,
    "shoulder_pitch": math.radians(-15),  # lean slightly back
    "elbow_pitch": math.radians(25),      # elbow up, presenting the gripper
    "wrist_pitch": 0.0,
    "wrist_roll": 0.0,
}
READY_GRIP = 0.6

DEFAULT_URL = os.environ.get("ARM_WS_URL", "ws://127.0.0.1:8765/viewer")


class _ArmLink:
    """A thread-safe WebSocket sender for ``arm_state`` messages.

    Aiko dispatches Actor command methods on its own (non-asyncio) thread, so we
    own a private asyncio loop on a daemon thread and marshal each send onto it.
    One short-lived connection per send keeps reconnection trivial and robust —
    chat commands are infrequent, so per-command connect cost is irrelevant.
    """

    def __init__(self, url: str) -> None:
        self._url = url
        self._loop = asyncio.new_event_loop()
        threading.Thread(
            target=self._loop.run_forever, name="arm-link", daemon=True
        ).start()

    def send_state(
        self, angles: dict[str, float], gripper: float, timeout: float = 3.0
    ) -> None:
        """Send one ``arm_state`` (blocks until sent or *timeout*). Raises on failure."""
        fut = asyncio.run_coroutine_threadsafe(
            self._send(angles, gripper), self._loop
        )
        fut.result(timeout=timeout)

    async def _send(self, angles: dict[str, float], gripper: float) -> None:
        payload = json.dumps(
            {
                "type": "arm_state",
                "timestamp": 0.0,  # server only echoes this for latency pings
                "angles": {**angles, "gripper": max(0.0, min(1.0, gripper))},
                "tracking": {"body": True, "hand": True},
            }
        )
        async with websockets.connect(self._url) as ws:
            await ws.send(payload)


class SO100Arm(Actor):
    Interface.default("SO100Arm", "__main__.SO100ArmImpl")

    # Command vocabulary — each maps to a same-named method, and is advertised
    # in the ``commands=`` tag so chat users see what the arm accepts.
    def ready(self):  # go to a neutral "presenting" pose
        pass

    def home(self):  # all joints centred, gripper half-open
        pass

    def open(self):  # open the gripper
        pass

    def close(self):  # close the gripper
        pass

    def gripper(self, percent):  # set gripper 0..100 %
        pass

    def joint(self, name, degrees):  # set one joint to an absolute angle (deg)
        pass

    def wave(self):  # a friendly wrist-roll wave
        pass


class SO100ArmImpl(SO100Arm):
    def __init__(self, context):
        context.call_init(self, "Actor", context)
        self.share["source_file"] = f"v{_VERSION}⇒ {__file__}"
        url = os.environ.get("ARM_WS_URL", DEFAULT_URL)
        self._link = _ArmLink(url)
        self._pose = dict(HOME_POSE)
        self._grip = HOME_GRIP
        log.info("SO100Arm linking to %s", url)

    # --- helpers ---------------------------------------------------------

    def _reply(self, text: str) -> None:
        """Speak back on the Actor's output topic; the bridge relays it to chat."""
        log.info("[%s] %s", self.name, text)
        print(f"[{self.name}] {text}", flush=True)
        aiko_process.message.publish(self.topic_out, generate("message", [text]))

    def _drive(self) -> None:
        """Push the current pose to the arm server, replying on success/failure."""
        try:
            self._link.send_state(self._pose, self._grip)
        except Exception as exc:  # noqa: BLE001 — surface any failure into chat
            self._reply(f"⚠️ couldn't reach the arm server: {exc}")
            raise

    # --- commands --------------------------------------------------------

    def ready(self):
        self._pose = dict(READY_POSE)
        self._grip = READY_GRIP
        self._drive()
        self._reply("ready — gripper presented 🙌")

    def home(self):
        self._pose = dict(HOME_POSE)
        self._grip = HOME_GRIP
        self._drive()
        self._reply("home — all joints centred")

    def open(self):
        self._grip = 1.0
        self._drive()
        self._reply("gripper open")

    def close(self):
        self._grip = 0.0
        self._drive()
        self._reply("gripper closed")

    def gripper(self, percent):
        try:
            pct = float(percent)
        except (TypeError, ValueError):
            self._reply(f"gripper: '{percent}' isn't a number (0–100)")
            return
        self._grip = max(0.0, min(1.0, pct / 100.0))
        self._drive()
        self._reply(f"gripper at {self._grip * 100:.0f}%")

    def joint(self, name, degrees):
        if name not in JOINT_NAMES:
            self._reply(f"unknown joint '{name}'. try: {', '.join(JOINT_NAMES)}")
            return
        try:
            deg = float(degrees)
        except (TypeError, ValueError):
            self._reply(f"joint {name}: '{degrees}' isn't a number of degrees")
            return
        self._pose[name] = math.radians(deg)
        self._drive()
        self._reply(f"{name} → {deg:.0f}° (server clamps to safe range)")

    def wave(self):
        saved_roll = self._pose["wrist_roll"]
        try:
            for offset in (25, -25, 25, -25, 0):
                self._pose["wrist_roll"] = saved_roll + math.radians(offset)
                self._drive()
                time.sleep(0.35)
        finally:
            self._pose["wrist_roll"] = saved_roll
        self._reply("👋")


def main() -> None:
    args = [a for a in sys.argv[1:]]
    url = DEFAULT_URL
    if "--url" in args:
        i = args.index("--url")
        url = args[i + 1]
        del args[i : i + 2]
    name = args[0] if args else ACTOR_TYPE

    os.environ["ARM_WS_URL"] = url  # read by SO100ArmImpl.__init__
    tags = ["type=robot", "commands=ready,home,open,close,gripper,joint,wave"]
    init_args = actor_args(name, protocol=PROTOCOL, tags=tags)
    compose_instance(SO100ArmImpl, init_args)
    log.info("SO-100 arm robot %r registered (protocol %s, tags %s)",
             name, PROTOCOL, tags)
    aiko.process.run()


if __name__ == "__main__":
    main()
