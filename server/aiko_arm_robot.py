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
        self,
        angles: dict[str, float],
        gripper: float,
        drive_gripper: bool = True,
        timeout: float = 3.0,
    ) -> None:
        """Send one ``arm_state`` (blocks until sent or *timeout*). Raises on failure.

        ``drive_gripper=False`` maps to ``tracking.hand=false`` — the server then
        leaves the gripper servo untouched (used by the reference seed).
        """
        fut = asyncio.run_coroutine_threadsafe(
            self._send(angles, gripper, drive_gripper), self._loop
        )
        fut.result(timeout=timeout)

    async def _send(
        self, angles: dict[str, float], gripper: float, drive_gripper: bool
    ) -> None:
        payload = json.dumps(
            {
                "type": "arm_state",
                "timestamp": 0.0,  # server only echoes this for latency pings
                "angles": {**angles, "gripper": max(0.0, min(1.0, gripper))},
                "tracking": {"body": True, "hand": drive_gripper},
            }
        )
        async with websockets.connect(self._url) as ws:
            await ws.send(payload)


class ArmCommandEngine:
    """Pose state + command execution — the ONE implementation of the arm's
    chat-facing vocabulary, shared by the bus-RPC Actor (this file) and the
    aiko-chat bot (``aiko_arm_chatbot.py``). Every command returns the reply
    string to surface back to the human; hardware failures come back as a
    ⚠️ string rather than raising, so callers can always just relay."""

    COMMANDS = ("ready", "home", "open", "close", "gripper", "joint", "wave")

    def __init__(self, url: str) -> None:
        self._link = _ArmLink(url)
        self._pose = dict(HOME_POSE)
        self._grip = HOME_GRIP
        self._seeded = False  # reference seed sent yet? (see _drive)

    def help(self) -> str:
        return (f"commands: {', '.join(self.COMMANDS)} · "
                f"joints: {', '.join(JOINT_NAMES)} · "
                "e.g. 'joint wrist_pitch 20', 'gripper 50'")

    def execute(self, command: str, args: list[str]) -> str:
        """Dispatch a named command with string args (chat / RPC boundary —
        decode-to-typed happens inside each command, once, here at the edge)."""
        if command in ("help", "?"):
            return self.help()
        if command not in self.COMMANDS:
            return f"unknown command '{command}'. {self.help()}"
        try:
            return getattr(self, command)(*args)
        except TypeError:
            return f"'{command}' arguments look wrong. {self.help()}"
        except Exception as exc:  # noqa: BLE001 — surface into chat, never die
            return f"⚠️ couldn't reach the arm server: {exc}"

    def _drive(self) -> None:
        """Push the current pose to the arm server.

        First drive ever sends a zero-pose *reference seed* with
        ``tracking.hand=false``: the controller locks its reference on the first
        ``body:OK`` frame, so seeding zeros pins the reference at 0 — making all
        our joint angles mean "relative to the arm's startup pose" — while the
        ``hand=false`` leaves the gripper servo untouched (zero motion seed).
        Without this, our first real command would silently BECOME the zero.
        """
        if not self._seeded:
            self._link.send_state(dict(HOME_POSE), 0.0, drive_gripper=False)
            self._seeded = True
            time.sleep(0.2)  # let the server lock the reference first
        self._link.send_state(self._pose, self._grip)

    # --- commands (each returns the chat reply) --------------------------

    def ready(self) -> str:
        self._pose = dict(READY_POSE)
        self._grip = READY_GRIP
        self._drive()
        return "ready — gripper presented 🙌"

    def home(self) -> str:
        self._pose = dict(HOME_POSE)
        self._grip = HOME_GRIP
        self._drive()
        return "home — all joints centred"

    def open(self) -> str:
        self._grip = 1.0
        self._drive()
        return "gripper open"

    def close(self) -> str:
        self._grip = 0.0
        self._drive()
        return "gripper closed"

    def gripper(self, percent) -> str:
        try:
            pct = float(percent)
        except (TypeError, ValueError):
            return f"gripper: '{percent}' isn't a number (0–100)"
        self._grip = max(0.0, min(1.0, pct / 100.0))
        self._drive()
        return f"gripper at {self._grip * 100:.0f}%"

    # Thumb-friendly aliases: chat users type "wrist" or "wrist pitch", not
    # "wrist_pitch". Multi-word names are joined by joint() before lookup.
    JOINT_ALIASES = {
        "wrist": "wrist_pitch",
        "elbow": "elbow_pitch",
        "roll": "wrist_roll",
        "shoulder": "shoulder_pitch",
        "base": "shoulder_yaw",
        "pan": "shoulder_yaw",
        "yaw": "shoulder_yaw",
    }

    def joint(self, *args) -> str:
        """``joint <name...> <degrees>`` — the LAST token is the angle, every
        token before it is the joint name (so "wrist pitch 20" works)."""
        if len(args) < 2:
            return f"usage: joint <name> <degrees> — joints: {', '.join(JOINT_NAMES)}"
        *name_parts, degrees = args
        name = "_".join(str(p).lower() for p in name_parts)
        name = self.JOINT_ALIASES.get(name, name)
        if name not in JOINT_NAMES:
            return f"unknown joint '{name}'. try: {', '.join(JOINT_NAMES)}"
        try:
            deg = float(degrees)
        except (TypeError, ValueError):
            return f"joint {name}: '{degrees}' isn't a number of degrees"
        self._pose[name] = math.radians(deg)
        self._drive()
        return f"{name} → {deg:.0f}° (server clamps to safe range)"

    def wave(self) -> str:
        saved_roll = self._pose["wrist_roll"]
        try:
            for offset in (25, -25, 25, -25, 0):
                self._pose["wrist_roll"] = saved_roll + math.radians(offset)
                self._drive()
                time.sleep(0.35)
        finally:
            self._pose["wrist_roll"] = saved_roll
        return "👋"


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
        self._engine = ArmCommandEngine(url)
        log.info("SO100Arm linking to %s", url)

    # --- helpers ---------------------------------------------------------

    def _reply(self, text: str) -> None:
        """Speak back on the Actor's output topic; the bridge relays it to chat."""
        log.info("[%s] %s", self.name, text)
        print(f"[{self.name}] {text}", flush=True)
        aiko_process.message.publish(self.topic_out, generate("message", [text]))

    def _run(self, command: str, *args) -> None:
        self._reply(self._engine.execute(command, [str(a) for a in args]))

    # --- commands (thin RPC shims over the shared ArmCommandEngine) -------

    def ready(self):
        self._run("ready")

    def home(self):
        self._run("home")

    def open(self):
        self._run("open")

    def close(self):
        self._run("close")

    def gripper(self, percent):
        self._run("gripper", percent)

    def joint(self, name, degrees):
        self._run("joint", name, degrees)

    def wave(self):
        self._run("wave")

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
