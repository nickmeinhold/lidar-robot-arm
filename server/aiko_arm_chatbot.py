#!/usr/bin/env python3
"""SO-100 arm as an aiko-chat bot — drive the arm from the Aiko Chat app.

This is the chat-side face of the arm. Where ``aiko_arm_robot.py`` exposes the
arm as a bus-RPC robot Actor, this joins the island's ChatServer as a chat
*participant* (an ``aiko_chat.ChatBot`` subclass): it sits in a channel, watches
for ``@@armbot <command>`` mentions, executes them through the shared
:class:`ArmCommandEngine`, and replies in-channel. Because the island gateway
fans every non-human bus message out to connected apps (``sender_kind='actor'``),
the Aiko Chat app on a phone sees the arm answer like any other member.

    Aiko Chat app ──WSS──▶ island gateway ──MQTT──▶ ChatServer ──▶ THIS bot
                                                                     │
                                             arm server ◀─WebSocket──┘
                                                  │
                                                SO-100

Usage::

    # local bus (default):    talks to ChatServer wherever the registrar says
    python -m server.aiko_arm_chatbot
    # explicit name/channel/arm:
    python -m server.aiko_arm_chatbot @@armbot --channel general \
        --url ws://127.0.0.1:8765/viewer

To join a REMOTE island (e.g. chat.enspyr.co) whose broker has no public port,
tunnel first and point aiko at it::

    ssh -N -L 1885:<mosquitto-container-ip>:1883 <island-host> &
    AIKO_MQTT_HOST=127.0.0.1 AIKO_MQTT_PORT=1885 python -m server.aiko_arm_chatbot
"""
from __future__ import annotations

import json
import os
import sys

import aiko_services as aiko
import aiko_chat.bot as _aiko_bot
from aiko_chat.bot import ChatBot, _ACTOR_BOT, _PROTOCOL_BOT
# The exact decoder the ChatServer/gateway use for channel payloads — reusing it
# (rather than a lookalike parser) keeps both sides of the wire on one codec.
from aiko_chat.protocol import _decode_message
from aiko_services.main.utilities import get_logger

from .aiko_arm_robot import ArmCommandEngine, DEFAULT_URL

log = get_logger(__name__)

DEFAULT_BOTNAME = "@@armbot"
DEFAULT_CHANNEL = "general"

# --- English → command translation (zero-cost Max via OAuth Bearer) ---------
# An unrecognized "@@armbot <free text>" goes through Claude Haiku, which may
# ONLY emit the engine's own vocabulary — the engine validates and the arm
# server clamps, so wild input can never exceed what typed commands already do.
_OAUTH_TOKEN = (os.environ.get("CLAUDE_CODE_OAUTH_TOKEN")
                or os.environ.get("ANTHROPIC_OAUTH_TOKEN") or "")
_TRANSLATE_SYSTEM = (
    "You translate chat messages into robot arm commands. Vocabulary: ready, "
    "home, open, close, wave, 'gripper <0-100>', 'joint <shoulder_yaw|"
    "shoulder_pitch|elbow_pitch|wrist_pitch|wrist_roll> <degrees -50..50>'. "
    "Reply ONLY a compact JSON array of at most 6 command strings, no code "
    "fences, no prose. Sequences are allowed (a wiggle = several joint moves). "
    "If the message is not an arm request, reply []"
)


def translate_to_commands(text: str) -> list[str]:
    """Free text → engine command strings via Claude Haiku. [] on any failure."""
    if not _OAUTH_TOKEN:
        return []
    import urllib.request
    body = json.dumps({
        "model": "claude-haiku-4-5-20251001",
        "max_tokens": 150,
        "system": _TRANSLATE_SYSTEM,
        "messages": [{"role": "user", "content": text[:500]}],
    }).encode()
    req = urllib.request.Request(
        "https://api.anthropic.com/v1/messages", data=body,
        headers={
            "Authorization": f"Bearer {_OAUTH_TOKEN}",
            "anthropic-beta": "oauth-2025-04-20",
            "anthropic-version": "2023-06-01",
            "content-type": "application/json",
        })
    try:
        with urllib.request.urlopen(req, timeout=12) as resp:
            reply = json.loads(resp.read())["content"][0]["text"]
        commands = json.loads(reply.strip())
        return [c for c in commands if isinstance(c, str)][:6]
    except Exception as exc:  # noqa: BLE001 — degrade to "didn't catch that"
        log.warning("translate failed: %s", exc)
        return []


def _any_version_server_filter() -> aiko.ServiceFilter:
    """Match a ChatServer under ANY protocol version.

    ``aiko_chat.bot``'s stock filter pins the protocol version of the LOCAL
    checkout (``chat_server:1``), so an island still running an older image
    (``chat_server:0`` — chat.enspyr.co as of 2026-08-03) is silently invisible
    to discovery. Actor name + wildcard protocol keeps us compatible in both
    directions; drop this once the islands run a matching aiko_chat.
    """
    return aiko.ServiceFilter("*", "chat_server", "*", "*", "*", "*")


# ChatBotImpl.__init__ resolves this name from its own module globals at call
# time, so patching it here re-scopes discovery for every bot in this process.
_aiko_bot.get_server_service_filter = _any_version_server_filter


class ArmChatBot(ChatBot):
    """A ChatBot whose vocabulary is the arm's command engine.

    ``ChatBotImpl`` (wired in via ``Interface.default``) does the heavy lifting:
    ChatServer discovery, channel subscribe, reconnect. We only implement
    :meth:`process_message`.
    """

    def __init__(self, context, botname: str, ws_url: str, channel: str,
                 wave_on_message: bool = False):
        self.current_channel = channel  # read by ChatBotImpl's discovery hook
        context.call_init(self, "ChatBot", context)
        self.botname = botname
        self.engine = ArmCommandEngine(ws_url)
        # Greeter mode (meetup demo): physically wave at ANY human message in
        # the channel, rate-limited so a busy chat doesn't queue waves forever.
        self.wave_on_message = wave_on_message
        self._wave_cooldown_s = 8.0
        self._last_greet = 0.0
        log.info("ArmChatBot %s watching #%s, arm at %s (greeter=%s)",
                 botname, channel, ws_url, wave_on_message)

    def process_message(self, payload_in, **kwargs):
        fields = _decode_message(payload_in)
        if not fields:
            return
        username = str(fields.get("username") or "")
        text = str(fields.get("message") or "").strip()
        # Log EVERYTHING we see — a silently-ignored message is indistinguishable
        # from a lost one without this (bit us live: an auto-capitalized mention
        # vanished without a trace and read as "command worked, arm didn't move").
        self.print(f"saw {username or '?'}: {text!r}")
        if username == self.botname:
            return  # our own reply echoing back — never self-respond
        # Addressing grammar — reconciled with the bridge's robot sigil
        # (aiko-chat-bridge robots.py: ^@@ required, single @ is a PERSON
        # mention, never a robot command). The FIRST whitespace token must be
        # exactly '@@<botname>' (case-insensitive — phone keyboards auto-
        # capitalize): '@armbot wave' is someone autocomplete-mentioning the
        # bot in conversation and must not drive hardware; '@@arm wave' is a
        # different (or mistyped) robot, not a prefix of us.
        tokens = text.split()
        if not tokens or tokens[0].lower() != self.botname.lower():
            # Ordinary chat. In greeter mode, wave hello (cooldown-limited) —
            # physical motion only, no chat reply (a reply per message is spam).
            if self.wave_on_message and tokens:
                import time as _time
                now = _time.monotonic()
                if now - self._last_greet >= self._wave_cooldown_s:
                    self._last_greet = now
                    self.print(f"greeting {username or '?'} 👋")
                    self.engine.execute("wave", [])
            return
        rest = " ".join(tokens[1:])
        if not rest:
            reply = self.engine.help()
        else:
            command, *args = rest.split()
            if command.lower() in self.engine.COMMANDS or \
                    command.lower() in ("help", "?"):
                reply = self.engine.execute(command, args)  # fast, deterministic
            else:
                reply = self._english(rest)  # free text → translated sequence
        self.print(f"{username or '?'}: {rest!r} -> {reply!r}")
        if self.chat_server:
            self.chat_server.send_message(
                self.botname, [self.current_channel], reply)

    def _english(self, text: str) -> str:
        """Translate free text into a command sequence and run it."""
        commands = translate_to_commands(text)
        if not commands:
            return f"didn't catch that — {self.engine.help()}"
        import time as _time
        replies = []
        for i, command_line in enumerate(commands):
            if i:  # dwell so each step of a sequence is VISIBLE — back-to-back
                _time.sleep(0.6)  # goal writes make the servo chase only the last
            token, *args = command_line.split()
            replies.append(self.engine.execute(token, args))
        return " · ".join(replies)


def main() -> None:
    args = list(sys.argv[1:])
    url = DEFAULT_URL
    channel = DEFAULT_CHANNEL
    if "--url" in args:
        i = args.index("--url")
        url = args[i + 1]
        del args[i:i + 2]
    if "--channel" in args:
        i = args.index("--channel")
        channel = args[i + 1]
        del args[i:i + 2]
    wave_on_message = "--wave-on-message" in args
    if wave_on_message:
        args.remove("--wave-on-message")
    botname = args[0] if args else DEFAULT_BOTNAME
    # The robot sigil is exactly two '@' (bridge grammar): normalize whatever
    # was typed ('armbot', '@armbot') so the matcher never accepts single-@.
    botname = "@@" + botname.lstrip("@")

    os.environ.setdefault("ARM_WS_URL", url)
    init_args = aiko.actor_args(_ACTOR_BOT, protocol=_PROTOCOL_BOT,
                                tags=["ec=true"])
    init_args["botname"] = botname
    init_args["ws_url"] = url
    init_args["channel"] = channel
    init_args["wave_on_message"] = wave_on_message
    aiko.compose_instance(ArmChatBot, init_args)
    log.info("Arm chat bot %r up — mention it in #%s", botname, channel)
    aiko.process.run()


if __name__ == "__main__":
    main()
