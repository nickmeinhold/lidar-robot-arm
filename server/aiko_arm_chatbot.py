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

    def __init__(self, context, botname: str, ws_url: str, channel: str):
        self.current_channel = channel  # read by ChatBotImpl's discovery hook
        context.call_init(self, "ChatBot", context)
        self.botname = botname
        self.engine = ArmCommandEngine(ws_url)
        log.info("ArmChatBot %s watching #%s, arm at %s",
                 botname, channel, ws_url)

    def process_message(self, payload_in, **kwargs):
        fields = _decode_message(payload_in)
        if not fields:
            return
        username = str(fields.get("username") or "")
        if username == self.botname:
            return  # our own reply echoing back — never self-respond
        text = str(fields.get("message") or "").strip()
        if not text.startswith(self.botname):
            return  # ordinary chat, not addressed to the arm
        rest = text[len(self.botname):].strip()
        if not rest:
            reply = self.engine.help()
        else:
            command, *args = rest.split()
            reply = self.engine.execute(command, args)
        self.print(f"{username or '?'}: {rest!r} -> {reply!r}")
        if self.chat_server:
            self.chat_server.send_message(
                self.botname, [self.current_channel], reply)


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
    botname = args[0] if args else DEFAULT_BOTNAME

    os.environ.setdefault("ARM_WS_URL", url)
    init_args = aiko.actor_args(_ACTOR_BOT, protocol=_PROTOCOL_BOT,
                                tags=["ec=true"])
    init_args["botname"] = botname
    init_args["ws_url"] = url
    init_args["channel"] = channel
    aiko.compose_instance(ArmChatBot, init_args)
    log.info("Arm chat bot %r up — mention it in #%s", botname, channel)
    aiko.process.run()


if __name__ == "__main__":
    main()
