"""Mention-grammar fixtures for the arm chat bot (claude-tasks #2616).

The addressing grammar is a SAFETY surface: a single ``@armbot`` is what a
person-mention autocomplete produces, so it must never drive hardware. These
fixtures pin the arm bot to the same sigil grammar the bridge enforces
(aiko-chat-bridge robots.py ``^@@`` + exact name), so the two addressing paths
can't drift apart again. No bus, no hardware — ``process_message`` is exercised
on a stub with a recording engine.
"""
from __future__ import annotations

import json

import pytest

from server.aiko_arm_chatbot import ArmChatBot


class RecordingEngine:
    """Stands in for ArmCommandEngine — records dispatches, moves nothing."""

    COMMANDS = ("ready", "home", "open", "close", "gripper", "joint", "wave")

    def __init__(self):
        self.calls: list[tuple[str, list[str]]] = []
        self.help_calls = 0

    def execute(self, command, args):
        self.calls.append((command, list(args)))
        return f"ok:{command}"

    def help(self):
        self.help_calls += 1
        return "help-text"


class Bot:
    """The minimal surface process_message touches, minus the aiko Actor."""

    botname = "@@armbot"
    chat_server = None
    wave_on_message = False
    _wave_cooldown_s = 8.0
    _last_greet = 0.0
    current_channel = "general"
    print = staticmethod(lambda *_: None)

    def __init__(self):
        self.engine = RecordingEngine()


def see(bot: Bot, message: str, username: str = "nick") -> None:
    payload = json.dumps(
        {"username": username, "channel": "general", "message": message}
    )
    ArmChatBot.process_message(bot, payload)


# --- the safety line: single-@ is a person mention, never a command ---------

def test_single_at_is_ignored():
    bot = Bot()
    see(bot, "@armbot wave")  # autocomplete person-mention shape
    assert bot.engine.calls == [] and bot.engine.help_calls == 0


def test_single_at_case_variant_is_ignored():
    bot = Bot()
    see(bot, "@Armbot wave")
    assert bot.engine.calls == []


def test_bare_name_without_sigil_is_ignored():
    bot = Bot()
    see(bot, "armbot wave")
    assert bot.engine.calls == []


# --- exact-name match: no prefix collisions ---------------------------------

def test_different_robot_name_is_ignored():
    bot = Bot()
    see(bot, "@@arm wave")  # '@@arm' is a DIFFERENT robot, not a prefix of us
    assert bot.engine.calls == []


def test_name_superstring_is_ignored():
    bot = Bot()
    see(bot, "@@armbots wave")
    assert bot.engine.calls == []


# --- what must still work ---------------------------------------------------

def test_exact_mention_dispatches():
    bot = Bot()
    see(bot, "@@armbot wave")
    assert bot.engine.calls == [("wave", [])]


def test_capitalized_mention_and_command_dispatch():
    bot = Bot()
    see(bot, "@@ARMBOT Wave")  # phone auto-capitalize, both levels
    assert bot.engine.calls == [("Wave", [])]  # engine lowercases the command


def test_args_pass_through():
    bot = Bot()
    see(bot, "@@armbot joint wrist pitch 20")
    assert bot.engine.calls == [("joint", ["wrist", "pitch", "20"])]


def test_bare_mention_replies_help():
    bot = Bot()
    see(bot, "@@armbot")
    assert bot.engine.help_calls == 1 and bot.engine.calls == []


def test_own_echo_is_skipped():
    bot = Bot()
    see(bot, "@@armbot wave", username="@@armbot")
    assert bot.engine.calls == []


def test_unrelated_chat_is_ignored():
    bot = Bot()
    see(bot, "the arm bot is fun")
    assert bot.engine.calls == []


# --- greeter mode (meetup demo) ---------------------------------------------

def test_greeter_waves_on_ordinary_chat():
    bot = Bot()
    bot.wave_on_message = True
    see(bot, "hello everyone!")
    assert bot.engine.calls == [("wave", [])]


def test_greeter_cooldown_limits_wave_rate():
    bot = Bot()
    bot.wave_on_message = True
    see(bot, "first")
    see(bot, "second, right after")
    assert bot.engine.calls == [("wave", [])]  # second greet suppressed


def test_greeter_never_waves_at_own_reply():
    bot = Bot()
    bot.wave_on_message = True
    see(bot, "👋", username="@@armbot")
    assert bot.engine.calls == []


def test_greeter_commands_still_dispatch():
    bot = Bot()
    bot.wave_on_message = True
    see(bot, "@@armbot home")
    assert bot.engine.calls == [("home", [])]


# --- english mode (stubbed translator) --------------------------------------

def test_unknown_command_routes_to_translator(monkeypatch):
    import server.aiko_arm_chatbot as m
    monkeypatch.setattr(m, "translate_to_commands",
                        lambda text: ["joint wrist_roll 30", "open"])
    bot = Bot()
    bot._english = m.ArmChatBot._english.__get__(bot)
    see(bot, "@@armbot do a little dance")
    assert bot.engine.calls == [("joint", ["wrist_roll", "30"]), ("open", [])]


def test_empty_translation_replies_help_hint(monkeypatch):
    import server.aiko_arm_chatbot as m
    monkeypatch.setattr(m, "translate_to_commands", lambda text: [])
    bot = Bot()
    bot._english = m.ArmChatBot._english.__get__(bot)
    see(bot, "@@armbot what is the meaning of life")
    assert bot.engine.calls == [] and bot.engine.help_calls == 1
