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
    """The minimal surface process_message touches, minus the aiko Actor.

    ``_submit`` runs jobs INLINE (the real class uses a worker queue), keeping
    fixtures synchronous; ``BUSY_DEPTH`` semantics get their own fixture via a
    rejecting ``_submit`` override.
    """

    botname = "@@armbot"
    chat_server = None
    wave_on_message = False
    _wave_cooldown_s = 8.0
    _last_greet = 0.0
    current_channel = "general"
    BUSY_REPLY = ArmChatBot.BUSY_REPLY
    LOCKED_REPLY = ArmChatBot.LOCKED_REPLY
    chat_help = staticmethod(ArmChatBot.chat_help)
    print = staticmethod(lambda *_: None)

    def __init__(self):
        self.engine = RecordingEngine()
        self.replies = []

    def _submit(self, job):
        job()
        return True

    def _reply(self, reply):
        self.replies.append(reply)


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
    assert bot.engine.calls == [("joint", ["wrist_pitch", "20"])]


def test_bare_mention_replies_help():
    bot = Bot()
    see(bot, "@@armbot")
    assert bot.engine.calls == [] and bot.replies == [ArmChatBot.chat_help()]


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
    assert bot.engine.calls == []
    assert bot.replies and "didn't catch that" in bot.replies[0]


# --- busy backpressure -------------------------------------------------------

def test_full_queue_replies_busy_and_runs_nothing():
    bot = Bot()
    bot._submit = lambda job: False  # queue at BUSY_DEPTH
    see(bot, "@@armbot wave")
    assert bot.engine.calls == []
    assert bot.replies == [ArmChatBot.BUSY_REPLY]


def test_greeter_skips_silently_when_queue_full():
    bot = Bot()
    bot.wave_on_message = True
    bot._submit = lambda job: False
    see(bot, "hello robot!")
    assert bot.engine.calls == [] and bot.replies == []


# --- crowd-observed UX (2026-08-10 meetup log) -------------------------------

def test_bare_joint_syntax_skips_llm():
    bot = Bot()
    see(bot, "@@armbot wrist_pitch 45")
    assert bot.engine.calls == [("joint", ["wrist_pitch", "45"])]


def test_bare_joint_alias_works():
    bot = Bot()
    see(bot, "@@armbot wrist 30")
    assert bot.engine.calls == [("joint", ["wrist_pitch", "30"])]


def test_locked_joint_gets_honest_reply():
    bot = Bot()
    see(bot, "@@armbot shoulder_pitch 100")
    assert bot.engine.calls == []
    assert bot.replies == [ArmChatBot.LOCKED_REPLY]


def test_typed_joint_shoulder_also_locked():
    bot = Bot()
    see(bot, "@@armbot joint shoulder_yaw -15")
    assert bot.engine.calls == []
    assert bot.replies == [ArmChatBot.LOCKED_REPLY]


def test_help_never_advertises_locked_joints():
    text = ArmChatBot.chat_help()
    assert "shoulder_yaw" not in text and "shoulder_pitch" not in text.replace(
        "shoulder & base are locked", "")


# --- routines stay inside the cage BY CONSTRUCTION ---------------------------

def test_every_routine_step_is_inside_the_cage():
    from server.aiko_arm_robot import ArmCommandEngine
    caps = {"wrist_pitch": 45.0, "wrist_roll": 45.0, "elbow_pitch": 15.0}
    for name, (_reply, steps) in ArmCommandEngine.ROUTINES.items():
        for step in steps:
            for key, value in step.items():
                if key in ("dwell", "grip"):
                    continue
                assert key in caps, f"{name}: joint {key} is not cage-allowed"
                assert abs(value) <= caps[key], f"{name}: {key}={value} over cap"


def test_every_routine_ends_at_neutral():
    from server.aiko_arm_robot import ArmCommandEngine
    for name, (_reply, steps) in ArmCommandEngine.ROUTINES.items():
        final = {}
        for step in steps:
            for key, value in step.items():
                if key not in ("dwell", "grip"):
                    final[key] = value
        assert all(v == 0 for v in final.values()), f"{name} ends off-neutral: {final}"


def test_routines_are_no_arg_commands():
    from server.aiko_arm_robot import ArmCommandEngine
    for name in ArmCommandEngine.ROUTINES:
        assert name in ArmCommandEngine.COMMANDS
        assert name in ArmCommandEngine._NO_ARG_COMMANDS
