# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for how the CLI decides what the runner is told about a manual boot.

The runner never reads stdin and never asks whether a board has a probe: it
knows only whether it was handed a way to ask an operator. That makes this
choice -- which callback, or none -- the whole of the feature, including whether
a harness that writes to the device is allowed on a board nothing can reset.

Checked here rather than through the runner because ``--no-confirm`` is
user-facing and easy to read backwards: it supplies *no* answer, which is what
gets a writing harness refused, rather than the answer "no".
"""

from __future__ import annotations

import builtins
import os
import sys
import unittest

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

_TOOLS_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, _TOOLS_DIR)

from sdk_py_actions.test_ext import _confirm_for  # noqa: E402
from sdk_py_actions.test_ext import _make_confirm  # noqa: E402


class _Stdin:
    """Stands in for stdin, which is a pipe in CI and a terminal at a desk."""

    def __init__(self, tty: bool) -> None:
        self._tty = tty

    def isatty(self) -> bool:
        return self._tty


class StdinTestCase(SdkTestCase):
    def setUp(self):
        super().setUp()
        self._real_stdin = sys.stdin
        self._real_input = builtins.input
        self.addCleanup(setattr, sys, "stdin", self._real_stdin)
        self.addCleanup(setattr, builtins, "input", self._real_input)

    def answer(self, text: str = "") -> None:
        """Make the next ``input()`` return ``text``."""
        builtins.input = lambda _prompt="": text

    def answer_with_eof(self) -> None:
        def raise_eof(_prompt=""):
            raise EOFError()

        builtins.input = raise_eof


class TestConfirmFor(StdinTestCase):
    """Two states: somebody to ask, or nobody.

    There is deliberately no third one that answers without asking. ``auto``
    reset says the same thing where it belongs -- in the lab's runner config --
    and it says it to the side that does the resetting.
    """

    def test_no_confirm_yields_no_answer(self):
        # Not "an answer of no": the difference is what keeps a writing harness
        # off a board nothing can reset.
        self.assertIsNone(_confirm_for(no_confirm=True, echo=lambda _m: None))

    def test_nothing_given_asks(self):
        sys.stdin = _Stdin(tty=True)
        self.answer("y")
        confirm = _confirm_for(no_confirm=False, echo=lambda _m: None)
        self.assertIsNotNone(confirm)
        self.assertTrue(confirm("press RESET"))


class TestMakeConfirm(StdinTestCase):
    """The default path: ask, when there is somewhere to ask."""

    def test_without_a_terminal_there_is_nobody_to_ask(self):
        sys.stdin = _Stdin(tty=False)
        # Handing back a prompt nobody can answer would block CI forever.
        self.assertIsNone(_make_confirm(lambda _m: None))

    def test_on_a_terminal_the_answer_decides(self):
        sys.stdin = _Stdin(tty=True)
        echo = []
        confirm = _make_confirm(echo.append)
        self.assertIsNotNone(confirm)

        self.answer("y")
        self.assertTrue(confirm("press RESET"))
        self.assertIn("press RESET", echo[0])

        self.answer("n")
        self.assertFalse(confirm("press RESET"))

        # Enter alone is a yes: the prompt's default is the useful one, and
        # requiring a keystroke for it would be friction on every run.
        self.answer("")
        self.assertTrue(confirm("press RESET"))

    def test_an_unanswerable_prompt_is_not_taken_as_yes(self):
        # Ctrl-D, a closed pipe: the operator never answered, so the boot is not
        # confirmed. Defaulting to yes here would run a writing harness on a
        # board nobody vouched for.
        sys.stdin = _Stdin(tty=True)
        self.answer_with_eof()

        confirm = _make_confirm(lambda _m: None)
        self.assertIsNotNone(confirm)
        self.assertFalse(confirm("press RESET"))


if __name__ == "__main__":
    unittest.main()
