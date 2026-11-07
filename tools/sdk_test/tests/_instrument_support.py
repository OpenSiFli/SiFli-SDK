# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Fake console for the instrument protocol tests.

A command-driven fake rather than a fixed byte script: the console sends a
line, this class appends the canned answer for that line, and ``read`` hands
back everything not yet read. That matches how the real console behaves -- one
read may carry a response plus events for the next call -- without a board.
"""

from __future__ import annotations

from typing import Dict
from typing import List
from typing import Union


def lines(*entries: str) -> str:
    """Join protocol lines with the newline the console expects."""
    return "".join(f"{entry}\n" for entry in entries)


def info_line(caps: str = "ble", proto: str = "1", **extra: object) -> str:
    fields = {"proto": proto, "fw": "test", "caps": caps}
    fields.update({key: value for key, value in extra.items()})
    body = " ".join(f"{key}={value}" for key, value in fields.items())
    return f"@info {body}\n"


class FakeLineConsole:
    """A console whose replies are keyed by a substring of the sent line."""

    def __init__(
        self,
        replies: Dict[str, Union[str, List[str]]] | None = None,
        initial: str = "",
    ) -> None:
        self.replies = dict(replies or {})
        self.sent: List[str] = []
        self._out = initial
        self._position = 0

    def feed(self, text: str) -> None:
        self._out += text

    def send_line(self, line: str) -> None:
        self.sent.append(line)
        for trigger, reply in self.replies.items():
            if trigger not in line:
                continue
            if isinstance(reply, list):
                # A list is consumed one entry per send, so a test can make the
                # first answer and the second silence.
                reply = reply.pop(0) if reply else ""
            self._out += reply

    def read(self, timeout: float = 1.0) -> str:
        chunk = self._out[self._position:]
        self._position = len(self._out)
        return chunk
