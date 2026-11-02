# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of the ulog example: drive its shell and read its logs.

The example prints nothing until `ulog_test` runs, so a read-only console
harness cannot check it. This suite sends the command through `dut` and asserts
on the five lines ulog emits, then exercises the runtime level filter in both
directions.

Two properties of `dut` shape the code below:

* it is session-scoped and its `expect` cursor only moves forward, so every
  test drains what is left before sending its command -- otherwise identical
  log lines from an earlier test could satisfy this one's expectation;
* only a whole-window read can prove a line did *not* appear, which is the
  other half of a filter test. That is what ``_collect`` is for.
"""

from __future__ import annotations

import re
import time

#: The prompt is ``msh />`` with a DFS working directory and ``msh >`` without.
PROMPT = r"msh [^>\n]*>"

#: The body every LOG_X line in the example carries.
MESSAGE = "RT-Thread is an open source IoT operating system from China."

#: Declaration order in ulog_test(), which is also output order.
LEVELS = ("LOG_D", "LOG_I", "LOG_W", "LOG_E")


def _collect(dut, seconds):
    """Read the console for ``seconds`` and return everything it said."""
    text = ""
    deadline = time.monotonic() + seconds
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return text
        text += dut.read(min(0.5, remaining))


def test_boot_reaches_the_shell(dut):
    """The RT-Thread banner appears and the shell prints its prompt."""
    dut.expect("- SiFli Corporation", timeout=30)
    dut.expect(PROMPT, timeout=20)


def test_ulog_test_prints_every_level(dut):
    """LOG_D/I/W/E, LOG_RAW and the hexdump all reach the console."""
    dut.drain()
    # Set the filter to "everything" first: the example's syslog mode uses a
    # bitmask, so 255 is what enables all eight levels.
    dut.send_line("ulog_lvl 255")
    dut.send_line("ulog_test")

    for level in LEVELS:
        dut.expect(re.escape(f"{level}: {MESSAGE}"), timeout=10)

    # LOG_RAW bypasses the level filter and carries no prefix at all.
    dut.expect(re.escape(f"LOG_RAW: {MESSAGE}"), timeout=10)

    # ulog_hexdump prints 16 bytes per line; the buffer is filled with 0..127,
    # so the first line starts with 00..0F.
    dut.expect(r"buf_dump_test:.*00 01 02 03 04 05 06 07", timeout=10)


def test_ulog_filter_reports_the_level(dut):
    """ulog_filter prints the level just set."""
    dut.drain()
    dut.send_line("ulog_lvl 8")
    dut.send_line("ulog_filter")
    # With ULOG_USING_SYSLOG the level is a bitmask number, not a name.
    dut.expect(r"level\s+:\s*8\b", timeout=10)


def test_level_filter_hides_lower_levels(dut):
    """At level 8 only ERROR survives; RAW still passes, DEBUG/INFO/WARN do not."""
    dut.drain()
    dut.send_line("ulog_lvl 8")
    dut.send_line("ulog_test")

    text = _collect(dut, 2.0)

    assert "LOG_E: " + MESSAGE in text, text
    assert "LOG_RAW: " + MESSAGE in text, text
    for hidden in ("LOG_D: ", "LOG_I: ", "LOG_W: "):
        assert hidden + MESSAGE not in text, text


def test_level_filter_is_reversible(dut):
    """Raising the filter back to 255 brings DEBUG output back.

    Without this the filter test above could pass on a device that simply
    stopped printing.
    """
    dut.drain()
    dut.send_line("ulog_lvl 255")
    dut.send_line("ulog_test")
    dut.expect(re.escape(f"LOG_D: {MESSAGE}"), timeout=10)
