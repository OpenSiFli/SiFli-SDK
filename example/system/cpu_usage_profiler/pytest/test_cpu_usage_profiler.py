# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of the cpu_usage_profiler example.

The profiler starts a periodic timer at app init (CPU_USAGE_METRICS_PRINT_DIRECTLY),
so it prints without any shell command and this suite only reads.

A report is a block: a ``CPU Usage:`` title, the thread-table header containing
``run_time``, and one line per thread, including the idle thread ``tidle``. The
property this case asserts is that the first two blocks are each complete.

That property is awkward to express with console records: their matcher
searches forward, so a block that printed only its title would borrow the
header and idle line from the next block. Splitting the transcript on the
title and checking each block sidesteps that -- and reads like the property it
asserts.
"""

from __future__ import annotations

import time

PROMPT = r"msh [^>\n]*>"
TITLE = "CPU Usage:"

#: The period is 10 s (CPU_USAGE_METRICS_PRINT_PERIOD), and the first report
#: waits the same amount, so 40 s leaves room for a slow boot.
REPORT_WAIT = 40.0


def _read_until_two_titles(dut, timeout=REPORT_WAIT):
    """Read until two report titles have arrived, and return the transcript."""
    text = ""
    deadline = time.monotonic() + timeout
    while text.count(TITLE) < 2:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise AssertionError(
                f"expected two {TITLE!r} reports within {timeout:.0f}s.\n"
                f"--- device said ---\n{text or '<nothing>'}\n-------------------"
            )
        text += dut.read(min(0.5, remaining))

    # The second title has arrived, but its table lines follow right behind it.
    # Give them one poll interval to land before the blocks are split.
    text += dut.read(0.5)
    return text


def test_boot_reaches_the_shell(dut):
    """The RT-Thread banner appears and the shell prints its prompt."""
    dut.expect("- SiFli Corporation", timeout=30)
    dut.expect(PROMPT, timeout=20)


def test_first_two_reports_are_complete(dut):
    """Both of the first two reports carry their header and idle line.

    Splitting on the title makes the ownership explicit: parts[1] is the text
    between the first and second title, so its checks cannot be satisfied by
    the second block (nor parts[2]'s by a third).
    """
    # Start from a clean buffer so the blocks counted are the ones produced
    # after this test began. A report left over from boot would otherwise
    # become parts[1] and hide an incomplete first block.
    dut.drain()

    text = _read_until_two_titles(dut)
    parts = text.split(TITLE)

    for number, body in ((1, parts[1]), (2, parts[2])):
        assert "run_time" in body, f"report {number} has no table header:\n{body}"
        assert "tidle" in body, f"report {number} has no idle thread line:\n{body}"
