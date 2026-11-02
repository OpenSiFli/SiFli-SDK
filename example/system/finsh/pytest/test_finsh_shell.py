# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of the FinSH example: drive its shell from Python.

The firmware under test registers two commands (``myhello`` and ``atcmd``) and
does nothing else; the shell thread owns the console. Exercising it therefore
means typing into that shell, which is exactly what a ``console`` harness
cannot do -- so this suite uses ``dut``, which sends a line and waits for the
reply.

Order matters. ``dut`` is session-scoped and its ``expect`` cursor only moves
forward, so the tests run in the order defined here: boot first, then one
command at a time.

Nothing asserts on the echoed command text: ``FINSH_ECHO_DISABLE_DEFAULT`` is
on for SF32LB52X/56X/57X, and a test that depends on echo would pass on 55X/58X
for the wrong reason.
"""

from __future__ import annotations

import re

#: The prompt is ``msh />`` when DFS keeps a working directory and ``msh >``
#: when it does not. Accept both rather than pinning one carrier's spelling.
PROMPT = r"msh [^>\n]*>"

#: ``re.escape`` because the message contains ``<``, ``|`` and ``>``.
USAGE = re.escape("Please input'atcmd <server|client>'")


def test_boot_reaches_the_shell(dut):
    """The RT-Thread banner appears and the shell prints its prompt.

    Both are printed before pytest attaches, but the runner opens the console
    first, so they are still in the retained buffer -- the same way
    hello_world's pytest case sees its boot output.
    """
    dut.expect("- SiFli Corporation", timeout=30)
    dut.expect(PROMPT, timeout=20)


def test_myhello(dut):
    """The no-argument command runs and prints its greeting."""
    dut.send_line("myhello")
    dut.expect(r"hello RT-Thread!", timeout=10)


def test_atcmd_requires_an_argument(dut):
    """With no argument, atcmd explains how it must be called."""
    dut.send_line("atcmd")
    dut.expect(USAGE, timeout=10)


def test_atcmd_rejects_an_unknown_mode(dut):
    """A mode that is neither server nor client takes the other error branch.

    This is a different path from the missing-argument case above: argc is 2
    here, so the comparison runs and falls through to the final else.
    """
    dut.send_line("atcmd serial")
    dut.expect(USAGE, timeout=10)


def test_atcmd_server(dut):
    """The server mode is recognised."""
    dut.send_line("atcmd server")
    dut.expect(r"AT server!", timeout=10)


def test_atcmd_client(dut):
    """The client mode is recognised."""
    dut.send_line("atcmd client")
    dut.expect(r"AT client!", timeout=10)


def test_unknown_command_is_reported(dut):
    """The shell itself answers an unregistered name.

    This proves the commands above ran through msh rather than through some
    other path: the command-not-found message is produced by the shell.
    """
    dut.send_line("finsh_no_such_command")
    dut.expect(r"finsh_no_such_command: command not found\.", timeout=10)
