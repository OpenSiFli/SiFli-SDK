# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

"""Hardware test for the efuse example's two msh commands."""
import pytest

PROMPT = r"msh [^>\n]*>"

STATE = {}

def _require(key):
    if not STATE.get(key):
        pytest.skip(f"the {key} step did not run")

def test_shell_is_alive(dut):
    dut.send_line("")
    dut.expect(PROMPT, timeout=30)
    STATE["shell"] = True

def test_efuse_write_prints_usage(dut):
    """No arguments must print the usage block, not program the OTP area."""
    _require("shell")
    dut.send_line("efuse_write")
    dut.expect(r"wrong arguments", timeout=30)
    dut.expect(r"argument list:", timeout=10)
    dut.expect(r"pos: bit position", timeout=10)
    dut.expect(r"len: bit length", timeout=10)
    dut.expect(r"data: data in hex", timeout=10)
    dut.expect(PROMPT, timeout=10)
    STATE["write_usage"] = True

def test_efuse_read_prints_usage(dut):
    """No arguments must print "wrong arguments" (src/main.c:75-80)."""
    _require("write_usage")
    dut.send_line("efuse_read")
    dut.expect(r"wrong arguments", timeout=30)
    dut.expect(PROMPT, timeout=10)
    STATE["read_usage"] = True

def test_efuse_read_returns(dut):
    """A read of the first efuse word must return to the shell.

    At this log level the example prints neither the dump nor "read fail:", so the
    only observable is that the command does not hang or crash. This is weaker than
    the other steps and cannot prove the word was read.
    """
    _require("read_usage")
    dut.send_line("efuse_read 0 8")
    dut.expect(PROMPT, timeout=30)
    STATE["read"] = True
