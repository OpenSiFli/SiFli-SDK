# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""LittleFS on NOR -- the procedure from the README, checked step by step.

the "expected results" section in the README gives four steps:
    `ls` -> `mkdir test2` -> `ls` (test2 is listed) -> `pwd`

The littlefs example has no `fs_test`, so reading and writing a file uses the
msh commands `echo` (write, and it appends) and `cat` (read). Both are in the
README command list, and they add the one thing the four steps cannot show: that
what was written comes back.
"""

import pytest

PROMPT = r"msh [^>\n]*>"
DIR = "test2"
FILE = "/lfs_test.txt"
TEXT = "hello_lfs"

STATE = {}


def _require(key: str) -> None:
    """Skip instead of failing when an earlier step did not get there."""
    if not STATE.get(key):
        pytest.skip(f"requires {key!r} from an earlier step")


def test_shell_is_alive(dut):
    dut.send_line("")
    dut.expect(PROMPT, timeout=30)
    STATE["shell"] = True


def test_readme_directory_walk(dut):
    """The README four steps: ls -> mkdir test2 -> ls -> pwd."""
    _require("shell")

    dut.send_line("ls")
    dut.expect(PROMPT, timeout=30)

    dut.send_line("mkdir " + DIR)
    dut.expect(PROMPT, timeout=30)

    dut.send_line("ls")
    dut.expect(DIR + r"\s+<DIR>", timeout=30)
    dut.expect(PROMPT, timeout=30)

    dut.send_line("pwd")
    dut.expect(r"/\r?\n", timeout=30)
    dut.expect(PROMPT, timeout=30)

    STATE["walked"] = True


def test_echo_writes_a_file(dut):
    _require("shell")

    # echo appends, so remove the old file first and the size is predictable.
    dut.send_line("rm " + FILE)
    dut.expect(PROMPT, timeout=30)

    dut.send_line(f"echo {TEXT} {FILE}")
    dut.expect(PROMPT, timeout=30)

    dut.send_line("ls")
    dut.expect(r"lfs_test\.txt\s+" + str(len(TEXT)) + r"\b", timeout=30)
    # Consume the prompt this command printed, so the next test starts from a
    # cursor of its own.
    dut.expect(PROMPT, timeout=30)
    STATE["written"] = True


def test_cat_reads_it_back(dut):
    _require("written")

    dut.send_line("cat " + FILE)
    dut.expect(TEXT, timeout=30)
    dut.expect(PROMPT, timeout=30)
    STATE["read"] = True


def test_the_file_and_directory_survive_a_reboot(dut):
    """Power loss safety is what littlefs sells; content and directory survive."""
    _require("read")
    # The directory comes from test_readme_directory_walk. Without this key the
    # ls assertion below would fail for an unrelated reason.
    _require("walked")

    dut.send_line("reboot")
    dut.expect("SFBL", timeout=60)
    dut.expect(r"mount fs on flash to root success|make lfs fs on flash sucess, mount again", timeout=120)
    dut.expect(PROMPT, timeout=60)

    dut.send_line("cat " + FILE)
    dut.expect(TEXT, timeout=30)
    dut.expect(PROMPT, timeout=30)

    dut.send_line("ls")
    dut.expect(DIR + r"\s+<DIR>", timeout=30)
    dut.expect(PROMPT, timeout=30)


def test_removing_the_file_makes_it_unreadable(dut):
    _require("read")

    dut.send_line("rm " + FILE)
    dut.expect(PROMPT, timeout=30)

    dut.send_line("cat " + FILE)
    dut.expect(r"Open /lfs_test\.txt failed", timeout=30)
    dut.expect(PROMPT, timeout=30)

    dut.send_line("deldir /" + DIR)
    dut.expect(PROMPT, timeout=30)
