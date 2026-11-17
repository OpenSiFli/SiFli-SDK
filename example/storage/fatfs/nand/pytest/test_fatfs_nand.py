# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""FatFs on NAND (D-HARA) -- the procedure from the README, checked step by step.

What the README asks for:
    1) mount at boot (covered by fatfs_nand.mount with the console harness)
    2) a bare `fs_test`: 1024 KB of data with a 1 KB buffer, one progress line
       per 100 KB
    3) `ls` / `mkdir test1` / `cd test1` / `mkdir test2` / `ls` / `pwd`

The README sample also prints `Write Ops/sec` and `Read Ops/sec`; the current
firmware does not, it prints `Write Speed` and `Read Speed` only, so only what
the device really prints is asserted here.
"""

import pytest

PROMPT = r"msh [^>\n]*>"
FS_TEST = "fs_test"
TEST_FILE = "/test_file.txt"
# 1024 KB with a 1 KB buffer, the defaults of a bare fs_test
TEST_FILE_BYTES = 1024 * 1024

STATE = {}


def _require(key: str) -> None:
    """Skip instead of failing when an earlier step did not get there."""
    if not STATE.get(key):
        pytest.skip(f"requires {key!r} from an earlier step")


def test_shell_is_alive(dut):
    dut.send_line("")
    dut.expect(PROMPT, timeout=30)
    STATE["shell"] = True


def test_fs_test_writes_and_reads_1024_kb(dut):
    _require("shell")

    # A run interrupted half way leaves the file behind; fs_test then fails with
    # "Failed to create test file", which reads like a broken file system rather
    # than a dirty one. rm is silent when the file is absent.
    dut.send_line("rm " + TEST_FILE)
    dut.expect(PROMPT, timeout=30)

    dut.send_line(FS_TEST)

    dut.expect(r"Creating test file with 1024 KB data using 1K buffer\.\.\.", timeout=60)
    dut.expect(r"Written 100 KB\.\.\.", timeout=120)
    dut.expect(r"Written 1024 KB\.\.\.", timeout=180)
    dut.expect(r"Write test completed", timeout=60)
    dut.expect(r"Write Speed: \d+ bytes/sec \(\d+\.\d+ KB/s\)", timeout=60)
    dut.expect(r"Reading test file\.\.\.", timeout=60)
    dut.expect(r"Read 100 KB\.\.\.", timeout=120)
    dut.expect(r"Read 1024 KB\.\.\.", timeout=180)
    dut.expect(r"Read test completed", timeout=60)
    dut.expect(r"Read Speed: \d+ bytes/sec \(\d+\.\d+ KB/s\)", timeout=60)
    dut.expect(PROMPT, timeout=30)

    STATE["fs_test"] = True


def test_the_file_is_listed_with_its_size(dut):
    _require("fs_test")

    dut.send_line("ls")
    # ls prints the bare name (no leading /) with the size right aligned.
    dut.expect(r"test_file\.txt\s+" + str(TEST_FILE_BYTES), timeout=30)
    # Consume the prompt this command printed: the next test's first expect()
    # would otherwise match it and skip validating its own command.
    dut.expect(PROMPT, timeout=30)
    STATE["listed"] = True


def test_readme_directory_walk(dut):
    """README step 3: mkdir test1 -> cd test1 -> mkdir test2 -> ls -> pwd."""
    _require("listed")

    dut.send_line("mkdir test1")
    dut.expect(PROMPT, timeout=30)

    dut.send_line("cd test1")
    dut.expect(r"msh /test1>", timeout=30)

    dut.send_line("mkdir test2")
    dut.expect(PROMPT, timeout=30)

    dut.send_line("ls")
    dut.expect(r"test2\s+<DIR>", timeout=30)

    dut.send_line("pwd")
    dut.expect(r"/test1\r?\n", timeout=30)
    dut.expect(PROMPT, timeout=30)

    STATE["walked"] = True


def test_removing_the_file_makes_it_unreadable(dut):
    _require("walked")

    dut.send_line("cd /")
    dut.expect(PROMPT, timeout=30)

    # Drop the README directories first; a failure here would not change the
    # assertion below, which is about the file.
    dut.send_line("deldir /test1/test2")
    dut.expect(PROMPT, timeout=30)
    dut.send_line("deldir /test1")
    dut.expect(PROMPT, timeout=30)

    dut.send_line("rm " + TEST_FILE)
    dut.expect(PROMPT, timeout=30)

    dut.send_line("cat " + TEST_FILE)
    dut.expect(r"Open /test_file\.txt failed", timeout=30)
    dut.expect(PROMPT, timeout=30)
