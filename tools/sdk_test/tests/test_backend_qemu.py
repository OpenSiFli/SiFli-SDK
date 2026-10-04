# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the QEMU execution backend.

The console is exercised against *real* child processes rather than a mock, so
the parts that actually broke during bring-up -- byte-vs-str from an unbuffered
pipe, multi-byte characters split across reads, EOF handling -- are covered by
the same code paths that run in production.

The end-to-end path (build, boot under QEMU, verdict) is verified by running
``sdk.py test run -T example/qemu``; it needs QEMU installed and a build, so it
is not repeated here.
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
import unittest

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from sdk_test.backends.qemu import DEFAULT_CPU
from sdk_test.backends.qemu import DEFAULT_MACHINE
from sdk_test.backends.qemu import ELF_NAME
from sdk_test.backends.qemu import ENV_QEMU_EXE
from sdk_test.backends.qemu import QemuBackend
from sdk_test.backends.qemu import QemuConsole
from sdk_test.backends.qemu import QemuSession
from sdk_test.backends.qemu import build_qemu_command
from sdk_test.backends.qemu import find_qemu
from sdk_test.errors import BackendError
from sdk_test.model import TestInstance


def spawn(script: str) -> subprocess.Popen:
    """Run a Python snippet as a stand-in for the emulator process."""
    return subprocess.Popen(
        [sys.executable, "-c", script],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=0,
    )


def collect(console: QemuConsole, timeout: float = 5.0) -> str:
    """Read until the stream ends or the budget runs out."""
    text = ""
    deadline = time.monotonic() + timeout
    cursor = 0
    while time.monotonic() < deadline:
        chunk, cursor = console.read_from(cursor, 0.1)
        text += chunk
        if not console.running and not chunk:
            break
    return text


def reap(process: subprocess.Popen) -> None:
    """Kill a child if it is alive and close its pipes.

    Without this the tests leak processes and file handles, which shows up as
    ResourceWarnings and, on a busy machine, as children outliving the run.
    """
    if process.poll() is None:
        process.kill()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:  # pragma: no cover - kill is decisive
            pass
    for stream in (process.stdin, process.stdout):
        try:
            if stream is not None:
                stream.close()
        except OSError:  # pragma: no cover - already closed
            pass


def make_instance() -> TestInstance:
    return TestInstance(
        id="qemu/case", testsuite_root="/s", testcase_dir="/s/qemu", tests_key="case"
    )


class TestFindQemu(SdkTestCase):
    def test_an_explicit_missing_path_names_the_variable(self):
        with self.assertRaises(BackendError) as caught:
            find_qemu(self.root + "/no-such-qemu")
        self.assertIn("not found", str(caught.exception))

    def test_an_explicit_existing_path_wins(self):
        path = self.write("tools/qemu-system-arm.exe", "")
        self.assertEqual(find_qemu(path), path)

    def test_the_environment_variable_is_honoured(self):
        path = self.write("tools/fake-qemu", "")
        previous = os.environ.get(ENV_QEMU_EXE)
        os.environ[ENV_QEMU_EXE] = path
        self.addCleanup(
            lambda: os.environ.pop(ENV_QEMU_EXE, None)
            if previous is None
            else os.environ.__setitem__(ENV_QEMU_EXE, previous)
        )
        self.assertEqual(find_qemu(), path)

    def test_a_bad_environment_value_is_reported(self):
        previous = os.environ.get(ENV_QEMU_EXE)
        os.environ[ENV_QEMU_EXE] = self.root + "/absent"
        self.addCleanup(
            lambda: os.environ.pop(ENV_QEMU_EXE, None)
            if previous is None
            else os.environ.__setitem__(ENV_QEMU_EXE, previous)
        )
        with self.assertRaises(BackendError):
            find_qemu()


class TestCommand(SdkTestCase):
    def test_the_console_is_the_uart_on_stdio(self):
        command = build_qemu_command("qemu-system-arm", "/b/main.elf")
        index = command.index("-serial")
        self.assertEqual(command[index + 1], "stdio")

    def test_the_display_and_monitor_are_off_so_stdio_is_not_shared(self):
        # With either left on stdio, QEMU's multiplexer consumes input meant for
        # the guest, and the console looks like it silently ignores keystrokes.
        command = build_qemu_command("qemu-system-arm", "/b/main.elf")
        self.assertEqual(command[command.index("-display") + 1], "none")
        self.assertEqual(command[command.index("-monitor") + 1], "none")

    def test_semihosting_is_not_used(self):
        # Its SYS_READC blocks the vCPU, so it cannot back an interactive
        # console. The BSP's console is the emulated UART instead.
        command = build_qemu_command("qemu-system-arm", "/b/main.elf")
        self.assertNotIn("-semihosting", command)
        self.assertNotIn("-semihosting-config", command)

    def test_the_machine_and_core_match_the_bsp(self):
        command = build_qemu_command("qemu-system-arm", "/b/main.elf")
        self.assertEqual(command[command.index("-machine") + 1], DEFAULT_MACHINE)
        self.assertEqual(command[command.index("-cpu") + 1], DEFAULT_CPU)

    def test_the_elf_is_passed_as_the_kernel(self):
        command = build_qemu_command("qemu-system-arm", "/b/main.elf")
        self.assertEqual(command[command.index("-kernel") + 1], "/b/main.elf")

    def test_extra_arguments_are_appended(self):
        command = build_qemu_command("q", "/e", extra=["-d", "in_asm"])
        self.assertEqual(command[-2:], ["-d", "in_asm"])


class TestConsoleOverARealProcess(SdkTestCase):
    def make_console(self, script: str):
        process = spawn(script)
        console = QemuConsole(process)
        console.start()

        def teardown():
            console.stop()
            reap(process)

        self.addCleanup(teardown)
        return process, console

    def test_a_transcript_is_reassembled_and_the_cursor_is_absolute(self):
        _process, console = self.make_console(
            "import sys; sys.stdout.write('hello '); sys.stdout.write('world'); sys.stdout.flush()"
        )
        text = collect(console)
        self.assertEqual(text, "hello world")
        self.assertEqual(console.cursor, len("hello world"))
        self.assertEqual(console.transcript, "hello world")

    def test_output_is_bytes_off_an_unbuffered_pipe_and_must_decode(self):
        # The pipe is opened with bufsize=0, so reads come back as bytes. This
        # is the exact defect that made the first QEMU run fail.
        _process, console = self.make_console(
            "import sys; sys.stdout.write('plain ascii'); sys.stdout.flush()"
        )
        self.assertIsInstance(console.transcript, str)
        self.assertEqual(collect(console), "plain ascii")

    def test_a_multibyte_character_split_across_reads_decodes_intact(self):
        # Written as two separate bytes with a pause between, so the reader
        # almost certainly sees them in different reads. Decoding each read on
        # its own would yield two replacement characters.
        _process, console = self.make_console(
            "import sys, time\n"
            "sys.stdout.buffer.write(b'\\xc3')\n"
            "sys.stdout.buffer.flush()\n"
            "time.sleep(0.3)\n"
            "sys.stdout.buffer.write(b'\\xa9')\n"
            "sys.stdout.buffer.flush()\n"
        )
        text = collect(console)
        self.assertEqual(text, "\u00e9")
        self.assertNotIn("\ufffd", text)

    def test_reads_are_incremental_and_never_repeat(self):
        _process, console = self.make_console(
            "import sys; sys.stdout.write('abcdef'); sys.stdout.flush()"
        )
        first, cursor = console.read_from(0, 2.0)
        self.assertTrue(first)
        second, cursor2 = console.read_from(cursor, 0.5)
        self.assertEqual(first + second, "abcdef")
        self.assertEqual(second, "")
        self.assertEqual(cursor2, cursor)

    def test_eof_ends_the_stream_instead_of_waiting_out_the_timeout(self):
        _process, console = self.make_console(
            "import sys; sys.stdout.write('bye'); sys.stdout.flush()"
        )
        collect(console)
        deadline = time.monotonic() + 5.0
        while console.running and time.monotonic() < deadline:
            time.sleep(0.05)

        start = time.monotonic()
        data, _ = console.read_from(console.cursor, 10.0)
        self.assertEqual(data, "")
        # A dead process must not make every later read block for its timeout.
        self.assertLess(time.monotonic() - start, 3.0)

    def test_writes_reach_the_process_stdin(self):
        _process, console = self.make_console(
            "import sys\n"
            "line = sys.stdin.readline()\n"
            "sys.stdout.write('got:' + line)\n"
            "sys.stdout.flush()\n"
        )
        console.write("ping\n")
        self.assertIn("got:ping", collect(console))

    def test_running_and_returncode_track_the_process(self):
        process, console = self.make_console("pass")
        collect(console)
        deadline = time.monotonic() + 5.0
        while console.running and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertFalse(console.running)
        self.assertEqual(console.returncode, 0)

    def test_a_read_with_nothing_pending_returns_promptly(self):
        _process, console = self.make_console("import time; time.sleep(5)")
        start = time.monotonic()
        data, _ = console.read_from(0, 0.0)
        self.assertEqual(data, "")
        self.assertLess(time.monotonic() - start, 1.0)


class TestSessionAndBackend(SdkTestCase):
    def test_close_terminates_the_process_and_is_idempotent(self):
        process = spawn("import time; time.sleep(60)")
        console = QemuConsole(process)
        console.start()
        session = QemuSession(process, console, "qemu_cortex_m33", "/b/main.elf")

        session.close()
        self.assertIsNotNone(process.poll(), "the emulator should be stopped")
        session.close()  # must not raise

    def test_a_session_has_no_testbed_because_there_is_no_hardware(self):
        process = spawn("pass")
        console = QemuConsole(process)
        session = QemuSession(process, console, "qemu_cortex_m33", "/b/main.elf")
        self.addCleanup(reap, process)
        self.addCleanup(session.close)
        self.assertIsNone(session.testbed_id)

    def test_the_backend_needs_no_lab_agent(self):
        self.assertFalse(QemuBackend.requires_lab_agent)

    def test_opening_without_a_build_directory_is_refused(self):
        with self.assertRaises(BackendError) as caught:
            QemuBackend().open(make_instance(), "qemu_cortex_m33", build_dir=None)
        self.assertIn("build directory", str(caught.exception))

    def test_a_missing_elf_names_the_build_command(self):
        build_dir = self.mkdir("build_qemu_cortex_m33_hcpu")
        with self.assertRaises(BackendError) as caught:
            QemuBackend().open(make_instance(), "qemu_cortex_m33", build_dir=build_dir)
        message = str(caught.exception)
        self.assertIn(ELF_NAME, message)
        self.assertIn("scons --board=qemu_cortex_m33", message)

    def test_a_missing_interpreter_is_reported_before_launch(self):
        build_dir = self.mkdir("build")
        self.write("build/main.elf", "not really an elf")
        with self.assertRaises(BackendError) as caught:
            QemuBackend(qemu_exe="definitely-not-qemu").open(
                make_instance(), "qemu_cortex_m33", build_dir=build_dir
            )
        self.assertIn("not found", str(caught.exception))


if __name__ == "__main__":
    unittest.main()
