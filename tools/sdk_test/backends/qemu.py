# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tier B carrier: QEMU.

This backend runs a test on this machine instead of on a board, and it needs no
Lab Agent -- there is no hardware to allocate, so there is no testbed to lock and
nothing to release.

It is the carrier that makes mock-based unit tests possible. ``SOC_QEMU``
puts the boot in ``rtos/rtthread/bsp/sifli/drivers_qemu/`` and, like the pc
simulator, makes ``drivers/SConscript`` skip the whole ``drivers/`` tree. No HAL
implementation is linked, so a CMock-generated mock is the *only* definition of
the symbol it replaces -- no duplicate symbol, no weak-symbol ordering, no ROM
table to compete with.

The console is the emulated UART0, wired to this process's stdio by ``-serial
stdio``. The harnesses read it exactly the way they read a board's serial port --
through a cursor over an accumulating buffer -- and drive the device by writing
to the same channel, so a `unity` or `pytest` case runs here unchanged.
"""

from __future__ import annotations

import codecs
import os
import shutil
import subprocess
import threading
import time
from typing import List
from typing import Optional
from typing import Tuple

from ..errors import BackendError
from ..model import TestInstance
from .base import Backend

#: How much to take from the emulator's stdout per read.
_READ_SIZE = 4096

#: The machine and core the BSP is written against (MPS2-AN505, Cortex-M33).
DEFAULT_MACHINE = "mps2-an505"
DEFAULT_CPU = "cortex-m33"

#: The ELF the build produces, relative to the build directory.
ELF_NAME = "main.elf"

#: Environment override for the emulator, for machines where it is not on PATH.
ENV_QEMU_EXE = "SIFLI_QEMU_EXE"

#: Where a QEMU install usually lands on Windows when it is not on PATH.
_QEMU_CANDIDATES = (
    r"C:\Program Files\qemu\qemu-system-arm.exe",
    r"C:\Program Files (x86)\qemu\qemu-system-arm.exe",
)

_QEMU_NAMES = ("qemu-system-arm", "qemu-system-arm.exe")


def find_qemu(explicit: Optional[str] = None) -> str:
    """Locate ``qemu-system-arm``, or explain how to point at it."""
    candidate = explicit or os.environ.get(ENV_QEMU_EXE)
    if candidate:
        if os.path.isfile(candidate):
            return candidate
        found = shutil.which(candidate)
        if found:
            return found
        raise BackendError(
            f'QEMU executable not found: "{candidate}" '
            f"(from {ENV_QEMU_EXE} or the backend argument)"
        )

    for name in _QEMU_NAMES:
        found = shutil.which(name)
        if found:
            return found

    for path in _QEMU_CANDIDATES:
        if os.path.isfile(path):
            return path

    raise BackendError(
        "Cannot find qemu-system-arm. Install QEMU, put it on PATH, or set "
        f"{ENV_QEMU_EXE} to the executable."
    )


def build_qemu_command(
    qemu_exe: str,
    elf: str,
    machine: str = DEFAULT_MACHINE,
    cpu: str = DEFAULT_CPU,
    extra: Optional[List[str]] = None,
) -> List[str]:
    """The QEMU invocation the BSP's console requires.

    The console is the MPS2's emulated UART0, bound to this process with
    ``-serial stdio``. ``-display`` and ``-monitor`` are turned off so that stdio
    carries the console and nothing else -- otherwise QEMU's multiplexer starts
    consuming input that belongs to the guest.

    Semihosting is deliberately absent. The BSP used to use it (and the bring-up
    scripts still do, for output); but its ``SYS_READC`` blocks the vCPU until a
    character arrives, so it cannot back an interactive console. See the comment
    at the top of ``drivers_qemu/uart_console.c``.
    """
    command = [
        qemu_exe,
        "-machine",
        machine,
        "-cpu",
        cpu,
        "-display",
        "none",
        "-monitor",
        "none",
        "-serial",
        "stdio",
        "-kernel",
        elf,
    ]
    command += [str(item) for item in (extra or [])]
    return command


class QemuConsole:
    """A cursor-based console over a QEMU process's stdio.

    Same discipline as the Lab Agent's serial console -- an accumulating buffer
    and a monotonic cursor, never line-by-line -- because the same host code has
    to read it. Keeping the cursor here means a caller that falls behind can
    still ask for what it missed.
    """

    def __init__(
        self,
        process: subprocess.Popen,
        max_buffer: int = 4 * 1024 * 1024,
        write_gap: float = 0.02,
    ) -> None:
        self._process = process
        self._max_buffer = max_buffer
        #: Pause between characters on write; see :meth:`write`. 20 ms is what
        #: was measured to be reliable -- 3 ms still dropped characters, which
        #: showed up as a shell command that echoed correctly and then never
        #: executed, because the terminating newline was the character lost.
        self._write_gap = write_gap
        #: Where :meth:`read` has consumed up to. Advanced by reads, so a caller
        #: looping over :meth:`read` sees each chunk once -- the same contract as
        #: the Lab Agent's RemoteConsole.
        self._read_cursor = 0
        self._condition = threading.Condition()
        self._buffer = ""
        self._dropped = 0
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._error: Optional[BaseException] = None

    # -- lifecycle -----------------------------------------------------------

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._pump, name="qemu-console", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        thread, self._thread = self._thread, None
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        if thread is not None:
            thread.join(timeout=3.0)

    def _pump(self) -> None:
        stream = self._process.stdout
        if stream is None:  # pragma: no cover - we always pass PIPE
            return
        # The pipe is unbuffered, so reads return raw bytes. Decoding each read
        # on its own would corrupt a multi-byte character split across a read
        # boundary, so an incremental decoder carries the partial sequence over.
        decoder = codecs.getincrementaldecoder("utf-8")("replace")
        while not self._stop.is_set():
            try:
                chunk = stream.read(_READ_SIZE)
            except (OSError, ValueError) as exc:
                # ValueError is what a closed pipe raises; both mean the process
                # is gone, which is a normal end for a short-lived test binary.
                with self._condition:
                    self._error = exc
                    self._condition.notify_all()
                return
            if not chunk:
                # EOF: the emulator exited. Readers see the end of the stream
                # rather than blocking until their own timeout.
                self._stop.set()
                with self._condition:
                    self._condition.notify_all()
                return

            text = decoder.decode(chunk)
            if not text:
                continue
            with self._condition:
                self._buffer += text
                excess = len(self._buffer) - self._max_buffer
                if excess > 0:
                    self._buffer = self._buffer[excess:]
                    self._dropped += excess
                self._condition.notify_all()

    # -- cursor arithmetic (call with the lock held) -------------------------

    @property
    def _end(self) -> int:
        return self._dropped + len(self._buffer)

    @property
    def cursor(self) -> int:
        with self._condition:
            return self._end

    @property
    def transcript(self) -> str:
        with self._condition:
            return self._buffer

    @property
    def dropped(self) -> int:
        """Bytes trimmed off the head of the buffer once it hit ``max_buffer``.

        Public because it is the only way a reader can tell a complete
        transcript from a truncated one, and the artefact that carries the
        transcript needs to say which it is.
        """
        with self._condition:
            return self._dropped

    @property
    def running(self) -> bool:
        return self._process.poll() is None

    @property
    def returncode(self) -> Optional[int]:
        return self._process.poll()

    # -- io ------------------------------------------------------------------

    def write(self, text: str) -> None:
        """Send ``text`` to the guest's console, one character at a time.

        Pacing is empirical, not theoretical: writing a whole line at once was
        observed to lose characters, and one at a time was not. What is *not*
        established is why -- the emulated UART's receive depth was never
        measured, and no interrupt is delivered on this machine even with the
        console's RX interrupt enabled and its NVIC line unmasked (verified:
        routing every vector to a probe and enabling all 32 lines produced no
        device IRQ). So this is a workaround whose cause is still open, recorded
        as such rather than dressed up as a datasheet fact.

        A few milliseconds per character is imperceptible for a shell command
        and is the difference between a working console and one that appears to
        drop keystrokes.
        """
        stream = self._process.stdin
        if stream is None:
            raise BackendError("The QEMU process has no stdin pipe")
        try:
            for character in text:
                stream.write(character.encode("utf-8"))
                stream.flush()
                time.sleep(self._write_gap)
        except (OSError, ValueError) as exc:
            raise BackendError(f"Writing to QEMU failed: {exc}") from exc

    def read_from(self, cursor: int, timeout: float) -> Tuple[str, int]:
        """Wait up to ``timeout`` for output after ``cursor``."""
        cursor = int(cursor)
        deadline = time.monotonic() + max(0.0, float(timeout))
        with self._condition:
            while True:
                if self._error is not None:
                    raise BackendError(f"QEMU console read failed: {self._error}")

                if cursor < self._dropped:
                    cursor = self._dropped
                if self._end > cursor:
                    return self._buffer[cursor - self._dropped :], self._end

                if self._stop.is_set():
                    return "", self._end

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return "", self._end
                self._condition.wait(remaining)

    def read(self, timeout: float = 1.0) -> str:
        """Return output not yet returned by a previous call; ``""`` if none."""
        data, self._read_cursor = self.read_from(self._read_cursor, timeout)
        return data


def wait_until_ready(
    console: QemuConsole,
    settle: float = 0.3,
    limit: float = 8.0,
) -> bool:
    """Wait until the guest has booted and gone quiet.

    A session must not be handed to a harness before the target can accept
    input. The harness sends its command as soon as it gets the console, but the
    guest takes a while to reach the point where its UART is enabled and being
    read -- and anything sent before that is simply gone. The symptom is a
    perfectly quiet failure: the command is dropped, the device sits at its
    prompt, and the harness reports "no Unity summary line found" after waiting
    out its whole timeout.

    Readiness is inferred from behaviour rather than from a device-specific
    banner: output has been seen, and then stopped arriving for ``settle``
    seconds. Nothing is consumed -- the caller's own cursor is untouched, so a
    console harness still sees the whole boot log from the first byte.

    Returns whether the device was seen to be ready; a timeout is not fatal
    (a test that fails is more useful than a run that hangs), so the caller
    proceeds either way.
    """
    deadline = time.monotonic() + limit
    cursor = 0
    quiet_since: Optional[float] = None

    while time.monotonic() < deadline:
        data, cursor = console.read_from(cursor, 0.1)
        if data:
            quiet_since = None
        elif quiet_since is None:
            quiet_since = time.monotonic()
        elif time.monotonic() - quiet_since >= settle:
            return True
    return False


class QemuSession:
    """A running emulator held for the duration of one test."""

    def __init__(
        self, process: subprocess.Popen, console: QemuConsole, platform: str, elf: str
    ) -> None:
        self._process = process
        self.console = console
        self.platform = platform
        self.elf = elf
        #: There is no hardware, so there is no testbed to name.
        self.testbed_id: Optional[str] = None
        #: Booting is the emulator's own job, so no operator is ever asked.
        self.manual_reset = False
        #: There is no agent here to have a revision.
        self.agent_revision = ""
        self._closed = False

    @property
    def transcript(self) -> str:
        """Everything the emulator's console has said, retained window aside.

        Read through to the console rather than snapshotted, so it is valid at
        any point before :meth:`close` -- which is where the runner reads it,
        since closing stops the pump thread and takes the buffer with it.
        """
        return self.console.transcript

    @property
    def dropped(self) -> int:
        """Bytes the console's buffer cap trimmed off the head; see QemuConsole."""
        return self.console.dropped

    def close(self) -> None:
        """Stop the emulator. Safe to call twice."""
        if self._closed:
            return
        self._closed = True

        self.console.stop()
        if self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
                try:
                    self._process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    pass
        for stream in (self._process.stdin, self._process.stdout):
            try:
                if stream is not None:
                    stream.close()
            except OSError:
                pass


class QemuBackend(Backend):
    """Runs tests under QEMU on this machine."""

    name = "qemu"
    requires_lab_agent = False

    def __init__(
        self,
        qemu_exe: Optional[str] = None,
        machine: str = DEFAULT_MACHINE,
        cpu: str = DEFAULT_CPU,
        extra_args: Optional[List[str]] = None,
    ) -> None:
        self.qemu_exe = qemu_exe
        self.machine = machine
        self.cpu = cpu
        self.extra_args = list(extra_args or [])

    def open(
        self,
        instance: TestInstance,
        platform: str,
        build_dir: Optional[str] = None,
        holder: Optional[str] = None,
        # Accepted and ignored: the emulator boots itself, so there is never a
        # boot for an operator to confirm.
        will_confirm_boot: bool = False,
    ) -> QemuSession:
        if not build_dir:
            raise BackendError(
                f'"{instance.id}": the qemu backend needs a build directory'
            )

        # Absolute, because the process is started with cwd=build_dir: a
        # relative kernel path would be resolved against that directory and not
        # against the caller's, which silently loads nothing.
        elf = os.path.abspath(os.path.join(build_dir, ELF_NAME))
        if not os.path.isfile(elf):
            raise BackendError(
                f'No {ELF_NAME} in "{build_dir}". Build it with '
                f"`scons --board={platform}`, or drop --no-build."
            )

        command = build_qemu_command(
            find_qemu(self.qemu_exe),
            elf,
            machine=self.machine,
            cpu=self.cpu,
            extra=self.extra_args,
        )

        try:
            process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                # Unbuffered, so output reaches the reader as it is produced
                # rather than in whatever block size the pipe happens to use.
                bufsize=0,
                cwd=build_dir,
            )
        except FileNotFoundError as exc:
            raise BackendError(f"Cannot start QEMU: {exc}") from exc

        console = QemuConsole(process)
        console.start()
        wait_until_ready(console)
        return QemuSession(process, console, platform, elf)
