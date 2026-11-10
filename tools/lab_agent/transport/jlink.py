# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""J-Link transport: out-of-band reset and direct flash programming.

Reset has to be out-of-band -- the serial port's DTR is not a reset line on this
hardware -- so it goes through the debug probe.

Flashing reuses the ``download.jlink`` script the *build* already generated in
``build_<board>/`` rather than recomputing image addresses here. Those addresses
come from the partition table and the packaging step; duplicating that logic in
the test framework would be a second source of truth for where images live. Only
the probe preamble is replaced, so that a lab machine with several J-Link probes
can select one by serial number -- something the generated ``download.bat``
cannot express.
"""

from __future__ import annotations

import os
import subprocess
import tempfile
from typing import List
from typing import Optional

from sdk_test.firmware import load_lines_from_download_script

from ..errors import TransportError


def _preamble(speed: int) -> List[str]:
    """Probe preamble shared by every script this transport writes."""
    return ["si SWD", f"speed {speed}", "connect"]


class JLinkTransport:
    """Drives one J-Link probe against one board."""

    def __init__(
        self,
        device: str,
        jlink_exe: str,
        interface: str = "SWD",
        speed: int = 4000,
        serial: Optional[str] = None,
        timeout: float = 60.0,
    ) -> None:
        if not device:
            raise TransportError("A J-Link device name is required (e.g. SF32LB52X)")
        # A relative path is resolved against the process CWD, but a memory
        # read runs the probe from a temporary directory. Pin an existing path
        # now; a bare command name is left alone so PATH lookup still works.
        if jlink_exe and os.path.isfile(jlink_exe):
            jlink_exe = os.path.abspath(jlink_exe)
        self.device = device
        self.jlink_exe = jlink_exe
        self.interface = interface
        self.speed = speed
        self.serial = str(serial) if serial else None
        self.timeout = timeout

    # -- script plumbing -----------------------------------------------------

    def _command(self, script_path: str) -> List[str]:
        command = [self.jlink_exe, "-device", self.device]
        if self.serial:
            command += ["-SelectEmuBySN", self.serial]
        command += [
            "-if",
            self.interface,
            "-speed",
            str(self.speed),
            "-autoconnect",
            "1",
            "-ExitOnError",
            "1",
            "-CommandFile",
            script_path,
        ]
        return command

    def _run_script(
        self, lines: List[str], what: str, cwd: Optional[str] = None
    ) -> str:
        """Write ``lines`` to a temporary script, run it, return its output.

        ``cwd`` is where the probe is run from, which matters as soon as the
        script names a file relatively: the script itself lands in the system
        temp directory, so a relative path in it resolves against the working
        directory and not against anything this transport chose. See
        :meth:`flash`.
        """
        handle = tempfile.NamedTemporaryFile(
            "w", suffix=".jlink", delete=False, encoding="ascii", newline="\n"
        )
        try:
            handle.write("\n".join(lines) + "\n")
            handle.close()
            command = self._command(handle.name)
            try:
                completed = subprocess.run(
                    command,
                    capture_output=True,
                    text=True,
                    timeout=self.timeout,
                    cwd=cwd,
                    check=False,
                )
            except FileNotFoundError as exc:
                raise TransportError(
                    f'J-Link executable not found: "{self.jlink_exe}"'
                ) from exc
            except subprocess.TimeoutExpired as exc:
                raise TransportError(
                    f"J-Link {what} timed out after {self.timeout:.0f}s"
                ) from exc
        finally:
            try:
                os.unlink(handle.name)
            except OSError:
                pass

        output = (completed.stdout or "") + (completed.stderr or "")
        if completed.returncode != 0:
            raise TransportError(
                f"J-Link {what} failed (exit {completed.returncode}):\n{output.strip()}"
            )
        return output

    # -- operations ----------------------------------------------------------

    def reset(self, extra_script: Optional[str] = None) -> str:
        """Reset the board and leave it running.

        ``extra_script`` is an optional ``.jlink`` file run first, for boards
        that need something done before the reset can take -- the SDK ships
        ``tools/segger/halt_all_cpu_and_disable_all_wdt_*.jlink`` for exactly
        that, since a running watchdog would otherwise reset the board out from
        under the test.
        """
        if extra_script:
            self.run_script_file(extra_script)

        # `r` resets and halts, `g` lets it run: the test needs a booting board,
        # not a halted one.
        return self._run_script(_preamble(self.speed) + ["r", "g", "exit"], "reset")

    def run_script_file(self, path: str) -> str:
        """Run an existing ``.jlink`` command file verbatim."""
        if not os.path.isfile(path):
            raise TransportError(f'J-Link script not found: "{path}"')
        try:
            with open(path, "r", encoding="utf-8", errors="replace") as handle:
                lines = [line.rstrip("\n") for line in handle if line.strip()]
        except OSError as exc:
            raise TransportError(f"Cannot read {path}: {exc}") from exc
        return self._run_script(lines, f"script {os.path.basename(path)}")

    def flash(self, build_dir: str) -> str:
        """Program every image named in ``<build_dir>/download.jlink``.

        Run from ``build_dir``, because that is what the paths in the script are
        relative to and nothing else here is. The build's own launcher does the
        same thing the hard way -- ``download.bat`` resolves its own directory
        into ``WORK_PATH`` and ``cd``-s there before starting the probe.
        """
        script = os.path.join(build_dir, "download.jlink")
        if not os.path.isfile(script):
            raise TransportError(
                f'No download.jlink in "{build_dir}": the build produced none, or '
                "it was not packed with the firmware"
            )

        loads = load_lines_from_download_script(script)
        if not loads:
            raise TransportError(
                f'"{script}" contains no loadbin/loadfile lines; nothing to program'
            )

        # `h` halts before programming; the trailing reset makes the new image
        # actually run, which is what the test then observes on the console.
        lines = _preamble(self.speed) + ["h"] + loads + ["r", "g", "exit"]
        return self._run_script(lines, "flash", cwd=build_dir)

    def read_memory(self, address: int, length: int) -> bytes:
        """Read ``length`` bytes of target memory and return them verbatim.

        The CPU is halted around the read and resumed afterwards: a probe can
        often read RAM while the core runs, but "often" is not a contract, and
        a torn read of the thing under observation is worse than a short pause.

        ``savebin`` writes to a relative file name and the probe runs from the
        directory that file lands in -- the same mechanism the crash-dump
        reader uses, and the reason ``cwd`` is passed to :meth:`_run_script`.
        """
        if not isinstance(address, int) or isinstance(address, bool) or address < 0:
            raise TransportError(
                f"memory address must be a non-negative integer, got {address!r}"
            )
        if not isinstance(length, int) or isinstance(length, bool) or length <= 0:
            raise TransportError(
                f"memory length must be a positive integer, got {length!r}"
            )

        with tempfile.TemporaryDirectory(prefix="jlink_read_") as workdir:
            raw_path = os.path.join(workdir, "memory.bin")
            lines = _preamble(self.speed) + [
                "h",
                f"savebin memory.bin 0x{address:x} 0x{length:x}",
                "g",
                "exit",
            ]
            try:
                self._run_script(lines, "memory read", cwd=workdir)
            except TransportError:
                # The script halts the core first. If the dump fails (or the
                # probe times out) the `g` after it never runs, and leaving the
                # board halted would turn one failed read into a dead console
                # for every later step. Resume on a best-effort basis, then
                # report the original failure.
                self._resume_after_failed_read()
                raise
            try:
                with open(raw_path, "rb") as handle:
                    data = handle.read()
            except OSError as exc:
                raise TransportError(
                    f"the probe did not produce a memory dump: {exc}"
                ) from exc

        if len(data) != length:
            raise TransportError(
                f"the probe returned {len(data)} of {length} requested bytes "
                f"from 0x{address:08x}"
            )
        return data

    def _resume_after_failed_read(self) -> None:
        """Best-effort ``g`` after a failed read, so the core is not left halted."""
        try:
            self._run_script(
                _preamble(self.speed) + ["g", "exit"], "resume after failed read"
            )
        except TransportError:
            pass
