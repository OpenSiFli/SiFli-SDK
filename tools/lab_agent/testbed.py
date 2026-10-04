# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Testbed lifecycle: exclusive occupation, preparation, and console access.

Two ideas from the design are enforced here rather than trusted to callers:

**Exclusive occupation.** A testbed is the smallest allocatable unit and may be
held by one test program at a time. The lock lives on the runtime object, so a
second allocation fails loudly instead of two tests interleaving on one board.

**Preparation is not optional.** Reset and flash are the runner's job and happen
behind :meth:`TestBedRuntime.prepare`; a test case cannot skip them, and cannot
reach the raw transport to do them differently.

Console reads are *cursor based*: :meth:`TestBedRuntime.console_read` takes the
offset the caller has already consumed and returns the text after it plus the
new offset. Nothing is streamed over HTTP, and a reader that falls behind does
not lose data that is still inside the retention window.
"""

from __future__ import annotations

import os
import re
import threading
import time
from dataclasses import dataclass
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

from sdk_test.fixtures import explain_no_match
from sdk_test.fixtures import parse_fixture_ref
from sdk_test.fixtures import select_testbeds
from sdk_test.model import TestBed
from sdk_test.model import TestInstance

from .config import RunnerSettings
from .config import find_jlink
from .errors import AllocationError
from .errors import LabAgentError
from .errors import TestbedError
from .errors import TransportError
from .fixture_drivers import driver_for
from .transport.jlink import JLinkTransport
from .transport.serial import SerialTransport
from .transport.sftool import SftoolTransport

_JLINK_DEVICE_RE = re.compile(
    r"^\s*JLINK_DEVICE\s*=\s*['\"]([^'\"]*)['\"]", re.MULTILINE
)


def board_jlink_device(platform: str, sdk_root: Optional[str] = None) -> Optional[str]:
    """Read ``JLINK_DEVICE`` out of a board's ``rtconfig.py``.

    Reusing the board description keeps the probe model in one place: a board
    that gains a new J-Link device name should not also require the lab config
    to be edited. Parsed with a regex rather than ``exec`` so that a config file
    can never execute code in the runner process.
    """
    root = sdk_root or os.environ.get("SIFLI_SDK_PATH")
    if not root:
        return None

    for core in ("hcpu", "acpu", "lcpu"):
        path = os.path.join(root, "customer", "boards", platform, core, "rtconfig.py")
        if not os.path.isfile(path):
            continue
        try:
            with open(path, "r", encoding="utf-8", errors="replace") as handle:
                match = _JLINK_DEVICE_RE.search(handle.read())
        except OSError:
            continue
        if match and match.group(1).strip():
            return match.group(1).strip()
    return None


class ConsoleStream:
    """A console with a monotonic cursor, fed by a background reader thread.

    The thread exists so that the device can keep talking while the server
    thread is busy answering another request -- a blocking read on the request
    thread would drop output, which for a boot log means a test that fails for
    no reason.
    """

    def __init__(
        self,
        transport: SerialTransport,
        max_buffer: int = 4 * 1024 * 1024,
        poll: float = 0.2,
    ) -> None:
        self._transport = transport
        self._max_buffer = max_buffer
        self._poll = poll
        self._condition = threading.Condition()
        self._buffer = ""
        #: Characters dropped off the front to respect ``max_buffer``. Cursors
        #: stay absolute, so they survive trimming.
        self._dropped = 0
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._error: Optional[BaseException] = None

    # -- lifecycle -----------------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self._thread is not None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._transport.open()
        self._stop.clear()
        self._error = None
        self._thread = threading.Thread(
            target=self._pump, name="console-pump", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        thread, self._thread = self._thread, None
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        if thread is not None:
            thread.join(timeout=3.0)
        self._transport.close()

    def _pump(self) -> None:
        while not self._stop.is_set():
            try:
                chunk = self._transport.read(self._poll)
            except BaseException as exc:  # noqa: BLE001 - surfaced to readers
                with self._condition:
                    self._error = exc
                    self._condition.notify_all()
                return
            if not chunk:
                continue
            with self._condition:
                self._buffer += chunk
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
        """Absolute offset just past the newest byte received."""
        with self._condition:
            return self._end

    @property
    def transcript(self) -> str:
        """The retained transcript, for logging a failed run."""
        with self._condition:
            return self._buffer

    # -- io ------------------------------------------------------------------

    def write(self, text: str) -> None:
        self._transport.write(text)

    def read_from(self, cursor: int, timeout: float) -> Tuple[str, int]:
        """Wait up to ``timeout`` for text after ``cursor``.

        Returns ``(text, new_cursor)``. ``text`` is empty when nothing new
        arrived in time, which is a normal outcome, not an error.
        """
        cursor = int(cursor)
        deadline = time.monotonic() + max(0.0, float(timeout))
        with self._condition:
            while True:
                if self._error is not None:
                    raise TransportError(f"Console read failed: {self._error}")

                if cursor < self._dropped:
                    # The caller's position has been trimmed away; the best we
                    # can do is resume at the oldest retained byte.
                    cursor = self._dropped
                if self._end > cursor:
                    return self._buffer[cursor - self._dropped :], self._end

                if self._stop.is_set():
                    # Shutting down. Report end-of-stream instead of waiting out
                    # the caller's timeout -- otherwise a stop during a long read
                    # blocks teardown for as long as the read was willing to wait.
                    return "", self._end

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return "", self._end
                self._condition.wait(remaining)


class TestBedRuntime:
    """One physical testbed: who holds it, and how to drive its hardware."""

    def __init__(
        self,
        testbed: TestBed,
        settings: Optional[RunnerSettings] = None,
        sdk_root: Optional[str] = None,
        console_factory: Optional[Callable[[TestBed], Any]] = None,
        jlink_factory: Optional[Callable[[TestBed, RunnerSettings], JLinkTransport]] = None,
    ) -> None:
        self.testbed = testbed
        self.settings = settings or RunnerSettings()
        self.sdk_root = sdk_root
        self._lock = threading.Lock()
        self._holder: Optional[str] = None
        self._console: Optional[ConsoleStream] = None
        self._jlink: Optional[JLinkTransport] = None
        self._active_fixtures: List[Any] = []
        # Seams so the agent can be exercised without hardware: a fake console
        # transport or a stub probe lets the HTTP surface be tested for real.
        self._console_factory = console_factory or (
            lambda tb: SerialTransport(port=tb.console.port, baud=tb.console.baud)
        )
        self._jlink_factory = jlink_factory

    # -- allocation ----------------------------------------------------------

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._holder is not None

    @property
    def holder(self) -> Optional[str]:
        with self._lock:
            return self._holder

    def allocate(self, holder: Optional[str] = None) -> None:
        """Claim the testbed. Raises if someone else already holds it."""
        with self._lock:
            if self._holder is not None:
                raise AllocationError(
                    f'testbed "{self.testbed.id}" is already held by '
                    f'"{self._holder}"'
                )
            self._holder = holder or "anonymous"

    def release(self, holder: Optional[str] = None) -> None:
        """Drop the claim and return the hardware to a known-idle state."""
        with self._lock:
            if self._holder is None:
                return
            if holder is not None and holder != self._holder:
                raise AllocationError(
                    f'testbed "{self.testbed.id}" is held by "{self._holder}", '
                    f'not by "{holder}"'
                )
            self._holder = None

        # Teardown happens outside the lock: it can block on hardware, and a
        # second release from another thread must not be able to deadlock it.
        self._teardown()

    def _teardown(self) -> None:
        self.console_close()
        for connection, driver in reversed(self._active_fixtures):
            try:
                driver.teardown(self.testbed, connection)
            except Exception:
                # Teardown is best-effort: one broken fixture must not prevent
                # the others from being released.
                pass
        self._active_fixtures = []

    # -- preparation ---------------------------------------------------------

    def _jlink_options(self) -> Dict[str, Any]:
        options = self.testbed.runner_option("jlink") or {}
        if not isinstance(options, dict):
            raise TestbedError(
                f'testbed "{self.testbed.id}": "jlink" must be a mapping'
            )
        return options

    def jlink(self) -> JLinkTransport:
        """Build (once) the probe transport for this testbed."""
        with self._lock:
            if self._jlink is not None:
                return self._jlink

        if self._jlink_factory is not None:
            transport = self._jlink_factory(self.testbed, self.settings)
            with self._lock:
                self._jlink = transport
            return transport

        options = self._jlink_options()
        device = options.get("device") or board_jlink_device(
            self.testbed.platform, self.sdk_root
        )
        if not device:
            raise TestbedError(
                f'testbed "{self.testbed.id}": no J-Link device name. Set '
                f'`jlink.device` in the runner config, or make sure board '
                f'"{self.testbed.platform}" has a JLINK_DEVICE in its rtconfig.py.'
            )

        transport = JLinkTransport(
            device=str(device),
            jlink_exe=find_jlink(options.get("exe") or self.settings.jlink_exe),
            interface=str(options.get("interface", "SWD")),
            speed=int(options.get("speed", 4000)),
            serial=options.get("serial"),
            timeout=float(options.get("timeout", self.settings.jlink_timeout)),
        )
        with self._lock:
            self._jlink = transport
        return transport

    def _reset_options(self) -> Dict[str, Any]:
        options = self.testbed.runner_option("reset") or {}
        if not isinstance(options, dict):
            raise TestbedError(
                f'testbed "{self.testbed.id}": "reset" must be a mapping'
            )
        return options

    def reset_mode(self) -> str:
        """How this board is reset: ``jlink`` (the default) or ``manual``.

        ``manual`` is for a board with no probe attached. The framework cannot
        reset it, so a person presses the button, and the mode is reported to
        the caller (see :meth:`describe`) rather than assumed -- only this side
        knows which of the two actually happened.

        Validated here, lazily, and deliberately not in :meth:`describe`: a typo
        in the config must fail the run that needs a reset, not ``GET
        /testbeds``, whose caller would report it as an unreachable agent.
        """
        method = str(self._reset_options().get("method", "jlink")).lower()
        if method not in ("jlink", "manual"):
            raise TestbedError(
                f'testbed "{self.testbed.id}": unknown reset method "{method}" '
                '(use "jlink" or "manual")'
            )
        return method

    def reset(self) -> str:
        """Reset the board out-of-band and leave it running.

        In ``manual`` mode this returns without touching the probe: the board
        has none, and an operator is expected to press its reset button. That is
        a deliberate no-op rather than a silent failure -- the mode is already
        known to the caller from the allocation, which is what lets it prompt
        for the button instead of waiting on a reset that will never come.
        """
        if self.reset_mode() == "manual":
            return ""
        options = self._jlink_options()
        return self.jlink().reset(extra_script=options.get("wdt_script"))

    def _flash_options(self) -> Dict[str, Any]:
        options = self.testbed.runner_option("flash") or {}
        if not isinstance(options, dict):
            raise TestbedError(
                f'testbed "{self.testbed.id}": "flash" must be a mapping'
            )
        return options

    def flash_method(self) -> str:
        """How images reach this board: ``jlink`` (the default) or ``sftool``."""
        method = str(self._flash_options().get("method", "jlink")).lower()
        if method not in ("jlink", "sftool"):
            raise TestbedError(
                f'testbed "{self.testbed.id}": unknown flash method "{method}" '
                '(use "jlink" or "sftool")'
            )
        return method

    def needs_jlink(self) -> bool:
        """Whether operating this testbed at all requires a debug probe.

        True when either half of the preamble goes through the probe. Flashing
        and resetting are chosen independently -- a board downloaded over UART
        but reset by probe is a real configuration -- so a manual reset does not
        by itself mean the probe can be absent.
        """
        return self.flash_method() == "jlink" or self.reset_mode() == "jlink"

    def flash(self, build_dir: str) -> str:
        """Program the images built in ``build_dir``.

        The transport is chosen per testbed, because the same lab can hold a
        board with a probe and one wired only for UART download.
        """
        if not build_dir:
            raise TestbedError("flash needs a build directory")

        method = self.flash_method()

        if method == "jlink":
            return self.jlink().flash(build_dir)

        if method == "sftool":
            if self._console is not None:
                # The download port *is* the console port, so the two cannot be
                # open at once. Refuse rather than silently producing a corrupt
                # download or a port conflict halfway through.
                raise TestbedError(
                    f'testbed "{self.testbed.id}": the console is open on '
                    f"{self.testbed.console.port}, which is also the download "
                    "port; close the console before flashing over UART"
                )
            if self.testbed.console is None:
                raise TestbedError(
                    f'testbed "{self.testbed.id}": sftool flashing needs the '
                    "console port to be configured (it is the download port)"
                )
            return SftoolTransport(
                port=self.testbed.console.port,
                sftool=self._flash_options().get("sftool"),
            ).flash(build_dir)

        raise AssertionError(f"unreachable flash method {method!r}")

    def setup_fixtures(self) -> None:
        """Prepare every fixture this testbed declares."""
        for connection in self.testbed.connections:
            driver = driver_for(connection)
            try:
                driver.setup(self.testbed, connection)
            except LabAgentError:
                raise
            except Exception as exc:
                raise TestbedError(
                    f'testbed "{self.testbed.id}": setting up fixture '
                    f'"{connection.fixture.spec}" failed: {exc}'
                ) from exc
            self._active_fixtures.append((connection, driver))

    def prepare(self, build_dir: Optional[str] = None, reset: bool = True) -> None:
        """Run the whole non-optional preamble: fixtures, flash, reset.

        A test case never calls this; the runner does it between allocation and
        the test itself, which is what makes it impossible to skip.
        """
        self.setup_fixtures()
        if build_dir:
            self.flash(build_dir)
        if reset:
            self.reset()

    # -- console -------------------------------------------------------------

    def console_open(self) -> None:
        if self._console is not None:
            return
        if self.testbed.console is None:
            raise TestbedError(
                f'testbed "{self.testbed.id}" has no console port configured'
            )

        stream = ConsoleStream(
            self._console_factory(self.testbed),
            max_buffer=self.settings.max_buffer,
            poll=self.settings.console_poll,
        )
        stream.start()
        self._console = stream

    def console_close(self) -> None:
        stream, self._console = self._console, None
        if stream is not None:
            stream.stop()

    def _require_console(self) -> ConsoleStream:
        if self._console is None:
            raise TestbedError(
                f'testbed "{self.testbed.id}": console is not open'
            )
        return self._console

    def console_read(self, cursor: int, timeout: float) -> Tuple[str, int]:
        return self._require_console().read_from(cursor, timeout)

    def console_write(self, data: str) -> None:
        self._require_console().write(data)

    def console_send_line(self, line: str) -> None:
        self._require_console().write(line + "\r\n")

    @property
    def console_cursor(self) -> int:
        return self._require_console().cursor

    @property
    def console_transcript(self) -> str:
        return self._require_console().transcript

    # -- reporting -----------------------------------------------------------

    def describe(self) -> Dict[str, Any]:
        testbed = self.testbed
        # Read leniently on purpose: a malformed "reset" block is
        # reset_mode()'s to report at the moment a reset is asked for. Raising
        # here instead would fail GET /testbeds, and the SDK side reports that
        # as an unreachable agent -- a misleading symptom for a typo.
        reset_options = testbed.runner_option("reset")
        reset = (
            str(reset_options.get("method", "jlink")).lower()
            if isinstance(reset_options, dict)
            else "jlink"
        )
        return {
            "id": testbed.id,
            "platform": testbed.platform,
            "runner": testbed.runner,
            "fixtures": [fixture.spec for fixture in testbed.fixtures],
            "muxes": [mux.id for mux in testbed.muxes],
            "console": (
                {"port": testbed.console.port, "baud": testbed.console.baud}
                if testbed.console
                else None
            ),
            "busy": self.busy,
            "holder": self.holder,
            "console_open": self._console is not None,
            # Surfaced so a control plane can tell three kinds of lab apart: one
            # that can power-cycle a wedged board, one that can only reset it
            # through the probe, and one where a person presses the button.
            "relay_available": False,
            "reset": reset,
        }


@dataclass
class AllocationRequest:
    """What a client asks for when it does not name a testbed."""

    platform: Optional[str] = None
    fixtures: List[str] = None  # type: ignore[assignment]
    testbed_id: Optional[str] = None
    holder: Optional[str] = None

    def __post_init__(self) -> None:
        if self.fixtures is None:
            self.fixtures = []


class LabManager:
    """Owns every testbed on this machine and hands them out one at a time."""

    def __init__(
        self,
        config,
        sdk_root: Optional[str] = None,
        console_factory: Optional[Callable[[TestBed], Any]] = None,
        jlink_factory: Optional[Callable[[TestBed, RunnerSettings], JLinkTransport]] = None,
    ) -> None:
        self.config = config
        self.sdk_root = sdk_root
        self._runtimes: Dict[str, TestBedRuntime] = {
            testbed.id: TestBedRuntime(
                testbed, config.settings, sdk_root, console_factory, jlink_factory
            )
            for testbed in config.testbeds
        }
        self._lock = threading.Lock()

    # -- queries -------------------------------------------------------------

    def ids(self) -> List[str]:
        return list(self._runtimes)

    def get(self, testbed_id: str) -> TestBedRuntime:
        runtime = self._runtimes.get(testbed_id)
        if runtime is None:
            raise TestbedError(
                f'Unknown testbed "{testbed_id}" '
                f"(this runner has: {', '.join(self._runtimes) or 'none'})"
            )
        return runtime

    def list(self) -> List[Dict[str, Any]]:
        return [runtime.describe() for runtime in self._runtimes.values()]

    # -- allocation ----------------------------------------------------------

    def _candidates(
        self, request: AllocationRequest
    ) -> Tuple[List[TestBed], TestInstance]:
        instance = TestInstance(
            id="<allocate>",
            testsuite_root="",
            testcase_dir="",
            tests_key="<allocate>",
            platform_allow=[request.platform] if request.platform else [],
            fixtures=[parse_fixture_ref(spec) for spec in request.fixtures],
        )
        testbeds = [runtime.testbed for runtime in self._runtimes.values()]
        return select_testbeds(instance, testbeds), instance

    def allocate(self, request: AllocationRequest) -> TestBedRuntime:
        """Claim a testbed matching ``request``.

        Matching reuses the SDK side's own selection rules, so a testbed the
        CLI would pick is the same one the agent offers -- the two cannot drift.
        """
        with self._lock:
            if request.testbed_id:
                runtime = self.get(request.testbed_id)
                runtime.allocate(request.holder)
                return runtime

            candidates, instance = self._candidates(request)
            free = [testbed for testbed in candidates if not self._runtimes[testbed.id].busy]
            if not free:
                if not candidates:
                    all_testbeds = [runtime.testbed for runtime in self._runtimes.values()]
                    raise AllocationError(explain_no_match(instance, all_testbeds))
                busy = ", ".join(
                    f"{testbed.id} (held by {self._runtimes[testbed.id].holder})"
                    for testbed in candidates
                )
                raise AllocationError(
                    f"every testbed matching the request is busy: {busy}"
                )

            chosen = self._runtimes[free[0].id]
            chosen.allocate(request.holder)
            return chosen

    def release(self, testbed_id: str, holder: Optional[str] = None) -> None:
        self.get(testbed_id).release(holder)

    def shutdown(self) -> None:
        """Release every testbed. Used on server stop."""
        for runtime in self._runtimes.values():
            try:
                runtime.release()
            except Exception:
                pass


def allocate_request_from_mapping(data: Dict[str, Any]) -> AllocationRequest:
    """Build an :class:`AllocationRequest` from a decoded JSON body."""
    if not isinstance(data, dict):
        raise AllocationError("Request body must be a JSON object")

    def string_list(key: str) -> List[str]:
        value = data.get(key)
        if value is None:
            return []
        if isinstance(value, str):
            return [value]
        if isinstance(value, (list, tuple)):
            return [str(item) for item in value]
        raise AllocationError(f'"{key}" must be a string or a list of strings')

    if "capabilities" in data:
        raise AllocationError(
            '"capabilities" is no longer part of an allocation request. What a '
            "platform can do is decided before a testbed is asked for, from the "
            "board's features.yaml; a request carries `fixtures` only."
        )

    return AllocationRequest(
        platform=data.get("platform"),
        fixtures=string_list("fixtures"),
        testbed_id=data.get("testbed_id"),
        holder=data.get("holder"),
    )
