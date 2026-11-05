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
from sdk_test.errors import BindingError
from sdk_test.model import Connection
from sdk_test.model import ConsoleConfig
from sdk_test.model import TestBed
from sdk_test.model import TestInstance

from .config import RunnerSettings
from .config import find_jlink
from .errors import AllocationError
from .errors import LabAgentError
from .errors import TestbedError
from .errors import TransportError
from .fixture_drivers import driver_for
from .firmware import FirmwareStore
from .transport.jlink import JLinkTransport
from .transport.serial import SerialTransport
from .transport.sftool import SftoolTransport

from sdk_test.platforms import jlink_device
from sdk_test.version import PROTOCOL
from sdk_test.version import package_root
from sdk_test.version import revision


def _fixture_console_config(connection: Connection) -> ConsoleConfig:
    """The serial port an instrument fixture answers on, from its binding entry.

    An active fixture keeps its own settings in the connection entry, and this
    is the one setting the agent needs: where to open a console. Refusing here
    with the fixture's name beats a ``KeyError`` surfacing as a 500.
    """
    options = connection.options.get("console")
    if not isinstance(options, dict):
        raise TestbedError(
            f'fixture "{connection.fixture.spec}" has no console configured; '
            'add "console: {port: ..., baud: ...}" to its connection'
        )
    port = options.get("port")
    if not port:
        raise TestbedError(
            f'fixture "{connection.fixture.spec}" has no console port'
        )
    try:
        baud = int(options.get("baud", 1000000))
    except (TypeError, ValueError) as exc:
        raise TestbedError(
            f'fixture "{connection.fixture.spec}": console baud must be an integer'
        ) from exc
    return ConsoleConfig(port=str(port), baud=baud)


def _default_fixture_console(testbed: TestBed, connection: Connection) -> SerialTransport:
    """The real transport for an instrument console, built from its binding entry."""
    config = _fixture_console_config(connection)
    return SerialTransport(port=config.port, baud=config.baud)


def resolve_jlink_device(
    configured: Optional[str],
    from_package: Optional[str],
    platform: str,
    sdk_root: Optional[str] = None,
) -> Optional[str]:
    """Which device name to give the probe, first source first.

    ``download.jlink`` names images and addresses but not the device, and a
    probe is told which one it is attached to in order to pick a flash
    algorithm -- so the name comes from somewhere else, and there are three
    somewheres:

    1. ``jlink.device`` in the lab's own config. A person who wrote it meant it,
       and they are the one holding the probe.
    2. The firmware package, which carries the name the *build* read from the
       board file it was built against.
    3. This checkout's own board file, for a package that says nothing.

    The order is the point. Before the package travelled, a lab on another
    machine had only (1) and (3) -- and (3) is that machine's checkout, which may
    be a different revision than the one that produced the image. A stale board
    file there is not an error the run can see: the probe simply uses the wrong
    flash algorithm, and the flash succeeds.
    """
    return configured or from_package or jlink_device(platform, sdk_root)


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
        firmware_root: Optional[str] = None,
        fixture_console_factory: Optional[Callable[[TestBed, Any], Any]] = None,
    ) -> None:
        self.testbed = testbed
        self.settings = settings or RunnerSettings()
        self.sdk_root = sdk_root
        self._lock = threading.Lock()
        self._holder: Optional[str] = None
        self._console: Optional[ConsoleStream] = None
        self._jlink: Optional[JLinkTransport] = None
        self._active_fixtures: List[Any] = []
        #: Consoles of instruments attached to this testbed, keyed by the fixture
        #: name the test asked for. They are not testbeds: nothing is built or
        #: flashed for them, and they are only opened when the allocation
        #: requested the fixture.
        self._fixture_consoles: Dict[str, ConsoleStream] = {}
        #: The fixture specs this allocation asked for, so an instrument cannot
        #: be driven by a test that never declared it.
        self._requested_fixtures: List[str] = []
        #: True from the moment the claim is dropped until teardown finishes.
        #: The holder is cleared first so the board is visibly free, but its
        #: per-claim state (requested fixtures, firmware, probe) is being wiped
        #: -- a new claim in that window would have its state deleted by the old
        #: teardown, so allocation is refused until it settles.
        self._releasing = False
        #: Serializes opening and closing instrument consoles against each other
        #: and against teardown. The HTTP server answers on a thread per request,
        #: and ``transport.open()`` blocks: without this, an open that started
        #: before a release can finish after it and leave a serial port held on
        #: a testbed nobody owns.
        self._fixture_lock = threading.Lock()
        #: Firmware uploaded for this board. Keyed by the package's own digest,
        #: and told which platform it serves so it can refuse an image built for
        #: a different board -- a wrong-platform image flashes perfectly well.
        self.firmware = FirmwareStore(
            root=os.path.join(firmware_root, testbed.id) if firmware_root else None,
            platform=testbed.platform,
        )
        # Seams so the agent can be exercised without hardware: a fake console
        # transport or a stub probe lets the HTTP surface be tested for real.
        self._console_factory = console_factory or (
            lambda tb: SerialTransport(port=tb.console.port, baud=tb.console.baud)
        )
        self._fixture_console_factory = fixture_console_factory or _default_fixture_console
        self._jlink_factory = jlink_factory

    # -- allocation ----------------------------------------------------------

    @property
    def busy(self) -> bool:
        with self._lock:
            # A testbed being torn down is not available either: its per-claim
            # state is still being cleared, so selection must skip it rather
            # than pick it and fail the allocation.
            return self._holder is not None or self._releasing

    @property
    def holder(self) -> Optional[str]:
        with self._lock:
            return self._holder

    def describe_state(self) -> str:
        """One phrase for why this testbed is (un)available, for a message."""
        holder = self.holder
        if holder is not None:
            return f"held by {holder}"
        return "being released" if self._releasing else "free"

    def allocate(
        self, holder: Optional[str] = None, fixtures: Optional[List[str]] = None
    ) -> None:
        """Claim the testbed. Raises if someone else already holds it.

        ``fixtures`` is what the allocation asked for. It is remembered rather
        than checked here -- selection already matched it -- because it is the
        gate that keeps a test from driving an instrument it never declared.
        """
        with self._lock:
            if self._holder is not None:
                raise AllocationError(
                    f'testbed "{self.testbed.id}" is already held by '
                    f'"{self._holder}"'
                )
            if self._releasing:
                raise AllocationError(
                    f'testbed "{self.testbed.id}" is still being released; '
                    "please retry"
                )
            self._holder = holder or "anonymous"
            self._requested_fixtures = list(fixtures or [])

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
            self._releasing = True

        # Teardown happens outside the lock: it can block on hardware, and a
        # second release from another thread must not be able to deadlock it.
        # The releasing flag is what keeps a new claim from starting in the
        # window where the old claim's state is still being cleared.
        try:
            self._teardown()
        finally:
            with self._lock:
                self._releasing = False

    def _teardown(self) -> None:
        self.console_close()
        self.fixture_consoles_close()
        for connection, driver in reversed(self._active_fixtures):
            try:
                driver.teardown(self.testbed, connection)
            except Exception:
                # Teardown is best-effort: one broken fixture must not prevent
                # the others from being released.
                pass
        self._active_fixtures = []
        # Firmware goes with the claim that brought it: the next run uploads its
        # own, and keeping a finished run's images unpacked is how a lab PC
        # fills up. It also means "uploaded" can never be confused with
        # "uploaded by the run happening now".
        self.firmware.clear()
        # The next claim declares its own fixtures; keeping last run's would let
        # an instrument be opened by a test that never asked for it.
        self._requested_fixtures = []
        # And the probe goes with it, because part of how it is built comes from
        # the package: a device name is a property of the board the image was
        # built for, and the next run's package may say a different one.
        with self._lock:
            self._jlink = None

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
        device = resolve_jlink_device(
            configured=options.get("device"),
            from_package=self.firmware.in_use().get("jlink_device"),
            platform=self.testbed.platform,
            sdk_root=self.sdk_root,
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
        """How this board is reset: ``jlink`` (the default), ``auto`` or ``manual``.

        ``jlink`` drives the probe. The other two touch nothing, and differ in
        what they promise: ``auto`` says the boot happens regardless of us -- a
        UART download ends by running the new image, and opening the console can
        reset the board again -- while ``manual`` says nothing guarantees it and
        a person may have to press the button. Only the second needs an operator
        standing by, and only the second leaves a harness that writes to the
        device racing a boot that could wipe its command.

        The mode is reported to the caller (see :meth:`describe`) rather than
        assumed -- only this side knows which of the three actually happened.

        Validated here, lazily, and deliberately not in :meth:`describe`: a typo
        in the config must fail the run that needs a reset, not ``GET
        /testbeds``, whose caller would report it as an unreachable agent.
        """
        method = str(self._reset_options().get("method", "jlink")).lower()
        if method not in ("jlink", "auto", "manual"):
            raise TestbedError(
                f'testbed "{self.testbed.id}": unknown reset method "{method}" '
                '(use "jlink", "auto" or "manual")'
            )
        return method

    def reset(self) -> str:
        """Reset the board out-of-band and leave it running.

        Only ``jlink`` does anything here. The other two return without touching
        the probe, for opposite reasons: in ``manual`` mode the board has no
        probe and an operator is expected to press the reset button, while in
        ``auto`` mode there is nothing to do at all -- the boot that follows the
        download, or the console being opened, is the reset.

        Both are deliberate no-ops rather than silent failures: the mode is
        already known to the caller from the allocation, which is what lets it
        prompt for the button in the one case and stay quiet in the other.
        """
        if self.reset_mode() in ("auto", "manual"):
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

    # -- instrument consoles -------------------------------------------------
    #
    # A fixture with a console is an instrument the test drives -- a peer board,
    # in this first use. It is deliberately not a console on the DUT's stream:
    # a test must not be able to mistake the two, and the two are opened and
    # closed independently. Addressing a stream by fixture name also keeps the
    # test out of the binding: the port number never leaves this file.

    def fixture_console_open(self, spec: str) -> None:
        connection = self._instrument_connection(spec)
        # Keyed by the *connection's* spec, not by what the caller wrote: the
        # bare name and a pinned instance are two spellings of one port, and two
        # ConsoleStreams on one serial port split or drop bytes unpredictably.
        key = connection.fixture.spec
        with self._fixture_lock:
            # Checked under the lock so a release that already tore the consoles
            # down cannot be outlived by this open.
            #
            # It cannot tell *which* allocation this request belongs to: the
            # console routes carry no claim identity, so a request abandoned by
            # an earlier round can still open the stream of a newer one that
            # re-declared the same fixture. That is bounded and benign -- the
            # open is idempotent (the new round would open the same stream), it
            # only starts the agent's retention sooner, and the next release
            # closes it. Removing the window entirely needs the client to send
            # its claim, i.e. a protocol addition.
            if self._holder is None:
                raise TestbedError(
                    f'testbed "{self.testbed.id}" is not allocated; refusing to '
                    f'open the console of fixture "{key}"'
                )
            if key in self._fixture_consoles:
                return
            requested = [parse_fixture_ref(entry) for entry in self._requested_fixtures]
            if not any(need.accepts(connection.fixture) for need in requested):
                declared = ", ".join(entry for entry in self._requested_fixtures) or "none"
                raise TestbedError(
                    f'testbed "{self.testbed.id}": fixture "{connection.fixture.spec}" '
                    f"was not requested by this allocation (requested: {declared})"
                )

            # Validate the port here rather than only inside the default factory:
            # the factory is a test seam, and a binding mistake must be caught on
            # the real path even when the seam is replaced.
            _fixture_console_config(connection)

            # Held across start() on purpose: it opens a port and starts a
            # thread, and a release must not slip in between that and the
            # registration below.
            stream = ConsoleStream(
                self._fixture_console_factory(self.testbed, connection),
                max_buffer=self.settings.max_buffer,
                poll=self.settings.console_poll,
            )
            stream.start()
            self._fixture_consoles[key] = stream

    def fixture_console_close(self, spec: str) -> None:
        key = self._instrument_connection(spec).fixture.spec
        # ``stop()`` closes the port only after its reader thread has exited, so
        # the lock is held across it: a reopen that slipped in after the pop but
        # before the port was released would fail on Windows (port still busy).
        with self._fixture_lock:
            stream = self._fixture_consoles.pop(key, None)
            if stream is not None:
                stream.stop()

    def fixture_consoles_close(self) -> None:
        with self._fixture_lock:
            streams = list(self._fixture_consoles.values())
            self._fixture_consoles.clear()
            for stream in streams:
                stream.stop()

    def fixture_console_read(
        self, spec: str, cursor: int, timeout: float
    ) -> Tuple[str, int]:
        return self._require_fixture_console(spec).read_from(cursor, timeout)

    def fixture_console_write(self, spec: str, data: str) -> None:
        self._require_fixture_console(spec).write(data)

    def _require_fixture_console(self, spec: str) -> ConsoleStream:
        key = self._instrument_connection(spec).fixture.spec
        with self._fixture_lock:
            stream = self._fixture_consoles.get(key)
        if stream is None:
            raise TestbedError(
                f'testbed "{self.testbed.id}": the console of fixture "{key}" '
                "is not open"
            )
        return stream

    def _instrument_connection(self, spec: str) -> Connection:
        """Find the connection an instrument stream name refers to.

        The client may name the fixture (``ble_peer``) or pin one of several
        instances (``ble_peer:peer1``). A pinned name must match exactly -- the
        same rule as ``FixtureRef.accepts``, so asking for ``peer2`` can never
        reach ``peer1`` under another spelling. Only an unpinned name falls back
        to the testbed's single connection of that type.
        """
        wanted = None
        try:
            wanted = parse_fixture_ref(spec)
        except BindingError as exc:
            # A caller-supplied name that cannot be parsed is a bad request, not
            # an agent failure: LabAgentError maps to 400, where a BindingError
            # would fall through to a 500 and a server-side stack trace.
            raise LabAgentError(
                f'testbed "{self.testbed.id}": "{spec}" is not a fixture name: {exc}'
            ) from exc
        exact = [c for c in self.testbed.connections if c.fixture.spec == wanted.spec]
        if exact:
            return exact[0]
        offered = ", ".join(c.fixture.spec for c in self.testbed.connections) or "none"
        if wanted.param is not None:
            raise TestbedError(
                f'testbed "{self.testbed.id}" has no fixture "{wanted.spec}" '
                f"(it offers: {offered})"
            )
        by_name = [c for c in self.testbed.connections if c.fixture.name == wanted.name]
        if len(by_name) == 1:
            return by_name[0]
        if not by_name:
            raise TestbedError(
                f'testbed "{self.testbed.id}" has no fixture named "{wanted.name}" '
                f"(it offers: {offered})"
            )
        raise TestbedError(
            f'testbed "{self.testbed.id}" has several fixtures named '
            f'"{wanted.name}"; address one by its full spec '
            f"({', '.join(c.fixture.spec for c in by_name)})"
        )

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
            # Instruments are not testbeds, but a control plane still needs to
            # see which ones are talking: a stuck peer console looks exactly
            # like a quiet one until someone asks.
            "fixture_consoles": sorted(self._fixture_consoles),
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
        firmware_root: Optional[str] = None,
        fixture_console_factory: Optional[Callable[[TestBed, Any], Any]] = None,
    ) -> None:
        self.config = config
        self.sdk_root = sdk_root
        #: What this agent speaks and what it is built from. Both are reported on
        #: every allocation: the caller refuses on a protocol mismatch, and
        #: records the revision so a strange result can be explained later. See
        #: :mod:`sdk_test.version` for why only the protocol is a gate.
        self.protocol = PROTOCOL
        self.revision = revision(package_root())
        self._runtimes: Dict[str, TestBedRuntime] = {
            testbed.id: TestBedRuntime(
                testbed,
                config.settings,
                sdk_root,
                console_factory,
                jlink_factory,
                firmware_root,
                fixture_console_factory,
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
                runtime.allocate(request.holder, request.fixtures)
                return runtime

            candidates, instance = self._candidates(request)
            free = [testbed for testbed in candidates if not self._runtimes[testbed.id].busy]
            if not free:
                if not candidates:
                    all_testbeds = [runtime.testbed for runtime in self._runtimes.values()]
                    raise AllocationError(explain_no_match(instance, all_testbeds))
                busy = ", ".join(
                    f"{testbed.id} ({self._runtimes[testbed.id].describe_state()})"
                    for testbed in candidates
                )
                raise AllocationError(
                    f"every testbed matching the request is busy: {busy}"
                )

            chosen = self._runtimes[free[0].id]
            chosen.allocate(request.holder, request.fixtures)
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

    return AllocationRequest(
        platform=data.get("platform"),
        fixtures=string_list("fixtures"),
        testbed_id=data.get("testbed_id"),
        holder=data.get("holder"),
    )
