# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Driving a BLE peer instrument from a pytest suite.

The instrument is a second board, wired to the testbed and declared in the
binding as a fixture (``ble_peer``) with its own console. It is not a testbed:
nothing is built or flashed for it, and its firmware is fixed by the lab. This
module speaks the one firmware it assumes -- the central role of
``example/ble/central_and_peripheral``, whose ``diss`` shell command already
scans, connects, discovers services, reads, writes and subscribes.

A suite gets one from the ``ble_peer`` fixture and never parses ``diss`` output
itself: this is the single place the instrument's protocol lives, so every BLE
test case shares it and a firmware change is one file away.

Reading is deliberately *not* built on ``expect``'s cursor. The console object
here is driven with ``read`` / ``send_line`` / ``drain`` only, and this class
keeps its own buffer, because a helper that both consumed text through a forward
cursor and accumulated raw chunks would lose whichever half it did not look at.
"""

from __future__ import annotations

import re
import time
from dataclasses import dataclass
from dataclasses import field
from typing import Dict
from typing import List
from typing import Optional
from typing import Sequence

#: The 128-bit service of the example, as bytes on the wire: "sifli_app" followed
#: by zeros. Its characteristic is the same bytes with the 0x00 at offset 10
#: replaced by 0x01; only the service UUID is needed, because discovery returns
#: the characteristics with their handles.
SERVICE_UUID_BYTES = bytes(
    [0x73, 0x69, 0x66, 0x6C, 0x69, 0x5F, 0x61, 0x70, 0x70] + [0x00] * 7
)

#: ``diss scan start`` argument: duplicate filter, interval, window, duration
#: (ms), RSSI filter (dBm). 100/50 ms over a 30 ms advertising interval finds a
#: nearby peripheral within a second or two.
SCAN_INTERVAL_MS = 100
SCAN_WINDOW_MS = 50

#: ``diss conn_idx start`` arguments for the initiator: own address type,
#: supervision timeout (ms), connection interval (ms), scan interval, scan
#: window (ms). 30 ms matches the example's advertising interval.
CONNECT_PARAMS = (0, 5000, 30, SCAN_INTERVAL_MS, SCAN_WINDOW_MS)


class _Console:
    """The part of the DUT console this module uses."""

    def send_line(self, line: str) -> None:  # pragma: no cover - protocol only
        raise NotImplementedError

    def read(self, timeout: float = 1.0) -> str:  # pragma: no cover
        raise NotImplementedError

    def drain(self) -> None:  # pragma: no cover
        raise NotImplementedError


@dataclass(frozen=True)
class Advertisement:
    """One device the instrument saw while scanning.

    ``rssi`` is ``None`` when the console carried the report's identity but not
    its signal line -- a split or a truncated read. Such a report sorts *last*
    rather than at 0 dBm: an incomplete entry must not win the "strongest"
    choice and send the peer after a device it cannot measure.
    """

    index: int
    address: str
    address_type: str
    rssi: Optional[int] = None


@dataclass(frozen=True)
class Characteristic:
    """One characteristic in a discovered service, with the handles a GATT client needs."""

    index: int
    handle: int
    value_handle: int
    descriptors: List[int] = field(default_factory=list)


@dataclass(frozen=True)
class ServiceListing:
    """What the instrument printed for one discovered service."""

    characteristics: List[Characteristic]

    def value_handle(self, index: int = 0) -> int:
        return self.characteristics[index].value_handle

    def cccd_handle(self, index: int = 0) -> int:
        """The first descriptor of a characteristic -- its CCCD.

        The example's service table has exactly one descriptor, the client
        characteristic configuration, so "first descriptor" is unambiguous here.
        """
        descriptors = self.characteristics[index].descriptors
        if not descriptors:
            raise AssertionError(
                f"characteristic {index} has no descriptor, so there is no CCCD to write"
            )
        return descriptors[0]


# -- parsing ----------------------------------------------------------------
#
# Kept as module functions on purpose: a recorded transcript can be fed straight
# to them in a test, without a board, and the failure then points at the parser
# rather than at the console plumbing around it.

_ADV_RE = re.compile(
    r"advertising device\s+(\d+): addr type: ([^,]+), addr: ([0-9a-fA-F]+(?:-[0-9a-fA-F]+)*)"
)
_RSSI_RE = re.compile(r"adv tx pwr: (-?\d+), rssi: (-?\d+)")
_CHAR_RE = re.compile(r"Charateristic\((\d+)\) handle (\d+), value handle (\d+)")
_DESC_RE = re.compile(r"Descriptor\((\d+)\) handle (\d+)")
_DEVICE_RE = re.compile(
    r"Device (\d+): role:\d+, address\(type \d+\): ([0-9a-fA-F]+(?:-[0-9a-fA-F]+)*)"
)
_DUMP_RANGE_RE = re.compile(r"([0-9A-Fa-f]{4})-([0-9A-Fa-f]{4}): ")


def parse_advertisements(text: str) -> List[Advertisement]:
    """Pull every advertisement report out of a scan transcript.

    The report is three lines (``advertising device``, ``adv type``, ``adv tx
    pwr``), so each block between two ``advertising device`` markers is searched
    for its RSSI; pairing by line index would break the moment on log line is
    split by the console buffer.
    """
    starts = [match.start() for match in _ADV_RE.finditer(text)]
    advertisements: List[Advertisement] = []
    for position, start in enumerate(starts):
        end = starts[position + 1] if position + 1 < len(starts) else len(text)
        block = text[start:end]
        head = _ADV_RE.match(block)
        if head is None:  # pragma: no cover - starts came from the same pattern
            continue
        rssi_match = _RSSI_RE.search(block)
        rssi = int(rssi_match.group(2)) if rssi_match else None
        advertisements.append(
            Advertisement(
                index=int(head.group(1)),
                address_type=head.group(2).strip(),
                address=head.group(3),
                rssi=rssi,
            )
        )
    return advertisements


def _by_signal(advertisement: Advertisement) -> int:
    """Sort key: strongest first, an unmeasured report last."""
    if advertisement.rssi is None:
        return -(2 ** 31)
    return advertisement.rssi


def parse_connected_devices(text: str) -> Dict[str, int]:
    """Map each connected peer's address to the instrument's device index.

    ``diss show_dev`` prints ``Device <idx>: role:..., address(type ..): a-b-c``
    with the same address spelling the scan used, so a suite can find the index
    of the link it just made instead of assuming it is 0 -- an instrument can
    already hold links to other devices.
    """
    return {match.group(2): int(match.group(1)) for match in _DEVICE_RE.finditer(text)}


def parse_service_listing(text: str) -> ServiceListing:
    """Parse the ``diss search_svc`` listing into characteristics and handles."""
    characteristics: List[Characteristic] = []
    current: Optional[dict] = None

    for line in text.splitlines():
        char_match = _CHAR_RE.search(line)
        if char_match:
            if current is not None:
                characteristics.append(Characteristic(**current))
            current = {
                "index": int(char_match.group(1)),
                "handle": int(char_match.group(2)),
                "value_handle": int(char_match.group(3)),
                "descriptors": [],
            }
            continue
        desc_match = _DESC_RE.search(line)
        if desc_match and current is not None:
            current["descriptors"].append(int(desc_match.group(2)))

    if current is not None:
        characteristics.append(Characteristic(**current))
    return ServiceListing(characteristics=characteristics)


def parse_hexdump(text: str, tag: str) -> bytes:
    """Collect the hexdump that ``tag`` introduced, from a console transcript.

    ulog prints ``D/HEX <tag>: NNNN-MMMM: <bytes>  <ascii>`` with a fixed 16-byte
    width. The hex region's length is fixed (three characters per byte, an extra
    space per eight, two spaces before the ASCII), so it is sliced rather than
    guessed -- the ASCII column can itself look like hex pairs, and reading until
    a gap would swallow it.

    Only *terminated* rows are returned. A console read can end in the middle of
    a row, and parsing that fragment would both truncate the payload and mark it
    consumed, so the rest would never be recovered.

    Lines are split on ``\\n`` rather than with :meth:`str.splitlines`: the
    consoles end lines with ``\\r\\r\\n``, which ``splitlines`` treats as two
    breaks -- the row (ending in ``\\r``) and the stray ``\\r\\n`` -- so a
    termination check against its pieces cannot tell a complete row from a
    fragment. Splitting on ``\\n`` gives each row whole, with the trailing
    ``\\r`` characters left on the end where they do no harm.
    """
    marker = f"{tag}:"
    lines = text.split("\n")
    start = None
    for position in range(len(lines) - 1):  # the last element is after the final "\n"
        if marker in lines[position] and _DUMP_RANGE_RE.search(lines[position]):
            start = position
            break
    if start is None:
        return b""

    collected = bytearray()
    for position in range(start, len(lines) - 1):
        line = lines[position]
        match = _DUMP_RANGE_RE.search(line)
        if match is None:
            # An empty line between rows is part of the dump, not its end.
            if line.strip():
                break
            continue
        start_offset = int(match.group(1), 16)
        end_offset = int(match.group(2), 16)
        width = end_offset - start_offset
        if width <= 0:
            break
        region_length = width * 3 + width // 8 + 2
        region = line[match.end(): match.end() + region_length]
        values = bytes(int(token, 16) for token in region.split() if len(token) == 2)
        collected.extend(values[:width])
    return bytes(collected)


# -- the instrument ---------------------------------------------------------


class BlePeer:
    """A BLE peer instrument, driven over its console.

    The console is the ``dut``-shaped object the plugin hands over (``read``,
    ``send_line``, ``drain``). ``dev_index`` is the index the example assigned to
    this peer's link in ITS OWN table, which :meth:`connect` reads back from
    ``diss show_dev`` -- an instrument may already be connected elsewhere, so
    assuming 0 would talk to the wrong device.
    """

    def __init__(self, console: _Console, *, dev_index: int = 0) -> None:
        self._console = console
        self.dev_index = dev_index
        #: Text read from the console but not yet consumed by a parser call.
        #: Notifications arrive unasked, so they must survive between calls.
        self._pending = ""

    # -- plumbing ------------------------------------------------------------

    def _pump(self, timeout: float) -> str:
        chunk = self._console.read(timeout)
        if chunk:
            self._pending += chunk
        return chunk

    def _take(self, pattern: str, timeout: float) -> str:
        """Read until ``pattern`` matches the pending text; return that text.

        The matched text stays in ``_pending`` -- callers parse it, and a caller
        that needs the following block (a hexdump, say) can read on.
        """
        return self._take_after(0, pattern, timeout)

    def _take_after(self, start: int, pattern: str, timeout: float) -> str:
        """``_take``, but only matches at or after ``start``.

        Text before ``start`` is other traffic -- a notification that arrived
        earlier, say -- and must neither satisfy this command's expectation nor
        be thrown away to make it match.
        """
        compiled = re.compile(pattern)
        deadline = _now() + timeout
        while True:
            if compiled.search(self._pending, start):
                return self._pending
            remaining = deadline - _now()
            if remaining <= 0:
                raise AssertionError(
                    f"expected /{pattern}/ within {timeout:.0f}s from the peer.\n"
                    f"--- peer said ---\n{self._pending or '<nothing>'}\n----------------"
                )
            self._pump(min(0.5, remaining))

    def _settle(self, quiet: float = 0.6, timeout: float = 3.0) -> None:
        """Read until the console has been quiet for ``quiet`` seconds."""
        deadline = _now() + timeout
        last_change = _now()
        while _now() < deadline:
            chunk = self._pump(0.2)
            now = _now()
            if chunk:
                last_change = now
            elif now - last_change >= quiet:
                return

    def _consume(self, count: int) -> None:
        self._pending = self._pending[count:]

    # -- shell ---------------------------------------------------------------

    def expect_shell(self, timeout: float = 10.0) -> None:
        """The instrument is running and its msh is answering.

        ``diss help`` prints the command list, which is the cheapest thing that
        proves the *right* firmware is on the instrument -- an empty prompt from
        another image would satisfy a bare prompt match.

        The pending buffer is cleared first: a second call in the same session
        must wait for a *new* answer rather than match the previous "Command
        list:" and pass against a dead instrument.
        """
        self._console.drain()
        self._pending = ""
        self._console.send_line("diss help")
        self._take(r"Command list:", timeout)

    # -- scan ----------------------------------------------------------------

    def scan(self, *, duration_ms: int = 3000, rssi: int = -127, timeout: float = 20.0) -> List[Advertisement]:
        """Scan and return every report, strongest first.

        The scan is left to expire (``duration_ms``), so its result does not
        depend on when the test happens to read; the stop indication is the end
        marker.
        """
        self._console.drain()
        self._pending = ""
        self._console.send_line(
            f"diss scan start 0 {SCAN_INTERVAL_MS} {SCAN_WINDOW_MS} {duration_ms} {rssi}"
        )
        self._take(r"Scan start status 0", 10.0)
        self._take(r"Scan stopped", timeout)
        self._settle(quiet=0.3, timeout=1.0)
        return sorted(parse_advertisements(self._pending), key=_by_signal, reverse=True)

    def strongest(self, advertisements: Sequence[Advertisement]) -> Advertisement:
        if not advertisements:
            raise AssertionError(
                "the peer scanned but saw no advertisement at all; "
                "check that the DUT is advertising and within radio range"
            )
        return advertisements[0]

    # -- connection ----------------------------------------------------------

    def connect(self, advertisement: Advertisement, timeout: float = 15.0) -> None:
        """Ask the instrument to initiate a connection to one scanned device.

        ``Advertisement.index`` is printed by the example *after* it appends the
        report, so the scan's 1-based number is exactly what ``conn_idx start``
        takes. (It is not a list index; adding one connects to the neighbour,
        which is a real mistake this test once made.)

        Once the link is up the device index is resolved from ``diss show_dev``,
        not assumed to be 0: an instrument may already be connected to
        something else, and every GATT command takes that index.
        """
        self._console.drain()
        self._pending = ""
        own_addr_type, supervision_ms, interval_ms, scan_interval, scan_window = CONNECT_PARAMS
        self._console.send_line(
            "diss conn_idx start "
            f"{advertisement.index} {own_addr_type} {supervision_ms} "
            f"{interval_ms} {scan_interval} {scan_window}"
        )
        text = self._take(r"connected as deivce \d+|Create connection failed", timeout)
        if "Create connection failed" in text:
            raise AssertionError(
                f"the peer failed to connect to {advertisement.address}:\n{text}"
            )
        self._pending = ""
        self._settle(quiet=0.5, timeout=3.0)
        self.dev_index = self._resolve_device_index(advertisement.address, timeout)

    def _resolve_device_index(self, address: str, timeout: float = 5.0) -> int:
        """Find the instrument's index for the link to ``address``."""
        deadline = _now() + timeout
        table = ""
        while True:
            self._console.drain()
            self._pending = ""
            self._console.send_line("diss show_dev")
            self._take(r"Total connected \d+ devices", max(0.5, deadline - _now()))
            self._settle(quiet=0.3, timeout=1.5)
            table = self._pending
            self._pending = ""
            index = parse_connected_devices(table).get(address)
            if index is not None:
                return index
            if _now() >= deadline:
                raise AssertionError(
                    f"the peer connected but its device table does not list "
                    f"{address} within {timeout:.0f}s:\n{table}"
                )

    # -- GATT client ---------------------------------------------------------

    def search_service(self, uuid: bytes = SERVICE_UUID_BYTES, timeout: float = 10.0) -> ServiceListing:
        """Discover a service by its 128-bit UUID and parse its characteristics.

        The listing is printed right after ``Service searched`` by the same
        handler, but the UART can lag; settle before parsing.
        """
        self._console.drain()
        self._pending = ""
        self._console.send_line(
            f"diss search_svc {self.dev_index} {len(uuid)} {uuid.hex()}"
        )
        self._take(r"Service searched", timeout)
        # The example registers the remote service right after printing the
        # listing, and read/write need that registration. Waiting for its
        # result is what makes the next call deterministic instead of racing it.
        registered = self._take(r"client register ret \d+", timeout)
        if "client register ret 0" not in registered:
            raise AssertionError(
                f"the peer could not register the discovered service:\n{registered}"
            )
        self._settle(quiet=0.4, timeout=2.0)
        listing = parse_service_listing(self._pending)
        self._pending = ""
        if not listing.characteristics:
            raise AssertionError("the peer searched the service but printed no characteristic")
        return listing

    def read_value(self, handle: int, timeout: float = 10.0) -> bytes:
        """Read a characteristic value and return its bytes.

        Anchored on the response line, not on the dump: a successful read of a
        zero-length value prints ``Read(handle N) received`` and then nothing,
        because ``ulog_hexdump`` emits no line for size 0. Waiting for
        ``Read content`` would turn a legal empty value into a timeout.

        Only the response and its dump are consumed. Notifications share this
        console, so draining the buffer or clearing it afterwards would silently
        drop packets the next ``wait_notification`` is owed.
        """
        start = len(self._pending)
        self._console.send_line(f"diss read_val {self.dev_index} {handle}")
        response = f"Read(handle {handle}) received"
        self._take_after(
            start, f"{re.escape(response)}|Read remote value", timeout
        )
        if "Read remote value" in self._pending[start:]:
            raise AssertionError(f"the peer failed to read handle {handle}")
        self._settle(quiet=0.4, timeout=1.5)

        marker = self._pending.find(response, start)
        dump_start = self._pending.find("Read content:", marker)
        if dump_start < 0:
            # The log carries no value length, so an empty value and a payload
            # that has not arrived yet look identical. The bounded grace is the
            # trade-off: a late dump beyond it would be reported as empty, while
            # waiting indefinitely would turn every legal empty read into a
            # timeout. The firmware prints the response and its dump back to
            # back, so the window only matters under a stalled transport.
            value = b""
            end = _line_end(self._pending, marker + len(response))
        else:
            end = self._await_dump_end(dump_start, timeout)
            # Same reason as in wait_notification: one terminated row is the
            # minimum, and a multi-row value needs a quiet window to prove the
            # rest is not still in flight.
            self._settle(quiet=0.5, timeout=1.5)
            end = max(end, _dump_end(self._pending, dump_start))
            value = parse_hexdump(self._pending[marker:], "Read content")
        # Only the response and its dump are removed. The slice assumes nothing
        # else was printed between them: the firmware dispatches the read
        # response and notification events from one switch, so they cannot be
        # interleaved inside this block.
        self._pending = self._pending[:marker] + self._pending[end:]
        return value

    def write_value(self, handle: int, data: bytes, timeout: float = 5.0) -> None:
        """Write a characteristic value with a write request.

        The text read while settling is deliberately kept: a write can be what
        starts the server's notification burst (the example does exactly that
        for a value whose low byte is 0x11), and those packets arrive on the same
        console. Draining here would throw away the very notifications the next
        ``wait_notification`` is waiting for.

        An empty payload is refused rather than sent: the firmware's
        ``diss write_val`` reads a hexadecimal argument unconditionally and
        asserts on the zero-byte buffer, so there is no safe way to express it.

        The shell prints ``write context`` for every write it executes, so this
        waits for it: silence means the instrument never ran the command (hung,
        wrong shell, console gone), and treating that as success hid a dead peer.
        """
        if not data:
            raise ValueError(
                "write_value() needs at least one byte; the instrument's "
                "diss write_val cannot express an empty write"
            )
        start = len(self._pending)
        self._console.send_line(
            f"diss write_val {self.dev_index} {handle} {len(data)} {data.hex()}"
        )
        self._take_after(
            start, r"write context: [0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}: ", timeout
        )
        self._settle(quiet=0.3, timeout=1.0)
        # The firmware logs this when the stack refuses the write (dead link,
        # bad handle). Checking is what keeps "the write went nowhere" from
        # looking like a pass.
        if "Write remote value" in self._pending[start:]:
            raise AssertionError(
                f"the peer failed to write handle {handle}:\n{self._pending[start:]}"
            )

    def subscribe(self, cccd_handle: int, timeout: float = 5.0) -> None:
        """Enable notifications by writing 0x0001 into the CCCD.

        This confirms the command ran, not that the stack accepted it: the
        example's ``write_cccd`` ignores the write's result and prints the
        confirmation unconditionally, so a rejected subscription has no distinct
        log line here. The real proof is the notification that follows -- a
        subscription that failed surfaces there, as a ``wait_notification``
        timeout rather than as a silent pass.

        Nothing is drained: notifications for an earlier subscription may
        already be queued, and a second subscription must not discard them.
        The confirmation is matched from this point on, so an older one cannot
        satisfy it instead.
        """
        start = len(self._pending)
        self._console.send_line(f"diss write_cccd {self.dev_index} {cccd_handle}")
        self._take_after(start, r"current_conn_idx \d+, conn_idx \d+", timeout)
        # Deliberately not cleared: a peripheral may notify the moment the CCCD
        # is enabled, and those packets can share this read with the
        # confirmation. ``wait_notification`` consumes from this same buffer.

    def wait_notification(self, handle: int, timeout: float = 10.0) -> bytes:
        """Wait for the next notification on ``handle`` and return its payload.

        Called repeatedly, each call consumes one notification; nothing is
        drained between calls, so a burst is not lost.

        The event line and its hexdump are two separate prints, so the payload
        is waited for (within the caller's budget) rather than assumed to be in
        the same read: a fixed quiet window once truncated every 20-byte packet
        to 16. What is consumed is exactly the event plus its dump -- a partial
        next event line is kept, or the following call would wait for a
        notification that already arrived.

        A zero-length notification prints no dump at all (``ulog_hexdump``
        emits nothing for size 0), and the firmware prints in order, so "a
        complete next event line arrived before any dump" means this one is
        empty, and ``b""`` is returned. Nothing else is taken as empty: a
        payload that is merely late would otherwise be silently replaced by an
        empty return, so an event with neither a payload nor a following event
        raises when the caller's budget runs out. That last case is a limit of
        the instrument's log, which does not carry the notification's length --
        and for the same reason, a multi-row payload is considered complete
        only after a quiet window (see ``_settle`` below).

        Other handles' notifications are left alone. The firmware writes every
        handle's event to the same console, so a caller waiting for handle 20
        must not discard handle 10's packet that arrived first.
        """
        deadline = _now() + timeout
        event = f"Notify(handle {handle}) received"
        pattern = rf"Notify\(handle {handle}\) received"
        self._take(pattern, timeout)
        match = re.search(pattern, self._pending)
        start = match.start() if match else 0
        search_from = start + len(event)

        following = re.compile(r"Notify\(handle \d+\) received")
        while True:
            dump_start = self._pending.find("Notify content:", search_from)
            next_event = following.search(self._pending, search_from)
            # A complete second event line (its newline has arrived) before any
            # dump is the only proof of an empty notification.
            empty = (
                next_event is not None
                and self._pending.find("\n", next_event.end()) >= 0
                and (dump_start < 0 or next_event.start() < dump_start)
            )
            if empty:
                self._pending = self._pending[:start] + self._pending[search_from:]
                return b""
            if dump_start >= 0 and (next_event is None or dump_start < next_event.start()):
                break
            remaining = deadline - _now()
            if remaining <= 0:
                raise AssertionError(
                    "the notification carried neither a complete payload nor a "
                    f"following event within {timeout:.0f}s:\n{self._pending}"
                )
            self._pump(min(0.2, remaining))

        end = self._await_dump_end(dump_start, max(0.05, deadline - _now()))
        # Longer than the agent's 0.2 s console poll, and the only completeness
        # signal the log offers: it has no length field, so the last row is
        # trusted once the line has been quiet this long.
        self._settle(quiet=0.5, timeout=1.5)
        end = max(end, _dump_end(self._pending, dump_start))
        value = parse_hexdump(self._pending[start:], "Notify content")
        self._pending = self._pending[:start] + self._pending[end:]
        return value

    def _await_dump_end(self, dump_start: int, timeout: float) -> int:
        """Wait until the dump at ``dump_start`` has at least one full row."""
        deadline = _now() + timeout
        end = _dump_end(self._pending, dump_start)
        while end == dump_start:
            if _now() >= deadline:
                raise AssertionError(
                    "the instrument started a hexdump but never completed a row:\n"
                    f"{self._pending}"
                )
            self._pump(min(0.2, max(0.1, deadline - _now())))
            end = _dump_end(self._pending, dump_start)
        return end


def _line_end(text: str, index: int) -> int:
    """Index just past the line containing ``index`` (or ``len(text)``)."""
    newline = text.find("\n", index)
    return len(text) if newline < 0 else newline + 1


def _dump_end(text: str, start: int) -> int:
    """Index just past the dump rows that begin at ``start``.

    An empty line between rows is part of the dump; the first non-blank line
    that is not a row ends it. A row without its newline is a fragment still in
    flight, so it does not count: consuming it would lose the rest. Splitting on
    ``\\n`` keeps each row with its own text (see :func:`parse_hexdump`).
    """
    end = start
    offset = start
    parts = text[start:].split("\n")
    for index, line in enumerate(parts):
        if index == len(parts) - 1:
            break  # unterminated tail
        line_end = offset + len(line) + 1
        if _DUMP_RANGE_RE.search(line):
            end = line_end
        elif line.strip():
            break
        offset = line_end
    return end


def _now() -> float:
    return time.monotonic()
