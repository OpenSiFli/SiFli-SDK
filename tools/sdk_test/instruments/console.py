# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""The line protocol shared by every instrument.

The instrument prints machine-readable lines and nothing here parses its human
log. That is the whole point of the protocol:

* a request has exactly one terminal answer, correlated by id, so "did the
  command run?" is answered by an ``@ok``/``@err`` rather than by guessing from
  a log line;
* a payload carries an explicit ``len``, so a hexdump is complete when that many
  bytes have arrived -- no quiet-window heuristic;
* asynchronous events carry a monotonic sequence number, so packets that arrive
  between two commands are neither lost nor mistaken for a response.

Only complete lines are parsed. Everything else on the console (ulog, the msh
prompt, a fragmented line) is ignored, which lets the instrument keep its normal
logging on the same UART.
"""

from __future__ import annotations

import re
import time
from collections import deque
from typing import Any
from typing import Deque
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Protocol
from typing import Tuple

from .errors import InstrumentCapabilityError
from .errors import InstrumentCommandError
from .errors import InstrumentProtocolError
from .errors import InstrumentTimeout

#: How much of the raw stream an error message keeps. Enough to see the command
#: and the answer that did not arrive; bounded so a long session cannot make
#: every failure message megabytes long.
TRANSCRIPT_LIMIT = 4096

#: How many asynchronous events are retained. A test that waits for one handle
#: must not lose another handle's packet, but an instrument that floods the
#: console must not grow the host process without bound either.
EVENT_LIMIT = 1024

#: Default budget for one request. Firmware commands are short; a BLE scan or a
#: connection is the caller's job to bound with its own argument.
DEFAULT_TIMEOUT = 10.0

#: A drain must finish; a console that keeps returning bytes without advancing
#: is broken, and spinning on it would hang the suite instead of reporting why.
MAX_DRAIN_READS = 4096

#: How many sent lines are kept for `console.sent[-1]` and diagnostics.
#: A long session issues thousands; keeping all of them is a slow leak.
SENT_LIMIT = 256

#: How many expired request ids are remembered, so a late answer can be
#: recognised as late instead of matching a later request.
EXPIRED_LIMIT = 1024

#: How many answered request ids are remembered, so a second answer can be
#: counted as a duplicate instead of being stored forever.
ANSWERED_LIMIT = 1024

#: ANSI SGR sequences. ulog can be built with colour, and its reset code
#: (``\x1b[0m``) is emitted before the *next* line -- so an ``@ok`` line can
#: arrive as ``\x1b[0m@ok ...``. Without stripping, that line is not recognised
#: as protocol at all and the request times out with the answer in hand.
_ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

#: Where a protocol line starts inside a chunk. The shell prompt (``msh />``)
#: has no trailing newline, so an asynchronous event can arrive as
#: ``msh />@evt ...``; requiring ``@`` at column zero would drop that packet.
#: The marker is accepted at column zero or right after that prompt -- not
#: anywhere in the line, because a ulog line that merely mentions ``@ok`` must
#: not be taken for the answer to a request.
_PROMPT_RE = re.compile(r"msh [^>\n]*>")
_PROTOCOL_START_RE = re.compile(r"@(?:info|ok|err|evt)\b")


def _protocol_start(line: str) -> Optional[int]:
    """Index where a protocol line begins, or ``None`` when it is not one."""
    if _PROTOCOL_START_RE.match(line):
        return 0
    prompt = _PROMPT_RE.match(line)
    if prompt is not None:
        rest = line[prompt.end():]
        stripped = rest.lstrip()
        if _PROTOCOL_START_RE.match(stripped):
            return prompt.end() + (len(rest) - len(stripped))
    return None


class LineConsole(Protocol):
    """The part of a console this module needs.

    ``RemoteConsole`` (the Lab Agent client) and the fake console in the tests
    both have this shape. The protocol deliberately does not require ``drain``:
    draining is a host-side operation here, because events must survive it.
    """

    def read(self, timeout: float = 1.0) -> str: ...  # pragma: no cover

    def send_line(self, line: str) -> None: ...  # pragma: no cover


class InstrumentConsole:
    """Request/response correlation and an event queue over one console."""

    def __init__(self, console: LineConsole) -> None:
        self._console = console
        #: Unparsed tail of the stream (a line still in flight).
        self._buffer = ""
        #: Everything received, bounded. `_buffer` only holds the incomplete
        #: tail, so an error message built from it would lose the log lines and
        #: responses that were already consumed -- exactly the text a reader
        #: needs.
        self._recent = ""
        self._next_id = 1
        #: Request id -> ("ok", values) or ("err", code, text).
        self._responses: Dict[int, tuple] = {}
        #: Requests whose wait already expired. A late answer is dropped rather
        #: than matched to the next request, which is the failure mode that
        #: makes a dead instrument look alive.
        self._expired: set = set()
        #: Requests whose answer was handed to a waiter. A second answer for one
        #: of these is a duplicate; storing it would leak and hide the bug.
        self._answered: set = set()
        self._info: Optional[Dict[str, str]] = None
        #: What `@info` said the firmware's line budget is; 0 means unknown.
        self._max_line = 0
        self._events: Deque[Dict[str, Any]] = deque()
        #: Highest event sequence seen. A scan/discovery captures this before
        #: sending its request so a stale event from an earlier one cannot
        #: satisfy the new call.
        self._last_seq = 0
        #: Recent request lines, bounded. Used by tests via `sent` and by
        #: nothing else -- failure messages carry the line they sent directly.
        self._sent: List[str] = []
        #: Counters a test can assert on. Silently dropping either kind of line
        #: would hide exactly the bug this protocol exists to make visible.
        self.late_responses = 0
        self.stray_responses = 0
        self.duplicate_responses = 0
        self.event_overflow = 0
        #: Events discarded because they predate a caller's boundary.
        self.stale_events = 0

    # -- public --------------------------------------------------------------

    @property
    def transcript(self) -> str:
        """The tail of what the instrument said, for failure messages."""
        return self._recent[-TRANSCRIPT_LIMIT:]

    @property
    def sent(self) -> List[str]:
        return list(self._sent)

    def _remember(self, line: str) -> None:
        """Record a sent line, keeping only the most recent ones."""
        self._sent.append(line)
        if len(self._sent) > SENT_LIMIT:
            del self._sent[0]

    @property
    def last_seq(self) -> int:
        """The highest event sequence received so far."""
        return self._last_seq

    @property
    def max_line(self) -> int:
        """The firmware's line budget from ``@info``, or 0 when unknown."""
        return self._max_line

    def command(self, op: str, timeout: float = DEFAULT_TIMEOUT, **kwargs: Any) -> Dict[str, str]:
        """Send one request and return its ``@ok`` values.

        ``bytes`` values are encoded as ``len=N data=<hex>``. A caller must not
        pass both a bytes ``data`` and its own ``len``; the length is derived
        from the payload so the two cannot disagree.
        """
        request_id = self._next_id
        self._next_id += 1
        line = self._build_request(request_id, op, kwargs)
        self._remember(line)
        self._console.send_line(line)
        return self._await_response(request_id, op, line, timeout)

    @staticmethod
    def event_boundary(result: Dict[str, str], op: str) -> int:
        """The event-sequence boundary an operation's ``@ok`` reports.

        The firmware takes it before the operation can emit anything, which is
        what makes its events separable from an earlier operation's still in
        flight. A reply without it cannot be used safely, so it is an error.
        """
        raw = result.get("first_seq")
        if raw is None:
            raise InstrumentProtocolError(
                f"the instrument's `{op}` reply carried no first_seq. Either the "
                "image predates the event-boundary feature (reflash the "
                "instrument; `peer info` should report fw=bt_peer-m3 or later) or "
                "this op is not supposed to report one -- in that case pass "
                "boundary=\"host\"."
            )
        try:
            return int(raw)
        except ValueError as exc:
            raise InstrumentProtocolError(
                f"first_seq={raw!r} is not an integer"
            ) from exc

    def command_events(
        self,
        op: str,
        event: str,
        *,
        count_key: str = "count",
        match: Optional[Dict[str, Any]] = None,
        timeout: float = DEFAULT_TIMEOUT,
        boundary: str = "firmware",
        **kwargs: Any,
    ) -> Tuple[Dict[str, str], List[Dict[str, Any]]]:
        """Send one request and consume exactly the events it promises.

        Every event-producing operation has the same shape: the terminal answer
        says how many events follow, and (unless ``boundary="host"`` is asked
        for) the sequence the firmware was at before any of them could be
        emitted. Consuming exactly that many, from that boundary, is what keeps
        an earlier operation's late events out of this one's result.

        ``boundary="host"`` exists only for a firmware that does not report
        ``first_seq`` yet; it is a documented fallback, not a default.

        Returns ``(result, events, boundary)``. The boundary is returned so a
        caller that also wants a related event (``scan.done``) filters it the
        same way this call did, instead of re-deriving one.
        """
        if boundary not in ("firmware", "host"):
            # Checked before sending: a typo must not run the operation.
            raise ValueError(f"unknown boundary kind {boundary!r}")

        self.drain()
        host_boundary = self.last_seq
        result = self.command(op, timeout=timeout, **kwargs)

        start = (
            self.event_boundary(result, op) if boundary == "firmware"
            else host_boundary
        )
        count = _count_value(result, count_key, op)

        # Applied even when no events are promised: the events this operation
        # displaced are stale either way.
        self._drop_stale(start)

        return result, [
            self.next_event(event, timeout=timeout, since_seq=start, **(match or {}))
            for _ in range(count)
        ], start

    def expect_ready(self, capability: str = "") -> Dict[str, str]:
        """Ask ``peer info`` and require the firmware to be the expected one.

        A bare msh prompt would also be printed by a board running the old
        ``diss`` image, so the prompt is not a readiness gate. ``proto`` and
        ``caps`` are.
        """
        self._info = None
        line = "peer info"
        self._remember(line)
        self._console.send_line(line)

        deadline = time.monotonic() + DEFAULT_TIMEOUT
        while self._info is None:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise InstrumentTimeout(
                    "the instrument did not answer `peer info` within "
                    f"{DEFAULT_TIMEOUT:.0f}s.\n--- instrument said ---\n"
                    f"{self.transcript or '<nothing>'}\n-----------------------"
                )
            self._pump(min(0.2, remaining))

        info = self._info
        if info.get("proto") != "1":
            raise InstrumentCapabilityError(
                "the instrument answered `peer info` without proto=1; this is "
                f"not the bt_peer firmware (got proto={info.get('proto')!r}).\n"
                f"{dict(info)}"
            )
        # A fixture parameter may name several capabilities at once
        # (`bt_peer:ble,classic`), because one console stream can carry both.
        # All of them are required: a half-equipped peer must not satisfy a
        # dual-mode case and then fail halfway through it.
        required = [entry.strip() for entry in capability.split(",") if entry.strip()]
        if required:
            caps = {
                entry.strip()
                for entry in info.get("caps", "").split(",")
                if entry.strip()
            }
            missing = [entry for entry in required if entry not in caps]
            if missing:
                raise InstrumentCapabilityError(
                    f"the instrument does not provide capability {missing!r} "
                    f"(caps={sorted(caps) or 'none'}). The lab must flash a "
                    "bt_peer image with that role, or the case must not ask for it."
                )
        return dict(info)

    def next_event(
        self,
        name: str,
        timeout: float = DEFAULT_TIMEOUT,
        *,
        since_seq: Optional[int] = None,
        **match: Any,
    ) -> Dict[str, Any]:
        """Return the first event called ``name`` matching ``match``.

        Non-matching events stay in the queue, in order: a caller waiting for
        one handle's notification must not discard another handle's packet that
        arrived first. Events at or before ``since_seq`` are *discarded*, not
        skipped -- a boundary only moves forward, so nothing can want them
        again, and keeping them would make every later wait scan history.
        """
        deadline = time.monotonic() + timeout
        while True:
            found = self._take_event(name, match, since_seq)
            if found is not None:
                return found
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise InstrumentTimeout(
                    f"expected event {name!r} matching {match or '{}'} within "
                    f"{timeout:.1f}s.\n--- instrument said ---\n"
                    f"{self.transcript or '<nothing>'}\n-----------------------"
                )
            self._pump(min(0.2, remaining))

    def _counters(self) -> str:
        """The non-zero bookkeeping, for a failure message.

        These exist so a protocol problem is visible in a real run, not only to
        a unit test: "duplicate=1" explains a strange answer, "stale=7" explains
        a queue that seemed to lose a packet.
        """
        parts = [
            f"{name}={getattr(self, name)}"
            for name in (
                "late_responses",
                "stray_responses",
                "duplicate_responses",
                "event_overflow",
                "stale_events",
            )
            if getattr(self, name)
        ]
        return f" ({', '.join(parts)})" if parts else ""

    def clear_events(self) -> None:
        """Drop queued events after a state reset.

        Called once the instrument has been returned to a known state; events
        from the previous state (a disconnection, a stale advertisement) must
        not satisfy a wait in the next one.
        """
        self._events.clear()

    def drain(self) -> None:
        """Consume whatever is buffered now, without losing events.

        This is a host-side read, not ``RemoteConsole.skip_to_end``: the latter
        would throw away text that has not been parsed, including events.
        """
        for _ in range(MAX_DRAIN_READS):
            chunk = self._console.read(0.0)
            if not chunk:
                return
            self._feed(chunk)
        raise InstrumentProtocolError(
            "drain() kept reading without reaching the end of the stream; the "
            "console is not advancing its cursor"
        )

    # -- request building ----------------------------------------------------

    def _build_request(self, request_id: int, op: str, kwargs: Dict[str, Any]) -> str:
        values = dict(kwargs)
        data = values.pop("data", None)
        if data is not None and not isinstance(data, (bytes, bytearray)):
            raise InstrumentProtocolError(
                f"a `data` argument must be bytes; got {type(data).__name__}"
            )
        if data is not None:
            # A caller-provided length is discarded: the only length that can
            # be right is the one derived from the payload.
            values.pop("len", None)

        parts = ["peer", "req", str(request_id), op]
        for key, value in values.items():
            if value is None:
                continue
            parts.append(f"{key}={value}")
        if data is not None:
            parts.append(f"len={len(data)}")
            parts.append(f"data={bytes(data).hex()}")

        line = " ".join(parts)
        if self._max_line and len(line) + 2 > self._max_line:
            raise InstrumentProtocolError(
                f"the request is {len(line) + 2} bytes including CRLF, over the "
                f"instrument's max_line={self._max_line}; shorten the payload "
                "or split it (the M1 firmware has no chunked continuation)"
            )
        return line

    def _await_response(
        self, request_id: int, op: str, line: str, timeout: float
    ) -> Dict[str, str]:
        deadline = time.monotonic() + timeout
        while True:
            answer = self._responses.pop(request_id, None)
            if answer is not None:
                self._answered.add(request_id)
                if len(self._answered) > ANSWERED_LIMIT:
                    self._answered = set(
                        sorted(self._answered)[-ANSWERED_LIMIT // 2:]
                    )
                kind = answer[0]
                if kind == "ok":
                    return answer[1]
                _, code, text = answer
                raise InstrumentCommandError(op, code, text, sent=line)

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._expired.add(request_id)
                if len(self._expired) > EXPIRED_LIMIT:
                    self._expired = set(
                        sorted(self._expired)[-EXPIRED_LIMIT // 2:]
                    )
                raise InstrumentTimeout(
                    f"no answer to `{op}` within {timeout:.1f}s.\n"
                    f"sent: {line}\n--- instrument said ---\n"
                    f"{self.transcript or '<nothing>'}\n-----------------------"
                )
            self._pump(min(0.2, remaining))

    # -- parsing -------------------------------------------------------------

    def _pump(self, timeout: float) -> None:
        chunk = self._console.read(timeout)
        if chunk:
            self._feed(chunk)

    def _feed(self, text: str) -> None:
        self._recent += text
        if len(self._recent) > TRANSCRIPT_LIMIT:
            self._recent = self._recent[-TRANSCRIPT_LIMIT:]
        self._buffer += text
        # Split on "\n", not splitlines(): the consoles end lines with
        # "\r\r\n", which splitlines() treats as two breaks.
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            self._handle_line(_ANSI_RE.sub("", line).rstrip("\r"))

    def _handle_line(self, line: str) -> None:
        start = _protocol_start(line)
        if start is None:
            return
        if start > 0:
            # Drop whatever preceded it (a shell prompt); the protocol line
            # itself starts at the marker.
            line = line[start:]
        head, _, rest = line[1:].partition(" ")
        tokens = rest.split()

        if head == "info":
            self._info = _key_values(tokens)
            try:
                self._max_line = int(self._info.get("max_line", "0"))
            except ValueError:
                self._max_line = 0
            return

        if head == "ok":
            request_id = _require_int(tokens, 0, line)
            self._store_response(request_id, ("ok", _key_values(tokens[1:])))
            return

        if head == "err":
            request_id = _require_int(tokens, 0, line)
            code = tokens[1] if len(tokens) > 1 else "error"
            text = " ".join(tokens[2:])
            self._store_response(request_id, ("err", code, text))
            return

        if head == "evt":
            sequence = _require_int(tokens, 0, line)
            if len(tokens) < 2:
                raise InstrumentProtocolError(f"malformed event line: {line!r}")
            event = {
                "seq": sequence,
                "name": tokens[1],
                "values": _key_values(tokens[2:]),
            }
            self._last_seq = max(self._last_seq, sequence)
            self._events.append(event)
            if len(self._events) > EVENT_LIMIT:
                # Drop the oldest and say so; a silent drop would leave a test
                # waiting for a packet that was thrown away here.
                self._events.popleft()
                self.event_overflow += 1
            return

        raise InstrumentProtocolError(f"unknown protocol line: {line!r}")

    def _store_response(self, request_id: int, answer: tuple) -> None:
        if request_id in self._expired:
            self._expired.discard(request_id)
            self.late_responses += 1
            return
        if request_id in self._answered:
            # Answered, handed over, and now answered again: a firmware bug.
            self.duplicate_responses += 1
            return
        if request_id < self._next_id:
            if request_id in self._responses:
                # Same, for the duplicate that arrives before the first is
                # collected: one request has exactly one terminal answer.
                self.duplicate_responses += 1
                return
            self._responses[request_id] = answer
            return
        # An id the host has not issued yet is not ours to guess at.
        self.stray_responses += 1

    def _drop_stale(self, since_seq: int) -> None:
        """Discard events a caller's boundary has put out of reach.

        Keeping them would cost every later wait a scan over history nobody can
        ask for again: boundaries only move forward.
        """
        dropped = 0
        while self._events and self._events[0]["seq"] <= since_seq:
            self._events.popleft()
            dropped += 1
        if dropped:
            self.stale_events += dropped

    def _take_event(
        self,
        name: str,
        match: Dict[str, Any],
        since_seq: Optional[int],
    ) -> Optional[Dict[str, Any]]:
        if since_seq is not None:
            self._drop_stale(since_seq)

        for position, event in enumerate(self._events):
            if event["name"] != name:
                continue
            if since_seq is not None and event["seq"] <= since_seq:
                continue
            values = event["values"]
            if all(
                value is None or values.get(key) == str(value)
                for key, value in match.items()
            ):
                del self._events[position]
                return event
        return None


def _count_value(result: Dict[str, str], key: str, op: str) -> int:
    """How many events a terminal answer promises."""
    raw = result.get(key)
    if raw is None:
        raise InstrumentProtocolError(
            f"the instrument's `{op}` reply carried no {key}, so the number of "
            "events that follow is unknown"
        )
    try:
        count = int(raw)
    except ValueError as exc:
        raise InstrumentProtocolError(f"{key}={raw!r} is not an integer") from exc
    if count < 0:
        raise InstrumentProtocolError(f"{key}={count} is negative")
    return count


def _key_values(tokens: Iterable[str]) -> Dict[str, str]:
    values: Dict[str, str] = {}
    for token in tokens:
        key, separator, value = token.partition("=")
        if separator:
            values[key] = value
    return values


def _require_int(tokens: List[str], index: int, line: str) -> int:
    if index >= len(tokens):
        raise InstrumentProtocolError(f"malformed protocol line: {line!r}")
    try:
        return int(tokens[index])
    except ValueError as exc:
        raise InstrumentProtocolError(f"malformed protocol line: {line!r}") from exc
