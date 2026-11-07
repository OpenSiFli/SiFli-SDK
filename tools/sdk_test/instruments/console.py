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

from .errors import InstrumentCapabilityError
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

#: ANSI SGR sequences. ulog can be built with colour, and its reset code
#: (``\x1b[0m``) is emitted before the *next* line -- so an ``@ok`` line can
#: arrive as ``\x1b[0m@ok ...``. Without stripping, that line is not recognised
#: as protocol at all and the request times out with the answer in hand.
_ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

#: Where a protocol line starts inside a chunk. The shell prompt (``msh />``)
#: has no trailing newline, so an asynchronous event can arrive as
#: ``msh />@evt ...``; requiring ``@`` at column zero would drop that packet.
_PROTOCOL_START_RE = re.compile(r"@(?:info|ok|err|evt)\b")


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
        self._info: Optional[Dict[str, str]] = None
        #: What `@info` said the firmware's line budget is; 0 means unknown.
        self._max_line = 0
        self._events: Deque[Dict[str, Any]] = deque()
        #: Highest event sequence seen. A scan/discovery captures this before
        #: sending its request so a stale event from an earlier one cannot
        #: satisfy the new call.
        self._last_seq = 0
        self._sent: List[str] = []
        #: Counters a test can assert on. Silently dropping either kind of line
        #: would hide exactly the bug this protocol exists to make visible.
        self.late_responses = 0
        self.stray_responses = 0
        self.event_overflow = 0

    # -- public --------------------------------------------------------------

    @property
    def transcript(self) -> str:
        """The tail of what the instrument said, for failure messages."""
        return self._recent[-TRANSCRIPT_LIMIT:]

    @property
    def sent(self) -> List[str]:
        return list(self._sent)

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
        self._sent.append(line)
        self._console.send_line(line)
        return self._await_response(request_id, op, line, timeout)

    def expect_ready(self, capability: str = "") -> Dict[str, str]:
        """Ask ``peer info`` and require the firmware to be the expected one.

        A bare msh prompt would also be printed by a board running the old
        ``diss`` image, so the prompt is not a readiness gate. ``proto`` and
        ``caps`` are.
        """
        self._info = None
        line = "peer info"
        self._sent.append(line)
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
        arrived first. ``since_seq`` ignores events from before the call that
        started this operation.
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

    def release_event(self, event: Dict[str, Any]) -> None:
        """Put an event back at the front, for a caller that looked too early."""
        self._events.appendleft(event)

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
                kind = answer[0]
                if kind == "ok":
                    return answer[1]
                _, code, text = answer
                raise InstrumentProtocolError(
                    f"the instrument rejected `{op}` with {code}: {text}\n"
                    f"sent: {line}"
                )

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._expired.add(request_id)
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
        start = _PROTOCOL_START_RE.search(line)
        if start is None:
            return
        if start.start() > 0:
            # Drop whatever preceded it (a shell prompt, a log line without a
            # newline); the protocol line itself starts at the marker.
            line = line[start.start():]
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
        if request_id < self._next_id:
            self._responses[request_id] = answer
            return
        # An id the host has not issued yet is not ours to guess at.
        self.stray_responses += 1

    def _take_event(
        self,
        name: str,
        match: Dict[str, Any],
        since_seq: Optional[int],
    ) -> Optional[Dict[str, Any]]:
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
