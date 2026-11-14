# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Serial sessions on the USB device's own CDC ports.

The same shape as the console stream in :mod:`lab_agent.testbed` -- a reader
thread, a bounded buffer and an absolute cursor -- but over bytes rather than
text, and with the extra rule that only one session may own a physical port at
a time. Two readers on one COM port split or lose bytes unpredictably, and a
test would then fail for a reason the report cannot show.

``pyserial`` is imported when a port is opened, not when the module is imported:
the agent's own self-tests run without it, and a missing dependency must fail
the one request that needs it rather than the whole server.
"""

from __future__ import annotations

import secrets
import threading
import time
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

from .errors import UsbHostPortBusy
from .errors import UsbHostPortClosed
from .errors import UsbHostPortDenied
from .errors import UsbHostTokenNotFound

#: Retained bytes per session. A session that falls this far behind loses the
#: oldest bytes, and the cursor keeps advancing so the arithmetic stays honest.
DEFAULT_MAX_BUFFER = 4 * 1024 * 1024

#: Upper bound on one read request, in seconds.
MAX_READ_TIMEOUT = 60.0

#: Upper bound on one write body, in bytes.
MAX_WRITE_BYTES = 64 * 1024


def _open_serial(port: str, baud: int) -> object:
    """Open the real port. Kept separate so tests can replace it."""
    import serial

    return serial.Serial(port=port, baudrate=baud, timeout=0.05)


class ByteStream:
    """One open USB CDC port with a monotonic cursor into what it received."""

    def __init__(
        self,
        port: str,
        baud: int,
        *,
        dtr: bool = False,
        rts: bool = False,
        factory=_open_serial,
        max_buffer: int = DEFAULT_MAX_BUFFER,
        poll: float = 0.05,
    ) -> None:
        self.port = port
        self.baud = baud
        self.dtr = bool(dtr)
        self.rts = bool(rts)
        self._factory = factory
        self._max_buffer = max_buffer
        self._poll = poll
        self._condition = threading.Condition()
        self._buffer = b""
        #: Bytes dropped off the front to respect ``max_buffer``. Cursors stay
        #: absolute, so they survive trimming.
        self._dropped = 0
        self._serial: Optional[object] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._failure: Optional[BaseException] = None
        self._lock = threading.Lock()

    # -- lifecycle -----------------------------------------------------------

    def start(self) -> None:
        if self._thread is not None:
            return
        handle = self._factory(self.port, self.baud)
        try:
            handle.dtr = self.dtr
            handle.rts = self.rts
        except Exception:
            # A fake used by the self-tests may not model modem lines; a real
            # pyserial port either accepts them or the open itself had failed.
            pass
        self._serial = handle
        self._stop.clear()
        self._failure = None
        self._thread = threading.Thread(
            target=self._pump, name=f"usb-serial-{self.port}", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        thread, self._thread = self._thread, None
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        if thread is not None:
            thread.join(timeout=3.0)
        handle, self._serial = self._serial, None
        if handle is not None:
            try:
                handle.close()
            except Exception:
                pass

    @property
    def closed(self) -> bool:
        with self._condition:
            return self._stop.is_set() or self._failure is not None

    @property
    def cursor(self) -> int:
        with self._condition:
            return self._end

    # -- reader thread -------------------------------------------------------

    def _pump(self) -> None:
        handle = self._serial
        while not self._stop.is_set():
            try:
                chunk = handle.read(4096)
            except BaseException as exc:  # noqa: BLE001 - surfaced to readers
                with self._condition:
                    self._failure = exc
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

    @property
    def _end(self) -> int:
        return self._dropped + len(self._buffer)

    # -- io ------------------------------------------------------------------

    def read_from(self, cursor: int, timeout: float) -> Tuple[bytes, int, bool]:
        """Wait up to ``timeout`` for bytes after ``cursor``.

        Returns ``(data, new_cursor, closed)``. Empty data with ``closed`` false
        is a normal timeout. ``closed`` true means the port failed or was
        stopped; whatever was already buffered is still returned.
        """
        cursor = int(cursor)
        deadline = time.monotonic() + max(0.0, float(timeout))
        with self._condition:
            while True:
                if cursor < self._dropped:
                    cursor = self._dropped
                if self._end > cursor:
                    return (
                        self._buffer[cursor - self._dropped :],
                        self._end,
                        self._stop.is_set() or self._failure is not None,
                    )
                if self._stop.is_set() or self._failure is not None:
                    return b"", self._end, True
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return b"", self._end, False
                self._condition.wait(remaining)

    def write(self, data: bytes) -> int:
        if self._stop.is_set() or self._failure is not None:
            raise UsbHostPortClosed(
                f"{self.port}: the serial port is closed"
            )
        handle = self._serial
        if handle is None:
            raise UsbHostPortClosed(f"{self.port}: the serial port is closed")
        with self._lock:
            try:
                written = handle.write(data)
            except Exception as exc:  # noqa: BLE001 - translated to the API
                raise UsbHostPortClosed(
                    f"{self.port}: write failed: {exc}"
                ) from exc
        return int(written if written is not None else len(data))

    def set_lines(self, dtr: Optional[bool] = None, rts: Optional[bool] = None) -> None:
        if self._stop.is_set() or self._failure is not None:
            raise UsbHostPortClosed(
                f"{self.port}: the serial port is closed"
            )
        handle = self._serial
        if handle is None:
            raise UsbHostPortClosed(f"{self.port}: the serial port is closed")
        with self._lock:
            try:
                if dtr is not None:
                    handle.dtr = bool(dtr)
                    self.dtr = bool(dtr)
                if rts is not None:
                    handle.rts = bool(rts)
                    self.rts = bool(rts)
            except Exception as exc:  # noqa: BLE001 - translated to the API
                raise UsbHostPortClosed(
                    f"{self.port}: setting modem lines failed: {exc}"
                ) from exc


class SerialSessionRegistry:
    """Every open USB serial session on this testbed, by token and by port."""

    def __init__(self, factory=_open_serial) -> None:
        self._factory = factory
        self._lock = threading.Lock()
        self._sessions: Dict[str, ByteStream] = {}
        self._by_port: Dict[str, str] = {}

    def open(
        self,
        port: str,
        baud: int,
        *,
        dtr: bool = False,
        rts: bool = False,
    ) -> Tuple[str, ByteStream]:
        """Open ``port`` and return its token. Raises if it is already open."""
        key = port.upper()
        with self._lock:
            if key in self._by_port:
                raise UsbHostPortBusy(
                    f"{port} is already open in this testbed"
                )
            # Held across start(): the port is genuinely opened here, and a
            # release that slipped in between the check and the registration
            # would leave a port held on a testbed nobody owns.
            stream = ByteStream(
                port, int(baud), dtr=dtr, rts=rts, factory=self._factory
            )
            try:
                stream.start()
            except Exception as exc:  # noqa: BLE001 - translated to the API
                # ``pyserial`` refuses a port that another process holds, is
                # unplugged, or has disappeared between enumeration and open.
                # That is the caller's conflict, not a crashed agent (500).
                stream.stop()
                raise UsbHostPortDenied(
                    f"cannot open {port}: {exc}", details={"port": port}
                ) from exc
            token = self._new_token()
            self._sessions[token] = stream
            self._by_port[key] = token
            return token, stream

    def _new_token(self) -> str:
        while True:
            token = secrets.token_urlsafe(16)
            if token not in self._sessions:
                return token

    def get(self, token: str) -> ByteStream:
        with self._lock:
            stream = self._sessions.get(token)
        if stream is None:
            raise UsbHostTokenNotFound(
                f'no open serial session with token "{token}"'
            )
        return stream

    def close(self, token: str) -> None:
        # ``stop()`` closes the port only after its reader thread has exited, so
        # the lock is held across it: a reopen that slipped in after the pop but
        # before the port was released would fail on Windows (port still busy).
        with self._lock:
            stream = self._sessions.pop(token, None)
            if stream is None:
                raise UsbHostTokenNotFound(
                    f'no open serial session with token "{token}"'
                )
            self._by_port.pop(stream.port.upper(), None)
            stream.stop()

    def close_all(self) -> None:
        # Same reason as ``close``: while the ports are being released no new
        # session may register, or teardown would leave one behind.
        with self._lock:
            sessions: List[ByteStream] = list(self._sessions.values())
            self._sessions.clear()
            self._by_port.clear()
            for stream in sessions:
                stream.stop()

    def tokens(self) -> List[str]:
        with self._lock:
            return list(self._sessions)
