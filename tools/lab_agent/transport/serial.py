# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Serial console transport.

The discipline mirrors ``tools/sdk_py_actions/sifli_uart_dap.py``, which is the
in-tree precedent for talking to a SiFli board over a serial port:

* pyserial is imported lazily, so the Lab Agent starts (and its configuration
  can be validated) on a machine without it;
* :meth:`SerialTransport.open` is idempotent;
* the input buffer is flushed on open, so a previous run's output cannot be
  mistaken for this run's;
* reads return whatever arrived within a short timeout instead of blocking for a
  whole line, because the console has no line discipline to rely on
  (``FINSH_ECHO_DISABLE_DEFAULT=1`` means nothing is echoed back, and
  ``RT_CONSOLEBUF_SIZE`` is 128 bytes, so lines arrive in pieces).

The default baud rate is 1 Mbaud, matching `customer/peripherals/console`.
"""

from __future__ import annotations

import time
from typing import Optional

from ..errors import TransportError

#: The SDK console runs at 1 Mbaud (see drv_usart.c).
DEFAULT_BAUD = 1000000

#: How long a single read waits for bytes before returning what it has.
DEFAULT_READ_TIMEOUT = 0.05


class SerialTransport:
    """A serial port, opened lazily and closed best-effort."""

    def __init__(
        self,
        port: str,
        baud: int = DEFAULT_BAUD,
        read_timeout: float = DEFAULT_READ_TIMEOUT,
        write_timeout: float = 3.0,
    ) -> None:
        self.port = port
        self.baud = baud
        self.read_timeout = read_timeout
        self.write_timeout = write_timeout
        self._serial = None

    # -- lifecycle -----------------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self._serial is not None

    def open(self) -> None:
        """Open the port. Idempotent."""
        if self._serial is not None:
            return

        try:
            import serial  # lazy: pyserial is optional at import time
        except ImportError as exc:
            raise TransportError(
                "pyserial is required to talk to a board console; "
                "install it with `pip install pyserial`"
            ) from exc

        try:
            self._serial = serial.Serial(
                self.port,
                baudrate=self.baud,
                timeout=self.read_timeout,
                write_timeout=self.write_timeout,
            )
        except Exception as exc:
            # A busy port is the single most common lab failure; say so plainly
            # rather than surfacing a bare OSError.
            raise TransportError(
                f'Cannot open serial port "{self.port}" at {self.baud} baud: {exc}'
            ) from exc

        try:
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except Exception:
            # Not all drivers implement these; a stale buffer is survivable
            # because reads are cursor-based above this layer.
            pass

    def close(self) -> None:
        """Close the port, ignoring errors (called from teardown paths)."""
        handle, self._serial = self._serial, None
        if handle is None:
            return
        try:
            handle.close()
        except Exception:
            pass

    def __enter__(self) -> "SerialTransport":
        self.open()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    # -- io ------------------------------------------------------------------

    def read(self, timeout: Optional[float] = None) -> str:
        """Read whatever is available within ``timeout``; may return ``""``."""
        handle = self._require_open()
        if timeout is not None and timeout != self.read_timeout:
            handle.timeout = timeout
        try:
            data = handle.read(4096)
        except Exception as exc:
            raise TransportError(f'Read from "{self.port}" failed: {exc}') from exc
        finally:
            if timeout is not None and timeout != self.read_timeout:
                handle.timeout = self.read_timeout

        if not data:
            return ""
        return data.decode("utf-8", errors="replace")

    def write(self, text: str) -> None:
        """Send ``text`` verbatim; the caller owns any terminator."""
        handle = self._require_open()
        try:
            handle.write(text.encode("utf-8"))
            handle.flush()
        except Exception as exc:
            raise TransportError(f'Write to "{self.port}" failed: {exc}') from exc

    def send_line(self, line: str) -> None:
        """Send ``line`` followed by CRLF, as a shell would receive it."""
        self.write(line + "\r\n")

    def drain(self, duration: float = 0.2) -> str:
        """Read for ``duration`` and return everything collected."""
        collected = []
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            chunk = self.read(min(self.read_timeout, max(0.0, deadline - time.monotonic())))
            if chunk:
                collected.append(chunk)
        return "".join(collected)

    def _require_open(self):
        if self._serial is None:
            raise TransportError(
                f'Serial port "{self.port}" is not open; call open() first'
            )
        return self._serial
