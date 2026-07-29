# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""SiFli UART Debug Access Protocol — lightweight Python implementation.

Talks the SiFli UART DEBUG IP binary protocol directly over a serial port,
avoiding probe-rs session boundaries so that core-switch register writes and
memory reads happen within a single connection.

Frame format (SF32LB52 / SF32LB56)::

    ┌─────────┬──────────────────────┬─────────────────┐
    │ 2 bytes │  header (4 or 10 B)  │    payload      │
    ├─────────┼──────────────────────┼─────────────────┤
    │ 7E 79   │ len:u16 marker:u16   │ command/response│
    └─────────┴──────────────────────┴─────────────────┘

Reference:
  probe-rs SiFli fork – probe-rs/src/probe/sifliuart/mod.rs
  https://github.com/OpenSiFli/probe-rs
"""

from __future__ import annotations

import struct
import time
from typing import Dict, Optional

_START_WORD: bytes = b"\x7e\x79"
_BAUD: int = 1_000_000
_TIMEOUT: float = 3.0
_READ_CHUNK: int = 32 * 1024  # bytes per MEMRead (<= 65535 words)
_FRAME_MARKER: bytes = b"\x10\x00"

# ── SF32LB52 address mapping ──────────────────────────────────────────
# The UART DEBUG IP uses a different physical address map than the CPU.
# These remappings match the probe-rs SiFliUartChipProfile::map_address logic.


def _map_address_sf32lb52(addr: int) -> int:
    if 0x0000_0000 <= addr <= 0x0000_FFFF:
        return 0xA000_0000 + addr
    if 0xE000_0000 <= addr <= 0xEFFF_FFFF:
        return (addr & 0x0FFF_FFFF) | 0xF000_0000
    if 0x1000_0000 <= addr <= 0x1FFF_FFFF:
        return (addr & 0x0FFF_FFFF) | 0x6000_0000
    return addr


def _map_address_sf32lb56(addr: int) -> int:
    if 0xE000_0000 <= addr < 0xF000_0000:
        return (addr & 0x0FFF_FFFF) | 0xF000_0000
    if 0x0040_0000 <= addr <= 0x0041_FFFF:
        return addr + 0x2000_0000
    if 0x20C0_0000 <= addr <= 0x20C1_FFFF:
        return addr - 0x0080_0000
    if 0x2000_0000 <= addr <= 0x200C_7FFF:
        return addr + 0x0A00_0000
    if 0x2080_0000 <= addr <= 0x20BF_FFFF:
        return addr - 0x2080_0000
    if 0x1000_0000 <= addr <= 0x1FFF_FFFF:
        return addr + 0x5000_0000
    return addr


_MAP_ADDRESS = {
    "SF32LB52": _map_address_sf32lb52,
    "SF32LB56": _map_address_sf32lb56,
}


class SifliUartDap:
    """Single-connection SiFli UART debug session.

    Usage::

        with SifliUartDap("/dev/cu.wchusbserial...", chip="SF32LB52") as dap:
            dap.write32(0x5000B008, 1)   # switch to LCPU
            dap.write32(0x5000B008, 0)   # switch to HCPU
            data = dap.read(0x20000000, 512 * 1024)
    """

    def __init__(self, port: str, chip: str = "SF32LB52") -> None:
        self._port_path = port
        self._chip = chip.upper()
        if self._chip not in _MAP_ADDRESS:
            raise ValueError(f"UART DAP does not support chip: {chip}")
        self._map = _MAP_ADDRESS[self._chip]
        self._ser: Optional[serial.Serial] = None

    # ── context manager ───────────────────────────────────────────────

    def __enter__(self) -> "SifliUartDap":
        self.open()
        return self

    def __exit__(self, *args: object) -> None:
        self.close()

    # ── open / close ──────────────────────────────────────────────────

    def open(self) -> None:
        if self._ser is not None:
            return
        import serial  # lazy – pyserial is an optional dependency
        self._ser = serial.Serial(
            self._port_path,
            baudrate=_BAUD,
            timeout=_TIMEOUT,
            write_timeout=_TIMEOUT,
        )
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        self._send_command(b"\x41\x54\x53\x46\x33\x32\x05\x21")  # Enter
        resp = self._read_response()
        if resp is None or resp[0] != 0xD1:
            raise IOError(f"SiFli UART Enter failed on {self._port_path}")

    def close(self) -> None:
        if self._ser is None:
            return
        try:
            self._send_command(b"\x41\x54\x53\x46\x33\x32\x18\x21")  # Exit
        except Exception:
            pass
        self._ser.close()
        self._ser = None

    # ── CPU control ───────────────────────────────────────────────────

    _DHCSR_ADDR: int = 0xE000EDF0  # Debug Halting Control and Status Register
    _DCRSR_ADDR: int = 0xE000EDF4  # Debug Core Register Selector Register
    _DCRDR_ADDR: int = 0xE000EDF8  # Debug Core Register Data Register
    _DHCSR_DBGKEY: int = 0xA05F0000
    _DHCSR_C_DEBUGEN: int = 1 << 0
    _DHCSR_C_HALT: int = 1 << 1
    _DHCSR_S_HALT: int = 1 << 17
    _DHCSR_S_REGRDY: int = 1 << 16

    def halt(self) -> None:
        """Halt the CPU core via DHCSR.

        Writes the debug key + C_DEBUGEN + C_HALT to DHCSR, then
        spins until S_HALT is set (or 100ms timeout).

        Must be called after the core-switch register has been configured,
        so that the write targets the correct core.
        """
        value = self._DHCSR_DBGKEY | self._DHCSR_C_DEBUGEN | self._DHCSR_C_HALT
        self.write32(self._DHCSR_ADDR, value)
        deadline = time.monotonic() + 0.1
        while time.monotonic() < deadline:
            resp = self._memread(self._DHCSR_ADDR, 4)
            if len(resp) >= 4:
                dhcsr = struct.unpack("<I", resp[:4])[0]
                if dhcsr & self._DHCSR_S_HALT:
                    return
            time.sleep(0.01)
        # Don't raise — the CPU may already be stopped (e.g. in a fault
        # handler); the write alone is often enough to quiet the bus.

    def read_core_registers(self) -> Dict[str, int]:
        registers = {f"r{i}": self._read_core_register(i) for i in range(13)}
        registers["sp"] = self._read_core_register(13)
        registers["lr"] = self._read_core_register(14)
        registers["pc"] = self._read_core_register(15)
        registers["xpsr"] = self._read_core_register(16)
        return registers

    # ── memory access ─────────────────────────────────────────────────

    def write32(self, addr: int, value: int) -> None:
        """Write a single 32-bit word to *addr*."""
        payload = bytearray()
        payload.extend(b"\x40\x77")  # MEMWrite
        payload.extend(struct.pack("<I", self._map(addr)))
        payload.extend(struct.pack("<H", 1))  # 1 word
        payload.extend(struct.pack("<I", value))
        self._send_command(bytes(payload))
        resp = self._read_response()
        if resp is None or resp[0] != 0xD3:
            raise IOError(f"MEMWrite 0x{addr:08X} failed")

    def read(self, addr: int, size: int) -> bytes:
        """Read *size* bytes starting from *addr*."""
        result = bytearray()
        for offset in range(0, size, _READ_CHUNK):
            chunk = min(_READ_CHUNK, size - offset)
            data = self._memread(addr + offset, chunk)
            result.extend(data)
        return bytes(result)

    # ── internals ─────────────────────────────────────────────────────

    def _read_core_register(self, register: int) -> int:
        self.write32(self._DCRSR_ADDR, register)
        deadline = time.monotonic() + 0.1
        while time.monotonic() < deadline:
            ready = self._memread(self._DHCSR_ADDR, 4)
            if len(ready) >= 4 and struct.unpack("<I", ready[:4])[0] & self._DHCSR_S_REGRDY:
                data = self._memread(self._DCRDR_ADDR, 4)
                if len(data) >= 4:
                    return struct.unpack("<I", data[:4])[0]
            time.sleep(0.01)
        raise IOError(f"Core register r{register} read timed out")

    def _memread(self, addr: int, size: int) -> bytes:
        """Issue a single MEMRead command (max 65535 words ≈ 256 KB)."""
        words = size // 4
        if size % 4 != 0:
            # Round up – the HW always returns word-aligned data.
            words += 1
        if words > 65535:
            raise ValueError(f"MEMRead word count {words} exceeds u16 max")

        mapped = self._map(addr)
        payload = bytearray()
        payload.extend(b"\x40\x72")  # MEMRead
        payload.extend(struct.pack("<I", mapped))
        payload.extend(struct.pack("<H", words))
        self._send_command(bytes(payload))

        resp = self._read_response()
        if resp is None:
            raise IOError(f"MEMRead 0x{addr:08X} timed out")
        if len(resp) < 2 or resp[0] != 0xD2:
            raise IOError(f"MEMRead 0x{addr:08X}: unexpected response type 0x{resp[0]:02X}")
        # Strip 0xD2 header and 0x06 terminator; return raw LE bytes.
        data = resp[1:-1]
        return data[:size]  # trim alignment padding

    def _send_command(self, payload: bytes) -> None:
        assert self._ser is not None
        self._ser.reset_input_buffer()
        frame = _START_WORD
        frame += struct.pack("<H", len(payload))
        frame += _FRAME_MARKER
        frame += payload
        self._ser.write(frame)
        self._ser.flush()

    def _read_response(self) -> Optional[bytes]:
        """Read and decode one debug frame from the serial port.

        Returns the raw payload bytes on success, or None on timeout.
        Non-debug frames (console output) are silently skipped.
        """
        assert self._ser is not None
        deadline = time.monotonic() + _TIMEOUT

        while time.monotonic() < deadline:
            # --- scan for start word -----------------------------------
            b = self._ser.read(1)
            if not b:
                continue
            if b[0] != 0x7E:
                continue
            b2 = self._ser.read(1)
            if not b2 or b2[0] != 0x79:
                continue

            # --- read header (4 bytes for SF32LB52) --------------------
            header = self._ser.read(4)
            if len(header) < 4:
                continue
            payload_len = struct.unpack("<H", header[:2])[0]
            is_debug = header[2:4] == _FRAME_MARKER
            if payload_len == 0:
                continue

            # --- read payload ------------------------------------------
            payload = self._ser.read(payload_len)
            if len(payload) < payload_len:
                continue

            if is_debug and len(payload) > 0 and payload[-1] == 0x06:
                return payload

        return None  # timeout


def normalize_chip_for_dap(chip: str) -> str:
    """Extract chip family from a chip string (e.g. 'SF32LB52' from 'SF32LB52X')."""
    import re
    match = re.search(r"(52|55|56|57|58)", chip.upper())
    if not match:
        raise ValueError(f"Unsupported chip: {chip}")
    return f"SF32LB{match.group(1)}"


def extract_probe_port(probe_spec: str) -> str:
    """Extract the serial-port path from a probe-rs --probe string.

    >>> extract_probe_port("1a86:55d3-1:/dev/cu.wchusbserial58CD1576891")
    '/dev/cu.wchusbserial58CD1576891'
    """
    # probe-rs format: VID:PID-index:serial_path
    if ":/dev/" in probe_spec or ":/dev/" in probe_spec.replace("\\", "/"):
        return probe_spec.split(":", 2)[-1]
    # plain serial path
    if probe_spec.startswith("/dev/"):
        return probe_spec
    raise ValueError(f"Cannot extract serial port from probe spec: {probe_spec}")
