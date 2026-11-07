# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Shared steps for the BLE examples that expose the ``sifli_app`` service.

Most peripheral examples use the same 128-bit service and the same shell-side
observability (``receive BLE power on!``, ``ADV start resutl``, ``Peer device
(...) connected``, ``Updated app value``, ``CCCD 1``). Keeping those steps here
means a per-example suite is mostly its own expectations, not a copy of the
connect/discover boilerplate.

The suite is still responsible for what it asserts: an example that does not
notify must not call :func:`wait_notification`, because a timeout would look
like a broken instrument rather than a property the example does not have.
"""

from __future__ import annotations

import re
from typing import Any

from .errors import InstrumentProtocolError
from .errors import InstrumentTimeout

#: The shared service UUID of the peripheral examples: "sifli_app".
SIFLI_APP_UUID = bytes(
    [0x73, 0x69, 0x66, 0x6C, 0x69, 0x5F, 0x61, 0x70, 0x70] + [0x00] * 7
)

#: The prompt is ``msh />`` when DFS keeps a working directory and ``msh >``
#: when it does not. Accept both rather than pinning one carrier's spelling.
PROMPT = r"msh [^>\n]*>"

#: A value the example echoes back on read; 0x22 avoids the special 0x11 trigger
#: some examples use to start a notification burst.
DEFAULT_VALUE = bytes([0x22, 0x00, 0x00, 0x00])

#: The name prefix the peripheral examples advertise. The full name carries the
#: board address, so selecting by it is deterministic where RSSI is not.
SIFLI_APP_NAME_PREFIX = "SIFLI_APP-"


def advertised_name(data: bytes) -> str:
    """Pull the complete/shortened local name out of raw AD structures."""
    position = 0
    while position + 1 < len(data):
        length = data[position]
        if length == 0:
            break
        end = position + 1 + length
        if end > len(data):
            break
        ad_type = data[position + 1]
        if ad_type in (0x08, 0x09):  # shortened / complete local name
            return data[position + 2:end].decode("utf-8", "replace")
        position = end
    return ""


def expect_ready(dut: Any, *, expected_mode: int | None = None) -> None:
    """The board booted, powered the stack and is advertising.

    ``expected_mode`` is the ``sibles_adv_running_mode_t`` value: 0 for
    ``SIBLES_ADV_CONNECT_MODE``, 5 for ``SIBLES_ADV_EXTENDED_MODE``. ``None``
    accepts any mode (an example that legitimately runs more than one).
    """
    dut.expect("- SiFli Corporation", timeout=30)
    dut.expect(PROMPT, timeout=20)
    dut.expect(r"receive BLE power on!", timeout=30)

    # `peripheral`/`file_transfer` also log `Broadcast ADV start resutl`; an
    # unanchored pattern would validate that line instead of the connect one.
    line = dut.expect(
        r"(?<!Broadcast )ADV start resutl \d+, mode \d+", timeout=30
    )
    match = re.search(r"ADV start resutl (\d+), mode (\d+)", line)
    assert match is not None  # the pattern above guarantees this
    assert int(match.group(1)) == 0, "advertising failed to start"
    if expected_mode is not None:
        assert int(match.group(2)) == expected_mode, (
            f"advertising runs in mode {match.group(2)}, expected {expected_mode}"
        )


def expect_classic_ready(dut: Any, timeout: float = 40.0) -> None:
    """Wait for the classic stack (SPP server) the profile examples also start.

    Their BLE advertisement is up long before the classic page scan finishes;
    connecting in that window ends in HCI reason 62 (supervision timeout) even
    though the initiator reported a link.
    """
    dut.expect(r"bt stack ready", timeout=timeout)


def connect_to_dut(dut: Any, peer: Any, *, attempts: int = 3) -> Any:
    """Scan, connect to a connectable report, and confirm the DUT saw the link.

    A dense lab has more than one connectable advertiser in range, so the
    strongest report is only a guess. The DUT-side ``connected`` line is the
    proof; when it does not appear, the link belongs to a neighbour -- drop it
    and try the next report, public addresses first.
    """
    reports = peer.ble.scan(duration_ms=3000)
    # A peripheral example may also run a non-connectable background
    # advertisement from a generated random address.
    connectable = [report for report in reports if report.connectable]
    if not connectable:
        raise AssertionError(
            "the scan saw no connectable advertisement (%d report(s)); "
            "check that the DUT is advertising connectably" % len(reports)
        )

    # The instrument is the central here, so *it* selects the DUT. The examples
    # advertise "SIFLI_APP-<address>"; matching that name is deterministic where
    # "strongest" is not. Fall back to every connectable report when no name was
    # seen (an example that advertises no name, or a truncated report).
    named = [
        report
        for report in connectable
        if advertised_name(report.data).startswith(SIFLI_APP_NAME_PREFIX)
    ]
    candidates = named or connectable
    ordered = sorted(
        candidates,
        key=lambda report: (
            report.address_type != "public",
            -(report.rssi if report.rssi is not None else -(2 ** 31)),
        ),
    )

    for report in ordered[:attempts]:
        try:
            link = peer.ble.connect(report, timeout=10.0)
        except (InstrumentTimeout, InstrumentProtocolError):
            continue
        try:
            dut.expect(r"Peer device\(.*\) connected", timeout=8)
            return link
        except AssertionError:
            # The link is up, but not to this DUT. Release it and try the next.
            try:
                peer.ble.disconnect(link)
            except (InstrumentTimeout, InstrumentProtocolError):
                pass

    raise AssertionError(
        "could not establish a link the DUT acknowledged after "
        f"{attempts} attempt(s); scanned {len(connectable)} connectable report(s)"
    )


def discover_service(peer: Any, link: Any) -> Any:
    service = peer.ble.discover(link, SIFLI_APP_UUID)
    characteristic = service.characteristics[0]
    # `discover` already rejects an empty listing; these check the handle
    # arithmetic a GATT client depends on.
    assert service.value_handle() > characteristic.handle, (
        "the value handle must follow its declaration handle"
    )
    assert service.cccd_handle() > service.value_handle(), (
        "the CCCD must follow the value handle"
    )
    return service


def write_then_read(
    dut: Any,
    peer: Any,
    link: Any,
    service: Any,
    payload: bytes = DEFAULT_VALUE,
    *,
    expect_log: bool = True,
    verify_readback: bool = True,
) -> None:
    """Write the value characteristic, check the DUT, then read it back.

    ``verify_readback=False`` is for servers that accept a write but serve a
    fixed value (``cscpc`` today): the write and its log are still checked, the
    read is not compared with the payload.
    """
    handle = service.value_handle()
    peer.ble.write(link, handle, payload)
    if expect_log:
        # The examples store the written bytes into a uint32 and log it as a
        # decimal; matching the value keeps "logged but dropped" from passing.
        expected = int.from_bytes(payload[:4].ljust(4, b"\x00"), "little")
        dut.expect(rf"Updated app value from:\d+ to:{expected}\b", timeout=10)
    if verify_readback:
        assert peer.ble.read(link, handle) == payload


def write_only(
    dut: Any,
    peer: Any,
    link: Any,
    service: Any,
    payload: bytes = DEFAULT_VALUE,
    *,
    log: str = r"BLE RX",
) -> None:
    """Write without comparing the read-back.

    ``cscpc`` accepts a write, logs the bytes as ``BLE RX`` and keeps serving a
    fixed value, so the write log is the observable, not the read.
    """
    peer.ble.write(link, service.value_handle(), payload)
    dut.expect(log, timeout=10)


def subscribe(dut: Any, peer: Any, link: Any, service: Any) -> None:
    peer.ble.subscribe(link, service.cccd_handle())
    dut.expect(r"CCCD 1", timeout=10)


def wait_notification(
    peer: Any,
    link: Any,
    service: Any,
    *,
    since_seq: int | None = None,
    timeout: float = 15.0,
) -> bytes:
    return peer.ble.wait_notification(
        service.value_handle(), link=link, since_seq=since_seq, timeout=timeout
    )


def disconnect(peer: Any, link: Any) -> None:
    peer.ble.disconnect(link)
