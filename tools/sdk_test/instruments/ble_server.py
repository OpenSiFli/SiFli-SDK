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


def connect_to_dut(dut: Any, peer: Any) -> Any:
    """Scan, connect to the strongest report, and confirm the DUT saw the link.

    The strongest report is only a heuristic; the DUT-side ``connected`` line is
    what proves the peer latched onto this board rather than a neighbour.
    """
    reports = peer.ble.scan(duration_ms=3000)
    target = peer.ble.strongest(reports)
    link = peer.ble.connect(target)
    dut.expect(r"Peer device\(.*\) connected", timeout=30)
    return link


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


def subscribe(dut: Any, peer: Any, link: Any, service: Any) -> None:
    peer.ble.subscribe(link, service.cccd_handle())
    dut.expect(r"CCCD 1", timeout=10)


def disconnect(peer: Any, link: Any) -> None:
    peer.ble.disconnect(link)
