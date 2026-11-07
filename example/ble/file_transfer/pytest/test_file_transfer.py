# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end peer path for the file_transfer example.

The DUT is a GATT server on legacy connectable advertising. It does not bring
up the classic stack. Notifications come from a 5-second CCCD timer (4-byte
packets) and the 0x1111 burst trigger (20 packets of 10 bytes).
"""

from __future__ import annotations

import time

import pytest

from sdk_test.instruments import ble_server
from sdk_test.instruments.errors import InstrumentTimeout

STATE: dict = {}


def _require(key: str):
    value = STATE.get(key)
    if value is None:
        pytest.skip(f"{key} was not set: an earlier test did not complete")
    return value


def test_dut_boots_and_advertises(dut):
    # The example starts the legacy connectable advertisement (running mode 0);
    # it also logs a "Broadcast ADV start resutl" line, which the helper skips.
    ble_server.expect_ready(dut, expected_mode=0)


def test_peer_instrument_is_ready(bt_peer):
    bt_peer.ble.reset()


def test_peer_connects_to_the_dut(dut, bt_peer):
    STATE["link"] = ble_server.connect_to_dut(dut, bt_peer)


def test_peer_discovers_the_service(bt_peer):
    STATE["service"] = ble_server.discover_service(bt_peer, _require("link"))


def test_peer_writes_and_reads_the_value(dut, bt_peer):
    ble_server.write_then_read(
        dut, bt_peer, _require("link"), _require("service")
    )


def test_peer_subscribes_to_notifications(dut, bt_peer):
    ble_server.subscribe(dut, bt_peer, _require("link"), _require("service"))
    STATE["subscribed"] = True


def test_notification_burst_reaches_the_peer(dut, bt_peer):
    """The 0x1111 trigger starts the example's 20-packet notification loop.

    The CCCD timer also sends 4-byte packets; they are skipped by length, and
    ``since_seq`` keeps a packet from before the trigger out of the burst.
    """
    link = _require("link")
    service = _require("service")
    _require("subscribed")

    start = bt_peer.console.last_seq
    bt_peer.ble.write(link, service.value_handle(), b"\x11\x11")
    dut.expect(r"example of ble notification", timeout=10)

    deadline = time.monotonic() + 15.0
    payloads = []
    while len(payloads) < 20 and time.monotonic() < deadline:
        try:
            payload = ble_server.wait_notification(
                bt_peer,
                link,
                service,
                since_seq=start,
                timeout=max(0.1, deadline - time.monotonic()),
            )
        except InstrumentTimeout:
            break
        if len(payload) == 10:
            payloads.append(payload)

    assert len(payloads) == 20, f"got {len(payloads)} burst packets"
    for index, payload in enumerate(payloads):
        assert payload == bytes([index]) * 10, (
            f"burst packet {index} carried {payload.hex()}"
        )
