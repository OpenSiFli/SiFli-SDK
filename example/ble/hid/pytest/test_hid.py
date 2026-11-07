# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end peer path for the hid example.

The DUT is a HID device: the standard 16-bit 0x1812 service with a report
characteristic that supports notifications. The instrument discovers it by the
16-bit UUID, subscribes to the report's CCCD and waits for the report produced
by the example's own `test_hids` shell command.
"""

from __future__ import annotations

import pytest

from sdk_test.instruments import ble_server
from sdk_test.instruments.bt_peer import CCCD_UUID

#: HID service, little-endian on the wire.
HID_SERVICE_UUID = bytes([0x12, 0x18])

STATE: dict = {}


def _require(key: str):
    value = STATE.get(key)
    if value is None:
        pytest.skip(f"{key} was not set: an earlier test did not complete")
    return value


def test_dut_boots_and_advertises(dut):
    # The main advertisement is legacy connectable (mode 0); the example also
    # runs a broadcast background set, which the helper's guard skips.
    ble_server.expect_ready(dut, expected_mode=0)


def test_peer_instrument_is_ready(bt_peer):
    bt_peer.ble.reset()


def test_peer_connects_to_the_dut(dut, bt_peer):
    STATE["link"] = ble_server.connect_to_dut(dut, bt_peer)


def test_peer_pairs_with_the_dut(bt_peer):
    # The HID report's CCCD is write-protected with a UNAUTH requirement, so a
    # plain-text write is rejected with ATT 0x05 until the link is encrypted.
    # Security level UNAUTH is what it asks for, so legacy pairing is enough;
    # Secure Connections pairing fails on this bench with a DHKey check error.
    bt_peer.ble.bond(_require("link"), auth="legacy")


def test_peer_discovers_the_hid_service(bt_peer):
    service = bt_peer.ble.discover(_require("link"), HID_SERVICE_UUID)
    # The report characteristic is the one that can notify, so it is the one
    # carrying a CCCD; it also has a report reference next to that CCCD, which
    # is why the CCCD must be picked by UUID and not by position.
    reports = [
        characteristic
        for characteristic in service.characteristics
        if CCCD_UUID in characteristic.descriptor_uuids
    ]
    assert reports, (
        "no HID report characteristic (a characteristic with a CCCD) in "
        f"{service.characteristics}"
    )
    STATE["report"] = reports[0]


def test_peer_subscribes_to_the_report(dut, bt_peer):
    report = _require("report")
    bt_peer.ble.subscribe(_require("link"), report.descriptor_handle(CCCD_UUID))
    dut.expect(r"CCCD 1", timeout=10)
    STATE["subscribed"] = True


def test_key_report_reaches_the_peer(dut, bt_peer):
    report = _require("report")
    _require("subscribed")

    start = bt_peer.console.last_seq
    dut.send_line("test_hids 4 p")
    dut.expect(r"key_report_send", timeout=10)

    payload = bt_peer.ble.wait_notification(
        report.value_handle, link=_require("link"), since_seq=start, timeout=15
    )
    # Boot keyboard report: control keys, reserved, then six key slots. The
    # README's "test_hids 4 p" types 'a', whose HID code is 0x04.
    assert len(payload) == 8, f"the keyboard report was {len(payload)} bytes: {payload.hex()}"
    assert payload[0] == 0, f"no modifier should be held: {payload.hex()}"
    assert payload[2] == 0x04, f"key slot 0 should hold 0x04: {payload.hex()}"
    assert payload[3:] == bytes(5), f"the other key slots should be empty: {payload.hex()}"


def test_release_clears_the_key(dut, bt_peer):
    """`test_hids 4 r` is the other half of the README's press/release pair."""
    report = _require("report")
    _require("subscribed")

    start = bt_peer.console.last_seq
    dut.send_line("test_hids 4 r")
    dut.expect(r"key_report_send", timeout=10)

    payload = bt_peer.ble.wait_notification(
        report.value_handle, link=_require("link"), since_seq=start, timeout=15
    )
    assert payload == bytes(8), f"the release report should be all zero: {payload.hex()}"
