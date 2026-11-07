# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end peer path for the pair example.

The DUT is a GAP peripheral that accepts pairing. The instrument connects,
pairs, and the DUT's own pairing line is the evidence that *it* completed the
procedure; a GATT read afterwards shows the link still carries traffic.
"""

from __future__ import annotations

from sdk_test.instruments import ble_server

STATE: dict = {}


def _require(key: str):
    value = STATE.get(key)
    if value is None:
        raise AssertionError(f"{key} was not set: an earlier test did not complete")
    return value


def test_dut_boots_and_advertises(dut):
    ble_server.expect_ready(dut, expected_mode=0)


def test_peer_instrument_is_ready(bt_peer):
    bt_peer.ble.reset()


def test_peer_connects_to_the_dut(dut, bt_peer):
    STATE["link"] = ble_server.connect_to_dut(dut, bt_peer)


def test_peer_pairs_with_the_dut(dut, bt_peer):
    """Pairing completes on both ends, not just on the instrument.

    Legacy pairing is asked for explicitly: both ends only need an encrypted
    link, and Secure Connections pairing fails on this bench with a DHKey check
    error (the HID suite carries the same note).
    """
    bt_peer.ble.bond(_require("link"), auth="legacy")

    # The connection manager logs this from the DUT's own state machine, so a
    # peer that paired with some other board cannot produce it.
    dut.expect(r"GAPC_PAIRING_SUCCEED", timeout=10)


def test_the_encrypted_link_still_carries_gatt(dut, bt_peer):
    service = ble_server.discover_service(bt_peer, _require("link"))

    payload = bt_peer.ble.read(_require("link"), service.value_handle())
    # The example serves a uint32, so a read is four bytes little-endian.
    assert len(payload) == 4, (
        f"the example serves a 4-byte value, got {len(payload)}: {payload.hex()}"
    )

    # A write lands on the DUT (it logs the new value) and reads back.
    ble_server.write_then_read(dut, bt_peer, _require("link"), service)
