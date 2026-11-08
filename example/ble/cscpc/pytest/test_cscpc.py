# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end peer path for the cscpc example.

The DUT is a GATT server exposing the shared ``sifli_app`` service. cscpc has no autonomous notification source: its profile client runs
against a remote device, so the suite stops at "subscribe accepted"
rather than waiting for a packet this device never sends.
"""

from __future__ import annotations

import pytest

from sdk_test.suite_helpers import sifli_app

STATE: dict = {}


def _require(key: str):
    value = STATE.get(key)
    if value is None:
        pytest.skip(f"{key} was not set: an earlier test did not complete")
    return value


def test_dut_boots_and_advertises(dut):
    # cscpc uses SIBLES_ADV_CONNECT_MODE, whose running mode is 0. It also
    # brings up the classic stack (SPP), which must be ready before connecting.
    sifli_app.expect_ready(dut, expected_mode=0)
    sifli_app.expect_classic_ready(dut)


def test_peer_instrument_is_ready(bt_peer):
    # The instrument is shared lab equipment; an earlier case may have left it
    # connected or scanning.
    bt_peer.ble.reset()


def test_peer_connects_to_the_dut(dut, bt_peer):
    STATE["link"] = sifli_app.connect_to_dut(dut, bt_peer)


def test_peer_discovers_the_service(bt_peer):
    STATE["service"] = sifli_app.discover_service(bt_peer, _require("link"))


def test_peer_writes_the_value(dut, bt_peer):
    """The write arrives; it is not read back because cscpc does not store it.

    Its callback logs the bytes as ``BLE RX`` and has no memcpy, so the served
    value never changes -- asserting a read-back here would encode that as if
    it were intended.
    """
    sifli_app.write_only(
        dut, bt_peer, _require("link"), _require("service")
    )


def test_peer_reads_the_value(bt_peer):
    value = bt_peer.ble.read(_require("link"), _require("service").value_handle())

    assert len(value) == 4, f"the example serves a uint32, got {value.hex()}"


def test_peer_subscribes_to_notifications(dut, bt_peer):
    sifli_app.subscribe(dut, bt_peer, _require("link"), _require("service"))
