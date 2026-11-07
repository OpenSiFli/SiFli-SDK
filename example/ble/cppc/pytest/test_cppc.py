# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end peer path for the cppc example.

The DUT is a GATT server exposing the shared ``sifli_app`` service. cppc has no
autonomous notification source: it answers read/write and the CCCD, so the
suite stops at "subscribe accepted".
"""

from __future__ import annotations

import pytest

from sdk_test.instruments import ble_server

STATE: dict = {}


def _require(key: str):
    value = STATE.get(key)
    if value is None:
        pytest.skip(f"{key} was not set: an earlier test did not complete")
    return value


def test_dut_boots_and_advertises(dut):
    # cppc uses SIBLES_ADV_CONNECT_MODE, whose running mode is 0. It also
    # brings up the classic stack (SPP), which must be ready before connecting.
    ble_server.expect_ready(dut, expected_mode=0)
    ble_server.expect_classic_ready(dut)


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
