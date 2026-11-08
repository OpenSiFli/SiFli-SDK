# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end peer path for the basc example.

The DUT is a GATT server exposing the shared ``sifli_app`` service. basc has no
autonomous notification source: it answers read/write and the CCCD, so the
suite stops at "subscribe accepted" rather than waiting for a packet it never
sends.
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
    # basc uses SIBLES_ADV_CONNECT_MODE, whose running mode is 0. It also
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


def test_peer_writes_and_reads_the_value(dut, bt_peer):
    sifli_app.write_then_read(
        dut, bt_peer, _require("link"), _require("service")
    )


def test_peer_subscribes_to_notifications(dut, bt_peer):
    sifli_app.subscribe(dut, bt_peer, _require("link"), _require("service"))
