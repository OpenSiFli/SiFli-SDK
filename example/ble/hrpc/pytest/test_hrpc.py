# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end peer path for the hrpc example.

The DUT is a GATT server exposing the shared ``sifli_app`` service; the
``bt_peer:ble`` instrument drives the client half. The tests run in order
because ``dut`` and ``bt_peer`` are session-scoped and their cursors only move
forward.

hrpc has no autonomous notification source: it answers read/write and the CCCD.
The suite therefore stops at "subscribe accepted" instead of waiting for a
notification it would never receive.
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
    # hrpc uses SIBLES_ADV_CONNECT_MODE, whose running mode is 0.
    sifli_app.expect_ready(dut, expected_mode=0)
    # hrpc also brings up the classic stack (SPP server); its page-scan setup
    # runs for ~20 s after the BLE advertisement. Connecting before it is done
    # ends in reason 62 (supervision timeout) even though the peer's initiator
    # reports a link, so wait for the readiness line first.
    dut.expect(r"bt stack ready", timeout=40)


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
