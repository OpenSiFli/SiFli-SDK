# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end peer path for the tip_cts example.

The DUT is a GATT server exposing the shared ``sifli_app`` service. tip_cts has no autonomous notification source of its own (its
profile client talks to a remote device), so the suite stops at
"subscribe accepted".
"""

from __future__ import annotations

import pytest

from sdk_test.suite_helpers import ble_server

#: Current Time Service and its Current Time characteristic, little-endian.
CTS_SERVICE_UUID = bytes([0x05, 0x18])
CURRENT_TIME_UUID = bytes([0x2B, 0x2A])

#: Year, month, day, hour, minute, second, day of week, fractions, reason.
CURRENT_TIME_LEN = 10

STATE: dict = {}


def _require(key: str):
    value = STATE.get(key)
    if value is None:
        pytest.skip(f"{key} was not set: an earlier test did not complete")
    return value


def test_dut_boots_and_advertises(dut):
    # tip_cts uses SIBLES_ADV_CONNECT_MODE, whose running mode is 0. It also
    # brings up the classic stack (SPP), which must be ready before connecting.
    ble_server.expect_ready(dut, expected_mode=0)
    ble_server.expect_classic_ready(dut)


def test_peer_instrument_is_ready(bt_peer):
    # The instrument is shared lab equipment; an earlier case may have left it
    # connected or scanning.
    bt_peer.ble.reset()


def test_peer_connects_to_the_dut(dut, bt_peer):
    STATE["link"] = ble_server.connect_to_dut(dut, bt_peer)


def test_the_current_time_service_serves_a_time(bt_peer):
    """The README's second and third expectations: visit CTS, read the time."""
    service = bt_peer.ble.discover(_require("link"), CTS_SERVICE_UUID)
    characteristic = service.characteristic(CURRENT_TIME_UUID)

    value = bt_peer.ble.read(_require("link"), characteristic.value_handle)

    assert len(value) == CURRENT_TIME_LEN, (
        f"the current time should be {CURRENT_TIME_LEN} bytes, got {value.hex()}"
    )
    year = value[0] | (value[1] << 8)
    assert 2000 <= year <= 2100, (
        f"the current time reports year {year}: {value.hex()}"
    )


def test_peer_discovers_the_service(bt_peer):
    STATE["service"] = ble_server.discover_service(bt_peer, _require("link"))


def test_peer_writes_and_reads_the_value(dut, bt_peer):
    ble_server.write_then_read(
        dut, bt_peer, _require("link"), _require("service")
    )


def test_peer_subscribes_to_notifications(dut, bt_peer):
    ble_server.subscribe(dut, bt_peer, _require("link"), _require("service"))
