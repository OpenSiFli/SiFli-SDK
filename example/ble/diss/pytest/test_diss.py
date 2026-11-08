# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end peer path for the diss example.

The DUT is a GATT server exposing the shared ``sifli_app`` service. diss answers read/write and, once the CCCD is enabled, notifies the
4-byte value from a 5-second timer.
"""

from __future__ import annotations

import time

import pytest

from sdk_test.suite_helpers import sifli_app

#: Device Information Service and its Manufacturer Name String, little-endian.
DIS_SERVICE_UUID = bytes([0x0A, 0x18])
MANUFACTURER_NAME_UUID = bytes([0x29, 0x2A])

#: Device Information Service: manufacturer name, model number, serial number,
#: hardware revision, firmware revision, software revision, system id, IEEE
#: regulatory certificate and PnP id.
DIS_CHARACTERISTIC_COUNT = 9

STATE: dict = {}


def _require(key: str):
    value = STATE.get(key)
    if value is None:
        pytest.skip(f"{key} was not set: an earlier test did not complete")
    return value


def test_dut_boots_and_advertises(dut):
    # diss uses SIBLES_ADV_CONNECT_MODE, whose running mode is 0. It also
    # brings up the classic stack (SPP), which must be ready before connecting.
    sifli_app.expect_ready(dut, expected_mode=0)
    sifli_app.expect_classic_ready(dut)


def test_peer_instrument_is_ready(bt_peer):
    # The instrument is shared lab equipment; an earlier case may have left it
    # connected or scanning.
    bt_peer.ble.reset()


def test_peer_connects_to_the_dut(dut, bt_peer):
    STATE["link"] = sifli_app.connect_to_dut(dut, bt_peer)


def test_the_device_information_service_is_present_and_enumerable(bt_peer):
    """The README's second expectation: a client can reach the DIS.

    Its values are not read here: on this build the DIS answers only over an
    authenticated link (a plain read is ATT 0x0F, an unauthenticated encrypted
    one is ATT 0x05), and MITM needs the passkey flow the instrument cannot
    drive yet -- recorded in BT_PEER_DESIGN.md 11.1 / 13.2.1.
    """
    service = bt_peer.ble.discover(_require("link"), DIS_SERVICE_UUID)

    assert len(service.characteristics) == DIS_CHARACTERISTIC_COUNT, (
        "a Device Information Service should hold the nine standard "
        f"characteristics, found {len(service.characteristics)}"
    )
    # Selecting by UUID is what proves the profile's identifiers arrived intact.
    characteristic = service.characteristic(MANUFACTURER_NAME_UUID)
    assert characteristic.value_handle > characteristic.handle


def test_peer_discovers_the_service(bt_peer):
    STATE["service"] = sifli_app.discover_service(bt_peer, _require("link"))


def test_peer_writes_and_reads_the_value(dut, bt_peer):
    sifli_app.write_then_read(
        dut, bt_peer, _require("link"), _require("service")
    )


def test_peer_subscribes_to_notifications(dut, bt_peer):
    sifli_app.subscribe(dut, bt_peer, _require("link"), _require("service"))


def test_the_cccd_timer_notifies_every_five_seconds(dut, bt_peer):
    """Enabling the CCCD starts the example's 5-second timer.

    The timer sends the 4-byte value (`BLE_APP_TIMEOUT_INTERVAL` = 5000 ms);
    nothing else in this example would notice it never being started.
    """
    link = _require("link")
    service = _require("service")

    start = bt_peer.console.last_seq
    deadline = time.monotonic() + 12.0
    payload = None
    while payload is None and time.monotonic() < deadline:
        packet = sifli_app.wait_notification(
            bt_peer,
            link,
            service,
            since_seq=start,
            timeout=max(0.1, deadline - time.monotonic()),
        )
        if len(packet) == 4:
            payload = packet

    assert payload is not None, (
        "the CCCD timer produced no 4-byte notification within 12s"
    )
