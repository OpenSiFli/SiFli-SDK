# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end peer path for the BLE half of the combined example.

`src/main.c` runs the shared "sifli_app" GATT server on legacy connectable
advertising, so this suite reuses the peripheral example's steps: the
`bt_peer:ble` instrument connects, discovers the service, writes, reads,
subscribes, and receives both the CCCD timer's packets and the burst the
0x11 trigger starts.

The example also compiles a Classic application module under `app_bt/` into
the same firmware; that half is out of scope here.

The example's notification burst differs from `example/ble/peripheral`: it
passes the written word to the send thread, which unpacks the packet count
from bits 16-31 and the payload length from bits 8-15 (`src/main.c:329-350`).
The 4-byte trigger below therefore selects twenty 10-byte packets; a bare
0x11 would satisfy the trigger but ask for zero packets.
"""

from __future__ import annotations

import time

from sdk_test.suite_helpers import sifli_app
from sdk_test.instruments.errors import InstrumentTimeout

#: Low byte 0x11 starts the burst, 0x0A is the payload length and 0x14 the
#: packet count. The wire writes the word little-endian.
BURST_WRITE = bytes([0x11, 0x0A, 0x14, 0x00])
BURST_COUNT = 20
BURST_LENGTH = 10

STATE: dict = {}


def _require(key: str):
    value = STATE.get(key)
    if value is None:
        raise AssertionError(f"{key} was not set: an earlier test did not complete")
    return value


def test_dut_boots_and_advertises(dut):
    # This example does not define APP_ENABLE_BG_ADV, so the only advertising
    # line is the connectable one (running mode 0).
    sifli_app.expect_ready(dut, expected_mode=0)


def test_peer_instrument_is_ready(bt_peer):
    bt_peer.ble.reset()


def test_peer_connects_to_the_dut(dut, bt_peer):
    STATE["link"] = sifli_app.connect_to_dut(dut, bt_peer)


def test_peer_bonds_with_the_dut(dut, bt_peer):
    """The value and CCCD carry PERM(WP, UNAUTH): writes need encryption.

    Unlike `example/ble/peripheral`, this example's attribute table marks both
    writable attributes as requiring an unauthenticated (encrypted) link, so a
    write before bonding is refused with ATT 0x0F. Legacy pairing is enough for
    the permission this example declares.
    """
    bt_peer.ble.bond(_require("link"), auth="legacy")
    dut.expect(r"GAPC_PAIRING_SUCCEED", timeout=10)


def test_peer_discovers_the_service(bt_peer):
    STATE["service"] = sifli_app.discover_service(bt_peer, _require("link"))


def test_peer_writes_and_reads_the_value(dut, bt_peer):
    sifli_app.write_then_read(dut, bt_peer, _require("link"), _require("service"))


def test_peer_subscribes_to_notifications(dut, bt_peer):
    sifli_app.subscribe(dut, bt_peer, _require("link"), _require("service"))
    STATE["subscribed"] = True


def test_the_cccd_timer_notifies_every_five_seconds(dut, bt_peer):
    """The example's periodic 4-byte notification.

    The period is `BLE_APP_TIMEOUT_INTERVAL` = 5000 ms. The burst test below
    skips these packets by length, so this is the only test that notices the
    timer never being started.
    """
    link = _require("link")
    service = _require("service")
    _require("subscribed")

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


def test_notification_burst_reaches_the_peer(dut, bt_peer):
    """The packed trigger starts the example's 20-packet notification loop.

    The CCCD timer also sends 4-byte packets; they are skipped by length, and
    ``since_seq`` keeps a packet from before the trigger out of the burst.
    """
    link = _require("link")
    service = _require("service")
    _require("subscribed")

    start = bt_peer.console.last_seq
    bt_peer.ble.write(link, service.value_handle(), BURST_WRITE)
    dut.expect(r"example of ble notification", timeout=10)

    deadline = time.monotonic() + 15.0
    payloads = []
    while len(payloads) < BURST_COUNT and time.monotonic() < deadline:
        try:
            payload = sifli_app.wait_notification(
                bt_peer,
                link,
                service,
                since_seq=start,
                timeout=max(0.1, deadline - time.monotonic()),
            )
        except InstrumentTimeout:
            break
        if len(payload) == BURST_LENGTH:
            payloads.append(payload)

    assert len(payloads) == BURST_COUNT, f"got {len(payloads)} burst packets"
    for index, payload in enumerate(payloads):
        assert payload == bytes([index]) * BURST_LENGTH, (
            f"burst packet {index} carried {payload.hex()}"
        )
