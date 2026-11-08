# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of the BLE central-and-peripheral example against a peer.

Where the one-board suite (`../pytest/`) stops at what a single radio can show,
this suite drives the other half of the example: a second board, wired to the
same testbed and declared in the binding as the `bt_peer:ble` instrument, acts
as the GAP central and GATT client. Its console is the `bt_peer` fixture; the
board under test keeps `dut`.

The instrument is not built or flashed by this framework. The lab flashes it
once with the fixed image from `example/test/bt_peer` and leaves it running.
That firmware speaks the line protocol in `tools/sdk_test/BT_PEER_DESIGN.md`;
this suite never parses its log.

Which advertisement the peer connects to is deliberately not hard-coded: it
scans, takes the strongest report, and the DUT-side connection log is the
assertion. In a rack with many equally close boards that choice can go wrong,
and the test fails at the DUT rather than silently passing. Keeping the peer
next to its DUT is the lab's part of the contract.

Order matters: `dut` and `bt_peer` are session-scoped and their cursors only
move forward, so the tests run in the order defined here.
"""

from __future__ import annotations

import re

import pytest

from sdk_test.suite_helpers import sifli_app
from sdk_test.instruments.errors import InstrumentCommandError

#: The prompt is ``msh />`` when DFS keeps a working directory and ``msh >``
#: when it does not. Accept both rather than pinning one carrier's spelling.
PROMPT = r"msh [^>\n]*>"

#: The modes the stack reports in ``sibles_adv_evt_startted_t``; the example
#: always starts extended advertising (``USING_LE_EXTENSION_ADV`` in main.c).
ADV_MODE_EXTENDED = 5

#: The example's 128-bit service: "sifli_app" followed by zeros. The *suite*
#: knows this because it is testing this example; the instrument does not, and
#: receives the UUID with each discovery. That is the difference from the old
#: peer module, whose constant named one example for every BLE test.
SERVICE_UUID = bytes([0x73, 0x69, 0x66, 0x6C, 0x69, 0x5F, 0x61, 0x70, 0x70] + [0x00] * 7)

#: Filled by one test and read by the next: the link and the discovered handles
#: are runtime facts (the server assigns handles when it registers the service).
STATE: dict = {}

#: Written by the peer in the notification test. The example treats a write
#: whose low byte is 0x11 as "start notifying": the high 16 bits are the count
#: and bits 8-15 the payload length, so 0x00051411 means five 20-byte
#: notifications. The low byte being sent first is what makes it little-endian.
NOTIFY_COUNT = 5
NOTIFY_LENGTH = 0x14
NOTIFY_WRITE = bytes([0x11, NOTIFY_LENGTH, NOTIFY_COUNT, 0x00])


def test_dut_boots_and_advertises(dut):
    """The DUT reaches the shell and its extended advertisement is running.

    The same gate as the one-board suite: the peer test is only meaningful if
    the peripheral side is up, and a failure here says so in one line instead
    of surfacing as "the peer could not find anything".
    """
    dut.expect("- SiFli Corporation", timeout=30)
    dut.expect(PROMPT, timeout=20)
    dut.expect(r"receive BLE power on!", timeout=30)

    match = re.search(
        r"ADV start resutl (\d+), mode (\d+)",
        dut.expect(r"ADV start resutl \d+, mode \d+", timeout=30),
    )
    assert match is not None  # the pattern above guarantees this
    assert int(match.group(1)) == 0, "advertising failed to start"
    assert int(match.group(2)) == ADV_MODE_EXTENDED, "expected extended advertising"


def test_peer_instrument_answers(bt_peer):
    """The instrument runs the expected firmware and advertises the BLE role.

    The fixture already asked ``peer info`` and refused an image without
    ``proto=1``; this test states the capability the suite relies on, so a lab
    that flashed a Classic-only peer fails here with a sentence rather than at
    the first scan.
    """
    assert "ble" in bt_peer.info.get("caps", "").split(","), (
        f"the instrument does not advertise the BLE capability: {bt_peer.info}"
    )

    # The instrument is shared lab equipment, not a fresh board per case: an
    # earlier case (or an aborted run) may have left it scanning, advertising
    # or connected. Reset once here, before the link tests.
    bt_peer.ble.reset()


def test_peer_updates_advertising_data(bt_peer):
    """A running extended advertisement can be replaced; an empty one is refused.

    Runs before the link is made: SIBLES refuses an advertising update while
    this peer already holds a connection. The payload is short because the
    instrument's msh drops protocol lines above ~500 characters, so the
    248-byte extended ceiling needs the unimplemented ``peer data``
    continuation.
    """
    bt_peer.ble.advertise(mode="extended", data=b"\x01\x02", interval_ms=30)
    try:
        bt_peer.ble.set_advertising_data(b"\x03\x04")
        with pytest.raises(InstrumentCommandError) as refused:
            bt_peer.ble.set_advertising_data(b"")
        assert refused.value.code == "invalid_arg"
    finally:
        bt_peer.ble.stop_advertising()


def test_peer_connects_to_the_dut(dut, bt_peer):
    """The peer finds the DUT, connects to it, and the DUT reports the link.

    Scanning is a real controller operation here: extended advertisements are
    only visible to a peer whose controller understands them, so this also
    pins the peer firmware to a BLE 5 stack.
    """
    # The helper picks a connectable SIFLI_APP- advertiser and retries, because
    # the strongest report in a lab is often a neighbouring board: the DUT's
    # own connection line is the ground truth that the link is the right one.
    STATE["link"] = sifli_app.connect_to_dut(dut, bt_peer)

    dut.send_line("diss show_dev")
    dut.expect(r"Total connected 1 devices", timeout=10)


def test_peer_discovers_the_service(bt_peer):
    """Service discovery finds the example's 128-bit service and its handles."""
    listing = bt_peer.ble.discover(STATE["link"], SERVICE_UUID)
    assert listing.characteristics, "no characteristic in the discovered service"

    characteristic = listing.characteristics[0]
    STATE["value_handle"] = characteristic.value_handle
    STATE["cccd_handle"] = listing.cccd_handle()

    assert STATE["value_handle"] > characteristic.handle, (
        "the value handle must follow its declaration handle"
    )
    assert STATE["cccd_handle"] > STATE["value_handle"], (
        "the CCCD must follow the value handle"
    )


def test_peer_writes_and_reads_the_value(dut, bt_peer):
    """A write from the peer lands on the DUT, and a read returns it.

    0x22 is used instead of 0x11 so this write does not also start the
    notification stream the next test is about.
    """
    payload = bytes([0x22, 0x00, 0x00, 0x00])

    bt_peer.ble.write(STATE["link"], STATE["value_handle"], payload)
    dut.expect(
        r"Device 0 updated app value from:\d+ to:34",
        timeout=10,
    )

    assert bt_peer.ble.read(STATE["link"], STATE["value_handle"]) == payload


def test_peer_subscribes_to_notifications(dut, bt_peer):
    """Writing the CCCD enables notifications, and the DUT records it."""
    bt_peer.ble.subscribe(STATE["link"], STATE["cccd_handle"])
    dut.expect(r"device 0 CCCD 1", timeout=10)


def test_notification_burst_reaches_the_peer(dut, bt_peer):
    """The example's notification loop actually delivers every packet.

    The write's low byte 0x11 starts the server-side loop; the high bits ask for
    five 20-byte notifications. Each is filled with its own index by
    ``ble_write_to_remote``, so the payloads also prove the order and the count,
    not just that one packet arrived.
    """
    # The boundary is taken before the trigger: the DUT sends the whole burst
    # back to back, so one host read carries several packets. Without it the
    # second wait would start "from now" and the rest of the burst would be
    # treated as history.
    start = bt_peer.console.last_seq
    bt_peer.ble.write(STATE["link"], STATE["value_handle"], NOTIFY_WRITE)

    # The DUT's own view of the trigger: this line is printed only when the
    # write's low byte is 0x11, i.e. when the notification loop was started.
    dut.expect(r"example of ble notification \d+", timeout=10)

    for index in range(NOTIFY_COUNT):
        payload = bt_peer.ble.wait_notification(
            STATE["value_handle"], link=STATE["link"], since_seq=start, timeout=15
        )
        assert payload == bytes([index]) * NOTIFY_LENGTH, (
            f"notification {index} carried {payload.hex()}, "
            f"expected {NOTIFY_LENGTH} bytes of 0x{index:02x}"
        )


def test_peer_tunes_the_link(bt_peer):
    """The link-level ops a throughput-style case needs are live.

    Each call waits for the controller's completion, so a firmware that merely
    accepts the request and never reports back fails here instead of passing.
    """
    link = STATE["link"]
    bt_peer.ble.update_connection(
        link, interval_min_ms=15, interval_max_ms=30, latency=0, timeout_ms=5000
    )
    bt_peer.ble.set_phy(link, 2)
    bt_peer.ble.set_data_length(link, 251, 2120)
