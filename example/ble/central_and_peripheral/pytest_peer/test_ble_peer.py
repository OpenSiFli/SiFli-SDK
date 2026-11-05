# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of the BLE central-and-peripheral example against a peer.

Where the one-board suite (`../pytest/`) stops at what a single radio can show,
this suite drives the other half of the example: a second board, wired to the
same testbed and declared in the binding as the `ble_peer` instrument, acts as
the GAP central and GATT client. Its console is the `ble_peer` fixture; the
board under test keeps `dut`.

The instrument is not built or flashed by this framework. The lab flashes it
once with this same example, built from this same project
(`example/ble/central_and_peripheral/project/hcpu`) -- its `diss` command is the
protocol `BlePeer` speaks -- and leaves it running. That is why this suite can
send `diss ...` without a build step of its own.

Which advertisement the peer connects to is deliberately not hard-coded: it
scans, takes the strongest report, and the DUT-side connection log is the
assertion. In a rack with many equally close boards that choice can go wrong,
and the test fails at the DUT rather than silently passing. Keeping the peer
next to its DUT is the lab's part of the contract.

Order matters: `dut` and `ble_peer` are session-scoped and their cursors only
move forward, so the tests run in the order defined here.
"""

from __future__ import annotations

import re

#: The prompt is ``msh />`` when DFS keeps a working directory and ``msh >``
#: when it does not. Accept both rather than pinning one carrier's spelling.
PROMPT = r"msh [^>\n]*>"

#: The modes the stack reports in ``sibles_adv_evt_startted_t``; the example
#: always starts extended advertising (``USING_LE_EXTENSION_ADV`` in main.c).
ADV_MODE_EXTENDED = 5

#: Filled by one test and read by the next: the discovered handles come from the
#: peer's service listing, and there is no sane way to hard-code them -- the
#: server assigns handles when it registers the service.
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
    the peripheral side is up, and a failure here says so in one line instead of
    surfacing as "the peer could not find anything".
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


def test_peer_instrument_answers(ble_peer):
    """The instrument is running the expected firmware and its shell answers.

    ``diss help`` rather than a bare prompt: an empty prompt would also come
    from a board with the wrong image on it, and the point of this test is that
    the *command set* this suite relies on is there.
    """
    ble_peer.expect_shell()


def test_peer_connects_to_the_dut(dut, ble_peer):
    """The peer finds the DUT, connects to it, and the DUT reports the link.

    Scanning is a real controller operation here: extended advertisements are
    only visible to a peer whose controller understands them, so this also
    pins the peer firmware to a BLE 5 stack.
    """
    advertisements = ble_peer.scan(duration_ms=3000)
    target = ble_peer.strongest(advertisements)
    ble_peer.connect(target)

    # The DUT's own connection event is the ground truth: the peer may have
    # connected to some other board in range, and only this line proves it did
    # not.
    dut.expect(r"Peer device\(role:\d+\) .* connected as deivce 0", timeout=30)

    dut.send_line("diss show_dev")
    dut.expect(r"Total connected 1 devices", timeout=10)


def test_peer_discovers_the_service(ble_peer):
    """Service discovery finds the example's 128-bit service and its handles."""
    listing = ble_peer.search_service()
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


def test_peer_writes_and_reads_the_value(dut, ble_peer):
    """A write from the peer lands on the DUT, and a read returns it.

    0x22 is used instead of 0x11 so this write does not also start the
    notification stream the next test is about.
    """
    value_handle = STATE["value_handle"]
    payload = bytes([0x22, 0x00, 0x00, 0x00])

    ble_peer.write_value(value_handle, payload)
    dut.expect(
        r"Device 0 updated app value from:\d+ to:34",
        timeout=10,
    )

    assert ble_peer.read_value(value_handle) == payload


def test_peer_subscribes_to_notifications(dut, ble_peer):
    """Writing the CCCD enables notifications, and the DUT records it."""
    ble_peer.subscribe(STATE["cccd_handle"])
    dut.expect(r"device 0 CCCD 1", timeout=10)


def test_notification_burst_reaches_the_peer(dut, ble_peer):
    """The example's notification loop actually delivers every packet.

    The write's low byte 0x11 starts the server-side loop; the high bits ask for
    five 20-byte notifications. Each is filled with its own index by
    ``ble_write_to_remote``, so the payloads also prove the order and the count,
    not just that one packet arrived.
    """
    ble_peer.write_value(STATE["value_handle"], NOTIFY_WRITE)

    # The DUT's own view of the trigger: this line is printed only when the
    # write's low byte is 0x11, i.e. when the notification loop was started.
    dut.expect(r"example of ble notification \d+", timeout=10)

    for index in range(NOTIFY_COUNT):
        payload = ble_peer.wait_notification(STATE["value_handle"], timeout=15)
        assert payload == bytes([index]) * NOTIFY_LENGTH, (
            f"notification {index} carried {payload.hex()}, "
            f"expected {NOTIFY_LENGTH} bytes of 0x{index:02x}"
        )
