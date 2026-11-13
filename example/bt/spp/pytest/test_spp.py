# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end peer path for the SPP example.

The DUT runs the SPP server and enables its Classic page scan at boot; it does
not advertise over BLE. The instrument is a `bt_peer:ble,classic` image: the
suite makes it inquire, connect, and exchange SPP payloads, and checks the
DUT's own console for the matching events, so a link to a neighbouring board
cannot pass.

Order matters: `dut` and `bt_peer` are session-scoped and their cursors only
move forward, so the tests run in the order defined here.
"""

from __future__ import annotations

import re
import time

from sdk_test.instruments.errors import InstrumentCommandError
from sdk_test.suite_helpers import sifli_app

#: Filled by one test and read by the next: the DUT name, the peer's own
#: address, the discovered device, the link and its service channel are all
#: runtime facts.
STATE: dict = {}


def _require(key: str):
    value = STATE.get(key)
    if value is None:
        raise AssertionError(f"{key} was not set: an earlier test did not complete")
    return value


def test_dut_classic_stack_is_ready(dut):
    """The DUT reaches the shell and its Classic stack reports ready.

    The local name is generated from the BLE public address
    (`SifliDemo-<b0>-<b1>-...`), so it identifies this board in the inquiry
    below where an address alone would need parsing.
    """
    dut.expect("- SiFli Corporation", timeout=30)
    dut.expect(sifli_app.PROMPT, timeout=20)

    line = dut.expect(r"Local device name:\s*\S+", timeout=40)
    match = re.search(r"Local device name:\s*(\S+)", line)
    assert match is not None  # the pattern above guarantees this
    STATE["dut_name"] = match.group(1)

    sifli_app.expect_classic_ready(dut)


def test_peer_is_the_classic_image(bt_peer):
    """The instrument really has the Classic half, and starts from a clean state.

    The fixture already refused an image without the declared capabilities;
    these assertions state what the suite relies on, so a mis-flashed peer
    fails here with a sentence instead of at the first `btc.*` command.
    """
    assert "classic" in bt_peer.caps, (
        f"the instrument does not advertise the Classic capability: {bt_peer.info}"
    )
    assert bt_peer.info.get("btc") == "1", (
        f"the instrument does not report btc=1: {bt_peer.info}"
    )

    # The instrument is shared lab equipment, not a fresh board per case: an
    # earlier case (or an aborted run) may have left a link open.
    bt_peer.classic.reset()
    STATE["peer_addr"] = bt_peer.classic.local().address


def test_peer_finds_the_dut(bt_peer):
    """Inquiry hears exactly one address carrying the DUT's name.

    Selecting by name is deterministic where RSSI is not. The controller may
    report the same device more than once during one inquiry (the firmware
    emits one `btc.dev` per DISCOVER_IND and does not deduplicate), so the
    matches are folded by address before counting. The failure message lists
    every report so a duplicate name or a missing page scan is visible.
    """
    expected = _require("dut_name")
    # A freshly flashed instrument can report the Classic stack ready before its
    # controller answers inquiries, and a DUT right after reset may still be
    # settling its inquiry scan. Ask a few times with a gap; the failure message
    # still lists everything heard when every attempt comes back empty.
    devices = []
    for attempt in range(3):
        if attempt:
            time.sleep(2.0)
        devices = bt_peer.classic.inquiry(duration_ms=5000)
        if any(device.name == expected for device in devices):
            break
    matches = [device for device in devices if device.name == expected]

    # Keep the first report per address: repeated reports are not extra DUTs.
    unique = {}
    for device in matches:
        unique.setdefault(device.address, device)

    heard = ", ".join(
        f"{device.address}:{device.name!r}:{device.rssi}" for device in devices
    )
    assert len(unique) == 1, (
        f"expected exactly one device named {expected!r}, got {len(unique)} "
        f"distinct address(es); heard: {heard or '<nothing>'}"
    )
    STATE["device"] = next(iter(unique.values()))


def test_peer_pairs_with_the_dut(bt_peer):
    """Bind a link key before opening SPP.

    The DUT's SPP channel authenticates: without a bond the first connect ends
    with HCI 0x06 (PIN or key missing) and the RFCOMM channel is refused. The
    key persists, so this one pairing also serves the DUT-initiated link later
    in the suite.

    The first attempt can fail when only one side still holds the old key: the
    DUT then asks to encrypt with a key the instrument (whose ``state.reset``
    dropped its bonds) no longer has. That KEY_MISSING also makes the DUT drop
    its record, so a second attempt starts from a symmetric state.
    """
    address = _require("device").address
    for attempt in (1, 2):
        try:
            bt_peer.classic.pair(address)
            return
        except InstrumentCommandError:
            if attempt == 2:
                raise


def test_peer_connects_to_the_dut(dut, bt_peer):
    """The peer pages the DUT and the DUT's own SPP event confirms the link."""
    device = _require("device")
    # SDP + ACL + DLC setup and the encryption the DUT asks for share this
    # budget; a 10 s one was observed to expire while RFC DLC negotiation was
    # still running, so give the instrument and the console room.
    link = bt_peer.classic.connect(device.address, timeout_ms=25000, timeout=40.0)

    dut.expect("spp connect success!!!", timeout=15)
    line = dut.expect(r"chl \d+", timeout=10)
    match = re.search(r"chl (\d+)", line)
    assert match is not None  # the pattern above guarantees this
    # Each end reports the service channel *it* allocated for this link; the
    # two numbers differ by design (the peer's `link.channel` is the one
    # `btc.spp.send` uses, this one is what the DUT's `send_data` needs).
    channel = int(match.group(1))
    assert channel > 0, f"the DUT reported service channel {channel}"

    STATE["link"] = link
    STATE["channel"] = channel


def test_peer_sends_data_to_the_dut(dut, bt_peer):
    """Each byte the peer sends reaches the DUT's RX handler."""
    link = _require("link")
    payload = bytes([0xA5, 0x5A, 0x01, 0x02])

    bt_peer.classic.spp_send(link, payload)

    # The header names the source and channel; the bytes follow it one line
    # each. `0x%x` does not zero-pad, so the expectation is not formatted.
    dut.expect(r"\[SPP .* RX\]:", timeout=10)
    for byte in payload:
        dut.expect("0x%x" % byte, timeout=5)


def test_the_dut_sends_data_back(dut, bt_peer):
    """The DUT's `spp send_data` command delivers its fixed test string."""
    link = _require("link")
    start = bt_peer.console.last_seq

    dut.send_line(f"spp send_data {_require('peer_addr')} {_require('channel')}")
    payload = bt_peer.classic.wait_data(link=link, since_seq=start, timeout=15)

    assert payload == b"This is spp test!!!", (
        f"the DUT sent {payload!r}, expected b'This is spp test!!!'"
    )


def test_the_dut_can_initiate_the_connection(dut, bt_peer):
    """The DUT pages the peer after the first link is torn down.

    The disconnect first is what makes the incoming link unambiguous: the peer
    already held a link to this DUT, and a stale `btc.conn` would otherwise
    satisfy the wait. `role == "server"` is the instrument's side of the link,
    i.e. the DUT connected to it.
    """
    link = _require("link")
    device = _require("device")

    start = bt_peer.console.last_seq
    bt_peer.classic.disconnect(link)
    bt_peer.classic.wait_disconnected(link=link, since_seq=start, timeout=15)
    dut.expect("spp connect disconnect!!!", timeout=15)

    start = bt_peer.console.last_seq
    # The DUT's SPP client needs the remote channel too; its own `spp connect`
    # does not search first (the instrument learned that the hard way), so ask
    # for the SDP result explicitly and only then page the peer.
    dut.send_line(f"spp search {_require('peer_addr')} 2 1101")
    line = dut.expect(r"sdp res = \d+,rmt_srv_chl = \d+", timeout=20)
    assert "sdp res = 0" in line, f"the DUT's SDP search failed: {line!r}"

    dut.send_line(f"spp connect {_require('peer_addr')} 2 1101")
    incoming = bt_peer.classic.wait_connected(since_seq=start, timeout=40)

    assert incoming.address == device.address, (
        f"the incoming link came from {incoming.address}, "
        f"expected the DUT at {device.address}"
    )
    assert incoming.role == "server", (
        f"the instrument is on the {incoming.role!r} side, expected 'server'"
    )
    dut.expect("spp connect success!!!", timeout=15)

    # The new link carries data too, so the suite does not pass on an event
    # that never became a usable SPP channel.
    bt_peer.classic.spp_send(incoming, b"\x5a\xa5")
    dut.expect(r"\[SPP .* RX\]:", timeout=10)
    dut.expect("0x5a", timeout=5)
    dut.expect("0xa5", timeout=5)
