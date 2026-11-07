# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end peer path for the iBeacon example.

The DUT only broadcasts (mode 2, non-connectable), so there is no link and no
GATT: the instrument scans and the assertions are on the advertisement payload
and on which device sent it.
"""

from __future__ import annotations

import re

#: The whole iBeacon AD structure: length 26, type 0xFF, Apple company id
#: 0x004C, beacon type 0x1502, the proximity UUID, major 0x0100, minor 0x0102
#: and measured power -50 dBm.
#:
#: Matched as a substring rather than at offset 0: the controller prepends the
#: Flags AD (``02 01 1c``), and that is not the example's business.
IBEACON_AD = bytes([
    0x1A, 0xFF, 0x4C, 0x00, 0x02, 0x15,
    0x95, 0xD0, 0xB4, 0x22, 0xA4, 0xBD, 0x45, 0xD4,
    0x99, 0x20, 0x57, 0x6B, 0xC6, 0x63, 0x23, 0x72,
    0x01, 0x00, 0x01, 0x02, 0xCE,
])

PROMPT = r"msh [^>\n]*>"

#: The example prints the address most-significant byte first; the peer reports
#: it in the on-air order, so the two are compared after reversing it.
ADDRESS_RE = re.compile(
    r"ble_get_public_address :([0-9a-f]{2}(?:-[0-9a-f]{2}){5})"
)

STATE: dict = {}


def _require(key: str):
    value = STATE.get(key)
    if value is None:
        raise AssertionError(f"{key} was not set: an earlier test did not complete")
    return value


def test_dut_boots_and_advertises(dut):
    dut.expect("- SiFli Corporation", timeout=30)
    dut.expect(PROMPT, timeout=20)

    # Before advertising: the main thread prints the effective address, then
    # reports power-on, then the stack answers the start request.
    line = dut.expect(
        r"ble_get_public_address :[0-9a-f]{2}(?:-[0-9a-f]{2}){5}", timeout=20
    )
    match = ADDRESS_RE.search(line)
    assert match is not None, f"could not read an address from {line!r}"
    # "5c-fd-52-00-1d-5a" -> "5a1d0052fd5c", the order the peer reports.
    STATE["address"] = "".join(reversed(match.group(1).split("-")))

    dut.expect(r"receive BLE power on!", timeout=30)

    # This example spells it "result", not the "resutl" the others use, and
    # broadcasts in mode 2.
    line = dut.expect(r"ADV start result \d+, mode \d+", timeout=30)
    match = re.search(r"ADV start result (\d+), mode (\d+)", line)
    assert match is not None
    assert int(match.group(1)) == 0, "advertising failed to start"
    assert int(match.group(2)) == 2, "expected broadcast advertising"


def test_peer_instrument_is_ready(bt_peer):
    bt_peer.ble.reset()


def test_advertisement_carries_the_ibeacon_payload(bt_peer):
    # Reset here too: this test must stand on its own under `pytest -k`, and a
    # scan left running by anything else would be rejected as a state error.
    bt_peer.ble.reset()
    address = _require("address")

    # One report per device: 64 identical reports would otherwise fill the
    # instrument's whole report table and hide the DUT's later broadcasts.
    reports = bt_peer.ble.scan(duration_ms=3000, duplicate_filter=True)

    ours = [report for report in reports if report.address == address]
    assert ours, (
        f"no advertisement from the DUT ({address}); saw: "
        + ", ".join(f"{r.address}:{r.data.hex()}" for r in reports)
    )

    matching = [report for report in ours if IBEACON_AD in report.data]
    assert matching, (
        f"the DUT's advertisement did not carry the iBeacon payload; got: "
        + ", ".join(report.data.hex() for report in ours)
    )


def test_adv_update_replaces_the_payload(dut, bt_peer):
    """The README's own example, with its own four fields.

    Only the three numeric fields are asserted: this build's
    ``uuid_string_to_bytes()`` does not parse the documented string form -- the
    DUT's own ``in uuid_string_to_bytes`` hexdump shows two bytes written
    (``0C 00``, i.e. "12" read as decimal) and the old UUID left in place, and
    the function returns 2 instead of 16. That is the example's bug, recorded in
    BT_PEER_DESIGN.md 14.4; asserting the UUID here would lock it in.
    """
    dut.send_line("cmd_diss adv_update 12345678-1234-1234-1234-123456789abc 1 2 -60")
    dut.expect(r"major: 1, minor: 2, measured_power: -60", timeout=10)
    dut.expect(r"update adv 0", timeout=10)

    # major 1, minor 2, power -60, big-endian, at the end of the 27-byte AD.
    updated_tail = bytes([0x00, 0x01, 0x00, 0x02, 0xC4])

    bt_peer.ble.reset()
    reports = bt_peer.ble.scan(duration_ms=3000, duplicate_filter=True)
    ours = [report for report in reports if report.address == _require("address")]
    assert ours, "the DUT was not found after the update"
    assert any(updated_tail in report.data for report in ours), (
        "the advertisement still carries the old major/minor/power; got: "
        + ", ".join(report.data.hex() for report in ours)
    )


def test_adv_stop_and_start(dut, bt_peer):
    """Both documented commands, with the peer as the observable."""
    address = _require("address")

    dut.send_line("cmd_diss adv_stop")
    dut.expect(r"ADV stopped reason 0, mode 2", timeout=10)

    bt_peer.ble.reset()
    reports = bt_peer.ble.scan(duration_ms=3000, duplicate_filter=True)
    assert not [report for report in reports if report.address == address], (
        "the DUT was still advertising after cmd_diss adv_stop"
    )

    dut.send_line("cmd_diss adv_start")
    dut.expect(r"ADV start result 0, mode 2", timeout=10)

    bt_peer.ble.reset()
    reports = bt_peer.ble.scan(duration_ms=3000, duplicate_filter=True)
    assert [report for report in reports if report.address == address], (
        "the DUT did not come back after cmd_diss adv_start"
    )
