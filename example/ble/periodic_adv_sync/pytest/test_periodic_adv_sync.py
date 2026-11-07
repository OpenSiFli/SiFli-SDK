# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end peer path for the periodic_adv_sync example.

Here the roles are reversed from most suites: the *instrument* is the periodic
advertiser, and the DUT scans for it, creates a sync, and prints the data it
receives. The suite therefore drives the DUT's own shell commands and checks
that what it reports back is the instrument's address and payload.
"""

from __future__ import annotations

import re

#: Distinctive enough that the DUT's hexdump cannot match another advertiser.
PAYLOAD = bytes([0xA1, 0xB2, 0xC3, 0xD4])

PROMPT = r"msh [^>\n]*>"

#: The example prints the address with a single "0x" prefix:
#: ``addr:0x42:51:a9:41:80:14``.
FOUND_RE = re.compile(
    r"Periodic adv found addr_type (\d+), "
    r"addr:0x([0-9a-f]{2}(?::[0-9a-f]{2}){5}), adv_sid:(\d+)"
)

STATE: dict = {}


def _require(key: str):
    value = STATE.get(key)
    if value is None:
        raise AssertionError(f"{key} was not set: an earlier test did not complete")
    return value


def test_dut_boots(dut):
    dut.expect("- SiFli Corporation", timeout=30)
    dut.expect(PROMPT, timeout=20)
    dut.expect(r"receive BLE power on!", timeout=30)


def test_peer_advertises_periodically(bt_peer):
    bt_peer.ble.reset()
    bt_peer.ble.start_periodic_advertising(PAYLOAD, interval_ms=100)
    # "4251a9418014" -> "14:80:41:a9:51:42", the order the DUT scans and logs.
    address = bt_peer.info["addr"]
    STATE["address"] = ":".join(
        address[index:index + 2] for index in range(0, len(address), 2)
    )


def test_dut_finds_the_instrument(dut):
    dut.send_line("diss sync create")
    line = dut.expect(r"Create PER_ADV_SYNC result \d+", timeout=10)
    assert re.search(r"result 0$", line.strip()), line

    dut.send_line("diss scan start 1 60 30 20000")
    dut.expect(r"Scan start status 0", timeout=10)

    line = dut.expect(FOUND_RE.pattern, timeout=20)
    match = FOUND_RE.search(line)
    assert match is not None, line

    addr_type, found, sid = match.group(1), match.group(2), match.group(3)
    assert found == _require("address"), (
        f"the DUT found {found}, not the instrument at {_require('address')}"
    )
    STATE["addr_type"] = addr_type
    STATE["sid"] = sid


def test_dut_syncs_and_receives_the_payload(dut):
    dut.send_line(
        f"diss sync start {_require('address')} {_require('addr_type')} "
        f"{_require('sid')} 400"
    )
    # The match itself is the assertion: the DUT only logs this when the sync
    # it asked for was established, with the address it was given.
    dut.expect(r"PER_ADV_SYNC established\(", timeout=20)

    # LOG_HEX prints the offset in lower case and the bytes in upper case.
    dump = dut.expect(
        r"per_adv_data: [0-9a-f]{4}-[0-9a-f]{4}: [0-9A-Fa-f ]+", timeout=15
    )
    assert "a1 b2 c3 d4" in dump.lower(), f"the DUT received {dump!r}"
