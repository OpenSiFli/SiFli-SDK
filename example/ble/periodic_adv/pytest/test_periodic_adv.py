# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end peer path for the periodic_adv example.

The DUT runs a periodic advertising set. Its primary-channel data carries the
Sifli manufacturer field and its periodic channel carries a payload that the
example's own ``diss keep_per`` command keeps rewriting, so the suite checks
both halves: a plain scan finds the first, a sync to the set finds the second.
"""

from __future__ import annotations

import time

from sdk_test.suite_helpers import ble_server

#: The DUT's manufacturer AD in full: length 36, type 0xFF, the Sifli company
#: id 0x0A4C little-endian, then the example's 33 bytes of additional data.
MANUFACTURER_AD = bytes([0x24, 0xFF, 0x4C, 0x0A]) + bytes(
    [0x20, 0xC4, 0x00, 0x91]
) + bytes([0x22] * 29)

#: Length ``cmd_diss keep_per start <inv> <len>`` gives the periodic payload.
#: (This example exports its shell command as ``cmd_diss``; the sync
#: example aliases it to ``diss``.)
PERIODIC_LEN = 16

STATE: dict = {}


def _require(key: str):
    value = STATE.get(key)
    if value is None:
        raise AssertionError(f"{key} was not set: an earlier test did not complete")
    return value


def _ours(reports):
    """The reports that came from a SiFli application board."""
    return [
        report
        for report in reports
        if ble_server.advertised_name(report.data).startswith(
            ble_server.SIFLI_APP_NAME_PREFIX
        )
    ]


def test_dut_boots_and_advertises(dut):
    # Periodic mode is running mode 6; asserting it is what tells this example
    # apart from one that merely broadcasts the same data.
    ble_server.expect_ready(dut, expected_mode=6)


def test_peer_instrument_is_ready(bt_peer):
    bt_peer.ble.reset()


def test_the_advertisement_carries_the_sifli_manufacturer_data(bt_peer):
    # duplicate_filter keeps the instrument's 64-report table from filling with
    # one device's repeats before the DUT is heard.
    reports = bt_peer.ble.scan(duration_ms=3000, duplicate_filter=True)
    ours = _ours(reports)
    assert ours, (
        "no SiFli application advertisement was scanned; saw: "
        + ", ".join(f"{r.address}:{r.data.hex()}" for r in reports)
    )

    matching = [report for report in ours if MANUFACTURER_AD in report.data]
    assert matching, (
        "the DUT's advertisement did not carry the complete Sifli manufacturer "
        "field; got: " + ", ".join(report.data.hex() for report in ours)
    )


def test_the_periodic_channel_carries_the_updated_payload(dut, bt_peer):
    """The periodic channel is what makes this example periodic.

    ``cmd_diss keep_per`` rewrites the periodic data every 100 ms with
    ``PERIODIC_LEN`` copies of a byte that increments each time, so a
    synchronised payload says both that the channel exists and that its update
    path works.
    """
    dut.send_line(f"cmd_diss keep_per start 100 {PERIODIC_LEN}")
    dut.expect(r"per_adv_refresh start", timeout=10)

    reports = bt_peer.ble.scan(duration_ms=2000, duplicate_filter=True)
    ours = _ours(reports)
    assert ours, "the DUT was not found while looking for its periodic set"

    STATE["sync"] = bt_peer.ble.sync_periodic(ours[0])
    payload = bt_peer.ble.wait_periodic(link=STATE["sync"], timeout=15)

    assert len(payload) == PERIODIC_LEN, (
        f"the periodic payload was {len(payload)} bytes: {payload.hex()}"
    )
    assert len(set(payload)) == 1, (
        "the periodic payload should be one repeated byte after keep_per: "
        + payload.hex()
    )


def test_the_repeated_byte_increments(bt_peer):
    """The README's "每次周期更新会递加": the value must move between updates."""
    sync = _require("sync")

    first = bt_peer.ble.wait_periodic(link=sync, timeout=15)
    time.sleep(0.6)
    second = bt_peer.ble.wait_periodic(link=sync, timeout=15)

    assert second[0] != first[0], (
        "the periodic payload did not change between updates: "
        f"{first.hex()} then {second.hex()}"
    )
    assert len(set(second)) == 1, second.hex()

# `cmd_diss keep_per stop` is deliberately not asserted: the example prints
# nothing for it, and on this build the payload keeps changing after it
# (`rt_work_cancel` loses to the work handler resubmitting itself), so there is
# nothing observable to pin. Recorded in BT_PEER_DESIGN.md 14.4.
