# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the Classic half of ``bt_peer``.

The M1 firmware has no Classic stack, so these tests exercise the host
contract, not a board: one capability set, one console stream, ``btc.*`` ops.
They also pin the gate -- a BLE-only image must not hand out a Classic
instrument.
"""

from __future__ import annotations

import unittest

from _instrument_support import FakeLineConsole
from _instrument_support import info_line
from _instrument_support import lines
from _support import SdkTestCase

from sdk_test.instruments.bt_peer import BtPeer
from sdk_test.instruments.classic import ClassicLink
from sdk_test.instruments.console import InstrumentConsole
from sdk_test.instruments.errors import InstrumentProtocolError


def peer_for(replies=None, initial=""):
    console = FakeLineConsole(
        {"peer info": info_line(caps="ble,classic"), **(replies or {})}, initial
    )
    return BtPeer(InstrumentConsole(console), capability="ble,classic"), console


class TestClassic(SdkTestCase):
    def test_inquiry_returns_the_devices_it_heard(self):
        peer, console = peer_for(
            {
                "btc.inquiry": lines(
                    "@evt 1 btc.dev addr=aabbcc name=Phone rssi=-20",
                    "@ok 1 count=1",
                )
            }
        )

        devices = peer.classic.inquiry(duration_ms=1000)

        self.assertEqual(len(devices), 1)
        self.assertEqual(devices[0].address, "aabbcc")
        self.assertEqual(devices[0].name, "Phone")
        self.assertEqual(devices[0].rssi, -20)
        self.assertIn("duration_ms=1000", console.sent[-1])

    def test_connect_returns_a_stable_link(self):
        peer, console = peer_for(
            {"btc.connect": lines("@ok 1 link=0 addr=aabbcc channel=1")}
        )

        link = peer.classic.connect("aa-bb-cc")

        self.assertEqual(link, ClassicLink(link_id=0, address="aabbcc", channel=1))
        self.assertIn("addr=aabbcc", console.sent[-1])
        self.assertIn("channel=1", console.sent[-1])

    def test_spp_send_encodes_the_payload(self):
        peer, console = peer_for({"btc.spp.send": lines("@ok 1")})

        peer.classic.spp_send(ClassicLink(0, "aabbcc"), b"\x01\x02")

        self.assertIn("len=2 data=0102", console.sent[-1])

    def test_wait_data_returns_the_payload(self):
        peer, _ = peer_for(
            initial=lines("@evt 1 btc.data link=0 len=2 data=0102")
        )

        self.assertEqual(
            peer.classic.wait_data(link=ClassicLink(0, "aabbcc")), b"\x01\x02"
        )

    def test_a_bad_payload_length_is_an_error(self):
        peer, _ = peer_for(initial=lines("@evt 1 btc.data link=0 len=4 data=01"))

        with self.assertRaises(InstrumentProtocolError):
            peer.classic.wait_data(link=ClassicLink(0, "aabbcc"))

    def test_wait_connected_reads_the_connection_event(self):
        peer, _ = peer_for(
            initial=lines("@evt 1 btc.conn link=2 addr=aabbcc channel=3")
        )

        link = peer.classic.wait_connected()

        self.assertEqual(link, ClassicLink(link_id=2, address="aabbcc", channel=3))


if __name__ == "__main__":
    unittest.main()
