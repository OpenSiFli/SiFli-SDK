# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the Classic half of ``bt_peer``.

These exercise the host's contract with the firmware, not a board: one
capability set, one console stream, ``btc.*`` ops. What is pinned here is the
wire shape -- request keys, the display-order UUID, hex names, the firmware
event boundary -- and that a malformed reply is an error rather than an empty
value. The capability gate is pinned too: a BLE-only image must not hand out a
Classic instrument.

Reverse verification of the inquiry cap (executed, each sabotage reverted to
green afterwards):

* ``max_rsp`` default back to 10 -> only
  ``test_inquiry_returns_the_devices_it_heard`` fails, with
  ``'max_rsp=0' not found in 'peer req 1 btc.inquiry duration_ms=1000
  max_rsp=10'``.
* ``max_rsp=max_rsp`` replaced by a hard-coded 0 -> only
  ``test_inquiry_passes_an_explicit_response_cap_through`` fails, with
  ``'max_rsp=25' not found in 'peer req 1 btc.inquiry duration_ms=1000
  max_rsp=0'``.
"""

from __future__ import annotations

import unittest

from _instrument_support import FakeLineConsole
from _instrument_support import info_line
from _instrument_support import lines
from _support import SdkTestCase

from sdk_test.instruments.bt_peer import BtPeer
from sdk_test.instruments.classic import ClassicDevice
from sdk_test.instruments.classic import ClassicLink
from sdk_test.instruments.classic import ClassicLocal
from sdk_test.instruments.classic import SPP_UUID
from sdk_test.instruments.console import InstrumentConsole
from sdk_test.instruments.errors import InstrumentProtocolError
from sdk_test.instruments.errors import InstrumentTimeout


def peer_for(replies=None, initial=""):
    console = FakeLineConsole(
        {"peer info": info_line(caps="ble,classic"), **(replies or {})}, initial
    )
    return BtPeer(InstrumentConsole(console), capability="ble,classic"), console


class TestClassic(SdkTestCase):
    # -- lifecycle -----------------------------------------------------------

    def test_reset_clears_events_from_the_previous_state(self):
        peer, _ = peer_for(
            {"state.reset": lines("@ok 1")},
            initial=lines("@evt 1 btc.disc link=1 reason=19"),
        )
        peer.console.drain()

        peer.classic.reset()

        with self.assertRaises(InstrumentTimeout):
            peer.console.next_event("btc.disc", timeout=0.1)

    def test_local_returns_the_instruments_own_identity(self):
        peer, console = peer_for(
            {"btc.local": lines("@ok 1 addr=112233445566 name=50656572 role=server")}
        )

        local = peer.classic.local(wait_ms=2500)

        self.assertEqual(local, ClassicLocal(address="112233445566", name="Peer"))
        self.assertIn("timeout_ms=2500", console.sent[-1])

    # -- discovery -----------------------------------------------------------

    def test_inquiry_returns_the_devices_it_heard(self):
        peer, console = peer_for(
            {
                "btc.inquiry": lines(
                    "@evt 1 btc.dev addr=aabbcc name=50686f6e65 rssi=-20",
                    "@evt 2 btc.dev addr=ddeeff name=48656164736574 rssi=-40",
                    "@ok 1 count=2 first_seq=0",
                )
            }
        )

        devices = peer.classic.inquiry(duration_ms=1000)

        self.assertEqual(
            devices,
            [
                ClassicDevice(address="aabbcc", name="Phone", rssi=-20),
                ClassicDevice(address="ddeeff", name="Headset", rssi=-40),
            ],
        )
        self.assertIn("duration_ms=1000", console.sent[-1])
        self.assertIn("max_rsp=0", console.sent[-1])

    def test_inquiry_passes_an_explicit_response_cap_through(self):
        peer, console = peer_for({"btc.inquiry": lines("@ok 1 count=0 first_seq=0")})

        peer.classic.inquiry(duration_ms=1000, max_rsp=25)

        self.assertIn("max_rsp=25", console.sent[-1])

    def test_an_inquiry_reply_without_first_seq_is_an_error(self):
        peer, _ = peer_for({"btc.inquiry": lines("@ok 1 count=0")})

        with self.assertRaises(InstrumentProtocolError) as caught:
            peer.classic.inquiry(duration_ms=1000)
        self.assertIn("first_seq", str(caught.exception))

    def test_a_name_that_is_not_hex_is_an_error(self):
        peer, _ = peer_for(
            {
                "btc.inquiry": lines(
                    "@evt 1 btc.dev addr=aabbcc name=zz",
                    "@ok 1 count=1 first_seq=0",
                )
            }
        )

        with self.assertRaises(InstrumentProtocolError):
            peer.classic.inquiry(duration_ms=1000)

    # -- pairing -------------------------------------------------------------

    def test_pair_sends_the_address_and_timeout(self):
        peer, console = peer_for({"btc.pair": lines("@ok 1 addr=aabbcc")})

        peer.classic.pair("aabbcc")

        self.assertIn("addr=aabbcc", console.sent[-1])
        self.assertIn("timeout_ms=15000", console.sent[-1])

    def test_pair_accepts_a_normalised_address(self):
        peer, console = peer_for({"btc.pair": lines("@ok 1 addr=aabbcc")})

        peer.classic.pair("AA-BB-CC")

        self.assertIn("addr=aabbcc", console.sent[-1])

    # -- connection ----------------------------------------------------------

    def test_connect_returns_a_stable_link(self):
        peer, console = peer_for(
            {"btc.connect": lines("@ok 1 link=1 addr=aabbcc channel=1 role=0")}
        )

        link = peer.classic.connect("AA-BB-CC")

        self.assertEqual(
            link, ClassicLink(link_id=1, address="aabbcc", channel=1, role="client")
        )
        self.assertIn("addr=aabbcc", console.sent[-1])
        self.assertIn(f"uuid={SPP_UUID}", console.sent[-1])
        self.assertIn("timeout_ms=10000", console.sent[-1])
        # The instrument derives the channel from paging; the host must not
        # pick one that could disagree with it.
        self.assertNotIn("channel=", console.sent[-1])

    def test_connect_sends_a_custom_display_order_uuid(self):
        peer, console = peer_for(
            {"btc.connect": lines("@ok 1 link=1 addr=aabbcc channel=2 role=0")}
        )

        peer.classic.connect("aabbcc", uuid="110b")

        self.assertIn("uuid=110b", console.sent[-1])

    def test_wait_connected_reads_the_connection_event(self):
        peer, _ = peer_for(
            initial=lines("@evt 1 btc.conn link=2 addr=aabbcc channel=3 role=0")
        )

        link = peer.classic.wait_connected()

        self.assertEqual(
            link, ClassicLink(link_id=2, address="aabbcc", channel=3, role="client")
        )

    def test_a_role_one_connection_is_the_server(self):
        peer, _ = peer_for(
            initial=lines("@evt 1 btc.conn link=1 addr=aabbcc channel=1 role=1")
        )

        link = peer.classic.wait_connected()

        self.assertEqual(link.role, "server")

    def test_wait_disconnected_waits_for_the_named_link(self):
        peer, _ = peer_for(initial=lines("@evt 1 btc.disc link=7 reason=19"))

        self.assertIsNone(
            peer.classic.wait_disconnected(link=ClassicLink(7, "aabbcc"))
        )

    def test_a_disconnection_for_another_link_does_not_satisfy_the_wait(self):
        peer, _ = peer_for(initial=lines("@evt 1 btc.disc link=7 reason=19"))

        with self.assertRaises(InstrumentTimeout):
            peer.classic.wait_disconnected(
                link=ClassicLink(8, "aabbcc"), timeout=0.05
            )

    # -- SPP data ------------------------------------------------------------

    def test_spp_send_encodes_the_payload(self):
        peer, console = peer_for({"btc.spp.send": lines("@ok 1 bytes=2")})

        peer.classic.spp_send(ClassicLink(1, "aabbcc"), b"\x01\x02")

        self.assertIn("len=2 data=0102", console.sent[-1])

    def test_wait_data_returns_the_payload(self):
        peer, _ = peer_for(
            initial=lines("@evt 1 btc.data link=1 len=2 data=0102")
        )

        self.assertEqual(
            peer.classic.wait_data(link=ClassicLink(1, "aabbcc")), b"\x01\x02"
        )

    def test_a_packet_for_another_link_is_kept(self):
        peer, _ = peer_for(
            initial=lines(
                "@evt 1 btc.data link=2 len=1 data=01",
                "@evt 2 btc.data link=1 len=1 data=02",
            )
        )

        self.assertEqual(
            peer.classic.wait_data(link=ClassicLink(1, "aabbcc")), b"\x02"
        )
        self.assertEqual(
            peer.classic.wait_data(link=ClassicLink(2, "ddeeff")), b"\x01"
        )

    def test_a_packet_for_another_link_does_not_satisfy_the_wait(self):
        peer, _ = peer_for(initial=lines("@evt 1 btc.data link=2 len=1 data=01"))

        with self.assertRaises(InstrumentTimeout):
            peer.classic.wait_data(link=ClassicLink(1, "aabbcc"), timeout=0.05)

    def test_a_bad_payload_length_is_an_error(self):
        peer, _ = peer_for(initial=lines("@evt 1 btc.data link=1 len=4 data=01"))

        with self.assertRaises(InstrumentProtocolError):
            peer.classic.wait_data(link=ClassicLink(1, "aabbcc"))


if __name__ == "__main__":
    unittest.main()
