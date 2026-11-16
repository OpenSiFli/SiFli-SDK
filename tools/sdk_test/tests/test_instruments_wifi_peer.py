# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the ``wifi_peer`` instrument.

The scripted console answers the protocol the firmware in
``example/test/wifi_peer`` implements. What it pins is the host's side of that
contract: which line goes out, how a reply becomes ``ApState``, and that a
malformed or refusing reply is an error rather than an empty value -- the same
things ``test_instruments_bt_peer`` pins for the BLE peer.
"""

from __future__ import annotations

import unittest

from _instrument_support import FakeLineConsole
from _instrument_support import info_line
from _instrument_support import lines
from _support import SdkTestCase

from sdk_test.instruments.console import InstrumentConsole
from sdk_test.instruments.errors import InstrumentCapabilityError
from sdk_test.instruments.errors import InstrumentCommandError
from sdk_test.instruments.errors import InstrumentProtocolError
from sdk_test.instruments.wifi_peer import WifiPeer

BSSID = "ce:0e:55:0b:3d:d7"


def _hex(text: str) -> str:
    return text.encode("utf-8").hex()


def peer_for(replies=None, initial="", capability="ap", caps="ap"):
    console = FakeLineConsole(
        {"peer info": info_line(caps=caps, fw="wifi_peer-1"), **(replies or {})},
        initial,
    )
    return WifiPeer(InstrumentConsole(console), capability=capability), console


class TestStart(SdkTestCase):
    def test_the_request_carries_the_credentials_and_the_reply_becomes_state(self):
        peer, console = peer_for(
            {
                "ap.start": lines(
                    f"@evt 1 ap_start ssid=sifli_test ch=6 mac={BSSID}",
                    "@ok 1 active=1 already=0 late=0 ssid=sifli_test ch=6 "
                    f"mac={BSSID} count=1 first_seq=0",
                )
            }
        )

        ap = peer.ap.start("sifli_test", "12345678")

        self.assertEqual(ap.ssid, "sifli_test")
        self.assertEqual(ap.channel, 6)
        self.assertEqual(ap.mac, BSSID)
        self.assertTrue(ap.active)
        self.assertFalse(ap.already)

        request = console.sent[-1]
        self.assertIn("ap.start", request)
        # The SSID and key travel hex-encoded: a space in either would be
        # split into two shell arguments and reach the peer as garbage.
        self.assertIn(f"ssid_hex={_hex('sifli_test')}", request)
        self.assertIn(f"key_hex={_hex('12345678')}", request)
        self.assertNotIn("ssid=sifli_test", request)
        self.assertIn("ch=6", request)
        self.assertIn("hidden=0", request)
        # Not pinned: the peer image's own default applies.
        self.assertNotIn("mac=", request)

    def test_a_pinned_bssid_is_sent_as_is(self):
        peer, console = peer_for(
            {
                "ap.start": lines(
                    "@evt 1 ap_start ssid=sifli_test ch=6 mac=02:00:5e:00:00:01",
                    "@ok 1 active=1 already=0 ssid=sifli_test ch=6 "
                    "mac=02:00:5e:00:00:01 count=1 first_seq=0",
                )
            }
        )

        ap = peer.ap.start("sifli_test", "12345678", mac="02:00:5e:00:00:01")

        self.assertIn("mac=02:00:5e:00:00:01", console.sent[-1])
        self.assertEqual(ap.mac, "02:00:5e:00:00:01")

    def test_an_open_network_sends_an_empty_key_rather_than_omitting_it(self):
        peer, console = peer_for(
            {
                "ap.start": lines(
                    f"@evt 1 ap_start ssid=open ch=6 mac={BSSID}",
                    "@ok 1 active=1 already=0 ssid=open ch=6 mac=%s count=1 first_seq=0"
                    % BSSID,
                )
            }
        )

        peer.ap.start("open", "")

        # An empty key is a statement ("open network"), not a missing argument:
        # the firmware treats a missing key and an empty one differently.
        self.assertIn(" key_hex= ", console.sent[-1])

    def test_an_already_running_ap_promises_no_event(self):
        peer, _ = peer_for(
            {
                "ap.start": lines(
                    f"@ok 1 active=1 already=1 ssid=sifli_test ch=6 mac={BSSID} "
                    "count=0 first_seq=4"
                )
            }
        )

        ap = peer.ap.start("sifli_test", "12345678")

        self.assertTrue(ap.already)

    def test_a_refusal_is_a_command_error_not_false(self):
        peer, _ = peer_for({"ap.start": "@err 1 radio start_ap err=-12\n"})

        with self.assertRaises(InstrumentCommandError):
            peer.ap.start("sifli_test", "12345678")

    def test_a_reply_for_another_ssid_is_rejected(self):
        peer, _ = peer_for(
            {
                "ap.start": lines(
                    f"@ok 1 active=1 already=0 ssid_hex={_hex('other')} ssid=other "
                    f"ch=6 mac={BSSID} count=0 first_seq=0"
                )
            }
        )

        with self.assertRaises(InstrumentProtocolError):
            peer.ap.start("sifli_test", "12345678")

    def test_an_ssid_with_a_space_survives_the_round_trip(self):
        peer, console = peer_for(
            {
                "ap.start": lines(
                    f"@ok 1 active=1 already=0 ssid_hex={_hex('Lab AP')} ch=6 "
                    f"mac={BSSID} count=1 first_seq=0"
                )
            }
        )

        ap = peer.ap.start("Lab AP", "key with space")

        self.assertEqual(ap.ssid, "Lab AP")
        self.assertIn(f"ssid_hex={_hex('Lab AP')}", console.sent[-1])
        self.assertIn(f"key_hex={_hex('key with space')}", console.sent[-1])

    def test_a_peer_that_disagrees_with_its_own_hex_ssid_is_rejected(self):
        peer, _ = peer_for(
            {
                "ap.start": lines(
                    f"@ok 1 active=1 already=0 ssid=Lab AP ssid_hex={_hex('other')} "
                    f"ch=6 mac={BSSID} count=0 first_seq=0"
                )
            }
        )

        with self.assertRaises(InstrumentProtocolError):
            peer.ap.start("Lab AP", "12345678")

    def test_a_different_channel_is_rejected(self):
        peer, _ = peer_for(
            {
                "ap.start": lines(
                    f"@ok 1 active=1 already=0 ssid_hex={_hex('sifli_test')} "
                    f"ch=11 mac={BSSID} count=0 first_seq=0"
                )
            }
        )

        with self.assertRaises(InstrumentProtocolError):
            peer.ap.start("sifli_test", "12345678", channel=6)

    def test_a_different_bssid_is_rejected(self):
        peer, _ = peer_for(
            {
                "ap.start": lines(
                    f"@ok 1 active=1 already=0 ssid_hex={_hex('sifli_test')} ch=6 "
                    "mac=02:00:5e:00:00:99 count=0 first_seq=0"
                )
            }
        )

        with self.assertRaises(InstrumentProtocolError):
            peer.ap.start("sifli_test", "12345678", mac="02:00:5e:00:00:01")


class TestReset(SdkTestCase):
    def test_reset_stops_the_ap_and_consumes_its_event(self):
        peer, _ = peer_for(
            {
                "state.reset": lines(
                    f"@evt 7 ap_stop mac={BSSID}",
                    "@ok 3 reset=1 active=0 count=1 first_seq=6",
                )
            }
        )

        peer.ap.reset()

    def test_reset_that_leaves_the_ap_up_is_an_error(self):
        peer, _ = peer_for(
            {"state.reset": lines("@ok 3 reset=1 active=1 count=0 first_seq=6")}
        )

        with self.assertRaises(InstrumentProtocolError):
            peer.ap.reset()


class TestStop(SdkTestCase):
    def test_stopping_an_ap_that_was_already_down_is_not_an_error(self):
        peer, _ = peer_for(
            {"ap.stop": lines("@ok 1 active=0 already=1 count=0 first_seq=2")}
        )

        ap = peer.ap.stop()

        self.assertFalse(ap.active)
        self.assertTrue(ap.already)

    def test_the_event_boundary_is_consumed_with_the_event(self):
        peer, _ = peer_for(
            {
                "ap.stop": lines(
                    f"@evt 5 ap_stop mac={BSSID}",
                    "@ok 1 active=0 already=0 count=1 first_seq=4",
                )
            }
        )

        ap = peer.ap.stop()

        self.assertFalse(ap.active)
        self.assertFalse(ap.already)


class TestStatus(SdkTestCase):
    def test_status_reports_the_bssid_and_the_informational_station_count(self):
        peer, _ = peer_for(
            {
                "ap.status": lines(
                    f"@ok 1 active=1 ssid=sifli_test ch=6 mac={BSSID} sta_num=0 "
                    "mode=1 count=0 first_seq=0"
                )
            }
        )

        ap = peer.ap.status()

        self.assertTrue(ap.active)
        self.assertEqual(ap.mac, BSSID)
        self.assertEqual(ap.station_count, 0)

    def test_an_off_ap_reports_no_channel(self):
        peer, _ = peer_for(
            {
                "ap.status": lines(
                    "@ok 1 active=0 ssid= ch=-1 mac=%s sta_num=0 mode=0 "
                    "count=0 first_seq=0" % BSSID
                )
            }
        )

        ap = peer.ap.status()

        self.assertFalse(ap.active)
        self.assertEqual(ap.ssid, "")
        self.assertEqual(ap.channel, -1)

    def test_a_peer_without_a_mac_is_not_a_usable_ap(self):
        peer, _ = peer_for(
            {
                "ap.status": lines(
                    "@ok 1 active=1 ssid=sifli_test ch=6 mac= sta_num=0 mode=1 "
                    "count=0 first_seq=0"
                )
            }
        )

        with self.assertRaises(InstrumentProtocolError):
            peer.ap.mac()


class TestCapability(SdkTestCase):
    def test_a_peer_that_does_not_advertise_ap_is_refused_at_construction(self):
        with self.assertRaises(InstrumentCapabilityError):
            peer_for(caps="ble")

    def test_a_bare_declaration_still_cannot_drive_the_ap(self):
        peer, _ = peer_for(capability="", caps="ble")

        with self.assertRaises(InstrumentCapabilityError):
            peer.ap


if __name__ == "__main__":
    unittest.main()
