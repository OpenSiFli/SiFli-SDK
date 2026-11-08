# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the BLE half of ``bt_peer``.

The scripted console here answers the protocol, not a log transcript. What it
pins is the host's contract with the firmware: which line is sent, how a reply
becomes a dataclass, and that a malformed reply is an error rather than an
empty value.
"""

from __future__ import annotations

import unittest

from _instrument_support import FakeLineConsole
from _instrument_support import info_line
from _instrument_support import lines
from _support import SdkTestCase

from sdk_test.instruments.bt_peer import AdvReport
from sdk_test.instruments.bt_peer import BtPeer
from sdk_test.instruments.bt_peer import CCCD_UUID
from sdk_test.instruments.bt_peer import Link
from sdk_test.instruments.console import InstrumentConsole
from sdk_test.instruments.errors import InstrumentCapabilityError
from sdk_test.instruments.errors import InstrumentCommandError
from sdk_test.instruments.errors import InstrumentProtocolError
from sdk_test.instruments.errors import InstrumentTimeout


def peer_for(replies=None, initial="", capability="ble", caps="ble"):
    console = FakeLineConsole({"peer info": info_line(caps=caps), **(replies or {})}, initial)
    return BtPeer(InstrumentConsole(console), capability=capability), console


class TestScan(SdkTestCase):
    def test_reports_are_sorted_strongest_first(self):
        peer, _ = peer_for(
            {
                "gap.scan": lines(
                    "@evt 1 adv index=2 addr=aa-bb type=public rssi=-50 len=2 data=0102",
                    "@evt 2 adv index=1 addr=cc-dd type=random rssi=-30",
                    "@evt 3 scan.done count=2",
                    "@ok 1 count=2 first_seq=0",
                )
            }
        )

        reports = peer.ble.scan(duration_ms=1000)

        self.assertEqual([report.address for report in reports], ["cc-dd", "aa-bb"])
        self.assertEqual(reports[1].data, b"\x01\x02")
        self.assertEqual(reports[1].address_type, "public")

    def test_a_numeric_address_type_is_spelled_out(self):
        peer, _ = peer_for(
            {
                "gap.scan": lines(
                    "@evt 1 adv index=1 addr=aa type=1 rssi=-40",
                    "@ok 1 count=1 first_seq=0",
                )
            }
        )

        self.assertEqual(peer.ble.scan()[0].address_type, "random")

    def test_an_unmeasured_report_sorts_last(self):
        peer, _ = peer_for(
            {
                "gap.scan": lines(
                    "@evt 1 adv index=1 addr=no-rssi",
                    "@evt 2 adv index=2 addr=weak rssi=-80",
                    "@evt 3 scan.done count=2",
                    "@ok 1 count=2 first_seq=0",
                )
            }
        )

        reports = peer.ble.scan()

        self.assertEqual([report.address for report in reports], ["weak", "no-rssi"])
        self.assertIsNone(reports[1].rssi)

    def test_a_late_event_from_a_timed_out_scan_does_not_satisfy_the_retry(self):
        console = FakeLineConsole(
            {
                "peer info": info_line(caps="ble"),
                "gap.scan": [
                    "",
                    lines("@evt 2 adv index=1 addr=fresh rssi=-30", "@ok 2 count=1 first_seq=1"),
                ],
            }
        )
        peer = BtPeer(InstrumentConsole(console), capability="ble")

        with self.assertRaises(InstrumentTimeout):
            peer.ble.scan(timeout=0.05)

        # The timed-out scan's report arrives after the timeout. It must be
        # parsed before the retry takes its boundary, not handed to the retry.
        console.feed(lines("@evt 1 adv index=9 addr=stale rssi=-20"))
        reports = peer.ble.scan()

        self.assertEqual([report.address for report in reports], ["fresh"])

    def test_a_stale_event_does_not_satisfy_a_new_scan(self):
        console = FakeLineConsole(
            {
                "peer info": info_line(caps="ble"),
                "gap.scan": lines("@evt 11 adv index=1 addr=fresh rssi=-40", "@ok 1 count=1 first_seq=10"),
            },
            initial=lines("@evt 10 adv index=9 addr=stale rssi=-20"),
        )
        peer = BtPeer(InstrumentConsole(console), capability="ble")
        peer.console.drain()

        reports = peer.ble.scan()

        self.assertEqual([report.address for report in reports], ["fresh"])

    def test_reset_clears_events_from_the_previous_state(self):
        peer, _ = peer_for(
            {"state.reset": lines("@ok 1")},
            initial=lines("@evt 1 adv index=0 addr=stale"),
        )
        peer.console.drain()

        peer.ble.reset()

        with self.assertRaises(InstrumentTimeout):
            peer.console.next_event("adv", timeout=0.1)

    def test_an_empty_scan_says_what_to_check(self):
        peer, _ = peer_for(
            {"gap.scan": lines("@evt 1 scan.done count=0", "@ok 1 count=0 first_seq=0")}
        )

        with self.assertRaises(AssertionError) as caught:
            peer.ble.strongest(peer.ble.scan())
        self.assertIn("advertising", str(caught.exception))


class TestConnection(SdkTestCase):
    def test_connect_uses_the_scan_index_and_returns_a_link(self):
        peer, console = peer_for(
            {"gap.connect": lines("@ok 1 link=0 addr=aabbcc role=central interval_ms=30")}
        )
        report = AdvReport(index=3, address="aa-bb-cc", address_type="public", rssi=-40)

        link = peer.ble.connect(report)

        self.assertEqual(link, Link(link_id=0, address="aabbcc", role="central", interval_ms=30))
        self.assertIn("index=3", console.sent[-1])
        self.assertIn("addr_type=0", console.sent[-1])
        # The index names the device; sending an address as well would be a
        # second, possibly contradictory, identifier.
        self.assertNotIn("addr=", console.sent[-1])

    def test_connect_by_address_does_not_need_a_scan(self):
        peer, console = peer_for(
            {"gap.connect": lines("@ok 1 link=2 addr=112233 role=central interval_ms=30")}
        )

        link = peer.ble.connect(address="11-22-33", address_type=1)

        self.assertEqual(link.link_id, 2)
        self.assertIn("addr=112233", console.sent[-1])
        self.assertIn("addr_type=1", console.sent[-1])

    def test_a_random_report_connects_with_addr_type_one(self):
        peer, console = peer_for(
            {"gap.connect": lines("@ok 1 link=0 addr=aa role=central interval_ms=30")}
        )

        peer.ble.connect(AdvReport(1, "aa", "random", -40))

        self.assertIn("addr_type=1", console.sent[-1])

    def test_a_refused_connection_is_an_error(self):
        peer, _ = peer_for({"gap.connect": lines("@err 1 failed connection refused")})

        with self.assertRaises(InstrumentCommandError) as caught:
            peer.ble.connect(AdvReport(1, "aa", "public", -40))
        self.assertEqual(caught.exception.code, "failed")
        self.assertIn("connection refused", str(caught.exception))

    def test_list_links_reads_the_event_before_the_terminal(self):
        peer, _ = peer_for(
            {
                "dev.list": lines(
                    "@evt 1 link link=0 addr=aa role=central interval_ms=30",
                    "@ok 1 links=1 first_seq=0",
                )
            }
        )

        links = peer.ble.list_links()

        self.assertEqual(len(links), 1)
        self.assertEqual(links[0].link_id, 0)
        self.assertEqual(links[0].role, "central")

    def test_list_links_spells_out_a_peripheral_role(self):
        peer, _ = peer_for(
            {
                "dev.list": lines(
                    "@evt 1 link link=0 addr=aa role=1 interval_ms=30",
                    "@ok 1 links=1 first_seq=0",
                )
            }
        )

        self.assertEqual(peer.ble.list_links()[0].role, "peripheral")


class TestGattClient(SdkTestCase):
    def test_discover_returns_characteristics_and_handles(self):
        peer, _ = peer_for(
            {
                "gatt.discover": lines(
                    "@evt 1 char link=0 index=0 handle=9 value_handle=10 descs=11",
                    "@ok 1 count=1 first_seq=0",
                )
            }
        )

        service = peer.ble.discover(Link(0, "aa"))

        self.assertEqual(service.value_handle(), 10)
        self.assertEqual(service.cccd_handle(), 11)

    def test_the_cccd_is_picked_by_uuid_not_by_position(self):
        """A HID report has a report reference next to its CCCD."""
        peer, _ = peer_for(
            {
                "gatt.discover": lines(
                    "@evt 1 char link=0 index=0 handle=9 value_handle=10 "
                    "descs=11:0829,12:0229",
                    "@ok 1 count=1 first_seq=0",
                )
            }
        )

        service = peer.ble.discover(Link(0, "aa"))

        self.assertEqual(service.cccd_handle(), 12)
        self.assertEqual(
            service.characteristics[0].descriptor_handle(CCCD_UUID), 12
        )

    def test_a_characteristic_with_several_descriptors_needs_a_cccd(self):
        """Guessing would write the wrong handle; say so instead."""
        peer, _ = peer_for(
            {
                "gatt.discover": lines(
                    "@evt 1 char link=0 index=0 handle=9 value_handle=10 "
                    "descs=11:2803,12:2a05",
                    "@ok 1 count=1 first_seq=0",
                )
            }
        )

        service = peer.ble.discover(Link(0, "aa"))

        with self.assertRaises(InstrumentProtocolError):
            service.cccd_handle()

    def test_discover_without_characteristics_is_a_failure(self):
        peer, _ = peer_for({"gatt.discover": lines("@ok 1 count=0 first_seq=0")})

        with self.assertRaises(AssertionError):
            peer.ble.discover(Link(0, "aa"))

    def test_read_returns_the_payload(self):
        peer, _ = peer_for({"gatt.read": lines("@ok 1 len=4 data=22000000")})

        self.assertEqual(peer.ble.read(Link(0, "aa"), 10), b"\x22\x00\x00\x00")

    def test_a_zero_length_read_is_an_empty_value(self):
        peer, _ = peer_for({"gatt.read": lines("@ok 1 len=0")})

        self.assertEqual(peer.ble.read(Link(0, "aa"), 10), b"")

    def test_a_length_that_does_not_match_the_payload_is_an_error(self):
        peer, _ = peer_for({"gatt.read": lines("@ok 1 len=4 data=22")})

        with self.assertRaises(InstrumentProtocolError) as caught:
            peer.ble.read(Link(0, "aa"), 10)
        self.assertIn("len=4", str(caught.exception))

    def test_write_sends_an_explicit_length(self):
        peer, console = peer_for({"gatt.write": lines("@ok 1")})

        peer.ble.write(Link(0, "aa"), 10, b"\x22\x00\x00\x00")

        self.assertIn("len=4 data=22000000", console.sent[-1])
        self.assertIn("response=1", console.sent[-1])

    def test_subscribe_sends_the_cccd_handle(self):
        peer, console = peer_for({"gatt.subscribe": lines("@ok 1")})

        peer.ble.subscribe(Link(0, "aa"), 11)

        self.assertIn("cccd=11", console.sent[-1])
        self.assertIn("enable=1", console.sent[-1])

    def test_a_notification_for_another_handle_is_kept(self):
        peer, _ = peer_for(
            initial=lines(
                "@evt 1 notify link=0 handle=10 len=2 data=aaaa",
                "@evt 2 notify link=0 handle=20 len=1 data=0b",
            )
        )

        # An explicit boundary asks for history; the default is "from now".
        self.assertEqual(peer.ble.wait_notification(20, since_seq=0), b"\x0b")
        self.assertEqual(peer.ble.wait_notification(10, since_seq=0), b"\xaa\xaa")

    def test_a_burst_is_consumed_when_the_caller_keeps_one_boundary(self):
        """Two packets in one read, one boundary: both arrive, in order."""
        peer, _ = peer_for(
            initial=lines(
                "@evt 1 notify link=0 handle=10 len=1 data=01",
                "@evt 2 notify link=0 handle=10 len=1 data=02",
            )
        )

        self.assertEqual(peer.ble.wait_notification(10, since_seq=0), b"")
        self.assertEqual(peer.ble.wait_notification(10, since_seq=0), b"")

    def test_a_notification_from_before_the_call_is_not_returned_by_default(self):
        peer, _ = peer_for(
            initial=lines("@evt 1 notify link=0 handle=10 len=2 data=aaaa")
        )

        with self.assertRaises(InstrumentTimeout):
            peer.ble.wait_notification(10, timeout=0.05)

    def test_a_missing_notification_times_out_with_the_transcript(self):
        peer, _ = peer_for(initial=lines("[I/ble_app] nothing useful"))

        with self.assertRaises(InstrumentTimeout) as caught:
            peer.ble.wait_notification(10, timeout=0.1)
        self.assertIn("nothing useful", str(caught.exception))


class TestLinkTuning(SdkTestCase):
    def test_update_connection_sends_the_requested_parameters(self):
        peer, console = peer_for({"gap.conn_params": lines("@ok 1")})

        peer.ble.update_connection(
            Link(1, "aa"),
            interval_min_ms=15,
            interval_max_ms=30,
            latency=0,
            timeout_ms=5000,
        )

        self.assertEqual(
            console.sent[-1],
            "peer req 1 gap.conn_params link=1 interval_min_ms=15 "
            "interval_max_ms=30 latency=0 timeout_ms=5000",
        )

    def test_set_phy_sends_the_phy_request(self):
        peer, console = peer_for({"gap.phy": lines("@ok 1 rx_phy=2 tx_phy=2")})

        peer.ble.set_phy(Link(1, "aa"), 2)

        self.assertEqual(console.sent[-1], "peer req 1 gap.phy link=1 phy=2")

    def test_set_data_length_sends_the_requested_octets(self):
        peer, console = peer_for({"gap.data_len": lines("@ok 1")})

        peer.ble.set_data_length(Link(1, "aa"), 251, 2120)

        self.assertEqual(
            console.sent[-1],
            "peer req 1 gap.data_len link=1 tx_octets=251 tx_time=2120",
        )

    def test_set_advertising_data_encodes_the_payload(self):
        peer, console = peer_for({"adv.data": lines("@ok 1 len=2")})

        peer.ble.set_advertising_data(b"\x01\x02")

        self.assertEqual(
            console.sent[-1], "peer req 1 adv.data len=2 data=0102"
        )

    def test_a_silent_phy_request_times_out(self):
        peer, _ = peer_for({"gap.phy": ""})

        with self.assertRaises(InstrumentTimeout):
            peer.ble.set_phy(Link(1, "aa"), 2, timeout=0.1)


class TestServerAndSecurity(SdkTestCase):
    def test_register_loopback_returns_its_value_handle(self):
        peer, _ = peer_for({"svc.register": lines("@ok 1 handle=12")})

        self.assertEqual(peer.ble.register_loopback(), 12)

    def test_bond_passes_the_io_capability(self):
        peer, console = peer_for({"sec.bond": lines("@ok 1 link=0 io=3")})

        peer.ble.bond(Link(0, "aa"), io="display_only", passkey=123456)

        self.assertIn("io=display_only", console.sent[-1])
        self.assertIn("passkey=123456", console.sent[-1])

    def test_bond_passes_the_authentication_requirement(self):
        peer, console = peer_for({"sec.bond": lines("@ok 1 link=0 io=3")})

        peer.ble.bond(Link(0, "aa"), auth="legacy")

        self.assertIn("auth=legacy", console.sent[-1])

    def test_a_bond_the_instrument_cannot_complete_is_an_error(self):
        """The instrument answers only after pairing; a failure is an @err."""
        peer, _ = peer_for(
            {"sec.bond": lines("@err 1 bond_failed sec.bond rc=107")}
        )

        with self.assertRaises(InstrumentCommandError) as caught:
            peer.ble.bond(Link(0, "aa"))
        self.assertEqual(caught.exception.code, "bond_failed")


class TestCapability(SdkTestCase):
    def test_a_ble_peer_serves_the_ble_capability(self):
        peer, _ = peer_for()

        self.assertIsNotNone(peer.ble)

    def test_asking_for_classic_from_a_ble_image_fails_at_construction(self):
        with self.assertRaises(InstrumentCapabilityError):
            peer_for(capability="classic")

    def test_a_dual_capability_peer_serves_both_halves(self):
        peer, _ = peer_for(capability="ble,classic", caps="ble,classic")

        self.assertIsNotNone(peer.ble)
        self.assertIsNotNone(peer.classic)

    def test_a_ble_only_peer_refuses_the_classic_half(self):
        peer, _ = peer_for()

        with self.assertRaises(InstrumentCapabilityError) as caught:
            peer.classic
        self.assertIn("classic", str(caught.exception))


if __name__ == "__main__":
    unittest.main()
