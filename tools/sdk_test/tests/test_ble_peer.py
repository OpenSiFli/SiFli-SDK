# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for ``sdk_test.ble_peer`` -- the instrument's protocol layer.

The transcripts here are the shape the example actually prints (its ``LOG_I``
format strings, and ulog's fixed-width hexdump), fed to the parsers and to
``BlePeer`` over a scripted console. No board is involved, which is the point:
the day the example changes a log line, this file goes red instead of a lab run
somewhere else.
"""

from __future__ import annotations

import unittest

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from sdk_test.ble_peer import Advertisement
from sdk_test.ble_peer import BlePeer
from sdk_test.ble_peer import parse_advertisements
from sdk_test.ble_peer import parse_connected_devices
from sdk_test.ble_peer import parse_hexdump
from sdk_test.ble_peer import parse_service_listing


def dump_line(tag: str, offset: int, data: bytes, width: int = 16, ascii_tail: str = "....") -> str:
    """Reproduce ``ulog_hexdump``'s line layout for one row."""
    hex_part = ""
    for position in range(width):
        if position < len(data):
            hex_part += f"{data[position]:02X} "
        else:
            hex_part += "   "
        if (position + 1) % 8 == 0:
            hex_part += " "
    return (
        f"[I/ble_app] D/HEX {tag}: {offset:04X}-{offset + width:04X}: "
        f"{hex_part}  {ascii_tail}\r\n"
    )


def dump_lines(tag: str, data: bytes, width: int = 16) -> str:
    """Reproduce a whole ``ulog_hexdump`` -- one line per 16-byte row."""
    return "".join(
        dump_line(tag, offset, data[offset: offset + width], width)
        for offset in range(0, len(data), width)
    )


SCAN_TRANSCRIPT = (
    "[I/ble_app] Scan start status 0\r\n"
    "[I/ble_app] advertising device  0: addr type: public, addr: 1-2-3-4-5-6\r\n"
    "[I/ble_app] adv type: ext adv, adv info: connectable  adv\r\n"
    "[I/ble_app] adv tx pwr: 0, rssi: -61\r\n"
    "[I/ble_app] adv_data\r\n"
    "[I/ble_app] advertising device  1: addr type: public, addr: a-b-c-d-e-f\r\n"
    "[I/ble_app] adv type: legacy adv, adv info: connectable  adv\r\n"
    "[I/ble_app] adv tx pwr: 0, rssi: -33\r\n"
    "[I/ble_app] Scan stopped 19\r\n"
)

SERVICE_TRANSCRIPT = (
    "[I/ble_app] Service UUID(128 bit):\r\n"
    "[I/ble_app] Service UUID: 0x7369666c695f61707000000000000000\r\n"
    "[I/ble_app]     Charateristic(0) handle 9, value handle 10\r\n"
    "[I/ble_app]     Charateristic UUID(128 bit):\r\n"
    "[I/ble_app]     Properties: Read Write Write_wo_rsp Notify\r\n"
    "[I/ble_app]         Descriptor(0) handle 11\r\n"
    "[I/ble_app]         Descriptor UUID(2 bit):\r\n"
)


class FakeConsole:
    """A shell-like console: a line in, the canned reply for its prefix out.

    Command-driven rather than a fixed chunk script, because the helpers send a
    command and then read -- and ``_settle`` polls in between, so a fixed
    sequence would hand the next reply to the previous call and leave the real
    one starved.
    """

    def __init__(self, replies=None, initial: str = ""):
        self.replies = dict(replies or {})
        self.sent = []
        self._out = initial
        self._position = 0

    def feed(self, text: str) -> None:
        self._out += text

    def send_line(self, line):
        self.sent.append(line)
        for trigger, reply in self.replies.items():
            if trigger in line:
                # A list is consumed one entry per send, so a test can make the
                # first answer and the second silence -- which is how a stale
                # match shows up.
                if isinstance(reply, list):
                    reply = reply.pop(0) if reply else ""
                self._out += reply

    def read(self, timeout=1.0):
        chunk = self._out[self._position:]
        self._position = len(self._out)
        return chunk

    def drain(self):
        self._position = len(self._out)


class TestAdvertisements(SdkTestCase):
    def test_reports_carry_index_address_and_rssi(self):
        advertisements = parse_advertisements(SCAN_TRANSCRIPT)
        self.assertEqual(len(advertisements), 2)
        self.assertEqual(advertisements[0].index, 0)
        self.assertEqual(advertisements[0].address, "1-2-3-4-5-6")
        self.assertEqual(advertisements[0].address_type, "public")
        self.assertEqual(advertisements[0].rssi, -61)
        self.assertEqual(advertisements[1].rssi, -33)

    def test_a_report_without_its_rssi_line_still_parses(self):
        # A console chunk boundary can split the three-line report; the index
        # and address are still what a connection request needs, but the signal
        # is unknown -- and unknown must not become 0 dBm.
        text = "[I/ble_app] advertising device  3: addr type: random, addr: 7-7-7\r\n"
        (advertisement,) = parse_advertisements(text)
        self.assertEqual(advertisement.index, 3)
        self.assertEqual(advertisement.address_type, "random")
        self.assertIsNone(advertisement.rssi)

    def test_scan_sorts_an_unmeasured_report_last(self):
        transcript = (
            "[I/ble_app] Scan start status 0\r\n"
            "[I/ble_app] advertising device  1: addr type: public, addr: aa-aa\r\n"
            "[I/ble_app] advertising device  2: addr type: public, addr: bb-bb\r\n"
            "[I/ble_app] adv tx pwr: 0, rssi: -80\r\n"
            "[I/ble_app] Scan stopped 19\r\n"
        )
        peer = BlePeer(FakeConsole(replies={"diss scan start": transcript}))
        advertisements = peer.scan(duration_ms=1000)
        self.assertEqual([adv.address for adv in advertisements], ["bb-bb", "aa-aa"])


class TestServiceListing(SdkTestCase):
    def test_characteristics_and_their_cccd_come_out(self):
        listing = parse_service_listing(SERVICE_TRANSCRIPT)
        self.assertEqual(len(listing.characteristics), 1)
        characteristic = listing.characteristics[0]
        self.assertEqual(characteristic.handle, 9)
        self.assertEqual(characteristic.value_handle, 10)
        self.assertEqual(characteristic.descriptors, [11])
        self.assertEqual(listing.value_handle(), 10)
        self.assertEqual(listing.cccd_handle(), 11)

    def test_a_characteristic_without_a_descriptor_has_no_cccd(self):
        listing = parse_service_listing(
            "[I/ble_app]     Charateristic(0) handle 9, value handle 10\r\n"
        )
        with self.assertRaises(AssertionError):
            listing.cccd_handle()


class TestConnectedDevices(SdkTestCase):
    def test_the_table_maps_addresses_to_device_indexes(self):
        table = parse_connected_devices(
            "[I/ble_app] Total connected 2 devices\r\n"
            "[I/ble_app] Device 0: role:0, address(type 1): d2-6e-6a-8b-96-7c\r\n"
            "[I/ble_app] MTU: 527, connection interval 60\r\n"
            "[I/ble_app] Device 1: role:0, address(type 0): 5a-1d-0-52-fd-5c\r\n"
        )
        self.assertEqual(table["d2-6e-6a-8b-96-7c"], 0)
        self.assertEqual(table["5a-1d-0-52-fd-5c"], 1)

    def test_an_empty_table_parses_to_nothing(self):
        self.assertEqual(parse_connected_devices("Total connected 0 devices\r\n"), {})


class TestHexdump(SdkTestCase):
    def test_two_rows_are_joined_in_order(self):
        text = dump_line("Notify content", 0, bytes(range(16))) + dump_line(
            "Notify content", 16, bytes([16, 17, 18, 19])
        )
        self.assertEqual(parse_hexdump(text, "Notify content"), bytes(range(20)))

    def test_the_devices_cr_cr_lf_line_end_is_handled(self):
        # A format guard, not a regression test: the old splitlines-based code
        # also handled this. What it pins is that the new "terminated row" rule
        # keeps working with the device's real line ending.
        first = dump_line("Notify content", 0, bytes([0xAA]) * 16)
        second = dump_line("Notify content", 16, bytes([0xBB]) * 4)
        text = first.replace("\r\n", "\r\r\n") + second.replace("\r\n", "\r\r\n")
        self.assertEqual(
            parse_hexdump(text, "Notify content"), bytes([0xAA]) * 16 + bytes([0xBB]) * 4
        )

    def test_a_row_without_its_newline_is_not_parsed(self):
        # A console read can end mid-row; parsing that fragment would truncate
        # the payload and mark it consumed, so the rest could never be recovered.
        first = dump_line("Notify content", 0, bytes(range(16)))
        partial = "                      0010-0020: 10 11 12 13"
        self.assertEqual(
            parse_hexdump(first + partial, "Notify content"), bytes(range(16))
        )

    def test_the_ascii_column_is_not_mistaken_for_hex(self):
        # The payload's ASCII rendering starts with text that reads like hex
        # byte pairs ("AB CD E ..."); slicing the documented fixed width is what
        # keeps it out of the returned payload.
        payload = b"AB CD E ABCDE "
        text = dump_line("Read content", 0, payload, ascii_tail="AB CD E ABCDE   ")
        self.assertEqual(parse_hexdump(text, "Read content"), payload)

    def test_a_missing_tag_yields_empty(self):
        self.assertEqual(parse_hexdump(SCAN_TRANSCRIPT, "Notify content"), b"")


class TestBlePeerOverAFakeConsole(SdkTestCase):
    def test_scan_sorts_strongest_first(self):
        console = FakeConsole(replies={"diss scan start": SCAN_TRANSCRIPT})
        peer = BlePeer(console)

        advertisements = peer.scan(duration_ms=3000)

        self.assertEqual([adv.address for adv in advertisements], ["a-b-c-d-e-f", "1-2-3-4-5-6"])
        self.assertEqual(console.sent, ["diss scan start 0 100 50 3000 -127"])

    def test_an_empty_scan_says_what_to_check(self):
        console = FakeConsole(
            replies={
                "diss scan start": "[I/ble_app] Scan start status 0\r\n"
                "[I/ble_app] Scan stopped 19\r\n"
            }
        )
        peer = BlePeer(console)
        with self.assertRaises(AssertionError) as caught:
            peer.strongest(peer.scan(duration_ms=1000))
        self.assertIn("advertising", str(caught.exception))

    def test_connect_uses_the_scan_index_and_finds_the_link(self):
        console = FakeConsole(
            replies={
                "diss conn_idx start": "[I/ble_app] Peer device(role:1) (a-b-c) "
                "connected as deivce 0\r\n",
                "diss show_dev": "[I/ble_app] Total connected 1 devices\r\n"
                "[I/ble_app] Device 0: role:0, address(type 0): a-b-c\r\n",
            }
        )
        peer = BlePeer(console)
        peer.connect(Advertisement(index=4, address="a-b-c", address_type="public", rssi=-40))
        # The scan prints the index after appending the report, so the printed
        # number is the one `conn_idx start` takes -- no +1.
        self.assertEqual(console.sent[0], "diss conn_idx start 4 0 5000 30 100 50")
        self.assertEqual(peer.dev_index, 0)

    def test_the_device_index_comes_from_the_table_not_from_zero(self):
        # An instrument may already hold a link to something else; the new link
        # is then device 1, and every GATT command must use that.
        console = FakeConsole(
            replies={
                "diss conn_idx start": "[I/ble_app] Peer device(role:1) (a-b-c) "
                "connected as deivce 1\r\n",
                "diss show_dev": "[I/ble_app] Total connected 2 devices\r\n"
                "[I/ble_app] Device 0: role:0, address(type 1): d2-6e-6a-8b-96-7c\r\n"
                "[I/ble_app] Device 1: role:0, address(type 0): a-b-c\r\n",
            }
        )
        peer = BlePeer(console)
        peer.connect(Advertisement(index=4, address="a-b-c", address_type="public", rssi=-40))
        self.assertEqual(peer.dev_index, 1)

    def test_a_connection_that_never_shows_up_in_the_table_is_reported(self):
        console = FakeConsole(
            replies={
                "diss conn_idx start": "[I/ble_app] Peer device(role:1) (a-b-c) "
                "connected as deivce 0\r\n",
                "diss show_dev": "[I/ble_app] Total connected 1 devices\r\n"
                "[I/ble_app] Device 0: role:0, address(type 1): 1-2-3\r\n",
            }
        )
        peer = BlePeer(console)
        with self.assertRaises(AssertionError) as caught:
            peer.connect(
                Advertisement(index=4, address="a-b-c", address_type="public", rssi=-40),
                timeout=0.3,
            )
        self.assertIn("does not list a-b-c", str(caught.exception))

    def test_a_refused_connection_is_a_failure(self):
        console = FakeConsole(
            replies={
                "diss conn_idx start": "[E/ble_app] Create connection failed 3!\r\n"
            }
        )
        peer = BlePeer(console)
        with self.assertRaises(AssertionError) as caught:
            peer.connect(Advertisement(index=1, address="a", address_type="public", rssi=-40))
        self.assertIn("Create connection failed", str(caught.exception))

    def test_search_service_waits_for_the_registration(self):
        console = FakeConsole(
            replies={
                "diss search_svc": "[I/ble_app] Service searched\r\n"
                + SERVICE_TRANSCRIPT
                + "[I/ble_app] client register ret 0\r\n"
            }
        )
        peer = BlePeer(console)

        listing = peer.search_service()

        self.assertEqual(listing.value_handle(), 10)
        self.assertEqual(console.sent[-1].split()[0:2], ["diss", "search_svc"])

    def test_a_failed_service_registration_is_a_failure(self):
        console = FakeConsole(
            replies={
                "diss search_svc": "[I/ble_app] Service searched\r\n"
                "[I/ble_app] client register ret 3\r\n"
            }
        )
        peer = BlePeer(console)
        with self.assertRaises(AssertionError) as caught:
            peer.search_service()
        self.assertIn("client register ret 3", str(caught.exception))

    def test_read_value_returns_the_dumped_bytes(self):
        console = FakeConsole(
            replies={
                "diss read_val": "[I/ble_app] Read(handle 10) received\r\n"
                + dump_line("Read content", 0, bytes([0x22, 0, 0, 0]))
            }
        )
        peer = BlePeer(console)
        self.assertEqual(peer.read_value(10), bytes([0x22, 0, 0, 0]))
        self.assertEqual(console.sent, ["diss read_val 0 10"])

    def test_a_zero_length_read_is_an_empty_value_not_a_timeout(self):
        # The firmware prints the response line, and ulog_hexdump prints nothing
        # at all for a zero-byte value -- so waiting for "Read content" would
        # turn a legal empty value into a failure.
        console = FakeConsole(
            replies={"diss read_val": "[I/ble_app] Read(handle 10) received\r\n"}
        )
        peer = BlePeer(console)
        self.assertEqual(peer.read_value(10), b"")

    def test_an_empty_write_is_refused_before_it_reaches_the_device(self):
        console = FakeConsole()
        peer = BlePeer(console)
        with self.assertRaises(ValueError):
            peer.write_value(10, b"")
        self.assertEqual(console.sent, [])

    def test_a_write_the_firmware_rejected_is_a_failure(self):
        console = FakeConsole(
            replies={
                "diss write_val": dump_line("write context", 0, b"\x22\x00\x00\x00")
                + "[E/ble_app] Write remote value \r\n"
            }
        )
        peer = BlePeer(console)
        with self.assertRaises(AssertionError) as caught:
            peer.write_value(10, b"\x22\x00\x00\x00")
        self.assertIn("failed to write handle 10", str(caught.exception))

    def test_a_write_the_instrument_never_ran_is_a_failure(self):
        # No "write context" at all: the shell never processed the command
        # (hung instrument, wrong image, dead console). That used to pass.
        console = FakeConsole()
        peer = BlePeer(console)
        with self.assertRaises(AssertionError) as caught:
            peer.write_value(10, b"\x22\x00\x00\x00", timeout=0.3)
        self.assertIn("write context", str(caught.exception))

    def test_read_value_keeps_notifications_that_arrived_first(self):
        # Notifications and read responses share the console; a read must not
        # drain the packets the next wait_notification is owed.
        notification = (
            "[I/ble_app] Notify(handle 10) received\r\n"
            + dump_lines("Notify content", bytes([0x05]) * 20)
        )
        console = FakeConsole(
            initial=notification,
            replies={
                "diss read_val": "[I/ble_app] Read(handle 10) received\r\n"
                + dump_line("Read content", 0, bytes([0x22, 0, 0, 0]))
            },
        )
        peer = BlePeer(console)

        self.assertEqual(peer.read_value(10), bytes([0x22, 0, 0, 0]))
        self.assertEqual(peer.wait_notification(10), bytes([0x05]) * 20)

    def test_notifications_are_consumed_one_per_call(self):
        console = FakeConsole(
            initial=(
                "[I/ble_app] Notify(handle 10) received\r\n"
                + dump_lines("Notify content", bytes([0x00]) * 20)
                + "[I/ble_app] Notify(handle 10) received\r\n"
                + dump_lines("Notify content", bytes([0x01]) * 20)
            )
        )
        peer = BlePeer(console)

        self.assertEqual(peer.wait_notification(10), bytes([0x00]) * 20)
        self.assertEqual(peer.wait_notification(10), bytes([0x01]) * 20)

    def test_a_second_expect_shell_waits_for_a_new_answer(self):
        console = FakeConsole(
            replies={"diss help": ["[I/ble_app] Command list:\r\n", ""]}
        )
        peer = BlePeer(console)
        peer.expect_shell()
        # The first command list is still in the buffer; matching it again would
        # "pass" against an instrument that has since died.
        with self.assertRaises(AssertionError):
            peer.expect_shell(timeout=0.2)

    def test_a_second_subscribe_does_not_match_the_first_confirmation(self):
        # The first confirmation stays in the buffer (notifications are kept
        # there); the second call matches only text after that point.
        console = FakeConsole(
            replies={
                "diss write_cccd": ["current_conn_idx 1, conn_idx 0\r\n", ""]
            }
        )
        peer = BlePeer(console)
        peer.subscribe(19)
        with self.assertRaises(AssertionError):
            peer.subscribe(20, timeout=0.3)

    def test_waiting_for_one_handle_keeps_another_handles_notification(self):
        # Every handle's event goes to the same console; waiting for 20 must not
        # discard the 10 that arrived first.
        console = FakeConsole(
            initial=(
                "[I/ble_app] Notify(handle 10) received\r\n"
                + dump_lines("Notify content", bytes([0x0A]) * 20)
                + "[I/ble_app] Notify(handle 20) received\r\n"
                + dump_lines("Notify content", bytes([0x14]) * 20)
            )
        )
        peer = BlePeer(console)

        self.assertEqual(peer.wait_notification(20), bytes([0x14]) * 20)
        self.assertEqual(peer.wait_notification(10), bytes([0x0A]) * 20)

    def test_subscribe_keeps_notifications_already_received(self):
        console = FakeConsole(
            initial=(
                "[I/ble_app] Notify(handle 10) received\r\n"
                + dump_lines("Notify content", bytes([0x05]) * 20)
            ),
            replies={
                "diss write_cccd": "[I/ble_app] current_conn_idx 1, conn_idx 0\r\n"
            },
        )
        peer = BlePeer(console)

        peer.subscribe(19)

        self.assertEqual(peer.wait_notification(10), bytes([0x05]) * 20)

    def test_notifications_that_arrive_with_the_write_are_kept(self):
        # The example answers the trigger write by notifying immediately, so the
        # packets land in the same read as the write's own log. Writing must not
        # consume them, or wait_notification waits for something already seen.
        console = FakeConsole(
            replies={
                "diss write_val": dump_line(
                    "write context", 0, bytes([0x11, 0x14, 0x05, 0x00])
                )
                + "[I/ble_app] Notify(handle 10) received\r\n"
                + dump_lines("Notify content", bytes([0x00]) * 20)
            }
        )
        peer = BlePeer(console)

        peer.write_value(10, bytes([0x11, 0x14, 0x05, 0x00]))

        self.assertEqual(peer.wait_notification(10), bytes([0x00]) * 20)

    def test_a_zero_length_notification_returns_empty_without_stealing_the_next(self):
        # ulog prints no dump for size 0; a complete next event line arriving
        # first is what says this one is empty. Reading the following
        # notification's payload as this one's would be worse than timing out.
        console = FakeConsole(
            initial=(
                "[I/ble_app] Notify(handle 10) received\r\n"
                "[I/ble_app] Notify(handle 10) received\r\n"
                + dump_lines("Notify content", bytes([0x09]) * 20)
            )
        )
        peer = BlePeer(console)

        self.assertEqual(peer.wait_notification(10, timeout=1.0), b"")
        self.assertEqual(peer.wait_notification(10), bytes([0x09]) * 20)

    def test_an_isolated_empty_notification_times_out_rather_than_guessing(self):
        # Nothing follows to prove the event carried no payload, and the log has
        # no length field: a truthful timeout beats returning b"" for a payload
        # that is merely late.
        console = FakeConsole(initial="[I/ble_app] Notify(handle 10) received\r\n")
        peer = BlePeer(console)
        with self.assertRaises(AssertionError) as caught:
            peer.wait_notification(10, timeout=0.3)
        self.assertIn("neither a complete payload nor a following event", str(caught.exception))

    def test_a_notification_arriving_with_the_subscribe_confirmation_is_kept(self):
        # A peripheral may notify the moment the CCCD is written; the event can
        # share the read with the confirmation line.
        console = FakeConsole(
            replies={
                "diss write_cccd": "[I/ble_app] current_conn_idx 1, conn_idx 0\r\n"
                "[I/ble_app] Notify(handle 10) received\r\n"
                + dump_lines("Notify content", bytes([0x07]) * 20)
            }
        )
        peer = BlePeer(console)

        peer.subscribe(19)

        self.assertEqual(peer.wait_notification(10), bytes([0x07]) * 20)

    def test_a_half_received_next_event_line_is_not_dropped(self):
        # The stream can pause inside the next "Notify(handle ...)" line. Losing
        # that partial line would make the following call wait for a
        # notification that had already been printed.
        first = (
            "[I/ble_app] Notify(handle 10) received\r\n"
            + dump_lines("Notify content", bytes([0x01]) * 20)
        )
        console = FakeConsole(initial=first + "[I/ble_app] Notify(hand")
        peer = BlePeer(console)

        self.assertEqual(peer.wait_notification(10), bytes([0x01]) * 20)

        console.feed(
            "le 10) received\r\n" + dump_lines("Notify content", bytes([0x02]) * 20)
        )
        self.assertEqual(peer.wait_notification(10), bytes([0x02]) * 20)

    def test_a_missing_notification_times_out_with_the_peer_transcript(self):
        console = FakeConsole(initial="nothing here\r\n")
        peer = BlePeer(console)
        with self.assertRaises(AssertionError) as caught:
            peer.wait_notification(10, timeout=0.2)
        self.assertIn("nothing here", str(caught.exception))


if __name__ == "__main__":
    unittest.main()
