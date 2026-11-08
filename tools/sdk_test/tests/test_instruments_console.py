# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the instrument line protocol.

These run against a scripted console, not a board. The point is that a change
to the firmware's answer shape fails here, in a unit test, instead of hours
later in a lab run.
"""

from __future__ import annotations

import unittest

from _instrument_support import FakeLineConsole
from _instrument_support import info_line
from _instrument_support import lines
from _support import SdkTestCase

from sdk_test.instruments.console import EVENT_LIMIT
from sdk_test.instruments.console import InstrumentConsole
from sdk_test.instruments.errors import InstrumentCapabilityError
from sdk_test.instruments.errors import InstrumentCommandError
from sdk_test.instruments.errors import InstrumentProtocolError
from sdk_test.instruments.errors import InstrumentTimeout


class TestRequests(SdkTestCase):
    def test_a_ok_is_matched_by_id(self):
        console = FakeLineConsole({"peer req 1 gap.scan": lines("@ok 1 count=2")})
        instrument = InstrumentConsole(console)

        self.assertEqual(instrument.command("gap.scan"), {"count": "2"})

    def test_an_err_raises_with_the_code_and_text(self):
        console = FakeLineConsole({"peer req 1 gap.scan": lines("@err 1 busy pending")})
        instrument = InstrumentConsole(console)

        with self.assertRaises(InstrumentCommandError) as caught:
            instrument.command("gap.scan")
        self.assertEqual(caught.exception.code, "busy")
        self.assertEqual(caught.exception.text, "pending")
        self.assertIn("busy", str(caught.exception))
        self.assertIn("pending", str(caught.exception))

    def test_bytes_become_len_and_hex(self):
        console = FakeLineConsole({"peer req 1 gatt.write": lines("@ok 1")})
        instrument = InstrumentConsole(console)

        instrument.command("gatt.write", link=0, handle=10, response=1, data=b"\x22\x00")

        self.assertEqual(
            console.sent,
            ["peer req 1 gatt.write link=0 handle=10 response=1 len=2 data=2200"],
        )

    def test_a_timeout_names_the_command_and_the_transcript(self):
        console = FakeLineConsole({"peer req 1 gap.scan": ""})
        instrument = InstrumentConsole(console)

        with self.assertRaises(InstrumentTimeout) as caught:
            instrument.command("gap.scan", timeout=0.1)
        self.assertIn("gap.scan", str(caught.exception))
        self.assertIn("peer req 1", str(caught.exception))

    def test_a_late_response_is_dropped_not_matched_to_the_next_request(self):
        console = FakeLineConsole(
            {
                "peer req 1 gap.scan": "",
                "peer req 2 gap.scan": lines("@ok 1 count=0", "@ok 2 count=7"),
            }
        )
        instrument = InstrumentConsole(console)
        with self.assertRaises(InstrumentTimeout):
            instrument.command("gap.scan", timeout=0.1)

        # The late `@ok 1` arrives in the same read as the answer to request 2.
        self.assertEqual(instrument.command("gap.scan"), {"count": "7"})
        self.assertEqual(instrument.late_responses, 1)

    def test_a_caller_supplied_length_is_discarded(self):
        console = FakeLineConsole({"peer req 1 gatt.write": lines("@ok 1")})
        instrument = InstrumentConsole(console)

        instrument.command("gatt.write", data=b"\x01\x02", len=99)

        self.assertEqual(console.sent, ["peer req 1 gatt.write len=2 data=0102"])

    def test_a_non_bytes_data_argument_is_refused(self):
        instrument = InstrumentConsole(FakeLineConsole())

        with self.assertRaises(InstrumentProtocolError):
            instrument.command("gatt.write", data="not bytes")

    def test_an_over_long_line_is_refused_before_sending(self):
        console = FakeLineConsole(
            {"peer info": info_line(caps="ble", max_line=64)}
        )
        instrument = InstrumentConsole(console)
        instrument.expect_ready("ble")

        with self.assertRaises(InstrumentProtocolError) as caught:
            instrument.command("gatt.write", data=b"\x00" * 40)

        self.assertIn("max_line", str(caught.exception))
        self.assertEqual(instrument.sent, ["peer info"])

    def test_an_ansi_prefixed_ok_is_still_matched(self):
        # ulog's colour reset is emitted before the next line, so a protocol
        # line can arrive as "\x1b[0m@ok ...".
        console = FakeLineConsole(
            {"peer req 1 gap.scan": "\x1b[32m[I/log] noise\r\n\x1b[0m@ok 1 count=0\n"}
        )
        instrument = InstrumentConsole(console)

        self.assertEqual(instrument.command("gap.scan"), {"count": "0"})

    def test_an_ok_glued_after_the_shell_prompt_is_matched(self):
        # The prompt has no trailing newline, so a fast response can land on
        # the same line: "msh />@ok ...".
        console = FakeLineConsole({"peer req 1 gap.scan": "msh />@ok 1 count=0\n"})
        instrument = InstrumentConsole(console)

        self.assertEqual(instrument.command("gap.scan"), {"count": "0"})

    def test_an_unknown_id_is_counted_not_stored(self):
        console = FakeLineConsole({"peer req 1 gap.scan": lines("@ok 9 count=0")})
        instrument = InstrumentConsole(console)

        with self.assertRaises(InstrumentTimeout):
            instrument.command("gap.scan", timeout=0.1)
        self.assertEqual(instrument.stray_responses, 1)


class TestReady(SdkTestCase):
    def test_info_requires_proto_one_and_the_declared_capability(self):
        console = FakeLineConsole({"peer info": info_line(caps="ble,classic")})
        instrument = InstrumentConsole(console)

        info = instrument.expect_ready("ble")

        self.assertEqual(info["proto"], "1")
        self.assertIn("classic", info["caps"])

    def test_an_old_image_without_proto_is_refused(self):
        console = FakeLineConsole({"peer info": "@info fw=diss-image\n"})
        instrument = InstrumentConsole(console)

        with self.assertRaises(InstrumentCapabilityError) as caught:
            instrument.expect_ready("ble")
        self.assertIn("proto=1", str(caught.exception))

    def test_a_missing_capability_is_refused(self):
        console = FakeLineConsole({"peer info": info_line(caps="ble")})
        instrument = InstrumentConsole(console)

        with self.assertRaises(InstrumentCapabilityError) as caught:
            instrument.expect_ready("classic")
        self.assertIn("classic", str(caught.exception))

    def test_a_comma_separated_capability_requires_every_entry(self):
        console = FakeLineConsole({"peer info": info_line(caps="ble")})
        instrument = InstrumentConsole(console)

        with self.assertRaises(InstrumentCapabilityError) as caught:
            instrument.expect_ready("ble,classic")
        self.assertIn("classic", str(caught.exception))

    def test_all_of_a_dual_capability_set_is_accepted(self):
        console = FakeLineConsole({"peer info": info_line(caps="ble,classic")})
        instrument = InstrumentConsole(console)

        info = instrument.expect_ready("ble,classic")

        self.assertEqual(info["caps"], "ble,classic")


class TestEvents(SdkTestCase):
    def test_events_are_parsed_and_returned_in_order(self):
        console = FakeLineConsole(
            initial=lines(
                "@evt 1 adv index=0 addr=aa rssi=-40",
                "@evt 2 adv index=1 addr=bb rssi=-30",
            )
        )
        instrument = InstrumentConsole(console)

        first = instrument.next_event("adv")
        second = instrument.next_event("adv")

        self.assertEqual(first["seq"], 1)
        self.assertEqual(second["values"]["addr"], "bb")

    def test_a_non_matching_event_stays_queued(self):
        console = FakeLineConsole(
            initial=lines(
                "@evt 1 notify link=0 handle=10 len=1 data=aa",
                "@evt 2 notify link=0 handle=20 len=1 data=bb",
            )
        )
        instrument = InstrumentConsole(console)

        event = instrument.next_event("notify", handle=20)

        self.assertEqual(event["values"]["data"], "bb")
        remaining = instrument.next_event("notify")
        self.assertEqual(remaining["values"]["handle"], "10")

    def test_since_seq_ignores_events_from_before_the_call(self):
        console = FakeLineConsole(initial=lines("@evt 5 adv index=0 addr=old"))
        instrument = InstrumentConsole(console)
        instrument.drain()
        start = instrument.last_seq

        with self.assertRaises(InstrumentTimeout):
            instrument.next_event("adv", since_seq=start, timeout=0.1)

        console.feed(lines("@evt 6 adv index=1 addr=new"))
        event = instrument.next_event("adv", since_seq=start)
        self.assertEqual(event["values"]["addr"], "new")

    def test_clearing_events_drops_what_came_before_a_reset(self):
        console = FakeLineConsole(initial=lines("@evt 1 adv index=0 addr=aa"))
        instrument = InstrumentConsole(console)
        instrument.drain()

        instrument.clear_events()

        with self.assertRaises(InstrumentTimeout):
            instrument.next_event("adv", timeout=0.1)

    def test_draining_text_keeps_queued_events(self):
        console = FakeLineConsole(initial=lines("@evt 1 adv index=0 addr=aa"))
        instrument = InstrumentConsole(console)

        instrument.drain()

        self.assertEqual(instrument.next_event("adv")["values"]["addr"], "aa")

    def test_an_event_flood_drops_the_oldest_and_counts_it(self):
        flood = lines(
            *[
                f"@evt {index} adv index={index} addr=aa"
                for index in range(1, EVENT_LIMIT + 11)
            ]
        )
        console = FakeLineConsole(initial=flood)
        instrument = InstrumentConsole(console)

        instrument.drain()

        self.assertEqual(instrument.event_overflow, 10)
        # The ten oldest are gone; the first survivor is seq 11.
        self.assertEqual(instrument.next_event("adv")["seq"], 11)

    def test_a_line_split_across_reads_is_parsed_whole(self):
        console = FakeLineConsole(initial="@evt 1 adv ind")
        instrument = InstrumentConsole(console)
        instrument.drain()
        with self.assertRaises(InstrumentTimeout):
            instrument.next_event("adv", timeout=0.05)

        console.feed("ex=0 addr=aa\r\r\n")

        self.assertEqual(instrument.next_event("adv")["values"]["addr"], "aa")

    def test_an_event_for_another_link_stays_queued(self):
        console = FakeLineConsole(
            initial=lines(
                "@evt 1 char link=1 index=0 handle=9 value_handle=10",
                "@evt 2 char link=0 index=0 handle=5 value_handle=6",
            )
        )
        instrument = InstrumentConsole(console)
        instrument.drain()

        mine = instrument.next_event("char", link=0)

        self.assertEqual(mine["values"]["handle"], "5")
        other = instrument.next_event("char", link=1)
        self.assertEqual(other["values"]["handle"], "9")

    def test_an_empty_payload_event_is_parsed(self):
        console = FakeLineConsole(initial=lines("@evt 7 notify link=0 handle=10 len=0 data="))
        instrument = InstrumentConsole(console)

        event = instrument.next_event("notify", handle=10)

        self.assertEqual(event["name"], "notify")
        self.assertEqual(event["values"]["len"], "0")

    def test_a_malformed_protocol_line_is_an_error(self):
        console = FakeLineConsole(initial="@evt nope adv\n")
        instrument = InstrumentConsole(console)

        with self.assertRaises(InstrumentProtocolError):
            instrument.drain()

    def test_an_event_glued_after_the_shell_prompt_is_parsed(self):
        console = FakeLineConsole(initial="msh />@evt 1 adv index=0 addr=aa\n")
        instrument = InstrumentConsole(console)

        instrument.drain()

        self.assertEqual(instrument.next_event("adv")["values"]["addr"], "aa")

    def test_an_ansi_prefixed_event_is_parsed(self):
        console = FakeLineConsole(initial="\x1b[0m@evt 1 adv index=0 addr=aa\n")
        instrument = InstrumentConsole(console)

        instrument.drain()

        self.assertEqual(instrument.next_event("adv")["values"]["addr"], "aa")

    def test_a_protocol_marker_inside_a_log_line_is_not_an_answer(self):
        """A ulog line that merely mentions `@ok` must not satisfy a request."""
        console = FakeLineConsole(
            {
                "peer req 1 gap.scan": lines(
                    "[D/sibles] echoed: @ok 1 count=9",
                    "@ok 1 count=2",
                )
            }
        )
        instrument = InstrumentConsole(console)

        self.assertEqual(instrument.command("gap.scan"), {"count": "2"})


class TestCommandEvents(SdkTestCase):
    def test_it_consumes_exactly_the_promised_events_from_the_firmware_boundary(self):
        console = FakeLineConsole(
            {
                "peer req 1 gap.scan": lines(
                    "@evt 1 adv index=9 addr=stale",
                    "@evt 2 adv index=1 addr=fresh",
                    "@ok 1 count=1 first_seq=1",
                )
            }
        )
        instrument = InstrumentConsole(console)

        result, events, _ = instrument.command_events("gap.scan", "adv")

        self.assertEqual(result["count"], "1")
        self.assertEqual([event["values"]["addr"] for event in events], ["fresh"])

    def test_a_reply_without_a_boundary_is_an_error(self):
        console = FakeLineConsole({"peer req 1 gap.scan": lines("@ok 1 count=0")})
        instrument = InstrumentConsole(console)

        with self.assertRaises(InstrumentProtocolError) as caught:
            instrument.command_events("gap.scan", "adv")
        self.assertIn("first_seq", str(caught.exception))

    def test_a_reply_without_a_count_is_an_error(self):
        console = FakeLineConsole({"peer req 1 gap.scan": lines("@ok 1 first_seq=0")})
        instrument = InstrumentConsole(console)

        with self.assertRaises(InstrumentProtocolError) as caught:
            instrument.command_events("gap.scan", "adv")
        self.assertIn("count", str(caught.exception))

    def test_the_host_boundary_is_available_for_firmware_without_one(self):
        console = FakeLineConsole(
            {
                "peer req 1 btc.inquiry": lines(
                    "@evt 2 btc.dev index=1 addr=aabbcc",
                    "@ok 1 count=1",
                )
            },
            initial=lines("@evt 1 btc.dev index=9 addr=stale"),
        )
        instrument = InstrumentConsole(console)
        instrument.drain()

        _, events, _ = instrument.command_events(
            "btc.inquiry", "btc.dev", boundary="host"
        )

        self.assertEqual([event["values"]["addr"] for event in events], ["aabbcc"])


class TestBookkeeping(SdkTestCase):
    def test_events_before_the_boundary_are_discarded_not_just_skipped(self):
        console = FakeLineConsole(
            {
                "peer req 1 gap.scan": lines(
                    "@evt 6 adv index=1 addr=fresh",
                    "@ok 1 count=1 first_seq=5",
                )
            },
            initial=lines(
                "@evt 1 adv index=1 addr=old",
                "@evt 2 adv index=2 addr=older",
            ),
        )
        instrument = InstrumentConsole(console)
        instrument.drain()

        _, events, _ = instrument.command_events("gap.scan", "adv")

        self.assertEqual([event["values"]["addr"] for event in events], ["fresh"])
        self.assertEqual(instrument.stale_events, 2)

    def test_a_second_answer_for_one_request_is_counted(self):
        console = FakeLineConsole(
            {"peer req 1 gap.scan": lines("@ok 1 count=0", "@ok 1 count=7")}
        )
        instrument = InstrumentConsole(console)

        self.assertEqual(instrument.command("gap.scan"), {"count": "0"})
        instrument.drain()

        self.assertEqual(instrument.duplicate_responses, 1)

    def test_a_second_answer_after_the_first_was_collected_is_counted(self):
        """The leak the review found: the duplicate arrives after the pop."""
        console = FakeLineConsole({"peer req 1 gap.scan": lines("@ok 1 count=0")})
        instrument = InstrumentConsole(console)
        self.assertEqual(instrument.command("gap.scan"), {"count": "0"})

        console.feed(lines("@ok 1 count=9"))
        instrument.drain()

        self.assertEqual(instrument.duplicate_responses, 1)
        self.assertEqual(instrument._responses, {})


    def test_ordinary_log_lines_are_ignored(self):
        console = FakeLineConsole(
            initial=lines(
                "[I/ble_app] starting",
                "msh />",
                "@evt 1 adv index=0 addr=aa",
            )
        )
        instrument = InstrumentConsole(console)

        instrument.drain()

        self.assertEqual(instrument.next_event("adv")["values"]["addr"], "aa")


if __name__ == "__main__":
    unittest.main()
