# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for ``sdk_test.harness.console``.

Two properties are load-bearing and are pinned here deliberately:

* **chunk tolerance** -- ``RT_CONSOLEBUF_SIZE`` is 128 bytes, so a long line
  arrives split across reads. The matcher is fed the same log in 7-byte slices
  to prove an expectation split mid-line still matches.
* **ordering** -- a shuffled boot log must fail rather than pass by accident,
  which is the failure mode a console-parsing harness is most prone to.
"""

from __future__ import annotations

import unittest

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from sdk_test.errors import HarnessError
from sdk_test.harness.console import ConsoleMatcher
from sdk_test.harness.console import parse_record

#: A realistic RT-Thread boot log, including the noise the design doc warns
#: about: unrelated log lines interleaved with the lines under assertion.
BOOT_LOG = (
    " \r\n"
    " | / - RT -     Thread Operating System\r\n"
    " 2006 - 2024 Copyright by RT-Thread team\r\n"
    "[I/DBG] nor flash id 0x1820c2\r\n"
    "[I/DBG] heap: 262144\r\n"
    "Hello world!\r\n"
    "msh />"
)


def feed_in_chunks(matcher: ConsoleMatcher, text: str, size: int) -> None:
    """Feed ``text`` in fixed-size slices, mimicking serial reads."""
    for start in range(0, len(text), size):
        matcher.feed(text[start : start + size])


class TestChunkTolerance(SdkTestCase):
    def test_a_log_split_into_seven_byte_reads_still_matches(self):
        matcher = ConsoleMatcher(parse_record(["Thread Operating System", "Hello world!", r"msh />"]))
        feed_in_chunks(matcher, BOOT_LOG, 7)

        state = matcher.state()
        self.assertTrue(state.passed, state.detail)
        self.assertEqual((state.matched, state.expected), (3, 3))

    def test_an_expectation_split_mid_word_matches_once_completed(self):
        matcher = ConsoleMatcher(parse_record(["Hello world!"]))
        matcher.feed("Hel")
        self.assertFalse(matcher.state().passed, "must not pass before the rest arrives")
        matcher.feed("lo wor")
        matcher.feed("ld!")
        self.assertTrue(matcher.state().passed)

    def test_byte_at_a_time_feeding_converges(self):
        matcher = ConsoleMatcher(parse_record(["Hello world!", r"msh />"]))
        feed_in_chunks(matcher, BOOT_LOG, 1)
        self.assertTrue(matcher.state().passed)


class TestFailureIsNotVacuous(SdkTestCase):
    def test_a_wrong_expectation_fails(self):
        matcher = ConsoleMatcher(parse_record(["Hello worldXYZ!"]))
        matcher.feed(BOOT_LOG)

        state = matcher.state()
        self.assertFalse(state.passed)
        self.assertIn("missing console expectation", state.detail)
        self.assertIn("Hello worldXYZ!", state.detail)

    def test_an_empty_stream_fails(self):
        matcher = ConsoleMatcher(parse_record(["Hello world!"]))
        self.assertFalse(matcher.state().passed)


class TestOrdering(SdkTestCase):
    def test_declared_order_is_enforced_across_entries(self):
        # The whole point of the default `ordered: true`: a boot log whose lines
        # arrive out of order must not pass.
        matcher = ConsoleMatcher(parse_record(["first", "second"]))
        matcher.feed("here is second and then first\n")

        state = matcher.state()
        self.assertFalse(state.passed, "out-of-order output must not pass")
        self.assertIn("second", state.detail)

    def test_in_order_output_passes(self):
        matcher = ConsoleMatcher(parse_record(["first", "second"]))
        matcher.feed("here is first and then second\n")
        self.assertTrue(matcher.state().passed)

    def test_a_later_entry_cannot_match_text_consumed_by_an_earlier_one(self):
        # Both "abc" and "a" are present, but "a" only occurs *before* the text
        # the first entry consumed, so the ordered second entry has nothing left.
        matcher = ConsoleMatcher(parse_record(["abc", "a"]))
        matcher.feed("abc\n")
        self.assertFalse(matcher.state().passed)

    def test_an_unmatched_entry_does_not_release_the_ones_after_it(self):
        matcher = ConsoleMatcher(parse_record(["one", "two", "three"]))
        matcher.feed("one three")  # "two" never appears
        state = matcher.state()
        self.assertFalse(state.passed)
        self.assertIn("two", state.detail)
        self.assertIn("three", state.detail)

    def test_ordered_false_matches_anywhere_and_does_not_block(self):
        matcher = ConsoleMatcher(
            parse_record(
                [
                    {"regex": "before", "ordered": True},
                    {"regex": "anywhere", "ordered": False},
                    {"regex": "after", "ordered": True},
                ]
            )
        )
        matcher.feed("anywhere\n")
        matcher.feed("before\n")
        matcher.feed("after\n")
        self.assertTrue(matcher.state().passed)

    def test_ordered_false_still_matches_when_its_text_precedes_the_others(self):
        matcher = ConsoleMatcher(
            parse_record(
                [
                    {"regex": "anywhere", "ordered": False},
                    {"regex": "later", "ordered": True},
                ]
            )
        )
        matcher.feed("anywhere\nlater\n")
        self.assertTrue(matcher.state().passed)


class TestRepeat(SdkTestCase):
    def test_repeat_matches_successive_occurrences(self):
        matcher = ConsoleMatcher(parse_record([{"regex": "tick", "repeat": 3}]))
        matcher.feed("tick tick tick\n")
        self.assertTrue(matcher.state().passed)

    def test_partial_repeat_reports_have_over_need(self):
        matcher = ConsoleMatcher(parse_record([{"regex": "tick", "repeat": 3}]))
        matcher.feed("tick\ntick\n")

        state = matcher.state()
        self.assertFalse(state.passed)
        self.assertIn("(2/3)", state.detail)

    def test_repeat_requires_successive_not_merely_repeated_text(self):
        matcher = ConsoleMatcher(parse_record([{"regex": "tick", "repeat": 3}]))
        matcher.feed("tick\n")  # only one occurrence
        self.assertEqual(matcher.state().matched, 1)

    def test_an_unordered_repeat_counts_occurrences_anywhere(self):
        matcher = ConsoleMatcher(
            parse_record([{"regex": "x", "repeat": 2, "ordered": False}])
        )
        matcher.feed("x and x\n")
        self.assertTrue(matcher.state().passed)


class TestMultiLine(SdkTestCase):
    def test_multi_line_joins_lines_with_a_space(self):
        # A literal space in the regex, which is what multi_line exists to make
        # unnecessary to spell as \s+ -- see the one_line counterpart below.
        matcher = ConsoleMatcher(
            parse_record([{"regex": "Hello world!", "type": "multi_line"}])
        )
        matcher.feed("Hello\nworld!\n")
        self.assertTrue(matcher.state().passed)

    def test_multi_line_uses_the_configured_separator(self):
        matcher = ConsoleMatcher(
            parse_record(
                [{"regex": "a-b", "type": "multi_line", "separator": "-"}]
            )
        )
        matcher.feed("a\nb\n")
        self.assertTrue(matcher.state().passed)

    def test_one_line_does_not_span_a_line_break(self):
        # Same regex as above but without multi_line: the literal space cannot
        # match the newline, so this must not pass.
        matcher = ConsoleMatcher(parse_record([{"regex": "Hello world!"}]))
        matcher.feed("Hello\nworld!\n")
        self.assertFalse(matcher.state().passed)

    def test_carriage_returns_are_collapsed_too(self):
        matcher = ConsoleMatcher(
            parse_record([{"regex": "a b", "type": "multi_line"}])
        )
        matcher.feed("a\r\nb\r\n")
        self.assertTrue(matcher.state().passed)

    def test_ordering_survives_a_mix_of_one_line_and_multi_line(self):
        # multi_line rewrites the buffer (newlines collapse), so the shared
        # ordering cursor has to be translated back to raw-buffer positions.
        matcher = ConsoleMatcher(
            parse_record(
                [
                    {"regex": "boot ok", "type": "multi_line"},
                    {"regex": "Hello world!"},
                ]
            )
        )
        matcher.feed("boot\nok\n")
        matcher.feed("Hello world!\n")
        self.assertTrue(matcher.state().passed)

    def test_a_multi_line_entry_still_blocks_on_missing_text(self):
        matcher = ConsoleMatcher(
            parse_record(
                [
                    {"regex": "never appears", "type": "multi_line"},
                    {"regex": "Hello world!"},
                ]
            )
        )
        matcher.feed("Hello world!\n")
        self.assertFalse(matcher.state().passed)


class TestDiagnostics(SdkTestCase):
    def test_counts_report_partial_progress(self):
        matcher = ConsoleMatcher(parse_record(["one", "two", "three"]))
        matcher.feed("one two")
        state = matcher.state()
        self.assertEqual((state.matched, state.expected), (2, 3))

    def test_consumed_advances_only_for_ordered_matches(self):
        matcher = ConsoleMatcher(
            parse_record(
                [
                    {"regex": "anywhere", "ordered": False},
                    {"regex": "later"},
                ]
            )
        )
        matcher.feed("anywhere\n")
        self.assertEqual(matcher.consumed, 0)
        matcher.feed("later\n")
        # A buffer position just past the match, not a length: the trailing
        # newline is not consumed.
        self.assertEqual(matcher.consumed, len("anywhere\nlater"))

    def test_buffer_is_exposed_for_failure_messages(self):
        matcher = ConsoleMatcher(parse_record(["nope"]))
        matcher.feed("some output")
        self.assertEqual(matcher.buffer, "some output")


class TestParseRecord(SdkTestCase):
    def test_a_bare_string_is_a_single_one_line_entry(self):
        patterns = parse_record("Hello")
        self.assertEqual(len(patterns), 1)
        self.assertEqual(patterns[0].regex, "Hello")
        self.assertTrue(patterns[0].ordered)
        self.assertEqual(patterns[0].repeat, 1)
        self.assertEqual(patterns[0].type, "one_line")

    def test_a_list_of_strings_keeps_its_order(self):
        patterns = parse_record(["a", "b", "c"])
        self.assertEqual([pattern.regex for pattern in patterns], ["a", "b", "c"])

    def test_missing_record_is_rejected(self):
        with self.assertRaises(HarnessError) as caught:
            parse_record(None)
        self.assertIn("needs harness_config.record", str(caught.exception))

    def test_an_empty_list_is_rejected(self):
        with self.assertRaises(HarnessError):
            parse_record([])

    def test_an_entry_without_a_regex_is_rejected(self):
        with self.assertRaises(HarnessError) as caught:
            parse_record([{"ordered": False}])
        self.assertIn('missing "regex"', str(caught.exception))

    def test_an_invalid_regex_is_rejected_with_its_position(self):
        with self.assertRaises(HarnessError) as caught:
            parse_record(["ok", "([unclosed"])
        self.assertIn("record[1]", str(caught.exception))
        self.assertIn("invalid regex", str(caught.exception))

    def test_repeat_must_be_an_integer_at_least_one(self):
        with self.assertRaises(HarnessError):
            parse_record([{"regex": "x", "repeat": 0}])
        with self.assertRaises(HarnessError):
            parse_record([{"regex": "x", "repeat": "many"}])

    def test_type_must_be_one_line_or_multi_line(self):
        with self.assertRaises(HarnessError) as caught:
            parse_record([{"regex": "x", "type": "sometimes"}])
        self.assertIn("one_line", str(caught.exception))

    def test_an_entry_that_is_neither_string_nor_mapping_is_rejected(self):
        with self.assertRaises(HarnessError):
            parse_record([42])

    def test_matcher_refuses_an_empty_pattern_list(self):
        with self.assertRaises(HarnessError):
            ConsoleMatcher([])


if __name__ == "__main__":
    unittest.main()
