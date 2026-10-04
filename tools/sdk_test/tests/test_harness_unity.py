# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for ``sdk_test.harness.unity``.

The parser reads Unity's *own* output contract off the console, so the tests
below feed literal Unity transcripts. The vacuous-pass guards matter most: a
binary that links Unity but registers no test must never be reported as green.
"""

from __future__ import annotations

import unittest

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from sdk_test.errors import HarnessError
from sdk_test.harness import unity

PASSING = """\
Unity test run 1 of 1
-----------------------
test/test_lz4.c:42:test_roundtrip:PASS
test/test_lz4.c:58:test_bounds:PASS

-----------------------
2 Tests 0 Failures 0 Ignored
OK
"""

FAILING = """\
test/test_lz4.c:42:test_roundtrip:PASS
test/test_lz4.c:58:test_bounds:FAIL:Expected 1 Was 2

-----------------------
2 Tests 1 Failures 0 Ignored
FAIL
"""


class TestPassingRun(SdkTestCase):
    def test_a_clean_run_passes_and_reports_its_cases(self):
        result = unity.parse_unity_output(PASSING)
        self.assertTrue(result.passed, result.detail)
        self.assertEqual((result.tests, result.failures, result.ignored), (2, 0, 0))
        self.assertEqual([case.name for case in result.cases], ["test_roundtrip", "test_bounds"])
        self.assertEqual(result.cases[0].file, "test/test_lz4.c")
        self.assertEqual(result.cases[0].line, 42)
        self.assertTrue(result.summary_seen)
        self.assertEqual(result.verdict_seen, "OK")

    def test_ignored_tests_are_reported_but_do_not_fail_the_run(self):
        text = "test/a.c:1:t:PASS\n1 Tests 0 Failures 2 Ignored\nOK\n"
        result = unity.parse_unity_output(text)
        self.assertTrue(result.passed)
        self.assertEqual(result.ignored, 2)
        self.assertIn("2 ignored", result.detail)


class TestVacuousPassGuards(SdkTestCase):
    def test_zero_tests_is_a_failure_even_though_unity_says_ok(self):
        # The single most dangerous false green: linked Unity, registered
        # nothing, printed OK.
        text = "0 Tests 0 Failures 0 Ignored\nOK\n"
        result = unity.parse_unity_output(text)
        self.assertFalse(result.passed)
        self.assertIn("vacuous", result.detail)

    def test_a_missing_summary_is_a_failure(self):
        result = unity.parse_unity_output("test/a.c:1:t:PASS\n")
        self.assertFalse(result.passed)
        self.assertIn("no Unity summary line found", result.detail)

    def test_an_empty_transcript_is_a_failure(self):
        result = unity.parse_unity_output("")
        self.assertFalse(result.passed)

    def test_console_noise_alone_is_a_failure(self):
        result = unity.parse_unity_output(
            " | / - RT - Thread Operating System\nmsh />\n"
        )
        self.assertFalse(result.passed)


class TestFailures(SdkTestCase):
    def test_failures_fail_the_run_and_name_the_cases(self):
        result = unity.parse_unity_output(FAILING)
        self.assertFalse(result.passed)
        self.assertIn("1 of 2", result.detail)
        self.assertIn("test_bounds", result.detail)

    def test_a_fail_case_message_is_captured(self):
        result = unity.parse_unity_output(FAILING)
        failing = [case for case in result.cases if not case.passed and not case.ignored]
        self.assertEqual(len(failing), 1)
        self.assertEqual(failing[0].message, "Expected 1 Was 2")
        self.assertEqual(result.failed_cases, failing)

    def test_a_fail_verdict_with_zero_failures_is_trusted_over_the_count(self):
        # Contradictory output: prefer the harsher signal rather than the number.
        text = "1 Tests 0 Failures 0 Ignored\nFAIL\n"
        result = unity.parse_unity_output(text)
        self.assertFalse(result.passed)
        self.assertIn("printed FAIL", result.detail)

    def test_the_last_summary_wins_when_a_session_runs_unity_twice(self):
        text = (
            "1 Tests 1 Failures 0 Ignored\nFAIL\n"
            "1 Tests 0 Failures 0 Ignored\nOK\n"
        )
        result = unity.parse_unity_output(text)
        self.assertTrue(result.passed, "the second, clean run should decide")
        self.assertEqual(result.tests, 1)


class TestLineEndings(SdkTestCase):
    """The console delivers CRLF, because ``rt_kprintf`` expands ``\\n``.

    This is not a corner case, it is the normal form on this hardware. Getting
    it wrong is silent: passing lines vanish and only failures survive, so a
    green run looks like a run that reported nothing.
    """

    CRLF = (
        "test/test_lz4.c:42:test_roundtrip:PASS\r\n"
        "test/test_lz4.c:58:test_bounds:PASS\r\n"
        "\r\n2 Tests 0 Failures 0 Ignored\r\nOK\r\n"
    )

    def test_crlf_terminated_case_lines_are_parsed(self):
        result = unity.parse_unity_output(self.CRLF)
        self.assertEqual(
            [case.name for case in result.cases], ["test_roundtrip", "test_bounds"]
        )
        self.assertEqual([case.status for case in result.cases], ["PASS", "PASS"])
        self.assertTrue(result.passed, result.detail)

    def test_a_crlf_fail_message_does_not_carry_the_carriage_return(self):
        text = "test/a.c:7:t:FAIL:Expected 1 Was 2\r\n\r\n1 Tests 1 Failures 0 Ignored\r\nFAIL\r\n"
        result = unity.parse_unity_output(text)
        self.assertEqual(result.cases[0].message, "Expected 1 Was 2")

    def test_the_summary_and_verdict_tolerate_crlf_too(self):
        result = unity.parse_unity_output(self.CRLF)
        self.assertTrue(result.summary_seen)
        self.assertEqual(result.verdict_seen, "OK")
        self.assertEqual(result.tests, 2)

    def test_bare_lf_still_works(self):
        # Both forms must parse: the host cannot dictate what a carrier emits.
        result = unity.parse_unity_output(self.CRLF.replace("\r\n", "\n"))
        self.assertTrue(result.passed, result.detail)

    def test_a_passing_run_is_not_reported_as_missing_its_summary(self):
        # The exact symptom of the CRLF bug: real content parsed as nothing.
        result = unity.parse_unity_output(self.CRLF)
        self.assertNotIn("no Unity summary line found", result.detail)


class TestCaseParsing(SdkTestCase):
    def test_ignored_cases_are_recognised_in_both_spellings(self):
        text = (
            "test/a.c:1:first:IGNORE\n"
            "test/a.c:2:second:IGNORED\n"
            "2 Tests 0 Failures 2 Ignored\nOK\n"
        )
        result = unity.parse_unity_output(text)
        self.assertEqual([case.ignored for case in result.cases], [True, True])

    def test_a_case_line_without_a_message_has_an_empty_message(self):
        result = unity.parse_unity_output("test/a.c:7:t:PASS\n1 Tests 0 Failures 0 Ignored\nOK\n")
        self.assertEqual(result.cases[0].message, "")

    def test_unity_padding_around_the_numbers_is_tolerated(self):
        text = "  2 Tests   1 Failures   0 Ignored  \nFAIL\n"
        result = unity.parse_unity_output(text)
        self.assertEqual((result.tests, result.failures), (2, 1))


class TestStartCommand(SdkTestCase):
    def test_the_default_command_matches_the_device_shell(self):
        self.assertEqual(unity.start_command({}), "unity_run")
        self.assertEqual(unity.DEFAULT_CMD, "unity_run")

    def test_cmd_overrides_the_default(self):
        self.assertEqual(unity.start_command({"cmd": "my_runner"}), "my_runner")

    def test_suite_is_appended_as_the_selection_argument(self):
        self.assertEqual(unity.start_command({"suite": "lz4"}), "unity_run lz4")
        self.assertEqual(
            unity.start_command({"cmd": "unity_run", "suite": "lz4.test_a"}),
            "unity_run lz4.test_a",
        )


class TestCompletion(SdkTestCase):
    def test_is_complete_only_once_the_summary_has_arrived(self):
        self.assertFalse(unity.is_complete("test/a.c:1:t:PASS\n"))
        self.assertTrue(unity.is_complete(PASSING))

    def test_is_complete_survives_a_split_summary_line(self):
        # The host may stop reading as soon as the summary is complete, so this
        # must not fire on a partial line.
        self.assertFalse(unity.is_complete("2 Tests 0 Failur"))


class FakeConsole:
    """A console that hands over scripted chunks, one per read.

    ``boot`` is what the device says unprompted; ``reply`` is only appended
    once something is written. Keeping them apart is what lets a test tell
    "discarded the boot log" from "discarded the answer it had not sent for
    yet" -- a fake where the summary is already queued makes the harness's own
    ``_discard_pending`` look like the bug.
    """

    def __init__(self, boot=(), reply=()) -> None:
        self._reply = list(reply)
        self._chunks = list(boot)
        self.written = []
        self.reads = 0
        self.reads_at_write = None

    def read(self, timeout: float = 0.0) -> str:
        self.reads += 1
        if not self._chunks:
            return ""
        return self._chunks.pop(0)

    def write(self, text: str) -> None:
        self.reads_at_write = self.reads
        self.written.append(text)
        self._chunks += self._reply

    def skip_to_end(self) -> None:
        self._chunks = []


def an_instance(**config):
    from sdk_test.model import TestInstance

    return TestInstance(
        id="unit_test.lz4",
        testsuite_root=".",
        testcase_dir=".",
        tests_key="unit_test.lz4",
        harness_config=config,
    )


class TestWaitsForTheShell(SdkTestCase):
    """The harness must not address a shell that is not up yet.

    On a board nothing can reset, the console opens while the board is still
    starting: the download that booted it has only just finished writing, and
    the ROM has not begun. A command sent then is lost, and the run reports
    "no Unity summary line found" -- which reads as a firmware fault.

    Waiting for *silence* would not do: a board that has not started booting is
    silent too, and worse, so is one whose download has not triggered the boot
    yet. The prompt is the positive signal.
    """

    def test_it_reads_past_the_boot_log_to_the_prompt(self):
        console = FakeConsole(
            boot=["SFBL\nNAND ID 0xc891c8\n", "sif_bbm_init done\n", "msh />"]
        )
        self.assertTrue(unity._wait_for_prompt(console, r"msh />", 5.0, 0.01))
        self.assertEqual(console.reads, 3)

    def test_a_board_that_never_shows_one_gives_up(self):
        # Bounded, so a device that chatters is not a hang.
        console = FakeConsole(boot=["still booting\n"] * 500)
        self.assertFalse(unity._wait_for_prompt(console, r"msh />", 0.05, 0.001))

    def test_an_uncompilable_pattern_does_not_hang(self):
        # `harness_config.prompt` is user input; a typo in it must not wedge
        # the run before the harness's own timeout can say anything.
        console = FakeConsole(boot=["msh />"])
        self.assertFalse(unity._wait_for_prompt(console, "(", 5.0, 0.01))
        self.assertEqual(console.reads, 0)

    def test_the_command_goes_out_only_after_the_prompt(self):
        # The ordering is the whole fix. Writing first and waiting after would
        # lose the command exactly as before.
        console = FakeConsole(
            boot=["booting\n", "  unity_run ...\n", "msh />"],
            reply=["2 Tests 0 Failures 0 Ignored\nOK\n"],
        )
        outcome = unity.run(an_instance(), console, timeout=5.0, poll=0.01)

        self.assertTrue(outcome.passed, outcome.detail)
        self.assertEqual(console.written, ["unity_run\r\n"])
        # Three reads to reach the prompt, and the write happened after them.
        self.assertEqual(console.reads_at_write, 3)


class TestRecordIsRejected(SdkTestCase):
    def test_a_unity_test_may_not_also_declare_console_records(self):
        with self.assertRaises(HarnessError) as caught:
            unity.require_record_free({"record": ["nope"]})
        self.assertIn('use harness "console"', str(caught.exception))

    def test_an_absent_record_is_fine(self):
        unity.require_record_free({})
        unity.require_record_free({"suite": "lz4"})


if __name__ == "__main__":
    unittest.main()
