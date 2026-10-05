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
test_lz4.c:1:lz4.roundtrip:PASS
test_lz4.c:2:lz4.boundaries:PASS

-----------------------
2 Tests 0 Failures 0 Ignored
OK
"""

FAILING = """\
test_lz4.c:1:lz4.roundtrip:PASS
test_lz4.c:129:lz4.boundaries:FAIL:Expected 1 Was 2

-----------------------
2 Tests 1 Failures 0 Ignored
FAIL
"""


class TestPassingRun(SdkTestCase):
    def test_a_clean_run_passes_and_reports_its_cases(self):
        result = unity.parse_unity_output(PASSING)
        self.assertTrue(result.passed, result.detail)
        self.assertEqual((result.tests, result.failures, result.ignored), (2, 0, 0))
        self.assertEqual([case.name for case in result.cases],
                         ["lz4.roundtrip", "lz4.boundaries"])
        # The file is the source file Unity printed, not the suite standing in
        # for one; the suite travels in the case's name.
        self.assertEqual(result.cases[0].file, "test_lz4.c")
        # On a PASS line that number is the case's ordinal, not a location --
        # the report drops it rather than presenting it as one (report.CaseResult).
        self.assertEqual(result.cases[0].line, 1)
        self.assertTrue(result.summary_seen)
        self.assertEqual(result.verdict_seen, "OK")

    def test_ignored_tests_are_reported_but_do_not_fail_the_run(self):
        text = "test/a.c:1:a.t:PASS\n1 Tests 0 Failures 2 Ignored\nOK\n"
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
        result = unity.parse_unity_output("test/a.c:1:a.t:PASS\n")
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
        # Qualified, because the bare name does not identify a case: cjson has a
        # `boundaries` too.
        self.assertIn("lz4.boundaries", result.detail)

    def test_the_case_knows_its_suite_from_its_own_name(self):
        # The suite used to ride in the file field; it travels in the name now,
        # so what a suite claims out of a transcript is still unambiguous.
        result = unity.parse_unity_output(FAILING)
        self.assertEqual([case.suite for case in result.cases], ["lz4", "lz4"])
        self.assertEqual([case.label for case in result.cases],
                         ["lz4.roundtrip", "lz4.boundaries"])

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
        "test_lz4.c:1:lz4.roundtrip:PASS\r\n"
        "test_lz4.c:2:lz4.boundaries:PASS\r\n"
        "\r\n2 Tests 0 Failures 0 Ignored\r\nOK\r\n"
    )

    def test_crlf_terminated_case_lines_are_parsed(self):
        result = unity.parse_unity_output(self.CRLF)
        self.assertEqual(
            [case.name for case in result.cases],
            ["lz4.roundtrip", "lz4.boundaries"],
        )
        self.assertEqual([case.status for case in result.cases], ["PASS", "PASS"])
        self.assertTrue(result.passed, result.detail)

    def test_a_crlf_fail_message_does_not_carry_the_carriage_return(self):
        text = "test/a.c:7:a.t:FAIL:Expected 1 Was 2\r\n\r\n1 Tests 1 Failures 0 Ignored\r\nFAIL\r\n"
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
            "test/a.c:1:a.first:IGNORE\n"
            "test/a.c:2:a.second:IGNORED\n"
            "2 Tests 0 Failures 2 Ignored\nOK\n"
        )
        result = unity.parse_unity_output(text)
        self.assertEqual([case.ignored for case in result.cases], [True, True])

    def test_a_case_line_without_a_message_has_an_empty_message(self):
        result = unity.parse_unity_output("test/a.c:7:a.t:PASS\n1 Tests 0 Failures 0 Ignored\nOK\n")
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
        self.assertFalse(unity.is_complete("test/a.c:1:a.t:PASS\n"))
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


#: What an auto-running firmware prints, in the shape the board produced it:
#: every suite in one session, so a single summary covers all of them, and each
#: case line carries its suite in its own name -- the runner calls
#: ``UnityDefaultTestRun(tc->func, "suite.case", ...)``, which is what lets a
#: test case claim its own slice of one transcript without any extra
#: bookkeeping, and what leaves the file field free to be the real source file.
#: The number on a PASS line is the case's ordinal and the number on the FAIL
#: line is the assertion's ``__LINE__``; only the second is a location.
AUTORUN_TRANSCRIPT = """\
SiFli SDK unit tests (Unity)
  unity_run [suite|case|suite.case]
test_circular_buf.c:1:circular_buf.fill_and_drain:PASS
test_cjson.c:2:cjson.parse_and_read:PASS
test_cjson.c:129:cjson.malformed:FAIL:Expected 1 Was 2
test_lz4.c:4:lz4.roundtrip:PASS

-----------------------
4 Tests 1 Failures 0 Ignored
FAIL
unity: 4 case(s) selected, 1 failure(s)
msh />
"""


class TestAutorun(SdkTestCase):
    """``autorun``: the firmware reports by itself, so the host only reads.

    Nothing is sent, and nothing is discarded -- the boot log *is* the result,
    so throwing it away as the command path must would discard the measurement.
    """

    def test_nothing_is_written_and_the_boot_log_is_not_discarded(self):
        console = FakeConsole(boot=["banner\n", AUTORUN_TRANSCRIPT])
        outcome = unity.run(an_instance(autorun=True), console, timeout=5.0, poll=0.01)

        self.assertEqual(console.written, [])
        self.assertFalse(outcome.passed, "the transcript contains a failure")
        self.assertIn("malformed", outcome.detail)
        self.assertEqual(len(outcome.cases), 4)

    def test_the_command_path_is_unchanged_when_autorun_is_off(self):
        console = FakeConsole(
            boot=["msh />"], reply=["2 Tests 0 Failures 0 Ignored\nOK\n"]
        )
        outcome = unity.run(an_instance(), console, timeout=5.0, poll=0.01)

        self.assertTrue(outcome.passed, outcome.detail)
        self.assertEqual(console.written, ["unity_run\r\n"])


class TestAutorunPerSuite(SdkTestCase):
    """One transcript holds every suite, so a test case owns its own slice."""

    def test_a_passing_suite_passes_although_another_suite_failed(self):
        console = FakeConsole(boot=[AUTORUN_TRANSCRIPT])
        outcome = unity.run(
            an_instance(autorun=True, suite="lz4"), console, timeout=5.0, poll=0.01
        )

        self.assertTrue(outcome.passed, outcome.detail)
        self.assertEqual([case.name for case in outcome.cases], ["lz4.roundtrip"])

    def test_a_failing_suite_fails_and_names_only_its_own_cases(self):
        console = FakeConsole(boot=[AUTORUN_TRANSCRIPT])
        outcome = unity.run(
            an_instance(autorun=True, suite="cjson"), console, timeout=5.0, poll=0.01
        )

        self.assertFalse(outcome.passed)
        self.assertIn("cjson.malformed", outcome.detail)
        self.assertEqual(
            [case.name for case in outcome.cases],
            ["cjson.parse_and_read", "cjson.malformed"],
        )

    def test_a_suite_that_never_ran_is_a_failure_not_a_pass(self):
        # The dangerous one. The firmware printed OK, so handing that to a suite
        # which contributed nothing would be a green light proving nothing.
        console = FakeConsole(boot=[AUTORUN_TRANSCRIPT])
        outcome = unity.run(
            an_instance(autorun=True, suite="hal_mock"), console, timeout=5.0, poll=0.01
        )

        self.assertFalse(outcome.passed)
        self.assertIn("did not reach it", outcome.detail)
        # And it names what the transcript did hold, so the claim is checkable.
        self.assertIn("lz4", outcome.detail)
        self.assertEqual(outcome.cases, [])

    def test_a_transcript_that_never_finished_is_reported_as_such(self):
        console = FakeConsole(boot=["test_lz4.c:1:lz4.roundtrip:PASS\n"])
        outcome = unity.run(
            an_instance(autorun=True, suite="lz4"), console, timeout=0.05, poll=0.001
        )

        self.assertFalse(outcome.passed)
        self.assertIn("no Unity summary line found", outcome.detail)


class TestAutorunIsUnityOnly(SdkTestCase):
    """A key another harness never reads is refused, not ignored.

    ``unity`` already refuses ``record`` from the other side; this is the same
    rule in the same direction. The console and pytest harnesses are exercised
    here because the instance and console helpers live in this file.
    """

    def test_the_rule_refuses_the_key_and_names_its_owner(self):
        from sdk_test.harness import reject_foreign_keys

        with self.assertRaises(HarnessError) as caught:
            reject_foreign_keys("console", {"autorun": True})
        self.assertIn("autorun", str(caught.exception))
        self.assertIn('"unity"', str(caught.exception))

    def test_an_absent_key_is_fine(self):
        from sdk_test.harness import reject_foreign_keys

        reject_foreign_keys("console", {})
        reject_foreign_keys("console", {"record": ["hello"]})

    def test_the_console_harness_refuses_it_before_reading_anything(self):
        from sdk_test.harness import console as console_harness

        console = FakeConsole(boot=["never read"])
        with self.assertRaises(HarnessError) as caught:
            console_harness.run(an_instance(autorun=True), console, timeout=0.01)

        self.assertIn("autorun", str(caught.exception))
        self.assertEqual(console.reads, 0)

    def test_the_pytest_harness_refuses_it_too(self):
        from sdk_test.harness import pytest as pytest_harness

        console = FakeConsole()
        with self.assertRaises(HarnessError) as caught:
            pytest_harness.run(an_instance(autorun=True), console, timeout=0.01)

        self.assertIn("autorun", str(caught.exception))

    def test_a_unity_case_may_not_describe_a_command_it_will_not_send(self):
        # `cmd` and `prompt` are for the path where the firmware waits for one.
        # Under autorun they are read by nothing, so writing them says something
        # the run will not do.
        with self.assertRaises(HarnessError) as caught:
            unity.require_no_send_keys({"autorun": True, "cmd": "unity_run"})
        self.assertIn("cmd", str(caught.exception))

        with self.assertRaises(HarnessError) as caught:
            unity.require_no_send_keys({"autorun": True, "prompt": "msh />"})
        self.assertIn("prompt", str(caught.exception))

    def test_those_keys_are_fine_when_the_firmware_waits_for_a_command(self):
        unity.require_no_send_keys({"cmd": "unity_run", "prompt": "msh />"})
        unity.require_no_send_keys({"autorun": True, "suite": "lz4"})


if __name__ == "__main__":
    unittest.main()
