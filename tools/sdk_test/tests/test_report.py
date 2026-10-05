# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for ``sdk_test.report`` -- aggregation, the run log, JSON, JUnit.

The XML is what CI consumes, so it is asserted structurally (parsed back and
inspected) rather than by string comparison. The JSON is the document the XML is
projected from, so the projection is asserted to be a pure function of it.
"""

from __future__ import annotations

import json
import os
import time
import unittest
import xml.etree.ElementTree as ElementTree

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from sdk_test import report


def make_summary():
    summary = report.RunSummary(started_at=0.0)
    summary.add(report.TestResult("a/one", report.PASSED, duration=1.5, log="boot ok"))
    summary.add(
        report.TestResult(
            "a/two",
            report.FAILED,
            duration=2.0,
            message="missing console expectation(s): /Hello/",
            log="Hello worldXYZ!",
            testbed_id="tb_01",
            platform="board_a",
        )
    )
    summary.add(report.TestResult("b/three", report.ERROR, message="agent unreachable"))
    summary.add(report.TestResult("b/four", report.SKIPPED, message="build_only"))
    summary.finished_at = 10.0
    return summary


class TestRunDir(SdkTestCase):
    """One directory per run, so yesterday's evidence survives today's run.

    Every run used to write into the same tree and overwrite it. The log of a
    run that passed is the baseline that makes the next flake legible, so losing
    it silently was the one thing this layout had to avoid.
    """

    def test_the_stamp_is_sortable_and_filesystem_safe(self):
        # Built from a local struct_time so the assertion does not depend on
        # the machine's timezone, which is the point of using local time.
        when = time.mktime(time.strptime("2026-10-03 17:09:22", "%Y-%m-%d %H:%M:%S"))
        stamp = report.run_stamp(when)
        self.assertEqual(stamp, "2026-10-03T17-09-22")
        # Windows forbids colons in a file name, and the format has to sort
        # lexicographically in time order or `ls` would not show the newest last.
        self.assertNotIn(":", stamp)
        earlier = report.run_stamp(when - 3661)
        self.assertLess(earlier, stamp)

    def test_two_runs_in_the_same_second_do_not_share_a_tree(self):
        taken = set()

        def is_taken(path: str) -> bool:
            return path in taken

        first = report.run_dir("/out", "2026-10-03T17-09-22", taken=is_taken)
        taken.add(first)
        second = report.run_dir("/out", "2026-10-03T17-09-22", taken=is_taken)
        taken.add(second)
        third = report.run_dir("/out", "2026-10-03T17-09-22", taken=is_taken)

        self.assertEqual(os.path.basename(first), "2026-10-03T17-09-22")
        self.assertEqual(os.path.basename(second), "2026-10-03T17-09-22-2")
        self.assertEqual(os.path.basename(third), "2026-10-03T17-09-22-3")

    def test_the_run_directory_hangs_off_the_outdir(self):
        path = report.run_dir("/out", "2026-10-03T17-09-22", taken=lambda _p: False)
        self.assertEqual(os.path.dirname(path), os.path.abspath("/out"))


class TestSeparator(SdkTestCase):
    def test_the_title_sits_in_the_middle_at_the_requested_width(self):
        rule = report.separator("[1/3] hello.console", width=40)
        self.assertEqual(len(rule), 40)
        self.assertIn(" [1/3] hello.console ", rule)

    def test_it_is_ascii_only(self):
        # The one piece of output whose whole job is to be readable: a console
        # in a non-UTF-8 code page turns box drawing into noise, which is worse
        # than no separator.
        rule = report.separator("[1/1] x", width=40)
        rule.encode("ascii")

    def test_a_title_too_long_for_the_width_is_not_truncated(self):
        rule = report.separator("[1/1] " + "x" * 100, width=40)
        self.assertIn("x" * 100, rule)


class TestManifest(SdkTestCase):
    def test_every_entry_is_numbered_and_carries_its_board_and_harness(self):
        text = report.format_manifest([
            (1, 2, "a/hello.console", "sf32lb52-lcd_n16r8", "console"),
            (2, 2, "b/hello.pytest", "eh-lb563", "pytest"),
        ])
        lines = text.splitlines()
        self.assertIn("2 test(s) to run", lines[0])
        self.assertIn("1/2", lines[1])
        self.assertIn("a/hello.console", lines[1])
        self.assertIn("(sf32lb52-lcd_n16r8, console)", lines[1])
        self.assertIn("2/2", lines[2])


class TestRunSummary(SdkTestCase):
    def test_counts_are_grouped_by_status(self):
        summary = make_summary()
        self.assertEqual(summary.total, 4)
        self.assertEqual(summary.passed, 1)
        self.assertEqual(summary.failed, 2)  # failed + error
        self.assertEqual(summary.skipped, 1)

    def test_errors_count_as_failures_but_stay_distinguishable(self):
        summary = make_summary()
        self.assertEqual(summary.count(report.FAILED), 1)
        self.assertEqual(summary.count(report.ERROR), 1)

    def test_a_run_is_ok_only_when_something_ran_and_nothing_failed(self):
        self.assertFalse(make_summary().ok)

        clean = report.RunSummary(started_at=0.0)
        clean.add(report.TestResult("a/one", report.PASSED))
        self.assertTrue(clean.ok)

    def test_an_empty_run_is_not_ok(self):
        # Matching the Unity harness's stance: nothing executed is not success.
        self.assertFalse(report.RunSummary().ok)

    def test_skips_do_not_fail_a_run(self):
        summary = report.RunSummary(started_at=0.0)
        summary.add(report.TestResult("a/one", report.PASSED))
        summary.add(report.TestResult("a/two", report.SKIPPED))
        self.assertTrue(summary.ok)

    def test_duration_uses_finished_at_when_present(self):
        self.assertAlmostEqual(make_summary().duration, 10.0)

    def test_duration_is_never_negative(self):
        summary = report.RunSummary(started_at=100.0)
        summary.finished_at = 50.0
        self.assertEqual(summary.duration, 0.0)

    def test_failures_by_status_omits_empty_statuses(self):
        self.assertEqual(
            make_summary().failures_by_status(),
            {report.ERROR: 1, report.FAILED: 1, report.SKIPPED: 1, report.PASSED: 1},
        )


class TestJunitXml(SdkTestCase):
    def write_and_parse(self, summary):
        path = self.write("junit.xml", "\n")  # create the directory
        report.write_junit_xml(summary, path)
        return ElementTree.parse(path).getroot()

    def test_the_suite_counts_match_the_summary(self):
        root = self.write_and_parse(make_summary())
        self.assertEqual(root.tag, "testsuite")
        self.assertEqual(root.get("tests"), "4")
        self.assertEqual(root.get("failures"), "1")
        self.assertEqual(root.get("errors"), "1")
        self.assertEqual(root.get("skipped"), "1")

    def test_every_result_becomes_a_testcase(self):
        root = self.write_and_parse(make_summary())
        cases = root.findall("testcase")
        self.assertEqual(len(cases), 4)
        self.assertEqual([case.get("name") for case in cases], ["one", "two", "three", "four"])

    def test_the_classname_is_the_directory_part_of_the_id(self):
        root = self.write_and_parse(make_summary())
        cases = {case.get("name"): case for case in root.findall("testcase")}
        self.assertEqual(cases["one"].get("classname"), "a")
        self.assertEqual(cases["four"].get("classname"), "b")

    def test_a_failure_carries_its_message_and_transcript(self):
        root = self.write_and_parse(make_summary())
        cases = {case.get("name"): case for case in root.findall("testcase")}
        failure = cases["two"].find("failure")
        self.assertIsNotNone(failure)
        self.assertIn("missing console expectation", failure.get("message"))
        self.assertEqual(failure.text, "Hello worldXYZ!")

    def test_an_error_is_distinct_from_a_failure(self):
        root = self.write_and_parse(make_summary())
        cases = {case.get("name"): case for case in root.findall("testcase")}
        self.assertIsNone(cases["three"].find("failure"))
        error = cases["three"].find("error")
        self.assertEqual(error.get("message"), "agent unreachable")

    def test_a_skip_uses_the_skipped_element(self):
        root = self.write_and_parse(make_summary())
        cases = {case.get("name"): case for case in root.findall("testcase")}
        self.assertEqual(cases["four"].find("skipped").get("message"), "build_only")

    def test_a_pass_carries_no_transcript(self):
        # The log is inlined for failures only, following twister. That keeps a
        # red report self-contained (see the failure test above) without copying
        # a console -- up to the backend's 4 MB retention cap -- into the report
        # once per green instance. A pass's transcript is on disk instead:
        # device.log beside the instance's runner.log, with the run's own record
        # naming the directory.
        root = self.write_and_parse(make_summary())
        cases = {case.get("name"): case for case in root.findall("testcase")}
        self.assertIsNone(cases["one"].find("system-out"))

    def test_one_testcase_per_case_when_the_harness_reported_cases(self):
        summary = report.RunSummary(started_at=0.0)
        summary.add(
            report.TestResult(
                "unit_test.lz4",
                report.FAILED,
                message="1 of 3 Unity test(s) failed",
                platform="qemu_sf32lb52x",
                cases=[
                    report.CaseResult("test_roundtrip", report.PASSED, file="lz4"),
                    report.CaseResult(
                        "test_bounds",
                        report.FAILED,
                        message="expected -1, got 1024",
                        file="lz4",
                        line=58,
                    ),
                    report.CaseResult("test_slow", report.SKIPPED, file="lz4"),
                ],
            )
        )
        root = self.write_and_parse(summary)
        cases = root.findall("testcase")

        self.assertEqual([c.get("name") for c in cases],
                         ["lz4.test_roundtrip", "lz4.test_bounds", "lz4.test_slow"])
        # Every case belongs to its instance, so that is the classname.
        self.assertEqual({c.get("classname") for c in cases}, {"unit_test.lz4"})
        self.assertEqual({c.get("platform") for c in cases}, {"qemu_sf32lb52x"})

        failure = cases[1].find("failure")
        self.assertIn("expected -1, got 1024", failure.get("message"))
        self.assertIn("lz4:58", failure.text)
        # The instance-level reason rides along: it says how many siblings failed.
        self.assertIn("1 of 3 Unity test(s) failed", failure.text)

        # A passing case explains nothing and carries no child element -- in
        # particular not its sibling's failure reason, which is what it once got.
        self.assertEqual(len(cases[0]), 0)
        self.assertIsNotNone(cases[2].find("skipped"))

    def test_two_suites_sharing_a_case_name_stay_apart(self):
        # Both cjson and lz4 have a `boundaries`, and one instance emits one row
        # per case, so an unqualified name would merge a pass and a failure into
        # one row -- in the report and in the XML. The board produced exactly
        # this pair, which is how it was found.
        summary = report.RunSummary(started_at=0.0)
        summary.add(
            report.TestResult(
                "unit_test.board",
                report.FAILED,
                cases=[
                    report.CaseResult("boundaries", report.PASSED, file="cjson", line=6),
                    report.CaseResult(
                        "boundaries",
                        report.FAILED,
                        message="Expected 1024 to be less than 0",
                        file="lz4",
                        line=129,
                    ),
                ],
            )
        )
        root = self.write_and_parse(summary)
        cases = root.findall("testcase")

        self.assertEqual(
            [case.get("name") for case in cases],
            ["cjson.boundaries", "lz4.boundaries"],
        )
        self.assertEqual({c.get("classname") for c in cases}, {"unit_test.board"})
        # And the failure is attached to the lz4 one, so a consumer keying on
        # the name cannot attribute it to the suite that passed.
        self.assertIsNone(cases[0].find("failure"))
        self.assertIsNotNone(cases[1].find("failure"))

    def test_the_counts_describe_what_was_emitted_not_the_instances(self):
        # One instance, three cases: the header must say 3/1/0/1, not 1/1/0/0.
        summary = report.RunSummary(started_at=0.0)
        summary.add(
            report.TestResult(
                "unit_test.lz4",
                report.FAILED,
                cases=[
                    report.CaseResult("a", report.PASSED),
                    report.CaseResult("b", report.FAILED),
                    report.CaseResult("c", report.SKIPPED),
                ],
            )
        )
        root = self.write_and_parse(summary)
        self.assertEqual(root.get("tests"), "3")
        self.assertEqual(root.get("failures"), "1")
        self.assertEqual(root.get("skipped"), "1")
        self.assertEqual(root.get("errors"), "0")

    def test_the_json_is_the_document_the_xml_projects(self):
        summary = make_summary()
        document = report.summary_to_dict(summary)
        json_path = self.write("r.json", json.dumps(document))

        self.assertEqual(document["summary"]["total"], 4)
        self.assertEqual(document["summary"]["passed"], 1)
        self.assertEqual(document["testsuites"][1]["name"], "a/two")
        # Failures carry their log; passes do not.
        self.assertEqual(document["testsuites"][1]["log"], "Hello worldXYZ!")
        self.assertNotIn("log", document["testsuites"][0])

        # Projecting the same document yields the same bytes as the direct path.
        from_dict = report.dict_to_xml(json.loads(open(json_path, encoding="utf-8").read()))
        from_summary = report.dict_to_xml(document)
        self.assertEqual(
            ElementTree.tostring(from_dict), ElementTree.tostring(from_summary)
        )

    def test_the_run_log_names_the_root_the_outdir_and_the_failures(self):
        summary = make_summary()
        summary.root = "example"
        summary.out_dir = "/tmp/test-out"
        summary.discovered = 6
        summary.add(
            report.TestResult(
                "c/five", report.SKIPPED, message="filtered: needs fixture 'spi'",
                platform="board_a",
            )
        )

        text = report.run_log(summary)
        self.assertIn("root:       example", text)
        self.assertIn("outdir:     /tmp/test-out", text)
        # Discovery found more than were run: that gap is the point of the line.
        self.assertIn("discovered: 6", text)
        self.assertIn("ran:        5", text)
        self.assertIn("FAIL  a/two", text)
        self.assertIn("not run (filtered):", text)
        self.assertIn("needs fixture 'spi'", text)

    def test_a_failure_says_which_stage_it_failed_at(self):
        # ERROR is one bucket for four different problems -- planning refused,
        # the build failed, the target would not open, the console dropped. The
        # stage is what tells them apart without parsing the message.
        summary = report.RunSummary(started_at=0.0)
        summary.add(
            report.TestResult(
                "a/never-built",
                report.ERROR,
                message="Command failed with exit code 2",
                stage=report.STAGE_BUILD,
            )
        )
        summary.add(
            report.TestResult(
                "a/ran-and-lost",
                report.FAILED,
                message="missing expectation",
                stage=report.STAGE_RUN,
            )
        )
        summary.add(report.TestResult("a/ok", report.PASSED, stage=report.STAGE_RUN))

        text = report.format_summary(summary)
        self.assertIn("at build", text)
        # A pass is always "run", so the stage would be noise on a green line:
        # exactly one occurrence, from the failing instance.
        self.assertEqual(text.count("at run"), 1)

        document = report.summary_to_dict(summary)
        self.assertEqual(document["testsuites"][0]["stage"], "build")
        self.assertEqual(document["testsuites"][1]["stage"], "run")

    def test_the_hardware_identity_is_recorded_when_known(self):
        root = self.write_and_parse(make_summary())
        cases = {case.get("name"): case for case in root.findall("testcase")}
        self.assertEqual(cases["two"].get("testbed"), "tb_01")
        self.assertEqual(cases["two"].get("platform"), "board_a")
        self.assertIsNone(cases["one"].get("testbed"))

    def test_a_result_with_no_directory_falls_back_to_the_suite_name(self):
        summary = report.RunSummary(started_at=0.0)
        summary.add(report.TestResult("bare", report.PASSED))
        root = self.write_and_parse(summary)
        case = root.find("testcase")
        self.assertEqual(case.get("classname"), "sifli-sdk")
        self.assertEqual(case.get("name"), "bare")

    def test_status_counts_are_exposed_as_properties(self):
        root = self.write_and_parse(make_summary())
        properties = {
            prop.get("name"): prop.get("value") for prop in root.find("properties")
        }
        self.assertEqual(properties["count.passed"], "1")
        self.assertEqual(properties["count.failed"], "1")

    def test_the_output_directory_is_created(self):
        path = self.root + "/nested/deeper/junit.xml"
        report.write_junit_xml(make_summary(), path)
        self.assertTrue(ElementTree.parse(path).getroot().tag == "testsuite")

    def test_write_reports_the_path_it_wrote(self):
        path = self.root + "/junit.xml"
        self.assertEqual(report.write_junit_xml(make_summary(), path), path)


class TestFormatSummary(SdkTestCase):
    def test_every_result_gets_a_line_and_failures_get_their_message(self):
        text = report.format_summary(make_summary())
        self.assertIn("[PASS ] a/one", text)
        self.assertIn("[FAIL ] a/two on tb_01", text)
        self.assertIn("[ERROR] b/three", text)
        self.assertIn("[SKIP ] b/four", text)
        self.assertIn("missing console expectation", text)

    def test_results_are_grouped_by_their_testcase_file(self):
        summary = report.RunSummary()
        summary.add(report.TestResult("rtt/hello.console", report.PASSED))
        summary.add(report.TestResult("rtt/hello.unity", report.PASSED))
        summary.add(report.TestResult("no-os/hello.noos", report.FAILED, message="boom"))

        lines = report.format_summary(summary).splitlines()
        heads = [line for line in lines if line.startswith("=")]

        self.assertEqual(len(heads), 2)
        self.assertIn("rtt", heads[0])
        self.assertIn("2 test(s): 2 passed", heads[0])
        self.assertIn("no-os", heads[1])
        self.assertIn("1 test(s): 1 failed", heads[1])
        # Each test's line follows its own group's header, not another's.
        self.assertLess(lines.index(heads[0]),
                        lines.index(next(l for l in lines if "rtt/hello.unity" in l)))
        self.assertLess(lines.index(heads[1]),
                        lines.index(next(l for l in lines if "no-os/hello.noos" in l)))

    def test_a_testcase_at_the_suite_root_is_named_after_the_root(self):
        # `-T example/test/unit_test` gives ids with no directory part, so the
        # group would otherwise have no name at all.
        summary = report.RunSummary(root=os.path.join("example", "test", "unit_test"))
        summary.add(report.TestResult("unit_test.lz4", report.PASSED))

        heads = [l for l in report.format_summary(summary).splitlines()
                 if l.startswith("=")]
        self.assertEqual(len(heads), 1)
        self.assertIn("unit_test", heads[0])

    def test_the_totals_line_counts_every_status(self):
        text = report.format_summary(make_summary())
        self.assertIn("4 test(s): 1 passed, 1 failed, 1 errored, 1 skipped", text)

    def test_inline_logs_prints_a_failing_instances_log(self):
        summary = make_summary()
        pointing = report.format_summary(summary, inline_logs=False)
        inlined = report.format_summary(summary, inline_logs=True)

        # The failing instance's log is what a reader wants inline ...
        self.assertNotIn("| Hello worldXYZ!", pointing)
        self.assertIn("| Hello worldXYZ!", inlined)
        # ... and a passing one's is not, so a green run stays quiet.
        self.assertNotIn("| boot ok", inlined)

    def test_a_failure_names_the_cases_that_failed(self):
        summary = report.RunSummary()
        summary.add(
            report.TestResult(
                "unit_test.lz4",
                report.FAILED,
                message="2 of 3 Unity test(s) failed",
                cases=[
                    report.CaseResult("test_roundtrip", report.PASSED),
                    report.CaseResult(
                        "test_bounds",
                        report.FAILED,
                        message="expected -1, got 1024",
                        file="test/test_lz4.c",
                        line=58,
                    ),
                    report.CaseResult("test_slow_path", report.SKIPPED),
                ],
            )
        )

        text = report.format_summary(summary)
        self.assertIn("FAIL  test_bounds (test/test_lz4.c:58): expected -1, got 1024", text)
        # A passing or skipped case is not worth a line on a failure list.
        self.assertNotIn("test_roundtrip", text)
        self.assertNotIn("test_slow_path", text)


if __name__ == "__main__":
    unittest.main()
