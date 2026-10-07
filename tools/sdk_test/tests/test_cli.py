# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the ``sdk.py test`` command layer.

Two things are pinned here that are easy to get wrong and expensive to miss:

* the **exit code** of a failing run. Reporting success for a failed test run is
  the one failure this framework must never have -- CI would go green on a
  broken build -- and the obvious implementation (raising
  ``click.exceptions.Exit``) silently stopped working on click 8.3.
* that ``list``/``plan`` stay usable without any hardware, which is what makes
  them safe to wire into a pipeline.
"""

from __future__ import annotations

import os
import unittest

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from sdk_test import cli
from sdk_test.errors import SdkTestError


class TestExitCodes(SdkTestCase):
    """The contract: 0 all passed, 1 something failed, 2 the request was unusable."""

    def test_the_codes_are_what_the_documentation_says(self):
        self.assertEqual(cli.EXIT_OK, 0)
        self.assertEqual(cli.EXIT_FAILURES, 1)
        self.assertEqual(cli.EXIT_USAGE, 2)

    def test_a_failing_run_returns_the_failure_code(self):
        summary = _summary_with_failure()
        self.assertFalse(summary.ok)
        # Mirrors run_tests' final decision.
        self.assertEqual(
            cli.EXIT_OK if summary.ok else cli.EXIT_FAILURES, cli.EXIT_FAILURES
        )


def _summary_with_failure():
    from sdk_test.report import FAILED
    from sdk_test.report import RunSummary
    from sdk_test.report import TestResult

    summary = RunSummary()
    summary.add(TestResult(id="a/b", status=FAILED, message="nope"))
    return summary


class TestGuardTurnsCodesIntoProcessStatus(SdkTestCase):
    """``test_ext._guard`` is where a library return code becomes an exit status.

    It raises ``FatalError(exit_code=...)`` rather than ``click.exceptions.Exit``
    on purpose. Raising ``Exit`` from a command callback is the natural way to
    carry a status out of click, and it worked on older click -- but click 8.3
    swallows it under ``standalone_mode=False``, and the process then exits 0.
    """

    def setUp(self):
        super().setUp()
        try:
            from sdk_py_actions import test_ext
        except ImportError as exc:  # pragma: no cover - environment problem
            self.skipTest(f"sdk_py_actions is not importable: {exc}")
        self.test_ext = test_ext

    def test_a_failure_code_becomes_a_fatal_error_carrying_it(self):
        from sdk_py_actions.errors import FatalError

        @self.test_ext._guard
        def callback(sdk_ctx, **kwargs):
            return cli.EXIT_FAILURES

        with self.assertRaises(FatalError) as caught:
            callback(None)
        self.assertEqual(caught.exception.exit_code, cli.EXIT_FAILURES)

    def test_success_is_returned_rather_than_raised(self):
        @self.test_ext._guard
        def callback(sdk_ctx, **kwargs):
            return cli.EXIT_OK

        self.assertEqual(callback(None), cli.EXIT_OK)

    def test_the_failure_message_says_what_happened(self):
        # "tests failed" is a result, not a fault, so the wording must not read
        # like an internal error.
        from sdk_py_actions.errors import FatalError

        @self.test_ext._guard
        def callback(sdk_ctx, **kwargs):
            return cli.EXIT_FAILURES

        with self.assertRaises(FatalError) as caught:
            callback(None)
        self.assertIn("test", str(caught.exception).lower())

    def test_library_errors_become_cli_errors(self):
        from sdk_py_actions.errors import FatalError

        @self.test_ext._guard
        def callback(sdk_ctx, **kwargs):
            raise SdkTestError("something specific went wrong")

        with self.assertRaises(FatalError) as caught:
            callback(None)
        self.assertIn("something specific went wrong", str(caught.exception))


class TestListingNeedsNoHardware(SdkTestCase):
    """``list`` and ``plan`` must work with no Lab Agent and no board."""

    def setUp(self):
        super().setUp()
        self.suite = self.write(
            "example/demo/testcase.yaml",
            """
            tests:
              - id: alpha
                description: first case
                harness: console
                tags: [smoke]
                timeout: 42
                harness_config:
                  record: ["hello"]
              - id: beta
                harness: unity
                skip: true
            """,
        )
        self.write("example/demo/project/SConstruct", "# project\n")
        self.write("example/demo/project/proj.conf", "CONFIG_X=y\n")
        self.suite_root = os.path.join(self.root, "example", "demo")

    def test_list_prints_every_case(self):
        lines = []
        code = cli.list_tests(self.suite_root, echo=lines.append)

        self.assertEqual(code, cli.EXIT_OK)
        text = "\n".join(lines)
        self.assertIn("alpha", text)
        self.assertIn("beta", text)

    def test_list_marks_a_skipped_case(self):
        lines = []
        cli.list_tests(self.suite_root, echo=lines.append)
        self.assertIn("skip", "\n".join(lines))

    def test_list_as_json_is_machine_readable(self):
        import json

        lines = []
        cli.list_tests(self.suite_root, as_json=True, echo=lines.append)
        payload = json.loads("\n".join(lines))

        by_id = {entry["id"]: entry for entry in payload}
        self.assertEqual(by_id["alpha"]["timeout"], 42)
        self.assertEqual(by_id["alpha"]["tags"], ["smoke"])
        self.assertTrue(by_id["beta"]["skip"])

    def test_list_reports_an_empty_tree_without_failing(self):
        empty = self.mkdir("empty")
        lines = []
        self.assertEqual(cli.list_tests(empty, echo=lines.append), cli.EXIT_OK)
        self.assertIn("No test cases found", "\n".join(lines))

    def test_plan_resolves_a_case_that_names_its_own_platform(self):
        # integration_platforms lets a case name its carrier directly, which is
        # how the QEMU cases run with no hardware and no Lab Agent at all.
        self.write(
            "example/solo/testcase.yaml",
            """
            tests:
              - id: solo
                harness: console
                integration_platforms: [qemu_sf32lb52x]
                harness_config:
                  record: ["x"]
            """,
        )
        self.write("example/solo/project/SConstruct", "# project\n")
        self.write("example/solo/project/proj.conf", "CONFIG_X=y\n")

        lines = []
        code = cli.describe_plan(os.path.join(self.root, "example", "solo"), echo=lines.append)
        text = "\n".join(lines)

        self.assertEqual(code, cli.EXIT_OK)
        self.assertIn("qemu_sf32lb52x", text)
        self.assertIn("(qemu)", text)
        # And it should say plainly that no hardware is involved.
        self.assertIn("no Lab Agent needed", text)

    def test_plan_lists_every_run_of_a_multi_platform_case(self):
        # Two platforms, two runs. The header has to say so, or a dry run
        # under-reports how many boards the run will occupy -- which is the one
        # thing someone reads it for before starting.
        self.write(
            "example/multi/testcase.yaml",
            """
            tests:
              - id: multi
                harness: console
                integration_platforms: [qemu_sf32lb55x, qemu_sf32lb52x]
                harness_config:
                  record: ["x"]
            """,
        )
        self.write("example/multi/project/SConstruct", "# project\n")
        self.write("example/multi/project/proj.conf", "CONFIG_X=y\n")

        lines = []
        code = cli.describe_plan(
            os.path.join(self.root, "example", "multi"), echo=lines.append
        )
        text = "\n".join(lines)

        self.assertEqual(code, cli.EXIT_OK)
        # One test case, two runs -- and the count says both numbers rather than
        # picking one of them.
        self.assertIn("1 test(s) would run, 2 run(s) in all:", text)
        self.assertIn("[1/2]", text)
        self.assertIn("[2/2]", text)

    def test_plan_keeps_the_single_run_header_when_there_is_only_one_per_case(self):
        self.write(
            "example/single/testcase.yaml",
            """
            tests:
              - id: single
                harness: console
                integration_platforms: [qemu_sf32lb52x]
                harness_config:
                  record: ["x"]
            """,
        )
        self.write("example/single/project/SConstruct", "# project\n")
        self.write("example/single/project/proj.conf", "CONFIG_X=y\n")

        lines = []
        cli.describe_plan(
            os.path.join(self.root, "example", "single"), echo=lines.append
        )
        text = "\n".join(lines)

        self.assertIn("1 test(s) would run:", text)
        self.assertNotIn("[1/1]", text)

    def test_plan_reports_nothing_to_do_rather_than_failing(self):
        empty = self.mkdir("empty2")
        lines = []
        self.assertEqual(cli.describe_plan(empty, echo=lines.append), cli.EXIT_OK)
        self.assertIn("Nothing to run", "\n".join(lines))

    def test_plan_shows_the_case_the_named_platform_turns_away(self):
        # `-p` asks for one board; a case whose own gate refuses that board is
        # not an error, and a dry run that hid it would leave the reader
        # guessing whether the case was dropped on purpose.
        self.write(
            "example/gated/testcase.yaml",
            """
            tests:
              - id: gated.board
                harness: console
                harness_config:
                  record: ["x"]
              - id: gated.qemu
                harness: console
                platform_allow: [qemu_sf32lb55x]
                integration_platforms: [qemu_sf32lb55x]
                harness_config:
                  record: ["x"]
            """,
        )
        self.write("example/gated/project/SConstruct", "# project\n")
        self.write("example/gated/project/proj.conf", "CONFIG_X=y\n")

        lines = []
        code = cli.describe_plan(
            os.path.join(self.root, "example", "gated"),
            platform="sf32lb52-lcd_a128r16",
            echo=lines.append,
        )
        text = "\n".join(lines)

        self.assertEqual(code, cli.EXIT_OK)
        self.assertIn("gated.board", text)
        self.assertIn("gated.qemu", text)
        self.assertIn("turned away", text)
        self.assertIn("not allowed", text)
        # The turned-away pairs come after the runs, not instead of them.
        self.assertLess(text.index("sf32lb52-lcd_a128r16 (board)"),
                        text.index("turned away"))

    def test_plan_prints_the_turned_away_block_verbatim(self):
        # The block is now shared with the run manifest, so its wording is the
        # one both print. Asserted literally rather than by keyword: a change to
        # it silently changes what `test run` says at its opening too.
        self.write(
            "example/verbatim/testcase.yaml",
            """
            tests:
              - id: verbatim.board
                harness: console
                harness_config:
                  record: ["x"]
              - id: verbatim.qemu
                harness: console
                platform_allow: [qemu_sf32lb55x]
                harness_config:
                  record: ["x"]
            """,
        )
        self.write("example/verbatim/project/SConstruct", "# project\n")
        self.write("example/verbatim/project/proj.conf", "CONFIG_X=y\n")

        lines = []
        cli.describe_plan(
            os.path.join(self.root, "example", "verbatim"),
            platform="sf32lb52-lcd_a128r16",
            echo=lines.append,
        )

        # The block is echoed as one string, the way the run manifest is.
        self.assertEqual(lines[-1].splitlines(), [
            "1 (test, platform) pair(s) turned away:",
            "  verbatim.qemu",
            "    platform:   sf32lb52-lcd_a128r16",
            "    reason:     platform sf32lb52-lcd_a128r16 not allowed "
            "(allow=['qemu_sf32lb55x'], exclude=none)",
        ])

    def test_plan_says_a_turned_away_tree_is_not_an_empty_one(self):
        # Cases were selected and every one of them was refused: saying only
        # "Nothing to run." would read as an empty tree.
        self.write(
            "example/qonly/testcase.yaml",
            """
            tests:
              - id: only.qemu
                harness: console
                platform_allow: [qemu_sf32lb55x]
                integration_platforms: [qemu_sf32lb55x]
                harness_config:
                  record: ["x"]
            """,
        )
        self.write("example/qonly/project/SConstruct", "# project\n")
        self.write("example/qonly/project/proj.conf", "CONFIG_X=y\n")

        lines = []
        code = cli.describe_plan(
            os.path.join(self.root, "example", "qonly"),
            platform="sf32lb52-lcd_a128r16",
            echo=lines.append,
        )
        text = "\n".join(lines)

        self.assertEqual(code, cli.EXIT_OK)
        self.assertIn("only.qemu", text)
        self.assertIn("turned away", text)
        self.assertIn("not allowed", text)

    def test_environment_report_lists_harnesses_and_backends(self):
        lines = []
        cli.describe_environment(echo=lines.append)
        text = "\n".join(lines)
        self.assertIn("console", text)
        self.assertIn("unity", text)
        self.assertIn("pytest", text)
        self.assertIn("board", text)
        self.assertIn("qemu", text)


if __name__ == "__main__":
    unittest.main()
