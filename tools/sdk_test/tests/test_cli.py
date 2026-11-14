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

import json
import os
import unittest
from unittest import mock

from _support import TOOLS_DIR
from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from sdk_test import cli
from sdk_test.errors import BindingError
from sdk_test.errors import SdkTestError
from sdk_test.params import ParamsError


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


WIFI_CASE = """
tests:
  upload:
    harness: pytest
    harness_config:
      fixture: wifi_ap:5g
    params:
      cloud.token: {required: true}
"""

TWO_MISSING_CASES = """
tests:
  first:
    params:
      alpha: {required: true}
  second:
    params:
      beta: {required: true}
"""


class TestCheckParams(SdkTestCase):
    """``sdk.py test params check``: names and provenance, never values."""

    def setUp(self):
        super().setUp()
        self.tree = os.path.join(self.root, "cases")
        # The developer's own params file must not decide these tests.
        patched = mock.patch.dict(
            os.environ, {"USERPROFILE": self.root}, clear=False
        )
        patched.start()
        self.addCleanup(patched.stop)
        os.environ.pop("SIFLI_TEST_PARAMS_FILE", None)

    def write_case(self, body: str) -> str:
        self.write("cases/testcase.yaml", body)
        return self.tree

    def check(self, **kwargs):
        lines = []
        code = cli.check_params(self.tree, echo=lines.append, **kwargs)
        return code, "\n".join(lines)

    def test_the_json_report_lists_every_key_and_its_source(self):
        self.write_case(WIFI_CASE)
        path = self.write(
            "params.yaml",
            """
            fixtures:
              "wifi_ap:5g":
                ssid: LabAP
                psk: a-real-psk
            params:
              cloud.token: tok-abc
            """,
        )
        code, out = self.check(as_json=True, params_file=path)
        document = json.loads(out)

        self.assertEqual(code, cli.EXIT_OK)
        self.assertTrue(document["ok"])
        self.assertEqual(
            [row["name"] for row in document["params"]],
            [
                "cloud.token",
                "fixtures.wifi_ap:5g.ssid",
                "fixtures.wifi_ap:5g.security",
                "fixtures.wifi_ap:5g.psk",
            ],
        )
        self.assertTrue(all(row["source"] for row in document["params"]))
        # The values themselves are never printed.
        self.assertNotIn("a-real-psk", out)
        self.assertNotIn("tok-abc", out)

    def test_missing_keys_fail_and_every_case_is_reported(self):
        self.write_case(TWO_MISSING_CASES)
        lines = []
        with self.assertRaises(ParamsError) as caught:
            cli.check_params(self.tree, as_json=True, echo=lines.append)
        document = json.loads("\n".join(lines))
        self.assertFalse(document["ok"])
        self.assertEqual(
            [error["case"] for error in document["errors"]], ["first", "second"]
        )
        self.assertIn("2 test case(s)", str(caught.exception))

    def test_unused_lists_keys_no_selected_case_refers_to(self):
        self.write_case(
            """
            tests:
              only:
                params:
                  alpha: {required: true}
            """
        )
        path = self.write(
            "params.yaml", "params:\n  alpha: one\n  spare.key: two\n"
        )
        code, out = self.check(as_json=True, show_unused=True, params_file=path)
        document = json.loads(out)
        self.assertEqual(code, cli.EXIT_OK)
        self.assertEqual(document["unused"], ["spare.key"])

    def test_list_json_includes_params_key_names_without_a_params_file(self):
        self.write_case(WIFI_CASE)
        lines = []
        cli.list_tests(self.tree, as_json=True, echo=lines.append)
        document = json.loads("\n".join(lines))
        self.assertEqual(
            document[0]["params"],
            [
                "cloud.token",
                "fixtures.wifi_ap:5g.ssid",
                "fixtures.wifi_ap:5g.security",
                "fixtures.wifi_ap:5g.psk",
            ],
        )


class TestCheckBinding(SdkTestCase):
    def test_a_params_sourced_key_fails_and_is_named(self):
        path = self.write(
            "bad.yaml",
            """
            testbed:
              - id: tb
                platform: board_a
                connections:
                  - fixture: wifi_ap:5g
                    psk: leaked
            """,
        )
        lines = []
        with self.assertRaises(BindingError) as caught:
            cli.check_binding(path, echo=lines.append)
        text = "\n".join(lines)
        self.assertIn('"psk"', text)
        self.assertIn("params", text)
        self.assertIn("1 problem", str(caught.exception))

    def test_the_shipped_template_passes(self):
        example = os.path.join(
            TOOLS_DIR, "sdk_test", "examples", "binding.local.yaml"
        )
        lines = []
        self.assertEqual(
            cli.check_binding(example, echo=lines.append), cli.EXIT_OK
        )
        self.assertIn("ok", "\n".join(lines))


class TestParamOverrides(SdkTestCase):
    def test_a_value_without_an_equals_sign_is_rejected(self):
        import click

        from sdk_py_actions import test_ext

        with self.assertRaises(click.BadParameter):
            test_ext._parse_param_overrides(["bad"])

    def test_a_value_may_contain_an_equals_sign(self):
        from sdk_py_actions import test_ext

        self.assertEqual(
            test_ext._parse_param_overrides(["k=a=b"]), {"k": "a=b"}
        )


class TestPlanOptions(SdkTestCase):
    """``test plan`` resolves with the same options and rule as ``test run``."""

    def setUp(self):
        super().setUp()
        patched = mock.patch.dict(
            os.environ, {"USERPROFILE": self.root}, clear=False
        )
        patched.start()
        self.addCleanup(patched.stop)
        os.environ.pop("SIFLI_TEST_PARAMS_FILE", None)
        self.write(
            "cases/testcase.yaml",
            """
            tests:
              upload:
                harness: pytest
                harness_config:
                  fixture: wifi_ap:5g
                params:
                  cloud.token: {required: true}
            """,
        )
        self.write("cases/project/SConstruct", "")
        self.write("cases/project/proj.conf", "")
        self.tree = os.path.join(self.root, "cases")

    def test_the_plan_uses_the_params_file_it_was_given(self):
        path = self.write(
            "params.yaml",
            """
            fixtures:
              "wifi_ap:5g":
                ssid: LabAP
                psk: a-real-psk
            params:
              cloud.token: tok-abc
            """,
        )
        lines = []
        code = cli.describe_plan(
            self.tree,
            platform="board_a",
            params_file=path,
            echo=lines.append,
        )
        self.assertEqual(code, cli.EXIT_OK)
        self.assertIn("upload", "\n".join(lines))

        # Without it the same plan is refused, which is what proves the option
        # above was the difference rather than a default that happened to work.
        with self.assertRaises(ParamsError):
            cli.describe_plan(self.tree, platform="board_a", echo=lambda _m: None)


class TestOptionConsistency(SdkTestCase):
    """The three parameter-aware commands take the same options."""

    def options_of(self, path: str) -> set:
        from sdk_py_actions import test_ext
        from sdk_py_actions.cli.registry import CommandRegistry

        registry = CommandRegistry()
        test_ext.register(registry)
        return {
            name
            for option in registry._commands[path].options
            for name in option.names
        }

    def test_plan_run_and_params_check_share_the_parameter_options(self):
        required = {"--params-file", "--param", "--agent-url"}
        for path in ("test/plan", "test/run", "test/params/check"):
            with self.subTest(command=path):
                self.assertTrue(required <= self.options_of(path))


if __name__ == "__main__":
    unittest.main()
