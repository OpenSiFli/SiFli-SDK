# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end tests for the whole SDK-side chain, with no hardware.

A real :class:`~lab_agent.server.LabAgentServer` runs on an ephemeral port with a
faked console and probe; the SDK side discovers a real ``testcase.yaml``, names a
platform, allocates a board from the agent, opens the console, runs the harness
and writes a real JUnit report. Only the two physical boundaries are faked.

This is the test that would have caught every integration bug found while
building the two halves, because it exercises the actual JSON contract between
them rather than either side's idea of it.

The negative case is the important one. A framework that reports success when
the device output does not match is worse than no framework, because it is
trusted -- so "the expectation is wrong and the run must fail" is asserted here,
not just the happy path.
"""

from __future__ import annotations

import json
import os
import time
import unittest
import xml.etree.ElementTree as ElementTree

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from lab_agent.config import parse_runner_config
from lab_agent.server import LabAgentServer
from lab_agent.testbed import LabManager
from sdk_test.errors import BuildError
from sdk_test.report import ERROR
from sdk_test.report import FAILED
from sdk_test.report import PASSED
from sdk_test.report import SKIPPED
from sdk_test.runner import RunOptions
from sdk_test.runner import TestRunner

#: What the faked device prints on boot -- shaped like a real RT-Thread log,
#: including the interleaved noise the console harness has to tolerate.
BOOT_LOG = (
    " \r\n"
    " | / - RT -     Thread Operating System\r\n"
    " 2006 - 2024 Copyright by RT-Thread team\r\n"
    "[I/DBG] nor flash id 0x1820c2\r\n"
    "Hello world!\r\n"
    "msh />"
)

#: What a Unity binary prints when it runs. The file is the source Unity was
#: handed, the case is qualified by its suite, and the number on a PASS line is
#: the case's ordinal -- only the FAIL line's number is a source line.
UNITY_LOG = (
    "\r\n"
    "test/test_lz4.c:1:lz4.roundtrip:PASS\r\n"
    "test/test_lz4.c:2:lz4.boundaries:PASS\r\n"
    "\r\n"
    "-----------------------\r\n"
    "2 Tests 0 Failures 0 Ignored\r\n"
    "OK\r\n"
)


class FakeTransport:
    """A console that behaves like a device rather than like a tape recorder.

    It emits ``boot`` on its own, then stays quiet until the host sends a line
    containing ``trigger``, and only then emits ``response``. Modelling that
    matters: the Unity driver deliberately discards output that predates its
    command so a previous run's summary cannot be mistaken for this one's, and a
    fake that replays a fixed script regardless of input would silently defeat
    that check instead of exercising it.
    """

    def __init__(self, boot: str = "", response: str = "", trigger: str = "unity_run") -> None:
        self._stream = boot
        self.response = response
        self.trigger = trigger
        self.written = []
        self.opened = 0
        self.closed = 0
        self._cursor = 0

    def open(self) -> None:
        self.opened += 1

    def close(self) -> None:
        self.closed += 1

    def read(self, timeout: float = 0.05) -> str:
        if self._cursor >= len(self._stream):
            # Block briefly like a real port would, so a reader loop cannot spin.
            time.sleep(0.001)
            return ""
        char = self._stream[self._cursor]
        self._cursor += 1
        return char

    def write(self, text: str) -> None:
        self.written.append(text)
        if self.trigger and self.trigger in text:
            self._stream += self.response

    @property
    def sent(self) -> str:
        return "".join(self.written)


class FakeProbe:
    def __init__(self) -> None:
        self.resets = []
        self.flashed = []

    def reset(self, extra_script=None) -> str:
        self.resets.append(extra_script)
        return "ok"

    def flash(self, build_dir: str) -> str:
        self.flashed.append(build_dir)
        return "ok"


class EndToEndCase(SdkTestCase):
    """Builds a suite tree and a live agent with a faked device.

    The platform is named on this side, as a user's `-p` would name it; the
    agent then picks the board that answers it out of the lab it was given.
    """

    #: Transcript the faked device emits unprompted (the boot log).
    boot = BOOT_LOG
    #: Transcript it emits only after being asked to run Unity.
    response = ""

    #: The platform the suite is built for. Allocation is the agent's job, so
    #: this is all the SDK side says; which board answers it is not the runner's
    #: to choose. Overridden where a bench serves different hardware.
    platform = "sf32lb52-lcd_n16r8"

    #: The lab the agent serves. Override in a subclass to describe a different
    #: bench.
    testbeds_yaml = """
    runner:
      name: e2e
      console_poll: 0.005

    testbed:
      - id: tb_01
        platform: sf32lb52-lcd_n16r8
        console:
          port: COM13
        connections:
          - fixture: spi_nor_flash:spi1
    """

    def setUp(self):
        super().setUp()
        self.transports = {}
        self.probes = {}
        self._start_agent()
        self.addCleanup(self.server.stop)

    def _start_agent(self):
        manager = LabManager(
            parse_runner_config(
                __import__("yaml").safe_load(self.testbeds_yaml), source="<test>"
            ),
            sdk_root=self.root,
            console_factory=self._console_factory,
            jlink_factory=self._probe_factory,
        )
        manager.config.settings.host = "127.0.0.1"
        manager.config.settings.port = 0
        self.manager = manager
        self.server = LabAgentServer(manager)
        self.server.start_background()

    def _console_factory(self, testbed):
        transport = FakeTransport(
            boot=self.boot, response=self.response
        )
        self.transports[testbed.id] = transport
        return transport

    def _probe_factory(self, testbed, settings):
        probe = FakeProbe()
        self.probes[testbed.id] = probe
        return probe

    def make_project(self, suffix="rtt", testcase_yaml="", extra_files=()):
        """Lay out a project the way the repository does: testcase beside project/."""
        base = f"example/{suffix}"
        self.write(f"{base}/testcase.yaml", testcase_yaml)
        self.write(f"{base}/project/SConstruct", "# a project\n")
        self.write(f"{base}/project/proj.conf", "CONFIG_X=y\n")
        for name, content in extra_files:
            self.write(f"{base}/{name}", content)
        return os.path.join(self.root, base.replace("/", os.sep))

    def harness_case(self, harness="console"):
        """A one-test project for ``harness``, laid out the way the repo does."""
        return self.make_project(
            testcase_yaml=f"""
            tests:
              - id: hello.{harness}
                harness: {harness}
                harness_config:
                  {"cmd: unity_run" if harness == "unity" else 'record: ["Hello world!"]'}
            """,
            suffix="hello",
        )

    def runner(self, suite_root, echo=None, confirm=None, build_fn=None, **overrides):
        options = RunOptions(
            testsuite_root=suite_root,
            platform=self.platform,
            agent_url=self.server.url,
            build=False,
            # Kept short so a deliberately-failing test does not pad the suite.
            timeout=5,
            # Artifacts under the test's own temp root: a run writes a log tree
            # wherever it is pointed, and the default is beside the working
            # directory.
            out_dir=self.root,
        )
        for key, value in overrides.items():
            setattr(options, key, value)
        return TestRunner(options, build_fn=build_fn, echo=echo, confirm=confirm)


class TestConsoleHarnessEndToEnd(EndToEndCase):
    def test_a_matching_boot_log_passes(self):
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: hello.console
                harness: console
                harness_config:
                  record:
                    - "Thread Operating System"
                    - "Hello world!"
                    - "msh />"
            """,
            suffix="hello",
        )
        summary = self.runner(suite).run()

        self.assertEqual(summary.total, 1)
        result = summary.results[0]
        self.assertEqual(result.status, PASSED, result.message)
        self.assertEqual(result.testbed_id, "tb_01")
        self.assertEqual(result.platform, "sf32lb52-lcd_n16r8")
        self.assertIn("Hello world!", result.log)

    def test_a_wrong_expectation_fails_the_run(self):
        # The anti-vacuous case: if this passed, nothing else in the framework
        # could be trusted.
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: hello.console
                harness: console
                timeout: 2
                harness_config:
                  record: ["Hello worldXYZ!"]
            """,
            suffix="hello_wrong",
        )
        summary = self.runner(suite).run()

        self.assertEqual(summary.count(FAILED), 1)
        self.assertFalse(summary.ok)
        self.assertIn("Hello worldXYZ!", summary.results[0].message)

    def test_out_of_order_boot_lines_fail(self):
        # The device printed the banner before "Hello world!", but the testcase
        # asks for the reverse; ordered matching must reject it.
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: hello.reversed
                harness: console
                timeout: 2
                harness_config:
                  record: ["Hello world!", "Thread Operating System"]
            """,
            suffix="hello_reversed",
        )
        summary = self.runner(suite).run()
        self.assertEqual(summary.count(FAILED), 1)


class TestUnityHarnessEndToEnd(EndToEndCase):
    boot = BOOT_LOG
    response = UNITY_LOG

    def test_a_unity_suite_runs_and_passes(self):
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: unit.lz4
                harness: unity
                harness_config:
                  suite: lz4
            """,
            suffix="unit",
        )
        summary = self.runner(suite).run()
        result = summary.results[0]

        self.assertEqual(result.status, PASSED, result.message)
        self.assertIn("2 Unity test(s) passed", result.message)
        # The host asked the device to run the suite it was told to.
        self.assertIn("unity_run lz4", self.transports["tb_01"].sent)

    def test_a_failing_unity_run_is_reported_as_a_failure(self):
        self.response = (
            "\r\n"
            "test/test_lz4.c:129:lz4.boundaries:FAIL:Expected 1 Was 2\r\n"
            "\r\n1 Tests 1 Failures 0 Ignored\r\nFAIL\r\n"
        )
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: unit.lz4
                harness: unity
            """,
            suffix="unit_fail",
        )
        summary = self.runner(suite).run()
        result = summary.results[0]

        self.assertEqual(result.status, FAILED, result.message)
        # Named the way the device printed it: suite and case, so a reader can
        # tell which suite's case this is without opening the transcript.
        self.assertIn("lz4.boundaries", result.message)

    def test_a_vacuous_unity_run_is_refused(self):
        # A binary that linked Unity but ran nothing prints a clean "OK".
        self.response = "0 Tests 0 Failures 0 Ignored\r\nOK\r\n"
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: unit.empty
                harness: unity
            """,
            suffix="unit_empty",
        )
        summary = self.runner(suite).run()
        result = summary.results[0]

        self.assertEqual(result.status, FAILED, result.message)
        self.assertIn("vacuous", result.message)

    def test_a_silent_device_times_out_rather_than_passing(self):
        # Nothing at all comes back: the harness must fail, not shrug.
        self.response = ""
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: unit.silent
                harness: unity
                timeout: 2
            """,
            suffix="unit_silent",
        )
        summary = self.runner(suite).run()
        result = summary.results[0]

        self.assertEqual(result.status, FAILED, result.message)
        self.assertIn("no Unity summary", result.message)


class TestSessionLifecycle(EndToEndCase):
    def test_the_testbed_is_released_and_the_console_closed(self):
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: hello.console
                harness: console
                harness_config:
                  record: ["Hello world!"]
            """,
            suffix="hello",
        )
        self.runner(suite).run()

        self.assertFalse(self.manager.get("tb_01").busy)
        self.assertEqual(self.transports["tb_01"].closed, 1)

    def test_the_board_is_flashed_then_reset_with_the_console_already_open(self):
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: hello.console
                harness: console
                harness_config:
                  record: ["Hello world!"]
            """,
            suffix="hello",
        )
        self.runner(suite).run()

        probe = self.probes["tb_01"]
        self.assertEqual(len(probe.flashed), 1)
        self.assertTrue(probe.flashed[0].endswith("build_sf32lb52-lcd_n16r8_hcpu"))
        self.assertEqual(len(probe.resets), 1)
        # The console must be open before the reset, or the boot log races it.
        self.assertEqual(self.transports["tb_01"].opened, 1)

    def test_a_second_run_can_reuse_the_testbed(self):
        # Proves the release actually happened rather than being reported.
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: hello.console
                harness: console
                harness_config:
                  record: ["Hello world!"]
            """,
            suffix="hello",
        )
        runner = self.runner(suite)
        for _ in range(2):
            summary = runner.run()
            self.assertEqual(summary.results[0].status, PASSED)


class TestSelectionAndGating(EndToEndCase):
    def test_selecting_an_unknown_test_id_fails_with_the_available_list(self):
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: real
                harness: console
                harness_config: {record: ["x"]}
            """,
            suffix="sel",
        )
        runner = self.runner(suite, testcase_ids=["nope"])
        from sdk_test.errors import DiscoveryError

        with self.assertRaises(DiscoveryError) as caught:
            runner.plan()
        self.assertIn("nope", str(caught.exception))
        self.assertIn("real", str(caught.exception))
        self.assertIn("Available", str(caught.exception))

    def test_skip_produces_a_skipped_result_without_touching_hardware(self):
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: skipped
                skip: true
                harness: console
                harness_config: {record: ["nothing"]}
            """,
            suffix="skip",
        )
        summary = self.runner(suite).run()

        self.assertEqual(summary.results[0].status, SKIPPED)
        self.assertTrue(summary.ok, "a skip must not fail a run")
        self.assertEqual(self.transports, {}, "a skipped test must not open a console")

    def test_build_only_builds_and_does_not_execute(self):
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: buildsonly
                build_only: true
                harness: console
                harness_config: {record: ["nothing"]}
            """,
            suffix="buildonly",
        )
        summary = self.runner(suite, build=False).run()
        self.assertEqual(summary.results[0].status, SKIPPED)

    def test_a_fixture_the_lab_does_not_have_is_explained(self):
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: needs.sensor
                harness: console
                harness_config:
                  fixture: i2c_sensor
                  record: ["x"]
            """,
            suffix="fixture",
        )
        summary = self.runner(suite).run()
        result = summary.results[0]

        self.assertEqual(result.status, ERROR)
        self.assertIn("i2c_sensor", result.message)

    def test_one_misconfigured_test_does_not_stop_the_rest(self):
        # A nightly run has to be usable for diagnosing the tests that *can*
        # run, so a test with no available hardware must not abort the suite.
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: needs.missing_sensor
                harness: console
                harness_config:
                  fixture: i2c_sensor
                  record: ["x"]
              - id: works
                harness: console
                harness_config:
                  record: ["Hello world!"]
            """,
            suffix="mixed",
        )
        summary = self.runner(suite).run()

        by_id = {result.id: result for result in summary.results}
        self.assertEqual(len(summary.results), 2)
        self.assertEqual(by_id["needs.missing_sensor"].status, ERROR)
        self.assertIn("i2c_sensor", by_id["needs.missing_sensor"].message)
        self.assertEqual(by_id["works"].status, PASSED, by_id["works"].message)

    def test_tags_select_a_subset(self):
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: quick
                tags: [quick]
                harness: console
                harness_config: {record: ["Hello world!"]}
              - id: slow
                tags: [slow]
                harness: console
                harness_config: {record: ["Hello world!"]}
            """,
            suffix="tags",
        )
        summary = self.runner(suite, tags=["slow"]).run()
        # The testcase file sits at the suite root here, so the id has no prefix.
        self.assertEqual([result.id for result in summary.results], ["slow"])


class TestReporting(EndToEndCase):
    def test_a_junit_report_is_written_with_the_hardware_identity(self):
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: hello.console
                harness: console
                harness_config:
                  record: ["Hello world!"]
            """,
            suffix="hello",
        )
        report_path = os.path.join(self.root, "out", "junit.xml")
        self.runner(suite, report_path=report_path).run()

        root = ElementTree.parse(report_path).getroot()
        self.assertEqual(root.get("tests"), "1")
        self.assertEqual(root.get("failures"), "0")
        case = root.find("testcase")
        self.assertEqual(case.get("testbed"), "tb_01")
        self.assertEqual(case.get("platform"), "sf32lb52-lcd_n16r8")

    def test_a_failure_is_recorded_in_the_report(self):
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: bad
                harness: console
                timeout: 2
                harness_config: {record: ["absent string"]}
            """,
            suffix="bad",
        )
        report_path = os.path.join(self.root, "junit.xml")
        self.runner(suite, report_path=report_path).run()

        root = ElementTree.parse(report_path).getroot()
        self.assertEqual(root.get("failures"), "1")
        self.assertIsNotNone(root.find("testcase").find("failure"))


class TestPlanIsHonest(EndToEndCase):
    def test_planning_resolves_platform_project_and_build_dir(self):
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: hello.console
                harness: console
                harness_config: {record: ["x"]}
            """,
            suffix="hello",
        )
        planned = self.runner(suite).plan()

        self.assertEqual(len(planned), 1)
        item = planned[0]
        self.assertEqual(item.platform, "sf32lb52-lcd_n16r8")
        self.assertEqual(item.backend, "board")
        self.assertTrue(item.project_dir.endswith(os.path.join("hello", "project")))
        # The build system always records a core suffix, even when the platform
        # name has none -- so the directory is build_<board>_hcpu.
        self.assertTrue(item.build_dir.endswith("build_sf32lb52-lcd_n16r8_hcpu"))
        # And no board is named: the agent chooses one from its own lab when the
        # hardware is asked for, so a plan has nothing to say about which.
        self.assertFalse(hasattr(item, "testbed"))

    def test_a_testcase_beside_its_project_is_found_by_convention(self):
        # testcase.yaml next to SConstruct, with no project/ subdirectory.
        self.write(
            "example/flat/testcase.yaml",
            "tests:\n  - id: t\n    harness: console\n    harness_config: {record: [x]}\n",
        )
        self.write("example/flat/SConstruct", "# project\n")
        self.write("example/flat/proj.conf", "CONFIG_X=y\n")

        planned = self.runner(os.path.join(self.root, "example", "flat")).plan()
        self.assertTrue(
            planned[0].project_dir.endswith(os.path.join("example", "flat"))
        )

    def test_an_ambiguous_project_is_refused_rather_than_guessed(self):
        self.write(
            "example/amb/testcase.yaml",
            "tests:\n  - id: t\n    harness: console\n    harness_config: {record: [x]}\n",
        )
        for name in ("one", "two"):
            self.write(f"example/amb/{name}/SConstruct", "# project\n")
            self.write(f"example/amb/{name}/proj.conf", "CONFIG_X=y\n")

        from sdk_test.errors import DiscoveryError

        with self.assertRaises(DiscoveryError) as caught:
            self.runner(os.path.join(self.root, "example", "amb")).plan()
        message = str(caught.exception)
        self.assertIn("several projects", message)
        self.assertIn("one", message)
        self.assertIn("two", message)

    def test_an_explicit_project_dir_wins(self):
        self.write(
            "example/pick/testcase.yaml",
            """
            tests:
              - id: t
                project_dir: elsewhere
                harness: console
                harness_config: {record: [x]}
            """,
        )
        self.write("example/pick/elsewhere/SConstruct", "# project\n")
        self.write("example/pick/elsewhere/proj.conf", "CONFIG_X=y\n")

        planned = self.runner(os.path.join(self.root, "example", "pick")).plan()
        self.assertTrue(planned[0].project_dir.endswith("elsewhere"))

    def test_a_project_dir_that_is_not_a_project_is_reported(self):
        self.write(
            "example/bad/testcase.yaml",
            """
            tests:
              - id: t
                project_dir: nowhere
                harness: console
                harness_config: {record: [x]}
            """,
        )
        from sdk_test.errors import DiscoveryError

        with self.assertRaises(DiscoveryError) as caught:
            self.runner(os.path.join(self.root, "example", "bad")).plan()
        self.assertIn("nowhere", str(caught.exception))


#: A bench with no probe. Download still goes through one in this fake lab --
#: what makes it manual is only that nothing resets the board automatically.
MANUAL_RESET_BENCH = """
runner:
  name: e2e
  console_poll: 0.005

testbed:
  - id: tb_manual
    platform: sf32lb52-lcd_a128r16
    console:
      port: COM8
    reset:
      method: manual
"""


class TestPlanListsEveryRun(EndToEndCase):
    """A dry run lists runs, not test cases.

    A case that names two platforms runs twice, on two boards. Reporting it once
    under-reported the work by exactly the factor a dry run is read for before
    anything is built: how many boards it takes, and therefore how long.
    """

    def test_a_case_naming_two_platforms_gets_two_runs(self):
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: hello.console
                harness: console
                integration_platforms: [sf32lb52-lcd_n16r8, eh-lb563]
                harness_config:
                  record: ["Hello world!"]
            """,
            suffix="hello",
        )
        # Named no platform on the command line (``platform=None``), so the
        # case's own two are what is planned.
        planned = self.runner(suite, platform=None).plan()

        # Both runs, in the order the test case named them, and both under the
        # one id -- which is what the summary groups and counts by.
        self.assertEqual(
            [item.platform for item in planned],
            ["sf32lb52-lcd_n16r8", "eh-lb563"],
        )
        # Both under the one id -- which is what the summary groups and counts by.
        self.assertEqual(len({item.id for item in planned}), 1)

    def test_a_case_naming_none_is_refused_rather_than_guessed_at(self):
        # Without a platform named on the command line or in the case there is
        # nothing to build for. The SDK side holds no inventory of the lab to
        # choose one from any more, so this is a usage error, and it says so.
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: hello.console
                harness: console
                harness_config:
                  record: ["Hello world!"]
            """,
            suffix="hello",
        )
        from sdk_test.errors import BackendError

        with self.assertRaises(BackendError) as caught:
            self.runner(suite, platform=None).plan()
        message = str(caught.exception)
        self.assertIn("integration_platforms", message)
        self.assertIn("--platform", message)

    def test_a_platform_forced_on_the_command_line_replaces_the_case_s_own(self):
        # `--platform` wins over `integration_platforms`, which is what makes
        # "run everything on this board" expressible without editing cases.
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: hello.console
                harness: console
                integration_platforms: [eh-lb563]
                harness_config:
                  record: ["Hello world!"]
            """,
            suffix="hello",
        )
        planned = self.runner(
            suite, platform="sf32lb52-lcd_n16r8"
        ).plan()

        self.assertEqual([item.platform for item in planned], ["sf32lb52-lcd_n16r8"])


class EndToEndManualResetCase(EndToEndCase):
    """The board has no probe: an operator presses the reset button.

    Nothing on this side can tell a board that rebooted from one that was
    already running, so the run does not pretend to know -- it asks, and the
    operator's reaction time comes out of the harness's own budget.
    """

    testbeds_yaml = MANUAL_RESET_BENCH
    platform = "sf32lb52-lcd_a128r16"
    #: What the faked device answers a `unity_run` with, so the unity cases
    #: below have a reply to find. The console cases never send that command,
    #: so it stays silent for them.
    response = UNITY_LOG

    def test_the_operator_is_told_the_board_is_not_reset_for_them(self):
        # This run also pins the honest half of the contract: the faked device
        # emits its boot log unprompted, so the test passes with nobody
        # pressing anything. That is what really happens on a UART-downloaded
        # board -- the download boots the new image and the console catches it
        # -- which is why the prompt must offer the press as a fallback rather
        # than demand it.
        lines = []
        summary = self.runner(self.harness_case(), echo=lines.append).run()
        self.assertEqual(summary.passed, 1)

        prompts = [line for line in lines if line.startswith("[reset]")]
        self.assertEqual(len(prompts), 1)
        # It has to name the board: with several benches in a lab, "waiting for
        # a boot" on its own is not an instruction.
        self.assertIn("tb_manual", prompts[0])
        self.assertIn("RESET", prompts[0])
        self.assertIn("if nothing appears", prompts[0])
        # And the window, because that is what the fallback is racing.
        self.assertIn("5s", prompts[0])

    def test_the_probe_is_never_asked_to_reset(self):
        self.runner(self.harness_case()).run()

        probe = self.probes["tb_manual"]
        # The download still happened; only the reset is somebody else's job.
        self.assertEqual(len(probe.flashed), 1)
        self.assertEqual(probe.resets, [])

    def test_a_harness_that_writes_to_the_device_is_refused_before_the_flash(self):
        # The unity harness sends its command the instant it starts, and the
        # reset the operator is about to press wipes it -- so the run would
        # spend its whole timeout and then blame the firmware. Refusing early
        # is the only way to make that failure legible, and refusing *before*
        # the download is the only way to make it cheap.
        lines = []
        summary = self.runner(self.harness_case("unity"), echo=lines.append).run()

        self.assertEqual(summary.total, 1)
        result = summary.results[0]
        self.assertEqual(result.status, ERROR)
        self.assertIn("console harness", result.message)
        # Not merely "nothing was downloaded": no probe was ever built. The
        # refusal has to land before the download, and a factory that never ran
        # is the strongest available proof of that.
        self.assertNotIn("tb_manual", self.probes)


#: The manual bench with the one word that changes everything: nothing resets the
#: board, and nothing needs to. The boot is the download's, and the lab says so
#: once instead of every caller vouching for it per run.
AUTO_RESET_BENCH = """
runner:
  name: e2e
  console_poll: 0.005

testbed:
  - id: tb_auto
    platform: sf32lb52-lcd_a128r16
    console:
      port: COM8
    reset:
      method: auto
"""


class EndToEndAutoResetCase(EndToEndCase):
    """A bench whose boot is guaranteed, declared where the wiring is known.

    The difference from the manual bench is one word in the lab's own config,
    and that word used to have to be typed on the command line: a caller
    promised the board was already up. A promise about the wiring belongs to
    whoever wired it, and it is the same promise on every run.
    """

    testbeds_yaml = AUTO_RESET_BENCH
    platform = "sf32lb52-lcd_a128r16"
    response = UNITY_LOG

    def test_nothing_is_said_about_a_reset_nobody_performed(self):
        # `jlink` prints nothing either: the `[reset]` line is addressed to an
        # operator, and there is none to address here.
        lines = []
        summary = self.runner(self.harness_case(), echo=lines.append).run()

        self.assertEqual(summary.passed, 1)
        self.assertEqual(
            [line for line in lines if line.startswith("[reset]")], []
        )

    def test_the_probe_downloads_but_never_resets(self):
        self.runner(self.harness_case()).run()

        probe = self.probes["tb_auto"]
        self.assertEqual(len(probe.flashed), 1)
        self.assertEqual(probe.resets, [])

    def test_a_harness_that_writes_runs_with_nobody_vouching_for_the_boot(self):
        # The point of the mode. This exact run is refused on the manual bench
        # before anything is flashed; here it needs no operator, no
        # confirmation and no flag -- only the lab saying its boot is its own.
        summary = self.runner(self.harness_case("unity")).run()

        result = summary.results[0]
        self.assertEqual(result.status, PASSED, result.message)


class EndToEndConfirmedManualResetCase(EndToEndCase):
    """The same probe-less bench, but with a terminal on the other end.

    Deliberately *not* a subclass of the case above: two of its tests are about
    what happens when nobody can be asked -- one asserts the non-blocking
    prompt, one asserts unity is refused -- and both are meant to be false
    here. Inheriting would have re-run them and then contradicted them.
    """

    testbeds_yaml = MANUAL_RESET_BENCH
    platform = "sf32lb52-lcd_a128r16"
    #: What the faked device answers a `unity_run` with. Set here rather than
    #: inherited -- this class is standalone on purpose (see above), and
    #: without it the unity case below fails for want of a reply.
    response = UNITY_LOG

    def setUp(self):
        super().setUp()
        self.asked = []

    def confirm(self, message: str) -> bool:
        self.asked.append(message)
        return True

    def test_the_operator_is_asked_before_the_harness_speaks(self):
        summary = self.runner(
            self.harness_case(), confirm=self.confirm
        ).run()

        self.assertEqual(summary.passed, 1)
        self.assertEqual(len(self.asked), 1)
        # Names the board -- a lab with several benches needs to know which
        # button to press -- and says what to do before pressing Enter.
        self.assertIn("tb_manual", self.asked[0])
        self.assertIn("RESET", self.asked[0])
        self.assertIn("Enter", self.asked[0])

    def test_declining_the_confirmation_skips_rather_than_fails(self):
        summary = self.runner(self.harness_case(), confirm=lambda _m: False).run()

        result = summary.results[0]
        # Skipped, not failed: the test never ran, and calling that a firmware
        # failure would be a claim nothing supports.
        self.assertEqual(result.status, SKIPPED)
        self.assertIn("not confirmed", result.message)

    def test_unity_is_allowed_once_the_boot_can_be_confirmed(self):
        # The whole reason the gate exists. Without it this is refused before
        # the download; with it, the harness speaks only after the operator has
        # seen the board come up, so nothing resets it out from under the
        # command.
        summary = self.runner(
            self.harness_case("unity"), confirm=self.confirm
        ).run()
        result = summary.results[0]
        self.assertEqual(result.status, PASSED, result.message)
        # And it really went through the gate rather than slipping past it.
        self.assertEqual(len(self.asked), 1)


class TestRunDirectoryPerRun(EndToEndCase):
    """Each run gets its own tree, so a passing run's log survives the next one.

    The bug this pins is not hypothetical: running the same suite twice used to
    write into one directory and overwrite it, so by the time a flake appeared
    the green baseline it had to be compared against was already gone.
    """

    def a_suite(self):
        return self.make_project(
            testcase_yaml="""
            tests:
              - id: hello.console
                harness: console
                harness_config:
                  record: ["Hello world!"]
            """,
            suffix="hello",
        )

    def test_the_second_run_does_not_clobber_the_first(self):
        suite = self.a_suite()
        first = self.runner(suite).run()
        second = self.runner(suite).run()

        self.assertNotEqual(first.out_dir, second.out_dir)
        # Both trees still hold their own run log. That file is the evidence
        # that used to be lost, so its survival on the *first* run is the
        # assertion that matters.
        for summary in (first, second):
            self.assertTrue(
                os.path.isfile(os.path.join(summary.out_dir, "runner.log")),
                summary.out_dir,
            )

    def test_the_run_directory_is_a_stamp_under_the_outdir(self):
        summary = self.runner(self.a_suite()).run()
        self.assertEqual(os.path.dirname(summary.out_dir), os.path.abspath(self.root))
        self.assertRegex(
            os.path.basename(summary.out_dir),
            r"^\d{4}-\d{2}-\d{2}T\d{2}-\d{2}-\d{2}(-\d+)?$",
        )

    def test_the_run_directory_is_announced_before_anything_runs(self):
        # With one tree per run the path is not guessable, and knowing it early
        # is what lets someone tail device.log while the board is still going.
        # First line, so it is on screen before the build output scrolls it away.
        lines = []
        summary = self.runner(self.a_suite(), echo=lines.append).run()
        self.assertEqual(lines[0], f"[outdir] {summary.out_dir}")


class TestRunManifest(EndToEndCase):
    """What is about to run, said before anything runs.

    A run of any size raises one question while it is going -- how many are
    left -- and neither the closing summary nor the per-test lines answer it,
    because both arrive at the end.
    """

    def two_cases(self):
        case = """
        tests:
          - id: hello.console
            harness: console
            harness_config:
              record: ["Hello world!"]
        """
        self.make_project(testcase_yaml=case, suffix="one")
        self.make_project(testcase_yaml=case, suffix="two")
        return os.path.join(self.root, "example")

    def test_the_whole_set_is_named_before_anything_else_happens(self):
        lines = []
        summary = self.runner(self.two_cases(), echo=lines.append).run()

        # Straight after the outdir and before any test has done anything --
        # not merely "somewhere before the first build line".
        self.assertEqual(lines[0], f"[outdir] {summary.out_dir}")
        self.assertIn("test(s) to run", lines[1])

        # The manifest is echoed as one block -- `print` renders it as several
        # lines, a capturing echo sees one string.
        listed = [line for line in "\n".join(lines).splitlines()
                  if line.strip()[:2] in ("1/", "2/")]
        self.assertEqual(len(listed), 2)
        self.assertIn("one/hello.console", listed[0])
        self.assertIn("two/hello.console", listed[1])

    def test_each_test_is_ruled_off_and_its_verdict_lands_immediately(self):
        lines = []
        self.runner(self.two_cases(), echo=lines.append).run()

        rules = [line for line in lines if line.startswith("---")]
        self.assertEqual(len(rules), 2)
        self.assertIn("[1/2] one/hello.console", rules[0])
        self.assertIn("[2/2] two/hello.console", rules[1])

        # The verdict as each finishes, not only in the closing summary: an
        # hour-long run should not withhold every result until the end of it.
        verdicts = [line for line in lines if line.startswith("[PASS ]")]
        self.assertEqual(len(verdicts), 2)

    def test_a_single_test_is_still_numbered(self):
        lines = []
        self.runner(self.harness_case(), echo=lines.append).run()
        rules = [line for line in lines if line.startswith("---")]
        self.assertIn("[1/1]", rules[0])

    def test_the_manifest_accounts_for_the_pair_planning_turns_away(self):
        # The defect as a user hit it: the opening counted only the runs while
        # the closing summary counted the refused pair too -- as a skipped
        # result, which it is -- so a second test appeared at the end that the
        # opening had never mentioned.
        suite = self.make_project(
            testcase_yaml="""
            tests:
              - id: hello.console
                harness: console
                harness_config:
                  record: ["Hello world!"]
              - id: gated.elsewhere
                harness: console
                platform_allow: [qemu_cortex_m33]
                harness_config:
                  record: ["Hello world!"]
            """,
            suffix="mixed",
        )
        lines = []
        summary = self.runner(suite, echo=lines.append).run()

        # One run and one pair turned away, both named in the same block, before
        # anything was built -- and the summary that follows reports both.
        manifest = lines[1]
        self.assertIn("1 test(s) to run", manifest)
        self.assertIn("1 (test, platform) pair(s) turned away:", manifest)
        self.assertIn("gated.elsewhere", manifest)
        self.assertIn("not allowed", manifest)
        self.assertLess(manifest.index("hello.console"),
                        manifest.index("turned away"))
        self.assertEqual(summary.total, 2)
        self.assertEqual(summary.skipped, 1)


class TestBuildLog(EndToEndCase):
    """What the compiler said is filed, not scrolled past.

    Before this, the build streamed to the terminal and nowhere else, which left
    a failed build with nothing to file: the exit code reads the same whether a
    header is missing or a type is wrong, and the answer was in the part that
    had already gone by.
    """

    def test_the_compilers_output_goes_to_a_file_and_off_the_console(self):
        lines = []
        summary = self.runner(
            self.harness_case(),
            build_fn=lambda *_args: "compiling foo.c\nlinking\n",
            echo=lines.append,
            build=True,
        ).run()

        result = summary.results[0]
        self.assertEqual(result.status, PASSED, result.message)
        # Off the terminal -- that is the whole point of filing it -- but the
        # run still says it built something.
        self.assertNotIn("compiling foo.c", "\n".join(lines))
        self.assertEqual(
            [line for line in lines if line.startswith("[build]")],
            [f"[build] {result.id} for {result.platform}"],
        )

        with open(os.path.join(result.out_dir, "build.log"), encoding="utf-8") as handle:
            self.assertEqual(handle.read(), "compiling foo.c\nlinking\n")

    def test_a_failed_build_files_what_the_compiler_said(self):
        # The case that matters most: a failure whose reason is only in the
        # output. The runner has to keep it even though the build threw.
        def failing(*_args):
            raise BuildError(
                "scons failed for sf32lb52-lcd_n16r8 (exit 2); "
                "the compiler's output is in build.log",
                "main.c:3:10: error: 'foo' undeclared\n",
            )

        summary = self.runner(
            self.harness_case(), build_fn=failing, build=True
        ).run()

        result = summary.results[0]
        self.assertEqual(result.status, ERROR)
        self.assertEqual(result.stage, "build")
        # The message has to say where to look, because the console no longer
        # shows it and the exit code never did.
        self.assertIn("build.log", result.message)

        with open(os.path.join(result.out_dir, "build.log"), encoding="utf-8") as handle:
            self.assertIn("undeclared", handle.read())

    def test_no_build_leaves_no_build_log(self):
        # --no-build never invoked a compiler; an empty file would claim one
        # ran and had nothing to say.
        summary = self.runner(self.harness_case(), build=False).run()
        result = summary.results[0]
        self.assertFalse(os.path.exists(os.path.join(result.out_dir, "build.log")))


if __name__ == "__main__":
    unittest.main()
