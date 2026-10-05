# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the ``pytest`` harness driver.

Most of this is pure logic (where the suite lives, how the command is built, how
the environment is passed). The last class runs a real pytest process against a
real Lab Agent with a faked console -- end to end, but without hardware -- and is
skipped when pytest is not installed, so the framework's own self-tests stay
runnable on a machine that only ever uses the ``console`` and ``unity``
harnesses.
"""

from __future__ import annotations

import os
import sys
import threading
import unittest

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from lab_agent.config import parse_runner_config
from lab_agent.server import LabAgentServer
from lab_agent.testbed import LabManager
from sdk_test.errors import HarnessError
from sdk_test.harness import RunContext
from sdk_test.harness import run as run_harness
from sdk_test.harness.pytest import ENV_AGENT_URL
from sdk_test.harness.pytest import ENV_PLATFORM
from sdk_test.harness.pytest import ENV_TESTBED_ID
from sdk_test.harness.pytest import ENV_TESTCASE_DIR
from sdk_test.harness.pytest import _summarise
from sdk_test.harness.pytest import build_command
from sdk_test.harness.pytest import ensure_available
from sdk_test.harness.pytest import plugin_env
from sdk_test.harness.pytest import resolve_pytest_root
from sdk_test.model import TestInstance

try:
    import pytest as _pytest  # noqa: F401

    HAVE_PYTEST = True
except ImportError:  # pragma: no cover - depends on the machine
    HAVE_PYTEST = False


def make_instance(testcase_dir: str, harness_config=None, **kwargs) -> TestInstance:
    return TestInstance(
        id="s/pytest_case",
        testsuite_root=testcase_dir,
        testcase_dir=testcase_dir,
        tests_key="pytest_case",
        harness="pytest",
        harness_config=dict(harness_config or {}),
        **kwargs,
    )


class TestPytestRoot(SdkTestCase):
    def test_the_default_root_is_the_testcase_directory(self):
        instance = make_instance(self.root)
        self.assertEqual(resolve_pytest_root(instance), os.path.normpath(self.root))

    def test_a_relative_root_is_resolved_against_the_testcase_directory(self):
        instance = make_instance(self.root, {"pytest_root": "cases"})
        self.assertEqual(
            resolve_pytest_root(instance), os.path.join(self.root, "cases")
        )

    def test_an_absolute_root_is_taken_as_is(self):
        absolute = os.path.join(self.root, "elsewhere")
        instance = make_instance(self.root, {"pytest_root": absolute})
        self.assertEqual(resolve_pytest_root(instance), os.path.normpath(absolute))


class TestCommand(SdkTestCase):
    def test_the_plugin_is_loaded_explicitly(self):
        # So a test directory needs no conftest.py to get `dut` and `console`.
        command = build_command(make_instance(self.root), self.root, "python")
        self.assertIn("-p", command)
        self.assertIn("sdk_test.pytest_plugin", command)

    def test_the_cache_provider_is_disabled(self):
        # Otherwise every run drops a .pytest_cache into the source tree.
        command = build_command(make_instance(self.root), self.root, "python")
        self.assertIn("no:cacheprovider", command)

    def test_configured_arguments_are_appended(self):
        instance = make_instance(self.root, {"pytest_args": ["-x", "-k", "lz4"]})
        command = build_command(instance, self.root, "python")
        self.assertEqual(command[-3:], ["-x", "-k", "lz4"])

    def test_the_interpreter_and_root_are_used(self):
        command = build_command(make_instance(self.root), "/some/root", "/py")
        self.assertEqual(command[:3], ["/py", "-m", "pytest"])
        self.assertEqual(command[3], "/some/root")


class TestPluginEnv(SdkTestCase):
    def test_the_dut_coordinates_are_exported(self):
        instance = make_instance(self.root)
        context = RunContext(
            agent_url="http://127.0.0.1:8760",
            testbed_id="tb_01",
            build_dir="/build",
            python_exe="/py",
        )
        env = plugin_env(instance, context)

        self.assertEqual(env[ENV_AGENT_URL], "http://127.0.0.1:8760")
        self.assertEqual(env[ENV_TESTBED_ID], "tb_01")
        self.assertEqual(env[ENV_TESTCASE_DIR], self.root)
        # So that `-p sdk_test.pytest_plugin` resolves in the subprocess.
        self.assertIn("tools", env["PYTHONPATH"])

    def test_the_platform_comes_from_the_instance_or_the_context(self):
        context = RunContext(platform="board_b")
        self.assertEqual(plugin_env(make_instance(self.root), context)[ENV_PLATFORM], "board_b")

        instance = make_instance(self.root)
        instance.platform = "board_a"
        self.assertEqual(plugin_env(instance, context)[ENV_PLATFORM], "board_a")

    def test_an_absent_context_yields_no_coordinates(self):
        env = plugin_env(make_instance(self.root), None)
        self.assertNotIn(ENV_AGENT_URL, env)
        self.assertNotIn(ENV_TESTBED_ID, env)


class TestSummarise(SdkTestCase):
    def test_the_final_tally_is_extracted(self):
        output = (
            "=== test session starts ===\n"
            "collected 3 items\n"
            "=== 2 passed, 1 failed in 0.14s ===\n"
        )
        summary = _summarise(output, 1)
        self.assertIn("2 passed", summary)
        self.assertIn("1 failed", summary)

    def test_output_without_a_tally_falls_back_to_the_exit_code(self):
        self.assertIn("7", _summarise("nothing useful here", 7))


class TestFailureModes(SdkTestCase):
    def test_a_missing_suite_directory_is_reported(self):
        instance = make_instance(self.root, {"pytest_root": "absent"})
        with self.assertRaises(HarnessError) as caught:
            run_harness(instance, None, 5)
        self.assertIn("not a directory", str(caught.exception))

    def test_a_directory_without_python_tests_is_reported_with_the_alternative(self):
        self.write("suite/README.md", "nothing here")
        instance = make_instance(self.root, {"pytest_root": "suite"})
        with self.assertRaises(HarnessError) as caught:
            run_harness(instance, None, 5)
        message = str(caught.exception)
        self.assertIn("test_*.py", message)
        # Point the user back at the harness that needs no Python at all.
        self.assertIn('harness "console"', message)

    def test_a_missing_interpreter_is_reported(self):
        with self.assertRaises(HarnessError) as caught:
            ensure_available(os.path.join(self.root, "no-such-python"))
        self.assertIn("Cannot run", str(caught.exception))

    def test_an_unimplemented_harness_still_refuses(self):
        instance = make_instance(self.root)
        instance.harness = "shell"
        with self.assertRaises(HarnessError) as caught:
            run_harness(instance, None, 5)
        self.assertIn("not implemented", str(caught.exception))


class FakeTransport:
    def __init__(self, boot: str = "", response: str = "", chunk: int = 1) -> None:
        self._stream = boot
        self.response = response
        #: How much one read hands over. A character by default, which is the
        #: harshest realistic chunking. A test that needs to prove something
        #: about a single read carrying several expectations' worth of text
        #: raises it, because a real port hands over whatever is buffered.
        self.chunk = max(1, chunk)
        self.written = []
        self.opened = 0
        self.closed = 0

    def open(self) -> None:
        self.opened += 1

    def close(self) -> None:
        self.closed += 1

    def read(self, timeout: float = 0.05) -> str:
        if not self._stream:
            return ""
        piece, self._stream = self._stream[:self.chunk], self._stream[self.chunk:]
        return piece

    def write(self, text: str) -> None:
        self.written.append(text)
        if "unity_run" in text:
            self._stream += self.response

    @property
    def sent(self) -> str:
        return "".join(self.written)


class PytestAgainstFakeAgentMixin:
    """The full pytest path: a real subprocess, a real agent, a faked device.

    Driven through :class:`~sdk_test.runner.TestRunner` rather than by calling
    the harness directly. That matters: the console is opened by the backend, so
    a direct call would test a situation that never occurs in practice and would
    pass for the wrong reason.

    Held apart from the test class below so that a second bench -- one where
    nothing resets the board -- can describe itself with a different
    :attr:`LAB_YAML` without inheriting and re-running every case.
    """

    BOOT = " | / - RT - Thread\r\nmsh />"
    UNITY = "\r\ntest/a.c:1:a.roundtrip:PASS\r\n\r\n1 Tests 0 Failures 0 Ignored\r\nOK\r\n"

    #: The lab the agent serves. The platform below is the one the run names,
    #: as a user's `-p` would name it; the agent picks the board that answers it.
    LAB_YAML = """
    runner: {name: pytest-e2e, console_poll: 0.005}
    testbed:
      - id: tb_01
        platform: sf32lb52-lcd_n16r8
        console: {port: COM13}
    """

    #: The platform the suite is built for. Allocation is the agent's job, so
    #: this is all the SDK side says.
    PLATFORM = "sf32lb52-lcd_n16r8"

    def setUp(self):
        super().setUp()
        self.transports = {}
        import yaml

        manager = LabManager(
            parse_runner_config(yaml.safe_load(self.LAB_YAML), source="<test>"),
            sdk_root=self.root,
            console_factory=self._console,
            jlink_factory=lambda testbed, settings: _FakeProbe(),
        )
        manager.config.settings.host = "127.0.0.1"
        manager.config.settings.port = 0
        self.manager = manager
        self.server = LabAgentServer(manager)
        self.server.start_background()
        self.addCleanup(self.server.stop)

    #: How much the faked console hands over per read; see FakeTransport.
    chunk = 1

    def _console(self, testbed):
        transport = FakeTransport(
            boot=self.BOOT, response=self.UNITY, chunk=self.chunk
        )
        self.transports[testbed.id] = transport
        return transport

    def make_case(self, suite_body: str) -> str:
        """A testcase directory: testcase.yaml, its pytest suite, and a project."""
        self.write(
            "example/ptcase/testcase.yaml",
            """
            tests:
              - id: ptcase.drive
                harness: pytest
                timeout: 60
                harness_config:
                  pytest_root: .
            """,
        )
        self.write("example/ptcase/project/SConstruct", "# project\n")
        self.write("example/ptcase/project/proj.conf", "CONFIG_X=y\n")
        self.write("example/ptcase/test_device.py", suite_body)
        return os.path.join(self.root, "example", "ptcase")

    def run_case(self, suite_body: str, **overrides):
        from sdk_test.runner import RunOptions
        from sdk_test.runner import TestRunner

        options = RunOptions(
            testsuite_root=self.make_case(suite_body),
            platform=self.PLATFORM,
            agent_url=self.server.url,
            build=False,
            python_exe=sys.executable,
            timeout=60,
            # Artifacts under the test's own temp root; the default would be a
            # test-out/ beside the working directory.
            out_dir=self.root,
        )
        for key, value in overrides.items():
            setattr(options, key, value)
        return TestRunner(options).run()


@unittest.skipUnless(HAVE_PYTEST, "pytest is not installed in this interpreter")
class TestRealPytestAgainstFakeAgent(PytestAgainstFakeAgentMixin, SdkTestCase):
    """A probe-reset bench: the ordinary case."""

    def test_a_suite_that_drives_the_dut_passes(self):
        summary = self.run_case(
            """
            def test_device_boots(dut):
                assert dut.expect("msh />", timeout=5)
            """
        )
        result = summary.results[0]
        self.assertEqual(result.status, "passed", result.message)
        self.assertIn("1 passed", result.log)

    def test_a_failing_assertion_fails_the_harness(self):
        summary = self.run_case(
            """
            def test_device_says_something_else(dut):
                assert dut.expect("never printed", timeout=1)
            """
        )
        result = summary.results[0]
        self.assertEqual(result.status, "failed", result.message)
        # The device transcript must reach the log, or the failure cannot be
        # diagnosed from CI output alone.
        self.assertIn("never printed", result.log)
        self.assertIn("msh />", result.log)

    def test_a_suite_can_collect_unity_output_through_the_fixture(self):
        summary = self.run_case(
            """
            def test_unity_suite(dut):
                result = dut.expect_unity_test_output(timeout=5)
                assert result.passed, result.detail
                assert result.tests == 1
            """
        )
        result = summary.results[0]
        self.assertEqual(result.status, "passed", result.message)
        # The plugin asked the device to run Unity over the same console.
        self.assertIn("unity_run", self.transports["tb_01"].sent)

    def test_the_testbed_is_released_after_a_pytest_run(self):
        self.run_case(
            """
            def test_device_boots(dut):
                assert dut.expect("msh />", timeout=5)
            """
        )
        self.assertFalse(self.manager.get("tb_01").busy)
        self.assertEqual(self.transports["tb_01"].closed, 1)

    def test_a_suite_run_outside_the_framework_skips_rather_than_hangs(self):
        # A developer running `pytest` by hand gets no testbed coordinates. The
        # `dut` fixture must skip with an explanation rather than hang or fail:
        # a suite that cannot run is not a suite that failed.
        self.write("bare/test_device.py", "def test_device(dut):\n    raise AssertionError('unreachable')\n")
        instance = make_instance(os.path.join(self.root, "bare"))
        outcome = run_harness(
            instance, None, timeout=30, context=RunContext(python_exe=sys.executable)
        )

        self.assertTrue(outcome.passed, outcome.detail)
        self.assertIn("skipped", outcome.log.lower())
        # And it must say *why*, not just skip silently.
        self.assertIn("SIFLI_TEST_AGENT_URL", outcome.log)


@unittest.skipUnless(HAVE_PYTEST, "pytest is not installed in this interpreter")
class TestPytestOnAManuallyBootedBoard(PytestAgainstFakeAgentMixin, SdkTestCase):
    """A pytest suite is fine on a board nothing resets.

    The unity harness is not -- it sends its run command before it reads
    anything, so the boot that follows wipes it. A pytest suite picks its own
    order, and the one in this tree reads the boot banner before it writes. So
    the refusal has to name unity rather than cover everything that is not the
    console harness; the wider rule took away a combination that works, which
    is how this test came to exist.
    """

    LAB_YAML = """
    runner: {name: pytest-e2e, console_poll: 0.005}
    testbed:
      - id: tb_01
        platform: sf32lb52-lcd_n16r8
        console: {port: COM13}
        reset: {method: manual}
    """

    def test_the_run_is_not_refused(self):
        # The assertion is the absence of a refusal: before the rule was
        # narrowed this came back ERROR at the `open` stage, quoting a unity
        # limitation at a suite that had not done anything wrong.
        summary = self.run_case(
            """
            def test_device_boots(dut):
                assert dut.expect("msh />", timeout=5)
            """
        )
        result = summary.results[0]
        self.assertEqual(result.status, "passed", result.message)
        self.assertEqual(result.stage, "run")


@unittest.skipUnless(HAVE_PYTEST, "pytest is not installed in this interpreter")
class TestExpectOverACoarseConsole(PytestAgainstFakeAgentMixin, SdkTestCase):
    """One read can carry more than one expectation's worth of text.

    The mixin's fake drips a character at a time, which is the harshest
    realistic chunking but hides this: a board that prints its banner and its
    prompt back to back hands both over in one go. An expectation that searched
    a fresh buffer per call dropped the second half, and the next one then
    waited out its whole timeout for bytes that had already arrived -- reported
    as a silent device, which points nowhere near the cause.
    """

    chunk = 4096

    def test_a_later_expectation_finds_text_from_the_same_read(self):
        summary = self.run_case(
            """
            def test_both(dut):
                assert dut.expect("Thread", timeout=5)
                assert dut.expect("msh />", timeout=5)
            """
        )
        result = summary.results[0]
        self.assertEqual(result.status, "passed", result.message)
        self.assertIn("1 passed", result.log)


@unittest.skipUnless(HAVE_PYTEST, "pytest is not installed in this interpreter")
class TestSessionScopedConsole(PytestAgainstFakeAgentMixin, SdkTestCase):
    """An expectation covers output from its own test onward.

    Without this the cursor rewinds between tests, so any expectation can be
    satisfied by something the device said before the test began -- a pass that
    says nothing about the code the test is meant to exercise.
    """

    def test_a_later_test_cannot_match_an_earlier_tests_output(self):
        summary = self.run_case(
            """
            def test_first(dut):
                assert dut.expect("msh />", timeout=5)

            def test_second(dut):
                assert dut.expect("msh />", timeout=1), "stale output matched"
            """
        )
        # The boot prompt is real, and the first test is entitled to it. The
        # second is not: it is behind the cursor by then, and the device says
        # nothing new because this suite never sends anything.
        log = summary.results[0].log
        self.assertIn("1 passed", log)
        self.assertIn("1 failed", log)
        self.assertIn("stale output matched", log)


class _FakeProbe:
    def reset(self, extra_script=None):
        return "ok"

    def flash(self, build_dir):
        return "ok"


if __name__ == "__main__":
    unittest.main()
