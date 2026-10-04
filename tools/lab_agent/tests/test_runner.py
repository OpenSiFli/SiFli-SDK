# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the agent's command line, starting with ``--check``.

``--check`` exists so a mis-wired lab is discovered at boot rather than halfway
through a nightly run, which makes it exactly the wrong place to be wrong: a
warning that fires for every healthy lab trains people to ignore the output, and
one that never fires hides the fault it was written for.

The interesting case is a lab that needs no probe at all. The probe checks are
unconditional in spirit but must not be unconditional in code, so these tests
force ``find_jlink`` to fail and assert on which labs still complain.
"""

from __future__ import annotations

import logging
import unittest

from _support import CONFIG_YAML
from _support import LabTestCase

from lab_agent import runner as runner_module
from lab_agent.errors import ConfigError

#: Downloaded over UART and reset by hand: no probe is involved anywhere.
NO_PROBE_YAML = """
runner:
  name: test-runner
  host: 127.0.0.1
  port: 0

testbed:
  - id: tb_manual
    platform: sf32lb52-lcd_a128r16
    console: {port: COM1}
    flash: {method: sftool}
    reset: {method: manual}
"""

#: Manual reset, but the probe is still how images get in.
MANUAL_RESET_ONLY_YAML = """
runner:
  name: test-runner
  host: 127.0.0.1
  port: 0

testbed:
  - id: tb
    platform: b
    console: {port: COM1}
    reset: {method: manual}
"""


class CheckTestCase(LabTestCase):
    def setUp(self) -> None:
        super().setUp()
        self.logger = logging.getLogger("lab_agent.tests.check")
        self.logger.addHandler(logging.NullHandler())

    def checks(self, yaml_text: str):
        """Run ``_run_checks`` with the probe locator forced to fail.

        Forcing the failure is what makes this deterministic: whether J-Link
        happens to be installed on the machine running the tests must not
        decide whether the assertion means anything.
        """
        manager = self.make_manager(yaml_text)
        original = runner_module.find_jlink
        runner_module.find_jlink = lambda *_args, **_kwargs: (_ for _ in ()).throw(
            ConfigError("Cannot find the J-Link commander (forced for this test).")
        )
        try:
            return manager, runner_module._run_checks(manager, self.logger)
        finally:
            runner_module.find_jlink = original


class TestCheckWithoutAProbe(CheckTestCase):
    def test_a_lab_that_needs_no_probe_is_not_told_it_is_missing(self):
        manager, problems = self.checks(NO_PROBE_YAML)
        self.assertEqual([p for p in problems if "J-Link" in p], [])
        # And it certainly did not go looking for one per testbed either.
        self.assertEqual(manager.probes, {})

    def test_a_lab_that_still_flashes_through_the_probe_is_still_told(self):
        # The predicate is "does either half need the probe", not "is the reset
        # manual" -- a manual reset over a probe download is a real setup, and
        # skipping the check for it would hide a genuinely missing probe.
        _manager, problems = self.checks(MANUAL_RESET_ONLY_YAML)
        self.assertTrue([p for p in problems if "J-Link" in p])

    def test_the_default_lab_is_checked_as_before(self):
        manager, problems = self.checks(CONFIG_YAML)
        self.assertTrue([p for p in problems if "J-Link" in p])
        # The probe was built for each testbed, which is the per-testbed half.
        self.assertIn("tb_01", manager.probes)


if __name__ == "__main__":
    unittest.main()
