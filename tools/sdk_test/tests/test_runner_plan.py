# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for planning in ``sdk_test.runner``.

Planning asks whether a platform can host the test at all, from platform
metadata alone. Which *board* of that platform will run it is no longer decided
here -- the Lab Agent allocates from its own configuration when hardware is
requested -- so the tests care about the seam that is left: a test that is
silently not scheduled is worse than one that fails, and that is what the
filtered half of a plan exists to prevent.
"""

from __future__ import annotations

import json
import os
import tempfile
import unittest
from unittest import mock

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from sdk_test.errors import BackendError
from sdk_test.errors import DiscoveryError
from sdk_test.errors import PlatformError
from sdk_test.harness.unity import registered_suites
from sdk_test.model import TestInstance
from sdk_test.report import SKIPPED
from sdk_test.runner import PlannedRun
from sdk_test.runner import RunOptions
from sdk_test.runner import TestRunner

#: A real board of this repository that declares no peripheral at all, and so
#: fails any `depends_on:` -- no fixture needed, the file is in the tree.
EMPTY_CARRIER = "qemu_sf32lb52x"

#: A real board with no feature declaration at all, for the gradual-rollout case.
UNANNOTATED = "sf32lb52-lcd_n16r8"


#: Artifacts go to a scratch directory, not the default ``test-out`` beside the
#: working directory: these tests exercise planning, and a run leaves a log tree
#: behind wherever it is pointed. Held in a module global so the directory is
#: removed when the interpreter exits.
_SCRATCH = tempfile.TemporaryDirectory(prefix="sdk_test_plan_")


def make_runner(**options):
    settings = {
        "testsuite_root": "/suite",
        "build": False,
        "out_dir": _SCRATCH.name,
    }
    settings.update(options)
    return TestRunner(RunOptions(**settings))


def make_instance(test_id="t", **kwargs):
    return TestInstance(
        id=test_id,
        testsuite_root="/suite",
        testcase_dir="/suite/dir",
        tests_key=test_id,
        **kwargs)


class PlannedPlatformsTest(unittest.TestCase):
    def test_an_explicit_platform_is_the_whole_answer(self):
        runner = make_runner(platform="board_a")
        instance = make_instance(integration_platforms=["board_b"])
        self.assertEqual(["board_a"], runner.planned_platforms(instance))

    def test_integration_platforms_is_the_list_it_says_it_is(self):
        # It used to be read as `[0]`, which silently discarded the rest of what
        # the testcase asked for -- and is not what the field means.
        runner = make_runner()
        instance = make_instance(integration_platforms=["a", "b", "c"])
        self.assertEqual(["a", "b", "c"], runner.planned_platforms(instance))

    def test_a_test_that_names_no_platform_leaves_it_to_the_lab(self):
        self.assertEqual([], make_runner().planned_platforms(make_instance()))


class FilterReasonTest(unittest.TestCase):
    def test_the_allow_and_exclude_lists_are_consulted_first(self):
        runner = make_runner()
        instance = make_instance(platform_exclude=["board_a"])
        self.assertIn("not allowed", runner.filter_reason(instance, "board_a"))

    def test_a_test_requiring_nothing_is_never_filtered(self):
        # And so a board nobody has annotated keeps working for it: no
        # feature file is read at all.
        self.assertIsNone(make_runner().filter_reason(make_instance(), UNANNOTATED))

    def test_a_type_requirement_is_unmet_by_a_carrier_with_no_instances(self):
        runner = make_runner()
        instance = make_instance(depends_on=["spi"])
        reason = runner.filter_reason(instance, EMPTY_CARRIER)
        self.assertIn("spi", reason)
        self.assertIn("declares: none", reason)

    def test_a_type_requirement_is_met_by_any_instance(self):
        runner = make_runner()
        instance = make_instance(depends_on=["spi"])
        with mock.patch("sdk_test.runner.load_platform_features",
                        return_value=["spi1", "i2c2"]):
            self.assertIsNone(runner.filter_reason(instance, "board_a"))

    def test_an_instance_requirement_pins_that_instance(self):
        runner = make_runner()
        instance = make_instance(depends_on=["spi2"])
        with mock.patch("sdk_test.runner.load_platform_features",
                        return_value=["spi1"]):
            self.assertIn("spi2", runner.filter_reason(instance, "board_a"))

    def test_a_requirement_needs_a_declaration_to_be_checked_against(self):
        # Silence is not "no peripherals": for a test that asks, it is an
        # omission, and saying so beats guessing.
        runner = make_runner()
        instance = make_instance(depends_on=["spi"])
        with self.assertRaises(PlatformError) as caught:
            runner.filter_reason(instance, UNANNOTATED)
        self.assertIn("features.yaml", str(caught.exception))


class PlanInstanceTest(SdkTestCase):
    def test_every_named_platform_becomes_a_run(self):
        runner = make_runner()
        instance = make_instance(integration_platforms=[EMPTY_CARRIER,
                                                        "qemu_cortex_m33"])
        with mock.patch("sdk_test.runner.find_project_dir", return_value="/p"):
            planned, filtered = runner.plan_instance(instance)
        self.assertEqual(2, len(planned))
        self.assertEqual([EMPTY_CARRIER, "qemu_cortex_m33"],
                         [run.platform for run in planned])
        self.assertEqual([], filtered)

    def test_a_platform_the_test_cannot_use_is_reported_not_dropped(self):
        runner = make_runner()
        instance = make_instance(depends_on=["spi"],
                                 integration_platforms=[EMPTY_CARRIER])
        planned, filtered = runner.plan_instance(instance)
        self.assertEqual([], planned)
        self.assertEqual(1, len(filtered))
        self.assertEqual(EMPTY_CARRIER, filtered[0].platform)
        self.assertIn("spi", filtered[0].reason)
        self.assertEqual("t", filtered[0].id)

    def test_a_run_names_a_platform_and_no_board(self):
        # Which board of the platform answers the run is the agent's business,
        # decided when hardware is requested -- so a plan entry carries no
        # testbed and the SDK side keeps no copy of the lab's inventory.
        runner = make_runner()
        instance = make_instance(integration_platforms=[EMPTY_CARRIER])
        with mock.patch("sdk_test.runner.find_project_dir", return_value="/p"):
            planned, _filtered = runner.plan_instance(instance)
        self.assertEqual(1, len(planned))
        self.assertEqual(EMPTY_CARRIER, planned[0].platform)
        self.assertFalse(hasattr(planned[0], "testbed"))

    def test_a_test_that_names_no_platform_is_refused_with_both_ways_out(self):
        # Nothing on this side can choose for it any more: naming the platform
        # is the user's job, and the case's own declaration is the other way.
        runner = make_runner()
        with self.assertRaises(BackendError) as caught:
            runner.plan_instance(make_instance())
        self.assertIn("integration_platforms", str(caught.exception))
        self.assertIn("--platform", str(caught.exception))


class RunReportsFilteredTest(SdkTestCase):
    """The end of the pipeline: a filtered pair must appear in the summary."""

    def test_a_filtered_test_is_skipped_with_its_reason(self):
        self.write("testcase.yaml", f"""
            tests:
              - id: needs_a_bus
                harness: unity
                integration_platforms: [{EMPTY_CARRIER}]
                depends_on: [spi]
        """)

        summary = make_runner(testsuite_root=self.root).run()

        self.assertEqual(1, summary.total)
        result = summary.results[0]
        self.assertEqual(SKIPPED, result.status)
        self.assertIn("filtered:", result.message)
        self.assertIn("spi", result.message)
        self.assertEqual(EMPTY_CARRIER, result.platform)
        # A filtered test is not a failure: nothing is broken, it just does not
        # belong here.
        self.assertFalse(result.failed)


class RegistryCheckTest(SdkTestCase):
    """The plan refuses a Unity suite the firmware does not have.

    The registry is generated from what the project compiles, so the firmware --
    not the testcase -- is the authority on which suites exist. Asking it at plan
    time is what separates "the suite failed" from "the suite is not in this
    build", which on a serial console look identical.
    """

    BUILD = "build_board_a_hcpu"

    def manifest(self, board, suites, where="src/gen"):
        self.write(f"{self.BUILD}/{where}/unity_suites_gen.json",
                   json.dumps({"board": board, "suites": {name: [] for name in suites}}))
        return os.path.join(self.root, self.BUILD)

    def planned(self, suite="lz4", harness="unity", board="board_a_hcpu"):
        instance = make_instance(harness=harness, harness_config={"suite": suite})
        return PlannedRun(
            instance=instance,
            platform="board_a",
            backend="qemu",
            project_dir=self.root,
            build_dir=os.path.join(self.root, self.BUILD),
            board=board)

    def test_the_suites_of_a_matching_build_are_read(self):
        build_dir = self.manifest("board_a", ["lz4", "cjson"])
        self.assertEqual(["cjson", "lz4"], registered_suites(build_dir, "board_a_hcpu"))

    def test_a_manifest_for_another_board_is_not_an_answer(self):
        # It says nothing about the build being planned, and guessing from it
        # would be worse than not checking.
        build_dir = self.manifest("board_b", ["lz4"])
        self.assertIsNone(registered_suites(build_dir, "board_a_hcpu"))

    def test_a_build_that_has_not_happened_is_not_an_answer(self):
        self.assertIsNone(registered_suites(os.path.join(self.root, "nowhere"),
                                            "board_a_hcpu"))

    def test_a_firmware_with_no_suite_is_an_answer_not_silence(self):
        build_dir = self.manifest("board_a", [])
        self.assertEqual([], registered_suites(build_dir, "board_a_hcpu"))

    def test_a_suite_that_is_missing_is_named_with_what_is_there(self):
        self.manifest("board_a", ["cjson", "lz4"])
        with self.assertRaises(DiscoveryError) as caught:
            make_runner().check_registry(self.planned(suite="circular_buf"))
        message = str(caught.exception)
        self.assertIn("circular_buf", message)
        self.assertIn("cjson, lz4", message)

    def test_a_suite_that_is_there_passes(self):
        self.manifest("board_a", ["lz4"])
        make_runner().check_registry(self.planned(suite="lz4"))

    def test_an_unbuilt_project_is_not_checked(self):
        make_runner().check_registry(self.planned(suite="lz4"))

    def test_another_harness_is_not_checked(self):
        self.manifest("board_a", ["cjson"])
        make_runner().check_registry(self.planned(suite="lz4", harness="console"))


if __name__ == "__main__":
    unittest.main()
