# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for ``sdk_test.platforms`` -- what a platform offers the build.

The rule worth pinning down here is the asymmetry: a test asks for a *type*, a
platform declares *instances*, and one satisfies the other. Getting it the other
way round would tie a test to one board's wiring, which is what the test's own
code already handles better than a testcase file could.
"""

from __future__ import annotations

import os
import unittest

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from sdk_test.errors import PlatformError
from sdk_test.platforms import REPOSITORY_ROOT
from sdk_test.platforms import declared_features
from sdk_test.platforms import features_path
from sdk_test.platforms import jlink_device
from sdk_test.platforms import load_platform_features
from sdk_test.platforms import platform_types
from sdk_test.platforms import split_platform
from sdk_test.platforms import type_of
from sdk_test.platforms import unsatisfied


class SplitPlatformTest(unittest.TestCase):
    def test_no_suffix_means_hcpu(self):
        self.assertEqual(("qemu_sf32lb52x", "hcpu"),
                         split_platform("qemu_sf32lb52x"))

    def test_an_explicit_suffix_is_kept(self):
        self.assertEqual(("qemu_sf32lb52x", "lcpu"),
                         split_platform("qemu_sf32lb52x_lcpu"))

    def test_every_core_suffix_is_recognised(self):
        for suffix in ("_hcpu", "_lcpu", "_acpu"):
            self.assertEqual(("board", suffix[1:]),
                             split_platform("board" + suffix))

    def test_a_board_whose_name_ends_in_a_suffix_lookalike(self):
        # "sf32lb52x" ends in neither suffix, so it is part of the board name.
        self.assertEqual(("sf32lb52x", "hcpu"), split_platform("sf32lb52x"))


class FeaturesPathTest(unittest.TestCase):
    def test_it_sits_beside_the_board_configuration(self):
        self.assertEqual(
            os.path.join(REPOSITORY_ROOT, "customer", "boards",
                         "sf32lb52-lcd_n16r8", "hcpu", "features.yaml"),
            features_path("sf32lb52-lcd_n16r8"))


class TypeOfTest(unittest.TestCase):
    def test_an_instance_names_its_type(self):
        self.assertEqual("spi", type_of("spi1"))
        self.assertEqual("i2c", type_of("i2c12"))

    def test_a_singleton_is_its_own_type(self):
        self.assertEqual("gpio", type_of("gpio"))

    def test_a_bare_type_stays_itself(self):
        self.assertEqual("spi", type_of("spi"))

    def test_platform_types_are_the_set_of_them(self):
        self.assertEqual({"spi", "i2c", "gpio"},
                         platform_types(["spi1", "spi2", "i2c1", "gpio"]))


class UnsatisfiedTest(unittest.TestCase):
    def test_a_type_is_met_by_any_instance_of_it(self):
        self.assertEqual([], unsatisfied(["spi1", "spi2"], ["spi"]))

    def test_a_type_is_not_met_by_another_type(self):
        self.assertEqual(["spi"], unsatisfied(["i2c1"], ["spi"]))

    def test_an_instance_requirement_pins_that_instance(self):
        self.assertEqual(["spi2"], unsatisfied(["spi1"], ["spi2"]))
        self.assertEqual([], unsatisfied(["spi1", "spi2"], ["spi1"]))

    def test_a_singleton_requirement_matches_itself(self):
        self.assertEqual([], unsatisfied(["gpio"], ["gpio"]))
        self.assertEqual(["gpio"], unsatisfied(["spi1"], ["gpio"]))

    def test_nothing_is_required_of_an_empty_platform(self):
        self.assertEqual([], unsatisfied([], []))

    def test_missing_names_keep_the_order_they_were_asked_in(self):
        self.assertEqual(["i2c", "spi"],
                         unsatisfied([], ["i2c", "spi"]))


class LoadTest(SdkTestCase):
    def declare(self, platform, body):
        path = features_path(platform, root=self.root)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        os.makedirs(os.path.join(self.root, "customer", "boards"), exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(body)
        return path

    def test_it_reads_the_instances_in_order(self):
        self.declare("board_a", "features:\n  - spi2\n  - spi1\n")
        self.assertEqual(["spi2", "spi1"],
                         load_platform_features("board_a", root=self.root))

    def test_an_empty_list_is_a_declaration(self):
        self.declare("board_a", "features: []\n")
        self.assertEqual([], load_platform_features("board_a", root=self.root))

    def test_an_empty_file_is_an_empty_declaration(self):
        self.declare("board_a", "")
        self.assertEqual([], load_platform_features("board_a", root=self.root))

    def test_a_missing_file_names_the_path_to_create(self):
        with self.assertRaises(PlatformError) as caught:
            load_platform_features("board_a", root=self.root)
        self.assertIn("features.yaml", str(caught.exception))

    def test_an_unknown_key_is_refused_rather_than_ignored(self):
        self.declare("board_a", "features: [spi1]\nbogus: [spi]\n")
        with self.assertRaises(PlatformError) as caught:
            load_platform_features("board_a", root=self.root)
        self.assertIn("bogus", str(caught.exception))

    def test_a_non_list_is_refused(self):
        self.declare("board_a", "features: spi1\n")
        with self.assertRaises(PlatformError):
            load_platform_features("board_a", root=self.root)

    def test_a_non_string_entry_is_refused(self):
        self.declare("board_a", "features:\n  - 1\n")
        with self.assertRaises(PlatformError):
            load_platform_features("board_a", root=self.root)

    def test_a_repeat_is_refused(self):
        self.declare("board_a", "features: [spi1, spi1]\n")
        with self.assertRaises(PlatformError):
            load_platform_features("board_a", root=self.root)

    def test_declaring_a_type_an_instance_already_provides_is_refused(self):
        # The redundancy that drifts: `spi` can only ever mean "some spiN".
        self.declare("board_a", "features: [spi1, spi]\n")
        with self.assertRaises(PlatformError) as caught:
            load_platform_features("board_a", root=self.root)
        self.assertIn("spi", str(caught.exception))

    def test_a_singleton_alongside_instances_of_another_type_is_fine(self):
        self.declare("board_a", "features: [spi1, gpio]\n")
        self.assertEqual(["spi1", "gpio"],
                         load_platform_features("board_a", root=self.root))


class DeclaredFeaturesTest(SdkTestCase):
    """A missing file is silence; an empty one is a claim.

    The Lab Agent's wiring check is the caller, and the distinction is the whole
    point of it there: a board nobody has annotated must not be reported as
    contradicting every binding that names a bus, while an empty declaration is
    a claim a binding can disagree with.
    """

    def declare(self, platform, body):
        path = features_path(platform, root=self.root)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(body)

    def test_a_missing_file_is_not_a_declaration(self):
        self.assertIsNone(declared_features("board_a", root=self.root))

    def test_an_empty_declaration_is_one(self):
        self.declare("board_a", "features: []\n")
        self.assertEqual([], declared_features("board_a", root=self.root))


class QemuCarrierTest(unittest.TestCase):
    """The carriers in this repository, as they actually are."""

    BOARD_NAMES = ("qemu_cortex_m33", "qemu_sf32lb52x", "qemu_sf32lb55x",
                   "qemu_sf32lb56x", "qemu_sf32lb57x", "qemu_sf32lb58x")

    def test_every_one_declares_that_it_has_no_peripherals(self):
        # Not a placeholder: saying "nothing" is what makes `depends_on: [spi]`
        # filter a test out instead of building and booting it to find out.
        for board in self.BOARD_NAMES:
            with self.subTest(board=board):
                self.assertEqual([], load_platform_features(board))

    def test_a_bus_requirement_is_therefore_unmet(self):
        for board in self.BOARD_NAMES:
            with self.subTest(board=board):
                self.assertEqual(
                    ["spi"],
                    unsatisfied(load_platform_features(board), ["spi"]))


class JLinkDeviceTest(SdkTestCase):
    """The probe's device name, read from the board's own ``rtconfig.py``.

    It lives beside the feature declaration because it is the same kind of fact:
    a property of the board, in the board's own directory. Both sides read it --
    the SDK side to put it in the firmware package, the agent as a fallback.
    """

    def test_the_device_is_read_from_the_boards_own_file(self):
        self.write(
            "customer/boards/my-board/hcpu/rtconfig.py",
            """
            import os
            TARGET_NAME = 'bf0_ap'
            JLINK_DEVICE = 'SF32LB52X_NOR'
            CORE = "HCPU"
            """,
        )
        self.assertEqual(jlink_device("my-board", self.root), "SF32LB52X_NOR")

    def test_a_core_suffix_picks_that_core(self):
        self.write(
            "customer/boards/my-board/lcpu/rtconfig.py", "JLINK_DEVICE = 'L'\n"
        )
        self.write(
            "customer/boards/my-board/hcpu/rtconfig.py", "JLINK_DEVICE = 'H'\n"
        )
        self.assertEqual(jlink_device("my-board", self.root), "H")
        self.assertEqual(jlink_device("my-board_lcpu", self.root), "L")

    def test_an_empty_device_is_not_offered_as_a_name(self):
        # Boards that are not real hardware (pc, dummy) declare an empty value,
        # and an empty name is worse than none: it would be handed to the probe.
        self.write("customer/boards/pc/hcpu/rtconfig.py", "JLINK_DEVICE = ''\n")
        self.assertIsNone(jlink_device("pc", self.root))

    def test_a_board_that_declares_nothing_yields_nothing(self):
        self.assertIsNone(jlink_device("no-such-board", self.root))

    def test_the_real_board_declares_one(self):
        # Pins the reading against the tree it actually runs in, so a change to
        # how this is spelled is noticed rather than silently yielding nothing.
        self.assertEqual(jlink_device("sf32lb52-lcd_a128r16"), "SF32LB52X_NAND")


if __name__ == "__main__":
    unittest.main()
