# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for ``sdk_test.fixtures`` -- fixture parsing and testbed matching.

Matching is the part of the framework a human never sees when it works, so the
tests care as much about :func:`explain_no_match` as about the match itself: a
test that is silently not scheduled is the worst outcome this layer can produce.
"""

from __future__ import annotations

import unittest

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from sdk_test.errors import BindingError
from sdk_test.errors import DiscoveryError
from sdk_test.fixtures import explain_no_match
from sdk_test.fixtures import implicit_requirements
from sdk_test.fixtures import instance_fixtures_from_config
from sdk_test.fixtures import parse_fixture_ref
from sdk_test.fixtures import parse_fixture_refs
from sdk_test.fixtures import select_testbeds
from sdk_test.fixtures import wiring_problems
from sdk_test.model import Connection
from sdk_test.model import FixtureRef
from sdk_test.model import TestInstance
from sdk_test.model import TestBed


def make_testbed(testbed_id, platform="board_a", fixtures=()):
    return TestBed(
        id=testbed_id,
        platform=platform,
        connections=[Connection(fixture=parse_fixture_ref(spec)) for spec in fixtures],
    )


def make_instance(test_id="t", fixtures=(), **kwargs):
    return TestInstance(
        id=test_id,
        testsuite_root="/suite",
        testcase_dir="/suite/dir",
        tests_key=test_id,
        fixtures=[parse_fixture_ref(spec) for spec in fixtures],
        **kwargs,
    )


class TestParseFixtureRef(SdkTestCase):
    def test_a_bare_name_accepts_any_wiring(self):
        ref = parse_fixture_ref("spi_nor_flash")
        self.assertEqual((ref.name, ref.param), ("spi_nor_flash", None))
        self.assertEqual(ref.spec, "spi_nor_flash")

    def test_a_colon_pins_the_instance(self):
        ref = parse_fixture_ref("spi_nor_flash:spi2")
        self.assertEqual((ref.name, ref.param), ("spi_nor_flash", "spi2"))
        self.assertEqual(ref.spec, "spi_nor_flash:spi2")

    def test_the_mapping_form_is_accepted(self):
        ref = parse_fixture_ref({"name": "sensor", "param": "i2c1"})
        self.assertEqual(ref.spec, "sensor:i2c1")

    def test_the_binding_file_form_is_accepted(self):
        ref = parse_fixture_ref({"fixture": "spi_nor_flash:spi2"})
        self.assertEqual(ref.spec, "spi_nor_flash:spi2")

    def test_the_binding_file_form_allows_a_param_override(self):
        ref = parse_fixture_ref({"fixture": "sensor", "param": "i2c0"})
        self.assertEqual(ref.spec, "sensor:i2c0")

    def test_a_numeric_param_is_stringified(self):
        ref = parse_fixture_ref({"name": "mux", "param": 3})
        self.assertEqual(ref.param, "3")

    def test_surrounding_whitespace_is_trimmed(self):
        ref = parse_fixture_ref("  sensor : i2c1  ")
        self.assertEqual((ref.name, ref.param), ("sensor", "i2c1"))

    def test_an_empty_string_is_rejected(self):
        with self.assertRaises(BindingError):
            parse_fixture_ref("   ")

    def test_a_reference_without_a_name_is_rejected(self):
        with self.assertRaises(BindingError):
            parse_fixture_ref(":spi2")

    def test_a_mapping_without_a_name_is_rejected(self):
        with self.assertRaises(BindingError) as caught:
            parse_fixture_ref({"param": "spi2"})
        self.assertIn("needs a name", str(caught.exception))

    def test_an_unsupported_type_is_rejected(self):
        with self.assertRaises(BindingError):
            parse_fixture_ref(42)

    def test_an_already_parsed_ref_passes_through(self):
        original = FixtureRef("sensor", "i2c1")
        self.assertIs(parse_fixture_ref(original), original)


class TestParseFixtureRefs(SdkTestCase):
    def test_a_scalar_becomes_a_single_element_list(self):
        self.assertEqual(len(parse_fixture_refs("sensor")), 1)

    def test_a_list_is_parsed_elementwise(self):
        refs = parse_fixture_refs(["a", "b:x"])
        self.assertEqual([ref.spec for ref in refs], ["a", "b:x"])

    def test_none_yields_nothing(self):
        self.assertEqual(parse_fixture_refs(None), [])


class TestAccepts(SdkTestCase):
    def test_a_bare_requirement_accepts_any_param(self):
        self.assertTrue(FixtureRef("flash").accepts(FixtureRef("flash", "spi1")))
        self.assertTrue(FixtureRef("flash").accepts(FixtureRef("flash")))

    def test_a_pinned_requirement_needs_the_same_param(self):
        self.assertTrue(FixtureRef("flash", "spi1").accepts(FixtureRef("flash", "spi1")))
        self.assertFalse(FixtureRef("flash", "spi1").accepts(FixtureRef("flash", "spi2")))

    def test_a_pinned_requirement_is_not_satisfied_by_an_unpinned_offer(self):
        # A testbed that does not say *which* bus is not evidence that it is the
        # bus the test asked for.
        self.assertFalse(FixtureRef("flash", "spi1").accepts(FixtureRef("flash")))

    def test_different_names_never_match(self):
        self.assertFalse(FixtureRef("flash").accepts(FixtureRef("sensor")))


class TestSelectTestbeds(SdkTestCase):
    def test_the_same_fixture_on_several_testbeds_makes_them_interchangeable(self):
        testbeds = [make_testbed("tb_01", fixtures=["flash"]), make_testbed("tb_02", fixtures=["flash"])]
        selected = select_testbeds(make_instance(fixtures=["flash"]), testbeds)
        self.assertEqual([t.id for t in selected], ["tb_01", "tb_02"])

    def test_binding_order_is_preserved_for_deterministic_allocation(self):
        testbeds = [make_testbed("tb_02", fixtures=["flash"]), make_testbed("tb_01", fixtures=["flash"])]
        selected = select_testbeds(make_instance(fixtures=["flash"]), testbeds)
        self.assertEqual([t.id for t in selected], ["tb_02", "tb_01"])

    def test_a_missing_fixture_excludes_the_testbed(self):
        testbeds = [make_testbed("tb_01", fixtures=["sensor"])]
        self.assertEqual(select_testbeds(make_instance(fixtures=["flash"]), testbeds), [])

    def test_a_test_with_no_fixtures_matches_everything(self):
        testbeds = [make_testbed("tb_01"), make_testbed("tb_02")]
        self.assertEqual(len(select_testbeds(make_instance(), testbeds)), 2)

    def test_an_explicit_platform_filters_to_that_platform(self):
        testbeds = [make_testbed("tb_01", platform="board_a"), make_testbed("tb_02", platform="board_b")]
        selected = select_testbeds(make_instance(), testbeds, platform="board_b")
        self.assertEqual([t.id for t in selected], ["tb_02"])

    def test_platform_allow_restricts_matching(self):
        testbeds = [make_testbed("tb_01", platform="board_a"), make_testbed("tb_02", platform="board_b")]
        instance = make_instance(platform_allow=["board_b"])
        self.assertEqual([t.id for t in select_testbeds(instance, testbeds)], ["tb_02"])

    def test_platform_exclude_wins_over_allow(self):
        testbeds = [make_testbed("tb_01", platform="board_a")]
        instance = make_instance(platform_allow=["board_a"], platform_exclude=["board_a"])
        self.assertEqual(select_testbeds(instance, testbeds), [])

    def test_depends_on_is_not_a_testbed_question(self):
        # What a board can do is settled against the platform -- before a testbed
        # is ever chosen (see sdk_test.platforms). A testbed neither knows nor
        # answers for it, so a `depends_on:` must not change which testbeds fit.
        testbeds = [make_testbed("tb_01"), make_testbed("tb_02", platform="board_b")]
        instance = make_instance(depends_on=["i2s"])
        self.assertEqual([t.id for t in select_testbeds(instance, testbeds)],
                         ["tb_01", "tb_02"])

    def test_every_fixture_must_be_offered_simultaneously(self):
        testbeds = [make_testbed("tb_01", fixtures=["flash"])]
        instance = make_instance(fixtures=["flash", "sensor"])
        self.assertEqual(select_testbeds(instance, testbeds), [])

    def test_allows_platform_accepts_anything_when_unconstrained(self):
        instance = make_instance()
        self.assertTrue(instance.allows_platform("any-board-at-all"))


class TestExplainNoMatch(SdkTestCase):
    def test_an_empty_binding_is_named_as_the_cause(self):
        self.assertIn("no testbeds", explain_no_match(make_instance(), []))

    def test_a_pinned_fixture_mismatch_names_both_sides(self):
        testbeds = [make_testbed("tb_01", fixtures=["spi_nor_flash_py25q128:spi1"])]
        instance = make_instance(fixtures=["spi_nor_flash_py25q128:spi2"])
        message = explain_no_match(instance, testbeds)
        self.assertIn("spi_nor_flash_py25q128:spi2", message)
        self.assertIn("spi_nor_flash_py25q128:spi1", message)

    def test_a_testbed_offering_nothing_says_so(self):
        message = explain_no_match(make_instance(fixtures=["flash"]), [make_testbed("tb_01")])
        self.assertIn("offers: none", message)

    def test_a_platform_rejection_explains_the_allow_and_exclude_lists(self):
        testbeds = [make_testbed("tb_01", platform="board_a")]
        instance = make_instance(platform_allow=["board_b"])
        message = explain_no_match(instance, testbeds)
        self.assertIn("board_a", message)
        self.assertIn("board_b", message)

    def test_depends_on_never_appears_in_the_reason(self):
        # `depends_on` is the platform's question, answered from the board's own
        # declaration before a bench is ever chosen. A reason that quoted it
        # here would be answering a question this layer never asked, and would
        # read as a fixture mismatch -- which it is not.
        testbeds = [make_testbed("tb_01", fixtures=["flash:spi1"])]
        message = explain_no_match(make_instance(depends_on=["i2s"]), testbeds)
        self.assertNotIn("capabilit", message.lower())


class TestWiringAgainstFeatures(SdkTestCase):
    """The one place a binding and a board's declaration meet."""

    def test_a_fixture_parameter_implies_the_type_of_that_bus(self):
        instance = make_instance(fixtures=["flash:spi1", "sensor"])
        self.assertEqual(["spi"], implicit_requirements(instance))

    def test_an_unparameterised_fixture_implies_nothing(self):
        self.assertEqual([],
                         implicit_requirements(make_instance(fixtures=["flash"])))

    def test_a_connection_the_platform_does_not_declare_is_a_problem(self):
        problems = wiring_problems([make_testbed("tb_01", fixtures=["flash:spi1"])], [])
        self.assertEqual(1, len(problems))
        self.assertIn("spi", problems[0])
        self.assertIn("spi1", problems[0])

    def test_an_instance_of_the_type_is_enough(self):
        self.assertEqual(
            [],
            wiring_problems([make_testbed("tb_01", fixtures=["flash:spi1"])],
                            ["spi2"]))

    def test_an_unparameterised_connection_is_never_a_problem(self):
        self.assertEqual(
            [],
            wiring_problems([make_testbed("tb_01", fixtures=["flash"])], []))

    def test_an_instrument_console_is_not_a_bus_claim(self):
        # `ble_peer:peer1` names the instrument, not an instance the board has
        # to declare. Only bus-wired fixtures carry that obligation.
        testbed = TestBed(
            id="tb_01",
            platform="board_a",
            connections=[
                Connection(
                    fixture=parse_fixture_ref("ble_peer:peer1"),
                    options={"console": {"port": "COM9"}},
                )
            ],
        )
        self.assertEqual([], wiring_problems([testbed], []))

    def test_every_testbed_gets_a_line(self):
        testbeds = [make_testbed("tb_01"), make_testbed("tb_02", platform="board_b")]
        message = explain_no_match(make_instance(platform_allow=["board_c"]), testbeds)
        self.assertIn("tb_01", message)
        self.assertIn("tb_02", message)


class TestInstanceFixturesFromConfig(SdkTestCase):
    def test_fixtures_are_read_from_the_config(self):
        refs = instance_fixtures_from_config({"fixture": "flash"})
        self.assertEqual([ref.spec for ref in refs], ["flash"])

    def test_an_empty_config_yields_nothing(self):
        self.assertEqual(instance_fixtures_from_config({}), [])
        self.assertEqual(instance_fixtures_from_config(None), [])

    def test_a_malformed_fixture_is_reported_as_a_discovery_error(self):
        # Callers of this helper are reading a testcase file, so the error type
        # should say so, not leak the binding-file vocabulary.
        with self.assertRaises(DiscoveryError):
            instance_fixtures_from_config({"fixture": ":oops"})


if __name__ == "__main__":
    unittest.main()
