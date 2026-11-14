# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the fixture type catalogue and the checks it enables.

The catalogue has two layers and they must not be confused: a *type entry*
records that a type exists and which capability tokens it accepts, while the
*option contract* says where its values come from. Only the second makes a spec
an identity, and only it makes connection keys checkable. Pinning the line
between them is the point of this file -- a catalogue entry must stay silent
about a driver's own keys, and adding one must not change matching.
"""

from __future__ import annotations

import os
import tempfile
import unittest
from unittest import mock

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from sdk_test import binding
from sdk_test import fixture_types
from sdk_test import fixtures
from sdk_test.errors import DiscoveryError
from sdk_test.fixture_types import FixtureTypeError
from sdk_test.fixtures import parse_fixture_ref
from sdk_test.runner import RunOptions
from sdk_test.runner import TestRunner

#: Artifacts are written per run; these tests only plan, but the directory is
#: still resolved, so it is pointed at a scratch tree rather than the cwd.
_SCRATCH = tempfile.TemporaryDirectory(prefix="sdk_test_catalogue_")


def parse(document):
    return fixture_types.parse_fixture_types(document, source="test.yaml")


def binding_issues(connections):
    testbeds = binding.parse_binding(
        {"testbed": [{"id": "tb", "platform": "board_a", "connections": connections}]}
    )
    return binding.fixture_contract_issues(testbeds)


class TestCapabilitiesParsing(SdkTestCase):
    def test_capabilities_are_parsed_in_order(self):
        catalogue = parse(
            {"bt_peer": {"description": "peer", "capabilities": ["classic", "ble"]}}
        )
        self.assertEqual(
            catalogue.get("bt_peer").capabilities, ("classic", "ble")
        )

    def test_letters_digits_and_separators_are_accepted(self):
        catalogue = parse(
            {
                "radio": {
                    "capabilities": ["5g", "2g", "lab_a", "v1.2", "x-y"],
                }
            }
        )
        self.assertEqual(
            catalogue.get("radio").capabilities, ("5g", "2g", "lab_a", "v1.2", "x-y")
        )

    def test_capabilities_must_be_a_list(self):
        with self.assertRaises(FixtureTypeError) as caught:
            parse({"bt_peer": {"capabilities": "ble"}})
        self.assertIn("list", str(caught.exception))

    def test_an_empty_token_is_rejected(self):
        with self.assertRaises(FixtureTypeError) as caught:
            parse({"bt_peer": {"capabilities": [""]}})
        self.assertIn("capability token", str(caught.exception))

    def test_a_token_with_a_separator_inside_is_rejected(self):
        # A comma list in one string would silently mean two capabilities.
        with self.assertRaises(FixtureTypeError):
            parse({"bt_peer": {"capabilities": ["ble,classic"]}})

    def test_a_non_string_token_is_rejected(self):
        with self.assertRaises(FixtureTypeError):
            parse({"bt_peer": {"capabilities": [5]}})


class TestCatalogueEntries(SdkTestCase):
    def test_options_may_be_omitted(self):
        catalogue = parse(
            {"bt_peer": {"description": "peer", "capabilities": ["ble", "classic"]}}
        )
        contract = catalogue.get("bt_peer")
        self.assertEqual(contract.options, ())
        self.assertEqual(contract.description, "peer")

    def test_an_empty_options_mapping_is_still_accepted(self):
        catalogue = parse({"bt_peer": {"options": {}}})
        self.assertEqual(catalogue.get("bt_peer").options, ())

    def test_a_catalogue_entry_is_not_parameterized(self):
        catalogue = parse(
            {"bt_peer": {"capabilities": ["ble"], "options": {}}}
        )
        self.assertFalse(catalogue.is_parameterized("bt_peer"))

    def test_the_shipped_bt_peer_entry_is_a_vocabulary_not_a_contract(self):
        contracts = fixture_types.default_fixture_types()
        contract = contracts.get("bt_peer")
        self.assertIsNotNone(contract)
        self.assertEqual(contract.capabilities, ("ble", "classic"))
        self.assertEqual(contract.options, ())
        # The matching rule must not change: bt_peer is still not an identity.
        self.assertFalse(fixture_types.is_parameterized_type("bt_peer"))

    def test_the_shipped_bus_fixtures_have_no_capability_vocabulary(self):
        # `spi_nor_flash:spi1` names a bus, not a capability, so the token must
        # not be checked against a vocabulary.
        contracts = fixture_types.default_fixture_types()
        for name in ("spi_nor_flash", "spi_nand_flash", "i2c_sensor"):
            with self.subTest(name=name):
                self.assertIsNotNone(contracts.get(name))
                self.assertEqual(contracts.get(name).capabilities, ())

    def test_the_shipped_wifi_ap_keeps_its_options_and_gains_capabilities(self):
        contract = fixture_types.default_fixture_types().get("wifi_ap")
        self.assertEqual(contract.capabilities, ("2g", "5g"))
        self.assertEqual(contract.option_names, ("ssid", "security", "psk"))
        self.assertTrue(contract.parameterized)


class TestBindingChecks(SdkTestCase):
    """What a catalogue entry makes checkable about a lab's binding."""

    def test_a_driver_private_key_on_a_catalogue_only_type_does_not_warn(self):
        issues = binding_issues(
            [{"fixture": "bt_peer:ble", "console": {"port": "COM9"}}]
        )
        self.assertEqual(issues, [])

    def test_an_unknown_capability_of_a_known_type_warns(self):
        issues = binding_issues([{"fixture": "bt_peer:ble,lab_a"}])
        self.assertEqual([issue.severity for issue in issues], ["warning"])
        self.assertIn("lab_a", issues[0].message)
        self.assertIn("ble, classic", issues[0].message)

    def test_a_listed_capability_does_not_warn(self):
        self.assertEqual(binding_issues([{"fixture": "bt_peer:ble,classic"}]), [])

    def test_an_unknown_wifi_capability_warns_but_the_known_ones_do_not(self):
        issues = binding_issues([{"fixture": "wifi_ap:5g,lab_a"}])
        self.assertEqual([issue.severity for issue in issues], ["warning"])
        self.assertIn("lab_a", issues[0].message)
        self.assertNotIn('"5g"', issues[0].message)
        self.assertIn("2g, 5g", issues[0].message)

    def test_a_type_without_a_capability_vocabulary_is_not_checked(self):
        # "spi1" is a bus name; treating it as an unknown capability would
        # reject a good binding on every board.
        self.assertEqual(binding_issues([{"fixture": "spi_nor_flash:spi1"}]), [])

    def test_an_unregistered_type_is_not_checked_for_capabilities(self):
        self.assertEqual(binding_issues([{"fixture": "lab_scope:gain10"}]), [])

    def test_a_params_key_on_a_parameterized_type_is_still_an_error(self):
        issues = binding_issues([{"fixture": "wifi_ap:5g", "psk": "leaked"}])
        self.assertEqual([issue.severity for issue in issues], ["error"])


class TestUnknownTypeWarning(SdkTestCase):
    """A testcase may name a type the catalogue does not know -- loudly."""

    def setUp(self):
        super().setUp()
        # The developer's own environment must not decide whether this class
        # sees warnings or errors; each test sets the value it is about and
        # patch.dict restores the original on exit.
        patched = mock.patch.dict(
            os.environ, {fixtures.ENV_STRICT_FIXTURES: ""}, clear=False
        )
        patched.start()
        self.addCleanup(patched.stop)

    def suite(self, spec: str) -> str:
        self.write(
            "testcase.yaml",
            f"""
            tests:
              - id: probe
                harness: console
                integration_platforms: [qemu_sf32lb52x]
                harness_config:
                  fixture: {spec}
                  record: ["x"]
            """,
        )
        self.write("project/SConstruct", "# project\n")
        self.write("project/proj.conf", "CONFIG_X=y\n")
        return self.root

    def runner(self, echo):
        return TestRunner(
            RunOptions(testsuite_root=self.root, build=False, out_dir=_SCRATCH.name),
            echo=echo,
        )

    def test_an_unregistered_type_warns_and_the_plan_still_runs(self):
        self.suite("lab_scope:scope1")
        lines = []
        planned = self.runner(lines.append).plan()

        self.assertEqual(1, len(planned))
        text = "\n".join(lines)
        self.assertIn("[warn]", text)
        self.assertIn("lab_scope", text)
        self.assertIn("fixture_types.yaml", text)

    def test_a_registered_type_does_not_warn(self):
        self.suite("bt_peer:ble")
        lines = []
        self.runner(lines.append).plan()
        self.assertEqual([], lines)

    def test_strict_mode_upgrades_the_warning_to_an_error(self):
        self.suite("lab_scope:scope1")
        lines = []
        with mock.patch.dict(os.environ, {fixtures.ENV_STRICT_FIXTURES: "1"}):
            with self.assertRaises(DiscoveryError) as caught:
                self.runner(lines.append).plan()
        self.assertIn("lab_scope", str(caught.exception))
        self.assertIn(fixtures.ENV_STRICT_FIXTURES, str(caught.exception))

    def test_strict_mode_is_opt_in(self):
        self.suite("lab_scope:scope1")
        with mock.patch.dict(os.environ, {fixtures.ENV_STRICT_FIXTURES: "0"}):
            self.assertEqual(1, len(self.runner(lambda _m: None).plan()))


class TestUnregisteredTypeHelper(SdkTestCase):
    def test_names_are_reported_once_each_in_request_order(self):
        refs = [
            parse_fixture_ref("lab_scope:one"),
            parse_fixture_ref("bt_peer:ble"),
            parse_fixture_ref("lab_scope:two"),
        ]
        self.assertEqual(fixtures.unregistered_fixture_types(refs), ["lab_scope"])

    def test_a_contract_may_be_passed_in(self):
        catalogue = parse({"lab_scope": {"description": "a scope"}})
        self.assertEqual(
            fixtures.unregistered_fixture_types(
                [parse_fixture_ref("lab_scope:one")], catalogue
            ),
            [],
        )


if __name__ == "__main__":
    unittest.main()
