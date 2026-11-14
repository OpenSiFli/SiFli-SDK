# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the fixture type contract and the matching rule it changes.

The contract is the single source for option *names*, and it is shipped with the
code, so every mistake in it is a mistake to fix rather than a value to ignore:
an unknown placeholder, an unsupported ``source``, a condition the resolver
cannot evaluate. The matching consequences are pinned here too -- a
parameterized type is identified by its spec, so a lab connection that offers a
superset of the requested capabilities must not satisfy it.
"""

from __future__ import annotations

import unittest

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from sdk_test import binding
from sdk_test import fixture_types
from sdk_test.fixture_types import FixtureTypeError
from sdk_test.fixtures import parse_fixture_ref
from sdk_test.fixtures import select_testbeds
from sdk_test.model import Connection
from sdk_test.model import TestInstance
from sdk_test.model import TestBed


def wifi_ap_contract(psk: dict = None, **overrides) -> dict:
    """A contract document close to the bundled one, adjustable per test."""
    document = {
        "wifi_ap": {
            "description": "An access point.",
            "options": {
                "ssid": {
                    "source": "params",
                    "key": "fixtures.{spec}.ssid",
                    "required": True,
                },
                "security": {
                    "source": "params",
                    "key": "fixtures.{spec}.security",
                    "enum": ["open", "wpa2"],
                    "default": "wpa2",
                },
                "psk": psk
                or {
                    "source": "params",
                    "key": "fixtures.{spec}.psk",
                    "required_if": "security != open",
                    "secret": True,
                },
            },
        }
    }
    document["wifi_ap"].update(overrides)
    return document


def parse(document: dict):
    return fixture_types.parse_fixture_types(document, source="test.yaml")


def make_instance(fixtures=(), platform: str = "board_a") -> TestInstance:
    return TestInstance(
        id="suite/case",
        testsuite_root=".",
        testcase_dir=".",
        tests_key="case",
        harness="pytest",
        fixtures=[parse_fixture_ref(item) for item in fixtures],
        integration_platforms=[platform],
    )


class TestBundledContract(SdkTestCase):
    def test_the_shipped_contract_describes_wifi_ap(self):
        contracts = fixture_types.default_fixture_types()
        contract = contracts.get("wifi_ap")
        self.assertIsNotNone(contract)
        self.assertEqual(contract.option_names, ("ssid", "security", "psk"))

    def test_ssid_is_required_and_psk_is_secret_and_conditional(self):
        contract = fixture_types.default_fixture_types().get("wifi_ap")
        self.assertTrue(contract.option("ssid").required)
        psk = contract.option("psk")
        self.assertTrue(psk.secret)
        self.assertEqual(str(psk.required_if), "security != open")
        self.assertEqual(contract.option("security").default, "wpa2")

    def test_the_keys_use_the_fixtures_prefix_and_the_requested_spec(self):
        contract = fixture_types.default_fixture_types().get("wifi_ap")
        self.assertEqual(
            contract.option("psk").key_for("wifi_ap:5g"),
            "fixtures.wifi_ap:5g.psk",
        )

    def test_a_type_without_a_contract_is_not_parameterized(self):
        self.assertFalse(fixture_types.is_parameterized_type("bt_peer"))
        self.assertFalse(fixture_types.is_parameterized_type("no_such_type"))

    def test_wifi_ap_is_parameterized(self):
        self.assertTrue(fixture_types.is_parameterized_type("wifi_ap"))


class TestContractValidation(SdkTestCase):
    def test_an_unknown_placeholder_is_rejected(self):
        document = wifi_ap_contract()
        document["wifi_ap"]["options"]["ssid"]["key"] = "fixtures.{spec}.{option}"
        with self.assertRaises(FixtureTypeError) as caught:
            parse(document)
        self.assertIn("unknown placeholder", str(caught.exception))

    def test_source_binding_is_reported_as_not_implemented(self):
        document = wifi_ap_contract()
        document["wifi_ap"]["options"]["ssid"]["source"] = "binding"
        with self.assertRaises(FixtureTypeError) as caught:
            parse(document)
        self.assertIn("not implemented", str(caught.exception))

    def test_an_unknown_source_is_rejected(self):
        document = wifi_ap_contract()
        document["wifi_ap"]["options"]["ssid"]["source"] = "magic"
        with self.assertRaises(FixtureTypeError):
            parse(document)

    def test_required_if_without_an_operator_is_rejected(self):
        document = wifi_ap_contract()
        document["wifi_ap"]["options"]["psk"]["required_if"] = "security"
        with self.assertRaises(FixtureTypeError) as caught:
            parse(document)
        self.assertIn("not supported", str(caught.exception))

    def test_required_if_with_a_quoted_literal_is_rejected(self):
        document = wifi_ap_contract()
        document["wifi_ap"]["options"]["psk"]["required_if"] = 'security != "open"'
        with self.assertRaises(FixtureTypeError):
            parse(document)

    def test_required_if_naming_an_unknown_option_is_rejected(self):
        document = wifi_ap_contract()
        document["wifi_ap"]["options"]["psk"]["required_if"] = "cipher != open"
        with self.assertRaises(FixtureTypeError) as caught:
            parse(document)
        self.assertIn("not an option", str(caught.exception))

    def test_a_type_may_have_no_options(self):
        # A catalogue entry that only says "this type exists" is complete.
        contracts = parse({"wifi_ap": {"description": "no options"}})
        self.assertEqual(contracts.get("wifi_ap").options, ())
        self.assertFalse(contracts.get("wifi_ap").parameterized)

    def test_required_if_naming_an_optional_option_without_a_default_is_rejected(self):
        # `security` may be absent, so the condition could never be evaluated
        # and `psk` would be reported instead of the missing dependency.
        document = wifi_ap_contract()
        del document["wifi_ap"]["options"]["security"]["default"]
        with self.assertRaises(FixtureTypeError) as caught:
            parse(document)
        message = str(caught.exception)
        self.assertIn("security", message)
        self.assertIn("required or have a default", message)

    def test_required_if_may_name_a_required_option(self):
        document = wifi_ap_contract()
        del document["wifi_ap"]["options"]["security"]["default"]
        document["wifi_ap"]["options"]["security"]["required"] = True
        contracts = parse(document)
        self.assertEqual(
            str(contracts.get("wifi_ap").option("psk").required_if),
            "security != open",
        )

    def test_required_if_may_reference_an_option_declared_later(self):
        # The dependency is declared after the option that names it; the check
        # runs once every option is parsed, so the order must not matter.
        document = {
            "wifi_ap": {
                "options": {
                    "psk": {
                        "source": "params",
                        "key": "fixtures.{spec}.psk",
                        "required_if": "security != open",
                        "secret": True,
                    },
                    "security": {
                        "source": "params",
                        "key": "fixtures.{spec}.security",
                        "default": "wpa2",
                    },
                }
            }
        }
        contracts = parse(document)
        self.assertEqual(
            str(contracts.get("wifi_ap").option("psk").required_if),
            "security != open",
        )

    def test_an_unmatched_opening_brace_in_a_key_is_rejected(self):
        document = wifi_ap_contract()
        document["wifi_ap"]["options"]["ssid"]["key"] = "fixtures.{spec.ssid"
        with self.assertRaises(FixtureTypeError) as caught:
            parse(document)
        self.assertIn("unmatched", str(caught.exception))

    def test_two_options_may_not_share_a_params_key(self):
        document = wifi_ap_contract()
        document["wifi_ap"]["options"]["psk_b"] = {
            "source": "params",
            "key": "fixtures.{spec}.psk",
        }
        with self.assertRaises(FixtureTypeError) as caught:
            parse(document)
        message = str(caught.exception)
        self.assertIn("psk_b", message)
        self.assertIn("same params key", message)

    def test_an_unmatched_closing_brace_in_a_key_is_rejected(self):
        document = wifi_ap_contract()
        document["wifi_ap"]["options"]["ssid"]["key"] = "fixtures.spec}"
        with self.assertRaises(FixtureTypeError) as caught:
            parse(document)
        self.assertIn("unmatched", str(caught.exception))

    def test_a_complete_placeholder_inside_a_key_is_accepted(self):
        document = wifi_ap_contract()
        document["wifi_ap"]["options"]["ssid"]["key"] = "fixtures.{spec}.ssid"
        self.assertEqual(
            parse(document).get("wifi_ap").option("ssid").key,
            "fixtures.{spec}.ssid",
        )

    def test_enum_entries_must_be_strings(self):
        document = wifi_ap_contract()
        document["wifi_ap"]["options"]["security"]["enum"] = [1, 2]
        with self.assertRaises(FixtureTypeError) as caught:
            parse(document)
        self.assertIn("must be strings", str(caught.exception))

    def test_duplicate_enum_entries_are_rejected(self):
        document = wifi_ap_contract()
        document["wifi_ap"]["options"]["security"]["enum"] = ["wpa2", "wpa2"]
        with self.assertRaises(FixtureTypeError) as caught:
            parse(document)
        self.assertIn("more than once", str(caught.exception))

    def test_a_default_must_be_a_string_when_the_option_has_an_enum(self):
        document = wifi_ap_contract()
        document["wifi_ap"]["options"]["security"]["enum"] = ["1"]
        document["wifi_ap"]["options"]["security"]["default"] = 1
        with self.assertRaises(FixtureTypeError) as caught:
            parse(document)
        self.assertIn("is not one of", str(caught.exception))

    def test_a_non_string_option_name_is_rejected(self):
        document = wifi_ap_contract()
        document["wifi_ap"]["options"][1] = {
            "source": "params",
            "key": "fixtures.{spec}.other",
        }
        with self.assertRaises(FixtureTypeError) as caught:
            parse(document)
        self.assertIn("option names must be strings", str(caught.exception))

    def test_a_non_string_type_name_is_rejected(self):
        with self.assertRaises(FixtureTypeError) as caught:
            parse({1: {"description": "numeric type name"}})
        self.assertIn("fixture type names must be strings", str(caught.exception))

    def test_a_duplicate_fixture_type_name_is_rejected(self):
        path = self.write(
            "fixture_types.yaml",
            """
            wifi_ap:
              description: one
            wifi_ap:
              description: two
            """,
        )
        with self.assertRaises(FixtureTypeError) as caught:
            fixture_types.load_fixture_types(path)
        message = str(caught.exception)
        self.assertIn("duplicate key 'wifi_ap'", message)
        self.assertIn("line 3", message)

    def test_a_duplicate_option_name_in_a_nested_mapping_is_rejected(self):
        path = self.write(
            "fixture_types.yaml",
            """
            wifi_ap:
              options:
                ssid:
                  source: params
                  key: "fixtures.{spec}.ssid"
                ssid:
                  source: params
                  key: "fixtures.{spec}.other"
            """,
        )
        with self.assertRaises(FixtureTypeError) as caught:
            fixture_types.load_fixture_types(path)
        message = str(caught.exception)
        self.assertIn("duplicate key 'ssid'", message)
        self.assertIn("line 6", message)

    def test_a_yaml_merge_key_is_flattened_not_treated_as_a_duplicate(self):
        # `<<` is not a literal key: the base loader flattens it, and scanning
        # it as a scalar would fail on a document `yaml.safe_load` accepts. An
        # explicit key that overrides a merged one is valid YAML, not a
        # duplicate.
        path = self.write(
            "fixture_types.yaml",
            """
            wifi_ap:
              <<: {description: from merge, options: {ssid: {source: params, key: "fixtures.{spec}.ssid"}}}
              description: explicit
            """,
        )
        contracts = fixture_types.load_fixture_types(path)
        contract = contracts.get("wifi_ap")
        self.assertEqual(contract.description, "explicit")
        self.assertEqual(contract.option_names, ("ssid",))


    def test_an_unknown_type_field_is_rejected(self):
        document = wifi_ap_contract()
        document["wifi_ap"]["driver"] = "wifi"
        with self.assertRaises(FixtureTypeError) as caught:
            parse(document)
        self.assertIn("unknown field", str(caught.exception))

    def test_an_unknown_option_field_is_rejected(self):
        document = wifi_ap_contract()
        document["wifi_ap"]["options"]["ssid"]["default_when"] = "always"
        with self.assertRaises(FixtureTypeError):
            parse(document)

    def test_an_option_may_not_shadow_a_binding_key(self):
        document = wifi_ap_contract()
        document["wifi_ap"]["options"]["mux"] = {
            "source": "params",
            "key": "fixtures.{spec}.mux",
        }
        with self.assertRaises(FixtureTypeError) as caught:
            parse(document)
        self.assertIn("binding key", str(caught.exception))

    def test_a_secret_option_may_not_have_a_default(self):
        document = wifi_ap_contract()
        document["wifi_ap"]["options"]["psk"]["default"] = "committed-secret"
        with self.assertRaises(FixtureTypeError) as caught:
            parse(document)
        self.assertIn("secret", str(caught.exception))

    def test_a_default_must_be_one_of_the_enum_values(self):
        document = wifi_ap_contract()
        document["wifi_ap"]["options"]["security"]["default"] = "wep"
        with self.assertRaises(FixtureTypeError):
            parse(document)

    def test_a_spec_substitution_happens_once_and_only_once(self):
        document = wifi_ap_contract()
        document["wifi_ap"]["options"]["ssid"]["key"] = "fixtures.{spec}.ssid"
        contracts = parse(document)
        # The spec itself contains a brace-looking fragment, which must not be
        # expanded a second time.
        self.assertEqual(
            contracts.get("wifi_ap").option("ssid").key_for("wifi_ap:{spec}"),
            "fixtures.wifi_ap:{spec}.ssid",
        )


class TestParameterizedMatching(SdkTestCase):
    """A parameterized type is an identity, not a capability set."""

    def test_a_superset_spec_does_not_satisfy_the_request(self):
        testbed = TestBed(
            id="tb",
            platform="board_a",
            connections=[Connection(fixture=parse_fixture_ref("wifi_ap:5g,lab_a"))],
        )
        self.assertEqual(select_testbeds(make_instance(["wifi_ap:5g"]), [testbed]), [])

    def test_an_exactly_equal_spec_matches(self):
        testbed = TestBed(
            id="tb",
            platform="board_a",
            connections=[Connection(fixture=parse_fixture_ref("wifi_ap:5g"))],
        )
        self.assertEqual(
            [item.id for item in select_testbeds(make_instance(["wifi_ap:5g"]), [testbed])],
            ["tb"],
        )

    def test_a_non_parameterized_type_still_matches_by_capability_subset(self):
        testbed = TestBed(
            id="tb",
            platform="board_a",
            connections=[Connection(fixture=parse_fixture_ref("bt_peer:ble,classic"))],
        )
        self.assertEqual(
            [item.id for item in select_testbeds(make_instance(["bt_peer:ble"]), [testbed])],
            ["tb"],
        )


class TestBindingContractCheck(SdkTestCase):
    def test_a_params_sourced_key_in_a_binding_is_an_error(self):
        testbeds = binding.parse_binding(
            {
                "testbed": [
                    {
                        "id": "tb",
                        "platform": "board_a",
                        "connections": [
                            {"fixture": "wifi_ap:5g", "ssid": "leaked", "psk": "leaked"}
                        ],
                    }
                ]
            }
        )
        errors, warnings = binding.check_fixture_contracts(testbeds)
        self.assertEqual(len(errors), 2)
        self.assertTrue(all("source: params" in error for error in errors))
        self.assertEqual(warnings, [])

    def test_an_unknown_key_only_warns_and_names_the_allowed_keys(self):
        testbeds = binding.parse_binding(
            {
                "testbed": [
                    {
                        "id": "tb",
                        "platform": "board_a",
                        "connections": [{"fixture": "wifi_ap:5g", "password": "typo"}],
                    }
                ]
            }
        )
        errors, warnings = binding.check_fixture_contracts(testbeds)
        self.assertEqual(errors, [])
        self.assertEqual(len(warnings), 1)
        for allowed in ("ssid", "security", "psk"):
            self.assertIn(allowed, warnings[0])

    def test_a_type_without_a_catalogue_entry_is_left_alone(self):
        # A lab-defined instrument keeps its own keys; that is the driver's
        # business and must not become a second schema.
        testbeds = binding.parse_binding(
            {
                "testbed": [
                    {
                        "id": "tb",
                        "platform": "board_a",
                        "connections": [
                            {
                                "fixture": "lab_scope:scope1",
                                "console": {"port": "COM9"},
                                "gain": 3,
                            }
                        ],
                    }
                ]
            }
        )
        errors, warnings = binding.check_fixture_contracts(testbeds)
        self.assertEqual((errors, warnings), ([], []))

    def test_a_catalogue_entry_without_options_does_not_check_connection_keys(self):
        # `bt_peer` is registered for its capabilities only. Its console is the
        # instrument driver's business, not an unknown key.
        testbeds = binding.parse_binding(
            {
                "testbed": [
                    {
                        "id": "tb",
                        "platform": "board_a",
                        "connections": [
                            {"fixture": "bt_peer:ble", "console": {"port": "COM9"}}
                        ],
                    }
                ]
            }
        )
        errors, warnings = binding.check_fixture_contracts(testbeds)
        self.assertEqual((errors, warnings), ([], []))

    def test_the_shipped_contract_is_used_by_default(self):
        testbeds = binding.parse_binding(
            {
                "testbed": [
                    {
                        "id": "tb",
                        "platform": "board_a",
                        "connections": [{"fixture": "wifi_ap:5g", "psk": "x"}],
                    }
                ]
            }
        )
        errors, _warnings = binding.check_fixture_contracts(testbeds)
        self.assertEqual(len(errors), 1)


if __name__ == "__main__":
    unittest.main()
