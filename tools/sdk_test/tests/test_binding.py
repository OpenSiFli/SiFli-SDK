# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for ``sdk_test.binding``.

The binding file is the only place in the framework that mentions physical
reality, and it is written by the hardware team rather than by test authors.
That makes a clear error message the feature under test here: a typo in a mux
reference or a missing console port must fail at load time, with the offending
location named.
"""

from __future__ import annotations

import unittest

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from sdk_test import binding
from sdk_test.errors import BindingError

MINIMAL = {
    "testbed": [
        {
            "id": "tb_01",
            "platform": "sf32lb52-lcd_n16r8",
            "console": {"port": "COM13"},
        }
    ]
}


class TestTopLevelShapes(SdkTestCase):
    def test_a_bare_list_is_accepted(self):
        testbeds = binding.parse_binding(list(MINIMAL["testbed"]))
        self.assertEqual([t.id for t in testbeds], ["tb_01"])

    def test_a_missing_testbed_key_is_reported(self):
        with self.assertRaises(BindingError) as caught:
            binding.parse_binding({"boards": []}, source="b.yaml")
        self.assertIn("testbed", str(caught.exception))

    def test_a_non_list_testbed_is_reported(self):
        with self.assertRaises(BindingError):
            binding.parse_binding({"testbed": {"id": "x"}})

    def test_a_scalar_top_level_is_reported(self):
        with self.assertRaises(BindingError):
            binding.parse_binding("just a string")


class TestTestbedValidation(SdkTestCase):
    def test_a_missing_id_is_reported(self):
        with self.assertRaises(BindingError) as caught:
            binding.parse_binding({"testbed": [{"platform": "p"}]})
        self.assertIn('needs an "id"', str(caught.exception))

    def test_a_missing_platform_is_reported_with_the_testbed_id(self):
        with self.assertRaises(BindingError) as caught:
            binding.parse_binding({"testbed": [{"id": "tb_01"}]})
        self.assertIn("tb_01", str(caught.exception))
        self.assertIn("platform", str(caught.exception))

    def test_a_non_mapping_entry_is_reported_with_its_index(self):
        with self.assertRaises(BindingError) as caught:
            binding.parse_binding({"testbed": ["nope"]})
        self.assertIn("testbed[0]", str(caught.exception))

    def test_duplicate_testbed_ids_are_rejected(self):
        entry = {"id": "tb_01", "platform": "p"}
        with self.assertRaises(BindingError) as caught:
            binding.parse_binding({"testbed": [entry, dict(entry)]}, source="b.yaml")
        self.assertIn("duplicate testbed id", str(caught.exception))

    def test_a_testbed_without_a_console_is_still_loadable(self):
        # A build-only or host-backed testbed has no serial port to name.
        testbeds = binding.parse_binding({"testbed": [{"id": "t", "platform": "p"}]})
        self.assertIsNone(testbeds[0].console)


class TestConsoleValidation(SdkTestCase):
    def test_the_default_baud_matches_the_sdk_console(self):
        testbeds = binding.parse_binding(MINIMAL)
        self.assertEqual(testbeds[0].console.baud, 1000000)

    def test_a_missing_port_is_reported(self):
        with self.assertRaises(BindingError) as caught:
            binding.parse_binding(
                {"testbed": [{"id": "t", "platform": "p", "console": {"baud": 9600}}]}
            )
        self.assertIn('needs a "port"', str(caught.exception))

    def test_a_non_integer_baud_is_reported(self):
        with self.assertRaises(BindingError) as caught:
            binding.parse_binding(
                {"testbed": [{"id": "t", "platform": "p", "console": {"port": "COM1", "baud": "fast"}}]}
            )
        self.assertIn("baud must be an integer", str(caught.exception))


class TestConnectionValidation(SdkTestCase):
    def test_a_connection_without_a_fixture_is_reported(self):
        with self.assertRaises(BindingError) as caught:
            binding.parse_binding(
                {"testbed": [{"id": "t", "platform": "p", "connections": [{"pinmap": []}]}]}
            )
        self.assertIn('needs a "fixture"', str(caught.exception))

    def test_a_connection_is_reported_against_its_testbed_and_index(self):
        with self.assertRaises(BindingError) as caught:
            binding.parse_binding(
                {"testbed": [{"id": "t", "platform": "p", "connections": [{"fixture": "f", "pinmap": 3}]}]}
            )
        message = str(caught.exception)
        self.assertIn("testbed[0].connections[0]", message)
        self.assertIn("pinmap must be a list", message)

    def test_a_connection_without_a_pinmap_gets_an_empty_one(self):
        testbeds = binding.parse_binding(
            {"testbed": [{"id": "t", "platform": "p", "connections": [{"fixture": "f"}]}]}
        )
        self.assertEqual(testbeds[0].connections[0].pinmap, [])


class TestMuxValidation(SdkTestCase):
    def test_muxes_are_parsed(self):
        testbeds = binding.parse_binding(
            {
                "testbed": [
                    {
                        "id": "t",
                        "platform": "p",
                        "muxes": [{"id": "mux_01", "type": "relay", "channel_num": 4}],
                    }
                ]
            }
        )
        self.assertEqual(testbeds[0].muxes[0].id, "mux_01")
        self.assertEqual(testbeds[0].muxes[0].channel_num, 4)

    def test_a_missing_mux_id_is_reported(self):
        with self.assertRaises(BindingError):
            binding.parse_binding({"testbed": [{"id": "t", "platform": "p", "muxes": [{"type": "relay"}]}]})

    def test_a_non_integer_channel_count_is_reported(self):
        with self.assertRaises(BindingError):
            binding.parse_binding(
                {"testbed": [{"id": "t", "platform": "p", "muxes": [{"id": "m", "channel_num": "four"}]}]}
            )

    def test_a_connection_naming_an_undeclared_mux_is_rejected(self):
        with self.assertRaises(BindingError) as caught:
            binding.parse_binding(
                {
                    "testbed": [
                        {
                            "id": "t",
                            "platform": "p",
                            "muxes": [{"id": "mux_01", "channel_num": 4}],
                            "connections": [{"fixture": "f", "mux": "mux_99.channel.1"}],
                        }
                    ]
                }
            )
        message = str(caught.exception)
        self.assertIn("mux_99", message)
        # The message should say what *is* available, not just what is wrong.
        self.assertIn("mux_01", message)

    def test_a_mux_reference_is_allowed_when_the_testbed_declares_none(self):
        # Muxes are owned by the runner rather than the testbed, so a binding
        # may name a mux from the runner's pool; only a *contradicted* reference
        # (testbed declares muxes, but not this one) is an error.
        testbeds = binding.parse_binding(
            {
                "testbed": [
                    {"id": "t", "platform": "p", "connections": [{"fixture": "f", "mux": "mux_01.channel.1"}]}
                ]
            }
        )
        self.assertEqual(testbeds[0].connections[0].mux, "mux_01.channel.1")

    def test_the_channel_suffix_is_not_part_of_the_mux_id(self):
        testbeds = binding.parse_binding(
            {
                "testbed": [
                    {
                        "id": "t",
                        "platform": "p",
                        "muxes": [{"id": "mux_01", "channel_num": 8}],
                        "connections": [{"fixture": "f", "mux": "mux_01.channel.7"}],
                    }
                ]
            }
        )
        self.assertEqual(len(testbeds[0].muxes), 1)


class TestRunnerOwnedKeys(SdkTestCase):
    """The binding file is shared with the Lab Agent, so keys parsing leaves
    alone must survive it instead of being dropped."""

    def test_unknown_keys_are_preserved_for_the_runner(self):
        testbeds = binding.parse_binding(
            {
                "testbed": [
                    {
                        "id": "tb_01",
                        "platform": "p",
                        "jlink": {"serial": "59605049", "speed": 4000},
                        "flash": {"method": "jlink"},
                    }
                ]
            }
        )
        testbed = testbeds[0]
        self.assertEqual(testbed.runner_option("flash"), {"method": "jlink"})
        self.assertEqual(testbed.raw["jlink"]["serial"], "59605049")

    def test_an_absent_runner_option_yields_the_default(self):
        testbeds = binding.parse_binding(MINIMAL)
        self.assertIsNone(testbeds[0].runner_option("jlink"))
        self.assertEqual(testbeds[0].runner_option("flash", {}), {})

    def test_interpreted_keys_are_still_in_raw_so_the_runner_sees_one_view(self):
        testbeds = binding.parse_binding(MINIMAL)
        self.assertEqual(testbeds[0].raw["console"]["port"], "COM13")

    def test_a_connection_keeps_the_keys_the_agent_needs(self):
        # An instrument's console lives on its connection, next to the fixture
        # name. Parsing only fixture/pinmap/mux would drop it silently, and the
        # agent would then report "no console configured" for a binding that
        # plainly has one.
        testbeds = binding.parse_binding(
            {
                "testbed": [
                    {
                        "id": "tb_01",
                        "platform": "sf32lb52-lcd_n16r8",
                        "connections": [
                            {
                                "fixture": "ble_peer",
                                "console": {"port": "COM99", "baud": 1000000},
                            }
                        ],
                    }
                ]
            }
        )
        connection = testbeds[0].connections[0]
        self.assertEqual(connection.fixture.name, "ble_peer")
        self.assertEqual(connection.options["console"]["port"], "COM99")
        self.assertEqual(connection.options["console"]["baud"], 1000000)


if __name__ == "__main__":
    unittest.main()
