# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for ``lab_agent.config``."""

from __future__ import annotations

import unittest

from _support import CONFIG_YAML  # noqa: F401  (also fixes sys.path)
from _support import LabTestCase

from lab_agent.config import RunnerSettings
from lab_agent.config import find_jlink
from lab_agent.config import load_runner_config
from lab_agent.config import parse_runner_config
from lab_agent.errors import ConfigError


class TestRunnerSettings(LabTestCase):
    def test_defaults_are_usable_without_a_runner_block(self):
        settings = RunnerSettings.from_mapping(None)
        self.assertEqual(settings.host, "127.0.0.1")
        self.assertEqual(settings.name, "local")

    def test_an_absent_block_and_an_empty_block_agree(self):
        self.assertEqual(RunnerSettings.from_mapping(None), RunnerSettings.from_mapping({}))

    def test_values_are_applied(self):
        settings = RunnerSettings.from_mapping({"name": "rack-3", "port": 9000})
        self.assertEqual((settings.name, settings.port), ("rack-3", 9000))

    def test_an_unknown_key_is_rejected_with_the_known_ones_listed(self):
        with self.assertRaises(ConfigError) as caught:
            RunnerSettings.from_mapping({"prot": 8760})
        message = str(caught.exception)
        self.assertIn("prot", message)
        self.assertIn("port", message)  # the suggestion

    def test_a_non_mapping_block_is_rejected(self):
        with self.assertRaises(ConfigError):
            RunnerSettings.from_mapping([1, 2])

    def test_port_must_be_an_integer(self):
        with self.assertRaises(ConfigError):
            RunnerSettings.from_mapping({"port": "8760"})

    def test_max_buffer_must_be_a_sane_integer(self):
        with self.assertRaises(ConfigError):
            RunnerSettings.from_mapping({"max_buffer": 10})

    def test_timings_must_be_positive(self):
        with self.assertRaises(ConfigError):
            RunnerSettings.from_mapping({"console_poll": 0})
        with self.assertRaises(ConfigError):
            RunnerSettings.from_mapping({"jlink_timeout": -1})


class TestParseRunnerConfig(LabTestCase):
    def test_a_full_config_parses(self):
        config = self.make_config()
        self.assertEqual(config.settings.name, "test-runner")
        self.assertEqual([tb.id for tb in config.testbeds], ["tb_01", "tb_02"])

    def test_the_runner_block_is_not_mistaken_for_a_testbed(self):
        config = self.make_config()
        self.assertEqual(len(config.testbeds), 2)

    def test_testbed_entries_keep_their_runner_owned_settings(self):
        config = self.make_config()
        first = config.by_id("tb_01")
        self.assertEqual(first.runner_option("jlink")["serial"], "59605049")
        self.assertEqual(first.runner_option("flash")["method"], "jlink")

    def test_a_non_mapping_document_is_rejected(self):
        with self.assertRaises(ConfigError):
            parse_runner_config(["not", "a", "mapping"])

    def test_duplicate_testbed_ids_are_rejected(self):
        text = """
        testbed:
          - id: same
            platform: a
          - id: same
            platform: b
        """
        with self.assertRaises(Exception) as caught:
            self.make_config(text)
        self.assertIn("duplicate", str(caught.exception).lower())

    def test_by_id_names_the_known_testbeds_when_asked_for_a_stranger(self):
        config = self.make_config()
        with self.assertRaises(ConfigError) as caught:
            config.by_id("nope")
        self.assertIn("tb_01", str(caught.exception))


class TestCheck(LabTestCase):
    def test_a_well_formed_config_reports_nothing(self):
        self.assertEqual(self.make_config().check(), [])

    def test_a_testbed_without_a_console_is_flagged(self):
        config = self.make_config(
            """
            testbed:
              - id: tb_no_console
                platform: some-board
            """
        )
        warnings = config.check()
        self.assertEqual(len(warnings), 1)
        self.assertIn("tb_no_console", warnings[0])
        self.assertIn("console", warnings[0])


class TestLoadRunnerConfig(LabTestCase):
    def test_a_missing_file_is_reported(self):
        with self.assertRaises(ConfigError) as caught:
            load_runner_config(self.root + "/absent.yaml")
        self.assertIn("not found", str(caught.exception))

    def test_an_empty_file_is_reported(self):
        path = self.write("runner.yaml", "\n")
        with self.assertRaises(ConfigError) as caught:
            load_runner_config(path)
        self.assertIn("empty", str(caught.exception))

    def test_malformed_yaml_is_reported(self):
        path = self.write("runner.yaml", "testbed: [unclosed\n")
        with self.assertRaises(ConfigError) as caught:
            load_runner_config(path)
        self.assertIn("malformed YAML", str(caught.exception))

    def test_the_shipped_example_config_is_valid(self):
        # The example is a template users copy; it must stay loadable.
        import os

        from _support import TOOLS_DIR

        example = os.path.join(
            TOOLS_DIR, "lab_agent", "examples", "runner.local.yaml"
        )
        config = load_runner_config(example)
        self.assertGreaterEqual(len(config.testbeds), 1)
        self.assertEqual(config.check(), [])


class TestFindJlink(LabTestCase):
    def test_an_explicit_missing_path_is_reported(self):
        with self.assertRaises(ConfigError) as caught:
            find_jlink(self.root + "/no-such-jlink.exe")
        self.assertIn("does not exist", str(caught.exception))

    def test_an_explicit_existing_path_wins(self):
        path = self.write("tools/jlink.exe", "")
        self.assertEqual(find_jlink(path), path)


if __name__ == "__main__":
    unittest.main()
