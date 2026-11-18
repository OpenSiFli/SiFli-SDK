# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for ``lab_agent.config``."""

from __future__ import annotations

import unittest
from unittest import mock

from _support import CONFIG_YAML  # noqa: F401  (also fixes sys.path)
from _support import LabTestCase

from sdk_test.binding import parse_binding
from sdk_test.model import Connection
from sdk_test.model import FixtureRef
from sdk_test.model import TestBed

from lab_agent.config import RunnerSettings
from lab_agent.config import find_jlink
from lab_agent.config import load_runner_config
from lab_agent.config import parse_runner_config
from lab_agent.config import rule_out_contradictions
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


class TestSerialPortResolution(LabTestCase):
    def _ports(self, *ports):
        return [
            mock.Mock(device=device, serial_number=serial, description=description)
            for device, serial, description in ports
        ]

    def test_com_ports_are_preserved_case_insensitively(self):
        config = self.make_config(
            """
            testbed:
              - id: tb
                platform: board_a
                console: {port: com5}
            """
        )
        self.assertEqual(config.by_id("tb").console.port, "com5")

    @mock.patch("serial.tools.list_ports.comports")
    def test_descriptor_resolves_dut_fixture_and_flash_ports(self, comports):
        comports.side_effect = [
            self._ports(("COM11", "DUT-SERIAL", "USB Serial")),
            self._ports(("COM13", "FLASH-SERIAL", "USB Serial")),
            self._ports(("COM12", "PEER-SERIAL", "USB Serial")),
        ]
        config = self.make_config(
            """
            testbed:
              - id: tb
                platform: board_a
                console: {port: DUT-SERIAL}
                flash: {method: sftool, port: FLASH-SERIAL}
                connections:
                  - fixture: bt_peer:ble
                    console: {port: PEER-SERIAL}
            """
        )
        testbed = config.by_id("tb")
        self.assertEqual(testbed.console.port, "COM11")
        self.assertEqual(testbed.connections[0].options["console"]["port"], "COM12")
        self.assertEqual(testbed.runner_option("flash")["port"], "COM13")
        self.assertEqual(comports.call_count, 3)

    @mock.patch("serial.tools.list_ports.comports")
    def test_descriptor_without_match_is_a_config_error(self, comports):
        comports.return_value = []
        with self.assertRaises(ConfigError) as caught:
            self.make_config(
                """
                testbed:
                  - id: tb
                    platform: board_a
                    console: {port: UNKNOWN-SERIAL}
                """
            )
        self.assertIn("no COM port found", str(caught.exception))

    @mock.patch("serial.tools.list_ports.comports")
    def test_ambiguous_descriptor_is_a_config_error(self, comports):
        comports.return_value = self._ports(
            ("COM11", "SHARED-SERIAL", "USB Serial"),
            ("COM12", "SHARED-SERIAL", "USB Serial"),
        )
        with self.assertRaises(ConfigError) as caught:
            self.make_config(
                """
                testbed:
                  - id: tb
                    platform: board_a
                    console: {port: SHARED-SERIAL}
                """
            )
        self.assertIn("multiple COM ports found", str(caught.exception))
        self.assertIn("COM11, COM12", str(caught.exception))


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


class TestUsbHostConfig(LabTestCase):
    """The ``usb:`` block is validated where the config is loaded."""

    USB_CONFIG = """
    testbed:
      - id: tb_usb
        platform: board_a
        console: {port: COM8}
        connections:
          - fixture: usb_host
            usb:
              backend: null
              exclude_ports: [COM8, COM6]
              max_file_bytes: 1024
    """

    def test_a_usb_host_connection_parses(self):
        from lab_agent.usb_host.base import parse_usb_options

        config = self.make_config(self.USB_CONFIG)
        connection = config.by_id("tb_usb").connections[0]
        self.assertEqual(connection.fixture.spec, "usb_host")
        options = parse_usb_options(connection.options["usb"])
        # YAML renders an unquoted ``null`` as Python None; it still names the
        # null backend rather than the default.
        self.assertEqual(options.backend, "null")
        self.assertEqual(options.exclude_ports, ("COM8", "COM6"))
        self.assertEqual(options.max_file_bytes, 1024)

    def test_an_unknown_option_is_refused(self):
        text = """
        testbed:
          - id: tb_usb
            platform: board_a
            connections:
              - fixture: usb_host
                usb:
                  backends: auto
        """
        with self.assertRaises(ConfigError) as caught:
            self.make_config(text)
        message = str(caught.exception)
        self.assertIn("backends", message)
        self.assertIn("backend", message)

    def test_a_usb_block_on_the_testbed_is_refused(self):
        text = """
        testbed:
          - id: tb_usb
            platform: board_a
            usb: {backend: null}
        """
        with self.assertRaises(ConfigError) as caught:
            self.make_config(text)
        message = str(caught.exception)
        self.assertIn("tb_usb", message)
        self.assertIn("connection", message)

    def test_a_usb_block_on_another_fixture_is_refused(self):
        text = """
        testbed:
          - id: tb
            platform: board_a
            connections:
              - fixture: bt_peer
                usb: {backend: auto}
        """
        with self.assertRaises(ConfigError) as caught:
            self.make_config(text)
        self.assertIn("usb_host", str(caught.exception))

    def test_a_non_windows_runner_is_warned_about_usb_host(self):
        config = self.make_config(self.USB_CONFIG)
        with mock.patch("lab_agent.config.os.name", "posix"):
            warnings = config.check()
        self.assertTrue(
            any("usb_host" in warning and "501" in warning for warning in warnings),
            warnings,
        )

    def test_a_windows_runner_is_not_warned(self):
        config = self.make_config(self.USB_CONFIG)
        with mock.patch("lab_agent.config.os.name", "nt"):
            warnings = config.check()
        self.assertFalse(any("usb_host" in warning for warning in warnings))


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


class TestContradictionCheck(LabTestCase):
    """A lab wired to a bus the board does not declare is refused.

    The two files are in hand only here: the wiring is this machine's, and
    ``features.yaml`` belongs to the board, in the SDK checkout the agent runs
    from. Refusing at load time means ``--check``, ``--list`` and startup all
    catch it, rather than a test failing to schedule later on for a reason
    nothing in the report can show.
    """

    def declare(self, platform: str, features: str) -> str:
        return self.write(
            f"customer/boards/{platform}/hcpu/features.yaml",
            f"features: {features}\n",
        )

    def a_testbed(self, platform: str, bus: str) -> TestBed:
        return TestBed(
            id="tb_01",
            platform=platform,
            connections=[Connection(FixtureRef("spi_nor_flash", bus))],
        )

    def test_a_connection_to_an_undeclared_bus_is_refused(self):
        self.declare("board_a", "[i2c0]")
        with self.assertRaises(ConfigError) as caught:
            rule_out_contradictions(
                [self.a_testbed("board_a", "spi1")], "cfg.yaml", root=self.root
            )
        message = str(caught.exception)
        self.assertIn("spi1", message)
        self.assertIn("board_a", message)

    def test_a_declared_bus_is_fine(self):
        self.declare("board_a", "[spi1, i2c0]")
        rule_out_contradictions(
            [self.a_testbed("board_a", "spi1")], "cfg.yaml", root=self.root
        )

    def test_an_empty_declaration_contradicts_a_connection(self):
        # An empty declaration is a claim -- "this board has no bus at all" --
        # and a claim can disagree. A missing file is silence, and cannot.
        self.declare("board_a", "[]")
        with self.assertRaises(ConfigError):
            rule_out_contradictions(
                [self.a_testbed("board_a", "spi1")], "cfg.yaml", root=self.root
            )

    def test_a_board_with_no_declaration_says_nothing_to_contradict(self):
        rule_out_contradictions(
            [self.a_testbed("board_a", "spi1")], "cfg.yaml", root=self.root
        )

    def test_the_config_loader_runs_the_check(self):
        # The point of the wiring: a config that is internally consistent but
        # contradicts the board is refused where the config is loaded, not later
        # when some test fails to schedule.
        self.declare("board_a", "[i2c0]")
        with self.assertRaises(ConfigError) as caught:
            parse_runner_config(
                {
                    "testbed": [
                        {
                            "id": "tb_01",
                            "platform": "board_a",
                            "connections": [{"fixture": "spi_nor_flash:spi1"}],
                        }
                    ]
                },
                source="cfg.yaml",
                sdk_root=self.root,
            )
        self.assertIn("spi1", str(caught.exception))


class TestFixtureContractAtLoad(LabTestCase):
    """A binding that carries a credential takes only its own testbed out.

    The agent must keep serving the rest of the bench: one lab entry with a
    password written into it is a mistake to report, not a reason to refuse to
    start.
    """

    BAD_AND_GOOD = """
    testbed:
      - id: tb_bad
        platform: board_a
        console: {port: COM1}
        connections:
          - fixture: wifi_ap:5g
            psk: leaked
      - id: tb_good
        platform: board_a
        console: {port: COM2}
        connections:
          - fixture: spi_nor_flash:spi1
    """

    def test_the_offending_testbed_is_dropped_and_the_other_stays(self):
        config = self.make_config(self.BAD_AND_GOOD)

        self.assertEqual([tb.id for tb in config.testbeds], ["tb_good"])
        self.assertEqual([item[0] for item in config.rejected], ["tb_bad"])
        reason = config.rejected[0][1][0]
        self.assertIn('"psk"', reason)
        self.assertIn("params", reason)
        # And the reason is where a startup / `--check` reader will see it.
        rendered = "\n".join(config.check())
        self.assertIn("tb_bad", rendered)
        self.assertIn("params", rendered)

    def test_an_unknown_connection_key_is_a_warning_not_a_drop(self):
        config = self.make_config(
            """
            testbed:
              - id: tb
                platform: board_a
                console: {port: COM1}
                connections:
                  - fixture: wifi_ap:5g
                    password: typo
            """
        )
        self.assertEqual([tb.id for tb in config.testbeds], ["tb"])
        self.assertEqual(config.rejected, [])
        self.assertEqual(len(config.contract_warnings), 1)
        self.assertIn("password", config.contract_warnings[0])

    def test_a_dropped_testbed_cannot_abort_the_lab_through_its_wiring(self):
        # tb_bad both leaks a credential and is wired to a bus board_a does not
        # declare. It is already out of service, so its contradiction must not
        # stop the agent; tb_good must still load.
        self.write(
            "customer/boards/board_a/hcpu/features.yaml",
            "features: [i2c0]\n",
        )
        self.write(
            "customer/boards/board_b/hcpu/features.yaml",
            "features: [spi1]\n",
        )

        document = {
            "testbed": [
                {
                    "id": "tb_bad",
                    "platform": "board_a",
                    "console": {"port": "COM1"},
                    "connections": [
                        {"fixture": "spi_nor_flash:spi1"},
                        {"fixture": "wifi_ap:5g", "psk": "leaked"},
                    ],
                },
                {
                    "id": "tb_good",
                    "platform": "board_b",
                    "console": {"port": "COM2"},
                    "connections": [{"fixture": "spi_nor_flash:spi1"}],
                },
            ]
        }
        # Pin the premise: on the unfiltered set the wiring check would abort.
        # If it ever stops seeing this, this test must fail rather than pass by
        # testing nothing.
        parsed = parse_binding(document, source="cfg.yaml")
        with self.assertRaises(ConfigError) as caught:
            rule_out_contradictions(parsed, "cfg.yaml", self.root)
        self.assertIn("spi1", str(caught.exception))

        config = parse_runner_config(
            document,
            source="cfg.yaml",
            sdk_root=self.root,
        )

        self.assertEqual([tb.id for tb in config.testbeds], ["tb_good"])
        self.assertEqual([item[0] for item in config.rejected], ["tb_bad"])

    def test_a_remaining_wiring_error_still_names_the_disabled_testbeds(self):
        # Two problems in one config: one testbed is dropped for a credential
        # in its binding, and another still contradicts its board. The operator
        # must see both in one start, not after two restarts.
        self.write(
            "customer/boards/board_a/hcpu/features.yaml",
            "features: [i2c0]\n",
        )
        document = {
            "testbed": [
                {
                    "id": "tb_bad",
                    "platform": "board_a",
                    "console": {"port": "COM1"},
                    "connections": [{"fixture": "wifi_ap:5g", "psk": "leaked"}],
                },
                {
                    "id": "tb_wired",
                    "platform": "board_a",
                    "console": {"port": "COM2"},
                    "connections": [{"fixture": "spi_nor_flash:spi1"}],
                },
            ]
        }

        with self.assertRaises(ConfigError) as caught:
            parse_runner_config(document, source="cfg.yaml", sdk_root=self.root)
        message = str(caught.exception)
        self.assertIn("spi1", message)
        self.assertIn("tb_bad", message)
        self.assertIn("params", message)


if __name__ == "__main__":
    unittest.main()
