# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the testbed runtime: allocation, preparation and dispatch.

The exclusivity rule is the one that matters most. Two test programs sharing a
board produce results that look like firmware bugs and are almost impossible to
diagnose after the fact, so it is enforced here rather than trusted to callers.
"""

from __future__ import annotations

import unittest

from _support import CONFIG_YAML  # noqa: F401  (also fixes sys.path)
from _support import LabTestCase

from lab_agent.errors import AllocationError
from lab_agent.errors import RelayNotAvailable
from lab_agent.errors import TestbedError
from lab_agent.testbed import allocate_request_from_mapping
from lab_agent.testbed import board_jlink_device


class TestAllocation(LabTestCase):
    def test_a_testbed_has_exactly_one_holder(self):
        manager = self.make_manager()
        runtime = manager.allocate(
            allocate_request_from_mapping({"testbed_id": "tb_01", "holder": "job-1"})
        )
        self.assertTrue(runtime.busy)
        self.assertEqual(runtime.holder, "job-1")

    def test_a_second_claim_is_refused_and_names_the_holder(self):
        manager = self.make_manager()
        manager.allocate(
            allocate_request_from_mapping({"testbed_id": "tb_01", "holder": "job-1"})
        )
        with self.assertRaises(AllocationError) as caught:
            manager.allocate(
                allocate_request_from_mapping({"testbed_id": "tb_01", "holder": "job-2"})
            )
        message = str(caught.exception)
        self.assertIn("job-1", message)
        self.assertIn("tb_01", message)

    def test_release_frees_the_testbed_for_the_next_job(self):
        manager = self.make_manager()
        manager.allocate(allocate_request_from_mapping({"testbed_id": "tb_01", "holder": "job-1"}))
        manager.release("tb_01", "job-1")
        runtime = manager.allocate(
            allocate_request_from_mapping({"testbed_id": "tb_01", "holder": "job-2"})
        )
        self.assertEqual(runtime.holder, "job-2")

    def test_release_by_a_different_holder_is_refused(self):
        manager = self.make_manager()
        manager.allocate(allocate_request_from_mapping({"testbed_id": "tb_01", "holder": "job-1"}))
        with self.assertRaises(AllocationError):
            manager.release("tb_01", "job-2")
        self.assertTrue(manager.get("tb_01").busy, "a refused release must not free it")

    def test_release_is_idempotent(self):
        manager = self.make_manager()
        manager.allocate(allocate_request_from_mapping({"testbed_id": "tb_01"}))
        manager.release("tb_01")
        manager.release("tb_01")  # must not raise
        self.assertFalse(manager.get("tb_01").busy)

    def test_release_closes_the_console(self):
        manager = self.make_manager()
        runtime = manager.allocate(allocate_request_from_mapping({"testbed_id": "tb_01"}))
        runtime.console_open()
        self.assertTrue(runtime.describe()["console_open"])

        manager.release("tb_01")
        self.assertFalse(runtime.describe()["console_open"])
        self.assertEqual(manager.transports["tb_01"].closed, 1)

    def test_an_unknown_testbed_id_is_reported(self):
        manager = self.make_manager()
        with self.assertRaises(TestbedError) as caught:
            manager.get("no_such_testbed")
        self.assertIn("tb_01", str(caught.exception))  # says what does exist


class TestCapabilityMatching(LabTestCase):
    def test_a_request_is_matched_on_fixture(self):
        manager = self.make_manager()
        runtime = manager.allocate(
            allocate_request_from_mapping({"fixtures": ["i2c_sensor"], "holder": "j"})
        )
        self.assertEqual(runtime.testbed.id, "tb_01")

    def test_capabilities_in_a_request_are_refused(self):
        # What a platform can do was settled before a testbed was asked for, by
        # the board's own features.yaml. Accepting the key here would be a second
        # place to state it, which is what the split exists to prevent.
        with self.assertRaises(AllocationError) as caught:
            allocate_request_from_mapping({"capabilities": ["i2c"], "holder": "j"})
        self.assertIn("features.yaml", str(caught.exception))

    def test_a_request_is_matched_on_platform(self):
        manager = self.make_manager()
        runtime = manager.allocate(
            allocate_request_from_mapping({"platform": "eh-lb563", "holder": "j"})
        )
        self.assertEqual(runtime.testbed.id, "tb_02")

    def test_two_testbeds_offering_the_same_fixture_are_interchangeable(self):
        # Both testbeds offer spi_nor_flash:spi1, so the first in binding order
        # wins, and the second is used once the first is taken.
        manager = self.make_manager()
        first = manager.allocate(
            allocate_request_from_mapping({"fixtures": ["spi_nor_flash:spi1"], "holder": "j1"})
        )
        second = manager.allocate(
            allocate_request_from_mapping({"fixtures": ["spi_nor_flash:spi1"], "holder": "j2"})
        )
        self.assertEqual((first.testbed.id, second.testbed.id), ("tb_01", "tb_02"))

    def test_an_unsatisfiable_request_explains_why(self):
        manager = self.make_manager()
        with self.assertRaises(AllocationError) as caught:
            manager.allocate(
                allocate_request_from_mapping(
                    {"fixtures": ["spi_nor_flash:spi9"], "holder": "j"}
                )
            )
        message = str(caught.exception)
        self.assertIn("spi_nor_flash:spi9", message)
        self.assertIn("offers", message)

    def test_all_candidates_busy_is_reported_as_such(self):
        manager = self.make_manager()
        manager.allocate(
            allocate_request_from_mapping({"fixtures": ["spi_nor_flash:spi1"], "holder": "j1"})
        )
        manager.allocate(
            allocate_request_from_mapping({"fixtures": ["spi_nor_flash:spi1"], "holder": "j2"})
        )
        with self.assertRaises(AllocationError) as caught:
            manager.allocate(
                allocate_request_from_mapping({"fixtures": ["spi_nor_flash:spi1"], "holder": "j3"})
            )
        self.assertIn("busy", str(caught.exception))

    def test_a_scalar_is_accepted_where_a_list_is_expected(self):
        request = allocate_request_from_mapping({"fixtures": "i2c_sensor"})
        self.assertEqual(request.fixtures, ["i2c_sensor"])

    def test_a_bad_field_type_is_rejected(self):
        with self.assertRaises(AllocationError):
            allocate_request_from_mapping({"fixtures": {"not": "a list"}})


class TestBoardMetadata(LabTestCase):
    def test_the_probe_device_is_read_from_the_boards_own_rtconfig(self):
        self.write(
            "customer/boards/my-board/hcpu/rtconfig.py",
            """
            import os
            TARGET_NAME = 'bf0_ap'
            JLINK_DEVICE = 'SF32LB52X_NOR'
            CORE = "HCPU"
            """,
        )
        self.assertEqual(
            board_jlink_device("my-board", sdk_root=self.root), "SF32LB52X_NOR"
        )

    def test_an_empty_jlink_device_is_not_offered_as_a_name(self):
        # Boards that are not real hardware (pc, dummy) declare an empty value.
        self.write(
            "customer/boards/pc/hcpu/rtconfig.py",
            "JLINK_DEVICE = ''\n",
        )
        self.assertIsNone(board_jlink_device("pc", sdk_root=self.root))

    def test_a_missing_board_yields_nothing(self):
        self.assertIsNone(board_jlink_device("no-such-board", sdk_root=self.root))

    def test_the_derived_device_reaches_the_probe(self):
        self.write(
            "customer/boards/my-board/hcpu/rtconfig.py",
            "JLINK_DEVICE = 'SF32LB56X_NAND'\n",
        )
        manager = self.make_manager(
            """
            testbed:
              - id: tb
                platform: my-board
                console:
                  port: COM1
            """
        )
        runtime = manager.get("tb")
        runtime._jlink_factory = None  # use the real construction path
        options = runtime._jlink_options()
        self.assertEqual(options, {})
        self.assertEqual(
            board_jlink_device(runtime.testbed.platform, runtime.sdk_root),
            "SF32LB56X_NAND",
        )

    def test_a_testbed_without_a_probe_and_without_board_metadata_fails_clearly(self):
        manager = self.make_manager(
            """
            testbed:
              - id: tb
                platform: unknown-board
                console:
                  port: COM1
            """
        )
        runtime = manager.get("tb")
        runtime._jlink_factory = None
        with self.assertRaises(TestbedError) as caught:
            runtime.jlink()
        self.assertIn("unknown-board", str(caught.exception))


class TestFlashDispatch(LabTestCase):
    def test_jlink_is_the_default_transport(self):
        manager = self.make_manager(
            """
            testbed:
              - id: tb
                platform: b
                console: {port: COM1}
            """
        )
        manager.get("tb").flash("/tmp/build")
        self.assertEqual(manager.probes["tb"].flashed, ["/tmp/build"])

    def test_an_explicit_jlink_method_uses_the_probe(self):
        manager = self.make_manager()
        manager.get("tb_01").flash("/tmp/build")
        self.assertEqual(manager.probes["tb_01"].flashed, ["/tmp/build"])

    def test_a_missing_build_dir_is_refused(self):
        manager = self.make_manager()
        with self.assertRaises(TestbedError):
            manager.get("tb_01").flash("")

    def test_an_unknown_method_is_refused_with_the_valid_ones(self):
        manager = self.make_manager(
            """
            testbed:
              - id: tb
                platform: b
                console: {port: COM1}
                flash: {method: telepathy}
            """
        )
        with self.assertRaises(TestbedError) as caught:
            manager.get("tb").flash("/tmp/build")
        self.assertIn("telepathy", str(caught.exception))
        self.assertIn("sftool", str(caught.exception))

    def test_sftool_flashing_refuses_while_the_console_holds_the_port(self):
        # The download port IS the console port; pretending otherwise would
        # produce a port conflict or a corrupt download.
        manager = self.make_manager(
            """
            testbed:
              - id: tb
                platform: b
                console: {port: COM1}
                flash: {method: sftool}
            """
        )
        runtime = manager.get("tb")
        runtime.console_open()
        with self.assertRaises(TestbedError) as caught:
            runtime.flash("/tmp/build")
        self.assertIn("close the console", str(caught.exception))

    def test_sftool_flashing_needs_a_console_port_to_download_over(self):
        manager = self.make_manager(
            """
            testbed:
              - id: tb
                platform: b
                flash: {method: sftool}
            """
        )
        with self.assertRaises(TestbedError) as caught:
            manager.get("tb").flash("/tmp/build")
        self.assertIn("download port", str(caught.exception))


class TestPreparation(LabTestCase):
    def test_prepare_flashes_then_resets_in_that_order(self):
        manager = self.make_manager()
        runtime = manager.get("tb_01")
        runtime.prepare(build_dir="/tmp/build")

        # Flash must precede reset: resetting first would boot the old image.
        self.assertEqual(manager.probes["tb_01"].flashed, ["/tmp/build"])
        self.assertEqual(manager.probes["tb_01"].resets, [None])

    def test_prepare_without_a_build_dir_still_resets(self):
        manager = self.make_manager()
        runtime = manager.get("tb_01")
        runtime.prepare()
        self.assertEqual(manager.probes["tb_01"].flashed, [])
        self.assertEqual(len(manager.probes["tb_01"].resets), 1)

    def test_prepare_can_skip_the_reset(self):
        manager = self.make_manager()
        manager.get("tb_01").prepare(build_dir="/tmp/build", reset=False)
        self.assertEqual(manager.probes["tb_01"].flashed, ["/tmp/build"])
        self.assertEqual(manager.probes["tb_01"].resets, [])

    def test_the_watchdog_script_is_passed_through_when_configured(self):
        manager = self.make_manager(
            """
            testbed:
              - id: tb
                platform: b
                console: {port: COM1}
                jlink: {wdt_script: tools/segger/halt_all_cpu_and_disable_all_wdt_56x.jlink}
            """
        )
        manager.get("tb").reset()
        self.assertEqual(
            manager.probes["tb"].resets,
            ["tools/segger/halt_all_cpu_and_disable_all_wdt_56x.jlink"],
        )


MANUAL_YAML = """
testbed:
  - id: tb
    platform: b
    console: {port: COM1}
    reset:
      method: manual
"""


class TestResetMode(LabTestCase):
    """A board with no probe: a person presses the button instead.

    The tests that matter here are the ones that would let a manual testbed
    quietly look like a probed one -- the probe must never be built, and the
    mode must reach the caller that has to prompt for the button.
    """

    def test_manual_reset_does_not_touch_the_probe(self):
        manager = self.make_manager(MANUAL_YAML)
        self.assertEqual(manager.get("tb").reset(), "")
        # The whole point: no probe is constructed, so a lab without J-Link
        # installed can still run this board.
        self.assertEqual(manager.probes, {})

    def test_manual_reset_survives_the_prepare_preamble(self):
        manager = self.make_manager(MANUAL_YAML)
        manager.get("tb").prepare()
        self.assertEqual(manager.probes, {})

    def test_the_default_is_still_the_probe(self):
        # The regression guard: adding a mode must not make resetting opt-in.
        manager = self.make_manager()
        manager.get("tb_01").reset()
        self.assertEqual(len(manager.probes["tb_01"].resets), 1)

    def test_an_unknown_reset_method_is_refused(self):
        manager = self.make_manager(
            """
            testbed:
              - id: tb
                platform: b
                console: {port: COM1}
                reset: {method: telepathy}
            """
        )
        with self.assertRaises(TestbedError) as caught:
            manager.get("tb").reset()
        self.assertIn("telepathy", str(caught.exception))
        self.assertIn("tb", str(caught.exception))

    def test_a_non_mapping_reset_block_is_refused(self):
        manager = self.make_manager(
            """
            testbed:
              - id: tb
                platform: b
                console: {port: COM1}
                reset: manual
            """
        )
        with self.assertRaises(TestbedError) as caught:
            manager.get("tb").reset()
        self.assertIn("must be a mapping", str(caught.exception))

    def test_describe_reports_the_mode(self):
        manager = self.make_manager(MANUAL_YAML)
        self.assertEqual(manager.get("tb").describe()["reset"], "manual")

    def test_describe_reports_a_typo_instead_of_failing(self):
        # describe() feeds GET /testbeds and the allocation. A typo must not
        # take those down -- the SDK side would report an unreachable agent,
        # which points nowhere near the actual mistake.
        manager = self.make_manager(
            """
            testbed:
              - id: tb
                platform: b
                console: {port: COM1}
                reset: {method: telepathy}
            """
        )
        self.assertEqual(manager.get("tb").describe()["reset"], "telepathy")
        with self.assertRaises(TestbedError):
            manager.get("tb").reset_mode()

    def test_describe_survives_a_non_mapping_reset_block(self):
        manager = self.make_manager(
            """
            testbed:
              - id: tb
                platform: b
                console: {port: COM1}
                reset: manual
            """
        )
        self.assertEqual(manager.get("tb").describe()["reset"], "jlink")


class TestNeedsJlink(LabTestCase):
    """Whether the probe is needed at all, decided from both halves.

    Flashing and resetting are configured independently, so the predicate has
    to look at both -- a manual reset must not hide a board that still needs
    the probe to flash.
    """

    def test_a_manual_reset_over_uart_needs_no_probe(self):
        manager = self.make_manager(
            """
            testbed:
              - id: tb
                platform: b
                console: {port: COM1}
                reset: {method: manual}
                flash: {method: sftool}
            """
        )
        self.assertFalse(manager.get("tb").needs_jlink())

    def test_a_manual_reset_still_needs_the_probe_to_flash(self):
        manager = self.make_manager(MANUAL_YAML)
        self.assertTrue(manager.get("tb").needs_jlink())

    def test_a_probe_reset_over_uart_still_needs_the_probe(self):
        manager = self.make_manager(
            """
            testbed:
              - id: tb
                platform: b
                console: {port: COM1}
                flash: {method: sftool}
            """
        )
        self.assertTrue(manager.get("tb").needs_jlink())


class TestConsoleAccess(LabTestCase):
    def test_reading_before_opening_is_refused(self):
        manager = self.make_manager()
        with self.assertRaises(TestbedError) as caught:
            manager.get("tb_01").console_read(0, 0.0)
        self.assertIn("not open", str(caught.exception))

    def test_a_testbed_without_a_console_port_cannot_open_one(self):
        manager = self.make_manager(
            """
            testbed:
              - id: tb
                platform: b
            """
        )
        with self.assertRaises(TestbedError) as caught:
            manager.get("tb").console_open()
        self.assertIn("no console port", str(caught.exception))

    def test_open_is_idempotent(self):
        manager = self.make_manager()
        runtime = manager.get("tb_01")
        runtime.console_open()
        runtime.console_open()
        self.assertEqual(manager.transports["tb_01"].opened, 1)

    def test_console_write_reaches_the_port(self):
        manager = self.make_manager()
        runtime = manager.get("tb_01")
        runtime.console_open()
        runtime.console_send_line("unity_run lz4")
        self.assertEqual(manager.transports["tb_01"].sent, "unity_run lz4\r\n")


class TestDescribe(LabTestCase):
    def test_describe_reports_what_a_control_plane_needs(self):
        manager = self.make_manager()
        described = {item["id"]: item for item in manager.list()}

        first = described["tb_01"]
        self.assertEqual(first["platform"], "sf32lb52-lcd_n16r8")
        self.assertEqual(first["console"], {"port": "COM13", "baud": 1000000})
        self.assertEqual(first["fixtures"], ["spi_nor_flash:spi1", "i2c_sensor"])
        # Not "capabilities": a board's own declaration is the only place that
        # lives, and the agent was never the right place to ask.
        self.assertNotIn("capabilities", first)
        self.assertFalse(first["busy"])
        self.assertIsNone(first["holder"])
        # A control plane must be able to tell a lab that can power-cycle a
        # wedged board from one that cannot.
        self.assertFalse(first["relay_available"])

    def test_describe_reflects_allocation(self):
        manager = self.make_manager()
        manager.allocate(allocate_request_from_mapping({"testbed_id": "tb_01", "holder": "j1"}))
        described = {item["id"]: item for item in manager.list()}
        self.assertTrue(described["tb_01"]["busy"])
        self.assertEqual(described["tb_01"]["holder"], "j1")
        self.assertFalse(described["tb_02"]["busy"])

    def test_shutdown_releases_everything(self):
        manager = self.make_manager()
        manager.allocate(allocate_request_from_mapping({"testbed_id": "tb_01"}))
        manager.allocate(allocate_request_from_mapping({"testbed_id": "tb_02"}))
        manager.shutdown()
        self.assertFalse(any(item["busy"] for item in manager.list()))


class TestRelay(LabTestCase):
    def test_relay_control_fails_loudly_rather_than_silently_doing_nothing(self):
        # A silent no-op would let a test "pass" without the power cycle it
        # asked for, which is worse than failing.
        from lab_agent.transport.relay import RelayTransport

        relay = RelayTransport("mux_01", channels=4)
        self.assertFalse(relay.available)
        for call in (relay.power_on, relay.power_off, relay.power_cycle, relay.mux_release):
            with self.assertRaises(RelayNotAvailable):
                call()
        with self.assertRaises(RelayNotAvailable):
            relay.mux_select(1)


if __name__ == "__main__":
    unittest.main()
