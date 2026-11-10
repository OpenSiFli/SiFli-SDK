# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the testbed runtime: allocation, preparation and dispatch.

The exclusivity rule is the one that matters most. Two test programs sharing a
board produce results that look like firmware bugs and are almost impossible to
diagnose after the fact, so it is enforced here rather than trusted to callers.
"""

from __future__ import annotations

import threading
import time
import unittest
from unittest import mock

from _support import CONFIG_YAML  # noqa: F401  (also fixes sys.path)
from _support import FakeJLink
from _support import FakeSerialTransport
from _support import LabTestCase

from lab_agent import testbed as testbed_module
from lab_agent.errors import AllocationError
from lab_agent.errors import LabAgentError
from lab_agent.errors import RelayNotAvailable
from lab_agent.errors import TestbedError
from lab_agent.testbed import LabManager
from lab_agent.testbed import allocate_request_from_mapping


class _SlowCloseTransport(FakeSerialTransport):
    """A port whose close blocks, so a concurrent reopen can be observed."""

    def __init__(self) -> None:
        super().__init__()
        self.closing = threading.Event()
        self.allow_close = threading.Event()

    def close(self) -> None:
        self.closing.set()
        self.allow_close.wait(5)
        super().close()


class TestAllocation(LabTestCase):
    TWO_PEERS_CONFIG = """
    runner:
      name: test-runner
      host: 127.0.0.1
      port: 0
      console_poll: 0.01

    testbed:
      - id: tb_a
        platform: sf32lb52-lcd_n16r8
        console:
          port: COM13
        connections:
          - fixture: ble_peer
            console:
              port: COM99

      - id: tb_b
        platform: sf32lb52-lcd_n16r8
        console:
          port: COM14
        connections:
          - fixture: ble_peer
            console:
              port: COM98
    """

    def test_a_releasing_testbed_is_skipped_by_selection(self):
        # A board being torn down is not free: its per-claim state is still
        # being cleared. Selection must pick another candidate instead of the
        # releasing one and then failing the allocation.
        manager = self.make_manager(self.TWO_PEERS_CONFIG)
        first = manager.allocate(
            allocate_request_from_mapping({"testbed_id": "tb_a"})
        )
        entered = threading.Event()
        finish = threading.Event()
        original = first._teardown

        def slow_teardown():
            entered.set()
            finish.wait(5)
            original()

        first._teardown = slow_teardown  # type: ignore[method-assign]
        release = threading.Thread(target=manager.release, args=("tb_a",))
        release.start()
        try:
            self.assertTrue(entered.wait(2), "release never reached teardown")
            chosen = manager.allocate(
                allocate_request_from_mapping(
                    {"platform": "sf32lb52-lcd_n16r8", "fixtures": ["ble_peer"]}
                )
            )
            self.assertEqual(chosen.testbed.id, "tb_b")
        finally:
            finish.set()
            release.join(5)

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


class TestJLinkDeviceResolution(LabTestCase):
    """Which device name the probe gets, and from where.

    A generated ``download.jlink`` names images and addresses but not the device
    it programs, and that name picks the flash algorithm. So it comes from
    somewhere else, and before the firmware travelled the only somewheres a lab
    on another machine had were its own config and its own checkout -- the
    second being a board file from a possibly different revision, whose wrong
    answer produces a flash that succeeds against the wrong algorithm.

    Driven through ``runtime.jlink()`` rather than through the resolver alone:
    what matters is the name the probe is actually built with.
    """

    def a_runtime(self, platform: str = "my-board", device: str = ""):
        """A testbed whose probe is really constructed, with JLinkExe stubbed."""
        configured = f"                jlink: {{device: {device}}}\n" if device else ""
        manager = self.make_manager(
            f"""
            testbed:
              - id: tb
                platform: {platform}
                console: {{port: COM1}}
{configured}"""
        )
        runtime = manager.get("tb")
        runtime._jlink_factory = None  # take the real construction path
        return runtime

    def the_probe_device(self, runtime) -> str:
        with mock.patch.object(testbed_module, "find_jlink", lambda _path: "JLinkExe"):
            return runtime.jlink().device

    def stage(
        self,
        runtime,
        device: str = "",
        platform: str = "my-board",
        suffix: str = "",
    ) -> None:
        """Put a package in front of the testbed, as a run would."""
        payload = self.a_firmware_package(
            platform=platform, jlink_device=device, suffix=suffix
        )
        runtime.firmware.path(runtime.firmware.stage(payload)["token"])

    def a_board_file(self, device: str) -> None:
        self.write(
            "customer/boards/my-board/hcpu/rtconfig.py",
            f"JLINK_DEVICE = '{device}'\n",
        )

    def test_the_config_wins_when_it_names_one(self):
        # A person who wrote the device into their own config meant it, and they
        # are the one holding the probe.
        self.a_board_file("FROM_RTCONFIG")
        runtime = self.a_runtime(device="FROM_CONFIG")
        self.stage(runtime, "FROM_PACKAGE")
        self.assertEqual(self.the_probe_device(runtime), "FROM_CONFIG")

    def test_the_package_is_used_when_the_config_is_silent(self):
        # The case the package was made to carry: the board file on *this*
        # machine says something else, and this machine is not the one that
        # built the image.
        self.a_board_file("FROM_RTCONFIG")
        runtime = self.a_runtime()
        self.stage(runtime, "FROM_PACKAGE")
        self.assertEqual(self.the_probe_device(runtime), "FROM_PACKAGE")

    def test_the_checkout_is_the_last_resort(self):
        self.a_board_file("FROM_RTCONFIG")
        runtime = self.a_runtime()
        self.assertEqual(self.the_probe_device(runtime), "FROM_RTCONFIG")

    def test_a_package_that_says_nothing_falls_through_to_the_checkout(self):
        # Boards that are not real hardware declare no device, and the package
        # omits the field rather than carrying an empty name.
        self.a_board_file("FROM_RTCONFIG")
        runtime = self.a_runtime()
        self.stage(runtime, "")
        self.assertEqual(self.the_probe_device(runtime), "FROM_RTCONFIG")

    def test_the_next_runs_package_builds_its_own_probe(self):
        # The device travels with the package, so a probe built for one run's
        # image must not be handed to the next run's. Claimed and released the
        # way a run does it, because that is the boundary the probe is dropped
        # at -- a release that had nothing to release is not a run.
        self.a_board_file("FROM_RTCONFIG")
        runtime = self.a_runtime()
        runtime.allocate("job-1")
        self.stage(runtime, "FIRST_PACKAGE")
        self.assertEqual(self.the_probe_device(runtime), "FIRST_PACKAGE")

        runtime.release("job-1")
        runtime.allocate("job-2")
        self.stage(runtime, "SECOND_PACKAGE", suffix="2")
        self.assertEqual(self.the_probe_device(runtime), "SECOND_PACKAGE")

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

#: The same probe-less bench, but with the boot guaranteed by the download
#: rather than left to whoever is standing there. Same no-op, opposite promise.
AUTO_YAML = """
testbed:
  - id: tb
    platform: b
    console: {port: COM1}
    reset:
      method: auto
"""


class TestResetMode(LabTestCase):
    """The two modes that never touch a probe -- and why they are two.

    Both leave the reset to somebody else, but they promise different things,
    and only that difference decides whether an operator is asked and whether a
    harness that writes to the device may run at all. The tests that matter are
    the ones that would let either quietly look like a probed bench: the probe
    must never be built, and the mode must reach the caller that has to tell
    them apart.
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

    def test_auto_reset_does_not_touch_the_probe(self):
        manager = self.make_manager(AUTO_YAML)
        self.assertEqual(manager.get("tb").reset(), "")
        self.assertEqual(manager.probes, {})

    def test_auto_reset_survives_the_prepare_preamble(self):
        # The regression guard in the other direction: `prepare` must not start
        # resetting a bench just because the mode it does not drive changed.
        manager = self.make_manager(AUTO_YAML)
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

    def test_describe_reports_auto_too(self):
        # It has to reach the caller: `auto` and `manual` are the same no-op on
        # this side and opposite behaviour on the other.
        manager = self.make_manager(AUTO_YAML)
        self.assertEqual(manager.get("tb").describe()["reset"], "auto")

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

    def test_an_auto_reset_over_uart_needs_no_probe(self):
        manager = self.make_manager(
            """
            testbed:
              - id: tb
                platform: b
                console: {port: COM1}
                reset: {method: auto}
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


class TestInstrumentConsoles(LabTestCase):
    """A fixture with a console is an instrument the test may drive.

    It is not a testbed: nothing is built or flashed for it, and it is opened
    only when the allocation asked for that fixture -- the alternative is a test
    quietly driving hardware it never declared.
    """

    PEER_CONFIG = """
    runner:
      name: test-runner
      host: 127.0.0.1
      port: 0
      console_poll: 0.01

    testbed:
      - id: tb_peer
        platform: sf32lb52-lcd_n16r8
        console:
          port: COM13
        connections:
          - fixture: ble_peer
            console:
              port: COM99
              baud: 1000000

      - id: tb_peer_no_console
        platform: sf32lb52-lcd_n16r8
        console:
          port: COM14
        connections:
          - fixture: ble_peer
    """

    PINNED_CONFIG = """
    runner:
      name: test-runner
      host: 127.0.0.1
      port: 0
      console_poll: 0.01

    testbed:
      - id: tb_peer
        platform: sf32lb52-lcd_n16r8
        console:
          port: COM13
        connections:
          - fixture: ble_peer@peer1
            console:
              port: COM99
              baud: 1000000
    """

    def test_two_spellings_of_one_instrument_share_one_stream(self):
        # The bare name and the pinned instance are the same physical port.
        # Opening both would put two pump threads on one serial port, splitting
        # or dropping bytes between them.
        manager = self.make_manager(self.PINNED_CONFIG)
        runtime = manager.allocate(
            allocate_request_from_mapping(
                {"testbed_id": "tb_peer", "fixtures": ["ble_peer"]}
            )
        )
        runtime.fixture_console_open("ble_peer")
        runtime.fixture_console_open("ble_peer@peer1")

        self.assertEqual(runtime.describe()["fixture_consoles"], ["ble_peer@peer1"])
        self.assertEqual(len(manager.fixture_transports), 1)

        # And closing by either spelling closes that one stream.
        runtime.fixture_console_close("ble_peer")
        self.assertEqual(manager.fixture_transports["ble_peer@peer1"].closed, 1)

    def test_a_pinned_instance_that_does_not_exist_is_not_substituted(self):
        # `ble_peer@peer2` must not reach peer1 because they share a name: a
        # pinned spec is a claim about which instance, and silently choosing
        # another would drive the wrong instrument.
        manager = self.make_manager(self.PINNED_CONFIG)
        runtime = manager.allocate(
            allocate_request_from_mapping(
                {"testbed_id": "tb_peer", "fixtures": ["ble_peer"]}
            )
        )
        with self.assertRaises(TestbedError) as caught:
            runtime.fixture_console_open("ble_peer@peer2")
        message = str(caught.exception)
        self.assertIn("ble_peer@peer2", message)
        self.assertIn("ble_peer@peer1", message)  # says what does exist

    SUBSET_CONFIG = """
    runner:
      name: test-runner
      host: 127.0.0.1
      port: 0
      console_poll: 0.01

    testbed:
      - id: tb_peer
        platform: sf32lb52-lcd_n16r8
        console:
          port: COM13
        connections:
          - fixture: ble_peer:ble
            console:
              port: COM99
          - fixture: ble_peer:ble,classic
            console:
              port: COM98
    """

    def test_a_case_asking_for_less_uses_the_exact_wiring(self):
        # Both connections satisfy `ble_peer:ble`, so the exact one is chosen:
        # a lab that wires a dual-mode peer next to a BLE-only one must not have
        # its BLE-only case silently routed to the other port.
        manager = self.make_manager(self.SUBSET_CONFIG)
        runtime = manager.allocate(
            allocate_request_from_mapping(
                {"testbed_id": "tb_peer", "fixtures": ["ble_peer:ble"]}
            )
        )

        runtime.fixture_console_open("ble_peer:ble")

        self.assertEqual(runtime.describe()["fixture_consoles"], ["ble_peer:ble"])

    def test_a_case_asking_for_more_gets_the_richer_peer(self):
        manager = self.make_manager(self.SUBSET_CONFIG)
        runtime = manager.allocate(
            allocate_request_from_mapping(
                {"testbed_id": "tb_peer", "fixtures": ["ble_peer:ble,classic"]}
            )
        )

        runtime.fixture_console_open("ble_peer:ble,classic")

        self.assertEqual(
            runtime.describe()["fixture_consoles"], ["ble_peer:ble,classic"]
        )

    def test_an_instrument_cannot_be_opened_after_the_testbed_was_released(self):
        # The HTTP server answers on a thread per request, so an open can
        # arrive after the release it raced with; it must be refused rather
        # than leave a port open on a testbed nobody owns.
        manager = self.make_manager(self.PINNED_CONFIG)
        runtime = manager.allocate(
            allocate_request_from_mapping(
                {"testbed_id": "tb_peer", "fixtures": ["ble_peer"]}
            )
        )
        manager.release("tb_peer")

        with self.assertRaises(TestbedError) as caught:
            runtime.fixture_console_open("ble_peer")
        self.assertIn("not allocated", str(caught.exception))

    def test_a_new_claim_waits_for_the_previous_release_to_finish(self):
        # The holder is cleared before teardown, so a claim that arrives during
        # teardown would have its own requested-fixtures/firmware wiped by the
        # old one's cleanup. Allocation is refused until the release settles.
        manager = self.make_manager(self.PINNED_CONFIG)
        runtime = manager.allocate(
            allocate_request_from_mapping({"testbed_id": "tb_peer"})
        )
        entered = threading.Event()
        finish = threading.Event()
        original = runtime._teardown

        def slow_teardown():
            entered.set()
            finish.wait(5)
            original()

        runtime._teardown = slow_teardown  # type: ignore[method-assign]
        release = threading.Thread(target=manager.release, args=("tb_peer",))
        release.start()
        try:
            self.assertTrue(entered.wait(2), "release never reached teardown")
            with self.assertRaises(AllocationError) as caught:
                manager.allocate(
                    allocate_request_from_mapping({"testbed_id": "tb_peer"})
                )
            self.assertIn("still being released", str(caught.exception))
        finally:
            finish.set()
            release.join(5)

        claimed = manager.allocate(
            allocate_request_from_mapping({"testbed_id": "tb_peer"})
        )
        self.assertTrue(claimed.busy)

    def test_a_malformed_instrument_name_is_the_callers_error(self):
        # `parse_fixture_ref(":peer1")` raises BindingError, which the server
        # does not map: it would surface as a 500 and a server stack trace.
        manager = self.make_manager(self.PEER_CONFIG)
        runtime = manager.allocate(
            allocate_request_from_mapping(
                {"testbed_id": "tb_peer", "fixtures": ["ble_peer"]}
            )
        )
        with self.assertRaises(LabAgentError) as caught:
            runtime.fixture_console_open(":peer1")
        self.assertIn("not a fixture name", str(caught.exception))

    def test_reopening_waits_until_the_old_port_is_closed(self):
        # ConsoleStream.stop() closes the port last; a reopen that slipped in
        # after the map entry was removed would find the port still busy.
        transports = []
        created = threading.Event()

        def factory(testbed, connection):
            transport = _SlowCloseTransport()
            transports.append(transport)
            created.set()
            return transport

        manager = LabManager(
            self.make_config(self.PINNED_CONFIG),
            sdk_root=self.root,
            console_factory=lambda tb: FakeSerialTransport(),
            jlink_factory=lambda tb, settings: FakeJLink(),
            fixture_console_factory=factory,
        )
        runtime = manager.allocate(
            allocate_request_from_mapping(
                {"testbed_id": "tb_peer", "fixtures": ["ble_peer"]}
            )
        )
        runtime.fixture_console_open("ble_peer")
        created.clear()

        closing = threading.Thread(
            target=runtime.fixture_console_close, args=("ble_peer",)
        )
        opening = threading.Thread(
            target=runtime.fixture_console_open, args=("ble_peer",)
        )
        closing.start()
        try:
            self.assertTrue(transports[0].closing.wait(2), "close never started")
            opening.start()
            opening.join(0.2)
            self.assertTrue(
                opening.is_alive(),
                "a reopen must wait until the old stream has released the port",
            )
            self.assertFalse(
                created.is_set(),
                "the reopen built a transport while the old port was still closing",
            )
        finally:
            transports[0].allow_close.set()
            closing.join(5)
            if opening.ident is not None:
                opening.join(5)

        self.assertFalse(opening.is_alive())
        self.assertTrue(created.is_set())
        self.assertEqual(len(transports), 2)

    def test_the_stream_is_refused_until_the_allocation_declares_it(self):
        manager = self.make_manager(self.PEER_CONFIG)
        runtime = manager.allocate(
            allocate_request_from_mapping({"testbed_id": "tb_peer"})
        )
        with self.assertRaises(TestbedError) as caught:
            runtime.fixture_console_open("ble_peer")
        message = str(caught.exception)
        self.assertIn("not requested", message)
        self.assertIn("ble_peer", message)

    def test_an_instrument_console_round_trips_text(self):
        manager = self.make_manager(
            self.PEER_CONFIG, fixture_script="SIFLI_APP-aa-bb\r\n"
        )
        runtime = manager.allocate(
            allocate_request_from_mapping(
                {"testbed_id": "tb_peer", "fixtures": ["ble_peer"]}
            )
        )
        runtime.fixture_console_open("ble_peer")

        data, _cursor = runtime.fixture_console_read("ble_peer", 0, 1.0)
        self.assertIn("SIFLI_APP-aa-bb", data)

        runtime.fixture_console_write("ble_peer", "diss help\r\n")
        self.assertIn("diss help", manager.fixture_transports["ble_peer"].sent)
        self.assertEqual(runtime.describe()["fixture_consoles"], ["ble_peer"])

    def test_reading_an_unopened_instrument_stream_is_reported(self):
        manager = self.make_manager(self.PEER_CONFIG)
        runtime = manager.allocate(
            allocate_request_from_mapping(
                {"testbed_id": "tb_peer", "fixtures": ["ble_peer"]}
            )
        )
        with self.assertRaises(TestbedError) as caught:
            runtime.fixture_console_read("ble_peer", 0, 0.1)
        self.assertIn("not open", str(caught.exception))

    def test_release_closes_the_instrument_console(self):
        manager = self.make_manager(self.PEER_CONFIG)
        runtime = manager.allocate(
            allocate_request_from_mapping(
                {"testbed_id": "tb_peer", "fixtures": ["ble_peer"]}
            )
        )
        runtime.fixture_console_open("ble_peer")
        manager.release("tb_peer")

        self.assertEqual(runtime.describe()["fixture_consoles"], [])
        self.assertEqual(manager.fixture_transports["ble_peer"].closed, 1)

    def test_a_fixture_without_a_console_says_so(self):
        manager = self.make_manager(self.PEER_CONFIG)
        runtime = manager.allocate(
            allocate_request_from_mapping(
                {"testbed_id": "tb_peer_no_console", "fixtures": ["ble_peer"]}
            )
        )
        with self.assertRaises(TestbedError) as caught:
            runtime.fixture_console_open("ble_peer")
        self.assertIn("no console configured", str(caught.exception))

    def test_an_unknown_fixture_lists_what_the_testbed_offers(self):
        manager = self.make_manager(self.PEER_CONFIG)
        runtime = manager.allocate(
            allocate_request_from_mapping(
                {"testbed_id": "tb_peer", "fixtures": ["ble_peer"]}
            )
        )
        with self.assertRaises(TestbedError) as caught:
            runtime.fixture_console_open("no_such_instrument")
        self.assertIn("ble_peer", str(caught.exception))


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


class TestProbeMemory(LabTestCase):
    """Memory reads through the testbed's probe.

    Three properties the transport cannot enforce on its own: an empty
    ``jlink: {}`` is a declaration, a testbed with no probe is refused, and one
    testbed never drives two J-Link processes at once.
    """

    EMPTY_JLINK_CONFIG = """
runner:
  name: test-runner
  host: 127.0.0.1
  port: 0
  console_poll: 0.01

testbed:
  - id: tb_probe
    platform: sf32lb52-lcd_n16r8
    console:
      port: COM13
    jlink: {}
    flash:
      method: sftool
    reset:
      method: auto
"""

    def test_an_empty_jlink_block_declares_a_probe(self):
        # `jlink: {}` means "the probe is here, defaults apply". Testing the
        # options for truthiness would refuse a declaration the README says is
        # enough.
        manager = self.make_manager(self.EMPTY_JLINK_CONFIG)
        runtime = manager.get("tb_probe")
        data = runtime.read_memory(0x20000000, 4)
        self.assertEqual(data, bytes((0x20000000 + i) & 0xFF for i in range(4)))
        self.assertEqual(manager.probes["tb_probe"].memory_reads, [(0x20000000, 4)])

    def test_without_a_declared_probe_the_read_is_refused(self):
        yaml_text = self.EMPTY_JLINK_CONFIG.replace("    jlink: {}\n", "")
        manager = self.make_manager(yaml_text)
        with self.assertRaises(TestbedError) as caught:
            manager.get("tb_probe").read_memory(0x20000000, 4)
        self.assertIn("jlink", str(caught.exception))

    def test_reads_on_one_testbed_are_serialized(self):
        probe = _ConcurrencyProbe()
        manager = LabManager(
            self.make_config(),
            sdk_root=self.root,
            console_factory=lambda testbed: FakeSerialTransport(),
            jlink_factory=lambda testbed, settings: probe,
        )
        runtime = manager.get("tb_01")
        threads = [
            threading.Thread(target=runtime.read_memory, args=(0x20000000, 4))
            for _ in range(4)
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=5)
        self.assertEqual(probe.max_active, 1)


class _ConcurrencyProbe:
    """Records how many reads are inside the probe at once."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.active = 0
        self.max_active = 0

    def read_memory(self, address: int, length: int) -> bytes:
        with self._lock:
            self.active += 1
            self.max_active = max(self.max_active, self.active)
        time.sleep(0.05)
        with self._lock:
            self.active -= 1
        return bytes(length)


if __name__ == "__main__":
    unittest.main()
