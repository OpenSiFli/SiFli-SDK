# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the shared server-class suite steps.

These run against a fake DUT and a fake peer: the helper's job is to turn the
DUT's log text into assertions, and that can be checked without a board.
"""

from __future__ import annotations

import re
import unittest

from _support import SdkTestCase

from sdk_test.suite_helpers import ble_server


class FakeDut:
    """A DUT transcript consumed one matching line at a time."""

    def __init__(self, lines):
        self._lines = list(lines)

    def expect(self, pattern, timeout=10.0):
        regex = re.compile(pattern)
        for index, line in enumerate(self._lines):
            if regex.search(line):
                del self._lines[: index + 1]
                return line
        raise AssertionError(f"{pattern!r} not found in {self._lines!r}")


class FakeService:
    def value_handle(self, index=0):
        return 10


class FakeBle:
    def __init__(self, read_value):
        self.read_value = read_value
        self.written = []

    def write(self, link, handle, payload):
        self.written.append((handle, payload))

    def read(self, link, handle):
        return self.read_value


class FakePeer:
    def __init__(self, read_value=b"\x22\x00\x00\x00"):
        self.ble = FakeBle(read_value)


READY_LINES = [
    "- SiFli Corporation",
    "msh />",
    "receive BLE power on!",
    "[I/ble_app] Broadcast ADV start resutl 0, mode 2",
    "[I/ble_app] ADV start resutl 0, mode 0",
]


class TestExpectReady(SdkTestCase):
    def test_a_broadcast_start_line_is_skipped_for_the_connect_line(self):
        dut = FakeDut(READY_LINES)

        ble_server.expect_ready(dut, expected_mode=0)

    def test_the_expected_mode_is_asserted(self):
        dut = FakeDut(READY_LINES)

        with self.assertRaises(AssertionError) as caught:
            ble_server.expect_ready(dut, expected_mode=5)
        self.assertIn("mode 0", str(caught.exception))


class TestWriteThenRead(SdkTestCase):
    LOG = "[I/ble_app] Updated app value from:0 to:34"

    def test_a_matching_write_log_and_readback_pass(self):
        dut = FakeDut([self.LOG])
        peer = FakePeer()

        ble_server.write_then_read(dut, peer, object(), FakeService())

    def test_the_written_value_must_appear_in_the_log(self):
        dut = FakeDut(["[I/ble_app] Updated app value from:0 to:0"])

        with self.assertRaises(AssertionError):
            ble_server.write_then_read(dut, FakePeer(), object(), FakeService())

    def test_a_fixed_value_server_can_skip_the_readback(self):
        dut = FakeDut([self.LOG])
        peer = FakePeer(read_value=b"\x00\x00\x00\x00")

        ble_server.write_then_read(
            dut, peer, object(), FakeService(), verify_readback=False
        )

    def test_a_different_readback_is_a_failure(self):
        dut = FakeDut([self.LOG])
        peer = FakePeer(read_value=b"\x00\x00\x00\x00")

        with self.assertRaises(AssertionError):
            ble_server.write_then_read(dut, peer, object(), FakeService())


if __name__ == "__main__":
    unittest.main()
