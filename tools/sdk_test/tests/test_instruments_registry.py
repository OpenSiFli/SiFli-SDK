# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the instrument registry.

The pytest plugin creates one fixture per entry at import time, so a bad name
is not a runtime inconvenience -- it is a suite that cannot start. These tests
pin the two mistakes that are easy to make: shadowing a framework fixture and
registering a name pytest cannot turn into a fixture.
"""

from __future__ import annotations

import unittest

from _support import SdkTestCase

from sdk_test.instruments import registry


class TestRegistry(SdkTestCase):
    def test_dut_and_console_are_reserved(self):
        for name in ("dut", "console"):
            with self.assertRaises(ValueError) as caught:
                registry.register_instrument(name, object)
            self.assertIn("collides", str(caught.exception))

    def test_a_name_that_is_not_an_identifier_is_refused(self):
        with self.assertRaises(ValueError) as caught:
            registry.register_instrument("bt peer", object)
        self.assertIn("identifier", str(caught.exception))

    def test_a_valid_registration_is_visible(self):
        class Dummy:
            pass

        registry.register_instrument("dummy_peer", Dummy)
        self.addCleanup(registry.INSTRUMENTS.pop, "dummy_peer", None)

        self.assertIs(registry.INSTRUMENTS["dummy_peer"], Dummy)


if __name__ == "__main__":
    unittest.main()
