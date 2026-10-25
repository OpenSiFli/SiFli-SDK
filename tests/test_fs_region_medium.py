#!/usr/bin/env python3
#
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
#
# SPDX-License-Identifier: Apache-2.0

"""Tests for resolving the fs_region medium of v1/v2 partition tables.

The base address alone cannot tell NOR from NAND: the 55x series maps NOR XIP
flash at 0x64000000/0x68000000, inside the address range the legacy heuristic
treated as NAND. The board QSPI mode must take precedence there.
"""

from __future__ import annotations

import os
import sys
import types
import unittest
from unittest import mock


ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "tools", "build"))


def _install_scons_stubs():
    """Let building.py import when SCons is not installed (unit-test runs)."""
    scons = types.ModuleType("SCons")
    scons.__path__ = []
    script = types.ModuleType("SCons.Script")
    cpp = types.ModuleType("SCons.cpp")

    class _PreProcessor:
        pass

    cpp.PreProcessor = _PreProcessor
    # building.py uses the SCons package object, which the real
    # SCons.Script exports to modules doing ``from SCons.Script import *``.
    script.SCons = scons
    scons.Script = script
    scons.cpp = cpp
    sys.modules.setdefault("SCons", scons)
    sys.modules.setdefault("SCons.Script", script)
    sys.modules.setdefault("SCons.cpp", cpp)


try:
    import SCons.cpp  # noqa: F401
except ImportError:
    _install_scons_stubs()

import building  # noqa: E402
import ptab  # noqa: E402


class _FakeEnv:
    def __init__(self, ptab_obj):
        self._ptab_obj = ptab_obj

    def GetPtab(self):
        return self._ptab_obj


class _FakePtab:
    def __init__(self, mems):
        self._mems = mems

    def is_v3(self):
        return False

    def content_mems(self):
        return self._mems


def _qspi_mode_mocks(mode):
    """Emulate the Kconfig accessors for a given BSP_QSPI<N>_MODE value."""

    def get_depend(name):
        return mode is not None and name.endswith("_MODE_{}".format(mode))

    def get_config_value(_name):
        return "" if mode is None else mode

    return get_depend, get_config_value


class FsRegionMediumTests(unittest.TestCase):
    def _medium(self, ptab_obj, mode):
        get_depend, get_config_value = _qspi_mode_mocks(mode)
        with mock.patch.object(building, "GetDepend", side_effect=get_depend), \
                mock.patch.object(building, "GetConfigValue", side_effect=get_config_value):
            return building._get_fs_region_medium(_FakeEnv(ptab_obj))

    def _load(self, rel_path):
        return ptab.load_ptab(os.path.join(ROOT, rel_path), fatal=True)

    def test_55x_nor_xip_flash_is_nor(self):
        # eh-lb551 fs_region lives on flash2, base 0x64000000 (NOR XIP).
        ptab_obj = self._load("customer/boards/eh-lb551/ptab.json")
        self.assertEqual(self._medium(ptab_obj, 0), "nor")

    def test_52x_nand_is_nand(self):
        # sf32lb52-nano fs_region lives on flash2, base 0x62000000 (NAND).
        ptab_obj = self._load(
            "example/bt/pan_ota/project/sf32lb52-nano_a128r16_hcpu/ptab.json")
        self.assertEqual(self._medium(ptab_obj, 1), "nand")

    def test_57x_nand_is_nand(self):
        # dpi-hdk_lb575 fs_region lives on flash3, base 0x64000000 (NAND).
        ptab_obj = self._load(
            "customer/boards/dpi-hdk_lb575ucnn6_a128_qspi/ptab.json")
        self.assertEqual(self._medium(ptab_obj, 1), "nand")

    def test_missing_mode_falls_back_to_base_address(self):
        cases = (
            ("0x64000000", "nand"),
            ("0x62000000", "nand"),
            ("0x10000000", "nor"),
        )
        for base, expected in cases:
            with self.subTest(base=base):
                fake = _FakePtab([{
                    "mem": "flash2",
                    "base": base,
                    "regions": [{"tags": ["FS_REGION"]}],
                }])
                self.assertEqual(self._medium(fake, None), expected)

    def test_psram_mode_falls_back_to_base_address(self):
        cases = (
            ("0x62000000", "nand"),
            ("0x10000000", "nor"),
        )
        for base, expected in cases:
            with self.subTest(base=base):
                fake = _FakePtab([{
                    "mem": "flash2",
                    "base": base,
                    "regions": [{"tags": ["FS_REGION"]}],
                }])
                self.assertEqual(self._medium(fake, 2), expected)

    def test_sd_memory_is_sd(self):
        fake = _FakePtab([{
            "mem": "sd1",
            "base": "0x50000000",
            "regions": [{"tags": ["FS_REGION"]}],
        }])
        self.assertEqual(self._medium(fake, None), "sd")


if __name__ == "__main__":
    unittest.main()
