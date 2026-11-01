#!/usr/bin/env python3
#
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

"""Regression tests for the v1/v2 SD default regions injected by ptab.py.

The v3 defaults are covered by ``test_ptab_v3_sf32lb57.py``. These tests pin
the legacy (v1/v2 JSON) SD layout instead: the flash table follows the MBR at
0x1000, while the bootloader starts at 0x61000 (after the 0x41000 factory
data region).
"""

from __future__ import annotations

import os
import sys
import unittest
from unittest import mock
from typing import Any, Dict, List, Optional


ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "tools", "build"))

import ptab  # noqa: E402


def _sd_mems() -> List[Dict[str, Any]]:
    """Minimal v1/v2 memory list for an SD-boot board."""
    return [
        {"mem": "sd1", "base": "0x64000000", "regions": []},
        {"mem": "hpsys_ram", "base": "0x20000000", "regions": []},
    ]


def _find_region(mems: List[Dict[str, Any]], mem_name: str, region_name: str) -> Optional[Dict[str, Any]]:
    for mem in mems:
        if mem.get("mem") != mem_name:
            continue
        for region in mem.get("regions", []):
            if isinstance(region, dict) and region.get("name") == region_name:
                return region
    return None


class SdDefaultRegionTests(unittest.TestCase):
    def add_defaults(self, soc_symbol: str) -> List[Dict[str, Any]]:
        mems = _sd_mems()

        def get_depend(name: str) -> bool:
            return name == soc_symbol

        with mock.patch.object(ptab, "_get_depend", side_effect=get_depend), \
                mock.patch.object(ptab, "_get_config_value", return_value=""):
            ptab.add_default_regions(mems)

        return mems

    def assert_sd_defaults(self, soc_symbol: str) -> None:
        mems = self.add_defaults(soc_symbol)

        ftab = _find_region(mems, "sd1", "ftab")
        bootloader = _find_region(mems, "sd1", "bootloader")
        exec_bootloader = _find_region(mems, "hpsys_ram", "bootloader")

        self.assertIsNotNone(ftab, "flash_table was not injected on sd1")
        self.assertIsNotNone(bootloader, "bootloader was not injected on sd1")
        self.assertIsNotNone(exec_bootloader, "bootloader exec region was not injected on hpsys_ram")

        # MBR occupies the first 4 KB; the flash table follows it.
        self.assertEqual(ptab.parse_size(ftab["offset"]), 0x00001000)
        self.assertEqual(ptab.parse_size(ftab["max_size"]), 0x00008000)

        # Factory data ends at 0x61000, where the bootloader image starts.
        self.assertEqual(ptab.parse_size(bootloader["offset"]), 0x00061000)
        self.assertEqual(ptab.parse_size(bootloader["max_size"]), 0x00010000)

        # The RAM-side execution window is independent of the storage offset.
        self.assertEqual(ptab.parse_size(exec_bootloader["offset"]), 0x00020000)
        self.assertEqual(ptab.parse_size(exec_bootloader["max_size"]), 0x00010000)

    def test_52_sd_default_bootloader_offset(self) -> None:
        self.assert_sd_defaults("SOC_SF32LB52X")

    def test_57_sd_default_bootloader_offset(self) -> None:
        self.assert_sd_defaults("SOC_SF32LB57X")


if __name__ == "__main__":
    unittest.main()
