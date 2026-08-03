#!/usr/bin/env python3
#
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
#
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for ptab region address resolution (SBUS/CBUS) and the
v3 ptab.h ACPU execution-address macros.

Covers the SF32LB58 ACPU CBUS mapping fix:
- convert_to_cbus_addr() with an explicit `series` (offline-safe).
- resolve_region_address() returns a 0-based CBUS view for ACPU RAM.
- GenPartitionTableHeaderContentV3() emits ACPU_CODE_REGION*[_EXEC*] macros
  using the shared SBUS view for the execution address
  (ACPU no longer selects its local CBUS view, so addresses ACPU passes to
  HCPU stay valid in HCPU's view).
"""

from __future__ import annotations

import os
import sys
import unittest
from typing import Dict

ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "tools", "build"))

import ptab  # noqa: E402
import sdk_resource  # noqa: E402


def _sf32lb58_chip_config():
    return ptab.load_chip_config("sf32lb58")


def _acpu_v3_ptab():
    """SF32LB58 v3 ptab with an ACPU factory app executing from hpsys_ram."""
    data = {
        "version": 3,
        "chip": "SF32LB58X",
        "memory": [{"mpi": "mpi4", "type": "nor", "size": 16 * 1024 * 1024}],
        "partitions": [
            {
                "name": "acpu",
                "type": "app",
                "subtype": "factory",
                "region": "mpi4",
                "offset": 0x00F60000,
                "size": 0x0007C000,
                "core": "ACPU",
                "aliases": ["ACPU_CODE_LOAD_REGION1"],
                "exec": {"region": "hpsys_ram", "offset": 0x00200000},
            },
        ],
    }
    return ptab.PtabV3("<in-memory>", data)


def _parse_macros(body: str) -> Dict[str, str]:
    """Extract {name: value} from `#define name value` lines in ptab.h."""
    macros: Dict[str, str] = {}
    for line in body.splitlines():
        line = line.strip()
        if line.startswith("#define"):
            parts = line.split(None, 2)
            if len(parts) == 3:
                macros[parts[1]] = parts[2]
    return macros


def _sf32lb52_boot_hcpu_ptab():
    """SF32LB52 v3 ptab with a bootloader (NAND storage, RAM exec) and an
    HCPU factory app (NAND storage, PSRAM exec) named `hcpu_flash_code`."""
    data = {
        "version": 3,
        "chip": "SF32LB52X",
        "memory": [{"mpi": "mpi2", "type": "nand", "size": 16 * 1024 * 1024}],
        "partitions": [
            {
                "name": "flash_table",
                "type": "ftab",
                "region": "mpi2",
                "offset": 0,
                "size": 0x20000,
            },
            {
                "name": "bootloader",
                "type": "bootloader",
                "region": "mpi2",
                "offset": 0x80000,
                "size": 0x10000,
                "core": "HCPU",
                "exec": {"region": "hpsys_ram", "offset": 0x20000},
            },
            {
                "name": "hcpu_flash_code",
                "type": "app",
                "subtype": "factory",
                "region": "mpi2",
                "offset": 0xA0000,
                "size": 0x100000,
                "core": "HCPU",
                "exec": {"region": "psram1", "offset": 0},
            },
        ],
    }
    return ptab.PtabV3("<in-memory>", data)


class ConvertToCbusAddrTests(unittest.TestCase):
    def test_sf32lb58_acpu_ram_is_zero_based(self) -> None:
        # 0x20200000 (HCPU/SBUS) <-> 0x00000000 (ACPU CBUS)
        self.assertEqual(
            ptab.convert_to_cbus_addr(0x20200000, 0x00200000, "ACPU", series="sf32lb58"),
            (0x00000000, 0x00000000),
        )
        self.assertEqual(
            ptab.convert_to_cbus_addr(0x20200004, 0x00200004, "ACPU", series="sf32lb58"),
            (0x00000004, 0x00000004),
        )

    def test_sf32lb58_non_acpu_ram_is_unchanged(self) -> None:
        self.assertEqual(
            ptab.convert_to_cbus_addr(0x20200000, 0x00200000, "HCPU", series="sf32lb58"),
            (0x20200000, 0x00200000),
        )
        self.assertEqual(
            ptab.convert_to_cbus_addr(0x20200000, 0x00200000, None, series="sf32lb58"),
            (0x20200000, 0x00200000),
        )

    def test_sf32lb58_high_address_range_shifts_to_cbus(self) -> None:
        # 0x60000000-0x6FFFFFFF (HCPU view) -> -0x50000000
        self.assertEqual(
            ptab.convert_to_cbus_addr(0x68000000, 0, "HCPU", series="sf32lb58"),
            (0x18000000, 0),
        )

    def test_other_series_acpu_ram_unchanged(self) -> None:
        for series in ("sf32lb52", "sf32lb55", "sf32lb56", "sf32lb57"):
            self.assertEqual(
                ptab.convert_to_cbus_addr(0x20200000, 0x00200000, "ACPU", series=series),
                (0x20200000, 0x00200000),
                "series={} should not remap ACPU RAM".format(series),
            )


class ResolveRegionAddressTests(unittest.TestCase):
    def test_sf32lb58_acpu_hpsys_ram_cbus_is_zero_based(self) -> None:
        chip_config = _sf32lb58_chip_config()
        sbus, cbus = ptab.resolve_region_address(
            "hpsys_ram", 0x00200000, chip_config, core="ACPU"
        )
        self.assertEqual((sbus, cbus), (0x20200000, 0x00000000))

    def test_sf32lb58_hcpu_hpsys_ram_cbus_equals_sbus(self) -> None:
        chip_config = _sf32lb58_chip_config()
        sbus, cbus = ptab.resolve_region_address(
            "hpsys_ram", 0x00200000, chip_config, core="HCPU"
        )
        self.assertEqual((sbus, cbus), (0x20200000, 0x20200000))


class PtabV3AcpuHeaderMacroTests(unittest.TestCase):
    def setUp(self) -> None:
        # sdk_resource.MakeLine() relies on the module-level indentation state.
        sdk_resource.InitIndentation()

    def test_acpu_exec_macros_use_sbus_view(self) -> None:
        ptab_obj = _acpu_v3_ptab()
        body = sdk_resource.GenPartitionTableHeaderContentV3({"name": "acpu"}, ptab_obj)
        macros = _parse_macros(body)

        # ACPU exec macros use the shared SBUS view (0x20200000), NOT ACPU's
        # local 0-based CBUS view. ACPU passes ACPU_CODE_REGION<N>_EXEC_START_ADDR
        # to HCPU, so the address must be valid in HCPU's view.
        self.assertEqual(macros.get("ACPU_CODE_REGION1_EXEC_SBUS_START_ADDR"), "(0x20200000)")
        self.assertEqual(macros.get("ACPU_CODE_REGION1_EXEC_START_ADDR"), "(0x20200000)")
        # Legacy aliases used by ACPU link scripts.
        self.assertEqual(
            macros.get("ACPU_CODE_REGION1_SBUS_START_ADDR"),
            "(ACPU_CODE_REGION1_EXEC_SBUS_START_ADDR)",
        )
        self.assertEqual(
            macros.get("ACPU_CODE_REGION1_START_ADDR"),
            "(ACPU_CODE_REGION1_EXEC_START_ADDR)",
        )
        # Storage macros come from the per-partition pass.
        self.assertIn("ACPU_START_ADDR", macros)
        self.assertIn("ACPU_CODE_LOAD_REGION1_START_ADDR", macros)


class PtabV3BootAndHcpuMacroTests(unittest.TestCase):
    def setUp(self) -> None:
        sdk_resource.InitIndentation()

    def test_flash_boot_loader_storage_and_exec_macros(self) -> None:
        ptab_obj = _sf32lb52_boot_hcpu_ptab()
        chip_config = ptab_obj.get_chip_config()
        body = sdk_resource.GenPartitionTableHeaderContentV3({"name": "bootloader"}, ptab_obj)
        macros = _parse_macros(body)

        # FLASH_BOOT_LOADER_* is the bootloader's STORAGE macro (== BOOTLOADER_*).
        bl_sbus, _ = ptab.resolve_region_address("mpi2", 0x80000, chip_config, core="HCPU")
        self.assertEqual(macros.get("FLASH_BOOT_LOADER_START_ADDR"), "(0x{:08X})".format(bl_sbus))
        self.assertEqual(
            macros.get("FLASH_BOOT_LOADER_START_ADDR"), macros.get("BOOTLOADER_START_ADDR")
        )
        self.assertEqual(macros.get("FLASH_BOOT_LOADER_OFFSET"), "(0x00080000)")

        # FLASH_BOOT_LOADER_EXEC_* is the execution (RAM) address.
        exec_sbus, _ = ptab.resolve_region_address("hpsys_ram", 0x20000, chip_config, core="HCPU")
        self.assertEqual(
            macros.get("FLASH_BOOT_LOADER_EXEC_START_ADDR"), "(0x{:08X})".format(exec_sbus)
        )
        self.assertNotEqual(
            macros.get("FLASH_BOOT_LOADER_EXEC_START_ADDR"),
            macros.get("FLASH_BOOT_LOADER_START_ADDR"),
        )

    def test_hcpu_flash_code_storage_and_exec_macros(self) -> None:
        ptab_obj = _sf32lb52_boot_hcpu_ptab()
        chip_config = ptab_obj.get_chip_config()
        body = sdk_resource.GenPartitionTableHeaderContentV3({"name": "main"}, ptab_obj)
        macros = _parse_macros(body)

        # HCPU_FLASH_CODE_START_ADDR is the storage/download (NAND SBUS) address.
        hcpu_sbus, _ = ptab.resolve_region_address("mpi2", 0xA0000, chip_config, core="HCPU")
        self.assertEqual(macros.get("HCPU_FLASH_CODE_START_ADDR"), "(0x{:08X})".format(hcpu_sbus))
        self.assertEqual(macros.get("HCPU_FLASH_CODE_OFFSET"), "(0x000A0000)")

        # HCPU_FLASH_CODE_EXEC_START_ADDR is the execution (PSRAM XIP) address.
        _, psram_cbus = ptab.resolve_region_address("psram1", 0, chip_config, core="HCPU")
        self.assertEqual(
            macros.get("HCPU_FLASH_CODE_EXEC_START_ADDR"), "(0x{:08X})".format(psram_cbus)
        )
        self.assertNotEqual(
            macros.get("HCPU_FLASH_CODE_EXEC_START_ADDR"),
            macros.get("HCPU_FLASH_CODE_START_ADDR"),
        )


if __name__ == "__main__":
    unittest.main()
