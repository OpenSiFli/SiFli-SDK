#!/usr/bin/env python3
#
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

"""Tests for the APP_ACPU_CODE_START_ADDR start alias.

HAL_RCC_ReleaseACPU() takes the ACPU VTOR, so APP_ACPU_CODE_START_ADDR must
be an address the ACPU can execute from:

* ptab v3: the macro is derived from the ACPU app partition name (like v2) and
  references ``ACPU_CODE_REGION<N>_EXEC_START_ADDR``, because the ACPU image
  runs from its ``exec`` region (PSRAM RAM/SBUS);
* ptab v1/v2: the ACPU image executes in place from flash, so the alias stays
  the storage address converted to the SBUS view.
"""

from __future__ import annotations

import os
import sys
import unittest
from typing import Any, Dict
from unittest import mock

ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "tools", "build"))

import ptab  # noqa: E402
import sdk_resource  # noqa: E402


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


def _acpu_v3_ptab() -> "ptab.PtabV3":
    """SF32LB57 v3 ptab: ACPU app stored in NOR, executing from PSRAM."""
    data: Dict[str, Any] = {
        "version": 3,
        "chip": "SF32LB57X",
        "memory": [{"mpi": "mpi3", "type": "nor", "size": 16 * 1024 * 1024}],
        "partitions": [
            {
                "name": "hcpu_flash_code",
                "type": "app",
                "subtype": "factory",
                "region": "mpi3",
                "offset": 0x00020000,
                "size": 0x00700000,
                "core": "HCPU",
                "exec": {"region": "psram1", "offset": 0x00000000},
            },
            {
                "name": "acpu",
                "type": "app",
                "subtype": "factory",
                "region": "mpi3",
                "offset": 0x00720000,
                "size": 0x00040000,
                "core": "ACPU",
                "aliases": ["ACPU_CODE_LOAD_REGION1"],
                "exec": {"region": "psram1", "offset": 0x00000000},
            },
            {
                "name": "acpu_ram_data",
                "type": "data",
                "subtype": "ram",
                "region": "hpsys_ram",
                "offset": 0x0006FC00,
                "size": 0x0000FC00,
                "core": "ACPU",
                "aliases": ["ACPU_RAM_DATA"],
            },
        ],
    }
    return ptab.PtabV3("<in-memory>", data)


def _acpu_v3_inplace_ptab() -> "ptab.PtabV3":
    """SF32LB57 v3 ptab: ACPU app executing in place from NOR (no exec)."""
    data: Dict[str, Any] = {
        "version": 3,
        "chip": "SF32LB57X",
        "memory": [{"mpi": "mpi3", "type": "nor", "size": 16 * 1024 * 1024}],
        "partitions": [
            {
                "name": "hcpu_flash_code",
                "type": "app",
                "subtype": "factory",
                "region": "mpi3",
                "offset": 0x00020000,
                "size": 0x00700000,
                "core": "HCPU",
            },
            {
                "name": "acpu",
                "type": "app",
                "subtype": "factory",
                "region": "mpi3",
                "offset": 0x00720000,
                "size": 0x00040000,
                "core": "ACPU",
                "aliases": ["ACPU_CODE_LOAD_REGION1"],
            },
            {
                "name": "acpu_ram_data",
                "type": "data",
                "subtype": "ram",
                "region": "hpsys_ram",
                "offset": 0x0006FC00,
                "size": 0x0000FC00,
                "core": "ACPU",
                "aliases": ["ACPU_RAM_DATA"],
            },
        ],
    }
    return ptab.PtabV3("<in-memory>", data)


def _acpu_v3_two_image_ptab() -> "ptab.PtabV3":
    """SF32LB57 v3 ptab: two ACPU images stored in NOR, executing from PSRAM.

    The second image follows the v3 naming: a trailing index in the partition
    name (``acpu2``) and a ``.rom2`` binding for its ELF output section.
    """
    data: Dict[str, Any] = {
        "version": 3,
        "chip": "SF32LB57X",
        "memory": [{"mpi": "mpi3", "type": "nor", "size": 16 * 1024 * 1024}],
        "partitions": [
            {
                "name": "hcpu_flash_code",
                "type": "app",
                "subtype": "factory",
                "region": "mpi3",
                "offset": 0x00020000,
                "size": 0x00700000,
                "core": "HCPU",
                "exec": {"region": "psram1", "offset": 0x00000000},
            },
            {
                "name": "acpu",
                "type": "app",
                "subtype": "factory",
                "region": "mpi3",
                "offset": 0x00720000,
                "size": 0x00040000,
                "core": "ACPU",
                "aliases": ["ACPU_CODE_LOAD_REGION1"],
                "exec": {"region": "psram1", "offset": 0x00000000},
            },
            {
                "name": "acpu2",
                "type": "app",
                "subtype": "factory",
                "region": "mpi3",
                "offset": 0x007A0000,
                "size": 0x00020000,
                "core": "ACPU",
                "exec": {"region": "psram1", "offset": 0x00040000},
                "sections": [{"section": ".rom2"}],
            },
            {
                "name": "acpu_ram_data",
                "type": "data",
                "subtype": "ram",
                "region": "hpsys_ram",
                "offset": 0x0006FC00,
                "size": 0x0000FC00,
                "core": "ACPU",
                "aliases": ["ACPU_RAM_DATA"],
            },
        ],
    }
    return ptab.PtabV3("<in-memory>", data)


class V3AcpuStartAliasTests(unittest.TestCase):
    def setUp(self) -> None:
        # sdk_resource.MakeLine() relies on the module-level indentation state.
        sdk_resource.InitIndentation()

    def test_start_alias_points_at_exec_address(self) -> None:
        ptab_obj = _acpu_v3_ptab()
        body = sdk_resource.GenPartitionTableHeaderContentV3({"name": "main"}, ptab_obj)
        macros = _parse_macros(body)

        # The macro is derived from the partition name and references the exec
        # macro, not the storage address.
        self.assertEqual(
            macros.get("APP_ACPU_CODE_START_ADDR"),
            "(ACPU_CODE_REGION1_EXEC_START_ADDR)",
        )

        # The referenced exec macro is the PSRAM SBUS address the ACPU runs
        # from (not the NOR storage address).
        exec_sbus, _ = ptab.resolve_region_address(
            "psram1", 0, ptab_obj.get_chip_config(), core="ACPU"
        )
        self.assertEqual(
            macros.get("ACPU_CODE_REGION1_EXEC_START_ADDR"),
            "(0x{:08X})".format(exec_sbus),
        )
        self.assertEqual(macros.get("ACPU_CODE_REGION1_EXEC_START_ADDR"), "(0x60000000)")

        # SIZE/OFFSET describe the same execution window as START_ADDR, like
        # v2 (the exec region, not the storage partition).
        self.assertEqual(macros.get("APP_ACPU_CODE_SIZE"), "(0x00040000)")
        self.assertEqual(macros.get("APP_ACPU_CODE_OFFSET"), "(0x00000000)")

        # Other aliases (e.g. ACPU_CODE_LOAD_REGION1) keep the storage address
        # in the SBUS view, which is the only one the ACPU can fetch from.
        storage_sbus, _ = ptab.resolve_region_address(
            "mpi3", 0x00720000, ptab_obj.get_chip_config(), core="ACPU"
        )
        self.assertEqual(
            macros.get("ACPU_CODE_LOAD_REGION1_START_ADDR"),
            "(0x{:08X})".format(storage_sbus),
        )
        self.assertNotEqual(
            macros.get("ACPU_CODE_LOAD_REGION1_START_ADDR"),
            macros.get("ACPU_CODE_REGION1_EXEC_START_ADDR"),
        )

        # The partition carries no APP_ACPU_CODE alias: the macro must come
        # from the name alone.
        acpu = next(
            p for p in ptab_obj.partitions if p.get("name") == "acpu")
        self.assertNotIn("APP_ACPU_CODE", acpu.get("aliases", []))

    def test_in_place_nor_uses_storage_sbus(self) -> None:
        """57 NOR without exec: the ACPU executes in place from the NOR SBUS."""
        ptab_obj = _acpu_v3_inplace_ptab()
        body = sdk_resource.GenPartitionTableHeaderContentV3({"name": "main"}, ptab_obj)
        macros = _parse_macros(body)

        storage_sbus, _ = ptab.resolve_region_address(
            "mpi3", 0x00720000, ptab_obj.get_chip_config(), core="ACPU"
        )
        self.assertEqual(storage_sbus, 0x64720000)

        # No exec window: the storage SBUS address is the execution address.
        self.assertEqual(
            macros.get("APP_ACPU_CODE_START_ADDR"),
            "(ACPU_CODE_REGION1_EXEC_START_ADDR)",
        )
        self.assertEqual(
            macros.get("ACPU_CODE_REGION1_EXEC_START_ADDR"),
            "(0x{:08X})".format(storage_sbus),
        )

        # Every ACPU storage macro must stay in the SBUS view.
        self.assertEqual(
            macros.get("ACPU_START_ADDR"), "(0x{:08X})".format(storage_sbus))
        self.assertEqual(
            macros.get("ACPU_CODE_LOAD_REGION1_START_ADDR"),
            "(0x{:08X})".format(storage_sbus),
        )
        # In-place execution: the partition itself is the execution window.
        self.assertEqual(macros.get("APP_ACPU_CODE_SIZE"), "(0x00040000)")
        self.assertEqual(macros.get("APP_ACPU_CODE_OFFSET"), "(0x00720000)")


class V3AcpuCodeNameIndexTests(unittest.TestCase):
    """An ACPU image index is spelled as a name suffix (`acpu2`)."""

    def setUp(self) -> None:
        sdk_resource.InitIndentation()

    def test_trailing_index_names_the_second_image(self) -> None:
        ptab_obj = _acpu_v3_two_image_ptab()
        body = sdk_resource.GenPartitionTableHeaderContentV3(
            {"name": "main"}, ptab_obj)
        macros = _parse_macros(body)

        # APP_<NAME>_CODE<N> follows the name suffix and references the same
        # image the partition binds with '.rom2'.
        self.assertEqual(
            macros.get("APP_ACPU_CODE2_START_ADDR"),
            "(ACPU_CODE_REGION2_EXEC_START_ADDR)",
        )
        exec_sbus, _ = ptab.resolve_region_address(
            "psram1", 0x00040000, ptab_obj.get_chip_config(), core="ACPU")
        self.assertEqual(exec_sbus, 0x60040000)
        self.assertEqual(
            macros.get("ACPU_CODE_REGION2_EXEC_START_ADDR"),
            "(0x{:08X})".format(exec_sbus),
        )
        # The macro group describes the second image's execution window.
        self.assertEqual(macros.get("APP_ACPU_CODE2_SIZE"), "(0x00020000)")
        self.assertEqual(macros.get("APP_ACPU_CODE2_OFFSET"), "(0x00040000)")

        # The first image keeps the unsuffixed names.
        self.assertEqual(
            macros.get("APP_ACPU_CODE_START_ADDR"),
            "(ACPU_CODE_REGION1_EXEC_START_ADDR)",
        )

        # The partition name is pasted into the macros, so it must stay a legal
        # identifier: the suffix form emits `ACPU2_START_ADDR`, where the v2
        # `acpu:2` form would have emitted `ACPU:2_START_ADDR`.
        self.assertIn("ACPU2_START_ADDR", macros)
        for line in body.splitlines():
            line = line.strip()
            if line.startswith(("#define", "#undef")):
                self.assertRegex(
                    line.split()[1], r"^[A-Za-z_][A-Za-z0-9_]*$",
                    "illegal macro name in: {}".format(line))

    def test_first_image_keeps_region_one(self) -> None:
        """A partition without '.rom<N>' is image 1 whatever its name says.

        The validator rejects a disagreeing name, but the generator must not let
        the suffix move the first image: APP_ACPU_CODE_START_ADDR is the ACPU
        VTOR and has to stay defined.
        """
        data: Dict[str, Any] = {
            "version": 3,
            "chip": "SF32LB57X",
            "memory": [{"mpi": "mpi3", "type": "nor", "size": 16 * 1024 * 1024}],
            "partitions": [
                {
                    "name": "hcpu_flash_code",
                    "type": "app",
                    "subtype": "factory",
                    "region": "mpi3",
                    "offset": 0x00020000,
                    "size": 0x00700000,
                    "core": "HCPU",
                },
                {
                    "name": "acpu2",
                    "type": "app",
                    "subtype": "factory",
                    "region": "mpi3",
                    "offset": 0x00720000,
                    "size": 0x00040000,
                    "core": "ACPU",
                    "exec": {"region": "psram1", "offset": 0x00000000},
                },
            ],
        }
        ptab_obj = ptab.PtabV3("<in-memory>", data)
        macros = _parse_macros(sdk_resource.GenPartitionTableHeaderContentV3(
            {"name": "main"}, ptab_obj))

        self.assertEqual(
            macros.get("APP_ACPU_CODE_START_ADDR"),
            "(ACPU_CODE_REGION1_EXEC_START_ADDR)",
        )
        self.assertEqual(
            macros.get("ACPU_CODE_REGION1_EXEC_START_ADDR"), "(0x60000000)")
        self.assertNotIn("ACPU_CODE_REGION2_EXEC_START_ADDR", macros)

    def test_trailing_index_is_a_valid_partition_name(self) -> None:
        """The name rule accepts `acpu2` and rejects the v2 `acpu:2` form."""
        from validate_ptab_v3 import validate_name

        self.assertEqual(validate_name("acpu2", 0), [])
        self.assertTrue(validate_name("acpu:2", 0))


class V3BoardAcpuStartMacroTests(unittest.TestCase):
    def setUp(self) -> None:
        sdk_resource.InitIndentation()

    def test_58_board_without_alias_generates_start_macro(self) -> None:
        """A hand-written board with no APP_ACPU_CODE alias still gets it."""
        ptab_path = os.path.join(
            ROOT,
            "customer", "boards", "sf32lb58-lcd_a128r32n1_qspi", "ptab.yaml",
        )
        ptab_obj = ptab.load_ptab(ptab_path, fatal=True)
        acpu = next(p for p in ptab_obj.partitions if p.get("name") == "acpu")
        self.assertNotIn("APP_ACPU_CODE", acpu.get("aliases", []))

        body = sdk_resource.GenPartitionTableHeaderContentV3(
            {"name": "main"}, ptab_obj)
        macros = _parse_macros(body)

        self.assertEqual(
            macros.get("APP_ACPU_CODE_START_ADDR"),
            "(ACPU_CODE_REGION1_EXEC_START_ADDR)",
        )
        # hpsys_ram 0x00200000 seen from the SBUS (HCPU) view.
        self.assertEqual(
            macros.get("ACPU_CODE_REGION1_EXEC_START_ADDR"), "(0x20200000)")

        # The storage alias keeps the NAND storage address.
        self.assertEqual(
            macros.get("ACPU_CODE_LOAD_REGION1_START_ADDR"),
            macros.get("ACPU_START_ADDR"),
        )
        self.assertNotEqual(
            macros.get("ACPU_CODE_LOAD_REGION1_START_ADDR"),
            macros.get("ACPU_CODE_REGION1_EXEC_START_ADDR"),
        )
        self.assertEqual(macros.get("APP_ACPU_CODE_SIZE"), "(0x0007C000)")
        # exec offset inside hpsys_ram, not the NAND storage offset.
        self.assertEqual(macros.get("APP_ACPU_CODE_OFFSET"), "(0x00200000)")


class V2AcpuStartAliasTests(unittest.TestCase):
    def setUp(self) -> None:
        sdk_resource.InitIndentation()

    def _render(self, mems: list) -> Dict[str, str]:
        # The v1/v2 generator injects default regions through ptab, which
        # imports the SCons build context. The alias emission under test does
        # not depend on those defaults, so skip the injection and stub the
        # Kconfig lookups used by the address converters.
        with mock.patch.object(sdk_resource, "PtabAddAddDefaultRegion", lambda m: None), \
                mock.patch.object(
                    ptab, "_get_depend", side_effect=lambda name: name == "SOC_SF32LB57X"
                ), \
                mock.patch.object(ptab, "_get_config_value", return_value=""):
            body = sdk_resource.GenPartitionTableHeaderContentV2(
                {"name": "main"}, mems
            )
        return _parse_macros(body)

    def test_start_alias_keeps_storage_sbus_address(self) -> None:
        # v2-style mems: the ACPU app executes in place from NOR flash.
        mems = [
            {
                "mem": "flash2",
                "base": "0x62000000",
                "regions": [
                    {
                        "offset": "0x00020000",
                        "max_size": "0x00700000",
                        "name": "main",
                        "type": ["app_img", "app_exec"],
                        "core": "HCPU",
                        "tags": ["HCPU_FLASH_CODE"],
                    },
                    {
                        "offset": "0x00720000",
                        "max_size": "0x00040000",
                        "name": "acpu",
                        "type": ["app_img", "app_exec"],
                        "core": "ACPU",
                        "tags": ["ACPU_CODE_LOAD_REGION1"],
                    },
                ],
            },
            {"mem": "psram1", "base": "0x60000000", "regions": []},
        ]

        macros = self._render(mems)

        # v2 has no exec region for ACPU: the alias stays the flash SBUS
        # storage address (0x62000000 + 0x00720000).
        self.assertEqual(macros.get("APP_ACPU_CODE_START_ADDR"), "(0x62720000)")
        self.assertEqual(macros.get("APP_ACPU_CODE_SIZE"), "(0x00040000)")
        self.assertEqual(macros.get("APP_ACPU_CODE_OFFSET"), "(0x00720000)")
        self.assertNotIn("_EXEC_START_ADDR", macros.get("APP_ACPU_CODE_START_ADDR", ""))


if __name__ == "__main__":
    unittest.main()
