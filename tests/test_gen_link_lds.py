#!/usr/bin/env python3
#
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

"""Regression tests for gen_link_lds on the SF32LB57 series.

Covers:
- ACPU execution addresses use the SBUS view (the ACPU cannot fetch from the
  CBUS/XIP alias), while HCPU keeps the XIP address for NOR.
- The ACPU data window comes from the `acpu_ram_data` partition and falls back
  to the `exec` window when the partition is absent.
- The 57 GCC/ARM linker-script templates render, including the ZBT fragments.
"""

from __future__ import annotations

import os
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Any, Dict

ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "tools", "build"))

import ptab  # noqa: E402

try:
    import gen_link_lds  # noqa: E402
except Exception as exc:  # pragma: no cover - depends on the local jinja2 install
    gen_link_lds = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None

# gen_link_lds resolves mem_map.h and template includes from the SDK root.
# Unit tests may run from any directory, so pin it to this checkout.
os.environ.setdefault("SIFLI_SDK", ROOT)

MB = 1024 * 1024
ACPU_PARTITION_SIZE = 0x00040000
ACPU_RAM_DATA_OFFSET = 0x0006FC00
ACPU_RAM_DATA_SIZE = 0x0000FC00


def _make_57_ptab(with_acpu_ram_data: bool = True) -> "ptab.PtabV3":
    """SF32LB57 v3 ptab with an HCPU app and an ACPU app in NOR.

    The ACPU image executes from psram1 (SBUS view for ACPU) and keeps its
    writable data in ``acpu_ram_data`` when present.
    """
    partitions = [
        {
            "name": "hcpu_flash_code",
            "type": "app",
            "subtype": "factory",
            "region": "mpi3",
            "offset": 0x00020000,
            "size": 0x00300000,
            "core": "HCPU",
        },
        {
            "name": "acpu",
            "type": "app",
            "subtype": "factory",
            "region": "mpi3",
            "offset": 0x00720000,
            "size": ACPU_PARTITION_SIZE,
            "core": "ACPU",
            "aliases": ["ACPU_CODE_LOAD_REGION1"],
            "exec": {"region": "psram1", "offset": 0},
        },
    ]
    if with_acpu_ram_data:
        partitions.append(
            {
                "name": "acpu_ram_data",
                "type": "data",
                "subtype": "ram",
                "region": "hpsys_ram",
                "offset": ACPU_RAM_DATA_OFFSET,
                "size": ACPU_RAM_DATA_SIZE,
                "core": "ACPU",
                "aliases": ["ACPU_RAM_DATA"],
            }
        )

    data = {
        "version": 3,
        "chip": "SF32LB57X",
        "memory": [
            {"mpi": "mpi1", "type": "psram", "size": 4 * MB},
            {"mpi": "mpi3", "type": "nor", "size": 16 * MB},
        ],
        "partitions": partitions,
    }
    return ptab.PtabV3("<in-memory>", data)


def _skip_reason() -> str:
    return "gen_link_lds requires jinja2: {}".format(_IMPORT_ERROR)


@unittest.skipUnless(gen_link_lds is not None, _skip_reason())
class ExecAddressSelectionTests(unittest.TestCase):
    def test_acpu_uses_sbus_for_nor(self) -> None:
        self.assertEqual(
            gen_link_lds._select_exec_addr("nor", 0x62000000, 0x12000000, "ACPU"),
            0x62000000,
        )

    def test_hcpu_uses_cbus_for_nor(self) -> None:
        self.assertEqual(
            gen_link_lds._select_exec_addr("nor", 0x62000000, 0x12000000, "HCPU"),
            0x12000000,
        )


@unittest.skipUnless(gen_link_lds is not None, _skip_reason())
class ComputeLinkDefinesTests(unittest.TestCase):
    def test_acpu_ram_window_uses_acpu_ram_data(self) -> None:
        ptab_obj = _make_57_ptab()
        defines = gen_link_lds.compute_link_defines(ptab_obj, "acpu", "ACPU", {})

        self.assertEqual(defines["__RAM_BASE"], 0x2006FC00)
        self.assertEqual(defines["__RAM_SIZE"], ACPU_RAM_DATA_SIZE)

        # The ROM window must be the SBUS view, not the CBUS/XIP alias.
        chip_config = ptab_obj.get_chip_config()
        sbus, cbus = ptab.resolve_region_address("psram1", 0, chip_config, core="ACPU")
        self.assertEqual(defines["__ROM_BASE"], sbus)
        self.assertNotEqual(defines["__ROM_BASE"], cbus)

    def test_acpu_without_ram_window_falls_back_to_exec(self) -> None:
        ptab_obj = _make_57_ptab(with_acpu_ram_data=False)
        defines = gen_link_lds.compute_link_defines(ptab_obj, "acpu", "ACPU", {})

        # Without a dedicated ACPU RAM partition the data window follows the
        # execution window, as the legacy linker scripts did.
        self.assertEqual(defines["__RAM_BASE"], defines["__ROM_BASE"])
        self.assertEqual(defines["__RAM_SIZE"], ACPU_PARTITION_SIZE)


@unittest.skipUnless(gen_link_lds is not None, _skip_reason())
class LinkTemplateRenderTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.out_dir = Path(self.temp_dir.name)
        ptab_obj = _make_57_ptab()
        self.hcpu_defines = gen_link_lds.compute_link_defines(ptab_obj, "main", "HCPU", {})
        self.acpu_defines = gen_link_lds.compute_link_defines(ptab_obj, "acpu", "ACPU", {})
        self.bootloader_defines = gen_link_lds.compute_link_defines(ptab_obj, "bootloader", "HCPU", {})

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    def render(self, rel_path: str, defines: Dict[str, Any], output_name: str) -> str:
        template = os.path.join(ROOT, rel_path)
        output = str(self.out_dir / output_name)
        gen_link_lds.render_link_lds(template, output, defines)
        return Path(output).read_text(encoding="utf-8")

    def test_57_gcc_hcpu_renders_with_and_without_zbt(self) -> None:
        template = "drivers/cmsis/sf32lb57x/Templates/gcc/hcpu/link.jinja2"

        for zbt in (False, True):
            with self.subTest(zbt=zbt):
                defines = dict(self.hcpu_defines)
                if zbt:
                    defines["ZBT"] = 1

                rendered = self.render(template, defines, "hcpu_zbt_{}.lds".format(int(zbt)))

                self.assertIn("__ROM_BASE = 0x", rendered)
                self.assertIn("__RAM_BASE = 0x", rendered)
                self.assertIn("__StackTop", rendered)
                # The ZBT fragments add the Bluetooth connection sections.
                self.assertEqual("bt_conn_cb_area" in rendered, zbt)

    def test_57_gcc_acpu_renders_separate_rom_and_ram_windows(self) -> None:
        rendered = self.render(
            "drivers/cmsis/sf32lb57x/Templates/gcc/acpu/link.jinja2",
            self.acpu_defines,
            "acpu.lds",
        )

        self.assertIn("__ROM_BASE = 0x60000000;", rendered)
        self.assertIn("__RAM_BASE = 0x2006FC00;", rendered)
        self.assertIn("__RAM_SIZE = 0x0000FC00;", rendered)
        # Separate windows select the explicit .data placement branch.
        self.assertIn(".data :", rendered)

    def test_57_bootloader_ram_v2_template_renders(self) -> None:
        rendered = self.render(
            "example/boot_loader/project/sf32lb57x/ram_v2/link.jinja2",
            self.bootloader_defines,
            "bootloader_ram_v2.lds",
        )

        self.assertIn("__ROM_BASE = 0x", rendered)
        self.assertIn("__RAM_BASE = 0x", rendered)
        self.assertIn("ROM2", rendered)
        self.assertIn("ROM3", rendered)

    def test_57_arm_scatter_templates_render(self) -> None:
        cases = [
            (
                "drivers/cmsis/sf32lb57x/Templates/arm/hcpu/link.sct.jinja2",
                self.hcpu_defines,
                "hcpu.sct",
            ),
            (
                "drivers/cmsis/sf32lb57x/Templates/arm/acpu/link.sct.jinja2",
                self.acpu_defines,
                "acpu.sct",
            ),
        ]

        for template, defines, output_name in cases:
            with self.subTest(template=template):
                rendered = self.render(template, defines, output_name)
                self.assertIn("LR_IROM1", rendered)
                self.assertIn("ER_IROM1", rendered)


if __name__ == "__main__":
    unittest.main()
