#!/usr/bin/env python
#
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from pathlib import Path
from typing import Any, Optional
import os
import sys
import tempfile
import unittest

import yaml


ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "tools", "build"))

from sdk_py_actions.new_board import ChipVariant  # noqa: E402
from sdk_py_actions.new_board import MB  # noqa: E402
from sdk_py_actions.new_board import MemoryEntry  # noqa: E402
from sdk_py_actions.new_board import SAMPLE_BASE  # noqa: E402
from sdk_py_actions.new_board import Spec  # noqa: E402
from sdk_py_actions.new_board import build_confirmation_summary  # noqa: E402
from sdk_py_actions.new_board import choose_hcpu_jlink_device  # noqa: E402
from sdk_py_actions.new_board import core_definitions  # noqa: E402
from sdk_py_actions.new_board import normalize_spec  # noqa: E402
from sdk_py_actions.new_board import render_board_files  # noqa: E402
from sdk_py_actions.new_board import _memory_summary_suffix  # noqa: E402

import ptab  # noqa: E402
import sdk_resource  # noqa: E402


ASSETS_ROOT = Path(ROOT) / "tools" / "sdk_py_actions" / "new_board"
SCHEMA_PATH = ASSETS_ROOT / "schema" / "new-board.schema.json"
BASE_TEMPLATE_FILES = [
    "Kconfig.board",
    "board.h",
    "bsp_board.h",
    "bsp_init.c",
    "bsp_lcd_tp.c",
    "bsp_pinmux.c",
    "bsp_power.c",
]


class NewBoardRenderTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.root = Path(self.temp_dir.name).resolve()
        self.output_root = self.root / "output"
        self.output_root.mkdir()
        self.sdk_root = self.root / "sdk"
        self.sdk_root.mkdir()
        self.variant = ChipVariant(
            series="52",
            chip_dir="SF32LB52x",
            model_id="SF32LB52X",
            part_number="SF32LB52X",
            memory=(MemoryEntry("mpi1", "nor", 2 * MB),),
        )

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    def make_spec(
        self,
        base_name: str,
        generate_base: bool = False,
        series: str = "52",
        chip_model: str = "SF32LB52X",
        storage_type: str = "none",
        storage_size_mb: Optional[int] = None,
        storage_port: Optional[str] = None,
        storage_pinmux_type: Optional[str] = None,
        nand_block_size_kb: Optional[int] = None,
        with_acpu: bool = False,
    ) -> Spec:
        return Spec(
            series=series,
            chip_model=chip_model,
            storage_type=storage_type,
            storage_size_mb=storage_size_mb,
            storage_port=storage_port,
            board_name="demo_board",
            base_name=base_name,
            generate_base=generate_base,
            storage_pinmux_type=storage_pinmux_type,
            nand_block_size_kb=nand_block_size_kb,
            with_acpu=with_acpu,
        )

    def make_sample_base(self, series: str = "52", real_pinmux: bool = False) -> None:
        base_dir = self.sdk_root / SAMPLE_BASE[series]
        base_dir.mkdir(parents=True)
        for name in BASE_TEMPLATE_FILES:
            if real_pinmux and name in {"bsp_pinmux.c", "bsp_power.c"}:
                content = (Path(ROOT) / SAMPLE_BASE[series] / name).read_text(encoding="utf-8")
            else:
                content = f"// {name}\n"
            (base_dir / name).write_text(content, encoding="utf-8")

    def make_existing_base(self, base_dir: Path, base_kind: str = "base") -> None:
        base_dir.mkdir(parents=True)
        (base_dir / "Kconfig.board").write_text("config EXISTING_BASE\n", encoding="utf-8")
        if base_kind == "script":
            script_dir = base_dir / "script"
            script_dir.mkdir()
            (script_dir / "SConscript").write_text("# existing script base\n", encoding="utf-8")
        else:
            (base_dir / "SConscript.base").write_text("# existing base\n", encoding="utf-8")

    def render_top_kconfig(self, spec: Spec) -> str:
        rendered = render_board_files(
            spec=spec,
            variant=self.variant,
            sdk_root=self.sdk_root,
            output_root=self.output_root,
            assets_root=ASSETS_ROOT,
        )
        return rendered[self.output_root / spec.board_name / "Kconfig.board"]

    def render_top_sconscript(self, spec: Spec) -> str:
        rendered = render_board_files(
            spec=spec,
            variant=self.variant,
            sdk_root=self.sdk_root,
            output_root=self.output_root,
            assets_root=ASSETS_ROOT,
        )
        return rendered[self.output_root / spec.board_name / "SConscript"]

    def render_top_ptab(self, spec: Spec, variant: Optional[ChipVariant] = None) -> str:
        rendered = render_board_files(
            spec=spec,
            variant=variant or self.variant,
            sdk_root=self.sdk_root,
            output_root=self.output_root,
            assets_root=ASSETS_ROOT,
        )
        return rendered[self.output_root / spec.board_name / "ptab.yaml"]

    def render_top_ptab_data(self, spec: Spec, variant: Optional[ChipVariant] = None) -> dict[str, Any]:
        return yaml.safe_load(self.render_top_ptab(spec, variant=variant))

    def test_memory_summary_uses_actual_kb_for_sub_mb_storage(self) -> None:
        variant = ChipVariant(
            series="56",
            chip_dir="SF32LB56x",
            model_id="SF32LB56X",
            part_number="SF32LB561UBN26",
            memory=(
                MemoryEntry("mpi5", "nor", 512 * 1024),
                MemoryEntry("mpi1", "psram", 4 * MB),
            ),
        )

        self.assertEqual(_memory_summary_suffix(variant), " [mpi1:psram:4MB, mpi5:nor:512KB]")

    def render_ptab_offsets(self, spec: Spec, variant: ChipVariant) -> dict[str, int]:
        data = self.render_top_ptab_data(spec, variant=variant)
        return {
            str(part["name"]): self.parse_int(part["offset"])
            for part in data["partitions"]
        }

    def find_ptab_partition(self, spec: Spec, variant: ChipVariant, name: str) -> dict[str, Any]:
        data = self.render_top_ptab_data(spec, variant=variant)
        for part in data["partitions"]:
            if str(part.get("name") or "") == name:
                return part
        self.fail(f"Partition not found: {name}")

    def render_ptab_h_macros(self, spec: Spec, variant: ChipVariant) -> dict[str, str]:
        data = self.render_top_ptab_data(spec, variant=variant)
        ptab_obj = ptab.PtabV3("<in-memory>", data)
        sdk_resource.InitIndentation()
        body = sdk_resource.GenPartitionTableHeaderContentV3({"name": "main"}, ptab_obj)
        macros: dict[str, str] = {}
        for line in body.splitlines():
            line = line.strip()
            if line.startswith("#define"):
                parts = line.split(None, 2)
                if len(parts) == 3:
                    macros[parts[1]] = parts[2]
        return macros

    @staticmethod
    def parse_int(value: Any) -> int:
        if isinstance(value, int):
            return value
        return int(str(value), 0)

    def test_generated_base_uses_relative_rsource(self) -> None:
        self.make_sample_base()

        content = self.render_top_kconfig(self.make_spec("demo_board_base", generate_base=True))

        self.assertEqual(content, 'rsource "../demo_board_base/Kconfig.board"\n')

    def test_generated_base_sconscript_uses_relative_path(self) -> None:
        self.make_sample_base()

        content = self.render_top_sconscript(self.make_spec("demo_board_base", generate_base=True))

        self.assertIn("SConscript('../demo_board_base/SConscript.base', variant_dir='base', duplicate=0)", content)

    def test_existing_base_name_uses_relative_rsource(self) -> None:
        self.make_existing_base(self.output_root / "shared_base")

        content = self.render_top_kconfig(self.make_spec("shared_base"))

        self.assertEqual(content, 'rsource "../shared_base/Kconfig.board"\n')

    def test_existing_base_name_sconscript_uses_relative_path(self) -> None:
        self.make_existing_base(self.output_root / "shared_base")

        content = self.render_top_sconscript(self.make_spec("shared_base"))

        self.assertIn("SConscript('../shared_base/SConscript.base', variant_dir='base', duplicate=0)", content)

    def test_existing_relative_base_path_uses_relative_rsource(self) -> None:
        self.make_existing_base(self.output_root / "bases" / "shared_base")

        content = self.render_top_kconfig(self.make_spec("bases/shared_base"))

        self.assertEqual(content, 'rsource "../bases/shared_base/Kconfig.board"\n')

    def test_existing_relative_script_base_sconscript_uses_relative_path(self) -> None:
        self.make_existing_base(self.output_root / "bases" / "shared_base", base_kind="script")

        content = self.render_top_sconscript(self.make_spec("bases/shared_base"))

        self.assertIn("SConscript('../bases/shared_base/script/SConscript', variant_dir='base', duplicate=0)", content)

    def test_existing_absolute_base_path_uses_source(self) -> None:
        external_base = self.root / "external" / "shared_base"
        self.make_existing_base(external_base)

        content = self.render_top_kconfig(self.make_spec(str(external_base)))

        self.assertEqual(content, f'source "{external_base.resolve().as_posix()}/Kconfig.board"\n')

    def test_existing_absolute_base_sconscript_uses_absolute_path(self) -> None:
        external_base = self.root / "external" / "shared_base"
        self.make_existing_base(external_base)

        content = self.render_top_sconscript(self.make_spec(str(external_base)))

        self.assertIn(
            f"SConscript('{external_base.resolve().as_posix()}/SConscript.base', variant_dir='base', duplicate=0)",
            content,
        )

    def test_52_sdmmc_ptab_uses_sdmmc1_region(self) -> None:
        self.make_sample_base()
        variant = ChipVariant(
            series="52",
            chip_dir="SF32LB52x",
            model_id="SF32LB52X",
            part_number="SF32LB52X",
            memory=(MemoryEntry("mpi1", "psram", 16 * MB),),
        )

        content = self.render_top_ptab(
            self.make_spec(
                "demo_board_base",
                generate_base=True,
                storage_type="sdmmc",
                storage_size_mb=16,
            ),
            variant=variant,
        )

        self.assertIn("  - sdmmc: sdmmc1\n    type: sd\n", content)
        self.assertIn("    region: sdmmc1\n", content)
        self.assertNotIn("mpi: mpi", content)

    def test_58_sdmmc_ptab_uses_sdmmc2_region(self) -> None:
        self.make_sample_base(series="58")
        variant = ChipVariant(
            series="58",
            chip_dir="SF32LB58x",
            model_id="SF32LB58X",
            part_number="SF32LB58X",
            memory=(MemoryEntry("mpi1", "psram", 32 * MB),),
        )

        content = self.render_top_ptab(
            self.make_spec(
                "demo_board_base",
                generate_base=True,
                series="58",
                chip_model="SF32LB58X",
                storage_type="sdmmc",
                storage_size_mb=16,
            ),
            variant=variant,
        )

        self.assertIn("  - sdmmc: sdmmc2\n    type: sd\n", content)
        self.assertIn("    region: sdmmc2\n", content)
        self.assertNotIn("mpi: mpi4", content)

    def test_58_sdmmc1_type1_uses_sdmmc1_region_and_jlink_suffix(self) -> None:
        self.make_existing_base(self.output_root / "shared_base")
        variant = ChipVariant(
            series="58",
            chip_dir="SF32LB58x",
            model_id="SF32LB58X",
            part_number="SF32LB58X",
            memory=(MemoryEntry("mpi1", "psram", 32 * MB),),
        )
        spec = self.make_spec(
            "shared_base",
            series="58",
            chip_model="SF32LB58X",
            storage_type="sdmmc",
            storage_size_mb=16,
            storage_port="sdmmc1",
            storage_pinmux_type="type1",
        )

        rendered = render_board_files(
            spec=spec,
            variant=variant,
            sdk_root=self.sdk_root,
            output_root=self.output_root,
            assets_root=ASSETS_ROOT,
        )

        ptab = rendered[self.output_root / spec.board_name / "ptab.yaml"]
        board_conf = rendered[self.output_root / spec.board_name / "hcpu" / "board.conf"]
        rtconfig = rendered[self.output_root / spec.board_name / "hcpu" / "rtconfig.py"]
        self.assertIn("  - sdmmc: sdmmc1\n    type: sd\n", ptab)
        self.assertIn("    region: sdmmc1\n", ptab)
        self.assertIn("CONFIG_BSP_USING_SDMMC1=y\n", board_conf)
        self.assertIn("CONFIG_SD_MAX_FREQ=48000000\n", board_conf)
        self.assertNotIn("CONFIG_BSP_USING_SDMMC2=y\n", board_conf)
        self.assertIn("JLINK_DEVICE = 'SF32LB58X_SD_TYPE1'\n", rtconfig)

    def test_52_nand_ptab_reserves_factory_data(self) -> None:
        self.make_existing_base(self.output_root / "shared_base")
        variant = ChipVariant(
            series="52",
            chip_dir="SF32LB52x",
            model_id="SF32LB52X",
            part_number="SF32LB52X",
            memory=(MemoryEntry("mpi1", "psram", 16 * MB),),
        )
        spec = self.make_spec(
            "shared_base",
            series="52",
            chip_model="SF32LB52X",
            storage_type="nand",
            storage_size_mb=16,
        )

        factory_data = self.find_ptab_partition(spec, variant, "factory_data")
        hcpu_flash_code = self.find_ptab_partition(spec, variant, "hcpu_flash_code")
        ptab = self.render_top_ptab_data(spec, variant=variant)

        self.assertEqual(self.parse_int(ptab["memory"][0]["block_size"]), 0x00020000)
        self.assertEqual(factory_data["type"], "data")
        self.assertEqual(factory_data["subtype"], "raw")
        self.assertEqual(factory_data["region"], "mpi2")
        self.assertEqual(self.parse_int(factory_data["offset"]), 0x00040000)
        self.assertEqual(self.parse_int(factory_data["size"]), 0x00020000)
        self.assertEqual(factory_data["aliases"], ["FACTORY_DATA"])
        self.assertEqual(self.parse_int(hcpu_flash_code["offset"]), 0x000A0000)

    def test_52_nand_256k_ptab_reserves_256k_factory_data(self) -> None:
        self.make_existing_base(self.output_root / "shared_base")
        variant = ChipVariant(
            series="52",
            chip_dir="SF32LB52x",
            model_id="SF32LB52X",
            part_number="SF32LB52X",
            memory=(MemoryEntry("mpi1", "psram", 16 * MB),),
        )
        spec = self.make_spec(
            "shared_base",
            series="52",
            chip_model="SF32LB52X",
            storage_type="nand",
            storage_size_mb=16,
            nand_block_size_kb=256,
        )

        ptab = self.render_top_ptab_data(spec, variant=variant)
        factory_data = self.find_ptab_partition(spec, variant, "factory_data")
        hcpu_flash_code = self.find_ptab_partition(spec, variant, "hcpu_flash_code")

        self.assertEqual(self.parse_int(ptab["memory"][0]["block_size"]), 0x00040000)
        self.assertEqual(self.parse_int(factory_data["offset"]), 0x00080000)
        self.assertEqual(self.parse_int(factory_data["size"]), 0x00040000)
        self.assertEqual(factory_data["aliases"], ["FACTORY_DATA"])
        self.assertEqual(self.parse_int(hcpu_flash_code["offset"]), 0x00140000)

    def test_52_sdmmc_ptab_reserves_mbr_and_factory_data(self) -> None:
        self.make_existing_base(self.output_root / "shared_base")
        variant = ChipVariant(
            series="52",
            chip_dir="SF32LB52x",
            model_id="SF32LB52X",
            part_number="SF32LB52X",
            memory=(MemoryEntry("mpi1", "psram", 16 * MB),),
        )
        spec = self.make_spec(
            "shared_base",
            series="52",
            chip_model="SF32LB52X",
            storage_type="sdmmc",
            storage_size_mb=16,
        )

        mbr = self.find_ptab_partition(spec, variant, "mbr")
        factory_data = self.find_ptab_partition(spec, variant, "factory_data")
        hcpu_flash_code = self.find_ptab_partition(spec, variant, "hcpu_flash_code")
        fs_region = self.find_ptab_partition(spec, variant, "fs_region")
        dfu = self.find_ptab_partition(spec, variant, "dfu")
        ble = self.find_ptab_partition(spec, variant, "ble")

        self.assertEqual(mbr["type"], "data")
        self.assertEqual(mbr["subtype"], "raw")
        self.assertEqual(mbr["region"], "sdmmc1")
        self.assertEqual(self.parse_int(mbr["offset"]), 0x00000000)
        self.assertEqual(self.parse_int(mbr["size"]), 0x00001000)
        self.assertEqual(mbr["aliases"], ["MBR"])
        self.assertEqual(factory_data["type"], "data")
        self.assertEqual(factory_data["subtype"], "raw")
        self.assertEqual(factory_data["region"], "sdmmc1")
        self.assertEqual(self.parse_int(factory_data["offset"]), 0x00041000)
        self.assertEqual(self.parse_int(factory_data["size"]), 0x00020000)
        self.assertEqual(factory_data["aliases"], ["FACTORY_DATA"])
        self.assertEqual(self.parse_int(hcpu_flash_code["offset"]), 0x00071000)
        self.assertEqual(self.parse_int(fs_region["offset"]), 0x008F1000)
        self.assertEqual(self.parse_int(dfu["offset"]), 0x00CF1000)
        self.assertEqual(self.parse_int(ble["offset"]), 0x00CF5000)

    def test_nand_ptab_offsets_are_128k_aligned(self) -> None:
        self.make_existing_base(self.output_root / "shared_base")

        cases = [
            (
                "52",
                ChipVariant(
                    series="52",
                    chip_dir="SF32LB52x",
                    model_id="SF32LB52X",
                    part_number="SF32LB52X",
                    memory=(MemoryEntry("mpi1", "psram", 16 * MB),),
                ),
                self.make_spec(
                    "shared_base",
                    series="52",
                    chip_model="SF32LB52X",
                    storage_type="nand",
                    storage_size_mb=16,
                ),
                0x00CC0000,
            ),
            (
                "56",
                ChipVariant(
                    series="56",
                    chip_dir="SF32LB56x",
                    model_id="SF32LB56X",
                    part_number="SF32LB56X",
                    memory=(MemoryEntry("mpi1", "psram", 8 * MB),),
                ),
                self.make_spec(
                    "shared_base",
                    series="56",
                    chip_model="SF32LB56X",
                    storage_type="nand",
                    storage_size_mb=16,
                ),
                0x00620000,
            ),
            (
                "58",
                ChipVariant(
                    series="58",
                    chip_dir="SF32LB58x",
                    model_id="SF32LB58X",
                    part_number="SF32LB58X",
                    memory=(MemoryEntry("mpi1", "psram", 32 * MB),),
                ),
                self.make_spec(
                    "shared_base",
                    series="58",
                    chip_model="SF32LB58X",
                    storage_type="nand",
                    storage_size_mb=16,
                    storage_port="mpi3",
                ),
                0x00420000,
            ),
        ]

        for series, variant, spec, expected_ble_offset in cases:
            with self.subTest(series=series):
                offsets = self.render_ptab_offsets(spec, variant)

                for name, offset in offsets.items():
                    self.assertEqual(offset % 0x20000, 0, f"{name} offset is not 128KB aligned")
                self.assertEqual(offsets["ble"], expected_ble_offset)

    def test_nand_ptab_offsets_follow_256k_block_size(self) -> None:
        self.make_existing_base(self.output_root / "shared_base")

        cases = [
            (
                "52",
                ChipVariant(
                    series="52",
                    chip_dir="SF32LB52x",
                    model_id="SF32LB52X",
                    part_number="SF32LB52X",
                    memory=(MemoryEntry("mpi1", "psram", 16 * MB),),
                ),
                self.make_spec(
                    "shared_base",
                    series="52",
                    chip_model="SF32LB52X",
                    storage_type="nand",
                    storage_size_mb=16,
                    nand_block_size_kb=256,
                ),
                0x00D00000,
            ),
            (
                "56",
                ChipVariant(
                    series="56",
                    chip_dir="SF32LB56x",
                    model_id="SF32LB56X",
                    part_number="SF32LB56X",
                    memory=(MemoryEntry("mpi1", "psram", 8 * MB),),
                ),
                self.make_spec(
                    "shared_base",
                    series="56",
                    chip_model="SF32LB56X",
                    storage_type="nand",
                    storage_size_mb=16,
                    nand_block_size_kb=256,
                ),
                0x00680000,
            ),
            (
                "58",
                ChipVariant(
                    series="58",
                    chip_dir="SF32LB58x",
                    model_id="SF32LB58X",
                    part_number="SF32LB58X",
                    memory=(MemoryEntry("mpi1", "psram", 32 * MB),),
                ),
                self.make_spec(
                    "shared_base",
                    series="58",
                    chip_model="SF32LB58X",
                    storage_type="nand",
                    storage_size_mb=16,
                    storage_port="mpi3",
                    nand_block_size_kb=256,
                ),
                0x00440000,
            ),
        ]

        for series, variant, spec, expected_ble_offset in cases:
            with self.subTest(series=series):
                data = self.render_top_ptab_data(spec, variant=variant)
                offsets = {
                    str(part["name"]): self.parse_int(part["offset"])
                    for part in data["partitions"]
                }

                self.assertEqual(self.parse_int(data["memory"][0]["block_size"]), 0x00040000)
                for name, offset in offsets.items():
                    self.assertEqual(offset % 0x40000, 0, f"{name} offset is not 256KB aligned")
                self.assertEqual(offsets["ble"], expected_ble_offset)

    def test_sdmmc_ptab_offsets_are_512_byte_aligned(self) -> None:
        self.make_existing_base(self.output_root / "shared_base")

        cases = [
            (
                "52",
                ChipVariant(
                    series="52",
                    chip_dir="SF32LB52x",
                    model_id="SF32LB52X",
                    part_number="SF32LB52X",
                    memory=(MemoryEntry("mpi1", "psram", 16 * MB),),
                ),
                self.make_spec(
                    "shared_base",
                    series="52",
                    chip_model="SF32LB52X",
                    storage_type="sdmmc",
                    storage_size_mb=16,
                ),
            ),
            (
                "56",
                ChipVariant(
                    series="56",
                    chip_dir="SF32LB56x",
                    model_id="SF32LB56X",
                    part_number="SF32LB56X",
                    memory=(MemoryEntry("mpi1", "psram", 8 * MB),),
                ),
                self.make_spec(
                    "shared_base",
                    series="56",
                    chip_model="SF32LB56X",
                    storage_type="sdmmc",
                    storage_size_mb=16,
                ),
            ),
            (
                "58",
                ChipVariant(
                    series="58",
                    chip_dir="SF32LB58x",
                    model_id="SF32LB58X",
                    part_number="SF32LB58X",
                    memory=(MemoryEntry("mpi1", "psram", 32 * MB),),
                ),
                self.make_spec(
                    "shared_base",
                    series="58",
                    chip_model="SF32LB58X",
                    storage_type="sdmmc",
                    storage_size_mb=16,
                ),
            ),
        ]

        for series, variant, spec in cases:
            with self.subTest(series=series):
                offsets = self.render_ptab_offsets(spec, variant)

                for name, offset in offsets.items():
                    self.assertEqual(offset % 512, 0, f"{name} offset is not 512-byte aligned")

    def test_generated_58_type1_base_specializes_storage_pinmux(self) -> None:
        self.make_sample_base(series="58", real_pinmux=True)
        variant = ChipVariant(
            series="58",
            chip_dir="SF32LB58x",
            model_id="SF32LB58X",
            part_number="SF32LB58X",
            memory=(MemoryEntry("mpi1", "psram", 32 * MB),),
        )
        spec = self.make_spec(
            "demo_board_base",
            generate_base=True,
            series="58",
            chip_model="SF32LB58X",
            storage_type="nand",
            storage_size_mb=128,
            storage_port="mpi4",
            storage_pinmux_type="type1",
        )

        rendered = render_board_files(
            spec=spec,
            variant=variant,
            sdk_root=self.sdk_root,
            output_root=self.output_root,
            assets_root=ASSETS_ROOT,
        )

        pinmux = rendered[self.output_root / spec.base_name / "bsp_pinmux.c"]
        rtconfig = rendered[self.output_root / spec.board_name / "hcpu" / "rtconfig.py"]
        self.assertIn("HAL_PIN_Set(PAD_PA39, MPI4_CLK, PIN_NOPULL, 1);", pinmux)
        self.assertIn("HAL_PIN_Set(PAD_PA49, GPIO_A49, PIN_PULLUP, 1);", pinmux)
        self.assertNotIn("HAL_PIN_Set(PAD_PA09, MPI4_CLK, PIN_NOPULL, 1);", pinmux)
        self.assertNotIn("HAL_PIN_Set(PAD_PA66, GPIO_A66, PIN_PULLUP, 1);", pinmux)
        self.assertIn("JLINK_DEVICE = 'SF32LB58X_NAND'\n", rtconfig)

    def test_generated_58_sdmmc1_type0_base_uses_reference_pinmux(self) -> None:
        self.make_sample_base(series="58", real_pinmux=True)
        variant = ChipVariant(
            series="58",
            chip_dir="SF32LB58x",
            model_id="SF32LB58X",
            part_number="SF32LB58X",
            memory=(MemoryEntry("mpi1", "psram", 32 * MB),),
        )
        spec = self.make_spec(
            "demo_board_base",
            generate_base=True,
            series="58",
            chip_model="SF32LB58X",
            storage_type="sdmmc",
            storage_size_mb=128,
            storage_port="sdmmc1",
            storage_pinmux_type="type0",
        )

        rendered = render_board_files(
            spec=spec,
            variant=variant,
            sdk_root=self.sdk_root,
            output_root=self.output_root,
            assets_root=ASSETS_ROOT,
        )

        pinmux = rendered[self.output_root / spec.base_name / "bsp_pinmux.c"]
        power = rendered[self.output_root / spec.base_name / "bsp_power.c"]
        rtconfig = rendered[self.output_root / spec.board_name / "hcpu" / "rtconfig.py"]
        self.assertIn("HAL_PIN_Set(PAD_PA09, SD1_CLK, PIN_NOPULL, 1);", pinmux)
        self.assertIn("HAL_PIN_Set(PAD_PA10, SD1_CMD, PIN_PULLUP, 1);", pinmux)
        self.assertIn("HAL_PIN_Set(PAD_PA00, SD1_DIO7, PIN_PULLUP, 1);", pinmux)
        self.assertIn("HAL_PIN_Set(PAD_PA02, GPIO_A2, PIN_PULLUP, 1);", pinmux)
        self.assertIn("HAL_PIN_Set(PAD_PA12, GPIO_A12, PIN_PULLUP, 1);", pinmux)
        self.assertIn("HAL_PIN_Set(PAD_PA13, GPIO_A13, PIN_PULLUP, 1);", pinmux)
        self.assertIn("HAL_PIN_Set(PAD_PA11, GPIO_A11, PIN_PULLUP, 1);", pinmux)
        self.assertNotIn("HAL_PIN_Set(PAD_PA39, SD1_CLK, PIN_NOPULL, 1);", pinmux)
        self.assertNotIn("HAL_PIN_Set(PAD_PA66, GPIO_A66, PIN_PULLUP, 1);", pinmux)
        self.assertNotIn("HAL_PIN_Set(PAD_PA49, GPIO_A49, PIN_PULLUP, 1);", pinmux)
        self.assertIn("#define SD1_RESET_PIN       (11)\n", power)
        self.assertIn("#define SD1_EN_PIN          (2)\n", power)
        self.assertNotIn("#define SD1_RESET_PIN       (49)\n", power)
        self.assertNotIn("#define SD1_EN_PIN          (80)\n", power)
        self.assertIn("JLINK_DEVICE = 'SF32LB58X_SD'\n", rtconfig)

    def test_generated_58_mpi4_type0_base_keeps_sdmmc1_type0_pinmux(self) -> None:
        self.make_sample_base(series="58", real_pinmux=True)
        variant = ChipVariant(
            series="58",
            chip_dir="SF32LB58x",
            model_id="SF32LB58X",
            part_number="SF32LB58X",
            memory=(MemoryEntry("mpi1", "psram", 32 * MB),),
        )
        spec = self.make_spec(
            "demo_board_base",
            generate_base=True,
            series="58",
            chip_model="SF32LB58X",
            storage_type="nand",
            storage_size_mb=128,
            storage_port="mpi4",
            storage_pinmux_type="type0",
        )

        rendered = render_board_files(
            spec=spec,
            variant=variant,
            sdk_root=self.sdk_root,
            output_root=self.output_root,
            assets_root=ASSETS_ROOT,
        )

        pinmux = rendered[self.output_root / spec.base_name / "bsp_pinmux.c"]
        power = rendered[self.output_root / spec.base_name / "bsp_power.c"]
        self.assertIn("HAL_PIN_Set(PAD_PA09, MPI4_CLK, PIN_NOPULL, 1);", pinmux)
        self.assertIn("HAL_PIN_Set(PAD_PA09, SD1_CLK, PIN_NOPULL, 1);", pinmux)
        self.assertIn("HAL_PIN_Set(PAD_PA02, GPIO_A2, PIN_PULLUP, 1);", pinmux)
        self.assertIn("#define SD1_RESET_PIN       (11)\n", power)
        self.assertIn("#define SD1_EN_PIN          (2)\n", power)

    def test_generated_58_sdmmc1_type1_base_uses_reference_pinmux(self) -> None:
        self.make_sample_base(series="58", real_pinmux=True)
        variant = ChipVariant(
            series="58",
            chip_dir="SF32LB58x",
            model_id="SF32LB58X",
            part_number="SF32LB58X",
            memory=(MemoryEntry("mpi1", "psram", 32 * MB),),
        )
        spec = self.make_spec(
            "demo_board_base",
            generate_base=True,
            series="58",
            chip_model="SF32LB58X",
            storage_type="sdmmc",
            storage_size_mb=128,
            storage_port="sdmmc1",
            storage_pinmux_type="type1",
        )

        rendered = render_board_files(
            spec=spec,
            variant=variant,
            sdk_root=self.sdk_root,
            output_root=self.output_root,
            assets_root=ASSETS_ROOT,
        )

        pinmux = rendered[self.output_root / spec.base_name / "bsp_pinmux.c"]
        power = rendered[self.output_root / spec.base_name / "bsp_power.c"]
        rtconfig = rendered[self.output_root / spec.board_name / "hcpu" / "rtconfig.py"]
        self.assertIn("HAL_PIN_Set(PAD_PA39, SD1_CLK, PIN_NOPULL, 1);", pinmux)
        self.assertIn("HAL_PIN_Set(PAD_PA34, SD1_CMD, PIN_PULLUP, 1);", pinmux)
        self.assertIn("HAL_PIN_Set(PAD_PA41, SD1_DIO0, PIN_PULLUP, 1);", pinmux)
        self.assertIn("HAL_PIN_Set(PAD_PA80, GPIO_A80, PIN_PULLUP, 1);", pinmux)
        self.assertIn("HAL_PIN_Set(PAD_PA49, GPIO_A49, PIN_PULLUP, 1);", pinmux)
        self.assertNotIn("HAL_PIN_Set(PAD_PA09, SD1_CLK, PIN_NOPULL, 1);", pinmux)
        self.assertNotIn("HAL_PIN_Set(PAD_PA66, GPIO_A66, PIN_PULLUP, 1);", pinmux)
        self.assertIn("#define SD1_RESET_PIN       (49)\n", power)
        self.assertIn("#define SD1_EN_PIN          (80)\n", power)
        self.assertIn("JLINK_DEVICE = 'SF32LB58X_SD_TYPE1'\n", rtconfig)

    def test_normalize_spec_allows_absolute_existing_base_path(self) -> None:
        external_base = self.root / "external" / "shared_base"

        spec, _variant = normalize_spec(
            {
                "chip_series": "52",
                "chip_model": "SF32LB52X",
                "storage": {"type": "none"},
                "board_name": "demo_board",
                "generate_base": False,
                "base_name": str(external_base),
            },
            SCHEMA_PATH,
            {"52": [self.variant]},
        )

        self.assertEqual(spec.base_name, str(external_base))

    def test_normalize_non_58_sdmmc_does_not_require_pinmux_type(self) -> None:
        variant = ChipVariant(
            series="52",
            chip_dir="SF32LB52x",
            model_id="SF32LB52X",
            part_number="SF32LB52X",
            memory=(MemoryEntry("mpi1", "psram", 16 * MB),),
        )

        spec, _variant = normalize_spec(
            {
                "chip_series": "52",
                "chip_model": "SF32LB52X",
                "storage": {"type": "sdmmc", "size_mb": 16},
                "board_name": "demo_board",
                "generate_base": False,
                "base_name": "shared_base",
            },
            SCHEMA_PATH,
            {"52": [variant]},
        )

        self.assertEqual(spec.storage_port, "sdmmc1")
        self.assertIsNone(spec.storage_pinmux_type)

    def test_normalize_58_pinmux_type_rules(self) -> None:
        variant = ChipVariant(
            series="58",
            chip_dir="SF32LB58x",
            model_id="SF32LB58X",
            part_number="SF32LB58X",
            memory=(MemoryEntry("mpi1", "psram", 32 * MB),),
        )

        with self.assertRaisesRegex(Exception, "storage_port=mpi4 requires"):
            normalize_spec(
                {
                    "chip_series": "58",
                    "chip_model": "SF32LB58X",
                    "storage": {"type": "nor", "size_mb": 16},
                    "storage_port": "mpi4",
                    "board_name": "demo_board",
                    "generate_base": False,
                    "base_name": "shared_base",
                },
                SCHEMA_PATH,
                {"58": [variant]},
            )

        with self.assertRaisesRegex(Exception, "pinmux_type is only supported"):
            normalize_spec(
                {
                    "chip_series": "58",
                    "chip_model": "SF32LB58X",
                    "storage": {"type": "sdmmc", "size_mb": 16, "pinmux_type": "type1"},
                    "storage_port": "sdmmc2",
                    "board_name": "demo_board",
                    "generate_base": False,
                    "base_name": "shared_base",
                },
                SCHEMA_PATH,
                {"58": [variant]},
            )

        spec, _variant = normalize_spec(
            {
                "chip_series": "58",
                "chip_model": "SF32LB58X",
                "storage": {"type": "nor", "size_mb": 16, "pinmux_type": "type0"},
                "storage_port": "mpi4",
                "board_name": "demo_board",
                "generate_base": False,
                "base_name": "shared_base",
            },
            SCHEMA_PATH,
            {"58": [variant]},
        )

        self.assertEqual(spec.storage_port, "mpi4")
        self.assertEqual(spec.storage_pinmux_type, "type0")

    def test_normalize_nand_block_size_rules(self) -> None:
        variant = ChipVariant(
            series="52",
            chip_dir="SF32LB52x",
            model_id="SF32LB52X",
            part_number="SF32LB52X",
            memory=(MemoryEntry("mpi1", "psram", 16 * MB),),
        )

        default_spec, _variant = normalize_spec(
            {
                "chip_series": "52",
                "chip_model": "SF32LB52X",
                "storage": {"type": "nand", "size_mb": 16},
                "board_name": "demo_board",
                "generate_base": False,
                "base_name": "shared_base",
            },
            SCHEMA_PATH,
            {"52": [variant]},
        )
        self.assertEqual(default_spec.nand_block_size_kb, 128)

        spec, _variant = normalize_spec(
            {
                "chip_series": "52",
                "chip_model": "SF32LB52X",
                "storage": {"type": "nand", "size_mb": 16, "block_size_kb": 256},
                "board_name": "demo_board",
                "generate_base": False,
                "base_name": "shared_base",
            },
            SCHEMA_PATH,
            {"52": [variant]},
        )
        self.assertEqual(spec.nand_block_size_kb, 256)

        with self.assertRaisesRegex(Exception, "block_size_kb"):
            normalize_spec(
                {
                    "chip_series": "52",
                    "chip_model": "SF32LB52X",
                    "storage": {"type": "sdmmc", "size_mb": 16, "block_size_kb": 256},
                    "board_name": "demo_board",
                    "generate_base": False,
                    "base_name": "shared_base",
                },
                SCHEMA_PATH,
                {"52": [variant]},
            )

    def test_confirmation_summary_explains_generated_base(self) -> None:
        summary = build_confirmation_summary(
            series="52",
            chip_model="SF32LB52X",
            storage_type="none",
            storage_port=None,
            storage_size_mb=None,
            board_name="demo_board",
            base_name="demo_board_base",
            generate_base=True,
        )

        self.assertIn("[Base board]", summary)
        self.assertIn("  mode: create new base board", summary)
        self.assertIn("  base_name_or_path: demo_board_base", summary)

    def test_confirmation_summary_explains_absolute_existing_base(self) -> None:
        external_base = self.root / "external" / "shared_base"

        summary = build_confirmation_summary(
            series="52",
            chip_model="SF32LB52X",
            storage_type="nor",
            storage_port=None,
            storage_size_mb=16,
            board_name="demo_board",
            base_name=str(external_base),
            generate_base=False,
        )

        self.assertIn("  mode: reuse existing base board", summary)
        self.assertIn(f"  base_name_or_path: {external_base}", summary)

    def _make_57_variant(self) -> ChipVariant:
        return ChipVariant(
            series="57",
            chip_dir="SF32LB57x",
            model_id="SF32LB57X",
            part_number="SF32LB575YBBN6",
            memory=(MemoryEntry("mpi1", "psram", 4 * MB),),
        )

    def _make_57_nor_spec(self, **kwargs: Any) -> Spec:
        return self.make_spec(
            "demo_board_base",
            generate_base=True,
            series="57",
            chip_model="SF32LB575YBBN6",
            storage_type="nor",
            storage_size_mb=16,
            storage_port="mpi3",
            **kwargs,
        )

    def test_57_nor_ptab_uses_mpi3_region_and_xip(self) -> None:
        self.make_sample_base(series="57")
        variant = self._make_57_variant()

        content = self.render_top_ptab(self._make_57_nor_spec(), variant=variant)

        self.assertIn("  - mpi: mpi3\n    type: nor\n", content)
        app = self.find_ptab_partition(self._make_57_nor_spec(), variant, "hcpu_flash_code")
        self.assertEqual(app["region"], "mpi3")
        self.assertNotIn("exec", app)

    def test_57_nand_ptab_executes_from_psram(self) -> None:
        self.make_sample_base(series="57")
        variant = self._make_57_variant()

        app = self.find_ptab_partition(
            self.make_spec(
                "demo_board_base",
                generate_base=True,
                series="57",
                chip_model="SF32LB575YBBN6",
                storage_type="nand",
                storage_size_mb=64,
                storage_port="mpi3",
            ),
            variant,
            "hcpu_flash_code",
        )

        self.assertEqual(app["region"], "mpi3")
        self.assertEqual(app["exec"]["region"], "psram1")

    def test_57_sdmmc_ptab_uses_sdmmc1_region_and_psram_exec(self) -> None:
        self.make_sample_base(series="57")
        variant = self._make_57_variant()

        data = self.render_top_ptab_data(
            self.make_spec(
                "demo_board_base",
                generate_base=True,
                series="57",
                chip_model="SF32LB575YBBN6",
                storage_type="sdmmc",
                storage_size_mb=16,
                storage_port="sdmmc1",
            ),
            variant=variant,
        )

        self.assertEqual(data["memory"][0]["sdmmc"], "sdmmc1")
        app = next(part for part in data["partitions"] if part["name"] == "hcpu_flash_code")
        self.assertEqual(app["region"], "sdmmc1")
        self.assertEqual(app["exec"]["region"], "psram1")

    def test_57_sdmmc_ptab_uses_shifted_layout(self) -> None:
        self.make_sample_base(series="57")
        variant = self._make_57_variant()
        spec = self.make_spec(
            "demo_board_base",
            generate_base=True,
            series="57",
            chip_model=variant.part_number,
            storage_type="sdmmc",
            storage_size_mb=16,
            storage_port="sdmmc1",
        )

        hcpu_flash_code = self.find_ptab_partition(spec, variant, "hcpu_flash_code")
        fs_region = self.find_ptab_partition(spec, variant, "fs_region")
        dfu = self.find_ptab_partition(spec, variant, "dfu")
        ble = self.find_ptab_partition(spec, variant, "ble")

        self.assertEqual(self.parse_int(hcpu_flash_code["offset"]), 0x00071000)
        self.assertEqual(self.parse_int(fs_region["offset"]), 0x00931000)
        self.assertEqual(self.parse_int(dfu["offset"]), 0x00F31000)
        self.assertEqual(self.parse_int(ble["offset"]), 0x00F35000)

    def test_57_without_acpu_has_no_acpu_partition(self) -> None:
        self.make_sample_base(series="57")
        variant = self._make_57_variant()

        for storage_type, storage_port in (
            ("nor", "mpi3"),
            ("nand", "mpi3"),
            ("sdmmc", "sdmmc1"),
        ):
            with self.subTest(storage_type=storage_type):
                spec = self.make_spec(
                    "demo_board_base",
                    generate_base=True,
                    series="57",
                    chip_model=variant.part_number,
                    storage_type=storage_type,
                    storage_size_mb=128 if storage_type == "nand" else 16,
                    storage_port=storage_port,
                )
                data = self.render_top_ptab_data(spec, variant=variant)
                names = [str(part["name"]) for part in data["partitions"]]
                self.assertNotIn("acpu", names)
                self.assertNotIn("acpu_ram_data", names)

                macros = self.render_ptab_h_macros(spec, variant)
                self.assertNotIn("ACPU_RAM_DATA_START_ADDR", macros)
                self.assertNotIn("ACPU_RAM_DATA_SIZE", macros)

    def test_57_nor_acpu_partition(self) -> None:
        self.make_sample_base(series="57")
        variant = self._make_57_variant()
        spec = self.make_spec(
            "demo_board_base",
            generate_base=True,
            series="57",
            chip_model=variant.part_number,
            storage_type="nor",
            storage_size_mb=16,
            storage_port="mpi3",
            with_acpu=True,
        )

        acpu = self.find_ptab_partition(spec, variant, "acpu")
        self.assertEqual(acpu["type"], "app")
        self.assertEqual(acpu["subtype"], "factory")
        self.assertEqual(acpu["core"], "ACPU")
        self.assertEqual(acpu["region"], "mpi3")
        self.assertEqual(self.parse_int(acpu["offset"]), 0x00720000)
        self.assertEqual(self.parse_int(acpu["size"]), 0x00040000)
        self.assertEqual(acpu["aliases"], ["ACPU_CODE_LOAD_REGION1", "APP_ACPU_CODE"])
        self.assertEqual(acpu["exec"]["region"], "psram1")
        self.assertEqual(self.parse_int(acpu["exec"]["offset"]), 0x00000000)

        ram_data = self.find_ptab_partition(spec, variant, "acpu_ram_data")
        self.assertEqual(ram_data["type"], "data")
        self.assertEqual(ram_data["subtype"], "ram")
        self.assertEqual(ram_data["region"], "hpsys_ram")
        self.assertEqual(self.parse_int(ram_data["offset"]), 0x0006FC00)
        self.assertEqual(self.parse_int(ram_data["size"]), 0x0000FC00)
        self.assertEqual(ram_data["core"], "ACPU")
        self.assertEqual(ram_data["aliases"], ["ACPU_RAM_DATA"])

        macros = self.render_ptab_h_macros(spec, variant)
        self.assertIn("APP_ACPU_CODE_START_ADDR", macros)
        self.assertIn("ACPU_RAM_DATA_START_ADDR", macros)
        self.assertIn("ACPU_RAM_DATA_SIZE", macros)

        offsets = self.render_ptab_offsets(spec, variant)
        self.assertEqual(offsets["acpu"], offsets["hcpu_flash_code"] + 0x00700000)

    def test_57_nand_acpu_partition(self) -> None:
        self.make_sample_base(series="57")
        variant = self._make_57_variant()
        spec = self.make_spec(
            "demo_board_base",
            generate_base=True,
            series="57",
            chip_model=variant.part_number,
            storage_type="nand",
            storage_size_mb=128,
            storage_port="mpi3",
            with_acpu=True,
        )

        acpu = self.find_ptab_partition(spec, variant, "acpu")
        app = self.find_ptab_partition(spec, variant, "hcpu_flash_code")
        self.assertEqual(acpu["type"], "app")
        self.assertEqual(acpu["subtype"], "factory")
        self.assertEqual(acpu["core"], "ACPU")
        self.assertEqual(acpu["region"], "mpi3")
        self.assertEqual(self.parse_int(app["offset"]), 0x000A0000)
        self.assertEqual(self.parse_int(acpu["offset"]), 0x003A0000)
        self.assertEqual(self.parse_int(acpu["size"]), 0x00040000)
        self.assertEqual(acpu["exec"]["region"], "psram1")
        self.assertEqual(self.parse_int(acpu["exec"]["offset"]), 0x00300000)

    def test_57_sdmmc_acpu_partition(self) -> None:
        self.make_sample_base(series="57")
        variant = self._make_57_variant()
        spec = self.make_spec(
            "demo_board_base",
            generate_base=True,
            series="57",
            chip_model=variant.part_number,
            storage_type="sdmmc",
            storage_size_mb=16,
            storage_port="sdmmc1",
            with_acpu=True,
        )

        acpu = self.find_ptab_partition(spec, variant, "acpu")
        self.assertEqual(acpu["type"], "app")
        self.assertEqual(acpu["subtype"], "factory")
        self.assertEqual(acpu["core"], "ACPU")
        self.assertEqual(acpu["region"], "sdmmc1")
        self.assertEqual(self.parse_int(acpu["offset"]), 0x00771000)
        self.assertEqual(self.parse_int(acpu["size"]), 0x00040000)
        self.assertEqual(acpu["exec"]["region"], "psram1")
        self.assertEqual(self.parse_int(acpu["exec"]["offset"]), 0x00300000)

    def test_57_nand_256k_app_starts_after_bootloader(self) -> None:
        self.make_sample_base(series="57")
        variant = self._make_57_variant()
        spec = self.make_spec(
            "demo_board_base",
            generate_base=True,
            series="57",
            chip_model=variant.part_number,
            storage_type="nand",
            storage_size_mb=128,
            storage_port="mpi3",
            nand_block_size_kb=256,
            with_acpu=True,
        )

        app = self.find_ptab_partition(spec, variant, "hcpu_flash_code")
        acpu = self.find_ptab_partition(spec, variant, "acpu")
        bootloader_end = 4 * 0x00040000 + 0x00010000

        self.assertEqual(self.parse_int(app["offset"]), 0x00140000)
        self.assertGreaterEqual(self.parse_int(app["offset"]), bootloader_end)
        self.assertEqual(self.parse_int(acpu["offset"]), 0x00440000)
        self.assertGreaterEqual(
            self.parse_int(acpu["offset"]),
            self.parse_int(app["offset"]) + self.parse_int(app["size"]),
        )

    def test_57_jlink_devices(self) -> None:
        nor = self.make_spec(
            "demo_board_base", series="57", chip_model="SF32LB575YBBN6",
            storage_type="nor", storage_size_mb=16, storage_port="mpi3",
        )
        nand = self.make_spec(
            "demo_board_base", series="57", chip_model="SF32LB575YBBN6",
            storage_type="nand", storage_size_mb=16, storage_port="mpi3",
        )
        sdmmc = self.make_spec(
            "demo_board_base", series="57", chip_model="SF32LB575YBBN6",
            storage_type="sdmmc", storage_size_mb=16, storage_port="sdmmc1",
        )

        self.assertEqual(choose_hcpu_jlink_device(nor), "SF32LB57X_NOR")
        self.assertEqual(choose_hcpu_jlink_device(nand), "SF32LB57X_NAND")
        self.assertEqual(choose_hcpu_jlink_device(sdmmc), "SF32LB57X_SD")

    def test_57_core_definitions_include_acpu(self) -> None:
        self.assertIn(("ACPU", "BF0_ACPU", "BF0_HCPU"), core_definitions("57"))

    def test_57_board_renders_three_cores_and_part_symbol(self) -> None:
        self.make_sample_base(series="57")
        variant = self._make_57_variant()

        rendered = render_board_files(
            spec=self._make_57_nor_spec(),
            variant=variant,
            sdk_root=self.sdk_root,
            output_root=self.output_root,
            assets_root=ASSETS_ROOT,
        )

        board = self.output_root / "demo_board"
        self.assertIn(board / "hcpu" / "Kconfig.board", rendered)
        self.assertIn(board / "lcpu" / "Kconfig.board", rendered)
        self.assertIn(board / "acpu" / "Kconfig.board", rendered)

        hcpu_kconfig = rendered[board / "hcpu" / "Kconfig.board"]
        self.assertIn("select SOC_SF32LB57X\n", hcpu_kconfig)
        self.assertIn("select SOC_SF32LB575YBBN6\n", hcpu_kconfig)
        self.assertIn("select BF0_HCPU\n", hcpu_kconfig)

        acpu_kconfig = rendered[board / "acpu" / "Kconfig.board"]
        self.assertIn("select SOC_SF32LB575YBBN6\n", acpu_kconfig)
        self.assertIn("select BF0_ACPU\n", acpu_kconfig)
        self.assertIn("select BF0_HCPU\n", acpu_kconfig)

    def test_normalize_57_storage_port_rules(self) -> None:
        variant = self._make_57_variant()
        variants = {"57": [variant]}

        with self.assertRaisesRegex(Exception, "requires storage_port=mpi2"):
            normalize_spec(
                {
                    "chip_series": "57",
                    "chip_model": "SF32LB575YBBN6",
                    "storage": {"type": "nor", "size_mb": 16},
                    "storage_port": "mpi4",
                    "board_name": "demo_board",
                    "generate_base": False,
                    "base_name": "shared_base",
                },
                SCHEMA_PATH,
                variants,
            )

        with self.assertRaisesRegex(Exception, "pinmux_type is only supported"):
            normalize_spec(
                {
                    "chip_series": "57",
                    "chip_model": "SF32LB575YBBN6",
                    "storage": {"type": "sdmmc", "size_mb": 16, "pinmux_type": "type1"},
                    "storage_port": "sdmmc1",
                    "board_name": "demo_board",
                    "generate_base": False,
                    "base_name": "shared_base",
                },
                SCHEMA_PATH,
                variants,
            )

        spec, _variant = normalize_spec(
            {
                "chip_series": "57",
                "chip_model": "SF32LB575YBBN6",
                "storage": {"type": "nor", "size_mb": 16},
                "storage_port": "mpi3",
                "board_name": "demo_board",
                "generate_base": False,
                "base_name": "shared_base",
            },
            SCHEMA_PATH,
            variants,
        )
        self.assertEqual(spec.storage_port, "mpi3")

        default_spec, _variant = normalize_spec(
            {
                "chip_series": "57",
                "chip_model": "SF32LB575YBBN6",
                "storage": {"type": "nor", "size_mb": 16},
                "board_name": "demo_board",
                "generate_base": False,
                "base_name": "shared_base",
            },
            SCHEMA_PATH,
            variants,
        )
        self.assertEqual(default_spec.storage_port, "mpi3")

    def test_normalize_57_nand_rejects_mpi2(self) -> None:
        variant = self._make_57_variant()
        variants = {"57": [variant]}

        with self.assertRaisesRegex(Exception, "requires storage_port=mpi3"):
            normalize_spec(
                {
                    "chip_series": "57",
                    "chip_model": variant.part_number,
                    "storage": {"type": "nand", "size_mb": 128},
                    "storage_port": "mpi2",
                    "board_name": "demo_board",
                    "generate_base": False,
                    "base_name": "shared_base",
                },
                SCHEMA_PATH,
                variants,
            )

    def test_normalize_57_nor_allows_mpi2(self) -> None:
        variant = self._make_57_variant()

        spec, _variant = normalize_spec(
            {
                "chip_series": "57",
                "chip_model": variant.part_number,
                "storage": {"type": "nor", "size_mb": 16},
                "storage_port": "mpi2",
                "board_name": "demo_board",
                "generate_base": False,
                "base_name": "shared_base",
            },
            SCHEMA_PATH,
            {"57": [variant]},
        )

        self.assertEqual(spec.storage_port, "mpi2")

    def test_normalize_with_acpu_only_supported_for_57(self) -> None:
        with self.assertRaisesRegex(Exception, "only supported for series 57"):
            normalize_spec(
                {
                    "chip_series": "52",
                    "chip_model": "SF32LB52X",
                    "storage": {"type": "nor", "size_mb": 16},
                    "board_name": "demo_board",
                    "generate_base": False,
                    "base_name": "shared_base",
                    "with_acpu": True,
                },
                SCHEMA_PATH,
                {"52": [self.variant]},
            )

    def test_normalize_57_with_acpu_rejects_none_storage(self) -> None:
        variant = ChipVariant(
            series="57",
            chip_dir="SF32LB57x",
            model_id="SF32LB57x",
            part_number="SF32LB573UB7N6",
            memory=(
                MemoryEntry("mpi1", "psram", 4 * MB),
                MemoryEntry("mpi2", "nor", 16 * MB),
            ),
        )

        with self.assertRaisesRegex(Exception, "with_acpu is not supported"):
            normalize_spec(
                {
                    "chip_series": "57",
                    "chip_model": "SF32LB573UB7N6",
                    "storage": {"type": "none"},
                    "board_name": "demo_board",
                    "generate_base": False,
                    "base_name": "shared_base",
                    "with_acpu": True,
                },
                SCHEMA_PATH,
                {"57": [variant]},
            )

    def test_normalize_57_with_acpu_defaults_to_false(self) -> None:
        variant = self._make_57_variant()
        spec, _variant = normalize_spec(
            {
                "chip_series": "57",
                "chip_model": variant.part_number,
                "storage": {"type": "nor", "size_mb": 16},
                "board_name": "demo_board",
                "generate_base": False,
                "base_name": "shared_base",
            },
            SCHEMA_PATH,
            {"57": [variant]},
        )

        self.assertFalse(spec.with_acpu)


if __name__ == "__main__":
    unittest.main()
