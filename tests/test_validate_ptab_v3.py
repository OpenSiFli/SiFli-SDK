#!/usr/bin/env python3
#
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import os
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Any

import yaml


ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "tools", "build"))

import ptab  # noqa: E402
from validate_ptab_v3 import validate_ptab_v3  # noqa: E402


class ValidatePtabV3ReservedPartitionTests(unittest.TestCase):
    def load_ptab(self, data: dict[str, Any]):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "ptab.yaml"
            path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
            return ptab.load_ptab(path, fatal=True)

    def validation_messages(self, data: dict[str, Any]) -> list[str]:
        ptab_obj = self.load_ptab(data)
        return [str(issue) for issue in validate_ptab_v3(ptab_obj)]

    def find_partition(self, data: dict[str, Any], name: str) -> dict[str, Any]:
        ptab_obj = self.load_ptab(data)
        return next(part for part in ptab_obj.partitions if part.get("name") == name)

    @staticmethod
    def base_52_nand_partitions(factory_data: dict[str, Any] | None = None) -> dict[str, Any]:
        partitions = []
        if factory_data is not None:
            partitions.append(factory_data)
        partitions.append({
            "name": "hcpu_flash_code",
            "type": "app",
            "subtype": "factory",
            "region": "mpi2",
            "offset": 0x000A0000,
            "size": 0x00100000,
            "core": "HCPU",
        })
        return {
            "version": 3,
            "chip": "SF32LB52X",
            "memory": [
                {"mpi": "mpi2", "type": "nand", "size": 16 * 1024 * 1024},
            ],
            "partitions": partitions,
        }

    @staticmethod
    def base_52_sdmmc_partitions(
        mbr: dict[str, Any] | None = None,
        factory_data: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        partitions = []
        if mbr is not None:
            partitions.append(mbr)
        if factory_data is not None:
            partitions.append(factory_data)
        partitions.append({
            "name": "hcpu_flash_code",
            "type": "app",
            "subtype": "factory",
            "region": "sdmmc1",
            "offset": 0x00061000,
            "size": 0x00100000,
            "core": "HCPU",
        })
        return {
            "version": 3,
            "chip": "SF32LB52X",
            "memory": [
                {"sdmmc": "sdmmc1", "type": "sd", "size": 16 * 1024 * 1024},
            ],
            "partitions": partitions,
        }

    def assertNoReservationMessages(self, data: dict[str, Any]) -> None:
        messages = self.validation_messages(data)
        reservation_messages = [
            message for message in messages
            if "FACTORY_DATA" in message or "MBR" in message
        ]
        self.assertEqual(reservation_messages, [])

    def test_52_nand_injects_default_factory_data_partition(self) -> None:
        data = self.base_52_nand_partitions()
        factory_data = self.find_partition(data, "factory_data")

        self.assertEqual(factory_data["type"], "data")
        self.assertEqual(factory_data["subtype"], "raw")
        self.assertEqual(factory_data["region"], "mpi2")
        self.assertEqual(ptab.parse_size(factory_data["offset"]), 0x00040000)
        self.assertEqual(ptab.parse_size(factory_data["size"]), 0x00020000)
        self.assertEqual(factory_data["aliases"], ["FACTORY_DATA"])
        self.assertNoReservationMessages(data)

    def test_52_nand_accepts_128k_factory_data_partition(self) -> None:
        data = self.base_52_nand_partitions({
            "name": "factory_data",
            "type": "data",
            "subtype": "raw",
            "region": "mpi2",
            "offset": 0x00040000,
            "size": 0x00020000,
            "aliases": ["FACTORY_DATA"],
        })

        self.assertNoReservationMessages(data)

    def test_52_nand_accepts_256k_factory_data_partition_when_layout_does_not_overlap(self) -> None:
        data = {
            "version": 3,
            "chip": "SF32LB52X",
            "memory": [
                {"mpi": "mpi2", "type": "nand", "size": 16 * 1024 * 1024, "block_size": 0x00040000},
            ],
            "partitions": [
                {
                    "name": "flash_table",
                    "type": "ftab",
                    "region": "mpi2",
                    "offset": 0x00000000,
                    "size": 0x00040000,
                },
                {
                    "name": "factory_data",
                    "type": "data",
                    "subtype": "raw",
                    "region": "mpi2",
                    "offset": 0x00080000,
                    "size": 0x00040000,
                    "aliases": ["FACTORY_DATA"],
                },
                {
                    "name": "bootloader",
                    "type": "bootloader",
                    "region": "mpi2",
                    "offset": 0x00100000,
                    "size": 0x00010000,
                    "core": "HCPU",
                    "exec": {
                        "region": "hpsys_ram",
                        "offset": 0x00020000,
                    },
                },
                {
                    "name": "hcpu_flash_code",
                    "type": "app",
                    "subtype": "factory",
                    "region": "mpi2",
                    "offset": 0x00140000,
                    "size": 0x00100000,
                    "core": "HCPU",
                },
            ],
        }

        self.assertNoReservationMessages(data)

    def test_52_nand_rejects_256k_factory_data_without_block_size(self) -> None:
        data = self.base_52_nand_partitions({
            "name": "factory_data",
            "type": "data",
            "subtype": "raw",
            "region": "mpi2",
            "offset": 0x00080000,
            "size": 0x00040000,
            "aliases": ["FACTORY_DATA"],
        })

        messages = self.validation_messages(data)

        self.assertTrue(any("FACTORY_DATA" in message and "0x40000" in message for message in messages), messages)

    def test_52_nand_256k_default_partitions_follow_v2_layout(self) -> None:
        data = {
            "version": 3,
            "chip": "SF32LB52X",
            "memory": [
                {"mpi": "mpi2", "type": "nand", "size": 16 * 1024 * 1024, "block_size": 0x00040000},
            ],
            "partitions": [
                {
                    "name": "hcpu_flash_code",
                    "type": "app",
                    "subtype": "factory",
                    "region": "mpi2",
                    "offset": 0x00140000,
                    "size": 0x00100000,
                    "core": "HCPU",
                },
            ],
        }

        ptab_obj = self.load_ptab(data)
        partitions = ptab_obj.partitions
        flash_table = next(part for part in partitions if part.get("name") == "flash_table")
        factory_data = next(part for part in partitions if part.get("name") == "factory_data")
        bootloader = next(part for part in partitions if part.get("name") == "bootloader")
        messages = [str(issue) for issue in validate_ptab_v3(ptab_obj)]

        # The flash table keeps the v2 size (0x8000); only the factory data
        # and the bootloader are aligned to the 256K erase block.
        self.assertEqual(ptab.parse_size(flash_table["size"]), 0x00008000)
        self.assertEqual(ptab.parse_size(factory_data["offset"]), 0x00080000)
        self.assertEqual(ptab.parse_size(factory_data["size"]), 0x00040000)
        self.assertEqual(ptab.parse_size(bootloader["offset"]), 0x00100000)
        self.assertFalse(any("overlap" in message for message in messages), messages)

    def test_nand_rejects_invalid_memory_block_size(self) -> None:
        data = self.base_52_nand_partitions({
            "name": "factory_data",
            "type": "data",
            "subtype": "raw",
            "region": "mpi2",
            "offset": 0x00040000,
            "size": 0x00020000,
            "aliases": ["FACTORY_DATA"],
        })
        data["memory"][0]["block_size"] = 0x00030000

        messages = self.validation_messages(data)

        self.assertTrue(any("block_size" in message and "0x20000" in message and "0x40000" in message for message in messages), messages)

    def test_nand_rejects_zero_memory_block_size_without_crashing(self) -> None:
        data = self.base_52_nand_partitions({
            "name": "factory_data",
            "type": "data",
            "subtype": "raw",
            "region": "mpi2",
            "offset": 0x00040000,
            "size": 0x00020000,
            "aliases": ["FACTORY_DATA"],
        })
        data["memory"][0]["block_size"] = 0

        messages = self.validation_messages(data)

        self.assertTrue(any("block_size" in message and "0x20000" in message and "0x40000" in message for message in messages), messages)

    def test_nand_rejects_unaligned_partition_when_block_size_is_explicit(self) -> None:
        data = self.base_52_nand_partitions({
            "name": "factory_data",
            "type": "data",
            "subtype": "raw",
            "region": "mpi2",
            "offset": 0x00040000,
            "size": 0x00020000,
            "aliases": ["FACTORY_DATA"],
        })
        data["memory"][0]["block_size"] = 0x00040000
        data["partitions"][1]["offset"] = 0x000A0000

        messages = self.validation_messages(data)

        self.assertTrue(any("hcpu_flash_code" in message and "not aligned" in message for message in messages), messages)

    def test_52_nand_256k_rejects_wrong_explicit_boot_layout(self) -> None:
        data = {
            "version": 3,
            "chip": "SF32LB52X",
            "memory": [
                {"mpi": "mpi2", "type": "nand", "size": 16 * 1024 * 1024, "block_size": 0x00040000},
            ],
            "partitions": [
                {
                    "name": "flash_table",
                    "type": "ftab",
                    "region": "mpi2",
                    "offset": 0x00000000,
                    "size": 0x00020000,
                },
                {
                    "name": "factory_data",
                    "type": "data",
                    "subtype": "raw",
                    "region": "mpi2",
                    "offset": 0x00080000,
                    "size": 0x00040000,
                    "aliases": ["FACTORY_DATA"],
                },
                {
                    "name": "bootloader",
                    "type": "bootloader",
                    "region": "mpi2",
                    "offset": 0x000C0000,
                    "size": 0x00010000,
                    "core": "HCPU",
                    "exec": {
                        "region": "hpsys_ram",
                        "offset": 0x00020000,
                    },
                },
                {
                    "name": "hcpu_flash_code",
                    "type": "app",
                    "subtype": "factory",
                    "region": "mpi2",
                    "offset": 0x00140000,
                    "size": 0x00100000,
                    "core": "HCPU",
                },
            ],
        }

        messages = self.validation_messages(data)

        self.assertTrue(any("flash_table" in message and "size=0x8000" in message for message in messages), messages)
        self.assertTrue(any("bootloader" in message and "offset=0x100000" in message for message in messages), messages)

    def test_52_nand_real_board_ptabs_accept_v2_flash_table_size(self) -> None:
        # Both NAND boards rely on the injected 0x8000 flash table while their
        # 128K erase block is larger; the rest of the block stays reserved.
        for board in ("sf32lb52-lcd_a128r16", "sf32lb52-nano_a128r16"):
            ptab_path = Path(ROOT) / "customer" / "boards" / board / "ptab.yaml"
            ptab_obj = ptab.load_ptab(ptab_path, fatal=True)

            flash_table = next(
                part for part in ptab_obj.partitions if part.get("name") == "flash_table"
            )
            self.assertEqual(ptab.parse_size(flash_table["size"]), 0x00008000, board)

            messages = [str(issue) for issue in validate_ptab_v3(ptab_obj)]
            self.assertFalse(
                any("flash_table" in message for message in messages),
                f"{board}: {messages}",
            )

    def test_non_nand_rejects_memory_block_size(self) -> None:
        data = self.base_52_sdmmc_partitions(
            mbr={
                "name": "mbr",
                "type": "data",
                "subtype": "raw",
                "region": "sdmmc1",
                "offset": 0x00000000,
                "size": 0x00001000,
                "aliases": ["MBR"],
            },
            factory_data={
                "name": "factory_data",
                "type": "data",
                "subtype": "raw",
                "region": "sdmmc1",
                "offset": 0x00041000,
                "size": 0x00020000,
                "aliases": ["FACTORY_DATA"],
            },
        )
        data["memory"][0]["block_size"] = 0x00020000

        messages = self.validation_messages(data)

        self.assertTrue(any("block_size" in message and "NAND" in message for message in messages), messages)

    def test_52_sdmmc_injects_default_mbr_partition(self) -> None:
        data = self.base_52_sdmmc_partitions(factory_data={
            "name": "factory_data",
            "type": "data",
            "subtype": "raw",
            "region": "sdmmc1",
            "offset": 0x00041000,
            "size": 0x00020000,
            "aliases": ["FACTORY_DATA"],
        })
        mbr = self.find_partition(data, "mbr")

        self.assertEqual(mbr["type"], "data")
        self.assertEqual(mbr["subtype"], "raw")
        self.assertEqual(mbr["region"], "sdmmc1")
        self.assertEqual(ptab.parse_size(mbr["offset"]), 0x00000000)
        self.assertEqual(ptab.parse_size(mbr["size"]), 0x00001000)
        self.assertEqual(mbr["aliases"], ["MBR"])
        self.assertNoReservationMessages(data)

    def test_52_sdmmc_injects_default_factory_data_partition(self) -> None:
        data = self.base_52_sdmmc_partitions(mbr={
            "name": "mbr",
            "type": "data",
            "subtype": "raw",
            "region": "sdmmc1",
            "offset": 0x00000000,
            "size": 0x00001000,
            "aliases": ["MBR"],
        })
        factory_data = self.find_partition(data, "factory_data")

        self.assertEqual(factory_data["type"], "data")
        self.assertEqual(factory_data["subtype"], "raw")
        self.assertEqual(factory_data["region"], "sdmmc1")
        self.assertEqual(ptab.parse_size(factory_data["offset"]), 0x00041000)
        self.assertEqual(ptab.parse_size(factory_data["size"]), 0x00020000)
        self.assertEqual(factory_data["aliases"], ["FACTORY_DATA"])
        self.assertNoReservationMessages(data)

    def test_52_sdmmc_rejects_reserved_partition_without_data_raw_type(self) -> None:
        data = self.base_52_sdmmc_partitions(
            mbr={
                "name": "mbr",
                "type": "data",
                "subtype": "raw",
                "region": "sdmmc1",
                "offset": 0x00000000,
                "size": 0x00001000,
                "aliases": ["MBR"],
            },
            factory_data={
                "name": "factory_data",
                "type": "data",
                "subtype": "filesystem",
                "region": "sdmmc1",
                "offset": 0x00041000,
                "size": 0x00020000,
                "aliases": ["FACTORY_DATA"],
            },
        )

        messages = self.validation_messages(data)

        self.assertTrue(any("FACTORY_DATA" in message and "data" in message and "raw" in message for message in messages), messages)

    def test_52_sdmmc_accepts_mbr_and_factory_data_partitions(self) -> None:
        data = self.base_52_sdmmc_partitions(
            mbr={
                "name": "mbr",
                "type": "data",
                "subtype": "raw",
                "region": "sdmmc1",
                "offset": 0x00000000,
                "size": 0x00001000,
                "aliases": ["MBR"],
            },
            factory_data={
                "name": "factory_data",
                "type": "data",
                "subtype": "raw",
                "region": "sdmmc1",
                "offset": 0x00041000,
                "size": 0x00020000,
                "aliases": ["FACTORY_DATA"],
            },
        )

        self.assertNoReservationMessages(data)


if __name__ == "__main__":
    unittest.main()
