#!/usr/bin/env python
#
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import hashlib
import io
import json
import struct
import sys
import tempfile
import unittest
from contextlib import redirect_stderr
from pathlib import Path
from unittest import mock

from click.testing import CliRunner

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

from sdk_py_actions import crash_dump  # noqa: E402
from sdk_py_actions import crash_dump_ext  # noqa: E402
from sdk_py_actions.sifli_uart_dap import SifliUartDap  # noqa: E402
from sdk_py_actions.cli.context import SdkConfig  # noqa: E402
from sdk_py_actions.cli.registry import CommandRegistry  # noqa: E402


def make_dump(*blocks: tuple[int, bytes], chip: str = "52X") -> bytes:
    header = f"{chip}_2026_07_03 12:34:56".encode().ljust(24, b"\0")
    payload = b"".join(
        struct.pack("<II", address, len(data)) + data for address, data in blocks
    )
    return header + payload + struct.pack("<II", 0xFFFFFFFF, 0xFFFFFFFF)


class CoredumpParserTests(unittest.TestCase):
    def test_parse_coredump_blocks_and_erased_tail(self) -> None:
        parsed = crash_dump.parse_coredump(
            make_dump((0x20000000, b"abcd"), (0x20000010, b"ef"))
        )

        self.assertEqual("52X", parsed["chip"])
        self.assertEqual("2026_07_03 12:34:56", parsed["timestamp"])
        self.assertEqual(
            [0x20000000, 0x20000010], [block["address"] for block in parsed["blocks"]]
        )
        self.assertEqual(
            [b"abcd", b"ef"], [block["data"] for block in parsed["blocks"]]
        )
        self.assertEqual([], parsed["warnings"])

    def test_minidump_tail_stops_without_consuming_padding(self) -> None:
        raw = make_dump((0x20000000, b"abcd"))[:-8]
        raw += struct.pack("<II", crash_dump.MINIDUMP_TAIL_MAGIC, 128)

        parsed = crash_dump.parse_coredump(raw)

        self.assertTrue(parsed["complete"])
        self.assertEqual(1, len(parsed["blocks"]))

    def test_ramlog_blocks_are_not_memory_segments(self) -> None:
        parsed = crash_dump.parse_coredump(
            make_dump(
                (0x20000000, b"ram"),
                (crash_dump.RAMLOG_MINIMUM_MAGIC, b"log-a"),
                (crash_dump.RAMLOG_REMAINING_MAGIC, b"log-b"),
            )
        )

        self.assertEqual([0x20000000], [block["address"] for block in parsed["blocks"]])
        self.assertEqual(
            [
                {"kind": "minimum", "data": b"log-a"},
                {"kind": "remaining", "data": b"log-b"},
            ],
            parsed["ramlogs"],
        )

    def test_truncated_block_is_reported(self) -> None:
        raw = make_dump()[:-8] + struct.pack("<II", 0x20000000, 8) + b"abcd"

        parsed = crash_dump.parse_coredump(raw)

        self.assertFalse(parsed["complete"])
        self.assertIn("truncated", parsed["warnings"][0])

    def test_memory_image_reads_across_one_segment(self) -> None:
        image = crash_dump.MemoryImage([{"address": 0x20000000, "data": b"abcdefgh"}])

        self.assertEqual(b"cdef", image.read(0x20000002, 4))
        self.assertIsNone(image.read(0x20000006, 4))

    def test_write_package_records_hashes_and_segments(self) -> None:
        raw = make_dump((0x20000000, b"abcd"))
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest = crash_dump.write_package(raw, Path(temp_dir), source="file")

            saved = json.loads((Path(temp_dir) / "manifest.json").read_text())
            segment_path = Path(temp_dir) / saved["segments"][0]["file"]
            self.assertEqual("SF32LB52", saved["chip"])
            self.assertEqual(
                hashlib.sha256(b"abcd").hexdigest(), saved["segments"][0]["sha256"]
            )
            self.assertEqual(b"abcd", segment_path.read_bytes())
            self.assertEqual(manifest, saved)

    def test_write_package_preserves_ramlogs_separately(self) -> None:
        raw = make_dump(
            (crash_dump.RAMLOG_MINIMUM_MAGIC, b"log-a"),
            (crash_dump.RAMLOG_REMAINING_MAGIC, b"log-b"),
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            manifest = crash_dump.write_package(raw, root, source="file")

            self.assertEqual([], manifest["segments"])
            self.assertEqual(
                ["ramlogs/minimum.bin", "ramlogs/remaining.bin"],
                [item["file"] for item in manifest["ramlogs"]],
            )
            self.assertEqual(b"log-a", (root / manifest["ramlogs"][0]["file"]).read_bytes())

    def test_write_package_records_elf_and_log_attachments(self) -> None:
        raw = make_dump((0x20000000, b"abcd"))
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            elf = root / "app.axf"
            log = root / "serial.log"
            elf.write_bytes(b"elf")
            log.write_text("assert")

            manifest = crash_dump.write_package(
                raw,
                root / "package",
                source="file",
                elf_path=elf,
                log_path=log,
            )

            self.assertEqual("attachments/app.axf", manifest["elf"]["file"])
            self.assertEqual(
                hashlib.sha256(b"elf").hexdigest(), manifest["elf"]["sha256"]
            )
            self.assertEqual("attachments/serial.log", manifest["log"]["file"])

    def test_attachment_cannot_overwrite_raw_dump(self) -> None:
        raw = make_dump((0x20000000, b"abcd"))
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            elf = root / "coredump.bin"
            elf.write_bytes(b"elf")

            crash_dump.write_package(
                raw,
                root / "package",
                source="file",
                elf_path=elf,
            )

            self.assertEqual(raw, (root / "package" / "coredump.bin").read_bytes())

    def test_write_legacy_layout_restores_original_file_names(self) -> None:
        raw = make_dump((0x20000000, b"ram!"))
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            package = root / "package"
            elf = root / "app.elf"
            debug_log = root / "capture.log"
            elf.write_bytes(b"elf")
            debug_log.write_text("registers")
            manifest = crash_dump.write_package(raw, package, source="file")

            legacy = crash_dump.write_legacy_layout(
                package,
                manifest,
                [
                    crash_dump.Region("hcpu_ram.bin", 0x20000000, 4),
                    crash_dump.Region("psram.bin", 0x60000000, 4),
                ],
                elf_path=elf,
                debug_log_path=debug_log,
            )

            self.assertEqual(b"ram!", (package / "hcpu_ram.bin").read_bytes())
            self.assertEqual(b"elf", (package / "hcpu.axf").read_bytes())
            self.assertEqual("registers", (package / "log.txt").read_text())
            self.assertEqual(["psram.bin"], legacy["missing"])
            assertdump_manifest = json.loads((package / "manifest.json").read_text())
            self.assertEqual("1.0.0", assertdump_manifest["schemaVersion"])
            self.assertEqual("SF32LB52", assertdump_manifest["chip"]["modelId"])
            self.assertEqual(
                ["hcpu_ram.bin"],
                [item["fileName"] for item in assertdump_manifest["files"]],
            )
            sdk_manifest = json.loads((package / "sdk_manifest.json").read_text())
            self.assertEqual(legacy, sdk_manifest["legacy"])
            self.assertEqual(1, sdk_manifest["schema_version"])

    def test_default_legacy_script_uses_assertdump_layout(self) -> None:
        script = crash_dump.default_jlink_script(
            ROOT,
            "SF32LB52",
            legacy_layout=True,
        )

        self.assertEqual(ROOT / "tools" / "AssertDump" / "sf32lb52x_psramless.jlink", script)

    def test_default_legacy_script_supports_sf32lb57(self) -> None:
        script = crash_dump.default_jlink_script(
            ROOT,
            "SF32LB57",
            legacy_layout=True,
        )

        self.assertEqual(ROOT / "tools" / "AssertDump" / "sf32lb57x_psramless.jlink", script)
        self.assertTrue(script.is_file())

    def test_default_legacy_script_can_select_psram_size(self) -> None:
        script = crash_dump.default_jlink_script(
            ROOT,
            "SF32LB52",
            legacy_layout=True,
            psram_size="8MB",
        )

        self.assertEqual(ROOT / "tools" / "AssertDump" / "sf32lb52x_psram_8MB.jlink", script)
        regions = crash_dump.parse_jlink_regions(script.read_text(encoding="utf-8"))
        self.assertIn("psram.bin", [region.filename for region in regions])

    def test_default_legacy_script_model_defaults_without_psram(self) -> None:
        script = crash_dump.default_jlink_script(
            ROOT,
            "SF32LB52",
            legacy_layout=True,
            chip_model="LB525",
        )

        self.assertEqual(ROOT / "tools" / "AssertDump" / "sf32lb52x_psramless.jlink", script)

    def test_default_legacy_script_model_can_include_psram(self) -> None:
        script = crash_dump.default_jlink_script(
            ROOT,
            "SF32LB52",
            legacy_layout=True,
            chip_model="LB525",
            include_psram=True,
        )

        self.assertEqual(ROOT / "tools" / "AssertDump" / "sf32lb52x_psram_8MB.jlink", script)

    def test_default_legacy_script_uses_model_psramless_mapping(self) -> None:
        script = crash_dump.default_jlink_script(
            ROOT,
            "SF32LB52",
            legacy_layout=True,
            chip_model="LB520",
        )

        self.assertEqual(ROOT / "tools" / "AssertDump" / "sf32lb52x_psramless.jlink", script)

    def test_write_legacy_layout_rejects_escaping_file_name(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            package = root / "package"
            manifest = crash_dump.write_package(
                make_dump((0x20000000, b"data")), package, source="file"
            )

            with self.assertRaisesRegex(Exception, "escapes"):
                crash_dump.write_legacy_layout(
                    package,
                    manifest,
                    [crash_dump.Region("../outside.bin", 0x20000000, 4)],
                )


class AnalysisTests(unittest.TestCase):
    def test_decode_saved_stack_frame_uses_final_exception_words(self) -> None:
        words = list(range(10)) + [10, 11, 12, 13, 14, 15, 16, 17]
        decoded = crash_dump.decode_saved_stack_frame(struct.pack("<18I", *words))

        self.assertEqual(9, decoded["r0"])
        self.assertEqual(14, decoded["lr"])
        self.assertEqual(15, decoded["pc"])
        self.assertEqual(16, decoded["xpsr"])
        self.assertEqual(1, decoded["r4"])
        self.assertEqual(8, decoded["r11"])

    def test_decode_saved_stack_frame_rejects_partial_full_frame(self) -> None:
        with self.assertRaisesRegex(ValueError, "32 or at least 68"):
            crash_dump.decode_saved_stack_frame(bytes(40))

    def test_decode_scb_registers(self) -> None:
        decoded = crash_dump.decode_scb_registers(struct.pack("<5I", 1, 2, 3, 4, 5))

        self.assertEqual(
            {"cfsr": 1, "hfsr": 2, "mmfar": 3, "bfar": 4, "shcsr": 5},
            decoded,
        )

    def test_parse_jlink_registers(self) -> None:
        registers = crash_dump.parse_jlink_registers(
            """
            R0 = 00000001, R1 = 00000002, R12 = 0000000D
            R13 (SP) = 20001000, R14 (LR) = 12000101
            R15 (PC) = 12000201, XPSR = 21000000
            """
        )

        self.assertEqual(1, registers["r0"])
        self.assertEqual(0x20001000, registers["sp"])
        self.assertEqual(0x12000101, registers["lr"])
        self.assertEqual(0x12000201, registers["pc"])
        self.assertEqual(0x21000000, registers["xpsr"])

    def test_symbolize_and_stack_candidates(self) -> None:
        symbols = [
            crash_dump.ElfSymbol("alpha", 0x1000, 0x20, "function"),
            crash_dump.ElfSymbol("beta", 0x1100, 0x10, "function"),
        ]

        self.assertEqual("alpha+0x4", crash_dump.symbolize(0x1004, symbols))
        candidates = crash_dump.find_stack_candidates(
            struct.pack("<4I", 0, 0x1005, 0xDEADBEEF, 0x1101),
            symbols,
        )
        self.assertEqual(
            ["alpha+0x5", "beta+0x1"], [item["symbol"] for item in candidates]
        )

    def test_analyze_package_reads_named_fault_objects(self) -> None:
        frame = struct.pack(
            "<18I", *(list(range(9)) + [1, 2, 3, 4, 5, 0x1008, 0x1004, 0x21000000, 0])
        )
        raw = make_dump(
            (0x2000, frame),
            (0x2100, struct.pack("<II", 0x2200, 2)),
            (0x2120, struct.pack("<5I", 1, 2, 3, 4, 5)),
            (0x2200, struct.pack("<2I", 0x100C, 0)),
        )
        symbols = [
            crash_dump.ElfSymbol("saved_stack_frame", 0x2000, len(frame), "object"),
            crash_dump.ElfSymbol("saved_stack_pointer", 0x2100, 4, "object"),
            crash_dump.ElfSymbol("error_reason", 0x2104, 4, "object"),
            crash_dump.ElfSymbol("saved_scb_reg", 0x2120, 20, "object"),
            crash_dump.ElfSymbol("faulty", 0x1000, 0x20, "function"),
        ]
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            package = root / "package"
            crash_dump.write_package(raw, package, "file")
            elf = root / "app.elf"
            elf.write_bytes(b"elf")

            with mock.patch.object(
                crash_dump, "load_elf_symbols", return_value=symbols
            ):
                analysis = crash_dump.analyze_package(package, elf)

            self.assertEqual("faulty+0x4", analysis["pc"]["symbol"])
            self.assertEqual(2, analysis["error_reason"])
            self.assertEqual("0x00002200", analysis["stack_pointer"])
            self.assertEqual("faulty+0xc", analysis["stack_candidates"][0]["symbol"])
            self.assertEqual("0x00001004", analysis["saved_registers"]["pc"])

    def test_analyze_package_keeps_current_and_saved_registers(self) -> None:
        saved_frame = struct.pack(
            "<18I", *(list(range(9)) + [1, 2, 3, 4, 5, 0x1008, 0x1004, 0x21000000, 0])
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            package = root / "package"
            elf = root / "app.elf"
            elf.write_bytes(b"elf")
            manifest = crash_dump.write_package(
                make_dump((0x2000, saved_frame), (0x3000, struct.pack("<I", 0x12000201))),
                package,
                "file",
                elf_path=elf,
            )
            manifest["current_register_source"] = "uart_dap"
            manifest["current_registers"] = {
                "sp": "0x00003000",
                "lr": "0x12000101",
                "pc": "0x12000201",
                "xpsr": "0x21000000",
            }
            manifest["register_source"] = manifest["current_register_source"]
            manifest["registers"] = manifest["current_registers"]
            (package / "sdk_manifest.json").write_text(json.dumps(manifest))
            symbols = [
                crash_dump.ElfSymbol("saved_stack_frame", 0x2000, len(saved_frame), "object"),
                crash_dump.ElfSymbol("saved", 0x1000, 0x20, "function"),
                crash_dump.ElfSymbol("current", 0x12000200, 0x20, "function"),
            ]

            with mock.patch.object(crash_dump, "load_elf_symbols", return_value=symbols):
                analysis = crash_dump.analyze_package(package, elf)

            self.assertEqual("uart_dap", analysis["current_register_source"])
            self.assertEqual("0x12000201", analysis["current_registers"]["pc"])
            self.assertEqual("0x00001004", analysis["saved_registers"]["pc"])
            self.assertEqual("uart_dap", analysis["register_source"])
            self.assertEqual("current+0x1", analysis["pc"]["symbol"])

    def test_analyze_package_keeps_legacy_registers_out_of_current_fields(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            package = root / "package"
            elf = root / "app.elf"
            elf.write_bytes(b"elf")
            manifest = crash_dump.write_package(
                make_dump((0x3000, struct.pack("<I", 0x12000201))),
                package,
                "file",
                elf_path=elf,
            )
            manifest["register_source"] = "debug_log"
            manifest["registers"] = {
                "sp": "0x00003000",
                "lr": "0x12000101",
                "pc": "0x12000201",
                "xpsr": "0x21000000",
            }
            (package / "sdk_manifest.json").write_text(json.dumps(manifest))
            symbols = [
                crash_dump.ElfSymbol("current", 0x12000200, 0x20, "function"),
            ]

            with mock.patch.object(crash_dump, "load_elf_symbols", return_value=symbols):
                analysis = crash_dump.analyze_package(package, elf)

            self.assertNotIn("current_registers", analysis)
            self.assertNotIn("current_register_source", analysis)
            self.assertEqual("debug_log", analysis["register_source"])
            self.assertEqual("current+0x1", analysis["pc"]["symbol"])

    def test_analyze_package_uses_attached_elf(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            package = root / "package"
            elf = root / "app.elf"
            elf.write_bytes(b"elf")
            crash_dump.write_package(
                make_dump((0x20000000, b"data")),
                package,
                "file",
                elf_path=elf,
            )

            with mock.patch.object(
                crash_dump, "load_elf_symbols", return_value=[]
            ) as load_symbols:
                analysis = crash_dump.analyze_package(package)

            load_symbols.assert_called_once_with(
                (package / "attachments" / "app.elf").resolve()
            )
            self.assertEqual(
                hashlib.sha256(b"elf").hexdigest(), analysis["elf"]["sha256"]
            )

    def test_analyze_package_tolerates_registers_without_pc_or_lr(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            package = root / "package"
            elf = root / "app.elf"
            elf.write_bytes(b"elf")
            manifest = crash_dump.write_package(
                make_dump((0x20000000, b"data")),
                package,
                "file",
                elf_path=elf,
            )
            manifest["registers"] = {"r0": "0x1", "sp": "0x20000000"}
            (package / "sdk_manifest.json").write_text(json.dumps(manifest))

            with mock.patch.object(crash_dump, "load_elf_symbols", return_value=[]):
                analysis = crash_dump.analyze_package(package, engine="python")

            self.assertEqual("0x00000001", analysis["registers"]["r0"])
            self.assertNotIn("pc", analysis)
            self.assertNotIn("lr", analysis)

    def test_python_analysis_preserves_existing_core_elf(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            package = root / "package"
            elf = root / "app.elf"
            elf.write_bytes(b"elf")
            crash_dump.write_package(
                make_dump((0x20000000, b"data")),
                package,
                "file",
                elf_path=elf,
            )
            core = package / "coredump.elf"
            core.write_bytes(b"existing core")

            with mock.patch.object(crash_dump, "load_elf_symbols", return_value=[]):
                crash_dump.analyze_package(package, engine="python")

            self.assertEqual(b"existing core", core.read_bytes())

    def test_analyze_package_prefers_sdk_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            package = root / "package"
            elf = root / "app.elf"
            elf.write_bytes(b"elf")
            crash_dump.write_package(
                make_dump((0x20000000, b"data")),
                package,
                "file",
                elf_path=elf,
            )
            (package / "sdk_manifest.json").write_text(
                (package / "manifest.json").read_text()
            )
            (package / "manifest.json").write_text(
                json.dumps({"schemaVersion": "1.0.0"})
            )

            with mock.patch.object(
                crash_dump, "load_elf_symbols", return_value=[]
            ):
                analysis = crash_dump.analyze_package(package)

            self.assertEqual("SF32LB52", analysis["chip"])

    def test_analyze_package_rejects_modified_attachment(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            package = root / "package"
            elf = root / "app.elf"
            elf.write_bytes(b"elf")
            crash_dump.write_package(
                make_dump((0x20000000, b"data")),
                package,
                "file",
                elf_path=elf,
            )
            (package / "attachments" / "app.elf").write_bytes(b"bad")

            with self.assertRaisesRegex(Exception, "sha256"):
                crash_dump.analyze_package(package)

    def test_analyze_package_falls_back_to_jlink_register_log(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            package = root / "package"
            elf = root / "app.elf"
            debug_log = root / "capture.log"
            elf.write_bytes(b"elf")
            debug_log.write_text(
                "R0 = 00000001, R13 (SP) = 20001000, "
                "R14 (LR) = 12000101, R15 (PC) = 12000201, XPSR = 21000000"
            )
            manifest = crash_dump.write_package(
                make_dump((0x20001000, struct.pack("<I", 0x12000201))),
                package,
                "file",
                elf_path=elf,
                log_path=debug_log,
            )
            manifest["debug_log"] = manifest.pop("log")
            (package / "manifest.json").write_text(json.dumps(manifest))
            symbols = [
                crash_dump.ElfSymbol("faulty", 0x12000200, 0x20, "function"),
            ]

            with mock.patch.object(
                crash_dump, "load_elf_symbols", return_value=symbols
            ):
                analysis = crash_dump.analyze_package(package)

            self.assertEqual("debug_log", analysis["register_source"])
            self.assertEqual("faulty+0x1", analysis["pc"]["symbol"])
            self.assertEqual("0x20001000", analysis["stack_pointer"])
            self.assertEqual("faulty+0x1", analysis["stack_candidates"][0]["symbol"])

    def test_write_core_elf_exports_load_segments_and_register_note(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            package = root / "package"
            elf = root / "app.elf"
            elf.write_bytes(b"elf")
            manifest = crash_dump.write_package(
                make_dump((0x20001000, b"stack-data")),
                package,
                "file",
                elf_path=elf,
            )
            manifest["registers"] = {
                "r0": "0x00000001",
                "sp": "0x20001000",
                "lr": "0x12000101",
                "pc": "0x12000201",
                "xpsr": "0x21000000",
            }
            (package / "sdk_manifest.json").write_text(json.dumps(manifest))

            core = crash_dump.write_core_elf(package, elf, root / "coredump.elf")

            data = core.read_bytes()
            self.assertEqual(b"\x7fELF", data[:4])
            self.assertIn(b"CORE", data)
            self.assertIn(b"stack-data", data)
            self.assertEqual(core, (root / "coredump.elf").resolve())

    def test_core_prstatus_uses_arm_linux_note_size(self) -> None:
        self.assertEqual(148, len(crash_dump._core_prstatus({"pc": 0x12000101})))

    def test_write_core_elf_restores_sp_from_saved_stack_pointer(self) -> None:
        frame = struct.pack(
            "<8I",
            1,
            2,
            3,
            4,
            0x12048C8D,
            0x12041A83,
            0x12041A82,
            0x40000000,
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            package = root / "package"
            elf = root / "app.elf"
            elf.write_bytes(b"elf")
            crash_dump.write_package(
                make_dump(
                    (0x20000000, frame),
                    (0x20000100, struct.pack("<I", 0x200145B8)),
                ),
                package,
                "file",
                elf_path=elf,
            )
            symbols = [
                crash_dump.ElfSymbol("saved_stack_frame", 0x20000000, len(frame), "object"),
                crash_dump.ElfSymbol("saved_stack_pointer", 0x20000100, 4, "object"),
            ]

            with mock.patch.object(crash_dump, "load_elf_symbols", return_value=symbols):
                core = crash_dump.write_core_elf(package, elf, root / "coredump.elf")

            data = core.read_bytes()
            note_offset = 52 + 32 * 3
            namesz, descsz, note_type = struct.unpack_from("<III", data, note_offset)
            desc_offset = note_offset + 12 + ((namesz + 3) & ~3)
            desc = data[desc_offset : desc_offset + descsz]
            words = struct.unpack("<37I", desc)
            self.assertEqual(1, note_type)
            self.assertEqual(0x200145B8, words[31])

    def test_analyze_package_prefers_gdb_when_core_elf_exists(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            package = root / "package"
            elf = root / "app.elf"
            elf.write_bytes(b"elf")
            crash_dump.write_package(
                make_dump((0x20001000, b"data")),
                package,
                "file",
                elf_path=elf,
            )
            core = package / "coredump.elf"
            core.write_bytes(b"\x7fELFcore")
            completed = mock.Mock(stdout="bt output", stderr="", returncode=0)

            with mock.patch.object(crash_dump.shutil, "which", return_value="arm-none-eabi-gdb"):
                with mock.patch.object(crash_dump.subprocess, "run", return_value=completed) as run:
                    analysis = crash_dump.analyze_package(package, elf)

            self.assertEqual("gdb", analysis["analysis_engine"])
            self.assertIn("bt output", analysis["gdb_report"])
            command = run.call_args.args[0]
            self.assertEqual("arm-none-eabi-gdb", command[0])
            script = Path(command[command.index("-x") + 1]).read_text()
            self.assertIn(f'file "{elf.resolve()}"', script)
            self.assertIn(f'core-file "{core.resolve()}"', script)

    def test_analyze_core_elf_quotes_paths_with_spaces(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir) / "path with spaces"
            root.mkdir()
            elf = root / "app image.elf"
            core = root / "core dump.elf"
            elf.write_bytes(b"elf")
            core.write_bytes(b"core")
            completed = mock.Mock(stdout="", stderr="", returncode=0)

            with mock.patch.object(crash_dump, "_find_gdb", return_value="gdb"), mock.patch.object(
                crash_dump.subprocess, "run", return_value=completed
            ):
                crash_dump.analyze_core_elf(core, elf)

            script = (root / "analyze.gdb").read_text()
            self.assertIn(f'file "{elf.resolve()}"', script)
            self.assertIn(f'core-file "{core.resolve()}"', script)


class BackendTests(unittest.TestCase):
    def test_parse_jlink_regions_ignores_comments(self) -> None:
        regions = crash_dump.parse_jlink_regions(
            """
            // savebin ignored.bin 0x1000 0x20
            savebin hcpu_ram.bin 0x20000000 0x80000
            savebin scb.bin 0xE000ED00 0x278
            """
        )

        self.assertEqual(
            [
                crash_dump.Region("hcpu_ram.bin", 0x20000000, 0x80000),
                crash_dump.Region("scb.bin", 0xE000ED00, 0x278),
            ],
            regions,
        )

    def test_render_jlink_usb_script_replaces_uart_bridge(self) -> None:
        rendered = crash_dump.render_jlink_usb_script(
            "ip 127.0.0.1:19025\nsavebin a.bin 0x1 0x2\n"
        )

        self.assertEqual("usb\nsavebin a.bin 0x1 0x2\n", rendered)

    def test_render_jlink_usb_script_can_use_uart_server_ip(self) -> None:
        rendered = crash_dump.render_jlink_usb_script(
            "ip 127.0.0.1:19025\nsavebin a.bin 0x1 0x2\n",
            jlink_ip="127.0.0.1:19025",
        )

        self.assertEqual("ip 127.0.0.1:19025\nsavebin a.bin 0x1 0x2\n", rendered)

    def test_filter_jlink_script_can_select_hcpu(self) -> None:
        rendered = crash_dump.filter_jlink_script_core(
            "w4 0x5000B008 1 //Switch to LCPU and halt it\n"
            "savebin hcpu_ram.bin 0x20000000 0x4\n"
            "savebin hpsys_cfg.bin 0x5000B000 0x4\n"
            "savebin lcpu_ram.bin 0x20400000 0x4\n"
            "savebin lpsys_aon_reg.bin 0x40040000 0x4\n"
            "w4 0x5000B008 0 //Switch to HCPU and halt it\n",
            "hcpu",
        )

        self.assertIn("savebin hcpu_ram.bin", rendered)
        self.assertIn("savebin hpsys_cfg.bin", rendered)
        self.assertNotIn("lcpu_ram", rendered)
        self.assertNotIn("lpsys_aon", rendered)
        self.assertNotIn("Switch to", rendered)

    def test_sftool_read_command(self) -> None:
        command = crash_dump.build_sftool_read_command(
            "sftool",
            "SF32LB57",
            "nor",
            "/dev/ttyUSB0",
            1_000_000,
            0x12000000,
            0x1000,
            "coredump.bin",
        )

        self.assertEqual(
            [
                "sftool",
                "-p",
                "/dev/ttyUSB0",
                "-b",
                "1000000",
                "-c",
                "SF32LB57",
                "-m",
                "nor",
                "--after",
                "no_reset",
                "read_flash",
                "coredump.bin@0x12000000:0x1000",
            ],
            command,
        )

    def test_find_legacy_ptab_region(self) -> None:
        ptab = [
            {
                "mem": "flash2",
                "base": "0x12000000",
                "regions": [
                    {
                        "offset": "0x1000",
                        "max_size": "0x2000",
                        "tags": ["COREDUMP_REGION"],
                    }
                ],
            }
        ]

        self.assertEqual(
            (0x12001000, 0x2000),
            crash_dump.find_legacy_ptab_region(ptab, "COREDUMP_REGION"),
        )

    def test_find_legacy_ptab_region_reports_missing_offset(self) -> None:
        ptab = [{"regions": [{"max_size": "0x2000", "tags": ["COREDUMP_REGION"]}]}]

        with self.assertRaisesRegex(ValueError, "offset"):
            crash_dump.find_legacy_ptab_region(ptab, "COREDUMP_REGION")

    def _make_uart_sdk_ctx(self, temp_root: Path, sdk_path: Path = ROOT) -> Any:
        """Return a mock sdk_ctx with SIFLI_SDK_PATH pointing at *temp_root*."""
        return mock.Mock(env={"SIFLI_SDK_PATH": str(sdk_path)}, runner=mock.Mock())

    def _make_mock_dap(self, region_data: bytes = b"data") -> mock.MagicMock:
        """Return a mock SifliUartDap instance."""

        def _read(addr: int, size: int) -> bytes:
            return (region_data * ((size // len(region_data)) + 1))[:size]

        dap = mock.MagicMock()
        dap.write32 = mock.MagicMock()
        dap.read = mock.MagicMock(side_effect=_read)
        dap.read_core_registers = mock.MagicMock(return_value={})
        dap.__enter__ = mock.MagicMock(return_value=dap)
        dap.__exit__ = mock.MagicMock(return_value=None)
        return dap

    @mock.patch("sdk_py_actions.sifli_uart_dap.normalize_chip_for_dap", return_value="SF32LB52")
    @mock.patch("sdk_py_actions.sifli_uart_dap.extract_probe_port", return_value="/dev/fake")
    @mock.patch("sdk_py_actions.sifli_uart_dap.SifliUartDap")
    def test_capture_live_uart_runs_each_region_and_writes_manifest(
        self, _dap_cls, _extract, _norm
    ) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            script = root / "capture.jlink"
            script.write_text(
                "savebin hcpu_ram.bin 0x20000000 0x4\nsavebin scb.bin 0xE000ED00 0x4\n"
            )
            elf = root / "app.elf"
            log = root / "serial.log"
            elf.write_bytes(b"elf")
            log.write_text("assert")
            sdk_ctx = self._make_uart_sdk_ctx(root)
            _dap_cls.return_value = self._make_mock_dap()

            manifest = crash_dump.capture_live(
                sdk_ctx,
                "uart",
                "SF32LB52",
                root / "out",
                script_path=script,
                elf_path=elf,
                log_path=log,
                probe="1a86:55d3-1:/dev/fake",
            )

            self.assertTrue(manifest["complete"])
            self.assertIn("captured_at", manifest)
            self.assertEqual("attachments/app.elf", manifest["elf"]["file"])
            self.assertEqual("attachments/serial.log", manifest["log"]["file"])

    @mock.patch("sdk_py_actions.sifli_uart_dap.normalize_chip_for_dap", return_value="SF32LB52")
    @mock.patch("sdk_py_actions.sifli_uart_dap.extract_probe_port", return_value="/dev/fake")
    @mock.patch("sdk_py_actions.sifli_uart_dap.SifliUartDap")
    def test_capture_live_uart_records_core_registers_from_dap(
        self, _dap_cls, _extract, _norm
    ) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            script = root / "capture.jlink"
            script.write_text("savebin hcpu_ram.bin 0x20000000 0x4\n")
            sdk_ctx = self._make_uart_sdk_ctx(root)
            dap = self._make_mock_dap()
            dap.read_core_registers.return_value = {
                "r0": 1,
                "sp": 0x20001000,
                "lr": 0x12000101,
                "pc": 0x12000201,
                "xpsr": 0x21000000,
            }
            _dap_cls.return_value = dap

            manifest = crash_dump.capture_live(
                sdk_ctx,
                "uart",
                "SF32LB52",
                root / "out",
                script_path=script,
                legacy_layout=False,
                probe="1a86:55d3-1:/dev/fake",
            )

            self.assertEqual("uart_dap", manifest["register_source"])
            self.assertEqual("0x12000201", manifest["registers"]["pc"])
            self.assertEqual("uart_dap", manifest["current_register_source"])
            self.assertEqual("0x12000201", manifest["current_registers"]["pc"])

    @mock.patch("sdk_py_actions.sifli_uart_dap.normalize_chip_for_dap", return_value="SF32LB52")
    @mock.patch("sdk_py_actions.sifli_uart_dap.extract_probe_port", return_value="/dev/fake")
    @mock.patch("sdk_py_actions.sifli_uart_dap.SifliUartDap")
    def test_capture_live_uart_splits_large_regions(
        self, _dap_cls, _extract, _norm
    ) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            script = root / "capture.jlink"
            script.write_text("savebin hcpu_ram.bin 0x20000000 0x10000\n")
            sdk_ctx = self._make_uart_sdk_ctx(root)
            _dap_cls.return_value = self._make_mock_dap(b"x")

            progress = io.StringIO()
            with redirect_stderr(progress):
                manifest = crash_dump.capture_live(
                    sdk_ctx,
                    "uart",
                    "SF32LB52",
                    root / "out",
                    script_path=script,
                    probe="1a86:55d3-1:/dev/fake",
                )

            self.assertTrue(manifest["complete"])
            self.assertIn("hcpu_ram.bin", progress.getvalue())
            self.assertIn("100%", progress.getvalue())
            self.assertEqual(0x10000, (root / "out" / "hcpu_ram.bin").stat().st_size)

    @mock.patch("sdk_py_actions.sifli_uart_dap.normalize_chip_for_dap", return_value="SF32LB52")
    @mock.patch("sdk_py_actions.sifli_uart_dap.extract_probe_port", return_value="/dev/fake")
    @mock.patch("sdk_py_actions.sifli_uart_dap.SifliUartDap")
    def test_capture_live_uart_default_warns_when_skipping_low_power_regions(
        self, _dap_cls, _extract, _norm
    ) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            script = root / "capture.jlink"
            script.write_text(
                "savebin hcpu_ram.bin 0x20000000 0x4\n"
                "savebin lcpu_ram.bin 0x20400000 0x4\n"
                "savebin lpsys_aon_reg.bin 0x40040000 0x4\n"
            )
            sdk_ctx = self._make_uart_sdk_ctx(root)
            _dap_cls.return_value = self._make_mock_dap()

            warnings = io.StringIO()
            with mock.patch.object(crash_dump, "default_jlink_script", return_value=script):
                with redirect_stderr(warnings):
                    manifest = crash_dump.capture_live(
                        sdk_ctx,
                        "uart",
                        "SF32LB52",
                        root / "out",
                        legacy_layout=False,
                        probe="1a86:55d3-1:/dev/fake",
                    )

            self.assertTrue(manifest["complete"])
            self.assertEqual(["hcpu_ram.bin"], [item["file"] for item in manifest["segments"]])
            self.assertIn("skipping LCPU/LPSYS regions", warnings.getvalue())

    def test_capture_live_uart_rejects_lcpu_core(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            script = root / "capture.jlink"
            script.write_text("savebin lcpu_ram.bin 0x20400000 0x4\n")

            with mock.patch.object(crash_dump, "default_jlink_script", return_value=script):
                with self.assertRaisesRegex(Exception, "does not support --core lcpu"):
                    crash_dump.capture_live(
                        self._make_uart_sdk_ctx(root),
                        "uart",
                        "SF32LB52",
                        root / "out",
                        legacy_layout=False,
                        core="lcpu",
                        probe="1a86:55d3-1:/dev/fake",
                    )

    def test_capture_live_jlink_can_write_legacy_layout(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            script = root / "capture.jlink"
            script.write_text("savebin hcpu_ram.bin 0x20000000 0x4\n")
            elf = root / "app.elf"
            elf.write_bytes(b"elf")
            runner = mock.Mock()

            def run(command, **kwargs):
                (Path(kwargs["cwd"]) / "hcpu_ram.bin").write_bytes(b"data")
                return mock.Mock(stdout="registers", stderr="")

            runner.run.side_effect = run
            sdk_ctx = mock.Mock(env={"SIFLI_SDK_PATH": str(ROOT)}, runner=runner)

            manifest = crash_dump.capture_live(
                sdk_ctx,
                "jlink",
                "SF32LB52",
                root / "out",
                script_path=script,
                elf_path=elf,
                legacy_layout=True,
            )

            self.assertEqual(b"elf", (root / "out" / "hcpu.axf").read_bytes())
            self.assertEqual("registers", (root / "out" / "log.txt").read_text())
            self.assertEqual("attachments/capture.log", manifest["debug_log"]["file"])
            self.assertEqual(
                ["hcpu_ram.bin", "hcpu.axf", "log.txt"],
                manifest["legacy"]["files"],
            )
            self.assertEqual(
                "1.0.0",
                json.loads((root / "out" / "manifest.json").read_text())["schemaVersion"],
            )
            self.assertEqual(
                1,
                json.loads((root / "out" / "sdk_manifest.json").read_text())[
                    "schema_version"
                ],
            )

    def test_capture_live_jlink_records_core_registers_from_log(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            script = root / "capture.jlink"
            script.write_text("savebin hcpu_ram.bin 0x20000000 0x4\n")
            elf = root / "app.elf"
            elf.write_bytes(b"elf")
            runner = mock.Mock()

            def run(command, **kwargs):
                (Path(kwargs["cwd"]) / "hcpu_ram.bin").write_bytes(b"data")
                return mock.Mock(
                    stdout=(
                        "R0 = 00000001, R13 (SP) = 20001000, "
                        "R14 (LR) = 12000101, R15 (PC) = 12000201, "
                        "XPSR = 21000000"
                    ),
                    stderr="",
                )

            runner.run.side_effect = run
            sdk_ctx = mock.Mock(env={"SIFLI_SDK_PATH": str(ROOT)}, runner=runner)

            manifest = crash_dump.capture_live(
                sdk_ctx,
                "jlink",
                "SF32LB52",
                root / "out",
                script_path=script,
                elf_path=elf,
                legacy_layout=False,
            )

            self.assertEqual("debug_log", manifest["register_source"])
            self.assertEqual("0x12000201", manifest["registers"]["pc"])
            self.assertEqual("debug_log", manifest["current_register_source"])
            self.assertEqual("0x12000201", manifest["current_registers"]["pc"])
            saved = json.loads((root / "out" / "manifest.json").read_text())
            self.assertEqual(manifest["registers"], saved["registers"])

    def test_capture_live_writes_legacy_layout_by_default(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            script = root / "capture.jlink"
            script.write_text("savebin hcpu_ram.bin 0x20000000 0x4\n")
            elf = root / "app.elf"
            elf.write_bytes(b"elf")
            runner = mock.Mock()

            def run(command, **kwargs):
                (Path(kwargs["cwd"]) / "hcpu_ram.bin").write_bytes(b"data")
                return mock.Mock(stdout="registers", stderr="")

            runner.run.side_effect = run
            sdk_ctx = mock.Mock(env={"SIFLI_SDK_PATH": str(ROOT)}, runner=runner)

            crash_dump.capture_live(
                sdk_ctx,
                "jlink",
                "SF32LB52",
                root / "out",
                script_path=script,
                elf_path=elf,
            )

            self.assertEqual(b"data", (root / "out" / "hcpu_ram.bin").read_bytes())
            self.assertEqual(b"elf", (root / "out" / "hcpu.axf").read_bytes())
            self.assertEqual(
                "1.0.0",
                json.loads((root / "out" / "manifest.json").read_text())["schemaVersion"],
            )

    def test_capture_live_jlink_ip_is_passed_to_commander(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            script = root / "capture.jlink"
            script.write_text("ip 127.0.0.1:19025\nsavebin hcpu_ram.bin 0x20000000 0x4\n")
            runner = mock.Mock()

            def run(command, **kwargs):
                (Path(kwargs["cwd"]) / "hcpu_ram.bin").write_bytes(b"data")
                return mock.Mock(stdout="", stderr="")

            runner.run.side_effect = run
            sdk_ctx = mock.Mock(env={"SIFLI_SDK_PATH": str(ROOT)}, runner=runner)

            crash_dump.capture_live(
                sdk_ctx,
                "jlink",
                "SF32LB52",
                root / "out",
                script_path=script,
                jlink_ip="127.0.0.1:19025",
            )

            command = runner.run.call_args.args[0]
            self.assertIn("-ip", command)
            self.assertEqual("127.0.0.1:19025", command[command.index("-ip") + 1])

    def test_capture_live_can_select_hcpu_regions(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            script = root / "capture.jlink"
            script.write_text(
                "w4 0x5000B008 1 //Switch to LCPU and halt it\n"
                "savebin hcpu_ram.bin 0x20000000 0x4\n"
                "savebin lcpu_ram.bin 0x20400000 0x4\n"
                "w4 0x5000B008 0 //Switch to HCPU and halt it\n"
            )
            runner = mock.Mock()

            def run(command, **kwargs):
                generated = Path(command[-1])
                self.assertNotIn("lcpu_ram", generated.read_text())
                self.assertNotIn("Switch to", generated.read_text())
                (Path(kwargs["cwd"]) / "hcpu_ram.bin").write_bytes(b"data")
                return mock.Mock(stdout="", stderr="")

            runner.run.side_effect = run
            sdk_ctx = mock.Mock(env={"SIFLI_SDK_PATH": str(ROOT)}, runner=runner)

            manifest = crash_dump.capture_live(
                sdk_ctx,
                "jlink",
                "SF32LB52",
                root / "out",
                script_path=script,
                core="hcpu",
            )

            self.assertEqual(["hcpu_ram.bin"], [item["file"] for item in manifest["segments"]])

    @mock.patch("sdk_py_actions.sifli_uart_dap.normalize_chip_for_dap", return_value="SF32LB52")
    @mock.patch("sdk_py_actions.sifli_uart_dap.extract_probe_port", return_value="/dev/fake")
    @mock.patch("sdk_py_actions.sifli_uart_dap.SifliUartDap")
    def test_capture_live_can_export_psram_region(
        self, _dap_cls, _extract, _norm
    ) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            script = root / "capture.jlink"
            script.write_text(
                "savebin hcpu_ram.bin 0x20000000 0x4\n"
                "savebin psram.bin 0x60000000 0x4\n"
            )
            sdk_ctx = self._make_uart_sdk_ctx(root, sdk_path=ROOT)
            _dap_cls.return_value = self._make_mock_dap()

            with mock.patch.object(
                crash_dump,
                "default_jlink_script",
                return_value=script,
            ) as default_script:
                manifest = crash_dump.capture_live(
                    sdk_ctx,
                    "uart",
                    "SF32LB52",
                    root / "out",
                    psram_size="8MB",
                    legacy_layout=True,
                    probe="1a86:55d3-1:/dev/fake",
                )

            default_script.assert_called_once_with(
                Path(str(ROOT)),
                "SF32LB52",
                legacy_layout=True,
                psram_size="8MB",
                chip_model=None,
                include_psram=False,
            )
            self.assertEqual(
                ["hcpu_ram.bin", "psram.bin"],
                [item["file"] for item in manifest["segments"]],
            )

    def test_uart_dap_reads_core_registers(self) -> None:
        dap = SifliUartDap("/dev/fake")
        values = iter(range(17))

        def memread(addr, size):
            if addr == dap._DHCSR_ADDR:
                return struct.pack("<I", dap._DHCSR_S_REGRDY)
            if addr == dap._DCRDR_ADDR:
                return struct.pack("<I", next(values))
            raise AssertionError(hex(addr))

        dap.write32 = mock.Mock()
        dap._memread = mock.Mock(side_effect=memread)

        registers = dap.read_core_registers()

        self.assertEqual(0, registers["r0"])
        self.assertEqual(13, registers["sp"])
        self.assertEqual(14, registers["lr"])
        self.assertEqual(15, registers["pc"])
        self.assertEqual(16, registers["xpsr"])
        self.assertEqual(17, dap.write32.call_count)

    def test_uart_dap_rejects_unsupported_chip(self) -> None:
        with self.assertRaisesRegex(ValueError, "UART DAP does not support"):
            SifliUartDap("/dev/fake", chip="SF32LB58")

    @mock.patch("sdk_py_actions.sifli_uart_dap.normalize_chip_for_dap", return_value="SF32LB56")
    @mock.patch("sdk_py_actions.sifli_uart_dap.extract_probe_port", return_value="/dev/fake")
    @mock.patch("sdk_py_actions.sifli_uart_dap.SifliUartDap")
    def test_capture_live_uart_uses_sf32lb56_core_switch_address(
        self, dap_cls, _extract, _norm
    ) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            script = root / "capture.jlink"
            script.write_text("savebin hcpu_ram.bin 0x20000000 0x4\n")
            dap = self._make_mock_dap()
            dap_cls.return_value = dap

            crash_dump.capture_live(
                self._make_uart_sdk_ctx(root),
                "uart",
                "SF32LB56",
                root / "out",
                script_path=script,
                probe="1a86:55d3-1:/dev/fake",
            )

            self.assertEqual(
                [mock.call(0x5000F000, 1), mock.call(0x5000F000, 0)],
                dap.write32.call_args_list[:2],
            )

    def test_export_coredump_can_write_legacy_layout(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            source = root / "coredump.bin"
            source.write_bytes(make_dump((0x20000000, b"data")))
            script = root / "capture.jlink"
            script.write_text("savebin hcpu_ram.bin 0x20000000 0x4\n")
            elf = root / "app.elf"
            elf.write_bytes(b"elf")
            sdk_ctx = mock.Mock(env={"SIFLI_SDK_PATH": str(ROOT)}, runner=mock.Mock())

            manifest = crash_dump.export_coredump(
                sdk_ctx,
                root / "out",
                input_path=source,
                script_path=script,
                elf_path=elf,
                legacy_layout=True,
            )

            self.assertEqual(b"data", (root / "out" / "hcpu_ram.bin").read_bytes())
            self.assertEqual(b"elf", (root / "out" / "hcpu.axf").read_bytes())
            self.assertEqual([], manifest["legacy"]["missing"])
            self.assertEqual(
                "1.0.0",
                json.loads((root / "out" / "manifest.json").read_text())["schemaVersion"],
            )
            self.assertEqual(
                1,
                json.loads((root / "out" / "sdk_manifest.json").read_text())[
                    "schema_version"
                ],
            )

    def test_export_coredump_writes_legacy_layout_by_default(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            source = root / "coredump.bin"
            source.write_bytes(make_dump((0x20000000, b"data")))
            script = root / "capture.jlink"
            script.write_text("savebin hcpu_ram.bin 0x20000000 0x4\n")
            elf = root / "app.elf"
            elf.write_bytes(b"elf")
            sdk_ctx = mock.Mock(env={"SIFLI_SDK_PATH": str(ROOT)}, runner=mock.Mock())

            crash_dump.export_coredump(
                sdk_ctx,
                root / "out",
                input_path=source,
                script_path=script,
                elf_path=elf,
            )

            self.assertEqual(b"data", (root / "out" / "hcpu_ram.bin").read_bytes())
            self.assertEqual(b"elf", (root / "out" / "hcpu.axf").read_bytes())


class ExtensionTests(unittest.TestCase):
    def test_register_adds_crash_dump_commands(self) -> None:
        registry = CommandRegistry(api_version=2)

        crash_dump_ext.register(registry)

        self.assertIn("crash-dump", registry._groups)
        self.assertIn("crash-dump/capture-live", registry._commands)
        self.assertIn("crash-dump/export-coredump", registry._commands)
        self.assertIn("crash-dump/readcore", registry._commands)
        self.assertIn("crash-dump/analyze", registry._commands)

    def test_export_existing_dump_through_click_command(self) -> None:
        registry = CommandRegistry(api_version=2)
        crash_dump_ext.register(registry)
        root = registry.build_click(lambda: None, "sdk")
        sdk_ctx = mock.Mock(config=SdkConfig(), runner=mock.Mock())

        with tempfile.TemporaryDirectory() as temp_dir:
            temp = Path(temp_dir)
            source = temp / "source.bin"
            output = temp / "package"
            source.write_bytes(make_dump((0x20000000, b"data")))

            result = CliRunner().invoke(
                root,
                [
                    "crash-dump",
                    "export-coredump",
                    "--input",
                    str(source),
                    "--output",
                    str(output),
                ],
                obj={"sdk_context": sdk_ctx},
            )

            self.assertEqual(0, result.exit_code, result.output)
            self.assertTrue((output / "manifest.json").is_file())
            sdk_ctx.runner.run.assert_not_called()

    def test_export_existing_dump_through_click_writes_legacy_layout(self) -> None:
        registry = CommandRegistry(api_version=2)
        crash_dump_ext.register(registry)
        root = registry.build_click(lambda: None, "sdk")
        sdk_ctx = mock.Mock(config=SdkConfig(), runner=mock.Mock(), env={})

        with tempfile.TemporaryDirectory() as temp_dir:
            temp = Path(temp_dir)
            source = temp / "source.bin"
            output = temp / "package"
            script = temp / "capture.jlink"
            elf = temp / "app.elf"
            source.write_bytes(make_dump((0x20000000, b"data")))
            script.write_text("savebin hcpu_ram.bin 0x20000000 0x4\n")
            elf.write_bytes(b"elf")

            result = CliRunner().invoke(
                root,
                [
                    "crash-dump",
                    "export-coredump",
                    "--input",
                    str(source),
                    "--output",
                    str(output),
                    "--script",
                    str(script),
                    "--elf",
                    str(elf),
                ],
                obj={"sdk_context": sdk_ctx},
            )

            self.assertEqual(0, result.exit_code, result.output)
            self.assertEqual(b"data", (output / "hcpu_ram.bin").read_bytes())
            self.assertEqual(b"elf", (output / "hcpu.axf").read_bytes())

    def test_capture_click_accepts_psram_size(self) -> None:
        registry = CommandRegistry(api_version=2)
        crash_dump_ext.register(registry)
        root = registry.build_click(lambda: None, "sdk")
        sdk_ctx = mock.Mock(config=SdkConfig(), runner=mock.Mock(), env={})

        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "package"
            with mock.patch.object(
                crash_dump,
                "capture_live",
                return_value={"chip": "SF32LB52"},
            ) as capture_live:
                result = CliRunner().invoke(
                    root,
                    [
                        "crash-dump",
                        "capture-live",
                        "--transport",
                        "uart",
                        "--chip",
                        "SF32LB52",
                        "--output",
                        str(output),
                        "--psram-size",
                        "8MB",
                    ],
                    obj={"sdk_context": sdk_ctx},
                )

            self.assertEqual(0, result.exit_code, result.output)
            self.assertEqual("8MB", capture_live.call_args.kwargs["psram_size"])

    def test_capture_click_accepts_chip_model_and_psram_flag(self) -> None:
        registry = CommandRegistry(api_version=2)
        crash_dump_ext.register(registry)
        root = registry.build_click(lambda: None, "sdk")
        sdk_ctx = mock.Mock(config=SdkConfig(), runner=mock.Mock(), env={})

        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "package"
            with mock.patch.object(
                crash_dump,
                "capture_live",
                return_value={"chip": "SF32LB52"},
            ) as capture_live:
                result = CliRunner().invoke(
                    root,
                    [
                        "crash-dump",
                        "capture-live",
                        "--transport",
                        "uart",
                        "--chip",
                        "SF32LB52",
                        "--chip-model",
                        "LB525",
                        "--include-psram",
                        "--output",
                        str(output),
                    ],
                    obj={"sdk_context": sdk_ctx},
                )

            self.assertEqual(0, result.exit_code, result.output)
            self.assertEqual("LB525", capture_live.call_args.kwargs["chip_model"])
            self.assertTrue(capture_live.call_args.kwargs["include_psram"])

    def test_legacy_layout_option_is_removed(self) -> None:
        registry = CommandRegistry(api_version=2)
        crash_dump_ext.register(registry)
        root = registry.build_click(lambda: None, "sdk")

        with mock.patch.object(crash_dump, "capture_live", return_value={}):
            result = CliRunner().invoke(
                root,
                [
                    "crash-dump",
                    "capture-live",
                    "--chip",
                    "SF32LB52",
                    "--output",
                    "/tmp/crash-dump-test",
                    "--legacy-layout",
                ],
                obj={"sdk_context": mock.Mock(config=SdkConfig(), runner=mock.Mock())},
            )

        self.assertNotEqual(0, result.exit_code)
        self.assertIn("No such option: --legacy-layout", result.output)

    def test_capture_click_accepts_jlink_ip(self) -> None:
        registry = CommandRegistry(api_version=2)
        crash_dump_ext.register(registry)
        root = registry.build_click(lambda: None, "sdk")
        sdk_ctx = mock.Mock(config=SdkConfig(), runner=mock.Mock(), env={})

        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "package"
            with mock.patch.object(
                crash_dump,
                "capture_live",
                return_value={"chip": "SF32LB52"},
            ) as capture_live:
                result = CliRunner().invoke(
                    root,
                    [
                        "crash-dump",
                        "capture-live",
                        "--transport",
                        "jlink",
                        "--chip",
                        "SF32LB52",
                        "--jlink-ip",
                        "127.0.0.1:19025",
                        "--output",
                        str(output),
                    ],
                    obj={"sdk_context": sdk_ctx},
                )

            self.assertEqual(0, result.exit_code, result.output)
            self.assertEqual("127.0.0.1:19025", capture_live.call_args.kwargs["jlink_ip"])

    def test_capture_click_accepts_core(self) -> None:
        registry = CommandRegistry(api_version=2)
        crash_dump_ext.register(registry)
        root = registry.build_click(lambda: None, "sdk")
        sdk_ctx = mock.Mock(config=SdkConfig(), runner=mock.Mock(), env={})

        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "package"
            with mock.patch.object(
                crash_dump,
                "capture_live",
                return_value={"chip": "SF32LB52"},
            ) as capture_live:
                result = CliRunner().invoke(
                    root,
                    [
                        "crash-dump",
                        "capture-live",
                        "--transport",
                        "jlink",
                        "--chip",
                        "SF32LB52",
                        "--core",
                        "hcpu",
                        "--output",
                        str(output),
                    ],
                    obj={"sdk_context": sdk_ctx},
                )

            self.assertEqual(0, result.exit_code, result.output)
            self.assertEqual("hcpu", capture_live.call_args.kwargs["core"])

    def test_analyze_click_uses_attached_elf_and_prints_json(self) -> None:
        registry = CommandRegistry(api_version=2)
        crash_dump_ext.register(registry)
        root = registry.build_click(lambda: None, "sdk")
        sdk_ctx = mock.Mock(config=SdkConfig(), runner=mock.Mock())

        with tempfile.TemporaryDirectory() as temp_dir:
            package = Path(temp_dir)
            result_data = {"schema_version": 1, "chip": "SF32LB52"}
            with mock.patch.object(
                crash_dump, "analyze_package", return_value=result_data
            ) as analyze:
                result = CliRunner().invoke(
                    root,
                    [
                        "crash-dump",
                        "analyze",
                        "--package",
                        str(package),
                        "--json",
                    ],
                    obj={"sdk_context": sdk_ctx},
                )

            self.assertEqual(0, result.exit_code, result.output)
            self.assertEqual(result_data, json.loads(result.output))
            analyze.assert_called_once_with(package, None, None, engine="python")

    def test_readcore_click_writes_core_elf(self) -> None:
        registry = CommandRegistry(api_version=2)
        crash_dump_ext.register(registry)
        root = registry.build_click(lambda: None, "sdk")
        sdk_ctx = mock.Mock(config=SdkConfig(), runner=mock.Mock())

        with tempfile.TemporaryDirectory() as temp_dir:
            temp = Path(temp_dir)
            package = temp / "package"
            elf = temp / "app.elf"
            output = temp / "coredump.elf"
            package.mkdir()
            elf.write_bytes(b"elf")
            with mock.patch.object(
                crash_dump,
                "write_core_elf",
                return_value=output,
            ) as readcore:
                result = CliRunner().invoke(
                    root,
                    [
                        "crash-dump",
                        "readcore",
                        "--package",
                        str(package),
                        "--elf",
                        str(elf),
                        "--output",
                        str(output),
                    ],
                    obj={"sdk_context": sdk_ctx},
                )

            self.assertEqual(0, result.exit_code, result.output)
            self.assertIn(str(output.resolve()), result.output)
            readcore.assert_called_once_with(package, elf, output)

    def test_export_click_prints_manifest_json(self) -> None:
        registry = CommandRegistry(api_version=2)
        crash_dump_ext.register(registry)
        root = registry.build_click(lambda: None, "sdk")
        sdk_ctx = mock.Mock(config=SdkConfig(), runner=mock.Mock())

        with tempfile.TemporaryDirectory() as temp_dir:
            temp = Path(temp_dir)
            source = temp / "source.bin"
            source.write_bytes(make_dump((0x20000000, b"data")))
            result = CliRunner().invoke(
                root,
                [
                    "crash-dump",
                    "export-coredump",
                    "--input",
                    str(source),
                    "--output",
                    str(temp / "package"),
                    "--json",
                ],
                obj={"sdk_context": sdk_ctx},
            )

            self.assertEqual(0, result.exit_code, result.output)
            self.assertEqual("SF32LB52", json.loads(result.output)["chip"])


if __name__ == "__main__":
    unittest.main()
