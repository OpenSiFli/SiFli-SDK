# SPDX-License-Identifier: Apache-2.0
"""Link real ARM ELFs through both SF32LB57 HCPU linker templates.

Run after export.ps1/export.sh. These build-integration tests exercise the
templates and GNU tools, not execution, cache behavior, or startup on a board.
The legacy fixture supplies memory-window headers; Jinja uses a real v3 MPI2
partition fixture and gen_link_lds. All outputs stay in a temporary directory.
"""

import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "build"))

import gen_link_lds  # noqa: E402
import ptab  # noqa: E402


TEMPLATES = ROOT / "drivers" / "cmsis" / "sf32lb57x" / "Templates" / "gcc" / "hcpu"
ALIAS_OFFSET = 0x50000000
KERNEL_OBJECTS = (
    "scheduler_smp", "thread", "irq", "clock", "timer", "object", "ipc",
    "idle", "kservice", "mem", "memheap", "memheap2", "mempool", "slab", "smp_sf32lb57",
    "context_smp_gcc", "cpuport",
)
CONTROL_OBJECTS = tuple(name for name in KERNEL_OBJECTS if name != "context_smp_gcc")


def partition_fixture(region="mpi2"):
    return ptab.PtabV3("<smp-linker-fixture>", {
        "version": 3,
        "chip": "SF32LB57X",
        "memory": [
            {"mpi": "mpi1", "type": "psram", "size": 8 * 1024 * 1024},
            {"mpi": region, "type": "nor", "size": 2 * 1024 * 1024},
        ],
        "partitions": [{
            "name": "hcpu_flash_code", "type": "app", "subtype": "factory",
            "region": region, "offset": 0x20000, "size": 0x100000, "core": "HCPU",
        }],
    })


def function(name, section):
    return (
        f'.section {section},"ax",%progbits\n.global {name}\n'
        f'.type {name},%function\n.thumb_func\n{name}:\n bx lr\n'
        f'.size {name}, .-{name}\n'
    )


def data(name, section, size=4, value=None):
    flags = '"a",%progbits' if value is not None else '"aw",%nobits'
    body = f'.word 0x{value:08x}' if value is not None else f'.space {size}'
    return f'.section {section},{flags}\n.global {name}\n{name}:\n {body}\n'


class SmpLinkerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tools = {name: shutil.which(f"arm-none-eabi-{name}")
                     for name in ("gcc", "ld", "objdump", "objcopy")}
        missing = [name for name, path in cls.tools.items() if not path]
        if missing:
            raise unittest.SkipTest("Activate the SDK environment; missing ARM tools: "
                                    + ", ".join(missing))
        cls.scratch = tempfile.TemporaryDirectory(prefix="smp-linker-")
        cls.addClassCleanup(cls.scratch.cleanup)
        cls.work = Path(cls.scratch.name)
        cls.images = {}
        cls.kernel_objects = []
        for index, name in enumerate(KERNEL_OBJECTS):
            source = function(f"ktext_{name}", ".text.fixture")
            source += data(f"kro_{name}", ".rodata.fixture", value=0x57010000 + index)
            if name in CONTROL_OBJECTS:
                source += data(f"kstate_{name}", ".bss.fixture", size=3)
            cls.kernel_objects.append(cls.assemble(name, source))
        cls.app_objects = {}
        for mode in ("off", "ram", "xip", "wt"):
            shared = mode != "off"
            source = '.section .vectors,"a",%progbits\n.word 0x20080000\n.word Reset_Handler\n'
            source += function("Reset_Handler", ".text.Reset_Handler")
            source += function("shared_entry", ".smp_text" if shared else ".text.shared_entry")
            source += function("shared_extra", ".smp_text.extra" if shared else ".text.shared_extra")
            source += data("shared_const", ".smp_rodata" if shared else ".rodata.shared_const",
                           value=0x51F157A1)
            source += data("shared_const_extra", ".smp_rodata.extra" if shared else ".rodata.shared_extra",
                           value=0x51F157B2)
            source += data("explicit_control", ".smp_nocache" if mode == "wt" else ".bss.control", size=3)
            source += data("explicit_control_extra", ".smp_nocache.extra" if mode == "wt" else ".bss.control_extra", size=5)
            source += data("application_bss", ".bss.application", size=37)
            source += '.section .data.fixture,"aw",%progbits\n.global initialized_data\ninitialized_data:\n.word 0x12345678\n'
            cls.app_objects[mode] = cls.assemble(f"app_{mode}", source)

    @classmethod
    def command(cls, tool, *arguments):
        return subprocess.run([cls.tools[tool], *(str(arg) for arg in arguments)],
                              capture_output=True, text=True, timeout=30, check=False)

    @classmethod
    def assemble(cls, name, source):
        path = cls.work / f"{name}.S"
        path.write_text(".syntax unified\n.cpu cortex-m33\n.thumb\n" + source, encoding="utf-8")
        obj = path.with_suffix(".o")
        result = cls.command("gcc", "-mcpu=cortex-m33", "-mthumb", "-c", path, "-o", obj)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)
        return obj

    def build(self, template, mode, *, region="mpi2", rom_window=None, mutation=None,
              expected_error=None):
        key = (template, mode, region, rom_window, mutation)
        if key in self.images:
            return self.images[key]
        folder = Path(tempfile.mkdtemp(dir=self.work))
        flags = {} if mode == "off" else {"RT_USING_SMP": 1}
        if mode in ("xip", "wt"):
            flags["RT_SMP_SHARED_XIP"] = 1
        if mode == "wt":
            flags["RT_SMP_SRAM_WT_SYNC"] = 1
        with mock.patch.dict(os.environ, {"SIFLI_SDK": str(ROOT)}):
            defines = gen_link_lds.compute_link_defines(partition_fixture(region), "main", "HCPU", flags)
        if rom_window:
            defines["__ROM_BASE"], defines["__ROM_SIZE"] = rom_window
        script = folder / "link.lds"
        if template == "jinja":
            gen_link_lds.render_link_lds(str(TEMPLATES / "link.jinja2"), str(script), defines)
        else:
            # Replace the configuration dependency, never the template under test.
            windows = {
                "CODE_START_ADDR": defines["__ROM_BASE"], "CODE_SIZE": defines["__ROM_SIZE"],
                "HCPU_RAM_DATA_START_ADDR": defines["__RAM_BASE"], "HCPU_RAM_DATA_SIZE": defines["__RAM_SIZE"],
                "HCPU_RO_DATA_START_ADDR": defines["__ROM_EX_BASE"], "HCPU_RO_DATA_SIZE": defines["__ROM_EX_SIZE"],
                "PSRAM_DATA_START_ADDR": defines["__PSRAM_BASE"], "PSRAM_DATA_SIZE": defines["__PSRAM_SIZE"],
                "HCPU_FLASH2_IMG_START_ADDR": defines["__ROM2_BASE"], "HCPU_FLASH2_IMG_SIZE": defines["__ROM2_SIZE"],
                "HCPU_FLASH2_FONT_START_ADDR": defines["__ROM3_BASE"], "HCPU_FLASH2_FONT_SIZE": defines["__ROM3_SIZE"],
                "HPSYS_MPI_MEM_CBUS_2_SBUS_OFFSET": ALIAS_OFFSET,
            }
            (folder / "rtconfig.h").write_text("".join(f"#define {name} 1\n" for name in flags), encoding="utf-8")
            (folder / "mem_map.h").write_text("".join(f"#define {name} 0x{value:x}\n" for name, value in windows.items()), encoding="utf-8")
            result = self.command("gcc", "-E", "-P", "-x", "c", "-I", folder,
                                  TEMPLATES / "link.lds", "-o", script)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        if mutation:
            text = script.read_text(encoding="utf-8")
            changes = {
                "wrong_load_region": ("> SMP_XIP AT > ROM", "> SMP_XIP AT > RAM"),
                "missing_markers": ("*(.smp_text .smp_text.* .smp_rodata .smp_rodata.*)", ""),
                "bad_cache_boundary": ("__smp_sram_cache_start__ = .;", "__smp_sram_cache_start__ = . + 1;"),
                "missing_control": ("*scheduler_smp.o (.bss .bss.* COMMON)", ""),
            }
            for name in ("mempool", "slab"):
                changes[f"missing_{name}_text"] = (f"*{name}.o (.text* .rodata*)", "")
                changes[f"missing_{name}_control"] = (f"*{name}.o (.bss .bss.* COMMON)", "")
            original, replacement = changes[mutation]
            self.assertEqual(text.count(original), 1, "fault injection must alter exactly one rule")
            script.write_text(text.replace(original, replacement), encoding="utf-8")
        elf = folder / "fixture.elf"
        # Application first: a missing kernel NC selector must not accidentally
        # pass because input-object ordering happened to put it before the heap.
        result = self.command("ld", "-T", script, "-Map", folder / "fixture.map", "-o", elf,
                              self.app_objects[mode], *self.kernel_objects)
        output = result.stdout + result.stderr
        if expected_error:
            self.assertNotEqual(result.returncode, 0, "invalid layout unexpectedly linked")
            self.assertIn(expected_error, output)
            print(f"negative {template}/{mutation or region}: ld exit={result.returncode}: {expected_error}")
            return None
        self.assertEqual(result.returncode, 0, output)
        sections_output = self.command("objdump", "-h", elf)
        symbols_output = self.command("objdump", "-t", elf)
        self.assertEqual(sections_output.returncode, 0, sections_output.stderr)
        self.assertEqual(symbols_output.returncode, 0, symbols_output.stderr)
        sections = {}
        lines = sections_output.stdout.splitlines()
        for index, line in enumerate(lines):
            match = re.match(r"\s*\d+\s+(\S+)\s+([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+", line)
            if match:
                name, size, vma, lma = match.groups()
                sections[name] = {"size": int(size, 16), "vma": int(vma, 16), "lma": int(lma, 16),
                                  "flags": lines[index + 1]}
        symbols = {}
        for line in symbols_output.stdout.splitlines():
            fields = line.split()
            if len(fields) >= 4 and re.fullmatch(r"[0-9a-fA-F]+", fields[0]) and re.fullmatch(r"[0-9a-fA-F]+", fields[-2]):
                symbols[fields[-1]] = {"address": int(fields[0], 16), "section": fields[-3]}
        binary = folder / "fixture.bin"
        result = self.command("objcopy", "-O", "binary", elf, binary)
        self.assertEqual(result.returncode, 0, result.stderr)
        image = {"sections": sections, "symbols": symbols, "binary": binary.read_bytes(), "defines": defines}
        self.images[key] = image
        return image

    def assert_shared_section(self, image, section):
        symbols = image["symbols"]
        names = ["shared_entry", "shared_extra", "shared_const", "shared_const_extra"]
        names += [f"{prefix}_{name}" for name in KERNEL_OBJECTS for prefix in ("ktext", "kro")]
        for name in names:
            self.assertEqual(symbols[name]["section"], section, name)

    def assert_nc_prefix(self, image):
        symbols = image["symbols"]
        boundary = symbols["__smp_sram_cache_start__"]["address"]
        bss = image["sections"][".bss"]
        self.assertEqual(boundary % 32, 0, "cache boundary must be cache-line aligned")
        self.assertGreater(boundary, bss["vma"])
        for name in ["explicit_control", "explicit_control_extra"] + [f"kstate_{name}" for name in CONTROL_OBJECTS]:
            self.assertEqual(symbols[name]["section"], ".bss", name)
            size = 5 if name == "explicit_control_extra" else 3
            self.assertGreaterEqual(symbols[name]["address"], bss["vma"], name)
            self.assertLessEqual(symbols[name]["address"] + size, boundary, name)
        self.assertGreaterEqual(symbols["application_bss"]["address"], boundary)
        self.assertLessEqual(symbols["application_bss"]["address"] + 37, bss["vma"] + bss["size"])
        self.assertGreaterEqual(symbols["initialized_data"]["address"], 0x20000000)
        self.assertLess(symbols["initialized_data"]["address"], boundary)

    def test_shared_xip_is_one_mpi2_image(self):
        for template in ("legacy", "jinja"):
            for mode in ("xip", "wt"):
                with self.subTest(template=template, mode=mode):
                    image = self.build(template, mode)
                    self.assertEqual(image["defines"]["__ROM_BASE"], 0x12020000)
                    self.assert_shared_section(image, ".smp_xip")
                    shared = image["sections"][".smp_xip"]
                    self.assertEqual(shared["vma"], shared["lma"] + ALIAS_OFFSET)
                    self.assertGreaterEqual(shared["vma"], 0x62000000)
                    self.assertLess(shared["vma"] + shared["size"], 0x64000000)
                    loaded = sorted((section["lma"], section["lma"] + section["size"])
                                    for section in image["sections"].values()
                                    if section["size"] and "LOAD" in section["flags"])
                    for index, (start, end) in enumerate(loaded):
                        self.assertGreaterEqual(start, 0x12020000)
                        self.assertLessEqual(end, 0x12120000)
                        if index:
                            self.assertGreaterEqual(start, loaded[index - 1][1], "physical load ranges overlap")
                    self.assertEqual(len(image["binary"]), loaded[-1][1] - loaded[0][0])
                    self.assertLess(len(image["binary"]), 0x10000, "must not emit the SBUS alias as a second image")
                    for value in (0x51F157A1, 0x51F157B2):
                        self.assertEqual(image["binary"].count(value.to_bytes(4, "little")), 1)

    def test_wt_control_bss_precedes_aligned_cache_boundary(self):
        for template in ("legacy", "jinja"):
            with self.subTest(template=template):
                self.assert_nc_prefix(self.build(template, "wt"))

    def test_smp_off_has_no_smp_sections_or_symbols(self):
        for template in ("legacy", "jinja"):
            with self.subTest(template=template):
                image = self.build(template, "off")
                self.assertFalse(any(name.startswith(".smp") for name in image["sections"]))
                self.assertFalse(any(name.startswith("__smp_") for name in image["symbols"]))
                self.assert_shared_section(image, ".text")

    def test_sram_fallback_places_shared_inputs_in_retained_sram(self):
        for template in ("legacy", "jinja"):
            with self.subTest(template=template):
                image = self.build(template, "ram")
                self.assertNotIn(".smp_xip", image["sections"])
                self.assert_shared_section(image, ".retm_data")
                section = image["sections"][".retm_data"]
                self.assertGreaterEqual(section["vma"], 0x20000000)
                self.assertLessEqual(section["vma"] + section["size"], 0x20080000)
                self.assertGreaterEqual(section["lma"], 0x12020000)
                self.assertLess(section["lma"] + section["size"], 0x12120000)

    def test_linker_assert_rejects_non_mpi2_and_overflowing_partitions(self):
        message = "SMP shared XIP requires an MPI2 NOR application partition"
        for template in ("legacy", "jinja"):
            for region, window in (("mpi3", None), ("mpi2", (0x13FFF000, 0x2000))):
                with self.subTest(template=template, region=region, window=window):
                    self.build(template, "xip", region=region, rom_window=window, expected_error=message)

    def test_linker_assert_rejects_wrong_physical_load_region(self):
        for template in ("legacy", "jinja"):
            with self.subTest(template=template):
                self.build(template, "xip", mutation="wrong_load_region",
                           expected_error="SMP shared XIP must alias its physical Flash load address")

    def test_elf_checks_detect_missing_shared_input_capture(self):
        for template in ("legacy", "jinja"):
            with self.subTest(template=template):
                # In XIP the orphan inputs already trip the alias ASSERT. The
                # RAM path still links, so inspect that ELF to verify the oracle
                # also detects a missing selector without the linker's help.
                self.build(template, "xip", mutation="missing_markers",
                           expected_error="SMP shared XIP must alias its physical Flash load address")
                image = self.build(template, "ram", mutation="missing_markers")
                with self.assertRaises(AssertionError):
                    self.assert_shared_section(image, ".retm_data")

    def test_elf_checks_detect_broken_control_prefix_and_alignment(self):
        for template in ("legacy", "jinja"):
            for mutation in ("missing_control", "bad_cache_boundary"):
                with self.subTest(template=template, mutation=mutation):
                    image = self.build(template, "wt", mutation=mutation)
                    with self.assertRaises(AssertionError):
                        self.assert_nc_prefix(image)

    def test_elf_checks_detect_missing_mempool_and_slab_rules(self):
        for template in ("legacy", "jinja"):
            for name in ("mempool", "slab"):
                for mode, section in (("xip", ".smp_xip"), ("ram", ".retm_data")):
                    with self.subTest(template=template, name=name, mode=mode):
                        image = self.build(template, mode, mutation=f"missing_{name}_text")
                        with self.assertRaises(AssertionError) as failure:
                            self.assert_shared_section(image, section)
                        print(f"negative {template}/{mode}/{name} text: {failure.exception}")
                with self.subTest(template=template, name=name, mode="wt"):
                    image = self.build(template, "wt", mutation=f"missing_{name}_control")
                    with self.assertRaises(AssertionError) as failure:
                        self.assert_nc_prefix(image)
                    print(f"negative {template}/wt/{name} control: {failure.exception}")


if __name__ == "__main__":
    unittest.main()
