# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for ``sdk_test.symbols``.

The one property that matters is that a name resolves to the address the
*current* image actually uses: a wrong address does not fail loudly, it reads
memory that looks plausible. ``find_elf`` is tested on its own; the resolver is
tested against a real object file compiled by the SDK toolchain in the test,
which pins the symbol-table and DWARF paths together.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import unittest

from _support import SdkTestCase

from sdk_test.errors import SymbolError
from sdk_test.symbols import _constant_member_offset
from sdk_test.symbols import address_of
from sdk_test.symbols import find_elf
from sdk_test.symbols import member_offset
from sdk_test.symbols import resolve_symbol

CC = shutil.which("arm-none-eabi-gcc")

try:
    import elftools  # noqa: F401

    HAVE_PYELFTOOLS = True
except ImportError:  # pragma: no cover - the SDK profile ships it
    HAVE_PYELFTOOLS = False


class TestFindElf(SdkTestCase):
    def test_main_elf_wins_over_another_one(self):
        self.write("build/main.elf", "x")
        self.write("build/other.elf", "y")
        self.assertEqual(
            find_elf(os.path.join(self.root, "build")),
            os.path.join(self.root, "build", "main.elf"),
        )

    def test_a_single_differently_named_elf_is_accepted(self):
        self.write("build/app.elf", "x")
        self.assertTrue(find_elf(self.mkdir("build")).endswith("app.elf"))

    def test_an_empty_build_directory_is_an_error(self):
        with self.assertRaises(SymbolError) as caught:
            find_elf(self.mkdir("build"))
        self.assertIn("no ELF", str(caught.exception))

    def test_several_elves_without_main_is_an_error(self):
        self.write("build/a.elf", "x")
        self.write("build/b.elf", "y")
        with self.assertRaises(SymbolError) as caught:
            find_elf(self.mkdir("build"))
        self.assertIn("several ELF", str(caught.exception))


@unittest.skipUnless(
    CC and HAVE_PYELFTOOLS,
    "needs the SDK toolchain (arm-none-eabi-gcc) and pyelftools",
)
class TestAddressOf(SdkTestCase):
    """Resolve against a real object file, not a mock of an ELF."""

    SOURCE = """\
    typedef struct { int a; unsigned b; } Inner;
    typedef struct { int a; Inner in; unsigned c; } Thing;
    typedef struct { unsigned flag : 1; unsigned rest : 7; } Bits;
    static volatile Thing thing;
    static volatile Bits bits;
    static volatile unsigned char buffer[64];
    unsigned first(void)
    {
        thing.c = 3;
        thing.in.b = 4;
        bits.flag = 1;
        bits.rest = 2;
        buffer[0] = (unsigned char)thing.c;
        return thing.in.b + buffer[0];
    }
    """

    def setUp(self):
        super().setUp()
        source = self.write("thing.c", self.SOURCE)
        self.elf = os.path.join(self.root, "thing.o")
        subprocess.run(
            [CC, "-g", "-Os", "-c", source, "-o", self.elf],
            check=True,
            capture_output=True,
        )

    def test_symbols_match_the_toolchains_own_reader(self):
        # A vacuous "it did not raise" would pass on a wrong address; compare
        # against nm, which is an independent reader of the same file.
        nm = shutil.which("arm-none-eabi-nm")
        if not nm:
            self.skipTest("arm-none-eabi-nm is not on PATH")
        output = subprocess.run(
            [nm, self.elf], check=True, capture_output=True, text=True
        ).stdout
        expected = {}
        for line in output.splitlines():
            parts = line.split()
            if len(parts) == 3:
                expected[parts[2]] = int(parts[0], 16)
        for name in ("buffer", "first", "thing"):
            self.assertEqual(resolve_symbol(self.elf, name), expected[name])

    def test_a_nested_struct_member_is_resolved_through_dwarf(self):
        base = resolve_symbol(self.elf, "thing")
        # Thing { int a; Inner in; unsigned c; }, Inner { int a; unsigned b; }.
        self.assertEqual(address_of(self.elf, "thing.in"), base + 4)
        self.assertEqual(address_of(self.elf, "thing.in.b"), base + 8)
        self.assertEqual(address_of(self.elf, "thing.c"), base + 12)

    def test_member_offset_reports_the_compiled_layout(self):
        self.assertEqual(member_offset(self.elf, "thing", "in"), 4)
        self.assertEqual(member_offset(self.elf, "thing", "c"), 12)

    def test_an_unknown_symbol_is_an_error(self):
        with self.assertRaises(SymbolError) as caught:
            resolve_symbol(self.elf, "not_here")
        self.assertIn("not_here", str(caught.exception))

    def test_an_unknown_member_is_an_error(self):
        with self.assertRaises(SymbolError):
            address_of(self.elf, "thing.nope")

    def test_a_bitfield_member_is_refused(self):
        # An address alone cannot read a bitfield: the field shares its storage
        # unit with its neighbours.
        with self.assertRaises(SymbolError) as caught:
            address_of(self.elf, "bits.flag")
        self.assertIn("bitfield", str(caught.exception))

    def test_dwarf_2_member_offsets_still_resolve(self):
        dwarf2 = os.path.join(self.root, "thing-dwarf2.o")
        subprocess.run(
            [
                CC,
                "-gdwarf-2",
                "-Os",
                "-c",
                os.path.join(self.root, "thing.c"),
                "-o",
                dwarf2,
            ],
            check=True,
            capture_output=True,
        )
        base = resolve_symbol(dwarf2, "thing")
        self.assertEqual(address_of(dwarf2, "thing.in.b"), base + 8)

    def test_a_dropped_static_cannot_lend_its_layout(self):
        # Two translation units declare a static of the same name with
        # different layouts. --gc-sections drops the one nothing references,
        # but its DWARF record survives; the resolver must tie the member
        # layout to the address the symbol table still has.
        self.write(
            "a.c",
            """\
            typedef struct { int a; unsigned b; } A;
            static volatile A dup;
            unsigned *use_a(void) { return (unsigned *)&dup; }
            """,
        )
        self.write(
            "b.c",
            """\
            typedef struct { int a; unsigned b; unsigned c; } B;
            static volatile B dup;
            unsigned *use_b(void) { return (unsigned *)&dup; }
            """,
        )
        self.write(
            "main.c",
            """\
            unsigned *use_a(void);
            unsigned *use_b(void);
            unsigned *start(void) { return use_a(); }
            """,
        )
        for name in ("a.c", "b.c", "main.c"):
            subprocess.run(
                [
                    CC,
                    "-ffunction-sections",
                    "-fdata-sections",
                    "-g",
                    "-Os",
                    "-c",
                    name,
                    "-o",
                    name.replace(".c", ".o"),
                ],
                cwd=self.root,
                check=True,
                capture_output=True,
            )
        app = os.path.join(self.root, "app.elf")
        subprocess.run(
            [
                CC,
                "-nostdlib",
                "-Wl,--gc-sections",
                "-Wl,-e,start",
                "-Wl,-Ttext=0x10000000",
                # b first, so its (dropped) DWARF record comes first.
                "b.o",
                "a.o",
                "main.o",
                "-o",
                app,
            ],
            cwd=self.root,
            check=True,
            capture_output=True,
        )
        address = resolve_symbol(app, "dup")
        self.assertEqual(address_of(app, "dup.a"), address)
        # `c` exists only in the dropped translation unit's record.
        with self.assertRaises(SymbolError):
            address_of(app, "dup.c")

    def test_an_elf_without_debug_information_is_reported(self):
        no_debug = os.path.join(self.root, "no_debug.o")
        subprocess.run(
            [CC, "-Os", "-c", os.path.join(self.root, "thing.c"), "-o", no_debug],
            check=True,
            capture_output=True,
        )
        with self.assertRaises(SymbolError) as caught:
            address_of(no_debug, "thing.in.b")
        self.assertIn("DWARF", str(caught.exception))

    def test_a_stripped_elf_is_reported(self):
        strip = shutil.which("arm-none-eabi-strip")
        if not strip:
            self.skipTest("arm-none-eabi-strip is not on PATH")
        stripped = os.path.join(self.root, "stripped.o")
        shutil.copyfile(self.elf, stripped)
        subprocess.run([strip, stripped], check=True, capture_output=True)
        with self.assertRaises(SymbolError) as caught:
            resolve_symbol(stripped, "buffer")
        self.assertIn("no symbol table", str(caught.exception))

    def test_two_static_symbols_with_one_name_are_ambiguous(self):
        linker = shutil.which("arm-none-eabi-ld")
        if not linker:
            self.skipTest("arm-none-eabi-ld is not on PATH")
        objects = []
        for index in (1, 2):
            source = self.write(
                f"dup{index}.c",
                f"static unsigned char dup[{index * 8}];\n"
                f"unsigned char *use{index}(void) {{ return dup; }}\n",
            )
            obj = os.path.join(self.root, f"dup{index}.o")
            subprocess.run(
                [CC, "-c", source, "-o", obj], check=True, capture_output=True
            )
            objects.append(obj)
        merged = os.path.join(self.root, "merged.o")
        subprocess.run(
            [linker, "-r", *objects, "-o", merged], check=True, capture_output=True
        )
        with self.assertRaises(SymbolError) as caught:
            resolve_symbol(merged, "dup")
        self.assertIn("more than once", str(caught.exception))


class TestMemberOffsetExpressions(SdkTestCase):
    """DWARF 2 may encode a member location as an expression, not an integer."""

    def test_an_integer_is_returned_as_is(self):
        self.assertEqual(_constant_member_offset(12), 12)

    def test_dw_op_plus_uconst(self):
        self.assertEqual(_constant_member_offset([0x23, 8]), 8)

    def test_dw_op_constu(self):
        self.assertEqual(_constant_member_offset([0x10, 0x0C]), 12)

    def test_a_multi_byte_uleb128_operand(self):
        # 300 = 0xAC 0x02 in ULEB128.
        self.assertEqual(_constant_member_offset(bytes([0x23, 0xAC, 0x02])), 300)

    def test_an_unknown_operation_is_refused(self):
        self.assertIsNone(_constant_member_offset([0x08, 0]))

    def test_a_trailing_operation_is_refused(self):
        self.assertIsNone(_constant_member_offset([0x23, 8, 0x22]))


if __name__ == "__main__":
    unittest.main()
