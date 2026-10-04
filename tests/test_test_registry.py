# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the generated Unity case registry.

The names this produces are user-visible -- they are what the host parses out of
the console and writes into a JUnit report -- so most of what is worth testing
here is that a given source file yields the names the projects used to write by
hand in unity_suites.c.
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "tools" / "build"))

import test_registry  # noqa: E402  (path set above)


class SuiteNameTest(unittest.TestCase):
    def test_taken_from_the_file_name(self):
        self.assertEqual("lz4", test_registry.suite_of("src/test_lz4.c", ""))

    def test_extension_and_prefix_are_not_part_of_it(self):
        self.assertEqual("circular_buf",
                         test_registry.suite_of("/a/b/test_circular_buf.c", ""))

    def test_hint_wins_over_the_file_name(self):
        text = "/* suite: spi_flash */\n"
        self.assertEqual("spi_flash",
                         test_registry.suite_of("src/test_spi_flash_status.c", text))

    def test_hint_may_sit_in_a_doc_block(self):
        text = "/*\n * suite: ipc_queue\n */\n"
        self.assertEqual("ipc_queue", test_registry.suite_of("src/whatever.c", text))


class CaseNameTest(unittest.TestCase):
    def test_suite_prefix_is_stripped(self):
        text = "void test_lz4_roundtrip(void)\n{\n}\n"
        self.assertEqual([("roundtrip", "test_lz4_roundtrip")],
                         test_registry.parse_cases(text, "lz4"))

    def test_plain_prefix_is_stripped_when_it_is_not_the_suite(self):
        text = "void test_something_else(void)\n{\n}\n"
        self.assertEqual([("something_else", "test_something_else")],
                         test_registry.parse_cases(text, "lz4"))

    def test_hint_sets_the_name(self):
        text = ("/* Malformed input has no partial success. */\n"
                "/* case: malformed */\n"
                "void test_cjson_malformed_returns_null(void)\n{\n}\n")
        self.assertEqual([("malformed", "test_cjson_malformed_returns_null")],
                         test_registry.parse_cases(text, "cjson"))

    def test_hint_only_applies_to_the_function_below_it(self):
        text = ("/* case: first */\n"
                "void test_a_one(void)\n{\n}\n"
                "\n"
                "void test_a_two(void)\n{\n}\n")
        self.assertEqual([("first", "test_a_one"), ("two", "test_a_two")],
                         test_registry.parse_cases(text, "a"))

    def test_prose_containing_the_word_case_is_not_a_hint(self):
        text = ("/* In this case: the buffer is already full, so nothing moves. */\n"
                "void test_a_full(void)\n{\n}\n")
        self.assertEqual([("full", "test_a_full")],
                         test_registry.parse_cases(text, "a"))

    def test_a_prototype_is_not_a_second_case(self):
        text = ("void test_a_one(void);\n"
                "\n"
                "void test_a_one(void)\n{\n}\n")
        self.assertEqual([("one", "test_a_one")],
                         test_registry.parse_cases(text, "a"))


class CollectTest(unittest.TestCase):
    def write(self, directory, name, text):
        path = os.path.join(directory, name)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(text)
        return path

    def test_suites_come_out_sorted_and_cases_keep_file_order(self):
        with tempfile.TemporaryDirectory() as directory:
            lz4 = self.write(directory, "test_lz4.c",
                             "void test_lz4_b(void)\n{\n}\n"
                             "void test_lz4_a(void)\n{\n}\n")
            cjson = self.write(directory, "test_cjson.c",
                               "void test_cjson_z(void)\n{\n}\n")

            suites = test_registry.collect([lz4, cjson])

        self.assertEqual(["cjson", "lz4"], list(suites))
        self.assertEqual(["z"], [short for short, _ in suites["cjson"]])
        self.assertEqual(["b", "a"], [short for short, _ in suites["lz4"]])

    def test_a_file_without_cases_contributes_nothing(self):
        with tempfile.TemporaryDirectory() as directory:
            main = self.write(directory, "unity_main.c", "int main(void)\n{\n}\n")
            self.assertEqual({}, test_registry.collect([main]))

    def test_two_cases_of_one_suite_may_not_share_a_name(self):
        with tempfile.TemporaryDirectory() as directory:
            first = self.write(directory, "test_a.c",
                               "void test_a_one(void)\n{\n}\n")
            # The hint puts this file in the same suite, and its function name
            # derives the same case name.
            second = self.write(directory, "test_other.c",
                                "/* suite: a */\n"
                                "void test_a_one(void)\n{\n}\n")
            with self.assertRaises(ValueError):
                test_registry.collect([first, second])


class GeneratedSourceTest(unittest.TestCase):
    def test_it_declares_every_case_and_ends_with_the_sentinel(self):
        suites = {"lz4": [("roundtrip", "test_lz4_roundtrip")]}
        source = test_registry.generate_c(suites)

        self.assertIn("void test_lz4_roundtrip(void);", source)
        self.assertIn('{ "lz4", "roundtrip", test_lz4_roundtrip },', source)
        self.assertTrue(source.rstrip().endswith("};"))
        self.assertIn("{ RT_NULL, RT_NULL, RT_NULL }", source)
        self.assertIn('#include "sifli_unity.h"', source)

    def test_an_empty_registry_still_compiles(self):
        source = test_registry.generate_c({})
        self.assertIn("sifli_unity_cases[] =", source)
        self.assertIn("{ RT_NULL, RT_NULL, RT_NULL }", source)
        self.assertNotIn("void test_", source)

    def test_the_manifest_lists_the_cases_of_every_suite(self):
        suites = {"lz4": [("roundtrip", "test_lz4_roundtrip")],
                  "cjson": [("malformed", "test_cjson_malformed_returns_null")]}
        manifest = test_registry.generate_manifest(
            suites, ["/a/src/test_lz4.c", "/a/src/test_cjson.c"], {"board": "b"})

        self.assertEqual(["roundtrip"], manifest["suites"]["lz4"])
        self.assertEqual(["malformed"], manifest["suites"]["cjson"])
        self.assertEqual(["test_lz4.c", "test_cjson.c"], manifest["sources"])
        self.assertEqual("b", manifest["board"])

    def test_the_manifest_sits_beside_the_source(self):
        self.assertEqual(os.path.join("a", "gen", "unity_suites_gen.json"),
                         test_registry.manifest_path(
                             os.path.join("a", "gen", "unity_suites_gen.c")))


class TreeTest(unittest.TestCase):
    """The three projects in example/test/, as they actually are."""

    CASES = {
        "example/test/unit_test": {
            "circular_buf": ["fill_and_drain", "wraparound", "full_and_empty"],
            "cjson": ["parse_and_read", "malformed", "boundaries"],
            "lz4": ["roundtrip", "incompressible", "boundaries"],
        },
        "example/test/cmock_unit": {
            "ipc_queue": ["write_reports_accepted", "full_buffer_times_out",
                          "null_buffer_triggers", "unopened_writes_nothing",
                          "partial_accept_retried"],
        },
        "example/test/hal_mock": {
            "spi_flash": ["status_returned_as_reported", "status_comes_from_mock",
                          "no_handle_skips_hal"],
        },
    }

    def sources(self, project):
        directory = REPOSITORY_ROOT / project / "src"
        return sorted(str(path) for path in directory.glob("test_*.c"))

    def test_the_names_each_project_reports(self):
        for project, expected in self.CASES.items():
            with self.subTest(project=project):
                suites = test_registry.collect(self.sources(project))
                self.assertEqual(
                    {suite: [short for short, _ in cases]
                     for suite, cases in suites.items()},
                    expected)

    def test_unit_test_has_the_ring_buffer_suite_the_qemu_carrier_drops(self):
        # The suite is gated by SrcRemove in the SConscript, not by an #if in a
        # hand-written registry; the sources alone still contain it.
        suites = test_registry.collect(self.sources("example/test/unit_test"))
        self.assertIn("circular_buf", suites)
        self.assertEqual(["fill_and_drain", "wraparound", "full_and_empty"],
                         [short for short, _ in suites["circular_buf"]])


if __name__ == "__main__":
    unittest.main()
