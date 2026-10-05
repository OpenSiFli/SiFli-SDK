# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for ``sdk_test.discovery``.

The test id rule (``<directory relative to the testsuite root>/<tests key>``)
and the ``common:`` merge are the two behaviours every other part of the
framework silently depends on, so they are pinned here first.
"""

from __future__ import annotations

import os
import unittest

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from sdk_test import discovery
from sdk_test.errors import DiscoveryError


class TestAllowedKeys(SdkTestCase):
    def test_schema_is_the_single_source_of_truth(self):
        keys = discovery.allowed_test_keys()
        # A representative sample: if the schema loses these, callers break.
        for expected in (
            "id",
            "tags",
            "harness",
            "harness_config",
            "timeout",
            "platform_allow",
            "platform_exclude",
            "depends_on",
            "extra_args",
            "build_only",
            "skip",
        ):
            self.assertIn(expected, keys)

    def test_the_keys_the_harnesses_read_validate(self):
        # Read by harness/unity.py. A schema that rejects one makes the guide's
        # own example invalid before anything is built -- which is what
        # `prompt` did until it was added here.
        discovery._validate(
            {
                "tests": [
                    {
                        "id": "a",
                        "harness": "unity",
                        "harness_config": {
                            "suite": "lz4",
                            "prompt": "msh />",
                            "autorun": True,
                        },
                    }
                ]
            },
            "testcase.yaml",
        )


class TestListForm(SdkTestCase):
    def test_ids_are_prefixed_with_the_relative_directory(self):
        self.write(
            "get-started/hello_world/testcase.yaml",
            """
            tests:
              - id: hello_world.console
                harness: console
                harness_config:
                  record: ["Hello world!"]
            """,
        )
        instances = discovery.discover(self.root)
        self.assertEqual(len(instances), 1)
        self.assertEqual(instances[0].id, "get-started/hello_world/hello_world.console")
        self.assertEqual(instances[0].tests_key, "hello_world.console")
        self.assertEqual(instances[0].harness, "console")

    def test_a_file_at_the_testsuite_root_has_no_prefix(self):
        self.write("testcase.yaml", "tests:\n  - id: bare\n")
        instances = discovery.discover(self.root)
        self.assertEqual([instance.id for instance in instances], ["bare"])


class TestMappingForm(SdkTestCase):
    def test_mapping_form_is_accepted(self):
        self.write(
            "unit/testcase.yaml",
            """
            tests:
              lz4.roundtrip:
                harness: unity
              lz4.bounds:
                harness: unity
            """,
        )
        instances = discovery.discover(self.root)
        self.assertEqual(
            [instance.id for instance in instances],
            ["unit/lz4.bounds", "unit/lz4.roundtrip"],
        )

    def test_an_empty_mapping_entry_is_allowed(self):
        self.write("unit/testcase.yaml", "tests:\n  only:\n")
        instances = discovery.discover(self.root)
        self.assertEqual(len(instances), 1)
        # Nothing was declared, so the documented default applies.
        self.assertEqual(instances[0].harness, "console")


class TestCommonMerge(SdkTestCase):
    def test_common_supplies_defaults_and_per_test_wins(self):
        self.write(
            "unit/testcase.yaml",
            """
            common:
              harness: unity
              timeout: 30
              tags: [unit]
            tests:
              - id: alpha
              - id: beta
                timeout: 90
                tags: [slow]
            """,
        )
        by_id = {instance.id: instance for instance in discovery.discover(self.root)}

        alpha = by_id["unit/alpha"]
        self.assertEqual(alpha.harness, "unity")
        self.assertEqual(alpha.timeout, 30)
        self.assertEqual(alpha.tags, ["unit"])

        beta = by_id["unit/beta"]
        self.assertEqual(beta.harness, "unity")  # still inherited
        self.assertEqual(beta.timeout, 90)  # overridden
        self.assertEqual(beta.tags, ["slow"])  # replaced, not merged

    def test_common_does_not_leak_between_entries(self):
        self.write(
            "unit/testcase.yaml",
            """
            common:
              harness_config:
                record: ["one"]
            tests:
              - id: alpha
              - id: beta
                harness_config:
                  record: ["two"]
            """,
        )
        by_id = {instance.id: instance for instance in discovery.discover(self.root)}
        self.assertEqual(by_id["unit/alpha"].harness_config["record"], ["one"])
        self.assertEqual(by_id["unit/beta"].harness_config["record"], ["two"])


class TestFixtures(SdkTestCase):
    def test_fixtures_come_from_harness_config(self):
        self.write(
            "unit/testcase.yaml",
            """
            tests:
              - id: spi
                harness_config:
                  fixture:
                    - spi_nor_flash_py25q128:spi2
                    - i2c_sensor
            """,
        )
        instance = discovery.discover(self.root)[0]
        self.assertEqual(
            [fixture.spec for fixture in instance.fixtures],
            ["spi_nor_flash_py25q128:spi2", "i2c_sensor"],
        )

    def test_a_scalar_fixture_is_accepted(self):
        self.write(
            "unit/testcase.yaml",
            """
            tests:
              - id: spi
                harness_config:
                  fixture: spi_nor_flash
            """,
        )
        instance = discovery.discover(self.root)[0]
        self.assertEqual([fixture.name for fixture in instance.fixtures], ["spi_nor_flash"])

    def test_no_fixtures_declared_yields_an_empty_list(self):
        self.write("unit/testcase.yaml", "tests:\n  - id: plain\n")
        self.assertEqual(discovery.discover(self.root)[0].fixtures, [])


class TestDiscoveryErrors(SdkTestCase):
    def test_missing_root_is_reported(self):
        with self.assertRaises(DiscoveryError) as caught:
            discovery.discover(os.path.join(self.root, "does-not-exist"))
        self.assertIn("does not exist", str(caught.exception))

    def test_empty_file_is_reported(self):
        path = self.write("unit/testcase.yaml", "\n")
        with self.assertRaises(DiscoveryError) as caught:
            discovery.discover(self.root)
        self.assertIn("empty", str(caught.exception))
        self.assertIn(os.path.basename(path), str(caught.exception))

    def test_missing_tests_key_is_reported(self):
        self.write("unit/testcase.yaml", "common:\n  harness: console\n")
        with self.assertRaises(DiscoveryError) as caught:
            discovery.discover(self.root)
        self.assertIn("'tests' is a required property", str(caught.exception))

    def test_unknown_field_is_rejected(self):
        self.write(
            "unit/testcase.yaml",
            """
            tests:
              - id: alpha
                not_a_real_field: 1
            """,
        )
        with self.assertRaises(DiscoveryError) as caught:
            discovery.discover(self.root)
        self.assertIn("not_a_real_field", str(caught.exception))

    def test_duplicate_id_inside_one_file_is_rejected(self):
        self.write(
            "unit/testcase.yaml",
            """
            tests:
              - id: alpha
              - id: alpha
            """,
        )
        with self.assertRaises(DiscoveryError) as caught:
            discovery.discover(self.root)
        self.assertIn("duplicate test id", str(caught.exception).lower())

    def test_duplicate_id_across_the_tree_is_rejected(self):
        # A root-level test may spell out an id that a subdirectory reaches
        # implicitly: `a/k` from both sides.
        self.write("testcase.yaml", "tests:\n  - id: a/k\n")
        self.write("a/testcase.yaml", "tests:\n  - id: k\n")
        with self.assertRaises(DiscoveryError) as caught:
            discovery.discover(self.root)
        self.assertIn("Duplicate test id", str(caught.exception))
        self.assertIn("a/k", str(caught.exception))

    def test_malformed_yaml_is_reported(self):
        self.write("unit/testcase.yaml", "tests: [unclosed\n")
        with self.assertRaises(DiscoveryError) as caught:
            discovery.discover(self.root)
        self.assertIn("malformed YAML", str(caught.exception))

    def test_a_non_mapping_entry_in_the_list_form_is_rejected(self):
        # The schema is checked before the hand-written parser, so the wording
        # comes from jsonschema; what matters is that it is rejected loudly.
        self.write("unit/testcase.yaml", "tests:\n  - just-a-string\n")
        with self.assertRaises(DiscoveryError) as caught:
            discovery.discover(self.root)
        self.assertIn("tests", str(caught.exception))


class TestSkippedDirectories(SdkTestCase):
    def test_build_and_hidden_directories_are_not_descended(self):
        self.write("unit/testcase.yaml", "tests:\n  - id: kept\n")
        self.write("build_sf32lb52/testcase.yaml", "tests:\n  - id: from_build\n")
        self.write("build/testcase.yaml", "tests:\n  - id: from_build_dir\n")
        self.write(".cache/testcase.yaml", "tests:\n  - id: from_cache\n")

        ids = [instance.id for instance in discovery.discover(self.root)]
        self.assertEqual(ids, ["unit/kept"])


class TestTagFilter(SdkTestCase):
    def setUp(self):
        super().setUp()
        self.write(
            "unit/testcase.yaml",
            """
            tests:
              - id: fast
                tags: [unit, quick]
              - id: flashy
                tags: [unit, flash]
              - id: lonely
            """,
        )

    def test_union_of_requested_tags(self):
        ids = [
            instance.id
            for instance in discovery.discover(self.root, tags=["quick", "flash"])
        ]
        self.assertEqual(ids, ["unit/fast", "unit/flashy"])

    def test_no_tag_filter_keeps_everything(self):
        self.assertEqual(len(discovery.discover(self.root)), 3)

    def test_a_tag_nobody_carries_selects_nothing(self):
        self.assertEqual(discovery.discover(self.root, tags=["nope"]), [])


class TestDiscoveryOrder(SdkTestCase):
    def test_results_are_sorted_by_id(self):
        self.write("z/testcase.yaml", "tests:\n  - id: last\n")
        self.write("a/testcase.yaml", "tests:\n  - id: first\n")
        ids = [instance.id for instance in discovery.discover(self.root)]
        self.assertEqual(ids, sorted(ids))


class TestBuildDirName(SdkTestCase):
    def test_build_dir_name_is_none_until_a_platform_is_chosen(self):
        self.write("unit/testcase.yaml", "tests:\n  - id: alpha\n")
        instance = discovery.discover(self.root)[0]
        self.assertIsNone(instance.build_dir_name)
        instance.platform = "sf32lb52-lcd_n16r8"
        self.assertEqual(instance.build_dir_name, "build_sf32lb52-lcd_n16r8")


if __name__ == "__main__":
    unittest.main()
