# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for :mod:`sdk_test.version`.

The value is only ever read by a person explaining a result afterwards, so the
tests are about the two ways it can quietly become useless: reporting a clean
tree when the code was edited, and reporting a dirty one when nothing that runs
was touched.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import unittest

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from sdk_test import version


def git(root: str, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["git", "-C", root, *args],
        capture_output=True,
        text=True,
        check=True,
    )


class VersionTestCase(SdkTestCase):
    def a_repository(self) -> str:
        """A one-commit checkout at the root of the temp tree."""
        if shutil.which("git") is None:
            self.skipTest("git is not installed")
        git(self.root, "init", "-q")
        self.write("README.md", "hello\n")
        git(self.root, "add", "README.md")
        git(
            self.root,
            "-c",
            "user.email=test@example.com",
            "-c",
            "user.name=test",
            "commit",
            "-q",
            "-m",
            "first",
        )
        # Each root is a fresh temp tree, so the memo is always cold here.
        version._CACHE.clear()
        return self.root


class TestRevision(VersionTestCase):
    def test_a_clean_checkout_reports_its_short_sha(self):
        root = self.a_repository()
        expected = git(root, "rev-parse", "--short", "HEAD").stdout.strip()
        self.assertEqual(version.revision(root), expected)

    def test_an_edited_tracked_file_is_reported(self):
        # The case the value exists for: an agent left running from a modified
        # tree. Without the suffix the revision claims a commit it is not.
        root = self.a_repository()
        self.write("README.md", "hello, but changed\n")
        self.assertTrue(version.revision(root).endswith("-dirty"))

    def test_an_untracked_file_is_not_dirt(self):
        # A checkout here collects build trees and downloads. Counting those
        # would mark every run dirty, and a flag that is always on records
        # nothing -- so only tracked edits count.
        root = self.a_repository()
        self.write("reports/leftover.json", "{}\n")
        self.assertFalse(version.revision(root).endswith("-dirty"))

    def test_a_directory_that_is_not_a_checkout_is_unknown(self):
        version._CACHE.clear()
        self.assertEqual(version.revision(self.root), version.UNKNOWN)

    def test_a_missing_directory_is_unknown(self):
        version._CACHE.clear()
        self.assertEqual(
            version.revision(os.path.join(self.root, "absent")), version.UNKNOWN
        )

    def test_the_answer_is_memoised_per_root(self):
        # One run asks once per instance; a run with twenty of them must not
        # spawn forty git processes. Checked by making the answer change and
        # seeing the old one come back.
        root = self.a_repository()
        clean = version.revision(root)
        self.write("README.md", "changed\n")
        self.assertEqual(version.revision(root), clean)


class TestProtocol(SdkTestCase):
    def test_the_protocol_is_a_positive_integer(self):
        # It is compared for equality across a wire, so a string would work but
        # a bool would not: `True == 1` is how a version check silently passes
        # for the wrong reason.
        self.assertIsInstance(version.PROTOCOL, int)
        self.assertNotIsInstance(version.PROTOCOL, bool)
        self.assertGreater(version.PROTOCOL, 0)


if __name__ == "__main__":
    unittest.main()
