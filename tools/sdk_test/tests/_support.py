# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Shared helpers for the ``sdk_test`` self-tests.

The tests are written against the standard library ``unittest`` on purpose: the
default SDK python environment does not ship pytest, and this framework must be
verifiable out of the box. Every module is therefore dual-mode -- ``python
<file>`` runs it directly, and pytest collects it unchanged.

Importing this module is also what puts ``tools/`` on ``sys.path``, so no test
needs to care about how it was started.
"""

from __future__ import annotations

import os
import sys
import tempfile
import textwrap
import unittest

#: ``tools/`` -- the directory that holds the ``sdk_test`` package.
TOOLS_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

if TOOLS_DIR not in sys.path:
    sys.path.insert(0, TOOLS_DIR)


class SdkTestCase(unittest.TestCase):
    """Base class handing each test a throwaway tree and a file writer."""

    def setUp(self) -> None:
        self._tmpdir = tempfile.TemporaryDirectory(prefix="sdk_test_")
        self.addCleanup(self._tmpdir.cleanup)
        self.root = self._tmpdir.name

    def write(self, relative_path: str, content: str) -> str:
        """Write ``content`` (dedented) to ``relative_path`` under the temp tree."""
        path = os.path.join(self.root, relative_path.replace("/", os.sep))
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(textwrap.dedent(content).lstrip("\n"))
        return path

    def mkdir(self, relative_path: str) -> str:
        path = os.path.join(self.root, relative_path.replace("/", os.sep))
        os.makedirs(path, exist_ok=True)
        return path


if __name__ == "__main__":  # pragma: no cover - helper module
    raise SystemExit("This module only provides helpers; run the test_*.py files.")
