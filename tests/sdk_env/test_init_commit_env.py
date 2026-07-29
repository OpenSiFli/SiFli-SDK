#!/usr/bin/env python
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


class InitCommitEnvTests(unittest.TestCase):
    def test_missing_hook_stops_before_success_message(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            temp = Path(temp_dir)
            repo = temp / "repo"
            script = temp / "init_commit_env.sh"
            subprocess.run(
                ["git", "init", "-q", str(repo)],
                check=True,
                capture_output=True,
                text=True,
            )
            shutil.copy2(ROOT / "init_commit_env.sh", script)

            result = subprocess.run(
                ["bash", str(script)],
                cwd=repo,
                input="1\n",
                capture_output=True,
                text=True,
            )

            self.assertNotEqual(0, result.returncode)
            self.assertNotIn("hooks installed", result.stdout)


if __name__ == "__main__":
    unittest.main()
