#!/usr/bin/env python
#
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import os
import shutil
import sys
import tempfile
import unittest
from unittest import mock


ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import sdk  # noqa: E402


class SDKPyEnvironmentTests(unittest.TestCase):
    def make_python_env(self) -> tuple[str, str]:
        temp_dir = tempfile.mkdtemp(prefix="sdk-py-env-")
        self.addCleanup(lambda: shutil.rmtree(temp_dir, ignore_errors=True))
        subdir = "Scripts" if sys.platform == "win32" else "bin"
        exe = "python.exe" if sys.platform == "win32" else "python"
        python_dir = os.path.join(temp_dir, subdir)
        os.makedirs(python_dir, exist_ok=True)
        python_path = os.path.join(python_dir, exe)
        with open(python_path, "w", encoding="utf-8") as f:
            f.write("#!/usr/bin/env python\n")
        return temp_dir, python_path

    def sdk_path(self) -> str:
        return os.path.realpath(os.path.join(os.path.dirname(sdk.__file__), ".."))

    def test_missing_sifli_sdk_path_fails_with_migration_hint(self) -> None:
        with mock.patch.dict(os.environ, {}, clear=True):
            with self.assertRaises(sdk.SdkEnvironmentError) as cm:
                sdk.check_environment()
        self.assertIn("SIFLI_SDK_PATH is not set", str(cm.exception))
        self.assertIn("./install.sh", str(cm.exception))

    def test_mismatched_sifli_sdk_path_fails(self) -> None:
        with mock.patch.dict(os.environ, {"SIFLI_SDK_PATH": "/tmp/not-this-sdk"}, clear=True):
            with self.assertRaises(sdk.SdkEnvironmentError) as cm:
                sdk.check_environment()
        self.assertIn("different SDK checkout", str(cm.exception))

    def test_missing_python_env_var_fails(self) -> None:
        with mock.patch.dict(os.environ, {"SIFLI_SDK_PATH": self.sdk_path()}, clear=True):
            with self.assertRaises(sdk.SdkEnvironmentError) as cm:
                sdk.check_environment()
        self.assertIn("SIFLI_SDK_PYTHON_ENV_PATH is not set", str(cm.exception))

    def test_missing_python_interpreter_fails(self) -> None:
        env_dir = tempfile.mkdtemp(prefix="sdk-py-empty-")
        self.addCleanup(lambda: shutil.rmtree(env_dir, ignore_errors=True))
        env = {
            "SIFLI_SDK_PATH": self.sdk_path(),
            "SIFLI_SDK_PYTHON_ENV_PATH": env_dir,
        }
        with mock.patch.dict(os.environ, env, clear=True):
            with self.assertRaises(sdk.SdkEnvironmentError) as cm:
                sdk.check_environment()
        self.assertIn("does not exist in SIFLI_SDK_PYTHON_ENV_PATH", str(cm.exception))

    def test_interpreter_outside_env_fails(self) -> None:
        env_path, _ = self.make_python_env()
        env = {
            "SIFLI_SDK_PATH": self.sdk_path(),
            "SIFLI_SDK_PYTHON_ENV_PATH": env_path,
        }
        with mock.patch.dict(os.environ, env, clear=True):
            with self.assertRaises(sdk.SdkEnvironmentError) as cm:
                sdk.check_environment()
        self.assertIn("is not from", str(cm.exception))

    def test_interpreter_inside_env_passes(self) -> None:
        env_path, python_path = self.make_python_env()
        env = {
            "SIFLI_SDK_PATH": self.sdk_path(),
            "SIFLI_SDK_PYTHON_ENV_PATH": env_path,
        }
        with mock.patch.dict(os.environ, env, clear=True):
            with mock.patch("sys.executable", python_path):
                result = sdk.check_environment()
        self.assertEqual(result, [])

    def test_symlinked_interpreter_inside_env_passes(self) -> None:
        env_path, python_path = self.make_python_env()
        target_dir = tempfile.mkdtemp(prefix="sdk-py-target-")
        self.addCleanup(lambda: shutil.rmtree(target_dir, ignore_errors=True))
        target_python = os.path.join(target_dir, "python")
        with open(target_python, "w", encoding="utf-8") as f:
            f.write("#!/usr/bin/env python\n")
        os.remove(python_path)
        os.symlink(target_python, python_path)

        env = {
            "SIFLI_SDK_PATH": self.sdk_path(),
            "SIFLI_SDK_PYTHON_ENV_PATH": env_path,
        }
        with mock.patch.dict(os.environ, env, clear=True):
            with mock.patch("sys.executable", python_path):
                with mock.patch("sys.prefix", env_path):
                    result = sdk.check_environment()
        self.assertEqual(result, [])


if __name__ == "__main__":
    unittest.main()
