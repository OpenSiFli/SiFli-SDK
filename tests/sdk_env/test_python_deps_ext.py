#!/usr/bin/env python
#
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import json
import logging
import os
import shutil
import subprocess
import sys
import tempfile
import textwrap
import unittest
from contextlib import redirect_stdout
from io import StringIO
from unittest import mock

import tomllib


ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))

from sdk_py_actions.cli.context import SdkConfig  # noqa: E402
from sdk_py_actions.cli.context import SdkContext  # noqa: E402
from sdk_py_actions.cli.registry import CommandRegistry  # noqa: E402
from sdk_py_actions.errors import CommandExecutionError  # noqa: E402
from sdk_py_actions.errors import UsageError  # noqa: E402
from sdk_py_actions import python_deps_ext  # noqa: E402


class PythonDepsExtTests(unittest.TestCase):
    def make_sdk_root(self, profile: str = "default", dependencies: list[str] | None = None) -> tuple[str, str, str]:
        sdk_root = os.path.realpath(tempfile.mkdtemp(prefix="sdk-python-deps-"))
        self.addCleanup(lambda: shutil.rmtree(sdk_root, ignore_errors=True))

        dependencies = dependencies or ["click"]
        profile_dir = os.path.join(sdk_root, "tools", "locks", profile)
        os.makedirs(profile_dir, exist_ok=True)

        lock_json_path = os.path.join(profile_dir, "lock.json")
        pyproject_path = os.path.join(profile_dir, "pyproject.toml")
        uv_lock_path = os.path.join(profile_dir, "uv.lock")

        with open(lock_json_path, "w", encoding="utf-8") as f:
            json.dump(
                {
                    "schema_version": 1,
                    "profile": profile,
                    "python": {
                        "version": "3.13.11",
                        "project_dir": f"tools/locks/{profile}",
                        "lock_file": f"tools/locks/{profile}/uv.lock",
                    },
                    "defaults": {"targets": ["all"]},
                    "tools": {"sftool": "0.1.16"},
                    "path_order": ["sftool"],
                    "conan": {
                        "config_id": "sdk.conan-config.v2.4",
                        "remote_name": "artifactory",
                        "remote_url": "https://example.com",
                        "home_subdir": profile,
                    },
                },
                f,
                indent=2,
            )

        deps_body = ",\n".join(f'  "{item}"' for item in dependencies)
        with open(pyproject_path, "w", encoding="utf-8") as f:
            f.write(
                textwrap.dedent(
                    f"""
                    [project]
                    name = "test-profile-{profile}"
                    version = "0.1.0"
                    requires-python = "==3.13.11"
                    dependencies = [
                    {deps_body}
                    ]

                    [build-system]
                    requires = ["setuptools>=68"]
                    build-backend = "setuptools.build_meta"
                    """
                ).strip()
                + "\n"
            )

        with open(uv_lock_path, "w", encoding="utf-8") as f:
            f.write('version = 1\nrequires-python = "==3.13.11"\n')

        return sdk_root, pyproject_path, uv_lock_path

    def make_sdk_ctx(self, sdk_root: str, extra_env: dict[str, str] | None = None, runner: object | None = None) -> SdkContext:
        env = {"SIFLI_SDK_PATH": sdk_root}
        if extra_env:
            env.update(extra_env)
        return SdkContext(
            project_dir=sdk_root,
            build_dir=os.path.join(sdk_root, "build"),
            env=env,
            logger=logging.getLogger("test-python-deps"),
            runner=runner if runner is not None else mock.Mock(),
            config=SdkConfig(),
        )

    def test_register_adds_python_deps_command(self) -> None:
        registry = CommandRegistry(api_version=2)
        python_deps_ext.register(registry)

        self.assertIn("python-deps", registry._groups)
        self.assertIn("python-deps/add", registry._commands)

    def test_build_canonical_uv_lock_command_ignores_local_mirror_settings(self) -> None:
        cmd = python_deps_ext._build_canonical_uv_lock_command("/tmp/project")

        self.assertEqual(
            cmd,
            [
                "uv",
                "lock",
                "--project",
                "/tmp/project",
                "--default-index",
                "https://pypi.org/simple",
                "--index-strategy",
                "first-index",
            ],
        )

    def test_add_callback_updates_pyproject_and_uv_lock(self) -> None:
        sdk_root, pyproject_path, uv_lock_path = self.make_sdk_root()

        def fake_run(args: list[str], cwd: str | None = None, **kwargs: object) -> subprocess.CompletedProcess:
            self.assertEqual(cwd, sdk_root)
            self.assertEqual(args[:3], ["uv", "lock", "--project"])
            self.assertEqual(args[3], os.path.join(sdk_root, "tools", "locks", "default"))
            with open(uv_lock_path, "w", encoding="utf-8") as f:
                f.write('version = 1\nrequires-python = "==3.13.11"\n# refreshed\n')
            return subprocess.CompletedProcess(args, 0)

        runner = mock.Mock()
        runner.run.side_effect = fake_run
        sdk_ctx = self.make_sdk_ctx(sdk_root, runner=runner)

        output = StringIO()
        with redirect_stdout(output):
            python_deps_ext.add_callback(sdk_ctx, ("requests>=2.32",), profile="default")

        with open(pyproject_path, "rb") as f:
            pyproject_doc = tomllib.load(f)
        self.assertEqual(pyproject_doc["project"]["dependencies"], ["click", "requests>=2.32"])
        with open(uv_lock_path, "r", encoding="utf-8") as f:
            self.assertIn("# refreshed", f.read())
        self.assertIn("Added Python dependencies to profile 'default':", output.getvalue())
        self.assertIn("`./install.sh --profile default`", output.getvalue())

    def test_add_callback_rejects_existing_dependency(self) -> None:
        sdk_root, pyproject_path, uv_lock_path = self.make_sdk_root(dependencies=["click", "requests>=2.0"])
        sdk_ctx = self.make_sdk_ctx(sdk_root)

        with open(pyproject_path, "rb") as f:
            pyproject_before = f.read()
        with open(uv_lock_path, "rb") as f:
            uv_lock_before = f.read()

        with self.assertRaises(UsageError) as cm:
            python_deps_ext.add_callback(sdk_ctx, ("requests<3",), profile="default")

        self.assertIn("already exists", str(cm.exception))
        with open(pyproject_path, "rb") as f:
            self.assertEqual(f.read(), pyproject_before)
        with open(uv_lock_path, "rb") as f:
            self.assertEqual(f.read(), uv_lock_before)
        sdk_ctx.runner.run.assert_not_called()

    def test_add_callback_rolls_back_when_uv_lock_fails(self) -> None:
        sdk_root, pyproject_path, uv_lock_path = self.make_sdk_root()
        sdk_ctx = self.make_sdk_ctx(sdk_root)

        with open(pyproject_path, "rb") as f:
            pyproject_before = f.read()
        with open(uv_lock_path, "rb") as f:
            uv_lock_before = f.read()

        def failing_run(args: list[str], **kwargs: object) -> None:
            with open(uv_lock_path, "w", encoding="utf-8") as f:
                f.write("mutated lock\n")
            raise CommandExecutionError(f"Command failed: {' '.join(args)}")

        sdk_ctx.runner.run.side_effect = failing_run

        with self.assertRaises(CommandExecutionError):
            python_deps_ext.add_callback(sdk_ctx, ("requests>=2.32",), profile="default")

        with open(pyproject_path, "rb") as f:
            self.assertEqual(f.read(), pyproject_before)
        with open(uv_lock_path, "rb") as f:
            self.assertEqual(f.read(), uv_lock_before)


if __name__ == "__main__":
    unittest.main()
