#!/usr/bin/env python
#
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import argparse
import contextlib
import os
import shutil
import sys
import tempfile
import textwrap
import tomllib
import unittest
from types import SimpleNamespace
from unittest import mock


ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import sdk_env  # noqa: E402


class FakeTool:
    is_executable = False

    def __init__(self, export_paths: list[list[str]] | None = None) -> None:
        self._current_options = SimpleNamespace(export_paths=export_paths or [["bin"]])

    def get_export_vars(self, _version: str) -> dict[str, str]:
        return {}


class CompatHashTests(unittest.TestCase):
    def make_profile_dir(self, lock_body: str, pyproject_body: str, uv_lock_body: str) -> str:
        temp_dir = tempfile.mkdtemp(prefix="sdk-env-test-")
        self.addCleanup(lambda: shutil.rmtree(temp_dir, ignore_errors=True))
        os.makedirs(os.path.join(temp_dir, "profile"), exist_ok=True)
        with open(os.path.join(temp_dir, "profile", "lock.json"), "w", encoding="utf-8") as f:
            f.write(lock_body)
        with open(os.path.join(temp_dir, "profile", "pyproject.toml"), "w", encoding="utf-8") as f:
            f.write(pyproject_body)
        with open(os.path.join(temp_dir, "profile", "uv.lock"), "w", encoding="utf-8") as f:
            f.write(uv_lock_body)
        return temp_dir

    def test_compat_hash_ignores_comments_whitespace_and_key_order(self) -> None:
        lock_a = textwrap.dedent(
            """
            {
              "schema_version": 1,
              "profile": "default",
              "python": {"version": "3.13.11", "project_dir": "tools/locks/default", "lock_file": "tools/locks/default/uv.lock"},
              "defaults": {"targets": ["all"]},
              "tools": {"sftool": "0.1.16"},
              "path_order": ["sftool"],
              "conan": {"config_id": "sdk.conan-config.v2.4", "remote_name": "artifactory", "remote_url": "https://example.com", "home_subdir": "default"}
            }
            """
        ).strip()
        lock_b = textwrap.dedent(
            """
            {
              "profile": "default",
              "schema_version": 1,
              "conan": {"remote_url": "https://example.com", "home_subdir": "changed", "remote_name": "artifactory", "config_id": "sdk.conan-config.v2.4"},
              "path_order": ["sftool"],
              "tools": {"sftool": "0.1.16"},
              "defaults": {"targets": ["all"]},
              "python": {"lock_file": "tools/locks/default/uv.lock", "project_dir": "tools/locks/default", "version": "3.13.11"}
            }
            """
        ).strip()
        pyproject_a = textwrap.dedent(
            """
            # comment
            [project]
            name = "demo"
            version = "0.1.0"
            dependencies = ["click", "requests"]
            """
        ).strip()
        pyproject_b = textwrap.dedent(
            """
            [project]
            version = "0.1.0"
            name = "demo"
            dependencies = [
              "click",
              "requests",
            ]
            # trailing comment
            """
        ).strip()
        uv_lock_a = textwrap.dedent(
            """
            version = 1

            [[package]]
            name = "click"
            version = "8.1.7"
            """
        ).strip()
        uv_lock_b = textwrap.dedent(
            """
            version = 1

            # comment
            [[package]]
            version = "8.1.7"
            name = "click"
            """
        ).strip()

        root_a = self.make_profile_dir(lock_a, pyproject_a, uv_lock_a)
        root_b = self.make_profile_dir(lock_b, pyproject_b, uv_lock_b)
        hash_a = sdk_env.compute_env_compat_sha256(
            os.path.join(root_a, "profile", "lock.json"),
            os.path.join(root_a, "profile", "pyproject.toml"),
            os.path.join(root_a, "profile", "uv.lock"),
        )
        hash_b = sdk_env.compute_env_compat_sha256(
            os.path.join(root_b, "profile", "lock.json"),
            os.path.join(root_b, "profile", "pyproject.toml"),
            os.path.join(root_b, "profile", "uv.lock"),
        )
        self.assertEqual(hash_a, hash_b)

    def test_compat_hash_changes_on_semantic_change(self) -> None:
        lock_body = textwrap.dedent(
            """
            {
              "schema_version": 1,
              "profile": "default",
              "python": {"version": "3.13.11", "project_dir": "tools/locks/default", "lock_file": "tools/locks/default/uv.lock"},
              "defaults": {"targets": ["all"]},
              "tools": {"sftool": "0.1.16"},
              "path_order": ["sftool"],
              "conan": {"config_id": "sdk.conan-config.v2.4", "remote_name": "artifactory", "remote_url": "https://example.com", "home_subdir": "default"}
            }
            """
        ).strip()
        pyproject_a = textwrap.dedent(
            """
            [project]
            name = "demo"
            version = "0.1.0"
            dependencies = ["click"]
            """
        ).strip()
        pyproject_b = textwrap.dedent(
            """
            [project]
            name = "demo"
            version = "0.1.0"
            dependencies = ["requests"]
            """
        ).strip()
        uv_lock_body = textwrap.dedent(
            """
            version = 1
            """
        ).strip()
        root_a = self.make_profile_dir(lock_body, pyproject_a, uv_lock_body)
        root_b = self.make_profile_dir(lock_body, pyproject_b, uv_lock_body)
        hash_a = sdk_env.compute_env_compat_sha256(
            os.path.join(root_a, "profile", "lock.json"),
            os.path.join(root_a, "profile", "pyproject.toml"),
            os.path.join(root_a, "profile", "uv.lock"),
        )
        hash_b = sdk_env.compute_env_compat_sha256(
            os.path.join(root_b, "profile", "lock.json"),
            os.path.join(root_b, "profile", "pyproject.toml"),
            os.path.join(root_b, "profile", "uv.lock"),
        )
        self.assertNotEqual(hash_a, hash_b)


class StateAndPathTests(unittest.TestCase):
    def make_args(self, **overrides: object) -> argparse.Namespace:
        defaults = {
            "cache_dir": None,
            "staging_dir": None,
            "offline": False,
            "mirror": None,
            "from_bundle": None,
            "profile": "default",
            "shell": "bash",
            "toolchain": "gcc",
        }
        defaults.update(overrides)
        return argparse.Namespace(**defaults)

    def test_write_profile_state_round_trip(self) -> None:
        temp_dir = tempfile.mkdtemp(prefix="sdk-env-state-")
        self.addCleanup(lambda: shutil.rmtree(temp_dir, ignore_errors=True))
        state_path = os.path.join(temp_dir, "sifli-sdk-env.json")
        env_key = "default|compat"
        installed = {
            "sdk": {
                "env_compat_algorithm": "v1",
                "env_compat_sha256": "compat",
                "git_head": "legacy-head",
                "version_txt": "v2.4.0",
                "dirty": True,
                "host_platform": "Darwin-arm64",
            },
            "python": {"version": "3.13.11", "env_path": "/tmp/env"},
        }
        sdk_env.write_profile_state(
            state_path,
            "/repo",
            "default",
            env_key_value=env_key,
            env_state=installed,
            auto_reconcile="always",
            last_seen_git_head="repo-head",
        )
        loaded = sdk_env.read_profile_state(state_path, "/repo", "default")
        state_doc = sdk_env.load_state(state_path)
        self.assertIsNotNone(loaded)
        self.assertEqual(loaded["selected_env_key"], env_key)
        self.assertEqual(loaded["preferences"]["auto_reconcile"], "always")
        self.assertEqual(loaded["last_seen_git_head"], "repo-head")
        self.assertEqual(state_doc["envs"][env_key]["python"], installed["python"])
        self.assertEqual(
            state_doc["envs"][env_key]["sdk"],
            {"env_compat_algorithm": "v1", "env_compat_sha256": "compat"},
        )
        self.assertIn("last_used_at", state_doc["envs"][env_key])

    def test_load_state_migrates_schema_v1_to_v2(self) -> None:
        temp_dir = tempfile.mkdtemp(prefix="sdk-env-state-")
        self.addCleanup(lambda: shutil.rmtree(temp_dir, ignore_errors=True))
        state_path = os.path.join(temp_dir, "sifli-sdk-env.json")
        with open(state_path, "w", encoding="utf-8") as f:
            f.write(
                textwrap.dedent(
                    """
                    {
                      "schema_version": 1,
                      "repos": {
                        "/repo": {
                          "profiles": {
                            "default": {
                              "installed": {
                                "sdk": {
                                  "env_compat_algorithm": "v1",
                                  "env_compat_sha256": "abc123",
                                  "git_head": "legacy-head",
                                  "version_txt": "v2.4.0",
                                  "dirty": true
                                },
                                "python": {
                                  "env_path": "/tmp/legacy-python"
                                }
                               },
                               "toolchains": {
                                 "keil": {
                                   "root": "/Keil_v5",
                                   "armclang_bin": "/Keil_v5/ARM/ARMCLANG/bin"
                                 }
                               },
                               "preferences": {
                                 "auto_reconcile": "always"
                               }
                             }
                          }
                        }
                      }
                    }
                    """
                ).strip()
            )

        loaded = sdk_env.load_state(state_path)
        self.assertEqual(loaded["schema_version"], 2)
        self.assertEqual(loaded["repos"]["/repo"]["profiles"]["default"]["selected_env_key"], "default|abc123")
        self.assertEqual(loaded["repos"]["/repo"]["profiles"]["default"]["preferences"]["auto_reconcile"], "always")
        self.assertEqual(loaded["repos"]["/repo"]["profiles"]["default"]["last_seen_git_head"], "legacy-head")
        self.assertEqual(
            loaded["repos"]["/repo"]["profiles"]["default"]["toolchains"]["keil"]["armclang_bin"],
            "/Keil_v5/ARM/ARMCLANG/bin",
        )
        self.assertEqual(loaded["envs"]["default|abc123"]["python"]["env_path"], "/tmp/legacy-python")
        self.assertEqual(
            loaded["envs"]["default|abc123"]["sdk"],
            {"env_compat_algorithm": "v1", "env_compat_sha256": "abc123"},
        )
        self.assertIn("last_used_at", loaded["envs"]["default|abc123"])

    def test_load_state_normalizes_legacy_v2_env_sdk_fields(self) -> None:
        temp_dir = tempfile.mkdtemp(prefix="sdk-env-state-")
        self.addCleanup(lambda: shutil.rmtree(temp_dir, ignore_errors=True))
        state_path = os.path.join(temp_dir, "sifli-sdk-env.json")
        with open(state_path, "w", encoding="utf-8") as f:
            f.write(
                textwrap.dedent(
                    """
                    {
                      "schema_version": 2,
                      "repos": {
                        "/repo": {
                          "profiles": {
                            "default": {
                              "selected_env_key": "default|abc123",
                              "preferences": {
                                "auto_reconcile": "ask"
                              },
                              "last_seen_git_head": "repo-head"
                            }
                          }
                        }
                      },
                      "envs": {
                        "default|abc123": {
                          "sdk": {
                            "env_compat_algorithm": "v1",
                            "env_compat_sha256": "abc123",
                            "git_head": "legacy-head",
                            "version_txt": "v2.4.0",
                            "dirty": true,
                            "host_platform": "Darwin-arm64"
                          },
                          "python": {
                            "env_path": "/tmp/python"
                          }
                        }
                      }
                    }
                    """
                ).strip()
            )

        loaded = sdk_env.load_state(state_path)
        self.assertEqual(loaded["repos"]["/repo"]["profiles"]["default"]["last_seen_git_head"], "repo-head")
        self.assertEqual(
            loaded["envs"]["default|abc123"]["sdk"],
            {"env_compat_algorithm": "v1", "env_compat_sha256": "abc123"},
        )

    def test_warn_non_blocking_drift_uses_repo_last_seen_git_head(self) -> None:
        warnings: list[str] = []
        with mock.patch("sdk_env.current_git_head", return_value="new-head"):
            with mock.patch("sdk_env.current_git_dirty", return_value=True):
                with mock.patch("sdk_env.log_warn", side_effect=warnings.append):
                    sdk_env.warn_non_blocking_drift({"last_seen_git_head": "old-head"})

        self.assertIn("git HEAD changed from old-head to new-head, but environment is still compatible.", warnings)
        self.assertIn(
            "worktree is dirty; environment export will continue because the compatibility hash is unchanged.",
            warnings,
        )

    def test_write_profile_state_preserves_keil_toolchain_when_install_updates_state(self) -> None:
        temp_dir = tempfile.mkdtemp(prefix="sdk-env-state-")
        self.addCleanup(lambda: shutil.rmtree(temp_dir, ignore_errors=True))
        state_path = os.path.join(temp_dir, "sifli-sdk-env.json")
        keil = {"root": "/Keil_v5", "armclang_bin": "/Keil_v5/ARM/ARMCLANG/bin"}

        sdk_env.write_profile_state(
            state_path,
            "/repo",
            "default",
            env_key_value="default|old",
            env_state={
                "sdk": {"env_compat_algorithm": "v1", "env_compat_sha256": "old"},
                "python": {"env_path": "/tmp/old-env"},
            },
            toolchains={"keil": keil},
        )
        sdk_env.write_profile_state(
            state_path,
            "/repo",
            "default",
            env_key_value="default|new",
            env_state={
                "sdk": {"env_compat_algorithm": "v1", "env_compat_sha256": "new"},
                "python": {"env_path": "/tmp/new-env"},
            },
        )

        loaded = sdk_env.read_profile_state(state_path, "/repo", "default")
        state_doc = sdk_env.load_state(state_path)
        self.assertIsNotNone(loaded)
        self.assertEqual(loaded["selected_env_key"], "default|new")
        self.assertEqual(state_doc["envs"]["default|new"]["python"]["env_path"], "/tmp/new-env")
        self.assertEqual(loaded["toolchains"]["keil"], keil)

    def test_validate_keil_toolchain_requires_windows(self) -> None:
        with mock.patch("sdk_env.is_windows_host", return_value=False):
            with self.assertRaisesRegex(sdk_env.SDKEnvError, "Windows"):
                sdk_env.validate_keil_toolchain("/Keil_v5")

    def test_handle_install_rejects_keil_on_non_windows_before_loading_config(self) -> None:
        args = argparse.Namespace(
            profile="default",
            targets=None,
            keil="/Keil_v5",
            cache_dir=None,
            staging_dir=None,
            mirror=None,
            offline=False,
            from_bundle=None,
            compat_args=[],
        )

        with mock.patch("sdk_env.is_windows_host", return_value=False):
            with mock.patch("sdk_env.RuntimeConfig.load") as load_config:
                with self.assertRaisesRegex(sdk_env.SDKEnvError, "Windows"):
                    sdk_env.handle_install(args)

        load_config.assert_not_called()

    def test_validate_keil_toolchain_records_normalized_paths(self) -> None:
        temp_dir = tempfile.mkdtemp(prefix="sdk-env-keil-")
        self.addCleanup(lambda: shutil.rmtree(temp_dir, ignore_errors=True))
        keil_root = os.path.join(temp_dir, "Keil_v5")
        armclang_bin = os.path.join(keil_root, "ARM", "ARMCLANG", "bin")
        os.makedirs(armclang_bin)

        with mock.patch("sdk_env.is_windows_host", return_value=True):
            keil = sdk_env.validate_keil_toolchain(os.path.join(temp_dir, "Keil_v5", "."))

        self.assertEqual(keil["root"], os.path.realpath(keil_root))
        self.assertEqual(keil["armclang_bin"], os.path.realpath(armclang_bin))

    def test_validate_keil_toolchain_requires_armclang_bin(self) -> None:
        temp_dir = tempfile.mkdtemp(prefix="sdk-env-keil-")
        self.addCleanup(lambda: shutil.rmtree(temp_dir, ignore_errors=True))
        keil_root = os.path.join(temp_dir, "Keil_v5")
        os.makedirs(keil_root)

        with mock.patch("sdk_env.is_windows_host", return_value=True):
            with self.assertRaisesRegex(sdk_env.SDKEnvError, "ARM/ARMCLANG/bin"):
                sdk_env.validate_keil_toolchain(keil_root)

    def test_merge_managed_paths_replaces_previous_paths_and_dedupes(self) -> None:
        merged = sdk_env.merge_managed_paths(
            current_path=os.pathsep.join(["/old/a", "/usr/bin", "/keep"]),
            old_managed_paths=["/old/a", "/old/b"],
            new_managed_paths=["/new/a", "/usr/bin"],
        )
        self.assertEqual(merged, os.pathsep.join(["/new/a", "/usr/bin", "/keep"]))

    def test_runtime_config_ignores_install_root_in_config_json(self) -> None:
        temp_home = tempfile.mkdtemp(prefix="sdk-env-home-")
        self.addCleanup(lambda: shutil.rmtree(temp_home, ignore_errors=True))
        install_root = os.path.join(temp_home, ".sifli")
        os.makedirs(install_root, exist_ok=True)
        with open(os.path.join(install_root, "config.json"), "w", encoding="utf-8") as f:
            f.write(
                textwrap.dedent(
                    """
                    {
                      "install_root": "/tmp/should-be-ignored",
                      "cache_root": "/tmp/cache-root",
                      "staging_root": "/tmp/staging-root",
                      "offline": true
                    }
                    """
                ).strip()
            )

        with mock.patch.dict(
            os.environ,
            {"HOME": temp_home, "SIFLI_SDK_TOOLS_PATH": install_root},
            clear=True,
        ):
            config = sdk_env.RuntimeConfig.load(self.make_args())

        self.assertEqual(config.install_root, os.path.realpath(install_root))
        self.assertEqual(config.cache_root, os.path.realpath("/tmp/cache-root"))
        self.assertEqual(config.staging_root, os.path.realpath("/tmp/staging-root"))
        self.assertTrue(config.offline)

    def test_apply_china_mirror_preset_overrides_fine_grained_settings(self) -> None:
        env = {
            "SIFLI_SDK_MIRROR_CHINA": "yes",
            "SIFLI_SDK_GITHUB_ASSETS": "https://override.example/github_assets",
            "SIFLI_SDK_PYPI_DEFAULT_INDEX": "https://override.example/pypi/simple",
            "UV_PYTHON_DOWNLOADS_JSON_URL": "https://override.example/python-downloads.json",
            "UV_PYPY_INSTALL_MIRROR": "https://override.example/pypy",
        }

        self.assertTrue(sdk_env.apply_china_mirror_preset(env))
        self.assertEqual(env["SIFLI_SDK_GITHUB_ASSETS"], "https://downloads.sifli.com/github_assets")
        self.assertEqual(env["SIFLI_SDK_PYPI_DEFAULT_INDEX"], "https://mirrors.ustc.edu.cn/pypi/simple")
        self.assertEqual(
            env["UV_PYTHON_DOWNLOADS_JSON_URL"],
            "https://uv.agentsmirror.com/metadata/python-downloads.json",
        )
        self.assertEqual(env["UV_PYPY_INSTALL_MIRROR"], "https://uv.agentsmirror.com/pypy")

    def test_runtime_config_load_applies_china_mirror_preset(self) -> None:
        temp_home = tempfile.mkdtemp(prefix="sdk-env-home-")
        self.addCleanup(lambda: shutil.rmtree(temp_home, ignore_errors=True))
        install_root = os.path.join(temp_home, ".sifli")
        os.makedirs(install_root, exist_ok=True)

        with mock.patch.dict(
            os.environ,
            {
                "HOME": temp_home,
                "SIFLI_SDK_TOOLS_PATH": install_root,
                "SIFLI_SDK_MIRROR_CHINA": "1",
            },
            clear=True,
        ):
            config = sdk_env.RuntimeConfig.load(self.make_args())
            self.assertEqual(
                os.environ["UV_PYTHON_DOWNLOADS_JSON_URL"],
                "https://uv.agentsmirror.com/metadata/python-downloads.json",
            )
            self.assertEqual(os.environ["UV_PYPY_INSTALL_MIRROR"], "https://uv.agentsmirror.com/pypy")

        self.assertEqual(config.python_default_index, "https://mirrors.ustc.edu.cn/pypi/simple")
        self.assertIn(
            {"type": "github-assets", "url": "https://downloads.sifli.com/github_assets"},
            config.sources,
        )

    def test_load_state_resets_unsupported_schema(self) -> None:
        temp_dir = tempfile.mkdtemp(prefix="sdk-env-state-")
        self.addCleanup(lambda: shutil.rmtree(temp_dir, ignore_errors=True))
        state_path = os.path.join(temp_dir, "sifli-sdk-env.json")
        with open(state_path, "w", encoding="utf-8") as f:
            f.write(
                textwrap.dedent(
                    """
                    {
                      "schema_version": 99,
                      "repos": {
                        "/repo": {
                          "profiles": {
                            "default": {
                              "installed": {
                                "python": {
                                  "env_path": "/tmp/old-env"
                                }
                              }
                            }
                          }
                        }
                      }
                    }
                    """
                ).strip()
            )

        loaded = sdk_env.load_state(state_path)
        self.assertEqual(loaded, {"schema_version": sdk_env.STATE_SCHEMA_VERSION, "repos": {}, "envs": {}})

    def test_export_reexec_argv_uses_target_env_python(self) -> None:
        temp_install_root = tempfile.mkdtemp(prefix="sdk-env-install-root-")
        self.addCleanup(lambda: shutil.rmtree(temp_install_root, ignore_errors=True))
        config = sdk_env.RuntimeConfig(
            install_root=temp_install_root,
            cache_root=os.path.join(temp_install_root, "cache"),
            staging_root=os.path.join(temp_install_root, "staging"),
            offline=False,
            python_default_index="https://pypi.org/simple",
            python_indexes=[],
            python_index_strategy="first-index",
            sources=[],
        )
        lock = sdk_env.ProfileLock(
            path="/tmp/lock.json",
            schema_version=1,
            profile="default",
            python_version="3.13.11",
            python_project_dir="tools/locks/default",
            python_lock_file="tools/locks/default/uv.lock",
            default_targets=["all"],
            tools={"sftool": "0.1.16"},
            path_order=["sftool"],
            conan_config_id="sdk.conan-config.v2.4",
            conan_remote_name="artifactory",
            conan_remote_url="https://example.com",
            conan_home_subdir="default",
        )
        args = self.make_args(
            profile="default",
            shell="bash",
            offline=True,
            mirror="https://mirror.example",
            toolchain="keil",
        )
        env_path = sdk_env.python_env_path(config, lock, "compat")
        argv = sdk_env.export_reexec_argv(args, lock, env_path)

        self.assertEqual(argv[0], sdk_env.python_executable(env_path))
        self.assertEqual(argv[1:5], [os.path.join(ROOT, "tools", "sdk_env.py"), "export", "--profile", "default"])
        self.assertIn("--toolchain", argv)
        self.assertEqual(argv[argv.index("--toolchain") + 1], "keil")
        self.assertIn("--offline", argv)
        self.assertIn("https://mirror.example", argv)


class TargetParsingTests(unittest.TestCase):
    def test_install_target_conflict_is_rejected(self) -> None:
        lock = sdk_env.ProfileLock(
            path="/tmp/lock.json",
            schema_version=1,
            profile="default",
            python_version="3.13.11",
            python_project_dir="tools/locks/default",
            python_lock_file="tools/locks/default/uv.lock",
            default_targets=["all"],
            tools={"sftool": "0.1.16"},
            path_order=["sftool"],
            conan_config_id="sdk.conan-config.v2.4",
            conan_remote_name="artifactory",
            conan_remote_url="https://example.com",
            conan_home_subdir="default",
        )
        with self.assertRaises(sdk_env.SDKEnvError):
            sdk_env.parse_install_targets(lock, "sf32lb52", ["sf32lb58"])

    def test_install_command_hint_matches_shell(self) -> None:
        self.assertEqual(sdk_env.install_command_hint("default", "bash"), "./install.sh --profile default")
        self.assertEqual(sdk_env.install_command_hint("default", "zsh"), "./install.sh --profile default")
        self.assertEqual(sdk_env.install_command_hint("default", "powershell"), ".\\install.ps1 --profile default")

    def test_export_parser_accepts_short_toolchain_option(self) -> None:
        args = sdk_env.build_parser().parse_args(["export", "--shell", "powershell", "-t", "keil"])
        self.assertEqual(args.toolchain, "keil")


class SDKVersionTests(unittest.TestCase):
    def test_sdk_release_line_parses_version_txt(self) -> None:
        self.assertEqual(sdk_env.sdk_release_line("v2.4.0"), "2.4")
        self.assertEqual(sdk_env.sdk_release_line("2.4.0"), "2.4")
        self.assertEqual(sdk_env.sdk_release_line("v2.4.0-rc1"), "2.4")

    def test_sdk_release_line_rejects_invalid_version_txt(self) -> None:
        with self.assertRaises(sdk_env.SDKEnvError):
            sdk_env.sdk_release_line("release/v2.4")

    def test_build_export_environment_exports_release_line_version(self) -> None:
        lock = sdk_env.ProfileLock(
            path="/tmp/lock.json",
            schema_version=1,
            profile="default",
            python_version="3.13.11",
            python_project_dir="tools/locks/default",
            python_lock_file="tools/locks/default/uv.lock",
            default_targets=["all"],
            tools={"sftool": "0.1.16"},
            path_order=["sftool"],
            conan_config_id="sdk.conan-config.v2.4",
            conan_remote_name="artifactory",
            conan_remote_url="https://example.com",
            conan_home_subdir="default",
        )
        config = sdk_env.RuntimeConfig(
            install_root="/tmp/.sifli",
            cache_root="/tmp/.sifli/cache",
            staging_root="/tmp/.sifli/staging",
            offline=False,
            python_default_index="https://pypi.org/simple",
            python_indexes=[],
            python_index_strategy="first-index",
            sources=[],
        )
        with mock.patch("sdk_env.repo_root", return_value="/repo"):
            with mock.patch("sdk_env.read_sdk_release_line", return_value="2.4"):
                with mock.patch("sdk_env.current_git_head", return_value="deadbeef"):
                    with mock.patch("sdk_env.installed_tool_versions", return_value={}):
                        with mock.patch("sdk_env.gcc_exec_path", return_value="/tmp/.sifli/tools/arm-none-eabi-gcc/14.2.1/bin"):
                            with mock.patch.dict(os.environ, {"PATH": "/usr/bin"}, clear=True):
                                env_map = sdk_env.build_export_environment(
                                    config=config,
                                    lock=lock,
                                    plans=[],
                                    env_path="/tmp/.sifli/python_env/default/py3.13.11",
                                    conan_home="/tmp/.sifli/envs/default/compat/conan",
                                    shell="bash",
                                )

        self.assertEqual(env_map["SIFLI_SDK_VERSION"], "2.4")
        self.assertEqual(env_map["SIFLI_SDK_PATH"], "/repo")
        self.assertEqual(env_map["SIFLI_SDK_GIT_HEAD"], "deadbeef")


class ToolchainExportTests(unittest.TestCase):
    def make_config(self) -> sdk_env.RuntimeConfig:
        temp_root = tempfile.mkdtemp(prefix="sdk-env-toolchain-")
        self.addCleanup(lambda: shutil.rmtree(temp_root, ignore_errors=True))
        return sdk_env.RuntimeConfig(
            install_root=temp_root,
            cache_root=os.path.join(temp_root, "cache"),
            staging_root=os.path.join(temp_root, "staging"),
            offline=False,
            python_default_index="https://pypi.org/simple",
            python_indexes=[],
            python_index_strategy="first-index",
            sources=[],
        )

    def make_lock(self) -> sdk_env.ProfileLock:
        return sdk_env.ProfileLock(
            path="/tmp/lock.json",
            schema_version=1,
            profile="default",
            python_version="3.13.11",
            python_project_dir="tools/locks/default",
            python_lock_file="tools/locks/default/uv.lock",
            default_targets=["all"],
            tools={sdk_env.GCC_TOOL_NAME: "14.2.1"},
            path_order=[sdk_env.GCC_TOOL_NAME],
            conan_config_id="sdk.conan-config.v2.4",
            conan_remote_name="artifactory",
            conan_remote_url="https://example.com",
            conan_home_subdir="default",
        )

    def build_env(self, config: sdk_env.RuntimeConfig, lock: sdk_env.ProfileLock, toolchain: str) -> dict[str, str]:
        gcc_plan = sdk_env.ToolPlan(
            name=sdk_env.GCC_TOOL_NAME,
            version="14.2.1",
            required=True,
            tool=FakeTool(),
        )
        env_path = os.path.join(config.install_root, "python_env", "default", "py3.13.11")
        with mock.patch("sdk_env.repo_root", return_value="/repo"):
            with mock.patch("sdk_env.read_sdk_release_line", return_value="2.4"):
                with mock.patch("sdk_env.current_git_head", return_value="deadbeef"):
                    with mock.patch("sdk_env.installed_tool_versions", return_value={sdk_env.GCC_TOOL_NAME: "14.2.1"}):
                        with mock.patch.dict(os.environ, {"PATH": "/usr/bin"}, clear=True):
                            return sdk_env.build_export_environment(
                                config=config,
                                lock=lock,
                                plans=[gcc_plan],
                                env_path=env_path,
                                conan_home=os.path.join(config.install_root, "envs", "default", "compat", "conan"),
                                shell="powershell",
                                toolchain=toolchain,
                            )

    def test_gcc_export_sets_rtt_exec_path_to_installed_gcc_bin(self) -> None:
        config = self.make_config()
        lock = self.make_lock()
        env_map = self.build_env(config, lock, "gcc")

        expected_gcc_bin = os.path.realpath(
            os.path.join(config.install_root, "tools", sdk_env.GCC_TOOL_NAME, "14.2.1", "bin")
        )
        self.assertEqual(env_map["RTT_CC"], "gcc")
        self.assertEqual(env_map["RTT_EXEC_PATH"], expected_gcc_bin)
        self.assertIn(expected_gcc_bin, env_map["SIFLI_SDK_MANAGED_PATHS"].split(os.pathsep))

    def test_keil_export_uses_recorded_keil_root_and_armclang_path(self) -> None:
        config = self.make_config()
        lock = self.make_lock()
        keil_root = os.path.join(config.install_root, "Keil_v5")
        armclang_bin = os.path.join(keil_root, "ARM", "ARMCLANG", "bin")
        os.makedirs(armclang_bin)
        sdk_env.write_profile_state(
            config.state_path,
            "/repo",
            "default",
            toolchains={"keil": {"root": os.path.realpath(keil_root), "armclang_bin": os.path.realpath(armclang_bin)}},
        )

        with mock.patch("sdk_env.is_windows_host", return_value=True):
            env_map = self.build_env(config, lock, "keil")

        self.assertEqual(env_map["RTT_CC"], "keil")
        self.assertEqual(env_map["RTT_EXEC_PATH"], os.path.realpath(keil_root))
        self.assertIn(os.path.realpath(armclang_bin), env_map["SIFLI_SDK_MANAGED_PATHS"].split(os.pathsep))
        self.assertIn("--toolchain keil", env_map["SIFLI_SDK_TOOLS_EXPORT_CMD"])

    def test_keil_export_requires_recorded_state(self) -> None:
        config = self.make_config()
        lock = self.make_lock()

        with mock.patch("sdk_env.is_windows_host", return_value=True):
            with self.assertRaisesRegex(sdk_env.SDKEnvError, "not configured"):
                self.build_env(config, lock, "keil")

    def test_keil_export_requires_windows(self) -> None:
        config = self.make_config()
        lock = self.make_lock()

        with mock.patch("sdk_env.is_windows_host", return_value=False):
            with self.assertRaisesRegex(sdk_env.SDKEnvError, "Windows"):
                self.build_env(config, lock, "keil")


class ExportErrorMessageTests(unittest.TestCase):
    def test_handle_export_never_choice_mentions_exact_install_command(self) -> None:
        lock = sdk_env.ProfileLock(
            path="/tmp/lock.json",
            schema_version=1,
            profile="default",
            python_version="3.13.11",
            python_project_dir="tools/locks/default",
            python_lock_file="tools/locks/default/uv.lock",
            default_targets=["all"],
            tools={"sftool": "0.1.16"},
            path_order=["sftool"],
            conan_config_id="sdk.conan-config.v2.4",
            conan_remote_name="artifactory",
            conan_remote_url="https://example.com",
            conan_home_subdir="default",
        )
        config = sdk_env.RuntimeConfig(
            install_root="/tmp/.sifli",
            cache_root="/tmp/.sifli/cache",
            staging_root="/tmp/.sifli/staging",
            offline=False,
            python_default_index="https://pypi.org/simple",
            python_indexes=[],
            python_index_strategy="first-index",
            sources=[],
        )
        args = argparse.Namespace(
            profile="default",
            shell="bash",
            cache_dir=None,
            staging_dir=None,
            mirror=None,
            offline=False,
            from_bundle=None,
        )

        with mock.patch("sdk_env.RuntimeConfig.load", return_value=config):
            with mock.patch("sdk_env.ProfileLock.load", return_value=lock):
                with mock.patch("sdk_env.read_version_txt", return_value="v2.4.0"):
                    with mock.patch("sdk_env.load_tool_plans", return_value=[]):
                        with mock.patch(
                            "sdk_env.validate_env_instance",
                            return_value=(
                                ["python env interpreter is missing"],
                                sdk_env.ResolvedEnvInstance(
                                    key="default|compat",
                                    compat_sha="compat",
                                    env_state=None,
                                    python_env_path="/tmp/env/python",
                                    conan_home="/tmp/env/conan",
                                ),
                            ),
                        ):
                            with mock.patch("sdk_env.auto_reconcile_preference", return_value="never"):
                                with self.assertRaises(sdk_env.SDKEnvError) as cm:
                                    sdk_env.handle_export(args)

        message = str(cm.exception)
        self.assertIn("environment instance for profile 'default' is missing or invalid", message)
        self.assertIn("`./install.sh --profile default`", message)
        self.assertIn("export again", message)


class FakeResponse:
    def __init__(self, chunks: list[bytes], headers: dict[str, str] | None = None) -> None:
        self._chunks = list(chunks)
        self.headers = headers or {}

    def read(self, _size: int = -1) -> bytes:
        if not self._chunks:
            return b""
        return self._chunks.pop(0)

    def __enter__(self) -> "FakeResponse":
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        return False


class FakeProgress:
    last_instance: "FakeProgress | None" = None

    def __init__(self, *args: object, **kwargs: object) -> None:
        self.args = args
        self.kwargs = kwargs
        self.tasks: list[dict[str, object]] = []
        self.updates: list[int] = []
        FakeProgress.last_instance = self

    def __enter__(self) -> "FakeProgress":
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        return False

    def add_task(self, description: str, total: object = None) -> int:
        self.tasks.append({"description": description, "total": total})
        return 1

    def update(self, _task_id: int, advance: int) -> None:
        self.updates.append(advance)


class DownloadTests(unittest.TestCase):
    def test_resolve_download_total_size_prefers_content_length(self) -> None:
        response = FakeResponse([], headers={"Content-Length": "12"})
        self.assertEqual(sdk_env.resolve_download_total_size(response, 99), 12)

    def test_resolve_download_total_size_falls_back_to_expected_size(self) -> None:
        response = FakeResponse([])
        self.assertEqual(sdk_env.resolve_download_total_size(response, 99), 99)

    def test_resolve_download_total_size_handles_unknown_size(self) -> None:
        response = FakeResponse([], headers={"Content-Length": "invalid"})
        self.assertIsNone(sdk_env.resolve_download_total_size(response, -1))

    def test_download_to_path_updates_progress_with_known_total(self) -> None:
        temp_dir = tempfile.mkdtemp(prefix="sdk-env-download-")
        self.addCleanup(lambda: shutil.rmtree(temp_dir, ignore_errors=True))
        destination = os.path.join(temp_dir, "artifact.bin")
        response = FakeResponse([b"abc", b"def"], headers={"Content-Length": "6"})

        with mock.patch("sdk_env.should_show_download_progress", return_value=True):
            with mock.patch("sdk_env.Progress", FakeProgress):
                with mock.patch("urllib.request.urlopen", return_value=response):
                    sdk_env.download_to_path("https://example.com/tool.zip", destination, "tool.zip", 10)

        self.assertTrue(os.path.isfile(destination))
        with open(destination, "rb") as f:
            self.assertEqual(f.read(), b"abcdef")
        assert FakeProgress.last_instance is not None
        self.assertEqual(FakeProgress.last_instance.tasks[0]["description"], "Downloading tool.zip")
        self.assertEqual(FakeProgress.last_instance.tasks[0]["total"], 6)
        self.assertEqual(sum(FakeProgress.last_instance.updates), 6)

    def test_ensure_cached_artifact_logs_fallback_between_urls(self) -> None:
        temp_root = tempfile.mkdtemp(prefix="sdk-env-cache-")
        self.addCleanup(lambda: shutil.rmtree(temp_root, ignore_errors=True))
        config = sdk_env.RuntimeConfig(
            install_root=os.path.join(temp_root, ".sifli"),
            cache_root=os.path.join(temp_root, ".sifli", "cache"),
            staging_root=os.path.join(temp_root, ".sifli", "staging"),
            offline=False,
            python_default_index="https://pypi.org/simple",
            python_indexes=[],
            python_index_strategy="first-index",
            sources=[
                {"type": "github-assets", "url": "https://mirror.example/github_assets"},
                {"type": "upstream", "url": "https://upstream.example"},
            ],
        )

        def fake_download_candidate(url: str, destination: str, offline: bool, label: str, expected_size: int) -> bool:
            if url.startswith("https://mirror.example/github_assets"):
                return False
            with open(destination, "wb") as f:
                f.write(b"payload")
            return True

        warnings: list[str] = []
        with mock.patch("sdk_env.download_candidate", side_effect=fake_download_candidate):
            with mock.patch("sdk_env.legacy_tools.get_file_size_sha256", return_value=(7, "abc123")):
                with mock.patch("sdk_env.log_warn", side_effect=warnings.append):
                    cache_path = sdk_env.ensure_cached_artifact(
                        config=config,
                        filename="tool.tar.xz",
                        expected_sha256="abc123",
                        expected_size=7,
                        original_url="https://github.com/OpenSiFli/tooling/releases/download/v1.0/tool.tar.xz",
                        bundle_dir=None,
                    )

        self.assertTrue(os.path.isfile(cache_path))
        self.assertIn(
            "Failed to download artifact tool.tar.xz from https://mirror.example/github_assets/OpenSiFli/tooling/releases/download/v1.0/tool.tar.xz, falling back to the next source.",
            warnings,
        )

    def test_ensure_python_env_passes_china_mirror_env_to_uv_commands(self) -> None:
        temp_install_root = tempfile.mkdtemp(prefix="sdk-env-install-root-")
        self.addCleanup(lambda: shutil.rmtree(temp_install_root, ignore_errors=True))
        config = sdk_env.RuntimeConfig(
            install_root=temp_install_root,
            cache_root=os.path.join(temp_install_root, "cache"),
            staging_root=os.path.join(temp_install_root, "staging"),
            offline=False,
            python_default_index="https://pypi.org/simple",
            python_indexes=[],
            python_index_strategy="first-index",
            sources=[],
        )
        lock = sdk_env.ProfileLock(
            path="/tmp/lock.json",
            schema_version=1,
            profile="default",
            python_version="3.13.11",
            python_project_dir="tools/locks/default",
            python_lock_file="tools/locks/default/uv.lock",
            default_targets=["all"],
            tools={"sftool": "0.1.16"},
            path_order=["sftool"],
            conan_config_id="sdk.conan-config.v2.4",
            conan_remote_name="artifactory",
            conan_remote_url="https://example.com",
            conan_home_subdir="default",
        )
        env_path = sdk_env.python_env_path(config, lock, "compat")
        python_path = sdk_env.python_executable(env_path)
        os.makedirs(os.path.dirname(python_path), exist_ok=True)
        with open(python_path, "w", encoding="utf-8") as f:
            f.write("")

        with mock.patch.dict(os.environ, {"SIFLI_SDK_MIRROR_CHINA": "true"}, clear=True):
            with mock.patch("sdk_env.run_command") as run_command:
                with mock.patch(
                    "sdk_env.temporary_uv_project",
                    return_value=contextlib.nullcontext("/tmp/project"),
                ):
                    result = sdk_env.ensure_python_env(config, lock, env_path)

        self.assertEqual(result, env_path)
        self.assertEqual(run_command.call_count, 2)

        python_install_call = run_command.call_args_list[0]
        self.assertEqual(python_install_call.args[0], ["uv", "python", "install", "3.13.11"])
        python_install_env = python_install_call.kwargs["env"]
        self.assertEqual(
            python_install_env["UV_PYTHON_DOWNLOADS_JSON_URL"],
            "https://uv.agentsmirror.com/metadata/python-downloads.json",
        )
        self.assertEqual(python_install_env["UV_PYPY_INSTALL_MIRROR"], "https://uv.agentsmirror.com/pypy")
        self.assertNotIn("UV_PROJECT_ENVIRONMENT", python_install_env)

        sync_call = run_command.call_args_list[1]
        self.assertEqual(sync_call.args[0][:4], ["uv", "sync", "--project", "/tmp/project"])
        sync_env = sync_call.kwargs["env"]
        self.assertEqual(
            sync_env["UV_PYTHON_DOWNLOADS_JSON_URL"],
            "https://uv.agentsmirror.com/metadata/python-downloads.json",
        )
        self.assertEqual(sync_env["UV_PYPY_INSTALL_MIRROR"], "https://uv.agentsmirror.com/pypy")
        self.assertEqual(sync_env["UV_PROJECT_ENVIRONMENT"], env_path)


class UvMirrorRewriteTests(unittest.TestCase):
    def test_github_assets_mirror_url_preserves_github_release_path(self) -> None:
        self.assertEqual(
            sdk_env.github_assets_mirror_url(
                "https://downloads.sifli.com/github_assets",
                "https://github.com/OpenSiFli/crosstool-ng/releases/download/14.2.0-20250221/arm-none-eabi-14.2.0-aarch64-apple-darwin.tar.xz",
                "arm-none-eabi-14.2.0-aarch64-apple-darwin.tar.xz",
            ),
            "https://downloads.sifli.com/github_assets/OpenSiFli/crosstool-ng/releases/download/14.2.0-20250221/arm-none-eabi-14.2.0-aarch64-apple-darwin.tar.xz",
        )

    def test_github_assets_mirror_url_accepts_base_without_scheme(self) -> None:
        self.assertEqual(
            sdk_env.github_assets_mirror_url(
                "downloads.sifli.com/github_assets",
                "https://github.com/OpenSiFli/sftool/releases/download/0.1.16/sftool-0.1.16-aarch64-apple-darwin.tar.xz",
                "sftool-0.1.16-aarch64-apple-darwin.tar.xz",
            ),
            "https://downloads.sifli.com/github_assets/OpenSiFli/sftool/releases/download/0.1.16/sftool-0.1.16-aarch64-apple-darwin.tar.xz",
        )

    def test_derive_pypi_artifact_mirror_prefix_from_simple(self) -> None:
        self.assertEqual(
            sdk_env.derive_pypi_artifact_mirror_prefix("https://mirrors.ustc.edu.cn/pypi/simple"),
            "https://mirrors.ustc.edu.cn/pypi/packages",
        )

    def test_derive_pypi_artifact_mirror_prefix_from_web_simple(self) -> None:
        self.assertEqual(
            sdk_env.derive_pypi_artifact_mirror_prefix("https://mirror.example/pypi/web/simple"),
            "https://mirror.example/pypi/packages",
        )

    def test_derive_pypi_artifact_mirror_prefix_rejects_unknown_layout(self) -> None:
        self.assertIsNone(sdk_env.derive_pypi_artifact_mirror_prefix("https://mirror.example/custom-index"))

    def test_rewrite_uv_lock_for_index_rewrites_registry_and_artifacts(self) -> None:
        uv_lock_doc = {
            "version": 1,
            "package": [
                {
                    "name": "demo",
                    "source": {"registry": "https://pypi.org/simple"},
                    "sdist": {"url": "https://files.pythonhosted.org/packages/source/demo.tar.gz"},
                    "wheels": [
                        {"url": "https://files.pythonhosted.org/packages/wheel/demo.whl"},
                    ],
                }
            ],
        }

        rewritten, rewrote_registry, rewrote_artifacts = sdk_env.rewrite_uv_lock_for_index(
            uv_lock_doc,
            "https://mirrors.ustc.edu.cn/pypi/simple",
        )

        self.assertTrue(rewrote_registry)
        self.assertTrue(rewrote_artifacts)
        package = rewritten["package"][0]
        self.assertEqual(package["source"]["registry"], "https://mirrors.ustc.edu.cn/pypi/simple")
        self.assertEqual(package["sdist"]["url"], "https://mirrors.ustc.edu.cn/pypi/packages/source/demo.tar.gz")
        self.assertEqual(package["wheels"][0]["url"], "https://mirrors.ustc.edu.cn/pypi/packages/wheel/demo.whl")

    def test_rewrite_uv_lock_for_index_rewrites_registry_only_for_unknown_layout(self) -> None:
        uv_lock_doc = {
            "version": 1,
            "package": [
                {
                    "name": "demo",
                    "source": {"registry": "https://pypi.org/simple"},
                    "sdist": {"url": "https://files.pythonhosted.org/packages/source/demo.tar.gz"},
                    "wheels": [
                        {"url": "https://files.pythonhosted.org/packages/wheel/demo.whl"},
                    ],
                }
            ],
        }

        rewritten, rewrote_registry, rewrote_artifacts = sdk_env.rewrite_uv_lock_for_index(
            uv_lock_doc,
            "https://mirror.example/custom-index",
        )

        self.assertTrue(rewrote_registry)
        self.assertFalse(rewrote_artifacts)
        package = rewritten["package"][0]
        self.assertEqual(package["source"]["registry"], "https://mirror.example/custom-index")
        self.assertEqual(package["sdist"]["url"], "https://files.pythonhosted.org/packages/source/demo.tar.gz")
        self.assertEqual(package["wheels"][0]["url"], "https://files.pythonhosted.org/packages/wheel/demo.whl")

    def test_temporary_uv_project_writes_rewritten_lock_for_standard_mirror(self) -> None:
        temp_root = tempfile.mkdtemp(prefix="sdk-env-uv-project-")
        self.addCleanup(lambda: shutil.rmtree(temp_root, ignore_errors=True))
        profile_dir = os.path.join(temp_root, "tools", "locks", "default")
        os.makedirs(profile_dir, exist_ok=True)
        pyproject_path = os.path.join(profile_dir, "pyproject.toml")
        uv_lock_path = os.path.join(profile_dir, "uv.lock")
        with open(pyproject_path, "w", encoding="utf-8") as f:
            f.write(
                textwrap.dedent(
                    """
                    [project]
                    name = "demo"
                    version = "0.1.0"
                    requires-python = "==3.13.11"
                    dependencies = ["click"]
                    """
                ).strip()
            )
        with open(uv_lock_path, "w", encoding="utf-8") as f:
            f.write(
                textwrap.dedent(
                    """
                    version = 1
                    requires-python = "==3.13.11"

                    [[package]]
                    name = "click"
                    version = "8.3.2"
                    source = { registry = "https://pypi.org/simple" }
                    sdist = { url = "https://files.pythonhosted.org/packages/source/click.tar.gz" }
                    wheels = [{ url = "https://files.pythonhosted.org/packages/wheel/click.whl" }]
                    """
                ).strip()
            )

        config = sdk_env.RuntimeConfig(
            install_root=os.path.join(temp_root, ".sifli"),
            cache_root=os.path.join(temp_root, ".sifli", "cache"),
            staging_root=os.path.join(temp_root, ".sifli", "staging"),
            offline=False,
            python_default_index="https://mirrors.ustc.edu.cn/pypi/simple",
            python_indexes=[],
            python_index_strategy="first-index",
            sources=[],
        )
        lock = sdk_env.ProfileLock(
            path=os.path.join(profile_dir, "lock.json"),
            schema_version=1,
            profile="default",
            python_version="3.13.11",
            python_project_dir="tools/locks/default",
            python_lock_file="tools/locks/default/uv.lock",
            default_targets=["all"],
            tools={"sftool": "0.1.16"},
            path_order=["sftool"],
            conan_config_id="sdk.conan-config.v2.4",
            conan_remote_name="artifactory",
            conan_remote_url="https://example.com",
            conan_home_subdir="default",
        )

        with mock.patch("sdk_env.repo_root", return_value=temp_root):
            with sdk_env.temporary_uv_project(lock, config) as project_dir:
                self.assertNotEqual(project_dir, profile_dir)
                with open(os.path.join(project_dir, "uv.lock"), "rb") as f:
                    rewritten = tomllib.load(f)

        package = rewritten["package"][0]
        self.assertEqual(package["source"]["registry"], "https://mirrors.ustc.edu.cn/pypi/simple")
        self.assertEqual(package["sdist"]["url"], "https://mirrors.ustc.edu.cn/pypi/packages/source/click.tar.gz")
        self.assertEqual(package["wheels"][0]["url"], "https://mirrors.ustc.edu.cn/pypi/packages/wheel/click.whl")

    def test_download_to_path_uses_indeterminate_progress_when_size_unknown(self) -> None:
        temp_dir = tempfile.mkdtemp(prefix="sdk-env-download-")
        self.addCleanup(lambda: shutil.rmtree(temp_dir, ignore_errors=True))
        destination = os.path.join(temp_dir, "artifact.bin")
        response = FakeResponse([b"abc"])

        with mock.patch("sdk_env.should_show_download_progress", return_value=True):
            with mock.patch("sdk_env.Progress", FakeProgress):
                with mock.patch("urllib.request.urlopen", return_value=response):
                    sdk_env.download_to_path("https://example.com/tool.zip", destination, "tool.zip", -1)

        assert FakeProgress.last_instance is not None
        self.assertIsNone(FakeProgress.last_instance.tasks[0]["total"])

    def test_download_to_path_writes_file_when_progress_is_disabled(self) -> None:
        temp_dir = tempfile.mkdtemp(prefix="sdk-env-download-")
        self.addCleanup(lambda: shutil.rmtree(temp_dir, ignore_errors=True))
        destination = os.path.join(temp_dir, "artifact.bin")
        response = FakeResponse([b"abc", b"def"], headers={"Content-Length": "6"})

        with mock.patch("sdk_env.should_show_download_progress", return_value=False):
            with mock.patch("urllib.request.urlopen", return_value=response):
                sdk_env.download_to_path("https://example.com/tool.zip", destination, "tool.zip", 6)

        with open(destination, "rb") as f:
            self.assertEqual(f.read(), b"abcdef")


if __name__ == "__main__":
    unittest.main()
