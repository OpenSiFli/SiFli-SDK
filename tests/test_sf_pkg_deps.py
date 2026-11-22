#!/usr/bin/env python3
#
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import contextlib
import io
import os
import sys
import tempfile
import unittest
from unittest import mock

ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import sdk_py_actions.sf_pkg_deps as sf_pkg_deps  # noqa: E402


def _write(path: str, text: str) -> str:
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    return path


def _load_yaml_manifest(tmpdir: str, text: str):
    path = _write(os.path.join(tmpdir, "sf-pkg.yaml"), text)
    return sf_pkg_deps.load_manifest(path), path


class ParseRequiresTests(unittest.TestCase):
    def test_legacy_string_list(self):
        with tempfile.TemporaryDirectory() as td:
            manifest, _ = _load_yaml_manifest(
                td,
                "requires:\n"
                "  - sht30/0.0.4@acme\n"
                "  - mesh-lib/[>=1.3 <2.0]@sifli\n",
            )
        self.assertEqual(
            manifest["requires"],
            [
                {"package": "sht30/0.0.4@acme", "enable": []},
                {"package": "mesh-lib/[>=1.3 <2.0]@sifli", "enable": []},
            ],
        )

    def test_single_string_requires(self):
        with tempfile.TemporaryDirectory() as td:
            manifest, _ = _load_yaml_manifest(td, "requires: sht30/0.0.4@acme\n")
        self.assertEqual(
            manifest["requires"],
            [{"package": "sht30/0.0.4@acme", "enable": []}],
        )

    def test_absent_requires(self):
        with tempfile.TemporaryDirectory() as td:
            manifest, _ = _load_yaml_manifest(td, "enable: [MOD_X]\n")
        self.assertEqual(manifest["requires"], [])

    def test_mixed_entries(self):
        with tempfile.TemporaryDirectory() as td:
            manifest, _ = _load_yaml_manifest(
                td,
                "requires:\n"
                "  - common-util/1.0.0@acme\n"
                "  - package: hw-codec/2.1.0@sifli\n"
                "    enable: [MODULE_CODEC_SUPPORT]\n",
            )
        self.assertEqual(
            manifest["requires"],
            [
                {"package": "common-util/1.0.0@acme", "enable": []},
                {
                    "package": "hw-codec/2.1.0@sifli",
                    "enable": ["MODULE_CODEC_SUPPORT"],
                },
            ],
        )

    def test_entry_enable_single_string(self):
        with tempfile.TemporaryDirectory() as td:
            manifest, _ = _load_yaml_manifest(
                td,
                "requires:\n"
                "  - package: sensor-lib/[>=1.2 <2.0]@sifli\n"
                "    enable: SENSOR_ENABLE\n",
            )
        self.assertEqual(
            manifest["requires"][0]["enable"], ["SENSOR_ENABLE"]
        )

    def test_entry_enable_multiline_list(self):
        with tempfile.TemporaryDirectory() as td:
            manifest, _ = _load_yaml_manifest(
                td,
                "requires:\n"
                "  - package: camera-lib/2.0.0@sifli\n"
                "    enable:\n"
                "      - CAMERA_ENABLE\n"
                "      - CAMERA_OV2640\n",
            )
        self.assertEqual(
            manifest["requires"][0]["enable"],
            ["CAMERA_ENABLE", "CAMERA_OV2640"],
        )

    def test_package_only_entry_has_empty_enable(self):
        with tempfile.TemporaryDirectory() as td:
            manifest, _ = _load_yaml_manifest(
                td, "requires:\n  - package: crypto/2.0.1@sifli\n"
            )
        self.assertEqual(
            manifest["requires"],
            [{"package": "crypto/2.0.1@sifli", "enable": []}],
        )

    def test_explicit_empty_enable_list(self):
        with tempfile.TemporaryDirectory() as td:
            manifest, _ = _load_yaml_manifest(
                td,
                "requires:\n"
                "  - package: crypto/2.0.1@sifli\n"
                "    enable: []\n",
            )
        self.assertEqual(manifest["requires"][0]["enable"], [])


class ParseErrorTests(unittest.TestCase):
    def _assert_error(self, text: str, *needles: str):
        with tempfile.TemporaryDirectory() as td:
            path = _write(os.path.join(td, "sf-pkg.yaml"), text)
            with self.assertRaises(sf_pkg_deps.SfPkgError) as ctx:
                sf_pkg_deps.load_manifest(path)
        message = str(ctx.exception)
        self.assertIn(path, message)
        for needle in needles:
            self.assertIn(needle, message)

    def test_unknown_entry_key(self):
        self._assert_error(
            "requires:\n"
            "  - package: a/1.0@acme\n"
            "    enabled: [X]\n",
            "requires[0]",
            "enabled",
        )

    def test_missing_package(self):
        self._assert_error(
            "requires:\n  - enable: [X]\n",
            "requires[0]",
            "package",
        )

    def test_empty_package(self):
        self._assert_error(
            'requires:\n  - package: ""\n',
            "requires[0].package",
        )

    def test_non_string_package(self):
        self._assert_error(
            "requires:\n  - package: 123\n",
            "requires[0].package",
        )

    def test_requires_is_mapping(self):
        self._assert_error(
            "requires:\n  a: b\n",
            "requires",
        )

    def test_entry_is_number(self):
        self._assert_error(
            "requires:\n  - 42\n",
            "requires[0]",
        )

    def test_entry_is_null(self):
        self._assert_error(
            "requires:\n  -\n",
            "requires[0]",
        )

    def test_entry_enable_non_string(self):
        self._assert_error(
            "requires:\n"
            "  - package: a/1.0@acme\n"
            "    enable: 5\n",
            "requires[0].enable",
        )


class RootManifestTests(unittest.TestCase):
    def test_root_accepts_string_entries(self):
        with tempfile.TemporaryDirectory() as td:
            _write(
                os.path.join(td, "sf-pkg.yaml"),
                "requires:\n  - sht30/0.0.4@acme\n",
            )
            root = sf_pkg_deps.load_root_manifest(td)
        self.assertEqual(
            root["requires"],
            [{"package": "sht30/0.0.4@acme", "enable": []}],
        )

    def test_root_rejects_mapping_entry(self):
        with tempfile.TemporaryDirectory() as td:
            path = _write(
                os.path.join(td, "sf-pkg.yaml"),
                "requires:\n"
                "  - package: factory-test-lib/1.0.0@sifli\n"
                "    enable: [FACTORY_TEST]\n",
            )
            with self.assertRaises(sf_pkg_deps.SfPkgError) as ctx:
                sf_pkg_deps.load_root_manifest(td)
        message = str(ctx.exception)
        self.assertIn(path, message)
        self.assertIn("requires[0]", message)
        self.assertIn("module/board manifests", message)

    def test_root_rejects_mapping_entry_without_enable(self):
        with tempfile.TemporaryDirectory() as td:
            _write(
                os.path.join(td, "sf-pkg.yaml"),
                "requires:\n  - package: crypto/2.0.1@sifli\n",
            )
            with self.assertRaises(sf_pkg_deps.SfPkgError):
                sf_pkg_deps.load_root_manifest(td)


class EntryGatingTests(unittest.TestCase):
    def _manifest(self, entry_enable, manifest_enable=None):
        manifest = {
            "requires": [{"package": "a/1.0@acme", "enable": entry_enable}]
        }
        if manifest_enable is not None:
            manifest["enable"] = manifest_enable
        return manifest

    def test_entry_symbol_y(self):
        self.assertEqual(
            sf_pkg_deps.collect_requires(
                [self._manifest(["E"])], {"E": "y"}
            ),
            ["a/1.0@acme"],
        )

    def test_entry_symbol_n(self):
        self.assertEqual(
            sf_pkg_deps.collect_requires(
                [self._manifest(["E"])], {"E": "n"}
            ),
            [],
        )

    def test_entry_symbol_absent(self):
        self.assertEqual(
            sf_pkg_deps.collect_requires([self._manifest(["E"])], {}),
            [],
        )

    def test_entry_without_enable_is_unconditional(self):
        self.assertEqual(
            sf_pkg_deps.collect_requires([self._manifest([])], {}),
            ["a/1.0@acme"],
        )

    def test_manifest_and_entry_gates(self):
        cases = [
            ({"M": "y", "E": "y"}, ["a/1.0@acme"]),
            ({"M": "y", "E": "n"}, []),
            ({"M": "n", "E": "y"}, []),
            ({"M": "n", "E": "n"}, []),
        ]
        for config, expected in cases:
            with self.subTest(config=config):
                self.assertEqual(
                    sf_pkg_deps.collect_requires(
                        [self._manifest(["E"], ["M"])], config
                    ),
                    expected,
                )

    def test_manifest_gate_off_beats_entry_symbol_y(self):
        self.assertEqual(
            sf_pkg_deps.collect_requires(
                [self._manifest(["E"], ["M1", "M2"])],
                {"M1": "n", "M2": "n", "E": "y"},
            ),
            [],
        )

    def test_multiple_entry_symbols_are_ored(self):
        manifest = self._manifest(["E1", "E2"])
        self.assertEqual(
            sf_pkg_deps.collect_requires([manifest], {"E1": "n", "E2": "y"}),
            ["a/1.0@acme"],
        )

    def test_entry_enabled_direct(self):
        self.assertTrue(sf_pkg_deps.entry_enabled({"enable": []}, {}))
        self.assertTrue(sf_pkg_deps.entry_enabled({"enable": ["E"]}, {"E": "y"}))
        self.assertFalse(sf_pkg_deps.entry_enabled({"enable": ["E"]}, {"E": "n"}))


class AggregateTests(unittest.TestCase):
    def test_dedupe_and_stable_order(self):
        merged = sf_pkg_deps.aggregate_requires(
            ["a/1.0@acme", "b/1.0@acme"],
            ["b/1.0@acme", "c/1.0@acme"],
            ["a/1.0@acme"],
        )
        self.assertEqual(merged, ["a/1.0@acme", "b/1.0@acme", "c/1.0@acme"])

    def test_empty_groups(self):
        self.assertEqual(sf_pkg_deps.aggregate_requires([], [], []), [])


class DuplicateAcrossManifestsTests(unittest.TestCase):
    def test_duplicate_gated_entry_collected_once(self):
        first = {
            "requires": [{"package": "dup/1.0@acme", "enable": ["A"]}]
        }
        second = {
            "requires": [{"package": "dup/1.0@acme", "enable": ["B"]}]
        }
        # Only the second manifest's entry passes its gate; the reference must
        # still appear exactly once in the collected list.
        collected = sf_pkg_deps.collect_requires(
            [first, second], {"A": "n", "B": "y"}
        )
        self.assertEqual(collected, ["dup/1.0@acme"])

        merged = sf_pkg_deps.aggregate_requires(
            collected, ["other/1.0@acme", "dup/1.0@acme"]
        )
        self.assertEqual(merged, ["dup/1.0@acme", "other/1.0@acme"])

    def test_first_group_order_wins(self):
        self.assertEqual(
            sf_pkg_deps.aggregate_requires(
                ["b/1.0@acme"], ["a/1.0@acme", "b/1.0@acme"]
            ),
            ["b/1.0@acme", "a/1.0@acme"],
        )


class CheckSdkVersionTests(unittest.TestCase):
    def _manifest(self, declared):
        manifest = {"path": os.path.join("tmp", "sf-pkg.yaml")}
        if declared is not None:
            manifest["support_sdk_version"] = declared
        return manifest

    def test_satisfied_range(self):
        with mock.patch.dict(
            os.environ, {"SIFLI_SDK_VERSION": "2.5"}, clear=False
        ):
            sf_pkg_deps.check_sdk_versions(
                sf_pkg_deps.sdk_version_from_env(),
                [self._manifest(">=2.4 <3")],
            )

    def test_missing_env_with_declared_range(self):
        with mock.patch.dict(
            os.environ, {"SIFLI_SDK_VERSION": ""}, clear=False
        ):
            with self.assertRaises(sf_pkg_deps.SfPkgError) as ctx:
                sf_pkg_deps.check_sdk_versions(
                    sf_pkg_deps.sdk_version_from_env(),
                    [self._manifest(">=2.4 <3")],
                )
        self.assertIn("SIFLI_SDK_VERSION", str(ctx.exception))

    def test_unsatisfied_range(self):
        with mock.patch.dict(
            os.environ, {"SIFLI_SDK_VERSION": "3.0"}, clear=False
        ):
            with self.assertRaises(sf_pkg_deps.SfPkgError):
                sf_pkg_deps.check_sdk_versions(
                    sf_pkg_deps.sdk_version_from_env(),
                    [self._manifest(">=2.4 <3")],
                )


class FingerprintTests(unittest.TestCase):
    def _manifest(self, entry_enable):
        return {
            "requires": [
                {"package": "a/1.0@acme", "enable": []},
                {"package": "b/1.0@acme", "enable": entry_enable},
            ]
        }

    def test_entry_switch_changes_fingerprint(self):
        on = sf_pkg_deps.collect_requires([self._manifest(["E"])], {"E": "y"})
        off = sf_pkg_deps.collect_requires([self._manifest(["E"])], {"E": "n"})
        self.assertNotEqual(
            sf_pkg_deps.fingerprint(on, "2.4"),
            sf_pkg_deps.fingerprint(off, "2.4"),
        )

    def test_same_package_set_same_fingerprint(self):
        a = sf_pkg_deps.collect_requires(
            [self._manifest(["E1"])], {"E1": "y"}
        )
        b = sf_pkg_deps.collect_requires(
            [self._manifest(["E2"])], {"E2": "y"}
        )
        self.assertEqual(a, b)
        self.assertEqual(
            sf_pkg_deps.fingerprint(a, "2.4"),
            sf_pkg_deps.fingerprint(b, "2.4"),
        )


class BoardIntegrationTests(unittest.TestCase):
    """Merge root + module manifests through ensure_board_deps, no conan."""

    def _setup(self, td, codec_y):
        project = os.path.join(td, "project")
        module_dir = os.path.join(td, "middleware", "mod")
        os.makedirs(project)
        os.makedirs(module_dir)
        _write(
            os.path.join(project, "sf-pkg.yaml"),
            "requires:\n  - root-lib/1.0.0@acme\n",
        )
        _write(os.path.join(module_dir, "Kconfig"), "config MOD_ENABLE\n\tbool\n")
        _write(
            os.path.join(module_dir, "sf-pkg.yaml"),
            "enable: [MOD_ENABLE]\n"
            "requires:\n"
            "  - always-lib/1.0.0@acme\n"
            "  - package: gated-lib/1.0.0@acme\n"
            "    enable: [MOD_CODEC]\n",
        )
        build_dir = os.path.join(project, "build_test")
        os.makedirs(build_dir)
        _write(
            os.path.join(build_dir, ".config"),
            "CONFIG_MOD_ENABLE=y\n"
            + ("CONFIG_MOD_CODEC=y\n" if codec_y
               else "# CONFIG_MOD_CODEC is not set\n"),
        )
        _write(
            os.path.join(build_dir, "kconfiglist"),
            os.path.join(module_dir, "Kconfig") + "\n",
        )
        return project, build_dir

    def _run(self, project, build_dir):
        captured = {}
        install_calls = []

        def fake_generate(project_dir, merged):
            captured["merged"] = list(merged)
            return os.path.join(project_dir, ".sf-pkg", "conanfile.py")

        def fake_install(project_dir, consumer):
            install_calls.append((project_dir, consumer))

        out = io.StringIO()
        # write_installed_fingerprint is deliberately NOT mocked: the real
        # .sf-pkg/installed.fingerprint must drive the "unchanged -> skip"
        # branch, otherwise a broken fingerprint would still look green.
        with mock.patch.dict(
            os.environ, {"SIFLI_SF_PKG_OFFLINE": ""}, clear=False
        ), mock.patch.object(
            sf_pkg_deps, "generate_consumer_conanfile", fake_generate
        ), mock.patch.object(
            sf_pkg_deps, "install", fake_install
        ), contextlib.redirect_stdout(out):
            changed = sf_pkg_deps.ensure_board_deps(project, build_dir)
        return changed, captured, install_calls, out.getvalue()

    def test_resolve_for_board(self):
        with tempfile.TemporaryDirectory() as td:
            project, build_dir = self._setup(td, codec_y=False)
            enabled, config, root = sf_pkg_deps.resolve_for_board(
                project, build_dir
            )
        self.assertEqual(len(enabled), 1)
        self.assertEqual(config["MOD_ENABLE"], "y")
        self.assertEqual(config["MOD_CODEC"], "n")
        self.assertIsNotNone(root)

    def test_merge_without_board_config(self):
        with tempfile.TemporaryDirectory() as td:
            project, _ = self._setup(td, codec_y=False)
            enabled, config, root = sf_pkg_deps.resolve_for_board(
                project, None
            )
        self.assertEqual(enabled, [])
        self.assertEqual(config, {})

    def test_entry_gate_on_module_manifest(self):
        with tempfile.TemporaryDirectory() as td:
            project, build_dir = self._setup(td, codec_y=False)

            # First run: nothing installed yet -> install once.
            changed, captured, installs, output = self._run(project, build_dir)
            self.assertTrue(changed)
            self.assertEqual(
                captured["merged"],
                ["root-lib/1.0.0@acme", "always-lib/1.0.0@acme"],
            )
            self.assertEqual(len(installs), 1)
            self.assertIn("2 manifest(s)", output)

            # Same config again: identical fingerprint -> no reinstall.
            changed, captured, installs, _ = self._run(project, build_dir)
            self.assertFalse(changed)
            self.assertEqual(installs, [])

            # Flip the entry switch on: the package reappears and the
            # fingerprint changes, so a reinstall is triggered.
            _write(
                os.path.join(build_dir, ".config"),
                "CONFIG_MOD_ENABLE=y\nCONFIG_MOD_CODEC=y\n",
            )
            changed, captured, installs, _ = self._run(project, build_dir)
            self.assertTrue(changed)
            self.assertEqual(len(installs), 1)
            self.assertEqual(
                captured["merged"],
                [
                    "root-lib/1.0.0@acme",
                    "always-lib/1.0.0@acme",
                    "gated-lib/1.0.0@acme",
                ],
            )


if __name__ == "__main__":
    unittest.main()
