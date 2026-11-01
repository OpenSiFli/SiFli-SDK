#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

# Unit tests for the configdefault validation in the vendored Kconfig library.
#
# `configdefault` is a SiFli/Zephyr extension: it sets a default on a symbol
# defined elsewhere. When the Kconfig file defining the target symbol is not
# sourced (e.g. an optional module was not fetched), an active configdefault
# is a hard error, while an inactive one is inert, exactly like
# `default <value> if <false>`.

import glob
import importlib.util
import io
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

ROOT = os.path.realpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))
LIB_PATH = os.path.join(ROOT, "tools", "kconfig", "kconfiglib.py")
KCONFIG_PY = os.path.join(ROOT, "tools", "kconfig", "kconfig.py")
SOC_57X_DIR = os.path.join(ROOT, "soc", "sf32lb57x")
KCONFIG_DRV = os.path.join(ROOT, "customer", "boards", "Kconfig_drv")


def _load_kconfiglib():
    # The SDK environment puts tools/build on PYTHONPATH, which holds an old
    # kconfiglib copy without configdefault support. Load the library under
    # test by path so a bare "import kconfiglib" cannot pick the wrong one.
    spec = importlib.util.spec_from_file_location(
        "sifli_kconfiglib_under_test", LIB_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    if os.path.realpath(mod.__file__) != os.path.realpath(LIB_PATH):
        raise AssertionError("loaded the wrong kconfiglib: " + mod.__file__)
    return mod


kconfiglib = _load_kconfiglib()

COMMON_KCONFIG = """\
config AUDIO
    bool "audio"
config TEST_CONFIG
    bool "test config"
"""

MODULE_KCONFIG = """\
config ABCD
    bool "test abcd"
"""

BOARD_DEFCONFIG = """\
if AUDIO
configdefault {target}
    default y if TEST_CONFIG
endif
"""


class ConfigdefaultTestCase(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmpdir, True)

    def write(self, relpath, text):
        path = os.path.join(self.tmpdir, relpath)
        dirname = os.path.dirname(path)
        if dirname and not os.path.isdir(dirname):
            os.makedirs(dirname)
        with io.open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
        return path

    def kconf(self, relpath):
        return kconfiglib.Kconfig(
            os.path.join(self.tmpdir, relpath),
            warn_to_stderr=False, suppress_traceback=True)

    def load(self, kconf, fragments):
        for i, frag in enumerate(fragments):
            kconf.load_config(os.path.join(self.tmpdir, frag),
                              replace=(i == 0))

    def configdefault_warnings(self, kconf):
        return [w for w in kconf.warnings if "configdefault" in w]

    def write_audio_tree(self, target="ABCD"):
        self.write("common/kconfig", COMMON_KCONFIG)
        self.write("module/kconfig", MODULE_KCONFIG)
        self.write("board/kconfig.defconfig",
                   BOARD_DEFCONFIG.format(target=target))
        self.write("Kconfig.with_module",
                   'rsource "board/kconfig.defconfig"\n'
                   'rsource "common/kconfig"\n'
                   'rsource "module/kconfig"\n')
        self.write("Kconfig.without_module",
                   'rsource "board/kconfig.defconfig"\n'
                   'rsource "common/kconfig"\n')
        self.write("audio_on.config", "CONFIG_AUDIO=y\n"
                                     "CONFIG_TEST_CONFIG=y\n")
        self.write("module_absent.config", "CONFIG_TEST_CONFIG=y\n")
        self.write("empty.config", "")


class TestConfigdefaultValidation(ConfigdefaultTestCase):
    def test_inert_when_module_absent(self):
        # AUDIO is defined after the defconfig file and stays n, so the
        # configdefault can never take effect. It must not be diagnosed.
        self.write_audio_tree()
        kconf = self.kconf("Kconfig.without_module")
        self.load(kconf, ["module_absent.config"])
        kconf.check_undefined_configdefaults()

        self.assertEqual([], self.configdefault_warnings(kconf))
        self.assertFalse([w for w in kconf.warnings
                          if "ABCD" in w and "defined without a type" in w])

    def test_applies_when_module_present(self):
        self.write_audio_tree()
        kconf = self.kconf("Kconfig.with_module")
        self.load(kconf, ["audio_on.config"])
        kconf.check_undefined_configdefaults()

        self.assertEqual("y", kconf.syms["ABCD"].str_value)
        self.assertEqual([], self.configdefault_warnings(kconf))

    def test_int_configdefault_when_defined(self):
        self.write("Kconfig", 'configdefault NUM\n'
                              '    default 16\n'
                              'config NUM\n'
                              '    int "num"\n')
        self.write("empty.config", "")
        kconf = self.kconf("Kconfig")
        self.load(kconf, ["empty.config"])
        kconf.check_undefined_configdefaults()

        self.assertEqual("16", kconf.syms["NUM"].str_value)
        self.assertEqual([], self.configdefault_warnings(kconf))

    def test_active_undefined_target_warns(self):
        # The module is sourced but defines ABCD, not the misspelled ABDC.
        self.write_audio_tree(target="ABDC")
        kconf = self.kconf("Kconfig.with_module")
        self.load(kconf, ["audio_on.config"])
        kconf.check_undefined_configdefaults()

        warnings = self.configdefault_warnings(kconf)
        self.assertEqual(1, len(warnings))
        self.assertIn("configdefault for ABDC is active", warnings[0])
        self.assertIn("kconfig.defconfig", warnings[0])
        self.assertIn("AUDIO", warnings[0])

    def test_unconditional_undefined_target_warns(self):
        self.write("Kconfig", 'configdefault NOPE\n'
                              '    default y\n')
        self.write("empty.config", "")
        kconf = self.kconf("Kconfig")
        self.load(kconf, ["empty.config"])
        kconf.check_undefined_configdefaults()

        warnings = self.configdefault_warnings(kconf)
        self.assertEqual(1, len(warnings))
        self.assertIn("(condition: y)", warnings[0])

    def test_typeless_config_still_warns(self):
        self.write("Kconfig", "config FOO\n")
        self.write("empty.config", "")
        kconf = self.kconf("Kconfig")
        self.load(kconf, ["empty.config"])
        kconf.check_undefined_configdefaults()

        self.assertTrue([w for w in kconf.warnings
                         if "FOO" in w and "defined without a type" in w])

    def test_module_absent_but_guard_active_warns(self):
        # Forward definition works: AUDIO is defined after the defconfig, and
        # the configuration turns it on, but the module was not sourced.
        self.write_audio_tree()
        kconf = self.kconf("Kconfig.without_module")
        self.load(kconf, ["audio_on.config"])
        kconf.check_undefined_configdefaults()

        self.assertEqual(1, len(self.configdefault_warnings(kconf)))

    def test_only_active_block_warns_once(self):
        self.write("Kconfig",
                   'if NEVER_DEFINED\n'
                   'configdefault ABDC\n'
                   '    default y\n'
                   'endif\n'
                   'if AUDIO\n'
                   'configdefault ABDC\n'
                   '    default y\n'
                   'endif\n'
                   'config AUDIO\n'
                   '    bool "audio"\n')
        self.write("audio_on.config", "CONFIG_AUDIO=y\n")
        kconf = self.kconf("Kconfig")
        self.load(kconf, ["audio_on.config"])
        kconf.check_undefined_configdefaults()

        warnings = self.configdefault_warnings(kconf)
        self.assertEqual(1, len(warnings))
        self.assertIn("(condition: AUDIO)", warnings[0])

    def test_false_dependency_still_counts_as_defined(self):
        self.write("Kconfig",
                   'configdefault ABCD\n'
                   '    default y\n'
                   'config ABCD\n'
                   '    bool "abcd"\n'
                   '    depends on NOPE\n')
        self.write("empty.config", "")
        kconf = self.kconf("Kconfig")
        self.load(kconf, ["empty.config"])
        kconf.check_undefined_configdefaults()

        self.assertEqual([], self.configdefault_warnings(kconf))

    def test_multiple_defaults_in_one_block(self):
        self.write("Kconfig",
                   'configdefault ABDC\n'
                   '    default y if NEVER_DEFINED\n'
                   '    default y if AUDIO\n'
                   'config AUDIO\n'
                   '    bool "audio"\n')
        self.write("audio_on.config", "CONFIG_AUDIO=y\n")
        kconf = self.kconf("Kconfig")
        self.load(kconf, ["audio_on.config"])
        kconf.check_undefined_configdefaults()

        warnings = self.configdefault_warnings(kconf)
        self.assertEqual(1, len(warnings))
        self.assertIn("(condition: AUDIO)", warnings[0])

    def test_requires_loaded_config(self):
        self.write("Kconfig", 'config FOO\n    bool "foo"\n')
        kconf = self.kconf("Kconfig")
        with self.assertRaises(kconfiglib.KconfigError):
            kconf.check_undefined_configdefaults()


class TestDriverIntegration(ConfigdefaultTestCase):
    def run_driver(self, root, fragment):
        out_config = os.path.join(self.tmpdir, "out.config")
        out_header = os.path.join(self.tmpdir, "rtconfig.h")
        out_list = os.path.join(self.tmpdir, "kconfig_files.txt")
        return subprocess.run(
            [sys.executable, KCONFIG_PY, "--handwritten-input-configs",
             os.path.join(self.tmpdir, root), out_config, out_header,
             out_list, os.path.join(self.tmpdir, fragment)],
            cwd=self.tmpdir, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            universal_newlines=True)

    def test_driver_accepts_inert_configdefault(self):
        self.write_audio_tree()
        result = self.run_driver("Kconfig.without_module",
                                 "module_absent.config")
        self.assertEqual(0, result.returncode, result.stderr)

    def test_driver_rejects_active_undefined_target(self):
        self.write_audio_tree(target="ABDC")
        # The module is sourced (so AUDIO is defined) and the fragment turns
        # the guard on, but the configdefault target is misspelled.
        result = self.run_driver("Kconfig.with_module", "audio_on.config")
        self.assertNotEqual(0, result.returncode)
        self.assertIn("configdefault", result.stderr)


class TestExistingSocConfigdefaults(ConfigdefaultTestCase):
    def _parse_real_tree(self, with_drv):
        soc_files = sorted(glob.glob(os.path.join(SOC_57X_DIR, "Kconfig.*")))
        self.assertTrue(soc_files)
        parts = [os.path.basename(path)[len("Kconfig."):].upper()
                 for path in soc_files]

        # The SOC_<part> symbols are promptless, so assigning them in a
        # .config fragment is ignored. Select them from a prompt'ed symbol
        # instead; otherwise every configdefault condition stays inactive and
        # the positive test below would pass even if all targets were deleted.
        lines = ['config TEST_ALL_PARTS',
                 '    bool "select all parts"',
                 '    default y']
        lines += ['    select SOC_{}'.format(part) for part in parts]
        lines += ['',
                  'source "{}"'.format(
                      os.path.join(ROOT, "customer", "boards",
                                   "Kconfig_soc.sf32lb57x")
                      .replace(os.sep, "/"))]
        if with_drv:
            lines.append('source "{}"'.format(
                KCONFIG_DRV.replace(os.sep, "/")))
        self.write("Kconfig.soc", "\n".join(lines) + "\n")
        self.write("empty.config", "")

        with mock.patch.dict(os.environ,
                             {"SIFLI_SDK": ROOT.replace(os.sep, "/")}):
            kconf = kconfiglib.Kconfig(
                os.path.join(self.tmpdir, "Kconfig.soc"),
                warn_to_stderr=False, suppress_traceback=True)
            self.load(kconf, ["empty.config"])
            kconf.check_undefined_configdefaults()
        return kconf

    def test_all_targets_of_real_soc_files_are_defined(self):
        kconf = self._parse_real_tree(with_drv=True)
        self.assertEqual([], self.configdefault_warnings(kconf))

    def test_missing_target_definitions_are_detected(self):
        # Negative control for the test above: without Kconfig_drv, the
        # targets it defines do not exist and the check must fire. This proves
        # the conditions are really active and the positive test is not
        # vacuous.
        kconf = self._parse_real_tree(with_drv=False)
        warnings = self.configdefault_warnings(kconf)
        self.assertTrue(warnings)
        self.assertTrue(any("BSP_ENABLE_MPI1" in warning
                            for warning in warnings))


if __name__ == "__main__":
    unittest.main()
