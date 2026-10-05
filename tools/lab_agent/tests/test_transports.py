# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the flash transports' pure logic.

Flashing itself needs hardware, but the parts that decide *what* gets programmed
are worth pinning: a wrong address silently bricks a board, and a missing image
must be caught before the probe is invoked rather than during it.
"""

from __future__ import annotations

import json
import subprocess
import unittest
from unittest import mock

from _support import LabTestCase

from lab_agent.errors import TransportError
from lab_agent.transport import jlink as jlink_module
from lab_agent.transport.jlink import JLinkTransport
from lab_agent.transport.jlink import load_lines_from_download_script
from lab_agent.transport.sftool import build_flash_arguments
from lab_agent.transport.sftool import normalize_uart_port
from lab_agent.transport.sftool import read_sftool_param

#: Exactly the shape tools/build/sdk_resource.py generates.
DOWNLOAD_JLINK = """\
si SWD
speed 10000
r
loadbin ER_IROM1.bin 0x12000000
loadbin fs.bin 0x64200000
loadfile main.hex
exit
"""


class TestDownloadScriptParsing(LabTestCase):
    def test_the_load_lines_are_extracted_verbatim(self):
        path = self.write("build_x/download.jlink", DOWNLOAD_JLINK)
        self.assertEqual(
            load_lines_from_download_script(path),
            [
                "loadbin ER_IROM1.bin 0x12000000",
                "loadbin fs.bin 0x64200000",
                "loadfile main.hex",
            ],
        )

    def test_the_probe_preamble_is_not_replayed(self):
        # `si`, `speed`, `r` and `exit` are replaced by the transport's own
        # preamble so a probe serial number can be selected; replaying the
        # build's `r` would reset the board mid-programming.
        path = self.write("build_x/download.jlink", DOWNLOAD_JLINK)
        loads = load_lines_from_download_script(path)
        for noise in ("si SWD", "speed 10000", "exit"):
            self.assertNotIn(noise, loads)

    def test_trailing_comments_are_stripped(self):
        path = self.write(
            "build_x/download.jlink",
            "loadbin a.bin 0x1000 // the boot image\n",
        )
        self.assertEqual(load_lines_from_download_script(path), ["loadbin a.bin 0x1000"])

    def test_a_script_with_no_loads_yields_nothing(self):
        path = self.write("build_x/download.jlink", "si SWD\nr\nexit\n")
        self.assertEqual(load_lines_from_download_script(path), [])

    def test_a_missing_script_is_reported(self):
        with self.assertRaises(TransportError):
            load_lines_from_download_script(self.root + "/absent.jlink")


class TestSftoolParam(LabTestCase):
    def make_param(self, payload: dict, images=("ER_IROM1.bin", "main.hex")) -> str:
        """Write a build directory containing ``payload`` and placeholder images."""
        directory = self.mkdir("build_x")
        for name in images:
            self.write(f"build_x/{name}", "placeholder")
        self.write("build_x/sftool_param.json", json.dumps(payload, indent=2))
        return directory

    def test_images_become_file_at_address_arguments(self):
        build_dir = self.make_param(
            {
                "chip": "SF32LB56",
                "memory": "NAND",
                "write_flash": {
                    "verify": True,
                    "files": [
                        {"path": "ER_IROM1.bin", "address": "0x12000000"},
                        {"path": "main.hex"},
                    ],
                },
            }
        )
        config = read_sftool_param(build_dir)
        arguments = build_flash_arguments(build_dir, config)
        # A hex/elf image carries no address: the file itself declares where it
        # goes. Only raw binaries get `file@address`.
        self.assertEqual(len(arguments), 2)
        self.assertTrue(arguments[0].endswith("ER_IROM1.bin@0x12000000"))
        self.assertTrue(arguments[1].endswith("main.hex"))
        self.assertNotIn("@", arguments[1])

    def test_relative_paths_resolve_against_the_build_directory(self):
        build_dir = self.mkdir("build_x")
        self.write("build_x/image.bin", "x")
        self.write(
            "build_x/sftool_param.json",
            json.dumps(
                {
                    "chip": "SF32LB52",
                    "memory": "NOR",
                    "write_flash": {"files": [{"path": "image.bin", "address": "0x0"}]},
                }
            ),
        )
        arguments = build_flash_arguments(build_dir, read_sftool_param(build_dir))
        self.assertTrue(arguments[0].startswith(build_dir.replace("/", "\\")) or
                        arguments[0].startswith(build_dir))

    def test_a_missing_image_is_caught_before_the_probe_is_used(self):
        build_dir = self.make_param(
            {
                "chip": "SF32LB52",
                "memory": "NOR",
                "write_flash": {"files": [{"path": "absent.bin", "address": "0x0"}]},
            }
        )
        with self.assertRaises(TransportError) as caught:
            build_flash_arguments(build_dir, read_sftool_param(build_dir))
        self.assertIn("absent.bin", str(caught.exception))

    def test_an_empty_image_list_is_reported(self):
        build_dir = self.make_param(
            {"chip": "SF32LB52", "memory": "NOR", "write_flash": {"files": []}}
        )
        with self.assertRaises(TransportError) as caught:
            build_flash_arguments(build_dir, read_sftool_param(build_dir))
        self.assertIn("no flashable images", str(caught.exception))

    def test_a_missing_param_file_names_the_build_step(self):
        with self.assertRaises(TransportError) as caught:
            read_sftool_param(self.root)
        self.assertIn("build the project first", str(caught.exception))

    def test_a_malformed_param_file_is_reported(self):
        self.write("build_x/sftool_param.json", "{not json")
        with self.assertRaises(TransportError):
            read_sftool_param(self.mkdir("build_x"))


class TestFlashRunsInTheBuildDirectory(LabTestCase):
    """The probe is run from the build directory, because the script needs it to be.

    A generated ``download.jlink`` names its images relatively -- the real one
    this board's build produced says ``loadbin output\\main.bin 0x620A0000`` --
    and the build's own launcher resolves that by ``cd``-ing into the build
    directory first. This transport writes its script to the system temp
    directory instead, so without a cwd those paths are resolved against
    wherever the agent happened to be started: the flash fails, or it finds
    some other file that happens to sit at that relative path.
    """

    def flash_with_a_faked_probe(self, build_dir: str) -> dict:
        """Run ``flash``; return what the probe was told and the script it read."""
        seen: dict = {}

        def fake_run(command, **kwargs):
            script_path = command[command.index("-CommandFile") + 1]
            # Read it here: the transport deletes it as soon as this returns.
            with open(script_path, "r", encoding="ascii") as handle:
                seen["script"] = handle.read()
            seen["kwargs"] = kwargs
            return subprocess.CompletedProcess(command, 0, "ok", "")

        with mock.patch.object(jlink_module.subprocess, "run", fake_run):
            JLinkTransport(device="SF32LB52X", jlink_exe="JLinkExe").flash(build_dir)
        return seen

    def a_build_directory(self) -> str:
        build = self.mkdir("build_x")
        self.write("build_x/download.jlink", DOWNLOAD_JLINK)
        return build

    def test_the_probe_runs_from_the_build_directory(self):
        build = self.a_build_directory()
        self.assertEqual(self.flash_with_a_faked_probe(build)["kwargs"]["cwd"], build)

    def test_the_load_paths_are_left_relative(self):
        # Verbatim, because `cwd` is what makes them resolvable. Rewriting them
        # into absolute paths would be a second statement of where the images
        # are, and it would tie a package to the directory it was built in.
        seen = self.flash_with_a_faked_probe(self.a_build_directory())
        self.assertIn("loadbin ER_IROM1.bin 0x12000000", seen["script"])
        self.assertIn("loadfile main.hex", seen["script"])


class TestPortNormalization(LabTestCase):
    def test_platform_specific_normalization_is_applied(self):
        import platform

        if platform.system() != "Windows":
            self.assertEqual(normalize_uart_port("13"), "13")
            return
        # Accepted forms, as sdk.py flash accepts them.
        self.assertEqual(normalize_uart_port("13"), "COM13")
        self.assertEqual(normalize_uart_port("com13"), "COM13")
        self.assertEqual(normalize_uart_port("COM13"), "COM13")


if __name__ == "__main__":
    unittest.main()
