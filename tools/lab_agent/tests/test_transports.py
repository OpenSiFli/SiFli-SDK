# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the flash transports' pure logic.

Flashing itself needs hardware, but the parts that decide *what* gets programmed
are worth pinning: a wrong address silently bricks a board, and a missing image
must be caught before the probe is invoked rather than during it.

The manifests those parts read are shared with the SDK side that packs them, so
parsing them is tested there (:mod:`sdk_test.tests.test_firmware`); what is left
here is what each transport does with one.
"""

from __future__ import annotations

import json
import os
import subprocess
import unittest
from unittest import mock

from _support import LabTestCase

from lab_agent.errors import TransportError
from lab_agent.transport import jlink as jlink_module
from lab_agent.transport.jlink import JLinkTransport
from lab_agent.transport.sftool import build_flash_arguments
from lab_agent.transport.sftool import normalize_uart_port
from sdk_test.errors import FirmwareError
from sdk_test.firmware import read_sftool_param

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
        with self.assertRaises(FirmwareError) as caught:
            build_flash_arguments(build_dir, read_sftool_param(build_dir))
        self.assertIn("absent.bin", str(caught.exception))

    def test_an_empty_image_list_is_reported(self):
        build_dir = self.make_param(
            {"chip": "SF32LB52", "memory": "NOR", "write_flash": {"files": []}}
        )
        with self.assertRaises(FirmwareError) as caught:
            build_flash_arguments(build_dir, read_sftool_param(build_dir))
        self.assertIn("no flashable images", str(caught.exception))

    def test_a_windows_separator_still_resolves(self):
        # The generator writes `output\main.bin`, and the package preserves that
        # text verbatim. Whatever separator a manifest uses, the lookup has to
        # work wherever the directory actually is.
        build_dir = self.mkdir("build_x")
        self.write("build_x/output/main.bin", "x")
        self.write(
            "build_x/sftool_param.json",
            json.dumps(
                {
                    "chip": "SF32LB52",
                    "memory": "NAND",
                    "write_flash": {
                        "files": [{"path": "output\\main.bin", "address": "0x620A0000"}]
                    },
                }
            ),
        )
        arguments = build_flash_arguments(build_dir, read_sftool_param(build_dir))
        self.assertEqual(len(arguments), 1)
        self.assertIn("main.bin@0x620A0000", arguments[0].replace("\\", "/"))


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


class TestMemoryRead(LabTestCase):
    """The probe's dump is how a suite sees what the firmware produced.

    Only the parts that decide whether the answer is usable are pinned here:
    the CPU is halted and resumed around the read, and a dump shorter than the
    request is an error rather than a smaller answer the caller would read as
    data.
    """

    def read_with_a_faked_probe(self, address: int, length: int, payload=None):
        seen = {}

        def fake_run(command, **kwargs):
            script_path = command[command.index("-CommandFile") + 1]
            # Read it here: the transport deletes the script on return.
            with open(script_path, "r", encoding="ascii") as handle:
                seen["script"] = handle.read()
            seen["kwargs"] = kwargs
            data = payload
            if data is None:
                data = bytes((address + offset) & 0xFF for offset in range(length))
            with open(os.path.join(kwargs["cwd"], "memory.bin"), "wb") as handle:
                handle.write(data)
            return subprocess.CompletedProcess(command, 0, "ok", "")

        with mock.patch.object(jlink_module.subprocess, "run", fake_run):
            data = JLinkTransport(
                device="SF32LB58X_NAND", jlink_exe="JLink.exe"
            ).read_memory(address, length)
        return data, seen

    def test_the_dump_round_trips_verbatim(self):
        data, _ = self.read_with_a_faked_probe(0x2005A7C0, 16)
        self.assertEqual(
            data, bytes((0x2005A7C0 + offset) & 0xFF for offset in range(16))
        )

    def test_the_cpu_is_halted_and_resumed_around_the_read(self):
        _, seen = self.read_with_a_faked_probe(0x20000000, 4)
        lines = seen["script"].splitlines()
        self.assertIn("h", lines)
        self.assertIn("g", lines)
        self.assertLess(lines.index("h"), lines.index("g"))
        self.assertTrue(
            any(line.startswith("savebin memory.bin 0x20000000 0x4") for line in lines)
        )

    def test_the_probe_runs_where_the_dump_lands(self):
        # A relative savebin name only resolves if the probe runs in the
        # directory the transport then reads the file from.
        _, seen = self.read_with_a_faked_probe(0x20000000, 4)
        self.assertTrue(seen["kwargs"]["cwd"])
        self.assertIn("savebin memory.bin ", seen["script"])

    def test_a_short_dump_is_an_error(self):
        with self.assertRaises(TransportError) as caught:
            self.read_with_a_faked_probe(0x20000000, 8, payload=b"abcd")
        self.assertIn("4 of 8", str(caught.exception))

    def test_a_zero_length_read_never_reaches_the_probe(self):
        calls = []

        def fake_run(*args, **kwargs):
            calls.append(kwargs)
            return subprocess.CompletedProcess([], 0, "", "")

        with mock.patch.object(jlink_module.subprocess, "run", fake_run):
            with self.assertRaises(TransportError):
                JLinkTransport(
                    device="SF32LB58X_NAND", jlink_exe="JLink.exe"
                ).read_memory(0x20000000, 0)
        self.assertEqual(calls, [])

    def test_a_relative_probe_path_is_pinned_before_the_temp_cwd(self):
        # A read runs the probe from a temporary directory; a relative
        # executable path would then resolve against that directory and the
        # probe would look missing.
        folder = self.mkdir("segger")
        exe = os.path.join(folder, "JLink.exe")
        with open(exe, "w", encoding="ascii") as handle:
            handle.write("")
        original = os.getcwd()
        os.chdir(self.root)
        self.addCleanup(os.chdir, original)
        transport = JLinkTransport(
            device="SF32LB58X_NAND", jlink_exe=os.path.join("segger", "JLink.exe")
        )
        self.assertTrue(os.path.isabs(transport.jlink_exe), transport.jlink_exe)
        self.assertEqual(
            os.path.normcase(transport.jlink_exe),
            os.path.normcase(os.path.abspath(exe)),
        )

    def test_a_failed_read_resumes_the_core(self):
        # The read happens between `h` and `g`; a failure there would leave the
        # board halted, so the transport must send a resume instead of walking
        # away from a dead console.
        scripts = []

        def fake_run(command, **kwargs):
            script_path = command[command.index("-CommandFile") + 1]
            with open(script_path, "r", encoding="ascii") as handle:
                scripts.append(handle.read())
            if len(scripts) == 1:
                return subprocess.CompletedProcess(command, 1, "savebin failed", "")
            return subprocess.CompletedProcess(command, 0, "ok", "")

        with mock.patch.object(jlink_module.subprocess, "run", fake_run):
            with self.assertRaises(TransportError):
                JLinkTransport(
                    device="SF32LB58X_NAND", jlink_exe="JLink.exe"
                ).read_memory(0x20000000, 8)

        self.assertEqual(len(scripts), 2, scripts)
        self.assertIn("savebin memory.bin 0x20000000 0x8", scripts[0])
        resume = [line.strip() for line in scripts[1].splitlines()]
        self.assertIn("g", resume)
        self.assertNotIn("savebin", scripts[1])


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
