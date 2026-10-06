# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for :mod:`sdk_test.firmware`, the package both sides import.

The packer and the unpacker are one module because they must agree exactly; these
tests are what holds them to it, and they lean on the round trip rather than on
either half's idea of the format. What arrives at a lab is a file from another
machine, so the refusals matter as much as the happy path: absolute member names,
``..``, symlinks and a manifest naming an image that is not in the package are
all things a tar can carry and none of them may reach a board.
"""

from __future__ import annotations

import io
import json
import os
import tarfile
import unittest

from _support import SdkTestCase  # noqa: F401  (also fixes sys.path)

from sdk_test import firmware
from sdk_test.errors import FirmwareError

#: Exactly the shape tools/build/sdk_resource.py generates, including the
#: backslashes: the build runs on Windows and the manifest is carried verbatim.
REAL_DOWNLOAD_JLINK = """\
si SWD
speed 10000
r
loadbin bootloader\\output\\bootloader.bin 0x62080000
loadbin output\\main.bin 0x620A0000
loadbin ftab.bin 0x62000000
exit
"""

REAL_SFTOOL_PARAM = {
    "chip": "SF32LB52",
    "memory": "NAND",
    "write_flash": {
        "verify": True,
        "files": [
            {"path": "bootloader\\output\\bootloader.bin", "address": "0x62080000"},
            {"path": "output\\main.bin", "address": "0x620A0000"},
            {"path": "ftab.bin", "address": "0x62000000"},
        ],
    },
}

#: The three images the two manifests above name, with contents that differ so a
#: mix-up between them is visible.
IMAGES = {
    "bootloader/output/bootloader.bin": b"bootloader image",
    "output/main.bin": b"application image",
    "ftab.bin": b"partition table",
}


class BuildDirectory(SdkTestCase):
    """A build directory with both manifests and the images they name."""

    def a_build_dir(self, jlink=True, sftool=True, images=True) -> str:
        build = self.mkdir("build_x")
        if jlink:
            self.write("build_x/download.jlink", REAL_DOWNLOAD_JLINK)
        if sftool:
            self.write(
                "build_x/sftool_param.json", json.dumps(REAL_SFTOOL_PARAM, indent=2)
            )
        if images:
            for name, content in IMAGES.items():
                self.write("build_x/" + name, content.decode())
        return build


# -- the manifests -----------------------------------------------------------


class TestDownloadScriptParsing(BuildDirectory):
    def test_the_load_lines_are_extracted_verbatim(self):
        path = self.write(
            "build_x/download.jlink",
            "loadbin ER_IROM1.bin 0x12000000\nloadfile main.hex\n",
        )
        self.assertEqual(
            firmware.load_lines_from_download_script(path),
            ["loadbin ER_IROM1.bin 0x12000000", "loadfile main.hex"],
        )

    def test_the_probe_preamble_is_not_replayed(self):
        # `si`, `speed`, `r` and `exit` are replaced by the transport's own
        # preamble so a probe serial number can be selected; replaying the
        # build's `r` would reset the board mid-programming.
        path = self.write("build_x/download.jlink", REAL_DOWNLOAD_JLINK)
        loads = firmware.load_lines_from_download_script(path)
        for noise in ("si SWD", "speed 10000", "exit"):
            self.assertNotIn(noise, loads)

    def test_trailing_comments_are_stripped(self):
        path = self.write(
            "build_x/download.jlink", "loadbin a.bin 0x1000 // the boot image\n"
        )
        self.assertEqual(
            firmware.load_lines_from_download_script(path), ["loadbin a.bin 0x1000"]
        )

    def test_a_script_with_no_loads_yields_nothing(self):
        path = self.write("build_x/download.jlink", "si SWD\nr\nexit\n")
        self.assertEqual(firmware.load_lines_from_download_script(path), [])

    def test_a_missing_script_is_reported(self):
        with self.assertRaises(FirmwareError):
            firmware.load_lines_from_download_script(self.root + "/absent.jlink")

    def test_the_images_named_by_a_real_script_are_read_back(self):
        build = self.a_build_dir(sftool=False)
        self.assertEqual(
            firmware.download_script_images(build),
            [
                "bootloader\\output\\bootloader.bin",
                "output\\main.bin",
                "ftab.bin",
            ],
        )


class TestSftoolParam(BuildDirectory):
    def test_a_missing_param_file_says_what_is_missing(self):
        # Not "build the project first": by the time a transport reads this, the
        # directory is as likely to be an unpacked package as a build directory,
        # and the build it came from may be on another machine.
        with self.assertRaises(FirmwareError) as caught:
            firmware.read_sftool_param(self.root)
        self.assertIn("sftool_param.json", str(caught.exception))
        self.assertIn("not packed", str(caught.exception))

    def test_a_malformed_param_file_is_reported(self):
        self.write("build_x/sftool_param.json", "{not json")
        with self.assertRaises(FirmwareError):
            firmware.read_sftool_param(self.mkdir("build_x"))

    def test_the_images_named_by_a_real_param_file_are_read_back(self):
        build = self.a_build_dir(jlink=False)
        self.assertEqual(
            firmware.sftool_images(firmware.read_sftool_param(build)),
            [
                "bootloader\\output\\bootloader.bin",
                "output\\main.bin",
                "ftab.bin",
            ],
        )


# -- packing -----------------------------------------------------------------


class TestCollect(BuildDirectory):
    def test_both_manifests_and_the_union_of_their_images_are_carried(self):
        # Which manifest matters is the lab's decision (`flash.method`), taken
        # after the package has been sent -- so both travel. Here they name the
        # same three images, so the union is three.
        members = firmware.collect(self.a_build_dir())
        self.assertEqual(
            sorted(members),
            [
                "bootloader/output/bootloader.bin",
                "download.jlink",
                "ftab.bin",
                "output/main.bin",
                "sftool_param.json",
            ],
        )

    def test_a_windows_separated_image_is_found_and_stored_portably(self):
        # The manifest says `output\main.bin`; the member is `output/main.bin`.
        # The package must not depend on which separator the build happened to
        # use, or a Linux SDK could not pack a Windows build's directory.
        members = firmware.collect(self.a_build_dir())
        self.assertIn("output/main.bin", members)
        self.assertNotIn("output\\main.bin", members)

    def test_a_manifest_naming_an_image_that_is_not_there_is_refused_here(self):
        # The one failure that must not wait for a board: on the bench it shows
        # up as a timeout, which reads like bad firmware.
        build = self.a_build_dir()
        os.unlink(os.path.join(build, "output", "main.bin"))

        with self.assertRaises(FirmwareError) as caught:
            firmware.collect(build)
        self.assertIn("does not hold", str(caught.exception))
        self.assertIn("main.bin", str(caught.exception))

    def test_a_manifest_naming_no_images_is_refused(self):
        build = self.a_build_dir(images=False)
        self.write("build_x/download.jlink", "si SWD\nr\nexit\n")
        self.write(
            "build_x/sftool_param.json",
            json.dumps({"chip": "X", "write_flash": {"files": []}}),
        )
        with self.assertRaises(FirmwareError) as caught:
            firmware.collect(build)
        self.assertIn("nothing to program", str(caught.exception))

    def test_a_directory_with_neither_manifest_is_refused(self):
        with self.assertRaises(FirmwareError) as caught:
            firmware.collect(self.a_build_dir(jlink=False, sftool=False))
        self.assertIn("build the project first", str(caught.exception))

    def test_one_manifest_alone_is_enough(self):
        # A build that only ever wrote one of them still packs.
        members = firmware.collect(self.a_build_dir(sftool=False))
        self.assertIn("download.jlink", members)
        self.assertNotIn("sftool_param.json", members)


class TestBuildPackage(BuildDirectory):
    def metadata(self, **overrides) -> dict:
        values = {"platform": "sf32lb52-lcd_a128r16", "board": "sf32lb52-lcd_a128r16_hcpu"}
        values.update(overrides)
        return values

    def test_the_metadata_and_every_member_travel(self):
        payload = firmware.build_package(self.a_build_dir(), self.metadata())
        with tarfile.open(fileobj=io.BytesIO(payload)) as tar:
            names = sorted(tar.getnames())
        self.assertEqual(
            names,
            [
                "bootloader/output/bootloader.bin",
                "download.jlink",
                "firmware.json",
                "ftab.bin",
                "output/main.bin",
                "sftool_param.json",
            ],
        )

    def test_an_unknown_metadata_field_is_refused(self):
        # A field nothing reads is a promise the sender thinks it is making.
        # Dropping it silently is how the two sides come apart.
        with self.assertRaises(FirmwareError) as caught:
            firmware.build_package(
                self.a_build_dir(), self.metadata(jlink_serial="59605049")
            )
        self.assertIn("jlink_serial", str(caught.exception))

    def test_the_probe_device_is_a_known_field(self):
        # A probe needs a device name and download.jlink does not carry one, so
        # the package does -- otherwise a lab on another machine is trusted to
        # hold the same board file as the machine that built the image.
        payload = firmware.build_package(
            self.a_build_dir(), self.metadata(jlink_device="SF32LB52X_NAND")
        )
        self.assertEqual(
            firmware.metadata_of(payload)["jlink_device"], "SF32LB52X_NAND"
        )

    def test_a_missing_platform_is_refused(self):
        with self.assertRaises(FirmwareError) as caught:
            firmware.build_package(self.a_build_dir(), {"board": "x"})
        self.assertIn("platform", str(caught.exception))

    def test_the_same_directory_packs_to_the_same_bytes(self):
        # Which is what lets the digest be the package's name: two packings of
        # one build are one package, and the agent stages it once.
        build = self.a_build_dir()
        first = firmware.build_package(build, self.metadata())
        second = firmware.build_package(build, self.metadata())
        self.assertEqual(first, second)
        self.assertEqual(firmware.digest(first), firmware.digest(second))

    def test_a_changed_image_changes_the_digest(self):
        build = self.a_build_dir()
        before = firmware.digest(firmware.build_package(build, self.metadata()))
        self.write("build_x/output/main.bin", "a different build")
        after = firmware.digest(firmware.build_package(build, self.metadata()))
        self.assertNotEqual(before, after)


# -- unpacking ---------------------------------------------------------------


class TestRoundTrip(BuildDirectory):
    def metadata(self) -> dict:
        return {
            "platform": "sf32lb52-lcd_a128r16",
            "board": "sf32lb52-lcd_a128r16_hcpu",
            "sdk_commit": "e83337433",
        }

    def test_what_was_packed_is_what_lands_on_disk(self):
        payload = firmware.build_package(self.a_build_dir(), self.metadata())
        dest = os.path.join(self.root, "unpacked")
        metadata = firmware.unpack_package(payload, dest)

        self.assertEqual(metadata, self.metadata())
        for name, content in IMAGES.items():
            with open(os.path.join(dest, *name.split("/")), "rb") as handle:
                self.assertEqual(handle.read(), content)

    def test_the_manifest_still_reads_the_same_after_the_trip(self):
        # The agent flashes from the unpacked directory and lets the manifest's
        # own relative paths find the images there. Verbatim is the contract.
        payload = firmware.build_package(self.a_build_dir(), self.metadata())
        dest = os.path.join(self.root, "unpacked")
        firmware.unpack_package(payload, dest)

        self.assertEqual(
            firmware.download_script_images(dest),
            ["bootloader\\output\\bootloader.bin", "output\\main.bin", "ftab.bin"],
        )
        for image in firmware.sftool_images(firmware.read_sftool_param(dest)):
            self.assertTrue(os.path.isfile(os.path.join(dest, firmware.portable(image))))

    def test_metadata_can_be_read_without_unpacking(self):
        payload = firmware.build_package(self.a_build_dir(), self.metadata())
        self.assertEqual(firmware.metadata_of(payload), self.metadata())


class TestUnpackRefusals(SdkTestCase):
    """A package comes from another machine, so none of these get the benefit
    of the doubt -- and every one of them is something a tar can carry."""

    def package(self, members, metadata=None) -> bytes:
        payload = io.BytesIO()
        with tarfile.open(fileobj=payload, mode="w") as tar:
            if metadata is not None:
                self.add_to(tar, firmware.METADATA, json.dumps(metadata).encode())
            for name, content in members:
                self.add_to(tar, name, content)
        return payload.getvalue()

    def add_to(self, tar, name, content, kind=tarfile.REGTYPE) -> None:
        info = tarfile.TarInfo(name)
        if kind == tarfile.REGTYPE:
            info.size = len(content)
            tar.addfile(info, io.BytesIO(content))
        else:
            info.type = kind
            tar.addfile(info)

    def unpack(self, payload):
        return firmware.unpack_package(payload, os.path.join(self.root, "out"))

    def test_a_well_formed_package_unpacks(self):
        payload = self.package([("main.bin", b"x")], {"platform": "p"})
        self.assertEqual(self.unpack(payload), {"platform": "p"})

    def test_an_absolute_member_name_is_refused(self):
        payload = self.package(
            [("/etc/passwd", b"x")], {"platform": "p"}
        )
        with self.assertRaises(FirmwareError) as caught:
            self.unpack(payload)
        self.assertIn("absolute", str(caught.exception))

    def test_a_drive_letter_is_refused(self):
        payload = self.package([("C:/windows/system32/x.dll", b"x")], {"platform": "p"})
        with self.assertRaises(FirmwareError) as caught:
            self.unpack(payload)
        self.assertIn("absolute", str(caught.exception))

    def test_climbing_out_with_dot_dot_is_refused(self):
        payload = self.package([("../escape.bin", b"x")], {"platform": "p"})
        with self.assertRaises(FirmwareError) as caught:
            self.unpack(payload)
        self.assertIn("relative path", str(caught.exception))

    def test_a_backslash_separator_is_refused(self):
        # Tar's own convention is `/`. A backslash here means the packer and
        # this side disagree about the format, and guessing which is which is
        # how a member lands in a directory nobody meant.
        payload = self.package([("output\\main.bin", b"x")], {"platform": "p"})
        with self.assertRaises(FirmwareError) as caught:
            self.unpack(payload)
        self.assertIn("backslash", str(caught.exception))

    def test_a_symlink_is_refused(self):
        payload = io.BytesIO()
        with tarfile.open(fileobj=payload, mode="w") as tar:
            self.add_to(tar, firmware.METADATA, json.dumps({"platform": "p"}).encode())
            self.add_to(tar, "link", b"", kind=tarfile.SYMTYPE)
        with self.assertRaises(FirmwareError) as caught:
            self.unpack(payload.getvalue())
        self.assertIn("regular file", str(caught.exception))

    def test_a_package_with_no_metadata_is_refused(self):
        payload = self.package([("main.bin", b"x")])
        with self.assertRaises(FirmwareError) as caught:
            self.unpack(payload)
        self.assertIn(firmware.METADATA, str(caught.exception))

    def test_metadata_without_a_platform_is_refused(self):
        with self.assertRaises(FirmwareError) as caught:
            self.unpack(self.package([], {"board": "x"}))
        self.assertIn("platform", str(caught.exception))

    def test_something_that_is_not_a_tar_is_refused(self):
        with self.assertRaises(FirmwareError) as caught:
            self.unpack(b"this is not a tar at all")
        self.assertIn("not a readable tar", str(caught.exception))

    def test_too_many_members_is_refused(self):
        members = [(f"image{index}.bin", b"x") for index in range(firmware.MAX_MEMBERS + 1)]
        with self.assertRaises(FirmwareError) as caught:
            self.unpack(self.package(members, {"platform": "p"}))
        self.assertIn("members", str(caught.exception))

    def test_nothing_is_written_when_a_later_member_is_refused(self):
        # Checked before anything lands, so a refused package cannot leave half
        # of itself behind for the next flash to find.
        payload = self.package(
            [("main.bin", b"x"), ("/absolute", b"y")], {"platform": "p"}
        )
        dest = os.path.join(self.root, "out")
        with self.assertRaises(FirmwareError):
            firmware.unpack_package(payload, dest)
        self.assertFalse(os.path.exists(os.path.join(dest, "main.bin")))


if __name__ == "__main__":
    unittest.main()
