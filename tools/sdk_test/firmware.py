# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""The firmware package: what a build hands to the agent that flashes it.

Both sides import this. The SDK packs a build directory into one tar and uploads
it; the agent unpacks that tar and flashes from where it landed. It is one
module because a packer and an unpacker that disagree produce a package which
unpacks "successfully" into the wrong thing, and that failure surfaces much
later as a board that will not boot.

What travels is small and nameable. Both flash routes are driven by a manifest
the *build* already wrote into ``build_<board>/`` -- ``download.jlink`` for the
probe, ``sftool_param.json`` for the UART download -- and each names the images
to program and where they go. Nothing here recomputes either. The rest of a
build directory, tens of thousands of object files, stays behind.

A package carries *both* manifests when the build has both, because which one
matters is the lab's decision (``flash.method``) and it is taken after the
package has been sent. They name the same images in practice, so the union of
the two costs nothing.

The image paths inside a manifest are left exactly as the build wrote them, and
the package preserves the relative layout they describe. The agent can therefore
flash from wherever it unpacked: each route resolves its paths against the
working directory, and the transport is run from the unpacked root.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import tarfile
from io import BytesIO
from typing import Any
from typing import Dict
from typing import List
from typing import Tuple

from .errors import FirmwareError

#: Manifests the build writes, in the order they are looked for.
MANIFESTS = ("download.jlink", "sftool_param.json")

#: The package's own manifest, saying what the package is for.
METADATA = "firmware.json"

#: Fields ``METADATA`` may carry. A package naming anything else is refused
#: rather than half-read: an unrecognised field is a promise the sender thinks
#: it is making, and silently dropping it is how the two sides come apart.
METADATA_FIELDS = ("platform", "board", "sdk_commit", "jlink_device")

#: The one field that is required. The rest are either for a person reading a
#: failure or for a lab whose own checkout cannot answer: ``jlink_device`` is
#: here because a probe needs a device name and ``download.jlink`` does not
#: carry one, so without it a lab on another machine has to be trusted to hold
#: the same board file as the machine that built the image.
REQUIRED_FIELDS = ("platform",)

#: Ceilings on what will be packed or accepted. A build directory of images is
#: well under these; anything past them is a mistake or an attack, and either
#: way it is not a firmware package.
MAX_MEMBERS = 64
MAX_TOTAL_BYTES = 256 * 1024 * 1024

#: Lines in a generated download script that actually program the device.
_LOAD_RE = re.compile(r"^\s*(?:loadbin|loadfile)\s+\S", re.IGNORECASE)


# -- the manifests, read identically on both sides ---------------------------


def load_lines_from_download_script(path: str) -> List[str]:
    """Extract the ``loadbin``/``loadfile`` lines from a generated script.

    They are returned verbatim: the paths are relative to the build directory
    and the addresses come from the partition table, so rewriting either here
    would be a second statement of something the build already decided.
    """
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as handle:
            raw = handle.read()
    except OSError as exc:
        raise FirmwareError(f"Cannot read {path}: {exc}") from exc

    lines: List[str] = []
    for line in raw.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith(("//", "#")):
            continue
        if _LOAD_RE.match(stripped):
            # Strip a trailing comment, which JLink would otherwise try to parse.
            lines.append(stripped.split("//")[0].strip())
    return lines


def read_sftool_param(build_dir: str) -> dict:
    """Load and sanity-check ``sftool_param.json`` from a directory of images."""
    path = os.path.join(build_dir, "sftool_param.json")
    if not os.path.isfile(path):
        # Deliberately not "build the project first": by the time this runs, the
        # directory is as likely to be an unpacked package as a build directory,
        # and the build it came from may be on another machine.
        raise FirmwareError(
            f'No sftool_param.json in "{build_dir}": the build produced none, or '
            "it was not packed with the firmware"
        )
    try:
        with open(path, "r", encoding="utf-8") as handle:
            config = json.load(handle)
    except (OSError, json.JSONDecodeError) as exc:
        raise FirmwareError(f"Cannot read {path}: {exc}") from exc

    if not isinstance(config, dict):
        raise FirmwareError(f"{path}: top level must be an object")
    return config


def download_script_images(build_dir: str) -> List[str]:
    """The images a generated ``download.jlink`` names, in the order it names them.

    ``loadbin <path> <address>`` and ``loadfile <path>``; the address belongs to
    the build and is not repeated here.
    """
    images: List[str] = []
    script = os.path.join(build_dir, "download.jlink")
    for line in load_lines_from_download_script(script):
        parts = line.split()
        if len(parts) >= 2:
            images.append(parts[1].strip('"'))
    return images


def sftool_images(config: dict) -> List[str]:
    """The images ``sftool_param.json`` names, in the order it names them."""
    files = (config.get("write_flash") or {}).get("files") or []
    if not isinstance(files, list):
        raise FirmwareError("sftool_param.json: write_flash.files must be a list")
    images: List[str] = []
    for entry in files:
        if isinstance(entry, dict) and entry.get("path"):
            images.append(str(entry["path"]))
    return images


def portable(path: str) -> str:
    """A manifest path as a package member: forward slashes, nothing else changed.

    The manifests are written by a build that may be on Windows (``output\\main.bin``)
    and read by one that may not, so the separator is normalised where a path
    becomes a *lookup* or a *member name* -- and nowhere else. The manifest text
    itself still travels verbatim.
    """
    return path.replace("\\", "/")


# -- packing -----------------------------------------------------------------


def collect(build_dir: str) -> List[str]:
    """Every file of ``build_dir`` that belongs in a package, relative to it.

    The manifests the build wrote, plus every image they name. Raises when there
    is nothing to flash, or when a manifest names an image the directory does not
    hold -- the one failure that must not wait for a board to be on the bench,
    because the symptom there is a timeout that reads like bad firmware.
    """
    members: List[str] = []
    images: List[str] = []

    for name in MANIFESTS:
        if not os.path.isfile(os.path.join(build_dir, name)):
            continue
        members.append(name)
        if name == "download.jlink":
            images += download_script_images(build_dir)
        else:
            images += sftool_images(read_sftool_param(build_dir))

    if not members:
        raise FirmwareError(
            f'"{build_dir}" holds none of {", ".join(MANIFESTS)}; '
            "build the project first"
        )

    missing: List[str] = []
    for image in images:
        relative = portable(image)
        if os.path.isfile(os.path.join(build_dir, relative)):
            if relative not in members:
                members.append(relative)
        else:
            missing.append(image)

    if missing:
        raise FirmwareError(
            f'"{build_dir}" names {len(missing)} image(s) it does not hold: '
            + ", ".join(sorted(missing))
        )
    if not images:
        # A manifest with no images in it programs nothing, and would otherwise
        # be uploaded, unpacked and flashed as a no-op that reports success.
        raise FirmwareError(
            f'"{build_dir}": the manifest names no images; nothing to program'
        )
    return members


def build_package(build_dir: str, metadata: Dict[str, Any]) -> bytes:
    """One tar: the metadata, the manifests, the images.

    Deterministic by construction -- fixed member metadata, no timestamps -- so
    that the same build directory packs to the same bytes and therefore to the
    same digest. That is what lets the digest be the package's name.
    """
    check_metadata(metadata)
    members = collect(build_dir)

    buffer = BytesIO()
    with tarfile.open(fileobj=buffer, mode="w", format=tarfile.USTAR_FORMAT) as tar:
        _add_bytes(tar, METADATA, json.dumps(metadata, indent=2).encode("utf-8"))
        for name in members:
            with open(os.path.join(build_dir, name), "rb") as handle:
                _add_bytes(tar, name, handle.read())
    return buffer.getvalue()


def _add_bytes(tar: tarfile.TarFile, name: str, payload: bytes) -> None:
    info = tarfile.TarInfo(name)
    info.size = len(payload)
    info.mtime = 0
    info.mode = 0o644
    info.uid = info.gid = 0
    info.uname = info.gname = ""
    tar.addfile(info, BytesIO(payload))


def digest(payload: bytes) -> str:
    """The package's name: both sides derive it, neither is told it."""
    return hashlib.sha256(payload).hexdigest()


def check_metadata(metadata: Dict[str, Any]) -> None:
    """Refuse metadata this module does not fully understand."""
    if not isinstance(metadata, dict):
        raise FirmwareError("Package metadata must be an object")
    unknown = sorted(set(metadata) - set(METADATA_FIELDS))
    if unknown:
        raise FirmwareError(
            "Package metadata has unknown field(s): " + ", ".join(unknown)
        )
    for field in REQUIRED_FIELDS:
        if not str(metadata.get(field) or "").strip():
            raise FirmwareError(f'Package metadata needs a "{field}"')


# -- unpacking ---------------------------------------------------------------


def unpack_package(payload: bytes, dest: str) -> Dict[str, Any]:
    """Write a package into ``dest`` and return its metadata.

    Every member is checked before anything is written, and only regular files
    are accepted: a tar can name absolute paths, climb out with ``..``, or carry
    a symlink that the *next* member then writes through. This unpacks packages
    sent by another machine, so none of those may be given the benefit of the
    doubt.
    """
    try:
        tar = tarfile.open(fileobj=BytesIO(payload), mode="r")
    except tarfile.TarError as exc:
        raise FirmwareError(f"Package is not a readable tar: {exc}") from exc

    members = tar.getmembers()
    if len(members) > MAX_MEMBERS:
        raise FirmwareError(
            f"Package holds {len(members)} members; at most {MAX_MEMBERS} are expected"
        )
    total = 0
    for member in members:
        _check_member(member)
        total += member.size
    if total > MAX_TOTAL_BYTES:
        raise FirmwareError(
            f"Package is {total} bytes; at most {MAX_TOTAL_BYTES} are expected"
        )

    metadata_file = tar.extractfile(METADATA) if METADATA in tar.getnames() else None
    if metadata_file is None:
        raise FirmwareError(f"Package has no {METADATA}")
    try:
        metadata = json.loads(metadata_file.read().decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise FirmwareError(f"Package {METADATA} is not valid JSON: {exc}") from exc
    check_metadata(metadata)

    os.makedirs(dest, exist_ok=True)
    for member in members:
        target = os.path.join(dest, *member.name.split("/"))
        parent = os.path.dirname(target)
        if parent:
            os.makedirs(parent, exist_ok=True)
        source = tar.extractfile(member)
        if source is None:
            raise FirmwareError(f"Package member {member.name} cannot be read")
        with open(target, "wb") as handle:
            handle.write(source.read())
    return metadata


def _check_member(member: tarfile.TarInfo) -> None:
    name = member.name
    if not member.isfile():
        raise FirmwareError(
            f"Package member {name} is not a regular file; only files are accepted"
        )
    if name.startswith("/") or os.path.isabs(name) or ":" in name.split("/")[0]:
        raise FirmwareError(f"Package member {name} names an absolute path")
    if "\\" in name:
        raise FirmwareError(f"Package member {name} uses a backslash separator")
    if any(part in ("..", "") for part in name.split("/")):
        raise FirmwareError(f"Package member {name} is not a plain relative path")


def metadata_of(payload: bytes) -> Dict[str, Any]:
    """Read a package's metadata without unpacking it."""
    try:
        with tarfile.open(fileobj=BytesIO(payload), mode="r") as tar:
            handle = tar.extractfile(METADATA)
            if handle is None:
                raise FirmwareError(f"Package has no {METADATA}")
            metadata = json.loads(handle.read().decode("utf-8"))
    except tarfile.TarError as exc:
        raise FirmwareError(f"Package is not a readable tar: {exc}") from exc
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise FirmwareError(f"Package {METADATA} is not valid JSON: {exc}") from exc
    check_metadata(metadata)
    return metadata
