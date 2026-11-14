# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Read-only access to a USB volume, with the path rules spelled out.

This is the one place in the capability that touches a filesystem the device
controls, so every rule is enforced here rather than trusted to the caller:

* only relative paths, with ``/`` as the separator -- a backslash, a colon, a
  drive letter or a leading slash is refused outright;
* no ``..`` (and no ``.``) segment, so the path cannot climb out;
* after ``realpath`` the file must still be under the volume root, which is
  what stops a junction or symlink from redirecting the read;
* every existing component is checked for the reparse-point attribute, so a
  link *inside* the volume is refused too, not just one that escapes;
* the size is checked before reading and the read is bounded, so a device that
  lies about its file size cannot make the agent allocate an arbitrary buffer.

Nothing here writes, and no caller above it can: there is no write function.
"""

from __future__ import annotations

import os
import re
from typing import List
from typing import Tuple

from .base import UsbVolume
from .errors import UsbHostFileNotFound
from .errors import UsbHostFileTooLarge
from .errors import UsbHostPathDenied
from .errors import UsbHostVolumeNotMounted

#: Windows refuses paths longer than this; refusing early keeps the error
#: attached to the path instead of to an ``open`` that failed for another reason.
MAX_PATH_LENGTH = 260

#: ``FILE_ATTRIBUTE_REPARSE_POINT``; ``os.lstat`` exposes it on Windows.
FILE_ATTRIBUTE_REPARSE_POINT = 0x400

#: Names Windows still interprets as devices even with an extension.
_RESERVED_NAMES = frozenset(
    {"CON", "PRN", "AUX", "NUL"}
    | {f"COM{index}" for index in range(1, 10)}
    | {f"LPT{index}" for index in range(1, 10)}
)

#: Characters Windows refuses in a file name, plus the separator we ban.
_INVALID_CHARACTERS = set('<>:"|?*\\')

_DRIVE_LETTER = re.compile(r"^[A-Za-z]:$")


def volume_root(volume: UsbVolume) -> str:
    """The absolute path that bounds every read on this volume.

    A Windows volume is named by its drive letter (``E:``); the self-tests and
    a non-Windows backend use an ordinary directory instead, so both spellings
    are accepted without a second code path.
    """
    drive = volume.drive or ""
    if _DRIVE_LETTER.match(drive):
        return drive + os.sep
    return os.path.abspath(drive)


def _deny(path: str, why: str) -> UsbHostPathDenied:
    return UsbHostPathDenied(
        f'path "{path}" is not allowed: {why}', details={"path": path}
    )


def validate_relative_path(path: str) -> Tuple[str, ...]:
    """Check a caller-supplied path and return its segments."""
    if not isinstance(path, str) or not path:
        raise _deny(str(path), "the path is empty")
    if len(path) > MAX_PATH_LENGTH:
        raise _deny(path, f"longer than {MAX_PATH_LENGTH} characters")
    if "\\" in path:
        raise _deny(path, 'use "/" as the separator')
    if path.startswith("/"):
        raise _deny(path, "absolute paths are refused")
    if ":" in path:
        raise _deny(path, "drive letters and alternate data streams are refused")
    if _DRIVE_LETTER.match(path):
        raise _deny(path, "drive letters are refused")
    bad = sorted(
        character
        for character in set(path)
        if character in _INVALID_CHARACTERS or ord(character) < 0x20
    )
    if bad:
        raise _deny(path, f"contains {bad!r}")

    segments: List[str] = []
    for segment in path.split("/"):
        if segment in ("", ".", ".."):
            raise _deny(path, f'segment {segment!r} is refused')
        stem = segment.rsplit(".", 1)[0].upper()
        if stem in _RESERVED_NAMES:
            raise _deny(path, f'"{segment}" is a reserved device name')
        segments.append(segment)
    if not segments:
        raise _deny(path, "the path names no file")
    return tuple(segments)


def _reject_reparse(path: str) -> None:
    """Refuse a component that is a link, junction or other reparse point."""
    try:
        stat = os.lstat(path)
    except OSError:
        return
    attributes = getattr(stat, "st_file_attributes", 0) or 0
    if attributes & FILE_ATTRIBUTE_REPARSE_POINT:
        raise _deny(path, "a reparse point (link or junction) is refused")


def _is_under(root: str, candidate: str) -> bool:
    try:
        common = os.path.commonpath([root, candidate])
    except ValueError:
        return False
    return os.path.normcase(common) == os.path.normcase(root)


def read_volume_file(
    volume: UsbVolume,
    path: str,
    *,
    max_bytes: int,
    allow_fixed_drives: bool = False,
) -> bytes:
    """Read one file from ``volume`` without leaving it or exceeding a bound."""
    if not volume.removable and not allow_fixed_drives:
        raise UsbHostVolumeNotMounted(
            f'{volume.drive}: a fixed drive is not a USB volume '
            '(set "allow_fixed_drives" to read it anyway)'
        )

    segments = validate_relative_path(path)
    root = os.path.realpath(volume_root(volume))
    if not os.path.isdir(root):
        raise UsbHostVolumeNotMounted(f"{volume.drive}: the volume is not mounted")

    current = root
    for segment in segments:
        current = os.path.join(current, segment)
        _reject_reparse(current)

    candidate = os.path.realpath(current)
    if not _is_under(root, candidate):
        raise _deny(path, "the resolved path leaves the volume")

    try:
        stat = os.stat(candidate)
    except OSError as exc:
        raise UsbHostFileNotFound(
            f'"{path}" does not exist on {volume.drive}: {exc}'
        ) from exc
    if not os.path.isfile(candidate):
        raise UsbHostFileNotFound(f'"{path}" is not a regular file')

    size = stat.st_size
    if size > max_bytes:
        raise UsbHostFileTooLarge(
            f'"{path}" is {size} bytes; at most {max_bytes} are read',
            details={"size": size, "max_file_bytes": max_bytes},
        )

    try:
        with open(candidate, "rb") as handle:
            data = handle.read(max_bytes + 1)
    except OSError as exc:
        raise UsbHostFileNotFound(
            f'cannot read "{path}" on {volume.drive}: {exc}'
        ) from exc
    if len(data) > max_bytes:
        # The file grew between stat() and read(). The bound still holds.
        raise UsbHostFileTooLarge(
            f'"{path}" is larger than {max_bytes} bytes',
            details={"max_file_bytes": max_bytes},
        )
    return data
