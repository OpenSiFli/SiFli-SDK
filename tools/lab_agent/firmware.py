# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Firmware packages uploaded to this agent, and where they were unpacked.

An agent runs for days next to the hardware while the SDK side is rebuilt many
times, so it never flashes "the last thing that arrived": every upload is
unpacked under the digest of its own contents, and a caller has to name that
digest to flash it. A run that forgot to upload therefore gets a refusal rather
than the previous run's firmware -- which would otherwise flash cleanly, boot,
and produce a confident result about the wrong image.

Bounded on purpose. A store that only grows is a lab PC's disk filling up
quietly. Eviction takes the oldest package, which belongs to a run that has
finished: the SDK uploads and then immediately names what it uploaded, so a
package in use is never the oldest one.
"""

from __future__ import annotations

import atexit
import os
import shutil
import tempfile
import threading
from collections import OrderedDict
from typing import Any
from typing import Dict
from typing import Optional

from sdk_test.errors import FirmwareError
from sdk_test.firmware import digest
from sdk_test.firmware import unpack_package

#: How many packages one agent keeps unpacked at a time.
DEFAULT_KEPT = 8


class FirmwareStore:
    """The packages one testbed has been sent, keyed by their own digest.

    One store per testbed rather than one per agent, because that is the thing a
    package is *for*: this bench, this board. It also gives the check that stops
    an image built for one board being programmed into another somewhere to live,
    and it lets releasing the testbed take its packages with it.
    """

    def __init__(
        self,
        root: Optional[str] = None,
        kept: int = DEFAULT_KEPT,
        platform: Optional[str] = None,
    ) -> None:
        if root is None:
            root = tempfile.mkdtemp(prefix="lab_agent_firmware_")
            atexit.register(shutil.rmtree, root, ignore_errors=True)
        self.root = root
        self.kept = kept
        #: The board these packages are for, when the store knows it.
        self.platform = platform
        self._staged: "OrderedDict[str, Dict[str, Any]]" = OrderedDict()
        #: Which staged package a caller last named, so the metadata that came
        #: with it (the device name a probe needs) can be read back.
        self._in_use: Optional[str] = None
        # Uploads and flashes arrive on separate HTTP threads; the dict and the
        # unpacked tree both have to be safe to touch from either.
        self._lock = threading.Lock()

    def stage(self, payload: bytes) -> Dict[str, Any]:
        """Unpack ``payload`` under its own digest; return what the package says it is.

        The digest is computed here rather than accepted from the caller: it is
        the package's name, and a name the sender chooses is a name the sender
        can get wrong.
        """
        token = digest(payload)
        with self._lock:
            if token in self._staged:
                # The same build uploaded twice -- two test cases sharing one
                # firmware, or a re-run. Same bytes, same directory; unpacking
                # again would only rewrite what is already correct.
                self._staged.move_to_end(token)
                metadata = self._staged[token]
            else:
                metadata = unpack_package(payload, os.path.join(self.root, token))
                self._check_platform(metadata)
                self._staged[token] = metadata
                self._evict()
        return {"token": token, "bytes": len(payload), **metadata}

    def path(self, token: str) -> str:
        """Where that package was unpacked.

        Refuses a digest nothing was sent under. This is the whole reason the
        upload and the flash are separate calls: "flash whatever arrived last"
        would make a run that skipped its upload silently test the image from
        the run before it.
        """
        with self._lock:
            if token not in self._staged:
                raise FirmwareError(
                    f'No firmware "{token}" has been uploaded to this testbed. '
                    "Upload it first (POST /testbed/<id>/firmware)."
                )
            self._staged.move_to_end(token)
            self._in_use = token
        return os.path.join(self.root, token)

    def in_use(self) -> Dict[str, Any]:
        """The metadata of the package this testbed is being driven with.

        Which package that is gets resolved by :meth:`path`, at the moment a
        caller names one for a preparation or a flash, so this is that answer
        rather than a guess at recency. Empty when nothing has been resolved --
        which is also what a testbed with no upload yet looks like.
        """
        with self._lock:
            if self._in_use is None:
                return {}
            return dict(self._staged.get(self._in_use) or {})

    def clear(self) -> None:
        """Forget every package. Used when the testbed is handed back."""
        with self._lock:
            for token in list(self._staged):
                shutil.rmtree(os.path.join(self.root, token), ignore_errors=True)
            self._staged.clear()
            self._in_use = None

    def tokens(self) -> list:
        """What is staged, oldest first. For a person looking at a running lab."""
        with self._lock:
            return list(self._staged)

    def _check_platform(self, metadata: Dict[str, Any]) -> None:
        """Refuse a package built for a different board.

        The build knows which board it was for and says so in the package; the
        agent knows which board is on its bench. Nothing else in the run does,
        so nothing else can catch this -- and a wrong-platform image flashes
        perfectly well.
        """
        if not self.platform:
            return
        built_for = str(metadata.get("platform") or "")
        if built_for != self.platform:
            raise FirmwareError(
                f'Package was built for platform "{built_for}", but this testbed '
                f'is "{self.platform}"'
            )

    def _evict(self) -> None:
        while len(self._staged) > self.kept:
            token, _ = self._staged.popitem(last=False)
            shutil.rmtree(os.path.join(self.root, token), ignore_errors=True)
