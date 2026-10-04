# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Execution backends: what a test actually runs on.

The design's tier split lives here.

*Tier A* -- :mod:`~sdk_test.backends.board` -- a real board, reached through a
Lab Agent. Hardware must be allocated, flashed and reset before the test starts.

*Tier B* -- :mod:`~sdk_test.backends.pc_sim` and :mod:`~sdk_test.backends.qemu`
-- a non-board carrier, where ``drivers/hal`` is not compiled at all. These are
designed and stubbed, not implemented; the important property they already have
is that they need **no** Lab Agent, because there is no hardware to allocate and
therefore no testbed to lock.

Choosing between them is a property of the platform name, which is why the same
`scons --board=<name>` covers all three: ``pc_hcpu``, ``qemu_cortex_m33`` and a
real board are all just board names to the build system.
"""

from __future__ import annotations

from typing import List
from typing import Optional

from ..errors import BackendError
from .base import Backend
from .base import Session
from .board import BoardBackend
from .pc_sim import PcSimBackend
from .qemu import QemuBackend

#: Registry of backend name -> implementation.
BACKENDS = {
    BoardBackend.name: BoardBackend,
    PcSimBackend.name: PcSimBackend,
    QemuBackend.name: QemuBackend,
}

#: Board-name prefixes that mean "no hardware": these are host-side carriers.
#: ``customer/boards/pc`` is the simulator; ``qemu_*`` targets QEMU.
HOST_PLATFORM_PREFIXES = ("qemu",)


def select_backend_name(platform: str) -> str:
    """Pick the backend a platform implies.

    Only the *name* is decided here; installing a Lab Agent client is the
    caller's job, so a host platform never needs one to exist.
    """
    name = (platform or "").strip()
    if not name:
        raise BackendError(
            "Cannot choose a backend without a platform; pass --platform/--board."
        )

    for prefix in HOST_PLATFORM_PREFIXES:
        if name.startswith(prefix):
            return QemuBackend.name

    if name == "pc" or name.startswith("pc_"):
        return PcSimBackend.name

    return BoardBackend.name


def is_host_platform(platform: str) -> bool:
    """True when the platform runs on this machine rather than on a board."""
    return select_backend_name(platform) != BoardBackend.name


def create_backend(
    name: str,
    client: Optional[object] = None,
) -> Backend:
    """Instantiate a backend by name."""
    implementation = BACKENDS.get(name)
    if implementation is None:
        raise BackendError(
            f'Unknown backend "{name}"; known: {", ".join(sorted(BACKENDS))}'
        )
    if implementation.requires_lab_agent:
        if client is None:
            raise BackendError(
                f'Backend "{name}" needs a Lab Agent client, but none was supplied'
            )
        return implementation(client)
    return implementation()


def backend_names() -> List[str]:
    return sorted(BACKENDS)


__all__ = [
    "Backend",
    "BoardBackend",
    "PcSimBackend",
    "QemuBackend",
    "Session",
    "BACKENDS",
    "backend_names",
    "create_backend",
    "is_host_platform",
    "select_backend_name",
]
