# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Shared steps for the CherryUSB examples.

Every example in ``example/cherryusb/`` prints the same banner and exposes the
same ``lsusb`` shell command (``external/cherryusb/core/usbh_core.c``), so the
boot and the host-stack probes belong here rather than copied into each suite.

The strings are taken from the source, not from the READMEs: several of those
are stale (the design document lists them), and a helper that encoded a README
line would turn a documentation error into a test failure.
"""

from __future__ import annotations

from typing import Any
from typing import List

#: The prompt is ``msh />`` when DFS keeps a working directory and ``msh >``
#: when it does not. Accept both rather than pinning one carrier's spelling.
PROMPT = r"msh [^>\n]*>"

#: ``usbh_core.c`` prints this for ``lsusb -V``; the version follows it.
LSUSB_VERSION = r"CherryUSB version \S+"

#: ``lsusb -t`` names the root hub on its first line:
#: ``/:  Bus 01.Port 1: Dev 1, Class=root_hub, Driver=hcd, 12M``.
LSUSB_ROOT_HUB = r"/:\s+Bus \d+\.Port \d+: Dev \d+, Class=root_hub"


def expect_all(dut: Any, *patterns: str, timeout: float = 10.0) -> List[str]:
    """Wait for each pattern in order and return what each matched.

    Host examples print several independent lines; a suite that asserted only
    the last one would pass on a run where the earlier evidence never appeared.
    """
    return [dut.expect(pattern, timeout=timeout) for pattern in patterns]


def lsusb_version(dut: Any, *, timeout: float = 10.0) -> str:
    """Run ``lsusb -V`` on the DUT and return the version line."""
    dut.send_line("lsusb -V")
    return dut.expect(LSUSB_VERSION, timeout=timeout)


def lsusb_root_hub(dut: Any, *, timeout: float = 10.0) -> str:
    """Run ``lsusb -t`` on the DUT and return the root hub line.

    The root hub is the proof that the host controller came up; a host example
    that fails to enumerate still prints its banner, so this line -- not the
    banner -- is what distinguishes "running" from "working".
    """
    dut.send_line("lsusb -t")
    return dut.expect(LSUSB_ROOT_HUB, timeout=timeout)


__all__ = [
    "LSUSB_ROOT_HUB",
    "LSUSB_VERSION",
    "PROMPT",
    "expect_all",
    "lsusb_root_hub",
    "lsusb_version",
]
