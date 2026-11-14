# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""The backend used when host-side USB observation is not available.

Two situations land here: a non-Windows agent, and a lab that asked for
``backend: null`` to keep tests hermetic. Both must answer ``capabilities`` with
``supported: false`` -- that is how a suite tells "this bench cannot see USB"
from "this bench cannot be reached" -- and must fail loudly on every other call
rather than returning an empty device list that reads like a real observation.
"""

from __future__ import annotations

import sys
from typing import Any
from typing import Dict
from typing import List

from .base import UsbDevice
from .base import UsbVolume
from .base import host_description
from .errors import UsbHostUnsupported


class UnsupportedUsbHost:
    """A backend that reports itself unsupported and refuses all queries."""

    supported = False

    def __init__(self, reason: str = "") -> None:
        self.reason = reason or (
            f"USB host observation is not implemented on {sys.platform}"
        )
        self.platform_name = sys.platform
        self.features: List[str] = []

    def capabilities(self) -> Dict[str, Any]:
        return {
            "supported": False,
            "platform": self.platform_name,
            "features": [],
            "host": host_description(),
            "reason": self.reason,
        }

    def enumerate(self) -> List[UsbDevice]:
        raise UsbHostUnsupported(self.reason)

    def volumes(self) -> List[UsbVolume]:
        raise UsbHostUnsupported(self.reason)
