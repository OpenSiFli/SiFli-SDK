# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""The Lab Agent's hardware transports.

Each transport wraps exactly one physical channel and knows nothing about test
semantics: :mod:`~lab_agent.transport.serial` moves bytes, the J-Link transport
resets and flashes, and the relay stub stands in for out-of-band power/mux
control that the hardware team has not delivered yet.
"""

from __future__ import annotations

from .jlink import JLinkTransport
from .relay import RelayTransport
from .serial import SerialTransport
from .sftool import SftoolTransport

__all__ = ["JLinkTransport", "RelayTransport", "SerialTransport", "SftoolTransport"]
