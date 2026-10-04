# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Lab Agent: the machine-side half of the SDK test framework.

This package runs *next to the hardware*. It owns the things a test case is
never allowed to know -- which serial port, which debug probe, which mux channel
-- and hands out one testbed at a time over a small local HTTP API. The SDK-side
half lives in ``tools/sdk_test`` and talks to this through
``sdk_test.agent_client``.

The dividing line is deliberate: the SDK side decides *what* to run and *where it
may* run, from the testcase and the board's own feature declaration; the agent
owns *what is physically there* and which of it is free. See
``tools/sdk_test/DESIGN.md``.
"""

from __future__ import annotations

__version__ = "0.1.0"

__all__ = ["__version__"]
