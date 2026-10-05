# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""SDK-side automated test platform for the SiFli SDK.

This package is the hardware-agnostic half of the test framework described in
``SDK自动化测试.docx``: it discovers ``testcase.yaml`` files, checks them against
the platform's own declaration of what it offers, drives the build/flash/run
sequence through a Lab Agent -- which owns the hardware, and allocates a board
when one is asked for -- and reports results.

Nothing in this package knows a real serial port or debugger id; those live in the
binding file consumed by :mod:`lab_agent`.
"""

from __future__ import annotations

__version__ = "0.1.0"

__all__ = ["__version__"]
