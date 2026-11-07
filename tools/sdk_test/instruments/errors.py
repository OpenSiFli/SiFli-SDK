# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Errors raised while driving an instrument.

These derive from :class:`sdk_test.errors.SdkTestError`, so callers that already
catch the framework's base class see them too. They are separate names because a
test author needs to tell "the instrument did not answer" (a lab/firmware
problem) apart from "the instrument answered with something this protocol cannot
represent" (a parser or firmware bug).
"""

from __future__ import annotations

from ..errors import SdkTestError


class InstrumentError(SdkTestError):
    """Base class for instrument failures."""


class InstrumentTimeout(InstrumentError):
    """The instrument did not produce the expected line in time."""


class InstrumentProtocolError(InstrumentError):
    """The instrument answered, but not in the shape the protocol defines."""


class InstrumentCapabilityError(InstrumentError):
    """The instrument's firmware does not provide the requested capability."""
