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


class InstrumentCommandError(InstrumentError):
    """The instrument refused a command with a well-formed ``@err``.

    A refusal is an answer, not a malformed one: ``busy`` means retry, ``state``
    means the instrument is not in the state the caller assumed, ``invalid_arg``
    means the request itself was wrong. A suite can branch on :attr:`code`;
    treating every refusal as a protocol error would hide that distinction.
    """

    def __init__(self, op: str, code: str, text: str, sent: str = "") -> None:
        self.op = op
        self.code = code
        self.text = text
        self.sent = sent
        detail = f"{code}: {text}" if text else code
        message = f"the instrument rejected `{op}` with {detail}"
        if sent:
            message += f"\nsent: {sent}"
        super().__init__(message)


class InstrumentCapabilityError(InstrumentError):
    """The instrument's firmware does not provide the requested capability."""
