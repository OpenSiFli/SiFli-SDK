# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Exceptions raised by the Lab Agent.

The Lab Agent is a separate process from the SDK-side framework, so it has its
own error family rather than reusing :mod:`sdk_test.errors`. What the two share
is the binding-file model, which is imported rather than copied.
"""

from __future__ import annotations


class LabAgentError(RuntimeError):
    """Base class for every error raised by the Lab Agent."""


class ConfigError(LabAgentError):
    """The runner configuration is missing, malformed, or inconsistent."""


class AllocationError(LabAgentError):
    """No testbed can serve a request, or the requested one is already taken."""


class TestbedError(LabAgentError):
    """A testbed was asked to do something its current state does not allow."""


class TransportError(LabAgentError):
    """A serial port, debug probe or relay failed."""


class RelayNotAvailable(LabAgentError):
    """Out-of-band power/mux control was requested but is not wired up.

    Raised rather than silently succeeding: the design requires reset and power
    switching to be genuinely out-of-band, and a silent no-op here would make a
    test look like it passed without the device ever being power-cycled.
    """
