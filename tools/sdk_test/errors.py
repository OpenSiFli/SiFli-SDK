# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Exceptions raised by the SDK-side test platform.

These are deliberately independent of :mod:`sdk_py_actions.errors` so that
``sdk_test`` can be imported and unit-tested without the ``sdk.py`` machinery.
``tools/sdk_py_actions/test_ext.py`` maps them onto the CLI error types.
"""

from __future__ import annotations


class SdkTestError(RuntimeError):
    """Base class for every error raised by this package."""


class DiscoveryError(SdkTestError):
    """A ``testcase.yaml`` is missing, malformed, or uses unknown fields."""


class BindingError(SdkTestError):
    """A binding/hardware-map file is missing, malformed, or inconsistent."""


class MatchError(SdkTestError):
    """No testbed can satisfy a test instance."""


class PlatformError(SdkTestError):
    """A platform's feature declaration is missing, malformed, or inconsistent."""


class BackendError(SdkTestError):
    """The selected execution backend cannot run the instance."""


class HarnessError(SdkTestError):
    """A harness is unknown, misconfigured, or reported an unusable result."""


class BuildError(SdkTestError):
    """A build failed.

    Carries what the compiler said, because the exit code does not: ``2`` reads
    the same whether a header is missing or a type is wrong, and the answer is
    always in the output. The runner files that output as ``build.log`` on the
    way past, which is what lets the message stay short.
    """

    def __init__(self, message: str, output: str = "") -> None:
        super().__init__(message)
        self.output = output


class FirmwareError(SdkTestError):
    """A firmware package is missing, malformed, or names a file it does not carry.

    Raised by :mod:`sdk_test.firmware`, which both sides import: the SDK packing
    a build directory and the agent unpacking what arrived. It is therefore the
    *sender's* kind of failure on the agent, and a 400 rather than a 5xx -- the
    lab is fine, the package is not.
    """


class LabAgentError(SdkTestError):
    """The Lab Agent runner is unreachable or refused an operation."""
