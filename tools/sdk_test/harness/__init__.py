# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Harness registry and dispatch.

A *harness* is the rule that decides whether a test scenario passed. The name
says nothing about which C framework is used — that is the point the design doc
makes when it reuses Twister's ``unity`` label for a device-side unit test whose
verdict the host reads off the console.

===========================  ==========================================
harness                      how the verdict is reached
===========================  ==========================================
``console``                  no device-side test code; the host matches
                             regular expressions against console output
``unity``                    a Unity test binary runs on the device and the
                             host parses Unity's own summary
``pytest``                   the test directory ships its own pytest suite,
                             which drives the DUT through the ``dut`` fixture
``shell``                    the host drives the device's msh/finsh shell
===========================  ==========================================

``shell`` is the one still unimplemented; it raises rather than silently doing
nothing, so a testcase cannot pass by virtue of never being checked.

Every harness driver takes the same three things -- the test instance, a console
and a timeout -- and returns a :class:`HarnessOutcome`. Because the console is
duck-typed, the same driver runs against a local serial port or one reached over
the Lab Agent's HTTP API.
"""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field
from typing import Any
from typing import List
from typing import Optional
from typing import Protocol
from typing import runtime_checkable

from ..errors import HarnessError
from ..model import TestInstance

SUPPORTED_HARNESSES = ("console", "unity", "shell", "pytest")
IMPLEMENTED_HARNESSES = ("console", "unity", "pytest")

#: How long a single console read waits before the driver loops again.
DEFAULT_POLL = 0.5


@runtime_checkable
class ConsoleLike(Protocol):
    """What a harness needs from a console.

    Deliberately tiny: anything with these two methods works, whether it is a
    local :class:`~lab_agent.transport.serial.SerialTransport` wrapper or a
    :class:`~sdk_test.agent_client.RemoteConsole` on the far side of HTTP.
    """

    def read(self, timeout: float) -> str:
        ...

    def write(self, data: str) -> None:
        ...


@dataclass
class RunContext:
    """Everything a harness might need beyond the console.

    Only the ``pytest`` harness uses any of it: it spawns a subprocess that has
    to find its way back to the same already-allocated testbed, so it needs
    coordinates the other harnesses never look at.
    """

    python_exe: Optional[str] = None
    agent_url: Optional[str] = None
    testbed_id: Optional[str] = None
    build_dir: Optional[str] = None
    platform: Optional[str] = None


@dataclass
class HarnessOutcome:
    """The verdict of one harness run, plus enough evidence to trust it."""

    passed: bool
    detail: str
    #: Everything the device said, kept even on success: it is what a human
    #: needs when the test later turns out to be flaky.
    log: str = ""
    #: Per-case detail where the harness has any (Unity does, console does not).
    cases: List[Any] = field(default_factory=list)


def ensure_supported(name: str) -> str:
    """Validate a harness name coming from a testcase file."""
    if name not in SUPPORTED_HARNESSES:
        raise HarnessError(
            f'Unknown harness "{name}"; supported: {", ".join(SUPPORTED_HARNESSES)}'
        )
    return name


def ensure_implemented(name: str) -> str:
    """Validate that this milestone can actually run the harness."""
    ensure_supported(name)
    if name not in IMPLEMENTED_HARNESSES:
        raise HarnessError(
            f'Harness "{name}" is specified but not implemented yet; '
            f'implemented: {", ".join(IMPLEMENTED_HARNESSES)}'
        )
    return name


def implemented_harnesses() -> List[str]:
    return list(IMPLEMENTED_HARNESSES)


#: ``harness_config`` keys only ``unity`` reads. A harness that never looks at
#: one must refuse it rather than ignore it: accepting a key that does nothing
#: teaches that writing it did something. ``unity`` already applies this rule to
#: ``record`` from the other side; this is the same rule, same direction.
UNITY_ONLY_KEYS = ("autorun",)


def reject_foreign_keys(name: str, harness_config: dict) -> None:
    """Refuse ``harness_config`` keys that belong to another harness."""
    for key in UNITY_ONLY_KEYS:
        if key in harness_config:
            raise HarnessError(
                f'harness "{name}" does not read harness_config.{key}; it '
                f'belongs to harness "unity"'
            )


def run(
    instance: TestInstance,
    console: ConsoleLike,
    timeout: float,
    poll: float = DEFAULT_POLL,
    context: Optional[RunContext] = None,
) -> HarnessOutcome:
    """Decide whether ``instance`` passed, reading the device through ``console``.

    Dispatch happens here rather than in the runner so that adding a harness is
    a single new module plus one line, and so every harness is driven through an
    identical code path (which is what makes them comparable).
    """
    harness = ensure_implemented(instance.harness)

    if harness == "console":
        from . import console as console_harness

        return console_harness.run(instance, console, timeout, poll)

    if harness == "unity":
        from . import unity as unity_harness

        return unity_harness.run(instance, console, timeout, poll)

    if harness == "pytest":
        from . import pytest as pytest_harness

        return pytest_harness.run(instance, console, timeout, poll, context)

    raise HarnessError(f'Harness "{harness}" has no driver')  # pragma: no cover
