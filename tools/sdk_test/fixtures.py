# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Fixture parsing and testbed matching.

A *fixture* is a capability, not a physical object: several testbeds may offer
the same fixture id and are then interchangeable (the hardware team guarantees
they are exactly equivalent). A test therefore never names a board or a port,
only the capabilities it needs.

This is the second of the two questions a test plan asks -- *which* board of the
platform, wired how -- and it is the lab's question, answered by the Lab Agent
from its own binding at the moment hardware is requested. The first question,
whether the platform can host the test at all, is answered from the board's own
declaration and lives in :mod:`sdk_test.platforms`; nothing here consults it, and
nothing there consults a binding.

One consequence is worth stating: a binding file says what is wired to a board,
and the board says what it has. Where the two contradict each other --
:func:`wiring_problems` -- one of the files is wrong, and that is reported rather
than left to surface as a test that mysteriously will not schedule.
"""

from __future__ import annotations

from typing import Any
from typing import Iterable
from typing import List
from typing import Optional

from .errors import BindingError
from .errors import DiscoveryError
from .model import FixtureRef
from .model import TestInstance
from .model import TestBed
from .platforms import type_of
from .platforms import unsatisfied


def parse_fixture_ref(value: Any) -> FixtureRef:
    """Parse one fixture reference.

    Accepted shapes::

        "spi_nor_flash"                 -> name only, any wiring
        "spi_nor_flash:spi2"            -> pinned to the spi2 instance
        {name: spi_nor_flash, param: spi2}
        {fixture: "spi_nor_flash:spi2"} -> binding-file connection form
    """
    if isinstance(value, FixtureRef):
        return value

    if isinstance(value, str):
        spec = value.strip()
        if not spec:
            raise BindingError("Empty fixture reference")
        name, _, param = spec.partition(":")
        name = name.strip()
        param = param.strip() or None
        if not name:
            raise BindingError(f'Fixture reference "{value}" has no name')
        return FixtureRef(name=name, param=param)

    if isinstance(value, dict):
        if "fixture" in value:
            ref = parse_fixture_ref(value["fixture"])
            if "param" in value and value["param"] is not None:
                ref = FixtureRef(ref.name, str(value["param"]))
            return ref
        name = value.get("name")
        if not name:
            raise BindingError(f"Fixture mapping needs a name: {value!r}")
        param = value.get("param")
        return FixtureRef(str(name), str(param) if param is not None else None)

    raise BindingError(f"Unsupported fixture reference: {value!r}")


def parse_fixture_refs(value: Any) -> List[FixtureRef]:
    """Parse the ``harness_config.fixture`` field, which may be scalar or list."""
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return [parse_fixture_ref(item) for item in value]
    return [parse_fixture_ref(value)]


def select_testbeds(
    instance: TestInstance,
    testbeds: Iterable[TestBed],
    platform: Optional[str] = None,
) -> List[TestBed]:
    """Return every testbed able to host ``instance``.

    The Lab Agent calls this when it allocates. Order is the binding file's
    order, which keeps allocation deterministic and therefore reproducible. An
    empty result is not an error here; the caller decides whether that is fatal
    (see :func:`explain_no_match`).
    """
    selected: List[TestBed] = []
    for testbed in testbeds:
        if platform is not None and testbed.platform != platform:
            continue
        if not instance.allows_platform(testbed.platform):
            continue
        if any(not testbed.offers(fixture) for fixture in instance.fixtures):
            continue
        selected.append(testbed)
    return selected


def explain_no_match(instance: TestInstance, testbeds: Iterable[TestBed]) -> str:
    """Build a human-readable reason why nothing matched.

    Fixture/mismatch failures are silent by design in the dispatch path, so this
    is the only place a human ever learns *why* a test was not scheduled.
    """
    testbeds = list(testbeds)
    if not testbeds:
        return "the binding file defines no testbeds"

    reasons: List[str] = []
    for testbed in testbeds:
        blockers: List[str] = []
        if not instance.allows_platform(testbed.platform):
            blockers.append(
                f"platform {testbed.platform} not allowed "
                f"(allow={instance.platform_allow or 'any'}, "
                f"exclude={instance.platform_exclude or 'none'})"
            )
        missing = [
            fixture.spec for fixture in instance.fixtures if not testbed.offers(fixture)
        ]
        if missing:
            have = ", ".join(fixture.spec for fixture in testbed.fixtures) or "none"
            blockers.append(f"missing fixture(s) {missing} (testbed offers: {have})")
        reasons.append(f"  - {testbed.id}: " + "; ".join(blockers) if blockers else f"  - {testbed.id}: unknown")

    return "no testbed satisfies the test:\n" + "\n".join(reasons)


def instance_fixtures_from_config(harness_config: Optional[dict]) -> List[FixtureRef]:
    """Read fixtures out of a ``harness_config`` block."""
    if not harness_config:
        return []
    try:
        return parse_fixture_refs(harness_config.get("fixture"))
    except BindingError as exc:  # normalise the error type for callers
        raise DiscoveryError(str(exc)) from exc


def implicit_requirements(instance: TestInstance) -> List[str]:
    """Feature types the test's own fixtures imply, from the bus each sits on.

    Only the binding knows that ``spi_nor_flash:spi1`` means a board with an
    ``spi`` instance -- which is exactly why this is derived rather than asked
    of the test. The test says "a NOR flash", the wiring says which bus it is on,
    and the board says whether it has one.
    """
    return [type_of(fixture.param) for fixture in instance.fixtures
            if fixture.param]


def wiring_problems(testbeds: Iterable[TestBed],
                    features: Iterable[str]) -> List[str]:
    """Contradictions between a lab's wiring and a board's declaration.

    A connection to ``spi1`` is a claim that the board has ``spi1``. When the
    platform's declaration does not back the claim, one of the two files is
    wrong, and the failure it would otherwise cause is a test that will not
    schedule for a reason nobody can read out of the report.

    Called by the Lab Agent when it loads its runner config -- the one place both
    files are in hand, now that the wiring is the agent's and the declaration is
    the board's.
    """
    available = list(features)
    problems: List[str] = []
    for testbed in testbeds:
        for connection in testbed.connections:
            if connection.options.get("console"):
                # An instrument with its own console is not wired to a board
                # bus: its spec names the instrument capability (``bt_peer:ble``),
                # not an instance the platform has to declare. Treating it as a
                # bus would reject a perfectly good binding on any board that
                # ships a features.yaml.
                continue
            bus = connection.fixture.param
            if not bus:
                continue
            if unsatisfied(available, [type_of(bus)]):
                problems.append(
                    f'{testbed.id}: fixture "{connection.fixture.spec}" is wired '
                    f"to {bus}, but platform {testbed.platform} declares no "
                    f"{type_of(bus)} instance"
                )
    return problems
