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

import os
from dataclasses import dataclass
from typing import Any
from typing import Iterable
from typing import List
from typing import Mapping
from typing import Optional
from typing import Tuple

from .errors import BindingError
from .errors import DiscoveryError
from .errors import LabAgentError
from .fixture_types import FixtureTypes
from .fixture_types import default_fixture_types
from .model import Connection
from .model import ConsoleConfig
from .model import FixtureRef
from .model import TestInstance
from .model import TestBed
from .platforms import type_of
from .platforms import unsatisfied

#: Being explicit about the fixture vocabulary is the default; setting this to
#: ``1`` makes a type missing from the catalogue an error instead of a warning.
#: The warning exists because a lab may define its own types, so this is only
#: for a tree whose authors want typos caught hard.
ENV_STRICT_FIXTURES = "SIFLI_TEST_STRICT_FIXTURES"


def parse_fixture_ref(value: Any) -> FixtureRef:
    """Parse one fixture reference.

    Accepted shapes::

        "spi_nor_flash"                 -> name only, any wiring
        "spi_nor_flash:spi2"            -> the instance wired to spi2
        "bt_peer:ble,classic"           -> an instrument providing both capabilities
        "bt_peer@peer1:ble"             -> that instance, which must provide ble
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
        instance = None
        if "@" in name:
            name, _, instance = name.partition("@")
            name = name.strip()
            instance = instance.strip() or None
        if not name:
            raise BindingError(f'Fixture reference "{value}" has no name')
        return FixtureRef(name=name, param=param, instance=instance)

    if isinstance(value, dict):
        if "fixture" in value:
            ref = parse_fixture_ref(value["fixture"])
            if "param" in value and value["param"] is not None:
                ref = FixtureRef(ref.name, str(value["param"]), ref.instance)
            if "instance" in value and value["instance"] is not None:
                ref = FixtureRef(ref.name, ref.param, str(value["instance"]))
            return ref
        name = value.get("name")
        if not name:
            raise BindingError(f"Fixture mapping needs a name: {value!r}")
        # `{"name": "bt_peer@peer1"}` means the same as the string form.
        text = str(name)
        instance = value.get("instance")
        if "@" in text:
            text, _, pinned = text.partition("@")
            instance = instance if instance is not None else pinned.strip() or None
        param = value.get("param")
        return FixtureRef(
            text.strip(),
            str(param) if param is not None else None,
            str(instance) if instance is not None else None,
        )

    raise BindingError(f"Unsupported fixture reference: {value!r}")


def parse_fixture_refs(value: Any) -> List[FixtureRef]:
    """Parse the ``harness_config.fixture`` field, which may be scalar or list."""
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return [parse_fixture_ref(item) for item in value]
    return [parse_fixture_ref(value)]


def strict_fixture_types(environ: Optional[Mapping[str, str]] = None) -> bool:
    """Whether an unregistered fixture type is an error rather than a warning."""
    current = os.environ if environ is None else environ
    return current.get(ENV_STRICT_FIXTURES) == "1"


def unregistered_fixture_types(
    fixtures: Iterable[FixtureRef],
    contracts: Optional[FixtureTypes] = None,
) -> List[str]:
    """The requested fixture type names that have no catalogue entry.

    A lab may define a type of its own, so this is the input to a warning by
    default; callers that want it stricter ask :func:`strict_fixture_types`.
    Names are returned once each, in request order.
    """
    catalogue = contracts if contracts is not None else default_fixture_types()
    missing: List[str] = []
    for ref in fixtures:
        if ref.name not in catalogue and ref.name not in missing:
            missing.append(ref.name)
    return missing


def testbeds_from_descriptions(entries: Any) -> List[TestBed]:
    """Turn a Lab Agent's ``GET /testbeds`` payload into testbeds.

    The agent reports each fixture as the spec its connection declared, so
    reparsing those specs is what lets the SDK side answer "does this lab offer
    what this test asks for?" with :meth:`TestBed.offers` -- the same rule the
    agent allocates with -- instead of a second matching rule that could
    disagree with it.

    :raises LabAgentError: the payload is not a list of testbed descriptions.
    """
    if not isinstance(entries, (list, tuple)):
        raise LabAgentError(
            "The Lab Agent returned no testbed list; expected a list under "
            f"\"testbeds\", got {type(entries).__name__}"
        )

    testbeds: List[TestBed] = []
    for index, entry in enumerate(entries):
        where = f"testbeds[{index}]"
        if not isinstance(entry, dict):
            raise LabAgentError(
                f"The Lab Agent returned a malformed {where}: expected an "
                f"object, got {type(entry).__name__}"
            )
        testbed_id = entry.get("id")
        platform = entry.get("platform")
        if not testbed_id or not platform:
            raise LabAgentError(
                f"The Lab Agent returned a {where} without "
                + ('an "id"' if not testbed_id else 'a "platform"')
            )

        specs = entry.get("fixtures") or []
        if not isinstance(specs, (list, tuple)):
            raise LabAgentError(
                f"The Lab Agent returned a malformed {where}.fixtures: "
                "expected a list of fixture specs"
            )
        try:
            connections = [
                Connection(fixture=parse_fixture_ref(spec)) for spec in specs
            ]
        except BindingError as exc:
            raise LabAgentError(
                f"The Lab Agent returned a malformed {where}.fixtures: {exc}"
            ) from exc

        console_raw = entry.get("console")
        console = None
        if isinstance(console_raw, dict) and console_raw.get("port"):
            if "baud" in console_raw:
                try:
                    baud = int(console_raw["baud"])
                except (TypeError, ValueError) as exc:
                    raise LabAgentError(
                        f"The Lab Agent returned a non-numeric "
                        f"{where}.console.baud"
                    ) from exc
                console = ConsoleConfig(port=str(console_raw["port"]), baud=baud)
            else:
                # The model's own default, rather than a second copy of it here.
                console = ConsoleConfig(port=str(console_raw["port"]))

        testbeds.append(
            TestBed(
                id=str(testbed_id),
                platform=str(platform),
                runner=entry.get("runner"),
                console=console,
                connections=connections,
                # `busy`, `holder` and the rest are the agent's own; keeping
                # them verbatim is the same rule as for a binding entry.
                raw=dict(entry),
            )
        )
    return testbeds


@dataclass(frozen=True)
class FixtureGap:
    """Why one test's fixture requests cannot be hosted on one platform.

    Two shapes, because they need different words and are different mistakes:

    * :attr:`unoffered` names fixtures *nobody* on the platform has -- the lab
      is missing hardware, or the spec is spelled differently;
    * :attr:`split` means every fixture exists somewhere on the platform, but no
      single testbed has them all. Allocation picks one testbed per run, so this
      is just as fatal; the fix is a wiring change, not a new purchase.
    """

    unoffered: Tuple[FixtureRef, ...] = ()
    split: bool = False

    def __bool__(self) -> bool:
        return bool(self.unoffered) or self.split


def fixture_gap(
    required: Iterable[FixtureRef],
    platform: str,
    testbeds: Iterable[TestBed],
) -> Optional[FixtureGap]:
    """Why ``required`` cannot all be hosted on ``platform``, or ``None``.

    A test's fixture list means "all of these", and allocation asks *one*
    testbed to hold them all (:func:`sdk_test.fixtures.select_testbeds`), so the
    question is asked of a single testbed here too. Checking each fixture
    separately would pass a plan that allocation then refuses -- after a build.

    Matching is :meth:`TestBed.offers`, which is what allocation uses, so a
    parameterized type must match its spec exactly here too.

    A busy testbed still counts: it is the lab's, and it becomes free. Whether
    hardware is available *now* is the allocation's question, not this one's.
    """
    wanted = list(required)
    candidates = [testbed for testbed in testbeds if testbed.platform == platform]
    if not wanted:
        return None
    if any(all(testbed.offers(ref) for ref in wanted) for testbed in candidates):
        return None

    unoffered = tuple(
        ref
        for ref in wanted
        if not any(testbed.offers(ref) for testbed in candidates)
    )
    if unoffered:
        return FixtureGap(unoffered=unoffered)
    return FixtureGap(split=True)


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
