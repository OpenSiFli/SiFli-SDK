# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Binding (hardware-map) loading.

This is the only file format in the framework that mentions physical reality:
ports, debugger ids, pinmaps and mux channels. The design doc puts it under the
hardware team's ownership and keeps it *out* of test cases entirely, so that a
test only ever states what it needs. The Lab Agent is its only reader in
production: the SDK side plans (instance, platform) pairs and never loads one.

What it does *not* state is what a board can do. That is the board's own
declaration, in ``customer/boards/<board>/<core>/features.yaml``, read by
:mod:`sdk_test.platforms`. A binding says what is wired to the board, and the
two files meeting -- :func:`sdk_test.fixtures.wiring_problems` -- is where a
contradiction between them is caught.

Shape (see ``examples/binding.local.yaml``)::

    testbed:
      - id: sf32lb52-lcd_n16r8_01
        platform: sf32lb52-lcd_n16r8
        runner: local
        console:
          port: /dev/ttyACM0
          baud: 1000000
        connections:
          - fixture: spi_nor_flash_py25q128:spi1
            pinmap:
              - {pin: PA00, signal: CS}
            mux: mux_01.channel.3
          # An instrument the test drives: a peer board that answers on its own
          # UART. It is not a testbed and is never built or flashed -- the lab
          # keeps its firmware fixed -- but the agent can open its console and
          # the pytest suite can read and write it like the DUT's.
          - fixture: bt_peer:ble
            console:
              port: COM9
              baud: 1000000
        muxes:
          - id: mux_01
            type: relay
            channel_num: 4

A binding does not state what a board *can do*. That is stated once, by the
board, in ``customer/boards/<board>/<core>/features.yaml``; a binding states only
what is wired to it. Two files stating the same fact is what let them disagree.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Tuple

from .errors import BindingError
from .fixture_types import FixtureTypes
from .fixture_types import SOURCE_PARAMS
from .fixture_types import default_fixture_types
from .fixtures import parse_fixture_ref
from .model import Connection
from .model import ConsoleConfig
from .model import Mux
from .model import TestBed

DEFAULT_BAUD = 1000000


def _require_mapping(value: Any, where: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise BindingError(f"{where} must be a mapping, got {type(value).__name__}")
    return value


def _parse_console(value: Any, where: str) -> Optional[ConsoleConfig]:
    if value is None:
        return None
    mapping = _require_mapping(value, where)
    port = mapping.get("port")
    if not port:
        raise BindingError(f'{where} needs a "port" (e.g. COM3, /dev/ttyACM0)')
    try:
        baud = int(mapping.get("baud", DEFAULT_BAUD))
    except (TypeError, ValueError) as exc:
        raise BindingError(f'{where}.baud must be an integer') from exc
    return ConsoleConfig(port=str(port), baud=baud)


def _parse_connection(value: Any, where: str) -> Connection:
    mapping = _require_mapping(value, where)
    if "fixture" not in mapping:
        raise BindingError(f'{where} needs a "fixture"')
    pinmap = mapping.get("pinmap") or []
    if not isinstance(pinmap, list):
        raise BindingError(f"{where}.pinmap must be a list")
    return Connection(
        fixture=parse_fixture_ref(mapping["fixture"]),
        pinmap=[dict(entry) for entry in pinmap],
        mux=mapping.get("mux"),
        # Anything else is the fixture driver's own business -- an active
        # fixture such as a peer board keeps its console here. Keeping it
        # verbatim is the same rule as TestBed.raw, and for the same reason:
        # a second schema would be a second place to forget a key.
        options={
            key: entry
            for key, entry in mapping.items()
            if key not in ("fixture", "pinmap", "mux")
        },
    )


def _parse_mux(value: Any, where: str) -> Mux:
    mapping = _require_mapping(value, where)
    mux_id = mapping.get("id")
    if not mux_id:
        raise BindingError(f'{where} needs an "id"')
    try:
        channels = int(mapping.get("channel_num", 0))
    except (TypeError, ValueError) as exc:
        raise BindingError(f"{where}.channel_num must be an integer") from exc
    return Mux(
        id=str(mux_id),
        type=str(mapping.get("type", "")),
        channel_num=channels,
        runner=mapping.get("runner"),
    )


def _parse_testbed(value: Any, index: int) -> TestBed:
    where = f"testbed[{index}]"
    mapping = _require_mapping(value, where)

    testbed_id = mapping.get("id")
    if not testbed_id:
        raise BindingError(f'{where} needs an "id"')
    platform = mapping.get("platform")
    if not platform:
        raise BindingError(f'{where} ("{testbed_id}") needs a "platform" (board name)')

    connections = [
        _parse_connection(entry, f"{where}.connections[{position}]")
        for position, entry in enumerate(mapping.get("connections") or [])
    ]
    muxes = [
        _parse_mux(entry, f"{where}.muxes[{position}]")
        for position, entry in enumerate(mapping.get("muxes") or [])
    ]

    mux_ids = {mux.id for mux in muxes}
    for connection in connections:
        if connection.mux:
            mux_name = connection.mux.split(".", 1)[0]
            if mux_ids and mux_name not in mux_ids:
                raise BindingError(
                    f'{where}: connection references unknown mux "{mux_name}" '
                    f"(known: {sorted(mux_ids)})"
                )

    return TestBed(
        id=str(testbed_id),
        platform=str(platform),
        runner=mapping.get("runner"),
        console=_parse_console(mapping.get("console"), f"{where}.console"),
        connections=connections,
        muxes=muxes,
        # Kept verbatim: the Lab Agent reads its own settings (probe serial,
        # flash transport) from the same entry, and the SDK side must not be
        # the thing that decides which keys those are.
        raw=dict(mapping),
    )


def parse_binding(data: Any, source: str = "<memory>") -> List[TestBed]:
    """Parse an already-loaded binding document."""
    if isinstance(data, list):
        entries = data
    elif isinstance(data, dict):
        entries = data.get("testbed")
        if entries is None:
            raise BindingError(
                f'{source}: expected a top-level "testbed:" list (or a bare list)'
            )
    else:
        raise BindingError(f"{source}: top level must be a mapping or a list")

    if not isinstance(entries, list):
        raise BindingError(f'{source}: "testbed" must be a list')

    testbeds = [_parse_testbed(entry, index) for index, entry in enumerate(entries)]

    seen: set = set()
    for testbed in testbeds:
        if testbed.id in seen:
            raise BindingError(f'{source}: duplicate testbed id "{testbed.id}"')
        seen.add(testbed.id)

    return testbeds


@dataclass(frozen=True)
class ContractIssue:
    """One problem with a connection, attributed to the testbed that carries it.

    The testbed id is kept apart from the text so a caller can drop exactly the
    offending testbed -- the Lab Agent does that at load time -- while still
    printing a message a human can act on.
    """

    testbed_id: str
    severity: str
    message: str


def fixture_contract_issues(
    testbeds: Iterable[TestBed],
    contracts: Optional[FixtureTypes] = None,
) -> List[ContractIssue]:
    """Check a binding against the fixture type catalogue and option contracts.

    The catalogue states which capability tokens a type accepts; the option
    contract states which options exist and which of them come from the
    runner-side params file. Three rules follow, and their severities differ on
    purpose:

    * a capability token the catalogue does not list is a warning. A lab may
      extend a vocabulary (``wifi_ap:6e``) and the framework cannot tell that
      from a typo, so it must not refuse hardware over it.
    * an option the contract sources from params must not appear in a binding.
      The binding belongs to the lab and is committed; a password written there
      is exactly the thing this mechanism exists to prevent. This is an error.
    * an option the contract does not mention is a warning, not an error. Driver
      private keys (an instrument's own ``console:``) are legitimate, and
      refusing them would be the second schema :mod:`sdk_test.binding` warned
      against. The warning names the keys the contract does allow, so a typo is
      still easy to see. This is checked only for a parameterized type -- a
      catalogue entry that merely records a type's name and capabilities says
      nothing about its options.

    Types with no entry are not checked at all, which keeps lab-defined types
    working unchanged.

    :param testbeds: parsed testbeds.
    :param contracts: contracts to consult; defaults to the ones shipped here.
    :return: one :class:`ContractIssue` per problem.
    """
    catalogue = contracts if contracts is not None else default_fixture_types()
    issues: List[ContractIssue] = []

    for testbed in testbeds:
        for connection in testbed.connections:
            ref = connection.fixture
            contract = catalogue.get(ref.name)
            if contract is None:
                continue

            where = f'testbed "{testbed.id}" connection "{ref.spec}"'

            vocabulary = set(contract.capabilities)
            if vocabulary:
                for token in sorted(ref.capabilities - vocabulary):
                    issues.append(
                        ContractIssue(
                            testbed_id=testbed.id,
                            severity="warning",
                            message=(
                                f'{where} requires capability "{token}", '
                                f"which the {contract.name} entry does not "
                                "list; it allows: "
                                f"{', '.join(contract.capabilities)}"
                            ),
                        )
                    )

            # A catalogue entry without a params-sourced option is a name and a
            # capability list, not a statement about connection keys. Checking
            # them would turn an instrument's driver-private `console:` into an
            # "unknown key".
            if not contract.params_option_names:
                continue

            allowed = contract.option_names
            for key in sorted(connection.options):
                if key in contract.params_option_names:
                    issues.append(
                        ContractIssue(
                            testbed_id=testbed.id,
                            severity="error",
                            message=(
                                f'{where} sets "{key}", which the '
                                f"{contract.name} contract declares source: "
                                f"{SOURCE_PARAMS}. Put it in the params file, "
                                "not in the binding."
                            ),
                        )
                    )
                elif key not in allowed:
                    permitted = ", ".join(allowed) or "none"
                    issues.append(
                        ContractIssue(
                            testbed_id=testbed.id,
                            severity="warning",
                            message=(
                                f'{where} has unknown key "{key}"; the '
                                f"{contract.name} contract allows: {permitted}"
                            ),
                        )
                    )
    return issues


def check_fixture_contracts(
    testbeds: Iterable[TestBed],
    contracts: Optional[FixtureTypes] = None,
) -> Tuple[List[str], List[str]]:
    """:func:`fixture_contract_issues` as ``(errors, warnings)`` strings."""
    issues = fixture_contract_issues(testbeds, contracts)
    errors = [issue.message for issue in issues if issue.severity == "error"]
    warnings = [issue.message for issue in issues if issue.severity != "error"]
    return errors, warnings
