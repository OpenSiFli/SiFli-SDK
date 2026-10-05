# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Binding (hardware-map) loading.

This is the only file format in the framework that mentions physical reality:
ports, debugger ids, pinmaps and mux channels. The design doc puts it under the
hardware team's ownership and keeps it *out* of test cases entirely, so that a
test only ever states what it needs.

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
        muxes:
          - id: mux_01
            type: relay
            channel_num: 4

A binding does not state what a board *can do*. That is stated once, by the
board, in ``customer/boards/<board>/<core>/features.yaml``; a binding states only
what is wired to it. Two files stating the same fact is what let them disagree.
"""

from __future__ import annotations

import os
from typing import Any
from typing import Dict
from typing import List
from typing import Optional

from .errors import BindingError
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


def load_binding(path: str) -> List[TestBed]:
    """Load and validate a binding file from disk."""
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - pyyaml is a core requirement
        raise BindingError("pyyaml is required to read binding files") from exc

    if not os.path.isfile(path):
        raise BindingError(f'Binding file not found: "{path}"')

    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
    except OSError as exc:
        raise BindingError(f"Cannot read {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise BindingError(f"{path}: malformed YAML: {exc}") from exc

    if data is None:
        raise BindingError(f"{path}: file is empty")

    return parse_binding(data, source=path)
