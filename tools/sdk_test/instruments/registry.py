# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Instrument name -> host class.

The pytest plugin builds one session fixture per entry, so adding an
instrument is a registration here rather than an edit to the plugin. A fixture
name must be a Python identifier and must not shadow the framework's own
``dut`` or ``console`` fixtures, and the factory must accept ``capability=``;
all three are checked here, where the mistake is cheap to see.

A registration made while pytest is starting (a ``conftest`` imported before
collection) still installs its fixture. A registration from a test module is
too late: pytest has already parsed each plugin's fixtures, so the fixture
exists but nothing can request it.
"""

from __future__ import annotations

import inspect
from typing import Dict
from typing import Type

from .bt_peer import BtPeer

#: Fixture names an instrument may not take over. `dut`/`console` are the
#: framework's own; the rest are pytest built-ins whose shadowing would break
#: suites in ways that are hard to trace back to a registration.
RESERVED_NAMES = frozenset(
    {
        "dut",
        "console",
        "request",
        "tmp_path",
        "tmp_path_factory",
        "tmpdir",
        "tmpdir_factory",
        "capsys",
        "capfd",
        "capsysbinary",
        "capfdbinary",
        "monkeypatch",
        "caplog",
        "recwarn",
        "pytestconfig",
        "record_property",
        "record_xml_attribute",
        "record_testsuite_property",
    }
)

#: The registry the pytest plugin turns into session fixtures.
INSTRUMENTS: Dict[str, Type] = {
    "bt_peer": BtPeer,
}


def register_instrument(name: str, factory: Type) -> None:
    """Make ``factory`` the class behind fixture ``name``.

    The fixture is installed right away when pytest has already configured the
    plugin, so a registration made from a ``conftest`` still produces a usable
    fixture. Before that point the plugin installs every entry itself.
    """
    candidate = str(name)
    if not candidate.isidentifier():
        raise ValueError(
            f"instrument fixture name {candidate!r} is not a Python identifier; "
            "the pytest plugin creates a fixture with this exact name"
        )
    if candidate in RESERVED_NAMES:
        raise ValueError(
            f"instrument fixture name {candidate!r} collides with a framework "
            f"fixture ({sorted(RESERVED_NAMES)})"
        )
    if not callable(factory):
        raise TypeError(f"instrument factory for {candidate!r} is not callable")
    parameters = inspect.signature(factory).parameters
    if "capability" not in parameters and not any(
        parameter.kind is inspect.Parameter.VAR_KEYWORD
        for parameter in parameters.values()
    ):
        raise TypeError(
            f"instrument factory {factory!r} must accept a `capability=` keyword: "
            "the fixture passes the part after ':' in the declared spec"
        )

    INSTRUMENTS[candidate] = factory

    # Late import: this module is imported by the plugin, which owns the
    # fixtures, so the dependency can only go this way at call time. Outside a
    # pytest process there is nothing to install into, and saying so is more
    # useful than a bare ModuleNotFoundError.
    try:
        from .. import pytest_plugin
    except ModuleNotFoundError as exc:  # pragma: no cover - only without pytest
        raise RuntimeError(
            "register_instrument() needs the pytest environment: the fixture it "
            "creates lives in sdk_test.pytest_plugin. Run it under the SDK "
            "environment (`export.ps1` / `export.sh`) or from a pytest plugin."
        ) from exc

    pytest_plugin.register_instrument_fixture(candidate, factory)
