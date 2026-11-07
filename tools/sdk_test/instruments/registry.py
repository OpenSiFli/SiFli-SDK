# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Instrument name -> host class.

The pytest plugin builds one session fixture per entry, so adding an instrument
is a registration here rather than an edit to the plugin. A fixture name must
be a Python identifier and must not shadow the framework's own ``dut`` or
``console`` fixtures; both are checked at import time, where the mistake is
cheap to see.
"""

from __future__ import annotations

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
    """Make ``factory`` the class behind fixture ``name``."""
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
    INSTRUMENTS[candidate] = factory
