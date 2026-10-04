# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Fixture drivers: what the Lab Agent does to a fixture before a test runs.

A *passive* fixture is pure wiring -- a flash chip soldered to a bus, a loopback
jumper -- and needs no action. An *active* fixture is a device the runner has to
drive: a peer board, a phone, an instrument. This milestone ships only the
passive driver plus the interface an active one must implement, so that adding
one later changes no caller.

The interface deliberately takes the ``TestBed`` and the ``Connection`` rather
than a bare id: a driver needs the pinmap and mux channel the connection
declares, and those are the things the design forbids test cases from knowing.
"""

from __future__ import annotations

from typing import Any
from typing import Dict
from typing import Optional
from typing import Type

from sdk_test.model import Connection
from sdk_test.model import TestBed

from ..errors import LabAgentError


class FixtureDriver:
    """Contract for preparing and releasing one fixture."""

    def setup(self, testbed: TestBed, connection: Connection) -> None:
        """Bring the fixture into the state a test expects. May be a no-op."""

    def teardown(self, testbed: TestBed, connection: Connection) -> None:
        """Release the fixture. Must be safe to call after a failed setup."""

    def action(self, testbed: TestBed, connection: Connection, action: str, **kwargs: Any) -> Any:
        """Perform a named action the test asked for (e.g. erase, power off)."""
        raise LabAgentError(
            f'Fixture "{connection.fixture.spec}" does not support action "{action}"'
        )

    @property
    def active(self) -> bool:
        """Whether this driver does anything beyond wiring."""
        return False


class PassiveFixtureDriver(FixtureDriver):
    """A fixture that is nothing but wiring."""


_REGISTRY: Dict[str, Type[FixtureDriver]] = {}


def register_driver(name: str, driver: Type[FixtureDriver]) -> None:
    """Make ``driver`` the implementation used for fixtures with this name."""
    _REGISTRY[str(name)] = driver


def driver_for(connection: Connection) -> FixtureDriver:
    """Instantiate the driver responsible for one connection."""
    factory = _REGISTRY.get(connection.fixture.name)
    if factory is None:
        return PassiveFixtureDriver()
    return factory()


def registered_drivers() -> Dict[str, Type[FixtureDriver]]:
    return dict(_REGISTRY)


def driver_name(connection: Connection) -> Optional[str]:
    """The registered driver's name for a connection, if one is registered."""
    if connection.fixture.name in _REGISTRY:
        return connection.fixture.name
    return None
