# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Data model shared by discovery, binding and the runner.

The vocabulary follows ``SDK自动化测试.docx``: a *testbed* is the smallest
allocatable unit (a board plus its wiring), a *fixture* is a capability
descriptor declared by both the test and the testbed, and a *test instance* is
one entry of a ``testcase.yaml`` ``tests:`` list.
"""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field
from typing import Any
from typing import Dict
from typing import List
from typing import Optional


@dataclass(frozen=True)
class FixtureRef:
    """What a test needs from a fixture, in one of five shapes.

    ::

        "spi_nor_flash"                 any instance
        "spi_nor_flash:spi2"            the instance wired to bus spi2
        "bt_peer:ble"                   an instrument providing the ble capability
        "bt_peer:ble,classic"           one providing both
        "bt_peer@peer1:ble"             that instance, which must provide ble

    The part after ``:`` is a comma-separated *capability* list; a lab whose
    connection offers more than the test asked for still satisfies it.
    ``@name`` pins one specific instance and is what a lab with several
    same-named instruments uses; it is separate from the capability list
    because one string cannot be both a capability and an instance name.
    """

    name: str
    param: Optional[str] = None
    instance: Optional[str] = None

    @property
    def capabilities(self) -> frozenset:
        """The capabilities this reference requires, from the comma list."""
        if not self.param:
            return frozenset()
        return frozenset(
            entry.strip() for entry in self.param.split(",") if entry.strip()
        )

    @property
    def spec(self) -> str:
        text = self.name
        if self.instance:
            text += f"@{self.instance}"
        if self.param:
            text += f":{self.param}"
        return text

    def accepts(self, available: "FixtureRef") -> bool:
        """True when ``available`` (what a testbed offers) satisfies ``self``."""
        if self.name != available.name:
            return False
        if self.instance is not None and self.instance != available.instance:
            return False
        return self.capabilities <= available.capabilities

    def __str__(self) -> str:  # pragma: no cover - convenience only
        return self.spec


@dataclass
class ConsoleConfig:
    """Where a testbed's console lands, from the hardware team's point of view."""

    port: str
    baud: int = 1000000


@dataclass
class Connection:
    """One fixture wired to a testbed, with the pinmap used to wire it."""

    fixture: FixtureRef
    pinmap: List[Dict[str, str]] = field(default_factory=list)
    mux: Optional[str] = None
    options: Dict[str, Any] = field(default_factory=dict)
    """The connection entry's other keys, kept verbatim.

    Same rule as :attr:`TestBed.raw`: which keys a driver needs is the driver's
    business, and parsing them here would be a second schema to keep in step.
    An active fixture such as a peer board keeps its own console here, e.g.
    ``console: {port: COM9, baud: 1000000}``.
    """


@dataclass
class Mux:
    """An out-of-band switch. Never driven through the DUT's own IO."""

    id: str
    type: str = ""
    channel_num: int = 0
    runner: Optional[str] = None


@dataclass
class TestBed:
    """The smallest allocation unit. Exclusive: one test program at a time.

    It states what is *wired to* the board -- its connections, and the console
    and probes that reach it. It deliberately does not state what the board can
    do: that is the platform's own declaration, read from
    ``customer/boards/<board>/<core>/features.yaml`` by
    :mod:`sdk_test.platforms`, and asking the lab to repeat it was how the two
    came to disagree.
    """

    id: str
    platform: str
    runner: Optional[str] = None
    console: Optional[ConsoleConfig] = None
    connections: List[Connection] = field(default_factory=list)
    muxes: List[Mux] = field(default_factory=list)
    raw: Dict[str, Any] = field(default_factory=dict)
    """Keys of the binding entry that parsing leaves untouched.

    The binding file belongs to the Lab Agent, which owns the hardware details
    kept here (which J-Link probe, which flash transport). Preserving them keeps
    one file format for the binding and for the agent's runner config, instead of
    two schemas that can drift apart.
    """

    @property
    def fixtures(self) -> List[FixtureRef]:
        return [connection.fixture for connection in self.connections]

    def offers(self, required: FixtureRef) -> bool:
        return any(required.accepts(fixture) for fixture in self.fixtures)

    def runner_option(self, key: str, default: Any = None) -> Any:
        """Read one Lab-Agent-owned setting out of :attr:`raw`."""
        return self.raw.get(key, default)


@dataclass
class TestInstance:
    """One ``tests:`` entry, resolved to a runnable identity."""

    id: str
    testsuite_root: str
    testcase_dir: str
    tests_key: str
    description: str = ""
    tags: List[str] = field(default_factory=list)
    harness: str = "console"
    harness_config: Dict[str, Any] = field(default_factory=dict)
    timeout: int = 60
    platform_allow: List[str] = field(default_factory=list)
    platform_exclude: List[str] = field(default_factory=list)
    integration_platforms: List[str] = field(default_factory=list)
    depends_on: List[str] = field(default_factory=list)
    extra_args: List[str] = field(default_factory=list)
    build_only: bool = False
    skip: bool = False
    platform: Optional[str] = None
    #: Directory holding the SConstruct to build, relative to the testcase file.
    #: When unset, :mod:`sdk_test.runner` falls back to the convention that the
    #: project is the testcase directory or its ``project`` subdirectory.
    project_dir: Optional[str] = None
    fixtures: List[FixtureRef] = field(default_factory=list)

    def allows_platform(self, platform: str) -> bool:
        """Platform gating: explicit allow/exclude lists win over everything."""
        if self.platform_exclude and platform in self.platform_exclude:
            return False
        if self.platform_allow:
            return platform in self.platform_allow
        return True

    @property
    def build_dir_name(self) -> Optional[str]:
        """``build_<board>`` convention used across the SDK.

        ``None`` until a platform has been selected, because there is no build
        directory to name before then.
        """
        return f"build_{self.platform}" if self.platform else None
