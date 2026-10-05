# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Lab Agent runner configuration.

A runner config describes *one machine*: which testbeds are plugged into it and
where its server listens. It is the same schema as a binding (hardware map),
parsed by the same code (:func:`sdk_test.binding.parse_binding`); the two
describe the same objects, and a second schema would only be a chance for them
to disagree.

The runner file adds an optional ``runner:`` block for machine-wide settings; a
testbed entry may additionally carry ``jlink:`` and ``flash:`` blocks, which this
file reads through :meth:`sdk_test.model.TestBed.runner_option`. The Lab Agent
reads all of it; the SDK side reads none of it.
"""

from __future__ import annotations

import os
import shutil
from dataclasses import dataclass
from dataclasses import field
from typing import Any
from typing import Dict
from typing import List
from typing import Optional

from sdk_test.binding import parse_binding
from sdk_test.fixtures import wiring_problems
from sdk_test.model import TestBed
from sdk_test.platforms import REPOSITORY_ROOT
from sdk_test.platforms import declared_features

from .errors import ConfigError

#: Where a J-Link install usually lands when it is not already on PATH.
_JLINK_CANDIDATES = (
    r"C:\Program Files\SEGGER\JLink\JLink.exe",
    r"C:\Program Files (x86)\SEGGER\JLink\JLink.exe",
    r"C:\Keil_v5\ARM\Segger\JLink.exe",
    r"C:\Keil_v5\ARM\Segger\JLink.exe",
)

_JLINK_NAMES = ("JLinkExe", "jlink.exe", "JLink.exe", "jlink")


def find_jlink(explicit: Optional[str] = None) -> str:
    """Locate the J-Link commander, or explain what to install.

    A lab machine may have several J-Link installs and no PATH entry, so an
    explicit path in the runner config always wins.
    """
    if explicit:
        if os.path.isfile(explicit):
            return explicit
        found = shutil.which(explicit)
        if found:
            return found
        raise ConfigError(f'Configured jlink_exe does not exist: "{explicit}"')

    for name in _JLINK_NAMES:
        found = shutil.which(name)
        if found:
            return found

    for candidate in _JLINK_CANDIDATES:
        if os.path.isfile(candidate):
            return candidate

    raise ConfigError(
        "Cannot find the J-Link commander. Install SEGGER J-Link, put it on "
        'PATH, or set `runner.jlink_exe` in the runner config.'
    )


@dataclass
class RunnerSettings:
    """Machine-wide settings, from the runner config's ``runner:`` block."""

    name: str = "local"
    host: str = "127.0.0.1"
    port: int = 8760
    #: How long a single console read blocks waiting for new data, in seconds.
    console_poll: float = 0.2
    #: Ceiling on the retained console transcript per testbed, in characters.
    max_buffer: int = 4 * 1024 * 1024
    #: Per-command timeout for the debug probe, in seconds.
    jlink_timeout: float = 60.0
    jlink_exe: Optional[str] = None

    @classmethod
    def from_mapping(cls, data: Any) -> "RunnerSettings":
        if data is None:
            return cls()
        if not isinstance(data, dict):
            raise ConfigError('The "runner" block must be a mapping')

        known = {field_name for field_name in cls.__dataclass_fields__}
        unknown = set(data) - known
        if unknown:
            raise ConfigError(
                f'Unknown key(s) in the "runner" block: {", ".join(sorted(unknown))} '
                f"(known: {', '.join(sorted(known))})"
            )

        settings = cls()
        for key, value in data.items():
            setattr(settings, key, value)

        if not isinstance(settings.port, int):
            raise ConfigError('runner.port must be an integer')
        if not isinstance(settings.max_buffer, int) or settings.max_buffer < 1024:
            raise ConfigError('runner.max_buffer must be an integer >= 1024')
        for key in ("console_poll", "jlink_timeout"):
            value = getattr(settings, key)
            if not isinstance(value, (int, float)) or value <= 0:
                raise ConfigError(f"runner.{key} must be a positive number")
        return settings


@dataclass
class RunnerConfig:
    """Everything one Lab Agent process needs to know about its machine."""

    settings: RunnerSettings = field(default_factory=RunnerSettings)
    testbeds: List[TestBed] = field(default_factory=list)
    path: Optional[str] = None

    def by_id(self, testbed_id: str) -> TestBed:
        for testbed in self.testbeds:
            if testbed.id == testbed_id:
                return testbed
        raise ConfigError(
            f'Unknown testbed "{testbed_id}" '
            f"(this runner has: {', '.join(t.id for t in self.testbeds) or 'none'})"
        )

    def check(self) -> List[str]:
        """Report configuration problems that are not fatal but will bite later.

        Called on startup so a mis-wired lab is discovered at boot rather than
        halfway through a nightly run.
        """
        warnings: List[str] = []
        for testbed in self.testbeds:
            if testbed.console is None:
                warnings.append(
                    f'{testbed.id}: no "console" port; console harnesses cannot run on it'
                )
            elif os.name == "nt" and not str(testbed.console.port).upper().startswith("COM"):
                warnings.append(
                    f'{testbed.id}: console port "{testbed.console.port}" is not a COM port '
                    "(expected on Windows)"
                )
        return warnings


def rule_out_contradictions(
    testbeds: List[TestBed], source: str, root: str = REPOSITORY_ROOT
) -> None:
    """Refuse a lab whose wiring contradicts a board's own declaration.

    The two files meet here and nowhere else: the wiring is this machine's, and
    the declaration belongs to the board, in the SDK checkout this agent runs
    from. A connection to ``spi1`` on a board that declares no ``spi`` instance
    means one of the two is wrong, and left alone it surfaces as a test that
    never gets scheduled, for a reason nothing in the report can show.

    A board with *no* declaration is not a contradiction: a missing file is
    silence, while an empty one is a claim, and only a claim can disagree.
    """
    for testbed in testbeds:
        features = declared_features(testbed.platform, root)
        if features is None:
            continue
        problems = wiring_problems([testbed], features)
        if problems:
            raise ConfigError(f"{source}: " + "; ".join(problems))


def parse_runner_config(
    data: Any,
    source: str = "<memory>",
    path: Optional[str] = None,
    sdk_root: Optional[str] = None,
) -> RunnerConfig:
    """Parse an already-loaded runner config document.

    ``sdk_root`` is where the boards' own declarations are looked for, for the
    contradiction check below. It defaults to the checkout this agent runs from,
    which is the one holding the boards it drives.
    """
    if not isinstance(data, dict):
        raise ConfigError(f"{source}: top level must be a mapping")

    settings = RunnerSettings.from_mapping(data.get("runner"))
    # Duplicate testbed ids are already rejected by the shared binding parser.
    testbeds = parse_binding(data, source=source)
    # Then the wiring is held against the boards' own declarations, which is the
    # only place both files are in hand at once.
    rule_out_contradictions(testbeds, source, sdk_root or REPOSITORY_ROOT)
    return RunnerConfig(settings=settings, testbeds=testbeds, path=path)


def load_runner_config(path: str) -> RunnerConfig:
    """Load and validate a runner config from disk."""
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - pyyaml is a core requirement
        raise ConfigError("pyyaml is required to read a runner config") from exc

    if not os.path.isfile(path):
        raise ConfigError(f'Runner config not found: "{path}"')

    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
    except OSError as exc:
        raise ConfigError(f"Cannot read {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ConfigError(f"{path}: malformed YAML: {exc}") from exc

    if data is None:
        raise ConfigError(f"{path}: file is empty")

    return parse_runner_config(data, source=path, path=os.path.abspath(path))
