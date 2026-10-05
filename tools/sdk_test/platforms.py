# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""What a platform offers, from the build's point of view.

This answers the first of the two questions a test plan asks, and it answers it
with no lab, no build and no serial port:

    can this test run on this platform at all?

The second question -- *which* board of that platform, wired how -- is the
binding's, and lives in :mod:`sdk_test.fixtures`. Keeping the two apart is what
makes ``sdk.py test plan`` able to say what is wrong before anything is built:
what a board offers is a property of the board and the build, not of the bench it
happens to sit on.

A platform states the **instances** it has; a test states the **type** it needs::

    customer/boards/<board>/<core>/features.yaml    features: [spi1, spi2, i2c2]
    <testcase>.yaml                                 depends_on: [spi]

The asymmetry is deliberate. Which bus a board routes a part out to is the
board's business, and the test's own code already resolves it -- what a driver
is built against is ``BSP_USING_SPI1``, not "some SPI". So a test names a type,
a platform names instances, and a type requirement is met by any instance of
that type. That is the same rule :meth:`FixtureRef.accepts` applies to a fixture
written with and without a parameter: the abstract form accepts any, the
concrete form pins one.

A resource that has no instances -- a GPIO port, say -- is declared by its own
name and is its own type. Declaring a type that another declared instance
already provides is an error, because it is precisely the redundancy that drifts
away from the build.
"""

from __future__ import annotations

import os
import re
from typing import Callable
from typing import Iterable
from typing import List
from typing import Optional
from typing import Set
from typing import Tuple

from .errors import PlatformError

#: The repository this package lives in. ``tools/sdk_test/platforms.py``.
REPOSITORY_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

#: Core suffixes the board directories use, mirroring ``runner``'s own list.
_CORE_SUFFIXES = ("_hcpu", "_lcpu", "_acpu")
_DEFAULT_CORE = "hcpu"

_TRAILING_DIGITS = re.compile(r'\d+$')

#: The prefix every peripheral switch in this SDK carries. ``spi1`` is compiled
#: in by ``BSP_USING_SPI1``, which is what :func:`feature_symbol` relies on.
_SYMBOL_PREFIX = "BSP_USING_"

#: The only key a feature declaration may carry.
_FEATURES_KEY = "features"


def split_platform(platform: str) -> Tuple[str, str]:
    """``qemu_sf32lb52x`` -> ``("qemu_sf32lb52x", "hcpu")``.

    A platform named without a core suffix means HCPU, the same rule
    :func:`sdk_test.runner.canonical_board_name` applies to build directories.
    """
    for suffix in _CORE_SUFFIXES:
        if platform.endswith(suffix):
            return platform[:-len(suffix)], suffix[1:]
    return platform, _DEFAULT_CORE


def features_path(platform: str, root: str = REPOSITORY_ROOT) -> str:
    """Where ``platform`` declares its features."""
    board, core = split_platform(platform)
    return os.path.join(root, "customer", "boards", board, core, "features.yaml")


def type_of(feature: str) -> str:
    """The type an instance belongs to: ``spi1`` -> ``spi``, ``gpio`` -> ``gpio``."""
    return _TRAILING_DIGITS.sub('', feature) or feature


def platform_types(features: Iterable[str]) -> Set[str]:
    """Every type the given instances provide."""
    return {type_of(feature) for feature in features}


def unsatisfied(features: Iterable[str], requirements: Iterable[str]) -> List[str]:
    """The requirements a platform does not meet, in the order they were given.

    A requirement is met by an instance of that name, or by any instance of that
    type. Empty means the platform satisfies everything asked of it.
    """
    available = set(features)
    types = platform_types(available)
    return [name for name in requirements
            if name not in available and name not in types]


def feature_symbol(feature: str) -> str:
    """The Kconfig symbol behind a declared instance: ``spi1`` -> ``BSP_USING_SPI1``.

    The convention is what makes a declaration checkable at all: a declared
    instance and the switch that compiles its driver are the same word, so the
    declaration can be held against the build it claims to describe.
    """
    return _SYMBOL_PREFIX + feature.upper()


def unresolved(features: Iterable[str], is_enabled: Callable[[str], bool]) -> List[str]:
    """Declared instances whose Kconfig symbol is off in this build."""
    return [name for name in features if not is_enabled(feature_symbol(name))]


def check_declaration(platform: str, is_enabled: Callable[[str], bool],
                      root: str = REPOSITORY_ROOT) -> None:
    """Raise when a board declares an instance that this build does not have.

    Called from a test project's ``SConscript`` with the build's own predicate
    (``GetDepend``), so a declaration cannot quietly go on describing a board
    that no longer exists. A ``features.yaml`` claiming a bus the build lacks
    lets a test be *allowed* onto a platform that cannot run it -- and that
    failure then lands on the device, where its cause is hardest to see.

    The reverse direction -- a peripheral the build has and the declaration
    omits -- is not checked: knowing which switches to look for would take a
    vocabulary of every peripheral in the tree, and that list would drift too.
    """
    features = declared_features(platform, root)
    if not features:
        return

    wrong = unresolved(features, is_enabled)
    if not wrong:
        return

    raise PlatformError(
        f"{features_path(platform, root)} declares {wrong}, but this build has "
        f"no {' or '.join(feature_symbol(name) for name in wrong)}. Either the "
        f"peripheral is not enabled for this board, or the declaration is stale "
        f"-- a declaration that does not match the build lets tests onto a "
        f"platform that cannot run them."
    )


def declared_features(platform: str,
                      root: str = REPOSITORY_ROOT) -> Optional[List[str]]:
    """Like :func:`load_platform_features`, but ``None`` when nothing is declared.

    The difference matters to a caller checking a *contradiction* rather than
    answering a *requirement*: an empty declaration is a claim that the board has
    no bus at all, while a missing file is merely silence, and only the first of
    those can contradict a binding.
    """
    if not os.path.isfile(features_path(platform, root)):
        return None
    return load_platform_features(platform, root)


def load_platform_features(platform: str,
                           root: str = REPOSITORY_ROOT) -> List[str]:
    """The instances ``platform`` declares, in file order.

    An *empty* declaration is a real answer -- the QEMU carrier genuinely has no
    peripherals, and saying so is what makes ``depends_on: [spi]`` filter
    anything on it. A *missing* file is an error, but only a caller that has
    requirements needs to read it at all, so a project that states none keeps
    working on a board that has not been annotated yet. The caller decides that;
    see :func:`sdk_test.runner.filter_reason`.
    """
    path = features_path(platform, root)
    if not os.path.isfile(path):
        raise PlatformError(
            f'"{platform}": no feature declaration. A test stating '
            f'`depends_on:` needs one. Either create {path} listing the bus '
            f'instances this board has (an empty list is a valid answer), or '
            f'drop the `depends_on:`.'
        )

    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - pyyaml is a core requirement
        raise PlatformError("pyyaml is required to read feature files") from exc

    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
    except OSError as exc:
        raise PlatformError(f"Cannot read {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise PlatformError(f"{path}: malformed YAML: {exc}") from exc

    if data is None:
        return []

    if not isinstance(data, dict):
        raise PlatformError(f"{path}: top level must be a mapping with a "
                            f'"{_FEATURES_KEY}:" list')

    unknown = sorted(set(data) - {_FEATURES_KEY})
    if unknown:
        raise PlatformError(
            f'{path}: unknown key(s) {unknown}; only "{_FEATURES_KEY}:" is read'
        )

    declared = data.get(_FEATURES_KEY) or []
    if not isinstance(declared, (list, tuple)):
        raise PlatformError(f'{path}: "{_FEATURES_KEY}" must be a list')

    features: List[str] = []
    for entry in declared:
        if not isinstance(entry, str) or not entry.strip():
            raise PlatformError(
                f'{path}: "{_FEATURES_KEY}" entries must be non-empty strings, '
                f"got {entry!r}"
            )
        name = entry.strip()
        if name in features:
            raise PlatformError(f'{path}: "{name}" is declared twice')
        features.append(name)

    redundant = sorted({
        name for name in features
        if any(other != name and type_of(other) == name for other in features)
    })
    if redundant:
        raise PlatformError(
            f"{path}: {redundant} name a type that another declared instance "
            f"already provides. Declare the instances only -- a type is derived "
            f"from them, and writing it as well is how the two drift apart."
        )

    return features
