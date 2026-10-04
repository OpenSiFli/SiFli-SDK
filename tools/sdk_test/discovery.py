# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Test-case discovery: walk a tree, turn ``testcase.yaml`` files into instances.

Rules kept from Twister because the design doc reuses its vocabulary:

* the test id is ``<directory relative to the test-suite root>/<tests key>``;
* ``common:`` supplies defaults that every ``tests:`` entry inherits, and a
  per-test key always wins;
* both the list form (``- id: <name>``) and the mapping form
  (``<name>: {..}``) are accepted, because the design doc uses the former and
  Twister users expect the latter.
"""

from __future__ import annotations

import json
import os
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Sequence

from .errors import DiscoveryError
from .fixtures import instance_fixtures_from_config
from .model import DutSpec
from .model import TestInstance

TESTCASE_FILENAMES = ("testcase.yaml", "testcase.yml")

#: Directories never descended into: build outputs and repo internals.
_SKIP_DIR_PREFIXES = ("build_", "build", ".")

_SCHEMA_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "schema", "testcase.schema.json")

_SCHEMA_CACHE: Optional[dict] = None


def load_schema() -> dict:
    """Load the JSON schema (cached). The schema is the single source of truth
    for which keys exist, so discovery and documentation cannot drift apart."""
    global _SCHEMA_CACHE
    if _SCHEMA_CACHE is None:
        try:
            with open(_SCHEMA_PATH, "r", encoding="utf-8") as handle:
                _SCHEMA_CACHE = json.load(handle)
        except OSError as exc:
            raise DiscoveryError(f"Cannot read testcase schema: {exc}") from exc
    return _SCHEMA_CACHE


def allowed_test_keys() -> set:
    """Every field accepted inside a ``tests:`` entry."""
    return set(load_schema()["$defs"]["testEntry"]["properties"].keys())


def _as_list(value: Any, field: str) -> List[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [value]
    if isinstance(value, (list, tuple)):
        return [str(item) for item in value]
    raise DiscoveryError(f'Field "{field}" must be a string or a list of strings')


def _validate(raw: Any, path: str) -> None:
    """Schema-validate a parsed YAML document, reporting every problem at once."""
    try:
        import jsonschema
    except ImportError:  # pragma: no cover - jsonschema is a core requirement
        return

    validator = jsonschema.Draft202012Validator(load_schema())
    errors = sorted(validator.iter_errors(raw), key=lambda error: list(error.path))
    if not errors:
        return

    lines = [f"{path}: invalid testcase file:"]
    for error in errors[:10]:
        location = "/".join(str(part) for part in error.path) or "<root>"
        lines.append(f"  - {location}: {error.message}")
    if len(errors) > 10:
        lines.append(f"  ... and {len(errors) - 10} more")
    raise DiscoveryError("\n".join(lines))


def _parse_dut_specs(value: Any, field: str) -> List[DutSpec]:
    if value is None:
        return []
    if not isinstance(value, list):
        raise DiscoveryError(f'Field "{field}" must be a list of DUT specifications')

    specs: List[DutSpec] = []
    for index, item in enumerate(value):
        if not isinstance(item, dict):
            raise DiscoveryError(f'Field "{field}[{index}]" must be a mapping')
        allow = _as_list(item.get("platform_allow"), f"{field}[{index}].platform_allow")
        specs.append(
            DutSpec(
                platform_allow=allow,
                fixtures=instance_fixtures_from_config(item),
                raw=dict(item),
            )
        )
    return specs


def _build_instance(
    entry: Dict[str, Any],
    tests_key: str,
    testcase_dir: str,
    testsuite_root: str,
) -> TestInstance:
    rel_dir = os.path.relpath(testcase_dir, testsuite_root).replace(os.sep, "/")
    test_id = tests_key if rel_dir == "." else f"{rel_dir}/{tests_key}"

    harness_config = entry.get("harness_config") or {}
    if not isinstance(harness_config, dict):
        raise DiscoveryError(f'"{test_id}": harness_config must be a mapping')

    instance = TestInstance(
        id=test_id,
        testsuite_root=testsuite_root,
        testcase_dir=testcase_dir,
        tests_key=tests_key,
        description=str(entry.get("description", "")),
        tags=_as_list(entry.get("tags"), "tags"),
        harness=str(entry.get("harness") or "console"),
        harness_config=dict(harness_config),
        timeout=int(entry.get("timeout", 60)),
        platform_allow=_as_list(entry.get("platform_allow"), "platform_allow"),
        platform_exclude=_as_list(entry.get("platform_exclude"), "platform_exclude"),
        integration_platforms=_as_list(
            entry.get("integration_platforms"), "integration_platforms"
        ),
        depends_on=_as_list(entry.get("depends_on"), "depends_on"),
        extra_args=_as_list(entry.get("extra_args"), "extra_args"),
        min_flash=entry.get("min_flash"),
        min_ram=entry.get("min_ram"),
        slow=bool(entry.get("slow", False)),
        build_only=bool(entry.get("build_only", False)),
        skip=bool(entry.get("skip", False)),
        type=entry.get("type"),
        project_dir=entry.get("project_dir"),
        fixtures=instance_fixtures_from_config(harness_config),
        dut=_parse_dut_specs(entry.get("dut"), "dut"),
        dut_client=_parse_dut_specs(entry.get("dut_client"), "dut_client"),
    )
    return instance


def _iter_entries(raw: dict, path: str) -> Iterable[tuple]:
    """Yield ``(tests_key, entry)`` for both accepted ``tests:`` shapes."""
    tests = raw["tests"]
    if isinstance(tests, list):
        for index, entry in enumerate(tests):
            if not isinstance(entry, dict):
                raise DiscoveryError(f"{path}: tests[{index}] must be a mapping")
            yield str(entry["id"]), entry
    else:
        for key, entry in tests.items():
            if entry is None:
                entry = {}
            if not isinstance(entry, dict):
                raise DiscoveryError(f'{path}: tests."{key}" must be a mapping')
            yield str(key), entry


def parse_testcase_file(path: str, testsuite_root: str) -> List[TestInstance]:
    """Parse one ``testcase.yaml`` into instances, applying ``common:`` defaults."""
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - pyyaml is a core requirement
        raise DiscoveryError("pyyaml is required to read testcase.yaml files") from exc

    try:
        with open(path, "r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle)
    except OSError as exc:
        raise DiscoveryError(f"Cannot read {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise DiscoveryError(f"{path}: malformed YAML: {exc}") from exc

    if raw is None:
        raise DiscoveryError(f"{path}: file is empty; expected a `tests:` list")
    if not isinstance(raw, dict):
        raise DiscoveryError(f"{path}: top level must be a mapping")

    _validate(raw, path)

    common = raw.get("common") or {}
    testcase_dir = os.path.dirname(os.path.abspath(path))

    instances: List[TestInstance] = []
    seen: set = set()
    for tests_key, entry in _iter_entries(raw, path):
        if tests_key in seen:
            raise DiscoveryError(f'{path}: duplicate test id "{tests_key}"')
        seen.add(tests_key)
        merged = dict(common)
        merged.update(entry)
        instances.append(
            _build_instance(merged, tests_key, testcase_dir, testsuite_root)
        )
    return instances


def discover(testsuite_root: str, tags: Optional[Sequence[str]] = None) -> List[TestInstance]:
    """Find every test case under ``testsuite_root``, sorted by id.

    ``tags`` selects the union of the requested tags, matching Twister: a test
    is kept when it carries at least one of them.
    """
    root = os.path.abspath(testsuite_root)
    if not os.path.isdir(root):
        raise DiscoveryError(f'Test-suite root does not exist: "{root}"')

    instances: List[TestInstance] = []
    for current_dir, subdirs, filenames in os.walk(root):
        subdirs[:] = sorted(
            name
            for name in subdirs
            if not name.startswith(_SKIP_DIR_PREFIXES)
        )
        for filename in TESTCASE_FILENAMES:
            if filename in filenames:
                path = os.path.join(current_dir, filename)
                instances.extend(parse_testcase_file(path, root))

    if tags:
        wanted = set(tags)
        instances = [
            instance for instance in instances if wanted & set(instance.tags)
        ]

    instances.sort(key=lambda instance: instance.id)

    ids = [instance.id for instance in instances]
    duplicates = {test_id for test_id in ids if ids.count(test_id) > 1}
    if duplicates:
        raise DiscoveryError(
            "Duplicate test id(s) across the tree: " + ", ".join(sorted(duplicates))
        )
    return instances
