# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import json
import os
import tempfile
import tomllib
from dataclasses import dataclass
from typing import Any
from typing import Dict
from typing import Sequence

import tomli_w
from packaging.requirements import InvalidRequirement
from packaging.requirements import Requirement
from packaging.utils import canonicalize_name

from sdk_py_actions.cli.context import SdkContext
from sdk_py_actions.cli.registry import CommandRegistry
from sdk_py_actions.errors import FatalError
from sdk_py_actions.errors import UsageError

EXTENSION_ID = "python-deps"
EXTENSION_VERSION = "1.0.0"
EXTENSION_API_VERSION = 2
MIN_SDK_VERSION = None

DEFAULT_PROFILE = "default"
DEFAULT_PYPI_INDEX = "https://pypi.org/simple"


@dataclass(frozen=True)
class LockProjectPaths:
    profile: str
    profile_dir: str
    lock_json_path: str
    pyproject_path: str
    uv_lock_path: str


def _build_canonical_uv_lock_command(project_dir: str) -> list[str]:
    return [
        "uv",
        "lock",
        "--project",
        project_dir,
        "--default-index",
        DEFAULT_PYPI_INDEX,
        "--index-strategy",
        "first-index",
    ]


def _sdk_root(sdk_ctx: SdkContext) -> str:
    root = sdk_ctx.env.get("SIFLI_SDK_PATH") or os.environ.get("SIFLI_SDK_PATH")
    if not root:
        raise FatalError("SIFLI_SDK_PATH is not set. Please run sdk.py from an exported SDK environment.")
    return os.path.realpath(root)


def _install_command_hint(profile: str) -> str:
    if os.name == "nt":
        return f".\\install.ps1 --profile {profile}"
    return f"./install.sh --profile {profile}"


def _profile_lock_paths(sdk_root: str, profile: str) -> LockProjectPaths:
    profile_dir = os.path.join(sdk_root, "tools", "locks", profile)
    lock_json_path = os.path.join(profile_dir, "lock.json")
    pyproject_path = os.path.join(profile_dir, "pyproject.toml")
    uv_lock_path = os.path.join(profile_dir, "uv.lock")

    for path in [lock_json_path, pyproject_path, uv_lock_path]:
        if not os.path.isfile(path):
            raise UsageError(f"Profile '{profile}' is missing required file: {path}")

    with open(lock_json_path, "rb") as f:
        lock_doc = json.load(f)

    python_cfg = lock_doc.get("python")
    if not isinstance(python_cfg, dict):
        raise FatalError(f"Profile lock is missing python configuration: {lock_json_path}")

    expected_project_dir = os.path.normpath(os.path.join("tools", "locks", profile))
    expected_lock_file = os.path.normpath(os.path.join(expected_project_dir, "uv.lock"))
    actual_project_dir = os.path.normpath(str(python_cfg.get("project_dir", "")))
    actual_lock_file = os.path.normpath(str(python_cfg.get("lock_file", "")))
    if actual_project_dir != expected_project_dir:
        raise FatalError(
            f"Profile lock {lock_json_path} points python.project_dir to '{python_cfg.get('project_dir')}', "
            f"expected '{expected_project_dir}'."
        )
    if actual_lock_file != expected_lock_file:
        raise FatalError(
            f"Profile lock {lock_json_path} points python.lock_file to '{python_cfg.get('lock_file')}', "
            f"expected '{expected_lock_file}'."
        )

    return LockProjectPaths(
        profile=profile,
        profile_dir=profile_dir,
        lock_json_path=lock_json_path,
        pyproject_path=pyproject_path,
        uv_lock_path=uv_lock_path,
    )


def _parse_requirement_name(raw_requirement: str) -> str:
    try:
        return canonicalize_name(Requirement(raw_requirement).name)
    except InvalidRequirement as exc:
        raise UsageError(f'Invalid Python requirement "{raw_requirement}": {exc}') from exc


def _existing_dependency_map(dependencies: Sequence[str]) -> Dict[str, str]:
    existing: Dict[str, str] = {}
    for item in dependencies:
        if not isinstance(item, str):
            raise FatalError("project.dependencies must be a list of strings")
        normalized_name = _parse_requirement_name(item)
        existing[normalized_name] = item
    return existing


def _new_dependency_map(requirements: Sequence[str]) -> Dict[str, str]:
    if not requirements:
        raise UsageError("At least one Python requirement is required.")

    normalized: Dict[str, str] = {}
    for raw_requirement in requirements:
        normalized_name = _parse_requirement_name(raw_requirement)
        if normalized_name in normalized:
            raise UsageError(
                f'Python requirement "{raw_requirement}" duplicates "{normalized[normalized_name]}" in the same command.'
            )
        normalized[normalized_name] = raw_requirement
    return normalized


def _load_pyproject(pyproject_path: str) -> Dict[str, Any]:
    with open(pyproject_path, "rb") as f:
        pyproject_doc = tomllib.load(f)

    project = pyproject_doc.get("project")
    if not isinstance(project, dict):
        raise FatalError(f"{pyproject_path} is missing the [project] table.")

    dependencies = project.get("dependencies")
    if not isinstance(dependencies, list):
        raise FatalError(f"{pyproject_path} is missing [project].dependencies.")

    for item in dependencies:
        if not isinstance(item, str):
            raise FatalError(f"{pyproject_path} contains a non-string dependency entry.")

    return pyproject_doc


def _atomic_write_toml(path: str, payload: Dict[str, Any]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(prefix=".tmp-", dir=os.path.dirname(path))
    try:
        with os.fdopen(fd, "wb") as f:
            tomli_w.dump(payload, f)
        os.replace(tmp_path, path)
    except Exception:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass
        raise


def _atomic_write_bytes(path: str, content: bytes) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(prefix=".tmp-", dir=os.path.dirname(path))
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(content)
        os.replace(tmp_path, path)
    except Exception:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass
        raise


def add_callback(
    sdk_ctx: SdkContext,
    requirements: Sequence[str],
    profile: str = DEFAULT_PROFILE,
) -> None:
    sdk_root = _sdk_root(sdk_ctx)
    paths = _profile_lock_paths(sdk_root, profile)
    pyproject_doc = _load_pyproject(paths.pyproject_path)
    project = pyproject_doc["project"]
    dependencies = list(project["dependencies"])

    new_dependencies = _new_dependency_map(requirements)
    existing_dependencies = _existing_dependency_map(dependencies)
    for normalized_name, raw_requirement in new_dependencies.items():
        if normalized_name in existing_dependencies:
            raise UsageError(
                f'Python dependency "{existing_dependencies[normalized_name]}" already exists in profile "{profile}". '
                f'"add" only supports new dependencies, so "{raw_requirement}" was not applied.'
            )

    original_pyproject = b""
    original_uv_lock = b""
    with open(paths.pyproject_path, "rb") as f:
        original_pyproject = f.read()
    with open(paths.uv_lock_path, "rb") as f:
        original_uv_lock = f.read()

    project["dependencies"] = dependencies + list(requirements)

    try:
        _atomic_write_toml(paths.pyproject_path, pyproject_doc)
        sdk_ctx.runner.run(
            _build_canonical_uv_lock_command(paths.profile_dir),
            cwd=sdk_root,
        )
    except Exception:
        _atomic_write_bytes(paths.pyproject_path, original_pyproject)
        _atomic_write_bytes(paths.uv_lock_path, original_uv_lock)
        raise

    print(f"Added Python dependencies to profile '{profile}':")
    for requirement in requirements:
        print(f"  - {requirement}")
    print(
        f"Updated tools/locks/{profile}/pyproject.toml and tools/locks/{profile}/uv.lock. "
        f"Re-run `{_install_command_hint(profile)}` and export again to refresh the SDK environment."
    )


def register(registry: CommandRegistry) -> None:
    registry.group(path="python-deps", help="Manage SDK Python dependency locks.")
    registry.command(
        path="python-deps/add",
        callback=add_callback,
        help="Add one or more Python dependencies to the selected SDK profile and refresh uv.lock.",
        options=[
            {
                "names": ["--profile"],
                "help": "SDK profile to update.",
                "default": DEFAULT_PROFILE,
            },
        ],
        arguments=[
            {
                "names": ["requirements"],
                "required": True,
                "nargs": -1,
            }
        ],
    )
