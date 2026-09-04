# -*- coding:utf-8 -*-
# SPDX-FileCopyrightText: 2025-2026 SiFli
# SPDX-License-Identifier: Apache-2.0

"""Module-level external component dependency aggregation for sf-pkg.

SDK built-in modules (middleware/*, customer/peripherals/*, ...) can declare
external components they need in a local ``sf-pkg.yaml`` placed next to the
Kconfig that gates them.  During a board build the SDK collects ``requires``
from every module whose ``enable`` symbol is active in the resolved config,
merges them with the project root ``sf-pkg.yaml`` and installs a single Conan
graph into ``<project>/sf-pkgs/``.

Discovery is bounded by the ``kconfiglist`` file that every ``InitBuild``
writes (the list of Kconfig files actually parsed for the current board), so
no recursive filesystem scan is needed and modules outside the current
chip/board config tree are never considered.

This module is importable both from the scons build (tools/build/building.py)
and from the sdk.py CLI (tools/sdk_py_actions/sf_pkg_ext.py).  Heavy third
party libs (PyYAML, semantic_version) are imported lazily.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
import sys
from typing import Dict
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple

MANIFEST_FILENAME = "sf-pkg.yaml"
CONSUMER_DIR = ".sf-pkg"
CONSUMER_CONANFILE = os.path.join(CONSUMER_DIR, "conanfile.py")
INSTALLED_FINGERPRINT = os.path.join(CONSUMER_DIR, "installed.fingerprint")
SF_PKGS_DIR = "sf-pkgs"
SF_PKG_REMOTE_NAME = "artifactory"
SF_PKG_REMOTE_URL = "https://jfrog.sifli.com/artifactory/api/conan/conan-local"
SDK_VERSION_ENV = "SIFLI_SDK_VERSION"

# Optional component (external package) that participates in the build is
# picked up by building.py through these generated files.
SCONSCRIPT_CONANDEPS = os.path.join(SF_PKGS_DIR, "SConscript_conandeps")

_SDK_ENV_OFFLINE = "SIFLI_SF_PKG_OFFLINE"


class SfPkgError(Exception):
    """Raised when module-level dependency aggregation cannot proceed."""


# ---------------------------------------------------------------------------
# Manifest model
# ---------------------------------------------------------------------------

def _is_truthy_config(value: Optional[str]) -> bool:
    if value is None:
        return False
    return str(value).strip().lower() not in ("", "n", "0", "no", "false")


def _yaml():
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - env always has it
        raise SfPkgError(
            "PyYAML is required to read sf-pkg.yaml manifests."
        ) from exc
    return yaml


def load_manifest(path: str) -> Dict:
    """Parse one sf-pkg.yaml into a normalized dict."""
    yaml = _yaml()
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    try:
        doc = yaml.safe_load(text) or {}
    except Exception as exc:
        raise SfPkgError("Failed to parse manifest '{}': {}".format(path, exc))

    if not isinstance(doc, dict):
        raise SfPkgError("Manifest '{}' must be a YAML mapping".format(path))

    requires = _as_str_list(doc.get("requires"), "requires", path)
    enable = _as_str_list(doc.get("enable"), "enable", path)
    support = doc.get("support_sdk_version")
    if support is not None and not isinstance(support, str):
        raise SfPkgError(
            "Manifest '{}': support_sdk_version must be a string".format(path)
        )
    return {
        "path": path,
        "dir": os.path.dirname(path),
        "enable": enable,
        "requires": requires,
        "support_sdk_version": (support or "").strip() or None,
    }


def _as_str_list(value, field, path) -> List[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [value.strip()] if value.strip() else []
    if not isinstance(value, (list, tuple)):
        raise SfPkgError(
            "Manifest '{}': '{}' must be a list of strings".format(path, field)
        )
    out = []
    for item in value:
        if not isinstance(item, str) or not item.strip():
            raise SfPkgError(
                "Manifest '{}': '{}' entries must be non-empty strings".format(
                    path, field
                )
            )
        out.append(item.strip())
    return out


def detect_mode(project_dir: str) -> str:
    """Return 'yaml', 'advanced' or 'none' for a project root.

    'yaml'     -> project uses sf-pkg.yaml (module aggregation enabled)
    'advanced' -> project keeps a hand-written conanfile.py (legacy escape
                  hatch: no module aggregation, install straight from it)
    'none'     -> project has no external component declaration
    """
    if os.path.isfile(os.path.join(project_dir, MANIFEST_FILENAME)):
        return "yaml"
    if os.path.isfile(os.path.join(project_dir, "conanfile.py")):
        return "advanced"
    return "none"


def load_root_manifest(project_dir: str) -> Optional[Dict]:
    """Load the project root sf-pkg.yaml or None when absent."""
    path = os.path.join(project_dir, MANIFEST_FILENAME)
    if not os.path.isfile(path):
        return None
    return load_manifest(path)


def manifests_from_kconfiglist(kconfiglist_path: str) -> List[Dict]:
    """Discover candidate manifests by looking next to parsed Kconfig files.

    ``kconfiglist`` holds every Kconfig file actually parsed for the current
    board.  A manifest only lives next to the Kconfig that gates its module,
    so checking the sibling of each listed file gives exactly the modules
    inside the current chip/board config tree.
    """
    if not os.path.isfile(kconfiglist_path):
        return []
    manifests: List[Dict] = []
    seen_dirs = set()
    try:
        with open(kconfiglist_path, "r", encoding="utf-8") as f:
            lines = f.read().splitlines()
    except OSError:
        return []

    for line in lines:
        line = line.strip()
        if not line:
            continue
        cfg_dir = os.path.dirname(line)
        if cfg_dir in seen_dirs:
            continue
        mf = os.path.join(cfg_dir, MANIFEST_FILENAME)
        if os.path.isfile(mf):
            seen_dirs.add(cfg_dir)
            manifests.append(load_manifest(mf))
    return manifests


def load_resolved_config(config_path: str) -> Dict[str, str]:
    """Parse a kconfiglib generated ``.config`` into {symbol: value}.

    kconfig's config writer prepends a single ``CONFIG_`` to every symbol when
    writing, so e.g. ``CONFIG_FOO=y`` is stored under ``FOO`` (that one added
    prefix stripped).  A symbol whose own name already starts with ``CONFIG_``
    is written as ``CONFIG_CONFIG_xxx`` and stored under its real name
    ``CONFIG_xxx``.
    """
    values: Dict[str, str] = {}
    if not os.path.isfile(config_path):
        return values
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            for raw in f:
                line = raw.strip()
                if not line:
                    continue
                if line.startswith("# ") and line.endswith(" is not set"):
                    name = line[2:-len(" is not set")].strip()
                    values[_strip_one_config_prefix(name)] = "n"
                    continue
                if "=" in line and not line.startswith("#"):
                    key, _, val = line.partition("=")
                    values[_strip_one_config_prefix(key.strip())] = val.strip()
    except OSError:
        return values
    return values


def _strip_one_config_prefix(name: str) -> str:
    return name[len("CONFIG_"):] if name.startswith("CONFIG_") else name


def config_symbol_enabled(config: Dict[str, str], symbol: str) -> bool:
    """Whether a Kconfig symbol resolves to y in the given config.

    ``symbol`` is the real Kconfig symbol name as defined in Kconfig, which is
    exactly the key used by load_resolved_config (kconfig adds a single
    CONFIG_ prefix only when writing the file).  No prefix guessing here.
    """
    return _is_truthy_config(config.get(symbol))


def manifest_enabled(manifest: Dict, config: Dict[str, str]) -> bool:
    """A manifest participates when any of its enable symbols is active."""
    enable = manifest.get("enable") or []
    if not enable:
        return True
    return any(config_symbol_enabled(config, sym) for sym in enable)


def collect_requires(manifests: Sequence[Dict], config: Dict[str, str]) -> List[str]:
    """requires of every manifest that participates under the given config."""
    return [
        req
        for manifest in manifests
        if manifest_enabled(manifest, config)
        for req in (manifest.get("requires") or [])
    ]


# ---------------------------------------------------------------------------
# SDK version check (mirrors sf-pkg-base.SourceOnlyBase.validate)
# ---------------------------------------------------------------------------

def sdk_version_from_env() -> Optional[str]:
    return os.environ.get(SDK_VERSION_ENV) or None


def check_sdk_versions(
    sdk_ver: Optional[str],
    manifests: Sequence[Dict],
) -> None:
    """Validate support_sdk_version of every manifest against the SDK.

    Mirrors ``sf-pkg-base.SourceOnlyBase.validate()``: semantic_version
    Version.coerce on the env SDK version and NpmSpec on the declared range.
    """
    constrained = [
        m for m in manifests if (m.get("support_sdk_version") or "").strip()
    ]
    if not constrained:
        return

    if not sdk_ver:
        raise SfPkgError(
            "Environment variable '{}' is not set, but sf-pkg.yaml manifest(s) "
            "declare support_sdk_version. It must contain the current SDK "
            "semver, e.g. '2.4' or '2.4.1'.".format(SDK_VERSION_ENV)
        )

    try:
        import semantic_version
    except ImportError as exc:  # pragma: no cover
        raise SfPkgError(
            "semantic_version is required to check support_sdk_version."
        ) from exc

    try:
        sdk_version = semantic_version.Version.coerce(sdk_ver)
    except ValueError as exc:
        raise SfPkgError(
            "Invalid SDK version '{}' in environment variable '{}'. It must be "
            "a valid semantic version like '2.4' or '2.4.1'.".format(
                sdk_ver, SDK_VERSION_ENV
            )
        ) from exc

    for manifest in constrained:
        constraint = manifest["support_sdk_version"]
        source = manifest.get("path", "<manifest>")
        try:
            spec = semantic_version.NpmSpec(constraint)
        except ValueError as exc:
            raise SfPkgError(
                "Invalid support_sdk_version expression '{}' in '{}'. It must "
                "be a valid npm-style semver range (e.g. '^2.4', '~2.4.1', "
                "'>=" "2.4,<3').".format(constraint, source)
            ) from exc
        if sdk_version not in spec:
            raise SfPkgError(
                "'{}' supports SDK versions '{}', but current SDK is '{}' "
                "(from environment '{}').".format(
                    source, constraint, sdk_ver, SDK_VERSION_ENV
                )
            )


# ---------------------------------------------------------------------------
# Aggregation
# ---------------------------------------------------------------------------

def aggregate_requires(*groups: Sequence[str]) -> List[str]:
    """Stable-order, deduplicated union of requires."""
    out: List[str] = []
    seen = set()
    for group in groups:
        for req in group or []:
            if req not in seen:
                seen.add(req)
                out.append(req)
    return out


# ---------------------------------------------------------------------------
# Consumer conanfile generation
# ---------------------------------------------------------------------------

# Fallback only used when `conan new sf-pkg-project` (the canonical, versioned
# project template shipped with the conan config bundle) cannot run.  Keeping
# this baked in means it may lag template updates, which is acceptable only as
# a last resort.
_FALLBACK_CONSUMER_TEMPLATE = """\
from conan import ConanFile

class SfPkgConsumer(ConanFile):
    name = "sf-pkg-consumer"
    version = "0.0.1"
    package_type = "application"

    # Produce the sf-pkgs/SConscript_conandeps and sf-pkgs/Kconfig.conandeps
    # files that the scons build consumes.
    generators = ("SConsDeps", "KconfigDeps")

    requires = (
{requires}
    )
"""


def _bootstrap_consumer_from_template(consumer_dir: str) -> bool:
    """Generate .sf-pkg/consumer/conanfile.py from the canonical template.

    Uses the same ``conan new sf-pkg-project`` mechanism as ``sf-pkg init`` so
    the generated file follows template updates shipped with the conan config
    bundle.  Returns False when conan/the template is unavailable.
    """
    import shutil

    dest = os.path.join(consumer_dir, "conanfile.py")
    if os.path.isfile(dest):
        os.remove(dest)
    try:
        result = subprocess.run(
            [
                "conan",
                "new",
                "sf-pkg-project",
                "-d",
                "name=sf-pkg-consumer",
                "-d",
                "version=0.0.1",
            ],
            cwd=consumer_dir,
            capture_output=True,
            text=True,
        )
    except OSError:
        return False
    return result.returncode == 0 and os.path.isfile(dest)


def _replace_requires(content: str, requires: Sequence[str]) -> Optional[str]:
    """Splice aggregated requires into the template's requires attribute.

    Only the ``requires = (... )`` literal is touched so the rest of the file
    keeps following the template.  Returns None when the block cannot be found
    (template changed shape) so the caller can fall back.
    """
    m = re.search(r"^(\s*)requires\s*=\s*\(", content, re.M)
    if not m:
        return None
    line_lead = m.group(1)
    start = m.start()
    open_idx = m.end() - 1  # index of '('

    depth = 0
    in_str = False
    i = open_idx
    n = len(content)
    close = -1
    while i < n:
        ch = content[i]
        if in_str:
            if ch == "\\":
                i += 2
                continue
            if ch == '"':
                in_str = False
        else:
            if ch == '"':
                in_str = True
            elif ch == "#":
                nl = content.find("\n", i)
                if nl < 0:
                    break
                i = nl
            elif ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
                if depth == 0:
                    close = i
                    break
        i += 1
    if close < 0:
        return None

    remainder = content[close + 1:]
    if remainder.startswith("\n"):
        remainder = remainder[1:]

    lines = []
    for req in requires:
        lines.append("{}    \"{}\",".format(line_lead, req))
    block = "{}requires = (\n{}\n{})".format(
        line_lead, "\n".join(lines), line_lead
    )
    return content[:start] + block + "\n" + remainder


def _replace_support_sdk_version(content: str, value: Optional[str]) -> str:
    """Rewrite the template's support_sdk_version to the declared value.

    Keeps the generated consumer file consistent with the project root
    sf-pkg.yaml.  No-op when the value is absent (template default is kept) or
    the attribute does not exist in the template output.
    """
    if not value or not value.strip():
        return content
    pattern = re.compile(
        r'^(\s*support_sdk_version\s*=\s*)(?:"[^"]*"|\'[^\']*\')', re.M
    )
    return pattern.sub(
        lambda m: m.group(1) + json.dumps(value.strip()), content
    )


def generate_consumer_conanfile(
    project_dir: str,
    merged_requires: Sequence[str],
) -> str:
    """Write the aggregated consumer conanfile and return its path.

    The base file is produced from the versioned ``sf-pkg-project`` conan
    template (so it tracks template updates); only the requires list is
    injected.  Falls back to an internal template when conan is unavailable.
    """
    dest = os.path.join(project_dir, CONSUMER_CONANFILE)
    consumer_dir = os.path.dirname(dest)
    os.makedirs(consumer_dir, exist_ok=True)
    root_manifest = load_root_manifest(project_dir)
    root_support = root_manifest.get("support_sdk_version") if root_manifest else None

    content = None
    if _bootstrap_consumer_from_template(consumer_dir):
        with open(dest, "r", encoding="utf-8") as f:
            raw = f.read()
        content = _replace_requires(raw, merged_requires)
        if content is None:
            print(
                "[sf-pkg] Warning: could not inject requires into the "
                "sf-pkg-project template output; using built-in fallback."
            )
    if content is None:
        requires_lines = "".join(
            '        "{}",\n'.format(req) for req in merged_requires
        )
        content = _FALLBACK_CONSUMER_TEMPLATE.format(
            requires=requires_lines
        )
    content = _replace_support_sdk_version(content, root_support)

    with open(dest, "w", encoding="utf-8", newline="\n") as f:
        f.write(content)
    return dest


# ---------------------------------------------------------------------------
# Fingerprint / install state
# ---------------------------------------------------------------------------

def fingerprint(merged_requires: Sequence[str], sdk_ver: Optional[str]) -> str:
    payload = json.dumps(
        {"sdk_version": sdk_ver, "requires": list(merged_requires)},
        sort_keys=True,
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def installed_fingerprint_path(project_dir: str) -> str:
    return os.path.join(project_dir, INSTALLED_FINGERPRINT)


def read_installed_fingerprint(project_dir: str) -> Optional[str]:
    path = installed_fingerprint_path(project_dir)
    try:
        with open(path, "r", encoding="utf-8") as f:
            return f.read().strip() or None
    except OSError:
        return None


def write_installed_fingerprint(project_dir: str, fp: str) -> None:
    path = installed_fingerprint_path(project_dir)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(fp)


def wipe_sf_pkgs(project_dir: str) -> None:
    target = os.path.join(project_dir, SF_PKGS_DIR)
    if os.path.isdir(target):
        shutil_rmtree(target)


def shutil_rmtree(path: str) -> None:
    # Deferred import keeps module light when unused (e.g. help).
    import shutil

    shutil.rmtree(path, ignore_errors=True)


# ---------------------------------------------------------------------------
# Conan invocation
# ---------------------------------------------------------------------------

def _run(cmd: Sequence[str], cwd: str) -> int:
    """Stream a command to the console; returns its return code."""
    return subprocess.call(list(cmd), cwd=cwd)


def ensure_public_remote(project_dir: str) -> None:
    """Make sure the shared 'artifactory' remote is registered (idempotent).

    Only adds when missing or pointing elsewhere so existing credentials are
    not clobbered.
    """
    try:
        result = subprocess.run(
            ["conan", "remote", "list", "-f", "json"],
            cwd=project_dir,
            capture_output=True,
            text=True,
        )
    except OSError:
        raise SfPkgError("'conan' not found on PATH. Run export.ps1 / export.sh first.")
    if result.returncode != 0:
        return  # let conan report remote problems on install

    remotes = []
    try:
        remotes = json.loads(result.stdout or "[]")
    except json.JSONDecodeError:
        remotes = []

    existing = None
    for item in remotes if isinstance(remotes, list) else []:
        if isinstance(item, dict) and item.get("name") == SF_PKG_REMOTE_NAME:
            existing = item
            break

    url = str(existing.get("url") or "").rstrip("/") if existing else ""
    if url == SF_PKG_REMOTE_URL.rstrip("/"):
        return
    _run(
        ["conan", "remote", "add", SF_PKG_REMOTE_NAME, SF_PKG_REMOTE_URL, "--force"],
        project_dir,
    )


def install(
    project_dir: str,
    consumer_conanfile: str,
) -> None:
    """Install the aggregated consumer conanfile into <project>/sf-pkgs."""
    ensure_public_remote(project_dir)
    cmd = [
        "conan",
        "install",
        consumer_conanfile,
        "--output-folder={}".format(os.path.join(project_dir, SF_PKGS_DIR)),
        "--deployer=full_deploy",
        "--envs-generation=false",
        "-r={}".format(SF_PKG_REMOTE_NAME),
    ]
    rc = _run(cmd, project_dir)
    if rc != 0:
        raise SfPkgError(
            "conan install failed. Check sf-pkg login / network and retry."
        )


# ---------------------------------------------------------------------------
# End-to-end orchestration used by the scons build
# ---------------------------------------------------------------------------

def offline_enabled() -> bool:
    return os.environ.get(_SDK_ENV_OFFLINE, "").strip().lower() in ("1", "true", "yes")


def normalize_board_name(board: str) -> str:
    """Default an unspecified core to HCPU, mirroring the scons board names."""
    if board and not any(
        suffix in board for suffix in ("_hcpu", "_lcpu", "_acpu")
    ):
        return board + "_hcpu"
    return board


def _import_building():
    """Lazily import the scons build module (tools/build/building.py)."""
    sdk_root = os.environ.get("SIFLI_SDK")
    if not sdk_root:
        raise SfPkgError(
            "SIFLI_SDK is not set. Run export.ps1 / export.sh first."
        )
    for d in (
        os.path.join(sdk_root, "tools", "build"),
        os.path.join(sdk_root, "tools"),
    ):
        if d not in sys.path:
            sys.path.insert(0, d)
    try:
        import building
    except Exception as exc:  # pragma: no cover
        raise SfPkgError(
            "Failed to import the SDK build module (building). If this keeps "
            "failing, run a scons build once and retry."
        ) from exc
    return building


def ensure_board_config(project_dir: str, board: str) -> str:
    """Non-interactively resolve board.conf + proj.conf for a board.

    Reuses ``building.ResolveBoardConfig`` (the same implementation the scons
    InitBuild uses) to generate ``<project>/build_<board>/`` (kconfiglist,
    .config, rtconfig.h), so ``sf-pkg install --board`` does not require a
    prior compilation. Returns the absolute build dir.
    """
    board = normalize_board_name(board)
    building = _import_building()
    # Honor a custom board search path (mirrors the scons --board_search_path
    # option and the SIFLI_SDK_BOARD_SEARCH_PATH environment variable).
    search = os.environ.get("SIFLI_SDK_BOARD_SEARCH_PATH")
    if search:
        building.BOARD_SEARCH_PATH = os.path.abspath(search)
    parent_dir, core_dir = building.GetBoardPath(board)
    if not (os.path.isdir(parent_dir) and os.path.isdir(core_dir)):
        raise SfPkgError(
            "Board '{}' was not found (expected {}/Kconfig.board + board.conf "
            "under customer/boards).".format(
                board, os.path.basename(core_dir)
            )
        )
    # building.LoadRtconfig does a plain `import rtconfig`, which must resolve
    # to the project's rtconfig.py.
    if project_dir not in sys.path:
        sys.path.insert(0, project_dir)
    sys.modules.pop("rtconfig", None)

    building.LoadRtconfig(board)
    import rtconfig  # synced by LoadRtconfig to the board's settings

    output_dir = os.path.abspath(
        os.path.join(project_dir, rtconfig.OUTPUT_DIR)  # e.g. 'build_<board>/'
    )
    building.ResolveBoardConfig(project_dir, output_dir, board)
    return output_dir


def resolve_for_board(
    project_dir: str,
    build_dir: Optional[str],
) -> Tuple[List[Dict], Dict[str, str], Dict]:
    """Compute (enabled module manifests, config, root manifest) for a build."""
    config: Dict[str, str] = {}
    if build_dir:
        config = load_resolved_config(os.path.join(build_dir, ".config"))
        manifests = manifests_from_kconfiglist(os.path.join(build_dir, "kconfiglist"))
    else:
        manifests = []
    root = load_root_manifest(project_dir)
    # The project root sf-pkg.yaml is handled separately as `root`; skip it
    # when discovering module manifests so it is not double-counted as a
    # "module" (its directory also holds parsed project Kconfig files).
    project_root = os.path.normpath(project_dir)
    enabled = [
        m
        for m in manifests
        if os.path.normpath(m["dir"]) != project_root
        and manifest_enabled(m, config)
    ]
    return enabled, config, root


def ensure_board_deps(project_dir: str, build_dir: Optional[str] = None) -> bool:
    """Auto install external deps required by modules enabled in this board.

    Returns True when a (re)install happened, in which case the caller must
    regenerate the board config (package Kconfigs may expose symbols that the
    modules ``select``).
    """
    mode = detect_mode(project_dir)
    if mode != "yaml":
        return False

    enabled, config, root = resolve_for_board(project_dir, build_dir)
    root_requires = list(root.get("requires") or []) if root else []

    manifests_to_check = list(enabled)
    if root:
        manifests_to_check = [root] + manifests_to_check
    check_sdk_versions(sdk_version_from_env(), manifests_to_check)

    merged = aggregate_requires(root_requires, collect_requires(enabled, config))
    sdk_ver = sdk_version_from_env()
    fp = fingerprint(merged, sdk_ver)

    if not merged:
        # Nothing to install.  In yaml mode the sf-pkgs/ dir belongs to this
        # aggregation, so clear it unconditionally: stale deployed packages
        # (from a previous non-empty set, an advanced-mode conanfile, or an
        # older flow) must not linger and get linked into the image.  Drop the
        # stale generated consumer conanfile too (keep installed.fingerprint).
        wipe_sf_pkgs(project_dir)
        consumer_file = os.path.join(project_dir, CONSUMER_CONANFILE)
        if os.path.isfile(consumer_file):
            os.remove(consumer_file)
        write_installed_fingerprint(project_dir, "")
        return False

    if fp == read_installed_fingerprint(project_dir):
        return False

    if offline_enabled():
        print(
            "[sf-pkg] {} requires external components, but SIFLI_SF_PKG_OFFLINE "
            "is set; skipping install.".format(os.path.basename(project_dir))
        )
        return False

    print(
        "[sf-pkg] Installing {} external component(s) contributed by "
        "{} module(s) into sf-pkgs/ ...".format(len(merged), len(enabled))
    )
    wipe_sf_pkgs(project_dir)
    consumer = generate_consumer_conanfile(project_dir, merged)
    install(project_dir, consumer)
    write_installed_fingerprint(project_dir, fp)
    print("[sf-pkg] External components installed.")
    return True
