#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 SiFli
#
# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import uuid
import urllib.error
import urllib.request
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime
from datetime import timezone
from typing import Any
from typing import Dict
from typing import List
from typing import MutableMapping
from typing import Optional
from typing import Sequence
from typing import Tuple

import tomllib
import tomli_w

import sifli_sdk_tools as legacy_tools
from rich.console import Console
from rich.panel import Panel
from rich.progress import BarColumn
from rich.progress import DownloadColumn
from rich.progress import Progress
from rich.progress import SpinnerColumn
from rich.progress import TaskProgressColumn
from rich.progress import TextColumn
from rich.progress import TimeRemainingColumn
from rich.progress import TransferSpeedColumn
from rich.text import Text
from rich.theme import Theme


DEFAULT_PROFILE = "default"
DEFAULT_INSTALL_ROOT = os.path.expanduser("~/.sifli")
ENV_COMPAT_ALGORITHM = "v1"
STATE_SCHEMA_VERSION = 2
LOCK_SCHEMA_VERSION = 1
DEFAULT_PYPI_INDEX = "https://pypi.org/simple"
DEFAULT_PYPI_FILES_PREFIX = "https://files.pythonhosted.org/packages"
DEFAULT_UPSTREAM_URL = "https://downloads.sifli.com/dl/sifli-sdk"
CONFIG_FILE_NAME = "config.json"
STATE_FILE_NAME = "sifli-sdk-env.json"
TOOLCHAIN_GCC = "gcc"
TOOLCHAIN_KEIL = "keil"
SUPPORTED_TOOLCHAINS = (TOOLCHAIN_GCC, TOOLCHAIN_KEIL)
GCC_TOOL_NAME = "arm-none-eabi-gcc"
CHINA_MIRROR_ENV_NAME = "SIFLI_SDK_MIRROR_CHINA"
CHINA_MIRROR_PRESET = {
    "SIFLI_SDK_GITHUB_ASSETS": "https://downloads.sifli.com/github_assets",
    "SIFLI_SDK_PYPI_DEFAULT_INDEX": "https://mirrors.ustc.edu.cn/pypi/simple",
    "UV_PYTHON_DOWNLOADS_JSON_URL": "https://uv.agentsmirror.com/metadata/python-downloads.json",
    "UV_PYPY_INSTALL_MIRROR": "https://uv.agentsmirror.com/pypy",
}

POSIX_SHELLS = {"bash", "zsh", "sh"}
POWERSHELL_SHELLS = {"powershell", "pwsh"}


class SDKEnvError(RuntimeError):
    pass


RICH_THEME = Theme(
    {
        "sdk.step": "bold cyan",
        "sdk.info": "bright_blue",
        "sdk.ok": "bold green",
        "sdk.warn": "bold yellow",
        "sdk.err": "bold red",
        "sdk.key": "bold white",
        "sdk.dim": "bright_black",
        "sdk.title": "bold cyan",
        "sdk.sub": "bright_black",
    }
)
RICH_CONSOLE = Console(stderr=True, theme=RICH_THEME, highlight=False, soft_wrap=True)


def log_stream() -> Any:
    return sys.stderr


def use_color() -> bool:
    if os.environ.get("NO_COLOR"):
        return False
    return hasattr(log_stream(), "isatty") and log_stream().isatty()


def verbose_logs() -> bool:
    return parse_bool(os.environ.get("SIFLI_SDK_ENV_DEBUG"), default=False)


def log_line(label: str, message: str, color: str = "", *, bold: bool = False) -> None:
    label_style = {
        "[STEP]": "sdk.step",
        "[INFO]": "sdk.info",
        "[ OK ]": "sdk.ok",
        "[WARN]": "sdk.warn",
        "[ERR ]": "sdk.err",
    }.get(label, "")
    RICH_CONSOLE.print(f"[{label_style}]{label}[/] {message}")


def log_step(message: str) -> None:
    RICH_CONSOLE.print(f"[sdk.step]*[/] {message}")


def log_done(message: str, result: str = "OK", result_style: str = "sdk.ok") -> None:
    RICH_CONSOLE.print(f"[sdk.step]*[/] {message} [sdk.dim]...[/] [{result_style}]{result}[/]")


def log_value(message: str, value: Any, value_style: str = "sdk.info") -> None:
    RICH_CONSOLE.print(f"[sdk.step]*[/] {message} [sdk.dim]...[/] [{value_style}]{value}[/]")


def log_info(message: str) -> None:
    log_line("[INFO]", message)


def log_ok(message: str) -> None:
    log_line("[ OK ]", message, bold=True)


def log_warn(message: str) -> None:
    log_line("[WARN]", message, bold=True)


def log_error(message: str) -> None:
    log_line("[ERR ]", message, bold=True)


def log_kv(key: str, value: Any) -> None:
    if not verbose_logs():
        return
    RICH_CONSOLE.print(f"    [sdk.key]{key}:[/] {value}")


def log_banner(title: str, subtitle: Optional[str] = None) -> None:
    body = Text()
    body.append(title, style="sdk.title")
    if subtitle:
        body.append("\n")
        body.append(subtitle, style="sdk.sub")
    RICH_CONSOLE.print(Panel.fit(body, border_style="sdk.step", padding=(0, 1)))


@dataclass(frozen=True)
class ProfileLock:
    path: str
    schema_version: int
    profile: str
    python_version: str
    python_project_dir: str
    python_lock_file: str
    default_targets: List[str]
    tools: Dict[str, str]
    path_order: List[str]
    conan_config_id: str
    conan_remote_name: str
    conan_remote_url: str
    conan_home_subdir: str

    @property
    def pyproject_path(self) -> str:
        return os.path.join(repo_root(), self.python_project_dir, "pyproject.toml")

    @property
    def uv_lock_path(self) -> str:
        return os.path.join(repo_root(), self.python_lock_file)

    @classmethod
    def load(cls, root: str, profile: str) -> "ProfileLock":
        lock_path = os.path.join(root, "tools", "locks", profile, "lock.json")
        try:
            with open(lock_path, "rb") as f:
                raw = json.load(f)
        except FileNotFoundError as exc:
            raise SDKEnvError(f"profile lock not found: {lock_path}") from exc

        try:
            python_cfg = raw["python"]
            defaults = raw["defaults"]
            conan_cfg = raw["conan"]
            tools_cfg = raw["tools"]
            path_order = raw["path_order"]
        except KeyError as exc:
            raise SDKEnvError(f"profile lock is missing required field: {exc}") from exc

        schema_version = int(raw.get("schema_version", 0))
        if schema_version != LOCK_SCHEMA_VERSION:
            raise SDKEnvError(
                f"unsupported lock schema_version {schema_version}, expected {LOCK_SCHEMA_VERSION}"
            )

        loaded_profile = str(raw.get("profile", "")).strip()
        if loaded_profile != profile:
            raise SDKEnvError(
                f"profile lock {lock_path} declares profile '{loaded_profile}', expected '{profile}'"
            )

        default_targets = [normalize_target(t) for t in defaults.get("targets", [])]
        if not default_targets:
            raise SDKEnvError(f"profile lock {lock_path} has no default targets")

        if not isinstance(tools_cfg, dict) or not tools_cfg:
            raise SDKEnvError(f"profile lock {lock_path} has no tool version bindings")

        normalized_tools = {str(name): str(version) for name, version in tools_cfg.items()}
        normalized_path_order = [str(name) for name in path_order]

        loaded = cls(
            path=lock_path,
            schema_version=schema_version,
            profile=loaded_profile,
            python_version=str(python_cfg["version"]),
            python_project_dir=str(python_cfg["project_dir"]),
            python_lock_file=str(python_cfg["lock_file"]),
            default_targets=default_targets,
            tools=normalized_tools,
            path_order=normalized_path_order,
            conan_config_id=str(conan_cfg["config_id"]),
            conan_remote_name=str(conan_cfg["remote_name"]),
            conan_remote_url=str(conan_cfg["remote_url"]),
            conan_home_subdir=str(conan_cfg["home_subdir"]),
        )
        log_done(f"Loaded profile lock '{loaded.profile}'")
        log_kv("lock", loaded.path)
        log_kv("python", loaded.python_version)
        log_kv("default targets", ",".join(loaded.default_targets))
        log_kv("tools", ", ".join(f"{name}@{version}" for name, version in loaded.tools.items()))
        log_kv("conan config", loaded.conan_config_id)
        return loaded


@dataclass(frozen=True)
class RuntimeConfig:
    install_root: str
    cache_root: str
    staging_root: str
    offline: bool
    python_default_index: str
    python_indexes: List[str]
    python_index_strategy: str
    sources: List[Dict[str, str]]

    @property
    def state_path(self) -> str:
        return os.path.join(self.install_root, STATE_FILE_NAME)

    @property
    def config_path(self) -> str:
        return os.path.join(self.install_root, CONFIG_FILE_NAME)

    @classmethod
    def load(cls, args: argparse.Namespace) -> "RuntimeConfig":
        china_mirror_applied = apply_china_mirror_preset()
        if china_mirror_applied:
            log_info(f"Applied China mirror preset from {CHINA_MIRROR_ENV_NAME}.")
            for key, value in CHINA_MIRROR_PRESET.items():
                log_kv(key, value)

        install_root = os.path.realpath(
            os.path.expanduser(
                os.environ.get("SIFLI_SDK_TOOLS_PATH")
                or DEFAULT_INSTALL_ROOT
            )
        )

        config_path = os.path.join(install_root, CONFIG_FILE_NAME)
        raw: Dict[str, Any] = {}
        if os.path.exists(config_path):
            with open(config_path, "rb") as f:
                raw = json.load(f)

        cache_root = os.path.realpath(
            os.path.expanduser(
                args.cache_dir
                or os.environ.get("SIFLI_SDK_CACHE_PATH")
                or raw.get("cache_root")
                or os.path.join(install_root, "cache")
            )
        )
        staging_root = os.path.realpath(
            os.path.expanduser(
                args.staging_dir
                or os.environ.get("SIFLI_SDK_STAGING_PATH")
                or raw.get("staging_root")
                or os.path.join(install_root, "staging")
            )
        )
        offline = parse_bool(
            os.environ.get("SIFLI_SDK_OFFLINE"),
            default=bool(raw.get("offline", False)),
        )
        if getattr(args, "offline", False):
            offline = True

        python_packages = raw.get("python_packages", {}) if isinstance(raw.get("python_packages"), dict) else {}
        default_index = (
            os.environ.get("SIFLI_SDK_PYPI_DEFAULT_INDEX")
            or python_packages.get("default_index")
            or DEFAULT_PYPI_INDEX
        )
        extra_indexes = parse_csv_env(os.environ.get("SIFLI_SDK_PYPI_INDEX"))
        if not extra_indexes:
            raw_indexes = python_packages.get("indexes", [])
            if isinstance(raw_indexes, list):
                extra_indexes = [str(item) for item in raw_indexes]
        index_strategy = (
            os.environ.get("SIFLI_SDK_PYPI_INDEX_STRATEGY")
            or python_packages.get("index_strategy")
            or "first-index"
        )

        sources: List[Dict[str, str]] = [
            {"type": "local-cache", "path": os.path.join(cache_root, "dist")}
        ]
        raw_sources = raw.get("sources", [])
        if isinstance(raw_sources, list):
            for item in raw_sources:
                if isinstance(item, dict) and "type" in item:
                    normalized_item = {str(k): str(v) for k, v in item.items()}
                    sources.append(normalized_item)

        github_assets = (
            args.mirror
            or os.environ.get("SIFLI_SDK_GITHUB_ASSETS")
            or os.environ.get("SIFLI_SDK_MIRROR")
        )
        if github_assets:
            sources.append({"type": "github-assets", "url": str(github_assets)})

        if not any(item.get("type") == "upstream" for item in sources):
            sources.append({"type": "upstream", "url": DEFAULT_UPSTREAM_URL})

        config = cls(
            install_root=install_root,
            cache_root=cache_root,
            staging_root=staging_root,
            offline=offline,
            python_default_index=str(default_index),
            python_indexes=extra_indexes,
            python_index_strategy=str(index_strategy),
            sources=sources,
        )
        log_done("Loaded runtime config")
        log_kv("install root", config.install_root)
        log_kv("cache root", config.cache_root)
        log_kv("staging root", config.staging_root)
        log_kv("offline", config.offline)
        log_kv("python default index", config.python_default_index)
        log_kv("python indexes", ",".join(config.python_indexes) if config.python_indexes else "(none)")
        log_kv("index strategy", config.python_index_strategy)
        log_kv("sources", ", ".join(item.get("type", "?") for item in config.sources))
        return config


@dataclass(frozen=True)
class ToolPlan:
    name: str
    version: str
    required: bool
    tool: legacy_tools.SiFliSDKTool


@dataclass(frozen=True)
class ResolvedEnvInstance:
    key: str
    compat_sha: str
    env_state: Optional[Dict[str, Any]]
    python_env_path: str
    conan_home: str


def repo_root() -> str:
    return os.path.realpath(os.path.join(os.path.dirname(__file__), ".."))


def parse_bool(value: Optional[str], default: bool = False) -> bool:
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def apply_china_mirror_preset(env: Optional[MutableMapping[str, str]] = None) -> bool:
    target_env = os.environ if env is None else env
    if not parse_bool(target_env.get(CHINA_MIRROR_ENV_NAME), default=False):
        return False
    for key, value in CHINA_MIRROR_PRESET.items():
        target_env[key] = value
    return True


def parse_csv_env(value: Optional[str]) -> List[str]:
    if not value:
        return []
    return [item.strip() for item in value.split(",") if item.strip()]


def is_windows_host() -> bool:
    return os.name == "nt"


def normalize_target(value: str) -> str:
    return value.strip().lower().replace("-", "")


def normalize_path(path: str) -> str:
    return os.path.normcase(os.path.normpath(path))


def normalize_filesystem_path(path: str) -> str:
    return os.path.normpath(os.path.realpath(os.path.expandvars(os.path.expanduser(path))))


def keil_armclang_bin(root: str) -> str:
    return os.path.join(root, "ARM", "ARMCLANG", "bin")


def validate_keil_toolchain(root: str) -> Dict[str, str]:
    if not is_windows_host():
        raise SDKEnvError("--keil is only supported on Windows")
    if not root or not str(root).strip():
        raise SDKEnvError("--keil requires a Keil root directory")

    normalized_root = normalize_filesystem_path(str(root).strip())
    if not os.path.isdir(normalized_root):
        raise SDKEnvError(f"Keil root directory does not exist: {normalized_root}")

    armclang_bin = normalize_filesystem_path(keil_armclang_bin(normalized_root))
    if not os.path.isdir(armclang_bin):
        raise SDKEnvError(
            f"Keil root directory must contain ARM/ARMCLANG/bin: {normalized_root}"
        )

    return {
        "root": normalized_root,
        "armclang_bin": armclang_bin,
    }


def shell_kind(shell: str) -> str:
    normalized = shell.strip().lower()
    if normalized in POWERSHELL_SHELLS:
        return "powershell"
    if normalized in POSIX_SHELLS:
        return normalized
    raise SDKEnvError(f"unsupported shell '{shell}'")


def quote_sh(value: str) -> str:
    return shlex.quote(value)


def quote_ps(value: str) -> str:
    return "'" + value.replace("'", "''") + "'"


def atomic_write_text(path: str, content: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(prefix=".tmp-", dir=os.path.dirname(path))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(content)
        os.replace(tmp_path, path)
    except Exception:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass
        raise


def atomic_write_json(path: str, payload: Dict[str, Any]) -> None:
    atomic_write_text(path, json.dumps(payload, indent=2, ensure_ascii=False) + "\n")


def atomic_write_toml(path: str, payload: Dict[str, Any]) -> None:
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


def file_sha256(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_json_bytes(payload: Any) -> bytes:
    return json.dumps(payload, sort_keys=True, ensure_ascii=False, separators=(",", ":")).encode("utf-8")


def compute_env_compat_sha256(lock_path: str, pyproject_path: str, uv_lock_path: str) -> str:
    try:
        with open(lock_path, "rb") as f:
            lock_raw = json.load(f)
        with open(pyproject_path, "rb") as f:
            pyproject_raw = tomllib.load(f)
        with open(uv_lock_path, "rb") as f:
            uv_lock_raw = tomllib.load(f)
    except FileNotFoundError as exc:
        raise SDKEnvError(f"environment lock input is missing: {exc.filename}") from exc
    except tomllib.TOMLDecodeError as exc:
        raise SDKEnvError(f"failed to parse TOML lock input: {exc}") from exc

    compat_payload = {
        "python": {
            "version": lock_raw["python"]["version"],
            "project_dir": lock_raw["python"]["project_dir"],
            "lock_file": lock_raw["python"]["lock_file"],
        },
        "defaults": {
            "targets": lock_raw["defaults"]["targets"],
        },
        "tools": lock_raw["tools"],
        "conan": {
            "config_id": lock_raw["conan"]["config_id"],
            "remote_name": lock_raw["conan"]["remote_name"],
            "remote_url": lock_raw["conan"]["remote_url"],
        },
    }

    payload = (
        b"lock-json\0"
        + canonical_json_bytes(compat_payload)
        + b"\0pyproject-toml\0"
        + canonical_json_bytes(pyproject_raw)
        + b"\0uv-lock\0"
        + canonical_json_bytes(uv_lock_raw)
    )
    return hashlib.sha256(payload).hexdigest()


def current_timestamp() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def normalize_auto_reconcile(value: Any) -> str:
    normalized = str(value or "ask")
    return normalized if normalized in {"ask", "always", "never"} else "ask"


def normalize_git_head(value: Any) -> Optional[str]:
    if not isinstance(value, str):
        return None
    normalized = value.strip()
    return normalized or None


def empty_state_doc() -> Dict[str, Any]:
    return {"schema_version": STATE_SCHEMA_VERSION, "repos": {}, "envs": {}}


def state_envs(state_doc: Dict[str, Any]) -> Dict[str, Any]:
    envs = state_doc.setdefault("envs", {})
    if not isinstance(envs, dict):
        state_doc["envs"] = {}
        envs = state_doc["envs"]
    return envs


def normalize_profile_state_doc(profile_state: Any) -> Dict[str, Any]:
    if not isinstance(profile_state, dict):
        profile_state = {}
    preferences = profile_state.get("preferences")
    if not isinstance(preferences, dict):
        preferences = {}
    preferences["auto_reconcile"] = normalize_auto_reconcile(preferences.get("auto_reconcile"))
    profile_state["preferences"] = preferences

    selected_env_key = profile_state.get("selected_env_key")
    if not isinstance(selected_env_key, str) or not selected_env_key.strip():
        profile_state["selected_env_key"] = None
    else:
        profile_state["selected_env_key"] = selected_env_key.strip()

    last_seen_git_head = normalize_git_head(profile_state.get("last_seen_git_head"))
    if last_seen_git_head is None:
        profile_state.pop("last_seen_git_head", None)
    else:
        profile_state["last_seen_git_head"] = last_seen_git_head

    if "toolchains" in profile_state and not isinstance(profile_state["toolchains"], dict):
        profile_state.pop("toolchains", None)

    profile_state.pop("installed", None)
    return profile_state


def env_key(profile: str, compat_sha: str) -> str:
    return f"{profile}|{compat_sha}"


def normalize_env_state_doc(env_state: Any) -> Dict[str, Any]:
    if not isinstance(env_state, dict):
        return {}

    normalized = copy.deepcopy(env_state)
    sdk_state = normalized.get("sdk")
    normalized_sdk_state: Dict[str, Any] = {}
    if isinstance(sdk_state, dict):
        compat_algorithm = str(sdk_state.get("env_compat_algorithm", "")).strip()
        compat_sha = str(sdk_state.get("env_compat_sha256", "")).strip()
        if compat_sha and not compat_algorithm:
            compat_algorithm = ENV_COMPAT_ALGORITHM
        if compat_algorithm:
            normalized_sdk_state["env_compat_algorithm"] = compat_algorithm
        if compat_sha:
            normalized_sdk_state["env_compat_sha256"] = compat_sha
    normalized["sdk"] = normalized_sdk_state

    last_used_at = normalized.get("last_used_at")
    if not isinstance(last_used_at, str) or not last_used_at.strip():
        normalized.pop("last_used_at", None)
    else:
        normalized["last_used_at"] = last_used_at.strip()
    return normalized


def migrate_v1_env_state(profile: str, installed: Any) -> tuple[Optional[str], Optional[Dict[str, Any]]]:
    if not isinstance(installed, dict) or not installed:
        return None, None

    migrated = normalize_env_state_doc(installed)
    sdk_state = migrated.get("sdk")
    if not isinstance(sdk_state, dict):
        sdk_state = {}
        migrated["sdk"] = sdk_state

    compat_sha = str(sdk_state.get("env_compat_sha256", "")).strip()
    if not compat_sha:
        compat_sha = hashlib.sha256(canonical_json_bytes(installed)).hexdigest()
        sdk_state["env_compat_sha256"] = compat_sha
    if not sdk_state.get("env_compat_algorithm"):
        sdk_state["env_compat_algorithm"] = ENV_COMPAT_ALGORITHM

    migrated["last_used_at"] = str(migrated.get("last_used_at") or current_timestamp())
    return env_key(profile, compat_sha), migrated


def migrate_v1_state(raw: Dict[str, Any]) -> Dict[str, Any]:
    migrated = empty_state_doc()
    repos = raw.get("repos")
    if not isinstance(repos, dict):
        return migrated

    for root, repo_entry in repos.items():
        if not isinstance(repo_entry, dict):
            continue
        profiles = repo_entry.get("profiles")
        if not isinstance(profiles, dict):
            continue
        for profile, legacy_profile_state in profiles.items():
            if not isinstance(profile, str):
                continue
            new_profile_state = get_profile_state(migrated, str(root), profile)
            preferences = legacy_profile_state.get("preferences") if isinstance(legacy_profile_state, dict) else {}
            if isinstance(preferences, dict):
                new_profile_state["preferences"]["auto_reconcile"] = normalize_auto_reconcile(
                    preferences.get("auto_reconcile")
                )

            toolchains = legacy_profile_state.get("toolchains") if isinstance(legacy_profile_state, dict) else None
            if isinstance(toolchains, dict):
                new_profile_state["toolchains"] = copy.deepcopy(toolchains)

            installed = legacy_profile_state.get("installed") if isinstance(legacy_profile_state, dict) else None
            legacy_sdk_state = installed.get("sdk") if isinstance(installed, dict) and isinstance(installed.get("sdk"), dict) else {}
            legacy_git_head = normalize_git_head(legacy_sdk_state.get("git_head"))
            if legacy_git_head is not None:
                new_profile_state["last_seen_git_head"] = legacy_git_head
            migrated_env_key, env_state = migrate_v1_env_state(profile, installed)
            if migrated_env_key and env_state:
                state_envs(migrated).setdefault(migrated_env_key, env_state)
                new_profile_state["selected_env_key"] = migrated_env_key
    return migrated


def normalize_state_doc(raw: Dict[str, Any]) -> Dict[str, Any]:
    schema_version = int(raw.get("schema_version", 0))
    if schema_version == STATE_SCHEMA_VERSION:
        normalized = empty_state_doc()
        repos = raw.get("repos")
        if isinstance(repos, dict):
            for root, repo_entry in repos.items():
                if not isinstance(repo_entry, dict):
                    continue
                for profile, profile_state in (repo_entry.get("profiles") or {}).items():
                    if not isinstance(profile, str):
                        continue
                    normalized_profile_state = get_profile_state(normalized, str(root), profile)
                    normalized_profile_state.update(normalize_profile_state_doc(copy.deepcopy(profile_state)))
        envs = raw.get("envs")
        if isinstance(envs, dict):
            normalized["envs"] = {
                str(key): normalize_env_state_doc(env_state)
                for key, env_state in envs.items()
                if isinstance(key, str) and isinstance(env_state, dict)
            }
        return normalized
    if schema_version == 1:
        return migrate_v1_state(raw)
    return empty_state_doc()


def load_state(path: str) -> Dict[str, Any]:
    if not os.path.exists(path):
        return empty_state_doc()
    with open(path, "rb") as f:
        raw = json.load(f)
    return normalize_state_doc(raw)


def get_profile_state(state_doc: Dict[str, Any], root: str, profile: str) -> Dict[str, Any]:
    repos = state_doc.setdefault("repos", {})
    repo_entry = repos.setdefault(root, {})
    profiles = repo_entry.setdefault("profiles", {})
    return profiles.setdefault(profile, {"selected_env_key": None, "preferences": {"auto_reconcile": "ask"}})


def read_profile_state(path: str, root: str, profile: str) -> Optional[Dict[str, Any]]:
    state_doc = load_state(path)
    return (
        state_doc.get("repos", {})
        .get(root, {})
        .get("profiles", {})
        .get(profile)
    )


def write_profile_state(
    path: str,
    root: str,
    profile: str,
    env_key_value: Optional[str] = None,
    env_state: Optional[Dict[str, Any]] = None,
    auto_reconcile: Optional[str] = None,
    toolchains: Optional[Dict[str, Optional[Dict[str, str]]]] = None,
    last_seen_git_head: Optional[str] = None,
    touch_selected_env: bool = False,
) -> None:
    state_doc = load_state(path)
    profile_state = get_profile_state(state_doc, root, profile)
    if env_key_value is not None:
        profile_state["selected_env_key"] = env_key_value
    if toolchains is not None:
        existing_toolchains = profile_state.get("toolchains")
        if not isinstance(existing_toolchains, dict):
            existing_toolchains = {}
        for name, value in toolchains.items():
            if value is None:
                existing_toolchains.pop(name, None)
            else:
                existing_toolchains[name] = value
        if existing_toolchains:
            profile_state["toolchains"] = existing_toolchains
        else:
            profile_state.pop("toolchains", None)
    profile_state.setdefault("preferences", {})
    if auto_reconcile is not None:
        profile_state["preferences"]["auto_reconcile"] = normalize_auto_reconcile(auto_reconcile)
    if last_seen_git_head is not None:
        normalized_git_head = normalize_git_head(last_seen_git_head)
        if normalized_git_head is None:
            profile_state.pop("last_seen_git_head", None)
        else:
            profile_state["last_seen_git_head"] = normalized_git_head

    envs = state_envs(state_doc)
    if env_state is not None:
        if not env_key_value:
            raise SDKEnvError("env_key is required when writing environment state")
        stored_env_state = normalize_env_state_doc(env_state)
        stored_env_state["last_used_at"] = current_timestamp()
        envs[env_key_value] = stored_env_state
    elif touch_selected_env:
        target_env_key = env_key_value or profile_state.get("selected_env_key")
        if isinstance(target_env_key, str) and isinstance(envs.get(target_env_key), dict):
            envs[target_env_key]["last_used_at"] = current_timestamp()
    atomic_write_json(path, state_doc)
    log_ok(f"Updated state file {path}")


def current_git_head(root: str) -> str:
    try:
        return (
            subprocess.check_output(["git", "-C", root, "rev-parse", "HEAD"], stderr=subprocess.DEVNULL)
            .decode("utf-8")
            .strip()
        )
    except Exception:
        return ""


def current_git_dirty(root: str) -> bool:
    try:
        output = subprocess.check_output(
            ["git", "-C", root, "status", "--short", "--untracked-files=no"],
            stderr=subprocess.DEVNULL,
        )
        return bool(output.strip())
    except Exception:
        return False


def read_version_txt(root: str) -> str:
    path = os.path.join(root, "version.txt")
    with open(path, "r", encoding="utf-8") as f:
        return f.read().strip()


def sdk_release_line(version_txt: str) -> str:
    match = re.match(r"^v?([0-9]+)\.([0-9]+)(?:\.[0-9]+)?(?:[-+][0-9A-Za-z.-]+)?$", version_txt.strip())
    if not match:
        raise SDKEnvError(f"failed to parse SDK release line from version '{version_txt}'")
    return f"{match.group(1)}.{match.group(2)}"


def read_sdk_release_line(root: str) -> str:
    return sdk_release_line(read_version_txt(root))


def env_instance_root(config: RuntimeConfig, profile: str, compat_sha: str) -> str:
    return os.path.join(config.install_root, "envs", profile, compat_sha)


def python_env_path(config: RuntimeConfig, lock: ProfileLock, compat_sha: str) -> str:
    return os.path.join(env_instance_root(config, lock.profile, compat_sha), "python")


def python_bin_dir(env_path: str) -> str:
    return os.path.join(env_path, "Scripts" if os.name == "nt" else "bin")


def python_executable(env_path: str) -> str:
    return os.path.join(python_bin_dir(env_path), "python.exe" if os.name == "nt" else "python")


def current_interpreter_env_path() -> str:
    return os.path.realpath(os.path.dirname(os.path.dirname(sys.executable)))


def conan_home_path(config: RuntimeConfig, lock: ProfileLock, compat_sha: str) -> str:
    return os.path.join(env_instance_root(config, lock.profile, compat_sha), "conan")


def repo_tools_dir(root: str) -> str:
    return os.path.join(root, "tools")


def pkg_root(config: RuntimeConfig) -> str:
    return os.path.join(config.install_root, "rt-pkg")


def configure_legacy_tools(root: str, install_root: str) -> None:
    legacy_tools.g.sifli_sdk_path = root
    legacy_tools.g.sifli_sdk_tools_path = install_root
    legacy_tools.g.tools_json = os.path.join(root, "tools", "tools.json")


def target_matches(tool: legacy_tools.SiFliSDKTool, targets: Sequence[str]) -> bool:
    if "all" in targets:
        return True
    supported = [normalize_target(item) for item in tool.get_supported_targets()]
    return "all" in supported or any(target in supported for target in targets)


def load_tool_plans(root: str, config: RuntimeConfig, lock: ProfileLock, targets: Sequence[str]) -> List[ToolPlan]:
    configure_legacy_tools(root, config.install_root)
    tools_info = legacy_tools.load_tools_info()
    plans: List[ToolPlan] = []
    for name, tool in tools_info.items():
        if not tool.compatible_with_platform():
            continue
        if not target_matches(tool, targets):
            continue
        if name not in lock.tools:
            raise SDKEnvError(f"profile lock does not bind a version for visible tool '{name}'")
        version = lock.tools[name]
        if version not in tool.versions:
            raise SDKEnvError(f"profile lock binds {name}@{version}, but that version does not exist in tools.json")
        if not tool.versions[version].compatible_with_platform():
            raise SDKEnvError(f"profile lock binds {name}@{version}, but it is not available on this platform")
        install_type = tool.get_install_type()
        if install_type == legacy_tools.SiFliSDKTool.INSTALL_NEVER:
            continue
        plans.append(
            ToolPlan(
                name=name,
                version=version,
                required=install_type == legacy_tools.SiFliSDKTool.INSTALL_ALWAYS,
                tool=tool,
            )
        )
    required = [f"{plan.name}@{plan.version}" for plan in plans if plan.required]
    optional = [f"{plan.name}@{plan.version}" for plan in plans if not plan.required]
    log_done("Resolved tool plan")
    log_kv("targets", ",".join(targets))
    log_kv("required", ", ".join(required) if required else "(none)")
    log_kv("optional", ", ".join(optional) if optional else "(none)")
    return plans


def tool_install_dir(config: RuntimeConfig, plan: ToolPlan) -> str:
    return os.path.join(config.install_root, "tools", plan.name, plan.version)


def tool_export_paths(plan: ToolPlan, base_dir: str) -> List[str]:
    return [os.path.join(base_dir, *parts) for parts in plan.tool._current_options.export_paths]


def validate_tool_dir(plan: ToolPlan, base_dir: str) -> bool:
    if not os.path.isdir(base_dir):
        return False
    if not plan.tool.is_executable:
        return True
    try:
        version = plan.tool.get_version(tool_export_paths(plan, base_dir))
    except Exception:
        return False
    return version == plan.version


def installed_tool_versions(plans: Sequence[ToolPlan], config: RuntimeConfig) -> Dict[str, str]:
    installed: Dict[str, str] = {}
    for plan in plans:
        if validate_tool_dir(plan, tool_install_dir(config, plan)):
            installed[plan.name] = plan.version
    return installed


DOWNLOAD_CHUNK_SIZE = 64 * 1024


def should_show_download_progress() -> bool:
    return use_color()


def resolve_download_total_size(response: Any, expected_size: int) -> Optional[int]:
    content_length = response.headers.get("Content-Length")
    if content_length:
        try:
            total = int(content_length)
            if total > 0:
                return total
        except (TypeError, ValueError):
            pass
    if expected_size > 0:
        return expected_size
    return None


def create_download_progress(total_size: Optional[int]) -> Progress:
    columns: List[Any] = [
        SpinnerColumn(style="sdk.step"),
        TextColumn("{task.description}", style="sdk.info"),
        BarColumn(),
    ]
    if total_size is not None:
        columns.append(TaskProgressColumn())
    columns.extend(
        [
            DownloadColumn(),
            TransferSpeedColumn(),
        ]
    )
    if total_size is not None:
        columns.append(TimeRemainingColumn())
    return Progress(
        *columns,
        console=RICH_CONSOLE,
        transient=True,
        disable=not should_show_download_progress(),
    )


def download_to_path(url: str, destination: str, label: str, expected_size: int) -> None:
    req = urllib.request.Request(url, headers={"User-Agent": "sdk-env/1"})
    with urllib.request.urlopen(req) as response, open(destination, "wb") as f:
        total_size = resolve_download_total_size(response, expected_size)
        with create_download_progress(total_size) as progress:
            task_id = progress.add_task(f"Downloading {label}", total=total_size)
            while True:
                chunk = response.read(DOWNLOAD_CHUNK_SIZE)
                if not chunk:
                    break
                f.write(chunk)
                progress.update(task_id, advance=len(chunk))


def download_candidate(
    url: str,
    destination: str,
    offline: bool,
    label: str,
    expected_size: int,
) -> bool:
    if offline:
        return False
    try:
        download_to_path(url, destination, label, expected_size)
    except urllib.error.URLError:
        return False
    return True


def artifact_cache_path(config: RuntimeConfig, filename: str) -> str:
    return os.path.join(config.cache_root, "dist", filename)


def install_tool_plan(
    config: RuntimeConfig,
    plan: ToolPlan,
    bundle_dir: Optional[str],
) -> None:
    final_dir = tool_install_dir(config, plan)
    if validate_tool_dir(plan, final_dir):
        log_done(f"Reusing installed tool {plan.name}@{plan.version}")
        return
    if os.path.exists(final_dir):
        log_warn(f"Removing invalid existing tool directory {final_dir}")
        shutil.rmtree(final_dir)

    download_obj = plan.tool.versions[plan.version].get_download_for_platform(legacy_tools.PYTHON_PLATFORM)
    if download_obj is None:
        raise SDKEnvError(f"tool {plan.name}@{plan.version} has no download for this platform")

    filename = download_obj.rename_dist or os.path.basename(download_obj.url)
    archive_path = ensure_cached_artifact(
        config=config,
        filename=filename,
        expected_sha256=download_obj.sha256,
        expected_size=download_obj.size,
        original_url=download_obj.url,
        bundle_dir=bundle_dir,
    )

    os.makedirs(config.staging_root, exist_ok=True)
    staging_dir = os.path.join(config.staging_root, f"{plan.name}-{plan.version}-{uuid.uuid4().hex}")
    os.makedirs(os.path.dirname(final_dir), exist_ok=True)
    log_kv("archive", archive_path)
    log_kv("staging", staging_dir)
    log_kv("final", final_dir)
    legacy_tools.unpack(archive_path, staging_dir)
    if plan.tool._current_options.strip_container_dirs:
        legacy_tools.do_strip_container_dirs(staging_dir, plan.tool._current_options.strip_container_dirs)
    if not validate_tool_dir(plan, staging_dir):
        shutil.rmtree(staging_dir, ignore_errors=True)
        raise SDKEnvError(f"tool {plan.name}@{plan.version} failed validation after extraction")
    legacy_tools.rename_with_retry(staging_dir, final_dir)
    log_done(f"Installing tool {plan.name}@{plan.version}")


def conan_archive_name(config_id: str) -> str:
    return f"{config_id}.zip" if not config_id.endswith(".zip") else config_id


def ensure_conan_config_archive(
    config: RuntimeConfig,
    lock: ProfileLock,
    bundle_dir: Optional[str],
) -> str:
    filename = conan_archive_name(lock.conan_config_id)
    archive_path = ensure_cached_artifact(
        config=config,
        filename=filename,
        expected_sha256="",
        expected_size=-1,
        original_url=DEFAULT_UPSTREAM_URL.rstrip("/") + "/" + filename,
        bundle_dir=bundle_dir,
    )
    log_done(f"Prepared Conan config archive", archive_path, "sdk.info")
    return archive_path


def ensure_cached_artifact(
    config: RuntimeConfig,
    filename: str,
    expected_sha256: str,
    expected_size: int,
    original_url: str,
    bundle_dir: Optional[str],
) -> str:
    cache_path = artifact_cache_path(config, filename)
    os.makedirs(os.path.dirname(cache_path), exist_ok=True)

    def matches(path: str) -> bool:
        if expected_size < 0 or not expected_sha256:
            return os.path.isfile(path)
        size, sha = legacy_tools.get_file_size_sha256(path)
        return size == expected_size and sha.lower() == expected_sha256.lower()

    if os.path.exists(cache_path):
        if matches(cache_path):
            log_done(f"Cache hit for {filename}")
            return cache_path
        log_warn(f"Cached artifact {cache_path} failed validation, removing")
        os.unlink(cache_path)

    candidate_paths: List[str] = []
    if bundle_dir:
        candidate_paths.append(os.path.join(bundle_dir, filename))

    for source in config.sources:
        source_type = source.get("type")
        if source_type in {"local-cache", "local-source"} and "path" in source:
            candidate_paths.append(os.path.join(os.path.expanduser(source["path"]), filename))

    for candidate in candidate_paths:
        if not os.path.isfile(candidate):
            continue
        log_info(f"Using local artifact candidate {candidate}")
        shutil.copy2(candidate, cache_path)
        if matches(cache_path):
            log_done(f"Stored {filename} in cache from local source")
            return cache_path
        os.unlink(cache_path)

    url_candidates: List[str] = []
    for source in config.sources:
        source_type = source.get("type")
        if source_type == "github-assets" and "url" in source:
            url_candidates.append(github_assets_mirror_url(source["url"], original_url, filename))
        elif source_type == "upstream":
            if source.get("url", DEFAULT_UPSTREAM_URL).rstrip("/") == DEFAULT_UPSTREAM_URL.rstrip("/"):
                url_candidates.append(original_url)
            elif "url" in source:
                url_candidates.append(source["url"].rstrip("/") + "/" + filename)

    tmp_path = f"{cache_path}.tmp"
    for index, url in enumerate(url_candidates):
        log_info(f"Downloading artifact {filename} from {url}")
        if download_candidate(url, tmp_path, config.offline, filename, expected_size):
            if matches(tmp_path):
                os.replace(tmp_path, cache_path)
                log_done(f"Downloaded and cached {filename}")
                return cache_path
            log_warn(f"Downloaded artifact {filename} from {url} but validation failed")
        elif index + 1 < len(url_candidates):
            log_warn(f"Failed to download artifact {filename} from {url}, falling back to the next source.")
        try:
            os.unlink(tmp_path)
        except OSError:
            pass

    raise SDKEnvError(f"unable to obtain artifact '{filename}' from cache, bundle, mirror, or upstream")


def run_command(
    cmd: Sequence[str],
    env: Optional[Dict[str, str]] = None,
    cwd: Optional[str] = None,
    capture_output: bool = False,
) -> str:
    rendered = " ".join(shlex.quote(part) for part in cmd)
    log_info(rendered)
    completed = subprocess.run(
        list(cmd),
        env=env,
        cwd=cwd,
        check=False,
        text=True,
        stdout=subprocess.PIPE if capture_output else None,
        stderr=subprocess.STDOUT if capture_output else None,
    )
    if completed.returncode != 0:
        output = completed.stdout or ""
        raise SDKEnvError(output.strip() or f"command failed with exit code {completed.returncode}: {rendered}")
    return completed.stdout or ""


def normalized_pypi_index_url(url: str) -> str:
    return url.rstrip("/")


def derive_pypi_artifact_mirror_prefix(default_index: str) -> Optional[str]:
    normalized = normalized_pypi_index_url(default_index)
    for suffix in ("/web/simple", "/simple"):
        if normalized.endswith(suffix):
            return normalized[: -len(suffix)] + "/packages"
    return None


def github_assets_mirror_url(mirror_base: str, original_url: str, filename: str) -> str:
    normalized_base = mirror_base.rstrip("/")
    if "://" not in normalized_base:
        normalized_base = "https://" + normalized_base
    github_prefix = "https://github.com/"
    if original_url.startswith(github_prefix):
        return normalized_base + "/" + original_url[len(github_prefix):]
    return normalized_base + "/" + filename


def rewrite_uv_lock_for_index(
    uv_lock_doc: Dict[str, Any],
    default_index: str,
) -> tuple[Dict[str, Any], bool, bool]:
    canonical_index = normalized_pypi_index_url(DEFAULT_PYPI_INDEX)
    target_index = normalized_pypi_index_url(default_index)
    if target_index == canonical_index:
        return copy.deepcopy(uv_lock_doc), False, False

    rewritten = copy.deepcopy(uv_lock_doc)
    artifact_prefix = derive_pypi_artifact_mirror_prefix(target_index)
    rewrote_registry = False
    rewrote_artifacts = False

    packages = rewritten.get("package", [])
    if not isinstance(packages, list):
        raise SDKEnvError("uv.lock does not contain a valid package list")

    for package in packages:
        if not isinstance(package, dict):
            continue
        source = package.get("source")
        if isinstance(source, dict) and source.get("registry") == canonical_index:
            source["registry"] = target_index
            rewrote_registry = True

        if artifact_prefix is None:
            continue

        sdist = package.get("sdist")
        if isinstance(sdist, dict):
            sdist_url = sdist.get("url")
            if isinstance(sdist_url, str) and sdist_url.startswith(DEFAULT_PYPI_FILES_PREFIX):
                sdist["url"] = artifact_prefix + sdist_url[len(DEFAULT_PYPI_FILES_PREFIX):]
                rewrote_artifacts = True

        wheels = package.get("wheels")
        if isinstance(wheels, list):
            for wheel in wheels:
                if not isinstance(wheel, dict):
                    continue
                wheel_url = wheel.get("url")
                if isinstance(wheel_url, str) and wheel_url.startswith(DEFAULT_PYPI_FILES_PREFIX):
                    wheel["url"] = artifact_prefix + wheel_url[len(DEFAULT_PYPI_FILES_PREFIX):]
                    rewrote_artifacts = True

    return rewritten, rewrote_registry, rewrote_artifacts


@contextmanager
def temporary_uv_project(lock: ProfileLock, config: RuntimeConfig) -> Any:
    project_dir = os.path.join(repo_root(), lock.python_project_dir)
    target_index = normalized_pypi_index_url(config.python_default_index)
    canonical_index = normalized_pypi_index_url(DEFAULT_PYPI_INDEX)
    if target_index == canonical_index:
        yield project_dir
        return

    with open(lock.uv_lock_path, "rb") as f:
        uv_lock_doc = tomllib.load(f)

    rewritten_doc, rewrote_registry, rewrote_artifacts = rewrite_uv_lock_for_index(
        uv_lock_doc,
        config.python_default_index,
    )
    if not rewrote_registry and not rewrote_artifacts:
        yield project_dir
        return

    temp_root = tempfile.mkdtemp(prefix=f"sdk-env-uv-project-{lock.profile}-")
    temp_project_dir = os.path.join(temp_root, lock.profile)
    os.makedirs(temp_project_dir, exist_ok=True)
    try:
        shutil.copy2(lock.pyproject_path, os.path.join(temp_project_dir, "pyproject.toml"))
        atomic_write_toml(os.path.join(temp_project_dir, "uv.lock"), rewritten_doc)

        if rewrote_registry:
            log_info(
                f"Using Python package mirror index {config.python_default_index} for profile '{lock.profile}'."
            )
        if rewrote_artifacts:
            artifact_prefix = derive_pypi_artifact_mirror_prefix(config.python_default_index)
            log_info(f"Rewriting canonical PyPI artifact URLs to {artifact_prefix}.")
        elif rewrote_registry:
            log_warn(
                f"Mirror index {config.python_default_index} is not a recognized standard PyPI mirror layout; "
                "artifact URLs in uv.lock remain upstream."
            )

        yield temp_project_dir
    finally:
        shutil.rmtree(temp_root, ignore_errors=True)


def uv_index_args(config: RuntimeConfig) -> List[str]:
    args = ["--default-index", config.python_default_index, "--index-strategy", config.python_index_strategy]
    for index in config.python_indexes:
        args.extend(["--index", index])
    if config.offline:
        args.append("--offline")
    return args


def recorded_python_env_path(env_state: Optional[Dict[str, Any]]) -> Optional[str]:
    if not isinstance(env_state, dict):
        return None
    python_state = env_state.get("python")
    if not isinstance(python_state, dict):
        return None
    env_path = python_state.get("env_path")
    if not isinstance(env_path, str) or not env_path.strip():
        return None
    return env_path


def recorded_conan_home(env_state: Optional[Dict[str, Any]]) -> Optional[str]:
    if not isinstance(env_state, dict):
        return None
    conan_state = env_state.get("conan")
    if not isinstance(conan_state, dict):
        return None
    conan_home = conan_state.get("home")
    if not isinstance(conan_home, str) or not conan_home.strip():
        return None
    return conan_home


def resolve_env_instance(
    config: RuntimeConfig,
    lock: ProfileLock,
    state_doc: Optional[Dict[str, Any]] = None,
) -> ResolvedEnvInstance:
    compat_sha = compute_env_compat_sha256(lock.path, lock.pyproject_path, lock.uv_lock_path)
    current_env_key = env_key(lock.profile, compat_sha)
    loaded_state = state_doc or load_state(config.state_path)
    env_state_raw = state_envs(loaded_state).get(current_env_key)
    env_state = env_state_raw if isinstance(env_state_raw, dict) else None
    return ResolvedEnvInstance(
        key=current_env_key,
        compat_sha=compat_sha,
        env_state=env_state,
        python_env_path=recorded_python_env_path(env_state) or python_env_path(config, lock, compat_sha),
        conan_home=recorded_conan_home(env_state) or conan_home_path(config, lock, compat_sha),
    )


def install_paths_for_env(
    config: RuntimeConfig,
    lock: ProfileLock,
    resolved_env: ResolvedEnvInstance,
) -> tuple[str, str]:
    recorded_env_path = recorded_python_env_path(resolved_env.env_state)
    recorded_home = recorded_conan_home(resolved_env.env_state)
    python_path = (
        recorded_env_path
        if recorded_env_path and os.path.exists(python_executable(recorded_env_path))
        else python_env_path(config, lock, resolved_env.compat_sha)
    )
    conan_home = (
        recorded_home
        if recorded_home and os.path.isdir(recorded_home)
        else conan_home_path(config, lock, resolved_env.compat_sha)
    )
    return python_path, conan_home


def ensure_python_env(config: RuntimeConfig, lock: ProfileLock, env_path: str) -> str:
    uv_env = os.environ.copy()
    apply_china_mirror_preset(uv_env)
    sync_env = uv_env.copy()
    sync_env["UV_PROJECT_ENVIRONMENT"] = env_path
    log_kv("env path", env_path)

    python_install_cmd = ["uv", "python", "install", lock.python_version]
    if config.offline:
        python_install_cmd.append("--offline")
    run_command(python_install_cmd, env=uv_env)

    with temporary_uv_project(lock, config) as project_dir:
        sync_cmd = [
            "uv",
            "sync",
            "--project",
            project_dir,
            "--locked",
            "--python",
            lock.python_version,
            "--no-install-project",
            *uv_index_args(config),
        ]
        run_command(sync_cmd, env=sync_env)
    python_path = python_executable(env_path)
    if not os.path.exists(python_path):
        raise SDKEnvError(f"python environment was created but interpreter is missing: {python_path}")
    log_done(f"Preparing Python environment for profile '{lock.profile}'")
    return env_path


def initialize_conan(
    config: RuntimeConfig,
    lock: ProfileLock,
    env_path: str,
    conan_home: str,
    bundle_dir: Optional[str],
) -> None:
    archive_path = ensure_conan_config_archive(config, lock, bundle_dir)
    os.makedirs(conan_home, exist_ok=True)
    conan_bin = os.path.join(python_bin_dir(env_path), "conan.exe" if os.name == "nt" else "conan")
    env = os.environ.copy()
    env["CONAN_HOME"] = conan_home
    log_kv("conan home", conan_home)
    log_kv("conan archive", archive_path)
    run_command([conan_bin, "config", "install", archive_path], env=env)
    run_command(
        [conan_bin, "remote", "add", lock.conan_remote_name, lock.conan_remote_url, "--force"],
        env=env,
    )
    log_done(f"Initializing Conan for profile '{lock.profile}'")


def collect_installed_state(
    config: RuntimeConfig,
    lock: ProfileLock,
    targets: Sequence[str],
    plans: Sequence[ToolPlan],
    compat_sha: str,
    env_path: str,
    conan_home: str,
) -> Dict[str, Any]:
    tool_versions = installed_tool_versions(plans, config)
    return {
        "sdk": {
            "env_compat_algorithm": ENV_COMPAT_ALGORITHM,
            "env_compat_sha256": compat_sha,
        },
        "locks": {
            "profile_lock_sha256": file_sha256(lock.path),
            "uv_lock_sha256": file_sha256(lock.uv_lock_path),
        },
        "python": {
            "version": lock.python_version,
            "env_path": env_path,
        },
        "targets": list(targets),
        "tools": tool_versions,
        "conan": {
            "config_id": lock.conan_config_id,
            "home": conan_home,
        },
        "cache_root": config.cache_root,
        "install_root": config.install_root,
    }


def parse_install_targets(lock: ProfileLock, cli_targets: Optional[str], compat_args: Sequence[str]) -> List[str]:
    compat_targets: List[str] = []
    unknown_args: List[str] = []
    for arg in compat_args:
        if arg.startswith("--enable-") or arg.startswith("--disable-"):
            raise SDKEnvError(
                f"legacy feature flag '{arg}' is no longer supported; Python dependencies are fully locked in tools/locks/{lock.profile}"
            )
        if arg.lower().startswith("sf32"):
            compat_targets.extend(normalize_target(item) for item in arg.split(","))
            continue
        unknown_args.append(arg)

    if unknown_args:
        raise SDKEnvError(f"unsupported install arguments: {' '.join(unknown_args)}")

    if cli_targets and compat_targets:
        raise SDKEnvError("--targets cannot be used together with compatibility target positionals")

    if cli_targets:
        return [normalize_target(item) for item in cli_targets.split(",") if item.strip()]
    if compat_targets:
        return compat_targets
    return list(lock.default_targets)


def merge_managed_paths(current_path: str, old_managed_paths: Sequence[str], new_managed_paths: Sequence[str]) -> str:
    current_items = [item for item in current_path.split(os.pathsep) if item]
    old_set = {normalize_path(item) for item in old_managed_paths if item}
    cleaned = [item for item in current_items if normalize_path(item) not in old_set]

    merged: List[str] = []
    seen: set[str] = set()
    for item in list(new_managed_paths) + cleaned:
        normalized = normalize_path(item)
        if normalized in seen:
            continue
        seen.add(normalized)
        merged.append(item)
    return os.pathsep.join(merged)


def install_command_hint(profile: str, shell: str) -> str:
    if shell_kind(shell) == "powershell":
        return f".\\install.ps1 --profile {profile}"
    return f"./install.sh --profile {profile}"


def keil_install_command_hint(profile: str) -> str:
    base = ".\\install.ps1"
    if profile != DEFAULT_PROFILE:
        base += f" --profile {profile}"
    return base + r" --keil C:\Keil_v5"


def export_command_name(shell: str) -> str:
    return "export.ps1" if shell_kind(shell) == "powershell" else "export.sh"


def render_export_command(root: str, profile: str, shell: str, toolchain: str) -> str:
    command = f"{os.path.join(root, export_command_name(shell))} --profile {profile}"
    if toolchain != TOOLCHAIN_GCC:
        command += f" --toolchain {toolchain}"
    return command


def load_keil_toolchain_state(config: RuntimeConfig, lock: ProfileLock) -> Dict[str, str]:
    if not is_windows_host():
        raise SDKEnvError("Keil export is only supported on Windows")

    profile_state = read_profile_state(config.state_path, repo_root(), lock.profile)
    toolchains = profile_state.get("toolchains") if isinstance(profile_state, dict) else None
    keil = toolchains.get(TOOLCHAIN_KEIL) if isinstance(toolchains, dict) else None
    if not isinstance(keil, dict):
        raise SDKEnvError(
            f"Keil toolchain is not configured for profile '{lock.profile}'. "
            f"Run `{keil_install_command_hint(lock.profile)}` first."
        )

    root = str(keil.get("root", "")).strip()
    armclang_bin = str(keil.get("armclang_bin", "")).strip()
    if not root or not armclang_bin:
        raise SDKEnvError(
            f"Keil toolchain state for profile '{lock.profile}' is incomplete. "
            f"Run `{keil_install_command_hint(lock.profile)}` again."
        )

    validated = validate_keil_toolchain(root)
    recorded_bin = normalize_filesystem_path(armclang_bin)
    if recorded_bin != validated["armclang_bin"]:
        raise SDKEnvError(
            f"Keil toolchain state for profile '{lock.profile}' is inconsistent. "
            f"Run `{keil_install_command_hint(lock.profile)}` again."
        )
    return validated


def gcc_exec_path(plans: Sequence[ToolPlan], config: RuntimeConfig, installed: Dict[str, str]) -> str:
    for plan in plans:
        if plan.name != GCC_TOOL_NAME:
            continue
        if installed.get(plan.name) != plan.version:
            continue
        paths = tool_export_paths(plan, tool_install_dir(config, plan))
        if paths:
            return os.path.realpath(paths[0])
    raise SDKEnvError(
        f"installed {GCC_TOOL_NAME} toolchain was not found. "
        "Run the install script again before exporting the GCC environment."
    )


def build_export_environment(
    config: RuntimeConfig,
    lock: ProfileLock,
    plans: Sequence[ToolPlan],
    env_path: str,
    conan_home: str,
    shell: str,
    toolchain: str = TOOLCHAIN_GCC,
) -> Dict[str, str]:
    root = repo_root()
    install_cmd_name = "install.ps1" if shell_kind(shell) == "powershell" else "install.sh"
    current_path = os.environ.get("PATH", "")
    old_managed = [item for item in os.environ.get("SIFLI_SDK_MANAGED_PATHS", "").split(os.pathsep) if item]
    installed = installed_tool_versions(plans, config)
    if toolchain not in SUPPORTED_TOOLCHAINS:
        raise SDKEnvError(f"unsupported toolchain '{toolchain}'")

    keil_toolchain: Optional[Dict[str, str]] = None
    if toolchain == TOOLCHAIN_KEIL:
        keil_toolchain = load_keil_toolchain_state(config, lock)

    managed_paths: List[str] = [python_bin_dir(env_path)]
    for tool_name in lock.path_order:
        for plan in plans:
            if plan.name != tool_name:
                continue
            if installed.get(plan.name) != plan.version:
                continue
            managed_paths.extend(tool_export_paths(plan, tool_install_dir(config, plan)))
    if keil_toolchain is not None:
        managed_paths.append(keil_toolchain["armclang_bin"])
    managed_paths.append(repo_tools_dir(root))

    managed_paths = [os.path.realpath(path) for path in managed_paths]
    path_value = merge_managed_paths(current_path, old_managed, managed_paths)
    rtt_cc = TOOLCHAIN_KEIL if keil_toolchain is not None else TOOLCHAIN_GCC
    rtt_exec_path = keil_toolchain["root"] if keil_toolchain is not None else gcc_exec_path(plans, config, installed)

    env_map: Dict[str, str] = {
        "SIFLI_SDK_PATH": root,
        "SIFLI_SDK_VERSION": read_sdk_release_line(root),
        "SIFLI_SDK_GIT_HEAD": current_git_head(root),
        "SIFLI_SDK_PROFILE": lock.profile,
        "SIFLI_SDK_PYTHON_ENV_PATH": env_path,
        "SIFLI_SDK_MANAGED_PATHS": os.pathsep.join(managed_paths),
        "SIFLI_SDK_TOOLS_INSTALL_CMD": f"{os.path.join(root, install_cmd_name)} --profile {lock.profile}",
        "SIFLI_SDK_TOOLS_EXPORT_CMD": render_export_command(root, lock.profile, shell, toolchain),
        "SIFLI_SDK": root,
        "CONAN_HOME": conan_home,
        "ENV_ROOT": env_path,
        "PKGS_ROOT": pkg_root(config),
        "PKGS_DIR": pkg_root(config),
        "RTT_CC": rtt_cc,
        "PYTHONPATH": os.path.join(root, "tools", "build"),
        "PATH": path_value,
        "RTT_EXEC_PATH": rtt_exec_path,
    }

    for plan in plans:
        if installed.get(plan.name) == plan.version:
            env_map.update(plan.tool.get_export_vars(plan.version))
    return env_map


def write_export_script(env_map: Dict[str, str], shell: str) -> str:
    shell_name = shell_kind(shell)
    temp_root = os.path.join(tempfile.gettempdir(), "sifli-sdk-export")
    os.makedirs(temp_root, exist_ok=True)
    suffix = ".ps1" if shell_name == "powershell" else ".sh"
    fd, script_path = tempfile.mkstemp(prefix="sdk-env-", suffix=suffix, dir=temp_root)
    os.close(fd)

    if shell_name == "powershell":
        lines = [f"$Env:{key} = {quote_ps(value)}" for key, value in env_map.items()]
        python_path = python_executable(env_map["SIFLI_SDK_PYTHON_ENV_PATH"])
        sdk_py_path = os.path.join(env_map["SIFLI_SDK_PATH"], "tools", "sdk.py")
        lines.append(
            f"function global:sdk.py {{ & {quote_ps(python_path)} {quote_ps(sdk_py_path)} @args }}"
        )
        lines.append('Write-Host ""')
        lines.append(f'Write-Host "Done! SiFli-SDK profile {env_map["SIFLI_SDK_PROFILE"]} is active."')
        lines.append('Write-Host "You can now run sdk.py from your project directory."')
    else:
        lines = [f"export {key}={quote_sh(value)}" for key, value in env_map.items()]
        lines.append('echo ""')
        lines.append(f'echo "Done! SiFli-SDK profile {env_map["SIFLI_SDK_PROFILE"]} is active."')
        lines.append('echo "You can now run sdk.py from your project directory."')

    atomic_write_text(script_path, "\n".join(lines) + "\n")
    return script_path


def export_reexec_argv(args: argparse.Namespace, lock: ProfileLock, env_path: str) -> List[str]:
    argv = [
        python_executable(env_path),
        os.path.join(repo_root(), "tools", "sdk_env.py"),
        "export",
        "--profile",
        lock.profile,
        "--shell",
        args.shell,
        "--toolchain",
        getattr(args, "toolchain", TOOLCHAIN_GCC),
    ]
    if args.cache_dir:
        argv.extend(["--cache-dir", args.cache_dir])
    if args.staging_dir:
        argv.extend(["--staging-dir", args.staging_dir])
    if args.mirror:
        argv.extend(["--mirror", args.mirror])
    if args.offline:
        argv.append("--offline")
    if args.from_bundle:
        argv.extend(["--from-bundle", args.from_bundle])
    return argv


def required_plan_names(plans: Sequence[ToolPlan]) -> List[str]:
    return [plan.name for plan in plans if plan.required]


def validate_env_instance(
    config: RuntimeConfig,
    lock: ProfileLock,
    plans: Sequence[ToolPlan],
) -> Tuple[List[str], ResolvedEnvInstance]:
    resolved_env = resolve_env_instance(config, lock)
    reasons: List[str] = []
    installed = resolved_env.env_state
    if not isinstance(installed, dict):
        return ["environment instance is not installed for the current checkout"], resolved_env

    sdk_state = installed.get("sdk") if isinstance(installed.get("sdk"), dict) else {}
    python_state = installed.get("python") if isinstance(installed.get("python"), dict) else {}
    conan_state = installed.get("conan") if isinstance(installed.get("conan"), dict) else {}
    tool_state = installed.get("tools") if isinstance(installed.get("tools"), dict) else {}
    installed_targets = installed.get("targets", [])

    if sdk_state.get("env_compat_algorithm") != ENV_COMPAT_ALGORITHM:
        reasons.append("state env_compat_algorithm is missing or unsupported")
    if sdk_state.get("env_compat_sha256") != resolved_env.compat_sha:
        reasons.append("environment compatibility hash does not match the current checkout")
    if python_state.get("version") != lock.python_version:
        reasons.append("python version changed")
    if not recorded_python_env_path(installed):
        reasons.append("python env path is missing from state")
    if not os.path.exists(python_executable(resolved_env.python_env_path)):
        reasons.append("python env interpreter is missing")
    if conan_state.get("config_id") != lock.conan_config_id:
        reasons.append("conan config_id changed")
    if not recorded_conan_home(installed):
        reasons.append("conan home is missing from state")
    if not os.path.isdir(resolved_env.conan_home):
        reasons.append("conan home is missing")
    if [normalize_target(item) for item in installed_targets] != list(lock.default_targets):
        reasons.append("installed targets do not match profile defaults")

    for plan in plans:
        if not plan.required:
            continue
        if tool_state.get(plan.name) != plan.version:
            reasons.append(f"required tool {plan.name}@{plan.version} is not recorded in state")
            continue
        if not validate_tool_dir(plan, tool_install_dir(config, plan)):
            reasons.append(f"required tool {plan.name}@{plan.version} is missing or invalid")
    return reasons, resolved_env


def prompt_reconcile_choice(reasons: Sequence[str]) -> str:
    log_warn("SDK environment for the current checkout is missing or invalid")
    for reason in reasons:
        print(f"  - {reason}", file=log_stream())
    print("Choose one of: once / always / never", file=log_stream())
    while True:
        choice = input("> ").strip().lower()
        if choice in {"once", "always", "never"}:
            return choice
        print("Please enter 'once', 'always', or 'never'.", file=log_stream())


def auto_reconcile_preference(config: RuntimeConfig, profile: str) -> str:
    profile_state = read_profile_state(config.state_path, repo_root(), profile)
    if not profile_state:
        return "ask"
    preferences = profile_state.get("preferences") if isinstance(profile_state.get("preferences"), dict) else {}
    return normalize_auto_reconcile(preferences.get("auto_reconcile"))


def perform_install(
    args: argparse.Namespace,
    config: RuntimeConfig,
    lock: ProfileLock,
    targets: Sequence[str],
    auto_reconcile: Optional[str] = None,
    keil_toolchain: Optional[Dict[str, str]] = None,
) -> Dict[str, Any]:
    log_step(f"Starting install for profile '{lock.profile}'")
    log_kv("mode", "install")
    plans = load_tool_plans(repo_root(), config, lock, targets)
    resolved_env = resolve_env_instance(config, lock)
    env_path, conan_home = install_paths_for_env(config, lock, resolved_env)
    log_kv("env key", resolved_env.key)
    log_kv("compat sha", resolved_env.compat_sha)
    env_path = ensure_python_env(config, lock, env_path)
    for plan in plans:
        if plan.required:
            install_tool_plan(config, plan, args.from_bundle)
    initialize_conan(config, lock, env_path, conan_home, args.from_bundle)
    installed_state = collect_installed_state(
        config,
        lock,
        targets,
        plans,
        resolved_env.compat_sha,
        env_path,
        conan_home,
    )
    preference = auto_reconcile or auto_reconcile_preference(config, lock.profile)
    write_profile_state(
        config.state_path,
        repo_root(),
        lock.profile,
        env_key_value=resolved_env.key,
        env_state=installed_state,
        auto_reconcile=preference,
        toolchains={TOOLCHAIN_KEIL: keil_toolchain} if keil_toolchain is not None else None,
        last_seen_git_head=current_git_head(repo_root()),
    )
    log_ok(f"Install completed for profile '{lock.profile}'")
    return installed_state


def warn_non_blocking_drift(profile_state: Optional[Dict[str, Any]]) -> None:
    current_head = current_git_head(repo_root())
    recorded_head = None
    if isinstance(profile_state, dict):
        recorded_head = normalize_git_head(profile_state.get("last_seen_git_head"))
    if recorded_head and current_head and recorded_head != current_head:
        log_warn(f"git HEAD changed from {recorded_head} to {current_head}, but environment is still compatible.")
    if current_git_dirty(repo_root()):
        log_warn("worktree is dirty; environment export will continue because the compatibility hash is unchanged.")


def handle_install(args: argparse.Namespace) -> int:
    keil_arg = getattr(args, "keil", None)
    keil_toolchain = validate_keil_toolchain(keil_arg) if keil_arg else None
    config = RuntimeConfig.load(args)
    lock = ProfileLock.load(repo_root(), args.profile)
    targets = parse_install_targets(lock, args.targets, args.compat_args)
    log_banner(
        f"Installing SiFli-SDK {read_version_txt(repo_root())}",
        f"profile={lock.profile}",
    )
    log_kv("selected targets", ",".join(targets))
    if keil_toolchain is not None:
        log_kv("keil root", keil_toolchain["root"])
        log_kv("keil armclang bin", keil_toolchain["armclang_bin"])
    perform_install(args, config, lock, targets, keil_toolchain=keil_toolchain)
    log_ok(f"SiFli-SDK profile '{lock.profile}' installed.")
    return 0


def handle_check(args: argparse.Namespace) -> int:
    config = RuntimeConfig.load(args)
    lock = ProfileLock.load(repo_root(), args.profile)
    log_banner(
        f"Checking SiFli-SDK {read_version_txt(repo_root())}",
        f"profile={lock.profile}",
    )
    plans = load_tool_plans(repo_root(), config, lock, lock.default_targets)
    reasons, resolved_env = validate_env_instance(config, lock, plans)
    if reasons:
        log_warn("Checking environment compatibility failed")
        for reason in reasons:
            print(f"  - {reason}", file=log_stream())
        return 1
    log_done("Checking environment compatibility")
    warn_non_blocking_drift(read_profile_state(config.state_path, repo_root(), lock.profile))
    log_done("Environment is ready")
    return 0


def handle_export(args: argparse.Namespace) -> int:
    toolchain = getattr(args, "toolchain", TOOLCHAIN_GCC)
    config = RuntimeConfig.load(args)
    lock = ProfileLock.load(repo_root(), args.profile)
    log_banner(
        f"Activating SiFli-SDK {read_version_txt(repo_root())}",
        f"profile={lock.profile}",
    )
    log_kv("shell", args.shell)
    log_kv("toolchain", toolchain)
    if toolchain == TOOLCHAIN_KEIL and not is_windows_host():
        raise SDKEnvError("Keil export is only supported on Windows")
    plans = load_tool_plans(repo_root(), config, lock, lock.default_targets)
    reasons, resolved_env = validate_env_instance(config, lock, plans)
    preference = auto_reconcile_preference(config, lock.profile)
    log_kv("auto reconcile", preference)

    if reasons:
        log_warn("Checking environment compatibility failed")
        for reason in reasons:
            print(f"  - {reason}", file=log_stream())
        effective_choice = preference
        if preference == "ask":
            if sys.stdin.isatty() and sys.stdout.isatty():
                effective_choice = prompt_reconcile_choice(reasons)
            else:
                effective_choice = "never"
        log_kv("effective choice", effective_choice)
        if effective_choice == "never":
            if preference == "ask" and sys.stdin.isatty() and sys.stdout.isatty():
                write_profile_state(config.state_path, repo_root(), lock.profile, auto_reconcile="never")
            raise SDKEnvError(
                f"environment instance for profile '{lock.profile}' is missing or invalid for the current checkout. "
                f"Re-run `{install_command_hint(lock.profile, args.shell)}` to prepare the SDK environment, then export again."
            )

        persist_preference = "ask" if effective_choice == "once" else effective_choice
        perform_install(
            argparse.Namespace(
                cache_dir=args.cache_dir,
                staging_dir=args.staging_dir,
                offline=args.offline,
                mirror=args.mirror,
                from_bundle=args.from_bundle,
                toolchain=toolchain,
            ),
            config,
            lock,
            lock.default_targets,
            auto_reconcile=persist_preference,
        )
        resolved_env = resolve_env_instance(config, lock)
        expected_env = resolved_env.python_env_path
        current_env = current_interpreter_env_path()
        if current_env != os.path.realpath(expected_env):
            new_python = python_executable(expected_env)
            if not os.path.exists(new_python):
                raise SDKEnvError(f"reconciled environment is missing interpreter: {new_python}")
            log_step("Restarting export with reconciled Python environment")
            os.execv(new_python, export_reexec_argv(args, lock, expected_env))
        plans = load_tool_plans(repo_root(), config, lock, lock.default_targets)
        reasons, resolved_env = validate_env_instance(config, lock, plans)
        if reasons:
            raise SDKEnvError("environment reconcile did not produce a clean install state")
    log_done("Checking environment compatibility")

    profile_state = read_profile_state(config.state_path, repo_root(), lock.profile)
    warn_non_blocking_drift(profile_state)
    write_profile_state(
        config.state_path,
        repo_root(),
        lock.profile,
        env_key_value=resolved_env.key,
        last_seen_git_head=current_git_head(repo_root()),
        touch_selected_env=True,
    )
    log_value("Identified shell", args.shell)
    env_map = build_export_environment(
        config,
        lock,
        plans,
        resolved_env.python_env_path,
        resolved_env.conan_home,
        args.shell,
        toolchain,
    )
    script_path = write_export_script(env_map, args.shell)
    log_done("Generating shell export script")
    log_value("Export script", script_path)
    print(script_path)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SiFli-SDK environment manager")
    subparsers = parser.add_subparsers(dest="command", required=True)

    install = subparsers.add_parser("install", help="Install the current profile environment")
    install.add_argument("--profile", default=DEFAULT_PROFILE)
    install.add_argument("--targets")
    install.add_argument("--keil", metavar="PATH", help="record a Windows Keil root containing ARM/ARMCLANG/bin")
    install.add_argument("--cache-dir")
    install.add_argument("--staging-dir")
    install.add_argument("--mirror")
    install.add_argument("--offline", action="store_true")
    install.add_argument("--from-bundle")
    install.add_argument("compat_args", nargs="*")

    export = subparsers.add_parser("export", help="Export shell environment for the current profile")
    export.add_argument("--profile", default=DEFAULT_PROFILE)
    export.add_argument("--shell", required=True)
    export.add_argument("-t", "--toolchain", choices=SUPPORTED_TOOLCHAINS, default=TOOLCHAIN_GCC)
    export.add_argument("--cache-dir")
    export.add_argument("--staging-dir")
    export.add_argument("--mirror")
    export.add_argument("--offline", action="store_true")
    export.add_argument("--from-bundle")

    check = subparsers.add_parser("check", help="Check whether the current profile environment is ready")
    check.add_argument("--profile", default=DEFAULT_PROFILE)
    check.add_argument("--cache-dir")
    check.add_argument("--staging-dir")
    check.add_argument("--mirror")
    check.add_argument("--offline", action="store_true")

    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "install":
            return handle_install(args)
        if args.command == "export":
            return handle_export(args)
        if args.command == "check":
            return handle_check(args)
        raise SDKEnvError(f"unsupported command '{args.command}'")
    except SDKEnvError as exc:
        log_error(str(exc))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
