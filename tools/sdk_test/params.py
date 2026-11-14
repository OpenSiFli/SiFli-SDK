# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Runner-side parameters and secrets: discovery, resolution, delivery, redaction.

A test case states *logical* names -- a fixture spec such as ``wifi_ap:5g``, or a
declared key such as ``cloud.token``. The value behind a name is machine-local:
it lives in a file outside the repository (or in the environment/CI secrets),
and it never enters a testcase or a binding. This module is the only place that
turns names into values, and the only place that knows which values are secret.

Resolution order per key, highest first:

1. ``--param KEY=VALUE`` on the command line (debugging; it lands in shell
   history, which is documented rather than prevented);
2. ``SIFLI_TEST_PARAM_<FLATTENED KEY>`` in the environment;
3. the params file;
4. a testcase default, or the fixture contract's default.

The params file is found once, from ``--params-file`` >
``SIFLI_TEST_PARAMS_FILE`` > ``~/.sifli/test-params.yaml``. Keys are flattened to
dotted names: ``fixtures`` keeps its prefix (``fixtures.<spec>.<option>``) so a
fixture option can never be confused with a declared parameter, and ``params``
keys are used exactly as written. Flattening to an environment variable name
maps every non-alphanumeric character to ``_``, so two distinct keys can collide;
that is reported rather than guessed at.

Two things are deliberately *not* done here. Values are resolved by the runner,
not by the Lab Agent, and they are delivered to the pytest subprocess through a
short-lived JSON file whose path is passed in the environment -- not through the
environment itself, which leaks into every child process and into process
listings. A secret is therefore only ever in a temporary file the runner deletes,
plus the console traffic that the design doc says is out of P1's control.
"""

from __future__ import annotations

import contextlib
import json
import os
import tempfile
import time
from dataclasses import dataclass
from dataclasses import field
from typing import Any
from typing import Dict
from typing import Iterable
from typing import Iterator
from typing import List
from typing import Mapping
from typing import Optional
from typing import Tuple
from urllib.parse import urlsplit

from .errors import SdkTestError
from .fixture_types import MISSING
from .fixture_types import FixtureTypes
from .fixture_types import RequiredIf
from .fixture_types import SOURCE_PARAMS
from .yaml_support import strict_safe_loader

#: Explicit params file. Highest-priority way to name the file.
ENV_PARAMS_FILE = "SIFLI_TEST_PARAMS_FILE"

#: Per-key override, e.g. ``SIFLI_TEST_PARAM_FIXTURES_WIFI_AP_5G_PSK``.
ENV_PARAM_PREFIX = "SIFLI_TEST_PARAM_"

#: Opt out of the loopback-only rule for runs that really do carry secrets to a
#: remote laboratory. Set to ``1`` to allow it.
ENV_ALLOW_REMOTE_SECRETS = "SIFLI_TEST_ALLOW_REMOTE_SECRETS"

#: Path of the temporary JSON the pytest subprocess reads its values from.
ENV_DATA_FILE = "SIFLI_TEST_DATA_FILE"

#: Prefix/suffix of the runner's temporary data files. Used for stale cleanup.
DATA_FILE_PREFIX = "sifli-test-data-"
DATA_FILE_SUFFIX = ".json"

#: A crash or a kill leaves the temporary file behind; one this old is stale.
STALE_DATA_FILE_SECONDS = 24 * 60 * 60

#: Values shorter than this still get redacted, but are flagged: replacing a
#: two-character secret throughout a log destroys the evidence around it.
MIN_REDACTABLE_SECRET = 6

#: What a redacted value becomes. Fixed text, so a reader can tell a secret was
#: removed from a value that merely looked odd.
REDACTED = "***"

#: Hosts treated as "this machine". A secrets-carrying run defaults to these.
LOOPBACK_HOSTS = frozenset({"127.0.0.1", "::1", "localhost"})


class ParamsError(SdkTestError):
    """Parameters are missing, ambiguous, unreadable, or unsafe to transmit."""


@dataclass(frozen=True)
class ParamValue:
    """One resolved parameter, with where it came from and whether it is secret."""

    key: str
    value: Any
    source: str
    secret: bool = False


@dataclass
class ResolvedParams:
    """Everything one test instance's ``params``/``fixture_options`` can return."""

    values: Dict[str, ParamValue] = field(default_factory=dict)
    #: Fixture specs this instance requested, in declaration order.
    specs: List[str] = field(default_factory=list)
    #: ``spec -> option name -> flat key``. Recorded rather than re-derived from
    #: the key prefix, because one spec may be a prefix of another and a prefix
    #: scan would attribute the longer spec's options to the shorter one.
    fixture_keys: Dict[str, Dict[str, str]] = field(default_factory=dict)
    #: The file values were read from, or ``None`` when no file was found.
    params_file: Optional[str] = None
    #: Where the file was looked for, for a message that can be acted on.
    search_paths: List[str] = field(default_factory=list)
    #: Non-fatal diagnostics (a secret too short to redact safely).
    warnings: List[str] = field(default_factory=list)
    #: Flat keys this instance asked for, in requirement order.
    requested_keys: List[str] = field(default_factory=list)

    @property
    def secrets(self) -> List[str]:
        """Values marked secret, longest first, for redaction."""
        found = [
            str(entry.value)
            for entry in self.values.values()
            if entry.secret and entry.value is not None and str(entry.value)
        ]
        return sorted(dict.fromkeys(found), key=len, reverse=True)

    def _fixture_key_set(self) -> set:
        return {
            key
            for options in self.fixture_keys.values()
            for key in options.values()
        }

    def params(self) -> Dict[str, Any]:
        """Declared parameters (everything that is not a fixture option)."""
        fixture = self._fixture_key_set()
        return {
            key: entry.value
            for key, entry in self.values.items()
            if key not in fixture
        }

    def fixtures(self) -> Dict[str, Dict[str, Any]]:
        """Resolved options per requested fixture spec.

        Every requested spec is present, an empty mapping when its type has no
        contract (and therefore no options the framework knows how to fill).
        """
        resolved: Dict[str, Dict[str, Any]] = {}
        for spec in self.specs:
            options = {}
            for option, key in self.fixture_keys.get(spec, {}).items():
                entry = self.values.get(key)
                if entry is not None:
                    options[option] = entry.value
            resolved[spec] = options
        return resolved

    def payload(self) -> Dict[str, Any]:
        """The document the pytest subprocess reads."""
        return {"params": self.params(), "fixtures": self.fixtures()}

    def sources(self) -> Dict[str, str]:
        """Key to source, for ``params check`` -- never the value."""
        return {key: entry.source for key, entry in self.values.items()}


# -- file discovery and flattening -------------------------------------------


def default_params_path(environ: Optional[Mapping[str, str]] = None) -> str:
    """``%USERPROFILE%\\.sifli\\test-params.yaml`` (Windows) or ``~/.sifli/...``."""
    current = os.environ if environ is None else environ
    home = current.get("USERPROFILE") or current.get("HOME") or os.path.expanduser("~")
    return os.path.join(home, ".sifli", "test-params.yaml")


def find_params_file(
    explicit: Optional[str] = None,
    environ: Optional[Mapping[str, str]] = None,
) -> Tuple[Optional[str], List[str]]:
    """Find the params file, and record where it was looked for.

    Only the first existing file is used. A file named explicitly -- by flag or by
    environment variable -- that does not exist is an error rather than a reason
    to fall through: the caller said where the secrets are, and silently reading
    a different file would be worse than failing.

    :return: ``(path or None, search paths to quote in a failure message)``.
    :raises ParamsError: an explicitly named file does not exist.
    """
    current = dict(os.environ) if environ is None else dict(environ)
    searched: List[str] = []

    if explicit:
        path = os.path.abspath(explicit)
        searched.append(f"--params-file {path}")
        if not os.path.isfile(path):
            raise ParamsError(
                f"the params file given with --params-file does not exist: {path}"
            )
        return path, searched
    searched.append("--params-file (not given)")

    named = current.get(ENV_PARAMS_FILE)
    if named:
        path = os.path.abspath(named)
        searched.append(f"{ENV_PARAMS_FILE}={path}")
        if not os.path.isfile(path):
            raise ParamsError(
                f"{ENV_PARAMS_FILE} points at a file that does not exist: {path}"
            )
        return path, searched
    searched.append(f"{ENV_PARAMS_FILE} (not set)")

    default = default_params_path(current)
    if os.path.isfile(default):
        searched.append(default)
        return default, searched
    searched.append(f"{default} (not found)")
    return None, searched


def flatten_env_name(key: str) -> str:
    """The environment variable that overrides ``key``.

    Every character that is not an ASCII letter or digit becomes ``_`` and the
    result is upper-cased, so ``fixtures.wifi_ap:5g.psk`` is overridden by
    ``SIFLI_TEST_PARAM_FIXTURES_WIFI_AP_5G_PSK``.
    """
    flattened = "".join(
        char if char.isascii() and char.isalnum() else "_" for char in key
    ).upper()
    return ENV_PARAM_PREFIX + flattened


def check_key_collisions(keys: Iterable[str]) -> None:
    """Refuse keys that flatten to the same environment variable.

    ``wifi_ap:5g`` and ``wifi_ap_5g`` differ only in a character that flattening
    removes; guessing which one an override meant would make the value depend on
    key order.
    """
    seen: Dict[str, str] = {}
    for key in keys:
        name = flatten_env_name(key)
        previous = seen.get(name)
        if previous is not None and previous != key:
            raise ParamsError(
                f'parameter keys "{previous}" and "{key}" both flatten to "{name}"; '
                "rename one so a per-key override is unambiguous"
            )
        seen[name] = key


def _is_value_record(value: Any) -> bool:
    return isinstance(value, dict) and "value" in value and set(value) <= {"value", "secret"}


def _flatten(section: Mapping[str, Any], prefix: str, where: str
             ) -> Iterator[Tuple[str, Any, bool]]:
    if not isinstance(section, dict):
        raise ParamsError(f'{where} must be a mapping of keys to values')
    for key, raw in section.items():
        full = f"{prefix}.{key}" if prefix else str(key)
        if _is_value_record(raw):
            yield full, raw.get("value"), bool(raw.get("secret", False))
        elif isinstance(raw, dict):
            yield from _flatten(raw, full, where)
        else:
            yield full, raw, False


def load_params_file(path: str) -> Dict[str, ParamValue]:
    """Read a params file into flattened keys.

    The document has at most two sections: ``fixtures`` (spec to options) and
    ``params`` (declared keys). A value may be a scalar or ``{value, secret}``.
    Unknown top-level sections are rejected: a typo such as ``fixture:`` would
    otherwise resolve nothing and read as "the value is missing".
    """
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - pyyaml is a core requirement
        raise ParamsError("pyyaml is required to read the params file") from exc

    try:
        with open(path, "r", encoding="utf-8") as handle:
            raw = yaml.load(
                handle, Loader=strict_safe_loader(yaml, path, ParamsError)
            )
    except OSError as exc:
        raise ParamsError(f"Cannot read the params file {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ParamsError(f"{path}: malformed YAML: {exc}") from exc

    if raw is None:
        return {}
    if not isinstance(raw, dict):
        raise ParamsError(f'{path}: top level must be a mapping with "fixtures"/"params"')

    unknown = sorted(set(raw) - {"fixtures", "params"})
    if unknown:
        raise ParamsError(
            f"{path}: unknown section(s) {', '.join(unknown)}; "
            'expected "fixtures" and/or "params"'
        )

    values: Dict[str, ParamValue] = {}

    def add(key: str, value: Any, secret: bool) -> None:
        if key in values:
            raise ParamsError(f'{path}: parameter "{key}" is defined more than once')
        values[key] = ParamValue(key=key, value=value, source=path, secret=secret)

    for key, value, secret in _flatten(raw.get("fixtures") or {}, "fixtures", f"{path}:fixtures"):
        add(key, value, secret)
    for key, value, secret in _flatten(raw.get("params") or {}, "", f"{path}:params"):
        add(key, value, secret)
    return values


# -- requirements and resolution ---------------------------------------------


@dataclass(frozen=True)
class _Requirement:
    """One key an instance needs, and the rules that decide whether it is required."""

    key: str
    label: str
    required: bool = False
    required_if: Optional[RequiredIf] = None
    secret: bool = False
    default: Any = MISSING
    default_source: str = ""
    #: Allowed values declared by the contract; empty means unrestricted.
    enum: Tuple[str, ...] = ()
    #: Set for fixture options: the requested spec and the option name, which is
    #: what ``required_if`` is evaluated against.
    spec: str = ""
    option_name: str = ""
    from_contract: bool = False


def _requirements(instance: Any, contracts: FixtureTypes) -> List[_Requirement]:
    """Derive every key an instance needs, without reading any file.

    Keys come from two places: the ``params:`` block the testcase declares, and
    the contract of each fixture it requests (expanded with the requested spec
    exactly as written, so a comma-qualified spec is not confused with another).
    """
    requirements: List[_Requirement] = []

    declared = getattr(instance, "params", None) or {}
    if not isinstance(declared, dict):
        raise ParamsError(f'"{getattr(instance, "id", "?")}": params must be a mapping')
    for key, declaration in declared.items():
        key = str(key)
        if isinstance(declaration, dict):
            required = bool(declaration.get("required", False))
            secret = bool(declaration.get("secret", False))
            default: Any = MISSING
            source = ""
        else:
            required = False
            secret = False
            default = declaration
            source = "default (testcase)"
        requirements.append(
            _Requirement(
                key=key,
                label=f'params "{key}"',
                required=required,
                secret=secret,
                default=default,
                default_source=source,
            )
        )

    for ref in getattr(instance, "fixtures", None) or []:
        contract = contracts.get(ref.name)
        if contract is None:
            continue
        for option in contract.options:
            if option.source != SOURCE_PARAMS:
                continue
            requirements.append(
                _Requirement(
                    key=option.key_for(ref.spec),
                    label=f'fixture "{ref.spec}" option "{option.name}"',
                    required=option.required,
                    required_if=option.required_if,
                    secret=option.secret,
                    default=option.default if option.has_default else MISSING,
                    default_source=f"default (contract {contract.name})",
                    enum=option.enum,
                    spec=ref.spec,
                    option_name=option.name,
                    from_contract=True,
                )
            )

    return _merge_requirements(requirements)


def _merge_requirements(requirements: List[_Requirement]) -> List[_Requirement]:
    """Collapse duplicate keys, keeping the strictest requirement.

    A testcase may declare a params key that is also a fixture option. Both name
    one value, so the merged entry is required if either says so, and is secret
    if the contract says so.
    """
    merged: Dict[str, _Requirement] = {}
    order: List[str] = []
    for entry in requirements:
        existing = merged.get(entry.key)
        if existing is None:
            merged[entry.key] = entry
            order.append(entry.key)
            continue
        default = existing.default
        default_source = existing.default_source
        if default is MISSING and entry.default is not MISSING:
            default = entry.default
            default_source = entry.default_source
        merged[entry.key] = _Requirement(
            key=entry.key,
            label=existing.label if existing.from_contract else entry.label,
            required=existing.required or entry.required,
            required_if=existing.required_if or entry.required_if,
            secret=existing.secret or entry.secret,
            default=default,
            default_source=default_source,
            enum=existing.enum or entry.enum,
            spec=existing.spec or entry.spec,
            option_name=existing.option_name or entry.option_name,
            from_contract=existing.from_contract or entry.from_contract,
        )
    return [merged[key] for key in order]


def requirement_keys(instance: Any, contracts: FixtureTypes) -> List[str]:
    """Every flat key ``instance`` needs, derived without reading a params file."""
    return [entry.key for entry in _requirements(instance, contracts)]


def _lookup(
    requirement: _Requirement,
    path: Optional[str],
    file_values: Mapping[str, ParamValue],
    overrides: Mapping[str, str],
    environ: Mapping[str, str],
) -> Optional[ParamValue]:
    """Resolve one key by the documented precedence, or ``None``."""
    file_entry = file_values.get(requirement.key)
    # ``secret`` is raised by the contract, by the testcase declaration, by the
    # file's own marker (for keys no contract describes), or by any combination
    # of them. The value's *source* must not matter: a secret supplied through
    # the environment is as secret as one read from the file.
    secret = requirement.secret or (
        not requirement.from_contract
        and file_entry is not None
        and file_entry.secret
    )

    if requirement.key in overrides:
        return ParamValue(requirement.key, overrides[requirement.key], "--param", secret)

    env_name = flatten_env_name(requirement.key)
    # Presence, not truthiness: an empty string is an explicit value and must
    # not silently fall through to the file.
    if env_name in environ:
        return ParamValue(
            requirement.key, environ[env_name], f"environment ({env_name})", secret
        )

    if file_entry is not None:
        return ParamValue(requirement.key, file_entry.value, str(path), secret)

    if requirement.default is not MISSING:
        return ParamValue(
            requirement.key, requirement.default, requirement.default_source, secret
        )
    return None


def is_loopback_agent(agent_url: Optional[str]) -> bool:
    """Whether ``agent_url`` names this machine."""
    if not agent_url:
        return True
    try:
        host = urlsplit(agent_url).hostname
    except ValueError:
        return False
    return (host or "").lower() in LOOPBACK_HOSTS


def _enforce_local_agent(
    agent_url: Optional[str],
    secret_keys: Iterable[str],
    environ: Mapping[str, str],
) -> None:
    """Refuse to carry secrets to a remote agent unless explicitly allowed.

    P1 transmits a secret through the agent's unauthenticated HTTP console, so
    the default exposure is "this machine only". The variable exists because a
    real lab may need it; setting it is an informed decision, not a bypass that
    happens silently.
    """
    names = list(secret_keys)
    if not names:
        return
    if environ.get(ENV_ALLOW_REMOTE_SECRETS) == "1":
        return
    if is_loopback_agent(agent_url):
        return
    raise ParamsError(
        f"refusing to send {len(names)} secret parameter(s) to a non-loopback "
        f"Lab Agent ({agent_url}): {', '.join(names)}. Secret values travel in "
        "clear text over the agent's unauthenticated HTTP. Use a local agent, or "
        f"set {ENV_ALLOW_REMOTE_SECRETS}=1 to accept the risk deliberately."
    )


def resolve_params(
    instance: Any,
    contracts: FixtureTypes,
    *,
    params_file: Optional[str] = None,
    overrides: Optional[Mapping[str, str]] = None,
    environ: Optional[Mapping[str, str]] = None,
    agent_url: Optional[str] = None,
    uses_agent: bool = True,
) -> ResolvedParams:
    """Resolve every key ``instance`` needs, or raise explaining what is missing.

    :param instance: a :class:`sdk_test.model.TestInstance`.
    :param contracts: the fixture type contracts.
    :param params_file: explicit file, as ``--params-file`` would name it.
    :param overrides: ``--param`` values, keyed by flat key.
    :param environ: environment to read; defaults to ``os.environ``.
    :param agent_url: the Lab Agent the values would be sent through.
    :param uses_agent: whether this instance talks to an agent at all; a host
        backend has nobody to send anything to, so the loopback rule does not
        apply.
    :raises ParamsError: a required key is missing, the file is ambiguous, or a
        secret would leave this machine without permission.
    """
    current = dict(os.environ) if environ is None else dict(environ)
    chosen = dict(overrides or {})
    requirements = _requirements(instance, contracts)
    requested_keys = [entry.key for entry in requirements]

    path, searched = find_params_file(params_file, current)
    file_values = load_params_file(path) if path else {}

    check_key_collisions(requested_keys + list(file_values))

    resolved: Dict[str, ParamValue] = {}
    option_values: Dict[str, Dict[str, Any]] = {}
    fixture_keys: Dict[str, Dict[str, str]] = {}
    for requirement in requirements:
        if requirement.spec and requirement.option_name:
            fixture_keys.setdefault(requirement.spec, {})[
                requirement.option_name
            ] = requirement.key
        entry = _lookup(requirement, path, file_values, chosen, current)
        if entry is None:
            continue
        resolved[requirement.key] = entry
        if requirement.spec and requirement.option_name:
            option_values.setdefault(requirement.spec, {})[
                requirement.option_name
            ] = entry.value

    # ``required_if`` is evaluated against an already-resolved sibling option.
    # When that sibling is itself unresolved -- the contract guarantees it is
    # either required or defaulted, so this means it is missing -- the condition
    # cannot be evaluated, and guessing would report the conditional option
    # instead of the missing dependency. Such conditions are skipped here, and
    # the dependency's own requirement reports it, naming what was waiting.
    option_requirements = {
        (entry.spec, entry.option_name): entry
        for entry in requirements
        if entry.spec and entry.option_name
    }
    unevaluable = set()
    waiting_on: Dict[str, List[str]] = {}
    for requirement in requirements:
        condition = requirement.required_if
        if condition is None:
            continue
        if condition.option in option_values.get(requirement.spec, {}):
            continue
        unevaluable.add(requirement.key)
        dependency = option_requirements.get((requirement.spec, condition.option))
        if dependency is not None:
            waiting_on.setdefault(dependency.key, []).append(requirement.key)

    missing: List[str] = []
    for requirement in requirements:
        if requirement.key in resolved or requirement.key in unevaluable:
            continue
        required = requirement.required
        because = ""
        if requirement.required_if is not None:
            required = requirement.required_if.evaluate(
                option_values.get(requirement.spec, {})
            )
            if required:
                because = f", required because {requirement.required_if}"
        if not required:
            continue
        dependents = waiting_on.get(requirement.key)
        if dependents:
            because += (
                ", its value is needed to evaluate the condition of "
                + ", ".join(f"{key}.required_if" for key in dependents)
            )
        missing.append(f"  - {requirement.key}  ({requirement.label}{because})")

    if missing:
        overrides_hint = "\n".join(
            f"  {flatten_env_name(entry.key)}  or  --param {entry.key}=<value>"
            for entry in requirements
        )
        raise ParamsError(
            f'"{getattr(instance, "id", "?")}": missing required parameter(s):\n'
            + "\n".join(missing)
            + "\nsearched for a params file:\n  "
            + "\n  ".join(searched)
            + "\nper-key overrides that would supply a value:\n"
            + (overrides_hint or "  (none)")
        )

    invalid: List[str] = []
    for requirement in requirements:
        entry = resolved.get(requirement.key)
        if entry is None or not requirement.enum:
            continue
        # A contract enum is a list of strings, so a non-string value is not a
        # member of it -- coercing would accept a value the contract rejects.
        if isinstance(entry.value, str) and entry.value in requirement.enum:
            continue
        # The offending value is echoed only when it is not a secret; a secret
        # option with an enum is unlikely, but the rule holds either way.
        shown = "" if entry.secret else f" (got {entry.value!r} from {entry.source})"
        invalid.append(
            f"  - {requirement.key} must be one of {list(requirement.enum)}{shown}"
        )
    if invalid:
        raise ParamsError(
            f'"{getattr(instance, "id", "?")}": parameter value(s) outside the '
            "contract's enum:\n" + "\n".join(invalid)
        )

    # The values are handed to pytest through JSON, so anything the encoder
    # cannot represent (a YAML date, a mapping) must fail here rather than after
    # a build and a flash. The value itself is not echoed.
    unserializable: List[str] = []
    for key, entry in resolved.items():
        try:
            json.dumps(entry.value)
        except (TypeError, ValueError):
            unserializable.append(f"  - {key} ({type(entry.value).__name__})")
    if unserializable:
        raise ParamsError(
            f'"{getattr(instance, "id", "?")}": parameter value(s) cannot be '
            "delivered as JSON:\n" + "\n".join(unserializable)
        )

    warnings: List[str] = []
    secret_keys: List[str] = []
    for key, entry in resolved.items():
        if not entry.secret:
            continue
        secret_keys.append(key)
        text = "" if entry.value is None else str(entry.value)
        if text and len(text) < MIN_REDACTABLE_SECRET:
            warnings.append(
                f'secret "{key}" is shorter than {MIN_REDACTABLE_SECRET} '
                "characters; it is still redacted, but may also match unrelated text"
            )

    if uses_agent:
        _enforce_local_agent(agent_url, secret_keys, current)

    return ResolvedParams(
        values=resolved,
        specs=[ref.spec for ref in getattr(instance, "fixtures", None) or []],
        fixture_keys=fixture_keys,
        params_file=path,
        search_paths=searched,
        warnings=warnings,
        requested_keys=requested_keys,
    )


def unused_keys(all_keys: Iterable[str], used_keys: Iterable[str]) -> List[str]:
    """File keys no selected test case refers to, sorted."""
    used = set(used_keys)
    return sorted(key for key in all_keys if key not in used)


# -- delivery to the pytest subprocess ---------------------------------------


def write_data_file(resolved: ResolvedParams, directory: Optional[str] = None) -> str:
    """Write the resolved values to a short-lived JSON file and return its path.

    ``mkstemp`` creates the file with a restrictive mode on POSIX and in the
    user's temporary directory otherwise; on Windows the protection comes from
    the profile's directory ACL, and this framework does not claim ``0600``
    there. The file is deliberately *not* written under ``test-out/``, which is
    uploaded as a CI artifact.
    """
    handle_fd, path = tempfile.mkstemp(
        prefix=DATA_FILE_PREFIX, suffix=DATA_FILE_SUFFIX, dir=directory
    )
    try:
        with os.fdopen(handle_fd, "w", encoding="utf-8") as handle:
            json.dump(resolved.payload(), handle, ensure_ascii=False, indent=2)
    except BaseException:
        # A half-written file must not outlive a failure; the caller reports the
        # write problem instead.
        remove_data_file(path)
        raise
    return path


def remove_data_file(path: Optional[str]) -> bool:
    """Delete a data file. Missing is success; only a real failure returns False."""
    if not path:
        return True
    try:
        os.remove(path)
    except FileNotFoundError:
        return True
    except OSError:
        return False
    return True


def cleanup_stale_data_files(
    directory: Optional[str] = None,
    max_age: float = STALE_DATA_FILE_SECONDS,
    now: Optional[float] = None,
) -> List[str]:
    """Delete data files older than ``max_age``, and return what was removed.

    A crashed or killed runner leaves its temporary file behind. Cleaning at
    start-up bounds how long a plaintext secret can survive such a crash. Errors
    are per-file and non-fatal: a file another live runner is using is not this
    run's problem.
    """
    target = directory or tempfile.gettempdir()
    current = time.time() if now is None else now
    removed: List[str] = []
    try:
        entries = list(os.scandir(target))
    except OSError:
        return removed

    for entry in entries:
        if not (
            entry.name.startswith(DATA_FILE_PREFIX)
            and entry.name.endswith(DATA_FILE_SUFFIX)
        ):
            continue
        try:
            if current - entry.stat().st_mtime <= max_age:
                continue
            os.remove(entry.path)
        except OSError:
            continue
        removed.append(entry.path)
    return sorted(removed)


@contextlib.contextmanager
def stripped_param_environment() -> Iterator[None]:
    """Temporarily remove ``SIFLI_TEST_PARAM_*`` from this process's environment.

    The build subprocess inherits ``os.environ``; a build never needs a test
    parameter, and this design explicitly does not support compile-time secret
    injection. Stripping here -- rather than changing the build helper's
    contract -- keeps the compile step unaware of the parameter mechanism.
    """
    saved = {
        key: os.environ.pop(key)
        for key in [key for key in os.environ if key.startswith(ENV_PARAM_PREFIX)]
    }
    try:
        yield
    finally:
        os.environ.update(saved)


# -- redaction -----------------------------------------------------------------


def redact_text(text: str, secrets: Iterable[str]) -> str:
    """Replace every secret value in ``text``.

    Longest first, so one secret that contains another is removed whole instead
    of leaving the tail of the longer one behind.
    """
    if not text:
        return text
    for secret in sorted(dict.fromkeys(str(item) for item in secrets if item), key=len, reverse=True):
        if secret and secret in text:
            text = text.replace(secret, REDACTED)
    return text


def _redact_case(case: Any, secrets: Iterable[str]) -> None:
    message = getattr(case, "message", None)
    if isinstance(message, str) and message:
        try:
            setattr(case, "message", redact_text(message, secrets))
        except AttributeError:  # frozen case objects simply keep their text
            pass


def redact_result(result: Any, secrets: Iterable[str]) -> None:
    """Redact every field of a :class:`sdk_test.report.TestResult` in memory.

    The logs and the report are projections of these fields, so doing it here --
    before anything is written -- covers ``device.log``, ``runner.log``,
    ``<fixture>.log``, ``sifli.json`` and ``sifli.xml`` at once. It also covers
    the failure path, where an ``expect`` timeout folds the device transcript
    into the assertion message.
    """
    if not secrets:
        return
    for name in ("message", "log", "transcript", "build_output"):
        value = getattr(result, name, "")
        if isinstance(value, str) and value:
            setattr(result, name, redact_text(value, secrets))
    instrument_logs = getattr(result, "instrument_logs", None)
    if isinstance(instrument_logs, dict):
        for key, value in list(instrument_logs.items()):
            if isinstance(value, str) and value:
                instrument_logs[key] = redact_text(value, secrets)
    warnings = getattr(result, "capture_warnings", None)
    if isinstance(warnings, list):
        result.capture_warnings = [redact_text(item, secrets) for item in warnings]
    for case in getattr(result, "cases", None) or []:
        _redact_case(case, secrets)


def redact_outcome(outcome: Any, secrets: Iterable[str]) -> None:
    """Redact a harness outcome before the runner files its log.

    ``runner.log`` carries the harness's own output verbatim, so redacting only
    the result would leave the unredacted copy in the file.
    """
    if outcome is None or not secrets:
        return
    for name in ("log", "detail"):
        value = getattr(outcome, name, "")
        if isinstance(value, str) and value:
            setattr(outcome, name, redact_text(value, secrets))
    for case in getattr(outcome, "cases", None) or []:
        _redact_case(case, secrets)


__all__ = [
    "DATA_FILE_PREFIX",
    "DATA_FILE_SUFFIX",
    "ENV_ALLOW_REMOTE_SECRETS",
    "ENV_DATA_FILE",
    "ENV_PARAMS_FILE",
    "ENV_PARAM_PREFIX",
    "MIN_REDACTABLE_SECRET",
    "ParamValue",
    "ParamsError",
    "REDACTED",
    "ResolvedParams",
    "STALE_DATA_FILE_SECONDS",
    "check_key_collisions",
    "cleanup_stale_data_files",
    "default_params_path",
    "find_params_file",
    "flatten_env_name",
    "is_loopback_agent",
    "load_params_file",
    "redact_outcome",
    "redact_result",
    "redact_text",
    "remove_data_file",
    "requirement_keys",
    "resolve_params",
    "stripped_param_environment",
    "unused_keys",
    "write_data_file",
]
