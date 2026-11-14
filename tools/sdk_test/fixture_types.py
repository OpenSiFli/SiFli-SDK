# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Fixture type contracts: the options a fixture type has and where they come from.

A fixture used to be a capability only -- a name and a comma-separated list --
and the rest of a connection entry was the fixture driver's own business (see
:mod:`sdk_test.binding`). That is still true for every type *without* an entry
here. A type that a test refers to *by name*, and whose values the framework has
to fetch before the test runs, needs one thing more: a statement of which
options exist and where each one comes from. That statement is this contract,
and it is the only place option names are defined.

The file carries two layers, and keeping them apart matters:

* the **type catalogue** -- which fixture types exist and which capability
  tokens each one accepts. This is framework knowledge: it answers "what may a
  test author write", lives in the repository, and is reviewed with the code.
  An entry may state only a description and a capability list;
* the **option contract** -- which options a type has and where their values
  come from. Only a type with at least one ``source: params`` option is
  *parameterized*, and only for it does the spec become an identity rather than
  a capability list.

How many instances a lab actually has, and which port each is wired to, is the
binding's business and is never written here. A type with no entry at all is
still permitted and unchecked, which keeps a lab free to define its own.

The contract is deliberately small. It says:

* which capability tokens the type accepts (optional, and only a vocabulary --
  an unlisted token in a binding warns rather than fails);
* which options the type has;
* whether an option's value comes from the runner-side params file
  (``source: params``) or from the binding (``source: binding``, reserved and
  not implemented in P1);
* the params key template, expanded with a single ``{spec}`` substitution;
* whether an option is always required, conditionally required, secret, or has
  a default.

Everything else -- SSIDs, passwords, which lab's AP it is -- belongs to the
runner-side params file, which is never committed. A binding therefore states
capabilities only, and a value cannot leak into it by accident: an option the
contract sources from params is rejected in a binding
(:func:`sdk_test.binding.check_fixture_contracts`).

Loading is strict because this file ships with the code: an unknown field, an
unknown ``{placeholder}``, an unsupported ``required_if`` expression or an
option name that collides with a binding key is a mistake to fix, not a value to
silently ignore.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass
from typing import Any
from typing import Dict
from typing import List
from typing import Mapping
from typing import Optional
from typing import Tuple

from .errors import SdkTestError
from .yaml_support import strict_safe_loader

CONTRACT_FILENAME = "fixture_types.yaml"

#: The contract shipped with the framework. A caller may pass another path, which
#: is what lets a test exercise the loader against a hand-written document.
DEFAULT_CONTRACT_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), CONTRACT_FILENAME
)

#: Only this source is implemented in P1.
SOURCE_PARAMS = "params"

#: Reserved for values that really belong to the lab; loading one is an error.
SOURCE_BINDING = "binding"

#: Keys a connection already means something by. An option may not shadow one.
RESERVED_OPTION_NAMES = frozenset({"fixture", "pinmap", "mux"})

_NAME_RE = re.compile(r"^[a-z][a-z0-9_]*$")
#: Capability tokens are short words a test writes after ``:``. Digits and a
#: leading digit are allowed (``5g``), and so are the separators a lab is
#: likely to use to name a bench (``lab_a``, ``v1.2``). Whatever a driver can
#: parse is not this file's business; that it is one token is.
_CAPABILITY_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_PLACEHOLDER_RE = re.compile(r"\{([^{}]*)\}")
_SPEC_PLACEHOLDER = "spec"

#: ``required_if`` accepts one shape only. A general expression evaluator was
#: deliberately not introduced: one comparison covers the real cases, and a
#: second mini-language would need its own tests and its own documentation.
_REQUIRED_IF_RE = re.compile(
    r"^(?P<option>[a-z][a-z0-9_]*)\s*(?P<operator>==|!=)\s*(?P<value>[^\s\"']+)$"
)

_TYPE_FIELDS = frozenset({"description", "capabilities", "options"})
_OPTION_FIELDS = frozenset(
    {"source", "key", "required", "required_if", "secret", "enum", "default"}
)

#: Sentinel distinguishing "no default given" from an explicit ``default: null``.
MISSING = object()


class FixtureTypeError(SdkTestError):
    """A fixture type contract is malformed or uses a feature P1 does not have."""


@dataclass(frozen=True)
class RequiredIf:
    """A condition on a sibling option, from ``required_if``.

    Only ``<option> ==|!= <literal>`` is accepted. The left side is an option of
    the same fixture type; the right side is a string literal, unquoted.
    """

    option: str
    operator: str
    value: str

    def evaluate(self, values: Mapping[str, Any]) -> bool:
        """Whether the condition holds for the resolved sibling ``values``."""
        equal = str(values.get(self.option)) == self.value
        return equal if self.operator == "==" else not equal

    def __str__(self) -> str:  # pragma: no cover - message building only
        return f"{self.option} {self.operator} {self.value}"


@dataclass(frozen=True)
class FixtureOption:
    """One option of a fixture type, and where its value comes from."""

    name: str
    source: str
    #: Params key template, expanded with a single ``{spec}`` substitution.
    key: str = ""
    required: bool = False
    required_if: Optional[RequiredIf] = None
    secret: bool = False
    enum: Tuple[str, ...] = ()
    default: Any = MISSING

    @property
    def has_default(self) -> bool:
        return self.default is not MISSING

    def key_for(self, spec: str) -> str:
        """The params key for this option, for the requested ``spec``."""
        return self.key.replace("{" + _SPEC_PLACEHOLDER + "}", spec)


@dataclass(frozen=True)
class FixtureType:
    """The contract of one fixture type."""

    name: str
    description: str = ""
    #: The type's closed capability vocabulary, if it has one. Empty means the
    #: part after ``:`` is not a capability list for this type (a bus name, as
    #: in ``spi_nor_flash:spi1``), so it is not checked.
    capabilities: Tuple[str, ...] = ()
    options: Tuple[FixtureOption, ...] = ()

    def option(self, name: str) -> Optional[FixtureOption]:
        for entry in self.options:
            if entry.name == name:
                return entry
        return None

    @property
    def parameterized(self) -> bool:
        """Whether this type's values come from params (and it is thus spec-identified)."""
        return any(option.source == SOURCE_PARAMS for option in self.options)

    @property
    def option_names(self) -> Tuple[str, ...]:
        return tuple(option.name for option in self.options)

    @property
    def params_option_names(self) -> Tuple[str, ...]:
        return tuple(
            option.name
            for option in self.options
            if option.source == SOURCE_PARAMS
        )


@dataclass(frozen=True)
class FixtureTypes:
    """Every loaded contract, plus the lookups the rest of the framework needs."""

    types: Mapping[str, FixtureType]
    source: str = "<memory>"

    def get(self, name: str) -> Optional[FixtureType]:
        return self.types.get(name)

    def is_parameterized(self, name: str) -> bool:
        """Whether ``name`` is a known type whose values come from params."""
        contract = self.types.get(name)
        return bool(contract and contract.parameterized)

    def option_names(self, name: str) -> Tuple[str, ...]:
        contract = self.types.get(name)
        return contract.option_names if contract else ()

    def __contains__(self, name: object) -> bool:
        return name in self.types


# -- validation --------------------------------------------------------------


def _mapping(value: Any, where: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise FixtureTypeError(f"{where} must be a mapping, got {type(value).__name__}")
    return value


def _boolean(value: Any, where: str) -> bool:
    if not isinstance(value, bool):
        raise FixtureTypeError(f"{where} must be true or false")
    return value


def _name(value: Any, where: str) -> str:
    if not isinstance(value, str) or not _NAME_RE.match(value):
        raise FixtureTypeError(
            f'{where} must be lower_case_with_underscores, got {value!r}'
        )
    if value in RESERVED_OPTION_NAMES:
        raise FixtureTypeError(
            f'{where}: "{value}" is a binding key and cannot be an option name'
        )
    if "." in value:
        raise FixtureTypeError(f'{where}: "{value}" must not contain a dot')
    return value


def _reject_unknown(mapping: Mapping[str, Any], allowed: frozenset, where: str) -> None:
    unknown = sorted(set(mapping) - allowed)
    if unknown:
        raise FixtureTypeError(
            f"{where}: unknown field(s) {', '.join(unknown)}; "
            f"allowed: {', '.join(sorted(allowed))}"
        )


def _parse_required_if(value: Any, where: str,
                       options: Mapping[str, Any]) -> RequiredIf:
    if not isinstance(value, str):
        raise FixtureTypeError(f'{where} must be a string such as "security != open"')
    match = _REQUIRED_IF_RE.match(value.strip())
    if not match:
        raise FixtureTypeError(
            f'{where}: {value!r} is not supported; the only accepted form is '
            '"<option> ==|!= <unquoted literal>"'
        )
    target = match.group("option")
    if target not in options:
        raise FixtureTypeError(
            f'{where}: "{target}" is not an option of this fixture type '
            f"(it has: {', '.join(sorted(options))})"
        )
    return RequiredIf(target, match.group("operator"), match.group("value"))


def _parse_capabilities(value: Any, where: str) -> Tuple[str, ...]:
    """Read a type's capability vocabulary, which is a list of tokens."""
    if not isinstance(value, list):
        raise FixtureTypeError(
            f"{where} must be a list of capability tokens, got "
            f"{type(value).__name__}"
        )
    tokens: List[str] = []
    for entry in value:
        if not isinstance(entry, str) or not _CAPABILITY_RE.match(entry):
            raise FixtureTypeError(
                f"{where}: {entry!r} is not a capability token; use letters, "
                "digits, '_', '.' and '-', starting with a letter or digit "
                "(e.g. ble, 5g, lab_a)"
            )
        tokens.append(entry)
    return tuple(tokens)


def _parse_key(value: Any, where: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise FixtureTypeError(f"{where} must be a non-empty key template")
    for placeholder in _PLACEHOLDER_RE.findall(value):
        if placeholder != _SPEC_PLACEHOLDER:
            raise FixtureTypeError(
                f'{where}: unknown placeholder "{{{placeholder}}}"; the only one '
                f"supported is {{{_SPEC_PLACEHOLDER}}} (substituted once, no recursion)"
            )
    # Remove every complete placeholder; a brace left behind is a typo that
    # would be substituted into a params key and then never match.
    remainder = _PLACEHOLDER_RE.sub("", value)
    if "{" in remainder or "}" in remainder:
        raise FixtureTypeError(
            f"{where}: {value!r} has an unmatched '{{' or '}}'; a placeholder "
            f"must be written {{{_SPEC_PLACEHOLDER}}}"
        )
    return value


def _parse_option(name: str, raw: Any, where: str,
                  siblings: Mapping[str, Any]) -> FixtureOption:
    mapping = _mapping(raw, where)
    _reject_unknown(mapping, _OPTION_FIELDS, where)

    source = mapping.get("source")
    if source is None:
        raise FixtureTypeError(f'{where} needs a "source"')
    if source == SOURCE_BINDING:
        raise FixtureTypeError(
            f'{where}: "source: binding" is not implemented; only '
            f'"source: {SOURCE_PARAMS}" is supported in this milestone'
        )
    if source != SOURCE_PARAMS:
        raise FixtureTypeError(
            f'{where}: unknown source {source!r}; supported: {SOURCE_PARAMS}'
        )

    key = _parse_key(mapping.get("key"), f'{where}.key')

    required = (
        _boolean(mapping["required"], f"{where}.required")
        if "required" in mapping
        else False
    )
    secret = (
        _boolean(mapping["secret"], f"{where}.secret")
        if "secret" in mapping
        else False
    )

    enum: Tuple[str, ...] = ()
    if "enum" in mapping:
        entries = mapping["enum"]
        if not isinstance(entries, list) or not entries:
            raise FixtureTypeError(f"{where}.enum must be a non-empty list")
        seen = set()
        for entry in entries:
            if not isinstance(entry, str):
                raise FixtureTypeError(
                    f"{where}.enum entries must be strings, got {entry!r}"
                )
            if entry in seen:
                raise FixtureTypeError(
                    f'{where}.enum lists "{entry}" more than once'
                )
            seen.add(entry)
        enum = tuple(entries)

    default = mapping.get("default", MISSING)
    if default is not MISSING:
        if isinstance(default, (dict, list)):
            raise FixtureTypeError(f"{where}.default must be a scalar")
        if secret:
            # A default is written in this repository, so a secret default would
            # be a committed secret -- the one thing this mechanism exists to
            # prevent.
            raise FixtureTypeError(
                f"{where}.default cannot be set on a secret option: defaults "
                "live in the repository and a secret must not"
            )
        if enum and (not isinstance(default, str) or default not in enum):
            raise FixtureTypeError(
                f"{where}.default {default!r} is not one of {list(enum)}"
            )

    required_if = (
        _parse_required_if(mapping["required_if"], f"{where}.required_if", siblings)
        if "required_if" in mapping
        else None
    )
    if required and required_if is not None:
        raise FixtureTypeError(
            f"{where}: use either \"required\" or \"required_if\", not both"
        )

    return FixtureOption(
        name=name,
        source=str(source),
        key=key,
        required=required,
        required_if=required_if,
        secret=secret,
        enum=enum,
        default=default,
    )


def _parse_type(name: str, raw: Any, source: str) -> FixtureType:
    where = f"{source}: {name}"
    mapping = _mapping(raw, where)
    _reject_unknown(mapping, _TYPE_FIELDS, where)

    if not _NAME_RE.match(name):
        raise FixtureTypeError(
            f'{where}: fixture type names must be lower_case_with_underscores'
        )
    if name in RESERVED_OPTION_NAMES or "." in name:
        raise FixtureTypeError(
            f'{where}: "{name}" collides with a binding key or contains a dot'
        )

    capabilities = (
        _parse_capabilities(mapping["capabilities"], f"{where}.capabilities")
        if "capabilities" in mapping
        else ()
    )

    # ``options`` is optional: a catalogue entry that only says "this type
    # exists, and these are its capabilities" is a complete entry. An explicit
    # ``options: {}`` means the same thing.
    options_raw = mapping.get("options")
    options_mapping: Mapping[str, Any] = (
        {} if options_raw is None else _mapping(options_raw, f"{where}.options")
    )

    options: List[FixtureOption] = []
    # ``required_if`` may point at an option declared later, so the whole set is
    # collected first and conditions are checked against every declared name.
    for option_name in options_mapping:
        if not isinstance(option_name, str):
            raise FixtureTypeError(
                f"{where}.options: option names must be strings, got {option_name!r}"
            )
        _name(option_name, f"{where}.options.{option_name}")

    for option_name, option_raw in options_mapping.items():
        options.append(
            _parse_option(
                option_name,
                option_raw,
                f"{where}.options.{option_name}",
                options_mapping,
            )
        )

    # A condition can only be evaluated when the option it names is resolved:
    # an optional option without a default may be absent, and report the
    # conditional option as missing instead of the dependency.
    by_name = {option.name: option for option in options}
    for option in options:
        if option.required_if is None:
            continue
        dependency = by_name[option.required_if.option]
        if not dependency.required and not dependency.has_default:
            raise FixtureTypeError(
                f"{where}.options.{option.name}.required_if: "
                f'"{dependency.name}" must be required or have a default, '
                "otherwise the condition can never be evaluated"
            )

    # Two options that read the same params entry would resolve to one value
    # under two names; that is a contract mistake, not an alias.
    keys_seen: Dict[str, str] = {}
    for option in options:
        previous = keys_seen.get(option.key)
        if previous is not None:
            raise FixtureTypeError(
                f'{where}: options "{previous}" and "{option.name}" use the same '
                f"params key {option.key!r}"
            )
        keys_seen[option.key] = option.name

    return FixtureType(
        name=name,
        description=str(mapping.get("description", "")),
        capabilities=capabilities,
        options=tuple(options),
    )


def parse_fixture_types(data: Any, source: str = "<memory>") -> FixtureTypes:
    """Parse an already-loaded contract document.

    :param data: the YAML document, as plain Python data.
    :param source: what to name in error messages.
    :return: the loaded contracts.
    :raises FixtureTypeError: the document is malformed or uses unsupported
        features.
    """
    mapping = _mapping(data, source)
    types: Dict[str, FixtureType] = {}
    for name, raw in mapping.items():
        if not isinstance(name, str):
            raise FixtureTypeError(
                f"{source}: fixture type names must be strings, got {name!r}"
            )
        types[name] = _parse_type(name, raw, source)
    return FixtureTypes(types=types, source=source)


def load_fixture_types(path: Optional[str] = None) -> FixtureTypes:
    """Load a contract document from disk.

    A duplicate mapping key is an error rather than a silent override; see
    :func:`sdk_test.yaml_support.strict_safe_loader`.

    :param path: file to read; defaults to :data:`DEFAULT_CONTRACT_PATH`.
    :return: the loaded contracts.
    :raises FixtureTypeError: the file cannot be read or parsed.
    """
    target = path or DEFAULT_CONTRACT_PATH
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - pyyaml is a core requirement
        raise FixtureTypeError("pyyaml is required to read fixture contracts") from exc

    try:
        with open(target, "r", encoding="utf-8") as handle:
            raw = yaml.load(
                handle,
                Loader=strict_safe_loader(yaml, target, FixtureTypeError),
            )
    except OSError as exc:
        raise FixtureTypeError(f"Cannot read fixture contracts {target}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise FixtureTypeError(f"{target}: malformed YAML: {exc}") from exc

    if raw is None:
        raise FixtureTypeError(f"{target} is empty; expected a mapping of fixture types")
    return parse_fixture_types(raw, source=target)


_DEFAULT: Optional[FixtureTypes] = None


def default_fixture_types() -> FixtureTypes:
    """The contracts that ship with the framework, loaded once."""
    global _DEFAULT
    if _DEFAULT is None:
        _DEFAULT = load_fixture_types()
    return _DEFAULT


def is_parameterized_type(name: str) -> bool:
    """Whether ``name`` is a known fixture type whose values come from params.

    Used by matching: a parameterized type is identified by its spec, because its
    values are looked up under ``fixtures.<spec>.…`` and a lab connection with a
    different spec would resolve them under the wrong keys.
    """
    return default_fixture_types().is_parameterized(name)


__all__ = [
    "CONTRACT_FILENAME",
    "DEFAULT_CONTRACT_PATH",
    "FixtureOption",
    "FixtureType",
    "FixtureTypeError",
    "FixtureTypes",
    "MISSING",
    "RequiredIf",
    "SOURCE_BINDING",
    "SOURCE_PARAMS",
    "default_fixture_types",
    "is_parameterized_type",
    "load_fixture_types",
    "parse_fixture_types",
]
