# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import re
from dataclasses import dataclass
from dataclasses import field
from typing import Any
from typing import Callable
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Protocol
from typing import Sequence

import click

from sdk_py_actions.errors import UsageError

SDK_CONTEXT_KEY = "sdk_context"

_PATH_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")


class Extension(Protocol):
    id: str
    version: str
    api_version: int
    min_sdk_version: Optional[str]

    def register(self, registry: "CommandRegistry") -> None:
        ...


@dataclass(frozen=True)
class OptionSpec:
    names: Sequence[str]
    help: str = ""
    default: Any = None
    type: Any = None
    required: bool = False
    is_flag: bool = False
    multiple: bool = False
    envvar: Optional[str] = None
    metavar: Optional[str] = None
    expose_value: bool = True
    callback: Optional[Callable[..., Any]] = None
    hidden: bool = False
    is_eager: bool = False
    show_default: Any = None

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "OptionSpec":
        allowed = {
            "names", "help", "default", "type", "required", "is_flag", "multiple",
            "envvar", "metavar", "expose_value", "callback", "hidden", "is_eager", "show_default",
        }
        unknown = set(data.keys()) - allowed
        if unknown:
            raise UsageError(f'Unknown option fields: {", ".join(sorted(unknown))}')

        names = data.get("names")
        if not isinstance(names, (list, tuple)) or not names:
            raise UsageError('Option "names" must be a non-empty list')

        return cls(
            names=tuple(str(item) for item in names),
            help=str(data.get("help", "")),
            default=data.get("default", None),
            type=data.get("type", None),
            required=bool(data.get("required", False)),
            is_flag=bool(data.get("is_flag", False)),
            multiple=bool(data.get("multiple", False)),
            envvar=data.get("envvar", None),
            metavar=data.get("metavar", None),
            expose_value=bool(data.get("expose_value", True)),
            callback=data.get("callback", None),
            hidden=bool(data.get("hidden", False)),
            is_eager=bool(data.get("is_eager", False)),
            show_default=data.get("show_default", None),
        )

    def to_click_param(self) -> click.Option:
        kwargs: Dict[str, Any] = {
            "help": self.help,
            "default": self.default,
            "required": self.required,
            "is_flag": self.is_flag,
            "multiple": self.multiple,
            "expose_value": self.expose_value,
            "hidden": self.hidden,
            "is_eager": self.is_eager,
        }

        if self.type is not None:
            kwargs["type"] = self.type
        if self.envvar is not None:
            kwargs["envvar"] = self.envvar
        if self.metavar is not None:
            kwargs["metavar"] = self.metavar
        if self.callback is not None:
            kwargs["callback"] = self.callback
        if self.show_default is not None:
            kwargs["show_default"] = self.show_default

        return click.Option(param_decls=tuple(self.names), **kwargs)


@dataclass(frozen=True)
class ArgumentSpec:
    names: Sequence[str]
    required: bool = True
    nargs: int = 1
    default: Any = None
    type: Any = None

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ArgumentSpec":
        allowed = {"names", "required", "nargs", "default", "type"}
        unknown = set(data.keys()) - allowed
        if unknown:
            raise UsageError(f'Unknown argument fields: {", ".join(sorted(unknown))}')

        names = data.get("names")
        if not isinstance(names, (list, tuple)) or len(names) != 1:
            raise UsageError('Argument "names" must contain exactly one item')

        return cls(
            names=tuple(str(item) for item in names),
            required=bool(data.get("required", True)),
            nargs=int(data.get("nargs", 1)),
            default=data.get("default", None),
            type=data.get("type", None),
        )

    def to_click_param(self) -> click.Argument:
        kwargs: Dict[str, Any] = {
            "required": self.required,
            "nargs": self.nargs,
            "default": self.default,
        }
        if self.type is not None:
            kwargs["type"] = self.type
        return click.Argument(param_decls=tuple(self.names), **kwargs)


@dataclass
class GroupSpec:
    path: str
    help: str = ""
    options: List[OptionSpec] = field(default_factory=list)
    aliases: List[str] = field(default_factory=list)
    before_invoke: Optional[Callable[..., Any]] = None
    implicit: bool = False


@dataclass
class CommandSpec:
    path: str
    callback: Callable[..., Any]
    help: str = ""
    options: List[OptionSpec] = field(default_factory=list)
    arguments: List[ArgumentSpec] = field(default_factory=list)
    aliases: List[str] = field(default_factory=list)
    before_invoke: Optional[Callable[..., Any]] = None
    hidden: bool = False


@dataclass
class _TreeNode:
    path: str
    name: str
    group: GroupSpec
    children: Dict[str, "_TreeNode"] = field(default_factory=dict)
    commands: Dict[str, CommandSpec] = field(default_factory=dict)


class AliasGroup(click.Group):
    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._aliases: Dict[str, str] = {}

    def add_alias(self, alias: str, target: str) -> None:
        self._aliases[alias] = target

    def get_command(self, ctx: click.Context, cmd_name: str) -> Optional[click.Command]:
        command = super().get_command(ctx, cmd_name)
        if command is not None:
            return command

        target = self._aliases.get(cmd_name)
        if target is None:
            return None

        return super().get_command(ctx, target)


class CommandRegistry:
    def __init__(self, api_version: int = 2) -> None:
        self.api_version = api_version
        self._groups: Dict[str, GroupSpec] = {
            "": GroupSpec(path="", help="", options=[], aliases=[]),
        }
        self._commands: Dict[str, CommandSpec] = {}
        self._root_options: List[OptionSpec] = []

    def register_root_options(self, options: Optional[Iterable[Dict[str, Any]]]) -> None:
        if not options:
            return

        self._root_options.extend(self._parse_options(options))

    def group(
        self,
        path: str,
        help: str = "",
        options: Optional[Iterable[Dict[str, Any]]] = None,
        aliases: Optional[Iterable[str]] = None,
        before_invoke: Optional[Callable[..., Any]] = None,
    ) -> None:
        normalized = self._normalize_path(path)
        if not normalized:
            raise UsageError("Group path cannot be empty")

        self._ensure_parent_groups(normalized)

        if normalized in self._commands:
            raise UsageError(f'Path conflict: "{normalized}" is already a command')

        alias_list = self._parse_aliases(aliases)
        option_list = self._parse_options(options)
        current = self._groups.get(normalized)
        if current and not current.implicit:
            raise UsageError(f'Duplicate group path: "{normalized}"')

        self._groups[normalized] = GroupSpec(
            path=normalized,
            help=help,
            options=option_list,
            aliases=alias_list,
            before_invoke=before_invoke,
            implicit=False,
        )

    def command(
        self,
        path: str,
        callback: Callable[..., Any],
        help: str = "",
        options: Optional[Iterable[Dict[str, Any]]] = None,
        arguments: Optional[Iterable[Dict[str, Any]]] = None,
        aliases: Optional[Iterable[str]] = None,
        before_invoke: Optional[Callable[..., Any]] = None,
        hidden: bool = False,
    ) -> None:
        normalized = self._normalize_path(path)
        if not normalized:
            raise UsageError("Command path cannot be empty")
        if not callable(callback):
            raise UsageError(f'Callback for command "{normalized}" must be callable')

        if normalized in self._groups:
            raise UsageError(f'Path conflict: "{normalized}" is already a group')
        if normalized in self._commands:
            raise UsageError(f'Duplicate command path: "{normalized}"')

        self._ensure_parent_groups(normalized)

        self._commands[normalized] = CommandSpec(
            path=normalized,
            callback=callback,
            help=help,
            options=self._parse_options(options),
            arguments=self._parse_arguments(arguments),
            aliases=self._parse_aliases(aliases),
            before_invoke=before_invoke,
            hidden=hidden,
        )

    def build_click(
        self,
        root_callback: Callable[..., Any],
        root_help: str,
        context_settings: Optional[Dict[str, Any]] = None,
    ) -> click.Command:
        tree = self._build_tree()
        return self._build_group_click(
            node=tree,
            root_callback=root_callback,
            root_help=root_help,
            context_settings=context_settings or {"max_content_width": 140},
        )

    def _normalize_path(self, path: str) -> str:
        value = (path or "").replace("\\", "/").strip("/")
        if not value:
            return ""

        for part in value.split("/"):
            if not _PATH_RE.match(part):
                raise UsageError(f'Invalid path segment "{part}" in "{path}"')
        return value

    def _parse_aliases(self, aliases: Optional[Iterable[str]]) -> List[str]:
        if not aliases:
            return []

        parsed = []
        for alias in aliases:
            if not isinstance(alias, str):
                raise UsageError("Alias must be a string")
            if not _PATH_RE.match(alias):
                raise UsageError(f'Invalid alias: "{alias}"')
            parsed.append(alias)

        if len(set(parsed)) != len(parsed):
            raise UsageError("Duplicate aliases are not allowed")

        return parsed

    def _parse_options(self, options: Optional[Iterable[Dict[str, Any]]]) -> List[OptionSpec]:
        if not options:
            return []
        return [OptionSpec.from_dict(option) for option in options]

    def _parse_arguments(self, arguments: Optional[Iterable[Dict[str, Any]]]) -> List[ArgumentSpec]:
        if not arguments:
            return []
        return [ArgumentSpec.from_dict(argument) for argument in arguments]

    def _ensure_parent_groups(self, path: str) -> None:
        parts = path.split("/")
        for index in range(1, len(parts)):
            current = "/".join(parts[:index])
            if current in self._commands:
                raise UsageError(f'Path conflict: "{current}" is already a command')
            if current not in self._groups:
                self._groups[current] = GroupSpec(path=current, implicit=True)

    def _build_tree(self) -> _TreeNode:
        nodes: Dict[str, _TreeNode] = {
            "": _TreeNode(path="", name="", group=self._groups[""])
        }

        for path, group in sorted(self._groups.items(), key=lambda item: item[0].count("/")):
            if path == "":
                continue

            name = path.split("/")[-1]
            node = _TreeNode(path=path, name=name, group=group)
            nodes[path] = node

            parent_path = "/".join(path.split("/")[:-1])
            parent = nodes.get(parent_path)
            if parent is None:
                raise UsageError(f'Missing parent group for path "{path}"')
            if name in parent.commands:
                raise UsageError(f'Path conflict: "{path}" conflicts with command name')
            parent.children[name] = node

        for path, command in self._commands.items():
            parent_path, _, command_name = path.rpartition("/")
            parent = nodes.get(parent_path)
            if parent is None:
                raise UsageError(f'Missing parent group for command "{path}"')
            if command_name in parent.children:
                raise UsageError(f'Path conflict: "{path}" conflicts with group path')
            parent.commands[command_name] = command

        return nodes[""]

    def _build_group_click(
        self,
        node: _TreeNode,
        root_callback: Callable[..., Any],
        root_help: str,
        context_settings: Dict[str, Any],
    ) -> AliasGroup:
        is_root = node.path == ""
        group_spec = node.group

        group_callback = root_callback if is_root else self._make_group_callback(group_spec)
        options = list(self._root_options if is_root else []) + list(group_spec.options)
        params = [option.to_click_param() for option in options]

        group = AliasGroup(
            name=None if is_root else node.name,
            callback=group_callback,
            params=params,
            help=root_help if is_root else group_spec.help,
            context_settings=context_settings,
            no_args_is_help=True,
            invoke_without_command=False,
        )

        for child_name in sorted(node.children):
            child_node = node.children[child_name]
            child_group = self._build_group_click(
                node=child_node,
                root_callback=root_callback,
                root_help=root_help,
                context_settings=context_settings,
            )
            group.add_command(child_group, name=child_name)

        for command_name in sorted(node.commands):
            command_spec = node.commands[command_name]
            click_command = self._build_command_click(command_name, command_spec)
            group.add_command(click_command, name=command_name)

        used_names = set(group.commands.keys())
        for child_name, child_node in node.children.items():
            for alias in child_node.group.aliases:
                if alias in used_names or alias in group._aliases:
                    raise UsageError(
                        f'Alias conflict in group "{node.path or "<root>"}": "{alias}"'
                    )
                group.add_alias(alias, child_name)

        for command_name, command_spec in node.commands.items():
            for alias in command_spec.aliases:
                if alias in used_names or alias in group._aliases:
                    raise UsageError(
                        f'Alias conflict in group "{node.path or "<root>"}": "{alias}"'
                    )
                group.add_alias(alias, command_name)

        return group

    def _make_group_callback(self, group_spec: GroupSpec) -> Callable[..., Any]:
        @click.pass_context
        def group_callback(click_ctx: click.Context, **kwargs: Any) -> None:
            sdk_ctx = self._maybe_get_sdk_context(click_ctx)
            if sdk_ctx is None:
                return

            sdk_ctx.config.group_options[group_spec.path] = kwargs
            if group_spec.before_invoke is not None:
                group_spec.before_invoke(sdk_ctx, kwargs)

        return group_callback

    def _build_command_click(self, command_name: str, command: CommandSpec) -> click.Command:
        params: List[click.Parameter] = [option.to_click_param() for option in command.options]
        params.extend(argument.to_click_param() for argument in command.arguments)

        @click.pass_context
        def command_callback(click_ctx: click.Context, **kwargs: Any) -> Any:
            sdk_ctx = self._require_sdk_context(click_ctx)
            if command.before_invoke is not None:
                command.before_invoke(sdk_ctx, kwargs)
            return command.callback(sdk_ctx, **kwargs)

        return click.Command(
            name=command_name,
            callback=command_callback,
            params=params,
            help=command.help,
            hidden=command.hidden,
        )

    def _maybe_get_sdk_context(self, click_ctx: click.Context) -> Any:
        root = click_ctx.find_root()
        if not isinstance(root.obj, dict):
            return None
        return root.obj.get(SDK_CONTEXT_KEY)

    def _require_sdk_context(self, click_ctx: click.Context) -> Any:
        sdk_ctx = self._maybe_get_sdk_context(click_ctx)
        if sdk_ctx is None:
            raise UsageError("SDK context is not initialized")
        return sdk_ctx
