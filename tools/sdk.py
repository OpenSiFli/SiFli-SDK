#!/usr/bin/env python
#
# SPDX-FileCopyrightText: 2019-2024 Espressif Systems (Shanghai) CO LTD
# SPDX-FileCopyrightText: 2025-2026 SiFli Technologies(Nanjing) Co., Ltd
#
# SPDX-License-Identifier: Apache-2.0
#
# 'sdk.py' is a top-level config/build command line tool for SiFli-SDK

from __future__ import annotations

import codecs
import importlib
import importlib.util
import locale
import logging
import os
import os.path
import shlex
import subprocess
import sys
import tomllib
import traceback
from dataclasses import dataclass
from dataclasses import field
from pkgutil import iter_modules
from typing import Any
from typing import List
from typing import Optional
from typing import Tuple

# pyc files remain in the filesystem when switching between branches which might raise errors for incompatible
# sdk.py extensions. Therefore, pyc file generation is turned off:
sys.dont_write_bytecode = True

import python_version_checker  # noqa: E402

try:
    import click  # noqa: E402

    from sdk_py_actions.cli import CommandRegistry  # noqa: E402
    from sdk_py_actions.cli import CommandRunner  # noqa: E402
    from sdk_py_actions.cli import SDK_CONTEXT_KEY  # noqa: E402
    from sdk_py_actions.cli import SdkConfig  # noqa: E402
    from sdk_py_actions.cli import SdkContext  # noqa: E402
    from sdk_py_actions.errors import EnvironmentError as SdkEnvironmentError  # noqa: E402
    from sdk_py_actions.errors import FatalError  # noqa: E402
    from sdk_py_actions.errors import UsageError  # noqa: E402
    from sdk_py_actions.tools import (  # noqa: E402
        PROG,
        SHELL_COMPLETE_RUN,
        SHELL_COMPLETE_VAR,
        debug_print_sdk_version,
        print_warning,
        sdk_version,
    )
except ImportError as e:
    print(
        (
            f"{e}\n"
            f'This usually means that "sdk.py" was not '
            f'spawned within an SiFli-SDK shell environment or the python virtual '
            f'environment used by "sdk.py" is corrupted.\n'
            f'Please use sdk.py only in an SiFli-SDK shell environment. If problem persists, '
            f'please try to install SiFli-SDK tools again as described in the Get Started guide.'
        ),
        file=sys.stderr,
    )
    if e.name is None:
        raise

    sys.exit(1)


EXTENSION_API_VERSION = 2


@dataclass
class ExtensionSource:
    module_name: str
    source_type: str
    source_path: str


@dataclass
class ExtensionLoadReport:
    loaded: List[str] = field(default_factory=list)
    skipped: List[str] = field(default_factory=list)
    failed: List[str] = field(default_factory=list)

    def print_summary(self) -> None:
        if SHELL_COMPLETE_RUN:
            return

        for item in self.skipped:
            print_warning(f"WARNING: skipped extension: {item}")
        for item in self.failed:
            print_warning(f"WARNING: failed extension: {item}")


# Use this Python interpreter for any subprocesses we launch
PYTHON = sys.executable


def check_environment() -> List[str]:
    """
    Verify the environment contains the top-level tools we need to operate.
    """
    checks_output: List[str] = []
    if os.getenv("LEGACY_ENV"):
        return checks_output

    detected_sifli_sdk_path = os.path.realpath(os.path.join(os.path.dirname(__file__), ".."))
    if "SIFLI_SDK_PATH" in os.environ:
        set_sifli_sdk_path = os.path.realpath(os.environ["SIFLI_SDK_PATH"])
        if set_sifli_sdk_path != detected_sifli_sdk_path:
            print_warning(
                "WARNING: SIFLI_SDK_PATH environment variable is set to %s but %s path indicates "
                "SIFLI SDK directory %s. Using the environment variable directory, but results may be unexpected..."
                % (set_sifli_sdk_path, PROG, detected_sifli_sdk_path)
            )
    else:
        print_warning("Setting SIFLI_SDK_PATH environment variable: %s" % detected_sifli_sdk_path)
        os.environ["SIFLI_SDK_PATH"] = detected_sifli_sdk_path

    try:
        python_version_checker.check()
    except RuntimeError as e:
        raise SdkEnvironmentError(str(e)) from e

    checks_output.append("Checking Python dependencies...")
    try:
        out = subprocess.check_output(
            [
                PYTHON,
                os.path.join(os.environ["SIFLI_SDK_PATH"], "tools", "sifli_sdk_tools.py"),
                "check-python-dependencies",
            ],
            env=os.environ,
        )
        checks_output.append(out.decode("utf-8", "ignore").strip())
    except subprocess.CalledProcessError as e:
        print_warning(e.output.decode("utf-8", "ignore"), stream=sys.stderr)
        debug_print_sdk_version()
        raise SdkEnvironmentError("Python dependency check failed") from e

    checks_output.append("Checking used Python interpreter...")
    try:
        python_venv_path = os.environ["SIFLI_SDK_PYTHON_ENV_PATH"]
        if python_venv_path and not sys.executable.startswith(python_venv_path):
            print_warning(
                f'WARNING: Python interpreter "{sys.executable}" used to start sdk.py '
                f'is not from installed venv "{python_venv_path}"'
            )
    except KeyError:
        print_warning("WARNING: The SIFLI_SDK_PYTHON_ENV_PATH is missing in environmental variables!")

    return checks_output


def _setup_logger(verbose: bool) -> logging.Logger:
    logger = logging.getLogger("sdk")
    if not logger.handlers:
        handler = logging.StreamHandler(sys.stderr)
        handler.setFormatter(logging.Formatter("[%(levelname)s] %(message)s"))
        logger.addHandler(handler)
    logger.setLevel(logging.DEBUG if verbose else logging.INFO)
    return logger


def _normalize_project_and_build_dir(project_dir: str, build_dir: Optional[str]) -> Tuple[str, str]:
    real_project_dir = os.path.realpath(project_dir)
    if build_dir is None:
        build_dir = _infer_default_build_dir(real_project_dir)

    real_build_dir = os.path.realpath(build_dir)
    if real_project_dir == real_build_dir:
        raise UsageError(
            "Setting the build directory to the project directory is not supported. "
            "Drop --build-dir to use the default build subdirectory."
        )

    return real_project_dir, real_build_dir


def _discover_project_dir(start_dir: str) -> str:
    current = os.path.realpath(start_dir)
    while True:
        if os.path.exists(os.path.join(current, ".project.toml")):
            return current
        parent = os.path.dirname(current)
        if parent == current:
            return os.path.realpath(start_dir)
        current = parent


def _infer_default_build_dir(project_dir: str) -> str:
    config_path = os.path.join(project_dir, ".project.toml")
    default_build_dir = os.path.join(project_dir, "build")
    if not os.path.exists(config_path):
        return default_build_dir

    try:
        with open(config_path, "rb") as f:
            config = tomllib.load(f)
    except Exception:
        return default_build_dir

    board = config.get("board")
    if isinstance(board, str):
        board = board.strip()
        if board:
            board_dir = board if board.endswith("_hcpu") else f"{board}_hcpu"
            return os.path.join(project_dir, f"build_{board_dir}")

    return default_build_dir


def _init_environment(project_dir: str) -> None:
    os.environ["PYTHON"] = PYTHON
    os.environ["BSP_ROOT"] = project_dir

    if not os.getenv("ENV_ROOT"):
        env_root = os.getenv("SIFLI_SDK_PYTHON_ENV_PATH")
        if env_root:
            os.environ["ENV_ROOT"] = env_root


def _version_callback(ctx: click.Context, param: click.Option, value: bool) -> None:
    if not value or ctx.resilient_parsing:
        return

    version = sdk_version()
    if not version:
        raise SdkEnvironmentError("SiFli-SDK version cannot be determined")

    print(f"SiFli-SDK {version}")
    ctx.exit(0)


def _preparse_root_args(argv: List[str]) -> Tuple[str, bool]:
    project_dir = _discover_project_dir(os.getcwd())
    strict_extensions = False

    index = 0
    while index < len(argv):
        arg = argv[index]

        if arg == "--strict-extensions":
            strict_extensions = True
        elif arg in ("-C", "--project-dir"):
            if index + 1 < len(argv):
                project_dir = argv[index + 1]
                index += 1
        elif arg.startswith("--project-dir="):
            project_dir = arg.split("=", 1)[1]

        index += 1

    return os.path.realpath(project_dir), strict_extensions


def _extension_dirs() -> Tuple[str, List[str]]:
    sdk_py_extensions_path = os.path.join(os.environ["SIFLI_SDK_PATH"], "tools", "sdk_py_actions")

    external_dirs: List[str] = []
    extra_paths = os.environ.get("SIFLI_SDK_EXTRA_ACTIONS_PATH")
    if extra_paths:
        separator = ";" if ";" in extra_paths else os.pathsep
        for raw_path in extra_paths.split(separator):
            path = os.path.realpath(raw_path.strip())
            if path and path not in external_dirs:
                external_dirs.append(path)

    return os.path.realpath(sdk_py_extensions_path), external_dirs


def _discover_modules(path: str) -> List[str]:
    return [name for _finder, name, _ispkg in sorted(iter_modules([path])) if name.endswith("_ext")]


def _import_project_extension(project_dir: str) -> Optional[Any]:
    sdk_ext_path = os.path.join(project_dir, "sdk_ext.py")
    if not os.path.exists(sdk_ext_path):
        return None

    module_name = "sdk_ext"
    spec = importlib.util.spec_from_file_location(module_name, sdk_ext_path)
    if spec is None or spec.loader is None:
        raise SdkEnvironmentError(f"Cannot load project extension: {sdk_ext_path}")

    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _validate_extension_module(module: Any, source: ExtensionSource) -> Tuple[str, Any]:
    extension_obj = getattr(module, "extension", None)
    if extension_obj is not None:
        extension_id = getattr(extension_obj, "id", source.module_name)
        api_version = getattr(extension_obj, "api_version", None)
        register_func = getattr(extension_obj, "register", None)
    else:
        extension_id = getattr(module, "EXTENSION_ID", source.module_name)
        api_version = getattr(module, "EXTENSION_API_VERSION", None)
        register_func = getattr(module, "register", None)

    if api_version != EXTENSION_API_VERSION:
        raise SdkEnvironmentError(
            f'Extension "{extension_id}" API version mismatch: '
            f"expected {EXTENSION_API_VERSION}, got {api_version}"
        )

    if not callable(register_func):
        raise SdkEnvironmentError(f'Extension "{extension_id}" does not provide register(registry)')

    return extension_id, register_func


def _handle_extension_error(
    source: ExtensionSource,
    strict_extensions: bool,
    report: ExtensionLoadReport,
    error: Exception,
) -> None:
    message = f"{source.module_name} ({source.source_type}: {source.source_path}): {error}"

    if source.source_type == "builtin":
        report.failed.append(message)
        raise SdkEnvironmentError(f"Failed to load builtin extension {source.module_name}: {error}") from error

    if strict_extensions:
        report.failed.append(message)
        raise SdkEnvironmentError(f"Failed to load extension {source.module_name}: {error}") from error

    report.skipped.append(message)


def _load_and_register_extensions(
    registry: CommandRegistry,
    project_dir: str,
    strict_extensions: bool,
) -> ExtensionLoadReport:
    report = ExtensionLoadReport()
    builtin_dir, external_dirs = _extension_dirs()

    extension_sources: List[ExtensionSource] = []

    if not os.path.exists(builtin_dir):
        raise SdkEnvironmentError(f"Builtin extension directory does not exist: {builtin_dir}")

    sys.path.append(builtin_dir)
    for module_name in _discover_modules(builtin_dir):
        extension_sources.append(
            ExtensionSource(module_name=module_name, source_type="builtin", source_path=builtin_dir)
        )

    for directory in external_dirs:
        if not os.path.exists(directory):
            message = f"{directory} (external directory does not exist)"
            if strict_extensions:
                report.failed.append(message)
                raise SdkEnvironmentError(f"Extension directory does not exist: {directory}")
            report.skipped.append(message)
            continue

        sys.path.append(directory)
        for module_name in _discover_modules(directory):
            extension_sources.append(
                ExtensionSource(module_name=module_name, source_type="external", source_path=directory)
            )

    try:
        project_ext = _import_project_extension(project_dir)
    except Exception as e:
        source = ExtensionSource(module_name="sdk_ext", source_type="project", source_path=project_dir)
        _handle_extension_error(source, strict_extensions, report, e)
        project_ext = None

    for source in extension_sources:
        try:
            module = importlib.import_module(source.module_name)
            extension_id, register_func = _validate_extension_module(module, source)
            register_func(registry)
            report.loaded.append(f"{extension_id} ({source.source_type})")
        except Exception as e:
            _handle_extension_error(source, strict_extensions, report, e)

    if project_ext is not None:
        source = ExtensionSource(module_name="sdk_ext", source_type="project", source_path=project_dir)
        try:
            extension_id, register_func = _validate_extension_module(project_ext, source)
            register_func(registry)
            report.loaded.append(f"{extension_id} ({source.source_type})")
        except Exception as e:
            _handle_extension_error(source, strict_extensions, report, e)

    return report


def _build_root_callback(verbose_output: Optional[List[str]]) -> Any:
    @click.pass_context
    def root_callback(
        ctx: click.Context,
        project_dir: str,
        build_dir: Optional[str],
        verbose: bool,
        no_hints: bool,
        strict_extensions: bool,
    ) -> None:
        if verbose and verbose_output:
            for line in verbose_output:
                if line:
                    print(line)

        project_dir, build_dir = _normalize_project_and_build_dir(project_dir, build_dir)
        _init_environment(project_dir)

        logger = _setup_logger(verbose)
        sdk_ctx = SdkContext(
            project_dir=project_dir,
            build_dir=build_dir,
            env=dict(os.environ),
            logger=logger,
            runner=CommandRunner(logger),
            config=SdkConfig(
                strict_extensions=strict_extensions,
                verbose=verbose,
                no_hints=no_hints,
            ),
        )

        ctx.ensure_object(dict)
        ctx.obj[SDK_CONTEXT_KEY] = sdk_ctx

    return root_callback


def _build_registry(argv: List[str], verbose_output: Optional[List[str]]) -> Tuple[CommandRegistry, ExtensionLoadReport, List[str]]:
    expanded_argv = expand_file_arguments(argv)
    pre_project_dir, strict_extensions = _preparse_root_args(expanded_argv)

    registry = CommandRegistry(api_version=EXTENSION_API_VERSION)
    registry.register_root_options(
        [
            {
                "names": ["--version"],
                "help": "Show SiFli-SDK version and exit.",
                "is_flag": True,
                "expose_value": False,
                "is_eager": True,
                "callback": _version_callback,
            },
            {
                "names": ["-C", "--project-dir"],
                "help": "Project directory.",
                "type": click.Path(),
                "default": pre_project_dir,
            },
            {
                "names": ["-B", "--build-dir"],
                "help": "Build directory.",
                "type": click.Path(),
                "default": None,
            },
            {
                "names": ["-v", "--verbose"],
                "help": "Verbose output.",
                "is_flag": True,
                "default": False,
            },
            {
                "names": ["--no-hints"],
                "help": "Disable hints on how to resolve errors and logging.",
                "is_flag": True,
                "default": False,
            },
            {
                "names": ["--strict-extensions"],
                "help": "Fail startup if any external/project extension fails to load.",
                "is_flag": True,
                "default": False,
            },
        ]
    )

    report = _load_and_register_extensions(
        registry=registry,
        project_dir=pre_project_dir,
        strict_extensions=strict_extensions,
    )

    return registry, report, expanded_argv


def init_cli(argv: List[str], verbose_output: Optional[List[str]] = None) -> Tuple[click.Command, ExtensionLoadReport, List[str]]:
    registry, report, expanded_argv = _build_registry(argv, verbose_output)

    cli_help = "SiFli-SDK CLI build management tool."
    cli = registry.build_click(
        root_callback=_build_root_callback(verbose_output),
        root_help=cli_help,
        context_settings={"max_content_width": 140},
    )

    return cli, report, expanded_argv


def _map_exception_to_exit_code(exc: Exception) -> int:
    if isinstance(exc, click.exceptions.Exit):
        return int(getattr(exc, "exit_code", 0))

    if isinstance(exc, click.exceptions.Abort):
        return 1

    if isinstance(exc, FatalError):
        return getattr(exc, "exit_code", 10)

    if isinstance(exc, click.UsageError):
        return 2

    if isinstance(exc, click.ClickException):
        return 2

    if isinstance(exc, subprocess.CalledProcessError):
        return 4

    return 10


def _print_exception(exc: Exception) -> None:
    if isinstance(exc, click.exceptions.Exit):
        return

    if isinstance(exc, click.exceptions.Abort):
        print("Aborted!", file=sys.stderr)
        return

    if isinstance(exc, click.ClickException):
        exc.show(file=sys.stderr)
        return

    print(str(exc), file=sys.stderr)


def main(argv: Optional[List[Any]] = None) -> None:
    checks_output = None if SHELL_COMPLETE_RUN else check_environment()

    try:
        os.getcwd()
    except FileNotFoundError as e:
        raise SdkEnvironmentError(f"Working directory cannot be established: {e}") from e

    args = [str(a) for a in (argv or sys.argv[1:])]

    try:
        cli, report, expanded_argv = init_cli(args, verbose_output=checks_output)
        report.print_summary()
        cli(expanded_argv, prog_name=PROG, complete_var=SHELL_COMPLETE_VAR, standalone_mode=False)
    except Exception as exc:
        _print_exception(exc)
        if os.getenv("SDK_PY_DEBUG_EXCEPTIONS") == "1":
            traceback.print_exc()
        raise SystemExit(_map_exception_to_exit_code(exc))


def expand_file_arguments(argv: List[Any]) -> List[str]:
    """
    Any argument starting with "@" gets replaced with all values read from a text file.
    Text file arguments can be split by newline or by space.
    Values are added as if they were specified in this order on the command line.
    """
    visited = set()
    expanded = False

    def expand_args(args: List[str], parent_path: str, file_stack: List[str]) -> List[str]:
        expanded_args = []
        for arg in args:
            if not arg.startswith("@"):
                expanded_args.append(arg)
            else:
                nonlocal expanded, visited
                expanded = True

                file_name = arg[1:]
                rel_path = os.path.normpath(os.path.join(parent_path, file_name))

                if rel_path in visited:
                    file_stack_str = " -> ".join(["@" + f for f in file_stack + [file_name]])
                    raise UsageError(f"Circular dependency in file argument expansion: {file_stack_str}")
                visited.add(rel_path)

                try:
                    with open(rel_path, "r", encoding="utf-8") as f:
                        for line in f:
                            expanded_args.extend(
                                expand_args(shlex.split(line), os.path.dirname(rel_path), file_stack + [file_name])
                            )
                except IOError as exc:
                    file_stack_str = " -> ".join(["@" + f for f in file_stack + [file_name]])
                    raise UsageError(
                        f"File '{rel_path}' (expansion of {file_stack_str}) could not be opened. "
                        "Please ensure the file exists and is readable."
                    ) from exc
        return expanded_args

    expanded_argv = expand_args([str(item) for item in argv], os.getcwd(), [])

    if expanded:
        print(f'Running: sdk.py {" ".join(expanded_argv)}')

    return expanded_argv


def _valid_unicode_config() -> bool:
    if sys.version_info[0] == 2:
        return True

    try:
        return codecs.lookup(locale.getpreferredencoding()).name != "ascii"
    except Exception:
        return False


def _find_usable_locale() -> str:
    try:
        locales = (
            subprocess.Popen(["locale", "-a"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            .communicate()[0]
            .decode("ascii", "replace")
        )
    except OSError:
        locales = ""

    usable_locales: List[str] = []
    for line in locales.splitlines():
        loc = line.strip()
        locale_name = loc.lower().replace("-", "")

        if locale_name == "c.utf8":
            return loc

        if locale_name.endswith(".utf8"):
            if loc.startswith("en_"):
                usable_locales.insert(0, loc)
            else:
                usable_locales.append(loc)

    if not usable_locales:
        raise SdkEnvironmentError(
            "Support for Unicode filenames is required, but no suitable UTF-8 locale was found on your system."
        )

    return usable_locales[0]


if __name__ == "__main__":
    try:
        if "MSYSTEM" in os.environ:
            print_warning(
                "MSys/Mingw is no longer supported. Please follow the getting started guide in the documentation "
                "to set up a suitable environment, or continue at your own risk."
            )
        elif os.name == "posix" and not _valid_unicode_config():
            best_locale = _find_usable_locale()
            print_warning(
                "Your environment is not configured to handle unicode filenames outside of ASCII range. "
                f"Environment variable LC_ALL is temporarily set to {best_locale}."
            )

            os.environ["LC_ALL"] = best_locale
            ret = subprocess.call([sys.executable] + sys.argv, env=os.environ)
            if ret:
                raise SystemExit(ret)
        else:
            main()
    except FatalError as e:
        print(e, file=sys.stderr)
        sys.exit(getattr(e, "exit_code", 10))
