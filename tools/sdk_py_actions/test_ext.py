# -*- coding:utf-8 -*-
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""``sdk.py test`` -- the automated-test framework's CLI entry point.

This file is deliberately thin. All the behaviour lives in
:mod:`sdk_test.cli`, which knows nothing about click or the SDK context; this
module parses options, supplies the one thing the library cannot have (a way to
run ``scons``), and maps :class:`~sdk_test.errors.SdkTestError` onto the CLI's
error types so the exit codes documented in ``sdk.py`` still hold.

Exit codes: ``0`` everything passed, ``1`` at least one test failed, ``2`` the
request was unusable, and the usual ``sdk.py`` codes (3, 4, 10) for environment,
command and fatal errors.
"""

from __future__ import annotations

import functools
import os
import sys
from typing import Any
from typing import Callable
from typing import List
from typing import Optional

import click

# `sdk.py` puts tools/sdk_py_actions on sys.path, not tools/, so the sdk_test
# package (its sibling) has to be made importable explicitly.
_TOOLS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, _TOOLS_DIR)

from sdk_py_actions.cli.context import SdkContext  # noqa: E402
from sdk_py_actions.cli.registry import CommandRegistry  # noqa: E402
from sdk_py_actions.errors import FatalError  # noqa: E402
from sdk_py_actions.errors import UsageError  # noqa: E402

from sdk_test import cli as sdk_test_cli  # noqa: E402
from sdk_test.backends import backend_names  # noqa: E402
from sdk_test.errors import BuildError  # noqa: E402
from sdk_test.errors import SdkTestError  # noqa: E402
from sdk_test.runner import RunOptions  # noqa: E402

EXTENSION_ID = "test"
EXTENSION_VERSION = "0.1.0"
EXTENSION_API_VERSION = 2
MIN_SDK_VERSION = None

#: Default test-suite root: the SDK's example tree, which is where testcase.yaml
#: files live. Overridable with -T.
_DEFAULT_SUITE_SUBDIR = "example"


def _default_suite_root() -> str:
    sdk_path = os.environ.get("SIFLI_SDK_PATH")
    if sdk_path:
        candidate = os.path.join(sdk_path, _DEFAULT_SUITE_SUBDIR)
        if os.path.isdir(candidate):
            return candidate
    return os.getcwd()


#: What to tell the user when a command finishes with a non-zero code but
#: nothing has gone wrong in the "error" sense. `run` exiting 1 means tests
#: failed, which is a result, not a fault -- so the wording has to say so.
_EXIT_MESSAGES = {
    sdk_test_cli.EXIT_FAILURES: "one or more tests failed",
    sdk_test_cli.EXIT_USAGE: "no test cases matched the selection",
}


def _guard(callback: Callable[..., int]) -> Callable[..., Any]:
    """Turn library errors into CLI errors, and exit codes into a process status.

    ``SdkTestError`` deliberately does not inherit from the CLI's error family --
    the library has to be importable and testable on its own -- so this is the
    single place the two vocabularies meet.

    The non-zero code is carried out as a :class:`FatalError` with an explicit
    ``exit_code``, NOT as ``click.exceptions.Exit``. Raising ``Exit`` from a
    callback looks like the natural way to do this, and it worked on older click
    -- but click 8.3 swallows it under ``standalone_mode=False`` and the process
    then exits 0. Silently reporting success for a failed test run is the one
    failure this framework must never have: CI would go green on a broken build.
    ``FatalError`` is an ordinary exception, so it reaches ``sdk.py``'s handler
    and is mapped to its ``exit_code``.
    """

    @functools.wraps(callback)
    def wrapper(sdk_ctx: SdkContext, **kwargs: Any) -> Any:
        try:
            code = callback(sdk_ctx, **kwargs)
        except SdkTestError as exc:
            raise FatalError(str(exc)) from exc
        if code:
            raise FatalError(
                _EXIT_MESSAGES.get(code, f"command failed with exit code {code}"),
                exit_code=code,
            )
        return code

    return wrapper


def _make_build_fn(sdk_ctx: SdkContext) -> Callable[..., str]:
    """The one capability the library cannot have: how to build firmware.

    Mirrors `core_ext`'s build command (``scons --board=<board> -j<n>``) rather
    than importing its private helper, so a change there cannot silently change
    how tests build.

    The output is captured, not streamed: the runner files it as ``build.log``.
    A compiler's wall of text is worth reading after the fact, not while it
    scrolls past. ``check=False`` for the same reason -- letting the runner
    raise on a non-zero exit would take the output with it, and a failed build's
    output is the only part that was ever going to be read.
    """

    def build_fn(project_dir: str, platform: str, extra_args: List[str], jobs: int) -> str:
        command = ["scons", f"--board={platform}"]
        command += [str(argument) for argument in extra_args]
        command.append(f"-j{jobs}")
        completed = sdk_ctx.runner.run(
            command,
            cwd=project_dir,
            check=False,
            capture_output=True,
            # Decoded explicitly rather than by the locale. Capturing means
            # decoding, and a Windows console in a GBK code page chokes on the
            # bytes a compiler emits -- an exception from a helper thread, no
            # less, since subprocess reads the pipe on one. Streaming never hit
            # this because it never decoded anything.
            encoding="utf-8",
            errors="replace",
        )
        output = (completed.stdout or "") + (completed.stderr or "")
        if completed.returncode != 0:
            raise BuildError(
                f"scons failed for {platform} (exit {completed.returncode}); "
                "the compiler's output is in build.log",
                output,
            )
        return output

    return build_fn


# -- option groups -----------------------------------------------------------

_SUITE_ROOT_OPTION = {
    "names": ["-T", "--testsuite-root"],
    "help": "Root directory to search for testcase.yaml files. Defaults to the SDK's example/ tree.",
    "type": str,
    "default": None,
}

_TESTCASE_OPTION = {
    "names": ["-t", "--testcase"],
    "help": "Test id (or bare tests key) to select. Repeatable.",
    "type": str,
    "multiple": True,
    "default": None,
}

_TAG_OPTION = {
    "names": ["--tag"],
    "help": "Select tests carrying at least one of these tags. Repeatable.",
    "type": str,
    "multiple": True,
    "default": None,
}

_BINDING_OPTION = {
    "names": ["-b", "--binding"],
    "help": "Hardware-map file describing this lab's testbeds.",
    "type": str,
    "default": None,
}

_NO_CONFIRM_OPTION = {
    "names": ["--no-confirm"],
    "is_flag": True,
    "default": False,
    "help": (
        "Never stop to ask. A board nothing can reset still prints what to do, "
        "but the run will not wait for an operator -- which also means a "
        "harness that writes to the device cannot be used on it."
    ),
}


_CONFIRM_RESET_OPTION = {
    "names": ["--confirm-reset"],
    "is_flag": True,
    "default": False,
    "help": (
        "Do not stop to ask: the board's boot is already arranged (its download "
        "boots it, or an operator is standing by). Without this, a board nothing "
        "can reset may only be driven by a harness that never writes -- unity is "
        "refused -- and the confirmation is offered only on a terminal."
    ),
}


def _assume_booted(echo: Callable[[str], None]) -> Callable[[str], bool]:
    """The ``--confirm-reset`` answer: say what is being assumed, then agree.

    It echoes the question the runner would have asked rather than swallowing
    it, so the log still records *what* was taken on trust.
    """

    def assume(message: str) -> bool:
        echo(message)
        echo("  boot assumed (--confirm-reset)")
        return True

    return assume


def _confirm_for(
    no_confirm: bool,
    confirm_reset: bool,
    echo: Callable[[str], None],
) -> Optional[Callable[[str], bool]]:
    """Which answer the runner gets about a board an operator has to boot.

    Three states, and they really are three:

    * **nothing given** -- ask, when there is a terminal to ask on. The only one
      of the three that both stops and knows the answer.
    * **``--no-confirm``** -- nobody to ask, therefore no answer, therefore a
      harness that writes to the device is refused rather than left to race a
      boot that may wipe its command.
    * **``--confirm-reset``** -- an answer without asking. The caller states the
      board is up; nothing is verified, so the promise is theirs to keep.
    """
    if no_confirm and confirm_reset:
        raise UsageError(
            "--confirm-reset and --no-confirm contradict each other: one says "
            "the boot is arranged, the other says there is nobody to arrange it."
        )
    if no_confirm:
        return None
    if confirm_reset:
        return _assume_booted(echo)
    return _make_confirm(echo)


def _make_confirm(echo: Callable[[str], None]) -> Optional[Callable[[str], bool]]:
    """How the runner asks an operator a question, when there is one to ask.

    The library takes a callback rather than reading stdin itself, so that it
    stays usable from a test or an embedder with no terminal at all. The check
    for a terminal lives here, at the edge, where the terminal actually is --
    and it is what keeps a CI run from blocking forever on a prompt nobody can
    answer.
    """
    if not sys.stdin.isatty():
        return None

    def ask(message: str) -> bool:
        echo(message)
        try:
            answer = input("  continue? [Y/n] ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            echo("")
            return False
        return not answer.startswith("n")

    return ask


def _suite_root(value: Optional[str]) -> str:
    return value or _default_suite_root()


# -- callbacks ---------------------------------------------------------------


@_guard
def test_list_callback(
    sdk_ctx: SdkContext,
    testsuite_root: Optional[str] = None,
    testcase: Optional[List[str]] = None,
    tag: Optional[List[str]] = None,
    json: bool = False,
) -> int:
    # Note: click derives the keyword from the flag, so `--json` arrives as
    # `json`. The library's own parameter is named `as_json`.
    return sdk_test_cli.list_tests(
        _suite_root(testsuite_root),
        testcase_ids=testcase,
        tags=tag,
        as_json=json,
    )


@_guard
def test_list_fixtures_callback(
    sdk_ctx: SdkContext,
    binding: Optional[str] = None,
    agent_url: Optional[str] = None,
) -> int:
    return sdk_test_cli.list_fixtures(binding=binding, agent_url=agent_url)


@_guard
def test_info_callback(sdk_ctx: SdkContext) -> int:
    return sdk_test_cli.describe_environment()


@_guard
def test_plan_callback(
    sdk_ctx: SdkContext,
    testsuite_root: Optional[str] = None,
    testcase: Optional[List[str]] = None,
    tag: Optional[List[str]] = None,
    binding: Optional[str] = None,
    platform: Optional[str] = None,
    backend: Optional[str] = None,
) -> int:
    return sdk_test_cli.describe_plan(
        _suite_root(testsuite_root),
        testcase_ids=testcase,
        tags=tag,
        binding=binding,
        platform=platform,
        backend=backend,
    )


@_guard
def test_run_callback(
    sdk_ctx: SdkContext,
    testsuite_root: Optional[str] = None,
    testcase: Optional[List[str]] = None,
    tag: Optional[List[str]] = None,
    binding: Optional[str] = None,
    platform: Optional[str] = None,
    backend: Optional[str] = None,
    agent_url: str = "http://127.0.0.1:8760",
    no_build: bool = False,
    jobs: int = 8,
    timeout: Optional[int] = None,
    report: Optional[str] = None,
    report_dir: Optional[str] = None,
    report_name: Optional[str] = None,
    report_suffix: Optional[str] = None,
    outdir: Optional[str] = None,
    inline_logs: bool = False,
    fail_fast: bool = False,
    no_confirm: bool = False,
    confirm_reset: bool = False,
) -> int:
    options = RunOptions(
        testsuite_root=_suite_root(testsuite_root),
        testcase_ids=list(testcase or []),
        tags=list(tag or []),
        binding=binding,
        agent_url=agent_url,
        platform=platform,
        backend=backend,
        # `--no-build` is the flag a user types; the library takes the positive
        # form so that its default (build) reads the same as the CLI's.
        build=not no_build,
        jobs=jobs,
        timeout=timeout,
        report_path=report,
        report_dir=report_dir,
        report_name=report_name,
        report_suffix=report_suffix or "",
        out_dir=outdir,
        inline_logs=inline_logs,
        fail_fast=fail_fast,
        python_exe=sys.executable,
    )
    return sdk_test_cli.run_tests(
        options,
        build_fn=_make_build_fn(sdk_ctx),
        confirm=_confirm_for(no_confirm, confirm_reset, print),
    )


# -- registration ------------------------------------------------------------


def register(registry: CommandRegistry) -> None:
    registry.group(
        path="test",
        help="Automated test framework: discover, build, run on hardware, report.",
    )

    registry.command(
        path="test/list",
        callback=test_list_callback,
        help="List discovered test cases without building or running anything.",
        options=[
            _SUITE_ROOT_OPTION,
            _TESTCASE_OPTION,
            _TAG_OPTION,
            {
                "names": ["--json"],
                "help": "Emit machine-readable JSON (for CI matrix generation).",
                "is_flag": True,
                "default": False,
            },
        ],
    )

    registry.command(
        path="test/list-fixtures",
        callback=test_list_fixtures_callback,
        help="Show the testbeds and fixtures a binding file describes.",
        options=[
            _BINDING_OPTION,
            {
                "names": ["-a", "--agent-url"],
                "help": "Also ask a running Lab Agent which testbeds are free.",
                "type": str,
                "default": None,
            },
        ],
    )

    registry.command(
        path="test/plan",
        callback=test_plan_callback,
        help="Show what a run would do (platform, project, testbed) without doing it.",
        options=[
            _SUITE_ROOT_OPTION,
            _TESTCASE_OPTION,
            _TAG_OPTION,
            _BINDING_OPTION,
            {
                "names": ["-p", "--platform"],
                "help": "Board name to build for. Defaults to the binding's choice.",
                "type": str,
                "default": None,
            },
            {
                "names": ["--backend"],
                "help": "Execution backend. Defaults to the one the platform implies.",
                "type": click.Choice(backend_names(), case_sensitive=False),
                "default": None,
            },
        ],
    )

    registry.command(
        path="test/run",
        callback=test_run_callback,
        help="Build and run tests, then write a JUnit report. Exit 1 if any test fails.",
        options=[
            _SUITE_ROOT_OPTION,
            _TESTCASE_OPTION,
            _TAG_OPTION,
            _BINDING_OPTION,
            _NO_CONFIRM_OPTION,
            _CONFIRM_RESET_OPTION,
            {
                "names": ["-p", "--platform"],
                "help": "Board name to build for. Defaults to the binding's choice.",
                "type": str,
                "default": None,
            },
            {
                "names": ["--backend"],
                "help": "Execution backend. Defaults to the one the platform implies.",
                "type": click.Choice(backend_names(), case_sensitive=False),
                "default": None,
            },
            {
                "names": ["--agent-url"],
                "help": "Base URL of the Lab Agent holding the hardware.",
                "type": str,
                "default": "http://127.0.0.1:8760",
            },
            {
                "names": ["--no-build"],
                "help": "Skip the build step and use whatever is already in build_<board>/.",
                "is_flag": True,
                "default": False,
            },
            {
                "names": ["-j", "--jobs"],
                "help": "Parallel build jobs.",
                "type": int,
                "default": 8,
            },
            {
                "names": ["--timeout"],
                "help": "Override every test's timeout, in seconds.",
                "type": int,
                "default": None,
            },
            {
                "names": ["--report"],
                "help": (
                    "Legacy spelling: pins the JUnit report's directory and name. "
                    "Reports are written by default now; use --report-dir/-name "
                    "unless you need this exact path."
                ),
                "type": str,
                "default": None,
            },
            {
                "names": ["--report-dir"],
                "help": "Where the reports go. Defaults to --outdir.",
                "type": str,
                "default": None,
            },
            {
                "names": ["--report-name"],
                "help": "Report file stem. Defaults to 'sifli'.",
                "type": str,
                "default": None,
            },
            {
                "names": ["--report-suffix"],
                "help": "Appended to the report file names; for CI, the build id.",
                "type": str,
                "default": None,
            },
            {
                "names": ["--outdir"],
                "help": (
                    "Root for this run's artifacts: each instance's device.log and "
                    "runner.log, the run-level runner.log, and the reports. "
                    "Defaults to test-out/ next to where you invoked sdk.py."
                ),
                "type": str,
                "default": None,
            },
            {
                "names": ["-i", "--inline-logs"],
                "help": (
                    "On failure, print the captured console log instead of just "
                    "naming the file it was written to."
                ),
                "is_flag": True,
                "default": False,
            },
            {
                "names": ["--fail-fast"],
                "help": "Stop after the first failure. Off by default: a nightly run wants the full picture.",
                "is_flag": True,
                "default": False,
            },
        ],
    )

    registry.command(
        path="test/info",
        callback=test_info_callback,
        help="Show the harnesses and execution backends this build supports.",
        options=[],
    )
