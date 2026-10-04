# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""The ``pytest`` harness: the test directory brings its own orchestration.

This is the escape hatch. ``console`` and ``unity`` are declarative -- a list of
regexes, or a summary line to parse -- and that covers most cases. When a test
needs control flow (converse with a protocol, coordinate two boards, drive an
instrument, retry until a condition holds), no amount of regular expression
expresses it, and the honest answer is a program.

So the host hands the *already allocated* DUT to the test directory's own pytest
suite. The suite reaches the device through the ``dut`` fixture, which reads the
coordinates below out of the environment -- that is what makes this the same
testbed the runner already claimed, rather than a second one.

The exit code is the verdict; the captured output is the evidence. Nothing is
parsed out of pytest's own report, because pytest is the authority on whether
its tests passed and re-deriving that would be a second opinion nobody asked for.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import TYPE_CHECKING

from ..errors import HarnessError

if TYPE_CHECKING:  # pragma: no cover - type hints only
    from ..model import TestInstance
    from . import ConsoleLike
    from . import HarnessOutcome
    from . import RunContext

# -- environment contract ----------------------------------------------------
#
# The names live here, next to the code that sets them, and pytest_plugin.py
# imports them from here. One definition, so the two halves cannot drift.

#: Base URL of the Lab Agent that holds the allocated testbed.
ENV_AGENT_URL = "SIFLI_TEST_AGENT_URL"
#: The testbed the runner already allocated and opened a console on.
ENV_TESTBED_ID = "SIFLI_TEST_TESTBED_ID"
#: Board name, for tests that want to branch on the platform.
ENV_PLATFORM = "SIFLI_TEST_PLATFORM"
#: Directory holding the build outputs, so a test can find artifacts.
ENV_BUILD_DIR = "SIFLI_TEST_BUILD_DIR"
#: Directory the testcase.yaml lives in.
ENV_TESTCASE_DIR = "SIFLI_TEST_TESTCASE_DIR"
#: The test id, for reporting.
ENV_CASE_ID = "SIFLI_TEST_CASE_ID"

#: Extra wall-clock margin granted to the pytest subprocess over the harness
#: timeout, so that pytest's own diagnostics survive long enough to be captured.
_SUBPROCESS_MARGIN = 60.0

#: pytest prints a final "=== 3 passed, 1 failed in 1.2s ===" line; that is the
#: most useful one-line summary to carry into the result.
_OUTCOME_RE = re.compile(r"^=+\s*(?P<summary>.+?)\s*=+$", re.MULTILINE)


def tools_dir() -> str:
    """The ``tools/`` directory, which holds both ``sdk_test`` and ``lab_agent``."""
    return os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def plugin_env(
    instance: "TestInstance",
    context: Optional["RunContext"],
) -> Dict[str, str]:
    """The environment the spawned pytest suite needs to reach the DUT."""
    if context is None:
        # Imported here, not at module scope: `RunContext` lives in the package
        # __init__, which imports this module for dispatch.
        from . import RunContext

        context = RunContext()
    env = {
        ENV_TESTCASE_DIR: instance.testcase_dir,
        ENV_CASE_ID: instance.id,
        # So `-p sdk_test.pytest_plugin` resolves inside the subprocess, and so
        # a suite can import sdk_test helpers (ConsoleMatcher, parse_unity_output)
        # without every test directory shipping a conftest.py.
        "PYTHONPATH": _prepend_pythonpath(tools_dir()),
    }
    if context.agent_url:
        env[ENV_AGENT_URL] = context.agent_url
    if context.testbed_id:
        env[ENV_TESTBED_ID] = context.testbed_id
    if context.build_dir:
        env[ENV_BUILD_DIR] = context.build_dir
    if instance.platform or context.platform:
        env[ENV_PLATFORM] = str(instance.platform or context.platform)
    return env


def _prepend_pythonpath(directory: str) -> str:
    existing = os.environ.get("PYTHONPATH", "")
    if not existing:
        return directory
    return directory + os.pathsep + existing


def resolve_pytest_root(instance: "TestInstance") -> str:
    """Where the test directory's pytest suite lives."""
    root = str(instance.harness_config.get("pytest_root") or ".")
    if os.path.isabs(root):
        return os.path.normpath(root)
    return os.path.normpath(os.path.join(instance.testcase_dir, root))


def build_command(instance: "TestInstance", root: str, python_exe: str) -> List[str]:
    """Assemble the pytest invocation.

    ``-p no:cacheprovider`` keeps pytest from dropping a ``.pytest_cache`` into
    the source tree, which would show up as an untracked directory in every
    developer's checkout after a test run.
    """
    command = [
        python_exe,
        "-m",
        "pytest",
        root,
        "-v",
        "--no-header",
        "-p",
        "no:cacheprovider",
        # Loaded explicitly so a test directory needs no conftest.py to get the
        # `dut` and `console` fixtures.
        "-p",
        "sdk_test.pytest_plugin",
    ]
    command += [str(argument) for argument in instance.harness_config.get("pytest_args") or []]
    return command


def ensure_available(python_exe: str) -> None:
    """Fail with instructions rather than a bare ModuleNotFoundError."""
    try:
        completed = subprocess.run(
            [python_exe, "-c", "import pytest"],
            capture_output=True,
            text=True,
            check=False,
        )
    except FileNotFoundError as exc:
        raise HarnessError(f"Cannot run {python_exe}: {exc}") from exc

    if completed.returncode != 0:
        raise HarnessError(
            f'The "pytest" harness needs pytest, but it is not installed in '
            f"{python_exe}. It is part of the SDK's locked environment, so "
            f"re-running install.ps1 (or install.sh) provides it; otherwise:\n"
            f"    {python_exe} -m pip install pytest"
        )


def _summarise(output: str, returncode: int) -> str:
    """One line describing the outcome, preferring pytest's own summary."""
    matches = _OUTCOME_RE.findall(output)
    if matches:
        # The last bracketed line is the final tally; earlier ones are section
        # headers such as "test session starts".
        for candidate in reversed(matches):
            if " in " in candidate or "passed" in candidate or "failed" in candidate:
                return candidate
        return matches[-1]
    return f"pytest exited with {returncode}"


def run(
    instance: "TestInstance",
    console: "ConsoleLike",
    timeout: float,
    poll: float = 0.5,
    context: Optional["RunContext"] = None,
) -> "HarnessOutcome":
    """Run the test directory's pytest suite against the allocated DUT.

    ``console`` is unused: the suite drives the device itself through the
    ``dut`` fixture. It stays in the signature because every harness driver
    takes the same arguments, which is what lets the runner treat them alike.
    """
    from . import HarnessOutcome
    from . import RunContext as _RunContext

    context = context if context is not None else _RunContext()
    interpreter = context.python_exe or sys.executable

    root = resolve_pytest_root(instance)
    if not os.path.isdir(root):
        raise HarnessError(
            f'harness_config.pytest_root "{root}" is not a directory'
        )

    if not _has_python_tests(root):
        raise HarnessError(
            f'No test_*.py found under "{root}". The "pytest" harness expects '
            "the test directory to ship its own suite; if you only need to match "
            'console output, use harness "console" instead.'
        )

    ensure_available(interpreter)

    command = build_command(instance, root, interpreter)
    env = dict(os.environ)
    env.update(plugin_env(instance, context))
    # Keep the emitted output stable and free of a user's own pytest settings.
    env["PYTHONDONTWRITEBYTECODE"] = "1"

    try:
        completed = subprocess.run(
            command,
            cwd=instance.testcase_dir,
            env=env,
            capture_output=True,
            text=True,
            timeout=max(1.0, float(timeout)) + _SUBPROCESS_MARGIN,
            check=False,
        )
    except subprocess.TimeoutExpired as expired:
        partial = _decode(expired.stdout) + _decode(expired.stderr)
        return HarnessOutcome(
            passed=False,
            detail=f"pytest timed out after {timeout:.0f}s",
            log=partial,
        )
    except FileNotFoundError as exc:
        raise HarnessError(f"Cannot run pytest: {exc}") from exc

    output = (completed.stdout or "") + (completed.stderr or "")
    return HarnessOutcome(
        passed=completed.returncode == 0,
        detail=_summarise(output, completed.returncode),
        log=output,
    )


def _has_python_tests(root: str) -> bool:
    for _current, _subdirs, filenames in os.walk(root):
        if any(
            name.startswith("test_") and name.endswith(".py") for name in filenames
        ):
            return True
    return False


def _decode(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)
