# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Command implementations behind ``sdk.py test``.

Kept free of ``click`` and of ``sdk_py_actions`` so the whole surface can be
exercised from a unit test. ``tools/sdk_py_actions/test_ext.py`` is the only
place that knows about the CLI framework, and it does nothing but parse options
and map errors.

Each function returns a process exit code and writes human output through an
injected ``echo``, which is what makes the output assertable in tests.
"""

from __future__ import annotations

from typing import Any
from typing import Callable
from typing import Dict
from typing import List
from typing import Optional

from .backends import backend_names
from .backends import is_host_platform
from .backends import select_backend_name
from .binding import load_binding
from .discovery import discover
from .errors import SdkTestError
from .harness import IMPLEMENTED_HARNESSES
from .harness import SUPPORTED_HARNESSES
from .runner import RunOptions
from .runner import TestRunner

Echo = Callable[[str], None]

EXIT_OK = 0
EXIT_FAILURES = 1
EXIT_USAGE = 2


def resolve_test_ids(
    testsuite_root: str,
    testcase_ids: Optional[List[str]] = None,
    tags: Optional[List[str]] = None,
) -> List[Any]:
    """Discover and filter, sharing the runner's selection rules."""
    return TestRunner(
        RunOptions(testsuite_root=testsuite_root, testcase_ids=list(testcase_ids or []), tags=list(tags or []))
    ).instances()


def list_tests(
    testsuite_root: str,
    testcase_ids: Optional[List[str]] = None,
    tags: Optional[List[str]] = None,
    as_json: bool = False,
    echo: Optional[Echo] = None,
) -> int:
    """``sdk.py test list`` -- enumerate test cases without touching hardware."""
    echo = echo or print
    instances = resolve_test_ids(testsuite_root, testcase_ids, tags)

    if as_json:
        import json

        echo(
            json.dumps(
                [
                    {
                        "id": instance.id,
                        "harness": instance.harness,
                        "tags": instance.tags,
                        "timeout": instance.timeout,
                        "fixtures": [fixture.spec for fixture in instance.fixtures],
                        "depends_on": instance.depends_on,
                        "platform_allow": instance.platform_allow,
                        "integration_platforms": instance.integration_platforms,
                        "skip": instance.skip,
                        "build_only": instance.build_only,
                        "description": instance.description,
                    }
                    for instance in instances
                ],
                indent=2,
            )
        )
        return EXIT_OK

    if not instances:
        echo("No test cases found.")
        return EXIT_OK

    width = max(len(instance.id) for instance in instances)
    echo(f"{len(instances)} test case(s) under {testsuite_root}:")
    for instance in instances:
        flags = []
        if instance.skip:
            flags.append("skip")
        if instance.build_only:
            flags.append("build-only")
        if instance.harness not in IMPLEMENTED_HARNESSES:
            flags.append(f"{instance.harness} (not implemented)")
        suffix = f"  [{', '.join(flags)}]" if flags else ""
        echo(f"  {instance.id:<{width}}  {instance.harness:<8}{suffix}")
        if instance.description:
            echo(f"  {'':<{width}}  {instance.description}")
    return EXIT_OK


def list_fixtures(
    binding: Optional[str] = None,
    agent_url: Optional[str] = None,
    echo: Optional[Echo] = None,
) -> int:
    """``sdk.py test list-fixtures`` -- what can this lab run right now.

    Reads the binding alone when no agent is reachable, and augments it with
    live allocation state when one is. A test author needs both: which fixtures
    are wired, and whether the hardware is free.

    What a board *can do* is not here. That is the board's own declaration --
    ``sdk.py test list`` reads it through ``depends_on`` -- and reading it out of
    a binding as well is what let the two disagree.
    """
    echo = echo or print

    if not binding:
        echo("No --binding given, so there is no hardware to describe.")
        echo("Test cases can still be listed with `sdk.py test list`.")
        return EXIT_OK

    testbeds = load_binding(binding)
    if not testbeds:
        echo(f"{binding}: no testbeds defined.")
        return EXIT_OK

    live = {}
    if agent_url:
        from .agent_client import LabAgentClient

        try:
            live = {item["id"]: item for item in LabAgentClient(agent_url).testbeds()}
        except SdkTestError as exc:
            echo(f"Note: could not reach the Lab Agent ({exc}); showing the binding only.")

    echo(f"{len(testbeds)} testbed(s) from {binding}:")
    for testbed in testbeds:
        status = ""
        if testbed.id in live:
            status = " [busy]" if live[testbed.id].get("busy") else " [free]"
        echo(f"  {testbed.id}  ({testbed.platform}){status}")
        console = (
            f"{testbed.console.port} @ {testbed.console.baud}"
            if testbed.console
            else "none"
        )
        echo(f"    console:      {console}")
        echo(
            "    fixtures:     "
            + (", ".join(fixture.spec for fixture in testbed.fixtures) or "none")
        )

    fixtures = sorted({fixture.spec for testbed in testbeds for fixture in testbed.fixtures})
    echo("")
    echo(f"Fixtures offered: {', '.join(fixtures) or 'none'}")
    return EXIT_OK


def describe_plan(
    testsuite_root: str,
    testcase_ids: Optional[List[str]] = None,
    tags: Optional[List[str]] = None,
    binding: Optional[str] = None,
    platform: Optional[str] = None,
    backend: Optional[str] = None,
    echo: Optional[Echo] = None,
) -> int:
    """Show what a run would do, without doing it."""
    echo = echo or print
    runner = TestRunner(
        RunOptions(
            testsuite_root=testsuite_root,
            testcase_ids=list(testcase_ids or []),
            tags=list(tags or []),
            binding=binding,
            platform=platform,
            backend=backend,
        )
    )
    planned = runner.plan()

    if not planned:
        echo("Nothing to run.")
        return EXIT_OK

    # One line per run. A test that names several platforms produces several,
    # and reporting "3 test(s)" for five runs on five boards would under-report
    # precisely what a dry run is read for: how many boards this occupies.
    runs_of: Dict[str, int] = {}
    for item in planned:
        runs_of[item.id] = runs_of.get(item.id, 0) + 1
    if len(runs_of) == len(planned):
        echo(f"{len(planned)} test(s) would run:")
    else:
        echo(f"{len(runs_of)} test(s) would run, {len(planned)} run(s) in all:")

    seen: Dict[str, int] = {}
    for item in planned:
        seen[item.id] = seen.get(item.id, 0) + 1
        total = runs_of[item.id]
        mark = f"  [{seen[item.id]}/{total}]" if total > 1 else ""
        echo(f"  {item.id}{mark}")
        echo(f"    platform:   {item.platform} ({item.backend})")
        echo(f"    project:    {item.project_dir}")
        echo(f"    build dir:  {item.build_dir}")
        if item.out_dir:
            echo(f"    logs:       {item.out_dir}")
        if item.testbed:
            echo(f"    testbed:    {item.testbed.id}")
        if is_host_platform(item.platform):
            echo("    note:       no Lab Agent needed for this platform")
    return EXIT_OK


def run_tests(
    options: RunOptions,
    build_fn: Optional[Callable[..., Any]] = None,
    echo: Optional[Echo] = None,
    confirm: Optional[Callable[[str], bool]] = None,
) -> int:
    """``sdk.py test run`` -- build, allocate, run, release, report.

    ``confirm`` is how the runner asks an operator something mid-run; the CLI
    supplies it only when there is a terminal to ask on. Its absence is not an
    error -- a CI run has nobody to ask -- but it does decide whether a board
    nothing can reset may be driven by a harness that writes.
    """
    echo = echo or print

    runner = TestRunner(options, build_fn=build_fn, echo=echo, confirm=confirm)
    summary = runner.run()

    echo("")
    echo(report_summary(summary, inline_logs=options.inline_logs))
    if summary.total == 0:
        echo("No test cases matched.")
        return EXIT_USAGE
    return EXIT_OK if summary.ok else EXIT_FAILURES


def report_summary(summary, inline_logs: bool = False) -> str:
    from .report import format_summary

    return format_summary(summary, inline_logs=inline_logs)


def describe_environment(echo: Optional[Echo] = None) -> int:
    """What this build of the framework can do, for `test --help`-adjacent use."""
    echo = echo or print
    echo(f"harnesses:  {', '.join(SUPPORTED_HARNESSES)}")
    echo(f"  implemented: {', '.join(IMPLEMENTED_HARNESSES)}")
    echo(f"backends:   {', '.join(backend_names())}")
    echo("  board    : real hardware via a Lab Agent  [implemented]")
    echo("  pc_sim   : customer/boards/pc, no Lab Agent  [stub]")
    echo("  qemu     : QEMU, no Lab Agent  [stub]")
    return EXIT_OK


__all__ = [
    "describe_environment",
    "describe_plan",
    "list_fixtures",
    "list_tests",
    "run_tests",
    "select_backend_name",
    "EXIT_FAILURES",
    "EXIT_OK",
    "EXIT_USAGE",
]
