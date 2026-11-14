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

from . import params
from . import report
from .backends import backend_names
from .backends import is_host_platform
from .backends import select_backend_name
from .binding import check_fixture_contracts
from .binding import parse_binding
from .discovery import discover
from .fixture_types import default_fixture_types
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
                        # Key names only, derived without reading a params file:
                        # this command must stay usable on a machine that has no
                        # credentials at all.
                        "params": params.requirement_keys(
                            instance, default_fixture_types()
                        ),
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


def describe_plan(
    testsuite_root: str,
    testcase_ids: Optional[List[str]] = None,
    tags: Optional[List[str]] = None,
    platform: Optional[str] = None,
    backend: Optional[str] = None,
    echo: Optional[Echo] = None,
    agent_url: str = "http://127.0.0.1:8760",
    params_file: Optional[str] = None,
    param_overrides: Optional[Dict[str, str]] = None,
) -> int:
    """Show what a run would do, without doing it.

    The plan names a platform, not a board: which board of that platform will
    host a run is the Lab Agent's to choose, from its own configuration, at the
    moment hardware is requested. Parameters are resolved here too, with the
    same options and the same loopback rule as ``test run``, so a plan cannot
    pass where a run would refuse.
    """
    echo = echo or print
    runner = TestRunner(
        RunOptions(
            testsuite_root=testsuite_root,
            testcase_ids=list(testcase_ids or []),
            tags=list(tags or []),
            platform=platform,
            backend=backend,
            agent_url=agent_url,
            params_file=params_file,
            param_overrides=dict(param_overrides or {}),
        )
    )
    planned, turned_away = runner.plan_with_filtered()

    if not planned and not turned_away:
        echo("Nothing to run.")
        return EXIT_OK

    if planned:
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
            if is_host_platform(item.platform):
                echo("    note:       no Lab Agent needed for this platform")
    else:
        # Cases were selected and every one of them was turned away, so the
        # reason is the whole answer here. "Nothing to run." alone would read
        # as an empty tree, which is the one thing this is not.
        echo(report.format_turned_away(turned_away, nothing_planned=True))

    # The pairs a run would leave out, with why. Reported rather than dropped:
    # a case that silently does not run is the worst thing a dry run can do,
    # and "not for this board" is an answer -- not the error it used to be.
    # Without a run of its own the block above has already said all of this.
    if planned and turned_away:
        echo(report.format_turned_away(turned_away))
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


# -- prerequisites, checked without hardware ---------------------------------


def check_params(
    testsuite_root: str,
    testcase_ids: Optional[List[str]] = None,
    tags: Optional[List[str]] = None,
    show_unused: bool = False,
    as_json: bool = False,
    echo: Optional[Echo] = None,
    params_file: Optional[str] = None,
    overrides: Optional[Dict[str, str]] = None,
    agent_url: str = "http://127.0.0.1:8760",
) -> int:
    """``sdk.py test params check`` -- can every selected case be supplied?

    Keys come from the testcase and the fixture type contract, so this answer
    needs neither a build nor a Lab Agent: it can run in CI before anything
    expensive starts. Only names and sources are printed, never values.

    The resolution path is the same one ``test run`` and ``test plan`` use,
    including the loopback rule for secrets, so a pre-flight that passes is a
    run that will not be refused for a configuration reason.

    A problem is reported for every case that has one rather than stopping at
    the first, because the point of a pre-flight check is to hand back the whole
    list at once.

    :raises params.ParamsError: at least one case cannot be supplied.
    """
    echo = echo or print
    contracts = default_fixture_types()
    instances = resolve_test_ids(testsuite_root, testcase_ids, tags)

    path, searched = params.find_params_file(params_file)
    file_values = params.load_params_file(path) if path else {}

    rows: List[Dict[str, Any]] = []
    warnings: List[str] = []
    errors: List[Dict[str, str]] = []
    used: List[str] = []

    for instance in instances:
        try:
            resolved = params.resolve_params(
                instance,
                contracts,
                params_file=params_file,
                overrides=overrides,
                agent_url=agent_url,
            )
        except params.ParamsError as exc:
            errors.append({"case": instance.id, "message": str(exc)})
            continue
        used.extend(resolved.requested_keys)
        for key in resolved.requested_keys:
            entry = resolved.values.get(key)
            if entry is not None:
                rows.append(
                    {"case": instance.id, "name": key, "source": entry.source}
                )
        warnings.extend(f"{instance.id}: {item}" for item in resolved.warnings)

    unused = params.unused_keys(file_values, used) if show_unused else []
    ok = not errors

    if as_json:
        import json

        echo(
            json.dumps(
                {
                    "ok": ok,
                    "params_file": path,
                    "search_paths": searched,
                    "params": rows,
                    "unused": unused,
                    "warnings": warnings,
                    "errors": errors,
                },
                indent=2,
            )
        )
    else:
        if not instances:
            echo("No test cases found.")
        else:
            echo(f"params file: {path or '(none found)'}")
            for row in rows:
                echo(f"  {row['case']}")
                echo(f"    {row['name']}  <-  {row['source']}")
            if warnings:
                echo("warnings:")
                for warning in warnings:
                    echo(f"  {warning}")
            if unused:
                echo(f"unused keys in {path}:")
                for key in unused:
                    echo(f"  - {key}")
            if errors:
                echo("errors:")
                for error in errors:
                    echo(f"  {error['case']}: {error['message']}")

    if not ok:
        raise params.ParamsError(
            f"params check: {len(errors)} test case(s) cannot be supplied"
        )
    return EXIT_OK


def check_binding(
    path: str,
    as_json: bool = False,
    echo: Optional[Echo] = None,
) -> int:
    """``sdk.py test binding check`` -- is the lab's wiring consistent?

    Strict where the design is strict: an option that a fixture type contract
    sources from params must not appear in a binding, because the binding is
    committed and a password in it is the leak this mechanism exists to stop.
    Unknown keys only warn -- driver-private keys (an instrument's ``console``)
    are legitimate, and refusing them would be a second schema.

    :raises BindingError: the file is malformed, or a params-sourced key is set
        in a connection.
    """
    from .errors import BindingError

    echo = echo or print
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - pyyaml is a core requirement
        raise BindingError("pyyaml is required to read a binding file") from exc

    try:
        with open(path, "r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle)
    except OSError as exc:
        raise BindingError(f"Cannot read binding {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise BindingError(f"{path}: malformed YAML: {exc}") from exc

    testbeds = parse_binding(raw, source=path)
    errors, warnings = check_fixture_contracts(testbeds)
    ok = not errors

    if as_json:
        import json

        echo(
            json.dumps(
                {
                    "ok": ok,
                    "file": path,
                    "testbeds": [testbed.id for testbed in testbeds],
                    "errors": errors,
                    "warnings": warnings,
                },
                indent=2,
            )
        )
    else:
        echo(f"binding: {path} ({len(testbeds)} testbed(s))")
        for warning in warnings:
            echo(f"  warning: {warning}")
        for error in errors:
            echo(f"  error:   {error}")
        if ok and not warnings:
            echo("  ok")

    if not ok:
        raise BindingError(f"binding check: {len(errors)} problem(s) in {path}")
    return EXIT_OK


__all__ = [
    "check_binding",
    "check_params",
    "describe_environment",
    "describe_plan",
    "list_tests",
    "run_tests",
    "select_backend_name",
    "EXIT_FAILURES",
    "EXIT_OK",
    "EXIT_USAGE",
]
