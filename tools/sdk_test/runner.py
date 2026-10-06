# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Orchestration: discover, match, build, allocate, run, release, report.

One instance's life, in order:

1. **plan** -- find the test, decide which platform it runs on, and where its
   project and build directory are. This is pure computation and is available
   without touching hardware, which is what makes ``--dry-run`` honest.
2. **build** -- ``scons --board=<platform>`` in the project directory.
3. **open** -- ask the backend for a prepared target with an open console.
4. **run** -- hand the console to the harness and take its verdict.
5. **close** -- release in a ``finally``, so a crash or a timeout still returns
   the board to the pool.

The build callable is injected rather than imported. That keeps this package
independent of ``sdk_py_actions`` (so it can be unit-tested on its own), and it
means the runner never needs to know how a build is spelled.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from dataclasses import field
from typing import Any
from typing import Callable
from typing import List
from typing import Optional
from typing import Tuple

from . import report
from .agent_client import LabAgentClient
from .backends import create_backend
from .backends import is_host_platform
from .backends import select_backend_name
from .discovery import discover
from .errors import BackendError
from .errors import BuildError
from .errors import DiscoveryError
from .errors import SdkTestError
from .harness import RunContext
from .harness import run as run_harness
from .harness import unity
from .model import TestInstance
from .platforms import load_platform_features
from .platforms import unsatisfied
from .report import ERROR
from .report import FAILED
from .report import PASSED
from .report import SKIPPED
from .report import RunSummary
from .report import TestResult

#: Files that mark a directory as a buildable project.
_PROJECT_MARKERS = ("SConstruct", "proj.conf")

#: How deep below the testcase directory to look for a lone project.
_PROJECT_SEARCH_DEPTH = 2

#: Core suffixes the build system recognises.
_CORE_SUFFIXES = ("_hcpu", "_lcpu", "_acpu")

#: Where a run puts its artefacts when ``--outdir`` is not given. Twister's
#: counterpart is ``twister-out`` in the current directory; the reports land at
#: its root and each instance gets a subtree.
DEFAULT_OUT_DIR = "test-out"

#: Report file stem; the JSON and the JUnit differ only by extension.
DEFAULT_REPORT_NAME = "sifli"


def canonical_board_name(platform: str) -> str:
    """Mirror ``building.py``'s ``GetBoardName``.

    A board named without a core suffix means HCPU, and the build directory is
    named after the *canonical* name: ``--board=sf32lb52-lcd_n16r8`` compiles
    into ``build_sf32lb52-lcd_n16r8_hcpu``. Getting this wrong is quiet -- the
    build succeeds and the run then looks for its artifacts in a directory that
    was never created -- so the rule is mirrored here rather than guessed.
    """
    if any(platform.endswith(suffix) for suffix in _CORE_SUFFIXES):
        return platform
    return platform + "_hcpu"


def is_project_dir(path: str) -> bool:
    return all(
        os.path.isfile(os.path.join(path, marker)) for marker in _PROJECT_MARKERS
    )


def find_project_dir(instance: TestInstance) -> str:
    """Locate the SConstruct to build for a test instance.

    A testcase file sits *beside* the project it drives, not inside it: the
    convention in this repository is ``<testcase dir>/project/`` (as in
    ``example/get-started/hello_world/rtt/project``). So the explicit
    ``project_dir`` wins, then the two conventional locations, then a bounded
    search that only accepts an unambiguous answer.
    """
    base = instance.testcase_dir

    if instance.project_dir:
        candidate = os.path.normpath(os.path.join(base, instance.project_dir))
        if not is_project_dir(candidate):
            raise DiscoveryError(
                f'"{instance.id}": project_dir "{instance.project_dir}" does not '
                f"contain {' and '.join(_PROJECT_MARKERS)}"
            )
        return candidate

    for candidate in (base, os.path.join(base, "project")):
        if is_project_dir(candidate):
            return candidate

    found: List[str] = []
    for current, subdirs, _files in os.walk(base):
        depth = 0 if current == base else os.path.relpath(current, base).count(os.sep) + 1
        if depth >= _PROJECT_SEARCH_DEPTH:
            subdirs[:] = []
            continue
        for name in list(subdirs):
            path = os.path.join(current, name)
            if is_project_dir(path):
                found.append(path)

    if len(found) == 1:
        return found[0]

    if not found:
        raise DiscoveryError(
            f'"{instance.id}": no project found near {base}. Expected a directory '
            f"containing {' and '.join(_PROJECT_MARKERS)} either there, in its "
            '"project/" subdirectory, or set "project_dir" in the testcase.'
        )

    listing = ", ".join(sorted(os.path.relpath(path, base) for path in found))
    raise DiscoveryError(
        f'"{instance.id}": several projects found near {base} ({listing}); '
        'set "project_dir" in the testcase to say which one to build.'
    )


@dataclass
class RunOptions:
    """Everything a run needs, decoupled from how the CLI spells it."""

    testsuite_root: str
    testcase_ids: List[str] = field(default_factory=list)
    tags: List[str] = field(default_factory=list)
    agent_url: str = "http://127.0.0.1:8760"
    platform: Optional[str] = None
    backend: Optional[str] = None
    build: bool = True
    jobs: int = 8
    timeout: Optional[int] = None
    #: Legacy spelling: pins the JUnit report's directory and name. Kept because
    #: every existing caller and every CI snippet spells it this way; the JSON
    #: lands beside it under the same stem.
    report_path: Optional[str] = None
    #: Where the reports go. Defaults to :data:`DEFAULT_OUT_DIR`, so a run
    #: produces its artifacts without being asked.
    report_dir: Optional[str] = None
    #: Report file stem, defaulting to :data:`DEFAULT_REPORT_NAME`, or to
    #: ``report_path``'s stem when that was given.
    report_name: Optional[str] = None
    #: Appended to the report file names; for a CI run, the build or pipeline id.
    report_suffix: str = ""
    fail_fast: bool = False
    python_exe: Optional[str] = None
    holder: Optional[str] = None
    #: Root for the run's artefacts: per-instance ``device.log`` /
    #: ``handler.log``, and the reports. Relative paths resolve against the
    #: directory ``sdk.py`` was invoked from. See :func:`report.instance_log_dir`.
    out_dir: Optional[str] = None
    #: Print a failed instance's log to the console instead of pointing at the
    #: file. Twister spells this ``--inline-logs``.
    inline_logs: bool = False


@dataclass
class PlannedRun:
    """One test instance resolved to a concrete place to run it.

    A run names a platform and nothing finer. Which *board* of that platform
    answers it is the Lab Agent's to choose -- at the moment hardware is
    requested, from its own configuration -- so the plan has no testbed to name
    and the runner learns the board's id from the session it opens.
    """

    instance: TestInstance
    platform: str
    backend: str
    project_dir: str
    build_dir: str
    #: The board name the build system uses, which carries a core suffix even
    #: when the platform name does not. See :func:`canonical_board_name`.
    board: str = ""
    #: Where this instance's log files go; computed at plan time so ``test plan``
    #: can show it before anything runs.
    out_dir: str = ""

    @property
    def id(self) -> str:
        return self.instance.id


@dataclass
class FilteredRun:
    """One (instance, platform) pair the plan will not run, and why.

    A filtered pair is an answer rather than a failure: the test is fine, the
    platform is fine, they are simply not a match. It is reported instead of
    dropped, because a test that silently does not run is the worst thing this
    layer can produce -- and because saying *why* before anything is built is the
    whole value of asking the question this early.
    """

    instance: TestInstance
    platform: str
    reason: str

    @property
    def id(self) -> str:
        return self.instance.id


class TestRunner:
    """Runs a set of test instances and reports what happened."""

    def __init__(
        self,
        options: RunOptions,
        build_fn: Optional[Callable[..., Any]] = None,
        client_factory: Optional[Callable[[str], LabAgentClient]] = None,
        echo: Optional[Callable[[str], None]] = None,
        confirm: Optional[Callable[[str], bool]] = None,
    ) -> None:
        self.options = options
        self._build_fn = build_fn
        self._client_factory = client_factory or (lambda url: LabAgentClient(url))
        self._echo = echo or (lambda _message: None)
        #: Asks an operator to confirm something, given a message; returns
        #: whether they agreed. Absent means there is nobody to ask -- which is
        #: the norm in CI, and not an error, only a thinner guarantee.
        self._confirm = confirm
        self._client: Optional[LabAgentClient] = None
        #: What the agent said it was built from, once one has answered. Carried
        #: to the report so a result can be explained against the code that
        #: produced it -- an agent outlives many SDK checkouts.
        self._agent_revision = ""
        # Resolved once, here, because three call sites need this path: taking
        # the timestamp at each would drop the instance logs and the run log
        # into different directories whenever a run straddles a second.
        self._run_dir = report.run_dir(
            self.options.out_dir or DEFAULT_OUT_DIR, report.run_stamp()
        )

    # -- planning ------------------------------------------------------------

    def instances(self) -> List[TestInstance]:
        """Discover and filter the tests this run covers."""
        found = discover(self.options.testsuite_root, tags=self.options.tags or None)

        if self.options.testcase_ids:
            wanted = set(self.options.testcase_ids)
            by_exact = {instance.id: instance for instance in found}
            by_key = {instance.tests_key: instance for instance in found}
            selected = []
            unknown = []
            for name in self.options.testcase_ids:
                instance = by_exact.get(name) or by_key.get(name)
                if instance is None:
                    unknown.append(name)
                else:
                    selected.append(instance)
            if unknown:
                known = "\n  ".join(sorted(by_exact))
                raise DiscoveryError(
                    "Unknown test id(s): "
                    + ", ".join(unknown)
                    + "\nAvailable:\n  "
                    + known
                )
            return selected

        # A `skip` marker is a decision about the tree, not about hardware, so
        # it is applied here where `list` can show it too.
        return found

    def planned_platforms(self, instance: TestInstance) -> List[str]:
        """The platforms the test itself names, if it names any.

        ``integration_platforms`` is a list and is taken as one: a test naming
        three platforms is meant to run on all three. It is not a "preferred
        platform" field, and reading only its first entry silently discarded the
        rest of what a testcase asked for.
        """
        if self.options.platform:
            return [self.options.platform]
        return list(instance.integration_platforms)

    def filter_reason(self, instance: TestInstance, platform: str) -> Optional[str]:
        """Why this instance cannot run on this platform, or ``None``.

        This is phase one, and it is deliberately cheap: no build, no lab, no
        serial port. Everything consulted is platform metadata -- the testcase's
        own allow/exclude lists, and the board's declaration of the bus
        instances it has.

        The two vocabularies meet here. The test asks for a *type* (``spi``),
        the board declares *instances* (``spi1``), and any instance of that type
        answers the requirement -- because which bus a board routes a part out
        to is the board's business, and the test's own code already resolves it.
        See :mod:`sdk_test.platforms`.
        """
        if not instance.allows_platform(platform):
            return (f"platform {platform} not allowed "
                    f"(allow={instance.platform_allow or 'any'}, "
                    f"exclude={instance.platform_exclude or 'none'})")

        if not instance.depends_on:
            # Nothing is required, so there is nothing to look up -- and a board
            # nobody has annotated yet keeps working for tests that ask for
            # nothing. A missing declaration is an error only for a test that
            # states a requirement.
            return None

        features = load_platform_features(platform)
        missing = unsatisfied(features, instance.depends_on)
        if missing:
            declared = ", ".join(features) or "none"
            return (f"platform {platform} lacks {missing} "
                    f"(it declares: {declared})")
        return None

    def make_planned_run(self, instance: TestInstance, platform: str) -> PlannedRun:
        """Everything needed to build and open one (instance, platform) pair."""
        instance.platform = platform
        project_dir = find_project_dir(instance)
        board = canonical_board_name(platform)
        planned = PlannedRun(
            instance=instance,
            platform=platform,
            backend=self.options.backend or select_backend_name(platform),
            project_dir=project_dir,
            build_dir=os.path.join(project_dir, f"build_{board}"),
            board=board,
            out_dir=report.instance_log_dir(self._run_dir, platform, instance.id),
        )
        self.check_registry(planned)
        return planned

    def check_registry(self, planned: PlannedRun) -> None:
        """Refuse a Unity suite the firmware it is about to run on does not have.

        The registry is generated from what the project actually compiles, so a
        suite can leave a build while a testcase still names it -- dropped for a
        carrier that cannot compile the module, or renamed in the C and not in
        the yaml. On the device that surfaces as the suite failing; here it can
        say what really happened, before a serial port is opened.

        Silent when there is nothing to check against: not built yet, or a
        manifest belonging to another board.
        """
        instance = planned.instance
        if instance.harness != "unity":
            return

        suite = (instance.harness_config or {}).get("suite")
        if not suite:
            return

        suites = unity.registered_suites(planned.build_dir, planned.board)
        if suites is None or suite in suites:
            return

        raise DiscoveryError(
            f'"{instance.id}": suite "{suite}" is not in the firmware built for '
            f"{planned.platform} ({planned.board}); it registered "
            f"{', '.join(suites) or 'nothing'}. Either the suite was dropped for "
            f"this carrier, or harness_config.suite and the test sources have "
            f"drifted apart."
        )

    def plan_instance(self, instance: TestInstance
                      ) -> Tuple[List[PlannedRun], List[FilteredRun]]:
        """Resolve one instance: the runs it has, and the pairs it does not.

        Both halves are returned rather than one being raised, because "this
        test does not belong on that board" is information the caller reports,
        not an error that stops a suite.

        A run is (instance, platform, backend, build dir). Which *board* of that
        platform will host it is not decided here and cannot be: the SDK side
        holds no inventory of the lab, and the agent allocates from its own when
        hardware is requested. Naming no platform is therefore unplannable, and
        is refused by name rather than guessed at.
        """
        wanted = self.planned_platforms(instance)
        if not wanted:
            raise BackendError(
                f'"{instance.id}": no platform. Name one with '
                f'"integration_platforms" in the testcase or --platform on the '
                f"command line."
            )

        planned: List[PlannedRun] = []
        filtered: List[FilteredRun] = []
        for platform in wanted:
            reason = self.filter_reason(instance, platform)
            if reason:
                filtered.append(FilteredRun(instance, platform, reason))
                continue
            planned.append(self.make_planned_run(instance, platform))
        return planned, filtered

    def plan_one(self, instance: TestInstance) -> PlannedRun:
        """The first run for an instance, or raise explaining why it cannot run.

        A convenience for callers that want one run and no more. Note that a
        test naming several ``integration_platforms`` has more than one, and
        this hides them -- :meth:`plan` returns them all, and that is what a
        dry run must use.
        """
        planned, filtered = self.plan_instance(instance)
        if planned:
            return planned[0]
        if filtered:
            raise BackendError(f'"{instance.id}": {filtered[0].reason}')
        raise BackendError(f'"{instance.id}": nothing to run')

    def plan_with_filtered(self) -> Tuple[List[PlannedRun], List[FilteredRun]]:
        """Every run the selection would produce, and every pair it turns away.

        Both halves, because a dry run that answers "what runs" while hiding
        "what does not, and why" leaves the reader guessing whether a case was
        dropped on purpose.

        A pair the case's own gate turns away is an answer and not an error
        when ``--platform`` named the platform it was turned away from: the
        operator asked for one board, and this case is simply not that board's
        work. With no ``--platform`` the only platforms considered are the ones
        the case itself named, so a case turned away from *all* of them
        contradicts itself -- and that, like a case naming no platform at all,
        still raises.

        Which half is which is decided here rather than left to the caller:
        the caller's job is to print an answer, and :meth:`plan` is a thin
        wrapper for callers that only want the runs.
        """
        runs: List[PlannedRun] = []
        turned_away: List[FilteredRun] = []
        for instance in self.instances():
            planned, filtered = self.plan_instance(instance)
            turned_away.extend(filtered)
            if planned:
                runs.extend(planned)
                continue
            if not filtered:
                raise BackendError(f'"{instance.id}": nothing to run')
            if self.options.platform:
                continue
            raise BackendError(f'"{instance.id}": {filtered[0].reason}')
        return runs, turned_away

    def plan(self) -> List[PlannedRun]:
        """Resolve every selected instance to the runs it would produce.

        One entry per run, not per test case: an instance that names several
        platforms contributes several. Listing only the first under-reported
        the work by exactly the factor that matters most before a run starts --
        how many boards it occupies, and therefore how long it takes.

        Still strict where strictness is warranted: the first genuinely
        unresolvable test raises, which is the right behaviour for a dry run
        whose whole purpose is to tell a human what is wrong before anything is
        built or flashed. A case that ``--platform`` named and the case's own
        gate refused is not one of those -- it is not this board's work, and
        :meth:`plan_with_filtered` hands it back to be reported instead of
        raised. :meth:`run` deliberately does not use either of them -- there,
        one mis-configured test must not stop the rest.
        """
        runs, _turned_away = self.plan_with_filtered()
        return runs

    # -- running -------------------------------------------------------------

    def _agent(self) -> LabAgentClient:
        if self._client is None:
            self._client = self._client_factory(self.options.agent_url)
        return self._client

    def _holder(self) -> str:
        return self.options.holder or f"sdk.py test run (pid {os.getpid()})"

    def _close_session(self, backend: Any, session: Any, result: TestResult) -> None:
        """Release a target, recording a release failure rather than hiding it."""
        try:
            backend.close(session)
        except Exception as exc:  # noqa: BLE001 - reported, not swallowed
            note = f"releasing the target failed: {exc}"
            result.message = f"{result.message}; {note}" if result.message else note
            if result.status == PASSED:
                # A test that passed but left hardware held is not a pass: the
                # next run would fail for no visible reason.
                #
                # The stage moves only in this branch. An instance that already
                # failed at run and then also failed to release still went wrong
                # at run, and saying "close" would hide that.
                result.stage = report.STAGE_CLOSE
                result.status = ERROR

    def _confirm_boot(
        self, session: Any, platform: str, timeout: float, instance: TestInstance
    ) -> bool:
        """Stop and let an operator confirm the board has come up, when we can.

        Nothing on this side resets such a board, and nothing on this side can
        tell one that just booted from one that was already running. A person
        can. Stopping here is also what makes a harness that *writes* to the
        device safe on this bench: by the time it speaks, the boot is behind us
        and no further reset is coming.

        An auto-running firmware has already reported by the time this is
        answered, so the gate stops being about *when* the run starts and is
        only about the boot landing inside the console's lifetime: those results
        are on the wire from the first byte, and a boot that happened before the
        console opened is a boot nobody saw. The prompt says which of the two it
        is, because "press Enter to run the test" would be a lie in that case.

        Returns True when the run should continue, which includes having nobody
        to ask. That case is not silent -- the prompt still says what to do, and
        the harness's own timeout is the window -- but it is a thinner
        guarantee, and it is why a writing harness is refused without it.
        """
        name = f"{session.testbed_id} ({platform})"

        if self._confirm is None:
            # Usually there is nothing to do: a UART download ends by booting
            # the new image, and opening the console can reset the board again.
            # The press is the fallback for when that boot was missed.
            self._echo(
                f"[reset] {name} has no probe - waiting for the board to boot; "
                f"press its RESET button if nothing appears ({timeout:.0f}s)"
            )
            return True

        action = (
            "then press Enter. It is already reporting, so what matters is that "
            "the boot happens while this console is open."
            if instance.harness_config.get("autorun")
            else "then press Enter to let the run send its first command."
        )
        if not self._confirm(
            f"[reset] {name} has no probe. Press the board's RESET button if it "
            f"has not already booted, {action}"
        ):
            self._echo(f"[skip]  {session.testbed_id}: the boot was not confirmed")
            return False

        return True

    def run_one(self, planned: PlannedRun) -> TestResult:
        instance = planned.instance
        started = time.time()

        result = TestResult(
            id=instance.id,
            status=PASSED,
            platform=planned.platform,
        )

        if instance.skip:
            result.status = SKIPPED
            result.message = "skipped by the testcase"
            return result

        session = None
        backend = None
        #: Held past the try so the finally can put the harness's own account of
        #: the run into runner.log, even when the harness raised.
        outcome = None
        try:
            if self.options.build:
                self._echo(f"[build] {instance.id} for {planned.platform}")
                # Set before the call, not after: a stage records how far the
                # run got, so the one that raises is the one that must stick.
                result.stage = report.STAGE_BUILD
                try:
                    result.build_output = self._do_build(planned)
                except BuildError as exc:
                    # Filed on the way past even though the run is over: the
                    # reason a build broke is in the compiler's words, and the
                    # report is where someone will look for it.
                    result.build_output = exc.output
                    raise

            if instance.build_only:
                result.status = SKIPPED
                result.message = "build_only: built, not executed"
                return result

            result.stage = report.STAGE_OPEN
            backend = create_backend(
                planned.backend,
                None if is_host_platform(planned.platform) else self._agent(),
            )
            self._echo(f"[run]   {instance.id} via {backend.name}")
            session = backend.open(
                instance,
                planned.platform,
                build_dir=planned.build_dir,
                holder=self._holder(),
                will_confirm_boot=self._confirm is not None,
            )
            result.testbed_id = session.testbed_id or result.testbed_id
            result.platform = session.platform or result.platform
            # One run talks to one agent in practice, so the first session that
            # names a revision is the run's. Recorded, never enforced: see
            # sdk_test.version.
            if session.agent_revision and not self._agent_revision:
                self._agent_revision = session.agent_revision

            context = RunContext(
                python_exe=self.options.python_exe,
                agent_url=self.options.agent_url,
                testbed_id=session.testbed_id,
                build_dir=planned.build_dir,
                platform=result.platform,
            )
            timeout = float(self.options.timeout or instance.timeout)

            if session.manual_reset and not self._confirm_boot(
                session, result.platform, timeout, instance
            ):
                result.status = SKIPPED
                result.message = "the board's boot was not confirmed by the operator"
                return result

            result.stage = report.STAGE_RUN
            outcome = run_harness(instance, session.console, timeout, context=context)

            result.status = PASSED if outcome.passed else FAILED
            result.message = outcome.detail
            result.cases = report.case_results(outcome.cases)
            result.log = outcome.log
        except SdkTestError as exc:
            result.status = ERROR
            result.message = str(exc)
        except Exception as exc:  # noqa: BLE001 - one bad test must not stop the run
            result.status = ERROR
            result.message = f"{type(exc).__name__}: {exc}"
        finally:
            # Before closing, because the transcript dies with the session.
            self._capture_transcript(session, result)
            if backend is not None:
                self._close_session(backend, session, result)
            result.duration = time.time() - started
            # After the duration and the final status, so handler.log reports
            # what the run actually concluded rather than an interim view.
            self._write_instance_logs(planned, result, outcome)

        return result

    def _capture_transcript(self, session: Any, result: TestResult) -> None:
        """Take the device's own account of the run, while it still exists.

        Kept separate from ``result.log`` rather than replacing it: neither is a
        superset of the other. The transcript is more complete for unity (whose
        harness discards the boot log before running anything) and for console
        (which stops reading at its last match); but for pytest the harness log
        is the one holding the assertion text, and the transcript is only the
        device's side of the conversation. The report inlines the harness log,
        following twister; the transcript goes to ``device.log`` verbatim.

        Silent when there is no transcript -- a backend that cannot provide one,
        or a session that never opened.
        """
        if session is None:
            return
        result.transcript = getattr(session, "transcript", "") or ""
        result.dropped = int(getattr(session, "dropped", 0) or 0)

    def _write_instance_logs(
        self, planned: PlannedRun, result: TestResult, outcome: Any
    ) -> None:
        """Write this instance's logs: build, device, and the runner's account.

        A write failure is echoed rather than recorded on the result: losing
        evidence must not turn a green test red, and the verdict was settled
        before this ran.
        """
        if not planned.out_dir:
            return
        try:
            report.write_instance_logs(
                planned.out_dir,
                result.transcript,
                self._runner_log(planned, result, outcome),
                build_output=result.build_output,
            )
            result.out_dir = planned.out_dir
        except OSError as exc:
            self._echo(f"[warn]  {result.id}: writing logs to {planned.out_dir} failed: {exc}")

    def _runner_log(
        self, planned: PlannedRun, result: TestResult, outcome: Any
    ) -> str:
        """The narrative half of the evidence: what was run, and how it ended.

        device.log is the raw transcript and stays unedited so it can be diffed
        and pasted as-is; everything a reader needs to interpret it -- which
        board, which backend, whether the transcript is complete, which cases
        passed -- belongs here instead. The harness supplies the verdict, the
        case table and its own output; everything above those is the runner's,
        which is why the file is named after the runner rather than the harness.
        """
        instance = planned.instance
        retained = len(result.transcript)
        if retained and result.dropped:
            console = (
                f"{retained} bytes retained, {result.dropped} trimmed from the head "
                f"(the backend's buffer is capped)"
            )
        elif retained:
            console = f"{retained} bytes, complete"
        else:
            console = "nothing -- no console was opened"

        lines = [
            f"instance:   {result.id}",
            f"platform:   {result.platform or planned.platform}    board: {planned.board}",
            f"backend:    {planned.backend}",
            f"harness:    {instance.harness}",
            f"status:     {result.status}    stage: {result.stage or '-'}"
            f"    duration: {result.duration:.1f}s",
        ]
        if result.testbed_id:
            lines.append(f"testbed:    {result.testbed_id}")
        lines.append(f"build dir:  {planned.build_dir}")
        lines.append(f"console:    {console}")
        # Named here so a reader of the narrative is not left wondering whether
        # a build ran at all: the file is absent under --no-build, and a build
        # that printed nothing leaves nothing to file.
        if result.build_output:
            lines.append(f"build log:  {len(result.build_output)} bytes, see build.log")
        lines.append("")
        lines.append(f"verdict: {result.message or '(none)'}")

        failing = [case for case in result.cases if not case.passed]
        if result.cases:
            lines.append("")
            lines.append("cases:")
            for case in result.cases:
                where = f"  {case.location}" if case.location else ""
                detail = f"  {case.message}" if case.message else ""
                lines.append(
                    f"  {case.status.upper():5} {case.name}{where}{detail}"
                )
            if failing:
                lines.append(
                    f"  -> {len(failing)} of {len(result.cases)} failed: "
                    + ", ".join(case.name for case in failing)
                )

        harness_output = getattr(outcome, "log", "") if outcome is not None else ""
        lines.append("")
        lines.append("harness output:")
        lines.append(harness_output if harness_output else "(the harness produced none)")
        lines.append("")
        return "\n".join(lines)

    def _do_build(self, planned: PlannedRun) -> str:
        """Build, and return everything the compiler printed.

        The output is returned rather than streamed so the runner can file it
        as ``build.log``. A build's noise is only interesting after the fact,
        and the one time it matters most -- the build that failed -- is exactly
        when it would otherwise scroll past unread.
        """
        if self._build_fn is None:
            raise BackendError(
                f'"{planned.instance.id}": building was requested but no build '
                "function was supplied"
            )
        return self._build_fn(
            planned.project_dir,
            planned.board or planned.platform,
            list(planned.instance.extra_args),
            self.options.jobs,
        )

    def run(self) -> RunSummary:
        summary = RunSummary(root=self.options.testsuite_root, out_dir=self._run_dir)
        # Said up front rather than only in the closing `[log]` line: with one
        # tree per run the path is not guessable, and it is worth knowing while
        # the run is still going -- `device.log` can be tailed as it is written.
        self._echo(f"[outdir] {self._run_dir}")

        # Materialised up front: the count is part of the run's own record, and
        # "twelve discovered, three not for this board" only reads if the twelve
        # was captured before planning started dropping them.
        instances = self.instances()
        summary.discovered = len(instances)

        # Planned in full before anything runs, so the manifest below can name
        # the whole set -- "how many are left" is the question a long run
        # raises, and nothing else answers it while the run is still going.
        #
        # Resilience is preserved rather than traded away: a test that cannot be
        # placed is captured as an error in this pass and carried, not raised,
        # so one mis-configured testcase still cannot stop the other ninety-nine.
        plan: List[Any] = []
        for instance in instances:
            started = time.time()
            try:
                planned_runs, filtered = self.plan_instance(instance)
            except SdkTestError as exc:
                plan.append((instance, [], [], str(exc), started))
                continue
            plan.append((instance, planned_runs, filtered, None, started))

        scheduled = [(instance, run) for instance, runs, _, _, _ in plan for run in runs]
        # The pairs planning refused, handed to the manifest so the opening
        # block accounts for them. They are results too -- recorded below as
        # skips, and counted by the closing summary -- so a manifest that named
        # only the runs announced a smaller set than the run went on to report.
        turned_away = [
            record for _instance, _runs, filtered, _error, _started in plan
            for record in filtered
        ]
        if scheduled:
            self._echo(report.format_manifest([
                (index, len(scheduled), run.instance.id, run.platform,
                 run.instance.harness)
                for index, (_instance, run) in enumerate(scheduled, start=1)
            ], turned_away))
        elif turned_away:
            # Every selected pair was refused, so there is no runs block to
            # append to -- but the opening still has to account for what the
            # closing summary will call a skipped test, and this is the block
            # `test plan` prints for the same plan.
            self._echo(report.format_turned_away(turned_away, nothing_planned=True))

        total = len(scheduled)
        done = 0
        stop = False
        for instance, planned_runs, filtered, error, started in plan:
            if stop:
                break

            if error is not None:
                # A test that cannot even be placed (no testbed offers its
                # fixtures, its project is missing) is recorded as an error and
                # the rest of the suite still runs. Refusing to run anything
                # because one testcase is mis-configured would make a nightly
                # run useless for diagnosing the other ninety-nine.
                summary.add(
                    TestResult(
                        id=instance.id,
                        status=ERROR,
                        duration=time.time() - started,
                        message=error,
                        stage=report.STAGE_PLAN,
                    )
                )
                if self.options.fail_fast:
                    break
                continue

            # A pair phase one filtered out is reported, never dropped: the
            # difference between "the test failed" and "the test was not for
            # this board" is the whole reason the decision is made before the
            # build rather than after the boot.
            for record in filtered:
                summary.add(
                    TestResult(
                        id=record.id,
                        status=SKIPPED,
                        duration=0.0,
                        message=f"filtered: {record.reason}",
                        platform=record.platform,
                    )
                )

            for planned in planned_runs:
                done += 1
                # A rule before each test, numbered to match the manifest: with
                # several boards in a lab the per-test lines all look alike, and
                # without the marker there is no way to tell whose output is
                # whose -- or how much of the run is still to come.
                self._echo("")
                self._echo(report.separator(
                    f"[{done}/{total}] {planned.instance.id}  "
                    f"({planned.platform}, {planned.instance.harness})"
                ))
                result = self.run_one(planned)
                # And the verdict here, not only in the closing summary: a run
                # that takes an hour should not withhold every result until the
                # end of it.
                self._echo(report.format_result(result))
                summary.add(result)
                if self.options.fail_fast and result.failed:
                    self._echo("[stop]  fail-fast: stopping after the first failure")
                    stop = True
                    break

        summary.finished_at = time.time()
        # Read here rather than when the summary was built: which agent answered
        # is only known once one has. Every result is already in by now.
        summary.agent_revision = self._agent_revision
        self._write_artifacts(summary)
        return summary

    def _write_artifacts(self, summary: RunSummary) -> None:
        """Write the run-level log and the reports. Always, following twister.

        Nothing here is fatal. The verdict is already decided by this point, and
        an unwritable outdir is a filesystem problem rather than a test result --
        turning a green run red over it would be worse than losing the file.
        """
        summary.out_dir = self._run_dir
        reports_dir = (
            os.path.dirname(os.path.abspath(self.options.report_path))
            if self.options.report_path
            else os.path.abspath(self.options.report_dir or summary.out_dir)
        )
        stem = self._report_stem() + (self.options.report_suffix or "")

        try:
            run_log_path = report.write_text(
                os.path.join(summary.out_dir, "runner.log"), report.run_log(summary)
            )
            self._echo(f"[log]    {run_log_path}")
        except OSError as exc:
            self._echo(f"[warn]  writing the run log failed: {exc}")

        try:
            document = report.summary_to_dict(summary)
            json_path = os.path.join(reports_dir, stem + ".json")
            report.write_json(document, json_path)
            xml_path = os.path.join(reports_dir, stem + ".xml")
            report.write_junit_from_dict(document, xml_path)
            self._echo(f"[report] {json_path}")
            self._echo(f"[report] {xml_path}")
        except OSError as exc:
            self._echo(f"[warn]  writing reports failed: {exc}")

    def _report_stem(self) -> str:
        """The report file stem: explicit name, else ``--report``'s, else default."""
        if self.options.report_name:
            return self.options.report_name
        if self.options.report_path:
            # The extension is dropped on purpose: `--report junit.xml` names a
            # JUnit file, and the stem it yields is what the JSON beside it is
            # called. Keeping the extension would produce `junit.xml.json`.
            return os.path.splitext(os.path.basename(self.options.report_path))[0]
        return DEFAULT_REPORT_NAME
