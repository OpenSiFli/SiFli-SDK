# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Result collection, per-instance log files, and JUnit XML reporting.

Both artefacts are written with the standard library -- ``xml.etree`` for the
XML and plain file writes for the logs -- so the framework adds no dependency,
and CI (GitLab accepts ``junit`` artifacts directly) needs no plugin.

The layout follows Zephyr's twister, which splits a test's evidence by who
produced it, because a failure can be in any of the three layers and mixing
them buries the one that matters:

    build.log     what the compiler said
    device.log    what the device said, byte for byte, unedited
    runner.log    what the harness made of it, plus a header naming the run
    <fixture>.log what an instrument the test drove said (e.g. ``bt_peer-ble.log``)

Each is deliberately pure -- no header, no separator -- so that ``diff`` and
``grep`` work on it and it can be pasted into a bug report as-is. The compiler's
output in particular used to go straight to the terminal and nowhere else, which
left a failed build with nothing to file: the exit code is the same whether a
header is missing or a type is wrong.
"""

from __future__ import annotations

import os
import time
import xml.etree.ElementTree as ElementTree
from dataclasses import dataclass
from dataclasses import field
from typing import Any
from typing import Callable
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional

from .version import PROTOCOL
from .version import package_root
from .version import revision

#: The three per-instance logs, named after the layer that produced them.
BUILD_LOG = "build.log"
DEVICE_LOG = "device.log"
RUNNER_LOG = "runner.log"

#: The logs of the three layers; an instrument must not be named one of these.
_RESERVED_LOG_NAMES = {BUILD_LOG, DEVICE_LOG, RUNNER_LOG}

#: Device names Windows resolves before the filesystem, with or without an
#: extension: ``aux.log`` is the AUX device, not a file. ``conin$``/``conout$``
#: are not listed because ``$`` is sanitized away before this check.
_WINDOWS_DEVICE_NAMES = (
    {"con", "prn", "aux", "nul"}
    | {f"com{number}" for number in "123456789¹²³"}
    | {f"lpt{number}" for number in "123456789¹²³"}
)


def _is_reserved_stem(stem: str) -> bool:
    """Whether ``stem`` is a name Windows will not treat as a file."""
    # Windows checks only the part before the first dot.
    return stem.split(".", 1)[0].strip().lower() in _WINDOWS_DEVICE_NAMES


def _instrument_log_stem(spec: str) -> str:
    """The file stem for one spec, before collisions are considered.

    The spec is sanitized because a fixture is written ``bt_peer:ble``
    and ``:`` is not portable in a file name. A name the platform reserves is
    prefixed rather than allowed to overwrite a layer log (``Device`` would be
    ``device.log`` on a case-insensitive filesystem) or to address a device
    (``aux.log``, ``COM1.log``).
    """
    safe = "".join(char if char.isalnum() or char in "-_." else "-" for char in spec)
    layer_names = {name.lower() for name in _RESERVED_LOG_NAMES}
    if f"{safe}.log".lower() in layer_names or _is_reserved_stem(safe):
        return f"instrument-{safe}"
    return safe


def instrument_log_names(specs: Iterable[str]) -> Dict[str, str]:
    """One collision-free file name per spec, in the order given.

    Collisions are real even though they are rare: ``bt_peer:ble`` and a
    fixture actually named ``bt_peer-ble`` sanitize to the same stem, and
    ``build``/``instrument-build`` meet through the reserved-name rule. Naming
    the whole set together is what makes the two instruments keep two files --
    computing a name per spec could only turn the second one into an overwrite,
    and the report would then point both entries at one file.

    Comparison is case-insensitive, because the filesystem may be (Windows and
    macOS by default): ``Peer1`` and ``peer1`` are two instruments in the log
    format and one file on disk.
    """
    names: Dict[str, str] = {}
    used = set()
    for spec in specs:
        stem = _instrument_log_stem(spec)
        candidate = f"{stem}.log"
        counter = 2
        while candidate.lower() in used:
            candidate = f"{stem}-{counter}.log"
            counter += 1
        used.add(candidate.lower())
        names[spec] = candidate
    return names


def instrument_log_name(spec: str) -> str:
    """The file name for a spec considered on its own.

    Collision-free only for a single spec; the writer and the report use
    :func:`instrument_log_names` over the whole set so they cannot disagree.
    """
    return instrument_log_names([spec])[spec]

PASSED = "passed"
FAILED = "failed"
ERROR = "error"
SKIPPED = "skipped"

_STATUS_ORDER = (ERROR, FAILED, SKIPPED, PASSED)

#: How far a run got. A different axis from the status, and the reason ERROR
#: needs it: ERROR covers "planning refused", "the build failed", "the target
#: would not open" and "the console dropped mid-test", which want different
#: reactions and used to be told apart only by reading the message text.
#:
#: Recorded as the last stage *entered*, so it is set on a pass too -- "got as
#: far as run" is what makes a failure at build mean something.
STAGE_PLAN = "plan"
STAGE_BUILD = "build"
STAGE_OPEN = "open"
STAGE_RUN = "run"
STAGE_CLOSE = "close"

#: Five columns wide so the instance lines and the case lines under them line up.
_MARKERS = {PASSED: "PASS", FAILED: "FAIL", ERROR: "ERROR", SKIPPED: "SKIP"}


def _marker(status: str) -> str:
    return _MARKERS.get(status, status.upper())


@dataclass
class TestResult:
    """One executed (or skipped) test instance."""

    id: str
    status: str
    duration: float = 0.0
    message: str = ""
    testbed_id: Optional[str] = None
    platform: Optional[str] = None
    #: What the harness made of the device's output -- its verdict, and for
    #: pytest the assertion text that says why. This is what the report and the
    #: console inlines, following twister, which inlines ``handler.log`` and
    #: falls back to ``device.log`` only when there is no handler log.
    log: str = ""
    #: The device's own account, byte for byte, as the backend retained it.
    #:
    #: Kept separate from ``log`` because neither is a superset of the other.
    #: For unity and console the transcript is strictly more complete -- the
    #: unity harness calls ``_discard_pending`` before running anything, so its
    #: own log begins after the boot banner. For pytest the two are
    #: complementary: the transcript has the device bytes, while the harness log
    #: has the Python-side assertion that explains them.
    #:
    #: Written verbatim to ``device.log``; not inlined into the report, which
    #: would repeat up to a few megabytes per instance for no gain.
    transcript: str = ""
    #: Retained transcripts of instrument consoles the test opened, keyed by the
    #: fixture spec the test case declared (``bt_peer``, ``bt_peer:ble``).
    #:
    #: Written next to ``device.log``, one file per instrument, so a failure that
    #: spans two radios can be read from both sides without re-running it. Empty
    #: for a case that declared no instrument, and a stream the test never
    #: opened is absent rather than written blank -- an empty log would claim a
    #: silent instrument.
    instrument_logs: Dict[str, str] = field(default_factory=dict)
    #: Evidence that could not be collected after the verdict, one line each.
    #:
    #: For an instrument console that could not be read back (agent error,
    #: timeout). The verdict is already settled and must not change; these go
    #: into the runner log so a missing ``<fixture>.log`` is not mistaken for a
    #: test that never opened one.
    capture_warnings: List[str] = field(default_factory=list)
    #: Bytes trimmed off the head of the transcript by the backend's buffer cap,
    #: so a reader can tell a complete log from a truncated one.
    dropped: int = 0
    #: Everything the compiler printed, kept whether or not the build succeeded
    #: -- especially when it did not, since that is where the reason is. Empty
    #: when no build ran (``--no-build``) or when it produced no output.
    build_output: str = ""
    #: Per-case detail, where the harness has any (unity does, console does
    #: not). Kept as-is from the harness so this module needs no import of it;
    #: see :func:`case_results` for the duck-typed view the renderers use.
    cases: List[Any] = field(default_factory=list)
    #: Where this instance's device.log / runner.log were written. Empty when
    #: the run never got far enough to have evidence (planning failure).
    out_dir: str = ""
    #: The last stage entered: one of the ``STAGE_*`` values, or empty when the
    #: instance never reached the runner. On a failure this says *where* it
    #: failed without anyone having to parse the message text.
    stage: str = ""

    @property
    def passed(self) -> bool:
        return self.status == PASSED

    @property
    def failed(self) -> bool:
        return self.status in (FAILED, ERROR)


@dataclass
class CaseResult:
    """One case inside an instance, in this module's own vocabulary.

    The unity harness reports ``UnityCase`` objects with ``PASS``/``FAIL``/
    ``IGNORE`` spellings; :func:`case_results` maps them onto the statuses the
    renderers already use, so nothing in ``harness/`` has to know about
    ``report.py``.
    """

    name: str
    status: str
    message: str = ""
    file: str = ""
    #: The line that decided the case, zero when nothing did: see
    #: :meth:`__post_init__`.
    line: int = 0

    def __post_init__(self) -> None:
        """Drop a line no assertion produced.

        Only a failure has an assertion line: Unity prints ``__LINE__`` when an
        assertion trips. On a PASS line the number is the case's ordinal -- the
        value the device has to hand Unity because Unity prints it there -- and
        carrying that into the report is how "1" and "2" came to be reported as
        source locations. A case that passed therefore has no line here,
        whatever the harness sent, and every renderer reads one field.
        """
        if self.status == PASSED:
            self.line = 0

    @property
    def passed(self) -> bool:
        return self.status == PASSED

    @property
    def location(self) -> str:
        """``file:line`` when the harness gave one, else empty."""
        if not self.file:
            return ""
        return f"{self.file}:{self.line}" if self.line else self.file


#: Unity spells its per-case verdicts its own way.
_CASE_STATUS = {
    "PASS": PASSED,
    "FAIL": FAILED,
    "IGNORE": SKIPPED,
    "IGNORED": SKIPPED,
}


def case_results(cases: Any) -> List[CaseResult]:
    """Map a harness's per-case objects onto :class:`CaseResult`.

    Duck-typed on purpose: only the unity harness produces cases today, and
    coupling ``report.py`` to ``harness.unity`` to say so would make the
    dependency run the wrong way.
    """
    mapped: List[CaseResult] = []
    for case in cases or []:
        raw = str(getattr(case, "status", "") or "").upper()
        mapped.append(
            CaseResult(
                name=str(getattr(case, "name", "") or ""),
                status=_CASE_STATUS.get(raw, raw.lower() or FAILED),
                message=str(getattr(case, "message", "") or ""),
                file=str(getattr(case, "file", "") or ""),
                line=int(getattr(case, "line", 0) or 0),
            )
        )
    return mapped


@dataclass
class RunSummary:
    """Aggregate of a whole ``sdk.py test run``."""

    results: List[TestResult] = field(default_factory=list)
    started_at: float = field(default_factory=time.time)
    finished_at: Optional[float] = None
    #: The ``-T`` root discovery ran from, and where the artefacts went. Both
    #: are recorded because neither is recoverable from the results: the
    #: instance ids are relative to the first, and the second is a choice.
    root: str = ""
    out_dir: str = ""
    #: How many instances discovery found, before planning filtered any. Kept
    #: apart from ``total`` so that "twelve discovered, three not for this
    #: board" stays readable.
    discovered: int = 0
    #: What the Lab Agent said it was built from, when one took part. Recorded
    #: rather than checked: an agent is a long-running process and is routinely
    #: older than the checkout driving it, so this is the trail for whoever has
    #: to explain a strange result later. See :mod:`sdk_test.version`.
    agent_revision: str = ""

    def add(self, result: TestResult) -> None:
        self.results.append(result)

    @property
    def duration(self) -> float:
        end = self.finished_at if self.finished_at is not None else time.time()
        return max(0.0, end - self.started_at)

    def count(self, status: str) -> int:
        return sum(1 for result in self.results if result.status == status)

    @property
    def total(self) -> int:
        return len(self.results)

    @property
    def passed(self) -> int:
        return self.count(PASSED)

    @property
    def failed(self) -> int:
        return self.count(FAILED) + self.count(ERROR)

    @property
    def skipped(self) -> int:
        return self.count(SKIPPED)

    @property
    def ok(self) -> bool:
        return self.failed == 0 and self.total > 0

    def failures_by_status(self) -> Dict[str, int]:
        return {
            status: self.count(status)
            for status in _STATUS_ORDER
            if self.count(status)
        }


def run_stamp(when: Optional[float] = None) -> str:
    """A filesystem-safe, sortable label for one run: ``2026-10-03T17-09-22``.

    Local time, not UTC: the person reading the directory name is standing next
    to the board. No colons -- Windows forbids them in a file name -- and the
    result still sorts lexicographically in the same order it sorts
    chronologically, so listing the outdir puts the newest run last with no help
    from this tool.
    """
    return time.strftime("%Y-%m-%dT%H-%M-%S", time.localtime(when))


def run_dir(out_dir: str, stamp: str,
            taken: Optional[Callable[[str], bool]] = None) -> str:
    """``<out_dir>/<stamp>``, made unique, as an absolute path.

    One directory per run is what keeps a passing run's log around to compare
    the next failing one against -- and the passing run's log is the baseline
    that makes a flake legible. Writing every run into one tree threw the
    previous one away, which is the one thing this layout exists to prevent.

    Two runs starting within the same second must not merge into a single tree
    either, for the same reason, so the name takes a counter until it is free.
    ``taken`` is injectable so the collision rule can be tested without the
    filesystem.
    """
    taken = taken or os.path.exists
    # Made absolute up front so that what `taken` inspects is exactly what this
    # returns -- a predicate looking at relative paths while the caller compares
    # absolute ones would let two runs pick the same free-looking name.
    base = os.path.abspath(out_dir)
    candidate = os.path.join(base, stamp)
    counter = 2
    while taken(candidate):
        candidate = os.path.join(base, f"{stamp}-{counter}")
        counter += 1
    return candidate


def instance_log_dir(out_dir: str, platform: str, instance_id: str) -> str:
    """Where one instance's log files live, as an absolute path.

    Twister puts them in the instance's build directory and names them
    ``device.log`` / ``handler.log``. Two things make that literal translation
    wrong here:

    * Our ``build_dir`` is per *project+board*, not per instance -- and it is
      scons' territory, so ``scons -c`` would take the logs with it. Testing
      evidence should not be destroyed by a build command.
    * ``instance.id`` alone is not unique. One testcase listing two
      ``integration_platforms`` produces two planned runs that share an id, so
      the platform has to be part of the path or the second one overwrites the
      first.

    Absolute on purpose: the QEMU backend chdirs into the build directory, so a
    relative path would mean different things at different call sites.
    """
    return os.path.abspath(os.path.join(out_dir, platform, instance_id))


def write_instance_logs(out_dir: str, transcript: str, runner_log: str,
                        build_output: str = "",
                        instrument_logs: Optional[Dict[str, str]] = None) -> str:
    """Write this instance's logs; return the directory.

    Each file is one layer's own words, verbatim -- no header, no separator --
    so that any of them can be diffed, grepped and pasted as evidence:

        build.log     what the compiler said
        device.log    what the device said
        runner.log    what the harness made of it
        <fixture>.log what an instrument the test drove said (see below)

    ``build.log`` is written only when there is something to write: a test run
    with ``--no-build`` never invoked a compiler, and an empty file would claim
    one ran and said nothing.

    ``instrument_logs`` maps a declared fixture spec to its retained console
    transcript. Each becomes ``<fixture>.log`` in the same directory as
    ``device.log``, so two radios are read side by side rather than from one
    side's account of the other. A stream the test never opened is omitted by
    the caller, not written blank.

    The name ``runner.log`` is deliberate at both levels: this file is the
    runner's record of *one instance*, and ``<outdir>/runner.log`` is its record
    of the *whole run*. Same name, different scope, told apart by the path. The
    per-instance one used to be called ``handler.log``, which is twister's name
    for it; we call that layer "harness" everywhere else, so the name pointed at
    vocabulary that does not exist in this tree.
    """
    os.makedirs(out_dir, exist_ok=True)

    if build_output:
        build_path = os.path.join(out_dir, BUILD_LOG)
        with open(build_path, "w", encoding="utf-8", errors="replace") as handle:
            handle.write(build_output)

    device_path = os.path.join(out_dir, DEVICE_LOG)
    with open(device_path, "w", encoding="utf-8", errors="replace") as handle:
        handle.write(transcript)

    logs = instrument_logs or {}
    names = instrument_log_names(logs)
    for spec, text in logs.items():
        path = os.path.join(out_dir, names[spec])
        with open(path, "w", encoding="utf-8", errors="replace") as handle:
            handle.write(text)

    runner_path = os.path.join(out_dir, RUNNER_LOG)
    with open(runner_path, "w", encoding="utf-8", errors="replace") as handle:
        handle.write(runner_log)

    return out_dir


def write_text(path: str, text: str) -> str:
    """Write a text artifact, creating its directory. Returns the path."""
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", encoding="utf-8", errors="replace") as handle:
        handle.write(text)
    return path


def _indent(element: ElementTree.Element, level: int = 0) -> None:
    """In-place pretty printing (``ElementTree.indent`` needs Python 3.9+)."""
    pad = "\n" + "  " * level
    if len(element):
        if not element.text or not element.text.strip():
            element.text = pad + "  "
        for child in element:
            _indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = pad
    if level and (not element.tail or not element.tail.strip()):
        element.tail = pad


def summary_to_dict(summary: RunSummary) -> Dict[str, Any]:
    """The run as plain data. This is the document; the others project it.

    Twister works the same way -- its JUnit writer re-opens the JSON file and
    builds the XML from it -- and the reason is worth keeping: with one
    intermediate, the terminal, the JSON and the JUnit cannot drift apart, and a
    renderer that wants to change what it shows only has to change how it reads
    one dict.
    """
    return {
        "environment": {
            "framework": "sdk_test",
            # Which code produced this run, both sides of it. Neither is a gate
            # -- a dirty checkout is a normal working state -- but a result
            # whose agent was built from something else has to be explainable
            # afterwards, and this is the only place that can say so.
            "sdk_revision": revision(package_root()),
            "sdk_protocol": PROTOCOL,
            "agent_revision": summary.agent_revision,
            "started": _iso(summary.started_at),
            "finished": _iso(summary.finished_at),
            "root": summary.root,
            "out_dir": summary.out_dir,
        },
        "summary": {
            "discovered": summary.discovered,
            "total": summary.total,
            "passed": summary.passed,
            "failed": summary.count(FAILED),
            "errored": summary.count(ERROR),
            "skipped": summary.skipped,
            "duration": round(summary.duration, 3),
        },
        "testsuites": [_suite_to_dict(result) for result in summary.results],
    }


def _iso(when: Optional[float]) -> Optional[str]:
    if when is None:
        return None
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(when))


def _suite_to_dict(result: TestResult) -> Dict[str, Any]:
    suite: Dict[str, Any] = {
        "name": result.id,
        "status": result.status,
        "reason": result.message,
        "execution_time": round(result.duration, 3),
    }
    if result.platform:
        suite["platform"] = result.platform
    if result.testbed_id:
        suite["testbed"] = result.testbed_id
    if result.out_dir:
        suite["out_dir"] = result.out_dir
    if result.stage:
        suite["stage"] = result.stage
    if result.dropped:
        suite["transcript_dropped"] = result.dropped
    if result.instrument_logs:
        # Names only: the transcripts themselves live beside device.log, for the
        # same reason the device's does -- a report should not copy megabytes.
        # The whole set is named together so the report and the writer agree on
        # a colliding pair.
        names = instrument_log_names(result.instrument_logs)
        suite["instrument_logs"] = [names[spec] for spec in result.instrument_logs]

    # The log is inlined for failures only, following twister: it makes a red
    # report self-contained. A passing instance keeps its transcript one path
    # away instead, which is what stops a multi-megabyte console from being
    # copied into the report once per instance.
    if result.status in (FAILED, ERROR) and result.log:
        suite["log"] = result.log

    suite["testcases"] = [
        {
            "identifier": _case_identifier(case),
            "status": case.status,
            "reason": case.message,
            "file": case.file,
            "line": case.line,
        }
        for case in result.cases
    ]
    return suite


def _case_identifier(case: CaseResult) -> str:
    """A case's name, which is already the fully-qualified ``suite.case``.

    The bare name is not unique: Unity case names repeat across suites -- both
    cjson and lz4 have a ``boundaries`` -- and one instance emits one row per
    case, so two suites sharing a name collide in the report and in the JUnit
    XML. Qualifying it is what stops a passing ``boundaries`` and a failing one
    from being the same row.

    No qualification happens here because none is needed: the device runs each
    case under ``suite.case`` (``UnityDefaultTestRun``), which is also the
    name it prints on the result line and the filter ``unity_run`` accepts, so
    the name arrives qualified and the report spells it the same way rather
    than inventing a second convention.
    """
    return case.name


def _case_label(case: "CaseResult") -> str:
    """A case on one line: its qualified name and where it failed.

    The suite belongs in the name because a Unity case name repeats across
    suites, and repeating it in the parentheses as well would say the same thing
    twice. What is left there is the location -- ``file:line``, the real source
    file and, for a failure, the assertion's own line, which is ``__LINE__``
    rather than the line the case was declared on.
    """
    name = _case_identifier(case)
    return f"{name} ({case.location})" if case.location else name


def dict_to_xml(
    data: Dict[str, Any],
    *,
    name: str = "sifli-sdk",
    selected: Optional[str] = None,
) -> ElementTree.Element:
    """Project the JSON document into JUnit XML.

    An instance that reported per-case results becomes one ``<testcase>`` per
    case, so a CI system shows the six cases that failed rather than the one
    instance that contained them. An instance with no per-case detail -- the
    console and pytest harnesses, or a failure that happened before the harness
    ran at all -- stays a single ``<testcase>``, which is what it always was.

    The ``tests``/``failures``/``errors``/``skipped`` attributes are counted
    from the elements actually emitted rather than from the instance totals, so
    the header cannot contradict the body.
    """
    suites = data.get("testsuites", [])
    if selected is not None:
        suites = [s for s in suites if s.get("platform") == selected]

    environment = data.get("environment", {})
    totals = {FAILED: 0, ERROR: 0, SKIPPED: 0, PASSED: 0}
    emitted: List[ElementTree.Element] = []

    for suite in suites:
        cases = suite.get("testcases") or []
        status = str(suite.get("status", PASSED))
        if cases:
            for case in cases:
                element = _xml_case(
                    classname=str(suite.get("name", "")),
                    name=str(case.get("identifier", "")),
                    status=str(case.get("status", PASSED)),
                    message=str(case.get("reason", "") or ""),
                    detail=_case_body(case, suite),
                    platform=suite.get("platform"),
                    testbed=suite.get("testbed"),
                )
                emitted.append(element)
                totals[_bucket(str(case.get("status", PASSED)))] += 1
        else:
            suite_log = str(suite.get("log", "") or "")
            element = _xml_case(
                classname=_classname(str(suite.get("name", ""))),
                name=_case_name(str(suite.get("name", ""))),
                status=status,
                message=str(suite.get("reason", "") or ""),
                detail=suite_log,
                log=suite_log,
                platform=suite.get("platform"),
                testbed=suite.get("testbed"),
                time=suite.get("execution_time"),
            )
            emitted.append(element)
            totals[_bucket(status)] += 1

    suite_element = ElementTree.Element(
        "testsuite",
        {
            "name": name,
            "tests": str(len(emitted)),
            "failures": str(totals[FAILED]),
            "errors": str(totals[ERROR]),
            "skipped": str(totals[SKIPPED]),
            "time": f"{float(data.get('summary', {}).get('duration', 0.0)):.3f}",
        },
    )
    started = environment.get("started")
    if started:
        suite_element.set("timestamp", str(started))

    properties = ElementTree.SubElement(suite_element, "properties")
    for status_key, count in (
        (PASSED, totals[PASSED]),
        (SKIPPED, totals[SKIPPED]),
        (FAILED, totals[FAILED]),
        (ERROR, totals[ERROR]),
    ):
        if count:
            ElementTree.SubElement(
                properties,
                "property",
                {"name": f"count.{status_key}", "value": str(count)},
            )

    # Which code produced this. JUnit's own properties block is where a reader
    # looks for run-level facts, and CI already surfaces it -- so a result whose
    # lab was running a different revision can be explained without the machine
    # it came from.
    for name in ("sdk_revision", "sdk_protocol", "agent_revision"):
        value = environment.get(name)
        if value:
            ElementTree.SubElement(
                properties, "property", {"name": name, "value": str(value)}
            )

    for element in emitted:
        suite_element.append(element)

    _indent(suite_element)
    return suite_element


def _bucket(status: str) -> str:
    """JUnit has three failure buckets; anything else counts as a pass."""
    if status in (FAILED, ERROR, SKIPPED):
        return status
    return PASSED


def _classname(instance_id: str) -> str:
    return instance_id.rpartition("/")[0] or "sifli-sdk"


def _case_name(instance_id: str) -> str:
    return instance_id.rpartition("/")[2] or instance_id


def _case_body(case: Dict[str, Any], suite: Dict[str, Any]) -> str:
    """What goes inside a per-case ``<failure>`` / ``<error>``.

    Empty for a case that did not fail: a passing element has nothing to explain,
    and appending the instance's reason to it is how a green case once ended up
    carrying its sibling's failure text.

    For a failing case: where it failed, then the instance-level reason, which
    adds how many of its siblings failed too. The case's own reason is not part
    of the body -- it is already the element's ``message`` attribute, and it was
    the body's first line as well, so every failure said the same sentence twice.
    """
    if str(case.get("status", "")) not in (FAILED, ERROR):
        return ""

    parts = []
    location = str(case.get("file", "") or "")
    if location:
        if case.get("line"):
            location += f":{case['line']}"
        parts.append(location)

    suite_reason = str(suite.get("reason", "") or "")
    if suite_reason:
        parts.append(suite_reason)
    return "\n".join(parts)


def _xml_case(
    *,
    classname: str,
    name: str,
    status: str,
    message: str,
    detail: str = "",
    log: str = "",
    platform: Any = None,
    testbed: Any = None,
    time: Any = None,
) -> ElementTree.Element:
    """One ``<testcase>``.

    ``detail`` is the body of a failure/error and ``log`` the body of a
    ``<system-out>``; they are separate parameters because they answer different
    questions and only the instance-level path has both. Conflating them is how
    a passing per-case element once grew a ``<system-out>`` holding its
    instance's failure reason.
    """
    attributes = {"classname": classname, "name": name}
    if time is not None:
        attributes["time"] = f"{float(time):.3f}"
    if testbed:
        attributes["testbed"] = str(testbed)
    if platform:
        attributes["platform"] = str(platform)

    element = ElementTree.Element("testcase", attributes)
    if status == FAILED:
        child = ElementTree.SubElement(element, "failure", {"message": message})
        child.text = detail or message
    elif status == ERROR:
        child = ElementTree.SubElement(element, "error", {"message": message})
        child.text = detail or message
    elif status == SKIPPED:
        ElementTree.SubElement(element, "skipped", {"message": message})
    elif log:
        # Kept for a pass too: it is what a human needs when a test later turns
        # flaky. Only the instance-level path has a log to offer -- a per-case
        # element has no transcript of its own, and its instance's is in
        # runner.log and device.log.
        ElementTree.SubElement(element, "system-out").text = log
    return element


def write_json(data: Dict[str, Any], path: str) -> str:
    """Write the run document as JSON.

    Takes the document rather than the summary so that a caller producing both
    reports builds it once: the JUnit is then provably a projection of the bytes
    on disk, not a second reading of the same objects.
    """
    import json

    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, ensure_ascii=False)
        handle.write("\n")
    return path


def write_junit_xml(summary: RunSummary, path: str) -> str:
    """Write ``summary`` as JUnit XML and return the path written."""
    return write_junit_from_dict(summary_to_dict(summary), path)


def write_junit_from_dict(data: Dict[str, Any], path: str) -> str:
    """Write the JUnit projection of ``data``."""
    return _write_element(dict_to_xml(data), path)


def _write_element(element: ElementTree.Element, path: str) -> str:
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    ElementTree.ElementTree(element).write(path, encoding="utf-8", xml_declaration=True)
    return path


def format_result(result: TestResult, prefix: str = "") -> str:
    """One test's verdict, as a single line.

    Shared by the end-of-run summary and by the line printed as each test
    finishes, so the two cannot drift into saying the same thing differently.
    """
    target = f" on {result.testbed_id}" if result.testbed_id else ""
    # The stage is shown only when something went wrong: on a pass it is always
    # "run" and would be noise on every line. On a failure it is the first thing
    # worth knowing -- a build that never produced a binary and a test that ran
    # and disagreed are different problems.
    where = f"  at {result.stage}" if result.stage and not result.passed else ""
    return (
        f"{prefix}[{_marker(result.status):5}] {result.id}{target}  "
        f"({result.duration:.1f}s){where}"
    )


def separator(title: str, width: int = 78, fill: str = "-") -> str:
    """A rule with ``title`` in the middle, so a long run stays parseable.

    Plain ASCII on purpose: this is the one piece of output whose whole job is
    to be readable, and a Windows console in a non-UTF-8 code page turns box
    drawing into noise -- which is worse than no separator at all.

    ``fill`` tells the two uses apart at a glance: a single line while the run
    is going, and a heavier one introducing each group in the closing report.
    """
    if len(title) + 4 >= width:
        return f"{fill * 3} {title} {fill * 3}"
    dashes = width - len(title) - 2
    left = dashes // 2
    return f"{fill * left} {title} {fill * (dashes - left)}"


def group_of(result: TestResult) -> str:
    """Which testcase file this result came from.

    An instance id is ``<dir relative to -T>/<tests key>`` (``discovery.py``),
    so dropping the last segment yields the directory -- which holds exactly
    one ``testcase.yaml``, and whose entries therefore share a firmware too.
    """
    return result.id.rsplit("/", 1)[0] if "/" in result.id else ""


def _tally(results: List[TestResult]) -> str:
    """``4 test(s): 2 passed, 1 failed`` -- zeros left out, they say nothing."""
    counted = []
    for status, word in ((PASSED, "passed"), (FAILED, "failed"),
                         (ERROR, "errored"), (SKIPPED, "skipped")):
        count = sum(1 for result in results if result.status == status)
        if count:
            counted.append(f"{count} {word}")
    return f"{len(results)} test(s): " + ", ".join(counted or ["none"])


def format_turned_away(pairs: List[Any], nothing_planned: bool = False) -> str:
    """The (test, platform) pairs a plan leaves out, and why.

    A filtered pair is an answer rather than a failure -- the test is fine, the
    platform is fine, they are not a match -- and it is reported rather than
    dropped, because a case that silently does not run is the worst thing a plan
    can produce.

    Both places that print a plan print this block, one because it says what
    will *not* happen before hardware is touched (:func:`format_manifest`) and
    one because that is all a dry run has to say (``cli.describe_plan``). Shared
    so the two cannot drift into spelling the same refusal two ways -- which is
    exactly how the opening manifest came to account for fewer tests than the
    closing summary did.

    ``pairs`` are objects with ``id``, ``platform`` and ``reason``
    (``runner.FilteredRun``); duck-typed for the same reason the manifest is.
    ``nothing_planned`` changes the header to the one that says the block is the
    whole answer, for a selection every one of whose pairs was refused.
    """
    if nothing_planned:
        lines = [
            f"Nothing to run. All {len(pairs)} (test, platform) pair(s) were "
            f"turned away:"
        ]
    else:
        lines = [f"{len(pairs)} (test, platform) pair(s) turned away:"]
    for pair in pairs:
        lines.append(f"  {pair.id}")
        lines.append(f"    platform:   {pair.platform}")
        lines.append(f"    reason:     {pair.reason}")
    return "\n".join(lines)


def format_manifest(entries: List[tuple],
                    turned_away: Optional[List[Any]] = None) -> str:
    """The list of tests about to run, numbered, one per line, and what was not.

    Printed before anything is built, because the question a long run raises is
    "how many are left", and neither the closing summary nor the per-test lines
    answer it while the run is still going. ``entries`` are
    ``(index, total, id, platform, harness)``.

    ``turned_away`` are the pairs planning refused, appended after the runs. The
    closing summary counts them -- a filtered pair is recorded as a skipped
    result -- so a manifest that named only the runs announced a smaller set
    than the run went on to report.
    """
    lines = [f"[plan]  {entries[0][1]} test(s) to run:"]
    width = max(len(str(entry[2])) for entry in entries)
    for index, total, test_id, platform, harness in entries:
        lines.append(
            f"        {index:>{len(str(total))}}/{total}  "
            f"{str(test_id):<{width}}  ({platform}, {harness})"
        )
    if turned_away:
        lines.append(format_turned_away(list(turned_away)))
    return "\n".join(lines)


def format_summary(summary: RunSummary, inline_logs: bool = False) -> str:
    """Human-readable console summary.

    A failed instance says *which* cases were red, not merely that something
    was: the harnesses that can attribute a failure to a case (unity) hand the
    list over, and the terminal is where a human looks first.

    ``inline_logs`` prints the captured log in place of its path, which is what
    twister's ``--inline-logs`` does. It is off by default because a transcript
    can be megabytes and a path plus a two-line re-run is usually what is
    wanted.
    """
    lines: List[str] = []

    # Grouped by testcase file, in the order the results arrived (dicts keep
    # insertion order). A run over a tree with one test per directory reads the
    # same as before; one with several entries in a testcase.yaml -- the usual
    # shape for a unit-test suite -- now says so instead of listing them flat.
    groups: Dict[str, List[TestResult]] = {}
    for result in summary.results:
        groups.setdefault(group_of(result), []).append(result)

    root = os.path.normpath(summary.root) if summary.root else ""
    root_label = os.path.basename(root)

    for key, results in groups.items():
        if lines:
            lines.append("")
        lines.append(separator(f"{key or root_label}  ({_tally(results)})", fill="="))

        for result in results:
            lines.append(format_result(result))
            # Details stay with their test rather than being collected at the
            # end: on a run with a handful of failures, having to match a line
            # at the bottom back to one at the top is the harder read.
            if result.passed:
                continue

            if result.message:
                lines.append(f"        {result.message}")

            # Only the cases that explain the failure. A skipped case is not
            # one, and listing the passes would bury the two lines that matter.
            for case in result.cases:
                if case.status not in (FAILED, ERROR):
                    continue
                detail = f": {case.message}" if case.message else ""
                lines.append(
                    f"        {_marker(case.status):5} {_case_label(case)}{detail}"
                )

            if inline_logs and result.log:
                for line in result.log.splitlines():
                    lines.append(f"        | {line}")
            elif result.out_dir:
                lines.append(f"        logs: {result.out_dir}")

    lines.append("")
    lines.append(
        f"{summary.total} test(s): {summary.passed} passed, "
        f"{summary.count(FAILED)} failed, {summary.count(ERROR)} errored, "
        f"{summary.skipped} skipped  in {summary.duration:.1f}s"
    )

    root = artifacts_root(summary)
    if root:
        lines.append(f"artifacts: {root}")
    return "\n".join(lines)


def artifacts_root(summary: RunSummary) -> str:
    """The directory a reader should look under, common to every instance.

    One line rather than a path per instance: for a green run that is the whole
    answer to "where are the logs", and for a red one the failing instances have
    already named their own above.
    """
    directories = [result.out_dir for result in summary.results if result.out_dir]
    if not directories:
        return ""
    if len(directories) == 1:
        return directories[0]
    return os.path.commonpath(directories)


def run_log(summary: RunSummary) -> str:
    """The run-level log, written to ``<outdir>/runner.log``.

    The per-instance ``runner.log`` answers "why is this one red". This answers
    "what happened in this run, and what was it run against" -- the question you
    have months later, when the device transcript is still readable but says
    nothing about which root was discovered, what got filtered, or how many
    instances there were.

    Same name at both levels on purpose: each is the runner's record at its own
    scope, told apart by where it sits.
    """
    failures = [r for r in summary.results if r.failed]
    filtered = [r for r in summary.results if _is_filtered(r)]
    skipped = [
        r for r in summary.results
        if r.status == SKIPPED and not _is_filtered(r)
    ]

    lines = [
        f"run:        {_iso(summary.started_at)} -> {_iso(summary.finished_at)}"
        f"  ({summary.duration:.1f}s)",
        f"root:       {summary.root or '(unknown)'}",
        f"outdir:     {summary.out_dir or '(none)'}",
        "",
        f"discovered: {summary.discovered or summary.total}",
        f"ran:        {summary.total}",
        f"result:     {summary.passed} passed, {summary.count(FAILED)} failed, "
        f"{summary.count(ERROR)} errored, {summary.skipped} skipped",
    ]

    if failures:
        lines.append("")
        lines.append("failures:")
        for result in failures:
            where = f" [{result.stage}]" if result.stage else ""
            lines.append(
                f"  {_marker(result.status):5} {result.id}{where}  {result.message}"
            )
    if filtered:
        lines.append("")
        lines.append("not run (filtered):")
        for result in filtered:
            reason = result.message.removeprefix("filtered: ")
            lines.append(f"  {result.platform or '-'}: {result.id}  {reason}")
    if skipped:
        lines.append("")
        lines.append("skipped:")
        for result in skipped:
            lines.append(f"  {result.id}  {result.message}")

    lines.append("")
    lines.append("artifacts:")
    for result in summary.results:
        if result.out_dir:
            lines.append(f"  {result.id}: {result.out_dir}")
            break
    if summary.out_dir:
        lines.append(f"  reports under {summary.out_dir}")
    lines.append("")
    return "\n".join(lines)


def _is_filtered(result: TestResult) -> bool:
    """A filtered pair, which the runner records as a skip with a prefix."""
    return result.status == SKIPPED and result.message.startswith("filtered:")
