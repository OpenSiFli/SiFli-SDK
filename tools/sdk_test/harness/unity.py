# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""The ``unity`` harness: parse Unity's own output, read off the console.

Unity (ThrowTheSwitch) is the C unit-test framework this framework introduces.
It runs *on the device*; the host only sends the start command and interprets
what comes back. The output contract relied on here is Unity's, not this SDK's:

* per test, ``<file>:<line>:<test name>:PASS`` (or ``:FAIL:<message>``)
* a summary, ``<n> Tests <n> Failures <n> Ignored``
* a verdict line, ``OK`` or ``FAIL``

Two rules exist purely to stop a vacuous pass, which is the failure mode a
console-parsing harness is most prone to: a summary line must have been seen at
all, and it must report at least one test.
"""

from __future__ import annotations

import glob
import json
import os
import re
import time
from dataclasses import dataclass
from dataclasses import field
from typing import List
from typing import Optional
from typing import TYPE_CHECKING

from ..errors import HarnessError

if TYPE_CHECKING:  # pragma: no cover - type hints only
    from ..model import TestInstance
    from . import ConsoleLike
    from . import HarnessOutcome

#: ``test/test_lz4.c:42:test_roundtrip:PASS[:message]``
#:
#: Tolerates a CRLF line ending. On this SDK's console that is the *normal*
#: form: ``rt_kprintf`` routes through ``_serial_poll_tx``, which expands every
#: ``\n`` into ``\r\n``. Without the ``\r?`` a passing case is silently dropped
#: -- only failing lines would survive, because the lazy ``.*?`` message group
#: happily swallows a stray ``\r``. That failure mode is invisible: the run
#: reports "no Unity summary line" or a wrong test count, never a parse error.
CASE_RE = re.compile(
    r"^(?P<file>.+?):(?P<line>\d+):(?P<name>[^:]+):"
    r"(?P<status>PASS|FAIL|IGNORE|IGNORED)(?::(?P<message>.*?))?\r?$",
    re.MULTILINE,
)

#: ``5 Tests 0 Failures 0 Ignored`` — Unity pads numbers, hence the ``\s*``.
SUMMARY_RE = re.compile(
    r"^\s*(?P<tests>\d+)\s+Tests\s+(?P<failures>\d+)\s+Failures\s+"
    r"(?P<ignored>\d+)\s+Ignored\s*$",
    re.MULTILINE,
)

#: Unity's final ``OK`` / ``FAIL`` line.
VERDICT_RE = re.compile(r"^\s*(?P<verdict>OK|FAIL)\s*$", re.MULTILINE)

#: Device-side shell command that starts a run, mirroring ``utest_run``.
DEFAULT_CMD = "unity_run"

#: The prompt a board shows when its shell is ready to take a command.
#:
#: ``unity_run`` *is* an msh command, so waiting for the msh prompt assumes
#: nothing the command did not already assume -- it just stops the harness from
#: addressing a shell that is not there yet.
DEFAULT_PROMPT = r"msh />"

#: The manifest ``tools/build/test_registry.py`` writes beside the registry it
#: generates, listing every suite the build registered.
MANIFEST_NAME = "unity_suites_gen.json"

#: Core suffixes a board name may carry, so that a manifest written for
#: ``--board=qemu_sf32lb52x`` matches a plan for ``qemu_sf32lb52x_hcpu``.
_CORE_SUFFIXES = ("_hcpu", "_lcpu", "_acpu")


def _without_core(name: str) -> str:
    for suffix in _CORE_SUFFIXES:
        if name.endswith(suffix):
            return name[:-len(suffix)]
    return name


def registered_suites(build_dir: str, board: str) -> Optional[List[str]]:
    """The suites the build in ``build_dir`` registered, or ``None`` if unknown.

    ``None`` means there is nothing to check against -- the project has not been
    built, or the only manifest in there belongs to another board, which says
    nothing about this one. Anything else is an answer, and an empty list is one
    too: a firmware that registered no suite at all is a fact worth having.

    The registry generator writes this file next to the registry, inside the
    build directory, and the search is by name rather than by path because the
    variant directory's layout belongs to the project, not to this framework.
    """
    if not build_dir or not os.path.isdir(build_dir):
        return None

    wanted = _without_core(board)
    for path in sorted(glob.glob(os.path.join(build_dir, "**", MANIFEST_NAME),
                                 recursive=True)):
        try:
            with open(path, "r", encoding="utf-8") as handle:
                manifest = json.load(handle)
        except (OSError, ValueError):
            continue
        if not isinstance(manifest, dict):
            continue
        if _without_core(str(manifest.get("board") or "")) == wanted:
            return sorted((manifest.get("suites") or {}).keys())
    return None


@dataclass
class UnityCase:
    name: str
    status: str
    file: str = ""
    line: int = 0
    message: str = ""

    @property
    def passed(self) -> bool:
        return self.status == "PASS"

    @property
    def ignored(self) -> bool:
        return self.status in ("IGNORE", "IGNORED")


@dataclass
class UnityResult:
    """Outcome of one Unity run, as read from the console."""

    passed: bool
    detail: str
    tests: int = 0
    failures: int = 0
    ignored: int = 0
    cases: List[UnityCase] = field(default_factory=list)
    summary_seen: bool = False
    verdict_seen: Optional[str] = None

    @property
    def failed_cases(self) -> List[UnityCase]:
        return [case for case in self.cases if case.status == "FAIL"]


def parse_unity_output(text: str) -> UnityResult:
    """Interpret console text produced by a Unity test binary."""
    cases = [
        UnityCase(
            name=match.group("name"),
            status=match.group("status"),
            file=match.group("file"),
            line=int(match.group("line")),
            message=(match.group("message") or "").strip(),
        )
        for match in CASE_RE.finditer(text)
    ]

    summary = None
    for match in SUMMARY_RE.finditer(text):
        summary = match  # last one wins: a run may be repeated in one session

    verdicts = [match.group("verdict") for match in VERDICT_RE.finditer(text)]
    verdict = verdicts[-1] if verdicts else None

    if summary is None:
        return UnityResult(
            passed=False,
            detail=(
                "no Unity summary line found "
                '(expected "<n> Tests <n> Failures <n> Ignored")'
            ),
            cases=cases,
            verdict_seen=verdict,
        )

    tests = int(summary.group("tests"))
    failures = int(summary.group("failures"))
    ignored = int(summary.group("ignored"))

    result = UnityResult(
        passed=False,
        detail="",
        tests=tests,
        failures=failures,
        ignored=ignored,
        cases=cases,
        summary_seen=True,
        verdict_seen=verdict,
    )

    if tests == 0:
        # A binary that linked Unity but registered no test would otherwise look
        # like a clean pass. Refuse it.
        result.detail = (
            "Unity reported 0 tests: the run is vacuous "
            "(no test registered, or the suite filter matched nothing)"
        )
        return result

    if failures > 0:
        names = ", ".join(case.name for case in result.failed_cases) or "unknown"
        result.detail = (
            f"{failures} of {tests} Unity test(s) failed: {names}"
        )
        return result

    if verdict == "FAIL":
        # Failures counter says zero yet Unity printed FAIL: trust the harsher
        # signal rather than the number.
        result.detail = "Unity printed FAIL although the failure count is 0"
        return result

    result.passed = True
    detail = f"{tests} Unity test(s) passed"
    if ignored:
        detail += f", {ignored} ignored"
    result.detail = detail
    return result


def start_command(harness_config: dict) -> str:
    """Build the device-side command that triggers the run.

    ``harness_config.cmd`` overrides the default; ``harness_config.suite`` is
    appended as the suite/name filter, so a testcase can run one suite out of a
    binary that carries several.
    """
    command = str(harness_config.get("cmd") or DEFAULT_CMD)
    suite = harness_config.get("suite")
    if suite:
        return f"{command} {suite}"
    return command


def is_complete(text: str) -> bool:
    """Cheap check used to stop reading the console as soon as Unity is done."""
    return SUMMARY_RE.search(text) is not None


def require_record_free(harness_config: dict) -> None:
    """``unity`` derives its verdict from Unity, so ``record`` is not used here."""
    if harness_config.get("record"):
        raise HarnessError(
            'harness "unity" takes its verdict from Unity output, so '
            "harness_config.record is not supported; use harness \"console\" "
            "if you want to match console text instead"
        )


def run(
    instance: "TestInstance",
    console: "ConsoleLike",
    timeout: float,
    poll: float = 0.5,
) -> "HarnessOutcome":
    """Ask the device to run Unity, then read its verdict off the console.

    Output already buffered is discarded first. Without that, a session that
    happens to end with a completed earlier run would satisfy
    :func:`is_complete` before the device had done anything, and the harness
    would report the previous verdict — the most plausible way for this harness
    to produce a confident wrong answer.
    """
    from . import HarnessOutcome

    require_record_free(instance.harness_config)
    command = start_command(instance.harness_config)

    # Wait for a shell before addressing one. The console is opened as soon as
    # the download that booted the board has finished writing, and on a board
    # nothing can reset -- no probe, so the boot is the download's and nobody
    # waits for it -- that means the console opens *while the board is still
    # starting*. A command sent into a booting board is simply lost, and the
    # run then reports "no Unity summary line found", which points at the
    # firmware instead of at the timing.
    #
    # Not finding the prompt is not itself a failure: the command goes out
    # anyway and the existing checks judge the run. Inventing a new failure
    # here would break firmware whose prompt this default does not match, and
    # those runs already report something true.
    prompt = str(instance.harness_config.get("prompt") or DEFAULT_PROMPT)
    _wait_for_prompt(console, prompt, timeout, poll)

    _discard_pending(console)
    console.write(command + "\r\n")

    text = ""
    deadline = time.monotonic() + max(0.0, float(timeout))
    while not is_complete(text):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        chunk = console.read(min(poll, remaining))
        if chunk:
            text += chunk

    result = parse_unity_output(text)
    return HarnessOutcome(
        passed=result.passed,
        detail=result.detail,
        log=text,
        cases=result.cases,
    )


def _wait_for_prompt(
    console: "ConsoleLike",
    pattern: str,
    timeout: float,
    poll: float,
) -> bool:
    """Wait until the device shows a shell prompt. True if it did.

    Reads rather than peeks, which is safe here and not in general: whatever
    arrives before the prompt is the boot log, and the caller discards it a
    line later anyway. Waiting for *silence* instead would not work -- a board
    that has not started booting is silent too, and that is precisely the state
    this exists to look past.
    """
    try:
        compiled = re.compile(pattern)
    except re.error:
        # A pattern nobody can match is not worth hanging over; the run's own
        # timeout still bounds everything that follows.
        return False

    text = ""
    deadline = time.monotonic() + max(0.0, float(timeout))
    while True:
        if compiled.search(text):
            return True
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        chunk = console.read(min(poll, remaining))
        if chunk:
            text += chunk


def _discard_pending(console: "ConsoleLike") -> None:
    """Throw away whatever the device has already said.

    ``skip_to_end`` is the efficient path when the console offers it (it can
    advance without transferring the bytes); otherwise a zero-timeout read is
    enough because the cursor advances past whatever it returns.
    """
    skip = getattr(console, "skip_to_end", None)
    if callable(skip):
        skip()
        return
    console.read(0.0)
