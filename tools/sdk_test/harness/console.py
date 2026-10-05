# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""The ``console`` harness: match expected output without device-side test code.

The device prints; the host decides. An example such as ``hello_world`` needs no
test framework at all, only a list of expectations in ``harness_config.record``.

Two properties of this SDK's console shape the matcher and are the reason it
matches over an *accumulated buffer* rather than line by line:

* ``RT_CONSOLEBUF_SIZE`` is 128 bytes, so a long line arrives split across reads;
* there is no global console lock, so unrelated log lines can appear between an
  expectation and its answer.
"""

from __future__ import annotations

import re
import time
from dataclasses import dataclass
from dataclasses import field
from typing import Any
from typing import List
from typing import Optional
from typing import TYPE_CHECKING

from ..errors import HarnessError

if TYPE_CHECKING:  # pragma: no cover - import cycle avoidance for type hints only
    from ..model import TestInstance
    from . import ConsoleLike
    from . import HarnessOutcome


@dataclass
class RecordPattern:
    """One expectation from ``harness_config.record``."""

    regex: str
    ordered: bool = True
    repeat: int = 1
    type: str = "one_line"
    separator: Optional[str] = None

    #: Filled in during matching.
    count: int = 0
    cursor: int = 0

    def describe(self) -> str:
        suffix = f" x{self.repeat}" if self.repeat > 1 else ""
        return f"/{self.regex}/{suffix}"


def parse_record(spec: Any) -> List[RecordPattern]:
    """Normalise ``harness_config.record`` into a list of patterns.

    Accepted, mirroring Twister: a bare regex string, a mapping, or a list of
    either. A plain list of strings means "these, in order".
    """
    if spec is None:
        raise HarnessError(
            'harness "console" needs harness_config.record '
            "(a regex, a mapping, or a list of them)"
        )

    entries = spec if isinstance(spec, (list, tuple)) else [spec]
    if not entries:
        raise HarnessError("harness_config.record is empty")

    patterns: List[RecordPattern] = []
    for index, entry in enumerate(entries):
        patterns.append(_parse_record_entry(entry, index))
    return patterns


def _parse_record_entry(entry: Any, index: int) -> RecordPattern:
    where = f"harness_config.record[{index}]"

    if isinstance(entry, str):
        if not entry:
            raise HarnessError(f"{where}: empty regex")
        pattern = RecordPattern(regex=entry)
        # Validate eagerly here too: the mapping form below already does, and a
        # bad regex should fail while the testcase file is being read rather
        # than when the run starts.
        _compile(pattern, where)
        return pattern

    if not isinstance(entry, dict):
        raise HarnessError(f"{where}: expected a regex string or a mapping")

    regex = entry.get("regex")
    if not regex:
        raise HarnessError(f'{where}: missing "regex"')

    try:
        repeat = int(entry.get("repeat", 1))
    except (TypeError, ValueError) as exc:
        raise HarnessError(f"{where}.repeat must be an integer") from exc
    if repeat < 1:
        raise HarnessError(f"{where}.repeat must be >= 1")

    record_type = str(entry.get("type", "one_line"))
    if record_type not in ("one_line", "multi_line"):
        raise HarnessError(f'{where}.type must be "one_line" or "multi_line"')

    pattern = RecordPattern(
        regex=str(regex),
        ordered=bool(entry.get("ordered", True)),
        repeat=repeat,
        type=record_type,
        separator=entry.get("separator"),
    )
    _compile(pattern, where)
    return pattern


def _compile(pattern: RecordPattern, where: str = "<record>") -> re.Pattern:
    try:
        return re.compile(pattern.regex)
    except re.error as exc:
        raise HarnessError(f'{where}: invalid regex "{pattern.regex}": {exc}') from exc


@dataclass
class MatchState:
    """Outcome of a console match, kept separate so it can be reported."""

    passed: bool
    detail: str
    matched: int = 0
    expected: int = 0
    unmatched: List[str] = field(default_factory=list)


class ConsoleMatcher:
    """Incremental, chunk-tolerant matcher over a console stream.

    Usage: feed whatever arrives (any chunking), then read :meth:`state`. Every
    search runs against the whole accumulated buffer, so an expectation split
    across two reads still matches once the rest arrives; position is carried by
    a cursor instead of by re-reading line by line.

    Ordering follows Twister. Entries are evaluated in declaration order against
    a single shared cursor, so an ``ordered: true`` entry (the default) can only
    match text at or after the previous ordered entry's match -- a boot log whose
    lines arrive shuffled therefore fails rather than passing by accident. An
    ``ordered: false`` entry is counted anywhere in the buffer and does not move
    the cursor, which is how a "this line appears somewhere" assertion is
    written.
    """

    def __init__(self, patterns: List[RecordPattern]) -> None:
        if not patterns:
            raise HarnessError("console harness needs at least one record entry")
        self._patterns = patterns
        self._buffer = ""
        self._compiled: List[re.Pattern] = []
        for index, pattern in enumerate(patterns):
            self._compiled.append(_compile(pattern, f"harness_config.record[{index}]"))

    # -- input ---------------------------------------------------------------

    def feed(self, chunk: str) -> None:
        """Append newly received console text and re-evaluate the expectations."""
        if not chunk:
            return
        self._buffer += chunk
        self._evaluate()

    @property
    def buffer(self) -> str:
        """Everything seen so far, for failure messages."""
        return self._buffer

    @property
    def consumed(self) -> int:
        """How far into the raw buffer matching has progressed (diagnostics).

        Counts only entries that actually advanced the stream -- an
        ``ordered: false`` entry is matched anywhere and consumes nothing.
        """
        cursor = 0
        for pattern in self._patterns:
            if not pattern.ordered or pattern.count < pattern.repeat:
                continue
            cursor = max(cursor, self._buffer_index(pattern, pattern.cursor))
        return cursor

    # -- matching ------------------------------------------------------------

    def _view(self, pattern: RecordPattern) -> tuple:
        """The text a pattern is matched against, plus a view->buffer index map.

        ``multi_line`` collapses line breaks into ``separator`` (default a single
        space) so a regex can span lines without the author writing ``\\s+``.
        Collapsing shifts every offset that follows it, so the mapping back to
        raw-buffer positions is returned alongside; ordering is expressed in
        buffer positions and must survive a mix of the two types.
        """
        if pattern.type != "multi_line":
            return self._buffer, None  # None means "identity mapping"

        separator = " " if pattern.separator is None else pattern.separator
        buffer = self._buffer
        out: List[str] = []
        mapping: List[int] = []
        index = 0
        length = len(buffer)
        while index < length:
            char = buffer[index]
            if char == "\n":
                # Matches the `\r?\n` -> separator substitution exactly.
                out.append(separator)
                mapping.append(index)
                index += 1
            elif char == "\r" and index + 1 < length and buffer[index + 1] == "\n":
                out.append(separator)
                mapping.append(index)
                index += 2
            else:
                out.append(char)
                mapping.append(index)
                index += 1
        mapping.append(length)  # sentinel for "one past the end"
        return "".join(out), mapping

    def _buffer_index(self, pattern: RecordPattern, view_index: int) -> int:
        """Translate a position in ``pattern``'s view back to a buffer offset."""
        if pattern.type != "multi_line":
            return view_index
        _, mapping = self._view(pattern)
        if view_index >= len(mapping):
            return len(self._buffer)
        return mapping[view_index]

    def _evaluate(self) -> None:
        """Re-match every entry against the buffer, from scratch.

        Recomputing rather than accumulating is what lets the cursor be shared:
        an entry that has not matched yet simply holds the cursor still for the
        entries after it, and a later chunk can still complete it.
        """
        cursor = 0  # shared stream position, in raw-buffer offsets

        for pattern, compiled in zip(self._patterns, self._compiled):
            view, _ = self._view(pattern)

            if not pattern.ordered:
                # Counted anywhere; deliberately does not move the cursor.
                found = sum(1 for _ in compiled.finditer(view))
                pattern.count = min(found, pattern.repeat)
                pattern.cursor = 0
                continue

            start = self._view_index(pattern, cursor)
            pattern.cursor = start
            pattern.count = 0
            while pattern.count < pattern.repeat:
                match = compiled.search(view, pattern.cursor)
                if match is None:
                    break
                pattern.count += 1
                # An empty match would not advance the cursor; nudge past it.
                pattern.cursor = (
                    match.end() if match.end() > match.start() else match.end() + 1
                )

            if pattern.count < pattern.repeat:
                # Do not let a later ordered entry search past text this one is
                # still waiting for.
                break
            cursor = self._buffer_index(pattern, pattern.cursor)

    def _view_index(self, pattern: RecordPattern, buffer_index: int) -> int:
        """Translate a raw-buffer offset into ``pattern``'s view (inverse of
        :meth:`_buffer_index`)."""
        if pattern.type != "multi_line" or buffer_index <= 0:
            return max(0, buffer_index)
        _, mapping = self._view(pattern)
        for position, source in enumerate(mapping):
            if source >= buffer_index:
                return position
        return len(mapping) - 1


    # -- result --------------------------------------------------------------

    @property
    def complete(self) -> bool:
        return all(pattern.count >= pattern.repeat for pattern in self._patterns)

    @property
    def expected_total(self) -> int:
        return sum(pattern.repeat for pattern in self._patterns)

    @property
    def matched_total(self) -> int:
        return sum(min(pattern.count, pattern.repeat) for pattern in self._patterns)

    def state(self) -> MatchState:
        unmatched = [
            # Show "have/need" as soon as there is partial progress: a loop that
            # ran once instead of three times is the common mistake here.
            f"{pattern.describe()} ({pattern.count}/{pattern.repeat})"
            if pattern.count
            else pattern.describe()
            for pattern in self._patterns
            if pattern.count < pattern.repeat
        ]
        if not unmatched:
            return MatchState(
                passed=True,
                detail=f"all {self.expected_total} console expectation(s) matched",
                matched=self.matched_total,
                expected=self.expected_total,
            )
        return MatchState(
            passed=False,
            detail="missing console expectation(s): " + ", ".join(unmatched),
            matched=self.matched_total,
            expected=self.expected_total,
            unmatched=unmatched,
        )


def run(
    instance: "TestInstance",
    console: "ConsoleLike",
    timeout: float,
    poll: float = 0.5,
) -> "HarnessOutcome":
    """Watch the console until every expectation matches, or time runs out.

    The device has already been reset and its console opened before this is
    called, so the buffer starts at the boot banner. Nothing is sent to the
    device: the ``console`` harness exists for images that are their own test.
    """
    from . import HarnessOutcome
    from . import reject_foreign_keys

    reject_foreign_keys("console", instance.harness_config)
    matcher = ConsoleMatcher(parse_record(instance.harness_config.get("record")))

    deadline = time.monotonic() + max(0.0, float(timeout))
    while not matcher.complete:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        chunk = console.read(min(poll, remaining))
        if chunk:
            matcher.feed(chunk)

    state = matcher.state()
    return HarnessOutcome(
        passed=state.passed,
        detail=state.detail,
        log=matcher.buffer,
    )
