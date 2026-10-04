# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""The backend contract.

A backend's whole job is to turn a *plan* (this test, on this platform, built
here) into a *session*: a running device with an open console, plus the
obligation to give it back. Everything above this line -- harnesses, reporting,
the CLI -- is written against :class:`Session` and never learns whether the
device is on a desk or simulated on this machine.
"""

from __future__ import annotations

from typing import Optional
from typing import Protocol
from typing import runtime_checkable

from ..errors import BackendError
from ..harness import ConsoleLike
from ..model import TestInstance


@runtime_checkable
class Session(Protocol):
    """A prepared target with a console, and the duty to release it."""

    #: The console the harness reads the device through.
    console: ConsoleLike
    #: The testbed this session holds, when hardware is involved.
    testbed_id: Optional[str]
    #: The platform actually used (a backend may resolve a default).
    platform: str
    #: Whether this target is booted by something other than a probe.
    #:
    #: True only for a board the lab cannot reset itself, which the agent
    #: reports when it hands the testbed over. The runner waits for the boot log
    #: and offers an operator a fallback, rather than resetting anything itself
    #: -- and the boot it captures usually came from the download, not from the
    #: operator, so a pass says nothing about whether anyone pressed anything.
    manual_reset: bool
    #: Everything the device said, as far as the backend still retains it.
    #:
    #: Only valid before :meth:`close` -- closing tears the console down and
    #: takes the buffer with it, so the runner reads this while the session is
    #: still open. May be empty when the transcript was never obtainable.
    transcript: str
    #: Bytes the backend's buffer cap trimmed off the head of that transcript,
    #: so an artefact can say whether it is complete.
    dropped: int

    def close(self) -> None:
        """Release everything. Must be safe to call twice, and after a failure."""
        ...


class Backend:
    """Base class for execution backends."""

    #: Registry key.
    name = "base"
    #: Whether hardware must be allocated through a Lab Agent.
    requires_lab_agent = False

    def open(
        self,
        instance: TestInstance,
        platform: str,
        build_dir: Optional[str] = None,
        holder: Optional[str] = None,
        confirm_reset: bool = False,
    ) -> Session:
        """Prepare the target and return a session.

        ``confirm_reset`` says whether this run will stop and ask an operator to
        confirm the board has booted before the harness speaks. It matters only
        for a board nothing can reset: without the stop, a harness that writes
        to the device is racing a boot that may wipe its command.

        Implementations must leave nothing allocated when they raise: a failed
        preparation that still holds a testbed would take the board out of the
        pool for the rest of the run.
        """
        raise NotImplementedError

    def close(self, session: Optional[Session]) -> None:
        """Release a session, tolerating ``None`` and repeated calls."""
        if session is None:
            return
        session.close()

    def describe(self) -> str:
        return self.name


def unimplemented(backend: str, platform: str, why: str) -> BackendError:
    """Build the error a stubbed backend raises."""
    return BackendError(
        f'The "{backend}" backend (platform "{platform}") is designed but not '
        f"implemented in this milestone. {why}"
    )
