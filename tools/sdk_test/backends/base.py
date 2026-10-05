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
    #: Whether nothing guarantees this target boots on its own.
    #:
    #: True only for a board whose reset mode is ``manual``: the lab cannot
    #: reset it and nothing else promises the boot either, so an operator may
    #: have to press the button. The runner waits for the boot log and offers
    #: that operator a fallback, rather than resetting anything itself -- and
    #: the boot it captures usually came from the download, not from the
    #: operator, so a pass says nothing about whether anyone pressed anything.
    #:
    #: ``auto`` is deliberately not this. That bench is just as probe-less, but
    #: its boot is the download's and therefore guaranteed, which is the whole
    #: difference between the two modes.
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
        will_confirm_boot: bool = False,
    ) -> Session:
        """Prepare the target and return a session.

        ``will_confirm_boot`` says this run will stop and let an operator vouch
        that the board has booted before the harness speaks. It matters only on
        a ``manual`` bench, and there it is the whole boot: nothing else resets
        the board, so the press the operator is asked for *is* the reset. That
        is what lets a harness which writes to the device run on one -- by the
        time it speaks, the boot is behind it.

        It is not the removed ``--confirm-reset`` under another name. That was a
        caller declaring the boot already arranged, which is a claim about the
        bench and now lives in the bench's own config as ``reset.method: auto``.
        This is an operator performing the reset.

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
