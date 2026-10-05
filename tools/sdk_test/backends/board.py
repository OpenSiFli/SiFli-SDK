# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tier A: a real board, reached through a Lab Agent.

The ordering in :meth:`BoardBackend.open` is the part worth reading, because
each step exists to prevent a specific and quiet failure:

1. **allocate** -- one test program per testbed, so two runs cannot interleave.
2. **flash** -- with the console still *closed*. For a board flashed over UART
   the console port *is* the download port, so the two cannot be open at once.
3. **open the console** -- before the reset, not after.
4. **reset** -- last, so the boot log lands in an already-open console's buffer.

If step 3 came after step 4, the earliest lines of the boot log would race the
serial port being opened, and a ``console`` harness asserting on the banner would
fail intermittently for a reason that has nothing to do with the firmware. That
race is the classic reason an embedded test suite is called flaky.
"""

from __future__ import annotations

from typing import Optional

from ..agent_client import LabAgentClient
from ..agent_client import RemoteConsole
from ..agent_client import require_testbed
from ..errors import BackendError
from ..model import TestInstance
from .base import Backend
from .base import Session


class BoardSession:
    """A board held for the duration of one test."""

    def __init__(
        self,
        client: LabAgentClient,
        testbed_id: str,
        console: RemoteConsole,
        platform: str,
        holder: Optional[str],
        manual_reset: bool = False,
    ) -> None:
        self._client = client
        self.testbed_id = testbed_id
        self.console = console
        self.platform = platform
        self.holder = holder
        #: This board has no probe: its reset button is the operator's job.
        self.manual_reset = manual_reset
        self._closed = False
        self._transcript: Optional[str] = None
        self._dropped = 0

    @property
    def transcript(self) -> str:
        """Everything the device said, as far as the agent still retains it.

        Read once and cached, because the agent drops the whole console stream
        on ``console_close`` -- so this is only answerable while the session is
        open, and the runner asks for it before closing.

        The read uses an explicit cursor of 0 and so does not disturb the cursor
        the harness has been reading through. The agent answers it immediately:
        ``ConsoleStream.read_from`` returns the whole retained buffer as soon as
        anything is buffered, without waiting out the timeout.

        The trimmed-byte count is derived rather than reported -- the agent's
        read response carries no equivalent of ``_dropped``, but reading from 0
        returns the entire retained buffer alongside the absolute end cursor, so
        the difference is exactly what the cap discarded.
        """
        if self._transcript is None:
            try:
                data, end = self.console.read_from(0, 0.0)
            except Exception:
                # No transcript is a degraded result, not a failed test: the
                # harness's own verdict still stands.
                data, end = "", 0
            self._transcript = data
            self._dropped = max(0, int(end) - len(data))
        return self._transcript

    @property
    def dropped(self) -> int:
        """Bytes the agent's buffer cap trimmed off the head; see transcript."""
        if self._transcript is None:
            _ = self.transcript
        return self._dropped

    def close(self) -> None:
        """Close the console and give the testbed back.

        Both steps are attempted even if the first fails: a stuck console must
        not also cost the lab a board.
        """
        if self._closed:
            return
        self._closed = True

        try:
            self._client.console_close(self.testbed_id)
        except Exception:
            # Best effort -- `release` below tears the console down anyway.
            pass

        self._client.release(self.testbed_id, self.holder)


class BoardBackend(Backend):
    """Runs tests on hardware allocated from a Lab Agent."""

    name = "board"
    requires_lab_agent = True

    def __init__(self, client: LabAgentClient) -> None:
        self.client = client

    def open(
        self,
        instance: TestInstance,
        platform: str,
        build_dir: Optional[str] = None,
        holder: Optional[str] = None,
        will_confirm_boot: bool = False,
    ) -> BoardSession:
        fixtures = [fixture.spec for fixture in instance.fixtures]

        allocation = self.client.allocate(
            platform=platform,
            fixtures=fixtures,
            testbed_id=None,
            holder=holder,
        )
        testbed_id = require_testbed(allocation)
        resolved_platform = str(allocation.get("platform") or platform)
        # How the board will be booted, as the agent reported it while handing
        # the testbed over. Read here rather than after the reset, because the
        # one thing that makes a manual reset unusable is a property of the
        # testcase -- known now, and expensive to discover after a flash.
        #
        # Only ``manual`` reaches the gate below. ``auto`` is the same probe-less
        # bench carrying the opposite promise -- its boot is the download's --
        # so it needs no gate and no prompt, and asking the caller to vouch for
        # it per run was always the wrong place for a fact the lab's own config
        # already states.
        manual_reset = str(allocation.get("reset", "jlink")).lower() == "manual"
        # A harness that writes before it reads is the one a hand-booted board
        # cannot be gated by. `autorun` takes unity out of that set: the firmware
        # reports by itself, so there is no command to lose.
        autorun = bool(instance.harness_config.get("autorun"))

        try:
            if (
                manual_reset
                and instance.harness == "unity"
                and not autorun
                and not will_confirm_boot
            ):
                # Named explicitly rather than "anything but console": the
                # unity harness writes its run command as its very first
                # action, by construction. With something to gate the boot --
                # an operator pressing Enter once they have seen it come up --
                # that stops being a race and unity is as usable here as
                # anywhere. Without a gate it cannot be made to work, and the
                # failure it produces ("no Unity summary line found") reads
                # like a firmware fault. `autorun` removes the premise rather
                # than gating it: nothing is sent, so nothing can be lost.
                #
                # So there are two exits where the caller stands -- an operator
                # who will press the button, or the firmware reporting by itself
                # -- and one where the lab owner does: declaring the bench
                # `auto`, which says the download's own boot is reliable. That
                # last one is a fact about their wiring, and it is the same
                # every run, which is why it is not an argument to this call.
                raise BackendError(
                    f'"{instance.id}": testbed {testbed_id} is booted by hand, '
                    "and the unity harness sends its run command before it "
                    "reads anything. That command does not survive the boot "
                    "which follows, so the run would burn its entire timeout "
                    "and then report a missing Unity summary, which reads as a "
                    "firmware fault rather than this. Either set "
                    "harness_config.autorun, so the firmware reports by itself "
                    "and there is no command to lose, or run it from a terminal "
                    "where the runner stops and an operator confirms the boot, "
                    "or declare this testbed's reset method as 'auto' in the "
                    "lab's runner config, if the download's own boot is what "
                    "resets it. The console harness, and any pytest suite that "
                    "reads before it writes, need none of the three."
                )

            console = self.client.console_for(testbed_id)

            # Fixtures and flash, but explicitly NOT the reset: the console has
            # to be open first (see the module docstring).
            self.client.prepare(testbed_id, build_dir=build_dir, reset=False)
            self.client.console_open(testbed_id)
            self.client.reset(testbed_id)
        except BackendError:
            # Already says what is wrong; just give the board back untouched.
            try:
                self.client.release(testbed_id, holder)
            except Exception:
                pass
            raise
        except Exception as exc:
            # Never leave a testbed allocated behind a failed preparation.
            try:
                self.client.release(testbed_id, holder)
            except Exception:
                pass
            raise BackendError(
                f"Preparing testbed {testbed_id} failed: {exc}"
            ) from exc

        return BoardSession(
            client=self.client,
            testbed_id=testbed_id,
            console=console,
            platform=resolved_platform,
            holder=holder,
            manual_reset=manual_reset,
        )
