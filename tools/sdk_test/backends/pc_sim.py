# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tier B carrier: the ``pc`` board's simulator.

Stubbed on purpose. The interface is the part that matters, and it is shaped by
one fact: **this backend needs no Lab Agent.** There is no hardware to allocate,
so there is no testbed, no lock, and no release. That asymmetry is why the
backend split exists at all rather than the runner talking to the agent directly.

What it will do when implemented: build with ``scons --board=pc_hcpu``, then run
the produced Windows executable locally. The simulator is the natural home for
mock-based unit tests (requirement 3), because ``drivers/SConscript`` skips the
entire ``drivers/`` tree under ``BSP_USING_PC_SIMULATOR`` and substitutes
``rtos/rtthread/bsp/sifli/drivers_pc/`` -- which is exactly the "do not compile
``drivers/hal``" precondition a CMock-generated mock needs in order to be the
only definition of the symbol it replaces.

Known constraints to carry into the implementation: the simulator needs MSVC, so
it cannot run on a Linux CI runner; and the console is reached through the
simulator's own UART plumbing rather than a serial port, so the ``console``
harness must read it differently from the board case.
"""

from __future__ import annotations

from typing import Optional

from ..errors import BackendError
from ..model import TestInstance
from .base import Backend
from .base import unimplemented

_WHY = (
    "See tools/sdk_test/DESIGN.md, 'requirement 3': the simulator "
    "is the carrier for mock-based unit tests, where mock_*.c replaces the HAL "
    "symbol under test and drivers/hal is not compiled at all."
)


class PcSimBackend(Backend):
    """Runs tests against ``customer/boards/pc`` on this machine."""

    name = "pc_sim"
    requires_lab_agent = False

    def open(
        self,
        instance: TestInstance,
        platform: str,
        build_dir: Optional[str] = None,
        holder: Optional[str] = None,
    ):
        raise unimplemented(self.name, platform, _WHY)
