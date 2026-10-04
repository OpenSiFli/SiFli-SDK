# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Out-of-band power and mux control -- interface only.

The design requires that power switching, board reset and mux selection never be
driven through the DUT's own IO: if the board is wedged or the firmware under
test owns the pin, a self-driven reset cannot recover it. Everything that
touches the physical world out-of-band therefore lives behind this interface,
which the Lab Agent calls and test cases only ever see indirectly.

Nothing is implemented yet. The relay hardware is driven by ``SwitchReset.exe``,
which is not in this repository -- it lives on the Jenkins host -- so there is no
in-tree implementation to wrap. Rather than guess at its command line or
silently pretend, every method raises :class:`RelayNotAvailable`. A test that
needs a power cycle must fail loudly today and pass once the hardware team
delivers the driver, never quietly skip the step.
"""

from __future__ import annotations

from typing import List
from typing import Optional

from ..errors import RelayNotAvailable

_UNAVAILABLE = (
    "Out-of-band relay control is not available: the relay driver "
    "(SwitchReset.exe) is not part of this repository and no runner-local "
    "implementation has been configured. Reset via the debug probe "
    "(JLinkTransport.reset) is the supported path in this milestone."
)


class RelayTransport:
    """Power and mux control for one relay/mux unit.

    The methods below are the contract a future implementation must satisfy;
    their signatures are deliberately the ones test fixtures will call, so
    adding a real driver does not change any caller.
    """

    def __init__(self, mux_id: str, channels: int = 0, port: Optional[str] = None) -> None:
        self.mux_id = mux_id
        self.channels = channels
        self.port = port

    @property
    def available(self) -> bool:
        """Always ``False`` until a driver is configured."""
        return False

    def power_on(self) -> None:
        raise RelayNotAvailable(_UNAVAILABLE)

    def power_off(self) -> None:
        raise RelayNotAvailable(_UNAVAILABLE)

    def power_cycle(self, off_seconds: float = 1.0) -> None:
        """Cut power, wait, restore it."""
        raise RelayNotAvailable(_UNAVAILABLE)

    def mux_select(self, channel: int) -> None:
        raise RelayNotAvailable(_UNAVAILABLE)

    def mux_release(self) -> None:
        raise RelayNotAvailable(_UNAVAILABLE)

    def channels_available(self) -> List[int]:
        """Which mux channels a future driver could select."""
        return list(range(self.channels))
