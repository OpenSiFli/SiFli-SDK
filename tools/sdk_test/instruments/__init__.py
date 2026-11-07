# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Instruments the pytest harness can drive.

An *instrument* is a second board declared in the lab binding as a fixture
connection with its own console. It is not a testbed: nothing is built or
flashed for it, and its firmware is fixed by the lab. This package holds the
host side -- the line protocol shared by every instrument and the BLE peer
built on it.

The protocol is deliberately independent of any one firmware image: the old
``sdk_test.ble_peer`` parsed ``diss`` log text and therefore knew the example it
was talking to. Here the firmware answers ``@ok``/``@err``/``@evt`` lines, and
nothing above :class:`~sdk_test.instruments.console.InstrumentConsole` reads
device logs.
"""

from .bt_peer import AdvReport
from .bt_peer import BleInstrument
from .bt_peer import BtPeer
from .bt_peer import Characteristic
from .bt_peer import Link
from .bt_peer import Service
from .classic import BtClassicInstrument
from .classic import ClassicDevice
from .classic import ClassicLink
from .console import InstrumentConsole
from .errors import InstrumentCapabilityError
from .errors import InstrumentError
from .errors import InstrumentProtocolError
from .errors import InstrumentTimeout
from .registry import INSTRUMENTS
from .registry import register_instrument

__all__ = [
    "AdvReport",
    "BleInstrument",
    "BtClassicInstrument",
    "BtPeer",
    "Characteristic",
    "ClassicDevice",
    "ClassicLink",
    "INSTRUMENTS",
    "InstrumentCapabilityError",
    "InstrumentConsole",
    "InstrumentError",
    "InstrumentProtocolError",
    "InstrumentTimeout",
    "Link",
    "Service",
    "register_instrument",
]
