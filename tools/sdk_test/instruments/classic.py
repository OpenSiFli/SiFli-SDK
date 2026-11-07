# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Classic Bluetooth half of the ``bt_peer`` instrument.

The M1 firmware does not include the Classic stack yet. This module is the host
contract that the ``bt_peer:ble,classic`` capability promises, so a dual-mode
suite can be written against the same shape the BLE half uses:

* one :class:`~sdk_test.instruments.console.InstrumentConsole`, one console
  stream -- ``bt_peer:ble,classic`` is a capability *set*, not two fixtures;
* operations are namespaced ``btc.*`` and multiplexed by request id / event
  sequence, exactly like ``gap.*`` / ``gatt.*``.

``BtPeer.classic`` only hands this class out when ``@info caps`` contains
``classic``. A BLE-only image therefore fails at ``bt_peer.classic`` with a
sentence, not at the first ``btc.*`` command. The firmware side is expected to
answer ``@err <id> unsupported <op>`` for anything it has not implemented, which
surfaces as a normal :class:`InstrumentProtocolError`.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any
from typing import Dict
from typing import List
from typing import Optional

from ._values import int_value as _int
from ._values import payload as _payload
from .console import InstrumentConsole
from .errors import InstrumentProtocolError


@dataclass(frozen=True)
class ClassicDevice:
    """One device returned by ``btc.inquiry``."""

    address: str
    name: str = ""
    rssi: Optional[int] = None


@dataclass(frozen=True)
class ClassicLink:
    """One Classic connection the instrument holds."""

    link_id: int
    address: str
    channel: int = 1


class BtClassicInstrument:
    """Classic operations over the same line protocol as the BLE half.

    Only the first contract is defined here: inquiry, connect/disconnect and
    SPP data. Profiles (A2DP, HFP, AVRCP) are added as their firmware modules
    land; keeping them out until then means a suite cannot accidentally depend
    on an operation the lab cannot deliver.
    """

    def __init__(self, console: InstrumentConsole) -> None:
        self._console = console

    # -- discovery -----------------------------------------------------------

    def inquiry(self, duration_ms: int = 10000, *, timeout: float = 20.0) -> List[ClassicDevice]:
        # Discard events already in flight before taking the boundary; see
        # BleInstrument.scan() for the stale-event failure this prevents.
        self._console.drain()
        start = self._console.last_seq
        result = self._console.command(
            "btc.inquiry", timeout=timeout, duration_ms=duration_ms
        )
        count = _int(result, "count")
        return [
            _device_from_event(
                self._console.next_event("btc.dev", since_seq=start, timeout=timeout)
            )
            for _ in range(count)
        ]

    # -- connection ----------------------------------------------------------

    def connect(
        self,
        address: str,
        *,
        channel: int = 1,
        timeout: float = 15.0,
    ) -> ClassicLink:
        result = self._console.command(
            "btc.connect",
            timeout=timeout,
            addr=address.replace("-", "").lower(),
            channel=channel,
        )
        return ClassicLink(
            link_id=_int(result, "link"),
            address=result.get("addr", address),
            channel=_int(result, "channel", default=channel),
        )

    def disconnect(self, link: ClassicLink) -> None:
        self._console.command("btc.disconnect", link=link.link_id)

    # -- SPP data ------------------------------------------------------------

    def spp_send(self, link: ClassicLink, data: bytes, *, timeout: float = 10.0) -> None:
        self._console.command(
            "btc.spp.send", timeout=timeout, link=link.link_id, data=data
        )

    def wait_data(
        self,
        *,
        link: Optional[ClassicLink] = None,
        since_seq: Optional[int] = None,
        timeout: float = 10.0,
    ) -> bytes:
        event = self._console.next_event(
            "btc.data",
            timeout=timeout,
            since_seq=since_seq,
            link=link.link_id if link is not None else None,
        )
        return _payload(event["values"])

    # -- events --------------------------------------------------------------

    def wait_connected(self, *, timeout: float = 15.0) -> ClassicLink:
        event = self._console.next_event("btc.conn", timeout=timeout)
        values = event["values"]
        return ClassicLink(
            link_id=_int(values, "link"),
            address=values.get("addr", ""),
            channel=_int(values, "channel", default=1),
        )


def _device_from_event(event: Dict[str, Any]) -> ClassicDevice:
    values = event["values"]
    rssi = values.get("rssi")
    try:
        signal = int(rssi) if rssi not in (None, "") else None
    except ValueError as exc:
        raise InstrumentProtocolError(f"rssi={rssi!r} is not an integer") from exc
    return ClassicDevice(
        address=values.get("addr", ""),
        name=values.get("name", ""),
        rssi=signal,
    )
