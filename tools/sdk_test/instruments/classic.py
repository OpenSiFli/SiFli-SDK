# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Classic Bluetooth half of the ``bt_peer`` instrument.

``bt_peer:ble,classic`` is a capability *set*, not two fixtures: one
:class:`~sdk_test.instruments.console.InstrumentConsole`, one console stream.
The Classic operations are namespaced ``btc.*`` and multiplexed by request id /
event sequence exactly like ``gap.*`` / ``gatt.*``.

Wire conventions, stated once because they are easy to get backwards:

* ``uuid`` is in *display order* -- most significant byte first -- the reverse
  of the BLE convention. SPP is ``1101`` on the wire, not ``0111``.
* ``name`` is the UTF-8 bytes of the name, hex encoded. Decoding replaces an
  invalid UTF-8 sequence instead of failing; a name that is not hexadecimal at
  all is a protocol error.
* ``addr`` is bare hex. The host strips ``:``/``-`` and lower-cases it before
  sending, and the instrument reports it the same way.
* a Classic ``link`` id and a BLE ``Link`` id are separate namespaces owned by
  separate stacks. Passing one where the other belongs is a bug the ids cannot
  detect, so keep them apart in a dual-mode suite.

``BtPeer.classic`` only hands this class out when ``@info caps`` contains
``classic``; a BLE-only image fails there with a sentence rather than at the
first ``btc.*`` command. Anything the firmware has not implemented is an
``@err <id> unsupported <op>``, which surfaces as a normal
:class:`~sdk_test.instruments.errors.InstrumentCommandError`.
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


#: SPP's UUID-16 in *display order*: the wire carries ``1101``. A BLE UUID is
#: spelled least significant byte first, so the same number would be ``0111``
#: there -- do not copy one spelling into the other API.
SPP_UUID = "1101"


@dataclass(frozen=True)
class ClassicDevice:
    """One device returned by ``btc.inquiry``."""

    address: str
    name: str = ""
    rssi: Optional[int] = None


@dataclass(frozen=True)
class ClassicLocal:
    """The instrument's own Classic identity, as ``btc.local`` reports it."""

    address: str
    name: str = ""


@dataclass(frozen=True)
class ClassicLink:
    """One Classic connection, addressed by the instrument's own link id.

    ``role`` says which side of the link the *instrument* is on: ``"client"``
    when it connected to the DUT (``role=0``), ``"server"`` when the DUT
    connected to it (``role=1``).

    ``link_id`` is a Classic id. It shares no numbering with a BLE
    ``Link`` and must not be passed to a ``gap.*`` / ``gatt.*`` operation (or
    the reverse).
    """

    link_id: int
    address: str
    channel: int = 1
    role: str = "client"


class BtClassicInstrument:
    """Classic operations over the same line protocol as the BLE half.

    Only the first contract is defined here: local identity, inquiry,
    connect/disconnect and SPP data. Profiles (A2DP, HFP, AVRCP) are added as
    their firmware modules land; keeping them out until then means a suite
    cannot accidentally depend on an operation the lab cannot deliver.
    """

    def __init__(self, console: InstrumentConsole) -> None:
        self._console = console

    # -- lifecycle -----------------------------------------------------------

    def reset(self) -> None:
        """Drop every link and forget events from the previous state.

        Same shape as ``BleInstrument.reset``: the firmware may have queued a
        ``btc.disc`` while resetting, and that event belongs to the state the
        test just left, not to the one it is about to set up.
        """
        self._console.command("state.reset")
        self._console.clear_events()

    def local(self, timeout: float = 10.0, wait_ms: int = 5000) -> ClassicLocal:
        """Bring up the local Classic stack and return its address and name.

        ``wait_ms`` is the instrument's own budget for the stack to come up,
        not the host's wait; ``timeout`` bounds the latter. ``name`` arrives
        hex encoded and is decoded here.
        """
        result = self._console.command(
            "btc.local", timeout=timeout, timeout_ms=wait_ms
        )
        return ClassicLocal(
            address=_normalize_address(result.get("addr", "")),
            name=_decode_name(result.get("name", "")),
        )

    # -- discovery -----------------------------------------------------------

    def inquiry(
        self,
        duration_ms: int = 10000,
        *,
        max_rsp: int = 0,
        timeout: Optional[float] = None,
    ) -> List[ClassicDevice]:
        """Inquire for ``duration_ms`` and return every device heard.

        ``max_rsp`` caps the number of responses: ``0`` (the default) means no
        cap, so only ``duration_ms`` ends the inquiry; a non-zero cap ends it
        early once that many devices have answered.

        The reply carries ``first_seq``, so the events are consumed from the
        firmware's boundary: a late report from an earlier inquiry cannot be
        counted as this one's. ``timeout`` defaults to the inquiry duration
        plus a fixed allowance, because the instrument holds the answer until
        the inquiry has finished.
        """
        if timeout is None:
            timeout = duration_ms / 1000.0 + 10.0
        _, events, _ = self._console.command_events(
            "btc.inquiry",
            "btc.dev",
            timeout=timeout,
            boundary="firmware",
            duration_ms=duration_ms,
            max_rsp=max_rsp,
        )
        return [_device_from_event(event) for event in events]

    # -- pairing -------------------------------------------------------------

    def pair(
        self,
        address: str,
        *,
        timeout: float = 20.0,
        timeout_ms: int = 15000,
    ) -> None:
        """Pair with ``address`` so its SPP channel will authenticate.

        SPP's RFCOMM channel requires an authenticated link. Without a bond,
        the firmware's ``btc.connect`` fails with a timeout rather than a
        clearer error, so a suite pairs first.

        Pairing is persistent: it survives a reconnect, and only
        ``state.reset`` removes the bond. ``timeout_ms`` is the instrument's
        own pairing budget; ``timeout`` bounds the host's wait. A refusal
        (``not_ready``, ``invalid_arg``, ``busy``, ``timeout``, ``state``)
        arrives as an ``@err`` and is raised by ``command``.
        """
        self._console.command(
            "btc.pair",
            timeout=timeout,
            addr=_normalize_address(address),
            timeout_ms=timeout_ms,
        )

    # -- connection ----------------------------------------------------------

    def connect(
        self,
        address: str,
        *,
        uuid: str = SPP_UUID,
        timeout: float = 15.0,
        timeout_ms: int = 10000,
    ) -> ClassicLink:
        """Connect to ``address`` and return the link the instrument holds.

        ``uuid`` selects the service, in display order (SPP is ``1101``).
        ``timeout_ms`` is the instrument's own connection budget; ``timeout``
        bounds the host's wait for the answer.

        No channel is sent: the instrument derives it from the paging result,
        and a host-chosen channel could only disagree with it.
        """
        result = self._console.command(
            "btc.connect",
            timeout=timeout,
            addr=_normalize_address(address),
            uuid=uuid,
            timeout_ms=timeout_ms,
        )
        return ClassicLink(
            link_id=_int(result, "link"),
            address=result.get("addr", _normalize_address(address)),
            channel=_int(result, "channel", default=1),
            role=_role_name(result.get("role")),
        )

    def disconnect(self, link: ClassicLink) -> None:
        """Ask for the link to be torn down; ``btc.disc`` confirms it."""
        self._console.command("btc.disconnect", link=link.link_id)

    # -- SPP data ------------------------------------------------------------

    def spp_send(self, link: ClassicLink, data: bytes, *, timeout: float = 10.0) -> None:
        """Write ``data`` to the link's SPP channel.

        The reply's ``bytes=N`` is deliberately not checked here: ``command``
        already turns a refusal into an exception, and a short write the
        firmware accepts is its own bug, to be reported through ``@err``.
        """
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
        """One SPP payload from ``link`` (from any link when omitted).

        A packet for another link stays queued: a test waiting on one link's
        data must not lose its peer's.
        """
        event = self._console.next_event(
            "btc.data",
            timeout=timeout,
            since_seq=since_seq,
            link=link.link_id if link is not None else None,
        )
        return _payload(event["values"])

    # -- events --------------------------------------------------------------

    def wait_connected(
        self,
        *,
        since_seq: Optional[int] = None,
        timeout: float = 15.0,
    ) -> ClassicLink:
        """Wait for an incoming connection and return the link.

        ``since_seq`` pins the wait to events after a point the caller captured
        before provoking the connection; without it the oldest queued
        ``btc.conn`` is taken.

        A connection the *instrument* initiates through :meth:`connect` also
        emits ``btc.conn``; the two paths are told apart by ``role``.
        """
        event = self._console.next_event(
            "btc.conn", timeout=timeout, since_seq=since_seq
        )
        values = event["values"]
        return ClassicLink(
            link_id=_int(values, "link"),
            address=values.get("addr", ""),
            channel=_int(values, "channel", default=1),
            role=_role_name(values.get("role")),
        )

    def wait_disconnected(
        self,
        *,
        link: Optional[ClassicLink] = None,
        since_seq: Optional[int] = None,
        timeout: float = 10.0,
    ) -> None:
        """Wait for ``btc.disc`` on ``link`` (on any link when omitted).

        ``btc.disconnect`` answers ``@ok`` as soon as the request is accepted;
        the link is down only when this event arrives, which is why teardown
        has a separate wait.
        """
        self._console.next_event(
            "btc.disc",
            timeout=timeout,
            since_seq=since_seq,
            link=link.link_id if link is not None else None,
        )


# -- parsing helpers ---------------------------------------------------------


def _normalize_address(address: str) -> str:
    """The wire carries bare lower-case hex; a human writes separators."""
    return address.replace(":", "").replace("-", "").strip().lower()


def _decode_name(raw: str) -> str:
    """Decode a hex-encoded UTF-8 name."""
    if not raw:
        return ""
    try:
        encoded = bytes.fromhex(raw)
    except ValueError as exc:
        raise InstrumentProtocolError(f"name={raw!r} is not hexadecimal") from exc
    return encoded.decode("utf-8", errors="replace")


def _role_name(raw: Optional[str]) -> str:
    """The wire carries 0/1; the dataclass keeps the readable spelling."""
    if raw is None:
        return "client"
    return {"0": "client", "1": "server"}.get(raw, raw)


def _device_from_event(event: Dict[str, Any]) -> ClassicDevice:
    values = event["values"]
    rssi = values.get("rssi")
    try:
        signal = int(rssi) if rssi not in (None, "") else None
    except ValueError as exc:
        raise InstrumentProtocolError(f"rssi={rssi!r} is not an integer") from exc
    return ClassicDevice(
        address=values.get("addr", ""),
        name=_decode_name(values.get("name", "")),
        rssi=signal,
    )
