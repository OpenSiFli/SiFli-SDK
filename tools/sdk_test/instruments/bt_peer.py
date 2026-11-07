# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""The BLE half of the ``bt_peer`` instrument.

Nothing here knows which example is under test or which service it exposes:
the suite passes the UUID it discovered from the DUT, and every command maps to
one protocol op. That is the difference from ``sdk_test.ble_peer``, whose
``SERVICE_UUID_BYTES`` named one example.

``BtPeer`` is the facade the pytest fixture yields. ``.ble`` is the BLE
instrument; a ``.classic`` sibling will be added when the same peer firmware
grows the Classic stack.
"""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Sequence

from ._values import int_value as _int
from ._values import payload as _payload
from .classic import BtClassicInstrument
from .console import InstrumentConsole
from .errors import InstrumentCapabilityError
from .errors import InstrumentProtocolError
from .errors import InstrumentTimeout


@dataclass(frozen=True)
class AdvReport:
    """One advertising report emitted while scanning.

    ``rssi`` is ``None`` when the instrument's report carried no signal line,
    which a split console read can produce. Such a report sorts last rather
    than at 0 dBm: an unmeasured device must not win "strongest".
    """

    index: int
    address: str
    address_type: str
    rssi: Optional[int]
    data: bytes = b""
    #: False for a non-connectable report (a background broadcaster, for
    #: instance); ``connect_to_dut`` must not pick one of those.
    connectable: bool = True


@dataclass(frozen=True)
class Link:
    """A connection the instrument holds, addressed by its own link id."""

    link_id: int
    address: str
    role: str = "central"
    interval_ms: int = 0


@dataclass(frozen=True)
class Characteristic:
    index: int
    handle: int
    value_handle: int
    descriptors: List[int] = field(default_factory=list)


@dataclass(frozen=True)
class Service:
    characteristics: List[Characteristic]

    def value_handle(self, index: int = 0) -> int:
        try:
            return self.characteristics[index].value_handle
        except IndexError as exc:
            raise InstrumentProtocolError(
                f"characteristic {index} was not discovered "
                f"({len(self.characteristics)} present)"
            ) from exc

    def cccd_handle(self, index: int = 0) -> int:
        """The first descriptor of a characteristic -- its CCCD.

        The same convention as the old parser: the examples' service table has
        exactly one descriptor, the client characteristic configuration.
        """
        try:
            descriptors = self.characteristics[index].descriptors
        except IndexError as exc:
            raise InstrumentProtocolError(
                f"characteristic {index} was not discovered "
                f"({len(self.characteristics)} present)"
            ) from exc
        if not descriptors:
            raise AssertionError(
                f"characteristic {index} has no descriptor, so there is no CCCD to write"
            )
        return descriptors[0]


class BleInstrument:
    """BLE operations over the line protocol."""

    def __init__(self, console: InstrumentConsole) -> None:
        self._console = console

    # -- lifecycle -----------------------------------------------------------

    def reset(self) -> None:
        """Drop every link, stop scanning/advertising, free dynamic services."""
        self._console.command("state.reset")
        # The firmware may have queued disconnection events while resetting;
        # they belong to the previous state, not to the next test.
        self._console.clear_events()

    # -- scanning ------------------------------------------------------------

    def scan(
        self,
        duration_ms: int = 3000,
        *,
        interval_ms: int = 100,
        window_ms: int = 50,
        duplicate_filter: bool = False,
        rssi_min: int = -127,
        timeout: float = 20.0,
    ) -> List[AdvReport]:
        """Scan for ``duration_ms`` and return every report, strongest first."""
        # Parse what is already in flight *before* taking the boundary. An event
        # from a previous, timed-out scan can still be sitting in the console
        # buffer; without this it is parsed during this scan's pump with a seq
        # above the boundary and would be returned as this scan's report.
        self._console.drain()
        start = self._console.last_seq
        result = self._console.command(
            "gap.scan",
            timeout=timeout,
            duration_ms=duration_ms,
            interval_ms=interval_ms,
            window_ms=window_ms,
            dup=int(duplicate_filter),
            rssi_min=rssi_min,
        )
        count = _int(result, "count")
        reports = [
            _adv_from_event(self._console.next_event("adv", since_seq=start, timeout=timeout))
            for _ in range(count)
        ]
        # `scan.done` is informational; consume it so a later wait cannot see it
        # as this scan's answer.
        try:
            self._console.next_event("scan.done", since_seq=start, timeout=0.05)
        except InstrumentTimeout:
            pass
        return sorted(reports, key=_by_signal, reverse=True)

    def strongest(self, reports: Sequence[AdvReport]) -> AdvReport:
        if not reports:
            raise AssertionError(
                "the peer scanned but saw no advertisement at all; check that "
                "the DUT is advertising and within radio range"
            )
        return reports[0]

    # -- connection ----------------------------------------------------------

    def connect(
        self,
        report: Optional[AdvReport] = None,
        *,
        address: Optional[str] = None,
        address_type: int = 0,
        timeout_ms: int = 5000,
        interval_ms: int = 30,
        timeout: float = 15.0,
    ) -> Link:
        """Connect either to a scan report's index or to an explicit address."""
        if report is None and address is None:
            raise ValueError("connect() needs an AdvReport or an address")

        kwargs: Dict[str, Any] = {
            "timeout_ms": timeout_ms,
            "interval_ms": interval_ms,
        }
        if report is not None:
            # The scan index is 1-based on the wire; the firmware prints it
            # after appending the report, so it is used verbatim.
            kwargs["index"] = report.index
            address = address or report.address
            if "addr_type" not in kwargs:
                kwargs["addr_type"] = 0 if report.address_type == "public" else 1
        if address is not None:
            kwargs["addr"] = address.replace("-", "").lower()
            kwargs.setdefault("addr_type", address_type)

        result = self._console.command("gap.connect", timeout=timeout, **kwargs)
        return Link(
            link_id=_int(result, "link"),
            address=result.get("addr", address or ""),
            role=_role_name(result.get("role")),
            interval_ms=_int(result, "interval_ms", default=0),
        )

    def disconnect(self, link: Link) -> None:
        self._console.command("gap.disconnect", link=link.link_id)

    def update_connection(
        self,
        link: Link,
        *,
        interval_min_ms: int,
        interval_max_ms: int,
        latency: int = 0,
        timeout_ms: int = 5000,
        timeout: float = 10.0,
    ) -> None:
        """Ask the controller to change the link's connection parameters."""
        self._console.command(
            "gap.conn_params",
            timeout=timeout,
            link=link.link_id,
            interval_min_ms=interval_min_ms,
            interval_max_ms=interval_max_ms,
            latency=latency,
            timeout_ms=timeout_ms,
        )

    def set_phy(self, link: Link, phy: int, *, timeout: float = 10.0) -> None:
        """Request a PHY change; ``phy`` is 1 (1M), 2 (2M) or 3 (Coded)."""
        self._console.command("gap.phy", timeout=timeout, link=link.link_id, phy=phy)

    def set_data_length(
        self,
        link: Link,
        tx_octets: int,
        tx_time: int = 0,
        *,
        timeout: float = 10.0,
    ) -> None:
        self._console.command(
            "gap.data_len",
            timeout=timeout,
            link=link.link_id,
            tx_octets=tx_octets,
            tx_time=tx_time,
        )

    def list_links(self) -> List[Link]:
        self._console.drain()
        result = self._console.command("dev.list")
        count = _int(result, "links")
        links = []
        for _ in range(count):
            event = self._console.next_event("link")
            values = event["values"]
            links.append(
                Link(
                    link_id=_int(values, "link"),
                    address=values.get("addr", ""),
                    role=_role_name(values.get("role")),
                    interval_ms=_int(values, "interval_ms", default=0),
                )
            )
        return links

    # -- advertising ---------------------------------------------------------

    def advertise(
        self,
        *,
        mode: str = "extended",
        data: bytes = b"",
        interval_ms: int = 30,
        connectable: bool = True,
    ) -> None:
        self._console.command(
            "adv.start",
            mode=mode,
            interval_ms=interval_ms,
            connectable=int(connectable),
            data=data,
        )

    def stop_advertising(self) -> None:
        self._console.command("adv.stop")

    def set_advertising_data(self, data: bytes, *, timeout: float = 10.0) -> None:
        """Replace the running advertisement's data."""
        self._console.command("adv.data", timeout=timeout, data=data)

    # -- GATT client ---------------------------------------------------------

    def discover(
        self, link: Link, uuid: Optional[bytes] = None, timeout: float = 10.0
    ) -> Service:
        """Discover a service and return its characteristics.

        The device emits every ``@evt char`` before the terminal ``@ok``, so by
        the time ``command`` returns the events are already queued.
        """
        # Same reason as scan(): a timed-out discovery can leave char events in
        # flight; they must not be counted as this discovery's answer.
        self._console.drain()
        start = self._console.last_seq
        result = self._console.command(
            "gatt.discover",
            timeout=timeout,
            link=link.link_id,
            uuid=uuid.hex() if uuid else None,
        )
        count = _int(result, "count")
        characteristics = [
            _characteristic_from_event(
                self._console.next_event(
                    "char", since_seq=start, link=link.link_id, timeout=timeout
                )
            )
            for _ in range(count)
        ]
        if not characteristics:
            raise AssertionError("the peer discovered the service but reported no characteristic")
        return Service(characteristics=characteristics)

    def read(self, link: Link, handle: int, timeout: float = 10.0) -> bytes:
        result = self._console.command(
            "gatt.read", timeout=timeout, link=link.link_id, handle=handle
        )
        return _payload(result)

    def write(
        self,
        link: Link,
        handle: int,
        data: bytes,
        *,
        response: bool = True,
        timeout: float = 10.0,
    ) -> None:
        self._console.command(
            "gatt.write",
            timeout=timeout,
            link=link.link_id,
            handle=handle,
            response=int(response),
            data=data,
        )

    def subscribe(
        self, link: Link, cccd_handle: int, *, enable: bool = True, timeout: float = 10.0
    ) -> None:
        self._console.command(
            "gatt.subscribe",
            timeout=timeout,
            link=link.link_id,
            cccd=cccd_handle,
            enable=int(enable),
        )

    def exchange_mtu(self, link: Link, timeout: float = 10.0) -> int:
        result = self._console.command("gatt.mtu", timeout=timeout, link=link.link_id)
        return _int(result, "mtu", default=0)

    # -- events --------------------------------------------------------------

    def wait_notification(
        self,
        handle: int,
        *,
        link: Optional[Link] = None,
        since_seq: Optional[int] = None,
        timeout: float = 10.0,
    ) -> bytes:
        """Wait for the next notification on ``handle`` and return its payload.

        ``since_seq`` pins the wait to events after a point the caller captured
        before the action that provokes the notification; without it, a packet
        left over from an earlier test can satisfy the wait.
        """
        event = self._console.next_event(
            "notify",
            timeout=timeout,
            since_seq=since_seq,
            link=link.link_id if link is not None else None,
            handle=handle,
        )
        return _payload(event["values"])

    def wait_read(
        self,
        handle: int,
        *,
        link: Optional[Link] = None,
        since_seq: Optional[int] = None,
        timeout: float = 10.0,
    ) -> bytes:
        """Wait for an unsolicited read (the DUT read the peer's value)."""
        event = self._console.next_event(
            "read",
            timeout=timeout,
            since_seq=since_seq,
            link=link.link_id if link is not None else None,
            handle=handle,
        )
        return _payload(event["values"])

    # -- GATT server ---------------------------------------------------------

    def register_loopback(self, timeout: float = 10.0) -> int:
        """Register the generic echo service and return its value handle."""
        result = self._console.command("svc.register", timeout=timeout, name="loopback")
        return _int(result, "handle")

    def notify(self, link: Link, handle: int, data: bytes, timeout: float = 10.0) -> None:
        self._console.command(
            "svc.notify", timeout=timeout, link=link.link_id, handle=handle, data=data
        )

    # -- security ------------------------------------------------------------

    def bond(
        self,
        link: Link,
        *,
        io: str = "no_input_no_output",
        passkey: Optional[int] = None,
        timeout: float = 30.0,
    ) -> None:
        self._console.command(
            "sec.bond", timeout=timeout, link=link.link_id, io=io, passkey=passkey
        )

    def wait_passkey(
        self,
        *,
        link: Optional[Link] = None,
        since_seq: Optional[int] = None,
        timeout: float = 30.0,
    ) -> int:
        event = self._console.next_event(
            "passkey",
            timeout=timeout,
            since_seq=since_seq,
            link=link.link_id if link is not None else None,
        )
        return _int(event["values"], "value")


class BtPeer:
    """The instrument fixture's facade.

    ``capability`` is the fixture parameter (``ble`` now, ``classic`` later).
    Constructing the facade verifies the firmware actually provides it, so a
    mis-flashed lab fails at the first test instead of at the first `scan`.
    """

    def __init__(self, console: InstrumentConsole, *, capability: str = "") -> None:
        self._console = console
        self.capability = capability
        self.info = console.expect_ready(capability)
        self._ble: Optional[BleInstrument] = None
        self._classic: Optional[BtClassicInstrument] = None

    @property
    def console(self) -> InstrumentConsole:
        return self._console

    @property
    def caps(self) -> set:
        """The capabilities the firmware actually advertises.

        The declared fixture parameter says what the *case* needs; this says
        what the *image* provides. A sub-API is handed out only when the image
        provides it, so a bare ``bt_peer`` declaration still cannot drive a
        Classic-only peer's BLE half by accident.
        """
        return {
            entry.strip()
            for entry in self.info.get("caps", "").split(",")
            if entry.strip()
        }

    @property
    def ble(self) -> BleInstrument:
        if "ble" not in self.caps:
            raise InstrumentCapabilityError(
                "this instrument does not advertise the 'ble' capability "
                f"(caps={sorted(self.caps) or 'none'})"
            )
        if self._ble is None:
            self._ble = BleInstrument(self._console)
        return self._ble

    @property
    def classic(self) -> BtClassicInstrument:
        if "classic" not in self.caps:
            raise InstrumentCapabilityError(
                "this instrument does not advertise the 'classic' capability "
                f"(caps={sorted(self.caps) or 'none'}); the lab must flash a "
                "bt_peer image built with the Classic stack"
            )
        if self._classic is None:
            self._classic = BtClassicInstrument(self._console)
        return self._classic


# -- parsing helpers ---------------------------------------------------------


def _adv_from_event(event: Dict[str, Any]) -> AdvReport:
    values = event["values"]
    rssi = values.get("rssi")
    # The wire carries the controller's 0/1; the host's dataclass keeps the
    # human spelling the suites compare against. Accept either so a recorded
    # transcript can be fed to the parser unchanged.
    raw_type = values.get("type", "0")
    address_type = {"0": "public", "1": "random"}.get(raw_type, raw_type)
    try:
        signal = int(rssi) if rssi not in (None, "") else None
    except ValueError as exc:
        raise InstrumentProtocolError(f"rssi={rssi!r} is not an integer") from exc
    # A report that carries a payload must carry its length too; dropping the
    # bytes would turn a firmware bug into a silent empty advertisement.
    data = _payload(values) if ("len" in values or "data" in values) else b""
    return AdvReport(
        index=_int(values, "index"),
        address=values.get("addr", ""),
        address_type=address_type,
        rssi=signal,
        data=data,
        connectable=values.get("conn", "1") != "0",
    )


def _characteristic_from_event(event: Dict[str, Any]) -> Characteristic:
    values = event["values"]
    try:
        descriptors = [
            int(token)
            for token in values.get("descs", "").split(",")
            if token
        ]
    except ValueError as exc:
        raise InstrumentProtocolError(
            f"descs={values.get('descs')!r} is not a handle list"
        ) from exc
    return Characteristic(
        index=_int(values, "index"),
        handle=_int(values, "handle"),
        value_handle=_int(values, "value_handle"),
        descriptors=descriptors,
    )


def _role_name(raw: Optional[str]) -> str:
    """The wire carries 0/1; the dataclass keeps the readable spelling."""
    if raw is None:
        return "central"
    return {"0": "central", "1": "peripheral"}.get(raw, raw)


def _by_signal(report: AdvReport) -> int:
    """Strongest first; an unmeasured report sorts last."""
    if report.rssi is None:
        return -(2 ** 31)
    return report.rssi
