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


#: Client characteristic configuration, as it appears on the wire (LSB first).
CCCD_UUID = bytes([0x02, 0x29])


@dataclass(frozen=True)
class Characteristic:
    index: int
    handle: int
    value_handle: int
    descriptors: List[int] = field(default_factory=list)
    #: UUID of each descriptor, parallel to ``descriptors``; ``None`` where the
    #: instrument did not report one. Bytes, in the order the peer sent them.
    descriptor_uuids: List[Optional[bytes]] = field(default_factory=list)
    #: The characteristic's own UUID (2 or 16 bytes, as the peer sent it).
    uuid: Optional[bytes] = None

    def descriptor_handle(self, uuid: bytes) -> int:
        """The handle of the descriptor with @p uuid, if the peer reported it."""
        for handle, reported in zip(self.descriptors, self.descriptor_uuids):
            if reported == uuid:
                return handle
        raise AssertionError(
            f"characteristic {self.index} has no descriptor {uuid.hex()}; "
            f"reported: {self.descriptor_uuids}"
        )


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

    def characteristic(self, uuid: bytes) -> Characteristic:
        """The characteristic with @p uuid, selected the way the profile defines it.

        Position is not an identifier: a profile's characteristic order is an
        implementation detail, and a service may hold several.
        """
        for characteristic in self.characteristics:
            if characteristic.uuid == uuid:
                return characteristic
        raise AssertionError(
            f"no characteristic {uuid.hex()} in this service; reported: "
            f"{[c.uuid.hex() if c.uuid else None for c in self.characteristics]}"
        )

    def cccd_handle(self, index: int = 0) -> int:
        """The CCCD of a discovered characteristic.

        Selected by UUID, because a characteristic may carry other descriptors
        (a HID report has a report reference next to its CCCD). A characteristic
        with exactly one descriptor can only be the CCCD, so that one is taken
        as it is; with several and no UUID match the instrument did not report
        enough to identify it, and guessing would write the wrong handle.
        """
        try:
            characteristic = self.characteristics[index]
        except IndexError as exc:
            raise InstrumentProtocolError(
                f"characteristic {index} was not discovered "
                f"({len(self.characteristics)} present)"
            ) from exc

        descriptors = characteristic.descriptors
        if not descriptors:
            raise AssertionError(
                f"characteristic {index} has no descriptor, so there is no CCCD to write"
            )
        if len(descriptors) == 1:
            return descriptors[0]

        try:
            return characteristic.descriptor_handle(CCCD_UUID)
        except AssertionError as exc:
            raise InstrumentProtocolError(
                f"characteristic {index} has {len(descriptors)} descriptors and "
                f"none is a CCCD ({CCCD_UUID.hex()}); the instrument reported: "
                f"{characteristic.descriptor_uuids}"
            ) from exc


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
        result, events, boundary = self._console.command_events(
            "gap.scan",
            "adv",
            timeout=timeout,
            duration_ms=duration_ms,
            interval_ms=interval_ms,
            window_ms=window_ms,
            dup=int(duplicate_filter),
            rssi_min=rssi_min,
        )
        reports = [_adv_from_event(event) for event in events]

        # `scan.done` is informational; consume it so a later wait cannot see it
        # as this scan's answer.
        try:
            self._console.next_event(
                "scan.done", since_seq=boundary, timeout=0.05
            )
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
        if report is not None and address is not None:
            raise ValueError(
                "connect() takes an AdvReport or an address, not both: the "
                "firmware prefers the report's index and would ignore the address"
            )

        kwargs: Dict[str, Any] = {
            "timeout_ms": timeout_ms,
            "interval_ms": interval_ms,
        }
        if report is not None:
            # The scan index is 1-based on the wire; the firmware prints it
            # after appending the report, so it is used verbatim.
            kwargs["index"] = report.index
            address = report.address
            kwargs["addr_type"] = 0 if report.address_type == "public" else 1
        else:
            kwargs["addr"] = address.replace("-", "").lower()
            kwargs["addr_type"] = address_type

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
        _, events, _ = self._console.command_events(
            "dev.list", "link", count_key="links"
        )
        links = []
        for event in events:
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

    def start_periodic_advertising(
        self,
        data: bytes = b"",
        *,
        interval_ms: int = 100,
        timeout: float = 15.0,
    ) -> None:
        """Run a periodic advertising set (SID 0) carrying @p data.

        The primary-channel advertisement that carries the sync info is
        implicit: the peer firmware starts it as part of this op.
        """
        self._console.command(
            "periodic.adv.start",
            timeout=timeout,
            interval_ms=interval_ms,
            data=data,
        )

    def stop_periodic_advertising(self, *, timeout: float = 10.0) -> None:
        self._console.command("periodic.adv.stop", timeout=timeout)

    def sync_periodic(
        self,
        report: Optional[AdvReport] = None,
        *,
        address: Optional[str] = None,
        address_type: int = 0,
        timeout: float = 20.0,
    ) -> int:
        """Synchronise to a periodic advertiser and return the sync activity.

        Either a scan report (its index names the advertisement the instrument
        stored, including its SID) or an address. The periodic channel's data
        then arrives as ``periodic`` events.
        """
        if report is None and address is None:
            raise ValueError("sync_periodic() needs an AdvReport or an address")
        if report is not None and address is not None:
            raise ValueError(
                "sync_periodic() takes an AdvReport or an address, not both"
            )

        if report is not None:
            result = self._console.command(
                "periodic.sync", timeout=timeout, index=report.index
            )
        else:
            result = self._console.command(
                "periodic.sync",
                timeout=timeout,
                addr=address.replace("-", "").lower(),
                addr_type=address_type,
            )
        return _int(result, "link")

    def stop_periodic_sync(self, *, timeout: float = 10.0) -> None:
        self._console.command("periodic.sync.stop", timeout=timeout)

    def wait_periodic(
        self,
        *,
        link: Optional[int] = None,
        since_seq: Optional[int] = None,
        timeout: float = 15.0,
    ) -> bytes:
        """One periodic-channel payload, optionally from a given sync activity.

        Like the notification waits, the default boundary is "now"; the
        instrument reports a periodic payload only when it changes.
        """
        if since_seq is None:
            since_seq = self._console.last_seq
        event = self._console.next_event(
            "periodic", timeout=timeout, since_seq=since_seq, link=link
        )
        return _payload(event["values"])

    # -- GATT client ---------------------------------------------------------

    def discover(
        self, link: Link, uuid: Optional[bytes] = None, timeout: float = 15.0
    ) -> Service:
        """Discover a service and return its characteristics.

        The device emits every ``@evt char`` before the terminal ``@ok``, so by
        the time ``command`` returns the events are already queued. The default
        timeout is above the firmware's own discovery budget, so a slow search
        is reported by the instrument rather than by the host giving up first.
        """
        # The firmware's boundary, not the host's: a timed-out discovery can
        # leave char events in flight, and they must not be counted here.
        _, events, _ = self._console.command_events(
            "gatt.discover",
            "char",
            timeout=timeout,
            match={"link": link.link_id},
            link=link.link_id,
            uuid=uuid.hex() if uuid else None,
        )
        characteristics = [_characteristic_from_event(event) for event in events]
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
        before the action that provokes the notification. Defaulting to the
        current sequence means "from now": a packet left over from an earlier
        test cannot satisfy it. A caller that wants history passes an explicit
        earlier boundary, which works as long as those events have not already
        been discarded by a previous boundary.
        """
        if since_seq is None:
            since_seq = self._console.last_seq
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
        if since_seq is None:
            since_seq = self._console.last_seq
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
        auth: Optional[str] = None,
        passkey: Optional[int] = None,
        timeout: float = 40.0,
    ) -> None:
        """Pair with the peer; returns once the link is encrypted.

        ``auth`` picks the procedure -- ``legacy``, ``legacy_mitm``, ``secure``,
        ``secure_mitm`` or the numeric ``gap_auth`` -- and defaults to whatever
        the instrument firmware was built with.

        The instrument holds the ``@ok`` until pairing has finished and the
        link is encrypted, because that is when a protected attribute (a HID
        CCCD, for one) starts accepting writes. A pairing that fails or leaves
        the link in the clear is an ``@err``, not a silent success.
        """
        self._console.command(
            "sec.bond",
            timeout=timeout,
            link=link.link_id,
            io=io,
            auth=auth,
            passkey=passkey,
        )

    def wait_passkey(
        self,
        *,
        link: Optional[Link] = None,
        since_seq: Optional[int] = None,
        timeout: float = 30.0,
    ) -> int:
        if since_seq is None:
            since_seq = self._console.last_seq
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
    descriptors: List[int] = []
    descriptor_uuids: List[Optional[bytes]] = []
    for token in values.get("descs", "").split(","):
        if not token:
            continue
        handle, _, uuid_hex = token.partition(":")
        try:
            descriptors.append(int(handle))
            descriptor_uuids.append(bytes.fromhex(uuid_hex) if uuid_hex else None)
        except ValueError as exc:
            raise InstrumentProtocolError(
                f"descs={values.get('descs')!r} is not a handle[:uuid] list"
            ) from exc
    uuid_hex = values.get("uuid", "")
    return Characteristic(
        index=_int(values, "index"),
        handle=_int(values, "handle"),
        value_handle=_int(values, "value_handle"),
        descriptors=descriptors,
        descriptor_uuids=descriptor_uuids,
        uuid=bytes.fromhex(uuid_hex) if uuid_hex else None,
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
