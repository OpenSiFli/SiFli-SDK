# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Host-side USB observation for the ``usb_host`` fixture.

A device example is plugged into the machine the Lab Agent runs on, so every
question about it -- what enumerated, which COM port it exposes, what its MSC
volume contains -- is a question for that machine. The suite therefore never
runs ``lsusb`` or opens a port itself: it asks this client, which asks the
agent. That is what lets the runner stay on Linux while the device port sits
on a Windows bench.

The wire contract lives in ``docs/design/usb-host-observation.md``. The agent
answers JSON; the types here are the stable shape a suite programs against, so
a change to the agent's JSON is absorbed in one place.

Only the standard library: the client is a few dozen calls over
:class:`~sdk_test.agent_client.LabAgentClient`.
"""

from __future__ import annotations

import base64
import binascii
import time
from dataclasses import dataclass
from dataclasses import field
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List
from typing import Mapping
from typing import Optional
from typing import Tuple

from .agent_client import LabAgentClient
from .agent_client import TRANSPORT_ERROR_CODE
from .errors import LabAgentError
from .errors import SdkTestError


# -- errors ------------------------------------------------------------------


class UsbHostError(SdkTestError):
    """Base class for every failure of the ``usb_host`` capability."""


class UsbHostUnavailable(UsbHostError):
    """The agent has no USB host backend, or is too old to have the routes.

    A 404 *without* a ``code`` is an agent from before this capability existed;
    answering it as "no such device" would send a reader looking at the board.
    """


class UsbHostTimeout(UsbHostError, TimeoutError):
    """The agent did not see the thing within the budget it was given."""


class UsbNotFound(UsbHostError, LookupError):
    """The named device, serial token or file does not exist."""


class UsbConflict(UsbHostError):
    """The request contradicts the port's or volume's current state."""


class UsbDenied(UsbHostError, PermissionError):
    """The agent refused for safety: a protected port or an unsafe path."""


#: Codes that mean "no such object", not "this agent predates the route".
_NOT_FOUND_CODES = frozenset({"device_not_found", "token_not_found", "file_not_found"})

#: Codes that mean the agent applied a safety rule rather than a lookup failure.
_DENIED_CODES = frozenset({"path_denied", "forbidden_port"})


def translate_error(error: LabAgentError) -> UsbHostError:
    """Map one agent error onto the exception a suite should see.

    The status alone is not enough. The agent answers 404 both for "there is no
    such route" (an old agent) and for "there is no such device"; it answers 409
    both for a busy port and for a refused one. The ``code`` disambiguates, and
    where the two disagree the code wins -- a binding may have been written
    against an agent that classified ``forbidden_port`` as 409.
    """
    code = error.code
    status = error.status
    if code in _DENIED_CODES:
        return UsbDenied(str(error))
    if status is None and code == TRANSPORT_ERROR_CODE:
        # The agent was never reached. That is the capability being
        # unavailable for this run, not a wrong answer from it.
        return UsbHostUnavailable(str(error))
    if status == 408:
        return UsbHostTimeout(str(error))
    if status in (501, 503):
        return UsbHostUnavailable(str(error))
    if status == 409:
        return UsbConflict(str(error))
    if status == 404:
        if code in _NOT_FOUND_CODES:
            return UsbNotFound(str(error))
        return UsbHostUnavailable(str(error))
    return UsbHostError(str(error))


# -- data model --------------------------------------------------------------


def _int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _optional_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _tuple(payload: Any) -> Tuple[Any, ...]:
    if isinstance(payload, (list, tuple)):
        return tuple(payload)
    return ()


@dataclass(frozen=True)
class UsbInterface:
    """One interface descriptor, as the agent read it from the device."""

    number: int
    interface_class: int
    subclass: int
    protocol: int
    name: Optional[str] = None

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> "UsbInterface":
        return cls(
            number=_int(payload.get("number")),
            interface_class=_int(payload.get("class")),
            subclass=_int(payload.get("subclass")),
            protocol=_int(payload.get("protocol")),
            name=payload.get("name"),
        )


@dataclass(frozen=True)
class ComPort:
    """A COM port the device exposes, and the driver behind it."""

    port: str
    driver: Optional[str] = None

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> "ComPort":
        return cls(port=str(payload.get("port", "")), driver=payload.get("driver"))


@dataclass(frozen=True)
class VolumeInfo:
    """A mounted volume as enumeration reports it."""

    drive: str
    label: Optional[str] = None
    filesystem: Optional[str] = None
    size: Optional[int] = None
    removable: bool = True
    device_key: Optional[str] = None

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> "VolumeInfo":
        return cls(
            drive=str(payload.get("drive", "")),
            label=payload.get("label"),
            filesystem=payload.get("filesystem"),
            size=_optional_int(payload.get("size")),
            removable=bool(payload.get("removable", True)),
            device_key=payload.get("device_key"),
        )


@dataclass(frozen=True)
class HidInfo:
    """One HID interface and the usage its report descriptor declares."""

    interface: int
    usage_page: int
    usage: int
    protocol: Optional[int] = None
    report_length: Optional[int] = None
    product: Optional[str] = None

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> "HidInfo":
        return cls(
            interface=_int(payload.get("interface")),
            usage_page=_int(payload.get("usage_page")),
            usage=_int(payload.get("usage")),
            protocol=_optional_int(payload.get("protocol")),
            report_length=_optional_int(payload.get("report_length")),
            product=payload.get("product"),
        )


@dataclass(frozen=True)
class AudioEndpoint:
    """One isochronous endpoint, named the way the USB descriptor names it."""

    interface: int
    endpoint: str
    direction: str

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> "AudioEndpoint":
        return cls(
            interface=_int(payload.get("interface")),
            endpoint=str(payload.get("endpoint", "")),
            direction=str(payload.get("direction", "")),
        )


@dataclass(frozen=True)
class AudioEndpoints:
    """Capture (IN) and render (OUT) endpoints of an audio function."""

    inputs: Tuple[AudioEndpoint, ...] = field(default_factory=tuple)
    outputs: Tuple[AudioEndpoint, ...] = field(default_factory=tuple)

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> "AudioEndpoints":
        return cls(
            inputs=tuple(
                AudioEndpoint.from_json(entry)
                for entry in _tuple(payload.get("inputs"))
                if isinstance(entry, Mapping)
            ),
            outputs=tuple(
                AudioEndpoint.from_json(entry)
                for entry in _tuple(payload.get("outputs"))
                if isinstance(entry, Mapping)
            ),
        )


@dataclass(frozen=True)
class UsbDevice:
    """One enumerated USB device.

    ``key`` is the device's identity on this bench -- VID + PID +
    iSerialNumber -- and is what every later call quotes. It is not a port
    path: the suite asks about a device, not about where Windows happened to
    put it.
    """

    key: str
    instance_id: str
    vid: Optional[int]
    pid: Optional[int]
    serial: Optional[str]
    product: Optional[str]
    manufacturer: Optional[str]
    device_class: Optional[str]
    classes: Tuple[str, ...] = field(default_factory=tuple)
    interfaces: Tuple[UsbInterface, ...] = field(default_factory=tuple)
    com_ports: Tuple[ComPort, ...] = field(default_factory=tuple)
    volumes: Tuple[VolumeInfo, ...] = field(default_factory=tuple)
    hid: Tuple[HidInfo, ...] = field(default_factory=tuple)
    audio: Optional[AudioEndpoints] = None
    speed: Optional[str] = None
    bus_port: Optional[int] = None
    present: bool = True

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> "UsbDevice":
        audio = payload.get("audio")
        return cls(
            key=str(payload.get("key", "")),
            instance_id=str(payload.get("instance_id", "")),
            vid=_optional_int(payload.get("vid")),
            pid=_optional_int(payload.get("pid")),
            serial=payload.get("serial"),
            product=payload.get("product"),
            manufacturer=payload.get("manufacturer"),
            device_class=payload.get("device_class"),
            classes=tuple(str(entry) for entry in _tuple(payload.get("classes"))),
            interfaces=tuple(
                UsbInterface.from_json(entry)
                for entry in _tuple(payload.get("interfaces"))
                if isinstance(entry, Mapping)
            ),
            com_ports=tuple(
                ComPort.from_json(entry)
                for entry in _tuple(payload.get("com_ports"))
                if isinstance(entry, Mapping)
            ),
            volumes=tuple(
                VolumeInfo.from_json(entry)
                for entry in _tuple(payload.get("volumes"))
                if isinstance(entry, Mapping)
            ),
            hid=tuple(
                HidInfo.from_json(entry)
                for entry in _tuple(payload.get("hid"))
                if isinstance(entry, Mapping)
            ),
            audio=AudioEndpoints.from_json(audio)
            if isinstance(audio, Mapping)
            else None,
            speed=payload.get("speed"),
            bus_port=_optional_int(payload.get("bus_port")),
            present=bool(payload.get("present", True)),
        )


@dataclass(frozen=True)
class UsbHostCapabilities:
    """What the agent's host-side USB backend can do on this bench."""

    supported: bool
    platform: Optional[str] = None
    features: Tuple[str, ...] = field(default_factory=tuple)
    host: Mapping[str, Any] = field(default_factory=dict)
    reason: Optional[str] = None

    @classmethod
    def from_json(cls, payload: Mapping[str, Any]) -> "UsbHostCapabilities":
        host = payload.get("host")
        return cls(
            supported=bool(payload.get("supported", False)),
            platform=payload.get("platform"),
            features=tuple(str(entry) for entry in _tuple(payload.get("features"))),
            host=dict(host) if isinstance(host, Mapping) else {},
            reason=payload.get("reason"),
        )


# -- client ------------------------------------------------------------------


class UsbHost:
    """The host-side USB view of one allocated testbed.

    Session-scoped in practice: the fixture yields one per suite, and the COM
    sessions it opens are closed when the suite ends. Nothing here allocates
    hardware -- the testbed is already allocated and has already requested
    ``usb_host`` by the time this class is used.
    """

    def __init__(self, client: LabAgentClient, testbed_id: str) -> None:
        self._client = client
        self.testbed_id = testbed_id
        self._serials: List[UsbSerial] = []

    # -- capabilities --------------------------------------------------------

    def capabilities(self) -> UsbHostCapabilities:
        payload = self._call(self._client.usb_capabilities, self.testbed_id)
        return UsbHostCapabilities.from_json(payload)

    def require_supported(self) -> None:
        """Refuse an agent that cannot observe USB, naming the reason."""
        capabilities = self.capabilities()
        if not capabilities.supported:
            reason = capabilities.reason or "the agent reported no reason"
            raise UsbHostUnavailable(
                f"the Lab Agent cannot observe USB on this bench: {reason}"
            )

    def require(self, *features: str) -> None:
        """Refuse an agent whose backend lacks any of ``features``."""
        capabilities = self.capabilities()
        if not capabilities.supported:
            reason = capabilities.reason or "the agent reported no reason"
            raise UsbHostUnavailable(
                f"the Lab Agent cannot observe USB on this bench: {reason}"
            )
        missing = sorted(set(features) - set(capabilities.features))
        if missing:
            raise UsbHostUnavailable(
                f"the Lab Agent's USB backend lacks {', '.join(missing)} "
                f"(it offers: {', '.join(capabilities.features) or 'nothing'})"
            )

    # -- enumeration ---------------------------------------------------------

    def devices(self) -> Tuple[UsbDevice, ...]:
        payload = self._call(self._client.usb_devices, self.testbed_id)
        devices = payload.get("devices")
        if not isinstance(devices, list):
            raise UsbHostError(
                "the Lab Agent returned no device list for "
                f"{self.testbed_id}: {payload!r}"
            )
        return tuple(
            UsbDevice.from_json(entry)
            for entry in devices
            if isinstance(entry, Mapping)
        )

    def wait_for(
        self,
        *,
        timeout: float = 30.0,
        interval: float = 0.25,
        since: Optional[Iterable[str]] = None,
        match: Optional[Mapping[str, Any]] = None,
        **criteria: Any,
    ) -> UsbDevice:
        """Wait until a device matching the criteria enumerates, and return it.

        Criteria may be written as a mapping (``match={"class": 10}``) or as
        keywords (``serial="..."``), or both. A key given twice is refused
        rather than silently taking one side: the two spellings exist for
        different call sites, not to let one override the other.

        ``since`` names the devices that were already present, so a re-plug is
        observed rather than the test passing on the previous run's device.
        """
        merged: Dict[str, Any] = dict(match or {})
        conflicts = sorted(set(merged) & set(criteria))
        if conflicts:
            raise ValueError(
                f"wait_for() got {', '.join(conflicts)} both in match= and as "
                "keyword arguments"
            )
        merged.update(criteria)
        payload = self._call(
            self._client.usb_wait,
            self.testbed_id,
            merged,
            list(since or []),
            timeout,
            interval,
        )
        device = payload.get("device")
        if not isinstance(device, Mapping):
            raise UsbHostError(
                "the Lab Agent answered a usb wait without a device: "
                f"{payload!r}"
            )
        return UsbDevice.from_json(device)

    # -- volumes -------------------------------------------------------------

    def volume(self, device: UsbDevice, *, timeout: float = 20.0) -> "UsbVolume":
        """Wait for ``device``'s first mounted volume.

        Windows mounts a removable volume a moment after the device
        enumerates, so "no volume yet" is a wait, not a failure. Anything the
        agent refuses for another reason is raised at once.
        """
        deadline = time.monotonic() + float(timeout)
        last: Optional[BaseException] = None
        while True:
            try:
                volumes = self._volume_infos()
                for info in volumes:
                    if info.device_key == device.key:
                        return UsbVolume(self._client, self.testbed_id, info)
            except (UsbConflict, UsbHostTimeout) as exc:
                last = exc
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                detail = f"; last answer: {last}" if last else ""
                raise UsbHostTimeout(
                    f"{device.key} exposed no mounted volume within "
                    f"{timeout:.0f}s{detail}"
                ) from last
            time.sleep(min(0.25, remaining))

    def _volume_infos(self) -> Tuple[VolumeInfo, ...]:
        payload = self._call(self._client.usb_volumes, self.testbed_id)
        volumes = payload.get("volumes")
        if not isinstance(volumes, list):
            raise UsbHostError(
                "the Lab Agent returned no volume list for "
                f"{self.testbed_id}: {payload!r}"
            )
        return tuple(
            VolumeInfo.from_json(entry)
            for entry in volumes
            if isinstance(entry, Mapping)
        )

    # -- serial --------------------------------------------------------------

    def serial(
        self,
        device: UsbDevice,
        *,
        com_port: Optional[str] = None,
        baud: int = 115200,
        dtr: bool = False,
        rts: bool = False,
    ) -> "UsbSerial":
        """Open one of ``device``'s COM ports as a CDC session."""
        port = com_port or (device.com_ports[0].port if device.com_ports else None)
        if not port:
            ports = ", ".join(entry.port for entry in device.com_ports) or "none"
            raise UsbNotFound(
                f"{device.key} exposes no COM port to open (reported: {ports})"
            )
        payload = self._call(
            self._client.usb_serial_open,
            self.testbed_id,
            device.key,
            port,
            baud,
            dtr,
            rts,
        )
        token = payload.get("token")
        if not token:
            raise UsbHostError(
                f"the Lab Agent opened no serial session for {port}: {payload!r}"
            )
        serial = UsbSerial(
            client=self._client,
            testbed_id=self.testbed_id,
            token=str(token),
            port=str(payload.get("port", port)),
            device_key=str(payload.get("device_key", device.key)),
            baud=_int(payload.get("baud"), baud),
            dtr=bool(payload.get("dtr", dtr)),
            rts=bool(payload.get("rts", rts)),
        )
        self._serials.append(serial)
        return serial

    # -- audio and HID -------------------------------------------------------

    def audio(self, device: UsbDevice) -> AudioEndpoints:
        payload = self._call(
            self._client.usb_audio, self.testbed_id, device.key
        )
        return AudioEndpoints.from_json(payload)

    def hid(self, device: UsbDevice) -> Tuple[HidInfo, ...]:
        payload = self._call(self._client.usb_hid, self.testbed_id, device.key)
        interfaces = payload.get("interfaces", [])
        if not isinstance(interfaces, list):
            raise UsbHostError(
                f"the Lab Agent returned no HID interface list for "
                f"{device.key}: {payload!r}"
            )
        return tuple(
            HidInfo.from_json(entry)
            for entry in interfaces
            if isinstance(entry, Mapping)
        )

    # -- teardown ------------------------------------------------------------

    def close(self) -> None:
        """Close every serial session this client opened. Idempotent.

        Every session is attempted even when one fails: leaking the rest would
        hold their ports against the next case.
        """
        serials, self._serials = self._serials, []
        first: Optional[UsbHostError] = None
        for serial in serials:
            try:
                serial.close()
            except UsbHostError as exc:
                first = first or exc
        if first is not None:
            raise first

    def _call(self, function, *args):
        try:
            return function(*args)
        except LabAgentError as exc:
            raise translate_error(exc) from exc


class UsbSerial:
    """One open CDC session on the agent.

    ``read`` returns bytes, not str: a CDC frame is not text, and decoding it
    in the transport would make a binary assertion depend on the suite's
    locale. The cursor lives here, so reads are stateless on the wire exactly
    as a console's are.

    Usable as a context manager, which closes the session on the way out even
    when an assertion inside the block fails.
    """

    def __init__(
        self,
        *,
        client: LabAgentClient,
        testbed_id: str,
        token: str,
        port: str,
        device_key: str,
        baud: int,
        dtr: bool,
        rts: bool,
    ) -> None:
        self._client = client
        self._testbed_id = testbed_id
        self.token = token
        self.port = port
        self.device_key = device_key
        self.baud = baud
        self.dtr = dtr
        self.rts = rts
        self._cursor = 0
        self._closed = False
        #: Bytes received from one wire read but not yet handed to a caller.
        #: `read_exact` can consume only part of a read, and the agent's cursor
        #: has already moved past the rest -- dropping them would lose data
        #: that no later read can ask for again.
        self._pending = bytearray()

    @property
    def cursor(self) -> int:
        """Read position on the agent's stream, as an absolute byte offset.

        It may be ahead of what the caller has consumed: a wire read can bring
        back more than was asked for, and the surplus sits in ``_pending``.
        The next agent read must therefore start from here, not from the
        caller's byte count.
        """
        return self._cursor

    def read(self, timeout: float = 1.0) -> bytes:
        """Return bytes received after the last read; ``b""`` on a quiet port.

        Bytes left over from an earlier ``read_exact`` come first: they arrived
        before anything a new read can return.
        """
        if self._pending:
            data = bytes(self._pending)
            self._pending.clear()
            return data
        try:
            data, self._cursor, closed = self._client.usb_serial_read(
                self._testbed_id, self.token, self._cursor, timeout
            )
        except LabAgentError as exc:
            raise translate_error(exc) from exc
        self._closed = closed
        return data

    def read_exact(self, size: int, timeout: float = 5.0) -> bytes:
        """Return exactly ``size`` bytes, or raise :class:`UsbHostTimeout`.

        A CDC frame arrives in whatever chunks the host's driver hands over, so
        one read is not one frame. The short case is a failure, not a shorter
        answer: a caller comparing a frame must not compare half of one.

        A read may also hand over *more* than is still missing. The surplus is
        kept for the next ``read``/``read_exact`` rather than returned: the
        agent's cursor already moved past it, so returning it would both break
        the "exactly ``size``" promise and discard it.
        """
        if size < 0:
            raise ValueError("read_exact() needs a non-negative size")
        deadline = time.monotonic() + float(timeout)
        buffer = bytearray()
        while len(buffer) < size:
            # Bytes already received are still deliverable after the port
            # reports closed; only an empty pending buffer makes the shortfall
            # permanent.
            if self._closed and not self._pending:
                raise UsbConflict(
                    f"{self.port} was closed after {len(buffer)} of {size} bytes"
                )
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            buffer += self.read(min(0.5, remaining))
        if len(buffer) < size:
            raise UsbHostTimeout(
                f"{self.port}: read {len(buffer)} of {size} bytes within "
                f"{timeout:.1f}s"
            )
        if len(buffer) > size:
            self._pending = bytearray(buffer[size:])
        return bytes(buffer[:size])

    def write(self, data: str | bytes) -> int:
        """Send ``data`` and return how many bytes the agent wrote."""
        if isinstance(data, str):
            payload = data.encode("utf-8")
        else:
            payload = bytes(data)
        try:
            return self._client.usb_serial_write(
                self._testbed_id, self.token, payload
            )
        except LabAgentError as exc:
            raise translate_error(exc) from exc

    def set_dtr(self, dtr: bool) -> None:
        self._lines(dtr=dtr, rts=self.rts)

    def set_rts(self, rts: bool) -> None:
        self._lines(dtr=self.dtr, rts=rts)

    def _lines(self, *, dtr: bool, rts: bool) -> None:
        try:
            payload = self._client.usb_serial_lines(
                self._testbed_id, self.token, dtr, rts
            )
        except LabAgentError as exc:
            raise translate_error(exc) from exc
        self.dtr = bool(payload.get("dtr", dtr))
        self.rts = bool(payload.get("rts", rts))

    def close(self) -> None:
        """Close the session; closing an already closed one is not an error."""
        if self._closed:
            return
        try:
            self._client.usb_serial_close(self._testbed_id, self.token)
        except LabAgentError as exc:
            # 404 is the contract's idempotent case: the agent has already
            # dropped the session, which is the state the caller asked for.
            translated = translate_error(exc)
            if not isinstance(translated, UsbNotFound):
                raise translated from exc
        self._closed = True

    def __enter__(self) -> "UsbSerial":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


class UsbVolume:
    """A mounted volume of a USB device, read-only through the agent.

    Every read is bounded and path-checked on the agent's side. Nothing here
    writes: a test that changed the device's filesystem would be verifying a
    different run from the next one.
    """

    def __init__(
        self,
        client: LabAgentClient,
        testbed_id: str,
        info: VolumeInfo,
    ) -> None:
        self._client = client
        self._testbed_id = testbed_id
        self.drive = info.drive
        self.label = info.label
        self.filesystem = info.filesystem
        self.size = info.size
        self.removable = info.removable
        self.device_key = info.device_key or ""

    def exists(self, path: str) -> bool:
        try:
            self.read_bytes(path)
        except UsbNotFound:
            return False
        return True

    def read_bytes(self, path: str) -> bytes:
        """Return the file's exact bytes.

        Bytes are transferred base64-encoded: a text round trip would make the
        comparison depend on the agent's code page.
        """
        payload = self._request(path, "base64")
        encoded = payload.get("data")
        if not isinstance(encoded, str):
            raise UsbHostError(
                f"the Lab Agent returned no data for {path!r} on {self.drive}"
            )
        try:
            return base64.b64decode(encoded, validate=True)
        except (ValueError, binascii.Error) as exc:
            raise UsbHostError(
                f"the Lab Agent returned undecodable data for {path!r}: {exc}"
            ) from exc

    def read_text(
        self,
        path: str,
        *,
        encoding: str = "utf-8",
        errors: str = "replace",
    ) -> str:
        """Return the file as text.

        The agent normally answers the text form directly. When it answers
        base64 instead -- the same payload the bytes path uses -- the local
        ``encoding`` and ``errors`` decide how the bytes are read.
        """
        payload = self._request(path, "text")
        data = payload.get("data")
        if isinstance(data, str) and payload.get("encoding") != "base64":
            return data
        if isinstance(data, str):
            try:
                raw = base64.b64decode(data, validate=True)
            except (binascii.Error, ValueError) as exc:
                # Only the wire encoding is the agent's fault. A
                # UnicodeDecodeError below is the caller's charset choice and
                # must stay a UnicodeDecodeError. ValueError covers a non-ASCII
                # string, which the validator rejects before decoding.
                raise UsbHostError(
                    f"the Lab Agent returned undecodable text for {path!r}: "
                    f"{exc}"
                ) from exc
            return raw.decode(encoding, errors)
        if isinstance(data, bytes):
            return data.decode(encoding, errors)
        raise UsbHostError(
            f"the Lab Agent returned no text for {path!r} on {self.drive}"
        )

    def _request(self, path: str, wire_encoding: str) -> Dict[str, Any]:
        try:
            return self._client.usb_file(
                self._testbed_id, self.device_key, path, wire_encoding
            )
        except LabAgentError as exc:
            raise translate_error(exc) from exc

    def __repr__(self) -> str:  # pragma: no cover - diagnostics only
        return f"UsbVolume(drive={self.drive!r}, label={self.label!r})"


__all__ = [
    "AudioEndpoint",
    "AudioEndpoints",
    "ComPort",
    "HidInfo",
    "UsbConflict",
    "UsbDenied",
    "UsbDevice",
    "UsbHost",
    "UsbHostCapabilities",
    "UsbHostError",
    "UsbHostTimeout",
    "UsbHostUnavailable",
    "UsbInterface",
    "UsbNotFound",
    "UsbSerial",
    "UsbVolume",
    "VolumeInfo",
    "translate_error",
]
