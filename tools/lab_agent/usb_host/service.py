# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""The per-testbed facade the HTTP layer talks to.

One :class:`UsbHostService` exists per allocated testbed. It owns the selected
backend, the options from the binding's ``usb:`` block, a short enumeration
cache and every open serial session. Everything the routes need is here, so
the server never reaches into a backend or a session directly.
"""

from __future__ import annotations

import base64
import os
import sys
import time
from datetime import datetime
from datetime import timezone
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Tuple

from .base import USB_HOST_FIXTURE
from .base import UsbDevice
from .base import UsbHostOptions
from .base import UsbVolume
from .base import host_description
from .base import match_device
from .base import match_keys
from .base import parse_usb_options
from .base import since_keys
from .errors import UsbHostBadRequest
from .errors import UsbHostDeviceNotFound
from .errors import UsbHostHidUnavailable
from .errors import UsbHostPathDenied
from .errors import UsbHostPortForbidden
from .errors import UsbHostPortNotUsb
from .errors import UsbHostTimeout
from .errors import UsbHostVolumeNotMounted
from .null import UnsupportedUsbHost
from .serial_sessions import MAX_READ_TIMEOUT
from .serial_sessions import SerialSessionRegistry
from .volume_paths import read_volume_file

#: How long one enumeration answers later requests before it is repeated.
#: SetupAPI is not free, and a suite checks devices and then opens a port.
DEFAULT_CACHE_TTL = 0.5

#: Default wait parameters, matching the endpoint table.
DEFAULT_WAIT_TIMEOUT = 15.0
DEFAULT_WAIT_INTERVAL = 0.25
MAX_WAIT_TIMEOUT = 300.0
MIN_WAIT_INTERVAL = 0.01


class UsbHostService:
    """Everything the USB routes of one testbed may do."""

    def __init__(
        self,
        backend,
        options: Optional[UsbHostOptions] = None,
        *,
        forbidden_ports: Iterable[str] = (),
        serial_factory=None,
        cache_ttl: float = DEFAULT_CACHE_TTL,
    ) -> None:
        self.backend = backend
        self.options = options or UsbHostOptions()
        self._forbidden = {
            str(port).upper() for port in forbidden_ports if port
        } | {
            str(port).upper()
            for port in self.options.exclude_ports
            if port
        }
        if serial_factory is None:
            self.sessions = SerialSessionRegistry()
        else:
            self.sessions = SerialSessionRegistry(factory=serial_factory)
        self._cache_ttl = cache_ttl
        self._cached_devices: Optional[List[UsbDevice]] = None
        self._cached_at = 0.0

    # -- capabilities --------------------------------------------------------

    def capabilities(self) -> Dict[str, Any]:
        """What this backend can do. Always a 200, supported or not."""
        payload = dict(self.backend.capabilities())
        payload.setdefault("host", host_description())
        payload.setdefault("reason", None)
        return payload

    # -- enumeration ---------------------------------------------------------

    def _devices(self, *, refresh: bool = False) -> List[UsbDevice]:
        now = time.monotonic()
        if (
            not refresh
            and self._cached_devices is not None
            and now - self._cached_at <= self._cache_ttl
        ):
            return list(self._cached_devices)
        devices = list(self.backend.enumerate())
        self._cached_devices = devices
        # Stamp after the enumeration: stamping before it would make the age
        # include the enumeration's own cost, and a slow enumeration would
        # never be reusable.
        self._cached_at = time.monotonic()
        return devices

    def devices(self) -> List[UsbDevice]:
        return self._devices()

    def devices_payload(self) -> Dict[str, Any]:
        return {
            "captured_at": datetime.now(timezone.utc).isoformat(),
            "devices": [device.to_dict() for device in self._devices()],
        }

    def device(self, device_key: str) -> UsbDevice:
        wanted = str(device_key or "").lower()
        for device in self._devices():
            if device.key.lower() == wanted:
                return device
        raise UsbHostDeviceNotFound(
            f'no USB device with key "{device_key}"'
        )

    def wait(
        self,
        match: Any = None,
        *,
        since: Any = None,
        timeout: float = DEFAULT_WAIT_TIMEOUT,
        interval: float = DEFAULT_WAIT_INTERVAL,
    ) -> Tuple[UsbDevice, float]:
        """Wait for a matching device the caller has not seen yet.

        ``since`` is the list of device keys the caller already knows about; a
        device whose key is in it never satisfies the wait, which is what makes
        "the device appeared after I started looking" expressible without the
        caller holding a timestamp this side cannot compare.
        """
        match = match_keys(match)
        seen = {key.lower() for key in since_keys(since)}
        timeout = _clamp_float(
            timeout, DEFAULT_WAIT_TIMEOUT, 0.0, MAX_WAIT_TIMEOUT
        )
        interval = _clamp_float(
            interval, DEFAULT_WAIT_INTERVAL, MIN_WAIT_INTERVAL, MAX_WAIT_TIMEOUT
        )

        started = time.monotonic()
        deadline = started + timeout
        candidates: List[UsbDevice] = []
        while True:
            devices = self._devices(refresh=True)
            candidates = [
                device for device in devices if match_device(device, match)
            ]
            for device in candidates:
                if device.key.lower() not in seen:
                    return device, time.monotonic() - started
            if time.monotonic() >= deadline:
                break
            time.sleep(min(interval, max(0.0, deadline - time.monotonic())))

        raise UsbHostTimeout(
            f"no matching USB device appeared within {timeout:g}s",
            details={
                "candidates": [device.to_dict() for device in candidates],
                "since": list(seen),
            },
        )

    # -- serial --------------------------------------------------------------

    def open_serial(
        self,
        device_key: str,
        com_port: Optional[str] = None,
        *,
        baud: Optional[int] = None,
        dtr: bool = False,
        rts: bool = False,
    ) -> Dict[str, Any]:
        device = self.device(device_key)
        ports = [entry.port for entry in device.com_ports]
        if not com_port:
            if not ports:
                raise UsbHostPortNotUsb(
                    f'device "{device.key}" has no USB CDC serial port'
                )
            com_port = ports[0]
        wanted = str(com_port)
        # The reservation is checked before the port's parentage: a console or
        # explicitly excluded port must be refused as such, even if the device
        # happens to expose it, and the negative test promises that answer.
        if wanted.upper() in self._forbidden:
            raise UsbHostPortForbidden(
                f"{wanted} is reserved: the DUT console, an instrument console "
                'or an "exclude_ports" entry'
            )
        if wanted.upper() not in {port.upper() for port in ports}:
            raise UsbHostPortNotUsb(
                f'{wanted} is not a COM port of device "{device.key}" '
                f"(it offers: {', '.join(ports) or 'none'})"
            )

        speed = int(baud if baud is not None else self.options.serial_baud)
        if speed <= 0:
            raise UsbHostBadRequest("baud must be a positive integer")
        token, stream = self.sessions.open(
            wanted, speed, dtr=bool(dtr), rts=bool(rts)
        )
        return {
            "token": token,
            "port": stream.port,
            "device_key": device.key,
            "baud": stream.baud,
            "dtr": stream.dtr,
            "rts": stream.rts,
        }

    def read_serial(
        self, token: str, cursor: int = 0, timeout: float = 1.0
    ) -> Dict[str, Any]:
        timeout = _clamp_float(timeout, 1.0, 0.0, MAX_READ_TIMEOUT)
        data, new_cursor, closed = self.sessions.get(token).read_from(
            cursor, timeout
        )
        return {
            "data": base64.b64encode(data).decode("ascii"),
            "cursor": new_cursor,
            "closed": closed,
        }

    def write_serial(self, token: str, data: bytes) -> Dict[str, Any]:
        written = self.sessions.get(token).write(data)
        return {"written": written}

    def set_lines(
        self,
        token: str,
        dtr: Optional[bool] = None,
        rts: Optional[bool] = None,
    ) -> Dict[str, Any]:
        stream = self.sessions.get(token)
        stream.set_lines(dtr, rts)
        return {"dtr": stream.dtr, "rts": stream.rts}

    def close_serial(self, token: str) -> Dict[str, Any]:
        self.sessions.close(token)
        return {"closed": True}

    def close_all(self) -> None:
        self.sessions.close_all()

    # -- volumes and files ---------------------------------------------------

    def volumes(self) -> List[UsbVolume]:
        return list(self.backend.volumes())

    def volumes_payload(self) -> Dict[str, Any]:
        return {"volumes": [volume.to_dict() for volume in self.volumes()]}

    def read_file(
        self, device_key: str, path: str, encoding: str = "text"
    ) -> Dict[str, Any]:
        if not path:
            raise UsbHostPathDenied("the path is empty", details={"path": path})
        if encoding not in ("text", "base64"):
            raise UsbHostBadRequest(
                f'unknown encoding "{encoding}" (use "text" or "base64")'
            )
        device = self.device(device_key)
        volumes = [
            volume
            for volume in self.volumes()
            if (volume.device_key or "").lower() == device.key.lower()
        ]
        if not volumes:
            raise UsbHostVolumeNotMounted(
                f'device "{device.key}" has no mounted volume'
            )
        volume = volumes[0]
        data = read_volume_file(
            volume,
            path,
            max_bytes=self.options.max_file_bytes,
            allow_fixed_drives=self.options.allow_fixed_drives,
        )
        if encoding == "text":
            payload = data.decode("utf-8", errors="replace")
        else:
            payload = base64.b64encode(data).decode("ascii")
        return {
            "device_key": device.key,
            "path": path,
            "size": len(data),
            "encoding": encoding,
            "data": payload,
        }

    # -- audio and HID -------------------------------------------------------

    def audio_payload(self, device_key: str) -> Dict[str, Any]:
        device = self.device(device_key)
        audio = device.audio
        payload = {
            "device_key": device.key,
            "inputs": [],
            "outputs": [],
        }
        if audio is not None:
            payload["inputs"] = [entry.to_dict() for entry in audio.inputs]
            payload["outputs"] = [entry.to_dict() for entry in audio.outputs]
        return payload

    def hid_payload(self, device_key: str) -> Dict[str, Any]:
        device = self.device(device_key)
        if not device.hid and any(
            entry.interface_class == 0x03 for entry in device.interfaces
        ):
            raise UsbHostHidUnavailable(
                f'device "{device.key}" has HID interfaces but no HID '
                "collection is bound (is the HID driver loaded?)"
            )
        return {
            "device_key": device.key,
            "interfaces": [entry.to_dict() for entry in device.hid],
        }


def _clamp_float(value: Any, default: float, low: float, high: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise UsbHostBadRequest(f"expected a number, got {value!r}") from exc
    return max(low, min(high, number))


# -- construction -----------------------------------------------------------


def usb_host_connection(testbed):
    """The testbed's ``usb_host`` connection, if the binding declares one."""
    for connection in testbed.connections:
        if connection.fixture.name == USB_HOST_FIXTURE:
            return connection
    return None


def forbidden_ports(testbed) -> List[str]:
    """Ports this testbed reserves: its consoles and instrument consoles.

    The sftool download port is the console port (see
    :meth:`TestBedRuntime.flash`), so it is covered here without a second rule.
    """
    ports: List[str] = []
    if testbed.console is not None:
        ports.append(str(testbed.console.port))
    for connection in testbed.connections:
        options = connection.options.get("console")
        if isinstance(options, dict) and options.get("port"):
            ports.append(str(options["port"]))
    return ports


def _build_backend(options: UsbHostOptions):
    if options.backend == "null":
        return UnsupportedUsbHost("backend is null")
    if os.name == "nt":
        from .windows import WindowsUsbHost

        return WindowsUsbHost(allow_fixed_drives=options.allow_fixed_drives)
    return UnsupportedUsbHost(
        f"USB host observation needs a Windows agent (this one runs {sys.platform})"
    )


def create_service(testbed, *, serial_factory=None) -> UsbHostService:
    """Build the service for one testbed from its binding entry.

    A testbed that does not declare ``usb_host`` still gets a service -- the
    allocation gate refuses its routes -- so a caller that already passed the
    gate cannot be surprised by a missing object. The options come from the
    connection's ``usb:`` block, which the config loader has already validated.
    """
    connection = usb_host_connection(testbed)
    where = f'testbed "{testbed.id}" fixture "{USB_HOST_FIXTURE}"'
    options = parse_usb_options(
        None if connection is None else connection.options.get("usb"),
        where=where,
    )
    return UsbHostService(
        _build_backend(options),
        options,
        forbidden_ports=forbidden_ports(testbed),
        serial_factory=serial_factory,
    )
