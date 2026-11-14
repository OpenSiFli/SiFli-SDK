# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""The USB host-observation data model, options and match rules.

Everything here is backend independent: a fake backend in the self-tests yields
the same :class:`UsbDevice` objects a Windows backend does, and the HTTP layer
serialises them with the same ``to_dict`` methods. Keeping the model and the
match semantics out of ``windows.py`` is what lets the whole capability be
exercised on a machine with no USB hardware.
"""

from __future__ import annotations

import platform
import socket
from dataclasses import dataclass
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List
from typing import Mapping
from typing import Optional
from typing import Protocol
from typing import Sequence
from typing import Tuple

from ..errors import ConfigError
from .errors import UsbHostBadMatchKey

#: The fixture name a binding/testcase uses for this capability.
USB_HOST_FIXTURE = "usb_host"

#: Keys accepted by ``POST /usb/wait``. Anything else is a 400.
MATCH_KEYS = frozenset(
    {
        "vid",
        "pid",
        "serial",
        "product_contains",
        "device_class",
        "class",
        "interface_class",
        "usage_page",
        "usage",
        "protocol",
        "com_port",
        "volume_label",
        "require_com",
        "require_volume",
        "speed",
        "present",
    }
)

#: The options a ``usb:`` block may carry, and their defaults.
USB_OPTION_KEYS = (
    "backend",
    "exclude_ports",
    "allow_fixed_drives",
    "max_file_bytes",
    "serial_baud",
)

DEFAULT_MAX_FILE_BYTES = 4 * 1024 * 1024
DEFAULT_SERIAL_BAUD = 115200


def host_description() -> Dict[str, str]:
    """The machine the agent runs on, for capability reporting.

    ``name`` is deliberately the constant ``local``: the SDK-side client records
    it to tell "the agent's own USB bus" from a future remote-host backend.
    ``machine`` is the actual host, so a run's report names the bench it used.
    """
    return {
        "name": "local",
        "machine": socket.gethostname() or platform.node() or "unknown",
    }


@dataclass(frozen=True)
class UsbInterface:
    """One interface of a USB configuration, as the device describes itself."""

    number: int
    interface_class: int
    subclass: int = 0
    protocol: int = 0
    name: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "number": int(self.number),
            "class": int(self.interface_class),
            "subclass": int(self.subclass),
            "protocol": int(self.protocol),
            "name": self.name,
        }


@dataclass(frozen=True)
class ComPort:
    """A COM port Windows bound to the USB device's CDC interface."""

    port: str
    driver: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {"port": self.port, "driver": self.driver}


@dataclass(frozen=True)
class UsbVolume:
    """A mounted volume the device owns."""

    drive: str
    label: Optional[str] = None
    filesystem: Optional[str] = None
    size: Optional[int] = None
    removable: bool = True
    device_key: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "device_key": self.device_key,
            "drive": self.drive,
            "label": self.label,
            "filesystem": self.filesystem,
            "size": self.size,
            "removable": bool(self.removable),
        }


@dataclass(frozen=True)
class HidInfo:
    """One HID collection exposed by the device."""

    interface: int
    usage_page: int
    usage: int
    protocol: int = 0
    report_length: int = 0
    product: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "interface": int(self.interface),
            "usage_page": int(self.usage_page),
            "usage": int(self.usage),
            "protocol": int(self.protocol),
            "report_length": int(self.report_length),
            "product": self.product,
        }


@dataclass(frozen=True)
class AudioEndpoint:
    """One isochronous endpoint of an audio interface."""

    interface: int
    endpoint: str
    direction: str

    def to_dict(self) -> Dict[str, Any]:
        return {
            "interface": int(self.interface),
            "endpoint": self.endpoint,
            "direction": self.direction,
        }


@dataclass(frozen=True)
class AudioEndpoints:
    """Capture (``in``) and render (``out``) endpoints kept apart."""

    inputs: Tuple[AudioEndpoint, ...] = ()
    outputs: Tuple[AudioEndpoint, ...] = ()

    def to_dict(self) -> Dict[str, Any]:
        return {
            "inputs": [endpoint.to_dict() for endpoint in self.inputs],
            "outputs": [endpoint.to_dict() for endpoint in self.outputs],
        }


@dataclass(frozen=True)
class UsbDevice:
    """One physical USB device, with its interfaces grouped under it."""

    key: str
    instance_id: Optional[str] = None
    vid: Optional[int] = None
    pid: Optional[int] = None
    serial: Optional[str] = None
    product: Optional[str] = None
    manufacturer: Optional[str] = None
    device_class: str = "other"
    classes: Tuple[str, ...] = ()
    interfaces: Tuple[UsbInterface, ...] = ()
    com_ports: Tuple[ComPort, ...] = ()
    volumes: Tuple[UsbVolume, ...] = ()
    hid: Tuple[HidInfo, ...] = ()
    audio: Optional[AudioEndpoints] = None
    speed: Optional[str] = None
    bus_port: Optional[int] = None
    present: bool = True

    def to_dict(self) -> Dict[str, Any]:
        return {
            "key": self.key,
            "instance_id": self.instance_id or self.key,
            "vid": self.vid,
            "pid": self.pid,
            "serial": self.serial,
            "product": self.product,
            "manufacturer": self.manufacturer,
            "device_class": self.device_class,
            "classes": list(self.classes),
            "interfaces": [entry.to_dict() for entry in self.interfaces],
            "com_ports": [entry.to_dict() for entry in self.com_ports],
            "volumes": [entry.to_dict() for entry in self.volumes],
            "hid": [entry.to_dict() for entry in self.hid],
            "audio": self.audio.to_dict() if self.audio is not None else None,
            "speed": self.speed,
            "bus_port": self.bus_port,
            "present": bool(self.present),
        }


@dataclass(frozen=True)
class UsbHostOptions:
    """The ``usb:`` block of a testbed's ``usb_host`` connection."""

    backend: str = "auto"
    exclude_ports: Tuple[str, ...] = ()
    allow_fixed_drives: bool = False
    max_file_bytes: int = DEFAULT_MAX_FILE_BYTES
    serial_baud: int = DEFAULT_SERIAL_BAUD


def parse_usb_options(
    value: Any, where: str = '"usb"'
) -> UsbHostOptions:
    """Validate a ``usb:`` mapping, or say which key is wrong.

    Called when the runner config is loaded, so a typo is refused at startup
    rather than surfacing as a 500 halfway through a nightly run. The returned
    object is the only shape the rest of the capability reads.
    """
    if value is None:
        return UsbHostOptions()
    if not isinstance(value, dict):
        raise ConfigError(f"{where} must be a mapping")

    unknown = set(value) - set(USB_OPTION_KEYS)
    if unknown:
        raise ConfigError(
            f"{where}: unknown option(s): {', '.join(sorted(unknown))} "
            f"(known: {', '.join(USB_OPTION_KEYS)})"
        )

    # Unquoted ``backend: null`` in YAML arrives as Python ``None``. It names
    # the null backend, not "use the default", because that is the only reason
    # the word appears in a binding.
    raw_backend = value.get("backend", "auto")
    backend = "null" if raw_backend is None else str(raw_backend).lower()
    if backend not in ("auto", "windows", "null"):
        raise ConfigError(
            f'{where}.backend must be "auto", "windows" or "null"'
        )

    exclude = value.get("exclude_ports", ())
    if isinstance(exclude, str):
        exclude = [exclude]
    if not isinstance(exclude, (list, tuple)):
        raise ConfigError(f"{where}.exclude_ports must be a list of port names")
    exclude_ports = tuple(str(entry) for entry in exclude)

    if not isinstance(value.get("allow_fixed_drives", False), bool):
        raise ConfigError(
            f"{where}.allow_fixed_drives must be true or false"
        )

    max_file_bytes = value.get("max_file_bytes", DEFAULT_MAX_FILE_BYTES)
    if (
        isinstance(max_file_bytes, bool)
        or not isinstance(max_file_bytes, int)
        or max_file_bytes <= 0
    ):
        raise ConfigError(f"{where}.max_file_bytes must be a positive integer")

    serial_baud = value.get("serial_baud", DEFAULT_SERIAL_BAUD)
    if (
        isinstance(serial_baud, bool)
        or not isinstance(serial_baud, int)
        or serial_baud <= 0
    ):
        raise ConfigError(f"{where}.serial_baud must be a positive integer")

    return UsbHostOptions(
        backend=backend,
        exclude_ports=exclude_ports,
        allow_fixed_drives=bool(value.get("allow_fixed_drives", False)),
        max_file_bytes=max_file_bytes,
        serial_baud=serial_baud,
    )


class UsbHostBackend(Protocol):
    """What a platform backend must provide. No I/O lives above this line."""

    supported: bool
    platform_name: str
    features: Sequence[str]

    def capabilities(self) -> Dict[str, Any]:
        ...

    def enumerate(self) -> List[UsbDevice]:
        ...

    def volumes(self) -> List[UsbVolume]:
        ...


# -- matching ---------------------------------------------------------------


def _as_int(value: Any, key: str) -> int:
    if isinstance(value, bool):
        raise UsbHostBadMatchKey(f'Match key "{key}" needs an integer')
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        try:
            return int(value.strip(), 0)
        except ValueError as exc:
            raise UsbHostBadMatchKey(
                f'Match key "{key}" needs an integer, got {value!r}'
            ) from exc
    raise UsbHostBadMatchKey(f'Match key "{key}" needs an integer')


def _as_bool(value: Any, key: str) -> bool:
    if not isinstance(value, bool):
        raise UsbHostBadMatchKey(f'Match key "{key}" needs true or false')
    return value


def match_device(device: UsbDevice, match: Mapping[str, Any]) -> bool:
    """True when ``device`` satisfies every key in ``match``.

    Unknown keys raise :class:`UsbHostBadMatchKey` rather than being ignored:
    silently ignoring a misspelled key would turn ``wait`` into "any device",
    which is the kind of green result that hides a broken suite.
    """
    for key, expected in match.items():
        if key not in MATCH_KEYS:
            raise UsbHostBadMatchKey(
                f'Unknown match key "{key}" '
                f"(known: {', '.join(sorted(MATCH_KEYS))})"
            )

        if key == "vid":
            if device.vid != _as_int(expected, key):
                return False
        elif key == "pid":
            if device.pid != _as_int(expected, key):
                return False
        elif key == "serial":
            if device.serial != str(expected):
                return False
        elif key == "product_contains":
            needle = str(expected).lower()
            if needle not in (device.product or "").lower():
                return False
        elif key == "device_class":
            if device.device_class != str(expected):
                return False
        elif key in ("class", "interface_class"):
            wanted = _as_int(expected, key)
            if not any(
                entry.interface_class == wanted for entry in device.interfaces
            ):
                return False
        elif key in ("usage_page", "usage", "protocol"):
            wanted = _as_int(expected, key)
            attribute = key
            if not any(
                getattr(entry, attribute) == wanted for entry in device.hid
            ):
                return False
        elif key == "com_port":
            wanted = str(expected).upper()
            if not any(
                entry.port.upper() == wanted for entry in device.com_ports
            ):
                return False
        elif key == "volume_label":
            wanted = str(expected).lower()
            if not any(
                (entry.label or "").lower() == wanted for entry in device.volumes
            ):
                return False
        elif key == "require_com":
            if bool(device.com_ports) != _as_bool(expected, key):
                return False
        elif key == "require_volume":
            if bool(device.volumes) != _as_bool(expected, key):
                return False
        elif key == "speed":
            if (device.speed or "").lower() != str(expected).lower():
                return False
        elif key == "present":
            if bool(device.present) != _as_bool(expected, key):
                return False
    return True


def match_keys(match: Any) -> Mapping[str, Any]:
    """Validate the ``match`` body of a wait request before it is used."""
    if match is None:
        return {}
    if not isinstance(match, Mapping):
        raise UsbHostBadMatchKey('"match" must be a mapping of match keys')
    return match


def since_keys(value: Any) -> Tuple[str, ...]:
    """Validate the ``since`` list: device keys already observed by the caller."""
    if value is None:
        return ()
    if isinstance(value, str):
        return (value,)
    if isinstance(value, Iterable):
        return tuple(str(entry) for entry in value)
    raise UsbHostBadMatchKey('"since" must be a list of device keys')
