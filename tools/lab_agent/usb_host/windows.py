# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""The Windows backend: enumerate the host's USB devices, volumes and HID.

Windows binds a class driver to every interface, so the PnP tree is the only
view of a device that is guaranteed to be there. A physical device appears as
the devnode without ``&MI_xx`` in its instance id; the composite function
children (``USB\\VID_..&PID_..&MI_00\\...``) are interfaces of that one device.
The identity the SDK side matches on is therefore **VID + PID + iSerialNumber**,
where the serial is the last segment of the parent's instance id -- not the
``SPDRP_DEVICEDESC`` string, which is the INF description and not what the
firmware reports as its product string.

No PowerShell, WMI or subprocess is used anywhere: every query is a documented
ctypes call, which keeps this usable from a service account and from a machine
where those tools are locked down.
"""

from __future__ import annotations

import re
from typing import Any
from typing import Dict
from typing import Iterable
from typing import List
from typing import Mapping
from typing import Optional
from typing import Sequence
from typing import Tuple

from .base import AudioEndpoint
from .base import AudioEndpoints
from .base import ComPort
from .base import HidInfo
from .base import UsbDevice
from .base import UsbInterface
from .base import UsbVolume
from .base import host_description
from .errors import UsbHostUnsupported
from . import winapi

#: Feature names reported by ``/usb/capabilities``.
FEATURES = (
    "devices",
    "wait",
    "serial",
    "volumes",
    "files",
    "hid",
    "audio",
)

_USB_NODE = re.compile(r"^USB\\", re.IGNORECASE)
_MI_NUMBER = re.compile(r"&MI_([0-9A-Fa-f]{2})", re.IGNORECASE)
_VID = re.compile(r"VID_([0-9A-Fa-f]{4})", re.IGNORECASE)
_PID = re.compile(r"PID_([0-9A-Fa-f]{4})", re.IGNORECASE)
_COMPATIBLE = re.compile(
    r"^USB\\Class_([0-9A-Fa-f]{2})&SubClass_([0-9A-Fa-f]{2})"
    r"&Prot_([0-9A-Fa-f]{2})",
    re.IGNORECASE,
)
_COM_NAME = re.compile(r"\((COM\d+)\)", re.IGNORECASE)
#: A volume interface path encodes the USBSTOR disk instance after ``_??_``.
_USBSTOR_IN_VOLUME = re.compile(r"_usbstor#(.+?)#\{", re.IGNORECASE)
#: HID boot protocol declared by the USB interface class triplet.
_USB_HID_TRIPLET = re.compile(
    r"Class_03&SubClass_01&Prot_(0?[12])", re.IGNORECASE
)

_DEVICE_CLASS_NAMES = {
    0x01: "audio",
    0x02: "cdc",
    0x03: "hid",
    0x06: "mtp",
    0x07: "printer",
    0x08: "msc",
    0x09: "hub",
    0x0A: "cdc",
    0x0E: "video",
    0xE0: "wireless",
    0xFF: "vendor",
}


# -- pure helpers (unit-tested without Windows) ------------------------------


def parse_instance_id(instance_id: str) -> Dict[str, Any]:
    """VID, PID, serial and MI number from a PnP instance id."""
    vid = _VID.search(instance_id)
    pid = _PID.search(instance_id)
    number = _MI_NUMBER.search(instance_id)
    parts = instance_id.split("\\")
    return {
        "vid": int(vid.group(1), 16) if vid else None,
        "pid": int(pid.group(1), 16) if pid else None,
        "serial": parts[-1] if len(parts) > 1 else None,
        "interface_number": int(number.group(1), 16) if number else None,
    }


def usb_parent_key(instance_id: str) -> str:
    """The instance id with any ``&MI_xx`` removed from its hardware id.

    Only a fallback for a child whose real parent could not be resolved through
    CfgMgr32: the suffix after the last backslash differs between a composite
    parent and its children, so the hardware id is all that can be reconstructed.
    """
    parts = instance_id.split("\\")
    # ``parts[0]`` is the enumerator ("USB"); the hardware id carrying
    # ``&MI_xx`` is the next component.
    if len(parts) > 1:
        parts[1] = re.sub(r"&MI_[0-9A-Fa-f]{2}", "", parts[1], count=1)
    return "\\".join(parts)


def group_usb_nodes(
    records: Iterable[Mapping[str, Any]]
) -> Dict[str, List[Mapping[str, Any]]]:
    """Group PnP records under their physical parent.

    ``records`` carry ``instance_id`` and optionally ``parent_id``. The explicit
    parent wins for a record with ``&MI_xx`` -- which is what the real
    enumerator supplies and what makes composite function children land under
    the one device they belong to. A downstream device whose parent is a hub is
    *not* a child interface; it groups under itself. The hardware-id fallback
    exists only for a child whose parent was not enumerated.
    """
    records = list(records)
    known = {str(record["instance_id"]).upper() for record in records}
    groups: Dict[str, List[Mapping[str, Any]]] = {}
    for record in records:
        instance_id = str(record["instance_id"])
        parent = record.get("parent_id")
        # Only a composite function child belongs to its parent. A downstream
        # device plugged into a hub also names the hub as its parent, but it is
        # a device in its own right: attaching it would make the hub inherit
        # that device's interfaces and compatible ids.
        if (
            parent
            and str(parent).upper() in known
            and _MI_NUMBER.search(instance_id)
        ):
            key = str(parent)
        else:
            key = usb_parent_key(instance_id)
        groups.setdefault(key.upper(), []).append(record)
    return groups


def interface_name(interface_class: int, subclass: int, protocol: int) -> str:
    """A short name for an interface, used only for human-readable reports."""
    if interface_class == 0x02:
        return "cdc_acm"
    if interface_class == 0x0A:
        return "cdc_data"
    if interface_class == 0x01:
        if subclass == 0x01:
            return "audio_control"
        if subclass == 0x02:
            return "audio_streaming"
        return "audio"
    if interface_class == 0x03:
        return "hid"
    if interface_class == 0x08:
        return "msc"
    if interface_class == 0x06 and subclass == 0x01 and protocol == 0x01:
        return "mtp"
    if interface_class == 0x0E:
        return "video"
    if interface_class == 0x07:
        return "printer"
    if interface_class == 0xFF:
        return "vendor"
    return "class_{:02x}".format(interface_class)


def device_class_name(interface_class: int) -> str:
    return _DEVICE_CLASS_NAMES.get(interface_class, "other")


def infer_hid_protocol(usage_page: int, usage: int) -> int:
    """The boot protocol implied by a top-level collection."""
    if usage_page == 0x01 and usage == 0x06:
        return 1
    if usage_page == 0x01 and usage == 0x02:
        return 2
    return 0


def endpoint_label(address: int) -> str:
    return "0x{:02x}".format(address & 0xFF)


def parse_configuration(
    data: Optional[bytes],
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """Interfaces and endpoints of a USB configuration descriptor.

    Alternate settings after the first are skipped: the device JSON lists one
    entry per interface number, and an alternate is a different view of the same
    one rather than a second interface.
    """
    interfaces: List[Dict[str, Any]] = []
    endpoints: List[Dict[str, Any]] = []
    if not data:
        return interfaces, endpoints
    index = 0
    current = None
    while index + 2 <= len(data):
        length = data[index]
        kind = data[index + 1]
        if length < 2:
            break
        chunk = data[index : index + length]
        if len(chunk) < length:
            break
        if kind == 0x04 and length >= 9:
            alternate = chunk[3]
            current = {
                "number": chunk[2],
                "alternate": alternate,
                "interface_class": chunk[5],
                "subclass": chunk[6],
                "protocol": chunk[7],
            }
            if alternate == 0:
                interfaces.append(current)
        elif kind == 0x05 and length >= 7 and current is not None:
            endpoints.append(
                {
                    "interface": current["number"],
                    "address": chunk[2],
                    "attributes": chunk[3],
                }
            )
        index += length
    return interfaces, endpoints


def interfaces_from_compatible_ids(
    records: Iterable[Mapping[str, Any]]
) -> List[Dict[str, Any]]:
    """Interface entries inferred from ``USB\\Class_xx&SubClass_yy&Prot_zz``."""
    interfaces: List[Dict[str, Any]] = []
    seen = set()
    for record in records:
        number = record.get("interface_number") or 0
        for value in record.get("compatible_ids") or []:
            match = _COMPATIBLE.match(str(value))
            if not match:
                continue
            entry = (
                int(number),
                int(match.group(1), 16),
                int(match.group(2), 16),
                int(match.group(3), 16),
            )
            if entry in seen:
                continue
            seen.add(entry)
            interfaces.append(
                {
                    "number": entry[0],
                    "interface_class": entry[1],
                    "subclass": entry[2],
                    "protocol": entry[3],
                }
            )
    return interfaces


# -- the backend -------------------------------------------------------------


def _ancestors(devinst: int) -> Iterable[int]:
    seen = set()
    current = devinst
    while current and current not in seen:
        seen.add(current)
        yield current
        current = winapi.parent_devinst(current)


def _physical_for(
    devinst: int, physical: Mapping[str, Mapping[str, Any]]
) -> Optional[str]:
    """The physical device key an interface/volume/HID devnode belongs to.

    Only ``CM_Get_Parent`` can reconstruct a composite parent: the child's
    suffix after ``&MI_xx`` is a location, not the parent's iSerialNumber, so a
    hardware-id fallback would never match a real ``physical`` key. Walking the
    parent chain is therefore the whole rule, and the value returned is the
    physical record's own instance id so callers can index their maps.
    """
    for ancestor in _ancestors(devinst):
        instance_id = winapi.device_id(ancestor)
        if not instance_id:
            continue
        record = physical.get(instance_id.upper())
        if record is not None:
            return record["instance_id"]
    return None


def usbstor_instance_from_volume_path(path: str) -> Optional[str]:
    r"""The ``USBSTOR\\DISK...`` instance id encoded in a volume interface path.

    ``\\?\storage#volume#_??_usbstor#disk&...#9&5fc6598&0&2025010101&0#...``
    carries the disk instance; the volume devnode's own parent chain goes to
    ``ROOT\\VOLMGR``, so this is the only link back to the USB device on that
    Windows stack.
    """
    match = _USBSTOR_IN_VOLUME.search(path)
    if not match:
        return None
    return ("USBSTOR\\" + match.group(1).replace("#", "\\")).upper()


def _compatible_ids_for(
    devinst: int, info: Mapping[int, Any]
) -> Tuple[str, ...]:
    """The first non-empty compatible-id list on the devnode or an ancestor.

    A HID collection's own devnode often has none; the USB interface child
    (``...&MI_03``) above it carries ``USB\\Class_03&SubClass_01&Prot_01/02``.
    """
    for ancestor in _ancestors(devinst):
        node = info.get(ancestor)
        ids = getattr(node, "compatible_ids", ()) if node is not None else ()
        if ids:
            return tuple(str(entry) for entry in ids)
    return ()


def _first_mi(devinst: int) -> int:
    for ancestor in _ancestors(devinst):
        instance_id = winapi.device_id(ancestor) or ""
        parsed = parse_instance_id(instance_id)
        if parsed["interface_number"] is not None:
            return int(parsed["interface_number"])
    return 0


class WindowsUsbHost:
    """Enumerate the local Windows host's USB devices through SetupAPI."""

    supported = True
    platform_name = "win32"
    features = FEATURES

    def __init__(self, allow_fixed_drives: bool = False) -> None:
        self.allow_fixed_drives = bool(allow_fixed_drives)

    def capabilities(self) -> Dict[str, Any]:
        return {
            "supported": True,
            "platform": self.platform_name,
            "features": list(self.features),
            "host": host_description(),
            "reason": None,
        }

    # -- enumeration ---------------------------------------------------------

    def enumerate(self) -> List[UsbDevice]:
        if not winapi._WINDOWS:
            raise UsbHostUnsupported(
                "the Windows USB backend needs a Windows agent"
            )
        nodes = winapi.enumerate_nodes()
        records = self._usb_records(nodes)
        groups = group_usb_nodes(records)
        physical_records = [
            record
            for record in records
            if record["instance_id"].upper().startswith("USB\\")
            and "&MI_" not in record["instance_id"].upper()
            and "ROOT_HUB" not in record["instance_id"].upper()
        ]
        physical = {
            record["instance_id"].upper(): record for record in physical_records
        }

        ports = self._com_ports(physical, records)
        volumes = self._volumes(physical, nodes)
        hid = self._hid(physical, {node.devinst: node for node in nodes})

        devices: List[UsbDevice] = []
        for record in physical_records:
            instance_id = record["instance_id"]
            children = [
                entry
                for entry in groups.get(instance_id.upper(), [])
                if entry["instance_id"].upper() != instance_id.upper()
                and _MI_NUMBER.search(entry["instance_id"])
            ]
            descriptor, speed, bus_port, pipes = self._configuration(record)
            interfaces, endpoints = self._interfaces(
                record, children, descriptor
            )
            audio = self._audio(interfaces, endpoints, pipes)
            key = instance_id
            classes = _classes(interfaces)
            if _is_mtp(record, children):
                classes = ["mtp"] + [name for name in classes if name != "mtp"]
            parsed = parse_instance_id(instance_id)
            devices.append(
                UsbDevice(
                    key=key,
                    instance_id=instance_id,
                    vid=parsed["vid"],
                    pid=parsed["pid"],
                    serial=parsed["serial"],
                    product=record.get("product"),
                    manufacturer=record.get("manufacturer"),
                    device_class=classes[0] if classes else "other",
                    classes=tuple(classes),
                    interfaces=tuple(
                        UsbInterface(
                            number=entry["number"],
                            interface_class=entry["interface_class"],
                            subclass=entry["subclass"],
                            protocol=entry["protocol"],
                            name=interface_name(
                                entry["interface_class"],
                                entry["subclass"],
                                entry["protocol"],
                            ),
                        )
                        for entry in interfaces
                    ),
                    com_ports=tuple(ports.get(key, ())),
                    volumes=tuple(volumes.get(key, ())),
                    hid=tuple(hid.get(key, ())),
                    audio=audio,
                    speed=speed,
                    bus_port=bus_port,
                    present=True,
                )
            )
        devices.sort(key=lambda device: device.key.lower())
        return devices

    def volumes(self) -> List[UsbVolume]:
        if not winapi._WINDOWS:
            raise UsbHostUnsupported(
                "the Windows USB backend needs a Windows agent"
            )
        nodes = winapi.enumerate_nodes()
        records = self._usb_records(nodes)
        physical_records = [
            record
            for record in records
            if record["instance_id"].upper().startswith("USB\\")
            and "&MI_" not in record["instance_id"].upper()
            and "ROOT_HUB" not in record["instance_id"].upper()
        ]
        physical = {
            record["instance_id"].upper(): record for record in physical_records
        }
        by_device = self._volumes(physical, nodes)
        return [
            volume
            for entries in by_device.values()
            for volume in entries
        ]

    # -- pieces --------------------------------------------------------------

    def _usb_records(self, nodes: Sequence[Any]) -> List[Dict[str, Any]]:
        records: List[Dict[str, Any]] = []
        for node in nodes:
            if not _USB_NODE.match(node.instance_id):
                continue
            if "ROOT_HUB" in node.instance_id.upper():
                continue
            parent = winapi.parent_devinst(node.devinst)
            parent_id = winapi.device_id(parent) if parent else None
            records.append(
                {
                    "devinst": node.devinst,
                    "instance_id": node.instance_id,
                    "parent_id": parent_id,
                    "product": node.description,
                    "manufacturer": node.manufacturer,
                    "address": node.address,
                    "compatible_ids": list(node.compatible_ids),
                }
            )
        return records

    def _hub_for(self, record: Mapping[str, Any]) -> Tuple[Optional[str], int]:
        """The parent hub's interface path and the port this device sits on."""
        parent = winapi.parent_devinst(record["devinst"])
        parent_id = winapi.device_id(parent) if parent else None
        port = record.get("address")
        if not parent_id or not isinstance(port, int):
            return None, 0
        for node in winapi.enumerate_interface_paths(
            winapi.GUID_DEVINTERFACE_USB_HUB
        ):
            if node.instance_id.upper() == parent_id.upper():
                return node.path, port
        return None, 0

    def _configuration(
        self, record: Mapping[str, Any]
    ) -> Tuple[Optional[bytes], Optional[str], Optional[int], List[Dict[str, Any]]]:
        hub, port = self._hub_for(record)
        descriptor = None
        speed = None
        pipes: List[Dict[str, Any]] = []
        if hub and port:
            descriptor = winapi.configuration_descriptor(hub, port)
            info = winapi.connection_information(hub, port)
            if info is not None:
                speed = info.get("speed")
                pipes = list(info.get("pipes") or [])
        return descriptor, speed, record.get("address"), pipes

    def _interfaces(
        self,
        record: Mapping[str, Any],
        children: Sequence[Mapping[str, Any]],
        descriptor: Optional[bytes],
    ) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
        interfaces, endpoints = parse_configuration(descriptor)
        if interfaces:
            return interfaces, endpoints
        # No descriptor (the parent hub could not be queried): the compatible
        # ids still say which classes this function exposes.
        return interfaces_from_compatible_ids([record, *children]), []

    def _com_ports(
        self,
        physical: Mapping[str, Mapping[str, Any]],
        records: Sequence[Mapping[str, Any]] = (),
    ) -> Dict[str, List[ComPort]]:
        result: Dict[str, List[ComPort]] = {}
        for node in winapi.enumerate_nodes(winapi.GUID_DEVCLASS_PORTS):
            match = _COM_NAME.search(str(node.friendly_name or ""))
            if not match:
                continue
            key = _physical_for(node.devinst, physical)
            if key is None:
                continue
            result.setdefault(key, []).append(
                ComPort(port=match.group(1).upper(), driver=str(node.service or ""))
            )

        # Fallback for a CDC port the Ports class enumeration missed: the
        # device's own Enum key still names it.
        for record in records:
            port = winapi.device_port_name(record["instance_id"])
            if not port:
                continue
            owner = record["instance_id"]
            if owner.upper() not in physical:
                parent = record.get("parent_id")
                if not parent or str(parent).upper() not in physical:
                    continue
                owner = str(parent)
            known = {entry.port.upper() for entry in result.get(owner, ())}
            if port.upper() not in known:
                result.setdefault(owner, []).append(ComPort(port=port.upper()))
        return result

    def _volumes(
        self,
        physical: Mapping[str, Mapping[str, Any]],
        pnps: Sequence[Any],
    ) -> Dict[str, List[UsbVolume]]:
        by_instance = {node.instance_id.upper(): node for node in pnps}
        volume_nodes = {}
        for node in winapi.enumerate_interface_paths(
            winapi.GUID_DEVINTERFACE_VOLUME
        ):
            volume_nodes[node.path.rstrip("\\").upper()] = node

        # A drive letter is joined to its USB disk by the storage device number:
        # both ``\\.\D:`` and the disk interface path answer the same one. This
        # is the reliable link on a stack where the volume devnode's parent
        # chain stops at ROOT\VOLMGR. The map is built lazily -- opening every
        # disk handle is wasted work when the first two associations succeed.
        disks: Optional[Dict[int, Any]] = None

        def disk_map() -> Dict[int, Any]:
            nonlocal disks
            if disks is None:
                disks = {}
                for node in winapi.enumerate_interface_paths(
                    winapi.GUID_DEVINTERFACE_DISK
                ):
                    number = winapi.device_number(node.path)
                    if number is not None:
                        disks[number] = node
            return disks

        result: Dict[str, List[UsbVolume]] = {}
        for drive in winapi.logical_drives():
            # Ask the cheap question first. ``GetVolumeNameForVolumeMountPointW``
            # blocks for seconds on a disconnected network drive, which is why
            # it is only reached for a drive type this testbed may read.
            kind = winapi.drive_type(drive)
            if kind != winapi.DRIVE_REMOVABLE and not (
                kind == winapi.DRIVE_FIXED and self.allow_fixed_drives
            ):
                continue
            volume_name = winapi.volume_name_for_mount_point(drive)
            if not volume_name:
                continue
            key = None
            node = volume_nodes.get(volume_name.rstrip("\\").upper())
            if node is not None:
                key = _physical_for(node.devinst, physical)
                if key is None:
                    # The volume devnode's parent chain can stop at
                    # ROOT\VOLMGR; its interface path still names the USBSTOR
                    # disk, whose own chain does reach the USB device.
                    encoded = usbstor_instance_from_volume_path(
                        node.path
                    ) or usbstor_instance_from_volume_path(node.instance_id)
                    disk = by_instance.get(encoded) if encoded else None
                    if disk is not None:
                        key = _physical_for(disk.devinst, physical)
            if key is None:
                number = winapi.device_number("\\\\.\\" + drive.rstrip("\\"))
                disk = disk_map().get(number) if number is not None else None
                if disk is not None:
                    key = _physical_for(disk.devinst, physical)
            if key is None:
                continue
            info = winapi.volume_information(drive)
            removable = info.get("drive_type") == winapi.DRIVE_REMOVABLE
            if not removable and not self.allow_fixed_drives:
                continue
            result.setdefault(key, []).append(
                UsbVolume(
                    drive=drive.rstrip("\\"),
                    label=info.get("label"),
                    filesystem=info.get("filesystem"),
                    size=info.get("size"),
                    removable=removable,
                    device_key=key,
                )
            )
        return result

    def _hid(
        self,
        physical: Mapping[str, Mapping[str, Any]],
        info: Mapping[int, Any],
    ) -> Dict[str, List[HidInfo]]:
        result: Dict[str, List[HidInfo]] = {}
        for node in winapi.enumerate_interface_paths(
            winapi.GUID_DEVINTERFACE_HID
        ):
            key = _physical_for(node.devinst, physical)
            if key is None:
                continue
            interface = _first_mi(node.devinst)
            caps = winapi.hid_capabilities(node.path)
            # A collection that cannot be opened, or whose report descriptor
            # yields no top-level usage, still has a USB interface class
            # triplet to fall back on. ``hid_capabilities`` reports zero rather
            # than None in the second case, so both must be checked.
            compatible = _compatible_ids_for(node.devinst, info)
            usage_page, usage = hid_usage(caps, ())
            if usage_page == 0 and usage == 0:
                # ``HidP_GetCaps`` is the primary source, but only a
                # zero-access handle reaches a keyboard/mouse collection.
                # Reading the report descriptor through the parent hub is a
                # best-effort second source: the Windows 11 stack this project
                # was verified against answers ERROR_GEN_FAILURE for HID
                # class/report requests, while some stacks allow it. Compatible
                # ids remain the last fallback.
                record = physical.get(key.upper())
                hub, port = (
                    self._hub_for(record) if record is not None else (None, 0)
                )
                if hub and port:
                    parsed = parse_hid_report_usage(
                        winapi.hid_report_descriptor(hub, port, interface)
                    )
                    if parsed is not None:
                        usage_page, usage = parsed
            if usage_page == 0 and usage == 0:
                usage_page, usage = _boot_usage(compatible)
            result.setdefault(key, []).append(
                HidInfo(
                    interface=interface,
                    usage_page=usage_page,
                    usage=usage,
                    protocol=infer_hid_protocol(usage_page, usage),
                    report_length=int(caps.get("report_length", 0))
                    if caps
                    else 0,
                    product=caps.get("product") if caps else None,
                )
            )
        return result

    def _audio(
        self,
        interfaces: Sequence[Mapping[str, Any]],
        endpoints: Sequence[Mapping[str, Any]],
        pipes: Sequence[Mapping[str, Any]] = (),
    ) -> Optional[AudioEndpoints]:
        audio_interfaces = [
            entry for entry in interfaces if entry["interface_class"] == 0x01
        ]
        if not audio_interfaces:
            return None
        inputs: List[AudioEndpoint] = []
        outputs: List[AudioEndpoint] = []
        audio_numbers = {entry["number"] for entry in audio_interfaces}
        for endpoint in endpoints:
            if int(endpoint.get("attributes", 0)) & 0x03 != 0x01:
                continue
            if endpoint.get("interface") not in audio_numbers:
                continue
            address = int(endpoint.get("address", 0))
            entry = AudioEndpoint(
                interface=int(endpoint.get("interface", 0)),
                endpoint=endpoint_label(address),
                direction="in" if address & 0x80 else "out",
            )
            (inputs if entry.direction == "in" else outputs).append(entry)
        if not inputs and not outputs:
            # No configuration descriptor (the parent hub could not be
            # queried). The hub IOCTL still lists the open pipes; isochronous
            # ones are the audio streams. Without interface numbers here, the
            # interface is reported as 0.
            for pipe in pipes:
                if int(pipe.get("attributes", 0)) & 0x03 != 0x01:
                    continue
                address = int(pipe.get("address", 0))
                entry = AudioEndpoint(
                    interface=0,
                    endpoint=endpoint_label(address),
                    direction="in" if address & 0x80 else "out",
                )
                (inputs if entry.direction == "in" else outputs).append(entry)
        return AudioEndpoints(inputs=tuple(inputs), outputs=tuple(outputs))


def _classes(interfaces: Sequence[Mapping[str, Any]]) -> List[str]:
    classes: List[str] = []
    for entry in interfaces:
        name = device_class_name(int(entry["interface_class"]))
        if name not in classes:
            classes.append(name)
    return classes


def _is_mtp(
    record: Mapping[str, Any], children: Sequence[Mapping[str, Any]]
) -> bool:
    for node in (record, *children):
        for value in node.get("compatible_ids") or ():
            if "MS_COMP_MTP" in str(value).upper():
                return True
    return False


def parse_hid_report_usage(data: Optional[bytes]) -> Optional[Tuple[int, int]]:
    """The first (Usage Page, Usage) pair in a HID report descriptor.

    Short items only: ``bSize`` is the low two bits, ``bType`` the next two and
    ``bTag`` the top four. Usage Page is a Global item (type 1, tag 0) and
    Usage a Local one (type 2, tag 0). Long items (``0xFE``) are skipped by
    their declared length. Anything truncated or without a complete pair is
    ``None``, which lets the caller fall back to the interface class.
    """
    if not data:
        return None
    index = 0
    page = None
    usage = None
    length = len(data)
    while index < length:
        prefix = data[index]
        if prefix == 0xFE:
            if index + 2 >= length:
                return None
            index += 3 + data[index + 1]
            continue
        size_code = prefix & 0x03
        item_type = (prefix >> 2) & 0x03
        tag = (prefix >> 4) & 0x0F
        size = 4 if size_code == 3 else size_code
        start = index + 1
        end = start + size
        if end > length:
            return None
        value = int.from_bytes(data[start:end], "little") if size else 0
        index = end
        if item_type == 1 and tag == 0:
            if page is None:
                page = value
        elif item_type == 2 and tag == 0:
            if usage is None:
                usage = value
        if page is not None and usage is not None:
            return page, usage
    return None


def hid_usage(
    caps: Optional[Mapping[str, Any]], compatible_ids: Sequence[Any]
) -> Tuple[int, int]:
    """The top-level usage, falling back to the USB interface class triplet.

    Windows binds the HID driver and may refuse to open a keyboard collection
    for a non-elevated process; ``hid_capabilities`` then returns None, or a
    caps dict whose usage is still 0. The interface's compatible ids always
    carry ``Class_03&SubClass_01&Prot_01/02`` (boot keyboard/mouse), which is
    enough to report the usage the design asserts.
    """
    usage_page = int(caps.get("usage_page", 0)) if caps else 0
    usage = int(caps.get("usage", 0)) if caps else 0
    if usage_page == 0 and usage == 0:
        return _boot_usage(compatible_ids)
    return usage_page, usage


def _boot_usage(compatible_ids: Sequence[Any]) -> Tuple[int, int]:
    text = " ".join(str(entry) for entry in compatible_ids).upper()
    if "HID_DEVICE_SYSTEM_KEYBOARD" in text:
        return 0x01, 0x06
    if "HID_DEVICE_SYSTEM_MOUSE" in text:
        return 0x01, 0x02
    # The HID devnode's compatible ids include the USB interface triplet too.
    match = _USB_HID_TRIPLET.search(text)
    if match:
        return (0x01, 0x06) if match.group(1).lstrip("0") == "1" else (0x01, 0x02)
    return 0, 0
