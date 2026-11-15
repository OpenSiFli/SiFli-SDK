# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""ctypes bindings for the Windows APIs USB observation needs.

Only bindings and structure layouts live here: SetupAPI, CfgMgr32, the kernel's
device IOCTLs and volume queries, and hid.dll. The decisions -- which nodes form
one physical device, which pipe is an audio endpoint, which volume belongs to
which device -- belong to :mod:`lab_agent.usb_host.windows`, where they can be
reasoned about and, in part, unit-tested without Windows.

The module imports on any platform so the package stays importable there; every
function that actually calls into Windows raises first. The DLLs are loaded on
first use rather than at import, for the same reason.
"""

from __future__ import annotations

import ctypes
import os
import struct
from ctypes import wintypes
from typing import Any
from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Optional
from typing import Tuple

_WINDOWS = os.name == "nt"

# -- constants --------------------------------------------------------------

DIGCF_PRESENT = 0x00000002
DIGCF_DEVICEINTERFACE = 0x00000010
DIGCF_ALLCLASSES = 0x00000004

SPDRP_DEVICEDESC = 0x00000000
SPDRP_COMPATIBLEIDS = 0x00000002
SPDRP_SERVICE = 0x00000004
SPDRP_MFG = 0x0000000B
SPDRP_FRIENDLYNAME = 0x0000000C
SPDRP_ADDRESS = 0x0000001C

REG_SZ = 1
REG_MULTI_SZ = 7
REG_DWORD = 4
REG_QWORD = 11
REG_BINARY = 3

CR_SUCCESS = 0

ERROR_NO_MORE_ITEMS = 259

GENERIC_READ = 0x80000000
GENERIC_WRITE = 0x40000000
FILE_SHARE_READ = 0x00000001
FILE_SHARE_WRITE = 0x00000002
OPEN_EXISTING = 3
INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value

DRIVE_UNKNOWN = 0
DRIVE_NO_ROOT_DIR = 1
DRIVE_REMOVABLE = 2
DRIVE_FIXED = 3
DRIVE_REMOTE = 4
DRIVE_CDROM = 5
DRIVE_RAMDISK = 6

#: ``CTL_CODE(FILE_DEVICE_USB, function, METHOD_BUFFERED, FILE_ANY_ACCESS)``
IOCTL_USB_GET_NODE_CONNECTION_INFORMATION_EX = 0x00220448
IOCTL_USB_GET_DESCRIPTOR_FROM_NODE_CONNECTION = 0x00220410
#: ``CTL_CODE(FILE_DEVICE_MASS_STORAGE, 0x420, METHOD_BUFFERED, FILE_ANY_ACCESS)``
IOCTL_STORAGE_GET_DEVICE_NUMBER = 0x002D1080

USB_REQUEST_GET_DESCRIPTOR = 0x06
USB_DESCRIPTOR_TYPE_CONFIGURATION = 0x02
USB_DESCRIPTOR_TYPE_HID_REPORT = 0x22
#: Device-to-host, standard, recipient = interface.
USB_REQUEST_INTERFACE_STANDARD_IN = 0x81

#: USB_DEVICE_SPEED values, as reported by the hub IOCTL.
USB_SPEEDS = {0: "low", 1: "full", 2: "high", 3: "super"}


class _Guid(ctypes.Structure):
    _fields_ = [
        ("Data1", wintypes.DWORD),
        ("Data2", wintypes.WORD),
        ("Data3", wintypes.WORD),
        ("Data4", ctypes.c_ubyte * 8),
    ]


def _guid(text: str) -> _Guid:
    value = _Guid()
    text = text.strip("{}")
    part1, part2, part3, part4, part5 = text.split("-")
    value.Data1 = int(part1, 16)
    value.Data2 = int(part2, 16)
    value.Data3 = int(part3, 16)
    rest = bytes.fromhex(part4 + part5)
    for index, byte in enumerate(rest):
        value.Data4[index] = byte
    return value


GUID_DEVINTERFACE_USB_HUB = _guid("f18a0e88-c30c-11d0-8815-00a0c906bed8")
GUID_DEVINTERFACE_VOLUME = _guid("53f5630d-b6bf-11d0-94f2-00a0c91efb8b")
GUID_DEVINTERFACE_DISK = _guid("53f56307-b6bf-11d0-94f2-00a0c91efb8b")
GUID_DEVINTERFACE_HID = _guid("4d1e55b2-f16f-11cf-88cb-001111000030")
GUID_DEVCLASS_PORTS = _guid("4d36e978-e325-11ce-bfc1-08002be10318")


class SP_DEVINFO_DATA(ctypes.Structure):
    _fields_ = [
        ("cbSize", wintypes.DWORD),
        ("ClassGuid", _Guid),
        ("DevInst", wintypes.DWORD),
        ("Reserved", ctypes.c_size_t),
    ]


class SP_DEVICE_INTERFACE_DATA(ctypes.Structure):
    _fields_ = [
        ("cbSize", wintypes.DWORD),
        ("InterfaceClassGuid", _Guid),
        ("Flags", wintypes.DWORD),
        ("Reserved", ctypes.c_size_t),
    ]


class USB_DEVICE_DESCRIPTOR(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("bLength", ctypes.c_ubyte),
        ("bDescriptorType", ctypes.c_ubyte),
        ("bcdUSB", ctypes.c_ushort),
        ("bDeviceClass", ctypes.c_ubyte),
        ("bDeviceSubClass", ctypes.c_ubyte),
        ("bDeviceProtocol", ctypes.c_ubyte),
        ("bMaxPacketSize0", ctypes.c_ubyte),
        ("idVendor", ctypes.c_ushort),
        ("idProduct", ctypes.c_ushort),
        ("bcdDevice", ctypes.c_ushort),
        ("iManufacturer", ctypes.c_ubyte),
        ("iProduct", ctypes.c_ubyte),
        ("iSerialNumber", ctypes.c_ubyte),
        ("bNumConfigurations", ctypes.c_ubyte),
    ]


class USB_ENDPOINT_DESCRIPTOR(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("bLength", ctypes.c_ubyte),
        ("bDescriptorType", ctypes.c_ubyte),
        ("bEndpointAddress", ctypes.c_ubyte),
        ("bmAttributes", ctypes.c_ubyte),
        ("wMaxPacketSize", ctypes.c_ushort),
        ("bInterval", ctypes.c_ubyte),
    ]


class USB_PIPE_INFO(ctypes.Structure):
    # ``usbioctl.h`` wraps these structures in ``pshpack1.h``: a pipe is 11
    # bytes (7-byte endpoint descriptor followed by a 4-byte offset).
    _pack_ = 1
    _fields_ = [
        ("EndpointDescriptor", USB_ENDPOINT_DESCRIPTOR),
        ("ScheduleOffset", wintypes.DWORD),
    ]


class USB_NODE_CONNECTION_INFORMATION_EX(ctypes.Structure):
    # Packed for the same reason: the header is 35 bytes and ``PipeList``
    # starts immediately after ``ConnectionStatus``.
    _pack_ = 1
    _fields_ = [
        ("ConnectionIndex", wintypes.DWORD),
        ("DeviceDescriptor", USB_DEVICE_DESCRIPTOR),
        ("CurrentConfigurationValue", ctypes.c_ubyte),
        ("Speed", ctypes.c_ubyte),
        ("DeviceIsHub", ctypes.c_ubyte),
        ("DeviceAddress", ctypes.c_ushort),
        ("NumberOfOpenPipes", wintypes.DWORD),
        ("ConnectionStatus", wintypes.DWORD),
        ("PipeList", USB_PIPE_INFO * 1),
    ]


class USB_DESCRIPTOR_REQUEST(ctypes.Structure):
    _fields_ = [
        ("ConnectionIndex", wintypes.DWORD),
        ("bmRequestType", ctypes.c_ubyte),
        ("bRequest", ctypes.c_ubyte),
        ("wValue", ctypes.c_ushort),
        ("wIndex", ctypes.c_ushort),
        ("wLength", ctypes.c_ushort),
        ("Data", ctypes.c_ubyte * 1),
    ]


class STORAGE_DEVICE_NUMBER(ctypes.Structure):
    _fields_ = [
        ("DeviceType", wintypes.ULONG),
        ("DeviceNumber", wintypes.ULONG),
        ("PartitionNumber", wintypes.ULONG),
    ]


class HIDD_ATTRIBUTES(ctypes.Structure):
    _fields_ = [
        ("Size", wintypes.ULONG),
        ("VendorID", wintypes.USHORT),
        ("ProductID", wintypes.USHORT),
        ("VersionNumber", wintypes.USHORT),
    ]


class HIDP_CAPS(ctypes.Structure):
    _fields_ = [
        ("Usage", wintypes.USHORT),
        ("UsagePage", wintypes.USHORT),
        ("InputReportByteLength", wintypes.USHORT),
        ("OutputReportByteLength", wintypes.USHORT),
        ("FeatureReportByteLength", wintypes.USHORT),
        ("Reserved", wintypes.USHORT * 17),
        ("NumberLinkCollectionNodes", wintypes.USHORT),
        ("NumberInputButtonCaps", wintypes.USHORT),
        ("NumberInputValueCaps", wintypes.USHORT),
        ("NumberInputDataIndices", wintypes.USHORT),
        ("NumberOutputButtonCaps", wintypes.USHORT),
        ("NumberOutputValueCaps", wintypes.USHORT),
        ("NumberOutputDataIndices", wintypes.USHORT),
        ("NumberFeatureButtonCaps", wintypes.USHORT),
        ("NumberFeatureValueCaps", wintypes.USHORT),
        ("NumberFeatureDataIndices", wintypes.USHORT),
    ]


class DeviceNode(NamedTuple):
    """A PnP devnode plus the registry properties the capability reads.

    ``CM_Get_DevNode_RegistryPropertyW`` is not exported by every cfgmgr32 build
    (this repository's Windows 11 image lacks it), while
    ``SetupDiGetDeviceRegistryPropertyW`` is available wherever SetupAPI is.
    Reading the properties while the enumeration handle is open also avoids a
    second query per value.
    """

    devinst: int
    instance_id: str
    description: Optional[str]
    manufacturer: Optional[str]
    friendly_name: Optional[str]
    address: Optional[int]
    service: Optional[str]
    compatible_ids: Tuple[str, ...]


class InterfaceNode(NamedTuple):
    """A device-interface path plus the devnode that exposes it."""

    devinst: int
    instance_id: str
    path: str


# -- lazily loaded DLLs -----------------------------------------------------

_setupapi = None
_cfgmgr32 = None
_kernel32 = None
_hid = None


def _require_windows() -> None:
    if not _WINDOWS:
        raise OSError("these bindings are only available on Windows")


def _pointer_size() -> int:
    return ctypes.sizeof(ctypes.c_void_p)


def setupapi():
    global _setupapi
    _require_windows()
    if _setupapi is not None:
        return _setupapi
    lib = ctypes.WinDLL("setupapi", use_last_error=True)
    lib.SetupDiGetClassDevsW.restype = wintypes.HANDLE
    lib.SetupDiGetClassDevsW.argtypes = [
        ctypes.POINTER(_Guid),
        wintypes.LPCWSTR,
        wintypes.HWND,
        wintypes.DWORD,
    ]
    lib.SetupDiEnumDeviceInfo.restype = wintypes.BOOL
    lib.SetupDiEnumDeviceInfo.argtypes = [
        wintypes.HANDLE,
        wintypes.DWORD,
        ctypes.POINTER(SP_DEVINFO_DATA),
    ]
    lib.SetupDiEnumDeviceInterfaces.restype = wintypes.BOOL
    lib.SetupDiEnumDeviceInterfaces.argtypes = [
        wintypes.HANDLE,
        ctypes.POINTER(SP_DEVINFO_DATA),
        ctypes.POINTER(_Guid),
        wintypes.DWORD,
        ctypes.POINTER(SP_DEVICE_INTERFACE_DATA),
    ]
    lib.SetupDiGetDeviceInterfaceDetailW.restype = wintypes.BOOL
    lib.SetupDiGetDeviceInterfaceDetailW.argtypes = [
        wintypes.HANDLE,
        ctypes.POINTER(SP_DEVICE_INTERFACE_DATA),
        ctypes.c_void_p,
        wintypes.DWORD,
        ctypes.POINTER(wintypes.DWORD),
        ctypes.POINTER(SP_DEVINFO_DATA),
    ]
    lib.SetupDiGetDeviceRegistryPropertyW.restype = wintypes.BOOL
    lib.SetupDiGetDeviceRegistryPropertyW.argtypes = [
        wintypes.HANDLE,
        ctypes.POINTER(SP_DEVINFO_DATA),
        wintypes.DWORD,
        ctypes.POINTER(wintypes.DWORD),
        ctypes.c_void_p,
        wintypes.DWORD,
        ctypes.POINTER(wintypes.DWORD),
    ]
    lib.SetupDiDestroyDeviceInfoList.restype = wintypes.BOOL
    lib.SetupDiDestroyDeviceInfoList.argtypes = [wintypes.HANDLE]
    _setupapi = lib
    return lib


def cfgmgr32():
    global _cfgmgr32
    _require_windows()
    if _cfgmgr32 is not None:
        return _cfgmgr32
    lib = ctypes.WinDLL("cfgmgr32", use_last_error=True)
    lib.CM_Get_Device_ID_Size.restype = wintypes.ULONG
    lib.CM_Get_Device_ID_Size.argtypes = [
        ctypes.POINTER(wintypes.ULONG),
        wintypes.DWORD,
        wintypes.ULONG,
    ]
    lib.CM_Get_Device_IDW.restype = wintypes.ULONG
    lib.CM_Get_Device_IDW.argtypes = [
        wintypes.DWORD,
        wintypes.LPWSTR,
        wintypes.ULONG,
        wintypes.ULONG,
    ]
    lib.CM_Get_Parent.restype = wintypes.ULONG
    lib.CM_Get_Parent.argtypes = [
        ctypes.POINTER(wintypes.DWORD),
        wintypes.DWORD,
        wintypes.ULONG,
    ]
    _cfgmgr32 = lib
    return lib


def kernel32():
    global _kernel32
    _require_windows()
    if _kernel32 is not None:
        return _kernel32
    lib = ctypes.WinDLL("kernel32", use_last_error=True)
    lib.CreateFileW.restype = wintypes.HANDLE
    lib.CreateFileW.argtypes = [
        wintypes.LPCWSTR,
        wintypes.DWORD,
        wintypes.DWORD,
        ctypes.c_void_p,
        wintypes.DWORD,
        wintypes.DWORD,
        wintypes.HANDLE,
    ]
    lib.CloseHandle.restype = wintypes.BOOL
    lib.CloseHandle.argtypes = [wintypes.HANDLE]
    lib.DeviceIoControl.restype = wintypes.BOOL
    lib.DeviceIoControl.argtypes = [
        wintypes.HANDLE,
        wintypes.DWORD,
        ctypes.c_void_p,
        wintypes.DWORD,
        ctypes.c_void_p,
        wintypes.DWORD,
        ctypes.POINTER(wintypes.DWORD),
        ctypes.c_void_p,
    ]
    lib.GetLogicalDrives.restype = wintypes.DWORD
    lib.GetLogicalDrives.argtypes = []
    lib.GetDriveTypeW.restype = wintypes.UINT
    lib.GetDriveTypeW.argtypes = [wintypes.LPCWSTR]
    lib.GetVolumeInformationW.restype = wintypes.BOOL
    lib.GetVolumeInformationW.argtypes = [
        wintypes.LPCWSTR,
        wintypes.LPWSTR,
        wintypes.DWORD,
        ctypes.POINTER(wintypes.DWORD),
        ctypes.POINTER(wintypes.DWORD),
        ctypes.POINTER(wintypes.DWORD),
        wintypes.LPWSTR,
        wintypes.DWORD,
    ]
    lib.GetDiskFreeSpaceExW.restype = wintypes.BOOL
    lib.GetDiskFreeSpaceExW.argtypes = [
        wintypes.LPCWSTR,
        ctypes.POINTER(ctypes.c_ulonglong),
        ctypes.POINTER(ctypes.c_ulonglong),
        ctypes.POINTER(ctypes.c_ulonglong),
    ]
    lib.GetVolumeNameForVolumeMountPointW.restype = wintypes.BOOL
    lib.GetVolumeNameForVolumeMountPointW.argtypes = [
        wintypes.LPCWSTR,
        wintypes.LPWSTR,
        wintypes.DWORD,
    ]
    lib.GetVolumePathNamesForVolumeNameW.restype = wintypes.BOOL
    lib.GetVolumePathNamesForVolumeNameW.argtypes = [
        wintypes.LPCWSTR,
        wintypes.LPWSTR,
        wintypes.DWORD,
        ctypes.POINTER(wintypes.DWORD),
    ]
    _kernel32 = lib
    return lib


def hid():
    global _hid
    _require_windows()
    if _hid is not None:
        return _hid
    lib = ctypes.WinDLL("hid", use_last_error=True)
    lib.HidD_GetAttributes.restype = wintypes.BOOL
    lib.HidD_GetAttributes.argtypes = [
        wintypes.HANDLE,
        ctypes.POINTER(HIDD_ATTRIBUTES),
    ]
    lib.HidD_GetPreparsedData.restype = wintypes.BOOL
    lib.HidD_GetPreparsedData.argtypes = [
        wintypes.HANDLE,
        ctypes.POINTER(ctypes.c_void_p),
    ]
    lib.HidD_FreePreparsedData.restype = wintypes.BOOL
    lib.HidD_FreePreparsedData.argtypes = [ctypes.c_void_p]
    lib.HidP_GetCaps.restype = wintypes.ULONG
    lib.HidP_GetCaps.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(HIDP_CAPS),
    ]
    lib.HidD_GetProductString.restype = wintypes.BOOL
    lib.HidD_GetProductString.argtypes = [
        wintypes.HANDLE,
        ctypes.c_void_p,
        wintypes.ULONG,
    ]
    _hid = lib
    return lib


# -- PnP ---------------------------------------------------------------------


def device_id(devinst: int) -> Optional[str]:
    size = wintypes.ULONG(0)
    if cfgmgr32().CM_Get_Device_ID_Size(ctypes.byref(size), devinst, 0):
        return None
    buffer = ctypes.create_unicode_buffer(size.value + 1)
    if cfgmgr32().CM_Get_Device_IDW(devinst, buffer, len(buffer), 0):
        return None
    return buffer.value or None


def parent_devinst(devinst: int) -> Optional[int]:
    parent = wintypes.DWORD(0)
    if cfgmgr32().CM_Get_Parent(ctypes.byref(parent), devinst, 0) != CR_SUCCESS:
        return None
    return int(parent.value) or None


def _registry_property(
    handle: int, info: SP_DEVINFO_DATA, prop: int
) -> Any:
    """One SPDRP_* value, decoded by its registry type."""
    setup = setupapi()
    reg_type = wintypes.DWORD(0)
    required = wintypes.DWORD(0)
    setup.SetupDiGetDeviceRegistryPropertyW(
        handle,
        ctypes.byref(info),
        prop,
        ctypes.byref(reg_type),
        None,
        0,
        ctypes.byref(required),
    )
    if required.value == 0:
        return None
    buffer = ctypes.create_string_buffer(required.value)
    if not setup.SetupDiGetDeviceRegistryPropertyW(
        handle,
        ctypes.byref(info),
        prop,
        ctypes.byref(reg_type),
        buffer,
        required.value,
        ctypes.byref(required),
    ):
        return None
    raw = buffer.raw[: required.value]
    if reg_type.value == REG_DWORD and len(raw) >= 4:
        return struct.unpack("<I", raw[:4])[0]
    if reg_type.value == REG_QWORD and len(raw) >= 8:
        return struct.unpack("<Q", raw[:8])[0]
    if reg_type.value == REG_BINARY:
        return raw
    text = raw.decode("utf-16-le", errors="replace")
    parts = [entry for entry in text.split("\x00") if entry]
    # SetupAPI reports COMPATIBLEIDS as REG_MULTI_SZ on some builds and as a
    # REG_SZ containing NUL-separated strings on others. A single-string
    # property (a description) has at most one part after trimming.
    if reg_type.value == REG_MULTI_SZ or len(parts) > 1:
        return parts
    return text.split("\x00", 1)[0]


def enumerate_nodes(guid: Optional[_Guid] = None) -> List[DeviceNode]:
    """Every present devnode of a class (all classes when ``guid`` is None)."""
    _require_windows()
    flags = DIGCF_PRESENT
    if guid is None:
        flags |= DIGCF_ALLCLASSES
    handle = setupapi().SetupDiGetClassDevsW(
        ctypes.byref(guid) if guid is not None else None, None, None, flags
    )
    if handle == INVALID_HANDLE_VALUE:
        raise ctypes.WinError(ctypes.get_last_error())
    nodes: List[DeviceNode] = []
    try:
        index = 0
        while True:
            info = SP_DEVINFO_DATA()
            info.cbSize = ctypes.sizeof(SP_DEVINFO_DATA)
            if not setupapi().SetupDiEnumDeviceInfo(handle, index, ctypes.byref(info)):
                break
            instance_id = device_id(int(info.DevInst))
            if instance_id:
                compatible = _registry_property(
                    handle, info, SPDRP_COMPATIBLEIDS
                )
                if isinstance(compatible, str):
                    compatible = (compatible,)
                nodes.append(
                    DeviceNode(
                        devinst=int(info.DevInst),
                        instance_id=instance_id,
                        description=_registry_property(
                            handle, info, SPDRP_DEVICEDESC
                        ),
                        manufacturer=_registry_property(handle, info, SPDRP_MFG),
                        friendly_name=_registry_property(
                            handle, info, SPDRP_FRIENDLYNAME
                        ),
                        address=_registry_property(handle, info, SPDRP_ADDRESS),
                        service=_registry_property(handle, info, SPDRP_SERVICE),
                        compatible_ids=tuple(compatible or ()),                    )
                )
            index += 1
    finally:
        setupapi().SetupDiDestroyDeviceInfoList(handle)
    return nodes


def enumerate_interface_paths(guid: _Guid) -> List[InterfaceNode]:
    """Every present device interface of a class, with its symbolic path."""
    _require_windows()
    handle = setupapi().SetupDiGetClassDevsW(
        ctypes.byref(guid), None, None, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE
    )
    if handle == INVALID_HANDLE_VALUE:
        raise ctypes.WinError(ctypes.get_last_error())
    results: List[InterfaceNode] = []
    try:
        index = 0
        while True:
            interface = SP_DEVICE_INTERFACE_DATA()
            interface.cbSize = ctypes.sizeof(SP_DEVICE_INTERFACE_DATA)
            if not setupapi().SetupDiEnumDeviceInterfaces(
                handle, None, ctypes.byref(guid), index, ctypes.byref(interface)
            ):
                break
            required = wintypes.DWORD(0)
            setupapi().SetupDiGetDeviceInterfaceDetailW(
                handle,
                ctypes.byref(interface),
                None,
                0,
                ctypes.byref(required),
                None,
            )
            if required.value:
                buffer = ctypes.create_string_buffer(required.value)
                # cbSize is 6 on x86 and 8 on x64; the structure is variable.
                ctypes.cast(buffer, ctypes.POINTER(wintypes.DWORD))[0] = (
                    6 if _pointer_size() == 4 else 8
                )
                info = SP_DEVINFO_DATA()
                info.cbSize = ctypes.sizeof(SP_DEVINFO_DATA)
                if setupapi().SetupDiGetDeviceInterfaceDetailW(
                    handle,
                    ctypes.byref(interface),
                    ctypes.cast(buffer, ctypes.c_void_p),
                    required.value,
                    ctypes.byref(required),
                    ctypes.byref(info),
                ):
                    path = ctypes.wstring_at(ctypes.addressof(buffer) + 4)
                    instance_id = device_id(int(info.DevInst)) or ""
                    results.append(
                        InterfaceNode(int(info.DevInst), instance_id, path)
                    )
            index += 1
    finally:
        setupapi().SetupDiDestroyDeviceInfoList(handle)
    return results


# -- hub IOCTL ---------------------------------------------------------------


def open_device(path: str, access: int = GENERIC_READ | GENERIC_WRITE) -> int:
    handle = kernel32().CreateFileW(
        path,
        access,
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        None,
        OPEN_EXISTING,
        0,
        None,
    )
    if handle == INVALID_HANDLE_VALUE:
        raise ctypes.WinError(ctypes.get_last_error())
    return handle


def close_handle(handle: int) -> None:
    try:
        kernel32().CloseHandle(handle)
    except Exception:
        pass


def _device_io_control(
    handle: int, code: int, buffer, size: int
) -> Tuple[bool, int]:
    returned = wintypes.DWORD(0)
    ok = kernel32().DeviceIoControl(
        handle,
        code,
        ctypes.cast(buffer, ctypes.c_void_p),
        size,
        ctypes.cast(buffer, ctypes.c_void_p),
        size,
        ctypes.byref(returned),
        None,
    )
    return bool(ok), int(returned.value)


def connection_information(
    hub_path: str, port: int, max_pipes: int = 32
) -> Optional[Dict[str, Any]]:
    """``USB_NODE_CONNECTION_INFORMATION_EX`` for one port of a hub."""
    struct_size = ctypes.sizeof(USB_NODE_CONNECTION_INFORMATION_EX)
    pipe_size = ctypes.sizeof(USB_PIPE_INFO)
    total = struct_size + max(0, max_pipes - 1) * pipe_size
    buffer = ctypes.create_string_buffer(total)
    structure = ctypes.cast(
        buffer, ctypes.POINTER(USB_NODE_CONNECTION_INFORMATION_EX)
    )
    structure.contents.ConnectionIndex = port
    try:
        handle = open_device(hub_path)
    except OSError:
        return None
    try:
        ok, returned = _device_io_control(
            handle,
            IOCTL_USB_GET_NODE_CONNECTION_INFORMATION_EX,
            buffer,
            total,
        )
        if not ok or returned < struct_size:
            return None
        count = min(int(structure.contents.NumberOfOpenPipes), max_pipes)
        offset = USB_NODE_CONNECTION_INFORMATION_EX.PipeList.offset
        pipes = []
        for index in range(count):
            item = ctypes.cast(
                ctypes.addressof(buffer) + offset + index * pipe_size,
                ctypes.POINTER(USB_PIPE_INFO),
            ).contents
            pipes.append(
                {
                    "address": int(item.EndpointDescriptor.bEndpointAddress),
                    "attributes": int(item.EndpointDescriptor.bmAttributes),
                }
            )
        return {
            "speed": USB_SPEEDS.get(int(structure.contents.Speed)),
            "pipes": pipes,
        }
    finally:
        close_handle(handle)


def _descriptor_from_hub(
    hub_path: str,
    port: int,
    request_type: int,
    value: int,
    index: int,
    length: int,
) -> Optional[bytes]:
    """Raw descriptor bytes from a device behind its parent hub."""
    # The flexible ``Data`` member starts at offset 12, not at ``sizeof``: the
    # ctypes structure rounds a ``UCHAR[1]`` up to a 4-byte tail, and using
    # ``sizeof`` would skip the first four descriptor bytes.
    header_size = USB_DESCRIPTOR_REQUEST.Data.offset
    total = header_size + length
    buffer = ctypes.create_string_buffer(total)
    request = ctypes.cast(buffer, ctypes.POINTER(USB_DESCRIPTOR_REQUEST))
    request.contents.ConnectionIndex = port
    request.contents.bmRequestType = request_type
    request.contents.bRequest = USB_REQUEST_GET_DESCRIPTOR
    request.contents.wValue = value
    request.contents.wIndex = index
    request.contents.wLength = length
    try:
        handle = open_device(hub_path)
    except OSError:
        return None
    try:
        ok, returned = _device_io_control(
            handle,
            IOCTL_USB_GET_DESCRIPTOR_FROM_NODE_CONNECTION,
            buffer,
            total,
        )
        if not ok or returned <= header_size:
            return None
        return ctypes.string_at(
            ctypes.addressof(buffer) + header_size, returned - header_size
        )
    finally:
        close_handle(handle)


def configuration_descriptor(
    hub_path: str, port: int, length: int = 4096
) -> Optional[bytes]:
    """The device's configuration descriptor, fetched through its parent hub."""
    data = _descriptor_from_hub(
        hub_path,
        port,
        0x80,
        USB_DESCRIPTOR_TYPE_CONFIGURATION << 8,
        0,
        length,
    )
    if data is None:
        return None
    if len(data) >= 4:
        total_length = struct.unpack("<H", data[2:4])[0]
        if 0 < total_length <= len(data):
            data = data[:total_length]
    return data


def hid_report_descriptor(
    hub_path: str, port: int, interface: int, length: int = 4096
) -> Optional[bytes]:
    """The HID report descriptor of one interface, or ``None`` if unreadable.

    **Best effort only.** On the Windows 11 stack this project was verified
    against, the hub ``IOCTL_USB_GET_DESCRIPTOR_FROM_NODE_CONNECTION`` answers
    ``ERROR_GEN_FAILURE`` for HID class/report requests (both ``0x81/0x21`` and
    ``0x81/0x22``); some USB stacks allow them. The primary usage source is
    therefore a zero-access ``HidP_GetCaps`` (see ``hid_capabilities``), this is
    an extra source where available, and the interface compatible ids are the
    last fallback. The parser in ``windows.py`` does not depend on where the
    bytes came from.
    """
    return _descriptor_from_hub(
        hub_path,
        port,
        USB_REQUEST_INTERFACE_STANDARD_IN,
        USB_DESCRIPTOR_TYPE_HID_REPORT << 8,
        interface,
        length,
    )


# -- HID ---------------------------------------------------------------------


def hid_capabilities(path: str) -> Optional[Dict[str, Any]]:
    """Attributes and top-level collection caps, or ``None`` if unreadable.

    The handle is opened with no desired access on purpose: Windows refuses
    ``GENERIC_READ``/``GENERIC_WRITE`` for an exclusive keyboard/mouse
    collection, while the HID metadata calls work on a zero-access handle. That
    is the difference between a real usage and a fallback guess.
    """
    try:
        handle = open_device(path, access=0)
    except OSError:
        return None
    try:
        attributes = HIDD_ATTRIBUTES()
        attributes.Size = ctypes.sizeof(HIDD_ATTRIBUTES)
        if not hid().HidD_GetAttributes(handle, ctypes.byref(attributes)):
            return None

        result: Dict[str, Any] = {
            "vid": int(attributes.VendorID),
            "pid": int(attributes.ProductID),
            "usage_page": 0,
            "usage": 0,
            "report_length": 0,
            "product": None,
        }
        preparsed = ctypes.c_void_p()
        if hid().HidD_GetPreparsedData(handle, ctypes.byref(preparsed)):
            try:
                caps = HIDP_CAPS()
                if hid().HidP_GetCaps(preparsed, ctypes.byref(caps)) == 0x110000:
                    result["usage_page"] = int(caps.UsagePage)
                    result["usage"] = int(caps.Usage)
                    result["report_length"] = int(caps.InputReportByteLength)
            finally:
                hid().HidD_FreePreparsedData(preparsed)

        product = ctypes.create_unicode_buffer(256)
        if hid().HidD_GetProductString(handle, product, ctypes.sizeof(product)):
            result["product"] = product.value or None
        return result
    finally:
        close_handle(handle)


# -- volumes -----------------------------------------------------------------


def drive_type(drive: str) -> int:
    """``GetDriveTypeW`` without the (possibly blocking) volume-name lookup."""
    root = drive if drive.endswith("\\") else drive + "\\"
    return int(kernel32().GetDriveTypeW(root))


def volume_information(drive: str) -> Dict[str, Any]:
    """Label, filesystem, total size and drive type for one mounted volume."""
    kernel = kernel32()
    root = drive if drive.endswith("\\") else drive + "\\"
    result: Dict[str, Any] = {
        "label": None,
        "filesystem": None,
        "size": None,
        "drive_type": int(kernel.GetDriveTypeW(root)),
    }
    label = ctypes.create_unicode_buffer(261)
    filesystem = ctypes.create_unicode_buffer(261)
    serial = wintypes.DWORD(0)
    max_component = wintypes.DWORD(0)
    flags = wintypes.DWORD(0)
    if kernel.GetVolumeInformationW(
        root,
        label,
        len(label),
        ctypes.byref(serial),
        ctypes.byref(max_component),
        ctypes.byref(flags),
        filesystem,
        len(filesystem),
    ):
        result["label"] = label.value or None
        result["filesystem"] = filesystem.value or None
    free = ctypes.c_ulonglong(0)
    total = ctypes.c_ulonglong(0)
    total_free = ctypes.c_ulonglong(0)
    if kernel.GetDiskFreeSpaceExW(
        root,
        ctypes.byref(free),
        ctypes.byref(total),
        ctypes.byref(total_free),
    ):
        result["size"] = int(total.value)
    return result


def logical_drives() -> List[str]:
    mask = kernel32().GetLogicalDrives()
    drives = []
    for index in range(26):
        if mask & (1 << index):
            drives.append(f"{chr(ord('A') + index)}:\\")
    return drives


def device_number(path: str) -> Optional[int]:
    """The underlying disk number of a volume or disk device path.

    ``\\\\.\\D:`` and the matching ``GUID_DEVINTERFACE_DISK`` path both answer
    the same number, which is what joins a drive letter to its USB disk devnode
    when the volume interface's parent chain does not reach the device.
    """
    handle = kernel32().CreateFileW(
        path, 0, FILE_SHARE_READ | FILE_SHARE_WRITE, None, OPEN_EXISTING, 0, None
    )
    if handle == INVALID_HANDLE_VALUE:
        return None
    try:
        data = STORAGE_DEVICE_NUMBER()
        returned = wintypes.DWORD(0)
        ok = kernel32().DeviceIoControl(
            handle,
            IOCTL_STORAGE_GET_DEVICE_NUMBER,
            None,
            0,
            ctypes.cast(ctypes.byref(data), ctypes.c_void_p),
            ctypes.sizeof(data),
            ctypes.byref(returned),
            None,
        )
        return int(data.DeviceNumber) if ok else None
    finally:
        close_handle(handle)


def device_port_name(instance_id: str) -> Optional[str]:
    """``PortName`` from the device's own Enum key, if it has one.

    The Ports class enumeration is the primary source; this is the fallback the
    design asks for when that enumeration misses a port. It is the precise form
    of ``HARDWARE\\DEVICEMAP\\SERIALCOMM``: the same name, keyed by the device
    instance instead of a kernel device path that cannot be mapped back.
    """
    import winreg

    path = (
        "SYSTEM\\CurrentControlSet\\Enum\\"
        + instance_id
        + "\\Device Parameters"
    )
    try:
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path) as key:
            value, _ = winreg.QueryValueEx(key, "PortName")
    except OSError:
        return None
    text = str(value or "").strip()
    return text or None


def volume_name_for_mount_point(drive: str) -> Optional[str]:
    root = drive if drive.endswith("\\") else drive + "\\"
    buffer = ctypes.create_unicode_buffer(261)
    if not kernel32().GetVolumeNameForVolumeMountPointW(
        root, buffer, len(buffer)
    ):
        return None
    return buffer.value or None


def volume_mount_points(volume_name: str) -> List[str]:
    buffer = ctypes.create_unicode_buffer(1024)
    length = wintypes.DWORD(0)
    if not kernel32().GetVolumePathNamesForVolumeNameW(
        volume_name, buffer, len(buffer), ctypes.byref(length)
    ):
        return []
    return [entry for entry in buffer[:].split("\x00") if entry]
