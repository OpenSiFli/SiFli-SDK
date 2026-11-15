# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Self-tests for the USB host-observation capability.

No USB hardware is involved: the backend is a fake that yields the same
:class:`~lab_agent.usb_host.base.UsbDevice` objects the Windows backend builds,
and the serial port is a fake ``pyserial`` handle. Everything above those two
seams -- the match rules, wait semantics, session registry, cursor arithmetic,
path safety and the JSON shapes -- is the production code.
"""

from __future__ import annotations

import os
import threading
import time
import unittest
from types import SimpleNamespace
from unittest import mock

from _support import LabTestCase

from lab_agent.testbed import TestBedRuntime
from lab_agent.usb_host import UnsupportedUsbHost
from lab_agent.usb_host import winapi
from lab_agent.usb_host.base import AudioEndpoint
from lab_agent.usb_host.base import AudioEndpoints
from lab_agent.usb_host.base import ComPort
from lab_agent.usb_host.base import HidInfo
from lab_agent.usb_host.base import UsbDevice
from lab_agent.usb_host.base import UsbHostOptions
from lab_agent.usb_host.base import UsbInterface
from lab_agent.usb_host.base import UsbVolume
from lab_agent.usb_host.base import match_device
from lab_agent.usb_host.base import parse_usb_options
from lab_agent.usb_host.base import since_keys
from lab_agent.usb_host.errors import UsbHostBadMatchKey
from lab_agent.usb_host.errors import UsbHostDeviceNotFound
from lab_agent.usb_host.errors import UsbHostFileNotFound
from lab_agent.usb_host.errors import UsbHostFileTooLarge
from lab_agent.usb_host.errors import UsbHostHidUnavailable
from lab_agent.usb_host.errors import UsbHostNotAllocated
from lab_agent.usb_host.errors import UsbHostPathDenied
from lab_agent.usb_host.errors import UsbHostPortBusy
from lab_agent.usb_host.errors import UsbHostPortClosed
from lab_agent.usb_host.errors import UsbHostPortDenied
from lab_agent.usb_host.errors import UsbHostPortForbidden
from lab_agent.usb_host.errors import UsbHostPortNotUsb
from lab_agent.usb_host.errors import UsbHostTimeout
from lab_agent.usb_host.errors import UsbHostTokenNotFound
from lab_agent.usb_host.errors import UsbHostUnsupported
from lab_agent.usb_host.errors import UsbHostVolumeNotMounted
from lab_agent.usb_host.serial_sessions import ByteStream
from lab_agent.usb_host.service import UsbHostService
from lab_agent.usb_host.service import create_service
from lab_agent.usb_host.service import forbidden_ports
from lab_agent.usb_host.volume_paths import read_volume_file
from lab_agent.usb_host.volume_paths import validate_relative_path
from lab_agent.usb_host.windows import WindowsUsbHost
from lab_agent.usb_host.windows import _compatible_ids_for
from lab_agent.usb_host.windows import _is_mtp
from lab_agent.usb_host.windows import _physical_for
from lab_agent.usb_host.windows import group_usb_nodes
from lab_agent.usb_host.windows import hid_usage
from lab_agent.usb_host.windows import infer_hid_protocol
from lab_agent.usb_host.windows import interface_name
from lab_agent.usb_host.windows import parse_configuration
from lab_agent.usb_host.windows import parse_hid_report_usage
from lab_agent.usb_host.windows import parse_instance_id
from lab_agent.usb_host.windows import usb_parent_key
from lab_agent.usb_host.windows import usbstor_instance_from_volume_path
from lab_agent.errors import ConfigError
from sdk_test.model import Connection
from sdk_test.model import FixtureRef
from sdk_test.model import TestBed

#: A device resembling the CDC ACM example: one physical node, one COM port.
CDC_DEVICE = UsbDevice(
    key="USB\\VID_38F4&PID_FFFF\\2022123456",
    vid=0x38F4,
    pid=0xFFFF,
    serial="2022123456",
    product="USB Serial Device",
    device_class="cdc",
    classes=("cdc",),
    interfaces=(
        UsbInterface(0, 0x02, 0x02, 0x01, "cdc_acm"),
        UsbInterface(1, 0x0A, 0x00, 0x00, "cdc_data"),
    ),
    com_ports=(ComPort("COM12", "usbser"),),
    speed="full",
    bus_port=1,
)


class FakeBackend:
    """A backend whose answers the test controls entirely."""

    supported = True
    platform_name = "test"
    features = ("devices", "wait", "serial", "volumes", "files", "hid", "audio")

    def __init__(self, devices=(), volumes=(), sequence=None, delay=0.0):
        self.devices = list(devices)
        self.volume_list = list(volumes)
        self._sequence = [list(batch) for batch in (sequence or [])]
        self.enumerations = 0
        self.delay = delay

    def capabilities(self):
        return {
            "supported": True,
            "platform": self.platform_name,
            "features": list(self.features),
            "host": {"name": "local", "machine": "fake"},
            "reason": None,
        }

    def enumerate(self):
        self.enumerations += 1
        if self.delay:
            time.sleep(self.delay)
        if self._sequence:
            return list(self._sequence.pop(0))
        return list(self.devices)

    def volumes(self):
        return list(self.volume_list)


class FakeSerial:
    """Stands in for ``serial.Serial``; records modem lines and writes."""

    def __init__(self, chunks=()):
        self._chunks = list(chunks)
        self.writes = []
        self._dtr = False
        self._rts = False
        self.closed = 0
        self.fail_reads = False

    @property
    def dtr(self):
        return self._dtr

    @dtr.setter
    def dtr(self, value):
        self._dtr = bool(value)

    @property
    def rts(self):
        return self._rts

    @rts.setter
    def rts(self, value):
        self._rts = bool(value)

    def read(self, size=4096):
        if self.fail_reads:
            raise OSError("simulated unplug")
        if self._chunks:
            return self._chunks.pop(0)
        time.sleep(0.005)
        return b""

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def close(self):
        self.closed += 1


class FakeSerialFactory:
    """One fake handle per port, so a test can inspect what was opened."""

    def __init__(self, chunks=()):
        self.chunks = chunks
        self.opened = {}

    def __call__(self, port, baud):
        handle = FakeSerial(self.chunks)
        self.opened[port.upper()] = handle
        return handle


def a_device(**overrides):
    values = {
        "key": "USB\\VID_1234&PID_5678\\SER001",
        "vid": 0x1234,
        "pid": 0x5678,
        "serial": "SER001",
        "product": "Test Device",
        "device_class": "cdc",
        "classes": ("cdc",),
    }
    values.update(overrides)
    return UsbDevice(**values)


class TestOptions(LabTestCase):
    def test_defaults_match_the_design(self):
        options = parse_usb_options(None)
        self.assertEqual(options.backend, "auto")
        self.assertEqual(options.exclude_ports, ())
        self.assertFalse(options.allow_fixed_drives)
        self.assertEqual(options.max_file_bytes, 4 * 1024 * 1024)
        self.assertEqual(options.serial_baud, 115200)

    def test_an_unknown_option_is_refused_with_the_known_ones(self):
        with self.assertRaises(ConfigError) as caught:
            parse_usb_options({"backends": "auto"})
        message = str(caught.exception)
        self.assertIn("backends", message)
        self.assertIn("backend", message)

    def test_an_unknown_backend_is_refused(self):
        with self.assertRaises(ConfigError):
            parse_usb_options({"backend": "linux"})

    def test_exclude_ports_accepts_one_string(self):
        options = parse_usb_options({"exclude_ports": "COM8"})
        self.assertEqual(options.exclude_ports, ("COM8",))

    def test_booleans_must_be_booleans(self):
        with self.assertRaises(ConfigError):
            parse_usb_options({"allow_fixed_drives": "yes"})

    def test_sizes_must_be_positive_integers(self):
        with self.assertRaises(ConfigError):
            parse_usb_options({"max_file_bytes": 0})
        with self.assertRaises(ConfigError):
            parse_usb_options({"serial_baud": "fast"})


class TestNullBackend(LabTestCase):
    def test_capabilities_say_unsupported_rather_than_failing(self):
        service = UsbHostService(UnsupportedUsbHost("no backend here"))
        payload = service.capabilities()
        self.assertFalse(payload["supported"])
        self.assertEqual(payload["features"], [])
        self.assertEqual(payload["reason"], "no backend here")

    def test_every_other_call_is_the_unsupported_error(self):
        service = UsbHostService(UnsupportedUsbHost("no backend here"))
        with self.assertRaises(UsbHostUnsupported) as caught:
            service.devices_payload()
        self.assertEqual(caught.exception.status, 501)
        self.assertEqual(caught.exception.code, "usb_host_unsupported")

    def test_create_service_honours_backend_null(self):
        testbed = TestBed(
            id="tb",
            platform="board",
            connections=[
                Connection(
                    FixtureRef("usb_host"), options={"usb": {"backend": "null"}}
                )
            ],
        )
        service = create_service(testbed)
        self.assertFalse(service.capabilities()["supported"])


class TestInstanceIdsAndGrouping(LabTestCase):
    def test_instance_id_parts(self):
        parsed = parse_instance_id("USB\\VID_38F4&PID_FFFF&MI_00\\6&1A2B3C&0&0000")
        self.assertEqual(parsed["vid"], 0x38F4)
        self.assertEqual(parsed["pid"], 0xFFFF)
        self.assertEqual(parsed["interface_number"], 0)
        self.assertEqual(parsed["serial"], "6&1A2B3C&0&0000")

    def test_mi_children_group_under_their_parent(self):
        parent = "USB\\VID_38F4&PID_FFFF\\2022123456"
        records = [
            {"instance_id": parent, "parent_id": "USB\\ROOT_HUB30\\4&1"},
            {
                "instance_id": "USB\\VID_38F4&PID_FFFF&MI_00\\6&1a2b3c&0&0000",
                "parent_id": parent,
            },
            {
                "instance_id": "USB\\VID_38F4&PID_FFFF&MI_01\\6&1a2b3c&0&0001",
                "parent_id": parent,
            },
        ]
        groups = group_usb_nodes(records)
        self.assertEqual(len(groups[parent.upper()]), 3)
        self.assertEqual(len(groups), 1)

    def test_a_child_without_an_enumerated_parent_falls_back_to_the_hardware_id(self):
        child = "USB\\VID_1234&PID_5678&MI_02\\7&ABCD&0&0002"
        self.assertEqual(
            usb_parent_key(child), "USB\\VID_1234&PID_5678\\7&ABCD&0&0002"
        )
        groups = group_usb_nodes([{"instance_id": child}])
        self.assertIn(usb_parent_key(child).upper(), groups)

    def test_configuration_descriptor_parsing(self):
        # A CDC device: configuration, interface 0 (comm), one interrupt IN
        # endpoint, interface 1 (data), one bulk OUT and one bulk IN.
        data = bytes(
            [
                9, 2, 67, 0, 2, 1, 0, 0x80, 50,
                9, 4, 0, 0, 1, 2, 2, 1, 0,
                7, 5, 0x81, 3, 8, 0, 10,
                9, 4, 1, 0, 2, 10, 0, 0, 0,
                7, 5, 0x02, 2, 64, 0, 0,
                7, 5, 0x82, 2, 64, 0, 0,
            ]
        )
        interfaces, endpoints = parse_configuration(data)
        self.assertEqual([entry["number"] for entry in interfaces], [0, 1])
        self.assertEqual(
            [entry["address"] for entry in endpoints], [0x81, 0x02, 0x82]
        )
        self.assertEqual(interface_name(2, 2, 1), "cdc_acm")
        self.assertEqual(interface_name(10, 0, 0), "cdc_data")

    def test_hid_protocol_from_usage(self):
        self.assertEqual(infer_hid_protocol(1, 6), 1)
        self.assertEqual(infer_hid_protocol(1, 2), 2)
        self.assertEqual(infer_hid_protocol(0x0C, 1), 0)


class TestWinapiConstants(LabTestCase):
    def test_spdrp_values_match_the_windows_headers(self):
        # These two were wrong once: SPDRP_SERVICE at 0x05 is UNUSED1 (so the
        # COM driver was always empty) and 0x0D is LOCATION_INFORMATION (so the
        # compatible-id fallback, MTP detection and HID boot fallback all read
        # "Port_#....Hub_#..." instead of USB\Class_xx).
        self.assertEqual(winapi.SPDRP_COMPATIBLEIDS, 0x02)
        self.assertEqual(winapi.SPDRP_SERVICE, 0x04)
        self.assertEqual(winapi.SPDRP_DEVICEDESC, 0x00)
        self.assertEqual(winapi.SPDRP_MFG, 0x0B)
        self.assertEqual(winapi.SPDRP_FRIENDLYNAME, 0x0C)
        self.assertEqual(winapi.SPDRP_ADDRESS, 0x1C)


class TestHubGrouping(LabTestCase):
    def test_a_downstream_device_is_not_a_hub_interface(self):
        hub = "USB\\VID_1111&PID_2222\\HUB1"
        function = "USB\\VID_1111&PID_2222&MI_00\\6&AAAA&0&0000"
        downstream = "USB\\VID_3333&PID_4444\\DEV1"
        records = [
            {"instance_id": hub, "parent_id": "USB\\ROOT_HUB30\\5&1"},
            {"instance_id": function, "parent_id": hub},
            {
                "instance_id": downstream,
                "parent_id": hub,
                "compatible_ids": ["USB\\MS_COMP_MTP"],
            },
        ]
        groups = group_usb_nodes(records)
        hub_children = [
            entry
            for entry in groups[hub.upper()]
            if entry["instance_id"] != hub
        ]
        self.assertEqual([entry["instance_id"] for entry in hub_children], [function])
        self.assertIn(downstream.upper(), groups)
        # The hub must not inherit the downstream device's MTP identity.
        hub_record = records[0]
        self.assertFalse(_is_mtp(hub_record, hub_children))

    def test_an_mi_child_still_follows_its_parent(self):
        parent = "USB\\VID_1111&PID_2222\\SER1"
        function = "USB\\VID_1111&PID_2222&MI_02\\6&BBBB&0&0001"
        groups = group_usb_nodes(
            [
                {"instance_id": parent},
                {"instance_id": function, "parent_id": parent},
            ]
        )
        self.assertEqual(len(groups[parent.upper()]), 2)


class TestPhysicalOwnership(LabTestCase):
    """A USB devnode belongs to a physical device only through CM_Get_Parent."""

    def test_a_non_mi_downstream_device_does_not_become_a_hub_interface(self):
        downstream = "USB\\VID_9999&PID_0000\\DEV1"
        hub = "USB\\VID_1111&PID_2222\\HUB1"
        physical = {hub.upper(): {"instance_id": hub}}
        with mock.patch(
            "lab_agent.usb_host.windows.winapi.device_id",
            side_effect=lambda devinst: {7: downstream}.get(devinst),
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.parent_devinst",
            return_value=None,
        ):
            self.assertIsNone(_physical_for(7, physical))

    def test_a_real_ancestor_chain_still_wins(self):
        child = "USB\\VID_1234&PID_5678&MI_00\\9&ABCD&0&0000"
        parent = "USB\\VID_1234&PID_5678\\SER1"
        ids = {1: child, 2: parent}
        parents = {1: 2}
        physical = {parent.upper(): {"instance_id": parent}}
        with mock.patch(
            "lab_agent.usb_host.windows.winapi.device_id",
            side_effect=lambda devinst: ids.get(devinst),
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.parent_devinst",
            side_effect=lambda devinst: parents.get(devinst),
        ):
            self.assertEqual(_physical_for(1, physical), parent)


class TestVolumeOwnershipPath(LabTestCase):
    def test_the_usbstor_disk_instance_is_read_from_the_volume_path(self):
        path = (
            "\\\\?\\storage#volume#_??_usbstor#disk&ven_&prod_&rev_0.01"
            "#9&5fc6598&0&2025010101&0"
            "#{53f56307-b6bf-11d0-94f2-00a0c91efb8b}"
            "#{53f5630d-b6bf-11d0-94f2-00a0c91efb8b}"
        )
        self.assertEqual(
            usbstor_instance_from_volume_path(path),
            "USBSTOR\\DISK&VEN_&PROD_&REV_0.01\\9&5FC6598&0&2025010101&0",
        )

    def test_a_fixed_disk_volume_has_no_usbstor_instance(self):
        path = (
            "\\\\?\\storage#volume#{d7253d87-0877-11ec-97e9-806e6f6e6963}"
            "#0000000000100000#{53f5630d-b6bf-11d0-94f2-00a0c91efb8b}"
        )
        self.assertIsNone(usbstor_instance_from_volume_path(path))


class TestVolumeOwnership(LabTestCase):
    """The three ways ``_volumes`` joins a drive letter to a USB device."""

    PARENT = "USB\\VID_38F4&PID_FFFF\\2025010101"
    USBSTOR_ID = (
        "USBSTOR\\DISK&VEN_&PROD_&REV_0.01\\9&5FC6598&0&2025010101&0"
    )
    USBSTOR_PATH = (
        "\\\\?\\storage#volume#_??_usbstor#disk&ven_&prod_&rev_0.01"
        "#9&5fc6598&0&2025010101&0"
        "#{53f56307-b6bf-11d0-94f2-00a0c91efb8b}"
        "#{53f5630d-b6bf-11d0-94f2-00a0c91efb8b}"
    )
    FIXED_VOLUME_PATH = (
        "\\\\?\\storage#volume#{d7253d87-0877-11ec-97e9-806e6f6e6963}"
        "#0000000011500000#{53f5630d-b6bf-11d0-94f2-00a0c91efb8b}"
    )

    def node(self, devinst, instance_id, path):
        return SimpleNamespace(
            devinst=devinst, instance_id=instance_id, path=path
        )

    def disk(self, devinst=20):
        return self.node(devinst, self.USBSTOR_ID, self.USBSTOR_PATH)

    def interface_paths(self, volumes=(), disks=()):
        def side_effect(guid):
            if guid is winapi.GUID_DEVINTERFACE_VOLUME:
                return list(volumes)
            if guid is winapi.GUID_DEVINTERFACE_DISK:
                return list(disks)
            return []

        return side_effect

    def volume_information(self):
        return {
            "label": "SiFli MSC",
            "filesystem": "FAT",
            "size": 6656,
            "drive_type": winapi.DRIVE_REMOVABLE,
        }

    def test_a_usbstor_path_identifies_the_composite_device(self):
        volume = self.node(10, self.USBSTOR_PATH, self.USBSTOR_PATH)
        disk = self.disk(20)
        physical = {self.PARENT.upper(): {"instance_id": self.PARENT}}

        def physical_for(devinst, _physical):
            return self.PARENT if devinst == 20 else None

        number = mock.Mock(return_value=None)
        with mock.patch(
            "lab_agent.usb_host.windows.winapi.logical_drives",
            return_value=["D:\\"],
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.drive_type",
            return_value=winapi.DRIVE_REMOVABLE,
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.volume_name_for_mount_point",
            return_value=self.USBSTOR_PATH,
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.enumerate_interface_paths",
            side_effect=self.interface_paths([volume], [disk]),
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.device_number",
            number,
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.volume_information",
            return_value=self.volume_information(),
        ), mock.patch(
            "lab_agent.usb_host.windows._physical_for",
            side_effect=physical_for,
        ):
            result = WindowsUsbHost()._volumes(physical, [disk])

        self.assertEqual(list(result), [self.PARENT])
        self.assertEqual(result[self.PARENT][0].drive, "D:")
        self.assertEqual(result[self.PARENT][0].label, "SiFli MSC")
        # The USBSTOR association answered before the disk-number fallback, so
        # no disk handle was opened.
        number.assert_not_called()

    def test_a_disk_number_joins_the_drive_when_the_path_says_nothing(self):
        volume = self.node(10, self.FIXED_VOLUME_PATH, self.FIXED_VOLUME_PATH)
        disk = self.disk(20)
        physical = {self.PARENT.upper(): {"instance_id": self.PARENT}}

        def physical_for(devinst, _physical):
            return self.PARENT if devinst == 20 else None

        def number_for(path):
            if path == "\\\\.\\D:" or path == self.USBSTOR_PATH:
                return 7
            return None

        with mock.patch(
            "lab_agent.usb_host.windows.winapi.logical_drives",
            return_value=["D:\\"],
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.drive_type",
            return_value=winapi.DRIVE_REMOVABLE,
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.volume_name_for_mount_point",
            return_value=self.FIXED_VOLUME_PATH,
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.enumerate_interface_paths",
            side_effect=self.interface_paths([volume], [disk]),
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.device_number",
            side_effect=number_for,
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.volume_information",
            return_value=self.volume_information(),
        ), mock.patch(
            "lab_agent.usb_host.windows._physical_for",
            side_effect=physical_for,
        ):
            result = WindowsUsbHost()._volumes(physical, [disk])

        self.assertEqual(list(result), [self.PARENT])
        self.assertEqual(result[self.PARENT][0].drive, "D:")

    def test_a_fixed_drive_is_skipped_before_any_mapping_when_not_allowed(self):
        drive_name = mock.Mock(return_value=None)
        number = mock.Mock(return_value=None)
        kind = mock.Mock(return_value=winapi.DRIVE_FIXED)
        with mock.patch(
            "lab_agent.usb_host.windows.winapi.logical_drives",
            return_value=["C:\\"],
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.drive_type",
            kind,
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.volume_name_for_mount_point",
            drive_name,
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.enumerate_interface_paths",
            side_effect=self.interface_paths([], [self.disk(20)]),
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.device_number",
            number,
        ), mock.patch(
            "lab_agent.usb_host.windows._physical_for",
            return_value=self.PARENT,
        ):
            result = WindowsUsbHost(allow_fixed_drives=False)._volumes({}, [])

        self.assertEqual(result, {})
        # The drive was examined and refused on its type, not ignored wholesale.
        kind.assert_called_once_with("C:\\")
        drive_name.assert_not_called()
        number.assert_not_called()

    def test_a_fixed_drive_without_a_usb_ancestor_is_dropped(self):
        disk = self.disk(20)

        def number_for(path):
            return 7 if path in ("\\\\.\\C:", self.USBSTOR_PATH) else None

        number = mock.Mock(side_effect=number_for)
        with mock.patch(
            "lab_agent.usb_host.windows.winapi.logical_drives",
            return_value=["C:\\"],
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.drive_type",
            return_value=winapi.DRIVE_FIXED,
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.volume_name_for_mount_point",
            return_value=self.FIXED_VOLUME_PATH,
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.enumerate_interface_paths",
            side_effect=self.interface_paths([], [disk]),
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.device_number",
            number,
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.volume_information",
            return_value={
                "label": None,
                "filesystem": "NTFS",
                "size": 1,
                "drive_type": winapi.DRIVE_FIXED,
            },
        ), mock.patch(
            "lab_agent.usb_host.windows._physical_for",
            return_value=None,
        ):
            result = WindowsUsbHost(allow_fixed_drives=True)._volumes({}, [disk])

        self.assertEqual(result, {})
        # The disk-number mapping was attempted and produced no USB ancestor;
        # the drive was not silently ignored before the lookup.
        number.assert_any_call("\\\\.\\C:")


class TestHidReportDescriptor(LabTestCase):
    """The report descriptor is the authoritative top-level usage."""

    def test_a_mouse_report_descriptor(self):
        data = b"\x05\x01\x09\x02\xa1\x01\x09\x01\xa1\x00"
        usage = parse_hid_report_usage(data)
        self.assertEqual(usage, (0x01, 0x02))
        self.assertEqual(infer_hid_protocol(*usage), 2)

    def test_a_keyboard_report_descriptor(self):
        data = b"\x05\x01\x09\x06\xa1\x01\x05\x07\x19\xe0"
        usage = parse_hid_report_usage(data)
        self.assertEqual(usage, (0x01, 0x06))
        self.assertEqual(infer_hid_protocol(*usage), 1)

    def test_empty_malformed_and_long_only_descriptors_are_none(self):
        for data in (
            b"",
            b"\x06\x01",  # Usage Page declares 2 bytes, only 1 present
            b"\xfe\x02\x00\xaa\xbb",  # a long item with no usage pair
        ):
            with self.subTest(data=data):
                self.assertIsNone(parse_hid_report_usage(data))


class TestHidSourceOrder(LabTestCase):
    """caps -> report descriptor (best effort) -> compatible ids."""

    PARENT = "USB\\VID_38F4&PID_FFFF\\2025010101"
    MOUSE_REPORT = b"\x05\x01\x09\x02\xa1\x01"

    def run_hid(self, *, caps, report, compatible):
        node = SimpleNamespace(
            devinst=10,
            instance_id="HID\\VID_38F4&PID_FFFF\\9&1",
            path="\\\\?\\hid#vid_38f4&pid_ffff#9&1",
        )
        physical = {self.PARENT.upper(): {"instance_id": self.PARENT}}
        report_mock = mock.Mock(return_value=report)
        with mock.patch(
            "lab_agent.usb_host.windows.winapi.enumerate_interface_paths",
            return_value=[node],
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.hid_capabilities",
            return_value=caps,
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.hid_report_descriptor",
            report_mock,
        ), mock.patch(
            "lab_agent.usb_host.windows._physical_for",
            return_value=self.PARENT,
        ), mock.patch(
            "lab_agent.usb_host.windows._first_mi",
            return_value=0,
        ), mock.patch(
            "lab_agent.usb_host.windows._compatible_ids_for",
            return_value=tuple(compatible),
        ), mock.patch.object(
            WindowsUsbHost,
            "_hub_for",
            return_value=("hub-path", 1),
        ):
            result = WindowsUsbHost()._hid(physical, {})
        return result[self.PARENT][0], report_mock

    def test_valid_caps_win_and_skip_the_report_descriptor(self):
        info, report = self.run_hid(
            caps={
                "usage_page": 0x01,
                "usage": 0x02,
                "report_length": 5,
                "product": "Mouse",
            },
            report=self.MOUSE_REPORT,
            compatible=["USB\\Class_03&SubClass_01&Prot_01"],
        )
        self.assertEqual((info.usage_page, info.usage, info.protocol), (1, 2, 2))
        report.assert_not_called()

    def test_zero_caps_use_the_report_descriptor(self):
        info, report = self.run_hid(
            caps={"usage_page": 0, "usage": 0},
            report=self.MOUSE_REPORT,
            compatible=["USB\\Class_03&SubClass_01&Prot_01"],
        )
        self.assertEqual((info.usage_page, info.usage, info.protocol), (1, 2, 2))
        report.assert_called_once_with("hub-path", 1, 0)

    def test_a_failed_report_descriptor_falls_back_to_compatible_ids(self):
        info, report = self.run_hid(
            caps=None,
            report=None,
            compatible=[
                "USB\\COMPAT_VID_38F4&Class_03&SubClass_01&Prot_01",
                "USB\\Class_03&SubClass_01&Prot_01",
            ],
        )
        self.assertEqual((info.usage_page, info.usage, info.protocol), (1, 6, 1))
        report.assert_called_once_with("hub-path", 1, 0)

    def test_a_failed_report_descriptor_continues_to_a_mouse_compatible_id(self):
        info, report = self.run_hid(
            caps={"usage_page": 0, "usage": 0},
            report=None,
            compatible=["USB\\Class_03&SubClass_01&Prot_02"],
        )
        self.assertEqual((info.usage_page, info.usage, info.protocol), (1, 2, 2))
        report.assert_called_once_with("hub-path", 1, 0)


class TestHidUsageFallback(LabTestCase):
    def test_a_keyboard_class_triplet_supplies_the_usage_when_caps_are_missing(self):
        compatible = [
            "USB\\COMPAT_VID_38f4&Class_03&SubClass_01&Prot_01",
            "USB\\Class_03&SubClass_01&Prot_01",
        ]
        self.assertEqual(hid_usage(None, compatible), (0x01, 0x06))

    def test_zero_caps_fall_back_to_a_mouse_class_triplet(self):
        caps = {"usage_page": 0, "usage": 0, "report_length": 0, "product": None}
        compatible = ["USB\\Class_03&SubClass_01&Prot_02"]
        self.assertEqual(hid_usage(caps, compatible), (0x01, 0x02))

    def test_a_usable_caps_reading_is_not_overridden(self):
        caps = {"usage_page": 0x01, "usage": 0x02}
        self.assertEqual(
            hid_usage(caps, ["USB\\Class_03&SubClass_01&Prot_01"]),
            (0x01, 0x02),
        )

    def test_compatible_ids_are_found_on_a_usb_interface_ancestor(self):
        info = {
            4: type(
                "Node", (), {"compatible_ids": ("USB\\Class_03&SubClass_01&Prot_01",)}
            )()
        }
        with mock.patch(
            "lab_agent.usb_host.windows.winapi.device_id",
            side_effect=lambda devinst: {
                3: "HID\\VID_38F4&PID_FFFF&MI_03\\9&1",
                4: "USB\\VID_38F4&PID_FFFF&MI_03\\8&2",
            }.get(devinst),
        ), mock.patch(
            "lab_agent.usb_host.windows.winapi.parent_devinst",
            side_effect=lambda devinst: {3: 4}.get(devinst),
        ):
            self.assertEqual(
                _compatible_ids_for(3, info),
                ("USB\\Class_03&SubClass_01&Prot_01",),
            )


class TestEnumerationCache(LabTestCase):
    def test_the_cache_timestamp_is_taken_after_enumeration(self):
        backend = FakeBackend(devices=[CDC_DEVICE], delay=0.08)
        service = UsbHostService(backend, cache_ttl=0.05)
        service.devices()
        service.devices()
        self.assertEqual(backend.enumerations, 1)
        service._devices(refresh=True)
        self.assertEqual(backend.enumerations, 2)


class TestMatching(LabTestCase):
    def test_every_documented_key_matches(self):
        device = a_device(
            interfaces=(UsbInterface(0, 0x02, 2, 1, "cdc_acm"),),
            com_ports=(ComPort("COM12"),),
            volumes=(UsbVolume("E:", label="SiFli MSC"),),
            hid=(HidInfo(3, 1, 6, protocol=1),),
            speed="full",
        )
        matching = {
            "vid": 0x1234,
            "pid": "0x5678",
            "serial": "SER001",
            "product_contains": "test",
            "device_class": "cdc",
            "class": 2,
            "interface_class": 2,
            "usage_page": 1,
            "usage": 6,
            "protocol": 1,
            "com_port": "com12",
            "volume_label": "sifli msc",
            "require_com": True,
            "require_volume": True,
            "speed": "full",
            "present": True,
        }
        self.assertTrue(match_device(device, matching))

    def test_an_unknown_key_is_refused(self):
        with self.assertRaises(UsbHostBadMatchKey) as caught:
            match_device(a_device(), {"serial_number": "x"})
        self.assertIn("serial_number", str(caught.exception))

    def test_since_accepts_one_key_or_a_list(self):
        self.assertEqual(since_keys(None), ())
        self.assertEqual(since_keys("a"), ("a",))
        self.assertEqual(since_keys(["a", "b"]), ("a", "b"))


class TestWait(LabTestCase):
    def service(self, sequence):
        backend = FakeBackend(sequence=sequence)
        return UsbHostService(backend, cache_ttl=0)

    def test_a_device_that_was_already_seen_does_not_satisfy_the_wait(self):
        old = a_device(key="old")
        new = a_device(key="new")
        service = self.service([[], [old], [old, new]])
        device, elapsed = service.wait(
            {"vid": 0x1234}, since=[old.key], timeout=2.0, interval=0.01
        )
        self.assertEqual(device.key, "new")
        self.assertGreaterEqual(elapsed, 0.0)

    def test_a_timeout_carries_the_candidates_it_saw(self):
        old = a_device(key="old")
        service = self.service([[], [old]])
        service.backend.devices = [old]
        with self.assertRaises(UsbHostTimeout) as caught:
            service.wait(
                {"vid": 0x1234}, since=[old.key], timeout=0.05, interval=0.01
            )
        error = caught.exception
        self.assertEqual(error.status, 408)
        self.assertEqual(error.code, "timeout")
        self.assertEqual(
            [entry["key"] for entry in error.details["candidates"]], ["old"]
        )


class TestSerialSessions(LabTestCase):
    def setUp(self):
        super().setUp()
        self.factory = FakeSerialFactory()
        self.backend = FakeBackend(devices=[CDC_DEVICE])
        self.service = UsbHostService(
            self.backend,
            UsbHostOptions(exclude_ports=("COM13",)),
            forbidden_ports=["COM8"],
            serial_factory=self.factory,
        )
        self.addCleanup(self.service.close_all)

    def open(self, **kwargs):
        return self.service.open_serial(CDC_DEVICE.key, "COM12", **kwargs)

    def test_open_returns_the_port_and_passes_dtr_through(self):
        payload = self.open(baud=921600, dtr=True, rts=True)
        self.assertEqual(payload["port"], "COM12")
        self.assertEqual(payload["baud"], 921600)
        self.assertTrue(payload["dtr"])
        self.assertTrue(payload["rts"])
        self.assertTrue(payload["token"])
        self.assertTrue(self.factory.opened["COM12"].dtr)
        self.assertTrue(self.factory.opened["COM12"].rts)

    def test_a_second_open_of_the_same_port_is_busy(self):
        self.open()
        with self.assertRaises(UsbHostPortBusy) as caught:
            self.open()
        self.assertEqual(caught.exception.status, 409)

    def test_a_port_the_os_refuses_is_a_documented_conflict(self):
        def refusing(port, baud):
            raise OSError("Access is denied")

        service = UsbHostService(
            self.backend, serial_factory=refusing
        )
        with self.assertRaises(UsbHostPortDenied) as caught:
            service.open_serial(CDC_DEVICE.key, "COM12")
        error = caught.exception
        self.assertEqual(error.status, 409)
        self.assertEqual(error.code, "port_denied")
        self.assertEqual(error.details["port"], "COM12")
        self.assertEqual(service.sessions.tokens(), [])

    def test_the_console_port_is_forbidden(self):
        service = UsbHostService(
            self.backend,
            forbidden_ports=["COM12"],
            serial_factory=self.factory,
        )
        with self.assertRaises(UsbHostPortForbidden) as caught:
            service.open_serial(CDC_DEVICE.key, "COM12")
        self.assertEqual(caught.exception.code, "forbidden_port")

    def test_an_excluded_port_is_forbidden(self):
        service = UsbHostService(
            self.backend,
            UsbHostOptions(exclude_ports=("COM12",)),
            serial_factory=self.factory,
        )
        with self.assertRaises(UsbHostPortForbidden):
            service.open_serial(CDC_DEVICE.key, "COM12")

    def test_a_port_that_is_not_a_child_of_the_device_is_refused(self):
        with self.assertRaises(UsbHostPortNotUsb):
            self.service.open_serial(CDC_DEVICE.key, "COM99")

    def test_an_unknown_device_is_not_found(self):
        with self.assertRaises(UsbHostDeviceNotFound):
            self.service.open_serial("USB\\VID_0000&PID_0000\\x", "COM12")

    def test_write_and_modem_lines_reach_the_port(self):
        token = self.open()["token"]
        self.assertEqual(self.service.write_serial(token, b"hello")["written"], 5)
        self.assertEqual(self.factory.opened["COM12"].writes, [b"hello"])
        payload = self.service.set_lines(token, dtr=True, rts=False)
        self.assertTrue(payload["dtr"])
        self.assertFalse(payload["rts"])
        self.assertTrue(self.factory.opened["COM12"].dtr)

    def test_closing_removes_the_token_and_closes_the_port(self):
        token = self.open()["token"]
        self.assertTrue(self.service.close_serial(token)["closed"])
        self.assertEqual(self.factory.opened["COM12"].closed, 1)
        with self.assertRaises(UsbHostTokenNotFound) as caught:
            self.service.read_serial(token)
        self.assertEqual(caught.exception.status, 404)

    def test_close_all_releases_every_session(self):
        token = self.open()["token"]
        self.service.close_all()
        self.assertEqual(self.factory.opened["COM12"].closed, 1)
        with self.assertRaises(UsbHostTokenNotFound):
            self.service.read_serial(token)

    def test_a_port_that_disappears_reports_closed_on_write(self):
        token = self.open()["token"]
        handle = self.factory.opened["COM12"]
        handle.fail_reads = True
        deadline = time.monotonic() + 2.0
        while not self.service.sessions.get(token).closed:
            if time.monotonic() > deadline:
                self.fail("the reader thread did not notice the failure")
            time.sleep(0.005)
        with self.assertRaises(UsbHostPortClosed):
            self.service.write_serial(token, b"x")


class BlockingBackend:
    """Stops inside ``enumerate()``, the window the USB lock must protect.

    ``usb_host_open_serial`` gates and opens under the runtime's ``_usb_lock``;
    the first slow step is the enumeration performed by ``service.device()``.
    Blocking there -- rather than inside the session registry, which is already
    behind its own lock -- is what makes the race test fail when the runtime
    lock is removed.
    """

    supported = True
    platform_name = "test"
    features = ("devices",)

    def __init__(self, devices, entered, release):
        self.devices = list(devices)
        self.entered = entered
        self.release = release

    def capabilities(self):
        return {
            "supported": True,
            "platform": self.platform_name,
            "features": list(self.features),
            "host": {"name": "local", "machine": "fake"},
            "reason": None,
        }

    def enumerate(self):
        self.entered.set()
        self.release.wait(5.0)
        return list(self.devices)

    def volumes(self):
        return []


class TestOpenReleaseRace(LabTestCase):
    """An open and a release must not leave a live session behind."""

    def setUp(self):
        super().setUp()
        testbed = TestBed(
            id="tb_race",
            platform="board",
            connections=[Connection(FixtureRef("usb_host"))],
        )
        self.backend = BlockingBackend(
            [CDC_DEVICE], threading.Event(), threading.Event()
        )
        self.serial_factory = FakeSerialFactory()
        self.service = UsbHostService(
            self.backend, serial_factory=self.serial_factory
        )
        self.runtime = TestBedRuntime(
            testbed, usb_host_factory=lambda testbed: self.service
        )
        self.runtime.allocate("holder", ["usb_host"])

        # Records whether teardown got as far as closing sessions. With the
        # lock in place it cannot, because the opener is still enumerating.
        self.close_all_called = threading.Event()
        original_close_all = self.service.close_all

        def tracking_close_all():
            self.close_all_called.set()
            original_close_all()

        self.service.close_all = tracking_close_all
        self.addCleanup(self.service.close_all)

    def test_an_open_racing_a_release_is_closed_by_the_release(self):
        result = {}

        def opener():
            try:
                result["payload"] = self.runtime.usb_host_open_serial(
                    CDC_DEVICE.key, "COM12"
                )
            except Exception as exc:  # noqa: BLE001 - asserted below
                result["error"] = exc

        opener_thread = threading.Thread(target=opener)
        opener_thread.start()
        self.assertTrue(self.backend.entered.wait(2.0), "open never started")

        releaser = threading.Thread(target=self.runtime.release)
        releaser.start()
        # The release clears the holder first and only then reaches teardown.
        deadline = time.monotonic() + 2.0
        while self.runtime.holder is not None:
            if time.monotonic() > deadline:
                self.fail("the release did not clear the holder")
            time.sleep(0.005)

        blocked = close_ran = None
        try:
            # Fix: the release is still blocked on `_usb_lock`, so teardown has
            # not reached `close_all`. Defect (no lock on open or on close): the
            # release finishes here and close_all has run.
            releaser.join(0.3)
            blocked = releaser.is_alive()
            close_ran = self.close_all_called.is_set()
        finally:
            self.backend.release.set()
            opener_thread.join(5.0)
            releaser.join(5.0)

        self.assertFalse(opener_thread.is_alive())
        self.assertFalse(releaser.is_alive())
        self.assertTrue(
            blocked,
            "the release finished while the open was still enumerating; "
            "the open is not protected by the USB lock",
        )
        self.assertFalse(
            close_ran,
            "teardown reached close_all during the open; the USB lock does "
            "not span the gate and the open",
        )
        self.assertNotIn("error", result)

        self.assertEqual(self.serial_factory.opened["COM12"].closed, 1)
        self.assertEqual(self.service.sessions.tokens(), [])
        self.assertEqual(self.service.sessions._by_port, {})

        token = result["payload"]["token"]
        for action in (
            lambda: self.service.read_serial(token),
            lambda: self.service.write_serial(token, b"x"),
            lambda: self.service.set_lines(token, dtr=True),
        ):
            with self.assertRaises(UsbHostTokenNotFound):
                action()

    def test_an_open_after_release_is_refused(self):
        self.runtime.release()
        with self.assertRaises(UsbHostNotAllocated):
            self.runtime.usb_host_open_serial(CDC_DEVICE.key, "COM12")
        self.assertEqual(self.service.sessions.tokens(), [])


class TestByteStreamCursor(LabTestCase):
    def test_the_cursor_is_absolute_and_monotonic(self):
        factory = FakeSerialFactory(chunks=[b"abc", b"def"])
        stream = ByteStream("COM9", 115200, factory=factory)
        stream.start()
        try:
            deadline = time.monotonic() + 2.0
            while stream.cursor < 6:
                if time.monotonic() > deadline:
                    self.fail("the reader thread did not deliver the chunks")
                time.sleep(0.005)
            data, cursor, closed = stream.read_from(0, 0.2)
            self.assertEqual(data, b"abcdef")
            self.assertEqual(cursor, 6)
            self.assertFalse(closed)

            data, cursor, _ = stream.read_from(3, 0.0)
            self.assertEqual(data, b"def")
            self.assertEqual(cursor, 6)

            data, cursor, _ = stream.read_from(6, 0.01)
            self.assertEqual(data, b"")
            self.assertEqual(cursor, 6)
        finally:
            stream.stop()


class TestVolumePaths(LabTestCase):
    def volume(self, removable=True):
        return UsbVolume(drive=self.root, removable=removable)

    def test_a_relative_path_inside_the_volume_is_read(self):
        self.write("README.TXT", "hello")
        data = read_volume_file(
            self.volume(), "README.TXT", max_bytes=1024
        )
        self.assertEqual(data, b"hello")

    def test_a_nested_relative_path_is_read(self):
        self.write("dir/README.TXT", "nested")
        data = read_volume_file(self.volume(), "dir/README.TXT", max_bytes=1024)
        self.assertEqual(data, b"nested")

    def test_traversal_and_absolute_paths_are_refused(self):
        for path in (
            "../outside.txt",
            "dir/../../outside.txt",
            "/etc/passwd",
            "C:\\Windows\\win.ini",
            "..\\escape.txt",
            "dir//file.txt",
            "NUL",
        ):
            with self.subTest(path=path):
                with self.assertRaises(UsbHostPathDenied):
                    validate_relative_path(path)

    def test_an_alternate_data_stream_is_refused(self):
        with self.assertRaises(UsbHostPathDenied):
            validate_relative_path("README.TXT:stream")

    def test_a_path_that_is_too_long_is_refused(self):
        with self.assertRaises(UsbHostPathDenied):
            validate_relative_path("a" * 261)

    def test_a_symlink_is_refused(self):
        self.write("real.txt", "content")
        link = os.path.join(self.root, "link.txt")
        try:
            os.symlink(os.path.join(self.root, "real.txt"), link)
        except (OSError, NotImplementedError):
            self.skipTest("creating a symlink needs a privilege this host lacks")
        with self.assertRaises(UsbHostPathDenied):
            read_volume_file(self.volume(), "link.txt", max_bytes=1024)

    def test_the_reparse_attribute_is_refused_even_without_a_link(self):
        # The symlink test above needs a privilege Developer Mode may not grant.
        # The rule under test is the attribute check, so exercise it directly.
        from types import SimpleNamespace
        from unittest import mock

        self.write("link.txt", "content")
        attribute = SimpleNamespace(
            st_file_attributes=0x400  # FILE_ATTRIBUTE_REPARSE_POINT
        )
        with mock.patch(
            "lab_agent.usb_host.volume_paths.os.lstat", return_value=attribute
        ):
            with self.assertRaises(UsbHostPathDenied):
                read_volume_file(self.volume(), "link.txt", max_bytes=1024)

    def test_a_file_above_the_bound_is_too_large(self):
        self.write("big.bin", "abcdef")
        with self.assertRaises(UsbHostFileTooLarge) as caught:
            read_volume_file(self.volume(), "big.bin", max_bytes=2)
        self.assertEqual(caught.exception.status, 413)

    def test_a_missing_file_is_not_found(self):
        with self.assertRaises(UsbHostFileNotFound):
            read_volume_file(self.volume(), "absent.txt", max_bytes=1024)

    def test_a_fixed_drive_is_refused_unless_allowed(self):
        self.write("README.TXT", "hello")
        with self.assertRaises(UsbHostVolumeNotMounted):
            read_volume_file(
                self.volume(removable=False), "README.TXT", max_bytes=1024
            )
        data = read_volume_file(
            self.volume(removable=False),
            "README.TXT",
            max_bytes=1024,
            allow_fixed_drives=True,
        )
        self.assertEqual(data, b"hello")


class TestFilesThroughTheService(LabTestCase):
    def setUp(self):
        super().setUp()
        # No newline: LabTestCase.write uses text mode, and Windows would turn
        # "\n" into "\r\n", making the byte count assert the host's line ending
        # rather than the capability.
        self.write("README.TXT", "SiFli MSC")
        self.device = a_device(
            key="USB\\VID_1234&PID_5678\\SER001",
            volumes=(
                UsbVolume(
                    drive=self.root,
                    label="SiFli MSC",
                    filesystem="FAT",
                    size=8192,
                    removable=True,
                    device_key="USB\\VID_1234&PID_5678\\SER001",
                ),
            ),
        )
        self.service = UsbHostService(
            FakeBackend(devices=[self.device], volumes=self.device.volumes),
            UsbHostOptions(max_file_bytes=1024),
        )

    def test_read_file_returns_text(self):
        payload = self.service.read_file(self.device.key, "README.TXT")
        self.assertEqual(payload["data"], "SiFli MSC")
        self.assertEqual(payload["size"], 9)
        self.assertEqual(payload["encoding"], "text")

    def test_read_file_returns_base64_on_request(self):
        payload = self.service.read_file(
            self.device.key, "README.TXT", encoding="base64"
        )
        self.assertEqual(payload["encoding"], "base64")
        self.assertEqual(payload["data"], "U2lGbGkgTVND")

    def test_a_device_without_a_volume_is_not_mounted(self):
        service = UsbHostService(FakeBackend(devices=[self.device]))
        with self.assertRaises(UsbHostVolumeNotMounted):
            service.read_file(self.device.key, "README.TXT")

    def test_volumes_payload_reports_ownership(self):
        payload = self.service.volumes_payload()
        self.assertEqual(payload["volumes"][0]["device_key"], self.device.key)
        self.assertEqual(payload["volumes"][0]["label"], "SiFli MSC")


class TestHidAndAudio(LabTestCase):
    def test_hid_keyboard_and_mouse_are_reported_with_their_protocol(self):
        device = a_device(
            interfaces=(UsbInterface(3, 0x03, 0, 0, "hid"),),
            hid=(
                HidInfo(3, 1, 6, protocol=1, report_length=8, product="Keyboard"),
                HidInfo(4, 1, 2, protocol=2, report_length=4, product="Mouse"),
            ),
        )
        payload = UsbHostService(FakeBackend(devices=[device])).hid_payload(
            device.key
        )
        self.assertEqual(
            [(entry["usage"], entry["protocol"]) for entry in payload["interfaces"]],
            [(6, 1), (2, 2)],
        )

    def test_hid_interfaces_without_a_bound_collection_are_a_503(self):
        device = a_device(interfaces=(UsbInterface(3, 0x03, 0, 0, "hid"),))
        with self.assertRaises(UsbHostHidUnavailable) as caught:
            UsbHostService(FakeBackend(devices=[device])).hid_payload(device.key)
        self.assertEqual(caught.exception.status, 503)

    def test_audio_inputs_and_outputs_are_kept_apart(self):
        device = a_device(
            interfaces=(UsbInterface(2, 0x01, 2, 0, "audio_streaming"),),
            audio=AudioEndpoints(
                inputs=(AudioEndpoint(2, "0x85", "in"),),
                outputs=(AudioEndpoint(2, "0x05", "out"),),
            ),
        )
        payload = UsbHostService(FakeBackend(devices=[device])).audio_payload(
            device.key
        )
        self.assertEqual(
            [entry["direction"] for entry in payload["inputs"]], ["in"]
        )
        self.assertEqual(
            [entry["direction"] for entry in payload["outputs"]], ["out"]
        )

    def test_a_device_without_audio_reports_empty_lists(self):
        device = a_device()
        payload = UsbHostService(FakeBackend(devices=[device])).audio_payload(
            device.key
        )
        self.assertEqual(payload["inputs"], [])
        self.assertEqual(payload["outputs"], [])


class TestDevicesPayload(LabTestCase):
    def test_the_device_document_has_the_contract_shape(self):
        service = UsbHostService(FakeBackend(devices=[CDC_DEVICE]))
        payload = service.devices_payload()
        self.assertTrue(payload["captured_at"])
        document = payload["devices"][0]
        self.assertEqual(document["key"], CDC_DEVICE.key)
        self.assertEqual(document["instance_id"], CDC_DEVICE.key)
        self.assertEqual(document["vid"], 0x38F4)
        self.assertEqual(document["com_ports"], [{"port": "COM12", "driver": "usbser"}])
        self.assertEqual(document["interfaces"][0]["class"], 2)
        self.assertEqual(document["interfaces"][0]["name"], "cdc_acm")
        self.assertIsNone(document["audio"])

    def test_the_device_lookup_ignores_case(self):
        service = UsbHostService(FakeBackend(devices=[CDC_DEVICE]))
        self.assertEqual(
            service.device(CDC_DEVICE.key.lower()).key, CDC_DEVICE.key
        )


class TestForbiddenPortsFromTheBinding(LabTestCase):
    def test_consoles_are_collected_from_the_testbed_and_its_instruments(self):
        testbed = TestBed(
            id="tb",
            platform="board",
            console=type("Console", (), {"port": "COM13", "baud": 1000000})(),
            connections=[
                Connection(
                    FixtureRef("bt_peer"),
                    options={"console": {"port": "COM99", "baud": 1000000}},
                ),
                Connection(FixtureRef("usb_host")),
            ],
        )
        self.assertEqual(forbidden_ports(testbed), ["COM13", "COM99"])


if __name__ == "__main__":
    unittest.main()
