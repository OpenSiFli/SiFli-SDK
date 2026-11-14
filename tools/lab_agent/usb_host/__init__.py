# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Host-side USB observation for automated tests.

The capability is a testbed-level connection, not an instrument: the Lab Agent
owns the physical wiring, watches the host's USB bus and the COM ports/volumes
the device exposes, and serves that view over ``/testbed/{id}/usb/*``. The
pytest suite names only ``usb_host``; it never sees a port number.

``create_service`` is the one entry point the testbed runtime uses. On a
platform without a backend it still returns a service -- every route except
``capabilities`` then answers 501 -- so the unsupported answer is a protocol
message rather than an import or attribute error.
"""

from __future__ import annotations

from .base import USB_HOST_FIXTURE
from .base import AudioEndpoint
from .base import AudioEndpoints
from .base import ComPort
from .base import HidInfo
from .base import UsbDevice
from .base import UsbHostOptions
from .base import UsbInterface
from .base import UsbVolume
from .base import parse_usb_options
from .errors import UsbHostBadMatchKey
from .errors import UsbHostBadRequest
from .errors import UsbHostDeviceNotFound
from .errors import UsbHostError
from .errors import UsbHostFileNotFound
from .errors import UsbHostFileTooLarge
from .errors import UsbHostFixtureNotRequested
from .errors import UsbHostHidUnavailable
from .errors import UsbHostNotAllocated
from .errors import UsbHostPathDenied
from .errors import UsbHostPortBusy
from .errors import UsbHostPortDenied
from .errors import UsbHostPortClosed
from .errors import UsbHostPortForbidden
from .errors import UsbHostPortNotUsb
from .errors import UsbHostTimeout
from .errors import UsbHostTokenNotFound
from .errors import UsbHostUnsupported
from .errors import UsbHostVolumeNotMounted
from .null import UnsupportedUsbHost
from .service import UsbHostService
from .service import create_service

__all__ = [
    "USB_HOST_FIXTURE",
    "AudioEndpoint",
    "AudioEndpoints",
    "ComPort",
    "HidInfo",
    "UsbDevice",
    "UsbHostOptions",
    "UsbInterface",
    "UsbVolume",
    "parse_usb_options",
    "UsbHostError",
    "UsbHostUnsupported",
    "UsbHostBadMatchKey",
    "UsbHostBadRequest",
    "UsbHostTimeout",
    "UsbHostNotAllocated",
    "UsbHostFixtureNotRequested",
    "UsbHostPortBusy",
    "UsbHostPortDenied",
    "UsbHostPortForbidden",
    "UsbHostPortNotUsb",
    "UsbHostPortClosed",
    "UsbHostDeviceNotFound",
    "UsbHostTokenNotFound",
    "UsbHostVolumeNotMounted",
    "UsbHostPathDenied",
    "UsbHostFileNotFound",
    "UsbHostFileTooLarge",
    "UsbHostHidUnavailable",
    "UnsupportedUsbHost",
    "UsbHostService",
    "create_service",
]
