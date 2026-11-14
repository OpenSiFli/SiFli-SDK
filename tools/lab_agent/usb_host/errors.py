# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Exceptions raised by the Lab Agent's USB host-observation capability.

Each class carries the ``code`` string the HTTP layer puts in the error body and
the ``status`` it answers with, so a route never has to translate an exception
by hand. The codes are the contract the SDK-side client switches on; the
messages are for a person reading the run report.
"""

from __future__ import annotations

from typing import Any
from typing import Dict
from typing import Optional

from ..errors import LabAgentError


class UsbHostError(LabAgentError):
    """Base class: a bad request against the ``usb_host`` capability."""

    code = "usb_host_error"
    status = 400

    def __init__(
        self, message: str, *, details: Optional[Dict[str, Any]] = None
    ) -> None:
        super().__init__(message)
        self.details: Dict[str, Any] = dict(details or {})


class UsbHostUnsupported(UsbHostError):
    """The agent cannot observe USB hosts at all (non-Windows, or ``null``)."""

    code = "usb_host_unsupported"
    status = 501


class UsbHostBadRequest(UsbHostError):
    """A route or wait request is structurally wrong (bad field, bad value)."""

    code = "bad_request"
    status = 400


class UsbHostNotAllocated(UsbHostError):
    """The testbed must be allocated before its USB view may be read."""

    code = "not_allocated"
    status = 409


class UsbHostFixtureNotRequested(UsbHostError):
    """This allocation did not ask for the ``usb_host`` fixture."""

    code = "fixture_not_requested"
    status = 409


class UsbHostBadMatchKey(UsbHostError):
    """A wait request named a match key the agent does not implement."""

    code = "bad_match_key"
    status = 400


class UsbHostTimeout(UsbHostError):
    """No matching device appeared before the deadline."""

    code = "timeout"
    status = 408


class UsbHostPortBusy(UsbHostError):
    """Another session already holds this physical COM port."""

    code = "port_busy"
    status = 409


class UsbHostPortDenied(UsbHostError):
    """The port belongs to the device but the OS refused to open it."""

    code = "port_denied"
    status = 409


class UsbHostPortForbidden(UsbHostError):
    """The port is the DUT console, an instrument console or excluded."""

    code = "forbidden_port"
    status = 409


class UsbHostPortNotUsb(UsbHostError):
    """The requested port is not a CDC child of the enumerated device."""

    code = "port_not_usb"
    status = 409


class UsbHostPortClosed(UsbHostError):
    """The session exists but its underlying serial port is gone."""

    code = "port_closed"
    status = 409


class UsbHostDeviceNotFound(UsbHostError):
    """No enumerated device has this key."""

    code = "device_not_found"
    status = 404


class UsbHostTokenNotFound(UsbHostError):
    """No serial session has this token (closed, or released with the testbed)."""

    code = "token_not_found"
    status = 404


class UsbHostVolumeNotMounted(UsbHostError):
    """The device owns no mounted volume the agent may read."""

    code = "volume_not_mounted"
    status = 409


class UsbHostPathDenied(UsbHostError):
    """A file path is absolute, escapes the volume, or is a reparse point."""

    code = "path_denied"
    status = 400


class UsbHostFileNotFound(UsbHostError):
    """The requested file does not exist on the volume."""

    code = "file_not_found"
    status = 404


class UsbHostFileTooLarge(UsbHostError):
    """The file exceeds the configured ``max_file_bytes``."""

    code = "file_too_large"
    status = 413


class UsbHostHidUnavailable(UsbHostError):
    """The device exposes HID interfaces but the HID driver did not bind."""

    code = "hid_unavailable"
    status = 503
