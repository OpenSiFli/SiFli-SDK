# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of the CherryUSB MSC + CDC ACM + HID composite example.

The device exposes three functions over one USB connection. What Windows saw --
the identity, the composite classes, the RAM disk and its file, the CDC echo,
the HID capabilities -- is asked of the ``usb_host`` fixture, which talks to
the Lab Agent that owns the USB port; the firmware's own console supplies the
device-side half of each check.

``usb_host`` has no cursor, so every host-side test re-acquires the device
through :func:`_wait_for_device` instead of carrying state over from the
previous test. That keeps each test runnable on its own. The boot test is
different: it needs the console to have been opened before the board reset,
which the runner arranges, and a direct invocation can deselect it.
"""

from __future__ import annotations

import re
import time

from sdk_test.suite_helpers import cherryusb
from sdk_test.usb_host import UsbConflict
from sdk_test.usb_host import UsbNotFound

#: Identity ``src/composite_device.h`` and the string descriptors declare.
VID = 0x38F4
PID = 0xFFFF
SERIAL = "2025010101"

#: What ``main`` and ``composite_device_init`` print, in order. They are
#: literals from the source and ``dut.expect`` compiles its pattern as a regex,
#: so each line is escaped before it is handed over. The design's per-example
#: table calls these "10 boot lines (including the greeting)"; this tuple is
#: the same ten lines, in print order.
BOOT_LINES = (
    "USB Composite Device Demo (MSC + CDC ACM + HID Keyboard)!",
    "Initializing USB Composite Device...",
    "USB Composite Device initialized successfully!",
    "- MSC (Virtual Disk): Available (RAM-based, 8KB)",
    "- CDC ACM (Virtual COM): Available",
    "- HID Keyboard: Available",
    "Waiting for USB enumeration...",
    "Composite device initialized. Press buttons to send text:",
    "KEY1: Send 'SiFli'",
    "KEY2: Send 'CherryUSB'",
)

#: The RAM disk's label and its only file, copied from ``README_CONTENTS`` in
#: ``src/composite_device.c``. ``read_text`` returns the bytes the host read;
#: the line endings are the device's CRLF, not the runner's newline.
VOLUME_LABEL = "SiFli MSC"
README_NAME = "README.TXT"
README_TEXT = (
    "SiFli USB Composite Device Demo!\r\n"
    "Features:\r\n"
    "- MSC: Virtual Disk (RAM-based)\r\n"
    "- CDC ACM: Virtual COM Port\r\n"
    "- HID: Virtual Keyboard\r\n"
)

#: The payload of the CDC echo test; exactly seven bytes, which is also the
#: length the firmware logs for both directions.
CDC_PAYLOAD = b"SiFli\r\n"

#: Bounded retry for the first open. The port can be reported as enumerated
#: while a handle still holds it, typically a session left over from an earlier
#: run; the design's board baseline saw a first open being refused and clearing
#: later. Only refusals that can clear on their own are retried, and the final
#: one is reported with the port name instead of being read as a firmware
#: failure.
OPEN_TIMEOUT = 30.0
OPEN_INTERVAL = 1.0

#: How long to keep re-reading the retained console transcript for a line the
#: enumeration events produced. ``wait_for`` has already confirmed the device on
#: the host side, so the wait only covers the console stream catching up.
TRANSCRIPT_TIMEOUT = 10.0
TRANSCRIPT_INTERVAL = 0.5


def _wait_for_device(usb_host, *, require_com=False, require_volume=False):
    """Return this example's device, waiting for it to enumerate.

    ``require_com`` and ``require_volume`` are added to the match only when the
    calling test needs them: the agent compares the key against
    ``bool(com_ports)`` / ``bool(volumes)``, so a ``False`` value means "must
    have none", not "do not care".
    """
    criteria = {"vid": VID, "pid": PID, "serial": SERIAL}
    if require_com:
        criteria["require_com"] = True
    if require_volume:
        criteria["require_volume"] = True
    return usb_host.wait_for(timeout=30, **criteria)


def _open_serial(usb_host, device, *, timeout=OPEN_TIMEOUT, interval=OPEN_INTERVAL):
    """Open the device's CDC port, retrying a refusal that can clear on its own.

    Only two refusals are retried: ``UsbConflict`` (the port is still held,
    e.g. by a session from an earlier run, or the OS refused the handle) and
    ``UsbNotFound`` (the device is re-enumerating). ``UsbDenied``,
    ``UsbHostUnavailable`` and any other ``UsbHostError`` are permanent for
    this run and propagate at once, so a misconfigured bench does not wait out
    the budget. On exhaustion the last refusal is chained onto an error of the
    same kind that names the port.
    """
    ports = ", ".join(entry.port for entry in device.com_ports) or "none"
    deadline = time.monotonic() + timeout
    attempts = 0
    while True:
        attempts += 1
        try:
            return usb_host.serial(device)
        except (UsbConflict, UsbNotFound) as exc:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise type(exc)(
                    f"{device.key}: could not open {ports} within {timeout:.0f}s "
                    f"after {attempts} attempt(s); the last refusal was: {exc}"
                ) from exc
            time.sleep(min(interval, remaining))


def _expect_in_transcript(
    dut, *patterns, timeout=TRANSCRIPT_TIMEOUT, interval=TRANSCRIPT_INTERVAL
):
    """Wait for every pattern in the console's retained transcript.

    ``dut.expect`` cannot serve here: its cursor only moves forward, and these
    lines are produced by the USB events, so the connect line can be printed
    before the shell prompt that an earlier test already consumed. Reading from
    offset 0 re-scans the retained stream without disturbing that cursor, which
    is what "the device reported this" needs. The stream is re-read until every
    pattern is present because the agent's reader may still be catching up with
    what the device printed.
    """
    deadline = time.monotonic() + timeout
    while True:
        transcript, _ = dut.console.read_from(0, 0.0)
        missing = [
            pattern
            for pattern in patterns
            if re.search(pattern, transcript) is None
        ]
        if not missing:
            return transcript
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise AssertionError(
                f"the console transcript never carried {missing}; "
                f"last transcript tail:\n{transcript[-2000:] or '<empty>'}"
            )
        time.sleep(min(interval, remaining))


def test_boots_the_composite_device(dut):
    """The greeting, the initializer report, the key hints and the prompt.

    ``expect_all`` consumes the patterns in print order because the console
    cursor only moves forward, so a missing middle line fails here even when a
    later one arrives.
    """
    cherryusb.expect_all(
        dut,
        *(re.escape(line) for line in BOOT_LINES),
        cherryusb.PROMPT,
        timeout=30,
    )


def test_enumerates_three_interfaces(usb_host):
    """The host sees the expected identity and all three composite functions.

    ``require_com`` and ``require_volume`` make the wait itself part of the
    assertion: Windows exposes the COM port and mounts the volume a moment
    after the device appears.
    """
    device = _wait_for_device(usb_host, require_com=True, require_volume=True)
    expected = {"msc", "cdc", "hid"}
    missing = expected - set(device.classes)
    assert not missing, (
        f"{device.key} enumerated with classes {device.classes!r}, "
        f"missing {sorted(missing)}"
    )


def test_reports_usb_configuration(dut, usb_host):
    """The console reports that the host configured the composite device.

    The line comes from ``usbd_event_handler``, not from ``main``: it can be
    printed before or after the shell prompt, so a forward-only ``dut.expect``
    would miss it once the boot test has consumed the prompt. The retained
    transcript is scanned instead, which does not move ``dut``'s cursor.

    Only ``USB configured - device ready!`` is asserted. The board evidence for
    this example shows that ``USBD_EVENT_CONNECTED`` is not produced on this
    MUSB port, so ``USB connected`` never reaches the console; its absence is a
    property of the port, not a firmware regression, and asserting it would
    make the suite fail for the wrong reason. The configured event is the
    reliable evidence that enumeration reached SET_CONFIGURATION.

    Scanning from offset 0 relies on the console having been opened before the
    board reset (the runner arranges that); in a direct run against an already
    booted board the stream starts after boot and the event was never retained,
    so this test must be deselected there.
    """
    _wait_for_device(usb_host)
    _expect_in_transcript(dut, r"USB configured - device ready!")


def test_msc_volume_carries_the_readme(usb_host):
    """The RAM disk mounts under its label and holds the expected README."""
    device = _wait_for_device(usb_host)
    volume = usb_host.volume(device, timeout=20)
    assert volume.label == VOLUME_LABEL, (
        f"{volume.drive} is labelled {volume.label!r}, expected {VOLUME_LABEL!r}"
    )
    text = volume.read_text(README_NAME)
    assert text == README_TEXT, (
        f"{README_NAME} on {volume.drive} is {text!r}, expected {README_TEXT!r}"
    )


def test_cdc_echoes_and_logs_lengths(dut, usb_host):
    """A host write comes back byte for byte and is logged in both directions."""
    device = _wait_for_device(usb_host, require_com=True)
    with _open_serial(usb_host, device) as serial:
        # Only output produced from here on may satisfy the console assertions,
        # so a direct run that reuses a console stream does not pass on lines
        # left by an earlier run.
        dut.drain()
        written = serial.write(CDC_PAYLOAD)
        assert written == len(CDC_PAYLOAD), (
            f"wrote {written} of {len(CDC_PAYLOAD)} bytes to {serial.port}"
        )
        echoed = serial.read_exact(len(CDC_PAYLOAD), timeout=5)
        assert echoed == CDC_PAYLOAD, (
            f"{serial.port} echoed {echoed!r}, expected {CDC_PAYLOAD!r}"
        )
        # The firmware reports the bulk OUT transfer and then the bulk IN echo;
        # the console cursor is forward-only, so the two are checked in that
        # order. Length 7 is this payload's length in both callbacks.
        dut.expect(r"CDC actual out len:7", timeout=5)
        dut.expect(r"CDC actual in len:7", timeout=5)


def test_hid_interface_is_a_boot_keyboard(usb_host):
    """The HID function is a boot keyboard (descriptor) carrying keyboard caps.

    Two independent sources are checked. The boot half comes from the interface
    descriptor: bInterfaceClass=3, bInterfaceSubClass=1 (boot) and
    bInterfaceProtocol=1 (keyboard). The caps half comes from the report
    descriptor: Usage Page 1 / Usage 6 (Generic Desktop / Keyboard).
    ``HidInfo.protocol`` cannot prove the boot half -- the agent infers it from
    the usage, so it is 1 whenever the usage check passes.
    """
    device = _wait_for_device(usb_host)
    hid_interfaces = [
        entry for entry in device.interfaces if entry.interface_class == 3
    ]
    assert any(
        entry.subclass == 1 and entry.protocol == 1
        for entry in hid_interfaces
    ), (
        f"{device.key} exposes HID interfaces {hid_interfaces!r}, none declares "
        "the boot subclass/protocol (bInterfaceSubClass=1, "
        "bInterfaceProtocol=1)"
    )
    caps = usb_host.hid(device)
    assert any(
        info.usage_page == 1 and info.usage == 6 for info in caps
    ), (
        f"{device.key} exposes HID report caps {caps!r}, none is a keyboard "
        "(usage page 1, usage 6)"
    )
