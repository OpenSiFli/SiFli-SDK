# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of the CherryUSB HID mouse device example.

The device enumerates as a boot-protocol mouse over USB. What the host saw --
the identity and the HID top-level collection -- is asked of the ``usb_host``
fixture, which talks to the Lab Agent that owns the USB port; the firmware's
own console supplies the boot and configuration evidence.

The mouse actions themselves need a physical KEY2 press and are therefore not
covered here; the case asserts the button's initialization and the thread that
serves it, not the reports it would send.

``usb_host`` has no cursor, so the host-side test re-acquires the device
through ``usb_host.wait_for`` and can run on its own. The console cursor only
moves forward, so the boot test consumes the startup block in print order; the
configuration event test instead reads the retained transcript from offset 0,
because that event can arrive before the prompt the boot test consumes.
"""

from __future__ import annotations

import re
import time

from sdk_test.suite_helpers import cherryusb

#: Identity ``src/main.c`` puts in the device and string descriptors.
VID = 0x38F4
PID = 0xFFFF
SERIAL = "2025101418"

#: The report descriptor declares one top-level collection: Generic Desktop
#: (0x01) / Mouse (0x02).
USAGE_PAGE_GENERIC_DESKTOP = 0x01
USAGE_MOUSE = 0x02

#: The Lab Agent reports the boot protocol implied by that collection. A
#: usage 1/2 mouse maps to 2 even though the interface descriptor's
#: bInterfaceProtocol is 1; see ``infer_hid_protocol`` in the agent backend.
HID_PROTOCOL_MOUSE = 2

#: The configuration event can be printed before the shell prompt, so it is
#: searched in the retained transcript rather than through the console cursor.
CONFIGURED_LINE = re.compile(r"USB HID Mouse configured!")

#: How long the event may take to reach the agent's transcript once the host
#: has enumerated the device, and the per-read wait while polling for it.
CONFIGURED_TIMEOUT = 10.0
CONFIGURED_POLL = 0.5


def test_boots_and_creates_the_mouse_thread(dut):
    """The three banner lines, the KEY2 setup, the thread and the prompt.

    The strings come from ``src/main.c`` and are consumed in print order
    because the console cursor only moves forward. The KEY2 pin number is a
    board-level value from ``CONFIG_BSP_KEY2_PIN`` (11 on the board this case
    was designed against, 33 or 43 on other 52X boards, other numbers on
    56X/58X), and this case has no ``platform_allow``, so the assertion treats
    "the button was initialized" as the contract and accepts any pin. The
    thread's own first line (``mouse thread started``, ``src/main.c:362``) is
    not part of this contract; the evidence is that ``main`` reported the
    creation and the shell prompt is back.
    """
    cherryusb.expect_all(
        dut,
        "========================================",
        "CherryUSB HID mouse Demo",
        "========================================",
        r"KEY2 button initialized, pin=\d+",
        "mouse thread created successfully!",
        cherryusb.PROMPT,
        timeout=30,
    )


def test_enumerates_as_a_mouse(usb_host):
    """The host sees the expected identity and a usage 1/2 HID collection.

    The protocol assertion follows the value the agent reports, which is
    inferred from the collection; the descriptor's own bInterfaceProtocol is
    not what this API exposes.
    """
    device = usb_host.wait_for(
        vid=VID,
        pid=PID,
        serial=SERIAL,
        timeout=30,
    )
    assert "hid" in device.classes, (
        f"{device.key} enumerated with classes {device.classes!r}, no HID function"
    )

    collections = usb_host.hid(device)
    assert collections, f"{device.key} reported no HID interfaces"

    mouse = [
        entry
        for entry in collections
        if entry.usage_page == USAGE_PAGE_GENERIC_DESKTOP
        and entry.usage == USAGE_MOUSE
    ]
    assert mouse, (
        f"{device.key} exposed no Generic Desktop / Mouse collection; "
        f"reported: {collections!r}"
    )
    mismatched = [entry for entry in mouse if entry.protocol != HID_PROTOCOL_MOUSE]
    assert not mismatched, (
        f"{device.key} reported protocol {HID_PROTOCOL_MOUSE} expected for a "
        f"mouse collection; mismatches: {mismatched!r}"
    )


def test_reports_the_configured_event(dut, usb_host):
    """SET_CONFIGURATION reaches the firmware's event handler.

    ``dut.expect`` is deliberately not used: its cursor only moves forward,
    and this line is printed from the USB event handler, which may run before
    the shell prompt consumed by the boot test. Reading the retained
    transcript from offset 0 does not move ``dut``'s cursor, so the check does
    not depend on where the line landed. ``wait_for`` first establishes that
    the host has enumerated the device; the bounded poll then gives the event
    time to arrive.
    """
    usb_host.wait_for(
        vid=VID,
        pid=PID,
        serial=SERIAL,
        timeout=30,
    )

    transcript = ""
    cursor = 0
    deadline = time.monotonic() + CONFIGURED_TIMEOUT
    while True:
        # The first read returns the whole retained transcript; later reads
        # wait for anything printed after it.
        chunk, cursor = dut.console.read_from(cursor, CONFIGURED_POLL)
        transcript += chunk
        if CONFIGURED_LINE.search(transcript):
            return
        if time.monotonic() >= deadline:
            raise AssertionError(
                "the firmware never reported 'USB HID Mouse configured!' "
                f"within {CONFIGURED_TIMEOUT:g}s of enumeration; "
                f"transcript tail:\n{transcript[-2000:]}"
            )
