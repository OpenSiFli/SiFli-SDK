# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of the CherryUSB CDC ACM UART bridge example.

The device enumerates as a CDC ACM serial port over USB and bridges it to
UART2. This bench has no UART peer, so the case stops at the USB side: what
Windows saw -- identity and COM port -- is asked of the ``usb_host`` fixture,
which talks to the Lab Agent that owns the USB port; the firmware's own console
supplies the boot evidence, the configured event and the record that the host's
line coding arrived.

The console cursor is forward-only: every ``expect`` searches from where the
previous one stopped, so the tests stay in collection order and each one owns
the piece of output it provokes. The boot test walks to the shell prompt; the
later tests re-acquire the device through ``usb_host.wait_for`` rather than
carrying state over, so they can also run on their own. The configured event is
the one exception to the cursor rule: the host can configure the device before
or after the boot walk reaches the prompt, so that test reads the retained
transcript through an explicit cursor instead of moving the shared one.
"""

from __future__ import annotations

import re
import time

from sdk_test.suite_helpers import cherryusb
from sdk_test.usb_host import UsbConflict
from sdk_test.usb_host import UsbNotFound

#: Identity ``src/cdc_acm_uart_template.c`` puts in the device and string
#: descriptors: USBD_VID / USBD_PID and the iSerialNumber at index 3.
VID = 0x38F4
PID = 0xFFFF
SERIAL = "202412345678"

#: The rate the host opens the CDC port with. The firmware logs exactly what
#: SET_LINE_CODING carried, so the value is the assertion, not a coincidence of
#: the agent's binding default.
BAUD = 115200

#: Bounded retry for the first open. A port can be reported as enumerated while
#: a handle still holds it (a session left over from an earlier test run), and
#: the design's board baseline saw a first open refused and clear later. Only
#: refusals that can clear on their own are retried; a permanent one is
#: reported immediately, with the port name.
#: Opening the CDC port on this example can take about a minute: Windows waits
#: for the firmware's SET_LINE_CODING handler before it returns. The transport's
#: own open timeout is longer; this is the budget for retries.
OPEN_TIMEOUT = 120.0
OPEN_INTERVAL = 1.0

#: The configuration event is read from the retained transcript, not through
#: ``dut.expect``: Windows may configure the device before the boot walk above
#: reaches its prompt, and the shared cursor only moves forward, so a fresh
#: ``expect`` would miss an event that already happened and fail on a healthy
#: bench. Polling an explicit cursor from offset 0 is what makes the check
#: independent of when in the boot the host got there.
CONFIGURED_TIMEOUT = 10.0
CONFIGURED_INTERVAL = 0.5

#: ``DBG_TAG`` in ``cdc_acm_uart_template.c`` prefixes every info record with
#: ``[I/usb_uart]``. ulog may put a time field before that prefix, so the
#: pattern is searched, never anchored at the start of a line.
CONFIGURED_EVENT = r"\[I/usb_uart\] USB Configured"


def _wait_for_device(usb_host):
    """Return this example's device, waiting for it to enumerate.

    ``require_com=True`` makes the lookup itself enforce that a COM port backs
    the device, so the enumeration test does not assert that half twice.
    """
    return usb_host.wait_for(
        vid=VID,
        pid=PID,
        serial=SERIAL,
        require_com=True,
        timeout=30,
    )


def _open_serial(
    usb_host,
    device,
    *,
    baud=BAUD,
    timeout=OPEN_TIMEOUT,
    interval=OPEN_INTERVAL,
):
    """Open the device's CDC port, retrying a refusal that can clear on its own.

    Only ``UsbConflict`` (the port is still held) and ``UsbNotFound`` (the
    device is re-enumerating) are retried. ``UsbDenied``, ``UsbHostUnavailable``
    and every other ``UsbHostError`` are permanent for this run and propagate at
    once, so a misconfigured bench does not wait out the budget. On exhaustion
    the last refusal is chained onto an error of the same kind that names the
    port, because the design requires the port and the refusal in the message.
    """
    ports = ", ".join(entry.port for entry in device.com_ports) or "none"
    deadline = time.monotonic() + timeout
    attempts = 0
    while True:
        attempts += 1
        try:
            return usb_host.serial(device, baud=baud)
        except (UsbConflict, UsbNotFound) as exc:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise type(exc)(
                    f"{device.key}: could not open {ports} within {timeout:.0f}s "
                    f"after {attempts} attempt(s); the last refusal was: {exc}"
                ) from exc
            time.sleep(min(interval, remaining))


def test_boots_and_reports_the_bridge(dut):
    """The RT-Thread banner, the main and bridge init lines, then the prompt.

    The lines are the source's, in print order: ``main`` logs the first and the
    last one, ``cdc_acm_uart_template.c`` logs the two between them as it opens
    UART2 and registers the USB descriptors. The prompt comes last, and the
    forward-only cursor then sits past the boot output for the later tests.
    """
    cherryusb.expect_all(
        dut,
        "- SiFli Corporation",
        "USB-UART Bridge starting...",
        "UART initialized successfully",
        "USB-UART bridge initialized",
        "USB-UART Bridge ready",
        cherryusb.PROMPT,
        timeout=30,
    )


def test_enumerates_with_its_own_serial(usb_host):
    """The host sees this example's VID/PID/iSerial and a CDC function.

    The serial matters as much as the VID/PID: every device example in this
    directory shares ``0x38F4:0xFFFF``, so a stale device from another example
    on the same bus would otherwise satisfy the wait.
    """
    device = _wait_for_device(usb_host)
    assert "cdc" in device.classes, (
        f"{device.key} enumerated with classes {device.classes!r}, no CDC function"
    )


def test_configured_event_is_reported(dut, usb_host):
    """The device prints the event the host's SET_CONFIGURATION produces.

    This is the console-side proof that this boot was configured by the host:
    the callback only runs when Windows itself sets the configuration. The
    event is read from the retained transcript through an explicit cursor at
    offset 0 rather than through ``dut.expect``, because Windows may configure
    the device before the boot walk above reaches its prompt and the shared
    cursor only moves forward -- a fresh ``expect`` would then wait out its
    timeout on a healthy bench. Reading from offset 0 also leaves the shared
    cursor untouched, so the line-coding test still sees only its own output.

    The device is waited for first: an event that is not in the transcript yet
    arrives while the host enumerates it. A transcript that never carries the
    event means the host did not configure this boot -- the "no host-visible
    re-enumeration after flash/reset" bench defect -- and the message says so.
    """
    _wait_for_device(usb_host)

    deadline = time.monotonic() + CONFIGURED_TIMEOUT
    transcript = ""
    matched = None
    while True:
        transcript, _ = dut.console.read_from(0, 0.0)
        matched = re.search(CONFIGURED_EVENT, transcript)
        if matched is not None:
            break
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(CONFIGURED_INTERVAL, remaining))

    assert matched is not None, (
        f"the console transcript has no /{CONFIGURED_EVENT}/ within "
        f"{CONFIGURED_TIMEOUT:.0f}s of the device enumerating: the host did not "
        "configure this boot. Replug the DUT's USB data cable during the "
        "enumeration wait.\n"
        "--- console transcript (tail) ---\n"
        f"{transcript[-2000:] or '<nothing>'}\n"
        "---------------------------------"
    )


def test_line_coding_reaches_the_uart(dut, usb_host):
    """The firmware receives the line coding of this open, at 115200.

    ``usbd_cdc_acm_set_line_coding`` logs the rate it was handed, so this ties
    the host's open to the device's USB control path rather than merely proving
    that a port exists. It does not prove the UART2 rate changed: the log is
    emitted before ``rt_device_control(RT_DEVICE_CTRL_CONFIG)`` is attempted
    (``cdc_acm_uart_template.c:469`` vs ``:485``), and no UART peer can observe
    the wire. Anything buffered before the open is discarded first, so the
    assertion can only be satisfied by the coding this test provokes.
    """
    device = _wait_for_device(usb_host)
    dut.drain()
    with _open_serial(usb_host, device, baud=BAUD):
        dut.expect(r"\[I/usb_uart\] Set baud rate: 115200", timeout=10)
