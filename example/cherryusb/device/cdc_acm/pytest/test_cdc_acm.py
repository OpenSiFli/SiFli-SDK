# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of the CherryUSB CDC ACM device example.

The device enumerates as a CDC ACM serial port over USB. What Windows saw --
the identity, the COM port, the bytes of each frame -- is asked of the
``usb_host`` fixture, which talks to the Lab Agent that owns the USB port; the
firmware's own console supplies the device-side half of each check.

``usb_host`` has no cursor, and the three host-side tests re-acquire the device
through ``usb_host.wait_for`` rather than carrying state over, so they can run
individually. The banner test is different: it needs the console to have been
opened before the board reset, which the runner arranges, and a direct
invocation can deselect it. Tests execute in pytest's collection order.
"""

from __future__ import annotations

import time

from sdk_test.suite_helpers import cherryusb
from sdk_test.usb_host import UsbConflict
from sdk_test.usb_host import UsbNotFound

#: Identity ``src/cdc_acm_template.c`` puts in the device and string
#: descriptors.
VID = 0x38F4
PID = 0xFFFF
SERIAL = "2022123456"

#: The frame is 10 digits followed by padding; see ``cdc_acm_init``.
FRAME_LEN = 2048
FRAME_PREFIX = b"1234567890"
FRAME_PADDING = b"a" * (FRAME_LEN - len(FRAME_PREFIX))

#: The payload of the bulk OUT test; exactly seven bytes.
OUT_PAYLOAD = b"SiFli\r\n"

#: Bounded retry for the first open. The port can be reported as enumerated
#: while a handle still holds it, typically a session left over from an earlier
#: run; the design's board baseline saw a first open being refused and clearing
#: later. Only refusals that can clear on their own are retried, and the final
#: one is reported with the port name instead of being read as a firmware
#: failure.
OPEN_TIMEOUT = 30.0
OPEN_INTERVAL = 1.0


def _wait_for_device(usb_host):
    """Return this example's device, waiting for it to enumerate."""
    return usb_host.wait_for(
        vid=VID,
        pid=PID,
        serial=SERIAL,
        require_com=True,
        timeout=30,
    )


def _open_serial(usb_host, device, *, timeout=OPEN_TIMEOUT, interval=OPEN_INTERVAL):
    """Open the device's CDC port, retrying a refusal that can clear on its own.

    Only two refusals are retried: ``UsbConflict`` (the port is still held,
    e.g. by a session from an earlier run) and ``UsbNotFound`` (the device is
    re-enumerating). ``UsbDenied``, ``UsbHostUnavailable`` and any other
    ``UsbHostError`` are permanent for this run and propagate at once, so a
    misconfigured bench does not wait out the budget. On exhaustion the last
    refusal is chained onto an error of the same kind that names the port.
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


def test_boots_and_prints_the_demo_banner(dut):
    """The RT-Thread banner, the example's greeting and the shell prompt.

    The strings are the source's, not the README's; ``expect_all`` consumes
    them in print order because the console cursor only moves forward.
    """
    cherryusb.expect_all(
        dut,
        "- SiFli Corporation",
        "cherryusb device cdc acm demo!",
        cherryusb.PROMPT,
        timeout=30,
    )


def test_enumerates_as_a_cdc_acm_device(usb_host):
    """The host sees the expected VID/PID/serial and a CDC serial function.

    ``_wait_for_device`` already requires a COM port, so that half of the
    contract is enforced by the lookup rather than asserted a second time.
    """
    device = _wait_for_device(usb_host)
    assert "cdc" in device.classes, (
        f"{device.key} enumerated with classes {device.classes!r}, no CDC function"
    )


def test_sends_frames_when_dtr_is_asserted(dut, usb_host):
    """DTR starts the periodic transfer; the frame is digits then 'a' padding."""
    device = _wait_for_device(usb_host)
    with _open_serial(usb_host, device) as serial:
        # Only output produced from here on may satisfy the console assertion,
        # so a direct run that reuses a console stream does not pass on a line
        # left by an earlier DTR session.
        dut.drain()
        serial.set_dtr(True)
        frame = serial.read_exact(FRAME_LEN, timeout=5)
        prefix = frame[: len(FRAME_PREFIX)]
        assert prefix == FRAME_PREFIX, (
            f"the first {len(FRAME_PREFIX)} bytes are {prefix!r}, "
            f"expected {FRAME_PREFIX!r}"
        )
        padding = frame[len(FRAME_PREFIX) :]
        assert padding == FRAME_PADDING, (
            f"bytes after the prefix are not 'a' padding: {padding[:32]!r}..."
        )
        # The firmware also reports that a 2048-byte IN transfer completed.
        # That is evidence of a transfer of this size on the device side, not
        # proof that it is the frame just read above.
        dut.expect(r"actual in len:2048", timeout=5)


def test_bulk_out_is_logged(dut, usb_host):
    """A host write reaches the firmware's bulk OUT callback with its length."""
    device = _wait_for_device(usb_host)
    with _open_serial(usb_host, device) as serial:
        dut.drain()
        written = serial.write(OUT_PAYLOAD)
        assert written == len(OUT_PAYLOAD), (
            f"wrote {written} of {len(OUT_PAYLOAD)} bytes to {serial.port}"
        )
        dut.expect(r"actual out len:7", timeout=5)
