# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Console-side checks for the CherryUSB MSC host example.

The bench must leave this board's USB host port empty. With no USB mass storage
device attached, the suite covers what is observable without one: the example
starts, the host controller's root hub comes up, the shell answers ``lsusb``,
the hub thread is alive, and the firmware keeps scanning instead of mounting.
A U-disk left plugged in makes the firmware stop scanning and report a mount,
so the case fails by design; the assertions must not be relaxed for that bench.
The mount and write paths are recorded as unverified in ``testcase.yaml``.

The expected strings are the source's, not the README's: the README still says
``Waiting for USB mass storage device connection...`` while ``src/main.c``
prints ``Scanning for USB mass storage devices...``.
"""

from __future__ import annotations

import time

from sdk_test.suite_helpers import cherryusb

#: ``src/main.c`` prints this once per second while no ``/dev/sd?`` exists.
SCANNING = "Scanning for USB mass storage devices..."

#: How long the "no device" window is observed. This is the observation
#: interval itself, not a sleep used to synchronize with an event: the scan
#: repeats every second, so a working firmware yields the line many times and a
#: stopped one yields none.
SCAN_WINDOW = 15.0

#: Lines that only a mounted volume produces; their absence is half the check.
MOUNT_MARKERS = ("mount successfully", "File content:")

#: ``common/usb_version.h`` is the single source of the stack version.
LSUSB_VERSION_LINE = "CherryUSB version v1.6.0"

#: ``usbh_core.c`` prints ``Class=root_hub, Driver=hcd`` on the ``lsusb -t``
#: root line. The shared ``cherryusb.LSUSB_ROOT_HUB`` stops at ``root_hub``, so
#: the driver name that proves the host controller is bound is asserted here.
ROOT_HUB_DRIVER = r"Class=root_hub, Driver=hcd"

#: ``usbh_hub.c`` creates ``usbh_hub%u``. ``RT_NAME_MAX`` is 8 in this build,
#: so ``ps`` shows the truncated prefix ``usbh_hub``; the optional digit keeps
#: the pattern correct if a build raises the limit and the digit becomes visible.
HUB_THREAD = r"usbh_hub\d*"


def _ready(dut, *, timeout=10.0):
    """Synchronise with the shell before typing a command.

    An empty line makes the shell print a fresh prompt, so the command that
    follows cannot be consumed as input for something that was still running.
    The echo of the empty line is not asserted: echo is disabled on
    SF32LB52X/56X/57X, and a test that needed it would pass on 55X/58X for the
    wrong reason.
    """
    dut.send_line("")
    dut.expect(cherryusb.PROMPT, timeout=timeout)


def _collect(dut, seconds):
    """Read the console for ``seconds`` and return everything it said.

    ``read`` returns as soon as any bytes arrive, so a single call would not
    cover the whole window; the deadline is what makes it a window.
    """
    text = ""
    deadline = time.monotonic() + seconds
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return text
        text += dut.read(min(0.5, remaining))


def _retained_transcript(dut):
    """The console's retained transcript, read without moving the session cursor.

    ``Dut.expect`` advances the session cursor past whatever it matched, so a
    mount line printed before the boot prompt is behind that cursor by the time
    this test runs. Reading from offset zero through the public
    ``RemoteConsole.read_from`` sees the retained boot output anyway, which is
    what lets the no-mount claim cover boot as well as the quiet window.
    """
    data, _cursor = dut.console.read_from(0, 0.0)
    return data


def test_boots_and_prints_the_demo_banner(dut):
    """The RT-Thread banner, the example's greeting and the shell prompt.

    ``expect_all`` consumes the strings in print order because the console
    cursor only moves forward.
    """
    cherryusb.expect_all(
        dut,
        "- SiFli Corporation",
        "cherryusb host demo!",
        cherryusb.PROMPT,
        timeout=30,
    )


def test_scans_for_storage_without_a_device(dut):
    """The firmware keeps scanning and never claims a volume was mounted.

    The boot transcript is read from the console's retained buffer first: the
    banner test necessarily moved the session cursor past the boot output, and
    a device attached at boot would have reported its mount there. The console
    is then drained and the following 15 s are collected whole, so the window
    assertion is as much about what did not appear as about the scan line.
    Both transcripts are checked for mount markers.
    """
    boot = _retained_transcript(dut)
    dut.drain()
    window = _collect(dut, SCAN_WINDOW)

    assert SCANNING in window, (
        f"{SCAN_WINDOW:.0f}s of console output did not contain {SCANNING!r}:\n"
        f"{window!r}"
    )
    for marker in MOUNT_MARKERS:
        assert marker not in window, (
            f"the host reported {marker!r} during the "
            f"{SCAN_WINDOW:.0f}s quiet window:\n{window!r}"
        )
    for marker in MOUNT_MARKERS:
        assert marker not in boot, (
            f"the host reported {marker!r} in the retained boot transcript:\n"
            f"{boot!r}"
        )


def test_host_stack_answers_lsusb(dut):
    """``lsusb`` reports the stack version and a bound root hub."""
    _ready(dut)
    version = cherryusb.lsusb_version(dut, timeout=10)
    assert version == LSUSB_VERSION_LINE, (
        f"lsusb -V reported {version!r}, expected {LSUSB_VERSION_LINE!r}"
    )

    _ready(dut)
    dut.send_line("lsusb -t")
    dut.expect(ROOT_HUB_DRIVER, timeout=10)


def test_usb_host_thread_is_alive(dut):
    """``ps`` lists the hub thread the host stack needs to poll ports."""
    _ready(dut)
    dut.send_line("ps")
    dut.expect(HUB_THREAD, timeout=10)
