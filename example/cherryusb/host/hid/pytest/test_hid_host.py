# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of the CherryUSB HID host example, without a HID device.

The bench has no external keyboard or mouse, so this case cannot exercise
enumeration or report parsing of a real HID device. What it can prove is that
the host stack itself came up: the example prints its banner, the shell is
alive, ``lsusb`` answers with this CherryUSB version and a root hub, and the
hub thread the stack created is running. The absence of a mount line is the
other half of the no-device contract: nothing may claim a keyboard or a mouse
was mounted.

The strings come from the source (``src/main.c``, ``src/hid_host.c`` and
``external/cherryusb/core/usbh_core.c``), not from the example's README: its
expected output is stale and shows ``New low-speed device`` and ``Ep=81``
lines that the current source does not print.

``dut`` is session-scoped and its ``expect`` cursor only moves forward, so the
tests run in the order defined here: boot, quiet window, ``lsusb``, ``ps``.
The tests that type a command synchronise with the shell first, so the command
cannot be consumed as input by something that was still running.
"""

from __future__ import annotations

import re
import time

from sdk_test.suite_helpers import cherryusb

#: ``CHERRYUSB_VERSION_STR`` in ``external/cherryusb/common/usb_version.h``,
#: printed verbatim by ``lsusb -V``.
CHERRYUSB_VERSION = "CherryUSB version v1.6.0"

#: ``usbh_hub.c`` creates the thread as ``usbh_hub%u``, and ``src/main.c``
#: initialises bus 0. ``RT_NAME_MAX`` is 8 on this board, so the kernel keeps
#: and prints only ``usbh_hub``; the optional digit keeps the pattern correct
#: if a build raises the limit and the trailing digit becomes visible.
HUB_THREAD = r"usbh_hub\d*"

#: What ``usbh_hid_run`` prints for a keyboard or a mouse (``src/hid_host.c``).
#: A device-less run must print neither.
MOUNT_LINES = ("mount a keyboard", "mount a mouse")

#: How long the console is watched for a mount line. Long enough to cover the
#: first enumeration poll of the hub thread after the shell prompt, short
#: enough to keep the case's budget for the boot assertion.
QUIET_WINDOW_SECONDS = 2.0


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


def _ready(dut, *, timeout=10.0):
    """Synchronise with the shell before typing a command.

    An empty line makes the shell print a fresh prompt, so the command that
    follows cannot be consumed as input for something still running. The echo
    of the empty line is not asserted: echo is disabled on SF32LB52X/56X/57X,
    and a test that needed it would pass on 55X/58X for the wrong reason.
    """
    dut.send_line("")
    dut.expect(cherryusb.PROMPT, timeout=timeout)


def test_boots_the_hid_host_stack(dut):
    """The banner, the example's greeting, the HID entry point and the prompt.

    ``expect_all`` consumes the lines in print order because the console cursor
    only moves forward.
    """
    cherryusb.expect_all(
        dut,
        "- SiFli Corporation",
        "cherryusb host demo!",
        "hid host test",
        cherryusb.PROMPT,
        timeout=30,
    )


def test_no_hid_device_is_mounted(dut):
    """A quiet console window carries no mount line.

    No HID device is attached, so ``usbh_hid_run`` is never reached and neither
    ``mount a ...`` line can appear. The empty line comes first to prove the
    shell answers: without it, a dead console would satisfy the "no mount line"
    assertion for the wrong reason.

    The prompt is checked inside the collected window rather than with
    ``expect``, because ``expect`` consumes the whole chunk it matched in.
    A mount line printed in the same chunk as the prompt would then never
    reach ``_collect``, and the window would pass for the wrong reason.
    """
    dut.drain()
    dut.send_line("")
    window = _collect(dut, QUIET_WINDOW_SECONDS)

    assert re.search(cherryusb.PROMPT, window), (
        "the shell did not answer an empty line, so a quiet window proves "
        f"nothing:\n{window}"
    )
    for line in MOUNT_LINES:
        assert line not in window, (
            f"a HID device announced itself on a device-less bench: {line!r} "
            f"appeared in the {QUIET_WINDOW_SECONDS:.0f}s window:\n{window}"
        )


def test_host_stack_answers_lsusb(dut):
    """``lsusb`` reports this CherryUSB version and a root hub driven by ``hcd``."""
    _ready(dut)
    version = cherryusb.lsusb_version(dut, timeout=10)
    assert version == CHERRYUSB_VERSION, (
        f"lsusb -V answered {version!r}, expected {CHERRYUSB_VERSION!r}"
    )

    _ready(dut)
    dut.send_line("lsusb -t")
    dut.expect(r"Class=root_hub, Driver=hcd", timeout=10)


def test_usb_host_thread_is_alive(dut):
    """``ps`` lists the hub thread that ``usbh_initialize`` created.

    A banner is printed even when the controller never starts, so the thread is
    the evidence that the host stack is actually running. ``ps`` is the shell's
    thread list and carries the thread's name.
    """
    dut.drain()
    _ready(dut)
    dut.send_line("ps")
    dut.expect(HUB_THREAD, timeout=10)
