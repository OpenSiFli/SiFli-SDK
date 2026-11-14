# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of the CherryUSB CDC ACM host example.

This round has no external USB device on the bench, so the suite covers what
the firmware can show without one: the boot banner, the no-device branch of the
single ``ttyACM0`` probe, and the host stack answering its own ``lsusb`` shell
command. The README's "find a device and receive data" path needs a real CDC
peer and is recorded as unverified in ``testcase.yaml`` rather than skipped
here.

All strings come from the source, not the README: several READMEs in this
family are stale. ``src/main.c:75-85`` prints exactly one of
``ttyACM0 found!`` / ``ttyACM0 not found!`` about 2 s after boot and never
retries.

The tests run in the order defined here, because ``dut`` is session-scoped and
its ``expect`` cursor only moves forward.
"""

from __future__ import annotations

from sdk_test.suite_helpers import cherryusb

#: ``src/main.c:81``; printed once, about 2 s after boot, with no retry.
NO_DEVICE = r"ttyACM0 not found!"

#: ``external/cherryusb/common/usb_version.h`` compiles this in. Pinned on
#: purpose: a stack upgrade that changes the version should be a deliberate
#: update of this case, not a silent change of what the bench runs.
CHERRYUSB_VERSION = "CherryUSB version v1.6.0"

#: ``usbh_hub.c`` creates ``usbh_hub<busid>``. ``RT_NAME_MAX`` is 8 on this
#: SDK, so the name is stored and printed as ``usbh_hub``; the optional digit
#: keeps the pattern correct if a build raises ``RT_NAME_MAX``.
HOST_HUB_THREAD = r"usbh_hub\d*"

#: The root hub line of ``lsusb -t`` (``usbh_core.c:980``). Asserted whole,
#: from ``Class=`` through the driver field, so it is one independent match
#: rather than a second assertion riding on the first.
ROOT_HUB = r"Class=root_hub, Driver=hcd"


def _ready(dut, *, timeout: float = 10.0) -> None:
    """Synchronise with the shell before typing a command.

    An empty line makes the shell print a fresh prompt, so the command that
    follows cannot be consumed as input for something that was still running.
    The echo of the empty line is not asserted: echo is disabled on
    SF32LB52X/56X/57X and a test that needed it would pass on 55X/58X for the
    wrong reason.
    """
    dut.send_line("")
    dut.expect(cherryusb.PROMPT, timeout=timeout)


def test_boots_and_prints_the_demo_banner(dut):
    """The RT-Thread banner, the example's greeting and the shell prompt.

    ``expect_all`` consumes them in print order because the console cursor only
    moves forward.
    """
    cherryusb.expect_all(
        dut,
        "- SiFli Corporation",
        "cherryusb host demo!",
        cherryusb.PROMPT,
        timeout=30,
    )


def test_reports_no_cdc_device(dut):
    """With no USB device attached, the one-shot probe reports its absence.

    The 15 s window covers the 2000 ms delay before the probe plus console
    latency. This test deliberately does not accept ``ttyACM0 found!``: a
    device found here means the bench is not in the state this case describes,
    and the README's data path would then need its own assertions.
    """
    dut.expect(NO_DEVICE, timeout=15)


def test_host_stack_answers_lsusb(dut):
    """``lsusb`` is registered and reports the stack version and root hub.

    The version line comes from ``lsusb -V`` (``usbh_core.c:1114``); the root
    hub line from ``lsusb -t`` (``usbh_core.c:980``). The root hub existing in
    the bus dump is the evidence that ``usbh_initialize`` brought the host
    stack up; the banner alone would also appear on a board where it did not.
    """
    _ready(dut)
    version = cherryusb.lsusb_version(dut, timeout=10)
    assert version == CHERRYUSB_VERSION, (
        f"lsusb -V said {version!r}, expected {CHERRYUSB_VERSION!r}"
    )

    _ready(dut)
    dut.send_line("lsusb -t")
    dut.expect(ROOT_HUB, timeout=10)


def test_usb_host_thread_is_alive(dut):
    """The host hub thread exists in the RT-Thread thread table.

    ``usbh_initialize`` (called from ``main``) creates ``usbh_hub<busid>``;
    ``ps`` lists it even with no device attached, which distinguishes a live
    host stack from a shell that only answers commands.
    """
    _ready(dut)
    dut.send_line("ps")
    dut.expect(HOST_HUB_THREAD, timeout=10)
