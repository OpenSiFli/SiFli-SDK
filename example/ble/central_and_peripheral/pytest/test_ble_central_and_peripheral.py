# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end check of the BLE central-and-peripheral example.

The image is both a GAP peripheral and a GAP central. At power on it registers
its 128-bit GATT service and starts connectable extended advertising; the
``diss`` shell command then drives scanning, the connection table and the
advertising data.

All of that is observable from this one board, and the tests below check it in
the order the firmware produces it. What is *not* checked, and cannot be with a
single board: connecting, service discovery, read/write and notification need a
second BLE device. The framework does not schedule two boards in one case, so
those paths stay out of scope -- a test for them here would be a test of the
peer, not of the example.

``dut`` is session-scoped and its ``expect`` cursor only moves forward, so the
tests run in the order defined here and each starts where the previous one
stopped. Nothing asserts on the echoed command text: ``FINSH_ECHO_DISABLE_DEFAULT``
is on for SF32LB52X/56X/57X and a test that depended on echo would pass on
55X/58X for the wrong reason.
"""

from __future__ import annotations

import re

#: The prompt is ``msh />`` when DFS keeps a working directory and ``msh >``
#: when it does not. Accept both rather than pinning one carrier's spelling.
PROMPT = r"msh [^>\n]*>"

#: The modes the stack reports in ``sibles_adv_evt_startted_t``, from
#: ``enum sibles_adv_running_mode_t`` in ``bf0_sibles_advertising.h``. The
#: example always starts extended advertising (``USING_LE_EXTENSION_ADV`` in
#: ``src/main.c``), so a connectable advertisement must report this one.
ADV_MODE_EXTENDED = 5


def test_boot_reaches_the_shell(dut):
    """The RT-Thread banner appears and the shell prints its prompt.

    Both are printed before pytest attaches, but the runner opens the console
    first, so they are still in the retained buffer -- the same way the other
    pytest cases in the tree see their boot output.
    """
    dut.expect("- SiFli Corporation", timeout=30)
    dut.expect(PROMPT, timeout=20)


def test_ble_powers_on_and_advertising_starts(dut):
    """The stack comes up and the peripheral role is actually advertising.

    Two separate facts, asserted in the order they happen:

    * ``receive BLE power on!`` is printed by ``main()`` once the stack signals
      that it is ready and the GATT service has been registered -- this is the
      line right after ``ble_app_service_init()``.
    * ``ADV start resutl 0`` is the advertising event handler's report on the
      controller's confirmation, so status 0 means the advertisement really is
      running, not merely requested.

    The order is safe rather than racy: the first line is written on the app
    thread immediately after the (asynchronous) start request, while the second
    can only arrive once the controller has answered. Expecting the immediate
    line first therefore never skips past the later one.
    """
    dut.expect(r"receive BLE power on!", timeout=30)

    match = re.search(
        r"ADV start resutl (\d+), mode (\d+)",
        dut.expect(r"ADV start resutl \d+, mode \d+", timeout=30),
    )
    assert match is not None  # the pattern above guarantees this

    status, mode = int(match.group(1)), int(match.group(2))
    assert status == 0, f"advertising failed to start, status {status}"
    assert mode == ADV_MODE_EXTENDED, (
        f"advertising runs in mode {mode}, expected extended ({ADV_MODE_EXTENDED})"
    )


def test_diss_lists_its_commands(dut):
    """An unrecognised subcommand falls through to the usage list.

    This proves the command is reachable through msh and that the example's own
    command table is wired up; two entries are enough to identify the list, and
    longer lines are avoided because ``RT_CONSOLEBUF_SIZE`` is 128 bytes.
    """
    dut.send_line("diss help")
    dut.expect(r"Command list:", timeout=10)
    dut.expect(r"diss scan\. Scan advertising device", timeout=10)
    dut.expect(r"diss show_dev\. Show all connected device", timeout=10)
    dut.expect(PROMPT, timeout=10)


def test_central_role_scans(dut):
    """The central role starts a scan and the scan stops when its time is up.

    ``Scan start status 0`` is the controller's confirmation of the scan
    request, and ``Scan stopped`` follows when the 2 s duration expires -- both
    from the CONFIRM/INDICATION handlers, so neither can be printed by a stack
    that did not run them.

    No prompt is expected after the scan: the shell returns as soon as the
    asynchronous start request is queued, so its prompt appears before the scan
    even begins. That is also why this test leaves the next one to wait for its
    own prompt instead of consuming this one.
    """
    dut.send_line("diss scan start 0 100 50 2000 -127")
    dut.expect(r"Scan start status 0", timeout=10)
    dut.expect(r"Scan stopped", timeout=15)


def test_connection_table_is_empty(dut):
    """Nothing has connected to the board, and the example reports exactly that.

    This is the state the peripheral side should be in while advertising: the
    connection-manager bookkeeping starts at zero and only the connect path can
    raise it.
    """
    dut.send_line("diss show_dev")
    dut.expect(r"Total connected 0 devices", timeout=10)
    dut.expect(PROMPT, timeout=10)


def test_advertising_accepts_new_data_after_scanning(dut):
    """The advertisement is still running once the scan has finished.

    ``sibles_advertising_update_adv_and_scan_rsp_data`` returns
    ``SIBLES_ADV_NO_ERR`` (0) only when the context is in the STARTED state and
    refuses with ``SIBLES_ADV_NOT_ALLOWED`` otherwise, so this ``0`` is a second
    independent reading of "advertising is on" -- after the scan, which is the
    step most likely to have disturbed it.
    """
    dut.send_line("diss adv_update")
    dut.expect(r"update adv 0", timeout=10)
    dut.expect(PROMPT, timeout=10)
