# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Acceptance test for the ``pytest`` harness: drive hello_world from Python.

Same firmware as the ``console`` case next door, but the verdict is a Python
program rather than a list of regexes. That contrast is the point of this test:
it exercises the escape hatch a test takes when a linear set of expectations is
not enough, and it proves the three pieces line up --

* ``sdk.py test run`` allocates a testbed and builds the project,
* the ``dut`` fixture attaches to *that* testbed (not a second one),
* and pytest's exit code becomes the harness verdict.

The tests below are deliberately the kind that ``record:`` cannot express: they
send a command and assert on the *reply*, which depends on what the device did
first.
"""

from __future__ import annotations


def test_boot_banner_and_greeting(dut):
    """The device boots, prints its banner, and reaches the greeting."""
    assert dut.expect("Hello world!", timeout=30), "the greeting never appeared"
    assert dut.expect(r"msh />", timeout=20), "the shell prompt never appeared"


def test_the_console_is_bidirectional(dut):
    """A command sent to the shell comes back with a fresh prompt.

    This is what a ``console`` test cannot do: the prompt asserted on here only
    means something *because* a line was sent first, so the assertion depends on
    the conversation rather than on a fixed string appearing somewhere.

    Nothing is expected before the send, and that is deliberate: an expectation
    covers output from *this test onward*, so the prompt the boot left behind
    would not satisfy it. Getting that wrong is how this test used to pass
    without proving anything.
    """
    # An empty line is the least assuming command there is: it needs no
    # particular shell command to exist, and any live shell answers it.
    dut.send_line("")

    assert dut.expect(r"msh />", timeout=10), "the shell did not answer"


def test_the_platform_is_visible_to_the_suite(dut):
    """The runner tells the suite which board it landed on.

    A suite that has to behave differently per board needs this, and getting it
    from the environment is better than guessing from the transcript.
    """
    assert dut.testbed_id, "the fixture did not report a testbed"
    assert dut.platform, "the fixture did not report a platform"
