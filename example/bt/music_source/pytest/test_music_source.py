# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end local-playback path for the music_source example.

The DUT mounts the pre-installed disk image and plays `disk/test.mp3` through
its own audio path, so no Bluetooth peer is involved. The A2DP Source / AVRCP
checkpoints need a real A2DP Sink and are not covered here; the reason is in
the example README and the design discussion.

The bundled track is about one minute long, so the suite stops it once the
first playback has started instead of waiting for the end. Order matters: the
DUT's console cursor only moves forward, so each test consumes the lines it
provokes and leaves the cursor after them.
"""

from __future__ import annotations

import pytest

from sdk_test.suite_helpers import sifli_app

#: Filled by one test and read by the next; a missing entry means an earlier
#: test did not complete, which is an error rather than a skip.
STATE: dict = {}


def _require(key: str):
    value = STATE.get(key)
    if value is None:
        raise AssertionError(f"{key} was not set: an earlier test did not complete")
    return value


def test_dut_boots_mounts_and_lists_the_track(dut):
    """The filesystem comes up and the bundled track is on it."""
    dut.expect("- SiFli Corporation", timeout=30)

    # mnt_init() is INIT_ENV_EXPORT, so its lines come before main(). A
    # factory-fresh partition takes the mkfs branch; both branches end mounted.
    line = dut.expect(r"mount fs on flash to root (success|fail)", timeout=30)
    if "fail" in line:
        dut.expect(r"make elm fs on flash sucess, mount again", timeout=10)
        dut.expect(r"mount fs on flash success", timeout=10)

    dut.expect(r"\[Music source\]Music source Example\.", timeout=20)

    # main() lists the root partition before it hands over to the shell.
    dut.expect(r"Directory /:", timeout=20)
    # ls() pads the name and right-aligns the size on the same line; matching
    # both proves the bundled file, not just its name.
    dut.expect(r"test\.mp3\s+960887", timeout=10)

    # The prompt is the readiness gate for the commands below. The main thread
    # outranks the finsh thread, so it prints the listing first.
    dut.expect(sifli_app.PROMPT, timeout=30)
    STATE["booted"] = True


def test_play_default_starts_the_bundled_track(dut):
    """`music play_default` opens and starts the bundled track."""
    _require("booted")
    dut.send_line("music play_default")

    dut.expect(r"\[LOCAL MUSIC\]play_file /test\.mp3", timeout=10)
    dut.expect(r"mp3 open /test\.mp3", timeout=15)
    dut.expect(r"mp3 run\.\.\.", timeout=15)
    STATE["playing"] = True


def test_stop_interrupts_playback(dut):
    """`music stop` closes the decoder instead of waiting out the track."""
    _require("playing")
    dut.send_line("music stop")

    dut.expect(r"\[LOCAL MUSIC\]play_stop", timeout=10)
    # mp3ctrl_close() blocks until the decode thread reports its exit, so this
    # line also means the audio client has been released.
    dut.expect(r"mp3 exit\.\.nFrames=\d+", timeout=20)
    STATE["stopped"] = True


def test_play_with_an_explicit_path(dut):
    """`music play 0 /test.mp3` takes the explicit-path branch."""
    _require("stopped")
    dut.send_line("music play 0 /test.mp3")

    dut.expect(r"\[LOCAL MUSIC\]play_file /test\.mp3", timeout=10)
    # Consumed here for two reasons: it proves playback really started, and it
    # leaves the cursor past this run so the next test's negative check cannot
    # match it.
    dut.expect(r"mp3 open /test\.mp3", timeout=15)
    dut.expect(r"mp3 run\.\.\.", timeout=15)
    STATE["explicit_playing"] = True


def test_a_missing_file_is_reported_and_does_not_start_playback(dut):
    """A nonexistent path fails at open and never reaches the decode loop.

    Opening a new file first closes the one playing, so the cursor is placed
    after `MP3 open failed!!` before the negative check: every line belonging
    to the previous playback is behind it.
    """
    _require("explicit_playing")
    dut.send_line("music play 0 /nofile.mp3")

    dut.expect(r"\[LOCAL MUSIC\]play_file /nofile\.mp3", timeout=10)
    # The decoder logs `mp3 open  error fd=-1` (the path is not part of that
    # line even though the successful open prints it).
    dut.expect(r"mp3 open\s*error fd=-1", timeout=15)
    dut.expect(r"MP3 open failed!!", timeout=10)

    with pytest.raises(AssertionError) as failure:
        dut.expect(r"mp3 run\.\.\.", timeout=2)
    # Only the timeout is expected; an unrelated failure must not be swallowed.
    assert "mp3 run" in str(failure.value), (
        f"unexpected failure while checking for a new mp3 run: {failure.value}"
    )
