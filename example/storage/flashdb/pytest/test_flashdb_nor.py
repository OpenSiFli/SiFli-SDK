# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""FlashDB on NOR (FAL mode) -- the commands from the README, checked in order.

the "expected results" section in the README lists both the commands and their output, and
this suite walks the same path:
    kvdb set "key1" int 100      -> set the key1 value to 100
    kvdb get "key1" int          -> [key1] int / get the key1 value is 100
    kvdb set "key2" str "hello"  -> set key2 value to hello
    kvdb get "key2" str          -> [key2] str / get the key2 value is hello
    kvdb del "key1"              -> delete the key1 finish
    tsdb clear / append 1..3 / query_all / query_by_time
"""

import time

import pytest

PROMPT = r"msh [^>\n]*>"

STATE = {}


def _require(key: str) -> None:
    """Skip instead of failing when an earlier step did not get there."""
    if not STATE.get(key):
        pytest.skip(f"requires {key!r} from an earlier step")


def test_shell_is_alive(dut):
    dut.send_line("")
    dut.expect(PROMPT, timeout=30)
    STATE["shell"] = True


def test_kvdb_set_and_get_an_int(dut):
    _require("shell")

    dut.send_line('kvdb set "key1" int 100')
    dut.expect(r"set the key1 value to 100", timeout=30)
    dut.expect(PROMPT, timeout=30)

    dut.send_line('kvdb get "key1" int')
    dut.expect(r"\[key1\] int", timeout=30)
    dut.expect(r"get the key1 value is 100", timeout=30)
    dut.expect(PROMPT, timeout=30)

    STATE["int"] = True


def test_kvdb_set_and_get_a_string(dut):
    _require("shell")

    dut.send_line('kvdb set "key2" str "hello"')
    dut.expect(r"set key2 value to hello", timeout=30)
    dut.expect(PROMPT, timeout=30)

    dut.send_line('kvdb get "key2" str')
    dut.expect(r"\[key2\] str", timeout=30)
    dut.expect(r"get the key2 value is hello", timeout=30)
    dut.expect(PROMPT, timeout=30)

    STATE["str"] = True


def test_kvdb_values_survive_a_reboot(dut):
    """KVDB lives in flash, so both keys have to be there after a reboot."""
    _require("int")
    _require("str")

    dut.send_line("reboot")
    dut.expect("SFBL", timeout=60)
    dut.expect(r"kvdb init success !!!", timeout=120)
    dut.expect(PROMPT, timeout=60)

    dut.send_line('kvdb get "key1" int')
    dut.expect(r"get the key1 value is 100", timeout=30)

    dut.send_line('kvdb get "key2" str')
    dut.expect(r"get the key2 value is hello", timeout=30)
    # Consume the prompt this command printed, so the next test starts from a
    # cursor of its own.
    dut.expect(PROMPT, timeout=30)


def test_kvdb_delete_removes_the_key(dut):
    _require("int")

    dut.send_line('kvdb del "key1"')
    dut.expect(r"delete the key1 finish", timeout=30)
    dut.expect(PROMPT, timeout=30)

    dut.send_line('kvdb get "key1" int')
    dut.expect(r"get the key1 failed", timeout=30)
    dut.expect(PROMPT, timeout=30)


def test_tsdb_append_and_query(dut):
    """The README TSDB sample: clear -> append 1/2/3 -> query_all -> query_by_time."""
    _require("shell")

    dut.send_line("tsdb clear")
    dut.expect(r"clear tsdb\.", timeout=30)
    dut.expect(PROMPT, timeout=30)

    # TSDB orders entries by second, and entries written within one second end up
    # as one record (the count stays at 1); the README sample spaces them by 2-3
    # seconds, so wait a bit between appends here too.
    for value in (1, 2, 3):
        if value > 1:
            time.sleep(1.2)
        dut.send_line(f"tsdb append {value}")
        dut.expect(rf"append tsdb item : value = {value}", timeout=30)
        dut.expect(rf"tsdb count is: {value}", timeout=30)
        dut.expect(PROMPT, timeout=30)

    dut.send_line("tsdb query_all")
    dut.expect(r"query all:", timeout=30)
    for value in (1, 2, 3):
        dut.expect(rf"\[query_cb\] queried a TSL: value: {value} time: \d+", timeout=30)
    dut.expect(PROMPT, timeout=30)

    # The upper bound has to stay inside a signed 32-bit time_t; the device reads
    # it with strtoul, so 9999999999 would wrap to 1410065407 (2014) and the
    # query would quietly lose the "everything" meaning. 2147483647 is 2038.
    dut.send_line("tsdb query_by_time 0 2147483647")
    dut.expect(r"query by time:", timeout=30)
    dut.expect(r"query count is: 3", timeout=30)
    dut.expect(PROMPT, timeout=30)
