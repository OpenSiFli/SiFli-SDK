# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Join the access point the lab puts on the air, chosen at run time.

The image keeps the placeholder credentials compiled into ``src/main.c``
(``wifi_ssid`` / ``wifi_password``), so the example's own auto-connect thread
fails on purpose and the real join is issued here, from the shell. That is also
why this case is pytest and not console: console can only read the UART, while
"which network to join" is a run-time decision.

Credentials never live in this file, and no longer in the environment either.
The case declares ``fixture: wifi_ap:2g``; the runner resolves that spec's
options from a machine-local params file (see ``tools/sdk_test/fixture_types.yaml``
for the contract) and this suite reads them with ``dut.fixture_options``. Which
AP the bench offers is therefore a lab fact, not a code fact: a bench with no
such AP does not declare the connection, and the case is filtered out.

There is a second suite over the same example, ``pytest_peer/``, where the AP is
a peer board this SDK drives itself (``example/test/wifi_peer``). That one needs
no params at all and checks the link from both ends; keep this one for benches
that only have a real router.

``dut`` is session scoped and ``expect``'s cursor only moves forward, so the
order of the functions below is part of the test: shell, startup, scan, join,
status, ping.
"""

from __future__ import annotations

import re
import time

import pytest

#: The prompt is ``msh />`` when DFS has a working directory and ``msh >`` when
#: it does not; both are accepted.
PROMPT = r"msh [^>\n]*>"

#: The fixture spec this case was planned with. It is the identity the bench
#: connection must match exactly, because ``wifi_ap`` is a parameterized type:
#: the values behind it come from the params file, not from the binding.
FIXTURE_SPEC = "wifi_ap:2g"

#: Resolved options, filled in once by ``_ap`` below. ``ssid`` and ``psk`` are
#: what the join needs; an open network has no ``psk`` key at all, which is the
#: contract's way of saying "do not send a passphrase".
AP: dict = {}


@pytest.fixture(scope="session", autouse=True)
def _ap(dut):
    """Ask the runner for this spec's options once, before the first test."""
    AP.update(dut.fixture_options(FIXTURE_SPEC))
    yield AP

#: Any of these means the example's own auto-connect thread has returned. It
#: scans, tries the SSID compiled into the image and gives up; until it does it
#: holds the wlan management lock, and a ``wifi join`` issued meanwhile waits in
#: the shell thread for up to the driver's 30 s connect timeout, which reads as a
#: hung console.
AUTO_CONNECT_DONE = (
    r"wifi connect success ssid:"
    r"|\[WiFi\] Connect failed!"
    r"|not find ap!"
    r"|\[WiFi\] Device not ready"
)


def _wait_for_auto_connect(dut) -> None:
    dut.expect(AUTO_CONNECT_DONE, timeout=90)


#: What a later test needs from an earlier one. pytest runs the functions in file
#: order and the console cursor only moves forward, so the order is part of the
#: test: a join issued before the scan would fail for the wrong reason and then
#: hide the real one.
STATE: dict = {}


def _require(key: str):
    if STATE.get(key) is None:
        pytest.skip(
            f"needs STATE[{key!r}] from the earlier step; it did not run, so there "
            "is nothing to assert against"
        )


def test_shell_is_alive(dut):
    """The image booted far enough to give the shell the console."""
    dut.expect(PROMPT, timeout=30)


def test_auto_connect_thread_finishes(dut):
    """Wait for the example's own connection attempt to let go of the lock.

    Success, failure and an early "device not ready" all count: the point is that
    the thread returned, not how it ended.
    """
    _wait_for_auto_connect(dut)


def test_scan_sees_the_ap(dut):
    """``wifi scan`` lists the target SSID.

    Proving the AP is on the air first keeps a later join failure from being read
    as a driver problem.
    """
    dut.send_line("wifi scan")
    dut.expect(re.escape(AP["ssid"]), timeout=30)
    # ``wifi scan`` is blocking; the prompt means the table is complete and the
    # management lock is free again.
    dut.expect(PROMPT, timeout=15)
    STATE["scanned"] = True


def test_join_succeeds(dut):
    """``wifi join`` reaches ``wifi connect success ssid:...``.

    That line is printed by ``wlan_mgnt.c`` only when the station really
    associated, so it is stronger evidence than the example's own
    ``Connect request sent``, which is printed before the driver is asked.

    Retried, because losing a single association happens on a real network
    without anything being wrong with the example: the log of such an attempt
    shows the authentication frame going out (``sm_auth_send``,
    ``tx cfm:1,80800000,b0``), no ``auth_handler`` at all, then
    ``deauth_handler in a connection`` and ``wlan_join failed: -7`` -- an AP that
    never answered, or a channel busy enough to drop the exchange. A second
    attempt a few seconds later associates. Failing only after several attempts
    keeps the case from turning that into a red run, while a station that cannot
    associate at all still fails, with every attempt in the log.
    """
    _require("scanned")
    ssid = AP["ssid"]
    # An open network has no ``psk`` key -- the contract omits it rather than
    # sending an empty passphrase, which the driver would reject as a mistyped key.
    psk = AP.get("psk") or ""
    attempts = 3
    last: AssertionError | None = None
    for attempt in range(1, attempts + 1):
        dut.send_line("wifi disc")
        try:
            dut.expect(PROMPT, timeout=10)
        except AssertionError:
            # The prompt of the previous command may already have been consumed;
            # `wifi disc` still ran, so carry on rather than fail here.
            pass
        # A hotspot that has just been left wants a moment before it takes the
        # same station again; without the gap the authentication frame sometimes
        # goes unanswered and the attempt ends in ``deauth_handler`` + -7.
        time.sleep(1.5)
        dut.send_line(f"wifi join {ssid} {psk}" if psk else f"wifi join {ssid}")
        try:
            dut.expect(re.escape("wifi connect success ssid:" + ssid), timeout=90)
        except AssertionError as exc:
            last = exc
            continue
        dut.expect(PROMPT, timeout=30)
        STATE["joined"] = True
        return
    assert last is not None
    raise AssertionError(
        f"the DUT did not associate with {ssid!r} in {attempts} attempts; the "
        "device output of the last attempt is above"
    ) from last


def test_status_reports_the_link(dut):
    """``wifi status`` shows the SSID and a live RSSI."""
    _require("joined")
    dut.send_line("wifi status")
    dut.expect(r"Wi-Fi STA Info", timeout=30)
    dut.expect(r"SSID\s*:\s*" + re.escape(AP["ssid"]), timeout=15)
    dut.expect(r"RSSI: -?\d+", timeout=15)
    dut.expect(PROMPT, timeout=15)


def _wait_for_gateway(dut, *, attempts: int = 15, interval: float = 2.0) -> str:
    """Poll ``ifconfig`` until DHCP has produced a default gateway.

    The association completes seconds after ``wifi join`` returns, but the lease
    arrives later and ``ifconfig`` printed before it does not update itself, so a
    single read would mostly report ``0.0.0.0`` on a network that is working
    fine. The cursor only moves forward, which is what makes retrying the
    command -- rather than re-reading the buffer -- the way to see the change.
    """
    last = "0.0.0.0"
    for _ in range(attempts):
        dut.send_line("ifconfig")
        line = dut.expect(r"gw address:\s*\d+\.\d+\.\d+\.\d+", timeout=15)
        dut.expect(PROMPT, timeout=15)
        # ``gw address: 192.168.1.1`` -> the address is the last field.
        address = line.rsplit(None, 1)[-1]
        if address != "0.0.0.0":
            return address
        last = address
        time.sleep(interval)
    raise AssertionError(
        f"the DUT never got a default gateway (last read: {last}). The station is "
        "associated but its DHCP client is still at 0.0.0.0, so the AP is not "
        "answering. In device.log look for repeated 'tx_sched DHCP dn ... tot_len=350' "
        "with no 'Got IP address' line -- that means DISCOVER went out and nothing came "
        "back, which is an AP/DHCP problem (hotspot client limit, no DHCP server, or a "
        "downlink fault), not an association failure"
    )


def test_dut_gets_an_address_and_pings_the_gateway(dut):
    """DHCP on this real network, then ICMP to the default gateway.

    Covers against a real router what ``pytest_peer/`` proves against our own AP:
    the station gets an address and can exchange data frames with something that
    is not itself. The ping is what separates "associated" from "usable" -- the
    example's own ``Connect request sent`` is printed before the driver is asked
    anything.
    """
    _require("joined")
    gateway = _wait_for_gateway(dut)

    # `ping <host> [netdev name] [times] [data size]` (netdev.c:1417-1437): the
    # count is the third positional, the second one is an interface name.
    dut.send_line(f"ping {gateway} w0 4")
    dut.expect(r"\d+ bytes from " + re.escape(gateway) + r" icmp_seq=", timeout=30)
    dut.expect(r"4 packets transmitted, 4 received, 0% packet loss", timeout=30)
    dut.expect(PROMPT, timeout=15)
    STATE["usable"] = True


def test_weather_command_reports_the_forecast(dut):
    """``weather`` fetches the forecast over HTTP (webclient + mbedTLS + cJSON).

    This is the weather item of the example's sample output: the command
    prints a ``id:`` / ``name:`` / ``txt:`` / ``temperature:`` block parsed from
    the Seniverse JSON. It needs real internet access *and* the API key compiled
    into ``src/wheather.c`` to still be valid, so the documented failure messages
    turn into a skip -- with the device's own words in the reason -- instead of a
    red case that says nothing about this example's code.
    """
    _require("usable")

    # MSH_CMD_EXPORT(weather, Get Weather) -- src/wheather.c:258.
    dut.send_line("weather")
    first = dut.expect(
        r'(id:"[^"]*"'
        r"|Coud not find [^,]+, please check PAN connection"
        r"|webclient GET request failed, response\(\d+\) error\.)",
        timeout=60,
    )
    if not first.startswith("id:"):
        pytest.skip(
            "weather needs internet and a live API key (api.seniverse.com); the "
            f"device said: {first!r}"
        )

    # http_weather_data_parse() prints these in this order (wheather.c:150-159).
    dut.expect(r'name:"[^"]*"', timeout=20)
    dut.expect(r'txt:"[^"]*"', timeout=20)
    dut.expect(r'temperature:"-?\d+"', timeout=20)
    dut.expect(r'last_update:"[^"]*"', timeout=20)
    dut.expect(PROMPT, timeout=20)
    STATE["weather"] = True


def test_disconnect_reports_success(dut):
    """``wifi disc`` drops the link and says so, as the sample output shows.

    ``rt_wlan_disconnect()`` prints ``disconnect success!`` itself
    (wlan_mgnt.c:1139), so this does not depend on the driver reporting a
    passive deauthentication -- that path is still unimplemented for this part.
    """
    _require("joined")
    dut.send_line("wifi disc")
    dut.expect(r"disconnect success!", timeout=30)
    dut.expect(PROMPT, timeout=20)
