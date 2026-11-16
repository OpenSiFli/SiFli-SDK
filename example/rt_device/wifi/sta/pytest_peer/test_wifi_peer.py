# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""The STA example against a Wi-Fi peer board that offers the access point.

Why this suite exists at all: ``pytest/test_wifi_join.py`` joins whatever
network the lab happens to have on the air, so it can prove "the DUT was pointed
at an SSID" but not "the DUT associated with *our* AP". Here the AP is the peer
board (``example/test/wifi_peer``), driven over the instrument protocol, so both
ends of the link are ours and the evidence can be two-sided:

* the peer reports its own MAC (the BSSID) in ``ap.status``;
* the DUT prints the BSSID it joined as ``MAC Addr:`` in ``wifi status``;
* a DUT that really associated also completes DHCP and gets an address from the
  peer's own server (``192.168.169.0/24``);
* that address is usable: the DUT pings the peer's AP address and gets every
  reply back, which is the only step that proves the ICMP path both ways;
* stopping the AP drops the DUT's link, which a "joined some other network of
  the same name" cannot do.

Credentials come from ``WIFI_PEER_SSID`` / ``WIFI_PEER_KEY`` so a lab whose
regulations forbid committed secrets can override them without editing this
file; the defaults match what ``example/test/wifi_peer`` documents.

Nothing here asserts on ``sta_num``: the AIC8800 driver does not forward
association indications to the wlan framework, so the peer's station count stays
0 even with the DUT attached.
"""

from __future__ import annotations

import os
import re

import pytest

PROMPT = r"msh [^>\n]*>"

SSID = os.environ.get("WIFI_PEER_SSID", "sifli_test")

#: ``WIFI_PEER_KEY`` is how a lab overrides the peer's passphrase. "open", "-"
#: and an empty value all mean "start the AP without a passphrase" -- the empty
#: one because Windows cannot hold an empty environment variable (PowerShell's
#: ``$env:WIFI_PEER_KEY = ""`` deletes it), and a suite that silently fell back
#: to the default passphrase would report on a network it never configured.
_KEY_ENV = os.environ.get("WIFI_PEER_KEY", "12345678").strip()
KEY = "" if _KEY_ENV.lower() in ("", "open", "-", "none") else _KEY_ENV

#: Whether the peer's SoftAP can send host-originated frames -- the DHCP
#: OFFER/ACK, and the deauth ``ap.stop`` emits.
#:
#: Kept as a switch because the failure it describes is what a bench with two
#: identical addresses looks like. Every AIC8800 module here reports the same
#: burned-in MAC (``88:00:33:77:69:22``), so an AP that keeps that address sends
#: its unicast frames to the station's identical address and the chip drops them
#: as "addressed to myself". Scan and association keep working (those frames come
#: from the firmware), which made it look like a dead transmit path.
#:
#: Two things make it work now, both on the peer side -- the DUT example is left
#: alone: the AP takes a locally administered address (``PEER_MAC``), and
#: ``peer_ap_apply_mac()`` copies that address into lwIP's ``netif->hwaddr``
#: too, because lwIP stamps the source of every data frame from its own copy.
#: With only the first step the AP is two-faced: management frames carry the new
#: address, DHCP/ARP/ICMP the burned-in one.
#:
#: Set this back to True only to keep the suite green on a bench where the
#: addresses cannot be made distinct.
PEER_AP_DOWNLINK_BROKEN = False

PEER_AP_DOWNLINK_REASON = (
    "peer AP downlink is unavailable on this bench: both boards share the "
    "burned-in MAC, so unicast frames the AP sends to the station are dropped as "
    "self-addressed; see wifi_peer/README.md. Scan, association and BSSID are "
    "still covered by this run; the lease and the disconnect are not."
)

#: The BSSID the peer must use. Every AIC8800 module in this lab reports the
#: same burned-in MAC (``88:00:33:77:69:22``), the DUT's own address included,
#: and a station does not scan or associate with a BSSID that is its own
#: address. Giving the peer a locally administered address keeps the two boards
#: distinct on the air. The firmware's default is the same value; passing it
#: here keeps the expectation explicit and overridable per bench.
PEER_MAC = os.environ.get("WIFI_PEER_MAC", "02:00:5e:00:00:01")

#: The address the peer's DHCP server gives itself, and therefore the address the
#: DUT can ping. ``dhcp_server.c`` uses it by default (``DHCPD_SERVER_IP``), and
#: the peer's ``ifconfig`` reports it on ``w1``; override it per bench the same
#: way the peer firmware's ``CONFIG_DHCPD_SERVER_IP`` is overridden.
PEER_AP_ADDRESS = os.environ.get("WIFI_PEER_AP_ADDRESS", "192.168.169.1")

#: Any of these means the example's own auto-connect thread has finished. It
#: scans, tries the SSID compiled into the image and gives up; until it returns
#: it holds the wlan management lock, and a ``wifi join`` issued meanwhile would
#: sit in the shell thread until the lock is released (up to the driver's 30 s
#: connect timeout), which looks like a hung console.
AUTO_CONNECT_DONE = (
    r"wifi connect success ssid:"
    r"|\[WiFi\] Connect failed!"
    r"|not find ap!"
    r"|\[WiFi\] Device not ready"
)

#: The state a later test needs from an earlier one. pytest runs the functions
#: in file order and the DUT console cursor only moves forward, so the order is
#: part of the test: joining before the AP exists would fail for the wrong
#: reason and then hide the real one.
STATE: dict = {}


def _require(key: str):
    value = STATE.get(key)
    if value is None:
        pytest.skip(
            f"needs STATE[{key!r}] from the earlier step; it did not run, so "
            "there is nothing to assert against"
        )
    return value


def _poll(dut, command: str, pattern: str, *, attempts: int, timeout: float) -> str:
    """Run @p command until its output matches @p pattern.

    ``expect`` only matches text that arrives *after* the cursor, so retrying is
    a matter of sending the command again rather than re-reading the buffer: a
    DHCP lease may land seconds after the association, and ``ifconfig`` printed
    before it does not update itself.
    """
    last: AssertionError | None = None
    for _ in range(attempts):
        dut.send_line(command)
        try:
            return dut.expect(pattern, timeout=timeout)
        except AssertionError as exc:
            last = exc
    assert last is not None
    raise last


def test_peer_raises_the_ap(wifi_peer):
    """Make the peer an AP before the DUT is asked to join anything."""
    # Drop whatever the previous suite left behind, so "already" cannot be
    # reported for an AP configured with different credentials.
    wifi_peer.ap.reset()
    ap = wifi_peer.ap.start(SSID, KEY, mac=PEER_MAC)
    assert ap.active, "the peer reported an AP that is not running"
    assert ap.mac, "the peer reported no BSSID; a two-sided check needs it"
    # A peer that ignored `mac=` would advertise the DUT's own address, which is
    # exactly the state in which the DUT cannot see the SSID at all.
    assert ap.mac.lower() == PEER_MAC.lower(), (
        f"the peer runs its AP on {ap.mac}, not on {PEER_MAC}: the DUT shares "
        "that address and will never see its own BSSID in a scan"
    )
    STATE["ap"] = ap


def test_dut_listens_while_the_ap_is_fresh(dut):
    """Scan once, seconds after ``ap.start``, and remember what came back.

    This is a probe, not a gate: it records whether the peer was heard while the
    AP had just been created, so that a later miss can be told apart from an AP
    that was never audible at all. It asserts nothing because the scan collides
    with the example's own boot-time scan for the shared scan buffer, which can
    corrupt the table on either side; the fact it records is therefore only
    trustworthy when it is *positive*.
    """
    _require("ap")
    dut.expect(PROMPT, timeout=60)
    dut.send_line("wifi scan")
    try:
        dut.expect(re.escape(SSID), timeout=45)
        STATE["heard_early"] = True
    except AssertionError:
        STATE["heard_early"] = False
    dut.expect(PROMPT, timeout=30)


def test_dut_finishes_its_own_startup(dut):
    """Wait for the example's auto-connect thread to let go of the lock."""
    dut.expect(AUTO_CONNECT_DONE, timeout=90)


def test_dut_sees_the_peer_on_the_air(dut):
    """A scan issued now lists the peer's SSID.

    ``wifi join`` connects through the *cached* scan table, not by scanning
    again, so a join issued while that table predates the AP fails with
    ``F:rt_wlan_connect ... not find ap!`` even though the AP is up. The
    example's own boot-time scan runs before the peer starts its AP, which is
    exactly the trap this step removes.
    """
    _require("ap")
    # Several scans, because one sweep is not evidence: a scan that collides with
    # the example's own boot-time scan loses entries on either side, and a single
    # miss then reads as "the AP is not there". Retrying separates "intermittent"
    # from "absent" instead of failing on the first unlucky sweep.
    attempts = 3
    for attempt in range(1, attempts + 1):
        dut.send_line("wifi scan")
        try:
            dut.expect(re.escape(SSID), timeout=40)
        except AssertionError as exc:
            if attempt == attempts:
                heard_early = STATE.get("heard_early")
                if heard_early is True:
                    diagnosis = (
                        "the early probe did hear it, so the AP stopped being "
                        "audible afterwards"
                    )
                elif heard_early is False:
                    diagnosis = (
                        "the early probe did not hear it either, so the peer's "
                        "AP is not audible from the DUT at all"
                    )
                else:
                    diagnosis = "the early probe did not run"
                raise AssertionError(
                    f"{SSID!r} was not listed by {attempts} scans issued after "
                    f"ap.start: {diagnosis}"
                ) from exc
            continue
        STATE["seen"] = True
        STATE["seen_on_attempt"] = attempt
        # ``wifi scan`` is blocking; the prompt proves the table is complete and
        # the management lock is free again.
        dut.expect(PROMPT, timeout=15)
        return


def test_dut_joins_the_peer(dut):
    """The DUT associates with the peer's AP."""
    _require("seen")
    # An empty WIFI_PEER_KEY means the peer was started as an open network, and
    # then ``wifi join <ssid>`` is the form the shell documents; appending an
    # empty argument would only look like a password of zero length.
    dut.send_line(f"wifi join {SSID} {KEY}" if KEY else f"wifi join {SSID}")
    dut.expect(r"wifi connect success ssid:" + re.escape(SSID), timeout=60)
    # ``wifi join`` is a blocking command; the prompt proves it returned and the
    # management lock is free again.
    dut.expect(PROMPT, timeout=30)
    STATE["joined"] = True


def test_dut_reports_the_peers_bssid(dut):
    """The DUT is on *this* AP, not on another network with the same name."""
    ap = _require("ap")
    _require("joined")
    dut.send_line("wifi status")
    dut.expect(r"Wi-Fi STA Info", timeout=30)
    dut.expect(r"SSID\s*:\s*" + re.escape(SSID), timeout=15)
    dut.expect(r"(?i)MAC Addr:\s*" + re.escape(ap.mac), timeout=15)


def test_dut_gets_a_lease_from_the_peer(dut):
    """The link carries IPv4: the peer's DHCP server hands the DUT an address."""
    _require("joined")
    if PEER_AP_DOWNLINK_BROKEN:
        pytest.xfail(PEER_AP_DOWNLINK_REASON)
    _poll(
        dut,
        "ifconfig",
        r"ip address:\s*192\.168\.169\.",
        attempts=10,
        timeout=5,
    )
    STATE["lease"] = True


def test_dut_can_ping_the_peer(dut):
    """ICMP crosses the link: the DUT reaches the peer's own AP address.

    The lease proves the DUT *receives*; ping proves the exchange is two-way and
    that the address it received is usable. ``192.168.169.1`` is the peer's AP
    address -- ``dhcp_server.c`` starts the server on that address by default and
    the peer's ``ifconfig`` shows it on ``w1``.
    """
    _require("joined")
    _require("lease")
    if PEER_AP_DOWNLINK_BROKEN:
        pytest.xfail(PEER_AP_DOWNLINK_REASON)
    # `ping <host> [netdev name] [times] [data size]` (netdev.c:1417-1437): the
    # count is the third positional, the second one is an interface name.
    #
    # The first echo request of a fresh association is dropped while ARP resolves
    # the gateway, so a bare four-packet run reports "3 received, 25% packet
    # loss" on a link that is perfectly healthy. A single-packet ping pays that
    # cost first, and the run below can then demand a clean 4/4 -- which keeps the
    # assertion strict instead of relaxing it to "some packets arrived".
    dut.send_line(f"ping {PEER_AP_ADDRESS} w0 1")
    dut.expect(r"ping statistics", timeout=30)
    dut.expect(PROMPT, timeout=15)

    dut.send_line(f"ping {PEER_AP_ADDRESS} w0 4")
    dut.expect(r"\d+ bytes from " + re.escape(PEER_AP_ADDRESS) + r" icmp_seq=", timeout=30)
    dut.expect(r"4 packets transmitted, 4 received, 0% packet loss", timeout=30)
    dut.expect(PROMPT, timeout=15)


def test_stopping_the_ap_drops_the_dut(dut, wifi_peer):
    """The association is this AP's: take it down and the DUT sees the link go.

    A failure here means the DUT never noticed the link loss within the timeout,
    not that the assertions above were wrong -- but it is still a real defect for
    a station driver, so it is an assertion rather than a warning.

    A DUT that never joined would report ``wifi disconnected!`` from the start,
    which would make this pass without proving anything: hence the requirement.

    Two layers can report the loss and this build only reports the lower one, so
    either is accepted (both mean the DUT dropped the link):

    * the RT-Thread wlan framework, which flips ``wifi status`` to
      ``wifi disconnected!`` -- it only does so on RT_WLAN_DEV_EVT_DISCONNECT;
    * the AIC firmware/host layer, which logs ``deauth_handler: reason code`` and
      tears the station down (keys unpaired, vif inactive).

    Known driver gap, tracked in example/test/wifi_peer/README.md: a passive
    deauth reaches the firmware (``rwnx_rx_sm_disconnect_ind: fw disconnect
    reason=1``) but never raises STA_DISCONNECT_EVENT, so the framework keeps
    claiming the link is up (no ``event: disconnect (async)`` in device.log).
    Once that is fixed the first alternative takes over and this assertion needs
    no change.
    """
    _require("ap")
    _require("joined")
    if PEER_AP_DOWNLINK_BROKEN:
        pytest.xfail(PEER_AP_DOWNLINK_REASON)
    wifi_peer.ap.stop()
    _poll(
        dut,
        "wifi status",
        r"wifi disconnected!|deauth_handler: reason code",
        attempts=12,
        timeout=10,
    )
