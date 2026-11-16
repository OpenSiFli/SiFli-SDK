# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""The Wi-Fi half of the ``wifi_peer`` instrument.

The peer firmware (``example/test/wifi_peer``) is an access point the DUT joins;
this module is how a suite drives it. Nothing here knows which example is under
test or which credentials the lab uses: the suite passes an SSID and a key, and
every method maps to one protocol op.

``WifiPeer`` is the facade the pytest fixture yields; ``.ap`` is the access
point instrument. A sibling (``.sta``) would be added the same way if the peer
ever needs to *join* a network instead of offering one.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

from ._values import int_value as _int
from .console import InstrumentConsole
from .errors import InstrumentCapabilityError
from .errors import InstrumentProtocolError


def _encode(text: str) -> str:
    """The wire form of a string that may contain a space.

    ``ssid=Lab AP`` would reach the peer as two shell arguments and the peer
    would answer ``invalid_arg``; two hex digits per byte cannot be split, and
    ``ssid_hex=``/``key_hex=`` are the argument names that say so.
    """
    return text.encode("utf-8").hex()


def _decode(encoded: str) -> str:
    try:
        return bytes.fromhex(encoded).decode("utf-8")
    except (ValueError, UnicodeDecodeError) as exc:
        raise InstrumentProtocolError(
            f"the peer sent a malformed ssid_hex {encoded!r}: {exc}"
        ) from exc


@dataclass(frozen=True)
class ApState:
    """What the peer says about its access point.

    ``already`` is True when the request found an AP with the same parameters
    already running; the instrument then starts nothing and emits no event.
    """

    ssid: str
    channel: int
    mac: str
    active: bool
    already: bool = False
    #: Association count as the wlan framework reports it. Informational only:
    #: the AIC8800 driver does not forward association indications, so it stays
    #: 0 even with a station attached. Never assert on it.
    station_count: int = 0
    mode: str = ""

    @classmethod
    def from_values(cls, values: Dict[str, str]) -> "ApState":
        # An SSID with a space cannot travel as `ssid=`: the request line is
        # split on whitespace. The firmware therefore always answers with
        # `ssid_hex=` and only adds the readable `ssid=` when it is one token.
        ssid_hex = values.get("ssid_hex", "")
        ssid = values.get("ssid", "")
        if ssid_hex:
            decoded = _decode(ssid_hex)
            # Both fields must agree when the readable one is present; a peer
            # that disagrees with itself is broken, not merely terse.
            if ssid and ssid != decoded:
                raise InstrumentProtocolError(
                    f"the peer disagreed with itself about the SSID: "
                    f"ssid={ssid!r} ssid_hex={ssid_hex!r}"
                )
            ssid = decoded
        return cls(
            ssid=ssid,
            # An AP that was never started has no channel; the firmware reports
            # -1 rather than inventing one.
            channel=_int(values, "ch", -1),
            mac=values.get("mac", ""),
            active=_int(values, "active", 0) != 0,
            already=_int(values, "already", 0) != 0,
            station_count=_int(values, "sta_num", 0),
            mode=values.get("mode", ""),
        )


class ApInstrument:
    """The peer's SoftAP, over the line protocol."""

    def __init__(self, console: InstrumentConsole) -> None:
        self._console = console

    # -- lifecycle -----------------------------------------------------------

    def reset(self) -> None:
        """Take any AP down and forget the previous test's credentials.

        The firmware stops a running AP, so this can wait for an association to
        time out rather than returning at once; the timeout is the peer's own
        stop budget plus slack. The stop it causes is history for this suite,
        so its event is dropped afterwards.
        """
        result, _events, _start = self._console.command_events(
            "state.reset", "ap_stop", timeout=60.0
        )
        state = ApState.from_values(result)
        if state.active:
            raise InstrumentProtocolError(
                f"the peer still reports an active AP after reset: {result}"
            )
        # Whatever the reset emitted belongs to the previous test.
        self._console.clear_events()

    def start(
        self,
        ssid: str,
        key: str = "",
        *,
        channel: int = 6,
        hidden: bool = False,
        mac: str | None = None,
        timeout: float = 30.0,
    ) -> ApState:
        """Offer @p ssid (open when @p key is empty) and wait until it is up.

        Replying takes up to a few seconds on the AIC8800: the firmware resets
        the SDIO link and downloads the AP firmware before the AP starts. A
        timeout here is a real failure, not a slow host.

        @p mac pins the BSSID. Leave it unset to use the peer image's default
        (``02:00:5e:00:00:01``). It has to differ from the DUT's own address:
        the lab modules share one burned-in MAC, and a BSSID equal to the DUT's
        address is one the DUT neither scans nor associates with.

        The SSID and key travel hex-encoded: a shell argument cannot carry a
        space, and an SSID is allowed to contain one.
        """
        if not ssid:
            raise ValueError("an access point needs an SSID")
        result, _events, _start = self._console.command_events(
            "ap.start",
            "ap_start",
            timeout=timeout,
            ssid_hex=_encode(ssid),
            key_hex=_encode(key),
            ch=channel,
            hidden=int(hidden),
            mac=mac,
        )
        state = ApState.from_values(result)
        if not state.active:
            raise InstrumentProtocolError(
                f"the peer said the AP started but is not active: {result}"
            )
        if state.ssid != ssid:
            raise InstrumentProtocolError(
                f"the peer started {state.ssid!r}, not {ssid!r}: {result}"
            )
        # A caller that asked for a channel or a BSSID is asking for a specific
        # network: accepting a different one would make the suite assert
        # against the wrong peer without ever failing.
        if state.channel != channel:
            raise InstrumentProtocolError(
                f"the peer started channel {state.channel}, not {channel}: {result}"
            )
        if mac is not None and state.mac.lower() != mac.lower():
            raise InstrumentProtocolError(
                f"the peer started with BSSID {state.mac!r}, not {mac!r}: {result}"
            )
        return state

    def stop(self, *, timeout: float = 30.0) -> ApState:
        """Take the AP down. Stopping an AP that is not up is not an error."""
        result, _events, _start = self._console.command_events(
            "ap.stop", "ap_stop", timeout=timeout
        )
        state = ApState.from_values(result)
        if state.active:
            raise InstrumentProtocolError(
                f"the peer still reports an active AP after stop: {result}"
            )
        return state

    # -- queries -------------------------------------------------------------

    def status(self, *, timeout: float = 10.0) -> ApState:
        """The AP's current parameters, or ``active=False`` when it is down."""
        return ApState.from_values(self._console.command("ap.status", timeout=timeout))

    def mac(self, *, timeout: float = 10.0) -> str:
        """The BSSID a DUT must report after associating with this peer."""
        mac = self.status(timeout=timeout).mac
        if not mac:
            raise InstrumentProtocolError(
                "the peer reported no MAC; the peer image must run on the board "
                "that offers the AP, not on a board that only drives a UART"
            )
        return mac


class WifiPeer:
    """The instrument fixture's facade.

    ``capability`` is the fixture parameter (``ap`` today). Constructing the
    facade verifies the firmware actually provides it, so a mis-flashed lab
    fails at the first test instead of at the first `ap.start`.
    """

    def __init__(self, console: InstrumentConsole, *, capability: str = "") -> None:
        self._console = console
        self.capability = capability
        self.info = console.expect_ready(capability)
        self._ap = None

    @property
    def console(self) -> InstrumentConsole:
        return self._console

    @property
    def caps(self) -> set:
        """The capabilities the firmware advertises, not the ones declared."""
        return {
            entry.strip()
            for entry in self.info.get("caps", "").split(",")
            if entry.strip()
        }

    @property
    def ap(self) -> ApInstrument:
        if "ap" not in self.caps:
            raise InstrumentCapabilityError(
                "this instrument does not advertise the 'ap' capability "
                f"(caps={sorted(self.caps) or 'none'}); the lab must flash a "
                "wifi_peer image built with the SoftAP path"
            )
        if self._ap is None:
            self._ap = ApInstrument(self._console)
        return self._ap
