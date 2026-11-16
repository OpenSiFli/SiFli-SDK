# wifi_peer: the Wi-Fi instrument firmware

`wifi_peer` is the Wi-Fi counterpart of [`bt_peer`](../bt_peer/README.md): a
board that is not an application under test but the *other end* of a link. It
is an access point the DUT joins, driven over the same text protocol `bt_peer`
uses, so one host layer (`sdk_test.instruments.console`) serves both.

That is what makes a Wi-Fi test self-contained. Without it a suite depends on
whatever access point happens to be in radio range, and on credentials that
cannot be committed: the case can only prove "the DUT was pointed at an SSID",
not "the DUT associated with the AP we control".

## What it can and cannot prove

It can prove, from both sides at once:

- the DUT associated with **this** AP, not with another network of the same
  name — the peer reports its own MAC in `ap.status`, and the DUT prints the
  BSSID it joined as `MAC Addr:` in `wifi status`;
- the DUT completed DHCP against **this** AP — the framework starts a DHCP
  server on the AP netif when the AP comes up, so a DUT that really associated
  gets an address in `192.168.169.0/24` that no other network would hand it;
- the link is this AP's — stopping the AP makes the DUT's `wifi status` print
  `wifi disconnected!`.

It cannot prove that the DUT's traffic is correctly encrypted or routed: it
serves no address beyond the DHCP server, and it never inspects frames.

The station list (`rt_wlan_ap_get_sta_num()`) is reported as `sta_num` and is
**information only**: the wlan framework fills it from
`RT_WLAN_DEV_EVT_AP_ASSOCIATED` indications, and the AIC8800 driver does not
forward association events to the wlan device layer, so it stays 0 even with a
station attached. A suite must not assert on it.

### The AP downlink only worked once both ends had distinct addresses

Until 2026-11-12 the two data-plane cases (`test_dut_gets_a_lease_from_the_peer`,
`test_stopping_the_ap_drops_the_dut`) could not pass. The symptom looked like a
broken transmit path on the peer:

- frames the **firmware** sends reached the DUT: beacons, the auth/assoc replies
  that make the association succeed;
- frames the **host** sends appeared to vanish: the DHCP OFFER/ACK from
  `dhcpd_start`, and the deauth `ap.stop` sends. The peer logged them as
  `[AIC][FHOST][FW] tx dhcp` and `[AIC][FHOST]tx_sched DHCP dn staid=33`, and
  the DUT, awake and associated, received nothing;
- an open AP (`WIFI_PEER_KEY=open`) failed the same way, so it was not the key
  path either.

The real cause was the duplicate MAC described below, one layer deeper than the
BSSID: **the peer AP was still addressed by the burned-in address internally**,
and the DUT carried that same address. Every unicast frame the AP sent to the
DUT therefore had the AP's own hardware address as destination and was dropped
as "addressed to myself". Renaming only the AP was not enough; once the *station*
takes an address of its own (`WIFI_LOCAL_MAC` in
`example/rt_device/wifi/sta/src/main.c`, `88:00:33:77:69:23` on this bench),
DHCP completes, ping works and `ap.stop` disconnects the DUT.

`PEER_AP_DOWNLINK_BROKEN` in
`example/rt_device/wifi/sta/pytest_peer/test_wifi_peer.py` is therefore `False`:
the two cases assert again instead of being marked xfail. Keep the switch while
testing on a bench whose two boards still share an address.

### The BSSID has to differ from the DUT's address

Every AIC8800 module in this lab reports the same burned-in MAC — on the two
boards this instrument was built on, `88:00:33:77:69:22`, the DUT included. An
AP that keeps that address advertises a BSSID equal to the station's own, and
the station then neither lists the SSID in a scan nor associates with it: the
suite fails at "the DUT cannot see the peer", which looks like a broken peer
and is really a duplicate address.

`ap.start` therefore gives the AP a locally administered address,
`02:00:5e:00:00:01` by default, or whatever `mac=` says. A bench with several
peer boards passes a different address to each. The suite
(`example/rt_device/wifi/sta/pytest_peer`) asserts the peer really runs on the
address it was asked for, so this failure mode reports itself instead of
masquerading as one.

### The AP interface is bound at boot

`peer_ap_init()` calls `rt_wlan_set_mode(RT_WLAN_DEVICE_AP_NAME, RT_WLAN_AP)`
before `main()` reaches the shell. Two reasons, both learned the hard way:

- The framework's per-device mutex (`wlan->lock`) is created by
  `rt_wlan_dev_init()`, which only `rt_wlan_set_mode()` reaches. Calling
  `rt_wlan_dev_get_mac()` / `rt_wlan_dev_set_mac()` on an unbound interface
  asserts in `rt_mutex_take()` — the peer dies inside `ap.start`.
- The driver's own auto-init binds `wlan0` as a station, so a SoftAP started
  from that state goes through `wifi_switch_mode()` into `WIFI_MODE_APSTA`.
  A concurrent AP with no station link was not heard by the DUT at all.
  Binding AP first keeps the chip in plain AP mode.

### A passive deauth is not reported to the wlan framework

`ap.stop` deauthenticates the stations, and the DUT does drop the link: the
firmware logs `deauth_handler: reason code =1`, unregisters the station and
unpairs the keys (`pairwise:0, sta_idx:255`, `[key]suite-del:ff`). What never
happens is the step above it — the driver's async bridge
(`aic8800_wlan_event_bridge()` in
`customer/peripherals/wifi/aic8800d40-n/platform/wlan/aic8800_wlan_dev.c`) is
only entered for `STA_DISCONNECT_EVENT` / `LEAVE_RESULT_EVENT`, and this path
raises neither: a 5042-line capture of the run contains no
`event: disconnect (async)` line. So `rt_wlan_event_dispatch()` never sees
`RT_WLAN_DEV_EVT_DISCONNECT`, `_sta_mgnt.state` keeps its CONNECT bit, and
`wifi status` goes on printing `Wi-Fi STA Info` with a stale RSSI.

The suite therefore accepts either report of the loss — `wifi disconnected!`
(the framework) or `deauth_handler: reason code` (the firmware) — and the
comment on `test_stopping_the_ap_drops_the_dut` records why. When the event
bridge is fixed, the framework's own message appears and the assertion keeps
working unchanged.

## Protocol

One line in, at most one terminal line out, plus the events the operation
causes. The exact grammar is documented in
[`src/peer.h`](src/peer.h); in short:

```
peer info                            -> @info proto=1 fw=wifi_peer-1 caps=ap iface=wlan1 max_line=200
peer req <id> <op> [key=value ...]   -> @ok <id> [key=value ...]
                                     |  @err <id> <code> <text>
                                     -> @evt <seq> <name> [key=value ...]   (before the terminal line)
```

`@ok` always carries `first_seq`: the event sequence before the operation could
emit anything. The host uses it to tell this operation's events from ones still
in flight from an earlier one.

## Serving the AP by hand

Like the shipped example under `example/rt_device/wifi/ap`, this image brings up
a SoftAP by itself and runs a TCP echo server, so the board is usable without any
host driving it:

```
wifi_peer: wifi_peer-1 ready, proto 1
wifi_peer: auto start softap "SiFli_AP", echo port 8888
wifi_peer: softap "SiFli_AP" is up, echo port 8888
[AP Echo] server listening on port 8888
```

The echo service is started whether or not the boot-time AP came up: its thread
retries `bind()` every two seconds until an AP interface has an address, and every
`ap.start` (protocol op or shell command) re-checks that the thread exists. A
board whose AP only comes up when a suite raises it therefore still serves port
8888 for the rest of the run.

The auto-started AP uses `PEER_AP_SSID` / `PEER_AP_PASSWORD` (`SiFli_AP` /
`12345678`) and the default address `02:00:5e:00:00:01`. Two shell commands cover
the same ground without a host:

| command | what it does |
|---|---|
| `peer req 1 ap.status` | reports `active`, `ssid`, `ch`, `mac`, `sta_num` |
| `ap_start [ssid] [key]` | restarts the AP with that name and key (defaults `SiFli_AP` / `12345678`) |
| `ap_stop` | stops the AP |

A case that wants a different identity still uses `peer req ... ap.start`; both
go through the same code, so what you see by hand is what the suite sees.

## Operations

| op | args | `@ok` | events |
|---|---|---|---|
| `state.reset` | — | `reset=1 active=0 count=0…1` | `ap_stop mac=` when an AP was up |
| `ap.start` | `ssid_hex=` / `ssid=` (required), `key_hex=` / `key=` (empty means open), `ch=` (1..13, default 6), `hidden=` (0/1), `mac=` (default `02:00:5e:00:00:01`) | `active=1 already=0…1 late= ssid_hex= ssid= ch= mac= count=` | `ap_start ssid_hex= ssid= ch= mac=` |
| `ap.stop` | — | `active=0 already=0…1 count=` | `ap_stop mac=` |
| `ap.status` | — | `active= ssid_hex= ssid= ch= mac= sta_num= mode= count=0` | — |

A string argument that may contain a space travels hex-encoded, two hex digits
per byte: `ssid_hex=4c6162204150` is `Lab AP`. The request line is split on
whitespace and the firmware does not unquote values, so `ssid=Lab AP` would reach
the peer as two arguments. Replies always carry `ssid_hex=`, and add the readable
`ssid=` only when the SSID is a single token; the host decodes the hex form and
treats a reply whose two forms disagree as a protocol error.

`ap.start` with the parameters of an AP already running answers
`already=1 count=0` rather than restarting it, which would drop the DUT
mid-suite. "The same parameters" means the whole configuration, not just the
name: a different key, a different hidden flag or a different channel restarts
the AP, because a suite that asks for a key must get a network protected by that
key.

`state.reset` is a real reset: it stops a running AP, forgets the previous
suite's SSID and key, and reports `count=1` with an `ap_stop` event. Stopping an
AP the peer is still associated with can take the framework's full ten-second
stop budget, so the host gives the op a 60-second timeout.

## Building and wiring it into the lab

```
sdk.py test run -T example/test/wifi_peer -p sf32lb57-lcd_n16r16_dpi   # build_only: compiles, flashes nothing
```

Flash `build_<board>/main.bin` once, then bind the peer's console to the board
that offers it. The binding lives in the Lab Agent's runner config, next to the
DUT:

```yaml
testbed:
  # the board that offers the AP
  - id: sf32lb57-lcd_n16r16_dpi_02
    platform: sf32lb57-lcd_n16r16_dpi
    console: {port: COM22, baud: 1000000}
    connections:
      - fixture: wifi_peer:ap
        console: {port: COM22, baud: 1000000}
```

A case that needs the peer declares only what it needs, never a port:

```yaml
    harness: pytest
    harness_config:
      fixture: wifi_peer:ap
      pytest_root: pytest_peer
```

## What a suite looks like

`example/rt_device/wifi/sta/pytest_peer/test_wifi_peer.py` is the reference
suite. In outline:

```python
def test_peer_raises_the_ap(wifi_peer):
    STATE["ap"] = wifi_peer.ap.start("sifli_test", "12345678", channel=6)

def test_dut_joins_it(dut):
    dut.send_line("wifi join sifli_test 12345678")
    dut.expect(r"wifi connect success ssid:sifli_test", timeout=60)

def test_dut_reports_this_ap(dut):
    dut.send_line("wifi status")
    dut.expect(r"MAC Addr: " + re.escape(STATE["ap"].mac), timeout=30)

def test_dut_leases_from_it(dut):
    dut.send_line("ifconfig")
    dut.expect(r"inet addr:192\.168\.169\.", timeout=30)

def test_stopping_the_ap_drops_the_dut(dut, wifi_peer):
    wifi_peer.ap.stop()
    dut.send_line("wifi status")
    dut.expect(r"wifi disconnected!", timeout=30)
```

The DUT example has its own auto-connect thread (`wifi_auto` in
`example/rt_device/wifi/sta/src/main.c`) which scans and tries the SSID compiled
into the image. A peer suite therefore waits for that thread to finish before
joining anything: it holds the wlan management lock while it connects, and a
`wifi status` issued in the middle would block on it.

## Negative verification

Break the feature and confirm the case goes red, otherwise a green run proves
only that the board booted:

- wrong key in `ap.start` (or in the DUT's `wifi join`) — the join must fail;
- `ap.start` with a different SSID than the DUT joins — the join must fail;
- delete the DHCP server (`CONFIG_LWIP_USING_DHCPD`) — the lease assertion must
  go red while the association assertions stay green;
- return `PEER_ERR_UNSUPPORTED` from `peer_ap_start` — the suite must fail at
  the first test, not pass vacuously.
