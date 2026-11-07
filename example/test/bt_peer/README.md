# bt_peer — BLE test instrument

`bt_peer` is the fixed firmware a test suite drives as the *other* end of a
Bluetooth link. It runs no application of its own: it powers on the BLE stack,
registers one MSH command (`peer`) and performs what the host asks over a
single-line text protocol. The design is in
[`tools/sdk_test/BT_PEER_DESIGN.md`](../../../tools/sdk_test/BT_PEER_DESIGN.md).

Host side: `tools/sdk_test/instruments/bt_peer.py` (`BtPeer` / `BleInstrument`).
The lab wiring is `fixture: bt_peer:ble` in the testcase and a `connections:`
entry in the lab binding.

## Build

```bash
source ./export.sh            # ./export.ps1 on Windows
cd example/test/bt_peer/project
scons --board=<board> -j8
```

The artifacts land in `build_<board>/`, as for any example. The project is
`build_only` in `testcase.yaml`: CI compiles it, nothing executes it.

The firmware needs `CONFIG_RT_CONSOLEBUF_SIZE=1024` and
`CONFIG_FINSH_CMD_SIZE=1024` (already in `project/proj.conf`): one protocol
line carries a 244-byte hex payload, which the default 80-byte shell line
would truncate.

## Flash once

Flash the image once with the usual downloader and leave the board in the
lab. It never needs re-flashing between test runs; `state.reset` returns it to
a known state. Point the binding at its console:

```yaml
connections:
  - fixture: bt_peer:ble
    console: {port: COM22, baud: 1000000}
```

Then run any suite that declares `fixture: bt_peer:ble`, for example:

```bash
sdk.py test run -T example/ble/central_and_peripheral --tag peer -p <board>
```

## Checking that the instrument answers

Type this in its console (or let the fixture do it):

```
peer info
```

Expected line, verbatim apart from `max_conn` (always 8 on this build):

```
@info proto=1 fw=bt_peer-m2 caps=ble addr=<uid-derived> name=bt_peer-<xxx> roles=central,peripheral max_conn=8 ble=5.2 max_line=1024
```

`proto=1` and `caps=ble` are what `expect_ready()` gates on; a board running
another image (the old `diss` one, for instance) fails here rather than later.

`addr` and `name` are UID-derived (`bt_mac_addr_generate_via_uid_v2`), so every
instrument in a lab has its own public address and its own `bt_peer-<suffix>`
name. Without that override the stack's weak default would give every board the
same address, `AB:05:03:02:05:08`.

`PEER_CAPS` in `src/peer.h` is the single source of that capability string. A
build that adds the Classic stack overrides it with `"ble,classic"`; the host
declares `fixture: bt_peer:ble,classic` and gets both `bt_peer.ble` and
`bt_peer.classic` over the same console stream. No protocol or Agent change.

## Protocol

Host to instrument:

```
peer info
peer req <id> <op> [key=value ...]
peer data <id> <hex>              # not implemented in M1
```

Instrument to host:

```
@info proto=1 fw=bt_peer-m2 caps=ble addr=<uid-derived> name=bt_peer-<xxx> roles=central,peripheral max_conn=8 ble=5.2 max_line=1024
@ok  <id> [key=value ...]
@err <id> <code> <text>
@evt <seq> <name> [key=value ...]
```

Rules that matter when extending it:

1. Exactly one terminal line per request, correlated by `id`.
2. Binary values are always `len=N data=<hex>`; an empty payload is
   `len=0 data=`. The host waits for N bytes instead of guessing a quiet
   window.
3. Every `@` line is one locked `rt_kprintf`. Never build a line across calls.
4. Non-`@` lines (ulog, the prompt) are ignored by the host.
5. `gatt.discover` emits every `@evt char` before `@ok count=<n>`.
6. One unfinished GATT operation per link; a second one answers `@err ... busy`.
7. Scan and connection indices are 1-based on the wire; a link id is stable
   for the life of the connection.

### M1 operations

| Op | Notes |
|---|---|
| `state.reset` | Drops links, stops scan/advertising/periodic, frees GATT client registrations. |
| `gap.scan` | Blocks for `duration_ms`, then `@ok count=<n>`; one `@evt adv` per report, up to `PEER_MAX_ADV_REPORTS` (64). |
| `gap.scan.stop` | Stops a scan early (the shell is serial, so mostly a no-op). |
| `gap.connect` | By scan `index` or `addr`; `@ok link=<n>`. |
| `gap.disconnect` | By `link`. |
| `dev.list` | One `@evt link` per connection, then `@ok links=<n>`. |
| `gatt.discover` | `uuid` optional; `@evt char` lines then `@ok count=<n>`. |
| `gatt.read` | `@ok len=N data=<hex>`. |
| `gatt.write` | `response=0/1`. |
| `gatt.subscribe` | Writes the CCCD. |
| `gatt.mtu` | `@ok mtu=<n>`. |
| `adv.start` / `adv.stop` | Legacy or extended, connectable or not. |
| `adv.data` | Replace the running advertisement's data; 1–31 bytes for legacy, 1–248 for extended. An empty payload is refused (SIBLES cannot clear one). |
| `gap.conn_params` | `link`, `interval_min_ms` (8–4000), `interval_max_ms`, `latency` (0–499), `timeout_ms` (100–32000). `@ok` means the controller accepted it; the negotiated values arrive as `@evt conn_params`. |
| `gap.phy` | `link`, `phy=1/2/3` (1M/2M/Coded); waits for the remote-PHY indication. |
| `gap.data_len` | `link`, `tx_octets` (27–251), `tx_time` (0–2120). `@ok` means accepted; the negotiated values arrive as `@evt data_len`. |
| `svc.register` | Registers the 128-bit loopback service; `@ok handle=<h>`. |
| `svc.notify` | `handle=<h> len=N data=<hex>`; requires the DUT (the GATT client) to have subscribed. |
| `sec.bond` | `io=<name>` (`no_input_no_output`, `display_only`, `display_yes_no`, `keyboard_only`, `keyboard_display`) or the numeric enum; `auth=<name>` (`none`, `legacy`, `legacy_mitm`, `secure`, `secure_mitm`) or the numeric `gap_auth`; `passkey=<n>`. The `@ok` waits for pairing to finish **and** the link to encrypt, so a protected attribute is immediately writable. |
| `sec.clear` | Deletes all bonds. |
| `periodic.adv.start` / `periodic.adv.stop` | Periodic advertiser, SID 0. |
| `periodic.sync` / `periodic.sync.stop` | By scan `index` or `addr`; reports arrive as `@evt periodic`. |

Anything else answers `@err <id> unsupported <op>`. The two remaining gaps are
`peer data` (chunked continuation; the 1024-byte line covers a 460-byte payload)
and `svc.unregister` (the stack cannot remove a service from the ATT database).

### Events

```
adv          index=<n> addr=<hex> type=0/1 rssi=<dBm> len=N data=<hex>
scan.done    count=<n>
conn         link=<n> addr=<hex> role=0/1 interval_ms=<n>
disconn      link=<n> reason=<n>
link         link=<n> addr=<hex> role=0/1 interval_ms=<n>
char         link=<n> index=<n> handle=<h> value_handle=<h> descs=<h>:<uuidhex>,...
             (UUID in the order the peer sent it, LSB first: a CCCD is 0229)
notify       link=<n> handle=<h> len=N data=<hex>
write        link=<n> handle=<h> len=N data=<hex>
read         link=<n> handle=<h> len=N data=<hex>
mtu          link=<n> mtu=<n>
passkey      link=<n> value=<n>
periodic     link=<n> len=N data=<hex>   (only when the payload changes)
```

## Loopback service

`svc.register name=loopback` registers the service `bt_peer-loop` with one
characteristic `bt_peer-val`: readable, writable (request and command) and
notifiable, plus a CCCD. Whatever the DUT writes is kept and served back to a
later read, and the host can notify the DUT after it subscribes.

The instrument keeps one shared value (the last write, whichever link made it)
and one subscription flag per connection, so it can track several links at once.

## Known limitations (M1)

- `svc.unregister` answers `unsupported`: the stack cannot remove a service
  from the ATT database, so the loopback service stays registered for the
  lifetime of the image. `state.reset` only clears subscription state.
- `peer data` (chunked payload continuation) is not implemented; the
  `max_line=1024` budget covers a 460-byte payload, which is enough for a
  244-byte MTU write.
- Classic Bluetooth is out of scope for this image; the design reserves
  `bt_peer:classic` for a later one.
