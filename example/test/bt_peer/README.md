# bt_peer — Bluetooth test instrument

`bt_peer` is the fixed firmware a test suite drives as the *other* end of a
Bluetooth link. It runs no application of its own: it powers on the radio
stack, registers one MSH command (`peer`) and performs what the host asks over
a single-line text protocol. One image carries the BLE half (GAP/GATT) and, on
58X, the Classic (BR/EDR) half (inquiry and SPP). The design is in
[`tools/sdk_test/BT_PEER_DESIGN.md`](../../../tools/sdk_test/BT_PEER_DESIGN.md).

Host side: `tools/sdk_test/instruments/bt_peer.py` (`BtPeer` /
`BleInstrument`) for BLE and `tools/sdk_test/instruments/classic.py`
(`BtClassicInstrument`) for Classic. The lab wiring is `fixture: bt_peer:ble`
or `fixture: bt_peer:ble,classic` in the testcase and a matching
`connections:` entry in the lab binding.

## Build

```bash
source ./export.sh            # ./export.ps1 on Windows
cd example/test/bt_peer/project

# BLE instrument: caps=ble
scons --board=sf32lb52-lcd_a128r16_hcpu -j8

# BLE + Classic instrument: caps=ble,classic, and btc=1 in @info
scons --board=sf32lb58-lcd_n16r32n1_qspi_hcpu -j8
```

The artifacts land in `build_<board>/`, as for any example. The project is
`build_only` in `testcase.yaml`: CI compiles it, nothing executes it.

The Classic half is selected by `project/sf32lb58x/proj.conf`, which the build
merges for 58X only (the `<chip>/proj.conf` next to `project/proj.conf`). It
sets `CONFIG_PEER_CLASSIC` — which depends on `SOC_SF32LB58X`, so a 52X build
cannot pick it up — plus `CONFIG_BT_FINSH` and the SPP profile options. The same
file also sets `LCPU_CONFIG_V2`, `HCI_DISABLE` and `LCPU_RUN_SEPERATE_IMG`,
which the 58X boot path needs.

The LCPU side takes the same path on both chips: `project/SConstruct` calls
`AddLCPU(...)`, which builds the shared controller project
`example/ble/lcpu_general` as a child project. Nothing is copied in from
`example/rom_bin`. The 58X image is about 889 KB
(`build_<board>/output/main.bin`).

The firmware needs `CONFIG_RT_CONSOLEBUF_SIZE=1024` (a long `@evt` line may
carry a 460-byte payload as 920 hex characters) and `CONFIG_FINSH_CMD_SIZE=1024`
(the default 80-byte shell line would truncate a request long before its
240-character cap). Both are already in `project/proj.conf`.

## Flash once

Flash the image once with the usual downloader and leave the board in the
lab. It never needs re-flashing between test runs; `state.reset` returns it to
a known state. Point the binding at the instrument's own console:

```yaml
connections:
  - fixture: bt_peer:ble            # 52X image
    console: {port: COM22, baud: 1000000}
  # or, for the 58X image that also carries the Classic half:
  # - fixture: bt_peer:ble,classic
  #   console: {port: COM22, baud: 1000000}
```

A 58X instrument is flashed once with the 58X build and then serves both
halves; the binding names the capability set, and the case states the same
spec (`fixture: bt_peer:ble,classic`). Capabilities are matched as a subset,
so a case that asks only for `bt_peer:ble` still runs against it.

Then run any suite that declares `fixture: bt_peer:ble`, for example:

```bash
sdk.py test run -T example/ble/central_and_peripheral --tag peer -p <board>
```

A Classic suite names the board its *DUT* needs; the instrument's build and
port come from the lab binding:

```bash
sdk.py test run -T example/bt/spp -p <board>
```

## Checking that the instrument answers

Type this in its console (or let the fixture do it):

```
peer info
```

Expected line on the 52X image, verbatim apart from `max_conn` (always 8 on
this build):

```
@info proto=1 fw=bt_peer-m6 caps=ble addr=<uid-derived> name=bt_peer-<xxx> roles=central,peripheral max_conn=8 ble=5.2 max_line=240
```

The 58X image adds the Classic capability and one marker field:

```
@info proto=1 fw=bt_peer-m6 caps=ble,classic btc=1 addr=<uid-derived> name=bt_peer-<xxx> roles=central,peripheral max_conn=8 ble=5.2 max_line=240
```

`proto=1` and `caps` are what `expect_ready()` gates on; a board running
another image (the old `diss` one, for instance) fails here rather than later.
`btc=1` sits at the same level as `caps`: it says the image includes the
Classic half (`#ifdef PEER_CLASSIC`) **and** that its init succeeded, not that
the Classic stack is up — the first `btc.*` op (`btc.local`) answers that. A
Classic build whose init failed omits it, so the SPP suite's explicit
assertion fails instead of the first `btc.*` timing out.

`addr` and `name` are UID-derived (`bt_mac_addr_generate_via_uid_v2`), so every
instrument in a lab has its own public address and its own `bt_peer-<suffix>`
name. Without that override the stack's weak default would give every board the
same address, `AB:05:03:02:05:08`.

`PEER_CAPS` is decided at build time: `project/Kconfig.proj` sets it to
`"ble,classic"` when `CONFIG_PEER_CLASSIC` is on and `"ble"` otherwise, while
`src/peer.h` keeps a C fallback for a build that does not go through Kconfig.
The host declares the same set (`fixture: bt_peer:ble,classic`) and gets both
`bt_peer.ble` and `bt_peer.classic` over the same console stream. No protocol
or Agent change.

## Protocol

Host to instrument:

```
peer info
peer req <id> <op> [key=value ...]
peer data <id> <hex>              # not implemented
```

Instrument to host:

```
@info proto=1 fw=bt_peer-m6 caps=<ble | ble,classic> [btc=1] addr=<uid-derived> name=bt_peer-<xxx> roles=central,peripheral max_conn=8 ble=5.2 max_line=240
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
8. `gap.*` / `gatt.*` and `btc.*` are separate operation namespaces, and a
   Classic `link` id is never a BLE `link` id. Never pass one where the other
   belongs; the numbers alone cannot tell you it is wrong.

### BLE operations (`gap.*`, `gatt.*`, ...)

| Op | Notes |
|---|---|
| `state.reset` | Drops links (BLE and Classic), stops scan/advertising/periodic, frees GATT client registrations. |
| `gap.scan` | Blocks for `duration_ms`, then `@ok count=<n>`; one `@evt adv` per report, up to `PEER_MAX_ADV_REPORTS` (64). |
| `gap.scan.stop` | Stops a scan early (the shell is serial, so mostly a no-op). |
| `gap.connect` | By scan `index` or `addr`; `@ok link=<n>`. |
| `gap.disconnect` | By `link`. |
| `dev.list` | One `@evt link` per connection, then `@ok links=<n> first_seq=<n>`. |
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
| `file.push` | `link=<n> handle=<h> name=<file> len=N data=<hex>`; `N` must be a multiple of 4. Runs the whole watchface download exchange (below) and answers `@ok name=<file> bytes=<N>` only when the peer reports success for every step. The peer must have discovered the `sifli_sd` service, and the host must have subscribed to the data characteristic's CCCD. |
| `periodic.adv.start` / `periodic.adv.stop` | Periodic advertiser, SID 0. |
| `periodic.sync` / `periodic.sync.stop` | By scan `index` or `addr`; reports arrive as `@evt periodic`. |

Anything else answers `@err <id> unsupported <op>`. The two remaining gaps are
`peer data` (chunked continuation; a request line is capped at 240 characters)
and `svc.unregister` (the stack cannot remove a service from the ATT database).

### Classic operations (`btc.*`)

Present only in a build with `PEER_CLASSIC` (58X). Without it the dispatcher
answers `@err <id> unsupported <op>`.

| Op | Notes |
|---|---|
| `btc.local` | Brings up the Classic stack and reports the instrument's own identity. `timeout_ms` (default 10000) is the *instrument's* budget for the stack, not the host's wait. `@ok addr=<12hex> name=<hex> role=server`; the instrument takes the pageable role, so a DUT may connect to it. |
| `btc.inquiry` | `duration_ms` (default 10000) and `max_rsp` (default 0 = no cap, 0–255; the inquiry ends early once that many devices have answered). Emits one `@evt btc.dev` per device, then `@ok count=<n> first_seq=<n>` once the inquiry has finished. It hears every device class: the firmware issues the inquiry directly because the finsh helper filters for audio devices on this branch. Refused with `@err ... busy` while an inquiry, a pair or a connect is pending. |
| `btc.pair` | `addr=<12hex>` and `timeout_ms` (default 20000, instrument budget). Binds a link key so the SPP channel can authenticate; `@ok addr=<12hex>`. Refused with `@err ... busy` while an inquiry, a connect or another pair is pending; a failed pairing answers `@err state pair`, a pairing that reports no outcome `@err timeout pair`. The bond persists across reconnects. When no event names the outcome the firmware falls back to the security manager's paired-device list and then the connection manager's bond table; both are persistent and the callback carries no request id, so an **existing bond for the same address can make a failed attempt answer `@ok`**. |
| `btc.connect` | `addr=<12hex>`, `uuid` (2, 4 or 16 bytes, display order, default `1101` = SPP) and `timeout_ms` (default 10000, instrument budget). Runs an SDP search for `uuid` first, then pages the peer; the two steps share `timeout_ms` plus the stack allowance (2 s). `@ok link=<n> addr=<12hex> channel=<n> role=0` only once the SPP link is up; the matching `@evt btc.conn` precedes it. Refused with `@err ... busy` while an inquiry, a pair or another connect is pending; an SDP refusal/failure answers `@err state connect`, a timeout in either step `@err timeout connect`. It does **not** pair implicitly — call `btc.pair` first. |
| `btc.disconnect` | `link=<n>`. `@ok` means the request was accepted, not that the link is down: `@evt btc.disc` is the confirmation. |
| `btc.spp.send` | `link=<n> len=N data=<hex>`; `@ok bytes=N`. `N` must fit the SPP link MTU, and the request line is capped at `max_line`, so the practical payload is roughly 90–100 bytes. An `N` above the link MTU is refused with `@err invalid_arg len`; an over-long line is refused by the host before it is sent. |

Classic events:

```
btc.dev    addr=<12hex> name=<hex> rssi=<dBm>
btc.conn   link=<n> addr=<12hex> channel=<n> role=0/1
btc.disc   link=<n> reason=<n>
btc.data   link=<n> len=N data=<hex> off=<n>
```

Conventions that are easy to get backwards:

- **`uuid` is in display order** — most significant byte first — the reverse of
  the BLE spelling. SPP is `1101` here; the same number would be `0111` in a
  `gatt.*` call.
- **`name` is the UTF-8 bytes of the name, hex encoded** (`btc.local`,
  `btc.dev`). The host decodes it; an invalid sequence is replaced, not fatal.
- **`role` is the instrument's side of the link**: 0 when it paged the DUT
  (client), 1 when the DUT paged it (server). An incoming SPP connection is
  accepted automatically and reported as `@evt btc.conn ... role=1`.
- **`channel` is each end's own local service channel.** The instrument's
  `channel` (from `btc.conn` / `btc.connect`) is what `btc.spp.send` uses; the
  DUT's own `chl` is what its `send_data` needs. The two numbers differ by
  design (8 and 1 on the bench) and are not expected to match.
- **Pair before connect.** SPP's RFCOMM channel requires an authenticated link,
  so `btc.connect` without a bond fails: the DUT reports HCI 0x06 (PIN or key
  missing) and the instrument answers `@err timeout connect`, not a clearer
  error. `state.reset` deletes the instrument's bonds while the DUT may keep
  the old key; the first `btc.pair` then fails with `@err state pair` (the DUT
  drops its record after KEY_MISSING) and a second attempt succeeds. The SPP
  suite retries once.
- **A Classic `link` id and a BLE `link` id are separate namespaces.** Both are
  small integers, so a mix-up is silent; keep them apart in a dual-mode suite.
- **A long SPP packet is split, not truncated.** `btc.data` carries at most
  `PEER_MAX_PAYLOAD` (460) bytes per line; `off` is the offset within the
  original packet, so a receiver concatenates by `off`. `len` is the length of
  that chunk.
- **`state.reset` drops the Classic links and deletes the bonds.** It waits at
  most one second for the stack to tear links down and suppresses events for
  the links it dropped, so a test that resets does not see the previous
  state's `btc.disc`.

### Events

```
adv          index=<n> addr=<hex> type=0/1 rssi=<dBm> len=N data=<hex>
scan.done    count=<n>
conn         link=<n> addr=<hex> role=0/1 interval_ms=<n>
disconn      link=<n> reason=<n>
link         link=<n> addr=<hex> role=0/1 interval_ms=<n>
char         link=<n> index=<n> handle=<h> value_handle=<h> uuid=<uuidhex> descs=<h>:<uuidhex>,...
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

## File push

`file.push` drives the watchface download server used by
`example/ble/file_transfer`: the instrument is the GATT client, the DUT is the
server of the `sifli_sd` serial-transport service. Every value on that service
is framed as:

```
[cate_id = 0x04][flag][length u16 LE][payload]
```

`flag` 0 is a complete packet; 1, 2, 3 are the first / continue / last packets
of a message too long for one ATT write. Inside the payload, a request is a
watchface message:

```
[message_id u16 LE][length u16 LE][data...]
```

The exchange is START_REQ/RSP (the response carries `max_data_len`), then
FILE_SEND_START_REQ/RSP (4-byte aligned length including the CRC word, name),
one FILE_SEND_DATA/RSP per chunk (`index` starts at 0, +1 per chunk), then
FILE_SEND_END_REQ/RSP and END_REQ/RSP. A non-zero `result` stops the push and
is reported as `@err file_failed` naming the step. The chunk size is the
smaller of `max_data_len`, 200 bytes and the negotiated ATT MTU minus the
4+4+4+3 bytes of framing. The payload is followed by the CRC-32 word
(polynomial `0x04C11DB7`, init `0xFFFFFFFF`, no final XOR), stored little
endian the way `file_recv_check_crc32()` rebuilds it.

The responses arrive as notifications on the same data characteristic, so the
host must subscribe before calling `file_push`; the notification hook feeds
the push engine and still emits the usual `@evt notify` lines.

## Known limitations

- `svc.unregister` answers `unsupported`: the stack cannot remove a service
  from the ATT database, so the loopback service stays registered for the
  lifetime of the image. `state.reset` only clears subscription state.
- `peer data` (chunked payload continuation) is not implemented. `max_line`
  is 240 because a request line longer than a few hundred characters reaches
  the shell damaged (a 329-character line arrived truncated, with a stray NUL);
  the request hex payload therefore stays under about 100 bytes.
  `PEER_MAX_PAYLOAD` (460) is the *output*/event bound, not a request one.
- The Classic half implements **SPP only**. A2DP, HFP, AVRCP and PAN have no
  peer: `btc.*` has no operation that would set up those profiles, and a suite
  that needs them cannot run against this instrument.
- An SPP packet longer than 460 bytes arrives as several `btc.data` lines with
  their `off` offsets. The host's `wait_data()` returns one chunk, so a caller
  that must reassemble larger payloads has to read the events itself.
- Classic `link` ids and BLE `link` ids are separate namespaces; see Protocol.
