# USB host-side observation for automated tests

Status: design accepted 2026-11-14. Implementation follows this file; do not
fork the contract in `tools/sdk_test/DESIGN.md` or the guide.

## Decisions taken with the user

- Device examples connect their USB device port to the lab PC that runs the
  Lab Agent. Host-side evidence must be collected on that PC.
- The runner and the agent may live on different machines. Every host-side
  check therefore goes through the agent's HTTP API, never through a local
  `subprocess` in the pytest suite.
- Example `proj.conf` files are **not** changed in this round. If a board
  cannot enumerate because `BSP_USING_USBD` / `BSP_USING_USBH` is missing, the
  case records the environment defect; it does not paper over it.
- The lab binding may gain a `fixture: usb_host` connection (the file lives
  outside this repository).
- The global "skip is not a pass" verdict change is a TODO in
  `tools/sdk_test/DESIGN.md` ("TODO：pytest 的 skip 不是 PASS"), not in this
  round. This round only makes the new `usb_host` fixture fail rather than
  skip.
- Stale README expected-output text is fixed in a separate round.

## Scope

In scope: 7 examples under `example/cherryusb/` that have a README.

- device: `cdc_acm`, `cdc_acm_uart`, `cdc_hid_msc`, `hid_mouse`
- host: `cdc_acm_host`, `hid`, `msc_host`

Excluded by decision (2026-11-14): no test cases for `device/mtp`,
`device/audio_v1_mic`, `device/audio_v1_mic_speaker` (UAC) or
`host/uvc_uac_host` (UVC/UAC). The agent's audio/HID/MTP queries stay in the
framework for later use, but this round does not assert them.

Out of scope: `host/usb_bt_bridge_l2`, `host/usb_bt_bridge_nat`,
`device/msc/*` (each has its own README but is not part of this round),
WPD/MTP file browsing, audio capture/playback, HID report injection, UVC/UAC
stream startup, Linux/macOS agent backends, and any write to a USB volume.

## Architecture

```
pytest suite (runner machine)
      |  usb_host fixture -> UsbHost client
      v
Lab Agent HTTP (agent machine)
      |  /testbed/{id}/usb/*
      v
UsbHostService -> WindowsUsbHost (SetupAPI / CfgMgr32 / hid.dll / hub IOCTL)
                -> pyserial (CDC COM sessions)
```

The capability is a **testbed-level connection declared in the binding**, not
an instrument and not a fixture driver. It has no console, no line protocol
and no build. The agent owns the physical wiring; the suite names only
`usb_host`.

## Binding

```yaml
testbed:
  - id: sf32lb52-lcd_a128r16_01
    platform: sf32lb52-lcd_a128r16_hcpu
    console: {port: COM8, baud: 1000000}
    connections:
      - fixture: bt_peer:ble,classic
        console: {port: COM6, baud: 1000000}
      - fixture: usb_host            # no parameter; a parameter is an error
        usb:
          backend: auto              # auto | windows | null (null: tests only)
          exclude_ports: []          # extra ports; console/instrument/download are reserved
          allow_fixed_drives: false
          max_file_bytes: 4194304
          serial_baud: 115200
```

`usb_host` must be written **without** `:capability`; otherwise
`wiring_problems` treats the suffix as a bus check. `Connection.options`
already carries the `usb:` dictionary unchanged; the binding schema does not
need a new field.

A test case declares it under `harness_config.fixture: usb_host`. If an
instrument is also needed, use a list: `fixture: [usb_host, bt_peer:ble]`.

## Agent HTTP API

JSON throughout. Error body: `{"error": str, "code": str, "details": {...}}`.
Every route except `capabilities` requires the testbed to be allocated to the
caller **and** that allocation to have requested `usb_host`, **except the
token-addressed `/usb/serial/{token}/*` routes**: they intentionally skip the
allocation gate so that a token read after release answers `404
token_not_found` instead of a misleading `409 not_allocated`. Teardown clears
the token registry under the same `_usb_lock` that guards open, so a token
from a released testbed cannot be used.

| Method / Path | Request | Success | Main errors |
|---|---|---|---|
| `GET /testbed/{id}/usb/capabilities` | — | `{"supported": true, "platform": "win32", "features": [...], "host": {"name": "local", "machine": "..."}, "reason": null}` | 200 with `supported: false` when unsupported |
| `GET /testbed/{id}/usb/devices` | — | `{"captured_at": "...", "devices": [Device]}` | 409 `not_allocated`, 409 `fixture_not_requested` |
| `POST /testbed/{id}/usb/wait` | `{"match": {...}, "since": ["key"], "timeout": 15.0, "interval": 0.25}` | `{"device": Device, "elapsed": 1.2}` | 408 `timeout` (body carries candidates), 400 `bad_match_key` |
| `POST /testbed/{id}/usb/serial/open` | `{"device_key": "...", "com_port": "COM12", "baud": 115200, "dtr": false, "rts": false}` | `{"token": "...", "port": "COM12", "device_key": "...", "baud": ..., "dtr": ..., "rts": ...}` | 409 `port_busy`, 409 `forbidden_port`, 409 `port_not_usb`, 409 `port_denied` (the OS refused the open; retry a bounded time), 404 `device_not_found` |
| `GET /testbed/{id}/usb/serial/{token}/read?cursor=N&timeout=S` | — | `{"data": "<base64>", "cursor": N2, "closed": false}`; a closed port answers 200 with `closed: true` and whatever bytes were already read | 404 `token_not_found` |
| `POST /testbed/{id}/usb/serial/{token}/write` | `{"data_base64": "..."}` (<= 64 KiB) | `{"written": 7}` | 400, 409 `port_closed` |
| `POST /testbed/{id}/usb/serial/{token}/lines` | `{"dtr": true, "rts": false}` | `{"dtr": ..., "rts": ...}` | 404, 409 |
| `POST /testbed/{id}/usb/serial/{token}/close` | — | `{"closed": true}` | 404 `token_not_found`; the client treats a second close as a no-op |
| `GET /testbed/{id}/usb/volumes` | — | `{"volumes": [{"device_key": "...", "drive": "E:", "label": "SiFli MSC", "filesystem": "FAT", "size": 8192, "removable": true}]}`; an empty list when nothing is mounted | 409 `not_allocated`, 409 `fixture_not_requested` |
| `GET /testbed/{id}/usb/files?device_key=...&path=README.TXT&encoding=text` | `encoding` is `text` or `base64` | `{"device_key": "...", "path": "README.TXT", "size": 106, "encoding": "text", "data": "..."}` | 400 `path_denied`, 404 `file_not_found`, 413 `file_too_large`, 409 `volume_not_mounted` |
| `GET /testbed/{id}/usb/audio?device_key=...` | — | `{"device_key": "...", "inputs": [{"interface": 2, "endpoint": "0x85", "direction": "in"}], "outputs": [...]}` | 404 `device_not_found` |
| `GET /testbed/{id}/usb/hid?device_key=...` | — | `{"device_key": "...", "interfaces": [{"interface": 3, "usage_page": 1, "usage": 6, "protocol": 1, "report_length": 8, "product": "..."}]}` | 404, 503 (HID driver not bound) |

`Device`:

```json
{
  "key": "USB\\VID_38F4&PID_FFFF\\2022123456",
  "instance_id": "USB\\VID_38F4&PID_FFFF\\2022123456",
  "vid": 14580,
  "pid": 65535,
  "serial": "2022123456",
  "product": "USB Serial Device",
  "manufacturer": null,
  "device_class": "cdc",
  "classes": ["cdc"],
  "interfaces": [
    {"number": 0, "class": 2, "subclass": 2, "protocol": 1, "name": "cdc_acm"},
    {"number": 1, "class": 10, "subclass": 0, "protocol": 0, "name": "cdc_data"}
  ],
  "com_ports": [{"port": "COM12", "driver": "usbser"}],
  "volumes": [],
  "hid": [],
  "audio": null,
  "speed": "full",
  "bus_port": 1,
  "present": true
}
```

Match keys: `vid`, `pid`, `serial`, `product_contains`, `device_class`,
`class`, `interface_class`, `usage_page`, `usage`, `protocol`, `com_port`,
`volume_label`, `require_com`, `require_volume`, `speed`, `present`. Unknown
keys are a 400. `class` is a Python reserved word, so callers pass it through
the `match=` mapping (`match={"class": 10}`) or as
`**{"class": 10}`; both are merged with the keyword criteria.

Windows binds class drivers, so `SPDRP_DEVICEDESC` is the INF description, not
iProduct. Device identity is therefore **VID + PID + iSerialNumber** (the last
segment of the PnP instance id). HID can additionally read the product string
through `HidD_GetProductString`.

## Runner API (`tools/sdk_test/usb_host.py`)

Pure standard library; usable from a Linux runner against a Windows agent.

```python
class UsbHost:
    def capabilities(self) -> UsbHostCapabilities: ...
    def require_supported(self) -> None: ...
    def require(self, *features: str) -> None: ...
    def devices(self) -> tuple[UsbDevice, ...]: ...
    def wait_for(self, *, timeout: float = 30.0, interval: float = 0.25,
                 since: Iterable[str] | None = None,
                 match: Mapping[str, Any] | None = None,
                 **criteria: Any) -> UsbDevice: ...
    def volume(self, device: UsbDevice, *, timeout: float = 20.0) -> "UsbVolume": ...
    def serial(self, device: UsbDevice, *, com_port: str | None = None,
               baud: int = 115200, dtr: bool = False,
               rts: bool = False) -> "UsbSerial": ...
    def audio(self, device: UsbDevice) -> AudioEndpoints: ...
    def hid(self, device: UsbDevice) -> tuple[HidInfo, ...]: ...
    def close(self) -> None: ...

class UsbSerial:
    port: str; token: str
    def read(self, timeout: float = 1.0) -> bytes: ...
    def read_exact(self, size: int, timeout: float = 5.0) -> bytes: ...
    def write(self, data: str | bytes) -> int: ...
    def set_dtr(self, dtr: bool) -> None: ...
    def set_rts(self, rts: bool) -> None: ...
    def close(self) -> None: ...
    def __enter__(self) -> "UsbSerial": ...
    def __exit__(self, *exc: object) -> None: ...

class UsbVolume:
    drive: str; label: str | None; filesystem: str | None
    def exists(self, path: str) -> bool: ...
    def read_bytes(self, path: str) -> bytes: ...
    def read_text(self, path: str, *, encoding: str = "utf-8",
                  errors: str = "replace") -> str: ...
```

Exceptions: `UsbHostError`, `UsbHostUnavailable`, `UsbHostTimeout`
(also a `TimeoutError`), `UsbNotFound` (also a `LookupError`),
`UsbConflict`, `UsbDenied` (also a `PermissionError`). HTTP mapping:
404 without `code` -> `UsbHostUnavailable` (old agent), 501/503 ->
`UsbHostUnavailable`, 408 -> `UsbHostTimeout`, 409 -> `UsbConflict`, 404
with `device_not_found` / `token_not_found` / `file_not_found` ->
`UsbNotFound`, 409 `forbidden_port` / 400 `path_denied` -> `UsbDenied`, and
409 `port_denied` -> `UsbConflict` (the OS refused the handle; the suite
retries it for a bounded time). The mapping keys on `code` first, so the HTTP
status of `forbidden_port` (409) does not decide the exception. A transport
failure (agent unreachable) is `UsbHostUnavailable`.

The `usb_host` pytest fixture is session-scoped and depends on `dut`. It
**raises** (does not skip) when the case did not declare the fixture, when the
agent has no `usb_host` capability, or when the agent is too old. This is
deliberate: the framework records the pytest exit code as the verdict, so a
skip would be a false pass. The global fix for that is a TODO below.

## Agent implementation

New package `tools/lab_agent/usb_host/`:

| File | Content |
|---|---|
| `errors.py` | `UsbHostError(LabAgentError)` and subclasses with `code` |
| `base.py` | device/interface/COM/volume/HID/audio dataclasses; `UsbHostBackend` protocol |
| `null.py` | `UnsupportedUsbHost`: `supported=false`; other calls raise |
| `winapi.py` | ctypes bindings only (SetupAPI, CfgMgr32, hid.dll, `DeviceIoControl` constants) |
| `windows.py` | `WindowsUsbHost`: enumeration, COM mapping, volumes, HID caps, audio endpoints |
| `serial_sessions.py` | `ByteStream` (reader thread, byte buffer, monotonic cursor) and `SerialSessionRegistry` |
| `volume_paths.py` | volume root resolution, relative-path validation, read-only bounded reads |
| `service.py` | `UsbHostService`: one facade per testbed, options, cache, `close_all()` |

Windows queries (no PowerShell, no WMI, no subprocess):

- PnP: `SetupDiGetClassDevsW(DIGCF_ALLCLASSES | DIGCF_PRESENT)`, instance ids,
  `SPDRP_DEVICEDESC` / `SPDRP_COMPATIBLEIDS` / `SPDRP_ADDRESS` /
  `SPDRP_FRIENDLYNAME`; `&MI_xx` children grouped under their parent.
- Endpoints/speed: root-hub handle via `GUID_DEVINTERFACE_USB_HUB`,
  `IOCTL_USB_GET_NODE_CONNECTION_INFORMATION_EX` (PipeList; `EndpointAddress`
  bit 7 = IN, isochronous = 1) and
  `IOCTL_USB_GET_DESCRIPTOR_FROM_NODE_CONNECTION` for the configuration
  descriptor (interfaces and endpoints).
- COM: `GUID_DEVCLASS_PORTS` + friendly-name `(COM\d+)`, then `CM_Get_Parent`
  to the USB parent. The fallback reads `Enum\<instance>\Device Parameters\
  PortName`, which maps the port back to exactly that instance (more precise
  than the `HARDWARE\DEVICEMAP\SERIALCOMM` list, which has no parent link).
- Volumes: `GetLogicalDrives`, `GetDriveTypeW(DRIVE_REMOVABLE)`,
  `GetVolumeInformationW`, `GetDiskFreeSpaceExW`; device interface
  `GUID_DEVINTERFACE_VOLUME` + `CM_Get_Parent` to the USB device.
- Audio: isochronous pipes from the hub IOCTL; IN -> capture, OUT -> render;
  cross-checked with `USB\Class_01&SubClass_01/02` interfaces.
- HID: `GUID_DEVINTERFACE_HID`, `HidD_GetPreparsedData`, `HidP_GetCaps`,
  `HidD_GetAttributes`, `HidD_GetProductString`; fall back to the interface
  protocol (1 keyboard, 2 mouse) when the device cannot be opened.
- MTP: interface class 6 / subclass 1 / protocol 1, or compatible id
  `USB\MS_COMP_MTP`; no WPD file enumeration.

Serial sessions: one session per physical port; `secrets.token_urlsafe(16)`;
a background reader thread with `pyserial.Serial(timeout=0.05)`, a
`Condition`, a byte buffer and an absolute cursor; writes and line changes
under the session lock; `close_all()` from `TestBedRuntime._teardown`. A
port can only be opened if it belongs to the enumerated USB CDC child and is
not the DUT console, an instrument console, the sftool download port, or an
explicitly excluded port.

Path safety: read-only; only enumerated removable volumes (unless
`allow_fixed_drives`); relative paths only; reject empty, `..`, absolute,
drive letters, `:` (ADS), `\`, `NUL`, `*?<>|`, length > 260; after
`realpath` the path must stay under the volume root; reject reparse points;
enforce `max_file_bytes` (413 above it).

Non-Windows: `create_service` returns `UnsupportedUsbHost`; `capabilities`
answers 200 + `supported: false`; other `/usb/*` routes answer 501
`usb_host_unsupported`; `RunnerConfig.check()` warns if a binding declares
`usb_host` on a non-Windows runner.

## Board baseline (2026-11-14)

A smoke run of `device/cdc_acm` on `sf32lb52-lcd_a128r16` flashed through the
lab agent produced:

- PnP `USB\VID_38F4&PID_FFFF\2022123456`, CDC port `COM32` (`usbser`), and
  `Win32_SerialPort` reporting it;
- one 2048-byte frame from `COM32` with prefix `1234567890` and the rest `a`,
  matching the firmware;
- the DUT console printing `cherryusb device cdc acm demo!`.

The `rtconfig.h` has no `BSP_USING_USBD`, yet full-speed enumeration and bulk
data work, so the board-level USB clock branch is not a blocker. Example
configs stay unchanged, as agreed.

The first attempt to open `COM32` from the host was rejected
(`PermissionError 13`) and the same open succeeded about 12 minutes later;
the root cause was not closed (the session was not elevated, so higher
privilege handle holders could not be enumerated; `Sifli_Trace` was configured
for `COM32` but held no handles at the time). Tests must therefore retry a
transient "port enumerated but open denied" for a bounded time and then report
the holder/port, rather than treating the first denial as a firmware failure.

**Host-visible re-enumeration is a bench requirement.** A later run flashed
and reset the board while the USB cable stayed connected. The SoC reinitialised
its USB stack, but D+ never dropped, so Windows kept the old address and
configuration: PnP still listed the device and `COM32`, `CreateFile` succeeded,
but `SetCommState` failed with `WinError 31` on every attempt, including a
direct pyserial open outside the framework. `reboot`, `sftool` stub entry,
eight minutes idle, `IOCTL_USB_HUB_CYCLE_PORT` and non-elevated
`pnputil /restart-device` did not produce a disconnect. Device-side cases
therefore need a host-visible disconnect before they run. A **physical replug
of the DUT's USB data port is the verified path**; an elevated
`pnputil /restart-device` and a controllable hub/VBUS switch are candidates,
unverified -- see `tools/sdk_test/DESIGN.md`, section "TODO：烧录/复位后的主机可见重枚举".
Do not weaken the assertions or the retry to hide
this; a case that cannot open the port must fail with the port and the last
refusal in its message.

On 2026-11-15 the same experiment with a host-mode image failed to help:
flashing `host/cdc_acm_host` over the device image did not produce a
host-visible disconnect either. PnP kept the stale `VID_38F4`/`COM32` node
with the same `LastArrivalDate`, and opening the port still failed with
WinError 31. A host flash is therefore not a substitute for a replug; device
cases need a physical replug (verified); a controllable VBUS and an elevated
device restart are candidates, unverified.

The working sequence is: unplug the DUT's USB data cable, start `sdk.py test
run`, and plug the cable back during the enumeration wait. On 2026-11-14 this
turned `device/cdc_acm` from `2 failed, 2 passed` into `4 passed in 88.28s`.
For negative verification, run pytest directly against the still-running board
(allocate the testbed with `usb_host`, open the DUT console, set the
`SIFLI_TEST_*` variables, run `-p sdk_test.pytest_plugin`) instead of flashing
again; skip the boot-banner test in that mode because the console stream was
opened after boot.

## Per-example check points

Every test case lives at `<example>/testcase.yaml` with `project_dir: project`
and `pytest_root: pytest`. No `platform_allow`; `-p` supplies the platform
(the real binding spells it `sf32lb52-lcd_a128r16_hcpu`).

| Example | Test id | Fixture | Console evidence | Host evidence |
|---|---|---|---|---|
| device/cdc_acm | `cdc_acm.usb_device` | `usb_host` | `cherryusb device cdc acm demo!`; `actual in len:2048`; `actual out len:7` | serial `2022123456`; COM; 2048-byte frame `1234567890` + `a`*2038; DTR required |
| device/cdc_acm_uart | `cdc_acm_uart.usb_bridge` | `usb_host` | `[I/main] USB-UART Bridge starting...`; `UART initialized successfully`; `USB-UART bridge initialized`; `USB-UART Bridge ready`; `USB Configured`; `Set baud rate: 115200` | serial `202412345678`; COM opens at 115200 |
| device/cdc_hid_msc | `cdc_hid_msc.composite` | `usb_host` | 10 boot lines (including the greeting); `USB configured - device ready!` (the MUSB port emits no `USB connected` event); `CDC actual out/in len:7` | serial `2025010101`; classes msc+cdc+hid; volume `SiFli MSC`; `README.TXT` exact bytes; CDC echo; HID boot interface (class 3 / subclass 1 / protocol 1) + usage 1/6 |
| device/hid_mouse | `hid_mouse.usb_device` | `usb_host` | 3 banner lines; `KEY2 button initialized, pin=<board-specific>`; `mouse thread created successfully!`; `USB HID Mouse configured!` | serial `2025101418`; HID usage 1/2; host-inferred protocol 2 |
| host/cdc_acm_host | `cdc_acm_host.host_stack` | — | `cherryusb host demo!`; `ttyACM0 not found!`; `lsusb -V`; `lsusb -t` | — |
| host/hid | `hid.host_stack` | — | `cherryusb host demo!`; `hid host test`; root hub; no `mount a ...` | — |
| host/msc_host | `msc_host.host_stack` | — | `cherryusb host demo!`; `Scanning for USB mass storage devices...`; root hub; no mount | — |

Exact strings come from the source, not from the READMEs: several READMEs are
stale (`host/hid` hub/hid lines and `mount a ...`, `msc_host` "Waiting for USB
mass storage device connection...", `cdc_hid_msc` "internal Flash" although the
volume is RAM-based). `device/cdc_acm`'s README is not stale but incomplete: it
shows `cherryusb device cdc acm demo!` and omits the `actual in len` /
`actual out len` lines the firmware prints; `CDC actual out len` belongs to
`cdc_hid_msc`, whose README documents it. README fixes are a separate round.

## Negative verification

Framework (unit, no hardware): null backend -> fixture setup fails, not
passes; undeclared fixture -> fail; serial open on the console port -> 409
`forbidden_port`; second open -> 409 `port_busy`; `path=../x` / `C:\...` /
reparse point -> 400 `path_denied`; release then read token -> 404.

Per example, at least one red assertion each, then restore and re-run green:

| Example | Sabotage | Expected red |
|---|---|---|
| cdc_acm | wrong serial in the test / wrong frame prefix / DTR left false | `wait_for` timeout / frame assertion / `read_exact` timeout |
| cdc_acm_uart | baud 9600 instead of 115200 (test or firmware `LOG_I`) | `Set baud rate: 115200` |
| cdc_hid_msc | wrong README first line / wrong echo / remove HID interface | volume content / echo / HID usage |
| hid_mouse | expect usage 1/6 | the usage assertion in `test_enumerates_as_a_mouse` |
| hid_mouse | break `USB HID Mouse configured!` | `test_reports_the_configured_event` |
| cdc_acm_host | expect `found` instead of `not found` | the `not found` assertion |
| hid | wrong `hid host test` | the boot assertion |
| msc_host | wrong `Scanning for...` text | the scanning assertion |

## Commits

1. `feat(test/lab_agent): serve a usb_host capability over the testbed API`
2. `feat(test/sdk_test): expose usb_host to pytest suites`
3. `docs(test): document the usb_host capability`
4. `feat(test/cherryusb): cover device/cdc_acm` (then one commit per example,
   through `host/msc_host`: 7 example commits in all)
5. `docs(cherryusb): correct stale expected output` (separate round)

Framework and examples are committed separately; each example commit carries
its own negative-verification record in the `testcase.yaml` comment.

## TODO (not in this round)

- **Global strict verdict**: not in this round. The issue and the planned fix
  live in the autotest design document, `tools/sdk_test/DESIGN.md`, section
  "TODO：pytest 的 skip 不是 PASS"; the guide's section 6 points there. The
  `usb_host` fixture does not wait for that change: it raises on an
  undeclared fixture or an unsupported agent.
- **Automatic host-visible re-enumeration after flash/reset**: not in this
  round. The constraint and the candidate fixes (elevated device restart,
  switchable hub/VBUS, a `--reuse` mode) are a TODO in
  `tools/sdk_test/DESIGN.md`, section "TODO：烧录/复位后的主机可见重枚举".
- MTP file browsing through WPD (assert `cherryusb.txt` content).
- Linux/macOS agent backend (udev / IOKit).
- UVC/UAC stream start and audio capture/playback.
- HID report injection (to verify the mouse/keyboard actions without a human).
- Stale README expected-output text.
- `host/uvc_uac_host/src/SConscript` references a non-existent
  `cherryusb_hostuvcuac/` directory (dead `Glob`).

## Implementation status (snapshot 2026-11-15)

The commit hashes are deliberately not recorded here: a rebase changes them,
and the content below is the status. Framework, guide text and all seven
examples are committed. The lab binding is
a local file outside the repository (`E:\code2\lab\sifli-lab.yaml`, backup
`sifli-lab.yaml.bak-2026-11-14`). Pre-existing, unrelated changes to leave
alone: `middleware/bluetooth` (modified) and
`example/multimedia/lvgl/lvgl_v9_external/` (untracked).

### Done

- Research: exact console strings and host identities for all 11 examples
  (the case set is now 7; the UAC/UVC/MTP research is kept for later);
  the stale README lines are listed above and must not be asserted.
- Build matrix: all 11 examples build for `sf32lb52-lcd_a128r16`, no compile
  warnings, two RWX linker warnings each. No `BSP_USING_USBD/USBH` in any
  `rtconfig.h`.
- Board smoke: `device/cdc_acm` flashed through the agent; Windows enumerated
  `USB\VID_38F4&PID_FFFF\2022123456` and `COM32` (`usbser`); a 2048-byte
  frame `1234567890` + `a`*2038 was read from `COM32`; the DUT printed
  `cherryusb device cdc acm demo!`. The first `COM32` open was denied
  transiently and succeeded later; tests need a bounded retry (see the board
  baseline section).
- Framework, agent side (committed): `tools/lab_agent/usb_host/`
  (9 files), `/testbed/{id}/usb/*` routes in `server.py`, gate/lifecycle in
  `testbed.py`, `usb:` option validation in `config.py`, 77 tests in
  `test_usb_host.py`. Last run: 283 tests OK (skipped=1).
- Framework, runner side (committed): `tools/sdk_test/usb_host.py`,
  `suite_helpers/cherryusb.py`, the `usb_*` client methods,
  `LabAgentError.code`, the `usb_host` pytest fixture (raises, never skips),
  the reserved name, 53 tests in `test_usb_host_client.py`. Focused suite OK
  after the review fixes.
- Documentation (committed): the `usb_host` capability and its binding in both languages,
  plus the skip-as-pass trap and the commit review order.
- Lab binding outside the repo: `E:\code2\lab\sifli-lab.yaml` now has a bare
  `fixture: usb_host` connection with `usb:` options; `--check` reports
  `Configuration OK.` Backup: `sifli-lab.yaml.bak-2026-11-14`.
- Docs: this file; `tools/sdk_test/DESIGN.md` TODO for the global skip
  verdict; the guide's known-trap note.
- Test asset `device/cdc_acm` (committed): approved. The
  official run passed 4/4 on the pre-review revision, after the cooperating
  replug. The final revision was re-checked directly against the running board
  and gave 3 passed with the boot-banner test deselected, because the console
  was opened after boot; four sabotages went red.
- Host cases (committed): official
  runs passed the case 1/1 each (72.6/65.6/79.3 s) and the final direct runs
  were 4/4 each. Four, four and five sabotages went red on the board,
  including real console lines injected into the 2 s / 15 s windows and into
  the retained boot transcript.
- Device cases (committed): official
  runs passed 4/4, 6/6 and 3/3 (132.05 / 38.85 / 26.02 s) and the final direct
  runs were 2, 4 and 1 passed with the boot/configured tests deselected. Two,
  four and three sabotages went red. The composite run exposed and fixed the
  volume attribution and HID usage gaps.

### Review state

- **Runner side: approved.** First review found one functional defect
  (`read_exact` over-returned and dropped bytes) and test gaps; all six
  findings are closed. The same reviewer then approved the three low-risk
  follow-ups (non-ASCII base64, `_pending` vs `_closed`, `transport` code
  docstring). Evidence: `test_usb_host_client.py` 53 tests OK, four behaviour
  classes plus two edge fixes have sabotage records, and the full `sdk_test`
  suite ran 687 tests OK after the final fixes.
- **Agent side: approved.** Two fix rounds. The first found a high-severity
  release/open race, two wrong `SPDRP_*` constants, hub grouping that adopted
  downstream devices, a gate that did not require the binding to declare
  `usb_host`, and missing sabotage records; all are closed. The second found
  that the new race regression test blocked inside the registry lock and
  therefore could not fail without the fix; it now blocks in the backend
  enumeration and fails when either half of the lock is removed. Evidence:
  `lab_agent` 283 tests OK (skipped=1) and the five sabotage red runs.
- **Example `device/cdc_acm`: approved.** One comment round: add
  `dut.drain()` before the console assertions, retry only `UsbConflict` /
  `UsbNotFound`, drop the redundant `com_ports` assertion, and tighten the
  docstrings. The official run passed 4/4 on the pre-review revision; the
  final revision was re-checked on the running board (four sabotages red,
  3 passed with the banner test deselected). `port_denied` is mapped to
  `UsbConflict` because the OS refusal is transient and retryable.
- **Host group: approved.** One comment round (negative-verification records,
  shell readiness, the redundant `Driver=hcd` assertion). Board runs passed
  the case 1/1 each and the direct runs 4/4; the sabotages above all went red
  on the board.
- Both full suites are green at the final state: `lab_agent` 283 tests OK,
  `sdk_test` 687 tests OK.

### Remaining

None: all seven cases are committed. The device cases were verified after the
operator returned to the bench, each with one replug during the enumeration
wait.

### Resume here

Nothing pending. The optional follow-up is to expose the HID usage source
(`caps` / `report` / `compatible`) in `HidInfo`, so a case can assert that a
fallback did not silently produce the expected value; the report-descriptor
read through the hub IOCTL is best-effort and did not work on this Windows
stack.

### Commands

```powershell
. .\export.ps1
python -m unittest discover -s tools/sdk_test/tests -p "test_*.py"
python -m unittest discover -s tools/lab_agent/tests -p "test_*.py"
python tools\lab_agent\runner.py -c E:\code2\lab\sifli-lab.yaml   # serve
python tools\sdk.py test run -T example/cherryusb/device/cdc_acm -p sf32lb52-lcd_a128r16_hcpu
```

From Git Bash, clear `MSYSTEM` before calling `sdk.py`, or it prints
"MSys/Mingw is no longer supported" and runs nothing.
