# BT.656 Example (dpi-hdk-lb57e)

Source Path: example/rt_device/bt656

This example demonstrates the generic BT.656 interface (`bt656.h`) of
`customer/peripherals/camera/bt656`: snapshot, frame and stream all use the same
continuous capture path, and the picture is shown on the 1024×600 DPI panel.

## Usage

The subsections below list only the necessary steps. For the full procedure to
set up SiFli-SDK, build and flash a project, see
[SiFli-SDK Getting Started](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html).

### Build and Run

Run this in a shell where the SDK environment is loaded:

```bash
cd example/rt_device/bt656/project
scons --board=dpi-hdk_lb57eybbn6_n16_qspi -j8
```

After flashing, the console is `uart1` at 1000000 baud (`CONFIG_CONSOLE_UART1=y`,
`CONFIG_RT_SERIAL_DEFAULT_BAUDRATE=1000000`).

## Supported Boards

- `dpi-hdk_lb57eybbn6_n16_qspi` (board code name dpi-hdk-lb57e, SF32LB57x HCPU)

### Hardware Required

- An ARK7128 video decoder module wired to the fixed I2C1 (SCL `PA23` / SDA
  `PA22`) and the DCMI PINMAP0 pins of this board; the pin table is in
  [`customer/peripherals/camera/bt656/README.md`](../../../customer/peripherals/camera/bt656/README.md);
- An analog video source: CVBS (NTSC/PAL) or AHD 720P (25/30 fps), connected to
  VIN2 of the module;
- A 1024×600 DPI panel (on-board).

### Software Required

- The SDK environment loaded (`export.ps1` / `export.sh`), with `scons` and
  `sdk.py` available;
- Optional: LVGL v8 (`CONFIG_PKG_USING_LITTLEVGL2RTT=y`) and its application
  configuration; required only by the `camera_lvgl` preview, see
  "LVGL Integration Example".

### Configure the Project

| Item | Location |
|---|---|
| Video input format (NTSC / PAL / AHD 25 / AHD 30) | `menuconfig` → `Board Peripherals` → `Camera` → `BT.656 Camera` → `Decoder chip` → `ARK7128` → `Video input format` |
| LVGL preview (optional) | Top-level option `BT.656 + LVGL preview demo` (requires `CONFIG_PKG_USING_LITTLEVGL2RTT=y`) |
| LCD model / backlight | `project/proj.conf` |
| I2C / DCMI pins | Configured in the board `bsp_pinmux.c` (`BSP_USING_BT656` block) |

```bash
cd example/rt_device/bt656/project
sdk.py menuconfig --board=dpi-hdk_lb57eybbn6_n16_qspi
```

## Example Output

`main()` prints the banner and the usage text; capture does not start
automatically (only these two parts are excerpted below; other components print
their own boot logs before them):

```
===== BT.656 camera demo (dpi-hdk-lb57e) =====
usage:
  camera snapshot [loops] [timeout_ms]  capture & show N frames (default 1, 1000 ms)
  camera frame [interval_ms]            continuous frame mode (default 0 ms)
  camera stream [interval_ms]           continuous stream mode (default 0 ms)
  camera init                           initialize decoder
  camera stop                           stop continuous display
  camera cpu                            print CPU usage (last ~2 s window)
  camera info                           print camera status
msh >
```

After `camera init`, confirm the decoder (`camera info` prints the chip ID it
reads). Only lines starting with `[camera]` are excerpted. The driver also
prints `LOG_I/LOG_W` lines such as `[I/bt656]` and `[I/ark7128.*]`: with the
default configuration `RT_DEBUG=y` (`rtos/rtthread/src/Kconfig`) they are
emitted through rtdbg; if `RT_DEBUG` is disabled in menuconfig they are compiled
out and only the `[camera]` `rt_kprintf` output remains. With
`CONFIG_RT_USING_ULOG=y` they go through ulog instead.

```
msh /> camera init
[camera] ready: CVBS NTSC, signal locked
```

### Commands and Interaction

The camera features are provided by the `camera` command; the LVGL preview has
its own `camera_lvgl` command:

| Command | Description |
|---|---|
| `camera snapshot [loops] [timeout_ms]` | Start capture, grab N frames, then stop; defaults to 1 frame / 1000 ms |
| `camera frame [interval_ms]` | Continuous preview (same capture path as stream) |
| `camera stream [interval_ms]` | Continuous preview (same capture path as frame) |
| `camera init` | Initialize the capture layer and print the format and lock state |
| `camera stop` | Stop continuous display |
| `camera cpu` | Print CPU usage (last ~2 s window) |
| `camera info` | Print format, lock state, preview state and CPU information |
| `camera help` | Print usage (also printed when no sub-command is given) |
| `camera_lvgl start` / `stop` | LVGL image-widget preview (enable `BT.656 + LVGL preview demo` in menuconfig first) |

`interval_ms` is the minimum interval between two frames; `0` (the default)
means full speed. `frame` and `stream` are currently equivalent: both use the
same continuous capture path, and only the mode name printed by `camera info`
differs.

Examples:

```
msh /> camera snapshot             # capture one frame
msh /> camera snapshot 10          # start capture, grab 10 frames
msh /> camera frame                # continuous preview, about 27~30 fps
msh /> camera stream 100           # continuous preview, at least 100 ms between frames
msh /> camera info
[camera] format : CVBS NTSC (720x240, stride 1440, interlaced field)
[camera] signal : locked (chip id 0x7128)
[camera] stats  : captured=120, dropped=0, errors=0
[camera] preview: running, mode=frame, captured=120, failed=0
[camera] cpu    : 45.2% (last ~2 s window)
msh /> camera stop
```

While running, one statistics line is printed every 2 seconds (the fps is the
cumulative average since the preview started):

```
[camera] 29.1 fps, 34 ms/frame, fail=0, cpu 45.2%
```

## Example Breakdown (optional)

### Boot Behavior

1. `bt656_init()`: calls `init()` of the ARK7128 chip adapter (configures I2C1,
   reads the ID, loads the register table of the current format), then
   initializes DMA, links the DCMI handle and enables the DCMI interrupt
   (`HAL_DCMI_Init()` runs inside `bt656_start()`);
2. Display does not start automatically. Run `camera stream` / `camera frame` /
   `camera snapshot` (they call `bt656_start()`, which re-locks the decoder,
   initializes DCMI and starts the stream in one go).

> The driver was split into two layers in 2026-11: applications only include
> `bt656.h` and use `bt656_start/get/release/stop`; the concrete decoder chip is
> registered through `bt656_decoder_ops`, so changing the chip does not require
> application changes. See
> [`customer/peripherals/camera/bt656/docs/接口改造说明.md`](../../../customer/peripherals/camera/bt656/docs/接口改造说明.md)
> for the before/after comparison.

### Actual Behavior of the Three Commands

| | snapshot | frame | stream |
|---|---|---|---|
| Trigger | `camera snapshot` | `camera frame` | `camera stream` |
| DCMI | one `bt656_start()` | same | same |
| Completion criterion | CVBS: DMA byte count; AHD: DCMI FRAME | same | same |
| End | `bt656_stop()` after N frames | `camera stop` | `camera stop` |
| Use case | still capture / debugging | continuous preview | continuous preview |

All three commands use the same capture path: `bt656_get()` borrows a frame →
render → `bt656_release()` returns it. Buffers are allocated statically by the
driver; when display is slower than capture, new frames are dropped (counted in
`dropped`), memory does not grow, so the preview needs a single thread only.

> Continuous capture has a known fixed horizontal offset issue (first stream
> after reset, camera power cycle, occasional cases after repeated start/stop);
> the driver does not compensate for it yet.

### Display Path

```
DCMI capture (YUV422/UYVY)
  └─ preview thread: bt656_get() borrows a frame
       └─ YUV→RGB565 (EPIC hardware / CPU fallback)
            └─ show_fb (1024×600, video centered 1:1)
                 └─ only the cache lines of the video rows are written back
                      └─ the panel continuously scans show_fb via LCDC/PTC
       └─ bt656_release() returns the frame
```

- YUV→RGB565 uses EPIC hardware by default (`drv_epic_blend`) and falls back to
  the CPU on failure;
- This panel is RAMLESS DPI + line-buffer bypass: `draw_rect` can only cover the
  whole screen, and the PTC state machine keeps scanning the first buffer it was
  given. Therefore the scan start is pointed at `show_fb` once, and afterwards
  only the canvas is updated and its cache lines are written back;
- `RTGRAPHIC_CTRL_SET_BUF_FORMAT(RGB565)` must be called first, otherwise the
  layer format assertion fires.

### LVGL Integration Example (optional)

`src/bt656_lvgl.c` shows how to embed the camera picture in an LVGL UI as an
image widget:

```c
/* lv_timer callback, executed in the LVGL thread */
if (bt656_get(&f, 0) == RT_EOK) {           /* non-blocking */
    dsc.header.cf = LV_IMG_CF_YUV422_PACKED_UYVY;
    dsc.data      = f.data;
    dsc.data_size = (uint32_t)f.stride * f.height;
    lv_img_set_src(cam_img, &dsc);
    if (prev_held_valid) {                   /* return the previous frame */
        bt656_release(&prev_held);
    }
    prev_held = f;
    prev_held_valid = RT_TRUE;
}
```

LVGL v8's YUV422 image format is handed to EPIC by the SDK LVGL GPU driver, so
no CPU color conversion and no `show_fb` are needed. (The example relies on the
LVGL v8 `lv_img_dsc_t` / `lv_timer_create` APIs; v9 does not apply.)

- Call `lv_img_cache_invalidate_src(&dsc)` for every frame: the buffer address
  does not change but its content does, and without invalidation the LVGL image
  cache serves the stale picture;
- Do not call `bt656_release()` right after `lv_img_set_src()`: EPIC is an
  asynchronous DMA engine, so the previous frame is returned on the next update.

Enable it as follows (LVGL needs a complete LVGL application configuration;
turning on the package switch alone is not enough):

```conf
# project/proj.conf -- use example/lvgl_v8_ttf/project/proj.conf as a reference
CONFIG_RT_USING_MEMHEAP=y
CONFIG_PKG_USING_LITTLEVGL2RTT=y
# CONFIG_LV_USING_EXT_RESOURCE_MANAGER is not set
# The SDK LVSF default theme refers to these font sizes (at 1024x600)
CONFIG_LV_FONT_MONTSERRAT_16=y
CONFIG_LV_FONT_MONTSERRAT_20=y
CONFIG_LV_FONT_MONTSERRAT_24=y
CONFIG_LV_FONT_MONTSERRAT_28=y
CONFIG_LV_FONT_MONTSERRAT_36=y
CONFIG_LV_FONT_DEFAULT_MONTSERRAT_16=y
```

> Measured: with only `CONFIG_PKG_USING_LITTLEVGL2RTT=y`, the SDK LVGL
> middleware first fails because `RT_USING_MEMHEAP` is off and `lang_pack.h` is
> missing; after fixing those it fails because `lv_font_montserrat_*` is not
> declared. The combination above covers all three problems.

Then enable the example's own option. It comes from `project/Kconfig.proj` and
depends on `CONFIG_PKG_USING_LITTLEVGL2RTT=y`, so it appears only after LVGL is
enabled:

```bash
cd example/rt_device/bt656/project
sdk.py menuconfig --board=dpi-hdk_lb57eybbn6_n16_qspi
# top-level option: BT.656 + LVGL preview demo
```

```bash
msh /> camera_lvgl start    # full-screen centered display
msh /> camera_lvgl stop
```

Limitations:

- CVBS only (packed UYVY, `stride == 2*width`); AHD buffers still contain
  timing codes and `lv_img_dsc_t` has no stride field, so AHD is not supported;
- This example creates its own LVGL host thread; if your project already has an
  LVGL thread, move the timer part there;
- Do not run it together with `camera stream` (both claim the `lcd` device).

### CPU Usage Measurement

This follows `example/system/cpu_usage_profiler` and uses the SDK CPU Usage
Profiler, which is enabled by default in `project/proj.conf`:

```conf
CONFIG_USING_CPU_USAGE_PROFILER=y
CONFIG_CPU_USAGE_METRICS_ENABLED=y
CONFIG_CPU_USAGE_METRICS_PRINT_DIRECTLY=y
CONFIG_CPU_USAGE_METRICS_PRINT_PERIOD=10
```

- `camera cpu`: prints CPU usage of the last ~2 s;
- `camera info`: appends CPU usage;
- The periodic line of `camera frame` / `camera stream` includes `cpu xx.x%`;
- With ULOG enabled, the system prints the per-thread load (`CPU Usage
  Metrics`) every 10 s. **Note**: that table and the SDK `cpu` command print
  through `LOG_I` and need `CONFIG_RT_USING_ULOG=y`; the current `proj.conf`
  does not enable ULOG, so neither is printed in this project, while
  `camera cpu` / `camera info` / the statistics line use `rt_kprintf` and are
  unaffected. Example output with ULOG enabled (values are illustrative):

```
============================
CPU Usage:   36.20/  50.01 (  72.39%)
========================================================
thread      run_time(s)    load(%)
--------------------------------------------------------
[ark_prev]:   24.10   48.19%
[tidle   ]:   13.81   27.61%
[tshell  ]:    0.05    0.10%
```

- The SDK also provides shell commands: `cpu` (print statistics now),
  `cpu_prof_reset` (clear), `cpu_prof_deinit` / `cpu_prof_init` (detach/attach
  the profiling hook); the output of `cpu` depends on ULOG (same reason as
  above);
- Metrics uses `thread->user_data` to record thread runtime, so it requires
  `RT_USING_PTHREADS` to be disabled (satisfied by the current project);
- `cpu_get_usage()` returns the usage of the last ~2 s statistics window, not
  an instantaneous sample;
- To stop measuring, remove the four configuration lines above and rebuild;
  the code automatically takes the "profiler disabled" printing branch.

### Memory (PSRAM)

| Purpose | CVBS NTSC | CVBS PAL | AHD 720P |
|---|---|---|---|
| `ark7128_stream_buf` (3 buffers / 2 for AHD) | 3 × 414,720 B | 3 × 414,720 B | 2 × 1,843,200 B |
| `show_fb` (display canvas) | 1,228,800 B | same | same |

Each CVBS stream buffer is sized for the largest CVBS format (PAL 1440×288):
one NTSC field carries only 345,600 B of valid data, but the buffer is still
414,720 B. CVBS stream buffers plus the canvas total about 2.4 MB, all in
PSRAM1. AHD stream buffers use the `frambuf` section and are placed in PSRAM2
(the `l2_non_ret_bss_frambuf*` rule in the linker script; the availability of
the second PSRAM on this board still needs to be confirmed).

## Troubleshooting

> The two `[W/bt656]` lines below are driver `LOG_W` messages: visible with the
> default `RT_DEBUG=y`, compiled out when `RT_DEBUG` is disabled. Without logs,
> use `signal` and `stats` from `camera info` (whether `errors` grows) and the
> preview thread's `[camera] N captures failed (ret=...)`: `ret=-2`
> (`-RT_ETIMEOUT`) means no frame can be captured, `ret=-8` (`-RT_EIO`) means
> capture has stopped.

- `camera info` prints `format not configured`: `bt656_init()` has not succeeded
  yet (run `camera init` first; on failure it prints `decoder init failed`);
  `chip id 0x0000` means the chip cannot be read over I2C, so check the module
  power and the I2C1 (`PA22`/`PA23`) wiring.
- `signal` is `no lock`: make sure the analog source is connected to VIN2 and
  its standard matches `Video input format`.
- On capture timeout the driver behaves according to the signal state:
  - signal present but no frames: prints
    `[W/bt656] no frame in N ms (remaining x/y), restart capture` and resets
    DCMI to restart the stream;
  - no signal: prints `[W/bt656] no frame in N ms, no signal, waiting for
    source` (at most about once per second), does not reset DCMI, and waits
    for the source to come back.

  After 50 consecutive failures the preview thread prints
  `[camera] 50 captures failed (ret=-2), check signal` (`-2` is
  `-RT_ETIMEOUT`), then repeats every 50 failures without exiting.

### Known Limitations

- There is no double-buffer flip between the panel self-scan and the CPU-written
  buffer, so fast motion may tear;
- Insufficient PCLK signal integrity causes occasional code loss (wrong colors,
  missing codes); this is a hardware issue.

## References

- Driver and board wiring: [`customer/peripherals/camera/bt656/README.md`](../../../customer/peripherals/camera/bt656/README.md)
- Interface evolution and buffer ownership: [`customer/peripherals/camera/bt656/docs/接口改造说明.md`](../../../customer/peripherals/camera/bt656/docs/接口改造说明.md)
- CPU usage: [`example/system/cpu_usage_profiler`](../system/cpu_usage_profiler/README.md)
