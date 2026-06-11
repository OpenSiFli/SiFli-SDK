# LVGL v8 Timeline Example

Source path: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_timeline`

## Overview

This example demonstrates the `lvsf_timeline` animation-timeline widget. It is a **generic animation-sequencing engine**: each element maps a time window `[start_time, end_time]` to a value range `[start_value, end_value]` and feeds the interpolated value to an exec callback, which can apply it to **any property of any object** (position, size, opacity, angle, scale, color, ...). Several elements share one clock, so multiple animations are choreographed in time for complex transitions/intros — moving an object is just one use.

Notes:
- `lv_timeline_add_element()` adds one segment: given the start/end value, start/end time and an exec callback, the timeline feeds the interpolated value to the callback as the clock advances.
- The exec callback signature is `void (*)(void *var, int32_t value, void *user_data)`; it applies the value to the target object (set position, size, opacity, ...).
- The timeline object itself is invisible (size 0); it ends and self-deletes after the total time set by `set_time`, so a fresh one is created for each play.

## Usage (key API)

```c
/* Two callbacks, each applies the value to a different property (position, size) */
static void move_x(void *var, int32_t v, void *ud) { lv_obj_set_x((lv_obj_t *)ud, (lv_coord_t)v); }
static void set_sz(void *var, int32_t v, void *ud) { lv_obj_set_size((lv_obj_t *)ud, v, v); }

lv_obj_t *tl = lv_timeline_create(lv_scr_act());
lv_obj_set_size(tl, 0, 0);
/* (start_value, end_value, start_time, end_time, exec_cb, ready_cb, var); last two share a window = parallel */
lv_timeline_add_element(tl, 40, 200, 0, 600, move_x, NULL, box);    /* 0..600    move right */
lv_timeline_add_element(tl, 40, 70, 600, 1200, set_sz, NULL, box);  /* 600..1200 grow */
lv_timeline_add_element(tl, 200, 40, 1200, 1900, move_x, NULL, box);/* 1200..1900 move back  | parallel */
lv_timeline_add_element(tl, 70, 40, 1200, 1900, set_sz, NULL, box); /* 1200..1900 shrink back | */
lv_timeline_set_time(tl, 1900);   /* total duration */
lv_timeline_start(tl);            /* start playing */
```

## Supported Boards

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## Build and Download

The board project lives under `project`. Build for a specific board by passing its name:
- e.g. for sf32lb52-lcd_n16r8: run `scons --board=sf32lb52-lcd_n16r8 -j8` in `project`
- Download via `download.bat` (or `uart_download.bat`) under the build directory
- The SF32LB52x/SF32LB56x series also generate `uart_download.bat`; run it and enter the download UART port

## Simulator

Run `scons --board=pc_hcpu -j8` in `project` to produce `build_pc_hcpu/main.exe`, then run it to open the window.
- First adjust `SiFli-SDK/msvc_setup.bat` to match your local MSVC installation.

## Run Description

On startup the screen shows a red box and a `Run` button below it.
- Click `Run`: the timeline plays for about 1.9 s — the box moves right (0–0.6 s), grows (0.6–1.2 s), then **shrinks and moves back at the same time** (1.2–1.9 s, two parallel segments).
- Click again to reset and replay.

## Troubleshooting

For technical questions, please open an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## References
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
