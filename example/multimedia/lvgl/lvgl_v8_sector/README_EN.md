# LVGL v8 Sector Example

Source path: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_sector`

## Overview

This example demonstrates the `lvsf_sector` widget: it reveals an image through an **angular (pie-wedge) mask** — the mapped value sets how wide the visible wedge is. sector is built on `lv_img`, so set the source image with `lv_img_set_src`, set the angle span and value range, allocate the mask, then drive it with `lv_sector_set_value`. This example sweeps the value back and forth over 0–100, so the wedge grows then shrinks.

Notes:
- `lv_obj_set_range_scale(obj, 0, 360)` sets the angle span (a full circle here); `lv_obj_set_range_value(obj, 0, 100)` sets the value range; the value maps proportionally onto the angle.
- After setting the ranges you must call `lv_sector_validate()` to allocate the angular mask buffer before `lv_sector_set_value()`.
- Set the image opacity just below `LV_OPA_COVER` (`LV_OPA_COVER - 1`): this makes the base `lv_img` cover-check report NOT_COVER so the parent repaints under the masked-out region; otherwise stale pixels remain there.

## Usage (key API)

```c
lv_obj_t *sector = lv_sector_create(parent);
lv_img_set_src(sector, &img_dsc);            /* source image (sector is built on lv_img) */
lv_obj_set_size(sector, 140, 140);
lv_obj_set_style_img_opa(sector, LV_OPA_COVER - 1, LV_PART_MAIN);  /* avoid stale pixels */
lv_obj_set_range_scale(sector, 0, 360);      /* angle span */
lv_obj_set_range_value(sector, 0, 100);      /* value range */
lv_sector_validate(sector);                  /* allocate the angular mask buffer */

lv_sector_set_value(sector, 50);             /* value -> angle: 50 is a half circle (180°) */
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

On startup an orange image sits in the center, clipped to a pie wedge by the mask. A timer sweeps the value back and forth over 0–100, so the wedge angle **repeatedly grows to a full circle and shrinks back**.

## Troubleshooting

For technical questions, please open an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## References
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
