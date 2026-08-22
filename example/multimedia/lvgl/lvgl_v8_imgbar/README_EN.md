# LVGL v8 Imgbar Example

Source path: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_imgbar`

## Overview

This example demonstrates the `lvsf_imgbar` widget: it clips a foreground image to a value, like a **progress bar whose fill is an image**. Create a foreground `lv_img`, hand it to the imgbar (`lv_imgbar_set_img_fg`, which sizes the imgbar to the image), pick a fill direction, set the value range, then drive the value. This example puts a grey track behind the foreground for the unfilled part, and a timer sweeps the value over 0–100.

Notes:
- `lv_imgbar_set_dir()` sets the fill direction (`BAR_DIR_LEFT_TO_RIGTH` / `RIGTH_TO_LEFT` / `TOP_TO_BOTTOM` / `BOTTOM_TO_TOP`).
- The value range is set with `lv_obj_set_range_value()`; `lv_imgbar_set_value()` clips the foreground within that range.
- `set_img_fg` wraps the foreground in an internal container (the fg's new parent). That container has default padding and is scrollable, which can push the foreground out of view; get it with `lv_obj_get_parent(fg)`, clear its padding/border, disable scrolling, and pin the foreground to `(0,0)` so the fill renders flush.

## Usage (key API)

```c
lv_obj_t *imgbar = lv_imgbar_create(parent);
lv_obj_clear_flag(imgbar, LV_OBJ_FLAG_SCROLLABLE);

lv_obj_t *fg = lv_img_create(imgbar);
lv_img_set_src(fg, &fg_img_dsc);             /* foreground image */
lv_obj_refr_size(fg);
lv_imgbar_set_img_fg(imgbar, fg);            /* sizes the imgbar to the fg image */

/* handle the internal container: clear padding/border, disable scroll, pin fg */
lv_obj_t *fg_box = lv_obj_get_parent(fg);
lv_obj_set_style_pad_all(fg_box, 0, 0);
lv_obj_set_style_border_width(fg_box, 0, 0);
lv_obj_clear_flag(fg_box, LV_OBJ_FLAG_SCROLLABLE);
lv_obj_set_pos(fg, 0, 0);

lv_imgbar_set_dir(imgbar, BAR_DIR_LEFT_TO_RIGTH);
lv_obj_set_range_value(imgbar, 0, 100);
lv_imgbar_set_value(imgbar, 60);             /* value -> clip width: 60 shows 60% of the fg */
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

On startup a grey track sits in the center with a blue fill bar on it. A timer sweeps the value over 0–100, so the blue fill **repeatedly grows left to right until full, then shrinks back**.

## Troubleshooting

For technical questions, please open an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## References
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
