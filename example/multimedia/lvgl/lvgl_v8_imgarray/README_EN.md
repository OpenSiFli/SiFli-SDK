# LVGL v8 Imgarray Example

Source path: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_imgarray`

## Overview

This example demonstrates the `lvsf_imgarray` widget (built on `lvsf_baseimg`): it builds a number out of images, like a 7-segment or flip display. It can render a full numeric form — **sign + integer digits + decimal point + decimal digits + unit** — each glyph being an image. Give it a glyph array (0–9 plus extras like a point and a unit), the integer/decimal digit counts, the indices of the point/unit glyphs, and a value; it lays out `[sign][int].[frac][unit]` from those images. This example shows a measurement like `38.5%` sweeping over 20.0–80.0%.

Notes:
- The value mode is set by `lv_imgarray_set_img_type()`: `BASEIMG_TYPE_ARRAY_INDEX` (integer value), `BASEIMG_TYPE_ARRAY_Q248` (Q24.8, **supports decimals**), etc. This example uses Q24.8.
- **Call order**: call `lv_imgarray_set_src_array()` first to store the glyph array, then `set_int_num()`/`set_point_idx()`/`set_unit_idx()` — these create the image cells and apply the stored array. The reverse order crashes because the array is not ready yet.
- `set_int_num()`/`set_float_num()` set the integer/decimal digit counts; `set_point_idx()` picks the decimal-point glyph, `set_unit_idx()` the unit glyph, `set_negative_idx()` the minus glyph (all by index into the glyph array); `set_leading_zero()`/`set_trailing_zero()` control leading/trailing zeros.
- `lv_imgarray_set_value()` sets the value (in Q24.8 mode pass `value * 256`).

## Usage (key API)

```c
/* glyph_arr: digits 0..9 + "." (idx 10) + "%" (idx 11), rendered with a canvas here */
lv_obj_t *ia = lv_imgarray_create(parent);
lv_imgarray_set_img_type(ia, BASEIMG_TYPE_ARRAY_Q248);  /* Q24.8: supports decimals */
lv_imgarray_set_src_array(ia, glyph_arr, 0, 11);        /* store the glyph array first */
lv_imgarray_set_int_num(ia, 2);                         /* 2 integer digits */
lv_imgarray_set_float_num(ia, 1);                       /* 1 decimal digit */
lv_imgarray_set_point_idx(ia, 10);                      /* decimal point = glyph 10 */
lv_imgarray_set_unit_idx(ia, 11);                       /* unit = glyph 11 */

lv_imgarray_set_value(ia, 365 * 256 / 10);             /* 36.5 (Q24.8): shows "36.5%" */
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

On startup a decimal value with a unit (e.g. `38.5%`) is shown in the center, built from separate images for the integer digits, decimal point, decimal digit and unit. A timer sweeps the value over 20.0–80.0%, so the reading **keeps changing**.

## Troubleshooting

For technical questions, please open an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## References
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
