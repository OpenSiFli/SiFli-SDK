# LVGL v8 Multslider Example

Source path: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_multslider`

## Overview

This example demonstrates the `lvsf_multslider` widget. What sets it apart from the plain `lv_slider`/`lv_bar` is that **it draws a readout on its circular knob**: by default it formats the current value as a number on the knob, and `lv_multslider_set_txt()` replaces that with a fixed short string. The control is draggable — the knob and readout follow as you drag.

Notes:
- `LV_PART_MAIN` is the track, `LV_PART_INDICATOR` is the fill, `LV_PART_KNOB` is the knob — all three need `bg_color`+`bg_opa` to be drawn. **In particular, if the knob is not styled it is not drawn, and neither is the readout on it.**
- The knob readout color comes from `LV_PART_MAIN`'s `text_color`.
- The knob is a circle roughly as wide as the bar height, so keep custom text short.

## Usage (key API)

```c
lv_obj_t *ms = lv_multslider_create(parent);

/* Style the three parts: MAIN=track, INDICATOR=fill, KNOB=knob (required, or
 * neither the knob nor the readout on it is drawn). */
lv_obj_set_style_bg_color(ms, lv_color_hex(0xD0D4DA), LV_PART_MAIN);       /* track */
lv_obj_set_style_bg_opa(ms, LV_OPA_COVER, LV_PART_MAIN);
lv_obj_set_style_bg_color(ms, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR); /* fill */
lv_obj_set_style_bg_opa(ms, LV_OPA_COVER, LV_PART_INDICATOR);
lv_obj_set_style_bg_color(ms, lv_color_white(), LV_PART_KNOB);             /* knob */
lv_obj_set_style_bg_opa(ms, LV_OPA_COVER, LV_PART_KNOB);
lv_obj_set_style_text_color(ms, lv_color_hex(0x303030), LV_PART_MAIN);     /* knob readout color */

lv_multslider_set_range(ms, 0, 100);
lv_multslider_set_value(ms, 60, LV_ANIM_OFF);  /* knob shows the live value, follows drag */
/* lv_multslider_set_txt(ms, "Vol");           // optional: fixed label, does not track value */
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

On startup the screen shows two multsliders:
- First (blue fill): default mode, the knob shows the **live current value** `60`; drag it with the mouse/finger and the number follows.
- Second (green fill): `lv_multslider_set_txt()` shows a **fixed label** `Vol` on the knob. Dragging still moves the knob, but the text stays put — `set_txt` sets a static string that does not track the current value.

## Troubleshooting

For technical questions, please open an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## References
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
