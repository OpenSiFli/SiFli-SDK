# LVGL v8 Multroller Example

Source path: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_multroller`

## Overview

This example demonstrates the `lvsf_multroller` widget: a simple, declarative roller picker (built on `lvsf_multlist`). Give it a single `'\n'`-separated options string and it lays out a **looping, snapping** list with a center focus area that highlights the current option; read the choice back with `lv_multroller_get_selected()`. It complements `lvsf_mulroller`: mulroller is a low-level, callback-driven, deeply customizable wheel, while multroller only needs options / visible count / focus area — a good fit for the common "here is a list of text options, pick one" case. This example is a month picker.

Notes:
- The widget defaults to full-screen and an opaque background, so set a smaller size with `lv_obj_set_size()` and make the background transparent first.
- Option text uses the widget's own font and color style, so set them with `lv_obj_set_style_text_font()` / `set_style_text_color()` **before** `set_options()`; the option inside the center focus area is shown highlighted.
- Call order: set size + font/color → `set_show_cnt()` → `set_options()` → `set_focus_param()`.
- The options string is split on `'\n'`; **every option (the last one included) must be followed by a `'\n'`** — i.e. the string ends with a blank line — otherwise the last option is dropped.
- `set_show_cnt(cnt)` sets the number of visible options (minimum 3); each item is sized by dividing the widget by the count.
- `set_focus_param(color, w, h)` marks a centered `w×h` focus area whose option is highlighted. Looping is on by default, so the list wraps from the end back to the start.
- `get_selected()` returns the index of the centered (selected) option; `set_selected(idx, anim_time)` selects programmatically (`anim_time` 0 = immediate). This example polls `get_selected()` with a small timer and mirrors it into a read-out.
- The widget is built on `lvsf_multlist` and uses the app-framework memory; this example enables `CONFIG_GUI_APP_FRAMEWORK` in `proj.conf`.

## Usage (key API)

```c
/* options string: split on '\n'; end with a blank line so the last item is kept */
static const char *OPTIONS =
    "January\nFebruary\nMarch\nApril\nMay\nJune\n"
    "July\nAugust\nSeptember\nOctober\nNovember\nDecember\n\n";

lv_obj_t *roller = lv_multroller_create(parent);
lv_obj_set_size(roller, 240, 240);
lv_obj_set_style_bg_opa(roller, LV_OPA_TRANSP, 0);               /* opaque by default; make transparent */
lv_obj_set_style_text_font(roller, &lv_font_montserrat_24, 0);   /* option font */
lv_obj_set_style_text_color(roller, lv_color_hex(0xBDBDBD), 0);  /* unselected: grey */

lv_multroller_set_show_cnt(roller, 5);                           /* 5 options visible */
lv_multroller_set_options(roller, OPTIONS);
lv_multroller_set_focus_param(roller, lv_palette_main(LV_PALETTE_RED), 240, 48); /* center focus highlight */

/* read / set the selection */
uint16_t sel = lv_multroller_get_selected(roller);      /* index of the centered option */
lv_multroller_set_selected(roller, 3, 200);             /* select option 3 with a 200ms animation */
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

On startup the screen center shows a month roller displaying 5 months at once, with the centered one in a highlight color (red) and the rest grey. Drag up/down with the mouse (simulator) or finger (board): the list loops (after December it wraps back to January) and snaps to the nearest item on release. The read-out below, `selected: <month>`, follows the current selection.

## Troubleshooting

For technical questions, please open an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## References
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
