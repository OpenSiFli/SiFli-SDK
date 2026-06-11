# LVGL v8 Select Example

Source path: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_select`

## Overview

This example demonstrates the `lvsf_select` choice-list widget: each row shows a "check" icon when selected and an "uncheck" icon otherwise. In single mode (`LV_SELECT_TYPE_SINGLE`) only one row is selected at a time, and tapping a row moves the selection there. Set the element count and size, the check/uncheck images and the initial state. This example is a 3-row single select with the first row selected.

Notes:
- `lv_select_set_ele_num()` sets the row count; `lv_select_set_ele_size()` sets each row's size (which lays the rows out).
- `lv_select_set_check_src()` / `lv_select_set_uncheck_src()` set the selected/unselected icons; `lv_select_set_ele_state()` sets a row's initial state.
- The select constructor clears its own `LV_OBJ_FLAG_CLICKABLE`, so re-add it for taps to reach the rows (each row's built-in click callback moves the single selection).

## Usage (key API)

```c
lv_obj_t *sel = lv_select_create(parent);
lv_obj_add_flag(sel, LV_OBJ_FLAG_CLICKABLE);   /* constructor clears it; taps need it */
lv_select_set_type(sel, LV_SELECT_TYPE_SINGLE); /* single choice */
lv_select_set_ele_num(sel, 3);                  /* row count */
lv_select_set_ele_size(sel, 220, 40);           /* row size */
lv_select_set_check_src(sel, &check_img_dsc);   /* selected icon */
lv_select_set_uncheck_src(sel, &uncheck_img_dsc);/* unselected icon */
lv_select_set_ele_state(sel, 0, LV_SELECT_STATE_CHECK);  /* row 0 selected initially */

/* read the user's choice back (a row click bubbles to the select, so attach a
 * callback on the select and read it there): */
uint16_t idx = lv_select_get_select_idx(sel);            /* single: the selected row */
/* for multi, check each row: lv_select_get_ele_state(sel, i) == LV_SELECT_STATE_CHECK */
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

On startup three rows sit in the center, labeled `Option A/B/C`, each with a small icon on the right: the first is green (selected), the other two are dark (unselected); a `Selected: Option A` label below shows the current choice. **Tap another row** and the green "check" icon moves to it while the `Selected:` label updates to the picked option — showing the app reading the choice back and reacting.

## Troubleshooting

For technical questions, please open an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## References
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
