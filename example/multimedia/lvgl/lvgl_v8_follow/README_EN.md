# LVGL v8 Follow Example

Source path: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_follow`

## Overview

This example demonstrates the `lvsf_follow` widget: a physics "gravity" icon menu. Icons arrange in concentric rings and move under gravity while avoiding each other. It is typically used as a smartwatch app launcher. On a board a g-sensor provides the gravity direction; in the PC simulator the widget has a built-in path that turns a mouse click into a gravity vector.

Notes:
- Get the config struct with `lv_follow_get_cfg_param()` and set the physics parameters (gravity, friction, collision type, ...) and the ring layout.
- **`cfg->custom_align` must be `true`** so the per-ring layout below is honored: `offset_r[i]` is ring i's radius, `target_r[i]` is the icon radius on that ring, and `gap_angle[i]` sets the icon count per ring (`360 / gap_angle`). With `false` the widget auto-sizes icons from its own width.
- `lv_follow_set_item_cb()` sets two callbacks: the item callback returns an icon object for each element, the delete callback runs when an element is destroyed.
- After adding elements call `lv_follow_on_start()` + `lv_follow_enter_order_status()` to pack the icons into the ring layout.

## Usage (key API)

```c
lv_obj_t *follow = lv_follow_create(parent);
lv_obj_set_size(follow, LV_PCT(100), 360);

lv_follow_cfg_t *cfg = lv_follow_get_cfg_param(follow);
cfg->collision_type = FOLLOW_TYPE_STANDARDS;
cfg->gravity = 0.01f;  cfg->friction = 0.2f;  cfg->icon_r = 23;  cfg->v_max = 3;
cfg->target_r[0] = 23; cfg->target_r[1] = 19; cfg->target_r[2] = 14;  /* icon radius per ring */
cfg->offset_r[0] = 0;  cfg->offset_r[1] = 70; cfg->offset_r[2] = 125; /* ring radius */
cfg->gap_angle[0] = 360; cfg->gap_angle[1] = 60; cfg->gap_angle[2] = 40; /* 360/gap = icons per ring */
cfg->custom_align = true;   /* required: use the per-ring layout above */

lv_follow_set_item_cb(follow, item_cb, delete_cb);  /* item_cb returns each icon object */
for (int i = 0; i < 8; i++)
    lv_follow_add_item_info(follow, 0, NULL);        /* add elements */
lv_follow_on_start(follow);
lv_follow_enter_order_status(follow);                /* pack into the ring layout */
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

On startup eight colored circles form a gravity menu ring (one large in the center, a ring of six around it, and one outer). Simulator interaction:
- **Click the center**: the icons re-pack into the ring layout.
- **Click off-center**: the icons are pulled that way by "gravity".
- **Long-press drag**: move an icon.

On a board the gravity is driven by a g-sensor (the app reads it and feeds it to the widget).

## Troubleshooting

For technical questions, please open an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## References
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
