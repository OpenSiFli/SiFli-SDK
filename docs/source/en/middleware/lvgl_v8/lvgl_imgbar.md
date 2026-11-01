# LVGL v8 Imgbar

`lvsf_imgbar` is a custom progress-bar widget that SiFli wraps around LVGL img. It crops a foreground image according to a value, acting like a **progress bar filled with an image**. Create a foreground `lv_img`, hand it to the imgbar (`lv_imgbar_set_img_fg` sizes the imgbar to the image), choose the fill direction, set the value range, and then drive the value.

- Main header: _middleware/lvgl/lvsf/gui_widgets/lvsf_imgbar.h_
- Value-range setting: _middleware/lvgl/lvsf/gui_widgets/lvsf_obj_ext.h_ (`lv_obj_set_range_value`)
- Dependent widget: LVGL img (`LV_USE_IMG`)
- Example project: _example/multimedia/lvgl/lvgl_v8_imgbar_

## Features

- Crops a foreground image by value to implement an image-filled progress bar.
- Supports four fill directions: left-to-right, right-to-left, top-to-bottom, and bottom-to-top.
- Supports two modes: bar (BAR) and switch (SWITCH).
- Supports drag-and-release to change the progress, and animation on progress changes.
- Supports three image layers: background, foreground, and indicator.
- Supports a user callback on progress changes.

## Use Cases

- Image-filled battery, volume, and brightness bars.
- Progress indicators with custom texture/gradient fills.
- Draggable sliding switches and sliders.
- Progress bars with an indicator knob.

## Supported Boards

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## Configuration and Initialization

`lvsf_imgbar` depends on the LVGL img component. In menuconfig, confirm it is enabled:

```none
CONFIG_LV_USE_IMG=y
```

In the BSP, the `LVSF_USE_IMGBAR` macro controls whether this widget is compiled.

Minimum flow to create an object:

```c
#include "lvsf_imgbar.h"

lv_obj_t *imgbar = lv_imgbar_create(lv_scr_act());
```

## API Reference

### Enumerations and callback types

| Type | Value | Description |
| --- | --- | --- |
| `lv_imgbar_dir_t` | `BAR_DIR_LEFT_TO_RIGTH` | Left to right |
| | `BAR_DIR_RIGTH_TO_LEFT` | Right to left |
| | `BAR_DIR_TOP_TO_BOTTOM` | Top to bottom |
| | `BAR_DIR_BOTTOM_TO_TOP` | Bottom to top |
| `lv_imgbar_mode_t` | `IMG_BAR_MODE_BAR` | Bar mode |
| | `IMG_BAR_MODE_SWITCH` | Switch mode |
| `lv_imgbar_process_cb_t` | `void (*)(lv_obj_t *obj, uint8_t percent)` | Progress-change processing callback; `obj`: image bar object; `percent`: current percentage |

### Creation and refresh

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_obj_t *lv_imgbar_create(lv_obj_t *parent)` | Creates an image bar object | `parent`: parent object; returns the object pointer on success, or `NULL` on failure |
| `void lv_imgbar_refresh_timer(lv_timer_t *timer)` | Refresh timer callback | `timer`: timer object; periodically refreshes the image bar display |

### Direction, mode, and interaction

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_imgbar_set_dir(lv_obj_t *img, lv_imgbar_dir_t dir)` | Sets the fill direction | `dir`: direction (`lv_imgbar_dir_t`) |
| `void lv_imgbar_set_drag(lv_obj_t *imgbar, bool en)` | Enables/disables dragging | `en`: `true` enables drag-and-release to change the effect |
| `void lv_imgbar_set_mode(lv_obj_t *img, lv_imgbar_mode_t mode)` | Sets the display mode | `mode`: bar or switch mode |
| `void lv_imgbar_set_user_cb(lv_obj_t *img, lv_imgbar_process_cb_t user_cb)` | Sets the progress-change callback | `user_cb`: user callback function pointer |

### Value settings

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_imgbar_set_value(lv_obj_t *img, int32_t value)` | Sets the progress value (Q24.8 format) | `value`: Q24.8-format value; crops the foreground according to the range set by `lv_obj_set_range_value()` |
| `void lv_imgbar_set_value2(lv_obj_t *imgbar, int32_t value)` | Sets the progress value (integer format) | `value`: integer value |

### Images and indicator

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_imgbar_set_img_fg(lv_obj_t *imgbar, lv_obj_t *img_fg)` | Sets the foreground image | `img_fg`: foreground image object; sizes the imgbar to the foreground image and wraps the foreground in an internal container |
| `void lv_imgbar_set_img_bg(lv_obj_t *imgbar, lv_obj_t *img_bg)` | Sets the background image | `img_bg`: background image object |
| `void lv_imgbar_set_img_indicator(lv_obj_t *imgbar, lv_obj_t *img_indicator)` | Sets the indicator image | `img_indicator`: indicator image object |
| `lv_obj_t *lv_imgbar_get_img_bg(lv_obj_t *imgbar)` | Gets the background image | Returns the background image object pointer |
| `lv_obj_t *lv_imgbar_get_img_indicator(lv_obj_t *imgbar)` | Gets the indicator image | Returns the indicator image object pointer |
| `void lv_imgbar_set_indicator_offset(lv_obj_t *img, lv_coord_t x, lv_coord_t y)` | Sets the indicator offset | `x`/`y`: X/Y offset |
| `void lv_imgbar_set_indicator_offset_x(lv_obj_t *imgbar, lv_coord_t x)` | Sets the indicator X offset | `x`: X offset |
| `void lv_imgbar_set_indicator_offset_y(lv_obj_t *imgbar, lv_coord_t y)` | Sets the indicator Y offset | `y`: Y offset |

## Typical Usage

```c
#include "lvsf_imgbar.h"
#include "lvsf_obj_ext.h"   /* lv_obj_set_range_value */

lv_obj_t *imgbar = lv_imgbar_create(parent);
lv_obj_clear_flag(imgbar, LV_OBJ_FLAG_SCROLLABLE);

lv_obj_t *fg = lv_img_create(imgbar);
lv_img_set_src(fg, &fg_img_dsc);          /* Foreground image */
lv_obj_refr_size(fg);
lv_imgbar_set_img_fg(imgbar, fg);         /* Size the imgbar to the foreground image */

/* Handle the internal container: clear padding/border, disable scrolling, pin the foreground to the top-left */
lv_obj_t *fg_box = lv_obj_get_parent(fg);
lv_obj_set_style_pad_all(fg_box, 0, 0);
lv_obj_set_style_border_width(fg_box, 0, 0);
lv_obj_clear_flag(fg_box, LV_OBJ_FLAG_SCROLLABLE);
lv_obj_set_pos(fg, 0, 0);

lv_imgbar_set_dir(imgbar, BAR_DIR_LEFT_TO_RIGTH);
lv_obj_set_range_value(imgbar, 0, 100);
lv_imgbar_set_value(imgbar, 60);          /* Value -> crop width: 60 reveals 60% of the foreground */
```

```{warning}
`lv_imgbar_set_img_fg()` wraps the foreground in an internal container (the new parent of the foreground). By default this container has padding and is scrollable, which pushes the foreground out of the visible area. Obtain it with `lv_obj_get_parent(fg)`, clear its padding/border, disable scrolling, and pin the foreground to `(0,0)`; only then does the fill sit flush against the edge.
```

## Demo

Run the `lvgl_v8_imgbar` example to see the result: a gray track is shown in the center of the screen with a blue fill bar on top. A timer drives the value back and forth between 0~100, so the blue fill repeatedly grows from left to right until it covers the bar, then shrinks back.

## See Also

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [LVGL v8 Image documentation](https://docs.lvgl.io/8.3/widgets/img.html)
- Source path: `middleware/lvgl/lvsf/gui_widgets/lvsf_imgbar.h`
- Example path: `example/multimedia/lvgl/lvgl_v8_imgbar`
