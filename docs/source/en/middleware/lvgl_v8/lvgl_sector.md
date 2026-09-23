# LVGL v8 Sector Mask (Sector)

`lvsf_sector` is a custom widget that SiFli wraps around `lv_img`. It uses a **sector (angle) mask** to reveal an image as a pie shape — it maps a value to an angle, so the larger the value, the wider the revealed sector angle. First set the source image with `lv_img_set_src`, then set the angle span and value range, allocate the mask, and drive it with `lv_sector_set_value`.

- Main header: _middleware/lvgl/lvsf/gui_widgets/lvsf_sector.h_
- Range setting: _middleware/lvgl/lvsf/gui_widgets/lvsf_obj_ext.h_ (`lv_obj_set_range_scale` / `lv_obj_set_range_value`)
- Dependent widget: LVGL img (`LV_USE_IMG`)
- Example project: _example/multimedia/lvgl/lvgl_v8_sector_

## Features

- Reveals an image as a pie/sector with an angle mask, mapping a value to an angle.
- Based on `lv_img`, reusing `lv_img_set_src` directly to set the source image.
- Supports configuring the angle span (`lv_obj_set_range_scale`) and the value range (`lv_obj_set_range_value`).
- Supports dragging to change the sector progress.
- Supports an indicator image and its X/Y offset.
- Allocates the angle mask buffer through `lv_sector_validate()`.

## Use Cases

- Pie charts and ring progress gauges.
- Clock dials, volume/battery sector indicators.
- Circular progress reveal animations that map a value to an angle.

## Supported Boards

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## Configuration and Initialization

`lvsf_sector` depends on the LVGL img component. In menuconfig, confirm it is enabled:

```none
CONFIG_LV_USE_IMG=y
```

In the BSP, the `LVSF_USE_SECTOR` macro controls whether this widget is compiled.

Minimum flow to create an object:

```c
#include "lvsf_sector.h"
#include "lvsf_obj_ext.h"

lv_obj_t *sector = lv_sector_create(lv_scr_act());
lv_img_set_src(sector, &img_dsc);
```

## API Reference

### Creation and refresh

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_obj_t *lv_sector_create(lv_obj_t *parent)` | Creates a sector object | `parent`: parent object; returns the object pointer on success, or `NULL` on failure |
| `void lv_sector_refresh_timer(lv_timer_t *timer)` | Refresh timer callback | `timer`: timer object; periodically refreshes the sector display |

### Configuration and validation

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_sector_validate(lv_obj_t *sector)` | Validates and allocates the angle mask buffer | `sector`: sector object; must be called after setting the angle span and value range, before `lv_sector_set_value()` can be used |
| `void lv_sector_refresh_mask_range(lv_obj_t *sector, int32_t min, int32_t max, uint8_t value)` | Refreshes the mask range | `min`/`max`: minimum/maximum value; `value`: current value |
| `void lv_sector_set_value(lv_obj_t *sector, int32_t value)` | Sets the sector value | `value`: mapped to an angle by the value range; the larger the value, the wider the revealed sector angle |
| `void lv_sector_set_drag(lv_obj_t *sector, bool en)` | Enables/disables dragging | `en`: `true` allows dragging to change the progress |

### Indicator and image

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_sector_set_img_indicator(lv_obj_t *sector, lv_obj_t *img_indicator)` | Sets the indicator image | `img_indicator`: indicator image object |
| `void lv_sector_set_indicator_offset(lv_obj_t *sector, lv_coord_t x, lv_coord_t y)` | Sets the indicator offset | `x`/`y`: X/Y offset |
| `void lv_sector_set_indicator_offset_x(lv_obj_t *sector, lv_coord_t x)` | Sets the indicator X offset | `x`: X offset |
| `void lv_sector_set_indicator_offset_y(lv_obj_t *sector, lv_coord_t y)` | Sets the indicator Y offset | `y`: Y offset |

### Query interfaces

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `int16_t lv_sector_get_start_angle(lv_obj_t *sector)` | Gets the start angle | Returns the start angle |
| `int16_t lv_sector_get_end_angle(lv_obj_t *sector)` | Gets the end angle | Returns the end angle |
| `lv_obj_t *lv_sector_get_img_bg(lv_obj_t *sector)` | Gets the background image | Returns the background image object pointer |
| `lv_obj_t *lv_sector_get_img_indicator(lv_obj_t *sector)` | Gets the indicator image | Returns the indicator image object pointer |

## Typical Usage

```c
#include "lvsf_sector.h"
#include "lvsf_obj_ext.h"   /* lv_obj_set_range_scale / lv_obj_set_range_value */

lv_obj_t *sector = lv_sector_create(parent);
lv_img_set_src(sector, &img_dsc);            /* Source image (sector is based on lv_img) */
lv_obj_set_size(sector, 140, 140);
lv_obj_set_style_img_opa(sector, LV_OPA_COVER - 1, LV_PART_MAIN);  /* Prevent ghosting */
lv_obj_set_range_scale(sector, 0, 360);      /* Angle span (full circle) */
lv_obj_set_range_value(sector, 0, 100);       /* Value range */
lv_sector_validate(sector);                   /* Allocate the angle mask buffer */

lv_sector_set_value(sector, 50);              /* Value -> angle: 50 means half a circle, 180° */
```

```{warning}
- After setting the angle span and value range, you must first call `lv_sector_validate()` to allocate the angle mask buffer before `lv_sector_set_value()` can be used.
- Set the image opacity slightly below `LV_OPA_COVER` (i.e. `LV_OPA_COVER - 1`): this makes the underlying `lv_img` cover-check return NOT_COVER, so the parent repaints the area masked by the sector; otherwise the masked region leaves a ghost.
```

## Demo

Run the `lvgl_v8_sector` example to see the result: an orange image is shown in the center of the screen, cut into a pie shape by a sector mask. A timer drives the value back and forth between 0~100, so the sector angle repeatedly grows to a full circle and then shrinks back.

## See Also

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [LVGL v8 Image documentation](https://docs.lvgl.io/8.3/widgets/img.html)
- Source path: `middleware/lvgl/lvsf/gui_widgets/lvsf_sector.h`
- Example path: `example/multimedia/lvgl/lvgl_v8_sector`
