# LVGL v8 arctext (curvetext)

SiFli SDK provides arc text (Arc Text) display capability on LVGL v8, with the corresponding SDK header `lvsf_curvetext.h` (widget class name `lvsfcurve`). Built on top of `lv_canvas`, it draws characters arranged along an arc, supports setting the text angle and radius along the arc, and can draw an arc background line. It is commonly used for arc text layout scenarios such as watch-face edges and ring-menu titles.

```{note}
The `lv_arctext_*` interfaces used in the solution document differ in naming from the `lv_lvsfcurve_*` interfaces in the current SDK v8 header `lvsf_curvetext.h`. The API table in this document follows the SDK header; the alignment, equal-spacing, mirroring, and other feature descriptions related to `lv_arctext_*` can serve as a design reference.
```

## Features

- Draws text arranged along an arc, with a configurable start angle and radius.
- Supports drawing an arc background line (radius, start/end angle, color, line width).
- Supports setting the rotation pivot point.
- Depends on the LVGL canvas (`LV_USE_CANVAS`) as the drawing backend.

## Use Cases

- Text wrapping around the outer edge of a watch face.
- Title text along the edge of a ring menu / circular widget.
- Interfaces that require text arranged along an arc together with arc scale lines.

## Supported Boards

Any platform supported by the general LVGL v8 examples, and any board from 55x onward (such as 58x, 56x, 52x). Enable `LV_USE_CANVAS` in `lv_conf.h` and `LVSF_USE_CURVE` in `menuconfig`.

## Configuration and Initialization

Arc text depends on the LVGL canvas, which must first be enabled in `lv_conf.h`:

```none
LV_USE_CANVAS  1
```

Then enable `LVSF_USE_CURVE` under `LittlevGL2RTT -> SiFli extend` in `menuconfig`.

```c
#include "lvsf/lvsf_curvetext.h"
```

## API Reference

The interface signatures below are taken verbatim from `middleware/lvgl/lvsf/lvsf_curvetext.h`:

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_obj_t *lv_lvsfcurve_create(lv_obj_t *parent)` | Create an arc text (curve) object | `parent`: parent object; returns the created object pointer |
| `void lv_lvsfcurve_set_buf(lv_obj_t *curve, uint16_t txt_width, uint16_t txt_height)` | Set the arc text drawing buffer size | `txt_width`/`txt_height`: text area width and height (pixels) |
| `void lv_lvsfcurve_set_pivot(lv_obj_t *curve, lv_coord_t x, lv_coord_t y)` | Set the rotation pivot point | `x`/`y`: pivot coordinates |
| `void lv_lvsfcurve_draw_arc(lv_obj_t *curve, lv_coord_t r, int32_t start_angle, int32_t end_angle, lv_color_t color, lv_coord_t width)` | Draw an arc background line | `r`: radius; `start_angle`/`end_angle`: start/end angles; `color`: color; `width`: line width |
| `void lv_lvsfcurve_text(lv_obj_t *curve, char *text, int angle, int r, lv_color_t color, int size)` | Draw text along the arc | `text`: text to display; `angle`: start angle; `r`: radius; `color`: color; `size`: font size |

## Typical Usage

Create an arc text object, set the buffer and pivot point, then draw the text along the arc and an arc line:

```c
lv_obj_t *arc = lv_lvsfcurve_create(parent);
lv_obj_set_size(arc, LV_HOR_RES_MAX >> 1, LV_HOR_RES_MAX >> 1);
lv_obj_center(arc);

/* Set the drawing buffer and rotation pivot */
lv_lvsfcurve_set_buf(arc, LV_HOR_RES_MAX >> 1, LV_HOR_RES_MAX >> 1);
lv_lvsfcurve_set_pivot(arc, LV_HOR_RES_MAX >> 2, LV_HOR_RES_MAX >> 2);

/* Draw text along the arc: start from the 0-degree direction, radius 80, white */
lv_lvsfcurve_text(arc, "ARCTEXT", 0, 80, LV_COLOR_WHITE, 20);

/* Draw an arc background line */
lv_lvsfcurve_draw_arc(arc, 80, 0, 360, LV_COLOR_BLUE, 2);
```

## Demo

```{image} ../../../assets/lvgl_v8/arctext.gif
:alt: arctext arc text demo
```

## See Also

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- Source path: `middleware/lvgl/lvsf/lvsf_curvetext.h`
