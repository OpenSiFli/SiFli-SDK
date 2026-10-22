# LVGL v8 arcimg

`lvsf_arcimg` is a custom arc image widget that SiFli built on top of the LVGL `lv_img`. It uses an SRAM mask to reveal an image as an arc segment (an arc-shaped progress bar). It supports setting the center, outer radius, line width, and the background-arc angle range, and drives the currently displayed angle with floating-point precision, with animated transitions.

- Main header: _middleware/lvgl/lvsf/gui_widgets/lvsf_arcimg.h_
- Dependency widget: LVGL img (`LV_USE_IMG`)

```{note}
This widget has no corresponding `lvgl_v8_arcimg` example yet; refer to the header comments and the SDK configuration for detailed usage.
```

## Features

- Uses an SRAM mask to reveal the image as an arc (arc progress bar).
- Configurable center coordinates, outer radius, and arc-line width.
- The start/end angle and direction (clockwise/counterclockwise) of the background arc can be set.
- Supports setting the current angle with floating-point precision and an animated transition duration.
- Also provides an integer-angle setting interface.
- Supports multiple SRAM mask buffers.

## Use Cases

- Arc/ring progress bars and dashboard arc segments.
- Image-style arc indicators on a watch face.
- Arc-shaped value display with smooth animated transitions.

## Supported Boards

See the corresponding lvgl_v8 example or SDK configuration.

## Configuration and Initialization

`lvsf_arcimg` depends on the LVGL img component. Confirm it is enabled in menuconfig:

```none
CONFIG_LV_USE_IMG=y
```

The BSP controls compilation of this widget through the `LVSF_USE_ARCIMG` macro.

Minimum flow to create an object:

```c
#include "lvsf_arcimg.h"

lv_obj_t *arcimg = lv_arcimg_create(lv_scr_act());
```

## API Reference

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_obj_t *lv_arcimg_create(lv_obj_t *parent)` | Create an arc image widget | `parent`: parent object; returns the created widget pointer |
| `void lv_arcimg_set_param(lv_obj_t *arcimg, lv_coord_t cent_x, lv_coord_t cent_y, uint16_t r, uint16_t w, uint8_t buf_cnt)` | Set the arc geometric parameters and the number of SRAM mask buffers | `arcimg`: widget object; `cent_x`/`cent_y`: center coordinates; `r`: outer arc radius; `w`: arc-line width; `buf_cnt`: number of SRAM mask buffers |
| `void lv_arcimg_set_bg_angles(lv_obj_t *arcimg, lv_coord_t start_angle, lv_coord_t end_angle, uint8_t clockwise)` | Set the background-arc angle range | `start_angle`/`end_angle`: start/end angles (degrees); `clockwise`: direction flag, 1 clockwise, 0 counterclockwise |
| `void lv_arcimg_set_angle(lv_obj_t *arcimg, float angle, uint32_t time)` | Set the currently displayed angle with floating-point precision | `angle`: current arc span angle (degrees, supports floating point); `time`: animation time (ms), 0 means immediate |
| `void lv_arcimg_set_angle_int(lv_obj_t *arcimg, lv_coord_t angle, uint32_t time)` | Set the currently displayed angle with an integer | `angle`: current arc span angle (degrees, integer); `time`: animation time (ms), 0 means immediate |
| `float lv_arcimg_get_angle(lv_obj_t *arcimg)` | Get the currently displayed angle | Returns the current arc span angle (degrees) |

## Typical Usage

Based on the header interfaces, the typical usage flow is: create the object → set the geometric parameters → set the background-arc angle range → drive the current angle:

```c
#include "lvsf_arcimg.h"

lv_obj_t *arcimg = lv_arcimg_create(parent);

/* Center (70,70), outer radius 60, line width 8, 2 SRAM mask buffers */
lv_arcimg_set_param(arcimg, 70, 70, 60, 8, 2);

/* Background arc from 135° to 405° (i.e. one full turn), clockwise */
lv_arcimg_set_bg_angles(arcimg, 135, 405, 1);

/* Currently display a 90° arc span, with a 500ms animated transition */
lv_arcimg_set_angle(arcimg, 90.0f, 500);
```

Refer to the header comments for detailed usage.

## Demo

This widget has no corresponding example yet; run the relevant `lvgl_v8` example or SDK configuration to see the actual effect.

## See Also

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- Source path: `middleware/lvgl/lvsf/gui_widgets/lvsf_arcimg.h`
