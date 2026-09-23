# LVGL v8 gif (gif_anim)

SiFli SDK provides GIF animation playback capability on LVGL v8, composed of two headers:

- **`lvsf_gif.h`**: the basic GIF widget (`lv_sfgif_*`), which provides GIF decoding and playback, cover display, pause/resume, zoom, and other controls.
- **`lvsf_gif_anim.h`**: a lightweight `gif_anim` widget (`lvsf_gif_anim_*`) wrapped on top of the basic widget, which additionally provides a cover image, delayed start, display layer, and overall zoom.

It suits animated effects in all scenarios. The low-level `lv_gif_dec_*` decoding interfaces are called internally by the widget; application code should use the public `lv_sfgif_*` / `lvsf_gif_anim_*` interfaces.

## Features

- Decodes and plays GIF animation, supporting cover (first frame) image display.
- Supports pause / resume playback and closing to release resources.
- Supports delayed-start playback and a configurable frame interval.
- The gif_anim widget supports aspect-ratio-preserving zoom, setting the display layer, and enabling/disabling background-color processing.
- Supports GIF zoom (`lv_sfgif_set_zoom`, where 256 means the original size).

## Use Cases

- App startup/loading animations, charging animations, and page background animations.
- Scenarios that need to show a cover image first and then play the GIF after a delay.
- Interfaces that need to place the GIF in a foreground/background layer.

## Supported Boards

Any platform supported by the general LVGL v8 examples, and any board from 55x onward (such as 58x, 56x, 52x).

## Configuration and Initialization

The basic GIF widget and the gif_anim widget are provided by the gui_widgets component and are enabled as part of the LVGL v8 build:

```c
#include "lvsf/gui_widgets/lvsf_gif.h"
#include "lvsf/gui_widgets/lvsf_gif_anim.h"
```

gif_anim layer enumeration (`lvsf_gif_layer_t`): `LVSF_GIF_LAYER_DEFAULT`, `LVSF_GIF_LAYER_FOREGROUND`, `LVSF_GIF_LAYER_BACKGROUND`.

## API Reference

### Basic GIF widget (`lvsf_gif.h`)

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_obj_t *lv_sfgif_create(lv_obj_t *parent)` | Create a GIF widget instance | `parent`: parent object; returns the widget pointer |
| `void lv_sfgif_open(lv_obj_t *gif, const char *gif_data, const char *src_img, lv_coord_t x, lv_coord_t y, uint32_t anim_time, uint16_t period)` | Open the GIF resource, optionally setting the cover image, position, delayed start, and frame interval at the same time | `gif_data`: GIF data; `src_img`: cover image data, may be `NULL`; `x`/`y`: position; `anim_time`: delayed-start time, 0 plays immediately; `period`: frame interval, 0 uses the default refresh period |
| `void lv_sfgif_resume(lv_obj_t *gif)` | Start or resume GIF playback | No return value |
| `void lv_sfgif_pause(lv_obj_t *gif)` | Pause GIF playback, keeping the widget object | No return value |
| `void lv_sfgif_close(lv_obj_t *gif)` | Close the GIF resource and release the internal GIF or cover object | No return value |
| `void lv_sfgif_set_zoom(lv_obj_t *gif, uint16_t zoom)` | Set the GIF zoom ratio | `zoom`: LVGL image zoom value, 256 means the original size |

### gif_anim extended widget (`lvsf_gif_anim.h`)

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lvsf_gif_anim_t *lvsf_gif_anim_init(lv_obj_t *parent, const void *gif_data, const void *src_img, lv_coord_t x, lv_coord_t y, uint32_t duration, uint32_t delay)` | Create a GIF widget with cover-image and delayed-playback capability | `gif_data`: GIF data; `src_img`: cover image; `x`/`y`: position; `duration`: frame playback interval; `delay`: start delay; returns the widget instance |
| `void lvsf_gif_anim_set_zoom(lvsf_gif_anim_t *gif_anim, int width_zoom, int height_zoom)` | Set the overall zoom | The smaller of the width and height zoom values is taken for the overall scaling |
| `void lvsf_gif_anim_set_layer(lvsf_gif_anim_t *gif_anim, lvsf_gif_layer_t layer)` | Set the display layer | `layer`: layer enumeration |
| `void lvsf_gif_anim_enable_bg_color(lvsf_gif_anim_t *gif_anim, bool enable)` | Enable or disable background-color processing | `enable`: whether to enable background color |
| `void lvsf_gif_anim_resume(lvsf_gif_anim_t *gif_anim)` | Start or resume playback | No return value |
| `void lvsf_gif_anim_pause(lvsf_gif_anim_t *gif_anim)` | Pause playback | No return value |
| `void lvsf_gif_anim_deinit(lvsf_gif_anim_t *gif_anim)` | Destroy the widget instance and release resources | No return value |

## Typical Usage

Basic GIF widget:

```c
lv_obj_t *bg_gif = lv_sfgif_create(lv_scr_act());

/* Open the GIF: no cover image, position (0,0), play immediately, frame interval 50ms */
lv_sfgif_open(bg_gif,
              APP_GET_gif(img_gif_demo),
              NULL,
              0, 0,
              0,
              50);

lv_obj_align(bg_gif, LV_ALIGN_CENTER, 0, 0);
lv_sfgif_resume(bg_gif);
lv_obj_move_background(bg_gif);
```

gif_anim widget (with cover image and delayed start), controlled within the page lifecycle:

```c
static lvsf_gif_anim_t *charge_gif = NULL;

static void on_start(void)
{
    charge_gif = lvsf_gif_anim_init(parent,
                                    APP_GET_gif(img_gif_demo),
                                    APP_GET_IMG(img_gif_demo_surface),
                                    0, 0,
                                    LV_DISP_DEF_REFR_PERIOD * 3,
                                    100);
}

static void on_resume(void)  { lvsf_gif_anim_resume(charge_gif); }
static void on_pause(void)  { lvsf_gif_anim_pause(charge_gif); }
static void on_stop(void)   { lvsf_gif_anim_deinit(charge_gif); charge_gif = NULL; }
```

## Demo

```{image} ../../../assets/lvgl_v8/gif_demo_src.png
:alt: gif example resource
```

```{image} ../../../assets/lvgl_v8/gif_demo.gif
:alt: gif playback demo
```

## Notes

- GIF images do not support transparency; if you need a transparent effect, use the APNG widget or the seqframe frame-sequence widget.
- The public entry point of the basic GIF widget is `lv_sfgif_*`; do not depend directly on the low-level `lv_gif_dec_*` interfaces.
- When a page is destroyed or switched, it is recommended to call `lv_sfgif_close()` or `lvsf_gif_anim_deinit()` to release resources.

## See Also

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- Source paths: `middleware/lvgl/lvsf/gui_widgets/lvsf_gif.h`, `middleware/lvgl/lvsf/gui_widgets/lvsf_gif_anim.h`
