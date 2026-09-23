# LVGL v8 multslider

`lvsf_multslider` is a numeric slider widget that SiFli wraps on top of LVGL v8, used to adjust a single-dimension value in a visual way. It supports a custom value range, a displayed text label, and animated or immediate value updates, and is commonly used for settings such as volume, brightness, and parameter thresholds.

- Main header: _middleware/lvgl/lvsf/gui_widgets/lvsf_multslider.h_
- Example project: _example/multimedia/lvgl/lvgl_v8_multslider_

## Features

- Built on LVGL's native slider; the current value, minimum, and maximum can be set.
- Supports a text label (such as "Volume") shown on the widget.
- Whether to animate the transition to the target value is selectable.
- The color and opacity of the background, indicator, and knob can be customized through the standard LVGL style APIs.

## Use Cases

- Adjusting values of system settings such as volume, brightness, and contrast.
- Dragging to adjust parameter thresholds and other single-dimension progress-like values.
- Horizontal sliders that need a text label ("-", "+", or a name) overlaid on the knob.

## Supported Boards

The reference example `example/multimedia/lvgl/lvgl_v8_multslider` is verified on the following boards:

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

Board projects are generated with `scons --board=<board>`.

## Configuration and Initialization

Enable the corresponding component in menuconfig (enabled by default):

```none
CONFIG_LVSF_USE_MULTSLIDER=y
```

After creation, set the size, range, and initial value, and adjust the appearance of each part through the standard LVGL style APIs:

```c
#include "lvsf_multslider.h"

lv_obj_t *slider = lv_multslider_create(parent);
lv_obj_set_size(slider, LV_HOR_RES_MAX - 40, 60);
lv_obj_align(slider, LV_ALIGN_CENTER, 0, 0);

/* Value range and initial value */
lv_multslider_set_range(slider, 0, 100);
lv_multslider_set_value(slider, 50, LV_ANIM_ON);
lv_multslider_set_txt(slider, "Volume");

/* Style: background / indicator / knob */
lv_obj_set_style_bg_color(slider, LV_COLOR_BLACK, LV_PART_MAIN);
lv_obj_set_style_bg_opa(slider, LV_OPA_100, LV_PART_MAIN);
lv_obj_set_style_bg_color(slider, LV_COLOR_GRAY, LV_PART_INDICATOR);
lv_obj_set_style_bg_opa(slider, LV_OPA_50, LV_PART_INDICATOR);
lv_obj_set_style_bg_color(slider, LV_COLOR_GRAY, LV_PART_KNOB);
lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_KNOB);
```

## API Reference

The function signatures below are taken verbatim from _lvsf_multslider.h_.

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_obj_t *lv_multslider_create(lv_obj_t *parent)` | Creates a multslider slider object | `parent`: parent object; returns the pointer to the new object |
| `void lv_multslider_set_txt(lv_obj_t *multslider, const char *txt)` | Sets the text label | `txt`: the text string to display |
| `void lv_multslider_set_value(lv_obj_t *multslider, int32_t value, lv_anim_enable_t anim)` | Sets the current value | `value`: target value; `anim`: `LV_ANIM_ON` animated transition / `LV_ANIM_OFF` set immediately |
| `void lv_multslider_set_range(lv_obj_t *multslider, int32_t min, int32_t max)` | Sets the value range | `min`: minimum; `max`: maximum |
| `int32_t lv_multslider_get_value(lv_obj_t *multslider)` | Gets the current value | Returns the current value |
| `int32_t lv_multslider_get_min_value(lv_obj_t *multslider)` | Gets the minimum value | Returns the configured minimum |
| `int32_t lv_multslider_get_max_value(lv_obj_t *multslider)` | Gets the maximum value | Returns the configured maximum |

```{note}
The `lv_gesture_disable()` / `lv_gesture_enable()` calls commonly seen in the example are page-level right-swipe-back gesture management and are not part of the multslider widget API.
```

## Typical Usage

The complete runnable example is in `example/multimedia/lvgl/lvgl_v8_multslider`, which demonstrates a slider style with child widgets below the knob:

```c
static void on_start(void)
{
    lv_obj_t *parent = lv_scr_act();

    lv_obj_t *slider = lv_multslider_create(parent);
    lv_obj_set_size(slider, LV_HOR_RES_MAX - 40, 60);
    lv_obj_refr_size(slider);
    lv_obj_align(slider, LV_ALIGN_CENTER, 0, 0);

    /* Background / indicator / knob style */
    lv_obj_set_style_bg_color(slider, LV_COLOR_BLACK, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(slider, LV_OPA_100, LV_PART_MAIN);
    lv_obj_set_style_bg_color(slider, LV_COLOR_GRAY, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(slider, LV_OPA_50, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(slider, LV_COLOR_GRAY, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_KNOB);

    /* Overlay "-" / "+" / name text on the knob */
    lv_obj_t *lab = lv_label_create(slider);
    lv_label_set_text(lab, "-");
    lv_obj_align(lab, LV_ALIGN_LEFT_MID, 20, 0);

    lab = lv_label_create(slider);
    lv_label_set_text(lab, "+");
    lv_obj_align(lab, LV_ALIGN_RIGHT_MID, -20, 0);

    lab = lv_label_create(slider);
    lv_label_set_text(lab, "multslider");
    lv_obj_align(lab, LV_ALIGN_CENTER, 0, 0);
}
```

Call `lv_multslider_get_value()` to read the current value.

## Demo

Drag the slider to adjust the value, with an overlaid text label:

```{image} ../../../assets/lvgl_v8/multslider.gif
:alt: multslider demo
:width: 400px
:align: center
```

## See Also

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- Header: _middleware/lvgl/lvsf/gui_widgets/lvsf_multslider.h_
- Example project: _example/multimedia/lvgl/lvgl_v8_multslider_
