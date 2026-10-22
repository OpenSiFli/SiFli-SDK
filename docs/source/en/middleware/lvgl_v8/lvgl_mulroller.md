# LVGL v8 mulroller

`lvsf_mulroller` is a low-level, callback-driven, highly customizable roller / dial selector widget provided by SiFli. It scrolls a strip of elements by dragging; on release it automatically snaps (aligns) to the centered element as the current selection, emphasizing the center element and fading out the flanking elements, producing the classic roller look.

Unlike the declarative `lvsf_multroller`, mulroller does not take an option string directly; instead it fills each slot on demand through callbacks. When a slot scrolls to a new data index, `appear_cb` is called and fills in the content; when the roller settles, `middle_cb` reports the current center index. As a result, only a small number of real elements is needed to cover a large range of values, while layout, scaling, color, opacity, and looping can be finely controlled.

- Main header: _middleware/lvgl/lvsf/gui_widgets/lvsf_mulroller.h_
- Dependencies: `lvsf_baselabel`, `lvsf_baseimg` (must be enabled in lv_conf)
- Example project: _example/multimedia/lvgl/lvgl_v8_mulroller_

## Features

- A draggable roller in either horizontal or vertical direction, snapping to the centered element on release.
- Element content is provided on demand by the `appear_cb` callback; a few elements cover a large range of values.
- Supports four element types: label, image, image array, and custom module.
- The scaling, color, and opacity of the center element and the flanking elements can be configured separately, producing size / color / gradient emphasis.
- Supports bounded looping (stops at the boundary) and infinite looping (wraps around automatically).
- Supports encoder (wheel) input, custom left / right (or top / bottom) edge objects, and custom fonts.

## Use Cases

- Time pickers (multi-column roller combinations of hours / minutes / seconds).
- Scroll selections over a wide range of values, such as date selection (year / month / day) and weekday selection.
- Image rollers, numeric dials, and selection interfaces that need a custom centered emphasis style.

## Supported Boards

The reference example `example/multimedia/lvgl/lvgl_v8_mulroller` (a weekday + hour:minute time picker) is verified on the following boards:

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

The PC simulator is also supported (`scons --board=pc_hcpu`).

## Configuration and Initialization

Enable the corresponding component in menuconfig (enabled by default):

```none
CONFIG_LVSF_USE_MULROLLER=y
```

Call flow: create the object → set the element type → create elements → register content / selection callbacks → set direction / alignment / layout / looping → set scaling / color / opacity ranges → finally call `lv_mulroller_validate()` to apply the configuration.

```c
#include "lvsf_mulroller.h"

/* Fill content into each slot while scrolling; idx is the data value of that slot */
static bool num_appear_cb(lv_obj_t *child, int16_t idx)
{
    lv_label_set_text_fmt(child, "%02d", idx);
    return true;
}

/* Callback when settled; idx is the current centered (selected) value */
static bool hh_middle_cb(lv_obj_t *child, int16_t idx)
{
    /* Record the selected hour and refresh the reading */
    return true;
}

lv_obj_t *r = lv_mulroller_create(parent);
lv_obj_set_size(r, 78, 156);                       /* The window must be smaller than the total element height for scrolling */
lv_mulroller_set_obj_type(r, MULROLLER_TYPE_LABEL);
lv_mulroller_create_element(r, 5, 78, 52);         /* 5 elements, 3 visible */

lv_mulroller_set_appear_cb(r, num_appear_cb);
lv_mulroller_set_middle_cb(r, hh_middle_cb);

lv_mulroller_set_dir(r, MULROLLER_DIR_VER);
lv_mulroller_set_align(r, MULROLLER_ALIGN_CENTER);
lv_mulroller_set_layout_mode(r, MULROLLER_LAYOUT_MID);
lv_mulroller_set_circle_mode(r, MULROLLER_CIRCLE_NORMAL);
lv_mulroller_set_circle_range(r, 0, 23);            /* value range 00..23 */

lv_mulroller_set_zoom_range(r, 36, 24);             /* LABEL: value = font size; large in the center, small on the sides */
lv_mulroller_set_color_mode(r, MULROLLER_COLOR_POS);
lv_mulroller_set_color_range(r, 0xFF0000, 0x9E9E9E);/* center red -> sides gray */
lv_mulroller_set_opa_mode(r, MULROLLER_OPA_MID);
lv_mulroller_set_opa_range(r, 255, 130);            /* center opaque -> sides fade out */

lv_mulroller_validate(r);                          /* Apply the settings above */
```

```{warning}
- Whether scrolling is possible depends on geometry: the window must be smaller than the total size of all elements (height for vertical, width for horizontal), so create more elements than are visible.
- Infinite looping mode (`MULROLLER_CIRCLE_INFINITE`) cannot be used together with `MULROLLER_LAYOUT_OVERLAP`, and the element count must be at least 2.
- `lv_mulroller_set_opa_range()` is only effective under `MULROLLER_CIRCLE_NORMAL`; infinite mode ignores it.
- All settings take effect only after `lv_mulroller_validate()` is called.
```

## API Reference

The function signatures below are taken verbatim from _lvsf_mulroller.h_.

### Creation and Elements

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_obj_t *lv_mulroller_create(lv_obj_t *parent)` | Creates a mulroller roller object | `parent`: parent object; returns the pointer to the new object |
| `void lv_mulroller_create_element(lv_obj_t *mulroller, uint8_t ele_num, lv_coord_t w, lv_coord_t h)` | Creates roller elements | `ele_num`: number of elements; `w` / `h`: size of one element; element type is determined by `set_obj_type` |
| `void lv_mulroller_set_element(lv_obj_t *mulroller, lv_obj_t *ele, uint8_t ele_idx, uint8_t data_idx)` | Sets a custom element | Only used with `MULROLLER_TYPE_MODULE`; `ele_idx` is the element slot, `data_idx` the corresponding data index |
| `void lv_mulroller_bind_attr(lv_obj_t *mulroller, const lv_mulroller_attr_t *attr)` | Binds an attribute table | `attr`: pointer to a `lv_mulroller_attr_t` attribute table |
| `void lv_mulroller_validate(lv_obj_t *mulroller)` | Validates and applies the configuration | Must be called after all settings are done |
| `void lv_mulroller_trans_refresh(lv_obj_t *mulroller)` | Refreshes the transfer state | For internal refresh use |
| `lv_obj_t *lv_mulroller_get_bg_obj(lv_obj_t *mulroller, uint8_t idx)` | Gets an element background object by index | Returns the element background object pointer |
| `void lv_mulroller_align_all_element(lv_obj_t *mulroller, lv_obj_t *bg_obj)` | Aligns all elements | `bg_obj`: background object |

### Direction, Type, and Layout

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_mulroller_set_dir(lv_obj_t *mulroller, lv_mulroller_dir_t dir)` | Sets the drag direction | `dir`: `MULROLLER_DIR_HOR` / `MULROLLER_DIR_VER` |
| `void lv_mulroller_set_obj_type(lv_obj_t *mulroller, lv_mulroller_obj_type_t obj_type)` | Sets the element object type | `obj_type`: `MULROLLER_TYPE_LABEL/IMG/IMGARRAY/MODULE` |
| `void lv_mulroller_set_layout_mode(lv_obj_t *mulroller, lv_mulroller_layout_mode_t layout_mode)` | Sets the layout mode | Supports `MULROLLER_LAYOUT_OVERLAP/MID/RHOMB` |
| `void lv_mulroller_set_align(lv_obj_t *mulroller, lv_mulroller_align_t align)` | Sets the element alignment | See `lv_mulroller_align_t` (LEFT/RIGHT/TOP/BOTTOM/CENTER) |
| `void lv_mulroller_set_interval(lv_obj_t *mulroller, lv_coord_t interval)` | Sets the spacing between elements | `interval`: element spacing (pixels) |
| `void lv_mulroller_set_offset(lv_obj_t *mulroller, lv_coord_t offset_lt, lv_coord_t offset_rb)` | Sets the top-left / bottom-right offset | — |
| `void lv_mulroller_set_custom_obj(lv_obj_t *mulroller, lv_obj_t *obj_lt, lv_obj_t *obj_rb)` | Sets a custom left / right (or top / bottom) edge object | Shown when the roller moves to the corresponding edge |

### Visual Emphasis (Scale / Color / Opacity)

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_mulroller_set_zoom_range(lv_obj_t *mulroller, uint16_t middle_zoom, uint16_t lateral_zoom)` | Sets the scaling range for the center / flanking elements | For the LABEL type the value denotes the font size; large in the center, small on the sides |
| `void lv_mulroller_set_color_mode(lv_obj_t *mulroller, lv_mulroller_color_mode_t color_mode)` | Sets the color variation mode | Currently mainly uses `MULROLLER_COLOR_POS` (color varies with position) |
| `void lv_mulroller_set_color_range(lv_obj_t *mulroller, uint32_t middle_color, uint32_t lateral_color)` | Sets the center / flanking color range | LABEL type only |
| `void lv_mulroller_set_opa_mode(lv_obj_t *mulroller, lv_mulroller_opa_mode_t opa_mode)` | Sets the opacity variation mode | `MULROLLER_OPA_NULL/MID/GRAD` |
| `void lv_mulroller_set_opa_range(lv_obj_t *mulroller, uint8_t middle_opa, uint8_t lateral_opa)` | Sets the center / flanking opacity range | Only effective in NORMAL looping mode |
| `void lv_mulroller_set_high_light(lv_obj_t *mulroller, bool high_light)` | Whether to show a highlight outline on the center element | — |
| `void lv_mulroller_set_custom_font(lv_obj_t *mulroller, bool en)` | Uses a custom font | When `en=true`, mulroller does not set the font and size for labels; LABEL type only |

### Looping and Range

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_mulroller_set_circle_mode(lv_obj_t *mulroller, lv_mulroller_circle_mode_t circle_mode)` | Sets the looping mode | `MULROLLER_CIRCLE_NORMAL` bounded / `MULROLLER_CIRCLE_INFINITE` infinite |
| `void lv_mulroller_set_circle_range(lv_obj_t *mulroller, int16_t min, int16_t max)` | Sets the value range for bounded looping | Used only by `MULROLLER_CIRCLE_NORMAL` |
| `void lv_mulroller_set_ori_mid_idx(lv_obj_t *mulroller, int16_t idx)` | Sets the original center index at initialization | — |
| `int16_t lv_mulroller_get_mid_idx(lv_obj_t *mulroller)` | Gets the current center (selected) index | Returns the current centered data index |
| `void lv_mulroller_set_wheel_scale(lv_obj_t *mulroller, float wheel_scale)` | Sets the scaling factor converting each wheel (encoder) detent to a distance | — |
| `void lv_mulroller_set_throw_scale(lv_obj_t *mulroller, float throw_scale)` | Sets the scaling factor converting each fling (inertia) pixel to a distance | — |

### Callbacks

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_mulroller_set_appear_cb(lv_obj_t *mulroller, lv_mulroller_appear_cb appear_lt_cb)` | Called when an element enters a new data index | Fill in the element's display content here |
| `void lv_mulroller_set_middle_cb(lv_obj_t *mulroller, lv_mulroller_appear_cb middle_cb)` | Called when the roller settles; reports the current center index | The entry point for reading the selected value |
| `void lv_mulroller_set_middle_cb2(lv_obj_t *mulroller, lv_mulroller_appear_cb middle_cb)` | Called while the roller is moving | Triggered continuously during movement |

### Animation and Others

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `bool lv_mulroller_create_anim(lv_obj_t *mulroller, lv_mulroller_anim_type type, int16_t num, uint32_t time, lv_mulroller_anim_ready_cb cb)` | Creates a roller animation | `type`: animation type; `num`: number of animations; `time`: duration; `cb`: completion callback |
| `void lv_mulroller_del_anim(lv_obj_t *mulroller)` | Deletes the current animation | — |
| `void lv_mulroller_set_moving_percent(lv_obj_t *mulroller, int16_t percent, int16_t param)` | Sets the movement percentage | — |
| `void lv_mulroller_set_snapshot(lv_obj_t *mulroller, bool en)` | Enables / disables element snapshots | Available only when `LV_OBJ_SNAPSHOT` is enabled |
| `void lv_mulroller_extend_area(lv_obj_t *mulroller, lv_coord_t size)` | Extends the roller area | Reserved in the current implementation |
| `void lv_mulroller_create_mask(lv_obj_t *mulroller, lv_mulroller_dir_t dir, const void *img_src)` | Creates a directional mask | Reserved in the current implementation |
| `void lv_mulroller_encoder_enable(lv_obj_t *mulroller, bool en)` | Enables / disables encoder input | Available only when `LVSF_USING_ENCODER` is enabled |

## Typical Usage

The complete runnable example is in `example/multimedia/lvgl/lvgl_v8_mulroller`, which combines a top horizontal infinite-loop weekday wheel with two vertical bounded numeric wheels (hour:minute) in the middle. The core pattern:

```c
/* Horizontal infinite-loop weekday wheel: set direction to HOR and looping to INFINITE; in appear_cb look up the name table by idx modulo 7 */
static bool week_appear_cb(lv_obj_t *label, int16_t idx)
{
    static const char *names[7] = { "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun" };
    lv_label_set_text(label, names[idx % 7]);
    return true;
}

lv_obj_t *week = lv_mulroller_create(parent);
lv_obj_set_size(week, 240, 60);
lv_mulroller_set_obj_type(week, MULROLLER_TYPE_LABEL);
lv_mulroller_create_element(week, 5, 80, 60);
lv_mulroller_set_appear_cb(week, week_appear_cb);
lv_mulroller_set_dir(week, MULROLLER_DIR_HOR);
lv_mulroller_set_circle_mode(week, MULROLLER_CIRCLE_INFINITE);
lv_mulroller_validate(week);
```

```{note}
mulroller and multroller are complementary: multroller only needs a `'\n'`-separated option string and suits the simple "give a set of text, pick one" case; mulroller fills content through callbacks and exposes all the details of layout / scaling / color / looping, suiting highly customized roller interfaces.
```

## Demo

Drag, release-snap, center-emphasis with side fading effect (see the multroller roller animation):

```{image} ../../../assets/lvgl_v8/multroller.gif
:alt: mulroller demo
:width: 400px
:align: center
```

## See Also

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- Header: _middleware/lvgl/lvsf/gui_widgets/lvsf_mulroller.h_
- Example project: _example/multimedia/lvgl/lvgl_v8_mulroller_
- Related widget: `lvsf_multroller` (declarative roller)
