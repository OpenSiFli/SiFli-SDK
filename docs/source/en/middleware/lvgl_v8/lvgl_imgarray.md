# LVGL v8 Imgarray

`lvsf_imgarray` is a custom widget that SiFli wraps around `lvsf_baseimg`. It assembles a numeric value from a set of images, like a seven-segment/flip display. It can display the full form of a number — **sign + integer digits + decimal point + fractional digits + unit** — where each glyph is an image. Give it a set of glyph images (0~9 and the decimal point, unit, and so on), the number of integer/fractional digits, and the indices of the decimal-point and unit glyphs, then feed it a value; it assembles `[sign][integer].[fraction][unit]` from those images.

- Main header: _middleware/lvgl/lvsf/gui_widgets/lvsf_imgarray.h_
- Base class and type definitions: _middleware/lvgl/lvsf/gui_widgets/lvsf_baseimg.h_
- Dependent widget: LVGL img (`LV_USE_IMG`)
- Example project: _example/multimedia/lvgl/lvgl_v8_imgarray_

## Features

- Assembles a value from image glyphs, supporting sign, integer digits, decimal point, fractional digits, and unit.
- Supports several value modes: `BASEIMG_TYPE_ARRAY_INDEX` (the value is the image index), `BASEIMG_TYPE_ARRAY_Q248` (Q24.8 fixed-point, supports fractions), and so on.
- Configurable number of integer/fractional digits, and the indices of the decimal point, sign, unit, and empty glyph in the glyph array.
- Switches to enable leading-zero and trailing-zero display.
- Configurable spacing between images.
- Supports three ways to set the source array: memory descriptor array, file path, and single-source sequence frames.

## Use Cases

- Seven-segment-style numeric readouts (battery, heart rate, temperature, percentage).
- Flip/dial numeric displays that need custom glyph images.
- Real-time numbers on watch dials and instrument clusters.

## Supported Boards

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## Configuration and Initialization

`lvsf_imgarray` depends on the LVGL img component. In menuconfig, confirm it is enabled:

```none
CONFIG_LV_USE_IMG=y
```

In the BSP, the `LVSF_USE_IMGARRAY` macro controls whether this widget is compiled.

Minimum flow to create an object:

```c
#include "lvsf_imgarray.h"
#include "lvsf_baseimg.h"

lv_obj_t *ia = lv_imgarray_create(lv_scr_act());
```

## API Reference

### Creation and refresh

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_obj_t *lv_imgarray_create(lv_obj_t *parent)` | Creates an image array object | `parent`: parent object; returns the object pointer on success, or `NULL` on failure |
| `void lv_imgarray_refresh_timer(lv_timer_t *timer)` | Refresh timer callback | `timer`: timer object; periodically refreshes the image array display |

### Source array settings

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_imgarray_set_src_array(lv_obj_t *img, const lv_img_dsc_t **dsc_array, int16_t index_start, int16_t index_end)` | Sets the source array (memory descriptors) | `img`: image array object; `dsc_array`: descriptor array; `index_start`/`index_end`: start/end index |
| `void lv_imgarray_set_src_array2(lv_obj_t *img, char *file_path, lv_img_file_data_t *dsc_array, int16_t index_start, int16_t index_end)` | Sets the source array by file path | `file_path`: file path; `dsc_array`: file descriptor array; `index_start`/`index_end`: start/end index |
| `void lv_imgarray_set_src_array3(lv_obj_t *img, const void *src, int16_t index_start, int16_t index_end)` | Sets the sequence-frame image source and index range | `src`: image source; `index_start`/`index_end`: start/end index |

### Layout and glyph configuration

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_imgarray_set_img_type(lv_obj_t *img, lv_baseimg_type_t img_type)` | Sets the image type | `img_type`: image type (`BASEIMG_TYPE_ARRAY_INDEX` / `BASEIMG_TYPE_ARRAY_Q248`, and so on) |
| `void lv_imgarray_set_int_num(lv_obj_t *img, uint8_t num)` | Sets the number of integer digits | `num`: number of integer-digit images |
| `void lv_imgarray_set_float_num(lv_obj_t *img, uint8_t num)` | Sets the number of fractional digits | `num`: number of fractional-digit images |
| `void lv_imgarray_set_unit_img(lv_obj_t *img, lv_obj_t *unit_img)` | Sets the unit image object | `unit_img`: unit image object pointer |
| `void lv_imgarray_set_point_idx(lv_obj_t *img, uint16_t idx)` | Sets the decimal-point glyph index | `idx`: index of the decimal point in the glyph array |
| `void lv_imgarray_set_negative_idx(lv_obj_t *img, uint16_t idx)` | Sets the sign glyph index | `idx`: index of the sign in the glyph array |
| `void lv_imgarray_set_unit_idx(lv_obj_t *img, uint16_t idx)` | Sets the unit glyph index | `idx`: index of the unit in the glyph array |
| `void lv_imgarray_set_empty_idx(lv_obj_t *img, uint16_t idx)` | Sets the empty glyph index | `idx`: index of the empty slot in the glyph array |
| `void lv_imgarray_set_leading_zero(lv_obj_t *img, bool leading_zero)` | Sets leading-zero display | `leading_zero`: `true` shows, `false` hides |
| `void lv_imgarray_set_trailing_zero(lv_obj_t *img, bool trailing_zero)` | Sets trailing-zero display | `trailing_zero`: `true` shows, `false` hides |
| `void lv_imgarray_set_interval(lv_obj_t *img, int16_t interval)` | Sets the image spacing | `interval`: spacing value between images |
| `uint16_t lv_imgarray_get_src_num(lv_obj_t *img)` | Gets the source count | Returns the source count |

### Value settings

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_imgarray_set_value(lv_obj_t *img, int32_t value)` | Sets the value (Q24.8 format) | `value`: Q24.8-format value; in Q24.8 mode pass `value * 256` |
| `void lv_imgarray_set_value2(lv_obj_t *img, int32_t value)` | Sets the value (integer format) | `value`: integer value |

## Typical Usage

The following example displays a measurement like `36.5%` (2 integer digits + 1 fractional digit + decimal point + percent-unit):

```c
#include "lvsf_baseimg.h"
#include "lvsf_imgarray.h"

/* glyph_arr: digits 0..9 + "." (index 10) + "%" (index 11) */
extern const lv_img_dsc_t *glyph_arr[12];

void demo_imgarray_init(void)
{
    lv_obj_t *ia = lv_imgarray_create(lv_scr_act());
    lv_imgarray_set_img_type(ia, BASEIMG_TYPE_ARRAY_Q248);   /* Q24.8: supports fractions */
    lv_imgarray_set_src_array(ia, glyph_arr, 0, 11);        /* Store the glyph array first */
    lv_imgarray_set_interval(ia, 2);
    lv_imgarray_set_leading_zero(ia, true);
    lv_imgarray_set_trailing_zero(ia, true);
    lv_imgarray_set_int_num(ia, 2);                         /* 2 integer digits */
    lv_imgarray_set_float_num(ia, 1);                       /* 1 fractional digit */
    lv_imgarray_set_point_idx(ia, 10);                      /* Decimal point uses index 10 */
    lv_imgarray_set_unit_idx(ia, 11);                       /* Unit uses index 11 */
    lv_obj_center(ia);
    lv_imgarray_set_value(ia, 365 * 256 / 10);              /* 36.5 (Q24.8): shows "36.5%" */
}
```

```{warning}
The call order matters: you must first call `lv_imgarray_set_src_array()` to store the glyph array, and only then call `set_int_num()` / `set_point_idx()` / `set_unit_idx()` and so on — these interfaces create the corresponding image cells and apply the stored array. If the order is reversed, it fails because the array is not ready.
```

## Demo

Run the `lvgl_v8_imgarray` example to see the result: a decimal with a unit (such as `38.5%`) is shown in the center of the screen, assembled from the integer digits, decimal point, fractional digits, and unit images. A timer sweeps the value back and forth between 20.0~80.0%, so the reading keeps changing.

## See Also

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [LVGL v8 Image documentation](https://docs.lvgl.io/8.3/widgets/img.html)
- Source path: `middleware/lvgl/lvsf/gui_widgets/lvsf_imgarray.h`
- Example path: `example/multimedia/lvgl/lvgl_v8_imgarray`
