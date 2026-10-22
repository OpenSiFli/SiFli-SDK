# LVGL v8 Baseimg

`lvsf_baseimg` is a basic image widget that SiFli wraps around LVGL `lv_img`, and it is the base class for image-type widgets such as `lvsf_imgarray`. It supports multiple image display modes: image-group index/value/Q24.8 fixed-point, pointer (stepless/snap), and sequence frames (forward/backward/forward-backward looping). It also supports angle, zoom, rotation pivot, loop count, and value-table mapping.

- Main header: _middleware/lvgl/lvsf/gui_widgets/lvsf_baseimg.h_
- Dependent widget: LVGL img (`LV_USE_IMG`)

```{note}
This widget does not yet have a corresponding `lvgl_v8_baseimg` example. For detailed usage, refer to the header comments and the SDK configuration.
```

## Features

- Supports multiple image types: image-group index (the value is the index), image-group value (a value range maps to one image), image-group Q24.8 fixed-point, stepless pointer, snap pointer, and sequence-frame forward/backward/forward-backward looping.
- Supports three source types: memory descriptor, file, and sequence frame.
- Supports angle, zoom, and rotation pivot X/Y settings.
- Supports sequence-frame start/end index, current index, and empty index settings, plus loop-count control (-1 means infinite loop).
- Supports state callbacks (play start/stop/resume/pause/done, and so on) and index callbacks.
- Supports a value table (`value_table`) for mapping numeric values to images/positions.

## Use Cases

- Sequence-frame animation playback (forward/backward/oscillating loop).
- Pointer-style instrument dials (stepless or snap).
- Image-group index switching and value-to-image-mapping indicators.
- Used as the base class for widgets such as `lvsf_imgarray`.

## Supported Boards

See the corresponding lvgl_v8 example or the SDK configuration.

## Configuration and Initialization

`lvsf_baseimg` depends on the LVGL img component. In menuconfig, confirm it is enabled:

```none
CONFIG_LV_USE_IMG=y
```

In the BSP, the `LVSF_USE_BASEIMG` macro controls whether this widget is compiled.

Minimum flow to create an object:

```c
#include "lvsf_baseimg.h"

lv_obj_t *img = lv_baseimg_create(lv_scr_act());
```

## API Reference

### Enumerations

**Image type `lv_baseimg_type_t`:**

| Value | Meaning |
| --- | --- |
| `BASEIMG_TYPE_ARRAY_INDEX` | Image group, sequence mode; the numeric value is the image index |
| `BASEIMG_TYPE_ARRAY_VALUE` | Image group, value mode; a range of values maps to one image |
| `BASEIMG_TYPE_ARRAY_Q248` | Image group, numeric mode; the data is in Q24.8 format |
| `BASEIMG_TYPE_POINTER` | Pointer, stepless change |
| `BASEIMG_TYPE_POINTER_GRID` | Snap pointer |
| `BASEIMG_TYPE_SEQUENCE` | Sequence frames, continuous forward loop (0x0f) |
| `BASEIMG_TYPE_SEQUENCE_BACK` | Sequence frames, continuous backward loop (0x1f) |
| `BASEIMG_TYPE_SEQUENCE_CIRCLE` | Sequence frames, continuous forward-backward loop (0x10f) |

**Playback state `lv_baseimg_state_t`:**

| Value | Meaning |
| --- | --- |
| `BASEIMG_STATE_NULL` | Empty state |
| `BASEIMG_STATE_PLAY_START` | Sequence-frame playback started (triggered from stopped state or at the very start) |
| `BASEIMG_STATE_PLAY_STOP` | Sequence-frame playback stopped; index reset to zero |
| `BASEIMG_STATE_PLAY_RESUME` | Sequence-frame playback resumed (triggered after being paused) |
| `BASEIMG_STATE_PLAY_PAUSE` | Sequence-frame playback paused; index unchanged |
| `BASEIMG_STATE_PLAY_DONE` | Sequence-frame forward playback finished |
| `BASEIMG_STATE_PLAY_BACK_DONE` | Sequence-frame backward playback finished |

**Source type `lv_baseimg_src_type_t`:**

| Value | Meaning |
| --- | --- |
| `BASEIMG_SRC_TYPE_DSC` | Memory descriptor |
| `BASEIMG_SRC_TYPE_FILE` | File |
| `BASEIMG_SRC_TYPE_SEQUENCE` | Sequence frame |

### Callback types

| Type definition | Description |
| --- | --- |
| `typedef void (*lv_baseimg_index_cb)(lv_obj_t *baseimg)` | Called when the base image is set to a specific index |
| `typedef void (*lv_baseimg_state_cb)(lv_obj_t *baseimg, lv_baseimg_state_t state)` | Called when the base image state changes; `state` is the new state |

### Creation and refresh

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_obj_t *lv_baseimg_create(lv_obj_t *parent)` | Creates a base image object | `parent`: parent object; returns the object pointer on success, or `NULL` on failure |
| `void lv_baseimg_refresh_timer(lv_timer_t *timer)` | Refresh timer callback | `timer`: timer object; periodically refreshes the base image display |

### State and source settings

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_baseimg_set_state(lv_obj_t *img, lv_baseimg_state_t state)` | Sets the state | `state`: state value (playback control) |
| `void lv_baseimg_set_src_array(lv_obj_t *img, const lv_img_dsc_t **dsc_array, int16_t index_star, int16_t index_end)` | Sets the source array (memory descriptors) | `dsc_array`: descriptor array; `index_star`/`index_end`: start/end index |
| `void lv_baseimg_set_src_array2(lv_obj_t *img, char *file_path, lv_img_file_data_t *dsc_array, int16_t index_start, int16_t index_end)` | Sets the source array by file path | `file_path`: file path; `dsc_array`: file descriptor array; `index_start`/`index_end`: start/end index |
| `void lv_baseimg_set_src_array3(lv_obj_t *img, const void *src, int16_t index_start, int16_t index_end)` | Sets the sequence-frame image source and index range | `src`: image source; `index_start`/`index_end`: start/end index |
| `void lv_baseimg_set_state_cb(lv_obj_t *img, lv_baseimg_state_cb cb)` | Sets the state callback | `cb`: callback function called when the state changes |
| `void lv_baseimg_set_value_table(lv_obj_t *img, char *value_table)` | Sets the value table | `value_table`: value table pointer, used for mapping numeric values to images |

### Angle, zoom, and index

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_baseimg_set_angle(lv_obj_t *img, int16_t angle)` | Sets the image angle | `angle`: angle value |
| `void lv_baseimg_set_zoom(lv_obj_t *img, uint16_t zoom)` | Sets the image zoom | `zoom`: zoom ratio |
| `void lv_baseimg_set_start_index(lv_obj_t *img, uint16_t index)` | Sets the start index | `index`: sequence start index |
| `void lv_baseimg_set_end_index(lv_obj_t *img, uint16_t index)` | Sets the end index | `index`: sequence end index |
| `void lv_baseimg_set_current_index(lv_obj_t *img, uint16_t index)` | Sets the current index | `index`: current index |
| `void lv_baseimg_set_img_type(lv_obj_t *img, uint16_t img_type)` | Sets the image type | `img_type`: image type (`lv_baseimg_type_t`) |
| `void lv_baseimg_set_empty_idx(lv_obj_t *img, uint16_t index)` | Sets the empty index | `index`: index of the empty image |
| `void lv_baseimg_set_pivot_X(lv_obj_t *img, int16_t x)` | Sets the rotation pivot X coordinate | `x`: rotation pivot X |
| `void lv_baseimg_set_pivot_y(lv_obj_t *img, int16_t y)` | Sets the rotation pivot Y coordinate | `y`: rotation pivot Y |
| `void lv_baseimg_set_loop(lv_obj_t *img, int16_t loop)` | Sets the loop count | `loop`: loop count; -1 means infinite loop |

### Query interfaces

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `int16_t lv_baseimg_get_angle(lv_obj_t *img)` | Gets the current angle | Returns the angle value |
| `uint16_t lv_baseimg_get_zoom(lv_obj_t *img)` | Gets the current zoom | Returns the zoom value |
| `uint16_t lv_baseimg_get_start_index(lv_obj_t *img)` | Gets the start index | Returns the start index |
| `uint16_t lv_baseimg_get_end_index(lv_obj_t *img)` | Gets the end index | Returns the end index |
| `uint16_t lv_baseimg_get_current_index(lv_obj_t *img)` | Gets the current index | Returns the current index |
| `uint16_t lv_baseimg_get_img_type(lv_obj_t *img)` | Gets the image type | Returns the image type (`lv_baseimg_type_t`) |
| `uint16_t lv_baseimg_get_empty_idx(lv_obj_t *img)` | Gets the empty index | Returns the empty image index |
| `int16_t lv_baseimg_get_pivot_x(lv_obj_t *img)` | Gets the rotation pivot X coordinate | Returns the rotation pivot X |
| `int16_t lv_baseimg_get_pivot_y(lv_obj_t *img)` | Gets the rotation pivot Y coordinate | Returns the rotation pivot Y |

## Typical Usage

Based on the header interfaces, the typical flow for continuous forward sequence-frame playback is: create the object → set the image type to sequence frame → set the source array → set the loop count → start playback:

```c
#include "lvsf_baseimg.h"

lv_obj_t *img = lv_baseimg_create(parent);

/* Sequence frames: continuous forward loop */
lv_baseimg_set_img_type(img, BASEIMG_TYPE_SEQUENCE);
lv_baseimg_set_src_array(img, frame_dsc_arr, 0, FRAME_CNT - 1);
lv_baseimg_set_loop(img, -1);                 /* Infinite loop */
lv_baseimg_set_state(img, BASEIMG_STATE_PLAY_START);   /* Start playback */
```

The typical flow for a pointer dial (stepless change) is:

```c
lv_baseimg_set_img_type(img, BASEIMG_TYPE_POINTER);
lv_baseimg_set_pivot_X(img, 60);
lv_baseimg_set_pivot_y(img, 60);
lv_baseimg_set_angle(img, angle);             /* Angle drives the pointer */
```

For detailed usage, refer to the header comments.

## Demo

This widget does not yet have a corresponding example. Run examples based on baseimg such as `lvgl_v8_imgarray` to see its derived widgets in action.

## See Also

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [LVGL v8 Image documentation](https://docs.lvgl.io/8.3/widgets/img.html)
- Source path: `middleware/lvgl/lvsf/gui_widgets/lvsf_baseimg.h`
