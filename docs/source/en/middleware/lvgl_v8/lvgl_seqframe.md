# LVGL v8 seqframe (seqimg)

SiFli SDK provides a picture frame-sequence playback widget on LVGL v8, with the corresponding SDK header `lvsf/lv_seqimg.h` (widget class name `lv_seqimg`). Built on top of the LVGL image object, it plays a group of pictures frame by frame in order. It supports two resource sources—memory-packed frame sequences (an `lv_img_dsc_t` array) and individual pictures on a file system (an array of file paths)—and supports play, pause, frame selection by index, and frame-interval setting.

```{note}
The `lv_seqframe_*` interfaces used in the solution document differ in naming from the `lv_seqimg_*` interfaces in the current SDK v8 header `lvsf/lv_seqimg.h`. The API table in this document follows the SDK header; the playback modes, prefix paths, and other feature descriptions related to `lv_seqframe_*` can serve as a design reference.
```

## Features

- Plays a group of pictures frame by frame in order, supporting both memory-array and file-path-array sources.
- Supports play / pause control and directly selecting a frame by index for display.
- Supports setting the frame playback interval.
- Resources can be frame sequences built into firmware, or individual pictures named by sequence number on a file system.

## Use Cases

- Looping frame sequences such as loading animations, WiFi-connecting animations, and charging animations.
- Simple animations that play extracted video frames in order.
- Scenarios that require frames to be stored by sequence number on a file system (such as an SD card) and loaded one by one.

## Supported Boards

Any platform supported by the general LVGL v8 examples, and any board from 55x onward (such as 58x, 56x, 52x).

## Configuration and Initialization

The frame-sequence widget is provided by `lvsf/lv_seqimg.h` and is enabled as part of the LVGL v8 build:

```c
#include "lvsf/lv_seqimg.h"
```

## API Reference

The interface signatures below are taken verbatim from `middleware/lvgl/lvsf/lv_seqimg.h`:

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_obj_t *lv_seqimg_create(lv_obj_t *parent)` | Create a frame-sequence widget instance | `parent`: parent object; returns the widget pointer |
| `void lv_seqimg_src_array(lv_obj_t *obj, const lv_img_dsc_t **dsc_array, uint16_t size)` | Bind an in-memory array of frame descriptors | `dsc_array`: array of `lv_img_dsc_t` pointers; `size`: number of frames |
| `void lv_seqimg_file_array(lv_obj_t *obj, const char **file_path_array, uint16_t size)` | Bind an array of image paths on the file system | `file_path_array`: array of image file paths; `size`: number of frames |
| `void lv_seqimg_select(lv_obj_t *obj, uint16_t index)` | Jump to and display the frame at the given index | `index`: frame index (starting from 0) |
| `void lv_seqimg_play(lv_obj_t *obj)` | Start playing the frame sequence | No return value |
| `void lv_seqimg_pause(lv_obj_t *obj)` | Pause playback | No return value |
| `void lv_seqimg_set_period(lv_obj_t *obj, uint32_t period)` | Set the frame playback interval | `period`: frame interval (ms) |

## Typical Usage

Play a frame sequence built into firmware:

```c
/* dsc_array is an array of lv_img_dsc_t pointers in playback order, count is the number of frames */
lv_obj_t *seqframe = lv_seqimg_create(lv_scr_act());
lv_seqimg_src_array(seqframe, dsc_array, count);
lv_seqimg_set_period(seqframe, 50);   /* frame interval 50ms */
lv_seqimg_select(seqframe, 0);        /* start from frame 0 */
lv_obj_align(seqframe, LV_ALIGN_CENTER, 0, 0);
lv_seqimg_play(seqframe);
```

Play individual pictures named by sequence number on the file system:

```c
const char *paths[] = {"/sd/photo/beauty0.bin",
                       "/sd/photo/beauty1.bin",
                       "/sd/photo/beauty2.bin"};

lv_obj_t *seq = lv_seqimg_create(lv_scr_act());
lv_seqimg_file_array(seq, paths, 3);
lv_seqimg_set_period(seq, 500);
lv_seqimg_select(seq, 0);
lv_seqimg_play(seq);
```

```{note}
When playing individually from the file system, external resource files must be named in playback order as "image name + consecutive numeric sequence + uniform suffix", so that the widget can load them correctly by index. The built-in array and the file array are mutually exclusive; do not configure both sources on the same widget.
```

## Demo

```{image} ../../../assets/lvgl_v8/seq_frame_demo_src.png
:alt: seqframe built-in frame-sequence resources
```

```{image} ../../../assets/lvgl_v8/seq_frame_demo.gif
:alt: seqframe built-in frame-sequence playback demo
```

```{image} ../../../assets/lvgl_v8/seq_frame_demo_prefix.png
:alt: seqframe external-directory frame-sequence resources
```

```{image} ../../../assets/lvgl_v8/prefix_seq_frame_demo.gif
:alt: seqframe external-directory frame-sequence playback demo
```

## See Also

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- Source path: `middleware/lvgl/lvsf/lv_seqimg.h`
