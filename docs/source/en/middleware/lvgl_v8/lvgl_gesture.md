# LVGL v8 gesture

SiFli SDK provides a unified wrapper for the swipe-right-to-exit (page-back) gesture on LVGL v8, with the corresponding header `lvsf_gesture.h`. It offers a consistent trigger and follow-finger animation for switching between pages. An application only needs to enable / disable the gesture within the page lifecycle to achieve a consistent swipe-right-back interaction.

The swipe-right-to-exit animation has three forms, determined by the size of the trigger area of the follow-finger animation, which is set by the position of `line` (the trigger line):

- Swipe-right-to-exit from anywhere on the full screen: `line` is at the right edge of the screen;
- Swipe-right-to-exit within a limited area: `line` is close to the left edge of the screen but leaves a gap;
- Full-screen swipe-right-to-exit without the follow-finger animation: `line` is at the left edge of the screen.

## Features

- Uniformly wraps the swipe-right-to-exit gesture, ensuring a consistent trigger for page switching.
- Supports configuring the position of the right-swipe trigger area line and whether to return directly.
- Supports enabling / disabling the gesture animation; the state switch takes effect after entering idle.
- Supports realignment of the gesture bars and custom gesture images.

## Use Cases

- Swipe right on a secondary page to return to the previous page.
- When it conflicts with in-page gestures such as tiled page turning, temporarily disable the swipe-right-to-exit gesture in `on_resume` and restore it in `on_pause`.
- Scenarios that require restricting swipe-right-back to only the left screen-edge area.

## Supported Boards

Any platform supported by the general LVGL v8 examples, and any board from 55x onward (such as 58x, 56x, 52x).

## Configuration and Initialization

The gesture module is initialized uniformly by the GUI framework, so applications normally do not need to call `lvsf_gesture_init()` manually. Enable / disable it as needed within the page lifecycle:

```c
#include "lvsf/lvsf_gesture.h"
```

## API Reference

The interface signatures below are taken verbatim from `middleware/lvgl/lvsf/lvsf_gesture.h`:

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lvsf_gesture_init(lv_obj_t *parent)` | Initialize the gesture module | `parent`: the parent object to which the gesture widget is attached |
| `void lvsf_gesture_deinit(void)` | Deinitialize the gesture module | No return value |
| `void lvsf_gesture_set_image(uint32_t idx, const void *src_img)` | Set the gesture indicator image | `idx`: image index; `src_img`: image source |
| `void lvsf_gesture_disable(void)` | Disable the gesture; the state switch is executed after entering idle | No return value |
| `void lvsf_gesture_enable(void)` | Enable the gesture; the state switch is executed after entering idle | No return value |
| `void lvsf_gesture_bars_realign(void)` | Realign the gesture bars | No return value |

```{note}
The page-level gesture parameter configuration interfaces described in the solution document—such as `lv_gesture_init()` and `gui_app_gesture_set_parem(left_area, goback_en)`—belong to the GUI framework layer (not `lvsf_gesture.h`), and are used to set the right-swipe trigger area line position and whether to return directly.
```

## Typical Usage

In a tiled page that already has a swipe-right page-turn gesture, temporarily disable the swipe-right-to-exit gesture and restore it on leaving:

```c
static void on_resume(void)
{
    /* The page already has a swipe-right page-turn gesture, so disable swipe-right-to-exit */
    lvsf_gesture_disable();
}

static void on_pause(void)
{
    /* Restore the swipe-right-to-exit gesture when leaving the page */
    lvsf_gesture_enable();
}
```

## Demo

```{image} ../../../assets/lvgl_v8/gesture_illustration.png
:alt: gesture swipe-right-to-exit area illustration
```

```{image} ../../../assets/lvgl_v8/gesture.gif
:alt: gesture follow-finger animation demo
```

## See Also

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- Source path: `middleware/lvgl/lvsf/lvsf_gesture.h`
