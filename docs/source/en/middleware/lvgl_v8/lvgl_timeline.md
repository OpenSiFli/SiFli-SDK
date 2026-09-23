# LVGL v8 timeline

`lvsf_timeline` is SiFli's general-purpose animation orchestration engine. Each element maps a time interval `[start_time, end_time]` to a value interval `[start_value, end_value]`, and feeds the interpolated value in that interval to an exec callback; the callback can apply this value to **any property of any object** (position, size, opacity, angle, zoom, color, ...). Multiple elements share the same timeline, so multiple animation segments are orchestrated together by time and can run both serially and in parallel. It is used for complex transition/entrance animations.

- Main header: _middleware/lvgl/lvsf/gui_widgets/lvsf_timeline.h_
- Example project: _example/multimedia/lvgl/lvgl_v8_timeline_

## Features

- General-purpose animation orchestration: each element maps a time interval to a value interval and outputs the interpolated value through a callback.
- The exec callback applies the interpolated value to any property of any object, not limited to position translation.
- Multiple elements share the same timeline, supporting serial and parallel orchestration.
- Supports a ready callback (`ready_cb`) and user data.
- The timeline object itself is invisible (set its size to 0) and finishes and self-deletes when it reaches the total duration.

## Use Cases

- Complex entrance/transition animation orchestration (multiple properties changing simultaneously).
- Combined animations of translation, zoom, and opacity fading for interface elements.
- Animation sequences in which multiple segments play serially/parallelly along a timeline.

## Supported Boards

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## Configuration and Initialization

The BSP controls compilation of this widget through the `LVSF_USING_TIMELINE` macro. The timeline object itself is invisible; after creating it, set its size to 0:

```c
#include "lvsf_timeline.h"

lv_obj_t *tl = lv_timeline_create(lv_scr_act());
lv_obj_set_size(tl, 0, 0);
```

## API Reference

### Callback Types

| Type definition | Description | Parameters |
| --- | --- | --- |
| `typedef void (*lv_timeline_exec_xcb_t)(void *var, int32_t value, void *user_data)` | Animation exec callback | `var`: timeline node variable; `value`: current interpolated value; `user_data`: user data. It applies the value to the target object |
| `typedef void (*lv_timeline_ready_cb_t)(void *, void *)` | Animation ready callback | Called when the animation node completes |

### Functions

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_obj_t *lv_timeline_create(lv_obj_t *parent)` | Create a timeline object | `parent`: parent object (created on the screen when `NULL`); returns the new object pointer |
| `void lv_timeline_set_time(lv_obj_t *timeline, int32_t time)` | Set the total run duration | `timeline`: timeline object; `time`: total duration (ms) |
| `void lv_timeline_set_ready_cb(lv_obj_t *timeline, lv_timeline_ready_cb_t ready_cb, void *user_data)` | Set the ready callback | `ready_cb`: ready callback; `user_data`: user data |
| `void lv_timeline_add_element(lv_obj_t *timeline, int32_t start_value, int32_t end_value, int32_t start_time, int32_t end_time, lv_timeline_exec_xcb_t xcb, lv_timeline_ready_cb_t ready_cb, void *user_data)` | Add an animation element | `start_value`/`end_value`: start/end values; `start_time`/`end_time`: start/end times (ms); `xcb`: exec callback; `ready_cb`: ready callback; `user_data`: user data |
| `void lv_timeline_start(lv_obj_t *timeline)` | Start the timeline task | Begin playback |
| `void lv_timeline_pause(lv_obj_t *timeline)` | Pause the timeline task | Pause playback |
| `lv_obj_t *lv_timeline_get_var(lv_obj_t *timeline)` | Get the timeline variable | Returns the variable pointer |
| `void lv_timeline_set_var(lv_obj_t *timeline, void *var)` | Set the timeline variable | `var`: the variable to animate |

## Typical Usage

The example below makes a block: first move right (0~600ms), then scale up (600~1200ms), and finally scale down while moving back to the original position at the same time (1200~1900ms, two parallel segments):

```c
#include "lvsf_timeline.h"

/* Two exec callbacks, each applying the value to a different property (position, size) of the same object */
static void move_x(void *var, int32_t v, void *ud)
{
    (void)var;
    lv_obj_set_x((lv_obj_t *)ud, (lv_coord_t)v);
}
static void set_sz(void *var, int32_t v, void *ud)
{
    (void)var;
    lv_obj_set_size((lv_obj_t *)ud, (lv_coord_t)v, (lv_coord_t)v);
}

void play_animation(lv_obj_t *box)
{
    lv_obj_t *tl = lv_timeline_create(lv_scr_act());
    lv_obj_set_size(tl, 0, 0);
    /* (start value, end value, start time, end time, exec callback, ready callback, user_data) */
    lv_timeline_add_element(tl, 40, 200, 0, 600, move_x, NULL, box);     /* 0..600    move right */
    lv_timeline_add_element(tl, 40, 70, 600, 1200, set_sz, NULL, box);  /* 600..1200 scale up */
    lv_timeline_add_element(tl, 200, 40, 1200, 1900, move_x, NULL, box);/* 1200..1900 move back ┐ parallel */
    lv_timeline_add_element(tl, 70, 40, 1200, 1900, set_sz, NULL, box); /* 1200..1900 scale down ┘ */
    lv_timeline_set_time(tl, 1900);   /* total duration */
    lv_timeline_start(tl);            /* start playback */
}
```

```{note}
The timeline object finishes and self-deletes when it reaches the total duration set by `lv_timeline_set_time()`, so each playback should create a new timeline object rather than reusing an old one.
```

## Demo

Run the `lvgl_v8_timeline` example to see the actual effect: there is a red block on the screen with a `Run` button below it. Click `Run`, and the timeline plays for about 1.9 seconds—the block first translates (0~0.6s), then scales up (0.6~1.2s), and finally scales down while moving back to the original position at the same time (1.2~1.9s, two parallel animation segments). Clicking again resets first and then replays.

## See Also

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [LVGL v8 Animation Documentation](https://docs.lvgl.io/8.3/overview/animation.html)
- Source path: `middleware/lvgl/lvsf/gui_widgets/lvsf_timeline.h`
- Example path: `example/multimedia/lvgl/lvgl_v8_timeline`
