# LVGL v8 Gravity Follow Menu (Follow)

`lvsf_follow` is a physical "gravity" icon menu widget provided by SiFli. The icons are arranged in concentric rings and move and avoid each other under gravity, which is commonly used as an app launcher on smartwatches. On the board, a g-sensor provides the gravity direction; in the PC simulator the widget provides its own interaction path, converting the mouse click position into a gravity vector.

- Main header: _middleware/lvgl/lvsf/gui_widgets/lvsf_follow.h_
- Example project: _example/multimedia/lvgl/lvgl_v8_follow_

## Features

- Icons are arranged in concentric rings, supporting up to 3 layers (`MMENU_ICON_LAYER_CNT`).
- Physics animation based on gravity/friction/collision: icons move under gravity and avoid each other.
- Supports custom ring layout: the radius of each ring, icon radius, start angle, and gap angle are all configurable.
- Supports several state transitions: normal, order, change, and edit.
- Creates an icon object for each element through a callback and recycles it on destruction.
- Supports square/round screen adaptation (`is_square`, `square_r`).

## Use Cases

- App launcher (watch home screen) on smartwatches/bands.
- A gravity-driven draggable icon grid.
- Ring menus and home-screen icon arrangements that need physics animation.

## Supported Boards

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## Configuration and Initialization

In the BSP, the lvsf configuration controls whether this widget is compiled. After creating the object, first obtain the configuration struct with `lv_follow_get_cfg_param()` and set the physics parameters and ring layout one by one.

```c
#include "lvsf_follow.h"

lv_obj_t *follow = lv_follow_create(lv_scr_act());
lv_obj_set_size(follow, LV_PCT(100), 360);
```

## API Reference

### Constants

| Macro | Meaning |
| --- | --- |
| `FOLLOW_TYPE_DEFAULT` | Default collision type (0) |
| `FOLLOW_TYPE_SIMILAR` | Similar collision type (1) |
| `FOLLOW_TYPE_STANDARDS` | Standard collision type (2) |
| `FOLLOW_STATUS_NORMAL` | Normal state (`1U << 0`) |
| `FOLLOW_STATUS_ORDER` | Order state (`1U << 1`) |
| `FOLLOW_STATUS_CHANGE` | Change state (`1U << 2`) |
| `FOLLOW_STATUS_EDIT` | Edit state (`1U << 3`) |
| `MMENU_ICON_LAYER_CNT` | Number of icon layers (3) |

### Callback types

| Type definition | Description |
| --- | --- |
| `typedef lv_obj_t *(*lv_follow_create_item_cb)(lv_obj_t *parent, uint16_t index, uint16_t type, void *user_data)` | Creates and returns an icon object for each element |
| `typedef lv_obj_t *(*lv_follow_set_border_cb)(lv_obj_t *parent, uint16_t index, void *user_data)` | Sets the border for a given element |
| `typedef void(*lv_follow_delete_info_cb)(lv_follow_item_info_t *item)` | Element-info destruction callback |

### Configuration struct `lv_follow_cfg_t`

| Field | Type | Description |
| --- | --- | --- |
| `gravity` | `float` | Gravity acceleration |
| `friction` | `float` | Friction coefficient |
| `opa_r` | `lv_coord_t` | Opacity radius |
| `opa_min` | `lv_coord_t` | Minimum opacity |
| `v_max` | `uint16_t` | Maximum speed |
| `margin` | `uint8_t` | Margin |
| `collision_type` | `uint8_t` | Collision type (`FOLLOW_TYPE_*`) |
| `icon_r` | `int16_t` | Icon radius |
| `hor_rate` | `float` | Horizontal rate |
| `ver_rate` | `float` | Vertical rate |
| `speed_ratio` | `float` | Speed ratio |
| `black_ratio` | `float` | Black-screen ratio |
| `square_r` | `int16_t` | Square-screen corner radius |
| `target_r[MMENU_ICON_LAYER_CNT]` | `int16_t[]` | Icon radius of each layer |
| `offset_r[MMENU_ICON_LAYER_CNT]` | `int16_t[]` | Polar radius of each layer's image center (ring radius) |
| `start_angle[MMENU_ICON_LAYER_CNT]` | `int16_t[]` | Icon start angle of each layer |
| `gap_angle[MMENU_ICON_LAYER_CNT]` | `int16_t[]` | Icon gap angle of each layer (icons per ring = 360 / gap_angle) |
| `custom_align` | `bool` | Whether to use custom alignment; when `true`, the per-ring layout takes effect |
| `is_square` | `bool` | Whether the screen is square |

### Functions

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_obj_t *lv_follow_create(lv_obj_t *parent)` | Creates a follow menu object | `parent`: parent object; returns the menu object pointer |
| `void lv_follow_add_item_info(lv_obj_t *crashmenu, uint16_t type, void *user_data)` | Adds an element | `crashmenu`: menu object; `type`: element type; `user_data`: user data |
| `void lv_follow_set_gravity(lv_obj_t *crashmenu, float g, lv_coord_t angle)` | Sets the gravity direction and magnitude | `g`: gravity magnitude; `angle`: direction angle |
| `lv_follow_cfg_t *lv_follow_get_cfg_param(lv_obj_t *crashmenu)` | Gets the configuration struct pointer | Returns a directly modifiable configuration pointer |
| `void lv_follow_disable_status(lv_obj_t *crashmenu, uint8_t status)` | Disables a given state | `status`: state bit (`FOLLOW_STATUS_*`) |
| `uint8_t lv_follow_get_status(lv_obj_t *crashmenu)` | Gets the current state | Returns the state bits |
| `void lv_follow_set_item_cb(lv_obj_t *crashmenu, lv_follow_create_item_cb create_cb, lv_follow_delete_info_cb delete_cb)` | Sets the element creation and destruction callbacks | `create_cb`: creation callback; `delete_cb`: destruction callback |
| `void lv_follow_enter_order_status(lv_obj_t *crashmenu)` | Enters the order state (icons form a ring queue) | — |
| `void lv_follow_enter_normal_status(lv_obj_t *crashmenu)` | Enters the normal state | — |
| `void lv_follow_enter_change_status(lv_obj_t *crashmenu)` | Enters the change state | — |
| `void lv_follow_enter_edit_status(lv_obj_t *crashmenu, int8_t layer)` | Enters the edit state | `layer`: edit layer |
| `void lv_follow_on_start(lv_obj_t *parent)` | Starts the widget | — |
| `void lv_follow_on_resume(lv_obj_t *parent)` | Resumes the widget | — |
| `void lv_follow_on_pause(lv_obj_t *parent)` | Pauses the widget | — |
| `void lv_follow_on_stop(lv_obj_t *parent)` | Stops the widget | — |

## Typical Usage

```c
#include "lvsf_follow.h"

#define ICON_NUM 8

/* Item callback: returns an icon object for each element (here a solid-color circle; in a real app it should be a launcher icon) */
static lv_obj_t *follow_item_cb(lv_obj_t *parent, uint16_t index, uint16_t type, void *user_data)
{
    lv_obj_t *ic = lv_obj_create(parent);
    lv_obj_remove_style_all(ic);
    lv_obj_set_size(ic, 46, 46);
    lv_obj_set_style_radius(ic, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(ic, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_bg_opa(ic, LV_OPA_COVER, 0);
    return ic;
}

static void follow_delete_cb(lv_follow_item_info_t *item)
{
}

void demo_follow_init(void)
{
    lv_obj_t *follow = lv_follow_create(lv_scr_act());
    lv_obj_set_size(follow, LV_PCT(100), 360);

    lv_follow_cfg_t *cfg = lv_follow_get_cfg_param(follow);
    cfg->collision_type = FOLLOW_TYPE_STANDARDS;
    cfg->gravity = 0.01f;
    cfg->friction = 0.2f;
    cfg->icon_r = 23;
    cfg->v_max = 3;
    cfg->target_r[0] = 23; cfg->target_r[1] = 19; cfg->target_r[2] = 14;  /* Icon radius of each ring */
    cfg->offset_r[0] = 0;  cfg->offset_r[1] = 70; cfg->offset_r[2] = 125; /* Radius of each ring */
    cfg->start_angle[0] = 0; cfg->start_angle[1] = 0; cfg->start_angle[2] = 0;
    cfg->gap_angle[0] = 360; cfg->gap_angle[1] = 60; cfg->gap_angle[2] = 40; /* 360/gap = icons per ring */
    cfg->custom_align = true;   /* Required: use the per-ring layout above */
    cfg->is_square = false;

    lv_follow_set_item_cb(follow, follow_item_cb, follow_delete_cb);
    for (int i = 0; i < ICON_NUM; i++)
        lv_follow_add_item_info(follow, 0, NULL);   /* Add elements */
    lv_follow_on_start(follow);
    lv_follow_enter_order_status(follow);            /* Form a ring queue */
}
```

```{warning}
`cfg->custom_align` must be set to `true` for the per-ring layout (`offset_r`, `target_r`, `gap_angle`) to take effect; when set to `false`, the widget automatically computes the icon sizes from its own width.
```

## Demo

Run the `lvgl_v8_follow` example to see the result: eight colored dots form a gravity menu ring on the screen (one large center dot + a ring of 6 + one outer dot). Simulator interaction: tap the center icon to rearrange into the ring queue (gather); tap an outer icon and it is pulled toward the tap direction by "gravity"; long-press and drag to move a specific icon. On the board, gravity is driven by the g-sensor.

## See Also

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- Source path: `middleware/lvgl/lvsf/gui_widgets/lvsf_follow.h`
- Example path: `example/multimedia/lvgl/lvgl_v8_follow`
