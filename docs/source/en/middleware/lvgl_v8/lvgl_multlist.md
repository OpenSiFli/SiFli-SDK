# LVGL v8 multlist

`lvsf_multlist` is a high-performance scrollable list container that SiFli wraps for LVGL v8. It organizes multiple homogeneous items row by row (vertical) or column by column (horizontal), and centrally manages scrolling, inertial fling, snapping, edge bounce, scaling / visual deformation, and item-node add, remove, and query operations. It addresses the recurring need at the application layer to hand-code "scrollable + auto-snap + edge bounce + dynamic item loading", and is commonly used in main menus, card streams, cover flows, chat message streams, and tiled (TLV) pages.

- Main header: _middleware/lvgl/lvsf/gui_widgets/lvsf_multlist.h_
- Implementation directory: _middleware/lvgl/lvsf/gui_widgets/_
- Example project: _example/multimedia/lvgl/lvgl_v8_multlist_

```{note}
multlist itself is only a "list container"; the visual content of each item is created and destroyed dynamically by the application through create / remove callbacks. Together with `lvsf_multswipe`, `lvsf_multedge`, and `lvsf_scrollbar`, it forms a plug-in system in which the widgets interact through events emitted by multlist. See each widget's documentation for details.
```

## Features

- A scrollable list in either vertical or horizontal direction, supporting inertial fling (throw), looping, and paged flipping.
- Automatic snapping after a scroll ends: supports none, head, center, and tail alignment, with configurable distance / velocity thresholds.
- Edge bounce: the maximum drag distance and the resting position after bounce can be configured separately for the head and tail.
- Dynamic items: item nodes are managed through a linked list; off-screen items are created / destroyed dynamically to save memory. Node removal with a deletion animation and reordering are supported.
- Visual deformation: the bezier scaling algorithm and the elliptical arc-offset algorithm can be enabled to produce a large center item with smaller flanking items, or an arc arrangement.
- Encoder (wheel) support: item movement can be driven by a rotary encoder.
- Plug-in integration: through flags and events it works together with swipe-to-delete (multswipe), side-edge (multedge), and scrollbar widgets.

## Use Cases

- Horizontal browsing of an app main menu / launcher icon list, card stream, or cover flow.
- Horizontal / vertical carousel and page flipping of full-screen images or pages.
- Chat / walkie-talkie style message streams (with item info added and removed dynamically).
- Focused browsing interfaces where items animate scale, opacity, or 3D-flip transitions with the scroll distance.
- List interfaces that need an external rotary encoder to switch options.

## Supported Boards

The reference example `example/multimedia/lvgl/lvgl_v8_multlist` is verified on the following boards:

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

The project can be adapted to different boards with `scons --board=<board>` and supports the SF32LB52x / SF32LB56x / SF32LB58x series.

## Configuration and Initialization

Enable the corresponding component in menuconfig (enabled by default):

```none
CONFIG_LVSF_USE_MULTLIST=y
```

The widget uses a set of internal flags to control feature switches, enabled or disabled through `lv_multlist_add_flag()` / `lv_multlist_clear_flag()`. Common flags are listed below:

| Flag | Meaning |
| --- | --- |
| `LV_MULTLIST_FLAG_THROW` | Enables inertial fling; enabled by default |
| `LV_MULTLIST_FLAG_LOOP` | Enables item looping mode |
| `LV_MULTLIST_FLAG_RESIDENCY` | When center-aligned, aligns the screen center to the item spacing (rather than the item center) |
| `LV_MULTLIST_FLAG_BEZIER_ALG` | Enables the bezier scaling algorithm; must be enabled after setting the bezier parameters |
| `LV_MULTLIST_FLAG_ELLIPSE_ALG` | Enables elliptical arc offset |
| `LV_MULTLIST_FLAG_EDGE` | Sends an edge event when the scroll focus animation ends, for interaction with multedge |
| `LV_MULTLIST_FLAG_SCROLLBAR` | Sends a scrollbar event while scrolling, for interaction with scrollbar |
| `LV_MULTLIST_FLAG_BOUNDARY` | At the end of the scroll animation, aligns the divider between two items to the center point |
| `LV_MULTLIST_FLAG_LOCK_SCRL` | Locks scrolling and disables swiping |
| `LV_MULTLIST_FLAG_SNAPSHOT` | Dynamic snapshot mode; off-screen items are deleted |
| `LV_MULTLIST_FLAG_SNAPSHOT_ALL` | Snapshots all items and keeps them without deletion |
| `LV_MULTLIST_FLAG_TOW_PAGE` / `LV_MULTLIST_FLAG_THREE_PAGE` | Two-page / three-page refresh mode |
| `LV_MULTLIST_FLAG_INFINTE` | Infinite node mode |
| `LV_MULTLIST_FLAG_SHOW_ALL` | Alignment when the total element length is shorter than the multlist (used in infinite mode) |
| `LV_MULTLIST_FLAG_ALIGN_HEAD` | Defaults to head alignment when elements do not fill one screen |

### Initialization Sequence

A typical sequence of creation, configuration, callback registration, node addition, and initial alignment is shown below (extracted from the example `demo_multlist_list.c`):

```c
#include "lvsf_multlist.h"

lv_obj_t *list = lv_multlist_create(lv_scr_act());
lv_obj_remove_style_all(list);
lv_obj_set_size(list, LV_HOR_RES_MAX, LV_VER_RES_MAX);
lv_obj_set_style_bg_color(list, lv_color_make(82, 93, 118), 0);
lv_obj_set_style_bg_opa(list, LV_OPA_COVER, 0);
lv_obj_center(list);

/* Bezier deformation + gap + drag bounds + direction */
float para[] = { 0, 0, 0, 0.1f, 0.3f };
lv_multlist_set_bezier_para(list, LV_VER_RES_MAX, para, para);
lv_multlist_set_gap(list, 20);
lv_multlist_set_scrl_pad(list, LV_VER_RES_MAX >> 1, LV_VER_RES_MAX >> 1);
lv_multlist_set_dir(list, LV_MULTLIST_DIR_VER);

/* Register the item lifecycle callbacks, then add node info */
lv_multlist_set_item_cb(list, demo_create_item, NULL, NULL);
for (uint32_t i = 0; i < 100; i++)
{
    lv_multlist_add_info(list, LV_HOR_RES, 110, NULL, NULL);
}

/* Spring-back position and initial alignment */
lv_multlist_set_springback(list, 0, 0);
lv_multlist_align_to(list, LV_MULTLIST_ALIGN_HEAD, 0, 0, 0);
```

On page resume / pause, encoder and lifecycle calls are typically used together:

```c
/* resume */
lv_multlist_on_resume(list);
lv_multlist_enable_encoder(list, 5, 400, false);

/* pause */
lv_multlist_on_pause(list);
lv_multlist_disable_encoder(list);
```

```{warning}
`lv_multlist_add_info()` must be passed an accurate estimate of the item width and height. A width/height error directly shifts alignment positions and makes scrollbar progress inaccurate. When aligning again after clearing all nodes, it is recommended to first reset the position to a valid pos, so that a stale out-of-range pos does not cause items to be displayed incompletely.
```

## API Reference

The function signatures below are taken verbatim from _lvsf_multlist.h_.

### Base Objects and Flags

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_obj_t *lv_multlist_create(lv_obj_t *parent)` | Creates a multlist list object | `parent`: parent object; returns the pointer to the new list object |
| `lv_dir_t lv_multlist_get_gesture(lv_obj_t *multlist)` | Gets the touch gesture direction | Returns the current gesture direction enum |
| `int lv_multlist_has_flag(lv_obj_t *multlist, uint32_t flag)` | Queries whether a given flag is set | Returns non-0 if set, 0 otherwise |
| `void lv_multlist_add_flag(lv_obj_t *multlist, uint32_t flag)` | Adds a feature flag (enables the corresponding feature) | `flag`: the flag bit to enable |
| `void lv_multlist_clear_flag(lv_obj_t *multlist, uint32_t flag)` | Clears a flag (disables the corresponding feature) | `flag`: the flag bit to clear |
| `void lv_multlist_refresh(lv_obj_t *multlist)` | Refreshes all items based on the current scroll position | Call after the scroll position / properties change |
| `int lv_multlist_is_item_full(lv_obj_t *multlist)` | Checks whether the total length of all items exceeds the multlist height | Returns true (1) if exceeded, false (0) otherwise |

### Scrolling and Position

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_multlist_set_pos(lv_obj_t *multlist, int32_t position)` | Sets the scroll position directly (no animated jump) | `position`: target scroll position (pixels) |
| `int32_t lv_multlist_get_pos(lv_obj_t *multlist)` | Gets the current scroll position | Returns the current scroll position (pixels) |
| `void lv_multlist_set_snapshot(lv_obj_t *multlist, lv_event_cb_t cb, lv_img_cf_t cf)` | Configures the dynamic item snapshot callback and image format | `cb`: snapshot event callback; `cf`: image color format |
| `void lv_multlist_set_scrl_pad(lv_obj_t *multlist, lv_coord_t head, lv_coord_t tail)` | Sets the maximum draggable offset at the head and tail | `head` / `tail`: maximum head / tail drag distance |
| `void lv_multlist_set_show_pad(lv_obj_t *multlist, uint16_t head, uint16_t tail)` | Sets an extra display area beyond which items are removed | Defaults to 0; controls the item destruction threshold |
| `void lv_multlist_set_gap(lv_obj_t *multlist, uint16_t gap)` | Sets the spacing between adjacent items | `gap`: item spacing (pixels) |
| `void lv_multlist_set_dir(lv_obj_t *multlist, lv_multlist_dir_t dir)` | Sets the scroll direction | `dir`: `LV_MULTLIST_DIR_VER` / `LV_MULTLIST_DIR_HOR` |
| `lv_multlist_dir_t lv_multlist_get_dir(lv_obj_t *multlist)` | Gets the scroll direction | Returns the current direction enum |
| `void lv_multlist_set_springback(lv_obj_t *multlist, lv_coord_t edge_head, int16_t edge_tail)` | Sets the bounce-back resting position after an overscroll | `edge_head` / `edge_tail`: spacing from the edge after head / tail bounce |
| `int32_t lv_multlist_get_springback_head(lv_obj_t *multlist)` | Computes the head bounce region position | Returns the head bounce position (pixels) |
| `int32_t lv_multlist_get_springback_tail(lv_obj_t *multlist)` | Computes the tail bounce region position | Returns the tail bounce position (pixels) |

### Alignment Control

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_multlist_set_align(lv_obj_t *multlist, lv_multlist_align_t align, lv_coord_t offset)` | Sets the default alignment after a scroll ends | `align`: `LV_MULTLIST_ALIGN_NONE/CENTER/HEAD/TAIL`; `offset`: alignment-point offset |
| `void lv_multlist_set_first_align(lv_obj_t *multlist, lv_multlist_align_t align, lv_coord_t offset, int16_t index)` | Sets the alignment on first use (on resume) | `index`: index of the initially aligned item |
| `int32_t lv_multlist_get_focus_pos(lv_obj_t *multlist, lv_multlist_align_t align, lv_multlist_item_t *item)` | Computes the scroll value needed to align a given item to the target position | Returns the required scroll position (pixels) |
| `void lv_multlist_focus_near(lv_obj_t *multlist, int32_t offset, uint8_t dir_en, uint32_t time)` | Scrolls by a relative offset and snaps to the nearest item | `offset`: relative offset; `dir_en`: true aligns in the offset direction, false aligns to the nearest; `time`: animation duration, 0 disables animation |
| `void lv_multlist_align_head_to(lv_obj_t *multlist, int16_t index, lv_coord_t edge_offset)` | Aligns the item at the given index to the head | `edge_offset`: head-edge offset |
| `void lv_multlist_align_tail_to(lv_obj_t *multlist, int16_t index, lv_coord_t edge_offset)` | Aligns the item at the given index to the tail | `edge_offset`: tail-edge offset |
| `void lv_multlist_align_center_to(lv_obj_t *multlist, int16_t index, lv_coord_t edge_offset)` | Aligns the item at the given index to the center | `edge_offset`: center offset |
| `void lv_multlist_align_to(lv_obj_t *multlist, lv_multlist_align_t align, int16_t index, lv_coord_t offset, uint32_t time)` | Animates the given item into alignment at the target position | `time`: animation duration |
| `void lv_multlist_set_focus_threshold(lv_obj_t *multlist, uint16_t dis, uint16_t vect)` | Sets the distance / velocity threshold for paged snapping | Exceeding the distance or velocity threshold triggers alignment to the next page |

### Item Node Management

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_multlist_item_t *lv_multlist_add_info(lv_obj_t *multlist, lv_coord_t w, lv_coord_t h, void *info, void *user_data)` | Adds node info to the linked list; items are dynamically loaded while scrolling | `w` / `h`: item width and height; `info` / `user_data`: user data; returns the item handle |
| `void lv_multlist_updata_element(lv_obj_t *multlist, lv_multlist_item_t *item, bool delete)` | Force-refreshes an element when item properties change | `delete`: whether to delete the old element before rebuilding |
| `uint8_t lv_multlist_insert_info(lv_obj_t *multlist, lv_multlist_item_t *item, lv_multlist_item_t *ref)` | Inserts an item before the reference item (no animation) | Returns 1 on success, 0 on failure |
| `uint32_t lv_multlist_item_remove(lv_obj_t *multlist, lv_multlist_item_t *delete_item, lv_multlist_anim_type_t type, uint8_t free)` | Removes an item with a deletion animation | `type`: animation type `LV_MULTLIST_ANIM_NONE/DEL/FLY/SALCE/ZOOM`; `free`: whether to free memory after the animation; returns 1 on success |
| `uint32_t lv_multlist_item_move_before(lv_obj_t *multlist, lv_multlist_item_t *insert_item, lv_multlist_item_t *item_ref, uint8_t en_anim)` | Moves an item to before the reference item | `en_anim`: 1 enables animation; returns 1 on success |
| `void lv_multlist_remove_info_all(lv_obj_t *multlist)` | Removes all node info added by add_info | Often used to rebuild nodes after pause |
| `void lv_multlist_load_all_item(lv_obj_t *multlist)` | Loads all items based on the added node info | — |
| `uint32_t lv_multlist_get_info_cnt(lv_obj_t *multlist)` | Gets the number of node info entries | Returns the node count |
| `lv_multlist_item_t *lv_multlist_get_center_item(lv_obj_t *multlist)` | Gets the currently centered item | Returns the centered item handle |
| `lv_multlist_item_t *lv_multlist_get_focus_item(lv_obj_t *multlist, lv_coord_t offset)` | Gets the item at the current alignment point | `offset`: alignment-point offset; returns the aligned item handle |
| `lv_multlist_item_t *lv_multlist_get_item_by_index(lv_obj_t *multlist, int16_t index)` | Gets an item handle by index | Returns the corresponding item handle |

### Deformation and Effects

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_multlist_set_bezier_para(lv_obj_t *multlist, uint32_t range, float *pos_arr, float *neg_arr)` | Sets the bezier parameters for item scaling | `range`: farthest valid offset of the alignment point; `pos_arr` / `neg_arr`: five increasing scaling factors for the positive / negative directions |
| `void lv_multlist_set_ellipse_para(lv_obj_t *multlist, lv_coord_t x_axis, lv_coord_t y_axis)` | Sets the item arc (ellipse) offset | `x_axis` / `y_axis`: ellipse X / Y axis distances |
| `void lv_multlist_set_angles(lv_obj_t *multlist, int16_t offset_angle, int16_t start_angle, int16_t end_angle)` | Sets angles in circular list mode | `offset_angle`: overall rotation angle; `start_angle` / `end_angle`: display start/end angles |
| `void lv_multlist_set_radius(lv_obj_t *multlist, uint16_t r, uint16_t virt_r)` | Sets the circular list radius | `r`: layout radius; `virt_r`: virtual radius derived from the drag distance |
| `void lv_multlist_set_overlap_cnt(lv_obj_t *multlist, uint16_t overlap_cnt)` | Sets the number of stacked (overlapping) items | Specific to stacked menus |
| `void lv_multlist_set_pivot_offset(lv_obj_t *multlist, lv_coord_t offset_x, lv_coord_t offset_y)` | Sets the offset of the item transform center from the center | Pivot offset used for scaling / rotation |

### Animation and Callbacks

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_multlist_set_anim_para(lv_obj_t *multlist, lv_coord_t rate, lv_coord_t min_time, lv_coord_t max_time)` | Sets the basic animation parameters | `rate`: time per pixel; `min_time` / `max_time`: minimum / maximum animation duration |
| `void lv_multlist_fly_anim(lv_obj_t *multlist, lv_multlist_fly_type_t type, uint16_t delay, uint32_t time, uint8_t fly_in, lv_anim_ready_cb_t ready_cb)` | Sets an item fly-in / fly-out animation | `type`: direction (top-left / top-right / middle-left / middle-right / bottom-left / bottom-right); `delay`: delay; `time`: duration; `fly_in`: 1 fly-in / 0 fly-out |
| `lv_anim_t *lv_multlist_anim(void *multlist, int32_t start, int32_t end, uint32_t pred, lv_anim_exec_xcb_t exe_cb, lv_anim_ready_cb_t ready_cb, lv_anim_path_cb_t path_cb)` | Creates a custom list animation | Returns the animation handle |
| `void lv_multlist_set_refresh_cb(lv_obj_t *multlist, lv_multlist_refresh_cb refresh_cb)` | Sets a custom refresh callback | Used to replace the default refresh logic |
| `void lv_multlist_set_page_anim_cb(lv_obj_t *multlist, lv_multlist_page_cb anim_cb)` | Sets the paged-mode animation callback | Triggered during multi-page refresh |
| `void lv_multlist_set_tranform_cb(lv_obj_t *multlist, lv_multlist_tranform_cb tranform_cb)` | Sets the item transform callback | Custom scaling / opacity / layering and other deformation |
| `void lv_multlist_set_item_cb(lv_obj_t *multlist, lv_multlist_create_item_cb create_cb, lv_multlist_remove_item_cb remove_cb, lv_multlist_delete_info_cb delete_cb)` | Registers item lifecycle callbacks | `create_cb`: creates the element; `remove_cb`: called before deleting the element (do not delete the element inside it; used for resource cleanup); `delete_cb`: deletes the info data (the element has already been freed at this point; do not access the element) |

### Encoder and Lifecycle

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_multlist_enable_encoder(lv_obj_t *multlist, uint32_t ratio, uint32_t vect_max, uint8_t reverse)` | Enables encoder (wheel) control of item movement | `ratio`: speed increment per detent; `vect_max`: maximum speed; `reverse`: whether to reverse the direction |
| `void lv_multlist_disable_encoder(lv_obj_t *multlist)` | Disables the encoder | — |
| `void lv_multlist_on_pause(lv_obj_t *multlist)` | Pauses the list: removes all displayed items and stops refresh | Called on page pause |
| `void lv_multlist_on_resume(lv_obj_t *multlist)` | Resumes the list: reloads items and restarts refresh | Called on page resume |

### Events

multlist sends custom events to the object at specific states; the application listens through `lv_obj_add_event_cb()`:

| Event | Meaning |
| --- | --- |
| `LV_EVENT_LIST_SCROLL_STRAT` / `LV_EVENT_LIST_SCROLLING` / `LV_EVENT_LIST_SCROLL_END` | Scroll started / scrolling / scroll ended |
| `LV_EVENT_LIST_FOCUS` | Sent when an item snaps to the target position |
| `LV_EVENT_LIST_FOCUS_LOSS` | Sent when an item leaves the aligned position |
| `LV_EVENT_LIST_SCROLLBAR` | Notifies the scrollbar to update its progress |
| `LV_EVENT_EDGE_DRAGE_*` | Edge-drag request / start / dragging / end events for multedge interaction |
| `LV_EVENT_SWIPE_DELETE` | Sent when an item is swipe-deleted |

## Typical Usage

The complete runnable example is in `example/multimedia/lvgl/lvgl_v8_multlist` and includes sub-pages for a normal list, paged flipping, custom animation, and a chat message stream. The minimal call sequence is:

```c
/* 1. Create and configure */
lv_obj_t *list = lv_multlist_create(lv_scr_act());
lv_multlist_set_dir(list, LV_MULTLIST_DIR_VER);
lv_multlist_set_gap(list, 20);
lv_multlist_set_scrl_pad(list, LV_VER_RES_MAX >> 1, LV_VER_RES_MAX >> 1);

/* 2. Register the item create callback (actually build each item's widget tree inside it) */
lv_multlist_set_item_cb(list, my_create_item, NULL, NULL);

/* 3. Add node info (the item's real widgets are created by the callback only when it slides onto the screen) */
lv_multlist_add_info(list, LV_HOR_RES, 110, user_info, NULL);

/* 4. Set alignment and spring-back */
lv_multlist_set_align(list, LV_MULTLIST_ALIGN_CENTER, 0);
lv_multlist_set_springback(list, 0, 0);
lv_multlist_align_center_to(list, 0, 0);

/* 5. Enable the encoder on resume; stop it on pause */
lv_multlist_on_resume(list);
lv_multlist_enable_encoder(list, 5, 400, false);
```

Example item create callback (extracted from the example):

```c
static lv_obj_t *demo_create_item(lv_obj_t *parent, lv_multlist_item_t *item)
{
    lv_obj_t *item_bg = lv_obj_create(parent);
    lv_obj_remove_style_all(item_bg);
    lv_obj_set_size(item_bg, item->org_w, item->org_h);
    lv_obj_set_style_bg_color(item_bg, ITEM_BG_COLOR, LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(item_bg, LV_OPA_100, LV_STATE_DEFAULT);
    lv_obj_add_flag(item_bg, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_clear_flag(item_bg, LV_OBJ_FLAG_SCROLLABLE);
    /* ... keep placing images, text, and other content on item_bg ... */
    return item_bg;
}
```

## Demo

Standard vertical list scrolling with center snapping:

```{image} ../../../assets/lvgl_v8/multlist_show.gif
:alt: multlist demo
```

Bezier scaling deformation enabled alone (left: bezier only; right: bezier + ellipse offset):

```{image} ../../../assets/lvgl_v8/multlist_bezier.png
:alt: multlist bezier deformation
:width: 800px
:align: center
```

Head bounce region (head) and tail bounce region (tail):

```{image} ../../../assets/lvgl_v8/multlist_head.png
:alt: multlist head bounce
:width: 800px
:align: center
```

```{image} ../../../assets/lvgl_v8/multlist_tail.png
:alt: multlist tail bounce
:width: 800px
:align: center
```

## See Also

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- Header: _middleware/lvgl/lvsf/gui_widgets/lvsf_multlist.h_
- Example project: _example/multimedia/lvgl/lvgl_v8_multlist_
