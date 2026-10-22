# LVGL v8 scrollbar

`lvsf_scrollbar` is the scrollbar (progress bar) plugin that SiFli provides to accompany `lvsf_multlist`. When multlist is scrolled, it sends the `LV_EVENT_LIST_SCROLLBAR` event, which carries the current page progress information. Simply create a scrollbar widget and attach it to multlist; it automatically listens to this event and displays the list's scrolling progress. The progress bar supports two visual styles: circular and long (square).

- Main header: _middleware/lvgl/lvsf/gui_widgets/lvsf_scrollbar.h_
- Dependency: `lvsf_multlist`

## Features

- Automatically shows the ratio between the current scroll position and the total content length when multlist is scrolled.
- Supports both long (square) and circular styles.
- The on-screen hold duration and the disappearance animation duration are configurable.
- Can be manually hidden / shown, or set to always visible.

## Use Cases

- Scrolling progress indication for long lists such as the main menu and card streams.
- Interfaces that need a thin progress bar or circular progress indicator along the screen edge.

## Supported Boards

scrollbar has no standalone SDK example; it is used as a plugin of multlist, so its runtime environment matches the `lvgl_v8_multlist` example:

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## Configuration and Initialization

Enable the corresponding component in menuconfig (enabled by default):

```none
CONFIG_LVSF_USE_SCROLLBAR=y
```

After multlist has been initialized, create the scrollbar with multlist as its parent. multlist must first have the `LV_MULTLIST_FLAG_SCROLLBAR` flag enabled before it sends progress events while scrolling:

```c
#include "lvsf_scrollbar.h"

/* Enable scrollbar events on multlist */
lv_multlist_add_flag(multlist, LV_MULTLIST_FLAG_SCROLLBAR);

/* Create a long-style scrollbar, parent is multlist */
lv_obj_t *bar = lv_scrollbar_create(multlist, LV_SCROLLBAR_SQUARE_TYPE);

/* Optional: configure the hold and disappearance animation durations (ms) */
lv_scrollbar_set_anim_time(multlist, 1500, 300);
```

```{note}
Once multlist is passed as the parent of `lv_scrollbar_create()`, the scrollbar automatically handles the `LV_EVENT_LIST_SCROLLBAR` events sent by multlist, so the application normally does not need to call `lv_scrollbar_update()` manually.
```

## API Reference

The function signatures below are taken verbatim from _lvsf_scrollbar.h_.

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_obj_t *lv_scrollbar_create(lv_obj_t *parent, uint16_t type)` | Create a scrollbar object | `parent`: parent object (usually multlist); `type`: `LV_SCROLLBAR_SQUARE_TYPE` (long) / `LV_SCROLLBAR_CIRCLE_TYPE` (circular); returns the scrollbar pointer |
| `void lv_scrollbar_update(lv_obj_t *scrollbar, lv_coord_t cur_pos, uint16_t ind_len, uint16_t tatol_len)` | Update the scrollbar position and display ratio | `cur_pos`: current scroll position; `ind_len`: scroll indicator length; `tatol_len`: total content length |
| `void lv_scrollbar_set_anim_time(lv_obj_t *parent, uint32_t hold_time, uint32_t disappear_time)` | Set the on-screen hold and disappearance animation durations | `hold_time`: hold duration (ms); `disappear_time`: disappearance animation duration (ms) |
| `void lv_scrollbar_set_hidden(lv_obj_t *scrollbar, bool is_hidden)` | Manually hide / show the scrollbar | `is_hidden`: true to hide, false to show |
| `void lv_scrollbar_set_always_visible(lv_obj_t *scrollbar, bool is_always_visible)` | Set whether to always visible | `is_always_visible`: true to always show, false to auto-hide after scrolling |

## Typical Usage

Simply create the scrollbar at the end of the multlist initialization flow. Complete example (excerpted from main menu initialization):

```c
lv_obj_t *multlist = lv_multlist_create(lv_scr_act());
/* ... configure multlist size, bezier, spacing, direction, item callback, nodes, alignment ... */

lv_multlist_add_flag(multlist, LV_MULTLIST_FLAG_SCROLLBAR);
lv_scrollbar_create(multlist, LV_SCROLLBAR_SQUARE_TYPE);   /* long progress bar */
```

For the circular style, change the type to `LV_SCROLLBAR_CIRCLE_TYPE`:

```c
lv_scrollbar_create(multlist, LV_SCROLLBAR_CIRCLE_TYPE);
```

## Demo

The progress bar updates in real time with the scroll position as the list scrolls:

```{image} ../../../assets/lvgl_v8/multlist_scroll_demo.gif
:alt: scrollbar demo
:width: 400px
:align: center
```

## See Also

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- Header: _middleware/lvgl/lvsf/gui_widgets/lvsf_scrollbar.h_
- Related widget: `lvsf_multlist` (list container)
