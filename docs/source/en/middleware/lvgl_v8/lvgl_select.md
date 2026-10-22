# LVGL v8 Selection List (Select)

`lvsf_select` is SiFli's custom selection list widget. Each row shows a "selected" icon when selected, and an "unselected" icon otherwise. In single-select mode only one row is selected at a time (radio style), while in multi-select mode several rows can be selected at once (checkbox style). Tapping a row moves the selected state to that row, and the application then reads back the user's choice.

- Main header: _middleware/lvgl/lvsf/gui_widgets/lvsf_select.h_
- Dependent widgets: LVGL button / label / img (`LV_USE_BTN`, `LV_USE_LABEL`, `LV_USE_IMG`)
- Example project: _example/multimedia/lvgl/lvgl_v8_select_

## Features

- Supports two modes: single select (`LV_SELECT_TYPE_SINGLE`) and multi select (`LV_SELECT_TYPE_MULTI`).
- Each row element maintains its own state independently: unselected, selected, and disabled.
- Configurable selected/unselected icon sources, element count, and element size.
- Row clicks bubble up to the select, and the built-in click callback moves the single-select state.
- Supports reading back the current selected index, or querying the selected state row by row.

## Use Cases

- Single/multi-select lists in a settings menu (such as options A/B/C).
- Radio-button and checkbox groups.
- Item lists that need custom selected/unselected icons.

## Supported Boards

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## Configuration and Initialization

`lvsf_select` depends on the LVGL button, label, and img components. In menuconfig, confirm they are enabled:

```none
CONFIG_LV_USE_BTN=y
CONFIG_LV_USE_LABEL=y
CONFIG_LV_USE_IMG=y
```

In the BSP, the `LVSF_USE_SELECT` macro controls whether this widget is compiled.

Minimum flow to create an object:

```c
#include "lvsf_select.h"

lv_obj_t *sel = lv_select_create(lv_scr_act());
```

## API Reference

### Enumerations and constants

| Type | Value | Description |
| --- | --- | --- |
| `lv_select_type_t` | `LV_SELECT_TYPE_SINGLE` | Single-select mode |
| | `LV_SELECT_TYPE_MULTI` | Multi-select mode |
| `lv_select_state_t` | `LV_SELECT_STATE_UNCHECK` | Unselected state |
| | `LV_SELECT_STATE_CHECK` | Selected state |
| | `LV_SELECT_STATE_DISABLE` | Disabled state |
| Macro | `LV_SELECT_ELE_INTERVAL` | Element spacing (4) |

### Functions

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_obj_t *lv_select_create(lv_obj_t *par)` | Creates a select widget | `par`: parent object; returns the select widget object pointer |
| `void lv_select_set_type(lv_obj_t *select, lv_select_type_t type)` | Sets the selection type | `type`: single/multi select |
| `void lv_select_set_ele_num(lv_obj_t *select, uint16_t num)` | Sets the element count | `num`: number of rows |
| `void lv_select_set_ele_size(lv_obj_t *select, lv_coord_t w, lv_coord_t h)` | Sets the element size | `w`/`h`: element width/height; rows are laid out from these |
| `void lv_select_set_ele_state(lv_obj_t *select, uint16_t idx, lv_select_state_t state)` | Sets the element state | `idx`: element index; `state`: element state |
| `void lv_select_set_check_src(lv_obj_t *select, const void *src)` | Sets the selected icon source | `src`: selected icon source |
| `void lv_select_set_uncheck_src(lv_obj_t *select, const void *src)` | Sets the unselected icon source | `src`: unselected icon source |
| `uint16_t lv_select_get_select_idx(lv_obj_t *select)` | Gets the current selected index | In single-select mode, returns the currently selected row |
| `lv_obj_t *lv_select_get_ele(lv_obj_t *select, uint16_t idx)` | Gets the element object at the given index | `idx`: element index; returns the element object pointer |
| `uint16_t lv_select_get_ele_idx(lv_obj_t *select, lv_obj_t *ele)` | Gets the index of an element | `ele`: element object pointer; returns the element index |
| `uint16_t lv_select_get_ele_num(lv_obj_t *select)` | Gets the element count | Returns the element count |
| `lv_select_state_t lv_select_get_ele_state(lv_obj_t *select, uint16_t idx)` | Gets the state of the given element | `idx`: element index; returns the element state |

## Typical Usage

```c
#include "lvsf_select.h"

lv_obj_t *sel = lv_select_create(parent);
lv_obj_add_flag(sel, LV_OBJ_FLAG_CLICKABLE);        /* The constructor clears it; clicking needs it */
lv_select_set_type(sel, LV_SELECT_TYPE_SINGLE);     /* Single select */
lv_select_set_ele_num(sel, 3);                      /* Number of rows */
lv_select_set_ele_size(sel, 220, 40);               /* Per-row size */
lv_select_set_check_src(sel, &check_img_dsc);       /* Selected icon */
lv_select_set_uncheck_src(sel, &uncheck_img_dsc);    /* Unselected icon */
lv_select_set_ele_state(sel, 0, LV_SELECT_STATE_CHECK);  /* Row 0 selected initially */

/* Read back the user's choice (row clicks bubble up to the select): */
uint16_t idx = lv_select_get_select_idx(sel);       /* Single select: currently selected row */
/* In multi select, query row by row: lv_select_get_ele_state(sel, i) == LV_SELECT_STATE_CHECK */
```

Attach an `LV_EVENT_SHORT_CLICKED` callback to the select to read back the user's latest choice:

```c
static void sel_changed_cb(lv_event_t *e)
{
    lv_obj_t *sel = (lv_obj_t *)lv_event_get_user_data(e);
    uint16_t idx = lv_select_get_select_idx(sel);
    /* Update the UI based on idx */
}

lv_obj_add_event_cb(sel, sel_changed_cb, LV_EVENT_SHORT_CLICKED, sel);
```

```{warning}
`lv_select_create()` clears its own `LV_OBJ_FLAG_CLICKABLE` during construction. You must re-add it with `lv_obj_add_flag(sel, LV_OBJ_FLAG_CLICKABLE)` before clicks can reach the rows.
```

## Demo

Run the `lvgl_v8_select` example to see the result: three rows (Option A/B/C) are shown in the center of the screen, with a small icon on the right of each row. The first row is green (selected) and the other two are dark (unselected); a row below reads `Selected: Option A` to show the current choice. Tap other rows and the green "selected" icon moves to the tapped row, and the `Selected:` text updates in real time.

## See Also

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- Source path: `middleware/lvgl/lvsf/gui_widgets/lvsf_select.h`
- Example path: `example/multimedia/lvgl/lvgl_v8_select`
