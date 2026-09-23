# LVGL v8 Baselabel

`lvsf_baselabel` is a custom label widget that SiFli wraps around LVGL v8's native `lv_label`. On top of an ordinary text label it builds in a **data-driven refresh path**: after binding a data-source ID, registering a data callback, and creating a refresh timer, the widget periodically pulls data and updates its own text, so the application layer does not need to poll on its own. It also keeps the regular text capabilities of `lv_label`.

- Main header: _middleware/lvgl/lvsf/gui_widgets/lvsf_baselabel.h_
- Data refresh path interfaces: _middleware/lvgl/lvsf/gui_widgets/lvsf_obj_ext.h_
- Dependent widget: LVGL label (`LV_USE_LABEL`)
- Example project: _example/multimedia/lvgl/lvgl_v8_baselabel_

## Features

- Inherits from `lv_label_t` and supports all regular label capabilities, such as `set_text`, `set_text_fmt`, long-text modes, recolor, and text selection and editing.
- Built-in data refresh path: bind a data-source ID + register a data callback + create a refresh timer to periodically pull data and automatically refresh the text.
- Supports custom text (dynamically allocated and copied) and static text (only the pointer is stored, suitable for constant strings).
- Supports a static string table (`sfat_str`): add, clear, and switch the displayed string by index, suitable for enumerated strings such as weekday/month.
- Supports text truncation with ellipsis (`...` is appended automatically based on the display width).
- Supports formatted text setting (`lv_baselabel_set_text_fmt`).

## Use Cases

- Real-time numeric readouts, such as runtime duration, heart rate, battery level, and sensor readings that change over time.
- Numbers that auto-refresh with a data source on a status bar or watch dial.
- Labels that must switch among several predefined strings (weekday, month, status enumerations).
- Long text truncated to a fixed width with an ellipsis.

## Supported Boards

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## Configuration and Initialization

`lvsf_baselabel` depends on the LVGL label component. In menuconfig, confirm it is enabled:

```none
CONFIG_LV_USE_LABEL=y
```

In the BSP, the `LVSF_USE_BASELABEL` macro controls whether this widget is compiled.

Minimum flow to create an object:

```c
#include "lvsf_baselabel.h"

lv_obj_t *label = lv_baselabel_create(lv_scr_act());
lv_baselabel_set_text(label, "hello");
```

## API Reference

### Callback types

| Type definition | Description | Parameters / Returns |
| --- | --- | --- |
| `typedef int32_t (*lv_baselabel_refresh_cb)(struct _lv_obj_t *obj, uint32_t *id_tab, uint8_t id_num)` | Base label refresh callback function type | `obj`: object pointer; `id_tab`: pointer to the data-source ID array; `id_num`: number of data-source IDs; returns a 32-bit integer. This callback is used to fetch data from the application data facility and update the label's displayed content |

### Creation and refresh

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_obj_t *lv_baselabel_create(lv_obj_t *parent)` | Creates a base label object | `parent`: parent object pointer; returns the label object pointer on success, or `NULL` on failure |
| `void lv_baselabel_refresh_timer(lv_timer_t *timer)` | Refresh timer callback | `timer`: timer object pointer; when the timer expires, it calls the user-registered data callback to update the text; if an original position was set, it also re-aligns the label to that position |

### Text settings

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_baselabel_set_custom_text(lv_obj_t *label, const char *text)` | Sets custom text | `label`: label object; `text`: text content (must not be empty); dynamically allocates memory and copies it into the `cus_text` field |
| `void lv_baselabel_set_text_fmt(lv_obj_t *obj, const char *fmt, ...)` | Sets the text with a format string | `obj`: label object; `fmt`: format string; `...`: variable arguments. Similar to printf usage |
| `void lv_baselabel_set_text(lv_obj_t *obj, const char *text)` | Sets the displayed text | Directly calls the underlying `lv_label_set_text` |
| `void lv_baselabel_set_text_static(lv_obj_t *obj, const char *text)` | Sets static text (does not copy content) | Only stores the pointer, suitable for constant strings; the text lifetime must outlive the label |
| `void lv_baselabel_set_long_mode(lv_obj_t *obj, lv_label_long_mode_t long_mode)` | Sets the long-text display mode | `long_mode`: long-text mode (wrap, scroll, ellipsis, and so on) |
| `void lv_baselabel_set_recolor(lv_obj_t *obj, bool en)` | Enables/disables recolor | `en`: when `true`, color markers in the text can recolor parts of the text |
| `void lv_baselabel_set_ellip_txt(lv_obj_t *obj, const char *text, uint16_t display_w, bool ellip_en)` | Sets text with ellipsis | `text`: text; `display_w`: display-area width (pixels); `ellip_en`: `true` enables the ellipsis, appending `...` at the end when the text overflows |
| `char *lv_baselabel_get_text(const lv_obj_t *obj)` | Gets the current text | Returns a pointer to the text content |

### Text selection and character positioning

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_baselabel_set_text_sel_start(lv_obj_t *obj, uint32_t index)` | Sets the text-selection start position | `index`: start character index |
| `void lv_baselabel_set_text_sel_end(lv_obj_t *obj, uint32_t index)` | Sets the text-selection end position | `index`: end character index |
| `uint32_t lv_baselabel_get_text_selection_start(const lv_obj_t *obj)` | Gets the text-selection start position | Returns the start character index |
| `uint32_t lv_baselabel_get_text_selection_end(const lv_obj_t *obj)` | Gets the text-selection end position | Returns the end character index |
| `lv_label_long_mode_t lv_baselabel_get_long_mode(const lv_obj_t *obj)` | Gets the long-text display mode | Returns the current long-text mode |
| `bool lv_baselabel_get_recolor(const lv_obj_t *obj)` | Gets the recolor state | Returns `true` when recolor is enabled |
| `void lv_baselabel_get_letter_pos(const lv_obj_t *obj, uint32_t char_id, lv_point_t *pos)` | Gets the coordinates of a specific character | `char_id`: character index; `pos`: output coordinates |
| `uint32_t lv_baselabel_get_letter_on(const lv_obj_t *obj, lv_point_t *pos_in)` | Gets the character index at the given coordinates | `pos_in`: input coordinates; returns the character index, or `LV_LABEL_POS_NONE` when the position is outside the text |
| `bool lv_baselabel_is_char_under_pos(const lv_obj_t *obj, lv_point_t *pos)` | Checks whether there is a character at the given coordinates | Returns `true` when there is a character |
| `void lv_baselabel_ins_text(lv_obj_t *obj, uint32_t pos, const char *txt)` | Inserts text at the given position | `pos`: insertion character index; `txt`: text to insert |
| `void lv_baselabel_cut_text(lv_obj_t *obj, uint32_t pos, uint32_t cnt)` | Cuts (deletes) text | `pos`: deletion start index; `cnt`: number of characters to delete |

### Static string table (sfat_str)

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_baselabel_add_sfat_str(lv_obj_t *label, char *str, uint8_t len)` | Adds a static string | `str`: string content; `len`: length; dynamically allocates memory and copies it, and increments the `sfat_str_num` counter |
| `void lv_baselabel_clear_sfat_str(lv_obj_t *label)` | Clears all static strings | Releases the memory of all added strings and resets the counter |
| `void lv_baselabel_select_sfat_str(lv_obj_t *label, uint8_t index)` | Selects and displays a static string by index | `index`: 0-based index; only sets the displayed content without copying the string; an out-of-range index neither reports an error nor shows anything |
| `uint8_t lv_baselabel_get_sfat_str_num(lv_obj_t *label)` | Gets the number of static strings | Returns the count; 0 means none have been added |
| `uint8_t lv_baselabel_get_select_sfat_str_idx(lv_obj_t *label)` | Gets the index of the currently displayed static string | Range 0 to `sfat_str_num-1`; reset to 0 after clearing |

## Typical Usage

### Data-driven refresh (recommended)

When the refresh timer expires it calls the data callback; inside the callback, format the current data into the label:

```c
#include "lvsf_baselabel.h"
#include "lvsf_obj_ext.h"   /* lv_obj_set_gmdata_cb / set_source_id / create_refresh_timer */

/* Data callback: called when the refresh timer expires */
static int32_t uptime_gmdata_cb(lv_obj_t *label, uint32_t *id_tab, uint8_t id_num)
{
    (void)id_tab;
    (void)id_num;
    static uint32_t secs = 0;
    secs++;
    char buf[32];
    lv_snprintf(buf, sizeof(buf), "uptime  %02u:%02u",
                (unsigned)((secs / 60) % 100), (unsigned)(secs % 60));
    lv_baselabel_set_text(label, buf);   /* Build the string yourself + set_text */
    return 0;
}

void demo_baselabel_init(void)
{
    lv_obj_t *label = lv_baselabel_create(lv_scr_act());
    lv_obj_set_style_text_font(label, &lv_font_montserrat_36, 0);
    lv_obj_set_style_text_color(label, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_baselabel_set_text(label, "uptime  00:00");   /* Initial text before the first refresh */
    lv_obj_center(label);

    /* Bind the data source + callback + 1-second refresh timer */
    static uint32_t source_id = 0x1105;
    lv_obj_set_source_id(label, &source_id, 1);
    lv_obj_set_gmdata_cb(label, uptime_gmdata_cb);
    lv_obj_create_refresh_timer(label, 1000, lv_baselabel_refresh_timer);
    lv_obj_refresh_start(label);
}
```

```{note}
To update the text inside the callback, build the string yourself with `lv_snprintf()` and then call `lv_baselabel_set_text()`. `lv_baselabel_set_text_fmt()` is intended for data-binding format strings, not ordinary printf; passing variable arguments directly to it produces incorrect results.
```

### Switching the static string table

```c
lv_baselabel_add_sfat_str(label, "Mon", 3);
lv_baselabel_add_sfat_str(label, "Tue", 3);
lv_baselabel_add_sfat_str(label, "Wed", 3);
lv_baselabel_select_sfat_str(label, 1);   /* Show "Tue" */
```

## Demo

Run the `lvgl_v8_baselabel` example to see the result: a large blue `uptime 00:00` is shown in the center of the screen. The refresh timer then triggers the data callback once per second, and the label increments to `00:01`, `00:02`…… The text change is driven entirely by the baselabel data refresh path.

## See Also

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [LVGL v8 Label documentation](https://docs.lvgl.io/8.3/widgets/label.html)
- Source path: `middleware/lvgl/lvsf/gui_widgets/lvsf_baselabel.h`
- Example path: `example/multimedia/lvgl/lvgl_v8_baselabel`
