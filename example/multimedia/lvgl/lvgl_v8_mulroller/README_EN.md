# LVGL v8 Mulroller Example

Source path: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_mulroller`

## Overview

This example demonstrates the `lvsf_mulroller` widget: a wheel/drum picker. A strip of elements scrolls as you drag, and on release it snaps to the centered item as the current selection, with the center emphasised and the neighbours fading out — the classic rolling-wheel look. Element content is supplied on demand through a callback: when a slot scrolls into a new data index the callback fills it in, so only a handful of real elements can cover a large range of values.

To cover the widget's capabilities more fully, this example combines three wheels on one screen:
- a top horizontal, infinitely-looping weekday wheel (text content that wraps around);
- two vertical, bounded number wheels forming an HH:MM time picker.

Each wheel reports its settled value through `middle_cb` into one combined read-out below: `Weekday HH:MM`.

Notes:
- The element type is set by `lv_mulroller_set_obj_type()`: `MULROLLER_TYPE_LABEL` (text, used here), `MULROLLER_TYPE_IMG` (image), `MULROLLER_TYPE_IMGARRAY` (image digits), `MULROLLER_TYPE_MODULE` (custom module).
- Content callback `appear_cb(child, idx)`: called for each slot as it scrolls into a new data index; `child` is the element's child, `idx` is the data value at that slot — this is where the wheel content is produced. Number wheels format `"%02d"`; the weekday wheel indexes a name table modulo 7 to wrap around.
- Selection callback `middle_cb(child, idx)`: called when the wheel stops, with `idx` the now-centered (selected) data index — the entry point for reading the selected value.
- Direction is set by `lv_mulroller_set_dir()`: `MULROLLER_DIR_HOR` (horizontal) or `MULROLLER_DIR_VER` (vertical).
- Circle mode is set by `lv_mulroller_set_circle_mode()`: `MULROLLER_CIRCLE_NORMAL` is bounded by `set_circle_range(min,max)` and stops at the limits; `MULROLLER_CIRCLE_INFINITE` loops forever and wraps around.
- The centered item is emphasised by three "range + mode" pairs: zoom (`set_zoom_range`), color (`set_color_mode(MULROLLER_COLOR_POS)` / `set_color_range`), and opacity (`set_opa_mode(MULROLLER_OPA_MID)` / `set_opa_range`). Each range only takes effect with its matching mode enabled; `set_opa_range` applies only in `MULROLLER_CIRCLE_NORMAL` (ignored in infinite mode).
- For the LABEL type, the `set_zoom_range` value is a font size (mapped to a theme font): larger for the center, smaller for the sides.
- Whether it can scroll depends on geometry: the window must be smaller than the total size of all elements (height for vertical, width for horizontal), so create more elements than are visible.
- Infinite mode cannot be combined with `MULROLLER_LAYOUT_OVERLAP`, and needs at least 2 elements.
- The wheel wraps each element in a default `lv_obj` card (border / background / scrollbar). This example calls `lv_obj_remove_style_all()` on each element holder to drop that chrome and show only the content.
- Call `lv_mulroller_validate()` after all settings to apply the configuration.

## Usage (key API)

```c
/* number wheel: fill each slot while scrolling; idx is the data value at that slot */
static bool num_appear_cb(lv_obj_t *label, int16_t idx)
{
    lv_label_set_text_fmt(label, "%02d", idx);
    return true;
}

/* called when the wheel stops; idx is the centered (selected) value */
static bool hh_middle_cb(lv_obj_t *label, int16_t idx)
{
    s_hh = idx;            /* remember the selected hour, then refresh the combined read-out */
    readout_refresh();
    return true;
}

/* a vertical, bounded number wheel (hour/minute share this setup, only range + callback differ) */
lv_obj_t *r = lv_mulroller_create(parent);
lv_obj_set_size(r, 78, 156);                       /* window < total element height -> scrollable */
lv_mulroller_set_obj_type(r, MULROLLER_TYPE_LABEL);
lv_mulroller_create_element(r, 5, 78, 52);         /* 5 elements, 3 visible */

lv_mulroller_set_appear_cb(r, num_appear_cb);
lv_mulroller_set_middle_cb(r, hh_middle_cb);

lv_mulroller_set_dir(r, MULROLLER_DIR_VER);            /* vertical */
lv_mulroller_set_align(r, MULROLLER_ALIGN_CENTER);
lv_mulroller_set_layout_mode(r, MULROLLER_LAYOUT_MID);
lv_mulroller_set_circle_mode(r, MULROLLER_CIRCLE_NORMAL);
lv_mulroller_set_circle_range(r, 0, 23);              /* value range 00..23 */

lv_mulroller_set_zoom_range(r, 36, 24);               /* LABEL: value = font size, big center / small sides */
lv_mulroller_set_color_mode(r, MULROLLER_COLOR_POS);
lv_mulroller_set_color_range(r, 0xFF0000, 0x9E9E9E);  /* center red -> sides grey */
lv_mulroller_set_opa_mode(r, MULROLLER_OPA_MID);
lv_mulroller_set_opa_range(r, 255, 130);              /* center opaque -> sides faded */

lv_mulroller_validate(r);                             /* apply the settings above */

/* horizontal, infinite-loop weekday wheel: just switch dir to HOR and circle to INFINITE,
 * and have appear_cb index a name table by idx (modulo 7) to wrap around. */
```

## Supported Boards

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## Build and Download

The board project lives under `project`. Build for a specific board by passing its name:
- e.g. for sf32lb52-lcd_n16r8: run `scons --board=sf32lb52-lcd_n16r8 -j8` in `project`
- Download via `download.bat` (or `uart_download.bat`) under the build directory
- The SF32LB52x/SF32LB56x series also generate `uart_download.bat`; run it and enter the download UART port

## Simulator

Run `scons --board=pc_hcpu -j8` in `project` to produce `build_pc_hcpu/main.exe`, then run it to open the window.
- First adjust `SiFli-SDK/msvc_setup.bat` to match your local MSVC installation.

## Run Description

On startup the screen shows three wheels: a horizontal row of weekdays at the top (the centered one large and red, the sides small and grey) and an HH:MM pair of vertical number wheels below (also centered-emphasised, neighbours faded and greyed). Drag each wheel with the mouse (simulator) or finger (board):
- the horizontal weekday wheel can be dragged continuously in one direction and wraps around at Sun/Mon (infinite loop);
- the vertical hour/minute wheels stop at 00 and their upper limit without overrunning (bounded), and snap to the nearest item on release.

Each time a wheel settles, the read-out below updates to `Weekday HH:MM`, the current combination of the three wheels.

## Troubleshooting

For technical questions, please open an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## References
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
