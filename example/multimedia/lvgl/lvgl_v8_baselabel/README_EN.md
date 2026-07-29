# LVGL v8 Baselabel Example

Source path: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_baselabel`

## Overview

This example demonstrates the `lvsf_baselabel` widget: a normal `lv_label` with a built-in **data-refresh path**. Bind one or more data source ids, a data callback and a refresh timer, and the widget periodically pulls the data and updates its own text — so the label tracks live values without the app polling. It also keeps the usual `lv_label` features (set_text/set_text_fmt, long modes, recolor, a string table, etc.). This example shows an uptime `uptime MM:SS` that ticks every second.

Notes:
- The data-driven refresh is set up in four steps: `lv_obj_set_source_id()` binds the data source id(s), `lv_obj_set_gmdata_cb()` registers the data callback, `lv_obj_create_refresh_timer()` creates the refresh timer (pass the widget's own `lv_baselabel_refresh_timer`), and `lv_obj_refresh_start()` starts it. These APIs come from `lvsf_obj_ext`.
- When the refresh timer fires it calls `lv_baselabel_refresh_timer()`, which in turn calls your `gmdata_cb(label, id_tab, id_num)`; the callback formats the current data into the label — this is how baselabel turns a live data source into displayed text.
- To update the text in the callback, build the string yourself with `lv_snprintf()` and call `lv_baselabel_set_text()`. `lv_baselabel_set_text_fmt()` is a data-binding format, not a plain printf — passing varargs to it directly gives wrong results.
- Without data binding, baselabel is simply an enhanced label: call `lv_baselabel_set_text()` for fixed text, or use `set_long_mode`, `set_recolor`, the string table (`set_str_tab`/`add_sfat_str`/`select_sfat_str`), etc.

## Usage (key API)

```c
/* data callback: invoked when the refresh timer fires; formats data into the label */
static int32_t uptime_gmdata_cb(lv_obj_t *label, uint32_t *id_tab, uint8_t id_num)
{
    static uint32_t secs = 0;
    secs++;
    char buf[32];
    lv_snprintf(buf, sizeof(buf), "uptime  %02u:%02u",
                (unsigned)((secs / 60) % 100), (unsigned)(secs % 60));
    lv_baselabel_set_text(label, buf);   /* build the string yourself + set_text */
    return 0;
}

lv_obj_t *label = lv_baselabel_create(parent);
lv_baselabel_set_text(label, "uptime  00:00");   /* initial text before first refresh */

/* bind a data source + callback + a 1 s refresh timer */
static uint32_t source_id = 0x1105;
lv_obj_set_source_id(label, &source_id, 1);
lv_obj_set_gmdata_cb(label, uptime_gmdata_cb);
lv_obj_create_refresh_timer(label, 1000, lv_baselabel_refresh_timer);
lv_obj_refresh_start(label);
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

On startup the screen shows `uptime 00:00` in large blue text. The refresh timer then fires once per second, invoking the data callback, and the label increments to `00:01`, `00:02`, and so on. The app does not update the text in its main loop — the change is driven entirely by baselabel's data-refresh path.

## Troubleshooting

For technical questions, please open an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## References
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
