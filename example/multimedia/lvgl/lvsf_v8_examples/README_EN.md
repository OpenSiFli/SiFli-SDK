# LVSF v8 Custom Widget Examples

Source path: `example/multimedia/lvgl/lvsf_v8_examples`

## Introduction

This project gathers SiFli's custom widgets (lvsf widgets) built on LVGL v8 into **a single project**: each widget demo lives in its own sub-folder under `src/widgets/<name>/`, and **only one widget is built per firmware**. To see another widget or inspect another API, just select the corresponding example in the configuration, rebuild and reflash — there is no need to open a dozen separate projects.

> The demo sources are aggregated from the former standalone `lvgl_v8_<name>` projects. The demonstration code itself is unchanged; only the per-project `main.c` files were removed. A single `src/main.c` now calls the entry function of the example selected in menuconfig.

## Examples at a glance

| Config option `LV_WIDGET_DEMO_*` | Sub-folder | Widget | Entry function | Description |
|---|---|---|---|---|
| `BASECHART` | `widgets/basechart/` | `lvsf_basechart` | `demo_basechart_init()` | Chart widget drawing one or more line/bar data series |
| `BASELABEL` | `widgets/baselabel/` | `lvsf_baselabel` | `demo_baselabel_init()` | Label with a built-in data refresh path; text follows a data source periodically |
| `FOLLOW` | `widgets/follow/` | `lvsf_follow` | `demo_follow_init()` | Physics “gravity” icon menu; icons arranged in rings move and avoid each other |
| `IMGARRAY` | `widgets/imgarray/` | `lvsf_imgarray` | `demo_imgarray_init()` | Builds a number out of glyph images, like a nixie/ticker display |
| `IMGBAR` | `widgets/imgbar/` | `lvsf_imgbar` | `demo_imgbar_init()` | Progress bar that clips a foreground image according to a value |
| `MULROLLER` | `widgets/mulroller/` | `lvsf_mulroller` | `demo_mulroller_init()` | Multi-function roller/wheel picker; callback-fed content, snaps to center |
| `MULTANIM` | `widgets/multanim/` | `lvsf_multanim` | `lv_example_multanim()` | Multiple transition animation effects (animated images) |
| `MULTLIST` | `widgets/multlist/` | `lvsf_multlist` | `gui_app_run(DEMO_MULTLIST_MAIN_ID)` | Multi-function list; a multi-page example based on the gui_app_fwk framework |
| `MULTROLLER` | `widgets/multroller/` | `lvsf_multroller` | `demo_multroller_init()` | Simple declarative roller from a `'\n'`-separated option string; cyclic and snapping |
| `MULTSLIDER` | `widgets/multslider/` | `lvsf_multslider` | `demo_multslider_init()` | Slider whose round knob shows the reading (or a custom short text) |
| `SECTOR` | `widgets/sector/` | `lvsf_sector` | `demo_sector_init()` | Sector (angle) mask that reveals an image as a pie according to a value |
| `SELECT` | `widgets/select/` | `lvsf_select` | `demo_select_init()` | Selectable list; each row shows a selected/unselected icon, single-select |
| `TIMELINE` | `widgets/timeline/` | `lvsf_timeline` | `demo_timeline_init()` | Generic animation timeline engine mapping a time range to a value range |

Each sub-folder contains its own `README.md` / `README_EN.md` with an overview, key APIs, typical usage and run instructions. To inspect a widget's API usage, open `demo_<name>.c` and the README in its sub-folder.

## Quick start: switching the widget being demonstrated

The flow is simply: **pick an example (menuconfig or edit proj.conf) → rebuild → flash and run**. Below we switch from the default basechart to multlist as an example.

### Option 1: menuconfig TUI (recommended)

```bash
# 1) Activate the environment once per terminal, from the SDK root
cd /path/to/SiFli-SDK
. export.sh          # note the leading ". " (source the script)

# 2) Enter this project's project/ directory and open the config menu
cd example/multimedia/lvgl/lvsf_v8_examples/project
scons --board=sf32lb52-lcd_n16r8 --menuconfig
```

Then navigate into:

```
LVSF v8 widget examples  --->
    Select widget example  --->
        (X) basechart - base chart widget
        ( ) baselabel - base label widget
        ( ) ...
        ( ) multlist - multi-function list (gui_app framework)
        ( ) ...
```

Keys:

- `Up/Down` to move, `Enter` to open a `--->` sub-menu;
- Inside **Select widget example** (a single-choice list), move onto the desired widget and press `Space` (or `Enter`) to move the `(X)` marker to that line (only one can be selected at a time);
- Press `Esc` to go back level by level; when prompted “Save configuration?”, choose **Yes**.

Then rebuild and flash:

```bash
scons --board=sf32lb52-lcd_n16r8 -j8
```

### Option 2: edit proj.conf directly (scriptable / straightforward)

Open `project/proj.conf` in a text editor; the selected example is at the end:

```ini
# Default widget example (choice, only one); see README for how to switch
CONFIG_LV_WIDGET_DEMO_BASECHART=y
```

Change it to the target example (comment out / remove the old line and set the new one to `y`; a choice allows only one `y` at a time). For example, switch to multlist:

```ini
# CONFIG_LV_WIDGET_DEMO_BASECHART=y
CONFIG_LV_WIDGET_DEMO_MULTLIST=y
```

The available macro names are listed in the “Config option” column above: `LV_WIDGET_DEMO_BASECHART / BASELABEL / FOLLOW / IMGARRAY / IMGBAR / MULROLLER / MULTANIM / MULTLIST / MULTROLLER / MULTSLIDER / SECTOR / SELECT / TIMELINE`.

Save and rebuild (no manual cache cleanup is needed; scons detects the proj.conf change and regenerates the configuration):

```bash
scons --board=sf32lb52-lcd_n16r8 -j8
```

> Note: after switching examples you **must rebuild and reflash**; the firmware only contains the demo code of the single widget currently selected.

## Building and flashing

The board project is in the `project` directory. Build for a specific board (replace `--board=` with yours):

```bash
cd project
scons --board=sf32lb52-lcd_n16r8 -j8
```

Flash using the generated `download.bat` (or `uart_download.bat`) in the build directory. On the SF32LB52x/SF32LB56x series an extra `uart_download.bat` is generated; run it and enter the download UART port number.

## Directory layout

```
lvsf_v8_examples/
├── README.md / README_EN.md        # this file
├── assets/                         # image assets (only multanim / multlist need them)
│   └── SConscript                  #   packs only the images of the selected example
├── project/                        # build project (SConstruct / proj.conf / Kconfig ...)
└── src/
    ├── Kconfig                     # widget example selection menu (choice)
    ├── SConscript                  # builds only the sources of the selected example
    ├── main.c                      # single entry, calls the selected example via Kconfig macro
    └── widgets/                    # one sub-folder per widget example
        ├── basechart/              #   demo_xxx.c/h + README
        ├── ...
        └── multlist/               #   gui_app_fwk multi-page example (several .c files)
```

## Notes

- Only one example is built and run at a time; the sources and images of unselected examples are not compiled and do not occupy flash.
- The `multlist` example is built on the application framework (gui_app_fwk); its entry differs from the other examples and is handled separately in `main.c`.
- Examples that require file-system resources (Lottie JSON, video, font files) — `lvgl_v8_lottie`, `lvgl_v8_media`, `lvgl_v8_freetype` — are not included here and remain standalone projects.
- See the SDK documentation chapter “LVGL v8 custom widgets” for detailed API references.
