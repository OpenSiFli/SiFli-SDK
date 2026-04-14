# LVGL v8 Multlist Example

Source path: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_multlist`

## Overview

This example demonstrates how the `lvsf_multlist` widget can be organized and used in a practical application, with a focus on the following capabilities:

- Application entry, page registration, and subpage navigation based on `gui_app_fwk`
- Layout and focus control of `lvsf_multlist` in vertical and horizontal list scenarios
- Looping page-flip effects of `lvsf_multlist` in full-page scrolling scenarios
- Custom card animation effects through a `transform callback`
- Dynamic text conversation lists built with `lv_txtimg`

This example is not a minimal single-widget demo. Instead, it is a complete demo framework with a main entry page, menu page, functional subpages, resource caching, and return navigation. It can be used as a reference project for building `lvsf_multlist`-based interfaces.

## Supported Boards

<!-- Supported boards and chip platforms -->
- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## Hardware Requirements

Before running this example, prepare:

+ A supported development board for this example ([supported platforms](quick_start)).
+ A display.

Build and download:

Supported boards:
- Boards after the 55x series, such as 58x, 56x, and 52x

The board project is located in the `project` directory. You can build a project for a specific board by passing the `board` option.

- For example, to build a project for HDK 563, run `scons --board=eh-lb563`
- For downloading, use `download.bat` in the build directory. For example, to flash the generated 563 project, run `.\build_eh-lb563\download.bat` to download it through J-Link
- Special note: for the SF32LB52x/SF32LB56x series, an additional `uart_download.bat` will be generated. You can run this script and enter the UART port number to perform the download

## Simulator Configuration

The simulator project is located in the `simulator` directory.

- Use `scons` to build it. The `SiFli-SDK/msvc_setup.bat` file needs to be adjusted to match the MSVC environment on your machine
- You can also use `scons --target=vs2017` to generate the MSVC project file `project.vcxproj` and build it with Visual Studio
  Note: if you are not using VS2017, for example VS2022, Visual Studio may prompt you to upgrade the MSVC SDK when loading the project. After the upgrade, the project can be used normally

## Runtime Description

### Startup Flow

The startup flow is as follows:

- Call `gui_app_init()` to initialize the application framework
- Call `gui_app_run(DEMO_MULTLIST_MAIN_ID)` to enter the Multlist demo main entry

Therefore, the runtime entry of this example is not a single `lv_example_xxx()` function. It is a group of pages managed by `gui_app_fwk`.

### Page Structure

The example can be divided into two levels:

- Main entry page
  Description:
  `demo_multlist_main.c` creates a launcher card. Tapping it opens the Multlist main menu
- Multlist main menu page
  Description:
  `demo_multlist.c` creates a vertical `lvsf_multlist` menu and jumps to different function pages through `gui_app_run_subpage()`

The main menu currently contains the following sub-functions:

- `horizontal list`
- `vertical list`
- `horizontal page`
- `vertical page`
- `horizontal animation`
- `vertical animation`
- `Intercom list`

### Design Logic Framework

The overall design of this example can be understood as a three-layer structure: application framework layer, page layer, and resource layer.

- Application framework layer
  Description:
  Registers pages into `gui_app_fwk` through `APPLICATION_REGISTER_*` and `APP_PAGE_REGISTER`, and handles start, resume, pause, stop, and back navigation in a unified way
- Page implementation layer
  Description:
  Each subpage focuses only on its own `on_start / on_resume / on_pause / on_stop` lifecycle and the corresponding `lvsf_multlist` configuration
- Resource and cache layer
  Description:
  `demo_multlist_cards.c` manages image sources, card size calculation, and snapshot caching so that card resources can be reused by list pages and animation pages

### Description of Each Functional Page

- List page `demo_multlist_list.c`
  Description:
  Demonstrates a regular list style. The page first generates thumbnail cards through `demo_multlist_card_cache_init()`, then creates `lvsf_multlist`, and finally fills 100 list items according to horizontal or vertical mode
- Page-flip page `demo_multlist_page.c`
  Description:
  Demonstrates horizontal or vertical paging with full-screen images and enables `LV_MULTLIST_FLAG_LOOP` to implement cyclic page flipping
- Animation page `demo_multlist_anim.c`
  Description:
  Uses `lv_multlist_set_tranform_cb()` to customize element scaling, opacity, and layer ordering, providing a stronger visual animation effect
- Dialogue page `demo_multlist_dialog.c`
  Description:
  Combines `lv_txtimg` and `lvsf_multlist` to implement a chat/intercom-style message flow, supporting both new messages and appending text to the last message

### Resource Organization

Instead of using raw images directly, this example abstracts card resources into a unified snapshot cache:

- Original image resources are provided by `demo_list_0 ~ demo_list_3`
- `demo_multlist_cards.c` dynamically calculates list card sizes and animation card sizes according to screen size
- Pages use cached `lv_img_dsc_t` snapshots instead of creating temporary objects repeatedly

### Navigation and Interaction

- Every page creates a unified back button at the top. Clicking it calls `gui_app_goback()`
- When a page resumes, it calls `lv_multlist_on_resume()` and `lv_multlist_enable_encoder()`
- When a page pauses, it calls `lv_multlist_on_pause()` and `lv_multlist_disable_encoder()`
- On horizontal pages, gestures are temporarily disabled to avoid conflicts with the page scrolling direction

### Recommended Reference Scenarios

This example is suitable as a reference template for the following interface types:

- Launcher pages or function entry lists
- Card flows, cover flows, horizontal or vertical menus
- Full-page carousels or paged browsing interfaces
- Focused browsing interfaces with transition animations
- Chat history or intercom-style dynamic text lists

## Troubleshooting

If you have any technical questions, please submit an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## Reference Documents

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
