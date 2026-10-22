# GPU Render List Mode Example Program
Source code path: example/rt_device/gpu/render_list_mode
```{warning}
Not verified
```
## Overview
EPIC (ePicasso) is SiFli's self-developed graphics engine, a hardware-accelerated module designed specifically for 2D/2.5D image processing. It is mainly used to offload image processing work from the CPU to improve image processing efficiency.

The graphics rendering driver layer provides two different implementations (which cannot be used at the same time):
1. Single-step rendering mode -- each operation is processed by the corresponding API immediately after it is prepared
2. Render list mode -- all operations for one frame are prepared, then processed asynchronously in batch

This example demonstrates the **render list mode**: a series of rendering operations (fill, image, rectangle, polygon, border, arc, line, bitmap font, etc.) are collected into a render list and rendered together. Rendering is performed in blocks using 2 framebuffers of 1/16 screen size (`render_buffer`), and the results are sent to the screen via callbacks. After one rendering is completed, new rendering operations are created and sent to the screen, repeating the demonstration in a loop.

## Features
This example is a graphics demonstration based on RT-Thread OS and the EPIC render list mode. It has 6 built-in scenes, switching automatically every 10 seconds in a loop:

| Scene | Contents |
|------|---------|
| Scene 0 | Full-screen gray fill → nested render list generating a random color-block test image (`generate_image`) → ARGB8565 image centered on screen → 4 concentric arcs (20°~160°, round start/end) |
| Scene 1 | Solid fill → EZIP-compressed clock background image (454x454, with scaling) → arc animation (6 concentric arcs, red foreground sweeping clockwise) |
| Scene 2 | Solid fill → rounded rectangles (alternating solid / 4-corner gradient) → rounded border (red, width 20) → bitmap fonts (27x30 glyphs, A8 half-transparent, tiled across the screen) |
| Scene 3 | Solid fill → lines (horizontal dashed, vertical dashed, skewed solid; width 20) → hexagon polygon → arc animation → bitmap fonts |
| Scene 4 | Solid fill → 3D flip animation of images around the Y / X axis (two images of different sizes, with perspective transform) |
| Scene 5 | Solid fill → JPEG hardware-decoded image (454x454, requires `EPIC_SUPPORT_JPEGD`, otherwise falls back to EZIP) → gradient buttons + 60x60 EZIP icons |

## Supported Boards
The example can run on the following boards (`project/proj.conf` is the default single-core config; sf32lb57x uses the dedicated dual-core HCPU config `project/sf32lb57x/proj.conf`):
* sf32lb52-lchspi-ulp
* sf32lb52 LCD series (e.g. `sf32lb52-lcd_n16r8`)
* sf32lb56 LCD series (e.g. `sf32lb56-lcd_a128r12n1`)
* sf32lb57x (dual-core HCPU mode; render list tasks run on the ACPU)

### Hardware Required
* A supported board: a package with PSRAM is required (applicable to 52x/56x; sf32lb57x needs the dual-core HCPU config)
* A matching LCD: the board must have an LCD interface and screen (e.g. sf32lb52-lcd_n16r8, sf32lb56-lcd_a128r12n1)
* A USB data cable (for serial flashing and log output)

## Usage
The following subsections provide only the information essential for this example. For complete instructions on configuring SiFli-SDK and using it to build and run projects, see the [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html).

### menuconfig Configuration
* The required key configurations are already written into `project/proj.conf` by default, so the project can be built directly. To modify them, run the following in the SDK root directory:
```
sdk.py menuconfig --board=<board_name>
```
The items below are explained one by one, in the same order as they appear in `project/proj.conf`:

#### 1. CONFIG_DRV_EPIC_NEW_API=y -- Render list mode (core switch)
- Path: Board Config → On-chip Peripherals → EPIC
    - Enable: Using epic render list API
    - Effect: Enables render list mode (new API); `main.c` is written entirely against this mode. Disabling it builds the single-step rendering mode instead.
    - Note: render list mode does NOT work on the SF32LB55x series, and rendering is asynchronous -- buffers referenced by the render list must stay valid until rendering completes.
- Sub-option EPIC Render List API Mode (single choice, default SINGLE_CORE):
    - `CONFIG_DRV_EPIC_NEW_API_SINGLE_CORE`: single-core mode (default config in `project/proj.conf`; the render list runs on the local CPU)
    - `CONFIG_DRV_EPIC_NEW_API_DUAL_CORE_HCPU`: dual-core HCPU mode (selects `USING_ACPU_CTRL_FWK`; used by the sf32lb57x project `project/sf32lb57x/proj.conf` -- the render list task `ACPU_TASK_epic_rl` is dispatched to and runs on the ACPU)
    - `CONFIG_DRV_EPIC_NEW_API_DUAL_CORE_ACPU`: dual-core ACPU mode (render list tasks run on the ACPU)

#### 2. CONFIG_BSP_USING_FULL_ASSERT=y -- HAL assert type
- Path: SiFli Built-in Components → HAL Assert Type
    - Select: Print and loop
    - Effect: On HAL assertion failure, prints the error and enters an endless loop. The choice has three options (Print and loop / Endless Loop / Empty); this project selects Print and loop, and `RT_ASSERT` also becomes active for easier debugging.

#### 3. CONFIG_RT_MAIN_THREAD_STACK_SIZE=4096 -- Main thread stack size
- Path: RT-Thread Components → The main() function as user entry function
    - Set: Set main thread stack size
    - Effect: Main thread stack size (bytes). This example performs many rendering operations inside main() (local structs, nested render lists), so the default 2048 is too small and 4096 is used instead.

#### 4. CONFIG_RT_MAIN_THREAD_PRIORITY=15 -- Main thread priority
- Path: RT-Thread Components → The main() function as user entry function
    - Set: Set main thread priority
    - Effect: Main thread priority; smaller value = higher priority (15 is medium; the EPIC render list thread `epic_task` uses priority `RT_THREAD_PRIORITY_HIGH-3`, which is numerically smaller than 15, i.e. higher priority).

#### 5. CONFIG_RT_USING_ULOG=y -- ulog logging
- Path: RT-Thread Components → Utilities
    - Enable: Enable ulog
    - Effect: Enables the RT-Thread ulog framework; once enabled, `rt_kprintf` output is processed by ulog (e.g. adding thread name / color control).

#### 6. # CONFIG_ULOG_USING_COLOR is not set -- Disable colored logs
- Path: RT-Thread Components → Utilities → ulog (after Enable ulog) → log format
    - Disable: Enable color log (enabled by default; disabled in this project)
    - Effect: Logs carry no ANSI color control characters, making serial-tool / log-file reading cleaner.

#### 7. CONFIG_ULOG_OUTPUT_THREAD_NAME=y -- Log thread name
- Path: RT-Thread Components → Utilities → ulog (after Enable ulog) → log format
    - Enable: Enable thread information (disabled by default; enabled in this project)
    - Effect: Every log line carries the thread name for easier multi-thread debugging.

### Build and Flash
Change to the example `project` directory and run scons to build (taking `sf32lb52-lchspi-ulp` as an example):
```
scons --board=sf32lb52-lchspi-ulp -j8
```
The build output is generated under `build_sf32lb52-lchspi-ulp_hcpu/`. Flash it with (Windows):
```
build_sf32lb52-lchspi-ulp_hcpu\uart_download.bat
```
On macOS/Linux:
```
./build_sf32lb52-lchspi-ulp_hcpu/uart_download.sh
```
Then select the serial port as prompted:
```none
please input the serial port num:6
```

## Example Output
The 6 scenes play in a loop on the LCD screen, switching every 10 seconds. See the "Features" table above for details. If you see the following console output, the example should be running correctly (excerpt; note that the scene number in the log is one greater than in the "Features" table: `Showing scene 1~6` corresponds to scenes 0~5, looping):

```none
__main start
generate_image start
generate_image done
Showing scene 1
Showing scene 2
Showing scene 3
Showing scene 4
Showing scene 5
Showing scene 6
```


---

## Code Walkthrough
### Main program flow:
```
Initialize GPU and LCD devices
  └─> drv_gpu_open() opens the GPU driver
  └─> open_lcd() opens the LCD device, gets screen info and pixel alignment requirement
  └─> drv_epic_setup_render_buffer() sets up 2 block-rendering buffers
      └─> generate test image (nested render list, EPIC_MSG_RENDER_TO_BUF renders into the buffer)
          └─> while(1) loop:
              ├─> drv_epic_alloc_render_list() allocates a new render list
              ├─> create a set of rendering operations for the current scene (drv_epic_alloc_op + drv_epic_commit_op)
              ├─> submit the EPIC_MSG_RENDER_DRAW message to start rendering
              ├─> partial_done_cb callback flushes each rendered block to the LCD (lcd_flush)
              ├─> wait for the render-done semaphore render_done_sema
              └─> wait for the LCD async flush to complete, then start the next frame
```

### Two execution scenarios of render list mode
1. **Render + display (`EPIC_MSG_RENDER_DRAW`)**
   Rendering is done block by block according to the configured render buffer size; after each block, the `partial_done_cb` callback is executed to flush the screen. This is what the main loop uses: `virtual_render_buf` only describes the color format and area of the render output, and `EPIC_MSG_RENDER_DRAW` does not draw into this buffer directly, so `data` can be a dummy address (the example uses `0xCCCCCCCC`).
2. **Render into a specified buffer (`EPIC_MSG_RENDER_TO_BUF`)**
   The render result is written directly into the specified buffer, and `done_cb` is called after the whole rendering finishes. The example's `generate_image()` uses this to generate the random color-block test image: it renders into the `test_image` buffer and `fill_done_cb` releases the semaphore when done.

### Key interfaces
#### 1. GPU / EPIC render list interfaces:
| Interface | Description |
|-----------------------------|------------------------------|
| drv_gpu_open()               | Initializes GPU resources, including interrupts |
| drv_epic_setup_render_buffer() | Sets up block-rendering buffers (2 buffers of the same size are required) |
| drv_epic_alloc_render_list() | Allocates a render list |
| drv_epic_alloc_op()          | Creates a rendering operation in the render list |
| drv_epic_op_alloc_letter()   | Allocates one glyph for a bitmap-font (LETTERS) operation |
| drv_epic_commit_op()         | Commits the rendering operation into the render list |
| drv_epic_render_msg_commit() | Submits the render message and starts rendering |
| drv_epic_render_trav()       | Traverses the render list (e.g. to free transform-matrix memory after rendering) |
| HAL_EPIC_LayerConfigInit()   | Initializes a layer / mask config structure |

#### 2. Render message types (EPIC_MsgIdDef):
| Message Type | Description |
|---------------------------|----------------------------------|
| EPIC_MSG_RENDER_TO_BUF    | Render into a specified buffer, then call done_cb |
| EPIC_MSG_RENDER_DRAW      | Render + block-wise display; call partial_done_cb after each block |
| EPIC_MSG_RENDER_DRAW2     | Render into the buffer first, then draw the result to the LCD |

#### 3. Rendering operation types (drv_epic_op_type_t):
| Operation Type | Description |
|---------------------------|----------------------------------|
| DRV_EPIC_DRAW_FILL        | Fill (solid color, with alpha) |
| DRV_EPIC_DRAW_RECT        | Rectangle (rounded corners, 4-corner gradient) |
| DRV_EPIC_DRAW_IMAGE       | Image (ARGB8565 / EZIP / JPEG, rotate/scale, 3D transform) |
| DRV_EPIC_DRAW_BORDER      | Border (rounded, top/bottom/left/right sides individually controllable) |
| DRV_EPIC_DRAW_ARC         | Arc (start/end angles, width, round start/end) |
| DRV_EPIC_DRAW_LINE        | Line (width, dash, round start/end, infinite length) |
| DRV_EPIC_DRAW_LETTERS     | Bitmap font (A8 glyphs, color + alpha) |
| DRV_EPIC_DRAW_POLYGON     | Polygon (up to 16 vertices) |

#### 4. LCD display interfaces:
| Interface | Description |
|-------------------------------------------|----------------------------------|
| rt_device_find("lcd") | Finds the LCD device |
| rt_device_open()      | Opens the LCD device |
| rt_device_control(..., RTGRAPHIC_CTRL_GET_INFO)   | Gets LCD screen info (resolution, pixel alignment) |
| rt_device_control(..., RTGRAPHIC_CTRL_SET_BUF_FORMAT) | Sets the framebuffer color format (RGB888) |
| rt_device_control(..., SF_GRAPHIC_CTRL_LCDC_FLUSH) | Flushes the rendered buffer to the screen (lcd_flush) |
| rt_device_control(..., RTGRAPHIC_CTRL_SET_NEXT_TE) | Sets the next-frame TE (tearing effect) sync |

## Troubleshooting

* Build warning "EPIC driver requrie RT_USING_MESSAGEQUEUE!": the EPIC driver submits render messages through a message queue; make sure `RT_USING_MESSAGEQUEUE` is enabled (RT-Thread Kernel → IPC → Enable message queue).
* Render list mode does NOT work on the SF32LB55x series: make sure the chip is SF32LB52x / SF32LB56x / SF32LB57x.
* Scene 5 does not show the JPEG image: make sure `EPIC_SUPPORT_JPEGD` is enabled (otherwise the scene falls back to the EZIP image).
* Flashing fails or no download: check that the correct serial port was selected (see `please input the serial port num` in "Build and Flash"), and that the board is connected with a working driver.
* Corrupted / torn display: make sure render buffers, image data, etc. meet the 4-byte alignment requirement; rendering is asynchronous, so all data referenced by the render list must stay valid until rendering completes.

For any technical questions, please file an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## Reference

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- EPIC graphics engine driver documentation: `docs/source/en/drivers/epic.md` (in the SDK)
