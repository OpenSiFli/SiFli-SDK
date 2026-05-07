# VGLite Demo Program

Source code path: example/vglite

## Supported Platforms

+ sf32lb58-lcd series

## Overview

This demo program demonstrates graphics rendering based on VG Lite hardware acceleration, containing 4 types of demo effects. Supports dynamic switching between different demos via the `demo_run_handler` command.

## Demo Effects

| ID | Name | Description |
|----|------|-------------|
| 0 | linear_grad | Linear gradient pattern, displays 10 frames |
| 1 | glyphs | Font rendering, displays small and large fonts |
| 2 | rotate | Rotating pointer image, displays 36 frames |
| 3 | coverflow | Album flip 3D effect, runs continuously |

## Usage

### Build and Flash

Navigate to the project directory and run scons to build:

```
scons -j20 --board=sf32lb58-lcd_n16r32n1_dpi
```

Run `build_sf32lb58-lcd_n16r32n1_dpi_hcpu\uart_download.bat` and follow the prompts to select the serial port for downloading.

### Running the Demo

After the device boots, the serial console will print the available demo list:

```
========== Available Demos ==========
0: linear_grad  - Linear gradient demo
1: glyphs       - Glyphs render demo
2: rotate       - Rotate image demo
3: coverflow    - Coverflow effect demo
======================================

Usage: demo_run_handler <0-3>
Example: demo_run_handler 0
```

Use the `demo_run_handler` command to run a specific demo:

| Command | Description |
|---------|-------------|
| `demo_run_handler 0` | Run linear gradient demo |
| `demo_run_handler 1` | Run glyphs demo |
| `demo_run_handler 2` | Run rotate demo |
| `demo_run_handler 3` | Run coverflow demo |

### Dynamic Switching

Switch between different demos dynamically:

```
msh />demo_run_handler 2    # Start rotating pointer
msh />demo_run_handler 0    # Switch to linear gradient (auto stops rotate)
msh />demo_run_handler 3    # Switch to coverflow (auto stops gradient)
```

## Expected Results

### linear_grad (ID: 0)

Displays a star-shaped path with linear gradient pattern, executes 10 frames and auto terminates:

```
Render size: 192 x 240, 2002fcc0
frame 0 done
frame 1 done
...
frame 9 done
```

### glyphs (ID: 1)

Displays text rendering in 2 frames, first with small font, second with enlarged characters:

```
Framebuffer size: 224 x 240
(Displays "Hello, Verisilicon!" text)
```

### rotate (ID: 2)

Displays a pointer image rotation animation, executes 36 frames (about 6 seconds):

```
Framebuffer size: 224 x 240
rot:1
rot:2
...
rot:36
```

### coverflow (ID: 3)

Displays 3D album flip effect with multiple images, runs continuously until manually switched:

```
Framebuffer size: 320 x 272
fps: 56.15
fps: 56.15
...
```

## Troubleshooting

1. If an Assert failure occurs when switching demos, wait for the previous demo to fully finish before executing the switch
2. To check memory usage, use the `list_mem` command
3. Debug info can be viewed via `list_thread` to check thread status

## Changelog

| Version | Date | Release Notes |
|:---|:---|:---|
| 0.0.1 | 04/2026 | Initial version with dynamic demo switching support |
