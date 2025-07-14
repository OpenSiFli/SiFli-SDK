# Screen Module Debug Program

## Overview
This is a screen refresh display and touch data reading and printing program based on rt-thread's rt driver framework.

Suitable for debugging and testing screen drivers and touch drivers.

The screen can display content that changes every 3 seconds:
- Color bar with 8 gradient colors (white, gray, red, green, blue, cyan, purple, yellow), each gradient color transitions from black.
- Vertical grayscale gradient (cycles every 256 lines)
- Solid colors (red, green, blue, white, black)



## main Function
The main function creates 2 threads:
- LCD screen refresh thread
- Touch reading thread


### 1. LCD Screen Refresh Thread
1. Open lcd rt device
2. Get and search screen information, print its width, height, set color depth, screen refresh pixel alignment requirement information
3. Fill framebuffer (see overview description)
4. Refresh framebuffer to screen
5. After refresh completion, set backlight brightness
6. Wait 3 seconds
7. If it's a ramless screen, switch to another framebuffer
8. Jump to step 3

#### framebuffer Related
Color format definition FB_COLOR_FORMAT defaults to 24bit
Bytes per pixel for color format FB_PIXEL_BYTES defaults to 3

For ramless screens, to prevent framebuffer from being refreshed during filling, framebuffer2 is defined. During display, they are used alternately to prevent filling and refresh conflicts.

### 2. Touch Reading Thread
1. Open touch rt device
2. Set touch interrupt trigger function tp_rx_indicate to release semaphore
3. In while loop, if semaphore is acquired, read touch data and print it on serial port.