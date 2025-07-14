# Hello World Example (RT-Thread)

## Supported Platforms
<!-- Which development boards and chip platforms are supported -->
- Any board (including `pc`)

## Overview
The Hello_world application will print "hello world!" on the board's serial port.

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```
scons --board=sf32lb52-lcd_n16r8 -j32
```

Simulator: 
```
scons --board=pc -j32
```