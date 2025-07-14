# LCD Module Stress Test Program
```{warning}
Not verified
```

## Overview
This is a test example program for testing the screen by using the "lcd" rt_device. The program will repeatedly call the following test cases:
1. In 3 RTOS threads, separately call the interfaces for setting screen brightness, synchronous screen refresh, and asynchronous screen refresh
2. Single thread repeatedly calls: close screen (rt_device_close), open screen (rt_device_open), refresh screen
3. Single thread repeatedly calls: close screen (RTGRAPHIC_CTRL_POWEROFF), system standby sleep, system wake up, open screen (RTGRAPHIC_CTRL_POWERON), refresh screen
4. Single thread repeatedly calls: screen enters idle mode, system standby sleep, system wake up, screen exits idle mode, refresh screen
5. Single thread repeatedly calls: system standby sleep, system wake up, refresh screen