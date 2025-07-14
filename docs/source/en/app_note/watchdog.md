
# Watchdog Usage Guide

## Using the Watchdog

The SiFli SF32LB55X chip provides a corresponding watchdog for both HCPU and LCPU. In the SDK, you can enable the watchdog by enabling the following switch in the project via menuconfig. <br/>

![](../../assets/watchdog.png)

This switch enables the chip's hardware watchdog and also activates the RT-Thread watchdog software driver. The default watchdog timeout is 30 seconds.

If you need to modify the watchdog's timeout, you can make the following configuration in menuconfig:

![](../../assets/watchdog2.png)

## Notes

- The system will automatically feed the watchdog in the IDLE thread. If other threads do not run continuously beyond the watchdog's timeout, no additional code is needed.

- If the user has a high-priority thread that needs to run continuously beyond the watchdog timeout (this use case is rare), the user needs to call `rt_hw_watchdog_pet()` in the corresponding thread to ensure the watchdog is fed before the timeout occurs.

- When the system enters Standby/hibernate sleep modes, the watchdog will be disabled. It will be re-enabled when the system wakes up. In other sleep modes, the watchdog will continue running. The system must be woken up and fed the watchdog before the timeout.

- The HCPU uses the system watchdog. When triggered, it will reset the entire chip.

- The LCPU uses the LCPU subsystem's watchdog. When triggered, it generates an interrupt. The interrupt handler will trigger a software restart of the entire chip.
