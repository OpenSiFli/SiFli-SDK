
# Low Power Usage Guide

## Low Power Modes

The two CPU cores in Sifli, HCPU and LCPU, can each operate in different low power modes:

- `PM_SLEEP_MODE_IDLE`:  
    The CPU enters the idle mode, where it halts at WFI or WFE. The system has high-speed clocks (HRC/HXT/DBLR/DLL), and all peripherals can be enabled to generate interrupts.
- `PM_SLEEP_MODE_LIGHT`:  
    The CPU enters light sleep mode, halting at the WFI instruction. The high-speed clock is turned off, peripheral devices related to the CPU stop working but do not lose power, and the system switches to the 32K clock.  
    The CPU can be awakened by a low power timer (LPTIM), RTC, BLE MAC (LCPU Only), Mailbox (other CPUs), or a specific wakeup pin. The wakeup time is between 30us-100us.  
    After waking up, the CPU continues executing the instructions after the WFI.
- `PM_SLEEP_MODE_DEEP`:  
    Similar to `PM_SLEEP_MODE_LIGHT`, but the system power switches to RET_LDO, and the wakeup time increases to 100us-1ms.  
    After waking up, the CPU continues executing the instructions after the WFI.
- `PM_SLEEP_MODE_STANDBY`:  
    The CPU enters standby mode, where the high-speed clock is turned off, peripherals related to the CPU lose power, and the system's RAM, except for the system-configured portions, stops receiving power. The system switches to RET_LDO, and the pin states are maintained.  
    The CPU can be awakened by a low power timer, RTC, BLE MAC (LCPU Only), Mailbox (other CPUs), or a specific wakeup pin. The wakeup time is between 1ms-2ms.  
    After waking up, the system will restart. Based on the low power mode indication in the AON register, software can determine if the system started from standby mode to distinguish it from a cold start.

In addition, the system also supports:

- Hibernate Mode:  
    In this mode, all system power is cut off, and the system switches to the 32K clock. The system can only be awakened by a specific pin or the RTC. The wakeup time is > 2ms. This mode is typically used when the system is powered off, allowing alarms and buttons to wake up the system.  
    The user can call `pm_shutdown()` to enter hibernation mode.
- Shutdown Mode:  
    In this mode, all system power is cut off, and the system switches to the RC10K clock. The system can only be awakened by a specific pin. The wakeup time is > 2ms. This mode is typically used during transportation, as it achieves the lowest power consumption, and buttons can wake up the system.  
    The user can call `HAL_PMU_EnterShutdown()` to enter shutdown mode.

## Low Power Implementation

The Sifli SDK implements a low power strategy based on the PM module of RT-Thread. After the system's idle time exceeds a specific threshold, it can enter different sleep modes.

Sifli's HCPU and LCPU have separate power domains controlling the peripherals they use. To share CPU peripheral resources, when LCPU is in the IDLE or ACTIVE state (not in sleep), HCPU can use all of LCPU's peripherals. However, it is important to avoid having both HCPU and LCPU use the same peripheral, as this would cause hardware resource conflicts when both cores are running simultaneously.  
If HCPU is using LCPU's peripherals (which is a common scenario), and HCPU is not in low power mode, LCPU must be in a wake-up state to allow proper access to LCPU's peripherals.  
In such cases, if LCPU enters standby mode, its peripherals will lose power, which limits HCPU's ability to operate in low power mode. HCPU must also enter standby mode. After chip wakeup, the system will first wake up LCPU, then reinitialize all peripherals (including LCPU's peripherals) to operate normally.

```{note} 
In standby mode, HCPU retains the RAM and CPU registers in PSRAM. Upon wakeup, HCPU reads the data saved in PSRAM to restore the operating context before sleep. LCPU retains the power to all RAM and stores the register contents in RAM. Upon wakeup, LCPU reads the data in RAM to restore the context.
```

Users can refer to the configuration in [](../middleware/pm.md) to enable low power support in the project. Using different low power strategies, the system can enter different low power modes. The default policy is currently configured as:

```c
const static pm_policy_t default_pm_policy[] =
{
    {15, PM_SLEEP_MODE_LIGHT},                  // Enter light sleep mode after 15ms of idle
#ifdef PM_STANDBY_ENABLE
    {10000, PM_SLEEP_MODE_STANDBY},             // Enter standby mode after 10s of idle
#endif /* PM_STANDBY_ENABLE */
};
```

## Low Power for Peripheral Devices

Since peripherals lose power in standby mode, the Sifli SDK in the RT-Thread drivers implements `RT_DEVICE_CTRL_RESUME` and `RT_DEVICE_CTRL_SUSPEND`. This allows each driver to save/restore settings before and after entering standby mode, enabling peripherals to enter low power mode and resume working with their original configurations after wakeup.  
The Sifli SDK also provides an interface to configure all pins connected to peripheral devices, helping manage leakage current from peripherals during sleep.  
```c
void BSP_IO_Power_Down(int coreid);
```
This function is called twice before HCPU enters sleep: once (`coreid=CORE_ID_LCPU`) before clearing the LCPU wakeup request (after which LCPU may enter low power mode, and HCPU can no longer access LCPU's power domain registers), to turn off the LCPU interface pins used by HCPU, and another time (`coreid=CORE_ID_HCPU`) before HCPU enters sleep, to turn off other pins used by HCPU.  
In LCPU projects, this function is called once before sleep to turn off the pins used by LCPU.

Different peripherals may require different ways to close the pins. By default, the low power configuration disables pull-up and pull-down resistors to avoid leakage current due to feedback loops. However, the high/low configuration of the pins needs to be modified based on the specific design.
