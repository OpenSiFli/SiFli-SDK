# SYSCFG

The SYSCFG (SYStem ConFiG) module is used to configure system functions, including security mode and SWD interface. It can also be used to obtain system information such as chipset ID, boot mode, etc. The Syscfg module is implemented using MACROs, and users need to include `bf0_hal.h`.

## Using SYSCFG

```c
{
    #include "bf0_hal.h"
    
    ...
    printf("Boot mode is in %d mode\n", __HAL_SYSCFG_GET_BOOT_MODE()?"uart loop":"normal");
    
    // Following code is A0 only
    {
        printf("Serial ID=%d\n", __HAL_SYSCFG_GET_SID());
        printf("Chip ID=%d\n", __HAL_SYSCFG_GET_CID());
        printf("Package ID=%d\n", __HAL_SYSCFG_GET_PID());    
        printf("Revision ID=%d\n", __HAL_SYSCFG_GET_REVID());
        ...
        __HAL_SYSCFG_SET_SWD(SYSCFG_SWD_LCPU);      // Switch SWD interface to LCPU Core
        ...
        __HAL_SYSCFG_SET_SWD(SYSCFG_SWD_HCPU);      // Switch SWD interface to HCPU Core
        ...
        __HAL_SYSCFG_SET_SECURITY();                // Change to security mode, used in secure boot
        // Change secure boot setting.
        __HAL_SYSCFG_CLEAR_SECURITY();
        ...
    }
}    
```

## API Reference
[bf0_hal.h](hal-syscfg)
