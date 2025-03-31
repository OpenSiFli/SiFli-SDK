# SYSCFG

SYSCFG（SYStem ConFiG）模块用于配置系统功能，包括安全模式和SWD接口。 它还可以用于获取芯片组 ID、启动模式等系统信息。 Syscfg 模块使用 MACRO 实现，用户需要包含`bf0_hal.h`。

## 使用syscfg

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

## API参考
[bf0_hal.h](hal-syscfg)

