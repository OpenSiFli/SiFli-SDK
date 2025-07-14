
# Application Startup Flow

The SF32LB52X is a dual-core chip with multiple built-in and external storage interfaces. MPI1 is the built-in storage interface that supports PSRAM and NOR Flash, while MPI2 and SDMMC are external storage interfaces. MPI2 supports NOR/PSRAM/NAND, and SDMMC can support SD-NAND or SD-eMMC. The application runs on the big core, while the Bluetooth Controller protocol stack runs on the small core. The small core is not accessible to users, and its startup is controlled by the big core's Bluetooth Host protocol stack, so users do not need to concern themselves with it.

The application startup flow on the big core consists of three stages:

1. **Primary Bootloader**: Embedded in the ROM of SF32LB52X, it loads the secondary Bootloader from Flash into RAM and jumps to execute.
2. **Secondary Bootloader**: Loads the application from Flash and jumps to execute.
3. **Application**: The user program.

## Primary Bootloader

The primary Bootloader is embedded in the chip's ROM, with the interrupt vector table address set to 0. After power-up, the primary Bootloader runs first, determines the location of the Flash partition table (either internal or external Flash, referred to as the boot Flash, which includes NOR, NAND, SD, and eMMC), and based on the secondary Bootloader address indicated in the Flash partition table (which must be located in the boot Flash), it copies the secondary Bootloader code to RAM and jumps to execute.

In the primary Bootloader stage, the big core runs at the default clock frequency after power-up and initializes the IO configuration of the boot Flash.

For the digital series chips, the primary Bootloader will enable VDD33_LDO2 (corresponding to the chip's VDD33_VOUT1 output). For the letter series chips, the primary Bootloader will set PA21 to a high level.

```{warning}
If booting from NOR Flash, ensure that the Flash chip is in 3-byte address mode (for chips larger than 16MB, the Flash will normally be configured to 4-byte address mode for full address space access in regular use). Otherwise, the primary Bootloader will not be able to correctly read data from Flash. For the digital series, if the `HAL_PMU_Reboot` function is called for reboot or `HAL_PMU_EnterHibernate` for shutdown, VDD33_LDO2 will be automatically turned off, so the Flash chip will return to the default 3-byte address mode on power-up.
```

## Secondary Bootloader

The secondary Bootloader loads the application and jumps to execute based on the chip package type and Flash partition table. The application has the following startup methods, which can either be XIP (Execute In Place, where code is executed directly from NOR Flash) or non-XIP (where code is copied from Flash to RAM before execution). Regardless of the startup method, both the application and secondary Bootloader are stored in the same boot Flash, with the only difference being the execution method:

1. **Built-in NOR Flash (MPI1)**: The boot Flash is the built-in NOR Flash, and the application runs in XIP mode.
2. **No Built-in NOR Flash**:
   1. **External NOR Flash (MPI2)**: The boot Flash is the external NOR Flash, and the application runs in XIP mode.
   2. **Built-in PSRAM (MPI1), External NAND Flash (MPI2)**: The boot Flash is the external NAND Flash, and the application runs in non-XIP mode, i.e., the code is copied to built-in PSRAM for execution.
   3. **Built-in PSRAM, External SD Flash (SDIO)**: Same as option 2.

For package types with built-in PSRAM, the secondary Bootloader will enable LDO1V8 and initialize the PSRAM.

The secondary Bootloader also adjusts clock settings, with the updated configuration shown in the table below:

| **Module**           | **Clock Source** | **Frequency (MHz)** |
|----------------------|------------------|---------------------|
| DLL1                 | /                | 144MHz              |
| DLL2                 | /                | 288MHz              |
| Big Core System Clock| DLL1             | 144MHz              |
| Built-in NOR Flash   | System Clock     | 48MHz               |
| Built-in PSRAM       | DLL2             | 144MHz              |
| External Flash       | DLL2             | 48MHz               |
| External SD          | DLL2             |                     |

The secondary Bootloader does not load PMU calibration parameters, only modifying IO settings related to storage. Cache and MPU are not enabled.

| **Module**           | **Configuration** |
|----------------------|-------------------|
| PMU                  | Default           |
| MPU                  | Disabled          |
| Cache                | Disabled          |

## Application

### ResetHandler 
The entry function for the application is `ResetHandler` (located in `drivers\cmsis\sf32lb52x\Templates\arm\startup_bf0_hcpu.S`), and its execution flow is shown in the diagram below. The user-defined `main` function is called by the main thread created by `rt_application_init`, as shown in {ref}`main_thread_entry_flow`.

![Alt text](../../assets/ResetHandler.png)

### SystemInit
`SystemInit` (located in `drivers/cmsis/sf32lb52x/Templates/system_bf0_ap.c`) executes before variable initialization (so variables with initial values or zero-initialized variables should not rely on their default values during this phase). It updates the VTOR register to redirect the interrupt vector table and calls `mpu_config` and `cache_enable` to initialize the MPU and enable the Cache. These functions are weak, and applications can override them to replace the default implementation.

![Alt text](../../assets/SystemInit.png)

### rt_hw_board_init
`rt_hw_board_init` completes low-level hardware initialization, such as clock and IO configuration, PSRAM and NOR Flash initialization, heap, and serial console initialization. `rt_components_board_init` is an application-defined initialization function, called differently based on the application's configuration.

![Alt text](../../assets/rt_hw_board_init.png)

#### HAL_Init
`HAL_Init` performs the HAL initialization, loads PMU calibration parameters, updates clock and IO settings, and initializes PSRAM and NOR Flash (based on the new clock configuration). The green functions in the diagram represent board-level driver functions, with each board having an independent implementation, including `HAL_PreInit`, `BSP_IO_Init`, `BSP_PIN_Init`, and `BSP_Power_Up`. The gray functions are virtual functions implemented by the application, allowing customization for different applications on the same board.

![Alt text](../../assets/hal_init.png)

The Config Clock modifications include:

* Loading PMU calibration values
* Starting the GTimer
* Switching PMU to RC32K
* Switching RTC to XT32K if using external XT32K
* Configuring system clock to 240MHz (DLL1)
* Configuring DLL2 to 288MHz (same as secondary Bootloader setting)

The loaded PMU calibration values include the following registers:

* BUCK_CR1_BG_BUF_VOS_POLAR
* BUCK_CR1_BG_BUF_VOS_TRIM
* LPSYS_VOUT_VOUT
* VRET_CR_TRIM
* PERI_LDO_LDO18_VREF_SEL
* PERI_LDO_LDO33_LDO2_SET_VOUT
* PERI_LDO_LDO33_LDO3_SET_VOUT
* AON_BG_BUF_VOS_POLAR
* AON_BG_BUF_VOS_TRIM
* HXT_CR1_CBANK_SEL

The code that loads these calibration values may run from Flash or PSRAM.

### rt_application_init

`rt_application_init` creates the main thread, with the thread entry function being `main_thread_entry`. Once thread scheduling is enabled (i.e., after calling `rt_system_scheduler_start`), the main thread is scheduled and enters the `main_thread_entry` function. It first calls `rt_components_init` to initialize components, then calls the main function (application-defined). The user code begins execution from the main function, for example, the main function in the `rt_driver` example is located in `example/rt_driver/src/main.c`.

![Alt text](../../assets/main_thread_entry.png)

## Board-Level Driver Interface

Each board needs to implement the following board-level driver functions, which can be referenced from files in the `customer/boards/eh-lb52xu` directory:

| **Function Name**      | **Required** | **Description** |
|------------------------|--------------|-----------------|
| HAL_PreInit            | YES          | It is recommended to refer to the implementation of similar hardware boards. |
| BSP_Power_Up           | NO           | Called after cold boot or wake-up. |
| BSP_IO_Power_Down      | NO           | Called before sleep. |
| BSP_LCD_Reset          | NO           |                 |
| BSP_LCD_PowerUp        | NO           | Called when the screen is powered up. |
| BSP_LCD_PowerDown      | NO           | Called when the screen is powered down. |
| BSP_TP_Reset           | NO           |                 |
| BSP_TP_PowerUp         | NO           | Called when touch is powered up. |
| BSP_TP_PowerDown       | NO           | Called when touch is powered down. |
| HAL_MspInit            | NO           | Called by HAL_PreInit; virtual function, implemented to call BSP_IO_Init. |
| HAL_PostMspInit        | NO           |                 |
| BSP_IO_Init            | NO           | By default, called by HAL_MspInit. |
| BSP_PIN_Init           | NO           | Called by BSP_IO_Init for IO configuration. |

## Application Custom Driver Interface

If different applications on the same board require different implementations of `HAL_MspInit`, the implementation of `HAL_MspInit` can be placed in the application directory; otherwise, it can be placed in the board directory.

| **Function Name**      | **Required** | **Description** |
|------------------------|--------------|-----------------|
| HAL_MspInit            | NO           |                 |
| HAL_PostMspInit        | NO           |                 |
