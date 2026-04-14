
# Application Startup Flow
```{only} SF32LB52X
SF32LB52X is a dual-core chip with multiple on-chip and off-chip memory interfaces. MPI1 is the on-chip memory interface and can connect PSRAM and NOR Flash. MPI2 and SDMMC are off-chip; MPI2 can connect NOR/PSRAM/NAND, and SDMMC can connect SD-NAND or SD-eMMC. The application runs on the big core. The Bluetooth Controller stack runs on the small core, which is not open to users. The small core is started by the Bluetooth Host stack on the big core; users do not need to care about it.
```
```{only} SF32LB55X
SF32LB55x uses a dual-core (big–little) architecture with rich internal and external memory interfaces. QSPI1, QSPI2 and QSPI3 are located in HPSYS and can send requests to DMAC1; QSPI4 is in LPSYS and can send requests to DMAC2; OPI‑PSRAM is in HPSYS for accessing the on‑chip SiP 8‑line pSRAM.
QSPI1 I/O connects to IO(SIP) to access the on‑chip SiP single or dual NOR Flash.
QSPI2 and QSPI3 I/O connect to IO(PA) to access external NOR/NAND Flash.
QSPI4 I/O connects to IO(PB) to access the on‑chip SiP 4‑line pSRAM or external NOR/NAND Flash.
```
```{only} SF32LB56X
SF32LB56x is a dual‑core (big–little) architecture with multiple internal and external memory interfaces. MPI1, MPI2 and MPI3 are in HPSYS and can send requests to DMAC1; MPI5 is in LPSYS and can send requests to DMAC2.
MPI1 I/O connects to IO(SA) to access the on‑chip SiP 8‑line pSRAM.
MPI2 I/O connects to IO(SB) to access another on‑chip SiP 8‑line pSRAM.
MPI3 I/O connects to IO(PA) to access external NOR/NAND Flash.
MPI5 I/O connects to IO(SC) to access the on‑chip SiP 4‑line NOR Flash.
```
```{only} SF32LB58X
SF32LB58x uses a tri‑core architecture (high‑performance big core + low‑power small core + coprocessor core) with multiple internal and external memory interfaces. MPI1, MPI2, MPI3 and MPI4 are in HPSYS and can send requests to DMAC1; MPI5 is in LPSYS and can send requests to DMAC3.
MPI1 I/O connects to IO(SA) to access the on‑chip SiP 8‑line/16‑line pSRAM or 4‑line NOR Flash.
MPI2 I/O connects to IO(SB) to access another on‑chip SiP 8‑line/16‑line pSRAM.
MPI3 and MPI4 I/O connect to IO(PA) to access external NOR/NAND Flash.
MPI5 I/O connects to IO(SC) to access the on‑chip SiP 4‑line NOR Flash.
```

```{only} SF32LB52X
The application startup on the big core consists of three stages:
1. First‑stage Bootloader: Burned in the SF32LB52X internal ROM; loads the second‑stage bootloader from Flash to RAM and jumps to run it
2. Second‑stage Bootloader: Loads the application from Flash and jumps to execute it
3. Application: User program
```
```{only} SF32LB55X
In the 55 series startup flow, Flash1 usually refers to the SiP NOR Flash connected via the QSPI1 interface.
The application startup on the big core consists of two stages:
1. First‑stage Bootloader: Burned in the SF32LB55X internal ROM; loads the user application from Flash1 and jumps to execute it
2. Application: User program
```
```{only} SF32LB56X
The application startup on the big core consists of three stages:
1. First‑stage Bootloader: Burned in the SF32LB56X internal ROM; loads the second‑stage bootloader from Flash to RAM and jumps to run it
2. Second‑stage Bootloader: Loads the application from Flash and jumps to execute it
3. Application: User program
```
```{only} SF32LB58X
The application startup on the big core consists of three stages:
1. First‑stage Bootloader: Burned in the SF32LB58X internal ROM; loads the second‑stage bootloader from Flash to RAM and jumps to run it
2. Second‑stage Bootloader: Loads the application from Flash and jumps to execute it
3. Application: User program
```

## First‑stage Bootloader
```{only} SF32LB52X
The first‑stage bootloader is burned in the chip ROM with the interrupt vector table address at 0. After power‑on, the first‑stage bootloader runs first. Depending on the package type, it determines the location of the Flash partition table (internal or external Flash, referred to as the boot Flash below; Flash includes NOR, NAND, SD and eMMC). According to the second‑stage bootloader address indicated by the partition table (must be on the boot Flash), it copies the second‑stage bootloader code to RAM and jumps to run it.
```

```{only} SF32LB58X or SF32LB56X
The first‑stage bootloader is burned in the chip ROM with the interrupt vector table address at 0. After power‑on, the first‑stage bootloader runs first. The Flash partition table is placed on Flash5 connected via MPI5 ("Flash5" usually refers to the external Flash storage device connected through MPI5). According to the second‑stage bootloader address indicated by the partition table (must be on the boot Flash), it copies the second‑stage bootloader code to RAM and jumps to run it.
```

````{only} SF32LB52X
During the first‑stage bootloader, the big core runs at the default clock frequency after power‑up and initializes the boot Flash IO configuration. For the numeric series, both cold boot and hibernate resumes wait for 2 seconds at the first‑stage boot; for the letter series, only cold boot waits 2 seconds.

For numeric‑series chips, VDD33_LDO2 (corresponding to the chip's VDD33_VOUT1 output) is turned on during the first‑stage bootloader. For letter‑series chips, PA21 is driven high during the first‑stage bootloader.

```{warning}
If booting from NOR Flash, make sure the Flash device is in 3‑byte address mode (for devices larger than 16 MB, normal operation may set 4‑byte address mode to access the full address space), otherwise the first‑stage bootloader cannot correctly read from Flash. For the numeric series, calling `HAL_PMU_Reboot` to reboot or `HAL_PMU_EnterHibernate` to power off will automatically turn off VDD33_LDO2, so on the next power‑up the device returns to the default 3‑byte address mode.
```
````

```{only} SF32LB55X
The first‑stage bootloader is burned in the chip ROM with the interrupt vector table address at 0. After power‑on, the first‑stage bootloader runs first, at the default big‑core clock frequency. After basic initialization, it directly loads the user application from Flash1.

In the 55 series, PA58 is driven high during the first‑stage bootloader to control Flash1 power supply.
```

```{only} SF32LB58X or SF32LB56X
During the first‑stage bootloader, the big core runs at the default clock frequency after power‑up and initializes the boot Flash IO configuration.
```


## Second‑stage Bootloader

```{only} SF32LB52X
The second‑stage bootloader loads the application and jumps to execute it according to the package type and the Flash partition table. Depending on the package type, applications support the following boot modes. Execution modes include XIP (execute directly at a NOR Flash address; code storage address equals execution address) and non‑XIP (copy code from Flash into RAM for execution; the storage address differs from the execution address). Regardless of boot mode, the application and the second‑stage bootloader reside on the same boot Flash; the difference is only how the application code executes:
```

```{only} SF32LB55X
For the 55 series, after basic initialization in the first‑stage bootloader, the user application is loaded directly from Flash1; there is no second‑stage bootloader.
```

```{only} SF32LB56X or SF32LB58X
The second‑stage bootloader loads the application and jumps to execute it according to the package type and the Flash partition table. Depending on the package type, applications support the following boot modes. Execution modes include XIP (execute directly at a NOR Flash address; code storage address equals execution address) and non‑XIP (copy code from Flash into RAM for execution; the storage address differs from the execution address):
```

```{only} SF32LB52X 
1. On‑chip NOR Flash (MPI1): Boot Flash is on‑chip NOR; the application is stored on on‑chip NOR and runs in XIP mode
2. No on‑chip NOR Flash:
   1. External NOR Flash (MPI2): Boot Flash is external NOR; the application is stored on external NOR and runs in XIP mode
   2. On‑chip PSRAM (MPI1), external NAND Flash (MPI2): Boot Flash is external NAND; the application is stored on external NAND and runs in non‑XIP mode, i.e., code is copied to on‑chip PSRAM for execution
   3. On‑chip PSRAM, external SD Flash (SDIO): Same as item 2.
```
```{only} SF32LB56X
1. On‑chip Flash (MPI5): Boot Flash is on‑chip NOR, used to store the Flash partition table, the second‑stage bootloader, and part of LCPU code.
2. External NOR Flash (MPI3): External NOR is connected via MPI3; the application is stored on external NOR and runs in XIP mode.
3. On‑chip PSRAM (MPI1/2), external NAND Flash (MPI3): The application is stored on external NAND and runs in non‑XIP mode, i.e., code is copied to on‑chip PSRAM for execution.
4. On‑chip PSRAM (MPI1/2), external SD Flash (SDIO): Same as item 2.
```
```{only} SF32LB58X 
1. On‑chip NOR Flash (MPI5): Boot Flash is on‑chip NOR, used to store the Flash partition table and the second‑stage bootloader.
2. External NOR Flash (MPI3/4): External NOR is connected via MPI3/4; the application is stored on external NOR and runs in XIP mode.
3. On‑chip PSRAM (MPI1/2), external NAND Flash (MPI3/4): The application is stored on external NAND and runs in non‑XIP mode, i.e., code is copied to on‑chip PSRAM for execution.
4. On‑chip PSRAM (MPI1/2), external SD Flash (SDIO): Same as item 2.
```
```{only} SF32LB52X 
For packages with on‑chip PSRAM, the 52 numeric series turns on LDO1V8 and initializes PSRAM in the second‑stage bootloader. For letter series, whether to use LDO1V8 to power PSRAM depends on the board configuration.

The second‑stage bootloader adjusts clock settings; the modified configuration is shown below:

| **Module** | **Clock Source** | **Frequency (MHz)** |
| --- | --- | --- |
| DLL1 | / | 144 MHz |
| DLL2 | / | 288 MHz |
| Big‑core system clock | DLL1 | 144 MHz |
| On‑chip NOR Flash | System clock | 48 MHz |
| On‑chip PSRAM | DLL2 | 144 MHz |
| External Flash | DLL2 | 48 MHz |
| External SD | DLL2 |  |


The second‑stage bootloader does not load PMU calibration parameters; it only modifies the IO settings related to the storage being used. Cache is disabled, and MPU is disabled.

| **Module** | **Setting** |
| --- | --- |
| PMU | Default |
| MPU | Disabled |
| Cache | Disabled |
```

## Application

### ResetHandler
 ```{only} SF32LB52X
The application entry function is `ResetHandler` (in `drivers\cmsis\sf32lb52x\Templates\arm\startup_bf0_hcpu.S`). Its flow is shown below. The user `main` function is called by the main thread created in `rt_application_init`. See {ref}`main_thread_entry flow <main_thread_entry_flow>`.
```
 ```{only} SF32LB55X
The application entry function is `ResetHandler` (in `drivers\cmsis\sf32lb55x\Templates\arm\startup_bf0_hcpu.S`). Its flow is shown below. The user `main` function is called by the main thread created in `rt_application_init`. See {ref}`main_thread_entry flow <main_thread_entry_flow>`.
```
 ```{only} SF32LB56X
The application entry function is `ResetHandler` (in `drivers\cmsis\sf32lb56x\Templates\arm\startup_bf0_hcpu.S`). Its flow is shown below. The user `main` function is called by the main thread created in `rt_application_init`. See {ref}`main_thread_entry flow <main_thread_entry_flow>`.
```
 ```{only} SF32LB58X
The application entry function is `ResetHandler` (in `drivers\cmsis\sf32lb58x\Templates\arm\startup_bf0_hcpu.S`). Its flow is shown below. The user `main` function is called by the main thread created in `rt_application_init`. See {ref}`main_thread_entry flow <main_thread_entry_flow>`.
```
```{image} ../../assets/ResetHandler.png
:alt: reset_handler_flow
:name: reset_handler_flow
```


### SystemInit
```{only} SF32LB52X
`SystemInit` (in `drivers/cmsis/sf32lb52x/Templates/system_bf0_ap.c`) runs before variable initialization (therefore do not use variables with initial values during this period, and avoid relying on the zero‑initialized section being 0). It updates the VTOR register to redirect the interrupt vector table, and calls `mpu_config` and `cache_enable` to initialize the MPU and enable Cache. These two functions are weak symbols, so applications can override them.
```
```{only} SF32LB55X
`SystemInit` (in `drivers/cmsis/sf32lb55x/Templates/system_bf0_ap.c`) runs before variable initialization (therefore do not use variables with initial values during this period, and avoid relying on the zero‑initialized section being 0). It updates the VTOR register to redirect the interrupt vector table, and calls `mpu_config` and `cache_enable` to initialize the MPU and enable Cache. These two functions are weak symbols, so applications can override them.
```
```{only} SF32LB56X
`SystemInit` (in `drivers/cmsis/sf32lb56x/Templates/system_bf0_ap.c`) runs before variable initialization (therefore do not use variables with initial values during this period, and avoid relying on the zero‑initialized section being 0). It updates the VTOR register to redirect the interrupt vector table, and calls `mpu_config` and `cache_enable` to initialize the MPU and enable Cache. These two functions are weak symbols, so applications can override them.
```
```{only} SF32LB58X
`SystemInit` (in `drivers/cmsis/sf32lb58x/Templates/system_bf0_ap.c`) runs before variable initialization (therefore do not use variables with initial values during this period, and avoid relying on the zero‑initialized section being 0). It updates the VTOR register to redirect the interrupt vector table, and calls `mpu_config` and `cache_enable` to initialize the MPU and enable Cache. These two functions are weak symbols, so applications can override them.
```
```{image} ../../assets/SystemInit.png
:alt: system_init_flow
:name: system_init_flow
```

### rt_hw_board_init
`rt_hw_board_init` performs low‑level hardware initialization such as clock and IO configuration, PSRAM and NOR Flash initialization, heap and serial console initialization. `rt_components_board_init` is an application‑defined initialization hook and calls different functions depending on the application configuration.

```{image} ../../assets/rt_hw_board_init.png
:alt: rt_hw_board_init
:name: rt_hw_board_init
```


#### HAL_Init

`HAL_Init` completes HAL initialization: it loads PMU calibration parameters, updates clock and IO settings, and initializes PSRAM and NOR Flash (according to the new clock configuration). In the diagram, green functions are board‑level driver functions with board‑specific implementations (`HAL_PreInit`, `BSP_IO_Init`, `BSP_PIN_Init`, `BSP_Power_Up`, etc.). Gray functions are virtual hooks implemented by the application (or not), independent of the board, so different applications on the same board can customize behavior (e.g., different IO configurations). Horizontally, the flow shows nested calls inside functions (e.g., `HAL_PreInit` calls clock‑configuration helpers; `HAL_MspInit` calls `BSP_IO_Init`). Vertically, it shows serially executed functions (e.g., after `HAL_PreInit` finishes, `HAL_PostMspInit` runs).


```{image} ../../assets/hal_init_english.png
:alt: hal_init_flow
:name: hal_init_flow
```

```{only} SF32LB52X
Config Clock adjusts:

* Load PMU calibration values
* Start GTimer
* Switch PMU to RC32K
* If external XT32K is used, switch RTC to XT32K
* Set system clock to 240 MHz (DLL1)
* Set DLL2 to 288 MHz (same as the second‑stage bootloader)

Loaded PMU calibration registers include:

* BUCK\_CR1\_BG\_BUF\_VOS\_POLAR
* BUCK\_CR1\_BG\_BUF\_VOS\_TRIM
* LPSYS\_VOUT\_VOUT
* VRET\_CR\_TRIM
* PERI\_LDO\_LDO18\_VREF\_SEL
* PERI\_LDO\_LDO33\_LDO2\_SET\_VOUT
* PERI\_LDO\_LDO33\_LDO3\_SET\_VOUT
* AON\_BG\_BUF\_VOS\_POLAR
* AON\_BG\_BUF\_VOS\_TRIM
* HXT\_CR1\_CBANK\_SEL (added in Xiaomi branch; previously loaded in the rt_component_board_init stage). The calibration‑loading code may run from Flash or PSRAM.

The details of the PMU parameters initialized by HAL_PMU_Init can be found in the HAL_PMU_Init function in drivers/hal/bf0_hal_pmu.c.

```

```{only} SF32LB55X
Config Clock adjusts:

* Start GTimer
* If external XT32K is not used, switch RTC to RC10K
* If external XT32K is used, switch RTC to XT32K
* Set system clock to 240 MHz (DLL1)
* Set DLL2 to 96 MHz

```
```{only} SF32LB56X or SF32LB58X
Config Clock adjusts:

* Start GTimer
* If external XT32K is not used, switch RTC to RC10K
* If external XT32K is used, switch RTC to XT32K
* Set system clock to 240 MHz (DLL1)
* Set DLL2 to 288 MHz
```

### rt_application_init

`rt_application_init` creates the main thread with entry `main_thread_entry`. After thread scheduling is enabled (i.e., after `rt_system_scheduler_start`), the main thread is scheduled and enters `main_thread_entry`, first calling `rt_components_init` to initialize components, then calling `main` (implemented by the application). User code starts from `main`. For example, the `rt_driver` sample's main function is in `example/rt_driver/src/main.c`.


```{image} ../../assets/main_thread_entry.png
:alt: main_thread_entry_flow
:name: main_thread_entry_flow
```

## Board‑level Driver Interfaces
```{only} SF32LB52X
Each board needs to implement the following board‑level driver functions. Refer to files under `customer/boards/eh-lb52xu`.
```
```{only} SF32LB55X
Each board needs to implement the following board‑level driver functions. Refer to files under `customer/boards/ec-lb55x`.
```
```{only} SF32LB56X
Each board needs to implement the following board‑level driver functions. Refer to files under `customer/boards/eh-lb56xu`.
```
```{only} SF32LB58X
Each board needs to implement the following board‑level driver functions. Refer to files under `customer/boards/ec-lb58x`.
```
| **Function** | **Required** | **Description** |
| --- | --- | --- |
| HAL\_PreInit | YES | Recommend referencing a board with similar hardware form |
| BSP\_Power\_Up | NO | Called after cold boot and wake‑up |
| BSP\_IO\_Power\_Down | NO | Called before sleep |
| BSP\_LCD\_Reset | NO |  |
| BSP\_LCD\_PowerUp | NO | Called when powering the display on |
| BSP\_LCD\_PowerDown | NO | Called when powering the display off |
| BSP\_TP\_Reset | NO |  |
| BSP\_TP\_PowerUp | NO | Called when powering touch on |
| BSP\_TP\_PowerDown | NO | Called when powering touch off |
| HAL\_MspInit | NO | Called by `HAL_PreInit`; virtual default calls `BSP_IO_Init` |
| HAL\_PostMspInit | NO |  |
| BSP\_IO\_Init | NO | Called by `HAL_MspInit` by default |
| BSP\_PIN\_Init | NO | Called by `BSP_IO_Init`; IO configuration function |
|  |  |  |


## Application‑defined Driver Interfaces

If different applications on the same board need different `HAL_MspInit` behavior, you can implement `HAL_MspInit` under the application directory; otherwise, it can reside under the board directory.

| **Function** | **Required** | **Description** |
| --- | --- | --- |
| HAL\_MspInit | NO |  |
| HAL\_PostMspInit | NO |  |


