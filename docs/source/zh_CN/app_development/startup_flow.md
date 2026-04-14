
# 应用程序启动流程
```{only} SF32LB52X
SF32LB52X为双核芯片，有内置和外置多个存储接口，MPI1为内置存储接口，可接PSRAM与NOR Flash，MPI2和SDMMC为外置存储，MPI2可接NOR/PSRAM/NAND，SDMMC可接SD-NAND或SD-eMMC。应用程序运行在大核，蓝牙Controller协议栈运行在小核，小核不对用户开放，小核的启动由大核的蓝牙Host协议栈控制，用户无需关心。
```
```{only} SF32LB55X
SF32LB55x为双核（大小核）架构，内置与外置存储接口丰富。其中QSPI1，QSPI2和QSPI3位于HPSYS，可向DMAC1发送请求；QSPI4位于LPSYS，可向DMAC2发送请求；OPI-PSRAM 位于 HPSYS，用于访问芯片内合封(SiP) 的8线pSRAM。
QSPI1的输入输出连接至IO(SIP)，用于访问芯片内合封(SiP)的单片或双片NOR Flash。
QSPI2与QSPI3的输入输出连接至IO(PA)，用于访问芯片外接的NOR/NAND Flash。
QSPI4的输入输出连接至IO(PB)，用于访问芯片内合封(SiP)的4线pSRAM或芯片外接的NOR/NAND Flash。
```
```{only} SF32LB56X
SF32LB56x为双核（大小核）架构，有内置和外置多个存储接口，其中MPI1，MPI2和MPI3位于HPSYS，可向DMAC1发送请求；MPI5位于LPSYS，可向DMAC2发送请求。
MPI1 的输入输出连接至IO(SA)，用于访问芯片内合封(SiP)的8线pSRAM。
MPI2 的输入输出连接至IO(SB)，用于访问芯片内合封(SiP)的另一片8线pSRAM。
MPI3 的输入输出连接至IO(PA)，用于访问芯片外接的NOR/NAND Flash。
MPI5 的输入输出连接至IO(SC)，用于访问芯片内合封(SiP)的4线NOR Flash。
```
```{only} SF32LB58X
SF32LB58x为三核架构（高性能大核+低功耗小核+协处理核），有内置和外置多个存储接口，其中MPI1，MPI2，MPI3和MPI4位于HPSYS，可向DMAC1发送请求；MPI5位于LPSYS，可向DMAC3发送请求。
MPI1的输入输出连接至IO(SA)，用于访问芯片内合封(SiP)的8线/16线pSRAM或4线NOR Flash。
MPI2的输入输出连接至IO(SB)，用于访问芯片内合封(SiP)的另一片8线/16线pSRAM。
MPI3和MPI4的输入输出连接至IO(PA)，用于访问芯片外接的NOR/NAND Flash。
MPI5 的输入输出连接至IO(SC)，用于访问芯片内合封(SiP)的4线NOR Flash。
```

```{only} SF32LB52X
大核上的应用程序启动流程分为三个阶段：
1. 一级Bootloader：固化在SF32LB52X内部的ROM中，加载Flash中的二级Bootloader到RAM中跳转运行
2. 二级Bootloader：加载Flash中的应用程序并跳转执行
3. 应用程序：用户程序
```
```{only} SF32LB55X
在55系列的启动流程中，Flash1 通常指代通过 QSPI1接口连接的合封NOR Flash
大核上的应用程序启动流程分为两个阶段：
1. 一级Bootloader：固化在SF32LB55X内部的ROM中，加载Flash1中的应用程序并跳转执行
2. 应用程序：用户程序
```
```{only} SF32LB56X
大核上的应用程序启动流程分为三个阶段：
1. 一级Bootloader：固化在SF32LB56X内部的ROM中，加载Flash中的二级Bootloader到RAM中跳转运行
2. 二级Bootloader：加载Flash中的应用程序并跳转执行
3. 应用程序：用户程序
```
```{only} SF32LB58X
大核上的应用程序启动流程分为三个阶段：
1. 一级Bootloader：固化在SF32LB58X内部的ROM中，加载Flash中的二级Bootloader到RAM中跳转运行
2. 二级Bootloader：加载Flash中的应用程序并跳转执行
3. 应用程序：用户程序
```

## 一级Bootloader
```{only} SF32LB52X
一级Bootloader固化在了芯片的ROM中，其中断向量表地址为0。上电后会首先运行一级Bootloader，根据芯片封装类型，确定Flash分区表的位置（内部或者外部Flash，下文称为启动Flash，Flash包括NOR、NAND、SD和eMMC），根据Flash分区表指示的二级Bootloader地址（必须在启动Flash上），拷贝二级Bootloader代码到RAM中并跳转运行。
```

```{only} SF32LB58X or SF32LB56X
一级Bootloader固化在了芯片的ROM中，其中断向量表地址为0。上电后会首先运行一级Bootloader，flash分区表放在MPI5接的Flash5上（“Flash5”通常指代通过MPI5接口连接的外部Flash存储设备），根据Flash分区表指示的二级Bootloader地址（必须在启动Flash上），拷贝二级Bootloader代码到RAM中并跳转运行。
```

````{only} SF32LB52X
一级Bootloader阶段大核以上电默认的时钟频率运行，初始化启动Flash的IO配置。数字系列冷启动和hibernate启动在一级boot阶段会等待2秒，字母系列只有冷启动是2秒。

数字系列芯片在一级bootloader阶段会打开VDD33_LDO2（对应芯片的VDD33_VOUT1输出）。字母系列芯片在一级bootloader阶段会将PA21输出高电平。

```{warning}
如果从NOR Flash启动，需要确保Flash颗粒处于3字节地址模式（对于容量大于16MB的颗粒，为了访问全地址空间，正常使用时会配置成4字节地址模式），否则一级bootloader无法正确读取Flash的数据。对于数字系列，如果调用驱动`HAL_PMU_Reboot`重启或者`HAL_PMU_EnterHibernate`关机，都会自动关闭VDD33_LDO2，这样开机时颗粒状态就会回到默认的3字节地址模式。
```
````

```{only} SF32LB55X
一级Bootloader固化在了芯片的ROM中，其中断向量表地址为0。上电后会首先运行一级Bootloader，一级Bootloader阶段大核以上电默认的时钟频率运行，一级Bootloader完成基础初始化后，直接从Flash1 中加载用户应用程序。

55系列芯片在一级bootloader阶段会将PA58输出高电平来控制flash1供电。
```

```{only} SF32LB58X or SF32LB56X
一级Bootloader阶段大核以上电默认的时钟频率运行，初始化启动Flash的IO配置。
```


## 二级Bootloader

```{only} SF32LB52X
二级Bootloader根据芯片封装类型以及Flash分区表，加载应用程序并跳转执行。根据芯片封装类型，应用程序分为以下几种启动方式，运行方式分为XIP（直接以NOR Flash地址执行代码，代码的存储地址与执行地址相同）和非XIP（从Flash拷贝代码到RAM中执行，即代码的存储地址与执行地址不同）两种，不论是哪种启动方式，应用程序与二级Bootloader均存放在同一个启动Flash上，区别只是应用程序代码的运行方式不同：
```

```{only} SF32LB55X
55系列一级Bootloader完成基础初始化后，直接从Flash1 中加载用户应用程序，无二级Bootloader。
```

```{only} SF32LB56X or SF32LB58X
二级Bootloader根据芯片封装类型以及Flash分区表，加载应用程序并跳转执行。根据芯片封装类型，应用程序分为以下几种启动方式，运行方式分为XIP（直接以NOR Flash地址执行代码，代码的存储地址与执行地址相同）和非XIP（从Flash拷贝代码到RAM中执行，即代码的存储地址与执行地址不同）两种：
```

```{only} SF32LB52X 
1. 内置NOR Flash（MPI1）：启动Flash为内置NOR Flash，应用程序存储在内置NOR Flash上，以XIP方式运行
2. 无内置NOR Flash：
   1. 外置NOR Flash（MPI2）：启动Flash为外置NOR Flash，应用程序存储在外置NOR Flash上，以XIP方式运行
   2. 内置PSRAM（MPI1），外置NAND Flash（MPI2）：启动Flash为外置NAND Flash，应用程序存储在外置NAND Flash上，非XIP执行，即代码被拷贝到内置PSRAM执行
   3. 内置PSRAM，外置SD Flash（SDIO）：同 2。
```
```{only} SF32LB56X
1. 内置 Flash（MPI5）：启动Flash为内置NOR Flash，用于存放flash分区表、二级bootloader，以及LCPU的部分代码。
2. 外置NOR Flash（MPI3）：外接NOR Flash通过MPI3接口连接，应用程序存储在外置NOR Flash上，以XIP方式运行。
3. 内置PSRAM（MPI1/2），外置NAND Flash（MPI3）：应用程序存储在外置NAND Flash上，非XIP执行，即代码被拷贝到内置PSRAM执行
4. 内置PSRAM（MPI1/2），外置SD Flash（SDIO）：同 2。
```
```{only} SF32LB58X 
1. 内置NOR Flash（MPI5）：启动Flash为内置NOR Flash，用于存放flash分区表、二级bootloader。
外置NOR Flash（MPI3/4）：外接NOR Flash通过MPI3/4接口连接，应用程序存储在外置NOR Flash上，以XIP方式运行。
3. 内置PSRAM（MPI1/2），外置NAND Flash（MPI3/4）：应用程序存储在外置NAND Flash上，非XIP执行，即代码被拷贝到内置PSRAM执行
4. 内置PSRAM（MPI1/2），外置SD Flash（SDIO）：同 2。
```
```{only} SF32LB52X 
对于有内置PSRAM的封装类型，52数字系列会在二级bootloader打开LDO1V8并初始化PSRAM，字母系列要看板子配置，可以用LDO1V8给PSRAM供电，也可以不用。

二级Bootloader会调整时钟设置，修改后的配置见下表

| **模块** | **时钟源** | **频率（MHz）** |
| --- | --- | --- |
| DLL1 | / | 144MHz |
| DLL2 | / | 288MHz |
| 大核系统时钟 | DLL1 | 144MHz |
| 内置NOR Flash | 系统时钟 | 48MHz |
| 内置PSRAM | DLL2 | 144MHz |
| 外置Flash | DLL2 | 48MHz |
| 外置SD | DLL2 |  |


二级Bootloader不加载PMU的校准参数，仅修改所使用的存储相关的IO设置。Cache未使能，MPU未使能。

| **模块** | **配置** |
| --- | --- |
| PMU | 默认值 |
| MPU | Disabled |
| Cache | Disabled |
```

## 应用程序

### ResetHandler
 ```{only} SF32LB52X
应用程序的入口函数为`ResetHandler`（位于`drivers\cmsis\sf32lb52x\Templates\arm\startup_bf0_hcpu.S`），其执行流程如下图所示，用户主函数`main`则由`rt_application_init`创建的main线程调用，见{ref}`main_thread_entry流程 <main_thread_entry_flow>`。
```
 ```{only} SF32LB55X
应用程序的入口函数为`ResetHandler`（位于`drivers\cmsis\sf32lb55x\Templates\arm\startup_bf0_hcpu.S`），其执行流程如下图所示，用户主函数`main`则由`rt_application_init`创建的main线程调用，见{ref}`main_thread_entry流程 <main_thread_entry_flow>`。
```
 ```{only} SF32LB56X
应用程序的入口函数为`ResetHandler`（位于`drivers\cmsis\sf32lb56x\Templates\arm\startup_bf0_hcpu.S`），其执行流程如下图所示，用户主函数`main`则由`rt_application_init`创建的main线程调用，见{ref}`main_thread_entry流程 <main_thread_entry_flow>`。
```
 ```{only} SF32LB58X
应用程序的入口函数为`ResetHandler`（位于`drivers\cmsis\sf32lb58x\Templates\arm\startup_bf0_hcpu.S`），其执行流程如下图所示，用户主函数`main`则由`rt_application_init`创建的main线程调用，见{ref}`main_thread_entry流程 <main_thread_entry_flow>`。
```
```{image} ../../assets/ResetHandler.png
:alt: reset_handler_flow
:name: reset_handler_flow
```


### SystemInit
```{only} SF32LB52X
`SystemInit`（在文件`drivers/cmsis/sf32lb52x/Templates/system_bf0_ap.c`里）在变量初始化之前执行（因此这期间不能使用带初值的变量，零段变量也要避免依赖于初值0），更新VTOR寄存器重定向中断向量表，调用`mpu_config`和`cache_enable`初始化MPU并使能Cache，这两个函数为weak函数，应用程序中可以重新实现来替换默认的实现。
```
```{only} SF32LB55X
`SystemInit`（在文件`drivers/cmsis/sf32lb55x/Templates/system_bf0_ap.c`里）在变量初始化之前执行（因此这期间不能使用带初值的变量，零段变量也要避免依赖于初值0），更新VTOR寄存器重定向中断向量表，调用`mpu_config`和`cache_enable`初始化MPU并使能Cache，这两个函数为weak函数，应用程序中可以重新实现来替换默认的实现。
```
```{only} SF32LB56X
`SystemInit`（在文件`drivers/cmsis/sf32lb56x/Templates/system_bf0_ap.c`里）在变量初始化之前执行（因此这期间不能使用带初值的变量，零段变量也要避免依赖于初值0），更新VTOR寄存器重定向中断向量表，调用`mpu_config`和`cache_enable`初始化MPU并使能Cache，这两个函数为weak函数，应用程序中可以重新实现来替换默认的实现。
```
```{only} SF32LB58X
`SystemInit`（在文件`drivers/cmsis/sf32lb58x/Templates/system_bf0_ap.c`里）在变量初始化之前执行（因此这期间不能使用带初值的变量，零段变量也要避免依赖于初值0），更新VTOR寄存器重定向中断向量表，调用`mpu_config`和`cache_enable`初始化MPU并使能Cache，这两个函数为weak函数，应用程序中可以重新实现来替换默认的实现。
```
```{image} ../../assets/SystemInit.png
:alt: system_init_flow
:name: system_init_flow
```

### rt_hw_board_init
`rt_hw_board_init`完成底层硬件初始化，例如时钟和IO配置，PSRAM和NOR Flash初始化，heap和串口console的初始化。`rt_components_board_init`是应用程序自定义的初始化函数，随应用程序配置的不同而调用不同的函数。

```{image} ../../assets/rt_hw_board_init.png
:alt: rt_hw_board_init
:name: rt_hw_board_init
```


#### HAL_Init

`HAL_Init`完成HAL初始化，加载PMU的校准参数，更新时钟、IO设置， 初始化PSRAM和NOR Flash（根据新的时钟配置），下图中绿色函数为板级驱动函数，每个板子有独立的实现，包括`HAL_PreInit`、`BSP_IO_Init`、`BSP_PIN_Init`和`BSP_Power_Up`等，灰色函数为虚函数，由应用程序实现（也可以不实现），独立于板子，目的是相同板子不同的应用程序可以有自定义的实现，比如不同应用程序在同一块板子上使用不同的IO配置。流程图中横向为函数内的嵌套调用子函数，比如`HAL_PreInit`调用了时钟配置的函数，`HAL_MspInit`调用`BSP_IO_Init`，纵向为串行执行的几个函数，如`HAL_PreInit`执行完再执行`HAL_PostMspInit`。


```{image} ../../assets/hal_init.png
:alt: hal_init_flow
:name: hal_init_flow
```

```{only} SF32LB52X
Config Clock修改的设置包括：

* 加载PMU校准值
* 启动GTimer
* 切换PMU到RC32K
* 如果使用外置XT32K，则切换RTC到XT32K
* 配置系统时钟为240MHz(DLL1)
* 配置DLL2为288MHz（与二级bootloader的设置相同）

加载的PMU校准值包括以下寄存器：

* BUCK\_CR1\_BG\_BUF\_VOS\_POLAR
* BUCK\_CR1\_BG\_BUF\_VOS\_TRIM
* LPSYS\_VOUT\_VOUT
* VRET\_CR\_TRIM
* PERI\_LDO\_LDO18\_VREF\_SEL
* PERI\_LDO\_LDO33\_LDO2\_SET\_VOUT
* PERI\_LDO\_LDO33\_LDO3\_SET\_VOUT
* AON\_BG\_BUF\_VOS\_POLAR
* AON\_BG\_BUF\_VOS\_TRIM
* HXT\_CR1\_CBANK\_SEL 

HAL_PMU_Init初始化的PMU参数详情见 drivers/hal/bf0_hal_pmu.c 中的 HAL_PMU_Init 函数。

```

```{only} SF32LB55X
Config Clock修改的设置包括：

* 启动GTimer
* 如果没使用外置XT32K，则切换RTC到RC10K
* 如果使用外置XT32K，则切换RTC到XT32K
* 配置系统时钟为240MHz(DLL1)
* 配置DLL2为96MHz

```
```{only} SF32LB56X or SF32LB58X
Config Clock修改的设置包括：

* 启动GTimer
* 如果没使用外置XT32K，则切换RTC到RC10K
* 如果使用外置XT32K，则切换RTC到XT32K
* 配置系统时钟为240MHz(DLL1)
* 配置DLL2为288MHz
```

### rt_application_init

`rt_application_init`中创建main线程，线程入口函数为`main_thread_entry`，当放开线程调度后（即调用`rt_system_scheduler_start`之后），main线程得到调度，进入`main_thread_entry`函数，先调用`rt_components_init`初始化组件，随后即调用main函数（应用程序实现），用户代码即从main函数开始，比如rt\_driver示例的主函数在`example/rt_driver/src/main.c`中。


```{image} ../../assets/main_thread_entry.png
:alt: main_thread_entry_flow
:name: main_thread_entry_flow
```

## 板级驱动接口
```{only} SF32LB52X
每块板子需要实现如下板级驱动函数，可参考`customer/boards/eh-lb52xu`目录下的文件，
```
```{only} SF32LB55X
每块板子需要实现如下板级驱动函数，可参考`customer/boards/ec-lb55x`目录下的文件，
```
```{only} SF32LB56X
每块板子需要实现如下板级驱动函数，可参考`customer/boards/eh-lb56xu`目录下的文件，
```
```{only} SF32LB58X
每块板子需要实现如下板级驱动函数，可参考`customer/boards/ec-lb58x`目录下的文件，
```
| **函数名** | **必选** | **说明** |
| --- | --- | --- |
| HAL\_PreInit | YES | 建议参考硬件形态相近的板子的实现 |
| BSP\_Power\_Up | NO | 冷启动与唤醒后调用 |
| BSP\_IO\_Power\_Down | NO | 睡眠前调用 |
| BSP\_LCD\_Reset | NO |  |
| BSP\_LCD\_PowerUp | NO | 屏幕上电时调用 |
| BSP\_LCD\_PowerDown | NO | 屏幕断电时调用 |
| BSP\_TP\_Reset | NO |  |
| BSP\_TP\_PowerUp | NO | 触控上电时调用 |
| BSP\_TP\_PowerDown | NO | 触控断电时调用 |
| HAL\_MspInit | NO | 被HAL\_PreInit调用，虚函数实现为调用BSP\_IO\_Init |
| HAL\_PostMspInit | NO |  |
| BSP\_IO\_Init | NO | 默认被HAL\_MspInit调用， |
| BSP\_PIN\_Init | NO | 由BSP\_IO\_Init调用，IO配置函数 |
|  |  |  |


## 应用程序自定义驱动接口

如果同一块板子的不同应用程序需要实现不同的`HAL_MspInit`功能，可以将`HAL_MspInit`的实现置于应用程序目录下，否则可以放在板子目录下。

| **函数名** | **必选** | **说明** |
| --- | --- | --- |
| HAL\_MspInit | NO |  |
| HAL\_PostMspInit | NO |  |


