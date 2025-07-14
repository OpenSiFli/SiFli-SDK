# PM Example
Source path: example\rt_device\pm
## Supported Platforms
The example can run on the following development boards.
* sf32lb56-devkit-lcd_n16r12n1
* eh-lb551
* sf32lb52-lcd_n16r8
* sf32lb58-lcd_n16r64n4
## Overview
* Demonstration of entering low power mode and waking up PM project under RT-Thread operating system

## Example Usage
### Compilation and Programming
#### sf32lb56-devkit-lcd_n16r12n1 compilation method
The 56 platform is configured by default for Standby Sleep mode<br>
Compilation command<br>
```c
> scons --board=sf32lb56-devkit-lcd_n16r12n1 -j8
```
* Switch to the example `project/build_xx` directory and run `uart_download.bat`, select the port as prompted (the serial port with print after pulling boot pin high) to download:
```shell
> build_sf32lb56-devkit-lcd_n16r12n1_hcpu\uart_download.bat

     Uart Download

please input the serial port num:5
```
#### eh-lb551 compilation method
The 56 platform is configured by default for Standby Sleep mode<br>
Compilation command<br>
```c
> scons --board=eh-lb551 -j8
```
Confirm that Jlink is properly connected and can communicate successfully, as follows<br>
![alt text](assets/551-jlink.png)
* Switch to the example `project/build_xx` directory. The 551 example currently only supports Jlink download, run `download.bat` to download
```shell
> build_eh-lb551_hcpu\download.bat
```
#### sf32lb52-lcd_n16r8 compilation method
Configure menuconfig for Deep Sleep mode<br>
```shell
menuconfig
```
![alt text](assets/sf32lb52-lcd_n16r8-menuconfig.png)
The 52 platform recommends using Deep Sleep mode. Here change to Enable Deep Mode, save and exit<br>
```
 → RTOS → RT-Thread Components → Device Drivers → Using Power Management device driverselect PM Mode
```
Compilation command<br>
```shell
> scons --board=sf32lb52-lcd_n16r8 -j8
```
* Switch to the example `project/build_xx` directory and run `uart_download.bat`, select the port as prompted to download:
```shell
> build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat

     Uart Download

please input the serial port num:5
```
#### sf32lb58-lcd_n16r64n4 compilation method
The 56 platform is configured by default for Standby Sleep mode<br>
Compilation command<br>
```shell
> scons --board=sf32lb58-lcd_n16r64n4 -j8
```
* Switch to the example `project/build_sf32lb58-lcd_n16r64n4_hcpu` directory and run `download.bat`, here demonstrating download via Jlink:<br>
```shell
> build_sf32lb58-lcd_n16r64n4_hcpu\download.bat

```
* sf32lb58-lcd_n16r64n4 board Jlink connection method is shown below:<br>
![alt text](assets/sf32lb58-lcd_n16r64n4-jlink.png)<br>
### Hardware Connection
#### sf32lb56-devkit-lcd_n16r12n1
Use button PB32 on the board as wakeup PIN0`#WKUP_PIN0`, pressing the button changes the level high to wake up Hcpu
#### eh-lb551
Button KEY1`PB48` on the board serves as Lcpu's wakeup PIN5`#WKUP_PIN5`, pressing the button changes the level low to wake up Lcpu<br>
#### sf32lb52-lcd_n16r8
Use button PA34 on the board as wakeup PIN10`#WKUP_PIN10`, pressing the button changes the level high to wake up Hcpu
#### sf32lb58-lcd_n16r64n4
Use button PB54 on the board as wakeup PIN0`#WKUP_PIN0`, pressing the button changes the level high to wake up Hcpu
### Example Output Results
#### sf32lb56-devkit-lcd_n16r12n1
* 566 hcpu log, entering standby
```
    Serial:c2,Chip:3,Package:1,Rev:2  Reason:00000000
    Serial PowerOnMOde:0 rtc_record:00000000
     \ | /
    - SiFli Corporation
     / | \     build on Apr 14 2025, 2.3.5 build aea79fd6
     2020 - 2022 Copyright by SiFli team
    [32;22m[I/FAL] Flash Abstraction Layer (V0.5.99) initialize success.[0m
    [32;22m[I/FAL] The FAL MTD NOR device (dfu) created successfully[0m
    [32;22m[I/FAL] The FAL MTD NOR device (ble) created successfully[0m
    [1701] I/drv.audprc main: init 00 ADC_PATH_CFG0 0x924
    [1723] I/drv.audprc main: HAL_AUDPRC_Init res 0
    [1744] I/drv.audcodec main: HAL_AUDCODEC_Init res 0
    [1765] I/TOUCH main: Regist touch screen driver, probe=1401f50d 
    call par CFG1(35bb)
    fc 9, xtal 2000, pll 2064
    call par CFG1(35bb)
    fc 7, xtal 2000, pll 1656
    mbox_stack:20007df0,1024
    proc_stack:200081f0,4096
    [1954] I/mw.sys ds_proc: turn on lcpu
    [139193] I/mw.sys ds_proc: rf ful ver:1.1.9_2841,calen:0xff
    hcpu main!!!
    msh >
    app_wakeup!
    SF32LB56X WER:0x1c4,WSR:0x0,CR1:0x80000002
    [pm]S:4,237566
    [pm]W:401728
    [pm]WSR:0x4
    hcpu timer wakeup!!!
    [pm]S:4,401761
    [pm]W:447328
    [pm]WSR:0x100
    hcpu timer wakeup!!!
```
* 566 lcpu log(Lcpu project path is ../lcpu)
```
     \ | /
    - SiFli Corporation
     / | \     build on Jun 27 2022, 1.8.0 build cc1e38
     2020 - 2022 Copyright by SiFli team
    proc_stack:20403638,2048
    lcpu main!!!
    msh >
    [pm]S:4,237763
    [pm]W:401547
    [pm]WSR:0x60
    [pm]S:4,401801
    [pm]W:447147
    [pm]WSR:0x60
```
#### eh-lb551
* 551 hcpu entering standby log
```
    Serial:c2,Chip:1,Package:0,Rev:80  Reason:00000000
    Serial PowerOnMOde:0 rtc_record:00000000
     \ | /
    - SiFli Corporation
     / | \     build on Apr 15 2025, 2.3.5 build aea79fd6
     2020 - 2022 Copyright by SiFli team
    [32;22m[I/FAL] Flash Abstraction Layer (V0.5.99) initialize success.[0m
    [32;22m[I/FAL] The FAL MTD NOR device (dfu) created successfully[0m
    [32;22m[I/FAL] The FAL MTD NOR device (ble) created successfully[0m
    [299] I/drv.rtc main: PSCLR=0x80000100 DivAI=128 DivAF=0 B=256
    [306] I/drv.rtc main: RC10K Freq=9kHz, cycle=985648, rtc ppm=0
    [314] I/drv.rtc main: DIVA=38, DIVA_FRA=754, DIVB=256
    [320] I/drv.rtc main: Init RTC, wake = 0
    [371] I/TOUCH main: Regist touch screen driver, probe=1003ef3d 
    mbox_stack:20000000,1024
    proc_stack:20000400,4096
    [389] I/mw.sys ds_proc: turn on lcpu
    [535] I/mw.sys ds_proc: rf ful ver:null,calen:0xff
    hcpu main!!!
    msh >
    [pm]S:4,29758
    [pm]W:78662
    [pm]WSR:0x2
    hcpu timer wakeup!!!
    [pm]S:4,78672
```
* 551 log for entering hibernate after booting when Hcpu hasn't slept yet using serial command `sleep down` and button wakeup
Since eh-lb551 board doesn't have 32768 crystal by default, the macro `LXT_DISABLE` is enabled, so after entering Hibernate and waking up with button, there is no PM_HIBERNATE_BOOT flag
```
   Serial:c2,Chip:1,Package:0,Rev:80  Reason:00000000
   Serial PowerOnMOde:0 rtc_record:00000000
    \ | /
   - SiFli Corporation
    / | \     build on Apr 15 2025, 2.3.5 build aea79fd6
    2020 - 2022 Copyright by SiFli team
   [32;22m[I/FAL] Flash Abstraction Layer (V0.5.99) initialize success.[0m
   [32;22m[I/FAL] The FAL MTD NOR device (dfu) created successfully[0m
   [32;22m[I/FAL] The FAL MTD NOR device (ble) created successfully[0m
   [299] I/drv.rtc main: PSCLR=0x80000100 DivAI=128 DivAF=0 B=256
   [306] I/drv.rtc main: RC10K Freq=9kHz, cycle=985621, rtc ppm=0
   [314] I/drv.rtc main: DIVA=38, DIVA_FRA=771, DIVB=256
   [320] I/drv.rtc main: Init RTC, wake = 0
   [371] I/TOUCH main: Regist touch screen driver, probe=1003ef3d 
   mbox_stack:20000000,1024
   proc_stack:20000400,4096
   [389] I/mw.sys ds_proc: turn on lcpu
   [534] I/mw.sys ds_proc: rf ful ver:null,calen:0xff
   hcpu main!!!
   msh >
TX:sleep down
   sleep down
   SF32LB55X entry_hibernate
   SF32LB55X CR:0x180000,WER:0x200
   Serial:c2,Chip:1,Package:0,Rev:80  Reason:00000000
   Serial PowerOnMOde:3 rtc_record:00000000
   [32;22m[I/FAL] Flash Abstraction Layer (V0.5.99) initialize success.[0m
   [32;22m[I/FAL] The FAL MTD NOR device (dfu) created successfully[0m
   [32;22m[I/FAL] The FAL MTD NOR device (ble) created successfully[0m
   [285] I/drv.rtc main: PSCLR=0x260c0d00 DivAI=38 DivAF=771 B=256
   [293] I/drv.rtc main: RC10K Freq=9kHz, cycle=985733, rtc ppm=0
   [300] I/drv.rtc main: DIVA=38, DIVA_FRA=701, DIVB=256
   [306] I/drv.rtc main: Init RTC, wake = 1
   [313] I/TOUCH main: Regist touch screen driver, probe=1003ef3d 
   mbox_stack:20000000,1024
   proc_stack:20000400,4096
   [332] I/mw.sys ds_proc: turn on lcpu
   [477] I/mw.sys ds_proc: rf ful ver:null,calen:0xff
   hcpu main!!!
   msh >
```
* 551 lcpu log including entering standby, entering hibernate and button wakeup log(Lcpu project path is ../lcpu)
```
    \ | /
   - SiFli Corporation
    / | \     build Jul 21 2021, 0.9.5.6
    2020 - 2021 Copyright by SiFli team
   proc_stack:20105ba4,2048
   lcpu main!!!
   msh >
   [pm]S:4,29794
   [pm]W:78613
   [pm]WSR:0x810
   [pm]S:4,78693
   [pm]W:89639
   [pm]WSR:0x410
   Lcpu wake_key_handle!!!
   [pm]S:4,89655
   [pm]W:90930
   [pm]WSR:0x410
   Lcpu wake_key_handle!!!
   [pm]S:4,90946
```
#### sf32lb52-lcd_n16r8
* sf32lb52-lcd_n16r8 running log output
```
   SFBL
   Serial:c2,Chip:4,Package:3,Rev:3  Reason:00000000
   Serial PowerOnMOde:0 rtc_record:00000000
    \ | /
   - SiFli Corporation
    / | \     build on Jan  8 2025, 2.2.4 build 00000000
    2020 - 2022 Copyright by SiFli team
   [32;22m[I/FAL] Flash Abstraction Layer (V0.5.99) initialize success.[0m
   [32;22m[I/FAL] The FAL MTD NOR device (dfu) created successfully[0m
   [32;22m[I/FAL] The FAL MTD NOR device (ble) created successfully[0m
   [573] I/drv.rtc main: PSCLR=0x80000100 DivAI=128 DivAF=0 B=256
   [599] I/drv.rtc main: RTC use LXT RTC_CR=00000001
   [619] I/drv.rtc main: Init RTC, wake = 1
   [645] I/drv.audprc main: init 00 ADC_PATH_CFG0 0x606
   [665] I/drv.audprc main: HAL_AUDPRC_Init res 0
   [686] I/drv.audcodec main: HAL_AUDCODEC_Init res 0
   [706] I/TOUCH main: Regist touch screen driver, probe=1203e225 
   call par CFG1(35bb)
   fc 9, xtal 2000, pll 2054
   call par CFG1(35bb)
   fc 9, xtal 2000, pll 2053
   mbox_stack:200067f8,1024
   proc_stack:20006bf8,4096
   [937] I/mw.sys ds_proc: turn on lcpu
   [36981] I/mw.sys ds_proc: rf ful ver:1.1.13_2859,calen:0xf
   hcpu main!!!
   msh >
   [pm]S:3,135137
   [pm]W:298069
   [pm]WSR:0x40002
   gpio_wakeup_handler!
   hcpu timer wakeup!!!
   [pm]S:3,298998
   [pm]W:462858
   [pm]WSR:0x4
   hcpu timer wakeup!!!
   [pm]S:3,462881
```
* sf32lb52-lcd_n16r8 entering hibernate and button wakeup log
```
    SFBL
    Serial:c2,Chip:4,Package:3,Rev:3  Reason:00000000
    Serial PowerOnMOde:0 rtc_record:00000000
     \ | /
    - SiFli Corporation
     / | \     build on Jan  8 2025, 2.2.4 build 00000000
     2020 - 2022 Copyright by SiFli team
    [32;22m[I/FAL] Flash Abstraction Layer (V0.5.99) initialize success.[0m
    [32;22m[I/FAL] The FAL MTD NOR device (dfu) created successfully[0m
    [32;22m[I/FAL] The FAL MTD NOR device (ble) created successfully[0m
    [585] I/drv.rtc main: PSCLR=0x80000100 DivAI=128 DivAF=0 B=256
    [611] I/drv.rtc main: RTC use LXT RTC_CR=00000001
    [631] I/drv.rtc main: Init RTC, wake = 1
    [658] I/drv.audprc main: init 00 ADC_PATH_CFG0 0x606
    [678] I/drv.audprc main: HAL_AUDPRC_Init res 0
    [699] I/drv.audcodec main: HAL_AUDCODEC_Init res 0
    [719] I/TOUCH main: Regist touch screen driver, probe=1203e245 
    call par CFG1(35bb)
    fc 9, xtal 2000, pll 2056
    call par CFG1(35bb)
    fc 7, xtal 2000, pll 1646
    mbox_stack:200067f8,1024
    proc_stack:20006bf8,4096
    [950] I/mw.sys ds_proc: turn on lcpu
    [36961] I/mw.sys ds_proc: rf ful ver:1.1.13_2859,calen:0xf
    hcpu main!!!
    msh >
 TX:sleep down
    sleep down
    SF32LB52X entry_hibernate
    SF32LB52X CR:0xa801,WER:0x8c
    SFBL
    Serial:c2,Chip:4,Package:3,Rev:3  Reason:00000000
    Serial PowerOnMOde:2 rtc_record:00000000
    [32;22m[I/FAL] Flash Abstraction Layer (V0.5.99) initialize success.[0m
    [32;22m[I/FAL] The FAL MTD NOR device (dfu) created successfully[0m
    [32;22m[I/FAL] The FAL MTD NOR device (ble) created successfully[0m
    [505] I/drv.rtc main: PSCLR=0x80000100 DivAI=128 DivAF=0 B=256
    [530] I/drv.rtc main: RTC use LXT RTC_CR=00000001
    [550] I/drv.rtc main: Init RTC, wake = 1
    [576] I/drv.audprc main: init 00 ADC_PATH_CFG0 0x606
    [597] I/drv.audprc main: HAL_AUDPRC_Init res 0
    [617] I/drv.audcodec main: HAL_AUDCODEC_Init res 0
    [638] I/TOUCH main: Regist touch screen driver, probe=1203e245 
    call par CFG1(35bb)
    fc 9, xtal 2000, pll 2058
    call par CFG1(35bb)
    fc 7, xtal 2000, pll 1645
    mbox_stack:200067f8,1024
    proc_stack:20006bf8,4096
    [869] I/mw.sys ds_proc: turn on lcpu
    [36944] I/mw.sys ds_proc: rf ful ver:1.1.13_2859,calen:0xf
    hcpu main!!!
    msh >
    boot from hibernate!!!
    [pm]S:3,135216
```
#### sf32lb58-lcd_n16r64n4
* sf32lb58-lcd_n16r64n4 hcpu running log output, including entering standby, PB54 button wakeup log
```
 Serial:c2,Chip:2,Package:0,Rev:1  Reason:00000000
 Serial PowerOnMOde:0 rtc_record:00000000
  \ | /
 - SiFli Corporation
  / | \     build on Jan 24 2025, 2.3.0 build 00000000
  2020 - 2022 Copyright by SiFli team
 [32;22m[I/FAL] Flash Abstraction Layer (V0.5.99) initialize success.[0m
 [31;22m[E/FAL] (fal_mtd_nor_device_create:496) Error: the flash device name (flash3) is not found.[0m
 [31;22m[E/FAL] (fal_mtd_nor_device_create:496) Error: the flash device name (flash3) is not found.[0m
 [D/USBD] No class register on usb device
 [1191] I/drv.rtc main: PSCLR=0x80000100 DivAI=128 DivAF=0 B=256
 [1215] I/drv.rtc main: RTC use LXT RTC_CR=00000000
 [1235] I/drv.rtc main: Init RTC, wake = 0
 [1397] I/drv.audprc main: init 00 ADC_PATH_CFG0 0x924
 [1418] I/drv.audprc main: HAL_AUDPRC_Init res 0
 [1437] I/drv.audcodec main: HAL_AUDCODEC_Init res 0
 [1458] I/TOUCH main: Regist touch screen driver, probe=10023e9d 
 call par CFG1(35bb)
 fc 11, xtal 2000, pll 1985
 call par CFG1(35bb)
 fc 9, xtal 2000, pll 1984
 mbox_stack:20000000,1024
 proc_stack:20000400,4096
 [1639] I/mw.sys ds_proc: turn on lcpu
 [47036] I/mw.sys ds_proc: rf ful ver:1.0.0.1,calen:0xff
 hcpu main!!!
 msh >
 SF32LB58X AON CR1:0x80000000,CR2:0x0,WER:0x1c4
 [pm]S:4,145384
 [pm]W:198520
 [pm]WSR:0x100
 gpio_wakeup_handler!
 [pm]S:4,205008
 [pm]W:309478
 [pm]WSR:0x4
 hcpu timer wakeup!!!
 [pm]S:4,309501
```
* sf32lb58-lcd_n16r64n4 lcpu running log output, including entering standby, PB54 button wakeup log
```
 \ | /
- SiFli Corporation
 / | \     build on Mar 20 2022, 1.2.0 build dbebac
 2020 - 2022 Copyright by SiFli team
proc_stack:2040759c,2048
lcpu main!!!
msh >
[pm]S:4,145530
[pm]W:177325
[pm]WSR:0x60
[pm]S:4,185126
[pm]W:309339
[pm]WSR:0x60
[pm]S:4,309537
```
* sf32lb58-lcd_n16r64n4 hcpu receiving command sleep down via serial port to enter hibernate and wake up with PB54 button log
```
    Serial:c2,Chip:2,Package:0,Rev:1  Reason:00000000
    Serial PowerOnMOde:0 rtc_record:00000000
     \ | /
    - SiFli Corporation
     / | \     build on Jan 24 2025, 2.3.0 build 00000000
     2020 - 2022 Copyright by SiFli team
    [32;22m[I/FAL] Flash Abstraction Layer (V0.5.99) initialize success.[0m
    [31;22m[E/FAL] (fal_mtd_nor_device_create:496) Error: the flash device name (flash3) is not found.[0m
    [31;22m[E/FAL] (fal_mtd_nor_device_create:496) Error: the flash device name (flash3) is not found.[0m
    [D/USBD] No class register on usb device
    [1192] I/drv.rtc main: PSCLR=0x80000100 DivAI=128 DivAF=0 B=256
    [1216] I/drv.rtc main: RTC use LXT RTC_CR=00000000
    [1235] I/drv.rtc main: Init RTC, wake = 0
    [1397] I/drv.audprc main: init 00 ADC_PATH_CFG0 0x924
    [1418] I/drv.audprc main: HAL_AUDPRC_Init res 0
    [1437] I/drv.audcodec main: HAL_AUDCODEC_Init res 0
    [1458] I/TOUCH main: Regist touch screen driver, probe=10023e9d 
    call par CFG1(35bb)
    fc 11, xtal 2000, pll 1986
    call par CFG1(35bb)
    fc 9, xtal 2000, pll 1986
    mbox_stack:20000000,1024
    proc_stack:20000400,4096
    [1639] I/mw.sys ds_proc: turn on lcpu
    [47034] I/mw.sys ds_proc: rf ful ver:1.0.0.1,calen:0xff
    hcpu main!!!
    msh >
 TX:sleep down
    sleep down
    SF32LB58X entry_hibernate
    SF32LB5XX CR:0x4001,WER:0x8,WKUP_CNT:0x10001,PBR0R:0x800002a7
    Serial:c2,Chip:2,Package:0,Rev:1  Reason:00000000
    Serial PowerOnMOde:2 rtc_record:00000000
    [32;22m[I/FAL] Flash Abstraction Layer (V0.5.99) initialize success.[0m
    [31;22m[E/FAL] (fal_mtd_nor_device_create:496) Error: the flash device name (flash3) is not found.[0m
    [31;22m[E/FAL] (fal_mtd_nor_device_create:496) Error: the flash device name (flash3) is not found.[0m
    [D/USBD] No class register on usb device
    [286] I/drv.rtc main: PSCLR=0x80000100 DivAI=128 DivAF=0 B=256
    [310] I/drv.rtc main: RTC use LXT RTC_CR=00000000
    [329] I/drv.rtc main: Init RTC, wake = 1
    [357] I/drv.audprc main: init 00 ADC_PATH_CFG0 0x924
    [377] I/drv.audprc main: HAL_AUDPRC_Init res 0
    [396] I/drv.audcodec main: HAL_AUDCODEC_Init res 0
    [416] I/TOUCH main: Regist touch screen driver, probe=10023e9d 
    call par CFG1(35bb)
    fc 11, xtal 2000, pll 1984
    call par CFG1(35bb)
    fc 9, xtal 2000, pll 1985
    mbox_stack:20000000,1024
    proc_stack:20000400,4096
    [598] I/mw.sys ds_proc: turn on lcpu
    [46091] I/mw.sys ds_proc: rf ful ver:1.0.0.1,calen:0xff
    hcpu main!!!
    msh >
    SF32LB58X AON CR1:0x80000000,CR2:0x0,WER:0x1c4
    boot from hibernate!!!
    [pm]S:4,144433
    [pm]W:308543
    [pm]WSR:0x4
    hcpu timer wakeup!!!
    [pm]S:4,308566
```
### Sleep Flow
#### sf32lb56-devkit-lcd_n16r12n1
* hcpu sleep wakeup<br>
When hcpu enters the idle thread and determines whether sleep conditions are met, it will sleep and wake up according to the following flow<br>
rt_thread_idle_entry->rt_system_power_manager->_pm_enter_sleep->'pm->ops->sleep(pm, mode);->sifli_sleep -> log print [pm]S:4,11620140 -> RT_DEVICE_CTRL_SUSPEND device suspend -> 
sifli_standby_handler ->BSP_IO_Power_Down-> assembly WFI enter standby-> timer or IO wakeup -> function
SystemInitFromStandby -> HAL_Init -> BSP_IO_Init-> restore_context-> PC pointer set to continue running from instruction after WFI in sifli_standby_handler function -> BSP_Power_Up-> execute RTT device RT_DEVICE_CTRL_RESUME device resume function -> log print [pm]W:11620520 -> log print [pm]WSR:0x80<br>
* LCPU sleep wakeup<br>
Lcpu sleep wakeup flow is basically the same as hcpu standby flow, difference: sifli_standby_handler-> sifli_standby_handler_core->IO configuration function during sleep BSP_IO_Power_Down->memory sleep function soc_power_down-> assembly WFI enter standby->timer or IO wakeup -> function SystemPowerOnModeInit ->SystemPowerOnInitLCPU-> HAL_Init-> BSP_IO_Init-> restore_context-> set to continue execution after WFI instruction -> memory exit sleep function soc_power_up-> IO configuration function after sleep BSP_Power_Up-> execute RTT device RT_DEVICE_CTRL_RESUME device resume function -> log print [pm]W:11620520 -> log print [pm]WSR:0x80<br>

#### eh-lb551
* hcpu sleep wakeup<br>
55 series has the same standby flow as 56 series, difference is in the wakeup function<br>
timer or IO wakeup -> function
SystemPowerOnModeInit -> HAL_Init -> BSP_IO_Init->restore_context-> PC pointer set to continue running from instruction after WFI in sifli_standby_handler function -> BSP_Power_Up-> execute RTT device RT_DEVICE_CTRL_RESUME device resume function -> log print [pm]W:11620520 -> log print [pm]WSR:0x80<br>
* LCPU sleep wakeup<br>
Lcpu wakeup flow is the same as 56 series
#### sf32lb52-lcd_n16r8
* hcpu sleep wakeup<br>
52 series differs from 56 series standby flow in that it enters `sifli_deep_handler();` standby function, and lacks peripheral SUSPEND/RESUME and context recovery process, making sleep wakeup faster<br>
sifli_sleep -> log print [pm]S:3,11620140 -> sifli_deep_handler ->BSP_IO_Power_Down-> assembly WFI enter deep-> timer or IO wakeup -> continue running from instruction after WFI -> BSP_Power_Up-> log print
[pm]W:11620520 -> log print [pm]WSR:0x80<br>
* LCPU sleep wakeup<br>
52 series Lcpu code is not open for modification, this part doesn't need consideration<br>
#### sf32lb58-lcd_n16r64n4
* hcpu sleep wakeup<br>
58 series has the same standby flow as 56 series<br>
* LCPU sleep wakeup<br>
Lcpu wakeup flow is the same as 56 series<br>
### Hibernate Flow
* Entering Hibernate<br>
Entering Hibernate flow is relatively simple, all call function `HAL_PMU_EnterHibernate();`. Before the chip sleeps, it needs to configure the PMU wakeup PIN and wakeup level after Hibernate. For 55 series MCU, all wakeup PINs are floating input under hibernate,
external circuits need to provide pull-up/down states, no other processing needed.
For 58, 56, 52 series MCU, since wakeup PINs have an additional set of PMU pull-up/down system effective under Hibernate, to avoid leakage, you can use function `HAL_PIN_Set` to configure wakeup PIN pull-up/down.
For 52 series MCU, since it has 3 built-in LDOs, you need to check your hardware connection to consider whether to configure shutdown through function `HAL_PMU_ConfigPeriLdo`<br>
The `hwp_pmuc->WKUP_CNT` register can configure how long the external signal needs to persist under hibernate to wake up (only for 58, 56, 52 series MCU);<br>
**Note:**<br>
For 55 series MCU, since each wakeup pin can be enabled individually, configuring Hibernate wakeup source only needs to call function `HAL_PMU_EnablePinWakeup` to enable the corresponding wakeup pin;<br>
For 58, 56, 52 series MCU, only 2 wakeup sources `pin0`, `pin1` are allowed simultaneously. Here an additional step is needed, call function `HAL_PMU_SelectWakeupPin` to configure which specific wakeup pin corresponds to pin0 and pin1, corresponding register configuration refer to `hwp_pmuc->CR`;<br>
* Hibernate wakeup<br>
After pressing wakeup PIN to wake up from Hibernate, you can judge `(PM_HIBERNATE_BOOT == SystemPowerOnModeGet())` whether it is `hibernate boot` and the button press duration to determine whether to boot<br>
The log print `boot from hibernate!!!` can indicate boot from hibernate<br>
## Exception Diagnosis
### Wakeup Log Interpretation
```
   [pm]S:3,298998<br>
    S indicates entered sleep, 3 indicates sleep mode is PM_SLEEP_MODE_DEEP, 4 represents PM_SLEEP_MODE_STANDBY, 298998 is Gtimer timestamp<br>
   [pm]W:462858 <br>
   W indicates woken up, 462858 is Gtimer timestamp<br>
   [pm]WSR:0x4 <br>
   WSR is wakeup register, 0x04 is WSR register value, need to refer to specification to check which wakeup source<br>
```
### No PM[S] print
Reason 1:<br>
Confirm the following macros are enabled<br>
```
#define RT_USING_PM 1
#define PM_STANDBY_ENABLE 1 //#define PM_DEEP_ENABLE 1 //525 is Deep
#define BSP_USING_PM 1
#define BSP_PM_DEBUG 1
```
Reason 2:<br>
Software didn't enter `sifli_sleep` function<br>
When hcpu is not busy, it will enter idle thread, and when sleep conditions are met, it will enter sleep `sifli_sleep` function according to the following flow<br>
rt_thread_idle_entry->rt_system_power_manager->_pm_enter_sleep->'pm->ops->sleep(pm, mode);->sifli_sleep -> log print [pm]S:4,11620140 <br>
In _pm_enter_sleep function, it will judge whether sleep conditions are met, only enter sleep when met. For how each sleep condition is judged, refer to FAQ document low power section and low power development guide<br>
### No [pm]W print
Reason 1:<br>
If there is already PM[S] print, and there are timer wakeups, but no [pm]W print, usually there is a crash during sleep or wakeup process. Common reasons include adding prints (serial port is suspended and unusable at this time) in `BSP_IO_Power_Down`, `BSP_PIN_Init` function or peripheral `RT_DEVICE_CTRL_SUSPEND` message, or operating the other core's GPIO registers (when Hcpu is standby, Lcpu is allowed to sleep, and Lcpu cannot operate PA port);<br>
As shown below, the most common crash is operating PA port outside the macro `#ifdef SOC_BF0_HCPU` in the shared function `BSP_PIN_Init` for Hcpu and Lcpu. Since Lcpu cannot operate PA port, it causes crash after standby wakeup<br>
![alt text](assets/wakeup_hardfault.png)
Crashes at this time are difficult to debug, Jlink and serial port may no longer work. The usual method is to use instruction `__asm("B .")` to stop the CPU at places where MCU is confirmed to still run normally with Jlink or serial port, may also need to disable interrupts (if not disabled, OS is running and will switch to other threads).
If crash is suspected during sleep process, can stop after `[pm]S:3,298998` print. If crash is suspected after wakeup, hcpu can use `B .` to stop MCU at the beginning of `Reset_Handler` in `startup_bf0_hcpu.S`, Lcpu can use it in `startup_bf0_lcpu.S`, then use Jlink single-step to find the crash point;<br>
### No button wakeup after Standby
Reason 1:<br>
Hcpu can compare WER, CR register values of HPSYS_AON at this time, Lcpu can compare WER, CR register values of LPSYS_AON at this time, check if corresponding wakeup PIN and wakeup mode are configured correctly, below shows 52 specification as example<br>
![alt text](assets/525-HCPU-WER-REG.png)
![alt text](assets/525-HCPU-CR-REG.png)
**Note:**
There are many ways to read register values at this time, including printing register values, Jlink, serial port read/write, not repeated here, refer to FAQ development tools section<br>
### 55 series MCU Lcpu no log print
For 55 series MCU, to solve Lcpu RAM shortage, production chips upgraded from A0 to A3 version. A3 version chips have more code in ROM, thus freeing up more usable RAM space. In SDK code, need to configure corresponding to chip, as shown below rtconfig.h positioning flag for A3 version<br>
```c
#define BSP_LB55X_CHIP_ID 3
```
Chip version check can look at chip bottom marking, for example: TFA2XXXXX represents A2 version chip (using default configuration Lcpu will crash, no log output), default example is verified on TFA3XXXXX(A3) version chip;<br>
If changing to A2 version configuration, both Hcpu and Lcpu rtconfig.h need to modify `#define BSP_LB55X_CHIP_ID 2`
### No button wakeup after Hibernate
Reason 1:<br> 
The example adds finsh command to enter hibernate, Hcpu serial input `sleep down` can enter hibernate shutdown state<br>
After Hibernate shutdown, unlike Standby wakeup which can resume to original program, waking up from Hibernate is equivalent to cold start (but has PM_HIBERNATE_BOOT flag), wakeup PIN and level mode are controlled by PMU registers<br> 
Whether related configuration is effective, can print corresponding PMU `WER, CR` register values to compare, below shows 52 specification as example<br>
```c
rt_kprintf("SF32LB52X CR:0x%x,WER:0x%x\n",hwp_pmuc->CR,hwp_pmuc->WER);
```
![alt text](assets/525-PMU-CR-WER-REG.png)
Reason 2:<br> 
Configured wrong wakeup PIN or wakeup level, for example 55 series MCU, under Hibernate, only Lcpu wakeup PIN0-5 have specific wakeup function, specifically check 55 series user manual `PMUC WER` register configuration<br> 
For 58, 56, 52 series MCU, only two wakeup sources PIN0 and PIN1 are allowed simultaneously, each wakeup source can be assigned to any Hcpu/Lcpu wakeup PIN, specific configuration check user manual `PMUC CR` register configuration<br>
**Note:**<br> 
For 55 series MCU, when system is in hibernate mode, PA77-80 cannot be used for boot wakeup.<br>
For 55 series MCU, when system is in hibernate mode, PB43-48 can be used for boot wakeup.<br>
### Automatic restart immediately after Hibernate
Reason 1:<br> 
Wakeup PIN level configuration error, after entering Hibernate, external level conditions are met, wake up immediately<br> 
Reason 2:<br> 
After entering Hibernate, didn't disable interrupts and add `while(1);` wait. Since entering Hibernate takes time, without disabling interrupts and infinite loop, code will continue running

### No PM_HIBERNATE_BOOT flag after Hibernate button wakeup
The difference between hibernate mode and powerdown mode is that RTC 32768 clock is still running under hibernate mode,
but some customers for cost considerations don't install 32768 crystal, need to enable macro `LXT_DISABLE`, after enabling 32768 clock will not run
```c
#define LXT_DISABLE 1
```
In this case, entering hibernate mode, even if 32768 crystal is installed, actually there is no RTC clock running. After entering Hibernate and waking up with button, there will be no `PM_HIBERNATE_BOOT` flag. Boot wakeup in this example will not see log: `boot from hibernate!!!`

## Reference Documentation
* EH-SF32LB52X_Pin_config_V1.3.0_20231110.xlsx
* DS0052-SF32LB52x-Chip Technical Specification V0p3.pdf
* UM0052-SF32LB52x-User Manual V0p5.pdf
* UM0055-SF32LB55x-User Manual V0p1.pdf
* UM0056-SF32LB56x-User Manual V0p7.pdf
* AN0003-SF32LB55x-Low Power Development Guide V0p5.pdf
* SF32LB5XX-FAQ-V0.04.pdf
* [RT-Thread Official Website](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/pm/pm?id=pm-%e7%bb%84%e4%bb%b6%e4%bb%8b%e7%bb%8d)<br>
https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/pm/pm
## Update Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.2 |01/2025 |Initial version |
| | | |