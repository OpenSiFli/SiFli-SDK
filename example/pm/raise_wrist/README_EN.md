# Raise Wrist to Light Up Screen Example

Source code path: example/pm/raise_wrist

## Supported Platforms

The example can run on the following development boards:
* sf32lb52-lchspi-ulp

## Example Overview

* Time display application: Upon power-on, it displays the default time of 9:30:00 and automatically updates the time display every second.
* Intelligent power management: Automatically enters sleep mode after 10 seconds of inactivity. When the wrist is raised, the sensor detects this gesture to wake up the device and light up the screen to continue displaying time. When the wrist is lowered, the screen turns off and the device enters sleep mode.
  Inactivity detection: If a button is pressed or the screen is touched within 10 seconds, it's considered as user activity, and the time will be reset to the default value.
* Sensor data processing: Uses LSM6DSL sensor (accelerometer + gyroscope), continuously reads acceleration data with a 200ms sampling period, and processes the data through algorithms to implement gesture recognition (wrist raise/lower detection).

* Data Summary (using 200mAh battery capacity as an example)
|          |Wake-up Work    |Screen-off Sleep      |Daily Consumption      |Usable Days |
|:---      |:---            |:---                  |:---                   |:---        |
|100 times/day|6.3mAh      |9.7mAh               |16.0mAh                |12.5 days   |
|300 times/day|18.8mAh     |9.5mAh               |28.3mAh                |7.1 days    |
|500 times/day|31.4mAh     |9.3mAh               |40.7mAh                |4.9 days    |
|**Average Power Consumption in Two Modes**|**Screen-off (Low Power Mode):** 410uAh |**Screen-on (Working Mode):** 22.6mAh  |
## How to Use the Example

### Hardware Connection

Remove the VBAT_S and VBAT jumper caps, and use a power consumption testing tool to supply power through VBAT. Refer to the following diagram for specific hardware connections:
![alt text](assets/connect.png)

### Menuconfig Configuration

* Required configurations are enabled by default.

```c
menuconfig --board=board_name
```

1. Enable 6-axis sensor
   - Path: Select board peripherals → Sensor → 6D Sensor for Accelerator and Gyro
     - Enable: LSM6DSL
       - Macro switch: `CONFIG_ACC_USING_LSM6DSL`
       - Purpose: Enable the 6-axis sensor

2. Enable LVGL
   - Path: Third party packages
     - Enable: LittlevGL2RTT: The LittlevGl gui lib adapter RT-Thread
       - Macro switch: `CONFIG_PKG_USING_LITTLEVGL2RTT`
       - Purpose: Use LVGL and select version (default v8)

3. Enable button
   - Path: Sifli middleware
     - Enable: Enable button library
       - Macro switch: `CONFIG_USING_BUTTON_LIB`
       - Purpose: Enable button functionality

4. Enable low power mode
   - Path: Sifli middleware
     - Enable: Enable low power support
       - Macro switch: `CONFIG_BSP_USING_PM`
       - Purpose: Enable low power mode

5. Enable outputting low power related logs (disabled by default to reduce power consumption. Can be manually enabled in menuconfig for debugging)
   - Path: SiFli Middleware → Enable Low Power Support
     - Enable: Enable PM Debug
     - Macro switch: `CONFIG_BSP_PM_DEBUG`
     - Purpose: Output low power related logs

### Compilation and Flashing

#### Compiling with sf32lb52-lchspi-ulp_hcpu project code as an example

The 52 platform is configured to Deep Sleep mode by default<br>

* Compile:
  Switch to the example project directory and run the scons command to compile:

> scons --board=sf32lb52-lchspi-ulp -j8

* Flash:
  1. Switch to the example `project/build_xx` directory, run `uart_download.bat`, and select the port as prompted to download:
  2. Switch to the example project directory and run: build_sf32lb52-lchspi-ulp_hcpu\uart_download.bat

```c
build_sf32lb52-lchspi-ulp_hcpu\uart_download.bat

Uart Download

please input the serial port num:5
```

## Example Output Results

```c
09-17 19:12:41:285    SFBL
09-17 19:12:43:345    Serial:c2,Chip:4,Package:3,Rev:3  Reason:00000000
09-17 19:12:43:346    Serial PowerOnMOde:0 rtc_record:00000000
09-17 19:12:43:350     \ | /
09-17 19:12:43:351    - SiFli Corporation
09-17 19:12:43:354     / | \     build on Sep 12 2025, 2.4.0 build 3c988077
09-17 19:12:43:355     2020 - 2022 Copyright by SiFli team
09-17 19:12:43:356    mount /dev sucess
09-17 19:12:43:358    [1994] I/drv.rtc main: PSCLR=0x80000100 DivAI=128 DivAF=0 B=256
09-17 19:12:43:358    [2022] I/drv.rtc main: RTC use LXT RTC_CR=00000001
09-17 19:12:43:361    [2042] I/drv.rtc main: Init RTC, wake = 0
09-17 19:12:43:361    [2202] I/drv.audprc main: init 00 ADC_PATH_CFG0 0x606
09-17 19:12:43:362    [2224] I/drv.audprc main: HAL_AUDPRC_Init res 0
09-17 19:12:43:366    [2244] I/drv.audcodec main: HAL_AUDCODEC_Init res 0
09-17 19:12:43:367    [2265] I/TOUCH main: Regist touch screen driver, probe=120a5cc5 
09-17 19:12:43:369    call par CFG1(3313)
09-17 19:12:43:370    fc 9, xtal 2000, pll 2030
09-17 19:12:43:370    call par CFG1(3313)
09-17 19:12:43:372    fc 9, xtal 2000, pll 2029
09-17 19:12:43:373    cwm_enable_gesture 0 1 0
09-17 19:12:43:374    [2525] I/sensor main: rt_sensor init success
09-17 19:12:43:374    [2543] I/sensor main: rt_sensor init success
09-17 19:12:43:375    [2561] I/sensor main: rt_sensor init success
09-17 19:12:43:377    [2633] I/sensor.st.lsm6dsl main: sensor init success
09-17 19:12:43:377    Accelerometer device opened: ODR=52Hz, Mode=FIFO
09-17 19:12:43:380    Wrist detect thread started
09-17 19:12:43:380    [2729] I/drv.lcd main: [NONE] -> [OPENING]
09-17 19:12:43:384    [2748] I/drv.lcd lcd_task: open
09-17 19:12:43:385    [2761] I/drv.epic lcd_task: drv_gpu opened.
09-17 19:12:43:385    [2778] I/drv.lcd lcd_task: HW open
09-17 19:12:43:386    [2813] I/drv.lcd lcd_task: Try registered LCD driver...
09-17 19:12:43:395    msh />
09-17 19:12:43:445    CO5300_ReadID 0x331100 
09-17 19:12:43:450    [5141] I/co5300 lcd_task: LCD module use CO5300 IC 
09-17 19:12:43:635    CO5300_ReadID 0x331100 
09-17 19:12:43:635    [11404] I/co5300 lcd_task: LCD module use CO5300 IC 
09-17 19:12:43:639    [11425] I/drv.lcd lcd_task: Found lcd co5300 id:331100h
09-17 19:12:43:640    [11447] I/drv.lcd lcd_task: HW open done.
09-17 19:12:43:642    [11463] I/drv.lcd lcd_task: [OPENING] -> [INITIALIZED]
09-17 19:12:43:643    [11483] I/drv.lcd lcd_task: open done.
09-17 19:12:43:645    [11504] I/drv.lcd_fb main: drv_lcd_fb_init
09-17 19:12:43:650    [11523] I/drv.lcd_fb main: drv_lcd_fb_init done.
09-17 19:12:43:652    [11631] I/TOUCH main: Open
09-17 19:12:43:653    [11648] I/TOUCH tp_init: Find touch screen driver...
09-17 19:12:43:653    [11668] I/TOUCH tp_init: Probe 120a5cc5
09-17 19:12:43:655    [11686] I/TOUCH tp_init: touch screen found driver  20028338, ft6146
09-17 19:12:43:656    [11714] I/TOUCH main: Opened.
09-17 19:12:43:657    [11727] I/LVGL main: [littlevgl2rtt] Welcome to the littlevgl2rtt lib.
09-17 19:12:43:658    Button initialized successfully
09-17 19:12:43:695    [13319] I/drv.lcd lcd_task: Auto turn on display.
09-17 19:12:43:698    [13340] I/drv.lcd lcd_task: set brightness 50
09-17 19:12:43:700    [13359] I/drv.lcd lcd_task: display on
09-17 19:12:43:701    [13374] I/drv.lcd lcd_task: [INITIALIZED] -> [ON]
09-17 19:12:43:730    [14479] E/drv.ft6146 tp_init: ft6146 id_H=64
09-17 19:12:43:731    [14502] E/drv.ft6146 tp_init: ft6146 id_L=56
09-17 19:12:49:097    cwm_ap_sensorListen called. sensorType=9, index=0, timestamp_ns=0
09-17 19:12:49:098    watch handup: 1.0000, 1.0000, 1.0000
09-17 19:12:49:103    set-- gesture:1, counter:1
09-17 19:12:49:104    >>>Raise wrist to light up screen<<<
09-17 19:12:44:178    824 /n
09-17 19:12:44:695    1340 /n
09-17 19:12:45:214    1856 /n
09-17 19:12:45:726    2372 /n
09-17 19:12:46:245    2888 /n
09-17 19:12:51:549    cwm_ap_sensorListen called. sensorType=9, index=0, timestamp_ns=0
09-17 19:12:51:551    watch handup: 2.0000, 2.0000, 2.0000
09-17 19:12:51:552    set-- gesture:2, counter:2
09-17 19:12:51:553    >>>Lower wrist to turn off screen<<<
09-17 19:12:51:553    Entering sleep mode...
09-17 19:12:51:554    [270793] I/drv.lcd lcd_task: Power off
09-17 19:12:51:559    [270808] I/drv.epic lcd_task: drv_gpu closed.
09-17 19:12:51:561    [270825] I/drv.lcd lcd_task: display off
09-17 19:12:51:561    [270842] I/drv.lcd lcd_task: [ON] -> [OFF]
09-17 19:12:51:562    [270859] I/drv.lcd lcd_task: HW close
09-17 19:12:51:564    [270875] I/drv.lcd lcd_task: HW close done.
09-17 19:12:51:564    [270891] I/drv.lcd lcd_task: Power off done
09-17 19:12:51:565    [270914] E/drv.i2c tpread: bus err:1, xfer:0/2, i2c_stat:20, i2c_errcode=21
09-17 19:12:51:565    [270947] E/drv.i2c tpread: reset and send 9 clks
09-17 19:12:51:567    [pm]S:3,271222
09-17 19:12:51:738    [pm]W:276898
09-17 19:12:51:739    [pm]WSR:0x4
09-17 19:12:51:759    [pm]S:3,277554
09-17 19:12:51:916    [pm]W:282706
09-17 19:12:51:917    [pm]WSR:0x4
09-17 19:12:51:961    [pm]S:3,284221
09-17 19:12:52:143    [pm]W:290125
09-17 19:12:52:144    [pm]WSR:0x4
09-17 19:12:53:797    cwm_ap_sensorListen called. sensorType=9, index=0, timestamp_ns=0
09-17 19:12:53:798    watch handup: 1.0000, 1.0000, 3.0000
09-17 19:12:53:800    set-- gesture:1, counter:3
09-17 19:12:53:804    >>>Raise wrist to light up screen<<<
09-17 19:12:53:820    Waking up system...
09-17 19:12:53:820    [344456] I/drv.lcd lcd_task: Power on
09-17 19:12:53:823    [344472] I/drv.epic lcd_task: drv_gpu opened.
09-17 19:12:53:823    [344490] I/drv.lcd lcd_task: HW open
09-17 19:12:53:826    [344525] I/drv.lcd lcd_task: Init LCD co5300
09-17 19:12:53:871    CO5300_ReadID 0x331100 
09-17 19:12:53:874    [346830] I/co5300 lcd_task: LCD module use CO5300 IC 
09-17 19:12:54:063    [353081] I/drv.lcd lcd_task: HW open done.
09-17 19:12:54:064    [353098] I/drv.lcd lcd_task: [OFF] -> [INITIALIZED]
09-17 19:12:54:068    [353118] I/drv.lcd lcd_task: Power on done.
09-17 19:12:54:069    [353135] I/drv.lcd lcd_task: set brightness 100
09-17 19:12:54:069    [353154] I/drv.lcd lcd_task: display on
09-17 19:12:54:070    [353169] I/drv.lcd lcd_task: [INITIALIZED] -> [ON]
09-17 19:12:54:070    Wakeup: reset inactive time to 0 ms
09-17 19:12:54:511    433 /n
09-17 19:12:55:028    954 /n
09-17 19:12:55:546    1470 /n
09-17 19:12:56:064    1986 /n
09-17 19:12:56:577    2502 /n
09-17 19:12:57:094    3018 /n
09-17 19:12:57:614    3534 /n
09-17 19:12:58:128    4050 /n
09-17 19:12:58:643    4566 /n
09-17 19:12:59:160    5082 /n
09-17 19:12:59:674    5598 /n
09-17 19:13:00:197    6114 /n
09-17 19:13:00:709    6630 /n
09-17 19:13:01:222    7146 /n
09-17 19:13:01:738    7662 /n
09-17 19:13:02:256    8178 /n
09-17 19:13:02:773    8694 /n
09-17 19:13:03:295    9210 /n
09-17 19:13:03:811    9726 /n
09-17 19:13:04:313    10242 /nEntering sleep mode...
09-17 19:13:04:320    [688987] I/drv.lcd lcd_task: Power off
09-17 19:13:04:321    [689002] I/drv.epic lcd_task: drv_gpu closed.
09-17 19:13:04:322    [689020] I/drv.lcd lcd_task: display off
09-17 19:13:04:323    [689036] I/drv.lcd lcd_task: [ON] -> [OFF]
09-17 19:13:04:324    [689053] I/drv.lcd lcd_task: HW close
09-17 19:13:04:324    [689069] I/drv.lcd lcd_task: HW close done.
09-17 19:13:04:325    [689085] I/drv.lcd lcd_task: Power off done
09-17 19:13:04:326    [689108] E/drv.i2c tpread: bus err:1, xfer:0/2, i2c_stat:20, i2c_errcode=21
09-17 19:13:04:327    [689141] E/drv.i2c tpread: reset and send 9 clks
09-17 19:13:04:369    [pm]S:3,690776
09-17 19:13:04:486    [pm]W:694583
09-17 19:13:04:487    [pm]WSR:0x4
09-17 19:13:04:564    [pm]S:3,697182
```

* Board operation phenomenon
![alt text](assets/wakeup_work.jpg)

* Board phenomenon when entering sleep mode
![alt text](assets/enter_pm.jpg)

## Power Consumption Test Results

* Assuming a battery capacity of 200mAh, tests were conducted under various usage scenarios to simulate user behavior of checking time on a smartwatch:
  - Light usage: 100 wrist raises per day
  - Moderate usage: 300 wrist raises per day
  - Heavy usage: 500 wrist raises per day

### When Screen is On

* Average power consumption during active work or wake-up state: 22.6mA
![alt text](assets/work.png)

* Daily power consumption for active work:
  - Light usage: 22.6 * 100 * 10 / 3600 = 6.3(mAh)
  - Moderate usage: 22.6 * 300 * 10 / 3600 = 18.8(mAh)
  - Heavy usage: 22.6 * 500 * 10 / 3600 = 31.4(mAh)

### When Screen is Off

* Average power consumption in sleep mode: 410uA
  Current breakdown:
  1. Base current: 50~60uA
  2. Sensor working current: ~60uA
  3. Charging chip current: ~30uA
  4. Current for data acquisition and wrist detection algorithm every 200ms: ~270uA

![alt text](assets/pm_mod_sensor.png)

![alt text](assets/pm_all.png)

* Daily total power consumption:
  - Light usage: 0.410 * (24 * 3600 - 100 * 10) / 3600 + 6.3 = 16.0(mAh)
  - Moderate usage: 0.410 * (24 * 3600 - 300 * 10) / 3600 + 18.8 = 28.3(mAh)
  - Heavy usage: 0.410 * (24 * 3600 - 500 * 10) / 3600 + 31.4 = 40.7(mAh)

