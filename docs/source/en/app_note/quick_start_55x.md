
# SDK Quick Start

## 1. Development Environment Setup
### Required Software
1. Sifli env and SiFli SDK (Download from Weiyun)
2. Keil V5.32 or above, armcc V6 version or above
   Add the ARM and Jlink paths to the Windows environment variable PATH, such as _C:\Keil_v5\ARM\ARMCLANG\bin_ and _C:\Program Files(x86)\SEGGER\JLink_v672b_, which is helpful for some scripts.
3. Segger Jlink, V6.72b or above. [Jlink Download Link](https://www.segger.com/downloads/jlink/JLink_Windows.exe)
4. Visual Studio 2017 or higher version

### Required Hardware
1. Windows PC x1
2. ARM Emulator x1   
```{note}  
  There is a conflict between the power supply of the emulator and the EVB board, so you need to disable the emulator's power supply. If the emulator connection fails, check the power supply jumper on the Jlink.  
  The following image shows the common Jlink power supply jumper that needs to be removed.
  ![](/assets/disable_jlink_power_supply.png)
```  
3. USB Type-C Cable x1  
   Used to power the EVB and connect the FTDI USB2UART chip on the EVB to capture serial log data from the EVB.
4. EVB Development Board x1 set  
   Includes an interface baseboard EI-LB55XXXX001, a CPU sub-board LB551, and a screen sub-board (the model of the sub-board varies depending on the screen).  
>  EVB Development Board Interface Overview  ![](../../assets/evb_a0_overview.png)

### Compilation Necessary Configuration
1. Extract the env to your PC, such as _c:\work\env_
2. Enter the env directory and run _env.exe_ in this directory. If it fails to open, try using _env.bat_ 
3. Right-click on the associated folder and select the "ConEmu Here" menu  
![ ](../../assets/Add_Env_To_Right-click_Menu.png)

### Adding Keil Flash Writing Driver
Copy _sf32lb55x.FLM_ from the _tools/flash/keil_drv/sf32lb55x_ directory to _C:\Keil_v5\ARM\Flash_ (this is the default Keil installation directory; if installed in another directory, place it in the appropriate folder).

## 2. Compile Example Projects

### Compile EVB Project (DSI Screen)
1. Open the SifliSDK directory, right-click and select "ConEmu Here" to open ConEmu, execute the batch file `set_env.bat` (this sets the necessary environment variables, and if ConEmu remains open, this only needs to be done once; reopening ConEmu will require the setup again).
```{note}  
The _set_env.bat_ file includes the Keil installation directory (default is _C:/Keil_v5_). Users need to modify it according to their installation directory.
```
2. Switch to _example/watch_demo/project/ec-lb551_
3. Run the command `scons --target=mdk5 –s` to generate the Keil project.
4. Then open the _project.uvprojx_ in Keil to compile.
```{note}  
The compilation for the `ec-lb555` project is similar.
```

### Compile Simulator Project
1. Open the SifliSDK directory, right-click and select "ConEmu Here" to open ConEmu, execute the batch file _set_env.bat_
2. Switch to _example\watch_demo\project\watch_simu_
3. Run the command `scons --target=vs2017 -s` to generate the VS project, then open the _project.vcxproj_ in Visual Studio 2017 to compile and run.
```{note}  
If Visual Studio prompts that it cannot find _SDL2.DLL_, add the path _env\tools\Python27_ to the PATH environment variable, then restart Visual Studio.
```
```{note}  
If a different version of Visual Studio is installed, change the Windows SDK version as prompted when opening the project.
```

## 3. Flashing the EVB

The SDK provides three methods to flash the EVB. One uses the Keil environment, and the other two use Jlink tools.

```{note}  
Normally, there is no need to change the jumper `boot_mode`. If there are persistent issues with flashing, or if the user program crashes, place the jumper on the VDD side, press reset to enter boot mode, and try flashing again. After flashing is complete, place the jumper on the GND side and press reset to reboot for normal operation.
```
### 3.1 Flash EVB using Keil
To flash the EVB using Keil, you first need to add the Keil Flash writing driver. Copy _sf32lb55x.FLM_ from the _tools\flash\keil_drv\sf32lb55x_ directory to _C:\Keil_v5\ARM\Flash_ (this is the default Keil installation directory; if installed in another directory, place it in the appropriate folder).

The EVB requires at least two parts to be flashed:  
- Flash Table (flash only once, used for ROM to read address from FlashTable and jump to user code after reset). The default flash table is already written when the user receives the EVB. Unless the flash is corrupted, this step can usually be ignored.  
- Project Code

#### Flashing the Flash Table
1. Open the project _example/flash_table/project.uvprojx_
2. Follow the previous section to select the Keil Flash writing driver.
3. Select Flash1 as the compilation target, then compile and flash.
![ ](../../assets/keil_download_flash_table.png)

#### Flashing the Project Code
1. Open _example/hal_example/project/ec_lb551/project.uvprojx_
2. Follow the previous section to select the Keil Flash writing driver.
3. After compiling, flash the code.

#### Selecting Keil Flash Writing Driver
- Open the project options, and choose the Flash driver as shown below:
![ ](../../assets/keil_flash_download_config_a0.png)

### 3.2 Flash EVB using Jlink

This example uses Jlink version v672b, with the installation path _D:\Software\JLink_v672b_

#### Adding Jlink Flash Writing Driver
1. Create a directory _SiFli_ in the Jlink program directory _D:\Software\JLink_v672b\Devices_
2. Copy the elf files from _tools/flash/jlink_drv/sf32lb55x_ to the newly created SiFli directory (each elf corresponds to a flash writing driver).
![](../../assets/add_sifli_jlink_device_A0_1.png)
3. Modify the Jlink registered Device list to include the newly added file path and run parameters, as shown below:
![](../../assets/add_sifli_jlink_device_2.png)

The added xml content is shown below:
```
  <!--                                    -->
  <!-- SiFli Z0(Cortex-M33 devices)-->
  <!--                                    -->
  <Device>
    <ChipInfo Vendor="SiFli" Name="SF32LB5XX" Core="JLINK_CORE_CORTEX_M33" WorkRAMAddr="0x20000000" WorkRAMSize="0x40000" />
    <FlashBankInfo Name="Internal Flash1" BaseAddr="0x10000000" MaxSize="0x400000" Loader="Devices/SiFli/SF32LB5XX_INT_FLASH1.elf" LoaderType="FLASH_ALGO_TYPE_OPEN" AlwaysPresent="1"/>
    <FlashBankInfo Name="External Flash2" BaseAddr="0x18000000" MaxSize="0x2000000" Loader="Devices/SiFli/SF32LB5XX_EXT_FLASH2.elf" LoaderType="FLASH_ALGO_TYPE_OPEN" AlwaysPresent="1"/>
  </Device>
  <!--                                    -->
  <!-- SiFli SF32LB55X(Cortex-M33 devices)-->
  <!--                                    -->
  <Device>
    <ChipInfo Vendor="SiFli" Name="SF32LB55X" Core="JLINK_CORE_CORTEX_M33" WorkRAMAddr="0x20000000" WorkRAMSize="0x40000" />
    <FlashBankInfo Name="Internal Flash1" BaseAddr="0x10000000" MaxSize="0x2000000" Loader="Devices/SiFli/SF32LB55X_INT_FLASH1.elf" LoaderType="FLASH_ALGO_TYPE_OPEN" AlwaysPresent="1"/>
    <FlashBankInfo Name="External Flash2" BaseAddr="0x64000000" MaxSize="0x2000000" Loader="Devices/SiFli/SF32LB55X_EXT_FLASH2.elf" LoaderType="FLASH_ALGO_TYPE_OPEN" AlwaysPresent="1"/>
    <FlashBankInfo Name="External Flash3" BaseAddr="0x68000000" MaxSize="0x2000000" Loader="Devices/SiFli/SF32LB55X_EXT_FLASH3.elf" LoaderType="FLASH_ALGO_TYPE_OPEN" AlwaysPresent="1"/>
    <FlashBankInfo Name="External Flash4" BaseAddr="0x12000000" MaxSize="0x2000000" Loader="Devices/SiFli/SF32LB55X_EXT_FLASH4.elf" LoaderType="FLASH_ALGO_TYPE_OPEN" AlwaysPresent="1"/>
  </Device>
```

#### Flashing Bin/Hex Files to Flash
1. Open Jlink, connect, and select the SiFli device (make sure SF32LB55X corresponds to the EVB flash driver).
![](../../assets/download_with_jlink_a0_1.png)

2. Select the SWD interface, configure the speed, choose the corresponding bin file, and flash it to the appropriate address (you can flash ROM/RAM/FLASH). Jlink's speed depends on the hardware and is usually set to over 4MHz, with the chip supporting a maximum of 10MHz.
![](../../assets/download_with_jlink_2.png)

3. Flashing result.
![](../../assets/download_with_jlink_3.png)

4. For flashing Hex files, change `loadbin` to `loadfile` and **do not include the address**.
![](../../assets/download_with_jlink_4.png)

### 3.3 Integrating into Windows, Right-Click to Flash Hex Files
After adding the Jlink drivers, for convenience in debugging, you can integrate flashing Hex files into the Windows right-click menu. Only Hex files can be flashed this way, as they have predefined addresses. Bin files do not have predefined addresses and are unsupported.

1. Add to the right-click menu
![](../../assets/integrate_jlink_download_to_right_click_menu_A0_1.png)

2. Right-click on the directory to flash, and it will convert the files in the directory to `.hex` and flash them one by one.
![](../../assets/integrate_jlink_download_to_right_click_menu_A0_2.png)

3. You can also choose to flash a single Hex file (only `.hex` files are supported).
![](../../assets/integrate_jlink_download_to_right_click_menu_3.png)

```{note}  
For BLE applications to run properly, input the following commands in the serial port:  
1. `nvds reset_all 1`  
2. `nvds update addr 6 <Bluetooth address>`  
   For example: `nvds update addr 6 1234567890C8`, note that the Bluetooth address format should be maintained, and it is recommended not to modify the C8 part. Other parts can be changed by the user.  
   Then press reset to reboot.
```

```{warning}  
For the 56 and 58 series projects, after compilation, the build directory will generate _download.bat_ and _download.jlink_ files. Run _download.bat_ to download the image. Do not use the right-click method to download the Hex file. If you do, it will modify the _download.jlink_ file content, causing the download to fail. Please delete the modified _download.jlink_ file and recompile to use _download.bat_ for downloading.
```

## 4. Power-Up Boot Process
| Phase                    | Function                        | File Path                                                       |
|--------------------------|---------------------------------|-----------------------------------------------------------------|
| Interrupt Vector Table    | ResetHandler                   | SifliSDK\drivers\cmsis\sf32lb55x\Templates\arm\startup_bf0_hcpu.S |
| MPU Setup, BOOTMODE Check | SystemInit()                   | SifliSDK\drivers\cmsis\sf32lb55x\Templates\system_bf0_ap.c  |
| RO/RW/ZI Initialization   | __main()                        |                                                                 |
| RT_THREAD_MAIN Entry      | `$Sub$$main`                    | SifliSDK\rtos\rtthread\src\components.c                    |
| ^                         | rtthread_startup()              | SifliSDK\rtos\rtthread\src\components.c                    |
| Hardware Initialization   | rt_hw_board_init()              |                                                                 |
|                          | HAL_Init();                     |                                                                 |
| RCC Configuration         | HAL_PreInit()                   | SifliSDK\drivers\boards\ec-lb551XXX\drv_io.c               |
|                          | HAL_MspInit()                   |                                                                 |
| Pin Configuration         | BSP_IO_Init()                   |                                                                 |
|                          | rt_system_heap_init             |                                                                 |
| Log Output Initialization | rt_console_set_device           |                                                                 |
| Driver Initialization     | rt_components_board_init();     |                                                                 |
| Upper-Level Initialization| rt_application_init()           | SifliSDK\rtos\rtthread\src\components.c                    |
|                          | main_thread_entry               | SifliSDK\rtos\rtthread\src\components.c                    |
| Upper-Level Middleware    | rt_components_init              |                                                                 |
|                          | Main()                          |   Project's main function                                       |
| Start Thread Scheduling   | rt_system_scheduler_start()     |                                                                 |

```{note}  
After plugging in the EVB development board to the PC, the PC will enumerate four serial ports. The smallest numbered port is the Boot/LCPU terminal, and the second smallest is the HCPU terminal. Currently, it is recommended for users to focus on the HCPU UART output. The other two serial ports are not assigned yet.
```

