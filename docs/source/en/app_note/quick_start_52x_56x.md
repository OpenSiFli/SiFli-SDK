
# SDK Quick Start (SF52X SF56X)

## 1. Development Tool Setup
### Required Software
1. Sifli env and SiFli SDK (Download from Weiyun).
2. Keil V5.32 or above, armcc V6 or above.
3. Segger Jlink, V6.80a or above.  
   [Jlink Download Link](https://www.segger.com/downloads/jlink/JLink_Windows.exe)
4. Code editing software: VSCode or Source Insight.
5. Serial log viewing tool: Use the SDK's built-in sifliTrace or sscom.

### Required Hardware
1. Windows PC x1
2. ARM Emulator x1  
```{note}  
There is a conflict between the power supply of the emulator and the HDK board, so you need to disable the emulator's power supply. If the emulator connection fails, check the power supply jumper on the Jlink.  
The following image shows the common Jlink power supply jumper that needs to be removed. 
![](/assets/disable_jlink_power_supply.png)
```
3. USB Type-C Cable x2  
   Used to connect the ARM emulator and the USB2UART debug board to power the HDK board and capture HDK serial log data.
4. USB2UART Debug Board x1 (SIFLI-Debug001-V1.1.0).  
>  Overview of the USB2UART Debug Board ![](../../assets/hdk_sifli_debug_overview.png)
5. HDK Development Board x1 set  
   Includes an HDK CPU board (SF56X or SF52X) and a screen sub-board (the model of the sub-board varies depending on the screen).  
>  HDK52x Development Board Interface Overview ![](../../assets/hdk_micro_overview.png)  
>  HDK56x Development Board Interface Overview ![](../../assets/hdk_lite_overview.png)  
>  Screen Sub-board Interface Overview ![](../../assets/hdk_screen_overview.png)

## 2. Development Environment Configuration
### Configuring the ENV
1. Extract the env to your PC, for example to _D:\work\env_.
2. Enter the env directory and run _env.exe_ in this directory. If it fails to open, try using _env.bat_.
3. Right-click on the associated folder and select the "ConEmu Here" menu.
>  Add ConEmu Here to the right-click menu ![](../../assets/add_env_conemu_1.png)  
>  Right-click any folder to open ConEmu Here ![](../../assets/add_env_conemu_2.png)

### Installing Keil
Double-click the Keil installer and follow the prompts to install it. Note that Keil should be installed to the _C:\Keil_v5_ directory. After starting Keil, open _Help->>About_,  
If the displayed version is not v5.32, uninstall Keil and install the latest version.  
![](../../assets/keil_version_about.png)  
After installing Keil, add the ARM path to the Windows environment variable PATH, for example: _C:\Keil_v5\ARM\ARMCLANG\bin_.

### Installing JLink
Jlink needs to be installed with version v6.80a or later. Double-click the installer _JLink_Windows_V680a.exe_, select the default installation path, and add the JLink path to the Windows environment variable PATH,  
For example: _C:\Program Files (x86)\SEGGER\JLink_.  
Copy all ELF files from the _$SDK_ROOT\tools\flash\jlink_drv\sf32lbxx_ directory to the _Devices\SiFli_ directory in the JLink installation path. The SiFli directory must be manually created.  
After copying, it should look like the following.  
Add Jlink drivers to Jlink installation directory ![](../../assets/JLink_drv_1.png)

Merge the SiFli-related content from _JLinkDevices.xml_ in the jlink_drv directory into the _JLinkDevices.xml_ in the JLink installation directory.  
![](../../assets/JLink_drv_2.png)

Modify the JLinkDevice file ![](../../assets/JLink_drv_3.png)

### Configuring System Environment Variables
Add the following paths to the PATH variable in system environment variables. The actual paths need to match the local tool and code paths.  
Add environment variable PATH![](../../assets/add_env_path.png)

## 3. Compiling Project Code
### SF52X Project Code Compilation
Using the `sf525` project under the `hal_example` as an example, the compilation steps are as follows:  
1. Go to the SDK root directory, _$SDK_ROOT_, right-click and select "ConEmu Here".
2. Execute the script `.\set_env.bat_` to configure the environment.
3. Enter the compilation directory: `cd example\hal_example\project\common`
4. Execute the compile command: `scons --board=eh-lb525_v2_hcpu -j8`
```{note}  
`--board=xx` specifies the board information. You can choose a directory from _customer\\boards_ with a directory name containing "v2" and add _hcpu_ to the end, for example, `eh-lb525_v2_hcpu`.  
After compilation, a folder `build_eh-lb525_v2_hcpu` will be created under the common directory, storing the generated images and other files.  
![sf525 Project Compilation Successful](/assets/sf525_compile_success.png)
```

### SF56X Project Code Compilation
Using the `sf561` project under the `hal_example` as an example, the compilation steps are as follows:  
1. Go to the SDK root directory, right-click and select "ConEmu Here".
2. Execute the script `.\set_env.bat` to configure the environment.
3. Enter the compilation directory: `cd example\hal_example\project\common`
4. Execute the compile command: `scons --board=eh-lb561_v2_hcpu -j8`
```{note}  
`--board= xx` specifies the board information. You can choose a directory from _customer\\boards_ with a directory name containing "v2" and add _hcpu_ to the end, for example, `eh-lb561_v2_hcpu`.  
After compilation, a folder `build_eh-lb561_v2_hcpu` will be created under the common directory, storing the generated images and other files.  
![sf561 Project Compilation Successful](../../assets/sf561_compile_success.png)
```

## 4. Download and Flash HDK
### SF52X Project Image Download and Flash
The `sf525` uses UART interface to download the image. Connect the HDK board to the siflideubg debug board, then use the USB Type-C cable to connect the siflideubg debug board to the computer. First, confirm the COM port of the CPU's log output, for example, COM4.  
1. Close the serial log tool's COM port.
2. In the compilation directory, run the download script `.\build_eh-lb525_v2_hcpu\uart_download.bat`.
3. It will prompt you to input the COM port used for downloading. Enter 4 and press Enter to start the download.  
> Downloading SF525 Image ![](../../assets/sf525_download_img.png)

### SF561 Project Image Download and Flash
The `sf561` uses JLink to download the image. Connect the HDK board to the siflideubg debug board, then connect the siflideubg debug board to the computer for serial log printing. Connect the ARM emulator to the siflideubg debug board using the JLink ribbon cable, and the other end of the ARM emulator should be connected to the computer via Type-C.  
1. Use J-Link Commander to send the `connect` command and check if the development board can connect successfully.  
```{note}  
The device name chosen during `connect` should match the `JLINK_DEVICE` in the `rtconfig.py` file in the project directory. For example, select `SF32LB56X` for the `sf561` project. After selecting the device, choose the interface (SWD), then configure the transmission rate with the default settings. Once connected successfully, you will see the following.
> SF561 Connecting to JLink  
![](../../assets/sf561_jlink_connect_1.png)  
![](../../assets/sf561_jlink_connect_2.png)  
```
2. In the compilation directory, run the download script `.\build_eh-lb561_v2_hcpu\download.bat` to start JLink downloading.  
> Downloading SF561 Image  
![](../../assets/sf561_download_img_1.png)  
![](../../assets/sf561_download_img_2.png)
