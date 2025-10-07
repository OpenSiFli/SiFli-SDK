# SquareLine Example (RT-Thread)

## Supported Platforms
<!-- Which boards and chip platforms are supported -->
- Any board (including [pc](file://d:\SiFli-SDK\SiFli-SDK\external\ffmpeg\libavcodec\aac_ac3_parser.h#L40-L40))

## Example Overview 
This example creates two screens, each containing other controls, and implements mutual switching between the two screens through button controls. The document details how to adapt and run the generated code in the current SDK environment.

## Using Squareline Software
Squareline software download address: [SquareLine Download](https://squareline.io/downloads#lastRelease). After entering the page, you need to register an account first before downloading and installing.

![alt test](assets/sqline_down2.png)
![alt test](assets/sqline_down1.png)

* After downloading and installing, open the SquareLine software, enter your registered email and password, click `LOG_IN` to log in, and check the obtained license
![alt test](assets/sqline_down3.png)

* After successful login, select Create to create a multi-platform UI project, configure the project information, and click `Create`
![alt test](assets/sqline1.png)

* The following demonstrates adding controls, where you can modify layout, events, set styles and other properties
![alt test](assets/sqline3.png)

* After completion, compile to generate .c files
![alt test](assets/sqline2.png)

For other detailed operations, please refer to: [squareline tutorial](https://www.bilibili.com/video/BV1Bu411p7cM)

## Squareline Generated Project Files

![alt test](assets/sqcode.png)

* components: This directory contains part of the embedded graphical interface development project, including component definitions, initialization and event handling code for the UI interface. If events and components are used in the software, they will be created in this directory.

* images: This directory contains generated image resource .c files for displaying PNG format images in the LVGL graphical interface

* screens/ui_Screen1.c, ui_Screen1.h: ui_Screen1.c is the specific implementation file of the (UI interface), containing the creation and destruction logic of the screen and its components. ui_Screen1.h declares screen-related variables and function interfaces for reference by other files. If multiple screens are created in Squareline software, multiple .c files will be generated, each corresponding to one screen with filenames consistent with screen names.

* CMakeLists.txt: This is a CMake build configuration file used to define and manage compilation rules for the entire UI project's source files.

* filelist.txt: Manages and compiles the entire LVGL user interface project, defining source file lists and creating static libraries

* ui_events.h: Event handling header file used to declare and manage UI component event callback functions. When event handling is added, this file will contain similar callback function declarations. Currently the file is almost empty, only containing basic header file protection structure.

* ui_helpers.c/ui_helpers.h: Common auxiliary function collection in the generated UI project, used to simplify common operations such as UI component property setting, animation control, and screen switching. They are the core utility function library of Squareline Studio generated code.

* ui.c/ui.h: The ui.c file implements UI interface functions, including UI initialization and screen destruction. The ui.h file defines public interfaces of the UI module for main program and other modules to call.

## Porting Squareline Generated Project Files
### Pre-porting Preparation
* "Since the SDK uses ezip software to further compress generated image resources to reduce space occupation, before porting we need to create a folder named image_ezip under the Square_Line directory, and then create another folder named ezip under this directory." The ezip folder is used to store image resources that need compression. Finally, a SConscript file needs to be created under the image_ezip folder for ezip hardware acceleration during compilation. If compression is not needed, images can be moved to the image directory, and the script will automatically decide whether compression is needed based on file location during compilation. The compilation linking script can refer to: [SConscript](image/SConscript).
![alt test](assets/image_ezip.png)

* After completing the above operations, add the following content in the SConscript in the project directory under `# Add application source code`, otherwise the ezip function will not be available during compilation.

```python
objs.extend(SConscript(cwd+'/../image_ezip/SConscript', variant_dir="image_ezip", duplicate=0))
```
### Start Porting
![alt text](assets/code_file.png)
* Step 1: After completing preparations, copy the generated images/ui_img_test_png.c file to the image_ezip/ezip folder in the SDK. During compilation, image resource files can be further compressed. (There is no mandatory requirement to use ezip compression, it can also be placed in the image/no_ezip folder without compression, but it is recommended to use ezip compression for better performance)

* Step 2: Except for the files already copied in the first step, copy all other files to the src directory. However, if copied directly, files in subdirectories will not be compiled, so we need to modify the SConscript compilation linking script, adding the following content to the SConscript file.
![alt test](assets/SConscript.png)

* Step 3: Call `ui_init();` in the main function in the main.c file to generate the UI interface startup function interface.

## Using the Example
### Hardware Requirements
* A board that supports this example
* A USB data cable

### menuconfig Configuration Process
* LVGL is enabled by default, no configuration needed
* Enable LittlevGL2RTT adaptation layer in menuconfig
![alt test](assets/menuconfig1.png)
* Select LVGL version in menuconfig
![alt test](assets/menuconfig2.png)
* Since the generated code uses default theme functionality, LVGL's default theme function needs to be enabled in menuconfig,
![alt test](assets/menu3.png)
### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```
scons --board=sf32lb52-lcd_n16r8 -j32
```
```
build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat
```

### Running Results
* The screen will first run screen1, as shown in the figure below.
![alt test](assets/result1.jpg)
* By clicking the button in screen1, you can switch to screen2 for display, as shown in the figure below.
![alt test](assets/result2.jpg)
* Clicking the button in screen2 can return to screen1 for display.