# lvgl_v8_media
```{warning}
Not verified
```
## Usage Guide
### Introduction
This example demonstrates media playback. Users can overwrite H264 format MP4 files to disk/video_example.mp4, and after compilation and download, playback can be performed.
If the T card file system loads successfully, video_example.mp4 will be read preferentially from the T card root directory.

### Project Compilation and Download:
Supported boards
- Boards after 55x series, such as 58x, 56x, 52x

Board projects in the project directory can be compiled for corresponding boards by specifying the board parameter,
- For example, to compile a project that can run on HDK 563, execute scons --board=eh-lb563 to generate the project
- Download can be performed through download.bat in the build directory. For example, to flash the 563 project generated in the previous step, execute .\build_eh-lb563\download.bat to download via jlink
- Special note: for SF32LB52x/SF32LB56x series, an additional uart_download.bat will be generated. You can execute this script and input the download UART port number to perform the download
Simulator projects are in the simulator directory,
- Use scons for compilation. The simulator/msvc_setup.bat file needs to be modified accordingly to match the local MSVC configuration
- You can also use scons --target=vs2017 to generate MSVC project project.vcxproj and compile using Visual Studio.
    Note: If not using VS2017, for example VS2022, when loading the project, it will prompt to upgrade the MSVC SDK. After upgrading, it can be used.