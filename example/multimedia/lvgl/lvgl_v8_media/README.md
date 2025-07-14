# 本地视频文件播放示例

源码路径：example/multimedia/lvgl/lvgl_v8_media

## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
+ eh-lb523
+ sf32lb52-lcd_n16r8

## 概述

本示例用来演示媒体播放，用户可以将H264格式的MP4文件覆盖到disk/video_example.mp4，编译下载后就可以播放了。
如果T卡加载文件系统成功，将优先从T卡根目录读取video_example.mp4

### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板（[支持的平台](quick_start)）。
+ 屏幕。



### 工程编译及下载：
支持的板子
- 55x之后的板子，比如58x，56x, 52x的

板子工程在project目录下可以通过指定board来编译适应相对board的工程，
- 比如想编译可以在HDK 563上运行的工程，执行scons --board=eh-lb563即可生成工程
- 下载可以通过build目录下的download.bat进行，比如同样想烧录上一步生成的563工程，可以执行.\build_eh-lb563\download.bat来通过jlink下载
- 特别说明下，对于SF32LB52x/SF32LB56x系列会生成额外的uart_download.bat。可以执行该脚本并输入下载UART的端口号执行下载
模拟器工程在simulator目录下，
- 使用 scons 进行编译，simulator/msvc_setup.bat文件需要相应修改，和本机MSVC配置对应
- 也可以使用 scons --target=vs2017 生成 MSVC工程 project.vcxproj, 使用Visual Studio 进行编译。
    注：如果不是使用VS2017, 例如 VS2022, 加载工程的时候，会提示升级MSVC SDK, 升级后就可以使用了。