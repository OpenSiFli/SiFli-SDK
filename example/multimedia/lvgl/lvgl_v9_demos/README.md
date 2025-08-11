# LVGL v9 Demo

源码路径：example/multimedia/lvgl/lvgl_v9_demos
## 概述

本示例用来演示LVGL V9的官方示例，使用官方提供的demo应用程序。
可以使用menuconfig来选择演示的demo应用程序。包含的应用程序有：
- Show some widget 演示lvgl widget的使用
    - Benchmark your system 演示benchmark(依赖于“Show some widget”)
- Demonstrate the usage of encoder and keyboard 演示键盘
- Stress test for LVGL 压力测试
- Music player demo 演示音乐播放

## 支持的开发板

<!-- 支持哪些板子和芯片平台 -->
- 55x之后的板子，比如58x，56x, 52x的
- SF32LB52x系列
- SF32LB56x系列

## 硬件需求

运行该例程前，需要准备：
- 一块本例程支持的开发板
- 屏幕

## 工程编译及下载：
板子工程在project目录下可以通过指定board来编译适应相对board的工程，
- 比如想编译可以在HDK 563上运行的工程，执行scons --board=eh-lb563即可生成工程
- 下载可以通过build目录下的download.bat进行，比如同样想烧录上一步生成的563工程，可以执行.\build_eh-lb563\download.bat来通过jlink下载
- 特别说明下，对于SF32LB52x/SF32LB56x系列会生成额外的uart_download.bat。可以执行该脚本并输入下载UART的端口号执行下载
模拟器工程在simulator目录下，
- 使用 scons 进行编译，SiFli-SDK/msvc_setup.bat文件需要相应修改，和本机MSVC配置对应
- 也可以使用 scons --target=vs2017 生成 MSVC工程 project.vcxproj, 使用Visual Studio 进行编译。

```{note}
如果不是使用VS2017, 例如 VS2022, 加载工程的时候，会提示升级MSVC SDK, 升级后就可以使用了。
```
## 异常诊断

如有任何技术疑问，请在GitHub上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
                
      
