# btstack 示例

源码路径：example/bt/btstack/src


## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
+ eh-lb52x
+ eh-lb56x
+ eh-lb58x

## 概述
<!-- 例程简介 -->
移植btstack的例子


## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
在`src/SConscript`选择历程, 默认的例子是a2dp_sink_demo

开始构建前要在`middleware/bluetooth`应用`bluetooth.patch`
### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板（[支持的平台](#Platform_music_sink)）。

### menuconfig配置

1. 无


### 编译和烧录
切换到例程project目录，运行scons命令执行编译：
```c
> scons --board=eh-lb525 -j32
```
切换到例程`project/build_xx`目录，运行`uart_download.bat`，按提示选择端口即可进行下载：
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
关于编译、下载的详细步骤，请参考[快速入门](/quickstart/get-started.md)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
例程启动后串口应显示：
```
Local version information:
- HCI Version    0x0c
- HCI Revision   0x200
- LMP Version    0x0c
- LMP Subversion 0x24e
- Manufacturer   0xa4c
BTstack up and running on CD:AB:78:56:34:12.
```
使用 `btstack <arg>`来交互, 例如`btstack b`用于建立连接(仅a2dp_sink_demo).
