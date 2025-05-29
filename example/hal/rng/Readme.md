# RNG示例
源码路径:example/hal/rng
## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
+ sf32lb52-lcd_n16r8
+ sf32lb58-lcd_n16r64n4
## 概述
<!-- 例程简介 -->
本例程演示使用RNG（随机数生成器）生成随机数种子和随机数。

## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### 硬件需求
运行该例程前，需要准备一块本例程支持的开发板

<!-- ### menuconfig配置 -->
 
### 编译和烧录
切换到例程project目录，运行scons命令执行编译：
```
scons --board=sf32lb52-lcd_n16r8 -j32
```
运行`build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`，按提示选择端口即可进行下载：
```
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
关于编译、下载的详细步骤，请参考[](/quickstart/get-started.md)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
例程启动后，串口输出如下：
1. 每500ms打印一组（16个）随机数。  
    ```c
    SFBL
    RND_BUFFER:
    E8C706A8 9951352B F8E1DA37 9F1187FE 1F646D6F DD91FFC6 555F62E3 F0331F25 
    AB574A6E 458BF6B9 9232FD31 D2087C59 C7D34012 237536ED 642E65DD D85D20F7 
    RND_BUFFER:
    2B8EB6A6 D06F3E0F 4CA2B859 C24998F1 FEFB0DD9 1CD7F97E 1E5EECA4 D165FE0C 
    02AE7A9A 3CA3E015 2CB76E67 ABBD8780 D017F211 9DC5F365 846AA886 0C348503 
    RND_BUFFER:
    3892391B 0317442D 5D722D31 6A99CE87 D7DA4DBC CC08065B EBE93330 4929224A 
    53B59A81 28B15E6A 6BCB5A60 979D54B1 4D470A5E 001C491C F70875DA 8DD9FD34 
    RND_BUFFER:
    234A3E47 1AB20C8F 3D06FD8D DA68708D 7C463B6E C31EDA11 1BE0B91E E0A46D41 
    2D6E9838 74A6C7F1 68117920 BCCFB254 13F8BC97 E2AFB62B 7025ADBC D995DC5E 
    ```

## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |10/2024 |初始版本 |
| | | |
| | | |


