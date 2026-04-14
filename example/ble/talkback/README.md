# BLE 对讲示例

源码路径：example/ble/talkback

## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
+ sf32lb52-lchspi-ulp
+ sf32lb52-nano系列
+ sf32lb52-lcd系列
+ sf32lb56-lcd系列
+ sf32lb58-lcd系列


## 概述
<!-- 例程简介 -->
1. 本例程演示了本平台如何实现基于BLE的语音对讲功能。
2. 该示例利用BLE 5.0的周期性广播(Periodic Advertising)和周期性广播同步(Periodic Advertising Sync)特性，实现了低功耗、低延迟的无线对讲功能。
3. 支持多设备组网，最多可同时连接3个设备进行对讲。
4. 使用Opus音频编解码技术，保证高质量的语音传输。

## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
1. 本例程开机后会自动开启扫描，寻找周围的对讲设备，并建立连接。
2. 同时也会开启广播，使其他设备能够发现本设备。
3. 按下开发板上的PTT(Push-To-Talk) 按键 (KEY1-对应开发版片上的丝印) 开始讲话，松开按键结束讲话。
4. 当其他设备讲话时，本设备会自动播放对方的语音。
5. 支持通过Finsh命令查看和调试系统状态。

### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板
+ 至少两块开发板进行对讲测试。
+ 音频输入输出设备（麦克风和扬声器）。

### 硬件连接
根据开发版指南连接麦克风和扬声器。
[sf32lb52-lcd_n16r8](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/supported_boards/boards/sf32lb52-lcd_n16r8/doc/index.html#)
[sf32lb52-nano_n16r16](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/supported_boards/boards/sf32lb52-nano_n16r16/doc/index.html#)
[sf32lb52-lchspi-ulp](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/supported_boards/boards/sf32lb52-lchspi-ulp/doc/index.html#)


### menuconfig配置
1. 使能蓝牙
    - 路径：Sifli middleware → Bluetooth
    - 开启：Enable bluetooth
        - 宏开关：`CONFIG_BLUETOOTH`
        - 作用：使能蓝牙功能
2. 使能音频相关功能：
    - 路径：Sifli middleware → Audio
    - 开启：Enable audio server
        - 宏开关：`CONFIG_USING_AUDIO_SERVER`
        - 作用：使能音频服务
    - 开启：Enable Opus codec
        - 宏开关：`CONFIG_PKG_LIB_OPUS`
        - 作用：使能Opus音频编解码库
3. 使能WebRTC音频处理：
    - 路径：Sifli middleware → Audio → WebRTC
    - 开启：Enable WebRTC AGC
        - 宏开关：`CONFIG_PKG_USING_WEBRTC`, `CONFIG_WEBRTC_AGC_FIX`
        - 作用：使能自动增益控制
    - 开启：Enable WebRTC AECM
        - 宏开关：`CONFIG_WEBRTC_AECM`
        - 作用：使能回声消除
4. 使能按键：
    - 路径：Sifli middleware
    - 开启：Enable button library
        - 宏开关：`CONFIG_USING_BUTTON_LIB`
        - 作用：使能按键库

**注意**：本系统基于BLE 5.0周期性广播实现，无需传统的GATT客户端和连接管理器功能。

### 编译和烧录
切换到例程project目录，运行scons命令执行编译：
```c
> scons --board=eh-lb525 -j32
```
切换到例程`project/build_xx`目录，运行`uart_download.bat`按提示选择端口即可进行下载：
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
关于编译、下载的详细步骤，请参考[快速入门](/quickstart/get-started.md)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
例程启动后：
1. 系统会自动初始化BLE和音频模块。
2. 开始扫描周围的对讲设备并建立连接。
3. 按下PTT按键开始讲话时，其他设备能够听到清晰的语音。
4. 当其他设备讲话时，本设备会自动播放语音。
5. 串口会输出相关的日志信息，便于调试和监控系统状态。

## 异常诊断
1. 如果无法发现其他设备，请检查：
   - 确保所有设备都在同一网络中（使用相同的DEFAULT_NETWORK_CODE）
   - 检查BLE功能是否正常启用
   - 确认设备间的距离在有效范围内
   
2. 如果音频质量不佳，请检查：
   - 麦克风和扬声器连接是否正确
   - 音频参数配置是否合适（采样率、位深度等）
   - 是否存在电磁干扰
   
3. 如果按键无响应，请检查：
   - 按键引脚配置是否正确
   - 按键驱动是否正常加载
   - 按键硬件连接是否可靠

## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->
- [SiFli BLE开发指南](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/middleware/bt_service.html)
- Opus音频编解码技术文档
- RT-Thread音频框架文档

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |12/2025 |初始版本 |
| | | |
| | | |