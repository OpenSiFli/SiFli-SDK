# BT A2DP Sharing 示例

源码路径：example/bt/a2dp_sharing

{#Platform_music_src}
## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
+ eh-lb525
+ eh-lb563
+ eh-lb567
+ eh-lb58x
+ sf32lb52-lcd系列
+ sf32lb56-lcd系列
+ sf32lb58-lcd系列

## 概述
<!-- 例程简介 -->
本例程演示 A2DP 音乐分享（sharing）和 HFP 通话中继（relay）功能：设备作为中继在同时连接手机和耳机的情况下，将手机播放的音乐分享到耳机播放，同时也能将耳机端的音乐控制命令转发给手机实现音乐控制（<span style="color: red;">不包括调音量</span>）。

同时，本例程支持 HFP HF 和 HFP AG 双角色：设备连接手机时作为 HFP HF，连接蓝牙耳机时作为 HFP AG。当手机和耳机均连接到中继设备后，可以将手机侧的通话状态、来电号码、信号/电量等信息同步给耳机，并将耳机侧发起的拨号、接听、挂断、DTMF、通话音量调节等 HFP 控制请求转发给手机。


## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
例程启动后会默认使能蓝牙，可以接受手机连接或主动发起对耳机的连接。手机侧用于 A2DP sink/HFP HF 连接，耳机侧用于 A2DP source/HFP AG 连接。

1. 搜索蓝牙设备
通过命令 `a2dp_trans inquiry start` 来搜索耳机类蓝牙设备，该命令只会上报搜到的 COD 的 Major Class 为 0x000400 的设备（Audio device）。
搜索到的设备会以 log “device [%s] searched” 和 “device COD is [%d], addr is xx:xx:xx:xx:xx:xx” 的形式打印。

2. 连接蓝牙设备
通过命令 `a2dp_trans conn [addr]` 来进行连接，addr 将上面搜到设备的地址（xx:xx:xx:xx:xx:xx）打印值复制即可。
如果已知晓耳机类蓝牙设备地址，可以不用进行搜索蓝牙设备，直接连接即可。

3. 音乐分享
    1. 单独连上手机的情况下播放音乐，中继设备不会出声音。
    2. 单独连上耳机的情况下播放音乐，耳机设备不会出声音。
    3. 同时连上手机和耳机的情况下播放音乐，耳机设备会出声音，中继设备不会出声音。
    4. 已经在分享音乐的情况下，断开耳机，中继设备不会出声音。
    5. 已经在分享音乐的情况下，断开手机，耳机设备不会出声音。
    6. 已经在分享音乐的情况下，断开耳机后再重新连接上耳机，耳机设备会出声音。
    7. 中继设备默认不会回连耳机和手机设备。

4. HFP 通话中继
    1. 手机和耳机均连接到中继设备后，手机侧来电、拨号、通话状态变化会同步到耳机侧。
    2. 耳机侧执行接听、挂断、拨号、发送 DTMF 按键、调节通话音量等操作时，中继设备会将对应 HFP 控制请求转发给手机。
    3. 手机侧的运营商服务状态、信号强度、电池电量、漫游状态、来电号码、本机号码和当前通话信息等 HFP 指示信息会缓存并回复给耳机。
    4. 当手机侧 SCO 通话音频建立后，中继设备会尝试建立耳机侧 SCO 音频链路，并通过 `CONFIG_CFG_BT_VOICE_RELAY` 进行通话语音中继；任一侧 SCO 断开后会同步关闭对应语音中继链路。
    5. 典型连接成功 log 为 “HFP HF connected” 和 “HFP AG connected”；断开时分别打印 “HFP HF disconnected” 和 “HFP AG disconnected”。


### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板（[支持的平台](#Platform_music_src)）。
+ 一个支持 HFP 和 A2DP 的手机。
+ 一个蓝牙耳机。

### menuconfig配置
1. 本例程需要读写文件，所以需要用到文件系统，配置`FAT`文件系统：
    - 路径：RTOS → RT-Thread Components → Device virtual file system
    - 开启：Enable elm-chan fatfs
        - 宏开关：`CONFIG_RT_USING_DFS_ELMFAT`
        - 作用：开启fatfs文件系统
    ```{tip}
     mnt_init 中mount root分区。
    ```
2. 使能AUDIO CODEC 和 AUDIO PROC：
    - 路径：On-chip Peripheral RTOS Drivers
    - 开启：Enable Audio Process driver
        - 宏开关：`CONFIG_BSP_ENABLE_AUD_PRC`
        - 作用：使能Audio process device，主要用于音频数据处理（包括重采样、音量调节等）
    - 开启：Enable Audio codec driver
        - 宏开关：`CONFIG_BSP_ENABLE_AUD_CODEC`
        - 作用：使能Audio codec device，主要用于进行DAC转换
3. 使能AUDIO(`AUDIO`)：
    - 路径：Sifli middleware
    - 开启：Enable Audio
        - 作用：使能音频配置选项
4. 使能AUDIO MANAGER(`AUDIO_USING_MANAGER`)：
    - 路径：Sifli middleware → Enable Audio
    - 开启：Enable audio manager
        - 宏开关：`CONFIG_AUDIO_USING_MANAGER`
        - 作用：使用audio manager模块进行audio的流程处理
5. 使能本地音频(`AUDIO_LOCAL_MUSIC`)
    - 路径：Sifli middleware → Enable Audio
    - 开启：Enable local audio
        - 宏开关：`CONFIG_AUDIO_LOCAL_MUSIC`
        - 作用：使能本地音频功能
6. 预置音频文件：将音频文件放入 `\disk\` 目录进行预置下载。
    - 音频文件位于 `music_source/disk/test.mp3`。
7. 使能蓝牙(`BLUETOOTH`)：
    - 路径：Sifli middleware → Bluetooth
    - 开启：Enable bluetooth
        - 宏开关：`CONFIG_BLUETOOTH`
        - 作用：使能蓝牙功能
8. 使能A2DP source、A2DP sink、AVRCP和HFP relay：
    - 路径：Sifli middleware → Bluetooth → Bluetooth service → Classic BT service
    - 开启：Enable BT finsh（可选）
        - 宏开关：`CONFIG_BT_FINSH`
        - 作用：使能finsh命令行，用于控制蓝牙
    - 开启：Manually select profiles
        - 宏开关：`CONFIG_BT_PROFILE_CUSTOMIZE`
        - 作用：手动选择使能的配置文件
    - 开启：Enable A2DP
        - 宏开关：`CONFIG_CFG_AV`
        - 作用：使能A2DP
    - 开启：Enable A2DP source profile
        - 宏开关：`CONFIG_CFG_AV_SRC`
        - 作用：使能A2DP SOURCE ROLE
    - 开启：Enable A2DP share
        - 宏开关：`CONFIG_CFG_AV_SHARING`
        - 作用：使能A2DP音乐分享功能
    - 开启：Enable A2DP sink profile
        - 宏开关：`CONFIG_CFG_AV_SNK`
        - 作用：使能A2DP SINK ROLE
    - 开启：Enable AVRCP
        - 宏开关：`CONFIG_CFG_AVRCP`
        - 作用：使能AVRCP profile
    - 开启：Enable Handsfree HF
        - 宏开关：`CONFIG_CFG_HFP_HF`
        - 作用：使能HFP HF角色，用于连接手机侧HFP AG
    - 开启：Enable Handsfree AG
        - 宏开关：`CONFIG_CFG_HFP_AG`
        - 作用：使能HFP AG角色，用于接受耳机侧HFP HF连接
    - 开启：Enable BT voice relay
        - 宏开关：`CONFIG_CFG_BT_VOICE_RELAY`
        - 作用：使能HFP通话语音中继能力
9. 使能BT connection manager：
    - 路径：Sifli middleware → Bluetooth → Bluetooth service → Classic BT service
    - 开启：Enable BT connection manager
        - 宏开关：`CONFIG_BSP_BT_CONNECTION_MANAGER`
        - 作用：使用connection manager模块管理bt的连接
10. 使能NVDS：
    - 路径：Sifli middleware → Bluetooth → Bluetooth service → Common service
    - 开启：Enable NVDS synchronous
        - 宏开关：`CONFIG_BSP_BLE_NVDS_SYNC`
        - 作用：蓝牙NVDS同步。当蓝牙被配置到HCPU时，BLE NVDS可以同步访问，打开该选项；蓝牙被配置到LCPU时，需要关闭该选项

### 编译和烧录
切换到例程project目录，运行scons命令执行编译：
```bash
> scons --board=eh-lb525 -j32
```
切换到例程`project/build_xx`目录，运行`uart_download.bat`，按提示选择端口即可进行下载：
```bash
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
关于编译、下载的详细步骤，请参考[快速入门](/quickstart/get-started.md)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
例程启动后：
1. 在不连接蓝牙的情况播放内置音乐。
2. 可以搜索耳机类蓝牙设备，并在连接后播放内置音乐。
3. 同时连接手机和耳机后，手机播放的音乐可以通过中继设备分享到耳机播放。
4. 手机和耳机均建立HFP连接后，手机侧通话状态和号码信息可以同步到耳机，耳机侧接听、挂断、拨号、DTMF和通话音量调节等控制可以转发到手机。
5. HFP连接成功时串口打印 “HFP HF connected” 和 “HFP AG connected”；通话状态变化时会打印 “the remote phone call_status”、“callsetup_status”、“callheld_status”等log。

## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |05/2026 |初始版本 |
