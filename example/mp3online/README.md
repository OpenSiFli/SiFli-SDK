# MP3在线播放例程使用指南

## 硬件需求
- 支持该例程的开发板: sf32lb52-lcd_n16r8
- 具备数据传输能力的USB数据线
- 网络连接：蓝牙PAN

## menuconfig配置

在项目根目录执行以下命令进行配置：
```bash
menuconfig --board=<board_name>
```
其中`<board_name>`替换为实际的板子名称，例如`sf32lb52-lcd_n16r8`。

### 需要开启的关键配置项

#### 蓝牙相关配置
##### menuconfig配置
1. 使能蓝牙(`BLUETOOTH`)：
    - 路径：Sifli middleware → Bluetooth
    - 开启：Enable bluetooth
        - 宏开关：`CONFIG_BLUETOOTH`
        - 作用：使能蓝牙功能
2. 使能PAN & A2DP，A2DP是为了避免IOS不支持单独连接PAN：
    - 路径：Sifli middleware → Bluetooth → Bluetooth service → Classic BT service
    - 开启：Enable BT finsh（可选）
        - 宏开关：`CONFIG_BT_FINSH`
        - 作用：使能finsh命令行，用于控制蓝牙
    - 开启：Manually select profiles
        - 宏开关：`CONFIG_BT_PROFILE_CUSTOMIZE`
        - 作用：手动选择使能的配置文件
    - 开启：Enable PAN
        - 宏开关：`CONFIG_CFG_PAN`
        - 作用：使能PAN协议
5. 蓝牙自动连接需要打开的menuconfig：
    - 路径：Sifli middleware → Bluetooth → Bluetooth service → Classic BT service
    - 开启：Enable BT connection manager 后，会默认开启 Re-connect to last device if connection timeout happened or system power on
        - 宏开关：`CONFIG_BT_AUTO_CONNECT_LAST_DEVICE`
        - 作用：使能自动连接上次连接的设备。  
    - 路径：Third party packages
    - 开启：FlashDB: Lightweight embedded database，一般为默认开启
        - 宏开关：`CONFIG_PKG_USING_FLASHDB`
        - 作用：启用FlashDB数据库，在断电或重启后依然能保留重要数据。


#### 网络相关配置
![alt text](../assets/TCP.png)

#### 文件系统配置
注意根据板子的flash类型进行配置：
![alt text](../assets/FLASH.png)

打开支持文件系统功能
![alt text](../assets/FATFS.png)


#### 音频相关配置
![alt text](../assets/Audio.png)


#### 图形界面配置
**注意**: 图形界面使用的是lvgl_v8中的lv_demos中的music demo，对界面按钮兼容实际mp3操作逻辑

![alt text](../assets/LittlevGL.png)
![alt text](../assets/LVGL_demo.png)


## 工程说明

### 编译方法
进入项目目录执行命令：
```bash
scons --board=<board_name> -j8
```
其中`<board_name>`替换为实际的板子名称。

编译生成的image文件存放在`build_<board_name>`目录下。

### 下载方法
进入项目目录执行相应的下载脚本：
```bash
build_<board_name>_hcpu\download.bat
```
或者使用串口下载：
```bash
build_<board_name>_hcpu\uart_download.bat
```

同时在LCD屏幕上会显示：
1. 音乐播放器主界面
2. 专辑封面图片
3. 歌曲标题和艺术家信息
4. 音乐频谱可视化效果
5. 播放控制按钮（播放/暂停、上一首、下一首）
6. 播放进度条
7. 音量控制滑块

界面支持手势操作：
- 向左滑动切换到下一首歌曲
- 向右滑动切换到上一首歌曲


## 故障排查

如果例程未能按预期运行，可以从以下几个方面进行故障排查：

### 硬件连接检查
- 确认开发板与电脑连接正常
- 检查USB线是否具备数据传输功能
- 确认网络连接设备工作正常

### 配置检查
- 确认menuconfig配置正确
- 特别检查板子flash型号配置是否正确

### 网络连接检查
- 确保开发板能够正常连接到网络
- 检查手机连接sifli-pan是否连接，手机已打开蓝牙共享网络
![alt text](../assets/android_enable_pan.png)
![alt text](../assets/ios_enable_pan)
### 文件系统检查
- 确认Flash初始化正常
- 检查文件系统是否正确挂载

### 串口log分析
- 通过串口输出的log信息定位问题
- 重点关注错误信息和警告信息

### 内存资源检查
- 确保系统有足够内存运行所有组件
- 必要时调整线程栈大小配置
