# PDM示例

源码路径：example/multimedia/audio/4pdm

## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
* sf32lb56-lcd_a128r12n1

## 概述
<!-- 例程简介 -->
本例程演示4mic或2mic PDM录音，固定使用了安凯的算法包含：
+ 通过PDM录音，录音wav文件保存。
+ 读取录音数据并播放。


## 例程的使用

### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板（[支持的平台](quick_start)）。
+ PDM
+ 喇叭。

### menuconfig配置

1. 使能PDM（这里双麦使用PDM1, 四麦使用PDM1和PDM2）

2. 使能AUDIO CODEC 和 AUDIO PROC

3. 使能AUDIO

4. 使能AUDIO MANAGER


```{tip}
配置3、4是为了播放录音文件，本例程中，录音播放使用Audio manager接口。
```  

### 硬件连接\PIN CONFIG
看下PDM用的哪个口，如果board中bsp_pinmux.c没配置，需要配置下pin function为PDM
* 更详细的引脚定义请参考
[sf32lb52-nano](https://wiki.sifli.com/board/sf32lb52x/SF32LB52-DevKit-Nano.html)
[sf32lb52-lcd](https://wiki.sifli.com/board/sf32lb52x/SF32LB52-DevKit-LCD.html)
[sf32lb56-lcd](https://wiki.sifli.com/board/sf32lb56x/SF32LB56-DevKit-LCD.html)
[sf32lb58-lcd](https://wiki.sifli.com/board/sf32lb58x/SF32LB58-DevKit-LCD.html)

以`f32lb56-lcd_a128r12n1`为例，管脚配置如下：
```C
    /* PIN CONFIG */
    HAL_PIN_Set(PAD_PA69, PDM1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA64, PDM1_DATA, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA73, PDM2_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA71, PDM2_DATA, PIN_PULLDOWN, 1);
````

### 编译和烧录
切换到例程project目录，运行scons命令执行编译：
```c
> scons --board=sf32lb56-lcd_a128r12n1 -j32
```
切换到例程`project/build_xx`目录，运行`uart_download.bat`，按提示选择端口即可进行下载：
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
关于编译、下载的详细步骤，请参考[快速上手](quick_start)的相关介绍。

## 例程的预期结果

本例程中通过FINSH命令来操作录音和播放， 录音生成test.wav文件：
1. 开始录音命令：`pdm open [PDM接口选择] [通道个数] [算法选择]`  
例如：
pdm open pdm1 1 raw  
表示，配置为PDM1单声道录音, 不经过任何算法，适合工厂测试。  
pdm open pdm1 2 raw  
表示，配置为PDM1双声道录音，不经过任何算法，适合工厂测试。  
pdm open pdm2 1 raw  
表示，配置为PDM2单声道录音, 不经过任何算法，适合工厂测试。  
pdm open pdm2 2 raw  
表示，配置为PDM2双声道录音，不经过任何算法，适合工厂测试。  
pdm open pdm12 4 raw  
表示，配置为PDM1/PDM2各自双声道，不经过任何算法，适合工厂测试， 只输出log， 不生成文件。  

pdm open pdm1 1 anyka  
表示，配置为PDM1单声道录音, 使用安凯算法，输出为单声道。  
pdm open pdm1 2 anyka  
表示，配置为PDM1双声道录音，使用安凯算法，输出为单声道。  
pdm open pdm2 1 anyka  
表示，配置为PDM2单声道录音, 使用安凯算法，输出为单声道。  
pdm open pdm2 2 anyka  
表示，配置为PDM2双声道录音，使用安凯算法，输出为单声道。  
pdm open pdm12 4 anyka  
表示，配置为PDM1/PDM2各自双声道，使用安凯算法处理4mic，输出为单声道。  

2. 结束录音命令：`pdm close`  
pdm close  
表示，结束录音。  

3. 播放录音命令：`pdm play`  
例如：  
pdm play

执行录音后，播放test.wav文件，可以正常播放。  

## 异常诊断

## 参考文档

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |04/2026 |初始版本 |
| | | |
| | | |
