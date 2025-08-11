# Hello World示例（RT-Thread）

源码路径: example/get-started/hello_world/rtt

## 概述
Hello_world应用会在板子的串口打印hello world!

## 用法

### 支持的开发板
此示例可在以下开发板上运行：
- 任意板子（包括`pc`）

### 编译和烧录
切换到例程project目录，运行scons命令执行编译：
```
scons --board=sf32lb52-lcd_n16r8 -j32
```

simulator: 
```
scons --board=pc -j32
```

## 示例输出
如果示例运行成功，您将在串口看到以下输出：
```
Serial:c2,Chip:4,Package:3,Rev:3  Reason:00000000


 \ | /
- SiFli Corporation
 / | \     build on Jul 23 2025, 2.4.0 build 92567879
 2020 - 2022 Copyright by SiFli team
mount /dev sucess
[I/drv.rtc] PSCLR=0x80000100 DivAI=128 DivAF=0 B=256
[I/drv.rtc] RTC use LXT RTC_CR=00000001

[I/drv.rtc] Init RTC, wake = 0

[I/drv.audprc] init 00 ADC_PATH_CFG0 0x606

[I/drv.audprc] HAL_AUDPRC_Init res 0

[I/drv.audcodec] HAL_AUDCODEC_Init res 0

[I/TOUCH] Regist touch screen driver, probe=1202611d 
call par CFG1(3313)

fc 9, xtal 2000, pll 2067

call par CFG1(3313)

fc 7, xtal 2000, pll 1656

Hello world!
msh />Serial:c2,Chip:4,Package:3,Rev:3  Reason:00000000
```

## 异常诊断
暂无特定异常诊断信息。如有问题，请在GitHub上提出[issue](https://github.com/OpenSiFli/SiFli-SDK/issues)。

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [SiFli-SDK 开发指南](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/development/index.html)
