# LCD显示示例
源码路径:example/hal/lcd

## 支持的平台
例程可以运行在以下开发板.
* sf32lb58-lcd_n16r32n1_dpi

## 概述
* 演示直接使用 LCDC 的 HAL 接口点亮并刷新一块 DPI(RGB) 接口的 LCD 屏
* 屏幕先显示竖直彩条测试图，之后每隔 2 秒在 彩条/红/绿/蓝/白 之间循环切换

## 例程的使用
### 编译和烧录
切换到例程 project 目录，运行 scons 命令编译:
```
scons --board=sf32lb58-lcd_n16r32n1_dpi -j8
```
运行 `build_sf32lb58-lcd_n16r32n1_dpi_hcpu\uart_download.bat`，按提示选择端口即可下载:
```
> build_sf32lb58-lcd_n16r32n1_dpi_hcpu\uart_download.bat

     Uart Download

please input the serial port num:5
```
关于编译、下载的详细步骤，请参考[](/quickstart/get-started.md)的相关介绍。

### 例程输出结果展示
* 屏幕显示: 先显示彩条，随后红/绿/蓝/白循环切换
* log输出:
```
I/hal.lcd main: show color bars
I/hal.lcd main: show solid color 0
I/hal.lcd main: show solid color 1
I/hal.lcd main: show solid color 2
I/hal.lcd main: show solid color 3
I/hal.lcd main: show color bars
...

```

## 异常诊断
* 屏幕无显示
  1. 确认背光是否点亮
  2. 确认屏的复位、供电是否正常
  3. 确认屏型号与开发板匹配
* 画面颜色/位置异常
  1. 确认所用屏与例程默认的 HTM-H070A20(1024x600) 一致
  2. 更换屏时需相应调整例程中的分辨率、接口时序与控制引脚

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |06/2026 |初始版本 |
| | | |
