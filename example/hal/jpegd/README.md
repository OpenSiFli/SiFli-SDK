# JPEGD HAL 示例

源码路径：`example/hal/jpegd`

## 概述

本例程在 SF32LB57X HCPU 上直接调用 `HAL_JPEGD`，以 AHB 输出方式解码内置的 100×100 JPEG，并通过 LCD 设备绘制到屏幕中心。

## 支持的开发板
* sf32lb57 系列

## 例程的使用

### 编译和烧录
切换到例程project目录，运行scons命令执行编译(board=版型)：
```
scons --board=spi-hdk_lb573ub7n6 -j11
```
`build_spi-hdk_lb573ub7n6_hcpu\uart_download.bat`，按提示选择端口即可进行下载：

```
build_spi-hdk_lb573ub7n6_hcpu\uart_download.bat

Uart Download

please input the serial port num:5
```

关于编译、下载的详细步骤，请参考[](/quickstart/get-started.md)的相关介绍。


### 例程输出结果展示

串口打印如下log

![text](./assets/image1.png)

如果开启了`CONFIG_JPEGD_USING_LCD`的话，lcd也会有图片显示。

## 运行流程

1. 从内置 JPEG 头读取实际宽高。
2. 初始化配置 JPEGD handle。
3. 查询工作缓冲区和三个输出平面的实际大小，检查静态容量。
4. 调用 `HAL_JPEGD_Decode`解码。
6. 启用 LCD 显示时，仅将实际 100×100 区域从 YUV420 转换为 RGB565显示。
7. 反初始化 JPEGD，并打印 `JPEGD example PASSED`。


## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |08/2026 |初始版本 |
| | | |
