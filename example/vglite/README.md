# vglite 演示程序

源码路径：example/vglite

## 支持的平台

+ sf32lb58-lcd系列

## 概述

本例程演示基于 VG Lite 硬件加速的图形渲染功能，包含4种演示效果。支持通过 `demo_run_handler` 命令动态切换不同的演示效果。

## 演示效果

| ID | 名称 | 说明 |
|----|------|------|
| 0 | linear_grad | 线性渐变图案，显示10帧 |
| 1 | glyphs | 字体渲染，显示大小字体 |
| 2 | rotate | 旋转指针图片，显示36帧 |
| 3 | coverflow | 翻唱片3D翻转效果，持续执行 |

## 例程的使用

### 编译和烧录

切换到例程 project 目录，运行 scons 命令执行编译：

```
scons -j20 --board=sf32lb58-lcd_n16r32n1_dpi
```

运行 `build_sf32lb58-lcd_n16r32n1_dpi_hcpu\uart_download.bat`，按提示选择端口即可进行下载。

### 运行演示

设备启动后，串口会打印可用演示列表：

```
========== Available Demos ==========
0: linear_grad  - Linear gradient demo
1: glyphs       - Glyphs render demo
2: rotate       - Rotate image demo
3: coverflow    - Coverflow effect demo
======================================

Usage: demo_run_handler <0-3>
Example: demo_run_handler 0
```

使用 `demo_run_handler` 命令运行指定演示：

| 命令 | 说明 |
|------|------|
| `demo_run_handler 0` | 运行线性渐变演示 |
| `demo_run_handler 1` | 运行字体渲染演示 |
| `demo_run_handler 2` | 运行旋转指针演示 |
| `demo_run_handler 3` | 运行翻唱片演示 |

### 动态切换

在不同演示之间动态切换：

```
msh />demo_run_handler 2    # 启动旋转指针
msh />demo_run_handler 0    # 切换到线性渐变（自动停止旋转）
msh />demo_run_handler 3    # 切换到翻唱片（自动停止渐变）
```

## 例程的预期结果

### linear_grad (ID: 0)

显示一个星形路径的线性渐变图案，执行10帧后自动结束：

```
Render size: 192 x 240, 2002fcc0
frame 0 done
frame 1 done
...
frame 9 done
```

### glyphs (ID: 1)

分两帧显示文字渲染效果，第一帧小字体，第二帧放大字符：

```
Framebuffer size: 224 x 240
(显示 "Hello, Verisilicon!" 文字)
```

### rotate (ID: 2)

显示一个指针图片旋转动画，执行36帧（约6秒）：

```
Framebuffer size: 224 x 240
rot:1
rot:2
...
rot:36
```

### coverflow (ID: 3)

显示多张图片的3D翻唱片效果，持续执行直到手动切换：

```
Framebuffer size: 320 x 272
fps: 56.15
fps: 56.15
...
```

## 异常诊断

1. 如果切换演示时出现 Assert 失败，等待上一演示完全结束后再执行切换
2. 如需查看内存使用情况，可使用 `list_mem` 命令
3. 调试信息可通过 `list_thread` 查看各线程状态

## 更新记录

| 版本 | 日期 | 发布说明 |
|:---|:---|:---|
| 0.0.1 | 04/2026 | 初始版本，支持动态切换演示 |
