# LVGL v8 Timeline 示例

源码路径: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_timeline`

## 概述

本例程演示 `lvsf_timeline` 动画时间线控件。它是一个**通用的动画编排引擎**：每个元素(element)把一个时间区间 `[start_time, end_time]` 映射到一个数值区间 `[start_value, end_value]`，并在该区间内把插值后的数值喂给一个 exec 回调；回调可以把这个值施加到**任意对象的任意属性**(位置、尺寸、透明度、角度、缩放、颜色……)。多个元素共用同一条时间轴，于是多段动画按时间被编排在一起，用来做复杂的转场/入场动画 —— 平移只是其中一种用法。

要点：
- `lv_timeline_add_element()` 添加一段：给定起止值、起止时间和 exec 回调，时间线运行时按当前时刻把插值后的数值传给回调。
- exec 回调签名 `void (*)(void *var, int32_t value, void *user_data)`，由它把数值施加到目标对象(设坐标、设尺寸、设透明度等)。
- 时间线对象本身不可见(尺寸设 0)；它运行到 `set_time` 设定的总时长后结束并自删，因此每次播放都新建一个。

## 用法（关键 API）

```c
/* 两个回调，各把数值施加到同一对象的不同属性(位置、尺寸) */
static void move_x(void *var, int32_t v, void *ud) { lv_obj_set_x((lv_obj_t *)ud, (lv_coord_t)v); }
static void set_sz(void *var, int32_t v, void *ud) { lv_obj_set_size((lv_obj_t *)ud, v, v); }

lv_obj_t *tl = lv_timeline_create(lv_scr_act());
lv_obj_set_size(tl, 0, 0);
/* (起值, 止值, 起时间, 止时间, exec 回调, ready 回调, var)；后两段时间窗相同 = 并行 */
lv_timeline_add_element(tl, 40, 200, 0, 600, move_x, NULL, box);    /* 0..600    右移 */
lv_timeline_add_element(tl, 40, 70, 600, 1200, set_sz, NULL, box);  /* 600..1200 放大 */
lv_timeline_add_element(tl, 200, 40, 1200, 1900, move_x, NULL, box);/* 1200..1900 移回 ┐ 并行 */
lv_timeline_add_element(tl, 70, 40, 1200, 1900, set_sz, NULL, box); /* 1200..1900 缩回 ┘ */
lv_timeline_set_time(tl, 1900);   /* 总时长 */
lv_timeline_start(tl);            /* 开始播放 */
```

## 支持的开发板

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## 编译和烧录

板子工程在 `project` 目录下，通过指定 board 编译对应板子的工程：
- 例如编译 sf32lb52-lcd_n16r8：在 `project` 目录执行 `scons --board=sf32lb52-lcd_n16r8 -j8`
- 烧录通过 build 目录下的 `download.bat`（或 `uart_download.bat`）进行
- SF32LB52x/SF32LB56x 系列会额外生成 `uart_download.bat`，执行后输入下载 UART 端口号即可

## 模拟器

在 `project` 目录执行 `scons --board=pc_hcpu -j8`，生成 `build_pc_hcpu/main.exe` 直接运行即可看到窗口。
- 需要先按本机 MSVC 配置改好 `SiFli-SDK/msvc_setup.bat`。

## 运行说明

例程启动后屏幕上有一个红色方块，下方是 `Run` 按钮。
- 点击 `Run`：时间线播放约 1.9 秒 —— 方块先平移(0~0.6 秒)，再放大(0.6~1.2 秒)，最后**同时缩小并移回原位**(1.2~1.9 秒，两段动画并行)。
- 再次点击会先复位再重新播放。

## 异常诊断

如有任何技术疑问，请在 GitHub 上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
