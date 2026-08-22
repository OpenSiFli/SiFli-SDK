# LVGL v8 Sector 示例

源码路径: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_sector`

## 概述

本例程演示 `lvsf_sector` 控件：用一个**扇形(角度)遮罩**把一张图片揭示成饼形 —— 把数值映射到角度，数值越大露出的扇形角度越大。sector 基于 `lv_img`，所以先用 `lv_img_set_src` 设源图，再设角度跨度和数值范围，分配遮罩后用 `lv_sector_set_value` 驱动。本例程让数值在 0~100 之间往返扫动，扇形随之增大、缩小。

要点：
- `lv_obj_set_range_scale(obj, 0, 360)` 设角度跨度(这里整圈)，`lv_obj_set_range_value(obj, 0, 100)` 设数值范围；数值按比例映射到角度。
- 设好范围后必须调用 `lv_sector_validate()` 分配角度遮罩缓冲，之后才能 `lv_sector_set_value()`。
- 把图片不透明度设为略低于 `LV_OPA_COVER`(`LV_OPA_COVER - 1`)：让底层 `lv_img` 的 cover-check 返回 NOT_COVER，父对象会重绘遮罩挡住的区域，否则被遮住的部分会留下残影。

## 用法（关键 API）

```c
lv_obj_t *sector = lv_sector_create(parent);
lv_img_set_src(sector, &img_dsc);            /* 源图(sector 基于 lv_img) */
lv_obj_set_size(sector, 140, 140);
lv_obj_set_style_img_opa(sector, LV_OPA_COVER - 1, LV_PART_MAIN);  /* 防残影 */
lv_obj_set_range_scale(sector, 0, 360);      /* 角度跨度 */
lv_obj_set_range_value(sector, 0, 100);      /* 数值范围 */
lv_sector_validate(sector);                  /* 分配角度遮罩缓冲 */

lv_sector_set_value(sector, 50);             /* 数值->角度: 50 即半圈 180° */
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

例程启动后屏幕中央有一块橙色图，被扇形遮罩裁成饼形。数值由定时器在 0~100 之间往返驱动，所以扇形角度**反复增大到整圈、再缩回**。

## 异常诊断

如有任何技术疑问，请在 GitHub 上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
