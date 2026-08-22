# LVGL v8 Imgbar 示例

源码路径: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_imgbar`

## 概述

本例程演示 `lvsf_imgbar` 控件：把一张前景图片按数值裁切显示，像一个**用图片做填充的进度条**。创建一个前景 `lv_img`，交给 imgbar(`lv_imgbar_set_img_fg`，会把 imgbar 尺寸设成图片大小)，选填充方向，设数值范围，再驱动数值即可。本例程在前景后面放一条灰色轨道表示未填充部分，数值由定时器在 0~100 之间往返扫动。

要点：
- `lv_imgbar_set_dir()` 设填充方向(`BAR_DIR_LEFT_TO_RIGTH` / `RIGTH_TO_LEFT` / `TOP_TO_BOTTOM` / `BOTTOM_TO_TOP`)。
- 数值范围用 `lv_obj_set_range_value()` 设定，`lv_imgbar_set_value()` 按该范围裁切前景。
- `set_img_fg` 会把前景包进一个内部容器(即前景的新父对象)。该容器默认有内边距且可滚动，会把前景挤出可视区；取 `lv_obj_get_parent(fg)` 拿到它，清掉内边距/边框、禁用滚动，并把前景钉到 `(0,0)`，填充才会齐平贴边显示。

## 用法（关键 API）

```c
lv_obj_t *imgbar = lv_imgbar_create(parent);
lv_obj_clear_flag(imgbar, LV_OBJ_FLAG_SCROLLABLE);

lv_obj_t *fg = lv_img_create(imgbar);
lv_img_set_src(fg, &fg_img_dsc);             /* 前景图片 */
lv_obj_refr_size(fg);
lv_imgbar_set_img_fg(imgbar, fg);            /* 把 imgbar 尺寸设成前景图大小 */

/* 处理内部容器: 清内边距/边框、禁滚动、前景钉左上角 */
lv_obj_t *fg_box = lv_obj_get_parent(fg);
lv_obj_set_style_pad_all(fg_box, 0, 0);
lv_obj_set_style_border_width(fg_box, 0, 0);
lv_obj_clear_flag(fg_box, LV_OBJ_FLAG_SCROLLABLE);
lv_obj_set_pos(fg, 0, 0);

lv_imgbar_set_dir(imgbar, BAR_DIR_LEFT_TO_RIGTH);
lv_obj_set_range_value(imgbar, 0, 100);
lv_imgbar_set_value(imgbar, 60);             /* 数值->裁切宽度: 60 即露出 60% 前景 */
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

例程启动后屏幕中央有一条灰色轨道，上面是蓝色填充条。数值由定时器在 0~100 之间往返驱动，所以蓝色填充**反复从左向右增长到铺满、再缩回**。

## 异常诊断

如有任何技术疑问，请在 GitHub 上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
