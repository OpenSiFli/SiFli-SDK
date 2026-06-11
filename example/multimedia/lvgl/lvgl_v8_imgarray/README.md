# LVGL v8 Imgarray 示例

源码路径: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_imgarray`

## 概述

本例程演示 `lvsf_imgarray` 控件(基于 `lvsf_baseimg`)：用**图片拼出一个数值**，像数码管/翻牌显示。它能显示完整的数值形态 —— **负号 + 整数位 + 小数点 + 小数位 + 单位**，每个字形都是一张图片。给它一组字形图片(0~9 以及小数点、单位等)、整数/小数位数、小数点和单位字形的下标，再喂一个数值，它就按 `[负号][整数].[小数][单位]` 用这些图片拼出来。本例程显示一个像 `38.5%` 的测量值，数值在 20.0~80.0% 之间往返。

要点：
- 数值模式由 `lv_imgarray_set_img_type()` 决定：`BASEIMG_TYPE_ARRAY_INDEX`(数值为整数)、`BASEIMG_TYPE_ARRAY_Q248`(数值为 Q24.8，**支持小数**)等。本例用 Q24.8。
- **调用顺序**：先 `lv_imgarray_set_src_array()` 存好字形数组，再调 `set_int_num()`/`set_point_idx()`/`set_unit_idx()` —— 这些会创建对应的图片单元并套用已存数组。顺序反了会因数组未就绪而崩。
- `set_int_num()`/`set_float_num()` 设整数/小数位数；`set_point_idx()` 指定小数点字形、`set_unit_idx()` 指定单位字形、`set_negative_idx()` 指定负号字形(都用字形数组里的下标)；`set_leading_zero()`/`set_trailing_zero()` 控制前导/后导零。
- `lv_imgarray_set_value()` 设数值(Q24.8 模式下传 `值 * 256`)。

## 用法（关键 API）

```c
/* glyph_arr: 0..9 数字 + "."(下标10) + "%"(下标11)，本例用 canvas 渲染 */
lv_obj_t *ia = lv_imgarray_create(parent);
lv_imgarray_set_img_type(ia, BASEIMG_TYPE_ARRAY_Q248);  /* Q24.8: 支持小数 */
lv_imgarray_set_src_array(ia, glyph_arr, 0, 11);        /* 先存字形数组 */
lv_imgarray_set_int_num(ia, 2);                         /* 2 位整数 */
lv_imgarray_set_float_num(ia, 1);                       /* 1 位小数 */
lv_imgarray_set_point_idx(ia, 10);                      /* 小数点用下标 10 的字形 */
lv_imgarray_set_unit_idx(ia, 11);                       /* 单位用下标 11 的字形 */

lv_imgarray_set_value(ia, 365 * 256 / 10);             /* 36.5 (Q24.8): 显示 "36.5%" */
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

例程启动后屏幕中央显示一个带单位的小数(如 `38.5%`)，由整数位、小数点、小数位、单位各自的图片拼成。定时器把数值在 20.0~80.0% 之间往返扫动，所以这个读数**不断变化**。

## 异常诊断

如有任何技术疑问，请在 GitHub 上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
