# LVGL v8 multroller 滚轮选择器

`lvsf_multroller` 是 SiFli 基于 `lvsf_multlist` 封装的声明式滚轮（roller）选择控件。应用只需给它一个以 `'\n'` 分隔的选项字符串，它就会自动排出一个**可循环、滑动后自动吸附**的竖向列表，并在中间焦点区把当前选中项高亮显示；通过 `lv_multroller_get_selected()` 即可读回选中项索引。

它面向“给一组文字选项、让用户滚动选一个”的常见场景（时间选择、参数配置、模式切换等），使用比底层的 `lvsf_mulroller` 简单得多。

- 主要头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_multroller.h_
- 继承自：`lvsf_multlist`，因此 multlist 的全部接口对 multroller 同样可用
- 示例工程：_example/multimedia/lvgl/lvgl_v8_multroller_

## 功能简介

- 传入 `'\n'` 分隔的选项字符串，自动生成竖向可循环滚轮。
- 滑动结束自动吸附对齐，中间焦点区内的选项以高亮色显示。
- 可设置可见选项数量（默认 3）、焦点区宽高与高亮颜色。
- 可编程指定选中项并带动画定位，也可随时读回当前选中索引。
- 继承 multlist，可直接使用其方向、回弹、编码器等高级接口。

## 使用场景

- 时间 / 日期选择器（时、分、月、日等单列或多列组合）。
- 系统设置中的参数项选择（亮度档位、语言、单位、模式等）。
- 任何“从一组预设文本选项中选一个”的滚动选择交互。

## 支持的开发板

参考例程 `example/multimedia/lvgl/lvgl_v8_multroller`（月份选择器）在以下开发板验证：

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

也支持 PC 模拟器（`scons --board=pc_hcpu`）。板型工程通过 `scons --board=<board>` 生成，支持 SF32LB52x / SF32LB56x 等系列。

## 配置与初始化

在 menuconfig 中启用对应组件（默认已开启）：

```none
CONFIG_LVSF_USE_MULTROLLER=y
```

调用顺序建议为：设置大小与字体 / 颜色 → `set_show_cnt()` → `set_options()` → `set_focus_param()`。

```c
#include "lvsf_multroller.h"

/* 选项字符串：按 '\n' 切分，最后一个选项也要以 '\n' 结尾（末尾留一个空行） */
static const char *MONTH_OPTIONS =
    "January\nFebruary\nMarch\nApril\nMay\nJune\n"
    "July\nAugust\nSeptember\nOctober\nNovember\nDecember\n\n";

lv_obj_t *roller = lv_multroller_create(parent);
lv_obj_set_size(roller, 240, 240);
lv_obj_set_style_bg_opa(roller, LV_OPA_TRANSP, 0);              /* 默认不透明，改透明 */
lv_obj_set_style_text_font(roller, &lv_font_montserrat_24, 0);  /* 选项字体 */
lv_obj_set_style_text_color(roller, lv_color_hex(0xBDBDBD), 0);/* 未选中：灰 */
lv_obj_center(roller);

lv_multroller_set_show_cnt(roller, 5);                         /* 可见 5 项 */
lv_multroller_set_options(roller, MONTH_OPTIONS);
lv_multroller_set_focus_param(roller,
                              lv_palette_main(LV_PALETTE_RED), 240, 48); /* 中间焦点区高亮 */
```

```{warning}
选项字符串 `options` 必须以 `'\n'` 分隔，并且**每个选项（含最后一个）都要以 `'\n'` 结尾**，即字符串末尾留一个空行，否则最后一项会被丢弃。`options` 所指字符串需保证生命周期，提前释放会导致滚轮读取无效内存。
```

## API 说明

以下函数签名均逐字来自 _lvsf_multroller.h_。

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_multroller_create(lv_obj_t *parent)` | 创建滚轮选择器对象 | `parent`：父对象；返回新建滚轮指针 |
| `void lv_multroller_set_show_cnt(lv_obj_t *multroller, uint8_t cnt)` | 设置可见选项数量（最小 3，默认 3） | `cnt`：可见选项数；每项尺寸由控件大小按可见数量等分 |
| `void lv_multroller_set_options(lv_obj_t *multroller, const char *options)` | 设置选项字符串 | `options`：以 `'\n'` 分隔的选项，如 `"One\nTwo\nThree\n"`，末尾需留空行 |
| `void lv_multroller_set_selected(lv_obj_t *multroller, uint16_t sel_opt, uint32_t anim_time)` | 编程指定选中项 | `sel_opt`：选项索引（0 ~ 选项数-1）；`anim_time`：动画时长，0 立即定位 |
| `void lv_multroller_set_focus_param(lv_obj_t *multroller, lv_color_t color, uint16_t w, uint16_t h)` | 设置中间焦点区大小与高亮文本色 | `color`：焦点区内文本颜色；`w` / `h`：焦点区宽高 |
| `uint16_t lv_multroller_get_selected(lv_obj_t *multroller)` | 获取当前居中（选中）项索引 | 返回选中项索引（0 ~ 选项数-1） |

```{note}
multroller 继承自 multlist，因此 `lv_multlist_set_dir()`、`lv_multlist_add_flag()`、`lv_multlist_enable_encoder()` 等 multlist 接口均可直接作用于 multroller 对象。例程中常见的 `lv_gesture_disable()` / `lv_gesture_enable()` 是页面级右滑返回手势管理，不属于 multroller 控件接口。
```

## 典型用法

完整可运行例程见 `example/multimedia/lvgl/lvgl_v8_multroller`（月份选择器）。核心用法如下：

```c
void demo_multroller_init(void)
{
    lv_obj_t *scr = lv_scr_act();

    lv_obj_t *roller = lv_multroller_create(scr);
    lv_obj_set_size(roller, 240, 240);
    lv_obj_set_style_bg_opa(roller, LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_font(roller, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(roller, lv_color_hex(0xBDBDBD), 0);
    lv_obj_center(roller);

    lv_multroller_set_show_cnt(roller, 5);
    lv_multroller_set_options(roller, MONTH_OPTIONS);
    lv_multroller_set_focus_param(roller,
                                  lv_palette_main(LV_PALETTE_RED), 240, 48);

    /* 滚轮松手后才吸附，用定时器轮询 get_selected() 同步读数 */
    uint16_t sel = lv_multroller_get_selected(roller);
    lv_multroller_set_selected(roller, 3, 200);   /* 选中第 3 项，200ms 动画 */
}
```

多个滚轮并排组成时间选择器时，各自独立创建并设置方向即可（水平 / 垂直由继承来的 `lv_multlist_set_dir()` 控制）。

## 效果展示

滚轮滚动、自动吸附并在中间焦点区高亮选中项：

```{image} ../../../assets/lvgl_v8/multroller.gif
:alt: multroller 效果
:width: 400px
:align: center
```

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- 头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_multroller.h_
- 示例工程：_example/multimedia/lvgl/lvgl_v8_multroller_
- 相关控件：`lvsf_mulroller`（底层、回调驱动、可深度定制的滚轮）、`lvsf_multlist`（列表容器）
