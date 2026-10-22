# LVGL v8 multslider 滑动条

`lvsf_multslider` 是 SiFli 在 LVGL v8 基础上封装的数值滑动条（slider）控件，用于直观地调节一个单维度数值。它支持自定义数值范围、显示文本标签，支持动画或即时方式更新数值，常用于音量、亮度、参数阈值等设置项。

- 主要头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_multslider.h_
- 示例工程：_example/multimedia/lvgl/lvgl_v8_multslider_

## 功能简介

- 在 LVGL 原生 slider 基础上封装，可设置当前值、最小值、最大值。
- 支持设置文本标签（如“音量”），并在控件上显示。
- 可选择是否以动画方式过渡到目标值。
- 可通过标准 LVGL 样式接口自定义背景、indicator、knob 的颜色与透明度。

## 使用场景

- 音量、亮度、对比度等系统设置项的数值调节。
- 参数阈值、进度类单维度数值的拖拽调节。
- 需要在滑块上叠加文本标签（“-”“+”或名称）的横向滑动条。

## 支持的开发板

参考例程 `example/multimedia/lvgl/lvgl_v8_multslider` 在以下开发板验证：

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

板型工程通过 `scons --board=<board>` 生成。

## 配置与初始化

在 menuconfig 中启用对应组件（默认已开启）：

```none
CONFIG_LVSF_USE_MULTSLIDER=y
```

创建后设置大小、范围、初始值，并用标准 LVGL 样式接口调整各部分外观：

```c
#include "lvsf_multslider.h"

lv_obj_t *slider = lv_multslider_create(parent);
lv_obj_set_size(slider, LV_HOR_RES_MAX - 40, 60);
lv_obj_align(slider, LV_ALIGN_CENTER, 0, 0);

/* 数值范围与初始值 */
lv_multslider_set_range(slider, 0, 100);
lv_multslider_set_value(slider, 50, LV_ANIM_ON);
lv_multslider_set_txt(slider, "音量");

/* 样式：背景 / indicator / knob */
lv_obj_set_style_bg_color(slider, LV_COLOR_BLACK, LV_PART_MAIN);
lv_obj_set_style_bg_opa(slider, LV_OPA_100, LV_PART_MAIN);
lv_obj_set_style_bg_color(slider, LV_COLOR_GRAY, LV_PART_INDICATOR);
lv_obj_set_style_bg_opa(slider, LV_OPA_50, LV_PART_INDICATOR);
lv_obj_set_style_bg_color(slider, LV_COLOR_GRAY, LV_PART_KNOB);
lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_KNOB);
```

## API 说明

以下函数签名均逐字来自 _lvsf_multslider.h_。

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_multslider_create(lv_obj_t *parent)` | 创建 multslider 滑动条对象 | `parent`：父对象；返回新建对象指针 |
| `void lv_multslider_set_txt(lv_obj_t *multslider, const char *txt)` | 设置文本标签 | `txt`：要显示的文本字符串 |
| `void lv_multslider_set_value(lv_obj_t *multslider, int32_t value, lv_anim_enable_t anim)` | 设置当前数值 | `value`：目标数值；`anim`：`LV_ANIM_ON` 动画过渡 / `LV_ANIM_OFF` 立即设置 |
| `void lv_multslider_set_range(lv_obj_t *multslider, int32_t min, int32_t max)` | 设置数值范围 | `min`：最小值；`max`：最大值 |
| `int32_t lv_multslider_get_value(lv_obj_t *multslider)` | 获取当前数值 | 返回当前值 |
| `int32_t lv_multslider_get_min_value(lv_obj_t *multslider)` | 获取最小值 | 返回设置的最小值 |
| `int32_t lv_multslider_get_max_value(lv_obj_t *multslider)` | 获取最大值 | 返回设置的最大值 |

```{note}
例程中常见的 `lv_gesture_disable()` / `lv_gesture_enable()` 是页面级右滑返回手势管理，不属于 multslider 控件本身的接口。
```

## 典型用法

完整可运行例程见 `example/multimedia/lvgl/lvgl_v8_multslider`，演示一种子控件位于滑块下方风格的滑动条：

```c
static void on_start(void)
{
    lv_obj_t *parent = lv_scr_act();

    lv_obj_t *slider = lv_multslider_create(parent);
    lv_obj_set_size(slider, LV_HOR_RES_MAX - 40, 60);
    lv_obj_refr_size(slider);
    lv_obj_align(slider, LV_ALIGN_CENTER, 0, 0);

    /* 背景 / indicator / knob 样式 */
    lv_obj_set_style_bg_color(slider, LV_COLOR_BLACK, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(slider, LV_OPA_100, LV_PART_MAIN);
    lv_obj_set_style_bg_color(slider, LV_COLOR_GRAY, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(slider, LV_OPA_50, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(slider, LV_COLOR_GRAY, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_KNOB);

    /* 在滑块上叠加 “-” / “+” / 名称文本 */
    lv_obj_t *lab = lv_label_create(slider);
    lv_label_set_text(lab, "-");
    lv_obj_align(lab, LV_ALIGN_LEFT_MID, 20, 0);

    lab = lv_label_create(slider);
    lv_label_set_text(lab, "+");
    lv_obj_align(lab, LV_ALIGN_RIGHT_MID, -20, 0);

    lab = lv_label_create(slider);
    lv_label_set_text(lab, "multslider");
    lv_obj_align(lab, LV_ALIGN_CENTER, 0, 0);
}
```

读取当前值时调用 `lv_multslider_get_value()` 即可。

## 效果展示

拖拽滑动条调节数值并叠加文本标签：

```{image} ../../../assets/lvgl_v8/multslider.gif
:alt: multslider 效果
:width: 400px
:align: center
```

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- 头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_multslider.h_
- 示例工程：_example/multimedia/lvgl/lvgl_v8_multslider_
