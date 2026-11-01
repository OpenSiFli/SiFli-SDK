# LVGL v8 图像数组（Imgarray）

`lvsf_imgarray` 是 SiFli 基于 `lvsf_baseimg` 封装的自定义控件，用一组图片拼出一个数值，类似数码管/翻牌显示。它能显示完整的数值形态——**负号 + 整数位 + 小数点 + 小数位 + 单位**，每个字形都是一张图片。给它一组字形图片（0~9 以及小数点、单位等）、整数/小数位数、小数点和单位字形的下标，再喂一个数值，它就按 `[负号][整数].[小数][单位]` 用这些图片拼出来。

- 主要头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_imgarray.h_
- 基类与类型定义：_middleware/lvgl/lvsf/gui_widgets/lvsf_baseimg.h_
- 依赖控件：LVGL img（`LV_USE_IMG`）
- 示例工程：_example/multimedia/lvgl/lvgl_v8_imgarray_

## 功能简介

- 用图片字形拼出数值，支持负号、整数位、小数点、小数位、单位。
- 支持多种数值模式：`BASEIMG_TYPE_ARRAY_INDEX`（数值即图片序号）、`BASEIMG_TYPE_ARRAY_Q248`（Q24.8 定点，支持小数）等。
- 可配置整数位/小数位数量、小数点/负号/单位/空字形在字形数组中的下标。
- 支持前导零、尾随零显示开关。
- 支持图像之间的间隔设置。
- 支持三种源数组设置方式：内存描述符数组、文件路径、单源序列帧。

## 使用场景

- 数码管风格的数值读数（电量、心率、温度、百分比）。
- 需要自定义字形图片的翻牌/拨盘数字显示。
- 表盘、仪表盘上的实时数字。

## 支持的开发板

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## 配置与初始化

`lvsf_imgarray` 依赖 LVGL img 组件。在 menuconfig 中确认已开启：

```none
CONFIG_LV_USE_IMG=y
```

BSP 中通过 `LVSF_USE_IMGARRAY` 宏控制该控件的编译。

创建对象的最小流程：

```c
#include "lvsf_imgarray.h"
#include "lvsf_baseimg.h"

lv_obj_t *ia = lv_imgarray_create(lv_scr_act());
```

## API 说明

### 创建与刷新

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_imgarray_create(lv_obj_t *parent)` | 创建图像数组对象 | `parent`：父对象；成功返回对象指针，失败返回 `NULL` |
| `void lv_imgarray_refresh_timer(lv_timer_t *timer)` | 刷新定时器回调 | `timer`：定时器对象；定时刷新图像数组显示 |

### 源数组设置

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `void lv_imgarray_set_src_array(lv_obj_t *img, const lv_img_dsc_t **dsc_array, int16_t index_start, int16_t index_end)` | 设置源数组（内存描述符） | `img`：图像数组对象；`dsc_array`：描述符数组；`index_start`/`index_end`：起始/结束索引 |
| `void lv_imgarray_set_src_array2(lv_obj_t *img, char *file_path, lv_img_file_data_t *dsc_array, int16_t index_start, int16_t index_end)` | 通过文件路径设置源数组 | `file_path`：文件路径；`dsc_array`：文件描述符数组；`index_start`/`index_end`：起止索引 |
| `void lv_imgarray_set_src_array3(lv_obj_t *img, const void *src, int16_t index_start, int16_t index_end)` | 设置序列帧图片源和序号 | `src`：图片源；`index_start`/`index_end`：起止索引 |

### 布局与字形配置

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `void lv_imgarray_set_img_type(lv_obj_t *img, lv_baseimg_type_t img_type)` | 设置图像类型 | `img_type`：图像类型（`BASEIMG_TYPE_ARRAY_INDEX` / `BASEIMG_TYPE_ARRAY_Q248` 等） |
| `void lv_imgarray_set_int_num(lv_obj_t *img, uint8_t num)` | 设置整数位数量 | `num`：整数位图像数量 |
| `void lv_imgarray_set_float_num(lv_obj_t *img, uint8_t num)` | 设置小数位数量 | `num`：小数位图像数量 |
| `void lv_imgarray_set_unit_img(lv_obj_t *img, lv_obj_t *unit_img)` | 设置单位图像对象 | `unit_img`：单位图像对象指针 |
| `void lv_imgarray_set_point_idx(lv_obj_t *img, uint16_t idx)` | 设置小数点字形索引 | `idx`：小数点在字形数组中的索引 |
| `void lv_imgarray_set_negative_idx(lv_obj_t *img, uint16_t idx)` | 设置负号字形索引 | `idx`：负号在字形数组中的索引 |
| `void lv_imgarray_set_unit_idx(lv_obj_t *img, uint16_t idx)` | 设置单位字形索引 | `idx`：单位在字形数组中的索引 |
| `void lv_imgarray_set_empty_idx(lv_obj_t *img, uint16_t idx)` | 设置空字形索引 | `idx`：空位在字形数组中的索引 |
| `void lv_imgarray_set_leading_zero(lv_obj_t *img, bool leading_zero)` | 设置前导零显示 | `leading_zero`：`true` 显示，`false` 不显示 |
| `void lv_imgarray_set_trailing_zero(lv_obj_t *img, bool trailing_zero)` | 设置尾随零显示 | `trailing_zero`：`true` 显示，`false` 不显示 |
| `void lv_imgarray_set_interval(lv_obj_t *img, int16_t interval)` | 设置图像间隔 | `interval`：图像之间的间隔值 |
| `uint16_t lv_imgarray_get_src_num(lv_obj_t *img)` | 获取源数量 | 返回源数量 |

### 数值设置

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `void lv_imgarray_set_value(lv_obj_t *img, int32_t value)` | 设置数值（Q24.8 格式） | `value`：Q24.8 格式数值；Q24.8 模式下传 `值 * 256` |
| `void lv_imgarray_set_value2(lv_obj_t *img, int32_t value)` | 设置数值（整数格式） | `value`：整数值 |

## 典型用法

下面示例显示一个像 `36.5%` 的测量值（2 位整数 + 1 位小数 + 小数点 + 百分号单位）：

```c
#include "lvsf_baseimg.h"
#include "lvsf_imgarray.h"

/* glyph_arr: 0..9 数字 + "."(下标10) + "%"(下标11) */
extern const lv_img_dsc_t *glyph_arr[12];

void demo_imgarray_init(void)
{
    lv_obj_t *ia = lv_imgarray_create(lv_scr_act());
    lv_imgarray_set_img_type(ia, BASEIMG_TYPE_ARRAY_Q248);   /* Q24.8：支持小数 */
    lv_imgarray_set_src_array(ia, glyph_arr, 0, 11);        /* 先存字形数组 */
    lv_imgarray_set_interval(ia, 2);
    lv_imgarray_set_leading_zero(ia, true);
    lv_imgarray_set_trailing_zero(ia, true);
    lv_imgarray_set_int_num(ia, 2);                         /* 2 位整数 */
    lv_imgarray_set_float_num(ia, 1);                       /* 1 位小数 */
    lv_imgarray_set_point_idx(ia, 10);                      /* 小数点用下标 10 */
    lv_imgarray_set_unit_idx(ia, 11);                       /* 单位用下标 11 */
    lv_obj_center(ia);
    lv_imgarray_set_value(ia, 365 * 256 / 10);              /* 36.5 (Q24.8)：显示 "36.5%" */
}
```

```{warning}
调用顺序很重要：必须先 `lv_imgarray_set_src_array()` 存好字形数组，再调用 `set_int_num()` / `set_point_idx()` / `set_unit_idx()` 等——这些接口会创建对应图片单元并套用已存数组。顺序反了会因数组未就绪而异常。
```

## 效果展示

运行 `lvgl_v8_imgarray` example 可查看实际效果：屏幕中央显示一个带单位的小数（如 `38.5%`），由整数位、小数点、小数位、单位各自的图片拼成。定时器把数值在 20.0~80.0% 之间往返扫动，读数不断变化。

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [LVGL v8 Image 文档](https://docs.lvgl.io/8.3/widgets/img.html)
- 源码路径：`middleware/lvgl/lvsf/gui_widgets/lvsf_imgarray.h`
- 示例路径：`example/multimedia/lvgl/lvgl_v8_imgarray`
