# LVGL v8 弧形图片（Arcimg）

`lvsf_arcimg` 是 SiFli 基于 LVGL `lv_img` 封装的自定义弧形图片控件，用 SRAM 遮罩把一张图片揭示成一段圆弧（弧形进度条）。它支持设置圆心、外半径、线宽、背景弧角度范围，并以浮点精度驱动当前显示角度，可带动画过渡。

- 主要头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_arcimg.h_
- 依赖控件：LVGL img（`LV_USE_IMG`）

```{note}
该控件暂无对应 `lvgl_v8_arcimg` example，详细用法参考头文件注释及 SDK 配置。
```

## 功能简介

- 用 SRAM 遮罩把图片揭示成弧形（圆弧进度条）。
- 可配置圆心坐标、外半径、弧线宽度。
- 可设置背景弧的起止角度与方向（顺时针/逆时针）。
- 支持浮点精度的当前角度设置，并支持动画过渡时间。
- 同时提供整数角度设置接口。
- 支持多个 SRAM 遮罩缓冲。

## 使用场景

- 弧形/环形进度条、仪表盘弧段。
- 表盘上的图片式弧形指示。
- 带平滑动画过渡的弧形数值显示。

## 支持的开发板

见对应 lvgl_v8 example 或 SDK 配置。

## 配置与初始化

`lvsf_arcimg` 依赖 LVGL img 组件。在 menuconfig 中确认已开启：

```none
CONFIG_LV_USE_IMG=y
```

BSP 中通过 `LVSF_USE_ARCIMG` 宏控制该控件的编译。

创建对象的最小流程：

```c
#include "lvsf_arcimg.h"

lv_obj_t *arcimg = lv_arcimg_create(lv_scr_act());
```

## API 说明

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_arcimg_create(lv_obj_t *parent)` | 创建弧形图片控件 | `parent`：父对象；返回创建的控件指针 |
| `void lv_arcimg_set_param(lv_obj_t *arcimg, lv_coord_t cent_x, lv_coord_t cent_y, uint16_t r, uint16_t w, uint8_t buf_cnt)` | 设置弧形几何参数与 SRAM 遮罩缓冲数量 | `arcimg`：控件对象；`cent_x`/`cent_y`：圆心坐标；`r`：弧外半径；`w`：弧线宽度；`buf_cnt`：SRAM 遮罩缓冲数量 |
| `void lv_arcimg_set_bg_angles(lv_obj_t *arcimg, lv_coord_t start_angle, lv_coord_t end_angle, uint8_t clockwise)` | 设置背景弧角度范围 | `start_angle`/`end_angle`：起止角度（度）；`clockwise`：方向标志，1 顺时针，0 逆时针 |
| `void lv_arcimg_set_angle(lv_obj_t *arcimg, float angle, uint32_t time)` | 以浮点精度设置当前显示角度 | `angle`：当前弧跨度角（度，支持浮点）；`time`：动画时间（ms），0 表示立即生效 |
| `void lv_arcimg_set_angle_int(lv_obj_t *arcimg, lv_coord_t angle, uint32_t time)` | 以整数设置当前显示角度 | `angle`：当前弧跨度角（度，整数）；`time`：动画时间（ms），0 表示立即生效 |
| `float lv_arcimg_get_angle(lv_obj_t *arcimg)` | 获取当前显示角度 | 返回当前弧跨度角（度） |

## 典型用法

根据头文件接口，典型使用流程为：创建对象 → 设置几何参数 → 设置背景弧角度范围 → 驱动当前角度：

```c
#include "lvsf_arcimg.h"

lv_obj_t *arcimg = lv_arcimg_create(parent);

/* 圆心(70,70)，外半径 60，线宽 8，2 个 SRAM 遮罩缓冲 */
lv_arcimg_set_param(arcimg, 70, 70, 60, 8, 2);

/* 背景弧从 135° 到 405°（即一圈），顺时针 */
lv_arcimg_set_bg_angles(arcimg, 135, 405, 1);

/* 当前显示 90° 弧跨度，用 500ms 动画过渡 */
lv_arcimg_set_angle(arcimg, 90.0f, 500);
```

详细用法参考头文件注释。

## 效果展示

该控件暂无对应 example，运行 `lvgl_v8` 相关示例或 SDK 配置可查看实际效果。

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- 源码路径：`middleware/lvgl/lvsf/gui_widgets/lvsf_arcimg.h`
