# LVGL v8 弧形文本（arctext / curvetext）

SiFli SDK 在 LVGL v8 上提供了弧形文本（Arc Text）显示能力，对应 SDK 头文件为 `lvsf_curvetext.h`（控件类名为 `lvsfcurve`）。它在 `lv_canvas` 之上把字符沿圆弧排列绘制，支持设置文本沿圆弧的角度、半径，以及绘制圆弧背景线，常用于手表表盘边缘、环形菜单标题等弧形文字排版场景。

```{note}
solution 文档中使用的 `lv_arctext_*` 接口与当前 SDK v8 头文件 `lvsf_curvetext.h` 中的 `lv_lvsfcurve_*` 命名不同。本文 API 表格以 SDK 头文件为准；`lv_arctext_*` 相关的对齐、等间距、镜像等功能描述可作为设计参考。
```

## 功能简介

- 把文本沿圆弧排列绘制，支持指定起始角度与半径。
- 支持绘制圆弧背景线（半径、起止角度、颜色、线宽）。
- 支持设置旋转中心点（pivot）。
- 依赖 LVGL canvas（`LV_USE_CANVAS`）作为绘制后端。

## 使用场景

- 手表表盘外圈环绕文字。
- 环形菜单 / 圆形控件边缘的标题文字。
- 需要文字沿圆弧排列、配合圆弧刻度线的界面。

## 支持的开发板

通用 LVGL v8 例程支持的平台，55x 之后的开发板均可使用（如 58x、56x、52x）。需在 `lv_conf.h` 中启用 `LV_USE_CANVAS`，并在 `menuconfig` 中启用 `LVSF_USE_CURVE`。

## 配置与初始化

弧形文本依赖 LVGL canvas，需先在 `lv_conf.h` 中开启：

```none
LV_USE_CANVAS  1
```

并在 `menuconfig` 的 `LittlevGL2RTT -> SiFli extend` 中启用 `LVSF_USE_CURVE`。

```c
#include "lvsf/lvsf_curvetext.h"
```

## API 说明

以下接口签名逐字取自 `middleware/lvgl/lvsf/lvsf_curvetext.h`：

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_lvsfcurve_create(lv_obj_t *parent)` | 创建弧形文本（curve）对象 | `parent`：父对象；返回创建的对象指针 |
| `void lv_lvsfcurve_set_buf(lv_obj_t *curve, uint16_t txt_width, uint16_t txt_height)` | 设置弧形文本绘制缓冲区尺寸 | `txt_width`/`txt_height`：文本区域宽高（像素） |
| `void lv_lvsfcurve_set_pivot(lv_obj_t *curve, lv_coord_t x, lv_coord_t y)` | 设置旋转中心点（pivot） | `x`/`y`：中心点坐标 |
| `void lv_lvsfcurve_draw_arc(lv_obj_t *curve, lv_coord_t r, int32_t start_angle, int32_t end_angle, lv_color_t color, lv_coord_t width)` | 绘制圆弧背景线 | `r`：半径；`start_angle`/`end_angle`：起止角度；`color`：颜色；`width`：线宽 |
| `void lv_lvsfcurve_text(lv_obj_t *curve, char *text, int angle, int r, lv_color_t color, int size)` | 沿圆弧绘制文本 | `text`：待显示文本；`angle`：起始角度；`r`：半径；`color`：颜色；`size`：字号 |

## 典型用法

创建弧形文本对象、设置缓冲区与中心点，再沿圆弧绘制文字和圆弧线：

```c
lv_obj_t *arc = lv_lvsfcurve_create(parent);
lv_obj_set_size(arc, LV_HOR_RES_MAX >> 1, LV_HOR_RES_MAX >> 1);
lv_obj_center(arc);

/* 设置绘制缓冲区与旋转中心 */
lv_lvsfcurve_set_buf(arc, LV_HOR_RES_MAX >> 1, LV_HOR_RES_MAX >> 1);
lv_lvsfcurve_set_pivot(arc, LV_HOR_RES_MAX >> 2, LV_HOR_RES_MAX >> 2);

/* 沿圆弧绘制文字：从 0 度方向开始，半径 80，白色 */
lv_lvsfcurve_text(arc, "ARCTEXT", 0, 80, LV_COLOR_WHITE, 20);

/* 绘制一条圆弧背景线 */
lv_lvsfcurve_draw_arc(arc, 80, 0, 360, LV_COLOR_BLUE, 2);
```

## 效果展示

```{image} ../../../assets/lvgl_v8/arctext.gif
:alt: arctext 弧形文本效果
```

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- 源码路径：`middleware/lvgl/lvsf/lvsf_curvetext.h`
