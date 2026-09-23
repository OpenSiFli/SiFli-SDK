# LVGL v8 扇形遮罩（Sector）

`lvsf_sector` 是 SiFli 基于 `lv_img` 封装的自定义控件，用一个**扇形（角度）遮罩**把一张图片揭示成饼形——把数值映射到角度，数值越大露出的扇形角度越大。先用 `lv_img_set_src` 设源图，再设角度跨度和数值范围，分配遮罩后用 `lv_sector_set_value` 驱动。

- 主要头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_sector.h_
- 范围设置：_middleware/lvgl/lvsf/gui_widgets/lvsf_obj_ext.h_（`lv_obj_set_range_scale` / `lv_obj_set_range_value`）
- 依赖控件：LVGL img（`LV_USE_IMG`）
- 示例工程：_example/multimedia/lvgl/lvgl_v8_sector_

## 功能简介

- 用角度遮罩把图片揭示成饼形/扇形，数值映射到角度。
- 基于 `lv_img`，直接复用 `lv_img_set_src` 设置源图。
- 支持角度跨度（`lv_obj_set_range_scale`）和数值范围（`lv_obj_set_range_value`）配置。
- 支持拖动修改扇形进度。
- 支持指示器图像及其 X/Y 偏移。
- 通过 `lv_sector_validate()` 分配角度遮罩缓冲。

## 使用场景

- 饼图、环形进度仪表盘。
- 时钟表盘、音量/电量扇形指示。
- 数值到角度映射的圆形进度揭示动画。

## 支持的开发板

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## 配置与初始化

`lvsf_sector` 依赖 LVGL img 组件。在 menuconfig 中确认已开启：

```none
CONFIG_LV_USE_IMG=y
```

BSP 中通过 `LVSF_USE_SECTOR` 宏控制该控件的编译。

创建对象的最小流程：

```c
#include "lvsf_sector.h"
#include "lvsf_obj_ext.h"

lv_obj_t *sector = lv_sector_create(lv_scr_act());
lv_img_set_src(sector, &img_dsc);
```

## API 说明

### 创建与刷新

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_sector_create(lv_obj_t *parent)` | 创建扇形对象 | `parent`：父对象；成功返回对象指针，失败返回 `NULL` |
| `void lv_sector_refresh_timer(lv_timer_t *timer)` | 刷新定时器回调 | `timer`：定时器对象；定时刷新扇形显示 |

### 配置与验证

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `void lv_sector_validate(lv_obj_t *sector)` | 验证并分配角度遮罩缓冲 | `sector`：扇形对象；设好角度跨度和数值范围后必须调用，之后才能 `lv_sector_set_value()` |
| `void lv_sector_refresh_mask_range(lv_obj_t *sector, int32_t min, int32_t max, uint8_t value)` | 刷新遮罩范围 | `min`/`max`：最小/最大值；`value`：当前值 |
| `void lv_sector_set_value(lv_obj_t *sector, int32_t value)` | 设置扇形值 | `value`：按数值范围映射到角度；数值越大露出扇形角度越大 |
| `void lv_sector_set_drag(lv_obj_t *sector, bool en)` | 启用/禁用拖动 | `en`：`true` 允许拖动修改进度 |

### 指示器与图片

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `void lv_sector_set_img_indicator(lv_obj_t *sector, lv_obj_t *img_indicator)` | 设置指示器图像 | `img_indicator`：指示器图像对象 |
| `void lv_sector_set_indicator_offset(lv_obj_t *sector, lv_coord_t x, lv_coord_t y)` | 设置指示器偏移 | `x`/`y`：X/Y 偏移 |
| `void lv_sector_set_indicator_offset_x(lv_obj_t *sector, lv_coord_t x)` | 设置指示器 X 偏移 | `x`：X 偏移 |
| `void lv_sector_set_indicator_offset_y(lv_obj_t *sector, lv_coord_t y)` | 设置指示器 Y 偏移 | `y`：Y 偏移 |

### 查询接口

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `int16_t lv_sector_get_start_angle(lv_obj_t *sector)` | 获取起始角度 | 返回起始角度 |
| `int16_t lv_sector_get_end_angle(lv_obj_t *sector)` | 获取结束角度 | 返回结束角度 |
| `lv_obj_t *lv_sector_get_img_bg(lv_obj_t *sector)` | 获取背景图像 | 返回背景图像对象指针 |
| `lv_obj_t *lv_sector_get_img_indicator(lv_obj_t *sector)` | 获取指示器图像 | 返回指示器图像对象指针 |

## 典型用法

```c
#include "lvsf_sector.h"
#include "lvsf_obj_ext.h"   /* lv_obj_set_range_scale / lv_obj_set_range_value */

lv_obj_t *sector = lv_sector_create(parent);
lv_img_set_src(sector, &img_dsc);            /* 源图（sector 基于 lv_img） */
lv_obj_set_size(sector, 140, 140);
lv_obj_set_style_img_opa(sector, LV_OPA_COVER - 1, LV_PART_MAIN);  /* 防残影 */
lv_obj_set_range_scale(sector, 0, 360);      /* 角度跨度（整圈） */
lv_obj_set_range_value(sector, 0, 100);       /* 数值范围 */
lv_sector_validate(sector);                   /* 分配角度遮罩缓冲 */

lv_sector_set_value(sector, 50);              /* 数值->角度：50 即半圈 180° */
```

```{warning}
- 设好角度跨度和数值范围后，必须先调用 `lv_sector_validate()` 分配角度遮罩缓冲，之后才能 `lv_sector_set_value()`。
- 把图片不透明度设为略低于 `LV_OPA_COVER`（即 `LV_OPA_COVER - 1`）：让底层 `lv_img` 的 cover-check 返回 NOT_COVER，父对象会重绘遮罩挡住的区域，否则被遮住的部分会留下残影。
```

## 效果展示

运行 `lvgl_v8_sector` example 可查看实际效果：屏幕中央有一块橙色图，被扇形遮罩裁成饼形。数值由定时器在 0~100 之间往返驱动，扇形角度反复增大到整圈、再缩回。

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [LVGL v8 Image 文档](https://docs.lvgl.io/8.3/widgets/img.html)
- 源码路径：`middleware/lvgl/lvsf/gui_widgets/lvsf_sector.h`
- 示例路径：`example/multimedia/lvgl/lvgl_v8_sector`
