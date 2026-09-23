# LVGL v8 图像进度条（Imgbar）

`lvsf_imgbar` 是 SiFli 基于 LVGL img 封装的自定义进度条控件，把一张前景图片按数值裁切显示，像一个**用图片做填充的进度条**。创建一个前景 `lv_img`，交给 imgbar（`lv_imgbar_set_img_fg` 会把 imgbar 尺寸设成图片大小），选填充方向，设数值范围，再驱动数值即可。

- 主要头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_imgbar.h_
- 数值范围设置：_middleware/lvgl/lvsf/gui_widgets/lvsf_obj_ext.h_（`lv_obj_set_range_value`）
- 依赖控件：LVGL img（`LV_USE_IMG`）
- 示例工程：_example/multimedia/lvgl/lvgl_v8_imgbar_

## 功能简介

- 用前景图片按数值裁切，实现图片填充的进度条。
- 支持四种填充方向：从左到右、从右到左、从上到下、从下到上。
- 支持条形（BAR）和开关（SWITCH）两种模式。
- 支持拖动与释放修改进度，支持进度变化动画。
- 支持背景图、前景图、指示器图三层图片。
- 支持进度变化用户回调。

## 使用场景

- 图片填充样式的电量条、音量条、亮度条。
- 带自定义纹理/渐变填充的进度指示。
- 可拖动的滑动开关、滑块。
- 带指示器旋钮的进度条。

## 支持的开发板

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## 配置与初始化

`lvsf_imgbar` 依赖 LVGL img 组件。在 menuconfig 中确认已开启：

```none
CONFIG_LV_USE_IMG=y
```

BSP 中通过 `LVSF_USE_IMGBAR` 宏控制该控件的编译。

创建对象的最小流程：

```c
#include "lvsf_imgbar.h"

lv_obj_t *imgbar = lv_imgbar_create(lv_scr_act());
```

## API 说明

### 枚举与回调类型

| 类型 | 取值 | 说明 |
| --- | --- | --- |
| `lv_imgbar_dir_t` | `BAR_DIR_LEFT_TO_RIGTH` | 从左到右 |
| | `BAR_DIR_RIGTH_TO_LEFT` | 从右到左 |
| | `BAR_DIR_TOP_TO_BOTTOM` | 从上到下 |
| | `BAR_DIR_BOTTOM_TO_TOP` | 从下到上 |
| `lv_imgbar_mode_t` | `IMG_BAR_MODE_BAR` | 条形模式 |
| | `IMG_BAR_MODE_SWITCH` | 开关模式 |
| `lv_imgbar_process_cb_t` | `void (*)(lv_obj_t *obj, uint8_t percent)` | 进度变化处理回调；`obj`：图像条对象；`percent`：当前百分比 |

### 创建与刷新

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_imgbar_create(lv_obj_t *parent)` | 创建图像条对象 | `parent`：父对象；成功返回对象指针，失败返回 `NULL` |
| `void lv_imgbar_refresh_timer(lv_timer_t *timer)` | 刷新定时器回调 | `timer`：定时器对象；定时刷新图像条显示 |

### 方向、模式与交互

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `void lv_imgbar_set_dir(lv_obj_t *img, lv_imgbar_dir_t dir)` | 设置填充方向 | `dir`：方向（`lv_imgbar_dir_t`） |
| `void lv_imgbar_set_drag(lv_obj_t *imgbar, bool en)` | 启用/禁用拖动 | `en`：`true` 启用拖动和释放修改效果 |
| `void lv_imgbar_set_mode(lv_obj_t *img, lv_imgbar_mode_t mode)` | 设置显示模式 | `mode`：条形或开关模式 |
| `void lv_imgbar_set_user_cb(lv_obj_t *img, lv_imgbar_process_cb_t user_cb)` | 设置进度变化回调 | `user_cb`：用户回调函数指针 |

### 数值设置

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `void lv_imgbar_set_value(lv_obj_t *img, int32_t value)` | 设置进度值（Q24.8 格式） | `value`：Q24.8 格式数值；按 `lv_obj_set_range_value()` 设定的范围裁切前景 |
| `void lv_imgbar_set_value2(lv_obj_t *imgbar, int32_t value)` | 设置进度值（整数格式） | `value`：整数值 |

### 图片与指示器

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `void lv_imgbar_set_img_fg(lv_obj_t *imgbar, lv_obj_t *img_fg)` | 设置前景图像 | `img_fg`：前景图像对象；会把 imgbar 尺寸设成前景图大小，并将前景包进内部容器 |
| `void lv_imgbar_set_img_bg(lv_obj_t *imgbar, lv_obj_t *img_bg)` | 设置背景图像 | `img_bg`：背景图像对象 |
| `void lv_imgbar_set_img_indicator(lv_obj_t *imgbar, lv_obj_t *img_indicator)` | 设置指示器图像 | `img_indicator`：指示器图像对象 |
| `lv_obj_t *lv_imgbar_get_img_bg(lv_obj_t *imgbar)` | 获取背景图像 | 返回背景图像对象指针 |
| `lv_obj_t *lv_imgbar_get_img_indicator(lv_obj_t *imgbar)` | 获取指示器图像 | 返回指示器图像对象指针 |
| `void lv_imgbar_set_indicator_offset(lv_obj_t *img, lv_coord_t x, lv_coord_t y)` | 设置指示器偏移 | `x`/`y`：X/Y 偏移 |
| `void lv_imgbar_set_indicator_offset_x(lv_obj_t *imgbar, lv_coord_t x)` | 设置指示器 X 偏移 | `x`：X 偏移 |
| `void lv_imgbar_set_indicator_offset_y(lv_obj_t *imgbar, lv_coord_t y)` | 设置指示器 Y 偏移 | `y`：Y 偏移 |

## 典型用法

```c
#include "lvsf_imgbar.h"
#include "lvsf_obj_ext.h"   /* lv_obj_set_range_value */

lv_obj_t *imgbar = lv_imgbar_create(parent);
lv_obj_clear_flag(imgbar, LV_OBJ_FLAG_SCROLLABLE);

lv_obj_t *fg = lv_img_create(imgbar);
lv_img_set_src(fg, &fg_img_dsc);          /* 前景图片 */
lv_obj_refr_size(fg);
lv_imgbar_set_img_fg(imgbar, fg);         /* 把 imgbar 尺寸设成前景图大小 */

/* 处理内部容器：清内边距/边框、禁滚动、前景钉左上角 */
lv_obj_t *fg_box = lv_obj_get_parent(fg);
lv_obj_set_style_pad_all(fg_box, 0, 0);
lv_obj_set_style_border_width(fg_box, 0, 0);
lv_obj_clear_flag(fg_box, LV_OBJ_FLAG_SCROLLABLE);
lv_obj_set_pos(fg, 0, 0);

lv_imgbar_set_dir(imgbar, BAR_DIR_LEFT_TO_RIGTH);
lv_obj_set_range_value(imgbar, 0, 100);
lv_imgbar_set_value(imgbar, 60);          /* 数值->裁切宽度：60 即露出 60% 前景 */
```

```{warning}
`lv_imgbar_set_img_fg()` 会把前景包进一个内部容器（即前景的新父对象）。该容器默认有内边距且可滚动，会把前景挤出可视区。需通过 `lv_obj_get_parent(fg)` 拿到它，清掉内边距/边框、禁用滚动，并把前景钉到 `(0,0)`，填充才会齐平贴边显示。
```

## 效果展示

运行 `lvgl_v8_imgbar` example 可查看实际效果：屏幕中央有一条灰色轨道，上面是蓝色填充条。数值由定时器在 0~100 之间往返驱动，蓝色填充反复从左向右增长到铺满、再缩回。

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [LVGL v8 Image 文档](https://docs.lvgl.io/8.3/widgets/img.html)
- 源码路径：`middleware/lvgl/lvsf/gui_widgets/lvsf_imgbar.h`
- 示例路径：`example/multimedia/lvgl/lvgl_v8_imgbar`
