# LVGL v8 GIF 动图（gif / gif_anim）

SiFli SDK 在 LVGL v8 上提供了 GIF 动画播放能力，由两个头文件组成：

- **`lvsf_gif.h`**：基础 GIF 控件（`lv_sfgif_*`），提供 GIF 解码播放、封面显示、暂停/恢复、缩放等控制。
- **`lvsf_gif_anim.h`**：在基础控件之上封装的简易 `gif_anim` 控件（`lvsf_gif_anim_*`），额外提供封面图、延迟启动、显示层级和整体缩放能力。

适用于全场景动效开发。底层 `lv_gif_dec_*` 解码接口由控件内部调用，业务代码应使用 `lv_sfgif_*` / `lvsf_gif_anim_*` 公开接口。

## 功能简介

- 解码并播放 GIF 动画，支持封面（首帧）图片显示。
- 支持暂停 / 恢复播放与关闭释放资源。
- 支持延迟启动播放、可配置帧间隔。
- gif_anim 控件支持按宽高比缩放、设置显示层级、开关背景色处理。
- 支持 GIF 缩放（`lv_sfgif_set_zoom`，256 表示原始尺寸）。

## 使用场景

- 应用启动/加载动效、充电动画、页面背景动效。
- 需要先显示封面图、延时后再播放 GIF 的场景。
- 需要把 GIF 置于前景/背景层级的界面。

## 支持的开发板

通用 LVGL v8 例程支持的平台，55x 之后的开发板均可使用（如 58x、56x、52x）。

## 配置与初始化

基础 GIF 控件与 gif_anim 控件由 gui_widgets 组件提供，随 LVGL v8 工程编译启用：

```c
#include "lvsf/gui_widgets/lvsf_gif.h"
#include "lvsf/gui_widgets/lvsf_gif_anim.h"
```

gif_anim 层级枚举（`lvsf_gif_layer_t`）：`LVSF_GIF_LAYER_DEFAULT`、`LVSF_GIF_LAYER_FOREGROUND`、`LVSF_GIF_LAYER_BACKGROUND`。

## API 说明

### 基础 GIF 控件（`lvsf_gif.h`）

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_sfgif_create(lv_obj_t *parent)` | 创建 GIF 控件实例 | `parent`：父对象；返回控件指针 |
| `void lv_sfgif_open(lv_obj_t *gif, const char *gif_data, const char *src_img, lv_coord_t x, lv_coord_t y, uint32_t anim_time, uint16_t period)` | 打开 GIF 资源，可同时设置封面图、位置、延迟启动与帧间隔 | `gif_data`：GIF 数据；`src_img`：封面图数据，可为 `NULL`；`x`/`y`：位置；`anim_time`：延迟启动时间，0 立即播放；`period`：帧间隔，0 用默认刷新周期 |
| `void lv_sfgif_resume(lv_obj_t *gif)` | 启动或恢复 GIF 播放 | 无返回值 |
| `void lv_sfgif_pause(lv_obj_t *gif)` | 暂停 GIF 播放，保留控件对象 | 无返回值 |
| `void lv_sfgif_close(lv_obj_t *gif)` | 关闭 GIF 资源，释放内部 GIF 或封面对象 | 无返回值 |
| `void lv_sfgif_set_zoom(lv_obj_t *gif, uint16_t zoom)` | 设置 GIF 缩放比例 | `zoom`：LVGL 图片缩放值，256 表示原始尺寸 |

### gif_anim 扩展控件（`lvsf_gif_anim.h`）

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `lvsf_gif_anim_t *lvsf_gif_anim_init(lv_obj_t *parent, const void *gif_data, const void *src_img, lv_coord_t x, lv_coord_t y, uint32_t duration, uint32_t delay)` | 创建带封面、延时播放能力的 GIF 控件 | `gif_data`：GIF 数据；`src_img`：封面图；`x`/`y`：位置；`duration`：帧播放间隔；`delay`：启动延时；返回控件实例 |
| `void lvsf_gif_anim_set_zoom(lvsf_gif_anim_t *gif_anim, int width_zoom, int height_zoom)` | 设置整体缩放 | 最终取宽高缩放中较小值执行整体缩放 |
| `void lvsf_gif_anim_set_layer(lvsf_gif_anim_t *gif_anim, lvsf_gif_layer_t layer)` | 设置显示层级 | `layer`：层级枚举 |
| `void lvsf_gif_anim_enable_bg_color(lvsf_gif_anim_t *gif_anim, bool enable)` | 使能或关闭背景色处理 | `enable`：是否启用背景色 |
| `void lvsf_gif_anim_resume(lvsf_gif_anim_t *gif_anim)` | 启动或恢复播放 | 无返回值 |
| `void lvsf_gif_anim_pause(lvsf_gif_anim_t *gif_anim)` | 暂停播放 | 无返回值 |
| `void lvsf_gif_anim_deinit(lvsf_gif_anim_t *gif_anim)` | 销毁控件实例，释放资源 | 无返回值 |

## 典型用法

基础 GIF 控件：

```c
lv_obj_t *bg_gif = lv_sfgif_create(lv_scr_act());

/* 打开 GIF：无封面图、位置 (0,0)、立即播放、帧间隔 50ms */
lv_sfgif_open(bg_gif,
              APP_GET_gif(img_gif_demo),
              NULL,
              0, 0,
              0,
              50);

lv_obj_align(bg_gif, LV_ALIGN_CENTER, 0, 0);
lv_sfgif_resume(bg_gif);
lv_obj_move_background(bg_gif);
```

gif_anim 控件（带封面图与延时启动），并在页面生命周期中控制：

```c
static lvsf_gif_anim_t *charge_gif = NULL;

static void on_start(void)
{
    charge_gif = lvsf_gif_anim_init(parent,
                                    APP_GET_gif(img_gif_demo),
                                    APP_GET_IMG(img_gif_demo_surface),
                                    0, 0,
                                    LV_DISP_DEF_REFR_PERIOD * 3,
                                    100);
}

static void on_resume(void)  { lvsf_gif_anim_resume(charge_gif); }
static void on_pause(void)  { lvsf_gif_anim_pause(charge_gif); }
static void on_stop(void)   { lvsf_gif_anim_deinit(charge_gif); charge_gif = NULL; }
```

## 效果展示

```{image} ../../../assets/lvgl_v8/gif_demo_src.png
:alt: gif 示例资源
```

```{image} ../../../assets/lvgl_v8/gif_demo.gif
:alt: gif 播放效果
```

## 注意事项

- GIF 图像不支持透明度；如需透明效果，可使用 APNG 控件或 seqframe 序列帧控件。
- 基础 GIF 控件的公开入口是 `lv_sfgif_*`，不要直接依赖底层 `lv_gif_dec_*` 接口。
- 页面销毁或切换时，建议调用 `lv_sfgif_close()` 或 `lvsf_gif_anim_deinit()` 释放资源。

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- 源码路径：`middleware/lvgl/lvsf/gui_widgets/lvsf_gif.h`、`middleware/lvgl/lvsf/gui_widgets/lvsf_gif_anim.h`
