# LVGL v8 手势（gesture）

SiFli SDK 在 LVGL v8 上对右滑退出（页面返回）手势进行了统一封装，对应头文件为 `lvsf_gesture.h`。它为页面之间的切换提供统一的触发方式和跟手动画，应用只需在页面生命周期中使能 / 禁止手势，即可实现一致的右滑返回交互。

右滑退出动画根据跟手动画的触发区域大小分为三种形式，区域大小由 `line`（触发线）位置决定：

- 全屏任意位置右滑退出：`line` 位于屏幕右侧；
- 限定区域右滑退出：`line` 靠近屏幕左侧但保留一定距离；
- 全屏右滑退出且不带跟手动画：`line` 位于屏幕左侧。

## 功能简介

- 统一封装右滑退出手势，保证页面切换触发方式一致。
- 支持配置右滑触发区域线位置与是否直接返回。
- 支持使能 / 禁止手势动画，状态切换在进入 idle 后生效。
- 支持手势条（bars）重新对齐与自定义手势图片。

## 使用场景

- 二级界面右滑返回上级页面。
- 与平铺翻页等页面内手势冲突时，在 `on_resume` 中临时禁止右滑退出手势，在 `on_pause` 中恢复。
- 需要限定只有屏幕左侧边缘区域才触发右滑返回的场景。

## 支持的开发板

通用 LVGL v8 例程支持的平台，55x 之后的开发板均可使用（如 58x、56x、52x）。

## 配置与初始化

手势模块由 GUI 框架统一初始化，应用通常不需要手动调用 `lvsf_gesture_init()`。在页面生命周期中按需使能 / 禁止即可：

```c
#include "lvsf/lvsf_gesture.h"
```

## API 说明

以下接口签名逐字取自 `middleware/lvgl/lvsf/lvsf_gesture.h`：

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `void lvsf_gesture_init(lv_obj_t *parent)` | 初始化手势模块 | `parent`：手势控件挂载的父对象 |
| `void lvsf_gesture_deinit(void)` | 反初始化手势模块 | 无返回值 |
| `void lvsf_gesture_set_image(uint32_t idx, const void *src_img)` | 设置手势指示图片 | `idx`：图片索引；`src_img`：图片源 |
| `void lvsf_gesture_disable(void)` | 禁止手势功能，状态切换在进入 idle 后执行 | 无返回值 |
| `void lvsf_gesture_enable(void)` | 使能手势功能，状态切换在进入 idle 后执行 | 无返回值 |
| `void lvsf_gesture_bars_realign(void)` | 手势条重新对齐 | 无返回值 |

```{note}
solution 文档中描述的 `lv_gesture_init()`、`gui_app_gesture_set_parem(left_area, goback_en)` 等页面级手势参数配置接口属于 GUI 框架层（非 `lvsf_gesture.h`），用于设置右滑触发区域线位置和是否直接返回。
```

## 典型用法

在带有右滑翻页手势的平铺页面中，临时禁止右滑退出手势，离开时恢复：

```c
static void on_resume(void)
{
    /* 页面内已有右滑翻页手势，禁止右滑退出 */
    lvsf_gesture_disable();
}

static void on_pause(void)
{
    /* 离开页面时恢复右滑退出手势 */
    lvsf_gesture_enable();
}
```

## 效果展示

```{image} ../../../assets/lvgl_v8/gesture_illustration.png
:alt: gesture 右滑退出区域示意
```

```{image} ../../../assets/lvgl_v8/gesture.gif
:alt: gesture 跟手动画效果
```

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- 源码路径：`middleware/lvgl/lvsf/lvsf_gesture.h`
