# LVGL v8 scrollbar 滚动条

`lvsf_scrollbar` 是 SiFli 为 `lvsf_multlist` 配套的滚动条（进度条）插件。multlist 在滑动时会发送 `LV_EVENT_LIST_SCROLLBAR` 事件，事件中携带当前页面进度信息；只要创建一个 scrollbar 控件并挂到 multlist 上，它就会自动监听该事件并显示列表滚动进度。进度条支持圆形和长条（方形）两种视觉样式。

- 主要头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_scrollbar.h_
- 依赖：`lvsf_multlist`

## 功能简介

- 在 multlist 滑动时自动显示当前滚动位置与内容总长度的比例。
- 支持长条（方形）和圆形两种样式。
- 可配置滚动条保持显示时长与消失动画时长。
- 可手动隐藏 / 显示，或设置为常显。

## 使用场景

- 主菜单、卡片流等较长列表的滚动进度指示。
- 需要在屏幕边缘显示一个细进度条或圆形进度指示的界面。

## 支持的开发板

scrollbar 没有独立的 SDK example，作为 multlist 的插件使用，运行环境与 `lvgl_v8_multlist` 例程一致：

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## 配置与初始化

在 menuconfig 中启用对应组件（默认已开启）：

```none
CONFIG_LVSF_USE_SCROLLBAR=y
```

在 multlist 初始化完成后，以 multlist 为父对象创建 scrollbar 即可。multlist 需要先打开 `LV_MULTLIST_FLAG_SCROLLBAR` flag，滑动时才会发送进度事件：

```c
#include "lvsf_scrollbar.h"

/* 在 multlist 上使能 scrollbar 事件 */
lv_multlist_add_flag(multlist, LV_MULTLIST_FLAG_SCROLLBAR);

/* 创建长条样式滚动条，父对象为 multlist */
lv_obj_t *bar = lv_scrollbar_create(multlist, LV_SCROLLBAR_SQUARE_TYPE);

/* 可选：配置保持与消失动画时长（ms） */
lv_scrollbar_set_anim_time(multlist, 1500, 300);
```

```{note}
`lv_scrollbar_create()` 的父对象传 multlist 后，scrollbar 会自动处理 multlist 发出的 `LV_EVENT_LIST_SCROLLBAR` 事件，应用通常无需再手动调用 `lv_scrollbar_update()`。
```

## API 说明

以下函数签名均逐字来自 _lvsf_scrollbar.h_。

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_scrollbar_create(lv_obj_t *parent, uint16_t type)` | 创建滚动条对象 | `parent`：父对象（通常传 multlist）；`type`：`LV_SCROLLBAR_SQUARE_TYPE`（长条）/ `LV_SCROLLBAR_CIRCLE_TYPE`（圆形）；返回滚动条指针 |
| `void lv_scrollbar_update(lv_obj_t *scrollbar, lv_coord_t cur_pos, uint16_t ind_len, uint16_t tatol_len)` | 更新滚动条位置与显示比例 | `cur_pos`：当前滚动位置；`ind_len`：滚动指示器长度；`tatol_len`：内容总长度 |
| `void lv_scrollbar_set_anim_time(lv_obj_t *parent, uint32_t hold_time, uint32_t disappear_time)` | 设置保持显示与消失动画时长 | `hold_time`：保持显示时长（ms）；`disappear_time`：消失动画时长（ms） |
| `void lv_scrollbar_set_hidden(lv_obj_t *scrollbar, bool is_hidden)` | 手动隐藏 / 显示滚动条 | `is_hidden`：true 隐藏，false 显示 |
| `void lv_scrollbar_set_always_visible(lv_obj_t *scrollbar, bool is_always_visible)` | 设置是否常显 | `is_always_visible`：true 始终显示，false 滑动后自动隐藏 |

## 典型用法

在 multlist 初始化流程末尾创建滚动条即可，完整示例（摘自主菜单初始化）：

```c
lv_obj_t *multlist = lv_multlist_create(lv_scr_act());
/* ... 配置 multlist 大小、贝塞尔、间距、方向、item 回调、节点、对齐 ... */

lv_multlist_add_flag(multlist, LV_MULTLIST_FLAG_SCROLLBAR);
lv_scrollbar_create(multlist, LV_SCROLLBAR_SQUARE_TYPE);   /* 长条进度条 */
```

若要圆形样式，把类型换成 `LV_SCROLLBAR_CIRCLE_TYPE`：

```c
lv_scrollbar_create(multlist, LV_SCROLLBAR_CIRCLE_TYPE);
```

## 效果展示

列表滑动时进度条随滚动位置实时更新：

```{image} ../../../assets/lvgl_v8/multlist_scroll_demo.gif
:alt: scrollbar 效果
:width: 400px
:align: center
```

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- 头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_scrollbar.h_
- 相关控件：`lvsf_multlist`（列表容器）
