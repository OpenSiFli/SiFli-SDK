# LVGL v8 multswipe 滑动删除

`lvsf_multswipe` 是 SiFli 为 `lvsf_multlist` 列表项提供的滑动操作插件。它本身不是独立控件，而是挂在某个 `lv_multlist_item_t` 上，为该 item 的内容添加一层可拖拽的覆盖层，从而实现“左滑 / 上滑露出删除按钮并删除该 item”的交互。滑动超过阈值时 multlist 会发出 `LV_EVENT_SWIPE_DELETE` 事件，由应用完成数据清理与界面刷新。

它与 multlist、multedge、scrollbar 同属插件化体系：multlist 负责列表主体，multswipe 负责单个 item 的滑动删除手势。

- 主要头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_multswipe.h_
- 依赖：`lvsf_multlist.h`

## 功能简介

- 在 multlist 的 item 上启用拖拽删除，支持默认样式、带删除按钮样式、带展开图标样式三种。
- 可通过 `process_cb` 自定义滑动过程中的视觉表现（删除按钮宽度、透明度、图标缩放等）。
- 可配置滑动阈值、最大拖拽距离（回弹范围）、拖拽方向。
- 支持仅在 item 对齐（聚焦）状态下才允许滑动删除。
- 删除时由 multlist 发出 `LV_EVENT_SWIPE_DELETE` 事件，应用据此释放数据。

## 使用场景

- 聊天 / 通知 / 运动记录等列表中，左滑露出删除按钮删除某一条。
- 设置项、应用列表中滑动删除或触发自定义操作。
- 需要随滑动距离动态改变删除按钮宽度、透明度、图标的场景。

## 支持的开发板

multswipe 没有独立的 SDK example，作为 multlist 的插件使用，运行环境与 `lvgl_v8_multlist` 例程一致：

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## 配置与初始化

在 menuconfig 中启用对应组件（默认已开启）：

```none
CONFIG_LVSF_USE_MULTSWIPE=y
```

在 multlist 的 item 创建回调中，对需要支持滑动删除的 item 调用 `lv_multswipe_enable()` 即可启用。返回的覆盖层对象应作为该 item 的内容返回：

```c
#include "lvsf_multswipe.h"

static lv_obj_t *my_item_create_cb(lv_obj_t *parent, lv_multlist_item_t *item)
{
    lv_obj_t *content = lv_obj_create(parent);
    lv_obj_remove_style_all(content);
    lv_obj_set_size(content, item->org_w, item->org_h);
    /* ... 在 content 上放置图标、文本 ... */

    /* 启用默认滑动删除，返回覆盖层对象 */
    content = lv_multswipe_enable(item, content, LV_MULTSWIPE_STYLE_DEFAULT);
    return content;
}
```

应用还需监听 multlist 的 `LV_EVENT_SWIPE_DELETE` 事件，在回调中释放该 item 的数据：

```c
static void multlist_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *multlist = lv_event_get_current_target(e);

    if (LV_EVENT_SWIPE_DELETE == code)
    {
        lv_multlist_item_t *item = lv_event_get_param(e);
        if (item && item->info)
        {
            /* 释放 item->info 指向的业务数据 */
        }
        /* 必要时重新对齐到相邻 item */
        lv_multlist_focus_near(multlist, 0, false, true);
    }
}
```

```{warning}
- `lv_multswipe_set_del_btn()` 仅对 `LV_MULTSWIPE_STYLE_WITH_BTN` 样式生效；`lv_multswipe_set_src()` 仅对 `LV_MULTSWIPE_STYLE_WITH_EXPAN` 样式生效，非对应样式调用无效。
- 删除 item 时需同步释放覆盖层、删除按钮等相关对象，避免内存泄漏。
- `lv_multswipe_enbale_focus()` 开启后，只有 item 处于对齐（聚焦）状态才允许滑动删除。
```

## API 说明

以下函数签名均逐字来自 _lvsf_multswipe.h_。

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_multswipe_enable(lv_multlist_item_t *item, lv_obj_t *content, lv_multswipe_style style)` | 为 item 启用拖拽删除，添加滑动覆盖层 | `item`：multlist 列表项；`content`：要加覆盖层的内容对象；`style`：`LV_MULTSWIPE_STYLE_DEFAULT/WITH_BTN/WITH_EXPAN`；返回滑动覆盖层对象 |
| `void lv_multswipe_set_process_cb(lv_multlist_item_t *item, process_cb process)` | 设置拖拽过程回调，自定义滑动中的视觉表现 | `process`：回调原型 `void (*)(lv_multlist_item_t *item, lv_obj_t *content, lv_obj_t *del_btn, int32_t proc)` |
| `void lv_multswipe_set_del_btn(lv_multlist_item_t *item, lv_obj_t *del_btn)` | 设置点击删除按钮（仅 `WITH_BTN` 样式） | `del_btn`：点击触发删除的按钮对象 |
| `void lv_multswipe_set_src(lv_multlist_item_t *item, const void *del_src)` | 设置删除图标资源（仅 `WITH_EXPAN` 样式） | `del_src`：图片资源指针 |
| `void lv_multswipe_enbale_focus(lv_multlist_item_t *item, uint8_t en)` | 仅在 item 对齐（聚焦）时才允许滑动删除 | `en`：1 启用 / 0 关闭 |
| `void lv_multswipe_set_thres(lv_multlist_item_t *item, uint16_t thres_val)` | 设置滑动删除阈值 | 拖拽超过该距离即触发删除 |
| `void lv_multswipe_set_springback(lv_multlist_item_t *item, uint16_t springback)` | 设置 item 可拖拽的最大距离（回弹范围） | `springback`：最大拖拽距离 |
| `void lv_multswipe_set_dir(lv_multlist_item_t *item, uint8_t dir)` | 设置拖拽方向 | `dir`：1 正方向 / 0 负方向 |

## 典型用法

### 默认左滑 / 上滑删除

在 item 创建回调中调用 `lv_multswipe_enable()` 并使用默认样式即可：

```c
if (need_swipe_delete)
    item_cont = lv_multswipe_enable(item, item_cont, LV_MULTSWIPE_STYLE_DEFAULT);
```

### 带删除按钮样式

通过 `process_cb` 在首次拖拽时创建删除按钮，并随滑动距离更新其宽度、位置与透明度：

```c
static void sport_item_del_proc(lv_multlist_item_t *item, lv_obj_t *content,
                                lv_obj_t *del_btn, int32_t offset)
{
    if (NULL == del_btn)
    {
        /* 首次拖拽时创建删除按钮并设置阈值 / 回弹 / 按钮 */
        del_btn = lv_obj_create(item->element);
        /* ... 配置 del_btn 外观 ... */
        lv_multswipe_set_thres(item, SPORT_DEL_BTN_W + 20);
        lv_multswipe_set_springback(item, SPORT_DEL_BTN_W + 10);
        lv_multswipe_set_del_btn(item, del_btn);
    }
    /* 随后按 offset 更新 del_btn 的宽度、位置、透明度 */
}
```

```c
/* 在 item 创建回调中注册 process_cb 并启用 WITH_BTN 样式 */
lv_multswipe_set_process_cb(item, sport_item_del_proc);
item_btn = lv_multswipe_enable(item, item_btn, LV_MULTSWIPE_STYLE_WITH_BTN);
```

## 效果展示

默认滑动删除：

```{image} ../../../assets/lvgl_v8/multswipe0.gif
:alt: multswipe 默认滑动删除
:width: 400px
:align: center
```

带删除按钮样式（随滑动露出删除按钮）：

```{image} ../../../assets/lvgl_v8/multswipe1.gif
:alt: multswipe 带删除按钮
:width: 400px
:align: center
```

消息列表删除样式：

```{image} ../../../assets/lvgl_v8/multswipe2.gif
:alt: multswipe 消息删除
:width: 400px
:align: center
```

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- 头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_multswipe.h_
- 相关控件：`lvsf_multlist`（列表容器）、`lvsf_multedge`（边缘交互）、`lvsf_scrollbar`（滚动条）
