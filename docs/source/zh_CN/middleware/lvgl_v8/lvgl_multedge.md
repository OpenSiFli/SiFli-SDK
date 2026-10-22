# LVGL v8 multedge 边缘交互

`lvsf_multedge` 是 SiFli 为 `lvsf_multlist` 提供的边缘交互插件。当 multlist 滑动到边界时会发出边缘拖拽事件；如果存在一个 edge 对象响应该事件并回复应答，multlist 就会把显示权交给 edge，由它显示侧边栏、下拉 / 上拉浮层等内容，从而实现类似“从屏幕边缘滑出的悬浮窗口”的效果。edge 与 multlist 之间的事件交互逻辑已在控件内部封装。

它与 multlist、multswipe、scrollbar 同属插件化体系：multlist 滑动到边缘时触发，multedge 接管显示。

- 主要头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_multedge.h_
- 依赖：`lvsf_multlist.h`

## 功能简介

- 支持左 / 右 / 上 / 下四种边缘类型，分别对应侧边栏、下拉、上拉等浮层。
- 可通过 check 回调或指定 item 索引，决定边缘交互作用于哪个列表项。
- 可配置边缘展开 / 收起的阈值距离。
- 内部通过事件与 multlist 完成“请求—应答—拖拽—结束”的交接，应用只需创建 edge 对象并配置即可。

## 使用场景

- 平铺（TLV）页面的左侧 / 右侧侧边栏。
- 顶部下拉消息框、底部上拉菜单。
- 任何“从屏幕边缘拖出一个悬浮窗口”的交互。

## 支持的开发板

multedge 没有独立的 SDK example，作为 multlist 的插件使用，运行环境与 `lvgl_v8_multlist` 例程一致：

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## 配置与初始化

在 menuconfig 中启用对应组件（默认已开启）：

```none
CONFIG_LVSF_USE_MULTEDGE=y
```

创建 edge 对象时，父对象必须是 multlist 控件本身。先布局 edge 的外观（背景、尺寸、位置），再设置边缘类型、绑定 item 的方式和阈值：

```c
#include "lvsf_multedge.h"

/* 左侧侧边栏 */
lv_obj_t *left_egd = lv_multedge_create(multlist);   /* parent 必须是 multlist */
lv_obj_remove_style_all(left_egd);
lv_obj_set_size(left_egd, SIDEBAR_W, LV_VER_RES_MAX);
lv_obj_align_to(left_egd, multlist, LV_ALIGN_OUT_LEFT_MID, 0, 0);
lv_obj_set_style_bg_color(left_egd, LV_COLOR_BLACK, LV_STATE_DEFAULT);
lv_obj_set_style_bg_opa(left_egd, LV_OPA_80, LV_STATE_DEFAULT);

lv_multedge_set_type(left_egd, LV_EDGE_LEFT);                 /* 左侧边缘 */
lv_multedge_set_item_index(left_egd, 0);                     /* 仅 item 索引为 0 时触发 */
lv_multedge_set_threshold(left_egd, -SIDEBAR_W + 5, -5);      /* 展开 / 收起阈值 */
```

也可以用 check 回调代替固定索引，在回调里根据 item 内容判断是否允许拖拽 edge：

```c
static int shortcut_check_item(lv_multlist_item_t *item)
{
    /* 返回 true 表示允许触发 edge 滑出，false 表示保持不动 */
    return my_item_is_wf(item);
}

lv_multedge_set_check_cb(top_egd, shortcut_check_item);
```

```{warning}
- `lv_multedge_create(parent)` 的 parent 必须是 multlist 控件对象，否则事件联动不会生效。
- 边缘类型必须取合法值 `LV_EDGE_LEFT/RIGHT/TOP/BOTTOM`。
- 阈值 start / end 要结合实际坐标范围设置，过小 / 过大会导致交互不灵敏或误触发。
```

## API 说明

以下函数签名均逐字来自 _lvsf_multedge.h_。

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_multedge_create(lv_obj_t *parent)` | 创建 multedge 边缘对象 | `parent`：必须为 multlist 对象；返回新建 edge 指针 |
| `void lv_multedge_set_type(lv_obj_t *edge, lv_multlist_edge_type_t type)` | 设置边缘类型 | `type`：`LV_EDGE_LEFT/RIGHT/TOP/BOTTOM` |
| `void lv_multedge_set_check_cb(lv_obj_t *edge, lv_multedge_check_item check_cb)` | 设置检查回调，判断边缘交互作用于哪个 item | `check_cb`：原型 `int (*)(lv_multlist_item_t *item)`，返回非 0 表示该 item 可触发 edge |
| `void lv_multedge_set_item_index(lv_obj_t *edge, int16_t index)` | 设置边缘交互作用的 item 索引 | 仅该索引的 item 触发 edge |
| `void lv_multedge_set_threshold(lv_obj_t *edge, lv_coord_t start, lv_coord_t end)` | 设置边缘展开 / 收起阈值 | `start`：展开起始阈值；`end`：收起结束阈值 |
| `uint16_t lv_multedge_get_state(lv_obj_t *edge)` | 获取 edge 当前状态 | 返回 `LV_MULTEDGE_HIDDEN`（隐藏）/ `LV_MULTEDGE_MOVING`（移动中）/ `LV_MULTEDGE_END`（完全拉出）/ `LV_MULTEDGE_GOBACK` |

## 典型用法

### 左侧侧边栏

```c
lv_obj_t *left_egd = lv_multedge_create(multlist);
lv_obj_remove_style_all(left_egd);
lv_obj_set_size(left_egd, SIDEBAR_W, LV_VER_RES_MAX);
lv_obj_align_to(left_egd, multlist, LV_ALIGN_OUT_LEFT_MID, 0, 0);
lv_obj_set_style_bg_color(left_egd, LV_COLOR_BLACK, LV_STATE_DEFAULT);
lv_obj_set_style_bg_opa(left_egd, LV_OPA_80, LV_STATE_DEFAULT);

lv_multedge_set_type(left_egd, LV_EDGE_LEFT);
lv_multedge_set_item_index(left_egd, 0);
lv_multedge_set_threshold(left_egd, -SIDEBAR_W + 5, -5);
```

### 顶部下拉消息框

```c
lv_obj_t *top_egd = lv_multedge_create(multlist);
lv_obj_remove_style_all(top_egd);
lv_obj_set_size(top_egd, LV_HOR_RES_MAX, LV_VER_RES_MAX);
lv_obj_align_to(top_egd, multlist, LV_ALIGN_OUT_TOP_MID, 0, 0);
lv_obj_set_style_bg_color(top_egd, lv_color_make(50, 50, 50), LV_STATE_DEFAULT);
lv_obj_set_style_bg_opa(top_egd, 128, LV_STATE_DEFAULT);

lv_multedge_set_type(top_egd, LV_EDGE_TOP);
lv_multedge_set_check_cb(top_egd, shortcut_check_item);
```

## 效果展示

从屏幕边缘拖出浮层的边缘交互效果：

```{image} ../../../assets/lvgl_v8/multedge.gif
:alt: multedge 效果
:width: 400px
:align: center
```

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- 头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_multedge.h_
- 相关控件：`lvsf_multlist`（列表容器）、`lvsf_multswipe`（滑动删除）、`lvsf_scrollbar`（滚动条）
