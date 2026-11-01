# LVGL v8 选择列表（Select）

`lvsf_select` 是 SiFli 的自定义选择列表控件，每一行在选中时显示"选中"图标、否则显示"未选中"图标。单选模式下同时只有一行被选中（radio 风格），多选模式下可同时选中多行（checkbox 风格）。点击某一行即把选中态移到该行，应用再读回用户的选择。

- 主要头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_select.h_
- 依赖控件：LVGL button / label / img（`LV_USE_BTN`、`LV_USE_LABEL`、`LV_USE_IMG`）
- 示例工程：_example/multimedia/lvgl/lvgl_v8_select_

## 功能简介

- 支持单选（`LV_SELECT_TYPE_SINGLE`）和多选（`LV_SELECT_TYPE_MULTI`）两种模式。
- 每行元素独立维护状态：未选中、选中、禁用。
- 可配置选中/未选中图标源，元素数量与元素尺寸。
- 行点击会冒泡到 select，内置点击回调会移动单选选中态。
- 支持通过接口读回当前选中索引或逐行查询选中状态。

## 使用场景

- 设置菜单中的单选/多选列表（如选项 A/B/C）。
- 单选框、复选框组。
- 需要自定义选中/未选中图标的条目列表。

## 支持的开发板

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## 配置与初始化

`lvsf_select` 依赖 LVGL button、label、img 组件。在 menuconfig 中确认已开启：

```none
CONFIG_LV_USE_BTN=y
CONFIG_LV_USE_LABEL=y
CONFIG_LV_USE_IMG=y
```

BSP 中通过 `LVSF_USE_SELECT` 宏控制该控件的编译。

创建对象的最小流程：

```c
#include "lvsf_select.h"

lv_obj_t *sel = lv_select_create(lv_scr_act());
```

## API 说明

### 枚举与常量

| 类型 | 取值 | 说明 |
| --- | --- | --- |
| `lv_select_type_t` | `LV_SELECT_TYPE_SINGLE` | 单选模式 |
| | `LV_SELECT_TYPE_MULTI` | 多选模式 |
| `lv_select_state_t` | `LV_SELECT_STATE_UNCHECK` | 未选中状态 |
| | `LV_SELECT_STATE_CHECK` | 选中状态 |
| | `LV_SELECT_STATE_DISABLE` | 禁用状态 |
| 宏 | `LV_SELECT_ELE_INTERVAL` | 元素间距（4） |

### 接口函数

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_select_create(lv_obj_t *par)` | 创建选择控件 | `par`：父对象；返回选择控件对象指针 |
| `void lv_select_set_type(lv_obj_t *select, lv_select_type_t type)` | 设置选择类型 | `type`：单选/多选 |
| `void lv_select_set_ele_num(lv_obj_t *select, uint16_t num)` | 设置元素数量 | `num`：行数 |
| `void lv_select_set_ele_size(lv_obj_t *select, lv_coord_t w, lv_coord_t h)` | 设置元素大小 | `w`/`h`：元素宽度/高度，据此排布各行 |
| `void lv_select_set_ele_state(lv_obj_t *select, uint16_t idx, lv_select_state_t state)` | 设置元素状态 | `idx`：元素索引；`state`：元素状态 |
| `void lv_select_set_check_src(lv_obj_t *select, const void *src)` | 设置选中图标源 | `src`：选中图标源 |
| `void lv_select_set_uncheck_src(lv_obj_t *select, const void *src)` | 设置未选中图标源 | `src`：未选中图标源 |
| `uint16_t lv_select_get_select_idx(lv_obj_t *select)` | 获取当前选中索引 | 单选模式下返回当前选中行 |
| `lv_obj_t *lv_select_get_ele(lv_obj_t *select, uint16_t idx)` | 获取指定索引的元素对象 | `idx`：元素索引；返回元素对象指针 |
| `uint16_t lv_select_get_ele_idx(lv_obj_t *select, lv_obj_t *ele)` | 获取元素的索引 | `ele`：元素对象指针；返回元素索引 |
| `uint16_t lv_select_get_ele_num(lv_obj_t *select)` | 获取元素数量 | 返回元素数量 |
| `lv_select_state_t lv_select_get_ele_state(lv_obj_t *select, uint16_t idx)` | 获取指定元素状态 | `idx`：元素索引；返回元素状态 |

## 典型用法

```c
#include "lvsf_select.h"

lv_obj_t *sel = lv_select_create(parent);
lv_obj_add_flag(sel, LV_OBJ_FLAG_CLICKABLE);        /* 构造会清掉，点击需要它 */
lv_select_set_type(sel, LV_SELECT_TYPE_SINGLE);     /* 单选 */
lv_select_set_ele_num(sel, 3);                      /* 行数 */
lv_select_set_ele_size(sel, 220, 40);               /* 每行尺寸 */
lv_select_set_check_src(sel, &check_img_dsc);       /* 选中图标 */
lv_select_set_uncheck_src(sel, &uncheck_img_dsc);    /* 未选中图标 */
lv_select_set_ele_state(sel, 0, LV_SELECT_STATE_CHECK);  /* 第 0 行初始选中 */

/* 读回用户选择（行点击会冒泡到 select）： */
uint16_t idx = lv_select_get_select_idx(sel);       /* 单选：当前选中行 */
/* 多选时逐行查：lv_select_get_ele_state(sel, i) == LV_SELECT_STATE_CHECK */
```

在 select 上挂 `LV_EVENT_SHORT_CLICKED` 回调即可读回用户最新选择：

```c
static void sel_changed_cb(lv_event_t *e)
{
    lv_obj_t *sel = (lv_obj_t *)lv_event_get_user_data(e);
    uint16_t idx = lv_select_get_select_idx(sel);
    /* 根据 idx 更新 UI */
}

lv_obj_add_event_cb(sel, sel_changed_cb, LV_EVENT_SHORT_CLICKED, sel);
```

```{warning}
`lv_select_create()` 构造时会清掉自身的 `LV_OBJ_FLAG_CLICKABLE`，需重新 `lv_obj_add_flag(sel, LV_OBJ_FLAG_CLICKABLE)`，点击才能下达到各行。
```

## 效果展示

运行 `lvgl_v8_select` example 可查看实际效果：屏幕中央有 3 行条目（Option A/B/C），每行右侧一个小图标，第一行绿色（选中），另外两行深色（未选中）；下方一行 `Selected: Option A` 显示当前选择。点击其它行，绿色"选中"图标移到被点行，`Selected:` 文字实时更新。

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- 源码路径：`middleware/lvgl/lvsf/gui_widgets/lvsf_select.h`
- 示例路径：`example/multimedia/lvgl/lvgl_v8_select`
