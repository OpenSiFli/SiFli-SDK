# LVGL v8 重力跟随菜单（Follow）

`lvsf_follow` 是 SiFli 提供的物理"重力"图标菜单控件。图标排成同心圆环，并在重力作用下移动、相互避让，常用于智能手表的应用启动器。板子上由 g-sensor 提供重力方向；PC 模拟器里控件自带交互路径，把鼠标点击位置转成重力向量。

- 主要头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_follow.h_
- 示例工程：_example/multimedia/lvgl/lvgl_v8_follow_

## 功能简介

- 图标按同心圆环分层排布，支持最多 3 层（`MMENU_ICON_LAYER_CNT`）。
- 基于重力/摩擦/碰撞的物理动画：图标在重力作用下移动、相互避让。
- 支持自定义环布局：每环半径、图标半径、起始角、间隔角均可配置。
- 支持多种状态切换：普通（normal）、排序（order）、换位（change）、编辑（edit）。
- 通过回调为每个元素创建图标对象，并在销毁时回收。
- 支持方屏/圆屏适配（`is_square`、`square_r`）。

## 使用场景

- 智能手表/手环的应用启动器（表盘桌面）。
- 重力感应驱动的可拖拽图标网格。
- 需要物理动效的圆环菜单、桌面图标排列。

## 支持的开发板

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## 配置与初始化

BSP 中通过 lvsf 配置控制该控件的编译。创建对象后，先通过 `lv_follow_get_cfg_param()` 获取配置结构体，逐项设置物理参数与环布局。

```c
#include "lvsf_follow.h"

lv_obj_t *follow = lv_follow_create(lv_scr_act());
lv_obj_set_size(follow, LV_PCT(100), 360);
```

## API 说明

### 常量定义

| 宏 | 含义 |
| --- | --- |
| `FOLLOW_TYPE_DEFAULT` | 默认碰撞类型（0） |
| `FOLLOW_TYPE_SIMILAR` | 相似碰撞类型（1） |
| `FOLLOW_TYPE_STANDARDS` | 标准碰撞类型（2） |
| `FOLLOW_STATUS_NORMAL` | 普通状态（`1U << 0`） |
| `FOLLOW_STATUS_ORDER` | 排序状态（`1U << 1`） |
| `FOLLOW_STATUS_CHANGE` | 换位状态（`1U << 2`） |
| `FOLLOW_STATUS_EDIT` | 编辑状态（`1U << 3`） |
| `MMENU_ICON_LAYER_CNT` | 图标层数（3） |

### 回调类型

| 类型定义 | 功能说明 |
| --- | --- |
| `typedef lv_obj_t *(*lv_follow_create_item_cb)(lv_obj_t *parent, uint16_t index, uint16_t type, void *user_data)` | 为每个元素创建并返回一个图标对象 |
| `typedef lv_obj_t *(*lv_follow_set_border_cb)(lv_obj_t *parent, uint16_t index, void *user_data)` | 为指定元素设置边框 |
| `typedef void(*lv_follow_delete_info_cb)(lv_follow_item_info_t *item)` | 元素信息销毁回调 |

### 配置结构体 `lv_follow_cfg_t`

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `gravity` | `float` | 重力加速度 |
| `friction` | `float` | 摩擦系数 |
| `opa_r` | `lv_coord_t` | 透明度半径 |
| `opa_min` | `lv_coord_t` | 最小透明度 |
| `v_max` | `uint16_t` | 最大速度 |
| `margin` | `uint8_t` | 边距 |
| `collision_type` | `uint8_t` | 碰撞类型（`FOLLOW_TYPE_*`） |
| `icon_r` | `int16_t` | 图标半径 |
| `hor_rate` | `float` | 水平速率 |
| `ver_rate` | `float` | 垂直速率 |
| `speed_ratio` | `float` | 速度比例 |
| `black_ratio` | `float` | 黑屏比例 |
| `square_r` | `int16_t` | 方屏圆角半径 |
| `target_r[MMENU_ICON_LAYER_CNT]` | `int16_t[]` | 每层图标半径 |
| `offset_r[MMENU_ICON_LAYER_CNT]` | `int16_t[]` | 每层图像中心极坐标半径（环半径） |
| `start_angle[MMENU_ICON_LAYER_CNT]` | `int16_t[]` | 每层图标起始角 |
| `gap_angle[MMENU_ICON_LAYER_CNT]` | `int16_t[]` | 每层图标间隔角（每环个数 = 360 / gap_angle） |
| `custom_align` | `bool` | 是否自定义对齐；`true` 时逐环布局生效 |
| `is_square` | `bool` | 是否方屏 |

### 接口函数

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_follow_create(lv_obj_t *parent)` | 创建 follow 菜单对象 | `parent`：父对象；返回菜单对象指针 |
| `void lv_follow_add_item_info(lv_obj_t *crashmenu, uint16_t type, void *user_data)` | 添加一个元素 | `crashmenu`：菜单对象；`type`：元素类型；`user_data`：用户数据 |
| `void lv_follow_set_gravity(lv_obj_t *crashmenu, float g, lv_coord_t angle)` | 设置重力方向与大小 | `g`：重力大小；`angle`：方向角 |
| `lv_follow_cfg_t *lv_follow_get_cfg_param(lv_obj_t *crashmenu)` | 获取配置结构体指针 | 返回可直接修改的配置指针 |
| `void lv_follow_disable_status(lv_obj_t *crashmenu, uint8_t status)` | 禁用指定状态 | `status`：状态位（`FOLLOW_STATUS_*`） |
| `uint8_t lv_follow_get_status(lv_obj_t *crashmenu)` | 获取当前状态 | 返回状态位 |
| `void lv_follow_set_item_cb(lv_obj_t *crashmenu, lv_follow_create_item_cb create_cb, lv_follow_delete_info_cb delete_cb)` | 设置元素创建与销毁回调 | `create_cb`：创建回调；`delete_cb`：销毁回调 |
| `void lv_follow_enter_order_status(lv_obj_t *crashmenu)` | 进入排序状态（图标排成环形队列） | — |
| `void lv_follow_enter_normal_status(lv_obj_t *crashmenu)` | 进入普通状态 | — |
| `void lv_follow_enter_change_status(lv_obj_t *crashmenu)` | 进入换位状态 | — |
| `void lv_follow_enter_edit_status(lv_obj_t *crashmenu, int8_t layer)` | 进入编辑状态 | `layer`：编辑层 |
| `void lv_follow_on_start(lv_obj_t *parent)` | 启动控件 | — |
| `void lv_follow_on_resume(lv_obj_t *parent)` | 恢复控件 | — |
| `void lv_follow_on_pause(lv_obj_t *parent)` | 暂停控件 | — |
| `void lv_follow_on_stop(lv_obj_t *parent)` | 停止控件 | — |

## 典型用法

```c
#include "lvsf_follow.h"

#define ICON_NUM 8

/* item 回调：为每个元素返回一个图标对象（这里是纯色圆，实际应用应为启动器图标） */
static lv_obj_t *follow_item_cb(lv_obj_t *parent, uint16_t index, uint16_t type, void *user_data)
{
    lv_obj_t *ic = lv_obj_create(parent);
    lv_obj_remove_style_all(ic);
    lv_obj_set_size(ic, 46, 46);
    lv_obj_set_style_radius(ic, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(ic, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_bg_opa(ic, LV_OPA_COVER, 0);
    return ic;
}

static void follow_delete_cb(lv_follow_item_info_t *item)
{
}

void demo_follow_init(void)
{
    lv_obj_t *follow = lv_follow_create(lv_scr_act());
    lv_obj_set_size(follow, LV_PCT(100), 360);

    lv_follow_cfg_t *cfg = lv_follow_get_cfg_param(follow);
    cfg->collision_type = FOLLOW_TYPE_STANDARDS;
    cfg->gravity = 0.01f;
    cfg->friction = 0.2f;
    cfg->icon_r = 23;
    cfg->v_max = 3;
    cfg->target_r[0] = 23; cfg->target_r[1] = 19; cfg->target_r[2] = 14;  /* 各环图标半径 */
    cfg->offset_r[0] = 0;  cfg->offset_r[1] = 70; cfg->offset_r[2] = 125; /* 各环半径 */
    cfg->start_angle[0] = 0; cfg->start_angle[1] = 0; cfg->start_angle[2] = 0;
    cfg->gap_angle[0] = 360; cfg->gap_angle[1] = 60; cfg->gap_angle[2] = 40; /* 360/gap=每环个数 */
    cfg->custom_align = true;   /* 必须：采用上面的逐环布局 */
    cfg->is_square = false;

    lv_follow_set_item_cb(follow, follow_item_cb, follow_delete_cb);
    for (int i = 0; i < ICON_NUM; i++)
        lv_follow_add_item_info(follow, 0, NULL);   /* 加入元素 */
    lv_follow_on_start(follow);
    lv_follow_enter_order_status(follow);            /* 排成环形队列 */
}
```

```{warning}
`cfg->custom_align` 必须设为 `true`，逐环布局（`offset_r`、`target_r`、`gap_angle`）才会生效；设为 `false` 时控件会按自身宽度自动计算图标尺寸。
```

## 效果展示

运行 `lvgl_v8_follow` example 可查看实际效果：屏幕上有 8 个彩色圆点排成重力菜单环（中心一个大的 + 一圈 6 个 + 外侧 1 个）。模拟器交互：点中心图标重新排回环形队列（聚拢）；点边缘图标朝点击方向被"重力"拉过去；长按拖动移动某个图标。板子上由 g-sensor 驱动重力。

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- 源码路径：`middleware/lvgl/lvsf/gui_widgets/lvsf_follow.h`
- 示例路径：`example/multimedia/lvgl/lvgl_v8_follow`
