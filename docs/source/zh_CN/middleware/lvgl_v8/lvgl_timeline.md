# LVGL v8 动画时间线（Timeline）

`lvsf_timeline` 是 SiFli 的通用动画编排引擎。每个元素（element）把一个时间区间 `[start_time, end_time]` 映射到一个数值区间 `[start_value, end_value]`，并在该区间内把插值后的数值喂给一个 exec 回调；回调可以把这个值施加到**任意对象的任意属性**（位置、尺寸、透明度、角度、缩放、颜色……）。多个元素共用同一条时间轴，多段动画按时间被编排在一起，可同时串行与并行，用于复杂的转场/入场动画。

- 主要头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_timeline.h_
- 示例工程：_example/multimedia/lvgl/lvgl_v8_timeline_

## 功能简介

- 通用动画编排：每个元素把时间区间映射到数值区间，插值后通过回调输出。
- exec 回调把插值数值施加到任意对象的任意属性，不局限于位置平移。
- 多个元素共用同一条时间轴，支持串行与并行编排。
- 支持就绪回调（`ready_cb`）和用户数据。
- 时间线对象本身不可见（尺寸设 0），运行到总时长后结束并自删。

## 使用场景

- 复杂的入场/转场动画编排（多属性同时变化）。
- 界面元素的平移、缩放、透明度渐变组合动画。
- 多段动画按时间轴串行/并行播放的动画序列。

## 支持的开发板

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## 配置与初始化

BSP 中通过 `LVSF_USING_TIMELINE` 宏控制该控件的编译。时间线对象本身不可见，创建后将尺寸设为 0：

```c
#include "lvsf_timeline.h"

lv_obj_t *tl = lv_timeline_create(lv_scr_act());
lv_obj_set_size(tl, 0, 0);
```

## API 说明

### 回调类型

| 类型定义 | 功能说明 | 参数说明 |
| --- | --- | --- |
| `typedef void (*lv_timeline_exec_xcb_t)(void *var, int32_t value, void *user_data)` | 动画执行回调 | `var`：时间线节点变量；`value`：当前插值后的数值；`user_data`：用户数据。由它把数值施加到目标对象 |
| `typedef void (*lv_timeline_ready_cb_t)(void *, void *)` | 动画就绪回调 | 动画节点完成时调用 |

### 接口函数

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_timeline_create(lv_obj_t *parent)` | 创建时间线对象 | `parent`：父对象（为 `NULL` 则创建到屏幕）；返回新对象指针 |
| `void lv_timeline_set_time(lv_obj_t *timeline, int32_t time)` | 设置总运行时长 | `timeline`：时间线对象；`time`：总时长（ms） |
| `void lv_timeline_set_ready_cb(lv_obj_t *timeline, lv_timeline_ready_cb_t ready_cb, void *user_data)` | 设置就绪回调 | `ready_cb`：就绪回调；`user_data`：用户数据 |
| `void lv_timeline_add_element(lv_obj_t *timeline, int32_t start_value, int32_t end_value, int32_t start_time, int32_t end_time, lv_timeline_exec_xcb_t xcb, lv_timeline_ready_cb_t ready_cb, void *user_data)` | 添加一个动画元素 | `start_value`/`end_value`：起止值；`start_time`/`end_time`：起止时间（ms）；`xcb`：执行回调；`ready_cb`：就绪回调；`user_data`：用户数据 |
| `void lv_timeline_start(lv_obj_t *timeline)` | 启动时间线任务 | 开始播放 |
| `void lv_timeline_pause(lv_obj_t *timeline)` | 暂停时间线任务 | 暂停播放 |
| `lv_obj_t *lv_timeline_get_var(lv_obj_t *timeline)` | 获取时间线变量 | 返回变量指针 |
| `void lv_timeline_set_var(lv_obj_t *timeline, void *var)` | 设置时间线变量 | `var`：要动画的变量 |

## 典型用法

下面示例让一个方块：先右移（0~600ms），再放大（600~1200ms），最后同时缩小并移回原位（1200~1900ms，两段并行）：

```c
#include "lvsf_timeline.h"

/* 两个 exec 回调，各把数值施加到同一对象的不同属性（位置、尺寸） */
static void move_x(void *var, int32_t v, void *ud)
{
    (void)var;
    lv_obj_set_x((lv_obj_t *)ud, (lv_coord_t)v);
}
static void set_sz(void *var, int32_t v, void *ud)
{
    (void)var;
    lv_obj_set_size((lv_obj_t *)ud, (lv_coord_t)v, (lv_coord_t)v);
}

void play_animation(lv_obj_t *box)
{
    lv_obj_t *tl = lv_timeline_create(lv_scr_act());
    lv_obj_set_size(tl, 0, 0);
    /* (起值, 止值, 起时间, 止时间, exec 回调, ready 回调, user_data) */
    lv_timeline_add_element(tl, 40, 200, 0, 600, move_x, NULL, box);     /* 0..600    右移 */
    lv_timeline_add_element(tl, 40, 70, 600, 1200, set_sz, NULL, box);  /* 600..1200 放大 */
    lv_timeline_add_element(tl, 200, 40, 1200, 1900, move_x, NULL, box);/* 1200..1900 移回 ┐ 并行 */
    lv_timeline_add_element(tl, 70, 40, 1200, 1900, set_sz, NULL, box); /* 1200..1900 缩回 ┘ */
    lv_timeline_set_time(tl, 1900);   /* 总时长 */
    lv_timeline_start(tl);            /* 开始播放 */
}
```

```{note}
时间线对象运行到 `lv_timeline_set_time()` 设定的总时长后会结束并自删，因此每次播放都应新建一个时间线对象，而不是复用旧对象。
```

## 效果展示

运行 `lvgl_v8_timeline` example 可查看实际效果：屏幕上有一个红色方块，下方是 `Run` 按钮。点击 `Run`，时间线播放约 1.9 秒——方块先平移（0~0.6 秒），再放大（0.6~1.2 秒），最后同时缩小并移回原位（1.2~1.9 秒，两段动画并行）。再次点击会先复位再重新播放。

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [LVGL v8 Animation 文档](https://docs.lvgl.io/8.3/overview/animation.html)
- 源码路径：`middleware/lvgl/lvsf/gui_widgets/lvsf_timeline.h`
- 示例路径：`example/multimedia/lvgl/lvgl_v8_timeline`
