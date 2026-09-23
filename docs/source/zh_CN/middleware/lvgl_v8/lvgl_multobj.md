# LVGL v8 multobj 多对象缩放容器

`lvsf_multobj` 是 SiFli 为 LVGL v8 提供的“多对象缩放容器”。它本身是一个 `lv_obj_t` 派生对象，可以在其内部放置普通控件（图片、文本、矩形等），并在运行时按一个缩放因子 `zoom` 同时调整所有子对象的大小和位置，从而在**不开启拍照（snapshot）**的情况下实现 item 的整体缩放变形。

它主要配合 `lvsf_multlist` 使用：在 multlist 的 item 创建回调里把返回的 element 做成 multobj 实例，multlist 滑动时调用其缩放接口即可让 item 随偏移连续缩放，无需为每个 item 生成离屏快照。

- 主要头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_multobj.h_
- 实现目录：_middleware/lvgl/lvsf/gui_widgets/_

## 功能简介

- 把一组子对象（图片、文本、矩形等）纳入统一层级，按单一 `zoom` 因子整体缩放并重新定位。
- 支持单独配置内部 label（文本）的对齐方式，以及 label 位置自定义回调。
- 可对内部文本控件开启拍照（snapshot），让文本在缩放时也能平滑缩放，而不是保持原大小。
- 配合 multlist 贝塞尔缩放时，避免为每个 item 分配离屏快照带来的内存开销。

## 使用场景

- multlist 列表项需要随滑动距离缩放，但不希望为每个 item 分配快照内存。
- 主菜单 / 图标列表中，居中 item 放大、两侧 item 缩小的视觉效果。
- 独立使用时，跟随手指拖动距离对一整组控件做捏合式缩放。

## 支持的开发板

multobj 没有独立的 SDK example，通常作为 multlist 列表项的内部容器使用，运行环境与 `lvgl_v8_multlist` 例程一致：

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

工程可通过 `scons --board=<board>` 适配 SF32LB52x / SF32LB56x / SF32LB58x 等系列板型。

## 配置与初始化

在 menuconfig 中启用对应组件（默认已开启）：

```none
CONFIG_LVSF_USE_MULTOBJ=y
```

创建后，先把它当作普通 `lv_obj_t` 使用，再在其内部创建子对象；子对象布局完成后，multobj 会记录各子对象的层级与位置，之后即可调用 `lv_multobj_set_zoom()` 缩放。

```c
#include "lvsf_multobj.h"

lv_obj_t *box = lv_multobj_create(parent);
lv_obj_remove_style_all(box);
lv_obj_set_size(box, item->org_w, item->org_h);
lv_obj_add_flag(box, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_EVENT_BUBBLE);
lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

/* 在 box 内放置图片、文本等子对象 */
lv_obj_t *img = lv_img_create(box);
lv_img_set_src(img, my_icon);
lv_obj_align(img, LV_ALIGN_LEFT_MID, 30, 0);

lv_obj_t *label = lv_label_create(box);
lv_label_set_text(label, "Item title");
lv_obj_align(label, LV_ALIGN_LEFT_MID, 150, 0);

/* 子对象布局完成后，记录层级并设置 label 对齐 */
lv_multobj_set_label_align(box, LABEL_ALIGN_CENTER, LABEL_ALIGN_CENTER);
```

```{warning}
由于 LVGL 对象位置不支持亚像素，缩放过程中通过调整大小和位置实现，会有轻微抖动。multobj 内部内容在 redraw 之后如果还要继续缩放，必须重新调用 `lv_multobj_reset_hier()` 重建层级记录。
```

## API 说明

以下函数签名均逐字来自 _lvsf_multobj.h_。

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_multobj_create(lv_obj_t *parent)` | 创建 multobj 多对象缩放容器 | `parent`：父对象；返回新建对象指针 |
| `void lv_multobj_set_zoom(lv_obj_t *multobj, float zoom)` | 按记录的层级对内部所有子对象整体缩放 | `zoom`：缩放因子，`[0,1)` 缩小，`1` 不变，`(1,x)` 放大 |
| `void lv_multobj_reset_hier(lv_obj_t *multobj)` | 重置子对象的层级结构与位置记录 | 内部内容 redraw 后需要再次缩放时调用 |
| `void lv_multobj_set_snapshot(lv_obj_t *multobj, bool is_enable)` | 开启 / 关闭内部文本控件的拍照缩放 | `is_enable`：true 开启；未开启时文本缩放过程保持原大小 |
| `void lv_multobj_set_label_pos_cb(lv_obj_t *multobj, lv_multobj_label_pos_cb callback)` | 注册 label 位置自定义回调 | 回调原型：`void (*)(lv_obj_t *label, float zoom, lv_coord_t x, lv_coord_t y)` |
| `void lv_multobj_set_label_align(lv_obj_t *multobj, lv_label_align_type hor_align, lv_label_align_type ver_align)` | 设置内部 label 默认对齐方式 | `hor_align`：`LABEL_ALIGN_LEFT/CENTER/RIGHT`；`ver_align`：`LABEL_ALIGN_UP/CENTER/DOWN` |

## 典型用法

### 独立使用：跟随手指缩放

```c
static void app_event_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_current_target(e);
    lv_event_code_t code = lv_event_get_code(e);
    static lv_point_t point_pre = {0, 0};

    if (code == LV_EVENT_PRESSED)
    {
        lv_indev_t *indev = lv_indev_get_act();
        lv_indev_get_point(indev, &point_pre);
        lv_multobj_set_zoom(obj, 1);
        lv_obj_center(obj);
    }
    else if (code == LV_EVENT_PRESSING)
    {
        lv_point_t cur_point = {0, 0};
        lv_indev_t *indev = lv_indev_get_act();
        lv_indev_get_point(indev, &cur_point);
        float vect = LV_ABS(cur_point.y - point_pre.y);
        float zoom = 1 - vect / LV_VER_RES_MAX;
        lv_multobj_set_zoom(obj, zoom);
        lv_obj_center(obj);
    }
}
```

### 配合 multlist 使用

在 multlist 的 item 创建回调里返回一个 multobj 实例，multlist 内部在滑动时自动调用其缩放接口实现变形：

```c
static lv_obj_t *my_item_create_cb(lv_obj_t *parent, lv_multlist_item_t *item)
{
    lv_obj_t *item_btn = lv_multobj_create(parent);
    lv_obj_remove_style_all(item_btn);
    lv_obj_set_size(item_btn, item->org_w, item->org_h);
    lv_obj_add_flag(item_btn, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_clear_flag(item_btn, LV_OBJ_FLAG_SCROLLABLE);

    /* 在 item_btn 内放置图标、标题等子对象 */
    lv_obj_t *icon = lv_img_create(item_btn);
    lv_img_set_src(icon, my_icon);
    lv_obj_align(icon, LV_ALIGN_LEFT_MID, 30, 0);

    lv_obj_t *title = lv_label_create(item_btn);
    lv_label_set_text(title, "App");
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 150, 0);

    return item_btn;
}
```

```{note}
相比直接给 multlist item 做离屏拍照缩放，使用 multobj 缩放不占用额外快照内存，但滑动过程中会有轻微抖动，对平滑度要求极高的场景建议改用 multlist 的 snapshot 机制。
```

## 效果展示

独立使用时跟随手指拖动整体缩放：

```{image} ../../../assets/lvgl_v8/multobj_zoom.gif
:alt: multobj 独立缩放
:width: 400px
:align: center
```

配合 multlist 作为列表项容器的缩放效果：

```{image} ../../../assets/lvgl_v8/multobj.gif
:alt: multobj 配合 multlist
:width: 400px
:align: center
```

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- 头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_multobj.h_
- 相关控件：`lvsf_multlist`（列表容器）
