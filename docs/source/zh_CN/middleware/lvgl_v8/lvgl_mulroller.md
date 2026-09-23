# LVGL v8 mulroller 滚轮 / 转盘选择器

`lvsf_mulroller` 是 SiFli 提供的底层、回调驱动、可深度定制的滚轮（roller）/ 转盘选择控件。它把一条元素带通过拖动滚动，松手后自动吸附（对齐）到居中的那一项作为当前选择，居中项被强调、两侧渐弱，呈现经典滚轮效果。

与声明式的 `lvsf_multroller` 相比，mulroller 不直接接收选项字符串，而是通过回调按需为每个槽位填充内容：当某个槽位滚动到新的数据索引时，`appear_cb` 被调用并填入内容；滚轮停稳时 `middle_cb` 上报当前居中索引。因此只需少量真实元素即可覆盖很大的取值范围，并且可以精细控制布局、缩放、颜色、透明度、循环等。

- 主要头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_mulroller.h_
- 依赖：`lvsf_baselabel`、`lvsf_baseimg`（需在 lv_conf 中使能）
- 示例工程：_example/multimedia/lvgl/lvgl_v8_mulroller_

## 功能简介

- 横 / 竖两个方向可拖动滚轮，松手后吸附到居中项。
- 元素内容由 `appear_cb` 回调按需提供，少量元素覆盖大范围取值。
- 支持标签、图片、图片数组、自定义模块四种元素类型。
- 可分别配置居中项与两侧项的缩放、颜色、透明度，形成大小 / 颜色 / 渐变强调。
- 支持有界循环（到边界停住）与无限循环（自动绕回）。
- 支持编码器（滚轮）输入、自定义左右 / 上下边缘对象、自定义字体。

## 使用场景

- 时间选择器（时 / 分 / 秒多列滚轮组合）。
- 日期选择（年 / 月 / 日）、星期选择等需要大范围取值的滚动选择。
- 图片滚轮、数字转盘、需要自定义居中强调样式的选择界面。

## 支持的开发板

参考例程 `example/multimedia/lvgl/lvgl_v8_mulroller`（星期 + 时分时间选择器）在以下开发板验证：

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

也支持 PC 模拟器（`scons --board=pc_hcpu`）。

## 配置与初始化

在 menuconfig 中启用对应组件（默认已开启）：

```none
CONFIG_LVSF_USE_MULROLLER=y
```

调用流程：创建对象 → 设置元素类型 → 创建元素 → 注册内容 / 选择回调 → 设置方向 / 对齐 / 布局 / 循环 → 设置缩放 / 颜色 / 透明度范围 → 最后调用 `lv_mulroller_validate()` 让配置生效。

```c
#include "lvsf_mulroller.h"

/* 滚动时为每个槽位填内容，idx 即该槽位的数据值 */
static bool num_appear_cb(lv_obj_t *child, int16_t idx)
{
    lv_label_set_text_fmt(child, "%02d", idx);
    return true;
}

/* 停稳时回调，idx 即当前居中（选中）值 */
static bool hh_middle_cb(lv_obj_t *child, int16_t idx)
{
    /* 记下选中的小时并刷新读数 */
    return true;
}

lv_obj_t *r = lv_mulroller_create(parent);
lv_obj_set_size(r, 78, 156);                       /* 窗口小于元素总高才可以滚动 */
lv_mulroller_set_obj_type(r, MULROLLER_TYPE_LABEL);
lv_mulroller_create_element(r, 5, 78, 52);         /* 5 个元素，可见 3 个 */

lv_mulroller_set_appear_cb(r, num_appear_cb);
lv_mulroller_set_middle_cb(r, hh_middle_cb);

lv_mulroller_set_dir(r, MULROLLER_DIR_VER);
lv_mulroller_set_align(r, MULROLLER_ALIGN_CENTER);
lv_mulroller_set_layout_mode(r, MULROLLER_LAYOUT_MID);
lv_mulroller_set_circle_mode(r, MULROLLER_CIRCLE_NORMAL);
lv_mulroller_set_circle_range(r, 0, 23);            /* 取值范围 00..23 */

lv_mulroller_set_zoom_range(r, 36, 24);             /* LABEL: 值=字号，中大两侧小 */
lv_mulroller_set_color_mode(r, MULROLLER_COLOR_POS);
lv_mulroller_set_color_range(r, 0xFF0000, 0x9E9E9E);/* 中间红 -> 两侧灰 */
lv_mulroller_set_opa_mode(r, MULROLLER_OPA_MID);
lv_mulroller_set_opa_range(r, 255, 130);            /* 中间不透明 -> 两侧渐淡 */

lv_mulroller_validate(r);                          /* 让以上设置生效 */
```

```{warning}
- 能否滚动取决于几何关系：窗口尺寸要小于所有元素的总尺寸（竖向比高、横向比宽），因此要创建比可见数量更多的元素。
- 无限循环模式（`MULROLLER_CIRCLE_INFINITE`）不能与 `MULROLLER_LAYOUT_OVERLAP` 同时使用，且元素数量不少于 2。
- `lv_mulroller_set_opa_range()` 仅在 `MULROLLER_CIRCLE_NORMAL` 下有效，无限模式会忽略它。
- 所有设置完成后必须调用 `lv_mulroller_validate()` 才会生效。
```

## API 说明

以下函数签名均逐字来自 _lvsf_mulroller.h_。

### 创建与元素

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_mulroller_create(lv_obj_t *parent)` | 创建 mulroller 滚轮对象 | `parent`：父对象；返回新建对象指针 |
| `void lv_mulroller_create_element(lv_obj_t *mulroller, uint8_t ele_num, lv_coord_t w, lv_coord_t h)` | 创建滚轮元素 | `ele_num`：元素数量；`w` / `h`：单个元素宽高；元素类型由 `set_obj_type` 决定 |
| `void lv_mulroller_set_element(lv_obj_t *mulroller, lv_obj_t *ele, uint8_t ele_idx, uint8_t data_idx)` | 设置自定义元素 | 仅在 `MULROLLER_TYPE_MODULE` 时使用；`ele_idx` 元素槽位，`data_idx` 对应数据索引 |
| `void lv_mulroller_bind_attr(lv_obj_t *mulroller, const lv_mulroller_attr_t *attr)` | 绑定属性表 | `attr`：指向 `lv_mulroller_attr_t` 属性表 |
| `void lv_mulroller_validate(lv_obj_t *mulroller)` | 校验并使配置生效 | 所有设置完成后必须调用 |
| `void lv_mulroller_trans_refresh(lv_obj_t *mulroller)` | 刷新传输状态 | 内部刷新用 |
| `lv_obj_t *lv_mulroller_get_bg_obj(lv_obj_t *mulroller, uint8_t idx)` | 按索引获取元素背景对象 | 返回元素背景对象指针 |
| `void lv_mulroller_align_all_element(lv_obj_t *mulroller, lv_obj_t *bg_obj)` | 对齐所有元素 | `bg_obj`：背景对象 |

### 方向、类型与布局

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `void lv_mulroller_set_dir(lv_obj_t *mulroller, lv_mulroller_dir_t dir)` | 设置拖动方向 | `dir`：`MULROLLER_DIR_HOR` / `MULROLLER_DIR_VER` |
| `void lv_mulroller_set_obj_type(lv_obj_t *mulroller, lv_mulroller_obj_type_t obj_type)` | 设置元素对象类型 | `obj_type`：`MULROLLER_TYPE_LABEL/IMG/IMGARRAY/MODULE` |
| `void lv_mulroller_set_layout_mode(lv_obj_t *mulroller, lv_mulroller_layout_mode_t layout_mode)` | 设置布局模式 | 支持 `MULROLLER_LAYOUT_OVERLAP/MID/RHOMB` |
| `void lv_mulroller_set_align(lv_obj_t *mulroller, lv_mulroller_align_t align)` | 设置元素对齐方式 | 见 `lv_mulroller_align_t`（LEFT/RIGHT/TOP/BOTTOM/CENTER） |
| `void lv_mulroller_set_interval(lv_obj_t *mulroller, lv_coord_t interval)` | 设置元素间间隔 | `interval`：元素间隔（像素） |
| `void lv_mulroller_set_offset(lv_obj_t *mulroller, lv_coord_t offset_lt, lv_coord_t offset_rb)` | 设置左上角 / 右下角偏移 | — |
| `void lv_mulroller_set_custom_obj(lv_obj_t *mulroller, lv_obj_t *obj_lt, lv_obj_t *obj_rb)` | 设置左 / 右（或上 / 下）自定义边缘对象 | 滚轮移到对应边缘时显示 |

### 视觉强调（缩放 / 颜色 / 透明度）

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `void lv_mulroller_set_zoom_range(lv_obj_t *mulroller, uint16_t middle_zoom, uint16_t lateral_zoom)` | 设置居中 / 两侧元素缩放范围 | LABEL 类型时值表示字号；中间大、两侧小 |
| `void lv_mulroller_set_color_mode(lv_obj_t *mulroller, lv_mulroller_color_mode_t color_mode)` | 设置颜色变化模式 | 目前主要用 `MULROLLER_COLOR_POS`（随位置变色） |
| `void lv_mulroller_set_color_range(lv_obj_t *mulroller, uint32_t middle_color, uint32_t lateral_color)` | 设置居中 / 两侧颜色范围 | 仅 LABEL 类型 |
| `void lv_mulroller_set_opa_mode(lv_obj_t *mulroller, lv_mulroller_opa_mode_t opa_mode)` | 设置透明度变化模式 | `MULROLLER_OPA_NULL/MID/GRAD` |
| `void lv_mulroller_set_opa_range(lv_obj_t *mulroller, uint8_t middle_opa, uint8_t lateral_opa)` | 设置居中 / 两侧透明度范围 | 仅 NORMAL 循环模式有效 |
| `void lv_mulroller_set_high_light(lv_obj_t *mulroller, bool high_light)` | 是否在中间元素显示高亮轮廓 | — |
| `void lv_mulroller_set_custom_font(lv_obj_t *mulroller, bool en)` | 使用自定义字体 | `en=true` 时 mulroller 不为标签设置字体和字号，仅 LABEL 类型 |

### 循环与范围

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `void lv_mulroller_set_circle_mode(lv_obj_t *mulroller, lv_mulroller_circle_mode_t circle_mode)` | 设置循环模式 | `MULROLLER_CIRCLE_NORMAL` 有界 / `MULROLLER_CIRCLE_INFINITE` 无限 |
| `void lv_mulroller_set_circle_range(lv_obj_t *mulroller, int16_t min, int16_t max)` | 设置有界循环的取值范围 | 仅 `MULROLLER_CIRCLE_NORMAL` 使用 |
| `void lv_mulroller_set_ori_mid_idx(lv_obj_t *mulroller, int16_t idx)` | 设置初始化时的原始中间索引 | — |
| `int16_t lv_mulroller_get_mid_idx(lv_obj_t *mulroller)` | 获取当前中间（选中）索引 | 返回当前居中数据索引 |
| `void lv_mulroller_set_wheel_scale(lv_obj_t *mulroller, float wheel_scale)` | 设置轮子（编码器）每格换算距离的缩放比例 | — |
| `void lv_mulroller_set_throw_scale(lv_obj_t *mulroller, float throw_scale)` | 设置投掷（惯性）每像素换算距离的缩放比例 | — |

### 回调

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `void lv_mulroller_set_appear_cb(lv_obj_t *mulroller, lv_mulroller_appear_cb appear_lt_cb)` | 元素进入新数据索引时回调 | 在此填充元素显示内容 |
| `void lv_mulroller_set_middle_cb(lv_obj_t *mulroller, lv_mulroller_appear_cb middle_cb)` | 滚轮停稳时回调，上报当前中间索引 | 读取选中值的入口 |
| `void lv_mulroller_set_middle_cb2(lv_obj_t *mulroller, lv_mulroller_appear_cb middle_cb)` | 滚轮移动过程中的回调 | 移动时持续触发 |

### 动画与其它

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `bool lv_mulroller_create_anim(lv_obj_t *mulroller, lv_mulroller_anim_type type, int16_t num, uint32_t time, lv_mulroller_anim_ready_cb cb)` | 创建滚轮动画 | `type`：动画类型；`num`：动画数量；`time`：时长；`cb`：完成回调 |
| `void lv_mulroller_del_anim(lv_obj_t *mulroller)` | 删除当前动画 | — |
| `void lv_mulroller_set_moving_percent(lv_obj_t *mulroller, int16_t percent, int16_t param)` | 设置移动百分比 | — |
| `void lv_mulroller_set_snapshot(lv_obj_t *mulroller, bool en)` | 使能 / 关闭元素快照 | 仅在 `LV_OBJ_SNAPSHOT` 打开时可用 |
| `void lv_mulroller_extend_area(lv_obj_t *mulroller, lv_coord_t size)` | 扩展滚轮区域 | 当前实现预留 |
| `void lv_mulroller_create_mask(lv_obj_t *mulroller, lv_mulroller_dir_t dir, const void *img_src)` | 创建方向掩码 | 当前实现预留 |
| `void lv_mulroller_encoder_enable(lv_obj_t *mulroller, bool en)` | 使能 / 关闭编码器输入 | 仅在 `LVSF_USING_ENCODER` 打开时可用 |

## 典型用法

完整可运行例程见 `example/multimedia/lvgl/lvgl_v8_mulroller`，一屏内组合了顶部横向无限循环星期轮和中部两个竖向有界数字轮（时:分）。核心模式：

```c
/* 横向无限循环星期轮：方向改 HOR、循环改 INFINITE，appear_cb 里用 idx 查名字表并对 7 取模 */
static bool week_appear_cb(lv_obj_t *label, int16_t idx)
{
    static const char *names[7] = { "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun" };
    lv_label_set_text(label, names[idx % 7]);
    return true;
}

lv_obj_t *week = lv_mulroller_create(parent);
lv_obj_set_size(week, 240, 60);
lv_mulroller_set_obj_type(week, MULROLLER_TYPE_LABEL);
lv_mulroller_create_element(week, 5, 80, 60);
lv_mulroller_set_appear_cb(week, week_appear_cb);
lv_mulroller_set_dir(week, MULROLLER_DIR_HOR);
lv_mulroller_set_circle_mode(week, MULROLLER_CIRCLE_INFINITE);
lv_mulroller_validate(week);
```

```{note}
mulroller 与 multroller 定位互补：multroller 只需传入 `'\n'` 分隔的选项字符串即可，适合“给一组文字、选一个”的简单场景；mulroller 通过回调填充内容并暴露布局 / 缩放 / 颜色 / 循环等全部细节，适合高度定制的滚轮界面。
```

## 效果展示

滚轮拖动、松手吸附、居中项强调两侧渐弱的效果（参考 multroller 滚轮动画）：

```{image} ../../../assets/lvgl_v8/multroller.gif
:alt: mulroller 效果
:width: 400px
:align: center
```

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- 头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_mulroller.h_
- 示例工程：_example/multimedia/lvgl/lvgl_v8_mulroller_
- 相关控件：`lvsf_multroller`（声明式滚轮）
