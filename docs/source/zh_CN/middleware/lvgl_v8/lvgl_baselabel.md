# LVGL v8 基础标签（Baselabel）

`lvsf_baselabel` 是 SiFli 基于 LVGL v8 原生 `lv_label` 封装的自定义标签控件，在普通文本标签之上内建了一条**数据驱动刷新通路**：绑定数据源 id、注册数据回调并创建刷新定时器后，控件会周期性地拉取数据并更新自身文本，应用层无需自行轮询。它同时保留了 `lv_label` 的常规文本能力。

- 主要头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_baselabel.h_
- 数据刷新通路相关接口：_middleware/lvgl/lvsf/gui_widgets/lvsf_obj_ext.h_
- 依赖控件：LVGL label（`LV_USE_LABEL`）
- 示例工程：_example/multimedia/lvgl/lvgl_v8_baselabel_

## 功能简介

- 继承自 `lv_label_t`，支持 `set_text`、`set_text_fmt`、长文本模式、recolor 重着色、文本选择与编辑等全部常规标签能力。
- 内建数据刷新通路：绑定数据源 id + 注册数据回调 + 创建刷新定时器，周期性拉取数据并自动刷新文本。
- 支持自定义文本（动态分配复制）、静态文本（仅保存指针，适用于常量字符串）。
- 支持静态字符串表（`sfat_str`）：添加、清除、按索引切换显示，适合周/月等枚举字符串。
- 支持带省略号的文本截断（按显示宽度自动加 `...`）。
- 支持格式化文本设置（`lv_baselabel_set_text_fmt`）。

## 使用场景

- 实时数值显示，如运行时长、心率、电量、传感器读数等随时间变化的文本。
- 状态栏/表盘上跟随数据源自动刷新的数字。
- 需要在多个预定义字符串之间切换的标签（星期、月份、状态枚举）。
- 长文本按固定宽度截断并显示省略号的场景。

## 支持的开发板

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## 配置与初始化

`lvsf_baselabel` 依赖 LVGL label 组件。在 menuconfig 中确认已开启：

```none
CONFIG_LV_USE_LABEL=y
```

BSP 中通过 `LVSF_USE_BASELABEL` 宏控制该控件的编译。

创建对象的最小流程：

```c
#include "lvsf_baselabel.h"

lv_obj_t *label = lv_baselabel_create(lv_scr_act());
lv_baselabel_set_text(label, "hello");
```

## API 说明

### 回调类型

| 类型定义 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `typedef int32_t (*lv_baselabel_refresh_cb)(struct _lv_obj_t *obj, uint32_t *id_tab, uint8_t id_num)` | 基础标签刷新回调函数类型 | `obj`：对象指针；`id_tab`：数据源 id 数组指针；`id_num`：数据源 id 数量；返回 32 位整型数据。该回调用于从应用数据工具获取数据并更新标签显示内容 |

### 创建与刷新

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_baselabel_create(lv_obj_t *parent)` | 创建基础标签对象 | `parent`：父对象指针；成功返回标签对象指针，失败返回 `NULL` |
| `void lv_baselabel_refresh_timer(lv_timer_t *timer)` | 刷新定时器回调 | `timer`：定时器对象指针；定时器到点时调用用户注册的数据回调以更新文本；若设置了原始位置坐标，还会将标签对齐到原始位置 |

### 文本设置

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `void lv_baselabel_set_custom_text(lv_obj_t *label, const char *text)` | 设置自定义文本 | `label`：标签对象；`text`：文本内容（不能为空）；动态分配内存并复制到 `cus_text` 字段 |
| `void lv_baselabel_set_text_fmt(lv_obj_t *obj, const char *fmt, ...)` | 使用格式化字符串设置文本 | `obj`：标签对象；`fmt`：格式化字符串；`...`：可变参数。类似 printf 用法 |
| `void lv_baselabel_set_text(lv_obj_t *obj, const char *text)` | 设置显示文本 | 直接调用底层 `lv_label_set_text` |
| `void lv_baselabel_set_text_static(lv_obj_t *obj, const char *text)` | 设置静态文本（不复制内容） | 仅保存指针，适用于常量字符串；文本生命周期须长于标签 |
| `void lv_baselabel_set_long_mode(lv_obj_t *obj, lv_label_long_mode_t long_mode)` | 设置长文本显示模式 | `long_mode`：长文本模式（换行、滚动、省略等） |
| `void lv_baselabel_set_recolor(lv_obj_t *obj, bool en)` | 启用/禁用颜色重绘 | `en`：`true` 启用后可在文本中用颜色标记改变部分文字颜色 |
| `void lv_baselabel_set_ellip_txt(lv_obj_t *obj, const char *text, uint16_t display_w, bool ellip_en)` | 设置带省略号的文本 | `text`：文本；`display_w`：显示区域宽度（像素）；`ellip_en`：`true` 启用省略号，超长时末尾显示 `...` |
| `char *lv_baselabel_get_text(const lv_obj_t *obj)` | 获取当前文本 | 返回指向文本内容的指针 |

### 文本选择与字符定位

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `void lv_baselabel_set_text_sel_start(lv_obj_t *obj, uint32_t index)` | 设置文本选择起始位置 | `index`：起始字符索引 |
| `void lv_baselabel_set_text_sel_end(lv_obj_t *obj, uint32_t index)` | 设置文本选择结束位置 | `index`：结束字符索引 |
| `uint32_t lv_baselabel_get_text_selection_start(const lv_obj_t *obj)` | 获取文本选择起始位置 | 返回起始字符索引 |
| `uint32_t lv_baselabel_get_text_selection_end(const lv_obj_t *obj)` | 获取文本选择结束位置 | 返回结束字符索引 |
| `lv_label_long_mode_t lv_baselabel_get_long_mode(const lv_obj_t *obj)` | 获取长文本显示模式 | 返回当前长文本模式 |
| `bool lv_baselabel_get_recolor(const lv_obj_t *obj)` | 获取颜色重绘状态 | 返回 `true` 表示启用 |
| `void lv_baselabel_get_letter_pos(const lv_obj_t *obj, uint32_t char_id, lv_point_t *pos)` | 获取指定字符的坐标 | `char_id`：字符索引；`pos`：输出坐标 |
| `uint32_t lv_baselabel_get_letter_on(const lv_obj_t *obj, lv_point_t *pos_in)` | 获取指定坐标处的字符索引 | `pos_in`：输入坐标；返回字符索引，不在文本范围内返回 `LV_LABEL_POS_NONE` |
| `bool lv_baselabel_is_char_under_pos(const lv_obj_t *obj, lv_point_t *pos)` | 检查坐标处是否有字符 | 返回 `true` 表示有字符 |
| `void lv_baselabel_ins_text(lv_obj_t *obj, uint32_t pos, const char *txt)` | 在指定位置插入文本 | `pos`：插入位置字符索引；`txt`：插入文本 |
| `void lv_baselabel_cut_text(lv_obj_t *obj, uint32_t pos, uint32_t cnt)` | 剪切（删除）文本 | `pos`：删除起始索引；`cnt`：删除字符数量 |

### 静态字符串表（sfat_str）

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `void lv_baselabel_add_sfat_str(lv_obj_t *label, char *str, uint8_t len)` | 添加静态字符串 | `str`：字符串内容；`len`：长度；动态分配内存并复制，`sfat_str_num` 计数加一 |
| `void lv_baselabel_clear_sfat_str(lv_obj_t *label)` | 清除所有静态字符串 | 释放全部已添加字符串内存并重置计数器 |
| `void lv_baselabel_select_sfat_str(lv_obj_t *label, uint8_t index)` | 按索引选择并显示静态字符串 | `index`：0-based 索引；仅设置显示内容，不复制字符串；索引越界不报错也不显示 |
| `uint8_t lv_baselabel_get_sfat_str_num(lv_obj_t *label)` | 获取静态字符串数量 | 返回数量，0 表示尚未添加 |
| `uint8_t lv_baselabel_get_select_sfat_str_idx(lv_obj_t *label)` | 获取当前显示的静态字符串索引 | 范围 0 到 `sfat_str_num-1`；清除后重置为 0 |

## 典型用法

### 数据驱动刷新（推荐用法）

刷新定时器到点时会调用数据回调，在回调里把当前数据格式化进标签：

```c
#include "lvsf_baselabel.h"
#include "lvsf_obj_ext.h"   /* lv_obj_set_gmdata_cb / set_source_id / create_refresh_timer */

/* 数据回调：刷新定时器到点时被调用 */
static int32_t uptime_gmdata_cb(lv_obj_t *label, uint32_t *id_tab, uint8_t id_num)
{
    (void)id_tab;
    (void)id_num;
    static uint32_t secs = 0;
    secs++;
    char buf[32];
    lv_snprintf(buf, sizeof(buf), "uptime  %02u:%02u",
                (unsigned)((secs / 60) % 100), (unsigned)(secs % 60));
    lv_baselabel_set_text(label, buf);   /* 自己拼串 + set_text */
    return 0;
}

void demo_baselabel_init(void)
{
    lv_obj_t *label = lv_baselabel_create(lv_scr_act());
    lv_obj_set_style_text_font(label, &lv_font_montserrat_36, 0);
    lv_obj_set_style_text_color(label, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_baselabel_set_text(label, "uptime  00:00");   /* 首次刷新前的初始文本 */
    lv_obj_center(label);

    /* 绑定数据源 + 回调 + 1 秒刷新定时器 */
    static uint32_t source_id = 0x1105;
    lv_obj_set_source_id(label, &source_id, 1);
    lv_obj_set_gmdata_cb(label, uptime_gmdata_cb);
    lv_obj_create_refresh_timer(label, 1000, lv_baselabel_refresh_timer);
    lv_obj_refresh_start(label);
}
```

```{note}
在回调里更新文本，请用 `lv_snprintf()` 自己拼好字符串再调用 `lv_baselabel_set_text()`。`lv_baselabel_set_text_fmt()` 面向数据绑定格式串，并非普通 printf，直接传可变参数会得到错误结果。
```

### 静态字符串表切换

```c
lv_baselabel_add_sfat_str(label, "Mon", 3);
lv_baselabel_add_sfat_str(label, "Tue", 3);
lv_baselabel_add_sfat_str(label, "Wed", 3);
lv_baselabel_select_sfat_str(label, 1);   /* 显示 "Tue" */
```

## 效果展示

运行 `lvgl_v8_baselabel` example 可查看实际效果：屏幕中央显示蓝色大字 `uptime 00:00`，随后由刷新定时器每秒触发一次数据回调，标签自增为 `00:01`、`00:02`…… 文本变化完全由 baselabel 的数据刷新通路驱动。

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [LVGL v8 Label 文档](https://docs.lvgl.io/8.3/widgets/label.html)
- 源码路径：`middleware/lvgl/lvsf/gui_widgets/lvsf_baselabel.h`
- 示例路径：`example/multimedia/lvgl/lvgl_v8_baselabel`
