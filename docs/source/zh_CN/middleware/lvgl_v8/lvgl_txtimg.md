# LVGL v8 图文混排文本（txtimg）

`txtimg` 是 SiFli SDK 在 LVGL v8 上提供的图文混排文本控件。它把文本按行渲染并缓存为图像（A8 位图），再以图像方式定位、缩放和播放动画，适合文本量大、需要缩短渲染时间或需要对整行文本做缩放/横向滚动动画的场景。

与普通 `lv_label` 每次重绘都重新排版不同，txtimg 把一行文本光栅化后缓存为位图，后续缩放、平移和动画直接操作图像，从而降低电子书、列表标题等长文本场景的渲染开销。

## 功能简介

- 文本按行添加、替换或追加，每一行文本被缓存为独立的位图描述符。
- 支持对整个文本图像设置缩放（zoom）。
- 支持横向滚动动画（`LV_TXTIMG_HOR_ANIM`）和动画缓冲区（`LV_TXTIMG_ANIM_BUF`）。
- 针对不支持位图转 A8 格式的语言（如泰文、印地语、阿拉伯语等），可通过 `lv_txtimg_snapshot_txt_line()` 把文本快照为图像后定位显示。
- 提供标志位控制动画与资源驻留行为。

## 使用场景

- 电子书等文本量较大的页面，把文本转 A8 位图以缩短渲染时间。
- 列表项标题需要横向滚动跑马灯动画。
- 不支持直接转 A8 格式的语种，需要以快照方式显示文本。
- 需要对整段文本统一缩放并刷新尺寸的场景。

## 支持的开发板

参考 `lvgl_v8_multlist` 等通用 LVGL v8 例程所支持的平台，55x 之后的开发板均可使用（如 58x、56x、52x），需在 `menuconfig` 中启用 `LVSF_USE_TXTIMG`。

## 配置与初始化

txtimg 由 `LVSF_USE_TXTIMG` 开关控制，在 `menuconfig` 的 `LittlevGL2RTT -> SiFli extend` 中启用。

```c
#include "lvsf.h"
#include "lvsf/gui_widgets/lvsf_txtimg.h"
```

标志位枚举（`lv_txtimg_flg_t`）：

| 标志 | 含义 |
| --- | --- |
| `LV_TXTIMG_ANIM_BUF` | 使用动画缓冲区 |
| `LV_TXTIMG_HOR_ANIM` | 启用横向滚动动画 |
| `LV_TXTIMG_RESIDENCY` | 资源驻留 |

## API 说明

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_txtimg_create(lv_obj_t *parent)` | 创建 txtimg 对象实例 | `parent`：父对象；返回创建的对象指针 |
| `void lv_txtimg_set_txt(lv_obj_t *txtimg, const char *text)` | 保留原有文本，追加插入最新字符串 | `text`：待插入字符串 |
| `void lv_txtimg_ins_txt(lv_obj_t *txtimg, const char *text)` | 替换原字符串，清空之前文本仅保留当前字符串 | `text`：待设置字符串 |
| `int32_t lv_txtimg_set_txt_line(lv_obj_t *txtimg, const char *text)` | 为 txtimg 添加一行文本 | `text`：行文本；返回该行文本长度 |
| `int32_t lv_txtimg_snapshot_txt_line(lv_obj_t *txtimg, const char *text)` | 对不支持位图转 A8 的语言，将文本快照为图像后定位显示 | `text`：待处理文本；返回该行文本长度 |
| `void lv_txtimg_set_zoom(lv_obj_t *txtimg, lv_coord_t zoom)` | 设置控件缩放比例 | `zoom`：缩放值 |
| `void lv_txtimg_refr_size(lv_obj_t *txtimg)` | 刷新控件尺寸，适配文本显示区域 | 在文本、缩放、标志位修改后调用 |
| `void lv_txtimg_set_flg(lv_obj_t *txtimg, uint32_t flg)` | 设置标志位（动画、驻留等） | `flg`：`LV_TXTIMG_*` 枚举的位或组合 |
| `uint32_t lv_txtimg_get_flg(lv_obj_t *txtimg)` | 获取当前标志位配置 | 返回当前标志位值 |

## 典型用法

电子书页面把文本转 A8 图片以缩短渲染时间：

```c
lv_obj_t *element = lv_txtimg_create(multlist);
lv_obj_set_size(element, w, h);
lv_txtimg_set_flg(element, LV_TXTIMG_ANIM_BUF);
lv_ext_set_local_text_font(element, font, LV_PART_MAIN | LV_STATE_DEFAULT);
lv_ext_set_local_text_color(element, color_txt, LV_PART_MAIN | LV_STATE_DEFAULT);

const char *str = (const char *)&p_reader_txt->txt_buf[info->txt_pos];
lv_txtimg_set_txt_line(element, str);
```

带缩放并需要横向滚动动画的文本（对不支持 A8 的语言使用快照接口）：

```c
const lv_font_t *font = LV_EXT_FONT_GET(FONT_BIGL);
lv_obj_t *txtimg = lv_txtimg_create(parent);
lv_obj_set_size(txtimg, LV_HOR_RES_MAX >> 1, item_h);
lv_obj_add_flag(txtimg, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_EVENT_BUBBLE);
lv_obj_clear_flag(txtimg, LV_OBJ_FLAG_SCROLLABLE);
lv_obj_align(txtimg, LV_ALIGN_RIGHT_MID, -20, 0);

lv_txtimg_set_flg(txtimg, LV_TXTIMG_HOR_ANIM | LV_TXTIMG_RESIDENCY);
lv_obj_set_style_text_font(txtimg, font, 0);
lv_obj_set_style_text_color(txtimg, LV_COLOR_WHITE, 0);

if (need_snapshot_lang)
{
    lv_txtimg_snapshot_txt_line(txtimg, txt);  /* 泰文/印地语/阿拉伯语等 */
}
else
{
    lv_txtimg_set_txt_line(txtimg, txt);
}
```

## 效果展示

```{image} ../../../assets/lvgl_v8/multlist_scroll_demo.gif
:alt: 长列表滚动渲染效果
```

```{note}
本页动图与 [scrollbar 控件](lvgl_scrollbar.md) 共用同一段录屏（取自官方 solution 文档的 UI 控件页面），展示的是长列表滚动场景，不体现 txtimg 文本转图片的内部细节。
```

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- 源码路径：`middleware/lvgl/lvsf/gui_widgets/lvsf_txtimg.h`
