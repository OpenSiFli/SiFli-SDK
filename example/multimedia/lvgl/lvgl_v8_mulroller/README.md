# LVGL v8 Mulroller 示例

源码路径: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_mulroller`

## 概述

本例程演示 `lvsf_mulroller` 控件：一个滚轮/转盘选择器。把一条元素带通过拖动滚动，松手后对齐(吸附)到居中的那一项作为当前选择，居中项被强调、两侧渐弱，呈现经典滚轮效果。元素内容由回调按需提供：当某个槽位滚动到新的数据索引时回调被调用并填入内容，因此只用少量真实元素就能覆盖很大的取值范围。

为了较完整地体现该控件的能力，本例程在一屏里组合了三个滚轮：
- 顶部一个**横向、无限循环**的星期轮(文字内容，滚到尾自动绕回)；
- 中部两个**竖向、有界**的数字轮，组成「时:分」时间选择器。

三个滚轮各自通过 `middle_cb` 把停稳的选中值汇总到下方的一个读数 `星期 HH:MM`。

要点：
- 元素类型由 `lv_mulroller_set_obj_type()` 决定：`MULROLLER_TYPE_LABEL`(文字，本例使用)、`MULROLLER_TYPE_IMG`(图片)、`MULROLLER_TYPE_IMGARRAY`(图片数字)、`MULROLLER_TYPE_MODULE`(自定义模块)。
- 内容回调 `appear_cb(child, idx)`：滚轮滚动时为每个进入新数据索引的槽位调用，`child` 是该元素的子对象，`idx` 是该槽位的数据值 —— 在这里产生滚轮显示的内容。数字轮按 `"%02d"` 显示；星期轮用 `idx` 去查名字表，并对 7 取模实现绕回。
- 选择回调 `middle_cb(child, idx)`：滚轮停稳时调用，`idx` 即当前居中(被选中)的数据索引 —— 这就是读取选中值的入口。
- 方向由 `lv_mulroller_set_dir()` 决定：`MULROLLER_DIR_HOR`(横向)或 `MULROLLER_DIR_VER`(竖向)。
- 循环模式由 `lv_mulroller_set_circle_mode()` 决定：`MULROLLER_CIRCLE_NORMAL` 受 `set_circle_range(min,max)` 约束、到边界停住；`MULROLLER_CIRCLE_INFINITE` 无限循环、自动绕回。
- 强调居中项靠三组“范围 + 模式”：缩放 `set_zoom_range`、颜色 `set_color_mode(MULROLLER_COLOR_POS)`/`set_color_range`(随位置改变颜色)、透明度 `set_opa_mode(MULROLLER_OPA_MID)`/`set_opa_range`。范围要配合对应的 mode 打开才生效；其中 `set_opa_range` 仅在 `MULROLLER_CIRCLE_NORMAL` 下有效(无限模式忽略)。
- 元素类型为 LABEL 时，`set_zoom_range` 的值表示字号(会映射到主题字体)，中间字号大、两侧字号小，形成大小渐变。
- 能否滚动取决于几何关系：窗口要小于所有元素的总尺寸(竖向比高、横向比宽)，所以要创建比可见数量更多的元素。
- 无限循环模式不能与 `MULROLLER_LAYOUT_OVERLAP` 同用，且元素数量不少于 2。
- 滚轮会给每个元素套一层默认的 `lv_obj` 卡片(边框/背景/滚动条)。本例对每个元素 holder 调 `lv_obj_remove_style_all()` 去掉这层外观，只留内容。
- 所有设置完成后必须调用 `lv_mulroller_validate()` 让配置生效。

## 用法（关键 API）

```c
/* 数字轮：滚动时为每个槽位填内容，idx 即该槽位的数据值 */
static bool num_appear_cb(lv_obj_t *label, int16_t idx)
{
    lv_label_set_text_fmt(label, "%02d", idx);
    return true;
}

/* 停稳时回调，idx 即当前居中(选中)的值 */
static bool hh_middle_cb(lv_obj_t *label, int16_t idx)
{
    s_hh = idx;            /* 记下选中的小时，再刷新合成读数 */
    readout_refresh();
    return true;
}

/* 竖向、有界的数字轮(时/分共用这套设置，仅范围与回调不同) */
lv_obj_t *r = lv_mulroller_create(parent);
lv_obj_set_size(r, 78, 156);                       /* 窗口 < 元素总高 -> 可滚动 */
lv_mulroller_set_obj_type(r, MULROLLER_TYPE_LABEL);
lv_mulroller_create_element(r, 5, 78, 52);         /* 5 个元素，可见 3 个 */

lv_mulroller_set_appear_cb(r, num_appear_cb);
lv_mulroller_set_middle_cb(r, hh_middle_cb);

lv_mulroller_set_dir(r, MULROLLER_DIR_VER);            /* 竖向 */
lv_mulroller_set_align(r, MULROLLER_ALIGN_CENTER);
lv_mulroller_set_layout_mode(r, MULROLLER_LAYOUT_MID);
lv_mulroller_set_circle_mode(r, MULROLLER_CIRCLE_NORMAL);
lv_mulroller_set_circle_range(r, 0, 23);              /* 取值范围 00..23 */

lv_mulroller_set_zoom_range(r, 36, 24);               /* LABEL: 值=字号，中大两侧小 */
lv_mulroller_set_color_mode(r, MULROLLER_COLOR_POS);
lv_mulroller_set_color_range(r, 0xFF0000, 0x9E9E9E);  /* 中间红 -> 两侧灰 */
lv_mulroller_set_opa_mode(r, MULROLLER_OPA_MID);
lv_mulroller_set_opa_range(r, 255, 130);              /* 中间不透明 -> 两侧渐淡 */

lv_mulroller_validate(r);                             /* 让以上设置生效 */

/* 横向、无限循环的星期轮：只需把方向改为 HOR、循环改为 INFINITE，
 * 并在 appear_cb 里用 idx 查名字表(对 7 取模实现绕回)即可。 */
```

## 支持的开发板

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## 编译和烧录

板子工程在 `project` 目录下，通过指定 board 编译对应板子的工程：
- 例如编译 sf32lb52-lcd_n16r8：在 `project` 目录执行 `scons --board=sf32lb52-lcd_n16r8 -j8`
- 烧录通过 build 目录下的 `download.bat`（或 `uart_download.bat`）进行
- SF32LB52x/SF32LB56x 系列会额外生成 `uart_download.bat`，执行后输入下载 UART 端口号即可

## 模拟器

在 `project` 目录执行 `scons --board=pc_hcpu -j8`，生成 `build_pc_hcpu/main.exe` 直接运行即可看到窗口。
- 需要先按本机 MSVC 配置改好 `SiFli-SDK/msvc_setup.bat`。

## 运行说明

例程启动后屏幕上有三个滚轮：顶部是一行横向的星期(居中那项大且红、两侧小且灰)，中部是「时:分」两个竖向数字轮(同样居中强调、两侧渐淡渐灰)。用鼠标(模拟器)或手指(板子)拖动各滚轮：
- 横向星期轮可一直往一个方向拖，到 Sun/Mon 处会自动绕回(无限循环)；
- 竖向时/分轮在 00 和上限处停住、不越界(有界循环)，松手后吸附到最近一项。

每次某个滚轮停稳，下方读数 `星期 HH:MM` 就更新为当前三轮的选中组合。

## 异常诊断

如有任何技术疑问，请在 GitHub 上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
