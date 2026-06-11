# LVGL v8 Baselabel 示例

源码路径: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_baselabel`

## 概述

本例程演示 `lvsf_baselabel` 控件：在普通 `lv_label` 之上内建了一条**数据刷新通路**。给它绑定一个或多个数据源 id、一个数据回调和一个刷新定时器后，控件会周期性地拉取数据并更新自身文本，于是标签能跟随实时数值变化，应用层不必自己轮询。它同时保留了 `lv_label` 的常规能力(set_text/set_text_fmt、长文本模式、recolor 重着色、字符串表等)。本例程显示一个每秒自增的运行时长 `uptime MM:SS`。

要点：
- 数据驱动刷新由四步组成：`lv_obj_set_source_id()` 绑定数据源 id、`lv_obj_set_gmdata_cb()` 注册数据回调、`lv_obj_create_refresh_timer()` 创建刷新定时器(传入控件自带的 `lv_baselabel_refresh_timer`)、`lv_obj_refresh_start()` 启动。这套 API 来自 `lvsf_obj_ext`。
- 刷新定时器到点时会调用 `lv_baselabel_refresh_timer()`，后者再回调你的 `gmdata_cb(label, id_tab, id_num)`，回调里把当前数据格式化进标签 —— 这就是 baselabel 把"实时数据源"变成"显示文本"的方式。
- 在回调里更新文本，请用 `lv_snprintf()` 自己拼好字符串再 `lv_baselabel_set_text()`；`lv_baselabel_set_text_fmt()` 是面向数据绑定的格式串，并非普通 printf，直接传可变参会得到错误结果。
- 不需要数据驱动时，baselabel 就是个增强标签：直接 `lv_baselabel_set_text()` 设固定文本，或用 `set_long_mode`、`set_recolor`、字符串表(`set_str_tab`/`add_sfat_str`/`select_sfat_str`)等。

## 用法（关键 API）

```c
/* 数据回调：刷新定时器到点时被调用，把数据格式化进标签 */
static int32_t uptime_gmdata_cb(lv_obj_t *label, uint32_t *id_tab, uint8_t id_num)
{
    static uint32_t secs = 0;
    secs++;
    char buf[32];
    lv_snprintf(buf, sizeof(buf), "uptime  %02u:%02u",
                (unsigned)((secs / 60) % 100), (unsigned)(secs % 60));
    lv_baselabel_set_text(label, buf);   /* 自己拼串 + set_text */
    return 0;
}

lv_obj_t *label = lv_baselabel_create(parent);
lv_baselabel_set_text(label, "uptime  00:00");   /* 首次刷新前的初始文本 */

/* 绑定数据源 + 回调 + 1 秒刷新定时器 */
static uint32_t source_id = 0x1105;
lv_obj_set_source_id(label, &source_id, 1);
lv_obj_set_gmdata_cb(label, uptime_gmdata_cb);
lv_obj_create_refresh_timer(label, 1000, lv_baselabel_refresh_timer);
lv_obj_refresh_start(label);
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

例程启动后屏幕中央显示蓝色大字 `uptime 00:00`，随后由刷新定时器每秒触发一次数据回调，标签自增为 `00:01`、`00:02`…… 应用层没有在主循环里更新文本，文本的变化完全由 baselabel 的数据刷新通路驱动。

## 异常诊断

如有任何技术疑问，请在 GitHub 上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
