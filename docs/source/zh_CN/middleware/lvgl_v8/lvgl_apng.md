# LVGL v8 APNG 动图（apng / ezipa）

SiFli SDK 在 LVGL v8 上基于 `lvsf_ezipa` 组件实现了 APNG（动态 PNG）格式图片的解析、播放与控制，对应头文件为 `lvsf_ezipa.h`。它适用于需要带透明度图片组成的序列帧动画场景，控件可解码 APNG 帧并按帧间隔自动播放，支持暂停、恢复、循环次数、缩放、透明度与播放结束回调。

```{note}
`lv_ezipa_*` 接口不在 `solution/framework/gui_widget` 目录下定义，当前实现位于 SDK 的 `middleware/lvgl/lvsf/lvsf_ezipa.h`，由 `USING_EZIPA_DEC` 开关控制。
```

## 功能简介

- 解析并播放 APNG（动态 PNG）序列帧动画，支持帧透明度。
- 支持暂停 / 恢复 / 延迟恢复播放。
- 支持设置循环次数（无限循环 / 播放 1 次 / 播放 N 次）。
- 支持强制设置帧间隔、缩放比例与透明度。
- 支持播放结束回调，以及基于前缀的多文件选择播放（NAND 文件系统）。

## 使用场景

- 带 alpha 通道的序列帧动画（如loading 动画、动态图标）。
- 需要循环播放并在结束时触发回调的一次性动画。
- 需要在页面 resume/pause 时同步控制播放与暂停的动画。

## 支持的开发板

通用 LVGL v8 例程支持的平台，55x 之后的开发板均可使用（如 58x、56x、52x）。需在 `menuconfig` 中启用 `USING_EZIPA_DEC`。

## 配置与初始化

APNG 播放由 `USING_EZIPA_DEC` 开关控制，在 `menuconfig` 中启用。颜色深度支持 16 位（RGB565）和 24 位（RGB888）。

```c
#include "lvsf/lvsf_ezipa.h"
```

播放状态枚举（`lv_ezipa_status_t`）：`LV_EZIPA_STOP`（停止）、`LV_EZIPA_CURR`（当前帧）、`LV_EZIPA_NEXT`（下一帧）。

## API 说明

以下接口签名逐字取自 `middleware/lvgl/lvsf/lvsf_ezipa.h`：

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_ezipa_create(lv_obj_t *parent)` | 创建 APNG 播放控件 | `parent`：父对象；返回创建的控件指针 |
| `void lv_ezipa_set_src(lv_obj_t *ezipa, const char *src)` | 设置 APNG 文件源路径 | `src`：APNG 文件路径（如 `"fs:/xxx.apng"`） |
| `void lv_ezipa_set_surface(lv_obj_t *ezipa, const void *src)` | 设置 APNG 播放背景层 | `src`：背景层数据源 |
| `void lv_ezipa_pause(lv_obj_t *ezipa)` | 暂停 APNG 播放 | 无返回值 |
| `void lv_ezipa_resume(lv_obj_t *ezipa)` | 恢复 APNG 播放 | 无返回值 |
| `void lv_ezipa_resume_with_delay(lv_obj_t *ezipa, uint16_t delay_time)` | 延迟恢复播放 | `delay_time`：延迟时间（ms） |
| `void lv_ezipa_set_loop_times(lv_obj_t *ezipa, int times)` | 设置播放循环次数 | `times`：`EZIPA_LOOP_FOREVER`(-1) 无限循环；0 播放 1 次；N 播放 N 次 |
| `void lv_ezipa_set_play_end_cb(lv_obj_t *ezipa, lv_ezipa_end_cb_t cb)` | 设置播放结束回调 | `cb`：原型为 `void (*)(lv_obj_t *ezipa)`；仅非循环模式下有效 |
| `void lv_ezipa_set_interval(lv_obj_t *ezipa, int32_t interval)` | 强制设置帧播放间隔 | `interval`：帧间隔（ms），>0 时覆盖 APNG 内置间隔 |
| `void lv_ezipa_set_zoom(lv_obj_t *ezipa, uint16_t zoom)` | 设置 APNG 缩放比例 | `zoom`：LVGL 标准缩放参数 |
| `void lv_ezipa_set_opa(lv_obj_t *ezipa, uint16_t opa)` | 设置 APNG 透明度 | `opa`：0~255，0 全透，255 不透明 |
| `void lv_ezipa_select(lv_obj_t *ezipa, uint8_t idx)` | 选择待播放的前缀索引 | 仅在设置了 select prefix 时有效 |
| `void lv_ezipa_set_select_prefix(lv_obj_t *ezipa, const void *ezipa_prefix, const void *surface_prefix, uint8_t max_num)` | 设置多文件前缀播放 | 仅支持 NAND 文件系统；`max_num` 最大为 99 |

## 典型用法

创建 APNG 控件、设置源与背景层，并在页面生命周期中同步播放/暂停：

```c
static lv_obj_t *p_apng = NULL;

static void apng_play_end_cb(lv_obj_t *ezipa)
{
    rt_kprintf("play end.\n");
}

static void on_start(void)
{
    p_apng = lv_ezipa_create(lv_scr_act());
    lv_ezipa_set_src(p_apng, APP_GET_IMG(apng));
    lv_ezipa_set_surface(p_apng, APP_GET_IMG(apng_thum));
    lv_ezipa_set_interval(p_apng, 50);
    lv_obj_center(p_apng);
}

static void on_resume(void)
{
    lv_ezipa_resume(p_apng);
}

static void on_pause(void)
{
    lv_ezipa_pause(p_apng);
}
```

```{note}
播放结束回调仅在非循环模式下被调用。默认 `loop_times = -1` 为无限循环，此时不会触发 `play_end` 回调。
```

## 效果展示

```{image} ../../../assets/lvgl_v8/apng.gif
:alt: apng 动图播放效果
```

## 注意事项

- 序列帧图片合成后的 APNG 文件后缀仍为 `.png`。
- 循环次数：`-1` 无限循环（默认）；`0` 播放 1 次；`>0` 按设定次数播放。
- 播放结束回调仅在非循环模式下有效。

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- 源码路径：`middleware/lvgl/lvsf/lvsf_ezipa.h`
