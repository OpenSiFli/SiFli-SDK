# LVGL v8 序列帧动画（seqframe / seqimg）

SiFli SDK 在 LVGL v8 上提供了图片序列帧播放控件，对应 SDK 头文件为 `lvsf/lv_seqimg.h`（控件类名为 `lv_seqimg`）。它基于 LVGL 图片对象扩展，用于把一组图片按顺序逐帧播放，支持内存打包序列帧（`lv_img_dsc_t` 数组）和文件系统逐张图片（文件路径数组）两种资源来源，支持播放、暂停、按索引选帧和帧间隔设置。

```{note}
solution 文档中使用的 `lv_seqframe_*` 接口与当前 SDK v8 头文件 `lvsf/lv_seqimg.h` 中的 `lv_seqimg_*` 命名不同。本文 API 表格以 SDK 头文件为准；`lv_seqframe_*` 相关的播放模式、prefix 路径等功能描述可作为设计参考。
```

## 功能简介

- 把一组图片按顺序逐帧播放，支持内存数组和文件路径数组两种来源。
- 支持播放 / 暂停控制与按索引直接选帧显示。
- 支持设置帧播放间隔。
- 资源可以是固件内置打包的序列帧，也可以是文件系统中按序号命名的逐张图片。

## 使用场景

- loading 动画、WiFi 连接中、充电动画等循环序列帧。
- 从视频抽帧后按顺序播放的简单动画。
- 需要在文件系统（如 SD 卡）中按序号存放并逐张加载帧的场景。

## 支持的开发板

通用 LVGL v8 例程支持的平台，55x 之后的开发板均可使用（如 58x、56x、52x）。

## 配置与初始化

序列帧控件由 `lvsf/lv_seqimg.h` 提供，随 LVGL v8 工程编译启用：

```c
#include "lvsf/lv_seqimg.h"
```

## API 说明

以下接口签名逐字取自 `middleware/lvgl/lvsf/lv_seqimg.h`：

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_seqimg_create(lv_obj_t *parent)` | 创建序列帧控件实例 | `parent`：父对象；返回控件指针 |
| `void lv_seqimg_src_array(lv_obj_t *obj, const lv_img_dsc_t **dsc_array, uint16_t size)` | 绑定内存中的序列帧描述符数组 | `dsc_array`：`lv_img_dsc_t` 指针数组；`size`：帧数量 |
| `void lv_seqimg_file_array(lv_obj_t *obj, const char **file_path_array, uint16_t size)` | 绑定文件系统中的图片路径数组 | `file_path_array`：图片文件路径数组；`size`：帧数量 |
| `void lv_seqimg_select(lv_obj_t *obj, uint16_t index)` | 跳转到指定索引帧并显示 | `index`：帧索引（从 0 开始） |
| `void lv_seqimg_play(lv_obj_t *obj)` | 开始播放序列帧 | 无返回值 |
| `void lv_seqimg_pause(lv_obj_t *obj)` | 暂停播放 | 无返回值 |
| `void lv_seqimg_set_period(lv_obj_t *obj, uint32_t period)` | 设置帧播放间隔 | `period`：帧间隔（ms） |

## 典型用法

播放固件内置打包的序列帧：

```c
/* dsc_array 为按播放顺序排列的 lv_img_dsc_t 指针数组，count 为帧数 */
lv_obj_t *seqframe = lv_seqimg_create(lv_scr_act());
lv_seqimg_src_array(seqframe, dsc_array, count);
lv_seqimg_set_period(seqframe, 50);   /* 帧间隔 50ms */
lv_seqimg_select(seqframe, 0);        /* 从第 0 帧开始 */
lv_obj_align(seqframe, LV_ALIGN_CENTER, 0, 0);
lv_seqimg_play(seqframe);
```

播放文件系统中按序号命名的逐张图片：

```c
const char *paths[] = {"/sd/photo/beauty0.bin",
                       "/sd/photo/beauty1.bin",
                       "/sd/photo/beauty2.bin"};

lv_obj_t *seq = lv_seqimg_create(lv_scr_act());
lv_seqimg_file_array(seq, paths, 3);
lv_seqimg_set_period(seq, 500);
lv_seqimg_select(seq, 0);
lv_seqimg_play(seq);
```

```{note}
文件系统逐张播放时，外置资源文件应按播放顺序以“图片名 + 连续数字序号 + 统一后缀”命名，控件才能正确按索引加载。内置数组与文件数组二选一，不要对同一控件同时配置两种来源。
```

## 效果展示

```{image} ../../../assets/lvgl_v8/seq_frame_demo_src.png
:alt: seqframe 内置序列帧资源
```

```{image} ../../../assets/lvgl_v8/seq_frame_demo.gif
:alt: seqframe 内置序列帧播放效果
```

```{image} ../../../assets/lvgl_v8/seq_frame_demo_prefix.png
:alt: seqframe 外置目录序列帧资源
```

```{image} ../../../assets/lvgl_v8/prefix_seq_frame_demo.gif
:alt: seqframe 外置目录序列帧播放效果
```

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- 源码路径：`middleware/lvgl/lvsf/lv_seqimg.h`
