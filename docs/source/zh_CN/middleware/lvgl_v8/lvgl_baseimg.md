# LVGL v8 基础图像（Baseimg）

`lvsf_baseimg` 是 SiFli 基于 LVGL `lv_img` 封装的基础图像控件，是 `lvsf_imgarray` 等图片类控件的基类。它支持多种图像显示模式：图组索引/数值/Q24.8 定点、指针（无级变化/跳针）、序列帧（正向/逆向/正逆向循环），并支持角度、缩放、旋转中心、循环次数和值表映射。

- 主要头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_baseimg.h_
- 依赖控件：LVGL img（`LV_USE_IMG`）

```{note}
该控件暂无对应 `lvgl_v8_baseimg` example，详细用法参考头文件注释及 SDK 配置。
```

## 功能简介

- 支持多种图像类型：图组索引（数值即序号）、图组数值（一段数值对应一张图）、图组 Q24.8 定点、指针无级变化、指针跳针、序列帧正/逆/正逆向循环。
- 支持三种图像源类型：内存描述符、文件、序列帧。
- 支持角度、缩放、旋转中心 X/Y 设置。
- 支持序列帧起止索引、当前索引、空索引设置与循环次数控制（-1 无限循环）。
- 支持状态回调（播放开始/停止/恢复/暂停/完成等）和索引回调。
- 支持值表（value_table）做数值到图像/位置的映射。

## 使用场景

- 序列帧动画播放（正向/逆向/往复循环）。
- 指针式仪表表盘（无级或跳针）。
- 图组索引切换、数值到图片映射的指示器。
- 作为 `lvsf_imgarray` 等控件的基类使用。

## 支持的开发板

见对应 lvgl_v8 example 或 SDK 配置。

## 配置与初始化

`lvsf_baseimg` 依赖 LVGL img 组件。在 menuconfig 中确认已开启：

```none
CONFIG_LV_USE_IMG=y
```

BSP 中通过 `LVSF_USE_BASEIMG` 宏控制该控件的编译。

创建对象的最小流程：

```c
#include "lvsf_baseimg.h"

lv_obj_t *img = lv_baseimg_create(lv_scr_act());
```

## API 说明

### 枚举定义

**图像类型 `lv_baseimg_type_t`：**

| 取值 | 含义 |
| --- | --- |
| `BASEIMG_TYPE_ARRAY_INDEX` | 图组，序列模式，数值就是图片序号 |
| `BASEIMG_TYPE_ARRAY_VALUE` | 图组，数值模式，一段数值对应一张图 |
| `BASEIMG_TYPE_ARRAY_Q248` | 图组，数字模式，数据是 Q24.8 格式 |
| `BASEIMG_TYPE_POINTER` | 指针，无级变化 |
| `BASEIMG_TYPE_POINTER_GRID` | 跳针 |
| `BASEIMG_TYPE_SEQUENCE` | 序列帧，持续正向循环（0x0f） |
| `BASEIMG_TYPE_SEQUENCE_BACK` | 序列帧，持续逆向循环（0x1f） |
| `BASEIMG_TYPE_SEQUENCE_CIRCLE` | 序列帧，持续正逆向循环（0x10f） |

**播放状态 `lv_baseimg_state_t`：**

| 取值 | 含义 |
| --- | --- |
| `BASEIMG_STATE_NULL` | 空状态 |
| `BASEIMG_STATE_PLAY_START` | 序列帧播放开始（停止或刚开始触发） |
| `BASEIMG_STATE_PLAY_STOP` | 序列帧播放停止，序号归零 |
| `BASEIMG_STATE_PLAY_RESUME` | 序列帧播放恢复（暂停后播放触发） |
| `BASEIMG_STATE_PLAY_PAUSE` | 序列帧播放暂停，序号不变 |
| `BASEIMG_STATE_PLAY_DONE` | 序列帧正向播放完成 |
| `BASEIMG_STATE_PLAY_BACK_DONE` | 序列帧反向播放完成 |

**源类型 `lv_baseimg_src_type_t`：**

| 取值 | 含义 |
| --- | --- |
| `BASEIMG_SRC_TYPE_DSC` | 内存描述符 |
| `BASEIMG_SRC_TYPE_FILE` | 文件 |
| `BASEIMG_SRC_TYPE_SEQUENCE` | 序列帧 |

### 回调类型

| 类型定义 | 功能说明 |
| --- | --- |
| `typedef void (*lv_baseimg_index_cb)(lv_obj_t *baseimg)` | 基础图像设置到指定索引时调用 |
| `typedef void (*lv_baseimg_state_cb)(lv_obj_t *baseimg, lv_baseimg_state_t state)` | 基础图像状态改变时调用；`state` 为新状态 |

### 创建与刷新

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_baseimg_create(lv_obj_t *parent)` | 创建基础图像对象 | `parent`：父对象；成功返回对象指针，失败返回 `NULL` |
| `void lv_baseimg_refresh_timer(lv_timer_t *timer)` | 刷新定时器回调 | `timer`：定时器对象；定时刷新基础图像显示 |

### 状态与源设置

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `void lv_baseimg_set_state(lv_obj_t *img, lv_baseimg_state_t state)` | 设置状态 | `state`：状态值（播放控制） |
| `void lv_baseimg_set_src_array(lv_obj_t *img, const lv_img_dsc_t **dsc_array, int16_t index_star, int16_t index_end)` | 设置源数组（内存描述符） | `dsc_array`：描述符数组；`index_star`/`index_end`：起始/结束索引 |
| `void lv_baseimg_set_src_array2(lv_obj_t *img, char *file_path, lv_img_file_data_t *dsc_array, int16_t index_start, int16_t index_end)` | 通过文件路径设置源数组 | `file_path`：文件路径；`dsc_array`：文件描述符数组；`index_start`/`index_end`：起止索引 |
| `void lv_baseimg_set_src_array3(lv_obj_t *img, const void *src, int16_t index_start, int16_t index_end)` | 设置序列帧图片源和序号 | `src`：图片源；`index_start`/`index_end`：起止索引 |
| `void lv_baseimg_set_state_cb(lv_obj_t *img, lv_baseimg_state_cb cb)` | 设置状态回调 | `cb`：状态改变时的回调函数 |
| `void lv_baseimg_set_value_table(lv_obj_t *img, char *value_table)` | 设置值表 | `value_table`：值表指针，用于数值到图像的映射 |

### 角度、缩放与索引

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `void lv_baseimg_set_angle(lv_obj_t *img, int16_t angle)` | 设置图像角度 | `angle`：角度值 |
| `void lv_baseimg_set_zoom(lv_obj_t *img, uint16_t zoom)` | 设置图像缩放 | `zoom`：缩放比例 |
| `void lv_baseimg_set_start_index(lv_obj_t *img, uint16_t index)` | 设置起始索引 | `index`：序列起始索引 |
| `void lv_baseimg_set_end_index(lv_obj_t *img, uint16_t index)` | 设置结束索引 | `index`：序列结束索引 |
| `void lv_baseimg_set_current_index(lv_obj_t *img, uint16_t index)` | 设置当前索引 | `index`：当前索引 |
| `void lv_baseimg_set_img_type(lv_obj_t *img, uint16_t img_type)` | 设置图像类型 | `img_type`：图像类型（`lv_baseimg_type_t`） |
| `void lv_baseimg_set_empty_idx(lv_obj_t *img, uint16_t index)` | 设置空索引 | `index`：空图像的索引 |
| `void lv_baseimg_set_pivot_X(lv_obj_t *img, int16_t x)` | 设置旋转中心 X 坐标 | `x`：旋转中心 X |
| `void lv_baseimg_set_pivot_y(lv_obj_t *img, int16_t y)` | 设置旋转中心 Y 坐标 | `y`：旋转中心 Y |
| `void lv_baseimg_set_loop(lv_obj_t *img, int16_t loop)` | 设置循环次数 | `loop`：循环次数，-1 表示无限循环 |

### 查询接口

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `int16_t lv_baseimg_get_angle(lv_obj_t *img)` | 获取当前角度 | 返回角度值 |
| `uint16_t lv_baseimg_get_zoom(lv_obj_t *img)` | 获取当前缩放 | 返回缩放值 |
| `uint16_t lv_baseimg_get_start_index(lv_obj_t *img)` | 获取起始索引 | 返回起始索引 |
| `uint16_t lv_baseimg_get_end_index(lv_obj_t *img)` | 获取结束索引 | 返回结束索引 |
| `uint16_t lv_baseimg_get_current_index(lv_obj_t *img)` | 获取当前索引 | 返回当前索引 |
| `uint16_t lv_baseimg_get_img_type(lv_obj_t *img)` | 获取图像类型 | 返回图像类型（`lv_baseimg_type_t`） |
| `uint16_t lv_baseimg_get_empty_idx(lv_obj_t *img)` | 获取空索引 | 返回空图像索引 |
| `int16_t lv_baseimg_get_pivot_x(lv_obj_t *img)` | 获取旋转中心 X 坐标 | 返回旋转中心 X |
| `int16_t lv_baseimg_get_pivot_y(lv_obj_t *img)` | 获取旋转中心 Y 坐标 | 返回旋转中心 Y |

## 典型用法

根据头文件接口，序列帧正向循环播放的典型流程为：创建对象 → 设置图像类型为序列帧 → 设置源数组 → 设置循环次数 → 启动播放：

```c
#include "lvsf_baseimg.h"

lv_obj_t *img = lv_baseimg_create(parent);

/* 序列帧：持续正向循环 */
lv_baseimg_set_img_type(img, BASEIMG_TYPE_SEQUENCE);
lv_baseimg_set_src_array(img, frame_dsc_arr, 0, FRAME_CNT - 1);
lv_baseimg_set_loop(img, -1);                 /* 无限循环 */
lv_baseimg_set_state(img, BASEIMG_STATE_PLAY_START);   /* 启动播放 */
```

指针式表盘（无级变化）的典型流程为：

```c
lv_baseimg_set_img_type(img, BASEIMG_TYPE_POINTER);
lv_baseimg_set_pivot_X(img, 60);
lv_baseimg_set_pivot_y(img, 60);
lv_baseimg_set_angle(img, angle);             /* 角度驱动指针 */
```

详细用法参考头文件注释。

## 效果展示

该控件暂无对应 example，运行 `lvgl_v8_imgarray` 等基于 baseimg 的示例可查看其派生控件效果。

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [LVGL v8 Image 文档](https://docs.lvgl.io/8.3/widgets/img.html)
- 源码路径：`middleware/lvgl/lvsf/gui_widgets/lvsf_baseimg.h`
