# LVGL v8 多动画（multanim）

SiFli SDK 在 LVGL v8 上提供了一套面向场景切换的过渡动画封装，由两部分组成：

- **multanim 控件**（`lvsf_multanim.h`）：把缩放、3D 翻转、轴向压缩、mask 渐隐等常用过渡动画封装成一个对象。应用只需创建实例、设置动画类型与主/次图像，再按 `[0, 1024]` 区间更新进度即可驱动动画，无需自己处理每帧的绘制逻辑。
- **switchanim 框架**（`lvsf_switchanim.h`）：屏幕（界面）之间切换的动画调度框架，管理 enter/exit 动画的创建、播放、结束与回调，配合 `BUILTIN_ANIMATION` 宏把自定义动画注册到链接段。

场景切换过程中同一种动画会反复出现（如 APP 切换、TLV 切换、表盘切换），这套封装把重复的动画绘制逻辑集中实现，应用调用统一接口即可复用。

## 功能简介

- 封装缩放（Zoom）、3D 翻转（3D）、反向翻转带缩放（Switch）、半页翻转（Turn）、轴向压缩（Scale）、mask 渐隐（Fade）、中间向两侧打开（Open）等多种过渡动画。
- 翻书（Book）、飞梭（Shuttle）、百叶窗（Shutter）等动画仅在 58x 及后续芯片（依赖 GPU / VGLite）上可用。
- 动画以进度值 `[0, 1024]` 驱动，既支持由 LVGL 动画自动播放（`lv_multanim_start_anim()`），也支持由触摸/手势按帧手动推进（`lv_multanim_set_process()`）。
- 支持左右渐隐 mask 资源，可从文件路径加载或由应用直接构造 `lv_img_dsc_t`。
- switchanim 框架通过优先级覆盖、链接段注册内置动画，实现界面切换动画的统一调度。

## 使用场景

- 手表/智能家居表盘、APP 列表之间的左右滑动切换。
- TLV（title-list-view）等多列表控件的页面过渡。
- 进入/退出二级界面时的进入（enter）与退出（exit）动画。
- 需要按手指拖拽位置实时预览过渡效果的交互场景。

## 支持的开发板

参考 `lvgl_v8_multanim` 例程：

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp
- 通用：55x 之后的开发板（如 58x、56x、52x）。

```{note}
3D、Roll、Turn、Scale 等动画依赖 GPU（`LV_USE_GPU` / EPIC）；Book、Shuttle、Shutter 依赖 VGLite（`USING_VGLITE`），仅 58x 及后续芯片可用。具体是否可用由编译配置和板级能力决定。
```

## 配置与初始化

multanim 由 `LVSF_USE_MULTANIM` 开关控制，在 `menuconfig` 的 `LittlevGL2RTT -> SiFli extend` 中启用。标准 GUI 初始化流程会在 `gui_lib_init()` 中完成 switchanim 框架的初始化（`lv_switchanim_init()`），应用通常无需再次调用。

```c
#include "lvsf.h"
#include "lvsf/gui_widgets/lvsf_multanim.h"
#include "lvsf/gui_widgets/lvsf_switchanim.h"
```

## API 说明

### multanim 控件接口（`lvsf_multanim.h`）

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_multanim_create(lv_obj_t *parent)` | 创建 multanim 动画实例 | `parent`：父对象；返回创建的对象指针，失败返回 `NULL` |
| `lv_multanim_type lv_multanim_set_type(lv_obj_t *multanim, lv_multanim_type type)` | 设置动画类型，需在启动动画前设置 | `multanim`：实例；`type`：动画类型；返回设置前的动画类型 |
| `void lv_multanim_set_dir(lv_obj_t *multanim, lv_multanim_dir dir)` | 设置动画方向 | `dir`：方向，可取 `LV_MULTANIM_LEFT/RIGHT/TOP/BOTTOM`，支持位或组合（如 `LV_MULTANIM_HOR`） |
| `void lv_multanim_set_major_img(lv_obj_t *multanim, lv_obj_t *major_img)` | 设置主动画图像（必须设置） | `major_img`：主图像对象指针 |
| `void lv_multanim_set_minor_img(lv_obj_t *multanim, lv_obj_t *minor_img)` | 设置次动画图像 | `minor_img`：次图像对象指针 |
| `void lv_multanim_set_range(lv_obj_t *multanim, int32_t range)` | 设置动画范围 | `range`：动画范围值 |
| `void lv_multanim_set_viewpoint(lv_obj_t *multanim, lv_point_t *start_v, lv_point_t *end_v)` | 设置动画视角起止点 | `start_v`/`end_v`：起始/结束视角点 |
| `void lv_multanim_set_zoom(lv_obj_t *multanim, lv_coord_t start_zoom, lv_coord_t zoom_end)` | 设置缩放起止值 | `start_zoom`/`zoom_end`：起始/结束缩放值 |
| `void lv_multanim_set_process(lv_obj_t *multanim, int32_t process)` | 设置动画进度（核心控制接口） | `process`：进度值，范围 `[0, 1024]` |
| `void lv_multanim_set_mask(lv_obj_t *multanim, const lv_img_dsc_t *mask_l, const lv_img_dsc_t *mask_r)` | 为蒙版类动画设置左右蒙版 | `mask_l`/`mask_r`：左/右侧渐隐蒙版描述符；不需要时传 `NULL` |
| `int32_t lv_multanim_get_proc(lv_obj_t *multanim)` | 获取当前动画进度 | 返回当前进度值 |
| `lv_obj_t *lv_multanim_get_major_img(lv_obj_t *multanim)` | 获取主动画图像对象 | 返回主图像对象指针 |
| `lv_obj_t *lv_multanim_get_minor_img(lv_obj_t *multanim)` | 获取次动画图像对象 | 返回次图像对象指针 |
| `lv_img_dsc_t *lv_multanim_create_mask(const void *src)` | 按路径读取蒙版图像描述符 | `src`：蒙版文件/资源路径；返回蒙版描述符指针 |
| `void lv_multanim_free_mask(lv_img_dsc_t *dsc)` | 释放 `lv_multanim_create_mask()` 创建的蒙版描述符 | `dsc`：待释放的蒙版描述符 |
| `lv_multanim_type lv_multanim_get_type(lv_obj_t *multanim)` | 获取当前动画类型 | 返回当前动画类型 |
| `void lv_multanim_start_anim(lv_obj_t *multanim, uint32_t period, int32_t start_pro, int32_t end_pro, lv_anim_ready_cb_t ready_cb)` | 启动自动播放动画 | `period`：时长（ms）；`start_pro`/`end_pro`：起止进度 `[0,1024]`；`ready_cb`：动画结束回调 |

动画类型枚举（`lv_multanim_type`）：`LV_MULTANIM_NONE`、`LV_MULTANIM_ZOOM`、`LV_MULTANIM_3D`、`LV_MULTANIM_SWITCH`、`LV_MULTANIM_TURN`、`LV_MULTANIM_SCALE`、`LV_MULTANIM_FADE`、`LV_MULTANIM_OPEN`、`LV_MULTANIM_ROLL`、`LV_MULTANIM_DRAG`、`LV_MULTANIM_BOOK`、`LV_MULTANIM_SHUTTLE`、`LV_MULTANIM_SHUTTER`。

### switchanim 框架接口（`lvsf_switchanim.h`）

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_baseanim_get_original_screen(lv_baseanim_t *baseanim)` | 获取动画对应的原始屏幕对象 | 返回原始屏幕对象指针 |
| `lv_baseanim_para_t *lv_baseanim_get_para(lv_baseanim_t *baseanim)` | 获取动画参数 | 返回参数结构体指针 |
| `lv_baseanim_type lv_baseanim_get_type(lv_baseanim_t *baseanim)` | 获取动画类型（enter/exit） | 返回 `LV_BASEANIM_ENTER_TYPE` / `LV_BASEANIM_EXIT_TYPE` 等 |
| `void lv_switchanim_init()` | 初始化动画列表并加载所有动画 | 标准启动流程自动调用 |
| `void lv_switchanim_load()` | 加载所有动画 | 无返回值 |
| `void lv_switchanim_deinit()` | 反初始化动画列表 | 无返回值 |
| `lv_baseanim_cb *lv_switchanim_register_anim(char *name, uint32_t major, lv_baseanim_progress_cb progress_cb)` | 注册一个动画到列表 | 返回注册后的动画回调描述符 |
| `void lv_switchanim_unregister_anim(uint32_t major)` | 按 major 移除动画 | 无返回值 |
| `void lv_switchanim_overwrite(const lv_baseanim_para_t *enter_org, lv_baseanim_para_t *enter_anim, const lv_baseanim_para_t *exit_org, lv_baseanim_para_t *exit_anim, uint32_t flag)` | 按优先级覆盖 enter/exit 动画参数 | `flag`：正向或反向动画标志 |
| `lv_obj_t *lv_switchanim_create(lv_obj_t *parent, lv_obj_t *enter_screen, lv_obj_t *exit_screen, lv_baseanim_para_t *enter_para, lv_baseanim_para_t *exit_para)` | 创建 switchanim 实例 | 返回实例句柄；`parent` 为 `NULL` 时挂到根屏幕 |
| `void lv_switchanim_manual_run(lv_obj_t *switchanim, int32_t progress)` | 手动按进度百分比推进动画帧 | 用于跟随手势的交互动画 |
| `void lv_switchanim_manual_finish(lv_obj_t *switchanim, bool is_enter)` | 手动结束动画 | `is_enter`：结束后加载 enter 还是 exit 屏幕 |
| `void lv_switchanim_auto_run(lv_obj_t *switchanim, uint32_t period, uint32_t progress, bool reverse)` | 自动播放动画 | `period`：时长；`progress`：起始进度；`reverse`：是否反向 |
| `void lv_switchanim_set_finish_cb(lv_obj_t *switchanim, lv_switchanim_finish_cb finish_cb)` | 设置动画结束回调 | 无返回值 |
| `void lv_switchanim_set_path(lv_obj_t *switchanim, lv_baseanim_path path)` | 设置进度增长的缓动路径 | `path`：`LINE`/`EASE_IN`/`EASE_OUT`/`EASE_IN_OUT` |
| `void lv_switchanim_set_def_anim(uint16_t anim_major, uint16_t anim_minor)` | 设置默认动画 | 无返回值 |
| `lv_baseanim_cb *lv_switchanim_find_anim(uint32_t major)` | 按 major 查找动画描述符 | 返回描述符指针 |

内置动画通过 `BUILTIN_ANIMATION(anim_name, anim_major, anim_progress_cb)` 宏注册到 `switch_anim` 链接段。

## 典型用法

最小使用流程：创建实例 → 设置类型 → 设置主/次图像 → 启动或按帧更新进度。

```c
#include "lvsf/gui_widgets/lvsf_multanim.h"

static lv_obj_t *multanim;

void demo_multanim(void)
{
    lv_obj_t *scr = lv_scr_act();

    /* 1. 创建 multanim 实例 */
    multanim = lv_multanim_create(scr);
    lv_obj_set_size(multanim, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
    lv_obj_center(multanim);

    /* 2. 创建主/次图像 */
    lv_obj_t *img1 = lv_img_create(multanim);
    lv_obj_t *img2 = lv_img_create(multanim);
    lv_obj_center(img1);
    lv_obj_center(img2);

    /* 3. 设置动画类型、方向与主/次图像 */
    lv_multanim_set_type(multanim, LV_MULTANIM_3D);
    lv_multanim_set_dir(multanim, LV_MULTANIM_HOR);
    lv_multanim_set_major_img(multanim, img1);
    lv_multanim_set_minor_img(multanim, img2);
    lv_multanim_set_zoom(multanim, LV_IMG_ZOOM_NONE, LV_IMG_ZOOM_NONE >> 1);

    /* 4. 自动播放：从进度 0 到 1024，时长 1000ms */
    lv_multanim_start_anim(multanim, 1000, 0, 1024, NULL);
}
```

跟随手势手动推进时，把手指偏移量映射到 `[0, 1024]` 后调用 `lv_multanim_set_process()`：

```c
/* offset 为水平拖拽偏移，hor_res 为屏幕宽度 */
int32_t proc = (offset << 10) / hor_res;
lv_multanim_set_process(multanim, proc);
```

在 switchanim 框架中注册一个自定义 3D 切换动画：

```c
static void turn3d_anim_progress(lv_baseanim_t *baseanim, lv_obj_t *anim_obj, int32_t progress)
{
    if (LV_BASEANIM_EXIT_TYPE == lv_baseanim_get_type(baseanim))
    {
        /* 创建 multanim 实例并设置类型、主图 */
        lv_multanim_set_process(switch_multanim, progress);
    }
    else
    {
        lv_multanim_set_minor_img(switch_multanim, anim_obj);
    }
}

BUILTIN_ANIMATION(turn3Danim, LV_SWITCHANIM_TURN_3D, turn3d_anim_progress);
```

## 效果展示

```{image} ../../../assets/lvgl_v8/multanim_all.gif
:alt: multanim 全部动画效果
```

```{image} ../../../assets/lvgl_v8/multanim_3d.gif
:alt: multanim 3D 翻转动画
```

```{image} ../../../assets/lvgl_v8/multanim_tlv.gif
:alt: multanim TLV 平铺切换动画
```

```{image} ../../../assets/lvgl_v8/switch_anim_conf.png
:alt: switchanim 动画配置
```

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- 源码路径：`middleware/lvgl/lvsf/gui_widgets/lvsf_multanim.h`、`middleware/lvgl/lvsf/gui_widgets/lvsf_switchanim.h`
- 例程路径：`example/multimedia/lvgl/lvgl_v8_multanim`
