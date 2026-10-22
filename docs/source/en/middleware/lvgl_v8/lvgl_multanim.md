# LVGL v8 multanim

SiFli SDK provides a set of transition-animation wrappers for scene switching on LVGL v8, consisting of two parts:

- **multanim widget** (`lvsf_multanim.h`): wraps common transition animations—zoom, 3D flip, axial compression, mask fade, and more—into a single object. An application only needs to create an instance, set the animation type and the major/minor images, and then drive the animation by updating a progress value in the `[0, 1024]` range; it does not need to handle per-frame drawing itself.
- **switchanim framework** (`lvsf_switchanim.h`): an animation scheduling framework for switching between screens (pages). It manages the creation, playback, completion, and callbacks of enter/exit animations, and registers custom animations into a linker section through the `BUILTIN_ANIMATION` macro.

The same animation recurs repeatedly during scene switching (such as APP switching, TLV switching, and watch-face switching). This wrapper centralizes the repeated animation drawing logic, so applications can reuse it through a unified interface.

## Features

- Wraps multiple transition animations: Zoom, 3D flip (3D), reverse flip with zoom (Switch), half-page flip (Turn), axial compression (Scale), mask fade (Fade), and center-out opening (Open).
- Book, Shuttle, and Shutter animations are available only on 58x and later chips (they depend on GPU / VGLite).
- Animations are driven by a progress value in `[0, 1024]`. They can either be played automatically by LVGL animation (`lv_multanim_start_anim()`) or advanced frame by frame from touch/gesture input (`lv_multanim_set_process()`).
- Supports left and right fade mask resources, which can be loaded from a file path or constructed directly by the application as `lv_img_dsc_t`.
- The switchanim framework uniformly schedules page-switch animations through priority-based overriding and linker-section registration of built-in animations.

## Use Cases

- Left/right swipe switching between watch/home-app watch faces and APP lists.
- Page transitions for multi-list widgets such as TLV (title-list-view).
- Enter and exit animations when entering or leaving a secondary page.
- Interactive scenarios that require real-time preview of the transition following the finger drag position.

## Supported Boards

Refer to the `lvgl_v8_multanim` example:

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp
- General: boards from 55x onward (such as 58x, 56x, 52x).

```{note}
The 3D, Roll, Turn, and Scale animations depend on the GPU (`LV_USE_GPU` / EPIC); Book, Shuttle, and Shutter depend on VGLite (`USING_VGLITE`) and are available only on 58x and later chips. Whether a given animation is available depends on the build configuration and board capabilities.
```

## Configuration and Initialization

multanim is controlled by the `LVSF_USE_MULTANIM` switch, which is enabled under `LittlevGL2RTT -> SiFli extend` in `menuconfig`. The standard GUI initialization flow completes the switchanim framework initialization (`lv_switchanim_init()`) inside `gui_lib_init()`, so applications normally do not need to call it again.

```c
#include "lvsf.h"
#include "lvsf/gui_widgets/lvsf_multanim.h"
#include "lvsf/gui_widgets/lvsf_switchanim.h"
```

## API Reference

### multanim widget interface (`lvsf_multanim.h`)

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_obj_t *lv_multanim_create(lv_obj_t *parent)` | Create a multanim animation instance | `parent`: parent object; returns the created object pointer, or `NULL` on failure |
| `lv_multanim_type lv_multanim_set_type(lv_obj_t *multanim, lv_multanim_type type)` | Set the animation type; must be set before starting the animation | `multanim`: instance; `type`: animation type; returns the animation type before setting |
| `void lv_multanim_set_dir(lv_obj_t *multanim, lv_multanim_dir dir)` | Set the animation direction | `dir`: direction, one of `LV_MULTANIM_LEFT/RIGHT/TOP/BOTTOM`; supports bitwise-OR combination (e.g. `LV_MULTANIM_HOR`) |
| `void lv_multanim_set_major_img(lv_obj_t *multanim, lv_obj_t *major_img)` | Set the major animation image (required) | `major_img`: major image object pointer |
| `void lv_multanim_set_minor_img(lv_obj_t *multanim, lv_obj_t *minor_img)` | Set the minor animation image | `minor_img`: minor image object pointer |
| `void lv_multanim_set_range(lv_obj_t *multanim, int32_t range)` | Set the animation range | `range`: animation range value |
| `void lv_multanim_set_viewpoint(lv_obj_t *multanim, lv_point_t *start_v, lv_point_t *end_v)` | Set the start and end viewpoints of the animation | `start_v`/`end_v`: start/end viewpoint points |
| `void lv_multanim_set_zoom(lv_obj_t *multanim, lv_coord_t start_zoom, lv_coord_t zoom_end)` | Set the start and end zoom values | `start_zoom`/`zoom_end`: start/end zoom values |
| `void lv_multanim_set_process(lv_obj_t *multanim, int32_t process)` | Set the animation progress (core control interface) | `process`: progress value, range `[0, 1024]` |
| `void lv_multanim_set_mask(lv_obj_t *multanim, const lv_img_dsc_t *mask_l, const lv_img_dsc_t *mask_r)` | Set the left and right masks for mask-type animations | `mask_l`/`mask_r`: left/right fade mask descriptors; pass `NULL` when not needed |
| `int32_t lv_multanim_get_proc(lv_obj_t *multanim)` | Get the current animation progress | Returns the current progress value |
| `lv_obj_t *lv_multanim_get_major_img(lv_obj_t *multanim)` | Get the major animation image object | Returns the major image object pointer |
| `lv_obj_t *lv_multanim_get_minor_img(lv_obj_t *multanim)` | Get the minor animation image object | Returns the minor image object pointer |
| `lv_img_dsc_t *lv_multanim_create_mask(const void *src)` | Read a mask image descriptor by path | `src`: mask file/resource path; returns the mask descriptor pointer |
| `void lv_multanim_free_mask(lv_img_dsc_t *dsc)` | Free a mask descriptor created by `lv_multanim_create_mask()` | `dsc`: mask descriptor to free |
| `lv_multanim_type lv_multanim_get_type(lv_obj_t *multanim)` | Get the current animation type | Returns the current animation type |
| `void lv_multanim_start_anim(lv_obj_t *multanim, uint32_t period, int32_t start_pro, int32_t end_pro, lv_anim_ready_cb_t ready_cb)` | Start an automatically played animation | `period`: duration (ms); `start_pro`/`end_pro`: start/end progress `[0,1024]`; `ready_cb`: animation completion callback |

Animation type enumeration (`lv_multanim_type`): `LV_MULTANIM_NONE`, `LV_MULTANIM_ZOOM`, `LV_MULTANIM_3D`, `LV_MULTANIM_SWITCH`, `LV_MULTANIM_TURN`, `LV_MULTANIM_SCALE`, `LV_MULTANIM_FADE`, `LV_MULTANIM_OPEN`, `LV_MULTANIM_ROLL`, `LV_MULTANIM_DRAG`, `LV_MULTANIM_BOOK`, `LV_MULTANIM_SHUTTLE`, `LV_MULTANIM_SHUTTER`.

### switchanim framework interface (`lvsf_switchanim.h`)

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_obj_t *lv_baseanim_get_original_screen(lv_baseanim_t *baseanim)` | Get the original screen object corresponding to the animation | Returns the original screen object pointer |
| `lv_baseanim_para_t *lv_baseanim_get_para(lv_baseanim_t *baseanim)` | Get the animation parameters | Returns the parameter struct pointer |
| `lv_baseanim_type lv_baseanim_get_type(lv_baseanim_t *baseanim)` | Get the animation type (enter/exit) | Returns `LV_BASEANIM_ENTER_TYPE` / `LV_BASEANIM_EXIT_TYPE`, etc. |
| `void lv_switchanim_init()` | Initialize the animation list and load all animations | Called automatically by the standard startup flow |
| `void lv_switchanim_load()` | Load all animations | No return value |
| `void lv_switchanim_deinit()` | Deinitialize the animation list | No return value |
| `lv_baseanim_cb *lv_switchanim_register_anim(char *name, uint32_t major, lv_baseanim_progress_cb progress_cb)` | Register an animation into the list | Returns the registered animation callback descriptor |
| `void lv_switchanim_unregister_anim(uint32_t major)` | Remove an animation by major | No return value |
| `void lv_switchanim_overwrite(const lv_baseanim_para_t *enter_org, lv_baseanim_para_t *enter_anim, const lv_baseanim_para_t *exit_org, lv_baseanim_para_t *exit_anim, uint32_t flag)` | Override enter/exit animation parameters by priority | `flag`: forward or reverse animation flag |
| `lv_obj_t *lv_switchanim_create(lv_obj_t *parent, lv_obj_t *enter_screen, lv_obj_t *exit_screen, lv_baseanim_para_t *enter_para, lv_baseanim_para_t *exit_para)` | Create a switchanim instance | Returns the instance handle; attached to the root screen when `parent` is `NULL` |
| `void lv_switchanim_manual_run(lv_obj_t *switchanim, int32_t progress)` | Manually advance animation frames by progress percentage | Used for gesture-following interactive animations |
| `void lv_switchanim_manual_finish(lv_obj_t *switchanim, bool is_enter)` | Manually finish the animation | `is_enter`: whether to load the enter or exit screen after finishing |
| `void lv_switchanim_auto_run(lv_obj_t *switchanim, uint32_t period, uint32_t progress, bool reverse)` | Automatically play the animation | `period`: duration; `progress`: start progress; `reverse`: whether to play in reverse |
| `void lv_switchanim_set_finish_cb(lv_obj_t *switchanim, lv_switchanim_finish_cb finish_cb)` | Set the animation completion callback | No return value |
| `void lv_switchanim_set_path(lv_obj_t *switchanim, lv_baseanim_path path)` | Set the easing path for progress growth | `path`: `LINE`/`EASE_IN`/`EASE_OUT`/`EASE_IN_OUT` |
| `void lv_switchanim_set_def_anim(uint16_t anim_major, uint16_t anim_minor)` | Set the default animation | No return value |
| `lv_baseanim_cb *lv_switchanim_find_anim(uint32_t major)` | Find an animation descriptor by major | Returns the descriptor pointer |

Built-in animations are registered into the `switch_anim` linker section through the `BUILTIN_ANIMATION(anim_name, anim_major, anim_progress_cb)` macro.

## Typical Usage

Minimum usage flow: create an instance → set the type → set the major/minor images → start playback or update the progress frame by frame.

```c
#include "lvsf/gui_widgets/lvsf_multanim.h"

static lv_obj_t *multanim;

void demo_multanim(void)
{
    lv_obj_t *scr = lv_scr_act();

    /* 1. Create a multanim instance */
    multanim = lv_multanim_create(scr);
    lv_obj_set_size(multanim, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
    lv_obj_center(multanim);

    /* 2. Create the major/minor images */
    lv_obj_t *img1 = lv_img_create(multanim);
    lv_obj_t *img2 = lv_img_create(multanim);
    lv_obj_center(img1);
    lv_obj_center(img2);

    /* 3. Set the animation type, direction, and major/minor images */
    lv_multanim_set_type(multanim, LV_MULTANIM_3D);
    lv_multanim_set_dir(multanim, LV_MULTANIM_HOR);
    lv_multanim_set_major_img(multanim, img1);
    lv_multanim_set_minor_img(multanim, img2);
    lv_multanim_set_zoom(multanim, LV_IMG_ZOOM_NONE, LV_IMG_ZOOM_NONE >> 1);

    /* 4. Auto play: from progress 0 to 1024 over 1000 ms */
    lv_multanim_start_anim(multanim, 1000, 0, 1024, NULL);
}
```

When manually advancing along a gesture, map the finger offset to `[0, 1024]` and then call `lv_multanim_set_process()`:

```c
/* offset is the horizontal drag offset, hor_res is the screen width */
int32_t proc = (offset << 10) / hor_res;
lv_multanim_set_process(multanim, proc);
```

Register a custom 3D switch animation in the switchanim framework:

```c
static void turn3d_anim_progress(lv_baseanim_t *baseanim, lv_obj_t *anim_obj, int32_t progress)
{
    if (LV_BASEANIM_EXIT_TYPE == lv_baseanim_get_type(baseanim))
    {
        /* Create a multanim instance and set its type and major image */
        lv_multanim_set_process(switch_multanim, progress);
    }
    else
    {
        lv_multanim_set_minor_img(switch_multanim, anim_obj);
    }
}

BUILTIN_ANIMATION(turn3Danim, LV_SWITCHANIM_TURN_3D, turn3d_anim_progress);
```

## Demo

```{image} ../../../assets/lvgl_v8/multanim_all.gif
:alt: multanim all animation effects
```

```{image} ../../../assets/lvgl_v8/multanim_3d.gif
:alt: multanim 3D flip animation
```

```{image} ../../../assets/lvgl_v8/multanim_tlv.gif
:alt: multanim TLV tiled switching animation
```

```{image} ../../../assets/lvgl_v8/switch_anim_conf.png
:alt: switchanim animation configuration
```

## See Also

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- Source paths: `middleware/lvgl/lvsf/gui_widgets/lvsf_multanim.h`, `middleware/lvgl/lvsf/gui_widgets/lvsf_switchanim.h`
- Example path: `example/multimedia/lvgl/lvgl_v8_multanim`
