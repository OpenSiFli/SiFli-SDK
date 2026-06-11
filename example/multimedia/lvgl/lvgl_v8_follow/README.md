# LVGL v8 Follow 示例

源码路径: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_follow`

## 概述

本例程演示 `lvsf_follow` 控件：一个**物理"重力"图标菜单**。图标排成同心圆环，并在重力作用下移动、相互避让。常用作智能手表的应用启动器。板子上由 g-sensor 提供重力方向；PC 模拟器里控件自带一条交互路径，把鼠标点击位置转成重力向量。

要点：
- 通过 `lv_follow_get_cfg_param()` 拿到配置结构体，逐项设置物理参数(重力、摩擦、碰撞类型等)和环布局。
- **`cfg->custom_align` 必须为 `true`**，这样下面的逐环布局才生效：`offset_r[i]` 是第 i 环半径、`target_r[i]` 是该环图标半径、`gap_angle[i]` 决定每环图标数(`360 / gap_angle`)。设为 `false` 时控件会按自身宽度自动算图标尺寸。
- `lv_follow_set_item_cb()` 设两个回调：item 回调为每个元素返回一个图标对象，delete 回调在元素销毁时调用。
- 加完元素后调用 `lv_follow_on_start()` + `lv_follow_enter_order_status()` 让图标排进环形队列。

## 用法（关键 API）

```c
lv_obj_t *follow = lv_follow_create(parent);
lv_obj_set_size(follow, LV_PCT(100), 360);

lv_follow_cfg_t *cfg = lv_follow_get_cfg_param(follow);
cfg->collision_type = FOLLOW_TYPE_STANDARDS;
cfg->gravity = 0.01f;  cfg->friction = 0.2f;  cfg->icon_r = 23;  cfg->v_max = 3;
cfg->target_r[0] = 23; cfg->target_r[1] = 19; cfg->target_r[2] = 14;  /* 各环图标半径 */
cfg->offset_r[0] = 0;  cfg->offset_r[1] = 70; cfg->offset_r[2] = 125; /* 各环半径 */
cfg->gap_angle[0] = 360; cfg->gap_angle[1] = 60; cfg->gap_angle[2] = 40; /* 360/gap=每环个数 */
cfg->custom_align = true;   /* 必须: 采用上面的逐环布局 */

lv_follow_set_item_cb(follow, item_cb, delete_cb);  /* item_cb 返回每个图标对象 */
for (int i = 0; i < 8; i++)
    lv_follow_add_item_info(follow, 0, NULL);        /* 加入元素 */
lv_follow_on_start(follow);
lv_follow_enter_order_status(follow);                /* 排成环形队列 */
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

例程启动后屏幕上有 8 个彩色圆点排成重力菜单环(中心一个大的 + 一圈 6 个 + 外侧 1 个)。模拟器交互：
- **点中心**：图标重新排回环形队列(聚拢)。
- **点边缘**：图标朝点击方向被"重力"拉过去。
- **长按拖动**：移动某个图标。

板子上则由 g-sensor(需 app 读取并喂给控件)驱动重力。

## 异常诊断

如有任何技术疑问，请在 GitHub 上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
