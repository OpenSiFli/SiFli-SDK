# External LVGL Example

源码路径: example/multimedia/lvgl/lvgl_external

本例程演示如何直接使用 **SDK 之外的上游 LVGL 源码**（`lvgl/lvgl`，已包含上游 SiFli EPIC 绘制后端），由工程自己完成构建、显示/输入移植和配置。SDK 的 LVGL 集成（`PKG_USING_LITTLEVGL2RTT`）保持关闭，不参与编译，也因此**不需要为外部模式修改任何 SDK 源码或 Python 构建脚本**。

阅读本例程可以了解：

- 如何把上游 LVGL 作为独立源码树接入 SiFli-SDK 的构建（源码收集、对象文件重定位）；
- 如何用上游 RT-Thread 设备（`lcd`、`touch`）实现 `lv_display` / `lv_indev` 的注册；
- 为什么这类工程应使用 LVGL 原生的 `lv_conf.h` 配置，而不是 SDK 的 Kconfig 生成宏。

## 用法

先按 [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html) 初始化工具链。

### 支持的开发板

- sf32lb58-lcd_a128r32n1_qspi：已完成编译、下载和实机验证（CO5300 390x450，上游 EPIC + partial 渲染，标签与上游 benchmark 均正常显示）。
- sf32lb52-lcd_n16r8：已完成编译、链接验证，尚未实机验证。
- sf32lb52-lchspi-ulp：尚未重新验证。

### 硬件需求

对应开发板、LCD/触摸屏和下载连接。

### LVGL 源码位置

例程以 LVGL 上游 `release/v9.6` 分支为集成目标。该分支（测试时检出为 `v9.6.0-24-gd8a4e1dfb`）已用本例程完成编译、下载和实机集成测试：标签和上游 benchmark 均正常显示。其它分支/版本未经验证——9.6 之前的上游没有 SiFli EPIC 后端，9.6 的 Kconfig 又与 SiFli 配置工具冲突（见下文），换版本时需要同步检查 `port/lv_conf.h`。

例程通过 `project/SConscript` 中的 `LVGL_ROOT` 引用上游 LVGL。检出通常位于 SDK 同级目录，脚本从 `SIFLI_SDK` 向上最多找三层，所以普通检出（`<parent>/lvgl`）和多一层目录的 git worktree 都能定位；其它布局可在环境里用 `LVGL_ROOT` 直接指定：

```python
LVGL_ROOT = find_lvgl_checkout(SIFLI_SDK)   # 或在环境里 export LVGL_ROOT=...
```

路径只出现在工程仓库里，SDK 不需要任何"外部 LVGL 路径"配置项。

### 配置工程

```bash
source "./export.sh"
cd "example/multimedia/lvgl/lvgl_external/project"
sdk.py menuconfig --board=sf32lb52-lcd_n16r8
```

`menuconfig` 里只配置 SDK 和工程自身的开关，**LVGL 的选项不在其中**（见下一节）。

## 为什么用 `lv_conf.h` 而不是 Kconfig 生成宏

LVGL 支持两种配置方式：

| | Kconfig 模式 | `lv_conf.h` 模式 |
| --- | --- | --- |
| 配置来源 | 平台 Kconfig 生成的 `CONFIG_LV_*` / `LV_*` 宏 | 工程里的 `lv_conf.h` |
| 典型使用者 | Zephyr、NuttX、SDK 内置 LVGL | 上游默认方式 |
| 与 LVGL 版本的对应 | Kconfig 树需与 LVGL 版本同步 | 配置文件与源码树一起维护 |

SDK 内置的 `lvgl_v9` 使用 Kconfig 模式，但**上游 9.6 的 Kconfig 不能照搬**。原因是 SiFli 的 Kconfig 工具把符号写入 `rtconfig.h` 时不加 `CONFIG_` 前缀，而上游 9.6 的 Kconfig 中有大量符号与 LVGL 的 C 标识符同名：

```kconfig
# 上游 Kconfig（include/lvgl/config/lv_conf_kconfig.h 对应的生成源）
choice LV_COLOR_FORMAT_DEFAULT
    config LV_COLOR_FORMAT_RGB565
        bool "RGB565"
```

```c
/* LVGL 头文件 lv_color.h */
typedef enum {
    LV_COLOR_FORMAT_RGB565 = 0x12,
    ...
} lv_color_format_t;
```

一旦把上游 Kconfig 加载进配置树，`rtconfig.h` 会生成：

```c
#define LV_COLOR_FORMAT_RGB565 1
```

`rtconfig.h` 会通过 `rtthread.h` 进入所有编译单元，于是枚举声明被替换成 `1 = 0x12`，编译直接失败。`LV_OS_NONE`、`LV_TXT_ENC_*` 等符号也有同类风险。

内置 `lvgl_v9` 没有这个问题，是因为它随 SDK 保存的 Kconfig（9.2/9.4 时代）使用 `LV_COLOR_DEPTH_16/24/32`，从未定义 `LV_COLOR_FORMAT_*`；这是上游 9.6 改名后才出现的新冲突。

因此本例程选择 LVGL 的原生配置机制：

- `port/lv_conf.h` 提供全部 LVGL 配置，构建时全局定义 `LV_CONF_INCLUDE_SIMPLE`，`lv_conf_internal.h` 会直接包含它；
- `lv_conf.h` 只 `#include "rtconfig.h"` 来读取**工程自己的**开关（如 `EXAMPLE_LVGL_BENCHMARK`），因为配置树里没有任何 LVGL 符号，不会引入同名宏；
- 规则：`lv_conf.h` 里可以**引用** LVGL 的标识符，但不要**定义**与 LVGL 标识符同名的宏。例如下面这种写法是安全的：

```c
#define LV_COLOR_FORMAT_DEFAULT LV_COLOR_FORMAT_RGB565
```

`LV_COLOR_FORMAT_RGB565` 只作为宏体出现，使用 `LV_COLOR_FORMAT_DEFAULT` 时才展开为枚举值。

### 关键配置项

`port/lv_conf.h`：

| 配置项 | 作用 |
| --- | --- |
| `LV_COLOR_FORMAT_DEFAULT` | 默认颜色格式，本示例为 RGB565，需与 LCD 的 `bits_per_pixel` 一致。 |
| `LV_USE_OS` | 设为 `LV_OS_NONE`：LVGL 由应用线程驱动，不引入第二套 OSAL。 |
| `LV_USE_STDLIB_MALLOC` | 使用 RT-Thread 的 `rt_malloc`。 |
| `LV_USE_STDLIB_STRING/SPRINTF` | 使用 CLIB：当前 RT-Thread 移植没有提供 `rt_strcpy` 等包装。 |
| `LV_USE_SIFLI_EPIC` | 启用上游自带的 SiFli EPIC 绘制后端。 |
| `LV_DRAW_BUF_ALIGN` | 绘制缓冲对齐：缓冲的起始地址和**分配尺寸**都要按这个值对齐，匹配 LCD/EPIC 的 DMA 要求（见“显示与输入”）。 |
| `LV_USE_DEMO_BENCHMARK` | 由 `EXAMPLE_LVGL_BENCHMARK` 控制，开启 benchmark 及其依赖字体、sysmon。 |

`project/proj.conf` / Kconfig：

| 配置项 | 作用 |
| --- | --- |
| `CONFIG_PKG_USING_LITTLEVGL2RTT` | 必须保持关闭，SDK 不编译任何 LVGL 集成代码。 |
| `CONFIG_PKG_USING_LVGL` | 由上游 SConscript 使用，编译外部 LVGL 源码。 |
| `CONFIG_PKG_LVGL_USING_DEMOS` | 编译上游 demos，benchmark 依赖它。 |
| `CONFIG_EXAMPLE_LVGL_BENCHMARK` | 运行上游 benchmark；关闭后显示一个标签。 |
| `CONFIG_LV_EXT_FB_TWO_PARTIAL` | 使用两块 partial 绘制缓冲。 |
| `CONFIG_LV_EXT_FB_LINE_NUM` | 每块绘制缓冲的行数。 |
| `CONFIG_BSP_USING_EPIC` | 必须关闭，避免 SDK EPIC 驱动与上游 EPIC 同时管理硬件和中断。 |
| `CONFIG_BSP_USING_PM` | 本示例关闭；上游 EPIC 尚未接入 SDK 休眠/唤醒协调。 |

## 示例输出

串口输出外部 LVGL 的版本，屏幕运行上游 benchmark：

```
LVGL 9.6.0
```

关闭 `CONFIG_EXAMPLE_LVGL_BENCHMARK` 后，屏幕显示 `LVGL external source example` 标签。

## 示例解析

### 构建

`project/SConscript` 调用上游根 `SConscript`（转发到 `env_support/rt-thread/SConscript`）收集 `lvgl/src/**/*.c`，并通过三点工程侧适配：

1. 用 `VariantDir` + 重包装 `DefineGroup` 把对象文件重定位到 `build_<board>/lvgl/source/`，保持 LVGL 检出目录干净；
2. 跳过上游的 `LVGL-port` 组：该文件整体在 `#ifdef __RTTHREAD__` 下，SDK 未定义该宏，且本例程自己驱动 `lv_timer_handler()`；
3. 补上 `demos/lv_demos.c`：上游 RT-Thread 脚本只遍历 `demos/` 的子目录，会漏掉这个文件。

### 显示与输入

- `port/lv_port_disp.c`：打开 `lcd` 设备，创建 `lv_display`，使用两块 100 行的 partial 缓冲；缓冲大小按 `LV_ROUND_UP(PORT_BUF_SIZE, LV_DRAW_BUF_ALIGN)` 分配、数组用 `ALIGN(LV_DRAW_BUF_ALIGN)`——`lv_draw_buf_reshape()` 判断能否重整形时用的是对齐后的尺寸，按 `stride × 行数` 精确分配会让整条带的 reshape 失败；flush 时用 `set_window` + `draw_rect_async` 发送，传输完成回调里调用 `lv_display_flush_ready()`。
- `port/lv_port_indev.c`：打开 `touch` 设备并注册 `LV_INDEV_TYPE_POINTER` 输入设备。
- `src/main.c`：`lv_init()` → 注册 tick → 初始化显示/输入 → 创建 UI → 循环 `lv_timer_handler()`。

## 异常诊断

- **`LVGL checkout not found`**：`project/SConscript` 在 SDK 检出旁（含向上三层）没找到 `lvgl/SConscript`；把环境变量 `LVGL_ROOT` 指向实际检出目录。
- **`rt_strcpy` 未定义**：使用了 `LV_STDLIB_RTTHREAD` 的 string/sprintf；本 SDK 的 RT-Thread 没有提供该包装，改为 `LV_STDLIB_CLIB`。
- **`undefined reference to lv_demo_args_init`**：demos 根目录的 `lv_demos.c` 未参与编译，检查构建脚本第 3 点适配。
- **枚举声明语法错误**：把上游 Kconfig 加载进了配置树，`rtconfig.h` 里出现了与 LVGL 标识符同名的宏。改用 `lv_conf.h` 配置。
- **画面不刷新**：确认 `BSP_USING_EPIC=n`，以及 LCD/触摸设备名分别为 `lcd`、`touch`。
- **PARTIAL 下跑 benchmark 死机：EPIC 断言 `total_width >= width`（`bf0_hal_epic.c`），或软件 `mem manage fault`（`lv_draw_sw_blend_color_to_rgb565`）**：绘制缓冲按 `stride × 行数` 精确分配，没有按 `LV_DRAW_BUF_ALIGN` 取整。`lv_draw_buf_reshape()` 计算尺寸时用 `LV_ROUND_UP(stride * h, LV_DRAW_BUF_ALIGN)`，缓冲略小会让整条带的 reshape 失败，header 保留在上一次的小区域，而 `buf_area` 已推进到整条带：EPIC 用 header 的 stride 当 `total_width` 触发断言，软件用 `buf_area` 算目的指针后越界写 NULL。按 `PORT_BUF_ALLOC_SIZE = LV_ROUND_UP(PORT_BUF_SIZE, LV_DRAW_BUF_ALIGN)` 分配、数组用 `ALIGN(LV_DRAW_BUF_ALIGN)`。FULL/DIRECT 模式不会暴露这个问题，因为初始 header 就是整屏。
- **`Assertion failed at function:output_lock` / `fatal error on thread: main`**：上游 EPIC 后端在完成中断里按 WARN 打日志，而 `LV_USE_OS=LV_OS_NONE` 时该中断没有调用 `rt_interrupt_enter()`，ulog 于是在"线程上下文"里对渲染线程已持有的 mutex 递归加锁。本例程的 `lv_log_print()` 丢弃异常上下文（`SCB->ICSR` 的 `VECTACTIVE` 非 0）里的日志来规避；根治应在上游 EPIC 后端把其中断登记给 OS。

## 参考文档

- [LVGL 集成文档](https://docs.lvgl.io/master/integration/index.html)
- [LVGL `lv_conf.h` 模板](https://github.com/lvgl/lvgl/blob/master/lv_conf_template.h)
- SDK 内置版本例程：../lvgl_v9_demos
