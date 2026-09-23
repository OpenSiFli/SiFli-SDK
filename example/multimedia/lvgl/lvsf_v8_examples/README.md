# LVSF v8 自定义控件示例合集

源码路径：`example/multimedia/lvgl/lvsf_v8_examples`

## 介绍

本工程演示 SiFli 在 LVGL v8 上的自定义控件（lvsf widgets）例程集中到**同一个工程**里：每个控件的演示代码是 `src/widgets/<name>/` 下的一个独立子文件夹，**一次编译只演示一个控件**，想换一个控件看效果/接口用法时，只需在配置里把要演示的例程切换过去、重新编译烧录即可。


## 例程一览

| 配置选项 `LV_WIDGET_DEMO_*` | 子文件夹 | 演示控件 | 入口函数 | 说明 |
|---|---|---|---|---|
| `BASECHART` | `widgets/basechart/` | `lvsf_basechart` | `demo_basechart_init()` | 图表控件，绘制一条/多条折线或柱状数据序列 |
| `BASELABEL` | `widgets/baselabel/` | `lvsf_baselabel` | `demo_baselabel_init()` | 内建数据刷新通路的标签，文本可跟随数据源周期更新 |
| `FOLLOW` | `widgets/follow/` | `lvsf_follow` | `demo_follow_init()` | 物理“重力”图标菜单，图标成环排布并随重力移动避让 |
| `IMGARRAY` | `widgets/imgarray/` | `lvsf_imgarray` | `demo_imgarray_init()` | 用字形图片拼出数值，类似数码管/翻牌显示 |
| `IMGBAR` | `widgets/imgbar/` | `lvsf_imgbar` | `demo_imgbar_init()` | 用前景图片按数值裁切填充的图片进度条 |
| `MULROLLER` | `widgets/mulroller/` | `lvsf_mulroller` | `demo_mulroller_init()` | 多功能滚轮/转盘选择器，回调按需提供内容，居中吸附 |
| `MULTANIM` | `widgets/multanim/` | `lvsf_multanim` | `lv_example_multanim()` | 多重过渡动画效果（动画图片） |
| `MULTLIST` | `widgets/multlist/` | `lvsf_multlist` | `gui_app_run(DEMO_MULTLIST_MAIN_ID)` | 多功能列表，基于 gui_app_fwk 框架的多页面综合示例 |
| `MULTROLLER` | `widgets/multroller/` | `lvsf_multroller` | `demo_multroller_init()` | 简单声明式滚轮，`'\n'` 分隔选项字符串即可，循环吸附 |
| `MULTSLIDER` | `widgets/multslider/` | `lvsf_multslider` | `demo_multslider_init()` | 圆形手柄上显示读数（或自定义短文本）的滑块 |
| `SECTOR` | `widgets/sector/` | `lvsf_sector` | `demo_sector_init()` | 扇形（角度）遮罩，把图片按数值揭示成饼形 |
| `SELECT` | `widgets/select/` | `lvsf_select` | `demo_select_init()` | 选择列表，每行显示选中/未选中图标，支持单选 |
| `TIMELINE` | `widgets/timeline/` | `lvsf_timeline` | `demo_timeline_init()` | 通用动画时间线编排引擎，把时间区间映射到数值区间 |

每个子文件夹下都有该例程自己的 `README.md` / `README_EN.md`，包含控件概述、关键 API、典型用法和运行说明。要查看某个控件的接口调用方式，直接打开对应子文件夹的 `demo_<name>.c` 和 README 即可。

## 快速开始：切换要演示的控件

核心思路：**选择控件例程（menuconfig 或改 proj.conf 二选一）→ 重新编译 → 烧录运行**。下面以从默认的 basechart 切换到 multlist 为例。

### 方式一：menuconfig 图形菜单（推荐）

```bash
# 1) 先在 SDK 根目录激活环境（每个终端执行一次）
cd /Volumes/TB/gerrit_sdk/SiFli-SDK        # 换成你的 SDK 路径
. export.sh        # 注意是“点 + 空格 + export.sh”，source 方式执行

# 2) 进入本工程的 project 目录，打开配置菜单
cd example/multimedia/lvgl/lvsf_v8_examples/project
scons --board=sf32lb52-lcd_n16r8 --menuconfig
```

在菜单中依次进入：

```
LVSF v8 widget examples  --->
    Select widget example  --->
        (X) basechart - base chart widget
        ( ) baselabel - base label widget
        ( ) ...
        ( ) multlist - multi-function list (gui_app framework)
        ( ) ...
```

操作按键：

- 方向键 `↑/↓` 移动光标，`Enter` 进入 `--->` 子菜单；
- 进入 **Select widget example** 后，这是一个单选（choice）列表，用 `↑/↓` 移到想演示的控件上，按 `空格`（或 `Enter`）把选中标记 `(X)` 切到该行（同一时间只有一个为选中）；
- 选好后按 `Esc` 逐级返回，退出时弹出 “Save configuration?” 选择 **Yes** 保存。

保存后重新编译、烧录：

```bash
scons --board=sf32lb52-lcd_n16r8 -j8
```

### 方式二：直接改 proj.conf（脚本化/更直观）

用文本编辑器打开 `project/proj.conf`，文件末尾就是当前选中的例程：

```ini
# 默认运行的控件例程（choice，只能选一个）；切换方法见 README「切换例程」
CONFIG_LV_WIDGET_DEMO_BASECHART=y
```

把它改成目标例程（把旧的一行删掉或留作注释，新的一行设为 `y`；choice 同一时间只能有一个为 `y`）。例如切换到 multlist：

```ini
# CONFIG_LV_WIDGET_DEMO_BASECHART=y
CONFIG_LV_WIDGET_DEMO_MULTLIST=y
```

可选的宏名就是上表“配置选项”列：`LV_WIDGET_DEMO_BASECHART / BASELABEL / FOLLOW / IMGARRAY / IMGBAR / MULROLLER / MULTANIM / MULTLIST / MULTROLLER / MULTSLIDER / SECTOR / SELECT / TIMELINE`。

改完保存，重新编译即可（无需手动删缓存，scons 检测到 proj.conf 变化会自动重新生成配置）：

```bash
scons --board=sf32lb52-lcd_n16r8 -j8
```

> 提示：切换例程后**务必重新编译并重新烧录**，固件里只会包含当前选中那一个控件的演示代码。

## 编译和烧录

板子工程在 `project` 目录下，通过指定 board 编译（把 `--board=` 换成你的板子）：

```bash
cd project
scons --board=sf32lb52-lcd_n16r8 -j8
```

烧录通过 build 目录下生成的 `download.bat`（或 `uart_download.bat`）进行；SF32LB52x/SF32LB56x 系列会额外生成 `uart_download.bat`，执行后输入下载 UART 端口号即可。

## 目录结构

```
lvsf_v8_examples/
├── README.md / README_EN.md        # 本文件
├── assets/                         # 图片资源（仅 multanim / multlist 需要）
│   └── SConscript                  #   按所选例程只打包对应图片
├── project/                        # 构建工程（SConstruct / proj.conf / Kconfig ...）
└── src/
    ├── Kconfig                     # 控件例程选择菜单（choice）
    ├── SConscript                  # 只编译所选例程子目录的源码
    ├── main.c                      # 统一入口，按 Kconfig 宏调用对应例程入口
    └── widgets/                    # 每个控件例程一个子文件夹
        ├── basechart/              #   demo_xxx.c/h + README
        ├── ...
        └── multlist/               #   gui_app_fwk 多页面示例（含多个 .c）
```

## 说明

- 一次只编译并运行一个例程；未选中例程的源码和图片不会参与编译，不占用固件空间。
- `multlist` 例程基于应用框架（gui_app_fwk），入口与其他例程不同，已在 `main.c` 中单独处理。
- 需要文件系统资源（Lottie JSON、视频、字体文件）的例程（`lvgl_v8_lottie`、`lvgl_v8_media`、`lvgl_v8_freetype`）未合入本工程，仍作为独立工程存在。
- 控件 API 的详细说明见 SDK 文档《LVGL v8 自定义控件》章节。
