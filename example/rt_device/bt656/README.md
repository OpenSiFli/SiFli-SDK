# BT.656 例程（dpi-hdk-lb57e）

源码路径: example/rt_device/bt656

演示 `customer/peripherals/camera/bt656` 的通用 BT.656 接口（`bt656.h`）：
单帧（snapshot）、逐帧（frame）和连续流（stream）三种用法都建立在同一条
连续采集路径上，画面显示到 1024×600 DPI 屏。

## 用法

下面的小节只列必要步骤；SDK 配置、构建与烧录的完整步骤见
[SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)。

### 编译与运行

在已加载 SDK 环境的 shell 里执行：

```bash
cd example/rt_device/bt656/project
scons --board=dpi-hdk_lb57eybbn6_n16_qspi -j8
```

烧写到板子后，串口为 `uart1`、波特率 1000000（`CONFIG_CONSOLE_UART1=y`、
`CONFIG_RT_SERIAL_DEFAULT_BAUDRATE=1000000`）。

## 支持的开发板

- `dpi-hdk_lb57eybbn6_n16_qspi`（板卡代号 dpi-hdk-lb57e，SF32LB57x HCPU）

### 硬件需求

- ARK7128 视频解码模块，接在本板固定的 I2C1（SCL `PA23` / SDA `PA22`）与
  DCMI PINMAP0 引脚上，引脚表见
  [`customer/peripherals/camera/bt656/README.md`](../../../customer/peripherals/camera/bt656/README.md)；
- 模拟视频源：CVBS（NTSC/PAL）或 AHD 720P（25/30 fps），接模块 VIN2；
- 1024×600 DPI 屏（本板自带）。

### 软件需求

- 已加载 SDK 环境（`export.ps1` / `export.sh`），`scons` 与 `sdk.py` 可执行；
- 可选：LVGL v8（`CONFIG_PKG_USING_LITTLEVGL2RTT=y`）及其应用配置，
  只有 `camera_lvgl` 预览需要，见“LVGL 集成示例”。

### 配置项目

| 项 | 位置 |
|---|---|
| 视频输入格式（NTSC / PAL / AHD 25 / AHD 30） | `menuconfig` → `Board Peripherals` → `Camera` → `BT.656 Camera` → `Decoder chip` → `ARK7128` → `Video input format` |
| LVGL 预览（可选） | 顶层选项 `BT.656 + LVGL preview demo`（依赖 `CONFIG_PKG_USING_LITTLEVGL2RTT=y`） |
| LCD 型号 / 背光 | `project/proj.conf` |
| I2C / DCMI 引脚 | 板级 `bsp_pinmux.c` 配置（`BSP_USING_BT656` 块） |

```bash
cd example/rt_device/bt656/project
sdk.py menuconfig --board=dpi-hdk_lb57eybbn6_n16_qspi
```

## 示例输出

`main()` 打印横幅和用法，采集不会自动开始（下面只摘录这两部分，前面还有
其它组件的启动日志）：

```
===== BT.656 camera demo (dpi-hdk-lb57e) =====
usage:
  camera snapshot [loops] [timeout_ms]  capture & show N frames (default 1, 1000 ms)
  camera frame [interval_ms]            continuous frame mode (default 0 ms)
  camera stream [interval_ms]           continuous stream mode (default 0 ms)
  camera init                           initialize decoder
  camera stop                           stop continuous display
  camera cpu                            print CPU usage (last ~2 s window)
  camera info                           print camera status
msh >
```

`camera init` 之后确认解码器（`camera info` 会打印读到的芯片 ID）。下面只摘录
`[camera]` 开头的行。驱动自身还会打印 `[I/bt656]`、`[I/ark7128.*]` 这类
`LOG_I/LOG_W` 日志：默认配置 `RT_DEBUG=y`（`rtos/rtthread/src/Kconfig`）时
它们经 rtdbg 直接输出；若在 menuconfig 关闭 `RT_DEBUG`，这些行会被编译掉，
只剩 `[camera]` 的 `rt_kprintf` 输出。打开 `CONFIG_RT_USING_ULOG=y` 后改由
ulog 输出。

```
msh /> camera init
[camera] ready: CVBS NTSC, signal locked
```

### 命令与交互

相机功能由一条 `camera` 命令提供，LVGL 预览另有 `camera_lvgl`：

| 命令 | 说明 |
|---|---|
| `camera snapshot [loops] [timeout_ms]` | 起流取 N 帧再停流，默认 1 帧 / 1000 ms |
| `camera frame [interval_ms]` | 连续预览（与 stream 同一条采集路径） |
| `camera stream [interval_ms]` | 连续预览（与 frame 同一条采集路径） |
| `camera init` | 初始化采集层，打印格式与锁定状态 |
| `camera stop` | 停止连续显示 |
| `camera cpu` | 打印 CPU 占用率（最近约 2 s 窗口） |
| `camera info` | 打印格式、锁定状态、预览状态、CPU 信息 |
| `camera help` | 打印用法（不带子命令时同样打印） |
| `camera_lvgl start` / `stop` | LVGL 图像控件预览（需在 menuconfig 里打开 `BT.656 + LVGL preview demo`） |

`interval_ms` 是两帧之间的最小间隔，`0`（默认）表示全速。`frame` 与 `stream`
目前完全等价：都走同一条连续采集路径，只有 `camera info` 打印的模式名不同。

示例：

```
msh /> camera snapshot             # 采一帧
msh /> camera snapshot 10          # 起流取 10 帧
msh /> camera frame                # 连续预览，约 27~30 fps
msh /> camera stream 100           # 连续预览，每帧至少间隔 100 ms
msh /> camera info
[camera] format : CVBS NTSC (720x240, stride 1440, interlaced field)
[camera] signal : locked (chip id 0x7128)
[camera] stats  : captured=120, dropped=0, errors=0
[camera] preview: running, mode=frame, captured=120, failed=0
[camera] cpu    : 45.2% (last ~2 s window)
msh /> camera stop
```

运行中每 2 秒打印一行统计（fps 为预览启动以来的累计平均值）：

```
[camera] 29.1 fps, 34 ms/frame, fail=0, cpu 45.2%
```

## 示例解析（可选）

### 上电行为

1. `bt656_init()`：调用 ARK7128 芯片适配层的 `init()`（配置 I2C1、
   读 ID、下发当前格式寄存器表），再初始化 DMA、链接 DCMI 句柄并打开
   DCMI 中断（`HAL_DCMI_Init()` 在 `bt656_start()` 里执行）；
2. 显示不会自动开始，需要执行 `camera stream` / `camera frame` /
   `camera snapshot`（内部调用 `bt656_start()`，一次完成重锁相、DCMI 初始化
   和开流）。

> 驱动在 2026-11 做了两层拆分：应用只包含 `bt656.h`，使用
> `bt656_start/get/release/stop`；具体解码芯片通过 `bt656_decoder_ops`
> 注册，换芯片不需要改应用。前后对照见
> [`customer/peripherals/camera/bt656/docs/接口改造说明.md`](../../../customer/peripherals/camera/bt656/docs/接口改造说明.md)。

### 三种命令的实际行为

| | snapshot | frame | stream |
|---|---|---|---|
| 触发 | `camera snapshot` | `camera frame` | `camera stream` |
| DCMI | `bt656_start()` 一次 | 同左 | 同左 |
| 完成判据 | CVBS: DMA 字节数；AHD: DCMI FRAME | 同左 | 同左 |
| 结束 | 取够帧数后 `bt656_stop()` | `camera stop` | `camera stop` |
| 适用 | 抓图/调试 | 连续预览 | 连续预览 |

三种命令都走同一条采集路径：`bt656_get()` 借一帧 → 渲染 →
`bt656_release()` 归还。缓冲由驱动固定分配，显示慢时直接丢弃新帧（计入
`dropped`），内存不会堆积，因此预览只需要一个线程。

> 连续采集存在已知的固定横向偏移问题（复位后首次开流、摄像头掉电重启、
> 反复启停偶发），当前驱动没有做补偿。

### 显示链路

```
DCMI 采集（YUV422/UYVY）
  └─ 预览线程：bt656_get() 借一帧
       └─ YUV→RGB565（EPIC 硬件 / CPU 回退）
            └─ show_fb（1024×600，视频 1:1 居中）
                 └─ 只回写图像所在行 cache
                      └─ 面板由 LCDC/PTC 连续自扫描 show_fb
       └─ bt656_release() 归还
```

- YUV→RGB565 默认走 EPIC 硬件（`drv_epic_blend`），失败自动回退 CPU；
- 这块屏是 RAMLESS DPI + line-buffer bypass：`draw_rect` 只能整屏，且 PTC
  状态机会一直扫描第一次下发的缓冲。因此只在首次把扫描起点指到 `show_fb`，
  之后每帧只更新画布 + 回写 cache；
- 必须先 `RTGRAPHIC_CTRL_SET_BUF_FORMAT(RGB565)`，否则图层格式断言。

### LVGL 集成示例（可选）

`src/bt656_lvgl.c` 演示把摄像头画面作为一张图片嵌入 LVGL 界面：

```c
/* lv_timer 回调，在 LVGL 线程里执行 */
if (bt656_get(&f, 0) == RT_EOK) {           /* 非阻塞 */
    dsc.header.cf = LV_IMG_CF_YUV422_PACKED_UYVY;
    dsc.data      = f.data;
    dsc.data_size = (uint32_t)f.stride * f.height;
    lv_img_set_src(cam_img, &dsc);
    if (prev_held_valid) {                   /* 归还上一帧 */
        bt656_release(&prev_held);
    }
    prev_held = f;
    prev_held_valid = RT_TRUE;
}
```

LVGL v8 的 YUV422 图片格式由 SDK 的 LVGL GPU 驱动交给 EPIC 合成，
不需要 CPU 转换，也不需要 `show_fb`。（示例依赖 LVGL v8 的
`lv_img_dsc_t` / `lv_timer_create` API，v9 不适用。）

- 每帧要先调用 `lv_img_cache_invalidate_src(&dsc)`：缓冲地址不变、内容变了，
  不失效会被 LVGL 的图片缓存命中，显示不更新；
- 不能在 `lv_img_set_src()` 之后立即 `bt656_release()`：EPIC 是异步 DMA，
  要等下一帧更新时才归还上一帧。

启用步骤（LVGL 需要完整的 LVGL 应用配置，只打开包开关是不够的）：

```conf
# project/proj.conf —— 以 example/lvgl_v8_ttf/project/proj.conf 为参考
CONFIG_RT_USING_MEMHEAP=y
CONFIG_PKG_USING_LITTLEVGL2RTT=y
# CONFIG_LV_USING_EXT_RESOURCE_MANAGER is not set
# SDK 的 LVSF 默认主题会引用这些字号（1024x600 时）
CONFIG_LV_FONT_MONTSERRAT_16=y
CONFIG_LV_FONT_MONTSERRAT_20=y
CONFIG_LV_FONT_MONTSERRAT_24=y
CONFIG_LV_FONT_MONTSERRAT_28=y
CONFIG_LV_FONT_MONTSERRAT_36=y
CONFIG_LV_FONT_DEFAULT_MONTSERRAT_16=y
```

> 实测：只加 `CONFIG_PKG_USING_LITTLEVGL2RTT=y` 时 SDK 的 LVGL middleware
> 会先因 `RT_USING_MEMHEAP` 未开、`lang_pack.h` 缺失而失败；补齐后再因
> `lv_font_montserrat_*` 未声明失败。上面的组合覆盖了这三类问题。

然后打开例程自己的选项。它来自 `project/Kconfig.proj`，依赖
`CONFIG_PKG_USING_LITTLEVGL2RTT=y`，所以要在打开 LVGL 之后才会出现：

```bash
cd example/rt_device/bt656/project
sdk.py menuconfig --board=dpi-hdk_lb57eybbn6_n16_qspi
# 顶层选项：BT.656 + LVGL preview demo
```

```bash
msh /> camera_lvgl start    # 全屏居中显示
msh /> camera_lvgl stop
```

限制：

- 只支持 CVBS（packed UYVY，`stride == 2*width`）；AHD 缓冲带定时码，
  `lv_img_dsc_t` 没有 stride 字段，暂不支持；
- 本示例自己创建 LVGL 宿主线程，工程里已有 LVGL 线程时请把 timer 那段搬过去；
- 不要与 `camera stream` 同时运行（两者独占 `lcd` 设备）。

### CPU 占用率测量

参照 `example/system/cpu_usage_profiler` 接入 SDK 自带的 CPU Usage Profiler，
`project/proj.conf` 已默认打开：

```conf
CONFIG_USING_CPU_USAGE_PROFILER=y
CONFIG_CPU_USAGE_METRICS_ENABLED=y
CONFIG_CPU_USAGE_METRICS_PRINT_DIRECTLY=y
CONFIG_CPU_USAGE_METRICS_PRINT_PERIOD=10
```

- `camera cpu`：打印最近约 2 s 的 CPU 占用率；
- `camera info`：末尾附带 CPU 占用率；
- `camera frame` / `camera stream` 的运行统计行会附带 `cpu xx.x%`；
- 启用 ULOG 后，系统每 10 s 打印一次各线程负载（`CPU Usage Metrics`）。
  **注意**：该表格和 SDK 的 `cpu` 命令通过 `LOG_I` 输出，需要
  `CONFIG_RT_USING_ULOG=y`；当前 `proj.conf` 没有启用 ULOG，因此这两项
  在本工程里没有输出，而 `camera cpu` / `camera info` / 统计行使用
  `rt_kprintf`，不受影响。启用 ULOG 后的示例（数值仅示例）：

```
============================
CPU Usage:   36.20/  50.01 (  72.39%)
========================================================
thread      run_time(s)    load(%)
--------------------------------------------------------
[ark_prev]:   24.10   48.19%
[tidle   ]:   13.81   27.61%
[tshell  ]:    0.05    0.10%
```

- SDK 还提供 shell 命令：`cpu`（立即打印统计）、`cpu_prof_reset`（清零）、
  `cpu_prof_deinit` / `cpu_prof_init`（关/开采集钩子）；其中 `cpu` 的输出依赖
  ULOG（原因同上）；
- Metrics 依赖 `thread->user_data` 记录线程运行时间，因此要求
  `RT_USING_PTHREADS` 关闭（当前工程满足）；
- `cpu_get_usage()` 返回最近一个约 2 s 统计区间的占用率，不是瞬时采样；
- 不想测量时把上面 4 行配置去掉再重编译即可，代码会自动走
  "未启用 profiler" 的打印分支。

### 内存（PSRAM）

| 用途 | CVBS NTSC | CVBS PAL | AHD 720P |
|---|---|---|---|
| `ark7128_stream_buf`（3 块 / AHD 2 块） | 3 × 414,720 B | 3 × 414,720 B | 2 × 1,843,200 B |
| `show_fb`（显示画布） | 1,228,800 B | 同左 | 同左 |

CVBS 的流缓冲每块按最大的 CVBS 格式（PAL 1440×288）预留：NTSC 一场只有
345,600 B 有效数据，缓冲仍是 414,720 B。CVBS 的流缓冲加画布共约 2.4 MB，
都在 PSRAM1。AHD 的流缓冲使用 `frambuf` 段，放到 PSRAM2（链接脚本的
`l2_non_ret_bss_frambuf*` 规则；本板需确认第二片 PSRAM 可用）。

## 异常诊断

> 下面两条 `[W/bt656]` 是驱动的 `LOG_W`：默认 `RT_DEBUG=y` 时可见，关闭
> `RT_DEBUG` 后会被编译掉。不依赖日志时，用 `camera info` 的 `signal` 与
> `stats`（`errors` 是否增长）以及预览线程的 `[camera] N captures failed
> (ret=...)` 判断：`ret=-2`（`-RT_ETIMEOUT`）表示一直取不到帧，
> `ret=-8`（`-RT_EIO`）表示采集已停止。

- `camera info` 打印 `format not configured`：还没有成功执行 `bt656_init()`
  （先执行 `camera init`；它失败时会打印 `decoder init failed`）；`chip id 0x0000`
  表示 I2C 读不到芯片，检查模块供电与 I2C1（`PA22`/`PA23`）接线。
- `signal` 为 `no lock`：确认模拟源接在 VIN2、制式与 `Video input format` 一致。
- 连续取帧超时时，驱动按信号状态分别处理：
  - 有信号但收不到帧：打印
    `[W/bt656] no frame in N ms (remaining x/y), restart capture`，复位 DCMI
    重新起流；
  - 无信号：打印 `[W/bt656] no frame in N ms, no signal, waiting for source`
    （约每秒最多一次），不复位 DCMI，等信号恢复。

  预览线程连续 50 次失败后打印
  `[camera] 50 captures failed (ret=-2), check signal`（`-2` 即
  `-RT_ETIMEOUT`），之后每 50 次再打印一次，线程不退出。

### 已知限制

- 面板自扫描与 CPU 写缓冲之间没有双缓冲翻转，快速运动画面可能撕裂；
- PCLK 信号完整性不足时会出现偶发丢码（颜色错乱、漏码），属于硬件问题。

## 参考文档

- 驱动与板级连接说明：[`customer/peripherals/camera/bt656/README.md`](../../../customer/peripherals/camera/bt656/README.md)
- 接口演进与缓冲所有权：[`customer/peripherals/camera/bt656/docs/接口改造说明.md`](../../../customer/peripherals/camera/bt656/docs/接口改造说明.md)
- CPU 占用率：[`example/system/cpu_usage_profiler`](../system/cpu_usage_profiler/README.md)
