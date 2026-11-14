# SF32LB57 SMP example

[English](README_EN.md)

源码路径: example/multicore/smp

本例程演示 SF32LB57 HCPU 与 ACPU 运行同一份应用镜像、共用 RT-Thread 3.1.3 内核的 SMP（对称多处理）适配。它使用现有线程、信号量、互斥量和堆 API，检查跨核数据交接、线程迁移、浮点/栈上下文和动态线程回收，可作为编写显式共享工作线程的起点。

该适配仍属实验功能。默认新线程绑定 HCPU，只有明确调整亲和性的线程才会运行到 ACPU；完整边界见 [SMP 适配说明](../../../rtos/rtthread/SMP_SF32LB57_PROTOTYPE.md)。

## 用法

先按 [SiFli-SDK 快速入门](../../../docs/source/zh_CN/quickstart/index.md) 安装 SDK 环境与 GCC 工具链。在 SDK 根目录启动 PowerShell 并执行：

```powershell
. ./export.ps1
cd example/multicore/smp/project
```

本例程使用标准 `project/` 工程结构，只构建一份 HCPU 应用 `main.bin`，由两核共用；不需要单独编译或烧录 `acpu.bin`。

## 支持的开发板

- `dpi-hdk_lb57gyd7n6_epd_hcpu`：SF32LB57GYD7N6，板级配置见 [开发板说明](../../../customer/boards/dpi-hdk_lb57gyd7n6_epd/doc/index.md)。

### 硬件需求

需要上述开发板、电源和连接 HCPU 调试串口的 USB 线/串口适配器。板载 16 MiB PSRAM 用于共享堆检查；本例程不需要连接或操作显示屏。运行后无需按键或其他交互。

### 配置项目

[proj.conf](project/proj.conf) 已启用 SMP、共享 Flash 执行和 memheap，并关闭与本例程不兼容的 AMP ACPU 服务、PM、软件定时器、信号及模块等功能。

| 配置项 | 本例程默认值 | 说明 |
| --- | --- | --- |
| `RT_USING_SMP` | `y` | 启用 SF32LB57 HCPU/ACPU SMP。 |
| `RT_SMP_SHARED_XIP` | `y` | 两核通过共同可访问的 MPI2 Flash 地址执行共享代码。 |
| `RT_SMP_PSRAM_NONCACHE` | `y` | 使用 Kconfig 默认值，两核共享 PSRAM 不缓存。 |
| `RT_SMP_PSRAM_WT_SYNC` | `n` | 可选 PSRAM 写穿透同步；启用前需关闭 PSRAM 非缓存选项。 |
| `RT_SMP_SRAM_WT_SYNC` | `n` | 可选 ACPU SRAM 数据/栈写穿透同步。 |
| `RT_USING_MEMPOOL` | `y` | SDK 默认启用固定块内存池；例程据此编译跨核内存池检查。 |
| `RT_USING_HOOK` | `y` | SDK 默认启用 hook；例程据此检查调度切换回调参数。 |
| `EXAMPLE_SMP_ROUNDS` | `32` | 动态线程创建轮数，范围为 1–10000。 |

需要调整配置时，在 `project/` 目录运行：

```powershell
sdk.py menuconfig --board=dpi-hdk_lb57gyd7n6_epd_hcpu
```

共享代码与常量由 SDK 链接脚本放置。`RT_SMP_TEXT` 和 `RT_SMP_RODATA` 仅标记指定函数或常量，其被调用函数、字符串池、编译器/库辅助代码也必须位于两核可达地址。它们不会自动完成整个应用的 SMP 适配。

### 可选 WT 配置

任意 WT 模式都会选择内部 `RT_SMP_CACHE_SYNC`，均要求 `CONFIG_PSRAM_CACHE_WB=n`，包括仅启用 SRAM WT 的情况。板级 PSRAM 配置继承的 `PSRAM_CACHE_WB` 默认值为 `y`；未关闭时会触发编译期错误 `WT synchronization cannot coexist with writable write-back mappings.`。

选择以下一种策略，在 `project/proj.conf` 中替换对应配置项，保留其余项目配置并完整重编。两个 WT 选项可以单独或同时启用，无需手动设置 `RT_SMP_CACHE_SYNC`。

仅 PSRAM WT：

```ini
CONFIG_RT_USING_SMP=y
CONFIG_RT_SMP_SHARED_XIP=y
CONFIG_RT_SMP_PSRAM_NONCACHE=n
CONFIG_RT_SMP_PSRAM_WT_SYNC=y
CONFIG_RT_SMP_SRAM_WT_SYNC=n
CONFIG_PSRAM_CACHE_WB=n
```

仅 SRAM WT，PSRAM 保持非缓存：

```ini
CONFIG_RT_USING_SMP=y
CONFIG_RT_SMP_SHARED_XIP=y
CONFIG_RT_SMP_PSRAM_NONCACHE=y
CONFIG_RT_SMP_PSRAM_WT_SYNC=n
CONFIG_RT_SMP_SRAM_WT_SYNC=y
CONFIG_PSRAM_CACHE_WB=n
```

PSRAM 与 SRAM 同时使用 WT：

```ini
CONFIG_RT_USING_SMP=y
CONFIG_RT_SMP_SHARED_XIP=y
CONFIG_RT_SMP_PSRAM_NONCACHE=n
CONFIG_RT_SMP_PSRAM_WT_SYNC=y
CONFIG_RT_SMP_SRAM_WT_SYNC=y
CONFIG_PSRAM_CACHE_WB=n
```

### 编译和烧录

在已初始化环境的 `project/` 目录运行：

```powershell
scons --board=dpi-hdk_lb57gyd7n6_epd_hcpu -j8
```

产物和下载脚本位于 `project/build_dpi-hdk_lb57gyd7n6_epd_hcpu/`。修改 SMP 或缓存配置后应完整重编，不要混用旧 RTOS 对象。

按 [SDK 编译与下载说明](../../../docs/source/zh_CN/quickstart/build.md) 使用生成的下载脚本。在同一 `project/` 目录中，Windows 串口下载命令为：

```powershell
.\build_dpi-hdk_lb57gyd7n6_epd_hcpu\uart_download.bat
```

按脚本提示输入开发板的实际串口号并复位。下载后打开 HCPU 控制台观察输出，波特率使用当前板级控制台配置。

### 自动化运行

配置好连接开发板的 lab-agent 后，在已初始化环境的 SDK 根目录运行：

```powershell
sdk.py test run -T example/multicore/smp -p dpi-hdk_lb57gyd7n6_epd_hcpu
```

环境准备及测试报告说明见 [自动化测试指南](../../../docs/source/zh_CN/app_note/autotest_guide.md)。控制台用例按默认 32 轮校验；修改 `EXAMPLE_SMP_ROUNDS` 后，需同步调整用例中的预期总数。

## 示例输出

以下记录来自当前例程在 SF32LB57GYD7N6 上的实际运行，使用 GCC `link.lds` 链接路径、共享 XIP、非缓存 PSRAM，且未启用 SRAM WT；此处仅规范换行并省略 shell 提示符：

```text
SMP_BEGIN
SMP_RESULT rounds=32 threads=64 migrations=2048 errors=0 heap_restored=1 cpus=3 ticks=354
SMP_PASS
```

同一固件复位运行三次均通过，`ticks` 分别为 354、352、354。每次均完成 32 轮、64 个线程和 2048 次迁移，未报告错误，堆统计恢复。`cpus=3` 是表示两核均被访问的位掩码。

`ticks` 是 RT-Thread tick 计数的差值，受时钟、缓存策略和调度负载影响，不是固定验收值；上面的数值仅对应默认配置。

任一检查失败时打印 `SMP_FAIL` 及失败阶段；关闭 SMP 时打印 `SMP_DISABLED`。

例程的输出均来自 HCPU，ACPU 工作线程不打印日志。

### 验证范围

当前源码在 SF32LB57GYD7N6 上完成了以下 10 个配置的板端验证：

| 链接路径/配置 | 覆盖项 | 结果 |
| --- | --- | --- |
| GCC `link.lds`，SMP 开启 | 默认非缓存、PSRAM+SRAM 双 WT、关闭 XIP 后共享代码运行于 SRAM | 3 项通过 |
| GCC `link.jinja2`，SMP 开启 | 默认非缓存、仅 PSRAM WT、仅 SRAM WT、双 WT | 4 项通过 |
| 两种链接路径，SMP 关闭 | 单核配置回归 | 2 项均按预期输出 `SMP_DISABLED` |
| GCC `link.lds`，SLAB 精简配置 | SLAB 分配器下运行例程 | 1 项通过，条件见下 |

SLAB 通过项仅使用测试时设置 `CONFIG_RT_USING_DFS=n`、`CONFIG_RT_USING_FINSH=n`、`CONFIG_RT_USING_AUDIO=n` 的精简配置。原板级完整配置因内存不足输出 `SMP_FAIL thread_create`，未计入通过项；项目默认配置未因此修改。

SDK 原生 [smp.console 用例](testcase.yaml) 通过。Jinja 双 WT 配置另完成 1000 轮压力运行，共 2000 个工作线程、64000 次迁移，`errors=0`、`heap_restored=1`、`ticks=90492`，含启动约 91 秒。

hook 反向固件输出 `SMP_FAIL scheduler_hook`，控制台匹配器将其判为失败。专项主机侧检查通过 24 个测试、153 个子测试。上述结果不构成长期可靠性验证。

## 示例解析

[src/main.c](src/main.c) 的执行流程如下：

1. HCPU 初始化 IPC 对象及配置启用的 PSRAM 堆、固定块内存池和调度 hook，再调用 `rt_smp_start_secondary()` 启动 ACPU。
2. 每轮动态创建两个线程，分别绑定 HCPU 和 ACPU。两个线程通过信号量交接带校验模式的负载，并使用互斥量保护计数。
3. 每个线程切换到另一核 32 次，同时校验栈数据和浮点累计值。默认 32 轮共创建 64 个线程、执行 2048 次迁移。
4. 一个核分配并填充内存，另一个核通过信号量接收、校验并释放。轮号从 0 开始，奇数轮使用 16 KiB PSRAM 例程堆；开启 `RT_USING_MEMPOOL` 时，部分偶数轮使用 2 KiB 存储区的固定块内存池，其余轮使用 SRAM 堆。只有启用 PSRAM、共享 XIP 及有效 PSRAM 共享策略时才编译 PSRAM 路径；关闭相应功能时使用 SRAM 堆路径。
5. 工作线程返回退出，由 HCPU `reaper` 回收。主线程等待清理通知及堆统计恢复，再进入下一轮；结束时检查 PSRAM 堆余额、内存池块全部归还和总计数。
6. 开启 `RT_USING_HOOK` 时，调度切换回调检查收到的两个线程指针均非 `NULL`。该检查不统计回调次数。

## 异常诊断

- `SMP_DISABLED`：确认最终构建配置启用了 `RT_USING_SMP`。
- `SMP_FAIL secondary_start=...`：检查共享代码放置、MPU 属性及是否误启用了 AMP ACPU 服务。返回值含义见 [SMP 适配说明](../../../rtos/rtthread/SMP_SF32LB57_PROTOTYPE.md)。
- `SMP_FAIL completion_timeout` 或只看到 `SMP_BEGIN`：检查 ACPU 启动与可达代码；调试器可查看 `ap_fault_cfsr`、`ap_fault_hfsr`、`ap_fault_pc` 和 `ap_fault_lr`。
- `SMP_FAIL round=...`、`psram_leak`、`pool_leak` 或 `counters`：检查数据交接、线程回收和堆/内存池统计；保留完整日志及最终配置，不要仅依据线程创建成功判断运行正常。
- `SMP_FAIL scheduler_hook`：调度切换回调收到空线程指针，需检查切换回调的调用条件。

本例程不覆盖 DMA 所有权交接、外设驱动并发、Flash 擦写与对端 XIP 同时运行、无锁共享数据或长期可靠性验证。

如有技术疑问，请在 GitHub 上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)。

## 参考文档

- [SMP 适配说明](../../../rtos/rtthread/SMP_SF32LB57_PROTOTYPE.md)
- [SMP 扩展头文件](../../../rtos/rtthread/include/rtsmp.h)
- [项目配置](project/Kconfig.proj) 与 [默认配置](project/proj.conf)
