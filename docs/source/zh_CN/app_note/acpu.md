# ACPU 使用指南

## 1. ACPU 概述

SF32LB57x 与 SF32LB58x 系列采用三核架构：HCPU（高性能计算）、ACPU 以及 LCPU（低功耗任务，如 BLE 和传感器数据处理）。完整的架构说明请参考 [HAL 索引](../hal/index.md)。

ACPU 在不同芯片系列中的角色不同：

| 芯片系列 | ACPU 角色 |
|----------|-----------|
| SF32LB57x | 音频和图形处理 |
| SF32LB58x | Audio DSP 处理 |

ACPU 运行独立的固件镜像（基于 RT-Thread），作为 HCPU 工程的子工程构建，启动时由二级 boot 加载到 ACPU RAM 中。HCPU 与 ACPU 之间通过基于 Mailbox 的核间 IPC Queue 通信。

### 1.1 典型应用场景

- **音频编解码卸载**：opus 编解码器运行在 ACPU 上，例如 `example/multimedia/audio/opus`。
- **语音 3A 处理**：ACPU 任务 `ACPU_TASK_audio_3a_open/uplink/downlink/close` 完成 3A 处理。
- **图形渲染**：EPIC 渲染列表在 ACPU 上执行（`ACPU_TASK_epic_rl`），例如 `example/rt_device/gpu/render_list_mode`。
- **自定义计算任务**：用户自定义任务由 `acpu_main()` 分发执行，例如 `example/multicore/acpu_task`。

### 1.2 HCPU 与 ACPU 的交互方式

- HCPU 通过 `acpu_run_task()` 发起任务，该调用为阻塞式，ACPU 完成任务后返回。
- ACPU 在 `acpu_main()` 中接收任务并执行，通过 `acpu_send_result()` 回传结果。
- 底层传输为 IPC Queue（双向队列 `sys_get_ha_ipc_queue` / `sys_get_ah_ipc_queue`），配合信号量同步。

API 细节见 {ref}`第 4 章 <acpu-api-usage>`。

## 2. 使能与配置

支持 ACPU 的工程由两部分组成：HCPU 工程和 ACPU 子工程（见 {ref}`第 3 章 <acpu-project-build>`）。两个工程各有独立的配置，均通过 menuconfig 修改。

### 2.1 在 HCPU 工程中使能 ACPU

在 `project/hcpu` 目录下，使用板名启动 menuconfig：

```bash
sdk.py menuconfig --board=ec-lb587
```

在 menuconfig 界面中：

1. 打开 **SiFli Built-in Components → ACPU Control Framework**（`USING_ACPU_CTRL_FWK`）。该选项会自动选中 **On-chip Peripherals → ACPU**（`BSP_USING_ACPU`）和 IPC Queue（`USING_IPC_QUEUE`）。
2. （可选）在 **SiFli Built-in Components → ACPU Control Framework** 下打开 **Enable Caller**（`ACPU_CALLER_ENABLED`，HCPU 侧默认开启），允许 ACPU 代码调用 HCPU 服务（内存分配、控制台输出）。

按 `D` 键将最小配置保存到 `proj.conf`。

说明：

- `BSP_USING_ACPU` 仅在 SF32LB57x 和 SF32LB58x 芯片上可用（`customer/boards/Kconfig_drv`）。
- `--board` 不带核后缀时默认按 HCPU 配置：`ec-lb587` 与 `ec-lb587_hcpu` 等价。

### 2.2 配置 ACPU 工程

ACPU 子工程有独立的配置。在 `project/acpu` 目录下，使用带 `_acpu` 后缀的板名启动 menuconfig（使 menuconfig 使用 ACPU 核的板级配置）：

```bash
sdk.py menuconfig --board=ec-lb587_acpu
```

打开 **SiFli Built-in Components → ACPU Control Framework**（`USING_ACPU_CTRL_FWK`，IPC Queue 会自动选中）。

### 2.3 功能相关选项

| 功能 | HCPU 工程（`project/hcpu`） | ACPU 工程（`project/acpu`） |
|------|------------------------------|-------------------------------|
| EPIC 双核渲染列表模式 | **On-chip Peripherals → EPIC → Using epic render list API → EPIC Render List API Mode**，选择 *Using epic render list API in dual core HCPU mode*（`DRV_EPIC_NEW_API_DUAL_CORE_HCPU`） | 相同路径，选择 *Using epic render list API in dual core ACPU mode*（`DRV_EPIC_NEW_API_DUAL_CORE_ACPU`） |
| ACPU 上的 opus 编解码 | – | **Third-Party Components → libopus**（`PKG_LIB_OPUS`） |
| ACPU 调用 HCPU 服务（内存分配、控制台输出） | 见 2.1 第 2 步 | – |

EPIC 双核选项会自动选择 `USING_ACPU_CTRL_FWK`（`customer/boards/Kconfig_drv`）。**EPIC Render List API Mode** 的选择项依赖核，因此 HCPU 与 ACPU 两侧需分别在各自工程的 menuconfig 会话（带 `_hcpu` / `_acpu` 后缀）中选择对应的双核模式。完整的 EPIC 使能 ACPU 工程配置可参考 `example/acpu/project/proj.conf`。

(acpu-project-build)=
## 3. 工程创建、编译与烧录

ACPU 运行独立的固件镜像。在 SDK 中，ACPU 工程作为 HCPU 工程的子工程管理，并与 HCPU 工程一起编译。

### 3.1 工程结构

以 `example/multicore/acpu_task` 为例：

```text
example/multicore/acpu_task/
├── project/hcpu/      # HCPU 工程
├── project/acpu/      # ACPU 子工程
├── src/acpu/          # ACPU 应用代码
└── src/hcpu/          # HCPU 应用代码
```

### 3.2 添加 ACPU 子工程

在 HCPU 工程的 `SConstruct.py` 中调用 `AddChildProj`，并指定 `core="ACPU"` 添加 ACPU 子工程（`example/multicore/acpu_task/project/hcpu/SConstruct.py`）：

```python
acpu_proj_path = os.path.join(proj_path, '../acpu')
acpu_proj_name = 'acpu'
AddChildProj(acpu_proj_name, acpu_proj_path, False, core="ACPU")
```

### 3.3 编译与烧录

在 `project/hcpu` 目录下，使用板名编译（仅基于 SF32LB57x/SF32LB58x 的开发板支持 ACPU，例如 `ec-lb583`、`ec-lb587`）：

```bash
scons --board=ec-lb587
```

编译产物生成在 `build_ec-lb587` 目录下，使用以下命令烧录镜像：

```bash
build_ec-lb587/download.bat
```

板名带 `_hcpu` 后缀时可以省略，因为默认编译 HCPU 核（例如 `ec-lb587` 与 `ec-lb587_hcpu` 均使用 HCPU 配置）。

### 3.4 ACPU 镜像加载

ACPU 镜像存放在 Flash 中，启动时由二级 boot 加载到 ACPU RAM：

1. Flash 表中将 ACPU 镜像映射到 `DFU_FLASH_HCPU_EXT2` 分区。例如，生成的 `ftab.c` 将镜像放在 Flash 地址 `0x69100000`，并设置 `xip_base = 0x20200000`，对应 ACPU 指令空间的 0 地址：

```c
RT_USED const struct sec_configuration sec_config =
{
    .magic = SEC_CONFIG_MAGIC,
    .ftab[DFU_FLASH_HCPU_EXT2] = {.base = 0x69100000, .size = 0x0007C000,  .xip_base = 0x20200000, .flags = 0},
    .imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_HCPU_EXT2)] = {.length = 0x00000AE4, .blksize = 512, .flags = DFU_FLAG_AUTO},
};
```

2. 启动时，二级 boot 将 ACPU 镜像从 Flash 拷贝到 ACPU RAM（`example/boot_loader/project/sf32lb58x_v2/board/main.c` 中的 `boot_images` 函数）：

```c
/* load extended img if present, such as ACPU img */
if (g_sec_config->imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_HCPU_EXT2)].length != FLASH_UNINIT_32)
{
    dfu_boot_img_in_flash(DFU_FLASH_HCPU_EXT2);
}
```

> `example/acpu` 是供其他工程引用的通用 ACPU 工程模板，修改它会影响到所有引用该模板的工程。

(acpu-api-usage)=
## 4. 通信与 API 使用

完整 API 定义在 `middleware/include/acpu_ctrl.h`（Doxygen 参考：[acpu_ctrl API](../api/middleware/acpu_ctrl.md)）。两个核之间的传输基于 Mailbox 之上的 IPC Queue（见 [IPC Queue](../middleware/ipc_queue.md) 和 [Mailbox](../hal/mailbox.md)）。

### 4.1 HCPU 侧 API

| 函数 | 说明 |
|------|------|
| `acpu_init()` | 初始化框架：创建 HCPU→ACPU 的 IPC Queue 并给 ACPU 上电。通过 `INIT_PRE_APP_EXPORT` 导出，在系统初始化时自动运行，无需用户调用。 |
| `acpu_run_task(task_name, param, param_size, error_code)` | 向 ACPU 发送任务并阻塞等待任务完成。返回 ACPU 的结果缓冲指针，并将错误码写入 `error_code`（`ACPU_ERR_OK` = 0，`ACPU_ERR_ASSERT` = 0xff）。 |
| `acpu_power_on()` / `acpu_power_off()` | 控制 ACPU 电源域。上电通常由 `acpu_init()` 完成；调试时也可使用控制台命令 `acpu on` / `acpu off`。 |

`acpu_run_task()` 通过阻塞信号量实现：调用线程挂起，直到 ACPU 返回响应；若等待 1 秒超时，SDK 会触发断言（见 `middleware/acpu_ctrl/acpu_ctrl.c`）。

### 4.2 ACPU 侧 API

| 函数 | 说明 |
|------|------|
| `acpu_main(task_name, param)` | ACPU 入口函数。默认弱实现会分发预定义任务（opus、音频 3A、read/write、EPIC 渲染列表）。自定义任务时覆盖该函数（见 {ref}`第 5 章 <acpu-examples>`）。 |
| `acpu_send_result(err_code, ret_value)` | 将任务结果回传给 HCPU，使 HCPU 挂起的 `acpu_run_task()` 解除阻塞。 |
| `acpu_printf()` | 将字符串转发到 HCPU 控制台（需要 HCPU 侧开启 `ACPU_CALLER_ENABLED`）。 |
| `acpu_call_hcpu_malloc()` / `acpu_call_hcpu_free()` | 在 ACPU 代码中向 HCPU 申请/释放内存（需要 HCPU 侧开启 `ACPU_CALLER_ENABLED`）。 |
| `acpu_send_assert(file, line)` | 向 HCPU 发送断言通知。 |

### 4.3 任务 ID

任务 ID 定义在 `middleware/include/acpu_ctrl.h`：

| 宏 | 值 | 说明 |
|----|----|------|
| `ACPU_TASK_INVALID` | 0 | 无效任务 |
| `ACPU_TASK_0` | 1 | 用户自定义任务 0 |
| `ACPU_TASK_1` | 2 | 用户自定义任务 1 |
| `ACPU_TASK_opus_encoder_init` | 3 | opus 编码器初始化 |
| `ACPU_TASK_opus_encoder_ctl` | 4 | opus 编码器控制 |
| `ACPU_TASK_opus_encode` | 5 | opus 编码 |
| `ACPU_TASK_opus_decoder_init` | 6 | opus 解码器初始化 |
| `ACPU_TASK_opus_decoder_ctl` | 7 | opus 解码器控制 |
| `ACPU_TASK_opus_decode` | 8 | opus 解码 |
| `ACPU_TASK_audio_3a_open` | 9 | 音频 3A 打开 |
| `ACPU_TASK_audio_3a_close` | 10 | 音频 3A 关闭 |
| `ACPU_TASK_audio_3a_downlink` | 11 | 音频 3A 下行 |
| `ACPU_TASK_audio_3a_uplink` | 12 | 音频 3A 上行 |
| `ACPU_TASK_read` | 13 | 读 |
| `ACPU_TASK_write` | 14 | 写 |
| `ACPU_TASK_epic_rl` | 15 | EPIC 渲染列表 |
| `ACPU_TASK_COUNT` | 16 | 任务数量 |

ACPU 回调 HCPU 时使用的 HCPU 侧任务 ID：

| 宏 | 值 | 说明 |
|----|----|------|
| `HCPU_TASK_INVALID` | 0 | 无效任务 |
| `HCPU_TASK_MALLOC` | 1 | 在 HCPU 上分配内存 |
| `HCPU_TASK_FREE` | 2 | 在 HCPU 上释放内存 |
| `HCPU_TASK_PRINTF` | 3 | 输出到 HCPU 控制台 |

### 4.4 注意事项与限制

- 任务消息（`acpu_ctrl_ipc_msg_t`）通过 IPC Queue 传递参数缓冲指针和结果值。按 `acpu_ctrl.h` 中 `acpu_run_task()` 的文档说明，参数大小受 `ACPU_TASK_INPUT_PARAM_SIZE` 限制，结果长度受 `ACPU_TASK_OUTPUT_VAL_SIZE` 限制。
- 不要在中断上下文中调用 `acpu_run_task()`：该调用是阻塞式的，会挂起调用线程。
- 等待 ACPU 结果期间，框架会请求 `PM_SLEEP_MODE_IDLE`，避免系统在调用期间进入更深睡眠状态。
- 如果 ACPU 返回 `ACPU_ERR_ASSERT`，HCPU 会打印 `acpu assert:` 及 ACPU 上报的 `<file> <line>`，并触发自身断言。
- ACPU 镜像必须正确烧录；否则 ACPU 不会响应，`acpu_run_task()` 会超时并断言。

(acpu-examples)=
## 5. 例程讲解

### 5.1 `acpu_task`：HCPU 向 ACPU 下发任务

`example/multicore/acpu_task` 是最小的 ACPU 例程。HCPU 注册了控制台命令 `run_acpu`（`src/hcpu/main.c`），命令中调用 `acpu_run_task()`：

```c
static int run_acpu(int argc, char *argv[])
{
    char *s;

    if (argc < 2)
    {
        rt_kprintf("wrong argument\n");
        return -1;
    }

    /* config ACPU to run specified task */
    s = acpu_run_task(atoi(argv[1]) + ACPU_TASK_0, NULL, 0, NULL);

    LOG_I(s);

    return 0;
}
MSH_CMD_EXPORT(run_acpu, "run acpu")
```

该调用会阻塞，直到 ACPU 完成任务。ACPU 入口 `acpu_main()` 在 `middleware/acpu_ctrl/acpu_main.c` 中有默认弱实现，可处理预定义任务 ID：任务 0/1 返回 `"task_0"` / `"task_1"`，未知任务 ID 返回 `"unknown task"`。HCPU 随后打印返回的字符串：

```text
msh />run_acpu 0
run_acpu 0
[I/main] task_0
msh />run_acpu 1
[I/main] task_1
msh />run_acpu 2
[I/main] unknown task
```

要实现自定义任务，请在 `src/acpu/main.c` 中定义 `acpu_main(task_name, param)`（例程中的 `__acpu_main()` 空函数仅为占位模板；对 `acpu_main` 的强定义会覆盖框架的默认弱实现），根据 `task_name` 分发任务，执行后用 `acpu_send_result()` 回传结果。

### 5.2 `opus`：ACPU 上的音频编解码

`example/multimedia/audio/opus` 将 opus 编码/解码器运行在 ACPU 上。ACPU 工程开启 `CONFIG_PKG_LIB_OPUS=y`，HCPU 通过 `ACPU_TASK_opus_*` 任务 ID 委派编解码操作，例如（`src/main.c`）：

```c
#if defined(USING_ACPU_CTRL_FWK)
    uint8_t error_code = 1;
    opus_encode_ctl_arg_t arg;
    arg.st = encoder;
    arg.id = 0;
    acpu_run_task(ACPU_TASK_opus_encoder_ctl, &arg, sizeof(arg), &error_code);
    RT_ASSERT(error_code == 0);
#else
    opus_encoder_ctl(encoder, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_10_MS));
    ...
#endif
```

`ACPU_TASK_opus_encode`、`ACPU_TASK_opus_decode` 等任务的使用方式相同。

### 5.3 `4pdm`：ACPU 上的音频采集与 3A

`example/multimedia/audio/4pdm` 采集 4 路 PDM 音频。当 HCPU 工程设置 `CONFIG_ANYKA_RUN_IN_ACPU=y` 时，3A 处理通过 `ACPU_TASK_audio_3a_*` 任务运行在 ACPU 上，ACPU 子工程在 `project/hcpu/SConstruct` 中添加。

### 5.4 `render_list_mode`：ACPU 上的图形渲染

`example/rt_device/gpu/render_list_mode` 将 EPIC 渲染列表执行放到 ACPU。HCPU 工程设置 `CONFIG_DRV_EPIC_NEW_API_DUAL_CORE_HCPU=y`，并复用通用 ACPU 模板 `example/acpu/project` 作为子工程（该模板开启 `CONFIG_DRV_EPIC_NEW_API_DUAL_CORE_ACPU=y`）。渲染列表任务以 `ACPU_TASK_epic_rl` 下发到 ACPU。

## 6. 调试与常见问题

### 6.1 ACPU 断言

ACPU 触发断言时，会调用 `acpu_send_assert(file, line)` 将位置上报给 HCPU。HCPU 打印：

```text
acpu assert:<file> <line>
```

并触发自身的断言（`RT_ASSERT(0)`）。请根据上报的 ACPU 源码位置排查问题。

### 6.2 ACPU 日志

ACPU 代码可以使用 `acpu_printf()` 将字符串转发到 HCPU 控制台，这要求 HCPU 侧开启 `ACPU_CALLER_ENABLED`（HCPU 侧默认开启）。控制台命令 `acpu on` / `acpu off` 用于调试时打开/关闭 ACPU 电源域。

### 6.3 内存

- 二级 boot 会将 ACPU 镜像拷贝到 ACPU RAM 的 `0x20200000`（ACPU 指令空间的 0 地址）。
- 板级内存映射位于 `customer/boards/<board>/{hcpu,lcpu,acpu}/`（例如 `custom_mem_map.h`）。

### 6.4 常见问题

| 现象 | 可能原因 |
|------|---------|
| `acpu_run_task()` 超时并断言 | ACPU 镜像未烧录，或 HCPU 工程未使能 `USING_ACPU_CTRL_FWK`/`BSP_USING_ACPU` |
| menuconfig 中没有 ACPU 选项 | 开发板基于 SF32LB52x/55x/56x，没有 ACPU 核 |
| `acpu assert: ...` | ACPU 代码在报错的文件:行位置失败 |
| EPIC 渲染异常或卡死 | HCPU 和 ACPU 两侧未同时使能 `DRV_EPIC_NEW_API_DUAL_CORE_*` |

## 7. 参考链接

- [acpu_ctrl API](../api/middleware/acpu_ctrl.md)
- [IPC Queue API](../api/middleware/inter-processor_queue_library.md)
- [Mailbox HAL](../hal/mailbox.md)
- 双核开发说明（SDK 路径：`docs/source/zh_CN/app_note/dualcore.md`，SF32LB55x HCPU/LCPU）
- 内存使用指南（SDK 路径：`docs/source/zh_CN/app_note/memory_usage.md`，SF32LB55x）
- `acpu_task` 例程（SDK 路径：`example/multicore/acpu_task`）
