# BT.656 视频输入驱动（ARK7128 / SF32LB57x）

本目录提供两层：

- **通用 BT.656 层**（`bt656.c` / `bt656.h`）：DCMI 配置、DMA、多缓冲借还、
  cache 维护、超时恢复。**应用只依赖这一层**，不出现任何解码芯片名字。
- **ARK7128 芯片适配层**（`ark7128.c` / `ark7128_cfg.c` / `ark7128_i2c.c`）：
  I2C 分页寄存器、寄存器表、格式几何、模拟输入通道、引脚，通过
  `bt656_decoder_ops` 注册到通用层。以后换一颗 BT.656 解码芯片，只需要
  新写一个 ops 实现，应用与 `bt656.c` 都不用改。

驱动只输出原始 YUV422(UYVY)，不做像素格式转换：显示侧由 EPIC 硬件完成
YUV422 → RGB565，EPIC 不可用或失败时回退 CPU（例程见
[`example/rt_device/bt656`](../../../../example/rt_device/bt656/README.md)）。

## 文件结构

| 文件 | 作用 |
|---|---|
| `bt656.h` | **应用头文件**：`bt656_init/start/get/release/stop/stats` 等 |
| `bt656_decoder.h` | 驱动内部头：`bt656_decoder_ops`、注册函数，应用不要包含 |
| `bt656.c` | 通用层：HAL_DCMI/HAL_DMA、缓冲池、借还、cache、恢复、IRQ |
| `ark7128.h` | 芯片适配层内部头（`ark7128_mode_t`、注册函数） |
| `ark7128.c` | 芯片适配：I2C、引脚、软复位/锁定、ops 注册 |
| `ark7128_cfg.c/.h` | 各输入格式的寄存器表与几何 |
| `ark7128_i2c.c/.h` | 分页寄存器读写、芯片 ID、锁定检测 |
| `Kconfig` | `Board Peripherals → Camera → BT.656 Camera` 菜单、`Decoder chip` 选择与 `Video input format` 选项 |

## 板级连接（由板级 pinmux 配置）

引脚在板级 `customer/boards/sf32lb57-dpi-hdk_base/bsp_pinmux.c` 的
`BSP_USING_BT656` 块中配置；驱动不再调用 `HAL_PIN_Set`。I2C 总线名由
Kconfig `ARK7128_I2C_BUS_NAME` 选择（默认 `"i2c1"`）。

| 功能 | 引脚 / 参数 |
|---|---|
| I2C 总线名 | Kconfig `ARK7128_I2C_BUS_NAME`（默认 `"i2c1"`） |
| I2C1 SCL | `PA23`（上拉） |
| I2C1 SDA | `PA22`（上拉） |
| 从机地址 | `0x58`（`0xB0 >> 1`） |
| 模拟输入 | VIN2（page `0x10` / reg `0x0B` = `0x11`） |
| 模块复位 | 无（模块自带 RC 上电复位） |
| DCMI CLK | `PA03` |
| DCMI D0..D7 | `PA04 / PA08 / PA09 / PA05 / PA10 / PA07 / PA36 / PA35` |

## 配置

`menuconfig` → `Board Peripherals` → `Camera` → `BT.656 Camera` →
`Decoder chip` → `ARK7128` → `Video input format`：

| 选项 | 符号 | 几何 | 完成判据 |
|---|---|---|---|
| CVBS NTSC（默认） | `ARK7128_VIDEO_CVBS_NTSC` | 720×240 单场 | DMA 字节数 |
| CVBS PAL | `ARK7128_VIDEO_CVBS_PAL` | 720×288 单场 | DMA 字节数 |
| AHD 720P 25fps | `ARK7128_VIDEO_AHD_720P_25` | 1280×720 整帧 | DCMI FRAME 事件 |
| AHD 720P 30fps | `ARK7128_VIDEO_AHD_720P_30` | 1280×720 整帧 | DCMI FRAME 事件 |

`proj.conf`：

```conf
CONFIG_BSP_USING_BT656=y        # generic BT.656 capture layer
CONFIG_BSP_USING_ARK7128=y      # decoder chip
CONFIG_ARK7128_VIDEO_CVBS_NTSC=y
```

I2C 控制总线由 `ARK7128_I2C_BUS_NAME`（string，默认 `"i2c1"`）选择；所选总线
需要在 `Board Peripherals → I2C` 里使能，引脚在板级 `bsp_pinmux.c` 配置。

> 通用层自己提供 `DCMI_IRQHandler`，因此 `CONFIG_BSP_USING_DCMI` 必须关闭
> （Kconfig 已做成互斥）。DCMI 由 HAL 直接驱动，不经过 RT-Thread 的
> `"dcmi"` 设备。

## 应用 API

```c
#include "bt656.h"

int  bt656_init(void);
int  bt656_start(void);
int  bt656_stop(void);
int  bt656_deinit(void);

int  bt656_get(struct bt656_frame *frame, rt_int32_t timeout_ms);
void bt656_release(struct bt656_frame *frame);
void bt656_stats(rt_uint32_t *frames, rt_uint32_t *dropped, rt_uint32_t *errors);

int  bt656_set_source(rt_uint8_t source);
const struct bt656_format *bt656_get_format(void);
rt_uint16_t bt656_detect(void);
rt_uint8_t  bt656_signal_locked(void);
```

只有连续采集一种模型；`camera snapshot` 这类"取一帧"的用法是
`bt656_start()` → `bt656_get()` → `bt656_release()` → `bt656_stop()`。

### 帧所有权

- `bt656_get()` 借出一帧，`frame->data` 指向驱动内部缓冲，已做 cache 无效化；
- `frame->token` 是借还令牌，原样传回 `bt656_release()`；
- 从 `get` 成功到 `release` 之间，这块缓冲不会被 DMA 重新写入；
- 忘记 `release` 不会泄漏内存，但可用缓冲会逐块减少，最终所有帧被丢弃；
- `bt656_stop()` 之后，尚未归还的帧立即失效（缓冲会被下一次 `start` 复用）；
  不得再使用旧句柄，用旧 token 调用 `bt656_release()` 可能清掉新会话的占用位；
- **只支持一个消费线程**：`ready` 槽和唤醒信号量按“一帧在途”设计，多个线程
  并发调用 `get` 会互相抢走唤醒。

### 背压

缓冲池固定（CVBS 3 块 / AHD 2 块），`ready` 槽最多保留一帧："丢旧保新"；
其余缓冲都被应用持有时，新帧直接丢弃且不发布，计入 `dropped`。内存占用
恒定，延迟不会随运行时间增长。

```c
bt656_init();
bt656_start();
while (running) {
    struct bt656_frame f;
    if (bt656_get(&f, 500) == RT_EOK) {
        /* f.data / f.width / f.height / f.stride / f.bytes / f.seq */
        bt656_release(&f);
    }
}
bt656_stop();
```

## 采集流程

```
bt656_init()
  └ ops->init()                        // I2C + 读 ID + 首次下发寄存器表
    HAL_DMA_Init + 链接 DCMI 句柄 + NVIC

bt656_start()
  └ ops->configure()                   // 解码器软复位 + 重下整表
    ops->wait_ready(800)               // 连续 5 次锁定，超时固定 800 ms
    HAL_DCMI_Init()                    // 嵌入式同步、码组、CM、IE 等
    清空缓冲池 / cache
    HAL_DCMI_Start_DMA(第 0 块)
```

每收完一场/一帧：

- 逐行（AHD）：`HAL_DCMI_FrameEventCallback` → 换下一块缓冲；
- 隔行（CVBS）：`bt656_get()` 轮询 `HAL_DCMI_GetRemainingCount()==0` → 换下一块。

两者都调用 `HAL_DCMI_Start_DMA()` 重装 DMA，**不清 `CR.CAPTURE`**。

启动后的第一帧由驱动直接丢弃（启动时刻可能落在场/帧中间，数据不是从边界
开始的），所以 `bt656_start()` 之后第一次 `bt656_get()` 返回的已经是从边界
开始写满的完整一场/一帧。

带超时的 `bt656_get(frame, timeout_ms > 0)` 在等待期间一直没有帧时返回
`-RT_ETIMEOUT`，并按信号状态分别处理：

- 解码器报“无信号”：只打印限频日志 `no frame in N ms, no signal, waiting for
  source`（约每秒最多一次），不复位 DCMI，等信号恢复；
- 有信号却收不到帧：打印 `no frame in N ms (remaining x/y), restart capture`，
  先复位 DCMI 模块并重新起流（`bt656_recover()`）。

### 同步码

BT.656 的 SAV/EAV 码字由协议规定，内置在 `bt656.c`：

| | FSC / LSC / LEC / FEC | 掩码 |
|---|---|---|
| 隔行（CVBS） | `AB / 80 / 9D / B6` | `FF / FF / D4 / FF` |
| 逐行（AHD） | `C7 / C7 / DA / F1` | `70 / 70 / 70 / 70` |

芯片适配层不提供同步码；只有遇到输出码组不标准的解码器时才需要扩展
`bt656_decoder_ops`。

## 内存

| 缓冲 | CVBS NTSC | CVBS PAL | AHD 720P | 位置 |
|---|---|---|---|---|
| `ark7128_stream_buf` | 3 × 414,720 B | 3 × 414,720 B | 2 × 1,843,200 B | CVBS: PSRAM1；AHD: PSRAM2 |
| 显示画布（例程） | 1,228,800 B | 同左 | 同左 | PSRAM1 |

AHD 流缓冲使用 `L2_NON_RET_BSS_SECT(frambuf, ...)`，依赖链接脚本里的
`.l2_non_ret_bss_frambuf*` 通配规则落到 PSRAM2。

## 新增一颗 BT.656 解码芯片

1. 新建 `<chip>.c`，实现 `struct bt656_decoder_ops`（`init/deinit/configure/`
   `wait_ready/set_source/detect/signal_locked/format`，以及缓冲池地址/块数/stride）；
2. 在 `INIT_BOARD_EXPORT` 阶段调用 `bt656_register_decoder(&ops)`；
3. 应用无需修改。

同一时刻只支持一颗芯片（`bt656_register_decoder()` 重复注册返回 `-RT_EBUSY`）。

## 修改指引

| 想改什么 | 改哪里 |
|---|---|
| 换输入格式 | `ark7128_cfg.c` 的 `ark7128_fmt_*` + `Kconfig` |
| 换解码器寄存器 | `ark7128_tbl_*`（主表）/ `ark7128_extra_*`（附加） |
| 换输入通道 | `ark7128_i2c.h` 的 `ARK7128_INPUT_SEL` |
| 换 DCMI 引脚 | 板级 `customer/boards/sf32lb57-dpi-hdk_base/bsp_pinmux.c` |
| 换 I2C 引脚 | 板级 `customer/boards/sf32lb57-dpi-hdk_base/bsp_pinmux.c` |
| 换 I2C 总线 | Kconfig 的 `ARK7128_I2C_BUS_NAME` |
| 调 ITU 输出相位 | `ark7128_cfg.c` 的 `0x08/0xF5` 附加寄存器 |
| 调显示位置/尺寸 | 例程的 `bt656_show.c` |
| 换缓冲块数/位置 | `ark7128.c` 的 `ARK7128_STREAM_BUFS` / `L2_NON_RET_BSS_SECT` |

## 注意

- `bt656_init()` 是幂等的，重复调用直接返回 `RT_EOK`；运行期间不要调用
  `bt656_deinit()`，它会停流并复位缓冲池。每次 stop→start 都会重新做解码器锁相。
- `frame->seq` 与 `bt656_stats()` 的三个计数在每次 `bt656_start()` 时清零；
  超时恢复（`bt656_recover()`）保留累计值。
- 采集等待必须是阻塞式的（`rt_sem_take` / `rt_thread_mdelay`），忙等会饿死显示线程。
- **连续采集的固定横向偏移是已知问题**，当前驱动没有做补偿。
- 驱动不再包含 bring-up 调试接口。
