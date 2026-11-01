# 打桩落点（CMock）

**当前状态：已被两个工程消费。**

| 工程 | 被测对象 | 被 mock 的符号 | 载体 |
|---|---|---|---|
| `example/test/cmock_unit/` | `ipc_queue.c` | `circular_buf_*`、`ipc_hw_*`（中间件） | `qemu_sf32lb52x` |
| `example/test/hal_mock/` | `drv_spi_flash.c` | `HAL_QSPI_GET_SR`（HAL） | 五个 `qemu_sf32lb<nnn>x` 板 |

```bash
sdk.py test run -T example/test/cmock_unit       # 5 个用例
sdk.py test run -T example/test/hal_mock         # 3 个用例
```

两个都不需要硬件、不需要 Lab Agent、不需要 binding 文件。

下文先记生成方式与配置，再记载体结论——**"HAL 函数打不了桩"那条旧结论已作废**，原因见后面"载体结论"一节。

---

## 生成方式（生成一次，入库）

CMock 是 **Ruby 编写的主机端生成器**，不在构建期运行。策略是**生成一次、把产物提交进仓库**，
这样固件构建（以及任何没装 Ruby 的机器）都不需要 Ruby。

```bash
# 需要 Ruby（本机 3.3.12，C:\Ruby33-x64\bin）
ruby external/cmock/lib/cmock.rb -o middleware/test_fwk/mocks/cmock_config.yml <header.h>
# 产物 mock_<header>.c / .h 落在当前目录，即本目录
```

**CMock 需要一个 Unity 的 Ruby 辅助文件**：`external/unity/auto/type_sanitizer.rb`。
`lib/cmock_generator.rb:33` 会去 `external/cmock/lib/../../unity/auto/` 找它，
找不到就 `Failed to find an instance of Unity to pull in type_sanitizer module!`。
Unity 裁剪时删掉了整个 `auto/`，所以这个文件被单独取回来了——它是 CMock 的依赖，
不是 Unity 运行期代码。也可以用 `UNITY_DIR=<path>` 环境变量覆盖查找位置。

## `cmock_config.yml` 里两处值得注意的配置

- **`:enforce_strict_ordering: true` + `:fail_on_unexpected_calls: true`**：
  期望顺序错、或调用了没设期望的函数，立即判定失败。这是打桩能"证明自己真被链接进去"的基础。
- **`:strippables`**：`circular_buf.h` 里这些调试辅助在开关关闭时是**宏**：

  ```c
  #ifdef DBG_CIRCULAR_BUFFER
  void circular_buf_log(...);
  #else
  #define circular_buf_log(cb,idx,data,len)
  #endif
  ```

  CMock 看到的是声明、于是生成函数；消费方看到的是宏、于是把 mock 自己的函数定义展开掉，
  结果是语法错误。`strippables` 的语义是**只删除匹配到的文本**，所以模式必须覆盖整条声明——
  只写函数名会留下一个悬空的参数列表，CMock 会以
  `Failed Parsing Declaration Prototype!` 报错。

## 已生成

| 文件 | 用于 | 大小 |
|---|---|---|
| `mock_circular_buf.c/h` | 打 `circular_buf_*`（来自 `middleware/ipc_queue/common/circular_buf.h`） | 80 KB + 16 KB |
| `mock_ipc_hw.c/h` | 打 `ipc_hw_*` | 40 KB + 9 KB |
| `mock_bf0_hal_mpi_ex.c/h` | 打 MPI 系的 HAL（来自 `drivers/Include/bf0_hal_mpi_ex.h`），52x/56x/57x/58x 用 | 588 KB + 116 KB |
| `mock_bf0_hal_qspi_ex.c/h` | 打 QSPI 系的 HAL（来自 `drivers/Include/bf0_hal_qspi_ex.h`），55x 用 | 320 KB + 64 KB |
| `cmock_config.yml` | 上述配置 | |

**关于 HAL mock 的体积**：CMock 没有"只生成某几个函数"的开关，只能整头文件生成。
`bf0_hal_mpi_ex.h` 声明了 **593 个 API**，所以生成物是 12470 行 / 588 KB，而
`example/test/hal_mock` 只用到其中 1 个。

这是刻意的取舍，不是疏漏：产物是生成的、入库后从不手改，头文件更新就重跑生成器；
代价是一次性的仓库体积和一点编译时间（实测编译这一份约 3 秒）。
替代方案是手工维护一个"seam 头"——只列出要拦截的原型，生成物降到约 200 行——
但那要手工维护原型清单，且与"不走手写桩"的原则有张力。选了前者。

**两份 HAL mock 是因为 HAL 家族按系列分岔，不是冗余**：`drv_spi_flash.c` 在
52x/56x/57x/58x 上经 `bf0_hal_mpi_ex.h` 拿到 `HAL_QSPI_GET_SR`，在 55x 上经
`bf0_hal_qspi_ex.h`。两者签名相同但 `FLASH_HandleTypeDef` 的定义不同，
而且 55x 下 `mpi.h` 根本不存在（实测：MPI 那份 mock 在 55x 上编不过，
`mpi.h: No such file or directory`）。选哪一份由**板子的芯片身份**决定，
见 `example/test/hal_mock/src/SConscript`。

---

## 载体结论（已实测）

先说结论：**走通了。** 中间件函数与 HAL 函数都能打桩，载体是 QEMU，
被 mock 的符号在镜像里是唯一定义。

### 打桩的前提是"不编译被 mock 的实现"

`SOC_QEMU` 让 `drivers/SConscript` 整树跳过 `drivers/`，镜像里没有任何 HAL 实现，
所以 CMock 生成的 mock 就是它替换掉的那个符号的唯一定义——无重复定义、不依赖 weak、
不与 ROM 表竞争。`rtos/rtthread/bsp/sifli/SConscript` 同时把 `drivers_qemu/` 换进来，
载体本身仍然完整（控制台、启动、时钟都在）。

顺带一条已核实：`drivers/hal/SConscript:142` 那段挂 `sifli_rom_<chip>_gcc.sym` 的逻辑
同样在 `drivers/` 整树里，所以载体上**永远不会**挂 ROM 符号表（在新板子的构建产物里
搜 `sifli_rom`：0 处）。README 里"55x_a3 的 ROM 表中有普通名 `HAL_QSPI_GET_SR`"那条提醒
在这个载体上不适用。

### 旧结论"两个非板卡载体都做不到打 HAL"——对 `pc` 成立，对 QEMU 不成立

当初的结论是基于 `pc` 板（MSVC）得出的，那里确实无解：MSVC 没有 `core_cm33.h`，
芯片头直接编不过。但 **QEMU 走的是 arm-none-eabi**，那个工具链自带 CMSIS，
芯片头能干净编过。原结论里"打桩目标必须换成中间件级函数"这一步是不必要的。

### 让 HAL 头编起来：芯片身份必须来自板子，不能来自工程的 `-D`

`drv_spi_flash.c` 会 `#include "bf0_hal.h"`，而 `bf0_hal.h` 要求一个芯片身份。
一开始的做法是在测试工程的 `SConstruct` 里手工注入：

```python
env.PrependUnique(CPPPATH=[...drivers/cmsis/sf32lb52x, ...drivers/cmsis/Include,
                           ...customer/boards/include/config/sf32lb52x])
env.AppendUnique(CCFLAGS=['-DSOC_BF0_HCPU', '-DSF32LB52X', '-DUSE_HAL_DRIVER'])
```

能编过，但**是错的**，两个原因都实测踩到过：

1. **编译器宏不是 Kconfig 符号。** `drv_spi_flash.c` 里
   `#ifndef SOC_SF32LB55X` 才是真正的分支条件，`-DSF32LB55X` 满足不了它。
   给 55x 头配 `-DSF32LB52X` 之类的错配会静默编错分支，报出来的是
   "缺某个声明"，看不出是身份问题。
2. **include 顺序是承重的，而且不可见。** `bf0_hal.h` 按名字
   `#include "register.h"`，芯片那份必须压过载体自己的 `drivers_qemu/register.h`。
   用 `Append` 而不是 `Prepend` 的话，报的是"缺寄存器映射"，与根因毫无关系。

现在的做法：给每个系列加一块 QEMU 板，板子的 `Kconfig.board` `select SOC_SF32LB5<nnn>X`，
由载体侧的 `rtos/rtthread/bsp/sifli/drivers_qemu/SConscript` 把它翻译成
include 路径与宏（并保证顺序）。工程侧一行芯片相关的东西都不用写。

板子清单：`customer/boards/qemu_sf32lb{52x,55x,56x,57x,58x}`，
都用同一个 QEMU 机器（mps2-an505）和同一份 `link.lds`，只有芯片身份不同。

### 打桩的成本比想象的小得多：`--gc-sections` 把闭包砍到 2 个符号

`drv_spi_flash.c` 有 1908 行，编出来的 `.o` 引用 **35 个外部符号**。
看着要 mock 一大片，实际不是：gcc 构建默认带 `-ffunction-sections`，
链接默认带 `--gc-sections`（`SifliGccEnv`，`tools/build/building.py:4223`），
所以只有被测函数真正到达的部分会被保留。

`rt_flash_get_status_register()` 的链接期闭包是**两个符号**：

```
Addr2Handle -> BSP_Flash_get_handle     板级，测试里给个两行定义
HAL_QSPI_GET_SR                         HAL，由 CMock 生成
```

独立佐证：ROM 镜像自己的链接映射
`example/rom_bin/lcpu_general_ble_img/lcpu_img_56x_52.axf` 里，这个函数同样只引这两条边。

**一个容易漏掉的地方**：镜像级的闭包比"这个函数的闭包"多一个符号。
`drv_spi_flash.c` 用了 `INIT_BOARD_EXPORT(rt_sys_spi_flash_init)`，
而 QEMU 的 `link.lds` 有 `KEEP(*(SORT(.rti_fn*)))`（不 KEEP 就没 shell、没 components），
`--gc-sections` 丢不掉它，于是 `rt_hw_flash_init -> BSP_Flash_Init` 也被拉进来。
所以测试里一共给了两个板级定义：`BSP_Flash_get_handle` 和 `BSP_Flash_Init`。
症状只是链接期一个未定义符号，不值钱，但知道来路省时间。

### 打桩验证的是调用方

打桩只替换符号，所以它验证的是**调用方的逻辑**——"HAL 报这种状态时驱动怎么做"——
而不是 HAL 本身的行为。这正是它的用处：那种分支在台架上复现不出来。

要往另一个方向走（验证 HAL 本身），是另一套机制：把 HAL 的实现编进镜像、让它的寄存器落在
RAM 里，见 `docs/source/zh_CN/app_note/autotest_guide.md` 第 7.2 节与 `example/test/hal_real/`。

### 反向验证（必须做，已做）

把 `HAL_QSPI_GET_SR_ExpectAndReturn(&g_flash, 0x00000003)` 的返回值改成 `0xDEADBEEF`，
对应用例确实 FAIL、退出码 1，另外两个用例不受影响。**这一步证明 mock 真的驱动了结果**——
如果链进去的是真实 HAL 或别的实现，改 mock 的期望不会有任何影响。改回后 3 个用例全绿。

`cmock_unit` 早先也做过同样的反向验证。

### 中间件那份踩到的三个点

1. **`ipc_queue_data_ind` 重复定义**：它在 `ipc_hw.h` 里声明、在 `ipc_queue.c` 里定义，
   而后者正是被测模块。mock 它就会有两个定义。用 `:strippables` 从 mock 里剥掉。
2. **`GlobalExpectCount` / `GlobalVerifyCount` 链接不到**：开了 `:enforce_strict_ordering`
   后，生成的 mock 只 `extern` 声明并赋值它们，从不定义——Ceedling 环境下由构建系统提供，
   在裸用时要测试工程自己定义。链接错误只说符号名，很容易误判成缺库。
3. **`_CallCount()` 不是调用计数**：它返回的是 **callback 调用计数**，
   普通 `_ExpectAndReturn` 下恒为 0。"有没有被调用"要靠 CMock 本来的方式证明：
   期望没被消费 → `_Verify()` 在 tearDown 里失败；没设期望却被调用 →
   `:fail_on_unexpected_calls` 直接判失败。真需要计数时用计数 stub。
