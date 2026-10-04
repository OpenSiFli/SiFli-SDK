# SiFli SDK 自动化测试框架（test_fwk）设计方案

## Context

**为什么要做**。当前 SDK 的质量把关只有"编得过"：`.gitlab-ci.yml` → `tools/ci/dynamic_pipeline.py` 只跑 `scons`，`build_job.sh` 用 exit code 判成败；根目录 `tests/` 是 stdlib unittest（220 个方法）但**没有任何 CI 或 make 目标调用它**；真机测试依赖 `tools/autotest/*.bat`，而其中的关键执行器（`Thinker.exe`、`SwitchReset.exe`）**不在仓库里**，是 Jenkins 上的外部工具，路径还硬编码到 `Z:\\gitroot`、`\\\\ftpserver`。结果是：没有统一的用例收集、没有硬件分配、没有可复现的结果上报。

**做成什么样**。按 `SDK自动化测试.docx` 建两个系统：**SDK 侧**（收集 testcase、pytest 调度、定义 scheme，不绑定具体硬件）+ **Lab Agent**（持有真实硬件的 runner，控制平面 + 去中心化 runner）。两者靠 binding 文件连接。schema 对齐 Zephyr twister，宿主编排对齐 ESP-IDF pytest-embedded。覆盖需求 1（模块 API/边界单测）、2（端到端 console 断言）、3（硬件行为打桩）、4（多设备协同）。

**已确认决策**（用户，含两轮修正）：
- 交付物 = 设计文档 + 可运行 MVP。
- **MVP 只做 Tier A（真板卡）**；Tier B（mock 单测）本期只出设计与接口。
- C 测试框架 = **引入 Unity**（能力是 utest 的超集：setUp/tearDown、浮点/数组/内存断言族、runner 生成），存量 utest 用例不回迁。
- 需求 3 打桩 = **CMock**（Unity 配套），机制是**不编译被 mock 的 HAL 实现**，不走手写桩、不依赖 weak。
- Tier B 载体路线 = **先 pc 仿真板打样 → 之后 QEMU**（`E:\code2\qemu-support` worktree 已打通）。
- Lab Agent = MVP 单机 runner + 手工 binding。
- MVP 必通链路 = hello_world console 端到端 + 真板卡 Unity 单测跑通调度。

**一个关键澄清（改变了分层）**：用户指出"mock flash 驱动就不会真的上板测试了，因为没有真实 flash 驱动板子无法运行程序"。精确地说——XIP 取指走硬件 MPI 映射、不经由 `drv_spi_flash.c`，所以只排这一个文件未必起不来；但**整个 `drivers/hal` 不编译**则时钟/RCC、UART、pinmux、MPI 全无，板子确实跑不起来，也没有串口上报通道。更根本的是语义：被测对象是 flash 驱动本身时，在真 flash 上跑它没有意义。**结论：mock 层必须有非板卡载体**——这也正好解释了 docx 那句"SDK 侧的测试可以不需要依赖任何硬件"。

---

## 交付状态（as-built，本文其余部分为设计意图）

> **本次提交的范围**：测试框架本身（发现 / binding / 能力匹配 / 编排 / 三种 harness /
> 后端 / 日志与报告）+ Lab Agent + 板上 Unity 运行时 + 用例注册表生成器。
>
> **不在内的**，各见另一批：
>
> | 不在本次提交里 | 内容 |
> |---|---|
> | QEMU 载体 | `customer/boards/qemu_*`、`rtos/rtthread/bsp/sifli/drivers_qemu/`、`tools/qemu/`、`example/qemu/`，以及它要求的构建侧改动（`BSP_USING_QEMU`/`SOC_QEMU` 两个 Kconfig 符号、`tools/build/building.py`、`cpuport.c` 的 `bf0_hal_hlp.h`） |
> | CMock 打桩（需求 3） | `external/cmock/`、`middleware/test_fwk/mocks/`、`example/test/{cmock_unit,hal_mock}/` |
>
> 下文凡提到这些的地方，请按这个边界读：**`backends/qemu.py` 这个后端本身在本次提交里，
> 它要驱动的那个载体不在**。而 QEMU 这条路线本身的做法尚未定型，本文对它的描述是设计意图，
> 不是既成事实。
>
> （示例用例工程 `example/test/unit_test/` 与 `example/get-started/hello_world/` 的
> `testcase.yaml` 曾列在上面，现已随框架一起提交。）

| 部分 | 状态 |
|---|---|
| SDK 侧 `tools/sdk_test/`：发现 / binding / 能力匹配 / 编排 / 报告上报 | ✅ 完成，**350 个自测全绿** |
| Lab Agent `tools/lab_agent/`：独占占用 / 串口 / 探针 / 烧录 | ✅ 完成，**105 个自测全绿**，不需要硬件 |
| `sdk.py test` 入口（list / list-fixtures / plan / run / info） | ✅ 完成 |
| harness `console` / `unity` / `pytest` | ✅ 完成（`pytest` 由设计态提前到本期） |
| harness `shell` | ⬜ 仍会明确 raise |
| 板上 Unity：`external/unity` + `middleware/test_fwk` | ✅ 在本次提交里；单元测试用例工程见另一批 |
| 后端 `board`（Tier A，真板卡） | ✅ 完成 |
| 后端 `qemu`（Tier B，无硬件） | ✅ 后端已实现并跑通；**它要驱动的 QEMU 载体不在本次提交里** |
| 后端 `pc_sim` | ⬜ 只有接口 |
| 带芯片身份的 QEMU 板（`qemu_sf32lb{52x…58x}`） | ⬜ 已构建并跑通过，但**不在本次提交里** |
| CMock 打桩用例（需求 3） | ✅ 已完成并反向验证过；**用例工程与 mock 生成物不在本次提交里** |
| 日志与报告（按 twister 对齐） | ✅ 完成：每实例 `device.log` + `runner.log`、run 级 `runner.log`、JSON 主文档 + JUnit 派生、失败阶段标记 |
| 需求 4（多设备） | ⬜ schema 与插件接口已支持，调度未实现 |

**自测怎么跑**（不需要硬件、不需要 Lab Agent）：

```bash
python -m unittest discover -s tools/sdk_test/tests -p "test_*.py"
python -m unittest discover -s tools/lab_agent/tests -p "test_*.py"
```

使用手册见 [`autotest_guide.md`](../../docs/source/zh_CN/app_note/autotest_guide.md)；SDK 侧实现说明见
[`README.md`](README.md)。

---

## 分层模型（本方案的骨架）

| | **Tier A：真板卡** | **Tier B：mock 单测** |
|---|---|---|
| 用例 | console 端到端、Unity 真外设单测 | 模块 API/边界 + CMock 打桩 |
| 载体 | 真板卡（Lab Agent 分配） | **非板卡**：`pc` 板打样 → `qemu_cortex_m33` |
| HAL | 真实 HAL | **不编译 `drivers/hal`** |
| 需求覆盖 | 1 / 2 / 4 | 1 / 3 |
| MVP | ✅ 本期交付 | ❌ 本期只出设计与接口 |

**载体在 SDK 里天然统一**：真板卡、`pc_hcpu`、`qemu_cortex_m33` 都是 `scons --board=<name>` 的取值。区别只在"要不要向 Lab Agent 申请硬件"——真板卡要，pc/QEMU 不要（无硬件，也就没有 testbed 分配与互斥）。

---

## 现状盘点（已核实，作为设计约束，不要重复探索）

| 事实 | 位置 / 结论 |
|---|---|
| 板上 C 测试框架 | RT-Thread **utest** 已在树内：`rtos/rtthread/components/utilities/utest/`，Kconfig `RT_USING_UTEST`（`rtos/rtthread/components/utilities/Kconfig:210`）。40+ 链接脚本模板已保留 `UtestTcTab` 段。**Unity 不在树内**（`external/lvgl_v9/tests/unity/` 是上游死代码，无 SConscript）。 |
| 现成真机测试套件 | `example/hal_example/`：37 个 `src/example/example_*.c`，`UTEST_TC_EXPORT`，`proj.conf` 里 `CONFIG_RT_USING_UTEST=y`。MSH 命令 `utest_list` / `utest_run <name>`。 |
| utest 结果契约 | `utest.c` 打印 `[  PASSED  ] [ result   ] testcase (%s)` / `[  FAILED  ]`。解析先例：`tools/audio_test_script/utestClient.py`。 |
| 宿主载体（两个） | ① `customer/boards/pc`：`BSP_USING_PC_SIMULATOR`，MSVC/`ARCH='sim'`/x86 `.exe`，**`drivers/SConscript` 整树跳过 `drivers/`**，用 `drivers_pc/` 假驱动替代。② `customer/boards/qemu_cortex_m33`：**不在本次提交里**（见开头的范围说明），`BSP_USING_QEMU` 同样整树跳过 `drivers/`，走 arm-none-eabi + QEMU mps2-an505，控制台是模拟 UART0（**不是** semihosting，理由见需求 3）。 |
| 命令扩展点 | `tools/sdk_py_actions/cli/registry.py` `CommandRegistry`，自动发现 `*_ext.py`；**`test` 路径未被占用**。参考 `flash_ext.py`（含 `build_<board>` 解析）。 |
| 烧录 | `sftool -p COMx -b 1000000 -c <chip> -m <nor\|nand> write_flash f@addr ...`，文件清单来自 `build_<board>/sftool_param.json`（机读）。J-Link 走生成的 `download.jlink`。`sdk.py flash` 已封装两条路径。 |
| 控制台 | 设备固定 `uart1`（`customer/peripherals/console/Kconfig`），**波特率 1000000**（`drv_usart.c:29`）。 |
| 串口坑 | `FINSH_ECHO_DISABLE_DEFAULT=1` → **输入不回显**；`RT_CONSOLEBUF_SIZE=128` → 长行会**截断/拆行**；无全局锁 → 行间可能插噪声。期望逻辑必须容忍。 |
| 复位 | 必须**带外**：J-Link（`tools/segger/halt_all_cpu_and_disable_all_wdt_*.jlink` + `tools/autotest/reset.txt`）或电源继电器。串口 DTR 不是复位。 |
| 板卡元数据 | `customer/boards/<board>/{hcpu,lcpu,acpu}/{rtconfig.py,board.conf,Kconfig.board}`（`JLINK_DEVICE`/`CHIP`/`CORE`）+ `ptab.{json,yaml}`。`build_config.yaml`/`build_config_gcc.yaml` 已给出 example×board 矩阵。 |
| 依赖现状 | 依赖不再按 feature 文件安装：`install.* --enable-<feature>` 机制**已在 commit 8259175ea 移除**，全部来自 `tools/locks/<profile>/uv.lock` 这一个锁定环境（`pyserial`/`pyyaml`/`jsonschema`/`pytest` 都在里面）。框架本身不需要额外依赖——`tools/requirements/` 下那几个 `requirements.*.txt` 是那个已移除机制的遗留，不再被任何东西读取。串口编程纪律参考 `tools/sdk_py_actions/sifli_uart_dap.py`（惰性 import、context manager、reset 缓冲区）。 |
| CI 许可证扫描 | `tools/autotest/copyright_check.bat` + scancode → 新引入第三方源码必须带 LICENSE 并过白名单。 |

---

## 架构

![SiFli SDK 自动化测试框架架构](assets/autotest_architecture.svg)

> 两个版本，同一份内容：上面这张 SVG 是给文档内嵌用的（跟着阅读器的明暗主题走）；
> `assets/autotest_architecture.html` 是**独立可浏览版**，同一套内容外加"尚未收敛"清单，
> 直接用浏览器打开即可。**改内容时两边都要改**——它们没有生成关系。

同一张图的纯文本版本（终端里读、diff 里看）：

```
+-- SDK 侧 tools/sdk_test/（不绑定硬件） ------------------------------------------------+
|                                                                                        |
| example/**/testcase.yaml -> discovery -> TestInstance[] -> ①平台过滤 <- features.yaml  |
| 用例只声明要什么              schema 校验     id=路径+tests 键   ②testbed 匹配 <- binding.yaml |
|                                                                   （硬件团队维护）     |
|                               |                                                        |
|                               v                                                        |
| runner.py 编排                                                                         |
|   1. plan   选平台 / 找工程目录 / 定 build_<board>_hcpu                                |
|   2. build  scons --board=<board> -j<n>      （构建函数由 CLI 注入，库不认识 scons）   |
|   3. open   backend.open() -> Session{console}                                         |
|   4. run    harness.run(instance, console, timeout)                                    |
|   5. close  backend.close() -- 在 finally 里，崩了也要把硬件还回去                     |
|                                                                                        |
| harness：[x] console  [x] unity  [x] pytest  [ ] shell    shell 会 raise，不静默空转   |
| report： JSON 主文档 + JUnit 派生 + run 级日志 + 终端汇总（默认产出）                  |
|                                                                                        |
+----------------------------------------------------------------------------------------+
                                            | backends/：按平台名自动选
+--------------------------+   +--------------------------+   +--------------------------+
| board [x] Tier A         |   | qemu [x] Tier B          |   | pc_sim [ ] Tier B        |
| 真板卡                   |   | 无硬件载体               |   | 无硬件载体               |
|                          |   |                          |   |                          |
| 申请 -> 烧录 -> 开串口   |   | 本机跑 qemu-system-arm   |   | 本机跑 .exe              |
| -> 复位（顺序是关键）    |   | 控制台 = 模拟 UART0      |   | MSVC，只能 Windows       |
|                          |   | 不需要 Lab Agent         |   | 只有接口，未实现         |
| 经 Lab Agent             |   |                          |   |                          |
|                          |   | 可跑 harness: unity      |   |                          |
+--------------------------+   +--------------------------+   +--------------------------+
                                             | 本机 HTTP（stdlib urllib）
+-- Lab Agent tools/lab_agent/（机器侧） ------------------------------------------------+
| 独占占用 testbed（一次一个测试程序） - 复位 / 烧录 / 开串口                            |
| transport：serial - jlink - sftool - relay（只有接口，会抛 RelayNotAvailable）         |
| 控制台用游标读，不在 HTTP 上做流                                                       |
+----------------------------------------------------------------------------------------+
                                            | sftool（1 Mbaud）/ J-Link
                                  真实板卡（跑 Unity 或 msh）
```

**两条载体路线是同一个抽象**：真板卡、`qemu_cortex_m33`、`pc_hcpu` 都是
`scons --board=<name>` 的取值，区别只在"要不要向 Lab Agent 申请硬件"。
所以用例自己不需要知道跑在哪；`integration_platforms` 声明了平台的用例
连 binding 文件都不需要。

**分工原则**（对应 docx）：SDK 侧只声明**要什么**，永远不出现真实端口号；端口、调试器 ID、mux 通道全在 runner 的 binding 里。fixture 同名即等价——分到哪块板子结果必须一致。

这条线现在划在**两个问题**上，各由一个文件回答，互不越界：

| 问题 | 谁来答 | 谁去问 |
|---|---|---|
| 这条用例能不能跑在这块**平台**上 | `customer/boards/<board>/<core>/features.yaml`（板子声明它有哪些总线实例） | `depends_on:` |
| 这块平台上**哪块板、怎么接的线** | binding（硬件团队维护） | `harness_config.fixture:` |

阶段一（平台过滤）不读 binding，阶段二（testbed 匹配）不读 features——所以
"有 flash 接在 spi1，但板子没声明 spi" 这类矛盾不会再静默存在：两处一旦
同时在手就会报出来（`fixtures.wiring_problems`）。

---

## 目录与文件

### 1. SDK 侧：`tools/sdk_test/`（twister 等价物）

| 文件 | 职责 |
|---|---|
| `cli.py` + `tools/sdk_py_actions/test_ext.py` | 注册 `sdk.py test`（`CommandRegistry`，路径 `test`/`test/run`/`test/list`/`test/list-fixtures`）。`test_ext.py` 只转发。 |
| `discovery.py` | 从 `-T/--testsuite-root` 递归找 `testcase.yaml`，按 `common:` + `tests:` 展开为 `TestInstance`，ID = 相对路径 + tests 键（twister 规则）。 |
| `schema/testcase.schema.json` | jsonschema 校验。字段对齐 twister：`tags / type / depends_on / platform_allow / platform_exclude / integration_platforms / extra_args / extra_configs / filter / build_only / timeout / harness / harness_config / min_flash / min_ram / slow / skip`；多设备另加 `dut:` / `dut_client:`。 |
| `model.py` | `TestInstance` / `TestBed` / `FixtureRef` / `RunnerConfig` 数据类。 |
| `binding.py` | 载入 binding/hardware-map YAML（testbed、console、connections、muxes），校验 fixture 引用存在。 |
| `fixtures.py` | fixture 能力匹配、`fixture:name:param` 后缀解析（twister 语义）。 |
| `harness/{console,unity,shell,pytest}.py` | 四种 harness 的判定逻辑（见下）。 |
| `pytest_plugin.py` | 提供 `testbed` / `dut` / `console` / `fixture` / `apppath` / `build_dir` fixture，形态对齐 pytest-embedded。 |
| `backends/{board,pc_sim,qemu}.py` | 执行后端。**MVP 只实现 `board`**；`pc_sim`（`customer/boards/pc`）与 `qemu`（`customer/boards/qemu_cortex_m33`）定义接口 + `NotImplementedError`，但**接口按"无需 Lab Agent 分配"设计**（它们在 SDK 里就是 board 名，构建+执行都在本地）。 |
| `runner.py` | 编排：解析 → 匹配 testbed → 构建 → （Tier A）申请 testbed → 烧录 → 执行 pytest → 收集。 |
| `agent_client.py` | 与 Lab Agent runner 的 HTTP 客户端（stdlib `urllib`，不引 FastAPI）。 |
| `report.py` | 结果聚合、每实例日志落盘、run 日志、JSON 主文档与 JUnit 派生、终端汇总。全部用标准库（`json` / `xml.etree`），不引依赖；`junit_xml` 这个包虽然还在 lock 里，但**无人 import**。 |
| `tests/` | **框架自身**的 pytest 单测（discovery / binding / harness 解析），不需要硬件。 |

### 2. Lab Agent：`tools/lab_agent/`

| 文件 | 职责 |
|---|---|
| `runner.py` | 服务入口：读 `--config runner.local.yaml`（本机有哪些 testbed），暴露 localhost HTTP：`GET /testbeds`、`POST /allocate`、`POST /release`、`POST /testbed/{id}/{reset,flash,console/open,console/close}`。MVP 用 stdlib `http.server` + JSON。 |
| `testbed.py` | testbed 生命周期与**互斥占用**（docx：同一时刻只能被一个测试程序占用）。前置准备（复位、烧录、fixture 就绪）对用例透明且不可跳过。 |
| `transport/serial.py` | pyserial 封装：惰性 import、context manager、`reset_input_buffer()`、`expect(regex, timeout)`（容忍拆行/噪声）、`sendline()`。**1 Mbaud 默认**。 |
| `transport/jlink.py` | 复位/烧录：`tools/segger/halt_all_cpu_and_disable_all_wdt_*.jlink` + `r`；或 `sdk.py flash -u jlink`。 |
| `transport/relay.py` | 电源/mux 带外控制。**接口先定**（`power_on/off`、`mux_select(channel)`），MVP 只留桩——`SwitchReset.exe` 不在仓库。 |
| `fixture_drivers/` | `passive.py`（纯接线，MVP）+ 驱动的注册/`setup/teardown/action` 接口（active 驱动预留）。 |

### 3. Unity 引入：`external/unity/`

新增 `unity.c/unity.h/unity_internals.h` + `SConscript`（`DefineGroup('unity', src, depend=['RT_USING_UNITY'])`）+ `Kconfig`（`RT_USING_UNITY`）+ **`LICENSE`/`README`**（CI scancode 会扫）。不重用 `external/lvgl_v9/tests/unity/`，避免和上游树耦合。

注：Unity 用显式 `RUN_TEST()` 注册，**不需要新的链接段**，与已有的 `UtestTcTab` 段模板互不干扰。

### 4. 板上胶水：`middleware/test_fwk/`

| 文件 | 职责 |
|---|---|
| `unity_config.h` | `UNITY_OUTPUT_CHAR` 重定向到 `rt_kprintf`；输出格式开关固定，保证文本契约稳定。 |
| `sifli_unity_runner.c` | `UNITY_BEGIN()/RUN_TEST()/UNITY_END()` 的 main；导出 MSH 命令 **`unity_run [suite\|case]`** 与 **`unity_list`**，形态与 `utest_run` 对称，让宿主只靠串口就能选择/枚举用例。 |
| `SConscript` / `Kconfig` | 由 `CONFIG_USING_TEST_FWK` 门控。 |
| `mocks/` | CMock 生成物落点。**本期建目录 + 设计说明，不实现**（Tier B）。 |

### 5. MVP 的测试工程（Tier A）

- `example/get-started/hello_world/rtt/testcase.yaml` — `harness: console`，断言 banner + `Hello world!` + `msh />`。
- `example/test/unit_test/` — 新工程，链接 `external/unity` + **纯逻辑模块用例**（首批 `external/lz4/lz4.c`、`external/cJSON-1.7.17/cJSON.c`、`middleware/ipc_queue/common/circular_buf.c`），`harness: unity`。用纯逻辑模块是因为它们不依赖特殊外设（任意 testbed 都能跑），正好验证 fixture 能力匹配；同时不需要打桩，与 Tier A 的边界一致。

### 6. 示例配置 + 文档

`examples/binding.local.yaml`、`examples/runner.local.yaml`（带注释的可抄模板）、`tools/sdk_test/DESIGN.md`（含 Tier A/B 分层说明与打桩规则）。

---

## 四种 harness 的判定契约

| harness | 板上有没有测试代码 | 宿主怎么判 | MVP |
|---|---|---|---|
| `console` | 没有 | 正则匹配串口输出；`harness_config.record` 支持 twister 的 `type: one_line\|multi_line`、`ordered`、`repeat`、`separator` | ✅ |
| `unity` | 有（Unity） | **先等 shell 提示符**，再发 `unity_run`，解析 Unity 汇总（`N Tests M Failures K Ignored` + `OK`/`FAIL`），并逐条映射 `test_<suite>_<case>:PASS` | ✅ |
| `shell` | 没有 | 按序下发 `{command, regex, ordered, repeat}` 到 msh | 设计 |
| `pytest` | 可选 | 用例目录自带 `test_*.py`，拿 `dut` fixture 自由编排（多设备/外部仪器）；解析 Unity 时用 `dut.expect_unity_test_output()` | 设计 |

`harness_config` 键名沿用 twister：`fixture`、`record`、`pytest_root`、`pytest_args`、`dut_fixture`、`pytest_dut_client`；`unity` 另有 `prompt`（默认 `msh />`）。

**unity 为什么要先等提示符。** `unity_run` 是一条 msh 命令，发早了 shell 还不存在，
命令就丢了——症状是耗完超时后报 `no Unity summary line found`，**看起来像固件坏了**。
在没有探针的板子上这是常态而非例外：console 是在下载刚写完、板子还没开始启动时打开的。

**为什么不是"等串口安静"**（试过，不行）：板子还没开始启动时也是安静的，于是"静默"
被当成了"已经说完"，命令照样发丢。**需要的是正向信号，不是没有信号。**
等提示符也不算额外假设——`unity_run` 本身就是 msh 命令。

等不到提示符**不算失败**，命令照发、由原有检查判：否则就是凭一个新启发式造出一种
新的失败，把提示符不同的固件误判成坏的。

---

## 需求 3 的设计（Tier B，已交付）

**选型**：CMock（ThrowTheSwitch，Unity 的配套 mock 框架）。按头文件自动生成 mock，自带调用次数期望、参数捕获、返回值序列、`Verify`、严格顺序检查，正好覆盖"擦除失败 / 写超时 / ID 不匹配"这类硬件复现不出的分支。

**机制：不编译被 mock 的 HAL 实现**（按用户要求）。mock 就是该符号的唯一定义——无重复定义、不依赖 weak、不与 ROM 表竞争。

**为什么载体必须在 Tier B**：Tier A 是真板卡，而结果上报走串口 console（→ `drv_usart.c` → `bf0_hal_usart.c`）、启动依赖 RCC/pinmux/PMU，所以**真板卡上不能整树不编译 `drivers/hal`**。两个非板卡载体都天然满足这个前提：

- `qemu_cortex_m33`（**首选；载体不在本次提交里**）：`BSP_USING_QEMU` 让 `drivers/SConscript` 整树跳过 `drivers/`。同一份 ARM elf、同一套工具链，跨平台、能进 CI，且控制台完全双向（已跑通 `harness: unity`，见下）。
- `pc` 板（`BSP_USING_PC_SIMULATOR`）：同样整树跳过 `drivers/`，用 `drivers_pc/` 假驱动替代。代价：MSVC / 只能 Windows，Linux CI 跑不了。
  - **`sim/x86` vs `sim/win32` 的疑点已被推翻**：`SifliMsvcEnv()` 会把 `rtconfig.CPU` 覆写成 `win32`，路径推导是对的。
  - `--board=pc` 的全量构建原本编不过，原因是 `external/liblc3` 需要 C11 `<stdalign.h>` 而 MSVC 2017 没有；已在 `external/liblc3/SConscript` 上对模拟器跳过，pc 全量构建现在能过。

**旧结论作废：HAL 外设函数在 QEMU 载体上可以打桩。**
下面这段曾是"两个非板卡载体都做不到"的论证，**它对 `pc` 板成立，对 QEMU 不成立**，
保留作对照——因为卡点的真正性质当初判错了：

1. `bf0_hal_qspi.h` 无条件 `#include "qspi.h"`，该头只在 `drivers/cmsis/sf32lb55x/`、`sf32lb58x` 下有；
   `drv_config.h` 要的 `dma_config.h` 在 `customer/boards/include/config/<chip>/`。
2. `bf0_hal.h` 要求 `SOC_BF0_HCPU`/`SOC_BF0_LCPU`，而 QEMU 是伪 SoC（`SOC_QEMU`）。
3. `drivers/cmsis/<chip>/register.h` 依赖 `core_cm33.h`，该文件整个仓库里都没有
   （它来自 arm-none-eabi 的 CMSIS）——**MSVC（`pc` 板）上无解，这就是老结论的来源**；
   但 QEMU 走 arm-none-eabi，那个工具链自带，芯片头能干净编过。

**真正的卡点不是"有没有芯片头"，而是芯片身份从哪来、include 顺序对不对。**
在测试工程的 `SConstruct` 里手工注入 `-DSOC_BF0_HCPU -DSF32LB52X -DUSE_HAL_DRIVER`
加三条 `PrependUnique`，能编过，但错在两处：

- **编译器宏不是 Kconfig 符号。** `drv_spi_flash.c` 的分支条件是 `#ifndef SOC_SF32LB55X`，
  `-DSF32LB55X` 满足不了它，会静默编错分支，报出来的是"缺某个声明"，看不出是身份问题。
- **include 顺序是承重的。** `bf0_hal.h` 按名字 `#include "register.h"`，芯片那份必须压过
  载体自己的 `drivers_qemu/register.h`；写成 `Append` 则报"缺寄存器映射"，与根因无关。

**解决办法：给每个系列一块带芯片身份的 QEMU 板**，共五块
（`customer/boards/qemu_sf32lb{52x,55x,56x,57x,58x}`，同一个 mps2-an505 机器、
同一份 `link.lds`，只有芯片身份不同）。板子的 `Kconfig.board` 里 `select SOC_SF32LB5<nnn>X`，
`rtos/rtthread/bsp/sifli/drivers_qemu/SConscript` 把 Kconfig 身份翻译成 include 路径与宏，
**顺序也在那里保证**。测试工程里一行芯片相关的东西都不用写，也就没有"两个身份不一致"的可能。

这些在真板卡上分别来自 `drivers/cmsis/<chip>/SConscript`（芯片头路径 + `SF32LB<nnn>X`）与
`drivers/hal/SConscript`（`SOC_BF0_HCPU` + `USE_HAL_DRIVER`），两者都在 `drivers/` 整树里、
被 `BSP_USING_QEMU` 一并跳过，所以由载体侧补上等价物。

**已交付（两个工程，都做了反向验证）**：

- `example/test/cmock_unit/` —— 打**中间件**函数 `circular_buf_*` / `ipc_hw_*`，
  测 `ipc_queue` 的写路径（5 个用例）。
- `example/test/hal_mock/` —— 打**HAL** 函数 `HAL_QSPI_GET_SR`，
  测 `drv_spi_flash.c` 的 `rt_flash_get_status_register()`（3 个用例）；
  在五个系列的新板上都构建并跑通。52x/56x/57x/58x 用 `mock_bf0_hal_mpi_ex.*`，
  55x 用 `mock_bf0_hal_qspi_ex.*`——两份 mock 不是冗余，是 HAL 家族按系列分岔，
  选哪份由板子的芯片身份决定。

**打桩的成本比预想小得多。** `drv_spi_flash.c` 有 1908 行、`nm -u` 列出 35 个外部符号，
看着要 mock 一大片，但 gcc 构建默认 `-ffunction-sections` + `--gc-sections`
（`SifliGccEnv`，`tools/build/building.py:4223`），链接期只保留被测函数真正到达的部分。
`rt_flash_get_status_register()` 的闭包是**两个符号**：

```
Addr2Handle -> BSP_Flash_get_handle     板级，测试里给个两行定义
HAL_QSPI_GET_SR                         HAL，由 CMock 生成
```

独立佐证：ROM 镜像自己的链接映射
（`example/rom_bin/lcpu_general_ble_img/lcpu_img_56x_52.axf`）里这个函数同样只有这两条边。

唯一容易漏的是**镜像级闭包比"这个函数的闭包"多一个符号**：`drv_spi_flash.c` 用了
`INIT_BOARD_EXPORT(rt_sys_spi_flash_init)`，而 QEMU 的 `link.lds` 有
`KEEP(*(SORT(.rti_fn*)))`（不 KEEP 就没 shell、没 components），`--gc-sections` 丢不掉它，
于是 `rt_hw_flash_init -> BSP_Flash_Init` 也被拉进来，测试里因此有两个板级定义。

**基建**：`external/cmock/` 已入库（`external/unity/` 见上）；
`external/unity/auto/type_sanitizer.rb` 是 CMock 的硬依赖，已补回；
`middleware/test_fwk/mocks/cmock_config.yml` 是验证过的配置（含 `:strippables` 的必要写法）；
`mock_circular_buf.c/h` 已生成入库。详见该目录的 `README.md`。

**实测的符号表边界（已核实）**：

先说一句结论上的简化：**在 QEMU 载体上这段完全不适用**——挂符号表的逻辑
（`drivers/hal/SConscript:142`）本身就在 `drivers/` 整树里，被 `BSP_USING_QEMU` 跳过，
所以载体上永远不会挂 `sifli_rom_<chip>_gcc.sym`（在新板子的构建产物里搜 `sifli_rom`：0 处）。
下面这段只在**把 mock 用例搬回真板卡**时才需要看——本文风险 7 提到的那种做法。

`drivers/hal/SConscript:142` 的条件是 `if not GetDepend('ROM_ATTR') ...`，普通工程（`ROM_ATTR` 只被 `example/lcpu_rom/Kconfig.proj` 打开）**会**把 `sifli_rom_<chip>_gcc.sym` 用 `-T` 挂上。但表里导出的是 `_rom` 后缀名，我们关心的外设 API 普通名**不在表内**：

```
HAL_ADC_Init                plain:[]   _rom:[HAL_ADC_Init_rom = 0x000028cd]
HAL_ADC_PollForConversion   plain:[]   _rom:[HAL_ADC_PollForConversion_rom = 0x00002979]
HAL_I2C_Master_Transmit     plain:[]   _rom:[HAL_I2C_Master_Transmit_rom = 0x00006af1]
HAL_QSPI_GET_SR             plain:[]   _rom:[]          <- 完全不在表中
HAL_QSPIEX_WRITE_PAGE       plain:[]   _rom:[]          <- 完全不在表中
```

→ 外设 API 打桩与链接脚本零冲突。表内另有 **414 个普通名绝对定义**（`sifli_rom_52x_gcc.sym` 共 422 条，`_rom` 只 8 条：`rt_hw_interrupt_disable`、`HAL_LPAON_EnterDeepSleep`、`HAL_GPIO_ClearInterrupt` 等内核/电源/AON 符号）；非板卡载体上这些符号本来也不需要 ROM 版本，若要 mock 它们，用 `CONFIG_ROM_ATTR=y` 让 `-T sym` 不被挂上（先例 `example/lcpu_rom`）。

**落地形态（已完成）**：
- ✅ `external/cmock/`：CMock 2.7.0 源码 + LICENSE 已入库，**故意没有 SConscript / Kconfig**（不接入构建）。
- ✅ 生成器是 **Ruby**，本机已装 3.3.12。策略：**生成一次后把 `mock_*.c/h` 提交进仓库**，构建路径零 Ruby 依赖；头文件变更时手工重跑（已写进 `mocks/README.md`，否则是长期摩擦点）。
- ✅ mock 落点 `middleware/test_fwk/mocks/`，配置与四份 mock 就位。
- ✅ **用例已写**：`example/test/cmock_unit/`（中间件）与 `example/test/hal_mock/`（HAL），
  见上文"已交付"。
- ✅ **载体已从"无芯片"改为"带芯片身份"**：五块 `qemu_sf32lb<nnn>x` 板。
  芯片相关的东西（include 路径、`SF32LB<nnn>X`、`SOC_BF0_HCPU`、`USE_HAL_DRIVER`、
  以及 include 顺序）全由板子与载体给出，测试工程不再注入。
- ⚠️ **`customer/boards/Kconfig`（v1 流程）没有为新板子加条目**，所以不带 `--board=`
  的旧版 `menuconfig` 里看不到它们——v1 的芯片是 choice、`SOC_QEMU` 与
  `SOC_SF32LB<nnn>X` 无法共存，而这个特性恰恰要求两者同时为 y。
  `--board=` 走的 KCONFIG_V2 流程里 `SOC_*` 都是普通 bool，所以能共存，新板子据此工作。
  要支持 v1 需要先动芯片 choice 的结构，本期没做。

---

## 日志与报告（按 twister 对齐）

**为什么**。手动跑 `sdk.py test run` 时有两条诉求：完整的串口日志要留存用于事后分析；一旦有用例红了，要知道是**哪几条**。改动前两条都答不上——完整的串口原文**已经存在但被丢掉**：`QemuConsole.transcript`（`backends/qemu.py`）与 `ConsoleStream.transcript`（`lab_agent/testbed.py`，docstring 写的就是"for logging a failed run"）都是死代码，session 一关就销毁；逐条用例结果被 unity harness 解析成 `HarnessOutcome.cases` 后在 `runner.run_one` 里被丢弃；而全包唯一的文件写入是 `--report` 触发的 JUnit。

**布局**。产物按**"谁产生这段文字"**分层，这是 twister 的结构——失败可能出在任一层，混在一起会互相淹没：

```
<outdir>/                        默认 test-out/，--outdir 可改
  <stamp>/                       每次 run 一个，2026-10-03T17-09-22
    runner.log                   run 级：这次跑成什么样、跑在什么之上
    sifli.json                   主文档
    sifli.xml                    从 JSON 派生
    <platform>/<instance-id>/
      build.log                  编译器原文（--no-build 时不存在）
      device.log                 设备原文，逐字节，零添加
      runner.log                 该实例的记录：元信息 + 裁决 + 逐条用例 + harness 原文
```

`device.log` 故意保持"纯"（无表头、无分隔线），这样能 `diff`、能 `grep`、能原样贴进工单；叙述性的东西全在 `runner.log` 里。

**为什么每次 run 要一个目录**（`report.run_dir`）。原先所有 run 写进同一棵树，后一次直接盖掉前一次——而**通过用例的日志正是判断下一次 flaky 的基线**，静默丢掉它恰好是这套布局要避免的那一件事。目录名用本地时间、`2026-10-03T17-09-22` 这个格式：不含冒号（Windows 文件名禁止），且**字典序即时间序**，`ls` 出来最新的在最后。同一秒内起的两次 run 由 `run_dir` 加 `-2`/`-3` 后缀分开，不会并成一棵树。

代价是失去了固定路径，所以**报告与日志被拆成两条独立路径**：`--report-dir` 可以把报告钉到固定目录（CI 收 junit 用），日志仍按 run 分目录。不指定时报告跟着时间戳走。

时间戳在 `TestRunner.__init__` 里**只算一次**——三处调用点（实例日志目录、`RunSummary.out_dir`、`_write_artifacts`）各算各的会差几秒，把同一次 run 的实例日志和 run 日志劈到两个目录里。

**为什么不是 `<build_dir>/`**（twister 放 `instance.build_dir`）：我们的 `build_dir` 是 per 工程+板、不是 per 实例，而且它是 scons 的地盘——`scons -c` 会把日志一起删掉。另外 `instance.id` 不足以唯一定位：一个 testcase 写多个 `integration_platforms` 会产出多个共享 id 的 planned run，所以 `<platform>/` 那层是必需的。还有一点：**id 随 `-T` 变**（`discovery.py:117`，id 相对 `-T` 根算），同一个测试用宽根和窄根跑，日志路径不同——不能从测试名反推。

**三层日志的分工**（沿用 twister 的名字）：`build.log` = 编译器说的；`device.log` = 设备说的；`runner.log` = harness 据此怎么判的。

**`build.log` 是后补的**，而且补的理由很直接：构建输出原先直通终端、不落盘，于是构建失败的 `reason` 只有"退出码 2"——同一个 2，可能是缺头文件也可能是类型错，答案全在被滚过去的那段里。现在 `build_fn` 的契约是**返回**编译器输出而不是往外流，`_do_build` 返回它、`run_one` 在 `BuildError` 上也要接住（失败时恰恰最需要那段），最后由 `write_instance_logs` 落到 `build.log`。

只在这一层做捕获，副作用正好是想要的：**控制台不再刷屏**。`--no-build` 不写这个文件——没调用过编译器，写个空文件等于声称调用过而它没话说。

**报告是同一份数据的投影**。JSON 是主文档，JUnit 由它派生（twister 的 `xunit_report(json_file, ...)` 就是打开 `.json` 再拼 XML）。这样终端 / JSON / JUnit 三个渲染器不会各自拼数据、不会漂移。日志只在**失败**时内联进报告（twister `reports.py:374`），通过实例的 transcript 留在磁盘上——否则每个绿实例都可能往报告里塞最多 4 MB。

**JUnit 展开成逐条用例**：有 `cases` 的实例一个用例一条 `<testcase>`，而 `tests/failures/errors/skipped` 头部属性从**实际输出的元素**重算，免得头和身子对不上。没有 cases 的（console / pytest harness，或根本没能跑到 harness）退回实例级一条。

**`stage` 区分失败阶段**。`ERROR` 是个筐：规划失败、构建失败、目标打不开、串口断掉都落进来，原先只能靠读消息文本分辨。`TestResult.stage` 取 `plan/build/open/run/close`，记录**最后进入的阶段**——所以通过实例恒为 `run`，而 `at build` 才有含义：这次根本没走到 run。

**保留策略在两个维度上都与 twister 相反**：

- **run 内**：twister 默认删通过用例的产物（`--runtime-artifact-cleanup=pass`），我们不删。
- **run 间**：twister 启动时清空 `twister-out/`（`--clobber-output` 默认开），我们每次 run 另起一个时间戳目录。

它那套是因为每个实例的构建目录装着一整个 Zephyr 构建（数百 MB）；我们只有几个文本文件，而**通过用例的日志恰恰是排查 flaky 时最需要的基线**——这条理由在 run 间同样成立，甚至更成立：横跨两次 run 的对比才是判断"这次是不是偶发"的唯一依据。

**两个已如实标注的缺口**：

- **板卡路径未端到端验证**。`BoardSession.transcript` 走 `read_from(0, 0.0)`（agent 端 `ConsoleStream.read_from` 在 `_end > cursor` 时立即返回整个保留缓冲，不等超时），语义核过源码，但**没有真板卡跑过**——QEMU 通过不等于板卡可用。
- **transcript 是保留窗口不是全量**。两端都有 4 MB 上限（`qemu.py` 的 `max_buffer`、`lab_agent/config.py`），超出从头部裁剪。实例 `runner.log` 的表头会写明保留了多少、裁掉了多少。

---

## 需求 4 的设计（多设备，未实现）

- **多 DUT**：`testcase.yaml` 支持 `dut:` / `dut_client:` 列表（twister 语义），每项可带 `platform_allow`；`runner.py` **原子地**一次申请 N 个 testbed（按 testbed id 排序申请，避免死锁）；pytest 插件在 N>1 时把 `dut` 暴露成 list（pytest-embedded 风格）。
- **I2C 双板**：一块板跑用例，另一块作为对端 fixture。对端板 = 一个 `fixture` + 一个 active driver。
- **手机**：建模成 fixture（如 `fixture: android_phone:pixel_01`），由 runner 侧 driver 插件驱动（adb / BLE central），手机池化后按能力分配；多台手机＝同名 fixture 多实例，等价性由硬件团队保证。
- **带外硬约束**（docx 强调两次）：mux/电源/复位**不得**用 MCU 自己的 IO 做控制信号，必须走 runner 的 relay/driver。`transport/relay.py` 与 fixture driver 接口就是为此预留。

---

## MVP 交付边界（原计划只做 Tier A；实际多做了 QEMU 与 pytest harness）

**做**：
- `tools/sdk_test/`：discovery + binding + 能力匹配 + 两种 harness（console/unity）+ pytest 插件 + 日志与报告（每实例 / run 级 / JSON / JUnit）+ 框架自身单测。
- `tools/lab_agent/`：单机 runner（serial expect + J-Link 复位 + sftool 烧录 + testbed 互斥占用）。
- `external/unity` + `middleware/test_fwk`（unity_config + sifli_unity_runner + SConscript/Kconfig）。
- 两个测试工程 + 示例 binding/runner 配置 + 一份文档。

**不做（但接口留好）**：`backends/pc_sim.py`（抛 `NotImplementedError`，接口按"无需 Lab Agent"设计）、CMock 与全部打桩实现（`middleware/test_fwk/mocks/` 只建目录 + 设计说明 + 生成好的 mock）、`transport/relay.py` 实体、active fixture driver、控制平面/dashboard、多 DUT 实际调度（schema 与插件接口先支持）。

**实际交付比原计划多做了两件**（都被"先做出来才能验"推着走）：

1. **`harness: pytest` 提前到本期**——原本只是设计态。
2. **`backends/qemu.py` 提前到本期**，连带把 QEMU 载体一起做了。
   这是最有价值的一次超范围：它让**板上的那套 Unity 单测可以完全不用硬件跑**，
   也第一次让 Tier B 有了现实载体。**后端在本次提交里，载体不在**（见开头的范围说明）。
   代价是 QEMU BSP 上踩了一串坑（链接脚本谎报 RAM 大小、链接脚本缺 init 段、
   semihosting 输入会把 vCPU 阻塞住、UART 基址是 AN505 特有的 0x40200000），
   记在 `docs/source/zh_CN/app_note/autotest_guide.md` 第 4 节——那一节也随载体一起在另一批。

**依赖新增**：无。框架自己（`console` / `unity` harness、Lab Agent、自测）是纯标准库；
`pyyaml` / `jsonschema` / `pyserial` 本就在锁定环境里，`pytest` 也已加入
`tools/locks/default/`（只有 `harness: pytest` 的用例需要它）。
串口按 `sifli_uart_dap.py` 的惰性 import 纪律。

---

## 验证方式

✅ = 已实际跑过。

1. ✅ **框架自测**：`python -m unittest discover -s tools/sdk_test/tests -p "test_*.py"` → **350 全绿**；
   `tools/lab_agent/tests` → **105 全绿**。都不需要硬件，也不需要 Lab Agent。
   （写成 stdlib `unittest` 而非 pytest：默认 SDK python 环境里没有 pytest，
   而这套自测必须开箱即用；两者兼容。）
2. ✅ **用例发现**：`sdk.py test list -T example` 列出 ID 与 harness；`--json` 给 CI 生成矩阵用。
3. ⬜ **真板卡端到端（需求 2）**：`sdk.py test run -T example/get-started/hello_world --binding <bind>`。
   代码路径已就绪、有端到端自测覆盖（真 Lab Agent + 伪造设备），但**没有在真实板卡上跑过**。
4. ✅ **无硬件端到端（QEMU）**：`sdk.py test run -T example/qemu` 通过；
   板上那套 Unity 单测也能整个搬过来跑：`-t unit_test.cjson_qemu` 通过。
5. ✅ **反例必须失败**：期望串写成 `Hello worldXYZ!` 时判定为 FAIL；
   Unity 的空转（`0 Tests` + `OK`）被拒；乱序的启动日志被拒。
   （这一条是这套框架最容易做假的地方，单测里钉死了。）
6. ✅ **退出码**：失败 → 1、通过 → 0，两条都实测过。
7. ⬜ **复位/释放鲁棒性连跑**：需要真板卡，未做。
8. ⬜ **CI 接入点**：`.gitlab/` 里加 test stage，artifact 收 `test-out/**`（用 `--report-dir` 把 `sifli.xml`
   钉到固定路径给 GitLab 当 `junit` 报告——时间戳目录让它不再有固定位置；`device.log` / `runner.log`
   当普通 artifact，`when: always`）；本次未启用。
9. ✅ **日志与报告**（都用 `example/test/hal_mock` 在 QEMU 上实测）：
   - 默认产出 `test-out/<时间戳>/{runner.log, sifli.json, sifli.xml}` 与每实例的 `{device.log, runner.log}`；
     `device.log` 里**含启动 banner**——那正是 unity harness 的 `_discard_pending` 会丢掉、只能从后端
     transcript 拿到的部分。
   - **每次 run 一个时间戳目录**，上一次的不被覆盖；同秒的两次 run 拿到 `-2` 后缀；run 目录在开跑前
     就打印出来（`[outdir]` 那行），可以照着 tail 正写着的 `device.log`。
   - **失败清单**：断言改红后终端列出红的子用例、`file:line` 与 `Expected/Was`，退出码 1；
     `-i` 把日志路径换成内联全文。
   - **逐条展开**：`sifli.xml` 里 hal_mock 从 1 条 `<testcase>` 变 3 条，头部计数与实际元素一致；
     通过用例是自闭合元素、无子元素。
   - **stage 的区分能力**：`#error` 造构建失败 → `error`/`build`；断言改红 → `failed`/`run`。
   - **兼容别名**：`--report legacy/junit.xml` 写出 `legacy/junit.{xml,json}`。

**Tier B（打桩）相关**：
- 反向验证**已做**（两个工程都做了）：故意让 mock 的期望不匹配，必须报 FAIL——否则证明不了
  mock 真被链接进去。见需求 3 一节。
- 载体用**带芯片身份的 QEMU 板**（`qemu_sf32lb52x` 等）。`pc_hcpu` 的路径疑点已排除，但后端仍是 stub。

---

## 风险与待确认

1. **hello_world 期望字符串有坑**：工作区 `example/get-started/hello_world/rtt/src/main.c` 是**未提交**的 `rt_kprintf("Hello world3!\n")`，而 `README.md` 写的是 `Hello world!`。期望串必须**写在 testcase.yaml 里**，绝不能从源码推导。
2. **Unity 引入成本**：新引入 `external/unity` + 输出重定向 + runner（Ruby 生成 runner，策略同 CMock：生成物入库）。CI 的 scancode 会扫许可证，需带 LICENSE 过白名单。
3. **两套框架并存**：存量 37 个 utest 用例（`example/hal_example`）不回迁，需明确"新用例写 Unity"。Unity 用显式 `RUN_TEST()`、不需要链接段，与 40 多个已保留 `UtestTcTab` 段的模板不冲突。
4. **串口物理限制**：1 Mbaud、`FINSH_ECHO_DISABLE_DEFAULT=1`（无回显，别把 expect 逻辑建在回显上）、`RT_CONSOLEBUF_SIZE=128`（长行拆块）——expect 必须是"累积缓冲 + 正则跨块匹配"。
5. **`SwitchReset.exe` 不在仓库**：MVP 复位只能靠 J-Link；纯串口（无调试器）的 testbed 在 MVP 里支持不了电源复位，需求 4 的 mux/电源部分要等硬件团队提供带外继电器方案。
6. **QEMU 载体**：`customer/boards/qemu_cortex_m33`、`rtos/rtthread/bsp/sifli/drivers_qemu/`、
   `example/qemu/`、`tools/qemu/` 都已做好并跑通过，**但不在本次提交里**——本次只带
   `backends/qemu.py` 这个后端本身。剩下的只是 BSP 自身的坑（链接脚本 RAM 大小、
   init 段、semihosting 输入阻塞 vCPU、UART 基址 0x40200000），
   见 `docs/source/zh_CN/app_note/autotest_guide.md` 第 4 节（同样在另一批）。
7. ~~**Tier B 的 HAL 排除粒度**~~ —— **已解决，且原先的判断是错的。**
   原文说"HAL 外设函数的打桩在两个非板卡载体上都做不到"。它对 `pc`（MSVC）成立，
   对 QEMU 不成立：QEMU 走 arm-none-eabi，`core_cm33.h` 由工具链提供，
   `qspi.h`/`dma_config.h` 只要芯片身份正确就能找到。真正的卡点是**芯片身份从哪来**
   （Kconfig 符号而非编译器宏）和 **include 顺序**，两者现在都由板子承担。
   详见需求 3 一节。
   仍然成立的那半句：若将来要把 mock 用例搬回真板卡跑，粒度必须收成**按文件排除**
   （保留 UART/RCC/pinmux/PMU），否则症状是镜像起不来或结果不打印、而不是链接报错，排查成本高。
8. ~~**pc 板可用性未验证**~~ —— **已查实**：`sim/x86` vs `sim/win32` 的疑点是**假警报**
   （`SifliMsvcEnv()` 会把 `rtconfig.CPU` 覆写成 `win32`）；`--board=pc` 的全量构建原本编不过，
   原因是 `external/liblc3` 缺 C11 `stdalign.h`，已修。`backends/pc_sim.py` 仍是 stub。

**本轮新暴露、尚未收敛的**：

9. **`unit_test.lz4` 的 `boundaries` 断言失败**（第一次真正被执行到）。
   它期望 `LZ4_decompress_safe(..., dstCapacity=4)` 在解出 1024 字节时返回**负值**，
   实测返回 **+1024**。LZ4 头文件的契约明说这种情况要返回负的错误码，
   且入库的 `lz4.c` 是未改动的上游副本、`_output_error` 分支也在。
   已排除是 Unity 报错格式的误读（Unity 先打 actual 再打 threshold）。

   **已在真板上复现**（此前只是记录在案、从未执行过）。事后补了两条：
   契约原文是 `necessarily <= dstCapacity` 且"不够时返回负值"，**实测两条都违反**；
   而 `restored[]` 实际是 1024 字节，所以没写坏内存——但返回 `op - dst = 1024`
   意味着它确实推进了 1024 字节，**若调用方真给 4 字节缓冲就是溢出**。

   **仍未定论，且机制没查明**：把安全解码路径里应当返回负值的分支（`cpy > oend`
   → `_output_error`）读了一遍，没找到 1024 是怎么出来的。这份 lz4 是上游 1.9.3
   原样，版本很成熟，所以不凭读代码下结论。开发机上没有 C 编译器，无法脱离板子复现。
   没有为了让套件变绿而弱化断言。
10. **QEMU 控制台没有走中断，是轮询的**。已实测：使能 UART 接收中断、把所有向量都指向探针、
    放开全部 32 条 NVIC 线，**没有任何设备中断产生**。所以别拿它验证中断相关行为；
    成因也还没查清（QEMU 的 mps2-an505 是否根本没把 UART 的 IRQ 连到 NVIC，未验证）。
11. **`tools/qemu/` 下的脚本已过期**：`qemu_run.bat` / `qemu_run.sh` / `qemu_smoke_test.py`
    仍用 semihosting 调用，而 BSP 的控制台已经换成 UART，照它们跑不会有输出。
    需要改成 `-display none -monitor none -serial stdio`。
12. ~~**CMock 打桩用例仍未交付**~~ —— **已交付两个**：
     `example/test/cmock_unit/`（中间件，5 个用例）与 `example/test/hal_mock/`
     （HAL 函数，3 个用例）。都是 `sdk.py test run -T example/test/<工程名>`，
     不需要硬件，反向验证都做过。踩到的点记在 `middleware/test_fwk/mocks/README.md`。
13. **真机首跑抓出的两个问题**（这一轮 `unit_test` 三条用例第一次在真板上跑）：

     - **`circular_buf_putchar_force` 少存一次写指针** —— **已修**。
       它丢掉最旧一字节（推进读指针）后**没有** `cb->write_idx_mirror = ...`，
       于是"丢一个收一个"变成了纯丢一个：满载 16 字节日志变成 15。函数自己的注释
       （"discard one old data"）和 `return 1` 都与实现矛盾。对照 `circular_buf_putchar`
       一眼可见——那边两个分支算完 idx/mirror 后统一存回，这边没有那一步。
       修法即在末尾补上同一句。
       顺带证伪了一个顾虑：`__ROM_USED` **不是**"放进 ROM 段"，它展开成 `RT_WEAK`
       或空；也没有任何 ROM 符号表导出这个函数，所以改的这一份就是生效的那一份。
     - **`unit_test.lz4` 的 `boundaries`** —— 见风险 9，仍未定论。
14. **测试工程引工程外源文件的坑**（`example/test/unit_test/src/SConscript`）。
    lz4/cJSON 原先是在这里手工 `src += [os.path.join(SIFLI_SDK, 'external/...')]` 引入的。
    **裸路径字符串经 `DefineGroup` 不会变成 SCons Node**，它会一路进链接对象列表；
    而在真板上（`drivers/` 被编进来，SPI 驱动那个带 `LOCAL_CCFLAGS` 的组存在；
    QEMU 上整树跳过 `drivers/`，所以不复现）有个去重循环会对每个对象取 `.abspath`，
    整个构建就这么死了：

    ```
    AttributeError: 'str' object has no attribute 'abspath'
      tools/build/building.py:3651
    ```

    现在 lz4/cJSON 改走 Kconfig（`CONFIG_PKG_USING_LZ4/CJSON=y`，组件自己的
    SConscript 负责源文件与 include 路径），只剩 `circular_buf` 一处手工引入——
    它的组件是按芯片分叉的目录树、不是一个文件，且 QEMU 上根本编不过。
    那一处保留 `File(...)` 包裹，**已反向验证**：去掉就复现上面的报错。

**关于 `example/test/unit_test` 的一个顺手可得的改进（本期未做）**：
`unit_test/src/SConscript` 现在在 QEMU 上把 `circular_buf.c` 整个跳过，
理由是 `ipc_hw.h` 会碰 `bf0_hal.h`、在无芯片载体上编不过。那个理由随着
带芯片身份的板子消失了——改用 `qemu_sf32lb52x` 后，
`unit_test.circular_buf` 这个用例应该也能在 QEMU 上跑（它目前在真板卡上跑）。
没有一并改，是因为本次的验收范围是打桩用例本身。
