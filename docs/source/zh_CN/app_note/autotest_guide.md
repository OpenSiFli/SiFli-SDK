# 自动化测试框架使用指南

> 设计背景、分层模型与决策记录见仓库内 `tools/sdk_test/DESIGN.md`（不随文档站发布）。
> 本文只讲**怎么用**。

> **可以照着跑的示例现在有了**：`example/test/unit_test/`（板上 Unity 单测）与
> `example/get-started/hello_world/`（console 与 pytest 两个 harness）。
>
> 仍**不在本次提交里**的：第 4 节的 QEMU 载体（`customer/boards/qemu_*`、
> `rtos/rtthread/bsp/sifli/drivers_qemu/`、`tools/qemu/`、`example/qemu/`），
> 以及第 7.1 节 CMock 的**两个示例工程**（下一笔）。提到这两节的地方各自另有批次。
> 完整边界见 `tools/sdk_test/DESIGN.md` 开头那张表。

---

## 1. 它由哪两半组成

```
+-- SDK 侧（tools/sdk_test/，不绑定硬件） ---------------------------+
| testcase.yaml -> 发现 -> 匹配 -> 构建 -> 执行 -> 日志 + 报告       |
| （用例，平台）-> 交给 Lab Agent                                    |
+--------------------------------------------------------------------+
                                   | 本机 HTTP
                                   v
                  +--------------------------------+
                  | Lab Agent（机器侧）            |  持有 binding 与板子：串口 / 探针 / 继电器
                  | tools/lab_agent/               |  一次只发放一个 testbed
                  +--------------------------------+
                                   |
                             真实板卡
```

**分工只有一句话**：测试用例只声明**能力**（需要什么 fixture、什么外设），
永远不出现端口号、探针序列号、mux 通道。物理现实全在 Lab Agent 那侧的 binding 文件里。

---

## 2. 快速上手（不需要硬件）

> **本节可以直接照着敲。** 树内有 `example/get-started/hello_world/`（console 与 pytest）
> 和 `example/test/unit_test/`（板上 Unity 单测）两个示例用例工程可列、可规划。
> 只有 `example/qemu/` 随 QEMU 载体在另一批，所以 `-T example` 会少它那一条。

```bash
./export.ps1                      # Windows；Linux/macOS 用 source ./export.sh
```

```bash
# 看看有哪些用例
sdk.py test list -T example
sdk.py test list -T example --json          # CI 生成矩阵用
sdk.py test info                            # 本 build 支持哪些 harness / backend

# 干跑：只算计划，不构建、不烧录
sdk.py test plan -T example/get-started/hello_world \
    -p sf32lb52-lcd_a128r16
```

跑框架自己的测试（不需要硬件，也不需要 Lab Agent）：

```bash
python -m unittest discover -s tools/sdk_test/tests -p "test_*.py"
python -m unittest discover -s tools/lab_agent/tests -p "test_*.py"
# 也可以用 pytest 收集，两者兼容
python -m pytest tools/sdk_test/tests -v
```

---

### 2.1 跑完之后：日志和报告在哪

`test run` **默认就产出**产物（对齐 twister），不用加参数。跑完终端会打印：

```
[outdir] E:\...\test-out\2026-10-03T17-09-22     ← 开跑前就打，可以照着 tail device.log
[plan]  3 test(s) to run:                        ← 先列清楚要跑哪些，看进度用
        1/3  rtt/hello_world.console    (sf32lb52-lcd_a128r16, console)
        2/3  rtt/pytest/hello_world.pytest  (sf32lb52-lcd_a128r16, pytest)
        3/3  no-os/hello_world.noos     (sf32lb52-lcd_a128r16, console)

------- [1/3] rtt/hello_world.console  (sf32lb52-lcd_a128r16, console) -------
[build] rtt/hello_world.console for sf32lb52-lcd_a128r16
[run]   rtt/hello_world.console via board
[PASS ] rtt/hello_world.console on sf32lb52-lcd_a128r16_01  (25.5s)

------- [2/3] rtt/pytest/hello_world.pytest  (sf32lb52-lcd_a128r16, pytest) -------
...
[log]    E:\...\test-out\2026-10-03T17-09-22\runner.log
[report] E:\...\test-out\2026-10-03T17-09-22\sifli.json
[report] E:\...\test-out\2026-10-03T17-09-22\sifli.xml

================= rtt  (2 test(s): 2 passed) =================                 ← 结尾按 testcase 分组
[PASS ] rtt/hello_world.console on sf32lb52-lcd_a128r16_01  (25.5s)
[PASS ] rtt/pytest/hello_world.pytest on sf32lb52-lcd_a128r16_01  (21.9s)

================ no-os  (1 test(s): 1 passed) ================
[PASS ] no-os/hello_world.noos on sf32lb52-lcd_a128r16_01  (22.3s)

3 test(s): 3 passed, 0 failed, 0 errored, 0 skipped  in 69.8s
artifacts: E:\...\test-out\2026-10-03T17-09-22
```

**开跑前先列清单、每条用例之间有分隔线、跑完立刻给裁决**——这三点都是为了"看进度"：
长跑里唯一会问的问题是"还剩几条"，而结尾的汇总回答不了它。
结尾的汇总再**按 `testcase.yaml` 分组**（一个目录一个文件、通常也就是一份固件），
每组一行小结，失败的详情直接缩进在它那行下面，不用来回找。

**每次 run 一个时间戳目录**，所以上一次的日志不会被这次覆盖——通过用例的日志正是排查 flaky 时的基线。
时间戳是本地时间、`2026-10-03T17-09-22` 这个格式，**按字典序排就是按时间排**，`ls` 出来最新的在最后。
同一秒内起的两次 run 会自动加 `-2`、`-3` 后缀，不会并到一棵树里。

```
test-out/                        ← --outdir，默认落在你执行 sdk.py 的目录下（已 gitignore）
  2026-10-03T17-09-22/
    runner.log                   run 级：这次跑成什么样、跑在什么之上
    sifli.json                   主文档
    sifli.xml                    JUnit，从 JSON 派生
    qemu_sf32lb52x/
      hal_mock.spi_flash_status/
        build.log                编译器原文（--no-build 时没有这个文件）
        device.log               设备原文，含启动 banner
        runner.log               该实例的记录
  2026-10-03T16-40-05/           ← 上一次，还在
    ...
```

**报告和日志是两条独立路径**：报告可以用 `--report-dir` 钉到一个固定目录（CI 要固定路径收 junit），
日志仍然按 run 分目录。不指定时报告也落在这次的时间戳目录里。

**编译器输出不在控制台刷屏，落在 `build.log` 里。** 一次 `scons` 几百行、绝大多数是噪音，
而真想看它的时刻（构建失败）恰恰是它已经滚过去的时候。所以默认只打一行
`[build] <id> for <board>`，全文进文件。

三个文件各管什么、什么时候看哪个：

| 文件 | 回答的问题 |
|---|---|
| `build.log` | **编译器说了什么**。构建失败时唯一有答案的地方——退出码 2 看不出是缺头文件还是类型错 |
| `device.log` | **设备到底说了什么**。逐字节原文，无表头无分隔线，能 `diff`、能 `grep`、能原样贴工单 |
| 实例 `runner.log` | **harness 据此怎么判的**：裁决、逐条用例表、harness 原文，外加"串口保留了多少、有没有被截断" |
| run 级 `runner.log` | **这次跑成什么样**：发现/运行数、过滤掉哪些及原因、时间与环境 |

排查时的一句话规则：harness 报"我什么都没看到"时，**只有 `device.log` 能区分"设备没输出"和"设备输出了但 harness 没读到"**。

失败时终端直接给清单，不用去翻文件：

```
[FAIL ] hal_mock.spi_flash_status  (4.0s)  at run
        1 of 3 Unity test(s) failed: status_returned_as_reported
        FAIL  status_returned_as_reported (spi_flash:170): Expected 3 Was -559038737
        logs: <上面那个目录>
```

`at run` 是**失败阶段**：`plan` / `build` / `open` / `run` / `close`。`at build` 说明这次根本没走到运行，
问题在编译而不是用例——这是 `ERROR` 那一堆里最值得一眼分清的区别。加 `-i` 会把 `logs:` 那行换成串口全文。

**保留策略**：什么都不删——通过用例的日志留着，上一次 run 的也留着（twister 两样都删）。
要清理就按目录删：`test-out/` 下的时间戳目录彼此独立，删掉哪个都不影响别次。

---

## 3. 跑真机

两个终端。

**终端 A** —— 在插着板子的机器上起 Lab Agent：

```bash
cp tools/lab_agent/examples/runner.local.yaml my-runner.yaml   # 照着注释改
python tools/lab_agent/runner.py -c my-runner.yaml --check     # 先校验配置
python tools/lab_agent/runner.py -c my-runner.yaml --list      # 看识别到哪些 testbed
python tools/lab_agent/runner.py -c my-runner.yaml             # 起服务
```

**终端 B** —— 跑用例。**你只要说清"跑在哪个平台上"，以及 agent 在哪**：

```bash
sdk.py test plan -T example -p sf32lb52-lcd_a128r16     # 先看计划（不碰硬件）
sdk.py test run  -T example -p sf32lb52-lcd_a128r16     # 由 agent 分配一块空闲的板
```

`--agent-url` 默认就是 `http://127.0.0.1:8760`，agent 和你在同一台机器上时可以省。

退出码：`0` 全过，`1` 有用例失败，`2` 请求本身不可用。

### 3.2 agent 与你的 SDK 不是同一份代码怎么办

agent 是**长期运行的进程**，可能几天前从另一份 checkout 起的，而你这边的 SDK 一直在
变。分两层：

- **协议版本**：对不上**直接拒**，报错里写清两边各是什么。它只在接口真的不兼容时才
  会变——也就是"我发的字段你读不懂"那种，靠版本号猜不出来。
- **两侧 revision**：**只记录，不检查**。脏 worktree 是正常工作状态，拿它当门槛会在
  最需要它工作的时候（正在改这个框架）全线误拒。`sifli.xml` 的 `<properties>` 里有
  `sdk_revision`、`agent_revision`、`sdk_protocol`，事后问"这次结果到底是哪两份代码
  跑出来的"，看这里。两个 revision 不一样本身不是错误，但它是长驻 agent 拿着旧代码
  这个最常见事故的唯一线索。

**SDK 侧手里根本没有这个 lab 的清单。** 一次运行就是 (用例, 平台, 后端, 构建目录)，
再细没有了：具体落到那个平台的**哪块板**，这里既不决定、也决定不了——申请硬件那一刻
由 agent 从自己的配置里分配。所以你只要说清平台；硬件映射根本不进这一侧（那个文件
属于硬件团队，见第 8 节）。

本来在**构建之前**做的三道检查，现在都推迟到申请硬件那一刻：

| 少了什么 | 后果 |
|---|---|
| fixture 匹配 | 用例要的 fixture 没人提供，会在 `allocate` 时报错，而不是在 `plan` 阶段 |
| binding ↔ `features.yaml` 的矛盾检查 | "有 flash 接在 spi1，但板子没声明 spi" 这类问题不会提前报出来 |
| 平台过滤② | 用例声明要什么，`-p` 直接覆盖（见下） |

**`-p` 会覆盖用例自己的 `integration_platforms`**——选项优先。所以整棵树会被拉到
你指定的那块平台上跑，包括那些本来钉死在别的平台上的用例。这正是"我就要在这块板上
跑一遍"想要的行为，但它绕过了用例自己的声明，看到跨平台的用例被拉过来时别意外。

**想跑多个平台**：现在命令行只能给一个 `-p`。让整棵树在两个平台上各跑一遍，得在
用例里写 `integration_platforms:`，或者跑两次。

常用参数：

| 参数 | 作用 |
|---|---|
| `-T/--testsuite-root` | 从哪个目录递归找 `testcase.yaml`（默认 SDK 的 `example/`） |
| `-t/--testcase` | 只跑指定 id（可重复）。支持全 id，也支持只写 tests 键 |
| `--tag` | 按 tag 选（并集） |
| `-p/--platform` | 指定要构建的板名；除非每条用例都自己写了 `integration_platforms`，否则必须给 |
| `--no-build` | 跳过构建，直接用 `build_<board>/` 里现成的 |
| `-j/--jobs` | 并行构建任务数 |
| `--timeout` | 覆盖所有用例的超时 |
| `--outdir` | 产物根目录，默认 `test-out/`。**每次 run 在它下面建一个时间戳目录**，本次的 run 级 `runner.log`、每实例 `device.log`/`runner.log`、报告都在那里 |
| `-i/--inline-logs` | 失败时把串口日志**打进终端**，而不是只给路径 |
| `--report-dir` | 报告落点，默认等于 `--outdir` |
| `--report-name` | 报告文件名主干，默认 `sifli`（产出 `<name>.json` 与 `<name>.xml`） |
| `--report-suffix` | 加到报告文件名后，CI 里区分 build 号（→ `sifli_ci123.xml`） |
| `--report` | **兼容别名**：钉住 JUnit 的目录与文件名（会剥掉扩展名，JSON 落在同主干下）。现在报告默认就产出，一般不需要它 |
| `--fail-fast` | 首个失败即停（默认继续跑完，nightly 更需要全景） |
| `--no-confirm` | 中途不再停下来问。只影响 `reset.method: manual` 的板子（见第 8 节）。给了它就**没人回答**"板子起来了没"，于是"先写后读"的 harness 会被拒——也就是**没开 `autorun` 的 `unity`**；`console`、`pytest`，以及开了自动运行的 unity，都不受影响 |

---

## 4. 没有硬件也能跑：QEMU 载体

> **本节的内容随 QEMU 载体一起在另一批提交里。** 框架侧的 `backends/qemu.py`
> 已经在树内，但下面这些命令要的板子（`customer/boards/qemu_*`）、BSP
> （`rtos/rtthread/bsp/sifli/drivers_qemu/`）和示例（`example/qemu/`）都还不在，
> **照敲会失败**。而且 QEMU 这条路线本身的做法尚未定型，本节描述的是设计意图。

真板卡不是唯一的载体。`qemu_cortex_m33` 在 SDK 里就是一个普通板名，
所以**同一套用例、同一套 harness，可以完全不用硬件跑**——不需要 Lab Agent，
不需要 binding 文件，不需要接线。

```bash
sdk.py test run -T example/qemu                      # console harness
sdk.py test run -T example/test/unit_test --no-build -t unit_test.qemu
```

后一条跑的是**板上那套 Unity 单测**（同一个镜像里的三个 suite），只是换了载体。
用例自己声明跑在哪：

```yaml
  - id: unit_test.qemu
    harness: unity
    timeout: 120
    integration_platforms: [qemu_cortex_m33]   # 有它就不需要 binding
    harness_config:
      autorun: true
```

需要本机装了 QEMU；不在 PATH 上时用 `SIFLI_QEMU_EXE` 指过去
（本机在 `E:\Program Files\qemu\qemu-system-arm.exe`）。

### 4.1 带芯片身份的 QEMU 板

`qemu_cortex_m33` 是**伪 SoC**——它一个芯片都没选。要碰芯片相关的东西
（`bf0_hal.h`、寄存器映射、任何 `#ifdef SOC_SF32LB5xx` 的代码），就得用下面这五块板：

| 板名 | 芯片身份 |
|---|---|
| `qemu_sf32lb52x` | SF32LB52X |
| `qemu_sf32lb55x` | SF32LB55X |
| `qemu_sf32lb56x` | SF32LB56X |
| `qemu_sf32lb57x` | SF32LB57X |
| `qemu_sf32lb58x` | SF32LB58X |

同一个 QEMU 机器（mps2-an505）、同一份 `link.lds`，**只有芯片身份不同**。

```bash
scons --board=qemu_sf32lb52x -j8            # 直接构建
sdk.py test run -T example/test/hal_mock               # 用例自己声明平台
sdk.py test run -T example/test/hal_mock --no-build -p qemu_sf32lb55x   # 换系列跑
```

**它替你做了什么**：板子的 `Kconfig.board` 里 `select SOC_SF32LB5<nnn>X`，
载体侧（`rtos/rtthread/bsp/sifli/drivers_qemu/SConscript`）据此加上芯片的
include 路径与 `SF32LB<nnn>X` / `SOC_BF0_HCPU` / `USE_HAL_DRIVER` 三个宏，
**并且保证 include 顺序**——`bf0_hal.h` 按名字 `#include "register.h"`，
芯片那份必须压过载体自己的 `drivers_qemu/register.h`。

**所以测试工程里不要再写这些**。自己注入 `-DSF32LB52X` 有两个坑：
编译器宏**不等于** Kconfig 符号（`drv_spi_flash.c` 的 `#ifndef SOC_SF32LB55X`
就满足不了），而且 include 顺序你控制不住——报出来的是"缺寄存器映射"或
"未知类型名"，跟根因看着毫无关系。`example/test/cmock_unit/` 和
`example/test/hal_mock/` 都按这个规矩写，可以直接抄。

**注意**：这些板子只存在于 `--board=` 走的 KCONFIG_V2 流程里，不带 `--board=` 的
旧版 `sdk.py menuconfig` 看不到它们（v1 的芯片是 choice，与 `SOC_QEMU` 互斥）。

**为什么 QEMU 重要**：`BSP_USING_QEMU` 和 `pc` 模拟器一样会让 `drivers/SConscript`
**整树跳过 `drivers/`**。也就是说镜像里没有任何 HAL 实现，
一个 CMock 生成的 mock 就是它替换掉的那个符号的**唯一定义**——
这正是需求 3（打桩）需要的前提。详见 `middleware/test_fwk/mocks/README.md`。

**几个坑（都是静默失败）**：

- **命令写入必须逐字符限速**（后端默认 20 ms/字符）。整行一次写进去会丢字符，
  现象是命令回显正常、却始终不执行——因为丢的正好是结尾的回车换行。
- **行结尾要用 CRLF**。只发 `\r` 会被丢掉。
- 会话不会在 QEMU 启动后立刻交出去，而是**等设备启动完并且安静下来**才交接；
  否则命令发在一个 UART 还没使能的时刻，整条被丢掉，
  表现为等满超时后报 "no Unity summary line found"。
- 控制台**没有走中断**，是轮询的（已实测：使能 UART 接收中断并放开全部
  NVIC 线，没有任何设备中断产生）。所以别指望它反映真实的中断行为。
- `tools/qemu/` 下的 `qemu_run.*` 与 `qemu_smoke_test.py` 仍用旧的 semihosting 调用，
  **已过期**，照它们跑不会有输出。

## 5. 写一个 `testcase.yaml`

放在被测算例的目录里。**id 由"相对路径 + tests 键"自动生成**，所以不要自己拼。

```yaml
tests:
  - id: my_case
    description: 一句话说明这个用例验什么
    harness: console          # console | unity | pytest（shell 未实现）
    tags: [smoke]
    timeout: 60
    harness_config:
      ...
```

字段全集以 `tools/sdk_test/schema/testcase.schema.json` 为准，**它是唯一真源**：
写错字段名会被直接拒绝，不会静默忽略。

**两种写法都支持。** `tests:` 可以是列表（每条自带 `id`），也可以是映射（**键就是 id**）：

```yaml
tests:
  my_case:                   # 键即 id，不必再写一行 id:
    harness: console
    harness_config:
      record: ["Hello world!"]
  my_other_case: ~           # 空条目 = 只有 id，其余全走默认
```

**顶层还有 `common:`**：写在那里的一组值会合并进 `tests:` 的**每一条**，
而**每条自己的同名键优先**：

```yaml
common:
  tags: [smoke]
  timeout: 60
tests:
  - id: my_case              # 继承上面的 tags 和 timeout
    harness: console
```

其它可用字段：`platform_allow` / `platform_exclude` / `integration_platforms` /
`depends_on`（匹配板子 `features.yaml` 声明的实例类型，见第 8 节）/ `extra_args`（透传给 scons）/
`skip` / `build_only` / `project_dir`。

**这些字段的现状并不一样**，写之前值得知道——有几项 schema 接受、但没有代码读：

| 字段 | 现状 |
|---|---|
| `depends_on` | 生效。匹配板子 `features.yaml`，在**构建之前**过滤（阶段一）。不写就完全不读那个文件。 |
| `integration_platforms` | 生效，而且是**整个列表**：写三个就跑三次。它不是"首选平台"。 |
| `platform_allow` / `platform_exclude` | 生效；显式给了 `--platform` 时也仍然生效。 |
| `extra_args` | 生效，但透传给的是 **scons 构建**，不是运行。 |
| `build_only` / `skip` | 生效。`skip` 在**构建之前**判断，所以能省掉一次构建。 |
| `min_flash` / `min_ram` / `slow` / `type` / `dut` / `dut_client` | **已移除**：schema 不再接受，写了会被直接拒绝。它们曾经是"被解析、但没有代码读"的字段。删掉的理由、以及它们原本要做什么（资源上限、慢用例标记、多设备），记在 `tools/sdk_test/DESIGN.md` 的 TODO 一节 |
| `harness: shell` | schema 合法但**未实现**，选中会报错，不会静默通过。 |

### `project_dir`：testcase 与工程不在同一层

`testcase.yaml` 通常挨着工程、而不是在工程里。默认会按约定找
"`testcase.yaml` 所在目录" 或 "它的 `project/` 子目录"；
都不匹配时会在下面两层的范围内找唯一一个，找到多个会**报错而不是猜**。
要指别处就写 `project_dir`（相对 testcase 文件）：

```yaml
    project_dir: ../project     # example/get-started/hello_world/rtt/pytest/ 用这个
```

---

## 6. 三种 harness 怎么写

### `console` —— 设备不写测试代码，宿主匹配串口输出

```yaml
  - id: hello_world.console
    harness: console
    timeout: 60
    harness_config:
      record:
        - "- SiFli Corporation"     # 正则，按声明顺序匹配
        - "Hello world!"
        - "msh />"
```

- 匹配是**在累积缓冲上做的**，不是按行。`RT_CONSOLEBUF_SIZE=128` 会把长行拆开，
  且没有全局 console 锁，所以行间会插噪声——不用管，正常写就行。
- 各条目默认**按顺序**匹配（共用一个游标）。想把某个串降级成"出现在任意位置"，
  写 `ordered: false`。
- 重复 N 次：`{regex: "tick", repeat: 3}`；缺了会报 `(2/3)` 而不是只说"缺"。
- 跨行匹配：`{regex: "Hello world!", type: "multi_line"}`（换行会被折叠成空格，
  可用 `separator` 换别的）。
- 失败信息会点名**缺哪一条**，并附上设备实际输出了什么。

**什么时候用它**：断言是"若干串按序出现"。三行 YAML 就够，不需要任何依赖。

### `unity` —— 板上跑 Unity，宿主解析它自己的汇总

一条用例就长这样：

```yaml
  - id: unit_test.board
    harness: unity
    timeout: 120
    harness_config:
      autorun: true       # 固件上电自己把所有 suite 跑完，宿主只读串口
      # suite: lz4        # 可省。写了就只认领 lz4 的结果，见下
```

**一条用例 = 一次构建 + 一次烧录 + 一次启动。** 工程在 `main()` 里调了一次
`sifli_unity_autorun()`（`CONFIG_TEST_FWK_AUTORUN`，测试工程的 `proj.conf` 里已经
打开），所以板子一上电就把注册表里**每个** suite 跑一遍，结果全打在同一份转录里，
宿主只读串口——**你不用发任何命令，也不用操心发在什么时刻**。

**所以不要按 suite 拆条目。** 一个镜像里有哪几个 suite 是**构建事实**：
`src/SConscript` 已经把实际编译的 `test_*.c` 扫成了 `unity_suites_gen.json`。在这里
再列一遍就是同一个事实写两处；而拆成多条只会变成多烧几次、多启动几次。

#### 多条用例共用一个镜像时：`suite` 用来认领

一次自动运行把**所有** suite 的结果打进同一份转录，所以一条用例可能要声明自己是
**谁**：

```yaml
      suite: lz4          # 可省。写了就只认领 lz4 的结果
```

宿主按结果行 `<file>:<line>:<suite.case>:PASS|FAIL` 里 case 名前那段 suite 名分派。不写 `suite`
的用例对整份转录负责——**一个工程一条用例时就是这么写的**（真板例子见下）。

两条要记住的：

- **自己那个 suite 一条结果都没出现 → 失败。** 一份转录只有一行汇总
  （`<n> Tests <n> Failures <n> Ignored`——一把 `UnityBegin`/`UnityEnd` 包住整个
  循环，所以那一行属于**固件**，不属于任何单个 suite）。把固件那句整体 `OK` 交给
  一个什么都没跑的 suite，就是一个什么都证明不了的绿灯。
- **别的 suite 失败了不影响自己。** 真板上的实际形态就是这样：同一份转录里
  cjson 和 circular_buf 判 PASS、lz4 判 FAIL，互不牵连。

**两道防"空转"的闸**：没看到汇总行 → 失败；汇总说 0 个用例 → 失败。
（后者是最危险的假绿：链接了 Unity 但一个用例都没注册，会打印干净的 `OK`。）

> **为什么不让宿主发 `unity_run`**：没有可靠的时刻可发——启动 banner 只打印一次、
> 且在提示符之前；提示符的文本又跟着构建设置走。命令发早了就丢，症状是报
> `no Unity summary line found`，看起来像固件坏了。把决定交给固件，这个时机问题就
> 不存在了。完整论证见 `tools/sdk_test/DESIGN.md`。

#### 固件等命令时才需要 `prompt` / `cmd`

把 `CONFIG_TEST_FWK_AUTORUN` 关掉（`proj.conf` 里改成 `n`），板子就停在 shell 等
命令，宿主才需要那两个字段。手动单跑某个 suite 走的也是这条路：烧一次，然后敲
`unity_run lz4`。

**反过来也成立**：`autorun: true` 时再写 `cmd` 或 `prompt` 会被**当面拒收**——那条
路根本不读这两个键，而写了不生效的键在本框架里一律报错，不静默忽略。

```yaml
    harness_config:
      prompt: "msh />"    # 可省，默认 msh />。发命令前先在串口输出里等它出现；
                          # 写自己的字符串是替换它，不是叠加
      # cmd: unity_run    # 可省，默认 unity_run。等到之后要发出去的命令本身
```

`prompt` 回答**什么时候可以发**，`cmd` 回答**发什么**。两者都是**替换**语义：写了
自己的字符串，默认值就不再被用——不是"先等默认的、再等你写的"。要同时接受多个
标志，把正则或起来，例如 `prompt: "msh />|READY"`，语义是"出现任意一个就放行"，
也不是按先后。

`prompt` 这一步是承重的，不是保险：在手动复位的板子上，console 是在**下载刚写完、
板子还在启动**的时候就打开的，此时把命令发出去，板子根本收不到。两个默认值是配套
的——`cmd` 默认发 `unity_run`，而它是一条 **msh 命令**，能执行的前提就是 msh 已经在
跑；所以 `prompt` 默认等 `msh />`，等的正是这个前提。换成别的 shell，就把 `prompt`
改成固件自己打印的就绪标志。

等不到提示符**不算失败**：命令照发，由原有的检查去判。

`harness: unity` 不接受 `record`——判定权归 Unity，硬要写会报错并提示改用 `console`。
`autorun` 反过来是 unity 专属：`console` 和 `pytest` 从不读它，收到会报错。

#### 怎么在真板上跑

`example/test/unit_test/` 是现成的工程。一个 `testcase.yaml` **两条用例**——一条跑
真板、一条钉在 QEMU 上——共用同一份固件，而那份固件里有三个 suite（lz4、cjson、
circular_buf）。

**一条用例 = 一次构建 + 一次烧录 + 一次启动**，整个镜像的 suite 都在这一次里跑完。
所以不按 suite 拆条目：拆了就是三次烧录、三次启动。

```powershell
# 干跑：先确认规划出什么（不碰硬件）
sdk.py test plan -T example/test/unit_test -t unit_test.board -p sf32lb52-lcd_a128r16

# 真板。这块板的 reset.method 是 auto —— 下载自己会把板子带起来，不需要任何
# 额外动作。自动运行的结果从板子启动的第一个字节就开始了，所以那次启动必须发
# 生在串口打开之后，否则那份转录没人看见。
sdk.py test run  -T example/test/unit_test -t unit_test.board `
    -p sf32lb52-lcd_a128r16
```

**`unit_test.qemu` 那条钉在 `qemu_cortex_m33` 上**，这个 lab 没有 QEMU 平台，于是
`test plan` 会**整体中止**（它是严格的），`test run` 则把它记为 ERROR 后继续。所以
用 `-t unit_test.board` 挑出真板那条。

**通过时长这样**（一条用例，九个 case）：

```
=================== unit_test  (1 test(s): 1 passed) ====================
[PASS ] unit_test.board on sf32lb52-lcd_a128r16_01  (19.0s)
```

> 截至本文，实际是 **1 failed**：`lz4.boundaries` 还没结案，见
> `tools/sdk_test/DESIGN.md` 风险 9。上面是全绿时的形态。

失败时终端直接给出**哪条子用例**红了，不用翻日志——**下面这条是当前的真实输出**：

```
[FAIL ] unit_test.board on sf32lb52-lcd_a128r16_01  (19.0s)  at run
        1 of 9 Unity test(s) failed: lz4.boundaries
        FAIL  lz4.boundaries (test_lz4.c:129): Expected 1024 to be less than 0
```

报告里每条 case 的标识是 **`suite.case`**（例如 `lz4.boundaries`）：case 名在 suite
之间会重复（cjson 和 lz4 都有 `boundaries`），不带 suite 就没法区分——这也正是
`unity_run` 接受的过滤语法。括号里是真实源文件和断言触发的那一行，读起来就是
一个 `file:line` 位置。

### `pytest` —— 用例目录自带 Python，需要控制流时用它

```
example/get-started/hello_world/rtt/pytest/
  testcase.yaml
  test_hello_world.py
```

```yaml
  - id: hello_world.pytest
    harness: pytest
    timeout: 180
    project_dir: ../project
    harness_config:
      pytest_root: .              # 相对 testcase 文件
      # pytest_args: [-x, -k, "smoke"]
```

```python
def test_the_console_is_bidirectional(dut):
    dut.send_line("")                      # 先问
    assert dut.expect(r"msh />", timeout=10), "the shell did not answer"
```

`dut` fixture 由框架自动加载（无需 `conftest.py`），它接到的是
**runner 已经申请好的那个 testbed**，不是新申请一块板子。

可用方法：`expect(regex, timeout)`、`send_line()`、`write()`、`read()`、`drain()`、
`reset()`、`expect_unity_test_output(timeout)`，以及 `dut.testbed_id` / `dut.platform` /
`dut.build_dir`。

**`expect` 的两条语义**（都是承重的，写用例时要记着）：

- **游标只前进，跨用例延续。** `dut` 是 session 作用域的，所以"设备说了 X"
  指的是**从这条用例往后**说的。启动时留下的提示符**不会**满足后一条用例的
  `expect`——那是一个什么都证明不了的绿灯。想要新提示符就先 `send_line`。
- **同一次串口读取可能同时带来本条和下一条期望的文本。** 匹配是在累积缓冲上、
  从游标往后搜的，所以前一条不会把后一条的内容一起吃掉。别写成一条 `expect`
  只读一次就返回的实现——那是之前的一个真 bug：后一条会等满整个超时，
  报出来却是"设备什么都没说"，指向完全错误的方向。

**什么时候用**：断言依赖"先问再答"、要协调多块板、要驱动仪器、要重试。
纯断言场景用它只会更啰嗦，还要多装 pytest。

> `pytest` 不是框架的硬依赖：`console` 和 `unity` 以及框架自身的测试都是纯标准库。
> 装了 `install.ps1` 才会有 pytest（它已在 `tools/locks/default` 里）。

---

## 7. 写板上 Unity 单测

工程模板看 `example/test/unit_test/`。

```c
/* 一个用例就是一个函数 */
void test_lz4_roundtrip(void)
{
    TEST_ASSERT_EQUAL_INT(orig_len, LZ4_compress_default(src, dst, orig_len, cap));
}

/* 断言的写法见 Unity 文档；失败会带 file:line */
```

**注册表是生成的，不用手写。** `src/SConscript` 用 `test_registry.action(...)`
读这个工程**实际参与编译的** `test_*.c`，产出 `unity_suites_gen.c`
（`sifli_unity_cases[]` 数组——普通数组不是链接段，所以不需要改任何链接脚本）
和一份同名 `.json` manifest。

于是一个用例只声明一次：它的定义。suite 名和 case 名按下面的规则推出来，
只有想要的名字和推出来的不一样时才需要多写一行提示：

| 想要 | 怎么写 |
|---|---|
| suite 名 | 文件名取 `test_<suite>.c`；文件名不合适时在文件里写一行 `suite: <name>` |
| case 名 | 函数名去掉 `test_<suite>_`（`test_lz4_roundtrip` → `roundtrip`）；想要别的名字时，在函数上方的注释块里写一行 `case: <name>` |

```c
/* Malformed input has no partial success: the parser must report NULL.
 *
 * case: malformed
 */
void test_cjson_malformed_returns_null(void)
```

提示必须**独占一行**（写成 `/* case: malformed */` 也行），这样散文里出现的
"case:" 不会被当成提示，而且它只作用于紧跟在下面的那个函数。

**平台门控也是自动的**：某个 suite 的模块在这个载体上编不了，就在 `src/SConscript`
里 `SrcRemove` 掉它，它随之从注册表消失——不再需要在注册表里写一遍 `#if`。

`proj.conf` 里开：

```
CONFIG_RT_USING_UNITY=y
CONFIG_USING_TEST_FWK=y
```

设备侧命令：`unity_run [suite|case|suite.case]`、`unity_list`。

**注意 EOL**：`rt_kprintf` 会把 `\n` 展开成 `\r\n`，`middleware/test_fwk` 已处理，
宿主解析器也同时容忍 LF 和 CRLF。

### 7.1 打桩：CMock

> CMock 本身与 `mock_*.c/h` 生成物**已随本次提交入库**；下面这两个工程见下一笔。
> 而它们依赖的 `BSP_USING_QEMU`（第 4 节）也不在这批里，所以这些命令暂时跑不起来
> ——要等 QEMU 载体跟上。

要测的分支在台架上复现不出来时（"擦除失败""写超时""ID 不匹配"），把那个依赖换成
CMock 生成的 mock。两个现成工程可以抄：

| 工程 | 打什么 | 用例数 |
|---|---|---|
| `example/test/cmock_unit/` | 中间件函数（`circular_buf_*`、`ipc_hw_*`） | 5 |
| `example/test/hal_mock/` | **HAL 函数**（`HAL_QSPI_GET_SR`） | 3 |

```bash
sdk.py test run -T example/test/cmock_unit
sdk.py test run -T example/test/hal_mock
```

两条都要用**带芯片身份的板子**（见 4.1），因为被测模块会 include `bf0_hal.h`。

**机制是"不编译被 mock 的实现"，不是拦截**：`BSP_USING_QEMU` 让
`drivers/SConscript` 整树跳过 `drivers/`，镜像里没有任何 HAL 实现，
所以 mock 就是那个符号的唯一定义——不重复、不靠 weak、不碰 ROM 表。

三个实际会咬人的点：

1. **mock 的体积**。CMock 没有"只生成某几个函数"的开关，只能整头文件生成。
   `bf0_hal_mpi_ex.h` 声明了 593 个 API，生成物 588 KB，而用例只用 1 个。
   这是刻意的取舍（生成物入库、从不手改），但要心里有数。
2. **`GlobalExpectCount` / `GlobalVerifyOrder` 要测试工程自己定义**。开了
   `:enforce_strict_ordering` 后，生成的 mock 只 `extern` 声明并赋值它们、从不定义。
   链接错误只说符号名，很容易误判成缺库。
3. **`_CallCount()` 不是调用次数**，是 callback 调用计数，普通 `_ExpectAndReturn` 下恒为 0。
   要证明"被调用过"，靠 `_Verify()` 消费期望（tearDown 里调），加上
   `:fail_on_unexpected_calls`。

**写完一定要做反向验证**：故意让 mock 的期望不匹配，用例必须 FAIL。
不做这一步就证明不了 mock 真被链接进去——如果链进去的是真实实现，改 mock 不会有任何反应。

细节（生成器怎么跑、`:strippables` 怎么写、芯片身份为什么必须来自板子、
链接期闭包为什么只有 2 个符号）见 `middleware/test_fwk/mocks/README.md`。

---

## 8. binding 文件：描述一个 lab（硬件团队的）

**这个文件通常不是你写的。** 它描述那台机器上插了哪些板、各自的串口和接线——那是
维护实验室的人的活。使用者只需要知道两件事：它的存在，以及**里面 `platform` 的名字
要和 `-p` 给的一致**；不想碰它，就用 `-p` 直接跑（见第 3 节）。

```yaml
testbed:
  - id: sf32lb52-lcd_n16r8_01
    platform: sf32lb52-lcd_n16r8      # 必须对应 customer/boards/<name>
    runner: local                     # 由哪台 Lab Agent 持有
    console:
      port: COM13
      # baud 默认 1000000，即 SDK console 的波特率
    connections:                       # 对应 testcase 的 harness_config.fixture
      - fixture: spi_nor_flash:spi1
        pinmap:
          - {pin: PA00, signal: CS}
```

**板子能做什么不写在这里。** `depends_on` 匹配的是**板子自己的声明**——
`customer/boards/<board>/<core>/features.yaml`，那里写这块板有哪些总线实例
（`spi1`、`i2c2`；空列表也是合法声明，QEMU 载体就是空的）。binding 只说
**接了什么东西**。同一个事实写两处，正是它们会互相矛盾的来由。

```yaml
# testcase.yaml：要什么
    depends_on: [spi]                  # “要有 spi”——不指定是哪条总线
    harness_config:
      fixture: spi_nor_flash           # “板上挂了 NOR flash”——不指定接在哪条
```

用例写**类型名**，板子写**实例名**，类型匹配接受该类型的任一实例；写成实例名
（`depends_on: [spi2]`、`fixture: spi_nor_flash:spi2`）就是精确锚定。
两条轴用的是同一条规则——因为哪条总线接了什么，是板子的事，用例自己的代码
（`BSP_USING_SPI1` 之类）已经解决过了。

**同名 fixture = 等价**：若干 testbed 提供同一个 fixture 时，用例落到哪块都必须得出同样结论。
如果两块板不满足这一点，它们就不该声明同一个 fixture。

Lab Agent 自己的设置（探针序列号、烧录方式）也写在这个文件里，
整个文件都由 agent 读，没有哪一部分流到 SDK 侧：

```yaml
    jlink:
      # device: SF32LB52X_NOR     # 不写则从 customer/boards/<platform>/hcpu/rtconfig.py 读
      serial: "59605049"          # 多探针时用 -SelectEmuBySN 选
      # wdt_script: tools/segger/halt_all_cpu_and_disable_all_wdt_56x.jlink
    flash:
      method: jlink               # jlink（探针）| sftool（UART）
    reset:
      method: jlink               # jlink（探针，默认）| auto（下载自己带起来）| manual（可能要人按）
```

### 8.1 字段速查

**如果你就是要写这个文件（比如你就是硬件团队），必须写对的只有两个**：`id` 和
`platform`。其余要么是 agent 自己的，要么读了也不影响行为——表里都标了，照着抄不会
写错。

**使用者的对应关系只有一条**：你 `-p` 给的那个名字，要等于某个 testbed 的
`platform`。

| 键 | 谁读 | 说明 |
|---|---|---|
| `testbed[].id` | agent | 这个 testbed 在本次运行里的标识。同一个文件里不能重复 |
| `testbed[].platform` | **agent** | **决定这条用例能不能落在这块板**。必须与 `customer/boards/<name>` 同名；不带核后缀即可，会补成 `_hcpu` |
| `testbed[].console` | **agent** | `{port, baud}`。`baud` 默认 1000000（SDK console 固定 1 Mbaud） |
| `testbed[].connections[].fixture` | agent | **只当用例写了 `harness_config.fixture` 时才会被查**。写 `name`，或 `name:param`（`spi_nor_flash:spi2`）精确锚定 |
| `testbed[].connections[].pinmap` | 无 | 记录怎么接的线（哪个脚是 CS/CLK）。**目前没有代码消费**，写它是给硬件团队对线用的 |
| `testbed[].connections[].mux` | agent（仅校验） | 形如 `mux_01.channel.3`；只用于检查它引用的 mux 确实在 `muxes:` 里声明过 |
| `testbed[].muxes[]` | agent（仅 `id`） | 带外开关（relay 等）的声明。只有 `id` 被读（供 `connections[].mux` 引用）；`type` / `channel_num` / `runner` 目前没有代码读，留给以后驱动 relay 的 fixture driver |
| `testbed[].runner` | agent（仅上报） | 这个 testbed 由哪台 Lab Agent 持有。会在 `/testbeds` 里报出去，**但不参与分配决策** |
| `testbed[].jlink` / `flash` / `reset` | **只有 agent** | 探针、烧录、复位。SDK 侧根本看不到这个文件——**所以 binding 里出现探针序列号不是失误**，它就是 agent 自己的配置 |
| `runner:`（顶层） | **只有 agent** | 机器级设置（监听地址、轮询间隔、缓冲上限、探针路径）。**SDK 侧从不读它们** |

> **一份文件一个读者**：`python tools/lab_agent/runner.py -c <f>` 读它的全部——
> `testbed:` 那部分和机器级的 `runner:` 块都读。所以同一份文件既是这个 lab 的硬件
> 映射、也是那台机器的 runner 配置。**真实的 COM 口不要提交进仓库。**

`jlink:` / `flash:` / `reset:` 各有哪些子键，见带注释的模板
`tools/lab_agent/examples/runner.local.yaml`——那份就是给人照着抄的。

### 三种复位模式：`jlink` / `auto` / `manual`

`reset:` 说的不只是"谁去按复位"，还有**这次启动有没有人担保**：

| `reset.method` | 谁复位 | 停下来问人 | 没开 `autorun` 的 `unity` |
|---|---|---|---|
| `jlink`（默认） | 探针 | 不 | ✅ |
| `auto` | 没有人——也不需要：下载结束跑起新镜像，那就是那次复位 | 不 | ✅ |
| `manual` | 没有人，**也不保证**（可能要人按按钮） | 要（有终端时） | ❌ 在下载**之前**就拒绝 |

`auto` 和 `manual` 在框架这一侧都是 no-op，区别只在承诺：**启动是不是必然发生**。
而这一条恰好就是"要不要问人"和"写型 harness 放不放行"的全部依据。

```yaml
    reset:
      method: auto      # 下载可靠地会把板子带起来（UART 下载结束时 ROM 跳到新镜像，
                        # 开 console 时 DTR/RTS 往往再复位一次）
      # method: manual  # 不能保证，靠人按
```

**`auto` 什么也不打印。** `[reset]` 那行是在**对操作员说话**（"按复位，然后回车"），
`auto` 没有操作员可交代——`jlink` 同样一声不吭。

#### `manual`：有终端时会停下来确认一次

```powershell
sdk.py test run ...        # 在终端里跑
```

console 打开后：

```
[reset] sf32lb52-lcd_a128r16_01 (sf32lb52-lcd_a128r16) has no probe.
        Press the board's RESET button if it has not already booted,
        then press Enter to run the test.
  continue? [Y/n]
```

**多半不用按。** UART 下载（`flash.method: sftool`）结束时 ROM 会跳到新镜像——
板子已经自己启动了一次；打开 console 口时 pyserial 又会驱动 DTR/RTS，
可能再复位一次。所以通常敲个回车就行，这句话只是兜底。
**如果这种情况每次都成立，就该把这块板的 `reset.method` 写成 `auto`**——
同一件事，把"问"去掉。

回答 `n`（或 Ctrl-D）这个用例记为 **skipped**，不是 failed——用例根本没跑，
说它坏了是没有依据的断言。

这一次回车不只是礼节：`manual` 板上**没有别的东西能让板子起来**，
所以那次按键就是复位本身。这也正是"有人守着"能让写型 harness 在此跑起来的原因。

#### `manual` 且没人可问（`--no-confirm`，或无终端）

不阻塞，只打一行提示，窗口是用例自己的 `timeout`（默认 60s）：

```
[reset] sf32lb52-lcd_a128r16_01 (sf32lb52-lcd_a128r16) has no probe -
waiting for the board to boot; press its RESET button if nothing appears (60s)
```

这时"先写后读"的 harness 会被拒——也就是**没开 `autorun` 的 `unity`**，
拒绝发生在**下载之前**。

#### `unity` 为什么能在这块板上跑

**因为它不写命令了。** 没开 `autorun` 时，`unity_run` 只在 shell 起来之后才有效，
发早了就丢——症状是耗完超时后报 `no Unity summary line found`，**看起来像固件坏了**。
自动运行之后固件自己把结果打完，宿主只读，这个时机问题就不存在了。

**这块板上真正要防的是另一件事**：自动运行的结果从**板子启动的第一个字节**就开始，
所以那次启动必须发生在**串口打开之后**。console 是烧录完成后才打开的，而打开串口时
的 DTR/RTS 往往会把板子再复位一次——启动日志通常早就躺在缓冲里了。那道"需要的话按
复位，然后回车"的确认就是为这件事留的兜底。

（关掉 `autorun`、回到发命令那条路时，宿主会先在串口输出里等 shell 提示符
（默认 `msh />`）——那是"可以发了吗"的正向信号。**不是"等串口安静"**：板子还没开始
启动时也是安静的，我们试过，"静默"被当成"已经说完"，命令照样发丢。等不到提示符
不算失败，命令照发，由原有的检查去判。）

#### 三条要注意的

- **无终端时，通过不代表有人按过**。绿灯只说明"console 打开后串口上出现了期望的内容"——
  这次启动可能来自下载、来自开端口、也可能来自人。框架区分不了，也没假装能区分。
  有终端时那个回车至少表示有人看过。
- **跑不了 `wdt_script`**：看门狗不会被关。用例超时长的话板子可能被看门狗中途复位。
- **按晚了会污染下一个用例**：在上一个用例 `release` 之后才按，启动日志会落在
  下一个用例刚打开的缓冲里——探针复位不会有这个问题。

模板：`tools/sdk_test/examples/binding.local.yaml`（硬件映射）
和 `tools/lab_agent/examples/runner.local.yaml`（机器侧，多了 `runner:` 全局块）。
**真实的 COM 口不要提交。**

---

## 9. 接进 CI

报告默认产出，所以脚本里不用再写 `--report`；日志和报告都在本次 run 的时间戳目录里，
而每次失败要看的 `device.log` / `runner.log` 作为普通 artifact 一起收（`when: always`，否则失败时反而拿不到）。
`test list --json` 可以用来生成矩阵（避免把板子名写死在 CI 配置里）。

**CI 里要用 `--report-dir` 把报告钉到固定路径**——目录带时间戳之后 `test-out/sifli.xml` 这个路径就不存在了。
日志仍然按 run 分目录（artifact 的 glob 是递归的，不受影响）：

```yaml
test:
  script:
    - sdk.py test run -T example -p $PLATFORM --outdir test-out --report-dir test-out/reports
  artifacts:
    when: always
    reports:
      junit: test-out/reports/sifli.xml
    paths:
      - test-out/**/*.log
      - test-out/**/device.log
```

`--report-suffix` 是给并行/多次运行区分报告文件名用的（twister 同款）；
不带它时报告每次覆盖 `test-out/reports/` 下的同名文件——那是有意的，CI 只关心最后一次。
`test-out/` 已在 `.gitignore` 里。

（本次只留了钩子，没有启用 CI stage。）

---

## 10. 测试数据放哪

**测试代码与被测代码同仓库**（同一次 review 要能一起看）；
**测试数据走外部存储**，两者用同一个机制表达：**向量就是一个 fixture**。

```yaml
    harness_config:
      fixture: audio_sample_16k     # 只声明能力，不写路径
```

测试不知道路径，所以同一份用例在有缓存的笔记本和对着远端存储的 CI 上都能跑。
本仓库已有 `jfrog.sifli.com/artifactory`（conan remote）可以承载。

**不要往 git 里提交大向量**：git 历史只增不减，一个 200 MB 的文件提交一次，
就永久留在**每一个** clone 里，之后删掉也没用。

> 把 `fixture:` 解析成"下载 + 校验哈希 + 落本地缓存"的 resolver **尚未实现**，
> 接口留在 `tools/lab_agent/fixture_drivers/`。
> 目前所有验收用例都不需要向量，所以还没有紧迫性。

---

## 11. 已知边界与排错

**未实现**

| 项 | 状态 |
|---|---|
| `harness: shell` | 会明确报错，不会静默空转 |
| `backends/pc_sim.py` | stub（`pc` 模拟器仍是设计态） |
| 多设备调度（需求 4） | schema 与插件接口已支持，实际调度未做 |
| 继电器 / 电源控制 | `transport/relay.py` 只有接口，会抛 `RelayNotAvailable` |
| `backends/qemu.py` | **已实现**：QEMU 载体可跑 console/unity/pytest，见第 4 节 |
| CMock 打桩用例 | **已交付**两个工程（中间件 + HAL），见第 7.1 节与 `middleware/test_fwk/mocks/README.md` |
| 每实例 `build.log` | **已实现**：编译器输出落盘，不再刷控制台 |

**排错**

- `Cannot reach the Lab Agent` → 终端 A 没起，或 `--agent-url` 不对。
- `every testbed matching the request is busy` → 有人占着；Lab Agent 自己的
  `python tools/lab_agent/runner.py -c <cfg> --list` 会标出哪些 testbed 正忙。
- `missing fixture(s) [...] (testbed offers: ...)` → 用例要的能力这个 lab 没有，
  信息里两侧都列了，直接对。
- 构建失败被记为该用例的 ERROR，**不会**中断整轮——所以一轮跑完能一次看到所有问题。
  **`at build` 就是它**：终端与产物的 `stage` 字段直接告诉你失败在哪个阶段（`plan`/`build`/`open`/`run`/`close`），
  不用猜。编译器的原话在同目录的 **`build.log`** 里（失败信息也会指向它）。
- 串口打不开：Lab Agent 侧报端口被占用——检查是不是有别的终端工具连着同一块板子。
- 日志里看到 `console: N bytes retained, M trimmed from the head` → 串口原文超过了后端 4 MB 的
  保留窗口，**头部被裁掉了**。这条只在实例 `runner.log` 的表头里，别把被裁过的 `device.log` 当全量。

**两条必须记住的**

1. **期望字符串不要从源码推导**，要固化在 `testcase.yaml` 里并跟实际输出对齐。
2. **上板单测不要用 mock**：真板卡的结果上报要走串口 console，而"不编译 `drivers/hal`"
   会把时钟/UART/MPI 一起拿掉。打桩只能落在非板卡载体上。
