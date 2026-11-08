# SDK 自动化测试框架 — 实现进度与上手

这是 `SDK自动化测试.docx` 那套自动化测试框架的 SDK 侧实现。

**设计文档（先读这个）**：[`DESIGN.md`](DESIGN.md)
—— 里面有已确认的决策、Tier A/B 分层模型、已核实的事实清单、MVP 边界、验证方式与风险。
本文件只讲**代码现状、怎么跑、下一步做什么**。

---

## 一句话现状

**Tier A 全链路已打通并通过自测**（发现 / 能力匹配 / 构建 / 申请硬件 / 烧录 / 复位 /
四种 harness 中的三种 / JUnit 上报 / `sdk.py test` 入口）。
**Tier B（CMock 打桩）的库、mock 生成物与 QEMU 载体已就绪；两个示例工程不在本分支** —— 见文末"未完成"。

框架自测：`tools/sdk_test/tests/` 与 `tools/lab_agent/tests/` **全绿**，都不需要硬件。
（不写具体数字——它每加一个用例就过期一次，跑一遍比读一行准。）

---

## 目录

| 位置 | 作用 |
|---|---|
| `tools/sdk_test/` | SDK 侧（无硬件绑定）：发现、匹配、编排、上报 |
| `tools/lab_agent/` | 机器侧：持有真实硬件，通过本机 HTTP 发放 testbed |
| `tools/sdk_py_actions/test_ext.py` | `sdk.py test` 命令入口（薄转发） |
| `middleware/test_fwk/` | 板上胶水：Unity 配置 + 自动运行（`sifli_unity_autorun()`，`TEST_FWK_AUTORUN`）+ `unity_run`/`unity_list` MSH 命令 |
| `external/unity/` | Unity 2.7.0（已裁剪，仅保留编译所需） |
| `external/cmock/` | CMock 2.7.0（**故意不接入构建**，见 `NOTES.md`） |
| `example/test/unit_test/` | 板上 Unity 单测工程（lz4 / cJSON / circular_buf） |
| `example/get-started/hello_world/rtt/testcase.yaml` | console harness 端到端验收 |
| `example/get-started/hello_world/rtt/pytest/` | pytest harness 端到端验收 |
| `example/get-started/hello_world/no-os/testcase.yaml` | 同一示例的裸机版：没有 RT-Thread，所以没有 banner 也没有 shell，期望串只有应用自己那一行 |

---

## 怎么跑

### 1. 框架自测（不需要硬件，也不需要启动 Lab Agent）

```bash
./export.ps1                       # 或 source ./export.sh
python -m unittest discover -s tools/sdk_test/tests -p "test_*.py"
python -m unittest discover -s tools/lab_agent/tests -p "test_*.py"
```

也可用 pytest 收集（两者兼容）：`python -m pytest tools/sdk_test/tests -v`。

### 2. 看有哪些用例

```bash
sdk.py test list -T example
sdk.py test list -T example --json          # 给 CI 生成矩阵用
sdk.py test info                            # 本 build 支持哪些 harness / backend
```

### 3. 端到端（需要真实硬件）

```bash
# 终端 A：在有板子的机器上起 Lab Agent
python tools/lab_agent/runner.py -c tools/lab_agent/examples/runner.local.yaml
python tools/lab_agent/runner.py -c <cfg> --check     # 只校验配置
python tools/lab_agent/runner.py -c <cfg> --list      # 只列出 testbed

# 终端 B：跑用例
sdk.py test plan -T example/get-started/hello_world -p <platform>   # 先干跑看计划
sdk.py test run  -T example/get-started/hello_world -p <platform>
```

`test run` 退出码：`0` 全过，`1` 有用例失败，`2` 请求本身不可用。

### 4. 构建固件

按 `skills/sifli-build-win/SKILL.md`：PowerShell + `export.ps1`，然后
`scons --board=<board> -j8`。`sdk.py test run` 内部就是这么调的。

---

## 实现要点（读代码前值得先知道的几个决定）

### 串口输出为什么必须"累积缓冲 + 跨块正则"

`RT_CONSOLEBUF_SIZE=128` 会把长行拆开，且没有全局 console 锁。
所以 `harness/console.py` 维护一个累积缓冲，游标只前进不回退——**已用 7 字节切片实测通过**。

### `ordered` 是跨条目生效的（本次修好）

`record:` 里各条目默认按声明顺序、共用一个游标匹配。修之前每条目各持游标，
等于"顺序"形同虚设：日志乱序也会判定通过。`test_end_to_end.py` 里有反例钉住这一点。

### CRLF 必须容忍（本次修好，且曾经静默出错）

`rt_kprintf` 经 `_serial_poll_tx` 会把每个 `\n` 展开成 `\r\n`。
旧 `CASE_RE` 的 `$` 锚点匹配不到 `\r` 前，结果是 **`PASS` 行被静默丢弃、
只有 `FAIL` 行能匹配上**（因为 `.*` 会吞掉 `\r`）——全绿的运行看起来像"什么都没跑"。
现在 `\r?$`，并有专门的反例用例。

### 复位必须在 console 打开**之后**

`backends/board.py` 的顺序是：申请 → 烧录 → **开 console** → 复位。
反过来会让启动日志和串口打开竞争，assert 启动 banner 的用例会偶发失败——
这是嵌入式测试"flaky"最经典的来源。

### 一个用例挂掉不能拖垮整轮

`runner.plan()` 严格（干跑就该报错），但 `runner.run()` 逐个用例规划：
某个用例没有可用硬件时记为 ERROR 并继续跑其余用例。
否则一个配错的 testcase 会让整轮 nightly 失去诊断价值。

### 测试向量不要进 git

测试**代码**与被测代码同仓库（同一次 review 能一起看）；测试**数据**走外部存储。
两者用**同一个机制**表达：向量就是一个 fixture（`fixture: audio_sample_16k`），
测试只声明能力，binding 说字节在哪。

原因：git 历史只增不减，一个 200 MB 的向量提交一次就永久留在**每个** clone 里，删掉也没用。
本仓库已有 `jfrog.sifli.com/artifactory`（conan remote），向量放那里即可。

**未实现**：把 `fixture: audio_sample_16k` 解析成"下载 + 校验哈希 + 落到本地缓存"的
*resolver*。接口已经留好（`lab_agent/fixture_drivers/`），是个增量而非返工。

---

## 未完成（明确的下一步）

1. ~~**Tier B / CMock 验收用例 —— 未交付。**~~ **设计记录为已交付两个工程；当前分支尚未包含。**
   下面两个工程不在本分支的 `example/test/` 里（现有：`bt_peer`、`hal_real`、`unit_test`）。
   当初记的两处环境阻塞都已不是阻塞：Ruby 装在 `C:\Ruby33-x64`（不在 PATH 上，
   要用绝对路径调）；`pc` 板也不再是唯一出路——载体换成了带芯片身份的 QEMU 板。

   | 工程 | 打什么 | 用例数 |
   |---|---|---|
   | `example/test/cmock_unit/` | 中间件（`circular_buf_*`、`ipc_hw_*`） | 5 |
   | `example/test/hal_mock/` | **HAL 函数**（`HAL_QSPI_GET_SR`） | 3 |

   两者都做了反向验证。当初认定的最佳打桩目标
   （`drv_spi_flash.c:667` 的 `rt_flash_get_status_register()`）就是 `hal_mock` 用的那个，
   实测闭包确实只有它和 `HAL_QSPI_GET_SR`——**旧记录里"~65 个符号"是估的上限，
   不是实测**：1908 行的文件 `nm -u` 出 35 个，但 `--gc-sections` 之后只剩 2 个。

   ROM 符号表那条注意仍然值得留着，但**在这个载体上不适用**：挂 `-T sifli_rom_*.sym`
   的逻辑本身就在 `drivers/` 整树里，被 `SOC_QEMU` 跳过，
   所以 QEMU 载体上永远不会挂（构建产物里搜 `sifli_rom`：0 处）。

   机制、踩到的点与"芯片身份必须来自板子"的完整推理见
   `middleware/test_fwk/mocks/README.md`。

2. **`harness: shell`** 仍是设计态（会明确 raise，不会静默空转）。

3. **`backends/pc_sim.py`** 是 stub，抛 `NotImplementedError`。
   **`backends/qemu.py` 已实现**，能跑 `console` 和 `unity`，不需要 Lab Agent；
   它要驱动的载体（`customer/boards/qemu_*`、`drivers_qemu/`、`tools/qemu/`、
   `example/qemu/`）也已在树内，所以下面这两条现在可以跑：

   ```bash
   sdk.py test run -T example/qemu
   sdk.py test run -T example/test/unit_test --no-build -t unit_test.qemu
   ```

   载体落地后 Tier B 就不再是"只能 Windows"。几个坑（都是静默失败）写在
   `backends/qemu.py` 的注释里：命令要逐字符限速、行结尾要 CRLF、
   会话要等设备启动完再交接。

4. **需求 4（多设备）** 未实现。原来为它预留的 `dut` / `dut_client` 字段已从 schema
   删掉（理由是没人读），设计仍留在 `DESIGN.md` 的需求 4 那节与文末的 TODO 里。

5. **requirements 的说明是假的（本次修正了自己的那份）。**
   `install.* --enable-<feature>` 这个机制**已经在 commit 8259175ea 里被移除**，
   依赖现在全部来自 `tools/locks/<profile>/uv.lock` 这一个锁定环境。
   其余几个 `requirements.*.txt` 顶部的 `--enable-...` 说明同样已过期。
   pytest 已加入 `tools/locks/default/pyproject.toml` 并重新 `uv lock`，
   所以 `install.ps1` 现在会自动拉取 pytest。

6. **构建会往源码树里丢 `.o`。** SCons 对 app 源码树之外的源文件是**就地**编译的，
   所以构建后 `external/lz4/lz4.o`、`middleware/ipc_queue/common/circular_buf.o`
   会出现在源码旁边。建议加一条根级 `*.o` 的 ignore。

---

## 两个必须记住的坑

1. **`hello_world` 的期望字符串不要从源码推导。** 期望串固化在
   `example/get-started/hello_world/rtt/testcase.yaml` 里，与 `main.c` 的实际输出对齐后才写死。

2. **不要给参与板级 bring-up 的依赖打桩。** 真板卡的结果上报要走串口 console
   （→ `drv_usart.c` → `bf0_hal_usart.c`），而"不编译 `drivers/hal`"会把时钟/UART/MPI
   一起拿掉，板子既起不来也没有上报通道。这类依赖的用例放 QEMU 载体；其他可隔离的依赖
   在真板上按 guide §7.3 的表处理。详见设计文档的分层模型与需求 3 章节。
