# BT 测试仪表（bt_peer）设计

## 1. 为什么改

当前的 `ble_peer` 不是一个"仪表"抽象，而是"某一份例程固件 + 针对它日志文本的解析器"：

- `tools/sdk_test/ble_peer.py` 明确假设仪表运行 `example/ble/central_and_peripheral`，
  连该例程的 `sifli_app` 128-bit 服务 UUID 都硬编码在 `SERVICE_UUID_BYTES` 里；
- `tools/sdk_test/pytest_plugin.py` 只定义了 `ble_peer` 一个仪表 fixture，
  并直接 `import BlePeer`；
- 断言建立在 ulog 文本上（连 `"deivce"`、`"Charateristic"` 这类拼写都必须兼容），
  hexdump 是否收全靠 "quiet window" 猜。

后果是：只能测 `central_and_peripheral` 一个例程；换一个例程要么换仪表固件，
要么改框架解析器。本设计把仪表拆成三层——**专用对端固件、稳定行协议、
分层的宿主 API**——并用它覆盖 `example/ble` 下 server 类与双角色例程。
同一套传输/协议/注册表以后可以直接扩展到 `example/bt` 的经典蓝牙。

## 2. 术语

| 术语 | 含义 |
|---|---|
| **SDK 侧 / 宿主** | 跑在 PC 上的 Python 进程：`tools/sdk_test` + pytest。本文说"宿主 API"都指这里，不是 Lab Agent，也不是板上的固件。 |
| **Lab Agent** | 插着板子的那台机器上的常驻服务，负责占用、烧录、复位、开串口。 |
| **仪表 / peer** | 第二块板及其固件。本文统一叫 `bt_peer`。 |
| **DUT** | 被测板。 |
| **能力** | 仪表能扮演的角色，如 `ble`、`classic`。fixture 参数表达能力。 |

命名约定（已确认）：

- 固件工程：`example/test/bt_peer/`
- fixture 名：`bt_peer`，能力用参数区分：
  - `bt_peer:ble` —— 本期实现；
  - `bt_peer:classic` —— 以后加经典蓝牙。
- 宿主包：`tools/sdk_test/instruments/`

fixture 名描述的是**能力**，不是物理设备；一个物理仪表可以同时提供多种能力，
由 binding 把能力映射到物理端口。

## 3. 范围

### 3.1 本期实现

BLE：

- GATT server 类：`basc`、`cscpc`、`hrpc`、`cppc`、`diss`、`tip_cts`、
  `peripheral`、`file_transfer`
- 广播类：`iBeacon`、`periodic_adv`
- 双角色：`throughput`、`multi_connection`、`central_and_peripheral`、`pair`、
  `periodic_adv_sync`
- `hid` 单独处理（16-bit HID 服务）

### 3.2 不在本期

- `ancs` / `ams`：需要模拟 iPhone 的 ANCS/AMS 服务
- `talkback`：音频通路
- `lcpu_general`：小核 controller 工程；`AddLCPU` 直接引用现成镜像，不改它
- OTA 专项：`*_with_ota` 只覆盖普通 GATT 部分
- Q2 的通用仪器抽象与电流表

## 4. 架构

```
+---------------------------- PC（SDK 侧 / 宿主） ----------------------------+
| pytest suite                                                                |
|   bt_peer fixture  ->  BtPeer                                               |
|                          +-- .ble     -> BleInstrument                      |
|                          +-- .classic -> BtClassicInstrument（宿主骨架已提供）|
|                                                                            |
|   InstrumentConsole：行协议、请求/响应关联、事件队列、迟到响应丢弃          |
+------------------------------------|---------------------------------------+
                                     | HTTP（现有 console 路由，stream=bt_peer:ble）
+---------------------------- Lab Agent -------------------------------------+
| ConsoleStream（文本 + 单调 cursor，不改）                                   |
+------------------------------------|---------------------------------------+
                                     | UART 1 Mbaud
+---------------------------- 仪表板（bt_peer 固件） -------------------------+
| peer_proto.c    行协议解析/输出                                             |
| peer_ble.c      GAP central + peripheral                                    |
| peer_gatt_client.c  discovery/read/write/subscribe/MTU                      |
| peer_gatt_server.c  loopback/echo 服务                                      |
| peer_sec.c      bond/passkey/IO capability                                  |
| peer_periodic.c periodic advertiser / synchronizer                          |
+----------------------------------------------------------------------------+
```

Lab Agent、binding、fixture 匹配、日志回收都不需要新机制。

## 5. 固件

### 5.1 工程布局

```
example/test/bt_peer/
  README.md
  testcase.yaml            # 可选：build_only，让 CI 只编译仪表固件
  project/
    SConstruct             # AddBootLoader / AddLCPU / SifliEnv / DoBuilding / AddFTAB
    SConscript             # SDK + LCPU patch + ../src/SConscript
    Kconfig                # source "$SIFLI_SDK/Kconfig.v2" + rsource Kconfig.proj
    Kconfig.proj           # BF0_HCPU + CUSTOM_MEM_MAP
    proj.conf              # BLUETOOTH + ULOG + FINSH/MSH + 缓冲
    rtconfig.py            # 必须存在；空文件即可
    board/
      SConscript
      board.c
      bf0_ap_hal_msp.c
  src/
    SConscript
    main.c
    peer_proto.c/.h
    peer_ble.c/.h
    peer_gatt_client.c
    peer_gatt_server.c
    peer_sec.c
    peer_periodic.c
```

- 扁平 `project/` 布局即可；只有同时要建 ACPU 子工程（58X）时才拆
  `project/hcpu` + `project/acpu`。
- BLE 栈不在 `src/SConscript` 里列源码：`SConscript($SIFLI_SDK/SConscript)`
  走 SDK 全树发现，`middleware/bluetooth/SConscript` 由
  `GetDepend('BLUETOOTH')` 控制，链接 `lib_bt_gcc`。
- 小核 controller 不改：`AddLCPU(...)` 直接引用 `example/rom_bin/lcpu_general_ble_img`
  里的现成镜像。
- 不要照抄多数 BLE 例程里的 `cwd + '../../src/SConscript'`；扁平布局用
  `cwd + '/../src/SConscript'`。

### 5.2 关键配置

```conf
CONFIG_BLUETOOTH=y
CONFIG_RT_USING_ULOG=y
CONFIG_ULOG_USING_ISR_LOG=y
CONFIG_ULOG_OUTPUT_THREAD_NAME=y
CONFIG_BSP_USING_FULL_ASSERT=y
CONFIG_PKG_USING_FLASHDB=y

CONFIG_RT_USING_FINSH=y
CONFIG_FINSH_USING_MSH=y
CONFIG_FINSH_CMD_SIZE=1024
CONFIG_RT_CONSOLEBUF_SIZE=1024
```

两个缓冲缺一不可：

- `RT_CONSOLEBUF_SIZE` 太小会把长行拆开；
- `FINSH_CMD_SIZE` 默认 80，长 hex 参数会被截断。

`FINSH_ARG_MAX` 默认 10；本协议每条命令控制在 10 个参数以内。

### 5.3 行协议

宿主发送：

```
peer info
peer req <id> <op> [key=value ...]
peer data <id> <hex>            # 超大负载的续传（可选，见 5.5）
```

设备输出：

```
@info proto=1 fw=<hash> caps=ble roles=central,peripheral max_conn=8 ble=5.2
@ok  <id> [key=value ...]
@err <id> <code> <text>
@evt <seq> <name> [key=value ...]
```

规则：

1. 每个请求恰好一个终态响应（`@ok` 或 `@err`），用 `id` 关联。
2. 二进制统一 `len=N data=<hex>`；宿主等满 N 字节，不需要 quiet window。
3. 异步事件带单调 `seq`；非 `@` 行（ulog、提示符）一律忽略。
4. 超时后迟到的响应按 id 丢弃并计数（`late_responses`），不能匹配到下一次请求。
5. 一条 `@` 响应必须是单次、加锁的 `rt_kprintf`，不允许跨线程拼行。
6. `gatt.discover` 先发完所有 `@evt char`，最后才发终态 `@ok count=N`。
7. 同一 link 同时只允许一个未完成的 GATT 操作；第二个请求回
   `@err <id> busy`。

### 5.4 第一版命令与事件

命令：

```
state.reset
gap.scan            duration_ms=.. interval_ms=.. window_ms=.. dup=0/1 rssi_min=..
gap.scan.stop
gap.connect         index=<scan idx> | addr=<hex> addr_type=0/1 timeout_ms=.. interval_ms=..
gap.disconnect      link=<n>
gap.conn_params     link=<n> interval_min_ms=.. interval_max_ms=.. latency=.. timeout_ms=..
gap.phy             link=<n> phy=1/2
gap.data_len        link=<n> tx_octets=.. tx_time=..
adv.start           mode=legacy/extended interval_ms=.. connectable=0/1 len=N data=<hex>
adv.stop
adv.data            len=N data=<hex>
gatt.discover       link=<n> uuid=<hex>
gatt.read           link=<n> handle=<h>
gatt.write          link=<n> handle=<h> response=0/1 len=N data=<hex>
gatt.subscribe      link=<n> cccd=<h> enable=0/1
gatt.mtu            link=<n>
dev.list
svc.register        name=loopback
svc.unregister      name=loopback
sec.bond            link=<n> io=<cap> passkey=<n>
sec.clear
periodic.adv.start  interval_ms=.. len=N data=<hex>
periodic.adv.stop
periodic.sync       index=<scan idx> | addr=<hex>
periodic.sync.stop
```

M1 固件实现到 `svc.notify`；`gap.conn_params` / `gap.phy` / `gap.data_len` /
`adv.data` / `peer data` 在协议里保留，但固件回 `@err ... unsupported`，
宿主不提供对应方法——一个总是失败的方法比没有更糟。

事件：

```
adv          index=<n> addr=<hex> type=0/1 rssi=<dBm> len=N data=<hex>
scan.done    count=<n>
conn         link=<n> addr=<hex> role=0/1 interval_ms=..
disconn      link=<n> reason=<n>
char         link=<n> index=<n> handle=<h> value_handle=<h> descs=<h,h,...>
link         link=<n> addr=<hex> role=0/1 interval_ms=..
notify       link=<n> handle=<h> len=N data=<hex>
read         link=<n> handle=<h> len=N data=<hex>
write        link=<n> handle=<h> len=N data=<hex>
passkey      link=<n> value=<n>
periodic     link=<n> len=N data=<hex>
mtu          link=<n> mtu=<n>
```

### 5.5 行长度

244-byte MTU 的 write 需要约 600 字符的 hex 行。仪表固件应把
`RT_CONSOLEBUF_SIZE` 与 `FINSH_CMD_SIZE` 都调到 1024。若某块板仍放不下：

1. `@info` 报告 `max_line=<n>`；
2. 宿主在超长时改用 `peer data <id> <hex>` 分批续传；
3. 宿主在发送前拒绝超过 `max_line` 的单行，而不是等 shell 静默截断。

### 5.6 能力校验

测试开始前调用 `expect_ready()`：

- 发送 `peer info`；
- 要求 `proto=1`；
- 要求 `caps` 包含本次 fixture 参数要求的能力（如 `ble`）；
- 版本或能力不匹配时给出明确错误，而不是靠一个空提示符通过。

这取代了旧的 `expect_shell()` / `diss help`。

## 6. 宿主 API

### 6.1 文件

```
tools/sdk_test/instruments/
  __init__.py
  console.py      InstrumentConsole（含 ANSI / shell 提示符处理）
  bt_peer.py      BtPeer / BleInstrument / Link / AdvReport / Characteristic / Service
  classic.py      BtClassicInstrument / ClassicDevice / ClassicLink
  _values.py      两个模块共用的 int / payload 解析
  registry.py     INSTRUMENTS: name -> class
  errors.py       InstrumentError / InstrumentTimeout / InstrumentProtocolError
```

### 6.2 `InstrumentConsole`

职责：

- 只按 `\n` 切完整行；非 `@` 行丢弃；
- `@ok/@err` 按 id 唤醒等待者；
- 已超时的 id 进 `expired` 集合，迟到响应丢弃并计数；
- `@evt` 进有界队列；`event_overflow` 计数，不静默丢；
- `drain()` 只清空待解析文本，不清事件队列。

接口：

```python
class InstrumentConsole:
    def command(self, op: str, timeout: float = 10.0, **kwargs) -> dict: ...
    def expect_ready(self, capability: str) -> dict: ...
    def next_event(self, name: str, timeout: float = 10.0, *,
                   since_seq: int | None = None, **match) -> dict: ...
    def release_event(self, event: dict) -> None: ...
    def drain(self) -> None: ...
```

### 6.3 `BtPeer`

```python
class BtPeer:
    def __init__(self, console: InstrumentConsole, *, capability: str) -> None: ...
    @property
    def ble(self) -> "BleInstrument": ...
    @property
    def classic(self) -> "BtClassicInstrument": ...  # 已提供宿主骨架
```

`BtPeer` 在构造时校验 `observe_ready(capability)`；能力不匹配直接报错，
不进入测试。

### 6.4 `BleInstrument`

```python
class BleInstrument:
    def reset(self) -> None: ...
    def scan(self, duration_ms=3000, ...) -> list[AdvReport]: ...
    def connect(self, report=None, *, address=None, ...) -> Link: ...
    def disconnect(self, link: Link) -> None: ...
    def advertise(self, *, mode="extended", data=b"", interval_ms=30, connectable=True) -> None: ...
    def stop_advertising(self) -> None: ...
    def discover(self, link: Link, uuid: bytes | None = None) -> Service: ...
    def read(self, link: Link, handle: int) -> bytes: ...
    def write(self, link: Link, handle: int, data: bytes, *, response=True) -> None: ...
    def subscribe(self, link: Link, cccd_handle: int, *, enable=True) -> None: ...
    def wait_notification(self, handle: int, *, link=None, timeout=10.0) -> bytes: ...
    def register_loopback(self) -> int: ...
    def bond(self, link: Link, *, io="no_input_no_output", passkey=None) -> None: ...
```

数据类：

```python
@dataclass(frozen=True)
class AdvReport:
    index: int
    address: str
    address_type: str
    rssi: int | None
    data: bytes

@dataclass(frozen=True)
class Link:
    link_id: int
    address: str
    role: str
    interval_ms: int

@dataclass(frozen=True)
class Characteristic:
    index: int
    handle: int
    value_handle: int
    descriptors: list[int]

@dataclass(frozen=True)
class Service:
    characteristics: list[Characteristic]
```

## 7. fixture、binding 与调度

### 7.1 测试用例声明

```yaml
  - id: ble_central_and_peripheral.peer
    harness: pytest
    project_dir: project/hcpu
    harness_config:
      fixture: bt_peer:ble
      pytest_root: pytest_peer
```

- `bt_peer` 是基名，`ble` 是能力参数。
- 一个用例可以声明多个 fixture（列表），但本期只需要一个。
- 同一用例声明两个同名 spec 会报错，不会随便挑一个。

### 7.2 binding

```yaml
connections:
  - fixture: bt_peer:ble
    console: {port: COM22, baud: 1000000}
```

以后加经典蓝牙：

```yaml
connections:
  - fixture: bt_peer:ble
    console: {port: COM22, baud: 1000000}
  - fixture: bt_peer:classic
    console: {port: COM22, baud: 1000000}   # 同一块板，同一串口
```

一个用例只会打开自己声明的那一个能力；同一物理端口不会被同时打开两次。

### 7.3 pytest 插件

不再写死 `ble_peer`，从注册表动态生成 session fixture：

```python
INSTRUMENTS = {"bt_peer": BtPeer}

for name, cls in INSTRUMENTS.items():
    globals()[name] = _make_instrument_fixture(name, cls)
```

规则：

- fixture 名必须是合法 Python 标识符，且不能与 `dut` / `console` 冲突；
  冲突在插件 import 时直接报错；
- 用例没声明该 fixture 时 `pytest.skip` 并说明缺什么；
- 参数可以是能力集合（`bt_peer:ble,classic`），宿主要求 `@info caps` 全部满足；
- 声明了多个同名 spec 时 `pytest.fail`，不猜；
- 流不主动关闭，留给 runner 回收 `<fixture>.log`。

### 7.4 同一块仪表同时提供 BLE 和 Classic

先看 `stream` 是什么：它是 Agent 的 **路由键**，不是能力开关。

```
GET /testbed/{id}/console/read?cursor=N&stream=bt_peer:ble
```

- `stream` 省略或 `dut` → DUT 的串口；
- `stream=bt_peer:ble` → binding 里 spec 恰好是 `bt_peer:ble` 的那条 connection 的串口；
- Agent 只按 cursor 存取字节，不解析 `@info`；能力判断在宿主。

所以"同一块板既有 BLE 又有 Classic"的核心问题是：
**一块板通常只有一个 console 串口，两种能力的命令必须共享同一条流。**

#### 方案 A（推荐）：一个 spec、一条流、能力集合

```yaml
# binding
connections:
  - fixture: bt_peer:ble,classic
    console: {port: COM22, baud: 1000000}
```

```yaml
# testcase.yaml
harness_config:
  fixture: bt_peer:ble,classic
```

固件：

```
@info proto=1 fw=... caps=ble,classic roles=... 
```

宿主：

```python
def test_dual(bt_peer):
    bt_peer.ble.scan(...)          # BLE ops
    bt_peer.classic.inquiry(...)   # Classic ops
```

- 一个 `BtPeer`、一个 `InstrumentConsole`；两种能力共用一条串口。
- 协议用 op 命名空间区分（`gap.*` / `gatt.*` vs `btc.*`），请求 id 和事件 seq 负责多路复用。
- `expect_ready("ble,classic")` 要求 `caps` 同时包含两者；缺一个就拒绝。
- Agent 不改，`PROTOCOL` 不 bump；binding 和用例只是多写一个参数。

这是同板双能力最省事、也最容易推理的方案。

**实现状态（方案 A）**

- 宿主：`InstrumentConsole.expect_ready()` 按逗号拆分能力集合，要求 `caps`
  包含全部条目；`BtPeer.caps` 报告固件实际能力；`BtPeer.ble` 和
  `BtPeer.classic` 各自检查对应能力，缺一个就明确拒绝。
- 固件：`peer.h` 的 `PEER_CAPS` 是能力字符串的唯一来源，M1 默认 `"ble"`；
  加入 Classic 栈时用构建宏覆盖成 `"ble,classic"`，协议不变。
- 宿主 Classic 半边：`tools/sdk_test/instruments/classic.py` 定义
  `BtClassicInstrument`（inquiry / connect / disconnect / SPP），M1 固件
  尚未包含 Classic 栈，所以只有 `caps` 含 `classic` 的镜像才会交出它。
- 单元测试覆盖：`bt_peer:ble,classic` 能同时取到 `.ble` 和 `.classic`；
  BLE-only 镜像访问 `.classic` 明确报错；`btc.*` 命令与事件映射有独立测试。

#### 方案 B：一个端口两条 connection，Agent 共享流

```yaml
connections:
  - fixture: bt_peer:ble
    console: {port: COM22}
  - fixture: bt_peer:classic
    console: {port: COM22}
```

当前 Agent 按 **connection 的 spec** 给每条 connection 建独立 `ConsoleStream`
（`tools/lab_agent/testbed.py`）。两个 spec 指向同一端口时，会开两个 reader 线程
抢同一串口，字节会丢/错位。要让这个方案成立，需要：

1. Agent 改成按 **端口** 共享一个 `ConsoleStream`；
2. 宿主侧两个 fixture 实例也共享一个 `InstrumentConsole`。

改动比方案 A 大，且收益不大。**不建议第一期做。**

#### 方案 C：两块板 / 两个端口

```yaml
connections:
  - fixture: bt_peer:ble
    console: {port: COM22}
  - fixture: bt_peer:classic
    console: {port: COM23}
```

- Agent 已经支持多个 fixture console，两条流互不干扰。
- pytest 插件目前对"同一基名声明多次"是 **报错**（不猜）；要用这个方案，
  需要让插件支持返回多个实例（列表/字典），或者用不同的基名（`bt_peer` / `bt_peer2`）。
- 适用场景：BLE 和 Classic 不能共存于一块板/一个协议栈，或者确实需要两个独立的物理对端。
- `PROTOCOL` 不需要改；这是插件能力扩展。

**结论**：同板双能力用 **方案 A**；只有硬件/协议栈不能共存时才用 **方案 C**；
方案 B 需要 Agent 改共享流，不划算。

## 8. 覆盖矩阵

| 例程 | DUT 角色 | 仪表用法 |
|---|---|---|
| `basc` `cscpc` `hrpc` `cppc` `diss` `tip_cts` `peripheral` `file_transfer` | 广播 + 注册 `sifli_app` 服务 | GATT client：scan/connect/discover/read/write/subscribe |
| `iBeacon` `periodic_adv` | 只广播 | scan + 断言 adv data |
| `throughput` `multi_connection` `central_and_peripheral` | 双向 | central + peripheral + loopback 服务 |
| `pair` | 广播 + 配对 | central + bond/passkey |
| `periodic_adv_sync` | 同步方 | periodic advertiser |
| `hid` | 16-bit HID 服务 | GATT client + input report 订阅 |

## 9. 迁移

1. 先把 `BlePeer` 的调用面拆成 `InstrumentConsole` + `BleInstrument`，现有用例行为不变。
2. 新固件实现 `@info/@req/@ok/@evt`，先支持最小命令集。
3. 切换 `central_and_peripheral/pytest_peer` 到 `bt_peer:ble`。
4. 删除 `tools/sdk_test/ble_peer.py` 与旧的 transcript 解析测试；
   重写为协议层测试。
5. 逐例程补 `testcase.yaml` + pytest 套件（M2/M3）。
6. 实验室固件与宿主代码同批切换；旧 `diss` 协议不保留兼容层。
   若实验室无法同批更新，先不合入，避免两套协议并存。

## 10. 验证

### 10.1 无硬件

```bash
python -m unittest discover -s tools/sdk_test/tests -p "test_*.py"
python -m unittest discover -s tools/lab_agent/tests -p "test_*.py"
```

协议单测覆盖：

- 分块行、`\r\r\n`、交错 ulog；
- `@ok`/`@err` 按 id 配对；
- 迟到响应被丢弃且不影响下一次请求；
- 空 payload（`len=0`）；
- 跨 handle 通知保留；
- 事件队列上限与 `event_overflow`；
- `@info` 能力不足 / 版本不符；
- 重复声明同一 fixture 报错；
- fixture 名与 `dut` 冲突报错。

### 10.2 反向验证（必须）

每条新用例都要故意打红：

- DUT 侧注释掉广播/服务注册 → 用例红；
- 仪表 `gap.scan` 不返回结果 → `strongest` 断言红；
- 仪表返回 `@ok` 但未真正执行写 → DUT 侧断言红；
- 仪表固件缺 `proto=1` 或能力 → `expect_ready()` 明确失败；
- 仪表返回 `len` 与 `data` 不一致 → 宿主报协议错误；
- 断电/拔串口 → `peer info` 超时并带 transcript；
- 旧 `diss` 镜像 → 即使 shell 有响应也必须失败。

### 10.3 真板

```bash
sdk.py test list -T example/ble/central_and_peripheral
sdk.py test run -T example/ble/central_and_peripheral -p <board>
sdk.py test run -T example/ble/central_and_peripheral --tag peer -p <board>
```

检查：

- `device.log` 与 `bt_peer-ble.log` 双端日志都在；
- `sdk.py test run` 退出码正确；
- 逐例程跑 M2/M3。

**首次真板通过（2026-11-07）**：DUT `sf32lb52-lcd_a128r16`（COM8）+ 仪表
`sf32lb58-lcd_n16r32n1_qspi`（COM6，`fw=bt_peer-m2`），
`ble_central_and_peripheral.peer` 7/7 通过（scan/connect/discover/write/read/
subscribe/5 条通知）。这次实测暴露并修掉了三个传输层问题：

1. ulog 的 `ULOG_LINE_BUF_SIZE=128` 截断长 `@evt adv` 行 → 协议行改为直接写
   console 设备，绕开 ulog 行缓冲；
2. ulog 的颜色码会粘在 `@ok`/`@evt` 行首 → 固件关 `ULOG_USING_COLOR`，宿主
   解析前剥 ANSI；
3. `msh />` 提示符没有换行，异步事件会变成 `msh />@evt ...` → 固件每条协议
   行前加换行，宿主也识别提示符后的 `@`。

## 11. 经典蓝牙扩展

结论：**传输、协议、注册表、binding/分配这四层可以直接复用；
profile 语义和音频通路不能复用，需要独立的 `classic/` 模块。**

`example/bt` 现有 15 个例程，按可自动化程度分四类：

| 类别 | 例程 | 仪表能否覆盖 |
|---|---|---|
| 数据/控制 profile | `spp`（明确面向板对板）、`test_example` 的 BLE 半 | 可以 |
| 音频 profile 的控制面 | `music_sink*`、`music_source`、`a2dp_multi_connect`（单路）、`3sco` 的 L2CAP 建立、`hfp`、`hfp_relay`、`a2dp_sharing` | 控制面可以；音质/SCO 音频需要 codec/模拟通路 |
| 必须手机/网络/多对端 | `pan`、`pan_ota`、`avrcp_cover_art`、`a2dp_multi_connect` 的双 source、`a2dp_sharing`/`hfp_relay` 的 phone+headset 拓扑 | 单板对端不够 |
| 传输层 | `HCI_over_uart`、`music_sink_with_hci_over_uart` | 不是 profile 对端，需要 HCI 收发/抓包 |

关键复用点：`middleware/rt_bt_app/` 已经是一套行协议框架：

- `MSH_CMD_EXPORT(bt, ...)`
- `bt <service> <cmd> [args]`
- `RT_BT_SERVICE_REGISTER()` 自注册
- `rt_bt_cmd_entry_t` 命令表
- 统一 `ret=0x..` 响应

现有 HFP / SPP service plugin 在
`example/rt_device/bt/src/services/bt_srv_{hfp,spp}.c`。
经典蓝牙仪表应当复用它，而不是另造命令格式。

实现时：

- 固件加 `classic/` service plugins（A2DP SRC+SNK、AVRCP CT+TG、
  HFP HF+AG、SPP、L2CAP/3SCO）；
- 宿主加 `BtClassicInstrument`，通过 `bt_peer.classic` 暴露；
- binding 加 `bt_peer:classic`；
- 用例参数改为 `bt_peer:classic`；
- `example/bt` 的工程布局是扁平 `project/`，没有 `project/hcpu`；
  测试用例的 `project_dir` 要按这个写。

## 12. 风险

1. **固件工作量**：GAP/GATT/安全/周期广播多套逻辑，比宿主侧大。
   M1 只做最小命令集，M2/M3 增量补。
2. **固件版本漂移**：实验室固定固件，宿主必须用 `@info` 校验；
   版本不匹配明确失败，不能靠空提示符。
3. **多链路**：必须用稳定的 link id，不能沿用轮询 `show_dev`。
4. **HID**：16-bit 服务与通用 128-bit 路径不同，单独做 helper。
5. **命令行长度**：见 5.5；长负载在 M2 前做一次 244-byte 实测。
6. **串口共线**：协议只认 `@` 行，但 ulog 可能插在行中间；
   固件侧必须保证一条响应是单次输出。
7. **并发 GATT 操作**：同一 link 只允许一个未完成操作，返回 `busy`。
8. **事件风暴**：宿主有界队列 + `event_overflow`，不静默丢。
9. **仪表是共享资源**：套件开始时 `state.reset()` 一次；改变全局状态的
   测试自己复位；文档里写明顺序敏感。

## 13. 已确认决策

- 本期只做 BLE，不做 Q2（电流表）。
- 新建专用固件工程，不扩展现有例程。
- 固件工程与 fixture 统一叫 `bt_peer`；能力用 `bt_peer:ble` / `bt_peer:classic`。
- 设计文档放在 `tools/sdk_test/BT_PEER_DESIGN.md`。
- 不考虑 `lcpu_general`（小核 controller 直接 `AddLCPU` 引用）。
