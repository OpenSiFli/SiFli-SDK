# BT 测试仪表（bt_peer）设计

## 1. 为什么改

上一版的 `ble_peer` 不是一个"仪表"抽象，而是"某一份例程固件 + 针对它日志文本的解析器"：

- `tools/sdk_test/ble_peer.py` 明确假设仪表运行 `example/ble/central_and_peripheral`，
  连该例程的 `sifli_app` 128-bit 服务 UUID 都硬编码在 `SERVICE_UUID_BYTES` 里；
- `tools/sdk_test/pytest_plugin.py` 只定义了 `ble_peer` 一个仪表 fixture，
  并直接 `import BlePeer`；
- 断言建立在 ulog 文本上（连 `"deivce"`、`"Charateristic"` 这类拼写都必须兼容），
  hexdump 是否收全靠 "quiet window" 猜。

后果是：只能测一个例程；换一个例程要么换仪表固件，要么改框架解析器。
本设计把仪表拆成三层——**专用对端固件、稳定行协议、
分层的宿主 API**——并用它覆盖 `example/ble` 下 server 类与双角色例程。
同一套传输/协议/注册表已经在 M6 用于扩展 `example/bt` 的经典蓝牙
（见 §10、§14）。

## 2. 术语

| 术语 | 含义 |
|---|---|
| **SDK 侧 / 宿主** | 跑在 PC 上的 Python 进程：`tools/sdk_test` + pytest。本文说"宿主 API"都指这里，不是 Lab Agent，也不是板上的固件。 |
| **Lab Agent** | 插着板子的那台机器上的常驻服务，负责占用、烧录、复位、开串口。 |
| **仪表 / peer** | 第二块板及其固件。本文统一叫 `bt_peer`。 |
| **DUT** | 被测板。 |
| **能力** | 仪表能扮演的角色，如 `ble`、`classic`；写在 fixture spec 的 `:` 之后。 |

命名约定（已确认）：

- 固件工程：`example/test/bt_peer/`
- fixture 名：`bt_peer`，能力用参数区分：
  - `bt_peer:ble` —— BLE 半边；
  - `bt_peer:classic` —— 经典蓝牙半边，M6 实现且只做 SPP（§14）；
  - `bt_peer:ble,classic` —— 一台 58X 仪表同时提供两项，宿主写这个能力集合。
- 宿主包：`tools/sdk_test/instruments/`

fixture 名描述的是**能力**，不是物理设备；一个物理仪表可以同时提供多种能力，
由 binding 把能力映射到物理端口。

## 3. 范围

### 3.1 本期覆盖

BLE：

- GATT server 类：`basc`、`cscpc`、`hrpc`、`cppc`、`diss`、`tip_cts`、
  `peripheral`、`file_transfer`
- 广播类：`iBeacon`、`periodic_adv`
- 双角色：`central_and_peripheral`、`pair`、`periodic_adv_sync`
- `hid` 单独处理（16-bit HID 服务）
- `throughput` / `multi_connection` 不在本期：前者需要仪表侧的吞吐收发通路，
  后者需要第二个对端（见 §11.2）

### 3.2 不在本期

- `ancs` / `ams`：需要模拟 iPhone 的 ANCS/AMS 服务
- `talkback`：音频通路
- `lcpu_general`：小核 controller 工程；52X 由 `AddLCPU` 作为子工程编入，本工程不改它
- OTA 专项：`*_with_ota` 只覆盖普通 GATT 部分
- Q2 的通用仪器抽象与电流表

经典蓝牙也不在 M1 范围内；M6 单列的范围与实现见 §10 / §14（只做 SPP）。

## 4. 架构

```
+---------------------------- PC（SDK 侧 / 宿主） ----------------------------+
| pytest suite                                                                |
|   bt_peer fixture  ->  BtPeer                                               |
|                          +-- .ble     -> BleInstrument                      |
|                          +-- .classic -> BtClassicInstrument（M6：inquiry/SPP）|
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
| peer_classic.c  经典：inquiry / SPP 链路 / `btc.*`（仅 PEER_CLASSIC）        |
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
    Kconfig.proj           # BF0_HCPU + CUSTOM_MEM_MAP + PEER_CLASSIC / PEER_CAPS
    proj.conf              # BLUETOOTH + ULOG + FINSH/MSH + 缓冲
    sf32lb58x/
      proj.conf            # 58X 覆盖：PEER_CLASSIC + 经典栈/SPP 配置（§14.1）
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
    peer_file.c
    peer_classic.c/.h
```

- 扁平 `project/` 布局即可；只有同时要建 ACPU 子工程（58X）时才拆
  `project/hcpu` + `project/acpu`。
- BLE 栈不在 `src/SConscript` 里列源码：`SConscript($SIFLI_SDK/SConscript)`
  走 SDK 全树发现，`middleware/bluetooth/SConscript` 由
  `GetDepend('BLUETOOTH')` 控制，链接 `lib_bt_gcc`。
- 小核 controller 不改：52X 与 58X 都走 `AddLCPU(...)`，它把
  `example/ble/lcpu_general` 作为 LCPU 子工程编入（`target_file` 参数只在
  55X 拷贝预编译镜像时用到）。经典半边只改 HCPU 侧：`PEER_CLASSIC` 由
  `project/sf32lb58x/proj.conf` 打开（见 §14.1）。
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
@info proto=1 fw=<hash> caps=<ble|ble,classic>[ btc=1] addr=<uid-derived> name=bt_peer-<xxx> roles=central,peripheral max_conn=8 ble=5.2 max_line=240
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

### 5.4 命令与事件

命令：

```
state.reset
gap.scan            duration_ms=.. interval_ms=.. window_ms=.. dup=0/1 rssi_min=..
gap.scan.stop
gap.connect         index=<scan idx> | addr=<hex> addr_type=0/1 timeout_ms=.. interval_ms=..
gap.disconnect      link=<n>
gap.conn_params     link=<n> interval_min_ms=.. interval_max_ms=.. latency=.. timeout_ms=..
gap.phy             link=<n> phy=1/2/3   # 1M / 2M / Coded
gap.data_len        link=<n> tx_octets=.. tx_time=..
adv.start           mode=legacy/extended interval_ms=.. connectable=0/1 len=N data=<hex>
adv.stop
adv.data            len=N data=<hex>
gatt.discover       link=<n> uuid=<hex>
gatt.read           link=<n> handle=<h>
gatt.write          link=<n> handle=<h> response=0/1 len=N data=<hex>
gatt.subscribe      link=<n> cccd=<h> enable=0/1
gatt.mtu            link=<n>
file.push           link=<n> handle=<h> name=<file> len=N data=<hex>
dev.list
svc.register        name=loopback
svc.unregister      name=loopback
sec.bond            link=<n> io=<cap> auth=<legacy|secure|...> passkey=<n>
sec.clear
periodic.adv.start  interval_ms=.. len=N data=<hex>
periodic.adv.stop
periodic.sync       index=<scan idx> | addr=<hex>
periodic.sync.stop
```

M1 固件已实现上表的命令分发与其余 op（含 `gap.conn_params` / `gap.phy` /
`gap.data_len` / `adv.data`）。只有两项明确回 `@err ... unsupported`：
`peer data`（分块续传，1024 行预算已覆盖 460-byte payload）和
`svc.unregister`（栈无法从 ATT 数据库移除服务）。

事件：

```
adv          index=<n> addr=<hex> type=0/1 rssi=<dBm> len=N data=<hex>
scan.done    count=<n>
conn         link=<n> addr=<hex> role=0/1 interval_ms=..
disconn      link=<n> reason=<n>
char         link=<n> index=<n> handle=<h> value_handle=<h> uuid=<uuidhex> descs=<h>:<uuidhex>,...
             (UUID in the order the peer sent it, LSB first: a CCCD is 0229)
link         link=<n> addr=<hex> role=0/1 interval_ms=..
notify       link=<n> handle=<h> len=N data=<hex>
read         link=<n> handle=<h> len=N data=<hex>
write        link=<n> handle=<h> len=N data=<hex>
passkey      link=<n> value=<n>
periodic     link=<n> len=N data=<hex>   (only when the payload changes)
mtu          link=<n> mtu=<n>
conn_params  link=<n> interval_ms=<n> latency=<n> timeout_ms=<n>
data_len     link=<n> tx_octets=<n> tx_time=<n>
```

经典半边（58X，`PEER_CLASSIC`）在同一个 dispatcher 里增加五个 `btc.*` op
和四个事件（`btc.dev` / `btc.conn` / `btc.disc` / `btc.data`），不改变上表
任何一行；参数、响应和语义见 §14.2。

### 5.5 行长度

`RT_CONSOLEBUF_SIZE` 与 `FINSH_CMD_SIZE` 都调到 1024，但**实际能收的行远短于此**：
实测 201 字符的命令行正常，329 字符的行到达 shell 时被截断并出现杂散 NUL 与重复
字节（`file.push` 一次 128 字节载荷时复现）。也就是说超长行的后果是"损坏"而不是
"没响应"，必须由宿主在发送前拦住。

因此固件把**请求侧**预算单列成 `PEER_CMD_MAX`（240 字符），并由 `@info max_line`
上报；`PEER_LINE_MAX`（1024）只约束**输出**行（`@evt adv` 这类）。宿主在
`_build_request()` 里按 `max_line` 预检，超长直接报错。需要更长载荷时只能分批
（`peer data` 续行仍未实现，见 13.2.2），不能靠加大 `FINSH_CMD_SIZE`。

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
  classic.py      BtClassicInstrument / ClassicDevice / ClassicLocal / ClassicLink
  _values.py      两个模块共用的 int / payload 解析
  registry.py     INSTRUMENTS: name -> class
  errors.py       InstrumentError / InstrumentTimeout / InstrumentCommandError /
                  InstrumentProtocolError / InstrumentCapabilityError

tools/sdk_test/suite_helpers/
  sifli_app.py    某一族例程共用的 DUT 日志约定（连接、写读、订阅），
                  不属于仪表的通用能力，所以不在 instruments/ 里
```

### 6.2 `InstrumentConsole`

职责：

- 只按 `\n` 切完整行；非 `@` 行丢弃；
- `@ok/@err` 按 id 唤醒等待者；
- 已超时的 id 进 `expired` 集合，迟到响应丢弃并计数；
- `@evt` 进有界队列；`event_overflow` 计数，不静默丢；
- 被调用方边界（`since_seq`）挡住的旧事件直接丢弃并记 `stale_events`，
  不再留在队列里让每次等待都整表遍历；
- 同一个 id 的第二次答复记 `duplicate_responses`，不覆盖第一次；
- `drain()` 只清空待解析文本，不清事件队列；
- 协议行只认行首或紧跟提示符（`msh ...>`）的 `@info/@ok/@err/@evt`，
  日志里偶然出现的 `@ok` 不算答复。

接口：

```python
class InstrumentConsole:
    def command(self, op: str, timeout: float = 10.0, **kwargs) -> dict: ...
    def command_events(self, op: str, event: str, *, count_key="count",
                       match=None, timeout=10.0, boundary="firmware",
                       **kwargs) -> tuple[dict, list[dict]]: ...
    @staticmethod
    def event_boundary(result: dict, op: str) -> int: ...
    def expect_ready(self, capability: str) -> dict: ...
    def next_event(self, name: str, timeout: float = 10.0, *,
                   since_seq: int | None = None, **match) -> dict: ...
    def release_event(self, event: dict) -> None: ...
    def drain(self) -> None: ...
    #: 计数器：late_responses / stray_responses / duplicate_responses /
    #: event_overflow / stale_events

```

### 6.3 `BtPeer`

```python
class BtPeer:
    def __init__(self, console: InstrumentConsole, *, capability: str) -> None: ...
    @property
    def ble(self) -> "BleInstrument": ...
    @property
    def classic(self) -> "BtClassicInstrument": ...  # M6：inquiry / SPP，见 §14.3
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
    def set_advertising_data(self, data: bytes) -> None: ...
    def start_periodic_advertising(self, data=b"", *, interval_ms=100) -> None: ...
    def stop_periodic_advertising(self) -> None: ...
    def sync_periodic(self, report=None, *, address=None, address_type=0) -> int: ...
    def stop_periodic_sync(self) -> None: ...
    def wait_periodic(self, *, link=None, since_seq=None, timeout=15.0) -> bytes: ...
    def update_connection(self, link: Link, *, interval_min_ms, interval_max_ms,
                          latency=0, timeout_ms=5000) -> None: ...
    def set_phy(self, link: Link, phy: int) -> None: ...
    def set_data_length(self, link: Link, tx_octets: int, tx_time: int = 0) -> None: ...
    def discover(self, link: Link, uuid: bytes | None = None) -> Service: ...
    def read(self, link: Link, handle: int) -> bytes: ...
    def write(self, link: Link, handle: int, data: bytes, *, response=True) -> None: ...
    def subscribe(self, link: Link, cccd_handle: int, *, enable=True) -> None: ...
    def exchange_mtu(self, link: Link) -> int: ...
    def file_push(self, link: Link, handle: int, name: str, data: bytes, *,
                  timeout: float = 60.0) -> None: ...
    def wait_notification(self, handle: int, *, link=None, since_seq=None, timeout=10.0) -> bytes: ...
    def wait_read(self, handle: int, *, link=None, since_seq=None, timeout=10.0) -> bytes: ...
    def wait_passkey(self, *, link=None, since_seq=None, timeout=30.0) -> int: ...
    def list_links(self) -> list[Link]: ...
    def register_loopback(self) -> int: ...
    def notify(self, handle: int, data: bytes) -> None: ...
    def bond(self, link: Link, *, io="no_input_no_output", auth=None, passkey=None) -> None: ...

#: 所有 wait_* 的 `since_seq` 默认是"从现在起"（取当前 `last_seq`）；
#: 要回溯历史就显式传入一个更早的边界（前提是那些事件还没被更早的边界丢弃）。
#: 收连续突发（例如一次写入触发的一串通知）必须在触发前取一次边界，
#: 整个循环复用同一个。
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
    uuid: bytes | None            # 选特征用它，不按顺序
    descriptors: list[int]
    descriptor_uuids: list[bytes | None]

@dataclass(frozen=True)
class Service:
    characteristics: list[Characteristic]
    def characteristic(self, uuid: bytes) -> Characteristic: ...
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

一块 58X 仪表同时提供两项能力时，binding 仍然只写一条：

```yaml
connections:
  - fixture: bt_peer:ble,classic
    console: {port: COM22, baud: 1000000}
```

能力按子集匹配：实验室接 `bt_peer:ble,classic` 时，只声明 `bt_peer:ble`
的用例照样能分到它。为什么不为每项能力各写一条 connection，见 §7.4。

### 7.3 pytest 插件

不再写死 `ble_peer`，从注册表生成 session fixture。生成动作放在
`pytest_configure` 里，`register_instrument()` 在插件已配置后会立即安装自己的
fixture，所以从 `conftest` 注册也有效：

```python
def pytest_configure(config):
    for name, cls in INSTRUMENTS.items():
        globals()[name] = _make_instrument_fixture(name, cls)
```

规则：

- fixture 名必须是合法 Python 标识符，且不能与 `dut` / `console` 冲突；
  冲突在注册时直接报错；
- 工厂必须接受 `capability=` 关键字（注册时校验），否则 fixture 会在构造时炸在
  用例里；
- 用例没声明该 fixture 时 `pytest.skip` 并说明缺什么；
- spec 语法：`name[@实例][:能力,能力]`。`:` 后面是能力列表，宿主要求
  `@info caps` 覆盖列表里的每一项（子集匹配）；`@实例` 是"钉住哪一台"，
  与能力分开，所以 `bt_peer@peer1:ble` 既钉住实例又要求能力；
- Agent 解析连接时用同一套 `accepts()`，所以"能力更全的仪表满足要求较少的用例"
  在调度和固件校验两处一致；
- 声明了多个同名 spec 时 `pytest.fail`，不猜；
- 流不主动关闭，留给 runner 回收 `<fixture>.log`。

### 7.4 同一块仪表同时提供 BLE 和 Classic

先看 `stream` 是什么：它是 Agent 的**路由键**，不是能力开关。

```
GET /testbed/{id}/console/read?cursor=N&stream=bt_peer:ble
```

- `stream` 省略或 `dut` → DUT 的串口；
- `stream=bt_peer:ble` → binding 里满足该 spec 的那条 connection 的串口
  （精确 spec 优先，其次 capability 子集匹配；`@实例` 用于消歧）；
- Agent 只按 cursor 存取字节，不解析 `@info`；能力判断在宿主。

一块板通常只有一个 console 串口，所以两套能力的命令必须共享同一条流。为此
采用**一个 spec、一条流、能力集合**：

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

```python
def test_dual(bt_peer):
    bt_peer.ble.scan(...)          # gap.* / gatt.*
    bt_peer.classic.inquiry(...)   # btc.*
```

- 多路复用交给协议本身：两套能力是不同 op 命名空间（`gap.*` / `gatt.*` 对
  `btc.*`），请求 id 与事件 seq 已经能把它们分开，不需要第二条流。
- spec 的 `:` 是能力列表，`expect_ready()` 要求 `@info caps` 覆盖其中每一项，
  缺一个就明确拒绝；binding 和用例写同一个 spec，Agent 与 `PROTOCOL` 都不用改。
- 固件的 `PEER_CAPS` 是能力字符串的唯一来源：`project/Kconfig.proj` 按
  `PEER_CLASSIC` 把它设成 `"ble,classic"` 或 `"ble"`，`src/peer.h` 只保留
  同值的 C 兜底。协议不变，`@info caps=` 随构建变化，`btc=1` 还要求
  `peer_classic_init()` 成功。

**不采用的两种做法**（记录原因，避免以后重新讨论）：

- 同一端口挂两条 connection（`bt_peer:ble` + `bt_peer:classic` 指向 COM22）：
  Agent 按 connection 各建一个 `ConsoleStream`，两个 reader 线程抢同一串口会
  丢字节/错位；要成立必须先让 Agent 按端口共享流，改动大、收益为零。
- 两块板、两个端口：只有当硬件或协议栈确实不能共存时才需要；场合上属于
  “两个独立对端”，不是“一块仪表两种能力”。插件还要求先支持同一基名返回多个
  实例，与本期目标无关。

## 8. 覆盖矩阵

| 例程 | DUT 角色 | 仪表用法 | 状态 |
|---|---|---|---|
| `basc` `cscpc` `hrpc` `cppc` `diss` `tip_cts` | 广播 + 注册 `sifli_app` 服务 | GATT client：scan/connect/discover/read/write/subscribe | 已覆盖；DUT 偶发挂死见 §11.1 |
| `peripheral` `file_transfer` | 同上 | 同上 | 已覆盖 |
| `central_and_peripheral` | 双向 | central + peripheral + loopback 服务 | 已覆盖 |
| `iBeacon` | 只广播 | scan + 断言完整 AD 与地址 | 已覆盖 |
| `periodic_adv` | 只广播（周期） | scan + `periodic.sync` 读周期信道 | 已覆盖 |
| `periodic_adv_sync` | 同步方 | `periodic.adv.start` + DUT 侧 shell | 已覆盖 |
| `hid` | 16-bit HID 服务 | GATT client + 配对 + input report 订阅 | 已覆盖 |
| `pair` | 广播 + 配对 | central + `sec.bond`（legacy） | 已覆盖 |
| `throughput` | 需要两块板跑同一例程 | 需要仪表侧的吞吐收发模式 | 未覆盖（见 §11.2） |
| `multi_connection` | 需要多个对端 | 一台仪表只提供一个对端 | 未覆盖（见 §11.2） |

## 9. 验证

### 9.1 无硬件

```bash
python -m unittest discover -s tools/sdk_test/tests -p "test_*.py"
python -m unittest discover -s tools/lab_agent/tests -p "test_*.py"
```

协议单测覆盖的是 console 的契约本身：跨读的分块行、`@ok`/`@err` 按 id 配对、
迟到与重复答复不落入下一次请求、空 payload、跨 handle 的事件保留、队列上限与
`event_overflow`、`first_seq` 缺失时报错、能力不足时报错、fixture 重名与
保留名报错。

### 9.2 反向验证（必须）

一条用例写没写对，只有在故意打红之后才知道：

- DUT 侧坏掉（去掉广播或服务注册、发错字节）→ 断言红；
- 仪表侧造假（`@ok` 却没执行、`len` 与 `data` 不符、旧镜像）→ 宿主报错，
  不能静默通过；
- 断电或断串口 → `peer info` 超时并带 transcript。

AGENTS.md 对此有硬性要求，设计上也要保证失败的信号是"用例红"，而不是
"跳过"。

### 9.3 真板

```bash
sdk.py test list -T <example>
sdk.py test run  -T <example> -p <board>
```

要看的不是"绿了"，而是证据链：`device.log` 与 `<fixture>.log` 两端都在；
断言针对的是 DUT 自己的输出（连接、配对、收到的数据），不是仪表的一厢情愿。

## 10. 经典蓝牙扩展（M6 已实现）

结论不变：**传输、协议、注册表、binding/分配这四层原样复用；
profile 语义和音频通路需要独立的固件模块。本期只做了 SPP。**

`example/bt` 现有 15 个例程，按可自动化程度分四类：

| 类别 | 例程 | 仪表能否覆盖 |
|---|---|---|
| 数据/控制 profile | `spp`（明确面向板对板）、`test_example` 的 BLE 半 | `spp` 已覆盖（§14.4）；`test_example` 只覆盖 BLE 半 |
| 音频 profile 的控制面 | `music_sink*`、`music_source`、`a2dp_multi_connect`（单路）、`3sco` 的 L2CAP 建立、`hfp`、`hfp_relay`、`a2dp_sharing` | 控制面可以；音质/SCO 音频需要 codec/模拟通路。本期未做 |
| 必须手机/网络/多对端 | `pan`、`pan_ota`、`avrcp_cover_art`、`a2dp_multi_connect` 的双 source、`a2dp_sharing`/`hfp_relay` 的 phone+headset 拓扑 | 单板对端不够。本期未做 |
| 传输层 | `HCI_over_uart`、`music_sink_with_hci_over_uart` | 不是 profile 对端，需要 HCI 收发/抓包。本期未做 |

两处原计划与实际不符，记录在此，避免以后照抄：

- **`middleware/rt_bt_app/` 没有复用。** 那套行协议框架
  （`MSH_CMD_EXPORT(bt, ...)`、`RT_BT_SERVICE_REGISTER()`、
  `rt_bt_cmd_entry_t`、统一 `ret=0x..`）只被
  `example/rt_device/bt/src/services/bt_srv_{hfp,spp}.c` 使用，
  `example/bt` 下没有任何例程源文件引用它。它是另一个例程的 shell 命令层，
  与宿主的 `peer req` 协议无关；硬挂进去会多出一套命令格式。因此
  `btc.*` 直接建在 `peer_classic.c` 里，使用
  `bt_interface_*` / `bts2_app_interface.h` 的异步接口。
- **经典主机栈只有 58X 构建打开。** `PEER_CLASSIC` 依赖 `SOC_SF32LB58X`，
  所以 `bt_peer:classic` 是 58X 专属能力；小核 controller 两端走同一条
  `AddLCPU` 路径（§14.1）。

实现落点：

- 固件：`src/peer_classic.c/.h`，op 命名空间 `btc.*`，SPP 客户端与服务端
  两种角色（入向连接自动 accept）；
- 宿主：`tools/sdk_test/instruments/classic.py` 的 `BtClassicInstrument`，
  通过 `bt_peer.classic` 暴露（§14.3）；
- binding / 用例：一个 spec、一条流、能力集合 `bt_peer:ble,classic`（§7.4）；
- `example/bt` 的工程布局是扁平 `project/`，测试用例的 `project_dir` 写
  `project`。

## 11. 风险

### 11.1 已知的、不是本期能修的

- **经典蓝牙 profile 例程偶发挂死**：`hrpc` / `basc` / `cppc`（以及尚未写套件的
  `cscpc` / `diss` / `tip_cts`）有时在 DUT 打印 `Page scan enbd` 之后 HCPU 不再
  响应，仪表侧看到 HCI reason 62；重跑即恢复，根因未定位。遇到时先重跑，再判断
  是不是例程侧的问题。
- **Secure Connections 配对失败**：52x DUT 与 58x 仪表做 SC 配对时双方都报
  DHKey check failed（`0x6B` / `0x7B`）。legacy 配对正常，`hid` 与 `pair`
  用例因此显式请求 `auth=legacy`；这两处例程只需要“加密链路”，不需要 SC。
- **passkey 流程**：固件的 `sec.bond` 在配对完成前不回 `@ok`，所以测试线程
  无法在配对过程中把 passkey 敲给对端。当前用例都用 Just Works；
  支持 passkey 需要把 `sec.bond` 拆成“开始/等待”两步。
- **命令行长度**：msh 名义 1024，但实测 201 字符正常、329 字符到达时已损坏
  （截断 + 杂散 NUL）。固件把请求预算降到 `PEER_CMD_MAX`=240 并由 `@info
  max_line` 上报，宿主超长直接拒绝；更大的负载仍需要 `peer data` 续行（未实现）。
- **52x 上的 DIS 读取要求已认证链路**：明文读回 ATT 0x0F，Just Works 加密后仍是
  0x05。MITM 只能靠 passkey（SC 配对在本台失败），而 passkey 流程又依赖
  `sec.bond` 拆分（见 13.2.1），所以 `diss` 套件只验证 DIS 可被发现并枚举，
  不读它的值。
- **被测例程自身的两个缺陷**（测试侧只绕开、不修）：`iBeacon` 的
  `cmd_diss adv_update` 解析不出 UUID 字符串（只写进两个字节）；`periodic_adv`
  的 `cmd_diss keep_per stop` 停不住周期数据的更新。

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
10. **烧录偶发失败**：DUT 走 `sftool` 时偶尔报
    `Failed to download stub: Timeout("waiting for shell prompt")`（板子没进入
    下载模式），重跑即可；不要把它当成用例失败去改用例。

### 11.2 两个例程本期未覆盖的原因

- `throughput`：例程本身要求两块板都跑它自己（一边 central 一边 peripheral），
  仪表要参与就得实现一条吞吐收发通路（大块数据双向持续传输），
  这不是“对端仪表”而是第二个 DUT，A/ B 两块板对跑更合适。
- `multi_connection`：例程的价值在一条 DUT 同时连多个对端；
  一台仪表只代表一个对端，实验室需要多台 `bt_peer` 才能验证。

## 12. 已确认决策

- M1–M5 只做 BLE，不做 Q2（电流表）；M6 增加经典蓝牙，且只做 SPP（§14）。
- 新建专用固件工程，不扩展现有例程。
- 固件工程与 fixture 统一叫 `bt_peer`；能力用 `bt_peer:ble` / `bt_peer:classic`。
- 设计文档放在 `tools/sdk_test/BT_PEER_DESIGN.md`。
- 52X 与 58X 的小核 controller 都由 `AddLCPU` 把
  `example/ble/lcpu_general` 作为子工程编入，本工程不改它；
  经典半边只改 HCPU 侧（§14.1）。
- 经典半边不复用 `middleware/rt_bt_app/`，直接在本工程实现 `btc.*`（§10）。

## 13. 下一期 TODO

下面这些是现在还欠的：宿主层残余、固件扩展、未覆盖的例程。按“先修哪个”排序，
每条都写了为什么；已完成的实现细节不在这里重复，设计层面的结论已经写进
§6 / §7。

M6 已完成，不再列在下面：经典蓝牙脚手架（`btc.*` 五个 op、四个事件、
`BtClassicInstrument` 宿主 API，以及 `example/bt/spp` 与
`example/bt/test_example` 两个用例），实现说明见 §14。

### 13.1 宿主层的残余项

- 单类事件的上报速率没有自动化检查，靠“固件只在变化时上报”加“宿主丢弃过期
  事件”两道防线；出现新的高频事件源时要补一道启动期检查。
- 计数（`late` / `stray` / `duplicate` / `overflow` / `stale`）只出现在失败信息里，
  正常跑完不汇总；排查“数据看起来丢了”时它们本该先说话。
- 协议行识别假设提示符是 `msh ...>`；换提示符的固件会静默丢弃应答。
- 事件队列只允许向前消费，没有“看一眼再放回去”的能力。

### 13.2 固件

1. **`sec.bond` 拆成“开始 / 等待”两步**：现在它等到配对完成才回 `@ok`，
   测试线程被卡住，passkey 流程（对端要人在配对过程中输入数字）无法实现。
   配对结果已经有 `CONNECTION_MANAGER_PAIRING_SUCCEED/FAILED` 可用，加一个
   `wait=0` 与 `@evt bond link=.. result=..` 即可，宿主侧对应
   `start_bond()` / `wait_bonded()`。
2. **长负载续行**：请求预算只有 `PEER_CMD_MAX`（240 字符），`adv.data` /
   `gatt.write` / `file.push` / `btc.spp.send` 的载荷都被 hex 长度限制住。
   实现 `peer data <index> <hex>` 分段续写，宿主按 `max_line` 切块。
3. **能力细分**：`caps` 只有 `ble` / `classic` 两块整能力。没编
   `peer_periodic.c` 的镜像照样过 `expect_ready()`，然后在用例中途
   `@err unsupported`；`classic` 同样只有 SPP，A2DP/HFP 等尚未存在。
   拆成 `ble` / `ble.periodic` / `sec` / `classic.spp`，宿主按子能力发 API。

### 13.3 用例

1. **`cscpc` / `diss` / `tip_cts`**：与 `basc`/`cppc` 同族，套件照抄即可
   （`bt_peer:ble` + 发现/写读/订阅）。
2. **`throughput`**：需要仪表侧一条吞吐收发通路，或改用两块 DUT 对跑；
   见 §11.2。
3. **`multi_connection`**：需要实验室多一台 `bt_peer`（或多块 DUT）；
   一台仪表只代表一个对端。
4. **Secure Connections**：定位 52x DUT 与 58x 仪表做 SC 配对时双方都报
   DHKey check failed 的原因（`0x6B`/`0x7B`）。定位前 `hid`/`pair` 继续显式用
   `auth=legacy`，并在用例注释指向这里。

### 13.4 覆盖缺口

现有断言都是真的，这些是“例程的主体功能还没有对端能力去测”的缺口：

- **`central_and_peripheral`**：DUT 作为 central/GATT-client 的那一半
  （`conn_idx` / `search_svc` / `read_val` / `write_val`）。需要仪表先注册回环
  服务并广播，让 DUT 连上来。
- **`hrpc` / `basc` / `cppc`**：主体是 profile 客户端（HRP / BAS / CPS），需要
  仪表侧实现对应服务。
- **`pair`**：DUT 发起的配对（`set_sec`）与 `SHOW PIN` / `SHOW NC`；后者依赖
  passkey 流程（见 13.2.1）。
- **经典 profile 例程**：`music_sink*`、`music_source`、`hfp`、`a2dp_multi_connect`、
  `pan` 等。`btc.*` 只有 inquiry 与 SPP，没有 A2DP / HFP / AVRCP / PAN 的
  操作；要覆盖得先在仪表侧实现对应 profile（§10 的分类表）。
- 其余命令级差异（如 `adv_start` / `adv_stop`、按钮库路径）在写套件时按 README
  逐条补，不单独列。

## 14. 经典蓝牙实现（M6）

本节记录实际做出来的东西，供以后扩展 profile 和排查实验台问题。协议语义以
`example/test/bt_peer/src/peer_classic.c` 与 `peer_proto.c` 为准；BLE 部分
仍看 §5.4。

### 14.1 能力与构建

| 项 | 事实 |
|---|---|
| 能力来源 | `project/Kconfig.proj` 的 `PEER_CAPS`：`CONFIG_PEER_CLASSIC` 打开时 `"ble,classic"`，否则 `"ble"`。`src/peer.h` 保留同值的 C 兜底。 |
| 开关 | `CONFIG_PEER_CLASSIC` 依赖 `SOC_SF32LB58X`，52X 无法选中。 |
| 58X 覆盖 | `project/sf32lb58x/proj.conf` 提供两部分配置：58X 构建本身需要的 `LCPU_CONFIG_V2` / `HCI_DISABLE` / `LCPU_RUN_SEPERATE_IMG`，以及经典栈需要的 `PEER_CLASSIC` / `BT_FINSH` / `BT_PROFILE_CUSTOMIZE` / `CFG_SPP_CLT` / `CFG_SPP_SRV` / `BT_CON_NUM_CUSTOMIZE` / `CFG_MAX_BT_ACL_NUM=2` / `CFG_MAX_BT_BOND_NUM=2`。构建自动合并 `<chip>/proj.conf`（`tools/build/building.py` 的 `ResolveBoardConfig`），所以 52X 构建看不到它。 |
| LCPU（52X/58X） | `project/SConstruct` 无条件调用 `AddLCPU(...)`，把 `example/ble/lcpu_general` 作为 LCPU 子工程编入。`example/rom_bin` 的镜像不参与构建，`src/SConscript` 也不引入 `PEER_LCPU_IMG`。 |
| 体积 | 58X 的 `build_<board>/output/main.bin` 约 889 KB；52X 的约 543 KB。 |
| `@info` | 经典构建在 `caps=` 后追加 `btc=1`：`@info proto=1 fw=bt_peer-m6 caps=ble,classic btc=1 ...`。`btc=1` 要求构建带经典半边（`#ifdef PEER_CLASSIC`）**且** `peer_classic_init()` 成功（IPC 对象与事件回调都就位），与 `caps` 同级表示“这份镜像的经典半边可用”，不代表协议栈已就绪；栈是否可用由第一次 `btc.*` op（通常是 `btc.local`）回答。宿主的能力校验仍只看 `caps`，SPP 用例另断言 `btc=1`。 |
| 固件版本 | `PEER_FW_ID` = `bt_peer-m6`。 |

```bash
cd example/test/bt_peer/project
scons --board=sf32lb52-lcd_a128r16_hcpu -j8          # caps=ble
scons --board=sf32lb58-lcd_n16r32n1_qspi_hcpu -j8    # caps=ble,classic
```

### 14.2 `btc.*` 协议

命令：

| op | 参数 | 成功响应 | 主要失败码 |
|---|---|---|---|
| `btc.local` | `timeout_ms`（默认 10000，仪表侧预算） | `@ok addr=<12hex> name=<hex> role=server` | `not_ready classic` / `not_ready stack` / `timeout addr` / `invalid_arg btc.local` |
| `btc.inquiry` | `duration_ms`（默认 10000）、`max_rsp`（默认 0 = 不限，0–255；达到该数量即提前结束 inquiry） | 每台设备一条 `@evt btc.dev`，结束后 `@ok count=<n> first_seq=<n>` | `busy inquiry`（已有 inquiry/connect/pair）、`state inquiry`、`timeout inquiry`、`invalid_arg btc.inquiry` |
| `btc.pair` | `addr=<12hex>`、`timeout_ms`（默认 20000，仪表侧预算） | `@ok addr=<12hex>`；绑定链路密钥，跨重连保留 | `invalid_arg addr/timeout_ms`、`busy pair`、`state pair`（配对失败）、`timeout pair`（没有任何结果上报） |
| `btc.connect` | `addr=<12hex>`、`uuid`（2/4/16 字节，显示序，默认 `1101`）、`timeout_ms`（默认 10000） | 先 SDP，再建 SPP：`@ok link=<n> addr=<12hex> channel=<n> role=0`，同一条链路的 `@evt btc.conn` 先到 | `invalid_arg addr/uuid/timeout_ms`、`busy connect`（inquiry/pair/connect pending）、`state connect`（SDP 请求被拒或结果失败、ACL 被拒）、`timeout connect`（SDP 或 page 超时） |
| `btc.disconnect` | `link=<n>` | `@ok`（只表示已发起断开） | `state link`（没有这条链路）、`write_failed disconnect` |
| `btc.spp.send` | `link=<n> len=N data=<hex>` | `@ok bytes=N` | `state link`、`invalid_arg len`（超过该链路 MTU）、`no_mem spp`、`write_failed spp` |

事件：

| 事件 | 字段 | 语义 |
|---|---|---|
| `btc.dev` | `addr=<12hex> name=<hex> rssi=<dBm>` | inquiry 报告，每条设备一行；`name` 是 UTF-8 字节的 hex |
| `btc.conn` | `link=<n> addr=<12hex> channel=<n> role=0/1` | SPP 链路建立；`role` 是仪表的角色，0 = 仪表发起，1 = DUT 发起（自动 accept） |
| `btc.disc` | `link=<n> reason=<n>` | 断开确认；栈没给原因时为 0 |
| `btc.data` | `link=<n> len=N data=<hex> off=<n>` | 收到 SPP 数据；超过 `PEER_MAX_PAYLOAD`（460）按 `off` 拆分 |

固件依赖 `middleware/bluetooth` 的 finsh 接口新公开的三个事件（定义在
`service/bt/bt_finsh/bts2_app_interface_type.h`）：`BT_NOTIFY_SPP_SDP_CFM`
（SDP 结果）、`BT_NOTIFY_COMMON_PAIR_CFM`（配对结果）、
`BT_NOTIFY_COMMON_PAIRED_DEV_RECORD`（SM 的已配对设备列表）。`btc.pair`
先等配对 CFM 或加密事件；两者都没有结果时查询该列表，最后退到 connection
manager 的 bond 表。列表只有查询真的返回时才算证据；等待超时时仅保留配对
事件已记录的结果（`g_clc_pair_result`）。列表是持久的、回调也没有请求 id：
同一地址已有的旧 bond 可能让本次失败被报成 `@ok`，这是已知限制。

语义与坑：

- **`uuid` 是显示序（MSB first）**，与 BLE 的 LSB-first 相反。SPP 写
  `1101`，同样的数在 `gatt.*` 里是 `0111`。固件按长度接受 2/4/16 字节。
- **`name` 是 UTF-8 字节的 hex**，不是转义字符串；宿主的
  `_decode_name()` 解码，非法 UTF-8 用替换字符，非法十六进制直接报协议错。
- **经典 `link` 与 BLE `link` 是两个独立命名空间**。经典 id 从 1 起、只在
  表里复用；把其中一个传给另一种 op 是最难查的错误，宿主侧不做转换。
- **`btc.inquiry` 不限设备类**：上游的 `bt_interface_start_inquiry_ex()` 走
  `bt_start_inquiry()`，后者在非 `BSP_BQB_TEST` 构建里把 `dev_mask_cls`
  固定成 `BT_DEVCLS_AUDIO`，SPP-only 的 DUT（COD 0x240704）永远收不到。
  固件因此直接调 `gap_discov_req(tid, max_rsp, seconds, &filter, TRUE)`，
  `filter.filter = BTS2_INQ_FILTER_CLEAR`、`dev_mask_cls = 0`；
  `DISCOVER_IND` / `INQUIRY_CMP` 由 finsh 层对任何调用者无条件上报，不依赖
  `bt_start_inquiry()` 的 `inquiry_list` 簿记。这是示例固件里的绕过，
  不是对 `middleware/bluetooth` 的修改。
- **先 `btc.pair` 再 `btc.connect`**。SPP 的 RFCOMM 通道要求认证链路，没有
  bond 时第一次 connect 会失败：DUT 侧报 HCI 0x06（PIN or key missing），
  仪表侧以 `@err timeout connect` 结束，而不是给出更清楚的错误；
  `btc.connect` 不做隐式配对。`state.reset` 会删掉仪表侧
  的 bond，而 DUT 可能还留着旧密钥：此时第一次 `btc.pair` 以
  `@err state pair` 失败，DUT 在 KEY_MISSING 后删掉记录，第二次即成功。
  SPP 用例按此重试一次。
- **`btc.connect` 内部先做 SDP**：`bt_interface_spp_client_sdp_search_req()`
  等 `BT_NOTIFY_SPP_SDP_CFM` 拿到远端服务通道后才发起 SPP 连接；SDP 与建链
  共享 `timeout_ms + 2 s` 预算，搜索慢就留给建链的时间少。
- **`channel` 是每端各自的本地服务通道**，两端数值不同（实测仪表 8、DUT 1）。
  仪表的 `channel`（`btc.conn` / `btc.connect` 返回）用于 `btc.spp.send`；
  DUT 的 `chl` 用于它自己的 `send_data`。不要断言两者相等。
- **请求行仍是 240 字符**（`PEER_CMD_MAX` / `@info max_line`），所以
  `btc.spp.send` 的实际载荷约 90–100 字节（id 越长越少）。`btc.data` 是
  输出方向，上限是 `PEER_MAX_PAYLOAD`=460/行。
- **`btc.connect` 的 `@ok` 等 SPP 链路建立**；`btc.disconnect` 的 `@ok`
  只表示已发起，`@evt btc.disc` 才是断开确认。用例必须两步都等。
- **超时后的迟到连接会被断开**：`btc.connect` 超时后，若该地址的连接随后
  完成，固件直接断开它（`g_clc_fail_addr`），不会冒充下一条入向链路。布防
  只在 SPP 请求真正下发之后（SDP 同步拒绝/失败/超时不布防，避免吞掉下一次
  合法入站）；超时且 ACL 尚未建立时还会调 `bt_interface_cancel_connect_req()`
  取消 page。
- **inquiry / pair / connect 三者互斥**：任一 pending 时另外两个回 `busy`。
- **入向连接自动 accept**：`btc.local` 把仪表设成 pageable
  （`bt_cm_set_profile_target(0, BT_SLAVE_ROLE, 0)`），所以 DUT 主动连它时
  不需要额外命令。DUT 侧自己的 `spp connect` 不会先做 SDP，所以用例先发
  `spp search <addr> 2 1101` 再 `spp connect`。
- **链路上限**：仪表同时跟踪 4 条经典链路（`PEER_CLC_MAX_LINKS`）；表满时
  入向连接被立即断开，不会静默丢弃。
- **`state.reset`**：清掉 inquiry/pair/SDP 的进行中状态、断开全部经典链路
  （最多等 1 s）、清链路表、删除 bond；reset 期间和 reset 之后被丢弃链路的
  `btc.*` 事件不再上报，所以下一次测试不会看到上一状态的 `btc.disc`。

### 14.3 宿主 API

`tools/sdk_test/instruments/classic.py`：

```python
SPP_UUID = "1101"          # 显示序

@dataclass(frozen=True)
class ClassicDevice:
    address: str
    name: str = ""
    rssi: int | None = None

@dataclass(frozen=True)
class ClassicLocal:
    address: str
    name: str = ""

@dataclass(frozen=True)
class ClassicLink:
    link_id: int
    address: str
    channel: int = 1
    role: str = "client"    # "client" = 仪表发起， "server" = DUT 发起

class BtClassicInstrument:
    def reset(self) -> None: ...
    def local(self, timeout: float = 10.0, wait_ms: int = 5000) -> ClassicLocal: ...
    def inquiry(self, duration_ms: int = 10000, *, max_rsp: int = 0,
                timeout: float | None = None) -> list[ClassicDevice]: ...
    def pair(self, address: str, *, timeout: float = 20.0,
             timeout_ms: int = 15000) -> None: ...
    def connect(self, address: str, *, uuid: str = SPP_UUID, timeout: float = 15.0,
                timeout_ms: int = 10000) -> ClassicLink: ...
    def disconnect(self, link: ClassicLink) -> None: ...
    def spp_send(self, link: ClassicLink, data: bytes, *, timeout: float = 10.0) -> None: ...
    def wait_data(self, *, link: ClassicLink | None = None,
                  since_seq: int | None = None, timeout: float = 10.0) -> bytes: ...
    def wait_connected(self, *, since_seq: int | None = None,
                       timeout: float = 15.0) -> ClassicLink: ...
    def wait_disconnected(self, *, link: ClassicLink | None = None,
                          since_seq: int | None = None, timeout: float = 10.0) -> None: ...
```

- `BtPeer.classic` 只在 `@info caps` 含 `classic` 时交出实例，否则抛
  `InstrumentCapabilityError` 并说明该烧哪份镜像。
- `inquiry()` 用 `command_events(..., boundary="firmware")`，以 `@ok`
  里的 `first_seq` 为界，上一轮迟到的 `btc.dev` 不会被算进这一轮。
- `pair()` 先绑密钥，再 `connect()`；SPP 通道要求认证链路，跳过这一步
  第一次连接会以超时告终。配对跨重连保留；`reset()` 后对端可能还留旧密钥，
  第一次 `pair()` 会抛 `InstrumentCommandError`（`state pair`），重试一次即可。
  列表回退的已知限制（同一地址的旧 bond 可能让失败报成功）见 §14.2。
- `connect()` 不传 channel：仪表先做 SDP，再从结果推导本地服务通道，宿主用
  返回的 `channel` 与 DUT 打印的信道各自核对（两端数值不同，不能相等）。
  连接预算是 SDP + 建链共享的，真板上 RFC DLC 协商偏慢时要把 `timeout_ms`
  抬到 20 s 以上（SPP 用例用 25000，宿主等待 40 s）。
- `wait_connected()` 把 `role=0/1` 映射成 `client`/`server`
  （仪表的角色）；默认拿队列里最早的 `btc.conn`，要区分本次与更早的连接
  就显式传 `since_seq`。
- `wait_data()` 只返回一条 `btc.data` 的载荷，不暴露 `off`；需要拼接
  460 字节以上的包时，用 `console.next_event("btc.data", ...)` 自己处理。
  别的 link 的数据留在队列里，不会被消费掉。
- `reset()` = `state.reset` + `clear_events()`，与 `BleInstrument.reset()`
  同形。
- 地址在发送前去掉 `:` / `-` 并转小写；固件回报的也是裸十六进制。
- 单测：`tools/sdk_test/tests/test_instruments_classic.py`，19 项，覆盖
  reset、local、inquiry 边界、name 解码、pair、connect/角色、双向数据、
  跨 link 事件不被误消费、长度校验。

### 14.4 用例

| 用例 | 用例文件 | DUT | 仪表 | 步骤 |
|---|---|---|---|---|
| `bt_spp.peer` | `example/bt/spp/testcase.yaml` + `pytest/test_spp.py` | 52X，SPP server（开机即 page scan） | 58X，`bt_peer:ble,classic` | 8 步 |
| `bt_test_example.ble_peer` | `example/bt/test_example/testcase.yaml` + `pytest_ble/test_ble_peer.py` | 52X，BLE 半（`sifli_app` 服务 + legacy 可连接广播） | 任一提供 `ble` 的镜像（52X 或 58X 都满足） | 9 步 |

`bt_spp.peer` 的 8 步：

1. DUT 到 shell，经典栈 ready，取 DUT 的本地名；
2. 仪表确实是经典镜像（`caps` 含 `classic`、`btc=1`），`classic.reset()`，
   取仪表自己的地址；
3. 5 s inquiry（最多重试 3 次），按名字匹配、按地址去重后确认只有一台 DUT；
4. `btc.pair` 先绑密钥（失败时重试一次，覆盖 reset 后对端留旧密钥的情况）；
5. 仪表 `connect` 到 DUT（`timeout_ms=25000`，SDP + 建链共享预算），DUT 打印
   `spp connect success!!!`，DUT 的 `chl <n>` 与仪表的 `channel` **各自使用**
   （不断言相等）；
6. 仪表 `spp_send` 四个字节，DUT 的 `[SPP ... RX]` 逐字节断言；
7. DUT 侧 `spp send_data <仪表地址> <DUT 的 chl>`，仪表 `wait_data` 收到
   `This is spp test!!!`；
8. 仪表断开并等 `btc.disc`，DUT 先 `spp search <仪表地址> 2 1101`（自己的
   `spp connect` 不做 SDP）再 `spp connect`，仪表 `wait_connected` 得到
   `role == "server"`，再发一次数据确认新链路可用。

两个用例的 `project_dir` 都是 `project`（`example/bt` 是扁平布局）。
`bt_test_example.ble_peer` 覆盖 BLE 半：连接、**legacy 配对**（该例程的值属性
与 CCCD 带 `PERM(WP, UNAUTH)`，不加密写入会被拒为 ATT 0x0F）、发现、写读、
订阅、5 s 的 4 字节 CCCD 通知，以及写入 `11 0A 14 00` 触发的 20 个 10 字节
通知突发。

### 14.5 实验台接线

仪表烧一次 58X 固件，之后不再由框架烧写；DUT 照常由 runner 烧写。

```yaml
testbed:
  - id: <testbed 名>
    platform: <DUT 板名>
    console: {port: <DUT 串口>, baud: 1000000}
    connections:
      - fixture: bt_peer:ble,classic
        console: {port: <仪表串口>, baud: 1000000}
```

```bash
# 有板子的机器上先起 agent（lab YAML 在仓库外维护，端口用占位符，不要写死 COM）
python tools/lab_agent/runner.py -c <lab yaml>

# 跑用例：平台用 -p 指定，run 没有 -b
sdk.py test run -T example/bt/spp -p <DUT 板名>
```

- 仪表的端口只写在 binding，用例只写 `fixture: bt_peer:ble,classic`。
- 一块仪表只有一条 console 流：不要为两项能力配两条 connection（§7.4）。
- 本机实验台的 lab YAML 在仓库外，不属于 SDK 仓库；换机器时按
  `tools/lab_agent/examples/runner.local.yaml` 的注释重建。

### 14.6 验证状态

已验证（真板）：

- `bt_spp.peer` **通过**：`1 test(s): 1 passed`，75.3 s（含 build + flash；
  子模块基于 `origin/main` 重做、本地提交 `9d5f86d`（未推送）后复跑）。
- `bt_spp.peer` 反向验证（执行过）：把 `peer_classic_send()` 改成直接回
  `@err ... write_failed sabotage` 后重编/烧录，结果 `2 failed, 6 passed`。
  `test_peer_sends_data_to_the_dut` 红在
  `InstrumentCommandError: the instrument rejected btc.spp.send with
  write_failed: sabotage`，DUT 没有 `[SPP ... RX]` 行。恢复后复跑通过。
- `bt_test_example.ble_peer` **通过**：`1 test(s): 1 passed`，52.0 s
  （新基线复跑，含 build + flash）。
- `bt_test_example.ble_peer` 反向验证（执行过）：去掉
  `ble_app_gatts_set_cbk()` 里的 `memcpy` 后重编/烧录，结果
  `2 failed, 7 passed`。`test_peer_writes_and_reads_the_value` 红在
  `expected /Updated app value from:\d+ to:34\b/ within 10s`，实际日志是
  `Updated app value from:0 to:0`；突发通知也因触发值未写入而红。已恢复。
- 52X 与 58X 两份仪表固件、两个 DUT 工程都能构建；58X 的
  `output/main.bin` 约 889 KB。
- 宿主层 19 项单测通过：
  `python -m unittest discover -s tools/sdk_test/tests -p "test_instruments_classic.py"`。
  inquiry 上限的反向验证已做并记录在测试文件头：默认值改回 10、或不把
  `max_rsp` 透传（写死 0），对应断言分别变红。

反向验证是在重做子模块之前做的；两个破坏点分别走 `btc.spp.send` 与 BLE
write 路径，之后针对 `origin/main` 的 inquiry 改动不经过这两条路径，所以
记录仍然有效。

真板备注：DUT 的 sftool 烧录出现过一次 exit 101（`sf32lb52-lab` 的已知现象），
重跑即通过；不要把它当成用例失败。

未验证（不要当成已通过）：

- **Classic 失败路径（#1/#3/#4/#5）没有宿主单测能覆盖**：它们都在固件状态机里，
  现有 `test_instruments_classic.py` 只钉协议形状。上板/人工验证清单：
  1. **init 失败**：让 `rt_mutex_create()` / `rt_event_create()` 返回
     `RT_NULL`，或让事件回调注册返回非 OK；`@info` 不应含 `btc=1`
     （`peer_classic_available()` 为假），SPP 用例的 `btc=1` 断言应在
     `@info` 处变红，而不是等到第一个 `btc.*` 超时。
  2. **connect 超时取消 page**：对端不在可连接范围内时 `btc.connect` 回
     `@err timeout connect`；取消失败才会出现
     `bt_peer: cancel page to <addr> returned N` 日志。
  3. **超时后立即入站**：`btc.connect` 超时后让 DUT 立刻 page 仪表，仪表不应
     把它当成入向链路（不发 `btc.conn`，直接断开）；同一地址下一次成功建链或
     `state.reset` 之后，入站连接恢复正常。
  4. **ACL 失败不串扰**：在 SDP 阶段制造一次同地址的 ACL 失败（对端拒绝），
     本次 `btc.connect` 不应提前失败；失败结果只属于已真正下发 SPP 请求的
     那一次。
  5. **SDP 期间建链**：SDP 等待期内 DUT 主动连仪表（同地址），
     `btc.connect` 应直接报告这条已建立的链路，而不是再发一次 SPP 请求后超时。
  6. **pair 列表回退超时**：让 `BT_NOTIFY_COMMON_PAIRED_DEV_RECORD` 不回包，
     列表回退不应凭空报成功；已有旧 bond 时仍受下面的已知限制影响。
- SPP 大包（>460 字节）拆分与 `off` 拼接没有真板数据；两个用例都只发小包。
- 经典链路表满（4 条）与多 bond 并存只在代码里处理过，未做真板验证。
- **BT 任务栈水位未测**：经典回调跑在 Bluetooth 任务上，该任务栈约 3 KB
  （`BTS_TASK_SIZE`），共享的 `peer_emit_evt()` 仍在这个任务上占约 1 KB
  行缓冲。`btc.data` / `btc.conn` 路径的真实栈水位没有在板上测量，属于
  待确认项；经典回调自己的 984 字节格式化缓冲已改成 `.bss`
  （`s_clc_data_line`），不计入这条。
- 52X 仪表与 58X 仪表在同一个实验室共存时，`addr` / `name` 的分配
  （BLE 侧 UID 派生，经典侧由栈上报）没有并列验证过。
