# Lab Agent — 机器侧 runner

设计文档：[`../sdk_test/DESIGN.md`](../sdk_test/DESIGN.md)
SDK 侧对应物：[`tools/sdk_test/`](../sdk_test/README.md)

---

## 它是什么

Lab Agent 跑在**插着板子的那台机器**上。它拥有测试用例永远不该知道的东西——
哪个串口、哪根调试探针、哪个 mux 通道——并通过一个本机 HTTP 接口一次发放一个 testbed。

分工是刻意的：SDK 侧决定**跑什么、能不能跑**（用例的 `depends_on` 对板子自己的
`features.yaml`），agent 只管**物理上有什么、哪个空闲**。所以同一份 binding 文件
两边共用一套 schema（由同一份代码解析），不会各自漂移。

binding 里**不写**板子能力——那是板子自己的声明，同一事实写两处正是它们会互相
矛盾的来由。

## 怎么跑

```bash
# 起服务
python tools/lab_agent/runner.py --config tools/lab_agent/examples/runner.local.yaml

# 只校验配置（含 J-Link 探针、pyserial 检查），不起服务
python tools/lab_agent/runner.py -c <cfg> --check

# 只列出本机 testbed
python tools/lab_agent/runner.py -c <cfg> --list

# 看每个请求
python tools/lab_agent/runner.py -c <cfg> -v
```

配置模板：`examples/runner.local.yaml`（**带注释，照着抄**；真实的 COM 口不要提交）。

## HTTP 接口

```
GET  /health
GET  /testbeds
POST /allocate                     {platform, fixtures, testbed_id, holder}
POST /release                      {testbed_id, holder}
POST /testbed/{id}/firmware        body = a firmware package (tar)
POST /testbed/{id}/prepare         {firmware, reset}
POST /testbed/{id}/reset
POST /testbed/{id}/flash           {firmware}
POST /testbed/{id}/console/open           [?stream=dut|bt_peer:ble]
POST /testbed/{id}/console/close          [?stream=dut|bt_peer:ble]
GET  /testbed/{id}/console/read?cursor=N&timeout=SECONDS[&stream=...]
POST /testbed/{id}/console/write   {data}   [?stream=dut|bt_peer:ble]
```

`stream` names which console on the testbed: `dut` (the default, so existing
callers are unchanged) or a fixture the allocation declared. A fixture with a
`console:` entry in its connection is an **instrument** — the `bt_peer` board,
say — with its own port, no build and no flash; the agent opens and closes that stream
with the testbed, and only for a fixture the allocation asked for.

`firmware` is the digest the upload answered with, not a path: the package is
unpacked under its own name and flashed from there, so this machine never needs
to see the one that built it. See `sdk_test/firmware.py`, which both sides
import.

**为什么控制台不是流式的。** 客户端问"游标 N 之后有什么"，服务端最多阻塞 `timeout` 秒。
流式意味着 chunked 响应、断线重连、以及"客户端读到哪了"的第二个真相来源；
游标只是一个整数，连接断了也不丢——下一次调用从原地继续。

**为什么用 stdlib `http.server`。** agent 就跑在硬件旁边的 PC 上，
为了它引入一套 ASGI 栈是纯部署负担。

## 几个关键实现决定

- **互斥占用**：`TestBedRuntime` 自己持锁，第二次申请直接失败并报出持有者是谁。
  两个测试程序共用一块板子产生的现象看起来像固件 bug，事后极难诊断。
- **复位走带外**：串口 DTR 在这套硬件上不是复位线，所以复位经调试探针
  （`transport/jlink.py`），或未来的继电器（`transport/relay.py`）。
- **烧录复用构建产物**：直接回放 `build_<board>/download.jlink` 里的
  `loadbin`/`loadfile` 行，只替换探针前导码（以便用 `-SelectEmuBySN` 选探针）。
  镜像地址来自分区表和打包步骤，在这里重算等于制造第二个真相来源。
- **`JLINK_DEVICE` 从板子自己的 `rtconfig.py` 读**，不在 runner 配置里重复一遍。
  用正则解析而不是 `exec`，配置文件永远无法在 runner 进程里执行代码。
- **继电器会大声失败**：`transport/relay.py` 每个方法都抛 `RelayNotAvailable`。
  静默 no-op 会让一个用例"通过"却从未真正断电——比失败更糟。
- **控制台读取是游标式的，且缓冲区有上限**：`max_buffer` 默认 4 MiB，
  裁掉的字节用绝对游标记录，所以截断不会让后续读取重复或报错。

## 自测

```bash
python -m unittest discover -s tools/lab_agent/tests -p "test_*.py"   # 105 个，不需要硬件
```

控制台与探针在**传输层**被伪造（`TestBedRuntime` 上的两个注入点），
所以它们之上的一切——缓冲、游标运算、占用、HTTP 接口——都是真实实现。
`test_server.py` 会在临时端口上起真服务，用真 HTTP 驱动。

## 未完成

- `transport/relay.py` 只有接口（`SwitchReset.exe` 不在仓库里，在 Jenkins 上）。
- `fixture_drivers/` 只有 passive 驱动 + 注册接口；active 驱动（对端板、手机、仪器）未实现。
  仪表目前走的是"带控制台的 fixture"这条更窄的路：`TestBedRuntime` 按 fixture 维护独立的
  `ConsoleStream`（路由参数 `stream=`），由 pytest 侧驱动，不走 driver 接口。
- 控制平面 / dashboard / 跨机调度未实现（MVP 是单机 runner + 手工 binding）。
