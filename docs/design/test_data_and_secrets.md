# 测试数据与凭据管理设计

| 项目 | 内容 |
|------|------|
| 范围 | `tools/sdk_test`（发现 / 编排 / pytest 插件）、`tools/lab_agent`（binding 与分配）、`testcase.yaml`、pytest 用例 |
| 状态 | P1 已实现并评审通过（P2/P3 未做）；真板链路未验证 |
| 相关工作 | `tools/sdk_test/DESIGN.md`（框架总设计）、`AGENTS.md`（自动化测试一节）、`docs/source/zh_CN/app_note/autotest_guide.md` §10 |
| 关键约束 | 凭据不进仓库、不进报告；向量不进 git 历史；用例只写逻辑名 |

## 0. 先看这里

四句话概括：

1. **凭据放 runner 侧**（跑 `sdk.py test` 的那台机器），文件在仓库外；binding 只声明能力，不出现 AP 的 ssid/密码。
2. **向量放外部内容存储**，仓库只留清单；本地内容寻址目录既是缓存也是校验基准。
3. **用例只写逻辑名**（fixture spec、asset name），值和路径由框架解析。
4. 秘密与向量**共用"逻辑名 → 解析"这一层**，但生命周期相反：秘密稀缺且用完即毁，向量大、共享、必须缓存。

## 1. 问题

给 `example/rt_device/wifi/sta` 写上板用例需要连接指定 AP，但：

- SSID/密码硬编码在例程宏里（`example/rt_device/wifi/sta/src/main.c:31-32`），无 Kconfig、无 `proj.conf` 项；
- 例程启动约 5 s + 1 s 后自动去连该宏（`main.c:187,199,203`）；
- 仓库里已有运行期入口，不用重编译：msh `wifi join <SSID> <PASSWORD>`（`rtos/rtthread/components/drivers/wlan/wlan_cmd.c:221-258`，导出在 `:564-565`）。

更大的问题是算法类测试会有大量比对数据（golden / 采样 / 波形），它们同样不适合进本仓库的版本管理。两者都不该被写进测试脚本，但也不是同一类东西。

## 2. 已确认决策

| 问题 | 决策 | 理由 |
|---|---|---|
| 凭据放哪 | runner 侧 `params.yaml`（仓库外） | 不新增 agent 契约；CI 可用 secret 注入；P1 仍经 agent 的 console 写入传输，只适用于可信网络（§7.7） |
| AP 参数放哪 | 全部在 runner 侧 `params.yaml`（`ssid` / `security` / `psk`） | 同一 fixture spec 的 testbed 可互换，参数按 spec 查表；binding 无需描述 AP |
| binding 是否描述 AP | 不描述，只声明能力 | 与实验室 AP 不匹配属于实验台声明错误，由用例失败暴露，框架不做交叉校验 |
| 向量放哪 | 仓库清单 + 外部内容存储 + 本地缓存 | git 历史只增不减；多实验室共享一份字节 |
| 是否建"向量管理系统" | 不建 | 清单 + 解析器 + 存取命令即可，治理规则写进文档 |
| wifi 例程是否改造 | 不改 | 测试容忍自动连接窗口；代价见 §10.3 |

## 3. 术语

| 术语 | 含义 |
|---|---|
| **host / runner** | 跑 `sdk.py test run` 的机器；pytest 子进程也在这里 |
| **Lab Agent** | 插着板子的机器上的常驻服务，只管占用、烧录、复位、串口 |
| **fixture** | 与 testbed 接线/环境相关的**能力**描述，由 agent 在 binding 上匹配 |
| **fixture 类型契约** | 框架定义某个 fixture 类型有哪些选项、每个选项从哪来；见 §7.2 |
| **params** | 值来自 runner 侧文件/环境的键值，本文只放秘密与非接线参数 |
| **asset / 向量** | 测试输入或比对数据文件，按内容哈希寻址 |
| **secret** | 标记为不可出现在日志/报告中的值 |

## 4. 范围

### 4.1 本期覆盖

- `testcase.yaml` 的 `params:` / `assets:` 声明；
- fixture 类型契约与 binding 选项校验；
- runner 侧参数解析、下发、脱敏；
- 资产清单、内容寻址存储、取用与校验；
- pytest 侧 `params` / `fixture_options` / `asset` 三个接口；
- CLI 与 CI 接入方式；
- `wifi/sta` 端到端实例。

### 4.2 不在本期

- agent 侧持有并使用秘密（fixture driver 执行模板命令）；
- agent 认证 / TLS；
- 从 conan 包体系取资产；
- 资产的自动上传（只提供 `add` 生成清单条目，上传仍人工）；
- 令牌轮换的自动化。

## 5. 现状与缺口

| 现状 | 位置 | 对本设计的影响 |
|---|---|---|
| `harness_config` 是 `additionalProperties: false` | `tools/sdk_test/schema/testcase.schema.json:77-107` | 新声明必须进 schema，不能靠额外键 |
| pytest 子进程完整继承宿主环境 | `tools/sdk_test/harness/pytest.py:224` | 今天唯一可用通道；本设计不再依赖它传值 |
| 框架只注入 7 个 `SIFLI_TEST_*` | `tools/sdk_test/harness/pytest.py:42-62` | 新增一个数据文件路径变量 |
| connection 的未知键原样保留 | `tools/sdk_test/binding.py:86-106`、`model.py:83-96` | 只拒绝契约声明为 `source: params` 的键；其余键（如仪表的 `console`）是 driver 私有的 |
| agent 只暴露 testbed 的固定字段 | `tools/lab_agent/testbed.py:804-839` | 本设计不需要新增字段：runner 只需知道 testbed 提供了该 fixture |
| fixture 能力匹配已实现 | `tools/sdk_test/fixtures.py:106-127`、`model.py:140`、`testbed.py:783-800` | 直接复用；参数化类型要求 spec 精确相等（§7.2），其余仍按能力子集匹配 |
| `fixture:` 目前只表达能力，不承载数据 | `tools/lab_agent/fixture_drivers/__init__.py:29-71` | 资产不走 fixture |
| agent HTTP 无认证 | `tools/lab_agent/examples/sifli-lab.yaml:17`（代码中无认证） | P1 的 psk 作为 console 负载经这条通道明文传输，只适用于本机/可信实验室网络 |
| 控制台缓冲是内存 4 MB、不落盘 | `tools/lab_agent/testbed.py:130-191` | 密码留在缓冲里，直到被 4 MB 新输出顶掉或 agent 重启 |
| 日志在实例跑完后一次性写入 | `tools/sdk_test/runner.py:664,725-746`、`report.py:413-465` | 脱敏在内存结构上做，所有序列化继承 |
| 指南 §10 写"向量就是一个 fixture"，resolver 未实现 | `docs/source/zh_CN/app_note/autotest_guide.md:1303-1321` | 本文修正该表述，见 §15 |

## 6. 总体设计

```
仓库内（可评审）                      runner 侧（仓库外）              内容存储（仓库外）
┌────────────────────────┐      ┌────────────────────────┐      ┌────────────────────────┐
│ testcase.yaml          │      │ params.yaml            │      │ sha256/<digest>        │
│   fixture: wifi_ap:5g  │      │   fixtures.wifi_ap:5g  │      │   (immutable, dedup)   │
│   params: {cloud.token}│      │     .ssid/.psk         │      └───────────┬────────────┘
│   assets: [golden]     │      └───────────┬────────────┘                  │
├────────────────────────┤                  │                               │
│ fixture_types.yaml     │                  │                               │
│   wifi_ap -> options   │                  │                               │
├────────────────────────┤                  │                               │
│ assets.yaml            │                  │                               │
│   name -> sha256/size  │                  │                               │
└───────────┬────────────┘                  │                               │
            │                               ▼                               ▼
            └──────────────────────▶ runner（解析 + 校验 + 脱敏）◀───────────┘
                                            │
                              temp JSON（跑完删除）
                                            ▼
                                     pytest 子进程
```

三条原则：

1. **归属与位置分离**：谁的语义谁定义（框架定选项名、实验室声明接线与能力、仓库定清单），但字节/秘密不必与定义者同处一地。
2. **解析在 runner**：pytest 跑在 host，它需要的值由 runner 解析后交过去；agent 不参与参数解析。
3. **秘密与向量生命周期相反**：秘密不缓存、用完即删；向量必须缓存、每次校验。

**P1 的传输边界（必须如实理解）**：`psk` 最终要写进 DUT 控制台，而控制台由 agent 转发——测试调用 `dut.send_line()` 时走 `POST /testbed/{id}/console/write`（`tools/lab_agent/server.py:400-413`）。因此 psk **每次运行都以明文经过无认证的 agent HTTP**，并留在 agent 的内存缓冲与 DUT 侧日志里（`wlan_mgnt.c:945,1032,1322` 以 debug 级别打印密码）。本设计不宣称"秘密不出 host"；消除这条路径需要 §12 的 P3。

## 7. 凭据与参数链

### 7.1 三处数据，三种职责

| 数据 | 存放 | 谁写 | 是否秘密 | 例子 |
|---|---|---|---|---|
| 能力声明 | binding（仓库外） | 实验室 | 否 | `fixture: wifi_ap:5g` |
| 类型目录与选项契约 | `tools/sdk_test/fixture_types.yaml` | 框架维护者（随代码评审） | 否 | `bt_peer` 的能力有 `ble`/`classic`；`wifi_ap.ssid` 来自 params |
| AP 参数与秘密值 | `params.yaml`（仓库外） | 实验室 / 使用者 | `psk` 是 | `fixtures["wifi_ap:5g"]` 的 `ssid`/`security`/`psk` |

**同一个键只允许有一个来源**。`wifi_ap` 的所有选项都声明为 `source: params`，所以 binding 的 connection 只需要 `fixture` 一个键；出现 `ssid`/`psk` 这类未声明的键必须报错——防止有人顺手把密码写进 binding 再提交。

### 7.2 fixture 类型目录与选项契约

契约文件承担两件事：

- **类型目录**：这个 fixture 类型存在吗、它的能力 token 有哪些。回答"写用例的人能写什么"。
- **选项契约**：选项名、来源、必填/条件必填/秘密。回答"值从哪来"。

两层要分清：**类型目录在仓库**（框架知识，随代码评审），**实例库存在 binding**（这个 lab 实际有几台、接哪个口）。没有条目的类型保持现状（options 自由，不校验），这是对 `binding.py:97-100` 那条"不要第二套 schema"的保留。

```yaml
# tools/sdk_test/fixture_types.yaml
bt_peer:
  description: A second board running the peer firmware.
  capabilities: [ble, classic]      # closed vocabulary for this type
  # no options: its console is driver-private

spi_nor_flash:
  description: A NOR flash chip wired to an SPI bus.
  # no capabilities: the token after ":" names the bus (spi1), not a capability

wifi_ap:
  description: An access point the DUT can associate with.
  capabilities: [2g, 5g]
  options:
    ssid:
      source: params
      key: "fixtures.{spec}.ssid"
      required: true
    security:
      source: params
      key: "fixtures.{spec}.security"
      enum: [open, wpa2, wpa3]
      default: wpa2
    psk:
      source: params
      key: "fixtures.{spec}.psk"
      required_if: "security != open"
      secret: true
```

规则：

- `{spec}` 是 `testcase.yaml` 里**请求的那个 spec 原文**（含能力列表），按精确字符串查表。因此键可以从用例静态推导，`params check` 不需要连 agent。
- `source: params`：值来自 params.yaml；`key` 是键模板，只做一层 `{spec}` 替换，不递归。
- `source: binding`：预留给"公开值确实属于实验室"的 fixture 类型（例如某个仪表的地址）；`wifi_ap` 一个都不用。
- **校验只针对同名键**：binding 里出现契约声明为 `source: params` 的键（如 `ssid`/`psk`）→ 错误；契约没提到的键（如仪表的 `console`）是 driver 私有，原样保留。`binding check` 对未知键告警并列出契约允许的键，但不阻断。不做"未声明键一律拒绝"，否则会重蹈 `binding.py:97-100` 警告的第二套 schema。
- `required_if` 由 runner 解析时求值，条件里的键来自同一 fixture 的已解析值；`open` 的 AP 不需要 psk。
- **`secret` 的权威是契约**；params.yaml 自带的 `secret` 标记只对没有契约的键生效。
- **`default` 由 runner 填充**（agent 不参与参数解析），因此默认值也必须是非秘密的。
- `enum` 同时校验 `default` 与解析出的值；运行值不在枚举内 → plan 阶段 `ParamsError`。
- 严格校验的落点是 `binding check`；运行期只让出错的 testbed 不可用，不影响其它 testbed（与 `testbed.py:806-810` 刻意宽松的 describe 一致）。
- **可互换性**：框架把提供同一 fixture spec 的 testbed 视为等价（`tools/sdk_test/fixtures.py:1-14`）。如果两个实验室的 AP 参数不同，它们必须声明**不同的 spec**（如 `wifi_ap:5g,lab_a`），而不是引入第二个配置来源。
- **参数化 fixture 的 spec 是身份，不只是能力**：契约里含 `source: params` 选项的类型，binding 的 connection spec 必须与用例请求的 spec **精确相等**，不做能力子集匹配。否则"请求 `wifi_ap:5g`、落到 `wifi_ap:5g,lab_a`"时参数会按错误的键查表。非参数化类型不受影响，仍按能力子集匹配。
- 这条语义变更需要 bump `PROTOCOL`（`tools/sdk_test/version.py`，本次实现为 3）：新旧 SDK/agent 混跑会退回子集匹配，得到"错结果而不是报错"，正是该常量要防的情况。同理，把某个**已有类型**从目录条目改成参数化（新增 `source: params` 选项）也改变匹配语义，必须 bump。
- `required_if` 只支持 `<选项> ==|!= <不加引号的字符串字面量>` 一种形式（如 `security != open`）；左侧必须是同一 fixture 的已解析选项，其它语法或未知选项在契约加载时报错。不引入表达式求值器。
- `required_if` 引用的选项必须自身 `required: true` 或有 `default`，否则条件在它缺失时无法求值——契约加载直接拒绝。运行期若依赖仍未解析，报"依赖缺失"，而不是把依赖它的选项判为必填。
- key 模板只允许 `{spec}`；去掉合法占位符后仍含 `{` 或 `}` → 契约加载错误。
- `enum` 的每一项必须是字符串且不重复；有 `enum` 时 `default` 必须是其中一个字符串。运行期有 `enum` 时，非字符串值直接判为不在枚举，不做 `str()` 归一化。
- 契约里的类型名和选项名必须是字符串键；YAML 重复的**字面**键（类型或选项）在加载时报错，不静默取最后一个。YAML 合并键（`<<`）允许，扫描时跳过它。
- 同一类型内两个选项不得使用相同的 `key` 模板——否则参数合并后会以错误的名字报缺失。
- **P1 只实现 `source: params`**：契约里出现 `source: binding` 时在加载阶段直接报"未实现"，等 P3 再开（P1 没有把 binding 选项送到 runner 的通道）。
- 无契约的类型：options 原样保留，不校验（兼容现有实验台）。
- **目录条目可以没有 `options`**：`capabilities` 与 `description` 只做词表与文档，不改变匹配语义。只有声明 `source: params` 选项的类型才是参数化类型；给 `bt_peer` 加目录不会让 `bt_peer:ble` 失去落到 `bt_peer:ble,classic` 的能力。
- `capabilities` 是该类型的封闭词表。binding 里出现已知类型的未知 token → **告警不阻断**（实验室可扩展 `wifi_ap:6e` 这类能力）；类型没有 `capabilities` 时不做这项检查（`spi1` 这类总线名不是能力）。
- testcase 请求了目录里没有的类型 → **告警不阻断**，提示"该类型未在 fixture_types.yaml 中登记，实验室自定义类型允许"。`SIFLI_TEST_STRICT_FIXTURES=1` 时可升级为错误（可选）。
- **校验只针对参数化类型**：`binding check` 不检查没有 params 来源选项的类型，避免把 `bt_peer` 的 `console` 误判为未知键。
- 契约文件本身随代码评审；新增类型或改名是**破坏性变更**，按仓库现行原则不做兼容层。

命名规则：小写 `snake_case`；避开 binding 已占用的 `fixture` / `pinmap` / `mux`；不用点号。

### 7.3 binding

```yaml
testbed:
  - id: tb-52-wifi
    platform: sf32lb52-wlan-core_n16r16
    console: {port: COM8, baud: 1000000}
    connections:
      - fixture: wifi_ap:5g
```

- binding 只声明**能力**：这个台架有一个可用于 5G 的 AP。具体是哪个 AP、密码是什么，不在实验室配置里。
- `platform` 字符串必须与 `sdk.py test run -p` **逐字一致**：`select_testbeds` 做精确比较（`tools/sdk_test/fixtures.py:118-127`），`canonical_board_name` 只用于推导构建目录，不参与匹配。模板统一用不带核后缀的板名。
- agent 不需要新增任何接口：现有 `GET /testbeds` 已暴露 fixture spec（`tools/lab_agent/testbed.py:816-826`），足以让 runner 做分配。
- 真实 binding 必须留在仓库外（`E:\code2\lab`）。仓库内只放模板；`tools/lab_agent/examples/sifli-lab.yaml` 当前被跟踪且含真实 COM 口，需要先修（§14）。
- **不匹配时的表现**：params 里的 SSID 与实验室实际 AP 不同，结果是连接失败。按本设计这是实验台声明错误，框架不做交叉校验；用例应先扫描并断言目标 SSID 可见，把诊断收敛到"哪个 SSID 不在扫描结果里"（§10.4）。

### 7.4 params.yaml

嵌套 YAML，解析时扁平化为点号键；值可以是标量或 `{value, secret}`：

```yaml
# E:\code2\lab\params.yaml -- machine-local, never committed
fixtures:
  "wifi_ap:5g":
    ssid: SiFli-Test-5G
    security: wpa2
    psk:
      value: "..."
      secret: true

params:
  cloud.token:
    value: "..."
    secret: true
  sta.timeout_s: 30
```

`fixtures` 段的键就是 `testcase.yaml` 里请求的 spec 原文，所以带 `:` 时必须加引号。

`fixtures` 段各键的 `secret` 由契约决定，文件里不必重复标记；`params` 段的键没有契约，`secret` 由文件自己声明。

testcase 的 `params` 声明也可以标 `secret: true`（`{required: true, secret: true}`）。CI 常用环境变量供值、不落文件，标记必须跟着声明走，否则 ENV 来源的值既不会被脱敏，也不会触发回环守卫。三个来源的 secret 判定取 OR：契约 > 声明 > 文件；契约仍是 fixture 选项的最高权威。

扁平化规则：`fixtures` 段保留 `fixtures.` 前缀（键为 `fixtures.<spec>.<option>`）；`params` 段的键原样使用，不加 `params.` 前缀——与 testcase 的声明和报告字段保持一致。

发现顺序（只取第一个存在的文件）：

1. `--params-file <path>`
2. `SIFLI_TEST_PARAMS_FILE`
3. `%USERPROFILE%\.sifli\test-params.yaml`（Windows）/ `~/.sifli/test-params.yaml`（POSIX）

单键覆盖优先级：

1. `--param KEY=VALUE`（调试用；会进 shell 历史，文档中标注）
2. `SIFLI_TEST_PARAM_<KEY>`（大写，非字母数字转 `_`）
3. params.yaml
4. testcase 声明的默认值，或契约里的默认值

`KEY` 是扁平化后的键名，例如 `fixtures.wifi_ap:5g.psk` → `SIFLI_TEST_PARAM_FIXTURES_WIFI_AP_5G_PSK`。扁平化会把 `:` 也变成 `_`，因此 `wifi_ap:5g` 与 `wifi_ap_5g` 会撞名；发生碰撞时直接报错，不做猜测。

逐键覆盖变量（`SIFLI_TEST_PARAM_*`）按**是否存在**判断：显式设为空字符串是"提供空值"，会覆盖文件里的值，不是"未设置"。
`SIFLI_TEST_PARAMS_FILE` 是路径而不是值，空串仍视为未设置并继续按发现顺序查找——两者语义不同，是有意的。

params 文件也使用同一套重复键检查：同一映射内重复的键在加载时报错，不静默保留最后一个。

建议的设置方式（写进用户自己的 shell profile，**不要**写进仓库里的 `export.ps1`）：

```powershell
$env:SIFLI_TEST_PARAMS_FILE = 'E:\code2\lab\params.yaml'
```

### 7.5 解析与下发

```
discovery        → TestInstance.params / fixtures / assets
allocation       → runner 向 agent 请求 testbed；能力匹配保证它提供该 fixture
                   （参数化类型要求 binding spec 与请求 spec 精确相等）
resolve          → 声明的 params：CLI > env > file > default
                   fixture 参数：用请求的 spec 套契约 key 模板，走同一优先级
build            → 启动构建子进程时，从环境副本里剥掉 SIFLI_TEST_PARAM_*
prepare          → 写临时 JSON（见下）：
                     {"params": {...}, "fixtures": {"wifi_ap:5g": {...}}}
run              → 设 SIFLI_TEST_DATA_FILE，启动 pytest
cleanup          → 删除临时文件
```

顺序固定为 **resolve → build**：缺值在构建之前失败，不会先花一轮编译再报配置错。剥离环境变量的落点是 runner 组装 scons 子进程环境的那一层；参数不影响编译（本设计明确不支持"编译期注入凭据"）。

临时文件的保护按平台分别说明：

- POSIX：`mkstemp`（0600），放在 runner 用户的临时目录。
- Windows：`0600` 没有意义，实际保护来自临时目录的用户 ACL；文档不把 0600 当作 Windows 的安全保证。
- 崩溃/强杀会留下残留：runner 启动时扫描并删除超过 24 h 的同类临时文件。
- 不放 `test-out/`（后者会作为 artifact 上传）。
- 缺 `required` 键 → **plan 阶段失败**，信息中列出搜索过的文件/变量；不 skip。
- 解析结果在 plan 阶段做一次 `json.dumps` 试序列化；不可序列化的 YAML 值（如未加引号的日期）直接报 `ParamsError`，不会等到占用 testbed 之后。

### 7.6 pytest 侧接口

```python
def test_connect(dut):
    ap = dut.fixture_options("wifi_ap:5g")
    dut.send_line(f'wifi join "{ap["ssid"]}" "{ap["psk"]}"')
```

- `dut.fixture_options(spec)`：返回**本实例需要的**选项；`required_if` 不成立的可选选项不出现（例如 `security: open` 时没有 `psk`）。
- `params` fixture：返回声明式参数的映射，供不依附 fixture 的值使用。
- `params` 只包含 testcase 声明过的键；`fixture_options` 的键由契约决定。拼错的 spec 报"未知 fixture"；拼错的选项是 `KeyError`，不返回 `None`。

### 7.7 脱敏

- 日志与报告是**实例跑完后一次性写**的（`runner.py:664,725-746`、`report.py:413-465`），所以脱敏做在内存结构上：`result.transcript`、`result.log`、`result.cases`，以及 `summary_to_dict()` 的输出。`device.log`、`runner.log`、`<fixture>.log`、`sifli.json`、`sifli.xml` 都是这些结构的投影，改内存即可全覆盖。
- 失败路径同样覆盖：`dut.expect()` 超时会把 `--- device said ---` 的原始 transcript 拼进异常消息（`pytest_plugin.py:157-164`），再进入 `result.log`；"故意给错 psk 必须失败"正好触发这条路径。
- 值长度 < 6 时仍替换；`params check` 与 `test run` 都会告警（只输出键名，过短会误伤日志）。
- 报告与 `test list --json` 只出现键名与来源，不出现值。
- **已知泄露面（必须如实写，不得宣称"密码不会出现在任何地方"）**：
  1. `wifi join` 由测试经 `POST /testbed/{id}/console/write` 写入（`tools/lab_agent/server.py:400-413`），psk 以明文经过**无认证**的 agent HTTP；
  2. psk 留在 agent 的 4 MB 内存缓冲里，直到被新输出顶掉或 agent 重启（`tools/lab_agent/testbed.py:130-191`）；
  3. DUT 的 wlan 驱动在 debug 级别打印密码（`wlan_mgnt.c:945,1032,1322`），密码因此进入 DUT 侧日志和 agent 转发的 transcript。
- DUT 打印的密码会随 transcript 转发到 host，**转发副本会被脱敏**；host 清不到的是 DUT 自己的其它日志后端（若 ulog 配了非 console 后端）与 agent 的内存缓冲（第 2 条）。消除 1/2 需要 P3（agent 侧组装命令）；消除第 3 条中 DUT 自身的日志需要固件侧关闭该日志，或由 agent 在源头脱敏。**P1 因此只适用于本机或可信实验室网络。**
- **强制边界**：解析到任何 `secret` 值时，若 agent URL 的主机不是回环地址（127.0.0.1 / ::1 / localhost），plan 阶段直接失败；只有显式设置 `SIFLI_TEST_ALLOW_REMOTE_SECRETS=1` 才放行。带秘密的用例默认不可能跨到远端 agent。

### 7.8 失败语义

| 情况 | 结果 |
|---|---|
| params 缺 required 键（含 fixture 参数） | plan 失败，列出键与搜索路径 |
| params 文件不存在/不可读 | plan 失败，指明期望路径 |
| binding 出现契约声明为 `source: params` 的键（如 `ssid`/`psk`） | `binding check` 失败；运行期只让该 testbed 报错，其它 testbed 照常可用 |
| binding 出现契约未提及的 driver 私有键（如仪表的 `console`） | 允许，不算未声明 |
| agent 加载 binding 时发现 params 来源键 | 该 testbed 从可用列表剔除并打印原因，其它 testbed 照常；原因可在 agent 启动日志、`--list`、`--check` 看到（`GET /testbeds` 暂不暴露，未来可加 additive 字段） |
| 被剔除的 testbed 同时接线矛盾 | 不影响 agent 启动：先按契约剔除，再只对保留下来的 testbed 做接线矛盾检查 |
| params 缺所请求 spec 的 `fixtures` 段 | plan 失败，指出 spec 与契约要求的键 |
| 契约的 `key` 模板出现未知占位符 | 契约加载失败（契约自身错误） |
| 用例访问契约未声明的选项 | `KeyError`，测试失败（不放宽为 `None`） |
| AP 参数与实验室实际 AP 不符 | 连接失败；用例先扫描并断言 SSID 可见，诊断指向具体 SSID |
| 参数值超短（< 6） | 告警，不失败 |
| params 文件有无人引用的键 | `params check --unused` 列出，不失败 |

### 7.9 工具

```
sdk.py test params check [--case ID] [--unused] [--json] \
    [--params-file PATH] [--param KEY=VALUE] [--agent-url URL]
sdk.py test binding check [--file PATH] [--json]
sdk.py test plan ... [--agent-url URL] [--params-file PATH] [--param KEY=VALUE]
```

输出"键名 → 来源文件/变量"；不打印值。`--json` 供 CI 预检。

由于键是从 testcase 的 spec 与契约静态推导的，`params check` **不连接 agent**，在编译之前就能判断配置是否齐全。`--agent-url` 只用于回环判定：带 secret 且指向远端 agent 时，pre-flight 与 `test run` 一样被拒绝。`test plan` 使用同一组选项与同一判定，避免"plan 通过、run 被拒"。

fixture 词汇的发现与预检：

```
sdk.py test fixtures types                              # offline catalogue
sdk.py test fixtures list [--agent-url URL | --file BINDING] [--platform P]
sdk.py test plan ... --check-fixtures                   # compares before building
```

- `fixtures types`：离线打印目录（名称、描述、capabilities、选项、是否参数化）。写用例的人从这里知道能写什么。
- `fixtures list`：打印某个来源的实际报价——在线读 `GET /testbeds`，离线读 binding 文件。参数化类型的 spec 必须逐字相等，所以在线列表才是"我该写哪个字符串"的权威。
- `plan --check-fixtures`：用与分配相同的 `FixtureRef.accepts` 语义，对每个实例比对请求与目标平台上的报价；不满足时在**构建之前**失败，并列出两侧。agent 不可达或响应缺 `testbeds` 键都是错误，不会被当成"没有 fixture"。
- `harness_config.fixture` 是列表时表示"全部都要"：必须有一个 testbed **同时**提供全部，预检与分配用同一条规则。两个 testbed 各提供一个不算满足，否则预检会通过而分配失败。
- `skip: true` / `build_only: true` 的实例不参与预检——它们不会申请硬件，与 params 解析对 skip 的豁免理由相同。

## 8. 向量与资产链

### 8.1 三分流

| 形态 | 处置 | 理由 |
|---|---|---|
| 可确定性生成 | 不存，测试期生成；清单记录生成器版本 | 避免数据与生成器漂移 |
| 小、文本、随代码演进（< ~256 KB） | 仓库内 `<example>/testdata/` | 评审时能一起看 |
| 大 / 二进制 / 现场采集 | 外部存储 + 仓库清单 | git 历史只增不减 |

### 8.2 清单

```yaml
# tools/sdk_test/assets.yaml
assets:
  wifi_scan_golden:
    source: store
    sha256: "3f9c..."
    size: 1048576
    version: 3
    class: internal
    generated_by: "tools/gen_wifi_golden.py @ 1.2.0"
    owner: wifi-team
    license: internal

  crc32_cases:
    source: repo
    path: example/test/crc/vectors/crc32_cases.json
    sha256: "..."
    size: 20480
    class: internal
```

规则：

- `source`：`store`（默认）| `repo`。`repo` 条目给出相对仓库根的 `path`，`asset()` 直接返回该路径；两种来源都要求 `sha256` + `size` 并强制校验。
- 名字唯一；内容变更必须 `version + 1` 且新 `sha256`，不许原地替换。
- `generated_by` 记录可复现路径；没有生成器的数据必须写明来源。
- `class`：`internal` | `sensitive`。敏感数据不进 CI 共享缓存。
- 名字与 `assets:` 在 testcase 里的引用一致；`repo` 条目的路径不暴露给用例。

### 8.3 存储与缓存

- 根目录：`SIFLI_TEST_ASSET_STORE`，缺省 Windows `%LOCALAPPDATA%\sifli-test-assets`、POSIX `~/.cache/sifli-test-assets`。CI 用 cache 持久化这个目录。
- 布局：`<store>/sha256/<digest>`，内容寻址、不可覆盖、天然去重。
- 来源列表：`file://` / `https://`（如 `jfrog.sifli.com/artifactory/...`），按顺序尝试；访问 token 走 CI secret，**token 是秘密，向量不是**。
- **写入原子性**：先写 `<store>/tmp/<digest>.<pid>`，校验 size + sha256 通过后再 `os.replace` 到最终路径。两个 runner 并发取同一 digest 也只会有一次有效发布；半截文件永远不落在 `sha256/` 下。
- 敏感存储：`SIFLI_TEST_SENSITIVE_STORE`（根目录）+ `SIFLI_TEST_SENSITIVE_TOKEN`（访问凭据）。`class: sensitive` 的条目只从该存储取，不写默认 store。
- 没有独立的 cache 层：本地 store 就是缓存，删了可重建。

### 8.4 解析与校验

- `test run` 在 plan 阶段检查用例声明的 asset；缺失则按来源取回；`--offline` 或取不到 → 失败并列出缺失名单，不 skip。
- `source: repo` 的条目直接按清单里的 `path` 解析，不复制、不缓存；仍校验 size + sha256。
- `source: store` 的条目在 `<store>/sha256/<digest>` 命中即用，未命中按来源列表取回并按 §8.3 原子写入。
- pytest 侧 `asset("name") -> Path`：进程内首次使用时校验 size + sha256，失败即报错，不使用可疑文件。
- 报告记录本次用到的 `name@sha256`，让一次运行可复现。

### 8.5 工具

```
sdk.py test assets list
sdk.py test assets fetch [NAME...] [--offline]
sdk.py test assets verify [NAME...]
sdk.py test assets add <path> --name NAME [--owner ...] [--class ...] [--generated-by ...]
```

`add` 只计算哈希并打印清单条目，上传仍走制品库的常规流程。

### 8.6 与 fixture 的关系

资产**不**声明为 fixture：

- fixture 由 agent 在 binding 上匹配，属于实验室接线；资产属于测试进程与内容存储。
- 若把资产塞进 fixture，agent 需要知道存储地址与外网，远端 agent 还要传输大文件。
- 消费者在实验室时（DUT 或仪表需要这份字节），由 runner 解析后**上传**，agent 可按 digest 缓存（复用 `tools/lab_agent/firmware.py:75-88` 的暂存思路）。这是传输，不是归属。

### 8.7 敏感向量

现场采集可能含用户数据：

- `class: sensitive` 的资产从 `SIFLI_TEST_SENSITIVE_STORE` 取，凭据用 `SIFLI_TEST_SENSITIVE_TOKEN`，不写入默认 store；
- 不入 CI 共享缓存，不作为 artifact 上传；
- 清单照常记录哈希，便于校验但不暴露内容。

## 9. 接口原型

### 9.1 `testcase.yaml`

```yaml
common:
  harness: pytest
  harness_config:
    pytest_root: pytest

tests:
  wifi_sta_connect:
    description: Associate with the lab AP and obtain an address.
    harness_config:
      fixture: wifi_ap:5g
    timeout: 120

  cloud_upload:
    description: Upload one report and verify the returned id.
    params:
      cloud.token: {required: true}
      upload.timeout_s: 30
    assets:
      - upload_payload_1k
```

schema 新增（`tools/sdk_test/schema/testcase.schema.json`，`tests` 条目）：

- `params`: object，值为标量（默认值）或 `{required: true, secret: true}`（至少一个为真；`{}` 与全为 false 都被拒绝）—— P1 加入；
- `assets`: string 列表 —— P2 加入（与资产解析一起落地，避免出现能解析但不生效的字段）。

### 9.2 `fixture_types.yaml`

见 §7.2；加载器同时被 `tools/sdk_test` 与 `tools/lab_agent` 使用。

### 9.3 binding

见 §7.3。

### 9.4 `params.yaml`

见 §7.4。

### 9.5 pytest

```python
# example/rt_device/wifi/sta/pytest/test_sta.py
def test_connect(dut):
    ap = dut.fixture_options("wifi_ap:5g")
    if ap["security"] == "open":
        dut.send_line(f'wifi join "{ap["ssid"]}"')          # argc==3: no key
    else:
        dut.send_line(f'wifi join "{ap["ssid"]}" "{ap["psk"]}"')
    # Anchor on the example's full success line: "connect" alone also matches
    # "disconnect". Confirm the exact text on a real board first.
    dut.expect("WiFi connected", timeout=40)
    dut.send_line("ifconfig")
    dut.expect(r"inet addr:\s*\d+\.\d+\.\d+\.\d+", timeout=10)
```

```python
import hashlib

def test_compare(asset):
    golden = asset("wifi_scan_golden")
    assert hashlib.sha256(golden.read_bytes()).hexdigest() == "3f9c..."
```

### 9.6 CLI

```
sdk.py test params check [--case ID] [--unused] [--json]
sdk.py test assets list|fetch|verify|add
sdk.py test fixtures types
sdk.py test fixtures list [--agent-url URL | --file BINDING] [--platform P]
sdk.py test plan ... --check-fixtures
sdk.py test list --json          # 每个实例含 "params": [键名...]（不读值）；assets 在 P2 加入
```

### 9.7 报告字段

```json
{
  "params": [
    {"name": "cloud.token", "source": "E:/code2/lab/params.yaml"},
    {"name": "fixtures.wifi_ap:5g.psk", "source": "E:/code2/lab/params.yaml"}
  ],
  "assets": [
    {"name": "wifi_scan_golden", "sha256": "3f9c..."}
  ]
}
```

只记录名字与来源，不记录值。

## 10. wifi sta 端到端实例

### 10.1 用例声明

```yaml
tests:
  sta_connect:
    harness: pytest
    harness_config:
      fixture: wifi_ap:5g
      pytest_root: pytest
    timeout: 120
```

### 10.2 实验室侧

binding 只有 `fixture: wifi_ap:5g`；`params.yaml` 的 `fixtures["wifi_ap:5g"]` 段给出 `ssid`/`security`/`psk`。

### 10.3 自动连接的容忍

例程在复位后约 6 s 主动连接 `"wifi_ssid"`（`main.c:187,199,203`），且该线程会做一次同步扫描。不改例程时，用例时序固定为：

1. 等自动连接那一轮结束：5 s 延时 + 最长 10 s 同步扫描 + 1 s + 最长 10 s 连接等待，最坏约 26 s，窗口放到 **30 s 以上**；只等连接结果标记，不要用"扫描完成"代替——两者语义不同；
2. 再 `wifi join`；
3. 断言"最终关联并拿到地址"，不能断言"发完命令立刻连上"——扫描可能造成瞬时扰动。

这些日志标记的稳定文本**尚未在真板上确认**，是第一件要验证的事。负向验证不受影响：故意给错 psk 必须失败。

### 10.4 AP 不匹配时

按 §2 的决策，框架不比对"实验室实际 AP"与 params 里的 SSID。用例应当把扫描结果纳入断言：

```python
import re

def test_connect(dut):
    ap = dut.fixture_options("wifi_ap:5g")
    dut.send_line("wifi scan")
    # `expect()` returns only the matched text, not the text before it, and
    # the scan has no "done" marker. Match the SSID's own line instead.
    # Confirm the scan table's exact layout on a real board.
    dut.expect(rf"(?m)^.*{re.escape(ap['ssid'])}\b", timeout=20)
    dut.send_line(f'wifi join "{ap["ssid"]}" "{ap["psk"]}"')
```

如果 SSID 不在扫描结果里，`expect()` 超时的异常会带出这段 transcript（`pytest_plugin.py:157-164`），诊断直接指向"目标 SSID 不可见"，而不是一串后续超时。

### 10.5 前置条件

- binding 里需要一块 wlan-core 板子的 testbed（当前仓库配置里没有）；
- 真实板没有 `features.yaml`（只有 QEMU 板有），所以暂时无法用 `depends_on: [wifi]` 声明能力；
- `sf32lb52-wlan-core_*` 的可用性未构建验证。

## 11. CI

### 11.1 runner 就在实验室机器上（推荐）

文件本来就在那台机器上，CI 不持有秘密：

```toml
# GitLab Runner config.toml
[[runners]]
  environment = ["SIFLI_TEST_PARAMS_FILE=E:\\code2\\lab\\params.yaml",
                 "SIFLI_TEST_ASSET_STORE=E:\\code2\\lab\\assets"]
```

注意 GitLab Runner 以服务运行时使用服务账号，需保证该账号能读 params 文件与 store 目录。

### 11.2 runner 在别处，远程驱动 agent

秘密必须来自 CI secret 存储：

- 逐键环境变量（推荐，不落文件）：`SIFLI_TEST_PARAM_CLOUD_TOKEN` 等，勾选 Masked + Protected；
- 或运行时物化整份文件：多行 YAML 用 base64 存在一个 masked 变量里，`before_script` 解码写临时文件，`after_script` 删除；`.ci/` 不进 artifacts。

向量则从制品库取，用 CI cache 持久化本地 store：

```yaml
variables:
  SIFLI_TEST_ASSET_STORE: "$CI_PROJECT_DIR/.cache/sifli-assets"
cache:
  key:
    files: [tools/sdk_test/assets.yaml]
  paths: [.cache/sifli-assets/]
script:
  - sdk.py test assets fetch
  - sdk.py test assets verify
  - sdk.py test run -T example -p $PLATFORM
```

`SIFLI_TEST_ASSET_STORE` 必须指向被缓存的那个目录，否则缓存不会命中真实的 store。

### 11.3 两项固定检查

1. **提前失败**：job 开头用 `test list --json` 核对所需键名/资产名单，并用 `test plan --check-fixtures --agent-url ...` 确认该 lab 真的提供这些 fixture；缺什么在构建前报。
2. **证明没泄露**：跑完检查 `test-out/` 中不含真实秘密值（可由 CI 脚本 grep 一次）。

## 12. 实施阶段

| 阶段 | 内容 | 主要落点 |
|---|---|---|
| P1 | schema、fixture 目录与契约、params 解析、下发、脱敏、`fixture_options`/`params` 接口、`params check`、`binding check`、报告 provenance、agent 侧 binding 校验、`fixtures types/list`、`plan --check-fixtures` | `schema/testcase.schema.json`、`fixture_types.yaml`（新）、`params.py`（新）、`discovery.py`、`model.py`、`runner.py`、`report.py`、`binding.py`（校验）、`lab_agent/config.py`、`cli.py`、`harness/pytest.py`、`pytest_plugin.py`、`test_ext.py`、`version.py`（PROTOCOL，若 bump） |
| P2 | 资产清单、store、取用/校验、`asset` fixture、`assets` CLI、报告字段 | `assets_store.py`（新）、`assets.yaml`（新）、`pytest_plugin.py`、`runner.py`、`report.py` |
| P3（按需） | agent 侧执行模板命令（秘密不出实验室）、agent 认证 | `fixture_drivers/`、`testbed.py`、`server.py` |

P1 完成后框架路径可用；真正跑 wifi 还需要 §10.5 的前置条件（wlan-core testbed 与日志标记确认）。P2 与 wifi 解耦，可在有真实向量需求时再做。

## 13. 负向验证

| 场景 | 期望 |
|---|---|
| 删除 `fixtures["wifi_ap:5g"].psk` | plan 失败，报键名与搜索路径，**不 skip** |
| 故意给错 psk | 用例红 |
| 跑完 grep `test-out/` 中的真实密码 | 零命中 |
| 把 psk/ssid 写进 binding | 加载失败，提示这些选项来自 params |
| binding 拼错选项名（`password`） | `binding check` 告警未知键并列出契约允许的键（不阻断） |
| 参数化 fixture 的 binding spec 是请求 spec 的 superset | 不匹配（精确相等要求），不会按错误键查参数 |
| params 缺 `fixtures["wifi_ap:5g"]` 段 | plan 失败，列出 spec 与缺失键 |
| 用例访问 `ap["pks"]` | `KeyError`（契约没有该选项） |
| SSID 指向不存在的 AP | 用例红；扫描断言指出该 SSID 不可见 |
| params 值长度 < 6 | 告警，但脱敏仍生效 |
| 资产 sha256 改一位 | 取用失败，不使用旧文件 |
| `--offline` 且本地缺失 | 失败并列出缺失名单 |
| 缓存文件被手工改坏 | `verify` 发现并重新获取 |
| 并发取同一资产 | `sha256/` 下只有一个完好文件，校验通过 |
| 强杀 runner 后重启 | 启动时清理残留临时文件，不残留明文 |
| `security: open` 且无 psk | 通过（条件必填不触发） |
| 带 secret 且 agent 非回环、未放行 | plan 失败，提示设置 `SIFLI_TEST_ALLOW_REMOTE_SECRETS=1` |
| 带 secret 且 agent 非回环、已放行 | 运行，风险由使用方承担 |
| 无契约条目的 fixture 类型 | options 仍可自由使用（不回归） |
| testcase 请求未登记的类型 | 告警，不阻断（实验室自定义类型允许） |
| 已知类型的未知 capability（`wifi_ap:6e`） | 告警，不阻断 |
| `bt_peer` 连接的 `console` 键 | 不告警（非参数化类型不校验 options） |
| `plan --check-fixtures` 缺 fixture | 构建前失败，列出请求与各 testbed 报价 |
| `wifi_ap:5g` vs binding `wifi_ap:5g,lab_a` | 预检失败（参数化类型精确相等） |
| `plan --check-fixtures` 时 agent 不可达 | 明确报错，不当作"没有 fixture" |
| `GET /testbeds` 响应缺 `testbeds` 键 | 明确报错（响应不完整），不当作空实验室 |
| 实例请求多个 fixture，两个 testbed 各提供一个 | 预检失败：必须单个 testbed 同时满足全部 |
| `skip` / `build_only` 实例缺 fixture | 不影响预检（不会申请硬件） |
| `required_if` 的依赖未解析 | 报依赖缺失，不把依赖它的选项误报为必填 |
| 契约的 `required_if` 依赖可选且无 default | 契约加载失败（条件无法求值） |
| key 模板含多余大括号 | 契约加载失败 |
| enum 含非字符串或重复项 | 契约加载失败 |
| 契约选项名/类型名不是字符串 | 契约加载失败 |
| YAML 中重复的类型名或选项名 | 契约加载失败（不静默取最后一个） |
| 环境变量显式设为空串 | 逐键覆盖按"已提供"处理，覆盖文件值，来源为 environment |
| `SIFLI_TEST_PARAMS_FILE` 设为空串 | 视为未设置，按发现顺序继续查找 |
| `params: {x: {}}` 或全为 false | schema/发现阶段拒绝（至少一个为 true） |
| 同一类型内两个选项用同一 `key` 模板 | 契约加载失败 |
| params 文件同一映射内重复键 | 加载失败并给出行号 |
| 契约使用 YAML 合并键 `<<` | 正常加载（不误报重复） |
| enum 存在时运行值是数字 | 判为不在枚举，不做字符串归一化 |

## 14. 风险与未决

1. **psk 经无认证的 agent HTTP 明文传输**（严重）：测试用 `dut.send_line()` 发 `wifi join`，走 `POST /testbed/{id}/console/write`。同时 psk 留在 agent 的内存缓冲直到被 4 MB 新输出顶掉或重启，且 DUT 的 wlan 驱动在 debug 级别打印密码。P1 用回环地址强制（§7.7）把默认暴露面收在本机；跨远端 agent 需显式放行。要彻底移除传输与缓冲路径需要 P3。
2. **流式脱敏**：若未来在 agent 侧做脱敏，秘密可能跨读取块边界；成行替换 + 最大行长 holdback 是备选。本期不做。
3. **两个被跟踪的示例配置都含真实端口**：`tools/lab_agent/examples/sifli-lab.yaml`（COM8/COM6）与 `tools/sdk_test/examples/binding.local.yaml`（COM13/COM30），与"请勿提交真实端口"的约定矛盾。在引入 params 之前必须一起修，否则新机制第一天就被绕过。
4. **`example/rt_device/wifi/sta/src/wheather.c:17` 含有真实 API key**，属既有问题，建议单独处理。
5. **真实板没有 `features.yaml`**：能力声明与矛盾检查目前只对 QEMU 板生效；wifi 能力无法用 `depends_on` 表达。
6. **Windows 上的临时文件保护**：`0600` 在 Windows 无效，实际保护来自临时目录的用户 ACL；强杀会留下残留，需要启动清理（§7.5）。服务方式运行的 CI runner 也不一定能读用户 ACL 收紧后的文件。
7. **CI cache 体积**：大向量走 GitLab cache 需打包，建议自托管 runner 用持久目录。
8. **能力声明与实际 AP 不符**：按决策，框架不做交叉校验，表现为连接失败。用例应在扫描阶段断言目标 SSID 可见（§10.4），否则诊断成本会落在每次排障上。

## 15. 对现有文档的修改

- `docs/source/zh_CN/app_note/autotest_guide.md` §10 与英文版：删除"向量就是一个 fixture"的表述，改为本文 §8 的清单 + 存储模型，并说明 fixture 只表达能力。
- `tools/sdk_test/DESIGN.md`：在测试数据相关段落链接本文。
- `tools/lab_agent/examples/sifli-lab.yaml` 与 `tools/sdk_test/examples/binding.local.yaml`：去除真实端口，改成模板或移出仓库（§14.3）。
- `example/rt_device/wifi/sta/README.md` / `README_EN.md`：**不改**。示例文档面向使用者，测试用法写在指南 §10。
