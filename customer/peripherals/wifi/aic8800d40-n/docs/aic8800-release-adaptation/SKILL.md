---
name: "aic8800-release-adaptation"
description: "AIC8800 release 发布包（diff 包）平台适配工作流。当用户拿到 aic8800 release 发布包（无 host/ 源码、预编译库交付、AIC_BUILD_HOST=0 模式）需要在新 RTOS/SDK 平台完成 platform 层适配时调用。覆盖六大场景：全新适配、默认 platform 直接编译、SDK 差异适配、平台 WiFi 框架 Wrap（含双 VIF 并发 APSTA/APP2P）、CLI 命令集成（含 aicwf_ATRESP 响应通道）、BT/BLE（NimBLE）通道集成。包含 platform_config.mk 配置指南、适配检查清单、常见编译错误排错表。不涉及 host 驱动编译或源码修改。"
---

# AIC8800 Release 发布包平台适配框架

面向 **release 发布包**（`dist/<driver>_release/` 或 diff 包）的平台适配指南。发布包以**预编译库**交付驱动核心，适配工作仅限 **platform 层**。

```
适用前提:
  ✓ 拿到 release 包 (lib/*.a 预编译库 + platform/ 源码 + port/inc 接口头)
  ✓ 包内无 host/ 目录 → 构建系统自动进入 AIC_BUILD_HOST=0 发布态
  ✓ 目标平台已有 RTOS + 网络协议栈 (lwIP 或等效) + 总线驱动 (SDIO/USB)

本 Skill 负责:
  platform 层适配 (实现/修改 platform/*.c 对接平台 SDK)
  platform_config.mk 配置 (工具链/头文件路径/宏/源文件列表)
  WiFi 框架 Wrap 与 CLI 命令集成

本 Skill 不负责:
  ✗ host 驱动核心编译 (无源码, 由预编译库交付)
  ✗ 驱动功能修改/裁剪重编 (需联系驱动提供方重新出包)
  ✗ 芯片型号/总线切换超出包内已有库范围 (见 1.3 铁律)
```

### 执行协议（三阶段）

所有代码级任务遵循**先计划、后执行、再汇总**的三阶段协议：

```
阶段 1: 输出 SPEC / Plan
  → 确认场景归属，产出结构化适配计划（修改文件列表、配置项、风险）
  → 等待用户确认

阶段 2: 确认后执行
  → 用户回复"确认"/"按建议执行"后，逐一落地适配变更
  → 每处变更需 make 编译验证

阶段 3: 执行后输出清单
  → 汇总所有修改点（文件 + 变更说明）
  → 输出 TODO 列表（未对接项、运行时待验证项）
```

---

## 第一步: 认识 release 包

### 1.1 包结构

```
dist/<driver>_release/                   ← 自包含发布包 (与 diff 包同级, 不再占驱动源码同级)
├── lib/
│   ├── libaic8800_driver.a              ← 驱动核心预编译库 (单库统一, 无型号后缀,
│   │                                      全芯片型号共用, 运行时按硬件自动分发)
│   ├── libaic_mbedtls.a                 ← mbedTLS 加密原语库 (USE_BUILTIN_MBEDTLS=1 时链接)
│   └── include/                         ← 驱动 API 公开头 (B 类, 镜像)
│       ├── drv_api/inc/                 ←   wifi_api.h / netif_api.h / sdio_api.h /
│       │                                    cli_api.h / sys_api.h / wifi_driver_event.h ...
│       ├── drv_wifi/wifi_if.h           ←   WiFi 接口
│       └── utils/inc/co_*.h             ←   通用工具 (list/ring/pool/math)
├── include/                             ← 与 lib/include/ 内容一致 (兼容路径)
├── port/inc/                            ← 移植接口层头文件 (A 类) — 适配契约
│   ├── rtos_port.h                      ←   RTOS 原语声明 (~87 函数)
│   ├── sdio_port.h / usb_port.h         ←   总线声明 (~23/~15 函数)
│   ├── netif_port.h                     ←   网络接口声明 (~42 函数)
│   ├── plat_port.h                      ←   HAL 声明 (~16 函数) + 任务优先级/栈宏
│   ├── aic_fw_port.h / fmac_port.h      ←   固件加载接口
│   ├── aic_log.h / aic_porting_types.h  ←   日志宏 / 基础类型
│   └── ...
├── platform/                            ← 平台适配层源码 (开源, 适配工作主体)
│   ├── hal/      sdio_port.c usb_port.c plat_port.c
│   ├── rtos/     rtos_freertos.c rtos_rtt.c rtos_errno.h
│   ├── netif/    netif_port.c
│   ├── fmac/     aic_fw_port.c userconfig.c aic_chip_desc_port.c fw/
│   ├── wifi/     wifi_port.c wifi_port_{sta,ap,wps,p2p,cmd,plt}.c
│   ├── bt/ iperf/ uapapi/
│   └── platform_init.c
├── Makefile                             ← 顶层构建 (AIC_BUILD_HOST=0 发布态)
├── CMakeLists.txt                       ← CMake 辅助构建 (可选)
├── aic_cfg.mk                           ← 功能开关 (型号/总线/P2P/BT/PLT)
├── platform_config.mk                   ← ★ 平台配置入口 (适配时编辑此文件)
├── BUILD.md                             ← 构建说明文档
├── docs/                                ← 适配/AT指令/定频测试/常见问题说明
└── STACK/nimble/                        ← 【条件】仅 AIC_FEATURE_NIMBLE=1 出包时携带
                                            (含独立 Makefile/CMakeLists, 可编出 libnimble.a;
                                             需要 BLE 上层栈时才存在, 见第八步)
```

### 1.2 AIC_BUILD_HOST=0 编译机制

顶层 Makefile 检测 `host/` 目录存在性自动选择构建模式；release 包无 `host/`，**自动进入发布态**：

```
make                          # 编译 platform 层 → build/libaic_platform.a
                              # 链接引用: 包内 lib/*.a + lib/include/ + port/inc/
make info                     # 查看当前生效配置 (CC/AR/型号/总线/SRC_PLATFORM)
make clean                    # 清理 build/
```

发布态数据流：

```
platform/*.c  ──CC/CFLAGS_ARCH──►  build/*.o  ──AR──►  build/libaic_platform.a
                                                              │
SDK 工程最终链接:  libaic_platform.a
                 + libaic8800_driver.a              ◄───────────┘
                 + libaic_mbedtls.a (USE_BUILTIN_MBEDTLS=1 时)
                 + 平台 SDK 库 (RTOS/lwIP/SDIO 驱动)
```

> **两种集成方式**（择一）：
> A. **Makefile 独立编译**：包内 `make` 产出 `libaic_platform.a`，SDK 工程链接三个 `.a`；
> B. **源码入工程**：将 `platform/*.c` 直接加入 SDK 工程（IDE/构建系统）编译，与 `libaic8800_driver.a` 一起链接。`platform_config.mk` 中的 `SRC_PLATFORM`/`EXT_INC_DIRS`/`PLATFORM_DFLAGS` 信息需同步映射到 IDE 工程配置。

### 1.3 release 适配三铁律

> **⚠️ 铁律 1 — ABI/架构必须与预编译库匹配**:
> 驱动核心的架构、浮点 ABI、`-mfloat-abi` 已由预编译库**固定**。平台工程工具链必须与库一致
> （如库为 RISC-V E906 `ilp32d` double-float，平台工程必须同架构同 ABI），
> 否则链接报错或运行崩溃。**目标平台架构不符时，必须向驱动提供方申请对应工具链的 release 包，
> 无法通过配置绕过**。确认方法：
> ```bash
> riscv64-unknown-elf-readelf -h lib/libaic8800_driver.a     # 查看 Machine/Flags
> # 或: file 解包后的 .o 目标文件
> ```

> **⚠️ 铁律 2 — port/inc 接口契约稳定，不可改签名**:
> `port/inc/*.h` 声明的 `aicwf_*` 函数签名与预编译库内的调用点绑定。
> 适配只能**实现**这些函数，不能修改声明（改签名 = 链接期/运行期错误）。

> **⚠️ 铁律 3 — 总线必须与包内库一致；型号由运行时硬件决定**:
> 库在出包时已按 `AIC_BUS_TYPE`（SDIO/USB）编译固定，`aic_cfg.mk`/`platform_config.mk` 中的
> `AIC_BUS_TYPE` **必须与包内库一致**，改配置不会改变库内容，只会造成 platform 层与库错配
> （如 platform 编 USB 版而库是 SDIO 版 → `undefined reference to aicwf_sdio_send` 或符号错链）。
> 芯片型号**不再**由库绑定：单库服务全部型号，驱动运行时读硬件 VID/PID 做 chipmatch，
> 按 chipid 查 `aic_chip_desc_port.c` 数据表分发差异逻辑。`AIC_CHIP_MODEL` 仅须与
> **目标硬件**一致（决定 platform 侧固件数据选择），与库无关。

---

## 第二步: 场景路由

### 2.1 环境侦察: 定位驱动包 → 识别 release 印记 → 评估适配状态

路由前先在用户工作区做三步侦察，弄清“驱动包在哪、是什么形态、适配到什么程度了”：

**Step 1 — 寻找 AIC8800 相关目录**

```bash
# 在 SDK/工作区内查找驱动目录 (常见名: aic8800_rtos_release / aic8800_rtos / aic8800_driver_*)
find <SDK_ROOT> -maxdepth 4 -type d -iname "*aic8800*"
```

**Step 2 — 识别 release 印记**（确认是否属本 Skill 适用范围）

| 印记 | 判定 |
|------|------|
| 无 `host/` 目录 | ✓ release 形态（自动进入 `AIC_BUILD_HOST=0`） |
| `lib/libaic8800_driver.a` 存在 | ✓ 预编译库交付（单库统一，无型号后缀，全型号共用） |
| `lib/libaic_mbedtls.a` 存在 | ✓ 加密库随包（`USE_BUILTIN_MBEDTLS=1` 时链接） |
| `platform_config.mk` + `aic_cfg.mk` 存在 | ✓ 构建配置入口 |
| 已存在 `build/` 或编译错误记录 | 适配已启动 → 直接进入 Step 3 状态评估 |

> 若发现 `host/` 目录 → 完整源码树，不属本 Skill 适用范围。

**Step 3 — 评估当前适配状态**

```bash
cd <驱动目录>
grep -E "^CC|^EXT_INC_DIRS|^SRC_PLATFORM" platform_config.mk  # 配置项是否已填写?
grep -rn "TODO: \[AIC8800-PORT\]" platform/ | wc -l           # 剩余多少未实现存根?
ls build/ 2>/dev/null                                          # 是否有编译产物?
make info                                                      # 当前生效配置一览
```

**部分适配判定（关键新增规则）**：platform 层与平台强相关的接口（RTOS 原语
`aicwf_rtos_*`、SDIO/USB 总线封装 `aic_sdio_*`/`aicwf_usb_*`、日志 HAL
`aicwf_log_printf`、电源 GPIO `aicwf_gpio_*`、netif 桥接 `aic_netif_*`）
如果在平台 SDK 中**已经有对应实现**（头文件可 include、函数可调用），即判定为
**已有部分适配**，不是从零开始：

```
# 快速检查部分适配 (任选其一即判定为部分适配):
# 1. platform_config.mk 已填 CC/EXT_INC_DIRS/SRC_PLATFORM
# 2. platform/ 下 rtos_xxx.c / plat_port.c / sdio_port.c 中 [PLATFORM] 标注点已替换,
#    且 grep -E "aicwf_rtos_task_create|aic_sdio_io_rw" platform/ 里能搜到真实 SDK 调用
# 3. build/ 下有历史 .o 或 .a 产物, 或 SDK 构建脚本已注册 platform 目录
```

> 部分适配 → 直接走 **场景三（SDK 差异适配）**，按 §5 流程：编译定位差异 →
> 归类 → 逐点更新，不重复从头实现。
>
> 状态评估完成后，若已存在 SDK 集成痕迹（SDK 构建脚本中有驱动注册、
> 或有历史编译日志），直接进入 **5.6 SDK 级集成编译与错误反查**，
> 从 SDK 级编译错误反推未完成/失效的适配点。

### 2.2 场景总览

根据侦察结果确定场景归属，进入对应章节：

| # | 场景 | 典型特征 | 工作量 | 章节 |
|---|------|---------|--------|------|
| 一 | 全新适配 | 新平台/新 RTOS，platform 层从零实现 | 高（数天~数周） | 第三步 |
| 二 | 默认 platform 直接适配 | 平台与默认参考实现接近（同为 FreeRTOS+lwIP+相近 SDIO 栈） | 低（半天~1 天） | 第四步 |
| 三 | SDK 差异适配 | 已有旧版适配，当前 SDK 版本 API 变化 | 中（定位差异点逐个更新） | 第五步 |
| 四 | WiFi 框架 Wrap | 平台已有自研/第三方 WiFi 管理框架，需映射到驱动接口 | 中（应用层 wrap） | 第六步 |
| 五 | CLI 命令集成 | 将驱动 AT/WiFi 命令挂到平台 Shell | 低 | 第七步 |
| 六 | BT/BLE 通道 | 需要蓝牙/BLE：HCI 传输通道 + 可选 NimBLE host 协议栈 | 中（UART HCI 需对接 UART HAL） | 第八步 |

> 场景非互斥：常见组合 = （一/二/三）+ 四 + 五 +（按需）六。先完成 platform 层编译链接（一~三），
> 再做应用层集成（四/五/六）。
>
> 双 VIF 并发（APSTA/APP2P）不是独立场景，而是应用层调用方式的约束，附在第六步 6.4 节。

### 2.3 单/多场景判定依据

```
Step 3 侦察到的状态是什么?
├─ platform/ 未改动、配置空白 (全新) → 基础场景, 按平台接近度二选一:
│    平台 RTOS 是 FreeRTOS/RT-Thread 且 lwIP/总线栈接近参考实现?
│    ├─ 是 → 场景二 (改 platform_config.mk 即可)
│    └─ 否 → 场景一 (新 RTOS 新建 rtos_<name>.c + 逐模块实现)
├─ 已有适配代码, 但换 SDK 后编译报错 → 场景三 (定位差异逐个更新)
└─ platform 层已可编译链接 → 追加应用层场景:
     平台有自研 WiFi 管理框架?  → 追加场景四 (Wrap 层)
     需要 Shell/AT 命令入口?      → 追加场景五 (注册 aic_wifi_cmd)
     需要蓝牙/BLE 能力?           → 追加场景六 (HCI 通道 + 可选 NimBLE, 见第八步)
```

---

## 第三步: 场景一 — 全新适配

platform 层为空白或需全新实现时，按本节流程从零适配。核心工作 = **实现 `port/inc` 中声明的全部 `aicwf_*` 接口**。

### 3.1 前置检查（动手前必做）

```
☐ 1. ABI 匹配确认 (铁律 1): readelf -h 检查库架构, 与平台工具链比对
☐ 2. 总线确认 (铁律 3): nm 检查库总线符号
        nm libaic8800_driver.a | grep aicwf_\(sdio\|usb\)_send
        → T/U 出现 sdio 版 = SDIO 库; usb 版 = USB 库
☐ 3. 型号确认: aic_cfg.mk 的 AIC_CHIP_MODEL 与目标硬件芯片一致
        (单库支持全型号, 运行时按硬件 VID/PID 自动分发, 此项仅决定固件选择)
☐ 4. 平台 SDK 依赖调研:
        RTOS API / SDIO 或 USB Host 栈 / lwIP / 内存堆 / 调试串口 / GPIO
☐ 5. 工具链写入 platform_config.mk (CC/AR/CFLAGS_ARCH), 后续 make 免传参
```

### 3.2 五大核心模块与接口清单

extern 直接链接模式：host 库内部直接调用 `aicwf_*` 函数，platform 层实现后由链接器绑定。
**所有声明的函数必须实现**，否则最终链接报 `undefined reference`。

| # | 模块 | 契约头文件 | 参考实现 | 函数量 | 适配要点 |
|---|------|-----------|---------|--------|----------|
| 1 | RTOS 原语 | `rtos_port.h` | `platform/rtos/rtos_freertos.c`（FreeRTOS）、`rtos_rtt.c`（RT-Thread） | ~87 | 任务/信号量/互斥/定时器/事件组/队列/内存；新 RTOS 新建 `rtos_<name>.c` |
| 2a | 总线（SDIO） | `sdio_port.h` | `platform/hal/sdio_port.c` | ~23 | CMD52/CMD53 读写、驱动注册、中断、块大小；`[PLATFORM]` 标注点替换 |
| 2b | 总线（USB） | `usb_port.h` | `platform/hal/usb_port.c` | ~15 | 需平台 USB Host 栈；`AIC_BUS_TYPE=USB` 时启用 |
| 2c | 平台 HAL | `plat_port.h` | `platform/hal/plat_port.c` | ~16 | 电源/GPIO/时钟/调试触发/日志输出（`aicwf_log_printf`）；`aicwf_dbg_snprintf` 被 **host 库直接引用**（格式化拼装统一入口），必须提供 |
| 3 | 网络接口 | `netif_port.h` | `platform/netif/netif_port.c` | ~42 | lwIP netif 注册、DHCP/DNS、TX/RX buffer 桥接 |
| 4 | FMAC 固件 | `aic_fw_port.h` / `fmac_port.h` | `platform/fmac/aic_fw_port.c`、`userconfig.c` | — | 默认 innermode 实现（固件内嵌数组）**通常无需适配**；仅 Flash/文件系统加载模式需改 |
| 5 | WiFi 应用层 | `wifi_port_priv.h`（内部） | `platform/wifi/wifi_port*.c` | — | sta/ap/wps/cmd 已完整实现**无需适配**；仅 `wifi_port_plt.c`（启动入口）需 porting |

> 函数数量为头文件顶层声明的大致统计，**以包内 `port/inc/*.h` 实际声明为准**。

**编译归属速查**（Makefile 自动 vs 手动）：

| 类别 | 文件 | 启用方式 |
|------|------|----------|
| 自动编译 | `platform_init.c`、`sdio_port.c`/`usb_port.c`（按 `AIC_BUS_TYPE`）、`netif_port.c`、`aic_fw_port.c`、`wifi_port{,_sta,_ap,_wps,_cmd}.c`、RTOS 实现（按 `AIC_RTOS`，默认 `rtos_freertos.c`） | Makefile 自动加入，无需配置 |
| 宏控制 | `wifi_port_p2p.c`（`AIC_FEATURE_P2P`）、`bt/aic_bt_uart.c`（`AIC_FEATURE_BT`）、`wifi_port_plt.c`（`AIC_FEATURE_PLT`） | `aic_cfg.mk` / `platform_config.mk` 设置 |
| 手动追加 | `plat_port.c`、`userconfig.c`、`iperf/iperf.c` | `SRC_PLATFORM += $(PLATFORM_PATH)/...` |

### 3.3 platform_config.mk 配置

适配时**只需编辑此文件**（完整注释模板见包内 `platform_config.mk` 本身），四组核心配置：

```makefile
# ── ① 工具链 (必须与预编译库 ABI 匹配, 见铁律 1) ──────────────────
CC  = <your-cross-gcc>
AR  = <your-cross-ar>
CFLAGS_ARCH = <与库一致的架构标志>     # 例: -march=... -mabi=ilp32d -mcmodel=medlow -O2
                                        # ⚠️ 浮点 ABI 必须与库一致, 否则 mbedtls 链接 ABI 错误

# ── ② 外部 SDK 头文件路径 (空格分隔) ────────────────────────────
EXT_INC_DIRS = /path/to/rtos/include /path/to/lwip/include /path/to/sdk/sdio

# ── ③ 平台特定宏 ────────────────────────────────────────────────
PLATFORM_DFLAGS = -DPLATFORM_MY_BOARD
# 类型冲突时: PLATFORM_DFLAGS += -DWPA_TYPES_DEFINED  (平台已定义 u8/u16 等短名)

# ── ④ 平台源文件列表 (仅手动追加项) ─────────────────────────────
SRC_PLATFORM += $(PLATFORM_PATH)/hal/plat_port.c        # 平台 HAL (必加, 理由见下注)
# SRC_PLATFORM += $(PLATFORM_PATH)/fmac/userconfig.c    # 按需
# SRC_PLATFORM += $(PLATFORM_PATH)/iperf/iperf.c        # 需同时 AIC_FEATURE_IPERF=1
```

> **⚠️ `plat_port.c` 为什么"必加"（两条依据）**：
> ① 平台 AT 回包通道 `aicwf_ATRESP`；
> ② **host 预编译库已直接引用 `aicwf_dbg_snprintf`**（`port/inc/plat_port.h` 声明、
> `plat_port.c` 实现，**强符号、无 weak 兜底**）——自 `d987355` 起 host 侧格式化拼装
> 统一收敛到该平台接口（此前 `fhost.c` 用 `#define aicwf_dbg_snprintf snprintf` 宏遮蔽、
> `wlan_if.c`/`fhost_cntrl.c` 用裸 libc，故不报错）。平台若**整体替换/重写**
> `plat_port.c`，必须保留该函数，否则最终链接报
> `undefined reference to 'aicwf_dbg_snprintf'`（即上文「extern 直接链接模式：所有声明的
> 函数必须实现」）。
> 另注：平台若在 `plat_port.c` 把 `dbg_vsnprintf` 重定向到自己的串口/日志通道，
> host 侧这批格式化拼装会随之走平台通道（默认经 `compiler.h` 映射到 libc `vsnprintf`，
> 行为与改动前一致）。

其余关键项（包内模板已含注释，按需调整）：

| 配置项 | 默认 | 说明 |
|--------|------|------|
| `AIC_CHIP_MODEL` | `D80` | 与包内库型号族一致（DC/D80/DW/DC_H/DW_H） |
| `AIC_BUS_TYPE` | `SDIO` | 与库总线一致，**勿改**（铁律 3） |
| `CFLAGS += -DSYS_SOCK_DEFINE` | 开 | 平台已提供 POSIX socket 类型（lwIP） |
| `CFLAGS += -DPLATFORM_NO_FILE` | 开 | 平台无 stdio `FILE` 类型（RTOS） |
| `CFLAGS += -DCONFIG_LOAD_FW=2` | 2 | 固件加载模式：0=文件系统 1=Flash 2=内嵌数组（默认） |
| `AIC_RXTASK_INSDIO` / `AIC_IRQ_SUS` | 1 / 0 | SDIO 运行时行为（运行时变量，无需重编库）。RXTASK_INSDIO 保持默认 1（ISR 内 RX，参考实现）；仅当平台 SDIO 驱动移植时已将 `aic_sdio_rx_task()` 调用移出 ISR 改为外部 task 才设 0 |
| `USE_BUILTIN_MBEDTLS` | 1 | 1=链接 `libaic_mbedtls.a`；0=平台自实现 35 个 `aic_mbedtls_*` 符号（22 MPI + 13 ECP，契约见库公开头） |

> **⚠️ 弱内存平台 host 常量裁剪（`#ifndef` 可覆盖，但仅从源码编译时才生效）**: `USB_RX_BUF_COUNT`、`TXBUF_CNT`、`MAX_RTOS_SOCKETS`、`NX_VIRT_DEV_MAX`、`NX_REMOTE_STA_MAX` 在 host 源码中带 `#ifndef` 保护。**release 包交付的是预编译 `libaic8800_driver.a`，这些常量已固化进库**，此时在 `platform_config.mk` 用 `DFLAGS += -D<宏>=<值>` 覆盖**不会影响已发布库**。若弱内存平台（如 GD32F4/Cortex-M4）确需裁剪，须从驱动源码路径编译 host（porting 场景，`AIC_BUILD_HOST=1`），或用 `DFLAGS` 覆盖后重新出包；且覆盖值须 host 与 platform 两层一致，尤其 `NX_VIRT_DEV_MAX`/`NX_REMOTE_STA_MAX` 联动 `STA_MAX`/`FHOST_TXQ_CNT`。参考裁剪值见 `platform_config.mk`「内存裁剪」注释块。
>
> **⚠️ 裁剪与双 VIF 的冲突**: `NX_VIRT_DEV_MAX` 需 **≥ 3**（并发时 vif0=STA/P2P、vif1=扫描、vif2=SoftAP），`NX_REMOTE_STA_MAX` 必须与固件 BCMC 基数一致（默认 32）。为省 RAM 把这两个常量调小等于**放弃 APSTA/APP2P 并发能力**（详见 6.4）。

### 3.4 渐进式适配流程（逐步 make 验证）

```
Step 1: 总线适配 — sdio_port.c / usb_port.c (自动编译)
        替换 [PLATFORM] 标注点: 平台 SDIO/MMC 头文件、aic_sdio_* 封装函数、
        中断注册、扫卡就绪 → make 验证
        ↓
Step 2: RTOS 原语 — AIC_RTOS 控制 (freertos/rtthread)
        其他 RTOS: 新建 rtos_<name>.c (以 rtos_freertos.c 为模板)
        ⚠️ release 包 Makefile 若无对应 AIC_RTOS 分支, 用 SRC_PLATFORM 手动追加
        → make 验证
        ↓
Step 3: netif_port.c — 对接平台 lwIP (自动编译) → make 验证
        ↓
Step 4: plat_port.c — HAL/日志/优先级 (手动追加 SRC_PLATFORM)
        ⚠️ TCPIP_PRIORITY 以平台 TCP/IP 任务实际优先级为基准;
           CONFIG_PRIO_HIGH_IS_LARGE 按 RTOS 优先级方向设 1/0
        → make 验证
        ↓
Step 5: wifi_port_plt.c — 启动入口 (AIC_FEATURE_PLT=1)
        实现 SDIO 扫卡等待 + aic_platform_init() 电源时序 → make 验证
        ↓
Step 6: CLI 集成 — cmd_test() 注册到平台 Shell (见第七步)
        平台无 CLI 时跳过, 自行集成命令入口
        ↓
Step 7: (可选) BT/BLE — AIC_FEATURE_BT (+ AIC_FEATURE_NIMBLE 带 host 栈)
        HCI=uart 时替换 aic_bt_uart.c 的 [AIC8800-PORT] UART HAL 区 (见第八步)
```

**TODO 存根规范**: 找不到平台接口时不写假实现，用 `#if 0 ... #endif` 保留原参考实现 + `// TODO: [AIC8800-PORT]` 标记，记录到适配报告。

> **独立编译验证**: 目标 SDK 不便整体编译时，包内 `make` 产出 `build/libaic_platform.a`
> 即完成语法/类型级验证；运行时功能需在目标板上验证。

---

## 第四步: 场景二 — 默认 platform 直接适配

release 包自带 platform 层默认实现（FreeRTOS + lwIP + 标准 SDIO 栈参考）。
目标平台与参考实现接近时，**仅改 `platform_config.mk` 即可编译通过**。

### 4.1 最小配置四项

```makefile
# ① 工具链 — 与预编译库 ABI 一致
CC  = <your-cross-gcc>
AR  = <your-cross-ar>
CFLAGS_ARCH = <架构标志, 浮点 ABI 与库一致>

# ② SDK 头文件路径
EXT_INC_DIRS = <RTOS-include> <lwIP-include> <SDK-SDIO-include>
```

```bash
make info     # 确认配置生效
make          # 期望: build/libaic_platform.a 编译通过
```

### 4.2 少量 RTOS API 差异适配

同为 FreeRTOS 但版本/裁剪不同时的常见差异点（均在 `rtos_freertos.c` 内）：

| 差异点 | 症状 | 处理 |
|--------|------|------|
| 信号量/互斥量 API 版本差异 | 编译报 API 不存在（如旧版 `xSemaphoreCreateMutex()` 行为差异） | 按平台 FreeRTOS 版本调整对应 `aicwf_rtos_sem_*`/`mutex_*` 实现 |
| `xTaskCreate` 参数差异（栈单位 word vs byte） | 运行时栈溢出 | 检查 `aicwf_rtos_task_create` 中栈大小换算 |
| 定时器 API 差异（一次性/周期） | 定时器行为异常 | 检查 `aicwf_rtos_timer_*` 实现 |
| 事件组未启用（`configUSE_EVENT_GROUPS=0`） | 链接或运行错误 | SDK 配置启用事件组，或改用信号量模拟 |
| 堆实现差异（heap_1~5） | 内存分配失败 | 确认 SDK 堆配置满足驱动峰值内存 |

> RT-Thread 平台：设 `AIC_RTOS=rtthread` 即切换 `rtos_rtt.c` 参考实现
> （注意 RT-Thread 优先级数值越小越高，`CONFIG_PRIO_HIGH_IS_LARGE=0`）。

### 4.3 SDIO/lwIP 轻微差异

默认 `sdio_port.c` 基于通用 MMC/SDIO 栈（`sdio.h`/`mmcsd_core.h` 风格 API）。
平台栈命名不同时仅需调整文件顶部 `[PLATFORM]` 头文件与 7 个 `aic_sdio_*` 封装函数
（签名差异在封装函数内吸收），驱动层调用代码不动。

---

## 第五步: 场景三 — SDK 差异适配

平台已部分移植（如基于旧版 SDK），当前 SDK 版本存在 API 变化时，
流程 = **编译定位差异 → 归类 → 逐点更新 platform 实现**。

### 5.1 三大差异类别定位

| 差异类别 | 典型编译错误 | 影响文件 |
|----------|-------------|----------|
| SDIO/USB 总线接口差异 | `sdio.h: No such file`、`struct sdio_func` 成员不存在、`unknown field 'ops'` | `platform/hal/sdio_port.c` / `usb_port.c` |
| lwIP 版本差异 | `netif_add` 参数不匹配、`LWIP_VERSION` 相关宏缺失、`tcpip_callback` 签名变化 | `platform/netif/netif_port.c` |
| WiFi 框架/RTOS 差异 | FreeRTOS API 废弃警告/错误、RTOS 头拆分（如 `FreeRTOS.h` 拆分子头） | `platform/rtos/rtos_*.c`、`wifi_port*.c` 少量 |

定位方法：

```bash
make 2>&1 | tee build_err.log
grep -E "No such file|error:" build_err.log | head -30   # 按错误聚类归入上表
```

### 5.2 总线接口差异处理

`sdio_port.c` 内置 SDK 差异适配宏机制，优先用宏隔离而非改逻辑：

| 差异点 | 适配宏 |
|--------|--------|
| `host->flags`(复数) vs `host->flag`(单数) | `SDK_MMC_HOST_FLAG_SINGULAR` |
| `host->ops->enable_sdio_irq` vs `host->enable_sdio_irq` | `SDK_MMC_HOST_NO_OPS` |
| 无 `mmcsd_wait_sdio_ready` 扫卡 API | 改用延时 + 直接注册（改 `wifi_port_plt.c`） |

```makefile
# platform_config.mk
PLATFORM_DFLAGS += -DSDK_MMC_HOST_NO_OPS -DSDK_MMC_HOST_FLAG_SINGULAR
```

新差异无现成宏时：在 `aic_sdio_*` 封装函数内做适配转换（签名差异在此吸收），
驱动层调用代码不修改。

### 5.3 lwIP 版本差异处理

| lwIP 差异点 | 版本边界 | 处理 |
|-------------|---------|------|
| `netif_add()` 参数（init 回调签名） | 1.4 vs 2.x | `netif_port.c` 中用 `#if LWIP_VERSION >= 0x02000000` 隔离 |
| `netif_set_link_up/down` 可用性 | 2.x 标配 | 旧版需 `LWIP_NETIF_LINK_CALLBACK` 启用 |
| DHCP/DNS API 头文件拆分 | 2.1+ | `EXT_INC_DIRS` 补 `lwip/apps` 路径或调整 include |
| `PBUF_LINK_ENCAPSULATION_HLEN` | ≥28 约束 | TX no-copy 路径依赖此 headroom，SDK lwipopts.h 必须 ≥28 |
| `LWIP_SUPPORT_CUSTOM_PBUF` | 1（推荐，默认即 1） | RX 零拷贝（`CONFIG_RX_NOCOPY`）路径经 `pbuf_alloced_custom` 建 custom pbuf 直指 SDIO 缓冲；平台未开该宏时 `aicwf_platform_rx_zero_copy` 自动降级为 alloc+memcpy copy 路径，不再编译错误 |
| `PBUF_POOL_SIZE` / `PBUF_POOL_BUFSIZE` | 按 RX 高速收帧 | RX 零拷贝失败时的回退拷贝路径（`aicwf_platform_net_buf_rx_alloc_copy` 内 `pbuf_alloc(...=PBUF_POOL)`）使用（参考根目录 `lwipopts.md` 的 `PBUF_POOL_SIZE=256`） |
| `LWIP_DHCPS` / `LWIP_NETIF_HOSTNAME` | 真实平台 lwipopts.h | 这些宏由平台 lwipopts.h 定义，`netif_port.c` 经 include lwip 头自动继承，**勿在 platform_config*.mk / 驱动 Makefile 用 `-D` 注入**（危害有二：① SDK lwIP 库不带该宏编译 → `struct netif` 布局与驱动库错位 4 字节（hostname 字段）→ 驱动写 netif 字段越界踩 lwIP 的 acd_list 指针槽 → DHCP ACD 阶段 acd_tmr 解引用野指针 HardFault；② 平台未开 `LWIP_DHCPS` 而驱动带 `-D` → `undefined reference to dhcpd_start`） |
| `CHECKSUM_BY_HARDWARE` | 禁止定义 | AIC8800 FW 无 TX 校验和 offload，定义后 lwIP TX 跳过软件校验 → 两头都不算 → DHCP DISCOVER IP 头校验和恒 0x0000（实测 Server 零 OFFER）。见根目录 `lwipopts.md` §8.1 |
| `LWIP_DONT_PROVIDE_BYTEORDER_FUNCTIONS` | 1（必须显式定义） | 防平台 lwIP 头将 `htons` 等函数宏化 → wpa 预编译库的 `U htons` 符号垫片无法解析（libaic8800_driver.a 内 14 个 wpa .o 引用）。见根目录 `lwipopts.md` §8.2 |

### 5.4 WiFi 框架/RTOS 接口差异处理

- FreeRTOS 10.x → 11.x：`xTaskCreateStatic` 等新增 API 不影响旧实现；若有废弃 API 警告，按新版头文件签名更新。
- RTOS 头拆分：新版 FreeRTOS 可能要求先 include `FreeRTOSConfig.h` 相关头，调整 `rtos_*.c` 顶部 include 顺序。
- 类型冲突（`u8`/`u16` redefinition）：`PLATFORM_DFLAGS += -DWPA_TYPES_DEFINED` 复用平台类型。

### 5.5 增量验证纪律

```
每修复一个差异点 → make AIC_BUILD_HOST=0 → 确认该类错误清零 → 下一个
禁止: 一次改多处后整体编译 (错误交叉难以定位)
包内 make 全部通过后 → 进入 5.6 SDK 级集成编译
最终: 对照第八步检查清单做编译后/运行时验证
```

---

### 5.6 SDK 级集成编译与错误反查

包内 `make` 通过只代表 platform 层语法/类型级正确；**链接布局、符号共存、
库放置等集成问题只有 SDK 整体编译才会暴露**。适配收尾必须做一轮 SDK 级编译。

**① 进入时机与前置动作**

```
包内 make 产出 build/libaic_platform.a (无 error)
    ↓
将驱动接入 SDK 工程 (按 SDK 构建体系二选一):
  A. 构建脚本接入: SDK 的 CMake/Make/IDE 工程注册 platform 源码目录
  B. 库接入: platform 层归档为 .a/.lib 后, 与预编译库一起加入 SDK 链接
    ↓
执行 SDK 顶层完整编译命令 (与平台正常出镜像一致)
```

若驱动包附带 `sdk_patch/`（SDK 侧修改文件 + manifest），先按 manifest 逐项
应用补丁再编译；漏打补丁是 SDK 级编译错误的高频来源。

**② SDK 级错误分类反查表**

| 错误表现 | 反查方向 | 通用处理 |
|----------|---------|---------|
| 编译期: 驱动源文件不在构建树 / include path 缺失 / 构建脚本语法错误 | SDK 集成点（构建脚本注册、`sdk_patch/`） | 对照包内 manifest 或 README 的集成步骤补齐 |
| 编译期: 类型/宏冲突（redefinition） | 平台 SDK 头与驱动头同名定义 | `PLATFORM_DFLAGS += -DWPA_TYPES_DEFINED` 等隔离宏（见 5.4） |
| 链接期: `multiple definition of <symbol>` | 平台 SDK 另有同名全局符号（常见于协议栈/加密库重叠） | 优先判断符号归属：驱动侧可改则加前缀隔离；不可改则用宏关闭驱动侧重叠功能，或改链接顺序/`--allow-multiple-definition`（仅短期规避） |
| 链接期: `undefined reference to aicwf_*` / 驱动符号 | 库未进入链接 / 型号库选错 / platform 接口未实现 | 核对库放置路径与构建系统链接列表；`AIC_CHIP_MODEL` 与包内库一致（铁律 3）；对照 `port/inc` 补实现 |
| 链接期: `undefined reference to <平台符号>` | platform 实现调用了 SDK 未启用的模块 | 在 SDK 配置中启用对应模块，或改用等效 API |
| 链接期/出镜像: 内存/镜像区溢出 | 驱动大数据段（如内嵌固件数组、大缓冲池）超出分区 | 反查平台链接布局/分区配置头，扩容对应 region；或裁剪驱动功能减小体积 |
| 运行时: 上电即死机 | 中断/优先级/栈配置与平台冲突 | 回到 8.3 运行时检查清单逐项排查 |

**③ 日志过滤技巧**

SDK 全量编译日志动辄数万行，直接读效率极低：

```bash
# 全量日志先落盘，再按需提取（可反复 grep，不必重编）
<SDK 编译命令> 2>&1 | tee sdk_build.log

# 只看驱动目录相关错误
grep -E "<driver_path>/|aicwf_|aic_" sdk_build.log | grep -i error

# 只看链接期错误（在日志尾部）
tail -200 sdk_build.log | grep -E "undefined|multiple|overflow|region"

# 首错优先: 编译期连锁错误成串, 定位第一处 error 往往即可归因
grep -n -m 5 -i "error" sdk_build.log
```

要点：
- **同一错误聚类**：相同错误重复 N 次 = 一个根因，勿逐条处理
- **编译期与链接期分开分析**：编译期错误先清零，再处理链接期
- 反查结论回填到适配记录（哪些是 platform 层问题、哪些是 SDK 侧补丁问题），
  便于后续升级驱动版本时复用

---

## 第六步: 场景四 — 平台 WiFi 框架 Wrap

目标平台已有自研/第三方 WiFi 管理框架（自研 Wi-Fi Manager、系统网络管理服务等），
需编写 **wrap/adapter 层**将平台框架调用映射到驱动接口。Wrap 是**纯应用层实现**，
放在平台自己的目录，只 include 驱动公开头（`lib/include/` + `port/inc/`），链接驱动库；
不涉及驱动内部的安全认证流程（扫描/连接中的 WPA 握手由驱动内部完成，对 wrap 透明）。

### 6.1 驱动 WiFi API 总览（`lib/include/drv_api/inc/wifi_api.h`）

| 操作类别 | 驱动接口 | 说明 |
|----------|---------|------|
| 初始化 | `aic_wifi_init(mode, param)` / `aic_wifi_deinit(mode)` | mode 区分 STA/AP |
| 模式 | `aic_wifi_mode_get()` / `aic_wifi_mode_set(mode)` | — |
| 扫描 | `aic_wifi_scan()` / `aic_wifi_super_scan(ssid)` / `aic_wifi_getscan(&ap_list)` | 异步扫描 + 取结果 |
| STA 连接 | `aic_wifi_start_sta(ssid, pw, timeout_ms)` | 阻塞式连接（含 WPA 握手），返回状态 |
| STA 断开 | `aic_wifi_stop_sta()` | — |
| STA 状态 | `aic_wifi_get_sta_status()` / `aic_wifi_ap_rssi_get()` / `aic_wifi_get_sta_connected_ap_info()` | — |
| AP 启停 | `aic_wifi_start_ap(&cfg)` / `aic_wifi_stop_ap()` / `aic_wifi_get_ap_status()` | `struct aic_ap_cfg` 配 SSID/密码/信道 |
| P2P | `aic_wifi_start_p2p(&cfg)` / `aic_wifi_stop_p2p()` / `aic_wifi_get_p2p_status()` | 包支持 P2P 时（`AIC_FEATURE_P2P=1`） |
| 事件回调 | `aic_wifi_sta_status_cb_set(func)` | 注册 STA 状态变化回调 |
| 电源管理 | `aic_wifi_set_ps_lvl(lvl)` / `aic_wifi_set_lp_level(...)` | 省电等级 |
| 其他 | 黑白名单、固定速率、信道/功率配置、温度读取等 | 见 `wifi_api.h` 完整声明 |

事件类型定义见 `lib/include/drv_api/inc/wifi_driver_event.h`：
`wifi_sta_event`（ASSOC/DISASSOC/ASSOC_FAIL）、`wifi_ap_event`（STA 关联/离线）、
`wifi_p2p_event`、`wifi_dev_event`（驱动加载状态）。

### 6.2 Wrap 实现模板

```c
/* my_wifi_adapter.c — 平台 WiFi 框架 → AIC8800 驱动 wrap 层 */
#include "wifi_api.h"          /* lib/include/drv_api/inc/ */
#include "wifi_def.h"
#include "wifi_driver_event.h"

/* ---- 平台框架: scan ---- */
int platform_wifi_scan_start(void)
{
    return aic_wifi_scan();                    /* 异步, 完成后 getscan 取结果 */
}

int platform_wifi_scan_result(platform_ap_info_t *out, int max)
{
    wifi_ap_list_t list;
    aic_wifi_getscan(&list);                   /* 映射字段: ssid/bssid/rssi/channel/security */
    /* TODO: 将 list 内容转换为平台框架的 ap_info 结构 */
    return 0;
}

/* ---- 平台框架: connect / disconnect ---- */
int platform_wifi_connect(const char *ssid, const char *psk)
{
    /* 阻塞式连接, timeout_ms 按平台框架要求传入 */
    return aic_wifi_start_sta((uint8_t *)ssid, (uint8_t *)psk, 30000);
}

int platform_wifi_disconnect(void)
{
    return aic_wifi_stop_sta();
}

/* ---- 平台框架: AP 启停 ---- */
int platform_wifi_ap_start(const char *ssid, const char *psk, uint8_t channel)
{
    struct aic_ap_cfg cfg;
    /* TODO: 填充 cfg (ssid/psk/channel/加密方式) */
    return aic_wifi_start_ap(&cfg);
}

int platform_wifi_ap_stop(void)
{
    return aic_wifi_stop_ap();
}

/* ---- 平台框架: 事件上报 ---- */
static void my_sta_status_cb(void)
{
    int status = aic_wifi_get_sta_status();
    /* TODO: 转换为平台框架事件 (CONNECTED/DISCONNECTED) 并上报 */
    (void)status;
}

void platform_wifi_adapter_init(void)
{
    aic_wifi_sta_status_cb_set(my_sta_status_cb);   /* 注册驱动事件回调 */
}
```

### 6.3 Wrap 注意事项

```
☐ wrap 层只 include 驱动公开头 (lib/include/ 与 port/inc/), 严禁引用 host 内部头
☐ aic_wifi_start_sta() 为阻塞式 (内部完成扫描+认证+4-way 握手),
   不要在平台框架的中断/高优先级任务中直接调用, 放到框架的 WiFi 工作线程
☐ 事件方向: 驱动 → 回调 (cb_set 注册) → wrap 转换 → 平台框架事件总线
☐ 驱动启动顺序不可颠倒: aic_platform_init → 总线就绪 → wifi_driver_init →
   GetWiFiInitStat() 就绪 → 才能调用 aic_wifi_* 业务接口 (参考 wifi_port_plt.c
   的 aic_wifi_demo_test() 调用链)
```

### 6.4 双 VIF 并发（APSTA / APP2P）适配

发布包内置双虚拟接口并发能力（APSTA = STA+SoftAP，APP2P = SoftAP+P2P GO），平台侧无需额外宏，
但需按下面的**接口索引**与**启动顺序**规则调用，否则表现为"第二个接口起不来"。

| 组合 | 启动顺序（硬约束） | 违反后果 |
|------|-------------------|---------|
| APSTA | 先 `aic_wifi_start_sta()`，后 `aic_wifi_start_ap()` | 并发不成立 |
| APP2P | **先 `aic_wifi_start_p2p()`，后 `aic_wifi_start_ap()`** | 返回 `-12`（host+app 双层校验） |

**vif 索引不要硬编码 0**：并发时 STA/P2P 占 vif0、扫描保留 vif1、SoftAP 落在
`softap_vif_idx`（通常 vif2）。所有带 `fhost_vif_idx` 参数的接口都要用 `aic_wifi_get_ap_vif_idx()`
取真实索引：

```c
int ap_vif = aic_wifi_get_ap_vif_idx();
aic_ap_macaddr_acl(ap_vif, acl);          /* ACL 开关 */
aic_wifi_add_blacklist(ap_vif, &mac);     /* 黑白名单 */
aic_ap_wps_pbc(ap_vif);                   /* WPS */
```

平台侧配套：

```
☐ host 库常量: 发布包已内置 NX_VIRT_DEV_MAX=4 (需 ≥3) 与 NX_REMOTE_STA_MAX=32 (须与固件 BCMC
  基数一致), 平台侧不要也无法再改 —— 若要省 RAM 自行裁剪这两个宏, 等于放弃并发能力
☐ netif 槽位: platform/netif/netif_port.c 的 PLAT_NETIF_POOL_SIZE (默认 4) ≥ 并发 vif 数
☐ AP 侧 DHCP Server: lwipopts.h 开 LWIP_DHCPS (SoftAP 与 P2P GO 共用)
☐ 信道跟随: 并发时 SoftAP 跟随 STA/P2P 已连接信道, 依赖 STA 侧 ctrl STATUS 可用
☐ 单侧退出: aic_wifi_stop_ap() / stop_sta() / stop_p2p() 后对端应恢复单模式, 无 vif 残留
```

验证：两路同时 ping/iperf；`aic_ap_get_associated_sta_list()` 能列出 AP vif 关联站点；
单侧 stop 后对端业务不中断，再重新 start 可复现。命令序列见包内 `docs/2-AT指令说明.md`
的「双 VIF 并发（APSTA / APP2P）启动顺序」。

---

## 第七步: 场景五 — CLI 命令集成

驱动提供统一的 WiFi/AT 命令入口 `aic_wifi_cmd()`（`wifi_port_cmd.c` 命令分发表，自动编译），
`wifi_port_plt.c` 中的 `cmd_test()` 是其串口转发包装。将命令注册到平台 CLI/Shell 框架即可。

### 7.1 命令入口

```c
/* platform/wifi/wifi_port_cmd.c 定义 (包内源码, 自动编译; 无公开头声明) */
void aic_wifi_cmd(char *atcmd, unsigned int athandle, unsigned char chan_num);

/* wifi_port_plt.c (AIC_FEATURE_PLT=1 时编译) */
void cmd_test(const char *temp_uart_buf);            /* 串口命令字符串转发包装 */
```

> **注意原型**: `aic_wifi_cmd()` 的参数是 **整行命令串 + AT 句柄 + 通道号**，不是
> `argc/argv`。最简对接传 `(buf, 0, 0)`；命令长度上限 64 字节（含 `\0`），超长直接丢弃。

命令语法与支持列表见包内 `docs/2-AT指令说明.md`。

### 7.2 三大框架注册示例

```c
/* ── RT-Thread FinSH ── */
#include <finsh.h>
static int wifi_cmd_entry(int argc, char *argv[])
{
    if (argc > 1) cmd_test(argv[1]);
    return 0;
}
MSH_CMD_EXPORT(wifi_cmd_entry, AIC8800 WiFi AT command);

/* ── FreeRTOS+CLI ── */
static BaseType_t prvWiFiCmd(char *out, size_t out_len, const char *cmd)
{
    cmd_test(cmd);
    return pdFALSE;
}
static const CLI_Command_Definition_t xWiFiCmd = {
    "wifi", "\r\nwifi <cmd>: AIC8800 AT command\r\n", prvWiFiCmd, 1 };
FreeRTOS_CLIRegisterCommand(&xWiFiCmd);

/* ── ThreadX / 自定义串口 Shell ── */
/* 在 Shell 命令表中登记回调, 内部调用 cmd_test(buf) 或直接 aic_wifi_cmd(): */
void shell_wifi_handler(char *line)
{
    aic_wifi_cmd(line, 0, 0);
}
```

### 7.3 平台无 CLI 框架时

```
方案 A (推荐): 保持 AIC_FEATURE_PLT=0 (默认), wifi_port_plt.c 不编译;
              自行在应用层实现启动流程 (参考 aic_wifi_demo_test 调用链)
              与命令入口 (直接调用 aic_wifi_cmd)。
方案 B: 保留 wifi_port_cmd.c 自动编译 (命令分发表), 仅跳过 wifi_port_plt.c,
              通过 UART 中断收包 → 拼帧 → aic_wifi_cmd() 自建极简命令行。
```

> `wifi_port_cmd.c` 是命令分发表本体（自动编译）；`wifi_port_plt.c` 是启动 Demo +
> `cmd_test` 包装（`AIC_FEATURE_PLT` 控制）。二者可独立取舍。

### 7.4 命令响应通道 `aicwf_ATRESP()`（必须实现）

命令分发表的所有回包、扫描结果、连接结果都经**平台函数** `aicwf_ATRESP()` 输出：

```c
/* port/inc/plat_port.h — 平台必须提供实现 */
void aicwf_ATRESP(int handle, int type, int level, char *buf);

/* platform/hal/plat_port.c — 包内参考实现 (默认直接打串口) */
```

适配要点：

```
☐ 包内 platform/hal/plat_port.c 不会被 standalone Makefile 自动编译, 需在 platform_config.mk
  追加: SRC_PLATFORM += $(PLATFORM_PATH)/hal/plat_port.c
  (未编入 → 链接期 undefined reference to 'aicwf_ATRESP')
☐ 平台已有 AT 框架: 在自建实现里按 handle 选择回包通道, 把命令结果回给发起命令的会话
☐ 平台无 AT 框架: 沿用参考实现 (串口打印) 即可, 但需保证 log 串口已初始化
☐ 命令是异步字符串回包, 与 aic_wifi_cmd() 返回值无关; 需要结构化结果请改用 wifi_api.h 的
  aic_wifi_getscan() / aic_wifi_get_sta_status() 等接口
```

### 7.5 命令前置 token（`AICWF_CMD_SKIP_PREFIX`）

`aic_wifi_cmd()` 先把整行按空白切分为 `argv[]`，再按 `AICWF_CMD_SKIP_PREFIX`（默认 `1`）跳过命令前的占位 token：

| 平台 Shell 形式 | 取值 |
|-----------------|------|
| 命令前有占位 token（如 `wifi iperf start ...`） | `1`（默认，无需修改） |
| 命令本身就是首 token（如 `iperf start ...`） | `0`，`DFLAGS += -DAICWF_CMD_SKIP_PREFIX=0` |

> 该宏在 `wifi_port_cmd.c` 顶部以 `#ifndef` 保护，平台可用编译宏覆盖，无需改源码。

---

## 第八步: 场景六 — BT/BLE（NimBLE）集成

需要蓝牙能力时（BT controller 通道 + 可选 NimBLE host 协议栈）按本节适配。release/diff 包
**默认不含** `STACK/nimble`（规避约 59 万行上游代码），需 BT 上层栈时按下面步骤单独取包。

### 8.1 包内 BT 相关内容

| 内容 | 位置 | 说明 |
|------|------|------|
| HCI 桥接 | `platform/bt/aic_hci.c` | H4 编解码 + RX 队列，对 NimBLE 提供 `aic_hci_init/send/set_rx_cb/deinit` |
| 传输后端（三选一） | `platform/bt/aic_bt_sdio.c` / `aic_bt_uart.c` / `aic_btusb.c` | 由 `AIC_HCI_TRANSPORT`（`sdio`/`uart`/`usb`）决定编哪个 |
| BT 公开 API 头 | `lib/include/drv_api/inc/bt_api.h` | `aic_bt_hci_send` / `aic_bt_hci_set_rx_cb` / `aic_bt_fw_ready` / `aic_bt_fw_reset` |
| NimBLE 协议栈 | `STACK/nimble/` | **条件携带**：仅 `AIC_FEATURE_NIMBLE=1` 时随包 |
| 命令通道 | `wifi_port_cmd.c` 的 `btstart` / `btstop` | 需 `AIC_FEATURE_BT=1` |

### 8.2 使能与配置

```makefile
# platform_config.mk / 命令行
AIC_FEATURE_BT   ?= 1          # 编入 platform/bt HCI 通道 + host BT 固件表
AIC_HCI_TRANSPORT ?= uart      # sdio(默认, 邮箱) | uart | usb
AIC_FEATURE_NIMBLE ?= 1        # 需要 BLE 上层协议栈时
```

```
☐ 只有 BT controller 通道、不要 host 栈 → AIC_FEATURE_NIMBLE=0 (双门控, 不会缺符号)
☐ BT UART 场景: 替换 platform/bt/aic_bt_uart.c 顶部 [AIC8800-PORT] 区
  (BT_UART_PORT_ID / BT_UART_BAUDRATE / xUartOpen / vUartInit / iUartRead / iUartWrite)
  —— 包内实现依赖平台 uart.h, 不替换会编译失败
☐ 切换 AIC_FEATURE_BT / AIC_HCI_TRANSPORT / AIC_FEATURE_NIMBLE 后须 make clean 重编
☐ 型号限制: DW/DW_H 强制关闭 BT; DC_H + sdio 邮箱需 _hbt 固件变体 (驱动自动选)
```

### 8.3 NimBLE 协议栈编译

包内 `STACK/nimble` 自带独立构建脚本（复用平台的 `CC/AR/CFLAGS_ARCH`），产出 `libnimble.a`：

```bash
# 在 release/diff 包根目录
make AIC_FEATURE_NIMBLE=1 AIC_FEATURE_BT=1      # 委托 STACK/nimble/Makefile
# 或直接进入协议栈目录
make -C STACK/nimble CC=<cross-gcc> AR=<cross-ar> CFLAGS_ARCH="<平台架构标志>"

# CMake 路径
cmake -DAIC_FEATURE_NIMBLE=ON -DAIC_FEATURE_BT=ON -DAIC_HCI_TRANSPORT=UART ..
```

```
☐ libnimble.a 独立于 libaic8800_driver.a (tinycrypt utils.c 与 wpa ap/utils.c 同名符号冲突),
  链接时两个库都要加, 放在引用方目标文件之后
☐ NimBLE OS 抽象只用 port/inc/rtos_port.h 原语, 不需要额外 SDK 适配
☐ 协议栈源码在包内以相对路径引用 lib/include (公开头) 与 platform/bt, 无需 host/ 源码
```

### 8.4 运行时

```
☐ 自行实现启动入口 (AIC_FEATURE_PLT=0) 时, 在 WiFi bring-up 完成后调 nimble_stack_start(),
  并用 #if defined(AIC_FEATURE_BT) && defined(AIC_FEATURE_NIMBLE) 门控
☐ nimble_stack_start() 内部自检 aic_bt_fw_ready(); BT patch 未加载则拒绝启动 (先确认 WiFi 侧
  驱动初始化完成, BT patch 随 aicbt_init 加载)
☐ 命令验证: btstart / btstop 幂等; btstop 只停 BLE, 不影响 WiFi 业务
☐ 回归: BLE 广播可见 → 连接/读写 GATT → 与 WiFi 并发收发无异常
```

---

## 第九步: 适配检查清单

### 9.1 编译前

```
☐ readelf/file 确认库架构与平台工具链 ABI 一致 (铁律 1)
☐ nm 确认库总线符号与 AIC_BUS_TYPE 配置一致 (铁律 3)
☐ aic_cfg.mk 的 AIC_CHIP_MODEL 与目标硬件芯片一致 (决定固件选择, 与库无关)
☐ CC/AR/CFLAGS_ARCH 写入 platform_config.mk
☐ EXT_INC_DIRS 覆盖: RTOS 头 / lwIP 头 / 总线驱动头 / 芯片头
☐ plat_port.h: TCPIP_PRIORITY / CONFIG_PRIO_HIGH_IS_LARGE 按平台 RTOS 设置
☐ lwipopts.h: PBUF_LINK_ENCAPSULATION_HLEN ≥ 28 (TX no-copy 依赖)
☐ lwipopts.h: LWIP_SUPPORT_CUSTOM_PBUF = 1 (RX no-copy 推荐开启, 未开则自动降级 copy)
☐ lwipopts.h: CHECKSUM_BY_HARDWARE 未定义 (FW 无 TX offload, 定义则 DHCP DISCOVER 校验和恒 0 → 零 OFFER, 见 5.3 / lwipopts.md §8.1)
☐ lwipopts.h: LWIP_DONT_PROVIDE_BYTEORDER_FUNCTIONS = 1 (防 htons 宏化破坏 wpa 预编译库符号垫片, 见 lwipopts.md §8.2)
☐ SRC_PLATFORM 已含 platform/hal/plat_port.c (命令响应 aicwf_ATRESP 所在, 见 7.4)
☐ 需要 BT/BLE 时: AIC_FEATURE_BT / AIC_HCI_TRANSPORT / AIC_FEATURE_NIMBLE 已确认 (见 8.2)
☐ 需要双 VIF 并发时: 不裁剪 host 常量 (发布包已内置 NX_VIRT_DEV_MAX=4 / NX_REMOTE_STA_MAX=32)
☐ 需要 AP/DHCP 服务时: lwipopts.h 已开 LWIP_DHCPS
☐ grep -rn 'LWIP_NETIF_HOSTNAME\|LWIP_DHCPS' Makefile platform_config*.mk
   确认构建配置无 -D 注入 (必须由平台 lwipopts.h 定义, 见 5.3 差异表)
```

### 9.2 编译后

```
☐ make 成功产出 build/libaic_platform.a, 无 warning 堆积
☐ nm 检查 platform 库导出全部 aicwf_* 实现符号 (T 类型)
☐ SDK 工程链接无 undefined reference:
   libaic_platform.a + libaic8800_driver.a + libaic_mbedtls.a
   (+ 启用 NimBLE 时再加 libnimble.a, 放在引用方目标文件之后)
☐ grep -rn '#include' platform/ 核对无 host 内部头越界引用
   (release 包无 host/, 若误引用会直接编译失败)
☐ 启用 BT 时切换过开关的构建已 make clean (避免旧 .o 残留)
```

### 9.3 运行时（目标板）

```
☐ 上电时序: aic_platform_init() 电源使能/复位延时符合芯片手册
☐ 总线枚举: SDIO 扫卡成功 (detect sdio device 日志) / USB 设备识别
☐ 固件加载: 固件下载完成日志 (innermode 无文件依赖)
☐ 驱动就绪: GetWiFiInitStat() 返回 Ready
☐ 网络接口: netif 注册, DHCP 获取 IP (STA) 或 DHCP Server 启动 (AP)
☐ 扫描/连接: aic_wifi_scan + aic_wifi_start_sta 连通 AP, ping 通网关
☐ 吞吐: iperf 或大流量拷贝验证 (对比附录定频/功耗文档基线)
☐ 稳定性: 长时间连接 + 断线重连 (参考 docs/4-常见问题说明.md)
☐ 命令通道: Shell 下发一条命令能收到 aicwf_ATRESP 回包 (如 scan/stainfo)
☐ BT/BLE (如启用): 可被扫描到广播, btstart/btstop 幂等且不影响 WiFi
☐ 双 VIF (如启用): APSTA/APP2P 按顺序启动成功, 单侧 stop 后对端不受影响 (见 6.4)
```

> **提交前 review**: 完成 9.1~9.3 检查清单后，调用 `aic8800-review` 统一 review（场景标签: `release`），重点复核分层边界与 host 内部头越界。

---

## 第十步: 常见编译错误排错

| 错误类型 | 典型信息 | 根因 | 解决方案 |
|----------|---------|------|----------|
| SDK 头缺失 | `sdio.h: No such file` | `EXT_INC_DIRS` 未含 SDK 路径 | 补 C 类头文件路径（RTOS/lwIP/总线驱动） |
| 类型 redefinition | `redefinition of typedef 'u8'` | 平台 SDK 已定义短名类型 | `PLATFORM_DFLAGS += -DWPA_TYPES_DEFINED` |
| 链接未定义 | `undefined reference to 'aicwf_xxx'` | platform 函数未实现/源文件未编译 | 对照 `port/inc` 声明补齐实现；手动项查 `SRC_PLATFORM` |
| 总线符号错链 | `undefined reference to 'aicwf_sdio_send'` | platform 编了 USB 版而库是 SDIO 版（或反之） | `AIC_BUS_TYPE` 改回与库一致（铁律 3） |
| ABI 不匹配 | `uses hard float, output uses soft float` / VFP 相关 | 平台工程浮点 ABI 与库不一致 | `CFLAGS_ARCH` 浮点标志与库对齐；架构不符则申请对应库 |
| 型号固件错配 | 固件下载失败 / 上电无响应 | `AIC_CHIP_MODEL` 与实际硬件芯片不一致（platform 侧固件数据错） | `AIC_CHIP_MODEL` 改为实际硬件型号（单库本身支持全型号） |
| 命令响应缺失 | `undefined reference to 'aicwf_ATRESP'` | `platform/hal/plat_port.c` 未加入编译（standalone Makefile 不自动编） | `SRC_PLATFORM += $(PLATFORM_PATH)/hal/plat_port.c` 或自建实现（见 7.4） |
| BT 符号错链 | `undefined reference to nimble_stack_start` | 开了 `AIC_FEATURE_BT` 但未编 NimBLE | 需 BLE 栈则编 `STACK/nimble`（见 8.3）；不需要则 `AIC_FEATURE_NIMBLE=0`（默认，双门控不引用该符号） |
| BT UART 编译失败 | `uart.h: No such file` / `xUartOpen` 未定义 | `AIC_HCI_TRANSPORT=uart` 但未替换 `aic_bt_uart.c` 的 `[AIC8800-PORT]` 区/未配 `EXT_INC_DIRS` | 替换 UART HAL 区并补 SDK UART 头路径（见 8.2） |
| 格式警告 | `format '%d' expects 'int'...uint32_t` | RV32 上 `uint32_t` 为 `long unsigned int` | 改 `%ld` 或 `PRIu32`；不影响功能 |
| lwIP API 不匹配 | `netif_add` 参数错误 | lwIP 版本差异 | 见 5.3 版本差异表，用 `LWIP_VERSION` 宏隔离 |
| 栈溢出（运行时） | 任务跑飞/重启 | RTOS 栈单位换算或栈宏过小 | 检查 `aicwf_rtos_task_create` 栈换算与 `plat_port.h` 栈宏 |

---

## 附录: 快速参考

### A. 典型命令序列

```bash
cd <driver>_release

# 1. 确认库属性 (架构/总线; 型号由运行时硬件决定无需确认)
riscv64-unknown-elf-readelf -h lib/libaic8800_driver.a | head
nm lib/libaic8800_driver.a | grep -E "T aicwf_(sdio|usb)_send"

# 2. 编辑 platform_config.mk (工具链/EXT_INC_DIRS/宏/SRC_PLATFORM)

# 3. 编译验证
make info          # 查看生效配置
make               # → build/libaic_platform.a

# 4. SDK 工程链接
#    libaic_platform.a + libaic8800_driver.a + libaic_mbedtls.a
```

### B. 配套文档

| 文档 | 用途 |
|------|------|
| `BUILD.md` | 构建系统与 port 层设计说明 |
| `docs/1-适配编译说明.md` | 适配编译详细说明（头文件三层分类等） |
| `docs/2-AT指令说明.md` | CLI/AT 命令语法与列表 |
| `docs/3-定频测试说明.md` | RF 定频测试（生产校准） |
| `docs/4-常见问题说明.md` | 常见问题 FAQ |

> **后续版本升级**：基准驱动发布新版本时的同步（含 `[PLATFORM]` 标注回填、锚点维护）走 `aic8800-sync` skill；重新生成 release/diff 包走 `aic8800-diff-package` skill。
