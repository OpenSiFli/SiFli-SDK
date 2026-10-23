# AIC8800 WiFi Driver - 构建说明

> **本集成副本的构建方式**: 顶层 `Makefile` 已移除, platform 适配层随 SDK 用
> **SCons** 编译 (入口 `SConscript`, 开关见同目录 `Kconfig`: 总开关
> `AIC_WLAN_AIC8800D40N`, 子项在 `AIC8800D40N Parameter` 菜单); host 层仍是
> `release/lib` 下的预编译库。下文 Makefile/CMake 章节保留作**历史参考**,
> 不适用于当前 SDK 构建。

## 目录

- [项目概述](#项目概述)
- [目录结构](#目录结构)
- [构建系统](#构建系统)
  - [两阶段编译架构](#两阶段编译架构)
  - [host Makefile](#host-makefile)
  - [顶层 Makefile](#顶层-makefile)
- [编译方法](#编译方法)
  - [快速开始](#快速开始)
  - [交叉编译](#交叉编译)
  - [自定义选项](#自定义选项)
  - [BT 与 NimBLE 构建开关](#bt-与-nimble-构建开关)
  - [双 VIF 并发](#双-vif-并发apsta--app2p)
- [Release 发布机制](#release-发布机制)
- [Port 移植层详解](#port-移植层详解)
  - [设计原则](#设计原则)
  - [port/inc 头文件清单](#portinc-头文件清单)
  - [类型定义控制](#类型定义控制)
- [平台移植指南](#平台移植指南)
  - [移植总览](#移植总览)
  - [各模块说明](#各模块说明)
  - [移植步骤](#移植步骤)
- [编译宏定义说明](#编译宏定义说明)
- [mbedTLS 外部接口层](#mbedtls-外部接口层)
- [日志框架](#日志框架)
- [CMake 构建 (辅助)](#cmake-构建-辅助)

---

## 项目概述

本项目为 AIC8800 系列 WiFi/BT 芯片的 RTOS 驱动程序，采用分层架构设计：

- **host/driver** — 平台无关的驱动核心层（FMAC 驱动、MAC 接口、WPA Supplicant、mbedTLS）
- **platform** — 平台相关的移植层（RTOS 适配、SDIO 总线、网络接口、WiFi 端口等）

驱动采用 **单库统一架构**：host 驱动库与芯片型号无关，所有型号（8801/DC/DW/D80/D80N/D80X2）共用同一个 `libaic8800_driver.a`；型号差异由 platform 侧提供的 `aic_chip_desc` 数据表在运行时分发（运行时 chipid 查表，见 [芯片型号](#芯片型号) 一节）。

驱动通过 **extern 直接链接** 模式实现平台与驱动核心的解耦：host 层通过 port 头文件中的宏和 extern 声明调用平台函数，platform 层直接实现这些函数，链接器在最终链接时完成绑定。

---

## 目录结构

```
aic8800_rtos/
├── Makefile                         # 顶层构建 (仅编译 platform, 引用 release/)
├── aic_cfg.mk                       # 功能裁剪开关 (P2P/BT/WPA日志)
├── platform_config.mk               # 平台配置 (工具链/头文件/源文件/类型定义宏)
├── port/
│   └── inc/                         # 移植抽象层头文件 (host + platform 共用)
│       ├── aic_porting_types.h      #   平台无关基础类型 (stdint/bool/BIT)
│       ├── rtos_port.h              #   RTOS 函数声明 + 类型定义
│       ├── sdio_port.h              #   SDIO 函数声明
│       ├── netif_port.h             #   网络接口函数声明
│       ├── plat_port.h              #   HAL/平台函数声明
│       ├── aic_log.h                #   模块化日志宏
│       ├── aic_netbuf.h             #   网络 buffer 类型 (net_buf_tx_t)
│       ├── aic_sdio_types.h         #   SDIO 类型定义
│       ├── aic_fw_port.h            #   固件 port 接口
│       ├── fmac_port.h              #   FMAC 平台相关定义
│       ├── wifi_port.h              #   WiFi 类型定义
│       ├── aic_assert.h             #   断言宏
│       ├── aic_cfg.h                #   功能配置宏
│       ├── aic_chip_desc.h          #   芯片差异数据表接口 (单库运行时分发)
│       └── userconfig.h             #   用户配置接口
├── host/
│   ├── Makefile                     # host 编译 & release 生成
│   ├── host_config.mk               # host 层独立编译配置
│   ├── build_SDIO/ build_USB/       # host 编译输出 (按总线区分, 统一产出 libaic8800_driver.a)
│   ├── driver/
│   │   ├── drv_api/inc/             #   驱动对外 API 头文件
│   │   ├── drv_fhost/               #   Full-hosted 驱动逻辑
│   │   ├── drv_macif/               #   MAC 接口层
│   │   ├── drv_wifi/                #   WiFi 接口
│   │   ├── drv_wlan/                #   WLAN 接口
│   │   ├── drv_bt/                  #   BT 驱动
│   │   ├── porting/src/             #   桩实现 (空文件)
│   │   ├── layer_common/            #   通用抽象层
│   │   └── utils/                   #   工具函数
│   └── thirdparty/
│       ├── wpa_supplicant/          # WPA Supplicant
│       └── mbedtls_v3_6_2/          # mbedTLS v3.6.2
├── platform/                        # 平台适配层 (参考实现)
│   ├── platform_init.c              # 统一初始化入口 (extern 模式下为空)
│   ├── fmac/                        # 固件管理适配
│   │   ├── aic_fw_port.c            #   固件加载实现 (固件数据按型号宏门控)
│   │   ├── aic_chip_desc_port.c     #   芯片差异数据表 (单库运行时分发数据源)
│   │   ├── fw/                      #   固件头文件 (C 数组)
│   │   └── userconfig.c             #   用户配置数据
│   ├── hal/                         # 硬件抽象适配 (SDIO/GPIO/电源)
│   │   ├── plat_port.c              #   HAL 函数实现
│   │   └── sdio_port.c              #   SDIO 驱动实现
│   ├── iperf/                       # iperf 测速工具
│   │   └── iperf.c                  #   iperf 实现
│   ├── netif/                       # 网络接口适配
│   │   └── netif_port.c             #   LwIP netif 对接
│   ├── rtos/                        # RTOS 适配
│   │   └── rtos_freertos.c          #   FreeRTOS 抽象层实现
│   ├── bt/                          # 蓝牙适配
│   │   └── aic_bt_uart.c            #   BT UART HCI 实现
│   └── wifi/                        # WiFi 业务逻辑适配
│       ├── wifi_port.c              #   WiFi 核心生命周期
│       ├── wifi_port_sta.c          #   STA 模式
│       ├── wifi_port_ap.c           #   AP 模式
│       ├── wifi_port_wps.c          #   WPS 配网
│       ├── wifi_port_p2p.c          #   P2P
│       ├── wifi_port_cmd.c          #   AT 命令分发
│       └── wifi_port_plt.c          #   驱动启动 Demo + AT 命令测试入口
├── release/                         # 发布产物 (make release 生成, 自包含)
│   ├── Makefile                     #   构建脚本 (第三方入口)
│   ├── aic_cfg.mk                   #   功能裁剪开关
│   ├── platform_config.mk           #   平台配置 (第三方编辑此文件)
│   ├── lib/                         #   预编译库 (单库统一, 与芯片型号无关)
│   │   ├── libaic8800_driver.a      #     驱动核心库 (含 WPA, 全型号共用)
│   │   └── libaic_mbedtls.a         #     mbedTLS 加密库 (通用)
│   ├── include/                     #   公开头文件
│   ├── port/inc/                    #   移植接口头文件
│   └── platform/                    #   平台适配层源码
├── build/                           # platform 编译输出 (libaic_platform.a)
├── CMakeLists.txt                   # CMake 构建 (辅助)
└── docs/                            # 文档
```

---

## 构建系统

### 两阶段编译架构

支持**调试模式**（`AIC_BUILD_HOST=1`，默认）和**发布模式**（`AIC_BUILD_HOST=0`）两种工作流：

```
┌─ 调试模式 (AIC_BUILD_HOST=1, 默认) ────────────────────────────┐
│                                                                │
│  Stage 1: host 编译                Stage 2: platform 编译        │
│  host/Makefile lib-sync   ──→    顶层 Makefile                  │
│  编译驱动库 → 镜像到 lib/         编译 platform, 链接 lib/        │
│  (每次 make 都重新编译)            (引用 port/inc/ 头文件)        │
│                                                                │
│  适用: 移植调试阶段, host 源码修改后立即生效                      │
└────────────────────────────────────────────────────────────────┘

┌─ 发布模式 (AIC_BUILD_HOST=0) ─────────────────────────────────┐
│                                                                │
│  Stage 1: host 发布                 Stage 2: platform 编译      │
│  host/Makefile release   ──→      顶层 Makefile                │
│  编译驱动库 → 输出到 release/      编译 platform, 链接 release/  │
│  (含头文件, 自包含发布包)           (引用 release/lib/include/)  │
│                                                                │
│  适用: 交付第三方, release/ 目录可独立分发                       │
└────────────────────────────────────────────────────────────────┘
```

**lib 镜像机制**（调试模式核心）：
- 顶层 `make` 通过 `host-build` 调用 `host/Makefile lib-sync`
- `lib-sync` 依赖 `all`，编译驱动库后镜像到根 `lib/` 目录（`lib/*.a` + `lib/include/` API 公开头）
- `lib/include/` 与 release 包输出形态一致，调试/发布统一引用公开头，杜绝平台代码依赖 host 内部头
- 每次 make 都从 host 重新编译生成，确保 lib 始终最新（无 mtime 缓存）
- `.gitignore` 已忽略 `/lib/`，避免提交编译产物

### host Makefile

负责编译驱动核心 + WPA Supplicant + mbedTLS，并生成发布包。

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `CC` | `gcc` | C 编译器 |
| `AR` | `ar` | 归档工具 |
| `AIC_MBEDTLS_INC_DIR` | `thirdparty/mbedtls_v3_6_2/include` | mbedTLS 头文件路径 |
| `AIC_BUILD_MBEDTLS_PORT` | `1` | 是否编译 mbedTLS port 层 |
| `AIC_BUS_TYPE` | `SDIO` | 总线类型 (SDIO/USB)，决定编译宏集合与构建目录 `build_$(AIC_BUS_TYPE)` |

> **单库统一**: host 编译自动剔除型号相关宏（`-DCONFIG_AIC88xx` / `-DUSE_5G`），`AIC_CHIP_MODEL` 不影响 host 库内容与命名，产物恒为 `libaic8800_driver.a`；BT 默认强制编入（命令行传 `AIC_FEATURE_BT=0` 可裁剪）。
> **切换总线注意**: `make clean` 只清理 `build_$(AIC_BUS_TYPE)`，切换 SDIO/USB 时须带 `AIC_BUS_TYPE=<总线> clean`，否则旧 .o 会被增量复用。

| 目标 | 说明 |
|------|------|
| `make` | 编译驱动库到 build/ |
| `make lib-sync` | 编译驱动库并镜像到 ../lib/ (lib/*.a + lib/include/, 供 Stage2 调试链接, 顶层 `make` 自动调用) |
| `make release` | 编译并生成发布包到 ../<driver>_release/ (lib + lib/include + include) |
| `make install-headers DEST=<dir>` | 安装公开头文件到指定目录 |
| `make clean` | 清理编译目录 |
| `make release-clean` | 清理 release 目录 |
| `make info` | 显示构建配置 |

### 顶层 Makefile

仅编译 platform 层，根据 `AIC_BUILD_HOST` 选择链接路径：

| 模式 | `AIC_BUILD_HOST` | 链接库路径 | 调用 host 目标 |
|------|------------------|-----------|---------------|
| 调试 (默认) | `1` | `lib/libaic8800_driver.a` | `lib-sync` |
| 发布 | `0` | `lib/libaic8800_driver.a` (驱动目录内预编译库) | 不调用 (使用预编译库) |

> 顶层 `make` 自动检测 `host/` 目录存在性：存在则 `AIC_BUILD_HOST=1`（host 编译到根 lib/），否则 `AIC_BUILD_HOST=0`（直接引用 lib/ 预编译库）。
> 两种模式头文件统一引用 `lib/include/` + `port/inc`，调试/发布形态一致。

| 变量 | 来源 | 说明 |
|------|------|------|
| `CC` | `platform_config.mk` | C 编译器 |
| `AR` | `platform_config.mk` | 归档工具 |
| `EXT_INC_DIRS` | `platform_config.mk` | 外部 SDK 头文件路径 |
| `PLATFORM_DFLAGS` | `platform_config.mk` | 平台特定编译宏 |
| `SRC_PLATFORM` | `platform_config.mk` (追加) | 平台源文件列表 |
| `AIC_BUILD_HOST` | 自动检测 / 用户指定 | 1=从 host 源码编译, 0=用预编译库 |

| 目标 | 说明 |
|------|------|
| `make` | 编译 platform (需先有 release/) |
| `make release` | 构建 host 驱动并生成自包含发布包到 release/ |
| `make diff-verify` | 生成 release 后以 AIC_BUILD_HOST=0 独立编译验证 (diff 打包前强制) |
| `make release-clean` | 清理 release 目录 |
| `make clean` | 清理 platform 构建 |
| `make info` | 显示构建配置信息 |

> **注**: 顶层 `make release` 会自动检测 `host/` 目录，从源码构建驱动并复制 platform 源码和配置文件到 release/，生成无需 host/ 的自包含发布包。

---

## 编译方法

### 快速开始

```bash
# Step 1: 生成自包含发布包 (编译驱动 + 复制 platform 源码和配置)
make release

# Step 2: 编译 platform 层 (验证移植)
make

# 第三方拿到 release/ 后:
cd release && make
```

### 交叉编译

```bash
# ARM 交叉编译: 先生成 release 再编译 platform
# (推荐先在 platform_config.mk 中配置 CC/AR/CFLAGS_ARCH, 然后直接 make)
make release CC=arm-none-eabi-gcc AR=arm-none-eabi-ar

make CC=arm-none-eabi-gcc AR=arm-none-eabi-ar
```

### 自定义选项

```bash
# host 编译选项:

# 使用平台已有的 mbedTLS (不链接内置 libaic_mbedtls.a):
# 在 platform_config.mk 中设置 USE_BUILTIN_MBEDTLS=0,
# 平台自行实现 35 个 aic_mbedtls_* 符号 (直接转发示例:
# platform/crypto/aic_mbedtls_port_example.c)
make release

# platform 编译选项 (推荐在 platform_config.mk 中配置):

# 指定外部头文件路径 (lwip, RTOS, SDK 等)
# 在 platform_config.mk 中设置:
#   EXT_INC_DIRS = /path/to/lwip/include /path/to/freertos/include

# 平台特定编译宏
# 在 platform_config.mk 中设置:
#   PLATFORM_DFLAGS = -DPLATFORM_THREADX

# 组合示例: ThreadX 平台完整编译
# (推荐在 platform_config.mk 中配置 CC/AR/CFLAGS_ARCH, 然后直接 make)
make release \
    CC=arm-none-eabi-gcc \
    AR=arm-none-eabi-ar

make \
    CC=arm-none-eabi-gcc \
    AR=arm-none-eabi-ar \
    PLATFORM_DFLAGS="-DPLATFORM_THREADX" \
    EXT_INC_DIRS="/sdk/os/threadx/inc /sdk/ipc/lwipv4v6/src/include /sdk/hal/SDIO/inc"
```

### BT 与 NimBLE 构建开关

BT 功能由四个独立开关组合控制（`AIC_HCI_TRANSPORT`、`AIC_FEATURE_NIMBLE` 为本次驱动更新新增）：

| 开关 | 作用层 | 默认 | 作用 |
|------|--------|------|------|
| `AIC_FEATURE_BT` | 构建 | 按型号 (D80N/D80X2=1) | BT 总开关: 编入 platform/bt HCI 通道 + host BT 固件表 |
| `AIC_HOST_BT_DRIVER` | host | 1 | host 库 BT 代码裁剪: =0 时库内零 BT (WiFi-only 产物) |
| `AIC_HCI_TRANSPORT` | 构建 | sdio | HCI 传输: sdio(邮箱)/uart/usb, 映射为 AIC_HCI_OVER_* 宏 |
| `AIC_FEATURE_NIMBLE` | 构建 | 0 / OFF | 是否编译 STACK/nimble host 协议栈 (独立库 `libnimble.a`; Make 路径 `make nimble`, CMake 路径 `-DAIC_FEATURE_NIMBLE=ON`) |

**职责边界**：
- `AIC_HOST_BT_DRIVER` 只决定 **host 库**是否含 BT 代码（`aicbt_init` patch 加载 + 邮箱 HCI 门面 + `bt_api`），与传输通道无关。
- `AIC_HCI_TRANSPORT` 只决定 **platform 层** HCI 后端（`aic_bt_sdio.c`/`aic_bt_uart.c`/`aic_btusb.c` 三选一）与 BT 固件变体（sdio 邮箱选 `_hbt`）；**host 库不含此宏**，单库兼容所有传输。
- 切换任一开关后须 `make clean`（增量归档不会剔除旧 `.o`）。

```bash
# WiFi-only (裁剪全部 host BT 代码)
make AIC_HOST_BT_DRIVER=0

# BT over UART
make AIC_FEATURE_BT=1 AIC_HCI_TRANSPORT=uart

# BT over SDIO 邮箱 + NimBLE 协议栈 (Make 路径: 顶层 all 会连带编 nimble)
make AIC_FEATURE_BT=1 AIC_FEATURE_NIMBLE=1
make nimble                                    # 仅编协议栈, 产出 STACK/nimble/build_<总线>/libnimble.a

# 同上, CMake 路径
cmake -DAIC_FEATURE_BT=ON -DAIC_FEATURE_NIMBLE=ON -DAIC_HCI_TRANSPORT=SDIO ..
```

> **NimBLE 链接约束**：`nimble_stack_start()` 由 `STACK/nimble` 提供，仅 `AIC_FEATURE_NIMBLE=1/ON` 时编入。platform 侧调用已用 `#if defined(AIC_FEATURE_BT) && defined(AIC_FEATURE_NIMBLE)` 双重门控，只开 BT 不编 nimble 不会缺符号。`libnimble.a` 必须**独立链接**（`ext/tinycrypt/src/utils.c` 与 wpa `src/ap/utils.c` 同名符号冲突，不入 `libaic8800_driver.a`），且放在引用方目标文件之后。release/diff 包仅在 `AIC_FEATURE_NIMBLE=1` 时携带 `STACK/nimble` 源码（默认不携带，规避约 59 万行上游代码），包内该目录可独立编译出 `libnimble.a`。

### 双 VIF 并发（APSTA / APP2P）

驱动支持双虚拟接口并发：`APSTA`（AP+STA）、`APP2P`（AP+P2P）。

- **vif 索引**：SoftAP 运行索引由 `softap_vif_idx` 记录（单独 AP 为 0；STA/P2P 已占 vif0 时 AP 落到 vif2，vif1 保留给扫描）。ACL 黑白名单、关联表遍历、信道切换均按 `softap_vif_idx` 下发，不再硬编码 vif0。
- **启动顺序**：`APP2P` 须**先 P2P 后 AP**（host+app 双层强制，违反返回 -12）；`APSTA` 为先 STA 后 AP。原“AP 先起、P2P 跟随 AP 信道”能力已调整，纯 APP2P 用户须按新顺序调用。
- **STA 上限**：AP 可关联 STA 上限由 16 提升至 32（对齐固件 BCMC sta 编号基数），依赖固件实际支持 32，相关静态数组 RAM 相应翻倍。
- **接口索引**：应用/平台调用带 `fhost_vif_idx` 参数的接口（`aic_ap_macaddr_acl`、`aic_wifi_add_blacklist`、`aic_ap_wps_pbc` 等）前须用 `aic_wifi_get_ap_vif_idx()` 取真实 AP 索引，**不要硬编码 0**。
- **平台配套**：`platform/netif/netif_port.c` 的 `PLAT_NETIF_POOL_SIZE`（默认 4）需 ≥ 并发 vif 数；AP 侧 DHCP Server 需平台 `lwipopts.h` 开 `LWIP_DHCPS`（SoftAP 与 P2P GO 共用）。
- **host 常量红线**：`NX_VIRT_DEV_MAX` 需 ≥ 3（vif0=STA/P2P、vif1=扫描、vif2=SoftAP），`NX_REMOTE_STA_MAX` 必须与固件 BCMC 基数一致（默认 32）。为省 RAM 裁剪这两个常量等于放弃并发能力（弱内存平台裁剪参考见 `platform_config.mk`）。

---

## Release 发布机制

`make release` 可在 **顶层** 直接执行，自动构建 host 驱动并生成自包含发布包到 `release/` 目录。无需进入 `host/` 子目录。

```bash
# 顶层一键生成 release 包
# (AIC_CHIP_MODEL 仅影响 platform 侧固件数据选择, host 库与型号无关)
make release CC=arm-xxx-gcc AR=arm-xxx-ar AIC_CHIP_MODEL=D80

# 清理 release 目录
make release-clean
```

`release-clean` 删除 release/ 下所有生成内容 (lib/ include/ platform/ port/ Makefile 及配置文件)。

生成内容（自包含，第三方可直接 `cd release && make`）：

```
release/
├── Makefile                    # 构建入口
├── aic_cfg.mk                  # 功能裁剪开关
├── platform_config.mk          # 平台配置 (第三方编辑)
├── lib/
│   ├── libaic8800_driver.a     # 驱动核心库 (含 WPA, 全型号共用单库)
│   ├── libaic_mbedtls.a        # mbedTLS 加密库 (通用)
│   └── include/                # 驱动公开头 (lib 自包含形态, 与调试 lib/include 一致)
├── include/                    # 驱动公开头文件
│   ├── drv_api/inc/            #   驱动对外 API
│   ├── drv_wifi/               #   WiFi 管理 API
│   └── utils/inc/              #   工具头文件
├── port/inc/                   # 移植接口头文件 (rtos_port.h 等)
└── platform/                   # 平台适配层源码 (第三方实现)
```

**diff 打包前验证（diff-verify）**：

```bash
# 生成自包含 release 包, 并以发布形态 (AIC_BUILD_HOST=0, 直接引用 lib 不经过 host) 独立编译 platform
make diff-verify AIC_CHIP_MODEL=D80
# 1) release 包结构完整性检查: lib / lib/include / port/inc / platform / Makefile
# 2) cd <driver>_release && make AIC_BUILD_HOST=0 独立编译, 产出 build/libaic_platform.a
# 注意: 第 2 步需 platform_config.mk 已适配目标 SDK (EXT_INC_DIRS 等)
```

第三方使用流程：

```bash
# 1. 编辑 platform_config.mk — 配置工具链、头文件路径、源文件
# 2. 编译
cd release && make
# 产出: build/libaic_platform.a
```

客户/平台直接集成时引用 release/ 中的内容:

```makefile
# 头文件引用 (单一事实源 = lib/include/, 无顶层 include/ 副本)
CFLAGS += -Irelease/lib/include/drv_api/inc
CFLAGS += -Irelease/lib/include/drv_wifi
CFLAGS += -Irelease/lib/include/utils/inc
CFLAGS += -Iport/inc

# 链接库 (单库统一, 库名无型号后缀)
LIB_DRIVER  := release/lib/libaic8800_driver.a
LIB_MBEDTLS := release/lib/libaic_mbedtls.a

# 或直接 -l 方式:
LDFLAGS += -Lrelease/lib -laic8800_driver -laic_mbedtls
```

---

## Port 移植层详解

### 平台头文件引用边界（强制规则）

**platform 层（含 `platform/` 全部源码与客户新增平台文件）严禁 `#include` host 内部头文件**。
host 内部头指 `host/driver/` 下除发布清单（`lib/include/` / `include/`）之外的任何头文件，典型违规样例：

```
host/driver/layer_common/inc/*   (sdio_co.h, usb_co.h, hal_co.h ...)
host/driver/drv_macif/*          (rwnx_main.h, rwnx_msg_tx.h ...)
host/driver/drv_fhost/*
host/driver/drv_wifi/*           (wifi_if.h 等发布清单之外的头)
```

平台代码只允许引用两类头：

1. **驱动公开头**：`lib/include/`（调试/发布统一形态，由 `copy_release_headers.py` 清单维护）
2. **移植接口头**：`port/inc/`

> **原因**：调试模式与发布模式（diff 包）头文件集合必须完全一致，否则出现“调试能编译、客户合并后缺头”的 diff 包不完整问题。
> **自检**：修改/新增 platform 层代码后执行 `make diff-verify`，并用
> `grep -rn '#include' platform/ port/` 核对是否引用 host 内部头（违规样例 `sdio_co.h`、`usb_co.h` 等 `layer_common/inc` 头）。

### 设计原则

`port/inc/` 是驱动移植的核心接口层，实现 **host（平台无关驱动核心）** 与 **platform（平台相关实现）** 的解耦：

- **host 编译 (Stage 1)**: port 头文件中的宏将函数调用映射为 `aicwf_*` 前缀的函数名
- **platform 编译 (Stage 2)**: 直接实现这些 `aicwf_*` 函数
- **链接时**: 链接器将 host 层的函数调用直接绑定到 platform 层的实现

这种 **extern 直接链接** 模式无需 ops 结构体或注册函数，架构更简单、性能更好。

### port/inc 头文件清单

```
port/inc/
├── aic_porting_types.h    # 平台无关基础类型: stdint/bool/BIT/NULL
│
│  ── 模块接口头文件 (函数声明 + 类型定义) ──
├── rtos_port.h            # RTOS 函数声明 (任务/信号量/互斥/队列/定时器/事件组)
├── sdio_port.h            # SDIO 函数声明 (CMD52/53/中断/驱动注册)
├── netif_port.h           # 网络接口函数声明 (netif 管理/DHCP/TX/RX buffer)
├── plat_port.h            # HAL/平台函数声明 (电源/GPIO/调试/日志)
├── wifi_port.h            # WiFi 类型定义 (struct aic_wifi_dev)
├── fmac_port.h            # FMAC 平台相关定义
│
│  ── 共享类型/工具 ──
├── aic_netbuf.h           # 网络 buffer 类型: net_buf_tx_t
├── aic_sdio_types.h       # SDIO 类型定义
├── aic_fw_port.h          # 固件 port 接口声明
├── aic_chip_desc.h        # 芯片差异数据表接口 (单库运行时分发, platform 实现数据源)
├── aic_log.h              # 模块化日志宏 (DBG_MOD_SEV)
├── aic_assert.h           # 断言宏
├── aic_cfg.h              # 功能配置宏 (AIC_DISABLE_*)
└── userconfig.h           # 用户配置接口
```

### 类型定义控制

**host 层** (`host/host_config.mk`):
- `PLATFORM_NO_FILE` — 不使用标准文件系统 (RTOS 环境)
- `CONFIG_SOCK_RTOS_AL` — 启用 RTOS socket 适配层

**platform 层** (`platform_config.mk`):

| 宏 | 作用 |
|----|------|
| `SYS_SOCK_DEFINE` | 平台已提供 socket 类型 (lwIP)，跳过 port 层定义 |
| `SYS_NET_DEFINE` | 平台已提供 netif 类型 (lwIP)，跳过 port 层定义 |
| `PLATFORM_NO_FILE` | 平台无 stdio FILE，避免使用文件操作 |

**WPA 类型冲突保护**:

`aic_porting_types.h` 中 `u8`/`u16`/`u32` 等类型定义受 `#ifndef WPA_TYPES_DEFINED` 保护。若平台 SDK 已有同名 typedef 产生冲突，在 `platform_config.mk` 中添加：

```makefile
# 跳过 AIC 类型重定义，使用平台已有类型
#PLATFORM_DFLAGS += -DWPA_TYPES_DEFINED
```

---

## 平台移植指南

### 移植总览

移植到新平台需要实现 **5 个核心模块** 的函数：

```
┌───────────────────────────────────────────────────────────────────┐
│ platform/                                                          │
│                                                                    │
│  platform_init.c ─── 统一入口 (extern 模式下为空)                   │
│                                                                    │
│  核心模块 (必须实现):                                               │
│       ├── rtos/rtos_freertos.c     RTOS 原语 (~60 个函数)          │
│       ├── hal/sdio_port.c          SDIO 总线 (~25 个函数)          │
│       ├── hal/plat_port.c          HAL/硬件抽象 (~17 个函数)       │
│       ├── netif/netif_port.c       网络接口 (~40 个函数)           │
│       ├── fmac/aic_fw_port.c      固件数据 (~12 个函数)           │
│       └── fmac/aic_chip_desc_port.c 芯片差异数据表 (单库必须)      │
│                                                                    │
│  WiFi 业务模块 (顶层 Makefile 自动编译):                             │
│       ├── wifi/wifi_port*.c        WiFi 连接/扫描/AP/WPS/P2P      │
│       └── wifi/wifi_port_cmd.c     AT 命令分发                     │
│                                                                    │
│  可选模块:                                                          │
│       ├── bt/aic_bt_uart.c         蓝牙 UART HCI (AIC_FEATURE_BT) │
│       └── iperf/iperf.c            iperf 测速工具                  │
└───────────────────────────────────────────────────────────────────┘
```

### 各模块说明

#### 1. rtos_freertos.c — RTOS 抽象层 (必须)

实现 `rtos_port.h` 中声明的所有 RTOS 函数。

| 分组 | 典型函数 | 说明 |
|------|---------|------|
| 内存管理 | `aicwf_rtos_malloc`, `aicwf_rtos_calloc`, `aicwf_rtos_free` | 堆操作 |
| 时间 | `aicwf_rtos_now`, `aicwf_rtos_msleep`, `aicwf_rtos_udelay` | 定时/延时 |
| 任务 | `aicwf_rtos_task_create`, `aicwf_rtos_task_delete`, `aicwf_rtos_task_suspend` | 任务管理 |
| 队列 | `aicwf_rtos_queue_create`, `aicwf_rtos_queue_write`, `aicwf_rtos_queue_read` | 消息队列 |
| 信号量 | `aicwf_rtos_semaphore_create`, `aicwf_rtos_semaphore_wait`, `aicwf_rtos_semaphore_signal` | 二值/计数信号量 |
| 互斥锁 | `aicwf_rtos_mutex_create`, `aicwf_rtos_mutex_lock`, `aicwf_rtos_mutex_unlock` | 互斥锁 |
| 定时器 | `aicwf_rtos_timer_create`, `aicwf_rtos_timer_start`, `aicwf_rtos_timer_stop` | 软件定时器 |
| 事件组 | `aicwf_rtos_event_group_create`, `aicwf_rtos_event_group_wait_bits` | 事件标志组 |
| 资源跟踪 | `aicwf_rtos_res_list_push`, `aicwf_rtos_net_mempush` 等 | 调试追踪 (可空实现) |
| 随机数 | `aicwf_random_init`, `aicwf_random_byte` | 随机数生成 |

#### 2. sdio_port.c — SDIO 总线适配 (必须)

实现 `sdio_port.h` 中声明的 SDIO 函数。

| 典型函数 | 说明 |
|---------|------|
| `aicwf_sdio_drv_register` | SDIO 驱动注册 (probe/remove 回调) |
| `aicwf_sdio_readb_cmd52` / `aicwf_sdio_writeb_cmd52` | CMD52 寄存器读写 |
| `aicwf_sdio_readsb` / `aicwf_sdio_writesb` | CMD53 块数据传输 |
| `aicwf_sdio_claim_host` / `aicwf_sdio_release_host` | SDIO host 锁 |
| `aicwf_sdio_host_enable_isr` / `aicwf_sdio_set_irq_handler` | 中断管理 |
| `aicwf_sdio_func_init` | SDIO function 初始化 |

#### 3. plat_port.c — HAL/硬件抽象 (必须)

实现 `plat_port.h` 中声明的平台函数。

| 典型函数 | 说明 |
|---------|------|
| `aicwf_pwr_wifi_pin_init/enable/disable` | WiFi 电源引脚控制 |
| `aicwf_sdio_reset` / `aicwf_sdio_pdn_wifi` | SDIO/WiFi 复位 |
| `aicwf_plat_gpio_trigger` | GPIO 触发 (调试用) |
| `aicwf_dbg_snprintf` / `aicwf_dbg_sprintf` | 格式化输出 |
| `aicwf_dump_buffer` | 十六进制 buffer dump |
| `aicwf_is_in_production_mode` | 产测模式查询 |
| `aicwf_log_printf` | 日志输出 (UART/串口) |
| `aicwf_platform_test_module_severity` | 日志模块/级别过滤 |

#### 4. netif_port.c — 网络接口适配 (必须)

实现 `netif_port.h` 中声明的网络函数。

| 分组 | 典型函数 | 说明 |
|------|---------|------|
| 接口管理 | `aicwf_net_if_add`, `aicwf_net_if_up/down`, `aicwf_net_init/deinit` | netif 生命周期 |
| Buffer 管理 | `aicwf_net_buf_tx_alloc/free`, `aicwf_net_buf_rx_alloc/free` | 收发 buffer |
| 数据通路 | `aicwf_tx_eth_data_process`, `aicwf_rx_eth_data_process` | 以太网帧收发 |
| DHCP | `aicwf_net_dhcp_start/stop/release`, `aicwf_net_if_dhcp_server_start` | DHCP |
| DNS | `aicwf_net_set_dns`, `aicwf_net_get_dns` | DNS 配置 |
| IP 配置 | `aicwf_net_if_set_ip`, `aicwf_net_if_get_ip` | 静态 IP |

#### 5. aic_fw_port.c — 固件数据与加载 (必须)

提供固件二进制数据的获取接口。

固件加载模式 (`CONFIG_LOAD_FW`):
- `0` (filemode) — 从文件系统读取
- `1` (flashmode) — 从 Flash 特定地址读取
- `2` (innermode) — 编译时内嵌为 C 数组 (默认)

#### 6. aic_chip_desc_port.c — 芯片差异数据表 (单库必须)

实现 `port/inc/aic_chip_desc.h` 声明的 `aic_chip_desc_lookup(uint16_t chipid)`，返回各芯片的差异数据表项（总线协议行为/电源管理/5G 支持等，原 host 侧型号宏的唯一数据归宿）。参考实现包含全部芯片条目；产品只用单一型号时可裁剪其余条目，同时新增芯片型号只需在此表新增一项，无需改动 host 源码。运行时链路：总线 probe 阶段读硬件 VID/PID 匹配 chipid → host 调用 `aic_chip_desc_cache_set()` 缓存 → 各层经 `aic_chip_desc_get()` 取用。

### 移植步骤

1. **编译 host 库并生成 release**:
   ```bash
   cd host && make release CC=arm-xxx-gcc AR=arm-xxx-ar
   ```

2. **编辑 platform_config.mk**: 配置工具链、外部头文件路径、平台宏

3. **实现 5 个核心模块**: 参考 `platform/` 中的参考实现，实现各模块函数

4. **在 platform_config.mk 中启用源文件**:
   ```makefile
   SRC_PLATFORM += $(PLATFORM_PATH)/rtos/rtos_freertos.c
   SRC_PLATFORM += $(PLATFORM_PATH)/hal/plat_port.c
   SRC_PLATFORM += $(PLATFORM_PATH)/hal/sdio_port.c
   SRC_PLATFORM += $(PLATFORM_PATH)/netif/netif_port.c
   SRC_PLATFORM += $(PLATFORM_PATH)/fmac/aic_fw_port.c
   SRC_PLATFORM += $(PLATFORM_PATH)/fmac/aic_chip_desc_port.c
   SRC_PLATFORM += $(PLATFORM_PATH)/fmac/userconfig.c
   ```

5. **编译 platform**:
   ```bash
   make
   ```

6. **链接**: 应用程序链接 `libaic_platform.a` + `libaic8800_driver.a`（单库，无型号后缀）+ `libaic_mbedtls.a`

### 关键注意事项

1. **所有声明的函数必须实现**: 驱动内部直接调用 `aicwf_*` 函数，未实现会导致链接错误或运行时崩溃
2. **固件数据**: 需将对应芯片型号的固件 bin 文件链接到程序中或从 Flash 读取
3. **类型定义**: host 层类型通过 `#ifndef` guard 自动处理冲突，platform 层如有冲突可在 `platform_config.mk` 中启用 `SYS_SOCK_DEFINE` / `SYS_NET_DEFINE`
4. **mbedTLS**: crypto 层如平台已有 mbedTLS，在 `platform_config.mk` 设置 `USE_BUILTIN_MBEDTLS=0`（平台自实现 35 个 `aic_mbedtls_*` 符号，参考 `platform/crypto/aic_mbedtls_port_example.c`；TLS port 层用 `AIC_BUILD_MBEDTLS_PORT=0`，两者独立）
5. **wifi_port_plt.c — 驱动启动 Demo + CLI 命令注册**: 
   - `aic_wifi_demo_test()`: 完整的 WiFi 驱动启动流程 Demo（`aic_platform_init()` → 等待 SDIO 就绪 → `wifi_driver_init()` → 等待驱动初始化 → `aic_wifi_init_mac()` → `aicwf_net_if_plat_init()` → 配置 IP/子网掩码 → GPIO 触发）。应用层调用此函数即可完成驱动上电全流程。
   - `cmd_test(const char* temp_uart_buf)`: 将串口输入的 AT/WiFi 命令转发给 `aic_wifi_cmd()`。移植时需注册到平台 CLI/Shell 框架（FreeRTOS+CLI、ThreadX shell 等）。若平台无 CLI 框架，可跳过编译该文件。`wifi_port_cmd.c` 为命令分发表，已自动编译无需额外操作。

### 头文件 Include 路径配置

platform 编译时需要的 `-I` 路径（顶层 Makefile 已配置）：

```makefile
# 驱动内部路径 (已在 Makefile 中配置)
-Iport/inc                          # port 头文件 (rtos_port.h, sdio_port.h 等)
-Irelease/lib/include/drv_api/inc   # netif_def.h, sys_api.h, wifi_api.h 等
-Irelease/lib/include/drv_wifi      # WiFi 管理 API
-Irelease/lib/include/utils/inc     # 工具头文件 (compiler.h, co_math.h 等)
-Iplatform/fmac                     # 固件管理
-Iplatform/hal                      # HAL
-Iplatform/netif                    # 网络接口
-Iplatform/rtos                     # RTOS
-Iplatform/wifi                     # WiFi 业务
-Iplatform/iperf                    # iperf 应用

# 平台 SDK 路径 (通过 EXT_INC_DIRS 在 platform_config.mk 中配置)
-I/sdk/freertos/include             # FreeRTOS 头文件
-I/sdk/lwip/include                 # lwIP 头文件
-I/sdk/sdio/include                 # SDIO HAL 头文件
```

---

## 编译宏定义说明

编译宏通过 `aic_cfg.mk` (功能裁剪) 和 `platform_config.mk` (平台配置) 管理。

### 功能裁剪 (aic_cfg.mk)

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `AIC_FEATURE_P2P` | `1` | 编译 P2P 源文件 |
| `AIC_FEATURE_BT` | `1` | 编译 BT 源文件 |
| `DFLAGS += -DCONFIG_NO_WPA_MSG` | 启用 | 关闭 wpa ctrl_iface 消息 |
| `DFLAGS += -DCONFIG_NO_STDOUT_DEBUG` | 注释 | 关闭 wpa 日志 |

### 芯片型号

**单库统一架构**：host 驱动库已完全型号无关——源码中不含任何 `CONFIG_AIC88xx` / `USE_5G` 型号宏（host 编译时自动从宏集合中剔除），所有型号产出同一个 `libaic8800_driver.a`（已验证：5 个型号编译产物 md5 逐字节一致）。

`AIC_CHIP_MODEL` 变量定义在 `aic_cfg.mk` 中（默认 `D80`），其作用边界变为：

| 作用范围 | 说明 |
|----------|------|
| ✅ platform/SDK 侧编译 | 型号宏（`CONFIG_AIC88xx` / `USE_5G`）仍传给 platform，控制 `aic_fw_port.c` 选择哪份固件数据（MB 级）及 platform 行为开关 |
| ✅ 固件数据选择 | 不同型号固件套件（fmacfw/lmacfw_rf/BT adid/patch 等）由型号宏门控编入镜像 |
| ❌ host 库内容/命名 | 不影响，产物恒为 `libaic8800_driver.a` |

**运行时 chipid 分发链路**（替代编译期型号宏）：

```
总线 probe 读硬件 VID/PID → chipmatch → g_runtime_chipid
→ aic_wifi_open() 存入 rwnx_hw->chipid
→ aic_chip_desc_cache_set(chipid) 查 platform 提供的数据表 (aic_chip_desc_port.c)
→ host 各层经 aic_chip_desc_get() / aicwf_is_5g_enable() 等取用型号差异
```

> **约束**: `AIC_CHIP_MODEL`（决定编哪份固件）必须与实际硬件一致；chipmatch 失败时驱动报错并保持 UNMATCHED，不再回退到编译期假定型号。
> **体积影响**: 全型号常编进 host 的仅逻辑分支与 KB 级小表（patch_tbl/信道表等）；MB 级固件数据仍在 platform 侧按型号门控，单库体积不劣于原分库。

通过 `AIC_CHIP_MODEL` 指定目标型号（仅影响 platform 侧固件选择）：
```bash
make release AIC_CHIP_MODEL=D80   # AIC8800D80
make release AIC_CHIP_MODEL=D80N  # AIC8800D80N
make release AIC_CHIP_MODEL=D80N_U02C  # AIC8800D80N (u02c 子分支)
make release AIC_CHIP_MODEL=D40   # AIC8800D40
```
输出库：统一 `libaic8800_driver.a`（无型号后缀）+ 通用 `libaic_mbedtls.a`

> D80N 分 u02 / u02c 两个子分支（对应芯片 sub_id），通过 `AIC_CHIP_MODEL=D80N`（u02，默认）与 `AIC_CHIP_MODEL=D80N_U02C` 选择，仅固件套件（BT adid/patch/table 等）不同，host 库与运行时行为完全相同。主 FMAC 复用 D80（与 Linux D80N 的 `wl_fw=fmacfw_8800d80.bin` 一致）。

### 总线与固件
| 宏定义 | 说明 |
|--------|------|
| `CONFIG_SDIO_SUPPORT` | 启用 SDIO 总线支持 |
| `CONFIG_DOWNLOAD_FW` | 启用固件下载功能 |
| `CONFIG_LOAD_FW=N` | 固件加载模式: 0=file, 1=flash, 2=inner(默认) |

### WiFi 功能
| 宏定义 | 说明 |
|--------|------|
| `CONFIG_RWNX_FULLMAC` | FullMAC 模式 |
| `CONFIG_REORD_FORWARD_LIST` | 数据包重排序转发列表 |
| `CONFIG_LWIP` | 使用 LwIP 协议栈 |

### 硬件设置
| 宏定义 | 说明 |
|--------|------|
| `CONFIG_PMIC_SETTING` | PMIC 电源管理 |
| `CONFIG_LOAD_USERCONFIG` | 加载用户配置 |
| `CONFIG_DPD` | 数字预失真 |
| `CONFIG_SDIO_BUS_PWRCTRL` | SDIO 总线电源控制 |

### 软件设置
| 宏定义 | 说明 |
|--------|------|
| `CONFIG_MBEDTLS` / `CONFIG_MBEDTLS_V3` | mbedTLS 加密库 |
| `SDIO_FC_NARB` | SDIO 流控仲裁 |
| `CONFIG_SOCK_SENDTO` | Socket sendto 优化 |
| `CONFIG_RTOS_TRACE` | RTOS 资源跟踪 |
| `CONFIG_SOCK_RTOS_AL` | Socket RTOS 抽象层 |
| `CONFIG_DRIVER_ORM` | 驱动 ORM 模式 |

### 调试
| 宏定义 | 说明 |
|--------|------|
| `AIC_LOG_DEBUG_ON` | 开启调试日志 |
| `CONFIG_NO_STDOUT_DEBUG` | 关闭 wpa 日志 |
| `CONFIG_NO_WPA_MSG` | 关闭 wpa ctrl_iface 消息 |
| `PLATFORM_NO_FILE` | 平台无 stdio FILE |

### 代码空间优化

`platform_config.mk` 中提供编译器优化和源级裁剪选项，帮助控制最终固件体积。

**编译器优化标志** (在 `CFLAGS_ARCH` 或 `CFLAGS` 中配置):

| 标志 | 作用 |
|------|------|
| `-Os` / `-Oz` | 体积优先优化 |
| `-ffunction-sections` | 每个函数放入独立段，配合 GC |
| `-fdata-sections` | 每个数据变量放入独立段 |
| `-Wl,--gc-sections` | 链接时移除未引用的段 |
| `--specs=nano.specs` | 使用 newlib-nano 精简 C 库 |

**源级裁剪开关** (在 `aic_cfg.mk` 和 `platform_config.mk` 中配置):

| 配置 | 作用 |
|------|------|
| `AIC_LOG_DEBUG_ON` 注释 | 关闭模块调试日志 |
| `CONFIG_NO_WPA_MSG` | 关闭 wpa ctrl_iface 消息 |
| `CONFIG_NO_STDOUT_DEBUG` | 关闭 wpa 标准输出日志 |
| `AIC_FEATURE_P2P = 0` | 关闭 P2P 功能源码编译 |
| `AIC_FEATURE_BT = 0` | 关闭蓝牙功能源码编译 |
| `AIC_BUILD_MBEDTLS_PORT = 0` | 不编译 mbedTLS port 层 |

**mbedTLS 模块裁剪** (修改 `mbedtls_v3_6_2/config/config_aic_default.h`):

按需求启用/禁用加解密模块（AES/SHA/RSA/ECC 等），从源头减少 `.o` 文件数量。

### 平台特定宏 (通过 PLATFORM_DFLAGS 传入)
| 宏定义 | 说明 |
|--------|------|
| `PLATFORM_THREADX` | ThreadX 平台示例 (替换为目标平台宏) |

---

## mbedTLS 外部接口层

mbedTLS 提供两层可替换接口：

### Crypto 加密原语层

wpa_supplicant 通过 `crypto_mbedtls.c` 调用的 MPI/ECP 加密原语。

| 项目 | 内容 |
|------|------|
| 接口头文件 | `mbedtls_v3_6_2/include/aic_mbedtls_crypto_if.h` |
| 函数数量 | 35 个（22 MPI + 13 ECP） |
| 内置实现 | `aic_mbedtls_crypto/aic_mbedtls_mpi.c` + `aic_mbedtls_ecp.c` → `libaic_mbedtls.a` |
| 外部实现示例 | `platform/crypto/aic_mbedtls_port_example.c`（去 `aic_` 前缀直接转发标准 mbedtls） |
| 编译控制 | `USE_BUILTIN_MBEDTLS=1`(默认) / `USE_BUILTIN_MBEDTLS=0`(外部实现) |

### Port 移植层（TLS 客户端）

用于 TLS over TCP 场景（如 MQTT over TLS），独立于 crypto 层。

| 项目 | 内容 |
|------|------|
| 接口头文件 | `mbedtls_v3_6_2/port/include/mbedtls_port_if.h` |
| 必须实现 | 硬件熵源、TLS 证书数据 |
| 编译控制 | `AIC_BUILD_MBEDTLS_PORT=1`(默认) / `AIC_BUILD_MBEDTLS_PORT=0`(外部实现) |

---

## 日志框架

日志系统分为两层：

```
┌─────────────────────────────────────────────────────────────────┐
│  应用/驱动代码                                                    │
│  使用: DBG_SDIO_ERR("fail..."), DBG_FHOST_INF("scan done")      │
└─────────────────────┬───────────────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────────────┐
│  port/inc/aic_log.h (模块接口层)                                  │
│  - 12 个模块 x 5 个级别 = 60 个 DBG_MOD_SEV 宏                   │
│  - platform_test_module_severity() 运行时过滤                     │
│  - 模块: CMN, OSAL, NETAL, XHCI, SDIO, USB, MACIF,              │
│          RDATA, VNET, FMAC, FHOST, APP                           │
│  - 级别: CRT(1), ERR(2), WRN(3), INF(4), VRB(5)                 │
└─────────────────────┬───────────────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────────────┐
│  port/inc/plat_port.h (平台输出层)                                │
│  - AIC_LOG_PRINTF -> aicwf_log_printf()                          │
│  - 受 AIC_LOG_DEBUG_ON / CONFIG_LOG_LEVEL / CONFIG_LOG_MODULES    │
│    编译开关控制                                                    │
└─────────────────────────────────────────────────────────────────┘
```

运行时通过 `dbg_env` 结构体控制过滤:
- `filter_module`: 位掩码，每个 bit 对应一个模块的开关
- `filter_severity`: 阈值，仅输出 <= 该级别的日志

---

## CMake 构建 (辅助)

项目同时保留 CMake 构建文件，主要用于 IDE 集成和 SDK 子项目模式。

### host/CMakeLists.txt

独立验证构建:

```bash
cd host && mkdir -p cmake-build && cd cmake-build
cmake .. && make
# mbedTLS crypto 层始终编译 (libaic_mbedtls.a); TLS port 层可关闭:
cmake .. -DAIC_BUILD_MBEDTLS_PORT=OFF && make
```

### 根 CMakeLists.txt (SDK 集成)

驱动根目录 `CMakeLists.txt` 与 Makefile 架构一致（两阶段、host 闭源），通过 `AIC_USE_PREBUILT_HOST` 切换：

- **引用模式（默认，`host/` 不存在时自动）**：只编译 platform 源码（`PACKAGE_SRC_OPEN_FILES`），
  host 引用 `lib/` 预编译库（`libaic8800_driver.a` + `libaic_mbedtls.a`，单库统一，无型号后缀），
  头文件用 `lib/include/` + `port/inc` —— 等价于 Makefile `AIC_BUILD_HOST=0`
- **全源码模式（`host/` 存在时自动，开发调试）**：编译 host 驱动 + WPA + platform 全部源码 —— 等价于 `AIC_BUILD_HOST=1`
- `AIC_CHIP_MODEL` 仅控制 platform 侧固件数据选择（host 库型号无关，变量保留仅为兼容）
- `cmake --build . --target release` 生成 `<驱动目录名>_release/` 自包含发布包
  （lib/ + lib/include/ + include/ + platform/ + port/inc/ + Makefile + CMakeLists.txt + 配置）

**CMake 侧开关**（与 Makefile 同名同义，默认值对齐 `aic_cfg.mk` / `host_config.mk`）：

| CMake 变量 | 默认 | 说明 | Makefile 对应 |
|---|---|---|---|
| `AIC_USE_PREBUILT_HOST` | 自动 | `ON`=引用 `lib/` 预编译库；`OFF`=全源码编译 | `AIC_BUILD_HOST` |
| `AIC_FEATURE_BT` | `0` | 平台 BT 层（`platform/bt/`：`aic_hci.c` + 传输后端）+ `AIC_FEATURE_BT` 宏 | `AIC_FEATURE_BT` |
| `AIC_HCI_TRANSPORT` | `SDIO` | BT HCI 传输：`SDIO`/`UART`/`USB`，恒注入 `AIC_HCI_OVER_*` | `AIC_HCI_TRANSPORT` |
| `AIC_HOST_BT_DRIVER` | `1` | host BT 驱动（`drv_bt` + `drv_api/bt_api.c`）+ `AIC_HOST_BT_DRIVER` 宏 | `AIC_HOST_BT_DRIVER` |
| `AIC_FEATURE_P2P` | `1` | P2P 宏（恒注入，含引用模式）+ 全源码模式的 wpa p2p 源 | `AIC_FEATURE_P2P` |
| `AIC_FEATURE_PLT` | `0` | 平台启动/demo 框架 `platform/wifi/wifi_port_plt.c` | `AIC_FEATURE_PLT` |
| `AIC_FEATURE_NIMBLE` | `OFF` | NimBLE host 栈（需同时 `AIC_FEATURE_BT=ON`） | `AIC_FEATURE_NIMBLE` |
| `AIC_BUILD_MBEDTLS_PORT` | `1` | 全源码模式编入 host mbedTLS port 层（TLS 客户端/证书/硬件熵源） | `AIC_BUILD_MBEDTLS_PORT` |
| `USE_BUILTIN_MBEDTLS` | `1` | `1`=链接内置 `libaic_mbedtls.a`；`0`=平台提供 `aic_mbedtls_*` 符号 | `USE_BUILTIN_MBEDTLS` |
| `AIC_MBEDTLS_FWD_EXAMPLE` | `1` | `USE_BUILTIN_MBEDTLS=0` 时编入转发示例 `platform/crypto/aic_mbedtls_port_example.c` | `AIC_MBEDTLS_FWD_EXAMPLE` |
| `AIC_RTOS_PORT_SRC` | `rtos_freertos.c` | `platform/rtos/` 下的 RTOS 端口文件名 | `AIC_RTOS` |
| `AIC_CHIP_MODEL` | `D80` | 仅 platform 侧固件数据选择（host 库型号无关） | `AIC_CHIP_MODEL` |

> **总线范围**：本 CMake 清单按 SDIO 固定（恒注入 `CONFIG_SDIO_SUPPORT`，不含 `platform/hal/usb_port.c`）；USB 总线平台需自行调整清单。
> **平台保留文件**：`platform/hal/plat_port.c` 在 CMake 路径恒编基准参考实现（Make 路径要求平台自行 `SRC_PLATFORM +=` 追加，见 §流程）。平台若已自带实现，请在集成 CMake 里移除该行，避免重复符号。

> **注意**：host 预编译库必须先由 make 体系产出（`make release` 或顶层 `make` 的 lib-sync），
> CMake 集成模式不编译 host 源码（IP 保护，与 Makefile 发布模式一致）。
