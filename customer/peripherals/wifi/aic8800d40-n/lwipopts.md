# lwipopts.h 平台配置要求 —— 吞吐优化与零拷贝功能配套说明

> **适用对象**: 将 AIC8800 基准驱动移植到新平台的工程师 / AI 辅助移植工具
>
> **背景**: 基准驱动已完成以下三项功能的驱动侧实现（代码均在驱动包内），但它们依赖
> 平台 lwip 的 `lwipopts.h` 配置。**基准驱动不含 lwip**（lwip 属于平台 SDK），
> 因此本文档列出必须在目标平台 `lwipopts.h` 中落实的配置项，供移植时对照填写。

---

## 1. 涉及功能与驱动侧代码位置（均已包含在基准驱动中）

| 功能 | 驱动侧代码 | lwip 依赖 |
|------|-----------|-----------|
| TX 吞吐优化 | `host/driver/drv_fhost/fhost_tx.c`（发送调度批量化）、`platform/iperf/iperf.c` | §2 TCP/内存/队列参数 |
| TX 零拷贝 (NET_IF_UAP_NCPYBUF) | `platform/netif/netif_port.c`、`host/driver/drv_fhost/fhost_tx.c` | §3 `PBUF_LINK_ENCAPSULATION_HLEN`（强制） |
| RX core-lock 直连入栈 | `platform/netif/netif_port.c` | §4 `LWIP_TCPIP_CORE_LOCKING`（强制） |
| RX 零拷贝 (CONFIG_RX_NOCOPY) | `platform/netif/netif_port.c`（custom pbuf 消费分支）、`host/driver/drv_fhost/fhost_rx.c` | §5 `LWIP_SUPPORT_CUSTOM_PBUF`（强制，默认推导式为 0 必须显式定义） |

iperf 本身无需 lwip 特殊配置；其历史缺陷（UDP 服务端 `SO_RCVTIMEO` 须传
`struct timeval`、UDP 单报文 1470B、限速除零保护等）已在基准驱动修复，无需平台处理。
iperf traffic 任务优先级已回流基准对齐为 `TCPIP_PRIORITY`（与 tcpip 线程持平，
基准 `b2ee157`），平台若另行调低会导致排空缓冲/归还窗口滞后、吞吐明显下降。

---

## 2. TX/RX 吞吐优化参数（iperf 实测调优结果）

### 2.1 TCP 窗口与缓冲（核心）

| 宏 | 推荐值 | lwip 默认 | 说明 |
|----|--------|-----------|------|
| `TCP_MSS` | `1460` | 536 | **必须** = MTU(1500)−IP头(20)−TCP头(20)。**勿设 1500**：每段超 MTU 被 IP 强制分片，任一片丢失整段重传，WiFi 空口大帧错误率高，吞吐反降 |
| `TCP_SND_BUF` | `(64 * 1460)` ≈92KB | 2920 | 覆盖 WiFi RTT×带宽积，避免应用写两口就等 Ack；从 `MEM_SIZE` 堆分配 |
| `TCP_SND_QUEUELEN` | `(4 * TCP_SND_BUF/TCP_MSS)` | 2*SND_BUF/MSS | 发送队列深度，≥ 2*SND_BUF/MSS 是 lwip 硬约束 |
| `TCP_SNDLOWAT` | `(TCP_SND_BUF/4)` | SND_BUF/2 | select 可写阈值。**勿设 SND_BUF-1**（缓冲全空才唤醒，发送锯齿）；BUF/2 与 iperf 46KB 大块发送组合会把流水线对半切割，BUF/4 让发送端连续填充 |
| `TCP_WND` | `(32 * TCP_MSS)` ≈47KB | 2920 | RX 方向带宽积。本端通告窗口 47KB 自身不做缩放，但 `LWIP_WND_SCALE`/`TCP_RCV_SCALE` 必须按下两行设置 |
| `LWIP_WND_SCALE` | `1` | 0 | **强制保持 1**：除控制本端窗口通告外，还决定 lwip 是否解析对端的窗口 scale 选项；关闭后对端启用 scale 时窗口解析错误（按未移位处理），吞吐失真 |
| `TCP_RCV_SCALE` | `0` | 0 | **勿设非 0**：通告窗口 = `TCP_WND >> RCV_SCALE`，设 6 时 ≈746B < 1 MSS，对端发送受限锁死 RX 吞吐（G32S10X 实测教训） |
| `TCP_QUEUE_OOSEQ` | `1` | 1 | WiFi 空口乱序常见，保持开启 |
| `TCP_OOSEQ_MAX_PBUFS` | `4` | 无限 | 乱序排队 pbuf 上限，防止乱序风暴吃光 pbuf 池 |

### 2.2 内存池

| 宏 | 推荐值 | lwip 默认 | 说明 |
|----|--------|-----------|------|
| `MEM_SIZE` | `(128 * 1024)` | 1600 | 约束：`MEM_SIZE >= TCP_SND_BUF + TX零拷贝余量`；nocopy TX 下 pbuf 被驱动持有至 SDIO 聚合拷贝完成，与 TCP 重传队列并发持有 |
| `MEMP_NUM_PBUF` | `512` | 16 | nocopy TX 下发送 pbuf 被驱动引用至 SDIO 聚合完成，高速发送时引用型 pbuf 峰值高 |
| `MEMP_NUM_TCP_SEG` | `512` | 16 | TCP_SND_BUF=64*MSS 最多 64 段在飞，叠加多连接与重传积压；约束：≥ `TCP_SND_QUEUELEN` |
| `MEMP_NUM_NETCONN` | `64` | 4 | 多连接并发（iperf 多实例等）余量 |
| `MEMP_NUM_TCPIP_MSG_INPKT` | `64` | 8 | RX 已改 core-lock 直连入栈（不走 tcpip 消息），INPKT 仅残留路径使用 |
| `PBUF_POOL_SIZE` | `512` | 16 | RX 高速收帧缓冲池；512×~1540B ≈ 800KB 静态 RAM（G32S10X 当前值，配合 RX 零拷贝头池），RAM 紧张可降回 256/120 |
| `PBUF_POOL_BUFSIZE` | `LWIP_MEM_ALIGN_SIZE(TCP_MSS+40+PBUF_LINK_HLEN)` | 同公式 | 公式自动跟随 TCP_MSS 跟涨（~1540B/块），保持公式形式即可 |

### 2.3 消息队列与行为开关

| 宏 | 当前值（G32S10X 实测） | lwip 默认 | 说明 |
|----|--------|-----------|------|
| `DEFAULT_RAW_RECVMBOX_SIZE` | `64` | 8 | RX 直连入栈后 tcpip mbox 只剩 API 消息，8 太浅 |
| `DEFAULT_UDP_RECVMBOX_SIZE` / `DEFAULT_TCP_RECVMBOX_SIZE` | `128` | 8 | socket recv 队列深度，高吞吐多帧待消费 |
| `DEFAULT_ACCEPTMBOX_SIZE` | `64` | 8 | accept 队列 |
| `TCPIP_MBOX_SIZE` | `128` | 8 | tcpip 线程消息队列；64~128 足够，无需 512 |
| `LWIP_DISABLE_TCP_SANITY_CHECKS` | `1` | 0 | 关闭 TCP 参数冗余健全性检查（参数间约束由配置时人工校验） |
| `LWIP_NETCONN_SEM_PER_THREAD` | `0` | 1 | netconn/socket 每线程共用一个信号量，节省每线程信号量资源 |
| `LWIP_TCPIP_CORE_LOCKING` | `1` | 1 | **强制保持**：与 RX 直连入栈（netif_port.c 直接持 core lock 调 tcp_input）配套 |
| `LWIP_NETIF_TX_SINGLE_PBUF` | `1` | 0 | 发送单 pbuf 模式；**副作用**：使 `LWIP_SUPPORT_CUSTOM_PBUF` 默认推导式变 0，需 §5 显式定义 |

### 2.4 lwip 线程优先级（平台相关，按平台线程框架折算）

`TCPIP_THREAD_PRIO` 的绝对值取决于平台优先级体系。G32S10X 当前值：**3**（平台五档制
的 high 档，枚举值直传 FreeRTOS 无缩放；原 lwip 参考值 128 越界被钳位到最高档 31，
曾改为 normal 档后又上调至 3）。驱动侧 `plat_port.h` 的 `TCPIP_PRIORITY` 同步为 3，
iperf traffic 任务 = `TCPIP_PRIORITY`（持平）、SDIO 任务 = `TCPIP_PRIORITY+2`。
**移植要点：tcpip 线程不应高于 WiFi 收发任务之外的关键实时任务，也不应低于
发送生产端；`LWIP_TCPIP_CORE_LOCKING=1` 下 socket 应用在调用者任务同步执行，
其任务优先级应与 tcpip 线程持平或更高，否则排空缓冲/归还窗口滞后压吞吐。**

---

## 3. TX 零拷贝 (NET_IF_UAP_NCPYBUF) —— 强制项

| 宏 | 值 | 说明 |
|----|----|------|
| `PBUF_LINK_ENCAPSULATION_HLEN` | `28` | **强制**。fhost_tx 在 payload 前 `offsetof(struct hostdesc, cfm_cb)`=28B 处写入 hostdesc 描述符，该空间必须落在 pbuf 分配区内的 headroom。定义后所有层级 pbuf 分配自动计入此预留。**缺失将导致 hostdesc 写越界、TX 崩溃** |

---

## 4. RX core-lock 直连入栈 —— 强制项

| 宏 | 值 | 说明 |
|----|----|------|
| `LWIP_TCPIP_CORE_LOCKING` | `1` | **强制**。netif_port.c 收帧后直接 `LOCK_TCPIP_CORE()` 调 lwip 输入，不再经 tcpip mbox 投递，降低 RX 时延 |

---

## 5. RX 零拷贝 (CONFIG_RX_NOCOPY + custom pbuf) —— 强制项

| 宏 | 值 | 说明 |
|----|----|------|
| `LWIP_SUPPORT_CUSTOM_PBUF` | `1`（**必须显式定义**） | 激活 netif_port.c 完整零拷贝实现（driver RX 帧直挂 custom pbuf，免一次 memcpy）。**默认推导式陷阱**：lwip 默认 `(IP_FRAG && !LWIP_NETIF_TX_SINGLE_PBUF) \|\| (LWIP_IPV6 && LWIP_IPV6_FRAG)`，本平台 `LWIP_NETIF_TX_SINGLE_PBUF=1` 且未开 IPv6 → 推导值 0，不显式定义则静默降级 copy 路径 |

**配套约束（缺一即回归）**：

1. 驱动侧 `CONFIG_RX_NOCOPY` 须编入（`aic_cfg.mk` 的 `AIC_RX_NOCOPY ?= 1` 默认开），
   host 库与 platform 层共用该开关；关闭时 SDIO RX 池 40，开启时 64（sdio_co.h）
2. **生产端与消费端必须同版本**：`fhost_rx.c`（建 custom pbuf + `hal_buf_ref`）与
   `netif_port.c` 的 `aicwf_rx_eth_data_process`（消费分支）必须都是基准版；
   若消费端是旧版 5 参壳（恒走 copy），每帧头池泄漏 + node 引用永不释放 →
   SDIO RX 池数十帧内耗尽 → TCP 无法建连（G32S10X 实测回归教训）
3. 链路自管理：init 懒自初始化（首帧触发），deinit 由 `sdio_co.c` 关链自动调用，
   hook 返 NULL 时驱动侧有 copy 兑底，平台无需额外初始化点
4. 回退手段：删除本宏（改回 0）重编即回 copy 路径，驱动侧自动降级无感

---

## 6. 参数约束清单（配置后自检）

1. `MEM_SIZE >= TCP_SND_BUF`（+ TX 零拷贝余量，推荐 128KB 对 92KB）
2. `TCP_SND_QUEUELEN >= 2 * TCP_SND_BUF / TCP_MSS`
3. `MEMP_NUM_TCP_SEG >= TCP_SND_QUEUELEN`
4. `TCP_MSS = MTU - 40`，勿等于 MTU
5. `LWIP_WND_SCALE` 保持 `1`（解析对端 scale 选项必需），`TCP_RCV_SCALE` 保持 `0`（勿移位通告窗口）
6. `TCP_SNDLOWAT` 介于 `SND_BUF/4 ~ SND_BUF/2`，勿设 `SND_BUF-1`
7. TX 零拷贝启用时 `PBUF_LINK_ENCAPSULATION_HLEN >= 28`
8. RX 直连入栈启用时 `LWIP_TCPIP_CORE_LOCKING = 1`
9. RX 零拷贝启用时 `LWIP_SUPPORT_CUSTOM_PBUF` 必须**显式** `= 1`（默认推导式因
   `LWIP_NETIF_TX_SINGLE_PBUF=1` 变 0），且驱动/平台消费端代码与基准同版本（§5）
10. `CHECKSUM_BY_HARDWARE` **未定义**（AIC8800 无 TX 校验和 offload，开了两头都不算，DHCP 零 OFFER，§8.1）
11. `LWIP_DONT_PROVIDE_BYTEORDER_FUNCTIONS = 1`（防 htons 宏化破坏 wpa 预编译库符号垫片，链接失败，§8.2）

## 7. RAM 预算参考（G32S10X 实测值）

| 项 | 占用 |
|----|------|
| `MEM_SIZE` 堆 | 128KB |
| `PBUF_POOL_SIZE(512)` × `PBUF_POOL_BUFSIZE(~1540B)` | ≈800KB 静态 RAM |
| RX 零拷贝头池/节点池（`netif_port.c`/`sdio_co.h`，开启时 node 池 64） | 数十 KB |
| 其余 MEMP 池增量（相对默认） | ≈数十 KB |

RAM 紧张平台优先收缩 `PBUF_POOL_SIZE`（512→256/120）与 `MEM_SIZE`（128KB→96KB 且
同步下调 `TCP_SND_BUF`），TCP 窗口参数按带宽积等比收缩。

---

## 8. 校验和与符号兼容（强制项，CCM4211 实测教训）

### 8.1 `CHECKSUM_BY_HARDWARE` —— 禁止定义

| 宏 | 值 | 说明 |
|----|----|------|
| `CHECKSUM_BY_HARDWARE` | **不定义**（注释掉） | **禁止**。AIC8800 固件无 TX IP/UDP/TCP 校验和 offload（netif flags 亦未挂 checksum 能力位）。平台 lwipopts.h 定义它会使 `CHECKSUM_GEN_IP` 等推导为 0 → lwIP 与硬件两头都不算校验和：STA 发出的 DHCP DISCOVER IP 头校验和恒为 0x0000 非法值，AP 侧丢包，零 OFFER（DHCP 建网失败但 EAPOL/ARP 等非 IP 帧握手正常，极具迷惑性）。CCM4211 空口抓包实测定位 |

### 8.2 `LWIP_DONT_PROVIDE_BYTEORDER_FUNCTIONS` —— 必须定义

| 宏 | 值 | 说明 |
|----|----|------|
| `LWIP_DONT_PROVIDE_BYTEORDER_FUNCTIONS` | `1`（**必须显式定义**） | 禁止 lwIP 提供 htons/htonl 兼容宏。lwip `def.h` 的 `#define htons(x) lwip_htons(x)` 会把驱动 platform 层的 `htons()` 包装函数（wpa 预编译库 `U htons` 符号垫片）整体改名为 `lwip_htons`，造成 `lwip_htons` 多重定义 + `htons` 符号缺失，链接失败（CCM4211 firmware.elf 实测）。与 `platform` 层 htons 垫片机制配套（见 project 记忆 htons 分层结论） |
