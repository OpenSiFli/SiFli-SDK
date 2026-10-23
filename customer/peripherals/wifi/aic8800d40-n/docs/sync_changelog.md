# AIC8800 平台驱动基准同步变更记录 (SiFli SF32LB52x)

> 双段式变更记录（依据 `docs/aic8800-sync/SKILL.md` §7）。
> **段一** = 基准驱动同步（与平台无关的变更，来自 `git diff` 结果）；**段二** = 平台特殊更新（平台适配相关，保留 `[PLATFORM]`/`[AIC8800-PORT]` 标注代码）。
>
> - 平台：SiFli SF32LB52x SDK（RT-Thread 3.1.3 + lwIP 2.1.2 + scons/Kconfig），芯片 AIC8800D80，SDIO 总线，Cortex-M33。
> - 集成：host 核心库用驱动 Makefile stage-1 预编译（`platform_config_cortex-m33.mk`）成 `lib/libaic8800_driver.a` + `libaic_mbedtls.a`；**platform 适配层由 SDK 的 SCons 编译**（`SConscript` DefineGroup，链入 `example/aic8800_d80/project`）。
> - 未移植：BT（`AIC_FEATURE_BT=0`）、nimble（`AIC_FEATURE_NIMBLE=0`）。

---

## 同步 #1

### 日期: 2026-09-15
### 基准版本: `4c6e0d8` → `d4509b7f99235130ffc525aaf63f28caa3f047bf` (origin/master)
### 基准仓库: /home/admin/project/aic8800_rtos

基准自移植锚点新增 4 个 commit：

| commit | 说明 |
|--------|------|
| `4695bc3` | docs(skills): 补全 NimBLE 规则 — 独立编译禁并入单库 + 源码形式发布 |
| `1d16913` | **fix(wifi): P2P 编译开关安全化 — 固定 ABI 布局 + 补齐条件守卫** |
| `87dd7d5` | fix(bt): 补全 aic_hci 停机残留 + 恢复 aic_bt_fw_reset 实现 |
| `d4509b7` | feat(platform): aic_wifi_demo_test 增加 pwrctl 开关与 FW 日志级别 demo 配置 |

`git diff 4c6e0d8..d4509b7 --stat` = 8 文件（+76 -5）。

#### 段一: 基准驱动同步（与平台无关的变更）

| 文件 | 层 | 变更 | 同步方式 | 说明 |
|------|----|------|---------|------|
| `host/driver/drv_api/inc/wifi_def.h` | host | +7-2 | 全量覆盖 | `1d16913`: `aic_wifi_event_data` 去掉 `#ifdef CONFIG_P2P_SUPPORT`，两个 P2P 字段无条件保留，**sizeof 恒 72**（修复前 P2P=0 为 64，错位 8 字节） |
| `host/driver/drv_fhost/fhost_wpa.c` | host | +7-1 | 全量覆盖 | `1d16913`: `p2p_no_group_iface` 引用加 `#ifdef CONFIG_P2P_SUPPORT` 守卫（该成员受 config.h `#ifdef CONFIG_P2P` 保护） |
| `host/thirdparty/wpa_supplicant/wpa_supplicant/wpa_supplicant.c` | host | +6 | 全量覆盖 | `1d16913`: `wpa_supplicant_deinit_iface` 内 P2P group 清理逻辑加 `#ifdef CONFIG_P2P` 守卫 |
| `host/driver/drv_api/src/bt_api.c` | host | +7 | 全量覆盖 | `87dd7d5`: BT 相关（host 层无例外全量覆盖，编译期由 `AIC_FEATURE_BT=0` 门控是否参与） |

> host 层覆盖后 `diff -rq host/ <平台>/host/`（排除 build 产物）验证：**平台 host/ == 基准 host/，逐文件一致**。

#### 段二: 平台特殊更新（平台适配相关）

| 文件 | 变更 | 同步方式 | 说明 |
|------|------|---------|------|
| `platform/wifi/wifi_port_plt.c` | 基准 +16 | 手动合并（§3.2） | 合入基准 `d4509b7` 新增：`#include "sys_api.h"` + `[DEMO]` pwrctl 块（`aic_pwrctl_enable_set(0)`，`#if CONFIG_SDIO_BUS_PWRCTRL\|\|CONFIG_USB_SUPPORT`）+ `[DEMO]` FW 日志块（`aic_wifi_fw_log_enable(AIC_FW_MOD_DEFAULT, AIC_FW_SEV_INF)`）；**保留** 平台 5 处 `[AIC8800-PORT][SiFli]` 适配（`wifi_if.h`、`<rtthread.h>`、`<drivers/mmcsd_core.h>`+`sdio_port.h`、3 个 extern 声明、`mmcsd_wait_cd_changed(RT_WAITING_FOREVER)`）。合并后 vs 基准差异只剩平台标注代码 ✓ |
| `platform/bt/aic_hci.c` | 基准 +19 | **跳过** | `87dd7d5` BT 停机残留修复；BT 未移植（`AIC_FEATURE_BT=0`），`platform/bt/` 未纳入 SCons 编译，同步无意义 |
| `port/inc/fmac_port.h` | — | **保留平台版** | 基准本次未改 port 层。平台版含 `[AIC8800-PORT][SiFli]` 追加声明 `int aicwf_rftest_fw_available(void);`（GCC14 隐式声明升 error 的修复），仅新增声明、不改结构体/宏 → **无 ABI 影响**；按 §8.2.1「[AIC8800-PORT] 标注正向同步不得剥离」保留 |
| `SConscript` + `platform_config_cortex-m33.mk` | P2P 1→0 | **配置回退** | `1d16913` 修复根因后，P2P **回归已审批 SPEC 的 =0**：SConscript 去掉 `wifi_port_p2p.c` src（15→14 文件）+ 去掉 `AIC_FEATURE_P2P`/`CONFIG_P2P_SUPPORT` 两宏；`platform_config_cortex-m33.mk` `AIC_FEATURE_P2P:=1→0`。host/platform 同步编 P2P=0，`wifi_def.h` sizeof 恒 72 保证 ABI 一致 |
| `docs/` (扁平镜像) | 全量镜像 | 覆盖 | 用户选择「全量镜像」：基准 docs(d4509b7) 镜像到 **`docs/` 扁平结构**（skills + 用户文档 1~5 + PDF），`diff -rq` 与基准 0 差异（除平台 .baseline_anchor/sync_changelog.md）。**注**：初版误用 `docs/baseline_reference/` 嵌套，违背 porting skill 4.2「docs 保持不变(扁平)」，已扁平化修正（见文末「结构修正 #1」） |

### 影响评估
- **功能完整性**：`aic_wifi_demo_test()` 新增 pwrctl 运行时开关（demo 默认 `aic_pwrctl_enable_set(0)` 关总线休眠便于调试）+ FW 日志级别配置（全模块 ≥INF）；WiFi STA/AP/scan 核心逻辑不变。
- **稳定性**：host 变更均为 P2P 条件守卫补齐 + BT 停机修复，无破坏性；P2P=0 减少 `wifi_port_p2p.c` 编译面。
- **兼容性**：`aic_wifi_event_data` sizeof 恒 72，**彻底消除** host/platform P2P 配置不一致导致的静默 ABI 错位隐患（上轮 P2P=1 workaround 的根因已被基准官方修复）。

### 验证结果
- [x] host 编译通过（`make host-clean` + `host-build`，P2P=0，cortex-m33，RC=0，162 objects：Driver 39 + WPA 120 + mbedTLS 3）
- [x] host 库 arch 对齐（`readelf -A`：`Tag_CPU_arch=v8-M.mainline` / `profile=Microcontroller` / `CPU_name=8-M.MAIN` / `VFP_args=VFP registers`）
- [x] platform（SCons）编译链接通过（`scons --board=eh-lb525 -j8`，RC=0，**0 error / 0 undefined / 0 conflicting CPU arch**）
- [x] 产物生成（`main.elf` 9.5M、`main.bin` 1.36M）
- [x] P2P=0 符号验证（`app_wlan_p2p_start` 已消失；`aic_pwrctl_enable_set`/`aic_wifi_fw_log_enable`/`aic_wifi_demo_test` 均链入；`htons`=W 与 `lwip_htons`=T 共存）
- [ ] 功能测试（需真实硬件：SDIO 卡就绪 + WiFi 扫描/连接；`aic_wifi_demo_test()` 阻塞等卡）
- [x] release 包重新生成 → 见阶段3 diff 包（`docs/aic8800-diff-package/SKILL.md`）

---

## 结构修正 #1（docs 扁平化回归 porting skill）

### 日期: 2026-09-15（同步 #1 后续）

**触发**：排查「diff 包缺失用户文档」→ 定位到 docs 结构偏离。

**根因链**：初版阶段1 自创 `docs/baseline_reference/` 嵌套子目录镜像基准 docs，违背 `docs/aic8800-porting/SKILL.md` 4.2 节「`docs/` ← Skill 文件（保持不变）」的**扁平结构**规定 → `make dist` 的清理规则 `rm -rf docs/aic8800-*`（正确假设扁平）匹配不到嵌套层里的 skills，自动清理失效 → 为防 IP 泄漏曾全删 release/docs，连带误删客户需要的用户文档（AT指令/定频测试/常见问题等）。

**修正动作**：
1. **docs 扁平化**：`docs/baseline_reference/*` 提升回 `docs/`，删空壳；`diff -rq` 与基准 **0 差异**（除平台 `.baseline_anchor`/`sync_changelog.md`）。变更记录按 sync skill line 430「与锚点一起提交」留在 `docs/`。
2. **重新 dist + 白名单清理**：`make dist` 的 `rm docs/aic8800-*` 恢复命中；另手动清理 dist 规则**漏删项**（`aic8800-release-adaptation/`、`.baseline_anchor`、`sync_changelog.md`），release/docs = 纯 6 用户文档（1~5.md + 附录 PDF）。
3. **重组 diff 包**：rsync release→驱动镜像（保护 SConscript），lib 三方 md5=1，IP 目录+tar 双扫描干净，重新打包 `tar.gz`（9.1M / 291 文件 / docs 6 个）。

**dist 规则缺陷（建议 backport 基准）**：`Makefile` dist 的 docs 清理（357-358 行）rm 黑名单漏 `aic8800-release-adaptation/`、`.baseline_anchor`、变更记录文件；建议改为**白名单**（只留 `N-*.md` + `*.pdf`）或补全黑名单，杜绝平台侧发布时 IP 泄漏。

### 验证结果
- [x] docs/ 扁平化，与基准 0 差异（除平台锚点/变更记录）
- [x] release/docs = 6 用户文档，无 skills/锚点/变更记录
- [x] diff 包 lib 三方 md5=1，IP 目录+tar 双扫描干净
- [x] tar.gz 重新生成（9.1M / 291 文件 / docs 6 个用户文档）

---

## 平台特性 #1（RT-Thread 标准 Wi-Fi port 对接 rt_wlan + netdev）

### 日期: 2026-09-23

**背景**：为 AIC8800 驱动新增一套 RT-Thread 标准 Wi-Fi 对接层，接入 `rt_wlan` 管理框架 + `netdev`，App 用标准 `rt_wlan_connect/scan/start_ap` API。**方案B 双模**：`CONFIG_AIC8800_USE_RT_WLAN` 开=标准框架（单 STA/AP），关（默认）=维持路线1（自建 netif + STA/AP/P2P 并发）。

**新增文件（平台专属，不进基准同步面）**：
- `platform/wlan/aic8800_wlan_dev.c`：实现 `rt_wlan_dev_ops`（22 op）+ `rt_wlan_dev_register("wlan0")`；op→AIC 映射；扫描结果 `wifi_ap_info_t[]`→`rt_wlan_info[]` 翻译；异步事件桥接。
- 源文件经顶层 `SConscript` 的 `GetDepend('AIC8800_USE_RT_WLAN')` 条件纳入 + CPPPATH 加框架头。
- `customer/peripherals/wifi/Kconfig`：新增 `config AIC8800_USE_RT_WLAN`（default n，`select RT_USING_WIFI` 一键联动）。

**基准派生文件的平台改动（[AIC8800-PORT]，正向同步须保留）**：
| 文件 | 改动 | 说明 |
|---|---|---|
| `platform/wifi/wifi_port.c` | 新增通用事件扩展回调钩子 | +21 行：`static aic_wifi_event_cb m_ext_event_cb`（NULL 守卫）+ `aic_wifi_ext_event_register()` 注册入口 + `aic_wifi_event_callback` 末尾分发。**根因**：AIC 原事件系统的 `m_scan_done_handle` 等 5 个 handle 为 static 且无对外注册入口（恒 NULL），`priv_aic_wifi_event_cb` 签名是 `(MAC_addr,mode)` 属 MAC 通知用途（且从未被调用）→ platform 层无可消费的异步事件通道。本钩子为 rt_wlan 桥接层提供 SCAN_DONE/LEAVE/STA_DISCONNECT 异步事件。**additive + NULL 守卫，路线1(OFF) 零影响**（未注册时恒 NULL，`aic_wifi_ext_event_register` 被链接器 GC）。 |
| `platform/netif/netif_port.c` | RX 桥接 + netif_add 抑制（#ifdef 门控） | `#ifdef AIC8800_USE_RT_WLAN`：`aicwf_rx_eth_data_process` early-return 到 `rt_wlan_dev_report_data`；`aicwf_net_if_add` 的 `netifapi_netif_add` 抑制（netif 归属让给框架）。OFF 时全走原路径。 |

**降级项（AIC8800 无对应 API）**：`wlan_cfg_promisc`/`wlan_cfg_filter`/`wlan_set·get_country`/`wlan_ap_deauth` 返回 `-RT_ENOSYS`；扫描结果无 security 字段→统一 `SECURITY_UNKNOWN`；`wlan_get_powersave` 返回本地缓存（AIC 无 ps_lvl getter）。

### 验证结果
- [x] 双配置编译：ON（`AIC8800_USE_RT_WLAN=y`）+ OFF（默认）各 `scons --board=eh-lb525 -j8` RC=0
- [x] ON：`aic8800_wlan_dev.o` 编入，固件含 40 个 wlan 符号（`rt_wlan_dev_register`/各 `aic8800_wlan_*` ops/`aic_wifi_ext_event_register`/`__rt_init_aic8800_wlan_dev_init` 自动注册入口），0 error / 0 undefined / 0 warning
- [x] OFF 零回归：固件 0 个 wlan 符号，路线1 符号（`aicwf_tx_eth_data_process`/`aic_wifi_start_sta`/`aic_wifi_event_callback`）完好；`wifi_port.c` 钩子改动编译通过且新函数被 GC（不增体积）
- [x] `select RT_USING_WIFI` 联动验证：仅开 `AIC8800_USE_RT_WLAN` 即自动拉起框架
- [ ] 真机联调（需硬件）：`rt_wlan_scan_sync` 出列表 / `rt_wlan_connect`→netdev 拿 IP→ping / `rt_wlan_start_ap` / 拔 AP→DISCONNECT 事件 / TCP-ack offload 吞吐不劣化

---

## 同步 #2

### 日期: 2026-10-13
### 基准版本: `d4509b7f99235130ffc525aaf63f28caa3f047bf` → `f5d2aef27cf35095e8ddfc7366f18237d970c4e5` (origin/master)
### 基准仓库: /home/admin/project/aic8800_rtos
### 变更规模: `git diff d4509b7..f5d2aef --stat` = 67 文件 (+2164 −354)

基准自上次锚点推进 40 个 commit，主线变更主题：
- `c989cbf`/`a078ac7`/`c6ee3e1`：D80N msg 通道免流控（`msg_fc_bypass`）+ chipmatch 失败终止初始化
- `71a90ef`：hidden SSID 定向扫描；扫描结果扩展 `ssid_len`/`auth_mode`；ATRESP 走 AIC 调试日志；gpio trigger id/pin 解耦
- `d987355`：格式化输出统一收敛到平台 porting 接口 `aicwf_dbg_snprintf`，消除 libc 直呼
- `3c823f2`/`696945d`/`1edecc3`：WEP open-first 回退、隐藏网络 scan_ssid、扫描列表占位修正
- `fd13c86`：回流 amt630 平台验证的 4 项共性修复（netif 生命周期/反查/MAC 单一来源/接口名唯一）
- `5048c13`/`25b97a0`/`99e29fa`：check-src-coverage 源-归档对账、交付头单一事实源收敛、版本串 commit 存在性
- `1d5e324`/`ec9bd58`：移除 BT 发送点 wake_rx 兜底，回归纯中断驱动；删除 Makefile lwIP 宏注入
- `b92e52e`：FW 日志使能按芯片族门控（DC/DW 跳过）
- ABI 宽度修正：`uint32_t` → `uintptr_t`（sdio_co/usb_co/netif_api `aic_tx_cb`/fhost_api `cb_fhost_tx`/rtos_port `OsResSt`）；`tx_swdesc.h` 新增 `HOSTDESC_WIRE_LEN`（28B，替代 `offsetof` 打包长度）

#### 段一: 基准驱动同步（与平台无关的变更）

| 文件 | 层 | 变更 | 同步方式 | 说明 |
|------|----|------|---------|------|
| `host/**`（26 文件 + 新增 `scripts/check_src_coverage.py`） | host | 全量覆盖 | 直接覆盖 | 协议/内存/生命周期/并发共性修复；`diff -rq host` 与基准 0 差异 |
| `host/Makefile` | host | +47 | 覆盖 | 版本宏并入 `HOST_DFLAGS`（命令行变量免疫 `+=`）+ `check-src-coverage` 目标 |
| `port/inc/aic_chip_desc.h` | port | +10 | 覆盖 | 新增 `msg_fc_bypass` 字段（仅 D80N=1） |
| `port/inc/aic_log.h` | port | +33 | 覆盖 | 新增 `AIC_MOD_BT` / `DBG_BT_*` 日志族 |
| `port/inc/aic_netbuf.h` | port | +8 | 覆盖 | `CONFIG_SDIO_ADMA` 下新增 `data_oprt`（本平台未用 ADMA） |
| `port/inc/aic_sdio_types.h` | port | +23 | 覆盖 | sg_list `addr`→`buf` + `AICWF_SDIO_SG_LIST_EXTERNAL`（ADMA 未用） |
| `port/inc/rtos_port.h` | port | +16 | 覆盖 | `OsResSt.para0/1` → `uintptr_t`；`aicwf_rtos_get_task_status` 返回原生状态语义 |
| `port/inc/sdio_port.h` | port | +40 | 覆盖 | ADMA 契约文档；三个 `CONFIG_*_DEFAULT` 加 `#ifndef`（可 `-D` 覆盖） |
| `platform/fmac/aic_chip_desc_port.c` | platform | +8 | 覆盖 | 表项补 `.msg_fc_bypass`（D80N=1） |
| `platform/hal/plat_port.c` | platform | +47 | 覆盖 | `aicwf_ATRESP` 改走 `DBG_APP_INF`（AIC 抓包可见）；gpio trigger id→pin 映射表 |
| `platform/hal/sdio_port.c` | platform | +19 | 手动合并 | 新增 `CONFIG_SDIO_ADMA` 两处 `#error` 契约守卫（本平台未开 ADMA） |
| `platform/netif/netif_port.c` | platform | +25 | 手动合并 | `lwip/init.h` + lwIP 版本开机日志 + 8 处 `[AIC8800-PORT][BUGFIX]`→`[BUGFIX](共性)` 重标 |
| `platform/rtos/rtos_freertos.c`/`rtos_rtt.c` | platform | +19/+6 | 手动合并 | `uint32_t`→`uintptr_t`；`get_task_status` 返回原生状态语义（防 ap stop 卡死） |
| `platform/wifi/wifi_port*.c` | platform | +10~30 | 手动合并/覆盖 | `aic_wifi_set_mode`→`aic_wifi_mode_set` 重命名；STA 扫描结果加 AUTH/SSID_LEN；FW 日志 DC/DW 门控 |
| `Makefile`/`CMakeLists.txt`/`aic_cfg.mk` | build | +34/+175/−10 | 直接覆盖 | 交付头收敛 `lib/include/`；`AIC_BUILD_MBEDTLS_PORT` 透传；P2P CMake 门控；**移除 `AIC_FW_LOG_AUTO_EN`** |
| `BUILD.md`/`lwipopts.md` | docs | 覆盖 | 直接覆盖 | 校验和/字节序符号强制项（§8）等 |
| `docs/**` | docs | 镜像 | 全量镜像 | 与基准 `docs/` 扁平 0 差异（除平台锚点/本文件） |

#### 段二: 平台特殊更新（平台适配相关）

| 文件 | 变更 | 同步方式 | 说明 |
|------|------|---------|------|
| `platform/wifi/wifi_port.c` | `aic_wifi_mode_set` 重命名 | 手动合并 | 保留平台 `[AIC8800-PORT]` 通用事件钩子（`m_ext_event_cb` / `aic_wifi_ext_event_register` / 分发块）+ 5 处 `(uint32_t*)` cast；**撤销**平台 `wifi_if.h` include（rename 后由 `wifi_api.h` 提供声明，基线无需该 include） |
| `platform/wifi/wifi_port_sta/ap/p2p/wps.c` | — | 直接覆盖 | 平台仅 1 行 `wifi_if.h` include 补丁，rename 后撤销，采用基准版 |
| `platform/wifi/wifi_port_plt.c` | +aic_chip_desc.h + DC/DW 门控 | 手动合并 | 保留 SiFli 头替换（`<rtthread.h>`/`<drivers/mmcsd_core.h>`）、3 个前置 extern 声明、`mmcsd_wait_cd_changed` |
| `platform/rtos/rtos_rtt.c` | `uintptr_t` ×3 | 手动合并 | 保留 SiFli 适配：`<rthw.h>`+`rt_hw_us_delay`、`RTTHREAD_VERSION>=50002/50001` 数值判据（3.1.3 无 `RT_VERSION_CHECK`）、`rt_mq_send` 回退 |
| `platform/hal/sdio_port.c` | ADMA `#error` ×2 | 手动合并 | 保留 SiFli SDIO 兼容层（`rt_sdio_*` 宏桥接、`aic_sdio_*` 薄封装、`rt_int32_t` 返回） |
| `platform/netif/netif_port.c` | `[BUGFIX](共性)` 重标 + lwIP 版本日志 | 手动合并 | 保留 SiFli 路线1：lwIP 头显式 include、`AIC8800_USE_RT_WLAN` 的 RX 桥接与 `netif_add` 抑制、`netif_linkoutput_fn` cast、`#undef htons` + weak 垫片 |
| `platform/hal/plat_port.c` | ATRESP→`DBG_APP_INF` | 直接覆盖 | 平台日志出口本就经 `aicwf_log_printf`→`rt_kprintf`（默认 `CONFIG_LOG_LEVEL=VRB`/`MODULES=ALL`），控制台可见性不变，且 AIC 抓包日志同步可见 |
| `SConscript` | 撤销 `FW_LOG_AUTO_EN=1` | 配置同步 | 基准移除 host 侧 auto-en（`aic_cfg.mk` 删 `AIC_FW_LOG_AUTO_EN`），FW 日志改由平台 `wifi_port_plt.c` init 后显式 `aic_wifi_fw_log_enable()`（含 DC/DW 门控）；该宏在 host/平台源码已无引用 |
| `port/inc/fmac_port.h` | — | **保留平台版** | 含 `[AIC8800-PORT][SiFli]` 追加声明 `aicwf_rftest_fw_available()`，基准本次未改 port 层此文件 |
| `platform_config_cortex-m33.mk` | — | **保留平台版** | 工具链/cortex-m33 旗标/功能开关为平台特有 |
| `platform_config_riscv-xuantie-e906.mk` | 基准新增 | **跳过** | RISC-V Xuantie 模板，SiFli 不使用 |

### 影响评估
- **功能完整性**：扫描结果新增 `auth_mode`/`ssid_len`（`wifi_ap_info_t` 布局变化，`ssid` 数组 32→33）；AT 扫描输出增加 `AUTH=`/`SSID_LEN=`；WEP 连接长度白名单；D80N msg 通道免流控（本平台 D80 走 `msg_fc_bypass=0`，行为不变）。
- **稳定性**：`aicwf_net_if_*` 一系列共性修复（netif 生命周期、按 vif 反查、MAC 单一来源、接口名唯一）消除驱动重初始化/APSTA 下的槽位泄漏与错接口问题；`hwdesc` 28B 定长打包与 `uintptr_t` 宽度修正消除跨 ABI 错位。
- **兼容性**：`aic_wifi_set_mode`→`aic_wifi_mode_set`（`drv_api/inc/wifi_api.h` 公共头）为**契约变更**，host 库与 platform/桥接层必须同版本重编（已重编，`lib/include` 同步刷新）；`FW_LOG_AUTO_EN` 机制移除。

### 验证结果
- [x] host 编译通过（`host-clean` + `host-build`，cortex-m33，SDIO，BT=0/P2P=0/NIMBLE=0，RC=0，**162 objects**：Driver 39 + WPA 120 + mbedTLS 3）
- [x] platform（SCons）编译链接通过（`scons --board=eh-lb525 -j8`，RC=0，`main.elf` 10.0M / `main.bin` 1.40M）
- [x] 符号核对：`aic_wifi_mode_set`/`aic_chip_desc_chipid_get`/`aicwf_dbg_snprintf`/`aicwf_ATRESP`/`aic_wifi_init_mac`/`aicwf_net_if_add` 均 **T 已定义**，无 `U` 未解析
- [x] 版本串：`NV1.1.6 ... g777ddd0ff 20260916`（锚点基准 commit 不在 sifli-sdk 仓库，按 §1.1 降级用平台仓库 HEAD）
- [ ] 功能测试（需真实硬件：SDIO 卡就绪 + 扫描/连接/DHCP/AP）
- [ ] release/diff 包重新生成（见 `docs/aic8800-diff-package/SKILL.md`）

### 同步备注
- 构建入口注意：**host stage-1 必须显式传功能开关**（`AIC_FEATURE_BT=0 AIC_FEATURE_NIMBLE=0 AIC_FEATURE_PLT=1 AIC_FEATURE_P2P=0`）。`platform_config_*.mk` 内的赋值不随子 make 传播，host 子构建会退回 `aic_cfg.mk` 默认（P2P=1）→ `WPA_P2P_SRC` 纳入 `gas_query.c` 但缺 `-DCONFIG_P2P_SUPPORT`（`CONFIG_GAS` 未定义）编译失败。命令见 `platform_config_cortex-m33.mk` 头部注释。

---

## 同步 #3（b4036bf 增量）

### 日期: 2026-10-13
### 基准版本: `f5d2aef` → `b4036bf8f64f04a1e6a0fa7bc9fc57cca9320cd9` (origin/master)

基准新增 1 个 commit：`b4036bf docs(release): 新增客户可见发布变更记录 docs/SYNC_CHANGELOG.md`（+131 行）。

#### 段一: 基准驱动同步（与平台无关）

| 文件 | 层 | 变更 | 同步方式 | 说明 |
|------|----|------|---------|------|
| `docs/SYNC_CHANGELOG.md` | docs | 新增 131 行 | 镜像 | 客户可见发布变更记录（版本对照表 + 每版三节：平台侧必做动作/功能修复/构建与交付）；不在 make dist 黑名单，随 `docs/` 入包 |

#### 段二: 平台特殊更新（平台适配相关）

| 文件 | 变更 | 同步方式 | 说明 |
|------|------|---------|------|
| `host/Makefile` | 版本解析新增 `baseline_repo` 回退 | **平台修改（host 层例外，需回流基准）** | 本平台驱动目录内嵌于「整个 SiFli-SDK 仓库」，**不含基准驱动仓库对象** → 锚点 commit 在本仓库不可解析 → 原逻辑降级用平台 HEAD（版本串 `g777ddd0ff 20260916`，丢失基准日期）。新增解析顺序 **① 本仓库 → ② 锚点 `baseline_repo` 路径 → ③ 当前 HEAD**，使版本串正确指向基准提交 |
| `docs/.baseline_anchor` | 锚点 `f5d2aef` → `b4036bf` | 更新 | 版本串随之指向 b4036bf/20261013 |

### 影响评估
- **版本号正确性**：修复前包版本串/包名日期取平台移植提交日期（20260916），与基准实际发布日（20261013）不符；修复后一致。
- **无功能影响**：`host/Makefile` 改动仅影响构建期版本宏注入（`AICWF_GIT_HASH/DATE`），不改变库代码/ABI。

### 验证结果
- [x] 版本串修正：`NV1.1.6 STA AP  gb4036bf 20261013`（修复前 `g777ddd0ff 20260916`）
- [x] host `check-src-coverage` 162/162（release 去 -g 库）
- [x] scons `--board=eh-lb525` RC=0；`main.elf` 5.11M（去 -g）/ `main.bin` 1.40M
- [x] diff 包重出：`aic8800_diff_sf32lb52x_sifli_gb4036bf_20261013.tar.gz`（257 文件 / 8.7M / 无 `.o`/`.d`）
- [ ] 真机功能验证（需硬件）

### 风险 / 遗留
- **`host/Makefile` 属 host 层改动**：与基准 host 不再逐字节一致，**下次基准同步若按 host 全量覆盖会丢失** → 建议**回流基准**（在基准 `host/Makefile` 增加同样的 `baseline_repo` 回退），或在 sync 流程中对本文件特殊保留（如 host 覆盖后回填）。
- **大小写冲突**：`docs/SYNC_CHANGELOG.md`（客户可见）与 `docs/sync_changelog.md`（平台内部台账）仅大小写不同，Windows（大小写不敏感）解压/合并会冲突，需保留两份内容。

