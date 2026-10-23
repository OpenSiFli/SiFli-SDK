# AIC8800D40-N WiFi 驱动注意事项

本目录是 AIC8800D40-N SDIO WiFi 在 SiFli-SDK 内的集成副本：host 层为预编译库
(`lib/libaic8800_driver.a` + `lib/libaic_mbedtls.a`)，platform 适配层由 SCons 编译
(`SConscript`)，Kconfig 开关见本目录 `Kconfig`。

---

## 1. 固件与芯片子分支（最容易踩的坑）

### 1.1 结论

| 模组 / 芯片子分支 | 需要的编译宏 | u02 套件 | u02c 套件 |
|---|---|---|---|
| D40L-N（chip sub_id = 1） | `CONFIG_AIC8800D80N` | 必须 | 不需要 |
| D40LS-N2C 等 u02c 子型号（chip sub_id != 1） | `CONFIG_AIC8800D80N` + `CONFIG_AIC8800D80N_U02C` | 保留 | **必须** |

- `CONFIG_AIC8800D80N`：注入后编入 u02 四段固件（PATCH / PATCH_TBL / CINIT / CALIB），
  位置 `platform/fmac/aic_fw_port.c`。
- `CONFIG_AIC8800D80N_U02C`：额外编入 u02c 四段固件。
  **该宏没打开时，u02c 芯片会取不到固件指针**，驱动会回落到 u02 套件并最终启动失败。

### 1.2 当前 SDK 集成的默认行为

`customer/peripherals/wifi/aic8800d40-n/SConscript` 中：

```python
if GetDepend('CHIP_SELECT_AIC8800D40N'):
    CPPDEFINES += ['CONFIG_AIC8800D80N', 'CONFIG_AIC8800D80N_U02C']
```

即默认 **两套固件都编进镜像**，D40L-N 和 D40LS-N2C 都能直接跑，
额外 Flash 开销约 79KB（u02c 的 patch 21,888 + patch_tbl 640 + cinit 18,056 + calib 40,128 字节）。

> 若只针对 D40L-N 量产、需要省 Flash，可去掉 `CONFIG_AIC8800D80N_U02C`
> （目前没有对应 Kconfig 开关，需改 SConscript）。

### 1.3 固件选择发生在哪一步（重要：不是 CIS）

固件选择分两步，不要混为一谈：

**第一步（编译期）**：决定镜像里“有哪些固件可用”，由上表两个宏控制。

**第二步（运行期）**：决定“这次用哪一套”，判据是 `chip_sub_id`，**不是 SDIO CIS**。

- SDIO CIS（`card->cis.manufacturer` / `card->cis.product`）只用来识别芯片家族：
  0x9084(D40N) / 0x9085(D40LN) / 0x9086(D40WN) / 0x9081 / 0x9082
  全部映射到 `PRODUCT_ID_AIC8800D80N`，因此 **CIS 无法区分 u02 和 u02c**。
  见 `platform/hal/sdio_port.c` 的 `aicwf_sdio_chipmatch()`。
- u02 / u02c 由 host 库在 `rwnx_fdrv_init()` 中从芯片内部寄存器读出 sub_id
  （32bit 值的 bit[11:4]），再按如下规则选择：

  ```
  chip_sub_id == 1  -> u02  套件（fw_id 58 / 59 / 60 / 61）
  chip_sub_id != 1  -> u02c 套件（fw_id 62 / 63 / 64 / 65）
  ```

  fw_id 枚举定义见 `port/inc/aic_fw_port.h`：
  58=PATCH、59=PATCH_TBL、60=CINIT、61=CALIB（u02）；
  62/63/64/65 为对应的 u02c 版本。

### 1.4 如何确认板子实际用了哪套固件

上电日志里看 MACIF 输出（格式示例）：

```
[AIC][MACIF]CHIP MATCH: USE <chip>, rev=U<xx>, sub_id=<n>, is_h=<n>
[AIC][MACIF]D80N bootrom stage: upload CINIT fw (fw_id=<id> for chip_sub_id=<n>)
[AIC][MACIF]D80N wifi_init stage: upload CALIB fw (fw_id=<id> for chip_sub_id=<n>)
[AIC][MACIF]D80N: WiFi PATCH upload (fw_id=<id> for chip_sub_id=<n>)
```

- `sub_id=1` + `fw_id=58/59/60/61` → u02（D40L-N）
- `sub_id!=1` + `fw_id=62/63/64/65` → u02c（D40LS-N2C）
- 若 u02c 芯片却打印 u02 的 fw_id，说明镜像里没编入 u02c 固件，需检查
  `CONFIG_AIC8800D80N_U02C` 是否注入成功。

> 模组型号名（如 D40LS-N2C）不在驱动识别范围内，驱动只认芯片级 ID。
> 不确定模组属于哪个子分支时，直接看上面日志的 `sub_id`。

---

## 2. 其它注意事项

### 2.1 ABI 必须与预编译库一致

- 编译 platform 层必须使用 `-fshort-enums -funsigned-char -fshort-wchar`
  （SDK 的 arm-none-eabi 配置默认已带，无需额外配置）。
- `CONFIG_RX_NOCOPY` 会改变 `sdio_def.h` 中 `aic_wifi_rxdata_t` 的成员布局，
  预编译库按“开启”编译，因此该宏在 `SConscript` 中写死，不可做成 Kconfig 选项。
- 芯片型号宏 `CONFIG_AIC8800D80N` / `CONFIG_AIC8800D80N_U02C` 由
  `CHIP_SELECT_AIC8800D40N` 映射注入，上层统一使用 D40-N 命名即可。

### 2.2 SDIO 时钟固定 48MHz

57X 走 `BSP_USING_SD_LINE` 路径，最高时钟由
`rtos/rtthread/bsp/sifli/drivers/drv_sdio.h` 的 `SDIO_MAX_FREQ`（48MHz）固定，
**没有对应 Kconfig 选项**。在 `proj.conf` 里写 `CONFIG_SDIO_MAX_FREQ` 无效，
还会与头文件定义冲突产生 redefine 警告。

### 2.3 内存预算

驱动初始化时一次性从系统堆申请约 105KB：

- SDIO RX 缓冲池：40 × 2048B = 80KB
- TX 聚合池：16 × 1548B ≈ 24KB

因此不要随意放大 lwIP 内存池（`PBUF_NUM` / `TCP_SEG_NUM` / `*_PCB_NUM` 等），
否则 `__bss_end__` 上移会挤掉堆，表现为启动时
`[AIC][OSAL][rtos]malloc fail` / `sdio rx buf alloc fail`。

### 2.4 两条对接路线，二选一

| 路线 | 开关 | 说明 |
|---|---|---|
| route 1 | `AIC8800_USE_RT_WLAN=n`（默认） | 驱动自建 lwIP netif，不依赖 rt_wlan |
| route 2 | `AIC8800_USE_RT_WLAN=y` | netif 由 `rt_wlan` 框架创建，驱动只提供 `rt_wlan_dev_ops` |

route 2 下设备绑定必须调用 `rt_wlan_set_mode()`（在 `platform/wifi/wifi_port_plt.c`
的 `aic_wifi_init_mac()` 之后完成），否则 `wifi scan/join` 直接返回 `-RT_EIO`。
此时驱动内部的 `aicwf_net_if_*` / `aicwf_net_dhcp_*` 均为空实现，DHCP 由框架负责。

### 2.5 scan 是阻塞式

`aic_wifi_scan()` 返回即表示扫描结束，且不保证回传 `SCAN_DONE_EVENT`。
route 2 的 scan op 内需要同步上报结果（见 `platform/wlan/aic8800_wlan_dev.c`
的 `aic8800_report_scan_results()`），不能只依赖事件回调。

### 2.6 上电脚

- 直连 GPIO：`AIC8800_RTOS_WIRELESS_PWR_DIRECT_GPIO` + `AIC8800_RTOS_WIRELESS_PWR_GPIO`
- 挂在 I2C IO 扩展（如 AW9523）：`AIC8800_RTOS_WIRELESS_PWR_VIA_IOEXP`，
  并配置 `AIC8800_RTOS_WIRELESS_PWR_IOEXP_PIN` / `_PIN_KEY`；
  这种情况下上电钩子会下移到 `INIT_PREV_EXPORT`，等 I2C 总线就绪。

### 2.7 构建方式

- 本集成副本**不使用 Makefile/CMake**，`aic_cfg.mk` / `platform_config*.mk` 已移除；
  编译入口只有 `SConscript` + `Kconfig`。
- 驱动宏分两类：ABI/平台环境常量（写死）与 Kconfig 选项映射（见 `SConscript` 注释）。
- `BUILD.md`、`docs/` 下仍保留上游 make/cmake 的说明，仅作参考资料，
  与本 SDK 的实际构建流程不一致。

---

## 3. 相关文件

| 文件 | 作用 |
|---|---|
| `SConscript` | 唯一的构建入口（源文件、头文件路径、编译宏） |
| `Kconfig` | 驱动选项（route 选择、RX 模式、固件加载方式、上电脚） |
| `platform/fmac/aic_fw_port.c` | 固件数组注册与 `aic_fw_ptr_get()` 实现 |
| `platform/hal/sdio_port.c` | SDIO 初始化、CIS 读取、`aicwf_sdio_chipmatch()` |
| `platform/netif/netif_port.c` | netif / DHCP / TX-RX 桥接（route 1 与 route 2 分支） |
| `platform/wlan/aic8800_wlan_dev.c` | `rt_wlan_dev_ops` 实现（route 2） |
| `port/inc/aic_fw_port.h` | 固件 ID 枚举 |
| `docs/` | 上游移植与问题排查文档（Make 流程部分已过时） |
