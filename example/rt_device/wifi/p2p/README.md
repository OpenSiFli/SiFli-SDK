# Wi-Fi P2P示例

源码路径：example/rt_device/wifi/p2p

## 概述

本示例演示了使用 Wi-Fi P2P GO 模式创建热点，手机等客户端连接后通过 TCP Echo 服务器进行数据收发。

## 支持的开发板

本示例可在以下开发板上运行：
* sf32lb58-core

> **注意：** 标准 sf32lb58-core 开发板板载的 Wi-Fi 模组为 SWT 系列，本示例依赖 AIC8800MC Wi-Fi 模组及其 P2P 驱动。运行本示例前需将板载 Wi-Fi 模组替换为 AIC8800MC，替换后实际相当于一块使用 AIC8800MC Wi-Fi 模组的板卡。

## 代码执行逻辑

app 代码中调用 wlan 的 `rt_wlan_p2p_go_start(ssid, password)` 接口，启动 P2P GO 模式：
* 该接口内部通过 complete 机制等待固件的 `CUSTOM_MSG_START_P2PGO_IND` 回调，确认 P2P GO 已成功开启后才返回。
* P2P GO 启动后板端 IP 为 192.168.88.1。

P2P GO 启动成功后创建 TCP Echo 服务器线程：
* 监听端口 8888，接收客户端数据并原样回传。
* 手机等客户端连接 P2P GO 热点后可进行 TCP 通信测试。

关闭 P2P GO：
* 通过 MSH 命令 `p2p_stop` 关闭 P2P GO。

## 示例使用方法

以 sf32lb58-core 为例：

### 硬件需求

1. 拥有一块支持该示例的开发板（板载 Wi-Fi 模组需为 AIC8800MC）
2. 一根具备数据传输能力的 USB 数据线
3. 一部安卓手机，需要安装网络调试助手
4. AIC8800MC Wi-Fi 模组（标准 sf32lb58-core 板载的是 SWT 模组，需替换为 AIC8800MC 才能运行本示例）

### 修改 SSID 和 Password

可以在 `main.c` 中将 SSID 和 Password 更换成你需要的：

```c
#define P2P_GO_SSID         "DIRECT-SiFli"      /* P2P GO SSID */
#define P2P_GO_PASSWORD     "12345678"          /* P2P GO password (min 8 chars) */
```

### menuconfig 配置

默认情况下已经配置好了。

```bash
// 在本示例项目下执行指令
sdk.py menuconfig --board=sf32lb58-core_n16r32n1
```

1. **打开 SDIO 接口**
    - 路径：On-chip Peripherals → SDIO → SDMMC2
    - 开启：SDIO、SDMMC2
    - 将 SDMMC2 下的 "SDIO mode" 配置为 2（SDIO 模式：0=SDCARD / 1=EMMC / 2=SDIO）
        - 宏开关：`CONFIG_BSP_USING_SDIO`、`CONFIG_BSP_USING_SDMMC2`、`CONFIG_SDIO2_CARD_MODE=2`
        
    > **注意：** 选择哪个 SDMMC 与板子硬件相关，取决于板载 Wi-Fi 模组连接的是哪个 SDMMC 控制器（sf32lb58-core 为 SDMMC2），请参考对应板卡的原理图确认。

2. **使能 AIC8800MC Wi-Fi 模块**
    - 路径：Peripherals → Wi-Fi → AIC8800MC
    - 开启：AIC8800MC Wi-Fi driver
        - 宏开关：`CONFIG_WIFI_USING_AIC8800MC`

3. **打开 lwIP 协议栈**
    - 路径：RTOS → RT-Thread Components → Network
    - 开启：lwIP: light weight TCP/IP stack
        - 宏开关：`NET_USING_LWIP`

4. **选择 lwIP 版本（版本选择为 2.1.2）**
    - 路径：RTOS → RT-Thread Components → Network → light weight TCP/IP stack
    - 开启：lwIP v2.1.2
        - 宏开关：`RT_USING_LWIP212`

5. **打开 rt-thread Wi-Fi 框架**
    - 路径：RTOS → RT-Thread Components → Device Drivers → Using WIFI
    - 开启：Using Wi-Fi framework
        - 宏开关：`RT_USING_WIFI`

6. **开启 P2P 支持**
    - 路径：Peripherals → Wi-Fi → AIC8800MC → Enable WiFi Direct (P2P) support
    - 开启：Enable WiFi Direct (P2P) support
        - 宏开关：`AIC_ENABLE_P2P`

### 编译和下载

按照以下步骤，可以完成编译和下载：

```bash
scons --board=sf32lb58-core_n16r32n1 -j8
build_sf32lb58-core_n16r32n1_hcpu\uart_download.bat
```

（操作不同的芯片开发板只需要将开发板名称进行更改即可，例如 sf32lb52-wlan-core 开发板，只需将 'sf32lb58-core_n16r32n1' 更换成 'sf32lb52-wlan-core_n16r16'）

## 测试流程

* 上电启动成功之后，使用手机打开 Wi-Fi，会看到 `DIRECT-SiFli`，连接到热点 `DIRECT-SiFli`（密码 `12345678`）
![](./assets/wifi_ap.png)
![](./assets/wifi_connect.png)


* 连接成功后，使用网络调试工具连接 `IP：192.168.88.1 port：8888`，发送任意数据即可收到回传，发送 `p2p_stop` 关闭 P2P GO，网络调试助手会打印连接断开
![](./assets/tcp_connect.png)
![](./assets/tcp_transmit.png)

## 示例输出结果展示

* **开机并启动 P2P GO，并配置为 TCP server**

```log
07-24 14:13:14:117    [P2P] Starting P2P GO: SSID="DIRECT-SiFli"
07-24 14:13:14:117    >>> rwnx_send_fhcustmsg_start_p2p_req()
07-24 14:13:15:087    >>> rwnx_fhcustmsg_start_p2pgo_ind()
07-24 14:13:15:091    p2p_ind: P2P GO started, status=0
07-24 14:13:15:094    ip: 192.168.88.1,  gw: 0.0.0.0,  mk: 255.255.255.0
07-24 14:13:15:095    [32m[65222] I/WLAN.mgnt: wifi connect success ssid:
07-24 14:13:15:097    [0m[P2P] P2P GO started
07-24 14:13:15:097    [P2P Echo] server listening on port 8888

```

* **手机连接 P2P GO 并发送数据**

```log
07-24 14:13:19:804    >>> rwnx_fhcustmsg_assoc_ap_ind()
07-24 14:13:19:807    P2P/AP: STA joined, MAC=22:06:B9:B6:35:0E
07-24 14:13:29:017    [P2P Echo] client connected: 192.168.88.10:33912
07-24 14:13:40:280    [P2P Echo] recv(11): 11 22 33 44
07-24 14:13:41:876    [P2P Echo] recv(8): p2p test
07-24 14:13:43:620    [P2P Echo] recv(11): hello sifli
07-24 14:13:44:287    [P2P Echo] recv(11): hello sifli
```

* **关闭 P2P GO（指令：p2p_stop）**

```log
07-24 14:13:54:102 TX:p2p_stop
07-24 14:13:54:108    p2p_stop
07-24 14:13:54:112    >>> rwnx_send_fhcustmsg_stop_p2p_req()
07-24 14:13:54:118    >>> rwnx_fhcustmsg_disassoc_ap_ind()
07-24 14:13:54:119    P2P/AP: STA left, MAC=22:06:B9:B6:35:0E
07-24 14:13:54:123    >>> rwnx_fhcustmsg_stop_p2pgo_ind()
07-24 14:13:54:125    p2p_ind: P2P GO stopped, status=0
07-24 14:13:54:126    [P2P] P2P GO stopped
07-24 14:13:54:129    msh />
```

## 故障排除

如果未能出现预期的日志，可以从以下方面进行故障排查：

* 检查 Wi-Fi 是否已经上电
* 若使用 SDHCI 模式，确认已配置为 SDIO 模式（参考前文 menuconfig 配置）
* 检查 Host 端与 Wi-Fi 芯片连接的两个 GPIO 的方向是否正确（一个为输入，一个为输出）
* 确认 menuconfig 中已开启 `AIC_ENABLE_P2P` 宏
* 确认手机端连接的 SSID 和密码与 `main.c` 中配置一致

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [RT-Thread Wi-Fi 框架](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/wlan/wlan)
