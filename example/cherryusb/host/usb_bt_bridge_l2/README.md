# USB-BT 透明桥接器（无 IP 转换方案）

源码路径：example/cherryusb/host/usb_bt_bridge_l2

## 概述

本示例实现了一个 **USB-BT 透明桥接器**：Dongle 把 USB RNDIS 与蓝牙 PAN/BNEP 之间收到的以太网帧**直接相互转发**，完全绕过 lwIP 的 IP 层（不做路由、不做 NAT、自身不参与 IP 协议栈）。耳机**直接拿到手机 USB 网络同一网段的 IP**，与手机处在同一个二层/三层网络中。

与 NAT 方案（[`example/cherryusb/host/usb_bt_bridge_nat`](../usb_bt_bridge_nat/README.md)，lwIP NAT）不同，本方案 Dongle 自身没有 IP、不上网，只做"透传管道"。


## 支持的开发板

已在以下开发板验证：
- sf32lb52 lcd 系列

### 硬件需求

- 支持 USB Host 功能的 SiFli 开发板
- USB Type-C 数据线
- 鸿蒙/Android 手机（需支持"USB 网络共享"）
- 支持蓝牙 PAN 的耳机（PANU）

## 技术原理

### 透明转发链路

```
耳机 PANU (10.196.255.x)
    │
    │ DHCP Discover
    ▼
BT PAN / BNEP ──► Dongle 直接转发 ──► USB RNDIS ──► 手机 DHCP Server
手机返回 DHCP Offer / ACK，再沿相反方向透传回耳机
```

转发成功后，耳机拿到手机 USB 网络同网段地址，例如：
- 手机网关：`10.196.255.57`
- 耳机地址：`10.196.255.x`

耳机与手机同处一个 IP 网络，因此**不再需要**：
- IPv4 转发（`IP_FORWARD`）
- NAT
- Dongle `b0` 的 DHCP Server
- Dongle `u2` 的 DHCP Client
- lwIP 的 IP 层路由处理

**说明**：`u2`/`b0` 是 lwIP 接口名（`ifconfig` 中可见的网卡名，不是 USB/蓝牙规范术语）：`u2` 为 USB RNDIS 网卡，`b0` 为蓝牙 PAN/BNEP 网卡。

### 弱钩子机制（核心）

是否走桥由 cherryusb 的 **Kconfig 选项** `PKG_CHERRYUSB_HOST_CDC_RNDIS_NO_NETIF`（对应 C 宏 `USBHOST_RNDIS_NO_NETIF`）决定：

- **桥模式（宏开启）**：`u2` 网卡不注册，USB 收到的每一帧**直接交给钩子** `usbh_rndis_on_raw_rx()` 转发，完全不经过 lwIP；
- **非桥模式（宏关闭）**：`u2` 网卡正常注册，帧走正常的 lwIP 网卡路径。


- **USB → BT**：`external/cherryusb/class/wireless/usbh_rndis.c` 定义弱函数 `usbh_rndis_on_raw_rx(buf, len)`；本工程在 `main.c` 中覆盖实现：当存在 PAN 连接（`s_pan_bnep_id` 有效）时调用 `bt_pan_send_data(s_pan_bnep_id, buf, len)`，把 USB 帧经 BNEP 通道发往耳机。
- **BT → USB**：`middleware/bluetooth/service/bt/bt_finsh/bts2_app_pan.c` 定义弱函数 `bt_pan_on_raw_rx(buff, len)`；本工程在 `main.c` 中覆盖实现：取 RNDIS 发送缓冲 `usbh_rndis_get_eth_txbuf()`，`memcpy` 后经 `usbh_rndis_eth_output()` 发往手机。

### 换机自动重同步（PAN re-sync）

透明桥模式下**耳机的 IP 由手机 DHCP 服务器分配**，必须与手机当前的共享网络处于同一网段。手机每次开启“USB 网络共享”会分配一个网段，且换机或手机重启后可能与之前不同；而蓝牙链路不会随 USB 断开，耳机手里的旧 DHCP 租约不会失效，也不会主动重新获取地址——表现为换机/手机重启后无法上网，直到租约超时或耳机重启。

为此本示例加入了自动重同步：Dongle 检测到**新的 RNDIS 会话**（USB 重新枚举，例如换手机、手机重启、重新插拔）且 PAN 已连接时，会主动断开并重连一次 PAN 连接；耳机侧在重连后会重建网卡并重新运行 DHCP，自动获取新手机的网段地址，无需重启耳机。

- 任何一次 USB 会话变化都会触发一次重连（包括同一台手机的拔插），连接短暂中断几秒，属预期行为；
- 也可在 dongle 侧执行 `pan_net disc` 手动断开 PAN（随后用 `pan_net conn <耳机MAC>` 重连），便于调试验证；
- 该机制依赖耳机侧在 PAN 重连后重新运行 DHCP（基于 SDK 蓝牙中间件的耳机固件，如 `example/bt/pan`，默认满足）；
- 预期日志：

```none
 [USB] RNDIS link up, USB<->BT bridge active
 [BT] New USB session: re-sync PAN, peer will re-run DHCP
 [BT] PAN disconnected
 [BT] PAN re-sync: reconnecting
 [BT] PAN connect request accepted: 5CFD:52:800923 (local NAP, remote PANU)
 [BT] PAN connected, USB<->BT bridge ready

```

### 关键配置

- `PKG_CHERRYUSB_HOST_CDC_RNDIS_NO_NETIF`（cherryusb 提供、本工程开启）：RNDIS 枚举后**不注册** lwIP 网卡 `u2`，链路状态由 RNDIS 驱动自身维护（`usbh_rndis_is_link_up()`），Dongle 完全不走 lwIP。
- `BT_PAN_NO_B0_NETIF`（蓝牙组件提供、本工程开启）：PAN 连接时**不创建** lwIP 网卡 `b0`，收到的帧交给应用钩子 `bt_pan_on_raw_rx()`。

### 局限

- Dongle 自身无 IP，无法用 `ping` 等命令本地联网（它是纯管道）。


## 例程的使用

### 编译和烧录
切换到例程project目录，运行scons命令执行编译：
```
scons --board=sf32lb52-lcd_a128r16 -j8
```
执行烧写命令：
```
build_sf32lb52-lcd_a128r16_hcpu\uart_download.bat
```
按提示选择端口即可进行下载：
```none
please input the serial port num:6
```
### 耳机侧
可另取一块开发板作为耳机侧，耳机侧的固件选择 `example\bt\pan` 这个例程，按如上的编译烧录方式将 `pan` 例程烧录进第二块开发板中作为耳机侧进行测试

## 运行步骤

1. 烧录固件，USB 连接手机并开启"USB 网络共享"
2. 观察 `[USB] RNDIS link up, USB<->BT bridge active`
3. 耳机侧烧录固件开机
4. dongle侧发送扫描命令：`pan_net inquiry start`
5. dongle侧发送连接命令：`pan_net conn <耳机MAC>`（如 `pan_net conn 12345678abcd`）
6. 蓝牙连接成功后耳机应拿到手机网络同网段 IP（如 `10.196.255.x`）并可访问 Internet，可通过 `ping 8.8.8.8` 验证。

## 示例输出

```
TX:ping 8.8.8.8
   D/BT.lwip bts: rt_bt_lwip_protocol_recv netif 0x20037094
   D/NO_TAG bts: Active mode st: 0, inv: 0.00
   D/btapp_pan bts:  BTS2MU_PAN_STS_IND
   D/BT.lwip bts: rt_bt_lwip_protocol_recv netif 0x20037094
   60 bytes from 8.8.8.8 icmp_seq=0 ttl=106 time=901 ms
   D/BT.lwip bts: rt_bt_lwip_protocol_recv netif 0x20037094
   60 bytes from 8.8.8.8 icmp_seq=1 ttl=106 time=283 ms
   D/BT.lwip bts: rt_bt_lwip_protocol_recv netif 0x20037094
   60 bytes from 8.8.8.8 icmp_seq=2 ttl=106 time=286 ms
   D/BT.lwip bts: rt_bt_lwip_protocol_recv netif 0x20037094
   60 bytes from 8.8.8.8 icmp_seq=3 ttl=106 time=284 ms
   msh />msh />
```

## 异常诊断

- **dongle 连接手机没反应**：确认 usb 线是否具备数据传输功能。
- **换手机/手机重启后耳机不能上网**：耳机仍持有旧手机的 DHCP 租约。本示例会在检测到新的 USB 会话后自动重同步 PAN（见“换机自动重同步”）；若未恢复，可在 dongle 侧执行 `pan_net disc` 后再 `pan_net conn <耳机MAC>` 手动重连验证，并检查耳机侧 `ifconfig` 是否已拿到手机当前网段的地址。


- [CherryUSB 官方文档](https://github.com/cherry-embedded/CherryUSB)


