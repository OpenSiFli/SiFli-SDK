# USB-BT 网络桥接器（lwIP NAT 方案）

源码路径：example/cherryusb/host/usb_bt_bridge_nat

## 概述

本示例实现了一个 **USB-BT 网络桥接器**：Dongle（SiFli SoC）通过 USB 以 **RNDIS Host** 身份接入手机（鸿蒙/Android）的"USB 网络共享"，同时以经典蓝牙 **PAN NAP** 角色接入蓝牙耳机（PANU）。耳机侧的数据报文经 Dongle 做 **lwIP NAT（IPv4 转发 + 网络地址转换）** 后，从 USB RNDIS 上行口转发到手机，从而让耳机获得访问 Internet 的能力。

本方案是"有 IP 的桥接"：Dongle 自身拥有两个 IP 接口（`b0` 蓝牙侧、`u2` USB 侧），通过 NAT 把耳机的私有网段 `192.168.43.0/24` 转换到手机 USB 网络（如 `10.196.255.x`）。


## 支持的开发板

已在以下开发板验证：
- sf32lb52 lcd 系列

### 硬件需求

- 支持 USB Host 功能的 SiFli 开发板
- USB Type-C 数据线
- 鸿蒙/Android 手机（需支持"USB 网络共享"）
- 支持蓝牙 PAN 的耳机（PANU）


## 技术原理

### 系统拓扑

```
Internet (WiFi/4G/5G)
        │
   ┌────┴───────┐
   │ 手机 (RNDIS Device) │  USB 网络共享（DHCP Server）
   └─────┬──────┘
         │ USB Type-C
   ┌─────┴──────┐
   │ Dongle（本示例）      │
   │  USB RNDIS Host      │  u2: 10.196.255.x（DHCP Client）
   │  BT PAN NAP          │  b0: 192.168.43.1（DHCP Server）
   │  lwIP NAT            │  NAT: 192.168.43.0/24 → u2
   └─────┬──────┘
         │ BT PAN / BNEP
   ┌─────┴──────┐
   │ 耳机 (PANU)           │  192.168.43.x（DHCP Client）
   └────────────┘
```

### 数据流

上行（耳机 → Internet）：

```
耳机 192.168.43.x
  → b0 接口（PAN BNEP 收包）
  → lwIP ip4_forward
  → ip_nat_out：源 192.168.43.0/24 命中 NAT 表 → 源地址改写为 u2 的 IP
  → u2 接口（USB RNDIS 发包）
  → 手机 USB 网络 → Internet
```

下行（Internet → 耳机）由 `ip_nat_input` 在收到回包时做反向 NAT（目的地址改写回 `192.168.43.x`），再从 `b0` 送回耳机。

### 关键实现点

- **USB 侧（`u2`）**：CherryUSB RNDIS Host 枚举手机后，`usbh_lwip.c` 注册 lwIP 网卡 `u2`，`eth_device_init()` 自动启动 DHCP 客户端，从手机获取 IP/网关（如 `10.196.255.3 / gw 10.196.255.57`）。
- **蓝牙侧（`b0`）**：`bt_pan` 栈在 PAN 连接建立后注册 lwIP 网卡 `b0`（NAP 模式），并启动 DHCP 服务器（`LWIP_USING_DHCPD`），向耳机分配 `192.168.43.2 ~ .254`，网关为 `b0` 的 `192.168.43.1`。
- **NAT**：`main.c` 的 `rndis_monitor_thread` 等待 `u2` DHCP 绑定成功后调用 `update_nat()`，建立 NAT 规则 `source=192.168.43.0/24 → out_if=u2`（`LWIP_USING_NAT`）。NAT 只作用于**转发路径**（`ip4_forward`），不影响 Dongle 自身流量。
- **Dongle 自身上网**：RNDIS 上行口拿到地址后，应用将其设为默认网卡（`netdev_set_default(u2)`），Dongle 自身的 ping/DNS/socket 走手机上行口。


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


### 运行步骤

**阶段一：USB 网络共享验证**
1. 烧录固件，串口打开终端
2. USB 连接手机，手机开启"USB 网络共享"
3. 观察 `[USB] DHCP bound ...` 日志
4. Dongle 自身联网验证：`ping 8.8.8.8`

**阶段二：蓝牙 PAN 连接**
1. 耳机侧烧录固件开机
2. dongle侧发送扫描命令：`pan_net inquiry start`
3. dongle侧发送连接命令：`pan_net conn <耳机MAC>`（如 `pan_net conn 12345678abcd`）
4. dongle侧观察 `[BT] PAN connected ...` 与 `[NAT] Enabled 192.168.43.0/24 b0 -> u2`

**阶段三：耳机上网验证**
- 耳机侧应通过 `b0` 的 DHCP 获得 `192.168.43.x` 地址，并能访问 Internet，可通过 `ping 8.8.8.8` 验证。

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


- [CherryUSB 官方文档](https://github.com/cherry-embedded/CherryUSB)
