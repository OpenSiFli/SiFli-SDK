# Wi-Fi 示例
源码路径：example/rt_device/wifi

## 概述

本示例演示了如何使用 wlan 组件的 API 实现 Wi-Fi 连接、扫描、断开连接、ping 网站以及获取天气信息等功能。


## 支持的开发板

本示例可在以下开发板上运行：

* sf32lb52-wlan-core
* sf32lb56-wlan-core
* sf32lb58-core

## 基本指令

| 指令 | 功能接口 | 说明 |
| -- | -- | -- |
| wifi scan | wifi_scan | 扫描周围的 Wi-Fi 热点，输出 SSID、MAC 地址、安全类型、信号强度、信道和速率等信息 |
| wifi join | wifi_join | 连接指定的 Wi-Fi 网络（支持无密码和有密码两种格式） |
| wifi disc | wifi_disconnect | 断开当前的 Wi-Fi 连接 |
| wifi status | wifi_status | 显示 STA 和 AP 模式的详细状态信息（SSID、MAC、信道、速率、RSSI、自动连接状态等） |

## 示例使用方法

以 sf32lb52-wlan-core 为例：

### 硬件需求

1. 拥有一块支持该示例的开发板
2. 一根具备数据传输能力的 USB 数据线

### menuconfig 配置

* 默认情况下已经配置好了

```bash
// 在本示例项目下执行指令
menuconfig --board=sf32lb52-wlan-core_n16r16
```

1. **打开 SDIO/SDHCI 接口**
    - 路径：On chip Peripheral RTOS Drivers → Enable SDIO（SDIO 路径） / On-chip Peripheral RTOS Drivers → Enable SDIO → Enable SD Host Control Interface → Enable SDHCI Handle 2（SDHCI 路径）
    - 开启：Enable SDIO
        - 宏开关：`CONFIG_BSP_USING_SD_LINE`（SDIO 宏开关） / `CONFIG_BSP_USING_SDHCI` `CONFIG_BSP_USING_SDHCI2` `CONFIG_SDIO2_CARD_MODE`（SDHCI 宏开关）
        - 作用：打开 SDIO 相关配置，根据硬件选择 SDIO 或 SDHCI 接口（如果采用的是 SDHCI IP，需要在此选择 SDIO 模式，原生 SDIO IP 无需该设置）

2. **使能 SWT6621SL Wi-Fi 模块**
    - 路径：Select board peripherals
    - 开启：Enable swt 6621 wifi mode
        - 宏开关：`CONFIG_BSP_WIFI_SWT6621SL`
        - 作用：使能 SWT6621SL Wi-Fi 模块，确保 Wi-Fi 模块能够被正确识别和使用

3. **配置电源/睡眠/唤醒引脚（根据开发板设计进行选择）**
    - 路径：Select board peripherals → Enable swt 6621 wifi mode
    - 开启：SWT6621 WAKEUP OUT pin number / SWT6621 WAKEUP IN pin number / SWT6621 POWER pin number
        - 宏开关：`CONFIG_WIFI_WAKEUP_OUT_PIN` `CONFIG_WIFI_WAKEUP_IN` `CONFIG_WIFI_POWER_PIN`
        - 作用：定义 Wi-Fi 模块的电源控制和睡眠/唤醒引脚，确保模块在不同工作状态下能够高效运行并降低功耗

| 开发板名称 | 电源引脚 | 睡眠/唤醒输入引脚 | 睡眠/唤醒输出引脚 |
| -- | -- | -- | -- |
| sf32lb52-wlan-core | PA_30 | PA_25 | PA_24 |
| sf32lb56-wlan-core | PA_53 | PA_05 | PA_02 |
| sf32lb58-core | PA_85 | PA_68 | PA_69 |

4. **使能 wpa_supplicant 应用框架**
    - 路径：Third party packages
    - 开启：Enable WPA Supplicant
        - 宏开关：`PKG_USING_WPA_SUPPLICANT`
        - 作用：使能 wpa_supplicant 应用框架，确保模块能够与 wpa_supplicant 应用进行通信，并获取正确的 Wi-Fi 状态信息

5. **选择 wpa 使用的网卡（SWT6621SL 默认网卡为 m0）**
    - 路径：Third party packages
    - 开启：wpa net device name
        - 宏开关：`PKG_USING_WPA_NET_NAME`
        - 作用：选择 wpa 网卡，默认为 m0

6. **打开 lwIP 协议栈**
    - 路径：RTOS → RT-Thread Components → Network
    - 开启：lwIP: light weight TCP/IP stack
        - 宏开关：`NET_USING_LWIP`
        - 作用：打开 lwIP 协议栈，确保模块能够与 lwIP 协议栈进行通信，并获取正确的 Wi-Fi 状态信息

7. **选择 lwIP 版本（版本选择为 2.1.2）**
    - 路径：RTOS → RT-Thread Components → Network → light weight TCP/IP stack
    - 开启：lwIP v2.1.2
        - 宏开关：`RT_USING_LWIP212`
        - 作用：为了明确指定使用 lwIP 2.1.2 版本，从而保证 Wi-Fi 示例能够正常运行并实现网络通信功能

8. **打开 rt-thread Wi-Fi 框架**
    - 路径：RTOS → RT-Thread Components → Device Drivers → Using WIFI
    - 开启：Using Wi-Fi framework
        - 宏开关：`RT_USING_WIFI`
        - 作用：启用 RT-Thread 的 Wi-Fi 框架，从而支持 Wi-Fi 模块的基本功能和高级特性

9. **打开 mbedtls 加密组件**
    - 路径：Third party packages
    - 开启：Seleted MBedTLS / Use ALL cert provided by mbedtld / User provided cert
        - 宏开关：`PKG_USING_MBEDTLS``PKG_USING_MBEDTLS_USER_ALL_CERTS` `PKG_USING_MBEDTLS_USER_CURTS`
        - 作用：启用 mbedtls 加密库以支持安全通信；加载默认证书链以验证公共服务器；允许用户自定义证书以满足特殊需求

### 编译和下载

按照以下步骤，可以完成编译和下载：

```bash
scons --board=sf32lb52-wlan-core_n16r16 -j8
build_sf32lb52-wlan-core_n16r16_hcpu\uart_download.bat
```

（操作不同的芯片开发板只需要将开发板名称进行更改即可，例如 sf32lb58-core 开发板，只需将 'sf32lb52-wlan-core_n16r16' 更换成 'sf32lb58-core_n16r32n1'）

## 示例输出结果展示

* **开机日志**

```log
01-29 15:44:12:972    sdio_scan_card
01-29 15:44:12:973    skw_sdio_probe 591 
01-29 15:44:12:975    SDIO: enabling function 1
01-29 15:44:12:977    SDIO: enabled function successfull
01-29 15:44:12:980    SDIO: enabling IRQ for function 1
01-29 15:44:13:017    skw_sdio_dt_read 480 address=0x40000000 len=16 buf=2002c2c0
01-29 15:44:13:082    fwk maxsize dram: 192492
01-29 15:44:13:084    skw_sdio_dt_write 433 address=0x20200000 maxsize=192492
01-29 15:44:13:679    debug----the address=0x20200000 
01-29 15:44:13:723    fwk maxsize iram: 357024
01-29 15:44:13:724    skw_sdio_dt_write 433 address=0x100000 maxsize=357024
01-29 15:44:14:707    debug----the address=0x100000 
01-29 15:44:14:711    BOOT CP BY SDIO
01-29 15:44:14:989     wifi version:trunk_W23.20.2-rev33829-rev33881-rev33848 20250110-01:39:39, slp=2
01-29 15:44:14:992     
01-29 15:44:14:997    BOOT CP BY SDIO DONE
01-29 15:44:15:000    skw_sdio2_handle_packet 704 WIFIREADY
01-29 15:44:15:001    skw_sdio_dt_read 480 address=0x40000000 len=16 buf=2002c2c0
01-29 15:44:15:002    fwk maxsize calibration: 2372
01-29 15:44:15:115    MAC Address: 00:00:00:00:00:00 
01-29 15:44:15:128    wifi_skw_wpa_supp_dev_init 572 2005bfe0
01-29 15:44:15:129    wpas_init_driver 6531 wpa_s->ifname=m01 drv_priv=20060ca4
01-29 15:44:16:136    l2_packet_init: iface m01 ifindex 2
01-29 15:44:16:153    [supp_if] wifi_skw_wpa_supp_set_default_scan_ies: Default scan ies set successfully
01-29 15:44:16:156    get_wpa_s_handle 123 ifname: m01,dev->num=2
01-29 15:44:16:157    wlan_set_regiontable 1642 
01-29 15:44:16:159    l2_packet_init: iface m01 ifindex 2
```

* **扫描 Wi-Fi（指令：wifi scan）**

```log
01-29 15:44:27:108 TX:wifi scan
01-29 15:44:30:153    [32m[572402] I/NO_TAG: scan quiet window: fill tail results 1->20
01-29 15:44:30:160    [0m[32m[572450] I/NO_TAG: scan quiet window elapsed, emit SCAN_DONE (total=20)
01-29 15:44:30:161    [0m             SSID                      MAC            security    rssi chn Mbps
01-29 15:44:30:162    ------------------------------- -----------------  -------------- ---- --- ----
01-29 15:44:30:163    sifli-employee                  a2:37:68:8c:73:0a  WPA2_AES_PSK   -55   48    0
01-29 15:44:30:163    sifli-employee-WiFi5            a2:37:68:ec:73:0a  WPA2_AES_PSK   -56   48    0
01-29 15:44:30:164    DIRECT-e0-HP M227f LaserJet     4a:5f:99:d7:0a:e0  WPA2_AES_PSK   -58    6    0
01-29 15:44:30:165    HalfSweet                       04:f8:f8:98:0c:22  WPA2_AES_PSK   -62    1    0
01-29 15:44:30:167    sifli-employee                  a0:37:68:3c:73:0a  WPA2_AES_PSK   -65    6    0
01-29 15:44:30:171    sifli-employee-WiFi5            a2:37:68:6c:73:0a  WPA2_AES_PSK   -65    6    0
01-29 15:44:30:171    hw_manage_9720                  64:3e:8c:bf:97:2b  WPA2_AES_PSK   -69    1    0
01-29 15:44:30:172    hw_manage_23a0                  64:3e:8c:bc:23:ab  WPA2_AES_PSK   -75    1    0
01-29 15:44:30:172    ChinaNet-igF9                   14:57:9f:3a:fc:24  WPA2_AES_PSK   -75   11    0
01-29 15:44:30:173    hw_manage_efc0                  40:ee:dd:30:ef:cb  WPA2_AES_PSK   -78    1    0
01-29 15:44:30:174    hw_manage_49c0                  64:3e:8c:bc:49:cb  WPA2_AES_PSK   -78    1    0
01-29 15:44:30:174    hw_manage_d600                  64:3e:8c:bf:d6:0b  WPA2_AES_PSK   -79    1    0
01-29 15:44:30:175    NJYC-5G                         20:87:ec:8e:a9:58  WPA2_AES_PSK   -79   48    0
01-29 15:44:30:176    hw_manage_d640                  64:3e:8c:bf:d6:4b  WPA2_AES_PSK   -83    1    0
01-29 15:44:30:177    NJYC                            20:87:ec:8e:94:68  WPA2_AES_PSK   -85    6    0
01-29 15:44:30:177    NJYC-5G                         20:87:ec:8e:94:6c  WPA2_AES_PSK   -85   48    0
01-29 15:44:30:178    TY UAV_Wi-Fi5                   48:05:e2:f9:0a:d4  WPA2_AES_PSK   -86    1    0
01-29 15:44:30:178    NJYC                            20:87:ec:8e:a9:54  WPA2_AES_PSK   -90   11    0
01-29 15:44:30:180    hw_manage_0fc0                  40:ee:dd:31:0f:cb  WPA2_AES_PSK   -91   10    0
01-29 15:44:30:180    NJHF                            c2:37:ff:e7:83:c2  WPA2_AES_PSK   -97   11    0
01-29 15:44:30:187    msh />msh />
```

* **连接 Wi-Fi（指令：wifi join [Wi-Fi 名称] [Wi-Fi 密码]）**

```log
01-29 15:44:33:452 TX:wifi join sifli-employee zmjnb666
01-29 15:44:36:790    get_wpa_s_handle 123 ifname: m01,dev->num=2
01-29 15:44:36:794    wpa_drv_freertos_authenticate 888
01-29 15:44:36:797    [supp_if] wifi_skw_wpa_supp_authenticate 1101
01-29 15:44:36:798    [supp_if] wifi_skw_wpa_supp_authenticate 1129 params=2005ea38
01-29 15:44:36:799    found ssid=sifli-employee i=1
01-29 15:44:36:799    wlan_cmd_tx_frame 2384 ssid=sifli-employee sizeof(struct skw_join_param)=25,params=2004e170 size=426
01-29 15:44:36:800    wlan_cmd_tx_frame 2402 len=401
01-29 15:44:36:801    wlan_skw_htinfo 2334 ht_oper->field1=0
01-29 15:44:36:803    ctx: 0, bind
01-29 15:44:36:804    invalid lmac id: 0
01-29 15:44:36:806    [supp_if] wifi_skw_wpa_supp_authenticate 1132 params=2005ea38,params->auth_alg=1,params->auth_data_len=0
01-29 15:44:36:807    [supp_if] wifi_skw_wpa_supp_authenticate:Authentication request sent successfully
01-29 15:44:36:808    wpa_drv_freertos_authenticate 891
01-29 15:44:36:823    sme_event_auth 1660 auth_type=0
01-29 15:44:36:824    sme_associate 2040 params=2004e170
01-29 15:44:36:825    wlan_set_regiontable 1642 
01-29 15:44:36:834    wlan_ops_sta_ioctl: 0x20000
01-29 15:44:36:835    wlan_ops_sta_ioctl: MLAN_IOCTL_BSS
01-29 15:44:36:836    wlan_bss_ioctl 346: sub_command = 131073
01-29 15:44:36:837    wlan_bss_ioctl_start 295 i=1
01-29 15:44:36:837    [supp_if] wifi_skw_wpa_supp_associate: Association request sent successfully
01-29 15:44:37:338    [32m[807843] I/WLAN.mgnt: wifi connect success ssid:sifli-employee
01-29 15:44:37:342    [0mapp_cb: WLAN: authenticated to network
01-29 15:44:40:991    wm_netif_status_callback 562 DHCP SUCCESS
01-29 15:44:40:995    skw_rt_wlan_event_forwarder 309 wifi connect success!
01-29 15:44:40:999    [32m[927587] I/WLAN.mgnt: wifi connect success ssid:sifli-employee
01-29 15:44:40:999    [0m[32m[927615] I/NO_TAG: connected: ssid=sifli-employee channel=6
01-29 15:44:41:000    [0mapp_cb: WLAN: connected to network
01-29 15:44:41:001    Connected to following BSS:
01-29 15:44:41:002    SSID = [sifli-employee]
01-29 15:44:41:003    IPv4 Address: [192.168.2.174]
```

* **测试本地设备与服务器之间的连通性（ping 网站）**

```log
01-29 15:44:53:437 TX:ping www.sifli.com
01-29 15:44:53:491    60 bytes from 58.144.196.181 icmp_seq=0 ttl=48 time=37 ms
01-29 15:44:54:532    60 bytes from 58.144.196.181 icmp_seq=1 ttl=48 time=32 ms
01-29 15:44:55:573    60 bytes from 58.144.196.181 icmp_seq=2 ttl=48 time=32 ms
01-29 15:44:56:614    60 bytes from 58.144.196.181 icmp_seq=3 ttl=48 time=32 ms
01-29 15:44:56:626    msh />msh />
```

* **获取天气信息**

```log
01-29 15:44:59:751 TX:weather
01-29 15:44:59:779    DNS lookup succeeded, IP: 116.62.81.138
01-29 15:44:59:834    id:"WM7B0X53DZW2"
01-29 15:44:59:835    name:"重庆"
01-29 15:44:59:836    country:"CN"
01-29 15:44:59:836    path:"重庆,重庆,中国"
01-29 15:44:59:838    timezone:"Asia/Shanghai"
01-29 15:44:59:839    timezone_offset:"+08:00"
01-29 15:44:59:841    txt:"阴"
01-29 15:44:59:842    code:"9"
01-29 15:44:59:844    temperature:"11"
01-29 15:44:59:846    last_update:"2026-01-29T15:30:20+08:00"
01-29 15:44:59:851    msh />msh />
```

* **断开连接（指令：wifi disc）**

```log
01-29 16:01:47:863 TX:wifi disc
01-29 16:01:47:865    get_wpa_s_handle 123 ifname: m01,dev->num=2
01-29 16:01:47:872    get_wpa_s_handle 123 ifname: m01,dev->num=2
01-29 16:01:47:883    wm_netif_status_callback 562 DHCP FAILURE
01-29 16:01:47:884    wifi_deauthenticate 78 pmpriv->media_connected=0 
01-29 16:01:47:884    app_cb: disconnected
01-29 16:01:47:917    [32m[923535] I/WLAN.mgnt: disconnect success!
01-29 16:01:47:924    [0mmsh />msh />
```

## 故障排除

如果未能出现预期的日志，可以从以下方面进行故障排查：

* 检查 Wi-Fi 是否已经上电
* 检查 SDIO/SDHCI 相关配置是否正确打开
* 检查软件 pinmux 配置是否与原理图一致
* 检查 SDIO/SDHCI 寄存器是否已正确使能
* 若使用 SDHCI 模式，确认已配置为 SDIO 模式（参考前文 menuconfig 配置）
* 检查 Host 端与 Wi-Fi 芯片连接的两个 GPIO 的方向是否正确（一个为输入，一个为输出）
