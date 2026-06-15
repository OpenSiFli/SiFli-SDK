# Wi-Fi Example
Source path: example/rt_device/wifi

## Overview

This example demonstrates how to use Wi-Fi networking to ping websites and retrieve weather information.


## Supported Boards

This example can run on the following boards:

* sf32lb52-wlan-core
* sf32lb56-wlan-core
* sf32lb58-core

## Code Execution Logic

The application code calls the WLAN API `rt_wlan_scan_sync` to perform a scan operation:  
* Upon successful scanning, information such as SSID, MAC address, RSSI, and channel is printed.

The application code calls the WLAN API `rt_wlan_connect(ssid, password)` to perform a connection operation:  
* After successful connection, a "DHCP SUCCESS" event is triggered, and an IP address is obtained.

Subsequent test functionalities:  
* Use `ping` to test connectivity (relies on lwIP's ICMP implementation).  
* Use the `weather` command to initiate an HTTP request (relies on mbedTLS + lwIP).

## Usage Guide

Taking sf32lb52-wlan-core as an example:

### Hardware Requirements

1. A development board that supports this example
2. A USB data cable with data transfer capability

### Modify ssid and password
* You can replace the ssid and password in main.c with those of the AP you want to connect to
```c
#define WIFI_SSID       "wifi_ssid"     /* WiFi SSID to connect */
#define WIFI_PASSWORD   "wifi_password"         /* WiFi password, set to RT_NULL if open */
```

### menuconfig Configuration

* The default configuration is already set up

```bash
// Run the following command under this example project
sdk.py menuconfig --board=sf32lb52-wlan-core_n16r16
```

1. **Enable SDIO/SDHCI Interface**
    - Path: On chip Peripheral RTOS Drivers → Enable SDIO (SDIO path) / On-chip Peripheral RTOS Drivers → Enable SDIO → Enable SD Host Control Interface → Enable SDHCI Handle 2 (SDHCI path)
    - Enable: Enable SDIO
        - Macro: `CONFIG_BSP_USING_SD_LINE` (SDIO macro) / `CONFIG_BSP_USING_SDHCI` `CONFIG_BSP_USING_SDHCI2` `CONFIG_SDIO2_CARD_MODE` (SDHCI macro)
        - Purpose: Enable SDIO-related configuration, select SDIO or SDHCI interface based on hardware (if using SDHCI IP, SDIO mode must be selected here; native SDIO IP does not require this setting)

2. **Enable AIC8800MC Wi-Fi Module**
    - Path: Select board peripherals
    - Enable: Enable swt 6621 wifi mode
        - Macro: `CONFIG_WIFI_USING_AIC8800MC`
        - Purpose: Enable the AIC8800MC Wi-Fi module to ensure it can be properly recognized and used

3. **Enable wpa_supplicant Framework**
    - Path: Third party packages
    - Enable: Enable WPA Supplicant
        - Macro: `PKG_USING_WPA_SUPPLICANT`
        - Purpose: Enable the wpa_supplicant application framework to ensure the module can communicate with wpa_supplicant and obtain correct Wi-Fi status information

4. **Select Network Interface for wpa (AIC8800MC default is m0)**
    - Path: Third party packages
    - Enable: wpa net device name
        - Macro: `PKG_USING_WPA_NET_NAME`
        - Purpose: Select the wpa network interface, default is m0

5. **Enable lwIP Protocol Stack**
    - Path: RTOS → RT-Thread Components → Network
    - Enable: lwIP: light weight TCP/IP stack
        - Macro: `NET_USING_LWIP`
        - Purpose: Enable the lwIP protocol stack to ensure the module can communicate via the lwIP stack and obtain correct Wi-Fi status information

6. **Select lwIP Version (version 2.1.2)**
    - Path: RTOS → RT-Thread Components → Network → light weight TCP/IP stack
    - Enable: lwIP v2.1.2
        - Macro: `RT_USING_LWIP212`
        - Purpose: Explicitly specify lwIP 2.1.2 to ensure the Wi-Fi example runs properly and achieves network communication

7. **Enable RT-Thread Wi-Fi Framework**
    - Path: RTOS → RT-Thread Components → Device Drivers → Using WIFI
    - Enable: Using Wi-Fi framework
        - Macro: `RT_USING_WIFI`
        - Purpose: Enable the RT-Thread Wi-Fi framework to support basic and advanced Wi-Fi module features

8. **Enable mbedtls Encryption Component**
    - Path: Third party packages
    - Enable: Selected MBedTLS / Use ALL cert provided by mbedtls / User provided cert
        - Macro: `PKG_USING_MBEDTLS` `PKG_USING_MBEDTLS_USER_ALL_CERTS` `PKG_USING_MBEDTLS_USER_CURTS`
        - Purpose: Enable mbedtls encryption library for secure communication; load default certificate chain to verify public servers; allow user-defined certificates for special requirements

### Build and Download

Follow the steps below to build and download:

```bash
scons --board=sf32lb52-wlan-core_n16r16 -j8
build_sf32lb52-wlan-core_n16r16_hcpu\uart_download.bat
```

(To use a different board, simply change the board name. For example, for the sf32lb58-core board, replace 'sf32lb52-wlan-core_n16r16' with 'sf32lb58-core_n16r32n1')

## Example Output

* **Boot Log**

```log
// WiFi Initialization
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
// WiFi Scan
01-29 15:44:30:153    [32m[572402] I/NO_TAG: scan quiet window: fill tail results 1->20
01-29 15:44:30:160    [0m[32m[572450] I/NO_TAG: scan quiet window elapsed, emit SCAN_DONE (total=20)
01-29 15:44:30:161    [0m             SSID                      MAC            security    rssi chn Mbps
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
// WiFi Connect
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
01-29 15:44:37:338    [32m[807843] I/WLAN.mgnt: wifi connect success ssid:sifli-employee
01-29 15:44:37:342    [0mapp_cb: WLAN: authenticated to network
01-29 15:44:40:991    wm_netif_status_callback 562 DHCP SUCCESS
01-29 15:44:40:995    skw_rt_wlan_event_forwarder 309 wifi connect success!
01-29 15:44:40:999    [32m[927587] I/WLAN.mgnt: wifi connect success ssid:sifli-employee
01-29 15:44:40:999    [0m[32m[927615] I/NO_TAG: connected: ssid=sifli-employee channel=6
01-29 15:44:41:000    [0mapp_cb: WLAN: connected to network
01-29 15:44:41:001    Connected to following BSS:
01-29 15:44:41:002    SSID = [sifli-employee]
01-29 15:44:41:003    IPv4 Address: [192.168.2.174]
```

* **Test Connectivity Between Local Device and Server (ping website)**

```log
01-29 15:44:53:437 TX:ping www.sifli.com
01-29 15:44:53:491    60 bytes from 58.144.196.181 icmp_seq=0 ttl=48 time=37 ms
01-29 15:44:54:532    60 bytes from 58.144.196.181 icmp_seq=1 ttl=48 time=32 ms
01-29 15:44:55:573    60 bytes from 58.144.196.181 icmp_seq=2 ttl=48 time=32 ms
01-29 15:44:56:614    60 bytes from 58.144.196.181 icmp_seq=3 ttl=48 time=32 ms
01-29 15:44:56:626    msh />msh />
```

* **Get Weather Information**

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

* **Disconnect (Command: wifi disc)**

```log
01-29 16:01:47:863 TX:wifi disc
01-29 16:01:47:865    get_wpa_s_handle 123 ifname: m01,dev->num=2
01-29 16:01:47:872    get_wpa_s_handle 123 ifname: m01,dev->num=2
01-29 16:01:47:883    wm_netif_status_callback 562 DHCP FAILURE
01-29 16:01:47:884    wifi_deauthenticate 78 pmpriv->media_connected=0 
01-29 16:01:47:884    app_cb: disconnected
01-29 16:01:47:917    [32m[923535] I/WLAN.mgnt: disconnect success!
01-29 16:01:47:924    [0mmsh />msh />
```

## Troubleshooting

If the expected logs do not appear, check the following:

* Verify that the Wi-Fi module is powered on
* Verify that SDIO/SDHCI configuration is correctly enabled
* Verify that software pinmux configuration matches the schematic
* Verify that SDIO/SDHCI registers are properly enabled
* If using SDHCI mode, confirm it is configured as SDIO mode (refer to menuconfig configuration above)
* Verify that the two GPIOs connecting the Host and Wi-Fi chip have correct directions (one input, one output)