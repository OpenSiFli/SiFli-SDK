#### 指令说明

**aicwifi** 驱动添加了基础的功能指令，指令格式为 **iwpriv[cmd] [argv1] [argv2] ...** 参数都是空格作为间隔
iwpriv是前缀可以根据平台自行修改适配。

> 前缀占位符: 驱动按 `AICWF_CMD_SKIP_PREFIX` (默认 1) 跳过行首第一个 token 作为前缀
> (如 `wifi` / `iwpriv`)，平台 shell 无占位符时可置 0，此时命令直接以 cmd 名开头。

#### 指令列表

*AT 指令后加 [] 为必填参数，加 <> 为可选参数*

1. **iwpriv aicrftest** [argv1] <argv2> ...

   定频测试，参考文档 **3-定频测试说明**

2. **iwpriv stainfo**

   AP 模式下，返回已经连接上的 STA 设备信息

   ```
   iwpriv stainfo
   addr:88:0:33        ← 每个已连接 STA 一行
   sta:n:2             ← STA 数量
   ```

3. **iwpriv mac [argv1] <argv2>**

   通常是主控启动后从flash中读取mac，再传递给 aicwifi 驱动，aicwifi 不涉及mac地址 存储，如果平台没有写入mac，aicwifi 驱动则会使用软件上的默认mac地址

   | 参数                           | 描述                                       |
   | ------------------------------ | ------------------------------------------ |
   | iwpriv mac 0 88 00 33 55 69 aa | 临时修改mac，重新启动ap生效，不会写入flash |
   | iwpriv mac 1                   | 读取当前使用的mac                          |

4. **iwpriv stanum**

   AP 模式下，获取当前连接的 STA 设备数量

   ```
   iwpriv stanum
   n:2
   ```

5. **iwpriv info**

   查看当前驱动版本、芯片 ID (cid/sid/fmat) 与当前工作模式

   ```
   iwpriv info
   ver:V1.1.6 STA AP P2P g50baafa 20260920 cid:0x43(H) sid:1 fmat:3 mode:1
   ```

6. **iwpriv fw**

   查看当前固件版本

   ```
   iwpriv fw
   fw ver: aic8800D80_fw_u02_v0.0.0.0
   ```

7. **iwpriv ps [argv1]**

   AP 模式下，设置无连接状态下的休眠等级，一共5个等级，电流功耗详情参考文档 **附录A**

   | 参数        | 描述                   |
   | ----------- | ---------------------- |
   | iwpriv ps 1 | 休眠等级最高，功耗最低 |
   | iwpriv ps 5 | 休眠等级最低，功耗最高 |

8. **iwpriv pp [argv1]**

   设置主控的主频等级

   | 参数                | 描述       |
   | ------------------- | ---------- |
   | iwpriv pp 2/4/6/7/8 | 可配置等级 |

9. **iwpriv temp**

   查看当前芯片温度，DW支持

   ```
   iwpriv temp
   get chip temp: 32
   ```

10. **iwpriv dpd**

    芯片启动会进行一次调制校准，查询校准结果，DW支持

    ```
    iwpriv dpd
    dpd result: 0
    ```

11. **iwpriv sta [argv1] <argv2>**

    | 参数                              | 描述                                                         |
    | --------------------------------- | ------------------------------------------------------------ |
    | iwpriv sta scan                   | 扫描周围ap列表                                               |
    | iwpriv sta conn ssid pass timeout | 连接ap，ssid=连接ap-ssid，pass=连接ap密码，timeout=连接超时时间(ms)，成功返回分配的 IP |
    | iwpriv sta stop                   | 断开连接的ap                                                 |

    ```
    iwpriv sta conn myssid 12345678 10000
    +OK ip: 192:168:13:66
    ```

12. **iwpriv rssi**

    STA 模式下，获取当前连接的AP信号强度

    ```
    iwpriv rssi
    Conn AP rssi: -48
    ```

13. **iwpriv shut [argv1]**

    芯片上下电，卸载加载驱动

    | 参数          | 描述                                                       |
    | ------------- | ---------------------------------------------------------- |
    | iwpriv shut 6 | wifi芯片掉电+上电+加载驱动                                 |
    | iwpriv shut 1 | wifi芯片上电+加载驱动                                      |
    | iwpriv shut 5 | wifi芯片掉电+卸载驱动                                      |
    | iwpriv shut 2 | 卸载驱动                                                   |
    | iwpriv shut 4 | 配置自适应模式+wifi芯片掉电+卸载驱动+wifi芯片上电+加载驱动 |
    | iwpriv shut 3 | wifi芯片掉电+卸载驱动+wifi芯片上电+加载驱动                |

    > 调试保留值: `shut 222` / `shut 666` / `shut 999` 分别置 SDIO 异常注入标志
    > `sdio_unexcept_test=1/2/0` (1: 总线异常模拟 / 2: 异常恢复模拟 / 0: 关闭)，
    > 仅用于 SDIO 总线异常恢复路径测试，量产勿用。

14. **iwpriv ap [argv1] [argv2]**

    ap配置参照 wifi_port.c wifi_ap_cfg

    | 参数                                        | 描述                                    |
    | ------------------------------------------- | --------------------------------------- |
    | iwpriv ap start band type he acs channel wpa | 打开 SoftAP                             |
    | iwpriv ap stop                              | 关闭ap                                  |

    - band: 0=2.4g, 1=5g
    - type: 0=20M, 1=40M
    - he:   1=ax 模式
    - acs:  1=自动信道
    - channel: 手动信道号
    - wpa:  0=none, 1=wep, 2=wpa, 3=wpa-wpa2, 4=wpa2, 5=wpa2-wpa3, 6=wpa3

    ```
    iwpriv ap start 1 1 0 1 36 0      ; 5G 40M ch36 ACS open
    iwpriv ap start 0 0 0 0 6 4       ; 2.4G 20M ch6 WPA2 (SSID/密码由 wifi_ap_cfg 配置)
    iwpriv ap stop
    ```

15. **iwpriv p2pstart [argv1] <argv2> <argv3>**

    | 参数                                        | 描述      |
    | ------------------------------------------- | --------- |
    | iwpriv p2pstart [devname] [passphrase] [ssid] | 打开 p2p (devname=P2P 设备名, passphrase=P2P 连接密码, ssid=P2P Group SSID) |
    | iwpriv p2pstart                             | 关闭 p2p  |

    > 注意参数顺序: devname → passphrase → ssid (与代码 argv[1..3] 一致)

    ```
    iwpriv p2pstart DIRECT-ab 12345678 DIRECT-ab-SS
    iwpriv p2pstart
    ```

16. **iwpriv switch [argv1]**

    运行时切换工作模式 (等价于驱动重载，约 2~3s 生效)

    | 参数             | 描述                                     |
    | ---------------- | ---------------------------------------- |
    | iwpriv switch sta  | 切换到 STA 模式                         |
    | iwpriv switch ap   | 切换到 AP 模式                          |
    | iwpriv switch test | 切换到 RF 定频测试模式 (需加载 rf bin)  |

    ```
    iwpriv switch test
    set OK wait 2~3s switch 1 to 3
    ```

17. **iwpriv uapst**

    查询当前 WiFi 状态与初始化状态

    ```
    iwpriv uapst
    wifi st:1 init:1
    ```

18. **iwpriv stalist**

    AP 模式下，获取已连接 STA 列表 (字符串形式，含完整 MAC)

    ```
    iwpriv stalist
    ```

19. **iwpriv apinfo**

    STA 模式下，查看当前连接 AP 的信息 (SSID/BSSID/频率/带宽/速率)。需 CONFIG_STATION。

    ```
    iwpriv apinfo
    ssid:myssid bssid:88:00:33:55:69:aa freq:2412 band:0 fmt:2 rate:866
    ```

20. **iwpriv mem [argv1]**

    内存/资源调试 (需 CONFIG_RTOS_RESLIST)

    | 参数          | 描述                             |
    | ------------- | -------------------------------- |
    | iwpriv mem 0  | 查询当前内存使用计数             |
    | iwpriv mem 1  | 打印资源列表详情 (结果看 log)    |
    | iwpriv mem 2  | 释放全部残留资源 (结果看 log)    |
    | iwpriv mem N  | 测试性分配 N 字节并返回指针      |

    ```
    iwpriv mem 0
    mem u:35  m:35 f:0
    ```

21. **iwpriv wps [argv1] <argv2>**

    WPS 操作 (AP/STA 按当前模式生效)

    | 参数                        | 描述                    |
    | --------------------------- | ----------------------- |
    | iwpriv wps pbc              | PBC 按键方式            |
    | iwpriv wps pin [pin]        | PIN 方式                |
    | iwpriv wps cancel           | 取消本次 WPS            |
    | iwpriv wps disable          | 关闭 WPS 功能           |
    | iwpriv wps cms [methods]    | 配置 AP WPS methods 位图 |

    ```
    iwpriv wps pbc
    ```

22. **iwpriv filter [mode] [mac1..6]**

    AP 模式 MAC 地址过滤 (黑白名单，单次配置 1 条 MAC；生效列表由 AP 配置侧叠加)

    | mode | 描述       |
    | ---- | ---------- |
    | 0    | 关闭过滤   |
    | 1    | 白名单模式 |
    | 2    | 黑名单模式 |

    ```
    iwpriv filter 2 aa bb cc dd ee ff    ; 将该 MAC 加入黑名单
    iwpriv filter 0 0 0 0 0 0 0          ; 关闭过滤
    ```

23. **iwpriv agg [en] [id]**

    聚合开关调试

    | 参数         | 描述                    |
    | ------------ | ----------------------- |
    | en           | 0=关闭聚合, 1=打开聚合  |
    | id           | TID (0~7)               |

    ```
    iwpriv agg 0 0     ; 关闭 TID0 聚合
    iwpriv agg 1 0     ; 恢复 TID0 聚合
    ```

24. **iwpriv bare_bypass [0/1]**

    纯 TCP ACK 重排旁路运行时开关 (调试用，1=旁路驱动内 TCP ACK 重排)

    ```
    iwpriv bare_bypass 1
    +OK bare_bypass:1
    ```

25. **iwpriv cca [rise] [fall]**

    CCA 阈值设置 (20M primary/secondary 上升沿=rise，下降沿=fall)

    ```
    iwpriv cca -60 -70
    +OK r:-60 f:-70
    ```

26. **iwpriv fixrate [mcs] [bw] [gi] [fmt]**

    固定速率发送 (STA/AP 均作用于当前 vif 的对端)

    | 参数 | 描述                                                  |
    | ---- | ----------------------------------------------------- |
    | mcs  | 速率索引 (legacy 0-11 / MCS 索引)                     |
    | bw   | 0=20M, 1=40M                                          |
    | gi   | 0=800ns, 1=400ns (short GI)                           |
    | fmt  | 0=legacy, 2=HT, 4=VHT, 5=HE_SU                        |

    ```
    iwpriv fixrate 11 0 0 0      ; legacy 11M 20M 长GI
    iwpriv fixrate 7 1 1 2       ; HT MCS7 40M shortGI
    iwpriv fixrate 9 1 1 5       ; HE MCS9 40M shortGI
    ```

27. **iwpriv 5g**

    一键快捷配置 5G AP 并设置 5G 扫描信道 (固定 SSID=BTDs-5G 密码=12345678，
    ch40 40M AX WPA，信道表 36~165 共 13 个，调试用)

    ```
    iwpriv 5g
    ```

28. **iwpriv iperf start [argv1]...**

    启动 iperf 测速

    ```
    iwpriv iperf start -s -i 2                 ; server 模式, 2s 周期上报
    iwpriv iperf start -c 192.168.13.20 -i 2   ; client 模式, 打流到 192.168.13.20
    ```

29. **iwpriv iperf stop**

    停止iperf。

    ```
    iwpriv iperf stop
    ```

30. **iwpriv btstart**

    启动 BT HCI 通道。BT patch 固件已在驱动初始化阶段无条件加载（host aicbt_init），此命令仅在 HCI over UART（AIC_HCI_TRANSPORT=uart）时拉起 bt uart 接口；SDIO 邮箱 / USB HCI 由 platform 层自动拉起，无需此命令。需 AIC_FEATURE_BT=1。

    ```
    iwpriv btstart
    +OK
    ```

31. **iwpriv btstop**

    停止 BLE 上层栈（仅 AIC_FEATURE_NIMBLE=1 时有效）：停广播 → 退宿主任务 → 停 HCI RX，并复位幂等标志使可重启（返回 +OK / +ERR）。只影响 BLE，不影响 WiFi 业务（RX 任务由 WLAN 侧共用保护）。

    ```
    iwpriv btstop
    +OK
    ```

32. **双 VIF 并发（APSTA / APP2P）启动顺序**

    APSTA（AP+STA）：先 iwpriv sta conn 连 AP，再 iwpriv ap start 起 SoftAP；STA 占 vif0，AP 落到 vif2。
    APP2P（AP+P2P）：必须先 iwpriv p2pstart 再 iwpriv ap start，顺序颠倒返回 -12。
    并发时 SoftAP 实际运行在 softap_vif_idx（非固定 vif0），ACL 黑白名单、stainfo、信道切换自动跟随该索引；AP 可关联 STA 上限 32。
    API 侧：带 fhost_vif_idx 参数的接口请先用 aic_wifi_get_ap_vif_idx() 取真实 AP 索引，不要硬编码 0。
    约束：NX_VIRT_DEV_MAX 需 ≥3（vif0=STA/P2P、vif1=扫描、vif2=SoftAP），NX_REMOTE_STA_MAX 须与固件 BCMC 基数一致（默认 32）。

33. **FW 日志调试接口 (API 侧)**

    FW 日志回传通道由平台侧显式使能 (host 不做开机自动使能)，常用 API:

    | API                                        | 描述                                       |
    | ------------------------------------------ | ------------------------------------------ |
    | aic_wifi_fw_log_enable(mod_filter, sev)    | 一键使能: 回传通道+lp_level 闸门+模块/级别过滤 |
    | aic_wifi_fwlog_switch(0/1)                 | 仅开关回传通道 (0x13)                      |
    | aic_wifi_set_lp_level(0, 1)                | lp_level=0 才允许 FW 发日志                |
    | aic_wifi_fw_log_set_mod_filter(mask)       | 模块位图 bit0-7=KE/DBG/IPC/DMA/MM/TX/RX/PHY, 0xFFFFFFFF=ALL |
    | aic_wifi_fw_log_set_sev_filter(sev)        | 级别阈值 0=NONE 1=CRT 2=ERR 3=WRN 4=INF 5=VRB |

    FW 日志经总线 Host-Type 0x13 (CFG_PRINT) 通道回传，host 侧以 `[FW] <log>` 格式打印。
