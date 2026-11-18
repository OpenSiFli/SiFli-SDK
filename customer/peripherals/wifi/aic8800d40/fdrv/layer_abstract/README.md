│   ├── layer_abstract/            # 抽象层接口
│   │   ├── inc/
│   │   │   ├── wifi_al.h          # WiFi 核心 API（init/start_ap/scan）
│   │   │   ├── wifi_def.h         # WiFi 模式/事件/配置定义
│   │   │   ├── netif_al.h         # 网络接口抽象（tx_start）
│   │   │   ├── netif_def.h        # 网络包结构定义
│   │   │   ├── sdio_al.h          # SDIO 抽象
│   │   │   ├── fmac_al.h          # FullMAC 抽象（固件信息）
│   │   │   └── sys_al.h           # 系统相关
│   │   └── src/                   # 抽象层实现
│   │       ├── wifi_al.c          # → 调用 wlan_if.h / rwnx_msg_tx
│   │       ├── netif_al.c         # → 调用 fhost_tx_start()
│   │       ├── sdio_al.c          # → 调用 sdio_co
│   │       └── fmac_al.c          # → 调用 aic_fw
