├── host/                          # Host 侧驱动
│   ├── driver/
│   │   ├── drv_fhost/            # Full Host 驱动
│   │   │   ├── fhost_tx.h        # TX 任务（队列管理、调度、流控）
│   │   │   ├── fhost_rx.h        # RX 任务（解封装、转发）
│   │   │   ├── fhost_config.h    # 配置管理（信道、国家码）
│   │   │   ├── fhost_cntrl.h     # 控制面（连接事件、socket）
│   │   │   ├── fhost_wpa.h       # WPA 安全管理
│   │   │   └── fhost.h/api.h     # 核心数据结构
│   │   ├── drv_wifi/wifi_if.h   # WiFi API 消息定义
│   │   └── drv_wlan/wlan_if.h   # WLAN 高层接口
│   └── libwlan_aic8800_sifli.a   # 预编译库（部分 fhost 实现）
