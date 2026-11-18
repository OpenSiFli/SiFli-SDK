├── platform/                      # 平台适配层
│   ├── hal/                       # SDIO 硬件操作
│   │   ├── sdio_port.h/c         # SDIO 读写、中断、DMA
│   │   ├── sdio_func.h           # SDIO 功能接口
│   │   └── plat_port.h/c         # 平台初始化（电源、reset）
│   ├── netif/                     # 网络接口适配（对接 lwIP）
│   │   └── netif_port.h/c        # netbuf 分配/释放、eth数据收发
│   ├── rtos/
│   │   └── rtos_port.h/c         # RT-Thread 任务/信号量/互斥量封装
│   ├── wifi/
│   │   ├── wifi_if_drv.c         # SDIO 驱动注册（probe/remove）
│   │   ├── wifi_driver_event.h   # 驱动事件上报机制
│   │   ├── wifi_port.h/c         # WiFi 设备状态管理
│   │   └── wifi_port_com.c/rtx.c # WiFi port 通信/RTX
│   └── utils/                     # 工具库（list/pool/ring/endian）
