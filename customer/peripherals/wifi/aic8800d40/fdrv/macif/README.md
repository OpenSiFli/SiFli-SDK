│   └── macif/                     # MAC 接口（RivieraWaves 框架）
│       ├── rwnx_main.h/c          # 驱动主结构体和初始化
│       ├── rwnx_msg_tx.h/c        # 向固件发送消息（控制/数据）
│       ├── rwnx_msg_rx.h/c        # 接收固件消息（事件/通知）
│       ├── rwnx_rx.h/c            # RX 数据处理（descriptor、radiotap）
│       ├── rwnx_platform.h/c      # 平台操作（固件加载、启动）
│       ├── rwnx_cmds.h/c          # 命令管理
│       ├── rwnx_defs.h            # 驱动数据结构定义
│       ├── tx_swdesc.h            # TX 软件描述符定义
│       ├── lmac_msg.h             # LMAC 消息协议定义
│       └── mac_frame.h            # MAC 帧格式定义
