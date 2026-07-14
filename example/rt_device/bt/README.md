# RT-Thread 设备框架操作蓝牙（BT）示例

源码路径：example/rt_device/bt

本示例演示**如何完全通过 RT-Thread 标准设备框架**（`rt_device_find` / `rt_device_control` + 事件回调）来驱动 SiFli 蓝牙子系统，而**不是**直接调用底层的 `bt_interface_*` 协议栈接口。

SiFli SDK 会把蓝牙注册成一个 RT-Thread 的 Miscellaneous 设备（设备名 `"bt_device"`）。这个设备比较特殊：它只实现了 `control()` 操作。打开、关闭、注册事件回调、状态查询、以及各个 profile 的动作，全部通过 `rt_device_control` 加不同的 `BT_CONTROL_*` 命令码来完成；异步结果则通过一个注册进去的 `bt_notify_cb` 回调以 `BT_EVENT_*` 事件的形式上报。

在此基础上，蓝牙应用被组织成一个**「核心 + 插件」的多服务框架**：核心已下沉为独立的中间件组件 `rt_bt_app_core`（位于 `middleware/rt_bt_app_core/`），可被任意工程复用；新增一个蓝牙 profile（HFP、SPP……）只需在应用侧新增一个 `.c` 文件和配置开关，无需改动核心或其它服务。


## 支持的平台
+ sf32lb52 系列
+ sf32lb56 系列
+ sf32lb58 系列

## 一、目录结构

核心框架是一个独立的中间件组件，示例工程只保留入口和各 profile 的服务插件：

```
middleware/rt_bt_app_core/       【核心框架，独立中间件组件，可复用】
├── rt_bt_app.h              框架契约：服务描述符 / 命令表 / 注册宏 / 接口
├── rt_bt_app_core.c         设备管理、事件线程与队列、通用事件内置处理、服务注册表
├── rt_bt_app_cmd.c          路由到各服务的命令表
├── Kconfig                  BT_USING_RT_BT_APP_CORE 及其子选项
└── SConscript

example/rt_device/bt/
├── project/                 工程目录
├── src/
│   ├── main.c               入口：初始化核心 + 使能协议栈
│   └── services/
│       ├── SConscript       按 BT_USING_XXX 宏决定编译哪些服务文件
│       ├── bt_srv_hfp.c     HFP 服务（完整实现）
│       └── bt_srv_spp.c     SPP 服务骨架（教学模板，演示简单的框架，不具备完整SPP功能）
└── README.md
```

- **核心（core）**：中间件组件 `rt_bt_app_core`（`rt_bt_app_core.c` / `rt_bt_app_cmd.c`）。持有 `bt_device` 句柄、运行事件服务线程、把事件深拷贝后按「事件码高字节」路由给对应服务、内置处理通用事件（栈就绪 / 搜索 / 连接 / 断开），并提供顶层 `bt` shell 命令。**新增 profile 时核心不需要改动。**
- **服务（service）**：`services/bt_srv_xxx.c`。每个 profile 一个文件，是一个自注册服务，提供三样东西：命令表、事件处理函数、（可选的）事件深拷贝钩子。

---

## 二、框架工作原理

### 1. 事件如何路由到对应服务

每个 profile 的事件码高字节是固定的（定义见 `bt_device.h`）：

| 高字节 | 宏 | 事件组 |
|:---|:---|:---|
| 0x40 | `BT_COMMON_TYPE_ID` | 通用（栈就绪 / 搜索 / 连接 / 断开），由核心内置处理 |
| 0x41 | `BT_HF_TYPE_ID` | HFP |
| 0x46 | `BT_SPP_TYPE_ID` | SPP |
| ... | ... | ... |

核心收到事件后用 `event >> 8` 取高字节，找到 `event_group` 与之匹配的服务，调用它的 `on_event`；若没有服务认领且高字节是 `BT_COMMON_TYPE_ID`，则走核心内置的通用处理。

### 2. 命令如何路由到对应服务

顶层命令是 `bt`，三级路由：

```
bt                          列出所有已注册服务
bt <service>                列出某个服务的子命令
bt <service> <cmd> [args]   执行某个子命令
```

`rt_bt_app_cmd.c` 是通用路由器，本身不认识任何具体 profile：它先按名字找服务，再在服务的命令表里按名字找子命令，检查是否需要协议栈就绪，最后调用命令表里登记的 `handler`。



## 三、如何新增一个蓝牙服务

以新增 `xxx` 服务为例，三步：

1. 在 `src/services/` 下新建 `bt_srv_xxx.c`，实现四部分并自注册：
   - **命令封装**：把动作转成 `rt_bt_app_control(BT_CONTROL_XXX, &arg)`；
   - **命令表** `rt_bt_cmd_entry_t[]`：把 shell 子命令映射到上述封装；
   - **事件处理** `xxx_on_event`：处理本 profile 的 `BT_EVENT_*`；
   - **深拷贝钩子** `xxx_clone`（可选，事件参数带指针时需要）；
   - 末尾 `RT_BT_SERVICE_REGISTER(xxx_service);` 自注册。
2. 在 `src/services/SConscript` 增加一行，按驱动宏门控编译：
   ```python
   if GetDepend('BT_USING_XXX'):
       src += ['bt_srv_xxx.c']
   ```
3. 在 menuconfig / `proj.conf` 里打开对应的 `CONFIG_BT_USING_XXX=y`。


可直接复制 `bt_srv_spp.c` 这个骨架作为起点。

> 服务只依赖 `rt_bt_app.h` 提供的框架契约，包含它即可使用 `rt_bt_service_t`、`rt_bt_cmd_entry_t`、`rt_bt_app_control()`、`RT_BT_SERVICE_REGISTER()` 等接口。

---

## 四、配置与依赖

核心框架由 Kconfig 选项 `BT_USING_RT_BT_APP_CORE` 控制，依赖 `BSP_BLE_SIBLES` 与 `BT_FINSH`。本示例的 `proj.conf` 已开启：

```
CONFIG_BT_USING_RT_BT_APP_CORE=y   # 使能核心框架组件
CONFIG_BT_APP_ENABLE_SHELL_CMD=y   # 使能 "bt" shell 命令（可选）
```

## 五、编译与烧录

在工程目录下（`example/rt_device/bt/project`）：

```
scons --board=sf32lb56-lcd_a128r12n1 -j8
```

编译产物在 `build_<board_name>` 目录下，进入该目录运行 `uart_download.bat` 按提示烧录：

```
build_sf32lb56-lcd_a128r12n1_hcpu\uart_download.bat
```
---



## 六、例程的预期结果

### 1. 开机日志

复位后应看到（关键几条）：

```
BT device "bt_device" found          # rt_device_find 成功
service "hfp" registered (group 0x41) # HFP 服务自注册
...
BT stack ready                        # 协议栈就绪（此后核心自动 power_on + 设置本机名）
```

### 2. 命令总览

```
bt                    # 列出所有服务
bt hfp                # 列出 HFP 的全部子命令             
```

### 3. HFP 子命令说明

| 命令 | 作用 |
|:---|:---|
| `bt hfp c` | 删除全部已配对设备（走连接管理接口，非设备框架命令） |
| `bt hfp start_inquiry` | 搜索周围蓝牙设备（不限设备类别，搜所有） |
| `bt hfp stop_inquiry` | 停止搜索 |
| `bt hfp hfp_connect <addr>` | 与手机建立 HFP 连接（地址格式 `xx:xx:xx:xx:xx:xx`，字节序同搜索日志里的 `device addr`） |
| `bt hfp hfp_disconnect` | 断开 HFP 连接 |
| `bt hfp local_phone_number` | 查询本机（订阅号）号码 |
| `bt hfp remote_calls_info` | 查询远端当前通话列表（CLCC） |
| `bt hfp remote_calls_status` | 查询远端通话状态 |
| `bt hfp make_call <number>` | 拨打电话 |
| `bt hfp call_back` | 重拨上一个号码 |
| `bt hfp answer_call` | 接听来电 |
| `bt hfp handup_call` | 挂断当前通话 |
| `bt hfp volume_control <0-15>` | 设置通话扬声器音量 |
| `bt hfp audio_connect` | 建立通话音频（SCO）链路 |
| `bt hfp audio_disconnect` | 断开通话音频（SCO）链路 |
| `bt hfp battery_update <0-9>` | 通过 HFP 上报本机电量 |

### 4. 典型验证流程（HFP）

由于本机会被设为可发现，最直接的方式是**用手机主动连接**，或用已知地址直接连：

```
bt hfp start_inquiry                 # 搜索，日志打印 device addr
bt hfp hfp_connect 22:5a:46:6d:42:e0 # 用日志里的地址连接
                                     # 连接成功打印 "profile 0x0 connected"
bt hfp make_call 10086               # 拨号
bt hfp answer_call                   # 对方接听后本端可控制
bt hfp handup_call                   # 挂断
```

---
