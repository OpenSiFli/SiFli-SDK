# RT经典蓝牙应用框架

`rt_bt_app` 是一个**可复用的经典蓝牙应用框架中间件**。它把「事件线程 + 事件路由 + 命令分发 + shell 命令 + profile 插件化」这些每个蓝牙应用都要写的重复代码，下沉成一个开箱即用的独立组件。

有了它，应用开发者只需为每个蓝牙 profile（HFP、SPP、A2DP……）写一个自注册的服务文件，专注于业务逻辑，而**无需**关心事件线程、消息队列、命令解析这些通用机制，也**无需**直接调用底层协议栈接口。

> 完整的端到端示例见 [example/rt_device/bt](../../../../example/rt_device/bt/README.md)，其中包含 HFP 的完整实现和 SPP 的教学骨架。

## 一、它解决什么问题

直接用 RT-Thread 设备框架驱动蓝牙（`rt_device_find("bt_device")` + `rt_device_control` + 事件回调）时，每个工程都会遇到同样几个麻烦：

| 麻烦 | 框架的解决办法 |
|:---|:---|
| 事件、命令按 profile 分散，加一个 profile 要改一大片 `switch-case` | **插件化服务注册表**：按事件/命令高字节自动路由，新增 profile 只加一个自注册文件，核心零改动 |
| 每个工程都要自己写一套调试命令 | 内置统一的 `bt <service> <cmd>` 三级 shell 命令 |
| 栈就绪后要手动打开设备、设本机名等重复初始化 | 核心**内置通用事件处理**，栈就绪后自动打开设备（`OPEN_DEVICE`）并设置本机名 |

## 二、在 SDK 中的位置

```
应用 / 服务插件  (bt_srv_hfp.c / bt_srv_spp.c ...)   ← 各 profile 业务，自注册
        │  依赖 rt_bt_app.h 的框架契约
        ▼
rt_bt_app   【本中间件】事件线程 / 路由 / 命令分发 / shell
        │  rt_device_find / rt_device_control / 注册 notify 回调
        ▼
bt_device        一个 RT-Thread 设备（只实现 control）
        │  由 customer/peripherals/bluetooth/bt_sifli 注册
        ▼
bt_sifli 驱动    按 cmd>>8 分发 BT_CONTROL_*，上报 BT_EVENT_*
        ▼
SiFli 蓝牙协议栈 (bts2 / sibles)
```

## 三、目录结构

```
middleware/rt_bt_app/
├── rt_bt_app.h        对外接口
├── rt_bt_app.c        核心实现代码
├── rt_bt_app_cmd.c    finsh 命令
├── Kconfig            配置文件
└── SConscript         构建脚本
```


## 四、如何使用

**第一步**：`proj.conf` 中开启 `CONFIG_BT_USING_RT_BT_APP=y`，并按需打开各 profile 的驱动宏（如 `CONFIG_BT_USING_HF=y`、`CONFIG_BT_USING_SPP=y`）。

**第二步**：在 `main` 中初始化框架，再使能协议栈：

```c
#include "rt_bt_app.h"

int main(void)
{
    if (rt_bt_app_init() != RT_EOK)
    {
        LOG_E("BT core init failed");
    }

    sifli_ble_enable();   
    while (1)
    {
        rt_thread_mdelay(10000);
    }
    return 0;
}
```

> 说明：`LOG_E` 需要先 `#include <rtdbg.h>` 并定义 `DBG_TAG`/`DBG_LVL`，完整可编译示例见 `example/rt_device/bt/src/main.c`。

**第三步**：把你要用的服务插件（`bt_srv_xxx.c`）加入工程编译。服务通过 `RT_BT_SERVICE_REGISTER` 自注册，无需在 `main` 里手动登记，参考示例见 `example/rt_device/bt/src/services`。

启动后即可用 `bt` 命令驱动：

```
bt            # 列出所有服务
bt hfp        # 列出 HFP 的全部子命令
```
