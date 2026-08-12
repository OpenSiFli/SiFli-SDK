# RT Classic Bluetooth Application Framework 

`rt_bt_app` is a reusable classic‑Bluetooth application framework middleware. It extracts repetitive code blocks required by every Bluetooth‑based application, including event‑thread management, event routing, command dispatching, shell command processing and pluggable profile modules, into an out‑of‑the‑box independent component.

With this framework, application developers only need to write a self-registering service file for each Bluetooth profile (HFP, SPP, A2DP, etc.), focusing on business logic **without** worrying about event threads, message queues, or command parsing, and **without** directly calling low-level protocol stack interfaces.

> For a complete end-to-end example, see [example/rt_device/bt](../../../../example/rt_device/bt/README_EN.md), which includes a full HFP implementation and an SPP skeleton for educational purposes.

## I. What Problems Does It Solve

When driving Bluetooth directly with the RT-Thread device framework (`rt_device_find("bt_device")` + `rt_device_control` + event callbacks), every project encounters the same challenges:

| Challenge | Framework Solution |
|:---|:---|
| Events and commands scattered by profile; adding a new profile requires modifying large `switch-case` blocks | **Plugin-based service registry**: Automatic routing by event/command high byte; adding a profile only requires one self-registering file with zero changes to the core |
| Each project needs to implement its own debug commands | Built-in unified `bt <service> <cmd>` three-level shell command |
| Manual device opening and name setting after stack ready in every project | Core **built-in common event handler** that automatically opens the device and sets the local name when stack becomes ready |

## II. Position in the SDK

```
Application / Service Plugins  (bt_srv_hfp.c / bt_srv_spp.c ...)   ← Profile business logic, self-registered
        │  Depends on rt_bt_app.h framework contract
        ▼
rt_bt_app   【This Middleware】Event thread / routing / command dispatch / shell
        │  rt_device_find / rt_device_control / register notify callback
        ▼
bt_device        An RT-Thread device (only implements control)
        │  Registered by customer/peripherals/bluetooth/bt_sifli
        ▼
bt_sifli driver  Dispatches BT_CONTROL_* by cmd>>8, reports BT_EVENT_*
        ▼
SiFli Bluetooth Stack (bts2 / sibles)
```

## III. Directory Structure

```
middleware/rt_bt_app/
├── rt_bt_app.h        Public API interface
├── rt_bt_app.c        Core implementation
├── rt_bt_app_cmd.c    Finsh shell commands
├── Kconfig            Configuration file
└── SConscript         Build script
```

## IV. How to Use

**Step 1**: Enable `CONFIG_BT_USING_RT_BT_APP=y` in `proj.conf`, and enable profile driver macros as needed (e.g., `CONFIG_BT_USING_HF=y`, `CONFIG_BT_USING_SPP=y`).

**Step 2**: Initialize the framework in `main`, then enable the protocol stack:

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

> Note: `LOG_E` requires `#include <rtdbg.h>` and the `DBG_TAG`/`DBG_LVL` macros. See the full example in `example/rt_device/bt/src/main.c`.

**Step 3**: Add the service plugins you need (`bt_srv_xxx.c`) to the project build. Services self-register via `RT_BT_SERVICE_REGISTER` without manual registration in `main`. Refer to examples in `example/rt_device/bt/src/services`.

After startup, use the `bt` command to control:

```
bt            # List all services
bt hfp        # List all HFP subcommands
```
