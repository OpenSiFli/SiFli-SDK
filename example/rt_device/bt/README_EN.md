# RT-Thread Device Framework Bluetooth (BT) Example

Source code path: example/rt_device/bt

This example demonstrates how to drive the SiFli Bluetooth subsystem entirely through the RT-Thread standard device framework (`rt_device_find` / `rt_device_control` + event callbacks), instead of calling the underlying `bt_interface_*` protocol stack APIs directly.

SiFli SDK registers Bluetooth as an RT-Thread miscellaneous device named `bt_device`. This device is special in that it only implements the `control()` operation. Opening, closing, registering event callbacks, querying status, and performing actions for each profile are all done through `rt_device_control` with different `BT_CONTROL_*` command codes. Asynchronous results are reported through a registered `bt_notify_cb` callback as `BT_EVENT_*` events.

On this basis, the Bluetooth application is organized as a multi-service "core + plugins" framework: the core has been extracted into an independent middleware component, `rt_bt_app_core` (located in `middleware/rt_bt_app_core/`), which can be reused by any project. Adding a new Bluetooth profile (HFP, SPP, etc.) only requires adding one `.c` file and one configuration option on the application side, without modifying the core or other services.

## Supported Platforms
+ sf32lb52 series
+ sf32lb56 series
+ sf32lb58 series

## 1. Directory Structure

The core framework is an independent middleware component. The example project keeps only the entry point and the service plugins for each profile:

```text
middleware/rt_bt_app_core/       [core framework, independent reusable middleware component]
├── rt_bt_app.h              framework contract: service descriptor / command table / registration macros / interfaces
├── rt_bt_app_core.c         device management, event thread and queue, built-in common event handling, service registry
├── rt_bt_app_cmd.c          command router for all services
├── Kconfig                  BT_USING_RT_BT_APP_CORE and its sub-options
└── SConscript

example/rt_device/bt/
├── project/                 project directory
├── src/
│   ├── main.c               entry point: initialize the core + enable the protocol stack
│   └── services/
│       ├── SConscript       selects which service files to compile based on BT_USING_XXX macros
│       ├── bt_srv_hfp.c     HFP service (complete implementation)
│       └── bt_srv_spp.c     SPP service skeleton (teaching template, demonstrates a simple framework, not full SPP functionality)
└── README.md
```

- **Core**: middleware component `rt_bt_app_core` (`rt_bt_app_core.c` / `rt_bt_app_cmd.c`). It holds the `bt_device` handle, runs the event service thread, deep-copies events before routing them to the appropriate service by the high byte of the event code, handles common events internally (stack ready / inquiry / connection / disconnection), and provides the top-level `bt` shell command. **The core does not need to change when adding a new profile.**
- **Service**: `services/bt_srv_xxx.c`. Each profile has one file, and each file is a self-registering service that provides three things: a command table, an event handler, and an optional event deep-copy hook.

---

## 2. How the Framework Works

### 1. How events are routed to the correct service

The high byte of each profile's event code is fixed (see `bt_device.h`):

| High byte | Macro | Event group |
|:---|:---|:---|
| 0x40 | `BT_COMMON_TYPE_ID` | Common events (stack ready / inquiry / connection / disconnection), handled internally by the core |
| 0x41 | `BT_HF_TYPE_ID` | HFP |
| 0x46 | `BT_SPP_TYPE_ID` | SPP |
| ... | ... | ... |

After the core receives an event, it uses `event >> 8` to get the high byte, finds the service whose `event_group` matches it, and calls that service's `on_event`. If no service claims it and the high byte is `BT_COMMON_TYPE_ID`, the core falls back to its built-in common event handling.

### 2. How commands are routed to the correct service

The top-level command is `bt`, with three levels of routing:

```text
bt                          list all registered services
bt <service>                list the subcommands of one service
bt <service> <cmd> [args]   execute a subcommand
```

`rt_bt_app_cmd.c` is a generic router and does not know any specific profile. It first finds the service by name, then finds the subcommand by name in that service's command table, checks whether the protocol stack needs to be ready, and finally calls the `handler` registered in the command table.

---

## 3. How to Add a New Bluetooth Service

Taking a new `xxx` service as an example, there are three steps:

1. Create `bt_srv_xxx.c` under `src/services/`, implement the following parts, and register it automatically:
   - **Command wrapper**: convert actions into `rt_bt_app_control(BT_CONTROL_XXX, &arg)`;
   - **Command table** `rt_bt_cmd_entry_t[]`: map shell subcommands to the wrappers above;
   - **Event handler** `xxx_on_event`: handle the profile's `BT_EVENT_*` events;
   - **Deep-copy hook** `xxx_clone` (optional, required when event parameters contain pointers);
   - End with `RT_BT_SERVICE_REGISTER(xxx_service);` for self-registration.
2. Add one line to `src/services/SConscript` to gate compilation with a driver macro:
   ```python
   if GetDepend('BT_USING_XXX'):
       src += ['bt_srv_xxx.c']
   ```
3. Enable the corresponding `CONFIG_BT_USING_XXX=y` in menuconfig / `proj.conf`.

You can copy `bt_srv_spp.c` as a starting skeleton.

> The service depends only on the framework contract provided by `rt_bt_app.h`. Including it gives you access to interfaces such as `rt_bt_service_t`, `rt_bt_cmd_entry_t`, `rt_bt_app_control()`, and `RT_BT_SERVICE_REGISTER()`.

---

## 4. Configuration and Dependencies

The core framework is controlled by the Kconfig option `BT_USING_RT_BT_APP_CORE` and depends on `BSP_BLE_SIBLES` and `BT_FINSH`. This example's `proj.conf` already enables:

```text
CONFIG_BT_USING_RT_BT_APP_CORE=y   # enable the core framework component
CONFIG_BT_APP_ENABLE_SHELL_CMD=y   # enable the "bt" shell command (optional)
```

## 5. Build and Flash

In the project directory (`example/rt_device/bt/project`):

```text
scons --board=sf32lb56-lcd_a128r12n1 -j8
```

The build artifacts are generated under the `build_<board_name>` directory. Enter that directory and run `uart_download.bat` to flash the firmware as prompted:

```text
build_sf32lb56-lcd_a128r12n1_hcpu\uart_download.bat
```

---

## 6. Expected Results

### 1. Boot log

After reset, you should see the following key messages:

```text
BT device "bt_device" found          # rt_device_find succeeded
service "hfp" registered (group 0x41) # HFP service self-registered
...
BT stack ready                        # protocol stack is ready (the core then automatically powers on and sets the local name)
```

### 2. Command overview

```text
bt                    # list all services
bt hfp                # list all HFP subcommands
```

### 3. HFP subcommands

| Command | Description |
|:---|:---|
| `bt hfp c` | Delete all paired devices (uses the connection management interface, not a device framework command) |
| `bt hfp start_inquiry` | Search for nearby Bluetooth devices (no device-class restriction; search all devices) |
| `bt hfp stop_inquiry` | Stop searching |
| `bt hfp hfp_connect <addr>` | Establish an HFP connection with a phone (address format `xx:xx:xx:xx:xx:xx`, byte order matches the `device addr` in the inquiry log) |
| `bt hfp hfp_disconnect` | Disconnect the HFP connection |
| `bt hfp local_phone_number` | Query the local phone number (subscriber number) |
| `bt hfp remote_calls_info` | Query the remote current call list (CLCC) |
| `bt hfp remote_calls_status` | Query the remote call status |
| `bt hfp make_call <number>` | Place a call |
| `bt hfp call_back` | Redial the last number |
| `bt hfp answer_call` | Answer an incoming call |
| `bt hfp handup_call` | Hang up the current call |
| `bt hfp volume_control <0-15>` | Set the call speaker volume |
| `bt hfp audio_connect` | Establish the call audio (SCO) link |
| `bt hfp audio_disconnect` | Disconnect the call audio (SCO) link |
| `bt hfp battery_update <0-9>` | Report the local battery level through HFP |

### 4. Typical verification flow (HFP)

Since this device will be discoverable, the most direct approach is to have a phone connect proactively, or connect directly using a known address:

```text
bt hfp start_inquiry                 # search; the log prints the device address
bt hfp hfp_connect 22:5a:46:6d:42:e0 # connect using the address from the log
                                     # on success, prints "profile 0x0 connected"
bt hfp make_call 10086               # dial
bt hfp answer_call                   # can be controlled locally after the other side answers
bt hfp handup_call                   # hang up
```