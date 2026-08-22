# BT HFP Relay 示例

源码路径：`example/bt/hfp_relay`

{#Platform_bt_hfp_relay}
## 支持的平台

+ eh-lb52x
+ eh-lb56x
+ eh-lb58x
+ sf32lb52-lcd系列
+ sf32lb56-lcd系列
+ sf32lb58-lcd系列

## 概述

本例程演示 HFP（Hands-Free Profile）通话中继功能。设备同时运行两种 HFP 角色：

+ **HF（Hands-Free）角色**：作为免提设备连接手机（手机侧为 AG）。
+ **AG（Audio Gateway）角色**：作为音频网关接受蓝牙耳机（耳机侧为 HF）连接。

当手机和蓝牙耳机分别与开发板建立 HFP 连接后，开发板会将手机侧的通话状态、来电号码、信号/电量等指示信息同步给耳机；同时将耳机发起的拨号、接听、挂断、DTMF 按键、音量调节、电池状态更新等请求转发至手机，实现手机与耳机之间的 HFP 控制中继。

> 本例程用于验证 HFP 控制与状态信息的中继流程。实际通话音频的建立和传输依赖对端设备及蓝牙协议栈的协商结果。

## 例程的使用

### 连接拓扑

运行例程需要以下三个设备：

1. 一块烧录本例程的开发板；
2. 一部支持 HFP 的手机；
3. 一副支持 HFP 的蓝牙耳机。

连接关系如下。开发板同时承担两种 HFP 角色：对手机是 **HF**，对蓝牙耳机是 **AG**。

```text
手机（AG） <--HFP--> 开发板（HF/AG Relay） <--HFP--> 蓝牙耳机（HF）
```

开发板负责在手机和蓝牙耳机之间转发通话状态、号码、音量和控制请求。

简单的交互关系如下：

+ 手机 → 耳机：来电状态、通话状态、号码等信息同步。
+ 耳机 → 手机：接听、挂断、拨号、DTMF、音量调节等控制请求转发。

蓝牙协议栈启动完成后，例程会设置开发板本地蓝牙名称。默认名称为 `sifli_hfp_relay_service`；如工程配置中定义了 `BT_DEVICE_NAME`，则使用该配置的名称。

### 操作步骤

例程启动后默认使能蓝牙，并注册 FINSH 命令 `hfp_cmd`。以下命令中 `[addr]` 为设备蓝牙地址，格式为 `xx:xx:xx:xx:xx:xx`；`[phone_number]` 为待拨打的电话号码。

1. **清除历史配对（可选）**

   首次测试或需要重新配对时，执行以下命令清除已保存的配对设备：
   ```shell
   hfp_cmd c
   ```

2. **搜索手机和蓝牙耳机**

   执行搜索命令后，例程会搜索手机类设备和音频类设备。搜索到设备时，串口会打印设备名称、COD 和蓝牙地址；必要时可使用 `hfp_cmd stop_inquiry` 停止搜索。
   ```shell
   hfp_cmd start_inquiry
   hfp_cmd stop_inquiry
   ```

   典型搜索日志如下，其中 `addr` 即后续连接命令使用的 `[addr]`：
   ```text
   device <name> searched
   device COD is <cod>, addr is xx:xx:xx:xx:xx:xx
   ```

3. **连接手机（HF 角色）**

   开发板以 HF 角色连接手机。将搜索到的手机地址填入命令：
   ```shell
   hfp_cmd hfp_connect [手机蓝牙地址]
   ```

   也可以在手机侧搜索并主动连接开发板。首次连接时请按手机提示完成配对。连接成功时串口打印：
   ```text
   HFP HF connected
   ```

4. **连接蓝牙耳机（AG 角色）**

   开发板以 AG 角色接受耳机连接。可让耳机搜索并连接开发板 `sifli_hfp_relay_service`，也可将搜索到的耳机地址填入命令，由开发板主动发起连接：
   ```shell
   hfp_cmd hfp_ag_connect [耳机蓝牙地址]
   ```

   连接成功时串口打印：
   ```text
   HFP AG connected
   ```

5. **验证手机到耳机的信息同步**

   在手机侧发起来电、拨打电话或接听通话，观察耳机是否收到来电号码和通话状态变化。手机侧的信号、电量等指示信息也会同步给耳机。需要主动查询手机信息时，可执行：
   ```shell
   hfp_cmd local_phone_number
   hfp_cmd remote_calls_info
   hfp_cmd remote_calls_status
   ```

   查询本机号码成功时打印 `the remote phone local number:<号码>`；通话状态变化会打印 `the remote phone call_status`、`callsetup_status` 和 `callheld_status`。

6. **验证耳机到手机的通话控制**

   在耳机侧执行接听、挂断、拨号、发送 DTMF 按键或调节通话音量等操作，确认请求已转发至手机并生效。也可以通过 FINSH 命令直接验证手机侧 HFP 控制：
   ```shell
   hfp_cmd make_call [phone_number]
   hfp_cmd call_back
   hfp_cmd answer_call
   hfp_cmd handup_call
   hfp_cmd dtmf_key [0-9#*ABCD]
   hfp_cmd volume_control [0-15]
   hfp_cmd voice_recognition [0|1]
   hfp_cmd battery_update [0-9]
   ```

   拨号、回拨、接听、挂断和音量调节的结果分别会打印 `make a call complete`、`make a callback complete`、`answer a call complete`、`hangup a call complete` 和 `change volume value complete`。

7. **验证通话音频链路**

   在通话过程中可通过以下命令建立或断开 SCO 音频链路：
   ```shell
   hfp_cmd audio_connect
   hfp_cmd audio_disconnect
   ```

   通话音频链路建立或断开时，串口分别打印：
   ```text
   HFP HF audio_connected
   HFP HF audio_disconnected
   ```

8. **断开 HFP 连接**

   按设备地址断开对应 HFP 连接：
   ```shell
   hfp_cmd hfp_disconnect [设备蓝牙地址]
   ```

   断开手机或耳机后，串口分别打印 `HFP HF disconnected` 或 `HFP AG disconnected`。

> 使用中继功能时，需要手机侧的 HF 连接和耳机侧的 AG 连接均已建立。通过 `hfp_cmd` 直接执行的通话控制命令作用于手机侧 HF 连接。

### 典型功能验证

| 测试项 | 操作 | 预期现象 |
|:---|:---|:---|
| 来电状态同步 | 手机收到来电 | 耳机收到来电状态；串口打印手机侧通话状态更新。 |
| 接听/挂断 | 在耳机侧接听或挂断 | 请求转发到手机，手机通话状态随之更新。 |
| 耳机拨号 | 在耳机侧发起拨号 | 拨号号码转发到手机执行。 |
| DTMF 按键 | 通话中在耳机侧发送按键 | 按键转发给手机。 |
| 音量同步 | 在手机或耳机侧调节通话音量 | 对端收到相应音量更新；启用 Audio Manager 时同步更新蓝牙语音音量。 |
| 信息查询 | 耳机查询运营商/通话/本机号码信息 | 开发板从手机侧获取并回复耳机。 |

### 硬件需求

运行本例程前，需要准备：

+ 一块本例程支持的开发板（[支持的平台](#Platform_bt_hfp_relay)）。
+ 一部支持 HFP 的手机。
+ 一副支持 HFP 的蓝牙耳机。

### menuconfig 配置

工程的 `project/proj.conf` 已包含所需默认配置。若手动创建或调整工程，请确认以下选项已使能：

1. 使能蓝牙（`BLUETOOTH`）：
   - 路径：`Sifli middleware → Bluetooth`
   - 开启：`Enable bluetooth`
     - 宏开关：`CONFIG_BLUETOOTH`
     - 作用：使能蓝牙功能。
2. 使能 HFP HF 和 AG 角色：
   - 路径：`Sifli middleware → Bluetooth → Bluetooth service → Classic BT service`
   - 开启：`Enable BT finsh`（可选）
     - 宏开关：`CONFIG_BT_FINSH`
     - 作用：使能 FINSH 蓝牙控制命令。
   - 开启：`Manually select profiles`
     - 宏开关：`CONFIG_BT_PROFILE_CUSTOMIZE`
     - 作用：允许手动选择经典蓝牙 Profile。
   - 开启：`Enable Handsfree HF`
     - 宏开关：`CONFIG_CFG_HFP_HF`
     - 作用：使开发板以 HF 角色连接手机。
   - 开启：`Enable Handsfree AG`
     - 宏开关：`CONFIG_CFG_HFP_AG`
     - 作用：使开发板以 AG 角色服务蓝牙耳机。
3. 自动回连（可选）：
   - 宏开关：`CONFIG_BT_AUTO_CONNECT_LAST_DEVICE`
   - 作用：启动后自动连接最近一次配对的设备。
4. 音频功能：
   - 路径：`Sifli middleware`
   - 开启：`Enable Audio`
     - 宏开关：`CONFIG_AUDIO`
     - 作用：提供蓝牙语音相关音频能力。

### 编译和烧录

切换到例程 `project` 目录，运行 SCons 编译。例如：

```shell
> scons --board=eh-lb525 -j32
```

切换至例程的 `project/build_xx` 目录，运行 `uart_download.bat`，按提示选择串口进行下载：

```shell
> uart_download.bat
```

关于编译、下载的详细步骤，请参考[快速入门](/quickstart/get-started.md)。

## 例程的预期结果

当手机和蓝牙耳机均成功连接至开发板后：

+ 串口依次打印 `HFP HF connected` 和 `HFP AG connected`。
+ 手机来电、拨号及通话状态变化可同步至耳机。
+ 耳机发起的接听、挂断、拨号、DTMF 和音量控制请求可转发至手机。
+ 建立或断开 SCO 通话音频链路时，串口打印相应的 `audio_connected` 或 `audio_disconnected` 日志。

## 异常诊断

1. **手机或耳机无法连接**：确认两端均支持 HFP，删除历史配对记录后重新配对，并确认 `CONFIG_CFG_HFP_HF` 与 `CONFIG_CFG_HFP_AG` 已同时使能。
2. **仅一端连接成功**：HFP 中继需要手机侧 HF 连接和耳机侧 AG 连接同时建立；请根据串口中的 `HFP HF connected`、`HFP AG connected` 日志确认连接状态。
3. **通话控制未生效**：确认手机 HFP 连接未断开，并检查串口是否出现 `HFP HF disconnected`。部分手机或耳机对拨号、号码查询、语音识别等可选 HFP 特性的支持存在差异。
4. **无法自动回连**：先完成一次成功配对，并确认已启用 `CONFIG_BT_AUTO_CONNECT_LAST_DEVICE`。

## 参考文档

+ [BT HFP HF 示例](../hfp/README.md)
+ [快速入门](/quickstart/get-started.md)

## 更新记录

| 版本 | 日期 | 发布说明 |
|:---|:---|:---|
| 0.0.1 | 07/2026 | 初始版本 |