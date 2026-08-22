# HFP Relay 功能说明

HFP Relay 用于在手机和耳机之间中继 HFP 通话能力。中继设备同时维护两条 HFP 链路：手机侧作为 **HFP HF** 连接手机，耳机侧作为 **HFP AG** 连接耳机，从而让耳机可以通过中继设备使用手机的通话服务。

核心能力包括：

- **双角色桥接**：手机侧使用 HFP HF，耳机侧使用 HFP AG，完成手机和耳机之间的角色转换。
- **手机信息同步**：同步通话状态、CIND indicator、本机号码、CLCC 通话列表、音量、in-band ring 等信息。
- **耳机控制转发**：将耳机侧拨号、接听、挂断、DTMF、音量、电量和扩展 AT 命令转发到手机侧执行。
- **SCO 语音中继**：手机侧 SCO 建立后联动耳机侧 SCO，并通过 voice relay 转发通话语音。
- **异常状态清理**：连接断开、通话结束或 SCO 断开时清理本地缓存，避免耳机侧显示过期状态。

## 1. 整体架构

### 1.1 角色关系

| 链路 | 中继设备角色 | 对端设备 | 作用 |
| --- | --- | --- | --- |
| 手机侧 | HFP HF | 手机 / HFP AG | 接收手机通话状态、号码、CLCC、音量等信息；向手机发送拨号、接听、挂断等 AT 命令。 |
| 耳机侧 | HFP AG | 耳机 / HFP HF | 向耳机同步手机状态；接收耳机侧控制请求。 |
| 音频侧 | SCO relay | 手机与耳机 | 在 `CFG_BT_VOICE_RELAY` 开启时联动两侧 SCO/eSCO，完成通话语音转发。 |

```text
手机 / HFP AG                 中继设备                         耳机 / HFP HF
    │                       HFP HF + HFP AG                         │
    │  通话状态 / CIND / CLCC / 号码 / 音量                          │
    ├──────────────────────────►│                                  │
    │                           │  手机信息同步                     │
    │                           ├─────────────────────────────────►│
    │  AT 控制：拨号 / 接听 / 挂断 / DTMF / 音量                      │
    │◄──────────────────────────┤                                  │
    │                           │  耳机控制请求                     │
    │                           │◄─────────────────────────────────┤
    │◄──────────── SCO/eSCO voice relay ───────────────────────────►│
```

### 1.2 事件入口

Demo 的 `main.c` 收到蓝牙 notify 后，按事件类型分发给 relay manager：

```c
BT_NOTIFY_COMMON  -> bt_hfp_relay_handle_sco_event()
BT_NOTIFY_HFP_HF  -> bt_hfp_relay_hf_event_handle()
BT_NOTIFY_HFP_AG  -> bt_hfp_relay_ag_event_handle()
```

- `BT_NOTIFY_HFP_HF`：处理手机侧 HFP HF 事件。
- `BT_NOTIFY_HFP_AG`：处理耳机侧 HFP AG 事件。
- `BT_NOTIFY_COMMON`：处理 SCO/eSCO 公共事件。

## 2. 核心数据结构

### 2.1 `bt_hfp_relay_notify_data_t`

```c
typedef struct
{
    uint16_t type;
    uint16_t event_id;
    uint16_t data_len;
    uint8_t *data;
} bt_hfp_relay_notify_data_t;
```

用于把蓝牙 notify 事件传入 HFP relay manager。

| 字段 | 说明 |
| --- | --- |
| `type` | notify 类型，例如 `BT_NOTIFY_HFP_HF`、`BT_NOTIFY_HFP_AG`。 |
| `event_id` | 具体事件 ID。 |
| `data_len` | 事件数据长度。 |
| `data` | 事件数据指针，实际结构由 `event_id` 决定。 |

### 2.2 `bt_hfp_relay_context_t`

```c
typedef struct
{
    uint16_t hf_channel;
    uint16_t ag_channel;
    hfp_cind_status_t cind_status;
    hfp_phone_number_t local_phone_num;
    hfp_phone_call_info_t remote_call;
    hfp_remote_calls_info_t remote_calls;
    uint8_t has_local_phone_num;
    uint8_t has_remote_call;
    uint8_t pending_ag_make_call;
    uint8_t phone_vgs;
    uint8_t phone_vgm;
    uint8_t phone_bsir;
    uint8_t phone_bvra;
    uint8_t hfp_ag_sco_state;
    bt_notify_device_mac_t hf_mac;
    bt_notify_device_mac_t ag_mac;
} bt_hfp_relay_context_t;
```

模块通过全局静态变量 `g_hfp_relay_ctx` 保存运行时状态。

| 字段 | 说明 |
| --- | --- |
| `hf_channel` / `hf_mac` | 手机侧 HFP HF channel 与手机地址。 |
| `ag_channel` / `ag_mac` | 耳机侧 HFP AG channel 与耳机地址。 |
| `cind_status` | 手机侧 CIND 状态缓存，包括 service、call、callsetup、callheld、battery、signal、roam。 |
| `local_phone_num` / `has_local_phone_num` | 手机本机号码缓存及有效标志。 |
| `remote_call` / `remote_calls` / `has_remote_call` | CLCC 通话信息缓存及有效标志。 |
| `pending_ag_make_call` | 耳机侧拨号请求已转发，等待手机侧 ATD 确认。 |
| `phone_vgs` / `phone_vgm` | 手机侧 speaker/microphone gain。 |
| `phone_bsir` / `phone_bvra` | 手机侧 in-band ring 和语音识别状态。 |
| `hfp_ag_sco_state` | 耳机侧 AG SCO 连接状态。 |

## 3. 连接管理

连接管理负责维护两侧 HFP channel 和 MAC，是信息转发、控制转发和 SCO 联动的基础。relay manager 不在主动连接 API 调用时直接修改上下文，而是在协议栈上报连接成功或断开成功事件后更新本地状态，避免本地状态与真实链路不一致。

### 3.1 手机侧 HFP HF 连接

手机侧链路用于访问手机通话能力。连接成功后，中继设备才能接收手机通话状态、号码、CLCC、音量等信息，也才能把耳机侧控制命令转发给手机。

主动连接/断开接口通常由上层连接管理流程调用：

```c
bt_interface_conn_ext((unsigned char *)&mac, BT_PROFILE_HFP);
bt_interface_disc_ext((unsigned char *)&mac, BT_PROFILE_HFP);
```

连接成功流程：

```text
bt_interface_conn_ext(..., BT_PROFILE_HFP)
    -> 协议栈连接手机侧 HFP profile
    -> BT_NOTIFY_HF_PROFILE_CONNECTED
    -> bt_hfp_relay_set_hf_channel(channel, mac)
    -> 缓存 hf_channel / hf_mac
    -> 主动查询并缓存手机本机号码
```

断开流程：

```text
BT_NOTIFY_HF_PROFILE_DISCONNECTED
    -> bt_hfp_relay_clear_hf_channel(channel)
    -> 清除 hf_channel
    -> 清理本机号码、通话缓存和 pending 拨号状态
```

### 3.2 耳机侧 HFP AG 连接

耳机侧链路用于让耳机接入中继设备。连接成功后，中继设备可以向耳机同步手机信息，并接收耳机侧控制请求。

主动连接接口：

```c
bt_interface_conn_to_source_ext((unsigned char *)&profile_info->mac, BT_PROFILE_HFP);
```

连接成功流程：

```text
bt_interface_conn_to_source_ext(..., BT_PROFILE_HFP)
    -> 协议栈连接耳机侧 HFP profile
    -> BT_NOTIFY_AG_PROFILE_CONNECTED
    -> bt_hfp_relay_set_ag_channel(channel, mac)
    -> 缓存 ag_channel / ag_mac
    -> bt_interface_conn_to_source_ext(mac, BT_PROFILE_AVRCP)
```

断开流程：

```text
BT_NOTIFY_AG_PROFILE_DISCONNECTED
    -> bt_hfp_relay_clear_ag_channel(channel)
    -> 清除 ag_channel 和 pending 拨号状态
```

## 4. SCO 语音中继

HFP AT 事件只负责通话控制，真实语音通过 SCO/eSCO 传输。SCO 中继以手机侧 SCO 为主链路：手机侧 SCO 建立后，relay manager 打开 voice relay，并确保耳机侧 SCO 建立；手机侧 SCO 断开后，关闭 voice relay，并释放耳机侧 SCO。

### 4.1 SCO 建立

```text
BT_NOTIFY_COMMON / SCO connected
    -> bt_hfp_relay_handle_sco_event(event_id, sco_info)
    -> hfp_audio_relay_option(&sco_info->para, 1)
    -> 如果是手机侧 HF SCO connected，且耳机侧 AG SCO 未连接：
       -> bt_hfp_connect_audio(ag_mac)
    -> 如果是耳机侧 AG SCO connected：
       -> hfp_ag_sco_state = 1
```

`sco_info->para` 携带 SCO/eSCO 音频参数，底层 voice relay 根据这些参数配置语音转发通路。

### 4.2 SCO 断开

```text
BT_NOTIFY_COMMON / SCO disconnected
    -> bt_hfp_relay_handle_sco_event(event_id, sco_info)
    -> hfp_audio_relay_option(&sco_info->para, 0)
    -> 如果是手机侧 HF SCO disconnected，且耳机侧 AG SCO 已连接：
       -> bt_hfp_disconnect_audio(ag_mac)
    -> 如果是耳机侧 AG SCO disconnected：
       -> hfp_ag_sco_state = 0
```

SCO 功能可以概括为：**手机侧 SCO 决定 voice relay 的开启和关闭，relay manager 负责补齐或释放耳机侧 SCO，实际语音数据由底层 voice relay 通道转发。**

## 5. 手机信息同步

手机侧 HFP HF 事件主要用于把手机状态同步给耳机侧。relay manager 会先缓存必要状态，再根据耳机侧需要进行通知或响应。

```text
手机侧 HFP HF 事件 -> relay 本地缓存 -> 耳机侧 HFP AG 通知/响应
```

### 5.1 通话状态同步

通话状态同步是耳机显示和控制通话的基础。手机侧上报的 call、callsetup、callheld 等状态会被缓存到 `cind_status`，再转换为耳机侧 HFP AG 通话状态通知。

该功能支持耳机侧感知：

- 来电、拨号、等待接听等 callsetup 状态。
- 通话中 active call 状态。
- 保持或三方通话状态。
- 通话结束后的状态清理。

```text
BT_NOTIFY_HF_CALL_STATUS_UPDATE
    -> bt_hfp_relay_update_cind_from_all_status(call_status)
    -> 更新 g_hfp_relay_ctx.cind_status
    -> 如果 call/callsetup/callheld 全为 0：bt_hfp_relay_reset_cached_call()
    -> bt_hfp_relay_notify_ag_call_state()
    -> bt_interface_phone_state_changed(call_info)
```

字段映射关系：

| `bt_notify_all_call_status` 字段 | `hfp_cind_status_t` 字段 |
| --- | --- |
| `call_status` | `call` |
| `callsetup_status` | `callsetup` |
| `callheld_status` | `callheld` |
| `service` | `service_status` |
| `signal` | `signal` |
| `batt_level` | `batt_level` |
| `roam` | `roam_status` |

### 5.2 单个 CIND indicator 同步

当手机侧只上报单个 indicator 变化时，relay manager 更新对应字段，并通知耳机侧。

```text
BT_NOTIFY_HF_INDICATOR_UPDATE
    -> bt_hfp_relay_update_cind_by_indicator(type, val)
    -> bt_interface_indicator_status_changed(ag_channel, ind_info)
```

| Indicator type | 更新字段 |
| --- | --- |
| `HFP_AG_CIND_SERVICE_TYPE` | `service_status` |
| `HFP_AG_CIND_CALL_TYPE` | `call` |
| `HFP_AG_CIND_CALLSETUP_TYPE` | `callsetup` |
| `HFP_AG_CIND_BATT_TYPE` | `batt_level` |
| `HFP_AG_CIND_SIGNAL_TYPE` | `signal` |
| `HFP_AG_CIND_ROAM_TYPE` | `roam_status` |
| `HFP_AG_CIND_CALLHELD_TYPE` | `callheld` |

### 5.3 CLCC 通话列表查询与转发

CLCC 用于把手机当前通话列表返回给耳机。耳机不直接连接手机，因此耳机请求 CLCC 时，relay manager 会向手机侧查询，缓存手机返回的通话信息，再返回给耳机。

CLCC 查询状态：

| 状态 | 说明 |
| --- | --- |
| `START` | 已启动手机端 CLCC 查询，等待定时器触发实际查询。 |
| `IN_PROGRESS` | 已收到至少一条手机端 CLCC 信息，正在刷新缓存。 |
| `COMPLETE` | 手机端 CLCC 查询完成，可将结果返回耳机。 |

查询流程：

```text
BT_NOTIFY_AG_GET_ALL_REMT_CALLS_INFO_REQ
    -> bt_hfp_relay_start_get_clcc()
    -> 定时器触发后向手机查询通话信息
    -> BT_NOTIFY_HF_REMOTE_CALL_INFO_IND
       -> bt_hfp_relay_cache_remote_call(clcc_info)
    -> HFP_HF_AT_CLCC confirm
       -> bt_hfp_relay_handle_at_cmd_cfm(HFP_HF_AT_CLCC, res)
       -> bt_interface_remote_call_info_res(ag_channel, &remote_calls)
```

当前实现只缓存一路 `remote_call`。如果手机返回多路 CLCC，后续记录会覆盖前一条；如需完整支持多方通话，需要扩展为数组缓存。

### 5.4 本机号码查询与转发

本机号码用于耳机侧查询当前手机号码信息。手机侧 HF 连接成功后，relay manager 会主动查询手机号码并缓存；耳机后续查询时优先返回缓存，减少实时查询手机带来的等待。

如果缓存尚未准备好且手机侧 HF channel 有效，则继续向手机查询，待手机返回号码后再同步给耳机。

```text
手机侧 HF 连接成功
    -> bt_interface_get_ph_num_by_id(hf_channel)
    -> BT_NOTIFY_HF_LOCAL_PHONE_NUMBER
    -> bt_hfp_relay_cache_local_phone_num(number, number_len)

BT_NOTIFY_AG_GET_LOCAL_PHONE_INFO_REQ
    -> 如果已有缓存：bt_interface_local_phone_info_res(ag_channel, &local_phone_num)
    -> 如果无缓存但 hf_channel 有效：继续向手机查询
```

### 5.5 音量和其他手机状态

音量、in-band ring 和 AT 命令确认用于补齐通话过程中的辅助体验。relay manager 根据手机侧上报缓存或转发这些状态，使耳机侧的音量显示、响铃方式和控制结果反馈与手机保持一致。

| 手机侧事件 | 处理逻辑 |
| --- | --- |
| `BT_NOTIFY_HF_VOLUME_CHANGE` | 缓存手机侧 VGS，并通知耳机侧音量变化。 |
| `BT_NOTIFY_HF_INBAND_RING_STATUS_CHANGE` | 缓存手机侧 in-band ring 状态，用于判断来电铃声由手机带内传输还是耳机本地产生。 |
| `BT_NOTIFY_HF_AT_CMD_CFM` | 处理手机侧 AT 命令确认，例如 CLCC 查询确认、ATD 拨号确认。 |

## 6. 耳机侧控制请求转发

耳机侧控制请求用于让耳机通过中继设备控制手机通话。relay manager 收到耳机侧 HFP AG 事件后，先检查手机侧 HF channel，再转换为手机侧 HFP HF 接口调用。

```text
耳机侧 HFP AG 事件 -> relay manager -> 手机侧 HFP HF 接口 -> 手机 AT 命令
```

| 耳机侧事件 | 转发到手机侧接口 | 说明 |
| --- | --- | --- |
| `BT_NOTIFY_AG_MAKE_CALL_REQ` | `bt_interface_hf_out_going_call_by_id()` | 拨号。 |
| `BT_NOTIFY_AG_ANSWER_CALL_REQ` | `bt_interface_start_hf_answer_req_send_by_id()` | 接听来电。 |
| `BT_NOTIFY_AG_HANGUP_CALL_REQ` | `bt_interface_handup_call_by_id()` | 挂断通话或拒接来电。 |
| `BT_NOTIFY_AG_RECV_DTMF_KEY` | `bt_interface_start_dtmf_req_send_by_id()` | 通话中发送 DTMF。 |
| `BT_NOTIFY_AG_VOLUME_CHANGE` | `bt_interface_set_speaker_volume_by_id()` | 将耳机侧音量同步给手机。 |
| `BT_NOTIFY_AG_BATTERY_UPDATE` | `bt_interface_hf_update_battery_by_id()` | 将耳机侧电量同步给手机。 |
| `BT_NOTIFY_AG_EXTERN_AT_CMD_KEY_REQ` | `bt_interface_hf_send_at_cmd_by_id()` | 转发扩展 AT 命令。 |

### 6.1 拨打电话

耳机侧发起拨号时，relay manager 校验号码和手机侧 HF channel 后发送手机侧 `ATD` 拨号命令。拨号是异步流程，命令下发后会设置 `pending_ag_make_call`，等待手机侧 ATD confirm 后再把结果返回给耳机。

```text
BT_NOTIFY_AG_MAKE_CALL_REQ
    -> pending_ag_make_call = 1
    -> bt_interface_hf_out_going_call_by_id(hf_channel, len, number)
    -> HFP_HF_AT_ATD confirm
    -> pending_ag_make_call = 0
    -> bt_interface_make_call_res(ag_channel, res)
```

如果手机侧 HF channel 无效或号码为空，则直接向耳机返回失败。

### 6.2 接听电话

耳机侧接听来电时，relay manager 将请求转换为手机侧接听命令。接听后的通话状态和 SCO 音频链路仍以手机侧后续上报为准。

```text
BT_NOTIFY_AG_ANSWER_CALL_REQ
    -> bt_interface_start_hf_answer_req_send_by_id(hf_channel)
    -> 手机侧状态/SCO 事件同步接听结果
```

### 6.3 挂断电话

耳机侧挂断或拒接时，relay manager 向手机发送挂断命令。手机执行后会重新上报 call、callsetup、callheld 状态；当状态恢复空闲时，relay manager 清理通话缓存并通知耳机。

```text
BT_NOTIFY_AG_HANGUP_CALL_REQ
    -> bt_interface_handup_call_by_id(hf_channel)
    -> 手机侧通话状态更新
    -> 必要时清理本地通话缓存
```

### 6.4 DTMF 功能

DTMF 用于通话中发送按键信令，例如语音菜单输入。relay manager 只透传耳机侧按键和目标 channel，不改变通话状态。

```text
BT_NOTIFY_AG_RECV_DTMF_KEY
    -> bt_interface_start_dtmf_req_send_by_id(hf_channel, key)
```

### 6.5 扩展命令

扩展命令用于透传标准流程之外的 HFP AT 命令，例如厂商自定义命令。relay manager 不解析命令语义，只负责校验 channel 并转发到手机侧 HF 链路。

```text
BT_NOTIFY_AG_EXTERN_AT_CMD_KEY_REQ
    -> bt_interface_hf_send_at_cmd_by_id(hf_channel, at_cmd)
```

## 7. API 接口汇总

### 7.1 初始化与上下文接口

| API | 作用 | 说明 |
| --- | --- | --- |
| `bt_hfp_relay_mgr_init()` | 初始化 relay manager | 清零上下文，初始化 channel、默认 CIND 状态、CLCC 状态和通话列表指针。 |
| `bt_hfp_relay_get_context()` | 获取全局上下文 | 返回 `g_hfp_relay_ctx` 地址。 |
| `bt_hfp_relay_get_hf_channel()` | 获取手机侧 HF channel | 无效时返回 `BT_HFP_RELAY_INVALID_CHANNEL`。 |
| `bt_hfp_relay_get_ag_channel()` | 获取耳机侧 AG channel | 无效时返回 `BT_HFP_RELAY_INVALID_CHANNEL`。 |

`bt_hfp_relay_mgr_init()` 主要初始化行为：

- 清零 `g_hfp_relay_ctx`。
- 将 `hf_channel`、`ag_channel` 设置为 `BT_HFP_RELAY_INVALID_CHANNEL`。
- 设置默认 CIND 状态：`service_status = 1`、`batt_level = 5`、`signal = 5`。
- 设置 `remote_calls.calls = &g_hfp_relay_ctx.remote_call`。
- 将 CLCC 状态置为 `COMPLETE`。

### 7.2 事件入口接口

| API | 作用 | 返回值 |
| --- | --- | --- |
| `bt_hfp_relay_hf_event_handle(bt_hfp_relay_notify_data_t *msg)` | 处理手机侧 HFP HF 事件 | 固定返回 `0`。 |
| `bt_hfp_relay_ag_event_handle(bt_hfp_relay_notify_data_t *msg)` | 处理耳机侧 HFP AG 事件 | 成功返回 `0`，参数非法返回 `-RT_EINVAL`。 |
| `bt_hfp_relay_handle_sco_event(uint16_t event_id, bt_notify_device_sco_info_t *sco_info)` | 处理 SCO 事件 | 无返回值。 |

### 7.3 Channel 管理接口

| API | 作用 | 关键行为 |
| --- | --- | --- |
| `bt_hfp_relay_set_hf_channel(uint16_t channel, bt_notify_device_mac_t *mac)` | 设置手机侧 HF channel | 缓存 `hf_channel` 和 `hf_mac`。 |
| `bt_hfp_relay_clear_hf_channel(uint16_t channel)` | 清除手机侧 HF channel | channel 匹配时清除 HF channel、通话缓存、本机号码缓存和 pending 状态。 |
| `bt_hfp_relay_set_ag_channel(uint16_t channel, bt_notify_device_mac_t *mac)` | 设置耳机侧 AG channel | 缓存 `ag_channel` 和 `ag_mac`。 |
| `bt_hfp_relay_clear_ag_channel(uint16_t channel)` | 清除耳机侧 AG channel | channel 匹配时清除 AG channel 和 pending 状态。 |

### 7.4 状态与缓存接口

| API | 作用 | 关键行为 |
| --- | --- | --- |
| `bt_hfp_relay_reset_cached_call()` | 清除远端通话缓存 | 清空 `remote_call`，设置 `remote_calls.num_call = 0`。 |
| `bt_hfp_relay_cache_local_phone_num(uint8_t *number, uint16_t number_len)` | 缓存本机号码 | 最多复制 `PHONE_NUM_LEN - 1` 字节，并设置 `has_local_phone_num`。 |
| `bt_hfp_relay_cache_remote_call(bt_notify_clcc_ind_t *clcc_info)` | 缓存 CLCC 通话信息 | 当前只保存一路通话，并设置 `remote_calls.num_call = 1`。 |
| `bt_hfp_relay_update_cind_from_all_status(bt_notify_all_call_status *call_status)` | 同步完整 CIND 状态 | 根据手机侧通话状态更新 `cind_status`。 |
| `bt_hfp_relay_update_cind_by_indicator(uint8_t type, uint8_t val)` | 同步单个 CIND indicator | 根据 indicator 类型更新指定字段。 |
| `bt_hfp_relay_notify_ag_call_state()` | 通知耳机侧通话状态 | 将缓存状态转换为 `HFP_CALL_INFO_T` 后调用 `bt_interface_phone_state_changed()`。 |

### 7.5 CLCC 与 AT 确认接口

| API | 作用 | 关键行为 |
| --- | --- | --- |
| `bt_hfp_relay_start_get_clcc()` | 启动 CLCC 查询 | 创建或重启 one-shot soft timer，并切换 CLCC 状态。 |
| `bt_hfp_relay_handle_at_cmd_cfm(uint8_t cmd_id, uint8_t res)` | 处理手机侧 AT 命令确认 | 完成 CLCC 查询响应或拨号响应。 |

| `cmd_id` | 处理逻辑 |
| --- | --- |
| `HFP_HF_AT_CLCC` | CLCC 查询完成，通过 `bt_interface_remote_call_info_res()` 返回通话列表给耳机。 |
| `HFP_HF_AT_ATD` | 拨号确认完成，清除 `pending_ag_make_call`，通过 `bt_interface_make_call_res()` 返回拨号结果给耳机。 |
| 其他 AT 命令 | 当前主要打印确认结果日志。 |

## 8. 注意事项与建议

1. **初始化调用需确认**  
   建议确保 `bt_hfp_relay_mgr_init()` 在功能启动时被调用，用于初始化 channel、默认 CIND、CLCC 状态和通话列表指针。

2. **空指针防护不足**  
   部分内部接口未做完整空指针检查，例如 `bt_hfp_relay_hf_event_handle()`、`bt_hfp_relay_set_hf_channel()`、`bt_hfp_relay_set_ag_channel()`、`bt_hfp_relay_cache_local_phone_num()`、`bt_hfp_relay_cache_remote_call()`，调用方需要保证参数有效。

3. **多通话缓存能力有限**  
   当前只缓存一路 `remote_call`。如果要完整支持三方或多方通话，需要扩展为数组缓存。

4. **CLCC 查询接口语义需确认**  
   `bt_hfp_relay_clcc_timeout()` 在 `START` 状态调用 `bt_interface_get_ph_num_by_id(hf_channel)`。从流程看其意图是查询 CLCC，建议确认底层接口是否复用，或是否存在接口误用。

5. **上下文可写指针暴露**  
   `bt_hfp_relay_get_context()` 返回全局上下文可写指针，外部模块可直接修改内部状态。后续可考虑收敛为只读查询或专用 setter/getter。