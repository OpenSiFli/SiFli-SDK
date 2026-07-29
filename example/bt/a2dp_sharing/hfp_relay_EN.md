# HFP Relay Feature Description

HFP Relay is used to relay HFP call capabilities between a mobile phone and a headset. The relay device maintains two HFP links at the same time: on the phone side, it connects to the phone as an **HFP HF**; on the headset side, it connects to the headset as an **HFP AG**. This allows the headset to use the phone's call service through the relay device.

Core capabilities include:

- **Dual-role bridging**: The phone side uses HFP HF and the headset side uses HFP AG, completing role conversion between the phone and headset.
- **Phone information synchronization**: Synchronizes call status, CIND indicators, local phone number, CLCC call list, volume, in-band ring, and other information.
- **Headset control forwarding**: Forwards dialing, answering, hanging up, DTMF, volume, battery, and extended AT commands from the headset side to the phone side for execution.
- **SCO voice relay**: After phone-side SCO is established, headset-side SCO is linked accordingly, and call voice is forwarded through voice relay.
- **Exception-state cleanup**: Clears local caches when the connection is disconnected, the call ends, or SCO is disconnected, preventing stale status from being displayed on the headset side.

## 1. Overall Architecture

### 1.1 Role Relationship

| Link | Relay device role | Peer device | Purpose |
| --- | --- | --- | --- |
| Phone side | HFP HF | Phone / HFP AG | Receives phone call status, number, CLCC, volume, and other information; sends AT commands such as dialing, answering, and hanging up to the phone. |
| Headset side | HFP AG | Headset / HFP HF | Synchronizes phone status to the headset and receives control requests from the headset side. |
| Audio side | SCO relay | Phone and headset | When `CFG_BT_VOICE_RELAY` is enabled, links SCO/eSCO on both sides and forwards call voice. |

```text
Phone / HFP AG              Relay device                         Headset / HFP HF
    │                       HFP HF + HFP AG                         │
    │  Call status / CIND / CLCC / number / volume                  │
    ├──────────────────────────►│                                  │
    │                           │  Phone information sync           │
    │                           ├─────────────────────────────────►│
    │  AT control: dial / answer / hang up / DTMF / volume          │
    │◄──────────────────────────┤                                  │
    │                           │  Headset control request          │
    │                           │◄─────────────────────────────────┤
    │◄──────────── SCO/eSCO voice relay ───────────────────────────►│
```

### 1.2 Event Entry Points

After receiving Bluetooth notifications, the demo `main.c` dispatches them to the relay manager based on event type:

```c
BT_NOTIFY_COMMON  -> bt_hfp_relay_handle_sco_event()
BT_NOTIFY_HFP_HF  -> bt_hfp_relay_hf_event_handle()
BT_NOTIFY_HFP_AG  -> bt_hfp_relay_ag_event_handle()
```

- `BT_NOTIFY_HFP_HF`: Handles phone-side HFP HF events.
- `BT_NOTIFY_HFP_AG`: Handles headset-side HFP AG events.
- `BT_NOTIFY_COMMON`: Handles common SCO/eSCO events.

## 2. Core Data Structures

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

Used to pass Bluetooth notify events into the HFP relay manager.

| Field | Description |
| --- | --- |
| `type` | Notify type, such as `BT_NOTIFY_HFP_HF` or `BT_NOTIFY_HFP_AG`. |
| `event_id` | Specific event ID. |
| `data_len` | Event data length. |
| `data` | Pointer to event data. The actual structure is determined by `event_id`. |

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

The module stores runtime state in the global static variable `g_hfp_relay_ctx`.

| Field | Description |
| --- | --- |
| `hf_channel` / `hf_mac` | Phone-side HFP HF channel and phone address. |
| `ag_channel` / `ag_mac` | Headset-side HFP AG channel and headset address. |
| `cind_status` | Phone-side CIND status cache, including service, call, callsetup, callheld, battery, signal, and roam. |
| `local_phone_num` / `has_local_phone_num` | Cached local phone number and valid flag. |
| `remote_call` / `remote_calls` / `has_remote_call` | Cached CLCC call information and valid flag. |
| `pending_ag_make_call` | Indicates that a headset-side dialing request has been forwarded and is waiting for phone-side ATD confirmation. |
| `phone_vgs` / `phone_vgm` | Phone-side speaker/microphone gain. |
| `phone_bsir` / `phone_bvra` | Phone-side in-band ring and voice recognition status. |
| `hfp_ag_sco_state` | Headset-side AG SCO connection state. |

## 3. Connection Management

Connection management maintains HFP channels and MAC addresses on both sides. It is the foundation for information forwarding, control forwarding, and SCO linkage. The relay manager does not directly modify the context when an active connection API is called. Instead, it updates local state after the protocol stack reports a successful connection or disconnection event, preventing local state from becoming inconsistent with the actual link.

### 3.1 Phone-side HFP HF Connection

The phone-side link is used to access phone call capabilities. After the connection succeeds, the relay device can receive phone call status, number, CLCC, volume, and other information, and can also forward headset-side control commands to the phone.

Active connection/disconnection APIs are usually called by the upper-layer connection management flow:

```c
bt_interface_conn_ext((unsigned char *)&mac, BT_PROFILE_HFP);
bt_interface_disc_ext((unsigned char *)&mac, BT_PROFILE_HFP);
```

Connection success flow:

```text
bt_interface_conn_ext(..., BT_PROFILE_HFP)
    -> Protocol stack connects the phone-side HFP profile
    -> BT_NOTIFY_HF_PROFILE_CONNECTED
    -> bt_hfp_relay_set_hf_channel(channel, mac)
    -> Cache hf_channel / hf_mac
    -> Actively query and cache the local phone number
```

Disconnection flow:

```text
BT_NOTIFY_HF_PROFILE_DISCONNECTED
    -> bt_hfp_relay_clear_hf_channel(channel)
    -> Clear hf_channel
    -> Clear local phone number, call cache, and pending dialing state
```

### 3.2 Headset-side HFP AG Connection

The headset-side link allows the headset to access the relay device. After the connection succeeds, the relay device can synchronize phone information to the headset and receive control requests from the headset side.

Active connection API:

```c
bt_interface_conn_to_source_ext((unsigned char *)&profile_info->mac, BT_PROFILE_HFP);
```

Connection success flow:

```text
bt_interface_conn_to_source_ext(..., BT_PROFILE_HFP)
    -> Protocol stack connects the headset-side HFP profile
    -> BT_NOTIFY_AG_PROFILE_CONNECTED
    -> bt_hfp_relay_set_ag_channel(channel, mac)
    -> Cache ag_channel / ag_mac
    -> bt_interface_conn_to_source_ext(mac, BT_PROFILE_AVRCP)
```

Disconnection flow:

```text
BT_NOTIFY_AG_PROFILE_DISCONNECTED
    -> bt_hfp_relay_clear_ag_channel(channel)
    -> Clear ag_channel and pending dialing state
```

## 4. SCO Voice Relay

HFP AT events are only responsible for call control. Actual voice is transmitted through SCO/eSCO. SCO relay uses the phone-side SCO as the primary link: after phone-side SCO is established, the relay manager enables voice relay and ensures that headset-side SCO is established; after phone-side SCO is disconnected, it disables voice relay and releases headset-side SCO.

### 4.1 SCO Establishment

```text
BT_NOTIFY_COMMON / SCO connected
    -> bt_hfp_relay_handle_sco_event(event_id, sco_info)
    -> hfp_audio_relay_option(&sco_info->para, 1)
    -> If phone-side HF SCO is connected and headset-side AG SCO is not connected:
       -> bt_hfp_connect_audio(ag_mac)
    -> If headset-side AG SCO is connected:
       -> hfp_ag_sco_state = 1
```

`sco_info->para` carries SCO/eSCO audio parameters. The lower-level voice relay configures the voice forwarding path based on these parameters.

### 4.2 SCO Disconnection

```text
BT_NOTIFY_COMMON / SCO disconnected
    -> bt_hfp_relay_handle_sco_event(event_id, sco_info)
    -> hfp_audio_relay_option(&sco_info->para, 0)
    -> If phone-side HF SCO is disconnected and headset-side AG SCO is connected:
       -> bt_hfp_disconnect_audio(ag_mac)
    -> If headset-side AG SCO is disconnected:
       -> hfp_ag_sco_state = 0
```

The SCO function can be summarized as follows: **phone-side SCO determines whether voice relay is enabled or disabled; the relay manager is responsible for establishing or releasing headset-side SCO; actual voice data is forwarded by the lower-level voice relay channel.**

## 5. Phone Information Synchronization

Phone-side HFP HF events are mainly used to synchronize phone status to the headset side. The relay manager first caches necessary status, and then sends notifications or responses according to headset-side requirements.

```text
Phone-side HFP HF event -> relay local cache -> headset-side HFP AG notification/response
```

### 5.1 Call Status Synchronization

Call status synchronization is the basis for headset call display and control. The call, callsetup, callheld, and other states reported by the phone side are cached in `cind_status`, and then converted into headset-side HFP AG call status notifications.

This feature allows the headset side to detect:

- Incoming call, outgoing call, waiting-to-answer, and other callsetup states.
- Active call state during a call.
- Held call or three-way call state.
- State cleanup after a call ends.

```text
BT_NOTIFY_HF_CALL_STATUS_UPDATE
    -> bt_hfp_relay_update_cind_from_all_status(call_status)
    -> Update g_hfp_relay_ctx.cind_status
    -> If call/callsetup/callheld are all 0: bt_hfp_relay_reset_cached_call()
    -> bt_hfp_relay_notify_ag_call_state()
    -> bt_interface_phone_state_changed(call_info)
```

Field mapping:

| `bt_notify_all_call_status` field | `hfp_cind_status_t` field |
| --- | --- |
| `call_status` | `call` |
| `callsetup_status` | `callsetup` |
| `callheld_status` | `callheld` |
| `service` | `service_status` |
| `signal` | `signal` |
| `batt_level` | `batt_level` |
| `roam` | `roam_status` |

### 5.2 Single CIND Indicator Synchronization

When the phone side reports only a single indicator change, the relay manager updates the corresponding field and notifies the headset side.

```text
BT_NOTIFY_HF_INDICATOR_UPDATE
    -> bt_hfp_relay_update_cind_by_indicator(type, val)
    -> bt_interface_indicator_status_changed(ag_channel, ind_info)
```

| Indicator type | Updated field |
| --- | --- |
| `HFP_AG_CIND_SERVICE_TYPE` | `service_status` |
| `HFP_AG_CIND_CALL_TYPE` | `call` |
| `HFP_AG_CIND_CALLSETUP_TYPE` | `callsetup` |
| `HFP_AG_CIND_BATT_TYPE` | `batt_level` |
| `HFP_AG_CIND_SIGNAL_TYPE` | `signal` |
| `HFP_AG_CIND_ROAM_TYPE` | `roam_status` |
| `HFP_AG_CIND_CALLHELD_TYPE` | `callheld` |

### 5.3 CLCC Call List Query and Forwarding

CLCC is used to return the phone's current call list to the headset. Because the headset is not directly connected to the phone, when the headset requests CLCC, the relay manager queries the phone side, caches the call information returned by the phone, and then returns it to the headset.

CLCC query states:

| State | Description |
| --- | --- |
| `START` | Phone-side CLCC query has been started and is waiting for the timer to trigger the actual query. |
| `IN_PROGRESS` | At least one phone-side CLCC record has been received, and the cache is being refreshed. |
| `COMPLETE` | Phone-side CLCC query is complete, and the result can be returned to the headset. |

Query flow:

```text
BT_NOTIFY_AG_GET_ALL_REMT_CALLS_INFO_REQ
    -> bt_hfp_relay_start_get_clcc()
    -> After the timer triggers, query call information from the phone
    -> BT_NOTIFY_HF_REMOTE_CALL_INFO_IND
       -> bt_hfp_relay_cache_remote_call(clcc_info)
    -> HFP_HF_AT_CLCC confirm
       -> bt_hfp_relay_handle_at_cmd_cfm(HFP_HF_AT_CLCC, res)
       -> bt_interface_remote_call_info_res(ag_channel, &remote_calls)
```

The current implementation caches only one `remote_call`. If the phone returns multiple CLCC records, later records overwrite earlier ones. To fully support three-way or multi-party calls, this should be extended to an array cache.

### 5.4 Local Phone Number Query and Forwarding

The local phone number is used when the headset side queries the current phone number information. After phone-side HF is connected successfully, the relay manager actively queries and caches the phone number. Later headset queries preferentially return the cached value, reducing the wait caused by real-time phone queries.

If the cache is not ready and the phone-side HF channel is valid, the relay manager continues to query the phone and synchronizes the number to the headset after the phone returns it.

```text
Phone-side HF connected successfully
    -> bt_interface_get_ph_num_by_id(hf_channel)
    -> BT_NOTIFY_HF_LOCAL_PHONE_NUMBER
    -> bt_hfp_relay_cache_local_phone_num(number, number_len)

BT_NOTIFY_AG_GET_LOCAL_PHONE_INFO_REQ
    -> If cached: bt_interface_local_phone_info_res(ag_channel, &local_phone_num)
    -> If not cached but hf_channel is valid: continue querying the phone
```

### 5.5 Volume and Other Phone Status

Volume, in-band ring, and AT command confirmations complement the call experience. The relay manager caches or forwards these states reported by the phone side so that headset-side volume display, ringing behavior, and control-result feedback remain consistent with the phone.

| Phone-side event | Handling logic |
| --- | --- |
| `BT_NOTIFY_HF_VOLUME_CHANGE` | Caches phone-side VGS and notifies the headset side of the volume change. |
| `BT_NOTIFY_HF_INBAND_RING_STATUS_CHANGE` | Caches phone-side in-band ring status, used to determine whether the incoming-call ringtone is transmitted in-band by the phone or generated locally by the headset. |
| `BT_NOTIFY_HF_AT_CMD_CFM` | Handles phone-side AT command confirmations, such as CLCC query confirmation and ATD dialing confirmation. |

## 6. Headset-side Control Request Forwarding

Headset-side control requests allow the headset to control phone calls through the relay device. After receiving a headset-side HFP AG event, the relay manager first checks the phone-side HF channel, and then converts the event into a phone-side HFP HF interface call.

```text
Headset-side HFP AG event -> relay manager -> phone-side HFP HF interface -> phone AT command
```

| Headset-side event | Forwarded to phone-side interface | Description |
| --- | --- | --- |
| `BT_NOTIFY_AG_MAKE_CALL_REQ` | `bt_interface_hf_out_going_call_by_id()` | Dialing. |
| `BT_NOTIFY_AG_ANSWER_CALL_REQ` | `bt_interface_start_hf_answer_req_send_by_id()` | Answer an incoming call. |
| `BT_NOTIFY_AG_HANGUP_CALL_REQ` | `bt_interface_handup_call_by_id()` | Hang up a call or reject an incoming call. |
| `BT_NOTIFY_AG_RECV_DTMF_KEY` | `bt_interface_start_dtmf_req_send_by_id()` | Send DTMF during a call. |
| `BT_NOTIFY_AG_VOLUME_CHANGE` | `bt_interface_set_speaker_volume_by_id()` | Synchronize headset-side volume to the phone. |
| `BT_NOTIFY_AG_BATTERY_UPDATE` | `bt_interface_hf_update_battery_by_id()` | Synchronize headset-side battery level to the phone. |
| `BT_NOTIFY_AG_EXTERN_AT_CMD_KEY_REQ` | `bt_interface_hf_send_at_cmd_by_id()` | Forward extended AT commands. |

### 6.1 Making a Call

When the headset side initiates dialing, the relay manager validates the number and phone-side HF channel, and then sends the phone-side `ATD` dialing command. Dialing is asynchronous. After the command is issued, `pending_ag_make_call` is set, and the result is returned to the headset after the phone-side ATD confirmation is received.

```text
BT_NOTIFY_AG_MAKE_CALL_REQ
    -> pending_ag_make_call = 1
    -> bt_interface_hf_out_going_call_by_id(hf_channel, len, number)
    -> HFP_HF_AT_ATD confirm
    -> pending_ag_make_call = 0
    -> bt_interface_make_call_res(ag_channel, res)
```

If the phone-side HF channel is invalid or the number is empty, failure is returned to the headset directly.

### 6.2 Answering a Call

When the headset side answers an incoming call, the relay manager converts the request into a phone-side answer command. The call status and SCO audio link after answering are still determined by subsequent reports from the phone side.

```text
BT_NOTIFY_AG_ANSWER_CALL_REQ
    -> bt_interface_start_hf_answer_req_send_by_id(hf_channel)
    -> Phone-side status/SCO events synchronize the answer result
```

### 6.3 Hanging Up a Call

When the headset side hangs up or rejects a call, the relay manager sends a hang-up command to the phone. After execution, the phone reports call, callsetup, and callheld status again. When the state returns to idle, the relay manager clears the call cache and notifies the headset.

```text
BT_NOTIFY_AG_HANGUP_CALL_REQ
    -> bt_interface_handup_call_by_id(hf_channel)
    -> Phone-side call status update
    -> Clear local call cache if necessary
```

### 6.4 DTMF Function

DTMF is used to send key signals during a call, such as voice menu input. The relay manager only transparently forwards the headset-side key and target channel, without changing call state.

```text
BT_NOTIFY_AG_RECV_DTMF_KEY
    -> bt_interface_start_dtmf_req_send_by_id(hf_channel, key)
```

### 6.5 Extended Commands

Extended commands are used to transparently forward HFP AT commands outside the standard flow, such as vendor-specific commands. The relay manager does not parse command semantics; it only validates the channel and forwards the command to the phone-side HF link.

```text
BT_NOTIFY_AG_EXTERN_AT_CMD_KEY_REQ
    -> bt_interface_hf_send_at_cmd_by_id(hf_channel, at_cmd)
```

## 7. API Summary

### 7.1 Initialization and Context APIs

| API | Purpose | Description |
| --- | --- | --- |
| `bt_hfp_relay_mgr_init()` | Initializes the relay manager | Clears the context and initializes channels, default CIND status, CLCC state, and call-list pointer. |
| `bt_hfp_relay_get_context()` | Gets the global context | Returns the address of `g_hfp_relay_ctx`. |
| `bt_hfp_relay_get_hf_channel()` | Gets the phone-side HF channel | Returns `BT_HFP_RELAY_INVALID_CHANNEL` when invalid. |
| `bt_hfp_relay_get_ag_channel()` | Gets the headset-side AG channel | Returns `BT_HFP_RELAY_INVALID_CHANNEL` when invalid. |

Main initialization behavior of `bt_hfp_relay_mgr_init()`:

- Clears `g_hfp_relay_ctx`.
- Sets `hf_channel` and `ag_channel` to `BT_HFP_RELAY_INVALID_CHANNEL`.
- Sets the default CIND status: `service_status = 1`, `batt_level = 5`, and `signal = 5`.
- Sets `remote_calls.calls = &g_hfp_relay_ctx.remote_call`.
- Sets the CLCC state to `COMPLETE`.

### 7.2 Event Entry APIs

| API | Purpose | Return value |
| --- | --- | --- |
| `bt_hfp_relay_hf_event_handle(bt_hfp_relay_notify_data_t *msg)` | Handles phone-side HFP HF events | Always returns `0`. |
| `bt_hfp_relay_ag_event_handle(bt_hfp_relay_notify_data_t *msg)` | Handles headset-side HFP AG events | Returns `0` on success and `-RT_EINVAL` for invalid parameters. |
| `bt_hfp_relay_handle_sco_event(uint16_t event_id, bt_notify_device_sco_info_t *sco_info)` | Handles SCO events | No return value. |

### 7.3 Channel Management APIs

| API | Purpose | Key behavior |
| --- | --- | --- |
| `bt_hfp_relay_set_hf_channel(uint16_t channel, bt_notify_device_mac_t *mac)` | Sets the phone-side HF channel | Caches `hf_channel` and `hf_mac`. |
| `bt_hfp_relay_clear_hf_channel(uint16_t channel)` | Clears the phone-side HF channel | If the channel matches, clears the HF channel, call cache, local phone number cache, and pending state. |
| `bt_hfp_relay_set_ag_channel(uint16_t channel, bt_notify_device_mac_t *mac)` | Sets the headset-side AG channel | Caches `ag_channel` and `ag_mac`. |
| `bt_hfp_relay_clear_ag_channel(uint16_t channel)` | Clears the headset-side AG channel | If the channel matches, clears the AG channel and pending state. |

### 7.4 Status and Cache APIs

| API | Purpose | Key behavior |
| --- | --- | --- |
| `bt_hfp_relay_reset_cached_call()` | Clears the remote call cache | Clears `remote_call` and sets `remote_calls.num_call = 0`. |
| `bt_hfp_relay_cache_local_phone_num(uint8_t *number, uint16_t number_len)` | Caches the local phone number | Copies at most `PHONE_NUM_LEN - 1` bytes and sets `has_local_phone_num`. |
| `bt_hfp_relay_cache_remote_call(bt_notify_clcc_ind_t *clcc_info)` | Caches CLCC call information | Currently stores only one call and sets `remote_calls.num_call = 1`. |
| `bt_hfp_relay_update_cind_from_all_status(bt_notify_all_call_status *call_status)` | Synchronizes complete CIND status | Updates `cind_status` based on phone-side call status. |
| `bt_hfp_relay_update_cind_by_indicator(uint8_t type, uint8_t val)` | Synchronizes a single CIND indicator | Updates the specified field based on the indicator type. |
| `bt_hfp_relay_notify_ag_call_state()` | Notifies the headset side of call status | Converts the cached status into `HFP_CALL_INFO_T` and calls `bt_interface_phone_state_changed()`. |

### 7.5 CLCC and AT Confirmation APIs

| API | Purpose | Key behavior |
| --- | --- | --- |
| `bt_hfp_relay_start_get_clcc()` | Starts a CLCC query | Creates or restarts a one-shot soft timer and switches the CLCC state. |
| `bt_hfp_relay_handle_at_cmd_cfm(uint8_t cmd_id, uint8_t res)` | Handles phone-side AT command confirmations | Completes CLCC query responses or dialing responses. |

| `cmd_id` | Handling logic |
| --- | --- |
| `HFP_HF_AT_CLCC` | CLCC query is complete; returns the call list to the headset through `bt_interface_remote_call_info_res()`. |
| `HFP_HF_AT_ATD` | Dialing confirmation is complete; clears `pending_ag_make_call` and returns the dialing result to the headset through `bt_interface_make_call_res()`. |
| Other AT commands | Currently mainly prints confirmation-result logs. |

## 8. Notes and Recommendations

1. **Confirm initialization call**  
   Ensure that `bt_hfp_relay_mgr_init()` is called when the feature starts, so channels, default CIND status, CLCC state, and the call-list pointer are initialized.

2. **Insufficient null-pointer protection**  
   Some internal interfaces do not perform complete null-pointer checks, such as `bt_hfp_relay_hf_event_handle()`, `bt_hfp_relay_set_hf_channel()`, `bt_hfp_relay_set_ag_channel()`, `bt_hfp_relay_cache_local_phone_num()`, and `bt_hfp_relay_cache_remote_call()`. Callers must ensure that parameters are valid.

3. **Limited multi-call cache capability**  
   Currently, only one `remote_call` is cached. To fully support three-way or multi-party calls, this should be extended to an array cache.

4. **CLCC query interface semantics need confirmation**  
   `bt_hfp_relay_clcc_timeout()` calls `bt_interface_get_ph_num_by_id(hf_channel)` in the `START` state. From the flow, its intent is to query CLCC. It is recommended to confirm whether the lower-level interface is reused or whether there is an interface misuse.

5. **Writable context pointer exposure**  
   `bt_hfp_relay_get_context()` returns a writable pointer to the global context, allowing external modules to directly modify internal state. In the future, consider narrowing it to read-only queries or dedicated setters/getters.