# HFP_AG
HFP (Hands-Free Profile) is a Bluetooth hands-free protocol that allows Bluetooth devices to control phone calls on peer Bluetooth devices, such as Bluetooth headsets controlling phone call answering, hanging up, rejecting, voice dialing, etc. Data interaction between the two Bluetooth ends in HFP is communicated through predefined AT commands. HFP defines two roles: Audio Gateway (AG) and Hands-Free component (HF):
- Audio Gateway (AG): This device serves as the gateway for audio input/output. Typical gateway devices are mobile phones
- Hands-Free component (HF): This device serves as the remote audio input/output mechanism for the audio gateway and can provide several remote control functions. Typical hands-free component devices are car systems and Bluetooth headsets
![HFP Role](../../assets/HFP_ARCH.png)
- AT command rules:
    - One command line can only represent one AT command
    - < cr > abbreviation for carriage return, equivalent to the enter key, ASCII code is 0x0D
    - < lf > abbreviation for NL line feed, new line, equivalent to the line feed key, ASCII code is 0x0A
    - HF -> AG sends AT command format: < AT command >< cr >
    - AG -> HF sends AT command format: < cr >< lf >result code< cr >< lf > 
    - If the AT command result code sent by AG to HF is a message reply, it must be followed by an OK message, unless the reply is a +CME ERROR message, where the following parameters represent the reason for failure.

This document is mainly based on Sifli SDK, introducing how to use basic functions of AG role. Related files include:
- bts2_app_interface
- bts2_app_hfp_ag

## HFP_AG Initialization
- AG initialization function: bt_hfp_ag_app_init, assigns initial values to AG-related states and flags
- AG service startup function: bt_hfp_start_profile_service, configures +BRSF related attributes, users can adjust corresponding feature values according to requirements
- AT cmd: 
    - +BRSF: AG support features (Bluetooth Retrieve Supported Features)
    - Format: +BRSF:< AG support features >
    - After the HF side sends its supported features to the AG side, the AG end must also send its supported features to HF through "+BRSF:< AG support features >".
```c
U32 features = (U32)(HFP_AG_FEAT_ECNR | \
                             HFP_AG_FEAT_INBAND | \
                             HFP_AG_FEAT_REJECT | \
                             HFP_AG_FEAT_ECS | \
                             HFP_AG_FEAT_EXTERR | \
                             HFP_AG_FEAT_CODEC | \
                             HFP_AG_FEAT_ESCO);

/* AG feature masks */
#define HFP_AG_FEAT_3WAY    0x00000001      /* Three-way calling */
#define HFP_AG_FEAT_ECNR    0x00000002      /* Echo cancellation/noise reduction */
#define HFP_AG_FEAT_VREC    0x00000004      /* Voice recognition */
#define HFP_AG_FEAT_INBAND  0x00000008      /* In-band ring tone */
#define HFP_AG_FEAT_VTAG    0x00000010      /* Attach a phone number to a voice tag */
#define HFP_AG_FEAT_REJECT  0x00000020      /* Ability to reject incoming call */
#define HFP_AG_FEAT_ECS     0x00000040      /* Enhanced Call Status */
#define HFP_AG_FEAT_ECC     0x00000080      /* Enhanced Call Control */
#define HFP_AG_FEAT_EXTERR  0x00000100      /* Extended error codes */
#define HFP_AG_FEAT_CODEC   0x00000200      /* Codec Negotiation */

/* Valid feature bit mask for HFP 1.6 (and below) */
#define HFP_1_6_FEAT_MASK   0x000003FF

/* HFP 1.7+ */
#define HFP_AG_FEAT_HF_IND  0x00000400      /* HF Indicators */
#define HFP_AG_FEAT_ESCO    0x00000800      /* eSCO S4 (and T2) setting supported */

/* Proprietary features: using 31 ~ 16 bits */
#define HFP_AG_FEAT_BTRH    0x00010000      /* CCAP incoming call hold */
#define HFP_AG_FEAT_UNAT    0x00020000      /* Pass unknown AT commands to app */
#define HFP_AG_FEAT_NOSCO   0x00040000      /* No SCO control performed by BTA AG */
#define HFP_AG_FEAT_NO_ESCO 0x00080000      /* Do not allow or use eSCO */
#define HFP_AG_FEAT_VOIP    0x00100000      /* VoIP call */
```

## HFP_AG Device Connection
When LM Link and RFCOMM Connection already exist, user behavior or other internal events that want to use HFP service need to first establish SLC (Service Level Connection). Establishing SLC connection requires the following 5 phases:
1. Supported features exchange (AT+BRSF)
2. Codec Negotiation (AT+BAC)
3. AG Indicators (AT+CIND, AT+CMER, +CIEV, AT+CHLD)
4. HF Indicators (AT+BIND, AT+BIEV)
5. End of Service Level Connection
![HFP Connect Progress](../../assets/hfp_connect_progress.png)
- AT cmd: 
    - AT+BAC: (Bluetooth Available Codecs)
    - Format: AT+BAC=< codec_id1 >,< codec_id2 >
    - HF side informs AG side which encoding methods are supported
    - codec: CVSD and msbc
- AT cmd: 
    - AT+CIND: (Standard indicator update AT command)
    - Format:
        - AT+CIND=? Test command. HF obtains indicator index values and ranges supported by AG side. Must be requested at least once before sending indicator-related commands (AT+CIND? or AT_CMER).
        - AT+CIND? Read command, HF reads current indicator values from AG side
- AT cmd: 
    - +CIEV: unsolicited result code (Standard indicator events reporting unsolicited result code)
    - Format: +CIEV: < ind >,< value >
    - When AG indicators change, AG needs to actively notify HF side using +ciev AT cmd
    ![indicator type](../../assets/HFP_AG_INDICATOR.png)
- AG device connection interface:
    - bts2_app_interface connection interface: bt_interface_conn_to_source_ext
    - bts2_app_hfp_ag connection interface: bt_hfp_connect_profile
       
- AG device disconnection interface:
    - bts2_app_hfp_ag disconnection interface: bt_hfp_disconnect_profile
        
- AG connection status callback events: 
        - AG connection successful: BT_NOTIFY_AG_PROFILE_CONNECTED 
        - AG connection failed: BT_NOTIFY_AG_PROFILE_DISCONNECTED
```c
// After calling the API to connect AG role, the AG connection success message is sent to user through notify
// Users need to implement hdl functions to receive notify events, such as: bt_notify_handle
// SLC message: BT_NOTIFY_AG_PROFILE_CONNECTED, users can forward the message to user task for processing.
// BT_NOTIFY_AG_PROFILE_CONNECTED event includes: address information, profile_type, res: 0 (success)
// To disconnect device connection, call bt_hfp_disconnect_profile
// BT_NOTIFY_AG_PROFILE_DISCONNECTED event includes: address information, profile_type, reason
// For specific structure information, refer to API comments
static int bt_notify_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    int ret = -1;

    switch (type)
    {
    case BT_NOTIFY_HFP_AG:
    {
        bt_sifli_notify_hfp_ag_event_hdl(event_id, data, data_len);
    }
    break;
    }
    return 0;
}


int bt_sifli_notify_hfp_ag_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_AG_PROFILE_CONNECTED:
    {
        //handle connected msg(bt_notify_profile_state_info_t *)
        break;
    }
    case BT_NOTIFY_AG_PROFILE_DISCONNECTED:
    {
        //handle disconnected msg(bt_notify_profile_state_info_t *)
        break;
    }
    }
    return 0;
}
```
:::{note}
Note: The address parameters passed by the two interfaces need to be converted accordingly.
:::
- During AG SLC connection process, event when receiving AT+CIND=? cmd from HF: BT_NOTIFY_AG_GET_INDICATOR_STATUS_REQ
- AG needs to reply to event: BT_NOTIFY_AG_GET_INDICATOR_STATUS_REQ
    - bts2_app_interface indicator status reply interface: bt_interface_get_all_indicator_info_res
    - bts2_app_hfp_ag indicator status reply interface: bt_hfp_ag_cind_response
```c
typedef struct
{
    // 0 (No home/roam network available) / 1 (home/roam network available)
    U8 service_status;
    // 0 (There are no calls in progress) / 1 (At least one call in progress)
    U8 call;
    // 0 (No currently in call set up)
    // 1 (An incoming call process ongoing)
    // 2 (An outgoing call process ongoing)
    // 3 (Remote party being alert in an outgoing call)
    U8 callsetup;
    // Phone battery val (0~5)
    U8 batt_level;
    // Phone signal val (0~5)
    U8 signal;
    // 0 (Roaming is not active) / 1 (Roaming is active)
    U8 roam_status;
    // 0 (No call held)
    // 1 (Call is placed on hold or active/held call swapped)
    // 2 (Call on hold, no active call)
    U8 callheld;
} hfp_cind_status_t;

    hfp_cind_status_t cind_status;
    cind_status.service_status = 1;
    cind_status.call = 0;
    cind_status.callsetup = 0;
    cind_status.batt_level = 5;
    cind_status.signal = 3;
    cind_status.roam_status = 0;
    cind_status.callheld = 0;
    bt_interface_get_all_indicator_info_res(&cind_status);
```

## HFP_AG Basic Functionality Usage
### Call Audio Establishment
Audio Connection in HFP specification usually refers to SCO/eSCO voice path connection. Before SCO/eSCO, HF (AT+BCC) needs to notify AG to first select codec algorithm.
- AT cmd: 
    - AT+BCC: Bluetooth Codec Connection
    - Format: AT+BCC
    - HF sends to AG, triggers AG to initiate codec connection process
    - If AG decides to initiate codec connection process, reply OK; otherwise ERROR. After replying OK, AG side will send +BCS:< codec_id > and HF side replies: AT+BCS=< codec_id >. After that, eSCO link is established
- AT cmd: 
    - AT+BCS: Bluetooth Codec Selection
    - Format: AT+BCS=< codec_id >
    - +BCS: Bluetooth Codec Selection
    - Format: +BCS:< codec_id >
    - codec: (codec_id=1) CVSD and (codec_id=2) msbc
    - Before AG establishes eSCO, it will send command +BCS:< codec_id > to HF. HF side replies: AT+BCS=< codec_id >. If both AG and HF support this ID, the secondary link will be established. But if ID is not supported, HF will respond with AT+BAC and its available codecs. If (e)SCO link cannot be established, AG will restart the Codec Connection establishment process. Before Codec connection establishment, CVSD encoding will be enabled.

#### Sifli HFP_AG Connect/Disconnect Call Audio
- bts2_app_interface call audio connection interface: bt_interface_ag_audio_switch
- bts2_app_hfp_ag call audio connection interface: bt_hfp_connect_audio
- bts2_app_interface call audio disconnection interface: bt_interface_ag_audio_switch
- bts2_app_hfp_ag call audio disconnection interface: bt_hfp_disconnect_audio
```c
    // Call audio establishment connection interface bt_interface_ag_audio_switch parameters: type 0:connect audio type 1:disconnect audio + device mac address
    // Call audio connection successful BT_NOTIFY_COMMON_SCO_CONNECTED event: sco type (distinguish HF or AG) status (status)
    // For specific structure explanation, please refer to relevant descriptions in interface
static int bt_notify_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    int ret = -1;

    switch (type)
    {
    case BT_NOTIFY_COMMON:
    {
        bt_sifli_notify_common_event_hdl(event_id, data, data_len);
    }
    break;
    }
    return 0;
}


int bt_sifli_notify_common_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_COMMON_SCO_CONNECTED:
    {
        //handle sco connected msg(bt_notify_device_sco_info_t *)
        break;
    }
    case BT_NOTIFY_COMMON_SCO_DISCONNECTED:
    {
        //handle sco disconnected msg(bt_notify_device_sco_info_t *)
        break;
    }
    }
    return 0;
}
```

### AG Phone Status Update
- AT cmd: 
    - AT+CIND: (Standard indicator update AT command)
    - Format:
        - AT+CIND=? Test command. HF obtains indicator index values and ranges supported by AG side. Must be requested at least once before sending indicator-related commands (AT+CIND? or AT_CMER).
        - AT+CIND? Read command, HF reads current indicator values from AG side
- AT cmd: 
    - +CIEV: unsolicited result code (Standard indicator events reporting unsolicited result code)
    - Format: +CIEV: < ind >,< value >
    - When AG indicators change, AG needs to actively notify HF side using +ciev AT cmd
- AT cmd: 
    - AT+CLCC: Standard list current calls command
    - Format: AT+CLCC
    - +CLCC: Standard list current calls command
    - Format: +CLCC: < idx >,< dir >,< status >,< mode >,< mpty >,< number >,< type >
    - HF requests current call information list, AG side replies current call information list through "+CLCC". If there are currently no calls, AG side must also reply with OK command.

Once the phone AG end call status changes, AG should notify HF of the current changed call status. For example, after an incoming call, Bluetooth headset HF end answers, call status changes, AG side +CIEV:2,1 notifies HF to reject or AG hangs up, AG side +CIEV:2,0 notifies HF end.
- call Status
    - 0: No calls (held or active)
    - 1: Call is present (active or held)
- callsetup status
    - 0: No call setup in progress
    - 1: Incoming call setup in progress
    - 2: Outgoing call setup in dialing state
    - 3: Outgoing call setup in alerting state
- callheld status
    - 0: No call held
    - 1: Call is placed on hold or active/held calls swapped
    - 2: Call on hold, no active call
- Incoming call state transition: 
    - call_idle <----> call_incoming <-----> call_active <----> call_idle
- Outgoing call state transition: 
    - call_idle <----> call_outgoing_dialing ----> call_outgoing_alerting <-----> call_active <----> call_idle
    - call_idle <----> call_outgoing_dialing ----> call_outgoing_alerting <-----> call_idle

#### Sifli AG Update Phone Information
- bts2_app_interface phone status update interface: bt_interface_phone_state_changed
- bts2_app_hfp_ag phone status update interface: bt_hfp_ag_call_state_update_listener
```c
typedef struct
{
    U16 type;  //ignore
    U8 num_active;
    U8 num_held;
    U8 callsetup_state;
    U8 phone_type;
    U8 phone_len;
    U8 phone_number[1];
} HFP_CALL_INFO_T;

// a call incoming
HFP_CALL_INFO_T call_info;
call_info.num_active = 0;
call_info.num_held = 0;
call_info.callsetup_state = 1;
call_info.phone_type = 0x81;
char *str = "1234567";
bmemcpy(&call_info.phone_number, str, strlen(str) + 1);
call_info.phone_len = strlen(str) + 1;
bt_interface_phone_state_changed(&call_info);


// a call active
HFP_CALL_INFO_T call_info;
call_info.num_active = 1;
call_info.num_held = 0;
call_info.callsetup_state = 0;
call_info.phone_type = 0x81;
char *str = "1234567";
bmemcpy(&call_info.phone_number, str, strlen(str) + 1);
call_info.phone_len = strlen(str) + 1;
bt_interface_phone_state_changed(&call_info);

// a call idle
HFP_CALL_INFO_T call_info;
call_info.num_active = 0;
call_info.num_held = 0;
call_info.callsetup_state = 0;
call_info.phone_type = 0x81;
char *str = "1234567";
bmemcpy(&call_info.phone_number, str, strlen(str) + 1);
call_info.phone_len = strlen(str) + 1;
bt_interface_phone_state_changed(&call_info);


// a call outgoing dialing
HFP_CALL_INFO_T call_info;
call_info.num_active = 0;
call_info.num_held = 0;
call_info.callsetup_state = 2;
call_info.phone_type = 0x81;
char *str = "1234567";
bmemcpy(&call_info.phone_number, str, strlen(str) + 1);
call_info.phone_len = strlen(str) + 1;
bt_interface_phone_state_changed(&call_info);

// a call outgoing alerting
HFP_CALL_INFO_T call_info;
call_info.num_active = 0;
call_info.num_held = 0;
call_info.callsetup_state = 3;
call_info.phone_type = 0x81;
char *str = "1234567";
bmemcpy(&call_info.phone_number, str, strlen(str) + 1);
call_info.phone_len = strlen(str) + 1;
bt_interface_phone_state_changed(&call_info);

// a call active
HFP_CALL_INFO_T call_info;
call_info.num_active = 1;
call_info.num_held = 0;
call_info.callsetup_state = 0;
call_info.phone_type = 0x81;
char *str = "1234567";
bmemcpy(&call_info.phone_number, str, strlen(str) + 1);
call_info.phone_len = strlen(str) + 1;
bt_interface_phone_state_changed(&call_info);

// a call idle
HFP_CALL_INFO_T call_info;
call_info.num_active = 0;
call_info.num_held = 0;
call_info.callsetup_state = 0;
call_info.phone_type = 0x81;
char *str = "1234567";
bmemcpy(&call_info.phone_number, str, strlen(str) + 1);
call_info.phone_len = strlen(str) + 1;
bt_interface_phone_state_changed(&call_info);
```

### AG Information Synchronization Processing
- AT cmd: 
    - +CIEV: unsolicited result code (Standard indicator events reporting unsolicited result code)
    - Format: +CIEV: < ind >,< value >
    - When AG indicators change, AG needs to actively notify HF side using +ciev AT cmd
- Indicator status value update
    - When AG end indicators change, AG needs to actively update status
    - bts2_app_interface status update interface: bt_interface_indicator_status_changed
    - bts2_app_hfp_ag status update interface: bt_hfp_ag_ind_status_update
    - Common indicator types are shown in the following figure:
    ![Figure 1: indicator type](../../assets/HFP_AG_INDICATOR.png)
    - Service status update demo:
```c
typedef struct
{
    uint8_t ind_type;
    uint8_t ind_val;
} hfp_ind_info_t;

enum
{
    HFP_AG_CIND_SERVICE_TYPE = 0x01,    //(0,1)
    HFP_AG_CIND_CALL_TYPE,              //(0,1)
    HFP_AG_CIND_CALLSETUP_TYPE,         //(0,3)
    HFP_AG_CIND_BATT_TYPE,              //(0,5)
    HFP_AG_CIND_SIGNAL_TYPE,            //(0,5)
    HFP_AG_CIND_ROAM_TYPE,              //(0,1)
    HFP_AG_CIND_CALLHELD_TYPE,          //(0,2)
};

hfp_ind_info_t ind_info;
ind_info.ind_type = HFP_AG_CIND_SERVICE_TYPE;
ind_info.ind_val = 3;
bt_interface_indicator_status_changed(&ind_info);
```

##### AG Local Number Synchronization
- AT cmd: 
    - AT+CNUM: Subscriber Number Information
    - Format: AT+CNUM
    - This command is used to query local number
    - +CNUM: Subscriber Number Information
    - Format: +CNUM: ,< number >,< type >[,, < service >]
    - After AG receives this request, it will reply the phone's local number to HF side through +CNUM.
- Event when receiving HF end request for local number: BT_NOTIFY_AG_GET_LOCAL_PHONE_INFO_REQ
- Interface to reply local number:
    - bts2_app_interface local number reply interface: bt_interface_local_phone_info_res 
    - bts2_app_hfp_ag local number reply interface: bt_hfp_ag_cnum_response
```c
typedef struct
{
    char phone_number[PHONE_NUM_LEN];
    U8 type;
} hfp_phone_number_t;

hfp_phone_number_t local_phone_num;
char *str = "19396395979";
bmemcpy(&local_phone_num.phone_number, str, strlen(str));
local_phone_num.type = 0x81;
bt_interface_local_phone_info_res(&local_phone_num);
```

#### AG All Call Status Information
- AT cmd: 
    - AT+CLCC: Standard list current calls command
    - Format: AT+CLCC
    - +CLCC: Standard list current calls command
    - Format: +CLCC: < idx >,< dir >,< status >,< mode >,< mpty >,< number >,< type >
    - HF requests current call information list, AG side replies current call information list through "+CLCC". If there are currently no calls, AG side must also reply with OK command.
- Event when receiving HF end request for all call status information: BT_NOTIFY_AG_GET_ALL_REMT_CALLS_INFO_REQ
- Interface to reply all call status information:
    - bts2_app_interface all call status information interface: bt_interface_remote_call_info_res
    - bts2_app_hfp_ag all call status information interface: bt_hfp_ag_remote_calls_res_hdl
```c
typedef struct
{
    // Number of phone calls
    U8 num_call;
    hfp_phone_call_info_t *calls;
} hfp_remote_calls_info_t;

typedef struct
{
    // Which call this is, counting from 1
    U8 call_idx;
    // Call direction, 0: outgoing call; 1: incoming call
    U8 call_dir;
    // 0: Active
    // 1: held
    // 2: Dialing (outgoing calls only)
    // 3: Alerting (outgoing calls only)
    // 4: Incoming (incoming calls only)
    // 5: Waiting (incoming calls only)
    // 6: Call held by Response and Hold
    U8 call_status;
    // Call mode, 0 (Voice), 1 (Data), 2 (FAX)
    U8 call_mode;
    // Whether it's a conference call. 0: not conference call, 1: is conference call
    U8 call_mtpty;
    // Phone number + phone number type
    hfp_phone_number_t phone_info;
} hfp_phone_call_info_t;

hfp_phone_call_info_t call_info;
bmemset(&call_info, 0x00, sizeof(hfp_phone_call_info_t));
call_info.call_idx = 1;
call_info.call_dir = 1;
call_info.call_status = 1;
call_info.call_mode = 0;
call_info.call_mtpty = 0;
char *str = "123456";
bmemcpy(&call_info.phone_info.phone_number, str, strlen(str) + 1);
call_info.phone_info.type = 0x81;

hfp_remote_calls_info_t calls;
calls.num_call = 1;
calls.calls = &call_info;
bt_interface_remote_call_info_res((hfp_remote_calls_info_t *)calls);
```

### HFP_AG Phone Related Functions
- AT cmd: 
    - AT+CLIP: Calling Line Identification notification
    - Format: AT+CLIP=(0/1)
    - +CLIP: Calling Line Identification notification
    - Format: +CLIP: < number >,< type >
    - Enable or disable caller ID notification. When enabled, AG side sends current incoming call number and type to HF through "+CLIP" command during incoming calls
- AT cmd: 
    - AT+CCWA: Three-Way Call Handling
    - Format: AT+CCWA=(0/1)
    - Enable or disable three-way call waiting reminder
    - +CCWA: Call Waiting Notification
    - Format: +CCWA: < number >,< type >
    - When enabled, if there's already one connected call and another call comes in, AG will send +CCWA
- AT cmd: 
    - ATD: Dial call req
    - Format: ATDdd…dd
    - HF actively requests to make a call
    - AT+BLDN: Bluetooth Last Dialed Number
    - Format: AT+BLDN
    - Request AG side to redial the last call. After receiving this request, AG initiates a call based on the most recently dialed number.
- AT cmd: 
    - ATA: call answer
    - Format: ATA
    - Answer call. When connected, +CIEV:< call_ind >, < 1 > and +CIEV:< call_setup_ind >, < 0 > appear
- AT cmd: 
    - AT+CHUP: Reject an Incoming Call/Terminate a Call Process
    - Format: AT+CHUP
    - Reject or hang up call, +CIEV:< call_setup_ind >, < 0 > / +CIEV:< call_ind >, < 0 > appears

- Phone related functions
    - Event when receiving HF end answer call: BT_NOTIFY_AG_ANSWER_CALL_REQ
    - Event when receiving HF end hang up call: BT_NOTIFY_AG_HANGUP_CALL_REQ
    - Event when receiving HF end control call volume: BT_NOTIFY_AG_VOLUME_CHANGE
    - Event when receiving HF end make call: BT_NOTIFY_AG_MAKE_CALL_REQ
    - When receiving make call request, need to check number validity, reply check result
        - bts2_app_interface result reply interface: bt_interface_make_call_res
        - bts2_app_hfp_ag result reply interface: bt_hfp_ag_at_result_res
    - During call, event when receiving DTMF key: BT_NOTIFY_AG_RECV_DTMF_KEY
- AT cmd: 
    - AT+VGM: Gain of Microphone
    - Format: AT+VGM=< gain >
    - +VGM: Gain of Microphone
    - Format: +VGM:< gain >
    - Adjust MIC volume in audio path, value range 0~15
- AT cmd: 
    - AT+VGS: Gain of Speaker
    - Format: AT+VGS=< gain >
    - +VGS: Gain of Speaker
    - Format: +VGS:< gain >
    - Adjust speaker volume in audio path, value range 0~15
- Active volume control interface:
    - bts2_app_interface speaker volume control interface: bt_interface_set_speaker_volume
    - bts2_app_hfp_hf speaker volume control interface: bt_hfp_hf_update_spk_vol
    - bts2_app_hfp_hf microphone volume control interface: bt_hfp_hf_update_mic_vol
    - Volume setting processing result event: BT_NOTIFY_HF_AT_CMD_CFM with cmd_id (HFP_HF_AT_VGS/HFP_HF_AT_VGM)
    - Active speaker volume control:
        - bts2_app_interface speaker volume control interface: bt_interface_spk_vol_change_req
        - bts2_app_hfp_ag speaker volume control interface: bt_hfp_ag_spk_vol_control
    - Active microphone volume control:
        - bts2_app_interface microphone volume control interface: bt_interface_mic_vol_change_req
        - bts2_app_hfp_ag microphone volume control interface: bt_hfp_ag_mic_vol_control
```c
//register notify event handle function start
//step1: Call connection interface, AG connection success message: BT_NOTIFY_AG_PROFILE_CONNECTED
int bt_sifli_notify_hfp_ag_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    //AG end SLC CONNECTED
    case BT_NOTIFY_AG_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        break;
    }
    //AG end SLC DISCONNECTED
    case BT_NOTIFY_AG_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        break;
    }
    //AG end receives make call request from HF end, AG end needs to judge number validity
    //Processing result sent back to HF through bt_hfp_ag_at_result_res(res)
    case BT_NOTIFY_AG_MAKE_CALL_REQ:
    {
        break;
    }
    //AG end receives incoming call answer request from HF end
    //Can send message when there's incoming call
    /*
    // a call active
    HFP_CALL_INFO_T call_info;
    call_info.num_active = 1;
    call_info.num_held = 0;
    call_info.callsetup_state = 0;
    call_info.phone_type = 0x81;
    char *str = "1234567";
    bmemcpy(&call_info.phone_number, str, strlen(str) + 1);
    call_info.phone_len = strlen(str) + 1;
    bt_interface_phone_state_changed(&call_info);
    */
    case BT_NOTIFY_AG_ANSWER_CALL_REQ:
    {
        break;
    }
    //AG end receives reject or hang up call from HF end
    /*
    // a call active
    HFP_CALL_INFO_T call_info;
    call_info.num_active = 0;
    call_info.num_held = 0;
    call_info.callsetup_state = 0;
    call_info.phone_type = 0x81;
    char *str = "1234567";
    bmemcpy(&call_info.phone_number, str, strlen(str) + 1);
    call_info.phone_len = strlen(str) + 1;
    bt_interface_phone_state_changed(&call_info);
    */
    case BT_NOTIFY_AG_HANGUP_CALL_REQ:
    {
        break;
    }
    //AG end receives DTMF value sent by HF end
    case BT_NOTIFY_AG_RECV_DTMF_KEY:
    {
        break;
    }
    //AG end receives volume control from HF end
    case BT_NOTIFY_AG_VOLUME_CHANGE:
    {
        break;
    }
    //AG end receives request for all current indicator values from HF end
    /*
        hfp_cind_status_t cind_status;
        cind_status.service_status = 1;
        cind_status.call = 0;
        cind_status.callsetup = 0;
        cind_status.batt_level = 5;
        cind_status.signal = 3;
        cind_status.roam_status = 0;
        cind_status.callheld = 0;
        bt_interface_get_all_indicator_info_res(&cind_status);
    */
    case BT_NOTIFY_AG_GET_INDICATOR_STATUS_REQ:
    {
        break;
    }
    //AG end receives request for all current call list information from HF end
    /*
        hfp_phone_call_info_t call_info;
        bmemset(&call_info, 0x00, sizeof(hfp_phone_call_info_t));
        call_info.call_idx = 1;
        call_info.call_dir = 1;
        call_info.call_status = 1;
        call_info.call_mode = 0;
        call_info.call_mtpty = 0;
        char *str = "123456";
        bmemcpy(&call_info.phone_info.phone_number, str, strlen(str) + 1);
        call_info.phone_info.type = 0x81;

        hfp_remote_calls_info_t calls;
        calls.num_call = 1;
        calls.calls = &call_info;
        bt_interface_remote_call_info_res((hfp_remote_calls_info_t *)calls);
    */
    case BT_NOTIFY_AG_GET_ALL_REMT_CALLS_INFO_REQ:
    {
        break;
    }
    //AG end receives request for local phone number from HF end
    /*
    hfp_phone_number_t local_phone_num;
    char *str = "19396395979";
    bmemcpy(&local_phone_num.phone_number, str, strlen(str));
    local_phone_num.type = 0x81;
    bt_interface_local_phone_info_res(&local_phone_num);
    */
    case BT_NOTIFY_AG_GET_LOCAL_PHONE_INFO_REQ:
    {
        break;
    }
    case BT_NOTIFY_AG_EXTERN_AT_CMD_KEY_REQ:
    {
        break;
    }
    default:
        return -1;
    }
    return 0;
}
static int bt_sifli_notify_common_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    //BT functionality enabled successfully, can be used normally
    case BT_NOTIFY_COMMON_BT_STACK_READY:
    {
        break;
    }
    //BT shutdown successful
    case BT_NOTIFY_COMMON_CLOSE_COMPLETE:
    {
        break;
    }
    // ACL connection successful
    case BT_NOTIFY_COMMON_ACL_CONNECTED:
    {
        bt_notify_device_acl_conn_info_t *acl_info = (bt_notify_device_acl_conn_info_t *) data;
        //device acl connected
        break;
    }
    // ACL disconnection successful
    case BT_NOTIFY_COMMON_ACL_DISCONNECTED:
    {
        bt_notify_device_base_info_t *device_info = (bt_notify_device_base_info_t *)data;
        //device acl disconnected
        break;
    }
    // SCO connection successful
    case BT_NOTIFY_COMMON_SCO_CONNECTED:
    {
        bt_notify_device_sco_info_t *sco_info = (bt_notify_device_sco_info_t *)data;
        break;
    }
    // SCO disconnection successful
    case BT_NOTIFY_COMMON_SCO_DISCONNECTED:
    {
        bt_notify_device_sco_info_t *sco_info = (bt_notify_device_sco_info_t *)data;
        break;
    }

    default:
        return -1;
    }
    return 0;
}

static int bt_notify_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    int ret = -1;

    switch (type)
    {
    case BT_NOTIFY_COMMON:
    {
        ret = bt_sifli_notify_common_event_hdl(event_id, data, data_len);
    }
    break;

    case BT_NOTIFY_HFP_AG:
    {
        bt_sifli_notify_hfp_ag_event_hdl(event_id, data, data_len);
    }
    break;

    default:
        break;
    }

    return 0;
}

int app_bt_notify_init(void)
{
    bt_interface_register_bt_event_notify_callback(bt_notify_handle);
    return BT_EOK;
}

INIT_ENV_EXPORT(app_bt_notify_init);
//register notify event handle function end
```