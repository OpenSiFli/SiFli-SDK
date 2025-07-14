# HFP_HF
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

This document is mainly based on Sifli SDK, introducing how to use basic functions of HF role. Related files include:
- bts2_app_interface
- bts2_app_hfp_hf

## HFP_HF Initialization
- HF initialization function: bt_hfp_hf_init, assigns initial values to HF-related states and flags
- HF service startup function: bt_hfp_hf_start_enb, configures AT+BRSF related attributes, users can adjust corresponding feature values according to requirements
- AT cmd: 
    - AT+BRSF: HF support features (Bluetooth Retrieve Supported Features)
    - Format: AT+BRSF=< HF support features >
    - When establishing connection, HF side sends its supported features to AG side
```c
#define HFP_HF_LOCAL_FEATURES        (  HFP_HF_FEAT_ECNR  | \
                                        HFP_HF_FEAT_3WAY  | \
                                        HFP_HF_FEAT_CLI   | \
                                        HFP_HF_FEAT_VREC  | \
                                        HFP_HF_FEAT_VOL   | \
                                        HFP_HF_FEAT_ECS   | \
                                        HFP_HF_FEAT_ECC   | \
                                        HFP_HF_FEAT_CODEC | \
                                        HFP_HF_FEAT_ESCO  )
/* HFP peer features */
#define HFP_HF_FEAT_ECNR        0x0001      /* Echo cancellation/noise reduction */
#define HFP_HF_FEAT_3WAY        0x0002      /* Call waiting and three-way calling */
#define HFP_HF_FEAT_CLI         0x0004      /* Caller ID presentation capability */
#define HFP_HF_FEAT_VREC        0x0008      /* Voice recognition activation */
#define HFP_HF_FEAT_VOL         0x0010      /* Remote volume control */
#define HFP_HF_FEAT_ECS         0x0020      /* Enhanced Call Status */
#define HFP_HF_FEAT_ECC         0x0040      /* Enhanced Call Control */
#define HFP_HF_FEAT_CODEC       0x0080      /* Codec Negotiation */
#define HFP_HF_FEAT_HF_IND      0x0100      /* HF Indicators */
#define HFP_HF_FEAT_ESCO        0x0200      /* eSCO S4 (and T2) setting supported */
#define HFP_HF_FEAT_ENVR_EXT    0x0400      /*ENHANCED_VOICE_RECOGNITION_ST*/
#define HFP_HF_FEAT_VREC_TEXT   0x0800

/* Proprietary features: using bits after 12 */

/* Pass unknown AT command responses to application */
#define HFP_HF_FEAT_UNAT 0x1000
#define HFP_HF_FEAT_VOIP 0x2000         /* VoIP call */
```

## HFP_HF Device Connection
AG devices and HF devices establishing connection and making calls will roughly go through the following four phases. Next, we will focus on introducing the Service Level Connection and Audio Connection processes according to the Specification.
1. LMP link: First establish LMP Link, there is no fixed master or slave in HFP connection
2. RFCOMM Connection: Then establish serial port emulation data link channel for HF to transmit user data to AG (including modem control signals and AT commands), AG parses AT commands and sends corresponding responses to HF through the serial port emulation
3. Service Level Connection: Service level connection is the foundation for application layer HF <<>> AG control and interaction information.
    - Service Level Connection establishment
    - Service Level Connection release
4. Synchronous Connection/Audio Connection: SCO/eSCO call connection usually refers to voice data connection
    - Codec Connection
    - Wide Band Speech Connection

### SLC (Service Level Connection) Establishment Process
When LM Link and RFCOMM Connection already exist, user behavior or other internal events that want to use HFP service need to first establish SLC (Service Level Connection). Establishing SLC connection requires the following 5 phases:
1. Supported features exchange (AT+BRSF)
2. Codec Negotiation (AT+BAC)
3. AG Indicators (AT+CIND, AT+CMER, +CIEV, AT+CHLD)
4. HF Indicators (AT+BIND, AT+BIEV)
5. End of Service Level Connection
![HFP Connect Progress](../../assets/hfp_connect_progress.png)

### Sifli HFP_HF Connect/Disconnect Device
- HF device connection interface:
    - bts2_app_interface connection interface: bt_interface_conn_ext
    - bts2_app_hfp_hf connection interface: bt_hfp_hf_start_connecting 
       
- HF device disconnection interface:
    - bts2_app_interface disconnection interface: bt_interface_disc_ext
    - bts2_app_hfp_hf disconnection interface: bt_hfp_hf_start_disc
        
- HF connection status callback events:
    - HF connection successful: BT_NOTIFY_HF_PROFILE_CONNECTED
    - HF connection failed: BT_NOTIFY_HF_PROFILE_DISCONNECTED
:::{note}
Note: The address parameters passed by the two interfaces need to be converted accordingly.
:::
```c
// After calling the API to connect HF role, the HF connection success message is sent to user through notify
// Users need to implement hdl functions to receive notify events, such as: bt_notify_handle
// SLC message: BT_NOTIFY_HF_PROFILE_CONNECTED, users can forward the message to user task for processing.
// BT_NOTIFY_HF_PROFILE_CONNECTED event includes: address information, profile_type, res: 0 (success)
// To disconnect device connection, call bt_hfp_hf_start_disc
// BT_NOTIFY_HF_PROFILE_DISCONNECTED event includes: address information, profile_type, reason
// For specific structure information, refer to API comments
static int bt_notify_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    int ret = -1;

    switch (type)
    {
    case BT_NOTIFY_HFP_HF:
    {
        bt_sifli_notify_hfp_hf_event_hdl(event_id, data, data_len);
    }
    break;
    }
    return 0;
}


int bt_sifli_notify_hfp_hf_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_HF_PROFILE_CONNECTED:
    {
        //handle hf connected msg(bt_notify_profile_state_info_t *)
        break;
    }
    case BT_NOTIFY_HF_PROFILE_DISCONNECTED:
    {
        //handle disconnected msg(bt_notify_profile_state_info_t *)
        break;
    }
    }
    return 0;
}
```

## HFP_HF Basic Functionality Usage
### Call Audio Establishment
Audio Connection in HFP specification usually refers to SCO/eSCO voice path connection. Before SCO/eSCO, HF (AT+BCC) needs to notify AG to first select codec algorithm.
- AT cmd: 
    - AT+BCC: Bluetooth Codec Connection
    - Format: AT+BCC
    - HF sends to AG, triggers AG to initiate codec connection process
    - If AG decides to initiate codec connection process, reply OK; otherwise ERROR. After replying OK, AG side will send +BCS:< codec_id > and HF side replies: AT+BCS=< codec_id >. After that, eSCO link is established

#### Sifli HFP_HF Connect/Disconnect Call Audio
- bts2_app_interface call audio connect/disconnect interface: bt_interface_audio_switch
- bts2_app_hfp_hf call audio connect/disconnect interface: bt_hfp_hf_audio_transfer 
- Prerequisites for successful call audio establishment: phone end call status is not idle; because when call is idle, phone end will reject AT+BCC request to establish SCO.
- Call audio encoding includes: CVSD and msbc, interface to control whether msbc codec is enabled: bt_interface_set_wbs_status 
- After call audio is successfully established, need to call audio_open; after call audio is disconnected or HF is disconnected, need to call audio_close. Related implementation can be found in: hfp_audio_api
```c
    // Call audio establishment connection interface bt_interface_audio_switch parameters: type 0:connect audio type 1:disconnect audio
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
        //handle hf sco connected msg(bt_notify_device_sco_info_t *)
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

### Call Audio Volume Control
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
- Peer device volume control event: BT_NOTIFY_HF_VOLUME_CHANGE
```c
// When call audio exists, to control call audio volume, call bt_interface_set_speaker_volume with volume value
// HF will notify AG of volume value using AT+VGS format
// After AG receives volume value, reply OK if processing successful, reply ERROR if processing failed
// Corresponding user event: BT_NOTIFY_HF_AT_CMD_CFM with cmd_id (HFP_HF_AT_VGS/HFP_HF_AT_VGM)
// For specific structure explanation, please refer to relevant descriptions in interface
static int bt_notify_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    int ret = -1;

    switch (type)
    {
    case BT_NOTIFY_HFP_HF:
    {
        bt_sifli_notify_hfp_hf_event_hdl(event_id, data, data_len);
    }
    break;
    }
    return 0;
}


int bt_sifli_notify_hfp_hf_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_HF_VOLUME_CHANGE:
    {
        //handle volume value
        break;
    }
    case BT_NOTIFY_HF_AT_CMD_CFM:
    {
        //handle at cmd result msg（bt_notify_at_cmd_cfm_t *）
        break;
    }
    }
    return 0;
}

// at cmd_id
enum
{
    HFP_HF_AT_NONE = 0x00,
    HFP_HF_AT_BRSF,
    HFP_HF_AT_BAC,
    HFP_HF_AT_CIND,
    HFP_HF_AT_CIND_STATUS,
    HFP_HF_AT_CMER,
    HFP_HF_AT_CHLD,
    HFP_HF_AT_CHLD_CMD,
    HFP_HF_AT_CMEE,
    HFP_HF_AT_BIA,
    HFP_HF_AT_CLIP,
    HFP_HF_AT_CCWA,
    HFP_HF_AT_COPS,
    HFP_HF_AT_COPS_CMD,
    HFP_HF_AT_CLCC,
    HFP_HF_AT_BVRA,
    HFP_HF_AT_VGS, //speaker volume control cmd_id
    HFP_HF_AT_VGM, //microphone volume control cmd_id
    HFP_HF_AT_ATD,
    HFP_HF_AT_BLDN,
    HFP_HF_AT_ATA,
    HFP_HF_AT_CHUP,
    HFP_HF_AT_BTRH,
    HFP_HF_AT_BTRH_MODE,
    HFP_HF_AT_VTS,
    HFP_HF_AT_BCC,
    HFP_HF_AT_BCS,
    HFP_HF_AT_CNUM,
    HFP_HF_AT_NREC,
    HFP_HF_AT_BINP,
    HFP_HF_AT_CBC,
    HFP_HF_AT_BIND,
    HFP_HF_AT_BIEV,
    HFP_HF_AT_BATT_UPDATE,
    HFP_HS_AT_CKPD,
    HFP_AT_EXTERN_AT_CMD
};
```

### Voice Recognition Function
- AT cmd: 
    - AT+BVRA: Voice Recognition Activation
    - Format: AT+BVRA=< status >
    - status
        - 0 (Disable Voice recognition in the AG)
        - 1 (Enable voice recognition in the AG)
        - 2 (Enable voice recognition in the AG with esco exist)
- AT cmd: 
    - +BVRA: Voice Recognition Activation
    - Format: +BVRA:< status >
    - status
        - 0 (Voice recognition is disabled in the AG)
        - 1 (Voice recognition is enabled in the AG)

HF initiates voice recognition application activation by sending AT+BVRA command to AG. Voice recognition application is installed in AG. Apart from audio routing, voice recognition activation control and other functions that need Bluetooth functionality, everything else depends on the voice recognition application implementation.
- Peer device supports voice recognition function event: BT_NOTIFY_HF_VOICE_RECOG_CAP_UPDATE
- Peer device actively uses voice recognition function status sync event: BT_NOTIFY_HF_VOICE_RECOG_STATUS_CHANGE
- Active wake up/close voice function interface:
    - bts2_app_interface voice recognition interface: bt_interface_voice_recog 
    - bts2_app_hfp_hf voice recognition interface: bt_hfp_hf_voice_recog_send 
    - Voice cmd processing result event: BT_NOTIFY_HF_AT_CMD_CFM with cmd_id (HFP_HF_AT_BVRA) 
```c
// step1: HF actively wakes up voice recognition by calling bt_interface_voice_recog(1);
// step2: HF sends AT+BVRA=1
// step3: AG receives AT+BVRA=1, sends OK/ERROR after processing
// step4: HF receives AG reply, notify event: BT_NOTIFY_HF_AT_CMD_CFM with cmd_id (HFP_HF_AT_BVRA)
// Close process is similar to open process
// When peer device actively wakes up voice recognition, HF receives notify event: BT_NOTIFY_HF_VOICE_RECOG_STATUS_CHANGE
// For specific structure explanation, please refer to relevant descriptions in interface
static int bt_notify_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    int ret = -1;

    switch (type)
    {
    case BT_NOTIFY_HFP_HF:
    {
        bt_sifli_notify_hfp_hf_event_hdl(event_id, data, data_len);
    }
    break;
    }
    return 0;
}


int bt_sifli_notify_hfp_hf_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_HF_VOICE_RECOG_CAP_UPDATE:
    {
        //handle
        break;
    }
    case BT_NOTIFY_HF_VOICE_RECOG_STATUS_CHANGE:
    {
        //handle
        break;
    }
    case BT_NOTIFY_HF_AT_CMD_CFM:
    {
        //handle at cmd result msg（bt_notify_at_cmd_cfm_t *）
        //cmd_id:HFP_HF_AT_BVRA
        break;
    }
    }
    return 0;
}
```

### Phone Control Related Functions
#### Phone Status Update Related AT cmd
- AT cmd: 
    - AT+CIND: (Standard indicator update AT command)
    - Format:
        - AT+CIND=? Test command. HF obtains indicator index values and ranges supported by AG side. Must be requested at least once before sending indicator-related commands (AT+CIND? or AT_CMER).
        - AT+CIND? Read command, HF reads current indicator values from AG side
- AT cmd: 
    - +CIEV: unsolicited result code (Standard indicator events reporting unsolicited result code)
    - Format: +CIEV: < ind >,< value >
    - When AG indicators change, AG needs to actively notify HF side using +ciev AT cmd
- Phone end active status update:
    - HF connection successful, sync phone call status event (call/callsetup/callheld): BT_NOTIFY_HF_CALL_STATUS_UPDATE
    - During connection process, actively get phone status:
        - bts2_app_interface get phone status interface: bt_interface_get_remote_call_status
        - Phone status sync event: BT_NOTIFY_HF_CALL_STATUS_UPDATE
        - Phone status cmd processing result event: BT_NOTIFY_HF_AT_CMD_CFM with cmd_id (HFP_HF_AT_CIND_STATUS)
    - During connection process, phone call status active update through event: BT_NOTIFY_HF_INDICATOR_UPDATE

#### Phone Control Related AT cmd
- AT cmd: 
    - AT+CNUM: Subscriber Number Information
    - Format: AT+CNUM
    - This command is used to query local number
    - +CNUM: Subscriber Number Information
    - Format: +CNUM: ,< number >,< type >[,, < service >]
    - After AG receives this request, it will reply the phone's local number to HF side through +CNUM.
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
- AT cmd: 
    - AT+CLCC: Standard list current calls command
    - Format: AT+CLCC
    - +CLCC: Standard list current calls command
    - Format: +CLCC: < idx >,< dir >,< status >,< mode >,< mpty >,< number >,< type >
    - HF requests current call information list, AG side replies current call information list through "+CLCC". If there are currently no calls, AG side must also reply with OK command.

#### Phone Control Function Usage
- Phone control
    - Get phone local number (AT+CNUM):
        - bts2_app_interface get local phone number interface: bt_interface_get_ph_num 
        - bts2_app_hfp_hf get local phone number interface: bt_hfp_hf_at_cnum_send 
        - Local number information event: BT_NOTIFY_HF_LOCAL_PHONE_NUMBER
        - Get local number cmd processing result message event: BT_NOTIFY_HF_AT_CMD_CFM with cmd_id (HFP_HF_AT_CNUM)
- Make call
    - Redial call (AT+BLDN)
        - bts2_app_interface redial call interface: bt_interface_start_last_num_dial_req_send 
        - bts2_app_hfp_hf redial call interface: bt_hfp_hf_last_num_dial_send 
        - Redial call processing result message event: BT_NOTIFY_HF_AT_CMD_CFM with cmd_id (HFP_HF_AT_BLDN)
- Make call by phone number (ATD10086)
    - bts2_app_interface make call by phone number interface: bt_interface_hf_out_going_call
    - bts2_app_hfp_hf make call by phone number interface: bt_hfp_hf_make_call_by_number_send
    - Make call by phone number processing result message event: BT_NOTIFY_HF_AT_CMD_CFM with cmd_id (HFP_HF_AT_ATD)
- Get all information of current active calls on phone end (AT+CLCC)
    - bts2_app_interface get all call information interface: bt_interface_get_remote_ph_num 
    - bts2_app_hfp_hf get all call information interface: bt_hfp_hf_at_clcc_send
    - Call status information event: BT_NOTIFY_HF_REMOTE_CALL_INFO_IND for notification
    - Get current active call information cmd processing result event: BT_NOTIFY_HF_AT_CMD_CFM with cmd_id (HFP_HF_AT_CLCC) 
```c
// step1: After HF connects successfully with AG, can get AG end phone number through bt_interface_get_ph_num
// step2: When AG sends local number to HF through +CNUM, and corresponding OK.
// step3: HF will receive local number (BT_NOTIFY_HF_LOCAL_PHONE_NUMBER), and BT_NOTIFY_HF_AT_CMD_CFM with cmd_id (HFP_HF_AT_CNUM) processing result
// AG end may not necessarily send local number, but will definitely have BT_NOTIFY_HF_AT_CMD_CFM with cmd_id (HFP_HF_AT_CNUM) event
// step4: When making a call, pass number length and number to bt_interface_hf_out_going_call
// step5: After AG receives phone number request, first send processing result BT_NOTIFY_HF_AT_CMD_CFM with cmd_id (HFP_HF_AT_ATD) to HF
// step6: If call is successful, HF side will receive BT_NOTIFY_HF_INDICATOR_UPDATE after this event
// step7: After user receives BT_NOTIFY_HF_INDICATOR_UPDATE, can get all current call information through bt_interface_get_remote_ph_num
// step8: AG side replies current call information list through "+CLCC". If there are currently no calls, AG side must also reply with OK command.
// step9: After HF receives information, will notify through event: BT_NOTIFY_HF_REMOTE_CALL_INFO_IND, user can process this message
static int bt_notify_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    int ret = -1;

    switch (type)
    {
    case BT_NOTIFY_HFP_HF:
    {
        bt_sifli_notify_hfp_hf_event_hdl(event_id, data, data_len);
    }
    break;
    }
    return 0;
}


int bt_sifli_notify_hfp_hf_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_HF_INDICATOR_UPDATE:
    {
        //handle msg
        break;
    }
    case BT_NOTIFY_HF_CALL_STATUS_UPDATE:
    {
        //handle msg
        break;
    }
    case BT_NOTIFY_HF_REMOTE_CALL_INFO_IND:
    {
        //handle msg
        break;
    }
    case BT_NOTIFY_HF_LOCAL_PHONE_NUMBER:
    {
        //handle msg
        break;
    }
    case BT_NOTIFY_HF_AT_CMD_CFM:
    {
        //handle at cmd result msg（bt_notify_at_cmd_cfm_t *）
        //cmd_id:HFP_HF_AT_CLCC
        //cmd_id:HFP_HF_AT_ATD
        //cmd_id:HFP_HF_AT_BLDN
        //cmd_id:HFP_HF_AT_CNUM
        break;
    }
    }
    return 0;
}
```
:::{note}
Note: HF part extensible interfaces are in bts2_app_hfp_hf, users can extend functionality according to requirements and encapsulate interfaces in interface according to needs.
Also, after each cmd is sent, need to wait for cmd_cfm message to avoid too many messages in protocol stack queue, triggering protection mechanism.
:::

## HFP_HF Call Function Usage Demo
- First, during project initialization, register the processing function for receiving notify events
- Enter the MAC address of the phone to connect to, wait for connection success message
- After receiving connection success, enter the phone number and length for making a call
```c
//register notify event handle function start
// HF side receives AG side AT cmd id processing result: 0 (success) 1 (failure) 2 (timeout, no reply from AG end)
static void bt_sifli_notify_hdl_at_cmd(uint8_t cmd_id, uint8_t res)
{
    switch (cmd_id)
    {
    //(voice recognition cmd processing result)
    case HFP_HF_AT_BVRA: 
    {
        break;
    }
    //(get current call information list processing complete)
    case HFP_HF_AT_CLCC:
    {
        //step5.request phone call information complete then hangup call
        bt_interface_handup_call();
        break;
    }
    //(AG side processing make call request result)
    case HFP_HF_AT_ATD:
    {
        //step2.make a call request result
        break;
    }
    //(AG side processing redial call request result)
    case HFP_HF_AT_BLDN:
    {
        break;
    }
    //(AG side DTMF voice key processing result)
    case HFP_HF_AT_VTS:
    {
        break;
    }
    //(AG side volume control processing result)
    case HFP_HF_AT_VGS:
    {
        break;
    }
    //(AG hang up call processing result)
    case HFP_HF_AT_CHUP:
    {
        //step6.hangup call over and recv phone call status change:BT_NOTIFY_HF_INDICATOR_UPDATE
        break;
    }
    default:
        break;
    }
}

int bt_sifli_notify_hfp_hf_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    // SLC CONNECTED
    case BT_NOTIFY_HF_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //step1.make a call request
        char *phone_number = "10086";
        bt_interface_hf_out_going_call(strlen(phone_number),phone_number);
        break;
    }
    //SLC DISCONNECTED
    case BT_NOTIFY_HF_PROFILE_DISCONNECTED: 
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        break;
    }
    // notify whether AG supports voice recognition function
    case BT_NOTIFY_HF_VOICE_RECOG_CAP_UPDATE:
    {
        break;
    }
    // notify AG voice recognition function status
    case BT_NOTIFY_HF_VOICE_RECOG_STATUS_CHANGE: 
    {
        break;
    }
    // notify local number
    case BT_NOTIFY_HF_LOCAL_PHONE_NUMBER:
    {
        break;
    }
    // notify all current call information
    case BT_NOTIFY_HF_REMOTE_CALL_INFO_IND:
    {
        bt_notify_clcc_ind_t *clcc_info = (bt_notify_clcc_ind_t *)data;
        //step4.recv phone call information
        break;
    }
    // volume change
    case BT_NOTIFY_HF_VOLUME_CHANGE:
    {
        break;
    }
    // all call status
    case BT_NOTIFY_HF_CALL_STATUS_UPDATE:
    {
        bt_notify_all_call_status *call_status = (bt_notify_all_call_status *)data;
        break;
    }
    // call status change
    case BT_NOTIFY_HF_INDICATOR_UPDATE:
    {
        bt_notify_cind_ind_t   *cind_status = (bt_notify_cind_ind_t *)data;
        //step3.recv phone call status change and then get call information
        bt_interface_get_remote_call_status();
        break;
    }
    case BT_NOTIFY_HF_AT_CMD_CFM:
    {
        bt_notify_at_cmd_cfm_t *at_cmd_cfm = (bt_notify_at_cmd_cfm_t *) data;
        bt_sifli_notify_hdl_at_cmd(at_cmd_cfm->at_cmd_id, at_cmd_cfm->res);
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

    case BT_NOTIFY_HFP_HF:
    {
        bt_sifli_notify_hfp_hf_event_hdl(event_id, data, data_len);
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

// start connect with phone:001bdcf4b6bd
unsigned char mac[6] = {0x00,0x1b,0xdc,0xf4,0xb6,0xbd}
bt_interface_conn_ext((unsigned char *)(mac), BT_PROFILE_HFP);

//
```