# BT_AVRCP
AVRCP (Audio/Video Remote Control Profile) defines the characteristics and processes for communication between Bluetooth devices and audio/video control functions, and is also used for remote control of audio/video devices. The underlying transport is based on the AVCTP transport protocol. This Profile defines the AV/C digital command control set. Commands and information are transmitted through the AVCTP (Audio/Video Control Transport Protocol) protocol. Browsing functionality is through the second channel of AVCTP rather than AV/C. Media information transmission is through the BIP (Bluetooth Basic Imaging Profile) protocol based on the OBEX protocol.

## AVCTP Introduction
1. AVCTP Protocol Architecture
The AVCTP protocol describes the format and mechanism for exchanging Audio/Video control signals between Bluetooth devices. It is a general protocol, with specific control information implemented by its designated protocols (such as AVRCP). AVCTP itself only specifies the general format of control commands and responses.

Several important points:
&emsp;&emsp;(1) AVCTP is based on point-to-point L2CAP connections.
&emsp;&emsp;(2) Each AVCTP connection should support both AVRCP CT and TG functionality.
&emsp;&emsp;(3) There may be multiple AVCTP connections between two devices, but each AVCTP connection has a unique PSM value.
&emsp;&emsp;(4) Each AVCTP packet should be transmitted in one L2CAP packet.
&emsp;&emsp;(5) The same transaction Label on different L2CAP channels belongs to different messages. That is, packets on two L2CAP channels are unrelated and cannot belong to the same message.
![AVCTP Architect](../../assets/avctp_arch.png)

2. AVCTP Packet Format
AVCTP packet format is divided into two types:
&emsp;&emsp;1) Unfragmented (smaller than L2CAP MTU)
&emsp;&emsp;2) Fragmented (larger than L2CAP MTU)
The following introduces the format of each type:
&emsp;&emsp;1) Unfragmented (smaller than L2CAP MTU)
![AVCTP Packet](../../assets/avctp_packet.png)
   - Transaction label field: (octet 0, bits 7-4) Transport identifier, provided by upper layer;

   - Packet_type field: (octet 0, bits 3 and 2) This part 00b indicates unfragmented;

   - C/R: (octet 0, bit 1) 0 represents command, 1 represents response

   - PID: bit (octet 0, bit 0) Set to 0 in command, set to 0 in response represents normal PID

   - Profile Identifier (PID): This part fills in 16-bit UUID, such as AVRCP UUID 0x110e. The subsequent Message Information is the upper layer protocol data

&emsp;&emsp;&emsp;&emsp;2) Fragmented (larger than L2CAP MTU)
Fragmented data packet format has three types in total:
&emsp;&emsp;&emsp;&emsp;![AVCTP Start Packet](../../assets/avctp_start_packet.png)
&emsp;&emsp;&emsp;&emsp;![AVCTP Continue Packet](../../assets/avctp_continue_packet.png)
&emsp;&emsp;&emsp;&emsp;![AVCTP End Packet](../../assets/avctp_end_packet.png)
&emsp;&emsp;&emsp;&emsp;Here we only introduce two aspects, others are consistent with unfragmented:
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Packet_type: Start packet is 01b, continue packet 10b, end packet is 11b
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Number of AVCTP Packets: Indicates the total number of fragmented packets, including this start packet, so the number of this packet must be greater than 1.

## AVRCP Introduction
1. AVRCP Protocol Architecture
![AVRCP ARCH](../../assets/avrcp_arch.png)
AVRCP Profile defines two roles:
    - CT (controller devices)
      Controller is a device that initiates transactions by sending command frames to target devices, such as headphones, speakers, car Bluetooth devices.
    - TG (target devices)
      Target is a device that receives command frames sent by the controller and generates corresponding response frames, such as mobile phones.

2. AVRCP Connection
AVCTP connection is divided into two channels: Control channel and Browsing channel, with different corresponding L2CAP PSMs. The control channel PSM is 0x0017, AVCTP browsing channel PSM is 0x001B. Both controller and target roles can initiate connections on both channels. Note that AVCTP browsing part based on L2CAP cannot use basic mode, Enhanced Retransmission Mode is required, and AVCTP browsing channel connection can only be initiated when both sides support AVCTP browsing.
The entire establishment process follows these flows:
![AVRCP Controller Connect](../../assets/avrcp_connect_ct.png)
![AVRCP Target Connect](../../assets/avrcp_connect_tg.png)
![AVRCP Mult Connect](../../assets/avrcp_connect_mult.png)

3. AVRCP Disconnection
Disconnection can be operated from AVRCP controller or target role. Note that if there is an AVCTP browsing connection, disconnect the AVCTP browsing channel connection first. The entire process is as follows:
![AVRCP Disconnect](../../assets/avrcp_disconnect.png)

4. AVRCP AV/C Command Format
&emsp;4.1 VENDOR DEPENDENT Format
&emsp;&emsp;1) Command header format:
&emsp;&emsp;![AVRCP Vendor Command](../../assets/avrcp_vendor_command.png)
&emsp;&emsp;Ctype: This is the command type, divided into 5 types in total. Generally, we use 3 types in AVRCP protocol: CONTROL control command, STATUS get status command, NOTIFY notification command. The types are as follows:
&emsp;&emsp;![AVRCP Vendor Ctype](../../assets/avrcp_vendor_ctype.png)
&emsp;&emsp;Subunit_type: Subunit type, with the following definitions. Generally use PANEL in AVRCP
&emsp;&emsp;![AVRCP Vendor Subunit](../../assets/avrcp_vendor_subunit.png)
&emsp;&emsp;Subunit_id: This part we generally fill 0
&emsp;&emsp;Opcode: We only need to know what the specific command opcode is. VENDOR DEPENDENT value is 0.
&emsp;&emsp;Company ID: This part needs to fill in the Bluetooth SIG ID
&emsp;&emsp;2) Response header format:
&emsp;&emsp;![AVRCP Response Header](../../assets/avrcp_response_header.png)
&emsp;&emsp;As you can see, command and response formats are exactly the same. We won't repeat other parameters. Let's talk about Response values:
&emsp;&emsp;![AVRCP Response status](../../assets/avrcp_response_status.png)

&emsp;4.2 PASS THROUGH Format
&emsp;&emsp;1) Command:
&emsp;&emsp;![AVRCP Pass Through Command](../../assets/avrcp_pass_through_command.png)

&emsp;&emsp;1) Response:
&emsp;&emsp;![AVRCP Pass Through Response](../../assets/avrcp_pass_through_response.png)

&emsp;&emsp;We won't introduce others in detail, only Operation_ID and state_flag
&emsp;&emsp;&emsp;Operation_ID: Operation ID
&emsp;&emsp;&emsp;![AVRCP Pass Through Operation ID](../../assets/avrcp_pass_through_operation_id.png)
&emsp;&emsp;&emsp;![AVRCP Pass Through Operation ID](../../assets/avrcp_pass_through_operation_id_1.png)
&emsp;&emsp;&emsp;![AVRCP Pass Through Operation ID](../../assets/avrcp_pass_through_operation_id_2.png)
&emsp;&emsp;&emsp;![AVRCP Pass Through Operation ID](../../assets/avrcp_pass_through_operation_id_3.png)

&emsp;&emsp;&emsp;State_flag: Simply put, it's divided into press and release actions. When sending, this value is 0 for press and 1 for release

&emsp;4.3 AVRCP Specific Commands
&emsp;&emsp;AVRCP specific commands include the following:
&emsp;&emsp;![AVRCP Special Command](../../assets/avrcp_special_command_0.png)
&emsp;&emsp;![AVRCP Special Command](../../assets/avrcp_special_command_1.png)
&emsp;&emsp;![AVRCP Special Command](../../assets/avrcp_special_command_2.png)
&emsp;&emsp;![AVRCP Special Command](../../assets/avrcp_special_command_3.png)
&emsp;&emsp;![AVRCP Special Command](../../assets/avrcp_special_command_4.png)
&emsp;&emsp;![AVRCP Special Command](../../assets/avrcp_special_command_5.png)
&emsp;&emsp;![AVRCP Special Command](../../assets/avrcp_special_command_6.png)

&emsp;&emsp;1) Get Capabilities
&emsp;&emsp;![AVRCP Get Capability](../../assets/avrcp_get_capability.png)
&emsp;&emsp;This is sent by AVRCP controller to target to get the capabilities of the peer device, including company name and supported events.
&emsp;&emsp;![AVRCP Get Capability Packet](../../assets/avrcp_get_capability_packet.png)
&emsp;&emsp;Get company ID response format:
&emsp;&emsp;![AVRCP Get Capability Company](../../assets/avrcp_get_capability_cpmpany.png)
&emsp;&emsp;Get supported event response format:
&emsp;&emsp;![AVRCP Get Capability Event](../../assets/avrcp_get_capability_event.png)
&emsp;&emsp;Where EventID includes:
&emsp;&emsp;![AVRCP Event Support](../../assets/avrcp_event_support.png)
&emsp;&emsp;In practical applications, we generally only use Get Capability for event to get the events supported by the peer, facilitating notify registration. As you can see in the above figure, it supports playback status change, track change, and player setting change.

&emsp;&emsp;2) Get PlayStatus
&emsp;&emsp;This command is sent by controller to target to get playback status. The return value includes total song length, current song progress, and playback status, totaling 9 bytes (4 bytes song length, 4 bytes current progress, 1 byte playback status)
&emsp;&emsp;![AVRCP Get Play Status](../../assets/avrcp_get_play_status.png)

&emsp;&emsp;3) Register Notification
&emsp;&emsp;This is CT registering messages with TG, then TG will notify when there are corresponding updates. The specific messages are those we mentioned earlier in get capability with event (Note: each registration is only valid once, need to re-register after receiving change):
&emsp;&emsp;![AVRCP Register Notify](../../assets/avrcp_register_notify.png)
&emsp;&emsp;![AVRCP Register Notify](../../assets/avrcp_register_notify_1.png)

&emsp;&emsp;Let's talk about each EVENT ID below:
&emsp;&emsp;EVENT_PLAYBACK_STATUS_CHANGED: Playback status change, with the following values:
&emsp;&emsp;![AVRCP Play Status Register Notify](../../assets/avrcp_play_status_notify.png)
&emsp;&emsp;EVENT_TRACK_CHANGED: Track name change
&emsp;&emsp;EVENT_TRACK_REACHED_END: Track reached end
&emsp;&emsp;EVENT_TRACK_REACHED_START: Track start
&emsp;&emsp;EVENT_PLAYBACK_POS_CHANGED: Track playback progress change
&emsp;&emsp;![AVRCP Playback Pos Change Notify](../../assets/avrcp_playback_pos_change.png)
&emsp;&emsp;EVENT_BATT_STATUS_CHANGED: Battery status change, with the following values:
&emsp;&emsp;![AVRCP Batt Status Change Notify](../../assets/avrcp_batt_status_change.png)

&emsp;&emsp;4) Metadata Attributes for Current Media Item
&emsp;&emsp;This is mainly for getting song information (including name/album name/artist name/song index/total number of songs, etc.)
&emsp;&emsp;![AVRCP Get Element](../../assets/avrcp_get_element.png)
&emsp;&emsp;AttributeID list is as follows:
&emsp;&emsp;![AVRCP Get Element Attribute](../../assets/avrcp_get_element_attribute.png)

# Sifli SDK AVRCP Introduction
This document is mainly based on Sifli SDK, introducing how to use basic AVRCP functions.
Related files include:
- bts2_app_interface
- bts2_app_avrcp
  
## bt_avrcp Initialization
- AVRCP initialization function: bt_avrcp_int, initializes AVRCP related states and assigns initial values to flags
- AVRCP service enable function: bt_avrcp_open, enables AVRCP profile

```c
//AVRCP states
typedef enum
{
    avrcp_idle,
    avrcp_conned
} bts2_avrcp_st;

//AVRCP initialization
//Users can add their own initialization process in this interface, but it is not recommended to delete related code
void bt_avrcp_int(bts2_app_stru *bts2_app_data)
{
    //Initialize AVRCP state to idle
    bts2_app_data->avrcp_inst.st = avrcp_idle;
    //release_type is used to record which command's push is currently being sent
    bts2_app_data->avrcp_inst.release_type = 0x00;
    bts2_app_data->avrcp_inst.avrcp_time_handle = NULL;
    bts2_app_data->avrcp_inst.avrcp_vol_time_handle = NULL;
    bts2_app_data->avrcp_inst.volume_change_sem = rt_sem_create("bt_avrcp_vol_change", 1, RT_IPC_FLAG_FIFO);
    //Record whether the peer has registered support for absolute volume
    bts2_app_data->avrcp_inst.tgRegStatus = 0;
    //Record whether the peer has registered playback status change notification events
    bts2_app_data->avrcp_inst.tgRegStatus1 = 0;
    //Flag indicating whether volume is being adjusted
    bts2_app_data->avrcp_inst.abs_volume_pending = 0;
    //AVRCP playback status
    bts2_app_data->avrcp_inst.playback_status = 0;
    //Initial absolute volume, will notify the peer of this end's initial volume when AVRCP is connected and the peer supports absolute volume
    bts2_app_data->avrcp_inst.ab_volume = 20;//default value;
    //Can control whether to enable AVRCP when initializing AVRCP through macro CFG_OPEN_AVRCP
#ifdef CFG_OPEN_AVRCP
    bts2s_avrcp_openFlag = 1;
#else
    bts2s_avrcp_openFlag = 0;
#endif
    //enable AVRCP
    if (1 == bts2s_avrcp_openFlag)
    {
        avrcp_enb_req(bts2_app_data->phdl, AVRCP_CT);
    }

    //Mainly used to record some current playback information and song information
    memset(&mp3_detail_info, 0x00, sizeof(bt_avrcp_mp3_detail_t));
    mp3_detail_info.track_id = 0xff;
}

//Interface to enable AVRCP
void bt_avrcp_open(void)
{
    bts2_app_stru *bts2_app_data = bts2g_app_p;

    USER_TRACE("bt_avrcp_open %d flag\n", bts2s_avrcp_openFlag);

    if (0 == bts2s_avrcp_openFlag)
    {
        bts2s_avrcp_openFlag = 0x01;
        avrcp_enb_req(bts2_app_data->phdl, AVRCP_CT);
    }
    else
    {
#if defined(CFG_AVRCP)
        bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_OPEN_COMPLETE, NULL, 0);
        INFO_TRACE(">> URC AVRCP open,alreay open\n");
#endif
    }
}
```

## AVRCP Device Connection
The following process describes how AVRCP CT discovers, connects to, and uses AVRCP TG services.
1. The first step is to discover available AVRCP TG devices in the vicinity. For this, AVRCP CT can perform a search on nearby devices, then use SDP to retrieve AVRCP TG services from those devices that support the AVRCP TG role.
2. Select an AVRCP TG device to connect to. Choose the device to connect to and initiate an ACL connection.
3. AVRCP connection. Once the ACL connection is established, AVRCP CT should initiate the AVRCP L2CAP connection.
4. After AVRCP is connected, it needs to first query the capabilities supported by the peer (the peer may also query the capabilities supported by this end and register corresponding notification events), then register corresponding notification events before using corresponding functions (such as controlling music pause/play, volume adjustment, etc.).
5. AVRCP TG and AVRCP CT can terminate the connection at any time.

- AVRCP device connection interface:
    - bts2_app_interface connection interface: bt_interface_conn_ext
    - bts2_app_avrcp connection interface: bt_avrcp_conn_2_dev
```c
/**
* @brief            Initiate connect with the specified device and profile(hf sink)
* @param[in] mac    Remote device address
* @param[in] ext_profile   Profile value
*
* @return           bt_err_t
**/
bt_err_t bt_interface_conn_ext(unsigned char *mac, bt_profile_t ext_profile);

//profile definition
typedef enum
{
    BT_PROFILE_HFP = 0,                /* HFP Profile */
    BT_PROFILE_AVRCP,                  /* AVRCP Profile */
    BT_PROFILE_A2DP,                   /* A2DP Profile */
    BT_PROFILE_PAN,                    /* PAN Profile */
    BT_PROFILE_HID,                    /* HID Profile */
    BT_PROFILE_AG,                     /* AG Profile */
    BT_PROFILE_SPP,                    /* SPP Profile */
    BT_PROFILE_BT_GATT,                /* BT_GATT Profile */
    BT_PROFILE_PBAP,                   /* PBAP Profile */
    BT_PROFILE_MAX
} bt_profile_t;

//Before calling this interface, the mac will be converted to BTS2S_BD_ADDR Bluetooth address format
//AVRCP CT and TG share one connection interface, need to pass in the connection role
//Users only need to call bt_interface_conn_ext, no need to modify this interface
void bt_avrcp_conn_2_dev(BTS2S_BD_ADDR *bd, BOOL is_target)
{
    bts2_app_stru *bts2_app_data = bts2g_app_p;

    //Will check if in idle state
    if (bts2_app_data->avrcp_inst.st == avrcp_idle)
    {
        USER_TRACE(" -- avrcp conn remote device...\n");
        USER_TRACE(" -- address: %04X:%02X:%06lX\n",
                bd->nap,
                bd->uap,
                bd->lap);

        //Determine if this end's role is CT or TG
        if (!is_target)
        {
            avrcp_conn_req(bts2_app_data->phdl, *bd, AVRCP_TG, AVRCP_CT);
        }
        else
        {
            avrcp_conn_req(bts2_app_data->phdl, *bd, AVRCP_CT, AVRCP_TG);
        }
    }
    else
    {
        USER_TRACE(" -- already connected with remote device\n");
    }
}
```
       
- AVRCP device disconnection interface:
    - bts2_app_interface disconnection interface: bt_interface_disc_ext
    - bts2_app_avrcp disconnection interface: bt_avrcp_disc_2_dev
```c
/**
* @brief            Disconnect with the specified profile
* @param[in] mac    Remote device address
* @param[in] ext_profile : Profile value
*
* @return           bt_err_t
**/
bt_err_t bt_interface_disc_ext(unsigned char *mac, bt_profile_t ext_profile);

//Before calling this interface, the mac will be converted to BTS2S_BD_ADDR Bluetooth address format
//Users only need to call bt_interface_disc_ext, no need to modify this interface
void bt_avrcp_disc_2_dev(BTS2S_BD_ADDR *bd_addr)
{
    bts2_app_stru *bts2_app_data = getApp();
    //Will first check if in connected state
    if (avrcp_conned == bts2_app_data->avrcp_inst.st)
    {
        USER_TRACE(">> disconnect avrcp with remote device...\n");
        avrcp_disc_req();
        bts2_app_data->avrcp_inst.st = avrcp_idle;
    }
    else
    {
        USER_TRACE(">> disconnect avrcp,already idle\n");
    }
}
```
        
- AVRCP event handling:
    - AVRCP connection status callback events:
        - AVRCP connection successful: BT_NOTIFY_AVRCP_PROFILE_CONNECTED
        - AVRCP connection failed: BT_NOTIFY_AVRCP_PROFILE_DISCONNECTED

:::{note}
Note: The address parameters passed by the two interfaces need to be converted accordingly.
:::
```c
// After calling the API to connect AVRCP, the AVRCP connection successful message is sent to the user through notify
// Users need to implement hdl functions to receive notify events, such as: bt_notify_handle
// BT_NOTIFY_AVRCP_PROFILE_CONNECTED event includes: address information, profile_type, res: 0 (success)
// After disconnecting AVRCP, corresponding events will also be received
// BT_NOTIFY_AVRCP_PROFILE_DISCONNECTED event includes: address information, profile_type, reason
// For specific structure information, refer to API comments
static int bt_notify_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    int ret = -1;

    switch (type)
    {
        //Register notify functions to handle AVRCP related events
        case BT_NOTIFY_AVRCP:
        {
            bt_sifli_notify_avrcp_event_hdl(event_id, data, data_len);
        }
        break;
    }
    return 0;
}

//Users need to register A2DP notify functions to handle various events
static int bt_sifli_notify_avrcp_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_AVRCP_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //Users implement their own corresponding processing functions
        break;
    }
    case BT_NOTIFY_AVRCP_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //Users implement their own corresponding processing functions
        break;
    }
    default:
        return -1;
    }
    return 0;
}
```

## AVRCP Basic Functionality Usage
### 1. Music Control Commands
```c
//Music play command
void bt_interface_avrcp_play(void);

//Music pause command
void bt_interface_avrcp_pause(void);

//Next track command
void bt_interface_avrcp_next(void);

//Previous track command
void bt_interface_avrcp_previous(void);

//Rewind command
void bt_interface_avrcp_rewind(void);

//Volume adjustment
bt_err_t bt_interface_avrcp_volume_changed(U8 volume);

//If this end acts as TG and the peer device supports absolute volume, the following interface can be used to adjust absolute volume
bt_err_t bt_interface_set_absolute_volume(U8 volume);
```
### 2. The following interface can be used to modify the capabilities supported by this end device, and peer devices will register corresponding events based on the replied response
```c
// notification events id
#define AVRCP_VENDOR_DEPENDENT_EVENT_PLAYBACK_STATUS_CHANGED 0x01
#define AVRCP_VENDOR_DEPENDENT_EVENT_TRACK_CHANGED 0x02
#define AVRCP_VENDOR_DEPENDENT_EVENT_PLAYBACK_POS_CHANGED 0x05
#define AVRCP_VENDOR_DEPENDENT_EVENT_VOLUME_CHANGED 0x0D

void bt_avrcp_get_capabilities_response(bts2_app_stru *bts2_app_data, int tlabel)
{
    U8 data_len = 12;
    U8 data[12];

    memcpy(data, VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, 4);
    *(data + 4) = AVRCP_VENDOR_DEPENDENT_PDU_ID_GET_CAPABILITIES;
    *(data + 5) = 0;

    // parameter length
    *(data + 6) = 0;
    *(data + 7) = 3;

    *(data + 8) = AVRCP_VENDOR_DEPENDENT_EVENT_CAPABILITY_FOR_EVENTS;
    //Modify the number of capabilities and assign values accordingly to add/remove the number of events supported by this end device
    *(data + 9) = 2;
    // event ID volume changed
    *(data + 10) = AVRCP_VENDOR_DEPENDENT_EVENT_PLAYBACK_STATUS_CHANGED;
    *(data + 11) = AVRCP_VENDOR_DEPENDENT_EVENT_VOLUME_CHANGED;

    avrcp_cmd_data_rsp(bts2_app_data->phdl,
                    tlabel,
                    BT_UUID_AVRCP_CT,
                    AVRCP_CR_STABLE,
                    AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE,
                    AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID,
                    data_len,
                    data);
}
```
### 3. Music Information Related Commands
```c
1. After getting the events supported by the peer, it will be reported through BT_NOTIFY_AVRCP_CAPABILITIES_CFM. Customers can call the following interfaces to enable corresponding event notify based on the received events and requirements:
//Register playback status notify event, peer will notify this end device when playback status changes
void bt_interface_avrcp_playback_register_request(void);

//Register playback pos change notify event, peer will notify this end device when playback progress changes
void bt_interface_playback_pos_register_request(void);

//Register track change notify event, peer will notify this end device when playing track changes
void bt_interface_track_change_register_request(void);

//Register volume change notify event, peer will notify this end device when playback volume changes
void bt_interface_volume_change_register_request(void);

2. Get song information (e.g., song name, artist, album, etc.)
//media attribute type
#define AVRCP_MEDIA_ATTRIBUTES_TITLE 0x01   //Song name
#define AVRCP_MEDIA_ATTRIBUTES_ARTIST 0x02  //Artist name
#define AVRCP_MEDIA_ATTRIBUTES_ALBUM 0x03   //Album
#define AVRCP_MEDIA_ATTRIBUTES_GENRE 0x06   //Genre
#define AVRCP_MEDIA_ATTRIBUTES_PLAYTIME 0x07 //Song length

//Get corresponding song information through this interface
void bt_interface_avrcp_get_element_attributes_request(U8 media_attribute);
```
### 4. Get Peer's Playback Status
void bt_interface_avrcp_get_play_status_request(void);

## AVRCP Functionality Usage Demo
- First, during project initialization, register the processing function for receiving notify events
- Enter the MAC address of the phone to connect to, wait for connection success message
- Play phone music, control phone play/pause
```c
int bt_sifli_notify_avrcp_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    // AVRCP connection event
    case BT_NOTIFY_AVRCP_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //Users implement their own corresponding processing functions
        break;
    }
    // AVRCP disconnection event
    case BT_NOTIFY_AVRCP_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //Users implement their own corresponding processing functions
        break;
    }
    // After getting music information of the peer device currently playing
    case BT_NOTIFY_AVRCP_MP3_DETAIL_INFO:
    {
        bt_notify_avrcp_mp3_detail_t *mp3_detail_info = (bt_notify_avrcp_mp3_detail_t *)data;
        //Users implement their own corresponding processing functions
        break;
    }
    // Received absolute volume registration request event from peer device
    case BT_NOTIFY_AVRCP_VOLUME_CHANGED_REGISTER:
    {
        //Users implement their own corresponding processing functions
        break;
    }
    // Received absolute volume adjustment from peer device
    case BT_NOTIFY_AVRCP_ABSOLUTE_VOLUME:
    {
        uint8_t *relative_volume = (uint8_t *)data;
        //Users implement their own corresponding processing functions
        break;
    }
    // Playback status change notification event
    case BT_NOTIFY_AVRCP_PLAY_STATUS:
    {
        uint8_t *play_status_notify = (uint8_t *)data;
        //Users implement their own corresponding processing functions
        break;
    }
    // Playback progress change notification event
    case BT_NOTIFY_AVRCP_SONG_PROGREAS_STATUS:
    {
        uint32_t *play_pos = (uint32_t *)data;
        //Users implement their own corresponding processing functions
        break;
    }
    // Playing track change notification event
    case BT_NOTIFY_AVRCP_TRACK_CHANGE_STATUS:
    {
        uint8_t *track_change = (uint8_t *)data;
        //Users implement their own corresponding processing functions
        break;
    }
    // After getting peer's supported capabilities notification event
    case BT_NOTIFY_AVRCP_CAPABILITIES_CFM:
    {
        bt_notify_avrcp_capabilities_cfm_t *track_change = (bt_notify_avrcp_capabilities_cfm_t *)data;
        //Users implement their own corresponding processing functions, based on peer's supported capabilities, customers can decide which event notify to register
        break;
    }
    // After getting peer's playing music information notification event (e.g., artist name, song name, etc.)
    case BT_NOTIFY_AVRCP_MEDIA_ATTRIBUTE_CFM:
    {
        bt_notify_avrcp_media_attribute_cfm_t *track_change = (bt_notify_avrcp_media_attribute_cfm_t *)data;
        //Users implement their own corresponding processing functions, based on media_attribute can determine what information was obtained
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

    case BT_NOTIFY_AVRCP:
    {
        bt_sifli_notify_avrcp_event_hdl(event_id, data, data_len);
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

// 1. Connect phone A2DP: 001bdcf4b6bd
unsigned char mac[6] = {0x00,0x1b,0xdc,0xf4,0xb6,0xbd}
bt_interface_conn_ext((unsigned char *)(mac), BT_PROFILE_A2DP);
// 2. Connect phone AVRCP
bt_interface_conn_ext((unsigned char *)(mac), BT_PROFILE_AVRCP);
// 3. Phone plays music, control phone music pause
bt_interface_avrcp_pause();
@endcode
*/
```