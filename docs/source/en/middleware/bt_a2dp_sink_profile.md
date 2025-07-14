# BT_A2DP
A2DP (Advanced Audio Distribution Profile) is a Bluetooth audio transmission protocol, typically used for Bluetooth headsets. A2DP protocol audio data is transmitted over ACL Link, which is different from voice data transmitted over SCO. A2DP does not include remote control functionality; for remote control functionality, refer to the AVRCP protocol. AVDTP defines the parameter negotiation, establishment, and transmission process of data stream handles between Bluetooth devices, as well as the forms of signaling entities exchanged. This protocol is the foundational protocol of the A2DP framework.

A2DP aims to transmit high-quality audio streams over Bluetooth connections. The basic compression algorithm it uses is SBC, which is used to reduce the size of audio data while maintaining high sound quality. SBC is a mandatory compression algorithm that must be supported. In addition to SBC, A2DP can also support other advanced codecs such as AAC and LHDC, but support for these codecs depends on the device itself.

The implementation of A2DP depends on GAVDP and GAP, and the stream connection establishment process is defined in GAVDP. The protocol dependencies of A2DP are shown in the following figure:
![A2DP Stack](../../assets/a2dp_arch.png)

In A2DP, two different roles are defined: SRC (source) and SNK (sink). SRC sends audio data, and SNK receives audio data. Their corresponding protocol models are as follows:
![A2DP Stack](../../assets/a2dp_stack.png)

The main roles included in the A2DP Profile are:
- source(SRC): Device that sends audio data
- sink(SNK): Device that receives audio data

A2DP is built on top of the AVDTP transport protocol. AVDTP specifies how connections are established. Once the connection is established, audio data can be sent and received after compression. Audio data is bidirectional. The data transmission and reception process of A2DP based on AVDTP is shown in the following figure:
![A2DP Stack](../../assets/a2dp_transport_data.png)

Here's a general summary of the entire process:
1. UL collects PCM data, then sends it to A2DP. A2DP compresses it into a specific audio format through codec PCM lib (SBC, MPEG-1,2 Audio, MPEG-2,4 AAC, ATRAC family or custom encoder pcm lib), then passes it to AVDTP. AVDTP forwards it to L2CAP, L2CAP forwards it to HCI through ACL format, then reaches the BT chip and is transmitted out through RF.
2. The BT chip receives data through RF, then passes it to HCI through ACL, then to L2CAP. L2CAP passes it to AVDTP, AVDTP passes it to A2DP. A2DP receives compressed data from remote, then decompresses it into PCM data through codec pcm lib (SBC, MPEG-1,2 Audio, MPEG-2,4 AAC, ATRAC family or custom encoder pcm lib), and then passes it to the sound card for playback.

## AVDTP Introduction
AVDTP (A/V Distribution Transport Protocol) is the audio/video distribution transport protocol, mainly used for transmitting audio/video data. The architecture diagram in the entire protocol stack is as follows:
![A2DP Stack](../../assets/avdtp_arch.png)
1. AVDTP Terminology Introduction
   - Stream: Streaming media data between two point-to-point devices.
   - Source (SRC) and Sink (SNK): SRC is the sender of audio/video, SNK is the receiver of audio/video.
   - Initiator (INT) and Acceptor (ACP): The device that initiates the process acts as the initiator, and the device that accepts the initiation is the acceptor. Note that INT and ACP are independent of the SRC and SNK defined by the upper-layer application. In a CMD and RESPONSE exchange, the one sending CMD is the INT role, and the one returning RESPONSE is the ACP role, so their roles will be constantly switching dynamically.
   - Application and Transport Service Capabilities: Functions of application services and transport services. Application service capabilities include negotiation, configuration of audio source device codec, content protection system, etc.; transport service capabilities include datagram segmentation and reassembly, packet loss detection, etc.
   - Services, Service Categories, and Service Parameters: Services, service categories, and service parameters.
   - Media Packets, Recovery Packets, and Reporting Packets: Streaming media packets, data recovery packets, and report packets.
   - Stream End Point (SEP): Stream endpoint, an application that exposes available transport services and A/V capabilities to negotiate a stream.
   - Stream Context (SC): Stream context. Refers to a common understanding of stream configuration reached by two peer devices during stream setup, including selected services, parameters, and transport channel allocation.
   - Stream Handle (SH): Stream handle. An independent identifier allocated after the connection is established between SRC and SNK, representing the upper layer's reference to the stream.
   - Stream End Point Identifier (SEID): Stream endpoint identifier, cross-device reference to a specific device.
   - Stream End Point State: Stream endpoint state.
   - Transport Session: Transport session. Within the A/V transport layer, between paired AVDTP entities, streams can be decomposed into one, two, or up to three transport sessions.
   - Transport Session Identifier (TSID): Transport session identifier. Represents a reference to a transport session.
   - Transport Channel: Transport channel. Transport channel refers to the abstraction of the lower-layer bearer program of the A/V transport layer, always corresponding to the L2CAP channel.
   - Transport Channel Identifier (TCID): Transport channel identifier. Represents a reference to a transport channel.
   - Reserved for Future Additions (RFA): Reserved for future additions.
   - Reserved for Future Definitions (RFD): Reserved for future definitions.
  
2. AVDTP Packet Format
AVDTP mainly has two types of packets: signal packets on the signal channel and media packets on the media channel.
![A2DP Stack](../../assets/avdtp_signal_packet.png)
The above is the Signal header format, which can be seen to have 3 packet formats:
&emsp;&emsp;1) Single packet
&emsp;&emsp;2) Start packet, generally used for the first packet when packet size > MTU
&emsp;&emsp;3) Continue packet and end packet, generally used for continue and end packets when packet size > MTU
Let's explain the parameters:
   - Transaction Label: Transport label, 4 bits, filled by INT role, ACP must return the same value
   - Packet Type: Packet type, with the following types:
![A2DP Stack](../../assets/avdtp_transaction_label.png)
Message Type: Message type, with the following types:
![A2DP Stack](../../assets/avdtp_message_type.png)
Signal Identifier: Signaling identifier, with the following values:
![A2DP Stack](../../assets/avdtp_signal_identifer.png)
NOSP = Number Of Signal Packets: Start packet will indicate how many subsequent packets need to be transmitted

3. AVDTP Signal Commands
There are the following types of signaling commands:
![A2DP Stack](../../assets/avdtp_signal_identifer.png)

Before explaining specific commands, let's first explain Service Capabilities, as these will be used, so we'll explain them in advance.
The Service Capabilities format is as follows:
![A2DP Stack](../../assets/avdtp_service_capability.png)
This part is also similar to TLV (Type Length Value) type, where Service Category is the TYPE, with the following values:
![A2DP Stack](../../assets/avdtp_service_capability_value.png)
Length Of Service Capabilities (LOSC) is similar to length, which is the length of subsequent Service Capabilities Information Elements.
Service Capabilities Information Elements are specific values. This requires extensive discussion. Here we'll mainly explain Media Codec Capabilities.
![A2DP Stack](../../assets/avdtp_media_codec_capability.png)
Media Type has the following values:
![A2DP Stack](../../assets/avdtp_media_type.png)
Media Codec Type has the following values:
![A2DP Stack](../../assets/avdtp_media_codec.png)
The Media Codec Specific Information Elements in this part are some upper-layer codec information, for example, the following figure shows SBC:
![A2DP Stack](../../assets/avdtp_sbc_media_codec.png)

- 3.1 AVDTP Signal Commands
Each AVDTP endpoint will register one or more SEPs, identified by SEID. This command is to obtain the peer's SEP information, including SEID (SEP ID), In Use (whether in use), Media Type (Audio, Media, MultiMedia), TSEP (role is Sink or Source)
The process is as follows:
![A2DP Stack](../../assets/avdtp_discover.png)

- 3.2 Get Capabilities
This command uses SEID to obtain the peer's Capabilities. Capabilities have been introduced previously. Let's look at the program flow:
![A2DP Stack](../../assets/avdtp_get_capability.png)

- 3.3 Get All Capabilities
This command is used to replace Get Capabilities, also using SEID to obtain the peer's Capabilities. Capabilities have been introduced previously. Let's look at the program flow:
![A2DP Stack](../../assets/avdtp_get_all_capability.png)

- 3.4 Set Configuration Command
After obtaining Capabilities, this part is to select specific function parameters. The program flow is as follows:
![A2DP Stack](../../assets/avdtp_set_configure.png)

- 3.5 Get Stream Configuration
This command is used to obtain configuration based on SEID. The program flow is as follows:
![A2DP Stack](../../assets/avdtp_get_configure.png)

- 3.6 Stream Establishment
This command is used to open a SEID, i.e., establish media channel. The program flow is as follows:
![A2DP Stack](../../assets/avdtp_stream_establishment.png)

- 3.7 Stream Start
This command is used to start media transmission of a SEID, i.e., start playing music. The program flow is as follows:
![A2DP Stack](../../assets/avdtp_stream_start.png)

- 3.8 Stream Suspend
This command is used to pause media transmission, i.e., pause music. The program flow is as follows:
![A2DP Stack](../../assets/avdtp_suspend.png)

- 3.9 Stream Release
This command is used to close media transmission. The program flow is as follows:
![A2DP Stack](../../assets/avdtp_release.png)

## A2DP Introduction
A2DP (Advanced Audio Distribution Profile) is a Bluetooth high-quality audio transmission protocol used for transmitting mono and stereo music (generally used for stereo in A2DP), with typical applications being Bluetooth headsets. A2DP does not include remote control functionality; for remote control functionality, refer to the AVRCP protocol.
1. Audio Codec
In the above process, we mentioned the need for Audio codec algorithms. In the A2DP protocol, there are the following specifications:
![A2DP Stack](../../assets/a2dp_codec.png)
First, all devices are mandatorily required to have SBC codec algorithm, which is a lossy algorithm with sound quality similar to MP3. Additionally, it supports 3 optional algorithms: MPEG-1,2 audio/MPEG-2,4 AAC, ATRAC family. Of course, there are also some custom extension codec algorithms, such as the popular APTX, LDAC, etc.
1.1 SBC Codec
SBC is a mandatory protocol specified by Bluetooth. The Codec Specific Information Elements are defined as follows:
![A2DP Stack](../../assets/a2dp_sbc_codec.png)
Sampling Frequency: This part is the sampling frequency. The Source end is mandatorily required to support either 44.1KHz or 48KHz, while Sink requires support for both 44.1KHz and 48KHz. Each value corresponds to the following:
![A2DP Stack](../../assets/a2dp_sbc_sample_frequency.png)
Channel Mode: Number of channels. Sink requires full support, while Source only mandatorily requires support for Mono, others are optional:
![A2DP Stack](../../assets/a2dp_sbc_channel_mode.png)
Block Length:
![A2DP Stack](../../assets/a2dp_sbc_block_length.png)
Subbands:
![A2DP Stack](../../assets/a2dp_sbc_subbands.png)
Allocation Method:
![A2DP Stack](../../assets/a2dp_sbc_allocation_method.png)
Minimum Bitpool Value:
Maximum Bitpool Value: In playback devices, you can set SBC encoding quality. This value is called bitpool, approximately 1 bitpool = 6-7 kbit/s. SBC is a low-complexity encoding format with slightly lower sound quality at the same bitrate. According to comparisons on websites, SBC at the highest 328kbit/s has sound quality approximately between 224 kbit/s and 256 kbit/s MP3. Additionally, improper settings, poor signal, devices not supporting high bitpool, etc., can all cause transmission bitrate to drop and reduce sound quality. The sound quality of the headset or speaker itself is also a very important factor. The following shows bitrates for different bitpools:
![A2DP Stack](../../assets/a2dp_sbc_bitpool.png)

# Sifli SDK A2DP Introduction
This document is based on the Sifli SDK and introduces how to use basic A2DP functions as an A2DP sink role.
The involved files are as follows:
- bts2_app_interface
- bts2_app_av
- bts2_app_av_snk
  
## A2DP Sink Initialization
- A2DP initialization functions: bt_av_init, bt_avsnk_init, will first initialize A2DP, then initialize A2DP sink
- A2DP sink service enable function: bt_avsnk_open, enables A2DP sink functionality

```c
//Initialize A2DP
//Users can add their own initialization process in this interface, but it's not recommended to delete related code
void bt_av_init(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst;

    inst = bcalloc(1, sizeof(bts2s_av_inst_data));
    //Manage A2DP-related variables
    global_inst = inst;
    //Initialize A2DP
    bt_av_init_data(inst, bts2_app_data);
    //Initialize A2DP sink
    bt_avsnk_init(inst, bts2_app_data);
}

//A2DP initialization
static void bt_av_init_data(bts2s_av_inst_data *inst, bts2_app_stru *bts2_app_data)
{
    U8 i = 0;
    inst->que_id = bts2_app_data->phdl;

    //This configures the A2DP stream endpoint, mainly including role configuration, type configuration, and encoding configuration
    //The number of stream endpoints can be modified through MAX_NUM_LOCAL_SNK_SEIDS
#ifdef CFG_AV_SNK
    for (; i < MAX_NUM_LOCAL_SNK_SEIDS; i++)
    {
        inst->local_seid_info[i].is_enbd = TRUE;
        inst->local_seid_info[i].local_seid.acp_seid = i + 1;
        inst->local_seid_info[i].local_seid.in_use = FALSE;
        inst->local_seid_info[i].local_seid.media_type = AV_AUDIO;
        inst->local_seid_info[i].local_seid.sep_type = AV_SNK; /*IS_SNK */
#ifdef CFG_AV_AAC
        inst->local_seid_info[i].local_seid.codec = i < (MAX_NUM_LOCAL_SNK_SEIDS - 1) ? AV_SBC : AV_MPEG24_AAC;
#else
        inst->local_seid_info[i].local_seid.codec = AV_SBC;
#endif
    }
#endif // CFG_AV_SNK

    //Initialize A2DP connection information, A2DP supports up to MAX_CONNS connections (note: the maximum number of connections is not the number of sink or source connections alone, but the total of both roles)
    inst->con_idx = 0;
    for (i = 0; i < MAX_CONNS; i++)
    {
        inst->con[i].st = avidle;
        inst->con[i].role = ACPTOR; //TODO :how to set the role and use it
        inst->con[i].local_seid_idx = 0xFF;
        inst->con[i].in_use = FALSE;
        inst->con[i].cfg = AV_AUDIO_NO_ROLE;
    }

    inst->max_frm_size = 672; 
    inst->time_stamp = 0;
    inst->close_pending = FALSE;
    inst->suspend_pending = FALSE;
}

//A2DP sink initialization
void bt_avsnk_init(bts2s_av_inst_data *inst, bts2_app_stru *bts2_app_data)
{
    bt_avsnk_init_data(&inst->snk_data, bts2_app_data);

    inst->snk_data.buf_sem = rt_sem_create("bt_av_sink", 1, RT_IPC_FLAG_FIFO);

    //You can use the CFG_OPEN_AVSNK macro to control whether to enable A2DP sink during initialization
#ifdef CFG_OPEN_AVSNK
    bts2s_avsnk_openFlag = 1;
#else
    bts2s_avsnk_openFlag = 0;
#endif

    if (1 == bts2s_avsnk_openFlag)
    {
        INFO_TRACE(">> AV sink enabled\n");
        //Enable A2DP interface
        av_enb_req(inst->que_id, AV_AUDIO_SNK); //act the svc
    }
}

//Mainly initialize some variables related to playing music
static void bt_avsnk_init_data(bts2s_avsnk_inst_data *inst, bts2_app_stru *bts2_app_data)
{
    inst->playlist.cnt = 0;
    inst->playlist.cnt_th = SINK_DATA_LIST_START_THRESHOLD;
    inst->playlist.first = NULL;
    inst->playlist.last = NULL;
    inst->play_state = FALSE;
    inst->can_play = 1;
    inst->filter_prompt_enable = 1;
    inst->reveive_start = 0;
#ifndef RT_USING_UTEST
    inst->slience_filter_enable = 1;
    inst->slience_count = 0;
#endif
#if defined(AUDIO_USING_MANAGER) && defined(AUDIO_BT_AUDIO)
    audio_client_t audio_client;
#endif
    inst->decode_buf = NULL;
}
```

## A2DP Sink Device Connection
The following process describes how A2DP sink discovers, connects, and plays music from remote devices.
1. The first step is to discover available A2DP source devices (e.g., phones, computers) nearby. To do this, A2DP sink can perform a search for nearby devices, then use SDP to retrieve A2DP source services from those devices that support the A2DP source role.
2. Select an A2DP source device to connect to. Initiate an ACL connection to the selected device.
3. A2DP connection. Once the ACL connection is created, A2DP sink can initiate an L2CAP connection for the A2DP signal channel. After connecting to the A2DP signal channel, a series of A2DP commands need to be initiated, then A2DP sink can initiate an L2CAP connection for the A2DP media channel.
4. Play audio sent by the A2DP source.
5. Either A2DP sink or A2DP source can terminate the connection at any time.
This section won't reorganize the connection process from HCI to L2CAP to AVDTP, but will directly show the AVDTP and A2DP interaction:
![A2DP Stack](../../assets/a2dp_connect.png)
Step 1) Development board initiates AVDTP discover command to query SEIDs supported by the phone, phone responds
Step 2) Development board gets SEP configurable information based on SEID, phone responds
Step 3) Development board sets playback parameters (sampling rate, number of channels, bit width, etc.), phone responds
Step 4) Development board opens SEP, phone responds
Step 5) Phone sends Start instruction to prepare for music playback, development board responds
Step 6) Phone sends music
Step 7) Phone pauses playback, development board responds
Step 8) Phone closes SEP, development board responds

- A2DP sink device connection interface:
    //Users can call the following interface to connect A2DP
    - bts2_app_interface connection interface: bt_interface_conn_ext
    - bts2_app_av_snk connection interface: bt_avsnk_conn_2_src
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

//Before calling this interface, mac will be converted to BTS2S_BD_ADDR Bluetooth address format
//Users only need to call bt_interface_conn_ext, no need to modify this interface
bt_err_t bt_avsnk_conn_2_src(BTS2S_BD_ADDR *bd)
{
    BTS2S_BD_ADDR temp = {0xffffff, 0xff, 0xffff};
    bt_err_t res = BT_EOK;

    //Check if address is valid
    if (!bd_eq(bd, &temp))
    {
        //Call connection interface
        bt_av_conn(bd, AV_SNK);
        USER_TRACE(">> av snk connect\n");
    }
    else
    {
        res = BT_ERROR_INPARAM;
        USER_TRACE(">> pls input remote device address\n");
    }
}

//A2DP connection interface, both A2DP sink and A2DP source use the same interface for connection
//Users only need to call bt_interface_conn_ext, no need to modify this interface
void bt_av_conn(BTS2S_BD_ADDR *bd_addr, uint8_t peer_role)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    uint16_t local_role = AV_AUDIO_NO_ROLE;
    uint16_t peer_role_1 = AV_AUDIO_NO_ROLE;

#ifdef CFG_AV_SNK
    if (peer_role == AV_SRC)
    {
        local_role = AV_AUDIO_SNK;
        peer_role_1 = AV_AUDIO_SRC;
    }
#endif //CFG_AV_SNK

    USER_TRACE(" -- av conn rmt device... peer_role:%d local_role:%d\n", peer_role_1, local_role);
    USER_TRACE(" -- address: %04X:%02X:%06lX\n",
            bd_addr->nap,
            bd_addr->uap,
            bd_addr->lap);

    if (local_role == AV_AUDIO_NO_ROLE
            || peer_role_1 == AV_AUDIO_NO_ROLE)
        LOG_E("Wrongly role!");
    else
        av_conn_req(bts2_task_get_app_task_id(), *bd_addr, peer_role_1, local_role);
}
```
       
- A2DP sink device disconnection interface:
    //Users can call the following interface to disconnect A2DP
    - bts2_app_interface disconnection interface: bt_interface_disc_ext
    - bts2_app_av_snk disconnection interface: bt_avsnk_disc_by_addr
```c
/**
* @brief            Disconnect with the specified profile
* @param[in] mac    Remote device address
* @param[in] ext_profile : Profile value
*
* @return           bt_err_t
**/
bt_err_t bt_interface_disc_ext(unsigned char *mac, bt_profile_t ext_profile);

//Before calling this interface, mac will be converted to BTS2S_BD_ADDR Bluetooth address format
//Users only need to call bt_interface_disc_ext, no need to modify this interface
void bt_avsnk_disc_by_addr(BTS2S_BD_ADDR *bd_addr, BOOL is_close)
{
    bts2s_av_inst_data *inst_data;
    inst_data = bt_av_get_inst_data();
    //This flag indicates whether to send A2DP close command before disconnecting A2DP, generally false is fine
    if (is_close)
    {
        inst_data->close_pending = TRUE;
    }

    //Since A2DP supports multiple connections, it will only disconnect the A2DP with the specified address
    for (uint32_t i = 0; i < MAX_CONNS; i++)
    {
        if (bd_eq(bd_addr, &inst_data->con[i].av_rmt_addr))
        {
            bt_av_disconnect(i);
            break;
        }
    }
}
```
        
- A2DP sink event handling:
    - A2DP sink connection status callback event
        - A2DP sink connection success: BT_NOTIFY_A2DP_PROFILE_CONNECTED
        - A2DP sink connection disconnected: BT_NOTIFY_A2DP_PROFILE_DISCONNECTED
    - A2DP sink playback status callback event
        - A2DP sink receives play command from remote: BT_NOTIFY_A2DP_START_IND
        - A2DP sink receives pause command from remote: BT_NOTIFY_A2DP_SUSPEND_IND
        - A2DP sink receives audio data from remote: BT_NOTIFY_A2DP_MEDIA_DATA_IND

:::{note}
Note: The address parameters passed by the two interfaces need to be converted accordingly.
:::
```c
// After calling the A2DP connection API, the A2DP connection success message is notified to the user
// Users need to implement the hdl function to receive notify events, e.g.: bt_notify_handle
// BT_NOTIFY_A2DP_PROFILE_CONNECTED event includes: address information, profile_type, res: 0 (success)
// After disconnecting A2DP, corresponding events will also be received
// BT_NOTIFY_A2DP_PROFILE_DISCONNECTED event includes: address information, profile_type, reason
// For specific structure information, refer to API comments
static int bt_notify_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    int ret = -1;

    switch (type)
    {
        //Register notify function to handle A2DP-related events
        case BT_NOTIFY_A2DP:
        {
            bt_sifli_notify_a2dp_event_hdl(event_id, data, data_len);
        }
        break;
    }
    return 0;
}

//Users need to register A2DP notify function to handle various events
static int bt_sifli_notify_a2dp_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_A2DP_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //Users implement corresponding handling functions
        break;
    }
    case BT_NOTIFY_A2DP_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //Users implement corresponding handling functions
        break;
    }
    case BT_NOTIFY_A2DP_START_IND:
    {
        //Received play command from remote device, users implement corresponding handling functions
        break;
    }
    case BT_NOTIFY_A2DP_SUSPEND_IND:
    {
        //Received pause command from remote device, users implement corresponding handling functions
        break;
    }
    case BT_NOTIFY_A2DP_MEDIA_DATA_IND:
    {
        //Received audio data from remote device, users implement corresponding handling functions
        break;
    }
    default:
        return -1;
    }
    return 0;
}
```

## Basic A2DP Sink Function Usage
### 1. Modify sampling rate and other information
```c
//When remote device connects to A2DP, you can define A2DP sampling rate and other information by modifying the configuration below
static const U8 av_cap_rsp[2][AAC_MEDIA_CODEC_SC_SIZE] =
{
    {AV_SC_MEDIA_CODEC, SBC_MEDIA_CODEC_SC_SIZE - 2, AV_AUDIO << 4, AV_SBC, 0x3F, 0xFF, 0x02, 0x35, 0x0, 0x0},
    {AV_SC_MEDIA_CODEC, AAC_MEDIA_CODEC_SC_SIZE - 2, AV_AUDIO << 4, AV_MPEG24_AAC, 0x80, 0x01, 0x8C, 0x84, 0xE2, 0x0}
};

//When local device connects to remote, you can select A2DP sampling rate and other information through the interface below
//Users can modify some configurations as needed in this interface, but it's not recommended to delete code
static void bt_av_sbc_cfg_para_select(bts2s_av_inst_data *inst, uint16_t con_idx, uint8_t *app_serv_cap, uint8_t *serv_cap)
{
    uint32_t i;
    inst->con[con_idx].act_cfg.min_bitpool = *(serv_cap + 6);
    inst->con[con_idx].act_cfg.max_bitpool = *(serv_cap + 7);

    /*build app. svc capabilities (media and if supp cont protection) */
    app_serv_cap[0] = AV_SC_MEDIA_CODEC;
    app_serv_cap[1] = SBC_MEDIA_CODEC_SC_SIZE - 2;
    app_serv_cap[2] = AV_AUDIO << 4;
    app_serv_cap[3] = AV_SBC;

    for (i = 0; i < (SBC_MEDIA_CODEC_SC_SIZE - 6); i++)
    {
        app_serv_cap[4 + i] = av_sbc_cfg[0][i]; /*first cfg setting is our default */
    }


    /*select a configuration from the capabilities (ours and the peers) */
    //Configure sampling rate
    do
    {
        if ((*(serv_cap + 4) == 0x6a) && (*(serv_cap + 7) == 0x2c))
        {
            /*... a workaround for sonorix (has very bad sound unless using 32k_hz) */
            app_serv_cap[4] = 0x40;
            break;
        }

        if (*(serv_cap + 4) & 0x20 & av_sbc_capabilities[0])
        {
            app_serv_cap[4] = (0x10 << 1);
            break;
        }

        if (*(serv_cap + 4) & 0x10 & av_sbc_capabilities[0])
        {
            app_serv_cap[4] = 0x10;
            break;
        }
    }
    while (0);

    //Configure channel mode
    for (i = 0; i < 4; i++)
    {
        if (*(serv_cap + 4) & (0x01 << i) & av_sbc_capabilities[0])
        {
            app_serv_cap[4] |= (0x01 << i);
            break;
        }
    }
    // app_serv_cap[4] |= 0x02;
    //Configure block length
    for (i = 0; i < 4; i++)
    {
        if (*(serv_cap + 5) & (0x10 << i) & av_sbc_capabilities[1])
        {
            app_serv_cap[5] = (0x10 << i);
            break;
        }
    }
    //Configure subbands
    for (i = 0; i < 2; i++)
    {
        if (*(serv_cap + 5) & (0x04 << i) & av_sbc_capabilities[1])
        {
            app_serv_cap[5] |= (0x04 << i);
            break;
        }
    }
    //Configure method
    for (i = 0; i < 2; i++)
    {
        if (*(serv_cap + 5) & (0x01 << i) & av_sbc_capabilities[1])
        {
            app_serv_cap[5] |= (0x01 << i);
            break;
        }
    }

    /*take the min/max bitpool from peer rsp */
    app_serv_cap[6] = inst->con[con_idx].act_cfg.min_bitpool;
    app_serv_cap[7] = 0x30;

    bt_av_store_sbc_cfg(&inst->con[con_idx].act_cfg, app_serv_cap + 4);

}
```
## A2DP Function Usage Demo
- First, register the handler function to receive notify events during project initialization
- Input the MAC address of the phone to connect, wait for connection success message
- Play music on phone
```c
int bt_sifli_notify_a2dp_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    // a2dp CONNECTED
    case BT_NOTIFY_A2DP_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //Users implement corresponding handling functions
        break;
    }
    //a2dp DISCONNECTED
    case BT_NOTIFY_A2DP_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //Users implement corresponding handling functions
        break;
    }
    case BT_NOTIFY_A2DP_START_IND:
    {
        //Received play command from remote device, users implement corresponding handling functions, need to prepare audio decode module
        break;
    }
    case BT_NOTIFY_A2DP_SUSPEND_IND:
    {
        //Received pause command from remote device, users implement corresponding handling functions
        break;
    }
    case BT_NOTIFY_A2DP_MEDIA_DATA_IND:
    {
        //Received audio data from remote device, users implement corresponding handling functions, pass data to audio decode module for decoding then output to hardware for playback
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
    //BT function enabled successfully, can be used normally
    case BT_NOTIFY_COMMON_BT_STACK_READY:
    {
        break;
    }
    //BT closed successfully
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

    case BT_NOTIFY_A2DP:
    {
        bt_sifli_notify_a2dp_event_hdl(event_id, data, data_len);
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

// Connect phone:001bdcf4b6bd
unsigned char mac[6] = {0x00,0x1b,0xdc,0xf4,0xb6,0xbd}
bt_interface_conn_ext((unsigned char *)(mac), BT_PROFILE_A2DP);
//
@endcode
*/
```