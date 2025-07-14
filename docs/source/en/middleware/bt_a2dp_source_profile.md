# BT_A2DP_SOURCE
A2DP (Advanced Audio Distribution Profile) is a Bluetooth audio transmission protocol, typically used for Bluetooth headphones. A2DP protocol's audio data is transmitted over ACL Link, which is distinguished from voice data transmitted over SCO. A2DP does not include remote control functionality; remote control functionality refers to the AVRCP protocol. AVDTP defines the parameter negotiation, establishment, and transmission process of data stream handles between Bluetooth devices, as well as the signaling entity formats that are mutually exchanged. This protocol is the foundation protocol of the A2DP framework.

A2DP is designed to transmit high-quality audio streams over Bluetooth connections. The basic compression algorithm it uses is SBC, which is used to reduce the size of audio data while maintaining high sound quality. SBC is a mandatory compression algorithm that must be supported. In addition to SBC, A2DP can also support other advanced codecs such as AAC, LHDC, etc., but support for these codecs depends on the device's own support capabilities.

A2DP implementation relies on GAVDP and GAP, where GAVDP defines the stream connection establishment process. The protocol dependency relationship of A2DP is shown in the following diagram:
![A2DP Stack](../../assets/a2dp_arch.png)

In A2DP, two different roles are defined: SRC (source) and SNK (sink). SRC sends audio data, SNK receives audio data. The corresponding protocol model is as follows:
![A2DP Stack](../../assets/a2dp_stack.png)

The main roles included in A2DP Profile are:
- source (SRC): device that sends audio data
- sink (SNK): device that receives audio data

A2DP is built on the foundation of the AVDTP transport protocol. AVDTP specifies how connections are established. After the connection is established, audio data can be sent and received after compression. Audio data is bidirectional. The A2DP data transmission and reception process based on AVDTP is shown in the following diagram:
![A2DP Stack](../../assets/a2dp_transport_data.png)

Overall summary of the entire process:
1. UL collects PCM data and sends it to A2DP. A2DP compresses it into specific audio formats through codec PCM lib (SBC, MPEG-1,2 Audio MPEG-2,4 AAC ATRAC family, or custom encoder pcm lib), then hands it to AVDTP. AVDTP transfers it to L2CAP, L2CAP transfers it to HCI through ACL format, then reaches the BT chip and transmits via RF.
2. BT chip receives incoming data via RF, then transfers it to HCI through ACL, then to L2CAP, L2CAP to AVDTP, AVDTP to A2DP. A2DP receives compressed data from the remote device, which is then decompressed into PCM data through codec pcm lib (SBC, MPEG-1,2 Audio MPEG-2,4 AAC ATRAC family, or custom encoder pcm lib), and then handed to the sound card for playback.

## AVDTP Introduction
AVDTP (A/V Distribution Transport Protocol) is an audio/video distribution transport protocol, mainly used for transmitting audio/video data. The architecture diagram of the entire protocol stack is as follows:
![A2DP Stack](../../assets/avdtp_arch.png)

1. AVDTP Terminology Introduction
   - Stream: Streaming data between two point-to-point devices.
   - Source (SRC) and Sink (SNK): SRC is the sender of audio/video, SNK is the receiver of audio/video.
   - Initiator (INT) and Acceptor (ACP): The device that initiates the process acts as the initiator, and the device that accepts the initiation is the acceptor. Note that INT and ACP are independent of the SRC and SNK defined by the upper application. In a CMD and RESPONSE, the device sending the CMD is the INT role, and the device responding with RESPONSE is the ACP role, so their roles will be dynamically switching.
   - Application and Transport Service Capabilities: Application service and transport service capabilities. Application service capabilities include negotiating and configuring codec of source devices, content protection systems, etc.; transport service capabilities include data packet segmentation and reassembly, packet loss detection, etc.
   - Services, Service Categories, and Service Parameters: Services, service categories, service parameters.
   - Media Packets, Recovery Packets, and Reporting Packets: Media packets, data recovery packets, report packets.
   - Stream End Point (SEP): Stream endpoint, which is an application that exposes available transport services and A/V capabilities for negotiating a stream.
   - Stream Context (SC): Stream context. Refers to the common understanding of stream configuration reached by two peer devices during the stream setup process, including selected services, parameters, and transport channel allocation.
   - Stream Handle (SH): Stream handle. An independent identifier allocated after SRC and SNK establish a connection, representing the upper layer's reference to the stream.
   - Stream End Point Identifier (SEID): Stream endpoint identifier, cross-device reference for specific devices.
   - Stream End Point State: Stream endpoint state.
   - Transport Session: Transport session. Within the A/V transport layer, streams can be decomposed into one, two, or multiple three transport sessions between paired AVDTP entities.
   - Transport Session Identifier (TSID): Transport session identifier. Represents a reference to a transport session.
   - Transport Channel: Transport channel. Transport channel refers to the abstraction of the underlying bearer program of the A/V transport layer, always corresponding to L2CAP channels.
   - Transport Channel Identifier (TCID): Transport channel identifier. Represents a reference to a transport channel.
   - Reserved for Future Additions (RFA): Reserved for future additions.
   - Reserved for Future Definitions (RFD): Reserved for future definitions.
  
2. AVDTP Packet Format
AVDTP mainly has two types of packets: signal packets on the signal channel and media packets on the media channel.
![A2DP Stack](../../assets/avdtp_signal_packet.png)
The above is the Signal header format, which can be seen to have 3 packet formats:
&emsp;&emsp;1) Single packet
&emsp;&emsp;2) Start packet, generally used for the first packet when packet size > MTU fragmentation
&emsp;&emsp;3) Continue packet and end packet, generally used for continue packets and end packets when packet size > MTU
The following explains the parameters:
   - Transaction Label: Transport identifier, 4 bits, filled by the INT role, ACP must return the same value
   - Packet Type: Packet type, with the following types:
![A2DP Stack](../../assets/avdtp_transaction_label.png)
Message Type: Message type, with the following types:
![A2DP Stack](../../assets/avdtp_message_type.png)
Signal Identifier: Signaling identifier, with the following values:
![A2DP Stack](../../assets/avdtp_signal_identifer.png)
NOSP = Number Of Signal Packets: Start packet will inform how many subsequent packets need to be transmitted

3. AVDTP Signal Commands
There are the following signaling commands in total:
![A2DP Stack](../../assets/avdtp_signal_identifer.png)

Before explaining specific commands, let's first lay some groundwork by explaining Service Capabilities, which will all be used, so we'll explain them in advance.
The Service Capabilities format is as follows:
![A2DP Stack](../../assets/avdtp_service_capability.png)
This part is also similar to the TLV (Type Length Value) type, where Service Category is TYPE, with values as follows:
![A2DP Stack](../../assets/avdtp_service_capability_value.png)
Where Length Of Service Capabilities (LOSC) is similar to length, i.e., the length of the subsequent Service Capabilities Information Elements.
Service Capabilities Information Elements are specific values, which require a large section to explain. Here we mainly explain Media Codec Capabilities.
![A2DP Stack](../../assets/avdtp_media_codec_capability.png)
Where Media Type has the following values:
![A2DP Stack](../../assets/avdtp_media_type.png)
Where Media Codec Type has the following values:
![A2DP Stack](../../assets/avdtp_media_codec.png)
The Media Codec Specific Information Elements in this part are some upper-layer codec information, such as the following figure for SBC:
![A2DP Stack](../../assets/avdtp_sbc_media_codec.png)

- 3.1 AVDTP Signal Commands
Each AVDTP endpoint will register one or more SEPs, identified by SEID. This command is to obtain the SEP information of the peer, including SEID (SEP ID), In Use (whether it is being used), Media Type (Audio, Media, MultiMedia), TSEP (whether the role is Sink or Source)
The process is as follows:
![A2DP Stack](../../assets/avdtp_discover.png)

- 3.2 Get Capabilities
This command uses SEID to obtain the peer's Capabilities. The Capabilities have been introduced before. Let's look at the program flow:
![A2DP Stack](../../assets/avdtp_get_capability.png)

- 3.3 Get All Capabilities
This command is used to replace Get Capabilities, also using SEID to obtain the peer's Capabilities. The Capabilities have been introduced before. Let's look at the program flow:
![A2DP Stack](../../assets/avdtp_get_all_capability.png)

- 3.4 Set Configuration Command
After obtaining Capabilities, this part selects specific functional parameters. The program flow is as follows:
![A2DP Stack](../../assets/avdtp_set_configure.png)

- 3.5 Get Stream Configuration
This command is used to obtain configuration based on SEID. The program flow is as follows:
![A2DP Stack](../../assets/avdtp_get_configure.png)

- 3.6 Stream Establishment
This command is used to open a certain SEID, i.e., establish a media channel. The program flow is as follows:
![A2DP Stack](../../assets/avdtp_stream_establishment.png)

- 3.7 Stream Start
This command is used to start media transmission for a SEID, i.e., start playing music. The program flow is as follows:
![A2DP Stack](../../assets/avdtp_stream_start.png)

- 3.8 Stream Suspend
This command is used to pause media transmission, i.e., pause music. The program flow is as follows:
![A2DP Stack](../../assets/avdtp_suspend.png)

- 3.9 Stream Release
This command is used to close media transmission. The program flow is as follows:
![A2DP Stack](../../assets/avdtp_release.png)

## A2DP Introduction
A2DP (Advanced Audio Distribution Profile) is a Bluetooth high-quality audio transmission protocol used for transmitting mono and stereo music (generally used for stereo dual-channel in A2DP), typically applied to Bluetooth headphones. A2DP does not include remote control functionality; remote control functionality refers to the AVRCP protocol.

1. Audio Codec
In the above process, we also mentioned the need for Audio codec algorithms. In the A2DP protocol, there are the following regulations:
![A2DP Stack](../../assets/a2dp_codec.png)
First, all devices are mandatorily required to have SBC codec algorithm, which is a lossy algorithm with sound quality comparable to MP3. Additionally, it supports 3 optional algorithms: MPEG-1,2 audio/MPEG-2,4 AAC, ATRAC family. Of course, there are also some custom extended codec algorithms, such as the popular APTX, LDAC, etc.

1.1 SBC Codec
SBC is a protocol mandatorily required to be supported by Bluetooth. The Codec Specific Information Elements are defined as follows:
![A2DP Stack](../../assets/a2dp_sbc_codec.png)
Sampling Frequency: This part is the sampling frequency. The Source end mandatorily requires support for either 44.1KHz or 48KHz, while Sink requires support for both 44.1KHz and 48KHz. The corresponding values are as follows:
![A2DP Stack](../../assets/a2dp_sbc_sample_frequency.png)
Channel Mode: Number of channels. Sink requires full support, while Source only mandatorily requires support for Mono, others are optional:
![A2DP Stack](../../assets/a2dp_sbc_channel_mode.png)
Block Length: Block length
![A2DP Stack](../../assets/a2dp_sbc_block_length.png)
Subbands: Subbands
![A2DP Stack](../../assets/a2dp_sbc_subbands.png)
Allocation Method: Allocation method
![A2DP Stack](../../assets/a2dp_sbc_allocation_method.png)
Minimum Bitpool Value:
Maximum Bitpool Value: In playback devices, you can set SBC encoding quality, this value is called bitpool, approximately 1 bitpool = 6~7 kbit/s. SBC is an encoding format with relatively low complexity and slightly inferior sound quality at the same bit rate. According to comparisons on this website, the highest 328kbit/s SBC sound quality is approximately between 224 kbit/s to 256 kbit/s MP3. Additionally, improper settings, poor signal, device not supporting high bitpool, etc., will cause transmission bit rate to decrease and sound quality to deteriorate. The sound quality of the headphones or speakers themselves is also a very important factor. The following are bit rates for different bitpools:
![A2DP Stack](../../assets/a2dp_sbc_bitpool.png)

# Sifli SDK A2DP Introduction
This document is mainly based on Sifli SDK, introducing how to use basic A2DP functions as an A2DP source role.
Related files include:
- bts2_app_interface
- bts2_app_av
- bts2_app_av_src
  
## A2DP Source Initialization
- A2DP initialization functions: bt_av_init, bt_avsrc_init, will first initialize A2DP, then initialize A2DP source

```c
//Initialize A2DP
//Users can add their own initialization process in this interface, but it is not recommended to delete related code
void bt_av_init(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst;

    inst = bcalloc(1, sizeof(bts2s_av_inst_data));
    //Manage A2DP related variables
    global_inst = inst;
    //Initialize A2DP
    bt_av_init_data(inst, bts2_app_data);
    //Initialize A2DP source
    bt_avsrc_init(inst, bts2_app_data);
}

//A2DP initialization
static void bt_av_init_data(bts2s_av_inst_data *inst, bts2_app_stru *bts2_app_data)
{
    U8 i = 0;
    inst->que_id = bts2_app_data->phdl;

    //This configures A2DP stream endpoints, mainly including role configuration, type configuration, encoding configuration
    //The number of stream endpoints can be modified through MAX_NUM_LOCAL_SRC_SEIDS
#ifdef CFG_AV_SRC
    for (; i < MAX_NUM_LOCAL_SRC_SEIDS + MAX_NUM_LOCAL_SNK_SEIDS; i++)
    {
        inst->local_seid_info[i].is_enbd = TRUE; 
        inst->local_seid_info[i].local_seid.acp_seid = i + 1;
        inst->local_seid_info[i].local_seid.in_use = FALSE;
        inst->local_seid_info[i].local_seid.media_type = AV_AUDIO;
        inst->local_seid_info[i].local_seid.sep_type = AV_SRC;//AV_SRC; /* IS_SRC */
        inst->local_seid_info[i].local_seid.codec = AV_SBC;
    }
#endif //CFG_AV_SRC

    //Initialize A2DP connection information, A2DP supports up to MAX_CONNS connections (Note: the maximum number of connections is not the number of connections for sink or source separately, but the total for both roles)
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

//A2DP source initialization
void bt_avsrc_init(bts2s_av_inst_data *inst, bts2_app_stru *bts2_app_data)
{
    bt_avsrc_init_data(inst, bts2_app_data);
#ifdef AUDIO_USING_MANAGER
    bt_avsrc_register_audio_service(bts2_app_data);
#endif // AUDIO_USING_MANAGER

#if AUDIO_DATA_FROM_PSRAM
    memcpy((void *)INPUT_ADDRESS, (void *)0x14120000, 0x1AEAA0);
    dataend = (uint8_t *)INPUT_ADDRESS + 0x1AEAA0;
#endif // AUDIO_DATA_FROM_PSRAM
    INFO_TRACE(">> AV source enabled \n");
    //Enable A2DP source functionality
    av_enb_req(inst->que_id, AV_AUDIO_SRC); //act the svc
}

//Mainly initialize some variables related to music playback
static void bt_avsrc_init_data(bts2s_av_inst_data *inst, bts2_app_stru *bts2_app_data)
{
#ifdef AUDIO_USING_MANAGER
    inst->src_data.audio_state = AVSRC_AUDIO_SER_IDLE;
    inst->src_data.tid = 0;
    inst->src_data.stream_frm_time_begin = 0;
#endif // AUDIO_USING_MANAGER
}
```

## A2DP Source Device Connection
The following process describes how A2DP source discovers, connects to, and plays music.
1. The first step is to discover available A2DP sink devices (e.g., headphones, speakers) in the vicinity. For this, the A2DP source can perform a search on nearby devices, then use SDP to retrieve A2DP sink services from those devices that support the A2DP sink role.
2. Select an A2DP sink device to connect to. Choose the device to connect to and initiate an ACL connection.
3. A2DP connection. Once the ACL connection is established, the A2DP source can initiate an L2CAP connection for the A2DP signal channel. After connecting to the A2DP signal channel, a series of A2DP commands need to be initiated, then the A2DP source can initiate an L2CAP connection for the A2DP media channel.
4. A2DP source starts playing music.
5. A2DP sink or A2DP source can terminate the connection at any time.

- A2DP source device connection interface:
    //Users can call the following interface to connect A2DP
    - bts2_app_interface connection interface: bt_interface_conn_to_source_ext
    - bts2_app_av_src connection interface: bt_avsrc_conn_2_snk
```c
/**
* @brief            Initiate connect with the specified device and profile(ag source)
* @param[in] mac    Remote device address
* @param[in] ext_profile   Profile value
*
* @return           bt_err_t
**/
bt_err_t bt_interface_conn_to_source_ext(unsigned char *mac, bt_profile_t ext_profile);

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
//Users only need to call bt_interface_conn_to_source_ext, no need to modify this interface
bt_err_t bt_avsnk_conn_2_src(BTS2S_BD_ADDR *bd)
{
    BTS2S_BD_ADDR temp = {0xffffff, 0xff, 0xffff};
    bt_err_t res = BT_EOK;

    //Check if address is valid
    if (!bd_eq(bd, &temp))
    {
        bt_av_conn(bd, AV_SRC);
        USER_TRACE(">> av snk connect\n");
    }
    else
    {
        res = BT_ERROR_INPARAM;
        USER_TRACE(">> pls input remote device address\n");
    }

    return res;
}

//A2DP connection interface, both A2DP sink and A2DP source use the same interface to connect
//Users only need to call bt_interface_conn_to_source_ext, no need to modify this interface
void bt_av_conn(BTS2S_BD_ADDR *bd_addr, uint8_t peer_role)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    uint16_t local_role = AV_AUDIO_NO_ROLE;
    uint16_t peer_role_1 = AV_AUDIO_NO_ROLE;

#ifdef CFG_AV_SRC
    if (peer_role == AV_SNK)
    {
        local_role = AV_AUDIO_SRC;
        peer_role_1 = AV_AUDIO_SNK;
    }
#endif // CFG_AV_SRC

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
       
- A2DP source device disconnection interface:
    //Users can call the following interface to disconnect A2DP
    - bts2_app_interface disconnection interface: bt_interface_disc_ext
    - bts2_app_av_src disconnection interface: bt_avsnk_disc_by_addr
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
void bt_avsnk_disc_by_addr(BTS2S_BD_ADDR *bd_addr, BOOL is_close)
{
    bts2s_av_inst_data *inst_data;
    inst_data = bt_av_get_inst_data();
    //This flag indicates whether to send A2DP close command before disconnecting A2DP, generally false is sufficient
    if (is_close)
    {
        inst_data->close_pending = TRUE;
    }

    //Because A2DP supports multiple connections, only the A2DP of the specified address will be disconnected
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

- A2DP source data sending interface:
    - bts2_app_av_src data sending interface: bt_avsrc_send_data
```c
//This interface needs to pass in a shared memory used together with the audio module
//Audio needs to store decoded data in this shared memory, Bluetooth needs to retrieve data from this shared memory for encoding and transmission

//Users need to implement corresponding callback functions in the audio module
static const struct audio_device g_bt_avsrc_audio_dev =
{
    .open = bt_avsrc_register_audio_open,
    .close = bt_avsrc_register_audio_close,
    .user_data = NULL,
    .output = bt_avsrc_register_audio_output,
    .ioctl = bt_avsrc_register_audio_ioctl,
};
static uint16_t bt_avsrc_send_data(struct rt_ringbuffer *rb)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    U8 *pkt_ptr = NULL;
    U8 *payload_ptr  = NULL;
    U8 *sbc_frm_ptr = NULL;
    U8 *samples_ptr = NULL;
    U8 *tmp_ptr = NULL;
    U8 frms;
    U16 payload_size;
    size_t bytes_rd = 0;
    U32 actual_time_expired;
    U32 actual_timer_delay;
    int con_idx;
    int numrd;
    bts2_sbc_cfg *act_cfg  = NULL;
    U16 i;
    U16 payload_len;

    //Find the corresponding source connection
    con_idx = bt_avsrc_get_plyback_conn(inst);


    if (con_idx == - 1)
        return 0;

    //Need to check A2DP status
    if (inst->con[con_idx].st == avconned_streaming && !inst->suspend_pending)
    {
        do
        {
            U32 pcm_pkg_pos;
            U32 sb_pos = 0;
            U32 rd_len = 0;
            U16 pl;
            U16 buffer_count = av_get_stream_buffize();

            //Source data transmission is triggered by timer, so need to calculate the time for sending the next packet
            if (inst->src_data.stream_frm_time_begin != 0)
            {

                inst->src_data.stream_frm_time_end = BTS2_GET_TIME();

                actual_time_expired = inst->src_data.stream_frm_time_end - inst->src_data.stream_frm_time_begin;
                actual_timer_delay = actual_time_expired - inst->src_data.m_sec_time_4_next_pkt;
                inst->src_data.u_sec_per_pkt_sum += inst->src_data.u_sec_per_pkt;
                inst->src_data.m_sec_time_4_next_pkt = inst->src_data.m_sec_per_pkt - actual_timer_delay;

                if (inst->src_data.u_sec_per_pkt_sum >= (inst->src_data.m_sec_per_pkt * 1000))
                {
#if 0
                    inst->src_data.m_sec_time_4_next_pkt += inst->src_data.m_sec_per_pkt - 1;
                    inst->src_data.u_sec_per_pkt_sum = 0;
#else
                    inst->src_data.m_sec_time_4_next_pkt += inst->src_data.m_sec_per_pkt;
                    inst->src_data.u_sec_per_pkt_sum = inst->src_data.u_sec_per_pkt_sum % 1000;
#endif
                }


            }

            inst->src_data.stream_frm_time_begin = BTS2_GET_TIME();

            //If data is sent too quickly, causing many cached packets in the lower layer, the cached packets in the lower layer will be sent first
            if (buffer_count >= av_get_max_stream_buffer_cnt() / 2)
            {
                LOG_I("a2dp buffer more than half, should send buffered frames,count = %d\n", buffer_count);
                break;
            }

            //First check if the data length in the shared buffer with audio is enough to send one packet, if not enough, will notify the audio module to decode
            uint16_t len = rt_ringbuffer_data_len((struct rt_ringbuffer *)rb);

            act_cfg = &inst->con[con_idx].act_cfg;

            if (len < act_cfg->bytes_to_rd)
            {
#ifdef AUDIO_USING_MANAGER
                LOG_I("a2dp_src: buffer empty\n");

                if (inst->src_data.input_cb == NULL)
                {
                    LOG_I("input_cb = NULL!!!!!!\n");
                    return 0;
                }
                //If the current data in shared memory is not enough for Bluetooth to send one packet, will notify the audio module to decode through callback
                inst->src_data.input_cb(as_callback_cmd_cache_half_empty, NULL, 0);
                send_timer_added = 1;
                inst->src_data.tid = bts2_timer_ev_add(inst->src_data.m_sec_per_pkt, bt_avsrc_send_cb, 0, (void *)rb);
#endif
                return 0;
            }

            uint8_t *ptr = bmalloc(act_cfg->bytes_to_rd);
            if (!ptr)
                break;

            payload_size = act_cfg->frms_per_payload * act_cfg->frmsize;
            pkt_ptr = bmalloc(payload_size + AV_FIXED_MEDIA_PKT_HDR_SIZE + 1);

            if (!pkt_ptr)
            {
                bfree(ptr);
                break;
            }

            rt_ringbuffer_get((struct rt_ringbuffer *)rb, ptr, act_cfg->bytes_to_rd);

            len = rt_ringbuffer_data_len((struct rt_ringbuffer *)rb);
#ifdef AUDIO_USING_MANAGER
            //If the current data in shared memory has consumed half or is not enough for the next packet, will notify the audio module to decode through callback
            if ((len <= rt_ringbuffer_get_size((struct rt_ringbuffer *)rb) / 2)
                    || (len < act_cfg->bytes_to_rd))
            {
                if (inst->src_data.input_cb)
                {
                    inst->src_data.input_cb(as_callback_cmd_cache_empty, NULL, 0);
                }
            }
#endif

            payload_ptr = sbc_frm_ptr = pkt_ptr + AV_FIXED_MEDIA_PKT_HDR_SIZE;
            frms = 0;
            numrd = bytes_rd = act_cfg->bytes_to_rd;

            samples_ptr = ptr;
            *sbc_frm_ptr++ = 0; /*reserve space for payload header */

            pcm_pkg_pos = AV_FIXED_MEDIA_PKT_HDR_SIZE + 1;
            pl = payload_size;

            //Encode the read PCM data
            while (numrd > 0)
            {
                BTS2S_SBC_STREAM bss;

                bss.dst_len = pl;
                bss.pdst = &pkt_ptr[pcm_pkg_pos];
                bss.src_len = numrd;
                bss.psrc = &samples_ptr[sb_pos];

                //Perform SBC encoding
                bts2_sbc_encode(&bss);

                sb_pos += bss.src_len_used;
                rd_len -= bss.src_len_used;

                samples_ptr += bss.src_len_used;
                numrd -= bss.src_len_used;


                pcm_pkg_pos += bss.dst_len_used;
                pl -= bss.dst_len_used;
                frms += bss.dst_len_used / act_cfg->frmsize;
            }

            bfree(ptr);
            payload_len = (U16)(frms * act_cfg->frmsize + 1 + AV_FIXED_MEDIA_PKT_HDR_SIZE);
            if (frms > 0)
            {
                *payload_ptr = frms;
                for (i = 0; i < MAX_CONNS; i++)
                {
                    if (inst->con[i].st == avconned_streaming)
                    {
                        //Send A2DP media packet
                        av_stream_data_req(inst->con[i].stream_hdl,
                                        FALSE,
                                        FALSE,
                                        96, //any dynamic PT in the range 96 - 127
                                        inst->time_stamp,
                                        payload_len,
                                        pkt_ptr); /**/
                    }
                }
                inst->time_stamp += frms * act_cfg->blocks * act_cfg->subbands;
            }
        }
        while (0);

#ifdef AUDIO_USING_MANAGER
        // Start a timer to prepare to send the next packet
        if (inst->src_data.m_sec_time_4_next_pkt > 0)
        {
            send_timer_added = 1;
            inst->src_data.tid = bts2_timer_ev_add(inst->src_data.m_sec_time_4_next_pkt, bt_avsrc_send_cb, 0, (void *)rb);
        }
        else
        {
            send_timer_added = 1;
            inst->src_data.tid = bts2_timer_ev_add(0, bt_avsrc_send_cb, 0, (void *)rb);
        }
#endif
    }

    return bytes_rd;
}
```
        
- A2DP source event handling:
    - A2DP source connection status callback events:
        - A2DP source connection successful: BT_NOTIFY_A2DP_PROFILE_CONNECTED
        - A2DP source connection disconnected: BT_NOTIFY_A2DP_PROFILE_DISCONNECTED
    - A2DP sink playback status callback events
        - A2DP source receives response to play command sent to peer: BT_NOTIFY_A2DP_START_CFM
        - A2DP source receives response to pause command sent to peer: BT_NOTIFY_A2DP_SUSPEND_CFM

:::{note}
Note: The address parameters passed by the two interfaces need to be converted accordingly.
:::
```c
// After calling the API to connect A2DP, the A2DP connection successful message is sent to the user through notify
// Users need to implement hdl functions to receive notify events, such as: bt_notify_handle
// BT_NOTIFY_A2DP_PROFILE_CONNECTED event includes: address information, profile_type, res: 0 (success)
// After disconnecting A2DP, corresponding events will also be received
// BT_NOTIFY_A2DP_PROFILE_DISCONNECTED event includes: address information, profile_type, reason
// For specific structure information, refer to API comments
static int bt_notify_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    int ret = -1;

    //Register notify functions to handle A2DP related events
    switch (type)
    {
        case BT_NOTIFY_A2DP:
        {
            bt_sifli_notify_a2dp_event_hdl(event_id, data, data_len);
        }
        break;
    }
    return 0;
}

//Users need to register A2DP notify functions to handle various events
static int bt_sifli_notify_a2dp_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_A2DP_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //Users implement their own corresponding processing functions
        break;
    }
    case BT_NOTIFY_A2DP_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //Users implement their own corresponding processing functions
        break;
    }
    case BT_NOTIFY_A2DP_START_CFM:
    {
        //Received start response from peer device, users implement their own corresponding processing functions
        //After receiving start response from peer, need to notify audio module to decode data and send decoded data to Bluetooth module for encoding
        break;
    }
    case BT_NOTIFY_A2DP_SUSPEND_CFM:
    {
        //Received suspend response from peer device, users implement their own corresponding processing functions
        break;
    }
    default:
        return -1;
    }
    return 0;
}
```

## A2DP Source Basic Functionality Usage
### 1. Modify Sampling Rate and Other Information
```c
//When the local device connects to a peer device, you can select A2DP sampling rate and other information through the following interface
//Users can modify some configurations according to needs, but it is not recommended to delete code
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


    /*select a cfguration from the capabilities (ours and the peers) */
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
### 2. Search for Headphones and Other Peripheral Devices
The search interface is: bt_interface_start_inquiry
- A2DP source search event: BT_NOTIFY_COMMON_DISCOVER_IND

## A2DP Functionality Usage Demo
- First, during project initialization, register the processing function for receiving notify events
- Search for connectable devices
- Enter the MAC address of the phone to connect to, wait for connection success message
- Phone plays music
```c
int bt_sifli_notify_a2dp_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    // a2dp CONNECTED
    case BT_NOTIFY_A2DP_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //Users implement their own corresponding processing functions
        break;
    }
    //a2dp DISCONNECTED
    case BT_NOTIFY_A2DP_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //Users implement their own corresponding processing functions
        break;
    }
    case BT_NOTIFY_A2DP_START_CFM:
    {
        //Received start response from peer device, users implement their own corresponding processing functions
        //After receiving start response from peer, need to notify audio module to decode data and send decoded data to Bluetooth module for encoding
        break;
    }
    case BT_NOTIFY_A2DP_SUSPEND_CFM:
    {
        //Received suspend response from peer device, users implement their own corresponding processing functions
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
    //Search event handling
    case BT_NOTIFY_COMMON_DISCOVER_IND:
    {
        bt_notify_remote_device_info_t *remote_info = (bt_notify_remote_device_info_t *)data;
        //Users implement their own corresponding processing functions, will report searched addresses, names and other information
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

//Start search
bt_interface_start_inquiry();

// Connect to headphones: 001bdcf4b6bd
unsigned char mac[6] = {0x00,0x1b,0xdc,0xf4,0xb6,0xbd}
bt_interface_conn_to_source_ext((unsigned char *)(mac), BT_PROFILE_A2DP);
```