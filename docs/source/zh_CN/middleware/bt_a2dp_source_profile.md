# BT_A2DP_SOURCE
A2DP(Advanced Audio Distribution Profile)是蓝牙的音频传输协议，典型应用为蓝牙耳机。A2DP协议的音频数据在ACL Link上传输，这与SCO上传输的语音数据要区别。A2DP不包括远程控制的功能，远程控制的功能参考协议AVRCP。AVDTP则定义了蓝牙设备之间数据流句柄的参数协商，建立和传输过程以及相互交换的信令实体形式，该协议是A2DP框架的基础协议。
A2DP旨在通过蓝牙连接传输高质量的audio stream。它使用的基本压缩算法是SBC。其用来减小音频数据的大小，同时保持高音质。SBC是必须支持的压缩算法。除了SBC，A2DP还可以支持AAC、LHDC等其他高级编解码器，但这些编解码器的支持取决于设备本身的支持情况。
A2DP的实现依赖于GAVDP和GAP，在GAVDP中定义了流连接的建立过程。A2DP的协议依赖关系，参见下图：
![A2DP Stack](../../assets/a2dp_arch.png)
A2DP中，定义了两种不同的角色，分别为：SRC（source）和SNK（sink）。SRC发送音频数据，SNK接收音频数据。其对应的协议模型如下。
![A2DP Stack](../../assets/a2dp_stack.png)
A2DP Profile中主要包含的角色是:
- source(SRC):发送音频数据的设备
- sink(SNK):接收音频数据的设备

A2DP建立在AVDTP传输协议的基础之上，AVDTP规定了连接是如何建立的，连接建立好之后，音频数据经过压缩之后，就可以收发了。音频数据是双向的。A2DP基于AVDTP的数据收发过程，参见下图：
![A2DP Stack](../../assets/a2dp_transport_data.png)

大体总结下整个流程：
1. UL收集到PCM 数据，然后发送到A2DP，A2DP经过 codec PCM lib(SBC,MPEG-1,2 AudioMPEG-2,4 AACATRAC family 或者自定义 encoder pcm lib)压缩成特定的音频格式，然后交给 AVDTP，AVDTP 转交给 L2CAP,L2CAP 通过 ACL 格式转交给 HCI，然后到达 BT chip，通过 RF 射频出去。
2. BT chip 通过 RF 接收进来数据，然后通过 ACL 交给 HCI，然后交给L2CAP,L2CAP 交给 AVDTP，AVDTP 交给 A2DP， A2DP 收到的是 remote 经过 压缩 的数 据，此 时通 过 codec pcm lib(SBC,MPEG-1,2 AudioMPEG-2, 4 AACATRAC family或者自定义encoder pcm lib)解压成PCM数据，然后交于声卡播放。

## avdtp简介
AVDTP（ A/V Distribution Transport Protocol）就是音视频分布传输协议，主要用于传输音频/视频数据。在整个协议栈的架构图如下：
![A2DP Stack](../../assets/avdtp_arch.png)
1. AVDTP术语介绍
   - Stream：两个点对点设备之间的流媒体数据。
   - Source (SRC) and Sink (SNK) ：SRC 是音视频的发送方，SNK 是音视频的接收方。
   - Initiator (INT) and Acceptor (ACP)：启动过程的设备作为启动者、接受启动的设备为接收者。要注意的是 INT 和 ACP 独立于上层应用定义的 SRC 和 SNK，也就是在一个 CMD 跟 RESPONSE中，发送 CMD 的是 INT 角色，回送 RESPONSE 的就是 ACP 角色，所以他的角色会一直在动态切换中。
   - Application and Transport Service Capabilities：应用服务和传输服务的功能。应用服务功能比如协商、配置音源设备的 codec，内容保护系统等；传输服务能力比如数据报文的分割和重组，数据包的防丢检测等等。
   - Services, Service Categories, and Service Parameters：服务、服务类别、服务参数。
   - Media Packets, Recovery Packets, and Reporting Packets：流媒体包，数据恢复包，报告报文。
   - Stream End Point (SEP) ：流端点，流端点是为了协商一个流而公开可用传输服务和 A/V 功能的应用程序。
   - Stream Context (SC)：流上下文。指在流设置过程中，两个对等设备达到一个公共的了解流的配置，包括选择的服务，参数，以及传输通道分配。
   - Stream Handle (SH) ：流句柄。在 SRC 和 SNK 建立了连接之后分配的一个独立的标识符，代表了上层对流的引用。
   - Stream End Point Identifier (SEID)：流端点标识，对特定设备的跨设备引用。
   - Stream End Point State：流端点状态。
   - Transport Session ：传输会话。在 A/V 传输层的内部，在配对的 AVDTP 实体之间，流可以分解为一个、两个或多个三个传输会话。
   - Transport Session Identifier (TSID)：传输会话标识。代表对一个传输会话的引用。
   - Transport Channel ：传输通道。传输通道指的是对 A/V 传输层下层承载程序的抽象，始终对应 L2CAP 的通道。
   - Transport Channel Identifier (TCID)：传输通道标识。代表对一个传输通道的引用。
   - Reserved for Future Additions （RFA ）：保留给将来添加。
   - Reserved for Future Definitions (RFD)：保留给将来定义。
  
2. AVDTP 封包格式
AVDTP主要有两种packet，一种为signal channel上的signal packet，另一种为media channel上的media packet。
![A2DP Stack](../../assets/avdtp_signal_packet.png)
以上就是 Signal 的 header format,可以看到分 3 种封包格式：
&emsp;&emsp;1）单一封包
&emsp;&emsp;2）开始封包，一般用于封包大小>MTU 的拆包的第一个封包
&emsp;&emsp;3）继续封包和结束封包，一般用于封包大小>MTU 的继续封包和结束封包
下面讲下参数：
   - Transaction Label ：传输标示，4bit，INT 角色来填写一个值，ACP 必须回送一样的值
   - Packet Type：封包类型，有以下几种:
![A2DP Stack](../../assets/avdtp_transaction_label.png)
Message Type：消息类型，有以下几种:
![A2DP Stack](../../assets/avdtp_message_type.png)
Signal Identifier：信令标识符，有以下几种值:
![A2DP Stack](../../assets/avdtp_signal_identifer.png)
NOSP = Number Of Signal Packets：Start 封包会告知后续有多少个封包要传输

3. AVDTP signal命令
一共有以下几种 signaling command：
![A2DP Stack](../../assets/avdtp_signal_identifer.png)

讲解具体的 command 之前，我们先来做一个铺垫，我们先来说明下 Service Capablities，这些都会用到，所以提前拉到前面想讲解下。
其中 Service Capablities 格式如下：
![A2DP Stack](../../assets/avdtp_service_capability.png)
此部分也是类似于TLV（Type Length Value）的类型，其中 Service Cate-gory 就是 TYPE,值如下：
![A2DP Stack](../../assets/avdtp_service_capability_value.png)
其中 Length Of Service Capabilities (LOSC)就是类似于 length,也就是后续 Service Capabilities Information Elements 的长度。
Service Capabilities Information Elements 这个就是特定的值，这个要分很大的篇章来讲，这里主要讲解一下Media Codec Capabilities。
![A2DP Stack](../../assets/avdtp_media_codec_capability.png)
其中 Media Type 有如下值：
![A2DP Stack](../../assets/avdtp_media_type.png)
其中 Media Codec Type 有如下值：
![A2DP Stack](../../assets/avdtp_media_codec.png)
此部分的 Media Codec Specific Information Elements 就是上层的一些 codec 信息，比如下图是SBC 的：
![A2DP Stack](../../assets/avdtp_sbc_media_codec.png)

- 3.1 AVDTP signal命令
每个 AVDTP 端都会注册一个或者多个 SEP,通过 SEID 来标示，这个命令就是获取对端的 SEP信息，包括 SEID(SEP 的 ID)，In Use(是否被使用)，Media Type(Audio,Media,MultiMedia),TSEP(角色是 Sink 还是 Source)
流程如下：
![A2DP Stack](../../assets/avdtp_discover.png)

- 3.2 Get Capabilities
此命令是通过 SEID 来获取对方的 Capabilities，其中 Capabilities 在前面已经介绍，我们来看下程序流程:
![A2DP Stack](../../assets/avdtp_get_capability.png)

- 3.3 Get All Capabilities
此命令是用于取代 Get Capabilities，同样是通过 SEID 来获取对方的 Capa-bilities，其中Capabilities 在前面已经介绍，我们来看下程序流程：
![A2DP Stack](../../assets/avdtp_get_all_capability.png)

- 3.4 Set Configuration Command
在获取 Capabilities 后，此部分就是选择特定的功能参数，程序流程如下：
![A2DP Stack](../../assets/avdtp_set_configure.png)

- 3.5 Get Stream configuration
此命令用于根据 SEID 来获取配置，程序流程如下：
![A2DP Stack](../../assets/avdtp_get_configure.png)

- 3.6 Stream Establishment
此命令用于打开某一个 SEID，即建立media channel，程序流程如下：
![A2DP Stack](../../assets/avdtp_stream_establishment.png)

- 3.7 Stream Start
此命令用于开启一个 SEID 的 media 传输，即开始播放音乐，程序流程如下：
![A2DP Stack](../../assets/avdtp_stream_start.png)

- 3.8 Stream Suspend
此命令用于暂停一个 media 的传输，即暂停音乐，程序流程如下：
![A2DP Stack](../../assets/avdtp_suspend.png)

- 3.9 Stream Release
此命令用于关闭一个 media 传输，程序流程如下：
![A2DP Stack](../../assets/avdtp_release.png)

## a2dp简介
A2DP(Advanced Audio Distribution Profile)是蓝牙高音质音频传输协议，用于传输单声道，双声道音乐（一般在 A2DP 中用于 stereo 双声道），典型应用为蓝牙耳机。A2DP 不包括远程控制的功能，远程控制的功能参考协议 AVRCP。
1. Audio codec
在上面流程我们也说了需要 Audio codec 音频算法，那么在 A2DP 协议中有以下规定：
![A2DP Stack](../../assets/a2dp_codec.png)
首先所有设备强制规定必须有 SBC 的 codec 算法，这个是有损算法，音质跟 MP3 差不多，另外还支持 3 种可选算法，MPEG-1,2 audio/MPEG-2,4 AAC、ATRAC family，当然还有一些自定义扩展 codec 算法，比如比较流行的 APTX,LDAC 等。
1.1 SBC codec
SBC 是蓝牙强制规定支持的协议，其中 Codec Specific Infomation Elements 定义如下：
![A2DP Stack](../../assets/a2dp_sbc_codec.png)
Sampling Frequency：这部分是采样频率，Source 端强制要求 44.1KHz，48KHz 支持一种，Sink要求 44.1KHz,48Khz 都支持，每个值对应的如下：
![A2DP Stack](../../assets/a2dp_sbc_sample_frequency.png)
Channel Mode：通道数，Sink 要求全支持，而 Source 只强制要求支持 Mono,其他可选：
![A2DP Stack](../../assets/a2dp_sbc_channel_mode.png)
Block Length：块长度
![A2DP Stack](../../assets/a2dp_sbc_block_length.png)
Subbands：次频带
![A2DP Stack](../../assets/a2dp_sbc_subbands.png)
Allocation Method：配置方法
![A2DP Stack](../../assets/a2dp_sbc_allocation_method.png)
Minimum Bitpool Value：
Maximum Bitpool Value：在播放设备中可以设置 SBC 编码质量，这个值叫 bitpool，大概 1 bitpool = 6～7 kbit/s。SBC是一种复杂度较低的编码格式，同等码率下音质稍差，根据这个网站上的比较，最高 328kbit/s 的 SBC 音质大约介于 224 kbit/s 到 256 kbit/s 的 MP3 之间。此外，设置不当、信号差、设备不支持高 bitpool 等都会造成传输码率下降而使音质下降，耳机或音箱本身的音质也是很重要的因素。以下为不同的 bitpool 的码率：
![A2DP Stack](../../assets/a2dp_sbc_bitpool.png)

# sifli SDK A2DP简介
本文档主要是基于Sifli SDK，介绍作为a2dp source角色如何使用a2dp的基本功能。
涉及文件如下：
- bts2_app_interface
- bts2_app_av
- bts2_app_av_src
  
## a2dp source初始化
- a2dp初始化的函数： bt_av_init，bt_avsrc_init，会先初始化a2dp，在初始化a2dp source

```c
//初始化a2dp
//用户可在此接口里添加自己的初始化流程，但不建议删除相关代码
void bt_av_init(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst;

    inst = bcalloc(1, sizeof(bts2s_av_inst_data));
    //管理a2dp相关的变量
    global_inst = inst;
    //初始化a2dp
    bt_av_init_data(inst, bts2_app_data);
    //初始化a2dp source
    bt_avsrc_init(inst, bts2_app_data);
}

//a2dp的初始化
static void bt_av_init_data(bts2s_av_inst_data *inst, bts2_app_stru *bts2_app_data)
{
    U8 i = 0;
    inst->que_id = bts2_app_data->phdl;

    //这个是配置a2dp的流端点，主要包括角色配置，类型配置，编码配置
    //流端点的数量可以通过修改MAX_NUM_LOCAL_SRC_SEIDS
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

    //初始化a2dp的连接信息，a2dp支持最多MAX_CONNS个连接（注意：最大连接数不是单独的sink还是source的连接数，而是两个角色总的）
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

//a2dp source的初始化
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
    //使能a2dp source的功能
    av_enb_req(inst->que_id, AV_AUDIO_SRC); //act the svc
}

//主要初始化一些和播放音乐有关的变量
static void bt_avsrc_init_data(bts2s_av_inst_data *inst, bts2_app_stru *bts2_app_data)
{
#ifdef AUDIO_USING_MANAGER
    inst->src_data.audio_state = AVSRC_AUDIO_SER_IDLE;
    inst->src_data.tid = 0;
    inst->src_data.stream_frm_time_begin = 0;
#endif // AUDIO_USING_MANAGER
}
```

## a2dp source连接设备
以下流程描述了a2dp source如何发现、连接和播放音乐的流程。
1. 第一步是发现周边可以使用的a2dp sink设备(例如：耳机，音响)。为此，a2dp source可以对附近的设备执行搜索，然后使用SDP从那些支持a2dp sink角色的设备中检索a2dp sink服务。
2. 选择一个a2dp sink设备进行连接。选择需要连接的设备发起ACL连接。
3. a2dp连接。一旦创建完成了ACL连接，a2dp source就可以发起a2dp signal channel的l2cap连接，连上a2dp signal channel后需要发起一系列a2dp command，然后a2dp source就可以发起a2dp media channel的l2cap连接。
4. a2dp source开始播放音乐。
5. a2dp sink或a2dp source可以随时终止连接。

- a2dp source连接设备接口为：：
    //用户可调用以下接口连接a2dp
    - bts2_app_interface连接接口：bt_interface_conn_to_source_ext
    - bts2_app_av_src连接接口：bt_avsrc_conn_2_snk
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

//调用这个接口之前会把mac转换为BTS2S_BD_ADDR蓝牙地址格式
//用户只需要调用bt_interface_conn_to_source_ext，不需要修改此接口
bt_err_t bt_avsnk_conn_2_src(BTS2S_BD_ADDR *bd)
{
    BTS2S_BD_ADDR temp = {0xffffff, 0xff, 0xffff};
    bt_err_t res = BT_EOK;

    //检查地址是否合法
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

//a2dp连接接口，a2dp sink和a2dp source都用同一个接口连接
//用户只需要调用bt_interface_conn_to_source_ext，不需要修改此接口
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
       
- a2dp source连接断开设备接口为：
    //用户可调用以下接口断开a2dp
    - bts2_app_interface断开连接接口：bt_interface_disc_ext
    - bts2_app_av_src断开接口：bt_avsnk_disc_by_addr
```c
/**
* @brief            Disconnect with the specified profile
* @param[in] mac    Remote device address
* @param[in] ext_profile : Profile value
*
* @return           bt_err_t
**/
bt_err_t bt_interface_disc_ext(unsigned char *mac, bt_profile_t ext_profile);

//调用这个接口之前会把mac转换为BTS2S_BD_ADDR蓝牙地址格式
//用户只需要调用bt_interface_disc_ext，不需要修改此接口
void bt_avsnk_disc_by_addr(BTS2S_BD_ADDR *bd_addr, BOOL is_close)
{
    bts2s_av_inst_data *inst_data;
    inst_data = bt_av_get_inst_data();
    //这个标志位表示是否要在断开a2dp之前发送a2dp close命令，一般为false即可
    if (is_close)
    {
        inst_data->close_pending = TRUE;
    }

    //因为a2dp支持多个连接，所以只会断开指定地址的a2dp
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

- a2dp source发送数据接口为
    - bts2_app_av_src发送数据接口：bt_avsrc_send_data
```c
//这个接口需要传入一个和audio模块一起使用的共享内存
//audio需要将解码后的数据存入这个共享内存，蓝牙需要从这个共享内存里取数据进行编码发送

//用户需要在audio模块实现相应的callback函数
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

    //找到对应的source连接
    con_idx = bt_avsrc_get_plyback_conn(inst);


    if (con_idx == - 1)
        return 0;

    //需要检查a2dp的状态
    if (inst->con[con_idx].st == avconned_streaming && !inst->suspend_pending)
    {
        do
        {
            U32 pcm_pkg_pos;
            U32 sb_pos = 0;
            U32 rd_len = 0;
            U16 pl;
            U16 buffer_count = av_get_stream_buffize();

            //source发送数据是靠定时器触发的，因此需要计算下一包数据发送的时间
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

            //如果发的数据很快，导致底层有很多cache包，则会先把底层的cache包发完
            if (buffer_count >= av_get_max_stream_buffer_cnt() / 2)
            {
                LOG_I("a2dp buffer more than half, should send buffered frames,count = %d\n", buffer_count);
                break;
            }

            //先检查和audio的共享buffer里数据长度是否够发一包，不够的话会通知audio模块解码
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
                //如果当前共享内存里的数据不够蓝牙发送一包数据，则会通过callback通知audio模块进行解码
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
            //如果当前共享内存里的数据已经消耗一半或者不够下一包数据，则会通过callback通知audio模块进行解码
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

            //将读出的pcm数据进行编码
            while (numrd > 0)
            {
                BTS2S_SBC_STREAM bss;

                bss.dst_len = pl;
                bss.pdst = &pkt_ptr[pcm_pkg_pos];
                bss.src_len = numrd;
                bss.psrc = &samples_ptr[sb_pos];

                //进行sbc编码
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
                        //发送a2dp media packet
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
        // 起一个定时器，准备发送下一包
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
        
- a2dp source事件处理:
    - a2dp source连接状态回调event:
        - a2dp source连接成功：BT_NOTIFY_A2DP_PROFILE_CONNECTED
        - a2dp source连接断开：BT_NOTIFY_A2DP_PROFILE_DISCONNECTED
    - a2dp sink播放状态回调event
        - a2dp source收到给对端发送的播放命令的response：BT_NOTIFY_A2DP_START_CFM
        - a2dp source收到给对端发送的暂停命令的response：BT_NOTIFY_A2DP_SUSPEND_CFM

:::{note}
注意：两个接口传递的地址参数需要进行相应的转换。
:::
```c
// 调用连接a2dp的API之后，a2dp连接成功的消息通过notify给用户
// 用户需要实现接收notify event的hdl函数 如：bt_notify_handle
// BT_NOTIFY_A2DP_PROFILE_CONNECTED event里面包含：地址信息 、profile_type、 res：0（成功）
// 断开a2dp后也会收到相应的事件
// BT_NOTIFY_A2DP_PROFILE_DISCONNECTED event里面包含：地址信息 、profile_type、 原因
// 具体结构体信息参考API注释
static int bt_notify_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    int ret = -1;

    //注册处理a2dp相关事件的notify函数
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

//用户需要注册a2dp notify函数，处理各种事件
static int bt_sifli_notify_a2dp_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_A2DP_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //用户自己实现相应的处理函数
        break;
    }
    case BT_NOTIFY_A2DP_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //用户自己实现相应的处理函数
        break;
    }
    case BT_NOTIFY_A2DP_START_CFM:
    {
        //收到对端设备回复的start response，用户自己实现相应的处理函数
        //在收到对端回复的start response后需要通知audio模块进行数据的解码，并且将解码后的数据送给蓝牙模块进行编码
        break;
    }
    case BT_NOTIFY_A2DP_SUSPEND_CFM:
    {
        //收到对端设备的suspend response，用户自己实现相应的处理函数
        break;
    }
    default:
        return -1;
    }
    return 0;
}
```

## a2dp source基本功能使用
### 1. 修改采样率等信息
```c
//本端设备连接对端的话，可以通过下面的接口选择a2dp的采样率等信息
//此接口用户可以根据需要修改一些配置，但不建议删除代码
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
    //配置采样率
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

    //配置channel mode
    for (i = 0; i < 4; i++)
    {
        if (*(serv_cap + 4) & (0x01 << i) & av_sbc_capabilities[0])
        {
            app_serv_cap[4] |= (0x01 << i);
            break;
        }
    }
    // app_serv_cap[4] |= 0x02;
    //配置block length
    for (i = 0; i < 4; i++)
    {
        if (*(serv_cap + 5) & (0x10 << i) & av_sbc_capabilities[1])
        {
            app_serv_cap[5] = (0x10 << i);
            break;
        }
    }
    //配置subbands
    for (i = 0; i < 2; i++)
    {
        if (*(serv_cap + 5) & (0x04 << i) & av_sbc_capabilities[1])
        {
            app_serv_cap[5] |= (0x04 << i);
            break;
        }
    }
    //配置method
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
### 2. 搜索耳机等外围设备
搜索的接口为：bt_interface_start_inquiry
- a2dp source搜索事件：BT_NOTIFY_COMMON_DISCOVER_IND

## a2dp功能使用demo
- 首先在工程初始化时，注册接收notify event 的处理函数
- 搜索可以连接的设备
- 输入需要连接手机的mac地址，等待连接成功的消息
- 手机播放音乐
```c
int bt_sifli_notify_a2dp_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    // a2dp CONNECTED
    case BT_NOTIFY_A2DP_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //用户自己实现相应的处理函数
        break;
    }
    //a2dp DISCONNECTED
    case BT_NOTIFY_A2DP_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //用户自己实现相应的处理函数
        break;
    }
    case BT_NOTIFY_A2DP_START_CFM:
    {
        //收到对端设备回复的start response，用户自己实现相应的处理函数
        //在收到对端回复的start response后需要通知audio模块进行数据的解码，并且将解码后的数据送给蓝牙模块进行编码
        break;
    }
    case BT_NOTIFY_A2DP_SUSPEND_CFM:
    {
        //收到对端设备的suspend response，用户自己实现相应的处理函数
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
    //bt功能开启成功，可以正常使用
    case BT_NOTIFY_COMMON_BT_STACK_READY:
    {
        break;
    }
    //bt关闭成功
    case BT_NOTIFY_COMMON_CLOSE_COMPLETE:
    {
        break;
    }
    //搜索事件处理
    case BT_NOTIFY_COMMON_DISCOVER_IND:
    {
        bt_notify_remote_device_info_t *remote_info = (bt_notify_remote_device_info_t *)data;
        //用户自己实现相应的处理函数,会上报搜索到的地址，名字等信息
        break;
    }
    // ACL 连接成功
    case BT_NOTIFY_COMMON_ACL_CONNECTED:
    {
        bt_notify_device_acl_conn_info_t *acl_info = (bt_notify_device_acl_conn_info_t *) data;
        //device acl connected
        break;
    }
    // ACL 断开连接成功
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

//开始搜索
bt_interface_start_inquiry();

// 连接耳机:001bdcf4b6bd
unsigned char mac[6] = {0x00,0x1b,0xdc,0xf4,0xb6,0xbd}
bt_interface_conn_to_source_ext((unsigned char *)(mac), BT_PROFILE_A2DP);
```