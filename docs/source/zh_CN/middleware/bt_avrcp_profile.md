# BT_AVRCP
AVRCP(Audio/Video Remote Control Profile)定义了蓝牙设备和 au-dio/video 控制功能通信的特点和过程，另用于远程控制音视频设备，底层传输基于 AVCTP 传输协议。该 Profile 定义了AV/C 数字命令控制集。命令和信息通过 AVCTP(Audio/Video Control Transport Protocol)协议进行传输。浏览功能通过 AVCTP 的第二个 channel 而不是 AV/C。传输媒体信息通过基于 OBEX协议的 BIP（Bluetooth Basic Imaging Profile）协议。
## AVCTP简介
1. AVCTP协议架构
AVCTP 协议描述了蓝牙设备间 Audio/Video 的控制信号交换的格式和机制，它是一个总体的协议，具体的控制信息由其指定的协议(如 AVRCP)实现，AVCTP 本身只指定控制 command和 response 的总体的格式。

几个重要的点：
&emsp;&emsp;(1)	AVCTP是基于点到点的l2cap连接。
&emsp;&emsp;(2)	每一条AVCTP连接都应该支持AVRCP CT和TG的功能。
&emsp;&emsp;(3)	每两个设备之间可能存在多条avctp连接，但是每个avctp连接的PSM值是唯一的。
&emsp;&emsp;(4)	每个avctp的包都应在一个l2cap包中传输。
&emsp;&emsp;(5)	不同 L2CAP channel 上的相同的 transaction Label 是属于不同的 message 的。也就是说，两条 L2CAP 上的 packets 是没有关系的，不可能属于同一个 message。
![AVCTP Architect](../../assets/avctp_arch.png)


2. AVCTP的封包格式
AVCTP 封包格式分为两种：
&emsp;&emsp;1）没有被分隔的（小于 L2CAP MTU）
&emsp;&emsp;2）被分隔的（大于 L2CAP MTU）
下面介绍下每种的格式：
&emsp;&emsp;1）没有被分隔的（小于 L2CAP MTU）
![AVCTP Packet](../../assets/avctp_packet.png)
   - Transaction label field：(octet 0, bits 7-4) 传输标示，由上层提供；

   - Packet_type field:(octet 0, bits 3 and 2) 此部分 00b 标示没有被分割；

   - C/R: (octet 0, bit 1) 0 代表 command,1 代表 response

   - PID :bit (octet 0, bit 0) 在 command 中设置为 0，在 response 中设置为 0 代表正常 PID

   - Profile Identifier (PID)：此部分填写 16bit 的 UUID，比如 AVRCP 的 UUID 0x110e后续的 Message Infomation 就是上层协议的数据

&emsp;&emsp;&emsp;&emsp;2）	被分隔的（大于 L2CAP MTU）
被分隔的的数据包格式一共有三种：
&emsp;&emsp;&emsp;&emsp;![AVCTP Start Packet](../../assets/avctp_start_packet.png)
&emsp;&emsp;&emsp;&emsp;![AVCTP Continue Packet](../../assets/avctp_continue_packet.png)
&emsp;&emsp;&emsp;&emsp;![AVCTP End Packet](../../assets/avctp_end_packet.png)
&emsp;&emsp;&emsp;&emsp;这里只介绍两个地方，其他跟没有被分隔的一致：
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Packet_type:开始封包是 01b, 继续封包 10b, 结束封包是 11b
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Number of AVCTP Packets：标示整个分隔封包的数量，包含这个开始封包，所以此封包的个数肯定是大于1的。


## AVRCP简介
1. AVRCP协议架构
![AVRCP ARCH](../../assets/avrcp_arch.png)
AVRCP Profile定义了两种角色:
    - CT（contorller devices）
      controller控制器是通过向目标设备发送命令帧来启动事务的设备，如耳机、音箱、车载蓝牙设备。
    - TG（target devices）
      target目标是接收控制器发送过来的命令帧并生成相应响应帧的设备，如手机。

2. AVRCP连接
AVCTP 的连接分为两个通道：Control 通道跟 Brwoing 通道,对应的 L2CAP PSM 不同，control通道的 PSM 为 0x0017,AVCTP browing 通道的 PSM 为 0x001B,两个通道 controller 跟 target 角色都可以发起连接，注意一点，AVCTP browing 部分基于 L2CAP 不能用 basic mode，需要用到 Enhanced Retransmission Mode，而且在双方都支持 AVCTP browing 的情况下才能发起AVCTP browing channel 的连接。
整个建立的过程如下流程：
![AVRCP Controller Connect](../../assets/avrcp_connect_ct.png)
![AVRCP Target Connect](../../assets/avrcp_connect_tg.png)
![AVRCP Mult Connect](../../assets/avrcp_connect_mult.png)


3. AVRCP断开
断开可以从 AVRCP controller 或者 target 角色操作，注意，如果有 AVCTP browing 的连线，那么先断开 AVCTP browing 通道的连接，整个流程如下：
![AVRCP Disconnect](../../assets/avrcp_disconnect.png)

4. AVRCP AV/C command 格式
&emsp;4.1 VENDOR DEPENDENT 格式
&emsp;&emsp;1）	command header 格式：
&emsp;&emsp;![AVRCP Vendor Command](../../assets/avrcp_vendor_command.png)
&emsp;&emsp;Ctype:这个是 command 类型，一共分为 5 种，一般我们在 AVRCP 协议中会用到 3 中，分别是 CONTROL 控制命令，STATUS 获取状态命令，NOTIFY 通知命令，分别类型如下：
&emsp;&emsp;![AVRCP Vendor Ctype](../../assets/avrcp_vendor_ctype.png)
&emsp;&emsp;Subunit_type：子信息类型，有如下定义，一般在 AVRCP 中用 PANEL
&emsp;&emsp;![AVRCP Vendor Subunit](../../assets/avrcp_vendor_subunit.png)
&emsp;&emsp;Subunit_id:此部分我们一般填 0
&emsp;&emsp;Opcode：我们仅仅需要知道每个特定的 comamnd opcode 是什么就行，VENDOR DEPENDENT的值为 0。
&emsp;&emsp;Company ID:此部分需要填写蓝牙 SIG 的 ID
&emsp;&emsp;2）	response header 格式：
&emsp;&emsp;![AVRCP Response Header](../../assets/avrcp_response_header.png)
&emsp;&emsp;可以看到 command 跟 response 格式完全相同，其他参数我们就不再重复，我们来说下Response 的值:
&emsp;&emsp;![AVRCP Response status](../../assets/avrcp_response_status.png)

&emsp;4.2 PASS THROUGH 格式
&emsp;&emsp;1）	Command：
&emsp;&emsp;![AVRCP Pass Through Command](../../assets/avrcp_pass_through_command.png)

&emsp;&emsp;1）	Response：
&emsp;&emsp;![AVRCP Pass Through Response](../../assets/avrcp_pass_through_response.png)

&emsp;&emsp;其他我们都不做过多介绍，只对 Operation_ID 以及 state_flag 做一个介绍
&emsp;&emsp;&emsp;Operation_ID:操作 ID
&emsp;&emsp;&emsp;![AVRCP Pass Through Operation ID](../../assets/avrcp_pass_through_operation_id.png)
&emsp;&emsp;&emsp;![AVRCP Pass Through Operation ID](../../assets/avrcp_pass_through_operation_id_1.png)
&emsp;&emsp;&emsp;![AVRCP Pass Through Operation ID](../../assets/avrcp_pass_through_operation_id_2.png)
&emsp;&emsp;&emsp;![AVRCP Pass Through Operation ID](../../assets/avrcp_pass_through_operation_id_3.png)

&emsp;&emsp;&emsp;State_flag:说白了就是分 press 跟 realease 动作，发送的时候press 的时候这个值为 0，realease 的时候这个值为 1

&emsp;4.3 AVRCP 特定的 command
&emsp;&emsp;AVRCP 特定的 command 有以下几种：
&emsp;&emsp;![AVRCP Special Command](../../assets/avrcp_special_command_0.png)
&emsp;&emsp;![AVRCP Special Command](../../assets/avrcp_special_command_1.png)
&emsp;&emsp;![AVRCP Special Command](../../assets/avrcp_special_command_2.png)
&emsp;&emsp;![AVRCP Special Command](../../assets/avrcp_special_command_3.png)
&emsp;&emsp;![AVRCP Special Command](../../assets/avrcp_special_command_4.png)
&emsp;&emsp;![AVRCP Special Command](../../assets/avrcp_special_command_5.png)
&emsp;&emsp;![AVRCP Special Command](../../assets/avrcp_special_command_6.png)

&emsp;&emsp;1）	Get Capabilities
&emsp;&emsp;![AVRCP Get Capability](../../assets/avrcp_get_capability.png)
&emsp;&emsp;这个是 AVRCP controller 发给 target 的，来获取对端设备的能力，包括公司名称，以及支持的 event。
&emsp;&emsp;![AVRCP Get Capability Packet](../../assets/avrcp_get_capability_packet.png)
&emsp;&emsp;获取公司 ID response 的格式为：
&emsp;&emsp;![AVRCP Get Capability Company](../../assets/avrcp_get_capability_cpmpany.png)
&emsp;&emsp;获取支持 event 的 response 格式为：
&emsp;&emsp;![AVRCP Get Capability Event](../../assets/avrcp_get_capability_event.png)
&emsp;&emsp;其中 EventID 有：
&emsp;&emsp;![AVRCP Event Support](../../assets/avrcp_event_support.png)
&emsp;&emsp;实际应用中我们一般只会用到 Get Capabiliby for event，来获取对端支持的 event,方便注册notify,可以看到上图支持播放状态改变，歌曲改变，播放器设置改变。


&emsp;&emsp;2）	Get PlayStatus
&emsp;&emsp;这个命令是 controller 向 target 发送获取播放状态的命令，返回值包括歌曲总长度，歌曲当前进度，以及播放状态，一共占用 9byte(4byte 歌曲长度，4byte 当前进度，1byte 播放状态)
&emsp;&emsp;![AVRCP Get Play Status](../../assets/avrcp_get_play_status.png)

&emsp;&emsp;3）	Register Notification
&emsp;&emsp;这个就是 CT 向 TG 注册消息，然后 TG 有对应的更新会通知，具体消息就是我们之前 get capability with event 的消息 （需要注意的是：每次注册只能一次生效，收到 change 后要重新注册）：
&emsp;&emsp;![AVRCP Register Notify](../../assets/avrcp_register_notify.png)
&emsp;&emsp;![AVRCP Register Notify](../../assets/avrcp_register_notify_1.png)

&emsp;&emsp;下面我们就 EVENT ID 一个个来说下：
&emsp;&emsp;EVENT_PLAYBACK_STATUS_CHANGED：播放状态改变，有以下值：
&emsp;&emsp;![AVRCP Play Status Register Notify](../../assets/avrcp_play_status_notify.png)
&emsp;&emsp;EVENT_TRACK_CHANGED：歌曲名称改变
&emsp;&emsp;EVENT_TRACK_REACHED_END：歌曲到结尾
&emsp;&emsp;EVENT_TRACK_REACHED_START：歌曲开始
&emsp;&emsp;EVENT_ PLAYBACK_POS_CHANGED：歌曲播放进度变化
&emsp;&emsp;![AVRCP Playback Pos Change Notify](../../assets/avrcp_playback_pos_change.png)
&emsp;&emsp;EVENT_BATT_STATUS_CHANGED：电量状态改变，有以下值：
&emsp;&emsp;![AVRCP Batt Status Change Notify](../../assets/avrcp_batt_status_change.png)

&emsp;&emsp;4） Metadata Attributes for Current Media Item
&emsp;&emsp;这个主要是获取歌曲信息的（包括名称/专辑名/歌手名/歌曲索引/歌曲总个数等等）
&emsp;&emsp;![AVRCP Get Element](../../assets/avrcp_get_element.png)
&emsp;&emsp;AttributeID 列表如下:
&emsp;&emsp;![AVRCP Get Element Attribute](../../assets/avrcp_get_element_attribute.png)

# sifli SDK AVRCP简介
本文档主要是基于Sifli SDK，介绍如何使用AVRCP的基本功能。
涉及文件如下：
- bts2_app_interface
- bts2_app_avrcp
  
## bt_avrcp初始化
- avrcp初始化的函数：bt_avrcp_int，初始化avrcp相关的状态、标志赋初始值
- avrcp服务使能的函数：bt_avrcp_open，使能avrcp profile

```c
//avrcp的状态
typedef enum
{
    avrcp_idle,
    avrcp_conned
} bts2_avrcp_st;

//avrcp初始化
//用户可在此接口里添加自己的初始化流程，但不建议删除相关代码
void bt_avrcp_int(bts2_app_stru *bts2_app_data)
{
    //初始化avrcp的状态为idle
    bts2_app_data->avrcp_inst.st = avrcp_idle;
    //release_type用来记录当前发送的是哪个命令的push
    bts2_app_data->avrcp_inst.release_type = 0x00;
    bts2_app_data->avrcp_inst.avrcp_time_handle = NULL;
    bts2_app_data->avrcp_inst.avrcp_vol_time_handle = NULL;
    bts2_app_data->avrcp_inst.volume_change_sem = rt_sem_create("bt_avrcp_vol_change", 1, RT_IPC_FLAG_FIFO);
    //记录对端是否有注册支持绝对音量
    bts2_app_data->avrcp_inst.tgRegStatus = 0;
    //记录对端是否有注册播放状态改变的通知事件
    bts2_app_data->avrcp_inst.tgRegStatus1 = 0;
    //是否正在调节音量的标志位
    bts2_app_data->avrcp_inst.abs_volume_pending = 0;
    //avrcp的播放状态
    bts2_app_data->avrcp_inst.playback_status = 0;
    //初始化的绝对音量，会在avrcp连上，且对端支持绝对音量时通知对端本端的初始音量
    bts2_app_data->avrcp_inst.ab_volume = 20;//default value;
    //可以通过宏CFG_OPEN_AVRCP来控制是否在初始化avrcp的时候enable avrcp
#ifdef CFG_OPEN_AVRCP
    bts2s_avrcp_openFlag = 1;
#else
    bts2s_avrcp_openFlag = 0;
#endif
    //enable avrcp
    if (1 == bts2s_avrcp_openFlag)
    {
        avrcp_enb_req(bts2_app_data->phdl, AVRCP_CT);
    }

    //主要用于记录一些当前的播放信息和歌曲信息
    memset(&mp3_detail_info, 0x00, sizeof(bt_avrcp_mp3_detail_t));
    mp3_detail_info.track_id = 0xff;
}

//enable avrcp的接口
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

## avrcp连接设备
以下流程描述了avrcp CT如何发现、连接和使用avrcp TG服务的过程。
1. 第一步是发现周边可以使用的avrcp TG设备。为此，avrcp CT可以对附近的设备执行搜索，然后使用SDP从那些支持avrcp TG角色的设备中检索avrcp TG服务。
2. 选择一个avrcp TG设备进行连接。选择需要连接的设备发起ACL连接。
3. AVRCP连接。一旦创建完成了ACL连接，avrcp CT应启动avrcp的L2CAP连接。
4. avrcp连上之后需要先查询对端支持的功能（可能对端也会查询本端支持的功能，并注册相应的通知事件），再注册对应的通知事件才能使用相应的功能（例如控制音乐暂停播放，调音量等）。
5. avrcp TG和avrcp CT可以随时终止连接。

- avrcp连接设备接口为：
    - bts2_app_interface连接接口：bt_interface_conn_ext
    - bts2_app_avrcp连接接口：bt_avrcp_conn_2_dev
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

//调用这个接口之前会把mac转换为BTS2S_BD_ADDR蓝牙地址格式
//avrcp的CT和TG共用一个连接接口，需要传入连接的角色
//用户只需要调用bt_interface_conn_ext，不需要修改此接口
void bt_avrcp_conn_2_dev(BTS2S_BD_ADDR *bd, BOOL is_target)
{
    bts2_app_stru *bts2_app_data = bts2g_app_p;

    //会判断是否在idle状态
    if (bts2_app_data->avrcp_inst.st == avrcp_idle)
    {
        USER_TRACE(" -- avrcp conn remote device...\n");
        USER_TRACE(" -- address: %04X:%02X:%06lX\n",
                bd->nap,
                bd->uap,
                bd->lap);

        //判断本端的角色时CT还是TG
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
       
- avrcp连接断开设备接口为：
    - bts2_app_interface断开连接接口：bt_interface_disc_ext
    - bts2_app_avrcp断开接口：bt_avrcp_disc_2_dev
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
void bt_avrcp_disc_2_dev(BTS2S_BD_ADDR *bd_addr)
{
    bts2_app_stru *bts2_app_data = getApp();
    //会先判断是不是已连接的状态
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
        
- avrcp事件处理:
    - avrcp连接状态回调event:
        - avrcp连接成功：BT_NOTIFY_AVRCP_PROFILE_CONNECTED
        - avrcp连接失败：BT_NOTIFY_AVRCP_PROFILE_DISCONNECTED

:::{note}
注意：两个接口传递的地址参数需要进行相应的转换。
:::
```c
// 调用连接avrcp的API之后，avrcp连接成功的消息通过notify给用户
// 用户需要实现接收notify event的hdl函数 如：bt_notify_handle
// BT_NOTIFY_AVRCP_PROFILE_CONNECTED event里面包含：地址信息 、profile_type、 res：0（成功）
// 断开avrcp后也会收到相应的事件
// BT_NOTIFY_AVRCP_PROFILE_DISCONNECTED event里面包含：地址信息 、profile_type、 原因
// 具体结构体信息参考API注释
static int bt_notify_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    int ret = -1;

    switch (type)
    {
        //注册处理avrcp相关事件的notify函数
        case BT_NOTIFY_AVRCP:
        {
            bt_sifli_notify_avrcp_event_hdl(event_id, data, data_len);
        }
        break;
    }
    return 0;
}

//用户需要注册a2dp notify函数，处理各种事件
static int bt_sifli_notify_avrcp_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_AVRCP_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //用户自己实现相应的处理函数
        break;
    }
    case BT_NOTIFY_AVRCP_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //用户自己实现相应的处理函数
        break;
    }
    default:
        return -1;
    }
    return 0;
}
```

## avrcp基本功能使用
### 1. 音乐控制命令
```c
//音乐播放命令
void bt_interface_avrcp_play(void);

//音乐暂停命令
void bt_interface_avrcp_pause(void);

//下一首命令
void bt_interface_avrcp_next(void);

//上一首命令
void bt_interface_avrcp_previous(void);

//倒带命令
void bt_interface_avrcp_rewind(void);

//调节音量
bt_err_t bt_interface_avrcp_volume_changed(U8 volume);

//如果本端作为TG，且对端设备支持绝对音量，则可以通过以下接口调节绝对音量
bt_err_t bt_interface_set_absolute_volume(U8 volume);
```
### 2. 可以通过以下接口来修改本端设备支持的能力，对端设备会根据回复的response来注册相应的event
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
    //修改capability的数量，并进行相应的赋值即可增删本端设备支持的event数量
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
### 3. 音乐信息相关命令
```c
//1.获取完对端支持的event后会通过BT_NOTIFY_AVRCP_CAPABILITIES_CFM上报，客户可以根据收到的event，结合需求调用以下接口使能对应的event notify：
//注册palyback status notify event，对端在播放状态改变时会通知本端设备
void bt_interface_avrcp_playback_register_request(void);

//注册palyback pos change notify event，对端在播放进度改变时会通知本端设备
void bt_interface_playback_pos_register_request(void);

//注册track change notify event，对端在播放歌曲改变时会通知本端设备
void bt_interface_track_change_register_request(void);

//注册volume change notify event，对端在播放音量改变时会通知本端设备
void bt_interface_volume_change_register_request(void);

//2.获取歌曲信息（例如：歌名、歌手、专辑等）
//media attribute type
#define AVRCP_MEDIA_ATTRIBUTES_TITLE 0x01   //歌曲名
#define AVRCP_MEDIA_ATTRIBUTES_ARTIST 0x02  //歌手名
#define AVRCP_MEDIA_ATTRIBUTES_ALBUM 0x03   //专辑
#define AVRCP_MEDIA_ATTRIBUTES_GENRE 0x06   //流派
#define AVRCP_MEDIA_ATTRIBUTES_PLAYTIME 0x07 //歌曲长度

//通过这个接口获取对应的歌曲信息
void bt_interface_avrcp_get_element_attributes_request(U8 media_attribute);
```
### 4. 获取对端的播放状态
void bt_interface_avrcp_get_play_status_request(void);

## avrcp功能使用demo
- 首先在工程初始化时，注册接收notify event 的处理函数
- 输入需要连接手机的mac地址，等待连接成功的消息
- 播放手机音乐，控制手机播放暂停
```c
int bt_sifli_notify_avrcp_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    // AVRCP连接事件
    case BT_NOTIFY_AVRCP_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //用户自己实现相应的处理函数
        break;
    }
    // AVRCP断开事件
    case BT_NOTIFY_AVRCP_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //用户自己实现相应的处理函数
        break;
    }
    // 获取完对端设备正在播放的音乐的相关信息
    case BT_NOTIFY_AVRCP_MP3_DETAIL_INFO:
    {
        bt_notify_avrcp_mp3_detail_t *mp3_detail_info = (bt_notify_avrcp_mp3_detail_t *)data;
        //用户自己实现相应的处理函数,
        break;
    }
    // 收到对端设备来注册绝对音量的请求事件
    case BT_NOTIFY_AVRCP_VOLUME_CHANGED_REGISTER:
    {
        //用户自己实现相应的处理函数
        break;
    }
    // 收到对端设备来调节绝对音量
    case BT_NOTIFY_AVRCP_ABSOLUTE_VOLUME:
    {
        uint8_t *relative_volume = (uint8_t *)data;
        //用户自己实现相应的处理函数
        break;
    }
    // 播放状态改变的通知事件
    case BT_NOTIFY_AVRCP_PLAY_STATUS:
    {
        uint8_t *play_status_notify = (uint8_t *)data;
        //用户自己实现相应的处理函数
        break;
    }
    // 播放进度改变的通知事件
    case BT_NOTIFY_AVRCP_SONG_PROGREAS_STATUS:
    {
        uint32_t *play_pos = (uint32_t *)data;
        //用户自己实现相应的处理函数
        break;
    }
    // 播放歌曲改变的通知事件
    case BT_NOTIFY_AVRCP_TRACK_CHANGE_STATUS:
    {
        uint8_t *track_change = (uint8_t *)data;
        //用户自己实现相应的处理函数
        break;
    }
    // 获取完对端支持的capabilities通知事件
    case BT_NOTIFY_AVRCP_CAPABILITIES_CFM:
    {
        bt_notify_avrcp_capabilities_cfm_t *track_change = (bt_notify_avrcp_capabilities_cfm_t *)data;
        //用户自己实现相应的处理函数,根据对端支持的capabilities，客户可决定要注册哪些event notify
        break;
    }
    // 获取完对端播放音乐的某个信息的通知事件(例如歌手名、歌曲名等)
    case BT_NOTIFY_AVRCP_MEDIA_ATTRIBUTE_CFM:
    {
        bt_notify_avrcp_media_attribute_cfm_t *track_change = (bt_notify_avrcp_media_attribute_cfm_t *)data;
        //用户自己实现相应的处理函数,根据media_attribute可确定获取的是什么信息
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

// 1.连接手机a2dp:001bdcf4b6bd
unsigned char mac[6] = {0x00,0x1b,0xdc,0xf4,0xb6,0xbd}
bt_interface_conn_ext((unsigned char *)(mac), BT_PROFILE_A2DP);
// 2.连接手机avrcp
bt_interface_conn_ext((unsigned char *)(mac), BT_PROFILE_AVRCP);
// 3.手机播放音乐，控制手机音乐暂停
bt_interface_avrcp_pause();

```