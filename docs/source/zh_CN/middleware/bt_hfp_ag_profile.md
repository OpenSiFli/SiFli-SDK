# HFP_AG
HFP(Hands-Free Profile)，是蓝牙免提协议，可以让蓝牙设备对对端蓝牙设备的通话进行控制，例如蓝牙耳机控制手机通话的接听、挂断、拒接、语音拨号等。HFP中蓝牙两端的数据交互是通过定义好的AT指令来通讯的。HFP定义了音频网关(AG)和免提组件(HF)两个角色：
- 音频网关(AG)：该设备为音频输入/输出的网关 。典型作为网关的设备为手机
- 免提组件(HF)：该设备作为音频网关的远程音频输入/输出机制，并可提供若干遥控功能。典型作为免提组件的设备为车机、蓝牙耳机  
![HFP Role](../../assets/HFP_ARCH.png)
- AT命令的规则：
    - 一个命令行，只能代表一个AT命令
    - < cr > carriage return的简写，相当于回车键，ASCII码为 0x0D
    - < lf > NL line feed, new line的简写，相当于换行键，ASCII码为0x0A
    - HF -> AG发送AT命令格式：< AT command >< cr >
    - AG -> HF发送AT命令格式：< cr >< lf >result code< cr >< lf > 
    - AG给HF发送result code的AT命令如果是消息回复，后面都得再回复一条OK消息，除非回复的是+CME ERROR消息，后面的参数代表失败的原因。  

本文档主要是基于Sifli SDK，介绍如何使用AG role的基本功能。涉及文件如下：
- bts2_app_interface
- bts2_app_hfp_ag
## HFP_AG初始化
- AG初始化的函数：bt_hfp_ag_app_init，AG相关的状态、标志赋初始值
- AG服务启动的函数：bt_hfp_start_profile_service，配置+BRSF相关的属性，用户可以根据需求调整相应的feature值
- AT cmd: 
    - +BRSF: AG support features (Bluetooth Retrieve Supported Features)
    - 格式：+BRSF:< AG support features >
    - HF侧将自己支持的features发送给AG侧后，AG端也得将它支持的features通过“+BRSF:< AG support features >”发送给HF。
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
## HFP_AG连接设备
当LM Link 和 RFCOMM Connection已存在的情况下，用户行为或者其他内部事件想要使用HFP服务就需要先建立SLC(Service Level Connection)连接。建立SLC连接需要进行一下5个阶段，分别是：
1. Supported features exchange(AT+BRSF)
2. Codec Negotiation(AT+BAC)
3. AG Indicators(AT+CIND 、AT+CMER 、+CIEV、AT+CHLD )
4. HF Indicators(AT+BIND、AT+BIEV)
5. End of Service Level Connection  
![HFP Connect Progress](../../assets/hfp_connect_progress.png)
- AT cmd: 
    - AT+BAC: (Bluetooth Available Codecs)
    - 格式：AT+BAC=< codec_id1 >,< codec_id2 >
    - HF侧告知AG侧支持哪些编码方式
    - codec: CVSD 和 msbc
- AT cmd: 
    - AT+CIND: (Standard indicator update AT command)
    - 格式：
        - AT+CIND=？测试指令。HF获取AG侧支持的指示器索引值和范围。在发送关于指示器的相关指令（AT+CIND? 或 AT_CMER）前，至少请求一次。 
        - AT+CIND? 读指令，HF读取AG侧当前的指示器各个值
- AT cmd: 
    - +CIEV: unsolicited result code(Standard indicator events reporting unsolicited result code)
    - 格式：+CIEV: < ind >,< value >
    - 当AG测indicators发生变化时，AG需要主动用+ciev AT cmd 通知HF侧
    ![indicator type](../../assets/HFP_AG_INDICATOR.png)
- AG连接设备接口为：
    - bts2_app_interface连接接口：bt_interface_conn_to_source_ext
    - bts2_app_hfp_ag连接接口：bt_hfp_connect_profile
       
- AG连接断开设备接口为：
    - bts2_app_hfp_ag断开接口：bt_hfp_disconnect_profile
        
- AG连接状态回调event: 
        - AG连接成功：BT_NOTIFY_AG_PROFILE_CONNECTED 
        - AG连接失败：BT_NOTIFY_AG_PROFILE_DISCONNECTED
```c
// 调用连接AG role的API之后，AG连接成功的消息通过notify给用户
// 用户需要实现接收notify event的hdl函数 如：bt_notify_handle
// SLC的消息：BT_NOTIFY_AG_PROFILE_CONNECTED ，用户可以将消息转发到用户task里面处理。
// BT_NOTIFY_AG_PROFILE_CONNECTED  event里面包含：地址信息 、profile_type、 res：0（成功）
// 断开设备连接调用bt_hfp_disconnect_profile
//BT_NOTIFY_AG_PROFILE_DISCONNECTED event里面包含：地址信息 、profile_type、 原因
//具体结构体信息参考API注释
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
        //handle conencted msg(bt_notify_profile_state_info_t *)
        break;
    }
    case BT_NOTIFY_AG_PROFILE_DISCONNECTED:
    {
        //handle disconneted msg(bt_notify_profile_state_info_t *)
        break;
    }
    }
    return 0;
}
```
:::{note}
注意：两个接口传递的地址参数需要进行相应的转换。
:::
- AG SLC连接过程中，收到HF端AT+CIND=?cmd时event：BT_NOTIFY_AG_GET_INDICATOR_STATUS_REQ
- AG端需要回复event：BT_NOTIFY_AG_GET_INDICATOR_STATUS_REQ
    - bts2_app_interface回复indicator状态接口：bt_interface_get_all_indicator_info_res
    - bts2_app_hfp_ag回复indicator状态接口：bt_hfp_ag_cind_response
```c
typedef struct
{
    // 0（No home/roam network available）/1(home/roam network available)
    U8 service_status;
    //  0（There are no calls in progress）/1(At least one call in progress)
    U8 call;
    // 0(No currently in call set up)
    // 1(An incoming call process ongoing)
    // 2(An outgoing call process ongoing)
    // 3(Remote party being alert in an outgoing call)
    U8 callsetup;
    // Phone battery val（0~5）
    U8 batt_level;
    // Phone signal val（0~5）
    U8 signal;
    // 0（Roaming is not active）/1(Roaming is active)
    U8 roam_status;
    // 0（No call held）
    // 1 (Call is placed on hold or active/held call swapped)
    // 2 (Call on hold,no active call)
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
## HFP_AG基本功能使用
### 通话音频的建立
HFP specification中Audio Connection通常是指SCO/eSCO语音通路的连接，在SCO/eSCO之前，HF(AT+BCC)需要通知AG先对编解码算法进行选择。
- AT cmd: 
    - AT+BCC：Bluetooth Codec Connection
    - 格式：AT+BCC
    - HF发送给AG，触发AG发起编解码器连接过程
    - AG决定发起编解码器连接过程，则回复OK；否则ERROR。回复ok之后，AG侧会发出+BCS:< codec_id >  HF侧回复:AT+BCS= < codec_id >.之后建立好esco链接
- AT cmd: 
    - AT+BCS：Bluetooth Codec Selection
    - 格式：AT+BCS=< codec_id >
    - +BCS: Bluetooth Codec Selection
    - 格式：+BCS:< codec_id >
    - codec:(codec_id=1) CVSD 和 (codec_id=2) msbc
    - AG建立esco之前将会发送命令+BCS:< codec_id >给HF。HF侧回复:AT+BCS=< codec_id >. AG和HF同时支持这个id，则会建立好而是从链路。但是如果ID不支持，HF将以AT+BAC和其可用的codec作为应答。如果(e)SCO link无法建立，AG将会重启启动Codec Connection建立过程。在Codec connection建立连接之前，CVSD编码将被启用。
#### Sifli HFP_AG 连接/断开通话音频
- bts2_app_interface中通话音频连接接口：bt_interface_ag_audio_switch
- bts2_app_hfp_ag中通话音频连接接口：bt_hfp_connect_audio
- bts2_app_interface中通话音频断开接口：bt_interface_ag_audio_switch
- bts2_app_hfp_ag中通话音频断开接口：bt_hfp_disconnect_audio
```c
    // 通话音频建立连接接口bt_interface_ag_audio_switch中入参 ：type 0:connect audio type 1 :disconnect audio + 设备的mac地址
    // 通话音频连接成功BT_NOTIFY_COMMON_SCO_CONNECTED event: sco type(区分HF或者AG) status（状态）
    //具体结构体解释请参考interface中相关说明
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
        //handle sco conencted msg(bt_notify_device_sco_info_t *)
        break;
    }
    case BT_NOTIFY_COMMON_SCO_DISCONNECTED:
    {
        //handle sco disconneted msg(bt_notify_device_sco_info_t *)
        break;
    }
    }
    return 0;
}
```
### AG电话状态更新
- AT cmd: 
    - AT+CIND: (Standard indicator update AT command)
    - 格式：
        - AT+CIND=？测试指令。HF获取AG侧支持的指示器索引值和范围。在发送关于指示器的相关指令（AT+CIND? 或 AT_CMER）前，至少请求一次。 
        - AT+CIND? 读指令，HF读取AG侧当前的指示器各个值
- AT cmd: 
    - +CIEV: unsolicited result code(Standard indicator events reporting unsolicited result code)
    - 格式：+CIEV: < ind >,< value >
    - 当AG测indicators发生变化时，AG需要主动用+ciev AT cmd 通知HF侧
- AT cmd: 
    - AT+CLCC: Standard list current calls command
    - 格式：AT+CLCC
    - +CLCC：Standard list current calls command
    - 格式：+CLCC: < idx >,< dir >,< status >,< mode >,< mpty >,< number >,< type >
    - HF请求当前的电话信息列表，AG侧通过"+CLCC"回复当前的电话信息列表。如果当前没有电话，AG侧也需回复OK指令。

一旦手机AG端电话状态发生改变，AG应该通知HF当前已变化call status，比如，来电之后，蓝牙耳机HF端接听，电话状态发生改变，AG侧+CIEV:2,1通知HF拒接或者AG挂断，AG 侧+CIEV:2,0通知HF端。
- call Status
    - 0: No calls(held or active)
    - 1: Call is present(active or held)
- callsetup status
    - 0: No call setup in progress
    - 1: Incoming call setup in progress
    - 2: Outgoing call setup in dialing state
    - 3: Outgoing call setup in alerting state
- callheld status
    - 0: No call held
    - 1: Call is placed on hold or active/held calls swapped
    - 2: Call on hold, no active call
- 来电状态转换： 
    -  call_idle<---->call_incoming<----->call_active<---->call_idle
- 去电状态转换： 
    - call_idle<---->call_outgoing_dialing---->call_outgoing_alerting<----->call_active<---->call_idle
    - call_idle<---->call_outgoing_dialing---->call_outgoing_alerting<----->call_idle
#### Sifli AG更新电话信息
- bts2_app_interface中电话状态更新接口：bt_interface_phone_state_changed
- bts2_app_hfp_ag中电话状态更新接口：bt_hfp_ag_call_state_update_listener
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
### AG信息同步处理
- AT cmd: 
    - +CIEV: unsolicited result code(Standard indicator events reporting unsolicited result code)
    - 格式：+CIEV: < ind >,< value >
    - 当AG测indicators发生变化时，AG需要主动用+ciev AT cmd 通知HF侧
- indicator 状态值变化时更新
    - 当AG端indicators发生变化时，AG需要主动更新状态
    - bts2_app_interface中更新状态接口：bt_interface_indicator_status_changed
    - bts2_app_hfp_ag中更新状态接口：bt_hfp_ag_ind_status_update
    - 常用的indicator type 如下图所示：  
    ![图1：indicator type](../../assets/HFP_AG_INDICATOR.png)
    - service状态更新demo：
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
ind_info.ind_type = HFP_AG_CIND_SERVICE_TYPE ;
ind_info.ind_val  = 3;
bt_interface_indicator_status_changed(&ind_info);
```
#### AG本地号码同步
- AT cmd: 
    - AT+CNUM: Subscriber Number Information
    - 格式：AT+CNUM
    - 该指令用于查询本机号码
    - +CNUM: Subscriber Number Information
    - 格式：+CNUM: ,< number >,< type >[,, < service >]
    - AG收到该请求之后，会将手机本机号码通过+CNUM回复给HF侧。
- 收到HF端获取本机号码的请求event：BT_NOTIFY_AG_GET_LOCAL_PHONE_INFO_REQ
- 回复本机号码的接口：
    - bts2_app_interface中回复本机号码接口：bt_interface_local_phone_info_res 
    - bts2_app_hfp_ag中回复本机号码接口：bt_hfp_ag_cnum_response
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
#### AG所有电话的状态信息
- AT cmd: 
    - AT+CLCC: Standard list current calls command
    - 格式：AT+CLCC
    - +CLCC：Standard list current calls command
    - 格式：+CLCC: < idx >,< dir >,< status >,< mode >,< mpty >,< number >,< type >
    - HF请求当前的电话信息列表，AG侧通过"+CLCC"回复当前的电话信息列表。如果当前没有电话，AG侧也需回复OK指令。
- 收到HF端获取所有电话的状态信息的event：BT_NOTIFY_AG_GET_ALL_REMT_CALLS_INFO_REQ
- 回复所有电话的状态信息的接口：
    - bts2_app_interface中所有电话的状态信息接口：bt_interface_remote_call_info_res
    - bts2_app_hfp_ag中所有电话的状态信息接口：bt_hfp_ag_remote_calls_res_hdl
```c
typedef struct
{
    //电话号码数量
    U8 num_call;
    hfp_phone_call_info_t *calls;
} hfp_remote_calls_info_t;

typedef struct
{
    //当前电话是第几路电话，从1开始计数
    U8 call_idx;
    // 电话方向，0:往外拨打的电话outgoing；1:来电incoming
    U8 call_dir;
    // 0: Active
    // 1: held
    // 2: Dialing(outgoing calls only)
    // 3: Alerting(outgoing calls only)
    // 4: Incoming (incoming calls only)
    // : Waiting (incoming calls only)
    // 6: Call held by Response and Hold
    U8 call_status;
    // 电话模式，0 (Voice), 1 (Data), 2 (FAX)
    U8 call_mode;
    // 是否为多方通话的电话。0:不是多方通话，1：是多方通话
    U8 call_mtpty;
    // 电话号码 + 电话号码类型
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
### HFP_AG 电话相关功能
- AT cmd: 
    - AT+CLIP: Calling Line Identification notification
    - 格式：AT+CLIP=（0/1）
    - +CLIP： Calling Line Identification notification
    - 格式：+CLIP: < number >,< type >
    - 使能或关闭主叫号码显示通知，使能后AG侧在来电时通过“+CLIP”指令将当前来电的号码和类型发送到HF
- AT cmd: 
    - AT+CCWA：Three-Way Call Handling
    - 格式：AT+CCWA =（0/1）
    - 使能或关闭三方电话waiting 提醒
    - +CCWA ：Call Waiting Notification
    - 格式：+CCWA: < number >,< type >
    - 使能后，当已有一个接通电话，那么再进来电话时，AG就会发出+CCWA
- AT cmd: 
    - ATD：Dial call req
    - 格式：ATDdd…dd
    - HF主动请求拨打电话
    - AT+BLDN：Bluetooth Last Dialed Number
    - 格式：AT+BLDN
    - 请求AG侧回拨最后一通电话。收到该请求后，AG根据最近一次拨打的号码，发起打电话。
- AT cmd: 
    - ATA: call answer
    - 格式：ATA
    - 应答电话，当接通后，出现+CIEV:< call_ind >, < 1 > 和 +CIEV:< call_setup_ind >, < 0 >
- AT cmd: 
    - AT+CHUP: Reject an Incoming Call/Terminate a Call Process
    - 格式：AT+CHUP
    - 拒接或者挂断电话，出现+CIEV:< call_setup_ind >, < 0 > / +CIEV:< call_ind >, < 0 >

- 电话相关功能
    - 收到HF端接听电话的event：BT_NOTIFY_AG_ANSWER_CALL_REQ
    - 收到HF端挂断电话的event：BT_NOTIFY_AG_HANGUP_CALL_REQ
    - 收到HF端控制通话音量的event：BT_NOTIFY_AG_VOLUME_CHANGE
    - 收到HF端拨打电话的event：BT_NOTIFY_AG_MAKE_CALL_REQ
    - 收到拨打电话请求时，需要对number进行合法性检查，检查结果回复
        - bts2_app_interface中结果回复接口：bt_interface_make_call_res
        - bts2_app_hfp_ag中结果回复接口：bt_hfp_ag_at_result_res
    - 通话过程中，收到DTMF key的event：BT_NOTIFY_AG_RECV_DTMF_KEY
- AT cmd: 
    - AT+VGM：Gain of Microphone
    - 格式： AT+VGM=< gain >
    - +VGM: Gain of Microphone
    - 格式： +VGM:< gain >
    - 调节audio 通路中MIC的音量，取值范围 0~15
- AT cmd: 
    - AT+VGS: Gain of Speaker
    - 格式： AT+VGS=< gain >
    - +VGS: Gain of Speaker
    - 格式： +VGS:< gain >
    - 调节audio 通路中speaker的音量，取值范围 0~15
- 主动控制音量接口：
    - bts2_app_interface speaker音量控制接口：bt_interface_set_speaker_volume
    - bts2_app_hfp_hf speaker音量控制接口：bt_hfp_hf_update_spk_vol
    - bts2_app_hfp_hf microphone音量控制接口：bt_hfp_hf_update_mic_vol
    - 设置音量处理结果event：BT_NOTIFY_HF_AT_CMD_CFM中cmd_id(HFP_HF_AT_VGS/HFP_HF_AT_VGM)
    - 主动控制speaker音量：
        - bts2_app_interface中控制speaker音量接口：bt_interface_spk_vol_change_req
        - bts2_app_hfp_ag中控制speaker音量接口：bt_hfp_ag_spk_vol_control
    - 主动控制microphone音量：
        - bts2_app_interface中控制microphone音量接口：bt_interface_mic_vol_change_req
        - bts2_app_hfp_ag中控制microphone音量接口：bt_hfp_ag_mic_vol_control
```c
//register notify event handle function start
//step1: 调用连接接口将AG连接成功消息：BT_NOTIFY_AG_PROFILE_CONNECTED
int bt_sifli_notify_hfp_ag_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    //AG端 SLC CONNECTED
    case BT_NOTIFY_AG_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        break;
    }
    //AG端 SLC DISCONNECTED
    case BT_NOTIFY_AG_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        break;
    }
    //AG端收到HF端的打电话请求，AG端需要判断号码的合法性
    //处理结果通过bt_hfp_ag_at_result_res(res)发回HF
    case BT_NOTIFY_AG_MAKE_CALL_REQ:
    {
        break;
    }
    //AG端收到HF端的接听来电请求
    //有来电可以发送消息
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
    //AG端收到HF端的拒接或挂断点电话
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
    //AG端收到HF端发送的DTMF值
    case BT_NOTIFY_AG_RECV_DTMF_KEY:
    {
        break;
    }
    //AG端收到HF端音量控制
    case BT_NOTIFY_AG_VOLUME_CHANGE:
    {
        break;
    }
    //AG端收到HF端请求所有当前indicators值
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
    //AG端收到HF端请求所有当前电话列表信息
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
    //AG端收到HF端请求本地电话号码
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
    // SCO 连接成功
    case BT_NOTIFY_COMMON_SCO_CONNECTED:
    {
        bt_notify_device_sco_info_t *sco_info = (bt_notify_device_sco_info_t *)data;
        break;
    }
    // SCO 断开连接成功
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