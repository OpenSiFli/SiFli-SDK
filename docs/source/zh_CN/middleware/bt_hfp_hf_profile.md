# HFP_HF
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

本文档主要是基于Sifli SDK，介绍如何使用HF role的基本功能。涉及文件如下：
- bts2_app_interface
- bts2_app_hfp_hf
## HFP_HF初始化
- HF初始化的函数：bt_hfp_hf_init，HF相关的状态、标志赋初始值
- HF服务启动的函数：bt_hfp_hf_start_enb，配置AT+BRSF相关的属性，用户可以根据需求调整相应的feature值
- AT cmd: 
    - AT+BRSF : HF support features (Bluetooth Retrieve Supported Features)
    - 格式：AT+BRSF=< HF support features >
    - 建立连接时，HF侧将自己支持的features发送给AG侧
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
## HFP_HF连接设备
AG 设备和 HF设备建立连接到进行通话，大致会经历一下四个阶段。接下来会根据Specification重点介绍Service Level Connection 和 Audio Connection的过程。
1. LMP link：首先建立LMP Link，HFP连接中没有固定的master 或者slave
2. RFCOMM Connection：接着建立仿真串口端口data link channel，用于HF传输用户数据给AG（包括调制器的控制信号和 AT command），AG解析AT command将相应的responses通过仿真串口端口发送给HF
3. Service Level Connection：服务级别的连接是应用层HF <<>>AG控制和交互信息的基础。
    - Service Level Connection establishment
    - Service Level Connection release
4. Synchronous Connection/Audio Connection：SCO/eSCO 通话的连接通常是指语音数据的连接
    - Codec Connection
    - Wide Band Speech Connection
### SLC(Service Level Connection)建立连接流程
当LM Link 和 RFCOMM Connection已存在的情况下，用户行为或者其他内部事件想要使用HFP服务就需要先建立SLC(Service Level Connection)连接。建立SLC连接需要进行一下5个阶段，分别是：
1. Supported features exchange(AT+BRSF)
2. Codec Negotiation(AT+BAC)
3. AG Indicators(AT+CIND 、AT+CMER 、+CIEV、AT+CHLD )
4. HF Indicators(AT+BIND、AT+BIEV)
5. End of Service Level Connection  
![HFP Connect Progress](../../assets/hfp_connect_progress.png)

### Sifli HFP_HF 连接/断开设备
- HF连接设备接口为：
    - bts2_app_interface连接接口：bt_interface_conn_ext
    - bts2_app_hfp_hf连接接口：bt_hfp_hf_start_connecting 
       
- HF连接断开设备接口为：
    - bts2_app_interface断开连接接口：bt_interface_disc_ext
    - bts2_app_hfp_hf断开接口：bt_hfp_hf_start_disc
        
- HF连接状态回调event:
    - HF连接成功：BT_NOTIFY_HF_PROFILE_CONNECTED
    - HF连接失败：BT_NOTIFY_HF_PROFILE_DISCONNECTED
:::{note}
注意：两个接口传递的地址参数需要进行相应的转换。
:::
```c
// 调用连接HF role的API之后，HF连接成功的消息通过notify给用户
// 用户需要实现接收notify event的hdl函数 如：bt_notify_handle
// SLC的消息：BT_NOTIFY_HF_PROFILE_CONNECTED，用户可以将消息转发到用户task里面处理。
// BT_NOTIFY_HF_PROFILE_CONNECTED event里面包含：地址信息 、profile_type、 res：0（成功）
// 断开设备连接调用bt_hfp_hf_start_disc
//BT_NOTIFY_HF_PROFILE_DISCONNECTED event里面包含：地址信息 、profile_type、 原因
//具体结构体信息参考API注释
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
        //handle hf conencted msg(bt_notify_profile_state_info_t *)
        break;
    }
    case BT_NOTIFY_HF_PROFILE_DISCONNECTED:
    {
        //handle disconneted msg(bt_notify_profile_state_info_t *)
        break;
    }
    }
    return 0;
}
```
## HFP_HF基本功能使用
### 通话音频的建立
HFP specification中Audio Connection通常是指SCO/eSCO语音通路的连接，在SCO/eSCO之前，HF(AT+BCC)需要通知AG先对编解码算法进行选择。
- AT cmd: 
    - AT+BCC：Bluetooth Codec Connection
    - 格式：AT+BCC
    - HF发送给AG，触发AG发起编解码器连接过程
    - AG决定发起编解码器连接过程，则回复OK；否则ERROR。回复ok之后，AG侧会发出+BCS:< codec_id >  HF侧回复:AT+BCS= < codec_id >.之后建立好esco链接
#### Sifli HFP_HF 连接/断开通话音频
- bts2_app_interface中通话音频连接/断开接口：bt_interface_audio_switch
- bts2_app_hfp_hf中通话音频连接/断开接口：bt_hfp_hf_audio_transfer 
- 通话音频建立成功的前提条件为：手机端通话状态非空闲；因为通话空闲状态时，手机端会拒绝 AT+BCC建立sco的请求。
- 通话音频编码包括：CVSD和 msbc，控制msbc codec是否启动的接口：bt_interface_set_wbs_status 
- 通话音频建立成功之后，需要调用audio_open,通话音频断开或者HF断开之后需要调用audio_close,相关实现可以查看：hfp_audio_api
```c
    // 通话音频建立连接接口bt_interface_audio_switch中入参 ：type 0:connect audio type 1 :disconnect audio
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
        //handle hf sco conencted msg(bt_notify_device_sco_info_t *)
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
### 通话音频音量控制
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
- 对端设备控制音量event：BT_NOTIFY_HF_VOLUME_CHANGE
```c
// 当存在通话音频，控制通话音频音量时，调用bt_interface_set_speaker_volume 传入音量值
// HF 会采用AT+VGS 格式将音量值通知 AG
// AG侧收到音量值之后，处理成功回复OK，处理失败回复ERROR
// 用户对应的事件event：BT_NOTIFY_HF_AT_CMD_CFM中cmd_id(HFP_HF_AT_VGS/HFP_HF_AT_VGM)
// 具体结构体解释请参考interface中相关说明
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
    HFP_HF_AT_VGS, //speaker音量控制cmd_id
    HFP_HF_AT_VGM, //microphone音量控制cmd_id
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
### 语音识别功能
- AT cmd: 
    - AT+BVRA: Voice Recognition Activation
    - 格式： AT+BVRA=< status >
    - status
        - 0(Disable Voice recognition in the AG)
        - 1(Enable voice recognition in the AG)
        - 2(Enable voice recognition in the AG with esco exist)
- AT cmd: 
    - +BVRA: Voice Recognition Activation
    - 格式： +BVRA:< status >
    - status
        - 0(Voice recognition is disabled in the AG)
        - 1(Voice recognition is enabled in the AG)  

HF通过发送AT+BVRA命令给AG来发起激活语音识别应用。语音识别应用安装在AG中。除了audio的路由、语音识别的激活控制等功能需要用到蓝牙功能，其余都依赖于语音识别应用的实现。
- 对端设备支持语音识别功能event：BT_NOTIFY_HF_VOICE_RECOG_CAP_UPDATE
- 对端设备主动使用语音识别功能状态同步event：BT_NOTIFY_HF_VOICE_RECOG_STATUS_CHANGE
- 主动唤醒/关闭语音功能接口：
    - bts2_app_interface语音识别接口：bt_interface_voice_recog 
    - bts2_app_hfp_hf中语音识别接口： bt_hfp_hf_voice_recog_send 
    - 语音cmd处理结果的event：BT_NOTIFY_HF_AT_CMD_CFM中cmd_id(HFP_HF_AT_BVRA) 
```c
// step1 : HF 主动唤醒语音识别调用 bt_interface_voice_recog(1)；
// step2 : HF 发出 AT+BVRA=1
// step3 ：AG 收到 AT+BVRA=1，处理结束之后发送ok/ERROR
// step4 ：HF 收到AG 回复，notify event：BT_NOTIFY_HF_AT_CMD_CFM中cmd_id(HFP_HF_AT_BVRA)
// 关闭流程类似开启流程
// 对端设备主动唤醒语音识别，HF收到之后notify event ：BT_NOTIFY_HF_VOICE_RECOG_STATUS_CHANGE
// 具体结构体解释请参考interface中相关说明
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
### 电话控制相关功能
#### 电话状态更新相关AT cmd
- AT cmd: 
    - AT+CIND: (Standard indicator update AT command)
    - 格式：
        - AT+CIND=？测试指令。HF获取AG侧支持的指示器索引值和范围。在发送关于指示器的相关指令（AT+CIND? 或 AT_CMER）前，至少请求一次。 
        - AT+CIND? 读指令，HF读取AG侧当前的指示器各个值
- AT cmd: 
    - +CIEV: unsolicited result code(Standard indicator events reporting unsolicited result code)
    - 格式：+CIEV: < ind >,< value >
    - 当AG测indicators发生变化时，AG需要主动用+ciev AT cmd 通知HF侧
- 手机端状态主动更新：
    - HF连接成功，同步手机电话状态event(call/callsetup/callheld):BT_NOTIFY_HF_CALL_STATUS_UPDATE
    - 连接过程中，主动获取电话状态：
        - bts2_app_interface中获取电话状态接口bt_interface_get_remote_call_status
        - 手机状态同步event：BT_NOTIFY_HF_CALL_STATUS_UPDATE
        - 手机状态cmd处理结果的event：BT_NOTIFY_HF_AT_CMD_CFM中cmd_id(HFP_HF_AT_CIND_STATUS)
    - 连接过程中，手机电话状态主动更新通过event：BT_NOTIFY_HF_INDICATOR_UPDATE
#### 电话控制相关AT cmd
- AT cmd: 
    - AT+CNUM: Subscriber Number Information
    - 格式：AT+CNUM
    - 该指令用于查询本机号码
    - +CNUM: Subscriber Number Information
    - 格式：+CNUM: ,< number >,< type >[,, < service >]
    - AG收到该请求之后，会将手机本机号码通过+CNUM回复给HF侧。
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
- AT cmd: 
    - AT+CLCC: Standard list current calls command
    - 格式：AT+CLCC
    - +CLCC：Standard list current calls command
    - 格式：+CLCC: < idx >,< dir >,< status >,< mode >,< mpty >,< number >,< type >
    - HF请求当前的电话信息列表，AG侧通过"+CLCC"回复当前的电话信息列表。如果当前没有电话，AG侧也需回复OK指令。
#### 电话控制功能使用
- 电话控制
    - 获取手机本机号码(AT+CNUM)：
        - bts2_app_interface获取本地电话号码接口：bt_interface_get_ph_num 
        - bts2_app_hfp_hf获取本地电话号码接口： bt_hfp_hf_at_cnum_send 
        - 本机号码信息event：BT_NOTIFY_HF_LOCAL_PHONE_NUMBER
        - 获取本机号码cmd处理结果的消息event：BT_NOTIFY_HF_AT_CMD_CFM中cmd_id(HFP_HF_AT_CNUM)
- 拨打电话
    - 回拨电话(AT+BLDN)
        - bts2_app_interface回拨电话接口：bt_interface_start_last_num_dial_req_send 
        - bts2_app_hfp_hf回拨电话接口： bt_hfp_hf_last_num_dial_send 
        - 回拨电话处理结果的消息结果event：BT_NOTIFY_HF_AT_CMD_CFM中cmd_id(HFP_HF_AT_BLDN)
- 通过电话号码拨打电话(ATD10086)
    - bts2_app_interface电话号码拨打电话接口：bt_interface_hf_out_going_call
    - bts2_app_hfp_hf电话号码拨打电话接口： bt_hfp_hf_make_call_by_number_send
    - 通过电话号码拨打电话处理结果的消息结果event：BT_NOTIFY_HF_AT_CMD_CFM中cmd_id(HFP_HF_AT_ATD)
- 获取手机端当前通话中电话的所有信息(AT+CLCC)
    - bts2_app_interface获取所有电话信息接口：bt_interface_get_remote_ph_num 
    - bts2_app_hfp_hf中获取所有电话信息接口： bt_hfp_hf_at_clcc_send
    - 电话的状态信息event：BT_NOTIFY_HF_REMOTE_CALL_INFO_IND进行通知
    - 获取当前通话中电话的信息cmd处理结果event：BT_NOTIFY_HF_AT_CMD_CFM中cmd_id(HFP_HF_AT_CLCC) 
```c
// step1： 当HF与AG连接成功之后，可以通过bt_interface_get_ph_num获取AG端电话号码
// step2： 当AG通过+CNUM将本机号码发送到HF，以及对应OK。
// step3:  HF会收到本地号码（BT_NOTIFY_HF_LOCAL_PHONE_NUMBER），以及BT_NOTIFY_HF_AT_CMD_CFM中cmd_id(HFP_HF_AT_CNUM)处理结果
// AG端不一定会有本机号码发送过来，但是一定会有BT_NOTIFY_HF_AT_CMD_CFM中cmd_id(HFP_HF_AT_CNUM) event
//step4： 拨打电话时，将号码长度和号码传入bt_interface_hf_out_going_call
//step5： AG收到电话号码请求之后，先将处理结果BT_NOTIFY_HF_AT_CMD_CFM中cmd_id(HFP_HF_AT_ATD)发送给HF
//step6： 如果拨打电话成功，HF侧会在这个event之后，收到BT_NOTIFY_HF_INDICATOR_UPDATE
//step7： 用户收到BT_NOTIFY_HF_INDICATOR_UPDATE之后，可以通过bt_interface_get_remote_ph_num获取当前电话的所有信息
//step8：AG侧通过"+CLCC"回复当前的电话信息列表。如果当前没有电话，AG侧也需回复OK指令。
//step9: HF收到信息之后，会通过event：BT_NOTIFY_HF_REMOTE_CALL_INFO_IND，用户可以处理该消息
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
注意：HF部分可扩展的接口在bts2_app_hfp_hf中，用户可以根据需求进行功能扩展，将接口按照需求在interface里进行封装。
另外每个cmd发送之后需等待cmd_cfm的消息，避免协议栈消息队列太多，触发保护机制。
:::

## HFP_HF通话功能使用demo
- 首先在工程初始化时，注册接收notify event 的处理函数
- 输入需要连接手机的mac地址，等待连接成功的消息
- 收到连接成功之后，输入拨打电话的号码以及长度
```c
//register notify event handle function start
// HF侧收到AG侧处理 at cmd id的处理结果：0（成功）1（失败） 2（超时，没有收到AG端回复）
static void bt_sifli_notify_hdl_at_cmd(uint8_t cmd_id, uint8_t res)
{
    switch (cmd_id)
    {
    //(语音识别cmd处理结果)
    case HFP_HF_AT_BVRA: 
    {
        break;
    }
    //(获取当前电话信息列表处理完成)
    case HFP_HF_AT_CLCC:
    {
        //step5.request phone call information complete then hangup call
        bt_interface_handup_call();
        break;
    }
    //(AG侧处理拨打电话请求结果)
    case HFP_HF_AT_ATD:
    {
        //step2.make a call request result
        break;
    }
    //(AG侧处理回拨电话请求结果)
    case HFP_HF_AT_BLDN:
    {
        break;
    }
    //(AG侧DTMF语音按键处理结果)
    case HFP_HF_AT_VTS:
    {
        break;
    }
    //(AG侧音量控制处理结果)
    case HFP_HF_AT_VGS:
    {
        break;
    }
    //(AG挂断电话处理结果)
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
    // notify AG是否支持语音识别功能
    case BT_NOTIFY_HF_VOICE_RECOG_CAP_UPDATE:
    {
        break;
    }
    // notify AG语音识别功能状态
    case BT_NOTIFY_HF_VOICE_RECOG_STATUS_CHANGE: 
    {
        break;
    }
    // notify 本机号码
    case BT_NOTIFY_HF_LOCAL_PHONE_NUMBER:
    {
        break;
    }
    // notify 当前电话所有信息
    case BT_NOTIFY_HF_REMOTE_CALL_INFO_IND:
    {
        bt_notify_clcc_ind_t *clcc_info = (bt_notify_clcc_ind_t *)data;
        //step4.recv phone call information
        break;
    }
    // 音量发生变化
    case BT_NOTIFY_HF_VOLUME_CHANGE:
    {
        break;
    }
    // 所有电话状态
    case BT_NOTIFY_HF_CALL_STATUS_UPDATE:
    {
        bt_notify_all_call_status *call_status = (bt_notify_all_call_status *)data;
        break;
    }
    // 电话状态发生变化
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