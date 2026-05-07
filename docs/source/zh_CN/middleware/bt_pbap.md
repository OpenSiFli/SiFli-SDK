# PBAP
PBAP：Phone Book Access Profile的简称，电话本访问协议，是一种基于OBEX的上层协议，
该协议可以同步手机这些具有电话本功能设备上的通讯录和通话记录等联系人数据。
PBAP协议规定两种角色：
- PSE：Phone Book Server Equipment，拥有电话本源数据的设备，作为服务端，比如手机。 
- PCE：Phone Book Client Equipment，向PSE端请求电话本信息的设备，作为客户端，比如车载

PSE是手机等带有SIM卡的设备，存储源数据的联系人信息可能是手机也可能是SIM卡。PCE在同步获取相关数据时需要指明从哪个路径下获取对应的数据，即从手机同步还是从SIM卡中同步数据。
## phone book object
PBAP phone book 对象有7种类型：
1. pb(Phonebook): 主电话簿（所有本地联系人）
2. ich(Incoming Call History): 来电记录
3. och(Outgoing Call History): 去电记录
4. mch(Missed Call History): 未接来电记录
5. cch(Combined Call History): 所有通话记录(合并 ich+och+mch)
6. spd(Speed Dial): 速拨号码列表
7. fav(Favorites): 收藏/常用联系人

在PBAP协议中，电话薄对象有两种表现形式：
## File Representations:
文件表示分为两类：列表描述文件
- phone book object(pb):               pb.vcf
- Incoming Calls History object (ich): ich.vcf
- Outgoing Calls History object (och): och.vcf
- Missed Calls History object (mch):   mch.vcf
- Combined Calls History object (cch): cch.vcf
- Speed-Dial object (spd):             spd.vcf
- Favorite Contacts object (fav):      fav.vcf

虚拟vCard文件:
文件命名格式（Handle）:
- 00000001.vcf      (本地条目)
- 0000ABCD.vcf      (本地条目，十六进制)

## Folder Representations:
- 文件夹对应不同的电话簿存储位置和通话记录类型，使用树形结构
├── telecom/            (设备本地存储)
│   ├── pb              (主电话簿 - Phonebook)
│   ├── ich             (来电记录 - Incoming Calls)
│   ├── och             (去电记录 - Outgoing Calls)
│   ├── mch             (未接来电 - Missed Calls)
│   └── cch             (所有通话 - Combined Calls)
├── SIM1/               (第一张 SIM 卡)
│   ├── pb              (SIM 卡电话簿)
│   └── ...             (部分实现支持 ich/och 等)
├── SIM2/               (第二张 SIM 卡，双卡设备)
│   └── pb
└── (厂商自定义扩展)

联系人路径：
- 手机路径：telecom/
- SIM卡路径：SIM1/telecom/

## phone book entry format
1. Phone Book Object 中的每个独立条目（Individual Entry）均以vCard格式表示。
2. PSE必须同时支持 vCard 2.1 和 vCard 3.0.
3. PCE通过 Format 参数请求指定版本.
4. 默认编码:UTF-8(无论vCard版本，必须使用UTF-8编码).

## phone book entry 扩展属性
1. X-IRMC-CALL-DATETIME(ich/och/mch/cch)
    - vCard 2.1: X-IRMC-CALL-DATETIME;MISSED:20050320T100000
    - vCard 3.0：X-IRMC-CALL-DATETIME;TYPE=MISSED:20050320T100000
    - X-IRMC-CALL-DATETIME;MISSED:（即使没有时间信息，该属性也必须存在）
2. X-BT-SPEEDDIALKEY(spd/fav): Speed-dial shortcut - 用于标识联系人的快速拨号键
3. X-BT-UID(pb): 联系人的唯一标识符（128位）, 32字符十六进制(0-9, A-F（大写）) PBAP 联系人唯一标识
4. X-BT-UCI:{client}:{user_id}

## PBAP 功能
1. Call Histories
    - 同一号码在不同文件夹（ich/och/mch）中的 handle 相互独立（och/ich XX.vcf 不同）
    - 不同文件夹中的 handle 值可能相同，也可能不同
    - 与主电话簿（pb）的静态 handle 不同，通话记录使用动态handle

2. vCard-Listing Object (x-bt/vcard-listing)
    - 浏览电话簿时返回的 XML 格式数据结构。当 PCE（Phone Book Client，如车载系统）使用 PullvCardListing 功能从 PSE（Phone Book Server，如手机） 获取联系人列表时，返回的就是这种格式的数据
        ```xml
        <?xml version="1.0"?>
        <!DOCTYPE vcard-listing SYSTEM "vcard-listing.dtd">
        <vCard-listing version="1.0">
            <card handle="0.vcf" name="Bolt;Usain;;;"/>
            <card handle="1.vcf" name="Sanvi;Nithisha;;;"/>
        </vCard-listing>
        ```
3. 功能支持列表：
    - Download:双方都必须支持才可用
    - Browsing:双方都必须支持才可用
    - Database Identifier:PSE 生成，PCE 存储
    - Folder Version Counters:PSE 生成，PCE 存储
    - vCard Selecting:PSE 过滤，PCE 解析
    - Enhanced Missed Calls:PSE 通知，PCE 显示
    - X-BT-UCI vCard Property:vCard 属性支持
    - X-BT-UID vCard Property:vCard 属性支持
    - Referencing Contacts:跨协议引用
    - Contact Image Default Format:图片格式协商
4. Phone Book Download Feature
    - pullphonebook:PullPhonebook 是 PBAP 协议中用于下载完整电话簿的核心功能，基于 OBEX GET 操作实现。
5. Phone Book Browsing Feature
    - setphonebook:用于设置当前路径/浏览文件夹的核心操作
    - pullvcardlisting:用于获取联系人列表的核心功能，返回vCard-Listing Object（XML格式，包含文件夹中所有条目的句柄和名称信息
    - pullvcardentry:是 PBAP 协议中用于获取单个联系人完整 vCard 数据的核心操作，基于 OBEX GET 命令实现

## PBAP 参数说明
1. PullPhoneBook Function
    - Request
        - Opcode:GET (0x03 或 0x83)(M)
        - Packet Length(M)
        - Connection ID(OBEX 连接ID)(M)
        - Single Response Mode(取决obex版本)
        - Single Response Mode Param(取决obex版本)
        - Name（M）
        - Type: x-bt/phonebook
        - Application Parameters
            - PropertySelector
            - Format(vCard 2.1/vCard 3.0)
            - MaxListCount
            - ListStartOffset(列表起始偏移量)
            - ResetNewMissedCalls(重置未接来电计数)
            - vCardSelector(vCard选择器)
            - vCardSelectorOperator(vCard选择器操作符)
    - Response
        - Code:Response (0x90 (Continue)/0xA0 (Success) 或错误码)
        - Packet Length
        - Single Response Mode(取决obex版本)
        - Single Response Mode Param(取决obex版本)
        - Application Parameters
            - PhonebookSize
            - NewMissedCalls
            - PrimaryFolderVersion
            - SecondaryFolderVersion
            - DatabaseIdentifier
        - Body/End of Body(vCard对象数据)
2. SetPhoneBook Function
    - Opcode: SETPATH (0x05)(M)
    - Packet Length(M)
    - Connection ID
    - Flags
    - Name

3. PullvCardListing Function
    - Request
        - Opcode:GET (0x03 或 0x83)(M)
        - Packet Length(M)
        - Connection ID(OBEX 连接ID)(M)
        - Single Response Mode(取决obex版本)
        - Single Response Mode Param(取决obex版本)
        - Name（M）
        - Type: x-bt/vcard-listing
        - Application Parameters
            - Order
            - SearchValue
            - SearchProperty
            - MaxListCount
            - ListStartOffset(列表起始偏移量)
            - ResetNewMissedCalls(重置未接来电计数)
            - vCardSelector(vCard选择器)
            - vCardSelectorOperator(vCard选择器操作符)
    - Response
        - Code:Response (0x90 (Continue)/0xA0 (Success) 或错误码)
        - Packet Length
        - Single Response Mode(取决obex版本)
        - Single Response Mode Param(取决obex版本)
        - Application Parameters
            - PhonebookSize
            - NewMissedCalls
            - PrimaryFolderVersion
            - SecondaryFolderVersion
            - DatabaseIdentifier
        - Body/End of Body(vCard对象数据)
4. PullvCardEntry Function
    - Request
        - Opcode:GET (0x03 或 0x83)(M)
        - Packet Length(M)
        - Connection ID(OBEX 连接ID)(M)
        - Single Response Mode(取决obex版本)
        - Single Response Mode Param(取决obex版本)
        - Name（M）
        - Type: x-bt/vcard
        - Application Parameters
            - PropertySelector
            - Format
    - Response
        - Code:Response (0x90 (Continue)/0xA0 (Success) 或错误码)
        - Packet Length
        - Single Response Mode(取决obex版本)
        - Single Response Mode Param(取决obex版本)
        - Application Parameters
            - DatabaseIdentifier
        - Body/End of Body(vCard对象数据)
## PABP获取电话簿的完整步骤
1. 建立蓝牙连接并授权
    - API 调用: bt_pbap_client_connect_request()
    - 预期事件: BT_NOTIFY_PBAP_PROFILE_CONNECTED
    - 详细步骤:
        - 确保手机端蓝牙已开启并处于可发现状态
        - 发起 PBAP 客户端连接请求
        - 等待手机端弹出的授权对话框，必须在手机上点击"允许"或"授权"访问联系人
        - 监听连接成功事件，确认 BT_NOTIFY_PBAP_PROFILE_CONNECTED 回调触发
        - 注意事项: 如果手机拒绝授权，连接会失败或无法获取联系人数据；部分手机需要在蓝牙配对时勾选"共享联系人"选项
2. 设置电话簿路径
    - API 调用: bt_pbap_client_set_pb()
    - 预期事件: BT_NOTIFY_PBAP_SET_PATH_CFM（路径设置确认）
    - 详细步骤:
    - 指定目标路径，常用路径包括：
        - telecom/pb.vcf —— 主电话簿（SIM 卡 + 手机存储的联系人）(对应参数：PBAP_LOCAL, PBAP_PB)
        - sim1/telecom/pb.vcf —— SIM 卡 1 的电话簿(对应参数：PBAP_SIM1, PBAP_PB)
        - sim2/telecom/pb.vcf —— SIM 卡 2 的电话簿（双卡手机），暂时不支持
        - 发送设置路径请求
    - 等待 BT_NOTIFY_PBAP_SET_PATH_CFM 确认事件，验证路径是否设置成功
    - 注意事项: 不同手机品牌支持的路径可能略有差异；如果路径错误会收到失败回调
3. 获取电话薄
    - API 调用: bt_pbap_client_pull_pb()
    - 预期事件:
        - BT_NOTIFY_PBAP_PULL_PB_CMPL —— 整个电话簿拉取完成
        - BT_NOTIFY_PBAP_VCARD_ITEM_IND —— 单个 vCard 条目到达指示（可能多次触发）
    - 详细步骤:
        - 发起拉取请求，可附带过滤参数（如只获取姓名和电话号码，减少传输量）
        - 处理实时数据流: 对于每个联系人条目，会收到 BT_NOTIFY_PBAP_VCARD_ITEM_IND 事件，事件中包含单个 vCard 数据
        - 解析每个 vCard 条目（通常为 vCard 2.1 或 3.0 格式），提取：
        - FN 或 N 字段：联系人姓名
        - 等待 BT_NOTIFY_PBAP_PULL_PB_CMPL 事件，表示所有联系人已传输完毕
        - 注意事项: 联系人较多时，vCard 条目指示事件会分批触发

本文档主要是基于Sifli SDK，介绍对PBAP的PCE基本功能支持。涉及文件如下：
- bts2_app_interface
- bts2_app_pbap_c
## PABP初始化
- PBAP初始化的函数：bt_pbap_clt_init，PBAP相关的状态、标志赋初始值
```c
void bt_pbap_clt_init(bts2_app_stru *bts2_app_data)
{
    local_inst = (bts2s_pbap_clt_inst_data *)bmalloc(sizeof(bts2s_pbap_clt_inst_data));
    // Must allocate successful
    BT_ASSERT(local_inst);
    local_inst->pbap_clt_st = BT_PBAPC_IDLE_ST;
    local_inst->is_valid_vcard = FALSE;
    local_inst->elem_index = BT_PBAP_ELEM_VCARD_IDLE;
    local_inst->pbab_vcard = NULL;
    local_inst->mfs = pbap_clt_get_max_mtu();
    local_inst->rmt_supp_repos = 0;
    local_inst->curr_cmd = BT_PBAP_CLT_IDLE;

    local_inst->curr_repos = PBAP_LOCAL;
    local_inst->curr_phonebook = PBAP_PB;

    local_inst->target_repos = PBAP_UNKNOWN_REPO;
    local_inst->target_phonebook = PBAP_UNKNOWN_PHONEBOOK;

    local_inst->cur_file_hdl = NULL;
}
```
### PABP获取联系人名称的功能
由于手机的电话本太多同步到手表端数据太多，因此采用在通话过程中通过号码去手机端获取联系人的方式。 注：此功能需要在配对时在手机端给予相应的权限。
- PBAP连接设备接口：
    - bts2_app_interface连接接口：bt_interface_conn_ext
    - bts2_app_pbap_c连接接口：bt_pbap_clt_conn_to_srv
       
- PBAP连接断开设备接口：
    - bts2_app_interface断开连接接口：bt_interface_disc_ext 
    - bts2_app_pbap_c断开接口：bt_pbap_clt_disc_to_srv 

- PBAP选择设置获取联系人存储库（连接时已默认设置为手机存储联系人）：
    - bts2_app_pbap_c设置获取联系人存储库接口：bt_pbap_client_set_pb
    - PBAP通过号码获取联系人名称（有通话时已调用）：
    - bts2_app_pbap_c设置获取联系人存储库接口：bt_pbap_client_get_name_by_number 
    - 联系人名称event:BT_NOTIFY_PBAP_VCARD_LIST_ITEM_IND
    - 联系人名称获取结束event：BT_NOTIFY_PBAP_VCARD_LIST_CMPL
```c
//register notify event handle function start
// step1: 通过接口将PBAP 建立连接成功
// step2：设置获取联系人路径 bt_pbap_client_set_pb（手机路径：telecom/xxx.vcf/ SIM卡路径：SIM1/telecom/xxx.vcf）
// step3: bt_pbap_client_get_name_by_number 传入号码获取联系人名字
int bt_sifli_notify_pbap_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    // PBAP连接成功
    case BT_NOTIFY_PBAP_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        break;
    }
    // PBAP断开连接成功
    case BT_NOTIFY_PBAP_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        break;
    }
    // 联系人handle值（1.vcf） + 联系人名字信息
    case BT_NOTIFY_PBAP_VCARD_LIST_ITEM_IND:
    {
        pbap_vcard_listing_item_t *list_item = (pbap_vcard_listing_item_t *)data;
        break;
    }
    // 联系人名字获取完毕
    case BT_NOTIFY_PBAP_VCARD_LIST_CMPL:
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

    case BT_NOTIFY_PBAP:
    {
        bt_sifli_notify_pbap_event_hdl(event_id, data, data_len);
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
    return 0;
}

INIT_ENV_EXPORT(app_bt_notify_init);
//register notify event handle function end
```