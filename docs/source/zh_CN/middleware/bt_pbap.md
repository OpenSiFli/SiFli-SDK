# PBAP
PBAP：Phone Book Access Profile的简称，电话本访问协议，是一种基于OBEX的上层协议，
该协议可以同步手机这些具有电话本功能设备上的通讯录和通话记录等信息。
PBAP协议规定两种角色：
- PSE：Phone Book Server Equipment，拥有电话本源数据的设备，作为服务端，比如手机。 
- PCE：Phone Book Client Equipment，向PSE端请求电话本信息的设备，作为客户端，比如车载

PSE是手机等带有SIM卡的设备，存储源数据的联系人信息可能是手机也可能是SIM卡。PCE在同步获取相关数据时需要指明从哪个路径下获取对应的数据，即从手机同步还是从SIM卡中同步数据。
- phone book object(pb):               pb.vcf
- Incoming Calls History object (ich): ich.vcf
- Outgoing Calls History object (och): och.vcf
- Missed Calls History object (mch):   mch.vcf
- Combined Calls History object (cch): cch.vcf
- Speed-Dial object (spd):             spd.vcf
- Favorite Contacts object (fav):      fav.vcf  

联系人路径：
- 手机路径：telecom/xxx.vcf
- SIM卡路径：SIM1/telecom/xxx.vcf

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