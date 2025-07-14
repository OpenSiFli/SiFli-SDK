# SPP
SPP: Serial Port Profile（串口协议，允许设备之间通过模拟串口的方式进行无线数据传输。基于 RFCOMM 通信层，SPP 协议与传统的 RS-232 串口标准类似，因此非常适合低速、短距离的数据传输，如 Android 设备和传感器、微控制器之间的通信。SPP协议规定两种角色：
- Device A (DevA) – This is the device that takes initiative to form a connection to another device.(发起连接的设备)
    - 使用SDP发起请求，查询DevB的RFCOMM channel通道
    - 能与对端设备进行安全认证
    - 能通过查询到的RFCOMM channel与对端设备建立L2CAP(RFCOMM) RFCOMM（DLC）通道
    - 收发数据
    - 断开连接
- Device B (DevB) – This is the device that waits for another device to take initiative to connect. (等待连接的设备)
    - 注册SPP相关的UUID到SDP database里，DevA能够通过SDP查询到
    - 能与对端设备进行安全认证
    - 接收对端设备的连接请求
    - 收发数据
    - 断开连接   

本文档主要是基于Sifli SDK，介绍对SPP的DevB基本功能支持。涉及文件如下：
- bts2_app_interface
- bts2_app_spp_s
## SPP初始化
- SPP初始化的函数：bt_spp_srv_init，SPP相关的状态、标志赋初始值,以及SPP uuid注册
```c
//step1: 用户可以通过重写bt_spp_srv_add_uuid_list函数将自定义的SPP UUID注册到BR/EDR 的SDP database中
//step2：启用SPP profile时，SPP会将数据注册到SDP中
//注意： 目前最多支持自定义UUID数量为7个
void bt_spp_srv_add_uuid_list(void)
{
    U8 uuid_128[] =
    {
        0x30, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
    };

    U8 uuid_32[] =
    {
        0x30, 0x01, 0x02, 0x03
    };

    U8 uuid_16[] =
    {
        0x30, 0x01
    };

    spp_add_uuid_list_node(uuid_128, sizeof(uuid_128), "aaaa");
    spp_add_uuid_list_node(uuid_32,  sizeof(uuid_32), "bbbb");
    spp_add_uuid_list_node(uuid_16,  sizeof(uuid_16), "cccc");
}

void bt_spp_srv_init(bts2_app_stru *bts2_app_data)
{
    U8 i = 0;

    bts2_app_data->select_device_id = 0;
    bts2_app_data->select_srv_chnl = 0;
    bts2_app_data->spp_srv_conn_nums = 0;
    bts2_app_data->spp_srv_inst_ptr = &bts2_app_data->spp_srv_inst[0];

    for (i = 0; i < CFG_MAX_ACL_CONN_NUM; i++)
    {
        bts2_app_data->spp_srv_inst[i].device_id = 0xff;
        bd_set_empty(&bts2_app_data->spp_srv_inst[i].bd_addr);
        bts2_app_data->spp_srv_inst[i].cur_link_mode = 0;
        bts2_app_data->spp_srv_inst[i].exit_sniff_pending = FALSE;
        bts2_app_data->spp_srv_inst[i].cod = 0;
        bts2_app_data->spp_srv_inst[i].service_list = 0;
        bts2_app_data->spp_srv_inst[i].spp_service_list = NULL;
    }
    bt_spp_srv_add_uuid_list();
}

void bt_spp_srv_start_enb(bts2_app_stru *bts2_app_data)
{
    U8 i;

    for (i = 0; i < SPP_SRV_MAX_CONN_NUM; i++)
    {
        spp_srv_enb_req(i, SPP_DEFAULT_NAME);
    }
    USER_TRACE(">> SPP enabled\n");
}
```
## SPP数据收发功能
SPP作为DevB功能，只能接收对方发起的连接请求。
- SPP断开设备接口为：
    - bts2_app_interface断开连接接口：bt_interface_dis_spp_by_addr_and_chl
    - bts2_app_spp_s断开接口：bt_spp_srv_disc_req
        
- SPP连接状态回调event:
    - SPP连接成功：BT_NOTIFY_SPP_PROFILE_CONNECTED
    - SPP连接失败：BT_NOTIFY_SPP_PROFILE_DISCONNECTED
    - SPP收到连接事件：BT_NOTIFY_SPP_CONN_IND
    - SPP收到连接断开事件：BT_NOTIFY_SPP_DISC_IND

- SPP数据收发相关的接口和事件(event):
    - 数据发送：
        - bts2_app_interface数据发送接口：bt_interface_spp_send_data
        - bts2_app_spp_s数据发送接口：bt_spp_srv_sending_data_by_device_id_and_srv_chnl
        - 数据发送成功event：BT_NOTIFY_SPP_DATA_CFM
    - 数据接收：
        - bts2_app_interface数据接收成功回复接口：bt_interface_spp_srv_data_rsp
        - bts2_app_spp_s数据接收成功回复接口：spp_srv_data_rsp_ext
        - 收到数据EVENT: BT_NOTIFY_SPP_DATA_IND
```c
// step1: 接收对方设备连接时，如果只连接单个channel时，可以处理：BT_NOTIFY_SPP_PROFILE_CONNECTED 和 BT_NOTIFY_SPP_PROFILE_DISCONNECTED
// step2：接收对方多个channel连接时，需要处理的连接相关的event：BT_NOTIFY_SPP_CONN_IND 和 BT_NOTIFY_SPP_DISC_IND
//step1 和 step2 中两者的区别在于：step1 中的event只有对端设备的addr 和连接原因，step2中的event信息包含连接的UUID 和 service channel
// step3: 收到数据之后，需要回复bt_interface_spp_srv_data_rsp
// step4: 发送数据调用接口：bt_interface_spp_send_data
// step5: 数据发送成功之后会收到event：BT_NOTIFY_SPP_DATA_CFM
int bt_sifli_notify_spp_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    /*
    typedef struct
    {
        bt_notify_device_mac_t mac;                  /// the bt device mac
        uint8_t profile_type;                        /// BT_NOTIFY type id
        uint8_t res;                                /// error code 为自定义类型
    } bt_notify_profile_state_info_t;
    */
    case BT_NOTIFY_SPP_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        break;
    }
    /*
    typedef struct
    {
        bt_notify_device_mac_t mac;                  /// the bt device mac
        uint8_t profile_type;                        /// BT_NOTIFY type id
        uint8_t res;                                /// error code 为自定义类型
    } bt_notify_profile_state_info_t;
    */
    case BT_NOTIFY_SPP_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        break;
    }
    /*
        ///  spp connection information
        typedef struct
        {
            ///  the address of the connected device
            bt_notify_device_mac_t mac;
            ///  the service channel of the connected device
            uint8_t  srv_chl;
            ///  uuid value
            SPP_UUID uuid;
            ///  uuid length
            uint8_t uuid_len;
            ///  the mtu size of the connected device
            uint16_t  mfs;
        } bt_notify_spp_conn_ind_t;
    */
    case BT_NOTIFY_SPP_CONN_IND:
    {
        bt_notify_spp_conn_ind_t *conn_ind = (bt_notify_spp_conn_ind_t *)data;
        break;
    }
    // 收到的每个包都需要回复：bt_interface_spp_srv_data_rsp
    // 回复之后才能收到下一包数据，以及对方设备更新credit发数据。
    // 回复速度可以决定对方设备发包的速率
    case BT_NOTIFY_SPP_DATA_IND:
    {
        bt_notify_spp_data_ind_t *data_info = (bt_notify_spp_data_ind_t *)data;
        BTS2S_BD_ADDR bd_addr;
        bt_addr_convert_to_bts((bd_addr_t *)data_info->mac.addr, &bd_addr);
        bt_interface_spp_srv_data_rsp(&bd_addr, data_info->srv_chl);
        break;
    }
    //  bt_interface_spp_send_data(data, len, bd_addr, srv_chl);
    // send data cfm event
    case BT_NOTIFY_SPP_DATA_CFM:
    {
        bt_notify_spp_data_cfm_t *data_cfm = (bt_notify_spp_data_cfm_t *)data;
        // Send another packet of data （bt_interface_spp_send_data）
        // 发送数据时，除了数据本身之外，还需要带地址以及对应的srv_chl通道，这些信息包含在event：BT_NOTIFY_SPP_CONN_IND中
        // 地址需要转换，eg:bt_addr_convert_to_bts((bd_addr_t *)data_info->mac.addr, &bd_addr);
        // bt_interface_spp_send_data(data, len, bd_addr, srv_chl); 
        break;
    }
    /*
        ///  spp disconnection information
        typedef struct
        {
            ///  the address of the disconnected device
            bt_notify_device_mac_t mac;
            ///  the service channel of the disconnected device
            uint8_t  srv_chl;
            ///  the uuid of the disconnected device
            SPP_UUID uuid;
            ///  the uuid length of spp connection that sent the data
            uint8_t uuid_len;
        } bt_notify_spp_disc_ind_t;
    */
    case BT_NOTIFY_SPP_DISC_IND:
    {
        bt_notify_spp_disc_ind_t *disc_ind = (bt_notify_spp_disc_ind_t *)data;
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

    case BT_NOTIFY_SPP:
    {
        bt_sifli_notify_spp_event_hdl(event_id, data, data_len);
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