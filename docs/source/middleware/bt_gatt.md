# GATT over BR/EDR
GATT 是一个在蓝牙连接之上的发送和接收很短的数据段的通用规范，这些很短的数据段被称为属性（Attribute）。GATT over BR/EDR主要是将数据通过BR/EDR L2CAP传输。

本文档主要是基于Sifli SDK，介绍对GATT over BR/EDR的基本功能支持。涉及文件如下：
- bts2_app_interface 
- bts2_app_bt_gatt 
## GATT基本功能
- GATT over BR/EDR的目的主要是复用BLE的服务，所以GATT主要是将BLE uuid注册到BR/EDR的sdp中
    - bts2_app_interface注册接口：bt_interface_bt_gatt_reg
    - 注册成功之后的event：BT_NOTIFY_GATT_REGISTER_RESPONSE
    - bts2_app_interface注销接口：bt_interface_bt_gatt_unreg
    - 注销成功之后的event：BT_NOTIFY_GATT_UNREGISTER_RESPONSE
    - bts2_app_interface更改L2CAP MTU接口：bt_interface_bt_gatt_mtu_changed
    - MTU值更改之后的event：BT_NOTIFY_GATT_CHANGE_MTU_RESPONSE
```c
//register notify event handle function start
//
/*
typedef struct
{
    U16 gatt_start_handle;
    U16 gatt_end_handle;
    U8 att_uuid_len;
    U8 *att_uuid;
} br_att_sdp_data_t;
    for (int i = 0; i < 8; i++)
    {
        if (svc[i].state == 1)
        {
            if (svc[i].uuid_len == ATT_UUID_16_LEN)
            {
                br_att_sdp_data_t sdp_reg_info;
                sdp_reg_info.gatt_start_handle = svc[i].start_handle;
                sdp_reg_info.gatt_end_handle = svc[i].end_handle) ;
                sdp_reg_info.att_uuid_len = ATT_UUID_16_LEN;
                sdp_reg_info.att_uuid = svc[i].uuid;
                bt_interface_bt_gatt_reg(&sdp_reg_info);
            }
        }
    }
*/
// step1: 将BLE相关的GATT信息通过bt_interface_bt_gatt_reg注册到BR/EDR的SDP数据库中
// step2：等待手机/或者其他设备连接BR/EDR
int bt_sifli_notify_gatt_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    // GATT connected
    case BT_NOTIFY_GATT_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        break;
    }
    // GATT disconnected
    case BT_NOTIFY_GATT_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        break;
    }
    // GATT 注册成功之后，发挥SDP中的service handle值，用户可以保存该值
    // service record handle可以用于动态删除
    case BT_NOTIFY_GATT_REGISTER_RESPONSE:
    {
        bt_notify_gatt_sdp_info_t *sdp_info = (bt_notify_gatt_sdp_info_t *)data;
        break;
    }
    // 动态删除SDP GATT信息结果
    case BT_NOTIFY_GATT_UNREGISTER_RESPONSE:
    {
        bt_notify_gatt_sdp_info_t *sdp_info = (bt_notify_gatt_sdp_info_t *)data;
        break;
    }
    // 未连接状态下，更改GATT L2CAP MTU 值的回复。
    case BT_NOTIFY_GATT_CHANGE_MTU_RESPONSE:
    {
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
    case BT_NOTIFY_GATT:
    {
        bt_sifli_notify_gatt_event_hdl(event_id, data, data_len);
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
typedef struct
{
    U16 gatt_start_handle;
    U16 gatt_end_handle;
    U8 att_uuid_len;
    U8 *att_uuid;
} br_att_sdp_data_t;
```