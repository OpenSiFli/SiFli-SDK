# GATT over BR/EDR
GATT is a general specification for sending and receiving short data segments over Bluetooth connections. These short data segments are called attributes. GATT over BR/EDR mainly transmits data through BR/EDR L2CAP.

This document is mainly based on Sifli SDK, introducing basic functionality support for GATT over BR/EDR. Related files include:
- bts2_app_interface 
- bts2_app_bt_gatt 

## GATT Basic Functionality
- The main purpose of GATT over BR/EDR is to reuse BLE services, so GATT mainly registers BLE UUIDs to BR/EDR SDP
    - bts2_app_interface registration interface: bt_interface_bt_gatt_reg
    - Event after successful registration: BT_NOTIFY_GATT_REGISTER_RESPONSE
    - bts2_app_interface unregistration interface: bt_interface_bt_gatt_unreg
    - Event after successful unregistration: BT_NOTIFY_GATT_UNREGISTER_RESPONSE
    - bts2_app_interface L2CAP MTU change interface: bt_interface_bt_gatt_mtu_changed
    - Event after MTU value change: BT_NOTIFY_GATT_CHANGE_MTU_RESPONSE
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
// step1: Register BLE-related GATT information to BR/EDR SDP database through bt_interface_bt_gatt_reg
// step2: Wait for phone/or other devices to connect BR/EDR
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
    // After GATT registration success, returns service handle value in SDP, users can save this value
    // service record handle can be used for dynamic deletion
    case BT_NOTIFY_GATT_REGISTER_RESPONSE:
    {
        bt_notify_gatt_sdp_info_t *sdp_info = (bt_notify_gatt_sdp_info_t *)data;
        break;
    }
    // Dynamic deletion of SDP GATT information result
    case BT_NOTIFY_GATT_UNREGISTER_RESPONSE:
    {
        bt_notify_gatt_sdp_info_t *sdp_info = (bt_notify_gatt_sdp_info_t *)data;
        break;
    }
    // Reply to changing GATT L2CAP MTU value in unconnected state.
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