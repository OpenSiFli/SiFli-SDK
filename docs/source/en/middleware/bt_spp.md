
# SPP (Serial Port Profile)

SPP stands for Serial Port Profile, which allows devices to wirelessly transfer data by simulating a serial port connection. Based on the RFCOMM communication layer, the SPP protocol is similar to the traditional RS-232 serial port standard, making it ideal for low-speed, short-range data transmission, such as communication between Android devices, sensors, and microcontrollers.

SPP defines two roles:

- **Device A (DevA)** – The device that initiates the connection to another device.
    - Initiates requests through SDP to query DevB's RFCOMM channel.
    - Can perform security authentication with the other device.
    - Can establish an L2CAP (RFCOMM) RFCOMM (DLC) channel with the other device through the queried RFCOMM channel.
    - Can send and receive data.
    - Can disconnect the connection.

- **Device B (DevB)** – The device that waits for another device to initiate the connection.
    - Registers the SPP-related UUID in the SDP database so DevA can query it.
    - Can perform security authentication with the other device.
    - Receives the connection request from the other device.
    - Can send and receive data.
    - Can disconnect the connection.

This document mainly describes the basic functionality support for DevB in SPP based on the Sifli SDK. The relevant files are:
- `bts2_app_interface`
- `bts2_app_spp_s`

## SPP Initialization

- The SPP initialization function: `bt_spp_srv_init`, which initializes SPP-related states, flags, and registers the SPP UUID.

```c
//step1: Users can override the bt_spp_srv_add_uuid_list function to register custom SPP UUIDs into the BR/EDR SDP database
//step2: When enabling the SPP profile, the SPP will register data into the SDP
//Note: Currently, a maximum of 7 custom UUIDs are supported
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
    USER_TRACE(">> SPP enabled
");
}
```

## SPP Data Transmission and Reception

SPP, as a DevB feature, can only receive connection requests initiated by the other device.

- **SPP disconnect device interface:**
    - `bts2_app_interface` disconnect interface: `bt_interface_dis_spp_by_addr_and_chl`
    - `bts2_app_spp_s` disconnect interface: `bt_spp_srv_disc_req`

- **SPP connection status callback events:**
    - SPP connection successful: `BT_NOTIFY_SPP_PROFILE_CONNECTED`
    - SPP connection failed: `BT_NOTIFY_SPP_PROFILE_DISCONNECTED`
    - SPP connection received: `BT_NOTIFY_SPP_CONN_IND`
    - SPP connection disconnected: `BT_NOTIFY_SPP_DISC_IND`

- **SPP data transmission and reception interfaces and events:**
    - **Data sending:**
        - `bts2_app_interface` data send interface: `bt_interface_spp_send_data`
        - `bts2_app_spp_s` data send interface: `bt_spp_srv_sending_data_by_device_id_and_srv_chnl`
        - Data send success event: `BT_NOTIFY_SPP_DATA_CFM`
    - **Data reception:**
        - `bts2_app_interface` data reception success response interface: `bt_interface_spp_srv_data_rsp`
        - `bts2_app_spp_s` data reception success response interface: `spp_srv_data_rsp_ext`
        - Data received event: `BT_NOTIFY_SPP_DATA_IND`

```c
// step1: When receiving a connection from the other device, if only a single channel is connected, handle the events: BT_NOTIFY_SPP_PROFILE_CONNECTED and BT_NOTIFY_SPP_PROFILE_DISCONNECTED
// step2: When multiple channels are connected, handle the related connection events: BT_NOTIFY_SPP_CONN_IND and BT_NOTIFY_SPP_DISC_IND
// The difference between step1 and step2: In step1, the event only contains the address and connection reason of the other device. In step2, the event contains the UUID and service channel information.
// step3: After receiving data, you need to reply using bt_interface_spp_srv_data_rsp
// step4: Send data using the interface: bt_interface_spp_send_data
// step5: After data is successfully sent, the event BT_NOTIFY_SPP_DATA_CFM will be received
int bt_sifli_notify_spp_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_SPP_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        break;
    }

    case BT_NOTIFY_SPP_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        break;
    }

    case BT_NOTIFY_SPP_CONN_IND:
    {
        bt_notify_spp_conn_ind_t *conn_ind = (bt_notify_spp_conn_ind_t *)data;
        break;
    }

    case BT_NOTIFY_SPP_DATA_IND:
    {
        bt_notify_spp_data_ind_t *data_info = (bt_notify_spp_data_ind_t *)data;
        BTS2S_BD_ADDR bd_addr;
        bt_addr_convert_to_bts((bd_addr_t *)data_info->mac.addr, &bd_addr);
        bt_interface_spp_srv_data_rsp(&bd_addr, data_info->srv_chl);
        break;
    }

    case BT_NOTIFY_SPP_DATA_CFM:
    {
        bt_notify_spp_data_cfm_t *data_cfm = (bt_notify_spp_data_cfm_t *)data;
        break;
    }

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
    case BT_NOTIFY_COMMON_BT_STACK_READY:
    {
        break;
    }

    case BT_NOTIFY_COMMON_CLOSE_COMPLETE:
    {
        break;
    }

    case BT_NOTIFY_COMMON_ACL_CONNECTED:
    {
        bt_notify_device_acl_conn_info_t *acl_info = (bt_notify_device_acl_conn_info_t *)data;
        break;
    }

    case BT_NOTIFY_COMMON_ACL_DISCONNECTED:
    {
        bt_notify_device_base_info_t *device_info = (bt_notify_device_base_info_t *)data;
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
// Register notify event handle function end
```

This document provides an overview of SPP functionality, implementation details, and the necessary SDK files.
