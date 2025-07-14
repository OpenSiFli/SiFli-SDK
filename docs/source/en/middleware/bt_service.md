# Classic Bluetooth Service

## Overview

Currently, Bluetooth is mainly divided into Classic Bluetooth BR (Basic Rate)/EDR (Enhanced Data Rate) and Low Energy (BLE) Bluetooth. Sifli Bluetooth is based on Bluetooth 5.3 and provides a comprehensive dual-mode Bluetooth solution. Sifli Bluetooth focuses on supporting Bluetooth audio and Bluetooth voice functions, providing convenient related development methods. Sifli Bluetooth has good readability, scalability, and portability. Sifli Bluetooth is module-oriented, providing rich interfaces with clear hierarchy and logic between modules, simple and easy to understand, providing a good foundation for users to learn and develop. The overall design goal of Sifli Bluetooth is to enable users to form high-quality products in a shorter time through configuration or secondary development of Sifli Bluetooth. This section mainly introduces Sifli's architecture and related specification configuration information for classic Bluetooth services.

To transmit data via Bluetooth on Bluetooth-enabled devices, Sifli provides basic Bluetooth functionality APIs and common Bluetooth protocol specifications (A2DP/AVRCP/HFP/PBAP/HID/SPP, etc.). Users can perform the following operations through APIs:
- Scan other Bluetooth devices
- Manage connected devices
- Implement call and audio functions wirelessly with other Bluetooth devices (such as phones, headphones, etc.)
- Transfer custom data with other devices

:::{note}
For more information about APIs, please refer to: [bluetooth_service](middleware-bluetooth_service)
:::

### Classic Bluetooth Architecture
The main purpose of Sifli Classic Bluetooth architecture is to build a convenient, stable, comfortable, and efficient solution for users, structurally encapsulating functions and logic that users care about, and minimizing users' secondary development workload as much as possible. Therefore, the design of Sifli Classic Bluetooth architecture always revolves around the hierarchical structure and functional modularization.
- From the overall Bluetooth architecture perspective, Bluetooth is divided into two major parts: controller and host.
    - The controller includes PHY, Baseband, Link Controller, Link Manager, Device Manager, HCI and other modules, used for hardware interface management, link management, etc.;
    - The host includes L2CAP, RFCOMM, SDP, GAP and various specifications, building the foundation for providing interfaces to the application layer, facilitating user access to the Bluetooth system.
- Sifli's controller runs on LCPU, while the host part and APIs that provide users access to Bluetooth-related functions run on HCPU.
- Sifli's host part is divided into three categories: protocol stack, service, and interface.
    - The protocol stack mainly implements state machines and message processing related to Bluetooth protocols and specifications, which is the core of Bluetooth-related specifications, providing standardized interfaces for Bluetooth communication establishment and data interaction. Provided to users in lib format;
    - Service mainly provides modules for processing specific Bluetooth function service-related information, designed for user-specific functions, released to users in source code format;
    - Interface mainly provides specific functions for users to access the Bluetooth system, which are simple and efficient general-purpose interfaces, released to users in source code format. Users can use or customize related functions according to their needs.
![Figure 1: Sifli Bluetooth Architecture](../../assets/bluetooth_arch.png)

### Bluetooth Specification Configuration Information
Currently, the classic Bluetooth specifications and protocols supported by the host protocol stack are as follows:
- Specifications: A2DP, AVRCP, GATT over BR, HFP, HID, PAN, PBAP, SPP
- Protocols: L2CAP, SDP, AVDTP, AVCTP, RFCOMM

![Figure 1: Bluetooth Specifications Supported by Sifli Bluetooth](../../assets/bluetooth_profiles.png)

## Bluetooth Service Configuration and Development Instructions
### Sifli SDK Project Introduction
![Sifli Project Files](../../assets/SDK_ARCH.png)
- customer: Mainly contains board-level related files and peripheral drivers
- drivers: Mainly contains HAL drivers for chip-related peripherals
- dvt: Mainly contains chip ATE-related test projects
- example: Mainly contains project examples
- external: Mainly contains third-party open source project code
- middleware: Mainly contains middleware-related code
- rtos: Mainly contains rtthread OS-related code

### Sifli Bluetooth Project Compilation Basic Process
1. Project environment: Run set_env.bat to build the compilation environment
2. Enter directory: siflisdk\example\bt\test_example\project\common
3. Compile the corresponding BT project in the above path, and the generated files are also in the common path
    - Compilation result build_<board_name>_hcpu directory description:
        - board: Mainly contains board-level related executable files
        - bootloader: Mainly contains bootloader-related executable files
        - ftab: Mainly contains flash table-related executable files
        - lcpu: Mainly contains lcpu-related executable files
        - peripheral: Mainly contains peripheral driver-related executable files
        - sifli_sdk: Mainly contains SDK middleware-related executable files
4. Flash files: Run script uart_download.bat
![Sifli Classic Bluetooth Example](../../assets/BT_PROJECT.png)
![Sifli Classic Bluetooth Compilation](../../assets/complie_result.png)

### Sifli SDK Bluetooth-Related File Description
- Sifli SDK Bluetooth-related source code requires entering folder: siflisdk\middleware\bluetooth\service\bt
    - bt_cm file contains Bluetooth connection_manager related content
    - bt_finsh file contains Bluetooth service and configuration-related content, specific description as shown in the figure below:
    ![Sifli SDK Bluetooth-Related Source Code Description](../../assets/service_profiles.png)

#### Sifli_SDK Bluetooth Initialization and Profile Configuration Brief Description
1. Sifli SDK runs on the rt-thread system. In addition to the tasks that come with rt-thread, Sifli implements a special task mechanism specifically for handling Bluetooth-related events according to Bluetooth characteristics, running on bts_task. Bluetooth custom tasks have special sub-task_ids, and Bluetooth service-related functions all run on sub-task_id: bts2_task_get_app_task_id()
```c
void bts2_main(void)
{
    BOOL r = TRUE;
    if (g_is_created)
        return;

    g_is_created = 1;
    /*BTS2 key functions*/
    bts2_app_reg_cbk((void *)app_fn_init, (void *)app_fn_rel, (void *)app_fn_hdl_ext);
    bts2_init((U8 *)"bts2_sm.db");
    bts_run_init();
}
```
2. bts2_main registers the initialization function, release function, and event handling function of sub-task_id: bts2_task_get_app_task_id() by calling bts2_app_reg_cbk
3. When sub-task related content is initialized, the app_fn_init function is executed in sequence
4. Sub-task related events are distributed and processed through app_fn_hdl_ext
```c
void app_fn_init(void **pp)
{
    bts2_app_stru *bts2_app_data;

    /*app */
    *pp = (void *)bmalloc(sizeof(bts2_app_stru));
    RT_ASSERT(*pp);
    memset(*pp, 0, sizeof(bts2_app_stru));
    bts2_app_data = (bts2_app_stru *)*pp;
    bts2g_app_p = bts2_app_data;

    bts2_app_data->menu_id = menu_start;
    bt_disply_menu(bts2_app_data);
    bts2_app_data->menu_id = menu_main;
    bt_disply_menu(bts2_app_data);

    bts2_app_data->input_str_len = 0;
    bts2_app_data->pin_code_len = strlen(CFG_PIN_CODE);
    strcpy((char *)bts2_app_data->pin_code, CFG_PIN_CODE);
    init_inst_data(bts2_app_data);

    init_bt_cm();


    gap_rd_local_bd_req(bts2_app_data->phdl);
    /*Send read local BD request,if receive confirm message,then BTS2 initialization is OK*/
    gap_app_init_req(bts2_app_data->phdl);


    //bts2_timer_ev_add(KEYB_CHK_TIMEOUT, key_msg_svc, 0, NULL);
}

void app_fn_hdl_ext(void **pp)
{
    U16 msg_cls;
    void *msg;

    bts2_msg_get(bts2_task_get_app_task_id(), &msg_cls, &msg);
    bt_event_publish(msg_cls, *((U16 *)msg), msg);
    bfree(msg);
}

void app_fn_rel(void **pp)
{
    U16 msg_type;
    void *msg_data = NULL;
    U8  i;
    bts2_app_stru *bts2_app_data;

    bts2_app_data = (bts2_app_stru *)(*pp);
    if (!bts2_app_data)
    {
        DBG_OUT()
        return;
    }

    /*get a msg from the demo application task*/
    while (bts2_msg_get(bts2_task_get_app_task_id(), &msg_type, &msg_data))
    {
        switch (msg_type)
        {
        case BTS2M_SPP_CLT:
        {
            U16 *msg_type;

            msg_type = (U16 *)msg_data;
            switch (*msg_type)
            {

            case BTS2MU_SPP_CLT_DATA_IND:
            {
                BTS2S_SPP_CLT_DATA_IND *msg;

                msg = (BTS2S_SPP_CLT_DATA_IND *)msg_data;
                bfree(msg->payload);
                break;
            }
            }
            bfree(msg_data);

            /*SPP */
            for (i = 0; i < SPP_CLT_MAX_CONN_NUM; i++)
            {
                if (bts2_app_data->spp_inst[i].timer_flag)
                {
                    bts2_timer_ev_cancel(bts2_app_data->spp_inst[i].time_id, NULL, NULL);
                }

                if (bts2_app_data->spp_inst[i].cur_file_hdl != NULL)
                {
                    fclose(bts2_app_data->spp_inst[i].cur_file_hdl);
                }

                if (bts2_app_data->spp_inst[i].wr_file_hdl != NULL)
                {
                    fclose(bts2_app_data->spp_inst[i].wr_file_hdl);
                }
            }
            break;
        }

        case BTS2M_SPP_SRV:
        {
            U16 *msg_type;

            msg_type = (U16 *)msg_data;
            switch (*msg_type)
            {

            case BTS2MU_SPP_SRV_DATA_IND:
            {
                BTS2S_SPP_SRV_DATA_IND *msg;

                msg = (BTS2S_SPP_SRV_DATA_IND *)msg_data;
                bfree(msg->payload);
                break;
            }

            }
            bfree(msg_data);

            /*SPP */
            for (i = 0; i < CFG_MAX_ACL_CONN_NUM; i++)
            {
                bts2_spp_srv_inst_data *bts2_spp_srv_inst = NULL;
                bts2_spp_service_list *spp_service_list = NULL;

                bts2_spp_srv_inst = &bts2_app_data->spp_srv_inst[i];

                if (bts2_spp_srv_inst->service_list != 0)
                {
                    bts2_spp_service_list *spp_service_list = bts2_spp_srv_inst->spp_service_list;
                    while (spp_service_list)
                    {
                        if (spp_service_list->timer_flag)
                        {
                            bts2_timer_ev_cancel(spp_service_list->time_id, NULL, NULL);
                        }

                        if (spp_service_list->cur_file_hdl != NULL)
                        {
                            fclose(spp_service_list->cur_file_hdl);
                        }

                        if (spp_service_list->wr_file_hdl != NULL)
                        {
                            fclose(spp_service_list->wr_file_hdl);
                        }
                        spp_service_list = (bts2_spp_service_list *)spp_service_list->next_struct;
                    }
                }
            }
            break;
        }


        case  BTS2M_AV:
        {
            bt_av_rel();
            break;
        }
        case BTS2M_PBAP_CLT:
        {
            bt_pbap_clt_rel(bts2_app_data, msg_data);
            break;
        }
        }
    }
    bt_pbap_clt_free_inst();
    bfree(bts2_app_data);
}

```
5. When sub-task: bts2_task_get_app_task_id() initialization is completed and receives BTS2MU_GAP_APP_INIT_CFM, Bluetooth profile function initialization begins.
```c
    case BTS2MU_GAP_APP_INIT_CFM:
    {
        if (bts2_app_data->state == BTS_APP_IDLE)
        {
            bt_init_profile(bts2_app_data);

            bt_init_local_ctrller(bts2_app_data);

            bts2_app_data->state = BTS_APP_READY;
        }
        break;
```

#### Bluetooth Basic Information Modification Interface
1. Change Bluetooth device COD (class of device): Re-implement the bt_get_class_of_device function
```c
__WEAK uint32_t bt_get_class_of_device(void)
{
    return 0x240704;
}
```
2. Change Bluetooth device name: Call interface void bt_interface_set_local_name(int len, void *data);
3. Bluetooth profile configuration: bt_init_profile
```c
void bt_init_profile(bts2_app_stru *bts2_app_data)
{
    /*SPP */
    bt_spp_clt_init(bts2_app_data);

    /*HFP_HF */
    bt_hfp_hf_init(bts2_app_data);
    bt_hfp_hf_start_enb(bts2_app_data);

    /*HFP_AG */
    bt_hfp_ag_app_init(bts2_app_data);
    bt_hfp_start_profile_service(bts2_app_data);

    /*PAN */
    bt_pan_init(bts2_app_data);
    bt_pan_reg(bts2_app_data);

    /*A2DP SNK */
    bt_av_init(bts2_app_data);

    /*AVRCP */
    bt_avrcp_int(bts2_app_data);

    /*HID */
    bt_hid_init(bts2_app_data);

    /*PBAP_CLT */
    bt_pbap_clt_init(bts2_app_data);

    /*SPP_SRV */
    bt_spp_srv_init(bts2_app_data);
    bt_spp_srv_start_enb(bts2_app_data);
}
```

### Bluetooth Service Interface Invocation and Callback Usage Methods and Precautions
- Bluetooth interface integrates BT discovery, connection, and reconnection functions. Related function interfaces are mainly in: bts2_app_interface
- Bluetooth service callbacks include data transmitted by remote Bluetooth devices, results of calling Bluetooth service interface processing, and active updates of Bluetooth status information, etc.
    - Sifli Bluetooth service callbacks require users to implement callback handling functions themselves and transfer data to user's own task to process related data
    - Registration function: bt_interface_register_bt_event_notify_callback
    - Unregistration function: bt_interface_unregister_bt_event_notify_callback
:::{note}
The data processing task for callback functions is a user-defined implemented task.
:::

```c
int bt_notify_event_hdl(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
     return 0;
}
```

#### Brief Description of Important Bluetooth Events
Messages received by bt_notify_event_hdl include:
- ACL connection successful message: BT_NOTIFY_COMMON_ACL_CONNECTED
- ACL disconnection successful message: BT_NOTIFY_COMMON_ACL_DISCONNECTED
- Profile connection successful message: BT_NOTIFY_(profile_type)_PROFILE_CONNECTED
- Profile disconnection successful message: BT_NOTIFY_(profile_type)_PROFILE_DISCONNECTED
- SCO connection successful message: BT_NOTIFY_COMMON_SCO_CONNECTED
- SCO disconnection successful message: BT_NOTIFY_COMMON_SCO_DISCONNECTED
- HFP_HF sending AT cmd peer device processing result message: BT_NOTIFY_HF_AT_CMD_CFM
```c
// ACL start
typedef struct
{
    bt_notify_device_mac_t mac;                  /// the remote device mac
    uint8_t res;                                /// core spec's error code
    uint8_t acl_dir;                            ///(0x00):ACL_INIT_LOCAL  (0x01):ACL_INIT_PEER
    uint32_t dev_cls;                            ///remote device class of device
} bt_notify_device_acl_conn_info_t;

typedef struct
{
    bt_notify_device_mac_t mac;                  /// the bt device mac
    uint8_t res;                               /// core spec's error code
} bt_notify_device_base_info_t;
// ACL end

// profile start
typedef struct
{
    bt_notify_device_mac_t mac;                  /// the bt device mac
    uint8_t profile_type;                        /// BT_NOTIFY type id
    uint8_t res;                                /// error code is custom type
} bt_notify_profile_state_info_t;
// profile end

// error code table start
typedef enum
{
    BTS2_SUCC = 0,
    BTS2_FAILED,       /*  1 */
    BTS2_TIMEOUT,
    BTS2_BOND_FAIL,
    BTS2_CONN_FAIL,
    BTS2_SCO_FAIL,     /*  5 */
    BTS2_LINK_LOST,
    BTS2_LOCAL_DISC,
    BTS2_RMT_DISC,
    BTS2_REJ,
    BTS2_PSM_REJ,
    BTS2_SECU_FAIL,    /* 10 */
    BTS2_SDP_CLT_FAIL,
    BTS2_SDP_SRV_FAIL,
    BTS2_CMD_ERR,
    BTS2_DATA_WR_FAIL,
    BTS2_HW_ERR,       /* 15 */
    BTS2_UNSUPP_FEAT,
    BTS2_EXCEED_MAX_CONN_NUM,
    BTS2_IN_W4CONN_ST,
    BTS2_LOCAL_DISC_FAIL,
    BTS2_SCO_DISC_FAIL,
    BTS2_INQURI_INTERUPT,
    BTS2_INQURI_REPEAT_ERR,
    BTS2_INQURI_DEV_BUSY,
    BTS2_DEV_BUSY,
    BTS2_NO_PROFILE_LINK, //ADD for
} BTS2E_RESULT_CODE;
// error code table end

// sco start
typedef struct
{
    uint8_t sco_type;
    uint8_t sco_res;                               /// error code is custom type: BTS2E_RESULT_CODE
} bt_notify_device_sco_info_t;
// sco end

// at cmd cfm start
typedef struct
{
    uint8_t  at_cmd_id;
    uint8_t  res;                              /// error code is custom type (BTS2_SUCC(0)/BTS2_FAILED(1)/BTS2_TIMEOUT(2))
} bt_notify_at_cmd_cfm_t;
// at cmd cfm end
```

1. Bluetooth on/off demo:
    - Turn on Bluetooth, corresponding interface: bt_interface_open_bt
    - Turn off Bluetooth, corresponding interface: bt_interface_close_bt
    - Corresponding processing result message event handling as follows:
```c
static int bt_sifli_notify_common_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_COMMON_BT_STACK_READY:
    {
        // handle open bt
        break;
    }
    case BT_NOTIFY_COMMON_CLOSE_COMPLETE:
    {
        //handle close bt
        break;
    }
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
```

2. Bluetooth device scanning demo:
    - Start Bluetooth scanning, corresponding interface: bt_interface_start_inquiry
    - Stop Bluetooth scanning, corresponding interface: bt_interface_stop_inquiry
    - Corresponding processing result message event handling as follows:
```c
static int bt_sifli_notify_common_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_COMMON_DISCOVER_IND:
    {
        //handle inquiry result device info
        break;
    }
    case BT_NOTIFY_COMMON_INQUIRY_CMP:
    {
        //handle inquiry complete result
        break;
    }
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
```

3. Bluetooth device connection demo:
    - Connect Bluetooth device, corresponding interface: bt_interface_conn_ext
    - Disconnect Bluetooth device, corresponding interface: bt_interface_disc_ext
    - Corresponding processing result message event handling as follows:
```c
static int bt_sifli_notify_common_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_COMMON_BT_STACK_READY:
    {
        // handle open bt
        break;
    }
    case BT_NOTIFY_COMMON_CLOSE_COMPLETE:
    {
        //handle close bt
        break;
    }
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
```

4. Bluetooth device scanning demo:
    - Start Bluetooth scanning, corresponding interface: bt_interface_start_inquiry
    - Stop Bluetooth scanning, corresponding interface: bt_interface_stop_inquiry
    - Corresponding processing result message event handling as follows:
```c
static int bt_sifli_notify_(profile)_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_(profile_type)_PROFILE_CONNECTED:
    {
        //handle profile connected event
        break;
    }
    case BT_NOTIFY_(profile_type)_PROFILE_DISCONNECTED:
    {
        //handle profile disconnected event
        break;
    }
}

static int bt_sifli_notify_common_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_COMMON_ACL_CONNECTED:
    {
        //handle acl connected event
        break;
    }
    case BT_NOTIFY_COMMON_ACL_DISCONNECTED:
    {
        //handle acl disconnected event
        break;
    }
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
    case BT_NOTIFY_(profile_type):
    {
        ret = bt_sifli_notify_(profile)_event_hdl(event_id, data, data_len);
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
```

## Bluetooth Protocols and Specifications

```{toctree}
:maxdepth: 1
:titlesonly:

bt_hfp_hf_profile.md
bt_hfp_ag_profile.md
bt_gatt.md
bt_pan_profile.md
bt_pbap.md
bt_spp.md
```