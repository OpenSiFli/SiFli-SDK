# BT_PAN
PAN (Personal Area Networking Profile) is a personal area networking protocol. It describes how two or more Bluetooth-enabled devices form an ad-hoc network and how to use the same mechanism to access remote networks through network access points.

The main roles included in PAN Profile are:
- Network Access Point NAP (Network Access Point)
    - A Bluetooth device supporting Network Access Point (NAP) is a Bluetooth device that provides some functions of an Ethernet bridge to support network services. Devices with NAP service forward network packets between each connected Bluetooth device. NAP exchanges data with PANU using Bluetooth Network Encapsulation Protocol (BNEP). Devices supporting NAP service can allow one or more connected devices to access the network.
- Group Ad-hoc Network GN (Group Ad-hoc Network)
    - Group ad-hoc network allows mobile hosts to collaboratively create ad-hoc wireless networks without using additional network hardware. A group ad-hoc network consists of one Bluetooth device operating as a master node, communicating with 1 to 7 Bluetooth devices operating as slave nodes. GN exchanges data with PANU using Bluetooth Network Encapsulation Protocol (BNEP). Additionally, there may be more inactive group network members in park mode. Bluetooth enforces a limit of only 7 active slave devices in a group ad-hoc network.
- Personal Area Network User PANU (Personal Area Network User)
    - PANU is a Bluetooth device that uses NAP or GN services. PANU supports client roles for both NAP and GN roles, or direct PANU to PANU communication.

This document is mainly based on Sifli SDK, introducing how to use basic bt pan functions as a PANU role.
Related files include:
- bts2_app_interface
- bts2_app_pan
  
## bt_pan Initialization
- PAN initialization function: bt_pan_init, assigns initial values to PAN-related states and flags
- PAN service enable function: bt_pan_reg, enables PAN profile

```c
void bt_pan_init(bts2_app_stru *bts2_app_data)
{
    U8 i;
    bts2_app_data->pan_inst_ptr = &bts2_app_data->pan_inst;
    
    //Assign initial state of PAN
    bts2_app_data->pan_inst.pan_st = PAN_REG_ST;
    //Each PAN connection will be allocated an ID
    bts2_app_data->pan_inst.id = 0xffff;
    //Initialize local role and remote role
    bts2_app_data->pan_inst.local_role = PAN_NO_ROLE;
    bts2_app_data->pan_inst.rmt_role = PAN_NO_ROLE;
    bts2_app_data->pan_inst.mode = ACT_MODE;

    //Initialize PAN SDP content, mainly used to determine if SDP query is in progress
    //PAN_MAX_NUM is the configurable maximum number of supported SDP queries
    for (i = 0; i < PAN_MAX_NUM; i++)
    {
        bd_set_empty(&(bts2_app_data->pan_inst.pan_sdp[i].bd_addr));
        bts2_app_data->pan_inst.pan_sdp[i].gn_sdp_pending = FALSE;
        bts2_app_data->pan_inst.pan_sdp[i].gn_sdp_fail = FALSE;
        bts2_app_data->pan_inst.pan_sdp[i].nap_sdp_pending = FALSE;
        bts2_app_data->pan_inst.pan_sdp[i].nap_sdp_fail = FALSE;
    }
}

void bt_pan_reg(bts2_app_stru *bts2_app_data)
{
    bts2_pan_inst_data *ptr = NULL;
    ptr = bts2_app_data->pan_inst_ptr;

    //Need to check PAN state, PAN_REG_ST is initialization state, PAN state definition refers to @bts2_pan_st
    if (ptr->pan_st == PAN_REG_ST)
    {
        pan_reg_req(bts2_task_get_app_task_id(), bts2_task_get_pan_task_id(), bts2_app_data->local_bd);
        //Switch to PAN_IDLE_ST state
        ptr->pan_st = PAN_IDLE_ST;
        //Enable mainly completes local SDP registration
        bt_pan_enable(bts2_app_data);
        USER_TRACE(">> PAN register start\n");
    }
    else
    {
        USER_TRACE(">> PAN register fail\n");
    }
}

typedef enum
{
    PAN_IDLE_ST,                    /* State of enabling or already enabled */
    PAN_REG_ST,                     /* Initialization state */
    PAN_SDS_REG_ST,                 /* Registering local SDP content */
    PAN_BUSY_ST                     /* Already connected to PAN state */
} bts2_pan_st;
```

## PAN Device Connection
The following process describes how PANU discovers, connects to, and uses NAP and its network services.
1. The first step is to discover available NAP devices in the vicinity. For this, PANU can perform a search on nearby devices, then use SDP to retrieve NAP services from those devices that support NAP role. (Note: Devices supporting NAP must include the Networking bit in the service class field of class of device)
2. Select a NAP device to connect to. Choose the device to connect to and initiate ACL connection. If the NAP device connects to multiple PANU devices simultaneously, role switch may be required after ACL connection.
3. BNEP connection. Once ACL connection is established, PANU should initiate BNEP L2CAP connection. The BNEP connection process includes required BNEP control commands and optional packet filter setup. If NAP supports filtering, all accepted network packet type filters should be stored for each connection.
4. Network packet interaction. NAP should forward network packets to connected PANU devices. This behaves similarly to a network bridge.
5. PANU or NAP can terminate the connection at any time.

- PAN device connection interface:
    - bts2_app_interface connection interface: bt_interface_conn_ext
    - bts2_app_pan connection interface: bt_pan_conn
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

//Before calling this interface, the mac will be converted to BTS2S_BD_ADDR Bluetooth address format
void bt_pan_conn(BTS2S_BD_ADDR *bd)
{
    bts2_app_stru *bts2_app_data = getApp();
    bts2_pan_inst_data *ptr = NULL;
    U8 idx, idx2;

    ptr = bts2_app_data->pan_inst_ptr;

    USER_TRACE(" pan st %x\n", ptr->pan_st);
    
    //Check if state is PAN_IDLE_ST
    if (ptr->pan_st == PAN_IDLE_ST)
    {
        //Check if already connecting PAN with the target device
        idx = bt_pan_get_idx_by_bd(bts2_app_data, bd);
        if (idx != PAN_MAX_NUM)
        {
            //SDP query is in progress, PAN connection will be initiated after completion
            if (ptr->pan_sdp[idx].gn_sdp_pending == TRUE)
            {
                USER_TRACE("SDP is in progress,connect pan later\n");
            }
            else
            {
                //Initiate SDP query for PAN NAP role, mainly to check if peer supports PAN NAP
                //After SDP query process, PAN connection process will be triggered
                ptr->pan_sdp[idx].gn_sdp_pending = TRUE;
                pan_svc_srch_req(bts2_task_get_app_task_id(), bd, PAN_NAP_ROLE);
                USER_TRACE(">> PAN connect\n");
            }
        }
        else
        {
            idx2 = bt_pan_get_idx(bts2_app_data);

            //If not a device already connecting, allocate a new one
            if (idx2 != PAN_MAX_NUM)
            {
                //Initiate SDP query for PAN NAP role, mainly to check if peer supports PAN NAP
                //After SDP query process, PAN connection process will be triggered
                bd_copy(&(ptr->pan_sdp[idx2].bd_addr), bd);
                ptr->pan_sdp[idx2].gn_sdp_pending = TRUE;
                pan_svc_srch_req(bts2_task_get_app_task_id(), bd, PAN_NAP_ROLE);
                USER_TRACE(">> PAN connect\n");
            }
            else
            {
                //No resources available, all are connecting to other devices
                USER_TRACE("No pan resources are available\n");
            }
        }
    }
    else
    {
        //Wrong state, connection failed
        USER_TRACE(">> PAN connect fail\n");
    }
}
```
       
- PAN device disconnection interface:
    - bts2_app_interface disconnection interface: bt_interface_disc_ext
    - bts2_app_pan disconnection interface: bt_pan_disc
```c
/**
* @brief            Disconnect with the specified profile
* @param[in] mac    Remote device address
* @param[in] ext_profile : Profile value
*
* @return           bt_err_t
**/
bt_err_t bt_interface_disc_ext(unsigned char *mac, bt_profile_t ext_profile);

//Before calling this interface, the mac will be converted to BTS2S_BD_ADDR Bluetooth address format
void bt_pan_disc(BTS2S_BD_ADDR *bd)
{
    bts2_app_stru *bts2_app_data = getApp();
    bts2_pan_inst_data *ptr = NULL;
    ptr = bts2_app_data->pan_inst_ptr;
    
    //State must be PAN_BUSY_ST to disconnect
    if (ptr->pan_st == PAN_BUSY_ST)
    {
        //Disconnect PAN
        pan_disc_req(ptr->id);
        USER_TRACE(">> PAN disconnect\n");
    }
    else
    {
        USER_TRACE(">> PAN disconnect fail");
    }
}
```
        
- PAN event handling:
    - PAN connection status callback events
        - PAN connection successful: BT_NOTIFY_PAN_PROFILE_CONNECTED
        - PAN connection failed: BT_NOTIFY_PAN_PROFILE_DISCONNECTED

:::{note}
Note: The address parameters passed by the two interfaces need to be converted accordingly.
:::
```c
// After calling the API to connect PAN, the PAN connection success message is sent to user through notify
// Users need to implement hdl functions to receive notify events, such as: bt_notify_handle
// BT_NOTIFY_PAN_PROFILE_CONNECTED event includes: address information, profile_type, res: 0 (success)
// After disconnecting PAN, corresponding events will also be received
// BT_NOTIFY_PAN_PROFILE_DISCONNECTED event includes: address information, profile_type, reason
// For specific structure information, refer to API comments
static int bt_notify_handle(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    int ret = -1;

    switch (type)
    {
        case BT_NOTIFY_PAN:
        {
            bt_sifli_notify_pan_event_hdl(event_id, data, data_len);
        }
        break;
    }
    return 0;
}


static int bt_sifli_notify_pan_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_PAN_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //Users implement their own corresponding processing functions
        break;
    }
    case BT_NOTIFY_PAN_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //Users implement their own corresponding processing functions
        break;
    }
    default:
        return -1;
    }
    return 0;
}
```

## PAN Basic Functionality Usage
### Public API
```c
//Set IP Address
void bt_pan_set_ip_addr(char *string);

//Set network mask
void bt_pan_set_netmask(char *string);

//Set gateway
void bt_pan_set_gw(char *string);

//Set route
void bt_pan_set_nap_route(char *string);

//Set DNS
void bt_pan_set_dns1(char *string);
void bt_pan_set_dns2(char *string);

//Traverse current net device
void bt_pan_scan_proc_net_dev(void);
```

#### bt_lwip Interface Layer
Sifli SDK's network implementation is based on lwip, and PAN does not directly call lwip interfaces, but indirectly calls through bt_lwip as an intermediate layer.
LwIP is Light Weight IP protocol, which can run with or without operating system support.
The focus of LwIP implementation is to reduce RAM usage while maintaining the main functions of TCP protocol. It only needs about ten KB of RAM and about 40K ROM to run, making LwIP protocol stack suitable for use in low-end embedded systems.
```c
//1. bt_lwip initialization
static struct rt_bt_prot_ops ops =
{
    //Data receive callback
    rt_bt_lwip_protocol_recv,
    //register ETH device
    rt_bt_lwip_protocol_register,
    //unregister ETH device
    rt_bt_lwip_protocol_unregister
};

int rt_bt_lwip_init(void)
{
    static struct rt_bt_prot prot;
    rt_bt_prot_event_t event;

    rt_memset(&prot, 0, sizeof(prot));
    rt_strncpy(&prot.name[0], RT_BT_PROT_LWIP, RT_BT_PROT_NAME_LEN);
    prot.ops = &ops;

    if (rt_bt_prot_regisetr(&prot) != RT_EOK)
    {
        LOG_E("F:%s L:%d protocol regisetr failed", __FUNCTION__, __LINE__);
        return -1;
    }

    return 0;
}

//Automatically called on power-up initialization
INIT_PREV_EXPORT(rt_bt_lwip_init);

rt_err_t rt_bt_prot_regisetr(struct rt_bt_prot *prot)
{
    int i;
    rt_uint32_t id;
    static rt_uint8_t num;

    /* Parameter checking */
    if ((prot == RT_NULL) ||
            (prot->ops->prot_recv == RT_NULL) ||
            (prot->ops->dev_reg_callback == RT_NULL))
    {
        LOG_E("F:%s L:%d Parameter Wrongful", __FUNCTION__, __LINE__);
        return -RT_EINVAL;
    }

    /* save prot */
    bt_prot = prot;

    // rt_kprintf("lwip:rt_bt_prot_regisetr \n");

    return RT_EOK;
}

//When receiving PAN connection event BTS2MU_PAN_CONN_IND, PAN will be registered to bt_lwip
void bt_lwip_pan_control_tcpip(bts2_app_stru *bts2_app_data)
{
    bt_pan_instance[0].bts2_app_data =  bts2_app_data;
    bt_pan_instance[0].ops =  &instance_ops;
    rt_bt_prot_attach_pan_instance(&bt_pan_instance[0]);
}

rt_err_t rt_bt_prot_attach_pan_instance(struct rt_bt_pan_instance *panInstance)
{
    panInstance->prot = bt_prot;
    //Will call rt_bt_lwip_protocol_register
    panInstance->prot = bt_prot->ops->dev_reg_callback(panInstance->prot, panInstance); /* attach prot */
    return RT_EOK;
}

//2. PAN data sending interface will be registered to ETH device, thus sending network packets through PAN
static rt_err_t rt_bt_lwip_protocol_send(rt_device_t device, struct pbuf *p)
{
    struct rt_bt_pan_instance *bt_instance = ((struct eth_device *)device)->parent.user_data;

    //LOG_D("F:%s L:%d run", __FUNCTION__, __LINE__);

    rt_uint8_t *frame;

    /* sending data directly */
    if (p->len == p->tot_len)
    {

        // rt_kprintf("enter rt_bt_lwip_protocol_send total ,total len %d\n",p->tot_len);
        frame = (rt_uint8_t *)p->payload;
        rt_bt_prot_transfer_instance(bt_instance, frame, p->tot_len);
        LOG_D("F:%s L:%d run len:%d", __FUNCTION__, __LINE__, p->tot_len);
        return RT_EOK;
    }

    frame = rt_malloc(p->tot_len);
    if (frame == RT_NULL)
    {
        LOG_E("F:%s L:%d malloc out_buf fail\n", __FUNCTION__, __LINE__);
        return -RT_ENOMEM;
    }
    /*copy pbuf -> data dat*/
    pbuf_copy_partial(p, frame, p->tot_len, 0);
    /* send data */
    //rt_kprintf("enter rt_bt_lwip_protocol_send fragment ,total len %d\n",p->tot_len);
    rt_bt_prot_transfer_instance(bt_instance, frame, p->tot_len);
    LOG_D("F:%s L:%d run len:%d", __FUNCTION__, __LINE__, p->tot_len);
    rt_free(frame);
    return RT_EOK;
}

rt_err_t rt_bt_prot_transfer_instance(struct rt_bt_pan_instance *bt_instance, void *buff, int len)
{
    if (bt_instance->ops->bt_send != RT_NULL)
    {
        //Call bt_lwip_pan_send
        bt_instance->ops->bt_send(bt_instance, buff, len);
        return RT_EOK;
    }
    return -RT_ERROR;
}

//PAN send data
void bt_lwip_pan_send(struct rt_bt_pan_instance *bt_instance, void *buff, int len)
{
    bts2_pan_inst_data *ptr = NULL;
    void  *p;
    U8   *eth_header;
    ptr = bt_instance->bts2_app_data->pan_inst_ptr;

    if (ptr->pan_st == PAN_BUSY_ST)
    {
        //PAN data sending will exit sniff to achieve faster performance
        if (ptr->mode == SNIFF_MODE)
            bt_exit_sniff_mode(bt_instance->bts2_app_data);

        BTS2S_PAN_DATA_REQ *msg;
        msg = (BTS2S_PAN_DATA_REQ *)bmalloc(sizeof(BTS2S_PAN_DATA_REQ));
        BT_OOM_ASSERT(msg);
        if (msg)
        {
            msg->type = BTS2MD_PAN_DATA_REQ;
            eth_header = buff;
            msg->ether_type = (eth_header[12] << 8) + eth_header[13];
            msg->len = len - 14;
            buff = eth_header + 14;

            //msg->dst_addr = bt_pan_get_remote_mac_address(bt_instance);
            msg->dst_addr.w[0] = (((U16)eth_header[0]) << 8) | (U16)eth_header[1];
            msg->dst_addr.w[1] = (((U16)eth_header[2]) << 8) | (U16)eth_header[3];
            msg->dst_addr.w[2] = (((U16)eth_header[4]) << 8) | (U16)eth_header[5];
            msg->src_addr = bt_pan_get_mac_address(bt_instance);
            p = bmalloc(msg->len);
            BT_OOM_ASSERT(p);
            if (p == NULL)
            {
                bfree(msg);
                return;
            }
            memcpy(p, buff, msg->len);
            msg->payload = p;
            bts2_msg_put(bts2_task_get_pan_task_id(), BTS2M_PAN, msg);
            //USER_TRACE("bt_lwip_pan_send\n");
        }
    }
    else
    {
        USER_TRACE(">> PAN data send fail\n");
    }
}

//3. After PAN receives network packets, it will call the registered data receive callback in PAN
case BTS2MU_PAN_DATA_IND:
{
    BTS2S_PAN_DATA_IND *msg;
    msg = (BTS2S_PAN_DATA_IND *)bts2_app_data->recv_msg;

    //USER_TRACE(" BTS2MU_PAN_DATA_IND\n");
    msg->len = msg->len + 14;
    memcpy(msg->payload, msg->dst_addr.w, 6);
    memcpy(msg->payload + 6, msg->src_addr.w, 6);
    msg->payload[12] = (msg->ether_type >> 8);
    msg->payload[13] = (msg->ether_type & 0xff);
    if (0x86dd == msg->ether_type)
    {
        bfree(msg->payload);
        break;
    }
    //Call corresponding callback
    rt_bt_instance_transfer_prot(&bt_pan_instance[0], (void *)msg->payload, msg->len);
    bfree(msg->payload);
    break;
}

//Register prot to bt_prot through rt_bt_prot_regisetr, then register bt_prot to PAN through rt_bt_prot_attach_pan_instance
rt_err_t rt_bt_instance_transfer_prot(struct rt_bt_pan_instance *bt_instance, void *buff, int len)
{
    struct rt_bt_prot *prot = bt_instance->prot;

    if (prot != RT_NULL)
    {
        return prot->ops->prot_recv(bt_instance, buff, len);
    }
    return -RT_ERROR;
}
```

## PAN Functionality Usage Demo
- First, during project initialization, register the processing function for receiving notify events
- Enter the MAC address of the phone to connect to, wait for connection success message
- To use PAN, you must enable Bluetooth network sharing on the phone. Different operating systems have different ways to enable Bluetooth network sharing, which can be searched online
```c
//register notify event handle function start
int bt_sifli_notify_pan_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    // PAN CONNECTED
    case BTS2MU_PAN_CONN_IND:
    {
        BTS2S_PAN_CONN_IND *msg;

        msg = (BTS2S_PAN_CONN_IND *)bts2_app_data->recv_msg;

        if (msg->res == BTS2_SUCC)
        {
            bt_notify_profile_state_info_t profile_state;

            //Initialize lwip
            lwip_sys_init();
            bt_lwip_pan_control_tcpip(bts2_app_data);
        }
        USER_TRACE(" BTS2MU_PAN_CONN_IND\n");
        //Users can start using network services after PAN is connected, specific reference can be found in the app files under lwip source code
        break;
    }
    //PAN DISCONNECTED
    case BTS2MU_PAN_DISC_IND:
    {
        BTS2S_PAN_DISC_IND *msg;
        msg = (BTS2S_PAN_DISC_IND *)bts2_app_data->recv_msg;

        //Unregister prot
        bt_lwip_pan_detach_tcpip(bts2_app_data);

        bt_notify_profile_state_info_t profile_state;

        lwip_system_uninit();
        INFO_TRACE(" BTS2MU_PAN_DISC_IND\n");
        break;
    }
    //data receive
    case BTS2MU_PAN_DATA_IND:
    {
        BTS2S_PAN_DATA_IND *msg;
        msg = (BTS2S_PAN_DATA_IND *)bts2_app_data->recv_msg;

        msg->len = msg->len + 14;
        memcpy(msg->payload, msg->dst_addr.w, 6);
        memcpy(msg->payload + 6, msg->src_addr.w, 6);
        msg->payload[12] = (msg->ether_type >> 8);
        msg->payload[13] = (msg->ether_type & 0xff);

        //Pass to registered network device
        rt_bt_instance_transfer_prot(&bt_pan_instance[0], (void *)msg->payload, msg->len);
        bfree(msg->payload);
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
    //BT functionality enabled successfully, can be used normally
    case BT_NOTIFY_COMMON_BT_STACK_READY:
    {
        break;
    }
    //BT shutdown successful
    case BT_NOTIFY_COMMON_CLOSE_COMPLETE:
    {
        break;
    }
    // ACL connection successful
    case BT_NOTIFY_COMMON_ACL_CONNECTED:
    {
        bt_notify_device_acl_conn_info_t *acl_info = (bt_notify_device_acl_conn_info_t *) data;
        //device acl connected
        break;
    }
    // ACL disconnection successful
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

    case BT_NOTIFY_PAN:
    {
        bt_sifli_notify_pan_event_hdl(event_id, data, data_len);
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
bt_interface_conn_ext((unsigned char *)(mac), BT_PROFILE_PAN);
//
```