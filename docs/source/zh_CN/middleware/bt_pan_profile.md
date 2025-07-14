# BT_PAN
PAN(Personal Area Networking Profile)即个人局域网协议。它描述了两台或多台支持蓝牙的设备如何组成一个自组网络，以及如何使用相同的机制通过网络接入点访问远程网络。
PAN Profile中主要包含的角色是:
- 网络接入点NAP(Network Access Point)
    - 支持网络接入点(NAP)的蓝牙设备是是一种为支持网络服务，提供以太网桥的一些功能的蓝牙设备。具有NAP服务的设备在每个连接的蓝牙设备之间转发网络数据包。NAP与PANU之间使用蓝牙网络封装协议(BNEP)交换数据。支持NAP服务的设备可以允许一个或多个连接设备访问网络。
- 群自组网络GN(Group Ad-hoc Network)
    - 群自组网允许移动主机协同创建自组网无线网络，而不使用额外的网络硬件。一个群自组网由一个蓝牙设备作为一个主节点操作，与作为从节点操作的1到7个蓝牙设备进行通信。GN与PANU之间使用蓝牙网络封装协议(BNEP)交换数据。此外，可能还有更多的非活动的群组网成员处于park mode。蓝牙强制执行了一个群自组网中只能有7个活动从设备的限制。
- 个人局域网用户PANU(Personal Area Network User)
    - PANU是使用NAP或GN服务的蓝牙设备。PANU支持作为NAP和GN角色的客户端角色，或者直接进行PANU到PANU的通信。


本文档主要是基于Sifli SDK，介绍作为PANU角色如何使用bt pan的基本功能。
涉及文件如下：
- bts2_app_interface
- bts2_app_pan
  
## bt_pan初始化
- pan初始化的函数：bt_pan_init，pan相关的状态、标志赋初始值
- pan服务使能的函数：bt_pan_reg，使能pan profile

```c
void bt_pan_init(bts2_app_stru *bts2_app_data)
{
    U8 i;
    bts2_app_data->pan_inst_ptr = &bts2_app_data->pan_inst;
    
    //赋值pan的初始状态
    bts2_app_data->pan_inst.pan_st = PAN_REG_ST;
    //每一个pan连接会分配一个id
    bts2_app_data->pan_inst.id = 0xffff;
    //初始化本地角色和对端角色
    bts2_app_data->pan_inst.local_role = PAN_NO_ROLE;
    bts2_app_data->pan_inst.rmt_role = PAN_NO_ROLE;
    bts2_app_data->pan_inst.mode = ACT_MODE;

    //初始化pan的sdp内容，主要是用于判断是否在做sdp查询
    //PAN_MAX_NUM是可以配置的最大支持sdp查询的数量
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

    //需要判断pan的状态，PAN_REG_ST为初始化状态，pan状态定义参考@bts2_pan_st
    if (ptr->pan_st == PAN_REG_ST)
    {
        pan_reg_req(bts2_task_get_app_task_id(), bts2_task_get_pan_task_id(), bts2_app_data->local_bd);
        //切换为PAN_IDLE_ST状态
        ptr->pan_st = PAN_IDLE_ST;
        //enable主要是完成本地sdp的注册
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
    PAN_IDLE_ST,                    /* 正在enable或者已经enable完成的状态 */
    PAN_REG_ST,                     /* 初始化状态 */
    PAN_SDS_REG_ST,                 /* 正在注册本端的sdp内容 */
    PAN_BUSY_ST                     /* 已经连上pan的状态 */
} bts2_pan_st;
```

## pan连接设备
以下流程描述了PANU如何发现、连接和使用NAP及其网络服务的过程。
1. 第一步是发现周边可以使用的NAP设备。为此，PANU可以对附近的设备执行搜索，然后使用SDP从那些支持NAP角色的设备中检索NAP服务。(注意：支持NAP的设备必须在class of device里的service class字段里包含Networking位)
2. 选择一个NAP设备进行连接。选择需要连接的设备发起ACL连接，如果NAP设备同时连接了多个PANU设备，则可能需要在连上ACL后做role switch。
3. BNEP连接。一旦创建完成了ACL连接，PANU应启动BNEP的L2CAP连接。BNEP连接过程包括所需的BNEP控制命令和可选的分组过滤器设置。如果NAP支持过滤，则应为每个连接存储所有已接受的网络数据包类型过滤器。
4. 网络数据包交互。NAP应将网络数据包转发到已连接的PANU设备。这与网络桥接器的行为类似。
5. PANU或NAP可以随时终止连接。

- pan连接设备接口为：
    - bts2_app_interface连接接口：bt_interface_conn_ext
    - bts2_app_pan连接接口：bt_pan_conn
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

//调用这个接口之前会把mac转换为BTS2S_BD_ADDR蓝牙地址格式
void bt_pan_conn(BTS2S_BD_ADDR *bd)
{
    bts2_app_stru *bts2_app_data = getApp();
    bts2_pan_inst_data *ptr = NULL;
    U8 idx, idx2;

    ptr = bts2_app_data->pan_inst_ptr;

    USER_TRACE(" pan st %x\n", ptr->pan_st);
    
    //检查state是不是PAN_IDLE_ST
    if (ptr->pan_st == PAN_IDLE_ST)
    {
        //判断是不是和想要连接的设备已经在连接pan了
        idx = bt_pan_get_idx_by_bd(bts2_app_data, bd);
        if (idx != PAN_MAX_NUM)
        {
            //正在做sdp查询了，完成之后会发起pan连接
            if (ptr->pan_sdp[idx].gn_sdp_pending == TRUE)
            {
                USER_TRACE("SDP is in progress,connect pan later\n");
            }
            else
            {
                //发起PAN NAP角色的sdp查询，主要是检查对端是否支持PAN NAP
                //做完sdp的查询过程会接着触发pan的连接过程
                ptr->pan_sdp[idx].gn_sdp_pending = TRUE;
                pan_svc_srch_req(bts2_task_get_app_task_id(), bd, PAN_NAP_ROLE);
                USER_TRACE(">> PAN connect\n");
            }
        }
        else
        {
            idx2 = bt_pan_get_idx(bts2_app_data);

            //不是正在连接的设备则分配一个新的
            if (idx2 != PAN_MAX_NUM)
            {
                //发起PAN NAP角色的sdp查询，主要是检查对端是否支持PAN NAP
                //做完sdp的查询过程会接着触发pan的连接过程
                bd_copy(&(ptr->pan_sdp[idx2].bd_addr), bd);
                ptr->pan_sdp[idx2].gn_sdp_pending = TRUE;
                pan_svc_srch_req(bts2_task_get_app_task_id(), bd, PAN_NAP_ROLE);
                USER_TRACE(">> PAN connect\n");
            }
            else
            {
                //没有资源可以用了，都是正在连接别的设备
                USER_TRACE("No pan resources are available\n");
            }
        }
    }
    else
    {
        //状态不对，连接失败
        USER_TRACE(">> PAN connect fail\n");
    }
}
```
       
- pan连接断开设备接口为：
    - bts2_app_interface断开连接接口：bt_interface_disc_ext
    - bts2_app_pan断开接口：bt_pan_disc
```c
/**
* @brief            Disconnect with the specified profile
* @param[in] mac    Remote device address
* @param[in] ext_profile : Profile value
*
* @return           bt_err_t
**/
bt_err_t bt_interface_disc_ext(unsigned char *mac, bt_profile_t ext_profile);

//调用这个接口之前会把mac转换为BTS2S_BD_ADDR蓝牙地址格式
void bt_pan_disc(BTS2S_BD_ADDR *bd)
{
    bts2_app_stru *bts2_app_data = getApp();
    bts2_pan_inst_data *ptr = NULL;
    ptr = bts2_app_data->pan_inst_ptr;
    
    //状态必须是PAN_BUSY_ST才能断开
    if (ptr->pan_st == PAN_BUSY_ST)
    {
        //断开pan
        pan_disc_req(ptr->id);
        USER_TRACE(">> PAN disconnect\n");
    }
    else
    {
        USER_TRACE(">> PAN disconnect fail");
    }
}
```
        
- pan事件处理:
    - pan连接状态回调event
        - pan连接成功：BT_NOTIFY_PAN_PROFILE_CONNECTED
        - pan连接失败：BT_NOTIFY_PAN_PROFILE_DISCONNECTED

:::{note}
注意：两个接口传递的地址参数需要进行相应的转换。
:::
```c
// 调用连接pan的API之后，pan连接成功的消息通过notify给用户
// 用户需要实现接收notify event的hdl函数 如：bt_notify_handle
// BT_NOTIFY_PAN_PROFILE_CONNECTED event里面包含：地址信息 、profile_type、 res：0（成功）
// 断开pan后也会收到相应的事件
// BT_NOTIFY_PAN_PROFILE_DISCONNECTED event里面包含：地址信息 、profile_type、 原因
// 具体结构体信息参考API注释
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
        //用户自己实现相应的处理函数
        break;
    }
    case BT_NOTIFY_PAN_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        //用户自己实现相应的处理函数
        break;
    }
    default:
        return -1;
    }
    return 0;
}
```

## pan基本功能使用
### public API
```c
//设置IP Address
void bt_pan_set_ip_addr(char *string);

//设置网络掩码
void bt_pan_set_netmask(char *string);

//设置网关
void bt_pan_set_gw(char *string);

//设置路由
void bt_pan_set_nap_route(char *string);

//设置DNS
void bt_pan_set_dns1(char *string);
void bt_pan_set_dns2(char *string);

//遍历当前的net device
void bt_pan_scan_proc_net_dev(void);
```

#### bt_lwip接口层
Sifli SDK的网络实现是基于lwip的，并且pan并不会直接调用lwip的接口，而是通过bt_lwip作为中间层间接进行调用。
LwIP是Light Weight (轻型)IP协议，有无操作系统的支持都可以运行。
LwIP实现的重点是在保持TCP协议主要功能的基础上减少对RAM 的占用，它只需十几KB的RAM和40K左右的ROM就可以运行，这使LwIP协议栈适合在低端的嵌入式系统中使用。
```c
//1.bt_lwip的初始化
static struct rt_bt_prot_ops ops =
{
    //收到数据的回调
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

//上电自动调用初始化
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

//再收到pan的连接事件BTS2MU_PAN_CONN_IND时，会把pan注册到bt_lwip里
void bt_lwip_pan_control_tcpip(bts2_app_stru *bts2_app_data)
{
    bt_pan_instance[0].bts2_app_data =  bts2_app_data;
    bt_pan_instance[0].ops =  &instance_ops;
    rt_bt_prot_attach_pan_instance(&bt_pan_instance[0]);
}

rt_err_t rt_bt_prot_attach_pan_instance(struct rt_bt_pan_instance *panInstance)
{
    panInstance->prot = bt_prot;
    //会调用rt_bt_lwip_protocol_register
    panInstance->prot = bt_prot->ops->dev_reg_callback(panInstance->prot, panInstance); /* attach prot */
    return RT_EOK;
}

//2.pan发送数据的接口会注册到ETH device，从而将网络数据包通过pan发送出去
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
        //调用bt_lwip_pan_send
        bt_instance->ops->bt_send(bt_instance, buff, len);
        return RT_EOK;
    }
    return -RT_ERROR;
}

//pan发送数据
void bt_lwip_pan_send(struct rt_bt_pan_instance *bt_instance, void *buff, int len)
{
    bts2_pan_inst_data *ptr = NULL;
    void  *p;
    U8   *eth_header;
    ptr = bt_instance->bts2_app_data->pan_inst_ptr;

    if (ptr->pan_st == PAN_BUSY_ST)
    {
        //pan发送数据会退出sniff，以达到更快的性能
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

//3.pan接收到网络数据包后，会调用注册在pan的接收数据回调
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
    //调用相应的回调
    rt_bt_instance_transfer_prot(&bt_pan_instance[0], (void *)msg->payload, msg->len);
    bfree(msg->payload);
    break;
}

//通过rt_bt_prot_regisetr将prot注册到bt_prot，再通过rt_bt_prot_attach_pan_instance将bt_prot注册到pan
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

## PAN功能使用demo
- 首先在工程初始化时，注册接收notify event 的处理函数
- 输入需要连接手机的mac地址，等待连接成功的消息
- 想要使用pan，必须要在手机端打开蓝牙网络共享功能，不同操作系统的手机蓝牙网络共享功能开启方式可在网上查询
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

            //初始化lwip
            lwip_sys_init();
            bt_lwip_pan_control_tcpip(bts2_app_data);
        }
        USER_TRACE(" BTS2MU_PAN_CONN_IND\n");
        //用户可以在pan连上之后，开始使用网络服务，具体可以参照lwip源代码下的app文件
        break;
    }
    //PAN DISCONNECTED
    case BTS2MU_PAN_DISC_IND:
    {
        BTS2S_PAN_DISC_IND *msg;
        msg = (BTS2S_PAN_DISC_IND *)bts2_app_data->recv_msg;

        //解除prot的注册
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

        //传给注册的网络设备
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