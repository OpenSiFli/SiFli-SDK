# 经典蓝牙服务

## 概述

目前蓝牙主要分为经典蓝牙BR(Basic Rate)/EDR(Enhanced Data Rate)和低功耗(BLE)蓝牙。Sifli蓝牙是基于蓝牙5.3提供了一套通用双模蓝牙方案。Sifli蓝牙集中支持蓝牙音频和蓝牙语音功能，提供便捷的相关开发方式。Sifli蓝牙具备良好的可阅读性，可扩展性和可移植性。Sifli蓝牙以模块化为导向，提供丰富的接口，模块之间层次和逻辑清晰，简单易懂，为用户学习和开发提供了良好的基础。Sifli蓝牙整体设计目标，是让用户在更短的时间之内，通过对Sifli蓝牙进行配置或者二次开发，就可以形成高品质的产品。本节内容主要是介绍Sifli关于经典蓝牙服务的架构和相关规范的配置信息。

为了在支持蓝牙的设备上，通过蓝牙传输数据，Sifli提供了基本蓝牙功能的API和常用蓝牙协议规范(A2DP/AVRCP/HFP/PABAP/HID/SPP等)，用户可以通过API执行如下操作：
- 扫描其他蓝牙设备 
- 管理已经连接设备
- 通过无线方式与其他蓝牙设备(如：手机，耳机等)实现通话、音频等功能   
- 与其他设备相互传输自定义数据  

:::{note}
更多关于API的信息可以查看：[bluetooth_service](middleware-bluetooth_service)
:::

### 经典蓝牙架构
Sifli经典蓝牙架构的主旨是打造一套方便用户使用的，稳定、舒适、便捷的解决方案，将用户关心的功能和逻辑进行结构化封装，最大可能地减少用户二次开发的工作量。为此，Sifli经典蓝牙架构的设计始终围绕着结构的层次化和功能的模块化两个角度进行架构。 
- 从蓝牙整体架构上看，蓝牙分为了控制器（controller）和主机（host）两大部分。
    - 控制器包括了 PHY、Baseband、Link Controller、Link Manager、Device Manager、HCI 等模块，用于硬件接⼝管理、链路管理等;
    - 主机则包括了 L2CAP、RFCOMM、SDP、GAP 以及各种规范，构建了向应用层提供接口的基础，方便用户对蓝牙系统的访问。
- Sifli的控制器在LCPU上运行，主机部分和提供给用户访问蓝牙相关功能的API在HCPU上运行。
- Sifli的主机部分分为三大类：协议栈（stack）/服务（service）/接口（interface）。
    - 协议栈主要是蓝牙协议和规范的相关状态机、消息处理的实现，是蓝牙相关规范的核心，提供蓝牙建立通信和进行数据交互的标准化接口。以lib的形式提供给用户; 
    - 服务（service）主要是提供具体蓝牙功能服务相关信息处理的模块，是面向用户特殊化功能而设计的，以源码的形式释放给用户; 
    - 接口（interface）主要是提供给用户访问蓝牙系统的具体功能，是简单高效的通用型接口，以源码的形式释放给用户，用户可以根据需求使用或者定制相关功能。  
![图1：Sifli思澈蓝牙架构](../../assets/bluetooth_arch.png)

### 蓝牙规范配置信息 
目前，主机协议栈支持的经典蓝⽛规范和协议如下:   
- 规范：A2DP、AVRCP、GATT over BR、HFP、HID、PAN、PBAP、SPP 
- 协议：L2CAP、SDP、AVDTP、AVCTP、RFCOMM  

![图1：Sifli思澈蓝牙支持的蓝牙规范](../../assets/bluetooth_profiles.png)

## 蓝牙服务配置和开发说明
### Sifli SDK工程介绍
![Sifli思澈工程文件](../../assets/SDK_ARCH.png)
- customer:  主要放置board板级相关的以及外设驱动
- drivers:   主要放置芯片相关外设的hal驱动 
- dvt:       主要放置芯片ATE相关测试工程
- example:   主要放置工程示例
- external:  主要放置第三方开源项目代码
- middleware:主要放置中间件相关的代码
- rtos:      主要放置rtthread os相关代码  

### Sifli 蓝牙工程编译基本流程
1. 工程环境：运行set_env.bat,构建编译环境
2. 进入目录：siflisdk\example\bt\test_example\project\common
3. 在上面的路径下进行编译对应的BT工程，生成的文件也在common路径下
    - 编译结果build_<板子名称>_hcpu目录说明：
        - board：      主要放置board板级相关可执行文件
        - bootloader： 主要放置bootloader相关可执行文件
        - ftab:        主要放置flash table相关可执行文件
        - lcpu:        主要放置lcpu相关可执行文件
        - peripheral:  主要放置外设驱动相关可执行文件
        - sifli_sdk:   主要放置sdk 中间件相关可执行文件  
4. 烧录文件：运行脚本uart_download.bat  
![Sifli经典蓝牙示例](../../assets/BT_PROJECT.png)
![Sifli经典蓝牙编译](../../assets/complie_result.png)

### Sifli SDK蓝牙相关文件说明 
- Sifli sdk蓝牙相关源码需要进入文件夹：siflisdk\middleware\bluetooth\service\bt
    - bt_cm 文件是蓝牙connection_manager相关的内容
    - bt_finsh 文件是蓝牙服务和配置相关的内容，具体说明如下图所示：  
    ![Sifli思澈SDK蓝牙相关源码说明](../../assets/service_profiles.png)
#### Sifli_SDK蓝牙初始化和profile配置简要说明
1. Sifli SDK在rt-thread系统上运行，除了rt-thread上自带的task外，Sifli根据蓝牙特性，实现了特殊的task机制专门用于处理蓝牙相关的event，运行在bts_task上，蓝牙定制task有特殊的子task_id，蓝牙服务相关功能均在子task_id：bts2_task_get_app_task_id()上运行  
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
2. bts2_main将子task_id：bts2_task_get_app_task_id()的初始化函数，释放函数和事件处理函数通过调用bts2_app_reg_cbk进行注册
3. 当子task相关的内容初始化时，按照顺序执行到app_fn_init函数
4. 子task相关的event通过app_fn_hdl_ext进行分发处理
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
5. 当子task：bts2_task_get_app_task_id()初始化完成收到BTS2MU_GAP_APP_INIT_CFM后，开始进行蓝牙profile功能的初始化。
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
#### 蓝牙基本信息更改接口
1. 更改蓝牙设备COD(class of device):重新实现bt_get_class_of_device函数
```c
__WEAK uint32_t bt_get_class_of_device(void)
{
    return 0x240704;
}
```
2. 更改蓝牙设备名称：调用接口void bt_interface_set_local_name(int len, void *data);
3. 蓝牙profile 配置 ：bt_init_profile
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
### 蓝牙服务接口调用和回调使用方法和注意事项
- 蓝牙接口整合了BT的发现、连接和回连的功能，相关功能接口主要在：bts2_app_interface
- 蓝牙服务回调包含远端蓝牙设备传输的数据，调用蓝牙服务接口处理的结果以及主动更新蓝牙状态信息等 
    - Sifli蓝牙服务回调需要用户自己实现回调处理函数，并将数据转移到用户自己task处理相关数据
    - 注册函数：bt_interface_register_bt_event_notify_callback
    - 注销函数：bt_interface_unregister_bt_event_notify_callback   
:::{note}
回调函数的数据处理task为用户自定义实现的task。
:::

```c
int bt_notify_event_hdl(uint16_t type, uint16_t event_id, uint8_t *data, uint16_t data_len)
{
     return 0;
}
```
#### 蓝牙重要event简易说明
bt_notify_event_hdl收到的消息有：
- ACL 连接成功的消息:BT_NOTIFY_COMMON_ACL_CONNECTED
- ACL 断开连接成功的消息： BT_NOTIFY_COMMON_ACL_DISCONNECTED
- profile 连接成功的消息：BT_NOTIFY_(profile_type)_PROFILE_CONNECTED
- profile 断开连接成功的消息：BT_NOTIFY_(profile_type)_PROFILE_DISCONNECTED
- SCO 连接成功的消息:BT_NOTIFY_COMMON_SCO_CONNECTED
- SCO 断开连接成功的消息:BT_NOTIFY_COMMON_SCO_DISCONNECTED
- HFP_HF发送at cmd对方设备处理结果消息：BT_NOTIFY_HF_AT_CMD_CFM
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
    uint8_t res;                                /// error code 为自定义类型
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
    uint8_t sco_res;                               /// error code 为自定义类型:BTS2E_RESULT_CODE
} bt_notify_device_sco_info_t;
// sco end

// at cmd cfm start
typedef struct
{
    uint8_t  at_cmd_id;
    uint8_t  res;                              /// error code 为自定义类型(BTS2_SUCC(0)/BTS2_FAILED(1)/BTS2_TIMEOUT(2))
} bt_notify_at_cmd_cfm_t;
// at cmd cfm end
```
1. 蓝牙开关demo：
    - 打开蓝牙，对应接口：bt_interface_open_bt
    - 关闭蓝牙，对应接口：bt_interface_close_bt
    - 对应处理结果的消息event处理如下：
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
2. 蓝牙扫描设备demo：
    - 开启蓝牙扫描，对应接口：bt_interface_start_inquiry
    - 关闭蓝牙扫描，对应接口：bt_interface_stop_inquiry
    - 对应处理结果的消息event处理如下：
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

3. 蓝牙连接设备demo：
    - 连接蓝牙设备，对应接口：bt_interface_conn_ext
    - 断开蓝牙设备，对应接口：bt_interface_disc_ext
    - 对应处理结果的消息event处理如下：
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
4. 蓝牙扫描设备demo：
    - 开启蓝牙扫描，对应接口：bt_interface_start_inquiry
    - 关闭蓝牙扫描，对应接口：bt_interface_stop_inquiry
    - 对应处理结果的消息event处理如下：
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
## 蓝牙协议和规范

```{toctree}
:maxdepth: 1
:titlesonly:

bt_hfp_hf_profile.md
bt_hfp_ag_profile.md
bt_gatt.md
bt_pan_profile.md
bt_pbap.md
bt_spp.md
bt_a2dp_sink_profile.md
bt_a2dp_source_profile.md
bt_avrcp_profile.md

```