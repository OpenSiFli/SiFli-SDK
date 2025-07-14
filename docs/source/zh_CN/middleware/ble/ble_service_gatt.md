# Sibles GATT 服务

## GATT

通用属性配置文件（GATT）使用属性（ATT）协议来定义一个服务框架来传输数据。 GATT 定义了两个角色，客户端和服务器。 
- 服务端按照ATT数据格式构建服务数据库，响应客户端的命令和请求。
- 客户端发现服务器的数据库并向服务器发送命令和请求。
- GATT 定义了客户端和服务器之间发现、读取、写入、通知和指示以交换数据的过程。

一个属性数据由4部分组成：
- 属性句柄，是指定属性的索引。 该值是从 0x0001 到 0xFFFF。
- 属性类型通过位/128位uuid对属性句柄的描述。 uuid 可以是 SIG 分配的或用户自定义的。
- 属性值，是属性的数据。
- 属性权限，指示属性是否可以读取或写入。

![](../../../assets/att_format.png)

基于属性，服务框架层次结构为：配置文件、服务、包含服务和特征。 
- 配置文件是由一个或多个服务组成的高级概念。 它定义了访问服务的行为。
- 服务由几个包括服务和特征组成。 它定义了数据格式和相关的行为。
- 包含服务是指服务器上存在的其他服务定义。
- 特征描述详细数据格式和行为。 它包含特征声明、特征值和特征描述符。

![](../../../assets/gatt_hierarchy.png)

以下是电池服务的示例。 电池服务是通知客户电池更换。 因此，该服务具有具有可读性和通知属性的电池电量特性。
客户端可以编写客户端特征配置描述符（CCCD）以启用通知。 然后电池服务将通知电池电量。

![](../../../assets/att_example.png)




## 实现GATT服务。

Sibles GATT 服务提供了可以由工具生成的格式化 API。 用户还可以按照以下步骤通过 API 自定义 GATT 服务：
1. 构建 GATT 服务并注册到 SIBLEs GATT 服务。 该服务将自动从蓝牙堆栈中保存用户 GATT 服务的分配 GATT 句柄。
1. 注册设置/获取回调以响应客户端命令。 SIBLEs GATT 服务将使用指定的句柄通知用户客户端请求。

以下是电池服务的示例：

```c

// Battery Service Attributes Indexes which are mapping to GATT handle.
enum
{
    BAS_IDX_SVC,

    BAS_IDX_BATT_LVL_CHAR,
    BAS_IDX_BATT_LVL_VAL,
    BAS_IDX_BATT_LVL_NTF_CFG,

    BAS_IDX_NB,
};

typedef enum
{
    BASS_STATE_IDLE,
    BASS_STATE_READY,
    BASS_STATE_BUSY,
} ble_bass_state_t;



// Full BAS Database Description - Used to add attributes into the database
const struct attm_desc bas_att_db[BAS_IDX_NB] =
{
    // Battery Service Declaration
    [BAS_IDX_SVC]                  =   {ATT_DECL_PRIMARY_SERVICE,  PERM(RD, ENABLE), 0, 0},

    // Battery Level Characteristic Declaration
    [BAS_IDX_BATT_LVL_CHAR]        =   {ATT_DECL_CHARACTERISTIC,   PERM(RD, ENABLE), 0, 0},
    // Battery Level Characteristic Value
    [BAS_IDX_BATT_LVL_VAL]         =   {ATT_CHAR_BATTERY_LEVEL,    PERM(RD, ENABLE) | PERM(NTF, ENABLE), PERM(RI, ENABLE), 0},
    // Battery Level Characteristic - Client Characteristic Configuration Descriptor
    [BAS_IDX_BATT_LVL_NTF_CFG]     =   {ATT_DESC_CLIENT_CHAR_CFG,  PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), 0, 0},
};


typedef struct
{
    ble_bass_callback callback;
    sibles_hdl handle;
    uint8_t state;
    uint8_t cccd_enable;
    uint8_t bas_lvl;
} ble_bass_env_t;

static ble_bass_env_t g_bass_env_t;

static ble_bass_env_t *ble_bass_get_env(void)
{
    return &g_bass_env_t;
}

// Read callback for specfied index.
static uint8_t *ble_bass_get_cbk(uint8_t conn_idx, uint8_t idx, uint16_t *len)
{
    ble_bass_env_t *env = ble_bass_get_env();
    switch (idx)
    {
    case BAS_IDX_BATT_LVL_VAL:
    {
        *len = sizeof(uint8_t);
        if (env->callback)
            env->bas_lvl = env->callback(conn_idx, BLE_BASS_GET_BATTERY_LVL);
        rt_kprintf("battery lvl %d", env->bas_lvl);
        return &env->bas_lvl;
        break;
    }
    default:
        break;
    }
    *len = 0;
    return NULL;
}

// Write callback for specfied index.
static uint8_t ble_bass_set_cbk(uint8_t conn_idx, sibles_set_cbk_t *para)
{
    ble_bass_env_t *env = ble_bass_get_env();
    switch (para->idx)
    {
    case BAS_IDX_BATT_LVL_NTF_CFG:
    {
        rt_kprintf("bas enable %d", *(para->value));
        env->cccd_enable = *(para->value);
        break;
    }
    default:
        break;
    }
    return 0;
}

int8_t ble_bass_notify_battery_lvl(uint8_t conn_idx, uint8_t lvl)
{
    ble_bass_env_t *env = ble_bass_get_env();
    uint8_t ret = -1;
    if (env->state == BASS_STATE_READY)
    {
        if (env->bas_lvl != lvl)
        {
            env->bas_lvl = lvl;
            sibles_value_t value;
            value.hdl = env->handle;
            value.idx = BAS_IDX_BATT_LVL_VAL;
            value.len = sizeof(uint8_t);
            value.value = &env->bas_lvl;
            int ret = sibles_write_value(conn_idx, &value);
            ret = 0;
        }
        ret = -2;
    }
    return ret;
}



void ble_bass_init(ble_bass_callback callback, uint8_t battery_lvl)
{
    ble_bass_env_t *env = ble_bass_get_env();

    if (env->state == BASS_STATE_IDLE)
    {
        sibles_register_svc_t svc;
		
		// Provided battery database.
        svc.att_db = (struct attm_desc *)&bas_att_db
        svc.num_entry = BAS_IDX_NB;
        svc.sec_lvl = PERM(SVC_AUTH, NO_AUTH);
        svc.uuid = ATT_SVC_BATTERY_SERVICE;
        env->handle = sibles_register_svc(&svc);
        if (env->handle)
            sibles_register_cbk(env->handle, ble_bass_get_cbk, ble_bass_set_cbk);  // Register read/write callback to respond client access.
        env->state = BASS_STATE_READY;
    }
    env->bas_lvl = battery_lvl;
    env->callback = callback;
}


```

## 消息流

- 立即阅读回复

![](../../../assets/gatt_read_response_realtime.png)

- 读取响应预设

![](../../../assets/gatt_read_response_preset.png)

- 写回复

![](../../../assets/gatt_write.png)

- 指示

![](../../../assets/gatt_indication.png)
