
# Sibles 广播

Sibles 广播提供基于 GAP 广播 API 的简单 API。 用户可以配置部分参数然后可以开始广播。
- 配置广播模式以确定广播行为。
    - SIBLES_ADV_CONNECT_MODE，启动可连接广播以建立链接。
    - SIBLES_ADV_BROADCAST_MODE，启动不可连接的广播广播。
    - SIBLES_ADV_DIRECTED_CONNECT_MODE，为专用链接建立启动定向广播。
- 为每种模式配置参数，例如间隔、持续时间和是否重复。
- 将广播数据和扫描响应数据设置为提供的结构。
	
这是示例：

```c

// Declare app advertising context
SIBLES_ADVERTISING_CONTEXT_DECLAR(g_app_advertising_context);

// Listen the advertising status changed and request.
static uint8_t app_advertising_event(uint8_t event, void *context, void *data)
{
    switch (event)
    {
        case SIBLES_ADV_EVT_ADV_STARTED:
        {
			// Check current started advertising mode
            sibles_adv_evt_startted_t *evt = (sibles_adv_evt_startted_t *)data;
            rt_kprintf("ADV start resutl %d, mode %d\r\n", evt->status, evt->adv_mode);
            break;
        }
        case SIBLES_ADV_EVT_ADV_STOPPED:
        {
			// Check current stopped advertising mode
            sibles_adv_evt_stopped_t *evt = (sibles_adv_evt_stopped_t *)data;
            rt_kprintf("ADV stopped reason %d, mode %d\r\n", evt->reason, evt->adv_mode);
            break;
        }
        default:
            break;
    }
		return 0;
}


static void app_advertising_start(void)
{
    sibles_advertising_para_t para = {0};
    char local_name[] = "TEST_SIFLI";
    uint8_t manu_additnal_data[] = {0x20, 0xC4, 0x00, 0x91};
    uint16_t manu_company_id = 0x01;
    uint8_t ret;

    para.own_addr_type = GAPM_GEN_RSLV_ADDR;
	// Configure advertising mode
    para.config.adv_mode = SIBLES_ADV_CONNECT_MODE;
	// Configure mode parameters
    para.config.mode_config.conn_config.duration = 3000;
    para.config.mode_config.conn_config.interval = 0x30;
	// Support two advertising interval switch.
    para.config.mode_config.conn_config.backgroud_mode_enabled = 0x1;
    para.config.mode_config.conn_config.backgroud_duration = 6000;
    para.config.mode_config.conn_config.backgroud_interval = 0x200;
	// Whether repeated if duration timeout
    para.config.mode_config.conn_config.is_repeated = 1;
	// Whether restart after disconnection.
    para.config.is_auto_restart = 1;
	// Whether advertising and scan response use same data
    para.config.is_rsp_data_duplicate = 1;

	// Set local name in advertising data
    para.adv_data.completed_name = rt_malloc(rt_strlen(local_name) + sizeof(sibles_adv_type_name_t));
    para.adv_data.completed_name->name_len = rt_strlen(local_name);
    rt_memcpy(para.adv_data.completed_name->name, local_name, para.adv_data.completed_name->name_len);

	// Set manufacturer data in advertising data
    para.adv_data.manufacturer_data = rt_malloc(sizeof(sibles_adv_type_manufacturer_data_t) + sizeof(manu_additnal_data));
    para.adv_data.manufacturer_data->company_id = manu_company_id;
    para.adv_data.manufacturer_data->data_len = sizeof(manu_additnal_data);
    rt_memcpy(para.adv_data.manufacturer_data->additional_data, manu_additnal_data, sizeof(manu_additnal_data));


    para.evt_handler = app_advertising_event;
    
    ret = sibles_advertising_init(g_app_advertising_context, &para);
    if (ret == SIBLES_ADV_NO_ERR)
    {
        sibles_advertising_start(g_app_advertising_context);
    }

}


```