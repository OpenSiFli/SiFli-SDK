#include "app_common.h"

#define LOG_TAG "main_app"
#include "log.h"


#ifdef BT_FINSH
    #include "bts2_app_interface.h"
#endif


static app_env_t g_app_env;
static rt_mailbox_t g_app_mb;

#if defined(BSP_USING_SPI_NAND) && defined(RT_USING_DFS)
#include "dfs_file.h"
#include "dfs_posix.h"
#include "drv_flash.h"
#define NAND_MTD_NAME    "root"
int mnt_init(void)
{
    //TODO: how to get base address
    register_nand_device(FS_REGION_START_ADDR & (0xFC000000), FS_REGION_START_ADDR - (FS_REGION_START_ADDR & (0xFC000000)), FS_REGION_SIZE, NAND_MTD_NAME);
    if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0) // fs exist
    {
        LOG_I("mount fs on flash to root success\n");
    }
    else
    {
        // auto mkfs, remove it if you want to mkfs manual
        LOG_I("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", NAND_MTD_NAME) == 0)
        {
            LOG_I("make elm fs on flash sucess, mount again\n");
            if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0)
                LOG_I("mount fs on flash success\n");
            else
                LOG_I("mount to fs on flash fail\n");
        }
        else
            LOG_I("dfs_mkfs elm flash fail\n");
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);
#endif

#ifdef LXT_DISABLE
void rc10k_timeout_handler(void *parameter)
{
    app_env_t *env = ble_app_get_env();
    if (!HAL_RTC_LXT_ENABLED())
    {
        HAL_RC_CAL_update_reference_cycle_on_48M(LXT_LP_CYCLE);
    }
    else
    {
        rt_timer_stop(env->rc10k_time_handle);
    }
}
#endif

#ifdef RT_USING_PM
static void app_wakeup_src(void)
{
    uint8_t pin = 0;
#if defined(SOC_SF32LB56X) || defined(SOC_SF32LB58X)
    HPAON_WakeupSrcTypeDef src = pin + HPAON_WAKEUP_SRC_PIN6;
#elif defined(SOC_SF32LB52X)
//    HPAON_WakeupSrcTypeDef src = pin + HPAON_WAKEUP_SRC_PIN8;
#else
    HPAON_WakeupSrcTypeDef src = pin + HPAON_WAKEUP_SRC_PIN12;
#endif
}

static int sleep(int argc, char **argv)
{
    char i;
    if (argc > 1)
    {
        if (strcmp("standby", argv[1]) == 0)
        {
            rt_kprintf("sleep on\r\n");
            rt_pm_release(PM_SLEEP_MODE_IDLE);
        }
        else if (strcmp("off", argv[1]) == 0)
        {
            rt_kprintf("sleep off\r\n");
            rt_pm_request(PM_SLEEP_MODE_IDLE);
        }
        else if (strcmp("down", argv[1]) == 0)
        {
            /* Changes PA24 to pulldown to avoid leakage if it's connected to GND */
            HAL_PIN_Set(PAD_PA24, GPIO_A24, PIN_PULLDOWN, 1);
            for (uint32_t i = PAD_PA28; i <= PAD_PA44; i++)
            {
                HAL_PIN_Set(i, i - PAD_PA28 + GPIO_A28, PIN_PULLDOWN, 1);
            }
            //disable ldo
#ifndef SF32LB52X_58
            hwp_pmuc->PERI_LDO &=  ~(PMUC_PERI_LDO_EN_LDO18 | PMUC_PERI_LDO_EN_VDD33_LDO2 | PMUC_PERI_LDO_EN_VDD33_LDO3);
#endif

            rt_kprintf("entry_hibernate\r\n");
            rt_hw_interrupt_disable();
            HAL_PMU_EnterHibernate();
            while (1) {};
        }
        else
        {
            rt_kprintf("sleep err\r\n");
        }
    }
    return 0;
}
MSH_CMD_EXPORT(sleep, forward sleep command); /* 导出到 msh 命令列表中 */
#endif //RT_USING_PM

// For debugging HAL, change to #if 1
#if 0
void HAL_DBG_printf(const char *fmt, ...)
{
    va_list args;
    static char rt_log_buf[128];
    extern void rt_kputs(const char *str);

    va_start(args, fmt);
    rt_vsnprintf(rt_log_buf, sizeof(rt_log_buf) - 1, fmt, args);
    rt_kputs(rt_log_buf);
    rt_kputs("\r\n");
    va_end(args);
}
#endif

app_env_t *ble_app_get_env(void)
{
    return &g_app_env;
}

int main(void)
{
    int count = 0;
#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);

    app_wakeup_src();
#endif  /* RT_USING_PM */
    app_env_t *env = ble_app_get_env();
    env->mb_handle = rt_mb_create("app", 8, RT_IPC_FLAG_FIFO);

    // Initialize BT/BLE stack/profile
    spp_init();
    ble_init();
    sifli_ble_enable();

#ifdef LXT_DISABLE
    env->rc10k_time_handle  = rt_timer_create("rc10", rc10k_timeout_handler,  NULL,
                              rt_tick_from_millisecond(15 * 1000), RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER); // 15s
    rt_timer_start(env->rc10k_time_handle);
#endif
    /*lptim pwm init*/
#if defined(BSP_USING_PWM_LPTIM2)

    extern void lptimer_pwm_init();
    lptimer_pwm_init();
    extern void lptimer_pwm_start();
    lptimer_pwm_start();
#endif
    while (1)
    {
        uint32_t value;
        int ret;
        rt_mb_recv(env->mb_handle, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
        if (value == BLE_POWER_ON_IND)
        {
            // LCPU is powered on, start BLE ADV
            env->is_power_on = 1;
            env->conn_para.mtu = 23; /* Default value. */
            ble_app_service_init();
            /* First enable connectable adv then enable non-connectable. */
            ble_app_advertising_start();
            LOG_I("receive BLE power on!\r\n");
        }
    }
    return RT_EOK;
}
#if defined(SF32LB52X_58)|| (defined(SF32LB52X) && (defined(SF32LB52X_REV_B) || defined(SF32LB52X_REV_AUTO)))
uint16_t g_em_offset[HAL_LCPU_CONFIG_EM_BUF_MAX_NUM] =
{
    0x178, 0x178, 0x740, 0x7A0, 0x810, 0x880, 0xA00, 0xBB0, 0xD48,
    0x133C, 0x13A4, 0x19BC, 0x21BC, 0x21BC, 0x21BC, 0x21BC, 0x21BC, 0x21BC,
    0x21BC, 0x21BC, 0x263C, 0x265C, 0x2734, 0x2784, 0x28D4, 0x28E8, 0x28FC,
    0x29EC, 0x29FC, 0x2BBC, 0x2BD8, 0x3BE8, 0x5804, 0x5804, 0x5804
};

void lcpu_rom_config(void)
{
    uint8_t is_config_allowed = 0;
#ifdef SF32LB52X
    uint8_t rev_id = __HAL_SYSCFG_GET_REVID();
    if (rev_id >= HAL_CHIP_REV_ID_A4)
        is_config_allowed = 1;
#elif defined(SF32LB52X_58)
    is_config_allowed = 1;
#endif

    extern void lcpu_rom_config_default(void);
    lcpu_rom_config_default();

    if (is_config_allowed)
    {
        hal_lcpu_bluetooth_em_config_t em_offset;
        memcpy((void *)em_offset.em_buf, (void *)g_em_offset, HAL_LCPU_CONFIG_EM_BUF_MAX_NUM * 2);
        em_offset.is_valid = 1;
        HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_EM_BUF, &em_offset, sizeof(hal_lcpu_bluetooth_em_config_t));

        hal_lcpu_bluetooth_act_configt_t act_cfg;
        act_cfg.ble_max_act = 6;
        act_cfg.ble_max_iso = 0;
        act_cfg.ble_max_ral = 3;
        act_cfg.bt_max_acl = 7;
        act_cfg.bt_max_sco = 0;
        act_cfg.bit_valid = CO_BIT(0) | CO_BIT(1) | CO_BIT(2) | CO_BIT(3) | CO_BIT(4);
        HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_ACT_CFG, &act_cfg, sizeof(hal_lcpu_bluetooth_act_configt_t));
    }
}
#endif
