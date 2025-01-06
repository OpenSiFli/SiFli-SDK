#include "charge.h"
#include "rtthread.h"
#include "bf0_hal.h"
#include <stdlib.h>
#include <string.h>


#define DBG_TAG "sifli_charge"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>
#define CHARGE_DEFAULT_CV_VCTRL     (0x27)
#define CHARGE_DEFAULT_BANDGAP_VREF (0x7)
#define CHARGE_DEFAULT_CC_MN        (0x8)
#define CHARGE_DEFAULT_CC_MP        (0x14)

#define CHARGE_DEBUG
#define CHARGE_DEBUG_IRQ_INTERVAL  (10000)


typedef struct
{
    uint32_t last_irq_time;
    uint32_t cur_irq_time;
    uint32_t old_pmu_irq;
    uint32_t new_pmu_irq;
    uint32_t pmu_irq_count;
    uint8_t  force_enable;
    uint8_t  vbat_high;
} sifli_charge_debug_t;

static rt_charge_device_t charge_device;
static PMU_ChgHandleTypeDef sifli_charge_handle;

#ifdef CHARGE_DEBUG
sifli_charge_debug_t sifli_charge_debug = {0};
#endif

#if defined(CHARGE_MONITOR_TEMP_CONFIG)
    static TSEN_HandleTypeDef    TsenHandle;
#endif

rt_err_t sifli_charge_control(rt_charge_device_t *charge, int cmd, void *args)
{
    rt_charge_err_t ret = RT_CHARGE_EOK;

    switch (cmd)
    {
    case RT_CHARGE_GET_STATUS:
    {
        uint8_t *status = (uint8_t *)args;
        *status = HAL_PMU_ChgReadStatus(&sifli_charge_handle, PMU_CHG_IRQ_VBUS_RDY);

    }
    break;

    case RT_CHARGE_GET_DETECT_STATUS:
    {
        uint8_t *status = (uint8_t *)args;
        *status = HAL_PMU_ChgReadStatus(&sifli_charge_handle, PMU_CHG_IRQ_VBUS_RDY);
    }
    break;

    case RT_CHARGE_GET_FULL_STATUS:
    {
        uint8_t *status = (uint8_t *)args;
        *status = HAL_PMU_ChgReadStatus(&sifli_charge_handle, PMU_CHG_IRQ_EOC);
    }
    break;

    case RT_CHARGE_ENABLE:
    {
        uint8_t *enable = (uint8_t *)args;
        HAL_PMU_ChgEnable(&sifli_charge_handle, *enable);
    }
    break;
#if defined(CHARGE_MONITOR_TEMP_CONFIG)
    case RT_CHARGE_GET_CORE_TEMPERATURE:
    {
        int *p_temp = (int *)args;
        *p_temp = HAL_TSEN_Read(&TsenHandle);
    }
    break;
#endif

    case RT_CHARGE_FORCE_ENABLE_CHARGING:
    {
        uint8_t *enable = (uint8_t *)args;
        if (*enable)
        {
            HAL_PMU_ChgStartForceCharging(&sifli_charge_handle);
        }
        else
        {
            HAL_PMU_ChgStopForceCharging(&sifli_charge_handle);
        }

#ifdef CHARGE_DEBUG
        sifli_charge_debug.force_enable = *enable;
#endif

    }
    break;

    case RT_CHARGE_FORCE_SUSPEND_CHARGING:
    {
        HAL_PMU_ChgSuspendForceCharging(&sifli_charge_handle);
    }
    break;

    case RT_CHARGE_FORCE_RESUME_CHARGING:
    {
        HAL_PMU_ChgResumeForceCharging(&sifli_charge_handle);
    }
    break;

    case RT_CHARGE_GET_HW_STATE:
    {
        uint8_t *status = (uint8_t *)args;
        *status = HAL_PMU_ChgGetHwState(&sifli_charge_handle);
        LOG_I("hw_state: %x", *status);
    }
    break;

    case RT_CHARGE_SET_CC_CURRENT:
    {
        uint32_t *current = (uint32_t *)args;
        HAL_PMU_ChgConfigCcCurrent(&sifli_charge_handle, *current);
    }
    break;

    case RT_CHARGE_SET_PRECC_CURRENT:
    {
        float *current = (float *)args;
        HAL_PMU_ChgConfigPreCcCurrent(&sifli_charge_handle, *current);
    }
    break;

    case RT_CHARGE_SET_TARGET_VOLT:
    {
        uint32_t *volt = (uint32_t *)volt;
        HAL_PMU_ChgConfigTargetVolt(&sifli_charge_handle, *volt);
    }
    break;

    case RT_CHARGE_SET_REPVOLT:
    {
        uint32_t *volt = (uint32_t *)volt;
        HAL_PMU_ChgConfigRepVolt(&sifli_charge_handle, *volt);
    }
    break;

    case RT_CHARGE_SET_OVER_VOLT:
    {
        uint32_t *volt = (uint32_t *)volt;
        HAL_PMU_ChgConfigVbatHighVolt(&sifli_charge_handle, *volt);
    }
    break;

    default:
        ret = RT_CHARGE_ERROR_UNSUPPORTED;
        break;
    }
    return ret;
}



static void sifli_charge_callback(PMU_ChgHandleTypeDef *handle, uint32_t status)
{
#ifdef CHARGE_DEBUG
    sifli_charge_debug.cur_irq_time = rt_tick_get();
    sifli_charge_debug.new_pmu_irq = status;
    if (sifli_charge_debug.new_pmu_irq != sifli_charge_debug.old_pmu_irq)
    {
        sifli_charge_debug.pmu_irq_count = 1;
    }
    else
    {

        uint32_t time_diff = (sifli_charge_debug.cur_irq_time - sifli_charge_debug.last_irq_time
                              + UINT32_MAX + 1) & UINT32_MAX;

        sifli_charge_debug.pmu_irq_count++;

        if (time_diff > CHARGE_DEBUG_IRQ_INTERVAL)
            sifli_charge_debug.pmu_irq_count = 1;

    }
    sifli_charge_debug.old_pmu_irq = sifli_charge_debug.new_pmu_irq;
    sifli_charge_debug.last_irq_time = sifli_charge_debug.cur_irq_time;
#endif
    if (status & (1 << PMU_CHG_IRQ_VBUS_RDY))
    {
        LOG_I("vbus: %d", HAL_PMU_ChgReadStatus(handle, PMU_CHG_IRQ_VBUS_RDY));
        rt_charge_event_notify(RT_CHARGE_EVENT_DETECT);
#ifdef CHARGE_DEBUG
        sifli_charge_debug.vbat_high = 0;
#endif
    }

    if (status & (1 << PMU_CHG_IRQ_EOC))
    {
        LOG_I("eoc: %d", HAL_PMU_ChgReadStatus(handle, PMU_CHG_IRQ_EOC));
        rt_charge_event_notify(RT_CHARGE_EVENT_END);
    }

    if (status & (1 << PMU_CHG_IRQ_VBAT_HIGH))
    {
        HAL_PMU_ChgStopForceCharging(&sifli_charge_handle);
        HAL_PMU_ChgEnable(&sifli_charge_handle, false);
        //HAL_PMU_ChgSuspendForceCharging(&sifli_charge_handle);
        //HAL_PMU_ChgStopForceCharging(&sifli_charge_handle);

#ifdef CHARGE_DEBUG
        sifli_charge_debug.vbat_high = 1;
#endif
        LOG_I("vbat high: %d", HAL_PMU_ChgReadStatus(handle, PMU_CHG_IRQ_VBAT_HIGH));
    }

    if (status & (1 << PMU_CHG_IRQ_ABOVE_REP))
    {
        LOG_I("above rep: %d", HAL_PMU_ChgReadStatus(handle, PMU_CHG_IRQ_ABOVE_REP));
    }

    if (status & (1 << PMU_CHG_IRQ_CV))
    {
        LOG_I("cv: %d", HAL_PMU_ChgReadStatus(handle, PMU_CHG_IRQ_CV));
    }

}


void PMUC_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_PMU_IRQHandler(&sifli_charge_handle);
    rt_interrupt_leave();
}


static const struct rt_charge_ops sifli_charge_ops =
{
    .control = sifli_charge_control
};


void sifli_config_charge_irq(void)
{
    HAL_StatusTypeDef status;
    HAL_NVIC_DisableIRQ(PMUC_IRQn);
    HAL_PMU_ChgRegisterCallback(&sifli_charge_handle, sifli_charge_callback);
    status = HAL_PMU_ChgConfigIRQ(&sifli_charge_handle, PMU_CHG_IRQ_VBUS_RDY, PMU_CHG_IRQ_TRIG_MODE_DOUBLE_EDGE);
    RT_ASSERT(HAL_OK == status);
    status = HAL_PMU_ChgConfigIRQ(&sifli_charge_handle, PMU_CHG_IRQ_VBAT_HIGH, PMU_CHG_IRQ_TRIG_MODE_POS_EDGE);
    RT_ASSERT(HAL_OK == status);
    status = HAL_PMU_ChgConfigIRQ(&sifli_charge_handle, PMU_CHG_IRQ_EOC, PMU_CHG_IRQ_TRIG_MODE_POS_EDGE);
    RT_ASSERT(HAL_OK == status);
    status = HAL_PMU_ChgConfigIRQ(&sifli_charge_handle, PMU_CHG_IRQ_ABOVE_REP, PMU_CHG_IRQ_TRIG_MODE_DISABLE);
    RT_ASSERT(HAL_OK == status);
    //status = HAL_PMU_ChgConfigIRQ(&pmu_chg_handle, PMU_CHG_IRQ_CV, PMU_CHG_IRQ_TRIG_MODE_DOUBLE_EDGE);
    //RT_ASSERT(HAL_OK == status);

    HAL_NVIC_SetPriority(PMUC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(PMUC_IRQn);

    return;
}

void sifli_charge_init(void)
{
    PMU_ChgCalParamTypeDef cal_param;
    FACTORY_CFG_CHARGER_T cfg;
    int len = sizeof(FACTORY_CFG_CHARGER_T);

    cal_param.cc_mn = CHARGE_DEFAULT_CC_MN;
    cal_param.cc_mp = CHARGE_DEFAULT_CC_MP;
    cal_param.cv_vctrl = CHARGE_DEFAULT_CV_VCTRL;
    cal_param.bg = CHARGE_DEFAULT_BANDGAP_VREF;
    cal_param.rep_vctrl = CHARGE_DEFAULT_CV_VCTRL - 7;

    if (BSP_CONFIG_get(FACTORY_CFG_ID_CHARGER, (uint8_t *)&cfg, len))
    {
        cal_param.cc_mn = cfg.cc_mn;
        cal_param.cc_mp = cfg.cc_mp;
        cal_param.cv_vctrl = cfg.cv_vctrl;
        cal_param.bg = cfg.prog_v1p2;
        cal_param.rep_vctrl = cfg.cv_vctrl - 7;
        rt_kprintf("cal data cc_mn:%d cc_mp:%d cv_vctrl:%d prog_vlp2:%d\n", cfg.cc_mn, cfg.cc_mp, cfg.cv_vctrl, cfg.prog_v1p2);
    }
    HAL_PMU_ChgInit(&sifli_charge_handle, &cal_param);
    //  HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_PMUC, AON_PIN_MODE_DOUBLE_EDGE);
    HAL_PMU_EnableChgWakeup();
    HAL_PMU_ChgConfigTargetVolt(&sifli_charge_handle, CHARGE_DEFAULT_FULL_VOLTAGE);
    HAL_PMU_ChgConfigRepVolt(&sifli_charge_handle, CHARGE_DEFAULT_REP_VOLTAGE);
    HAL_PMU_ChgConfigVbatHighVolt(&sifli_charge_handle, CHARGE_DEFAULT_VBAT_HIGH_VOLTAGE);
    HAL_PMU_ChgConfigCcCurrent(&sifli_charge_handle, CHARGE_CC_CURRENT_LEVEL0);
#if defined(CHARGE_MONITOR_TEMP_CONFIG)
    HAL_TSEN_Init(&TsenHandle);
#endif
    return;
}

int sifli_charge_device_init(void)
{
    sifli_charge_init();
#if !defined (DFU_OTA_MANAGER)
    sifli_config_charge_irq();
#endif
    rt_charge_register(&charge_device, &sifli_charge_ops, RT_NULL);
    return RT_EOK;
}

INIT_PREV_EXPORT(sifli_charge_device_init);


#ifdef RT_USING_FINSH

static int charger(int argc, char *argv[])
{
    char *cmd;
    HAL_StatusTypeDef status;

    if (argc < 2)
    {
        goto __EXIT;
    }

    cmd = argv[1];

    if (0 == strcmp("init", cmd))
    {
        PMU_ChgCalParamTypeDef cal_param;

        if (argc < 7)
        {
            goto __EXIT;
        }
        cal_param.cc_mn = atoi(argv[2]);
        cal_param.cc_mp = atoi(argv[3]);
        cal_param.bg = atoi(argv[4]);
        cal_param.cv_vctrl = atoi(argv[5]);
        cal_param.rep_vctrl = atoi(argv[6]);

        status = HAL_PMU_ChgInit(&sifli_charge_handle, &cal_param);
        RT_ASSERT(HAL_OK == status);
        LOG_I("init done");
    }
    else if (0 == strcmp("cc_i", cmd))
    {
        uint32_t current;

        if (argc < 3)
        {
            goto __EXIT;
        }
        current = atoi(argv[2]);
        current = HAL_PMU_ChgConfigCcCurrent(&sifli_charge_handle, current);
        LOG_I("config cc current: %d", current);
    }
    else if (0 == strcmp("precc_i", cmd))
    {
        float current;

        if (argc < 3)
        {
            goto __EXIT;
        }
        current = atof(argv[2]);
        current = HAL_PMU_ChgConfigPreCcCurrent(&sifli_charge_handle, current);
        LOG_I("config precc current: %.1f", current);
    }
    else if (0 == strcmp("tgt_volt", cmd))
    {
        uint32_t volt;

        if (argc < 3)
        {
            goto __EXIT;
        }
        volt = atoi(argv[2]);
        volt = HAL_PMU_ChgConfigTargetVolt(&sifli_charge_handle, volt);
        LOG_I("config target voltage: %d", volt);
    }
    else if (0 == strcmp("rep_volt", cmd))
    {
        uint32_t volt;

        if (argc < 3)
        {
            goto __EXIT;
        }
        volt = atoi(argv[2]);
        volt = HAL_PMU_ChgConfigRepVolt(&sifli_charge_handle, volt);
        LOG_I("config rep voltage: %d", volt);
    }
    else if (0 == strcmp("high_volt", cmd))
    {
        uint32_t volt;

        if (argc < 3)
        {
            goto __EXIT;
        }
        volt = atoi(argv[2]);
        volt = HAL_PMU_ChgConfigVbatHighVolt(&sifli_charge_handle, volt);
        LOG_I("config rep voltage: %d", volt);
    }
    else if (0 == strcmp("enable", cmd))
    {
        uint32_t enable;

        if (argc < 3)
        {
            goto __EXIT;
        }
        enable = atoi(argv[2]);
        HAL_PMU_ChgEnable(&sifli_charge_handle, enable);
        LOG_I("config enable: %d", enable);
    }
    else if (0 == strcmp("force_enable", cmd))
    {
        uint32_t enable;

        if (argc < 3)
        {
            goto __EXIT;
        }
        enable = atoi(argv[2]);
        if (enable)
        {
            HAL_PMU_ChgStartForceCharging(&sifli_charge_handle);
        }
        else
        {
            HAL_PMU_ChgStopForceCharging(&sifli_charge_handle);
        }

        LOG_I("config force enable: %d", enable);
    }
    else if (0 == strcmp("list_cal", cmd))
    {
        uint32_t enable;

        if (argc < 2)
        {
            goto __EXIT;
        }

        FACTORY_CFG_CHARGER_T cfg;
        int len = sizeof(FACTORY_CFG_CHARGER_T);
        if (BSP_CONFIG_get(FACTORY_CFG_ID_CHARGER, (uint8_t *)&cfg, len))  // TODO: configure read charger parameters method after ATE confirm
        {
            LOG_I("cal data cc_mn:%d cc_mp:%d cv_vctrl:%d prog_vlp2:%d", cfg.cc_mn, cfg.cc_mp, cfg.cv_vctrl, cfg.prog_v1p2);
        }
        else
        {
            LOG_I("read cal data err");
        }
    }
    else if (0 == strcmp("suspend", cmd))
    {
        uint32_t enable;

        if (argc < 2)
        {
            goto __EXIT;
        }
        HAL_PMU_ChgSuspendForceCharging(&sifli_charge_handle);
        LOG_I("suspend");
    }
    else if (0 == strcmp("resume", cmd))
    {
        uint32_t enable;

        if (argc < 2)
        {
            goto __EXIT;
        }
        HAL_PMU_ChgResumeForceCharging(&sifli_charge_handle);
        LOG_I("resume", enable);
    }
    else if (0 == strcmp("hw_state", cmd))
    {
        uint32_t state = 0;

        if (argc < 2)
        {
            goto __EXIT;
        }
        state = HAL_PMU_ChgGetHwState(&sifli_charge_handle);
        LOG_I("hw_state:%x", state);
    }

    return 0;
__EXIT:
    rt_kprintf("wrong arg\n");
    return -1;
}
MSH_CMD_EXPORT(charger, "Test Charger")

#endif

