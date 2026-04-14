/*
 * SPDX-FileCopyrightText: 2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "rtdevice.h"
#include "charge.h"
#include "board.h"
#include "drv_gpio.h"
#include "stdlib.h" 
#ifdef BSP_USING_PM
    #include "bf0_pm.h"
#endif
static struct rt_i2c_bus_device *aw32001_i2c_bus;

//#define AW32001_DEBUG
static rt_charge_device_t charge_device;
#define AW32001_I2C_ADDRESS 0x49
#define AW32001_REG_SOURCE_CTRL                     0x00
#define AW32001_REG_POWERON_CONF                    0x01
#define AW32001_REG_CHARGE_CURRENT_CTRL             0x02
#define AW32001_REG_DISCHARGE_CURRENT_CTRL          0x03
#define AW32001_REG_CHARGE_VOLTAGE                  0x04
#define AW32001_REG_TIMER_CTRL                      0x05
#define AW32001_REG_MAIN_CTRL                       0x06
#define AW32001_REG_VOLT_CTRL                       0x07
#define AW32001_REG_SYS_STATUS                      0x08
#define AW32001_REG_FAULT                           0x09
#define AW32001_REG_ADDRESS                         0x0A
#define AW32001_REG_INDIVIDUAL_CHARGE               0x0B
#define AW32001_REG_ADD_FUNC_0                      0x0C
#define AW32001_REG_ADD_FUNC_1                      0x22

#define AW32001_NO_CHARGING                         0
#define AW32001_PRE_CHARGING                        1
#define AW32001_CHARGING                            2
#define AW32001_CHARG_FULL                          3

uint8_t aw32001_get_charge_enable()
{
    uint8_t reg = 0, ret = 0;
    rt_i2c_mem_read(aw32001_i2c_bus, AW32001_I2C_ADDRESS, AW32001_REG_POWERON_CONF, 8, &reg, 1);
    reg = (reg & (1 << 3)) >> 3;
    if (!reg)
        ret = 1;
    return ret;
}

void aw32001_charge_enable(uint8_t en)
{
    uint8_t reg;

    rt_i2c_mem_read(aw32001_i2c_bus, AW32001_I2C_ADDRESS, AW32001_REG_POWERON_CONF, 8, &reg, 1);
    if (en)
        reg &= ~(1 << 3);   // clear bit3  CEB: 0 charge enable  1 charge disable(default)
    else
        reg |= (1 << 3);    //set bit3 CEB: 0 charge enable  1 charge disable(default)
    rt_i2c_mem_write(aw32001_i2c_bus, AW32001_I2C_ADDRESS, AW32001_REG_POWERON_CONF, 8, &reg, 1);
}
void aw32001_wdt_enable(uint8_t en)
{
    uint8_t reg;
    rt_i2c_mem_read(aw32001_i2c_bus, AW32001_I2C_ADDRESS, AW32001_REG_TIMER_CTRL, 8, &reg, 1);
    if (en)
        reg |= 3 << 5;      //enable wdt
    else
        reg &= ~(3 << 5);   //disable wdt
    rt_i2c_mem_write(aw32001_i2c_bus, AW32001_I2C_ADDRESS, AW32001_REG_TIMER_CTRL, 8, &reg, 1);
}
int aw32001_get_detect_status()
{
    int status = 0xff;
#ifdef BSP_CHARGER_INT_PIN_ACTIVE_HIGH
    status = rt_pin_read(BSP_CHARGER_INT_PIN);
#else
    status = !rt_pin_read(BSP_CHARGER_INT_PIN);
#endif
    return status;
}

uint32_t aw32001_get_battery_voltage(void)
{
    rt_device_t dev = rt_device_find(BSP_BATTERY_DETECT_ADC);
    if (NULL == dev)
        rt_kprintf("%s failed\n", __func__);
    rt_adc_enable((rt_adc_device_t) dev, BSP_BATTERY_DETECT_ADC_CHANNEL);

    HAL_LCPU_CONFIG_BATTERY_T battery_para = {0};
    uint16_t len = (uint16_t)sizeof(HAL_LCPU_CONFIG_BATTERY_T);

    uint32_t voltage = rt_adc_read((rt_adc_device_t) dev, BSP_BATTERY_DETECT_ADC_CHANNEL);
#if defined(AW32001_DEBUG)
    rt_kprintf("rt_adc_read:  %d;\n", voltage);
#endif
#if defined(SF32LB55X)
    voltage = ((voltage * 1220) / 220) / 10;
#elif defined(SF32LB52X)
    voltage /= 10;
#else
    voltage = ((voltage * 1470) / 1000) / 10;
#endif

#ifndef SF32LB52X
    if (HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_BATTERY_CALIBRATION, (uint8_t *)&battery_para, &len) == 0)
    {
        if (battery_para.a >= 9000 && battery_para.a <= 11000)
        {
            voltage = (voltage * battery_para.a + 5000) / 10000 + battery_para.b;
        }
    }
#endif
#if defined(AW32001_DEBUG)
    rt_kprintf("%s: volt %d \n", __func__, voltage);
#endif
    rt_adc_disable((rt_adc_device_t) dev, BSP_BATTERY_DETECT_ADC_CHANNEL);
    return voltage;
}

bool aw32001_get_charge_status(uint8_t *status)
{
    uint8_t reg;
    rt_size_t size = 0;
    size = rt_i2c_mem_read(aw32001_i2c_bus, AW32001_I2C_ADDRESS, AW32001_REG_SYS_STATUS, 8, &reg, 1);
    if (size < 1)
    {
        rt_kprintf("aw32001_get_charge_status i2c read 1st fail,try agin!\n");
        size = rt_i2c_mem_read(aw32001_i2c_bus, AW32001_I2C_ADDRESS, AW32001_REG_SYS_STATUS, 8, &reg, 1);
        if (size < 1)
        {
            rt_kprintf("aw32001_get_charge_status i2c read 2ed fail, exit!\n");
            return RT_FALSE;
        }
    }
    *status = (reg & 0x18) >> 3;  //get bit3~bit4
    return RT_TRUE;
}
bool aw32001_get_fault_status(uint8_t *status)
{
    uint8_t reg;
    rt_size_t size = 0;

    size = rt_i2c_mem_read(aw32001_i2c_bus, AW32001_I2C_ADDRESS, AW32001_REG_FAULT, 8, &reg, 1);
    if (size < 1)
    {
        rt_kprintf("aw32001_get_fault_status i2c read 1st fail,try agin!\n");
        size = rt_i2c_mem_read(aw32001_i2c_bus, AW32001_I2C_ADDRESS, AW32001_REG_FAULT, 8, &reg, 1);
        if (size < 1)
        {
            rt_kprintf("aw32001_get_fault_status i2c read 2ed fail, exit!\n");
            return RT_FALSE;
        }
    }
    *status = reg;
    return RT_TRUE;
}


rt_err_t aw32001_set_charge_current(uint16_t set_chg)
{
    int8_t reg;
    uint16_t current;
    rt_charge_err_t ret = RT_CHARGE_EOK;
    rt_size_t size = 0;
    size = rt_i2c_mem_read(aw32001_i2c_bus, AW32001_I2C_ADDRESS, AW32001_REG_CHARGE_CURRENT_CTRL, 8, &reg, 1);
    if (size < 1)
    {
        rt_kprintf("aw32001_set_charge_current i2c read fail!\n");
        return RT_CHARGE_ERROR_UNSUPPORTED;
    }
    current = ((reg & 0x3f) + 1) * 8;
#if defined(AW32001_DEBUG)
    rt_kprintf("%s:old charge current = %d;\n", __func__, current);
#endif
    if (set_chg > 512)
        return RT_CHARGE_ERROR_UNSUPPORTED;
    reg = (reg & 0xC0) | (set_chg / 8);
#if defined(AW32001_DEBUG)
    rt_kprintf("%s:set current reg= %d;\n", __func__, reg);
#endif
    rt_i2c_mem_write(aw32001_i2c_bus, AW32001_I2C_ADDRESS, AW32001_REG_CHARGE_CURRENT_CTRL, 8, &reg, 1);
    return RT_CHARGE_EOK;
}

rt_err_t aw32001_get_target_volt(uint32_t *target_volt)
{
    int8_t reg;
    rt_charge_err_t ret = RT_CHARGE_EOK;
    rt_i2c_mem_read(aw32001_i2c_bus, AW32001_I2C_ADDRESS, AW32001_REG_CHARGE_VOLTAGE, 8, &reg, 1);
    *target_volt = 3600 + (reg >> 2) * 15;
#if defined(AW32001_DEBUG)
    rt_kprintf("%s:target volt = %d;\n", __func__, *target_volt);
#endif
    return ret;
}

rt_err_t aw32001_set_target_volt(uint32_t set_volt)
{
    int8_t reg;
    uint32_t volt;
    rt_charge_err_t ret = RT_CHARGE_EOK;
    rt_i2c_mem_read(aw32001_i2c_bus, AW32001_I2C_ADDRESS, AW32001_REG_CHARGE_VOLTAGE, 8, &reg, 1);
    volt = 3600 + (reg >> 2) * 15;
#if defined(AW32001_DEBUG)
    rt_kprintf("%s:old target volt = %d;\n", __func__, volt);
#endif
    if ((set_volt > 4545) || (set_volt < 3600))
        return RT_CHARGE_ERROR_UNSUPPORTED;

    reg = ((uint8_t)((set_volt - 3600) / 15) << 2) | (reg & 0x03);
#if defined(AW32001_DEBUG)
    rt_kprintf("%s:set target volt reg= %d;\n", __func__, reg);
#endif
    rt_i2c_mem_write(aw32001_i2c_bus, AW32001_I2C_ADDRESS, AW32001_REG_CHARGE_VOLTAGE, 8, &reg, 1);
    return ret;
}


rt_err_t aw32001_control(rt_charge_device_t *charge, int cmd, void *args)
{
    rt_charge_err_t ret = RT_CHARGE_EOK;

    switch (cmd)
    {
    case RT_CHARGE_GET_STATUS:
    {
        uint8_t *status = (uint8_t *)args;

        uint8_t aw32001_status;
        uint8_t aw32001_fault;
        if ((!aw32001_get_charge_status(&aw32001_status)) || (!aw32001_get_fault_status(&aw32001_fault)))
        {
            ret = RT_CHARGE_ERROR_UNSUPPORTED;
            break;
        }
        rt_kprintf("aw32001_status = %d, fault = 0x%x;\n", aw32001_status, aw32001_fault);
#ifdef CHARGE_NO_BATTERY
        if (CHARGE_NO_BATTERY)
        {
            *status = 0;
            break;
        }
#endif
        if (aw32001_fault & 0x08)   //battery fault
        {

            aw32001_charge_enable(0);
            rt_thread_mdelay(1);
            aw32001_charge_enable(1);

            *status = 0;
            break;
        }

        if (aw32001_status == AW32001_NO_CHARGING)
        {
            *status = 0;
        }
        else
        {
            *status = 1;
        }
    }
    break;

    case RT_CHARGE_GET_DETECT_STATUS:
    {
        uint8_t *status = (uint8_t *)args;
        if (BSP_CHARGER_INT_PIN >= 0)
        {
            int detect_status = aw32001_get_detect_status();
            if (detect_status != 0xff)
                *status = detect_status;
        }
        else
            ret = RT_CHARGE_ERROR_UNSUPPORTED;
    }
    break;

    case RT_CHARGE_GET_FULL_STATUS:
    {
        uint8_t *status = (uint8_t *)args;
        uint8_t aw32001_status;
        if (!aw32001_get_charge_status(&aw32001_status))
        {
            ret = RT_CHARGE_ERROR_UNSUPPORTED;
            break;
        }
        if (aw32001_status == AW32001_CHARG_FULL)
        {
            *status = 1;
        }
        else
        {
            *status = 0;
        }
    }
    break;

    case RT_CHARGE_ENABLE:
    {
        uint8_t *enable = (uint8_t *)args;
        aw32001_charge_enable(*enable);
    }
    break;

    case RT_CHARGE_SET_CC_CURRENT:
    {
        uint32_t *current = (uint32_t *)args;
        ret = aw32001_set_charge_current((uint16_t) * current);
    }
    break;
    case RT_CHARGE_SET_TARGET_VOLT:
    {
        uint32_t *target_volt = (uint32_t *)args;
        aw32001_set_target_volt(*target_volt);
    }
    break;
    case RT_CHARGE_FORCE_SUSPEND_CHARGING:
    {
        rt_kprintf("aw32001 suspend charging;\n");
        aw32001_charge_enable(0);

    }
    break;
    case RT_CHARGE_FORCE_RESUME_CHARGING:
    {
        rt_kprintf("aw32001 resume charging;\n");
        aw32001_charge_enable(1);
    }
    break;
    default:
        ret = RT_CHARGE_ERROR_UNSUPPORTED;
        break;
    }
    return ret;
}

static const struct rt_charge_ops aw32001_ops =
{
    .control = aw32001_control
};


void aw32001_input_handle(void *args)
{
    static uint32_t last_time;

    uint32_t cur_time = rt_tick_get();
    uint32_t tick = (cur_time - last_time + UINT32_MAX + 1) & UINT32_MAX;

    if (tick > 150)
    {
        rt_charge_event_notify(RT_CHARGE_EVENT_DETECT);
    }

    last_time = cur_time;
    return;

}
static rt_err_t aw32001_I2C_Init(const char *name)
{
    /* get i2c bus device */
    aw32001_i2c_bus = rt_i2c_bus_device_find(name);
    if (aw32001_i2c_bus)
    {
#if defined(AW32001_DEBUG)
        rt_kprintf("aw32001 Find i2c bus device %s\n", name);
#endif
    }
    else
    {
        rt_kprintf("aw32001 Can not found i2c bus %s, init fail\n", name);
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t aw32001_hw_init(void)
{
#ifdef BSP_BATTERY_USE_I2C_BUS
    if (aw32001_I2C_Init(BSP_BATTERY_USE_I2C_BUS) != RT_EOK)
    {
        rt_kprintf("aw32001 i2c init fail!\n");
        return -RT_ERROR;
    }
#endif
    aw32001_wdt_enable(false);
    if (!aw32001_get_charge_enable())
    {
#ifdef RT_USING_PM
        rt_pm_request(PM_SLEEP_MODE_IDLE);
#endif
        aw32001_charge_enable(1);
        rt_thread_mdelay(50);   /*if disable to enable, should delay 50ms for voltage rising.*/
#ifdef RT_USING_PM
        rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif

    }
    aw32001_set_target_volt(4215);
#ifdef CHARGE_NO_BATTERY
    if (CHARGE_NO_BATTERY)
    {
#ifdef BSP_ENABLE_AUD_PRC
        extern void bf0_audprc_set_max_call_dac_vol(int8_t vol);
        extern void bf0_audprc_set_max_music_dac_vol(int8_t vol);
        bf0_audprc_set_max_call_dac_vol(-30);
        bf0_audprc_set_max_music_dac_vol(-30);
#endif
    }
#endif


#if defined(BSP_USING_CHARGER_DETECT)
#if defined(BSP_USING_PM)
    GPIO_TypeDef *gpio = GET_GPIO_INSTANCE(BSP_CHARGER_INT_PIN);
    uint16_t gpio_pin = GET_GPIOx_PIN(BSP_CHARGER_INT_PIN);
    int8_t wakeup_pin;
    if (BSP_CHARGER_INT_PIN > 96)
        wakeup_pin = HAL_LPAON_QueryWakeupPin(gpio, gpio_pin);
    else
        wakeup_pin = HAL_HPAON_QueryWakeupPin(gpio, gpio_pin);

    RT_ASSERT(wakeup_pin >= 0);
    pm_enable_pin_wakeup(wakeup_pin, AON_PIN_MODE_DOUBLE_EDGE);


#ifndef BSP_PM_STANDBY_SHUTDOWN
    /* CHARGE DETECT PIN attach PMUC PIN1, for hibernate wake*/
#ifdef PMUC_CR_PIN0_SEL
    if (BSP_CHARGER_INT_PIN < GPIO1_PIN_NUM)
    {
        HAL_PMU_SelectWakeupPin(1, HAL_HPAON_QueryWakeupPin(hwp_gpio1, BSP_CHARGER_INT_PIN));
    }
    else
    {
        HAL_PMU_SelectWakeupPin(1, HAL_LPAON_QueryWakeupPin(hwp_gpio2, BSP_CHARGER_INT_PIN - GPIO1_PIN_NUM));
    }
    HAL_PMU_EnablePinWakeup(1, AON_PIN_MODE_DOUBLE_EDGE);

    /*set wakeup count*/
    uint32_t pin1_wkup_cnt = 1;
    MODIFY_REG(hwp_pmuc->WKUP_CNT, PMUC_WKUP_CNT_PIN1_CNT_Msk,
               MAKE_REG_VAL(pin1_wkup_cnt, PMUC_WKUP_CNT_PIN1_CNT_Msk, PMUC_WKUP_CNT_PIN1_CNT_Pos));
#endif
#endif

#endif/* BSP_USING_PM */

    rt_pin_mode(BSP_CHARGER_INT_PIN, PIN_MODE_INPUT);

#if !defined (DFU_OTA_MANAGER)  //avoid ota triggers charging interruption to clear WSR before entering app
    // enable LSM int
    rt_pin_attach_irq(BSP_CHARGER_INT_PIN, PIN_IRQ_MODE_RISING_FALLING, (void *) aw32001_input_handle,
                      (void *)(rt_uint32_t) BSP_CHARGER_INT_PIN);
    rt_pin_irq_enable(BSP_CHARGER_INT_PIN, 1);
#endif

#endif

    return RT_EOK;

}

int aw32001_init(void)
{
    uint32_t pre_tick = 0, tick = 0;
    float time = 0.0;
    tick = HAL_GTIMER_READ();
    pre_tick = tick;

    if (aw32001_hw_init() != RT_EOK)
    {
        rt_kprintf("aw32001 init fail!\n");
        return -RT_ERROR;
    }
    rt_charge_register(&charge_device, &aw32001_ops, RT_NULL);

    tick = HAL_GTIMER_READ();
    tick = tick - pre_tick;
    time = tick / HAL_LPTIM_GetFreq();
    rt_kprintf("aw32001_init ok,  time consum: %f ms.\n", time);
    return RT_EOK;
}
INIT_PREV_EXPORT(aw32001_init);


static void get_cc_volt(int argc, char **argv)
{
    uint32_t current_voltage = aw32001_get_battery_voltage();
    uint32_t target_voltage = 0;
    uint8_t reg;
    rt_err_t ret = aw32001_get_target_volt(&target_voltage);

    /* Read the original value of the SYS_STATUS register and parse bits 4:3. */
    uint8_t sys_reg = 0;
    rt_size_t size = rt_i2c_mem_read(aw32001_i2c_bus, AW32001_I2C_ADDRESS, AW32001_REG_SYS_STATUS, 8, &sys_reg, 1);
    if (size < 1)
    {
        rt_kprintf("read SYS_STATUS fail\n");
    }
    else
    {
        uint8_t state = (sys_reg & 0x18) >> 3; /* First mask then right shift to get bit4:3 */
        rt_kprintf("SYS_STATUS raw=0x%02X (%d), state(bits4:3)=%d\n", sys_reg, sys_reg, state);
        rt_kprintf("state mapping: 0=no charging, 1=pre-charge, 2=charging, 3=charge full\n");
    }
}
MSH_CMD_EXPORT(get_cc_volt, get battery voltage);

#if defined(RT_USING_FINSH)
#include "string.h"
int32_t aw32001(int32_t argc, char **argv)
{
    if (argc < 3)
    {
        rt_kprintf("Wrong argument\n");
    }
    if (strcmp(argv[1], "charge") == 0)
    {
        uint8_t en = atoi(argv[2]);
        if (en)
            aw32001_charge_enable(true);
        else
            aw32001_charge_enable(false);

    }
#ifdef CHARGE_NO_BATTERY
    else if (strcmp(argv[1], "have_bat") == 0)
    {
        uint8_t is_present = 1;
        if (CHARGE_NO_BATTERY)
            is_present = 0;
        int8_t vol = 0;
#ifdef BSP_ENABLE_AUD_PRC
        extern int8_t bf0_audprc_get_max_call_dac_vol(void);
        vol = bf0_audprc_get_max_call_dac_vol();
#endif
        rt_kprintf("battery_is_present = %d, max_call_dac_vol = %d\n", is_present, vol);

    }
#endif
    else if (strcmp(argv[1], "set_cc") == 0)
    {
        uint16_t current = atoi(argv[2]);
        aw32001_set_charge_current(current);
    }
    else if (strcmp(argv[1], "status") == 0)
    {
        uint8_t status;
        rt_charge_get_status(&status);
        rt_kprintf("charge status =%d;\n", status);
    }
    else if (strcmp(argv[1], "fault") == 0)
    {
        uint8_t fault;
        if (!aw32001_get_fault_status(&fault))
        {
            rt_kprintf("read fault reg fail;\n");
            return 0;
        }
        rt_kprintf("fault =%d;\n", fault);
    }
    return 0;
}

MSH_CMD_EXPORT(aw32001, aw32001 test     cmd);
#endif

