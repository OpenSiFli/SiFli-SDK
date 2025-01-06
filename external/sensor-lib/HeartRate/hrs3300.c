#include <stdio.h>
#include <stdbool.h>
#include "stdlib.h"
#include "sdk_config.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
//////////////////////////////
#include "hrs3300.h"
#include "hrs3300_reg_init.h"

#include "nrf_log.h"


#include "board_config.h"
#include "kct_config.h"

#include "dal_hrs.h"
#include "hal_iic.h"

#if ( (KCT_HRS_ENABLE == 1) && (KCT_CUR_HRS_DEV == HRS_HRS3300_DEV))

#define KCT_HRS_LOG                     NRF_LOG_INFO

//////////////////////////////
#define GSENSER_DATA
#define BP_CUSTDOWN_ALG_LIB

#include <stdbool.h>

MONITOR_S  Monitor_para;

// hrs3300 customer config
const uint8_t   hrs3300_bp_timeout_grade = 0;  // max 15
const uint8_t   hrs3300_agc_init_stage = 0x04;  // init AGC state
const uint8_t   hrs3300_bp_power_grade = 0;
const uint8_t   hrs3300_accurate_first_shot = 0;
const uint8_t   hrs3300_up_factor = 3;
const uint8_t   hrs3300_up_shift = 2;
const uint16_t  hrs3300_AMP_LTH = 120;
const uint16_t  hrs3300_hr_AMP_LTH = 150;
const uint16_t  hrs3300_hr_PVAR_LTH = 10;
// hrs3300 customer config end

//20161117 added by ericy for "low power in no_touch state"
static bool hrs3300_power_up_flg = 0 ;
uint8_t reg_0x7f ;
uint8_t reg_0x80 ;
uint8_t reg_0x81 ;
uint8_t reg_0x82 ;
//20161117 added by ericy for "low power in no_touch state"

uint8_t Oxygen_Process(uint8_t m_HRF);

bool Hrs3300_write_reg(uint8_t addr, uint8_t data)
{
    // I2c custom  return  0(success), 1(fail)
    uint8_t data_buf[2];

    data_buf[0] = addr;
    data_buf[1] = data;
    hal_i2c_transfer((IIC_DRIVER_NUM)HRS_IIC_NUM, 0x88, data_buf, 2, true);
    return 0;
}

uint8_t Hrs3300_read_reg(uint8_t addr)
{
    uint8_t data_buf;
    hal_i2c_transfer((IIC_DRIVER_NUM)HRS_IIC_NUM, 0x88, &addr, 1, false);
    hal_i2c_transfer((IIC_DRIVER_NUM)HRS_IIC_NUM, 0x89, &data_buf, 1, true);
    return data_buf;
}


uint16_t Hrs3300_read_hrs(void)
{
    uint8_t  databuf[3];
    uint16_t data;

    databuf[0] = Hrs3300_read_reg(0x09);    // addr09, bit
    databuf[1] = Hrs3300_read_reg(0x0a);    // addr0a, bit
    databuf[2] = Hrs3300_read_reg(0x0f);    // addr0f, bit

    data = ((databuf[0] << 8) | ((databuf[1] & 0x0F) << 4) | (databuf[2] & 0x0F));

    return data;
}

uint16_t Hrs3300_read_als(void)
{
    uint8_t  databuf[3] = {0};
    uint16_t data;

    databuf[0] = Hrs3300_read_reg(0x08);    // addr09, bit [10:3]
    databuf[1] = Hrs3300_read_reg(0x0d);    // addr0a, bit [17:11]
    databuf[2] = Hrs3300_read_reg(0x0e);    // addr0f, bit [2:0]

    data = ((databuf[0] << 3) | ((databuf[1] & 0x3F) << 11) | (databuf[2] & 0x07));

    if (data > 32767) data = 32767;  // prevent overflow of other function

    return data;
}

void hrs3300_gpio_init(void)
{
    nrf_gpio_cfg_output(HRS_HRD_SW_PIN);
    nrf_gpio_pin_clear(HRS_HRD_SW_PIN);
    nrf_delay_ms(20);
    nrf_gpio_pin_set(HRS_HRD_SW_PIN);
    nrf_delay_ms(20);
}

void hrs3300_gpio_clear(void)
{
    nrf_gpio_cfg_output(HRS_HRD_SW_PIN);
    nrf_gpio_pin_clear(HRS_HRD_SW_PIN);
}

bool Hrs3300_chip_init()
{
    int i = 0 ;
    uint8_t id = 0;
//#ifdef RTT_PRINT
    KCT_HRS_LOG(">>> hrs3300 init \r\n");
//#endif
    hrs3300_gpio_init();

    for (i = 0; i < INIT_ARRAY_SIZE; i++)
    {
        if (Hrs3300_write_reg(init_register_array[i][0], init_register_array[i][1]) != 0)
        {
            goto RTN;
        }
    }

    //20161117 added by ericy for "low power in no_touch state"
    if (hrs3300_power_up_flg == 0)
    {
        reg_0x7f = Hrs3300_read_reg(0x7f) ;
        reg_0x80 = Hrs3300_read_reg(0x80) ;
        reg_0x81 = Hrs3300_read_reg(0x81) ;
        reg_0x82 = Hrs3300_read_reg(0x82) ;
        hrs3300_power_up_flg =  1;
    }
    //20161117 added by ericy for "low power in no_touch state"
    id = Hrs3300_read_reg(0x00); //id = Hrs3300_read_reg(0x00);
//#ifdef RTT_PRINT
    KCT_HRS_LOG("<<< hrs3300 init done id = %d \r\n", id); // 0x21
//#endif

    Hrs3300_chip_disable();
    return true;
RTN:
    return false;
}

void Hrs3300_chip_enable()
{
    Hrs3300_write_reg(0x16, 0x78);
    Hrs3300_write_reg(0x01, 0xd0);
    Hrs3300_write_reg(0x0c, 0x2e);

    return ;
}

void Hrs3300_chip_disable(void)
{
    Hrs3300_write_reg(0x01, 0x08);
    Hrs3300_write_reg(0x02, 0x80);
    Hrs3300_write_reg(0x0c, 0x4e);

    Hrs3300_write_reg(0x16, 0x88);

    Hrs3300_write_reg(0x0c, 0x22);
    Hrs3300_write_reg(0x01, 0xf0);
    Hrs3300_write_reg(0x0c, 0x02);

    Hrs3300_write_reg(0x0c, 0x22);
    Hrs3300_write_reg(0x01, 0xf0);
    Hrs3300_write_reg(0x0c, 0x02);

    Hrs3300_write_reg(0x0c, 0x22);
    Hrs3300_write_reg(0x01, 0xf0);
    Hrs3300_write_reg(0x0c, 0x02);

    Hrs3300_write_reg(0x0c, 0x22);
    Hrs3300_write_reg(0x01, 0xf0);
    Hrs3300_write_reg(0x0c, 0x02);

    return ;
}

void heartrate_meas_timeout_handler(void *p_context)
{
    uint16_t hrm_raw_data;
    uint16_t als_raw_data;
    hrs3300_results_t alg_results;
#ifdef BP_CUSTDOWN_ALG_LIB
    hrs3300_bp_results_t    bp_alg_results ;
#endif
    static uint16_t timer_index = 0;

    hrm_raw_data = Hrs3300_read_hrs();
    als_raw_data = Hrs3300_read_als();  // 20170430

    Hrs3300_alg_send_data(hrm_raw_data, als_raw_data, 0, 0, 0, 0);

    timer_index ++;
    if (timer_index >= 25)      // get result per second
    {
        timer_index = 0;
        alg_results = Hrs3300_alg_get_results();
        KCT_HRS_LOG("hrs = %d , statu:%d\r\n", alg_results.hr_result, alg_results.alg_status);
        if (alg_results.alg_status == MSG_NO_TOUCH)
        {
            Monitor_para.hrs = 0;
            Monitor_para.mmghH = 0;
            Monitor_para.mmghL = 0;
            Monitor_para.sop2 = 0;
        }
        else if (alg_results.alg_status == MSG_PPG_LEN_TOO_SHORT)
        {
            Monitor_para.hrs = 0;
            Monitor_para.mmghH = 0;
            Monitor_para.mmghL = 0;
            Monitor_para.sop2 = 0;
        }
        else
        {
#ifdef BP_CUSTDOWN_ALG_LIB
            bp_alg_results = Hrs3300_alg_get_bp_results();
            if (bp_alg_results.sbp != 0)
            {
                //    opr_display_bp(bp_alg_results.sbp, bp_alg_results.dbp);
            }
#endif

            if (alg_results.object_flg == 1)
            {

            }
            Monitor_para.hrs = alg_results.hr_result;

            if (bp_alg_results.sbp != 0) //(bp_alg_results.bp_alg_status == MSG_BP_READY)
            {
                Monitor_para.mmghH = bp_alg_results.sbp ;
                Monitor_para.mmghL = bp_alg_results.dbp ;
            }

            Monitor_para.sop2 = Oxygen_Process(alg_results.hr_result);      //97+alg_results.hr_result%10/4;
        }
    }
}

void Get_User_heart_value(uint8_t *value)
{
    *value = Monitor_para.hrs ;// Usert_Health_para.heart  ;
}

void Get_User_blood_value(uint8_t *bloodl, uint8_t *bloodh)
{
    *bloodl = Monitor_para.mmghL;
    *bloodh = Monitor_para.mmghH;
}

void Get_User_spo2_value(uint8_t *value)
{
    *value = Monitor_para.sop2 ;
}

void  Open_alg_hrs(void)
{
    Hrs3300_chip_init();
    Hrs3300_chip_enable();

    Hrs3300_alg_open();
}


uint8_t Oxygen_Process(uint8_t m_HRF)
{
    const static    uint8_t spo2h_buff0[6] = {88, 89, 90, 91, 92, 93};
    const static    uint8_t spo2h_buff1[4] = {94, 95, 96, 97};
    const static    uint8_t spo2h_buff2[3] = {97, 98, 99};
    uint8_t spo2h = 0;

    uint8_t num = rand() % 10;
    if (m_HRF < 45)
    {
        spo2h =  0;
    }
    else if (m_HRF < 50)
    {
        spo2h = spo2h_buff0[0];
    }
    else if (m_HRF < 60)
    {
        num = num % 6;
        spo2h = spo2h_buff0[num];
    }
    else if (m_HRF < 70)
    {
        num = num % 4;
        spo2h = spo2h_buff1[num];
    }
    else if (m_HRF <= 100)
    {
        num = num % 3;
        spo2h = spo2h_buff2[num];
    }
    else
        spo2h = spo2h_buff2[2];

    return  spo2h;
}


void close_hrs3300(void)
{
    Hrs3300_alg_close();
    Hrs3300_chip_disable();
    nrf_delay_ms(10);
}

ret_code_t hrs_drv_data(HRS_DATA_TYPE type, void *data)
{
    ret_code_t err_code = NRF_SUCCESS;
    uint8_t   *press;

    switch (type)
    {
    case HRS_HEARTRATE:
        Get_User_heart_value((uint8_t *)data);
        break;
    case HRS_BLOODPRESS:
        press = (uint8_t *)data;
        Get_User_blood_value(press, press++);
        break;
    case HRS_BLOODOXYGEN:
        Get_User_spo2_value((uint8_t *)data);
        break;
    }
    return err_code;
}

ret_code_t hrs_drv_close(void)
{
    ret_code_t err_code = NRF_SUCCESS;

    close_hrs3300();
    return err_code;
}

ret_code_t hrs_drv_open(HRS_DATA_TYPE type)
{
    ret_code_t err_code = NRF_SUCCESS;

    KCT_HRS_LOG("hrs_drv_open(%d)", type);

    switch (type)
    {
    case HRS_HEARTRATE:
        Open_alg_hrs();
        break;
    case HRS_BLOODPRESS:
        Open_alg_hrs();
        break;
    case HRS_BLOODOXYGEN:
        Open_alg_hrs();
        break;
    }

    return err_code;
}

ret_code_t hrs_drv_updata(void)
{
    ret_code_t err_code = NRF_SUCCESS;

    heartrate_meas_timeout_handler(0);

    return err_code;
}

ret_code_t hrs_drv_init(void)
{
    ret_code_t err_code = NRF_SUCCESS;

    Hrs3300_chip_init();

    return err_code;
}

const kct_hrs_t kct_hrs_dev =
{
    .hrs_init       = hrs_drv_init,
    .hrs_open       = hrs_drv_open,
    .hrs_close      = hrs_drv_close,
    .hrs_get_data   = hrs_drv_data,
    .hrs_updata     = hrs_drv_updata,
};

#endif
