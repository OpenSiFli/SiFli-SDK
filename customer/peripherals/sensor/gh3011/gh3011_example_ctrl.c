/**
 * @copyright (c) 2003 - 2020, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3011_example_ctrl.c
 *
 * @brief   example code for gh3011 (condensed  hbd_ctrl lib)
 *
 */

#include "gh3011_example_common.h"
#if (__USE_GOODIX_APP__)
    #include "hbd_communicate.h"
#endif


/// debug log string
#if (__EXAMPLE_DEBUG_LOG_LVL__) // debug level > 0

/// dbg run mode string
const char dbg_rum_mode_string[][24] =
{
    "RUN_MODE_INVALID\0",
    "RUN_MODE_INVALID\0",
    "RUN_MODE_ADT_HB_DET\0",
    "RUN_MODE_HRV_DET\0",
    "RUN_MODE_HSM_DET\0",
    "RUN_MODE_BPD_DET\0",
    "RUN_MODE_PFA_DET\0",
    "RUN_MODE_SPO2_DET\0",
};
/// dbg communicate cmd string
const char dbg_comm_cmd_string[][35] =
{
    "COMM_CMD_ALGO_IN_MCU_HB_START\0",
    "COMM_CMD_ALGO_IN_MCU_HB_STOP\0",
    "COMM_CMD_ALGO_IN_APP_HB_START\0",
    "COMM_CMD_ALGO_IN_APP_HB_STOP\0",
    "COMM_CMD_ALGO_IN_MCU_HRV_START\0",
    "COMM_CMD_ALGO_IN_MCU_HRV_STOP\0",
    "COMM_CMD_ALGO_IN_APP_HRV_START\0",
    "COMM_CMD_ALGO_IN_APP_HRV_STOP\0",
    "COMM_CMD_ADT_SINGLE_MODE_START\0",
    "COMM_CMD_ADT_SINGLE_MODE_STOP\0",
    "COMM_CMD_ADT_CONTINUOUS_MODE_START\0",
    "COMM_CMD_ADT_CONTINUOUS_MODE_STOP\0",
    "COMM_CMD_ALGO_IN_MCU_SPO2_START\0",
    "COMM_CMD_ALGO_IN_MCU_SPO2_STOP\0",
    "COMM_CMD_ALGO_IN_APP_SPO2_START\0",
    "COMM_CMD_ALGO_IN_APP_SPO2_STOP\0",
    "COMM_CMD_INVALID\0",
};
/// dbg ret val string
const char dbg_ret_val_string[][35] =
{
    "HBD_RET_LED_CONFIG_ALL_OFF_ERROR\0",
    "HBD_RET_NO_INITED_ERROR\0",
    "HBD_RET_RESOURCE_ERROR\0",
    "HBD_RET_COMM_ERROR\0",
    "HBD_RET_COMM_NOT_REGISTERED_ERROR\0",
    "HBD_RET_PARAMETER_ERROR\0",
    "HBD_RET_GENERIC_ERROR\0",
    "HBD_RET_OK\0",
};

#endif


/// gh30x load new config
void gh30x_Load_new_config(const ST_REGISTER *config_ptr, uint16_t len)
{
    uint8_t index = 0;
    for (index = 0; index < __RETRY_MAX_CNT_CONFIG__; index ++) // retry
    {
        if (HBD_LoadNewRegConfigArr(config_ptr, len) == HBD_RET_OK)
        {
            break;
        }
        EXAMPLE_DEBUG_LOG_L1("gh30x load new config error\r\n");
    }
}

/// gh30x adt func start
void gh30x_adt_wear_detect_start(const ST_REGISTER *config_ptr, uint16_t config_len)
{
    uint8_t index = 0;
    HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_STOP();
    if ((config_ptr != NULL) & (config_len != 0))
    {
        gh30x_Load_new_config(config_ptr, config_len);
        if (HBD_AdtWearDetectStart() != HBD_RET_OK)  // start
        {
            EXAMPLE_DEBUG_LOG_L1("gh30x adt start error\r\n");
            for (index = 0; index < __RETRY_MAX_CNT_CONFIG__; index ++) // retry
            {
                if (HBD_SimpleInit(&gh30x_init_config) == HBD_RET_OK)
                {
                    if (HBD_LoadNewRegConfigArr(config_ptr, config_len) == HBD_RET_OK)
                    {
                        if (HBD_AdtWearDetectStart() == HBD_RET_OK)
                        {
                            EXAMPLE_DEBUG_LOG_L1("HBD_AdtWearDetectStart return OK\n");
                            break;
                        }
                    }
                }
            }
        }
    }
}

/// gh30x adt confirm func start
void gh30x_adt_confirm_start(void)
{
    uint8_t index = 0;

    HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_STOP();
    HBD_AdtConfirmConfig(__HB_ADT_CONFIRM_GS_AMP__, __HB_ADT_CONFIRM_CHECK_CNT__, __HB_ADT_CONFIRM_THR_CNT__);

    if (HBD_AdtConfirmStart() != HBD_RET_OK)  // start
    {
        EXAMPLE_DEBUG_LOG_L1("gh30x adt confirm start error\r\n");
        for (index = 0; index < __RETRY_MAX_CNT_CONFIG__; index ++) // retry
        {
            if (HBD_SimpleInit(&gh30x_init_config) == HBD_RET_OK)
            {
                if (HBD_LoadNewRegConfigArr(hb_adt_confirm_reg_config, hb_adt_confirm_reg_config_len) == HBD_RET_OK)
                {
                    if (HBD_AdtConfirmStart() == HBD_RET_OK)
                    {
                        HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
                        break;
                    }
                }
            }
        }
    }
    else
    {
        HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
    }
}

#if (__HB_DET_SUPPORT__)
/// gh30x hb func start
void gh30x_hb_start(void)
{
    uint8_t index = 0;

    HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_STOP();

#if (__HBA_ENABLE_WEARING__)
    GF32 wearing_config_array[3] = {0, 0, 0};
    HBD_EnableWearing(wearing_config_array);
#endif

    HBD_SetFifoThrCnt(FIFO_THR_CONFIG_TYPE_HB, __HB_FIFO_THR_CNT_CONFIG__);

    if (HBD_HbDetectStart() != HBD_RET_OK)  // start
    {
        EXAMPLE_DEBUG_LOG_L1("gh30x hb start error\r\n");
        for (index = 0; index < __RETRY_MAX_CNT_CONFIG__; index ++) // retry
        {
            if (HBD_SimpleInit(&gh30x_init_config) == HBD_RET_OK)
            {
                if (HBD_LoadNewRegConfigArr(hb_reg_config_array, hb_reg_config_array_len) == HBD_RET_OK)
                {
                    if (HBD_HbDetectStart() == HBD_RET_OK)
                    {
                        EXAMPLE_DEBUG_LOG_L1("HBD_HbDetectStart OK");
                        HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
                        break;
                    }
                }
            }
        }
    }
    else
    {
        HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
    }
}

#endif

/// gh30x wear confirm func start
void gh30x_wear_confirm_start(void)
{
    uint8_t index = 0;

    HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_STOP();

    gh30x_Load_new_config(hb_wear_confirm_reg_config_array, hb_wear_confirm_reg_config_array_len);
    if (HBD_WearStateConfirmStart() != HBD_RET_OK)  // start
    {
        EXAMPLE_DEBUG_LOG_L1("gh30x wear confirm start error\r\n");
        for (index = 0; index < __RETRY_MAX_CNT_CONFIG__; index ++) // retry
        {
            if (HBD_SimpleInit(&gh30x_init_config) == HBD_RET_OK)
            {
                if (HBD_LoadNewRegConfigArr(hb_wear_confirm_reg_config_array, hb_wear_confirm_reg_config_array_len) == HBD_RET_OK)
                {
                    if (HBD_WearStateConfirmStart() == HBD_RET_OK)
                    {
                        HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
                        break;
                    }
                }
            }
        }
    }
    else
    {
        HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
    }
}

#if (__HRV_DET_SUPPORT__)
/// gh30x hrv func start
void gh30x_hrv_start(void)
{
    uint8_t index = 0;

    HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_STOP();

    HBD_SetFifoThrCnt(FIFO_THR_CONFIG_TYPE_HRV, __HRV_FIFO_THR_CNT_CONFIG__);

    // gh30x_Load_new_config(hrv_reg_config_array, hrv_reg_config_array_len);
    if (HBD_HrvDetectStart() != HBD_RET_OK)  // start
    {
        EXAMPLE_DEBUG_LOG_L1("gh30x hrv start error\r\n");
        for (index = 0; index < __RETRY_MAX_CNT_CONFIG__; index ++) // retry
        {
            if (HBD_SimpleInit(&gh30x_init_config) == HBD_RET_OK)
            {
                if (HBD_LoadNewRegConfigArr(hrv_reg_config_array, hrv_reg_config_array_len) == HBD_RET_OK)
                {
                    if (HBD_HrvDetectStart() == HBD_RET_OK)
                    {
                        HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
                        break;
                    }
                }
            }
        }
    }
    else
    {
        HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
    }
}

#endif

#if (__HSM_DET_SUPPORT__)
/// gh30x hsm func start
void gh30x_hsm_start(void)
{
    uint8_t index = 0;

    HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_STOP();

    HBD_SetFifoThrCnt(FIFO_THR_CONFIG_TYPE_HSM, __HSM_FIFO_THR_CNT_CONFIG__);

    // gh30x_Load_new_config(hsm_reg_config_array, hsm_reg_config_array_len);
    if (HBD_HsmDetectStart() != HBD_RET_OK)  // start
    {
        EXAMPLE_DEBUG_LOG_L1("gh30x hsm start error\r\n");
        for (index = 0; index < __RETRY_MAX_CNT_CONFIG__; index ++) // retry
        {
            if (HBD_SimpleInit(&gh30x_init_config) == HBD_RET_OK)
            {
                if (HBD_LoadNewRegConfigArr(hsm_reg_config_array, hsm_reg_config_array_len) == HBD_RET_OK)
                {
                    if (HBD_HsmDetectStart() == HBD_RET_OK)
                    {
                        HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
                        break;
                    }
                }
            }
        }
    }
    else
    {
        HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
    }
}

#endif

#if (__BPD_DET_SUPPORT__)
/// gh30x bpd func start
void gh30x_bpd_start(void)
{
    uint8_t index = 0;

    HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_STOP();

    HBD_SetFifoThrCnt(FIFO_THR_CONFIG_TYPE_BPD, __BPD_FIFO_THR_CNT_CONFIG__);

    // gh30x_Load_new_config(bpd_reg_config_array, bpd_reg_config_array_len);
    if (HBD_BpdDetectStart() != HBD_RET_OK)  // start
    {
        EXAMPLE_DEBUG_LOG_L1("gh30x bpd start error\r\n");
        for (index = 0; index < __RETRY_MAX_CNT_CONFIG__; index ++) // retry
        {
            if (HBD_SimpleInit(&gh30x_init_config) == HBD_RET_OK)
            {
                if (HBD_LoadNewRegConfigArr(bpd_reg_config_array, bpd_reg_config_array_len) == HBD_RET_OK)
                {
                    if (HBD_BpdDetectStart() == HBD_RET_OK)
                    {
                        HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
                        break;
                    }
                }
            }
        }
    }
    else
    {
        HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
    }
}

#endif

#if (__PFA_DET_SUPPORT__)
/// gh30x pfa func start
void gh30x_pfa_start(void)
{
    uint8_t index = 0;

    HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_STOP();

    HBD_SetFifoThrCnt(FIFO_THR_CONFIG_TYPE_PFA, __PFA_FIFO_THR_CNT_CONFIG__);

    // gh30x_Load_new_config(pfa_reg_config_array, pfa_reg_config_array_len);
    if (HBD_PfaDetectStart() != HBD_RET_OK)  // start
    {
        EXAMPLE_DEBUG_LOG_L1("gh30x pfa start error\r\n");
        for (index = 0; index < __RETRY_MAX_CNT_CONFIG__; index ++) // retry
        {
            if (HBD_SimpleInit(&gh30x_init_config) == HBD_RET_OK)
            {
                if (HBD_LoadNewRegConfigArr(pfa_reg_config_array, pfa_reg_config_array_len) == HBD_RET_OK)
                {
                    if (HBD_PfaDetectStart() == HBD_RET_OK)
                    {
                        HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
                        break;
                    }
                }
            }
        }
    }
    else
    {
        HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
    }
}

#endif

#if (__SPO2_DET_SUPPORT__)
/// gh30x spo2 func start
void gh30x_spo2_start(void)
{
    uint8_t index = 0;

    HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_STOP();

    HBD_SetFifoThrCnt(FIFO_THR_CONFIG_TYPE_SPO2, __SPO2_FIFO_THR_CNT_CONFIG__);

    // gh30x_Load_new_config(spo2_reg_config_array, spo2_reg_config_array_len);
    if (HBD_SpO2DetectStart() != HBD_RET_OK)  // start
    {
        EXAMPLE_DEBUG_LOG_L1("gh30x spo2 start error\r\n");
        for (index = 0; index < __RETRY_MAX_CNT_CONFIG__; index ++) // retry
        {
            if (HBD_SimpleInit(&gh30x_init_config) == HBD_RET_OK)
            {
                if (HBD_LoadNewRegConfigArr(spo2_reg_config_array, spo2_reg_config_array_len) == HBD_RET_OK)
                {
                    if (HBD_SpO2DetectStart() == HBD_RET_OK)
                    {
                        HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
                        break;
                    }
                }
            }
        }
    }
    else
    {
        HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
    }
}
#endif

#if (__GH30X_COMMUNICATION_INTERFACE__ == GH30X_COMMUNICATION_INTERFACE_SPI)

/// i2c exchange to spi for gh30x wrtie
uint8_t gh30x_i2c_write_exchange_to_spi(uint8_t device_id, const uint8_t write_buffer[], uint16_t length)
{
    uint8_t ret = GH30X_EXAMPLE_OK_VAL;
    if ((length == 3) && (write_buffer[0] == 0xDD) && (write_buffer[1] == 0xDD))
    {
        hal_gh30x_spi_cs_set_low();
        hal_gh30x_spi_write(&write_buffer[2], 1);
        hal_gh30x_spi_cs_set_high();
        HBD_DelayUs(10);
    }
    else
    {
        uint8_t spi_write_buffer[5] = {0};
        uint16_t spi_real_len = length - 2;

        hal_gh30x_spi_cs_set_low();
        spi_write_buffer[0] = 0xF0;
        spi_write_buffer[1] = write_buffer[0];
        spi_write_buffer[2] = write_buffer[1];
        spi_write_buffer[3] = GET_HIGH_BYTE_FROM_WORD(spi_real_len);
        spi_write_buffer[4] = GET_LOW_BYTE_FROM_WORD(spi_real_len);
        hal_gh30x_spi_write(spi_write_buffer, 5);
        hal_gh30x_spi_write(&write_buffer[2], spi_real_len);
        HBD_DelayUs(20);
        hal_gh30x_spi_cs_set_high();
        HBD_DelayUs(10);
    }
    return ret;
}

/// i2c exchange to spi for gh30x read
uint8_t gh30x_i2c_read_exchange_to_spi(uint8_t device_id, const uint8_t write_buffer[], uint16_t write_length, uint8_t read_buffer[], uint16_t read_length)
{
    uint8_t ret = GH30X_EXAMPLE_OK_VAL;
    if (write_length == 2)
    {
        uint8_t spi_write_buffer[3] = {0};
        hal_gh30x_spi_cs_set_low();
        spi_write_buffer[0] = 0xF0;
        spi_write_buffer[1] = write_buffer[0];
        spi_write_buffer[2] = write_buffer[1];
        hal_gh30x_spi_write(spi_write_buffer, 3);
        HBD_DelayUs(20);
        hal_gh30x_spi_cs_set_high();
        HBD_DelayUs(10);
        hal_gh30x_spi_cs_set_low();
        spi_write_buffer[0] = 0xF1;
        hal_gh30x_spi_write(spi_write_buffer, 1);
        hal_gh30x_spi_read(read_buffer, read_length);
        HBD_DelayUs(20);
        hal_gh30x_spi_cs_set_high();
        HBD_DelayUs(10);
    }

    return ret;
}

#endif

/// system test otp check
uint8_t gh30x_systemtest_otp_check(void)
{
    uint8_t ret = 0;

#if (__SYSTEM_TEST_SUPPORT__)
    uint8_t systemtest_otp_buffer[64] = {0};
    ROMAHBD_Interfcae gh30x_wr_i;
    gh30x_wr_i.WR_Fun = (Write_fun)HBD_I2cWriteReg;
    gh30x_wr_i.RD_Fun = (Read_fun)HBD_I2cReadReg;

    HBD_I2cSendCmd(0xC0);
    HBD_DelayUs(600);
    ret = OTP_Check(&gh30x_wr_i, systemtest_otp_buffer);
    HBD_I2cSendCmd(0xC4);
    HBD_DelayUs(600);
#endif

    return ret;
}

/// system test os start
void gh30x_systemtest_os_start(uint8_t led_num)
{
    uint8_t index = 0;
    const ST_REGISTER *system_test_reg_config_ptr = NULL;
    uint16_t system_test_reg_config_len = 0;

    HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_STOP();
    // load config
    if (led_num == 2)
    {
        system_test_reg_config_ptr = systemtest_led2_reg_config_array;
        system_test_reg_config_len = systemtest_led2_reg_config_array_len;
    }
    else if (led_num == 1)
    {
        system_test_reg_config_ptr = systemtest_led1_reg_config_array;
        system_test_reg_config_len = systemtest_led1_reg_config_array_len;
    }
    else // fixed to 0
    {
        system_test_reg_config_ptr = systemtest_led0_reg_config_array;
        system_test_reg_config_len = systemtest_led0_reg_config_array_len;
    }

    gh30x_Load_new_config(system_test_reg_config_ptr, system_test_reg_config_len);
    HBD_FifoConfig(0, HBD_FUNCTIONAL_STATE_DISABLE);
    HBD_FifoConfig(1, HBD_FUNCTIONAL_STATE_DISABLE);
    if (HBD_HbDetectStart() != HBD_RET_OK)  // start
    {
        EXAMPLE_DEBUG_LOG_L1("gh30x system start error\r\n");
        for (index = 0; index < __RETRY_MAX_CNT_CONFIG__; index ++) // retry
        {
            if (HBD_SimpleInit(&gh30x_init_config) == HBD_RET_OK)
            {
                if (HBD_LoadNewRegConfigArr(system_test_reg_config_ptr, system_test_reg_config_len) == HBD_RET_OK)
                {
                    if (HBD_HbDetectStart() == HBD_RET_OK)
                    {
                        HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
                        break;
                    }
                }
            }
        }
    }
    HBD_FifoConfig(0, HBD_FUNCTIONAL_STATE_ENABLE);
    HBD_FifoConfig(1, HBD_FUNCTIONAL_STATE_ENABLE);
}

/// system test os calc
uint8_t gh30x_systemtest_os_calc(uint8_t led_num)
{
    uint8_t ret = 0xFF;
#if (__SYSTEM_TEST_SUPPORT__)
    static int32_t systemtest_rawdata_buffer[__SYSTEM_TEST_DATA_CNT_CONFIG__] = {0};
    static uint8_t systemtest_rawdata_buffer_index = 0;
    static uint8_t systemtest_last_led_num = 0;

    if (systemtest_last_led_num != led_num)
    {
        systemtest_rawdata_buffer_index = 0;
        systemtest_last_led_num = led_num;
    }

    systemtest_rawdata_buffer[systemtest_rawdata_buffer_index] = HBD_I2cReadRawdataReg(g_usReadRawdataRegList[0]) & 0x0001FFFF;
    systemtest_rawdata_buffer_index++;

    if (systemtest_rawdata_buffer_index >= __SYSTEM_TEST_DATA_CNT_CONFIG__)
    {
        if (led_num == 2)
        {
            ret = Check_Rawdata_Noise((int *)systemtest_rawdata_buffer, __SYSTEM_TEST_DATA_CNT_CONFIG__, (const ROMAHBData *)&systemtest_led2_os_result);
        }
        else if (led_num == 1)
        {
            ret = Check_Rawdata_Noise((int *)systemtest_rawdata_buffer, __SYSTEM_TEST_DATA_CNT_CONFIG__, (const ROMAHBData *)&systemtest_led1_os_result);
        }
        else // fixed to 0
        {
            ret = Check_Rawdata_Noise((int *)systemtest_rawdata_buffer, __SYSTEM_TEST_DATA_CNT_CONFIG__, (const ROMAHBData *)&systemtest_led0_os_result);
        }
        systemtest_rawdata_buffer_index = 0;
    }
#else
    ret = 0;
#endif
    return ret;
}

/********END OF FILE********* Copyright (c) 2003 - 2020, Goodix Co., Ltd. ********/
