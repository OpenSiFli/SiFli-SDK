/**
 * @copyright (c) 2003 - 2020, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3011_example_process.c
 *
 * @brief   example code for gh3011 (condensed  hbd_ctrl lib)
 *
 */

#include "gh3011_example_common.h"
#include "gh3011_example.h"


/// app mode start flag
bool goodix_app_start_app_mode = false;

/// system test mode start flag
bool goodix_system_test_mode = false;

/// system test os led num
uint8_t goodix_system_test_os_led_num = 0;

#if (__HB_NEED_ADT_CONFIRM__)
    /// adt confirm flag
    bool adt_confirm_flag = false;
    /// start flag without adt confirm flag
    bool hb_start_without_adt_confirm = false;
#endif

/// gsensor fifo index and len
static uint16_t gsensor_soft_fifo_buffer_index = 0;
/// gsnesor iffo
static ST_GS_DATA_TYPE gsensor_soft_fifo_buffer[GSENSOR_SOFT_FIFO_BUFFER_MAX_LEN];

/// gh30x run mode
uint8_t gh30x_run_mode = RUN_MODE_INVALID;



/// gh30x module init, include gsensor init
int gh30x_module_init(void)
{
    static bool first_init_flag = true;
    int8_t init_err_flag = HBD_RET_OK;

    /* log all version string */
    EXAMPLE_DEBUG_LOG_L1(GH30X_EXAMPLE_VER_STRING);
    EXAMPLE_DEBUG_LOG_L1("hbd ctrl version: %s\r\n", HBD_GetHbdVersion());
    EXAMPLE_DEBUG_LOG_L1("hba version: %s\r\n", HBD_GetHbaVersion());
    EXAMPLE_DEBUG_LOG_L1("spo2 version: %s\r\n", HBD_GetSpo2Version());

    if (first_init_flag)
    {
#if (__GH30X_COMMUNICATION_INTERFACE__ == GH30X_COMMUNICATION_INTERFACE_SPI)
        hal_gh30x_spi_init(); // spi init
        HBD_SetI2cRW(HBD_I2C_ID_SEL_1L0L, gh30x_i2c_write_exchange_to_spi, gh30x_i2c_read_exchange_to_spi); // register i2c exchange to spi api
#else // (__GH30X_COMMUNICATION_INTERFACE__ == GH30X_COMMUNICATION_INTERFACE_I2C)
        hal_gh30x_i2c_init(); // i2c init
        HBD_SetI2cRW(HBD_I2C_ID_SEL_1L0L, hal_gh30x_i2c_write, hal_gh30x_i2c_read); // register i2c RW func api
#endif

#if (__PLATFORM_DELAY_US_CONFIG__)
        HBD_SetDelayUsCallback(hal_gh30x_delay_us);
#endif
    }

    init_err_flag = HBD_SimpleInit(&gh30x_init_config); // init gh30x
    if (HBD_RET_OK != init_err_flag)
    {
        EXAMPLE_DEBUG_LOG_L1("gh30x init error[%s]\r\n", dbg_ret_val_string[DEBUG_HBD_RET_VAL_BASE + init_err_flag]);
        return GH30X_EXAMPLE_ERR_VAL;
    }

    init_err_flag = gsensor_drv_init(); // gsensor init
    if (GH30X_EXAMPLE_ERR_VAL == init_err_flag)
    {
        EXAMPLE_DEBUG_LOG_L1("gsensor init error\r\n");
        return GH30X_EXAMPLE_ERR_VAL;
    }

    if (first_init_flag)
    {
        hal_gsensor_int1_init(); // gsensor int pin init
        hal_gh30x_int_init(); // gh30x int pin init

#if (__GH30X_IRQ_PLUSE_WIDTH_CONFIG__)
        HBD_SetIrqPluseWidth(255); // set Irq pluse width (255us)
#endif

        gh30x_comm_pkg_init(); // comm pkg init

        HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_INIT();
    }

    EXAMPLE_DEBUG_LOG_L1("gh30x module init ok\r\n");
    first_init_flag = false;
    return GH30X_EXAMPLE_OK_VAL;
}

/// gh30x module start, with adt
void gh30x_module_start(EMGh30xRunMode start_run_mode)
{
#if ((__HB_DET_SUPPORT__) && (__HB_NEED_ADT_CONFIRM__))
    if (start_run_mode == RUN_MODE_ADT_HB_DET)
    {
        hb_start_without_adt_confirm = true;
    }
#endif
    gh30x_run_mode = (uint8_t)start_run_mode;
    EXAMPLE_DEBUG_LOG_L1("gh30x module start, mode [%s]\r\n", dbg_rum_mode_string[gh30x_run_mode]);
    if (gh30x_run_mode == RUN_MODE_SPO2_DET)
    {
        SEND_MCU_SPO2_UNWEAR_EVT(NULL, 0); // send start cmd with unwear evt data
    }
    gsensor_enter_normal_and_clear_buffer();
    gh30x_start_adt_with_mode((uint8_t)start_run_mode);
}

/// gh30x module start, without adt
void gh30x_module_start_without_adt(EMGh30xRunMode start_run_mode)
{
#if ((__HB_DET_SUPPORT__) && (__HB_NEED_ADT_CONFIRM__))
    if (start_run_mode == RUN_MODE_ADT_HB_DET)
    {
        hb_start_without_adt_confirm = true;
    }
#endif
    gh30x_run_mode = (uint8_t)start_run_mode;
    EXAMPLE_DEBUG_LOG_L1("gh30x module start, mode [%s]\r\n", dbg_rum_mode_string[gh30x_run_mode]);
    gsensor_enter_clear_buffer_and_enter_fifo();
    gh30x_start_func_with_mode((uint8_t)start_run_mode);
}

/// gh30x module stop
void gh30x_module_stop(void)
{
    gsensor_enter_normal_and_clear_buffer();
    gh30x_stop_func();
    gh30x_run_mode = RUN_MODE_INVALID;
    EXAMPLE_DEBUG_LOG_L1("gh30x module stop\r\n");
#if (__USE_GOODIX_APP__)
    goodix_app_start_app_mode = false; // if call stop, clear app mode
#endif
}

/// gh30x reset evt handler
void gh30x_reset_evt_handler(void)
{
    int8_t reinit_ret = HBD_RET_OK;
    uint8_t reinit_cnt = __RESET_REINIT_CNT_CONFIG__;
    gsensor_enter_normal_and_clear_buffer();
    // reinit
    do
    {
        reinit_ret = HBD_SimpleInit(&gh30x_init_config);
        reinit_cnt --;
    }
    while (reinit_ret != HBD_RET_OK);
    if ((reinit_ret == HBD_RET_OK) && (gh30x_run_mode != RUN_MODE_INVALID)) // if reinit ok, restart last mode
    {
#if (__USE_GOODIX_APP__)
        if (goodix_app_start_app_mode)
        {
            SEND_GH30X_RESET_EVT();
        }
        else
#endif
        {
            gh30x_start_adt_with_mode(gh30x_run_mode);
        }
    }
    EXAMPLE_DEBUG_LOG_L1("got gh30x reset evt, reinit [%s]\r\n", dbg_ret_val_string[DEBUG_HBD_RET_VAL_BASE + reinit_ret]);
}

/// gh30x unwear  evt handler
void gh30x_unwear_evt_handler(void)
{
#if (__USE_GOODIX_APP__)
    if (goodix_app_start_app_mode)
    {
        SEND_AUTOLED_FAIL_EVT();
        EXAMPLE_DEBUG_LOG_L1("got gh30x unwear evt, restart func\r\n");
        HBD_FifoConfig(0, HBD_FUNCTIONAL_STATE_DISABLE);
        HBD_FifoConfig(1, HBD_FUNCTIONAL_STATE_DISABLE);
        gh30x_start_func_whithout_adt_confirm(gh30x_run_mode);
        HBD_FifoConfig(0, HBD_FUNCTIONAL_STATE_ENABLE);
        HBD_FifoConfig(1, HBD_FUNCTIONAL_STATE_ENABLE);
    }
    else
#endif
    {
        gsensor_enter_normal_and_clear_buffer();
        if (gh30x_run_mode == RUN_MODE_ADT_HB_DET)
        {
#if (__HB_START_WITH_GSENSOR_MOTION__)
            gsensor_drv_enter_motion_det_mode();
            EXAMPLE_DEBUG_LOG_L1("got gh30x unwear evt, start gsensor motion\r\n");
#else
            gh30x_start_adt_with_mode(gh30x_run_mode);
            EXAMPLE_DEBUG_LOG_L1("got gh30x unwear evt, start adt detect\r\n");
#endif
            SEND_MCU_HB_MODE_WEAR_STATUS(WEAR_STATUS_UNWEAR, NULL, 0);
        }
        else
        {
            if (gh30x_run_mode == RUN_MODE_SPO2_DET)
            {
                SEND_MCU_SPO2_UNWEAR_EVT(NULL, 0);
            }
            gh30x_start_adt_with_mode(gh30x_run_mode);
            EXAMPLE_DEBUG_LOG_L1("got gh30x unwear evt, start adt detect\r\n");
        }
        HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_STOP();
        handle_wear_status_result(WEAR_STATUS_UNWEAR);
    }
}

/// gh30x wear evt handler
void gh30x_wear_evt_handler(void)
{
    gsensor_enter_clear_buffer_and_enter_fifo();
    gh30x_start_func_with_mode(gh30x_run_mode);

#if (__HB_NEED_ADT_CONFIRM__)
    EXAMPLE_DEBUG_LOG_L1("got gh30x wear evt, start func[%s], adt confrim [%d]\r\n", dbg_rum_mode_string[gh30x_run_mode], adt_confirm_flag);
    if (gh30x_run_mode != RUN_MODE_ADT_HB_DET)
    {
        handle_wear_status_result(WEAR_STATUS_WEAR);
    }
#else
    EXAMPLE_DEBUG_LOG_L1("got gh30x wear evt, start func[%s]\r\n", dbg_rum_mode_string[gh30x_run_mode]);
    if (gh30x_run_mode == RUN_MODE_ADT_HB_DET)
    {
        SEND_MCU_HB_MODE_WEAR_STATUS(WEAR_STATUS_WEAR, NULL, 0);
    }
    handle_wear_status_result(WEAR_STATUS_WEAR);
#endif
}

/// calc unwear status handle
void gh30x_handle_calc_unwear_status(void)
{
    gsensor_enter_normal_and_clear_buffer();
    if (gh30x_run_mode == RUN_MODE_ADT_HB_DET)
    {
#if (__HB_START_WITH_GSENSOR_MOTION__)
        gsensor_drv_enter_motion_det_mode();
        EXAMPLE_DEBUG_LOG_L1("calc unwear status, start gsensor motion\r\n");
#else
        gh30x_start_adt_with_mode(gh30x_run_mode);
        EXAMPLE_DEBUG_LOG_L1("calc unwear status, start adt detect\r\n");
#endif
    }
    else
    {
        gh30x_start_adt_with_mode(gh30x_run_mode);
        EXAMPLE_DEBUG_LOG_L1("calc unwear status, start adt detect\r\n");
        handle_wear_status_result(WEAR_STATUS_UNWEAR);
    }
}

#if (__HB_DET_SUPPORT__)
/// fifo evt hb mode calc
static void gh30x_fifo_evt_hb_mode_calc(int32_t *dbg_rawdata_ptr)
{
    uint16_t dbg_rawdata_len = __ALGO_CALC_DBG_BUFFER_LEN__;
#if (__HB_NEED_ADT_CONFIRM__)
    if (adt_confirm_flag)
    {
        uint8_t adt_confirm_res = HBD_AdtConfirmCalculateByFifoIntDbgOutputData(gsensor_soft_fifo_buffer, gsensor_soft_fifo_buffer_index,
                                  __GS_SENSITIVITY_CONFIG__, (GS32(*)[DBG_MCU_MODE_PKG_LEN])dbg_rawdata_ptr, &dbg_rawdata_len);
        EXAMPLE_DEBUG_LOG_L1("adt confirm calc, gs_len=%d, result=0x%x\r\n", gsensor_soft_fifo_buffer_index, adt_confirm_res);
        if (adt_confirm_res != ADT_CONFRIM_STATUS_DETECTING)
        {
            adt_confirm_flag = false;
            HBD_Stop();
            if (adt_confirm_res == ADT_CONFRIM_STATUS_WEAR)
            {
                gh30x_module_start_without_adt((EMGh30xRunMode)gh30x_run_mode);
                HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
                SEND_MCU_HB_MODE_WEAR_STATUS(WEAR_STATUS_WEAR, (GS32(*)[DBG_MCU_MODE_PKG_LEN])dbg_rawdata_ptr, dbg_rawdata_len);
                handle_wear_status_result(WEAR_STATUS_WEAR);
            }
            else if (adt_confirm_res == ADT_CONFRIM_STATUS_UNWEAR)
            {
                SEND_MCU_HB_MODE_WEAR_STATUS(WEAR_STATUS_UNWEAR, (GS32(*)[DBG_MCU_MODE_PKG_LEN])dbg_rawdata_ptr, dbg_rawdata_len);
                gh30x_handle_calc_unwear_status();
            }
        }
        EXAMPLE_LOG_RAWDARA("adt confirm calc:\r\n", dbg_rawdata_ptr, dbg_rawdata_len);
    }
    else
#endif
    {
        uint8_t hb_value = 0;
        uint8_t hb_value_lvl = 0;
        uint8_t wearing_state = 0;
        uint8_t wearing_quality = 0;
        uint8_t voice_broadcast = 0;
        uint16_t rr_value = 0;
        uint8_t hb_res = HBD_HbCalculateWithLvlByFifoIntDebugOutputData(gsensor_soft_fifo_buffer, gsensor_soft_fifo_buffer_index, __GS_SENSITIVITY_CONFIG__,
                         &hb_value, &hb_value_lvl, &wearing_state, &wearing_quality, &voice_broadcast, &rr_value, (GS32(*)[DBG_MCU_MODE_PKG_LEN])dbg_rawdata_ptr, &dbg_rawdata_len);
        handle_hb_mode_result(hb_value, hb_value_lvl, wearing_state, rr_value, (int32_t (*)[DBG_MCU_MODE_PKG_LEN])dbg_rawdata_ptr, dbg_rawdata_len);
        EXAMPLE_DEBUG_LOG_L1("hb calc, gs_len=%d, result=%d,%d,%d,%d\r\n", gsensor_soft_fifo_buffer_index, hb_value, hb_value_lvl, wearing_state, rr_value);
        if (wearing_state == WEAR_STATUS_UNWEAR)
        {
            SEND_MCU_HB_MODE_RESULT(hb_value, hb_value_lvl, WEAR_STATUS_UNWEAR, wearing_quality, voice_broadcast, hb_res, rr_value, (GS32(*)[DBG_MCU_MODE_PKG_LEN])dbg_rawdata_ptr, dbg_rawdata_len);
            HBD_Stop();
            gh30x_handle_calc_unwear_status();
        }
        else
        {
            HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
            SEND_MCU_HB_MODE_RESULT(hb_value, hb_value_lvl, WEAR_STATUS_WEAR, wearing_quality, voice_broadcast, hb_res, rr_value, (GS32(*)[DBG_MCU_MODE_PKG_LEN])dbg_rawdata_ptr, dbg_rawdata_len);
        }
        BLE_MODULE_SEND_HB(hb_value);
        EXAMPLE_LOG_RAWDARA("hb calc:\r\n", dbg_rawdata_ptr, dbg_rawdata_len);
    }
}

#endif

#if (__SPO2_DET_SUPPORT__)
/// fifo evt spo2 mode calc
static void gh30x_fifo_evt_spo2_mode_calc(int32_t *dbg_rawdata_ptr)
{
    uint16_t dbg_rawdata_len = __ALGO_CALC_DBG_BUFFER_LEN__;
    uint8_t spo2_value = 0;
    uint8_t spo2_lvl = 0;
    uint8_t hb_value = 0;
    uint8_t hb_lvl = 0;
    uint16_t hrv_val[4] = {0};
    uint8_t hrv_lvl = 0;
    uint8_t hrv_cnt = 0;
    uint16_t spo2_r_value = 0;
    uint8_t wearing_state = 0;
    uint8_t valid_lvl = 0;

    HBD_Spo2CalculateByFifoIntDbgRawdataInnerUse(gsensor_soft_fifo_buffer, gsensor_soft_fifo_buffer_index, __GS_SENSITIVITY_CONFIG__,
            &spo2_value, &spo2_lvl, &hb_value, &hb_lvl, &hrv_val[0], &hrv_val[1], &hrv_val[2], &hrv_val[3], &hrv_lvl, &hrv_cnt,
            &spo2_r_value, &wearing_state, (GS32(*)[DBG_MCU_MODE_PKG_LEN])dbg_rawdata_ptr, &dbg_rawdata_len, &valid_lvl);
    handle_spo2_mode_result(spo2_value, spo2_lvl, hb_value, hb_lvl, hrv_val, hrv_lvl, hrv_cnt, spo2_r_value, wearing_state, (int32_t (*)[DBG_MCU_MODE_PKG_LEN])dbg_rawdata_ptr, dbg_rawdata_len);
    EXAMPLE_DEBUG_LOG_L1("spo2 calc, gs_len=%d, result=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", gsensor_soft_fifo_buffer_index, spo2_value, spo2_lvl, hb_value, hb_lvl, hrv_val[0], hrv_val[1], hrv_val[2],
                         hrv_val[3], hrv_lvl, hrv_cnt, spo2_r_value, wearing_state);
    if (wearing_state == WEAR_STATUS_UNWEAR)
    {
        SEND_MCU_SPO2_UNWEAR_EVT((GS32(*)[DBG_MCU_MODE_PKG_LEN])dbg_rawdata_ptr, dbg_rawdata_len);
        HBD_Stop();
        gh30x_handle_calc_unwear_status();
    }
    else
    {
        HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
        FIXED_SPO2_ALGO_RES(spo2_value);
        SEND_MCU_SPO2_MODE_RESULT(spo2_value, spo2_lvl, hb_value, hb_lvl, hrv_val, hrv_lvl, hrv_cnt, spo2_r_value, wearing_state, valid_lvl, (GS32(*)[DBG_MCU_MODE_PKG_LEN])dbg_rawdata_ptr, dbg_rawdata_len);
    }
    EXAMPLE_LOG_RAWDARA("spo2 calc:\r\n", dbg_rawdata_ptr, dbg_rawdata_len);
}

#endif

#if (__HRV_DET_SUPPORT__)
/// fifo evt hrv mode calc
static void gh30x_fifo_evt_hrv_mode_calc(int32_t *dbg_rawdata_ptr)
{
    uint16_t dbg_rawdata_len = __ALGO_CALC_DBG_BUFFER_LEN__;
    uint16_t hrv_rr_value_array[HRV_MODE_RES_MAX_CNT];
    uint8_t hrv_rr_value_fresh_cnt = 0;
    uint8_t hrv_lvl = 0;
    hrv_rr_value_fresh_cnt = HBD_HrvCalculateWithLvlByFifoIntDbgRawdata(gsensor_soft_fifo_buffer, gsensor_soft_fifo_buffer_index, __GS_SENSITIVITY_CONFIG__, hrv_rr_value_array, &hrv_lvl, (GS32(*)[DBG_MCU_MODE_PKG_LEN])dbg_rawdata_ptr, &dbg_rawdata_len);
    HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
    handle_hrv_mode_result(hrv_rr_value_array, hrv_rr_value_fresh_cnt, hrv_lvl, (int32_t (*)[DBG_MCU_MODE_PKG_LEN])dbg_rawdata_ptr, dbg_rawdata_len);
    SEND_MCU_HRV_MODE_RESULT(hrv_rr_value_array, hrv_rr_value_fresh_cnt, hrv_lvl, (GS32(*)[DBG_MCU_MODE_PKG_LEN])dbg_rawdata_ptr, dbg_rawdata_len);
    EXAMPLE_DEBUG_LOG_L1("hrv calc, result=%d,%d,%d,%d,%d\r\n", hrv_rr_value_array[0], hrv_rr_value_array[1], hrv_rr_value_array[2], hrv_rr_value_array[3],
                         hrv_rr_value_fresh_cnt); //just print 4 result
    BLE_MODULE_SEND_RRI(hrv_rr_value_array, hrv_rr_value_fresh_cnt);
}

#endif

#if (__HSM_DET_SUPPORT__)
/// fifo evt hsm mode calc
static void gh30x_fifo_evt_hsm_mode_calc(int32_t *dbg_rawdata_ptr)
{
    uint16_t dbg_rawdata_len = __ALGO_CALC_DBG_BUFFER_LEN__;
    uint8_t hb_value = 0;
    uint8_t sleep_state = 0;
    uint8_t respiratory_rate = 0;
    uint16_t asleep_time = 0;
    uint16_t wakeup_time = 0;
    HBD_HsmCalculateByFifoIntDbgRawdata(gsensor_soft_fifo_buffer, gsensor_soft_fifo_buffer_index, __GS_SENSITIVITY_CONFIG__,
                                        &sleep_state, &hb_value, &respiratory_rate, &asleep_time, &wakeup_time, (GS32(*)[DBG_MCU_MODE_PKG_LEN])dbg_rawdata_ptr, &dbg_rawdata_len);
    HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
    handle_hsm_mode_result(sleep_state, hb_value, respiratory_rate, asleep_time, wakeup_time, (int32_t (*)[DBG_MCU_MODE_PKG_LEN])dbg_rawdata_ptr, dbg_rawdata_len);
    EXAMPLE_DEBUG_LOG_L1("hsm calc, result=%d,%d,%d,%d,%d\r\n", sleep_state, hb_value, respiratory_rate, asleep_time, wakeup_time);
}

#endif

#if (__BPD_DET_SUPPORT__)
/// fifo evt bpd mode calc
static void gh30x_fifo_evt_bpd_mode_calc(int32_t *dbg_rawdata_ptr)
{
    uint16_t dbg_rawdata_len = __ALGO_CALC_DBG_BUFFER_LEN__;
    uint16_t sbp_value = 0;
    uint16_t dbp_value = 0;
    HBD_BpdCalculateByFifoInt(gsensor_soft_fifo_buffer, gsensor_soft_fifo_buffer_index, __GS_SENSITIVITY_CONFIG__, &sbp_value, &dbp_value);
    HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
    handle_bpd_mode_result(sbp_value, dbp_value, (int32_t (*)[DBG_MCU_MODE_PKG_LEN])dbg_rawdata_ptr, dbg_rawdata_len);
    EXAMPLE_DEBUG_LOG_L1("bpd calc, result=%d,%d\r\n", sbp_value, dbp_value);
}

#endif

#if (__PFA_DET_SUPPORT__)
/// fifo evt pfa mode calc
static void gh30x_fifo_evt_pfa_mode_calc(int32_t *dbg_rawdata_ptr)
{
    uint16_t dbg_rawdata_len = __ALGO_CALC_DBG_BUFFER_LEN__;
    uint8_t pressure_level = 0;
    uint8_t fatigue_level = 0;
    uint8_t body_age = 0;
    HBD_PfaCalculateByFifoInt(gsensor_soft_fifo_buffer, gsensor_soft_fifo_buffer_index, __GS_SENSITIVITY_CONFIG__, &pressure_level, &fatigue_level, &body_age);
    HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START();
    handle_pfa_mode_result(pressure_level, fatigue_level, body_age, (int32_t (*)[DBG_MCU_MODE_PKG_LEN])dbg_rawdata_ptr, dbg_rawdata_len);
    EXAMPLE_DEBUG_LOG_L1("pfa calc, val=%d,%d,%d\r\n", pressure_level, fatigue_level, body_age);
}

#endif

/// gh30x fifo evt handler
void gh30x_fifo_evt_handler(void)
{
    int32_t *dbg_rawdata_buffer_ptr = NULL;
#if (__DBG_OUTPUT_RAWDATA__)
    int32_t dbg_rawdata_buffer[__ALGO_CALC_DBG_BUFFER_LEN__][DBG_MCU_MODE_PKG_LEN];
    dbg_rawdata_buffer_ptr = (int32_t *)dbg_rawdata_buffer;
#endif
    EXAMPLE_DEBUG_LOG_L1("got gh30x fifo evt, func[%s]\r\n", dbg_rum_mode_string[gh30x_run_mode]);
    gsensor_read_fifo_data();
    switch (gh30x_run_mode)
    {
#if (__HB_DET_SUPPORT__)
    case RUN_MODE_ADT_HB_DET:
        gh30x_fifo_evt_hb_mode_calc(dbg_rawdata_buffer_ptr);
        break;
#endif

#if (__SPO2_DET_SUPPORT__)
    case RUN_MODE_SPO2_DET:
        gh30x_fifo_evt_spo2_mode_calc(dbg_rawdata_buffer_ptr);
        break;
#endif

#if (__HRV_DET_SUPPORT__)
    case RUN_MODE_HRV_DET:
        gh30x_fifo_evt_hrv_mode_calc(dbg_rawdata_buffer_ptr);
        break;
#endif

#if (__HSM_DET_SUPPORT__)
    case RUN_MODE_HSM_DET:
        gh30x_fifo_evt_hsm_mode_calc(dbg_rawdata_buffer_ptr);
        break;
#endif

#if (__BPD_DET_SUPPORT__)
    case RUN_MODE_BPD_DET:
        gh30x_fifo_evt_bpd_mode_calc(dbg_rawdata_buffer_ptr);
        break;
#endif

#if (__PFA_DET_SUPPORT__)
    case RUN_MODE_PFA_DET:
        gh30x_fifo_evt_pfa_mode_calc(dbg_rawdata_buffer_ptr);
        break;
#endif

    default:
        EXAMPLE_DEBUG_LOG_L1("clac that mode[%s] is not support!\r\n", dbg_rum_mode_string[gh30x_run_mode]);
        break;
    }
    gsensor_drv_clear_buffer(gsensor_soft_fifo_buffer, &gsensor_soft_fifo_buffer_index, GSENSOR_SOFT_FIFO_BUFFER_MAX_LEN);
}

/// gh30x newdata evt handler
void gh30x_new_data_evt_handler(void)
{
    ST_GS_DATA_TYPE gsensor_data;
    gsensor_drv_get_data(&gsensor_data); // get gsensor data

#if (__USE_GOODIX_APP__)
    if ((goodix_app_start_app_mode) && ((gh30x_run_mode == RUN_MODE_ADT_HB_DET) || (gh30x_run_mode == RUN_MODE_HRV_DET) || (gh30x_run_mode == RUN_MODE_SPO2_DET)))
    {
        EXAMPLE_DEBUG_LOG_L2("got gh30x new data evt, send rawdata to app\n");
        if (GH30X_AUTOLED_ERR_VAL == HBD_SendRawdataPackageByNewdataInt(&gsensor_data, __GS_SENSITIVITY_CONFIG__))
        {
            SEND_AUTOLED_FAIL_EVT();
        }
    }
    else
#endif
    {
#if (__USE_GOODIX_APP__)
        goodix_app_start_app_mode = false; // if call stop, clear app mode
#endif

#if (__SYSTEM_TEST_SUPPORT__)
        if (goodix_system_test_mode)
        {
            EXAMPLE_DEBUG_LOG_L2("got gh30x new data evt, put data to system test module.\r\n");
            uint8_t os_test_ret = gh30x_systemtest_os_calc(goodix_system_test_os_led_num);
            if (os_test_ret != 0xFF) // test has done
            {
                handle_system_test_os_result(goodix_system_test_os_led_num, os_test_ret);
                EXAMPLE_DEBUG_LOG_L1("system test os[led %d] ret: %d!\r\n", goodix_system_test_os_led_num, os_test_ret);
                if (goodix_system_test_os_led_num < 2)
                {
                    goodix_system_test_os_led_num++;
                    gh30x_systemtest_os_start(goodix_system_test_os_led_num);
                    EXAMPLE_DEBUG_LOG_L1("system test change to test next led:%d!\r\n", goodix_system_test_os_led_num);
                }
                else
                {
                    goodix_system_test_mode = false;
                    EXAMPLE_DEBUG_LOG_L1("system test has done!\r\n");
                    HBD_Stop();
                }
            }
        }
        else
#endif
        {
            EXAMPLE_DEBUG_LOG_L1("got gh30x new data evt, shouldn't reach here!!\r\n");
            gh30x_handle_calc_unwear_status();
        }
    }
}

/// gh30x fifo full evt handler
void gh30x_fifo_full_evt_handler(void)
{
    HBD_Stop();
    gsensor_enter_clear_buffer_and_enter_fifo();
    gh30x_start_func_with_mode(gh30x_run_mode);
    EXAMPLE_DEBUG_LOG_L1("got gh30x fifo full evt, func[%s],  shouldn't reach here!!\r\n", dbg_rum_mode_string[gh30x_run_mode]);
}

/// gh30x int msg handler
void gh30x_int_msg_handler(void)
{
    uint8_t gh30x_irq_status;
    uint8_t gh30x_adt_working_flag;
    HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_STOP();
    gh30x_irq_status = HBD_GetIntStatus();
    gh30x_adt_working_flag = HBD_IsAdtWearDetectHasStarted();

    EXAMPLE_DEBUG_LOG_L1("irq %d, adt %d\n", gh30x_irq_status, gh30x_adt_working_flag);

    if (gh30x_irq_status == INT_STATUS_FIFO_WATERMARK)
    {
        gh30x_fifo_evt_handler();
    }
    else if (gh30x_irq_status == INT_STATUS_NEW_DATA)
    {
        gh30x_new_data_evt_handler();
    }
    else if (gh30x_irq_status == INT_STATUS_WEAR_DETECTED)
    {
        gh30x_wear_evt_handler();
    }
    else if (gh30x_irq_status == INT_STATUS_UNWEAR_DETECTED)
    {
        gh30x_unwear_evt_handler();
    }
    else if (gh30x_irq_status == INT_STATUS_CHIP_RESET) // if gh30x reset, need reinit
    {
        gh30x_reset_evt_handler();
    }
    else if (gh30x_irq_status == INT_STATUS_FIFO_FULL) // if gh30x fifo full, need restart
    {
        gh30x_fifo_full_evt_handler();
    }

    if ((gh30x_adt_working_flag == 1) && (gh30x_irq_status != INT_STATUS_WEAR_DETECTED) && (gh30x_irq_status != INT_STATUS_UNWEAR_DETECTED)) // adt working
    {
        gh30x_start_adt_with_mode(gh30x_run_mode);
    }
}

/// gh30x fifo int timeout msg handler
void gh30x_fifo_int_timeout_msg_handler(void)
{
    uint8_t gh30x_irq_status_1;
    uint8_t gh30x_irq_status_2;

    HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_STOP();

    EXAMPLE_DEBUG_LOG_L1("fifo int time out!!!\r\n");

    gh30x_irq_status_1 = HBD_GetIntStatus();
    gh30x_irq_status_2 = HBD_GetIntStatus();
    if ((gh30x_irq_status_1 == INT_STATUS_FIFO_WATERMARK) && (gh30x_irq_status_2 == INT_STATUS_INVALID))
    {
        gh30x_fifo_evt_handler();
    }
    else
    {
        gh30x_reset_evt_handler();
    }
}

/// communicate parse handler
void gh30x_communicate_parse_handler(int8_t communicate_type, uint8_t *buffer, uint8_t length)
{
    EM_COMM_CMD_TYPE comm_cmd_type  = HBD_CommParseHandler(communicate_type, buffer, length); // parse recv data
    if ((uint8_t)communicate_type == COMM_TYPE_INVALID_VAL)
    {
        EXAMPLE_DEBUG_LOG_L1("comm_type error, pelase check inited or not, @ref gh30x_module_init!!!\r\n");
    }
    else
    {
        EXAMPLE_DEBUG_LOG_L1("parse: cmd[%x-%s], comm_type[%d], length[%d]\r\n", buffer[0], dbg_comm_cmd_string[(uint8_t)comm_cmd_type], communicate_type, length);
    }
    if (comm_cmd_type < COMM_CMD_INVALID)
    {
        handle_goodix_communicate_cmd(comm_cmd_type);
        if ((comm_cmd_type == COMM_CMD_ALGO_IN_APP_HB_START)
                || (comm_cmd_type == COMM_CMD_ALGO_IN_APP_HRV_START)
                || (comm_cmd_type == COMM_CMD_ALGO_IN_APP_SPO2_START))  // handle all app mode cmd
        {
            uint8_t app_start_mode = RUN_MODE_INVALID;
            goodix_app_start_app_mode = true;
            if (comm_cmd_type == COMM_CMD_ALGO_IN_APP_HB_START)
            {
                app_start_mode = RUN_MODE_ADT_HB_DET;
            }
            else if (comm_cmd_type == COMM_CMD_ALGO_IN_APP_HRV_START)
            {
                app_start_mode = RUN_MODE_HRV_DET;
            }
            else if (comm_cmd_type == COMM_CMD_ALGO_IN_APP_SPO2_START)
            {
                app_start_mode = RUN_MODE_SPO2_DET;
            }
            HBD_FifoConfig(0, HBD_FUNCTIONAL_STATE_DISABLE);
            HBD_FifoConfig(1, HBD_FUNCTIONAL_STATE_DISABLE);
            gh30x_start_func_whithout_adt_confirm(app_start_mode);
            HBD_FifoConfig(0, HBD_FUNCTIONAL_STATE_ENABLE);
            HBD_FifoConfig(1, HBD_FUNCTIONAL_STATE_ENABLE);
        }
        else if ((comm_cmd_type == COMM_CMD_ADT_SINGLE_MODE_START)
                 || (comm_cmd_type == COMM_CMD_ALGO_IN_MCU_HRV_START)
                 || (comm_cmd_type == COMM_CMD_ALGO_IN_MCU_SPO2_START)) // handle all mcu mode cmd
        {
            uint8_t mcu_start_mode = RUN_MODE_INVALID;
            goodix_app_start_app_mode = false;
            if (comm_cmd_type == COMM_CMD_ADT_SINGLE_MODE_START)
            {
                mcu_start_mode = RUN_MODE_ADT_HB_DET;
            }
            else if (comm_cmd_type == COMM_CMD_ALGO_IN_MCU_HRV_START)
            {
                mcu_start_mode = RUN_MODE_HRV_DET;
            }
            else if (comm_cmd_type == COMM_CMD_ALGO_IN_MCU_SPO2_START)
            {
                mcu_start_mode = RUN_MODE_SPO2_DET;
            }
            gh30x_module_start((EMGh30xRunMode)mcu_start_mode);
        }
        else // handle all stop cmd
        {
            goodix_app_start_app_mode = false;
            gh30x_module_stop();
        }
    }
}

/// enter normal mode and clear fifo buffer
void gsensor_enter_normal_and_clear_buffer(void)
{
    gsensor_drv_enter_normal_mode();
    gsensor_drv_clear_buffer(gsensor_soft_fifo_buffer, &gsensor_soft_fifo_buffer_index, GSENSOR_SOFT_FIFO_BUFFER_MAX_LEN);
}

/// clear fifo buffer and enter fifo mode
void gsensor_enter_clear_buffer_and_enter_fifo(void)
{
    gsensor_drv_enter_normal_mode();
    gsensor_drv_clear_buffer(gsensor_soft_fifo_buffer, &gsensor_soft_fifo_buffer_index, GSENSOR_SOFT_FIFO_BUFFER_MAX_LEN);
    gsensor_drv_enter_fifo_mode();
}

/// motion detect irq handler
void gsensor_motion_has_detect(void)
{
    gsensor_enter_normal_and_clear_buffer();
    gh30x_start_adt_with_mode(gh30x_run_mode);
    EXAMPLE_DEBUG_LOG_L1("got gsensor motion evt, start adt [%s]\r\n", dbg_rum_mode_string[gh30x_run_mode]);
}

/// get data into fifo buffer
void gsensor_read_fifo_data(void)
{
    gsensor_drv_get_fifo_data(gsensor_soft_fifo_buffer, &gsensor_soft_fifo_buffer_index, GSENSOR_SOFT_FIFO_BUFFER_MAX_LEN);
}

/// start gh30x adt func with adt_run_mode
void gh30x_start_adt_with_mode(uint8_t adt_run_mode)
{
    const ST_REGISTER *reg_config_ptr = NULL;
    uint16_t reg_config_len = 0;
    switch (adt_run_mode)
    {
#if (__HB_DET_SUPPORT__)
    case RUN_MODE_ADT_HB_DET:
#if (__HB_NEED_ADT_CONFIRM__)
        if (hb_start_without_adt_confirm)
        {
            reg_config_ptr = hb_reg_config_array;
            reg_config_len = hb_reg_config_array_len;
        }
        else
        {
            reg_config_ptr = hb_adt_confirm_reg_config;
            reg_config_len = hb_adt_confirm_reg_config_len;
        }
#else
        reg_config_ptr = hb_reg_config_array;
        reg_config_len = hb_reg_config_array_len;
#endif
        break;
#endif

#if (__SPO2_DET_SUPPORT__)
    case RUN_MODE_SPO2_DET:
        reg_config_ptr = spo2_reg_config_array;
        reg_config_len = spo2_reg_config_array_len;
        break;
#endif

#if (__HRV_DET_SUPPORT__)
    case RUN_MODE_HRV_DET:
        reg_config_ptr = hrv_reg_config_array;
        reg_config_len = hrv_reg_config_array_len;
        break;
#endif

#if (__HSM_DET_SUPPORT__)
    case RUN_MODE_HSM_DET:
        reg_config_ptr = hsm_reg_config_array;
        reg_config_len = hsm_reg_config_array_len;
        break;
#endif

#if (__BPD_DET_SUPPORT__)
    case RUN_MODE_BPD_DET:
        reg_config_ptr = bpd_reg_config_array;
        reg_config_len = bpd_reg_config_array_len;
        break;
#endif

#if (__PFA_DET_SUPPORT__)
    case RUN_MODE_PFA_DET:
        reg_config_ptr = pfa_reg_config_array;
        reg_config_len = pfa_reg_config_array_len;
        break;
#endif

    default:
        EXAMPLE_DEBUG_LOG_L1("adt start that mode[%s] is not support!\r\n", dbg_rum_mode_string[gh30x_run_mode]);
        break;
    }
    gh30x_adt_wear_detect_start(reg_config_ptr, reg_config_len);
}

/// start gh30x func with func_run_mode
void gh30x_start_func_with_mode(uint8_t func_run_mode)
{
    switch (func_run_mode)
    {
#if (__HB_DET_SUPPORT__)
    case RUN_MODE_ADT_HB_DET:
#if (__HB_NEED_ADT_CONFIRM__)
        if (hb_start_without_adt_confirm)
        {
            hb_start_without_adt_confirm = false;
#if (__USE_GOODIX_APP__)
            if (!goodix_app_start_app_mode)
            {
                gh30x_Load_new_config(hb_reg_config_array, hb_reg_config_array_len);
            }
#endif
            gh30x_hb_start();
        }
        else
        {
            gh30x_adt_confirm_start();
            adt_confirm_flag = true;
        }
#else
        gh30x_hb_start();
#endif
        break;
#endif

#if (__SPO2_DET_SUPPORT__)
    case RUN_MODE_SPO2_DET:
        gh30x_spo2_start();
        break;
#endif

#if (__HRV_DET_SUPPORT__)
    case RUN_MODE_HRV_DET:
        gh30x_hrv_start();
        break;
#endif

#if (__HSM_DET_SUPPORT__)
    case RUN_MODE_HSM_DET:
        gh30x_hsm_start();
        break;
#endif

#if (__BPD_DET_SUPPORT__)
    case RUN_MODE_BPD_DET:
        gh30x_bpd_start();
        break;
#endif

#if (__PFA_DET_SUPPORT__)
    case RUN_MODE_PFA_DET:
        gh30x_pfa_start();
        break;
#endif

    default:
        EXAMPLE_DEBUG_LOG_L1("func start that mode[%s] is not support!\r\n", dbg_rum_mode_string[gh30x_run_mode]);
        break;
    }
}

/// stop gh30x func
void gh30x_stop_func(void)
{
#if (__HB_NEED_ADT_CONFIRM__)
    hb_start_without_adt_confirm = false;
    adt_confirm_flag = false;
#endif
    HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_STOP();
    HBD_Stop();
}

/// gh30x start func fix adt confirm
void gh30x_start_func_whithout_adt_confirm(uint8_t start_run_mode)
{
#if ((__HB_DET_SUPPORT__) && (__HB_NEED_ADT_CONFIRM__))
    if (start_run_mode == RUN_MODE_ADT_HB_DET)
    {
        hb_start_without_adt_confirm = true;
    }
#endif
    gsensor_enter_normal_and_clear_buffer();
    gh30x_start_func_with_mode(start_run_mode);
    gh30x_run_mode = start_run_mode;
    EXAMPLE_DEBUG_LOG_L1("gh30x module start, mode [%s]\r\n", dbg_rum_mode_string[gh30x_run_mode]);
}

/// gh30x module system test otp check
void gh30x_module_system_test_otp_check(void)
{
    uint8_t ret = 0;
    gh30x_module_stop();
    ret = gh30x_systemtest_otp_check();
    handle_system_test_otp_check_result(ret);
    EXAMPLE_DEBUG_LOG_L1("system test otp check, ret = %d\r\n", ret);
}

/// gh30x module system test os check
void gh30x_module_system_test_os_start(void)
{
    gh30x_module_stop();
    goodix_system_test_os_led_num = 0;
    goodix_system_test_mode = true;
    gh30x_systemtest_os_start(goodix_system_test_os_led_num);
    EXAMPLE_DEBUG_LOG_L1("system test os check start\r\n");
}

/********END OF FILE********* Copyright (c) 2003 - 2020, Goodix Co., Ltd. ********/
