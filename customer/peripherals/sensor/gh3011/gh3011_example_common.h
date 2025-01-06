/**
 * @copyright (c) 2003 - 2020, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3011_example_common.h
 *
 * @brief   example code for gh3011 (condensed  hbd_ctrl lib)
 *
 * @version  @ref GH30X_EXAMPLE_VER_STRING
 *
 */


#ifndef _GH3011_EXAMPLE_COMMON_H_
#define _GH3011_EXAMPLE_COMMON_H_

/* includes */
#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "gh3011_example_config.h"
#include "hbd_ctrl.h"
#include "hbd_communicate.h"
#if ((__USE_GOODIX_APP__) && (__GOODIX_APP_MCU_SUP__))
    #include "app_cmd_send.h"
#endif
#if (__SYSTEM_TEST_SUPPORT__)
    #include "systemTest.h"
#endif


/// version string
#define GH30X_EXAMPLE_VER_STRING            "example code v0.0.1.1 (For hbd_ctrl lib v0.4.8.2 and later)\r\n"

///Module enable limited by hbd_ctrl lib
#if !__HBD_HB_ENABLE__
    #undef __HB_DET_SUPPORT__
    #define __HB_DET_SUPPORT__ (0)
#endif
#if !__HBD_SPO2_ENABLE__
    #undef __SPO2_DET_SUPPORT__
    #define __SPO2_DET_SUPPORT__ (0)
#endif
#if !__HBD_HRV_ENABLE__
    #undef __HRV_DET_SUPPORT__
    #define __HRV_DET_SUPPORT__ (0)
#endif
#if !__HBD_HSM_ENABLE__
    #undef __HSM_DET_SUPPORT__
    #define __HSM_DET_SUPPORT__ (0)
#endif
#if !__HBD_BP_ENABLE__
    #undef __BPD_DET_SUPPORT__
    #define __BPD_DET_SUPPORT__ (0)
#endif
#if !__HBD_PFA_ENABLE__
    #undef __PFA_DET_SUPPORT__
    #define __PFA_DET_SUPPORT__ (0)
#endif


/// dbg output rawdata macro
#if ((__GOODIX_APP_MCU_SUP__) || (__ALGO_CALC_WITH_DBG_DATA__))
    #define __DBG_OUTPUT_RAWDATA__          (1)
#else
    #define __DBG_OUTPUT_RAWDATA__          (0)
#endif


/// define all dbg mcu mode api
#if ((__USE_GOODIX_APP__) && (__GOODIX_APP_MCU_SUP__))
#define BLE_MODULE_SEND_HB(val)
#define BLE_MODULE_SEND_RRI(val, cnt)
#define SEND_GH30X_RESET_EVT()                                          comm_send_app_cmd_reset(ble_module_send_pkg)
#define SEND_AUTOLED_FAIL_EVT()                                         comm_send_app_cmd_auto_led_fail(ble_module_send_pkg)
#define SEND_MCU_HB_MODE_WEAR_STATUS(sta, data, len)                    send_mcu_hb_mode_wear_status_pkg(sta, data, len)
#define SEND_MCU_HB_MODE_RESULT(hb, lv, sta, wq, vb, ret, rr, data, len)    \
                                                                        send_mcu_hb_mode_result_pkg(hb, lv, sta, wq, vb, ret, rr, data, len)
#define SEND_MCU_HRV_MODE_RESULT(rr_arr, cnt, lvl, data, len)           send_mcu_hrv_mode_result_pkg(rr_arr, cnt, lvl, data, len)
#define SEND_MCU_SPO2_UNWEAR_EVT(data, len)                             send_mcu_spo2_mode_unwear_pkg(data, len)
#define SEND_MCU_SPO2_MODE_RESULT(spo2, spo2_lv, hb, hb_lv, rr, rr_lv, rr_c, spo2_r, sta, vd, data, len)     \
                                                                        send_mcu_spo2_mode_result_pkg(spo2, spo2_lv, hb, hb_lv, rr, rr_lv, rr_c, spo2_r, sta, vd, data, len)
#else
#define BLE_MODULE_SEND_HB(val)                                         ble_module_send_heartrate(val)
#define BLE_MODULE_SEND_RRI(val, cnt)                                   do { ble_module_add_rr_interval(val, cnt); ble_module_send_heartrate(0); } while (0)
#define SEND_GH30X_RESET_EVT()
#define SEND_AUTOLED_FAIL_EVT()
#define SEND_MCU_HB_MODE_WEAR_STATUS(sta, data, len)
#define SEND_MCU_HB_MODE_RESULT(hb, lv, sta, wq, vb, ret, rr, data, len)
#define SEND_MCU_HRV_MODE_RESULT(rr_arr, cnt, lvl, data, len)
#define SEND_MCU_SPO2_UNWEAR_EVT(data, len)
#define SEND_MCU_SPO2_MODE_RESULT(spo2, spo2_lv, hb, hb_lv, rr, rr_lv, rr_c, spo2_r, sta, vd, data, len)
#endif

/// define all fifo int timeout timer api
#if (__FIFO_INT_TIMEOUT_CHECK__)
    #define HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_INIT()                         hal_gh30x_fifo_int_timeout_timer_init()
    #define HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START()                        hal_gh30x_fifo_int_timeout_timer_start()
    #define HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_STOP()                         hal_gh30x_fifo_int_timeout_timer_stop()
#else
    #define HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_INIT()
    #define HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_START()
    #define HAL_GH30X_FIFO_INT_TIMEOUT_TIMER_STOP()
#endif


/// ok and err val for some api ret
#define GH30X_EXAMPLE_OK_VAL                (1)
#define GH30X_EXAMPLE_ERR_VAL               (0)

/// autoled error val
#define GH30X_AUTOLED_ERR_VAL               (1)

/// interface type
#define GH30X_COMMUNICATION_INTERFACE_I2C   (1)
#define GH30X_COMMUNICATION_INTERFACE_SPI   (2)

/// fifo thr default
#define COMMON_FIFO_THR_DEFAULT             (25)  /* include hb, spo2, hsm, bpd, pfa*/
#define HRV_FIFO_THR_DEFAULT                (100)

/// autoled bg thr default
#define AUTOLED_BG_THR_MAX_DEFAULT          (6)
#define AUTOLED_BG_THR_MIN_DEFAULT          (2)

/// run mode
#define RUN_MODE_INVALID                    (0)
#define RUN_MODE_ADT_HB_DET                 (2)
#define RUN_MODE_HRV_DET                    (3)
#define RUN_MODE_HSM_DET                    (4)
#define RUN_MODE_BPD_DET                    (5)
#define RUN_MODE_PFA_DET                    (6)
#define RUN_MODE_SPO2_DET                   (7)

/// wear status
#define WEAR_STATUS_DETECTING               (0)
#define WEAR_STATUS_WEAR                    (1)
#define WEAR_STATUS_UNWEAR                  (2)
#define WEAR_STATUS_ALMOST_UNWEAR           (3)

/// adt confirm status
#define ADT_CONFRIM_STATUS_DETECTING        (0)
#define ADT_CONFRIM_STATUS_WEAR             (0x11)
#define ADT_CONFRIM_STATUS_UNWEAR           (0x12)

// hb algorithm scenario macro
#define HBA_SCENARIO_DEFAULT                (0)
#define HBA_SCENARIO_ROUTINE                (1)
#define HBA_SCENARIO_RUN                    (2)
#define HBA_SCENARIO_CLIBM                  (3)
#define HBA_SCENARIO_BIKE                   (4)
#define HBA_SCENARIO_IRREGULAR              (5)

/// fifo thr config type
#define FIFO_THR_CONFIG_TYPE_HRV            (0)
#define FIFO_THR_CONFIG_TYPE_HB             (1)
#define FIFO_THR_CONFIG_TYPE_HSM            (2)
#define FIFO_THR_CONFIG_TYPE_BPD            (3)
#define FIFO_THR_CONFIG_TYPE_PFA            (4)
#define FIFO_THR_CONFIG_TYPE_SPO2           (5)

/// gsensor soft fifo buffer len
#define GSENSOR_SOFT_FIFO_BUFFER_MAX_LEN    (101)

/// mutli pkg magic byte
#define MUTLI_PKG_HEADER_LEN                (4)
#define MUTLI_PKG_MAGIC_0_VAL               (0x47)
#define MUTLI_PKG_MAGIC_1_VAL               (0x44)
#define MUTLI_PKG_MAGIC_2_VAL               (0x0A)
#define MUTLI_PKG_CMD_VERIFY(cmd)           ((cmd & 0xF0) == 0x80)
#define MUTLI_PKG_UART_MAX_LEN              (20)

/// mcu pkg
#define MCU_PKG_HB_TYPE_VAL                 (0)
#define MCU_PKG_HRV_TYPE_VAL                (1)
#define MCU_PKG_HSM_TYPE_VAL                (2)
#define MCU_PKG_BPD_TYPE_VAL                (3)
#define MCU_PKG_PFA_TYPE_VAL                (4)
#define MCU_PKG_SPO2_TYPE_VAL               (5)

#define MCU_PKG_HB_ALGO_RESULT_LEN          (8)
#define MCU_PKG_HRV_ALGO_RESULT_LEN         (10)
#define MCU_PKG_SPO2_WEAR_RESULT_LEN        (16)
#define MCU_PKG_SPO2_ALGO_RESULT_LEN        (17)

/// spo2 mcu unwear
#define SPO2_MCU_UNWEAR_VAL                 (255)

/// fix spo2 mcu val
#define FIXED_SPO2_ALGO_RES(val)            do { if ((val) == SPO2_MCU_UNWEAR_VAL) { (val) = 0; } } while (0)

/// mcu rawdata cnt
#define DBG_MCU_MODE_PKG_LEN                (6)

/// mcu rawdata frame len
#define DBG_MCU_PKG_RAW_FRAME_LEN           (13)

/// mcu pkg header len
#define DBG_MCU_PKG_HEADER_LEN              (8)

/// hrv output res max len
#define HRV_MODE_RES_MAX_CNT                (8)

/// comm type val
#define COMM_TYPE_INVALID_VAL               (0xFF)

/// hbd ret val base
#define DEBUG_HBD_RET_VAL_BASE              (7)

/* debug log control, if enable debug, example_dbg_log must define in platforms code */
#if (__EXAMPLE_DEBUG_LOG_LVL__) // debug level > 0
/// dbg run mode string
extern const char dbg_rum_mode_string[][24];
/// dbg communicate cmd string
extern const char dbg_comm_cmd_string[][35];
/// dbg ret val string
extern const char dbg_ret_val_string[][35];
/// lv1 log string
#define   EXAMPLE_DEBUG_LOG_L1(...)       do {\
                                                    char dbg_log_str[__EXAMPLE_LOG_DEBUG_SUP_LEN__] = {0};\
                                                    snprintf(dbg_log_str, __EXAMPLE_LOG_DEBUG_SUP_LEN__, \
                                                            "[gh30x]: "__VA_ARGS__);\
                                                    example_dbg_log(dbg_log_str);\
                                                } while (0)
#if (__EXAMPLE_DEBUG_LOG_LVL__ > 1) // debug level > 1
/// lv2 log string
#define   EXAMPLE_DEBUG_LOG_L2(...)        { EXAMPLE_DEBUG_LOG_L1(__VA_ARGS__); }
#define   EXAMPLE_LOG_RAWDARA(str, dat, len)    do { \
                                                            EXAMPLE_DEBUG_LOG_L2(str);\
                                                            uint8_t log_index = 0; \
                                                            for (log_index = 0; log_index < len; log_index++) \
                                                            {\
                                                                EXAMPLE_DEBUG_LOG_L2("raw,%ld,%ld,%ld,%ld,%ld,%ld\r\n", \
                                                                            dat[log_index * DBG_MCU_MODE_PKG_LEN + 0], \
                                                                            dat[log_index * DBG_MCU_MODE_PKG_LEN + 1], \
                                                                            dat[log_index * DBG_MCU_MODE_PKG_LEN + 2], \
                                                                            dat[log_index * DBG_MCU_MODE_PKG_LEN + 3], \
                                                                            dat[log_index * DBG_MCU_MODE_PKG_LEN + 4], \
                                                                            dat[log_index * DBG_MCU_MODE_PKG_LEN + 5]); \
                                                            }\
                                                        } while (0)
#else
#define   EXAMPLE_DEBUG_LOG_L2(...)
#define   EXAMPLE_LOG_RAWDARA(str, dat, len)
#endif
#else   // debug level <= 0
#define   EXAMPLE_DEBUG_LOG_L1(...)
#define   EXAMPLE_DEBUG_LOG_L2(...)
#define   EXAMPLE_LOG_RAWDARA(str, dat, len)
#endif

//// get bytes macro
#define   GET_BYTE3_FROM_DWORD(val)     ((uint8_t)((val >> 24) & 0x000000FFU))
#define   GET_BYTE2_FROM_DWORD(val)     ((uint8_t)((val >> 16) & 0x000000FFU))
#define   GET_BYTE1_FROM_DWORD(val)     ((uint8_t)((val >> 8) & 0x000000FFU))
#define   GET_BYTE0_FROM_DWORD(val)     ((uint8_t)((val ) & 0x000000FFU))
#define   GET_HIGH_BYTE_FROM_WORD(val)  ((uint8_t)(((val) >> 8) & 0xFF))
#define   GET_LOW_BYTE_FROM_WORD(val)   ((uint8_t)((val) & 0xFF))

/// config var

// init
extern ST_HBD_INIT_CONFIG_TYPE gh30x_init_config;

// hb
extern const ST_REGISTER hb_adt_confirm_reg_config[];
extern const uint8_t hb_adt_confirm_reg_config_len;
extern const ST_REGISTER hb_reg_config_array[];
extern const uint8_t hb_reg_config_array_len;
extern const ST_REGISTER hb_wear_confirm_reg_config_array[];
extern const uint8_t hb_wear_confirm_reg_config_array_len;

// spo2
extern const ST_REGISTER spo2_reg_config_array[];
extern const uint8_t spo2_reg_config_array_len;

// hrv
extern const ST_REGISTER hrv_reg_config_array[];
extern const uint8_t hrv_reg_config_array_len;

// hsm
extern const ST_REGISTER hsm_reg_config_array[];
extern const uint8_t hsm_reg_config_array_len;

// bpd
extern const ST_REGISTER bpd_reg_config_array[];
extern const uint8_t bpd_reg_config_array_len;

// pfa
extern const ST_REGISTER pfa_reg_config_array[];
extern const uint8_t pfa_reg_config_array_len;

// system test
extern const ST_REGISTER systemtest_led0_reg_config_array[];
extern const uint8_t systemtest_led0_reg_config_array_len;

extern const ST_REGISTER systemtest_led1_reg_config_array[];
extern const uint8_t systemtest_led1_reg_config_array_len;

extern const ST_REGISTER systemtest_led2_reg_config_array[];
extern const uint8_t systemtest_led2_reg_config_array_len;

#if (__SYSTEM_TEST_SUPPORT__)
    extern ROMAHBData systemtest_led0_os_result;
    extern ROMAHBData systemtest_led1_os_result;
    extern ROMAHBData systemtest_led2_os_result;
#endif


/// porting api

/// i2c
void hal_gh30x_i2c_init(void);
uint8_t hal_gh30x_i2c_write(uint8_t device_id, const uint8_t write_buffer[], uint16_t length);
uint8_t hal_gh30x_i2c_read(uint8_t device_id, const uint8_t write_buffer[], uint16_t write_length, uint8_t read_buffer[], uint16_t read_length);

/// spi
void hal_gh30x_spi_init(void);
uint8_t hal_gh30x_spi_write(const uint8_t write_buffer[], uint16_t length);
uint8_t hal_gh30x_spi_read(uint8_t read_buffer[], uint16_t length);
void hal_gh30x_spi_cs_set_low(void);
void hal_gh30x_spi_cs_set_high(void);

/// delay
void hal_gh30x_delay_us(uint16_t us_cnt);

/// gsensor driver
int8_t gsensor_drv_init(void);
void gsensor_drv_enter_normal_mode(void);
void gsensor_drv_enter_fifo_mode(void);
void gsensor_drv_enter_motion_det_mode(void);
void gsensor_drv_get_fifo_data(ST_GS_DATA_TYPE gsensor_buffer[], uint16_t *gsensor_buffer_index, uint16_t gsensor_max_len);
void gsensor_drv_clear_buffer(ST_GS_DATA_TYPE gsensor_buffer[], uint16_t *gsensor_buffer_index, uint16_t gsensor_buffer_len);
void gsensor_drv_get_data(ST_GS_DATA_TYPE *gsensor_data_ptr);
void gsensor_drv_int1_handler(void);

/// int
void hal_gh30x_int_init(void);
void hal_gsensor_int1_init(void);

/// handle result
void handle_hb_mode_result(uint8_t hb_val, uint8_t hb_lvl_val, uint8_t wearing_state_val, uint16_t rr_val, int32_t rawdata_ptr[][DBG_MCU_MODE_PKG_LEN], uint16_t rawdata_len);
void handle_spo2_mode_result(uint8_t spo2_val, uint8_t spo2_lvl_val, uint8_t hb_val, uint8_t hb_lvl_val, uint16_t rr_val[4], uint8_t rr_lvl_val, uint8_t rr_cnt,
                             uint16_t spo2_r_val, uint8_t wearing_state_val, int32_t rawdata_ptr[][DBG_MCU_MODE_PKG_LEN], uint16_t rawdata_len);
void handle_hrv_mode_result(uint16_t rr_val_arr[HRV_MODE_RES_MAX_CNT], uint8_t rr_val_cnt, uint8_t rr_lvl, int32_t rawdata_ptr[][DBG_MCU_MODE_PKG_LEN], uint16_t rawdata_len);
void handle_hsm_mode_result(uint8_t hb_val, uint8_t sleep_state_val, uint8_t respiratory_rate_val, uint16_t asleep_time_val,
                            uint16_t wakeup_time_val, int32_t rawdata_ptr[][DBG_MCU_MODE_PKG_LEN], uint16_t rawdata_len);
void handle_bpd_mode_result(uint16_t sbp_val, uint16_t dbp_val, int32_t rawdata_ptr[][DBG_MCU_MODE_PKG_LEN], uint16_t rawdata_len);
void handle_pfa_mode_result(uint8_t pressure_level_val, uint8_t fatigue_level_val, uint8_t body_age_val, int32_t rawdata_ptr[][DBG_MCU_MODE_PKG_LEN], uint16_t rawdata_len);
void handle_wear_status_result(uint8_t wearing_state_val);
void handle_system_test_otp_check_result(uint8_t otp_res);
void handle_system_test_os_result(uint8_t led_num, uint8_t os_res);

/// ble
void ble_module_send_heartrate(uint32_t heartrate); // send value via heartrate profile
void ble_module_add_rr_interval(uint16_t rr_interval_arr[], uint8_t cnt); // add value to heartrate profile
uint8_t ble_module_send_data_via_gdcs(uint8_t string[], uint8_t length); // send value via through profile

// ble repeat send timer
void ble_module_repeat_send_timer_start(void);
void ble_module_repeat_send_timer_stop(void);
void ble_module_repeat_send_timer_init(void);

// fifo int timeout
void hal_gh30x_fifo_int_timeout_timer_start(void);
void hal_gh30x_fifo_int_timeout_timer_stop(void);
void hal_gh30x_fifo_int_timeout_timer_init(void);

// uart
uint8_t uart_module_send_data(uint8_t string[], uint8_t length);

// handle cmd
void handle_goodix_communicate_cmd(EM_COMM_CMD_TYPE cmd_type);

// log
void example_dbg_log(char *log_string);


/// ble pkg api
void uart_module_handle_recv_byte(uint8_t recv_byte);
void uart_module_send_pkg(uint8_t string[], uint8_t length);
void gh30x_comm_pkg_init(void);
void gh30x_app_cmd_parse(uint8_t *buffer, uint8_t length); // ble recv data parse
void gh30x_uart_cmd_parse(uint8_t *buffer, uint8_t length); // uart recv data parse
void ble_module_send_pkg(uint8_t string[], uint8_t length);
void send_mcu_rawdata_packet_repeat(void);
void send_mcu_hb_mode_wear_status_pkg(uint8_t wear_status, GS32 rawdata_dbg[][DBG_MCU_MODE_PKG_LEN], uint8_t rawdata_dbg_len);
void send_mcu_hb_mode_result_pkg(uint8_t hb_val, uint8_t hb_lvl, uint8_t wear_status, uint8_t wearing_q, uint8_t vb_val, uint8_t clac_ret, uint16_t rr_val,
                                 GS32 rawdata_dbg[][DBG_MCU_MODE_PKG_LEN], uint8_t rawdata_dbg_len);
void send_mcu_hrv_mode_result_pkg(uint16_t rr_val_arr[], uint8_t rr_cnt, uint8_t rr_lvl, GS32 rawdata_dbg[][DBG_MCU_MODE_PKG_LEN], uint8_t rawdata_dbg_len);
void send_mcu_spo2_mode_unwear_pkg(GS32 rawdata_dbg[][DBG_MCU_MODE_PKG_LEN], uint8_t rawdata_dbg_len);
void send_mcu_spo2_mode_result_pkg(uint8_t spo2_val, uint8_t spo2_lvl_val, uint8_t hb_val, uint8_t hb_lvl_val, uint16_t rr_val[4], uint8_t rr_lvl_val, uint8_t rr_cnt,
                                   uint16_t spo2_r_val, uint8_t wearing_state_val, uint8_t valid_lvl_val, GS32 rawdata_dbg[][DBG_MCU_MODE_PKG_LEN], uint16_t rawdata_dbg_len);


/// ctrl api
void gh30x_Load_new_config(const ST_REGISTER *config_ptr, uint16_t len);
void gh30x_adt_wear_detect_start(const ST_REGISTER *config_ptr, uint16_t config_len);
void gh30x_adt_confirm_start(void);
#if (__HB_DET_SUPPORT__)
    void gh30x_hb_start(void);
#endif
void gh30x_wear_confirm_start(void);
#if (__HRV_DET_SUPPORT__)
    void gh30x_hrv_start(void);
#endif
#if (__HSM_DET_SUPPORT__)
    void gh30x_hsm_start(void);
#endif
#if (__BPD_DET_SUPPORT__)
    void gh30x_bpd_start(void);
#endif
#if (__PFA_DET_SUPPORT__)
    void gh30x_pfa_start(void);
#endif
#if (__SPO2_DET_SUPPORT__)
    void gh30x_spo2_start(void);
#endif
uint8_t gh30x_i2c_write_exchange_to_spi(uint8_t device_id, const uint8_t write_buffer[], uint16_t length);
uint8_t gh30x_i2c_read_exchange_to_spi(uint8_t device_id, const uint8_t write_buffer[], uint16_t write_length, uint8_t read_buffer[], uint16_t read_length);
uint8_t gh30x_systemtest_otp_check(void);
void gh30x_systemtest_os_start(uint8_t led_num);
uint8_t gh30x_systemtest_os_calc(uint8_t led_num);

/// process api
void gh30x_start_adt_with_mode(uint8_t adt_run_mode);
void gh30x_start_func_with_mode(uint8_t func_run_mode);
void gh30x_start_func_whithout_adt_confirm(uint8_t start_run_mode);
void gh30x_stop_func(void);

void gsensor_enter_clear_buffer_and_enter_fifo(void);
void gsensor_read_fifo_data(void);
void gsensor_enter_normal_and_clear_buffer(void);
void gsensor_motion_has_detect(void);

void gh30x_communicate_parse_handler(int8_t communicate_type, uint8_t *buffer, uint8_t length);
void gh30x_int_msg_handler(void); // gh30x int msg handler
void gh30x_fifo_int_timeout_msg_handler(void);

/// hbd ctrl extern api & var
extern void HBD_DelayUs(GU16 usUsCnt);
extern void HBD_I2cSendCmd(GU8 uchCmd);
extern void HBD_I2cWriteReg(GU16 usAddr, GU16 usValue);
extern GU16 HBD_I2cReadReg(GU16 usAddr);
extern GU32 HBD_I2cReadRawdataReg(GU16 usAddr);
extern GU16 g_usReadRawdataRegList[2];

#endif /* _GH3011_EXAMPLE_COMMON_H_ */

/********END OF FILE********* Copyright (c) 2003 - 2020, Goodix Co., Ltd. ********/
