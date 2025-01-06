/**
 * @copyright (c) 2003 - 2020, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3011_example_port.c
 *
 * @brief   example code for gh3011 (condensed  hbd_ctrl lib)
 *
 */

#include <rtthread.h>
#include <math.h>
#include "stdlib.h"
#include "board.h"

#define DRV_DEBUG
#define LOG_TAG              "drv.hbd"
#include <drv_log.h>

#include "gh3011_example_common.h"

static struct rt_i2c_bus_device *gh3011_i2cbus = NULL;
static struct rt_semaphore gh3011_int_sem;
rt_thread_t gh3011_thread = NULL;
static uint8_t loc_hb_value;
static uint8_t loc_spo2_value;

/* gh30x i2c interface */

/// reset pin init for gh30x
void hal_gh30x_pin_set(uint8_t en)
{
    struct rt_device_pin_mode m;
    struct rt_device_pin_status st;

    // get pin device
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        LOG_E("GPIO pin device not found at GH3011\n");
        return ;
    }

    rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
    // set reset pin to 1
    m.pin = GH3011_RST_PIN + 1;
    m.mode = PIN_MODE_OUTPUT;
    rt_device_control(device, 0, &m);

    st.pin = GH3011_RST_PIN;
    st.status = 0;
    rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));
    // move to power up
    if (en)
    {
        rt_thread_delay(2);

        st.pin = GH3011_RST_PIN;
        st.status = 1;
        rt_device_write(device, 0, &st, sizeof(struct rt_device_pin_status));
    }
}
/// i2c for gh30x init
void hal_gh30x_i2c_init(void)
{
    // code implement by user
#if 1
    hal_gh30x_pin_set(1);
#endif

    /* get i2c bus device */
    gh3011_i2cbus = rt_i2c_bus_device_find(GH3011_I2C_BUS);
    if (gh3011_i2cbus)
    {
        LOG_D("Find i2c bus device %s\n", GH3011_I2C_BUS);
    }
    else
    {
        LOG_E("Can not found i2c bus %s, hal_gh30x_i2c_init fail\n", GH3011_I2C_BUS);
    }
}

/// i2c for gh30x wrtie
uint8_t hal_gh30x_i2c_write(uint8_t device_id, const uint8_t write_buffer[], uint16_t length)
{
    uint8_t ret = GH30X_EXAMPLE_OK_VAL;
    // code implement by user

    struct rt_i2c_msg msgs[2];
    uint32_t res;

    if (gh3011_i2cbus)
    {
        msgs[0].addr  = device_id >> 1;  /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = (uint8_t *)write_buffer;             /* Slave register address */
        msgs[0].len   = length;                /* Number of bytes sent */

        res = rt_i2c_transfer(gh3011_i2cbus, msgs, 1);
        if (res == 1)
        {
            //LOG_D("GH3011_I2C_Write OK: 0x%x, %d\n", device_id, length);
            ret = GH30X_EXAMPLE_OK_VAL;
        }
        else
        {
            LOG_I("hal_gh30x_i2c_write FAIL: 0x%x, %d,  %d\n", device_id, length, res);
            ret = GH30X_EXAMPLE_ERR_VAL;
        }
    }
    return ret;
}

/// i2c for gh30x read
uint8_t hal_gh30x_i2c_read(uint8_t device_id, const uint8_t write_buffer[], uint16_t write_length, uint8_t read_buffer[], uint16_t read_length)
{
    uint8_t ret = GH30X_EXAMPLE_OK_VAL;
    // code implement by user
    struct rt_i2c_msg msgs[2];
    uint32_t res;

    if (gh3011_i2cbus)
    {
        msgs[0].addr  = device_id >> 1;  /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = (uint8_t *)write_buffer;         /* Slave register address */
        msgs[0].len   = write_length;                /* Number of bytes sent */

        msgs[1].addr  = device_id >> 1;  /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = read_buffer;             /* Read data pointer */
        msgs[1].len   = read_length;              /* Number of bytes read */

        res = rt_i2c_transfer(gh3011_i2cbus, msgs, 2);
        if (res == 2)
        {
            //LOG_D("GH3011_I2C_Read OK: 0x%x\n", device_id);
            ret = GH30X_EXAMPLE_OK_VAL;
        }
        else
        {
            LOG_I("hal_gh30x_i2c_read FAIL: 0x%x,  %d\n", device_id, res);
            ret = GH30X_EXAMPLE_ERR_VAL;
        }

    }
    return ret;
}

/* gh30x spi interface */

/// spi for gh30x init
void hal_gh30x_spi_init(void)
{
    // code implement by user
}

/// spi for gh30x wrtie
uint8_t hal_gh30x_spi_write(const uint8_t write_buffer[], uint16_t length)
{
    uint8_t ret = 1;
    // code implement by user
    return ret;
}

/// spi for gh30x read
uint8_t hal_gh30x_spi_read(uint8_t read_buffer[], uint16_t length)
{
    uint8_t ret = 1;
    // code implement by user
    return ret;
}

/// spi cs set low for gh30x
void hal_gh30x_spi_cs_set_low(void)
{
    // code implement by user
}

/// spi cs set high for gh30x
void hal_gh30x_spi_cs_set_high(void)
{
    // code implement by user
}


/* delay */

/// delay us
void hal_gh30x_delay_us(uint16_t us_cnt)
{
    // code implement by user
    //HAL_Delay_us(us_cnt);
    rt_thread_delay(1);
}

/* gsensor driver */

/// gsensor motion detect mode flag
bool gsensor_drv_motion_det_mode = false;

/// gsensor init
int8_t gsensor_drv_init(void)
{
    int8_t ret = GH30X_EXAMPLE_OK_VAL;
    gsensor_drv_motion_det_mode = false;
    // code implement by user
    /* if enable all func equal 25Hz, should config > 25Hz;
    but if enable have 100hz, should config to > 100hz. if not, feeback to GOODIX!!!
    */
    return ret;
}

/// gsensor enter normal mode
void gsensor_drv_enter_normal_mode(void)
{
    // code implement by user
    gsensor_drv_motion_det_mode = false;
}

/// gsensor enter fifo mode
void gsensor_drv_enter_fifo_mode(void)
{
    // code implement by user
    gsensor_drv_motion_det_mode = false;
}

/// gsensor enter motion det mode
void gsensor_drv_enter_motion_det_mode(void)
{
    // code implement by user
    /* if enable motion det mode that call @ref hal_gsensor_drv_int1_handler when motion generate irq
        e.g. 1. (hardware) use gsensor motion detect module with reg config
             2. (software) gsensor enter normal mode, then define 30ms timer get gsensor rawdata,
                if now total acceleration(sqrt(x*x+y*y+z*z)) - last total acceleration >= 30 (0.05g @512Lsb/g) as motion
                generate that call @ref hal_gsensor_drv_int1_handler
    */
    gsensor_drv_motion_det_mode = true;
}

/// gsensor get fifo data
void gsensor_drv_get_fifo_data(ST_GS_DATA_TYPE gsensor_buffer[], uint16_t *gsensor_buffer_index, uint16_t gsensor_max_len)
{
    // code implement by user
}

/// gsensor clear buffer
void gsensor_drv_clear_buffer(ST_GS_DATA_TYPE gsensor_buffer[], uint16_t *gsensor_buffer_index, uint16_t gsensor_buffer_len)
{
    if ((gsensor_buffer != NULL) && (gsensor_buffer_index != NULL))
    {
        memset(gsensor_buffer, 0, sizeof(ST_GS_DATA_TYPE) * gsensor_buffer_len);
        *gsensor_buffer_index = 0;
    }
}

/// gsensor get data
void gsensor_drv_get_data(ST_GS_DATA_TYPE *gsensor_data_ptr)
{
    // code implement by user
}


/* int */

/// gh30x int handler
void hal_gh30x_int_handler(void)
{
    gh30x_int_msg_handler();
}

static void gh30x_int_handle(void *args)
{
    rt_sem_release(&gh3011_int_sem);
}
// start a thread to process semphone
static void gh3011_sensor_task(void *params)
{
    // add by whj for thread loop
    while (1)
    {
        rt_sem_take(&gh3011_int_sem, RT_WAITING_FOREVER) ;
        LOG_I("GH3011 int\n");
        hal_gh30x_int_handler();
    }
}

/// gh30x int pin init, should config as rise edge trigger
void hal_gh30x_int_init(void)
{
    struct rt_device_pin_mode m;
    struct rt_device_pin_status st;

    // get pin device
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        LOG_E("GPIO pin device not found at GH3011\n");
        return ;
    }

    rt_device_open(device, RT_DEVICE_OFLAG_RDWR);

    // int pin cfg
    m.pin = GH3011_INT_BIT;
    m.mode = PIN_MODE_INPUT;
    rt_device_control(device, 0, &m);

    // enable gh3011 int
    rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_RISING, gh30x_int_handle, (void *)(rt_uint32_t)m.pin);
    rt_pin_irq_enable(m.pin, 1);

    // start a thread to check data available
    rt_sem_init(&gh3011_int_sem, "gh3011_int", 0, RT_IPC_FLAG_FIFO);
    gh3011_thread = rt_thread_create("gh3011", gh3011_sensor_task, NULL, 6144, RT_THREAD_PRIORITY_MIDDLE, RT_THREAD_TICK_DEFAULT);
    if (gh3011_thread != NULL)
    {
        rt_thread_startup(gh3011_thread);
        LOG_D("gh3011 thread started\n");
    }
    else
        LOG_E("Create gh3011 thread fail\n");

}

/// gsensor int handler
void hal_gsensor_drv_int1_handler(void)
{
// code implement by user
    if (gsensor_drv_motion_det_mode)
    {
        gsensor_motion_has_detect();
    }
    else
    {
        /* if using gsensor fifo mode, should get data by fifo int
            * e.g. gsensor_read_fifo_data();
        */
        gsensor_read_fifo_data(); // got fifo int
    }
}

/// gsensor int1 init, should config as rise edge trigger
void hal_gsensor_int1_init(void)
{
    // code implement by user
    // must register func hal_gsensor_drv_int1_handler as callback

    /* if using gsensor fifo mode,
    and gsensor fifo depth is not enough to store 1 second data,
    please connect gsensor fifo interrupt to the host,
    or if using gsensor motion detect mode(e.g  motion interrupt response by 0.5G * 5counts),
    and implement this function to receive gsensor interrupt.
    */
}


/* handle algorithm result */

/// handle hb mode algorithm result
void handle_hb_mode_result(uint8_t hb_val, uint8_t hb_lvl_val, uint8_t wearing_state_val, uint16_t rr_val, int32_t rawdata_ptr[][DBG_MCU_MODE_PKG_LEN],
                           uint16_t rawdata_len)
{
    // code implement by user
    LOG_D("%s: %d", __FUNCTION__, hb_val);
    loc_hb_value = hb_val;
}

/// handle spo2 mode algorithm result
void handle_spo2_mode_result(uint8_t spo2_val, uint8_t spo2_lvl_val, uint8_t hb_val, uint8_t hb_lvl_val, uint16_t rr_val[4], uint8_t rr_lvl_val, uint8_t rr_cnt,
                             uint16_t spo2_r_val, uint8_t wearing_state_val, int32_t rawdata_ptr[][DBG_MCU_MODE_PKG_LEN], uint16_t rawdata_len)
{
    // code implement by user
    LOG_D("%s: %d", __FUNCTION__, spo2_val);
    loc_spo2_value = spo2_val;
}

/// handle hrv mode algorithm result
void handle_hrv_mode_result(uint16_t rr_val_arr[HRV_MODE_RES_MAX_CNT], uint8_t rr_val_cnt, uint8_t rr_lvl, int32_t rawdata_ptr[][DBG_MCU_MODE_PKG_LEN], uint16_t rawdata_len)
{
    // code implement by user
    LOG_D("%s: %d", __FUNCTION__, rr_val_cnt);
}

/// handle hsm mode algorithm result
void handle_hsm_mode_result(uint8_t hb_val, uint8_t sleep_state_val, uint8_t respiratory_rate_val, uint16_t asleep_time_val,
                            uint16_t wakeup_time_val, int32_t rawdata_ptr[][DBG_MCU_MODE_PKG_LEN], uint16_t rawdata_len)
{
    // code implement by user
    LOG_D("%s: %d", __FUNCTION__, hb_val);
}

/// handle bpd mode algorithm result
void handle_bpd_mode_result(uint16_t sbp_val, uint16_t dbp_val, int32_t rawdata_ptr[][DBG_MCU_MODE_PKG_LEN], uint16_t rawdata_len)
{
    // code implement by user
    LOG_D("%s: %d", __FUNCTION__, sbp_val);
}

/// handle pfa mode algorithm result
void handle_pfa_mode_result(uint8_t pressure_level_val, uint8_t fatigue_level_val, uint8_t body_age_val,
                            int32_t rawdata_ptr[][DBG_MCU_MODE_PKG_LEN], uint16_t rawdata_len)
{
    // code implement by user
    LOG_D("%s: %d", __FUNCTION__, pressure_level_val);
}

/// handle wear status result
void handle_wear_status_result(uint8_t wearing_state_val)
{
    // code implement by user
    LOG_D("%s: %d", __FUNCTION__, wearing_state_val);
}

/// handle wear status result, otp_res: <0=> ok , <1=> err , <2=> para err
void handle_system_test_otp_check_result(uint8_t otp_res)
{
    // code implement by user
}

/// handle wear status result, led_num: {0-2};os_res: <0=> ok , <1=> rawdata err , <2=> noise err , <3=> para err
void handle_system_test_os_result(uint8_t led_num, uint8_t os_res)
{
    // code implement by user
}

/* ble */

/// send value via heartrate profile
void ble_module_send_heartrate(uint32_t heartrate)
{
    // code implement by user
}

/// add value to heartrate profile
void ble_module_add_rr_interval(uint16_t rr_interval_arr[], uint8_t cnt)
{
    // code implement by user
}

/// send string via through profile
uint8_t ble_module_send_data_via_gdcs(uint8_t string[], uint8_t length)
{
    uint8_t ret = GH30X_EXAMPLE_OK_VAL;
    // code implement by user
    return ret;
}

/// recv data via through profile
void ble_module_recv_data_via_gdcs(uint8_t *data, uint8_t length)
{
    gh30x_app_cmd_parse(data, length);
}


/* timer */

/// gh30x fifo int timeout timer handler
void hal_gh30x_fifo_int_timeout_timer_handler(void)
{
    LOG_D("%s", __FUNCTION__);
    gh30x_fifo_int_timeout_msg_handler();
}

/// gh30x fifo int timeout timer start
void hal_gh30x_fifo_int_timeout_timer_start(void)
{
    // code implement by user
    LOG_D("%s", __FUNCTION__);
}

/// gh30x fifo int timeout timer stop
void hal_gh30x_fifo_int_timeout_timer_stop(void)
{
    // code implement by user
    LOG_D("%s", __FUNCTION__);
}

/// gh30x fifo int timeout timer init
void hal_gh30x_fifo_int_timeout_timer_init(void)
{
    // code implement by user
    // must register func gh30x_fifo_int_timeout_timer_handler as callback
    /* should setup timer interval with fifo int freq(e.g. 1s fifo int setup 1080ms timer)
    */
    LOG_D("%s", __FUNCTION__);
}

/// ble repeat send data timer handler
void ble_module_repeat_send_timer_handler(void)
{
#if ((__USE_GOODIX_APP__) && (__GOODIX_APP_MCU_SUP__))
    send_mcu_rawdata_packet_repeat();
#endif
}

/// ble repeat send data timer start
void ble_module_repeat_send_timer_start(void)
{
    // code implement by user
}

/// ble repeat send data timer stop
void ble_module_repeat_send_timer_stop(void)
{
    // code implement by user
}

/// ble repeat send data timer init
void ble_module_repeat_send_timer_init(void)
{
    // code implement by user
    // must register func ble_module_repeat_send_timer_handler as callback
    /* should setup 100ms timer and ble connect interval should < 100ms
    */
}


/* uart */

/// recv data via uart
void uart_module_recv_data(uint8_t *data, uint8_t length)
{
    gh30x_uart_cmd_parse(data, length);
}

/// send value via uart
uint8_t uart_module_send_data(uint8_t string[], uint8_t length)
{
    uint8_t ret = GH30X_EXAMPLE_OK_VAL;
    // code implement by user
    return ret;
}


/* handle cmd with all ctrl cmd @ref EM_COMM_CMD_TYPE */
void handle_goodix_communicate_cmd(EM_COMM_CMD_TYPE cmd_type)
{
    // code implement by user
}


/* log */

/// print dbg log
void example_dbg_log(char *log_string)
{
    // code implement by user
    LOG_D("%s", log_string);
}


void comm_send_ext_packet(void (*pSendDataFunc)(uint8_t uchDataBuffArr[], uint8_t uchLen), uint8_t data_array[], uint8_t data_len)
{
}

void comm_send_app_cmd(void (*pSendDataFunc)(uint8_t uchDataBuffArr[], uint8_t uchLen), uint8_t dev_state, uint8_t data_array[], uint8_t data_len)
{
}

void comm_send_app_cmd_auto_led_fail(void (*pSendDataFunc)(uint8_t uchDataBuffArr[], uint8_t uchLen))
{
}
void comm_send_app_cmd_reset(void (*pSendDataFunc)(uint8_t uchDataBuffArr[], uint8_t uchLen))
{
}
void comm_send_app_cmd_battery_low(void (*pSendDataFunc)(uint8_t uchDataBuffArr[], uint8_t uchLen))
{
}

void comm_send_app_cmd_hb_algo_val(void (*pSendDataFunc)(uint8_t uchDataBuffArr[], uint8_t uchLen),
                                   uint8_t hb_aval, uint8_t wear_sval, uint8_t wear_qval, uint8_t sc_bval,
                                   uint8_t lvl_lval, uint8_t func_rval, uint16_t rri_rval)
{
}

void comm_send_app_cmd_hrv_algo_val(void (*pSendDataFunc)(uint8_t uchDataBuffArr[], uint8_t uchLen),
                                    uint16_t rri1_rval, uint16_t rri2_rval, uint16_t rri3_rval, uint16_t rri4_rval,
                                    uint8_t lvl_lval, uint8_t cnt_cval)
{
}

void comm_send_app_cmd_spo2_algo_val(void (*pSendDataFunc)(uint8_t uchDataBuffArr[], uint8_t uchLen),
                                     uint8_t spo2_rval,  uint8_t spo2_reliable_rval, uint8_t hb_aval, uint8_t hb_reliable_rval,
                                     uint16_t rri1_rval, uint16_t rri2_rval, uint16_t rri3_rval, uint16_t rri4_rval,
                                     uint8_t lvl_lval, uint8_t cnt_cval, uint16_t spo2R_rval)
{
}

uint16_t comm_packaging_rawdata_packet(uint8_t *packet_buff, uint16_t comm_max_len,
                                       uint8_t algo_type, uint16_t total_len, uint8_t packet_index, uint8_t algo_result_len,
                                       uint8_t data_array[], uint16_t data_len)
{
    return 0;
}


// add interface for wrapper

uint32_t gh3011_get_i2c_handle(void)
{
    return (uint32_t)gh3011_i2cbus;
}

uint8_t gh3011_get_dev_addr(void)
{
    return (uint8_t)0x14;
}

int gh3011_self_check(void)
{
    struct rt_device_pin_status st;

    // check register first, register 1 should init to 0x14
    //init_err_flag = HBD_SimpleInit(&gh30x_init_config); // init gh30x
    //if (HBD_RET_OK != init_err_flag)
    //{
    //    EXAMPLE_DEBUG_LOG_L1("gh30x init error[%s]\r\n", dbg_ret_val_string[DEBUG_HBD_RET_VAL_BASE + init_err_flag]);
    //    return GH30X_EXAMPLE_ERR_VAL;
    //}

    // check gpio
    if (rt_pin_read(GH3011_RST_PIN) == 0)
        return -1;

    return 0;
}

uint32_t gh3011_get_hr(void)
{
    return (uint32_t)loc_hb_value;
}


/********END OF FILE********* Copyright (c) 2003 - 2020, Goodix Co., Ltd. ********/
