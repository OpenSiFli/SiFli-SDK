

#include <stdio.h>
#include <string.h>
#include "rtdevice.h"
#include "drv_gpio.h"
#include "ag3335m.h"
#include "ag3335m_epo.h"
#ifdef BSP_USING_PM
    #include "bf0_pm.h"
#endif


#define DBG_TAG               "ag3335m.control"
#define DBG_LVL               DBG_INFO
#include <rtdbg.h>

static void ag3335m_reset(void)
{
#if (-1 != MCU_RESET_GPS_PIN)
    rt_pin_mode(MCU_RESET_GPS_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(MCU_RESET_GPS_PIN, PIN_LOW);
    rt_thread_mdelay(50);
    rt_pin_mode(MCU_RESET_GPS_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(MCU_RESET_GPS_PIN, PIN_HIGH);
#endif
    return;
}


static void ag3335m_wake_up(void)
{
#if (-1 != MCU_WAKEUP_GPS_PIN)
    rt_pin_mode(MCU_WAKEUP_GPS_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(MCU_WAKEUP_GPS_PIN, PIN_LOW);
    rt_thread_mdelay(15);
    rt_pin_mode(MCU_WAKEUP_GPS_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(MCU_WAKEUP_GPS_PIN, PIN_HIGH);
#endif
    return;
}

static void ag3335m_enable_level_shift(void)
{
#if (-1 != GPS_LEVEL_SHIFT_PIN)
    rt_pin_mode(GPS_LEVEL_SHIFT_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(GPS_LEVEL_SHIFT_PIN, PIN_HIGH);
    rt_thread_mdelay(1);
#endif
    return;
}

static void ag3335m_disable_level_shift(void)
{
#if (-1 != GPS_LEVEL_SHIFT_PIN)
    rt_pin_mode(GPS_LEVEL_SHIFT_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(GPS_LEVEL_SHIFT_PIN, PIN_LOW);
#endif
    return;
}


static int ag3335m_get_locus_num_rsp(at_response_t resp, void *args)
{
    int reuslt = 0;
    const char *resp_line_buf = RT_NULL;
    uint32_t num = 0;
    resp_line_buf = at_resp_get_line_by_kw(resp, "$PAIR909");
    if (RT_NULL == resp_line_buf)
    {
        return GPS_ERROR_PARSING;
    }

    reuslt = sscanf(resp_line_buf + 9, "%d", &num);
    if (1 != reuslt)
    {
        LOG_I("get locus num err:%d", reuslt);
        return GPS_ERROR_PARSING;
    }
    LOG_I("get locus num:%d", num);
    return GPS_EOK;
}

int32_t ag3335m_get_checksum(int8_t *buffer, int32_t buffer_len)
{
    int8_t *ind;
    uint8_t checkSumL = 0, checkSumR;
    int32_t checksum = 0;

    ind = buffer;
    while (ind - buffer < buffer_len)
    {
        checkSumL ^= *ind;
        ind++;
    }

    checkSumR = checkSumL & 0x0F;
    checkSumL = (checkSumL >> 4) & 0x0F;
    checksum = checkSumL * 16;
    checksum = checksum + checkSumR;
    return checksum;
}

void ag3335m_send_ex(char *cmd_buf)
{
    int32_t checksum = 0;
    char temp_buffer[GPS_MAX_CMD_BUFF_LEN] = {0};
    checksum = ag3335m_get_checksum((int8_t *)cmd_buf, strlen(cmd_buf));
    rt_sprintf((char *)temp_buffer, "$%s*%02X\r\n", cmd_buf, (int)checksum);
    rt_size_t length = rt_strlen(temp_buffer);
    at_client_t client = at_client_get(GPS_UART_NAME);
    ag3335m_wake_up();
    at_client_obj_send(client, temp_buffer, length);
    return;
}



static int ag3335m_at_cmd_handle(int cmd, rt_size_t line_num, const char *param, rt_size_t size, void *args)
{
    int result = GPS_EOK;
    at_client_t client = at_client_get(GPS_UART_NAME);
    ag3335m_wake_up();
    result = at_client_cmd_control(client, cmd, line_num, param, size, args);
    if (-RT_ETIMEOUT == result)
    {
        result = GPS_ERROR_DEVICE_EXCEPTION;
    }
    return result;
}

/**
* @brief set nmea mode .
* @param[in] nmea_mode 0:disable 1:nmea v4.1 2:nmea v3.0
* @param[in] proprietary_mode    0:disable 1:enable
*/
static gps_err_t ag3335m_set_nmea_mode(uint8_t nmea_mode, uint8_t proprietary_mode)
{
    int32_t checksum = 0;
    gps_err_t ret = GPS_EOK;
    char temp_buffer[32] = {0};
    rt_sprintf((char *)temp_buffer, "PAIR100,%d,%d", nmea_mode, proprietary_mode);
    checksum = ag3335m_get_checksum((int8_t *)temp_buffer, rt_strlen(temp_buffer));
    rt_sprintf((char *)temp_buffer + rt_strlen(temp_buffer), "*%02X", (int)checksum);
    ret = ag3335m_at_cmd_handle(GPS_SET_NMEA_OUTPUT_MODE, 0, temp_buffer, rt_strlen(temp_buffer), RT_NULL);
    return ret;
}

/* 0:disable 1:enable */
static gps_err_t ag3335m_enable_locus(uint8_t enable)
{
    int32_t checksum = 0;
    gps_err_t ret = GPS_EOK;
    char temp_buffer[32] = {0};
    rt_sprintf((char *)temp_buffer, "PAIR900,%d", enable);
    checksum = ag3335m_get_checksum((int8_t *)temp_buffer, rt_strlen(temp_buffer));
    rt_sprintf((char *)temp_buffer + rt_strlen(temp_buffer), "*%02X", (int)checksum);
    ret = ag3335m_at_cmd_handle(GPS_ENABLE_LOCUS, 0, temp_buffer, rt_strlen(temp_buffer), RT_NULL);
    return ret;
}

/*
Mode: Saving Mode
    Normal, (1 << 0). Record per fix.
    Out of time, (1 << 1). Record every N s. N is customer configuration (PAIR_LOCUS_SET_THRESHOLD).
    Out of speed, (1 << 2). Record after speed more than N m/s. N is customer configuration (PAIR_LOCUS_SET_THRESHOLD).
    Out of distance, (1 << 3). Record after distance more than N m. N is customer configuration (PAIR_LOCUS_SET_THRESHOLD).
    Before entry sleep, (1 << 4). Record before entry sleep.
    User control, (1 << 5). Record after user send PAIR_LOCUS_LOG_NOW.
Check_3D_Fix: Need check 3D fix or not.
    0: not check.
    1: need check. If set this type as 1, system will not save the location without 3D fixed.
*/
static gps_err_t ag3335m_locus_set_mode(uint8_t mode_mask, uint8_t check_3d_fix)
{
    int32_t checksum = 0;
    gps_err_t ret = GPS_EOK;
    char temp_buffer[32] = {0};
    rt_sprintf((char *)temp_buffer, "PAIR902,%d,%d", mode_mask, check_3d_fix);
    checksum = ag3335m_get_checksum((int8_t *)temp_buffer, rt_strlen(temp_buffer));
    rt_sprintf((char *)temp_buffer + rt_strlen(temp_buffer), "*%02X", (int)checksum);
    ret = ag3335m_at_cmd_handle(GPS_LOCUS_SET_MODE, 0, temp_buffer, rt_strlen(temp_buffer), RT_NULL);
    return ret;
}



gps_err_t ag3335m_set_refutc(struct tm *now_time)
{
    int32_t checksum = 0;
    gps_err_t ret = GPS_EOK;
    char temp_buffer[32] = {0};
    rt_sprintf((char *)temp_buffer,  "PAIR590,%d,%d,%d,%d,%d,%d", now_time->tm_year + 1900, now_time->tm_mon + 1, \
               now_time->tm_mday, now_time->tm_hour, now_time->tm_min, now_time->tm_sec);
    checksum = ag3335m_get_checksum((int8_t *)temp_buffer, rt_strlen(temp_buffer));
    rt_sprintf((char *)temp_buffer + rt_strlen(temp_buffer), "*%02X", (int)checksum);
    ret = ag3335m_at_cmd_handle(GPS_SET_REF_UTC, 0, temp_buffer, rt_strlen(temp_buffer), RT_NULL);
    return ret;
}


static gps_err_t ag3335m_sleep()
{
    gps_err_t ret = GPS_EOK;
    ret = ag3335m_set_nmea_mode(0, 0);
    if (ret != GPS_EOK)
    {
        return ret;
    }
    ret = ag3335m_at_cmd_handle(GPS_CLEAR_LOCUS_DATA, 0, RT_NULL, 0, RT_NULL);
    if (GPS_EOK != ret)
    {
        return ret;
    }

    ret = ag3335m_locus_set_mode(1, 1);
    if (GPS_EOK != ret)
    {
        return ret;
    }
    ret = ag3335m_enable_locus(1);
    return ret;
}

static gps_err_t ag3335m_resume()
{
    gps_err_t ret = GPS_EOK;

    ret = ag3335m_at_cmd_handle(GPS_GET_LOCUS_NUM, 0, RT_NULL, 0, RT_NULL);
    if (ret != GPS_EOK)
    {
        return ret;
    }

    ret = ag3335m_enable_locus(0);
    if (GPS_EOK != ret)
    {
        return ret;
    }

    ret = ag3335m_at_cmd_handle(GPS_GET_LOCUS_DATA, 0, RT_NULL, 0, RT_NULL);
    if (ret != GPS_EOK)
    {
        return ret;
    }
    ret = ag3335m_set_nmea_mode(1, 0);
    return ret;
}


static gps_err_t ag3335m_open()
{
    gps_err_t ret = GPS_EOK;
    ag3335m_enable_level_shift();
    ret = ag3335m_at_cmd_handle(GPS_OPEN_DEVICE, 0, RT_NULL, 0, RT_NULL);
    if (ret != GPS_EOK)
    {
        return ret;
    }
    ret = ag3335m_set_nmea_mode(1, 0);
    return ret;
}

gps_err_t ag3335m_close()
{
    gps_err_t ret = GPS_EOK;
    ret = ag3335m_set_nmea_mode(0, 0);
    if (ret != GPS_EOK)
    {
        return ret;
    }

    ret = ag3335m_at_cmd_handle(GPS_CLOSE_DEVICE, 0, RT_NULL, 0, RT_NULL);
    if (ret != GPS_EOK)
    {
        return ret;
    }
    ag3335m_disable_level_shift();
    return ret;
}




gps_err_t ag3335m_control(struct rt_gps_device *gps_handle, int cmd, void *args)
{
    gps_err_t ret = GPS_EOK;
    switch (cmd)
    {

    case GPS_STOP_LOCUS:
    {
        ret = ag3335m_resume();
    }
    break;

    case GPS_START_LOCUS:
    {
        ret = ag3335m_sleep();
    }
    break;

    case RT_DEVICE_CTRL_RESUME:
    {
        /*TODO*/
    }
    break;

    case RT_DEVICE_CTRL_SUSPEND:
    {
        /*TODO*/
    }
    break;


    case GPS_CLOSE_DEVICE:
    {
        ret = ag3335m_close();
    }
    break;

    case GPS_OPEN_DEVICE:
    {
        ret = ag3335m_open();
    }
    break;

    case GPS_EPO_TRANSFER_START:
    {
        epo_file_t *file = (epo_file_t *)args;
        ret = epo_ading_start(file);
    }
    break;

    case GPS_SET_REF_UTC:
    {
        struct tm *cur_time = (struct tm *)args;
        ret = ag3335m_set_refutc(cur_time);
    }
    break;

    case GPS_RESET_DEVICE:
    {
        ag3335m_reset();
    }
    break;

    default:
        ret = ag3335m_at_cmd_handle(cmd, 0, RT_NULL, 0, args);
        break;
    }
    return ret;
}


static const at_client_cmd_t ag3335m_at_cmd_table[] =
{
    {GPS_OPEN_DEVICE, "$PAIR002*38", "$PAIR001,002,0", 400, RT_NULL},
    {GPS_CLOSE_DEVICE, "$PAIR003*39", "$PAIR001,003,0", 200, RT_NULL},
    {GPS_HOT_START, "$PAIR004*3E", "$PAIR001,004,0", 500, RT_NULL},
    {GPS_WARM_START, "$PAIR005*3F", "$PAIR001,005,0", 500, RT_NULL},
    {GPS_COLD_START, "$PAIR006*3C", "$PAIR001,006,0", 500, RT_NULL},
    {GPS_ENABLE_LOCUS, "$", "$PAIR001,900,0", 500, RT_NULL},
    {GPS_CLEAR_LOCUS_DATA, "$PAIR906,1*28", "$PAIR001,906,0", 500, RT_NULL},
    {GPS_GET_LOCUS_NUM, "$PAIR909*3A", "$PAIR909", 500, ag3335m_get_locus_num_rsp},
    {GPS_GET_LOCUS_DATA, "$PAIR908,0*27", "$PAIR001,908,0", 500, RT_NULL},
    {GPS_LOCUS_SET_MODE, "$", "$PAIR001,902,0", 500, RT_NULL},
    {GPS_SET_NMEA_OUTPUT_MODE, "$", "$PAIR001,100,0", 200, RT_NULL},
    {GPS_SET_REF_UTC, "$,", "$PAIR001,590,0", 500, RT_NULL}
};

void ag3335m_set_cmd_table(at_client_t client)
{
    at_obj_set_cmd_table(client, ag3335m_at_cmd_table, sizeof(ag3335m_at_cmd_table) / sizeof(ag3335m_at_cmd_table[0]));
    return;
}


#ifdef RT_US_GPS_DEBUG
int32_t gps_test(int32_t argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Wrong argument\n");
    }
    if (!strcmp(argv[1], "file"))
    {
        if (argc < 4)
            rt_kprintf("gps file wrong argument\n");
        epo_file_t file = {0};
        rt_strncpy(file.name, argv[2], rt_strlen(argv[2]));
        file.type = atoi(argv[3]);
        epo_ading_start(&file);
    }
    else
    {
        rt_kprintf("cmd:%s\n", argv[1]);
        ag3335m_send_ex(argv[1]);
    }
    return 0;
}

MSH_CMD_EXPORT(gps_test, ag3335m test     cmd);
#endif



