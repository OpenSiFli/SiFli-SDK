

#include <stdio.h>
#include <string.h>
#include <dfs_fs.h>
#include <dfs_file.h>
#include <dfs_posix.h>
#include "ag3335m_epo.h"
#include "ag3335m_control.h"
#define DBG_TAG               "ag3335m.epo"
#define DBG_LVL               DBG_INFO
#include <rtdbg.h>

epo_transfer_t g_epo_transfer = {0};

static void epo_transfer_result_notify(gps_err_t result)
{
    gps_notify_t args;
    gps_err_t ret = result;
    args.event = GPS_EPO_TRANSFER_RESULT_IND;
    args.args = &ret;
    rt_gps_event_notify(&args);
    return;
}

int32_t epo_utc_to_gnss_hour(int32_t year, int32_t month, int32_t day, int32_t hour)
{
    int32_t years_elapsed; // Years since 1980
    int32_t days_elapsed; // Days elapsed since Jan 6, 1980
    int32_t leap_days; // Leap days since Jan 6, 1980
    int32_t i;
    // Number of days into the year at the start of each month (ignoring leap years)
    const unsigned short doy[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    years_elapsed = year - 1980;
    i = 0;
    leap_days = 0;
    while (i <= years_elapsed)
    {
        if ((i % 100) == 20)
        {
            if ((i % 400) == 20)
            {
                leap_days++;
            }
        }
        else if ((i % 4) == 0)
        {
            leap_days++;
        }
        i++;
    }
    if ((years_elapsed % 100) == 20)
    {
        if (((years_elapsed % 400) == 20) && (month <= 2))
        {
            leap_days--;
        }
    }
    else if (((years_elapsed % 4) == 0) && (month <= 2))
    {
        leap_days--;
    }
    days_elapsed = years_elapsed * 365 + (int)doy[month - 1] + day + leap_days - 6;
    // Convert time to GNSS weeks and seconds
    return (days_elapsed * 24 + hour);
}

int32_t epo_time_to_gnss_min(int32_t week_num, int32_t tow)
{
    return (week_num * 24 * 7 * 60) + (tow / 60);
}

int32_t epo_get_3d_valid_hours(char *file_name, epo_file_type_t type_3d, uint32_t trunk_num_3d, int32_t *max_epo_hours)
{
    int32_t epo_gnss_hour = 0;
    int32_t current_gnss_hour = 0;
    epo_data_t data;
    int32_t i;
    int32_t max_gnss_hours = 0;
    time_t rtc_time;
    struct tm now;
    rtc_time = time(RT_NULL);
    gmtime_r(&rtc_time, &now);

    current_gnss_hour = epo_utc_to_gnss_hour(now.tm_year + 1900, now.tm_mon + 1, now.tm_mday, now.tm_hour);

    for (i = 0; i < trunk_num_3d; i++)
    {
#if 0
        if (type_3d == EPO_TYPE_GR_3D)
        {
            sprintf(file_name, EPO_GR_3D_STR_F, (int)i + 1);
        }
        else if (type_3d == EPO_TYPE_GPS_3D)
        {
            sprintf(file_name, EPO_GPS_3D_STR_F, (int)i + 1);
        }
#endif
        data.fd = open(file_name, O_RDONLY | O_BINARY);
        if (data.fd < 0)
        {
            continue;
        }

        read(data.fd, &epo_gnss_hour, 4);
        epo_gnss_hour = epo_gnss_hour & 0x00FFFFFF;
        if (epo_gnss_hour >= max_gnss_hours)
        {
            max_gnss_hours = epo_gnss_hour;
        }

        close(data.fd);
    }
    if ((max_gnss_hours + (3 * 24) - current_gnss_hour) > 0)
    {
        *max_epo_hours = max_gnss_hours + (3 * 24);
        return (max_gnss_hours + (3 * 24) - current_gnss_hour);
    }
    *max_epo_hours = 0;
    return 0;
}



uint32_t epo_get_sv_prn(int32_t type, uint8_t *data)
{
    int32_t sv_id, sv_prn = 0;

    sv_id = data[3];

    switch (type)
    {
    case EPO_MODE_GPS:
        sv_prn = sv_id;
        break;
    case EPO_MODE_GLONASS:
        sv_prn = sv_id - GNSS_GLONASS_EPO_BASE_ID;
        break;
    case EPO_MODE_GALILEO:
        if (sv_id == 255)
        {
            sv_prn = 255;
        }
        else
        {
            sv_prn = sv_id - GNSS_GALILEO_EPO_BASE_ID;
        }
        break;
    case EPO_MODE_BEIDOU:
        if (sv_id == 255)
        {
            sv_prn = 255;
        }
        else
        {
            sv_prn = sv_id - GNSS_BEIDOU_EPO_BASE_ID;
        }

        break;
    default:
        sv_prn = 0;

    }
    return sv_prn;
}

void epo_transfer_ind(at_client_t client, const char *data, rt_size_t size)
{
    LOG_I("%s:buf:%s", __func__, data);
    rt_sem_release(&g_epo_transfer.transfer_sem);
    return;
}

gps_err_t epo_send_data(epo_data_t *data_p, int32_t data_num, int32_t type)
{

    char temp_buffer[GPS_MAX_CMD_BUFF_LEN] = {0};
    uint8_t data_buffer[EPO_RECORD_SIZE] = {0};
    int32_t i = 0;
    int32_t sv_prn = 0;
    int read_size = 0;
    LOG_I("%s: num:%d", __func__, data_num);
    for (i = 0; i < data_num; i++)
    {
        unsigned int *epobuf = (unsigned int *)data_buffer;
        read_size = read(data_p->fd, data_buffer, EPO_RECORD_SIZE);
        if (read_size != EPO_RECORD_SIZE)
        {
            LOG_E("%s index:%d read error:%d", __func__, i, read_size);
            return GPS_ERROR_EPO_READ;
        }
        sv_prn = epo_get_sv_prn(type, data_buffer);
        sprintf((char *) temp_buffer,
                "PAIR471,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X,%X",
                (unsigned int)type,
                (unsigned int)sv_prn,
                epobuf[0], epobuf[1], epobuf[2], epobuf[3], epobuf[4], epobuf[5],
                epobuf[6], epobuf[7], epobuf[8], epobuf[9], epobuf[10], epobuf[11],
                epobuf[12], epobuf[13], epobuf[14], epobuf[15], epobuf[16], epobuf[17]);

        ag3335m_send_ex(temp_buffer);
        if (rt_sem_take(&g_epo_transfer.transfer_sem, rt_tick_from_millisecond(500)) != RT_EOK)
        {
            LOG_E("%s time out:%d", __func__, i);
            return GPS_ERROR_TIMEOUT;
        }
        memset(temp_buffer, 0, GPS_MAX_CMD_BUFF_LEN);
        LOG_I("%s send_index:%d pass\n", __func__, i);
        rt_thread_delay(10);
    }
    LOG_I("%s: PASS\n", __func__);
    return GPS_EOK;
}


gps_err_t epo_ading_qepo(char *file_name, epo_file_type_t type)
{
    epo_data_t data;
    gps_err_t ret = GPS_EOK;
    int32_t current_gnss_hour = 0;
    int32_t epo_gnss_hour = 0;
    int32_t sv_type;
    struct tm now;
    time_t rtc_time;
    rtc_time = time(RT_NULL);
    gmtime_r(&rtc_time, &now);
    current_gnss_hour = epo_utc_to_gnss_hour(now.tm_year + 1900, now.tm_mon + 1, now.tm_mday, now.tm_hour);
    LOG_I("epo_ading_qepo:%d,Y:%d,M:%d D:%d H:%d\n", type, now.tm_year + 1900, \
          now.tm_mon + 1, now.tm_mday, now.tm_hour);
    data.fd = open(file_name, O_RDONLY | O_BINARY);
    if (data.fd >= 0)
    {
        struct dfs_fd *d = fd_get(data.fd);
        int32_t sv_num = (int)(d->size / EPO_RECORD_SIZE);
        fd_put(d);
        if ((type == EPO_TYPE_BD2_Q) || (type == EPO_TYPE_GA_Q))
        {
            lseek(data.fd, 72, SEEK_SET);
        }
        read(data.fd, &epo_gnss_hour, 4);
        epo_gnss_hour = epo_gnss_hour & 0x00FFFFFF;
        if (((current_gnss_hour - epo_gnss_hour) < 0) || ((current_gnss_hour - epo_gnss_hour) > 6))
        {
            LOG_I("epo_ading_qepo_int:%d QEPO out of time.\n", type);
            ret = GPS_ERROR_EPO_INVALID;
            goto _exit;
        }

        lseek(data.fd, 0, SEEK_SET);
        if (type == EPO_TYPE_BD2_Q)
        {
            sv_type = EPO_MODE_BEIDOU;
        }
        else if (type == EPO_TYPE_GA_Q)
        {
            sv_type = EPO_MODE_GALILEO;
        }
        else if (type == EPO_TYPE_GPS_Q)
        {
            sv_type = EPO_MODE_GPS;
        }
        else if (type == EPO_TYPE_GR_Q)
        {
            ret = epo_send_data(&data, sv_num, EPO_MODE_GPS);
            if (GPS_EOK != ret)
            {
                goto _exit;
            }
            sv_type = EPO_MODE_GLONASS;   /* @note sv_num GPS &GR*/
            lseek(data.fd, EPO_MAX_GPS_SV * EPO_RECORD_SIZE, SEEK_SET);
        }
        else
        {
            ret = GPS_ERROR_EPO_FILE_TYPE;
            goto _exit;
        }
        ret = epo_send_data(&data, sv_num, sv_type);
        goto _exit;
    }
    LOG_E("[EPO]epo_demo_ading_qepo_int:%d without QEPO file.\n", type);
    ret = GPS_ERROR_EPO_NOT_EXIST;
_exit:
    if (data.fd >= 0)
    {
        close(data.fd);
    }
    return ret;
}

gps_err_t epo_ading_3d(char *file_name, epo_file_type_t type)
{
    int32_t epo_gnss_hour = 0;
    int32_t current_gnss_hour = 0;
    int32_t segment = -1;
    epo_data_t data;
    int32_t sv_num = 0;
    struct tm now;
    time_t rtc_time;
    gps_err_t ret = GPS_EOK;

    data.fd = open(file_name, O_RDONLY | O_BINARY);
    if (data.fd < 0)
    {
        // TODO: Need try other file?
        LOG_E("[EPO]epo_demo_ading_3d:%d, without EPO file 1.\n", type);
        ret = GPS_ERROR_EPO_NOT_EXIST;
        goto _exit;
    }

    read(data.fd, &epo_gnss_hour, 4);
    epo_gnss_hour = epo_gnss_hour & 0x00FFFFFF;

    rtc_time = time(RT_NULL);
    LOG_I("% s", ctime(&rtc_time));
    gmtime_r(&rtc_time, &now);
    current_gnss_hour = epo_utc_to_gnss_hour(now.tm_year + 1900, now.tm_mon + 1, now.tm_mday, now.tm_hour);

    LOG_I("epo_ading_3d:%d,Y:%d,M:%d D:%d H:%d\n", type, now.tm_year + 1900, \
          now.tm_mon + 1, now.tm_mday, now.tm_hour);

    segment = (current_gnss_hour - epo_gnss_hour) / 6;
    LOG_I("[EPO]epo_demo_ading_3d c_h:%d, g_h:%d, segment:%d", current_gnss_hour, epo_gnss_hour, segment);

    if ((segment >= 0) && (segment < 12))
    {
        if (EPO_TYPE_GR_3D == type)
        {
            lseek(data.fd, segment * (EPO_MAX_GPS_SV + EPO_MAX_GLONASS_SV) * EPO_RECORD_SIZE, SEEK_SET);
            sv_num = EPO_MAX_GPS_SV;
            ret = epo_send_data(&data, sv_num, EPO_MODE_GPS);
            if (GPS_EOK != ret)
            {
                goto _exit;
            }
            lseek(data.fd, (segment * (EPO_MAX_GPS_SV + EPO_MAX_GLONASS_SV) * EPO_RECORD_SIZE) + \
                  (EPO_MAX_GPS_SV * EPO_RECORD_SIZE), SEEK_SET);
            sv_num = EPO_MAX_GLONASS_SV;
            ret = epo_send_data(&data, sv_num, EPO_MODE_GLONASS);
        }
        else
        {
            lseek(data.fd, segment * EPO_MAX_GPS_SV * EPO_RECORD_SIZE, SEEK_SET);
            sv_num = EPO_MAX_GPS_SV;
            ret = epo_send_data(&data, sv_num, EPO_MODE_GPS);
        }
        goto _exit;
    }
    ret = GPS_ERROR_EPO_INVALID;

_exit:
    if (data.fd >= 0)
    {
        close(data.fd);
    }
    return ret;;
}

gps_err_t epo_ading_start(epo_file_t *file)
{
    if (g_epo_transfer.transfer_flag)
    {
        return GPS_ERROR_TRANSFER_BUSY;
    }
    rt_memcpy(&g_epo_transfer.file, file, sizeof(epo_file_t));
    g_epo_transfer.transfer_flag = RT_TRUE;
    rt_sem_release(&g_epo_transfer.transfer_sem);
    return GPS_EOK;
}

void epo_transfer_task(void *params)
{
    gps_err_t ret = GPS_EOK;
    while (1)
    {
        rt_sem_take(&g_epo_transfer.transfer_sem, RT_WAITING_FOREVER);
        rt_sem_control(&g_epo_transfer.transfer_sem, RT_IPC_CMD_RESET, RT_NULL);
        if (EPO_TYPE_GR_3D == g_epo_transfer.file.type || \
                EPO_TYPE_GPS_3D == g_epo_transfer.file.type)
        {
            ret = epo_ading_3d(g_epo_transfer.file.name, g_epo_transfer.file.type);
        }
        else
        {
            ret = epo_ading_qepo(g_epo_transfer.file.name, g_epo_transfer.file.type);
        }
        epo_transfer_result_notify(ret);
        g_epo_transfer.transfer_flag = RT_FALSE;
    }
}

int epo_transfer_init(void)
{
    /*memset battery information*/
    memset(&g_epo_transfer, 0x00, sizeof(epo_transfer_t));
    rt_sem_init(&g_epo_transfer.transfer_sem, "epo_transfer_sem", 0, RT_IPC_FLAG_FIFO);
    rt_sem_control(&g_epo_transfer.transfer_sem, RT_IPC_CMD_RESET, RT_NULL);
    g_epo_transfer.transfer_thread = rt_thread_create("epo_transfer", epo_transfer_task, NULL, 2048, RT_THREAD_PRIORITY_MAX / 3 - 1, 10);
    RT_ASSERT(g_epo_transfer.transfer_thread);
    rt_thread_startup(g_epo_transfer.transfer_thread);
    return 0;
}

INIT_APP_EXPORT(epo_transfer_init);



