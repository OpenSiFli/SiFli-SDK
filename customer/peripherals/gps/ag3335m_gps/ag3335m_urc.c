

#include <stdio.h>
#include <string.h>
#include "rtdevice.h"
#include "nmea.h"
#include "ag3335m_epo.h"
#include "ag3335m_control.h"
#define DBG_TAG               "ag3335m.urc"
#define DBG_LVL               DBG_INFO
#include <rtdbg.h>


static void ag3335m_urc_rmc(at_client_t client, const char *data, rt_size_t size)
{
    nmea_parser_t parser = {0};
    nmea_info_t *info = (nmea_info_t *)rt_malloc(sizeof(nmea_info_t));
    if (RT_NULL == info)
    {
        LOG_I("ag3335m_urc_rmc:malloc fail");
        return;
    }
    nmea_zero_info(info);
    nmea_parser_init(&parser);
    nmea_parse(&parser, data, (int)size, info);
    if (NMEA_FIX_BAD == info->fix)
    {
        LOG_I("rmc fix bad:%s", data);
        nmea_parser_destroy(&parser);
        rt_free(info);
        return;
    }
    gps_notify_t args;
    gps_location_t location = {0};
    location.lat = nmea_ndeg2degree(info->lat);
    location.lon = nmea_ndeg2degree(info->lon);;
    location.elv = info->elv;
    location.speed = info->speed;
    args.event = GPS_LOCATION_IND;
    args.args = &location;
    rt_gps_event_notify(&args);
    nmea_parser_destroy(&parser);
    rt_free(info);
    LOG_I("ag3335m_urc_rmc:lat:%f lon:%f", location.lat, location.lon);
    return;
}

static void ag3335m_urc_locus_begin(at_client_t client, const char *data, rt_size_t size)
{
    /*@todo*/
    LOG_I("locus begin:%s", data);
    return;
}

static void ag3335m_urc_locus_num(at_client_t client, const char *data, rt_size_t size)
{
    int reuslt = 0;
    uint32_t record_num = 0;
    uint32_t record_size = 0;
    const char *resp_line_buf = RT_NULL;
    resp_line_buf = rt_strstr(data, "$PAIR908,1");
    if (RT_NULL == resp_line_buf)
    {
        return;
    }
    reuslt = sscanf(resp_line_buf + 11, "%d,%d", &record_num, &record_size);
    if (2 != reuslt)
    {
        LOG_I("locus num urc err:%d", reuslt);
        return;
    }
    LOG_I("locus num:%d size:%d", record_num, record_size);
    return;
}


static void ag3335m_urc_locus_end(at_client_t client, const char *data, rt_size_t size)
{
    /*@todo*/
    LOG_I("locus end:%s", data);
    return;
}

static void ag3335m_urc_time_request(at_client_t client, const char *data, rt_size_t size)
{
    LOG_I("ref time requet:%s", data);
    gps_notify_t args;
    args.event = GPS_TIME_REQUEST_IND;
    args.args = RT_NULL;
    rt_gps_event_notify(&args);
    return;
}

static void ag3335m_urc_location_request(at_client_t client, const char *data, rt_size_t size)
{
    /*@todo*/
    LOG_I("ref location requet:%s", data);
    return;
}



static const struct at_urc gps_urc_table[] =
{
    /*GSA*/
    {"$GAGSA",        "\r\n",           RT_NULL},
    {"$GBGSA",        "\r\n",           RT_NULL},
    {"$GLGSA",       "\r\n",           RT_NULL},
    {"$GNGSA",        "\r\n",           RT_NULL},
    {"$GPGSA",       "\r\n",           RT_NULL},
    /*GGA*/
    {"$GAGGA",        "\r\n",           RT_NULL},
    {"$GBGGA",       "\r\n",           RT_NULL},
    {"$GLGGA",       "\r\n",           RT_NULL},
    {"$GNGGA",        "\r\n",           RT_NULL},
    {"$GPGGA",       "\r\n",           RT_NULL},
    /*GSV*/
    {"$GAGSV",        "\r\n",           RT_NULL},
    {"$GBGSV",       "\r\n",           RT_NULL},
    {"$GLGSV",        "\r\n",           RT_NULL},
    {"$GNGSV",       "\r\n",           RT_NULL},
    {"$GPGSV",       "\r\n",           RT_NULL},
#if 1
    /*RMC*/
    {"$GARMC",       "\r\n",           ag3335m_urc_rmc},
    {"$GBRMC",       "\r\n",           ag3335m_urc_rmc},
    {"$GLRMC",       "\r\n",           ag3335m_urc_rmc},
    {"$GNRMC",       "\r\n",           ag3335m_urc_rmc},
    {"$GPRMC",       "\r\n",           ag3335m_urc_rmc},
#endif
    /*VTG*/
    {"$GAVTG",       "\r\n",           RT_NULL},
    {"$GBVTG",       "\r\n",           RT_NULL},
    {"$GLVTG",       "\r\n",           RT_NULL},
    {"$GNVTG",       "\r\n",           RT_NULL},
    {"$GPVTG",       "\r\n",           RT_NULL},

    /*ZDA*/
    {"$GAZDA",       "\r\n",           RT_NULL},
    {"$GBZDA",       "\r\n",           RT_NULL},
    {"$GLZDA",       "\r\n",           RT_NULL},
    {"$GNZDA",       "\r\n",           RT_NULL},
    {"$GPZDA",       "\r\n",           RT_NULL},

    /*GRS*/
    {"$GAGRS",       "\r\n",           RT_NULL},
    {"$GBGRS",       "\r\n",           RT_NULL},
    {"$GLGRS",       "\r\n",           RT_NULL},
    {"$GNGRS",       "\r\n",           RT_NULL},
    {"$GPGRS",       "\r\n",           RT_NULL},

    /*GST*/
    {"$GAGST",       "\r\n",           RT_NULL},
    {"$GBGST",       "\r\n",           RT_NULL},
    {"$GLGST",       "\r\n",           RT_NULL},
    {"$GNGST",       "\r\n",           RT_NULL},
    {"$GPGST",       "\r\n",           RT_NULL},

    /*GLL*/
    {"$GAGLL",       "\r\n",           RT_NULL},
    {"$GBGLL",       "\r\n",           RT_NULL},
    {"$GLGLL",       "\r\n",           RT_NULL},
    {"$GNGLL",       "\r\n",           RT_NULL},
    {"$GPGLL",       "\r\n",           RT_NULL},

    /* epo set */
    {"$PAIR001,471", "\r\n",           epo_transfer_ind},

    /*locus*/
    {"$PAIR908,0", "\r\n",           ag3335m_urc_locus_begin},
    {"$PAIR908,1", "\r\n",           ag3335m_urc_locus_num},
    {"$PAIR908,3", "\r\n",           ag3335m_urc_locus_end},

    /*locus data*/
    {"$LOGGA", "\r\n",           RT_NULL},
    {"$LORMC", "\r\n",           ag3335m_urc_rmc},

    /*ref time request*/
    {"$PAIR010,1", "\r\n",           ag3335m_urc_time_request},

    /*ref location request*/
    {"$PAIR010,2", "\r\n",           ag3335m_urc_location_request}

};

void ag3335m_set_urc(at_client_t client)
{
    at_obj_set_urc_table(client, gps_urc_table, sizeof(gps_urc_table) / sizeof(gps_urc_table[0]));
    return;
}




