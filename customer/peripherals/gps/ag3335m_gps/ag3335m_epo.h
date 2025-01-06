/*********************
 *      INCLUDES
 *********************/
#ifndef _AT_AG3335M_EPO_H
#define _AT_AG3335M_EPO_H
#include "rtdevice.h"

#define EPO_RECORD_SIZE (72)

#define EPO_SUPPORT_GPS       (0x01)
#define EPO_SUPPORT_GLONASS   (0x02)
#define EPO_SUPPORT_BD2       (0x04)
#define EPO_SUPPORT_GALILEO   (0x08)

#define EPO_MAX_GPS_SV     (32)
#define EPO_MAX_GLONASS_SV (24)

#define EPOs_RECORD_SIZE (72)

#define GNSS_GLONASS_EPO_BASE_ID (64)
#define GNSS_GALILEO_EPO_BASE_ID (100)
#define GNSS_BEIDOU_EPO_BASE_ID (200)
#if 0
    #define EPO_GR_3D_STR     "EPO_GR_3_%d.DAT"
    #define EPO_GPS_3D_STR    "EPO_GPS_3_%d.DAT"
    #define EPO_GR_Q_STR      "QG_R.DAT"
    #define EPO_GPS_Q_STR     "QGPS.DAT"
    #define EPO_BD2_Q_STR     "QBD2.DAT"
    #define EPO_GA_Q_STR      "QGA.DAT"

    #define EPO_GR_3D_STR_F     "1:/GR%d.DAT"
    #define EPO_GPS_3D_STR_F    "1:/GP%d.DAT"
    #define EPO_GR_Q_STR_F      "1:/QGR.DAT"
    #define EPO_GPS_Q_STR_F     "1:/QG.DAT"
    #define EPO_BD2_Q_STR_F     "1:/QBD.DAT"
    #define EPO_GA_Q_STR_F      "1:/QGA.DAT"
#endif

typedef enum
{
    EPO_MODE_GPS,
    EPO_MODE_GLONASS,
    EPO_MODE_GALILEO,
    EPO_MODE_BEIDOU
} epo_mode_t;


typedef struct
{
    int fd;
    epo_file_type_t type;
} epo_data_t;


typedef struct
{
    epo_file_t file;
    rt_bool_t  transfer_flag;    /* false:idle true:in transmitting  */
    struct rt_semaphore transfer_sem;
    rt_thread_t transfer_thread;
} epo_transfer_t;

gps_err_t epo_ading_start(epo_file_t *file);
void epo_transfer_ind(at_client_t client, const char *data, rt_size_t size);

#endif /* _AT_AG3335M_EPO_H */
