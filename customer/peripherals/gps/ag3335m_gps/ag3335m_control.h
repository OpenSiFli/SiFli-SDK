/*********************
 *      INCLUDES
 *********************/
#ifndef _AT_AG3335M_CONTROL_H
#define _AT_AG3335M_CONTROL_H
#include "rtdevice.h"

#define GPS_MAX_CMD_BUFF_LEN  (352)

typedef enum
{
    GPS_ENABLE_LOCUS = GPS_GENERAL_CMD_MAX,
    GPS_CLEAR_LOCUS_DATA,
    GPS_GET_LOCUS_NUM,
    GPS_GET_LOCUS_DATA,
    GPS_LOCUS_SET_MODE,
    GPS_SET_NMEA_OUTPUT_MODE,
    AG3335M_CUSTOM_CMD_MAX
} ag3335m_custom_cmd_t;

gps_err_t ag3335m_control(struct rt_gps_device *gps_handle, int cmd, void *args);
void ag3335m_set_cmd_table(at_client_t client);

void ag3335m_send_ex(char *cmd_buf);

#endif /* _AT_AG3335M_CONTROL_H */
