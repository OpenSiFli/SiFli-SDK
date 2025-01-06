#ifndef __GPS_DEVICE_H__
#define __GPS_DEVICE_H__
#include <rtthread.h>
#include <at.h>
#include "info.h"

#define GPS_MAX_EVENT_NOTIFY_CB_NUM (8)
#define GPS_DEVICE_FLAG_OPEN (0x010)
typedef enum
{
    /*GPS device*/
    GPS_OPEN_DEVICE = RT_DEVICE_CTRL_GET_INT + 1,             /**< open gnss device */
    GPS_CLOSE_DEVICE,                /**< close gnss device */
    GPS_REGISTER_NOTIFY,             /**< register gnss event notify func */
    GPS_UNREGISTER_NOTIFY,           /**< unresgiter gnss event notify func */
    GPS_QUERY_STATE,                 /**< query current gnss state */
    GPS_HOT_START,                   /**< Hot Start. Use the available data in the NVRAM. */
    GPS_WARM_START,                  /**< warm Start. not using Ephemeris data at the start. */
    GPS_COLD_START,                  /**< cold Start. not using the Position, Almanac and Ephemeris data at the start */
    GPS_EPO_TRANSFER_START,          /**< send epo data to gnss chip */
    GPS_SET_REF_UTC,                 /**< send utc time to gnss chip */
    GPS_START_LOCUS,                 /**< enabling gnss chip track caching */
    GPS_STOP_LOCUS,                  /**< Stop track cache and report historical track data and real-time location data */
    GPS_RESET_DEVICE,                /**< reset gnss chip */
    GPS_GENERAL_CMD_MAX
} gps_general_cmd_t;

typedef enum
{
    GPS_STATE_POWER_OFF = 0,
    GPS_STATE_POWER_ON,
    GPS_STATE_MAX,
} gps_state_t;


typedef enum
{
    GPS_LOCATION_IND = 0,                  /**< location ind */
    GPS_EPO_TRANSFER_RESULT_IND,       /**< epo transfer result ind */
    GPS_TIME_REQUEST_IND,              /**< please send reference time when this event is received */
    GPS_EVENT_MAX
} gps_notify_event_t;

typedef enum
{
    GPS_EOK = 0,
    /* general error code */
    GPS_ERROR_INPARAM            = 0x11000001,  /**< input param error */
    GPS_ERROR_UNSUPPORTED        = 0x11000002,  /**< unsupported function */
    GPS_ERROR_TIMEOUT            = 0x11000003,  /**< error timout */
    GPS_ERROR_DISCONNECTED       = 0x11000004,  /**< the gps device is disconnected */
    GPS_ERROR_STATE              = 0x11000005,  /**< current state  unsupported this function */
    GPS_ERROR_PARSING            = 0x11000006,  /**< parsing at response error */
    GPS_ERROR_POWER_OFF          = 0x11000007,  /**< current gps device has been power off */
    GPS_ERROR_NOTIFY_CB_FULL     = 0x11000008,  /**< register notify cb is more than GPS_MAX_EVENT_NOTIFY_CB_NUM */
    GPS_ERROR_DEVICE_EXCEPTION   = 0x11000009,  /**< current gps device has happend exception */
    GPS_ERROR_RESP_FAIL          = 0x11000010,  /**< at cmd response fail */
    GPS_ERROR_TRANSFER_BUSY      = 0x11000011,  /**< file transfer busy */
    GPS_ERROR_EPO_INVALID        = 0x11000012,  /**< epo data is invalid */
    GPS_ERROR_EPO_NOT_EXIST      = 0x11000013,  /**< epo file not exist */
    GPS_ERROR_EPO_FILE_TYPE      = 0x11000014,  /**< epo file type is error */
    GPS_ERROR_EPO_READ           = 0x11000015   /**< epo file read error */
} gps_err_t;

typedef struct
{
    double  lat;        /**< Latitude in NDEG - +/-[degree][min].[sec/60] */
    double  lon;        /**< Longitude in NDEG - +/-[degree][min].[sec/60] */
    double  elv;        /**< altitude above/below mean sea level (geoid) in meters */
    double  speed;      /**< Speed over the ground in kilometers/hour */
} gps_location_t;


typedef enum
{
    EPO_TYPE_3D_START,
    EPO_TYPE_GR_3D = EPO_TYPE_3D_START,
    EPO_TYPE_GPS_3D,
    EPO_TYPE_3D_END = EPO_TYPE_GPS_3D,

    EPO_TYPE_QEPO_START,
    EPO_TYPE_GR_Q = EPO_TYPE_QEPO_START,
    EPO_TYPE_GPS_Q,
    EPO_TYPE_BD2_Q,
    EPO_TYPE_GA_Q,
    EPO_TYPE_QEPO_END = EPO_TYPE_GA_Q,
    EPO_TYPE_MAX
} epo_file_type_t;


typedef struct
{
    char name[128];
    epo_file_type_t type;
} epo_file_t;


typedef struct
{
    gps_notify_event_t event;
    void *args;
} gps_notify_t;


typedef void (*gps_notify_cb)(gps_notify_t *param);

typedef struct
{
    uint32_t size;
    gps_notify_cb cb[GPS_MAX_EVENT_NOTIFY_CB_NUM];
} gps_notify_cb_array_t;

typedef struct rt_gps_device
{
    struct rt_device   parent;
    gps_notify_cb_array_t cb_arry;
    rt_mutex_t handle_lock;
    gps_state_t status;
    const struct rt_gps_ops *ops;
} rt_gps_t;

typedef gps_err_t (*gps_control_cb)(struct rt_gps_device *dev_handle, int cmd, void *arg);

struct rt_gps_ops
{
    gps_control_cb control;
};

void rt_gps_event_notify(gps_notify_t      *param);
int rt_hw_gps_init(const struct rt_gps_ops *ops);
#endif
