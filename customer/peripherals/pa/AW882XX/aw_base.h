#ifndef __AWINIC_BASE_H__
#define __AWINIC_BASE_H__


#include <stdbool.h>

#include "bf0_hal.h"
#include "rtthread.h"
/*awinic:add and delete main.h according to the actual needs*/

/********************************************
 * delay: Modify delay function according to different platform
 *******************************************/
#define AW_MS_DELAY(time) HAL_Delay(time)

/********************************************
 * function control of each module
 *******************************************/

//#define AW_DEBUG
//#define AW_FADE
//#define AW_VOLUME
//#define AW_IRQ
//#define AW_MONITOR

/********************************************
 * dev_index information
 *******************************************/
typedef enum
{
    AW_DEV_0 = 0,
    AW_DEV_1,
    AW_DEV_2,
    AW_DEV_3,
    AW_DEV_4,
    AW_DEV_5,
    AW_DEV_6,
    AW_DEV_7,
    AW_DEV_8,
    AW_DEV_9,
    AW_DEV_10,
    AW_DEV_11,
    AW_DEV_12,
    AW_DEV_13,
    AW_DEV_14,
    AW_DEV_15,
    AW_DEV_MAX,
} aw_dev_index_t;

typedef enum
{
    AW_PHASE_SYNC_DISABLE = 0,
    AW_PHASE_SYNC_ENABLE,
} aw_phase_sync_t;

/********************************************
 * error symbol
 *******************************************/
#ifndef EPERM
    #define EPERM        1  /* Operation not permitted */
    #define ENOENT       2  /* No such file or directory */
    #define ESRCH        3  /* No such process */
    #define EINTR        4  /* Interrupted system call */
    #define EIO      5  /* I/O error */
    #define ENXIO        6  /* No such device or address */
    #define E2BIG        7  /* Argument list too long */
    #define ENOEXEC      8  /* Exec format error */
    #define EBADF        9  /* Bad file number */
    #define ECHILD      10  /* No child processes */
    #define EAGAIN      11  /* Try again */
    #define ENOMEM      12  /* Out of memory */
    #define EACCES      13  /* Permission denied */
    #define EFAULT      14  /* Bad address */
    #define ENOTBLK     15  /* Block device required */
    #define EBUSY       16  /* Device or resource busy */
    #define EEXIST      17  /* File exists */
    #define EXDEV       18  /* Cross-device link */
    #define ENODEV      19  /* No such device */
    #define ENOTDIR     20  /* Not a directory */
    #define EISDIR      21  /* Is a directory */
    #define EINVAL      22  /* Invalid argument */
    #define ENFILE      23  /* File table overflow */
    #define EMFILE      24  /* Too many open files */
    #define ENOTTY      25  /* Not a typewriter */
    //#define   ETXTBSY     26  /* Text file busy */
    #define EFBIG       27  /* File too large */
    #define ENOSPC      28  /* No space left on device */
    #define ESPIPE      29  /* Illegal seek */
    #define EROFS       30  /* Read-only file system */
    #define EMLINK      31  /* Too many links */
    #define EPIPE       32  /* Broken pipe */
    #define EDOM        33  /* Math argument out of domain of func */
    #define ERANGE      34  /* Math result not representable */
#endif


/********************************************
 * print information control:Modify print function according to different platform
 *******************************************/
//#define AWINIC_DEBUG_LOG
//#define AWINIC_INFO_LOG
#define AWINIC_ERR_LOG


#ifdef AWINIC_ERR_LOG
#define aw_dev_err(dev_index, format, ...); \
    do { \
        rt_kprintf("[Awinic][dev%d]%s: " format "\r\n", dev_index, __func__, ##__VA_ARGS__);  \
    } while (0)
#else
#define aw_dev_err(dev_index, format, ...)
#endif

#ifdef AWINIC_INFO_LOG
#define aw_dev_info(dev_index, format, ...) \
    do { \
        rt_kprintf("[Awinic][dev%d]%s: " format "\r\n", dev_index, __func__, ##__VA_ARGS__);  \
    } while (0)
#else
#define aw_dev_info(dev_index, format, ...)
#endif

#ifdef AWINIC_DEBUG_LOG
#define aw_dev_dbg(dev_index, format, ...); \
    do { \
        rt_kprintf("[Awinic][dev%d]%s: " format "\r\n", dev_index, __func__, ##__VA_ARGS__);  \
    } while (0)
#else
#define aw_dev_dbg(dev_index, format, ...)
#endif


#ifdef AWINIC_ERR_LOG
#define aw_pr_err(format, ...); \
    do { \
        rt_kprintf("[Awinic]%s: " format "\r\n", __func__, ##__VA_ARGS__);  \
    } while (0)
#else
#define aw_pr_err(format, ...)
#endif

#ifdef AWINIC_INFO_LOG
#define aw_pr_info(format, ...); \
    do { \
        rt_kprintf("[Awinic]%s: " format "\r\n", __func__, ##__VA_ARGS__);  \
    } while (0)
#else
#define aw_pr_info(format, ...)
#endif

#ifdef AWINIC_DEBUG_LOG
#define aw_pr_dbg(format, ...); \
    do { \
        rt_kprintf("[Awinic]%s: " format "\r\n", __func__, ##__VA_ARGS__);  \
    } while (0)
#else
#define aw_pr_dbg(format, ...)
#endif


#endif

