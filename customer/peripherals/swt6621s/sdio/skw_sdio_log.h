/******************************************************************************
 *
 * Copyright(c) 2020-2030  Seekwave Corporation.
 *
 *****************************************************************************/
#ifndef __SKW_SDIO_LOG_H__
#define __SKW_SDIO_LOG_H__
#define LOG_TAG                "drv.sdhci"
#include <drv_log.h>
#include <stdlib.h>

#define SKW_SDIO_ERROR    1//BIT(0)
#define SKW_SDIO_WARNING  1//BIT(1)
#define SKW_SDIO_INFO     0//BIT(2)
#define SKW_SDIO_DEBUG    0//BIT(3)

#define SKW_SDIO_CMD      1//BIT(16)
#define SKW_SDIO_EVENT    1//BIT(17)
#define SKW_SDIO_SCAN     1//BIT(18)
#define SKW_SDIO_TIMER    1//BIT(19)
#define SKW_SDIO_STATE    1//BIT(20)

#define SKW_SDIO_PORT0     1//BIT(21)
#define SKW_SDIO_PORT1     1//BIT(22)
#define SKW_SDIO_PORT2     1//BIT(23)
#define SKW_SDIO_PORT3     1//BIT(24)
#define SKW_SDIO_PORT4     1//BIT(25)
#define SKW_SDIO_PORT5     1//BIT(26)
#define SKW_SDIO_PORT6     1//BIT(27)
#define SKW_SDIO_PORT7     1//BIT(28)
#define SKW_SDIO_SAVELOG     1//BIT(29)
#define SKW_SDIO_DUMP     1//BIT(31)

unsigned long skw_sdio_log_level(void);

#define skw_sdio_log(level, fmt, ...) \
    do { \
        if (level) \
            rt_kprintf(fmt,  ##__VA_ARGS__); \
    } while (0)

#define skw_sdio_port_log(port_num, fmt, ...) \
    do { \
        if (SKW_SDIO_PORT0<<port_num) \
            rt_kprintf(fmt,  ##__VA_ARGS__); \
    } while (0)

#define skw_port_log(port_num,fmt, ...) \
    skw_sdio_log((SKW_SDIO_PORT0<<port_num), "[PORT_LOG] %s: %d \n"fmt, __func__,__LINE__, ##__VA_ARGS__)

#define skw_sdio_err(fmt, ...) \
    skw_sdio_log(SKW_SDIO_ERROR, "[SKWSDIO ERROR] %s: %d \n"fmt, __func__,__LINE__, ##__VA_ARGS__)

#define skw_sdio_warn(fmt, ...) \
    skw_sdio_log(SKW_SDIO_WARNING, "[SKWSDIO WARN] %s: %d \n"fmt, __func__,__LINE__, ##__VA_ARGS__)

#define skw_sdio_info(fmt, ...) \
    skw_sdio_log(SKW_SDIO_INFO, "[SKWSDIO INFO] %s: %d \n"fmt, __func__,__LINE__, ##__VA_ARGS__)

#define skw_sdio_dbg(fmt, ...) \
    skw_sdio_log(SKW_SDIO_DEBUG, "[SKWSDIO DBG] %s: %d \n"fmt, __func__,__LINE__, ##__VA_ARGS__)

#if 0
#define skw_sdio_hex_dump(prefix, buf, len) \
    do { \
        if (skw_sdio_log_level() & SKW_SDIO_DUMP) { \
            u8 str[32] = {0};  \
            snprintf(str, sizeof(str), "[SKWSDIO DUMP] %s", prefix); \
            print_hex_dump(KERN_ERR, str, \
                DUMP_PREFIX_OFFSET, 16, 1, buf, len, true); \
        } \
    } while (0)

#define skw_sdio_port_log(port_num, fmt, ...) \
    do { \
        if (skw_sdio_log_level() &(SKW_SDIO_PORT0<<port_num)) \
            pr_err("[PORT_LOG] %s:"fmt,__func__,  ##__VA_ARGS__); \
    } while (0)

#endif
void skw_sdio_log_level_init(void);
#endif

