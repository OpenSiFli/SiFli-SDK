/**
 ****************************************************************************************
 *
 * @file arch_main.c
 *
 * @brief Main loop of the application.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */


/*
 * INCLUDES
 ****************************************************************************************
 */

#include "rtconfig.h"
#include "rwip_config.h" // RW SW configuration
#include "board.h"

#include "arch.h"      // architectural platform definitions
#include <stdlib.h>    // standard lib functions
#include <stddef.h>    // standard definitions
#include <stdint.h>    // standard integer definition
#include <stdbool.h>   // boolean definition
//#include "ipc_queue.h"
#include "rwip.h"      // RW SW initialization
#include "dbg_trc.h"
#include "rom_config.h"

#ifdef BSP_USING_RTTHREAD
#include "rtthread.h"

/**
 ****************************************************************************************
 * @addtogroup DRIVERS
 * @{
 *
 *
 * ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

#ifdef CFG_SERVICE_ON_HCPU
    #define OUT_MB_CH   "mb_b2h1"
    #define IN_MB_CH    "mb_h2b1"
#elif defined (CFG_SERVICE_ON_LCPU)
    #define OUT_MB_CH   "mb_b2l1"
    #define IN_MB_CH    "mb_l2b1"
#else
    #define OUT_MB_CH   "mb_b2h1"
    #define IN_MB_CH    "mb_h2b1"
#endif

/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */


/* TX and RX channel class holding data used for asynchronous read and write data
 * transactions
 */
/// UART TX RX Channel
struct uart_txrxchannel
{
    /// call back function pointer
    void (*callback)(void *, uint8_t);
    /// Dummy data pointer returned to callback when operation is over.
    void *dummy;
    uint8_t *buf;
    uint32_t data_size;
    uint32_t offset;
};

/// UART environment structure
struct uart_env_tag
{
    rt_device_t device;
    /// tx channel
    struct uart_txrxchannel tx;
    /// rx channel
    struct uart_txrxchannel rx;
    /// error detect
    uint8_t errordetect;
    /// external wakeup
    bool ext_wakeup;
};
#if 0
struct mbox_env_tag
{
    ipc_queue_handle_t ipc_port;
    // Adapt for uart
    struct uart_txrxchannel tx;
    struct uart_txrxchannel rx;
    uint8_t is_init;
};


typedef struct
{
    char *write_dev;
    char *read_dev;
} mbox_port_t;
#endif

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */


const struct bt_eif_api *_rwip_eif_get(uint8_t idx);
rwip_eif_get_handler _rwip_eif_get_handler = _rwip_eif_get;

void *rw_os_get_if = (void *) &_rwip_eif_get_handler;

/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */




/**
 ****************************************************************************************
 * @brief Initialize unloaded RAM area
 *
 * The unloaded RAM area is a part of RAM data memory that is not loaded at platform boot.
 * Information written in this area is maintained until device power-off.
 ****************************************************************************************
 */



/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * MAIN FUNCTION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief RW main function.
 *
 * This function is called right after the booting process has completed.
 *
 * @return status   exit status
 ****************************************************************************************
 */


const struct bt_eif_api *_rwip_eif_get(uint8_t idx)
{
    const struct bt_eif_api *ret = NULL;

    switch (idx)
    {
    case 1:
    case 0:
    {
        ret = rom_config_get_tl(idx);
        ASSERT_ERR(ret);
    }
    break;
    default:
    {
        ASSERT_INFO_FORCE(0, idx, 0);
    }
    break;
    }
    return ret;
}


void rw_os_tl_reconfig_for_flush(uint8_t idx)
{
    switch (idx)
    {
    case 0:
    {
#if 0
        if (NULL != uart0_env.device)
        {
            uart0_env.device->open_flag &= ~(RT_DEVICE_FLAG_DMA_TX | RT_DEVICE_FLAG_INT_TX);
        }
#endif
    }
    break;
    default:
    {
        ASSERT_INFO_FORCE(0, idx, 0);
    }
    break;
    }
}

#endif


/// @} DRIVERS
