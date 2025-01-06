/**
 ****************************************************************************************
 *
 * @file ahi.h
 *
 * @brief This file contains definitions related to the Application Host Interface
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef AHI_H_
#define AHI_H_

/**
 ****************************************************************************************
 * @addtogroup AHI Application Host Interface
 * @ingroup AHI
 * @brief Application Host Interface, based on AHI functionality.
 *
 *@{
 *
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (AHI_TL_SUPPORT)
#include "rwip.h"            // SW interface

#include "stdbool.h"         // boolean definition
#include "ke_msg.h"          // kernel message definition
#include "co_list.h"         // list API



/*
 * Defines
 ****************************************************************************************
 */
///Kernel message header length for transport through interface between App and SW stack.
#define AHI_KE_MSG_HDR_LEN       8

/// GAPM Reset size (LC + KE_HEADER + Reset_operation code)
#define AHI_RESET_MSG_LEN    (1+AHI_KE_MSG_HDR_LEN+1)
/// GAPM Reset Message use to resync.
#define AHI_RESET_MSG_BUF    {AHI_KE_MSG_TYPE,                                           \
                              (GAPM_RESET_CMD & 0xFF), ((GAPM_RESET_CMD >> 8) & 0xFF),   \
                              (TASK_ID_GAPM   & 0xFF), ((TASK_ID_GAPM   >> 8) & 0xFF),   \
                              (TASK_ID_AHI    & 0xFF), ((TASK_ID_AHI    >> 8) & 0xFF),   \
                              0x01, 0x00, GAPM_RESET};


/// Size of the sync pattern (RW!)
#define AHI_SYNC_PATTERN_SIZE              3
/// Transport layer synchronization pattern
#define AHI_SYNC_PATTERN_BUF               "RW!"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

///Structure for application system interface packet header
struct ahi_kemsghdr
{
    ///Message id
    ke_msg_id_t  id;
    ///Destination task identifier for KE
    ke_task_id_t dest_id;
    ///Source task identifier for KE
    ke_task_id_t src_id;
    ///Message parameter length
    uint16_t     param_len;
};


/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief AHI initialization function: initializes states and transport.
 *****************************************************************************************
 */
void ahi_init(void);


/**
 ****************************************************************************************
 * @brief Send a data message over the Application to Host Interface
 *
 * @param[in] msg_type Message type (logical channel)
 * @param[in] len      Message data length
 * @param[in] data     Data buffer to send
 * @param[in] msg_type Callback called when message is send. It provides message pointer
 *****************************************************************************************
 */
void ahi_send_msg(uint8_t msg_type, uint16_t len, uint8_t *data, void (*tx_callback)(uint8_t *));

uint8_t *ahi_rx_hdr_handle(uint8_t *buffer, uint16_t *length);


void ahi_rx_done(uint8_t *buffer);

void ahi_reset(void);


#endif //AHI_TL_SUPPORT

/// @} AHI
#endif // AHI_H_
