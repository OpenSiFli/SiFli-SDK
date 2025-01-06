/**
 ****************************************************************************************
 *
 * @file isogen.h
 *
 * @brief Declaration of the functions used for Isochronous Payload Generator transport layer
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup ISOGEN
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_ISOGEN)

#include "rwip.h"
#include "isogen_int.h"
#include "ke_mem.h"
#include "ke_event.h"
#include "co_list.h"
#include "arch.h"
#include "hci.h"
#include "ke_msg.h"
#include "lli.h"

#include <string.h>
#include "co_math.h"

/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */



/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Isochronous Payload Generator TL environment
struct isogen_env_tag isogen_env;

/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DEFINITION
 ****************************************************************************************
 */

void isogen_init(bool is_reset)
{
    if (!is_reset)
    {
        // Initialize environment variable
        memset(&isogen_env.p_tx, 0, sizeof(isogen_env.p_tx));
        memset(&isogen_env.p_rx, 0, sizeof(isogen_env.p_rx));
        memset(&isogen_env.p_stat, 0, sizeof(isogen_env.p_stat));
    }
    else
    {
        // Clean-up open channel
        for (uint8_t channel = 0 ; channel < BLE_ACTIVITY_MAX ; channel++)
        {
            if (isogen_env.p_tx[channel] != NULL)
            {
                ke_free(isogen_env.p_tx[channel]);
                isogen_env.p_tx[channel] = NULL;
            }

            if (isogen_env.p_rx[channel] != NULL)
            {
                ke_free(isogen_env.p_rx[channel]);
                isogen_env.p_rx[channel] = NULL;
            }

            if (isogen_env.p_stat[channel] != NULL)
            {
                ke_free(isogen_env.p_stat[channel]);
                isogen_env.p_stat[channel] = NULL;
            }
        }
    }
}

const struct data_path_itf *isogen_itf_get(uint8_t direction)
{
    const struct data_path_itf *p_res = NULL;

    // Get the required interface
    switch (direction)
    {
    case ISO_SEL_TX:
    {
        p_res = &isogen_tx_itf;
    }
    break;
    case ISO_SEL_RX:
    {
        p_res = &isogen_rx_itf;
    }
    break;
    default:         { /* Nothing to do */       } break;
    }

    return (p_res);
}

void isogen_stat_create(uint8_t channel, uint8_t direction)
{
    // check if statistics not already available for channel
    if (isogen_env.p_stat[channel] == NULL)
    {
        // allocate an initiate statistics for the channel
        isogen_env.p_stat[channel] = ke_malloc_system(sizeof(struct isogen_stat), KE_MEM_ENV);

        memset(isogen_env.p_stat[channel], 0, sizeof(struct isogen_stat));

        // Initialize test payload type to default maximum length
        isogen_env.p_stat[channel]->payload_type = ISO_TEST_MAX_LEN;
    }

    isogen_env.p_stat[channel]->direction |= CO_BIT(direction);
}

void isogen_stat_cleanup(uint8_t channel, uint8_t direction)
{
    if (isogen_env.p_stat[channel] != NULL)
    {
        isogen_env.p_stat[channel]->direction &= ~CO_BIT(direction);

        // check if no more environment is set for the channel
        if (isogen_env.p_stat[channel]->direction == 0)
        {
            // Clean-up statistics environment
            ke_free(isogen_env.p_stat[channel]);
            isogen_env.p_stat[channel] = NULL;
        }
    }
}

uint8_t isogen_get_rx_stat(uint8_t channel, uint32_t *received_pkt_cnt, uint32_t *missed_pkt_cnt, uint32_t *failed_pkt_cnt)
{
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    struct isogen_stat *p_stat = isogen_env.p_stat[channel];

    if ((p_stat != NULL) && p_stat->started)
    {
        // If RX direction has been configured
        if (isogen_env.p_stat[channel]->direction & CO_BIT(ISO_DP_OUTPUT))
        {
            // Fill RX statistics
            *received_pkt_cnt = p_stat->rx_pkt_cnt;
            *missed_pkt_cnt = p_stat->missed_pkt_cnt;
            *failed_pkt_cnt = p_stat->failed_pkt_cnt;

            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            status = CO_ERROR_UNSUPPORTED;
        }
    }

    if (status != CO_ERROR_NO_ERROR)
    {
        // Set all RX statistics to 0
        *received_pkt_cnt = 0;
        *missed_pkt_cnt = 0;
        *failed_pkt_cnt = 0;
    }

    return (status);
}

uint8_t isogen_payload_type_set(uint8_t channel, uint8_t direction, uint8_t payload_type)
{
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    struct isogen_stat *p_stat = isogen_env.p_stat[channel];

    if ((p_stat != NULL) && p_stat->started)
    {
        // Check direction
        switch (direction)
        {
        case ISO_SEL_TX:
        {
            // If a TX data path has been setup
            if (isogen_env.p_tx[channel] != NULL)
            {
                p_stat->payload_type = payload_type;
                status = CO_ERROR_NO_ERROR;
            }
        }
        break;
        case ISO_SEL_RX:
        {
            // If a RX data path has been setup
            if (isogen_env.p_rx[channel] != NULL)
            {
                p_stat->payload_type = payload_type;
                status = CO_ERROR_NO_ERROR;
            }
        }
        break;
        default:
        {
            // Nothing to do
        }
        break;
        }
    }

    return (status);
}

uint8_t isogen_sdu_cnt_set(uint8_t channel, uint8_t direction, uint32_t sdu_cnt)
{
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    struct isogen_stat *p_stat = isogen_env.p_stat[channel];

    if ((p_stat != NULL) && p_stat->started)
    {
        // Check direction
        switch (direction)
        {
        case ISO_SEL_TX:
        {
            // If a TX data path has been setup
            if (isogen_env.p_tx[channel] != NULL)
            {
                isogen_env.p_tx[channel]->sdu_cnt = sdu_cnt;
                status = CO_ERROR_NO_ERROR;
            }
        }
        break;
        case ISO_SEL_RX:
        {
            // If a RX data path has been setup
            if (isogen_env.p_rx[channel] != NULL)
            {
                isogen_env.p_rx[channel]->sdu_cnt = sdu_cnt;
                isogen_env.p_rx[channel]->first_sdu_received = true;
                status = CO_ERROR_NO_ERROR;
            }
        }
        break;
        default:
        {
            // Nothing to do
        }
        break;
        }
    }

    return (status);
}

#endif //(BLE_ISOGEN)

/// @} ISOGEN
