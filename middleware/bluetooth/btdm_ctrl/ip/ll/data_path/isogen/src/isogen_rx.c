/**
 ****************************************************************************************
 *
 * @file isogen_rx.c
 *
 * @brief Isochronous data generator reception path
 *
 * Copyright (C) RivieraWaves 2009-2017
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

#include <string.h>              // standard definition
#include "co_math.h"
#include "co_bt.h"
#include "co_endian.h"
#include "isogen_int.h"
#include "ke_mem.h"
#include "dbg_swdiag.h"


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

__STATIC uint8_t isogen_rx_start(uint16_t conhdl, uint32_t sdu_interval, uint32_t trans_latency, uint16_t max_sdu);
__STATIC void isogen_rx_stop(uint16_t conhdl, uint8_t reason);
__STATIC uint8_t *isogen_rx_sdu_get(uint16_t conhdl, uint32_t *p_ref_time, uint16_t *p_sdu_len);
__STATIC void isogen_rx_sdu_done(uint16_t conhdl, uint16_t sdu_len, uint32_t bts, uint8_t *p_buf, uint8_t status);

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// ISOGEN RX interface
const struct data_path_itf isogen_rx_itf =
{
    isogen_rx_start,
    isogen_rx_stop,
    isogen_rx_sdu_get,
    isogen_rx_sdu_done,
    NULL,
    NULL,
};

/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */

__STATIC uint8_t isogen_rx_start(uint16_t conhdl, uint32_t sdu_interval, uint32_t trans_latency, uint16_t max_sdu)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);

    if (channel < BLE_ACTIVITY_MAX)
    {
        struct isogen_rx_info *p_rx_env = isogen_env.p_rx[channel];

        if (p_rx_env == NULL)
        {
            struct isogen_stat    *p_stat;
            // Compute number of TX buffer
            uint8_t nb_buf = CO_DIVIDE_CEIL(trans_latency, sdu_interval);
            // Compute environment size
            uint16_t env_size = sizeof(struct isogen_rx_info) + (sizeof(uint8_t *) * nb_buf) + (nb_buf * max_sdu);
            uint8_t *p_buf_cursor;
            uint8_t i;

            // Initialize statistics
            isogen_stat_create(channel, ISO_SEL_RX);
            p_stat = isogen_env.p_stat[channel];

            // Allocate environment memory for Controller to Host path
            p_rx_env = (struct isogen_rx_info *) ke_malloc_system(env_size, KE_MEM_ENV);
            memset(p_rx_env, 0, env_size);

            p_rx_env->nb_buf             = nb_buf;
            p_rx_env->max_sdu            = max_sdu;
            p_rx_env->buf_idx            = 0;
            p_rx_env->sdu_cnt            = 0;
            p_rx_env->first_sdu_received = false;

            p_buf_cursor = ((uint8_t *) p_rx_env) + sizeof(struct isogen_rx_info) + (sizeof(uint8_t *) * nb_buf);

            // Allocate number of buffer required
            for (i = 0; i < nb_buf; i++)
            {
                p_rx_env->buf[i] = p_buf_cursor;
                p_buf_cursor     += max_sdu;
            }

            isogen_env.p_rx[channel] = p_rx_env;

            // Prepare everything to start channel
            p_stat->started          = true;

            status = CO_ERROR_NO_ERROR;
        }
    }

    return (status);
}

__STATIC void isogen_rx_stop(uint16_t conhdl, uint8_t reason)
{
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);

    // Check channel available and active
    if (channel < BLE_ACTIVITY_MAX)
    {
        struct isogen_rx_info *p_rx_env = isogen_env.p_rx[channel];

        // Check that channel is available
        if (p_rx_env != NULL)
        {
            // Clear environment
            ke_free(p_rx_env);
            isogen_env.p_rx[channel] = NULL;

            // stop tx statistics
            isogen_stat_cleanup(channel, ISO_SEL_RX);
        }
    }
}

__STATIC uint8_t *isogen_rx_sdu_get(uint16_t conhdl, uint32_t *p_ref_time, uint16_t *p_sdu_len)
{
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);
    uint8_t *p_buf = NULL;

    if (channel < BLE_ACTIVITY_MAX)
    {
        struct isogen_rx_info *p_rx_env = isogen_env.p_rx[channel];

        if (p_rx_env != NULL)
        {
            *p_sdu_len = p_rx_env->max_sdu;
            p_buf      = p_rx_env->buf[p_rx_env->buf_idx];
        }
    }

    return (p_buf);
}

__STATIC void isogen_rx_sdu_done(uint16_t conhdl, uint16_t sdu_len, uint32_t ref_time, uint8_t *p_buf, uint8_t status)
{
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);

    if (channel < BLE_ACTIVITY_MAX)
    {
        struct isogen_rx_info *p_rx_env = isogen_env.p_rx[channel];
        struct isogen_stat    *p_stat   = isogen_env.p_stat[channel];

        // Check if the data path exists
        if ((p_rx_env != NULL) && (p_rx_env->nb_buf > 0))
        {
            // Increment SDU counter
            p_rx_env->sdu_cnt++;
            p_stat->nb_rx++;
            CO_VAL_INC(p_rx_env->buf_idx, p_rx_env->nb_buf);

            switch (status)
            {
            case SDU_OK:
            {
                p_stat->nb_rx_ok++;

                // Check SDU length
                if (sdu_len == 0)
                {
                    // Check if zero-length SDU is expected
                    if (p_stat->payload_type == ISO_TEST_ZERO_LEN)
                    {
                        p_stat->rx_pkt_cnt++;
                    }
                    else
                    {
                        p_stat->failed_pkt_cnt++;
                    }
                    p_stat->nb_rx_empty++;
                }
                else if (sdu_len < 4)
                {
                    p_stat->failed_pkt_cnt++;
                }
                else
                {
                    // Check if zero-length SDU is expected
                    if (p_stat->payload_type == ISO_TEST_ZERO_LEN)
                    {
                        p_stat->failed_pkt_cnt++;
                    }
                    else
                    {
                        uint32_t sdu_cnt_rx = co_btohl(co_read32p(p_buf));

                        // In Framed mode, the SDU counter offset must be computed according to first valid SDU received
                        if (!p_rx_env->first_sdu_received)
                        {
                            p_rx_env->sdu_cnt = sdu_cnt_rx;
                            p_rx_env->first_sdu_received = true;
                        }

                        // Check SDU counter
                        if (sdu_cnt_rx == p_rx_env->sdu_cnt)
                        {
                            p_stat->rx_pkt_cnt++;
                        }
                        else
                        {
                            p_stat->failed_pkt_cnt++;
                        }
                    }
                }
            }
            break;
            case SDU_SYNC_ERR:
            {
                p_stat->nb_rx_sync_err++;
                p_stat->missed_pkt_cnt++;
            }
            break;
            case SDU_SIZE_EXCEED:
            case SDU_UNEXPECTED_FORMAT:
            case SDU_PARTIALLY_RECEIVED:
            case SDU_PKT_LOST:
            case SDU_CRC_ERR:
            {
                p_stat->nb_rx_crc_err++;
                p_stat->missed_pkt_cnt++;
            }
            break;
            case SDU_NOT_GRANTED:
            {
                p_stat->nb_rx_not_granted++;
                p_stat->missed_pkt_cnt++;
            }
            break;
            default: // Not expected
            {
                ASSERT_INFO_FORCE(0, channel, status);
            }
            break;
            }
        }
    }
}


/*
 * EXPORTED FUNCTION DEFINITION
 ****************************************************************************************
 */


#endif //(BLE_ISOGEN)

/// @} ISOGEN
