/**
 ****************************************************************************************
 *
 * @file isogen_tx.c
 *
 * @brief Isochronous data generator transmission path
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
#include "isogen_int.h"
#include "dbg_swdiag.h"
#include "ke_mem.h"
#include "co_math.h"
#include "co_bt.h"
#include "co_endian.h"
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
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */

__STATIC uint8_t isogen_tx_start(uint16_t conhdl, uint32_t sdu_interval, uint32_t trans_latency, uint16_t max_sdu);
__STATIC void isogen_tx_stop(uint16_t conhdl, uint8_t reason);
__STATIC uint8_t *isogen_tx_sdu_get(uint16_t conhdl, uint32_t *p_ref_time, uint16_t *p_sdu_len);
__STATIC void isogen_tx_sdu_done(uint16_t conhdl, uint16_t sdu_len, uint32_t bts, uint8_t *p_buf, uint8_t status);

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// ISOGEN TX interface
const struct data_path_itf isogen_tx_itf =
{
    isogen_tx_start,
    isogen_tx_stop,
    isogen_tx_sdu_get,
    isogen_tx_sdu_done,
    NULL,
    NULL,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

__STATIC uint8_t isogen_tx_start(uint16_t conhdl, uint32_t sdu_interval, uint32_t trans_latency, uint16_t max_sdu)
{
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    if (channel < BLE_ACTIVITY_MAX)
    {
        struct isogen_tx_info *p_tx_env = isogen_env.p_tx[channel];

        if (p_tx_env == NULL)
        {
            struct isogen_stat    *p_stat;
            // Compute number of TX buffer
            uint8_t nb_buf = CO_DIVIDE_CEIL(trans_latency, sdu_interval);
            // Compute environment size
            uint16_t env_size = sizeof(struct isogen_tx_info) + (sizeof(uint8_t *) * nb_buf) + (nb_buf * max_sdu);
            uint8_t *p_buf_cursor;
            uint8_t i;

            // Initialize statistics
            isogen_stat_create(channel, ISO_SEL_TX);
            p_stat = isogen_env.p_stat[channel];

            // Allocate environment memory for Controller to Host path
            p_tx_env = (struct isogen_tx_info *) ke_malloc_system(env_size, KE_MEM_ENV);

            p_tx_env->sdu_interval = sdu_interval;
            p_tx_env->nb_buf   = nb_buf;
            p_tx_env->max_sdu  = max_sdu;
            p_tx_env->sdu_cnt  = 0;
            p_tx_env->buf_idx  = 0;
            p_tx_env->next_sdu_ref_time = 0; // First SDU timestamp will be generated at the first request

            p_buf_cursor = ((uint8_t *) p_tx_env) + sizeof(struct isogen_tx_info) + (sizeof(uint8_t *) * nb_buf);

            // Allocate number of buffer required
            for (i = 0; i < nb_buf; i++)
            {
                p_tx_env->buf[i] = p_buf_cursor;
                p_buf_cursor       += max_sdu;
            }

            isogen_env.p_tx[channel] = p_tx_env;

            // Prepare everything to start channel
            p_stat->started          = true;

            status = CO_ERROR_NO_ERROR;
        }
    }

    return (status);
}

__STATIC void isogen_tx_stop(uint16_t conhdl, uint8_t reason)
{
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);

    // Check that channel exists and is active
    if (channel < BLE_ACTIVITY_MAX)
    {
        struct isogen_tx_info *p_tx_env = isogen_env.p_tx[channel];

        // Check that channel is available
        if (p_tx_env != NULL)
        {
            // Clear environment
            ke_free(p_tx_env);
            isogen_env.p_tx[channel] = NULL;

            // Stop TX statistics
            isogen_stat_cleanup(channel, ISO_SEL_TX);
        }
    }
}

__STATIC  uint8_t *isogen_tx_sdu_get(uint16_t conhdl, uint32_t *p_ref_time, uint16_t *p_sdu_len)
{
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);
    uint8_t *p_buf = NULL;

    if (channel < BLE_ACTIVITY_MAX)
    {
        struct isogen_tx_info *p_tx_env = isogen_env.p_tx[channel];

        if (p_tx_env != NULL)
        {
            // If first SDU request
            if (p_tx_env->next_sdu_ref_time == 0)
            {
                // Generate the timestamp of the first SDU
                p_tx_env->next_sdu_ref_time = CLK_BTS_SUB(*p_ref_time, 1000);
            }

            // Check if a new SDU can be generated
            if (!CLK_BTS_LOWER_EQ(*p_ref_time, p_tx_env->next_sdu_ref_time))
            {
                struct isogen_stat     *p_stat     = isogen_env.p_stat[channel];
                uint16_t sdu_len = p_tx_env->max_sdu;

                ASSERT_ERR(p_stat != NULL);

                // Check payload type
                if (p_stat->payload_type == ISO_TEST_ZERO_LEN)
                {
                    sdu_len = 0;
                }
                else if ((p_stat->payload_type == ISO_TEST_VARIABLE_LEN) && (sdu_len > ISO_TEST_PKT_CNT_SIZE))
                {
                    sdu_len = ISO_TEST_PKT_CNT_SIZE + CO_MOD(co_rand_hword(), (p_tx_env->max_sdu - ISO_TEST_PKT_CNT_SIZE + 1));
                }

                // Return SDU information
                *p_sdu_len  = sdu_len;
                *p_ref_time = p_tx_env->next_sdu_ref_time;
                p_buf       = p_tx_env->buf[p_tx_env->buf_idx];

                // Increment SDU counter
                p_tx_env->sdu_cnt++;
                CO_VAL_INC(p_tx_env->buf_idx, p_tx_env->nb_buf);
                p_tx_env->next_sdu_ref_time += p_tx_env->sdu_interval;

                if (sdu_len >= ISO_TEST_PKT_CNT_SIZE)
                {
                    co_write32p(p_buf, co_htobl(p_tx_env->sdu_cnt));
                    memset(p_buf + ISO_TEST_PKT_CNT_SIZE, p_buf[0], sdu_len - ISO_TEST_PKT_CNT_SIZE);
                }
            }
        }
    }

    return (p_buf);
}

__STATIC void isogen_tx_sdu_done(uint16_t conhdl, uint16_t sdu_len, uint32_t ref_time, uint8_t *p_buf, uint8_t status)
{
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);

    if (channel < BLE_ACTIVITY_MAX)
    {
        struct isogen_tx_info *p_tx_env = isogen_env.p_tx[channel];
        struct isogen_stat    *p_stat   = isogen_env.p_stat[channel];

        if ((p_tx_env != NULL))
        {
            // Update statistics
            p_stat->nb_tx++;
            if (status == SDU_NOT_GRANTED)
            {
                p_stat->nb_tx_not_granted++;

                DBG_SWDIAG(ISOGEN, TX_NOT_GRANT, 1);
                DBG_SWDIAG(ISOGEN, TX_NOT_GRANT, 0);
            }
            else
            {
                p_stat->nb_tx_ok++;
                DBG_SWDIAG(ISOGEN, TX_OK, 1);
                DBG_SWDIAG(ISOGEN, TX_OK, 0);
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
