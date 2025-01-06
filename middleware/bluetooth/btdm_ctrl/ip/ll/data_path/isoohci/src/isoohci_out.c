/**
 ****************************************************************************************
 *
 * @file isoohci_out.c
 *
 * @brief Isochronous over HCI output (Controller to Host)
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup ISOOHCI
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_ISOOHCI)

#include "isoohci_int.h"

#include "co_math.h"
#include "dbg_swdiag.h"
#include "ke_mem.h"
#include "ke_event.h"
#include "ke_msg.h"
#include "h4tl.h"
#include "hci.h"
#include "co_bt.h"
#include "lli.h"

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
__STATIC uint8_t isoohci_out_start(uint16_t conhdl, uint32_t sdu_interval, uint32_t trans_latency, uint16_t max_sdu);
__STATIC void isoohci_out_stop(uint16_t conhdl, uint8_t reason);
__STATIC uint8_t *isoohci_out_sdu_get(uint16_t conhdl, uint32_t *p_ref_time, uint16_t *p_sdu_len);
__STATIC void isoohci_out_sdu_done(uint16_t conhdl, uint16_t sdu_len, uint32_t ref_time, uint8_t *p_buf, uint8_t status);


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// ISOoHCI Output interface
const struct data_path_itf isoohci_out_itf =
{
    isoohci_out_start,
    isoohci_out_stop,
    isoohci_out_sdu_get,
    isoohci_out_sdu_done,
    NULL,
    NULL,
};


/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */

__STATIC uint8_t isoohci_out_start(uint16_t conhdl, uint32_t sdu_interval, uint32_t trans_latency, uint16_t max_sdu)
{
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    if (channel < BLE_ACTIVITY_MAX)
    {
        struct isoohci_out_info *p_out_env = isoohci_env.p_out[channel];

        if (p_out_env == NULL)
        {
            // Compute number of TX buffer
            uint8_t nb_buf = CO_DIVIDE_CEIL(trans_latency, sdu_interval);
            // Compute environment size
            uint16_t buf_size = CO_ALIGN4_HI(sizeof(isoohci_buffer_t) + max_sdu);
            uint8_t *p_buf_data;

            // Allocate environment memory for Controller to Host path
            p_out_env = (struct isoohci_out_info *) ke_malloc_system(CO_ALIGN4_HI(sizeof(struct isoohci_out_info)) + (buf_size * nb_buf), KE_MEM_ENV);
            p_out_env->nb_buf = nb_buf;
            p_out_env->buf_idx = 0;
            p_out_env->sdu_cnt = 0;
            p_out_env->nb_buf_in_use = 0;
            p_out_env->remove = false;
            p_out_env->max_sdu = max_sdu;

            p_buf_data = ((uint8_t *) p_out_env) + CO_ALIGN4_HI(sizeof(struct isoohci_out_info));

            // initialize buffer queue
            co_list_pool_init(&(p_out_env->buffer_queue), p_buf_data, buf_size, nb_buf);

            co_list_init(&(p_out_env->rx_queue));

            isoohci_env.p_out[channel] = p_out_env;

            status = CO_ERROR_NO_ERROR;
        }
    }

    return (status);
}

__STATIC void isoohci_out_stop(uint16_t conhdl, uint8_t reason)
{
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);

    // Check channel available and active
    if (channel < BLE_ACTIVITY_MAX)
    {
        struct isoohci_out_info *p_out_env = isoohci_env.p_out[channel];

        // Check that channel is available and data path can be removed
        if (p_out_env != NULL)
        {
            if (p_out_env->nb_buf_in_use == 0)
            {
                // Everything has been properly cleaned
                ke_free(p_out_env);
                isoohci_env.p_out[channel] = NULL;
            }
            else
            {
                // Defer the removal until packets are transmitted to the Host
                p_out_env->remove = true;
            }
        }
    }
}

__STATIC void isoohci_out_sdu_done(uint16_t conhdl, uint16_t sdu_len, uint32_t ref_time, uint8_t *p_buf, uint8_t status)
{
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);

    DBG_SWDIAG(ISOOHCI, RX_ISO, 1);

    if (channel < BLE_ACTIVITY_MAX)
    {
        struct isoohci_out_info *p_out_env = isoohci_env.p_out[channel];

        // Handles the isochronous interrupt to process isochronous data.
        if ((p_out_env != NULL) && (p_buf != NULL))
        {
            isoohci_buffer_t *p_out_buf = (isoohci_buffer_t *) co_list_pop_front(&(p_out_env->rx_queue));
            p_out_env->nb_buf_in_use++;

            if (p_out_buf != NULL)
            {
                ASSERT_ERR(&(p_out_buf->sdu[0]) == p_buf);

                p_out_buf->time_stamp  = ref_time;
                p_out_buf->sdu_length  = sdu_len;

                switch (status)
                {
                case SDU_OK:
                {
                    // Valid data. The ISO_SDU field belongs to a valid received isochronous
                    // data packet in the Link Layer.
                    p_out_buf->status = HCI_ISO_PKT_STAT_FLAG_VALID;
                }
                break;
                case SDU_SIZE_EXCEED:
                case SDU_UNEXPECTED_FORMAT:
                case SDU_PARTIALLY_RECEIVED:
                case SDU_PKT_LOST:
                case SDU_CRC_ERR:
                {
                    // Possibly invalid data. The ISO_SDU field belongs to a received isochronous
                    // data packet marked by the Link Layer as "data with possible errors".
                    p_out_buf->status = HCI_ISO_PKT_STAT_FLAG_INVALID;
                }
                break;
                case SDU_NOT_GRANTED:
                case SDU_SYNC_ERR:
                {
                    // No packet received. The ISO_SDU field belongs to an isochronous data
                    // packet marked by the Link Layer as "lost data".
                    p_out_buf->status = HCI_ISO_PKT_STAT_FLAG_LOST;
                }
                break;
                default: // Not expected
                {
                    ASSERT_INFO_FORCE(0, channel, status);
                }
                break;
                }

                co_list_push_back(&(isoohci_env.hci_wait_tx_queue), &(p_out_buf->hdr));

                // Defer isochronous data process
                ke_event_set(KE_EVENT_ISOOHCI_OUT_DEFER);
            }
        }
    }

    DBG_SWDIAG(ISOOHCI, RX_ISO, 0);
}

__STATIC uint8_t *isoohci_out_sdu_get(uint16_t conhdl, uint32_t *p_ref_time, uint16_t *p_sdu_len)
{
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);
    uint8_t *p_buf = NULL;

    if (channel < BLE_ACTIVITY_MAX)
    {
        struct isoohci_out_info *p_out_env = isoohci_env.p_out[channel];

        if (p_out_env != NULL)
        {
            isoohci_buffer_t *p_out_buf = (isoohci_buffer_t *) co_list_pop_front(&(p_out_env->buffer_queue));

            // Increment SDU counter
            p_out_env->sdu_cnt++;

            if (p_out_buf != NULL)
            {
                p_out_buf->conhdl      = conhdl;
                p_out_buf->buf_idx     = p_out_env->buf_idx;
                p_out_buf->pkt_seq_nb  = (uint16_t) p_out_env->sdu_cnt;
                p_out_buf->time_stamp  = *p_ref_time;
                CO_VAL_INC(p_out_env->buf_idx, p_out_env->nb_buf);

                p_buf                  = &(p_out_buf->sdu[0]);
                *p_sdu_len             = p_out_env->max_sdu;

                co_list_push_back(&(p_out_env->rx_queue), &(p_out_buf->hdr));
            }
        }
    }

    return (p_buf);
}

/*
 * FUNCTION DEFINITION
 ****************************************************************************************
 */

void isoohci_out_defer_handler(void)
{
    // clear event
    ke_event_clear(KE_EVENT_ISOOHCI_OUT_DEFER);

    while (!co_list_is_empty(&(isoohci_env.hci_wait_tx_queue)))
    {
        isoohci_buffer_t    *p_out_buf;
        struct hci_iso_data *p_iso_data;
        uint16_t             conhdl;
        // get buffer to transmit
        GLOBAL_INT_DISABLE();
        p_out_buf = (isoohci_buffer_t *) co_list_pop_front(&(isoohci_env.hci_wait_tx_queue));
        GLOBAL_INT_RESTORE();

        conhdl = p_out_buf->conhdl;

        p_iso_data = KE_MSG_ALLOC(HCI_ISO_DATA, conhdl, 0, hci_iso_data);

        // Connection handle
        p_iso_data->conhdl_pbf_tsf = conhdl;

        // The HCI packet contains a complete SDU
        SETF(p_iso_data->conhdl_pbf_tsf, HCI_ISO_HDR_PB_FLAG, PB_FLAG_CMP_FRAG);

        // Timestamp present
        SETB(p_iso_data->conhdl_pbf_tsf, HCI_ISO_HDR_TS_FLAG, 1);

        // ISO_Data_Load Length
        p_iso_data->iso_data_load_len = p_out_buf->sdu_length + HCI_ISO_DATA_LOAD_HDR_LEN_MAX;

        /*
         * ISO_Data_Load
         */
        p_iso_data->iso_data_load.time_stamp             = p_out_buf->time_stamp;
        p_iso_data->iso_data_load.pkt_seq_nb             = p_out_buf->pkt_seq_nb;
        p_iso_data->iso_data_load.iso_sdu_length_psf     = p_out_buf->sdu_length;
        SETF(p_iso_data->iso_data_load.iso_sdu_length_psf, HCI_ISO_DATA_LOAD_PKT_STAT_FLAG, p_out_buf->status);
        p_iso_data->iso_data_load.iso_sdu_ptr            = &p_out_buf->sdu[0];

        // put buffer in wait queue
        co_list_push_back(&(isoohci_env.hci_in_tx_queue), &(p_out_buf->hdr));

        // send the message
        hci_send_2_host(p_iso_data);
    }
}

void isoohci_out_buf_release(uint8_t *iso_sdu_ptr)
{
    isoohci_buffer_t *p_out_buf;
    struct isoohci_out_info *p_out_env;
    uint8_t channel;

    GLOBAL_INT_DISABLE();
    p_out_buf = (isoohci_buffer_t *) co_list_pop_front(&(isoohci_env.hci_in_tx_queue));
    channel = BLE_ISOHDL_TO_ACTID(p_out_buf->conhdl);
    p_out_env = isoohci_env.p_out[channel];

    // If channel is still active
    if (p_out_env != NULL)
    {
        ASSERT_ERR(&p_out_buf->sdu[0] == iso_sdu_ptr);

        // push buffer in LL buffer reception queue
        co_list_push_back(&(p_out_env->buffer_queue), &(p_out_buf->hdr));
        p_out_env->nb_buf_in_use--;

        // If data path needs to be removed
        if (p_out_env->remove && (p_out_env->nb_buf_in_use == 0))
        {
            // Everything has been properly cleaned
            ke_free(p_out_env);
            isoohci_env.p_out[channel] = NULL;
        }
    }
    GLOBAL_INT_RESTORE();
}

#endif //(BLE_ISOOHCI)

/// @} ISOOHCI
