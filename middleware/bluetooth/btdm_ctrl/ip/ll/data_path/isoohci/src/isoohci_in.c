/**
 ****************************************************************************************
 *
 * @file isoohci_in.c
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

#include "dbg_swdiag.h"
#include "ke_event.h"
#include "ke_mem.h"
#include "ke_msg.h"
#include "h4tl.h"
#include "lli.h"
#include "hci.h"


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
__STATIC uint8_t isoohci_in_start(uint16_t conhdl, uint32_t sdu_interval, uint32_t trans_latency, uint16_t max_sdu);
__STATIC void isoohci_in_stop(uint16_t conhdl, uint8_t reason);
__STATIC uint8_t *isoohci_in_sdu_get(uint16_t conhdl, uint32_t *p_ref_time, uint16_t *p_sdu_len);
__STATIC void isoohci_in_sdu_done(uint16_t conhdl, uint16_t sdu_len, uint32_t ref_time, uint8_t *p_buf, uint8_t status);


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// ISOoHCI Input interface
const struct data_path_itf isoohci_in_itf =
{
    isoohci_in_start,
    isoohci_in_stop,
    isoohci_in_sdu_get,
    isoohci_in_sdu_done,
    NULL,
    NULL,
};


/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * Start ISOHCI input path
 *
 * @param[in]     channel            Isochronous channel
 * @param[in]     sdu_interval       SDU interval (in us)
 * @param[in]     trans_latency      Transport latency (in us)
 * @param[in]     max_sdu            Maximum SDU size (in bytes) (12 bits meaningful)
 *
 * @return status of the operation (@see enum co_error)
 ****************************************************************************************
 */
__STATIC uint8_t isoohci_in_start(uint16_t conhdl, uint32_t sdu_interval, uint32_t trans_latency, uint16_t max_sdu)
{
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    if (channel < BLE_ACTIVITY_MAX)
    {
        struct isoohci_in_info *p_in_env = isoohci_env.p_in[channel];

        if (p_in_env == NULL)
        {
            // Allocate environment memory for Host to Controller path
            p_in_env = (struct isoohci_in_info *) ke_malloc_system(sizeof(struct isoohci_in_info), KE_MEM_ENV);
            p_in_env->max_sdu_size = max_sdu;
            p_in_env->current_buf = NULL;
            p_in_env->buf_in_use = NULL;
            p_in_env->sdu_rx_len = 0;
            p_in_env->last_sdu.pkt_seq_nb = 0;
            p_in_env->last_sdu.time_stamp = 0;
            p_in_env->last_sdu.time_offset = 0;

            co_list_init(&(p_in_env->in_queue));

            isoohci_env.p_in[channel] = p_in_env;

            status = CO_ERROR_NO_ERROR;
        }
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Inform that isochronous stream is stopped
 *
 * @param[in] channel Isochronous Channel number
 * @param[in] status  Isochronous Stop reason
 ****************************************************************************************
 */
__STATIC void isoohci_in_stop(uint16_t conhdl, uint8_t reason)
{
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);

    // Check channel available and active
    if (channel < BLE_ACTIVITY_MAX)
    {
        struct isoohci_in_info *p_in_env = isoohci_env.p_in[channel];

        // Check that channel is available
        if (p_in_env != NULL)
        {
            // Get the first buffer from the input queue
            isoohci_buffer_t *p_in_buf = (isoohci_buffer_t *) co_list_pop_front(&(p_in_env->in_queue));

            while (p_in_buf != NULL)
            {
                // Release buffer
                ke_free(p_in_buf);

                // Move onto the next buffer
                p_in_buf = (isoohci_buffer_t *) co_list_pop_front(&(p_in_env->in_queue));
            }

            // If there is a buffer under construction
            if (p_in_env->current_buf != NULL)
            {
                ke_free(p_in_env->current_buf);
            }

            // If there is a buffer in use by ISOAL
            if (p_in_env->buf_in_use != NULL)
            {
                ke_free(p_in_env->buf_in_use);
            }

            // Everything has been properly cleaned
            ke_free(p_in_env);
            isoohci_env.p_in[channel] = NULL;
        }
    }
}


/**
 ****************************************************************************************
 * Handle buffer TX done for a specific buffer index
 *
 * @param[in] channel  Isochronous channel
 * @param[in] sdu_len  Size of SDU received (only for output direction)
 * @param[in] p_buf    Pointer of the SDU buffer
 * @param[in] bts      SDU synchronization reference (only for output direction) (in us, based on Bluetooth timestamp)
 * @param[in] status   Reception or transmission buffer status (@see enum sdu_status)
 ****************************************************************************************
 */
__STATIC void isoohci_in_sdu_done(uint16_t conhdl, uint16_t sdu_len, uint32_t ref_time, uint8_t *p_buf, uint8_t status)
{
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);

    DBG_SWDIAG(ISOOHCI, RX_ISO, 1);

    if (channel < BLE_ACTIVITY_MAX)
    {
        struct isoohci_in_info *p_in_env = isoohci_env.p_in[channel];

        // Check that ISOoHCI input path exists
        if ((p_in_env != NULL) && (p_buf != NULL))
        {
            isoohci_buffer_t *p_in_buf = (isoohci_buffer_t *) CONTAINER_OF(p_buf, isoohci_buffer_t, sdu);

            ASSERT_ERR(p_in_env->buf_in_use == p_in_buf);

            if (p_in_buf != NULL)
            {
                // Save the number of HCI ISO data packets in the SDU
                isoohci_env.in_sdu_info[isoohci_env.in_sdu_idx].hci_iso_data_pkt_nb = p_in_buf->hci_iso_data_pkt_nb;

                // Release buffer
                ke_free(p_in_buf);
                p_in_env->buf_in_use = NULL;

                // Save the channel
                isoohci_env.in_sdu_info[isoohci_env.in_sdu_idx].conhdl = conhdl;
                CO_VAL_INC(isoohci_env.in_sdu_idx, BLE_HCI_ISO_IN_SDU_BUF_NB);

                // Defer event message
                ke_event_set(KE_EVENT_ISOOHCI_IN_DEFER);
            }
        }
    }

    DBG_SWDIAG(ISOOHCI, RX_ISO, 0);
}

__STATIC uint8_t *isoohci_in_sdu_get(uint16_t conhdl, uint32_t *p_ref_time, uint16_t *p_sdu_len)
{
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);
    uint8_t *p_buf = NULL;

    if (channel < BLE_ACTIVITY_MAX)
    {
        struct isoohci_in_info *p_in_env = isoohci_env.p_in[channel];

        if (p_in_env != NULL)
        {
            ASSERT_ERR(p_in_env->buf_in_use == NULL);

            // Get the first buffer from the input queue
            isoohci_buffer_t *p_in_buf = (isoohci_buffer_t *) co_list_pop_front(&(p_in_env->in_queue));

            if (p_in_buf != NULL)
            {
                // Remember last transmitted SDU
                p_in_env->buf_in_use = p_in_buf;
                p_in_env->last_sdu.time_offset = *p_ref_time - p_in_buf->time_stamp;
                p_in_env->last_sdu.time_stamp = *p_ref_time;
                p_in_env->last_sdu.pkt_seq_nb = p_in_buf->pkt_seq_nb;

                // Timestamp
                *p_ref_time = p_in_buf->time_stamp;

                // SDU length
                *p_sdu_len = p_in_buf->sdu_length;

                // SDU buffer
                p_buf = &(p_in_buf->sdu[0]);
            }
            else
            {
                // No SDU
                *p_sdu_len = 0;
            }
        }
    }

    return (p_buf);
}


/*
 * FUNCTION DEFINITION
 ****************************************************************************************
 */

uint8_t *isoohci_in_buf_alloc(struct hci_iso_hdr *hdr)
{
    uint8_t *buffer = NULL;
    uint16_t conhdl = GETF(hdr->conhdl_flags, HCI_ISO_HDR_HDL);
    uint16_t iso_data_load_len = GETF(hdr->iso_data_load_len, HCI_ISO_HDR_ISO_DATA_LOAD_LEN);
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);

    // check if ISO channel is valid
    if (channel < BLE_ACTIVITY_MAX)
    {
        struct isoohci_in_info *p_in_env = isoohci_env.p_in[channel];

        // Check that HCI ISO input data path exists
        if (p_in_env != NULL)
        {
            uint8_t pb_flag = GETF(hdr->conhdl_flags, HCI_ISO_HDR_PB_FLAG);

            // Check packet boundary flag
            switch (pb_flag)
            {
            case PB_FLAG_1ST_FRAG:
            case PB_FLAG_CMP_FRAG:
            {
                // Check that no SDU is currently in reception
                if (p_in_env->current_buf == NULL)
                {
                    uint8_t iso_data_load_hdr_len = HCI_ISO_DATA_LOAD_HDR_LEN_MAX - (GETB(hdr->conhdl_flags, HCI_ISO_HDR_TS_FLAG) ? 0 : HCI_ISO_DATA_LOAD_TIME_STAMP_LEN);

                    // Check that the ISO_Data_Load_Length is greater than or equal to the ISO_Data_Load header
                    if ((iso_data_load_len >= iso_data_load_hdr_len) && (iso_data_load_len <= (p_in_env->max_sdu_size + iso_data_load_hdr_len)))
                    {
                        // Allocate a free buffer
                        isoohci_buffer_t *p_in_buf = (isoohci_buffer_t *) ke_malloc_system(CO_ALIGN4_HI(sizeof(isoohci_buffer_t) + p_in_env->max_sdu_size), KE_MEM_ENV);

                        if (p_in_buf != NULL)
                        {
                            p_in_buf->conhdl = conhdl;
                            p_in_env->current_buf = p_in_buf;
                            p_in_env->sdu_rx_len = 0;

                            // Provide pointer to the ISO_Data_Load header (with or without Time_Stamp field)
                            buffer = &p_in_buf->hci_iso_data_load_hdr[0] + (GETB(hdr->conhdl_flags, HCI_ISO_HDR_TS_FLAG) ? 0 : HCI_ISO_DATA_LOAD_TIME_STAMP_LEN);
                        }
                    }
                }
            }
            break;
            case PB_FLAG_CONT_FRAG:
            case PB_FLAG_LAST_FRAG:
            {
                // If there is a current buffer in use
                if (p_in_env->current_buf != NULL)
                {
                    isoohci_buffer_t *p_in_buf = p_in_env->current_buf;

                    // Verify that the new length does not overflow the buffer
                    if ((p_in_env->sdu_rx_len + iso_data_load_len) <= p_in_env->max_sdu_size)
                    {
                        // Append new data to the already received data
                        buffer = &p_in_buf->sdu[0] + p_in_env->sdu_rx_len;
                    }
                    else
                    {
                        // Release current buffer
                        ke_free(p_in_buf);
                        p_in_env->current_buf = NULL;
                    }
                }
            }
            break;
            default:
            {
                // Invalid packet boundary flag
            }
            break;
            }
        }
    }

    // If no buffer available
    if (buffer == NULL)
    {
        buffer = ke_malloc_system(iso_data_load_len, KE_MEM_NON_RETENTION);
        isoohci_env.trash_buffer = buffer;
    }

    return buffer;
}

void isoohci_in_buf_received(struct hci_iso_hdr *hdr, uint8_t *buf)
{
    uint16_t conhdl = GETF(hdr->conhdl_flags, HCI_ISO_HDR_HDL);
    uint8_t channel = BLE_ISOHDL_TO_ACTID(conhdl);

    ASSERT_ERR(channel < BLE_ACTIVITY_MAX);

    // Clean-up trash buffer
    if (buf == isoohci_env.trash_buffer)
    {
        ke_free(isoohci_env.trash_buffer);
        isoohci_env.trash_buffer = NULL;
    }
    else
    {
        struct isoohci_in_info *p_in_env = isoohci_env.p_in[channel];
        if (p_in_env != NULL)
        {
            uint8_t pb_flag = GETF(hdr->conhdl_flags, HCI_ISO_HDR_PB_FLAG);
            uint16_t iso_data_load_len = GETF(hdr->iso_data_load_len, HCI_ISO_HDR_ISO_DATA_LOAD_LEN);
            isoohci_buffer_t *p_in_buf = p_in_env->current_buf;

            ASSERT_ERR(p_in_buf != NULL);

            // Check packet boundary flag
            switch (pb_flag)
            {
            case PB_FLAG_CMP_FRAG:
            case PB_FLAG_1ST_FRAG:
            {
                bool ts_flag = GETB(hdr->conhdl_flags, HCI_ISO_HDR_TS_FLAG);
                uint8_t *ptr = &p_in_buf->hci_iso_data_load_hdr[0];
                uint16_t iso_sdu_len_flag;
                uint16_t sdu_rx_len;

                // Extract ISO_DATA_Load header

                // Time_Stamp
                if (ts_flag)
                {
                    p_in_buf->time_stamp = co_read32p(ptr);
                }
                else
                {
                    p_in_buf->time_stamp = rwip_time_get().bts;
                }
                ptr += HCI_ISO_DATA_LOAD_TIME_STAMP_LEN;

                // Packet Sequence Number
                p_in_buf->pkt_seq_nb = co_read16p(ptr);
                ptr += HCI_ISO_DATA_LOAD_PKT_SEQ_NB_LEN;

                // ISO SDU Length
                iso_sdu_len_flag = co_read16p(ptr);
                p_in_buf->sdu_length = GETF(iso_sdu_len_flag, HCI_ISO_DATA_LOAD_ISO_SDU_LEN);

                // Packet Status Flag - ignored from Host to Controller

                // Length of received SDU data
                sdu_rx_len = iso_data_load_len - HCI_ISO_DATA_LOAD_HDR_LEN_MAX + (ts_flag ? 0 : HCI_ISO_DATA_LOAD_TIME_STAMP_LEN);

                // Initialize the number of HCI ISO data packets in the SDU
                p_in_buf->hci_iso_data_pkt_nb = 1;

                // If SDU is complete
                if ((pb_flag == PB_FLAG_CMP_FRAG) || (sdu_rx_len >= p_in_buf->sdu_length))
                {
                    // Add received data packet to the queue
                    co_list_push_back(&p_in_env->in_queue, (co_list_hdr_t *) p_in_buf);

                    // Clear current reception information
                    p_in_env->current_buf = NULL;
                    p_in_env->sdu_rx_len = 0;
                }
                else
                {
                    p_in_env->sdu_rx_len = sdu_rx_len;
                }
            }
            break;
            case PB_FLAG_CONT_FRAG:
            case PB_FLAG_LAST_FRAG:
            {
                // Increment the number of HCI ISO data packets in the SDU
                p_in_buf->hci_iso_data_pkt_nb++;

                // Add received fragment length
                p_in_env->sdu_rx_len += iso_data_load_len;

                // If SDU is complete
                if ((pb_flag == PB_FLAG_LAST_FRAG) || (p_in_env->sdu_rx_len == p_in_buf->sdu_length))
                {
                    // Add received data packet to the queue
                    co_list_push_back(&p_in_env->in_queue, (co_list_hdr_t *) p_in_buf);

                    // Clear current reception information
                    p_in_env->current_buf = NULL;
                    p_in_env->sdu_rx_len = 0;
                }
                else if (p_in_env->sdu_rx_len < p_in_buf->sdu_length)
                {
                    // Keep receiving the remaining fragments
                }
                else
                {
                    ASSERT_INFO_FORCE(0, p_in_env->sdu_rx_len, p_in_buf->sdu_length);
                }
            }
            break;
            default:
            {
                // Invalid packet boundary flag
            }
            break;
            }
        }
    }
}

void isoohci_in_defer_handler(void)
{
    // clear event
    ke_event_clear(KE_EVENT_ISOOHCI_IN_DEFER);

    while (isoohci_env.in_sdu_send_idx != isoohci_env.in_sdu_idx)
    {
        uint16_t conhdl = isoohci_env.in_sdu_info[isoohci_env.in_sdu_send_idx].conhdl;

        // Send the Number Of Completed Packets event
        struct hci_nb_cmp_pkts_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_NB_CMP_PKTS_EVT_CODE, hci_nb_cmp_pkts_evt);

        event->nb_of_hdl          = 1;
        event->con[0].hdl         = conhdl;
        event->con[0].nb_comp_pkt = isoohci_env.in_sdu_info[isoohci_env.in_sdu_send_idx].hci_iso_data_pkt_nb;

        hci_send_2_host(event);

        CO_VAL_INC(isoohci_env.in_sdu_send_idx, BLE_HCI_ISO_IN_SDU_BUF_NB);
    }
}

uint8_t isoohci_in_tx_sync_get(uint8_t act_id, uint16_t *pkt_seq_nb, uint32_t *time_stamp, uint32_t *time_offset)
{
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;

    // Initialize return values
    *pkt_seq_nb = 0;
    *time_stamp = 0;
    *time_offset = 0;

    // Check if channel is valid
    if (act_id < BLE_ACTIVITY_MAX)
    {
        // Check if ISOoHCI Input path is present on this channel
        struct isoohci_in_info *p_in_env = isoohci_env.p_in[act_id];

        if (p_in_env != NULL)
        {
            // Return timing info from last transmitted SDU
            *pkt_seq_nb = p_in_env->last_sdu.pkt_seq_nb;
            *time_stamp = p_in_env->last_sdu.time_stamp;
            *time_offset = p_in_env->last_sdu.time_offset;

            status = CO_ERROR_NO_ERROR;
        }
    }

    return status;
}

#endif //(BLE_ISOOHCI)

/// @} ISOOHCI
