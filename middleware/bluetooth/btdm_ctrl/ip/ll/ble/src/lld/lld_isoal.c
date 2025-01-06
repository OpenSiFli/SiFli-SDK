/**
****************************************************************************************
*
* @file lld_isoal.c
*
* @brief LLD Isochronous adaptation layer driver source code
*
* Copyright (C) RivieraWaves 2009-2019
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LLDISOAL
 * @ingroup LLD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"               // stack configuration

#if (BLE_ISO_PRESENT)

#include <string.h>
#include "rwip.h"
#include "lld.h"                       // link driver API
#include "lld_int.h"                   // link layer driver internal
#include "lld_int_iso.h"               // LLD Internal API for ISO
#include "ke_mem.h"                    // Environment Memory allocation

#include "ble_util_buf.h"              // Buffer API
#include "reg_blecore.h"               // BLE core registers
#include "reg_em_ble_rx_iso_buf.h"     // RX ISO buffer api
#include "reg_em_ble_tx_iso_buf.h"     // TX ISO buffer api

#include "data_path.h"                 // Datapath Interface definition

#include "co_endian.h"                 // convert value to air format

//#include "dma.h"                       // HW accelerator for memory copy
#include "dbg.h"                       // Debug SW diags
#include "btdm_patch.h"


/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// SDU information
struct lld_isoal_sdu_inf
{
    /// Memory pointer of SDU
    uint8_t    *p_buf;
    /// SDU synchronization reference (output direction only) (in us, based on Bluetooth timestamp)
    uint32_t    sdu_sync_ref;
    /// Current size of SDU (12 bits meaningful)
    uint16_t    size;
    /// SDU data cursor into the buffer (in bytes)
    uint16_t    cursor;
    /// SDU transmission or reception status (@see enum sdu_status)
    uint8_t     status;
    /// Number of PDU remaining to transmit or receive this SDU (unframed mode only)
    uint8_t     remaining_pdu;
};

/// LLD ISOAL Environment
struct lld_isoal_env_tag
{
    /// Data path interface
    const struct data_path_itf *p_dp;
    /// SDU Interval in microseconds
    uint32_t                    sdu_interval;
    /// Transport latency in microsecond
    uint32_t                    trans_latency;
    /// Offset to apply to synchronization reference (in us) (only for output direction)
    int32_t                     sync_ref_offset;
    /// Expected SDU synchronization reference for the next SDU (in us, based on Bluetooth timestamp) (output direction only)
    uint32_t                    next_sdu_sync_ref;
    /// Maximum deviation of an SDU timestamp (in us) (output direction only)
    uint32_t                    max_sdu_deviation;
    /// Maximum SDU Size (12 bits meaningful) (in bytes)
    uint16_t                    max_sdu;
    /// Connection handle of a CIS or BIS (Range 0x0000-0x0EFF)
    uint16_t                    conhdl;
    /// Frame mode (@see enum iso_frame)
    uint8_t                     framing;
    /// Indication whether Audio Mode 0
    bool                        am0;
    /// Indicate whether PDUs are encrypted using MIC
    bool                        mic_present;
    /// Maximum PDU size (in bytes)
    uint8_t                     max_pdu;
    /// Number of PDU used to transmit an SDU (for unframed mode only)
    uint8_t                     sdu_pdu_nb;
    /// Information about current SDU
    struct lld_isoal_sdu_inf    sdu;
};

/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LLD ISOAL Environment
struct lld_isoal_env_tag *lld_isoal_env[ISO_SEL_MAX][BLE_ACTIVITY_MAX];


/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

// ----------------------------------------------------------------------------------- //
//                                 UNFRAMED MODE                                       //
// ----------------------------------------------------------------------------------- //

__STATIC void lld_isoal_tx_get_unframed(struct lld_isoal_env_tag *p_env, uint8_t iso_buf_idx, uint32_t ref_anchor, bool rejected)
{
    FUNC_PATCH_ENTRY_4_PARAM_NO_RETURN(LLD_ISOAL_PATCH_TYPE, LLD_ISOAL_TX_GET_UNFRAMED_FUNC_BIT, p_env, iso_buf_idx, ref_anchor, rejected);

    struct lld_isoal_sdu_inf *p_sdu = &(p_env->sdu);
    bool first_fragment = false;

    // If current SDU is completed
    if (p_sdu->remaining_pdu == 0)
    {
        // Initialize new SDU
        p_sdu->cursor = 0;
        p_sdu->remaining_pdu = p_env->sdu_pdu_nb;
        p_sdu->status = SDU_OK;

        DBG_SWDIAG(ISOAL, SDU_TX_GET, 1);

        // If data path configured
        if ((p_env->p_dp != NULL) && (p_env->p_dp->cb_sdu_get != NULL))
        {
            // Get SDU buffer from data path
            p_sdu->p_buf = p_env->p_dp->cb_sdu_get(p_env->conhdl, &ref_anchor, &p_sdu->size);
        }
        else
        {
            p_sdu->p_buf   = NULL;
            p_sdu->size    = 0;
        }

        DBG_SWDIAG(ISOAL, SDU_TX_GET, 0);

        // First fragment of the SDU
        first_fragment = true;
    }

    // Decrement number of remaining PDUs to transmit the SDU
    p_sdu->remaining_pdu--;

    // check if ISO packet not granted
    if (rejected)
    {
        // mark that SDU not granted
        if (p_sdu != NULL)
        {
            p_sdu->status = SDU_NOT_GRANTED;
        } // else nothing to do
    }
    // Sanity check on SDU buffer preparation, nothing to transmit
    else if ((p_sdu == NULL) || (p_sdu->status != SDU_OK))
    {
        // Mark header ready for transmission - of a null packet
        uint8_t llid = LLID_UNFRAMED_CONT;
        if (p_env->am0)
        {
            llid = LLID_UNFRAMED_END;
        }
        em_ble_txisobufsetup_pack(iso_buf_idx, /*mute*/ false, /*llid*/ llid, /*length*/ 0);
    }
    else
    {
        uint32_t em_buf_addr;
        uint8_t pkt_len = co_min(p_env->max_pdu, p_sdu->size - p_sdu->cursor);
        uint8_t llid = LLID_UNFRAMED_CONT;

        // Last fragment of SDU (or complete fragment)
        if ((((pkt_len == 0) && first_fragment) || ((pkt_len > 0) && (pkt_len == (p_sdu->size - p_sdu->cursor)))) || (p_env->am0))
        {
            llid = LLID_UNFRAMED_END;
        }

        // Exchange memory buffer address
        em_buf_addr = (EM_BASE_ADDR + ble_util_buf_iso_emptr_get(iso_buf_idx) + (EM_BLE_TXISODATABUF_INDEX << 1));

        if ((pkt_len > 0) && (p_sdu->p_buf != NULL))
        {
            // Transfer the data
            rwip_iso_data_transfer((void *) em_buf_addr, &(p_sdu->p_buf[p_sdu->cursor]), pkt_len);

            // update cursor value
            p_sdu->cursor += pkt_len;

            if (p_env->mic_present)
            {
                pkt_len += MIC_LEN;
            }
        }
        else
        {
            pkt_len = 0;
        }

        // Mark header ready for transmission
        em_ble_txisobufsetup_pack(iso_buf_idx, /*mute*/ false, /*llid*/ llid, /*length*/ pkt_len);
    }

    // If last fragment of the SDU
    if (p_sdu->remaining_pdu == 0)
    {
        DBG_SWDIAG(ISOAL, SDU_TX_DONE, 1);

        // If data path configured
        if ((p_env->p_dp != NULL) && (p_env->p_dp->cb_sdu_done != NULL))
        {
            // Indicate SDU transmission is completed
            p_env->p_dp->cb_sdu_done(p_env->conhdl, p_sdu->size, 0, p_sdu->p_buf, p_sdu->status);
        }

        DBG_SWDIAG(ISOAL, SDU_TX_DONE, 0);
    }
}

__STATIC void lld_isoal_rx_done_unframed(struct lld_isoal_env_tag *p_env, uint8_t iso_buf_idx, uint32_t ref_anchor, bool rejected)
{
    FUNC_PATCH_ENTRY_4_PARAM_NO_RETURN(LLD_ISOAL_PATCH_TYPE, LLD_ISOAL_RX_DONE_UNFRAMED_FUNC_BIT, p_env, iso_buf_idx, ref_anchor, rejected);

    struct lld_isoal_sdu_inf *p_sdu = &(p_env->sdu);;

    // Check if SDU is still used for this PDU
    if (p_sdu->remaining_pdu == 0)
    {
        // Initialize new SDU
        p_sdu->cursor = 0;
        p_sdu->remaining_pdu = p_env->sdu_pdu_nb;
        p_sdu->status = SDU_PARTIALLY_RECEIVED;

        // If an SDU has already been received earlier
        if (p_env->next_sdu_sync_ref != 0)
        {
            int16_t deviation = 0;
            uint32_t new_sdu_sync_ref = (ref_anchor + p_env->sync_ref_offset);

            // Compute the difference between the new reference and the expected reference
            if (CLK_BTS_LOWER_EQ(p_env->next_sdu_sync_ref, new_sdu_sync_ref))
            {
                deviation = CO_MOD((new_sdu_sync_ref - p_env->next_sdu_sync_ref), p_env->sdu_interval);
            }
            else
            {
                deviation = -CO_MOD((p_env->next_sdu_sync_ref - new_sdu_sync_ref), p_env->sdu_interval);
            }

            // Align SDU synchronization reference with the anchor point timestamp
            p_sdu->sdu_sync_ref = p_env->next_sdu_sync_ref + deviation;
        }
        else
        {
            // Initialize its synchronization reference for the first SDU
            p_sdu->sdu_sync_ref = (ref_anchor + p_env->sync_ref_offset);
        }

        // Compute next SDU synchronization reference
        p_env->next_sdu_sync_ref = p_sdu->sdu_sync_ref + p_env->sdu_interval + p_env->max_sdu_deviation / 2;

        DBG_SWDIAG(ISOAL, SDU_RX_GET, 1);

        // If data path configured
        if ((p_env->p_dp != NULL) && (p_env->p_dp->cb_sdu_get != NULL))
        {
            // Get SDU buffer from data path
            p_sdu->p_buf = p_env->p_dp->cb_sdu_get(p_env->conhdl, &ref_anchor, &p_sdu->size);
        }
        else
        {
            p_sdu->p_buf   = NULL;
            p_sdu->size    = 0;
        }

        DBG_SWDIAG(ISOAL, SDU_RX_GET, 0);
    }

    // Decrement number of remaining PDUs to complete this SDU
    p_sdu->remaining_pdu--;

    // check if ISO packet not granted
    if (rejected)
    {
        // mark that SDU not granted
        if (p_sdu != NULL)
        {
            p_sdu->status = SDU_NOT_GRANTED;
        } // else nothing to do
    }
    // Sanity check on SDU buffer preparation, nothing to receive
    else if ((p_sdu == NULL) || (p_sdu->status != SDU_PARTIALLY_RECEIVED))
    {
        // Nothing to do - ignore the packet
    }
    else
    {
        uint32_t em_buf_addr;
        uint8_t pkt_len;
        uint8_t pkt_status;
        uint8_t pkt_llid;

        // fill ISO buffer information
        em_ble_rxisobufsetup_unpack(iso_buf_idx, /*invl*/ &pkt_status, /*llid*/ &pkt_llid, /*length*/ &pkt_len);

        // If PDU is encrypted

        if (p_env->mic_present && (pkt_len > MIC_LEN))
        {
            pkt_len -= MIC_LEN;
        }

        em_buf_addr = (EM_BASE_ADDR + ble_util_buf_iso_emptr_get(iso_buf_idx) + (EM_BLE_RXISODATABUF_INDEX << 1));

        // SYNC error detected
        if (pkt_status == LLD_ISO_INVL_SYNC_ERR)
        {
            p_sdu->status = (p_sdu->cursor == 0) ? SDU_SYNC_ERR : SDU_PKT_LOST;
        }
        // CRC error detected
        else if (pkt_status == LLD_ISO_INVL_CRC_ERR)
        {
            p_sdu->status = SDU_CRC_ERR;
        }
        // Invalid LLID received
        else if ((pkt_llid != LLID_UNFRAMED_CONT) && (pkt_llid != LLID_UNFRAMED_END))
        {
            p_sdu->status = SDU_UNEXPECTED_FORMAT;
        }
        //
        else if ((pkt_len + p_sdu->cursor) > p_env->max_sdu)
        {
            p_sdu->status = SDU_SIZE_EXCEED;
        }
        else
        {
            // If data to copy, and buffer available to receive the data
            if ((pkt_len > 0) && (p_sdu->p_buf != NULL))
            {
                // Transfer the data
                rwip_iso_data_transfer(&(p_sdu->p_buf[p_sdu->cursor]), (void *) em_buf_addr, pkt_len);
            }

            // update cursor
            p_sdu->cursor += pkt_len;

            // Last fragment received, mark it done
            if (pkt_llid == LLID_UNFRAMED_END)
            {
                p_sdu->status = SDU_OK;
            }
        }
    }

    // If last fragment of the SDU
    if (p_sdu->remaining_pdu == 0)
    {
        DBG_SWDIAG(ISOAL, SDU_RX_DONE, 1);

        // If data path configured
        if ((p_env->p_dp != NULL) && (p_env->p_dp->cb_sdu_done != NULL))
        {
            // Indicate SDU reception is completed
            p_env->p_dp->cb_sdu_done(p_env->conhdl, (p_sdu->p_buf != NULL) * p_sdu->cursor, p_sdu->sdu_sync_ref, p_sdu->p_buf, p_sdu->status);
        }

        DBG_SWDIAG(ISOAL, SDU_RX_DONE, 0);
    }
}

// ----------------------------------------------------------------------------------- //
//                                   FRAMED MODE                                       //
// ----------------------------------------------------------------------------------- //

__STATIC void lld_isoal_tx_get_framed(struct lld_isoal_env_tag *p_env, uint8_t iso_buf_idx, uint32_t ref_anchor, bool rejected)
{
    FUNC_PATCH_ENTRY_4_PARAM_NO_RETURN(LLD_ISOAL_PATCH_TYPE, LLD_ISOAL_TX_GET_FRAMED_FUNC_BIT, p_env, iso_buf_idx, ref_anchor, rejected);

    uint8_t  pkt_cursor = 0;
    uint8_t  remain_len = p_env->max_pdu;

    // Fill PDU until full or no more SDU
    while (remain_len >= BLE_ISOAL_SEG_HEADER_SIZE)
    {
        // Point to current SDU
        struct lld_isoal_sdu_inf *p_sdu = &(p_env->sdu);
        uint32_t sdu_ref_time = 0;
        int32_t  time_offset = 0;
        uint8_t  seg_len = 0;

        // Check if there is still data to transmit from this SDU
        if (p_sdu->cursor == p_sdu->size)
        {
            // Check that a segment header with time offset information fits in the PDU
            if (remain_len < (BLE_ISOAL_SEG_HEADER_SIZE + BLE_ISOAL_TIME_OFFSET_SIZE))
                break;

            // Initialize new SDU
            p_sdu->cursor = 0;
            p_sdu->size = 0;

            DBG_SWDIAG(ISOAL, SDU_TX_GET, 1);

            // If data path configured
            if ((p_env->p_dp != NULL) && (p_env->p_dp->cb_sdu_get != NULL))
            {
                // Get SDU buffer from data path
                sdu_ref_time = ref_anchor;
                p_sdu->p_buf = p_env->p_dp->cb_sdu_get(p_env->conhdl, &sdu_ref_time, &p_sdu->size);
            }
            else
            {
                p_sdu->p_buf   = NULL;
            }

            DBG_SWDIAG(ISOAL, SDU_TX_GET, 0);
        }

        // If nothing to transmit
        if (p_sdu->p_buf == NULL)
            break;

        // If new SDU
        if (p_sdu->cursor == 0)
        {
            // Check that a segment header with time offset information fits in the PDU
            if (remain_len < (BLE_ISOAL_SEG_HEADER_SIZE + BLE_ISOAL_TIME_OFFSET_SIZE))
                break;

            // Compute time offset
            time_offset = CLK_BTS_SUB(ref_anchor, sdu_ref_time);

            // If the SDU is too early, postpone transmission for next event
            if (time_offset < 0)
                break;

            // Allocate time offset field size to the PDU
            remain_len -= BLE_ISOAL_TIME_OFFSET_SIZE;
        }

        // Allocate Segment header size to the PDU
        remain_len -= BLE_ISOAL_SEG_HEADER_SIZE;

        // Allocate SDU data portion to the PDU
        seg_len = co_min(remain_len, p_sdu->size - p_sdu->cursor);
        remain_len -= seg_len;

        // Prepare packet transmission
        if (!rejected)
        {
            uint16_t em_buf_addr = 0;
            uint16_t header    = 0;
            bool     start_seg = (p_sdu->cursor == 0);

            // Exchange memory buffer address
            em_buf_addr = (ble_util_buf_iso_emptr_get(iso_buf_idx) + (EM_BLE_TXISODATABUF_INDEX << 1));

            // Prepare segment header
            SETF(header, ISO_SEG_HDR_LENGTH, seg_len + (start_seg ? BLE_ISOAL_TIME_OFFSET_SIZE : 0));
            SETB(header, ISO_SEG_HDR_SC,     !start_seg);
            SETB(header, ISO_SEG_HDR_CMPLT, (seg_len + p_sdu->cursor) == p_sdu->size);

            // write segment header
            em_wr16p(em_buf_addr + pkt_cursor, co_htobs(header));
            pkt_cursor += BLE_ISOAL_SEG_HEADER_SIZE;

            // Add timestamp information
            if (p_sdu->cursor == 0)
            {
                em_wr24p(em_buf_addr + pkt_cursor, co_htob24((uint32_t)time_offset));
                pkt_cursor += BLE_ISOAL_TIME_OFFSET_SIZE;
            }

            if (seg_len)
            {
                // Transfer the data
                rwip_iso_data_transfer((uint8_t *)(EM_BASE_ADDR + em_buf_addr + pkt_cursor), p_sdu->p_buf + p_sdu->cursor, seg_len);
            }

            pkt_cursor += seg_len;

            // Update SDU information
            p_sdu->cursor += seg_len;
        }
        else
        {
            // PDU not transmitted, the entire SDU is considered flushed
            p_sdu->cursor = p_sdu->size;
        }

        // If last segment transmitted for this SDU
        if (p_sdu->cursor == p_sdu->size)
        {
            DBG_SWDIAG(ISOAL, SDU_TX_DONE, 1);

            // If data path configured
            if ((p_env->p_dp != NULL) && (p_env->p_dp->cb_sdu_done != NULL))
            {
                // Indicate SDU transmission is completed
                p_env->p_dp->cb_sdu_done(p_env->conhdl, p_sdu->size, 0, p_sdu->p_buf, p_sdu->status);
            }

            DBG_SWDIAG(ISOAL, SDU_TX_DONE, 0);

            // Invalidate SDU
            p_sdu->p_buf = NULL;
        }
    }

    // Fill ISO packet header information
    if (!rejected)
    {
        // Compute PDU length
        uint8_t pdu_len = pkt_cursor + ((pkt_cursor > 0) && p_env->mic_present) * MIC_LEN;

        // Mark header ready for transmission
        em_ble_txisobufsetup_pack(iso_buf_idx, /*mute*/ false, /*llid*/ LLID_FRAMED_SEG, /*length*/ pdu_len);
    }
}

__STATIC void lld_isoal_rx_done_framed(struct lld_isoal_env_tag *p_env, uint8_t iso_buf_idx, uint32_t ref_anchor, bool rejected)
{
    FUNC_PATCH_ENTRY_4_PARAM_NO_RETURN(LLD_ISOAL_PATCH_TYPE, LLD_ISOAL_RX_DONE_FRAMED_FUNC_BIT, p_env, iso_buf_idx, ref_anchor, rejected);

    // Point to current SDU
    struct lld_isoal_sdu_inf *p_sdu = &(p_env->sdu);
    uint8_t pkt_len = 0;
    uint8_t pkt_llid = LLID_FRAMED_SEG;
    uint8_t pkt_status = LLD_ISO_INVL_PKT_OK;

    // If a reception occurred
    if (!rejected)
    {
        // Fetch reception information
        em_ble_rxisobufsetup_unpack(iso_buf_idx, /*invl*/ &pkt_status, /*llid*/ &pkt_llid, /*length*/ &pkt_len);

        // If PDU is encrypted
        if (p_env->mic_present && (pkt_len > MIC_LEN))
        {
            // Remove MIC length (as it is counted in PDU length)
            pkt_len -= MIC_LEN;
        }
    }

    // If reception has failed
    if (rejected || (pkt_status == LLD_ISO_INVL_SYNC_ERR) || (pkt_status == LLD_ISO_INVL_CRC_ERR) || (pkt_llid != LLID_FRAMED_SEG))
    {
        // If an SDU is currently under reception
        if (p_sdu->p_buf != 0)
        {
            // Current SDU is considered completed
            DBG_SWDIAG(ISOAL, SDU_RX_DONE, 1);

            // If data path configured
            if ((p_env->p_dp != NULL) && (p_env->p_dp->cb_sdu_done != NULL))
            {
                // Indicate SDU reception is completed
                p_env->p_dp->cb_sdu_done(p_env->conhdl, (p_sdu->p_buf != NULL) * p_sdu->cursor, p_sdu->sdu_sync_ref, p_sdu->p_buf, SDU_PKT_LOST);
            }

            // Clear SDU
            p_sdu->size    = 0;
            p_sdu->cursor  = 0;
            p_sdu->p_buf   = NULL;

            DBG_SWDIAG(ISOAL, SDU_RX_DONE, 0);
        }

        // If an SDU has already been received earlier
        if (p_env->next_sdu_sync_ref != 0)
        {
            // If time of the next SDU is exceeded
            if (CLK_BTS_LOWER_EQ(p_env->next_sdu_sync_ref, (uint32_t)(ref_anchor + p_env->sync_ref_offset - p_env->sdu_interval)))
            {
                DBG_SWDIAG(ISOAL, SDU_RX_DONE, 1);

                // If data path configured
                if ((p_env->p_dp != NULL) && (p_env->p_dp->cb_sdu_done != NULL))
                {
                    // Indicate SDU lost
                    p_env->p_dp->cb_sdu_done(p_env->conhdl, 0, p_env->next_sdu_sync_ref, NULL, SDU_PKT_LOST);
                }

                DBG_SWDIAG(ISOAL, SDU_RX_DONE, 0);

                // Compute next SDU synchronization reference
                p_env->next_sdu_sync_ref = p_env->next_sdu_sync_ref + p_env->sdu_interval + p_env->max_sdu_deviation / 2;
            }
        }
    }
    else
    {
        uint16_t em_buf_addr = (ble_util_buf_iso_emptr_get(iso_buf_idx) + (EM_BLE_RXISODATABUF_INDEX << 1));
        uint8_t  pkt_cursor = 0;

        // Process every segment
        while ((pkt_len - pkt_cursor) >= BLE_ISOAL_SEG_HEADER_SIZE)
        {
            uint16_t seg_header;
            uint8_t seg_len;
            bool sdu_complete = false;
            uint8_t sdu_status = SDU_OK;

            // Read segment header
            seg_header = co_btohs(em_rd16p(em_buf_addr + pkt_cursor));
            pkt_cursor += BLE_ISOAL_SEG_HEADER_SIZE;
            seg_len = GETF(seg_header, ISO_SEG_HDR_LENGTH);

            // New SDU
            if (GETB(seg_header, ISO_SEG_HDR_SC) == 0)
            {
                uint32_t time_offset = 0;

                // If an SDU is currently under reception
                if (p_sdu->p_buf != NULL)
                {
                    DBG_SWDIAG(ISOAL, SDU_RX_DONE, 1);

                    // If data path configured
                    if ((p_env->p_dp != NULL) && (p_env->p_dp->cb_sdu_done != NULL))
                    {
                        // Indicate SDU reception is completed
                        p_env->p_dp->cb_sdu_done(p_env->conhdl, (p_sdu->p_buf != NULL) * p_sdu->cursor, p_sdu->sdu_sync_ref, p_sdu->p_buf, SDU_PARTIALLY_RECEIVED);
                    }

                    // Clear SDU
                    p_sdu->size    = 0;
                    p_sdu->cursor  = 0;
                    p_sdu->p_buf   = NULL;

                    DBG_SWDIAG(ISOAL, SDU_RX_DONE, 0);
                }

                // Check that time offset can be read
                if (((pkt_len - pkt_cursor) < BLE_ISOAL_TIME_OFFSET_SIZE) || (seg_len < BLE_ISOAL_TIME_OFFSET_SIZE))
                    break;

                // Read time offset
                time_offset = co_btoh24(em_rd16p(em_buf_addr + pkt_cursor));
                pkt_cursor += BLE_ISOAL_TIME_OFFSET_SIZE;
                seg_len    -= BLE_ISOAL_TIME_OFFSET_SIZE;

                // Compute SDU synchronization reference, for the new received SDU
                p_sdu->sdu_sync_ref = (uint32_t)(ref_anchor + p_env->sync_ref_offset - time_offset);

                // Compute next SDU synchronization reference
                p_env->next_sdu_sync_ref = p_sdu->sdu_sync_ref + p_env->sdu_interval + p_env->max_sdu_deviation / 2;

                DBG_SWDIAG(ISOAL, SDU_RX_GET, 1);

                // If data path configured
                if ((p_env->p_dp != NULL) && (p_env->p_dp->cb_sdu_get != NULL))
                {
                    // Get SDU buffer from data path
                    p_sdu->p_buf = p_env->p_dp->cb_sdu_get(p_env->conhdl, &ref_anchor, &p_sdu->size);
                }
                else
                {
                    p_sdu->p_buf   = NULL;
                    p_sdu->size    = 0;
                }

                DBG_SWDIAG(ISOAL, SDU_RX_GET, 0);
            }

            // If maximum SDU size is exceeded
            if ((p_sdu->cursor + seg_len) > p_sdu->size)
            {
                sdu_status = SDU_SIZE_EXCEED;
                sdu_complete = true;
            }
            // If segment length is bigger than remaining data in the PDU
            else if ((pkt_len - pkt_cursor) < seg_len)
            {
                sdu_status = SDU_UNEXPECTED_FORMAT;
                sdu_complete = true;
            }
            else
            {
                // Copy segment data
                if ((p_sdu->p_buf != NULL) && (seg_len > 0))
                {
                    // Transfer the data
                    rwip_iso_data_transfer(p_sdu->p_buf + p_sdu->cursor, (uint8_t *)(EM_BASE_ADDR + em_buf_addr + pkt_cursor), seg_len);
                }

                // Append segment length
                p_sdu->cursor += seg_len;

                // If last segment of the SDU
                if (GETB(seg_header, ISO_SEG_HDR_CMPLT))
                {
                    sdu_complete = true;
                }
            }

            // If SDU is complete
            if (sdu_complete)
            {
                DBG_SWDIAG(ISOAL, SDU_RX_DONE, 1);

                // If data path configured
                if ((p_env->p_dp != NULL) && (p_env->p_dp->cb_sdu_done != NULL))
                {
                    // Indicate SDU reception is completed
                    p_env->p_dp->cb_sdu_done(p_env->conhdl, (p_sdu->p_buf != NULL) * p_sdu->cursor, p_sdu->sdu_sync_ref, p_sdu->p_buf, sdu_status);
                }

                // Clear SDU
                p_sdu->size    = 0;
                p_sdu->cursor  = 0;
                p_sdu->p_buf   = NULL;

                DBG_SWDIAG(ISOAL, SDU_RX_DONE, 0);
            }

            // If the format is wrong, stop processing the PDU
            if ((sdu_status == SDU_UNEXPECTED_FORMAT) || (sdu_status == SDU_SIZE_EXCEED))
                break;

            // Shift cursor after segment
            pkt_cursor += seg_len;
        }
    }
}


/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void lld_isoal_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_ISOAL_PATCH_TYPE, LLD_ISOAL_INIT_FUNC_BIT, is_reset);

    if (!is_reset)
    {
        memset(&lld_isoal_env, 0, sizeof(lld_isoal_env));
    }
    else
    {
        // clean-up allocated environment
        uint8_t direction, idx;

        for (direction = 0 ; direction < ISO_SEL_MAX ; direction++)
        {
            for (idx = 0 ; idx < BLE_ACTIVITY_MAX ; idx++)
            {
                if (lld_isoal_env[direction][idx] != NULL)
                {
                    ke_free(lld_isoal_env[direction][idx]);
                    lld_isoal_env[direction][idx] = NULL;
                }
            }
        }
    }
}

uint8_t lld_isoal_start(uint8_t act_id, uint8_t direction, uint16_t conhdl,
                        uint32_t sdu_interval, uint32_t trans_latency, int32_t sync_ref_offset, uint16_t max_sdu, uint32_t iso_interval,
                        uint8_t bn, uint8_t max_pdu, uint8_t framing, bool mic_present, bool am0)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_HAVE_RETURN(LLD_ISOAL_PATCH_TYPE, LLD_ISOAL_START_FUNC_BIT, uint8_t, 13, act_id, direction, conhdl, sdu_interval, trans_latency, sync_ref_offset,
                                            max_sdu, iso_interval, bn, max_pdu, framing, mic_present, am0);
    uint8_t status = CO_ERROR_NO_ERROR;
    uint16_t env_size;
    struct lld_isoal_env_tag *p_env;
    ASSERT_ERR(direction < ISO_SEL_MAX);
    ASSERT_INFO(lld_isoal_env[direction][act_id] == NULL, direction, act_id);

    // Allocate memory for ISOAL environment structure
    env_size  = CO_ALIGN4_HI(sizeof(struct lld_isoal_env_tag));
    p_env = (struct lld_isoal_env_tag *) ke_malloc_system(env_size, KE_MEM_ENV);

    if (p_env)
    {

        p_env->p_dp            = NULL;
        p_env->sdu_interval    = sdu_interval;
        p_env->trans_latency   = trans_latency;
        p_env->sync_ref_offset = sync_ref_offset;
        p_env->next_sdu_sync_ref = 0;
        p_env->max_sdu_deviation = sdu_interval * (BLE_MAX_DRIFT_SLEEP + rwip_max_drift_get()) / 1000000;
        p_env->max_sdu         = max_sdu;
        p_env->max_pdu         = max_pdu;
        p_env->conhdl          = conhdl;
        p_env->mic_present     = mic_present;
        p_env->framing         = framing;
        p_env->am0             = am0;
        p_env->sdu.remaining_pdu = 0;
        p_env->sdu.cursor        = 0;
        p_env->sdu.size          = 0;
        p_env->sdu.sdu_sync_ref  = 0;
        p_env->sdu.status        = 0;
        p_env->sdu.p_buf         = NULL;

        // Number of PDUs to transfer an SDU in unframed mode
        p_env->sdu_pdu_nb           = (sdu_interval * bn) / iso_interval;

        // Link ISOAL environment structure
        lld_isoal_env[direction][act_id] = p_env;
    }
    else
    {
        status = CO_ERROR_MEMORY_CAPA_EXCEED;
        ASSERT_ERR_FORCE(0);
    }

    return status;
}

void lld_isoal_stop(uint8_t act_id, uint8_t reason)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_ISOAL_PATCH_TYPE, LLD_ISOAL_STOP_FUNC_BIT, act_id, reason);

    uint8_t direction;
    for (direction = 0 ; direction < ISO_SEL_MAX ; direction++)
    {
        struct lld_isoal_env_tag *p_env = lld_isoal_env[direction][act_id];

        if (p_env != NULL)
        {
            // Stop data-path
            if ((p_env->p_dp != NULL) && (p_env->p_dp->cb_stop != NULL))
            {
                p_env->p_dp->cb_stop(p_env->conhdl, reason);
            }

            // Free environment information
            ke_free(p_env);
            lld_isoal_env[direction][act_id] = NULL;
        }
    }
}

void lld_isoal_tx_get(uint8_t act_id, uint8_t iso_buf_idx, uint32_t ref_anchor, bool rejected)
{
    FUNC_PATCH_ENTRY_4_PARAM_NO_RETURN(LLD_ISOAL_PATCH_TYPE, LLD_ISOAL_TX_GET_FUNC_BIT, act_id, iso_buf_idx, ref_anchor, rejected);

    struct lld_isoal_env_tag *p_env;
    p_env = lld_isoal_env[ISO_SEL_TX][act_id];

    DBG_SWDIAG(ISOAL, TX_GET, 1);

    if (p_env != NULL)
    {
        if (p_env->framing == ISO_UNFRAMED_MODE)
        {
            lld_isoal_tx_get_unframed(p_env, iso_buf_idx, ref_anchor, rejected);
        }
        else
        {
            lld_isoal_tx_get_framed(p_env, iso_buf_idx, ref_anchor, rejected);
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(ISOAL, TX_GET, 0);
}

void lld_isoal_rx_done(uint8_t act_id, uint8_t iso_buf_idx, uint32_t ref_anchor, bool rejected)
{
    FUNC_PATCH_ENTRY_4_PARAM_NO_RETURN(LLD_ISOAL_PATCH_TYPE, LLD_ISOAL_RX_DONE_FUNC_BIT, act_id, iso_buf_idx, ref_anchor, rejected);

    struct lld_isoal_env_tag *p_env = lld_isoal_env[ISO_SEL_RX][act_id];

    DBG_SWDIAG(ISOAL, RX_DONE, 1);

    if (p_env != NULL)
    {
        if (p_env->framing == ISO_UNFRAMED_MODE)
        {
            lld_isoal_rx_done_unframed(p_env, iso_buf_idx, ref_anchor, rejected);
        }
        else
        {
            lld_isoal_rx_done_framed(p_env, iso_buf_idx, ref_anchor, rejected);
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    // Mark buffer ready for new reception
    if (!rejected)
    {
        em_ble_rxisobufsetup_pack(iso_buf_idx, /*invl*/ LLD_ISO_INVL_SYNC_ERR, /*llid*/ LLID_UNFRAMED_END, /*length*/ 0);
    }

    DBG_SWDIAG(ISOAL, RX_DONE, 0);
}

uint8_t lld_isoal_datapath_set(uint8_t act_id, uint8_t direction, const struct data_path_itf *p_dp)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_ISOAL_PATCH_TYPE, LLD_ISOAL_DATAPATH_SET_FUNC_BIT, uint8_t, act_id, direction, p_dp);

    uint8_t status = CO_ERROR_NO_ERROR;
    struct lld_isoal_env_tag *p_env;
    ASSERT_ERR(direction < ISO_SEL_MAX);
    ASSERT_ERR(act_id < BLE_ACTIVITY_MAX);

    GLOBAL_INT_DISABLE();
    p_env = lld_isoal_env[direction][act_id];
    if (p_env == NULL)
    {
        status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    }
    else if (p_env->p_dp != NULL)
    {
        status = CO_ERROR_COMMAND_DISALLOWED;
    }
    else
    {
        if (p_dp->cb_start != NULL)
        {
            // Start data path
            status = p_dp->cb_start(p_env->conhdl, p_env->sdu_interval, p_env->trans_latency, p_env->max_sdu);

            if (status == CO_ERROR_NO_ERROR)
            {
                p_env->p_dp = p_dp;
            }
        }
    }
    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_isoal_datapath_remove(uint8_t act_id, uint8_t direction)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_ISOAL_PATCH_TYPE, LLD_ISOAL_DATAPATH_REMOVE_FUNC_BIT, uint8_t, act_id, direction);

    uint8_t status = CO_ERROR_NO_ERROR;
    struct lld_isoal_env_tag *p_env;
    ASSERT_ERR(direction < ISO_SEL_MAX);
    ASSERT_ERR(act_id < BLE_ACTIVITY_MAX);

    GLOBAL_INT_DISABLE();
    p_env = lld_isoal_env[direction][act_id];
    if (p_env != NULL)
    {
        if (p_env->p_dp != NULL)
        {
            if (p_env->p_dp->cb_stop != NULL)
            {
                // Stop data path
                p_env->p_dp->cb_stop(p_env->conhdl, CO_ERROR_OPERATION_CANCELED_BY_HOST);

                // Clear SDU pointers
                p_env->sdu.p_buf = NULL;
            }
            p_env->p_dp = NULL;
        }
    }
    else
    {
        status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    }
    GLOBAL_INT_RESTORE();

    return (status);
}

#endif //(BLE_ISO_PRESENT)

///@} LLDISOAL
