/**
 ****************************************************************************************
 *
 * @file isoohci_int.h
 *
 * @brief Declaration of the functions used for Isochronous Payload over HCI Transport layer Internals
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 ****************************************************************************************
 */

#ifndef ISOOHCI_INT_H_
#define ISOOHCI_INT_H_

/**
 ****************************************************************************************
 * @addtogroup ISOOHCI_INT
 * @ingroup ISOOHCI
 * @brief Isochronous Payload over HCI Transport Layer Internals
 *
 * This module implements the primitives used for Isochronous Payload over HCI Transport layer
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_ISOOHCI)

#include <stdint.h>
#include <stdbool.h>
#include "co_math.h"

#include "arch.h"
#include "isoohci.h"

#include "co_list.h"
#include "co_utils.h"

/*
 * MACROS
 ****************************************************************************************
 */
/*
 * DEFINES
 ****************************************************************************************
 */

/// Invalid buffer index
#define ISOOHCI_INVALID_BUF_IDX    (0xFF)


/*
 * ENUMERATIONS
 ****************************************************************************************
 */


/*
 * PROTOCOL STRUCTURES
 ****************************************************************************************
 */

/// SDU Buffer structure
typedef struct isoohci_buffer
{
    /// used to put buffer in a queue
    co_list_hdr_t   hdr;
    /// Time_Stamp
    uint32_t        time_stamp;
    /// Packet Sequence Number
    uint16_t        pkt_seq_nb;
    /// length of the ISO SDU (in bytes)
    uint16_t        sdu_length;
    /// Connection handle
    uint16_t        conhdl;
    /// Buffer index
    uint8_t         buf_idx;
    /// Reception status (@see enum hci_iso_pkt_stat_flag)
    uint8_t         status;
    /// Number of HCI ISO data packets in the SDU (used only for input direction - from Host to Controller)
    uint8_t         hci_iso_data_pkt_nb;
    /// Area reserved for HCI header (used only for output direction - from Controller to Host)
    uint8_t         hci_iso_hdr[HCI_TRANSPORT_HDR_LEN + HCI_ISO_HDR_LEN];
    /**
     * ISO_Data_Load header
     *     - Output: always present, timestamp always included
     *     - Input: only in first fragment of an SDU, timestamp may or may not be present
     */
    uint8_t         hci_iso_data_load_hdr[HCI_ISO_DATA_LOAD_HDR_LEN_MAX];
    /// SDU
    uint8_t         sdu[__ARRAY_EMPTY];
} isoohci_buffer_t;


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Information about pending SDUs in HCI ISO input data path
struct isoohci_in_sdu_info_tag
{
    /// Connection handle
    uint16_t conhdl;

    /// Number of HCI ISO data packets in the SDU
    uint8_t hci_iso_data_pkt_nb;
};

/// Timing information about last transmitted SDU
struct isoohci_in_last_sdu_tx_sync
{
    /// The time stamp, in microseconds, of the reference anchor point of a transmitted SDU (Range 0x00000000-0xFFFFFFFF)
    uint32_t time_stamp;
    /// The time offset, in microseconds, that is associated with a transmitted SDU (Range 0x000000-0xFFFFFF)
    uint32_t time_offset;
    /// The packet sequence number of an SDU (Range 0x0000-0xFFFF)
    uint16_t pkt_seq_nb;
};

/// Isochronous Buffer management structure
struct isoohci_in_info
{
    /// Queue of buffers received from HCI
    co_list_t   in_queue;
    /// Timing information about last transmitted SDU
    struct isoohci_in_last_sdu_tx_sync last_sdu;
    /// Buffer of the current SDU in reception
    struct isoohci_buffer *current_buf;
    /// Buffer in use by ISOAL
    struct isoohci_buffer *buf_in_use;
    /// Max SDU Size (in bytes)
    uint16_t max_sdu_size;
    /// Length of SDU data received in current SDU (in bytes)
    uint16_t sdu_rx_len;
};

/// Isochronous Buffer management structure
struct isoohci_out_info
{
    /// SDU packet counter
    uint32_t    sdu_cnt;
    /// Buffer queue
    co_list_t   buffer_queue;
    /// Buffer queue (in use by transport)
    co_list_t   rx_queue;
    /// Maximum SDU length (in bytes)
    uint16_t max_sdu;
    /// Total number of buffers
    uint8_t     nb_buf;
    /// Number of buffer in use (received from transport, in transmission to Host)
    uint8_t     nb_buf_in_use;
    /// Buffer index
    uint8_t     buf_idx;
    /// Data path to be removed
    bool remove;
};

/// Isochronous Payload over HCI TL environment definition
struct isoohci_env_tag
{
    /// Information about SDUs pending in HCI ISO input paths
    struct isoohci_in_sdu_info_tag in_sdu_info[BLE_HCI_ISO_IN_SDU_BUF_NB];
    /// Host to controller information
    struct isoohci_in_info  *p_in[BLE_ACTIVITY_MAX];
    /// Pointer array of available ISO channel path
    struct isoohci_out_info *p_out[BLE_ACTIVITY_MAX];
    /// list of TX buffer waiting to be sent
    struct co_list           hci_wait_tx_queue;
    /// list of TX buffer in HCI transmission queue
    struct co_list           hci_in_tx_queue;
    /// Buffer used for trash reception over transport layer
    uint8_t *trash_buffer;
    /// Position of current SDU in HCI ISO input paths
    uint8_t in_sdu_idx;
    /// Position of SDU to be sent in HCI ISO input paths
    uint8_t in_sdu_send_idx;
};


/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/// Isochronous Payload over HCI TL environment
extern struct isoohci_env_tag isoohci_env;

/// Host To Controller interface
extern const struct data_path_itf isoohci_in_itf;
/// Controller to Host interface
extern const struct data_path_itf isoohci_out_itf;


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Defer Processing of Isochronous data into a kernel event. Host to controller
 ****************************************************************************************
 */
void isoohci_in_defer_handler(void);
/**
 ****************************************************************************************
 * @brief Defer Processing of Isochronous data into a kernel event. Controller to host
 ****************************************************************************************
 */
void isoohci_out_defer_handler(void);

#endif //(BLE_ISOOHCI)

/// @} ISOOHCI

#endif // ISOOHCI_INT_H_
