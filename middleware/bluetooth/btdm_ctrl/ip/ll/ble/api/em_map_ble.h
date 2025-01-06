/**
 ****************************************************************************************
 *
 * @file em_map_ble.h
 *
 * @brief Mapping of BLE section in exchange memory
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

#ifndef EM_MAP_BLE_H_
#define EM_MAP_BLE_H_

/**
 ****************************************************************************************
 * @addtogroup EM Exchange Memory
 * @ingroup CONTROLLER
 * @brief Mapping of BLE section in exchange memory
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#include <stdint.h>                      // integer
#include "co_llcp.h"
#include "co_math.h"                     // For use of CO_ALIGN4_HI
#include "_reg_em_ble_cs.h"              // EM Control Structure
#include "_reg_em_ble_tx_desc.h"         // BLE EM TX descriptors
#include "_reg_em_ble_rx_desc.h"         // BLE EM RX descriptors
#include "_reg_em_ble_rx_cte_desc.h"     // BLE EM RX CTE descriptors
#include "_reg_em_ble_wpal.h"            // BLE EM White and periodic list
#include "_reg_em_ble_ral.h"             // BLE EM resolving list

#include "rom_config.h"

#if (BLE_ISO_PRESENT)
    #include "_reg_em_ble_tx_iso_desc.h"     // BLE EM TX ISO descriptors
    #include "_reg_em_ble_rx_iso_desc.h"     // BLE EM RX ISO descriptors
    #include "_reg_em_ble_tx_iso_buf.h"   // BLE EM TX Buffer description
    #include "_reg_em_ble_rx_iso_buf.h"   // BLE EM RX Buffer description
#endif // (BLE_ISO_PRESENT)

/*
 * CONTROL STRUCTURES
 ****************************************************************************************
 */

/**
 * Control Structures area definition
 *
 * - LINK   (N)
 * - SCAN/INIT
 */
//#define EM_BLE_CS_OFFSET          (EM_BLE_OFFSET)
#define EM_BLE_CS_OFFSET          (rom_config_get_em_offset(ROM_EM_BLE_CS_OFFSET))

//#define EM_BLE_CS_NB              (BLE_ACTIVITY_MAX + (BLE_OBSERVER*3) + BLE_CENTRAL)
#define EM_BLE_CS_NB_MAX              (BLE_ACTIVITY_MAX + (BLE_OBSERVER*3) + BLE_CENTRAL)
#define EM_BLE_CS_NB              ((EM_BLE_WPAL_OFFSET - EM_BLE_CS_OFFSET) / REG_EM_BLE_CS_SIZE)
#define EM_BLE_CS_END             (EM_BLE_CS_OFFSET + EM_BLE_CS_NB * REG_EM_BLE_CS_SIZE)


#if ((REG_EM_BLE_CS_SIZE % 4) != 0)
    #error "CS Pointers must be 32 bits aligned"
#endif // ((REG_EM_BLE_CS_SIZE % 4) != 0)

/**
 * Control Structures indexes definition
 */
#define EM_BLE_CS_ACT_ID_TO_INDEX(act_id)   (act_id)
#define EM_BLE_CS_IDX_TO_ACT_ID(cs_idx)     (cs_idx)

#if (BLE_OBSERVER)
    #define EM_BLE_CS_EXT_SCAN_IDX2             (EM_BLE_CS_NB - 1)
    #define EM_BLE_CS_AUX_SCAN_IDX1             (EM_BLE_CS_NB - 2)
    #define EM_BLE_CS_AUX_SCAN_IDX2             (EM_BLE_CS_NB - 3)
#endif // (BLE_OBSERVER)
#if (BLE_CENTRAL)
    #define EM_BLE_CS_EXT_INIT_IDX2             (EM_BLE_CS_NB - 4)
#endif // (BLE_CENTRAL)

#define EM_BLE_CS_INDEX_MAX                 (EM_BLE_CS_NB)

/**
 * Frame Format
 * Bluetooth state and device activity used for frequency selection and TDD frame format. See section 3.5 for further details
 * Note this field is automatically updated by the RW-BT Core when changing from Page to Master Page Response mode.
 *
 *  - 0000x: Do not use
 *  - 00010: Master Connect
 *  - 00011: Slave Connect
 *  - 00100: Low Duty Cycle Advertiser
 *  - 00101: High Duty Cycle Advertiser
 *  - 00110: Extended Advertiser
 *  - 00111: Do not use
 *  - 01000: Passive Scanner
 *  - 01001: Active Scanner
 *  - 01010: Extended Passive Scanner
 *  - 01011: Extended Active Scanner
 *  - 0110x: Do not use
 *  - 01110: Initiator
 *  - 01111: Extended Initiator
 *  - 10xxx: Do not use
 *  - 110xx: Do not use
 *  - 11100: Tx Test Mode
 *  - 11101: Rx Test Mode
 *  - 11110: Tx / Rx Test Mode
 *  - 11111: Do not use
 */
enum em_ble_cd_fmt
{
    EM_BLE_CS_FMT_MST_CONNECT      = (0x02),
    EM_BLE_CS_FMT_SLV_CONNECT      = (0x03),
    EM_BLE_CS_FMT_LDC_ADV          = (0x04),
    EM_BLE_CS_FMT_HDC_ADV          = (0x05),
    EM_BLE_CS_FMT_EXT_ADV          = (0x06),
    EM_BLE_CS_FMT_PASSIVE_SCAN     = (0x08),
    EM_BLE_CS_FMT_ACTIVE_SCAN      = (0x09),
    EM_BLE_CS_FMT_EXT_PASSIVE_SCAN = (0x0A),
    EM_BLE_CS_FMT_EXT_ACTIVE_SCAN  = (0x0B),
    EM_BLE_CS_FMT_CHAN_SCAN        = (0x0C),
    EM_BLE_CS_FMT_INITIATOR        = (0x0E),
    EM_BLE_CS_FMT_EXT_INITIATOR    = (0x0F),
    EM_BLE_CS_FMT_TX_TEST          = (0x1C),
    EM_BLE_CS_FMT_RX_TEST          = (0x1D),
    EM_BLE_CS_FMT_TXRX_TEXT        = (0x1E),
};

/// Maximum value of RXWINSZ field (16383)
#define EM_BLE_CS_RXWINSZ_MAX        ((1 << 14)-1)

/// Maximum value of MAXFRMTIME field (65535)
#define EM_BLE_CS_MAXEVTIME_MAX      ((1 << 16)-1)

/// MIC Mode
enum ENC_MIC
{
    /// AES-CCM with MIC
    ENC_MIC_PRESENT = 0,
    /// AES-CCM MIC-less
    ENC_MIC_LESS    = 1,
};

/// Encryption Mode
enum ENC_MODE
{
    /// AES-CCM nonce use Packet/Payload counter
    ENC_MODE_PKT_PLD_CNT = 0,
    /// AES-CCM nonce use Event counter
    ENC_MODE_EVT_CNT = 1,
};


/*
 * WHITE LIST
 ****************************************************************************************
 */

/// White list area definition
//#define EM_BLE_WPAL_OFFSET         CO_ALIGN4_HI(EM_BLE_CS_END)
#define EM_BLE_WPAL_OFFSET          (rom_config_get_em_offset(ROM_EM_BLE_WPAL_OFFSET))

#define EM_BLE_WPAL_SIZE           (BLE_WHITELIST_MAX_CONFIG * REG_EM_BLE_WPAL_SIZE)
#define EM_BLE_WPAL_END            (EM_BLE_WPAL_OFFSET + EM_BLE_WPAL_SIZE)
#define BLE_WHITELIST_MAX            ((EM_BLE_RAL_OFFSET - EM_BLE_WPAL_OFFSET) / REG_EM_BLE_WPAL_SIZE)

#if ((REG_EM_BLE_WPAL_SIZE % 4) != 0)
    #error "WPAL Pointers must be 32 bits aligned"
#endif // ((REG_EM_BLE_WPAL_SIZE % 4) != 0)
/*
 * RESOLVING LIST
 ****************************************************************************************
 */

/// Resolving list area definition
//#define EM_BLE_RAL_OFFSET        CO_ALIGN4_HI(EM_BLE_WPAL_END)
#define EM_BLE_RAL_OFFSET          (rom_config_get_em_offset(ROM_EM_BLE_RAL_OFFSET))

#define EM_BLE_RAL_SIZE          (BLE_RESOL_ADDR_LIST_MAX_CONFIG * REG_EM_BLE_RAL_SIZE)
#define EM_BLE_RAL_END           (EM_BLE_RAL_OFFSET + EM_BLE_RAL_SIZE)
#define BLE_RAL_MAX              ((EM_BLE_RX_DESC_OFFSET - EM_BLE_RAL_OFFSET) / REG_EM_BLE_RAL_SIZE)
#define BLE_RESOL_ADDR_LIST_MAX     (BLE_RAL_MAX)

#if ((REG_EM_BLE_RAL_SIZE % 4) != 0)
    #error "RAL Pointers must be 32 bits aligned"
#endif // ((REG_EM_BLE_RAL_SIZE % 4) != 0)
/*
 * RX DESCRIPTORS
 ****************************************************************************************
 */

/// RX Descriptors area definition
//#define EM_BLE_RX_DESC_OFFSET      CO_ALIGN4_HI(EM_BLE_RAL_END)
#define EM_BLE_RX_DESC_OFFSET          (rom_config_get_em_offset(ROM_EM_BLE_RX_DESC_OFFSET))

#define EM_BLE_RX_DESC_NB          (BLE_RX_DESC_NB)
#define EM_BLE_RX_DESC_END         (EM_BLE_RX_DESC_OFFSET + EM_BLE_RX_DESC_NB * REG_EM_BLE_RX_DESC_SIZE)
#define EM_NUM_BLE_RX_DESC         ((EM_BLE_TX_DESC_OFFSET - EM_BLE_RX_DESC_OFFSET) / REG_EM_BLE_RX_DESC_SIZE)

#if ((REG_EM_BLE_RX_DESC_SIZE % 4) != 0)
    #error "TX Descriptors must be 32 bits aligned"
#endif // ((REG_EM_BLE_TX_DESC_SIZE % 4) != 0)
/*
 * TX DESCRIPTORS
 ****************************************************************************************
 */

/**
 * TX Descriptors area definition
 *
 * - N per connection
 * - 1 per advertising data buffer
 */
//#define EM_BLE_TX_DESC_OFFSET           CO_ALIGN4_HI(EM_BLE_RX_DESC_END)
#define EM_BLE_TX_DESC_OFFSET          (rom_config_get_em_offset(ROM_EM_BLE_TX_DESC_OFFSET))

#define EM_BLE_TX_DESC_NB               (BLE_TX_DESC_NB)
#define EM_BLE_TX_DESC_END              (EM_BLE_TX_DESC_OFFSET + EM_BLE_TX_DESC_NB * REG_EM_BLE_TX_DESC_SIZE)

#define EM_BLE_TXDESC_INDEX(act_id, idx)      (BLE_NB_TX_DESC_PER_ACT * act_id + idx)

#if ((REG_EM_BLE_TX_DESC_SIZE % 4) != 0)
    #error "TX Descriptors must be 32 bits aligned"
#endif // ((REG_EM_BLE_TX_DESC_SIZE % 4) != 0)
/*
 * LLCP TX BUFFERS
 ****************************************************************************************
 */

/// LLCP TX buffers area definition
//#define EM_BLE_LLCPTXBUF_OFFSET    CO_ALIGN4_HI(EM_BLE_TX_DESC_END)
#define EM_BLE_LLCPTXBUF_OFFSET          (rom_config_get_em_offset(ROM_EM_BLE_LLCPTXBUF_OFFSET))

#define EM_BLE_LLCPTXBUF_NB        (2*BLE_ACTIVITY_MAX)
#define EM_BLE_LLCPTXBUF_SIZE      CO_ALIGN4_HI(LL_PDU_LENGTH_MAX) // ensure that LLCP buffer is 32bits aligned
#define EM_BLE_LLCPTXBUF_END       (EM_BLE_LLCPTXBUF_OFFSET + EM_BLE_LLCPTXBUF_NB * EM_BLE_LLCPTXBUF_SIZE)
#define EM_NUM_BLE_LLCPTXBUF ((EM_BLE_ADVEXTHDRTXBUF_OFFSET - EM_BLE_LLCPTXBUF_OFFSET) / EM_BLE_LLCPTXBUF_SIZE)


/*
 * ADV EXTENDED HEADERS TX BUFFER
 ****************************************************************************************
 */

/// Advertising TX buffer area definition
//#define EM_BLE_ADVEXTHDRTXBUF_OFFSET    CO_ALIGN4_HI(EM_BLE_LLCPTXBUF_END)
#define EM_BLE_ADVEXTHDRTXBUF_OFFSET          (rom_config_get_em_offset(ROM_EM_BLE_ADVEXTHDRTXBUF_OFFSET))

#define EM_BLE_ADVEXTHDRTXBUF_NB        (BLE_ACTIVITY_MAX)
#define EM_BLE_ADVEXTHDRTXBUF_SIZE      (60 + 8*(BLE_ADV_FRAG_NB_TX - 1))
#define EM_BLE_ADVEXTHDRTXBUF_END       (EM_BLE_ADVEXTHDRTXBUF_OFFSET + EM_BLE_ADVEXTHDRTXBUF_NB * EM_BLE_ADVEXTHDRTXBUF_SIZE)

#define EM_BLE_ADVEXTHDRTXBUF_OFF(act_id)  (EM_BLE_ADVEXTHDRTXBUF_OFFSET + act_id * EM_BLE_ADVEXTHDRTXBUF_SIZE)

/*
  * ADVERTISING DATA TX BUFFERS
  ****************************************************************************************
  */

/// Advertising data TX buffers area definition
//#define EM_BLE_ADVDATATXBUF_OFFSET    CO_ALIGN4_HI(EM_BLE_ADVEXTHDRTXBUF_END)
#define EM_BLE_ADVDATATXBUF_OFFSET          (rom_config_get_em_offset(ROM_EM_BLE_ADVDATATXBUF_OFFSET))

#define EM_BLE_ADVDATATXBUF_NB        (BLE_ADV_BUF_NB_TX)
#define EM_BLE_ADVDATATXBUF_SIZE      (BLE_CFG_MAX_ADV_DATA_LEN)
#define EM_BLE_ADVDATATXBUF_END       (EM_BLE_ADVDATATXBUF_OFFSET + EM_BLE_ADVDATATXBUF_NB * EM_BLE_ADVDATATXBUF_SIZE)
#define EM_NUM_BLE_ADVDATATXBUF       ((EM_BLE_AUXCONNECTREQTXBUF_OFFSET - EM_BLE_ADVDATATXBUF_OFFSET) / EM_BLE_ADVDATATXBUF_SIZE)

/*
 * AUX_CONNECT_REQ TX BUFFER
 ****************************************************************************************
 */

/// Aux Connect Req TX buffer area definition
//#define EM_BLE_AUXCONNECTREQTXBUF_OFFSET    CO_ALIGN4_HI(EM_BLE_ADVDATATXBUF_END)
#define EM_BLE_AUXCONNECTREQTXBUF_OFFSET          (rom_config_get_em_offset(ROM_EM_BLE_AUXCONNECTREQTXBUF_OFFSET))

#define EM_BLE_AUXCONNECTREQTXBUF_NB        (1)
#define EM_BLE_AUXCONNECTREQTXBUF_SIZE      (0x66) // 3 * PDU_CON_REQ_LEN 
#define EM_BLE_AUXCONNECTREQTXBUF_END       (EM_BLE_AUXCONNECTREQTXBUF_OFFSET + EM_BLE_AUXCONNECTREQTXBUF_NB * EM_BLE_AUXCONNECTREQTXBUF_SIZE)


/*
 * ACL RX BUFFERS
 ****************************************************************************************
 */

/// Data RX buffers area definition
//#define EM_BLE_DATARXBUF_OFFSET    CO_ALIGN4_HI(EM_BLE_AUXCONNECTREQTXBUF_END)
#define EM_BLE_DATARXBUF_OFFSET          (rom_config_get_em_offset(ROM_EM_BLE_DATARXBUF_OFFSET))

#define EM_BLE_DATARXBUF_NB        (BLE_DATA_BUF_NB_RX)
#define EM_BLE_DATARXBUF_SIZE      (BLE_DATA_BUF_SIZE_RX)
#define EM_BLE_DATARXBUF_END       (EM_BLE_DATARXBUF_OFFSET + EM_BLE_DATARXBUF_NB * EM_BLE_DATARXBUF_SIZE)
#define EM_NUM_BLE_DATARXBUF       ((EM_BLE_ACLTXBUF_OFFSET - EM_BLE_DATARXBUF_OFFSET)/EM_BLE_DATARXBUF_SIZE)
/*
 * ACL TX BUFFERS
 ****************************************************************************************
 */

/// ACL TX buffers area definition
//#define EM_BLE_ACLTXBUF_OFFSET    CO_ALIGN4_HI(EM_BLE_DATARXBUF_END)
#define EM_BLE_ACLTXBUF_OFFSET          (rom_config_get_em_offset(ROM_EM_BLE_ACLTXBUF_OFFSET))

#define EM_BLE_ACLTXBUF_NB        (BLE_ACL_BUF_NB_TX)
#define EM_BLE_ACLTXBUF_SIZE      (256)
#define EM_BLE_ACLTXBUF_END       (EM_BLE_ACLTXBUF_OFFSET + EM_BLE_ACLTXBUF_NB * EM_BLE_ACLTXBUF_SIZE)
#define EM_NUM_BLE_ACLTXBUF       ((EM_BLE_ISO_HOP_SEQ_OFFSET - EM_BLE_ACLTXBUF_OFFSET)/EM_BLE_ACLTXBUF_SIZE)


#if (BLE_ISO_PRESENT)
    /*
    * ISO HOP SEQUENCE
    ****************************************************************************************
    */
    #if (BLE_CIS | BLE_BIS)
        /// Hop Sequence Exchange memory area
        //        #define EM_BLE_ISO_HOP_SEQ_OFFSET               CO_ALIGN4_HI(EM_BLE_ACLTXBUF_END)
        #define EM_BLE_ISO_HOP_SEQ_OFFSET          (rom_config_get_em_offset(ROM_EM_BLE_ISO_HOP_SEQ_OFFSET))

        #define EM_BLE_ISO_HOP_SEQ_NB                   (BLE_ISO_HOP_SEQ_NB)
        #define EM_BLE_ISO_HOP_SEQ_SIZE                 CO_ALIGN4_HI(BLE_ISO_HOP_SEQ_SIZE)
        #define EM_BLE_ISO_HOP_SEQ_END                  (EM_BLE_ISO_HOP_SEQ_OFFSET + (EM_BLE_ISO_HOP_SEQ_NB * EM_BLE_ISO_HOP_SEQ_SIZE))

        #define EM_NUM_BLE_ISO_HOP_SEQ                   ((EM_BLE_ISO_DESC_OFFSET - EM_BLE_ISO_HOP_SEQ_OFFSET)/EM_BLE_ISO_HOP_SEQ_SIZE)

    #else // !(BLE_CIS | BLE_BIS)
        #define EM_BLE_ISO_HOP_SEQ_END                  (EM_BLE_ACLTXBUF_END)
    #endif // (BLE_CIS | BLE_BIS)
    #if ((EM_BLE_ISO_HOP_SEQ_SIZE % 4) != 0)
        #error "HOP Seq PTR must be 32 bits aligned"
    #endif // ((EM_BLE_ISO_HOP_SEQ_SIZE % 4) != 0)

    /*
    * ISO TX/RX Descriptors
    ****************************************************************************************
    */
    /**
    * ISO Descriptors area definition
    * Pool to be used for TX and RX ISO descriptors. It is assumed that both kind of descriptors
    * have exactly the same length.
    * For each CIS channel or ISO Mode 0 connection, 2 RX ISO descriptors and 2 TX ISO descriptors will be taken from this pool
    * For each BIS channel, either 4 RX ISO descriptors or 4 TX ISO descriptors will be taken from this pool.
    */
    //#define EM_BLE_ISO_DESC_OFFSET                CO_ALIGN4_HI(EM_BLE_ISO_HOP_SEQ_END)
    #define EM_BLE_ISO_DESC_OFFSET          (rom_config_get_em_offset(ROM_EM_BLE_ISO_DESC_OFFSET))

    #define EM_BLE_ISO_DESC_NB                    (BLE_ISO_DESC_NB)
    #define EM_BLE_ISO_DESC_SIZE                  (REG_EM_BLE_TX_ISO_DESC_SIZE)
    #define EM_BLE_ISO_DESC_END                   (EM_BLE_ISO_DESC_OFFSET + EM_BLE_ISO_DESC_NB * EM_BLE_ISO_DESC_SIZE)

    #define EM_NUM_BLE_ISO_DESC                   ((EM_BLE_ISO_BUF_OFFSET - EM_BLE_ISO_DESC_OFFSET) / EM_BLE_ISO_DESC_SIZE)

    /// !!!! REMINDER: TX and RX ISO descriptors have the same size !!!! Can use REG_EM_BLE_TX_ISO_DESC_SIZE and REG_EM_BLE_RX_ISO_DESC_SIZE
    #define EM_BLE_TX_ISO_DESC_OFFSET             (EM_BLE_ISO_DESC_OFFSET)
    #define EM_BLE_RX_ISO_DESC_OFFSET             (EM_BLE_ISO_DESC_OFFSET)
    #if (REG_EM_BLE_TX_ISO_DESC_SIZE != REG_EM_BLE_RX_ISO_DESC_SIZE)
        #error "TX and RX ISO descriptors shall have the same size"
    #endif // (REG_EM_BLE_TX_ISO_DESC_SIZE != REG_EM_BLE_RX_ISO_DESC_SIZE)
    #if ((REG_EM_BLE_TX_ISO_DESC_SIZE % 4) != 0)
        #error "TX/RX ISO Descriptor must be 32 bits aligned"
    #endif // ((REG_EM_BLE_TX_ISO_DESC_SIZE % 4) != 0)

    /*
    * ISO TX/RX DATA BUFFERS
    ****************************************************************************************
    */
    /// RX/TX Isochronous buffer area definition
    //#define EM_BLE_ISO_BUF_OFFSET                 CO_ALIGN4_HI(EM_BLE_ISO_DESC_END)
    #define EM_BLE_ISO_BUF_OFFSET          (rom_config_get_em_offset(ROM_EM_BLE_ISO_BUF_OFFSET))

    #define EM_BLE_ISO_BUF_NB                     (BLE_ISO_BUF_NB)
    #define EM_BLE_ISO_BUF_SIZE                   (REG_EM_BLE_TX_ISO_BUF_SIZE)
    #define EM_BLE_ISO_BUF_END                    (EM_BLE_ISO_BUF_OFFSET + EM_BLE_ISO_BUF_NB * EM_BLE_ISO_BUF_SIZE)
    #define EM_NUM_BLE_ISO                        ((EM_BLE_RX_CTE_DESC_OFFSET - EM_BLE_ISO_BUF_OFFSET) / EM_BLE_ISO_BUF_SIZE)

    /// !!!! REMINDER: TX and RX ISO buffer have the same size !!!! Can use REG_EM_BLE_TX_ISO_BUF_SIZE and REG_EM_BLE_RX_ISO_BUF_SIZE
    #define EM_BLE_TX_ISO_BUF_OFFSET           (EM_BLE_ISO_BUF_OFFSET)
    #define EM_BLE_RX_ISO_BUF_OFFSET           (EM_BLE_ISO_BUF_OFFSET)
    #if (REG_EM_BLE_TX_ISO_BUF_SIZE != REG_EM_BLE_RX_ISO_BUF_SIZE)
        #error "TX and RX ISO buffer shall have the same size"
    #endif // (REG_EM_BLE_TX_ISO_DESC_SIZE != REG_EM_BLE_RX_ISO_DESC_SIZE)
    #if ((REG_EM_BLE_TX_ISO_BUF_SIZE % 4) != 0)
        #error "TX/RX ISO Buffers must be 32 bits aligned"
    #endif // ((REG_EM_BLE_TX_ISO_BUF_SIZE % 4) != 0)

#else
    #define EM_BLE_ISO_BUF_END                    (EM_BLE_ACLTXBUF_END)
#endif // (BLE_ISO_PRESENT)

/*
 * RX DESCRIPTORS FOR DIRECTION FINDING
 ****************************************************************************************
 */
/// RX CTE Descriptors area definition
//#define EM_BLE_RX_CTE_DESC_OFFSET      CO_ALIGN4_HI(EM_BLE_ISO_BUF_END)
#define EM_BLE_RX_CTE_DESC_OFFSET       (rom_config_get_em_offset(ROM_EM_BLE_RX_CTE_DESC_OFFSET))

#if (BLE_CON_CTE_REQ | BLE_CONLESS_CTE_RX)
    #define EM_BLE_RX_CTE_DESC_NB          (BLE_RX_CTE_DESC_NB)
    #define EM_BLE_RX_CTE_DESC_END         (EM_BLE_RX_CTE_DESC_OFFSET + EM_BLE_RX_CTE_DESC_NB * REG_EM_BLE_RX_CTE_DESC_SIZE)
#else // (BLE_CON_CTE_REQ | BLE_CONLESS_CTE_RX)
    #define EM_BLE_RX_CTE_DESC_END         (EM_BLE_RX_CTE_DESC_OFFSET)
#endif // (BLE_CON_CTE_REQ | BLE_CONLESS_CTE_RX)

/*
 * ANTENNA IDs FOR DIRECTION FINDING
 ****************************************************************************************
 */
/// Area definition for Tx antenna IDs
//#define EM_BLE_TX_ANTENNA_ID_OFFSET   CO_ALIGN4_HI(EM_BLE_RX_CTE_DESC_END)
#define EM_BLE_TX_ANTENNA_ID_OFFSET   (rom_config_get_em_offset(ROM_EM_BLE_TX_ANTENNA_ID_OFFSET))

#if (BLE_AOA | BLE_AOD)
    #define EM_BLE_TX_ANTENNA_ID_NB       (BLE_MAX_SW_PAT_LEN)
    #define EM_BLE_TX_ANTENNA_ID_END      (EM_BLE_TX_ANTENNA_ID_OFFSET + EM_BLE_TX_ANTENNA_ID_NB)
#else // (BLE_CON_CTE_REQ | BLE_CONLESS_CTE_RX)
    #define EM_BLE_TX_ANTENNA_ID_END      (EM_BLE_TX_ANTENNA_ID_OFFSET)
#endif // (BLE_CON_CTE_REQ | BLE_CONLESS_CTE_RX)

/// Area definition for Rx antenna IDs
//#define EM_BLE_RX_ANTENNA_ID_OFFSET   CO_ALIGN4_HI(EM_BLE_TX_ANTENNA_ID_END)
#define EM_BLE_RX_ANTENNA_ID_OFFSET   (rom_config_get_em_offset(ROM_EM_BLE_RX_ANTENNA_ID_OFFSET))

#if (BLE_AOA | BLE_AOD)
    #define EM_BLE_RX_ANTENNA_ID_NB       (BLE_MAX_SW_PAT_LEN)
    #define EM_BLE_RX_ANTENNA_ID_END      (EM_BLE_RX_ANTENNA_ID_OFFSET + EM_BLE_RX_ANTENNA_ID_NB)
#else // (BLE_CON_CTE_REQ | BLE_CONLESS_CTE_RX)
    #define EM_BLE_RX_ANTENNA_ID_END      (EM_BLE_RX_ANTENNA_ID_OFFSET)
#endif // (BLE_CON_CTE_REQ | BLE_CONLESS_CTE_RX)

/*
 * END
 ****************************************************************************************
 */
#define EM_BLE_END                              (rom_config_get_em_offset(ROM_EM_BLE_END))

/// @} EM

#endif // EM_MAP_BLE_H_
