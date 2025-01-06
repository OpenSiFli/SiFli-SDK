/**
 ****************************************************************************************
 *
 * @file em_map_bt.h
 *
 * @brief Mapping of the different descriptors in the exchange memory
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef EM_MAP_BT_H_
#define EM_MAP_BT_H_

/**
 ****************************************************************************************
 * @addtogroup EM Exchange Memory
 * @ingroup CONTROLLER
 * @brief Mapping of the different descriptors in the exchange memory
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#include "rom_config.h"

#include <stdint.h>                      // integer

#include "_reg_em_bt_cs.h"
#include "_reg_em_bt_rxdesc.h"
#include "_reg_em_bt_txdesc.h"

/// Null pointer in EM offset space
#define EM_PTR_NULL              (0x0000)


/*
 * CONTROL STRUCTURES
 ****************************************************************************************
 */

/**
 * Control Structures area definition
 *
 * - ACL   (N)
 * - INQ
 * - ISCAN
 * - Active slave broadcast
 * - Sync train
 * - Sync scan
 * - Connectionless slave broadcast TX
 * - Connectionless slave broadcast RX
 */
//#define EM_BT_CS_OFFSET          (CO_ALIGN4_HI(EM_BT_OFFSET))
#define EM_BT_CS_OFFSET          (rom_config_get_em_offset(ROM_EM_BT_CS_OFFSET))

#define EM_BT_CS_NB              (MAX_NB_ACTIVE_ACL+5+2*CSB_SUPPORT)
#define EM_BT_CS_END             (EM_BT_CS_OFFSET + EM_BT_CS_NB * REG_EM_BT_CS_SIZE)

/**
 * Control Structures indexes definition
 */
#define EM_BT_CS_ACL_INDEX(acl)  (acl)
#define EM_BT_CS_IDX_TO_LID(idx) (idx)
#define EM_BT_CS_INQ_INDEX       (MAX_NB_ACTIVE_ACL)
#define EM_BT_CS_ISCAN_INDEX     (EM_BT_CS_INQ_INDEX + 1)
#define EM_BT_CS_ACT_BCST_INDEX  (EM_BT_CS_ISCAN_INDEX + 1)
#define EM_BT_CS_STRAIN_INDEX    (EM_BT_CS_ACT_BCST_INDEX + 1)
#define EM_BT_CS_SSCAN_INDEX     (EM_BT_CS_STRAIN_INDEX + 1)
#define EM_BT_CS_CSB_TX_INDEX    (EM_BT_CS_SSCAN_INDEX + 1)
#define EM_BT_CS_CSB_RX_INDEX    (EM_BT_CS_CSB_TX_INDEX + 1)
#define EM_BT_CS_INDEX_MAX       (EM_BT_CS_CSB_RX_INDEX + 1)

/**
 * Frame Format
 * Bluetooth state and device activity used for frequency selection and TDD frame format. See section 3.5 for further details
 * Note this field is automatically updated by the RW-BT Core when changing from Page to Master Page Response mode.
 *  - 0000x: Do not use.               -
 *  - 00010: Master connect.           Master device
 *  - 00011: Slave connect.            Slave device
 *  - 00100: Page.                     Initiator device
 *  - 00101: Page scan.                Responder device
 *  - 00110: Master page response.     -
 *  - 00111: Slave page response.      -
 *  - 01000: Inquiry.                  Initiator device
 *  - 01001: Inquiry response          Responder device
 *  - 01010: Do not use.               -
 *  - 01011: Do not use.               -
 *  - 011xx: Do not use.               -
 *  - 10xxx: Do not use.               -
 *  - 11000: Master Broadcast          Master device
 *  - 11001: Broadcast Scan            Slave device
 *  - 11010: Master Access window.     Master device
 *  - 11011: Slave Access window 1.    Slave device
 *  - 11100: Slave Access window 2.    Slave device
 *  - 11101: Do not use.               -
 *  - 11110: Direct Test Mode          Tx test
 *  - 11110: Direct Test Mode          Rx test
 */
#define EM_BT_CS_FMT_MST_CONNECT    0x02
#define EM_BT_CS_FMT_SLV_CONNECT    0x03
#define EM_BT_CS_FMT_PAGE           0x04
#define EM_BT_CS_FMT_PAGE_SCAN      0x05
#define EM_BT_CS_FMT_MST_PAGE_RSP   0x06
#define EM_BT_CS_FMT_SLV_PAGE_RSP   0x07
#define EM_BT_CS_FMT_INQUIRY        0x08
#define EM_BT_CS_FMT_INQ_RSP        0x09
#if (EAVESDROPPING_SUPPORT)
    #define EM_BT_CS_FMT_ED_MST_CONNECT 0x0C
    #define EM_BT_CS_FMT_ED_SLV_CONNECT 0x0D
#endif // EAVESDROPPING_SUPPORT
#define EM_BT_CS_FMT_MST_BCST       0x18
#define EM_BT_CS_FMT_BCST_SCAN      0x19
#define EM_BT_CS_FMT_MST_ACC_WIN    0x1A
#define EM_BT_CS_FMT_SLV_ACC_WIN1   0x1B
#define EM_BT_CS_FMT_SLV_ACC_WIN2   0x1C
#if (BT_HCI_TEST_MODE)
    #define EM_BT_CS_FMT_DTM_TX         0x1E
    #define EM_BT_CS_FMT_DTM_RX         0x1F
#endif //(BT_HCI_TEST_MODE)

/// Maximum value of RXWINSZ field (8191)
#define EM_BT_CS_RXWINSZ_MAX        ((1 << 13)-1)

/// Maximum value of MAXFRMTIME field (8191)
#define EM_BT_CS_MAXFRMTIME_MAX     ((1 << 13)-1)

/*
 * RX DESCRIPTORS
 ****************************************************************************************
 */

/**
 * RX Descriptors area definition
 */
//#define EM_BT_RXDESC_OFFSET      (CO_ALIGN4_HI(EM_BT_CS_END))
#define EM_BT_RXDESC_OFFSET          (rom_config_get_em_offset(ROM_EM_BT_RXDESC_OFFSET))

#define EM_BT_RXDESC_NB          (BT_RXDESC_NB)
#define EM_BT_RXDESC_END         (EM_BT_RXDESC_OFFSET + EM_BT_RXDESC_NB * REG_EM_BT_RXDESC_SIZE)
#define EM_NUM_BT_RXDESC         ((EM_BT_TXDESC_OFFSET - EM_BT_RXDESC_OFFSET)/REG_EM_BT_RXDESC_SIZE)

/*
 * TX DESCRIPTORS
 ****************************************************************************************
 */

/**
 * TX Descriptors area definition
 *
 * - ACL (2xN)
 * - Active slave broadcast
 * - FHS
 * - EIR
 * - Sync train packet
 * - Connectionless slave broadcast
 */
#define EM_BT_TXDESC_OFFSET      (rom_config_get_em_offset(ROM_EM_BT_TXDESC_OFFSET))
#define EM_BT_TXDESC_NB          (2*MAX_NB_ACTIVE_ACL + 4 + CSB_SUPPORT)
#define EM_BT_TXDESC_END         (EM_BT_TXDESC_OFFSET + EM_BT_TXDESC_NB * REG_EM_BT_TXDESC_SIZE)

#define EM_BT_TXDESC_ACL_INDEX(acl, tog)      (2*acl + tog)
#define EM_BT_TXDESC_ACT_BCST_INDEX           (2*MAX_NB_ACTIVE_ACL)
#define EM_BT_TXDESC_ISCANFHS_INDEX           (EM_BT_TXDESC_ACT_BCST_INDEX + 1)
#define EM_BT_TXDESC_EIR_INDEX                (EM_BT_TXDESC_ISCANFHS_INDEX + 1)
#define EM_BT_TXDESC_STP_INDEX                (EM_BT_TXDESC_EIR_INDEX + 1)
#define EM_BT_TXDESC_CSB_INDEX                (EM_BT_TXDESC_STP_INDEX + 1)

/*
 * LMP RX BUFFERS
 ****************************************************************************************
 */

/// LMP RX buffers area definition
//#define EM_BT_LMPRXBUF_OFFSET    (CO_ALIGN4_HI(EM_BT_TXDESC_END))
#define EM_BT_LMPRXBUF_OFFSET          (rom_config_get_em_offset(ROM_EM_BT_LMPRXBUF_OFFSET))

#define EM_BT_LMPRXBUF_NB        (EM_BT_RXDESC_NB)
#define EM_BT_LMPRXBUF_SIZE      (20)
#define EM_BT_LMPRXBUF_END       (EM_BT_LMPRXBUF_OFFSET + EM_BT_LMPRXBUF_NB * EM_BT_LMPRXBUF_SIZE)

/*
 * LMP TX BUFFERS
 ****************************************************************************************
 */

/// LMP TX buffers area definition
//#define EM_BT_LMPTXBUF_OFFSET    (CO_ALIGN4_HI(EM_BT_LMPRXBUF_END))
#define EM_BT_LMPTXBUF_OFFSET          (rom_config_get_em_offset(ROM_EM_BT_LMPTXBUF_OFFSET))

#define EM_BT_LMPTXBUF_NB        (BT_LMP_BUF_NB_TX)
#define EM_BT_LMPTXBUF_SIZE      (24)
#define EM_BT_LMPTXBUF_END       (EM_BT_LMPTXBUF_OFFSET + EM_BT_LMPTXBUF_NB * EM_BT_LMPTXBUF_SIZE)
#define EM_NUM_BT_LMPTXBUF       ((EM_BT_ISCANFHSTXBUF_OFFSET - EM_BT_LMPTXBUF_OFFSET)/EM_BT_LMPTXBUF_SIZE)

/*
 * ISCAN FHS TX BUFFER
 ****************************************************************************************
 */

/// ISCAN FHS TX buffer area definition
//#define EM_BT_ISCANFHSTXBUF_OFFSET    (CO_ALIGN4_HI(EM_BT_LMPTXBUF_END))
#define EM_BT_ISCANFHSTXBUF_OFFSET          (rom_config_get_em_offset(ROM_EM_BT_ISCANFHSTXBUF_OFFSET))

#define EM_BT_ISCANFHSTXBUF_SIZE      (20)
#define EM_BT_ISCANFHSTXBUF_END       (EM_BT_ISCANFHSTXBUF_OFFSET + EM_BT_ISCANFHSTXBUF_SIZE)

/*
 * PAGE FHS TX BUFFER
 ****************************************************************************************
 */

/// PAGE FHS TX buffer area definition
//#define EM_BT_PAGEFHSTXBUF_OFFSET    (CO_ALIGN4_HI(EM_BT_ISCANFHSTXBUF_END))
#define EM_BT_PAGEFHSTXBUF_OFFSET          (rom_config_get_em_offset(ROM_EM_BT_PAGEFHSTXBUF_OFFSET))

#define EM_BT_PAGEFHSTXBUF_SIZE      (20)
#define EM_BT_PAGEFHSTXBUF_END       (EM_BT_PAGEFHSTXBUF_OFFSET + EM_BT_PAGEFHSTXBUF_SIZE)

/*
 * EIR TX BUFFER
 ****************************************************************************************
 */

/// EIR TX buffer area definition
//#define EM_BT_EIRTXBUF_OFFSET    (CO_ALIGN4_HI(EM_BT_PAGEFHSTXBUF_END))
#define EM_BT_EIRTXBUF_OFFSET          (rom_config_get_em_offset(ROM_EM_BT_EIRTXBUF_OFFSET))

#define EM_BT_EIRTXBUF_SIZE      (240)
#define EM_BT_EIRTXBUF_END       (EM_BT_EIRTXBUF_OFFSET + EM_BT_EIRTXBUF_SIZE)

/*
 * SAM SUBMAP BUFFER
 ****************************************************************************************
 */

/// SAM SUBMAP (Local) buffer area definition
//#define EM_BT_LOCAL_SAM_SUBMAP_OFFSET    (CO_ALIGN4_HI(EM_BT_EIRTXBUF_END))
#define EM_BT_LOCAL_SAM_SUBMAP_OFFSET          (rom_config_get_em_offset(ROM_EM_BT_LOCAL_SAM_SUBMAP_OFFSET))

#define EM_BT_LOCAL_SAM_SUBMAP_SIZE      (SAM_TYPE0_SUBMAP_LEN)
#define EM_BT_LOCAL_SAM_SUBMAP_END       (EM_BT_LOCAL_SAM_SUBMAP_OFFSET + EM_BT_LOCAL_SAM_SUBMAP_SIZE)

/// SAM MAP (Peer) buffer area definition
//#define EM_BT_PEER_SAM_MAP_OFFSET    (CO_ALIGN4_HI(EM_BT_LOCAL_SAM_SUBMAP_END))
#define EM_BT_PEER_SAM_MAP_OFFSET          (rom_config_get_em_offset(ROM_EM_BT_PEER_SAM_MAP_OFFSET))

#define EM_BT_PEER_SAM_MAP_OFF(acl)  (EM_BT_PEER_SAM_MAP_OFFSET + (acl * RW_PEER_SAM_MAP_MAX_LEN))
#define EM_BT_PEER_SAM_MAP_SIZE      (MAX_NB_ACTIVE_ACL * RW_PEER_SAM_MAP_MAX_LEN) // HW restriction
#define EM_BT_PEER_SAM_MAP_END       (EM_BT_PEER_SAM_MAP_OFFSET + EM_BT_PEER_SAM_MAP_SIZE)


/*
 * STP TX BUFFER
 ****************************************************************************************
 */

/// Sync Train Packet TX buffer area definition
//#define EM_BT_STPTXBUF_OFFSET    (CO_ALIGN4_HI(EM_BT_PEER_SAM_MAP_END))
#define EM_BT_STPTXBUF_OFFSET          (rom_config_get_em_offset(ROM_EM_BT_STPTXBUF_OFFSET))

#define EM_BT_STPTXBUF_SIZE      (28)
#define EM_BT_STPTXBUF_END       (EM_BT_STPTXBUF_OFFSET + EM_BT_STPTXBUF_SIZE)

/*
 * ACL RX BUFFERS
 ****************************************************************************************
 */

/// ACL RX buffers area definition
//#define EM_BT_ACLRXBUF_OFFSET    (CO_ALIGN4_HI(EM_BT_STPTXBUF_END))
#define EM_BT_ACLRXBUF_OFFSET          (rom_config_get_em_offset(ROM_EM_BT_ACLRXBUF_OFFSET))

#define EM_BT_ACLRXBUF_NB        (ACL_DATA_BUF_NB_RX)
#define EM_BT_ACLRXBUF_SIZE      (1028)
#define EM_BT_ACLRXBUF_END       (EM_BT_ACLRXBUF_OFFSET + EM_BT_ACLRXBUF_NB * EM_BT_ACLRXBUF_SIZE)

#define EM_NUM_BT_ACLRXBUF       ((EM_BT_ACLTXBUF_OFFSET - EM_BT_ACLRXBUF_OFFSET)/EM_BT_ACLRXBUF_SIZE)

/*
 * ACL TX BUFFERS
 ****************************************************************************************
 */

/// ACL TX buffers area definition
//#define EM_BT_ACLTXBUF_OFFSET    (CO_ALIGN4_HI(EM_BT_ACLRXBUF_END))
//#define EM_BT_ACLTXBUF_NB        (ACL_DATA_BUF_NB_TX)
#define EM_BT_ACLTXBUF_OFFSET    (rom_config_get_em_offset(ROM_EM_BT_ACLTXBUF_OFFSET))
#define EM_BT_ACLTXBUF_NB        (ACL_DATA_BUF_NB_TX)
#define EM_BT_ACLTXBUF_SIZE      (1028)
#define EM_BT_ACLTXBUF_END       (EM_BT_ACLTXBUF_OFFSET + EM_BT_ACLTXBUF_NB * EM_BT_ACLTXBUF_SIZE)

#define EM_NUM_BT_ACLTXBUF       ((EM_BT_AUDIOBUF_OFFSET-EM_BT_ACLTXBUF_OFFSET)/EM_BT_ACLTXBUF_SIZE)

/*
 * AUDIO RX BUFFERS
 ****************************************************************************************
 */

/// AUDIO buffers area definition
//#define EM_BT_AUDIOBUF_OFFSET  (CO_ALIGN4_HI(EM_BT_ACLTXBUF_END))
#define EM_BT_AUDIOBUF_OFFSET          (rom_config_get_em_offset(ROM_EM_BT_AUDIOBUF_OFFSET))


#define EM_BT_AUDIOBUF_NB      (MAX_NB_SYNC)
#define EM_BT_AUDIOBUF_SIZE    (2*1080) // (4*1080) !!! Maximum configuration (2 x eSCO-3EV5) not possible on Bubble
#define EM_BT_AUDIOBUF_END     (EM_BT_AUDIOBUF_OFFSET + EM_BT_AUDIOBUF_NB * EM_BT_AUDIOBUF_SIZE)

#define EM_BT_AUDIOBUF_OFF(sco)   (EM_BT_AUDIOBUF_OFFSET + (sco * EM_BT_AUDIOBUF_SIZE))

/*
 * END
 ****************************************************************************************
 */

#define EM_BT_END                (rom_config_get_em_offset(ROM_EM_BT_END))

/// @} EM

#endif // EM_MAP_BT_H_
