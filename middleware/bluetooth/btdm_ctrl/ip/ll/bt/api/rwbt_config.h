/**
 ****************************************************************************************
 *
 * @file rwbt_config.h
 *
 * @brief Configuration of the BT protocol stack (max number of supported connections,
 * type of partitioning, etc.)
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef RWBT_CONFIG_H_
#define RWBT_CONFIG_H_

/**
 ****************************************************************************************
 * @addtogroup ROOT
 * @{
 *
 * @name RWBT Stack Configuration
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/******************************************************************************************/
/* --------------------------   GENERAL SETUP       --------------------------------------*/
/******************************************************************************************/

/// Maximum number of ACL links
#define MAX_NB_ACTIVE_ACL                   CFG_CON_ACL

/// BT Heap size
#define BT_HEAP_MSG_SIZE                    (256 + 256 * MAX_NB_ACTIVE_ACL)
#define BT_HEAP_ENV_SIZE                    (1024 + 1024 * MAX_NB_ACTIVE_ACL)

/// Number of BT tasks
#define BT_TASK_SIZE                        4

/// Number of RX descriptors
#define BT_RXDESC_NB                 (4)

/// Number of RX/TX ACL data buffers
#if 0
    //#if defined(CFG_SLIM)
    #define ACL_DATA_BUF_NB_RX           (BT_RXDESC_NB)
    #define ACL_DATA_BUF_NB_TX           (MAX_NB_ACTIVE_ACL + 2)
    //#else // !(CFG_SLIM)
#endif
#define ACL_DATA_BUF_NB_RX           (BT_RXDESC_NB + 2)
#define ACL_DATA_BUF_NB_TX           (MAX_NB_ACTIVE_ACL + 6)

//#endif //(CFG_SLIM)
/// Guard space reserved for HCI header in ACL RX buffers
#define ACL_RX_BUF_HEADER_SPACE      (HCI_ACL_HDR_LEN + 1)

/// Number of TX LMP buffers
#if (MAX_NB_ACTIVE_ACL > 3)
    #define BT_LMP_BUF_NB_TX             (2*MAX_NB_ACTIVE_ACL)
#else // (MAX_NB_ACTIVE_ACL > 3)
    #define BT_LMP_BUF_NB_TX             (7)
#endif // (MAX_NB_ACTIVE_ACL > 3)

/// Maximum ACL Data Packet Size
#define ACL_DATA_BUF_SIZE            DH5_3_PACKET_SIZE

/// Normal Rx window size
#define NORMAL_WIN_SIZE              26

/// Range of key size supported by the FW
#define ENC_KEY_SIZE_MAX        16
#define ENC_KEY_SIZE_MIN        7
#define ENC_KEY_SIZE_MASK       0xFF80

#if (BT_53)
    /// Vendor-specific Default min encrytion key size
    #define MIN_ENC_KEY_SIZE_DEFAULT    ENC_KEY_SIZE_MIN
#endif // (BT_53)

/// Inquiry response TX power (the value code depends on radio used)
#define INQ_RSP_TX_PWR  rwip_rf.txpwr_max

/// Size of the inquiry result filtering list
#define INQ_FILT_LEN                 10

/******************************************************************************************/
/* --------------------------   SYNC SETUP      ------------------------------------------*/
/******************************************************************************************/


#if VOICE_OVER_HCI
    /// Number of Synchronous TX buffers used for one VoHCI link
    #define SYNC_TX_BUF_NB     4
    /// Number of Synchronous RX buffers used for one VoHCI link
    #define SYNC_RX_BUF_NB     3
    /// Guard space reserved for HCI header in Sync RX buffers
    #define SYNC_RX_BUF_HEADER_SPACE      (HCI_SYNC_HDR_LEN + 1)
#endif //(VOICE_OVER_HCI)

/// Synchronous link negotiation number of retries
#define SYNC_NEGO_TRY                  1

/******************************************************************************************/
/* -------------------------   FIRMWARE SETUP      ---------------------------------------*/
/******************************************************************************************/

///Number of HCI Event filter
#define HCI_FILTER_NB                                   0x5

/*
 * Test mode option to be able to enter test mode as defined
 * in part I1: BT Test Mode, of the BT specification.
 */
/// Test mode support
#define TEST_MODE                                   1

/******************************************************************************************/
/* -------------------------   COEXISTENCE SETUP      ------------------------------------*/
/******************************************************************************************/

/// WLAN Coexistence
#define RW_BT_WLAN_COEX         RW_WLAN_COEX
///WLAN test mode
#define RW_BT_WLAN_COEX_TEST    RW_WLAN_COEX_TEST

/// MWS Coexistence
#define RW_BT_MWS_COEX         RW_MWS_COEX
///WLAN test mode
#define RW_BT_MWS_COEX_TEST    RW_MWS_COEX_TEST

///To let the HW using the default values set in the registers
#define RW_BT_PTI_PRIO_AUTO    31

/// Coexistence Packet traffic information - Index Definitions
enum rwbt_pti_config_idx
{
    BT_PTI_PAGERES_IDX,
    BT_PTI_MCONPEND_IDX,
    BT_PTI_PSCANRES_IDX,
    BT_PTI_SCONPEND_IDX,
    BT_PTI_MSSWITCH_IDX,
    BT_PTI_SNIFFANC_IDX,
    BT_PTI_BEACON_IDX,
    BT_PTI_SNIFFATT_IDX,
    BT_PTI_ACCWIN_IDX,
    BT_PTI_POLLINT_IDX,
    BT_PTI_PAGE_IDX,
    BT_PTI_PSCAN_IDX,
    BT_PTI_INQ_IDX,
    BT_PTI_INQSCAN_IDX,
    BT_PTI_INQRES_IDX,
    BT_PTI_SCORSVD_IDX,
    BT_PTI_SCORETX_IDX,
    BT_PTI_ACLTX_IDX,
    BT_PTI_ACLRETX_IDX,
    BT_PTI_LMPTX_IDX,
    BT_PTI_BCAST_IDX,
    BT_PTI_CONNECT_IDX,
    BT_PTI_IDX_MAX
};

/******************************************************************************************/
/* --------------------          Tx/Rx Path Compensation         -------------------------*/
/******************************************************************************************/

/// Default TX Path Compensation - The default value is vendor-specific (in 0.1 dB).
#define BT_DFT_TX_PATH_RF_COMPENSATION    0

/// Default RX Path Compensation - The default value is vendor-specific (in 0.1 dB).
#define BT_DFT_RX_PATH_RF_COMPENSATION    0


/******************************************************************************************/
/* -------------------------        AFH SETUP         ------------------------------------*/
/******************************************************************************************/

/// Minimum classification interval requested to slave devices (in slots) (2 secs)
#define BT_AFH_CH_CLASS_INT_MIN      0x0C80
/// Maximum classification interval requested to slave devices (in slots) (4 secs)
#define BT_AFH_CH_CLASS_INT_MAX      0x1900

/// AFH update period (in sec)
#define BT_AFH_UPDATE_PERIOD         4

/******************************************************************************************/
/* --------------------------     CSB SETUP       ----------------------------------------*/
/******************************************************************************************/


/******************************************************************************************/
/* --------------------------     PCA SETUP       ----------------------------------------*/
/******************************************************************************************/


/// PCA update period (in msec)
#define BT_PCA_UPDATE_PERIOD        30

/// Minimum offset to perform a coarse clock adjust
#define BT_PCA_MIN_CC_ADJ_US        40

/// PCA request clk_adj_slots in addition to clk_adj_us: enable=1, disable=0
#define BT_PCA_CLK_ADJ_SLOTS        0 // No mandate to request a clk_adj_slots currently - for test only

/// Default knudge increment on page/inq trains.
#define BT_KNUDGE_INC               2


/******************************************************************************************/
/* ------------------------   CONNECTION HANDLE   ----------------------------------------*/
/******************************************************************************************/

/// Bit set for BT ACL connection handles
#define BT_ACL_CONHDL_BIT       (0x80)
#define BT_ACL_CONHDL_MIN       (BT_ACL_CONHDL_BIT)
#define BT_ACL_CONHDL_MAX       (BT_ACL_CONHDL_BIT + MAX_NB_ACTIVE_ACL - 1)
#if EAVESDROPPING_SUPPORT
    #define BT_ACL_CORR_CONHDL_BIT       (0x90)
    #define BT_ACL_CORR_CONHDL_MIN       (BT_ACL_CORR_CONHDL_BIT)
    #define BT_ACL_CORR_CONHDL_MAX       (BT_ACL_CORR_CONHDL_BIT + MAX_NB_ACTIVE_ACL - 1)
#endif // EAVESDROPPING_SUPPORT

/// Bit set for BT SCO connection handles
#define BT_SYNC_CONHDL_OFF                      (8)
#define BT_SYNC_CONHDL_MSK                      (((1 << MAX_NB_SYNC) - 1) << BT_SYNC_CONHDL_OFF)
#define BT_SYNC_CONHDL_BIT(sco_link_id)         ((sco_link_id + 1) << BT_SYNC_CONHDL_OFF)
#define BT_SYNC_CONHDL_LID(sco_conhdl)          ((sco_conhdl >> BT_SYNC_CONHDL_OFF) - 1)
#define BT_SYNC_CONHDL(link_id, sco_link_id)    (BT_ACL_CONHDL_MIN + link_id + BT_SYNC_CONHDL_BIT(sco_link_id))

/// @} BT Stack Configuration
/// @} ROOT

#endif //RWBT_CONFIG_H_
