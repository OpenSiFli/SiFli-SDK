/**
 ****************************************************************************************
 *
 * @file rwip_config.h
 *
 * @brief Configuration of the RW IP SW
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef RWIP_CONFIG_H_
#define RWIP_CONFIG_H_

#include "rtconfig.h"
#include <stdint.h>

#undef BSP_BLE_SIBLES

/**
 ****************************************************************************************
 * @addtogroup ROOT
 * @{
 *
 *  Information about RW SW IP options and flags
 *
 *        BT_DUAL_MODE             BT/BLE Dual Mode
 *        BT_STD_MODE              BT Only
 *        BLE_STD_MODE             BLE Only
 *
 *        BT_EMB_PRESENT           BT controller exists
 *        BLE_EMB_PRESENT          BLE controller exists
 *        BLE_HOST_PRESENT         BLE host exists
 *
 * @name RW Stack Configuration
 * @{
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */
#define CFG_SLIM 1

#if defined(CFG_HCI_SLIM)
    #define SMP_SLIM 1
    #define GAPM_SLIM 1
    #define GAPC_SLIM 1
    #define ATT_SLIM 1
#endif

/******************************************************************************************/
/* -------------------------   EAVESDROPPING FLAG      -----------------------------------*/
/******************************************************************************************/
#if defined(CFG_ED)
    #define EAVESDROPPING_SUPPORT           1
#else // defined(CFG_ED)
    #define EAVESDROPPING_SUPPORT           0
#endif // defined(CFG_ED)

/******************************************************************************************/
/* -------------------------   STACK PARTITIONING      -----------------------------------*/
/******************************************************************************************/

#if defined(CFG_BT_EMB)
    #define BT_EMB_PRESENT              1
#else // !defined(CFG_BT_EMB)
    #define BT_EMB_PRESENT              0
#endif // defined(CFG_BT_EMB)

#if defined(CFG_BLE_EMB)
    #define BLE_EMB_PRESENT             1
#else // !defined(CFG_BLE_EMB)
    #define BLE_EMB_PRESENT             0
#endif // defined(CFG_BLE_EMB)

#define EMB_PRESENT                 (BT_EMB_PRESENT || BLE_EMB_PRESENT)

#if (defined(CFG_BLE_HOST))
    #define BLE_HOST_PRESENT            1
#else // !(defined(CFG_BLE_HOST))
    #define BLE_HOST_PRESENT            0
#endif // (defined(CFG_BLE_HOST))


#if (defined(CFG_BT_HOST))
    #define BT_HOST_PRESENT             1
    #define BT_HOST_OTHER_STACK         1
#else // !(defined(CFG_BT_HOST))
    #define BT_HOST_PRESENT             0
    #define BT_HOST_OTHER_STACK         0
#endif // (defined(CFG_BT_HOST))

#define HOST_PRESENT                (BLE_HOST_PRESENT || BT_HOST_PRESENT)
#define HOST_MSG_API                 HOST_PRESENT
#define CFG_HL_MSG_API               CFG_HCITL


#if (defined(CFG_APP) && HOST_PRESENT)
    #define APP_PRESENT                 1
    #define BLE_APP_PRESENT             1
#else // !(defined(CFG_APP) && HOST_PRESENT)
    #define APP_PRESENT                 0
    #define BLE_APP_PRESENT             0
#endif // (defined(CFG_APP) && HOST_PRESENT)

#define BT_DUAL_MODE                (BT_EMB_PRESENT && BLE_EMB_PRESENT)
#define HCI_PRESENT                 1

/// Flag indicating that Mesh is supported
#if defined(CFG_BLE_MESH)
    #define BLE_MESH      1
#else  // !defined(CFG_BLE_MESH)
    #define BLE_MESH      0
#endif // defined(CFG_BLE_MESH)

/// Flag indicating that Generic Audio Framework is supported
#if defined(CFG_BLE_GAF)
    #define BLE_GAF_PRESENT      1
#else //defined(CFG_BLE_GAF)
    #define BLE_GAF_PRESENT      0
#endif //defined(CFG_BLE_GAF)

/// Memory leak detection - platform specific debug
#if (defined(CFG_MEM_LEAK_DETECT))
    #define PLF_MEM_LEAK_DETECT 1
#else
    #define PLF_MEM_LEAK_DETECT 0
#endif // (defined(CFG_MEM_LEAK_DETECT))

/******************************************************************************************/
/* --------------------------      BT VERSION       --------------------------------------*/
/******************************************************************************************/

/// Flag indicating BT 5.3 support
#if defined(CFG_BT_53)
    #define BT_53                       1
#else // !defined(CFG_BT_53)
    #define BT_53                       0
#endif // defined(CFG_BT_53)

/******************************************************************************************/
/* -------------------------   INTERFACES DEFINITIONS      -------------------------------*/
/******************************************************************************************/

/// Application Host Interface - requires host message API
#if defined(CFG_AHITL) && defined(CFG_HL_MSG_API)
    #define AHI_TL_SUPPORT       1
#else // defined(CFG_AHITL) && defined(CFG_HL_MSG_API)
    #define AHI_TL_SUPPORT       0
#endif // defined(CFG_AHITL) && defined(CFG_HL_MSG_API)

/// Application Host Interface - in the same core
#if defined(CFG_AHIINT)
    #define AHI_INT_SUPPORT       1
#else // defined(CFG_AHITL)
    #define AHI_INT_SUPPORT       0
#endif // defined(CFG_AHITL)


/// Host Controller Interface Support (defines if HCI parser is present or not)
#if (defined(CFG_HCITL) || BT_HOST_PRESENT)
    #define HCI_TL_SUPPORT      1
#else //defined(CFG_HCITL)
    #define HCI_TL_SUPPORT      0
#endif //defined(CFG_HCITL)


#if BLE_HOST_PRESENT
    #define H4TL_SUPPORT      ((AHI_TL_SUPPORT) + (HCI_TL_SUPPORT))
#else // !BLE_HOST_PRESENT
    #define H4TL_SUPPORT      (HCI_TL_SUPPORT)
#endif // BLE_HOST_PRESENT

/// Number of HCI commands the stack can handle simultaneously
#define HCI_NB_CMD_PKTS   (5 * HCI_TL_SUPPORT)

/******************************************************************************************/
/* --------------------------   BT COMMON DEFINITIONS      ------------------------------*/
/******************************************************************************************/

#if defined (CFG_CON_SCO)
    /// Maximum number of Synchronous connections (0 to 2)
    #define MAX_NB_SYNC                      (CFG_CON_SCO)
#else //  defined (CFG_CON_SCO)
    #define MAX_NB_SYNC                      (0)
#endif // defined (CFG_CON_SCO)

/// Flag indicating if debug mode is activated or not
#if defined (CFG_VOHCI) && (MAX_NB_SYNC > 0)
    #define VOICE_OVER_HCI                   1
#else
    #define VOICE_OVER_HCI                   0
#endif // defined (CFG_VOHCI) && (MAX_NB_SYNC > 0)


/// Support of Connectionless Slave Broadcast
#if defined (CFG_CSB)
    #define CSB_SUPPORT  1
#else
    #define CSB_SUPPORT  0
#endif // defined (CFG_CSB)

/// Support of Piconet Clock Adjust
#if defined (CFG_PCA)
    #define PCA_SUPPORT  1
#else
    #define PCA_SUPPORT  0
#endif // defined (CFG_PCA)

// Mutually exclusive features - Support of Broadcast Encryption only if not PCA Supported
#if (EMB_PRESENT)
    #define BCAST_ENC_SUPPORT (!PCA_SUPPORT)
#else
    #define BCAST_ENC_SUPPORT (0)
#endif

/******************************************************************************************/
/* --------------------------   BLE COMMON DEFINITIONS      ------------------------------*/
/******************************************************************************************/
/// Kernel Heap memory sized reserved for allocate dynamically connection environment
#define KE_HEAP_MEM_RESERVED        (4)


/// Broadcaster
#if (defined(CFG_BROADCASTER) || defined(CFG_PERIPHERAL) || defined(CFG_ALLROLES))
    #define BLE_BROADCASTER      1
#else
    #define BLE_BROADCASTER      0
#endif // (defined(CFG_BROADCASTER) || defined(CFG_PERIPHERAL) || defined(CFG_ALLROLES))

/// Observer
#if (defined(CFG_OBSERVER) || defined(CFG_CENTRAL) || defined(CFG_ALLROLES))
    #define BLE_OBSERVER      1
#else
    #define BLE_OBSERVER      0
#endif // (defined(CFG_OBSERVER) || defined(CFG_CENTRAL) || defined(CFG_ALLROLES))

/// Central
#if (defined(CFG_CENTRAL) || defined(CFG_ALLROLES))
    #define BLE_CENTRAL      1
#else
    #define BLE_CENTRAL      0
#endif // (defined(CFG_CENTRAL) || defined(CFG_ALLROLES))

/// Peripheral
#if (defined(CFG_PERIPHERAL) || defined(CFG_ALLROLES))
    #define BLE_PERIPHERAL      1
#else
    #define BLE_PERIPHERAL      0
#endif // (defined(CFG_PERIPHERAL) || defined(CFG_ALLROLES))

#if (HOST_PRESENT)
    #define HOST_ACTIVITY_MAX        (CFG_ACT)
    #if (BT_HOST_PRESENT || BLE_CENTRAL || BLE_PERIPHERAL)
        #define BLE_CONNECTION_MAX      (CFG_CON)
    #else // !(BT_HOST_PRESENT || BLE_CENTRAL || BLE_PERIPHERAL)
        #define BLE_CONNECTION_MAX       0
    #endif // !(BT_HOST_PRESENT || BLE_CENTRAL || BLE_PERIPHERAL)

    // BT use another configuration.
    #define HOST_CONNECTION_MAX     BLE_CONNECTION_MAX
    #if (HOST_CONNECTION_MAX >= HOST_ACTIVITY_MAX)
        #error "Number of connections must be strictly less than number of activities"
    #endif // (HOST_CONNECTION_MAX >= BLE_ACTIVITY_MAX)
#endif // (HOST_PRESENT)
extern uint8_t rom_config_get_ble_max_act(void);

#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
    #if ((BLE_BROADCASTER+BLE_OBSERVER+BLE_PERIPHERAL+BLE_CENTRAL) == 0)
        #error "No application role defined"
    #endif // ((BLE_BROADCASTER+BLE_OBSERVER+BLE_PERIPHERAL+BLE_CENTRAL) == 0)

    #if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
        /// Maximum number of simultaneous BLE activities (scan, connection, advertising, initiating)
        #define BLE_ACTIVITY_MAX          (CFG_ACT)
        #define BLE_ACTIVITY_MAX_CFG          (rom_config_get_ble_max_act())
    #endif // (BLE_EMB_PRESENT || BLE_HOST_PRESENT)

    /// Maximum number of devices in RAL
    #define BLE_RAL_MAX_CONFIG          (CFG_RAL)

    /// Max advertising reports before sending the info to the host
    #define BLE_ADV_REPORTS_MAX              (1)

    #if (BLE_EMB_PRESENT)
        /// Maximum number of ADV reports in the HCI queue to Host
        #define BLE_MAX_NB_ADV_REP_FRAG       (2 * HCI_TL_SUPPORT)
        /// Maximum number of IQ reports in the HCI queue to Host
        #define BLE_MAX_NB_IQ_REP             (2 * HCI_TL_SUPPORT)
    #endif // (BLE_EMB_PRESENT)
#endif // (BLE_EMB_PRESENT || BLE_HOST_PRESENT)

/******************************************************************************************/
/* --------------------------      Power Control                 -------------------------*/
/******************************************************************************************/

/// BT Classic Power Control
#if defined(CFG_BT_PWR_CTRL)
    #define BT_PWR_CTRL                 (1)
#else // !defined(CFG_BT_PWR_CTRL)
    #define BT_PWR_CTRL                 (0)
#endif // defined(CFG_BT_PWR_CTRL)

/// BLE Power Control
#if defined(CFG_LE_PWR_CTRL)
    #define BLE_PWR_CTRL                (1)
#else // !defined(CFG_LE_PWR_CTRL)
    #define BLE_PWR_CTRL                (0)
#endif // defined(CFG_LE_PWR_CTRL)

/******************************************************************************************/
/* ----------------------- Periodic Advertising Sync Transfer (PAST)   -------------------*/
/******************************************************************************************/

/// Periodic Adverising Sync Transfer
#if defined(CFG_PAST)
    #define BLE_PAST                        (1)
#else  // !defined(CFG_PAST)
    #define BLE_PAST                        (0)
#endif // defined(CFG_PAST)

/******************************************************************************************/
/* ----------------------- Advertising Channel Index                   -------------------*/
/******************************************************************************************/

/// Advertising Channel Index
#if defined(CFG_ADV_CH_IDX)
    #define BLE_ADV_CH_IDX                  (1)
#else  // !defined(CFG_ADV_CH_IDX)
    #define BLE_ADV_CH_IDX                  (0)
#endif // defined(CFG_ADV_CH_IDX)

/******************************************************************************************/
/* --------------------------   Direction finding (AoA/AoD)      -------------------------*/
/******************************************************************************************/

#if defined(CFG_CON_CTE_REQ)
    #define BLE_CON_CTE_REQ                (1)
#else // defined(CFG_CON_CTE_REQ)
    #define BLE_CON_CTE_REQ                (0)
#endif // defined(CFG_CON_CTE_REQ)

#if defined(CFG_CON_CTE_RSP)
    #define BLE_CON_CTE_RSP                (1)
#else // defined(CFG_CON_CTE_RSP)
    #define BLE_CON_CTE_RSP                (0)
#endif // defined(CFG_CON_CTE_RSP)

#if defined(CFG_CONLESS_CTE_TX)
    #define BLE_CONLESS_CTE_TX             (1)
#else // defined(CFG_CONLESS_CTE_TX)
    #define BLE_CONLESS_CTE_TX             (0)
#endif // defined(CFG_CONLESS_CTE_TX)

#if defined(CFG_CONLESS_CTE_RX)
    #define BLE_CONLESS_CTE_RX             (1)
#else // defined(CFG_CONLESS_CTE_RX)
    #define BLE_CONLESS_CTE_RX             (0)
#endif // defined(CFG_CONLESS_CTE_RX)

#if defined(CFG_AOD)
    #define BLE_AOD                        (1)
#else // defined(CFG_AOD)
    #define BLE_AOD                        (0)
#endif // defined(CFG_AOD)

#if defined(CFG_AOA)
    #define BLE_AOA                        (1)
#else // defined(CFG_AOA)
    #define BLE_AOA                        (0)
#endif // defined(CFG_AOA)

/******************************************************************************************/
/* --------------------------   ISOCHRONOUS CONFIGURATION        -------------------------*/
/******************************************************************************************/

// check if isochronous is enabled or not
#if  (defined(CFG_ISO_CON) && (CFG_ISO_CON > 0) && (defined(CFG_ISO_MODE_0) || defined(CFG_CIS) || defined(CFG_BIS)))
    #define BLE_ISO_PRESENT                (1)
    #define BLE_ISO_HW_PRESENT             (1)
#else // !(defined(CFG_ISO_CON) && (CFG_ISO_CON > 0))
    #define BLE_ISO_PRESENT                (0)
    #define BLE_ISO_HW_PRESENT             (BLE_EMB_PRESENT && defined(CFG_BLE_ISO_HW_PRESENT))
#endif // (defined(CFG_ISO_CON) && (CFG_ISO_CON > 0))

//  *** Definition of supported isochronous mode ***

// Isochronous Mode 0 - Proprietary mode
#if  (BLE_ISO_PRESENT && defined(CFG_ISO_MODE_0))
    #define BLE_ISO_MODE_0                (BLE_CENTRAL | BLE_PERIPHERAL)
    #define BLE_ISO_MODE_0_PROFILE        (BLE_ISO_MODE_0 & BLE_HOST_PRESENT)
    #if defined(CFG_ISO_MODE_0_RSA)
        #define BLE_ISO_MODE_0_RSA            (BLE_ISO_MODE_0_PROFILE)
        #define BLE_RSA                       (BLE_ISO_MODE_0_PROFILE)
    #else
        #define BLE_ISO_MODE_0_RSA            (0)
        #define BLE_RSA                       (0)
    #endif // defined(CFG_ISO_MODE_0_RSA)
    #if defined(CFG_ISO_MODE_0_DEVICE)
        #define BLE_ISO_MODE_0_DEVICE         (BLE_ISO_MODE_0_PROFILE)
    #else
        #define BLE_ISO_MODE_0_DEVICE         (0)
    #endif // defined(CFG_ISO_MODE_0_RSA)
#else
    #define BLE_ISO_MODE_0                (0)
    #define BLE_ISO_MODE_0_PROFILE        (0)
    #define BLE_ISO_MODE_0_RSA            (0)
    #define BLE_ISO_MODE_0_DEVICE         (0)
#endif // (BLE_ISO_PRESENT && defined(CFG_AUDIO))

// Connected Isochronous Stream
#if  (BLE_ISO_PRESENT && defined(CFG_CIS))
    #define BLE_CIS                           (BLE_CENTRAL | BLE_PERIPHERAL)
    #if defined(CFG_CIS_IN_FLASH)
        #define BLE_CIS_IN_ROM               (0)
    #else
        #define BLE_CIS_IN_ROM               (BLE_CIS)
    #endif
#else
    #define BLE_CIS                           (0)
    #define BLE_CIS_IN_ROM                    (0)
#endif // (BLE_ISO_PRESENT && defined(CFG_CIS))

// Broadcast Isochronous Stream
#if  (BLE_ISO_PRESENT && defined(CFG_BIS))
    #define BLE_BIS                           (BLE_BROADCASTER | BLE_OBSERVER)
    #if defined(CFG_BIS_IN_FLASH)
        #define BLE_BIS_IN_ROM               (0)
    #else
        #define BLE_BIS_IN_ROM               (BLE_BIS)
    #endif
#else
    #define BLE_BIS                           (0)
    #define BLE_BIS_IN_ROM                    (0)
#endif // (BLE_ISO_PRESENT && defined(CFG_BIS))

// sanity check for ISO presence
#if !(BLE_ISO_MODE_0 | BLE_BIS | BLE_CIS)
    #undef  BLE_ISO_PRESENT
    #define BLE_ISO_PRESENT                   (0)
#endif // !(BLE_ISO_MODE_0 | BLE_BIS | BLE_CIS)

#if (BLE_ISO_PRESENT)
    /// Maximum number of ISO streams/groups
    #define BLE_ISO_STREAM_MAX       (CFG_ISO_CON)
    #define BLE_ISO_GROUP_MAX        (CFG_ISO_CON)

    /// Maximum number of octets that can be received/transmitted over Isochronous channels
    #define BLE_MAX_ISO_OCTETS  (251) // number of octets

    /// Define number of ISO TX/RX buffers per isochronous channel
    #if defined(CFG_SLIM)
        #define BLE_NB_ISO_BUFF_PER_STREAM          (5)
    #else // !(CFG_SLIM)
        #define BLE_NB_ISO_BUFF_PER_STREAM          (10)
    #endif // (CFG_SLIM)
    /// Define number of ISO descriptors per isochronous channel
    /// Must be equal to max(BLE_NB_ISODESC_PER_BIS, BLE_NB_RX_ISODESC_PER_CIS + BLE_NB_TX_ISODESC_PER_CIS)
    #define BLE_NB_ISODESC_PER_STREAM           (4)

    /// Number of ISO Descriptors - one descriptor required for update sub-event: 1 per stream
    #define BLE_ISO_DESC_NB                     ((BLE_ISO_STREAM_MAX * (BLE_NB_ISODESC_PER_STREAM)) + BLE_ISO_GROUP_MAX)
    /// Number of ISO buffers
    #define BLE_ISO_BUF_NB                         (BLE_ISO_STREAM_MAX * BLE_NB_ISO_BUFF_PER_STREAM)

    #if (BLE_CIS)
        /// Define number of ISO TX/RX descriptors and buffers per CIS
        #define BLE_NB_RXTX_ISO_DESC_BUF_PER_CIS       (2)
    #endif // (BLE_CIS)

    #if (BLE_BIS)
        /// Define number of ISO RX or TX descriptors per BIS
        #define BLE_NB_ISODESC_PER_BIS                 (4)
    #endif // (BLE_BIS)

    #if (BLE_CIS | BLE_BIS)
        /// Number of hopping sequence per channel
        #define BLE_ISO_HOP_SEQ_PER_CHAN   (2)
        #define BLE_ISO_HOP_SEQ_SIZE       (0x20) // Depends on max number of sub-event supported

        /// Number of RX ISO buffers (add one more buffer for fake reception/transmit)
        #define BLE_ISO_HOP_SEQ_NB         (BLE_ISO_STREAM_MAX * BLE_ISO_HOP_SEQ_PER_CHAN)
    #endif // (BLE_CIS | BLE_BIS)
#endif // (BLE_ISO_PRESENT)
/******************************************************************************************/
/* --------------------------   CHANNEL SCAN CONFIGURATION       -------------------------*/
/******************************************************************************************/

#if defined(CFG_BLE_CH_SCAN)
    #define BLE_CH_SCAN_SUPPORT (1)
#else
    #define BLE_CH_SCAN_SUPPORT (0)
#endif

#define RWIP_RSSI_INTERF_THR     (-70)

/// Maximum duration without receiving a packet before considering a channel quality unknown (in slots) (10 secs)
#define RWIP_ASSESS_VALID_TO       0x3E00

/// Maximum counter value for channel assessment
#define RWIP_ASSESS_COUNT_MAX      4
/// Counter threshold to consider a channel good
#define RWIP_ASSESS_COUNT_THR_GOOD 2
/// Counter threshold to consider a channel bad
#define RWIP_ASSESS_COUNT_THR_BAD  -2
/// Minimum counter value for channel assessment
#define RWIP_ASSESS_COUNT_MIN      -4

// Access address for the channel scan
#define RWIP_CH_SCAN_ACCESS_ADDR 0x555555555

/// Check status of Isochronous Data path drivers

/// Proprietary ISO over HCI
#if defined(CFG_ISOOHCI)
    #define BLE_ISOOHCI                  (BLE_ISO_PRESENT)
#else
    #define BLE_ISOOHCI                  (0)
#endif

/// Internal ISO generator for validation purpose
#if defined(CFG_ISOGEN)
    #define BLE_ISOGEN                   (BLE_ISO_PRESENT)
#else
    #define BLE_ISOGEN                   (0)
#endif

/// Bluebud TWS Data Path
#if defined(CFG_BLUEBUD_PRIMARY)
    #define BLE_BLUEBUD_TWS_TX           (BLE_ISO_PRESENT)
#else
    #define BLE_BLUEBUD_TWS_TX           (0)
#endif // defined(CFG_BLUEBUD_PRIMARY)

#if defined(CFG_BLUEBUD_SECONDARY)
    #define BLE_BLUEBUD_TWS_RX           (BLE_ISO_PRESENT)
#else
    #define BLE_BLUEBUD_TWS_RX           (0)
#endif // defined(CFG_BLUEBUD_SECONDARY)

#if (BLE_BLUEBUD_TWS_RX || BLE_BLUEBUD_TWS_TX)
    #define BLE_BLUEBUD_TWS              (BLE_ISO_PRESENT)
#else
    #define BLE_BLUEBUD_TWS              (0)
#endif // (BLE_BLUEBUD_TWS_RX || BLE_BLUEBUD_TWS_TX)

/// Platform PCM
#if defined(CFG_PCM)
    #define BLE_ISO_PCM                  (BLE_ISO_PRESENT)
#else // !defined(CFG_PCM)
    #define BLE_ISO_PCM                  (0)
#endif // defined(CFG_PCM)

/******************************************************************************************/
/* --------------------------   BUFFER SETUP       --------------------------------------*/
/******************************************************************************************/

/// Buffer management - used only by host for the moment
//#if (HOST_PRESENT)
// Yier, don't find where it used.
#if 0
    #define CO_BUF_PRESENT         1

    /// Size and number of big buffers available by default
    #define CO_BUF_BIG_SIZE        (GAP_LE_MTU_MAX)
    #define CO_BUF_BIG_NB          (2)
    #define CO_BUF_BIG_POOL_SIZE   ((CO_ALIGN4_HI(sizeof(co_buf_t) + CO_BUF_BIG_SIZE)) * CO_BUF_BIG_NB)


    /// Size and number of small buffers available by default
    #define CO_BUF_SMALL_SIZE      (128)
    #if(BLE_EMB_PRESENT)
        #define CO_BUF_SMALL_NB        (BLE_ACL_BUF_NB_TX)
    #else // !(BLE_EMB_PRESENT)
        #define CO_BUF_SMALL_NB        (12)
    #endif //(BLE_EMB_PRESENT)
    #define CO_BUF_SMALL_POOL_SIZE ((CO_ALIGN4_HI(sizeof(co_buf_t) + CO_BUF_SMALL_SIZE)) * CO_BUF_SMALL_NB)
#else
    #define CO_BUF_PRESENT         0
#endif //HOST_PRESENT

#if defined(CFG_ECC_P256_SUPPORT)
    #define ECC_P256_SUPPORT      (1)
#else
    #define ECC_P256_SUPPORT      (0)
#endif // defined(CFG_ECC_P256_SUPPORT)

#if defined(CFG_RSA_SUPPORT)
    #define RSA_SUPPORT           (1)
#else
    #define RSA_SUPPORT           (0)
#endif // defined(CFG_RSA_SUPPORT)

/******************************************************************************************/
/* --------------------------   DISPLAY SETUP        -------------------------------------*/
/******************************************************************************************/

/// Display controller enable/disable
#if defined(CFG_DISPLAY)
    #define DISPLAY_SUPPORT      1
#else
    #define DISPLAY_SUPPORT      0
#endif //CFG_DISPLAY

/******************************************************************************************/
/* --------------------------   GPIO SETUP        -------------------------------------*/
/******************************************************************************************/

/// Display controller enable/disable
#if defined(CFG_GPIO)
    #define GPIO_SUPPORT      1
#else
    #define GPIO_SUPPORT      0
#endif //CFG_GPIO

/******************************************************************************************/
/* --------------------------      RTC SETUP         -------------------------------------*/
/******************************************************************************************/

/// RTC enable/disable
#if defined(CFG_RTC)
    #define RTC_SUPPORT      1
#else
    #define RTC_SUPPORT      0
#endif //CFG_RTC

/******************************************************************************************/
/* --------------------------      PS2 SETUP         -------------------------------------*/
/******************************************************************************************/

/// PS2 enable/disable
#if defined(CFG_PS2)
    #define PS2_SUPPORT      1
#else
    #define PS2_SUPPORT      0
#endif //CFG_PS2

/******************************************************************************************/
/* --------------------------   AUDIO SYNC SETUP        ----------------------------------*/
/******************************************************************************************/

/// Audio Sync enable/disable
#if defined(CFG_AUDIO_SYNC)
    #define AUDIO_SYNC_SUPPORT      1
#else
    #define AUDIO_SYNC_SUPPORT      0
#endif //CFG_AUDIO_SYNC

/******************************************************************************************/
/* --------------------------      GAIA SETUP        -------------------------------------*/
/******************************************************************************************/

/// GAIA enable/disable
#if defined(CFG_GAIA)
    #define GAIA_SUPPORT      1
#else
    #define GAIA_SUPPORT      0
#endif //CFG_GAIA

/******************************************************************************************/
/* --------------------------      TRACER SETUP      -------------------------------------*/
/******************************************************************************************/

/// tracer enable/disable
#if defined(CFG_TRC_EN)
    #define TRACER_PRESENT                   1
    #include "dbg_trc_config.h"
#else
    #define TRACER_PRESENT                   0
#endif // CFG_TRC_EN

/******************************************************************************************/
/* -------------------------   DEEP SLEEP SETUP      -------------------------------------*/
/******************************************************************************************/

/// Low Power clock: 32kHz or 32.768kHz
#define HZ32000                                     (GAIA_SUPPORT)

/// Time to wake-up Radio Module (in us)
#define SLEEP_RM_WAKEUP_DELAY                       625
/// Time for stabilization of the high frequency oscillator following a sleep-timer expiry (in us)
#define SLEEP_OSC_NORMAL_WAKEUP_DELAY               5000
/// Time for stabilization of the high frequency oscillator following an external wake-up request (in us)
#define SLEEP_OSC_EXT_WAKEUP_DELAY                  5000

#if (HZ32000)
    /// Sleep Duration Value in periodic wake-up mode (in LP clock cycles)
    #define MAX_SLEEP_DURATION_PERIODIC_WAKEUP      16000   // 0.5s
    /// Sleep Duration Value in external wake-up mode (in LP clock cycles)
    #define MAX_SLEEP_DURATION_EXTERNAL_WAKEUP      320000  // 10s
#else // (HZ32000)
    /// Sleep Duration Value in periodic wake-up mode (in LP clock cycles)
    #define MAX_SLEEP_DURATION_PERIODIC_WAKEUP      16384   // 0.5s
    /// Sleep Duration Value in external wake-up mode (in LP clock cycles)
    #define MAX_SLEEP_DURATION_EXTERNAL_WAKEUP      327680  // 10s
#endif // (HZ32000)

/**
 * Margins for entering sleep and waking-up (in LP clock cycles)
 *  - 50 us + 2 LP cycles for sleep entering (UART check, system enters sleep)
 *  - 70 us for SW processing at wake-up
 *  - 312.5 us for clock correction at wake-up
 *  - Total ~ 500 us
 */
#define SLEEP_ENTER_FUNC_MARGIN         4       // 125 us at 32kHz, 122 us at 32.768kHz
#define SLEEP_WAKEUP_FUNC_MARGIN        3       // 93.75 us at 32kHz, 91.5 us at 32.768kHz
#define SLEEP_WAKEUP_CLOCK_CORR_MARGIN  10      // 312.5us at 32kHz, 305 us at 32.768kHz

/// Margin for SW to program a frame at wake-up (in half-us)
#define SLEEP_PROG_MARGIN   700      // 350 us


/******************************************************************************************/
/* --------------------------   BASEBAND SETUP       -------------------------------------*/
/******************************************************************************************/

#if (EMB_PRESENT)
    #include "rf.h"     // rf configurations, for BB & radio setup
    #include "arch.h"   // platform configurations, for BB setup
#endif // (EMB_PRESENT)

/// Default programming delay, margin for programming the baseband in advance of each activity (in half-slots)
#define IP_PROG_DELAY_DFT  (3)

/**
 * Prefetch time (in us)
 *  - Radio power up: radio defined (worst case)
 *  - EM fetch & HW logic: platform defined (worst case)
 *
 * Prefetch time = power up time + EM fetch & HW logic time
 */
#define RF_POWERUP_TIME_US 100
#define PLF_EM_FETCH_TIME_US 50
#define IP_PREFETCH_TIME_US         (RF_POWERUP_TIME_US + PLF_EM_FETCH_TIME_US)

/**
 * Prefetch Abort time (in us)
 *
 * - EM update & HW logic: platform defined (worst case)
 * - Radio power down: radio defined (worst case)
 *
 * Prefetch abort time = prefetch time + + EM update & HW logic time + power down time
 */
#define PLF_EM_UPDATE_TIME_US   20
#define RF_POWERDOWN_TIME_US    20
#define IP_PREFETCHABORT_TIME_US  (IP_PREFETCH_TIME_US + PLF_EM_UPDATE_TIME_US + RF_POWERDOWN_TIME_US)


/******************************************************************************************/
/* --------------------------   RADIO SETUP       ----------------------------------------*/
/******************************************************************************************/

/// Power control features
#define RF_TXPWR                            1
/// Class of device
#define RF_CLASS1                           0

/******************************************************************************************/
/* -------------------------   SUPPORTED RADIO PHY    ------------------------------------*/
/******************************************************************************************/

#define BLE_PHY_1MBPS_SUPPORT                       1
#define BLE_PHY_2MBPS_SUPPORT                       1
#define BLE_PHY_CODED_SUPPORT                       1
#define BLE_STABLE_MOD_IDX_TX_SUPPORT               0
#define BLE_STABLE_MOD_IDX_RX_SUPPORT               0
#define BLE_PWR_CLASS_1_SUPPORT                     1
/******************************************************************************************/
/* -------------------------   COEXISTENCE SETUP      ------------------------------------*/
/******************************************************************************************/

/// WLAN Coexistence
#if defined(CFG_WLAN_COEX)
    #define RW_WLAN_COEX                 1
    #define RW_WLAN_COEX_TEST            (defined(CFG_WLAN_COEX_TEST))
#else
    #define RW_WLAN_COEX                 0
    #define RW_WLAN_COEX_TEST            0
#endif // defined(CFG_WLAN_COEX)

/// MWS Coexistence
#if defined(CFG_MWS_COEX)
    #define RW_MWS_COEX                 1
    #define RW_MWS_COEX_TEST            (defined(CFG_MWS_COEX_TEST))
#else
    #define RW_MWS_COEX                 0
    #define RW_MWS_COEX_TEST            0
#endif // defined(CFG_MWS_COEX)


/******************************************************************************************/
/* ------------------------   RSSI & POWER CONTROL   -------------------------------------*/
/******************************************************************************************/

#if BLE_PWR_CTRL

    /// RSSI averaging weight (2^N)
    /*
    * Calculate an RSSI average based on a weighted history of RSSI values (RW proprietary method):
    * A weight of 2 (2^2 = 4) provides a weighted averaging at 25%, 18.5%, 14%, 10.5%, 8%, 6%, 4.5%, 3.3%, 2.5%...
    * A weight of 3 (2^3 = 8) provides a weighted averaging at 12.5%, 10.9%, 9.6%, 8.3%, 7.3%, 6.4%, 5.6%, 4.9%, 4.3%...
    */
    #define RW_RSSI_AV_WEIGHT 2

    /// RSSI APR target within golden receive window
    #define RW_RSSI_APR_TARGET           (10) // 10 dBm threshold

#endif // BLE_PWR_CTRL

#if (BT_53)
    /******************************************************************************************/
    /* ------------------------   AUTO-SUBRATE THRESHOLD   -----------------------------------*/
    /******************************************************************************************/

    // Subrate factor at which subrating is auto-disabled for procedures with instants
    #define RW_AUTO_SUBRATE_THRESHOLD   (4) // subrate threshold, in units of connection event

#endif // (BT_53)

/******************************************************************************************/
/* -----------------------   SLOT AVAILABILITY MASKS   -----------------------------------*/
/******************************************************************************************/

/// Maximum support peer SAM map size
#define RW_MAX_PEER_SAM_MAP_SLOTS      (256)
#define RW_PEER_SAM_MAP_MAX_LEN      (RW_MAX_PEER_SAM_MAP_SLOTS/4) // 2-bit field per slot

/******************************************************************************************/
/* --------------------   SECURE CONNECTIONS SETUP  --------------------------------------*/
/******************************************************************************************/
#if defined(CFG_SEC_CON)
    #define SECURE_CONNECTIONS        (1)
#endif
#if defined(CFG_ECC_16_BITS_ALGO)
    #define ECC_MULT_ALGO_TYPE        (16)
#else // !defined(CFG_ECC_16_BITS_ALGO)
    #define ECC_MULT_ALGO_TYPE        (32)
#endif // defined(CFG_ECC_16_BITS_ALGO)
#if defined(CFG_CRYPTO_UT)
    #define CRYPTO_UT                 (1)
#else //defined(CFG_CRYPTO_UT)
    #define CRYPTO_UT                 (0)
#endif //defined(CFG_CRYPTO_UT)

/******************************************************************************************/
/* --------------------------   DEBUG SETUP       ----------------------------------------*/
/******************************************************************************************/

/// Flag indicating if debug mode is activated or not
#if defined(CFG_DBG)
    #define RW_DEBUG                        (EMB_PRESENT || HOST_PRESENT)
    #if (EMB_PRESENT)
        #define RW_SWDIAG                       1
    #else
        #define RW_SWDIAG                       0
    #endif
    #define KE_PROFILING                    1
#else
    #define RW_DEBUG                        0
    #define RW_SWDIAG                       0
    #define KE_PROFILING                    0
#endif /* CFG_DBG */

/// Flag indicating if Read/Write memory commands are supported or not
#if defined(CFG_DBG_MEM)
    #define RW_DEBUG_MEM               1
#else //CFG_DBG_MEM
    #define RW_DEBUG_MEM               0
#endif //CFG_DBG_MEM

/// Flag indicating if Flash debug commands are supported or not
#if defined(CFG_DBG_FLASH)
    #define RW_DEBUG_FLASH                  1
#else //CFG_DBG_FLASH
    #define RW_DEBUG_FLASH                  0
#endif //CFG_DBG_FLASH

/// Flag indicating if CPU stack profiling commands are supported or not
#if defined(CFG_DBG_STACK_PROF)
    #define RW_DEBUG_STACK_PROF             1
#else
    #define RW_DEBUG_STACK_PROF             0
#endif // defined (CFG_DBG_STACK_PROF)

/// Scheduling Planner unit test (HCI debug commands to test scheduling planner functions)
#define SCH_PLAN_UT                 (1)

/// BLE I&Q sample Generator control interface
#define BLE_IQ_GEN                  (RW_DEBUG && (BLE_CON_CTE_REQ | BLE_CONLESS_CTE_RX))


/******************************************************************************************/
/* --------------------------      VS SETUP         --------------------------------------*/
/******************************************************************************************/

/// Read piconnect clock
#if defined (CFG_BT_READ_PICONET_CLOCK)
    #define BT_READ_PICONET_CLOCK   1
#else
    #define BT_READ_PICONET_CLOCK   0
#endif //defined (CFG_BT_READ_PICONET_CLOCK)

#define BT_HCI_TEST_MODE        1

/// I2C over HCI bridge
#if defined (CFG_BT_I2C_OVER_HCI)
    #define BT_I2C_OVER_HCI       1
#else
    #define BT_I2C_OVER_HCI       0
#endif // defined (CFG_BT_I2C_OVER_HCI)
/******************************************************************************************/
/* --------------------------      NVDS SETUP       --------------------------------------*/
/******************************************************************************************/

/// Flag indicating if NVDS feature is supported or not
#if defined(CFG_NVDS)
    #define NVDS_SUPPORT                    1
#else //CFG_DBG_NVDS
    #define NVDS_SUPPORT                    0
#endif //CFG_DBG_NVDS


/******************************************************************************************/
/* --------------------------     MEMORY SETUP      --------------------------------------*/
/******************************************************************************************/
/// Amount of memory that must be reserved to system allocation in case of memory overflow.
#if defined(CFG_SYSTEM_MEM_SAFETY_MARGIN_SIZE)
    #define SYSTEM_MEM_SAFETY_MARGIN_SIZE               (CFG_SYSTEM_MEM_SAFETY_MARGIN_SIZE)
#else // CFG_SYSTEM_MEM_SAFETY_MARGIN_SIZE
    #define SYSTEM_MEM_SAFETY_MARGIN_SIZE               (0)
#endif // CFG_SYSTEM_MEM_SAFETY_MARGIN_SIZE

/******************************************************************************************/
/* --------------------------      MISC SETUP       --------------------------------------*/
/******************************************************************************************/
/// Manufacturer: For Sifli
#define RW_COMP_ID                           0x0A4C



/******************************************************************************************/
/* -------------------------   BT / BLE / BLE HL CONFIG    -------------------------------*/
/******************************************************************************************/

#if (BT_EMB_PRESENT)
    #include "rwbt_config.h"    // bt stack configuration
#endif //BT_EMB_PRESENT

#if (BLE_EMB_PRESENT)
    #include "rwble_config.h"   // ble stack configuration
#endif //BLE_EMB_PRESENT

#if (HOST_PRESENT)
    #include "rwhl_config.h"  // ble Host stack configuration
#endif //HOST_PRESENT

#if defined(CFG_APP)
    #include "rwapp_config.h"     // Application configuration
#endif // defined(CFG_APP)


/******************************************************************************************/
/* -------------------------   KERNEL SETUP          -------------------------------------*/
/******************************************************************************************/

/// Force zero initialization of Kernel messages
/// (burden allocation but ensure that memory properly overwritten)
#if(defined(CFG_KE_MSG_CALLOC))
    #define KE_MSG_CALLOC                 (1)
#else
    #define KE_MSG_CALLOC                 (0)
#endif // (defined(CFG_KE_MSG_CALLOC))

/// Event types definition (ordered in reversed priority order)
/*@TRACE*/
enum KE_EVENT_TYPE
{
#if(APP_PRESENT)
    APP_EVENT,
#endif // (APP_PRESENT)

#if DISPLAY_SUPPORT
    KE_EVENT_DISPLAY,
#endif //DISPLAY_SUPPORT

#if RTC_SUPPORT
    KE_EVENT_RTC_1S_TICK,
#endif //RTC_SUPPORT

#if RSA_SUPPORT
    KE_EVENT_RSA_SIGN,
#endif //BLE_ISO_MODE_0_RSA

    /// ECDH background event
    KE_EVENT_ECC_P256_MULTIPLICATION,

#if BT_EMB_PRESENT
    KE_EVENT_ECC_P192_MULTIPLICATION,
#endif // BT_EMB_PRESENT

#if (TRACER_PRESENT)
    KE_EVENT_TRC,
#endif /*(TRACER_PRESENT)*/

    /// Delayed job handler - Low Priority
    KE_EVENT_DJOB_LP,

    /// Yier: Make High Job lower than KE message to keep ke could handle more messages
    /// Delayed job handler - High Priority
    KE_EVENT_DJOB_HP,

    /// Message handling
    KE_EVENT_KE_MESSAGE,


#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /// Handle AES result
    KE_EVENT_AES_END,
#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

    /// Timer handler
    KE_EVENT_TIMER,

#if (BLE_HOST_PRESENT)
#if (BLE_L2CC)
    KE_EVENT_L2CAP_TX,
#endif //(BLE_L2CC)
#endif// (BLE_HOST_PRESENT)

#if (BLE_ISOOHCI)
    KE_EVENT_ISOOHCI_IN_DEFER,
    KE_EVENT_ISOOHCI_OUT_DEFER,
#endif //(BLE_ISOOHCI)

    /// Delayed job handler - ISR Defer
    KE_EVENT_DJOB_ISR,

    KE_EVENT_MAX,
};

/// Tasks types definition
/*@TRACE*/
enum KE_TASK_TYPE
{
#if (BT_EMB_PRESENT)
    // BT Controller Tasks
    TASK_LM,
    TASK_LC,
    TASK_LB,
#endif // (BT_EMB_PRESENT)

#if (BLE_EMB_PRESENT)
    // Link Layer Tasks
    TASK_LLM,
    TASK_LLC,
#if (BLE_ISO_PRESENT)
    TASK_LLI,
#endif // (BLE_ISO_PRESENT)
#endif // (BLE_EMB_PRESENT)

#if ((BLE_EMB_PRESENT) || (BT_EMB_PRESENT))
    TASK_DBG,
#endif // ((BLE_EMB_PRESENT) || (BT_EMB_PRESENT))

#if (EAVESDROPPING_SUPPORT)
    TASK_ED,
#endif // EAVESDROPPING_SUPPORT

#if (DISPLAY_SUPPORT)
    TASK_DISPLAY,
#endif // (DISPLAY_SUPPORT)

#if (APP_PRESENT && HOST_MSG_API)
    TASK_APP,
#endif // (APP_PRESENT && HOST_MSG_API)

#if (BT_HOST_PRESENT)
    TASK_BT,
#endif // (APP_PRESENT && HOST_MSG_API)


#if (HOST_PRESENT && HOST_MSG_API)
    TASK_L2CC,   // L2CAP Task
    /* 5.2 use only one task, following 5.0's configration. '*/
    //TASK_GATT,    // Generic Attribute Profile
    TASK_GATTM,   // Generic Attribute Profile Manager Task
    TASK_GATTC,   // Generic Attribute Profile Controller Task
#if (BLE_CENTRAL || BLE_PERIPHERAL || BT_HOST_PRESENT)
    TASK_GAPC,    // Generic Access Profile Controller
#endif // (BLE_CENTRAL || BLE_PERIPHERAL || BT_HOST_PRESENT)
    TASK_GAPM,    // Generic Access Profile Manager
    TASK_PRF_MIN,
    // allocate a certain number of profiles task
    TASK_PRF_MAX = (TASK_PRF_MIN + (HOST_NB_PROFILES - 1)),
#endif // (HOST_PRESENT && HOST_MSG_API)

#if (AHI_TL_SUPPORT)
    TASK_AHI,
#endif // (AHI_TL_SUPPORT)

#if (AHI_INT_SUPPORT)
    TASK_AHI_INT,
#endif // (AHI_TL_SUPPORT)

#if (BLE_GAF_PRESENT)
    TASK_GAF,
#endif //(BLE_GAF_PRESENT)

#if (BLE_ISO_PRESENT)
    TASK_GAPI,
#endif //(BLE_ISO_PRESENT)

#if ((EMB_PRESENT) || (HOST_PRESENT))
    TASK_COMMON,
#endif


    /// Maximum number of tasks
    TASK_MAX,

    TASK_NONE = 0xFF,
};

/// Kernel memory heaps types.
/*@TRACE*/
enum KE_MEM_HEAP
{
    /// Memory allocated for environment variables
    KE_MEM_ENV,
#if (HOST_PRESENT)
    /// Memory allocated for profiles
    KE_MEM_ATT_DB,
#endif // (HOST_PRESENT)
    /// Memory allocated for kernel messages
    KE_MEM_KE_MSG,
    /// Non Retention memory block
    KE_MEM_NON_RETENTION,
    KE_MEM_BLOCK_MAX,
};



#if (BT_EMB_PRESENT)
    #define BT_HEAP_MSG_SIZE_       (BT_HEAP_MSG_SIZE)
    #define BT_HEAP_ENV_SIZE_       (BT_HEAP_ENV_SIZE)
#else
    #define BT_HEAP_MSG_SIZE_       (0)
    #define BT_HEAP_ENV_SIZE_       (0)
#endif //BT_EMB_PRESENT

#if (BLE_EMB_PRESENT)
    #define BLE_HEAP_MSG_SIZE_      (BLE_HEAP_MSG_SIZE)
    #define BLE_HEAP_ENV_SIZE_      (BLE_HEAP_ENV_SIZE)
#else
    #define BLE_HEAP_MSG_SIZE_      (0)
    #define BLE_HEAP_ENV_SIZE_      (0)
#endif //BLE_EMB_PRESENT

#if (HOST_PRESENT)
    #define HOST_HEAP_MSG_SIZE_     (HOST_HEAP_MSG_SIZE)
    #define HOST_HEAP_ENV_SIZE_     (HOST_HEAP_ENV_SIZE)
    #define HOST_HEAP_PROFILE_SIZE_ (HOST_HEAP_PROFILE_SIZE)
#else
    #define HOST_HEAP_MSG_SIZE_     (0)
    #define HOST_HEAP_ENV_SIZE_     (0)
    #define HOST_HEAP_PROFILE_SIZE_ (0)
#endif // (HOST_PRESENT)

#if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    #define ECC_HEAP_NON_RET_SIZE_   (328*2) // Could only have 2 ECC computations simultaneously
#else // (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    #define ECC_HEAP_NON_RET_SIZE_   (0)
#endif // (BT_EMB_PRESENT || BLE_EMB_PRESENT)

/// Kernel Message Heap
#define RWIP_HEAP_MSG_SIZE         (  BT_HEAP_MSG_SIZE_      + \
                                      BLE_HEAP_MSG_SIZE_     + \
                                      HOST_HEAP_MSG_SIZE_      )

/// Size of Environment heap
#define RWIP_HEAP_ENV_SIZE         ( BT_HEAP_ENV_SIZE_       + \
                                     BLE_HEAP_ENV_SIZE_      + \
                                     HOST_HEAP_ENV_SIZE_       )


/// Size of Attribute database heap
#define RWIP_HEAP_PROFILE_SIZE     (  HOST_HEAP_PROFILE_SIZE_  )

/**
 * Size of non-retention heap
 *
 * This heap can be used to split the RAM into 2 parts:
 *    - an always-on part that can handle a certain number of links
 *    - a secondary memory that could be powered-off when not used, and retained only when used
 *
 * With such mechanism, the previous heaps need to be reduced so that they can contain all required data
 * in a light scenario (few connections, few profiles). Then the non-retention heap is sized in order to
 * cover the worst case scenario (max connections, max profiles, etc ...)
 *
 * The current size show what is already known as not needing to be retained during deep sleep.
 */
#define RWIP_HEAP_NON_RET_SIZE    ( ECC_HEAP_NON_RET_SIZE_ )

/******************************************************************************************/
/* -------------------------     CONFIGURABLE PARAMETERS     -----------------------------*/
/******************************************************************************************/

/// List of parameters identifiers
enum PARAM_ID
{
    /// Definition of the tag associated to each parameters
    /// Local Bd Address
    PARAM_ID_BD_ADDRESS                 = 0x01,
    /// Device Name
    PARAM_ID_DEVICE_NAME                = 0x02,
    /// Low Power Clock Drift
    PARAM_ID_LPCLK_DRIFT                = 0x07,
    /// Active Clock Drift
    PARAM_ID_ACTCLK_DRIFT               = 0x09,
    /// External wake-up time
    PARAM_ID_EXT_WAKEUP_TIME            = 0x0D,
    /// Oscillator wake-up time
    PARAM_ID_OSC_WAKEUP_TIME            = 0x0E,
    /// Radio wake-up time
    PARAM_ID_RM_WAKEUP_TIME             = 0x0F,
    /// UART baudrate
    PARAM_ID_UART_BAUDRATE              = 0x10,
    /// Enable sleep mode
    PARAM_ID_SLEEP_ENABLE               = 0x11,
    /// Enable External Wakeup
    PARAM_ID_EXT_WAKEUP_ENABLE          = 0x12,

    /// Activity Move Configuration (enables/disables activity move for BLE connections and BT (e)SCO links)
    PARAM_ID_ACTIVITY_MOVE_CONFIG       = 0x15,

    /// Enable/disable scanning for extended advertising PDUs
    PARAM_ID_SCAN_EXT_ADV               = 0x16,

    /// Duration of the schedule reservation for long activities such as scan, inquiry, page, HDC advertising
    PARAM_ID_SCHED_SCAN_DUR             = 0x17,

    /// Programming delay, margin for programming the baseband in advance of each activity (in half-slots)
    PARAM_ID_PROG_DELAY                 = 0x18,

    /// Enable/disable channel assessment for BT and/or BLE
    PARAM_ID_CH_ASS_EN                  = 0x19,

    /// Default MD bit used by slave when sending a data packet on a BLE connection
    PARAM_ID_DFT_SLAVE_MD               = 0x20,

    /// Synchronous links configuration
    PARAM_ID_SYNC_CONFIG                = 0x2C,
    /// PCM Settings
    PARAM_ID_PCM_SETTINGS               = 0x2D,
    /// Tracer configuration
    PARAM_ID_TRACER_CONFIG              = 0x2F,

    /// Diagport configuration
    PARAM_ID_DIAG_BT_HW                 = 0x30,
    PARAM_ID_DIAG_BLE_HW                = 0x31,
    PARAM_ID_DIAG_SW                    = 0x32,
    PARAM_ID_DIAG_DM_HW                 = 0x33,
    PARAM_ID_DIAG_PLF                   = 0x34,

    /// Channel classification reporting interval (in ms) (0 if not used) (default value: 0ms)
    PARAM_ID_CH_CLASS_REP_INTV           = 0x36,

    /// IDC selection (for audio demo)
    PARAM_ID_IDCSEL_PLF                 = 0x37,

    /// RSSI threshold tags
    PARAM_ID_RSSI_HIGH_THR              = 0x3A,
    PARAM_ID_RSSI_LOW_THR               = 0x3B,
    PARAM_ID_RSSI_INTERF_THR            = 0x3C,

    /// RF identifier
    PARAM_ID_RF_ID                      = 0x3D,

    /// RF BTIPT
    PARAM_ID_RF_BTIPT_VERSION          = 0x3E,
    PARAM_ID_RF_BTIPT_XO_SETTING       = 0x3F,
    PARAM_ID_RF_BTIPT_GAIN_SETTING     = 0x40,

    /// Link keys
    PARAM_ID_BT_LINK_KEY_NB             = 0x60,
    PARAM_ID_BT_LINK_KEY_FIRST          = 0x61,
    PARAM_ID_BLE_LINK_KEY_FIRST         = 0x70,
    PARAM_ID_BLE_LINK_KEY_LAST          = 0x7F,
    /// SC Private Key (Low Energy)
    PARAM_ID_LE_PRIVATE_KEY_P256        = 0x80,
    /// SC Public Key (Low Energy)
    PARAM_ID_LE_PUBLIC_KEY_P256         = 0x81,
    /// SC Debug: Used Fixed Private Key from NVDS (Low Energy)
    PARAM_ID_LE_DBG_FIXED_P256_KEY      = 0x82,

    /// LE Coded PHY 500 Kbps selection
    PARAM_ID_LE_CODED_PHY_500           = 0x85,

    /// Application specific
    PARAM_ID_APP_SPECIFIC_FIRST         = 0x90,
    PARAM_ID_APP_SPECIFIC_LAST          = 0xAF,
};

/// List of parameters lengths
enum PARAM_LEN
{
    // Definition of length associated to each parameters
    /// Local Bd Address
    PARAM_LEN_BD_ADDRESS                 = 6,
    /// Device Name
    PARAM_LEN_DEVICE_NAME                = 248,
    /// Low power clock drift
    PARAM_LEN_LPCLK_DRIFT                = 2,
    /// Active clock drift
    PARAM_LEN_ACTCLK_DRIFT               = 1,
    /// External wake-up time
    PARAM_LEN_EXT_WAKEUP_TIME            = 2,
    /// Oscillator wake-up time
    PARAM_LEN_OSC_WAKEUP_TIME            = 2,
    /// Radio wake-up time
    PARAM_LEN_RM_WAKEUP_TIME             = 2,
    /// UART baudrate
    PARAM_LEN_UART_BAUDRATE              = 4,
    /// Enable sleep mode
    PARAM_LEN_SLEEP_ENABLE               = 1,
    /// Enable External Wakeup
    PARAM_LEN_EXT_WAKEUP_ENABLE          = 1,

    /// Activity Move Configuration
    PARAM_LEN_ACTIVITY_MOVE_CONFIG       = 1,

    /// Enable/disable scanning for extended advertising PDUs
    PARAM_LEN_SCAN_EXT_ADV               = 1,

    /// Duration of the schedule reservation for long activities such as scan, inquiry, page, HDC advertising
    PARAM_LEN_SCHED_SCAN_DUR             = 2,

    /// Programming delay, margin for programming the baseband in advance of each activity (in half-slots)
    PARAM_LEN_PROG_DELAY                 = 1,

    /// Enable/disable channel assessment for BT and/or BLE
    PARAM_LEN_CH_ASS_EN                  = 1,

    /// Default MD bit used by slave when sending a data packet on a BLE connection
    PARAM_LEN_DFT_SLAVE_MD               = 1,

    /// Synchronous links configuration
    PARAM_LEN_SYNC_CONFIG                = 2,
    /// PCM Settings
    PARAM_LEN_PCM_SETTINGS               = 8,
    /// Tracer configuration
    PARAM_LEN_TRACER_CONFIG              = 4,

    /// Diagport configuration
    PARAM_LEN_DIAG_BT_HW                 = 4,
    PARAM_LEN_DIAG_BLE_HW                = 4,
    PARAM_LEN_DIAG_SW                    = 4,
    PARAM_LEN_DIAG_DM_HW                 = 4,
    PARAM_LEN_DIAG_PLF                   = 4,

    /// Channel classification reporting interval (in ms) (0 if not used) (default value: 0ms)
    PARAM_LEN_CH_CLASS_REP_INTV          = 2,
    /// IDC selection (for audio demo)
    PARAM_LEN_IDCSEL_PLF                 = 4,

    /// RSSI thresholds
    PARAM_LEN_RSSI_THR                   = 1,

    /// RF identifier
    PARAM_LEN_RF_ID                     = 4,

    /// RF BTIPT
    PARAM_LEN_RF_BTIPT_VERSION          = 1,
    PARAM_LEN_RF_BTIPT_XO_SETTING       = 1,
    PARAM_LEN_RF_BTIPT_GAIN_SETTING     = 2,

    /// Link keys
    PARAM_LEN_BT_LINK_KEY_NB             = 1,
    PARAM_LEN_BT_LINK_KEY                = 22,
    PARAM_LEN_BLE_LINK_KEY               = 48,

    /// P256
    PARAM_LEN_PRIVATE_KEY_P256           = 32,
    PARAM_LEN_PUBLIC_KEY_P256            = 64,
    PARAM_LEN_DBG_FIXED_P256_KEY         = 1,

    /// LE Coded PHY 500 Kbps selection
    PARAM_LEN_LE_CODED_PHY_500           = 1,
};

/******************************************************************************************/
/* -------------------------        BT-BLE COEX        -----------------------------------*/
/******************************************************************************************/

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
/// Enable and diable definition for the PTI
///Enable TX busy signal
#define RWIP_PTI_TXEN           1
///Disable TX busy signal
#define RWIP_PTI_TXDIS          0
/// Tx busy position
#define RWIP_TXBSY_POS          0

///Enable RX busy signal
#define RWIP_PTI_RXEN           1
///Disable RX busy signal
#define RWIP_PTI_RXDIS          0
/// Rx busy position
#define RWIP_RXBSY_POS          1

///Enable do not abort TX
#define RWIP_PTI_DNABORTEN      1
///Disable do not abort TX
#define RWIP_PTI_DNABORTDIS     0
/// Do not abort busy position
#define RWIP_DNABORT_POS        2

/// SAM disabled
#define RWIP_SAM_DIS            0
/// SAM enabled
#define RWIP_SAM_EN             1
/// SAM enable position
#define RWIP_SAMEN_POS          3

/// Bit masking
#define RWIP_COEX_BIT_MASK      1

/// Coex configuration index
enum rwip_coex_config_idx
{
#if (BT_EMB_PRESENT)
    RWIP_COEX_MSSWITCH_IDX,
    RWIP_COEX_SNIFFATT_IDX,
    RWIP_COEX_PAGE_IDX,
    RWIP_COEX_PSCAN_IDX,
    RWIP_COEX_INQ_IDX,
    RWIP_COEX_INQRES_IDX,
    RWIP_COEX_SCORSVD_IDX,
    RWIP_COEX_BCAST_IDX,
    RWIP_COEX_CONNECT_IDX,
#endif //#if (BT_EMB_PRESENT)
#if (BLE_EMB_PRESENT)
    RWIP_COEX_CON_IDX,
    RWIP_COEX_CON_DATA_IDX,
    RWIP_COEX_ADV_IDX,
    RWIP_COEX_SCAN_IDX,
    RWIP_COEX_INIT_IDX,
    RWIP_COEX_CH_SCAN_IDX,
#endif // #if (BLE_EMB_PRESENT)
    /// Max configuration index
    RWIP_COEX_CFG_MAX,
};
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)

/******************************************************************************************/
/* -------------------------     BT-BLE PRIORITIES     -----------------------------------*/
/******************************************************************************************/
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
/// Priority index definition
enum rwip_prio_idx
{
#if (BT_EMB_PRESENT)
    /// ACL event default priority
    RWIP_PRIO_ACL_DFT_IDX,
    /// ACL event priority with activity
    RWIP_PRIO_ACL_ACT_IDX,
    /// ACL Role Switch event default priority
    RWIP_PRIO_ACL_RSW_IDX,
    /// ACL sniff event default priority
    RWIP_PRIO_ACL_SNIFF_DFT_IDX,
    /// ACL sniff transition event default priority
    RWIP_PRIO_ACL_SNIFF_TRANS_IDX,
#if MAX_NB_SYNC
    /// SCO event default priority
    RWIP_PRIO_SCO_DFT_IDX,
#endif //MAX_NB_SYNC
    /// Broadcast ACL event default priority
    RWIP_PRIO_BCST_DFT_IDX,
    /// Broadcast ACL event with LMP activity priority
    RWIP_PRIO_BCST_ACT_IDX,
    /// CSB RX event default priority
    RWIP_PRIO_CSB_RX_DFT_IDX,
    /// CSB TX event default priority
    RWIP_PRIO_CSB_TX_DFT_IDX,
    /// Inquiry event default priority
    RWIP_PRIO_INQ_DFT_IDX,
    /// Inquiry Scan event default priority
    RWIP_PRIO_ISCAN_DFT_IDX,
    /// Page event default priority
    RWIP_PRIO_PAGE_DFT_IDX,
    /// Page event default priority
    RWIP_PRIO_PAGE_1ST_PKT_IDX,
    /// Page first packet event default priority
    RWIP_PRIO_PCA_DFT_IDX,
    /// Page scan event default priority
    RWIP_PRIO_PSCAN_DFT_IDX,
    /// Page scan event priority increment when canceled
    RWIP_PRIO_PSCAN_1ST_PKT_IDX,
    /// Synchronization Scan event default priority
    RWIP_PRIO_SSCAN_DFT_IDX,
    /// Synchronization Train event default priority
    RWIP_PRIO_STRAIN_DFT_IDX,
#endif //#if (BT_EMB_PRESENT)
#if (BLE_EMB_PRESENT)
    /// Default priority for scanning events
    RWIP_PRIO_SCAN_IDX,
    /// Default priority for auxillary scan/init (no_asap) rx events
    RWIP_PRIO_AUX_RX_IDX,
    /// Default priority for periodic adv rx events
    RWIP_PRIO_PER_ADV_RX_DFT_IDX,
    /// Priority for periodic adv rx events before sync is established
    RWIP_PRIO_PER_ADV_RX_ESTAB_IDX,
    /// Default priority for initiating events
    RWIP_PRIO_INIT_IDX,
    /// LE connection events default priority
    RWIP_PRIO_CONNECT_DFT_IDX,
    /// LE connection events priority with activity
    RWIP_PRIO_CONNECT_ACT_IDX,
    /// LE connection events priority before the connection is established
    RWIP_PRIO_CONNECT_ESTAB_IDX,
    /// LE connection events priority before instant
    RWIP_PRIO_CONNECT_INSTANT_IDX,
    /// Default priority for advertising events
    RWIP_PRIO_ADV_IDX,
    /// Default priority for advertising high duty cycle events
    RWIP_PRIO_ADV_HDC_IDX,
    /// Default priority for aux advertising events
    RWIP_PRIO_ADV_AUX_IDX,
    /// Default priority for periodic advertising events
    RWIP_PRIO_PER_ADV_IDX,
    /// Default priority for resolvable private addresses renewal event
    RWIP_PRIO_RPA_RENEW_IDX,
#if (BLE_CIS)
    /// Default priority for master CIS connect events
    RWIP_PRIO_M_CIS_IDX,
    /// Default priority for slave CIS connect events
    RWIP_PRIO_S_CIS_IDX,
#endif // (BLE_CIS)
#if (BLE_BIS)
    /// Default priority for master BIS events
    RWIP_PRIO_M_BIS_IDX,
    /// Default priority for slave BIS events
    RWIP_PRIO_S_BIS_IDX,
#endif // (BLE_BIS)
#if (BLE_CH_SCAN_SUPPORT)
    /// Priority for Channel Scan activity
    RWIP_PRIO_CH_SCAN_IDX,
#endif //(BLE_CH_SCAN_SUPPORT)
#endif // #if (BLE_EMB_PRESENT)
    RWIP_PRIO_IDX_MAX
};
/// Default priority value definition
enum rwip_prio_dft
{
#if (BT_EMB_PRESENT)
    /// ACL event default priority
    RWIP_PRIO_ACL_DFT               = 40,
    /// ACL event priority with activity
    RWIP_PRIO_ACL_ACT               = 80,
    /// ACL Role Switch event default priority
    RWIP_PRIO_ACL_RSW               = 160,
    /// ACL sniff event default priority
    RWIP_PRIO_ACL_SNIFF_DFT         = 120,
    /// ACL sniff transition event default priority
    RWIP_PRIO_ACL_SNIFF_TRANS       = 80,
#if MAX_NB_SYNC
    /// SCO event default priority
    RWIP_PRIO_SCO_DFT               = 144,
#endif //MAX_NB_SYNC
    /// Broadcast ACL event default priority
    RWIP_PRIO_BCST_DFT              = 40,
    /// Broadcast ACL event with LMP activity priority
    RWIP_PRIO_BCST_ACT              = 80,
    /// CSB RX event default priority
    RWIP_PRIO_CSB_RX_DFT            = 80,
    /// CSB TX event default priority
    RWIP_PRIO_CSB_TX_DFT            = 80,
    /// Inquiry event default priority
    RWIP_PRIO_INQ_DFT               = 40,
    /// Inquiry Scan event default priority
    RWIP_PRIO_ISCAN_DFT             = 40,
    /// Page event default priority
    RWIP_PRIO_PAGE_DFT              = 64,
    /// Page first packet event default priority
    RWIP_PRIO_PAGE_1ST_PKT          = 160,
    /// PCA event default priority
    RWIP_PRIO_PCA_DFT               = 160,
    /// Page scan event default priority
    RWIP_PRIO_PSCAN_DFT             = 64,
    /// Page scan event priority increment when canceled
    RWIP_PRIO_PSCAN_1ST_PKT         = 160,
    /// Synchronization Scan event default priority
    RWIP_PRIO_SSCAN_DFT             = 80,
    /// Synchronization Train event default priority
    RWIP_PRIO_STRAIN_DFT            = 40,
#endif //#if (BT_EMB_PRESENT)
#if (BLE_EMB_PRESENT)
    /// Default priority for scanning events
    RWIP_PRIO_SCAN_DFT              = 40,
    /// Default priority for auxillary scan/init (no_asap) rx events
    RWIP_PRIO_AUX_RX_DFT            = 96,
    /// Default priority for periodic adv rx events
    RWIP_PRIO_PER_ADV_RX_DFT        = 100, /* Must be higher than AUX_RX_DFT (LL/DDI/SCN/BV-47-C) */
    /// Priority for periodic adv rx events before sync is established
    RWIP_PRIO_PER_ADV_RX_ESTAB      = 160,
    /// Default priority for initiating events
    RWIP_PRIO_INIT_DFT              = 80,
    /// LE connection events default priority
    RWIP_PRIO_CONNECT_DFT           = 112,
    /// LE connection events priority with activity
    RWIP_PRIO_CONNECT_ACT           = 128,
    /// LE connection events priority before the connection is established
    RWIP_PRIO_CONNECT_ESTAB         = 160,
    /// LE connection events priority before instant
    RWIP_PRIO_CONNECT_INSTANT       = 160,
    /// Default priority for advertising events
    RWIP_PRIO_ADV_DFT               = 40,
    /// Default priority for advertising high duty cycle events
    RWIP_PRIO_ADV_HDC_DFT           = 80,
    /// Default priority for aux advertising events
    RWIP_PRIO_ADV_AUX_DFT           = 96,
    /// Default priority for periodic advertising events
    RWIP_PRIO_PER_ADV_DFT           = 80,
    /// Default priority for resolvable private addresses renewal event
    RWIP_PRIO_RPA_RENEW_DFT         = 80,
#if (BLE_CIS)
    /// Default priority for Master CIS Connect
    RWIP_PRIO_M_CIS_DFT             = 160,
    /// Default priority for Slave CIS Connect
    RWIP_PRIO_S_CIS_DFT             = 160,
#endif // (BLE_CIS)
#if (BLE_BIS)
    /// Default priority for Master BIS
    RWIP_PRIO_M_BIS_DFT             = 160,
    /// Default priority for Slave BIS
    RWIP_PRIO_S_BIS_DFT             = 160,
#endif // (BLE_BIS)
#if (BLE_CH_SCAN_SUPPORT)
    /// Default priority for Channel Scan activity
    RWIP_PRIO_CH_SCAN_DFT           = 40,
#endif // (BLE_CH_SCAN_SUPPORT)
#endif // #if (BLE_EMB_PRESENT)
    /// Max priority
    RWIP_PRIO_MAX                   = 255,
};
/// Default increment value definition
enum rwip_incr_dft
{
#if (BT_EMB_PRESENT)
    /// ACL event default increment
    RWIP_INCR_ACL_DFT               = 8,
    /// ACL event increment with activity
    RWIP_INCR_ACL_ACT               = 8,
    /// ACL Role Switch event default increment
    RWIP_INCR_ACL_RSW               = 8,
    /// ACL sniff event default increment
    RWIP_INCR_ACL_SNIFF_DFT         = 8,
    /// ACL sniff transition event default increment
    RWIP_INCR_ACL_SNIFF_TRANS       = 8,
#if MAX_NB_SYNC
    /// SCO event default increment
    RWIP_INCR_SCO_DFT               = 8,
#endif //MAX_NB_SYNC
    /// Broadcast ACL event default increment
    RWIP_INCR_BCST_DFT              = 8,
    /// Broadcast ACL event with LMP activity increment
    RWIP_INCR_BCST_ACT              = 8,
    /// CSB RX event default increment
    RWIP_INCR_CSB_RX_DFT            = 8,
    /// CSB TX event default increment
    RWIP_INCR_CSB_TX_DFT            = 8,
    /// Inquiry event default increment
    RWIP_INCR_INQ_DFT               = 8,
    /// Inquiry Scan event default increment
    RWIP_INCR_ISCAN_DFT             = 8,
    /// Page event default increment
    RWIP_INCR_PAGE_DFT              = 8,
    /// Page event default increment
    RWIP_INCR_PAGE_1ST_PKT          = 12,
    /// Page first packet event default increment
    RWIP_INCR_PCA_DFT               = 8,
    /// Page scan event default increment
    RWIP_INCR_PSCAN_DFT             = 8,
    /// Page scan event increment increment when canceled
    RWIP_INCR_PSCAN_1ST_PKT         = 8,
    /// Synchronization Scan event default increment
    RWIP_INCR_SSCAN_DFT             = 8,
    /// Synchronization Train event default increment
    RWIP_INCR_STRAIN_DFT            = 8,
#endif //#if (BT_EMB_PRESENT)
#if (BLE_EMB_PRESENT)
    /// Default increment for scanning events
    RWIP_INCR_SCAN_DFT              = 8,
    /// Default increment for auxillary scan/init (no_asap) rx events
    RWIP_INCR_AUX_RX_DFT            = 8,
    /// Default increment for periodic adv rx events
    RWIP_INCR_PER_ADV_RX_DFT        = 8,
    /// Increment for periodic adv rx events before sync is established
    RWIP_INCR_PER_ADV_RX_ESTAB      = 8,
    /// Default increment for initiating events
    RWIP_INCR_INIT_DFT              = 8,
    /// LE connection events default increment
    RWIP_INCR_CONNECT_DFT           = 8,
    /// LE connection events increment with activity
    RWIP_INCR_CONNECT_ACT           = 8,
    /// LE connection events increment before the connection is established
    RWIP_INCR_CONNECT_ESTAB         = 8,
    /// LE connection events increment before instant
    RWIP_INCR_CONNECT_INSTANT       = 8,
    /// Default increment for advertising events
    RWIP_INCR_ADV_DFT               = 8,
    /// Default increment for advertising high duty cycle events
    RWIP_INCR_ADV_HDC_PRIO_DFT      = 8,
    /// Default increment for aux advertising events
    RWIP_INCR_ADV_AUX_DFT           = 8,
    /// Default increment for periodic advertising events
    RWIP_INCR_PER_ADV_DFT           = 8,
    /// Default increment for resolvable private addresses renewal event
    RWIP_INCR_RPA_RENEW_DFT         = 8,
#if (BLE_CIS)
    /// Default priority for Master CIS Connect
    RWIP_INCR_M_CIS_DFT             = 8,
    /// Default priority for Slave CIS Connect
    RWIP_INCR_S_CIS_DFT             = 8,
#endif // (BLE_CIS)
#if (BLE_BIS)
    /// Default priority for Master BIS
    RWIP_INCR_M_BIS_DFT             = 8,
    /// Default priority for Slave BIS
    RWIP_INCR_S_BIS_DFT             = 8,
#endif // (BLE_BIS)
    /// Default priority for Channel Scan activity
    RWIP_INCR_CH_SCAN_DFT           = 0,
#endif // #if (BLE_EMB_PRESENT)
};
#endif //#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
/// @} BT Stack Configuration
/// @} ROOT

/******************************************************************************************/
/* -------------------------     ble bqb test     -----------------------------------*/
/******************************************************************************************/
//#define CFG_BQB
//#define TP_LE_CPU_BV_01_C
//#define TP_LE_CID_BI_01_C

#endif //RWIP_CONFIG_H_
