/**
 ****************************************************************************************
 *
 * @file rwble_config.h
 *
 * @brief Configuration of the BLE lower layer stack
 *
 * Copyright (C) RivieraWaves 2009-2019
 *
 ****************************************************************************************
 */

#ifndef RWBLE_CONFIG_H_
#define RWBLE_CONFIG_H_

/**
 ****************************************************************************************
 * @addtogroup ROOT
 * @{
 * @name BLE LL stack configuration
 * @{
 ****************************************************************************************
 */
/******************************************************************************************/
/* --------------------------   FEATURE SETUP       --------------------------------------*/
/******************************************************************************************/

/// Define enable bits based on build flags - defined explicitly for extraction script operation
#if (BLE_PAST)
    #define BLE_FEAT_PAST_EN        (1)
#else // !(BLE_PAST)
    #define BLE_FEAT_PAST_EN        (0)
#endif // !(BLE_PAST)

#if (BLE_PWR_CTRL)
    #define BLE_FEAT_PWR_CTRL_EN    (1)
#else // !(BLE_PWR_CTRL)
    #define BLE_FEAT_PWR_CTRL_EN    (0)
#endif // !(BLE_PWR_CTRL)

#if (BLE_CIS)
    #define BLE_FEAT_CIS_EN         (1)
#else // !(BLE_CIS)
    #define BLE_FEAT_CIS_EN         (0)
#endif // !(BLE_CIS)

#if (BLE_BIS)
    #define BLE_FEAT_BIS_EN         (1)
#else // !(BLE_BIS)
    #define BLE_FEAT_BIS_EN         (0)
#endif // !(BLE_BIS)

/// Features byte 0
#define BLE_FEATURES_BYTE0  (  (1 << BLE_FEAT_ENC                   ) \
                             | (1 << BLE_FEAT_CON_PARAM_REQ_PROC    ) \
                             | (1 << BLE_FEAT_EXT_REJ_IND           ) \
                             | (1 << BLE_FEAT_SLAVE_INIT_FEAT_EXCHG ) \
                             | (1 << BLE_FEAT_PING                  ) \
                             | (1 << BLE_FEAT_DATA_PKT_LEN_EXT      ) \
                             | (1 << BLE_FEAT_LL_PRIVACY            ) \
                             | (1 << BLE_FEAT_EXT_SCAN_FILT_POLICY  ) )

/// Features byte 1
#define BLE_FEATURES_BYTE1  (  (BLE_PHY_2MBPS_SUPPORT         << (BLE_FEAT_2M_PHY            - 8) ) \
                             | (BLE_STABLE_MOD_IDX_TX_SUPPORT << (BLE_FEAT_STABLE_MOD_IDX_TX - 8) ) \
                             | (BLE_STABLE_MOD_IDX_RX_SUPPORT << (BLE_FEAT_STABLE_MOD_IDX_RX - 8) ) \
                             | (BLE_PHY_CODED_SUPPORT         << (BLE_FEAT_CODED_PHY         - 8) ) \
                             | (1                             << (BLE_FEAT_EXT_ADV           - 8) ) \
                             | (1                             << (BLE_FEAT_PER_ADV           - 8) ) \
                             | (1                             << (BLE_FEAT_CHAN_SEL_ALGO_2   - 8) ) \
                             | (BLE_PWR_CLASS_1_SUPPORT       << (BLE_FEAT_PWR_CLASS_1       - 8) ) )

/// Features byte 2
#define BLE_FEATURES_BYTE2  (  (1                                      << (BLE_FEAT_MIN_NUM_USED_CHAN_PROC - 16) ) \
                             | (BLE_CON_CTE_REQ                        << (BLE_FEAT_CON_CTE_REQ            - 16) ) \
                             | (BLE_CON_CTE_RSP                        << (BLE_FEAT_CON_CTE_RSP            - 16) ) \
                             | (BLE_CONLESS_CTE_TX                     << (BLE_FEAT_CONLESS_CTE_TX         - 16) ) \
                             | (BLE_CONLESS_CTE_RX                     << (BLE_FEAT_CONLESS_CTE_RX         - 16) ) \
                             | (BLE_AOD                                << (BLE_FEAT_AOD                    - 16) ) \
                             | (BLE_AOA                                << (BLE_FEAT_AOA                    - 16) ) \
                             | ((BLE_CON_CTE_REQ | BLE_CONLESS_CTE_RX) << (BLE_FEAT_CTE_RX                 - 16) ) )

/// Features byte 3
#define BLE_FEATURES_BYTE3  (  (BLE_FEAT_PAST_EN << (BLE_FEAT_PER_ADV_SYNC_TRANSF_TX - 24) ) \
                             | (BLE_FEAT_PAST_EN << (BLE_FEAT_PER_ADV_SYNC_TRANSF_RX - 24) ) \
                             | (1 << (BLE_FEAT_SLEEP_CLK_ACC_UPD - 24) ) \
                             | (1 << (BLE_FEAT_PUB_KEY_VALID - 24) ) \
                             | ((BLE_FEAT_CIS_EN && BLE_CENTRAL)     << (BLE_FEAT_CIS_MASTER - 24) ) \
                             | ((BLE_FEAT_CIS_EN && BLE_PERIPHERAL)  << (BLE_FEAT_CIS_SLAVE - 24) ) \
                             | ((BLE_FEAT_BIS_EN && BLE_BROADCASTER) << (BLE_FEAT_ISO_BROADCASTER - 24) ) \
                             | ((BLE_FEAT_BIS_EN && BLE_OBSERVER)    << (BLE_FEAT_SYNCED_RECEIVER - 24) ) )
#if (BT_53)
/// Features byte 4
#define BLE_FEATURES_BYTE4  (  (BLE_FEAT_PWR_CTRL_EN << (BLE_FEAT_POWER_CONTROL_REQ - 32) ) \
                             | (BLE_FEAT_PWR_CTRL_EN << (BLE_FEAT_POWER_CHANGE_IND - 32) ) \
                             | (BLE_FEAT_PWR_CTRL_EN << (BLE_FEAT_PATH_LOSS_MONITORING - 32) ) \
                             | (1 << (BLE_FEAT_PER_ADV_ADI - 32) ) \
                             | (1 << (BLE_FEAT_CON_SUBRATING - 32) ) \
                             | (1 << (BLE_FEAT_CHAN_CLASS - 32) ) )
#else // (BT_53)
/// Features byte 4
#define BLE_FEATURES_BYTE4  (  (BLE_FEAT_PWR_CTRL_EN << (BLE_FEAT_POWER_CONTROL_REQ - 32) ) \
                             | (BLE_FEAT_PWR_CTRL_EN << (BLE_FEAT_POWER_CHANGE_IND - 32) ) \
                             | (BLE_FEAT_PWR_CTRL_EN << (BLE_FEAT_PATH_LOSS_MONITORING - 32) ) )
#endif // (BT_53)

/// Features byte 5
#define BLE_FEATURES_BYTE5  (0)

/// Features byte 6
#define BLE_FEATURES_BYTE6  (0)

/// Features byte 7
#define BLE_FEATURES_BYTE7  (0)

/// States byte 0
#define BLE_STATES_BYTE0    ( BLE_NON_CON_ADV_STATE | BLE_DISC_ADV_STATE\
                            | BLE_CON_ADV_STATE | BLE_HDC_DIRECT_ADV_STATE\
                            | BLE_PASS_SCAN_STATE | BLE_ACTIV_SCAN_STATE\
                            | BLE_INIT_MASTER_STATE | BLE_CON_SLAVE_STATE)

/// States byte 1
#define BLE_STATES_BYTE1    ( BLE_NON_CON_ADV_PASS_SCAN_STATE | BLE_DISC_ADV_PASS_SCAN_STATE\
                            | BLE_CON_ADV_PASS_SCAN_STATE | BLE_HDC_DIRECT_ADV_PASS_SCAN_STATE\
                            | BLE_NON_CON_ADV_ACTIV_SCAN_STATE | BLE_DISC_ADV_ACTIV_SCAN_STATE\
                            | BLE_CON_ADV_ACTIV_SCAN_STATE | BLE_HDC_DIRECT_ADV_ACTIV_SCAN_STATE)

/// States byte 2
#define BLE_STATES_BYTE2    ( BLE_NON_CON_ADV_INIT_STATE | BLE_DISC_ADV_INIT_STATE\
                            | BLE_NON_CON_ADV_MASTER_STATE | BLE_DISC_ADV_MASTER_STATE\
                            | BLE_NON_CON_ADV_SLAVE_STATE | BLE_DISC_ADV_SLAVE_STATE\
                            | BLE_PASS_SCAN_INIT_STATE | BLE_ACTIV_SCAN_INIT_STATE)

/// States byte 3
#define BLE_STATES_BYTE3    ( BLE_PASS_SCAN_MASTER_STATE | BLE_ACTIV_SCAN_MASTER_STATE\
                            | BLE_PASS_SCAN_SLAVE_STATE | BLE_ACTIV_SCAN_SLAVE_STATE\
                            | BLE_INIT_MASTER_MASTER_STATE | BLE_LDC_DIRECT_ADV_STATE\
                            | BLE_LDC_DIRECT_ADV_PASS_SCAN_STATE | BLE_LDC_DIRECT_ADV_ACTIV_SCAN_STATE)

/// States byte 4
#define BLE_STATES_BYTE4    ( BLE_CON_ADV_INIT_MASTER_SLAVE_STATE | BLE_HDC_DIRECT_ADV_INIT_MASTER_SLAVE_STATE\
                            | BLE_LDC_DIRECT_ADV_INIT_MASTER_SLAVE_STATE | BLE_CON_ADV_MASTER_SLAVE_STATE\
                            | BLE_HDC_DIRECT_ADV_MASTER_SLAVE_STATE | BLE_LDC_DIRECT_ADV_MASTER_SLAVE_STATE\
                            | BLE_CON_ADV_SLAVE_SLAVE_STATE | BLE_HDC_DIRECT_ADV_SLAVE_SLAVE_STATE)

/// States byte 5
#define BLE_STATES_BYTE5    ( BLE_LDC_DIRECT_ADV_SLAVE_SLAVE_STATE |  BLE_INIT_MASTER_SLAVE_STATE)

/// States byte 6
#define BLE_STATES_BYTE6    0x0
/// States byte 7
#define BLE_STATES_BYTE7    0x0


/******************************************************************************************/
/* --------------------------   GENERAL SETUP       --------------------------------------*/
/******************************************************************************************/

/************************************
 * KE message heap                  *
 ************************************
 *
 * LLD->LLC/LLM messages:
 *      - connection      < 6
 *      - advertising     adv end 56
 *      - scan            max adv chain 1650
 *      - initiating      init end 56
 *      - per adv tx      2
 *      - per adv rx      max adv chain 1650
 *      - encryption      16
 *      => 1650 + 16 + (N-1) x 56
 *
 * HCI params stored by LLM:
 *      - advertising     26
 *      - scan            16
 *      - initiating      58
 *      - per adv tx      8
 *      - per adv rx      14
 *      => 58 + (N-1) x 26
 *
 * LLCP TX queue:
 *      - Max 3 LLCP in the queue per link
 *      - Average 10 bytes per PDU
 *      - N x 3 x (12 + 10) = N x 66
 *
 * HCI commands reception:
 *      - N commands of max 255 bytes
 *      - N x 255
 *
 * HCI events:
 *      - Max advertising reports in TX queue: N x 255
 *      - + 1 100-bytes event per activity: N x 100
 *
 * Kernel timers:
 *     - 1 timer per connection
 *     - N x 12
 *
 * Due to high probability of fragmentation, and sizes of the Kernel message headers, the total of above additions is
 * doubled.
 */

#define BLE_HEAP_MSG_SIZE  (   1650*BLE_OBSERVER                                               \
                             + 2 * (  (16 + (BLE_ACTIVITY_MAX-1) * 56)                         \
                                    + (58 + (BLE_ACTIVITY_MAX-1) * 26)                         \
                                    + (BLE_ACTIVITY_MAX * 66)                                  \
                                    + (HCI_NB_CMD_PKTS * 255)                                  \
                                    + (BLE_OBSERVER * BLE_MAX_NB_ADV_REP_FRAG * 255 + BLE_ACTIVITY_MAX * 100) \
                                    + (BLE_ACTIVITY_MAX * 12)                                  \
                                   ) )

/************************************
 * Environments heap                *
 ************************************
 *
 * Max env usage per activity:
 *    - connection     84 (LLC) + 144          228
 *    - advertising    140                     140
 *    - scan           32 + 2 x 108 + 16       264
 *    - initiating     84 + 3 x 80             324
 *    - per adv tx     80                      80
 *    - per adv rx     96                      96
 *
 * It is possible to have one initiating + 1 scan + (N-2) other activities.
 * Add 2 activities to consider Heap fragmentation.
 *
 * Heap size = 330 + 270 + (N-2 x 230) + 2 x 230 = 600 + N x 230
 */
#define BLE_HEAP_ENV_SIZE               (330*BLE_CENTRAL + 270*BLE_OBSERVER + BLE_ACTIVITY_MAX * 230)

/// Normal Rx window size (in us)
#define BLE_NORMAL_WIN_SIZE              14
/// Normal Rx window size for coded phy (in us)
// In Long Range, the symbol for the Access Address is 8 times longer than the symbol for 1Mbps.
#define BLE_PHY_CODED_NORMAL_WIN_SIZE   (BLE_NORMAL_WIN_SIZE * 8)


/// Number of devices in the white list
#define BLE_WHITELIST_MAX_CONFIG           (BLE_ACTIVITY_MAX + 2)

/// Number of devices in the Resolution Address List
/// This have to be tuned according to the core frequency. Worst case is having in scan mode
/// all IRK and valid in resolving list and device receive a Direct Adv Report that contains
/// RPAs for InitA and AdvA
#define BLE_RESOL_ADDR_LIST_MAX_CONFIG     (BLE_RAL_MAX_CONFIG)

/// Number of RX data buffers (common for all activities)
#define BLE_DATA_BUF_NB_RX           (BLE_RX_DESC_NB + 2)
/// Size of RX data buffers (common for all activities) (the first 5 bytes are reserved for HCI header)
#define BLE_DATA_BUF_SIZE_RX         (260)
/// Guard space reserved for HCI ACL header in BLE RX buffers
#define BLE_ACL_RX_BUF_HEADER_SPACE  (HCI_ACL_HDR_LEN + 1)

/// Number of TX ACL data buffers
#define BLE_ACL_BUF_NB_TX            (BLE_ACTIVITY_MAX + 2)

/// Number of advertising data buffers
#define BLE_ADV_BUF_NB_TX            ((BLE_ACTIVITY_MAX+1) / 2)
/// Number of advertising or scan response data fragments in extended advertising PDU chain
#if defined(CFG_SLIM)
    #define BLE_ADV_FRAG_NB_TX           (2)
#else // !(CFG_SLIM)
    #define BLE_ADV_FRAG_NB_TX           (5)
#endif //(CFG_SLIM)
/// Size of advertising or scan response data fragments in extended advertising PDU chain
#define BLE_ADV_FRAG_SIZE_TX         (254)
/// Maximum advertising data length
#define BLE_CFG_MAX_ADV_DATA_LEN         (BLE_ADV_FRAG_NB_TX * BLE_ADV_FRAG_SIZE_TX)

/// Maximum number of Rx Descriptors that can be used during the reception of Auxiliary chained
/// advertising packet on Secondary advertising channels (0: no limit)
#define BLE_ADV_FRAG_NB_RX_MAX (7)

/// Define the AUX Frame Space duration
#if defined(CFG_AFS_EXT)
    // Aux Frame Space extended for scanner Aux offload testing
    #define BLE_AFS_DUR                  (3000) // 3.0ms
#else // !(CFG_DBG_AUX_OFFLOAD)
    // Packed advertising - Minimum AUX Frame Space
    #define BLE_AFS_DUR                  (BLE_MAFS_DUR)
#endif // (CFG_DBG_AUX_OFFLOAD)

/// Data packet transmission size and duration
/// These values represent what the device supports
#define BLE_MIN_OCTETS  (27) // number of octets
#define BLE_MIN_TIME    (328) // in us
#define BLE_MAX_OCTETS  (251) // number of octets
#define BLE_MAX_TIME    (17040) // in us

/// Number of devices capacity for the scan duplicate filtering
#if (BLE_OBSERVER)
    #define BLE_DUPLICATE_FILTER_MAX    (10)
#endif //(BLE_OBSERVER)

/// Number of TX descriptors per BLE connection [2|3|4]
#define BLE_NB_TX_DESC_PER_CON          2

/// Number of TX descriptors per advertising set
#define BLE_NB_TX_DESC_PER_ADV          (2+BLE_ADV_FRAG_NB_TX)

/// Number of TX descriptors per BLE activity (maximum needed for connection or advertising)
#if (BLE_NB_TX_DESC_PER_CON > BLE_NB_TX_DESC_PER_ADV)
    #define BLE_NB_TX_DESC_PER_ACT          (BLE_NB_TX_DESC_PER_CON)
#else //(BLE_NB_TX_DESC_PER_CON > BLE_NB_TX_DESC_PER_ADV)
    #define BLE_NB_TX_DESC_PER_ACT          (BLE_NB_TX_DESC_PER_ADV)
#endif //(BLE_NB_TX_DESC_PER_CON > BLE_NB_TX_DESC_PER_ADV)

/// Number of RX descriptors
#define BLE_RX_DESC_NB               (6)
/// Number of TX descriptors
#define BLE_TX_DESC_NB               (BLE_NB_TX_DESC_PER_ACT * BLE_ACTIVITY_MAX)

/// Legacy advertising HCI interface
#define BLE_ADV_LEGACY_ITF           (HCI_TL_SUPPORT)

/// Low energy event mask (should be the standard value if device is in controller-only)
#if ((BLE_EMB_PRESENT) && (!BLE_HOST_PRESENT))
    #define BLE_EVT_MASK                {0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#else // (!(BLE_EMB_PRESENT) || (BLE_HOST_PRESENT))
    /// Low energy event mask (should be the custom mask if device is in full-stack)
    #if (BT_53)
        #define BLE_EVT_MASK                {0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0x00}
    #else //(BT_53)
        #define BLE_EVT_MASK                {0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00}
    #endif //(BT_53)
#endif // ((BLE_EMB_PRESENT) && (!BLE_HOST_PRESENT))

/******************************************************************************************/
/* --------------------------   DEBUG SETUP       ----------------------------------------*/
/******************************************************************************************/

#define BLE_TEST_MODE_SUPPORT       (1)

/******************************************************************************************/
/* -------------------------   COEXISTENCE SETUP      ------------------------------------*/
/******************************************************************************************/

///To let the HW using the default values set in the registers
#define RW_BLE_PTI_PRIO_AUTO    31

/******************************************************************************************/
/* --------------------        CHANNEL ASSESSMENT SETUP         --------------------------*/
/******************************************************************************************/

/// Channel map update period (in sec)
#define BLE_CH_MAP_UPDATE_PERIOD         4

#if (BT_53)
    /******************************************************************************************/
    /* --------------------       CHANNEL CLASSIFICATION SETUP      --------------------------*/
    /******************************************************************************************/

    /// Channel classification min spacing (in units of 200 ms)
    #define BLE_CH_CLASS_DFT_MIN_SPACING 5
    /// Channel classification max delay (in units of 200 ms)
    #define BLE_CH_CLASS_DFT_MAX_DELAY   5
#endif // (BT_53)


/******************************************************************************************/
/* --------------------------   Direction finding (AoA/AoD)      -------------------------*/
/******************************************************************************************/

/// Number of antennae used for constant tone extension transmission or reception
#define BLE_ANTENNA_NB         4

/// Maximum length of switching pattern
#define BLE_MAX_SW_PAT_LEN     12

/// Number of RX descriptors used for constant tone extension reception
#define BLE_RX_CTE_DESC_NB     3

/******************************************************************************************/
/* --------------------          Tx/Rx Path Compensation         -------------------------*/
/******************************************************************************************/

/// Default TX Path Compensation - The default value is vendor-specific.
#define BLE_DFT_TX_PATH_RF_COMPENSATION    0

/// Default RX Path Compensation - The default value is vendor-specific.
#define BLE_DFT_RX_PATH_RF_COMPENSATION    0

/******************************************************************************************/
/* --------------------------             ISOoHCI                -------------------------*/
/******************************************************************************************/

#if (BLE_ISOOHCI)
    /**
    * When receiving HCI ISO packets, the stack allocates a buffer with a sufficient size to receive a complete SDU.
    * There is no necessity for the Host to fragment the SDU. If it still does, all fragments of an SDU are appended to the SDU buffer.
    *
    * The constraints imposed on the Host are:
    *    - Number of HCI ISO Input buffers:
    *          - Limits the amount of memory resources consumed in the Controller in the worst case
    *          - This value needs to consider the possible use cases
    *    - Size of HCI ISO Input buffers (in bytes):
    *          - There is no limitation on HCI ISO fragment sizes, so the controller uses the maximum allowed SDU size
    */
    #define BLE_HCI_ISO_IN_SDU_BUF_NB            (5)
    #define BLE_HCI_ISO_IN_SDU_BUF_SIZE          (BLE_ISO_MAX_SDU_SIZE)
#endif // (BLE_ISOOHCI)


/******************************************************************************************/
/* -----------------------   SUPPORTED HCI COMMANDS       --------------------------------*/
/******************************************************************************************/

//byte0
#define BLE_CMDS_BYTE0      BLE_DISC_CMD
//byte2
#define BLE_CMDS_BYTE2      BLE_RD_REM_VERS_CMD
//byte5
#define BLE_CMDS_BYTE5      (BLE_SET_EVT_MSK_CMD | BLE_RESET_CMD)
//byte10
#define BLE_CMDS_BYTE10     (BLE_HL_NB_CMP_PKT_CMD | BLE_RD_TX_PWR_CMD\
                            |BLE_HL_BUF_SIZE_CMD | BLE_SET_CTRL_TO_HL_FCTRL_CMD)
//byte14
#define BLE_CMDS_BYTE14     (BLE_RD_LOC_VERS_CMD | BLE_RD_LOC_SUP_FEAT_CMD)
//byte15
#define BLE_CMDS_BYTE15     (BLE_RD_BD_ADDR_CMD | BLE_RD_RSSI_CMD)
//byte22
#define BLE_CMDS_BYTE22     (BLE_SET_EVT_MSK_PG2_CMD)
//byte25
#define BLE_CMDS_BYTE25     (BLE_LE_SET_EVT_MSK_CMD | BLE_LE_RD_BUF_SIZE_CMD\
                            |BLE_LE_RD_LOC_SUP_FEAT_CMD | BLE_LE_SET_RAND_ADDR_CMD\
                            |BLE_LE_SET_ADV_PARAM_CMD | BLE_LE_RD_ADV_TX_PWR_CMD\
                            |BLE_LE_SET_ADV_DATA_CMD)
//byte26
#define BLE_CMDS_BYTE26     (BLE_LE_SET_SC_RSP_DATA_CMD | BLE_LE_SET_ADV_EN_CMD\
                            |BLE_LE_SET_SC_PARAM_CMD | BLE_LE_SET_SC_EN_CMD\
                            |BLE_LE_CREAT_CNX_CMD | BLE_LE_CREAT_CNX_CNL_CMD\
                            |BLE_LE_RD_WL_SIZE_CMD | BLE_LE_CLEAR_WL_CMD)
//byte27
#define BLE_CMDS_BYTE27     (BLE_LE_ADD_DEV_WL_CMD | BLE_LE_REM_DEV_WL_CMD\
                            |BLE_LE_CNX_UPDATE_CMD | BLE_LE_SET_HL_CH_CLASS_CMD\
                            |BLE_LE_RD_CH_MAP_CMD | BLE_LE_RD_REM_FEAT_CMD\
                            |BLE_LE_ENCRYPT_CMD | BLE_LE_RAND_CMD)
//byte28
#define BLE_CMDS_BYTE28     (BLE_LE_EN_ENC_CMD | BLE_LE_LTK_REQ_RPLY_CMD\
                            |BLE_LE_LTK_REQ_NEG_RPLY_CMD | BLE_LE_RD_SUPP_STATES_CMD\
                            |BLE_LE_RX_TEST_V1_CMD | BLE_LE_TX_TEST_V1_CMD\
                            |BLE_LE_STOP_TEST_CMD)
//byte32
#define BLE_CMDS_BYTE32     (BLE_RD_AUTH_PAYL_TO_CMD | BLE_WR_AUTH_PAYL_TO_CMD)
//byte33
#define BLE_CMDS_BYTE33     (BLE_LE_REM_CON_PARA_REQ_RPLY_CMD | BLE_LE_REM_CON_PARA_REQ_NEG_RPLY_CMD\
                            | BLE_LE_SET_DATA_LEN_CMD | BLE_LE_RD_SUGGTED_DFT_DATA_LEN_CMD)
//byte34
#define BLE_CMDS_BYTE34     (  BLE_LE_WR_SUGGTED_DFT_DATA_LEN_CMD   \
                             | BLE_LE_RD_LOC_P256_PUB_KEY_CMD       \
                             | BLE_LE_GEN_DHKEY_V1_CMD              \
                             | BLE_LE_ADD_DEV_TO_RESOLV_LIST_CMD    \
                             | BLE_LE_REM_DEV_FROM_RESOLV_LIST_CMD  \
                             | BLE_LE_CLEAR_RESOLV_LIST_CMD         \
                             | BLE_LE_RD_RESOLV_LIST_SIZE_CMD       \
                             | BLE_LE_RD_PEER_RESOLV_ADDR_CMD        )
//byte35
#define BLE_CMDS_BYTE35     (  BLE_LE_RD_LOCAL_RESOLV_ADDR_CMD      \
                             | BLE_LE_SET_ADDR_RESOL_CMD            \
                             | BLE_LE_SET_RESOLV_PRIV_ADDR_TO_CMD   \
                             | BLE_LE_RD_MAX_DATA_LEN_CMD           \
                             | BLE_LE_RD_PHY_CMD                    \
                             | BLE_LE_SET_DFT_PHY_CMD               \
                             | BLE_LE_SET_PHY_CMD                   \
                             | BLE_LE_RX_TEST_V2_CMD                 )
//byte36
#define BLE_CMDS_BYTE36     (  BLE_LE_TX_TEST_V2_CMD                \
                             | BLE_LE_SET_ADV_SET_RAND_ADDR_CMD     \
                             | BLE_LE_SET_EXT_ADV_PARAM_CMD         \
                             | BLE_LE_SET_EXT_ADV_DATA_CMD          \
                             | BLE_LE_SET_EXT_SCAN_RSP_DATA_CMD     \
                             | BLE_LE_SET_EXT_ADV_EN_CMD            \
                             | BLE_LE_RD_MAX_ADV_DATA_LEN_CMD       \
                             | BLE_LE_RD_NB_SUPP_ADV_SETS_CMD        )
//byte37
#define BLE_CMDS_BYTE37     (  BLE_LE_RMV_ADV_SET_CMD               \
                             | BLE_LE_CLEAR_ADV_SETS_CMD            \
                             | BLE_LE_SET_PER_ADV_PARAM_CMD         \
                             | BLE_LE_SET_PER_ADV_DATA_CMD          \
                             | BLE_LE_SET_PER_ADV_EN_CMD            \
                             | BLE_LE_SET_EXT_SCAN_PARAM_CMD        \
                             | BLE_LE_SET_EXT_SCAN_EN_CMD           \
                             | BLE_LE_EXT_CREATE_CON_CMD             )
//byte38
#define BLE_CMDS_BYTE38     (  BLE_LE_PER_ADV_CREATE_SYNC_CMD        \
                             | BLE_LE_PER_ADV_CREATE_SYNC_CANCEL_CMD \
                             | BLE_LE_PER_ADV_TERM_SYNC_CMD          \
                             | BLE_LE_ADD_DEV_TO_PER_ADV_LIST_CMD    \
                             | BLE_LE_RMV_DEV_FROM_PER_ADV_LIST_CMD  \
                             | BLE_LE_CLEAR_PER_ADV_LIST_CMD         \
                             | BLE_LE_RD_PER_ADV_LIST_SIZE_CMD       \
                             | BLE_LE_RD_TX_PWR_CMD                   )
//byte39
#define BLE_CMDS_BYTE39     (  BLE_LE_RD_RF_PATH_COMP_CMD            \
                             | BLE_LE_WR_RF_PATH_COMP_CMD            \
                             | BLE_LE_SET_PRIV_MODE_CMD              \
                             | BLE_LE_RX_TEST_V3_CMD                 \
                             | BLE_LE_TX_TEST_V3_CMD                 \
                             | BLE_LE_SET_CONLESS_CTE_TX_PARAM_CMD   \
                             | BLE_LE_SET_CONLESS_CTE_TX_EN_CMD      \
                             | BLE_LE_SET_CONLESS_IQ_SAMPL_EN_CMD     )
//byte40
#define BLE_CMDS_BYTE40     (  BLE_LE_SET_CON_CTE_RX_PARAM_CMD       \
                             | BLE_LE_SET_CON_CTE_TX_PARAM_CMD       \
                             | BLE_LE_CON_CTE_REQ_EN_CMD             \
                             | BLE_LE_CON_CTE_RSP_EN_CMD             \
                             | BLE_LE_RD_ANTENNA_INF_CMD             \
                             | BLE_LE_SET_PER_ADV_RX_EN_CMD          \
                             | BLE_LE_PER_ADV_SYNC_TRANSF_CMD        \
                             | BLE_LE_PER_ADV_SET_INFO_TRANSF_CMD     )
//byte41
#define BLE_CMDS_BYTE41     (  BLE_LE_SET_PER_ADV_SYNC_TRANSF_PARAM_CMD        \
                             | BLE_LE_SET_DFT_PER_ADV_SYNC_TRANSF_PARAM_CMD    \
                             | BLE_LE_GEN_DHKEY_V2_CMD                         \
                             | BLE_LE_MOD_SLP_CLK_ACC_CMD                      \
                             | BLE_LE_RD_BUF_SIZE_V2_CMD                       \
                             | BLE_LE_RD_ISO_TX_SYNC_CMD                       \
                             | BLE_LE_SET_CIG_PARAMS_CMD                        )

//byte42
#define BLE_CMDS_BYTE42     (  BLE_LE_SET_CIG_PARAMS_TEST_CMD                  \
                             | BLE_LE_CREATE_CIS_CMD                           \
                             | BLE_LE_REMOVE_CIG_CMD                           \
                             | BLE_LE_ACCEPT_CIS_REQ_CMD                       \
                             | BLE_LE_REJECT_CIS_REQ_CMD                       \
                             | BLE_LE_CREATE_BIG_CMD                           \
                             | BLE_LE_CREATE_BIG_TEST_CMD                      \
                             | BLE_LE_TERMINATE_BIG_CMD                         )

//byte43
#define BLE_CMDS_BYTE43     (  BLE_LE_CREATE_BIG_SYNC_CMD                      \
                             | BLE_LE_BIG_TERMINATE_SYNC_CMD                   \
                             | BLE_LE_REQ_PEER_SCA_CMD                         \
                             | BLE_LE_SETUP_ISO_DATA_PATH_CMD                  \
                             | BLE_LE_REMOVE_ISO_DATA_PATH_CMD                 \
                             | BLE_LE_ISO_TX_TEST_CMD                          \
                             | BLE_LE_ISO_RX_TEST_CMD                          \
                             | BLE_LE_ISO_RD_TEST_COUNTERS_CMD                  )

//byte44
#define BLE_CMDS_BYTE44     (  BLE_LE_ISO_TEST_END_CMD                         \
                             | BLE_LE_SET_HOST_FEAT_CMD                        \
                             | BLE_LE_RD_ISO_LINK_QUALITY_CMD                  \
                             | BLE_LE_ENH_RD_TX_POWER_LEVEL_CMD                \
                             | BLE_LE_RD_REMOTE_TX_POWER_CMD                   \
                             | BLE_LE_SET_PATH_LOSS_REPORTING_PARAMS_CMD       \
                             | BLE_LE_SET_PATH_LOSS_REPORTING_ENABLE_CMD       \
                             | BLE_LE_SET_TX_POWER_REPORTING_ENABLE_CMD         )

//byte 45
#if (BT_53)
#define BLE_CMDS_BYTE45     (  BLE_LE_TX_TEST_V4_CMD                           \
                             | BLE_LE_SET_ECO_BASE_INTERVAL_CMD                \
                             | BLE_LE_RD_LOC_SUPP_CODECS_V2_CMD                \
                             | BLE_LE_RD_LOC_SUPP_CODEC_CAP_CMD                \
                             | BLE_LE_RD_LOC_SUPP_CTRL_DELAY_CMD               \
                             | BLE_LE_CONFIG_DATA_PATH_CMD                     \
                             | BLE_LL_SET_DATA_RELATED_ADDR_CHG_CMD            )

//byte 46
#define BLE_CMDS_BYTE46     (  BLE_LL_SET_DEFAULT_SUBRATE_CMD                 \
                             | BLE_LL_SUBRATE_REQUEST_CMD                      )

#else // (BT_53)
#define BLE_CMDS_BYTE45     (  BLE_LE_TX_TEST_V4_CMD                           \
                             | BLE_LE_SET_ECO_BASE_INTERVAL_CMD                \
                             | BLE_LE_RD_LOC_SUPP_CODECS_V2_CMD                \
                             | BLE_LE_RD_LOC_SUPP_CODEC_CAP_CMD                \
                             | BLE_LE_RD_LOC_SUPP_CTRL_DELAY_CMD               \
                             | BLE_LE_CONFIG_DATA_PATH_CMD                      )
#endif // (BT_53)



/******************************************************************************************/
/* ------------------------   CONNECTION HANDLE   ----------------------------------------*/
/******************************************************************************************/

/// Bit set for BLE connection handles
#define BLE_CONHDL_MIN                 (0)
#define BLE_CONHDL_MAX                 (BLE_CONHDL_MIN + BLE_ACTIVITY_MAX - 1)
#define BLE_LINKID_TO_CONHDL(link_id)  ((uint16_t) (BLE_CONHDL_MIN + (link_id)))
#define BLE_CONHDL_TO_LINKID(conhdl)   ((uint8_t) ((conhdl) - BLE_CONHDL_MIN))

/// Bit set for BLE sync handles (used in periodic advertising receiver mode)
#define BLE_SYNCHDL_MIN                (0)
#define BLE_SYNCHDL_MAX                (BLE_SYNCHDL_MIN + BLE_ACTIVITY_MAX - 1)
#define BLE_ACTID_TO_SYNCHDL(act_id)   ((uint16_t) (BLE_SYNCHDL_MIN + (act_id)))
#define BLE_SYNCHDL_TO_ACTID(synchdl)  ((uint8_t) ((synchdl) - BLE_SYNCHDL_MIN))

/// CIS Channel handle mapping
#define BLE_CISHDL_MIN                  (0x100)
#define BLE_ACTID_TO_CISHDL(act_id)     ((uint16_t) (BLE_CISHDL_MIN + (act_id)))
#define BLE_CISHDL_TO_ACTID(cishdl)     ((uint8_t) ((cishdl) - BLE_CISHDL_MIN))
#define BLE_IS_CISHDL(conhdl)           ((((conhdl) >= BLE_CISHDL_MIN) && ((conhdl) < BLE_BISHDL_MIN)) ? true : false)
/// BIS Channel handle mapping
#define BLE_BISHDL_MIN                  (0x200)
#define BLE_ACTID_TO_BISHDL(act_id)     ((uint16_t) (BLE_BISHDL_MIN + (act_id)))
#define BLE_BISHDL_TO_ACTID(bishdl)     ((uint8_t) ((bishdl) - BLE_BISHDL_MIN))
#define BLE_IS_BISHDL(conhdl)           ((((conhdl) >= BLE_BISHDL_MIN) && ((conhdl) < BLE_AM0HDL_MIN)) ? true : false)




/// AM0 Channel handle mapping
#define BLE_AM0HDL_MIN                  (0x300)
#define BLE_ACTID_TO_AM0HDL(act_id)     ((uint16_t) (BLE_AM0HDL_MIN + (act_id)))
#define BLE_AM0HDL_TO_ACTID(am0)        ((uint8_t) ((am0) - BLE_AM0HDL_MIN))
#define BLE_IS_AM0HDL(conhdl)           (((conhdl) >= BLE_AM0HDL_MIN) ? true : false)

/// ISO Channel handle mapping
#define BLE_ISOHDL_TO_ACTID(isohdl)     ((isohdl >= BLE_AM0HDL_MIN) \
                                               ? BLE_AM0HDL_TO_ACTID(isohdl) \
                                               : ((isohdl >= BLE_BISHDL_MIN) \
                                                    ? BLE_BISHDL_TO_ACTID(isohdl) \
                                                    : BLE_CISHDL_TO_ACTID(isohdl)))


/// @} BLE stack configuration
/// @} ROOT

#endif // RWBLE_CONFIG_H_
