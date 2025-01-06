/**
 ****************************************************************************************
 *
 * @file lm_task.c
 *
 * @brief LM task source file
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LMTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#include <string.h>
#include "em_map.h"
#include "co_endian.h"
#include "co_utils.h"
#include "co_math.h"
#include "co_bt.h"          // BT standard definitions
#include "co_version.h"

#include "lm.h"             // link manager definitions
#include "lm_int.h"         // link manager internal definitions

#include "ke_mem.h"         // kernel memory definitions
#include "ke_timer.h"       // kernel timer definitions
#include "ecc_p256.h"       // Elliptic curve calculation P256

#include "lc.h"             // link controller definitions
#include "../lc/lc_int.h"   // link controller internal definitions
#include "lb.h"             // link broadcast controller
#include "ld.h"             // link driver

#include "rwip.h"           // stack main module

#if HCI_PRESENT
    #include "hci.h"            // host controller interface
#endif //HCI_PRESENT

#if (BLE_EMB_PRESENT && BLE_ISO_PRESENT)
    #include "data_path.h"      // Data Path API
#endif // (BLE_EMB_PRESENT && BLE_ISO_PRESENT)

#include "dbg.h"

#if (EAVESDROPPING_SUPPORT)
    #include "ed.h"
#endif // EAVESDROPPING_SUPPORT


/******************************************************************************************/
/* -------------------------   FEATURES SETUP      ---------------------------------------*/
/******************************************************************************************/

#define FEAT_3_SLOT_SUPP               1
#define FEAT_5_SLOT_SUPP               1
#define FEAT_ENC_SUPP                  1
#define FEAT_SLOT_OFF_SUPP             1
#define FEAT_TIMING_ACCU_SUPP          1
#define FEAT_ROLE_SWITCH_SUPP          1
#define FEAT_HOLD_MODE_SUPP            0
#define FEAT_SNIFF_MODE_SUPP           1

#define FEAT_PARK_SUPP                 0
#define FEAT_RSSI_SUPP                 (BT_PWR_CTRL)
#define FEAT_CQDDR_SUPP                1
#define FEAT_SCO_SUPP                  (MAX_NB_SYNC > 0)
#define FEAT_HV2_SUPP                  0
#define FEAT_HV3_SUPP                  (MAX_NB_SYNC > 0)
#define FEAT_MULAW_SUPP                1
#define FEAT_ALAW_SUPP                 1

#define FEAT_CVSD_SUPP                 1
#define FEAT_PAGING_PAR_NEGO_SUPP      0
#define FEAT_PWR_CTRL_SUPP             (BT_PWR_CTRL)
#define FEAT_TRANSPARENT_SCO_SUPP      1
#define FEAT_FLOW_CTRL_LAG_SUPP        4
#define FEAT_BCAST_ENC_SUPP            (BCAST_ENC_SUPPORT)

#define FEAT_EDR_2MBPS_ACL_SUPP        1
#define FEAT_EDR_3MBPS_ACL_SUPP        1
#define FEAT_ENH_INQSCAN_SUPP          1
#define FEAT_INT_INQSCAN_SUPP          1
#define FEAT_INT_PAGESCAN_SUPP         1
#define FEAT_RSSI_INQ_RES_SUPP         1
#define FEAT_ESCO_EV3_SUPP             1

#define FEAT_EV4_PKT_SUPP              (MAX_NB_SYNC > 0)
#define FEAT_EV5_PKT_SUPP              (MAX_NB_SYNC > 0)
#define FEAT_AFH_CAP_SLV_SUPP          1
#define FEAT_AFH_CLASS_SLV_SUPP        1
#define FEAT_BR_EDR_NOT_SUPP_SUPP      0
#define FEAT_LE_SUPP_SUPP              (BLE_EMB_PRESENT)
#define FEAT_3_SLOT_EDR_ACL_SUPP       1

#define FEAT_5_SLOT_EDR_ACL_SUPP       1
#define FEAT_SSR_SUPP                  1
#define FEAT_PAUSE_ENC_SUPP            1
#define FEAT_AFH_CAP_MST_SUPP          1
#define FEAT_AFH_CLASS_MST_SUPP        1
#define FEAT_EDR_ESCO_2MBPS_SUPP       (MAX_NB_SYNC > 0)
#define FEAT_EDR_ESCO_3MBPS_SUPP       (MAX_NB_SYNC > 0)
#define FEAT_3_SLOT_EDR_ESCO_SUPP      (MAX_NB_SYNC > 0)

#define FEAT_EIR_SUPP                  1
#define FEAT_SIM_LE_BREDR_DEV_CAP_SUPP 1
#define FEAT_SSP_SUPP                  1
#define FEAT_ENCAPS_PDU_SUPP           1
#define FEAT_ERR_DATA_REP_SUPP         1
#define FEAT_NONFLUSH_PBF_SUPP         1

#define FEAT_LST_CHANGE_EVT_SUPP       1
#define FEAT_INQRES_TXPOW_SUPP         (BT_PWR_CTRL)
#define FEAT_ENH_PWR_CTRL_SUPP         (BT_PWR_CTRL)
#define FEAT_EXT_FEATS_SUPP            1

#define FEAT_HOST_SSP_SUPP             0 // this bit is set by Host
#define FEAT_HOST_LE_SUPP              0 // this bit is set by Host
#define FEAT_HOST_LE_BR_EDR_SUPP       0 // this bit is set by Host
#define FEAT_HOST_SECURE_CON_SUPP      0 // this bit is set by Host

#define FEAT_CSB_MASTER_SUPP           CSB_SUPPORT
#define FEAT_CSB_SLAVE_SUPP            CSB_SUPPORT
#define FEAT_SYNC_TRAIN_SUPP           (CSB_SUPPORT|PCA_SUPPORT)
#define FEAT_SYNC_SCAN_SUPP            (CSB_SUPPORT|PCA_SUPPORT)
#define FEAT_INQ_RES_NOTIF_EVT_SUPP    1
#define FEAT_GEN_INTERL_SCAN_SUPP      RW_BT_MWS_COEX
#define FEAT_COARSE_CLK_ADJ_SUPP       PCA_SUPPORT

#define FEAT_SEC_CON_CTRL_SUPP         1
#define FEAT_PING_SUPP                 1
#define FEAT_SAM_SUPP                  1
#define FEAT_TRAIN_NUDGING_SUPP        RW_BT_MWS_COEX

#define B(byte, feat)     ((FEAT_##feat##_SUPP << B##byte##_##feat##_POS) & B##byte##_##feat##_MSK)
#define FEAT_P0_BYTE0     (B(0, 3_SLOT              ) | \
                           B(0, 5_SLOT              ) | \
                           B(0, ENC                 ) | \
                           B(0, SLOT_OFF            ) | \
                           B(0, TIMING_ACCU         ) | \
                           B(0, ROLE_SWITCH         ) | \
                           B(0, HOLD_MODE           ) | \
                           B(0, SNIFF_MODE          ) )
#define FEAT_P0_BYTE1     (B(1, PARK                ) | \
                           B(1, RSSI                ) | \
                           B(1, CQDDR               ) | \
                           B(1, SCO                 ) | \
                           B(1, HV2                 ) | \
                           B(1, HV3                 ) | \
                           B(1, MULAW               ) | \
                           B(1, ALAW                ) )
#define FEAT_P0_BYTE2     (B(2, CVSD                ) | \
                           B(2, PAGING_PAR_NEGO     ) | \
                           B(2, PWR_CTRL            ) | \
                           B(2, TRANSPARENT_SCO     ) | \
                           B(2, FLOW_CTRL_LAG       ) | \
                           B(2, BCAST_ENC           ) )
#define FEAT_P0_BYTE3     (B(3, EDR_2MBPS_ACL       ) | \
                           B(3, EDR_3MBPS_ACL       ) | \
                           B(3, ENH_INQSCAN         ) | \
                           B(3, INT_INQSCAN         ) | \
                           B(3, INT_PAGESCAN        ) | \
                           B(3, RSSI_INQ_RES        ) | \
                           B(3, ESCO_EV3            ) )
#define FEAT_P0_BYTE4     (B(4, EV4_PKT             ) | \
                           B(4, EV5_PKT             ) | \
                           B(4, AFH_CAP_SLV         ) | \
                           B(4, AFH_CLASS_SLV       ) | \
                           B(4, BR_EDR_NOT_SUPP     ) | \
                           B(4, LE_SUPP             ) | \
                           B(4, 3_SLOT_EDR_ACL      ) )
#define FEAT_P0_BYTE5     (B(5, 5_SLOT_EDR_ACL      ) | \
                           B(5, SSR                 ) | \
                           B(5, PAUSE_ENC           ) | \
                           B(5, AFH_CAP_MST         ) | \
                           B(5, AFH_CLASS_MST       ) | \
                           B(5, EDR_ESCO_2MBPS      ) | \
                           B(5, EDR_ESCO_3MBPS      ) | \
                           B(5, 3_SLOT_EDR_ESCO     ) )
#define FEAT_P0_BYTE6     (B(6, EIR                 ) | \
                           B(6, SIM_LE_BREDR_DEV_CAP) | \
                           B(6, SSP                 ) | \
                           B(6, ENCAPS_PDU          ) | \
                           B(6, ERR_DATA_REP        ) | \
                           B(6, NONFLUSH_PBF        ) )
#define FEAT_P0_BYTE7     (B(7, LST_CHANGE_EVT      ) | \
                           B(7, INQRES_TXPOW        ) | \
                           B(7, ENH_PWR_CTRL        ) | \
                           B(7, EXT_FEATS           ) )
#define FEAT_P1_BYTE0     (B(0, HOST_SSP            ) | \
                           B(0, HOST_LE             ) | \
                           B(0, HOST_LE_BR_EDR      ) | \
                           B(0, HOST_SECURE_CON     ) )
#define FEAT_P2_BYTE0     (B(0, CSB_MASTER          ) | \
                           B(0, CSB_SLAVE           ) | \
                           B(0, SYNC_TRAIN          ) | \
                           B(0, SYNC_SCAN           ) | \
                           B(0, INQ_RES_NOTIF_EVT   ) | \
                           B(0, GEN_INTERL_SCAN     ) | \
                           B(0, COARSE_CLK_ADJ      ) )
#define FEAT_P2_BYTE1     (B(1, SEC_CON_CTRL        ) | \
                           B(1, PING                ) | \
                           B(1, SAM                ) | \
                           B(1, TRAIN_NUDGING       ) )


/******************************************************************************************/
/* -----------------------   SUPPORTED HCI COMMANDS       --------------------------------*/
/******************************************************************************************/

/* Byte 0
0 Inquiry                           Yes
1 Inquiry Cancel                    Yes
2 Periodic Inquiry Mode             Yes
3 Exit Periodic Inquiry Mode        Yes
4 Create Connection                 Yes
5 Disconnect                        Yes
6 Add SCO Connection                No (deprecated command)
7 Cancel Create Connection          Yes
*/
#define BT_CMDS_BYTE0               0xBF

/* Byte 1
0 Accept Connection Request         Yes
1 Reject Connection Request         Yes
2 Link Key Request Reply            Yes
3 Link Key Request Negative Reply   Yes
4 PIN Code Request Reply            Yes
5 PIN Code Request Negative Reply   Yes
6 Change Connection Packet Type     Yes
7 Authentication Request            Yes
*/
#define BT_CMDS_BYTE1               0xFF

/* Byte 2
0 Set Connection Encryption         Yes
1 Change Connection Link Key        Yes
2 Master Link Key                   Yes
3 Remote Name Request               Yes
4 Cancel Remote Name Request        Yes
5 Read Remote Supported Features    Yes
6 Read Remote Extended Features     Yes
7 Read Remote Version Information   Yes
*/
#if (BCAST_ENC_SUPPORT)
    #define BT_CMDS_BYTE2               0xFF
#else // !(BCAST_ENC_SUPPORT)
    #define BT_CMDS_BYTE2               0xFB
#endif // !(BCAST_ENC_SUPPORT)

/* Byte 3
0 Read Clock Offset                 Yes
1 Read LMP Handle                   Yes
2 Reserved                          No
3 Reserved                          No
4 Reserved                          No
5 Reserved                          No
6 Reserved                          No
7 Reserved                          No
*/
#define BT_CMDS_BYTE3               0x03

/* Byte 4
0 Reserved                          No
1 Hold Mode                         No
2 Sniff Mode                        Yes
3 Exit Sniff Mode                   Yes
4 Park State                        No
5 Exit Park State                   No
6 QoS Setup                         Yes
7 Role Discovery                    Yes
*/
#define BT_CMDS_BYTE4               0xCC

/* Byte 5
0 Switch Role                        Yes
1 Read Link Policy Settings          Yes
2 Write Link Policy Settings         Yes
3 Read Default Link Policy Settings  Yes
4 Write Default Link Policy Settings Yes
5 Flow Specification                 Yes (Not compatible with LM_QoSSetup)
6 Set Event Mask                     Yes
7 Reset                              Yes
*/
#define BT_CMDS_BYTE5               0xFF

/* Byte 6
0 Set Event Filter                  Yes
1 Flush                             Yes
2 Read PIN Type                     Yes
3 Write PIN Type                    Yes
4 Create New Unit Key               No
5 Read Stored Link Key              Yes
6 Write Stored Link Key             Yes
7 Delete Stored Link Key            Yes
*/
#define BT_CMDS_BYTE6               0xEF

/* Byte 7
0 Write Local Name                  Yes
1 Read Local Name                   Yes
2 Read Connection Accept Timeout    Yes
3 Write Connection Accept Timeout   Yes
4 Read Page Timeout                 Yes
5 Write Page Timeout                Yes
6 Read Scan Enable                  Yes
7 Write Scan Enable                 Yes
*/
#define BT_CMDS_BYTE7               0xFF

/* Byte 8
0 Read Page Scan Activity           Yes
1 Write Page Scan Activity          Yes
2 Read Inquiry Scan Activity        Yes
3 Write Inquiry Scan Activity       Yes
4 Read Authentication Enable        Yes
5 Write Authentication Enable       Yes
6.Read Encryption Mode              No (deprecated command)
7.Write Encryption Mode             No (deprecated command)
*/
#define BT_CMDS_BYTE8               0x3F

/* Byte 9
0 Read Class Of Device                Yes
1 Write Class Of Device               Yes
2 Read Voice Setting                  Yes
3 Write Voice Setting                 Yes
4 Read Automatic Flush Timeout        Yes
5 Write Automatic Flush Timeout       Yes
6 Read Num Broadcast Retransmissions  Yes
7 Write Num Broadcast Retransmissions Yes
*/
#define BT_CMDS_BYTE9                 0xFF

/* Byte 10
0 Read Hold Mode Activity                  No
1 Write Hold Mode Activity                 No
2 Read Transmit Power Level                Yes
3 Read Synchronous Flow Control Enable     Yes
4 Write Synchronous Flow Control Enable    Yes
5 Set Host Controller To Host Flow Control Yes
6 Host Buffer Size                         Yes
7 Host Number Of Completed Packets         Yes
*/
#define BT_CMDS_BYTE10              0xFC

/* Byte 11
0 Read Link Supervision Timeout     Yes
1 Write Link Supervision Timeout    Yes
2 Read Number of Supported IAC      Yes
3 Read Current IAC LAP              Yes
4 Write Current IAC LAP             Yes
5 Read Page Scan Period Mode        No (deprecated command)
6 Write Page Scan Period Mode       No (deprecated command)
7 Read Page Scan Mode               No (deprecated command)
*/
#define BT_CMDS_BYTE11              0x1F

/* Byte 12
0 Write Page Scan Mode              No (deprecated command)
1 Set AFH Channel Classification    Yes
2 reserved                          No
3 reserved                          No
4 Read Inquiry Scan Type            Yes
5 Write Inquiry Scan Type           Yes
6 Read Inquiry Mode                 Yes
7 Write Inquiry Mode                Yes
*/
#define BT_CMDS_BYTE12              0xF2

/* Byte 13
0 Read Page Scan Type               Yes
1 Write Page Scan Type              Yes
2 Read AFH Channel Assessment Mode  Yes
3 Write AFH Channel Assessment Mode Yes
4 Reserved                          No
5 Reserved                          No
6 Reserved                          No
7 Reserved                          No
*/
#define BT_CMDS_BYTE13              0x0F

/* Byte 14
0 Reserved                          No
1 Reserved                          No
2 Reserved                          No
3 Read Local Version Information    Yes
4 Reserved                          No
5 Read Local Supported Features     Yes
6 Read Local Extended Features      Yes
7 Read Buffer Size                  Yes
*/
#define BT_CMDS_BYTE14              0xE8

/* Byte 15
0 Read Country Code                 No (deprecated command)
1 Read BD ADDR                      Yes
2 Read Failed Contact Count         Yes
3 Reset Failed Contact Count        Yes
4 Get Link Quality                  Yes
5 Read RSSI                         Yes
6 Read AFH Channel Map              Yes
7 Read BD Clock                     Yes
*/
#define BT_CMDS_BYTE15              0xFE

/* Byte 16
0 Read Loopback Mode                Yes
1 Write Loopback Mode               Yes
2 Enable Device Under Test Mode     Yes
3 Setup Synchronous Connection      Yes
4 Accept Synchronous Connection     Yes
5 Reject Synchronous Connection     Yes
6 Reserved                          No
7 Reserved                          No
*/
#define BT_CMDS_BYTE16              0x3F

/* Byte 17
0.Read Extended Inquiry Response    Yes
1.Write Extended Inquiry Response   Yes
2.Refresh Encryption Key            Yes
3.Reserved                          No
4.Sniff Subrating                   Yes
5.Read Simple Pairing Mode          Yes
6.Write Simple Pairing Mode         Yes
7.Read Local OOB Data               Yes
*/
#define BT_CMDS_BYTE17              0xF7

/* Byte 18
0.Read Inquiry Response Transmit Power    Yes
1.Write Inquiry Transmit Power Level      Yes
2.Read Default Erroneous Data Reporting   Yes
3.Write Default Erroneous Data Reporting  Yes
4.Reserved                                No
5.Reserved                                No
6.Reserved                                No
7.IO Capability Request Reply             Yes
*/
#define BT_CMDS_BYTE18                    0x8F

/* Byte 19
0.User Confirmation Request Reply            Yes
1.User Confirmation Request Negative Reply   Yes
2.User Passkey Request Reply                 Yes
3.User Passkey Request Negative Reply        Yes
4.Remote OOB Data Request Reply              Yes
5.Write Simple Pairing Debug Mode            Yes
6 Enhanced Flush                             Yes
7 Remote OOB Data Request Negative Reply     Yes
*/
#define BT_CMDS_BYTE19                   0xFF

/* Byte 20
0.Reserved                               No
1.Reserved                               No
2.Send Keypress Notification             Yes
3.IO Capabilities Request Negative Reply Yes
4.Read Encryption Key Size               Yes
5.Reserved                               No
6.Reserved                               No
7.Reserved                               No
*/
#define BT_CMDS_BYTE20                 0x1C
/* Byte 21
0.Create Physical Link                 No (deprecated command)
1.Accept Physical Link                 No (deprecated command)
2.Disconnect Physical Link             No (deprecated command)
3.Create Logical Link                  No (deprecated command)
4.Accept Logical Link                  No (deprecated command)
5.Disconnect Logical Link              No (deprecated command)
6.Logical Link Cancel                  No (deprecated command)
7.Flow Spec Modify                     No (deprecated command)
*/
#define BT_CMDS_BYTE21                 0x00
/* Byte 22
0.Read Logical Link Accept Timeout     No (deprecated command)
1.Write Logical Link Accept Timeout    No (deprecated command)
2.Set Event Mask Page 2                Yes
3.Read Location Data                   No (deprecated command)
4.Write Location Data                  No (deprecated command)
5.Read Local AMP Info                  No (deprecated command)
6.Read Local AMP_ASSOC                 No (deprecated command)
7.Write Remote AMP_ASSOC               No (deprecated command)
*/
#define BT_CMDS_BYTE22                 0x04
/* Byte 23
0.Read Flow Control Mode               No
1.Write Flow Control Mode              No
2.Read Data Block Size                 No
3.Reserved                             No
4.Reserved                             No
5.Enable AMP Receiver Report           No (deprecated command)
6.AMP Test End                         No (deprecated command)
7.AMP Test Command                     No (deprecated command)
*/
#define BT_CMDS_BYTE23                 0x00
/* Byte 24
0.Read Enhanced Transmit Power Level   Yes
1.Reserved                             No
2.Read Best Effort Flush Timeout       No (deprecated command)
3.Write Best Effort Flush Timeout      No (deprecated command)
4.Short Range Mode                     No (deprecated command)
5.Read LE Host Support                 Yes
6.Write LE Host Support                Yes
7.Reserved                             No
*/
#if (BT_PWR_CTRL)
    #define BT_CMDS_BYTE24                 0x61
#else // !(BT_PWR_CTRL)
    #define BT_CMDS_BYTE24                 0x60
#endif // !(BT_PWR_CTRL)


/// BT command byte25
#define BT_CMDS_BYTE25     0
/// BT command byte26
#define BT_CMDS_BYTE26     0
/// BT command byte27
#define BT_CMDS_BYTE27     0
/// BT command byte28
#define BT_CMDS_BYTE28     0

#define MWS_SUPPORT        (RW_MWS_COEX)

/* BT command byte29
 * 0.Reserved                               No
 * 1.Reserved                               No
 * 2.Reserved                               No
 * 3.Enhanced Setup Synchronous Connection  Yes
 * 4.Enhanced Accept Synchronous Connection Yes
 * 5.Read Local Supported Codecs            Yes
 * 6.Set MWS Channel Parameters             MWS_SUPPORT
 * 7.Set External Frame Configuration       MWS_SUPPORT
 */
#if MWS_SUPPORT
    #define BT_CMDS_BYTE29_MWS              0xC0
#else
    #define BT_CMDS_BYTE29_MWS              0x00
#endif //MWS_SUPPORT
#define BT_CMDS_BYTE29                  (0x38 | BT_CMDS_BYTE29_MWS)

/* BT command byte30
 * 0 Set MWS Signaling                  MWS_SUPPORT
 * 1 Set Transport Layer                MWS_SUPPORT
 * 2 Set MWS Scan Frequency Table       MWS_SUPPORT
 * 3 Get Transport Layer Configuration  MWS_SUPPORT
 * 4 Set MWS PATTERN Configuration      MWS_SUPPORT
 * 5 Set Triggered Clock Capture        No
 * 6 Truncated Page                     CSB_SUPPORT
 * 7 Truncated Page Cancel              CSB_SUPPORT
 */
#if MWS_SUPPORT
    #define BT_CMDS_BYTE30_MWS              0x1F
#else
    #define BT_CMDS_BYTE30_MWS              0x00
#endif //MWS_SUPPORT
#if CSB_SUPPORT
    #define BT_CMDS_BYTE30_CSB              0xC0
#else
    #define BT_CMDS_BYTE30_CSB              0x00
#endif //CSB_SUPPORT
#define BT_CMDS_BYTE30                  (BT_CMDS_BYTE30_CSB | BT_CMDS_BYTE30_MWS)

/* BT command byte31
 * 0 Set Connectionless Slave Broadcast          CSB_SUPPORT
 * 1 Set Connectionless Slave Broadcast Receive  CSB_SUPPORT
 * 2 Start Synchronization Train                 CSB_SUPPORT
 * 3 Receive Synchronization Train               CSB_SUPPORT
 * 4 Set Reserved LT_ADDR                        CSB_SUPPORT
 * 5 Delete Reserved LT_ADDR                     CSB_SUPPORT
 * 6 Set Connectionless Slave Broadcast Data     CSB_SUPPORT
 * 7 Read Synchronization Train Parameters       CSB_SUPPORT
 */
#if CSB_SUPPORT
    #define BT_CMDS_BYTE31                  (0xFF)
#else // CSB_SUPPORT
    #define BT_CMDS_BYTE31                  (0x00)
#endif // CSB_SUPPORT

/* BT command byte32
 * 0 Write Synchronization Train Parameters      CSB_SUPPORT
 * 1 Remote OOB Extended Data Request Reply      Yes
 * 2 Read Secure Connections Host Support        Yes
 * 3 Write Secure Connections Host Support       Yes
 * 4 Read Authenticated Payload Timeout          Yes
 * 5 Write Authenticated Payload Timeout         Yes
 * 6 Read Local OOB Extended Data                Yes
 * 7 Write Secure Connections Test Mode          Yes
 */
#define BT_CMDS_BYTE32                  (0xFE | CSB_SUPPORT)

/* BT command byte33
 * 0 Read Extended Page Timeout                                     Yes
 * 1 Write Extended Page Timeout                                    Yes
 * 2 Read Extended Inquiry Length                                   Yes
 * 3 Write Extended Inquiry Length                                  Yes
 * 4 LE Remote Connection Parameter Request Reply Command           No
 * 5 LE Remote Connection Parameter Request Negative Reply Command  No
 * 6 Reserved                                                       No
 * 7 Reserved                                                       No
 */
#define BT_CMDS_BYTE33                  0x0F

/* BT command byte41
 * 0 HCI_LE_Set_Periodic_Advertising_Sync_Transfer_Parameters       No
 * 1 HCI_LE_Set_Default_Periodic_Advertising_Sync_Transfer_Params   No
 * 2 HCI_LE_Generate_DHKey [v2]                                     No
 * 3 HCI_Read_Local_Simple_Pairing_Options                          Yes
 * 4 HCI_LE_Modify_Sleep_Clock_Accuracy                             No
 * 5 HCI_LE                                                         No
 * 6 HCI_LE                                                         No
 * 7 HCI_LE                                                         No
 */
#define BT_CMDS_BYTE41                  0x08

/* BT command byte45
 * 0 HCI_LE                                                         No
 * 1 HCI_LE                                                         No
 * 2 HCI_LE                                                         No
 * 3 HCI_LE                                                         No
 * 4 HCI_LE                                                         No
 * 5 HCI_LE                                                         No
 * 6 HCI_LE                                                         No
 * 7 HCI_Set_Min_Encryption_Key_Size                                BT_53
 */
#if (BT_53)
    #define BT_CMDS_BYTE45                  0x80
#else // (BT_53)
    #define BT_CMDS_BYTE45                  0x00
#endif // (BT_53)

/*
 * TYPES DEFINITION
 ****************************************************************************************
 */

/// Format of a HCI command handler function
typedef int (*lm_hci_cmd_hdl_func_t)(void const *param, uint16_t opcode);


/*
 * STRUCT DEFINITION
 ****************************************************************************************
 */

/// Element of a HCI command handler table.
struct lm_hci_cmd_handler
{
    /// Command opcode
    uint16_t opcode;
    /// Pointer to the handler function for HCI command.
    lm_hci_cmd_hdl_func_t func;
};


/*
 * CONSTANT DEFINITION
 ****************************************************************************************
 */

/// Local supported commands
#if BLE_EMB_PRESENT
__STATIC uint8_t const lm_local_supp_cmds[] =
{
    BT_CMDS_BYTE0 | BLE_CMDS_BYTE0,
    BT_CMDS_BYTE1,
    BT_CMDS_BYTE2 | BLE_CMDS_BYTE2,
    BT_CMDS_BYTE3,
    BT_CMDS_BYTE4,
    BT_CMDS_BYTE5 | BLE_CMDS_BYTE5,
    BT_CMDS_BYTE6,
    BT_CMDS_BYTE7,
    BT_CMDS_BYTE8,
    BT_CMDS_BYTE9,
    BT_CMDS_BYTE10 | BLE_CMDS_BYTE10,
    BT_CMDS_BYTE11,
    BT_CMDS_BYTE12,
    BT_CMDS_BYTE13,
    BT_CMDS_BYTE14 | BLE_CMDS_BYTE14,
    BT_CMDS_BYTE15 | BLE_CMDS_BYTE15,
    BT_CMDS_BYTE16,
    BT_CMDS_BYTE17,
    BT_CMDS_BYTE18,
    BT_CMDS_BYTE19,
    BT_CMDS_BYTE20,
    BT_CMDS_BYTE21,
    BT_CMDS_BYTE22,
    BT_CMDS_BYTE23,
    BT_CMDS_BYTE24,
    BT_CMDS_BYTE25 | BLE_CMDS_BYTE25,
    BT_CMDS_BYTE26 | BLE_CMDS_BYTE26,
    BT_CMDS_BYTE27 | BLE_CMDS_BYTE27,
    BT_CMDS_BYTE28 | BLE_CMDS_BYTE28,
    BT_CMDS_BYTE29,
    BT_CMDS_BYTE30,
    BT_CMDS_BYTE31,
    BT_CMDS_BYTE32,
    BT_CMDS_BYTE33 | BLE_CMDS_BYTE33,
    BLE_CMDS_BYTE34,
    BLE_CMDS_BYTE35,
    BLE_CMDS_BYTE36,
    BLE_CMDS_BYTE37,
    BLE_CMDS_BYTE38,
    BLE_CMDS_BYTE39,
    BLE_CMDS_BYTE40,
    BT_CMDS_BYTE41 | BLE_CMDS_BYTE41,
    BLE_CMDS_BYTE42,
    BLE_CMDS_BYTE43,
    BLE_CMDS_BYTE44,
#if (BT_53)
    BT_CMDS_BYTE45 | BLE_CMDS_BYTE45,
    BLE_CMDS_BYTE46,
#else // (BT_53)
    BLE_CMDS_BYTE45,
#endif // (BT_53)
};
#else // !BLE_EMB_PRESENT
uint8_t const lm_local_supp_cmds[] =
{
    BT_CMDS_BYTE0,
    BT_CMDS_BYTE1,
    BT_CMDS_BYTE2,
    BT_CMDS_BYTE3,
    BT_CMDS_BYTE4,
    BT_CMDS_BYTE5,
    BT_CMDS_BYTE6,
    BT_CMDS_BYTE7,
    BT_CMDS_BYTE8,
    BT_CMDS_BYTE9,
    BT_CMDS_BYTE10,
    BT_CMDS_BYTE11,
    BT_CMDS_BYTE12,
    BT_CMDS_BYTE13,
    BT_CMDS_BYTE14,
    BT_CMDS_BYTE15,
    BT_CMDS_BYTE16,
    BT_CMDS_BYTE17,
    BT_CMDS_BYTE18,
    BT_CMDS_BYTE19,
    BT_CMDS_BYTE20,
    BT_CMDS_BYTE21,
    BT_CMDS_BYTE22,
    BT_CMDS_BYTE23,
    BT_CMDS_BYTE24,
    BT_CMDS_BYTE25,
    BT_CMDS_BYTE26,
    BT_CMDS_BYTE27,
    BT_CMDS_BYTE28,
    BT_CMDS_BYTE29,
    BT_CMDS_BYTE30,
    BT_CMDS_BYTE31,
    BT_CMDS_BYTE32,
    BT_CMDS_BYTE33,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    BT_CMDS_BYTE41,
    0,
    0,
    0,
#if (BT_53)
    BT_CMDS_BYTE45,
#endif // (BT_53)
};
#endif // BLE_EMB_PRESENT

/// Local supported features
uint8_t const lm_local_supp_feats[FEATURE_PAGE_MAX][FEATS_LEN] =
{
    // Page 0 (default)
    {
        FEAT_P0_BYTE0,
        FEAT_P0_BYTE1,
        FEAT_P0_BYTE2,
        FEAT_P0_BYTE3,
        FEAT_P0_BYTE4,
        FEAT_P0_BYTE5,
        FEAT_P0_BYTE6,
        FEAT_P0_BYTE7,
    },
    // Page 1
    {
        FEAT_P1_BYTE0,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
    },
    // Page 2
    {
        FEAT_P2_BYTE0,
        FEAT_P2_BYTE1,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
    }
};

/// Value for N Page depending on the remote page scan repetition mode, see table in standard BB:8.3.2
uint16_t const lm_n_page_tab[3] = { 1, 128, 256 };


/*
 * ENUMERATIONS DEFINITION
 ****************************************************************************************
 */

#if CRYPTO_UT
/// Function to be tested
enum func_id
{
    TST_F1_256 = 0x01,
    TST_F2_256 = 0x02,
    TST_F3_256 = 0x03,
    TST_G_256  = 0x04,
    TST_H3     = 0x05,
    TST_H4     = 0x06,
    TST_H5     = 0x07
};
#endif //CRYPTO_UT

/*
 * FUNCTIONS PROTOYPES
 ****************************************************************************************
 */

#if (BT_READ_PICONET_CLOCK)
    int hci_vs_rd_piconet_clock_cmd_handler(struct hci_vs_rd_piconet_clock_cmd const *param, uint16_t opcode);
#endif // (BT_READ_PICONET_CLOCK)

/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */

/// Send HCI CC event returning a status only
__STATIC void lm_cmd_cmp_send(uint16_t opcode, uint8_t status)
{
    // allocate the complete event message
    struct hci_basic_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_cmd_cmp_evt);
    // update the status
    evt->status = status;
    // send the message
    hci_send_2_host(evt);
}

/// Send HCI CC event returning a status and a BD address
__STATIC void lm_cmd_cmp_bd_addr_send(uint16_t opcode, uint8_t status, struct bd_addr const *bd_addr)
{
    struct hci_basic_bd_addr_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_bd_addr_cmd_cmp_evt);
    event->status = status;
    memcpy(&event->bd_addr, bd_addr, BD_ADDR_LEN);
    hci_send_2_host(event);
}

/// Send HCI CS event
__STATIC void lm_cmd_stat_send(uint16_t opcode, uint8_t status)
{
    // allocate the status event message
    struct hci_cmd_stat_event *evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, 0, opcode, hci_cmd_stat_event);
    // update the status
    evt->status = status;
    // send the message
    hci_send_2_host(evt);
}

/// Check if a BD address is in the inquiry result filtering list and add it if it is not
__STATIC bool lm_inq_reports_list_check(const struct bd_addr *addr_to_add)
{
    bool found = false;

    // Check for duplicates until the BD address is found or the first NULL BD address is encountered
    for (uint8_t i = 0; i < INQ_FILT_LEN; i++)
    {
        if (co_bdaddr_compare(&lm_env.inq_filt.dev[i], addr_to_add))
        {
            found = true;
            break;
        }
        else if (co_bdaddr_compare(&lm_env.inq_filt.dev[i], &co_null_bdaddr))
        {
            break;
        }
    }

    // If not in the list
    if (!found)
    {
        // Add the BD address to the list
        memcpy(&lm_env.inq_filt.dev[lm_env.inq_filt.curr_pos].addr[0], &addr_to_add->addr[0], BD_ADDR_LEN);
        // Advance the current position
        lm_env.inq_filt.curr_pos = CO_MOD(lm_env.inq_filt.curr_pos + 1, INQ_FILT_LEN);
    }

    return (found);
}

#if RW_BT_MWS_COEX
// Configure SAM index
__STATIC void lm_sam_index_config(uint8_t pattern_index, rwip_time_t time)
{
    struct lm_sam_pattern *sam_pattern = NULL;
    uint8_t t_sam_sm = 0;
    uint16_t t_sam = 0;
    bool update = false;

    if (pattern_index == SAM_DISABLED)
    {
        update = true;
    }
    else
    {
        sam_pattern = &lm_env.sam_info.pattern[pattern_index];

        // If the pattern is defined, inform all links to update to this pattern
        if (sam_pattern->n_sam_sm)
        {
            t_sam_sm = sam_pattern->t_sam_sm;
            t_sam = t_sam_sm * (sam_pattern->n_sam_sm);

            update = true;
        }
    }

    if (update)
    {
        lm_env.sam_info.active_index = pattern_index;

        // Search for active links(s)
        for (int i = 0; i < MAX_NB_ACTIVE_ACL; i++)
        {
            if (lm_env.con_info[i].state == LM_CONNECTED)
            {
                // Send an indication to LC to trigger SAM switch on the link
                struct lc_mws_pattern_ind *msg = KE_MSG_ALLOC(LC_MWS_PATTERN_IND, KE_BUILD_ID(TASK_LC, i), TASK_LM, lc_mws_pattern_ind);

                msg->pattern_index = pattern_index;

                if (pattern_index != SAM_DISABLED)
                {
                    msg->t_sam = t_sam;
                    msg->t_sam_sm = t_sam_sm;
                    msg->n_tx_slots = sam_pattern->n_tx_slots;
                    msg->n_rx_slots = sam_pattern->n_rx_slots;
                    msg->n_ex_sm = sam_pattern->n_ex_sm;
                }

                msg->time.hs = time.hs;
                msg->time.hus = time.hus;

                ke_msg_send(msg);
            }
        }
    }
}
#endif // RW_BT_MWS_COEX

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/*
 * HCI LINK CONTROL COMMANDS HANDLERS
 ****************************************************************************************
 */

/// Handle the command HCI inquiry
HCI_CMD_HANDLER(inq, struct hci_inq_cmd)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // Check if inquiry procedure ongoing
    if (lm_env.inq_state == LM_INQ_OFF)
    {
        // Check HCI parameters
        if ((param->inq_len < INQ_LEN_MIN)  || (param->inq_len > INQ_LEN_MAX)
                || (param->lap.A[2] != GIAC_LAP_2) || (param->lap.A[1] != GIAC_LAP_1) || (param->lap.A[0] > DIAC_MAX_LAP_0))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
        else
        {
            // Start Inquiry
            struct ld_inquiry_params inq_par;
            uint8_t nb_sync = 0;
#if (MAX_NB_SYNC > 0)
            nb_sync = lm_get_nb_sync_link();
#endif //(MAX_NB_SYNC > 0)
            memcpy(&inq_par.lap, &param->lap, sizeof(struct lap));
            inq_par.inq_len  = param->inq_len + (lm_env.hci.ext_inq_len >> 11);
            inq_par.per_min  = 0;
            inq_par.per_max  = 0;
            inq_par.nb_rsp_max = param->nb_rsp;
            inq_par.tx_pwr_lvl = lm_env.hci.inq_tx_pwr_lvl;
            inq_par.eir_en   = (lm_env.hci.inq_mode == EXTENDED_INQUIRY);
            // Calculate the value of n_inq according to the number of synchronous links
            if (nb_sync == 0)
            {
                inq_par.n_inq = 256;
            }
            else if (nb_sync == 1)
            {
                inq_par.n_inq = 512;
            }
            else
            {
                inq_par.n_inq = 768;
            }

#if (RW_BT_MWS_COEX)
            // Train nudging enabled for COEX where slots to receive are periodically not available
            inq_par.knudge_en = lm_local_ext_fr_configured();
#else // !(RW_BT_MWS_COEX)
            inq_par.knudge_en = 0;
#endif // !(RW_BT_MWS_COEX)

            status = ld_inq_start(&inq_par);

            ASSERT_ERR(status == CO_ERROR_NO_ERROR);

            lm_env.inq_state = LM_INQ_NORMAL;

            // Initialize the inquiry result filtering list
            memset(&lm_env.inq_filt, 0, sizeof(lm_env.inq_filt));
        }
    }

    // Send CS event
    lm_cmd_stat_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI inquiry cancel
HCI_CMD_HANDLER(inq_cancel, void)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // Check if a normal inquiry procedure is ongoing
    if (lm_env.inq_state == LM_INQ_NORMAL)
    {
        // Stop Inquiry
        status = ld_inq_stop();

        ASSERT_ERR(status == CO_ERROR_NO_ERROR);

        lm_env.inq_state = LM_INQ_OFF;
    }

    // Send CC event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI periodic inquiry mode
HCI_CMD_HANDLER(per_inq_mode, struct hci_per_inq_mode_cmd)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // Check if an inquiry procedure is ongoing
    if (lm_env.inq_state == LM_INQ_OFF)
    {
        // Check HCI parameters
        if ((param->max_per_len <= param->min_per_len)  || (param->min_per_len <= (param->inq_len + (lm_env.hci.ext_inq_len >> 11))) ||
                (param->min_per_len < INQ_MIN_PER_LEN_MIN) || (param->inq_len < INQ_LEN_MIN) || (param->inq_len > INQ_LEN_MAX))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
        else
        {
            // Start Inquiry
            struct ld_inquiry_params inq_par;
            uint8_t nb_sync = 0;
#if (MAX_NB_SYNC > 0)
            nb_sync = lm_get_nb_sync_link();
#endif //(MAX_NB_SYNC > 0)
            memcpy(&inq_par.lap, &param->lap, sizeof(struct lap));
            inq_par.inq_len    = param->inq_len + (lm_env.hci.ext_inq_len >> 11);
            inq_par.per_min    = param->min_per_len;
            inq_par.per_max    = param->max_per_len;
            inq_par.nb_rsp_max = param->nb_rsp;
            inq_par.tx_pwr_lvl = lm_env.hci.inq_tx_pwr_lvl;
            inq_par.eir_en     = (lm_env.hci.inq_mode == EXTENDED_INQUIRY);
            // Calculate the value of n_inq according to the number of synchronous links
            if (nb_sync == 0)
            {
                inq_par.n_inq = 256;
            }
            else if (nb_sync == 1)
            {
                inq_par.n_inq = 512;
            }
            else
            {
                inq_par.n_inq = 768;
            }

#if (RW_BT_MWS_COEX)
            // Train nudging enabled for COEX where slots to receive are periodically not available
            inq_par.knudge_en = lm_local_ext_fr_configured();
#else // !(RW_BT_MWS_COEX)
            inq_par.knudge_en = 0;
#endif // !(RW_BT_MWS_COEX)

            status = ld_inq_start(&inq_par);

            ASSERT_ERR(status == CO_ERROR_NO_ERROR);

            lm_env.inq_state = LM_INQ_PERIODIC;

            // Initialize the inquiry result filtering list
            memset(&lm_env.inq_filt, 0, sizeof(lm_env.inq_filt));
        }
    }

    // Send CC event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI exit periodic inquiry mode
HCI_CMD_HANDLER(exit_per_inq_mode, void)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // Check if a periodic inquiry procedure is ongoing
    if (lm_env.inq_state == LM_INQ_PERIODIC)
    {
        // Stop Inquiry
        status = ld_inq_stop();

        ASSERT_ERR(status == CO_ERROR_NO_ERROR);

        lm_env.inq_state = LM_INQ_OFF;
    }

    // Send CC event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI create connection
HCI_CMD_HANDLER(create_con, struct hci_create_con_cmd)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    do
    {
        uint8_t link_id = 0;

        // Check if Page procedure ongoing or if connection to this BD Address already exists
        for (link_id = 0 ; link_id < MAX_NB_ACTIVE_ACL ; link_id++)
        {
            // Check state
            if (lm_env.con_info[link_id].state == LM_PAGE)
            {
                break;
            }
            if ((lm_env.con_info[link_id].state >= LM_CONNECTED)
                    && !memcmp(&lm_env.con_info[link_id].bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN))
            {
                status = CO_ERROR_CON_ALREADY_EXISTS;
                break;
            }
        }

        // Check if link identifier found
        if (link_id < MAX_NB_ACTIVE_ACL)
        {
            break;
        }

        // Check HCI parameters
        if ((param->page_scan_rep_mode > R2) || (param->switch_en > ROLE_SWITCH_ALLOWED))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // Allocate link identifier
        for (link_id = 0 ; link_id < MAX_NB_ACTIVE_ACL ; link_id++)
        {
            // Check state
            if (lm_env.con_info[link_id].state == LM_FREE)
            {
                break;
            }
        }

        // Check if link identifier found
        if (link_id >= MAX_NB_ACTIVE_ACL)
        {
            status = CO_ERROR_CON_LIMIT_EXCEED;
            break;
        }

        // Allocate LT address
        lm_env.con_info[link_id].lt_addr = lm_lt_addr_alloc();

        // Check if free LT address found
        if (lm_env.con_info[link_id].lt_addr == 0x00)
        {
            status = CO_ERROR_CON_LIMIT_EXCEED;
            break;
        }

        // Connection can be started on this link ID
        {
            // Start Page
            struct ld_page_params page_par;
            memcpy(&page_par.bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN);
            page_par.link_id = link_id;
            page_par.page_to = lm_env.hci.page_to + lm_env.hci.ext_page_to;

            if (param->clk_off & CLK_OFFSET_VALID_FLAG_MSK)
            {
                page_par.clk_off = (param->clk_off & ~CLK_OFFSET_VALID_FLAG_MSK) << 1;
            }
            else
            {
                page_par.clk_off = 0;
            }

            page_par.lt_addr = lm_env.con_info[link_id].lt_addr;
            page_par.n_page = lm_n_page_tab[param->page_scan_rep_mode];
            page_par.page_scan_rep_mode = lm_env.hci.page_scan_rep_mode;
#if (MAX_NB_SYNC > 0)
            page_par.n_page *= (1 + lm_get_nb_sync_link());
#endif //(MAX_NB_SYNC > 0)
            page_par.truncated = false;

#if (RW_BT_MWS_COEX)
            // Train nudging enabled for COEX where slots to receive are periodically not available
            page_par.knudge_en = lm_local_ext_fr_configured();
#else // !(RW_BT_MWS_COEX)
            page_par.knudge_en = 0;
#endif // !(RW_BT_MWS_COEX)

            status = ld_page_start(&page_par);

            // Set current connection state
            lm_env.con_info[link_id].state = LM_PAGE;
            memcpy(&lm_env.con_info[link_id].bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN);

            // Store pointer to HCI command
            lm_env.create_con_cmd = param;

            ASSERT_ERR(status == CO_ERROR_NO_ERROR);

            // Send CS event
            lm_cmd_stat_send(opcode, status);

            return (KE_MSG_NO_FREE);
        }

    }
    while (0);

    // Send CS event
    lm_cmd_stat_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI cancel create connection
HCI_CMD_HANDLER(create_con_cancel, struct hci_basic_bd_addr_cmd)
{
    uint8_t link_id = 0;

    // Find link identifier associated to BD Address
    for (link_id = 0 ; link_id < MAX_NB_ACTIVE_ACL ; link_id++)
    {
        // Check state
        if ((lm_env.con_info[link_id].state != LM_FREE)
                && !memcmp(&lm_env.con_info[link_id].bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN))
        {
            break;
        }
    }

    // Check if link identifier found
    if (link_id < MAX_NB_ACTIVE_ACL)
    {
        switch (lm_env.con_info[link_id].state)
        {
        case LM_PAGE:
        {
            ASSERT_ERR((lm_env.create_con_cmd != NULL) || (lm_env.rem_name_req_cmd != NULL)
#if CSB_SUPPORT
                       || (lm_env.trunc_page_cmd != NULL)
#endif //CSB_SUPPORT
                      );

            // Check if the Page concerns a connection creation
            if (lm_env.create_con_cmd != NULL)
            {
                ASSERT_ERR(!memcmp(&param->bd_addr.addr[0], &lm_env.create_con_cmd->bd_addr.addr[0], BD_ADDR_LEN));

                // Stop Page
                uint8_t status = ld_page_stop();

                if (status == CO_ERROR_COMMAND_DISALLOWED)
                {
                    /*
                     * LM is waiting for Page End indication, whereas LD has finished Page ...
                     * => Re-post message in order to re-process after Page End indication
                     */
                    ke_msg_forward(param, TASK_LM, opcode);

                    return KE_MSG_NO_FREE;
                }

                // Indicate page is stopping
                lm_env.con_info[link_id].state = LM_PAGE_STOPPING;
            }
            else
            {
                // Send CC event
                lm_cmd_cmp_bd_addr_send(opcode, CO_ERROR_UNKNOWN_CONNECTION_ID, &param->bd_addr);
            }
        }
        break;
        case LM_CONNECTED:
        case LM_SWITCH:
        {
            // Forward to LC
            ke_msg_forward(param, KE_BUILD_ID(TASK_LC, link_id), opcode);

            return KE_MSG_NO_FREE;
        }
        default:
        {
        }
        break;
        }
    }
    else
    {
        // Send CC event
        lm_cmd_cmp_bd_addr_send(opcode, CO_ERROR_UNKNOWN_CONNECTION_ID, &param->bd_addr);
    }

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI remote name request
HCI_CMD_HANDLER(rem_name_req, struct hci_rem_name_req_cmd)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    do
    {
        uint8_t link_id = 0;

        // Check HCI parameters
        if (param->page_scan_rep_mode > R2)
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // Check if connection to this BD Address already exists
        for (link_id = 0 ; link_id < MAX_NB_ACTIVE_ACL ; link_id++)
        {
            // Check state
            if ((lm_env.con_info[link_id].state >= LM_CONNECTED)
                    && !memcmp(&lm_env.con_info[link_id].bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN))
            {
                // Forward to LC
                ke_msg_forward(param, KE_BUILD_ID(TASK_LC, link_id), opcode);

                // Send CS event
                lm_cmd_stat_send(opcode, CO_ERROR_NO_ERROR);

                return KE_MSG_NO_FREE;
            }
        }

        // Check if Page procedure ongoing
        for (link_id = 0 ; link_id < MAX_NB_ACTIVE_ACL ; link_id++)
        {
            // Check state
            if (lm_env.con_info[link_id].state == LM_PAGE)
            {
                break;
            }
        }

        // Check if link identifier found
        if (link_id < MAX_NB_ACTIVE_ACL)
        {
            break;
        }

        // Allocate link identifier
        for (link_id = 0 ; link_id < MAX_NB_ACTIVE_ACL ; link_id++)
        {
            // Check state
            if (lm_env.con_info[link_id].state == LM_FREE)
            {
                break;
            }
        }

        // Check if link identifier found
        if (link_id >= MAX_NB_ACTIVE_ACL)
        {
            status = CO_ERROR_CON_LIMIT_EXCEED;
            break;
        }

        // Allocate LT address
        lm_env.con_info[link_id].lt_addr = lm_lt_addr_alloc();

        // Check if free LT address found
        if (lm_env.con_info[link_id].lt_addr == 0x00)
        {
            status = CO_ERROR_CON_LIMIT_EXCEED;
            break;
        }

        // Connection can be started on this link ID
        {
            // Start Page
            struct ld_page_params page_par;
            memcpy(&page_par.bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN);
            page_par.link_id = link_id;
            page_par.page_to = lm_env.hci.page_to + lm_env.hci.ext_page_to;
            if (param->clk_off & CLK_OFFSET_VALID_FLAG_MSK)
            {
                page_par.clk_off = (param->clk_off & ~CLK_OFFSET_VALID_FLAG_MSK) << 1;
            }
            else
            {
                page_par.clk_off = 0;
            }
            page_par.lt_addr = lm_env.con_info[link_id].lt_addr;
            page_par.n_page = lm_n_page_tab[param->page_scan_rep_mode];
#if (MAX_NB_SYNC > 0)
            page_par.n_page *= (1 + lm_get_nb_sync_link());
#endif //(MAX_NB_SYNC > 0)
            page_par.page_scan_rep_mode = param->page_scan_rep_mode;
            page_par.truncated = false;

#if (RW_BT_MWS_COEX)
            // Train nudging enabled for COEX where slots to receive are periodically not available
            page_par.knudge_en = lm_local_ext_fr_configured();
#else // !(RW_BT_MWS_COEX)
            page_par.knudge_en = 0;
#endif // !(RW_BT_MWS_COEX)

            status = ld_page_start(&page_par);

            // Set current connection state
            lm_env.con_info[link_id].state = LM_PAGE;
            memcpy(&lm_env.con_info[link_id].bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN);

            // Store pointer to HCI command
            lm_env.rem_name_req_cmd = param;

            ASSERT_ERR(status == CO_ERROR_NO_ERROR);

            // Send CS event
            lm_cmd_stat_send(opcode, status);

            return (KE_MSG_NO_FREE);
        }

    }
    while (0);

    // Send CS event
    lm_cmd_stat_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI remote name cancel cancel
HCI_CMD_HANDLER(rem_name_req_cancel, struct hci_basic_bd_addr_cmd)
{
    uint8_t link_id = 0;

    // Find link identifier associated to BD Address
    for (link_id = 0 ; link_id < MAX_NB_ACTIVE_ACL ; link_id++)
    {
        // Check state
        if ((lm_env.con_info[link_id].state != LM_FREE)
                && !memcmp(&lm_env.con_info[link_id].bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN))
        {
            break;
        }
    }

    // Check if link identifier found
    if (link_id < MAX_NB_ACTIVE_ACL)
    {
        switch (lm_env.con_info[link_id].state)
        {
        case LM_PAGE:
        {
            ASSERT_ERR((lm_env.create_con_cmd != NULL) || (lm_env.rem_name_req_cmd != NULL)
#if CSB_SUPPORT
                       || (lm_env.trunc_page_cmd != NULL)
#endif //CSB_SUPPORT
                      );

            // Check if the Page concerns a remote name request
            if (lm_env.rem_name_req_cmd != NULL)
            {
                ASSERT_ERR(!memcmp(&param->bd_addr.addr[0], &lm_env.rem_name_req_cmd->bd_addr.addr[0], BD_ADDR_LEN));

                // Stop Page
                uint8_t status = ld_page_stop();

                if (status == CO_ERROR_COMMAND_DISALLOWED)
                {
                    /*
                     * LM is waiting for Page End indication, whereas LD has finished Page ...
                     * => Re-post message in order to re-process after Page End indication
                     */
                    ke_msg_forward(param, TASK_LM, opcode);

                    return KE_MSG_NO_FREE;
                }

                // Indicate page is stopping
                lm_env.con_info[link_id].state = LM_PAGE_STOPPING;
            }
            else
            {
                // Send CC event
                lm_cmd_cmp_bd_addr_send(opcode, CO_ERROR_INVALID_HCI_PARAM, &param->bd_addr);
            }
        }
        break;
        case LM_CONNECTED:
        case LM_SWITCH:
        {
            // Forward to LC
            ke_msg_forward(param, KE_BUILD_ID(TASK_LC, link_id), opcode);

            return KE_MSG_NO_FREE;
        }
        default:
        {
        }
        break;
        }
    }
    else
    {
        // Send CC event
        lm_cmd_cmp_bd_addr_send(opcode, CO_ERROR_INVALID_HCI_PARAM, &param->bd_addr);
    }

    return (KE_MSG_CONSUMED);
}

#if CSB_SUPPORT
/// Handle the command HCI Truncated Page
HCI_CMD_HANDLER(trunc_page, struct hci_trunc_page_cmd)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    do
    {
        uint8_t link_id = 0;

        // Check if Page procedure ongoing or if connection to this BD Address already exists
        for (link_id = 0 ; link_id < MAX_NB_ACTIVE_ACL ; link_id++)
        {
            // Check state
            if (lm_env.con_info[link_id].state == LM_PAGE)
            {
                break;
            }
            if ((lm_env.con_info[link_id].state >= LM_CONNECTED)
                    && !memcmp(&lm_env.con_info[link_id].bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN))
            {
                status = CO_ERROR_CON_ALREADY_EXISTS;
                break;
            }
        }

        // Check if link identifier found
        if (link_id < MAX_NB_ACTIVE_ACL)
        {
            break;
        }

        // Check HCI parameters
        if (param->page_scan_rep_mode > R2)
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // Allocate link identifier
        for (link_id = 0 ; link_id < MAX_NB_ACTIVE_ACL ; link_id++)
        {
            // Check state
            if (lm_env.con_info[link_id].state == LM_FREE)
            {
                break;
            }
        }

        // Check if link identifier found
        if (link_id >= MAX_NB_ACTIVE_ACL)
        {
            status = CO_ERROR_CON_LIMIT_EXCEED;
            break;
        }

        // Truncated page can be started on this link ID
        {
            // Start Page
            struct ld_page_params page_par;
            memcpy(&page_par.bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN);
            page_par.link_id = link_id;
            page_par.page_to = lm_env.hci.page_to + lm_env.hci.ext_page_to;

            if (param->clk_off & CLK_OFFSET_VALID_FLAG_MSK)
            {
                page_par.clk_off = (param->clk_off & ~CLK_OFFSET_VALID_FLAG_MSK) << 1;
            }
            else
            {
                page_par.clk_off = 0;
            }

            page_par.lt_addr = 0;
            page_par.n_page = lm_n_page_tab[param->page_scan_rep_mode];
#if (MAX_NB_SYNC > 0)
            page_par.n_page *= (1 + lm_get_nb_sync_link());
#endif //(MAX_NB_SYNC > 0)
            page_par.truncated = true;

#if (RW_BT_MWS_COEX)
            // Train nudging enabled for COEX where slots to receive are periodically not available
            page_par.knudge_en = lm_local_ext_fr_configured();
#else // !(RW_BT_MWS_COEX)
            page_par.knudge_en = 0;
#endif // !(RW_BT_MWS_COEX)

            status = ld_page_start(&page_par);

            // Set current connection state
            lm_env.con_info[link_id].state = LM_PAGE;
            memcpy(&lm_env.con_info[link_id].bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN);

            // Store pointer to HCI command
            lm_env.trunc_page_cmd = param;

            ASSERT_ERR(status == CO_ERROR_NO_ERROR);

            // Send CS event
            lm_cmd_stat_send(opcode, status);

            return (KE_MSG_NO_FREE);
        }

    }
    while (0);

    // Send CS event
    lm_cmd_stat_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI truncated page cancel connection
HCI_CMD_HANDLER(trunc_page_can, struct hci_trunc_page_can_cmd)
{
    uint8_t link_id = 0;

    // Find link identifier associated to BD Address
    for (link_id = 0 ; link_id < MAX_NB_ACTIVE_ACL ; link_id++)
    {
        // Check state
        if ((lm_env.con_info[link_id].state != LM_FREE)
                && !memcmp(&lm_env.con_info[link_id].bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN))
        {
            break;
        }
    }

    // Check if link identifier found
    if (link_id < MAX_NB_ACTIVE_ACL)
    {
        switch (lm_env.con_info[link_id].state)
        {
        case LM_PAGE:
        {
            ASSERT_ERR((lm_env.create_con_cmd != NULL) || (lm_env.rem_name_req_cmd != NULL) || (lm_env.trunc_page_cmd != NULL));

            // Check if the Page concerns a connection creation
            if (lm_env.trunc_page_cmd != NULL)
            {
                ASSERT_ERR(!memcmp(&param->bd_addr.addr[0], &lm_env.trunc_page_cmd->bd_addr.addr[0], BD_ADDR_LEN));

                // Stop Page
                uint8_t status = ld_page_stop();

                if (status == CO_ERROR_COMMAND_DISALLOWED)
                {
                    /*
                     * LM is waiting for Page End indication, whereas LD has finished Page ...
                     * => Re-post message in order to re-process after Page End indication
                     */
                    ke_msg_forward(param, TASK_LM, opcode);

                    return KE_MSG_NO_FREE;
                }

                // Indicate page is stopping
                lm_env.con_info[link_id].state = LM_PAGE_STOPPING;
            }
            else
            {
                // Send CC event
                lm_cmd_cmp_bd_addr_send(opcode, CO_ERROR_UNKNOWN_CONNECTION_ID, &param->bd_addr);
            }
        }
        break;
        default:
        {
            // Send CC event
            lm_cmd_cmp_bd_addr_send(opcode, CO_ERROR_CON_ALREADY_EXISTS, &param->bd_addr);
        }
        break;
        }
    }
    else
    {
        // Send CC event
        lm_cmd_cmp_bd_addr_send(opcode, CO_ERROR_UNKNOWN_CONNECTION_ID, &param->bd_addr);
    }

    return (KE_MSG_CONSUMED);
}
#endif //CSB_SUPPORT


/*
 * HCI LINK POLICY COMMANDS HANDLERS
 ****************************************************************************************
 */

/// Handle the command HCI read default link policy settings
HCI_CMD_HANDLER(rd_dft_link_pol_stg, void)
{
    // allocate the complete event message
    struct hci_rd_dft_link_pol_stg_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_dft_link_pol_stg_cmd_cmp_evt);

    evt->link_pol_stg = lm_env.hci.link_pol_stg;
    evt->status = CO_ERROR_NO_ERROR;

    // send message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write default link policy settings
HCI_CMD_HANDLER(wr_dft_link_pol_stg, struct hci_wr_dft_link_pol_stg_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    // Check if all bits set are known
    if ((param->link_pol_stg & ~(POLICY_SWITCH | POLICY_HOLD | POLICY_SNIFF | POLICY_PARK)) == 0)
    {
        status = CO_ERROR_NO_ERROR;
        lm_env.hci.link_pol_stg = param->link_pol_stg;
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}


/*
 * HCI CONTROL & BASEBAND COMMANDS HANDLERS
 ****************************************************************************************
 */

/// Handles the command HCI set event mask.
HCI_CMD_HANDLER(set_evt_mask, struct hci_set_evt_mask_cmd)
{
    // Set the event mask in the HCI
    uint8_t status = hci_evt_mask_set(&param->event_mask, HCI_PAGE_DFT);

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI set event mask page 2.
HCI_CMD_HANDLER(set_evt_mask_page_2, struct hci_set_evt_mask_cmd)
{
    // Set the event mask in the HCI
    uint8_t status = hci_evt_mask_set(&param->event_mask, HCI_PAGE_2);

    // send the command complete event message
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}
/// Handles the command HCI reset
HCI_CMD_HANDLER(reset, void)
{
    // Reset BT
    rwip_reset();

    // Send the command complete event
    lm_cmd_cmp_send(opcode, CO_ERROR_NO_ERROR);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI set event mask.
HCI_CMD_HANDLER(set_evt_filter, struct hci_set_evt_filter_cmd)
{
    uint8_t status = hci_evt_filter_add(param);

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read PIN type
HCI_CMD_HANDLER(rd_pin_type, void)
{
    // allocate the complete event message
    struct hci_rd_pin_type_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_pin_type_cmd_cmp_evt);
    evt->status = CO_ERROR_NO_ERROR;
    evt->pin_type = lm_env.hci.pin_type;

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write PIN type
HCI_CMD_HANDLER(wr_pin_type, struct hci_wr_pin_type_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    if ((param->pin_type == VARIABLE_PIN) || (param->pin_type == FIXED_PIN))
    {
        lm_env.hci.pin_type = param->pin_type;
        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read stored link key
HCI_CMD_HANDLER(rd_stored_lk, struct hci_rd_stored_lk_cmd)
{
    // allocate the complete event message
    struct hci_rd_stored_lk_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_stored_lk_cmd_cmp_evt);

    evt->num_key_rd = 0;

    // Get the max number of link key which can be stored
    evt->num_key_max = lm_env.nb_stored_link_keys;

    if (param->rd_all_flag == LINK_KEY_BD_ADDR)
    {
        if (lm_look_for_stored_link_key((struct bd_addr *) &param->bd_addr, NULL))
        {
            // Report the link key to the Host
            struct hci_return_link_keys_evt *evt2 = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_RETURN_LINK_KEYS_EVT_CODE, hci_return_link_keys_evt);
            memcpy(&evt2->bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN);
            // The link keys value parameter shall always contain the value of zero
            memset(&evt2->key.ltk[0], 0, KEY_LEN);
            evt2->num_keys = 1;
            hci_send_2_host(evt2);

            evt->num_key_rd = 1;
            evt->status = CO_ERROR_NO_ERROR;
        }
        else
        {
            evt->status = CO_ERROR_UNSPECIFIED_ERROR;
        }
    }
    else if (param->rd_all_flag == LINK_KEY_ALL)
    {
        evt->status = CO_ERROR_NO_ERROR;

        // Parse all entries in storage
        for (int i = 0; i < lm_env.nb_stored_link_keys ; i++)
        {
            uint8_t buffer[PARAM_LEN_BT_LINK_KEY];
            uint8_t length = PARAM_LEN_BT_LINK_KEY;

            // Fetch entry from storage
            if (rwip_param.get(PARAM_ID_BT_LINK_KEY_FIRST + i, &length, buffer) == PARAM_OK)
            {
                // Report the link key to the Host
                struct hci_return_link_keys_evt *evt2 = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_RETURN_LINK_KEYS_EVT_CODE, hci_return_link_keys_evt);
                memcpy(&evt2->bd_addr.addr[0], &buffer[0], BD_ADDR_LEN);
                // The link keys value parameter shall always contain the value of zero
                memset(&evt2->key.ltk[0], 0, KEY_LEN);
                evt2->num_keys = 1;
                hci_send_2_host(evt2);

                // Increment number of key returned
                (evt->num_key_rd)++;
            }
        }
    }
    else
    {
        evt->status = CO_ERROR_INVALID_HCI_PARAM;
    }

    // Send CC event
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write stored link key
HCI_CMD_HANDLER(wr_stored_lk, struct hci_wr_stored_lk_cmd)
{
    // allocate the complete event message
    struct hci_wr_stored_lk_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_wr_stored_lk_cmd_cmp_evt);
    evt->num_key_wr = 0;

    evt->status = CO_ERROR_NO_ERROR;

    // Check every given 'BD address + key' entry
    while (evt->num_key_wr < param->num_key_wr)
    {
        uint8_t position = lm_env.nb_stored_link_keys;

        // Parse all entries in storage
        for (int i = lm_env.nb_stored_link_keys - 1; i >= 0 ; i--)
        {
            // Look if tag is present
            uint8_t buffer[PARAM_LEN_BT_LINK_KEY];
            uint8_t length = PARAM_LEN_BT_LINK_KEY;

            // Fetch entry from storage
            if (rwip_param.get(PARAM_ID_BT_LINK_KEY_FIRST + i, &length, buffer) == PARAM_OK)
            {
                // Check BD address
                if (!memcmp(&param->link_keys[evt->num_key_wr].bd_addr.addr[0], buffer, BD_ADDR_LEN))
                {
                    position = i;
                    break;
                }
            }
            else
            {
                // Save the free parameter ID
                position = i;
            }
        }

        // If BD address or free entry found
        if (position < lm_env.nb_stored_link_keys)
        {
            // Write into NVDS
            if (rwip_param.set(PARAM_ID_BT_LINK_KEY_FIRST + position, sizeof(struct bd_addr_plus_key), (uint8_t *)&param->link_keys[evt->num_key_wr]) != PARAM_OK)
            {
                evt->status = CO_ERROR_HARDWARE_FAILURE;
            }
        }
        else
        {
            evt->status = CO_ERROR_MEMORY_CAPA_EXCEED;
        }

        // Check status
        if (evt->status != CO_ERROR_NO_ERROR)
            break;

        // Increment counter
        evt->num_key_wr++;
    }

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI delete stored link key
HCI_CMD_HANDLER(del_stored_lk, struct hci_del_stored_lk_cmd)
{
    // allocate the complete event message
    struct hci_del_stored_lk_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_del_stored_lk_cmd_cmp_evt);

    // Initialize number of link key erased
    evt->num_key_del = 0;

    evt->status = CO_ERROR_NO_ERROR;

    if (param->del_all_flag == LINK_KEY_BD_ADDR)
    {
        // Parse all entries in storage
        for (int i = lm_env.nb_stored_link_keys - 1; i >= 0 ; i--)
        {
            uint8_t buffer[PARAM_LEN_BT_LINK_KEY];
            uint8_t length = PARAM_LEN_BT_LINK_KEY;

            // Fetch entry from storage
            if (rwip_param.get(PARAM_ID_BT_LINK_KEY_FIRST + i, &length, buffer) == PARAM_OK)
            {
                // Check BD address
                if (!memcmp(&param->bd_addr.addr[0], buffer, BD_ADDR_LEN))
                {
                    // Delete entry
                    if (rwip_param.del(PARAM_ID_BT_LINK_KEY_FIRST + i) == PARAM_OK)
                    {
                        evt->num_key_del = 1;
                    }
                    else
                    {
                        evt->status = CO_ERROR_HARDWARE_FAILURE;
                    }
                    break;
                }
            }
        }
    }
    else if (param->del_all_flag == LINK_KEY_ALL)
    {
        // Parse all entries in storage
        for (int i = lm_env.nb_stored_link_keys - 1; i >= 0 ; i--)
        {
            // Delete entry
            uint8_t status = rwip_param.del(PARAM_ID_BT_LINK_KEY_FIRST + i);

            if (status == PARAM_OK)
            {
                evt->num_key_del++;
            }
            else if (status != PARAM_ID_NOT_DEFINED)
            {
                evt->status = CO_ERROR_HARDWARE_FAILURE;
                break;
            }
        }
    }
    else
    {
        evt->status = CO_ERROR_INVALID_HCI_PARAM;
    }

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write local name
HCI_CMD_HANDLER(wr_local_name, struct hci_wr_local_name_cmd)
{
    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t name_len = strlen((char *) param->name.name);
    name_len = co_min(name_len, BD_NAME_SIZE);

    do
    {
        // Check if a name is present
        if (lm_env.local_name != NULL)
        {
            // Compare current name and new name
            if (!strncmp((char *) &param->name.name[0], lm_env.local_name, BD_NAME_SIZE))
                break;

            // Free current name buffer
            ke_free(lm_env.local_name);
        }

        // Allocate new buffer from Heap
        lm_env.local_name = ke_malloc_system(CO_ALIGN4_HI(name_len + 1), KE_MEM_ENV);

        if (lm_env.local_name != NULL)
        {
            // Copy name to buffer
            memcpy(lm_env.local_name, param->name.name, name_len);
            lm_env.local_name[name_len] = '\0';

            // Lib function strlen() has been found to use 32-bit memory accesses, so can perform read-access a few bytes beyond end of string
            DBG_MEM_INIT(&lm_env.local_name[name_len], CO_ALIGN4_HI(name_len + 1) - name_len);
        }
        else
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
        }
    }
    while (0);

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read local name
HCI_CMD_HANDLER(rd_local_name, void)
{
    // allocate the complete event message
    struct hci_rd_local_name_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_local_name_cmd_cmp_evt);

    // Check if a name is present
    if (lm_env.local_name != NULL)
    {
        // Copy name with zero padding
        strncpy((char *) &evt->name[0], lm_env.local_name, BD_NAME_SIZE);
    }
    else
    {
        // Initialize with '\0' values
        memset(&evt->name[0], '\0', BD_NAME_SIZE);
    }

    evt->status = CO_ERROR_NO_ERROR;
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read connection accept timeout
HCI_CMD_HANDLER(rd_con_accept_to, void)
{
    // allocate the complete event message
    struct hci_rd_con_accept_to_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_con_accept_to_cmd_cmp_evt);
    evt->status = CO_ERROR_NO_ERROR;
    evt->con_acc_to = hci_con_accept_to_get();
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write connection accept timeout
HCI_CMD_HANDLER(wr_con_accept_to, struct hci_wr_con_accept_to_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    if ((param->con_acc_to >= CON_ACCEPT_TO_MIN) && (param->con_acc_to <= CON_ACCEPT_TO_MAX))
    {
        hci_con_accept_to_set(param->con_acc_to);
        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read page timeout
HCI_CMD_HANDLER(rd_page_to, void)
{
    // allocate the complete event message
    struct hci_rd_page_to_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_page_to_cmd_cmp_evt);
    evt->status = CO_ERROR_NO_ERROR;
    evt->page_to = lm_env.hci.page_to;
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write page timeout
HCI_CMD_HANDLER(wr_page_to, struct hci_wr_page_to_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    if (param->page_to >= PAGE_TO_MIN)
    {
        lm_env.hci.page_to = param->page_to;
        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read scan enable
HCI_CMD_HANDLER(rd_scan_en, void)
{
    // allocate the complete event message
    struct hci_rd_scan_en_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_scan_en_cmd_cmp_evt);
    evt->status = CO_ERROR_NO_ERROR;
    evt->scan_en = lm_env.hci.scan_en;
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write scan enable
HCI_CMD_HANDLER(wr_scan_en, struct hci_wr_scan_en_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;
    bool page_scan_stopping = false;

    do
    {

        // Check parameter validity
        if (param->scan_en <= BOTH_SCAN_ENABLE)
        {
            /*
             * Inquiry Scan
             */
            if (param->scan_en & INQUIRY_SCAN_ENABLE)
            {
                if ((lm_env.hci.scan_en & INQUIRY_SCAN_ENABLE) != INQUIRY_SCAN_ENABLE)
                {
                    struct ld_inquiry_scan_params iscan_par;

                    iscan_par.lap = lm_env.hci.iac_lap;
                    iscan_par.iscan_intv = lm_env.hci.inq_scan_intv;
                    iscan_par.iscan_win = lm_env.hci.inq_scan_win;
                    iscan_par.iscan_type = lm_env.hci.inq_scan_type;
                    iscan_par.page_scan_rep_mode = lm_env.hci.page_scan_rep_mode;

                    // Start Inquiry Scan
                    status = ld_iscan_start(&iscan_par);

                    if (CO_ERROR_NO_ERROR != status)
                    {
                        break;
                    }
                }
            }
            else
            {
                // Stop Inquiry Scan
                ld_iscan_stop();
            }

            /*
             * Page Scan
             */
            if (param->scan_en & PAGE_SCAN_ENABLE)
            {
                // Check the current Page Scan state
                if ((lm_env.hci.scan_en & PAGE_SCAN_ENABLE) != PAGE_SCAN_ENABLE)
                {
                    uint8_t link_id;

                    // Allocate link identifier
                    for (link_id = 0 ; link_id < MAX_NB_ACTIVE_ACL ; link_id++)
                    {
                        // Check state
                        if (lm_env.con_info[link_id].state == LM_FREE)
                        {
                            break;
                        }
                    }

                    // Check if link identifier found
                    if (link_id < MAX_NB_ACTIVE_ACL)
                    {
                        struct ld_page_scan_params pscan_par;

                        pscan_par.pscan_intv = lm_env.hci.page_scan_intv;
                        pscan_par.pscan_win = lm_env.hci.page_scan_win;
                        pscan_par.pscan_type = lm_env.hci.page_scan_type;
                        pscan_par.link_id = link_id;

                        // Start Page Scan
                        status = ld_pscan_start(&pscan_par);

                        if (CO_ERROR_NO_ERROR == status)
                        {
                            // Set current connection state
                            lm_env.con_info[link_id].state = LM_PAGE_SCAN;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
            }
            else
            {
                // Check the current Page Scan state
                if (lm_env.hci.scan_en & PAGE_SCAN_ENABLE)
                {
                    uint8_t link_id;

                    // Release the reserved link identifier
                    for (link_id = 0 ; link_id < MAX_NB_ACTIVE_ACL ; link_id++)
                    {
                        // Check state
                        if (lm_env.con_info[link_id].state == LM_PAGE_SCAN)
                            break;
                    }

                    // Check if link identifier found
                    if (link_id < MAX_NB_ACTIVE_ACL)
                    {
                        // Stop Page Scan
                        if (ld_pscan_stop() == CO_ERROR_NO_ERROR)
                        {
                            // Indicate the link identifier as stopping page scan
                            lm_env.con_info[link_id].state = LM_PAGE_SCAN_STOPPING;
                            page_scan_stopping = true;
                        }
                    }
                }
            }

            // Store the parameter
            lm_env.hci.scan_en = param->scan_en;
            status = CO_ERROR_NO_ERROR;
        }

    }
    while (0);

    if (!page_scan_stopping)
    {
        // Send the command complete event
        lm_cmd_cmp_send(opcode, status);
    }

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read page scan activity
HCI_CMD_HANDLER(rd_page_scan_act, void)
{
    // allocate the complete event message
    struct hci_rd_page_scan_act_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_page_scan_act_cmd_cmp_evt);
    evt->status = CO_ERROR_NO_ERROR;
    evt->page_scan_intv = lm_env.hci.page_scan_intv;
    evt->page_scan_win = lm_env.hci.page_scan_win;

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write page scan activity
HCI_CMD_HANDLER(wr_page_scan_act, struct hci_wr_page_scan_act_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    if ((param->page_scan_intv >= PAGE_SCAN_INTV_MIN) && (param->page_scan_intv <= PAGE_SCAN_INTV_MAX) && ((param->page_scan_intv & 0x1) == 0)
            && (param->page_scan_win >= PAGE_SCAN_WIN_MIN) && (param->page_scan_win <= PAGE_SCAN_WIN_MAX) && (param->page_scan_win <= param->page_scan_intv))
    {
        lm_env.hci.page_scan_intv = param->page_scan_intv;
        lm_env.hci.page_scan_win = param->page_scan_win;

        if ((lm_env.hci.page_scan_intv == lm_env.hci.page_scan_win) && (lm_env.hci.page_scan_intv <= 2048))
        {
            lm_env.hci.page_scan_rep_mode = R0;
        }
        else if (lm_env.hci.page_scan_intv <= 2048) // 1.28s
        {
            lm_env.hci.page_scan_rep_mode = R1;
        }
        else if (lm_env.hci.page_scan_intv <= 4096) // 2.56s
        {
            lm_env.hci.page_scan_rep_mode = R2;
        }

        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read inquiry scan activity
HCI_CMD_HANDLER(rd_inq_scan_act, void)
{
    // allocate the complete event message
    struct hci_rd_inq_scan_act_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_inq_scan_act_cmd_cmp_evt);
    evt->status = CO_ERROR_NO_ERROR;
    evt->inq_scan_intv = lm_env.hci.inq_scan_intv;
    evt->inq_scan_win = lm_env.hci.inq_scan_win;
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write inquiry scan activity
HCI_CMD_HANDLER(wr_inq_scan_act, struct hci_wr_inq_scan_act_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    if ((param->inq_scan_intv >= INQ_SCAN_INTV_MIN) && (param->inq_scan_intv <= INQ_SCAN_INTV_MAX) && ((param->inq_scan_intv & 0x1) == 0)
            && (param->inq_scan_win >= INQ_SCAN_WIN_MIN) && (param->inq_scan_win <= INQ_SCAN_WIN_MAX) && (param->inq_scan_win <= param->inq_scan_intv))
    {
        lm_env.hci.inq_scan_intv = param->inq_scan_intv;
        lm_env.hci.inq_scan_win = param->inq_scan_win;
        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read authentication enable
HCI_CMD_HANDLER(rd_auth_en, void)
{
    // allocate the complete event message
    struct hci_rd_auth_en_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_auth_en_cmd_cmp_evt);
    evt->auth_en = lm_env.hci.auth_en;
    evt->status = CO_ERROR_NO_ERROR;
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write authentication enable
HCI_CMD_HANDLER(wr_auth_en, struct hci_wr_auth_en_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    if ((param->auth_en == AUTH_DISABLED) || (param->auth_en <= AUTH_ENABLED))
    {
        lm_env.hci.auth_en = param->auth_en;
        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read class of device
HCI_CMD_HANDLER(rd_class_of_dev, void)
{
    // allocate the complete event message
    struct hci_rd_class_of_dev_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_class_of_dev_cmd_cmp_evt);
    evt->status = CO_ERROR_NO_ERROR;
    ld_class_of_dev_get(&evt->class_of_dev);

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write class of device
HCI_CMD_HANDLER(wr_class_of_dev, struct hci_wr_class_of_dev_cmd)
{
    ld_class_of_dev_set((struct devclass *) &param->class_of_dev);
    lm_cmd_cmp_send(opcode, CO_ERROR_NO_ERROR);

    return (KE_MSG_CONSUMED);
}

#if (MAX_NB_SYNC > 0)
/// Handle the command HCI read voice settings
HCI_CMD_HANDLER(rd_voice_stg, void)
{
    // Allocate the complete event message
    struct hci_rd_voice_stg_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_voice_stg_cmd_cmp_evt);

    evt->status = CO_ERROR_NO_ERROR;
    evt->voice_stg = hci_voice_settings_get();

    // Send the message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write voice settings
HCI_CMD_HANDLER(wr_voice_stg, struct hci_wr_voice_stg_cmd)
{
    // Send the command complete event
    lm_cmd_cmp_send(opcode, hci_voice_settings_set(param->voice_stg));

    return (KE_MSG_CONSUMED);
}
#endif // (MAX_NB_SYNC > 0)

/// Handle the command HCI read synchronous flow control enable
HCI_CMD_HANDLER(rd_sync_flow_ctrl_en, void)
{
    struct hci_rd_sync_flow_ctrl_en_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_sync_flow_ctrl_en_cmd_cmp_evt);

#if VOICE_OVER_HCI
    // This command is allowed only when no connection exists
    evt->status = CO_ERROR_NO_ERROR;
    evt->sync_flow_ctrl_en = lm_env.hci.sync_flow_ctrl_en;
#else
    evt->status = CO_ERROR_UNSUPPORTED;
#endif //VOICE_OVER_HCI

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write synchronous flow control enable
HCI_CMD_HANDLER(wr_sync_flow_ctrl_en, struct hci_wr_sync_flow_ctrl_en_cmd)
{
    // This command is allowed only when no connection exists
    uint8_t status = CO_ERROR_NO_ERROR;

#if VOICE_OVER_HCI
    lm_env.hci.sync_flow_ctrl_en = param->sync_flow_ctrl_en;
#else
    status = CO_ERROR_UNSUPPORTED;
#endif //VOICE_OVER_HCI

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI set controller to host flow control.
HCI_CMD_HANDLER(set_ctrl_to_host_flow_ctrl, struct hci_set_ctrl_to_host_flow_ctrl_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    bool acl_flow_cntl_en  = false;
#if ((MAX_NB_SYNC > 0) & VOICE_OVER_HCI)
    bool sync_flow_cntl_en = false;
#endif //((MAX_NB_SYNC > 0) & VOICE_OVER_HCI)
    // check the buffer size command is already received
    // by checking the data_packet sizes are set to !=0

    if (param->flow_cntl <= FLOW_CONTROL_ACL_SCO)
    {
        switch (param->flow_cntl)
        {
        case FLOW_CONTROL_ACL:
        {
            // enable the ACL host flow control
            acl_flow_cntl_en = true;
        }
        break;
#if ((MAX_NB_SYNC > 0) & VOICE_OVER_HCI)
        case FLOW_CONTROL_SCO:
        {
            // enable the SCO host flow control
            sync_flow_cntl_en = true;
        }
        break;
#endif // ((MAX_NB_SYNC > 0) & VOICE_OVER_HCI)
        case FLOW_CONTROL_ACL_SCO:
        {
            // enable the ACL & SCO host flow control
            acl_flow_cntl_en = true;
#if ((MAX_NB_SYNC > 0) & VOICE_OVER_HCI)
            sync_flow_cntl_en = true;
#endif // ((MAX_NB_SYNC > 0) & VOICE_OVER_HCI)
        }
        break;
        default: // FLOW_CONTROL_OFF
        {
        }
        break;
        }

        // Enable HCI Host flow control for ACL data
        status = hci_fc_acl_en(acl_flow_cntl_en);

#if ((MAX_NB_SYNC > 0) & VOICE_OVER_HCI)
        if (status == CO_ERROR_NO_ERROR)
        {
            // Enable HCI Host flow control for synchronous data
            hci_fc_sync_en(sync_flow_cntl_en);
        }
#endif //((MAX_NB_SYNC > 0) & VOICE_OVER_HCI)
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI host buffer size.
HCI_CMD_HANDLER(host_buf_size, struct hci_host_buf_size_cmd)
{
    uint8_t status = hci_fc_acl_buf_size_set(param->acl_pkt_len, param->nb_acl_pkts);

#if VOICE_OVER_HCI
    if (status == CO_ERROR_NO_ERROR)
    {
        status = hci_fc_sync_buf_size_set(param->sync_pkt_len, param->nb_sync_pkts);
    }
#endif //VOICE_OVER_HCI

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI host number of completed packets.
HCI_CMD_HANDLER(host_nb_cmp_pkts, struct hci_host_nb_cmp_pkts_cmd)
{
    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t idx;
    uint16_t acl_pkt_cnt = 0;
#if VOICE_OVER_HCI
    uint16_t sync_pkt_cnt = 0;
#endif //VOICE_OVER_HCI

    for (idx = 0; idx < param->nb_of_hdl; idx++)
    {
        // BT ACL link
        if ((param->con[idx].hdl <= BT_ACL_CONHDL_MAX) && (param->con[idx].hdl >= BT_ACL_CONHDL_MIN))
        {
            acl_pkt_cnt += param->con[idx].nb_comp_pkt;
        }

#if VOICE_OVER_HCI
        // Synchronous link
        else if (param->con[idx].hdl & BT_SYNC_CONHDL_MSK)
        {
            sync_pkt_cnt += param->con[idx].nb_comp_pkt;
        }
#endif //VOICE_OVER_HCI

#if (BLE_EMB_PRESENT)
#if (BLE_CENTRAL || BLE_PERIPHERAL)
        // BLE ACL link
        else if ((param->con[idx].hdl <= BLE_CONHDL_MAX)) // && (param->con_hdl[idx] >= BLE_CONHDL_MIN))
        {
            acl_pkt_cnt += param->con[idx].nb_comp_pkt;
        }
#endif //(BLE_CENTRAL || BLE_PERIPHERAL)
#endif //(BLE_EMB_PRESENT)

        // Not a valid connection handle
        else
        {
            ASSERT_ERR_FORCE(0);
            status = CO_ERROR_INVALID_HCI_PARAM;
        }

    }

    // A command complete event must be sent to host only if there is an error
    if (status != CO_ERROR_NO_ERROR)
    {
        // Send the command complete event
        lm_cmd_cmp_send(opcode, status);
    }
    else
    {
        // update the Flow Control module with counted packets
        hci_fc_host_nb_acl_pkts_complete(acl_pkt_cnt);

#if VOICE_OVER_HCI
        hci_fc_host_nb_sync_pkts_complete(sync_pkt_cnt);
#endif //VOICE_OVER_HCI
    }

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read number of supported IAC
HCI_CMD_HANDLER(rd_nb_supp_iac, void)
{
    // allocate the complete event message
    struct hci_rd_nb_supp_iac_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_nb_supp_iac_cmd_cmp_evt);

    evt->status = CO_ERROR_NO_ERROR;
    evt->nb_iac = NB_IAC_MIN;

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read current IAC LAP
HCI_CMD_HANDLER(rd_curr_iac_lap, void)
{
    // allocate the complete event message
    struct hci_rd_curr_iac_lap_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_curr_iac_lap_cmd_cmp_evt);

    evt->status = CO_ERROR_NO_ERROR;
    evt->nb_curr_iac = NB_IAC_MIN;
    evt->iac_lap = lm_env.hci.iac_lap;

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write current IAC LAP
HCI_CMD_HANDLER(wr_curr_iac_lap, struct hci_wr_curr_iac_lap_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    if ((param->nb_curr_iac >= NB_IAC_MIN) && (param->nb_curr_iac <= NB_IAC_MAX)
            && (param->iac_lap[0].A[2] == GIAC_LAP_2) && (param->iac_lap[0].A[1] == GIAC_LAP_1) && (param->iac_lap[0].A[0] <= DIAC_MAX_LAP_0))
    {
        // only minimum is supported
        lm_env.hci.iac_lap = param->iac_lap[0];

#if RW_BT_MWS_COEX
        // Build inquiry scan X-input table for  Generalized Interlaced Scan Support
        ld_iscan_mwscoex_xi_mask_build(&lm_env.hci.iac_lap);
#endif // RW_BT_MWS_COEX

        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI set AFH host channel classification
HCI_CMD_HANDLER(set_afh_host_ch_class, struct hci_set_afh_host_ch_class_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;
    uint8_t nbchgood = 0;

    DBG_SWDIAG(AFH, HOST, 1);

    // Count number of good channels
    nbchgood = co_nb_good_channels(&param->afh_ch);

    // Check if there are enough channels in the map
    if (nbchgood >= AFH_NB_CHANNEL_MIN)
    {
        // Save Host channel classification
        memcpy(&lm_env.afh.host_ch_class.map[0], &param->afh_ch.map[0], BT_CH_MAP_LEN);
        // Immediate update of AFH channel maps for expected event generation
        ke_msg_send_basic(LM_AFH_TO, TASK_LM, TASK_LM);
        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    DBG_SWDIAG(AFH, HOST, 0);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read inquiry scan type
HCI_CMD_HANDLER(rd_inq_scan_type, void)
{
    // allocate the complete event message
    struct hci_rd_inq_scan_type_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_inq_scan_type_cmd_cmp_evt);

    evt->inq_scan_type = lm_env.hci.inq_scan_type;
    evt->status = CO_ERROR_NO_ERROR;

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write inquiry scan type
HCI_CMD_HANDLER(wr_inq_scan_type, struct hci_wr_inq_scan_type_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    if ((param->inq_scan_type == STANDARD_SCAN) || (param->inq_scan_type == INTERLACED_SCAN))
    {
        lm_env.hci.inq_scan_type = param->inq_scan_type;
        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read inquiry mode
HCI_CMD_HANDLER(rd_inq_mode, void)
{
    // allocate the complete event message
    struct hci_rd_inq_mode_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_inq_mode_cmd_cmp_evt);

    evt->inq_mode = lm_env.hci.inq_mode;
    evt->status = CO_ERROR_NO_ERROR;

    // send message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write inquiry mode
HCI_CMD_HANDLER(wr_inq_mode, struct hci_wr_inq_mode_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    switch (param->inq_mode)
    {
    case STANDARD_INQUIRY:
    case RSSI_INQUIRY:
    case EXTENDED_INQUIRY:
        lm_env.hci.inq_mode = param->inq_mode;
        status = CO_ERROR_NO_ERROR;
        break;
    default:
        break;
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read page scan type
HCI_CMD_HANDLER(rd_page_scan_type, void)
{
    // allocate the complete event message
    struct hci_rd_page_scan_type_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_page_scan_type_cmd_cmp_evt);

    evt->page_scan_type = lm_env.hci.page_scan_type;
    evt->status = CO_ERROR_NO_ERROR;

    // send message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write page scan type
HCI_CMD_HANDLER(wr_page_scan_type, struct hci_wr_page_scan_type_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    if ((param->page_scan_type == STANDARD_SCAN) || (param->page_scan_type == INTERLACED_SCAN))
    {
        lm_env.hci.page_scan_type = param->page_scan_type;
        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read AFH channel assessment mode
HCI_CMD_HANDLER(rd_afh_ch_assess_mode, void)
{
    // allocate the complete event message
    struct hci_rd_afh_ch_assess_mode_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_afh_ch_assess_mode_cmd_cmp_evt);

    evt->status = CO_ERROR_NO_ERROR;
    evt->afh_ch_ass_mode = rwip_ch_ass_en_get();

    hci_send_2_host(evt);
    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write AFH channel assessment mode
HCI_CMD_HANDLER(wr_afh_ch_assess_mode, struct hci_wr_afh_ch_assess_mode_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    if (param->afh_ch_ass_mode <= AFH_CH_ASS_ENABLED)
    {
        /* If the AFH_Channel_Assessment_Mode parameter is enabled and the Controller does not support
         * a channel assessment scheme, other than via the HCI_Set_AFH_Host_Channel_Classification command,
         * then a Status parameter of Channel Assessment Not Supported should be returned */
        if ((param->afh_ch_ass_mode == AFH_CH_ASS_DISABLED) || rwip_ch_ass_en_get())
        {
            rwip_ch_ass_en_set(param->afh_ch_ass_mode);
            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            status = CO_ERROR_CHANNEL_CLASS_NOT_SUP;
        }
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read extended inquiry response
HCI_CMD_HANDLER(rd_ext_inq_rsp, void)
{
    // allocate the complete event message
    struct hci_rd_ext_inq_rsp_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_ext_inq_rsp_cmd_cmp_evt);

    evt->status = ld_iscan_eir_get(&evt->fec_req, &evt->eir);

    // send message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write extended inquiry response
HCI_CMD_HANDLER(wr_ext_inq_rsp, struct hci_wr_ext_inq_rsp_cmd)
{
    // Send the command complete event
    lm_cmd_cmp_send(opcode, ld_iscan_eir_set(param->fec_req, &param->eir));

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read simple pairing mode
HCI_CMD_HANDLER(rd_sp_mode, void)
{
    // allocate the complete event message
    struct hci_rd_sp_mode_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_sp_mode_cmd_cmp_evt);

    evt->status = CO_ERROR_NO_ERROR;
    evt->sp_mode = lm_env.hci.sp_mode;

    // send message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write simple pairing mode
HCI_CMD_HANDLER(wr_sp_mode, struct hci_wr_sp_mode_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    // A Host shall not set the Simple Pairing Mode to disabled, specification only allows "enabled" as acceptable parameter
    if (param->sp_mode == SP_MODE_EN)
    {
        lm_env.hci.sp_mode = param->sp_mode;
        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read OOB data
HCI_CMD_HANDLER(rd_loc_oob_data, void)
{
    lm_gen_local_oob_data(false);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read inquiry response transmit power level
HCI_CMD_HANDLER(rd_inq_rsp_tx_pwr_lvl, void)
{
    // allocate the complete event message
    struct hci_rd_inq_rsp_tx_pwr_lvl_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_inq_rsp_tx_pwr_lvl_cmd_cmp_evt);

    evt->tx_pwr = LD_TXPWR_DBM_GET(rwip_rf.txpwr_max, MOD_GFSK, rwip_rf.txpwr_max_mod);//INQ_RSP_TX_PWR
    evt->status = CO_ERROR_NO_ERROR;

    // send message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write inquiry transmit power level
HCI_CMD_HANDLER(wr_inq_tx_pwr_lvl, struct hci_wr_inq_tx_pwr_lvl_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    // Check parameter range
    if ((param->tx_pwr >= INQ_TX_PWR_DBM_MIN) && (param->tx_pwr <= INQ_TX_PWR_DBM_MAX))
    {
        // Store value
        lm_env.hci.inq_tx_pwr_lvl = param->tx_pwr;
        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read default erroneous data reporting
HCI_CMD_HANDLER(rd_dft_err_data_rep, void)
{
    // allocate the complete event message
    struct hci_rd_dft_err_data_rep_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_dft_err_data_rep_cmd_cmp_evt);

    evt->status = CO_ERROR_NO_ERROR;
    evt->err_data_rep = lm_env.hci.err_data_rep;

    // send message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write default erroneous data reporting
HCI_CMD_HANDLER(wr_dft_err_data_rep, struct hci_wr_dft_err_data_rep_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    if (param->err_data_rep <= ERR_DATA_REP_EN)
    {
        lm_env.hci.err_data_rep = param->err_data_rep;
        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read LE Host Supported
HCI_CMD_HANDLER(rd_le_host_supp, void)
{
    // allocate the complete event message
    struct hci_rd_le_host_supp_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_le_host_supp_cmd_cmp_evt);

    evt->status = CO_ERROR_NO_ERROR;
    evt->le_supported_host = lm_env.hci.le_supported_host;
    evt->simultaneous_le_host = lm_env.hci.simultaneous_le_host;

    // send message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write LE Host Supported
HCI_CMD_HANDLER(wr_le_host_supp, struct hci_wr_le_host_supp_cmd)
{
    // Store values
    lm_env.hci.le_supported_host = param->le_supported_host;
    lm_env.hci.simultaneous_le_host = param->simultaneous_le_host;

    // Send the command complete event
    lm_cmd_cmp_send(opcode, CO_ERROR_NO_ERROR);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read Secure Connections Host Support
HCI_CMD_HANDLER(rd_sec_con_host_supp, void)
{
    // allocate the complete event message
    struct hci_rd_sec_con_host_supp_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_sec_con_host_supp_cmd_cmp_evt);

    evt->status = CO_ERROR_NO_ERROR;
    evt->sec_con_host_supp = lm_env.hci.sec_con_host_supp;

    // send message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write Secure Connections Host Support
HCI_CMD_HANDLER(wr_sec_con_host_supp, struct hci_wr_sec_con_host_supp_cmd)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // Check if page scan is ongoing or connections are present
    if (((lm_env.hci.scan_en & PAGE_SCAN_ENABLE) == 0) && (lm_get_nb_acl(MASTER_FLAG | SLAVE_FLAG) == 0))
    {
        lm_env.hci.sec_con_host_supp = param->sec_con_host_supp;
        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read OOB extended data
HCI_CMD_HANDLER(rd_loc_oob_ext_data, void)
{
    lm_gen_local_oob_data(true);
    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read Extended Page Timeout
HCI_CMD_HANDLER(rd_ext_page_to, void)
{
    // allocate the complete event message
    struct hci_rd_ext_page_to_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_ext_page_to_cmd_cmp_evt);

    evt->status = CO_ERROR_NO_ERROR;
    evt->ext_page_to = lm_env.hci.ext_page_to;

    // send message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}


#if RW_BT_MWS_COEX

/// Handle the command HCI set MWS channel params
HCI_CMD_HANDLER(set_mws_channel_params, struct hci_set_mws_channel_params_cmd)
{
    // allocate the complete event message
    struct hci_basic_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_cmd_cmp_evt);

    uint8_t mws_channel_enable = param->mws_channel_enable;
    uint8_t mws_channel_type = param->mws_channel_type;

    uint8_t status = CO_ERROR_NO_ERROR;

    // paramater checking
    if (!((MWS_CHANNEL_DISABLED == mws_channel_enable) || (MWS_CHANNEL_ENABLED == mws_channel_enable)) ||
            !((MWS_TDD_CHANNEL_TYPE == mws_channel_type) || (MWS_FDD_CHANNEL_TYPE == mws_channel_type)))
    {
        status = CO_ERROR_INVALID_HCI_PARAM;
    }

    // config
    if (CO_ERROR_NO_ERROR == status)
    {
        //  Configure MWS channel paramaters
        status = ld_mws_channel_params_set(mws_channel_enable, param->mws_rx_center_frequency, param->mws_tx_center_frequency,
                                           param->mws_rx_channel_bandwidth, param->mws_tx_channel_bandwidth, mws_channel_type);

        // Build inquiry scan X-input table for  Generalized Interlaced Scan Support
        ld_iscan_mwscoex_xi_mask_build(&lm_env.hci.iac_lap);

        // Build page scan X-input table for  Generalized Interlaced Scan Support
        ld_pscan_mwscoex_xi_mask_build();
    }

    evt->status = status;
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI set Exteral Frame Config
HCI_CMD_HANDLER(set_external_frame_config, struct hci_set_external_frame_config_cmd)
{
    // allocate the complete event message
    struct hci_basic_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_cmd_cmp_evt);

    uint16_t ext_fr_duration_agg = 0;
    uint16_t ext_fr_scan_duration_agg = 0;

#if PCA_SUPPORT
    bool ext_fr_downlink_offset_found = false;
    bool ext_fr_uplink_offset_found = false;

    int16_t ext_fr_downlink_offset = 0;
    int16_t ext_fr_uplink_offset = 0;
    int16_t ext_fr_mid_guard_period_offset = 0;
#endif //PCA_SUPPORT

    int i, j;

    uint8_t status = CO_ERROR_NO_ERROR;

    /* Number of specified periods in an external frame. Valid range: 1 to 32 (unsigned integer) HCI:7.3.81 */
    if ((param->ext_fr_num_periods < MWS_EXT_NUM_PERIODS_MIN) || (param->ext_fr_num_periods > MWS_EXT_NUM_PERIODS_MAX))
    {
        status  = CO_ERROR_INVALID_HCI_PARAM;
    }

    for (i = 0; i < param->ext_fr_num_periods; i++)
    {
        uint8_t period_type = param->period[i].type;

        /* Period_Type: 0x00 Downlink 0x01 Uplink 0x02 Bi-Directional 0x03 Guard Period ; 0x04-0xFF Reserved HCI:7.3.81 */
        if (period_type >= MWS_PERIOD_TYPE_RESERVED)
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }
        else
        {
#if PCA_SUPPORT
            if (!ext_fr_downlink_offset_found && ((period_type == MWS_PERIOD_TYPE_DOWNLINK)
                                                  || (period_type == MWS_PERIOD_TYPE_BIDIRECTIONAL)))
            {

                ext_fr_downlink_offset = ext_fr_duration_agg;
                ext_fr_downlink_offset_found = true;
            }
            else if (!ext_fr_uplink_offset_found && (period_type == MWS_PERIOD_TYPE_UPLINK))
            {

                ext_fr_uplink_offset = ext_fr_duration_agg;
                ext_fr_uplink_offset_found = true;
            }
            else if (period_type == MWS_PERIOD_TYPE_GUARD_PERIOD)
            {
                ext_fr_mid_guard_period_offset = ext_fr_duration_agg + (param->period[i].duration) / 2;
            }
#endif // PCA_SUPPORT

            ext_fr_duration_agg += param->period[i].duration;

            // Device can scan in MWS Downlink and the Guard Period only
            if ((period_type == MWS_PERIOD_TYPE_DOWNLINK) || (period_type == MWS_PERIOD_TYPE_GUARD_PERIOD))
            {
                ext_fr_scan_duration_agg += param->period[i].duration;
            }
        }
    }


    /* The sum of all Period_Duration[i] parameters shall be equal to the Ext_Frame_Duration parameter.    HCI:7.3.81  */
    if (ext_fr_duration_agg != param->ext_fr_duration)
    {
        status  = CO_ERROR_INVALID_HCI_PARAM;
    }

    /* When external frame structure is a multiple of 1.25ms, e.g. 5ms or 10ms MSW frames, it can be aligned
      in a stable manner with the piconet clock. HCI:7.3.81  */
    else if (CO_MOD(param->ext_fr_duration, 1250) != 0)
    {
        status = CO_ERROR_UNSUPPORTED;
    }

    if (CO_ERROR_NO_ERROR == status)
    {

#if PCA_SUPPORT
        /* For absolute offsets from frame_sync */
        uint16_t downlink_offset, uplink_offset, frame_duration;

        uint8_t nb_acl = lm_get_nb_acl(SLAVE_FLAG | MASTER_FLAG);

        if (!ext_fr_uplink_offset_found)
        {
            /* In LTE, if the implementation does not have access to the exact LTE timings, it can assume that the
              downlink to uplink boundary is in the middle of the GP in a special LTE subframe.  HCI:7.4.81 */
            ext_fr_uplink_offset = ext_fr_mid_guard_period_offset;
        }

        frame_duration = param->ext_fr_duration;

        downlink_offset = CO_MOD(((ext_fr_downlink_offset + frame_duration) - param->ext_fr_sync_assert_offset), frame_duration);
        uplink_offset = CO_MOD(((ext_fr_uplink_offset + frame_duration) - param->ext_fr_sync_assert_offset), frame_duration);

        ld_pca_local_config(downlink_offset, uplink_offset, frame_duration, param->ext_fr_sync_assert_jitter, nb_acl);

        lm_env.local_ext_fr_config = true;

        /* As the external frame configuration is now known, enable clock adjust reporting/requests */
        ld_pca_reporting_enable(true);
#endif // PCA_SUPPORT

        // SAM Support
        {
            uint8_t sam_submap0[SAM_TYPE0_SUBMAP_LEN];
            uint8_t byte_idx;
            uint8_t bit_pos;
            uint8_t avail_msk;

            uint8_t num_slots = param->ext_fr_duration / SLOT_SIZE;
            uint8_t subslot_idx = 0;
            uint16_t period_idx = 0;

            memset(&sam_submap0[0], 0, SAM_TYPE0_SUBMAP_LEN);

            /*
             * Build the Slot availability Submask
             * Bluetooth transmissions are not available during MWS downlink durations
             * Bluetooth reception is not available during MWS uplink durations
             */
            for (i = 0, j = 0; i < num_slots; i++)
            {
                byte_idx = i >> 2; // 4 sam slots per byte
                bit_pos = (i & 0x3) << 1;  // 2 bits fields

                avail_msk = SAM_SLOT_TX_AVAILABLE | SAM_SLOT_RX_AVAILABLE;

                do
                {
                    if (avail_msk)
                    {
                        switch (param->period[j].type)
                        {
                        case MWS_PERIOD_TYPE_BIDIRECTIONAL:
                        {
                            avail_msk = 0; // not available
                        }
                        break;
                        case MWS_PERIOD_TYPE_DOWNLINK:
                        {
                            avail_msk &= ~SAM_SLOT_TX_AVAILABLE; // tx not available
                        }
                        break;
                        case MWS_PERIOD_TYPE_UPLINK:
                        {
                            avail_msk &= ~SAM_SLOT_RX_AVAILABLE; // rx not available
                        }
                        break;

                        default: /*No impact*/
                            break;
                        }
                    }

                    if ((param->period[j].duration - period_idx) >= (SLOT_SIZE - subslot_idx))
                    {
                        period_idx += (SLOT_SIZE - subslot_idx);
                        subslot_idx = 0; // move to next slot
                    }
                    else
                    {
                        subslot_idx += (param->period[j].duration - period_idx);
                        period_idx = 0;
                        j++; // move to next period
                    }

                }
                while (subslot_idx && (j < param->ext_fr_num_periods));

                sam_submap0[byte_idx] |= (avail_msk) << bit_pos;
            }

            // Save SAM configuration information
            lm_env.sam_info.active_index = SAM_DISABLED;
            memcpy(&lm_env.sam_info.type0submap[0], &sam_submap0[0], SAM_TYPE0_SUBMAP_LEN);
            lm_env.sam_info.frame_len = num_slots;

            // Search for active link(s)
            for (i = 0; i < MAX_NB_ACTIVE_ACL; i++)
            {
                if (lm_env.con_info[i].state == LM_CONNECTED)
                {
                    // Send an indication to LC to trigger SAM update on the link
                    struct lc_sam_submap_update_ind *msg = KE_MSG_ALLOC(LC_SAM_SUBMAP_UPDATE_IND, KE_BUILD_ID(TASK_LC, i), TASK_LM, lc_sam_submap_update_ind);
                    msg->update_mode = SAM_UPDATE_INVALIDATE_MAPS;
                    memcpy(&msg->submap[0], &sam_submap0[0], SAM_TYPE0_SUBMAP_LEN);
                    ke_msg_send(msg);
                }
            }

            // LD write the submap to EM
            ld_local_sam_submap_write(&sam_submap0[0], num_slots);
        }

        // Set MWS external frame info
        ld_ext_frame_info_set(param->ext_fr_duration, param->ext_fr_sync_assert_offset, ext_fr_scan_duration_agg);
    }

    // Enable MWS COEX as now configured.
    rwip_mwscoex_set(1);

    evt->status = status;
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI set MWS signaling
HCI_CMD_HANDLER(set_mws_signaling, struct hci_set_mws_signaling_cmd)
{
    // allocate the complete event message
    struct hci_set_mws_signaling_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_set_mws_signaling_cmd_cmp_evt);

    struct ld_mws_signaling_wr_offsets mws_wroff;
    struct ld_mws_signaling_rd_offsets mws_rdoff;

    uint8_t status = CO_ERROR_NO_ERROR;

    /* BT Spec vol 7, Part A, section 2.2 tolerances for offsets and jitter recommends should use values in specified ranges as commented below,
      These are not explicit min/max values, merely recommendations, so accept configuration regardless of these ranges. */

    mws_wroff.rx_assert_offset = param->mws_rx_assert_offset; /* Earliest: -2ms. Latest: -20us. */
    mws_wroff.rx_deassert_offset = param->mws_rx_deassert_offset; /* Earliest: -2ms. Latest: 0us. */
    mws_wroff.tx_assert_offset = param->mws_tx_assert_offset; /* Earliest: -2ms. Latest: 0us. */
    mws_wroff.tx_deassert_offset = param->mws_tx_deassert_offset; /* Earliest: -2ms. Latest: 0us. */
    mws_wroff.pattern_assert_offset = param->mws_pattern_assert_offset; /* Earliest: 0ms. Latest: +Ext_Frame_Duration. */
    mws_wroff.inactivity_duration_assert_offset = param->mws_inactivity_duration_assert_offset; /* Earliest: 0ms. Latest: +Ext_Frame_Duration. */
    mws_wroff.scan_frequency_assert_offset = param->mws_scan_frequency_assert_offset; /* Earliest: -2ms. Latest: -20us. */

    mws_wroff.rx_assert_jitter = param->mws_rx_assert_jitter; /* Max: 5us */
    mws_wroff.rx_deassert_jitter = param->mws_rx_deassert_jitter; /* Max: 5us */
    mws_wroff.tx_assert_jitter = param->mws_tx_assert_jitter; /* Max: 5us */
    mws_wroff.tx_deassert_jitter = param->mws_tx_deassert_jitter; /* Max: 5us */
    mws_wroff.pattern_assert_jitter = param->mws_pattern_assert_jitter; /* Max: 5us */
    mws_wroff.inactivity_duration_assert_jitter = param->mws_inactivity_duration_assert_jitter; /* Max: 5us */
    mws_wroff.scan_frequency_assert_jitter = param->mws_scan_frequency_assert_jitter; /* Max: 5us */

    mws_wroff.priority_assert_offset_request = param->mws_priority_assert_offset_request; /* Earliest: -1ms. Latest: -200us */

    // config
    if (CO_ERROR_NO_ERROR == status)
    {
        status = ld_mws_signaling_offsets_set(&mws_wroff, &mws_rdoff);
    }

    evt->bt_rx_prio_assert_offset = mws_rdoff.rx_prio_assert_offset;
    evt->bt_rx_prio_assert_jitter = mws_rdoff.rx_prio_assert_jitter;
    evt->bt_rx_prio_deassert_offset = mws_rdoff.rx_prio_deassert_offset;
    evt->bt_rx_prio_deassert_jitter = mws_rdoff.rx_prio_deassert_jitter;
    evt->bt_tx_on_assert_offset = mws_rdoff.tx_on_assert_offset;
    evt->bt_tx_on_assert_jitter = mws_rdoff.tx_on_assert_jitter;
    evt->bt_tx_on_deassert_offset = mws_rdoff.tx_on_deassert_offset;
    evt->bt_tx_on_deassert_jitter = mws_rdoff.tx_on_deassert_jitter;

    evt->_802_rx_prio_assert_offset = 0; /* User defined else always 0x0 */
    evt->_802_rx_prio_assert_jitter = 0; /* User defined else always 0x0 */
    evt->_802_rx_prio_deasssert_offset = 0; /* User defined else always 0x0 */
    evt->_802_rx_prio_deassert_jitter = 0; /* User defined else always 0x0 */
    evt->_802_tx_on_assert_offset = 0; /* User defined else always 0x0 */
    evt->_802_tx_on_assert_jitter = 0; /* User defined else always 0x0 */
    evt->_802_tx_on_deassert_offset = 0; /* User defined else always 0x0 */
    evt->_802_tx_on_deassert_jitter = 0; /* User defined else always 0x0 */

    evt->status = status;
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI set MWS transport layer
HCI_CMD_HANDLER(set_mws_transport_layer, struct hci_set_mws_transport_layer_cmd)
{
    // allocate the complete event message
    struct hci_basic_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_cmd_cmp_evt);

    uint8_t status = CO_ERROR_NO_ERROR;

    uint8_t transport_layer = param->transport_layer;

    // paramater checking
    if (transport_layer > MWS_TRANSPORT_TYPE_MAX)
    {
        status  = CO_ERROR_INVALID_HCI_PARAM;
    }

    // config
    if (CO_ERROR_NO_ERROR == status)
    {
        status = ld_mws_transport_layer_set(transport_layer, param->to_mws_baud_rate, param->from_mws_baud_rate);
    }

    evt->status = status;
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI set MWS scan frequency table
HCI_CMD_HANDLER(set_mws_scan_freq_table, struct hci_set_mws_scan_freq_table_cmd)
{
    // allocate the complete event message
    struct hci_basic_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_cmd_cmp_evt);

    uint8_t status;

    uint8_t num_freqs = param->num_scan_frequencies;
    const struct mws_scan_freq *p_scan_freqs = &(param->scan_freq[0]);

    // config
    status = ld_mws_scan_freq_table_set(num_freqs, p_scan_freqs);

    evt->status = status;
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI set MWS pattern configuration
HCI_CMD_HANDLER(set_mws_pattern_config, struct hci_set_mws_pattern_config_cmd)
{
    // allocate the complete event message
    struct hci_basic_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_cmd_cmp_evt);

    uint8_t status = CO_ERROR_NO_ERROR;

    int i, j;
    uint8_t mws_pattern_index = param->mws_pattern_index;
    uint8_t num_intervals = param->num_intervals;
    const struct mws_pattern_intv *p_intv = &param->intv[0];
    uint32_t sum_intervals = 0;
    uint16_t ext_duration = 0;

    // paramater checking
    if (mws_pattern_index > MWS_PATTERN_INDEX_MAX)
    {
        status  = CO_ERROR_INVALID_HCI_PARAM;
    }

    for (i = 0; i < num_intervals; i++)
    {
        if (p_intv[i].type > MWS_PATTERN_TYPE_MAX)
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        if (p_intv[i].type == MWS_PATTERN_EXT_FRAME)
        {
            /*
             * Any interval with type 4 (ext_frame) shall have a MWS PATTERN interval duration not greater
             * than the length of a frame, and the sum of the MWS_PATTERN duration parameters for the previous intervals
             * shall be a multiple of the length of the frame. HCI 7.3.85
             */
            if ((p_intv[i].duration > (lm_env.sam_info.frame_len * SLOT_SIZE)) || CO_MOD(sum_intervals, (lm_env.sam_info.frame_len * SLOT_SIZE)))
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }

            // Saves the longest use of ext frame configuration
            if (p_intv[i].duration > ext_duration)
            {
                ext_duration = p_intv[i].duration;
            }

        }

        sum_intervals += p_intv[i].duration;
    }

    /*
     * The sum of the MWS_PATTERN interval durations shall be an integral multople of the length of a frame as defined by
     * the most recent Set_External_Frame_Configuration command. HCI 7.3.85
     */
    if (CO_MOD(sum_intervals, (lm_env.sam_info.frame_len * SLOT_SIZE)))
    {
        status = CO_ERROR_INVALID_HCI_PARAM;
    }

    // config
    if (CO_ERROR_NO_ERROR == status)
    {
        status = ld_mws_pattern_config_set(mws_pattern_index, num_intervals, p_intv);

        // SAM Support
        if (CO_ERROR_NO_ERROR == status)
        {
            uint8_t sam_submaps[SAM_SUBMAPS_LEN];
            uint8_t byte_idx;
            uint8_t bit_pos;
            uint8_t avail_msk;

            uint8_t t_sam_sm;
            uint8_t n_sam_sm;

            struct lm_sam_pattern *sam_pattern;

            uint16_t n_tx_slots = 0;
            uint16_t n_rx_slots = 0;
            uint8_t n_ex_sm = 0;

            bool tx_allowed;
            bool rx_allowed;

            uint16_t subslot_idx = 0;
            uint16_t period_idx = 0;

            uint8_t slot_pairs = sum_intervals / (2 * SLOT_SIZE); // number of slot pairs in the pattern

            t_sam_sm = lm_env.sam_info.frame_len;
            n_sam_sm = sum_intervals / (t_sam_sm * SLOT_SIZE);

            // reduce t_sam_sm if interval durations lower than external frame length
            for (i = 0; i < num_intervals; i++)
            {
                // must not clip the t_sam_sm shorter than required for submap definitions
                if ((p_intv[i].duration < (t_sam_sm * SLOT_SIZE)) && (p_intv[i].duration >= ext_duration))
                {
                    n_sam_sm = slot_pairs / (((p_intv[i].duration + (2 * SLOT_SIZE) - 1)) / (2 * SLOT_SIZE));
                    t_sam_sm = sum_intervals / (n_sam_sm * SLOT_SIZE);
                    ASSERT_ERR((t_sam_sm & 0x01) == 0); // resulting t_sam_sm should be an even number of slots
                }
            }

            memset(&sam_submaps[0], 0, SAM_SUBMAPS_LEN);

            for (i = 0, j = 0; i < n_sam_sm; i++)
            {
                byte_idx = i >> 2; // 4 sam submaps per byte
                bit_pos = (i & 3) << 1; // 2 bit fields

                avail_msk = SAM_SLOTS_AVAILABLE;
                tx_allowed = true;
                rx_allowed = true;

                do
                {
                    if (avail_msk != SAM_SLOTS_UNAVAILABLE)
                    {
                        // Determine availablilty mask and SM counts.
                        switch (p_intv[j].type)
                        {
                        case MWS_PATTERN_EXT_FRAME:
                            tx_allowed = false; // submap defined
                            rx_allowed = false; // submap defined
                            avail_msk = SAM_SLOTS_SUBMAPPED;
                            n_ex_sm++;
                            break;
                        case MWS_PATTERN_TX_ALLOWED:
                            rx_allowed = false;
                            break;
                        case MWS_PATTERN_RX_ALLOWED:
                            avail_msk = SAM_SLOTS_UNAVAILABLE;
                            tx_allowed = false;
                            break;
                        case MWS_PATTERN_NO_TXRX:
                            avail_msk = SAM_SLOTS_UNAVAILABLE;
                            tx_allowed = false;
                            rx_allowed = false;
                            break;
                        default: /* No impact */
                            break;
                        }
                    }

                    if ((p_intv[j].duration - period_idx) == ((t_sam_sm * SLOT_SIZE) - subslot_idx))
                    {
                        subslot_idx = 0; // move to next submap
                        period_idx = 0;
                        j++; // move to next duration
                    }
                    else if ((p_intv[j].duration - period_idx) > ((t_sam_sm * SLOT_SIZE) - subslot_idx))
                    {
                        period_idx += (t_sam_sm * SLOT_SIZE) - subslot_idx;
                        subslot_idx = 0; // move to next submap
                    }
                    else
                    {
                        subslot_idx += p_intv[j].duration - period_idx;
                        period_idx = 0;
                        j++; // move to next duration
                    }
                }
                while (subslot_idx && (j < num_intervals));

                if (tx_allowed)
                    n_tx_slots += t_sam_sm;

                if (rx_allowed)
                    n_rx_slots += t_sam_sm;

                sam_submaps[byte_idx] |= (avail_msk) << bit_pos;
            }

            // Save SAM configuration information
            sam_pattern = &lm_env.sam_info.pattern[mws_pattern_index];

            memcpy(&sam_pattern->submaps.map[0], &sam_submaps[0], SAM_SUBMAPS_LEN);
            sam_pattern->t_sam_sm = t_sam_sm;
            sam_pattern->n_sam_sm = n_sam_sm;
            sam_pattern->n_tx_slots = n_tx_slots;
            sam_pattern->n_rx_slots = n_rx_slots;
            sam_pattern->n_ex_sm = n_ex_sm;

            // Search for active links(s)
            for (i = 0; i < MAX_NB_ACTIVE_ACL; i++)
            {
                if (lm_env.con_info[i].state == LM_CONNECTED)
                {
                    // Send an indication to LC to trigger SAM defined map update on the link
                    struct lc_sam_map_update_ind *msg = KE_MSG_ALLOC(LC_SAM_MAP_UPDATE_IND, KE_BUILD_ID(TASK_LC, i), TASK_LM, lc_sam_map_update_ind);
                    msg->pattern_index = mws_pattern_index;
                    msg->t_sam_sm = t_sam_sm;
                    msg->n_sam_sm = n_sam_sm;
                    memcpy(&msg->submaps.map[0], &sam_submaps[0], SAM_SUBMAPS_LEN);
                    ke_msg_send(msg);
                }
            }
        }
    }

    evt->status = status;
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI get MWS transport layer configuration
HCI_CMD_HANDLER(get_mws_transport_layer_config, void)
{
    // allocate the complete event message
    struct hci_get_mws_transport_layer_config_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_get_mws_transport_layer_config_cmd_cmp_evt);

    uint8_t status = CO_ERROR_NO_ERROR;

    // The current HW does not support WCI transports.
    evt->num_transports = 0;

    evt->status = status;
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

#endif // RW_BT_MWS_COEX

/// Handle the command HCI write Extended Page Timeout
HCI_CMD_HANDLER(wr_ext_page_to, struct hci_wr_ext_page_to_cmd)
{
    // Store value
    lm_env.hci.ext_page_to = param->ext_page_to;

    // Send the command complete event
    lm_cmd_cmp_send(opcode, CO_ERROR_NO_ERROR);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read Extended Inquiry Length
HCI_CMD_HANDLER(rd_ext_inq_len, void)
{
    // allocate the complete event message
    struct hci_rd_ext_inq_len_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_ext_inq_len_cmd_cmp_evt);

    evt->status = CO_ERROR_NO_ERROR;
    evt->ext_inq_len = lm_env.hci.ext_inq_len;

    // send message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write Extended Inquiry Length
HCI_CMD_HANDLER(wr_ext_inq_len, struct hci_wr_ext_inq_len_cmd)
{
    // Store value
    lm_env.hci.ext_inq_len = param->ext_inq_len;

    // Send the command complete event
    lm_cmd_cmp_send(opcode, CO_ERROR_NO_ERROR);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI Set Ecosystem Base Interval
HCI_CMD_HANDLER(set_eco_base_intv, struct hci_set_eco_base_intv_cmd)
{
    // Send the command complete event
    lm_cmd_cmp_send(opcode, CO_ERROR_NO_ERROR);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI Configure Data Path
HCI_CMD_HANDLER(config_data_path, struct hci_config_data_path_cmd)
{
    uint8_t status;

    // Configure the data path
#if (BLE_EMB_PRESENT && BLE_ISO_PRESENT)
    status = data_path_config(param->data_path_id, param->vendor_specific_cfg_len, &param->vendor_specific_cfg[0]);
#else // !(BLE_EMB_PRESENT && BLE_ISO_PRESENT)
    status = CO_ERROR_INVALID_HCI_PARAM;
#endif // !(BLE_EMB_PRESENT && BLE_ISO_PRESENT)

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

#if (BT_53)
/// Handle the command HCI Set Min Encryption Key Size command
HCI_CMD_HANDLER(set_min_enc_key_size, struct hci_set_min_enc_key_size_cmd)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    // parameter checking
    if ((param->min_enc_key_size < MIN_ENC_KEY_SIZE_MIN) || (param->min_enc_key_size > MIN_ENC_KEY_SIZE_MAX))
    {
        // outside valid range
        status = CO_ERROR_INVALID_HCI_PARAM;
    }
    else if (param->min_enc_key_size < ENC_KEY_SIZE_MIN)
    {
        // controller does not support the value
        status = CO_ERROR_UNSUPPORTED;
    }
    else
    {
        lm_env.hci.min_enc_key_size = param->min_enc_key_size;
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}
#if (RW_DEBUG)
/// Handle HCI_DBG_Set_Max_Enc_Key_Size debug command
HCI_CMD_HANDLER(dbg_set_max_enc_key_size, struct hci_dbg_set_max_enc_key_size_cmd)
{
    lm_env.hci.max_enc_key_size = param->max_enc_key_size;

    // Send the command complete event
    lm_cmd_cmp_send(opcode, CO_ERROR_NO_ERROR);

    return (KE_MSG_CONSUMED);
}

#endif // (RW_DEBUG)
#endif //(BT_53)

/*
 * HCI INFORMATIONAL PARAMETERS COMMANDS HANDLERS
 ****************************************************************************************
 */

/// Handles the command HCI read local version informations
HCI_CMD_HANDLER(rd_local_ver_info, void)
{
    // allocate the status event message
    struct hci_rd_local_ver_info_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_local_ver_info_cmd_cmp_evt);

    // gets the hci version
    evt->hci_ver = RWBT_SW_VERSION_MAJOR;
    // gets the hci revision
    evt->hci_rev = co_htobs(CO_SUBVERSION_BUILD(RWBT_SW_VERSION_MINOR, RWBT_SW_VERSION_BUILD));
    // gets the lmp version
    evt->lmp_ver = RWBT_SW_VERSION_MAJOR;
    // gets the lmp subversion
    evt->lmp_subver = co_htobs(CO_SUBVERSION_BUILD(RWBT_SW_VERSION_MINOR, RWBT_SW_VERSION_BUILD));
    // gets the manufacturer name
    evt->manuf_name = co_htobs(RW_COMP_ID);
    // sets the status
    evt->status = CO_ERROR_NO_ERROR;
    // send the message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI read local supported commands
HCI_CMD_HANDLER(rd_local_supp_cmds, void)
{
    // allocate the status event message
    struct hci_rd_local_supp_cmds_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_local_supp_cmds_cmd_cmp_evt);

    // Copy the first part from ROM stored table
    memcpy(&evt->local_cmds.cmds[0], lm_local_supp_cmds, sizeof(lm_local_supp_cmds));

    // Fill remaining bytes with 0
    memset(&evt->local_cmds.cmds[sizeof(lm_local_supp_cmds)], 0x00, SUPP_CMDS_LEN - sizeof(lm_local_supp_cmds));

    // sets the status
    evt->status = CO_ERROR_NO_ERROR;

    // send the message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read local supported features
HCI_CMD_HANDLER(rd_local_supp_feats, void)
{
    // allocate the complete event message
    struct hci_rd_local_supp_feats_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_local_supp_feats_cmd_cmp_evt);

    // get the local features
    memcpy(&evt->feats.feats[0], &lm_local_supp_feats[0][0], FEATS_LEN);

    // update the status
    evt->status = CO_ERROR_NO_ERROR;

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read local extended features
HCI_CMD_HANDLER(rd_local_ext_feats, struct hci_rd_local_ext_feats_cmd)
{
    // allocate the complete event message
    struct hci_rd_local_ext_feats_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_local_ext_feats_cmd_cmp_evt);
    uint8_t status = (param->page_nb < FEATURE_PAGE_MAX) ? CO_ERROR_NO_ERROR : CO_ERROR_UNSUPPORTED;

    // get the local features
    lm_read_features(param->page_nb, NULL, &evt->ext_feats);

    evt->page_nb = param->page_nb;
    evt->status = status;
    evt->page_nb_max = FEATURE_PAGE_MAX - 1;

    // send message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read buffer size
HCI_CMD_HANDLER(rd_buf_size, void)
{
    // structure type for the complete command event
    struct hci_rd_buf_size_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_buf_size_cmd_cmp_evt);

    evt->hc_tot_nb_data_pkts = EM_NUM_BT_ACLTXBUF;
    evt->hc_data_pk_len = ACL_DATA_BUF_SIZE;
#if VOICE_OVER_HCI
    evt->hc_tot_nb_sync_pkts = SYNC_TX_BUF_NB;
    evt->hc_sync_pk_len = HCI_SYNC_MAX_DATA_SIZE;
#else
    evt->hc_tot_nb_sync_pkts = 0;
    evt->hc_sync_pk_len = 0;
#endif //VOICE_OVER_HCI

    // update the status
    evt->status = CO_ERROR_NO_ERROR;

    // send the message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI read bd address
HCI_CMD_HANDLER(rd_bd_addr, void)
{
    struct hci_rd_bd_addr_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_bd_addr_cmd_cmp_evt);
    ld_bd_addr_get(&evt->local_addr);
    evt->status = CO_ERROR_NO_ERROR;
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI read local supported codecs
HCI_CMD_HANDLER(rd_local_supp_codecs, void)
{
    // allocate the status event message
    struct hci_rd_local_supp_codecs_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_local_supp_codecs_cmd_cmp_evt);

    // gets the local supported commands
//    LM_GetLocalSupportedCodecs(&evt->local_cmds);

    // sets the status
    evt->status = CO_ERROR_NO_ERROR;
    evt->nb_supp_codecs = 0;
    evt->nb_supp_vendor_specific_codecs = 0;

    // send the message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI read local supported codecs v2
HCI_CMD_HANDLER(rd_local_supp_codecs_v2, void)
{
    // allocate the status event message
    struct hci_rd_local_supp_codecs_v2_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_local_supp_codecs_v2_cmd_cmp_evt);

    // gets the local supported commands
//    LM_GetLocalSupportedCodecs(&evt->local_cmds);

    // sets the status
    evt->status = CO_ERROR_NO_ERROR;
    evt->num_supp_standard_codecs = 0;
    evt->num_supp_vendor_specific_codecs = 0;

    // send the message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI read local simple pairing options
HCI_CMD_HANDLER(rd_local_sp_opt, void)
{
    struct hci_rd_local_sp_opt_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_local_sp_opt_cmd_cmp_evt);
    evt->status = CO_ERROR_NO_ERROR;
    evt->sp_opt = 1;
    evt->max_enc_key_size = ENC_KEY_SIZE_MAX;
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI Read Local Supported Codec Capabilities
HCI_CMD_HANDLER(rd_local_supp_codec_cap, struct hci_rd_local_supp_codec_cap_cmd)
{
    // allocate the complete event message
    struct hci_rd_local_supp_codec_cap_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_local_supp_codec_cap_cmd_cmp_evt);

    evt->status = CO_ERROR_NO_ERROR;
    evt->num_codec_cap = 1;
    evt->codec_cap_len = 4;
    memset(&evt->codec_cap[0], 0, evt->codec_cap_len);

    // send the message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);

}

/// Handles the command HCI Read Local Supported Controller Delay
HCI_CMD_HANDLER(rd_local_supp_ctrl_delay, struct hci_rd_local_supp_ctrl_delay_cmd)
{
    // allocate the complete event message
    struct hci_rd_local_supp_ctrl_delay_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_local_supp_ctrl_delay_cmd_cmp_evt);

    evt->status = CO_ERROR_NO_ERROR;
    memset(&evt->min_ctrl_delay[0], 0, 3);
    memset(&evt->max_ctrl_delay[0], 0, 3);

    // send the message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}


/*
 * HCI STATUS PARAMETERS COMMANDS HANDLERS
 ****************************************************************************************
 */

/// Handle the command HCI read clock
HCI_CMD_HANDLER(rd_clk, struct hci_rd_clk_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;
    uint32_t clock = 0;

    switch (param->clk_type)
    {
    case LOCAL_CLOCK:
    {
        clock = ld_read_clock();
        status = CO_ERROR_NO_ERROR;
    }
    break;
    case PICONET_CLOCK:
    {
        // Check connection handle
        if ((param->conhdl >= BT_ACL_CONHDL_MIN) && (param->conhdl <= BT_ACL_CONHDL_MAX))
        {
            uint8_t link_id = param->conhdl - BT_ACL_CONHDL_MIN;
            if (lm_env.con_info[link_id].state >= LM_CONNECTED)
            {
                // Forward to LC
                ke_msg_forward(param, KE_BUILD_ID(TASK_LC, link_id), opcode);

                return (KE_MSG_NO_FREE);
            }
        }
    }
    break;
    default:
        break;
    }

    {
        // allocate the complete event message
        struct hci_rd_clk_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_clk_cmd_cmp_evt);
        evt->conhdl = param->conhdl;
        evt->clk_acc = CLOCK_ACCURACY_UNKNOWN;
        evt->clk = clock;
        evt->status = status;
        hci_send_2_host(evt);
    }

    return (KE_MSG_CONSUMED);
}


/*
 * HCI TESTING COMMANDS HANDLERS
 ****************************************************************************************
 */

/// Handle the command HCI read loop back mode
HCI_CMD_HANDLER(rd_loopback_mode, void)
{
    // allocate the complete event message
    struct hci_rd_loopback_mode_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_loopback_mode_cmd_cmp_evt);
    evt->lb_mode = lm_env.hci.loopback_mode;
    evt->status = CO_ERROR_NO_ERROR;

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write loop back mode
HCI_CMD_HANDLER(wr_loopback_mode, struct hci_wr_loopback_mode_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    // Check parameter
    switch (param->lb_mode)
    {
    case REMOTE_LOOPBACK:
    {
        // Check if there are existing connections
        if (lm_get_nb_acl(MASTER_FLAG | SLAVE_FLAG))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }
    }
    // No break

    case NO_LOOPBACK:
    {
        lm_env.hci.loopback_mode = param->lb_mode;
        status = CO_ERROR_NO_ERROR;
    }
    break;

    case LOCAL_LOOPBACK:
    {
        status = CO_ERROR_UNSUPPORTED;
    }
    break;

    default:
        break;
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI enable device under test mode
HCI_CMD_HANDLER(en_dut_mode, void)
{
    // Enable device under test mode
    lm_env.hci.dut_mode_en = true;

    // Send the command complete event
    lm_cmd_cmp_send(opcode, CO_ERROR_NO_ERROR);

    return (KE_MSG_CONSUMED);
}


/// Handle the command HCI write simple pairing debug mode
HCI_CMD_HANDLER(wr_sp_dbg_mode, struct hci_wr_sp_dbg_mode_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    // Check parameter
    if (param->sp_mode <= SP_MODE_EN)
    {
        lm_env.hci.sp_debug_mode = param->sp_mode;
        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}


/*
 * HCI VENDOR SPECIFIC COMMANDS HANDLERS
 ****************************************************************************************
 */

#if CRYPTO_UT
/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command to test Secure Connection Crypto Functions.
 * This command checks the functions against the Spec Test Vectors and returns PASS/FAIL.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
HCI_CMD_HANDLER(dbg_test_crypto_func, struct hci_dbg_test_crypto_func_cmd)
{
    // The data field determines the function to be verified and the input parameters

// structure type for the complete command event
    struct hci_basic_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_DBG_TEST_CRYPTO_FUNC_CMD_OPCODE, hci_basic_cmd_cmp_evt);

    switch (param->function)
    {
    case TST_F1_256 :
    {
        uint8_t output[16];

        F1_256(&param->buf.data[0], &param->buf.data[32], &param->buf.data[64], &param->buf.data[80], output);
        if (memcmp(output, &param->buf.data[81], sizeof(output)) == 0)
        {
            // Success
            event->status = CO_ERROR_NO_ERROR;
        }
        else
        {
            // Failure
            event->status = CO_ERROR_UNSPECIFIED_ERROR;
        }
    }
    break;

    case TST_G_256 :
    {
        uint32_t output;
        uint32_t user_confirm_value = 0x00;
        uint32_t expected_confirm_value;

        G_256(&param->buf.data[0], &param->buf.data[32], &param->buf.data[64], &param->buf.data[80], &output);

        user_confirm_value = output;
        expected_confirm_value = (param->buf.data[96] * 0x01000000) + (param->buf.data[97] * 0x010000) + (param->buf.data[98] * 0x0100) + param->buf.data[99];
        if (user_confirm_value == expected_confirm_value)
        {
            // Success
            event->status = CO_ERROR_NO_ERROR;
        }
        else
        {
            // Failure
            event->status = CO_ERROR_UNSPECIFIED_ERROR;
        }
    }
    break;

    case TST_F2_256 :
    {
        uint8_t output[16];

        F2_256(&param->buf.data[0], &param->buf.data[32], &param->buf.data[48], &param->buf.data[64],
               &param->buf.data[68], &param->buf.data[74], output);

        if (memcmp(output, &param->buf.data[80], sizeof(output)) == 0)
        {
            // Success
            event->status = CO_ERROR_NO_ERROR;
        }
        else
        {
            // Failure
            event->status = CO_ERROR_UNSPECIFIED_ERROR;
        }

    }
    break;

    case TST_F3_256 :
    {
        uint8_t output[16];

        F3_256(&param->buf.data[0], &param->buf.data[32], &param->buf.data[48], &param->buf.data[64],
               &param->buf.data[80], &param->buf.data[83], &param->buf.data[89], output);


        if (memcmp(output, &param->buf.data[95], sizeof(output)) == 0)
        {
            // Success
            event->status = CO_ERROR_NO_ERROR;
        }
        else
        {
            // Failure
            event->status = CO_ERROR_UNSPECIFIED_ERROR;
        }
    }
    break;

    case TST_H3 :
    {
        uint8_t output[16];

        H3(&param->buf.data[0], &param->buf.data[16], &param->buf.data[22], &param->buf.data[28], output);

        if (memcmp(output, &param->buf.data[36], sizeof(output)) == 0)
        {
            // Success
            event->status = CO_ERROR_NO_ERROR;
        }
        else
        {
            // Failure
            event->status = CO_ERROR_UNSPECIFIED_ERROR;
        }
    }
    break;

    case TST_H4 :
    {
        uint8_t output[16];

        H4(&param->buf.data[0], &param->buf.data[16], &param->buf.data[22], output);

        if (memcmp(output, &param->buf.data[28], sizeof(output)) == 0)
        {
            // Success
            event->status = CO_ERROR_NO_ERROR;
        }
        else
        {
            // Failure
            event->status = CO_ERROR_UNSPECIFIED_ERROR;
        }
    }
    break;

    case TST_H5 :
    {
        uint8_t output[16];

        H5((const uint8_t *)&param->buf.data[0], (const uint8_t *)&param->buf.data[16], (const uint8_t *)&param->buf.data[32], output);

        if (memcmp(output, &param->buf.data[48], sizeof(output)) == 0)
        {
            // Success
            event->status = CO_ERROR_NO_ERROR;
        }
        else
        {
            // Failure
            event->status = CO_ERROR_UNSPECIFIED_ERROR;
        }
    }
    break;

    default :
    {
        // Failure
        event->status = CO_ERROR_UNSPECIFIED_ERROR;
    }
    break;
    }

    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}
#endif //CRYPTO_UT


#if (EAVESDROPPING_SUPPORT)
/// Handle the command HCI VS set segmented inquiry scan
HCI_CMD_HANDLER(vs_set_seg_inq_scan, struct hci_vs_set_seg_inq_scan_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    do
    {
        // Check if the offset is greater than distance
        if (param->time_offset >= param->distance)
            break;

        // Check if the distance is greater than duration
        if ((param->distance * SLOT_SIZE) < param->duration)
            break;

        // Check if the distance/duration allows sufficient time for rescheduling
        if (((param->distance * SLOT_SIZE) - param->duration) < (4 * SLOT_SIZE))
            break;

        // Check the number of segments (maximum 16 supported)
        if ((param->segments == 0) || (param->segments > 16))
            break;

        if (param->enable)
        {
            // Enable segmented scan
            struct ld_seg_scan_params params;
            params.time_offset = param->time_offset << 1;
            params.periodicity = param->periodicity << 1;
            params.distance    = param->distance << 1;
            params.duration    = param->duration << 1;
            params.segments    = param->segments;
            params.hus_offset  = param->us_offset << 1;
            memcpy(&params.phase_offsets, &param->phase_offsets, 16);
            memcpy(&params.bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN);
            status = ld_iscan_seg_scan_en(&params);
        }
        else
        {
            // Disable segmented scan
            status = ld_iscan_seg_scan_dis();
        }

    }
    while (0);

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI VS set segmented page scan
HCI_CMD_HANDLER(vs_set_seg_page_scan, struct hci_vs_set_seg_page_scan_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    do
    {
        // Check if the offset is greater than distance
        if (param->time_offset >= param->distance)
            break;

        // Check if the distance is greater than duration
        if ((param->distance * SLOT_SIZE) < param->duration)
            break;

        // Check if the distance/duration allows sufficient time for rescheduling
        if (((param->distance * SLOT_SIZE) - param->duration) < (4 * SLOT_SIZE))
            break;

        // Check the number of segments (maximum 16 supported)
        if ((param->segments == 0) || (param->segments > 16))
            break;

        if (param->enable)
        {
            // Enable segmented scan
            struct ld_seg_scan_params params;
            params.time_offset = param->time_offset << 1;
            params.periodicity = param->periodicity << 1;
            params.distance    = param->distance << 1;
            params.duration    = param->duration << 1;
            params.segments    = param->segments;
            params.hus_offset  = param->us_offset << 1;
            memcpy(&params.phase_offsets, &param->phase_offsets, 16);
            memcpy(&params.bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN);
            status = ld_pscan_seg_scan_en(&params);
        }
        else
        {
            // Disable segmented scan
            status = ld_pscan_seg_scan_dis();
        }
    }
    while (0);

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI VS set host max slot
HCI_CMD_HANDLER(vs_set_host_max_slot, struct hci_vs_set_host_max_slot_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    // Check if the host_max_slot parameter in 1, 3 or 5
    if ((param->host_max_slot != 1) && (param->host_max_slot != 3) && (param->host_max_slot != 5))
    {
        status = CO_ERROR_INVALID_HCI_PARAM;
    }
    else
    {
        lm_env.hci.host_max_slot = param->host_max_slot;
        lc_max_slot_update();
        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI VS set sco link request delay
HCI_CMD_HANDLER(vs_set_sco_link_request_delay, struct hci_vs_set_sco_link_request_delay_cmd)
{
    // Store the parameters
    lm_env.hci.sco_link_request_delay = param->delay;

    // Send the command complete event
    lm_cmd_cmp_send(opcode, CO_ERROR_NO_ERROR);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI VS notify sniff events
HCI_CMD_HANDLER(vs_notify_sniff_events, struct hci_vs_notify_sniff_events_cmd)
{

    // Store the parameters
    lm_env.hci.notify_sniff_events = param->enable;

    // Send the command complete event
    lm_cmd_cmp_send(opcode, CO_ERROR_NO_ERROR);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI VS notify clock wrap
HCI_CMD_HANDLER(vs_notify_clock_wrap, struct hci_vs_notify_clock_wrap_cmd)
{
    // Store the parameters
    lm_env.hci.clock_wrap_notification_enabled = param->enable;
    lm_env.hci.clock_wrap_notification_bd_addr = param->bd_addr;

    // Send the command complete event
    lm_cmd_cmp_send(opcode, CO_ERROR_NO_ERROR);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI VS set custom paging
HCI_CMD_HANDLER(vs_set_cust_paging, struct hci_vs_set_cust_paging_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    // Check received parameters
    if (!param->enable || (param->min_page_timeout >= PAGE_TO_MIN && param->train_nudging_offset % 2 == 0))
    {
        // Store the parameters
        status = ld_page_set_cust_params(param->enable, param->min_page_timeout, param->train_nudging_offset);
    }

    // Send the command complete event
    lm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}
#endif //EAVESDROPPING_SUPPORT

#if (RW_DEBUG) && (RW_BT_MWS_COEX)
/// Handle the command HCI VS set custom paging
HCI_CMD_HANDLER(vs_ebq_init_sam_nego, struct hci_vs_ebq_init_sam_nego_cmd)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    // structure type for the complete command event
    struct hci_basic_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_VS_EBQ_INIT_SAM_NEGO_CMD_OPCODE, hci_basic_cmd_cmp_evt);

    if (param->index < SAM_INDEX_CONTINUE)
    {
        lm_sam_index_config(param->index, rwip_time_get());
    }
    else
    {
        status = CO_ERROR_INVALID_HCI_PARAM;
    }

    event->status = status;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}
#endif // (RW_DEBUG) && (RW_BT_MWS_COEX)

/*
 * HCI COMMAND HANDLING
 ****************************************************************************************
 */

#if (BT_HCI_TEST_MODE)
    extern int hci_vs_rx_test_cmd_handler(struct hci_vs_rx_test_cmd const *param, uint16_t opcode);
    extern int hci_vs_tx_test_cmd_handler(struct hci_vs_tx_test_cmd const *param, uint16_t opcode);
    extern int hci_vs_test_end_cmd_handler(void const *param, uint16_t opcode);
#endif //(BT_HCI_TEST_MODE)

/// The message handlers for HCI commands
HCI_CMD_HANDLER_TAB(lm)
{
    {HCI_INQ_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_inq_cmd_handler                       },
    {HCI_INQ_CANCEL_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_inq_cancel_cmd_handler                },
    {HCI_PER_INQ_MODE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_per_inq_mode_cmd_handler              },
    {HCI_EXIT_PER_INQ_MODE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_exit_per_inq_mode_cmd_handler         },
    {HCI_CREATE_CON_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_create_con_cmd_handler                },
    {HCI_CREATE_CON_CANCEL_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_create_con_cancel_cmd_handler         },
    {HCI_REM_NAME_REQ_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rem_name_req_cmd_handler              },
    {HCI_REM_NAME_REQ_CANCEL_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rem_name_req_cancel_cmd_handler       },
#if CSB_SUPPORT
    {HCI_TRUNC_PAGE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_trunc_page_cmd_handler                },
    {HCI_TRUNC_PAGE_CAN_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_trunc_page_can_cmd_handler            },
#endif //CSB_SUPPORT
    {HCI_RD_DFT_LINK_POL_STG_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_dft_link_pol_stg_cmd_handler       },
    {HCI_WR_DFT_LINK_POL_STG_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_dft_link_pol_stg_cmd_handler       },
    {HCI_SET_EVT_MASK_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_set_evt_mask_cmd_handler              },
    {HCI_SET_EVT_MASK_PAGE_2_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_set_evt_mask_page_2_cmd_handler       },
    {HCI_RESET_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_reset_cmd_handler                     },
    {HCI_SET_EVT_FILTER_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_set_evt_filter_cmd_handler            },
    {HCI_RD_PIN_TYPE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_pin_type_cmd_handler               },
    {HCI_WR_PIN_TYPE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_pin_type_cmd_handler               },
    {HCI_RD_STORED_LK_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_stored_lk_cmd_handler              },
    {HCI_WR_STORED_LK_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_stored_lk_cmd_handler              },
    {HCI_DEL_STORED_LK_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_del_stored_lk_cmd_handler             },
    {HCI_WR_LOCAL_NAME_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_local_name_cmd_handler             },
    {HCI_RD_LOCAL_NAME_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_local_name_cmd_handler             },
    {HCI_RD_CON_ACCEPT_TO_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_con_accept_to_cmd_handler          },
    {HCI_WR_CON_ACCEPT_TO_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_con_accept_to_cmd_handler          },
    {HCI_RD_PAGE_TO_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_page_to_cmd_handler                },
    {HCI_WR_PAGE_TO_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_page_to_cmd_handler                },
    {HCI_RD_SCAN_EN_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_scan_en_cmd_handler                },
    {HCI_WR_SCAN_EN_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_scan_en_cmd_handler                },
    {HCI_RD_PAGE_SCAN_ACT_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_page_scan_act_cmd_handler          },
    {HCI_WR_PAGE_SCAN_ACT_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_page_scan_act_cmd_handler          },
    {HCI_RD_INQ_SCAN_ACT_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_inq_scan_act_cmd_handler           },
    {HCI_WR_INQ_SCAN_ACT_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_inq_scan_act_cmd_handler           },
    {HCI_RD_AUTH_EN_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_auth_en_cmd_handler                },
    {HCI_WR_AUTH_EN_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_auth_en_cmd_handler                },
    {HCI_RD_CLASS_OF_DEV_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_class_of_dev_cmd_handler           },
    {HCI_WR_CLASS_OF_DEV_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_class_of_dev_cmd_handler           },
#if (MAX_NB_SYNC > 0)
    {HCI_RD_VOICE_STG_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_voice_stg_cmd_handler              },
    {HCI_WR_VOICE_STG_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_voice_stg_cmd_handler              },
#endif // (MAX_NB_SYNC > 0)
    {HCI_RD_SYNC_FLOW_CTRL_EN_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_sync_flow_ctrl_en_cmd_handler      },
    {HCI_WR_SYNC_FLOW_CTRL_EN_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_sync_flow_ctrl_en_cmd_handler      },
    {HCI_SET_CTRL_TO_HOST_FLOW_CTRL_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_set_ctrl_to_host_flow_ctrl_cmd_handler},
    {HCI_HOST_BUF_SIZE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_host_buf_size_cmd_handler             },
    {HCI_HOST_NB_CMP_PKTS_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_host_nb_cmp_pkts_cmd_handler          },
    {HCI_RD_NB_SUPP_IAC_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_nb_supp_iac_cmd_handler            },
    {HCI_RD_CURR_IAC_LAP_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_curr_iac_lap_cmd_handler           },
    {HCI_WR_CURR_IAC_LAP_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_curr_iac_lap_cmd_handler           },
    {HCI_SET_AFH_HOST_CH_CLASS_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_set_afh_host_ch_class_cmd_handler     },
    {HCI_RD_INQ_SCAN_TYPE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_inq_scan_type_cmd_handler          },
    {HCI_WR_INQ_SCAN_TYPE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_inq_scan_type_cmd_handler          },
    {HCI_RD_INQ_MODE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_inq_mode_cmd_handler               },
    {HCI_WR_INQ_MODE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_inq_mode_cmd_handler               },
    {HCI_RD_PAGE_SCAN_TYPE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_page_scan_type_cmd_handler         },
    {HCI_WR_PAGE_SCAN_TYPE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_page_scan_type_cmd_handler         },
    {HCI_RD_AFH_CH_ASSESS_MODE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_afh_ch_assess_mode_cmd_handler     },
    {HCI_WR_AFH_CH_ASSESS_MODE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_afh_ch_assess_mode_cmd_handler     },
    {HCI_RD_EXT_INQ_RSP_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_ext_inq_rsp_cmd_handler            },
    {HCI_WR_EXT_INQ_RSP_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_ext_inq_rsp_cmd_handler            },
    {HCI_RD_SP_MODE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_sp_mode_cmd_handler                },
    {HCI_WR_SP_MODE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_sp_mode_cmd_handler                },
    {HCI_RD_LOC_OOB_DATA_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_loc_oob_data_cmd_handler           },
    {HCI_RD_INQ_RSP_TX_PWR_LVL_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_inq_rsp_tx_pwr_lvl_cmd_handler     },
    {HCI_WR_INQ_TX_PWR_LVL_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_inq_tx_pwr_lvl_cmd_handler         },
    {HCI_RD_DFT_ERR_DATA_REP_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_dft_err_data_rep_cmd_handler       },
    {HCI_WR_DFT_ERR_DATA_REP_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_dft_err_data_rep_cmd_handler       },
    {HCI_RD_LE_HOST_SUPP_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_le_host_supp_cmd_handler           },
    {HCI_WR_LE_HOST_SUPP_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_le_host_supp_cmd_handler           },
#if RW_BT_MWS_COEX
    {HCI_SET_MWS_CHANNEL_PARAMS_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_set_mws_channel_params_cmd_handler    },
    {HCI_SET_EXTERNAL_FRAME_CONFIG_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_set_external_frame_config_cmd_handler },
    {HCI_SET_MWS_SIGNALING_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_set_mws_signaling_cmd_handler         },
    {HCI_SET_MWS_TRANSPORT_LAYER_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_set_mws_transport_layer_cmd_handler   },
    {HCI_SET_MWS_SCAN_FREQ_TABLE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_set_mws_scan_freq_table_cmd_handler   },
    {HCI_SET_MWS_PATTERN_CONFIG_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_set_mws_pattern_config_cmd_handler    },
    {HCI_GET_MWS_TRANSPORT_LAYER_CONFIG_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_get_mws_transport_layer_config_cmd_handler },
#endif // RW_BT_MWS_COEX
    {HCI_RD_SEC_CON_HOST_SUPP_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_sec_con_host_supp_cmd_handler      },
    {HCI_WR_SEC_CON_HOST_SUPP_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_sec_con_host_supp_cmd_handler      },
    {HCI_RD_LOC_OOB_EXT_DATA_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_loc_oob_ext_data_cmd_handler       },
    {HCI_RD_EXT_PAGE_TO_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_ext_page_to_cmd_handler            },
    {HCI_WR_EXT_PAGE_TO_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_ext_page_to_cmd_handler            },
    {HCI_RD_EXT_INQ_LEN_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_ext_inq_len_cmd_handler            },
    {HCI_WR_EXT_INQ_LEN_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_ext_inq_len_cmd_handler            },
    {HCI_RD_LOCAL_VER_INFO_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_local_ver_info_cmd_handler         },
    {HCI_RD_LOCAL_SUPP_CMDS_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_local_supp_cmds_cmd_handler        },
    {HCI_RD_LOCAL_SUPP_FEATS_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_local_supp_feats_cmd_handler       },
    {HCI_RD_LOCAL_EXT_FEATS_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_local_ext_feats_cmd_handler        },
    {HCI_RD_BUF_SIZE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_buf_size_cmd_handler              },
    {HCI_RD_BD_ADDR_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_bd_addr_cmd_handler                },
    {HCI_RD_LOCAL_SUPP_CODECS_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_local_supp_codecs_cmd_handler      },
    {HCI_RD_LOCAL_SUPP_CODECS_V2_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_local_supp_codecs_v2_cmd_handler   },
    {HCI_RD_LOCAL_SP_OPT_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_local_sp_opt_cmd_handler           },
    {HCI_RD_CLK_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_clk_cmd_handler                    },
    {HCI_RD_LOOPBACK_MODE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_loopback_mode_cmd_handler          },
    {HCI_WR_LOOPBACK_MODE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_loopback_mode_cmd_handler          },
    {HCI_EN_DUT_MODE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_en_dut_mode_cmd_handler               },
    {HCI_WR_SP_DBG_MODE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_wr_sp_dbg_mode_cmd_handler            },
#if CRYPTO_UT
    {HCI_DBG_TEST_CRYPTO_FUNC_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_dbg_test_crypto_func_cmd_handler      },
#endif //CRYPTO_UT
#if (BT_READ_PICONET_CLOCK)
    {HCI_VS_RD_PICONET_CLOCK_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_vs_rd_piconet_clock_cmd_handler        },
#endif //(BT_READ_PICONET_CLOCK)
#if (BT_HCI_TEST_MODE)
    {HCI_VS_RX_TEST_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_vs_rx_test_cmd_handler                },
    {HCI_VS_TX_TEST_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_vs_tx_test_cmd_handler                },
    {HCI_VS_TEST_END_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_vs_test_end_cmd_handler               },
#endif //(BT_HCI_TEST_MODE)
    {HCI_SET_ECO_BASE_INTV_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_set_eco_base_intv_cmd_handler          },
    {HCI_CONFIG_DATA_PATH_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_config_data_path_cmd_handler           },
    {HCI_RD_LOCAL_SUPP_CODEC_CAP_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_local_supp_codec_cap_cmd_handler    },
    {HCI_RD_LOCAL_SUPP_CTRL_DELAY_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_rd_local_supp_ctrl_delay_cmd_handler   },
#if (BT_53)
    {HCI_SET_MIN_ENC_KEY_SIZE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_set_min_enc_key_size_cmd_handler      },
#if (RW_DEBUG)
    {HCI_DBG_SET_MAX_ENC_KEY_SIZE_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_dbg_set_max_enc_key_size_cmd_handler  },
#endif // (RW_DEBUG)
#endif // (BT_53)
#if (EAVESDROPPING_SUPPORT)
    {HCI_VS_SET_SEG_INQ_SCAN_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_vs_set_seg_inq_scan_cmd_handler       },
    {HCI_VS_SET_SEG_PAGE_SCAN_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_vs_set_seg_page_scan_cmd_handler      },
    {HCI_VS_SET_HOST_MAX_SLOT_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_vs_set_host_max_slot_cmd_handler      },
    {HCI_VS_NOTIFY_SNIFF_EVENTS_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_vs_notify_sniff_events_cmd_handler    },
    {HCI_VS_SET_SCO_LINK_REQUEST_DELAY_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_vs_set_sco_link_request_delay_cmd_handler},
    {HCI_VS_NOTIFY_CLOCK_WRAP_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_vs_notify_clock_wrap_cmd_handler      },
    {HCI_VS_SET_CUST_PAGING_CMD_OPCODE, (lm_hci_cmd_hdl_func_t) hci_vs_set_cust_paging_cmd_handler        },
#endif //EAVESDROPPING_SUPPORT
#if (RW_DEBUG) && (RW_BT_MWS_COEX)
    {HCI_VS_EBQ_INIT_SAM_NEGO_CMD_OPCODE, (lm_hci_cmd_hdl_func_t)hci_vs_ebq_init_sam_nego_cmd_handler},
#endif // (RW_DEBUG) && (RW_BT_MWS_COEX)
};

/**
 ****************************************************************************************
 * @brief Handles any HCI command
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(hci_command_lm, void)
{
    int return_status = KE_MSG_CONSUMED;

    // Check if there is a handler corresponding to the original command opcode
    for (uint16_t i = 0; i < ARRAY_LEN(lm_hci_command_handler_tab); i++)
    {
        // Check if opcode matches
        if (lm_hci_command_handler_tab[i].opcode == src_id)
        {
            // Check if there is a handler function
            if (lm_hci_command_handler_tab[i].func != NULL)
            {
                // Call handler
                return_status = lm_hci_command_handler_tab[i].func(param, src_id);
            }
            break;
        }
    }

    return return_status;
}

/**
 ****************************************************************************************
 * @brief Handles Inquiry Result Indication
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lm_inq_res_ind, struct lm_inq_res_ind)
{
    // Check if inquiry procedure ongoing
    if (lm_env.inq_state != LM_INQ_OFF)
    {
        // If not in the inquiry result filtering list, send the event
        if (!lm_inq_reports_list_check(&param->bd_addr))
        {
            // Check result event type
            switch (lm_env.hci.inq_mode)
            {
            case STANDARD_INQUIRY:
            {
                struct hci_inq_res_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_INQ_RES_EVT_CODE, hci_inq_res_evt);
                evt->nb_rsp = 1;
                memcpy(&evt->bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN);
                memcpy(&evt->class_of_dev.A[0], &param->class_of_dev.A[0], DEV_CLASS_LEN);
                evt->page_scan_rep_mode = param->page_scan_rep_mode;
                evt->clk_off        = param->clk_off;
                evt->reserved1      = 0;
                evt->reserved2      = 0;
                hci_send_2_host(evt);
            }
            break;
            case RSSI_INQUIRY:
            {
                struct hci_inq_res_with_rssi_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_INQ_RES_WITH_RSSI_EVT_CODE, hci_inq_res_with_rssi_evt);
                evt->nb_rsp = 1;
                memcpy(&evt->bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN);
                memcpy(&evt->class_of_dev.A[0], &param->class_of_dev.A[0], DEV_CLASS_LEN);
                evt->rssi           = LD_RSSI_CONVERT(param->rssi);
                evt->page_scan_rep_mode = param->page_scan_rep_mode;
                evt->clk_off        = param->clk_off;
                evt->reserved1      = 0;
                hci_send_2_host(evt);
            }
            break;
            case EXTENDED_INQUIRY:
            {
                if (param->fhs_eir_bit)
                {
                    struct hci_ext_inq_res_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_EXT_INQ_RES_EVT_CODE, hci_ext_inq_res_evt);
                    evt->nb_rsp = 1;
                    memcpy(&evt->bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN);
                    memcpy(&evt->class_of_dev.A[0], &param->class_of_dev.A[0], DEV_CLASS_LEN);
                    evt->rssi           = LD_RSSI_CONVERT(param->rssi);
                    evt->page_scan_rep_mode = param->page_scan_rep_mode;
                    evt->clk_off        = param->clk_off;
                    evt->reserved1      = 0;
                    // Copy EIR data packet
                    memcpy(&evt->eir.data[0], &param->eir_data[0], co_min(param->eir_len, EIR_DATA_SIZE));
                    // Pad with 0
                    if (param->eir_len < EIR_DATA_SIZE)
                    {
                        memset(&evt->eir.data[param->eir_len], 0x00, (EIR_DATA_SIZE - param->eir_len));
                    }
                    hci_send_2_host(evt);
                }
                else
                {
                    struct hci_inq_res_with_rssi_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_INQ_RES_WITH_RSSI_EVT_CODE, hci_inq_res_with_rssi_evt);
                    evt->nb_rsp = 1;
                    memcpy(&evt->bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN);
                    memcpy(&evt->class_of_dev.A[0], &param->class_of_dev.A[0], DEV_CLASS_LEN);
                    evt->rssi           = LD_RSSI_CONVERT(param->rssi);
                    evt->page_scan_rep_mode = param->page_scan_rep_mode;
                    evt->clk_off        = param->clk_off;
                    evt->reserved1      = 0;
                    hci_send_2_host(evt);
                }
            }
            break;
            default:
            {
                ASSERT_INFO_FORCE(0, lm_env.hci.inq_mode, 0);
            }
            break;
            }
        }
    }
    else
    {
        ASSERT_WARN(0, 0, 0);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles Inquiry End Indication
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lm_inq_end_ind, void)
{
    // Check if inquiry procedure ongoing
    if (lm_env.inq_state != LM_INQ_OFF)
    {
        // Allocate the complete event message
        struct hci_inq_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_INQ_CMP_EVT_CODE, hci_inq_cmp_evt);
        evt->status = CO_ERROR_NO_ERROR;
        hci_send_2_host(evt);

        if (lm_env.inq_state == LM_INQ_NORMAL)
        {
            // For normal inquiry, this message indicates the end of the single process
            lm_env.inq_state = LM_INQ_OFF;
        }

        // Initialize the inquiry result filtering list
        memset(&lm_env.inq_filt, 0, sizeof(lm_env.inq_filt));
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles Page End Indication
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lm_page_end_ind, struct lm_page_end_ind)
{
    ASSERT_INFO(param->link_id < MAX_NB_ACTIVE_ACL, param->link_id, 0);

    // Check if Host requested to stop
    if (lm_env.con_info[param->link_id].state == LM_PAGE_STOPPING)
    {
        if (lm_env.create_con_cmd != NULL)
        {
            // Report connection cancel completion
            lm_cmd_cmp_bd_addr_send(HCI_CREATE_CON_CANCEL_CMD_OPCODE, CO_ERROR_NO_ERROR, &lm_env.con_info[param->link_id].bd_addr);

            // Report connection complete event
            {
                struct hci_con_cmp_evt *event = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_CON_CMP_EVT_CODE, hci_con_cmp_evt);
                event->status = CO_ERROR_UNKNOWN_CONNECTION_ID;
                event->link_type = ACL_TYPE;
                memcpy(&event->bd_addr.addr[0], &lm_env.create_con_cmd->bd_addr.addr[0], BD_ADDR_LEN);
                event->conhdl = 0xFFFF;
                event->enc_en = 0;
                hci_send_2_host(event);
            }

            // Free HCI command
            ke_msg_free(ke_param2msg(lm_env.create_con_cmd));
            lm_env.create_con_cmd = NULL;

            // Free LT address
            lm_lt_addr_free(lm_env.con_info[param->link_id].lt_addr);
        }
        else if (lm_env.rem_name_req_cmd != NULL)
        {
            // Report remote name request cancel completion
            lm_cmd_cmp_bd_addr_send(HCI_REM_NAME_REQ_CANCEL_CMD_OPCODE, CO_ERROR_NO_ERROR, &lm_env.con_info[param->link_id].bd_addr);

            {
                // Report remote name request failure
                struct hci_rem_name_req_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_REM_NAME_REQ_CMP_EVT_CODE, hci_rem_name_req_cmp_evt);
                evt->status = CO_ERROR_UNKNOWN_CONNECTION_ID;
                memcpy(&evt->bd_addr.addr[0], &lm_env.rem_name_req_cmd->bd_addr.addr[0], BD_ADDR_LEN);
                hci_send_2_host(evt);
            }

            // Free HCI command
            ke_msg_free(ke_param2msg(lm_env.rem_name_req_cmd));
            lm_env.rem_name_req_cmd = NULL;

            // Free LT address
            lm_lt_addr_free(lm_env.con_info[param->link_id].lt_addr);
        }
        else if (lm_env.trunc_page_cmd != NULL)
        {
            // Free HCI command
            ke_msg_free(ke_param2msg(lm_env.trunc_page_cmd));
            lm_env.trunc_page_cmd = NULL;

            // Report connection cancel completion
            lm_cmd_cmp_bd_addr_send(HCI_TRUNC_PAGE_CAN_CMD_OPCODE, CO_ERROR_NO_ERROR, &lm_env.con_info[param->link_id].bd_addr);
        }

        // Free link identifier
        lm_env.con_info[param->link_id].state = LM_FREE;
    }
    // Check if Page procedure ongoing
    else if (lm_env.con_info[param->link_id].state == LM_PAGE)
    {
        struct devclass class_of_dev = {{0, 0, 0}};

        // Check status
        switch (param->status)
        {
        case CO_ERROR_NO_ERROR:
        {
            // Update link identifier
            lm_env.con_info[param->link_id].state = LM_CONNECTED;
            lm_env.con_info[param->link_id].role = ROLE_MASTER;

            // Register linkID / BD address at HCI level
            hci_bt_acl_bdaddr_register(param->link_id, &lm_env.con_info[param->link_id].bd_addr);

            if (lm_env.create_con_cmd != NULL)
            {
                // Start LC for connection establishment
                struct lc_init_parameters lc_par;

                lc_par.reason = LC_MASTER_CON;
                lc_par.bd_addr = lm_env.create_con_cmd->bd_addr;
                lc_par.acl_pkt_type = lm_env.create_con_cmd->pkt_type;
                lc_par.role_switch_en = lm_env.create_con_cmd->switch_en;
                lc_par.class_of_dev = class_of_dev;
                lc_par.link_pol_stg = lm_env.hci.link_pol_stg;
                lc_par.slave_timing_info_ptr = NULL;
#if (BT_53)
                lc_par.min_enc_key_size = lm_env.hci.min_enc_key_size;
#if (RW_DEBUG)
                lc_par.max_enc_key_size = lm_env.hci.max_enc_key_size;
#endif  // (RW_DEBUG)
#endif // (BT_53)

                lc_start(param->link_id, &lc_par);

                // Activate AFH timer if needed
                lm_afh_activate_timer();

#if PCA_SUPPORT && RW_BT_MWS_COEX
                // Reconfigure target offset for MWS uplink/downlink based on new role
                ld_pca_update_target_offset(param->link_id);
#endif //PCA_SUPPORT && RW_BT_MWS_COEX
            }
            else if (lm_env.rem_name_req_cmd != NULL)
            {
                // Start LC for remote name
                struct lc_init_parameters lc_par;

                lc_par.reason = LC_REM_NAME;
                lc_par.bd_addr = lm_env.rem_name_req_cmd->bd_addr;
                lc_par.acl_pkt_type = PACKET_TYPE_DM1_FLAG;
                lc_par.role_switch_en = ROLE_SWITCH_NOT_ALLOWED;
                lc_par.class_of_dev = class_of_dev;
                lc_par.link_pol_stg = lm_env.hci.link_pol_stg;
                lc_par.slave_timing_info_ptr = NULL;
#if (BT_53)
                lc_par.min_enc_key_size = 0;
#if (RW_DEBUG)
                lc_par.max_enc_key_size = 0;
#endif  // (RW_DEBUG)
#endif // (BT_53)

                lc_start(param->link_id, &lc_par);
            }
#if CSB_SUPPORT
            else if (lm_env.trunc_page_cmd != NULL)
            {
                // Report truncated page completion
                struct hci_basic_stat_bd_addr_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_TRUNC_PAGE_CMP_EVT_CODE, hci_basic_stat_bd_addr_evt);
                evt->status = CO_ERROR_NO_ERROR;
                memcpy(&evt->bd_addr.addr[0], &lm_env.con_info[param->link_id].bd_addr.addr[0], BD_ADDR_LEN);
                hci_send_2_host(evt);

                // Release link identifier
                lm_env.con_info[param->link_id].state = LM_FREE;
                break;
            }
#endif //CSB_SUPPORT
            else
            {
                ASSERT_ERR_FORCE(0);
            }
        }
        break;
        case CO_ERROR_PAGE_TIMEOUT:
        {
            if (lm_env.create_con_cmd != NULL)
            {
                // Report connection failure
                struct hci_con_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_CON_CMP_EVT_CODE, hci_con_cmp_evt);
                evt->status = CO_ERROR_PAGE_TIMEOUT;
                memcpy(&evt->bd_addr.addr[0], &lm_env.con_info[param->link_id].bd_addr.addr[0], BD_ADDR_LEN);
                evt->conhdl = 0xFFFF;
                evt->link_type = ACL_TYPE;
                evt->enc_en = 0;
                hci_send_2_host(evt);
            }
            else if (lm_env.rem_name_req_cmd != NULL)
            {
                // Report remote name request failure
                struct hci_rem_name_req_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_REM_NAME_REQ_CMP_EVT_CODE, hci_rem_name_req_cmp_evt);
                evt->status = CO_ERROR_PAGE_TIMEOUT;
                memcpy(&evt->bd_addr.addr[0], &lm_env.con_info[param->link_id].bd_addr.addr[0], BD_ADDR_LEN);
                hci_send_2_host(evt);
            }
#if CSB_SUPPORT
            else if (lm_env.trunc_page_cmd != NULL)
            {
                // Report truncated page completion
                struct hci_basic_stat_bd_addr_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_TRUNC_PAGE_CMP_EVT_CODE, hci_basic_stat_bd_addr_evt);
                evt->status = CO_ERROR_PAGE_TIMEOUT;
                memcpy(&evt->bd_addr.addr[0], &lm_env.con_info[param->link_id].bd_addr.addr[0], BD_ADDR_LEN);
                hci_send_2_host(evt);

                // Release link identifier
                lm_env.con_info[param->link_id].state = LM_FREE;
                break;
            }
#endif //CSB_SUPPORT
            else
            {
                ASSERT_ERR_FORCE(0);
            }

            // Free link identifier and LT address
            lm_lt_addr_free(lm_env.con_info[param->link_id].lt_addr);
            lm_env.con_info[param->link_id].state = LM_FREE;
        }
        break;
        default:
        {
            ASSERT_INFO_FORCE(0, param->status, 0);
        }
        break;
        }

        // Free HCI command
        if (lm_env.create_con_cmd != NULL)
        {
            ke_msg_free(ke_param2msg(lm_env.create_con_cmd));
            lm_env.create_con_cmd = NULL;
        }
        else if (lm_env.rem_name_req_cmd != NULL)
        {
            ke_msg_free(ke_param2msg(lm_env.rem_name_req_cmd));
            lm_env.rem_name_req_cmd = NULL;
        }
#if CSB_SUPPORT
        else if (lm_env.trunc_page_cmd != NULL)
        {
            ke_msg_free(ke_param2msg(lm_env.trunc_page_cmd));
            lm_env.trunc_page_cmd = NULL;
        }
#endif //CSB_SUPPORT
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles Page Scan End Indication
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lm_page_scan_end_ind, struct lm_page_scan_end_ind)
{
    ASSERT_INFO(param->link_id < MAX_NB_ACTIVE_ACL, param->link_id, 0);

    // Check if Host requested to stop
    if (lm_env.con_info[param->link_id].state == LM_PAGE_SCAN_STOPPING)
    {
        // Report the completion of the command
        lm_cmd_cmp_send(HCI_WR_SCAN_EN_CMD_OPCODE, CO_ERROR_NO_ERROR);

        // Release link identifier
        lm_env.con_info[param->link_id].state = LM_FREE;
    }
    // Check if Page Scan procedure ongoing
    else if (lm_env.con_info[param->link_id].state == LM_PAGE_SCAN)
    {
        uint8_t link_id;
        struct lc_init_parameters lc_par;

        // Check if a connection to this BD Address already exists
        for (link_id = 0 ; link_id < MAX_NB_ACTIVE_ACL ; link_id++)
        {
            if ((lm_env.con_info[link_id].state >= LM_CONNECTED)
                    && !memcmp(&lm_env.con_info[link_id].bd_addr.addr[0], &param->peer_bd_addr.addr[0], BD_ADDR_LEN))
            {
                uint16_t dest_id = KE_BUILD_ID(TASK_LC, link_id);
                lc_stop(dest_id, CO_ERROR_CON_ALREADY_EXISTS);
            }
        }

        // Update link identifier
        lm_env.con_info[param->link_id].state = LM_CONNECTED;
        lm_env.con_info[param->link_id].role = ROLE_SLAVE;
        memcpy(&lm_env.con_info[param->link_id].bd_addr.addr[0], &param->peer_bd_addr.addr[0], BD_ADDR_LEN);

        // Register linkID / BD address at HCI level
        hci_bt_acl_bdaddr_register(param->link_id, &lm_env.con_info[param->link_id].bd_addr);

        // Start LC
        lc_par.reason = LC_SLAVE_CON;
        lc_par.bd_addr = param->peer_bd_addr;
        lc_par.acl_pkt_type = 0;
        lc_par.role_switch_en = 0;
        lc_par.class_of_dev = param->class_of_dev;
        lc_par.link_pol_stg = lm_env.hci.link_pol_stg;
        lc_par.slave_timing_info_ptr = (void *) &param->slave_timing_info;
#if (BT_53)
        lc_par.min_enc_key_size = lm_env.hci.min_enc_key_size;
#if (RW_DEBUG)
        lc_par.max_enc_key_size = lm_env.hci.max_enc_key_size;
#endif  // (RW_DEBUG)
#endif // (BT_53)

        lc_start(param->link_id, &lc_par);

#if PCA_SUPPORT && RW_BT_MWS_COEX
        // Reconfigure target offset for MWS uplink/downlink based on new role
        ld_pca_update_target_offset(param->link_id);
#endif //PCA_SUPPORT && RW_BT_MWS_COEX

        // Restart page scan if required
        if (lm_env.hci.scan_en & PAGE_SCAN_ENABLE)
        {
            // Allocate link identifier
            for (link_id = 0 ; link_id < MAX_NB_ACTIVE_ACL ; link_id++)
            {
                // Check state
                if (lm_env.con_info[link_id].state == LM_FREE)
                {
                    break;
                }
            }

            // Check if link identifier found
            if (link_id < MAX_NB_ACTIVE_ACL)
            {
                struct ld_page_scan_params pscan_par;

                pscan_par.pscan_intv = lm_env.hci.page_scan_intv;
                pscan_par.pscan_win = lm_env.hci.page_scan_win;
                pscan_par.pscan_type = lm_env.hci.page_scan_type;
                pscan_par.link_id = link_id;

                if (ld_pscan_start(&pscan_par) == CO_ERROR_NO_ERROR)
                {
                    // Set current connection state
                    lm_env.con_info[link_id].state = LM_PAGE_SCAN;
                }
            }
        }
    }

    return (KE_MSG_CONSUMED);
}

#if CSB_SUPPORT
/**
 ****************************************************************************************
 * @brief Handles Page Response Timeout Indication
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lm_page_resp_to_ind, void)
{
    // Report to Host
    void *evt = ke_msg_alloc(HCI_EVENT, 0, HCI_SLV_PAGE_RSP_TO_EVT_CODE, 0);
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}
#endif //CSB_SUPPORT

#if RW_BT_MWS_COEX
/**
****************************************************************************************
* @brief Handles a SAM configuration request on new connection
*
* @param[in] msgid Id of the message received (probably unused).
* @param[in] param Pointer to the parameters of the message.
* @param[in] dest_id ID of the receiving task instance (probably unused).
* @param[in] src_id ID of the sending task instance.
* @return If the message was consumed or not.
****************************************************************************************
*/
KE_MSG_HANDLER(lm_sam_config_req, struct lm_sam_config_req)
{
    bool maps_updating = false;

    if (lm_env.sam_info.frame_len)
    {
        // If a submap is defined, and the requesting link does not have it
        if (!param->submap_av)
        {
            // Send an indication to LC to trigger SAM update on the link
            struct lc_sam_submap_update_ind *msg = KE_MSG_ALLOC(LC_SAM_SUBMAP_UPDATE_IND, KE_BUILD_ID(TASK_LC, param->link_id), TASK_LM, lc_sam_submap_update_ind);
            msg->update_mode = SAM_UPDATE_INVALIDATE_MAPS;
            memcpy(&msg->submap[0], &lm_env.sam_info.type0submap[0], SAM_TYPE0_SUBMAP_LEN);

            ke_msg_send(msg);
            maps_updating = true;
        }
        // Already has submap, check if patterns are defined
        else
        {
            struct lm_sam_pattern *sam_pattern;

            for (int idx = 0; idx < SAM_INDEX_MAX; idx++)
            {
                sam_pattern = &lm_env.sam_info.pattern[idx];

                // If a pattern is defined, and the requesting link does not have it, though already has submap
                if (sam_pattern->n_sam_sm && !param->pattern_av[idx])
                {
                    // Send an indication to LC to trigger SAM defined map update on the link
                    struct lc_sam_map_update_ind *msg = KE_MSG_ALLOC(LC_SAM_MAP_UPDATE_IND, KE_BUILD_ID(TASK_LC, param->link_id), TASK_LM, lc_sam_map_update_ind);
                    msg->pattern_index = idx;
                    msg->t_sam_sm = sam_pattern->t_sam_sm;
                    msg->n_sam_sm = sam_pattern->n_sam_sm;
                    memcpy(&msg->submaps.map[0], &sam_pattern->submaps.map[0], SAM_SUBMAPS_LEN);

                    ke_msg_send(msg);
                    maps_updating = true;
                }
            }
        }

        // Maps are configured, check if SAM requires activation on link
        if (!maps_updating)
        {
            if (lm_env.sam_info.active_index != SAM_DISABLED)
            {
                // Send an indication to LC to trigger SAM switch on the link
                struct lc_mws_pattern_ind *msg = KE_MSG_ALLOC(LC_MWS_PATTERN_IND, KE_BUILD_ID(TASK_LC, param->link_id), TASK_LM, lc_mws_pattern_ind);
                struct lm_sam_pattern *sam_pattern = &lm_env.sam_info.pattern[lm_env.sam_info.active_index];

                msg->pattern_index = lm_env.sam_info.active_index;
                msg->t_sam_sm = sam_pattern->t_sam_sm;
                msg->t_sam = sam_pattern->t_sam_sm * sam_pattern->n_sam_sm;
                msg->n_tx_slots = sam_pattern->n_tx_slots;
                msg->n_rx_slots = sam_pattern->n_rx_slots;
                msg->n_ex_sm = sam_pattern->n_ex_sm;
#if RW_BT_MWS_COEX
                msg->time = ld_pca_ext_frame_ts_get();
#else //!RW_BT_MWS_COEX
                msg->time = rwip_time_get();
#endif //RW_BT_MWS_COEX

                ke_msg_send(msg);
            }

            // Confirm map configurations to link are complete
            ke_msg_send_basic(LC_OP_SAM_CFM, KE_BUILD_ID(TASK_LC, param->link_id), TASK_LM);
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
****************************************************************************************
* @brief Handles an MWS PATTERN indication
*
* @param[in] msgid Id of the message received (probably unused).
* @param[in] param Pointer to the parameters of the message.
* @param[in] dest_id ID of the receiving task instance (probably unused).
* @param[in] src_id ID of the sending task instance.
* @return If the message was consumed or not.
****************************************************************************************
*/
KE_MSG_HANDLER(lm_mws_pattern_ind, struct lm_mws_pattern_ind)
{
    // Check if update on pattern_index is required
    if (param->pattern_index != lm_env.sam_info.active_index)
    {
        lm_sam_index_config(param->pattern_index, param->time);
    }

    return (KE_MSG_CONSUMED);
}
#endif // RW_BT_MWS_COEX

/**
 ****************************************************************************************
 * @brief Handles the AFH time out message.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lm_afh_to, void)
{
    bool afh = false;

    DBG_SWDIAG(AFH, TO, 1);

    // Search for master active link(s)
    for (int i = 0; i < MAX_NB_ACTIVE_ACL; i++)
    {
        // Check if link is active and master
        if ((lm_env.con_info[i].state == LM_CONNECTED) && (lm_env.con_info[i].role == ROLE_MASTER))
        {
            afh = true;
        }
    }

#if CSB_SUPPORT
    // Report channel map change event for CSB
    if (lb_util_get_csb_mode())
    {
        afh = true;
    }
#endif //CSB_SUPPORT

    // If AFH is needed
    if (afh)
    {
        struct bt_ch_map ch_map;

        // Compute new channel map
        lm_ch_map_compute(&ch_map);

        // Search for master active link(s)
        for (int i = 0; i < MAX_NB_ACTIVE_ACL; i++)
        {
            // Check if link is active and master
            if ((lm_env.con_info[i].state == LM_CONNECTED) && (lm_env.con_info[i].role == ROLE_MASTER))
            {
                lc_ch_map_update(i, &ch_map);
            }
        }

        // If the channel map does not contain enough enabled channel, activate channels in order to have
        // a minimum of AFH_NB_CHANNEL_MIN channels activated.
        lm_fill_ch_map(&ch_map);

        // Update CSB and/or active broadcast channel map
        lb_ch_map_update(&ch_map);

        // Restart AFH timer
        ke_timer_set(LM_AFH_TO, TASK_LM, BT_AFH_UPDATE_PERIOD * 1000);
    }
    else
    {
        // Indicate AFH as inactive
        lm_env.afh.active = false;
    }

    DBG_SWDIAG(AFH, TO, 0);

    return (KE_MSG_CONSUMED);
}

#if (EAVESDROPPING_SUPPORT)
/**
 ****************************************************************************************
 * @brief Handles the clock wrap
 ****************************************************************************************
 */
KE_MSG_HANDLER(lm_clock_wrap_ind, struct lm_clock_wrap_ind)
{
    // Check if clock wrap notification is enabled
    if (lm_env.hci.clock_wrap_notification_enabled)
    {
        bool send_hci_event = false;

        if (co_bdaddr_compare(&param->addr, &lm_env.hci.clock_wrap_notification_bd_addr))
        {
            // Addresses match: send event
            send_hci_event = true;
        }
        else
        {
            if (co_bdaddr_compare(&param->addr, &co_null_bdaddr))
            {
                // Local CLKN wrapped: send event if either master or not connected
                uint8_t link_id = lm_find_link_id(lm_env.hci.clock_wrap_notification_bd_addr);
                if (link_id < MAX_NB_ACTIVE_ACL)
                {
                    if (ld_acl_role_get(link_id) == ROLE_MASTER)
                    {
                        send_hci_event = true;
                    }
                }
                else
                {
                    send_hci_event = true;
                }
            }
        }

        if (send_hci_event)
        {
            // Send the custom HCI event
            struct hci_vs_clock_wrap_evt *event =
                KE_MSG_ALLOC(HCI_DBG_EVT, 0, 0, hci_vs_clock_wrap_evt);

            event->subcode = HCI_VS_CLOCK_WRAP_EVT_SUBCODE;
            memcpy(&event->bd_addr.addr[0], &lm_env.hci.clock_wrap_notification_bd_addr, BD_ADDR_LEN);
            hci_send_2_host(event);
        }
    }
    return (KE_MSG_CONSUMED);
}
#endif // EAVESDROPPING_SUPPORT

/*
 * TASK DESCRIPTOR DEFINITIONS
 ****************************************************************************************
 */

#if (BT_HCI_TEST_MODE)
    extern KE_MSG_HANDLER_NO_STATIC(lm_test_end_ind, struct lm_test_end_ind);
#endif //(BT_HCI_TEST_MODE)

/// Specifies the default message handlers
KE_MSG_HANDLER_TAB(lm)
{
    // Note: all messages must be sorted in ID ascending order
    {LM_INQ_RES_IND, (ke_msg_func_t) lm_inq_res_ind_handler       },
    {LM_INQ_END_IND, (ke_msg_func_t) lm_inq_end_ind_handler       },
    {LM_PAGE_END_IND, (ke_msg_func_t) lm_page_end_ind_handler      },
    {LM_PAGE_SCAN_END_IND, (ke_msg_func_t) lm_page_scan_end_ind_handler },
#if (CSB_SUPPORT)
    {LM_PAGE_RESP_TO_IND, (ke_msg_func_t) lm_page_resp_to_ind_handler  },
#endif // (CSB_SUPPORT)
#if (RW_BT_MWS_COEX)
    {LM_SAM_CONFIG_REQ, (ke_msg_func_t)lm_sam_config_req_handler},
    {LM_MWS_PATTERN_IND, (ke_msg_func_t)lm_mws_pattern_ind_handler },
#endif // (RW_BT_MWS_COEX)
#if (BT_HCI_TEST_MODE)
    {LM_TEST_END_IND, (ke_msg_func_t) lm_test_end_ind_handler },
#endif //(BT_HCI_TEST_MODE)

    { LM_AFH_TO, (ke_msg_func_t)lm_afh_to_handler },
#if (EAVESDROPPING_SUPPORT)
    {LM_CLOCK_WRAP_IND, (ke_msg_func_t) lm_clock_wrap_ind_handler    },
#endif // EAVESDROPPING_SUPPORT

    {HCI_COMMAND, (ke_msg_func_t) hci_command_lm_handler       },
};

/// Specifies the message handlers that are common to all states.
//KE_MSG_STATE(lm)

/// Defines the place holder for the states of all the task instances.
ke_state_t lm_state[LM_IDX_MAX];

/// LM task descriptor
const struct ke_task_desc TASK_DESC_LM = {lm_msg_handler_tab, lm_state, LM_IDX_MAX, ARRAY_LEN(lm_msg_handler_tab)};

/// @} LMTASK
