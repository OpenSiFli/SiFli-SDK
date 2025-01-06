/**
 ****************************************************************************************
 *
 * @file co_hci.h
 *
 * @brief This file contains the HCI Bluetooth defines, enumerations and structures
 *        definitions for use by all modules in RW stack.
 *
 * Copyright (C) RivieraWaves 2009-2019
 *
 ****************************************************************************************
 */

#ifndef CO_HCI_H_
#define CO_HCI_H_

/**
 ****************************************************************************************
 * @addtogroup COMMON Common SW Block
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdbool.h>       // standard boolean definitions
#include <stddef.h>        // standard definitions
#include <stdint.h>        // standard integer definitions
#include "rwip_config.h"   // IP configuration
#include "co_bt_defines.h" // Common definitions
#include "compiler.h"      // compiler definitions

/*
 * DEFINES
 ****************************************************************************************
 */

// Maximum number of connection info in number of completed packets
#if (EMB_PRESENT)
    #if(BT_EMB_PRESENT)
        #define MAX_NB_CMP_PKTS_CON  (MAX_NB_ACTIVE_ACL)
    #else // (BLE_EMB_PRESENT)
        #define MAX_NB_CMP_PKTS_CON  (BLE_ACTIVITY_MAX+1) // ensure that at least 1 element is present
    #endif // (BT_EMB_PRESENT)

#else // (HOST_PRESENT)
    #define MAX_NB_CMP_PKTS_CON  (HOST_ACTIVITY_MAX)
#endif // (EMB_PRESENT)

/******************************************************************************************/
/* -------------------------   H4TL DEFINITIONS Part IV.A    -----------------------------*/
/******************************************************************************************/

///HCI Transport Header length - change if different transport
#define HCI_TRANSPORT_HDR_LEN                       0x01

enum hci_msg_type
{
    ///UART header: command message type
    HCI_CMD_MSG_TYPE                           = 0x01,
    ///UART header: ACL data message type
    HCI_ACL_MSG_TYPE                           = 0x02,
    ///UART header: Synchronous data message type
    HCI_SYNC_MSG_TYPE                          = 0x03,
    ///UART header: event message type
    HCI_EVT_MSG_TYPE                           = 0x04,
    ///UART header: ISO data message type
    HCI_ISO_MSG_TYPE                           = 0x05,

    TRC_MSG_TYPE                               = 0x06,

    AHI_KE_MSG_TYPE                            = 0x07,

    ISOOHCI_ISO_MSG_TYPE                       = 0x0A,

    BSP_TEST                                   = 0x10

};

/******************************************************************************************/
/* -------------------------   HCI DEFINITIONS Part 4.E     ------------------------------*/
/******************************************************************************************/

///HCI Command Opcode byte length
#define HCI_CMD_OPCODE_LEN         (0x02)

///HCI Event code byte length
#define HCI_EVT_CODE_LEN           (0x01)

///HCI Command/Event parameter length field byte length
#define HCI_CMDEVT_PARLEN_LEN      (0x01)

///HCI Command header length
#define HCI_CMD_HDR_LEN            (HCI_CMD_OPCODE_LEN + HCI_CMDEVT_PARLEN_LEN)

///HCI Event header length
#define HCI_EVT_HDR_LEN            (HCI_EVT_CODE_LEN + HCI_CMDEVT_PARLEN_LEN)

/// HCI ACL header: handle and flags decoding
enum  hci_acl_hdr_fields
{
    /// bits[00:11]: Connection handle
    HCI_ACL_HDR_HDL_LSB        = (0),
    HCI_ACL_HDR_HDL_MASK       = (0x0FFF),
    /// bits[12:13]: Packet boundary flag
    HCI_ACL_HDR_PB_FLAG_LSB    = (12),
    HCI_ACL_HDR_PB_FLAG_MASK   = (0x3000),
    /// bits[14:15]: Broadcast flag
    HCI_ACL_HDR_BC_FLAG_LSB    = (14),
    HCI_ACL_HDR_BC_FLAG_MASK   = (0xC000),
    /// Packet boundary and Broadcast flags
    HCI_ACL_HDR_DATA_FLAGS_LSB  = (12),
    HCI_ACL_HDR_DATA_FLAGS_MASK = (0xF000),
};

#define HCI_ACL_HDR_HDL_FLAGS_POS  (0)
#define HCI_ACL_HDR_HDL_FLAGS_LEN  (2)
/// HCI ACL header: data length field length
#define HCI_ACL_HDR_DATA_LEN_POS   (HCI_ACL_HDR_HDL_FLAGS_LEN)
#define HCI_ACL_HDR_DATA_LEN_LEN   (2)

///HCI ACL data packet header length
#define HCI_ACL_HDR_LEN            (HCI_ACL_HDR_HDL_FLAGS_LEN + HCI_ACL_HDR_DATA_LEN_LEN)

/// HCI Synchronous header: handle and flags decoding
enum hci_syn_hdr_fields
{
    /// bits[00:11]: Connection handle
    HCI_SYNC_HDR_HDL_LSB   = (0),
    HCI_SYNC_HDR_HDL_MASK  = (0x0FFF),
    /// bits[12:13]: Packet status flag
    HCI_SYNC_HDR_PSF_LSB   = (12),
    HCI_SYNC_HDR_PSF_MASK  = (0x3000),
    /// bits[14:15]: RFU
    HCI_SYNC_HDR_RFU_LSB   = (14),
    HCI_SYNC_HDR_RFU_MASK  = (0xC000),
};

#define HCI_SYNC_HDR_HDL_FLAGS_POS  (0)
#define HCI_SYNC_HDR_HDL_FLAGS_LEN  (2)


/// HCI Synchronous header: data length field length
#define HCI_SYNC_HDR_DATA_LEN_POS   (HCI_SYNC_HDR_HDL_FLAGS_LEN)
#define HCI_SYNC_HDR_DATA_LEN_LEN   (1)
#define HCI_SYNC_MAX_DATA_SIZE      (255)

///HCI sync data packet header length
#define HCI_SYNC_HDR_LEN           (HCI_SYNC_HDR_HDL_FLAGS_LEN + HCI_SYNC_HDR_DATA_LEN_LEN)

///HCI Command Complete Event minimum parameter length: 1(nb_pk)+2(opcode)
#define HCI_CCEVT_HDR_PARLEN       (0x03)

///HCI Command Complete Event header length:1(code)+1(len)+1(pk)+2(opcode)
#define HCI_CCEVT_HDR_LEN          (HCI_EVT_HDR_LEN + HCI_CCEVT_HDR_PARLEN)

///HCI Basic Command Complete Event packet length
#define HCI_CCEVT_BASIC_LEN        (HCI_CCEVT_HDR_LEN + 1)

///HCI Command Status Event parameter length - constant
#define HCI_CSEVT_PARLEN           (0x04)

///HCI Command Status Event length:1(code)+1(len)+1(st)+1(pk)+2(opcode)
#define HCI_CSEVT_LEN              (HCI_EVT_HDR_LEN + HCI_CSEVT_PARLEN)

///HCI Reset Command parameter length
#define HCI_RESET_CMD_PARLEN       0

/// Default return parameter length for HCI Command Complete Event
#define HCI_CCEVT_BASIC_RETPAR_LEN 1

/// Max HCI commands param size
#define HCI_MAX_CMD_PARAM_SIZE    255

/// Macro to extract OCF from OPCODE
#define HCI_OP2OCF(opcode)        ((opcode) & 0x03FF)

/// Macro to extract OGF from OPCODE
#define HCI_OP2OGF(opcode)        ((opcode) >> 10 & 0x003F)

/// Macro to create OPCODE from OGF and OCF
#define HCI_GET_OPCODE(ocf, ogf)      (((ogf) << 10) | ocf)

/// Maximum length of HCI advertising data fragments
#define HCI_ADV_DATA_FRAG_MAX_LEN        251

/// Maximum length of HCI periodic advertising data fragments
#define HCI_PER_ADV_DATA_FRAG_MAX_LEN    252

/// HCI ISO header: handle and flags decoding
enum  hci_iso_hdr_fields
{
    /// Connection handle (12 bits)
    HCI_ISO_HDR_HDL_LSB        = (0),
    HCI_ISO_HDR_HDL_MASK       = (0x0FFF),
    /// Packet boundary flag (2 bits)
    HCI_ISO_HDR_PB_FLAG_LSB    = (12),
    HCI_ISO_HDR_PB_FLAG_MASK   = (0x3000),
    /// Time_Stamp flag (1 bit)
    HCI_ISO_HDR_TS_FLAG_POS    = (14),
    HCI_ISO_HDR_TS_FLAG_BIT    = (0x4000),
    /// RFU (1 bit)
    HCI_ISO_HDR_RFU_FLAG_POS    = (0),
    HCI_ISO_HDR_RFU_FLAG_BIT    = (0x8000),
    /// ISO_Data_Load_Length (14 bits)
    HCI_ISO_HDR_ISO_DATA_LOAD_LEN_LSB  = (0),
    HCI_ISO_HDR_ISO_DATA_LOAD_LEN_MASK = (0x3FFF),
    /// RFU2 (2 bits)
    HCI_ISO_HDR_ISO_RFU2_LSB    = (14),
    HCI_ISO_HDR_ISO_RFU2_MASK   = (0xC000),
};

#define HCI_ISO_HDR_HDL_FLAGS_POS  (0)
#define HCI_ISO_HDR_HDL_FLAGS_LEN  (2)
/// HCI ISO header: ISO_Data_Load field length
#define HCI_ISO_HDR_ISO_DATA_LOAD_LEN_POS   (HCI_ISO_HDR_HDL_FLAGS_LEN)
#define HCI_ISO_HDR_ISO_DATA_LOAD_LEN_LEN   (2)

///HCI ACL data packet header length
#define HCI_ISO_HDR_LEN            (HCI_ISO_HDR_HDL_FLAGS_LEN + HCI_ISO_HDR_ISO_DATA_LOAD_LEN_LEN)

/// Packet Boundary Flag   HCI:5.4.5
#define PB_FLAG_1ST_FRAG          0x00
#define PB_FLAG_CONT_FRAG         0x01
#define PB_FLAG_CMP_FRAG          0x02
#define PB_FLAG_LAST_FRAG         0x03

/// HCI ISO header: handle and flags decoding
/// Time_Stamp (32 bits)
#define HCI_ISO_DATA_LOAD_TIME_STAMP_POS      (0)
#define HCI_ISO_DATA_LOAD_TIME_STAMP_LSB      (0)
#define HCI_ISO_DATA_LOAD_TIME_STAMP_MASK     (0xFFFFFFFF)
/// Packet_Sequence_Number (16 bits)
#define HCI_ISO_DATA_LOAD_PKT_SEQ_NB_POS      (4)
#define HCI_ISO_DATA_LOAD_PKT_SEQ_NB_LSB      (0)
#define HCI_ISO_DATA_LOAD_PKT_SEQ_NB_MASK     (0xFFFF)
/// ISO_SDU_Length (12 bits)
#define HCI_ISO_DATA_LOAD_ISO_SDU_LEN_POS     (6)
#define HCI_ISO_DATA_LOAD_ISO_SDU_LEN_LSB     (0)
#define HCI_ISO_DATA_LOAD_ISO_SDU_LEN_MASK    (0x0FFF)
/// RFU (2 bits)
#define HCI_ISO_DATA_LOAD_RFU_POS             (6)
#define HCI_ISO_DATA_LOAD_RFU_LSB             (12)
#define HCI_ISO_DATA_LOAD_RFU_MASK            (0x3000)
/// Packet_Status_Flag (2 bits)
#define HCI_ISO_DATA_LOAD_PKT_STAT_FLAG_LSB   (14)
#define HCI_ISO_DATA_LOAD_PKT_STAT_FLAG_MASK  (0xC000)

/// HCI ISO_Data_Load - Length of Time_Stamp field
#define HCI_ISO_DATA_LOAD_TIME_STAMP_LEN    (4)

/// HCI ISO_Data_Load - Length of Packet Sequence Number field
#define HCI_ISO_DATA_LOAD_PKT_SEQ_NB_LEN    (2)

/// HCI ISO_Data_Load - Length of ISO SDU Length and packet status flags field
#define HCI_ISO_DATA_LOAD_ISO_SDU_LEN_LEN   (2)

/// HCI ISO_Data_Load - maximum header length
#define HCI_ISO_DATA_LOAD_HDR_LEN_MAX    (HCI_ISO_DATA_LOAD_TIME_STAMP_LEN + HCI_ISO_DATA_LOAD_PKT_SEQ_NB_LEN + HCI_ISO_DATA_LOAD_ISO_SDU_LEN_LEN)

/// HCI ISO_Data_Load - Packet Status Flag
enum  hci_iso_pkt_stat_flag
{
    /// Valid data. The complete ISO_SDU was received correctly
    HCI_ISO_PKT_STAT_FLAG_VALID   = (0),
    /// Possibly invalid data. The contents of the ISO_SDU may contain errors or part of the ISO_SDU may
    /// be missing. This is reported as "data with possible errors".
    HCI_ISO_PKT_STAT_FLAG_INVALID = (1),
    /// Part(s) of the ISO_SDU were not received correctly. This is reported as "lost data".
    HCI_ISO_PKT_STAT_FLAG_LOST    = (2),
};

#if (EAVESDROPPING_SUPPORT)
    /// HCI ECI Rx stats array maximum length
    #define HCI_ECI_RX_STATS_LEN_MAX         (81)
#endif // EAVESDROPPING_SUPPORT

/// HCI Connection event number of events tracing maximum
#define HCI_CON_MAX_TRACE_NB_EVENT       (10)

/// Channel scan channel selection
enum ch_scan_ch_sel
{
    /// All channels
    CH_SCAN_ALL_CH                    = 0,
    /// Inactive channels
    CH_SCAN_INACTIVE_CH               = 1,
};

///HCI Command header components structure
struct hci_cmd_hdr
{
    /// Opcode field
    uint16_t opcode;
    ///Parameter length - the number of bytes of the command parameters
    uint8_t parlen;
};

///HCI ACL data packets header structure
struct hci_acl_hdr
{
    ///Connection handle & Data Flags
    uint16_t hdl_flags;
    ///Data length in number of bytes
    uint16_t datalen;
};

///HCI synchronous data packets header structure
struct hci_sync_hdr
{
    /// Connection handle & Data Flags
    uint16_t conhdl_flags;
    /// Data total length in number of bytes
    uint8_t data_total_len;
};

///HCI Event header components structure - contains all details possible in an event
struct hci_evt_hdr
{
    ///Event code
    uint8_t  code;
    ///Event parameters length
    uint8_t  parlen;
};

///HCI ISO data packets header structure
struct hci_iso_hdr
{
    /// Connection handle & Data Flags
    uint16_t conhdl_flags;
    /// ISO Data load length in number of bytes
    uint16_t iso_data_load_len;
};


/**************************************************************************************
 **************                       HCI COMMANDS                     ****************
 **************************************************************************************/

///HCI enumeration of possible Command OGF values.
enum
{
    ///HCI Link Control Commands Group OGF code
    LK_CNTL_OGF = 0x01,
    ///HCI Link Policy Commands Group OGF code
    LK_POL_OGF,
    ///HCI Controller and Baseband Commands Group OGF code
    CNTLR_BB_OGF,
    ///HCI Information Parameters Commands Group OGF code
    INFO_PAR_OGF,
    ///HCI Status Commands Group OGF code
    STAT_PAR_OGF,
    ///HCI Test Commands Group OGF code
    TEST_OGF,
    ///HCI Low Energy Commands Group OGF code
    LE_CNTLR_OGF = 0x08,
    ///HCI Vendor Specific Group OGF code
    VS_OGF = 0x3F,
    MAX_OGF
};

///Commands Opcodes: OGF(6b) | OCF(10b)
/* Some Abbreviation used in names:
 *  - LK   = Link Key
 *  - RD   = Read
 *  - WR   = Write
 *  - REM  = Remote
 *  - STG  = Settings
 *  - CON  = Connection
 *  - CHG  = Change
 *  - DFT  = Default
 *  - PER  = Periodic
 */

///HCI enumeration of possible Command OP Codes.
/*@TRACE*/
enum hci_opcode
{
    HCI_NO_OPERATION_CMD_OPCODE               = 0x0000,

    //Link Control Commands
    HCI_INQ_CMD_OPCODE                        = 0x0401,
    HCI_INQ_CANCEL_CMD_OPCODE                 = 0x0402,
    HCI_PER_INQ_MODE_CMD_OPCODE               = 0x0403,
    HCI_EXIT_PER_INQ_MODE_CMD_OPCODE          = 0x0404,
    HCI_CREATE_CON_CMD_OPCODE                 = 0x0405,
    HCI_DISCONNECT_CMD_OPCODE                 = 0x0406,
    HCI_CREATE_CON_CANCEL_CMD_OPCODE          = 0x0408,
    HCI_ACCEPT_CON_REQ_CMD_OPCODE             = 0x0409,
    HCI_REJECT_CON_REQ_CMD_OPCODE             = 0x040A,
    HCI_LK_REQ_REPLY_CMD_OPCODE               = 0x040B,
    HCI_LK_REQ_NEG_REPLY_CMD_OPCODE           = 0x040C,
    HCI_PIN_CODE_REQ_REPLY_CMD_OPCODE         = 0x040D,
    HCI_PIN_CODE_REQ_NEG_REPLY_CMD_OPCODE     = 0x040E,
    HCI_CHG_CON_PKT_TYPE_CMD_OPCODE           = 0x040F,
    HCI_AUTH_REQ_CMD_OPCODE                   = 0x0411,
    HCI_SET_CON_ENC_CMD_OPCODE                = 0x0413,
    HCI_CHG_CON_LK_CMD_OPCODE                 = 0x0415,
    HCI_MASTER_LK_CMD_OPCODE                  = 0x0417,
    HCI_REM_NAME_REQ_CMD_OPCODE               = 0x0419,
    HCI_REM_NAME_REQ_CANCEL_CMD_OPCODE        = 0x041A,
    HCI_RD_REM_SUPP_FEATS_CMD_OPCODE          = 0x041B,
    HCI_RD_REM_EXT_FEATS_CMD_OPCODE           = 0x041C,
    HCI_RD_REM_VER_INFO_CMD_OPCODE            = 0x041D,
    HCI_RD_CLK_OFF_CMD_OPCODE                 = 0x041F,
    HCI_RD_LMP_HDL_CMD_OPCODE                 = 0x0420,
    HCI_SETUP_SYNC_CON_CMD_OPCODE             = 0x0428,
    HCI_ACCEPT_SYNC_CON_REQ_CMD_OPCODE        = 0x0429,
    HCI_REJECT_SYNC_CON_REQ_CMD_OPCODE        = 0x042A,
    HCI_IO_CAP_REQ_REPLY_CMD_OPCODE           = 0x042B,
    HCI_USER_CFM_REQ_REPLY_CMD_OPCODE         = 0x042C,
    HCI_USER_CFM_REQ_NEG_REPLY_CMD_OPCODE     = 0x042D,
    HCI_USER_PASSKEY_REQ_REPLY_CMD_OPCODE     = 0x042E,
    HCI_USER_PASSKEY_REQ_NEG_REPLY_CMD_OPCODE = 0x042F,
    HCI_REM_OOB_DATA_REQ_REPLY_CMD_OPCODE     = 0x0430,
    HCI_REM_OOB_DATA_REQ_NEG_REPLY_CMD_OPCODE = 0x0433,
    HCI_IO_CAP_REQ_NEG_REPLY_CMD_OPCODE       = 0x0434,
    HCI_ENH_SETUP_SYNC_CON_CMD_OPCODE         = 0x043D,
    HCI_ENH_ACCEPT_SYNC_CON_CMD_OPCODE        = 0x043E,
    HCI_TRUNC_PAGE_CMD_OPCODE                 = 0x043F,
    HCI_TRUNC_PAGE_CAN_CMD_OPCODE             = 0x0440,
    HCI_SET_CON_SLV_BCST_CMD_OPCODE           = 0x0441,
    HCI_SET_CON_SLV_BCST_REC_CMD_OPCODE       = 0x0442,
    HCI_START_SYNC_TRAIN_CMD_OPCODE           = 0x0443,
    HCI_REC_SYNC_TRAIN_CMD_OPCODE             = 0x0444,
    HCI_REM_OOB_EXT_DATA_REQ_REPLY_CMD_OPCODE = 0x0445,

    //Link Policy Commands
    HCI_HOLD_MODE_CMD_OPCODE                  = 0x0801,
    HCI_SNIFF_MODE_CMD_OPCODE                 = 0x0803,
    HCI_EXIT_SNIFF_MODE_CMD_OPCODE            = 0x0804,
    HCI_PARK_STATE_CMD_OPCODE                 = 0x0805,
    HCI_EXIT_PARK_STATE_CMD_OPCODE            = 0x0806,
    HCI_QOS_SETUP_CMD_OPCODE                  = 0x0807,
    HCI_ROLE_DISCOVERY_CMD_OPCODE             = 0x0809,
    HCI_SWITCH_ROLE_CMD_OPCODE                = 0x080B,
    HCI_RD_LINK_POL_STG_CMD_OPCODE            = 0x080C,
    HCI_WR_LINK_POL_STG_CMD_OPCODE            = 0x080D,
    HCI_RD_DFT_LINK_POL_STG_CMD_OPCODE        = 0x080E,
    HCI_WR_DFT_LINK_POL_STG_CMD_OPCODE        = 0x080F,
    HCI_FLOW_SPEC_CMD_OPCODE                  = 0x0810,
    HCI_SNIFF_SUB_CMD_OPCODE                  = 0x0811,

    //Controller and Baseband Commands
    HCI_SET_EVT_MASK_CMD_OPCODE               = 0x0C01,
    HCI_RESET_CMD_OPCODE                      = 0x0C03,
    HCI_SET_EVT_FILTER_CMD_OPCODE             = 0x0C05,
    HCI_FLUSH_CMD_OPCODE                      = 0x0C08,
    HCI_RD_PIN_TYPE_CMD_OPCODE                = 0x0C09,
    HCI_WR_PIN_TYPE_CMD_OPCODE                = 0x0C0A,
    HCI_CREATE_NEW_UNIT_KEY_CMD_OPCODE        = 0x0C0B,
    HCI_RD_STORED_LK_CMD_OPCODE               = 0x0C0D,
    HCI_WR_STORED_LK_CMD_OPCODE               = 0x0C11,
    HCI_DEL_STORED_LK_CMD_OPCODE              = 0x0C12,
    HCI_WR_LOCAL_NAME_CMD_OPCODE              = 0x0C13,
    HCI_RD_LOCAL_NAME_CMD_OPCODE              = 0x0C14,
    HCI_RD_CON_ACCEPT_TO_CMD_OPCODE           = 0x0C15,
    HCI_WR_CON_ACCEPT_TO_CMD_OPCODE           = 0x0C16,
    HCI_RD_PAGE_TO_CMD_OPCODE                 = 0x0C17,
    HCI_WR_PAGE_TO_CMD_OPCODE                 = 0x0C18,
    HCI_RD_SCAN_EN_CMD_OPCODE                 = 0x0C19,
    HCI_WR_SCAN_EN_CMD_OPCODE                 = 0x0C1A,
    HCI_RD_PAGE_SCAN_ACT_CMD_OPCODE           = 0x0C1B,
    HCI_WR_PAGE_SCAN_ACT_CMD_OPCODE           = 0x0C1C,
    HCI_RD_INQ_SCAN_ACT_CMD_OPCODE            = 0x0C1D,
    HCI_WR_INQ_SCAN_ACT_CMD_OPCODE            = 0x0C1E,
    HCI_RD_AUTH_EN_CMD_OPCODE                 = 0x0C1F,
    HCI_WR_AUTH_EN_CMD_OPCODE                 = 0x0C20,
    HCI_RD_CLASS_OF_DEV_CMD_OPCODE            = 0x0C23,
    HCI_WR_CLASS_OF_DEV_CMD_OPCODE            = 0x0C24,
    HCI_RD_VOICE_STG_CMD_OPCODE               = 0x0C25,
    HCI_WR_VOICE_STG_CMD_OPCODE               = 0x0C26,
    HCI_RD_AUTO_FLUSH_TO_CMD_OPCODE           = 0x0C27,
    HCI_WR_AUTO_FLUSH_TO_CMD_OPCODE           = 0x0C28,
    HCI_RD_NB_BDCST_RETX_CMD_OPCODE           = 0x0C29,
    HCI_WR_NB_BDCST_RETX_CMD_OPCODE           = 0x0C2A,
    HCI_RD_HOLD_MODE_ACTIVITY_CMD_OPCODE      = 0x0C2B,
    HCI_WR_HOLD_MODE_ACTIVITY_CMD_OPCODE      = 0x0C2C,
    HCI_RD_TX_PWR_LVL_CMD_OPCODE              = 0x0C2D,
    HCI_RD_SYNC_FLOW_CTRL_EN_CMD_OPCODE       = 0x0C2E,
    HCI_WR_SYNC_FLOW_CTRL_EN_CMD_OPCODE       = 0x0C2F,
    HCI_SET_CTRL_TO_HOST_FLOW_CTRL_CMD_OPCODE = 0x0C31,
    HCI_HOST_BUF_SIZE_CMD_OPCODE              = 0x0C33,
    HCI_HOST_NB_CMP_PKTS_CMD_OPCODE           = 0x0C35,
    HCI_RD_LINK_SUPV_TO_CMD_OPCODE            = 0x0C36,
    HCI_WR_LINK_SUPV_TO_CMD_OPCODE            = 0x0C37,
    HCI_RD_NB_SUPP_IAC_CMD_OPCODE             = 0x0C38,
    HCI_RD_CURR_IAC_LAP_CMD_OPCODE            = 0x0C39,
    HCI_WR_CURR_IAC_LAP_CMD_OPCODE            = 0x0C3A,
    HCI_SET_AFH_HOST_CH_CLASS_CMD_OPCODE      = 0x0C3F,
    HCI_RD_INQ_SCAN_TYPE_CMD_OPCODE           = 0x0C42,
    HCI_WR_INQ_SCAN_TYPE_CMD_OPCODE           = 0x0C43,
    HCI_RD_INQ_MODE_CMD_OPCODE                = 0x0C44,
    HCI_WR_INQ_MODE_CMD_OPCODE                = 0x0C45,
    HCI_RD_PAGE_SCAN_TYPE_CMD_OPCODE          = 0x0C46,
    HCI_WR_PAGE_SCAN_TYPE_CMD_OPCODE          = 0x0C47,
    HCI_RD_AFH_CH_ASSESS_MODE_CMD_OPCODE      = 0x0C48,
    HCI_WR_AFH_CH_ASSESS_MODE_CMD_OPCODE      = 0x0C49,
    HCI_RD_EXT_INQ_RSP_CMD_OPCODE             = 0x0C51,
    HCI_WR_EXT_INQ_RSP_CMD_OPCODE             = 0x0C52,
    HCI_REFRESH_ENC_KEY_CMD_OPCODE            = 0x0C53,
    HCI_RD_SP_MODE_CMD_OPCODE                 = 0x0C55,
    HCI_WR_SP_MODE_CMD_OPCODE                 = 0x0C56,
    HCI_RD_LOC_OOB_DATA_CMD_OPCODE            = 0x0C57,
    HCI_RD_INQ_RSP_TX_PWR_LVL_CMD_OPCODE      = 0x0C58,
    HCI_WR_INQ_TX_PWR_LVL_CMD_OPCODE          = 0x0C59,
    HCI_RD_DFT_ERR_DATA_REP_CMD_OPCODE        = 0x0C5A,
    HCI_WR_DFT_ERR_DATA_REP_CMD_OPCODE        = 0x0C5B,
    HCI_ENH_FLUSH_CMD_OPCODE                  = 0x0C5F,
    HCI_SEND_KEYPRESS_NOTIF_CMD_OPCODE        = 0x0C60,
    HCI_SET_EVT_MASK_PAGE_2_CMD_OPCODE        = 0x0C63,
    HCI_RD_FLOW_CNTL_MODE_CMD_OPCODE          = 0x0C66,
    HCI_WR_FLOW_CNTL_MODE_CMD_OPCODE          = 0x0C67,
    HCI_RD_ENH_TX_PWR_LVL_CMD_OPCODE          = 0x0C68,
    HCI_RD_LE_HOST_SUPP_CMD_OPCODE            = 0x0C6C,
    HCI_WR_LE_HOST_SUPP_CMD_OPCODE            = 0x0C6D,
    HCI_SET_MWS_CHANNEL_PARAMS_CMD_OPCODE     = 0x0C6E,
    HCI_SET_EXTERNAL_FRAME_CONFIG_CMD_OPCODE  = 0x0C6F,
    HCI_SET_MWS_SIGNALING_CMD_OPCODE          = 0x0C70,
    HCI_SET_MWS_TRANSPORT_LAYER_CMD_OPCODE    = 0x0C71,
    HCI_SET_MWS_SCAN_FREQ_TABLE_CMD_OPCODE    = 0x0C72,
    HCI_SET_MWS_PATTERN_CONFIG_CMD_OPCODE     = 0x0C73,
    HCI_SET_RES_LT_ADDR_CMD_OPCODE            = 0x0C74,
    HCI_DEL_RES_LT_ADDR_CMD_OPCODE            = 0x0C75,
    HCI_SET_CON_SLV_BCST_DATA_CMD_OPCODE      = 0x0C76,
    HCI_RD_SYNC_TRAIN_PARAM_CMD_OPCODE        = 0x0C77,
    HCI_WR_SYNC_TRAIN_PARAM_CMD_OPCODE        = 0x0C78,
    HCI_RD_SEC_CON_HOST_SUPP_CMD_OPCODE       = 0x0C79,
    HCI_WR_SEC_CON_HOST_SUPP_CMD_OPCODE       = 0x0C7A,
    HCI_RD_AUTH_PAYL_TO_CMD_OPCODE            = 0x0C7B,
    HCI_WR_AUTH_PAYL_TO_CMD_OPCODE            = 0x0C7C,
    HCI_RD_LOC_OOB_EXT_DATA_CMD_OPCODE        = 0x0C7D,
    HCI_RD_EXT_PAGE_TO_CMD_OPCODE             = 0x0C7E,
    HCI_WR_EXT_PAGE_TO_CMD_OPCODE             = 0x0C7F,
    HCI_RD_EXT_INQ_LEN_CMD_OPCODE             = 0x0C80,
    HCI_WR_EXT_INQ_LEN_CMD_OPCODE             = 0x0C81,
    HCI_SET_ECO_BASE_INTV_CMD_OPCODE          = 0x0C82,
    HCI_CONFIG_DATA_PATH_CMD_OPCODE           = 0x0C83,
#if (BT_53)
    HCI_SET_MIN_ENC_KEY_SIZE_CMD_OPCODE       = 0x0C84,
#endif //(BT_53)

    //Info Params
    HCI_RD_LOCAL_VER_INFO_CMD_OPCODE               = 0x1001,
    HCI_RD_LOCAL_SUPP_CMDS_CMD_OPCODE              = 0x1002,
    HCI_RD_LOCAL_SUPP_FEATS_CMD_OPCODE             = 0x1003,
    HCI_RD_LOCAL_EXT_FEATS_CMD_OPCODE              = 0x1004,
    HCI_RD_BUF_SIZE_CMD_OPCODE                     = 0x1005,
    HCI_RD_BD_ADDR_CMD_OPCODE                      = 0x1009,
    HCI_RD_DATA_BLOCK_SIZE_CMD_OPCODE              = 0x100A,
    HCI_RD_LOCAL_SUPP_CODECS_CMD_OPCODE            = 0x100B,
    HCI_RD_LOCAL_SP_OPT_CMD_OPCODE                 = 0x100C,
    HCI_RD_LOCAL_SUPP_CODECS_V2_CMD_OPCODE         = 0x100D,
    HCI_RD_LOCAL_SUPP_CODEC_CAP_CMD_OPCODE         = 0x100E,
    HCI_RD_LOCAL_SUPP_CTRL_DELAY_CMD_OPCODE        = 0x100F,

    //Status Params
    HCI_RD_FAIL_CONTACT_CNT_CMD_OPCODE             = 0x1401,
    HCI_RST_FAIL_CONTACT_CNT_CMD_OPCODE            = 0x1402,
    HCI_RD_LINK_QUAL_CMD_OPCODE                    = 0x1403,
    HCI_RD_RSSI_CMD_OPCODE                         = 0x1405,
    HCI_RD_AFH_CH_MAP_CMD_OPCODE                   = 0x1406,
    HCI_RD_CLK_CMD_OPCODE                          = 0x1407,
    HCI_RD_ENC_KEY_SIZE_CMD_OPCODE                 = 0x1408,
    HCI_GET_MWS_TRANSPORT_LAYER_CONFIG_CMD_OPCODE  = 0x140C,

    //Testing Commands
    HCI_RD_LOOPBACK_MODE_CMD_OPCODE                = 0x1801,
    HCI_WR_LOOPBACK_MODE_CMD_OPCODE                = 0x1802,
    HCI_EN_DUT_MODE_CMD_OPCODE                     = 0x1803,
    HCI_WR_SP_DBG_MODE_CMD_OPCODE                  = 0x1804,
    HCI_WR_SEC_CON_TEST_MODE_CMD_OPCODE            = 0x180A,

    /// LE Commands Opcodes
    HCI_LE_SET_EVT_MASK_CMD_OPCODE                      = 0x2001,
    HCI_LE_RD_BUF_SIZE_CMD_OPCODE                       = 0x2002,
    HCI_LE_RD_LOCAL_SUPP_FEATS_CMD_OPCODE               = 0x2003,
    HCI_LE_SET_RAND_ADDR_CMD_OPCODE                     = 0x2005,
    HCI_LE_SET_ADV_PARAM_CMD_OPCODE                     = 0x2006,
    HCI_LE_RD_ADV_CHNL_TX_PW_CMD_OPCODE                 = 0x2007,
    HCI_LE_SET_ADV_DATA_CMD_OPCODE                      = 0x2008,
    HCI_LE_SET_SCAN_RSP_DATA_CMD_OPCODE                 = 0x2009,
    HCI_LE_SET_ADV_EN_CMD_OPCODE                        = 0x200A,
    HCI_LE_SET_SCAN_PARAM_CMD_OPCODE                    = 0x200B,
    HCI_LE_SET_SCAN_EN_CMD_OPCODE                       = 0x200C,
    HCI_LE_CREATE_CON_CMD_OPCODE                        = 0x200D,
    HCI_LE_CREATE_CON_CANCEL_CMD_OPCODE                 = 0x200E,
    HCI_LE_RD_WLST_SIZE_CMD_OPCODE                      = 0x200F,
    HCI_LE_CLEAR_WLST_CMD_OPCODE                        = 0x2010,
    HCI_LE_ADD_DEV_TO_WLST_CMD_OPCODE                   = 0x2011,
    HCI_LE_RMV_DEV_FROM_WLST_CMD_OPCODE                 = 0x2012,
    HCI_LE_CON_UPDATE_CMD_OPCODE                        = 0x2013,
    HCI_LE_SET_HOST_CH_CLASS_CMD_OPCODE                 = 0x2014,
    HCI_LE_RD_CH_MAP_CMD_OPCODE                         = 0x2015,
    HCI_LE_RD_REM_FEATS_CMD_OPCODE                      = 0x2016,
    HCI_LE_ENC_CMD_OPCODE                               = 0x2017,
    HCI_LE_RAND_CMD_OPCODE                              = 0x2018,
    HCI_LE_EN_ENC_CMD_OPCODE                            = 0x2019,
    HCI_LE_LTK_REQ_REPLY_CMD_OPCODE                     = 0x201A,
    HCI_LE_LTK_REQ_NEG_REPLY_CMD_OPCODE                 = 0x201B,
    HCI_LE_RD_SUPP_STATES_CMD_OPCODE                    = 0x201C,
    HCI_LE_RX_TEST_V1_CMD_OPCODE                        = 0x201D,
    HCI_LE_TX_TEST_V1_CMD_OPCODE                        = 0x201E,
    HCI_LE_TEST_END_CMD_OPCODE                          = 0x201F,
    HCI_LE_REM_CON_PARAM_REQ_REPLY_CMD_OPCODE           = 0x2020,
    HCI_LE_REM_CON_PARAM_REQ_NEG_REPLY_CMD_OPCODE       = 0x2021,
    HCI_LE_SET_DATA_LEN_CMD_OPCODE                      = 0x2022,
    HCI_LE_RD_SUGGTED_DFT_DATA_LEN_CMD_OPCODE           = 0x2023,
    HCI_LE_WR_SUGGTED_DFT_DATA_LEN_CMD_OPCODE           = 0x2024,
    HCI_LE_RD_LOC_P256_PUB_KEY_CMD_OPCODE               = 0x2025,
    HCI_LE_GEN_DHKEY_V1_CMD_OPCODE                      = 0x2026,
    HCI_LE_ADD_DEV_TO_RSLV_LIST_CMD_OPCODE              = 0x2027,
    HCI_LE_RMV_DEV_FROM_RSLV_LIST_CMD_OPCODE            = 0x2028,
    HCI_LE_CLEAR_RSLV_LIST_CMD_OPCODE                   = 0x2029,
    HCI_LE_RD_RSLV_LIST_SIZE_CMD_OPCODE                 = 0x202A,
    HCI_LE_RD_PEER_RSLV_ADDR_CMD_OPCODE                 = 0x202B,
    HCI_LE_RD_LOC_RSLV_ADDR_CMD_OPCODE                  = 0x202C,
    HCI_LE_SET_ADDR_RESOL_EN_CMD_OPCODE                 = 0x202D,
    HCI_LE_SET_RSLV_PRIV_ADDR_TO_CMD_OPCODE             = 0x202E,
    HCI_LE_RD_MAX_DATA_LEN_CMD_OPCODE                   = 0x202F,
    HCI_LE_RD_PHY_CMD_OPCODE                            = 0x2030,
    HCI_LE_SET_DFT_PHY_CMD_OPCODE                       = 0x2031,
    HCI_LE_SET_PHY_CMD_OPCODE                           = 0x2032,
    HCI_LE_RX_TEST_V2_CMD_OPCODE                        = 0x2033,
    HCI_LE_TX_TEST_V2_CMD_OPCODE                        = 0x2034,
    HCI_LE_SET_ADV_SET_RAND_ADDR_CMD_OPCODE             = 0x2035,
    HCI_LE_SET_EXT_ADV_PARAM_CMD_OPCODE                 = 0x2036,
    HCI_LE_SET_EXT_ADV_DATA_CMD_OPCODE                  = 0x2037,
    HCI_LE_SET_EXT_SCAN_RSP_DATA_CMD_OPCODE             = 0x2038,
    HCI_LE_SET_EXT_ADV_EN_CMD_OPCODE                    = 0x2039,
    HCI_LE_RD_MAX_ADV_DATA_LEN_CMD_OPCODE               = 0x203A,
    HCI_LE_RD_NB_SUPP_ADV_SETS_CMD_OPCODE               = 0x203B,
    HCI_LE_RMV_ADV_SET_CMD_OPCODE                       = 0x203C,
    HCI_LE_CLEAR_ADV_SETS_CMD_OPCODE                    = 0x203D,
    HCI_LE_SET_PER_ADV_PARAM_CMD_OPCODE                 = 0x203E,
    HCI_LE_SET_PER_ADV_DATA_CMD_OPCODE                  = 0x203F,
    HCI_LE_SET_PER_ADV_EN_CMD_OPCODE                    = 0x2040,
    HCI_LE_SET_EXT_SCAN_PARAM_CMD_OPCODE                = 0x2041,
    HCI_LE_SET_EXT_SCAN_EN_CMD_OPCODE                   = 0x2042,
    HCI_LE_EXT_CREATE_CON_CMD_OPCODE                    = 0x2043,
    HCI_LE_PER_ADV_CREATE_SYNC_CMD_OPCODE               = 0x2044,
    HCI_LE_PER_ADV_CREATE_SYNC_CANCEL_CMD_OPCODE        = 0x2045,
    HCI_LE_PER_ADV_TERM_SYNC_CMD_OPCODE                 = 0x2046,
    HCI_LE_ADD_DEV_TO_PER_ADV_LIST_CMD_OPCODE           = 0x2047,
    HCI_LE_RMV_DEV_FROM_PER_ADV_LIST_CMD_OPCODE         = 0x2048,
    HCI_LE_CLEAR_PER_ADV_LIST_CMD_OPCODE                = 0x2049,
    HCI_LE_RD_PER_ADV_LIST_SIZE_CMD_OPCODE              = 0x204A,
    HCI_LE_RD_TX_PWR_CMD_OPCODE                         = 0x204B,
    HCI_LE_RD_RF_PATH_COMP_CMD_OPCODE                   = 0x204C,
    HCI_LE_WR_RF_PATH_COMP_CMD_OPCODE                   = 0x204D,
    HCI_LE_SET_PRIV_MODE_CMD_OPCODE                     = 0x204E,
    HCI_LE_RX_TEST_V3_CMD_OPCODE                        = 0x204F,
    HCI_LE_TX_TEST_V3_CMD_OPCODE                        = 0x2050,
    HCI_LE_SET_CONLESS_CTE_TX_PARAM_CMD_OPCODE          = 0x2051,
    HCI_LE_SET_CONLESS_CTE_TX_EN_CMD_OPCODE             = 0x2052,
    HCI_LE_SET_CONLESS_IQ_SAMPL_EN_CMD_OPCODE           = 0x2053,
    HCI_LE_SET_CON_CTE_RX_PARAM_CMD_OPCODE              = 0x2054,
    HCI_LE_SET_CON_CTE_TX_PARAM_CMD_OPCODE              = 0x2055,
    HCI_LE_CON_CTE_REQ_EN_CMD_OPCODE                    = 0x2056,
    HCI_LE_CON_CTE_RSP_EN_CMD_OPCODE                    = 0x2057,
    HCI_LE_RD_ANTENNA_INF_CMD_OPCODE                    = 0x2058,
    HCI_LE_SET_PER_ADV_REC_EN_CMD_OPCODE                = 0x2059,
#if (BLE_PAST)
    HCI_LE_PER_ADV_SYNC_TRANSF_CMD_OPCODE               = 0x205A,
    HCI_LE_PER_ADV_SET_INFO_TRANSF_CMD_OPCODE           = 0x205B,
    HCI_LE_SET_PER_ADV_SYNC_TRANSF_PARAM_CMD_OPCODE     = 0x205C,
    HCI_LE_SET_DFT_PER_ADV_SYNC_TRANSF_PARAM_CMD_OPCODE = 0x205D,
#endif // (BLE_PAST)
    HCI_LE_GEN_DHKEY_V2_CMD_OPCODE                      = 0x205E,
    HCI_LE_MOD_SLEEP_CLK_ACC_CMD_OPCODE                 = 0x205F,
    HCI_LE_RD_BUF_SIZE_V2_CMD_OPCODE                    = 0x2060,
#if (BLE_CIS || BLE_BIS)
    HCI_LE_RD_ISO_TX_SYNC_CMD_OPCODE                    = 0x2061,
#endif // (BLE_CIS || BLE_BIS)
#if (BLE_CIS)
    HCI_LE_SET_CIG_PARAMS_CMD_OPCODE                    = 0x2062,
    HCI_LE_SET_CIG_PARAMS_TEST_CMD_OPCODE               = 0x2063,
    HCI_LE_CREATE_CIS_CMD_OPCODE                        = 0x2064,
    HCI_LE_REMOVE_CIG_CMD_OPCODE                        = 0x2065,
    HCI_LE_ACCEPT_CIS_REQ_CMD_OPCODE                    = 0x2066,
    HCI_LE_REJECT_CIS_REQ_CMD_OPCODE                    = 0x2067,
#endif // (BLE_CIS)
#if (BLE_BIS)
    HCI_LE_CREATE_BIG_CMD_OPCODE                        = 0x2068,
    HCI_LE_CREATE_BIG_TEST_CMD_OPCODE                   = 0x2069,
    HCI_LE_TERMINATE_BIG_CMD_OPCODE                     = 0x206A,
    HCI_LE_BIG_CREATE_SYNC_CMD_OPCODE                   = 0x206B,
    HCI_LE_BIG_TERMINATE_SYNC_CMD_OPCODE                = 0x206C,
#endif // (BLE_BIS)
    HCI_LE_REQ_PEER_SCA_CMD_OPCODE                      = 0x206D,
#if (BLE_CIS || BLE_BIS)
    HCI_LE_SETUP_ISO_DATA_PATH_CMD_OPCODE               = 0x206E,
    HCI_LE_REMOVE_ISO_DATA_PATH_CMD_OPCODE              = 0x206F,
    HCI_LE_ISO_TX_TEST_CMD_OPCODE                       = 0x2070,
    HCI_LE_ISO_RX_TEST_CMD_OPCODE                       = 0x2071,
    HCI_LE_ISO_READ_TEST_COUNTERS_CMD_OPCODE            = 0x2072,
    HCI_LE_ISO_TEST_END_CMD_OPCODE                      = 0x2073,
#endif // (BLE_CIS || BLE_BIS)
    HCI_LE_SET_HOST_FEATURE_CMD_OPCODE                  = 0x2074,
#if (BLE_CIS || BLE_BIS)
    HCI_LE_RD_ISO_LINK_QUALITY_CMD_OPCODE               = 0x2075,
#endif // (BLE_CIS || BLE_BIS)
    HCI_LE_ENH_RD_TX_PWR_LVL_CMD_OPCODE                 = 0x2076,
    HCI_LE_RD_REMOTE_TX_PWR_LVL_CMD_OPCODE              = 0x2077,
    HCI_LE_SET_PATH_LOSS_REP_PARAM_CMD_OPCODE           = 0x2078,
    HCI_LE_SET_PATH_LOSS_REP_EN_CMD_OPCODE              = 0x2079,
    HCI_LE_SET_TX_POWER_REP_EN_CMD_OPCODE               = 0x207A,
    HCI_LE_TX_TEST_V4_CMD_OPCODE                        = 0x207B,
#if (BT_53)
    HCI_LE_SET_DATA_RELATED_ADDR_CHG_CMD_OPCODE         = 0x207C,
    HCI_LE_SET_DFT_SUBRATE_CMD_OPCODE                   = 0x207D,
    HCI_LE_SUBRATE_REQ_CMD_OPCODE                       = 0x207E,
#endif // (BT_53)

    ///Debug commands - OGF = 0x3F (spec)
    HCI_DBG_RD_MEM_CMD_OPCODE                      = 0xFC01,
    HCI_DBG_WR_MEM_CMD_OPCODE                      = 0xFC02,
    HCI_DBG_DEL_PAR_CMD_OPCODE                     = 0xFC03,
    HCI_DBG_ID_FLASH_CMD_OPCODE                    = 0xFC05,
    HCI_DBG_ER_FLASH_CMD_OPCODE                    = 0xFC06,
    HCI_DBG_WR_FLASH_CMD_OPCODE                    = 0xFC07,
    HCI_DBG_RD_FLASH_CMD_OPCODE                    = 0xFC08,
    HCI_DBG_RD_PAR_CMD_OPCODE                      = 0xFC09,
    HCI_DBG_WR_PAR_CMD_OPCODE                      = 0xFC0A,
    HCI_DBG_WLAN_COEX_CMD_OPCODE                   = 0xFC0B,
    HCI_DBG_WLAN_COEXTST_SCEN_CMD_OPCODE           = 0xFC0D,
    HCI_DBG_BT_SEND_LMP_CMD_OPCODE                 = 0xFC0E,
    HCI_DBG_SET_LOCAL_CLOCK_CMD_OPCODE             = 0xFC0F,
    HCI_DBG_RD_KE_STATS_CMD_OPCODE                 = 0xFC10,
    HCI_DBG_PLF_RESET_CMD_OPCODE                   = 0xFC11,
    HCI_DBG_RD_MEM_INFO_CMD_OPCODE                 = 0xFC12,
    HCI_VS_SET_PREF_SLAVE_LATENCY_CMD_OPCODE       = 0xFC13,
    HCI_VS_SET_PREF_SLAVE_EVT_DUR_CMD_OPCODE       = 0xFC14,
    HCI_VS_SET_MAX_RX_SIZE_AND_TIME_CMD_OPCODE     = 0xFC15,
    HCI_VS_SET_PRIO_ELEVATION_CMD_OPCODE           = 0xFC16,
#if (RW_DEBUG)
    HCI_DBG_LE_CON_EVT_TRACE_CMD_OPCODE            = 0xFC17,
#endif // (RW_DEBUG)
#if (BLE_CH_SCAN_SUPPORT)
    HCI_VS_LE_CH_SCAN_CMD_OPCODE                   = 0xFC18,
    HCI_VS_LE_CH_SCAN_END_CMD_OPCODE               = 0xFC19,
#endif // (BLE_CH_SCAN_SUPPORT)
    HCI_DBG_BLE_REG_RD_CMD_OPCODE                  = 0xFC30,
    HCI_DBG_BLE_REG_WR_CMD_OPCODE                  = 0xFC31,
    HCI_DBG_SEND_LLCP_CMD_OPCODE                   = 0xFC35,
    HCI_DBG_LLCP_DISCARD_CMD_OPCODE                = 0xFC36,
    HCI_DBG_RF_REG_RD_CMD_OPCODE                   = 0xFC39,
    HCI_DBG_RF_REG_WR_CMD_OPCODE                   = 0xFC3A,
    HCI_DBG_RF_SWITCH_CLK_CMD_OPCODE               = 0xFC3C,
    HCI_DBG_RF_WR_DATA_TX_CMD_OPCODE               = 0xFC3D,
    HCI_DBG_RF_RD_DATA_RX_CMD_OPCODE               = 0xFC3E,
    HCI_DBG_RF_CNTL_TX_CMD_OPCODE                  = 0xFC3F,
    HCI_DBG_RF_SYNC_P_CNTL_CMD_OPCODE              = 0xFC40,

#if (RW_DEBUG && BT_EMB_PRESENT)
    HCI_DBG_BT_DISCARD_LMP_EN_CMD_OPCODE           = 0xFC44,
#endif //RW_DEBUG && BT_EMB_PRESENT

    HCI_DBG_MWS_COEX_CMD_OPCODE                    = 0xFC45,
    HCI_DBG_MWS_COEXTST_SCEN_CMD_OPCODE            = 0xFC46,

    HCI_VS_I2C_READ_CMD_OPCODE                     = 0xFC47,
    HCI_VS_I2C_WRITE_CMD_OPCODE                    = 0xFC48,

#if (BT_READ_PICONET_CLOCK)
    HCI_VS_RD_PICONET_CLOCK_CMD_OPCODE             = 0xFC50,
#endif // (BT_READ_PICONET_CLOCK)

#if (BLE_ISO_MODE_0)
    /// Vendor Specific commands for ISO Mode 0
    HCI_VS_MIC_LESS_SET_CMD_OPCODE                   = 0xFC51,
    HCI_VS_SETUP_AM0_DATA_PATH_CMD_OPCODE            = 0xFC52,
    HCI_VS_REMOVE_AM0_DATA_PATH_CMD_OPCODE           = 0xFC53,
    HCI_VS_SETUP_AM0_STREAM_CMD_OPCODE               = 0xFC54,
    HCI_VS_REMOVE_AM0_STREAM_CMD_OPCODE              = 0xFC55,
#endif // (BLE_ISO_MODE_0)

    /// Debug commands for ISO
    HCI_DBG_ISO_SET_PARAM_CMD_OPCODE               = 0xFC57,

#if CRYPTO_UT
    HCI_DBG_TEST_CRYPTO_FUNC_CMD_OPCODE            = 0xFC60,
#endif //CRYPTO_UT

#if RW_DEBUG
    HCI_DBG_TEST_SCH_PLAN_SET_CMD_OPCODE           = 0xFC61,
    HCI_DBG_TEST_SCH_PLAN_REM_CMD_OPCODE           = 0xFC62,
    HCI_DBG_TEST_SCH_PLAN_CHK_CMD_OPCODE           = 0xFC63,
    HCI_DBG_TEST_SCH_PLAN_REQ_CMD_OPCODE           = 0xFC64,
#endif //RW_DEBUG

#if BLE_IQ_GEN
    HCI_DBG_IQGEN_CFG_CMD_OPCODE                   = 0xFC65,
#endif //BLE_IQ_GEN

#if (AUDIO_SYNC_SUPPORT)
    HCI_VS_AUSY_CON_EVT_CNT_GET_CMD_OPCODE        = 0xFC66,
    HCI_VS_AUSY_EVT_TX_TIME_GET_CMD_OPCODE        = 0xFC67,
    HCI_VS_AUSY_LAST_RX_TIME_GET_CMD_OPCODE       = 0xFC68,
    HCI_VS_AUSY_CLOCK_SAMPLE_GET_CMD_OPCODE       = 0xFC69,
    HCI_VS_AUSY_CLOCK_CONVERT_CMD_OPCODE          = 0xFC6A,
    HCI_VS_AUSY_CIS_EVT_CTRL_CMD_OPCODE           = 0xFC6B,
    HCI_VS_AUSY_BIS_EVT_CTRL_CMD_OPCODE           = 0xFC6C,
#endif // (AUDIO_SYNC_SUPPORT)

#if (BT_HCI_TEST_MODE)
    HCI_VS_RX_TEST_CMD_OPCODE                     = 0xFC70,
    HCI_VS_TX_TEST_CMD_OPCODE                     = 0xFC71,
    HCI_VS_TEST_END_CMD_OPCODE                    = 0xFC72,
#endif // (BT_HCI_TEST_MODE)

#if (BT_53)
    HCI_DBG_SET_MAX_ENC_KEY_SIZE_CMD_OPCODE       = 0xFC73,
#endif // (BT_53)

#if ((BLE_EMB_PRESENT && !HOST_PRESENT) || (!EMB_PRESENT && BLE_HOST_PRESENT))
    HCI_VS_LE_DECRYPT_CMD_OPCODE                  = 0xFC74,
#endif // ((BLE_EMB_PRESENT && !HOST_PRESENT) || (!EMB_PRESENT && BLE_HOST_PRESENT))

#if (EAVESDROPPING_SUPPORT)
    HCI_ECI_SET_OPERATIONAL_MODE_CMD_OPCODE         = 0xFC80,
    HCI_ECI_SET_BINAURAL_ACK_MODE_CMD_OPCODE        = 0xFC81, // Deprecated
    HCI_ECI_SET_POWER_CMD_OPCODE                    = 0xFC82,
    HCI_ECI_SET_CHANNEL_CLASSIFICATION_CMD_OPCODE   = 0xFC83,
    HCI_ECI_CAPTURE_MASTER_CLOCK_CMD_OPCODE         = 0xFC84,
    HCI_ECI_GET_AES_CCM_COUNTER_CMD_OPCODE          = 0xFC85,
    HCI_ECI_SET_EAVESDROPPING_STATE_CMD_OPCODE      = 0xFC86,
    HCI_ECI_SET_ACL_CONNECTION_CMD_OPCODE           = 0xFC87,
    HCI_ECI_SET_AFH_CHANNEL_MAP_CMD_OPCODE          = 0xFC88,
    HCI_ECI_SET_ENCRYPTION_CMD_OPCODE               = 0xFC89,
    HCI_ECI_SET_PACKET_TYPE_TABLE_CMD_OPCODE        = 0xFC8A,
    HCI_ECI_SET_MAX_SLOT_CMD_OPCODE                 = 0xFC8B,
    HCI_ECI_SET_SCO_CONNECTION_CMD_OPCODE           = 0xFC8C,
    HCI_ECI_SET_MIN_MAX_POWER_CMD_OPCODE            = 0xFC8D,
    HCI_ECI_SET_MASTER_CLOCK_CMD_OPCODE             = 0xFC8E,
    HCI_ECI_SET_AES_CCM_COUNTER_CMD_OPCODE          = 0xFC8F,
    HCI_ECI_SET_CLOCK_OFFSETS_CMD_OPCODE            = 0xFC90,
    HCI_ECI_CLEAR_CAPTURE_MASTER_CLOCK_CMD_OPCODE   = 0xFC91,
    HCI_ECI_SET_LRD_EAVESDROPPING_CMD_OPCODE        = 0xFC92,
    HCI_ECI_ALLOW_TX_PROG_CMD_OPCODE                = 0xFC93,
    HCI_ECI_SET_TX_SEQN_CMD_OPCODE                  = 0xFC94,
    HCI_ECI_DELIVER_CORR_ACL_PAYL_CMD_OPCODE        = 0xFC95,
    HCI_ECI_RX_STATS_REPORTING_CMD_OPCODE           = 0xFC96,
    HCI_ECI_SAMPLE_ESCO_TX_SEQN_CMD_OPCODE          = 0xFC97,
    HCI_ECI_SYNC_ESCO_TX_SEQN_CMD_OPCODE            = 0xFC98,
    HCI_VS_SET_SEG_INQ_SCAN_CMD_OPCODE              = 0xFCA0,
    HCI_VS_SET_SEG_PAGE_SCAN_CMD_OPCODE             = 0xFCA1,
    HCI_VS_SET_CUST_ADV_CMD_OPCODE                  = 0xFCA2,
    HCI_VS_SET_CUST_ACL_SCHED_CMD_OPCODE            = 0xFCA3,
    HCI_VS_SET_HOST_MAX_SLOT_CMD_OPCODE             = 0xFCA4,
    HCI_VS_GET_DEEP_SLEEP_COUNTER_CMD_OPCODE        = 0xFCA5,
    HCI_VS_PREVENT_DEEP_SLEEP_CMD_OPCODE            = 0xFCA6,
    HCI_VS_SET_WLAN_COEX_PRIORITIES_CMD_OPCODE      = 0xFCA7,
    HCI_VS_SET_SCO_LINK_REQUEST_DELAY_CMD_OPCODE    = 0xFCA8,
    HCI_VS_SET_CUST_PAGING_CMD_OPCODE               = 0xFCA9,
    HCI_VS_NOTIFY_CLOCK_WRAP_CMD_OPCODE             = 0xFCAA,
    HCI_VS_NOTIFY_SNIFF_EVENTS_CMD_OPCODE           = 0xFCAB,
    HCI_VS_LE_ENABLE_ENH_SET_DATA_LEN_CMD_OPCODE    = 0xFCAC,
    HCI_VS_LE_ENH_SET_DATA_LEN_CMD_OPCODE           = 0xFCAD,
    HCI_VS_LE_ENH_WR_SUGG_DFT_DATA_LEN_CMD_OPCODE   = 0xFCAE,
    HCI_VS_LE_SET_THROTTLING_CMD_OPCODE             = 0xFCAF, // Deprecated
    HCI_VS_LE_SET_CUST_TRANSMITTER_TEST_CMD_OPCODE  = 0xFCB0,
    HCI_VS_HANDLE_MIC_INVALID_CMD_OPCODE            = 0xFCB1,
    HCI_VS_LE_SET_MANUAL_ADV_CMD_OPCODE             = 0xFCB2,
    HCI_VS_LE_SET_SEG_SCAN_CMD_OPCODE               = 0xFCB3,
#endif // EAVESDROPPING_SUPPORT
    HCI_DBG_SET_TRC_CFG_EXT_CMD_OPCODE            = 0xFC84,
    HCI_DBG_RESERVED1_EXT_CMD_OPCODE              = 0xFC85,
    HCI_DBG_RESERVED2_EXT_CMD_OPCODE              = 0xFC86,
    HCI_DBG_RESERVED6_EXT_CMD_OPCODE              = 0xFC8A,

    /*
     * Ellisys defined HCI VS Commands for EBQ control, from default IXIT Ellisys HCI VCS Offset 0xE0.
     * 0xE0-0xE7 reserved for future extension.
     */
#if (RW_DEBUG) && (RW_MWS_COEX)
    HCI_VS_EBQ_INIT_SAM_NEGO_CMD_OPCODE             = 0xFCE0,
#endif // (RW_DEBUG) && (RW_MWS_COEX)

    // Sifli custom command
    // NVDS config command
    HCI_NVDS_DATA_UPDATE_CMD_OPCODE               = 0xFD80,
    // WLAN CO-EX command
    HCI_WLAN_COEX_ENABLE_CMD_OPCODE               = 0xFD81,
    HCI_WLAN_COEX_SET_PRIORITY_CMD_OPCODE         = 0xFD82,
    HCI_WLAN_COEX_SET_FILTER_CHANNEL_CMD_OPCODE   = 0xFD83,
    HCI_DBG_SET_TRC_CFG_CMD_OPCODE                = 0xFD84,
    HCI_DBG_RESERVED1_CMD_OPCODE                  = 0xFD85,
    HCI_DBG_RESERVED2_CMD_OPCODE                  = 0xFD86,
    HCI_DBG_RESERVED3_CMD_OPCODE                  = 0xFD87,
    HCI_DBG_RESERVED4_CMD_OPCODE                  = 0xFD88,
    HCI_DBG_RESERVED5_CMD_OPCODE                  = 0xFD89,
    HCI_DBG_RESERVED6_CMD_OPCODE                  = 0xFD8A,
    HCI_DBG_RESERVED7_CMD_OPCODE                  = 0xFD8B,
    HCI_DBG_RESERVED8_CMD_OPCODE                  = 0xFD8C,
    HCI_DBG_RESERVED9_CMD_OPCODE                  = 0xFD8D,
    HCI_DBG_RESERVED10_CMD_OPCODE                 = 0xFD8E,
    HCI_DBG_RESERVED11_CMD_OPCODE                 = 0xFD8F,
    HCI_DBG_RESERVED12_CMD_OPCODE                 = 0xFD90,

};

/**************************************************************************************
 **************                        HCI EVENTS                      ****************
 **************************************************************************************/

///Event Codes
/*@TRACE*/
enum hci_evt_code
{
    HCI_INQ_CMP_EVT_CODE                       = 0x01,
    HCI_INQ_RES_EVT_CODE                       = 0x02,
    HCI_CON_CMP_EVT_CODE                       = 0x03,
    HCI_CON_REQ_EVT_CODE                       = 0x04,
    HCI_DISC_CMP_EVT_CODE                      = 0x05,
    HCI_AUTH_CMP_EVT_CODE                      = 0x06,
    HCI_REM_NAME_REQ_CMP_EVT_CODE              = 0x07,
    HCI_ENC_CHG_EVT_CODE                       = 0x08,
    HCI_CHG_CON_LK_CMP_EVT_CODE                = 0x09,
    HCI_MASTER_LK_CMP_EVT_CODE                 = 0x0A,
    HCI_RD_REM_SUPP_FEATS_CMP_EVT_CODE         = 0x0B,
    HCI_RD_REM_VER_INFO_CMP_EVT_CODE           = 0x0C,
    HCI_QOS_SETUP_CMP_EVT_CODE                 = 0x0D,
    HCI_CMD_CMP_EVT_CODE                       = 0x0E,
    HCI_CMD_STATUS_EVT_CODE                    = 0x0F,
    HCI_HW_ERR_EVT_CODE                        = 0x10,
    HCI_FLUSH_OCCURRED_EVT_CODE                = 0x11,
    HCI_ROLE_CHG_EVT_CODE                      = 0x12,
    HCI_NB_CMP_PKTS_EVT_CODE                   = 0x13,
    HCI_MODE_CHG_EVT_CODE                      = 0x14,
    HCI_RETURN_LINK_KEYS_EVT_CODE              = 0x15,
    HCI_PIN_CODE_REQ_EVT_CODE                  = 0x16,
    HCI_LK_REQ_EVT_CODE                        = 0x17,
    HCI_LK_NOTIF_EVT_CODE                      = 0x18,
    HCI_DATA_BUF_OVFLW_EVT_CODE                = 0x1A,
    HCI_MAX_SLOT_CHG_EVT_CODE                  = 0x1B,
    HCI_RD_CLK_OFF_CMP_EVT_CODE                = 0x1C,
    HCI_CON_PKT_TYPE_CHG_EVT_CODE              = 0x1D,
    HCI_QOS_VIOL_EVT_CODE                      = 0x1E,
    HCI_PAGE_SCAN_REPET_MODE_CHG_EVT_CODE      = 0x20,
    HCI_FLOW_SPEC_CMP_EVT_CODE                 = 0x21,
    HCI_INQ_RES_WITH_RSSI_EVT_CODE             = 0x22,
    HCI_RD_REM_EXT_FEATS_CMP_EVT_CODE          = 0x23,
    HCI_SYNC_CON_CMP_EVT_CODE                  = 0x2C,
    HCI_SYNC_CON_CHG_EVT_CODE                  = 0x2D,
    HCI_SNIFF_SUB_EVT_CODE                     = 0x2E,
    HCI_EXT_INQ_RES_EVT_CODE                   = 0x2F,
    HCI_ENC_KEY_REFRESH_CMP_EVT_CODE           = 0x30,
    HCI_IO_CAP_REQ_EVT_CODE                    = 0x31,
    HCI_IO_CAP_RSP_EVT_CODE                    = 0x32,
    HCI_USER_CFM_REQ_EVT_CODE                  = 0x33,
    HCI_USER_PASSKEY_REQ_EVT_CODE              = 0x34,
    HCI_REM_OOB_DATA_REQ_EVT_CODE              = 0x35,
    HCI_SP_CMP_EVT_CODE                        = 0x36,
    HCI_LINK_SUPV_TO_CHG_EVT_CODE              = 0x38,
    HCI_ENH_FLUSH_CMP_EVT_CODE                 = 0x39,
    HCI_USER_PASSKEY_NOTIF_EVT_CODE            = 0x3B,
    HCI_KEYPRESS_NOTIF_EVT_CODE                = 0x3C,
    HCI_REM_HOST_SUPP_FEATS_NOTIF_EVT_CODE     = 0x3D,
    HCI_LE_META_EVT_CODE                       = 0x3E,
    HCI_MAX_EVT_MSK_PAGE_1_CODE                = 0x40,
    HCI_TRIGGERED_CLOCK_CAPTURE_CODE           = 0x4E,
    HCI_SYNC_TRAIN_CMP_EVT_CODE                = 0x4F,
    HCI_SYNC_TRAIN_REC_EVT_CODE                = 0x50,
    HCI_CON_SLV_BCST_REC_EVT_CODE              = 0x51,
    HCI_CON_SLV_BCST_TO_EVT_CODE               = 0x52,
    HCI_TRUNC_PAGE_CMP_EVT_CODE                = 0x53,
    HCI_SLV_PAGE_RSP_TO_EVT_CODE               = 0x54,
    HCI_CON_SLV_BCST_CH_MAP_CHG_EVT_CODE       = 0x55,
    HCI_AUTH_PAYL_TO_EXP_EVT_CODE              = 0x57,
    HCI_SAM_STATUS_CHANGE_EVT_CODE             = 0x58,
#if (BT_53)
    HCI_ENC_CHG_V2_EVT_CODE                    = 0x59,
    HCI_MAX_EVT_MSK_PAGE_2_CODE                = 0x60,
#else // (BT_53)
    HCI_MAX_EVT_MSK_PAGE_2_CODE                = 0x59,
#endif // (BT_53)
    HCI_DBG_META_EVT_CODE                      = 0xFF,

#if (EAVESDROPPING_SUPPORT)
    HCI_ECI_ACL_CONNECTION_CHANGED_EVT_SUBCODE     = 0x80,
    HCI_ECI_AFH_CHANNEL_MAP_CHANGED_EVT_SUBCODE    = 0x81,
    HCI_ECI_ENCRYPTION_CHANGED_EVT_SUBCODE         = 0x82,
    HCI_ECI_PACKET_TYPE_TABLE_CHANGED_EVT_SUBCODE  = 0x83,
    HCI_ECI_MAX_SLOT_CHANGED_EVT_SUBCODE           = 0x84,
    HCI_ECI_SCO_CONNECTION_CHANGED_EVT_SUBCODE     = 0x85,
    HCI_ECI_MIN_MAX_POWER_REACHED_EVT_SUBCODE      = 0x86,
    HCI_ECI_MASTER_CLOCK_CAPTURED_EVT_SUBCODE      = 0x87,
    HCI_ECI_AES_CCM_COUNTER_EVT_SUBCODE            = 0x88,
    HCI_ECI_ADJUST_POWER_EVT_SUBCODE               = 0x89,
    HCI_ECI_CHANNEL_CLASSIFICATION_EVT_SUBCODE     = 0x8A,
    HCI_ECI_MASTER_CLOCK_ADJUSTED_EVT_SUBCODE      = 0x8B,
    HCI_ECI_RX_TIMEOUT_EVT_SUBCODE                 = 0x8C,
    HCI_ECI_MIC_INVALID_EVT_SUBCODE                = 0x8D,
    HCI_VS_SCO_LINK_REQUEST_EVT_SUBCODE            = 0x8E,
    HCI_VS_CLOCK_WRAP_EVT_SUBCODE                  = 0x8F,
    HCI_VS_SNIFF_REQUEST_EVT_SUBCODE               = 0x90,
    HCI_VS_SNIFF_MODE_CHANGE_EVT_SUBCODE           = 0x92,
    HCI_ECI_TX_PENDING_EVT_SUBCODE                 = 0x93,
    HCI_ECI_EAVESDROPPING_STATE_CHANGED_EVT_SUBCODE = 0x94,
    HCI_ECI_CORR_ACL_PAYL_EVT_SUBCODE              = 0x95, // Deprecated
    HCI_ECI_RX_STATS_EVT_SUBCODE                   = 0x96,
    HCI_VS_CUST_TX_TEST_END_EVT_SUBCODE            = 0x97,
    HCI_ECI_ESCO_TX_SEQN_SAMPLED_EVT_SUBCODE       = 0x98,
    HCI_ECI_ESCO_TX_SEQN_SYNCED_EVT_SUBCODE        = 0x99,
#endif // EAVESDROPPING_SUPPORT
};

/*@TRACE*/
enum hci_le_evt_subcode
{
    /// LE Events Subcodes
    HCI_LE_CON_CMP_EVT_SUBCODE                 = 0x01,
    HCI_LE_ADV_REPORT_EVT_SUBCODE              = 0x02,
    HCI_LE_CON_UPDATE_CMP_EVT_SUBCODE          = 0x03,
    HCI_LE_RD_REM_FEATS_CMP_EVT_SUBCODE        = 0x04,
    HCI_LE_LTK_REQUEST_EVT_SUBCODE             = 0x05,
    HCI_LE_REM_CON_PARAM_REQ_EVT_SUBCODE       = 0x06,
    HCI_LE_DATA_LEN_CHG_EVT_SUBCODE            = 0x07,
    HCI_LE_RD_LOC_P256_PUB_KEY_CMP_EVT_SUBCODE = 0x08,
    HCI_LE_GEN_DHKEY_CMP_EVT_SUBCODE           = 0x09,
    HCI_LE_ENH_CON_CMP_EVT_SUBCODE             = 0x0A,
    HCI_LE_DIR_ADV_REP_EVT_SUBCODE             = 0x0B,
    HCI_LE_PHY_UPD_CMP_EVT_SUBCODE             = 0x0C,
    HCI_LE_EXT_ADV_REPORT_EVT_SUBCODE          = 0x0D,
    HCI_LE_PER_ADV_SYNC_EST_EVT_SUBCODE        = 0x0E,
    HCI_LE_PER_ADV_REPORT_EVT_SUBCODE          = 0x0F,
    HCI_LE_PER_ADV_SYNC_LOST_EVT_SUBCODE       = 0x10,
    HCI_LE_SCAN_TIMEOUT_EVT_SUBCODE            = 0x11,
    HCI_LE_ADV_SET_TERMINATED_EVT_SUBCODE      = 0x12,
    HCI_LE_SCAN_REQ_RCVD_EVT_SUBCODE           = 0x13,
    HCI_LE_CH_SEL_ALGO_EVT_SUBCODE             = 0x14,
    HCI_LE_CONLESS_IQ_REPORT_EVT_SUBCODE       = 0x15,
    HCI_LE_CON_IQ_REPORT_EVT_SUBCODE           = 0x16,
    HCI_LE_CTE_REQ_FAILED_EVT_SUBCODE          = 0x17,
#if (BLE_PAST)
    HCI_LE_PER_ADV_SYNC_TRANSF_REC_EVT_SUBCODE = 0x18,
#endif // (BLE_PAST)
#if (BLE_CIS)
    HCI_LE_CIS_ESTABLISHED_EVT_SUBCODE         = 0x19,
    HCI_LE_CIS_REQUEST_EVT_SUBCODE             = 0x1A,
#endif // (BLE_CIS)
#if (BLE_BIS)
    HCI_LE_CREATE_BIG_CMP_EVT_SUBCODE          = 0x1B,
    HCI_LE_TERMINATE_BIG_CMP_EVT_SUBCODE       = 0x1C,
    HCI_LE_BIG_SYNC_ESTABLISHED_EVT_SUBCODE    = 0x1D,
    HCI_LE_BIG_SYNC_LOST_EVT_SUBCODE           = 0x1E,
#endif // (BLE_BIS)
    HCI_LE_REQ_PEER_SCA_CMP_EVT_SUBCODE        = 0x1F,
    HCI_LE_PATH_LOSS_THRESHOLD_EVT_SUBCODE     = 0x20,
    HCI_LE_TX_POWER_REPORTING_EVT_SUBCODE      = 0x21,
#if (BLE_BIS)
    HCI_LE_BIG_INFO_ADV_REPORT_EVT_SUBCODE     = 0x22,
#endif // (BLE_BIS)
#if (BT_53)
    HCI_LE_SUBRATE_CHG_EVT_SUBCODE             = 0x23,
#endif // (BT_53)
};

/*@TRACE*/
enum hci_vs_evt_subcode
{
#if (RW_DEBUG)
    /// DBG Events Subcodes
    HCI_DBG_ASSERT_EVT_SUBCODE              = 0x02,
    HCI_DBG_LE_CON_EVT_TRACE_EVT_SUBCODE    = 0x03,
#endif //(RW_DEBUG)

#if (AUDIO_SYNC_SUPPORT && BLE_CIS)
    HCI_VS_AUSY_CIS_ESTAB_PARAM_EVT_SUBCODE = 0x04,
#endif // (AUDIO_SYNC_SUPPORT && BLE_CIS)
#if (AUDIO_SYNC_SUPPORT && BLE_BIS)
    HCI_VS_AUSY_BIS_ESTAB_PARAM_EVT_SUBCODE = 0x05,
#endif // (AUDIO_SYNC_SUPPORT)

#if (RW_DEBUG)
    HCI_DBG_BT_RSSI_NOTIFY_EVT_SUBCODE        = 0x30,
#endif //(RW_DEBUG)

    HCI_VS_INVALID_EVT_SUBCODE              = 0xFF,
};

/// Event mask page enum
enum hci_evt_mask_page
{
    /// page 0
    HCI_PAGE_0,
    /// page 1
    HCI_PAGE_1,
    /// page 2
    HCI_PAGE_2,
    /// Default
    HCI_PAGE_DFT,
    /// LE event
    HCI_PAGE_LE,
};

#if (BLE_ISO_PRESENT)
#if (BLE_ISO_MODE_0)
/// Current audio mode
enum iso_am0_ctrl
{
    // Stop Audio Mode 0 Stream
    ISO_AM0_CRL_DISABLE,
    // Start Audio Mode 0 Stream
    ISO_AM0_CRL_ENABLE,
};
#endif // (BLE_ISO_MODE_0)

/// Isochronous Channel data path selection
enum iso_dp_type
{
    /// ISO over HCI Data Path
    ISO_DP_ISOOHCI                  = 0x00,
    /// Data Path direction is disabled
    ISO_DP_DISABLE                  = 0xFF,

    // vendor specifics
    // @see enum dp_type
    // @see enum plf_dp_type
};

/// Isochronous Data Path Direction
enum iso_dp_direction
{
    /// Input (Host to Controller)
    ISO_DP_INPUT                     = 0x00,
    /// Output (Controller to Host)
    ISO_DP_OUTPUT                    = 0x01,
};

#endif // (BLE_ISO_PRESENT)



/**************************************************************************************
 **************                 HCI MESSAGE STRUCTURES                 ****************
 **************************************************************************************/

/// HCI ACL data packet structure
/*@TRACE*/
struct hci_acl_data
{
    /// bits[00:11]: Connection handle
    /// bits[12:13]: Packet boundary flag
    /// bits[14:15]: Broadcast flag
    uint16_t  conhdl_pb_bc_flag;
    /// length of the data
    uint16_t  length;
    /// Memory Pointer address
    uint32_t  buf_ptr;
};

/// HCI Synchronous data packet structure
/*@TRACE*/
struct hci_sync_data
{
    /// bits[00:11]: Connection handle
    /// bits[12:13]: Packet status flag
    uint16_t  conhdl_psf;
    /// length of the data
    uint8_t  length;
    /// EM buffer pointer
    uint16_t  buf_ptr;
};

struct hci_packed_data
{
    uint16_t  length;
    uint8_t  reserved;
    uint8_t  type;
};

#if BLE_EMB_PRESENT
#if BLE_ISO_PRESENT
/// ISO_Data_load structure
/*@TRACE*/
struct iso_data_load
{
    /// Time_Stamp
    uint32_t time_stamp;
    /// Packet Sequence Number
    uint16_t  pkt_seq_nb;
    /// length of the ISO SDU
    /// bits[00:11]: ISO SDU length
    /// bits[12:13]: RFU
    /// bits[14:15]: Packet_Status_Flag
    uint16_t  iso_sdu_length_psf;
    /// Pointer to the SDU
    uint8_t *iso_sdu_ptr;
};

/// HCI ISO data packet structure
/*@TRACE*/
struct hci_iso_data
{
    /// bits[00:11]: Connection handle
    /// bits[12:13]: Packet boundary flag
    /// bit[14]: Time_Stamp flag
    /// bit[15]: RFU
    uint16_t  conhdl_pbf_tsf;
    /// ISO_Data_Load_Length and 2 RFU bits
    uint16_t  iso_data_load_len;
    /// ISO_Data_Load
    struct  iso_data_load iso_data_load;
};
#endif //BLE_ISO_PRESENT
#endif //BLE_EMB_PRESENT
/*
 * HCI COMMANDS PARAMETERS (to classify)
 ****************************************************************************************
 */

/// HCI basic command structure with connection handle
/*@TRACE
 * hci_rd_rssi_cmd = hci_basic_conhdl_cmd
 * hci_le_rd_chnl_map_cmd = hci_basic_conhdl_cmd
 * hci_le_ltk_req_neg_reply_cmd = hci_basic_conhdl_cmd*/
struct hci_basic_conhdl_cmd
{
    /// connection handle
    uint16_t    conhdl;
};

/// HCI basic command structure with BD address
struct hci_basic_bd_addr_cmd
{
    ///BdAddr
    struct bd_addr  bd_addr;
};

/// HCI Accept connection request command structure
/*@TRACE*/
struct hci_accept_con_req_cmd
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///Page Scan Repetition Mode
    uint8_t         role;
};

/// HCI Accept synchronous connection request command structure
/*@TRACE*/
struct hci_accept_sync_con_req_cmd
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///Transmit bandwidth
    uint32_t    tx_bw;
    ///Receive bandwidth
    uint32_t    rx_bw;
    ///Max latency
    uint16_t    max_lat;
    ///Voice settings
    uint16_t    vx_set;
    ///Retransmission effort
    uint8_t      retx_eff;
    ///Packet type
    uint16_t     pkt_type  ;
};

/// HCI Enhanced Accept synchronous connection request command structure
/*@TRACE*/
struct hci_enh_accept_sync_con_cmd
{

    struct bd_addr    bd_addr;            // BD address
    uint32_t          tx_bw;              // Transmit Bandwidth (in B/sec)
    uint32_t          rx_bw;              // Receive Bandwidth (in B/sec)
    uint8_t           tx_cod_fmt[5];      // Transmit Coding Format
    uint8_t           rx_cod_fmt[5];      // Receive Coding Format
    uint16_t          tx_cod_fr_sz;       // Transmit Codec Frame Size (in B)
    uint16_t          rx_cod_fr_sz;       // Receive Codec Frame Size (in B)
    uint32_t          in_bw;              // Input Bandwidth (in B/sec)
    uint32_t          out_bw;             // Output Bandwidth (in B/sec)
    uint8_t           in_cod_fmt[5];      // Input Coding Format
    uint8_t           out_cod_fmt[5];     // Output Coding Format
    uint16_t          in_cod_data_sz;     // Input Coded Data Size (in bits)
    uint16_t          out_cod_data_sz;    // Output Coded Data Size (in bits)
    uint8_t           in_data_fmt;        // Input PCM Data Format
    uint8_t           out_data_fmt;       // Output PCM Data Format
    uint8_t           in_msb_pos;         // Input PCM Sample Payload MSB Position (in bits)
    uint8_t           out_msb_pos;        // Output PCM Sample Payload MSB Position (in bits)
    uint8_t           in_data_path;       // Input Data Path
    uint8_t           out_data_path;      // Output Data Path
    uint8_t           in_tr_unit_sz;      // Input Transport Unit Size (in bits)
    uint8_t           out_tr_unit_sz;     // Output Transport Unit Size (in bits)
    uint16_t          max_lat;            // Max Latency (in ms)
    uint16_t          packet_type;        // Packet Type
    uint8_t           retx_eff;           // Retransmission Effort


};

/// HCI reject connection request command structure
/*@TRACE*/
struct hci_reject_con_req_cmd
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///Reason
    uint8_t         reason;
};

/// HCI reject synchronous connection request command structure
/*@TRACE*/
struct hci_reject_sync_con_req_cmd
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///Reason
    uint8_t         reason;
};

/// HCI link key request reply command structure
/*@TRACE*/
struct hci_lk_req_reply_cmd
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///Key
    struct ltk         key;
};

/// HCI link key request reply command structure
/*@TRACE*/
struct hci_pin_code_req_reply_cmd
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///Pin code length
    uint8_t     pin_len;
    ///Key
    struct pin_code pin;
};

/// HCI switch role command structure
/*@TRACE*/
struct hci_switch_role_cmd
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///Read all flag
    uint8_t role;
};

/// HCI flow specification command parameters structure
/*@TRACE*/
struct hci_flow_spec_cmd
{
    ///Connection handle
    uint16_t conhdl;
    ///Flags
    uint8_t flags;
    ///Flow direction
    uint8_t flow_dir;
    ///Service type
    uint8_t serv_type;
    ///Token rate
    uint32_t tk_rate;
    ///Token buffer size
    uint32_t tk_buf_sz;
    ///Peak bandwidth
    uint32_t pk_bw;
    ///Access latency
    uint32_t acc_lat;
};

/// HCI enhanced flush command parameters structure
/*@TRACE*/
struct hci_enh_flush_cmd
{
    ///Connection handle
    uint16_t conhdl;
    ///Packet Type
    uint8_t pkt_type;
};

/// HCI command complete event structure for the read auto flush TO command
struct hci_rd_auto_flush_to_cmd_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///Flush timeout
    uint16_t flush_to;
};

/// HCI write flush timeout command parameters structure
/*@TRACE*/
struct hci_wr_auto_flush_to_cmd
{
    ///Connection handle
    uint16_t conhdl;
    ///Flush timeout
    uint16_t flush_to;
};

/// HCI change connection packet type command parameters structure
/*@TRACE*/
struct hci_chg_con_pkt_type_cmd
{
    ///Connection handle
    uint16_t conhdl;
    ///Packet type
    uint16_t pkt_type;
};

/// HCI read link policy settings command parameters structure
/*@TRACE*/
struct hci_rd_link_pol_stg_cmd_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///Link policy
    uint16_t lnk_policy;
};

/// HCI read link policy settings command parameters structure
/*@TRACE*/
struct hci_wr_link_pol_stg_cmd
{
    ///Connection handle
    uint16_t    conhdl;
    ///Link policy
    uint16_t lnk_policy;
};

/// HCI sniff mode request command parameters structure
/*@TRACE*/
struct hci_sniff_mode_cmd
{
    ///Connection handle
    uint16_t    conhdl;
    /// Maximum interval (in slots)
    uint16_t    max_int;
    /// Minimum interval (in slots)
    uint16_t    min_int;
    /// Attempts (number of receive slots) (in slots)
    uint16_t    attempt;
    /// Timeout (number of receive slots) (in slots)
    uint16_t    timeout;
};

/// HCI sniff subrating mode request command parameters structure
/*@TRACE*/
struct hci_sniff_sub_cmd
{
    ///Connection handle
    uint16_t    conhdl;
    /// Maximum latency used to calculate the maximum sniff subrate that the remote device may use (in slots)
    uint16_t    max_lat;
    /// Minimum base sniff subrate timeout that the remote device may use (in slots)
    uint16_t    min_rem_to;
    /// Minimum base sniff subrate timeout that the local device may use (in slots)
    uint16_t    min_loc_to;
};

/// HCI role discovery complete event parameters structure
/*@TRACE*/
struct hci_role_discovery_cmd_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///Role
    uint8_t role;

};

/// HCI read failed contact counter command parameters structure
/*@TRACE*/
struct hci_rd_fail_contact_cnt_cmd_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///Fail contact counter
    uint16_t fail_cnt;
};

/// HCI read link quality complete event parameters structure
/*@TRACE*/
struct hci_rd_link_qual_cmd_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///Quality type
    uint8_t quality;
};

/// HCI read afh channel map complete event parameters structure
/*@TRACE*/
struct hci_rd_afh_ch_map_cmd_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    /// AFH mode
    uint8_t afh_mode;
    /// AFH channel map
    struct bt_ch_map afh_map;
};

/// HCI read lmp handle complete event parameters structure
/*@TRACE*/
struct hci_rd_lmp_hdl_cmd_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///lmp handle
    uint8_t lmp_hdl;
    ///rsvd
    uint32_t rsvd;
};

/// HCI read remote extended features command parameters structure
/*@TRACE*/
struct hci_rd_rem_ext_feats_cmd
{
    ///Connection handle
    uint16_t conhdl;
    ///page number
    uint8_t pg_nb;
};

/// HCI read encryption key size complete event parameters structure
/*@TRACE*/
struct hci_rd_enc_key_size_cmd_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///Key size
    uint8_t key_sz;
};

/// HCI read enhanced transmit power command parameters structure
/*@TRACE*/
struct hci_rd_enh_tx_pwr_lvl_cmd
{
    ///Connection handle
    uint16_t conhdl;
    ///Type
    uint8_t type;
};

/// HCI read enhanced transmit power complete event parameters structure
/*@TRACE*/
struct hci_rd_enh_tx_pwr_lvl_cmd_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///Transmit power GFSK
    int8_t pw_gfsk;
    ///Transmit power DQPSK
    int8_t pw_dqpsk;
    ///Transmit power 8DPSK
    int8_t pw_8dpsk;
};


/*
 * HCI LINK CONTROL COMMANDS PARAMETERS
 ****************************************************************************************
 */

/// Format of the message of the Group: LINK_CONTROL_COMMANDS
/// HCI Inquiry command parameters structure
/*@TRACE*/
struct hci_inq_cmd
{
    ///Lap
    struct lap  lap;
    ///Inquiry Length in units of 1.28 s
    uint8_t     inq_len;
    ///Number of response
    uint8_t     nb_rsp;
};
/*@TRACE*/
struct hci_per_inq_mode_cmd
{
    ///Maximum period length
    uint16_t max_per_len;
    ///Minimum period length
    uint16_t min_per_len;
    ///lap
    struct lap lap;
    ///Inquiry length in units of 1.28 s
    uint8_t inq_len;
    ///Number of response
    uint8_t nb_rsp;
};
/*@TRACE*/
struct hci_create_con_cmd
{
    /// BdAddr
    struct bd_addr  bd_addr;
    /// Packet Type
    uint16_t        pkt_type;
    /// Page Scan Repetition Mode
    uint8_t         page_scan_rep_mode;
    /// Reserved
    uint8_t         rsvd;
    /**
     * Clock Offset
     *
     * Bits 14-0 : Bits 16-2 of CLKNslave-CLK
     * Bit 15 : Clock_Offset_Valid_Flag
     *   Invalid Clock Offset = 0
     *   Valid Clock Offset = 1
     */
    uint16_t        clk_off;
    /// Allow Switch
    uint8_t         switch_en;
};

/// HCI disconnect command structure
/*@TRACE*/
struct hci_disconnect_cmd
{
    /// connection handle
    uint16_t    conhdl;
    /// reason
    uint8_t     reason;
};

/// HCI master link key command structure
/*@TRACE*/
struct hci_master_lk_cmd
{
    ///Key flag
    uint8_t key_flag;
};

/// HCI authentication request command parameters structure
/*@TRACE*/
struct hci_set_con_enc_cmd
{
    ///Connection handle
    uint16_t conhdl;
    ///Encryption mode
    uint8_t enc_en;
};

/*@TRACE*/
struct hci_rem_name_req_cmd
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///Page Scan Repetition Mode
    uint8_t         page_scan_rep_mode;
    ///Reserved
    uint8_t         rsvd;
    /**
     * Clock Offset
     *
     * Bits 14-0 : Bits 16-2 of CLKNslave-CLK
     * Bit 15 : Clock_Offset_Valid_Flag
     *   Invalid Clock Offset = 0
     *   Valid Clock Offset = 1
     */
    uint16_t        clk_off;
};

/// HCI remote name request complete event structure
/*@TRACE*/
struct hci_rem_name_req_cmp_evt
{
    /// Status
    uint8_t status;
    /// BD Addr
    struct bd_addr bd_addr;
    /// Name
    struct device_name name;
};

/// HCI setup synchronous connection command structure
/*@TRACE*/
struct hci_setup_sync_con_cmd
{
    ///Connection handle
    uint16_t conhdl;
    ///Transmit bandwidth
    uint32_t tx_bw;
    ///Receive bandwidth
    uint32_t rx_bw;
    ///Max latency
    uint16_t max_lat;
    ///Voice setting
    uint16_t vx_set;
    ///Retransmission effort
    uint8_t retx_eff;
    ///Packet type
    uint16_t pkt_type;
};

/// HCI setup synchronous connection command structure
/*@TRACE*/
struct hci_enh_setup_sync_con_cmd
{
    uint16_t      conhdl;               // Connection Handle
    uint32_t      tx_bw;                // Transmit Bandwidth (in B/sec)
    uint32_t      rx_bw;                // Receive Bandwidth (in B/sec)
    uint8_t       tx_cod_fmt[5];        // Transmit Coding Format
    uint8_t       rx_cod_fmt[5];        // Receive Coding Format
    uint16_t      tx_cod_fr_sz;         // Transmit Codec Frame Size (in B)
    uint16_t      rx_cod_fr_sz;         // Receive Codec Frame Size (in B)
    uint32_t      in_bw;                // Input Bandwidth (in B/sec)
    uint32_t      out_bw;               // Output Bandwidth (in B/sec)
    uint8_t       in_cod_fmt[5];        // Input Coding Format
    uint8_t       out_cod_fmt[5];       // Output Coding Format
    uint16_t      in_cod_data_sz;       // Input Coded Data Size (in bits)
    uint16_t      out_cod_data_sz;      // Output Coded Data Size (in bits)
    uint8_t       in_data_fmt;          // Input PCM Data Format
    uint8_t       out_data_fmt;         // Output PCM Data Format
    uint8_t       in_msb_pos;           // Input PCM Sample Payload MSB Position (in bits)
    uint8_t       out_msb_pos;          // Output PCM Sample Payload MSB Position (in bits)
    uint8_t       in_data_path;         // Input Data Path
    uint8_t       out_data_path;        // Output Data Path
    uint8_t       in_tr_unit_sz;        // Input Transport Unit Size (in bits)
    uint8_t       out_tr_unit_sz;       // Output Transport Unit Size (in bits)
    uint16_t      max_lat;              // Max Latency (in ms)
    uint16_t      packet_type;          // Packet Type
    uint8_t       retx_eff;             // Retransmission Effort
};

/// HCI io capability request reply command structure
/*@TRACE*/
struct hci_io_cap_req_reply_cmd
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///IO capability
    uint8_t io_capa;
    ///OOB data present
    uint8_t oob_data_pres;
    ///Authentication requirements
    uint8_t auth_req;

};

/// HCI io capability request negative reply command structure
/*@TRACE*/
struct hci_io_cap_req_neg_reply_cmd
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///Reason
    uint8_t reason;
};

/// HCI user pass key request reply command structure
/*@TRACE*/
struct hci_user_passkey_req_reply_cmd
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///Numeric value
    uint32_t num_val;
};

/// HCI remote oob data request reply command structure
/*@TRACE*/
struct hci_rem_oob_data_req_reply_cmd
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///hash part
    struct hash oob_c;
    ///random part
    struct randomizer oob_r;
};

/// HCI send key press notification command structure
/*@TRACE*/
struct hci_send_keypress_notif_cmd
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///Notification type
    uint8_t notif_type;
};

/// HCI truncated page command structure
/*@TRACE*/
struct hci_trunc_page_cmd
{
    ///BdAddr
    struct bd_addr  bd_addr;
    /// Page Scan Repetition Mode
    uint8_t         page_scan_rep_mode;
    /**
     * Clock Offset
     *
     * Bits 14-0 : Bits 16-2 of CLKNslave-CLK
     * Bit 15 : Clock_Offset_Valid_Flag
     *   Invalid Clock Offset = 0
     *   Valid Clock Offset = 1
     */
    uint16_t        clk_off;
};

/// HCI truncated page cancel command structure
/*@TRACE*/
struct hci_trunc_page_can_cmd
{
    ///BdAddr
    struct bd_addr  bd_addr;
};

/// HCI set connectionless slave broadcast command structure
/*@TRACE*/
struct hci_set_con_slv_bcst_cmd
{
    /// Enable
    uint8_t enable;
    /// LT_ADDR
    uint8_t lt_addr;
    /// LPO_Allowed
    uint8_t lpo_allowed;
    /// Packet_Type
    uint16_t packet_type;
    /// Interval_Min (in slots)
    uint16_t interval_min;
    /// Interval_Max (in slots)
    uint16_t interval_max;
    /// CSB_supervisionTO (in slots)
    uint16_t csb_supv_to;
};

/// HCI set connectionless slave broadcast command complete event structure
/*@TRACE*/
struct hci_set_con_slv_bcst_cmd_cmp_evt
{
    /// Status
    uint8_t status;
    /// LT_ADDR
    uint8_t lt_addr;
    /// Interval (in slots)
    uint16_t interval;
};

/// HCI set connectionless slave broadcast receive command structure
/*@TRACE*/
struct hci_set_con_slv_bcst_rec_cmd
{
    /// Enable
    uint8_t enable;
    /// BD_ADDR
    struct bd_addr  bd_addr;
    /// LT_ADDR
    uint8_t lt_addr;
    /// Interval (in slots)
    uint16_t interval;
    /// Clock_Offset (28 bits) - (CLKNslave - CLK) modulo 2^28
    uint32_t clock_offset;
    /// Next_Connectionless_Slave_Broadcast_Clock (28 bits)
    uint32_t next_csb_clock;
    /// CSB_supervisionTO (in slots)
    uint16_t csb_supv_to;
    /// Remote_Timing_Accuracy (in ppm)
    uint8_t remote_timing_accuracy;
    /// Skip
    uint8_t skip;
    /// Packet_Type
    uint16_t packet_type;
    /// AFH_Channel_Map
    struct bt_ch_map afh_ch_map;
};

/// HCI set connectionless slave broadcast receive command complete event structure
/*@TRACE*/
struct hci_set_con_slv_bcst_rec_cmd_cmp_evt
{
    /// Status
    uint8_t status;
    /// BD_ADDR
    struct bd_addr  bd_addr;
    /// LT_ADDR
    uint8_t lt_addr;
};

/// HCI Receive Synchronization Train command structure
/*@TRACE*/
struct hci_rec_sync_train_cmd
{
    /// BD_ADDR
    struct bd_addr  bd_addr;
    /// Synchronization_scanTO (in slots)
    uint16_t sync_scan_to;
    /// Sync_Scan_Window (in slots)
    uint16_t sync_scan_win;
    /// Sync_Scan_Interval (in slots)
    uint16_t sync_scan_int;
};

/// HCI remote oob extended data request reply command structure
/*@TRACE*/
struct hci_rem_oob_ext_data_req_reply_cmd
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///hash part
    struct hash oob_c_192;
    ///random part
    struct randomizer oob_r_192;
    ///hash part
    struct hash oob_c_256;
    ///random part
    struct randomizer oob_r_256;
};

/*@TRACE*/
struct hci_le_gen_dhkey_v1_cmd
{
    /// Remote P-256 public key
    uint8_t public_key[64];
};

/*@TRACE*/
struct hci_le_gen_dhkey_v2_cmd
{
    /// Remote P-256 public key
    uint8_t public_key[64];

    /// Private key type (@see enum priv_key_type)
    uint8_t key_type;
};

/*@TRACE*/
struct hci_le_mod_sleep_clk_acc_cmd
{
    /// Switch to more or less accurate clock (@see enum clk_acc_action)
    uint8_t action;
};


/*
 * HCI LINK POLICY COMMANDS PARAMETERS
 ****************************************************************************************
 */

/// HCI setup quality of service command structure
/*@TRACE*/
struct hci_qos_setup_cmd
{
    ///Connection handle
    uint16_t conhdl;
    ///Flags
    uint8_t flags;
    ///Service type
    uint8_t serv_type;
    ///Token rate
    uint32_t tok_rate;
    ///Peak bandwidth
    uint32_t pk_bw;
    ///Latency
    uint32_t lat;
    ///Delay variation
    uint32_t del_var;
};

/// HCI command complete event structure for read default link policy command structure
/*@TRACE*/
struct hci_rd_dft_link_pol_stg_cmd_cmp_evt
{
    ///Status of the command reception
    uint8_t     status;
    ///Link policy
    uint16_t    link_pol_stg;
};

/*@TRACE*/
struct hci_wr_dft_link_pol_stg_cmd
{
    ///Link policy
    uint16_t    link_pol_stg;
};

/*
 * HCI CONTROL & BASEBAND COMMANDS PARAMETERS
 ****************************************************************************************
 */

/// HCI set event mask command structure
/*@TRACE*/
struct hci_set_evt_mask_cmd
{
    ///Event Mask
    struct evt_mask    event_mask;
};

/// HCI set event filter command structure
/*@TRACE*/
struct hci_set_evt_filter_cmd
{
    /// Filter type
    uint8_t filter_type;

    /// Filters
    union hci_filter
    {
        uint8_t clear_all_filter_reserved;

        /// Inquiry Result Filter
        struct inq_res_filter
        {
            /// Filter Condition type
            uint8_t cond_type;

            /// Filter conditions
            union hci_inq_filter_cond
            {
                /// Reserved value (Inquiry Result Filter - condition type 0x00 has no condition)
                uint8_t cond_0_reserved;

                /// Inquiry Result Filter Condition - condition type 0x01
                struct inq_res_filter_cond_1
                {
                    /// Class_of_Device
                    struct devclass class_of_dev;
                    /// Class_of_Device_Mask
                    struct devclass class_of_dev_msk;
                } cond_1;

                /// Inquiry Result Filter Condition - condition type 0x02
                struct inq_res_filter_cond_2
                {
                    /// BD Address
                    struct bd_addr bd_addr;
                } cond_2;
            } cond;
        } inq_res;

        /// Connection Setup Filter
        struct con_set_filter
        {
            /// Filter Condition type
            uint8_t cond_type;

            /// Filter conditions
            union hci_con_filter_cond
            {
                /// Connection Setup Filter Condition - condition type 0x00
                struct con_set_filter_cond_0
                {
                    /// Auto_Accept_Flag
                    uint8_t auto_accept;
                } cond_0;

                /// Connection Setup Filter Condition - condition type 0x01
                struct con_set_filter_cond_1
                {
                    /// Class_of_Device
                    struct devclass class_of_dev;
                    /// Class_of_Device_Mask
                    struct devclass class_of_dev_msk;
                    /// Auto_Accept_Flag
                    uint8_t auto_accept;
                } cond_1;

                /// Connection Setup Filter Condition - condition type 0x02
                struct con_set_filter_cond_2
                {
                    /// BD Address
                    struct bd_addr bd_addr;
                    /// Auto_Accept_Flag
                    uint8_t auto_accept;
                } cond_2;
            } cond;

        } con_set;

    } filter;
};

/// HCI command completed event structure for the flush command
/*@TRACE*/
struct hci_flush_cmd_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
};

/// HCI command complete event structure for the Read pin type command
/*@TRACE*/
struct hci_rd_pin_type_cmd_cmp_evt
{
    ///Status of the command
    uint8_t     status;
    ///PIN type
    uint8_t   pin_type;
};

/*@TRACE*/
struct hci_wr_pin_type_cmd
{
    ///PIN type
    uint8_t pin_type;
};

/*@TRACE*/
struct hci_rd_stored_lk_cmd
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///Read all flag
    uint8_t rd_all_flag;
};

/// HCI command complete event structure for read stored link key command
/*@TRACE*/
struct hci_rd_stored_lk_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t     status;
    ///Maximum number of key
    uint16_t num_key_max;
    ///Read number of key
    uint16_t num_key_rd;
};

#if BT_EMB_PRESENT
/*@TRACE*/
struct hci_wr_stored_lk_cmd
{
    /// Number of key to write
    uint8_t num_key_wr;

    /// BD Address + Key table
    struct bd_addr_plus_key link_keys[HCI_MAX_CMD_PARAM_SIZE / sizeof(struct bd_addr_plus_key)];
};
#endif //BT_EMB_PRESENT

/// HCI command complete event structure for write stored link key command
/*@TRACE*/
struct hci_wr_stored_lk_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t     status;
    ///number of key written
    uint8_t    num_key_wr;
};

/*@TRACE*/
struct hci_del_stored_lk_cmd
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///Delete all flag
    uint8_t del_all_flag;
};

/// HCI command complete event structure for delete stored link key command
/*@TRACE*/
struct hci_del_stored_lk_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t     status;
    ///Read number of key
    uint16_t num_key_del;
};

/*@TRACE*/
struct hci_wr_local_name_cmd
{
    ///Name
    struct device_name  name;
};

/// HCI command complete event structure for the read local name command
/*@TRACE*/
struct hci_rd_local_name_cmd_cmp_evt
{
    ///Status of the command
    uint8_t     status;
    ///Name
    uint8_t  name[BD_NAME_SIZE];
};

/// HCI command complete event structure for the Read connection accept to command
/*@TRACE*/
struct hci_rd_con_accept_to_cmd_cmp_evt
{
    ///Status of the command
    uint8_t     status;
    /// Connection accept timeout (in slots)
    uint16_t    con_acc_to;
};

/*@TRACE*/
struct hci_wr_con_accept_to_cmd
{
    /// Connection accept timeout (in slots)
    uint16_t    con_acc_to;
};

/// HCI command complete event structure for the Read page to command
/*@TRACE*/
struct hci_rd_page_to_cmd_cmp_evt
{
    ///Status of the command
    uint8_t     status;
    /// Page timeout (in slots)
    uint16_t    page_to;
};

/*@TRACE*/
struct hci_wr_page_to_cmd
{
    /// Page timeout (in slots)
    uint16_t    page_to;
};

/// HCI command complete event structure for the Read scan enable command
/*@TRACE*/
struct hci_rd_scan_en_cmd_cmp_evt
{
    ///Status of the command
    uint8_t     status;
    ///Status of the scan enable
    uint8_t     scan_en;
};

/*@TRACE*/
struct hci_wr_scan_en_cmd
{
    ///Status of the scan enable
    uint8_t scan_en;
};

/// HCI command complete event structure for the Read scan activity command
/*@TRACE*/
struct hci_rd_page_scan_act_cmd_cmp_evt
{
    ///Status of the command
    uint8_t     status;
    /// Page scan interval (in slots)
    uint16_t page_scan_intv;
    /// Page scan window (in slots)
    uint16_t page_scan_win;
};

/*@TRACE*/
struct hci_wr_page_scan_act_cmd
{
    /// Page scan interval (in slots)
    uint16_t page_scan_intv;
    /// Page scan window (in slots)
    uint16_t page_scan_win;
};

/// HCI command complete event structure for the Read inquiry scan activity command
/*@TRACE*/
struct hci_rd_inq_scan_act_cmd_cmp_evt
{
    /// Status of the command
    uint8_t  status;
    /// Inquiry scan interval (in slots)
    uint16_t inq_scan_intv;
    /// Inquiry scan window (in slots)
    uint16_t inq_scan_win;
};

/*@TRACE*/
struct hci_wr_inq_scan_act_cmd
{
    /// Inquiry scan interval (in slots)
    uint16_t inq_scan_intv;
    /// Inquiry scan window (in slots)
    uint16_t inq_scan_win;
};

/// HCI command complete event structure for the Read authentication command
/*@TRACE*/
struct hci_rd_auth_en_cmd_cmp_evt
{
    ///Status of the command
    uint8_t     status;
    ///Value of the authentication
    uint8_t    auth_en;
};

/*@TRACE*/
struct hci_wr_auth_en_cmd
{
    ///Value of the authentication
    uint8_t auth_en;
};

/// HCI command complete event structure for the read class of device command
/*@TRACE*/
struct hci_rd_class_of_dev_cmd_cmp_evt
{
    ///Status of the command
    uint8_t     status;
    ///Class of device
    struct devclass class_of_dev;
};

/*@TRACE*/
struct hci_wr_class_of_dev_cmd
{
    ///Class of device
    struct devclass class_of_dev;
};

/// HCI read voice settings complete event
/*@TRACE*/
struct hci_rd_voice_stg_cmd_cmp_evt
{
    ///Status of the command reception
    uint8_t     status;
    /// Voice setting
    uint16_t voice_stg;
};

/*@TRACE*/
struct hci_wr_voice_stg_cmd
{
    /// voice setting
    uint16_t voice_stg;
};

/// HCI command complete event structure for read number of broadcast retrans command
/*@TRACE*/
struct hci_rd_nb_bdcst_retx_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t     status;
    ///Read number of broadcast retransmission
    uint8_t num_bcst_ret;
};

/*@TRACE*/
struct hci_wr_nb_bdcst_retx_cmd
{
    ///Read number of broadcast retransmission
    uint8_t num_bcst_ret;
};

/// HCI command complete event structure for the Read Synchronous Flow Control command
/*@TRACE*/
struct hci_rd_sync_flow_ctrl_en_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t     status;
    ///Synchronous flow control enable
    uint8_t     sync_flow_ctrl_en;
};

/*@TRACE*/
struct hci_wr_sync_flow_ctrl_en_cmd
{
    /// Synchronous Flow Control enable
    uint8_t   sync_flow_ctrl_en;
};

///HCI set controller to host flow control command
/*@TRACE*/
struct hci_set_ctrl_to_host_flow_ctrl_cmd
{
    ///Flow control enable for controller
    uint8_t flow_cntl;
};

///HCI host buffer size command
/*@TRACE*/
struct hci_host_buf_size_cmd
{
    ///Host ACL packet length
    uint16_t    acl_pkt_len;
    ///Host synchronous packet length
    uint8_t     sync_pkt_len;
    ///Host Total number of ACL packets allowed
    uint16_t    nb_acl_pkts;
    ///Host total number of synchronous packets allowed
    uint16_t    nb_sync_pkts;
};

/// Number of completed packet for a connection
/*@TRACE*/
struct hci_nb_cmp_pkt
{
    /// connection handles
    uint16_t    hdl;
    /// Number of completed packets values for connection handles.
    uint16_t    nb_comp_pkt;
};

///HCI host number of completed packets command
/*@TRACE*/
struct hci_host_nb_cmp_pkts_cmd
{
    ///Number of handles for which the completed packets number is given
    uint8_t     nb_of_hdl;
    /// Array of number of completed packets per connection
    struct hci_nb_cmp_pkt con[MAX_NB_CMP_PKTS_CON];
};

/// HCI read link supervision timeout command parameters structure
struct hci_rd_link_supv_to_cmd_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///Link supervision timeout
    uint16_t lsto_val;
};

/// HCI write link supervision timeout command parameters structure
/*@TRACE*/
struct hci_wr_link_supv_to_cmd
{
    ///Connection handle
    uint16_t conhdl;
    ///Link supervision timeout
    uint16_t lsto_val;
};

/// HCI command complete event structure for the nb of supported IAC command
/*@TRACE*/
struct hci_rd_nb_supp_iac_cmd_cmp_evt
{
    ///Status of the command
    uint8_t     status;
    ///nb_of iac
    uint8_t  nb_iac;
};

/// HCI command complete event structure for read current IAC LAP command
/*@TRACE*/
struct hci_rd_curr_iac_lap_cmd_cmp_evt
{
    ///Status of the command
    uint8_t     status;
    ///nb of current iac
    uint8_t  nb_curr_iac;
    ///lap
    struct lap iac_lap;
};

/// HCI write current IAC LAP command structure
/*@TRACE*/
struct hci_wr_curr_iac_lap_cmd
{
    /// Number of current iac laps
    uint8_t  nb_curr_iac;
    ///lap
    struct lap iac_lap[(HCI_MAX_CMD_PARAM_SIZE / BD_ADDR_LAP_LEN) - 1];
};

/*@TRACE*/
struct hci_set_afh_host_ch_class_cmd
{
    ///AFH channel map
    struct bt_ch_map   afh_ch;
};

/// HCI command complete event structure for write inquiry scan type command structure
/*@TRACE*/
struct hci_rd_inq_scan_type_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t     status;
    /// Inquiry scan type
    uint8_t     inq_scan_type;
};

/*@TRACE*/
struct hci_wr_inq_scan_type_cmd
{
    /// Inquiry scan type
    uint8_t     inq_scan_type;
};

/// HCI command complete event structure for read inquiry mode command structure
/*@TRACE*/
struct hci_rd_inq_mode_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t     status;
    /// Inquiry mode
    uint8_t     inq_mode;
};

/*@TRACE*/
struct hci_wr_inq_mode_cmd
{
    /// Inquiry mode
    uint8_t     inq_mode;
};

/// HCI command complete event structure for write page scan type command structure
/*@TRACE*/
struct hci_rd_page_scan_type_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t     status;
    /// Page scan type
    uint8_t     page_scan_type;
};

/*@TRACE*/
struct hci_wr_page_scan_type_cmd
{
    /// Page scan type
    uint8_t     page_scan_type;
};

/// HCI command complete event structure for read assessment mode command structure
/*@TRACE*/
struct hci_rd_afh_ch_assess_mode_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t     status;
    ///AFH channel assessment mode
    uint8_t     afh_ch_ass_mode;
};

/*@TRACE*/
struct hci_wr_afh_ch_assess_mode_cmd
{
    ///AFH channel assessment mode
    uint8_t     afh_ch_ass_mode;
};

/// HCI command complete event structure for remote name request cancel command
/*@TRACE*/
struct hci_rd_ext_inq_rsp_cmd_cmp_evt
{
    ///status
    uint8_t     status;
    ///FEC required
    uint8_t     fec_req;
    ///Extended inquiry response
    struct eir  eir;
};

/*@TRACE*/
struct hci_wr_ext_inq_rsp_cmd
{
    ///FEC required
    uint8_t     fec_req;
    ///Extended inquiry response
    struct eir  eir;
};

/// HCI command complete event structure for remote name request cancel command
/*@TRACE*/
struct hci_rd_sp_mode_cmd_cmp_evt
{
    ///status
    uint8_t     status;
    ///Simple pairing mode
    uint8_t     sp_mode;
};

/*@TRACE*/
struct hci_wr_sp_mode_cmd
{
    ///Simple pairing mode
    uint8_t     sp_mode;
};

/// HCI command complete event structure for read oob data command
/*@TRACE*/
struct hci_rd_loc_oob_data_cmd_cmp_evt
{
    ///status
    uint8_t     status;
    ///hash part
    struct hash oob_c;
    ///random part
    struct randomizer oob_r;
};

/// HCI command complete event structure for read inquiry response transmit power command
/*@TRACE*/
struct hci_rd_inq_rsp_tx_pwr_lvl_cmd_cmp_evt
{
    ///status
    uint8_t status;
    ///TX power
    uint8_t tx_pwr;
};

/*@TRACE*/
struct hci_wr_inq_tx_pwr_lvl_cmd
{
    ///TX power
    int8_t tx_pwr;
};

/// HCI command complete event structure for read erroneous data reporting command
/*@TRACE*/
struct hci_rd_dft_err_data_rep_cmd_cmp_evt
{
    ///status
    uint8_t     status;
    ///Erroneous data reporting
    uint8_t     err_data_rep;
};

/*@TRACE*/
struct hci_wr_dft_err_data_rep_cmd
{
    ///Erroneous data reporting
    uint8_t err_data_rep;
};

/// HCI read LE Host Supported complete event
/*@TRACE*/
struct  hci_rd_le_host_supp_cmd_cmp_evt
{
    ///Status
    uint8_t status;
    ///LE_Supported_Host
    uint8_t le_supported_host;
    ///Simultaneous_LE_Host
    uint8_t simultaneous_le_host;
};

/// HCI write LE Host Supported command
/*@TRACE*/
struct  hci_wr_le_host_supp_cmd
{
    ///LE_Supported_Host
    uint8_t le_supported_host;
    ///Simultaneous_LE_Host
    uint8_t simultaneous_le_host;
};

/// HCI Set MWS Channel Parameters command
/*@TRACE*/
struct hci_set_mws_channel_params_cmd
{
    ///MWS_Channel_Enable
    uint8_t mws_channel_enable;
    ///MWS_RX_Center_Frequency
    uint16_t mws_rx_center_frequency;
    ///MWS_TX_Center_Frequency
    uint16_t mws_tx_center_frequency;
    ///MWS_RX_Channel_Bandwidth
    uint16_t mws_rx_channel_bandwidth;
    ///MWS_TX_Channel_Bandwidth
    uint16_t mws_tx_channel_bandwidth;
    ///MWS_Channel_Type
    uint8_t mws_channel_type;
};

/// HCI Set External Frame Configuration command
/*@TRACE*/
struct hci_set_external_frame_config_cmd
{
    /// Ext_Frame_Duration
    uint16_t ext_fr_duration;
    /// Ext_Frame_Sync_Assert_Offset
    int16_t ext_fr_sync_assert_offset;
    /// Ext_Frame_Sync_Assert_Jitter
    uint16_t ext_fr_sync_assert_jitter;
    /// Ext_Frame_Num_Periods
    uint8_t ext_fr_num_periods;
    /// Period Durations & Types
    struct ext_fr_period period[1/*__ARRAY_EMPTY*/];
};

/// HCI Set MWS Signaling command
/*@TRACE*/
struct hci_set_mws_signaling_cmd
{
    ///MWS_RX_Assert_Offset
    int16_t mws_rx_assert_offset;
    ///MWS_RX_Assert_Jitter
    uint16_t mws_rx_assert_jitter;
    ///MWS_RX_Deassert_Offset
    int16_t mws_rx_deassert_offset;
    ///MWS_RX_Deassert_Jitter
    uint16_t mws_rx_deassert_jitter;
    ///MWS_TX_Assert_Offset
    int16_t mws_tx_assert_offset;
    ///MWS_TX_Assert_Jitter
    uint16_t mws_tx_assert_jitter;
    ///MWS_TX_Deassert_Offset
    int16_t mws_tx_deassert_offset;
    ///MWS_TX_Deassert_Jitter
    uint16_t mws_tx_deassert_jitter;
    ///MWS_Pattern_Assert_Offset
    int16_t mws_pattern_assert_offset;
    ///MWS_Pattern_Assert_Jitter
    uint16_t mws_pattern_assert_jitter;
    ///MWS_Inactivity_Duration_Assert_Offset
    int16_t mws_inactivity_duration_assert_offset;
    ///MWS_Inactivity_Duration_Assert_Jitter
    uint16_t mws_inactivity_duration_assert_jitter;
    ///MWS_Scan_Frequency_Assert_Offset
    int16_t mws_scan_frequency_assert_offset;
    ///MWS_Scan_Frequency_Assert_Jitter
    uint16_t mws_scan_frequency_assert_jitter;
    ///MWS_Priority_Assert_Offset_Request
    uint16_t mws_priority_assert_offset_request;
};

/// HCI Set MWS Signaling command complete event
/*@TRACE*/
struct hci_set_mws_signaling_cmd_cmp_evt
{
    ///Status
    uint8_t status;
    ///Bluetooth_Rx_Priority_Assert_Offset
    int16_t bt_rx_prio_assert_offset;
    ///Bluetooth_Rx_Priority_Assert_Jitter
    uint16_t bt_rx_prio_assert_jitter;
    ///Bluetooth_Rx_Priority_Deassert_Offset
    int16_t bt_rx_prio_deassert_offset;
    ///Bluetooth_Rx_Priority_Deassert_Jitter
    uint16_t bt_rx_prio_deassert_jitter;
    ///802_Rx_Priority_Assert_Offset
    int16_t _802_rx_prio_assert_offset;
    ///802_Rx_Priority_Assert_Jitter
    uint16_t _802_rx_prio_assert_jitter;
    ///802_Rx_Priority_Deassert_Offset
    int16_t _802_rx_prio_deasssert_offset;
    ///802_Rx_Priority_Deassert_Jitter
    uint16_t _802_rx_prio_deassert_jitter;
    ///Bluetooth_Tx_On_Assert_Offset
    int16_t bt_tx_on_assert_offset;
    ///Bluetooth_Tx_On_Assert_Jitter
    uint16_t bt_tx_on_assert_jitter;
    ///Bluetooth_Tx_On_Deassert_Offset
    int16_t bt_tx_on_deassert_offset;
    ///Bluetooth_Tx_On_Deassert_Jitter
    uint16_t bt_tx_on_deassert_jitter;
    ///802_Tx_On_Assert_Offset
    int16_t _802_tx_on_assert_offset;
    ///802_Tx_On_Assert_Jitter
    uint16_t _802_tx_on_assert_jitter;
    ///802_Tx_On_Deassert_Offset
    int16_t _802_tx_on_deassert_offset;
    ///802_Tx_On_Deassert_Jitter
    uint16_t _802_tx_on_deassert_jitter;
};

/// HCI Set MWS Transport Layer command
/*@TRACE*/
struct hci_set_mws_transport_layer_cmd
{
    ///Transport_Layer
    uint8_t transport_layer;
    ///To_MWS_Baud_Rate
    uint32_t to_mws_baud_rate;
    ///From_MWS_Baud_Rate
    uint32_t from_mws_baud_rate;
};

/// HCI Set MWS Scan Frequency Table command
/*@TRACE*/
struct hci_set_mws_scan_freq_table_cmd
{
    ///Num_Scan_Frequencies
    uint8_t num_scan_frequencies;
    ///Scan_Frequencys Low & High
    struct mws_scan_freq scan_freq[1/*__ARRAY_EMPTY*/];
};

/// HCI Set MWS Pattern Configuration command
/*@TRACE*/
struct hci_set_mws_pattern_config_cmd
{
    ///MWS_PATTERN_Index
    uint8_t mws_pattern_index;
    ///MWS_PATTERN_NumIntervals
    uint8_t num_intervals;
    ///MWS_PATTERN_Interval Duration & Type
    struct mws_pattern_intv intv[1/*__ARRAY_EMPTY*/];
};

/// Hci Get MWS Transport Layer Configuration command complete event
/*@TRACE*/
struct hci_get_mws_transport_layer_config_cmd_cmp_evt
{
    ///Status
    uint8_t status;
    ///Num_Transports
    uint8_t num_transports;
    ///Transport_Layers
    struct mws_transport tran[1/*__ARRAY_EMPTY*/];
};

/// HCI read Secure Connections Host Support complete event
/*@TRACE*/
struct  hci_rd_sec_con_host_supp_cmd_cmp_evt
{
    ///Status
    uint8_t status;
    /// Secure Connections Host Support
    uint8_t sec_con_host_supp;
};

/// HCI write Secure Connections Host Support command
/*@TRACE*/
struct  hci_wr_sec_con_host_supp_cmd
{
    /// Secure Connections Host Support
    uint8_t sec_con_host_supp;
};

/// HCI write Secure Connections Test Mode command
struct  hci_wr_sec_con_test_mode_cmd
{
    /// Connection handle
    uint16_t conhdl;
    /// DM1 ACL-U mode
    uint8_t dm1_acl_u_mode;
    /// eSCO loopback mode
    uint8_t esco_loopback_mode;
};

/// HCI write Secure Connections Test Mode complete event
struct  hci_wr_sec_con_test_mode_cmd_cmp_evt
{
    ///Status
    uint8_t status;
    /// Connection handle
    uint16_t conhdl;
};

/// HCI Set Reserved LT_ADDR command
/*@TRACE*/
struct hci_set_res_lt_addr_cmd
{
    /// LT_ADDR
    uint8_t lt_addr;
};

/// HCI Set Reserved LT_ADDR command complete event
/*@TRACE*/
struct hci_set_res_lt_addr_cmd_cmp_evt
{
    /// Status
    uint8_t  status;
    /// LT_ADDR
    uint8_t lt_addr;
};

/// HCI Delete Reserved LT_ADDR command
/*@TRACE*/
struct hci_del_res_lt_addr_cmd
{
    /// LT_ADDR
    uint8_t lt_addr;
};

/// HCI Delete Reserved LT_ADDR command complete event
/*@TRACE*/
struct hci_del_res_lt_addr_cmd_cmp_evt
{
    /// Status
    uint8_t  status;
    /// LT_ADDR
    uint8_t lt_addr;
};

/// HCI Set Connectionless Slave Broadcast Data command
/*@TRACE*/
struct hci_set_con_slv_bcst_data_cmd
{
    /// LT_ADDR
    uint8_t lt_addr;
    /// Fragment
    uint8_t fragment;
    /// Data_Length (in bytes)
    uint8_t data_length;
    /// Data
    uint8_t data[__ARRAY_EMPTY];
};

/// HCI Set Connectionless Slave Broadcast Data command complete event
/*@TRACE*/
struct hci_set_con_slv_bcst_data_cmd_cmp_evt
{
    /// Status
    uint8_t  status;
    /// LT_ADDR
    uint8_t lt_addr;
};

/// HCI Read Synchronization Train Parameters command complete event
/*@TRACE*/
struct hci_rd_sync_train_param_cmd_cmp_evt
{
    /// Status
    uint8_t  status;
    /// Sync_Train_Interval (in slots)
    uint16_t sync_train_int;
    /// synchronization_trainTO (in slots)
    uint32_t sync_train_to;
    /// Service_Data
    uint8_t service_data;
};

/// HCI Write Synchronization Train Parameters command
/*@TRACE*/
struct hci_wr_sync_train_param_cmd
{
    /// Interval_Min (in slots)
    uint16_t int_min;
    /// Interval_Max (in slots)
    uint16_t int_max;
    /// synchronization_trainTO (in slots)
    uint32_t sync_train_to;
    /// Service_Data
    uint8_t service_data;
};

/// HCI Write Synchronization Train Parameters command complete event
/*@TRACE*/
struct hci_wr_sync_train_param_cmd_cmp_evt
{
    /// Status
    uint8_t  status;
    /// Sync_Train_Interval (in slots)
    uint16_t sync_train_int;
};

// HCI Synchronization Train Complete event
struct hci_sync_train_cmp_evt
{
    /// Status
    uint8_t  status;
};

/// HCI read authenticated payload timeout command
/*@TRACE*/
struct hci_rd_auth_payl_to_cmd
{
    ///Connection handle
    uint16_t     conhdl;
};

/// HCI command complete event structure for the Read Authenticated Payload Timeout Command
/*@TRACE*/
struct hci_rd_auth_payl_to_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t     status;
    ///Connection handle
    uint16_t     conhdl;
    ///Authenticated payload timeout
    uint16_t     auth_payl_to;
};

/// HCI command complete event structure for read oob extended data command
/*@TRACE*/
struct hci_rd_loc_oob_ext_data_cmd_cmp_evt
{
    ///status
    uint8_t     status;
    ///hash part
    struct hash oob_c_192;
    ///random part
    struct randomizer oob_r_192;
    ///hash part
    struct hash oob_c_256;
    ///random part
    struct randomizer oob_r_256;
};

/// HCI read Extended Page Timeout CC event
/*@TRACE*/
struct hci_rd_ext_page_to_cmd_cmp_evt
{
    /// Status
    uint8_t  status;
    /**
     * Extended Page Timeout measured in Number of Baseband slots. Interval Length = N * 0.625 msec (1 Baseband slot)
     * Range for N: 0x0000 (default) - 0xFFFF
     * Time Range: 0 - 40.9 Seconds
     */
    uint16_t ext_page_to;
};

/// HCI write Extended Page Timeout
/*@TRACE*/
struct hci_wr_ext_page_to_cmd
{
    /**
     * Extended Page Timeout measured in Number of Baseband slots. Interval Length = N * 0.625 msec (1 Baseband slot)
     * Range for N: 0x0000 (default) - 0xFFFF
     * Time Range: 0 - 40.9 Seconds
     */
    uint16_t ext_page_to;
};

/// HCI read Extended Inquiry Length CC event
/*@TRACE*/
struct hci_rd_ext_inq_len_cmd_cmp_evt
{
    /// Status
    uint8_t  status;
    /// Extended Inquiry Length
    uint16_t ext_inq_len;
};

/// HCI write Extended Inquiry Length
/*@TRACE*/
struct hci_wr_ext_inq_len_cmd
{
    /// Extended Inquiry Length
    uint16_t ext_inq_len;
};

/// HCI Set Ecosystem Base Interval command
/*@TRACE*/
struct hci_set_eco_base_intv_cmd
{
    /// Interval
    uint16_t interval;
};

/// HCI Configure Data Path command
/*@TRACE*/
struct hci_config_data_path_cmd
{
    /// Data Path Direction
    uint8_t data_path_direction;
    /// Data Path ID
    uint8_t data_path_id;
    /// Vendor Specific Config Length (in bytes)
    uint8_t vendor_specific_cfg_len;
    /// Vendor Specific Config
    uint8_t vendor_specific_cfg[__ARRAY_EMPTY];
};

#if (BT_53)
/// HCI Set Min Encryption Key Size command
/*@TRACE*/
struct hci_set_min_enc_key_size_cmd
{
    /// Min Encryption Key Size
    uint8_t min_enc_key_size;
};

/// HCI encryption change V2 event structure
/*@TRACE*/
struct hci_enc_chg_v2_evt
{
    /// Status for command reception
    uint8_t status;
    /// Connection handle
    uint16_t conhdl;
    /// Encryption enabled information
    uint8_t enc_stat;
    /// Encryption key size
    uint8_t enc_key_size;
};
#endif // (BT_53)

/*
 * HCI INFORMATIONAL PARAMETERS COMMANDS PARAMETERS
 ****************************************************************************************
 */

///HCI command complete event structure for read local version information
/*@TRACE*/
struct hci_rd_local_ver_info_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t     status;
    ///HCI version number
    uint8_t     hci_ver;
    ///HCI revision number
    uint16_t    hci_rev;
    ///LMP version
    uint8_t     lmp_ver;
    ///manufacturer name
    uint16_t    manuf_name;
    ///LMP Subversion
    uint16_t    lmp_subver;
};

///HCI command complete event structure for read local supported commands
/*@TRACE*/
struct hci_rd_local_supp_cmds_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t             status;
    ///Supported Commands structure
    struct supp_cmds    local_cmds;
};

/// HCI command complete event structure for read local supported features command
/*@TRACE*/
struct hci_rd_local_supp_feats_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t         status;
    ///Local  supported features
    struct features feats;
};

/*@TRACE*/
struct hci_rd_local_ext_feats_cmd
{
    ///Page number
    uint8_t page_nb;
};

/// HCI command complete event structure for read local extended features command
/*@TRACE*/
struct hci_rd_local_ext_feats_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t     status;
    ///Page number
    uint8_t     page_nb;
    ///Maximum page number
    uint8_t     page_nb_max;
    ///Extended LMP features
    struct features ext_feats;
};

///HCI command complete event structure for the Read Buffer Size Command
/*@TRACE*/
struct hci_rd_buf_size_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t     status;
    ///ACL data packet length controller can receive from host
    uint16_t     hc_data_pk_len;
    ///Synchronous data packet length controller can receive from host
    uint8_t      hc_sync_pk_len;
    ///Total number of ACL data packets controller can receive from host
    uint16_t     hc_tot_nb_data_pkts;
    ///Total number of synchronous data packets controller can receive from host
    uint16_t     hc_tot_nb_sync_pkts;
};

///HCI command complete event structure for read bd address
/*@TRACE*/
struct hci_rd_bd_addr_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t             status;
    ///BD address
    struct bd_addr      local_addr;
};

/// HCI command complete event structure for read local supported codecs
/*@TRACE*/
struct hci_rd_local_supp_codecs_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t               status;
    uint8_t               nb_supp_codecs;
    uint8_t               nb_supp_vendor_specific_codecs;

//    ///Supported Codecs structure
//    struct supp_codecs    local_codecs;
};

/// HCI command complete event structure for read local supported codecs v2
/*@TRACE*/
struct hci_rd_local_supp_codecs_v2_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t               status;
    uint8_t               num_supp_standard_codecs;
    uint8_t               num_supp_vendor_specific_codecs;

//    ///Supported Codecs structure
//    struct supp_codecs    local_codecs;
};

/// HCI command complete event structure for read local simple pairing options
struct hci_rd_local_sp_opt_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t status;
    /// Simple Pairing options (bit 0: Remote public key validation)
    uint8_t sp_opt;
    /// Maximum Encryption Key Size (in octets)
    uint8_t max_enc_key_size;
};

/// HCI Read Local Supported Codec Capabilities command
struct hci_rd_local_supp_codec_cap_cmd
{
    /// Codec ID
    uint8_t codec_id[CODEC_ID_LEN];
    /// Logical Transport Type
    uint8_t logical_transport_type;
    /// Direction
    uint8_t direction;
};

/// HCI command complete event structure for Read Local Supported Codec Capabilities command
struct hci_rd_local_supp_codec_cap_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t status;
    /// Num Codec Capabilities
    uint8_t num_codec_cap;
    /// Codec Capability Length [i]
    uint8_t codec_cap_len; // TODO: update when the number of codecs is available
    /// Codec Capability [i]
    uint8_t codec_cap[__ARRAY_EMPTY]; // TODO: update when the number of codecs is available
};

/// HCI Read Local Supported Controller Delay command
struct hci_rd_local_supp_ctrl_delay_cmd
{
    /// Codec ID
    uint8_t codec_id[CODEC_ID_LEN];
    /// Logical Transport Type
    uint8_t logical_transport_type;
    /// Direction
    uint8_t direction;
    /// Codec Configuration Length
    uint8_t codec_cfg_len;
    /// Codec Configuration
    uint8_t codec_cfg[__ARRAY_EMPTY];
};

/// HCI command complete event structure for Read Local Supported Controller Delay command
struct hci_rd_local_supp_ctrl_delay_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t status;
    /// Min Controller Delay
    uint8_t min_ctrl_delay[3];
    /// Max Controller Delay
    uint8_t max_ctrl_delay[3];
};

/*
 * HCI STATUS PARAMETERS COMMANDS PARAMETERS
 ****************************************************************************************
 */

/// HCI command complete event structure for read rssi
/*@TRACE*/
struct hci_rd_rssi_cmd_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///RSSI value
    int8_t rssi;
};

/*@TRACE*/
struct hci_rd_clk_cmd
{
    ///Connection handle
    uint16_t conhdl;
    ///Which clock
    uint8_t clk_type;
};

/// HCI read clock command structure
/*@TRACE*/
struct hci_rd_clk_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t     status;
    ///Connection handle
    uint16_t conhdl;
    ///clock
    uint32_t clk;
    ///Accuracy
    uint16_t clk_acc;
};


/*
 * HCI TESTING COMMANDS PARAMETERS
 ****************************************************************************************
 */

/// HCI command complete event structure for read loop back mode command
/*@TRACE*/
struct hci_rd_loopback_mode_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t     status;
    ///Local  supported features
    uint8_t     lb_mode;
};

/*@TRACE*/
struct hci_wr_loopback_mode_cmd
{
    ///Local  supported features
    uint8_t     lb_mode;
};

/*@TRACE*/
struct hci_wr_sp_dbg_mode_cmd
{
    ///Simple pairing mode
    uint8_t     sp_mode;
};


/*
 * HCI LE CONTROLLER COMMANDS PARAMETERS
 ****************************************************************************************
 */

///HCI LE Set Event Mask Command parameters structure
/*@TRACE*/
struct hci_le_set_evt_mask_cmd
{
    ///LE Event Mask
    struct evt_mask le_mask;
};

///HCI LE Set Random Address Command parameters structure
/*@TRACE*/
struct hci_le_set_rand_addr_cmd
{
    ///Random address to set
    struct bd_addr rand_addr;
};

///HCI LE Set Advertising Parameters Command parameters structure
/*@TRACE*/
struct hci_le_set_adv_param_cmd
{
    ///Minimum interval for advertising
    uint16_t       adv_intv_min;
    ///Maximum interval for advertising
    uint16_t       adv_intv_max;
    ///Advertising type
    uint8_t        adv_type;
    ///Own address type:  public=0 / random=1 / rpa_or_pub=2 / rpa_or_rnd=3
    uint8_t        own_addr_type;
    ///Peer address type: public=0 / random=1
    uint8_t        peer_addr_type;
    ///Peer Bluetooth device address
    struct bd_addr peer_addr;
    ///Advertising channel map
    uint8_t        adv_ch_map;
    ///Advertising filter policy
    uint8_t        adv_filt_policy;
};

///HCI LE Set Advertising Data Command parameters structure
/*@TRACE*/
struct hci_le_set_adv_data_cmd
{
    ///Advertising data length
    uint8_t         adv_data_len;
    ///Advertising data - maximum 31 bytes
    struct adv_data data;
};

///HCI LE Set Scan Response Data Command parameters structure
/*@TRACE*/
struct hci_le_set_scan_rsp_data_cmd
{
    ///Scan response data length
    uint8_t              scan_rsp_data_len;
    ///Scan response data - maximum 31 bytes
    struct scan_rsp_data data;
};

///HCI LE Set Advertise Enable Command parameters structure
/*@TRACE*/
struct hci_le_set_adv_en_cmd
{
    ///Advertising enable - 0=disabled, 1=enabled
    uint8_t        adv_en;
};

///HCI LE Set Scan Parameters Command parameters structure
/*@TRACE*/
struct hci_le_set_scan_param_cmd
{
    ///Scan type - 0=passive / 1=active
    uint8_t        scan_type;
    ///Scan interval
    uint16_t       scan_intv;
    ///Scan window size
    uint16_t       scan_window;
    ///Own address type - public=0 / random=1 / rpa_or_pub=2 / rpa_or_rnd=3
    uint8_t        own_addr_type;
    ///Scan filter policy
    uint8_t        scan_filt_policy;
};

///HCI LE Set Scan Enable Command parameters structure
/*@TRACE*/
struct hci_le_set_scan_en_cmd
{
    ///Scan enable - 0=disabled, 1=enabled
    uint8_t        scan_en;
    ///Enable for duplicates filtering - 0 =disabled/ 1=enabled
    uint8_t        filter_duplic;
};

///HCI LE Create Connection Command parameters structure
/*@TRACE*/
struct hci_le_create_con_cmd
{
    ///Scan interval (N * 0.625 ms)
    uint16_t       scan_intv;
    ///Scan window size (N * 0.625 ms)
    uint16_t       scan_window;
    ///Initiator filter policy
    uint8_t        init_filt_policy;
    ///Peer address type - public=0 / random=1 / rpa_or_pub=2 / rpa_or_rnd=3
    uint8_t        peer_addr_type;
    ///Peer BD address
    struct bd_addr peer_addr;
    ///Own address type - public=0 / random=1 / rpa_or_pub=2 / rpa_or_rnd=3
    uint8_t        own_addr_type;
    ///Minimum of connection interval (N * 1.25 ms)
    uint16_t       con_intv_min;
    ///Maximum of connection interval (N * 1.25 ms)
    uint16_t       con_intv_max;
    ///Connection latency
    uint16_t       con_latency;
    ///Link supervision timeout
    uint16_t       superv_to;
    ///Minimum CE length (N * 0.625 ms)
    uint16_t       ce_len_min;
    ///Maximum CE length (N * 0.625 ms)
    uint16_t       ce_len_max;
};

///HCI LE Add Device to White List Command parameters structure
/*@TRACE*/
struct hci_le_add_dev_to_wlst_cmd
{
    ///Type of address of the device to be added to the White List - 0=public/1=random
    uint8_t        dev_addr_type;
    ///Address of device to be added to White List
    struct bd_addr dev_addr;
};

///HCI LE Remove Device from White List Command parameters structure
/*@TRACE*/
struct hci_le_rmv_dev_from_wlst_cmd
{
    ///Type of address of the device to be removed from the White List - 0=public/1=random
    uint8_t        dev_addr_type;
    ///Address of device to be removed from White List
    struct bd_addr dev_addr;
};

///HCI LE Set Extended Scan Paramaters Command parameters structure
/*@TRACE*/
struct hci_le_set_ext_scan_param_cmd
{
    ///Own address type public/random/rpa
    uint8_t         own_addr_type;
    ///Scanning filter policy
    uint8_t         scan_filt_policy;
    ///Indicates the PHY(s) on which the advertising packets should be received
    uint8_t         scan_phys;
    ///Paramaters for PHY(s)
    struct scan_phy_param
    {
        ///Scaning Type: passive/active
        uint8_t         scan_type;
        ///Scan interval (slots)
        uint16_t        scan_intv;
        ///Scan window size (slots)
        uint16_t        scan_window;
    } phy[MAX_SCAN_PHYS];
};

///HCI LE Set Extended Scan Enable Command parameters structure
/*@TRACE*/
struct hci_le_set_ext_scan_en_cmd
{
    ///Scan enable - 0=disabled, 1=enabled
    uint8_t             scan_en;
    ///Filter duplicates - 0=disabled, 1=enabled, 2=enabled & reset each scan period
    uint8_t             filter_duplic;
    ///Scan duration (Time=N*10ms)  | 0x0000: Scan continuously until explicitly disable
    uint16_t            duration;
    ///Scan period (Time=N*1.28sec) | 0x0000: Periodic scanning disabled
    uint16_t            period;
};

/*@TRACE*/
struct init_phy_param
{
    ///Scan interval (N * 0.625 ms)
    uint16_t        scan_interval;
    ///Scan window size (N * 0.625 ms)
    uint16_t        scan_window;
    ///Minimum of connection interval (N * 1.25 ms)
    uint16_t       con_intv_min;
    ///Maximum of connection interval (N * 1.25 ms)
    uint16_t       con_intv_max;
    ///Connection latency
    uint16_t       con_latency;
    ///Link supervision timeout
    uint16_t       superv_to;
    ///Minimum CE length (N * 0.625 ms)
    uint16_t       ce_len_min;
    ///Maximum CE length (N * 0.625 ms)
    uint16_t       ce_len_max;
};

///HCI LE Extended Create Connection Command parameters structure
/*@TRACE*/
struct hci_le_ext_create_con_cmd
{
    ///Initiator filter policy
    uint8_t         init_filter_policy;
    ///Own address type public/random/rpa
    uint8_t         own_addr_type;
    ///Peer address type public/random/rpa
    uint8_t         peer_addr_type;
    ///Peer address
    struct bd_addr  peer_addr;
    ///Indicates the PHY(s) on which the advertising packets should be received
    uint8_t         init_phys;
    ///Paramaters for PHY(s)
    struct init_phy_param phy[MAX_INIT_PHYS];
};

///HCI LE Periodic Advertising Create Sync Command parameters structure
/*@TRACE*/
struct hci_le_per_adv_create_sync_cmd
{
    /// Options (@see enum per_sync_opt)
    uint8_t         options;
    ///Advertising SID
    uint8_t         adv_sid;
    ///Advertising address type
    uint8_t         adv_addr_type;
    ///Advertiser address
    struct bd_addr  adv_addr;
    ///max Skip after receive
    uint16_t        skip;
    ///Sync timeout (Time=N*10ms)
    uint16_t        sync_to;
    /// Sync CTE type (@see enum sync_cte_type)
    uint8_t         sync_cte_type;
};

///HCI LE Periodic Advertising Terminate Sync Command parameters structure
/*@TRACE*/
struct hci_le_per_adv_term_sync_cmd
{
    ///Sync handle
    uint16_t        sync_handle;
};

///HCI LE Add Device to Periodic Advertiser List Command parameters structure
/*@TRACE*/
struct hci_le_add_dev_to_per_adv_list_cmd
{
    ///Advertiser address type
    uint8_t         adv_addr_type;
    ///Advertiser address
    struct bd_addr  adv_addr;
    ///Advertising SID
    uint8_t         adv_sid;
};

///HCI LE Remove Device from Periodic Advertiser List Command parameters structure
/*@TRACE*/
struct hci_le_rmv_dev_from_per_adv_list_cmd
{
    ///Advertiser address type
    uint8_t         adv_addr_type;
    ///Advertiser address
    struct bd_addr  adv_addr;
    ///Advertising SID
    uint8_t         adv_sid;
};

///HCI LE Set Privacy Mode Command parameters structure
/*@TRACE*/
struct hci_le_set_priv_mode_cmd
{
    ///Peer identity address type
    uint8_t         peer_addr_type;
    ///Peer identity address
    struct bd_addr  peer_addr;
    ///Privacy mode
    uint8_t         priv_mode;
};

///HCI LE Set Host Channel Classification Command parameters structure
/*@TRACE*/
struct hci_le_set_host_ch_class_cmd
{
    ///Channel map
    struct le_ch_map ch_map;
};


///HCI LE Receiver Test v1 Command parameters structure
/*@TRACE*/
struct hci_le_rx_test_v1_cmd
{
    /// RX channel, range: 0x00 to 0x27
    uint8_t     rx_channel;
};

///HCI LE Transmitter Test v1 Command parameters structure
/*@TRACE*/
struct hci_le_tx_test_v1_cmd
{
    /// TX channel, range: 0x00 to 0x27
    uint8_t     tx_channel;
    /// Length of test data in bytes, range: 0x00 to 0xFF
    uint8_t     test_data_len;
    /**
     * Packet payload
     * 0x00 PRBS9 sequence "11111111100000111101" (in transmission order) as described in [Vol 6] Part F, Section 4.1.5
     * 0x01 Repeated "11110000" (in transmission order) sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x02 Repeated "10101010" (in transmission order) sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x03 PRBS15 sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x04 Repeated "11111111" (in transmission order) sequence
     * 0x05 Repeated "00000000" (in transmission order) sequence
     * 0x06 Repeated "00001111" (in transmission order) sequence
     * 0x07 Repeated "01010101" (in transmission order) sequence
     * 0x08-0xFF Reserved for future use
     */
    uint8_t     pkt_payl;
};

///HCI LE Encrypt Command parameters structure
/*@TRACE*/
struct hci_le_enc_cmd
{
    ///Long term key structure
    struct ltk     key;
    ///Pointer to buffer with plain data to encrypt - 16 bytes
    uint8_t        plain_data[16];
};

/// HCI LE Connection Update Command parameters structure
/*@TRACE*/
struct hci_le_con_update_cmd
{
    ///Connection Handle
    uint16_t       conhdl;
    ///Minimum of connection interval (units of 1.25 ms)
    uint16_t       con_intv_min;
    ///Maximum of connection interval (units of 1.25 ms)
    uint16_t       con_intv_max;
    ///Connection latency (units of connection event)
    uint16_t       con_latency;
    ///Link supervision timeout (units of 10 ms)
    uint16_t       superv_to;
    ///Minimum of CE length (units of 0.625 ms)
    uint16_t       ce_len_min;
    ///Maximum of CE length (units of 0.625 ms)
    uint16_t       ce_len_max;
};

/// HCI LE Enable Encryption Command parameters structure
/*@TRACE*/
struct hci_le_en_enc_cmd
{
    ///Connection handle
    uint16_t        conhdl;
    ///Random number - 8B
    struct rand_nb  nb;
    ///Encryption Diversifier
    uint16_t       enc_div;
    ///Long term key
    struct ltk     ltk;
};

/// HCI long term key request reply command parameters structure
/*@TRACE*/
struct hci_le_ltk_req_reply_cmd
{
    ///Connection handle
    uint16_t        conhdl;
    ///Long term key
    struct ltk      ltk;
};

/// HCI long term key request negative reply command parameters structure
/*@TRACE*/
struct hci_le_ltk_req_neg_reply_cmd
{
    ///Connection handle
    uint16_t        conhdl;
};

/// HCI LE remote connection parameter request reply command parameters structure
/*@TRACE*/
struct hci_le_rem_con_param_req_reply_cmd
{
    ///Connection handle
    uint16_t        conhdl;
    ///Interval_Min
    uint16_t        interval_min;
    ///Interval_Max
    uint16_t        interval_max;
    ///Latency
    uint16_t        latency;
    ///Timeout
    uint16_t        timeout;
    ///Minimum_CE_Length
    uint16_t        min_ce_len;
    ///Maximum_CE_Length
    uint16_t        max_ce_len;
};

/// HCI LE remote connection parameter request negative reply command parameters structure
/*@TRACE*/
struct hci_le_rem_con_param_req_neg_reply_cmd
{
    ///Connection handle
    uint16_t        conhdl;
    ///Reason
    uint8_t         reason;
};


/// HCI LE Set Data Length  Command parameters structure
/*@TRACE*/
struct hci_le_set_data_len_cmd
{
    ///Connection Handle
    uint16_t       conhdl;
    ///Preferred maximum number of payload octets that the local Controller should include
    ///in a single Link Layer Data Channel PDU.
    uint16_t       tx_octets;
    ///Preferred maximum number of microseconds that the local Controller should use to transmit
    ///a single Link Layer Data Channel PDU
    uint16_t       tx_time;
};

/// HCI LE Read Suggested Default Data Length Command
/*@TRACE*/
struct hci_le_wr_suggted_dft_data_len_cmd
{
    ///Suggested value for the Controller's maximum transmitted number of payload octets to be used
    uint16_t       suggted_max_tx_octets;
    ///Suggested value for the Controller's maximum packet transmission time to be used
    uint16_t       suggted_max_tx_time;
};

/// HCI LE Add Device to Resolving List Command
/*@TRACE*/
struct hci_le_add_dev_to_rslv_list_cmd
{
    /// Peer Identity Address Type
    uint8_t             peer_id_addr_type;
    /// Peer Identity Address
    struct bd_addr      peer_id_addr;
    /// Peer IRK
    struct irk          peer_irk;
    /// Local IRK
    struct irk          local_irk;
};

/// HCI LE Remove Device From Resolving List Command
/*@TRACE*/
struct hci_le_rmv_dev_from_rslv_list_cmd
{
    /// Peer Identity Address Type
    uint8_t             peer_id_addr_type;
    /// Peer Identity Address
    struct bd_addr      peer_id_addr;
};

/// HCI LE Read Peer Resolvable Address Command
/*@TRACE*/
struct hci_le_rd_peer_rslv_addr_cmd
{
    /// Peer Identity Address Type
    uint8_t             peer_id_addr_type;
    /// Peer Identity Address
    struct bd_addr      peer_id_addr;
};

/// HCI LE Read Local Resolvable Address Command
/*@TRACE*/
struct hci_le_rd_loc_rslv_addr_cmd
{
    /// Peer Identity Address Type
    uint8_t             peer_id_addr_type;
    /// Peer Identity Address
    struct bd_addr      peer_id_addr;
};

/// HCI LE Set Address Resolution Enable Command
/*@TRACE*/
struct hci_le_set_addr_resol_en_cmd
{
    /// Address Resolution Enable
    uint8_t             enable;
};

/// HCI LE Set Resolvable Private Address Timeout Command
/*@TRACE*/
struct hci_le_set_rslv_priv_addr_to_cmd
{
    /// RPA Timeout
    uint16_t            rpa_timeout;
};

/*
 * HCI EVENTS PARAMETERS
 ****************************************************************************************
 */

/// HCI inquiry complete event structure
/*@TRACE*/
struct hci_inq_cmp_evt
{
    ///Status of the procedure
    uint8_t status;
};


/// HCI Inquiry result event structure (with only 1 result)
/*@TRACE*/
struct hci_inq_res_evt
{

    ///Number of response
    uint8_t     nb_rsp;
    ///BdAddr
    struct bd_addr bd_addr;
    ///Page Scan Repetition Mode
    uint8_t     page_scan_rep_mode;
    ///Reserved
    uint8_t     reserved1;
    ///Reserved
    uint8_t     reserved2;
    ///class of device
    struct devclass class_of_dev;
    ///Clock Offset
    uint16_t        clk_off;

};

/// HCI Inquiry result with rssi event structure (with only 1 result)
/*@TRACE*/
struct hci_inq_res_with_rssi_evt
{
    ///Number of response
    uint8_t     nb_rsp;
    ///BdAddr
    struct bd_addr  bd_addr;
    ///Page Scan Repetition Mode
    uint8_t     page_scan_rep_mode;
    ///Reserved
    uint8_t     reserved1;
    ///class of device
    struct devclass class_of_dev;
    ///Clock Offset
    uint16_t     clk_off;
    ///RSSI
    int8_t       rssi;

};

/// HCI Extended inquiry result indication structure (with only 1 result)
/*@TRACE*/
struct hci_ext_inq_res_evt
{
    ///Number of response
    uint8_t     nb_rsp;
    ///BdAddr
    struct bd_addr  bd_addr;
    ///Page Scan Repetition Mode
    uint8_t     page_scan_rep_mode;
    ///Reserved
    uint8_t     reserved1;
    ///class of device
    struct devclass class_of_dev;
    ///Clock Offset
    uint16_t        clk_off;
    ///RSSI
    int8_t          rssi;
    ///Extended inquiry response data
    struct eir      eir;
};

/// HCI disconnect complete event structure
/*@TRACE*/
struct hci_disc_cmp_evt
{
    ///Status of received command
    uint8_t     status;
    ///Connection Handle
    uint16_t    conhdl;
    ///Reason for disconnection
    uint8_t     reason;
};

/// HCI basic command complete event structure
/*@TRACE
 * hci_dbg_wr_par_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_dbg_llcp_discard_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_reset_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_set_evt_mask_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_le_set_per_adv_en_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_le_create_con_cancel_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_le_set_evt_mask_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_le_set_host_ch_class_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_le_wr_suggted_dft_data_len_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_le_set_dft_phy_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_le_rmv_dev_from_wlst_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_le_set_adv_param_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_le_set_adv_data_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_le_set_scan_rsp_data_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_le_set_adv_en_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_le_set_scan_param_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_le_set_scan_en_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_le_set_rand_addr_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_le_set_addr_resol_en_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_le_set_rslv_priv_addr_to_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_le_rmv_dev_from_rslv_list_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_le_create_con_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_dbg_plf_reset_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * hci_le_wr_rf_path_comp_cmd_cmp_evt = hci_basic_cmd_cmp_evt
 * */
struct hci_basic_cmd_cmp_evt
{
    ///Status of the command reception
    uint8_t status;
};

/// HCI basic command complete event structure with connection handle
/*@TRACE*/
struct hci_basic_conhdl_cmd_cmp_evt
{
    /// status
    uint8_t     status;
    /// connection handle
    uint16_t    conhdl;
};

/// HCI basic command complete event structure with connection handle
/*@TRACE*/
struct hci_basic_conhdl_le_cmd_cmp_evt
{
    ///LE Subevent code
    uint8_t     subcode;
    /// status
    uint8_t     status;
    /// connection handle
    uint16_t    conhdl;
};

/// HCI basic command complete event structure with BD address
struct hci_basic_bd_addr_cmd_cmp_evt
{
    ///status
    uint8_t         status;
    ///BdAddr
    struct bd_addr bd_addr;
};

/// HCI basic event structure with status and BD address
struct hci_basic_stat_bd_addr_evt
{
    ///status
    uint8_t         status;
    ///BdAddr
    struct bd_addr bd_addr;
};

/// HCI basic event including a connection handle as parameter
struct hci_basic_conhdl_evt
{
    ///Connection handle
    uint16_t     conhdl;
};

/// HCI complete event with status only.
/*@TRACE*/
struct hci_cmd_stat_event
{
    /// Status of the command reception
    uint8_t status;
};

/// HCI number of packet complete event structure
/*@TRACE*/
struct hci_nb_cmp_pkts_evt
{
    /// number of handles
    uint8_t     nb_of_hdl;

    /// Array of number of completed packets per connection
    struct hci_nb_cmp_pkt con[MAX_NB_CMP_PKTS_CON];
};

/// HCI data buffer overflow event structure
/*@TRACE*/
struct hci_data_buf_ovflw_evt
{
    ///Link type
    uint8_t link_type;
};

/// HCI Hardware Error Event parameters structure
/*@TRACE*/
struct hci_hw_err_evt
{
    /// HW error code
    uint8_t hw_code;
};

/// HCI encryption change event structure
struct hci_enc_change_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///Encryption enabled information
    uint8_t     enc_stat;
};

/// HCI encryption key refresh complete event structure
/*@TRACE
 * hci_enc_key_refresh_evt = hci_enc_key_ref_cmp_evt */
struct hci_enc_key_ref_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
};

/// HCI Authenticated Payload Timeout Expired Event structure
/*@TRACE*/
struct hci_auth_payl_to_exp_evt
{
    ///Connection handle
    uint16_t     conhdl;
};

/// HCI command complete event structure for create connection
/*@TRACE*/
struct hci_con_cmp_evt
{
    /// Status
    uint8_t             status;
    ///Connection handle
    uint16_t            conhdl;
    ///Bluetooth Device address
    struct bd_addr      bd_addr;
    ///Link type
    uint8_t             link_type;
    ///Encryption state
    uint8_t             enc_en;
};

/// HCI command complete event structure for qos setup
/*@TRACE*/
struct hci_qos_setup_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///Flags
    uint8_t flags;
    ///Service type
    uint8_t serv_type;
    ///Token rate
    uint32_t tok_rate;
    ///Peak bandwidth
    uint32_t pk_bw;
    ///Latency
    uint32_t lat;
    ///Delay variation
    uint32_t del_var;
};

/// HCI flow specification complete event parameters structure
/*@TRACE*/
struct hci_flow_spec_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///Flags
    uint8_t flags;
    ///Flow direction
    uint8_t flow_dir;
    ///Service type
    uint8_t serv_type;
    ///Token rate
    uint32_t tk_rate;
    ///Token buffer size
    uint32_t tk_buf_sz;
    ///Peak bandwidth
    uint32_t pk_bw;
    ///Access latency
    uint32_t acc_lat;
};

/// HCI role change event parameters structure
/*@TRACE*/
struct hci_role_chg_evt
{
    ///Status
    uint8_t status;
    ///BD address
    struct bd_addr bd_addr;
    ///New role
    uint8_t new_role;
};

/// HCI complete event structure for the read clock offset command
/*@TRACE*/
struct hci_rd_clk_off_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///Clock offset
    uint16_t clk_off_val;
};

/// HCI event structure for the flush occurred event
/*@TRACE*/
struct hci_flush_occurred_evt
{
    ///Connection handle
    uint16_t conhdl;
};

/// HCI max slot change event structure
/*@TRACE*/
struct hci_max_slot_chg_evt
{
    ///Connection handle
    uint16_t conhdl;
    ///Max slot
    uint8_t max_slot;
};

/// HCI sniff subrating event parameters structure
/*@TRACE*/
struct hci_sniff_sub_evt
{
    ///Status.
    uint8_t     status;
    ///Connection handle
    uint16_t    conhdl;
    ///Maximum transmit latency
    uint16_t    max_lat_tx;
    ///Maximum receive latency
    uint16_t    max_lat_rx;
    ///Minimum remote TO
    uint16_t    min_rem_to;
    ///Minimum local TO
    uint16_t    min_loc_to;
};

/// HCI read remote extended features complete event parameters structure
/*@TRACE*/
struct hci_rd_rem_ext_feats_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///page number
    uint8_t pg_nb;
    ///page number max
    uint8_t pg_nb_max;
    ///ext LMP features
    struct features ext_feats;
};

/// HCI read remote extended features complete event parameters structure
/*@TRACE*/
struct hci_rem_host_supp_feats_notif_evt
{
    ///BD address
    struct bd_addr bd_addr;
    ///ext lmp features
    struct features ext_feats;
};

/// HCI command complete event structure for the read remote supported features command
/*@TRACE*/
struct hci_rd_rem_supp_feats_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///Remote features
    struct features rem_feats;
};

/// HCI command complete event structure for the read remote information version command
/*@TRACE*/
struct hci_rd_rem_ver_info_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///LMP version
    uint8_t     vers;
    ///Manufacturer name
    uint16_t    compid;
    ///LMP subversion
    uint16_t    subvers;
};

/// HCI encryption change event structure
/*@TRACE*/
struct hci_enc_chg_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///Encryption enabled information
    uint8_t     enc_stat;
};

/// HCI mode change event structure
/*@TRACE*/
struct hci_mode_chg_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t    conhdl;
    ///Current mode
    uint8_t    cur_mode;
    /// Interval
    uint16_t    interv;
};

/// HCI simple pairing complete event structure
/*@TRACE*/
struct hci_sp_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Bluetooth Device address
    struct bd_addr      bd_addr;
};

/// HCI Authentication complete event structure
/*@TRACE*/
struct hci_auth_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t    conhdl;
};

/// HCI change connection link key complete event structure
/*@TRACE*/
struct hci_chg_con_lk_cmp_evt
{
    ///Status
    uint8_t status;
    ///Connection handle
    uint16_t    conhdl;
};

/// HCI encryption key refresh complete event structure
/*@TRACE*/
struct hci_enc_key_refresh_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t    conhdl;
};

/// HCI master link key complete event structure
/*@TRACE*/
struct hci_master_lk_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t    conhdl;
    ///Key flag
    uint8_t key_flag;
};
/// HCI synchronous link connection complete event structure
/*@TRACE*/
struct hci_sync_con_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t    conhdl;
    ///BD address
    struct bd_addr bd_addr;
    ///Link type
    uint8_t lk_type;
    ///Transmit interval
    uint8_t tx_int;
    ///Retransmission window
    uint8_t ret_win;
    ///rx packet length
    uint16_t rx_pkt_len;
    ///tx packet length
    uint16_t tx_pkt_len;
    ///Air mode
    uint8_t air_mode;

};

/// HCI synchronous connection change event structure
/*@TRACE*/
struct hci_sync_con_chg_evt
{
    ///Status for command reception
    uint8_t status;
    ///Synchronous Connection handle
    uint16_t    sync_conhdl;
    ///Transmit interval
    uint8_t tx_int;
    ///Retransmission window
    uint8_t ret_win;
    ///rx packet length
    uint16_t rx_pkt_len;
    ///tx packet length
    uint16_t tx_pkt_len;
};

/// HCI connection packet type change event structure
/*@TRACE*/
struct hci_con_pkt_type_chg_evt
{
    ///Status for command reception
    uint8_t status;
    ///Synchronous Connection handle
    uint16_t    sync_conhdl;
    ///Synchronous packet type
    uint16_t    pkt_type;
};

/// HCI link supervision timeout change event structure
/*@TRACE*/
struct hci_link_supv_to_chg_evt
{
    ///Connection handle
    uint16_t    conhdl;
    ///Link supervision timeout
    uint16_t    lsto_val;
};

/// HCI link key request event structure
/*@TRACE*/
struct hci_lk_req_evt
{
    ///BD address
    struct bd_addr bd_addr;
};

/// HCI encryption key refresh event structure
/*@TRACE*/
struct hci_enc_key_refresh_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t    conhdl;
};

/// HCI connection request event structure
/*@TRACE*/
struct hci_con_req_evt
{
    ///BD address
    struct bd_addr bd_addr;
    ///Class of device
    struct devclass classofdev;
    ///link type
    uint8_t lk_type;
};

/// HCI quality of service violation event structure
/*@TRACE*/
struct hci_qos_viol_evt
{
    ///Connection handle
    uint16_t conhdl;
};

/// HCI io capability response event structure
/*@TRACE*/
struct hci_io_cap_rsp_evt
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///IO capability
    uint8_t io_capa;
    ///OOB data present
    uint8_t oob_data_pres;
    ///Authentication requirements
    uint8_t auth_req;

};

/// HCI IO capability response event structure
/*@TRACE*/
struct hci_io_cap_req_evt
{
    ///BdAddr
    struct bd_addr  bd_addr;
};

/// HCI Return link keys event structure
/*@TRACE*/
struct hci_return_link_keys_evt
{
    ///Number of Keys
    uint8_t num_keys;
    ///BdAddr
    struct bd_addr  bd_addr;
    ///Key
    struct ltk      key;
};

/// HCI pin code request event structure
/*@TRACE*/
struct hci_pin_code_req_evt
{
    ///BdAddr
    struct bd_addr  bd_addr;
};

/// HCI user passkey request event structure
/*@TRACE*/
struct hci_user_passkey_req_evt
{
    ///BdAddr
    struct bd_addr  bd_addr;
};

/// HCI user passkey notification event structure
/*@TRACE*/
struct hci_user_passkey_notif_evt
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///Passkey
    uint32_t passkey;
};

/// HCI remote OOB data request event structure
/*@TRACE*/
struct hci_rem_oob_data_req_evt
{
    ///BdAddr
    struct bd_addr  bd_addr;
};

/// HCI user confirmation request event structure
/*@TRACE*/
struct hci_user_cfm_req_evt
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///Passkey
    uint32_t passkey;
};

/// HCI keypress notification event structure
/*@TRACE*/
struct hci_keypress_notif_evt
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///type
    uint8_t type;
};

/// HCI link key notification event structure
/*@TRACE*/
struct hci_lk_notif_evt
{
    ///BdAddr
    struct bd_addr  bd_addr;
    ///Key
    struct ltk  key;
    ///type
    uint8_t key_type;
};

/// HCI SAM status change event strucutre
struct hci_sam_status_change_evt
{
    ///Connection handle
    uint16_t    conhdl;
    ///Local SAM index
    uint8_t     loc_idx;
    ///Local SAM TX availability
    uint8_t     loc_tx_av;
    ///Local SAM RX availability
    uint8_t     loc_rx_av;
    ///Remote SAM index
    uint8_t     rem_idx;
    ///Remote SAM TX availability
    uint8_t     rem_tx_av;
    ///Remote SAM RX availability
    uint8_t     rem_rx_av;
};


/*
 * HCI LE META EVENTS PARAMETERS
 ****************************************************************************************
 */


// LE event structures

/// HCI command complete event structure for the Read Local Supported Features
/*@TRACE*/
struct hci_le_rd_local_supp_feats_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t             status;
    ///Local LE supported features
    struct le_features  feats;
};

/// HCI command complete event structure for the Read Advertising Channel Tx Power Command
/*@TRACE
 * hci_le_rd_adv_chnl_tx_pw_cmd_cmp_evt = hci_rd_adv_chnl_tx_pw_cmd_cmp_evt*/
struct hci_rd_adv_chnl_tx_pw_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t     status;
    ///Advertising channel Tx power level
    int8_t     adv_tx_pw_lvl;
};

///HCI command complete event structure for the Read White List Size Command
/*@TRACE*/
struct hci_le_rd_wlst_size_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t     status;
    ///White List size
    uint8_t     wlst_size;
};

///HCI command complete event structure for the Read Buffer Size Command
/*@TRACE*/
struct hci_le_rd_buf_size_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t     status;
    ///ACL data packet length that can be sent from host to controller
    uint16_t    hc_data_pk_len;
    ///Total number of ACL data packets that can be sent from host to controller.
    uint8_t     hc_tot_nb_data_pkts;
};

///HCI command complete event structure for LE Rand Command
/*@TRACE*/
struct hci_le_rand_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t             status;
    ///Random number
    struct rand_nb      nb;
};

///HCI command complete event structure for Read Supported States Command
/*@TRACE*/
struct hci_le_rd_supp_states_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t             status;
    ///LE supported states response
    struct le_states    states;
};

///HCI command complete event structure for Test End
/*@TRACE
 *  * hci_le_test_end_cmd_cmp_evt = hci_test_end_cmd_cmp_evt*/
struct hci_test_end_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t             status;
    ///Number of RX packets - null if TX test was the ended one
    uint16_t            nb_packet_received;
};

///HCI LE Encrypt complete event structure
/*@TRACE*/
struct hci_le_enc_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t status;
    ///Encrypted data to return to command source.
    uint8_t encrypted_data[ENC_DATA_LEN];
};

#if BLE_EMB_PRESENT || BLE_HOST_PRESENT
///HCI LE advertising report event structure
/*@TRACE*/
struct hci_le_adv_report_evt
{
    ///LE Subevent code
    uint8_t             subcode;
    ///Number of advertising reports in this event
    uint8_t             nb_reports;
    ///Advertising reports structures array
    struct adv_report   adv_rep[BLE_ADV_REPORTS_MAX];
};

///HCI LE extended advertising report event structure
/*@TRACE*/
struct hci_le_ext_adv_report_evt
{
    ///LE Subevent code
    uint8_t             subcode;
    ///Number of advertising reports in this event
    uint8_t             nb_reports;
    ///Paramaters for each report
    struct ext_adv_report adv_rep[BLE_ADV_REPORTS_MAX];
};

///HCI LE periodic advertising sync established event structure
/*@TRACE
 * hci_le_periodic_adv_sync_est_evt = hci_le_per_adv_sync_est_evt*/
struct hci_le_per_adv_sync_est_evt
{
    ///LE Subevent code
    uint8_t             subcode;
    /// Status of the advertising sync
    uint8_t             status;
    /// Sync Handle to be used
    uint16_t            sync_handle;
    /// Advertising SID
    uint8_t             adv_sid;
    ///Advertising address type: public/random
    uint8_t             adv_addr_type;
    ///Advertising address value
    struct bd_addr      adv_addr;
    /// Advertiser PHY (@enum le_phy_value)
    uint8_t             phy;
    /// Advertising interval (Time=N*1.25ms)
    uint16_t            interval;
    /// Advertiser clock accuracy (@see enum SCA)
    uint8_t             adv_ca;
};

///HCI LE periodic advertising report event structure
/*@TRACE
 * hci_le_periodic_adv_report_evt = hci_le_per_adv_report_evt*/
struct hci_le_per_adv_report_evt
{
    ///LE Subevent code
    uint8_t             subcode;
    /// Sync Handle to be used
    uint16_t            sync_handle;
    /// Tx Power
    int8_t              tx_power;
    /// RSSI
    int8_t              rssi;
    /// CTE type
    uint8_t             cte_type;
    /// Data Status
    uint8_t             status;
    ///Data length in advertising packet
    uint8_t             data_len;
    ///Data of advertising packet
    uint8_t             data[PER_ADV_DATA_MAX_LEN];
};

///HCI LE periodic advertising sync lost event structure
/*@TRACE
 * hci_le_periodic_adv_sync_lost_evt = hci_le_per_adv_sync_lost_evt*/
struct hci_le_per_adv_sync_lost_evt
{
    ///LE Subevent code
    uint8_t             subcode;
    /// Sync Handle to be used
    uint16_t            sync_handle;
};

///HCI LE scan timeout event structure
/*@TRACE*/
struct hci_le_scan_timeout_evt
{
    ///LE Subevent code
    uint8_t             subcode;
};

#endif //BLE_EMB_PRESENT || BLE_HOST_PRESENT

/// HCI command complete event structure for Read Channel Map Command
/*@TRACE*/
struct hci_le_rd_ch_map_cmd_cmp_evt
{
    ///Status of command reception
    uint8_t            status;
    ///Connection handle
    uint16_t           conhdl;
    ///Channel map
    struct le_ch_map ch_map;
};

/// HCI command complete event structure for Long Term Key Request Reply Command
/*@TRACE*/
struct hci_le_ltk_req_reply_cmd_cmp_evt
{
    ///Status of command reception
    uint8_t        status;
    ///Connection handle
    uint16_t       conhdl;
};

/// HCI command complete event structure for Long Term Key Request Negative Reply Command
/*@TRACE*/
struct hci_le_ltk_req_neg_reply_cmd_cmp_evt
{
    ///Status of command reception
    uint8_t        status;
    ///Connection handle
    uint16_t       conhdl;
};

/// HCI command complete event structure for LE Read Suggested Default Data Length Command
/*@TRACE*/
struct hci_le_rd_suggted_dft_data_len_cmd_cmp_evt
{
    ///Status of command reception
    uint8_t        status;
    ///Host's suggested value for the Controller's maximum transmitted number of payload octets
    uint16_t       suggted_max_tx_octets;
    ///Host's suggested value for the Controller's maximum packet transmission time
    uint16_t       suggted_max_tx_time;
};
/// HCI command complete event structure for LE Read Maximum Data Length Command
/*@TRACE*/
struct hci_le_rd_max_data_len_cmd_cmp_evt
{
    ///Status of command reception
    uint8_t        status;
    ///Maximum number of payload octets that the local Controller supports for transmission
    uint16_t       suppted_max_tx_octets;
    ///Maximum time, in microseconds, that the local Controller supports for transmission
    uint16_t       suppted_max_tx_time;
    ///Maximum number of payload octets that the local Controller supports for reception
    uint16_t       suppted_max_rx_octets;
    ///Maximum time, in microseconds, that the local Controller supports for reception
    uint16_t       suppted_max_rx_time;
};

/// HCI LE Read Peer Resolvable Address Command Complete Event
/*@TRACE*/
struct hci_le_rd_peer_rslv_addr_cmd_cmp_evt
{
    ///Status
    uint8_t status;
    /// Peer Resolvable Address
    struct bd_addr      peer_rslv_addr;
};

/// HCI LE Read Local Resolvable Address Command Complete Event
/*@TRACE*/
struct hci_le_rd_loc_rslv_addr_cmd_cmp_evt
{
    ///Status
    uint8_t status;
    /// Local Resolvable Address
    struct bd_addr      loc_rslv_addr;
};

/// HCI LE Read Resolving List Size Command Complete Event
/*@TRACE*/
struct hci_le_rd_rslv_list_size_cmd_cmp_evt
{
    ///Status
    uint8_t status;
    /// Resolving List Size
    uint8_t             size;
};


/// HCI write authenticated payload timeout command
/*@TRACE*/
struct hci_wr_auth_payl_to_cmd
{
    ///Connection handle
    uint16_t     conhdl;
    ///Authenticated payload timeout (N*10ms)
    uint16_t     auth_payl_to;
};

/// HCI command complete event structure for the Write Authenticated Payload Timeout Command
/*@TRACE*/
struct hci_wr_auth_payl_to_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t     status;
    ///Connection handle
    uint16_t     conhdl;
};

/// HCI command complete event structure for HCI LE Connection Update Command
/*@TRACE*/
struct hci_le_con_update_cmp_evt
{
    ///LE Subevent code
    uint8_t             subcode;
    ///Status of received command
    uint8_t             status;
    ///Connection handle
    uint16_t            conhdl;
    ///Connection interval value
    uint16_t            con_interval;
    ///Connection latency value
    uint16_t            con_latency;
    ///Supervision timeout
    uint16_t            sup_to;
};

/// HCI command complete event structure for create connection
/*@TRACE*/
struct hci_le_con_cmp_evt
{
    ///LE Subevent code
    uint8_t             subcode;
    ///Status of received command
    uint8_t             status;
    ///Connection handle
    uint16_t            conhdl;
    ///Device role - 0=Master/ 1=Slave
    uint8_t             role;
    ///Peer address type - 0=public/1=random
    uint8_t             peer_addr_type;
    ///Peer address
    struct bd_addr      peer_addr;
    ///Connection interval
    uint16_t            con_interval;
    ///Connection latency
    uint16_t            con_latency;
    ///Link supervision timeout
    uint16_t            sup_to;
    ///Master clock accuracy
    uint8_t             clk_accuracy;
};

/// HCI LE read remote used feature command parameters structure
/*@TRACE*/
struct hci_le_rd_rem_feats_cmd
{
    ///Connection handle
    uint16_t            conhdl;
};

/// HCI command complete event structure for HCI LE read remote feature Command
/*@TRACE*/
struct hci_le_rd_rem_feats_cmd_cmp_evt
{
    ///LE Subevent code
    uint8_t             subcode;
    ///Status of received command
    uint8_t             status;
    ///Connection handle
    uint16_t            conhdl;
    ///Le Features
    struct le_features  le_feats;
};

/// HCI command structure for the read transmit power level command
/*@TRACE*/
struct hci_rd_tx_pwr_lvl_cmd
{
    ///Connection handle
    uint16_t    conhdl;
    ///Power Level type: current or maximum
    uint8_t     type;
};

/// HCI command complete event structure for the read transmit power level command
/*@TRACE*/
struct hci_rd_tx_pwr_lvl_cmd_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///Value of TX power level
    int8_t     tx_pow_lvl;
};

/// HCI read remote information version command parameters structure
/*@TRACE*/
struct hci_rd_rem_ver_info_cmd
{
    ///Connection handle
    uint16_t    conhdl;
};

/// HCI LE remote connection parameter request event
/*@TRACE*/
struct hci_le_rem_con_param_req_evt
{
    ///LE Subevent code
    uint8_t             subcode;
    ///Connection handle
    uint16_t            conhdl;
    ///Interval_Min
    uint16_t            interval_min;
    ///Interval_Max
    uint16_t            interval_max;
    ///Latency
    uint16_t            latency;
    ///Timeout
    uint16_t            timeout;
};


/// HCI command complete event structure for enhance create connection
/*@TRACE*/
struct hci_le_enh_con_cmp_evt
{
    ///LE Subevent code
    uint8_t             subcode;
    ///Status of received command
    uint8_t             status;
    ///Connection handle
    uint16_t            conhdl;
    ///Device role - 0=Master/ 1=Slave
    uint8_t             role;
    ///Peer address type - 0=public/1=random
    uint8_t             peer_addr_type;
    ///Peer address
    struct bd_addr      peer_addr;
    ///Local Resolvable Private Address
    struct bd_addr      loc_rslv_priv_addr;
    ///Peer Resolvable Private Address
    struct bd_addr      peer_rslv_priv_addr;
    ///Connection interval
    uint16_t            con_interval;
    ///Connection latency
    uint16_t            con_latency;
    ///Link supervision timeout
    uint16_t            sup_to;
    ///Master clock accuracy
    uint8_t             clk_accuracy;
};

struct hci_generate_dhkey_cmp_evt
{
    ///LE Subevent code
    uint8_t             subcode;
    ///Status of received command
    uint8_t             status;
    /// The 32 byte Diffie Helman Key
    uint8_t             dh_key[32];
};

struct hci_rd_local_p256_public_key_cmp_evt
{
    ///LE Subevent code
    uint8_t             subcode;
    ///Status of received command
    uint8_t             status;
    /// The 32 byte Diffie Helman Key
    uint8_t             public_key[64];

};

#if BLE_EMB_PRESENT || BLE_HOST_PRESENT
/// HCI LE Direct Advertising Report Event
/*@TRACE*/
struct hci_le_dir_adv_rep_evt
{
    ///LE Subevent code
    uint8_t                 subcode;
    ///Number of reports
    uint8_t                 nb_reports;
    ///Direct Advertising reports structures array
    struct dir_adv_report   adv_rep[BLE_ADV_REPORTS_MAX];
};
#endif //BLE_EMB_PRESENT || BLE_HOST_PRESENT

/// Connected LE event
struct hci_le_con_evt
{
    ///LE Subevent code
    uint8_t             subcode;
    ///Connection handle
    uint16_t            conhdl;
};

/// HCI command complete event structure for HCI LE read remote used feature Command
/*@TRACE*/
struct hci_le_ltk_request_evt
{
    ///LE Subevent code
    uint8_t             subcode;
    ///Connection handle
    uint16_t            conhdl;
    ///Random number
    struct rand_nb      rand;
    ///Encryption diversifier
    uint16_t            ediv;
};

/// HCI LE META event LE Data Length Change Event
/*@TRACE*/
struct hci_le_data_len_chg_evt
{
    ///LE Subevent code
    uint8_t             subcode;
    ///Connection handle
    uint16_t            conhdl;
    ///The maximum number of payload octets in TX
    uint16_t            max_tx_octets;
    ///The maximum time that the local Controller will take to TX
    uint16_t            max_tx_time;
    ///The maximum number of payload octets in RX
    uint16_t            max_rx_octets;
    ///The maximum time that the local Controller will take to RX
    uint16_t            max_rx_time;
};


/// HCI Synchronization Train Received Event
/*@TRACE*/
struct hci_sync_train_rec_evt
{
    /// Status
    uint8_t            status;
    /// BD_ADDR
    struct bd_addr     bd_addr;
    /// Clock_Offset (28 bits) - (CLKNslave - CLK) modulo 2^28
    uint32_t           clock_offset;
    /// AFH_Channel_Map
    struct bt_ch_map    afh_ch_map;
    /// LT_ADDR
    uint8_t            lt_addr;
    /// Next_Broadcast_Instant (28 bits)
    uint32_t next_bcst_instant;
    /// Connectionless_Slave_Broadcast_Interval (in slots)
    uint16_t csb_int;
    /// Service_Data
    uint8_t service_data;
};

/// HCI Connectionless Slave Broadcast Receive Event
/*@TRACE*/
struct hci_con_slv_bcst_rec_evt
{
    /// BD_ADDR
    struct bd_addr     bd_addr;
    /// LT_ADDR
    uint8_t            lt_addr;
    /// CLK (28 bits)
    uint32_t           clk;
    /// Offset (28 bits) - (CLKNslave - CLK) modulo 2^28
    uint32_t           offset;
    /// Receive Status
    uint8_t            receive_status;
    /// Fragment
    uint8_t fragment;
    /// Data_Length (in bytes)
    uint8_t data_length;
    /// Data
    uint8_t data[__ARRAY_EMPTY];
};

/// HCI Connectionless Slave Broadcast Timeout Event
/*@TRACE*/
struct hci_con_slv_bcst_to_evt
{
    /// BD_ADDR
    struct bd_addr     bd_addr;
    /// LT_ADDR
    uint8_t            lt_addr;
};

/// HCI Connectionless Slave Broadcast Channel Map Change Event
/*@TRACE*/
struct hci_con_slv_bcst_ch_map_chg_evt
{
    /// Channel_Map
    struct bt_ch_map    ch_map;
};

/*@TRACE*/
struct hci_le_gen_dhkey_cmp_evt
{
    ///LE Subevent code
    uint8_t      subcode;
    uint8_t      status;
    uint8_t      dh_key[32];
};

/*@TRACE*/
struct hci_le_rd_loc_p256_pub_key_cmp_evt
{
    ///LE Subevent code
    uint8_t       subcode;
    uint8_t       status;
    t_public_key  public_key;
};


/// HCI command structure for the LE receiver test v3 command
/*@TRACE*/
struct hci_le_rx_test_v3_cmd
{
    /// RX channel, range: 0x00 to 0x27
    uint8_t     rx_channel;
    /// PHY (@enum le_phy_value)
    uint8_t     phy;
    /// Modulation index (0: standard | 1: stable)
    uint8_t     mod_idx;
    /// Expected CTE length in 8us units, range: 0x02 to 0x14
    uint8_t     exp_cte_len;
    /// Expected CTE type (0: AOA | 1: AOD-1us | 2: AOD-2us)
    uint8_t     exp_cte_type;
    /// Slot durations (1: 1 us | 2: 2 us)
    uint8_t     slot_dur;
    /// Length of switching pattern (number of antenna IDs in the pattern), range: 0x02 to 0x4B
    uint8_t     switching_pattern_len;
    /// Antenna IDs
    uint8_t     antenna_id[MAX_SWITCHING_PATTERN_LEN];
};

/// HCI command structure for the LE transmitter test v3 command
/*@TRACE*/
struct hci_le_tx_test_v3_cmd
{
    /// TX channel, range: 0x00 to 0x27
    uint8_t     tx_channel;
    /// Length of test data in bytes, range: 0x00 to 0xFF
    uint8_t     test_data_len;
    /**
     * Packet payload
     * 0x00 PRBS9 sequence "11111111100000111101" (in transmission order) as described in [Vol 6] Part F, Section 4.1.5
     * 0x01 Repeated "11110000" (in transmission order) sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x02 Repeated "10101010" (in transmission order) sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x03 PRBS15 sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x04 Repeated "11111111" (in transmission order) sequence
     * 0x05 Repeated "00000000" (in transmission order) sequence
     * 0x06 Repeated "00001111" (in transmission order) sequence
     * 0x07 Repeated "01010101" (in transmission order) sequence
     * 0x08-0xFF Reserved for future use
     */
    uint8_t     pkt_payl;
    /// PHY (@enum le_phy_value)
    uint8_t     phy;
    /// CTE length (in 8us unit)
    uint8_t     cte_len;
    /// CTE type (0: AOA | 1: AOD-1us | 2: AOD-2us)
    uint8_t     cte_type;
    /// Length of switching pattern (number of antenna IDs in the pattern)
    uint8_t     switching_pattern_len;
    /// Antenna IDs
    uint8_t     antenna_id[MAX_SWITCHING_PATTERN_LEN];
};

/// HCI command structure for the LE transmitter test v4 command
/*@TRACE*/
struct hci_le_tx_test_v4_cmd
{
    /// TX channel, range: 0x00 to 0x27
    uint8_t     tx_channel;
    /// Length of test data in bytes, range: 0x00 to 0xFF
    uint8_t     test_data_len;
    /**
     * Packet payload
     * 0x00 PRBS9 sequence "11111111100000111101" (in transmission order) as described in [Vol 6] Part F, Section 4.1.5
     * 0x01 Repeated "11110000" (in transmission order) sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x02 Repeated "10101010" (in transmission order) sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x03 PRBS15 sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x04 Repeated "11111111" (in transmission order) sequence
     * 0x05 Repeated "00000000" (in transmission order) sequence
     * 0x06 Repeated "00001111" (in transmission order) sequence
     * 0x07 Repeated "01010101" (in transmission order) sequence
     * 0x08-0xFF Reserved for future use
     */
    uint8_t     pkt_payl;
    /// PHY (@enum le_phy_value)
    uint8_t     phy;
    /// CTE length (in 8us unit)
    uint8_t     cte_len;
    /// CTE type (0: AOA | 1: AOD-1us | 2: AOD-2us)
    uint8_t     cte_type;
    /// Length of switching pattern (number of antenna IDs in the pattern)
    uint8_t     switching_pattern_len;
    /// Antenna IDs
    uint8_t     antenna_id[MAX_SWITCHING_PATTERN_LEN];
    /// Transmit power level in dBm (0x7E: minimum | 0x7F: maximum | range: -127 to +20)
    int8_t     tx_pwr_lvl;
};

/// HCI command structure for the LE set connectionless CTE transmit parameters command
/*@TRACE*/
struct hci_le_set_conless_cte_tx_param_cmd
{
    /// Advertising handle
    uint8_t     adv_hdl;
    /// CTE length (in 8us unit)
    uint8_t     cte_len;
    /// CTE type (0: AOA | 1: AOD-1us | 2: AOD-2us)
    uint8_t     cte_type;
    /// CTE count (number of CTEs to transmit in each periodic advertising interval, range 0x01 to 0x10)
    uint8_t     cte_count;
    /// Length of switching pattern (number of antenna IDs in the pattern)
    uint8_t     switching_pattern_len;
    /// Antenna IDs
    uint8_t     antenna_id[MAX_SWITCHING_PATTERN_LEN];
};

/// HCI command structure for the LE set connectionless CTE transmit enable command
/*@TRACE*/
struct hci_le_set_conless_cte_tx_en_cmd
{
    /// Advertising handle
    uint8_t     adv_hdl;
    /// CTE enable
    uint8_t     cte_en;
};

/// HCI command structure for LE set connectionless IQ sampling enable command
/*@TRACE*/
struct hci_le_set_conless_iq_sampl_en_cmd
{
    /// Sync handle
    uint16_t    sync_hdl;
    /// Sampling enable
    uint8_t     sampl_en;
    /// Slot durations (1: 1us | 2: 2us)
    uint8_t     slot_dur;
    /// Max sampled CTEs
    uint8_t     max_sampl_cte;
    /// Length of switching pattern
    uint8_t     switching_pattern_len;
    /// Antenna IDs
    uint8_t     antenna_id[MAX_SWITCHING_PATTERN_LEN];
};

/// HCI command structure for the LE set connection CTE receive parameters command
/*@TRACE*/
struct hci_le_set_con_cte_rx_param_cmd
{
    /// Connection handle
    uint16_t    conhdl;
    /// Sampling enable
    uint8_t     sampl_en;
    /// Slot durations (1: 1us | 2: 2us)
    uint8_t     slot_dur;
    /// Length of switching pattern (number of antenna IDs in the pattern)
    uint8_t     switching_pattern_len;
    /// Antenna IDs
    uint8_t     antenna_id[MAX_SWITCHING_PATTERN_LEN];
};

/// HCI command structure for the LE set connection CTE transmit parameters command
/*@TRACE*/
struct hci_le_set_con_cte_tx_param_cmd
{
    /// Connection handle
    uint16_t    conhdl;
    /// CTE types (bit0: AOA | bit1: AOD-1us | bit2: AOD-2us)
    uint8_t     cte_types;
    /// Length of switching pattern (number of antenna IDs in the pattern)
    uint8_t     switching_pattern_len;
    /// Antenna IDs
    uint8_t     antenna_id[MAX_SWITCHING_PATTERN_LEN];
};

/// HCI command structure for the LE connection CTE request enable command
/*@TRACE*/
struct hci_le_con_cte_req_en_cmd
{
    /// Connection handle
    uint16_t    conhdl;
    /// Enable
    uint8_t     en;
    /// CTE request interval (in number of connection events)
    uint16_t    cte_req_intv;
    /// Requested CTE length (in 8us unit)
    uint8_t     req_cte_len;
    /// Requested CTE type (0: AOA | 1: AOD-1us | 2: AOD-2us)
    uint8_t     req_cte_type;
};

/// HCI command structure for the LE connection CTE response enable command
/*@TRACE*/
struct hci_le_con_cte_rsp_en_cmd
{
    /// Connection handle
    uint16_t    conhdl;
    /// Enable
    uint8_t     en;
};

/// HCI command structure for LE set connectionless IQ sampling enable command complete event
/*@TRACE*/
struct hci_le_set_conless_iq_sampl_en_cmd_cmp_evt
{
    /// Status
    uint8_t     status;
    /// Sync handle
    uint16_t    sync_hdl;
};

/// HCI command structure for LE read antenna information command complete event
/*@TRACE*/
struct hci_le_rd_antenna_inf_cmd_cmp_evt
{
    /// Status
    uint8_t     status;
    /// Supported switching sampling rates
    uint8_t     supp_switching_sampl_rates;
    /// Number of antennae
    uint8_t     antennae_num;
    /// Max length of switching pattern (number of antenna IDs in the pattern)
    uint8_t     max_switching_pattern_len;
    /// Max CTE length
    uint8_t     max_cte_len;
};

/// HCI command structure for the LE set periodic advertising receive enable command
/*@TRACE*/
struct hci_le_set_per_adv_rec_en_cmd
{
    /// Sync handle
    uint16_t sync_hdl;
    /// Enable (@see enum per_adv_rec_en)
    uint8_t en;
};
#if (BLE_PAST)
/// HCI command structure for the LE set periodic advertising receive enable command
/*@TRACE*/
struct hci_le_per_adv_sync_transf_cmd
{
    /// Connection handle
    uint16_t conhdl;
    /// Service data (value provided by the Host)
    uint16_t serv_data;
    /// Sync handle
    uint16_t sync_hdl;
};

/// HCI command structure for the LE Periodic Advertising Set Info Transfer command
/*@TRACE*/
struct hci_le_per_adv_set_info_transf_cmd
{
    /// Connection handle
    uint16_t conhdl;
    /// Service data (value provided by the Host)
    uint16_t serv_data;
    /// Advertising handle
    uint8_t adv_hdl;
};

/// HCI command structure for the LE Set Periodic Advertising Sync Transfer Parameters command
/*@TRACE*/
struct hci_le_set_per_adv_sync_transf_param_cmd
{
    /// Connection handle
    uint16_t  conhdl;
    /// Mode (@see enum per_adv_sync_info_rec_mode)
    uint8_t   mode;
    /// The number of periodic advertising packets that can be skipped after a successful receive
    uint16_t  skip;
    /// Sync timeout (Time=N*10ms)
    uint16_t  sync_to;
    /// CTE type (@see enum sync_cte_type)
    uint8_t   cte_type;
};

/// HCI command structure for the LE Set Default Periodic Advertising Sync Transfer Parameters command
/*@TRACE*/
struct hci_le_set_dft_per_adv_sync_transf_param_cmd
{
    /// Mode (@see enum per_adv_sync_info_rec_mode)
    uint8_t   mode;
    /// The number of periodic advertising packets that can be skipped after a successful receive
    uint16_t  skip;
    /// Sync timeout (Time=N*10ms)
    uint16_t  sync_to;
    /// CTE type (@see enum sync_cte_type)
    uint8_t   cte_type;
};

/// HCI command structure for the LE Periodic Advertising Sync Transfer Received event
/*@TRACE*/
struct hci_le_per_adv_sync_transf_rec_evt
{
    /// LE Subevent code
    uint8_t        subcode;
    /// Status
    uint8_t        status;
    /// Connection handle
    uint16_t       conhdl;
    /// Service data (value provided by the Host)
    uint16_t       serv_data;
    /// Sync handle
    uint16_t       sync_hdl;
    /// Advertising SID
    uint8_t        adv_sid;
    ///Advertising address type: public/random
    uint8_t        adv_addr_type;
    ///Advertising address value
    struct bd_addr adv_addr;
    /// Advertiser PHY (@enum le_phy_value)
    uint8_t        phy;
    /// Advertising interval (Time=N*1.25ms)
    uint16_t       interval;
    /// Advertiser clock accuracy (@see enum SCA)
    uint8_t        adv_ca;
};
#endif // (BLE_PAST)

/// HCI command structure for the LE enhanced read transmit power level command
/*@TRACE*/
struct hci_le_enh_rd_tx_pwr_lvl_cmd
{
    /// Connection handle
    uint16_t        conhdl;
    /// PHY (@see enum le_phy_pwr_value)
    uint8_t        phy;
};

/// HCI command structure for the LE enhanced read transmit power level command complete event
/*@TRACE*/
struct hci_le_enh_rd_tx_pwr_lvl_cmd_cmp_evt
{
    /// Status
    uint8_t     status;
    /// Connection handle
    uint16_t        conhdl;
    /// PHY (@see enum le_phy_pwr_value)
    uint8_t         phy;
    /// Current transmit power level (dBm)
    int8_t         curr_tx_pwr_lvl;
    /// Max transmit power level (dBm)
    int8_t         max_tx_pwr_lvl;
};

/// HCI command structure for the LE read remote transmit power level command
/*@TRACE*/
struct hci_le_rd_remote_tx_pwr_lvl_cmd
{
    /// Connection handle
    uint16_t        conhdl;
    /// PHY (@see le_phy_pwr_value)
    uint8_t         phy;
};

/// HCI command structure for the LE set path loss reporting parameters command
/*@TRACE*/
struct hci_le_set_path_loss_rep_param_cmd
{
    /// Connection handle
    uint16_t        conhdl;
    /// High threshold (dB)
    uint8_t         hi_thr;
    /// High hysteresis (dB)
    uint8_t         hi_hyst;
    /// Low threshold (dB)
    uint8_t         lo_thr;
    /// Low hysteresis (dB)
    uint8_t         lo_hyst;
    /// Min time spent (conn events)
    uint16_t        min_time;
};

/// HCI command structure for the LE set path loss reporting parameters command complete event
/*@TRACE*/
struct hci_le_set_path_loss_rep_param_cmd_cmp_evt
{
    /// Status
    uint8_t     status;
    /// Connection handle
    uint16_t        conhdl;
};

/// HCI command structure for the LE set path loss reporting enable command
/*@TRACE*/
struct hci_le_set_path_loss_rep_en_cmd
{
    /// Connection handle
    uint16_t        conhdl;
    /// Enable (@see enum pwr_report_en)
    uint8_t         en;
};

/// HCI command structure for the LE set path loss reporting enable command complete event
/*@TRACE*/
struct hci_le_set_path_loss_rep_en_cmd_cmp_evt
{
    /// Status
    uint8_t     status;
    /// Connection handle
    uint16_t        conhdl;
};

/// HCI command structure for the LE set transmit power reporting enable command
/*@TRACE*/
struct hci_le_set_tx_power_rep_en_cmd
{
    /// Connection handle
    uint16_t        conhdl;
    /// Local Enable (@see enum pwr_report_en)
    uint8_t         local_en;
    /// Remote Enable (@ see enum pwr_report_en)
    uint8_t         remote_en;
};

/// HCI command structure for the LE set transmit power reporting enable command complete event
/*@TRACE*/
struct hci_le_set_tx_power_rep_en_cmd_cmp_evt
{
    /// Status
    uint8_t     status;
    /// Connection handle
    uint16_t        conhdl;
};

/// HCI command structure for the LE Path Loss Threshold event
/*@TRACE*/
struct hci_le_path_loss_threshold_evt
{
    /// LE Subevent code
    uint8_t         subcode;
    /// Connection handle
    uint16_t        conhdl;
    /// Current path loss (dB)
    uint8_t         curr_path_loss;
    /// Zone entered (@see enum le_path_loss_zone)
    uint8_t         zone_entered;
};

/// HCI command strucutre for the LE Transmit Power Reporting event
/*@TRACE*/
struct hci_le_tx_power_rep_evt
{
    /// LE Subevent code
    uint8_t         subcode;
    /// Status
    uint8_t        status;
    /// Connection handle
    uint16_t        conhdl;
    /// Reason (@see enum pwr_report_reason)
    uint8_t         reason;
    /// PHY (@see enum le_phy_pwr_value)
    uint8_t         phy;
    /// Transmit Power level (dBm)
    int8_t         tx_pwr;
    /// Transmit Power level flags (@see enum pwr_ctrl_flags)
    uint8_t         flags;
    /// Delta (dB)
    int8_t         delta;
};

/*
 * HCI VENDOR SPECIFIC COMMANDS PARAMETERS
 ****************************************************************************************
 */

/// Buffer structure
/*@TRACE*/
struct buffer_tag
{
    /// length of buffer
    uint8_t length;
    /// data of 128 bytes length
    uint8_t data[128];
};

/// Common structure for Command Complete Event of HCI Debug Read Memory/Flash/Param complete event parameters - vendor specific
/*@TRACE
 * hci_dbg_rd_mem_cmd_cmp_evt = hci_dbg_basic_rd_data_cmd_cmp_evt
 * hci_dbg_rd_flash_cmd_cmp_evt = hci_dbg_basic_rd_data_cmd_cmp_evt
 * hci_dbg_rd_par_cmd_cmp_evt = hci_dbg_basic_rd_data_cmd_cmp_evt
 * */
struct hci_dbg_basic_rd_data_cmd_cmp_evt
{
    ///Status
    uint8_t status;
    ///buffer structure to return
    struct buffer_tag buf;
};

///HCI Debug read memory variable command parameters - vendor specific
/*@TRACE*/
struct hci_dbg_rd_mem_cmd
{
    ///Start address to read
    uint32_t start_addr;
    ///Access size
    uint8_t type;
    ///Length to read
    uint8_t length;
};

///HCI Debug write memory variable command parameters - vendor specific
/*@TRACE*/
struct hci_dbg_wr_mem_cmd
{
    ///Start address to read
    uint32_t start_addr;
    ///Access size
    uint8_t type;
    ///buffer structure to return
    struct buffer_tag buf;
};

///HCI Debug delete parameter command parameters - vendor specific
/*@TRACE
 * hci_dbg_del_par_cmd = hci_dbg_del_param_cmd*/
struct hci_dbg_del_param_cmd
{
    ///Parameter tag
    uint16_t param_tag;
};

///HCI Debug erase flash command parameters - vendor specific
/*@TRACE*/
struct hci_dbg_er_flash_cmd
{
    ///Flash type
    uint8_t flashtype;
    ///Start offset address
    uint32_t startoffset;
    ///Size to erase
    uint32_t size;
};

///HCI Debug write flash command parameters - vendor specific
/*@TRACE*/
struct hci_dbg_wr_flash_cmd
{
    ///Flash type
    uint8_t flashtype;
    ///Start offset address
    uint32_t startoffset;
    ///buffer structure
    struct buffer_tag buf;
};

///HCI Debug read flash command parameters - vendor specific
/*@TRACE*/
struct hci_dbg_rd_flash_cmd
{
    ///Flash type
    uint8_t flashtype;
    ///Start offset address
    uint32_t startoffset;
    ///Size to read
    uint8_t size;
};

///HCI Debug read parameter command parameters - vendor specific
/*@TRACE*/
struct hci_dbg_rd_par_cmd
{
    ///Parameter tag
    uint16_t param_tag;
};

///HCI Debug read parameters command parameters - vendor specific
/*@TRACE*/
struct hci_dbg_wr_par_cmd
{
    ///Parameter tag
    uint16_t param_tag;
    ///Structure buffer
    struct buffer_tag buf;
};

#if CRYPTO_UT
///HCI Debug Test cryptographic functions command parameters - vendor specific
/*@TRACE*/
struct hci_dbg_test_crypto_func_cmd
{
    /// Id of Function to be tested
    uint8_t function;
    /// Structure buffer
    struct buffer_tag buf;
};
#endif //CRYPTO_UT

#if (BT_READ_PICONET_CLOCK)
struct hci_vs_rd_piconet_clock_cmd
{
    ///Connection handle of the link
    uint16_t conhdl;
    ///Pulse trigger management
    uint8_t trig_pulse;
    ///Clock type (0=native clock, 1=piconet clock of conhndl)
    uint8_t clk_type;
};

struct hci_vs_rd_piconet_clock_cmd_cmp_evt
{
    /// Status
    uint8_t status;
    ///Connection handle of the link
    uint16_t conhdl;
    /// Local timestamp(position in half slots)
    uint32_t loc_clk_hslt;
    /// Local timestamp fractional part (in half us)
    uint16_t loc_clk_hus;
    /// Piconet timestamp(position in half slots)
    uint32_t pic_clk_off_hslt;
    /// Piconet bit offset (in half us)
    uint16_t pic_bit_off_hus;
};
#endif //(BT_READ_PICONET_CLOCK)

#if (BT_HCI_TEST_MODE)
/// HCI command structure for the LE receiver test v3 command
/*@TRACE*/
struct hci_vs_rx_test_cmd
{
    /// RX channel, range: 0 to 79
    uint8_t     rx_channel;
    /// Packet type
    uint8_t     pkt_type;
};

/// HCI command structure for the LE transmitter test v4 command
/*@TRACE*/
struct hci_vs_tx_test_cmd
{
    /// TX channel, range: 0 to 79
    uint8_t     tx_channel;
    /// Length of test data in bytes, range: 0 to 1021
    uint16_t     test_data_len;
    /**
     * Packet payload
     * 0x00 PRBS9 sequence "11111111100000111101" (in transmission order) as described in [Vol 6] Part F, Section 4.1.5
     * 0x01 Repeated "11110000" (in transmission order) sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x02 Repeated "10101010" (in transmission order) sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x03 PRBS15 sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x04 Repeated "11111111" (in transmission order) sequence
     * 0x05 Repeated "00000000" (in transmission order) sequence
     * 0x06 Repeated "00001111" (in transmission order) sequence
     * 0x07 Repeated "01010101" (in transmission order) sequence
     * 0x08-0xFF Reserved for future use
     */
    uint8_t     pkt_payl;
    /// Packet_Type
    uint8_t     pkt_type;
    /// Transmit power level in dBm (0x7E: minimum | 0x7F: maximum | range: -127 to +20)
    int8_t     tx_pwr_lvl;
};

///HCI command complete event structure for Test End
/*@TRACE*/
struct hci_vs_test_end_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t             status;
    ///Number of RX/TX packets
    uint16_t            nb_packets;
};
#endif //(BT_HCI_TEST_MODE)

#if RW_DEBUG
///HCI Debug Test scheduling planner set function command parameters - vendor specific
/*@TRACE*/
struct hci_dbg_test_sch_plan_set_cmd
{
    /// Interval (in half-slots)
    uint32_t interval;
    /// Offset (in half-slots)
    uint32_t offset;
    /// Minimum duration (in half-slots)
    uint32_t duration_min;
    /// Maximum duration (in half-slots)
    uint32_t duration_max;
    /// Connection handle
    uint16_t conhdl;
    /// Reference connection handle, the primary activity on which the new activity refers to (e.g. ACL connection for a synchronous connection)
    uint16_t conhdl_ref;
    /// Margin to consider around the activity events, when not related to the same activity (in half-slots)
    uint16_t margin;
    /// Indicate activity is movable
    bool mobility_level;
};

///HCI Debug Test scheduling planner set function command complete event parameters - vendor specific
/*@TRACE*/
struct hci_dbg_test_sch_plan_set_cmd_cmp_evt
{
    /// Status
    uint8_t status;
    /// Connection handle of the moved activity
    uint16_t moved_conhdl;
};

///HCI Debug Test scheduling planner remove function command parameters - vendor specific
/*@TRACE*/
struct hci_dbg_test_sch_plan_rem_cmd
{
    /// Connection handle
    uint16_t conhdl;
};

///HCI Debug Test scheduling planner check function command parameters - vendor specific
/*@TRACE*/
struct hci_dbg_test_sch_plan_chk_cmd
{
    /// Interval (in half-slots)
    uint32_t interval;
    /// Offset (in half-slots)
    uint32_t offset;
    /// Minimum duration (in half-slots)
    uint32_t duration_min;
    /// Connection handle
    uint16_t conhdl;
    /// Reference connection handle, the primary activity on which the new activity refers to (e.g. ACL connection for a synchronous connection)
    uint16_t conhdl_ref;
    /// Margin to consider around the activity events, when not related to the same activity (in half-slots)
    uint16_t margin;
};

///HCI Debug Test scheduling planner request function command parameters - vendor specific
/*@TRACE*/
struct hci_dbg_test_sch_plan_req_cmd
{
    /// Minimum interval (in half-slots)
    uint32_t interval_min;
    /// Maximum interval (in half-slots)
    uint32_t interval_max;
    /// Minimum duration (in half-slots)
    uint32_t duration_min;
    /// Maximum duration (in half-slots)
    uint32_t duration_max;
    /// Connection handle
    uint16_t conhdl;
    /// Reference connection handle, the primary activity on which the new activity refers to (e.g. ACL connection for a synchronous connection)
    uint16_t conhdl_ref;
    /// Margin to consider around the activity events, when not related to the same activity (in half-slots)
    uint16_t margin;
    /// Period (in half-slots)
    uint8_t period;
};

///HCI Debug Test scheduling planner request function command complete event parameters - vendor specific
/*@TRACE*/
struct hci_dbg_test_sch_plan_req_cmd_cmp_evt
{
    /// Status
    uint8_t status;
    /// Interval (in half-slots)
    uint32_t interval;
    /// Minimum offset (in half-slots)
    uint32_t offset_min;
    /// Maximum offset (in half-slots)
    uint32_t offset_max;
};
#endif //RW_DEBUG

///HCI Debug Read Kernel Statistics complete event parameters - vendor specific
/*@TRACE*/
struct hci_dbg_rd_ke_stats_cmd_cmp_evt
{
    ///Status
    uint8_t status;
    ///Max message sent
    uint32_t max_msg_sent;
    ///Max message saved
    uint32_t max_msg_saved;
    ///Max timer used
    uint32_t max_timer_used;
    ///Max heap used
    uint32_t max_heap_used;
    ///Max stack used
    uint32_t max_stack_used;
};


/// HCI Debug Read information about memory usage. - vendor specific
/*@TRACE*/
struct hci_dbg_rd_mem_info_cmd_cmp_evt
{
    ///Status
    uint8_t status;
    /// memory size currently used into each heaps.
    uint16_t mem_used[KE_MEM_BLOCK_MAX];
    /// peak of memory usage measured
    uint32_t max_mem_used;
};

///HCI Debug identify Flash command complete event parameters - vendor specific
/*@TRACE*/
struct hci_dbg_id_flash_cmd_cmp_evt
{
    ///Status
    uint8_t status;
    ///Flash identity
    uint8_t flash_id;
};

///HCI Debug RF Register read command
/*@TRACE*/
struct hci_dbg_rf_reg_rd_cmd
{
    /// register address
    uint16_t addr;
};

///HCI Debug RF Register read command complete event
/*@TRACE*/
struct hci_dbg_rf_reg_rd_cmd_cmp_evt
{
    /// status
    uint8_t status;
    /// register address
    uint16_t addr;
    /// register value
    uint32_t value;
};

///HCI Debug RF Register write command
/*@TRACE*/
struct hci_dbg_rf_reg_wr_cmd
{
    /// register address
    uint16_t addr;
    /// register value
    uint32_t value;
};

///HCI Debug RF Register write command complete event
/*@TRACE*/
struct hci_dbg_rf_reg_wr_cmd_cmp_evt
{
    /// status
    uint8_t status;
    /// address
    uint16_t addr;
};

///HCI Debug platform reset command parameters - vendor specific
/*@TRACE*/
struct hci_dbg_plf_reset_cmd
{
    /// reason
    uint8_t reason;
};

#if (RW_DEBUG && BT_EMB_PRESENT)
/// Send LMP Packets
/*@TRACE*/
struct hci_dbg_bt_send_lmp_cmd
{
    /// Connection handle
    uint16_t conhdl;
    ///buffer structure to return
    struct buffer_tag buf;
};

/// Discard LMP Packets
/*@TRACE*/
struct hci_dbg_bt_discard_lmp_en_cmd
{
    /// Connection handle
    uint16_t conhdl;
    /// Enable/Disable LMP discard (0: disable / 1: enable)
    uint8_t enable;
};

/// Set local clock
/*@TRACE*/
struct hci_dbg_set_local_clock_cmd
{
    /// Clock (in half-slots)
    uint32_t clock;
};
#endif //(RW_DEBUG && BT_EMB_PRESENT)


#if(BT_I2C_OVER_HCI)
/// Read I2C Driver
/*@TRACE*/
struct hci_vs_i2c_read_cmd
{
    /// I2C Slave Address
    uint8_t slave_address;
    /// I2C Register Address to read
    uint8_t reg_address;
    /// Data size to read
    uint8_t length;
};

/// Read I2C Driver
/*@TRACE*/
struct hci_vs_i2c_read_cmd_cmp_evt
{
    /// Status
    uint8_t status;
    /// Data size
    uint8_t length;
    /// Data read
    uint8_t data[__ARRAY_EMPTY];
};

/// Write I2C Driver
/*@TRACE*/
struct hci_vs_i2c_write_cmd
{
    /// I2C Slave Address
    uint8_t slave_address;
    /// I2C Register Address to read
    uint8_t reg_address;
    /// Data size to write
    uint8_t length;
    /// Data to write
    uint8_t data[__ARRAY_EMPTY];
};
#endif // (BT_I2C_OVER_HCI)

#if (RW_WLAN_COEX)
///HCI Debug wlan coexistence command parameters - vendor specific
/*@TRACE*/
struct hci_dbg_wlan_coex_cmd
{
    /// State
    uint8_t state;
};
#if (RW_WLAN_COEX_TEST)
///HCI Debug wlan coexistence test scenario command parameters - vendor specific
/*@TRACE*/
struct hci_dbg_wlan_coextst_scen_cmd
{
    /// Scenario
    uint32_t scenario;
};
#endif //RW_WLAN_COEX_TEST
#endif //RW_WLAN_COEX

#if (RW_MWS_COEX)
///HCI Debug mws coexistence command parameters - vendor specific
/*@TRACE*/
struct hci_dbg_mws_coex_cmd
{
    /// State
    uint8_t state;
};
#if (RW_MWS_COEX_TEST)
///HCI Debug mws coexistence test scenario command parameters - vendor specific
/*@TRACE*/
struct hci_dbg_mws_coextst_scen_cmd
{
    /// Scenario
    uint32_t scenario;
};
#endif //RW_MWS_COEX_TEST
#if (RW_DEBUG)
///HCI init SAM Negotiations - EBQ vendor specific
/*@TRACE*/
struct hci_vs_ebq_init_sam_nego_cmd
{
    /// SAM Index
    uint8_t index;
};
#endif //RW_MWS_COEX_TEST
#endif //RW_MWS_COEX

#if BLE_IQ_GEN
///HCI Debug I&Q samples generator command parameters - vendor specific
/// Maximum number of supported antenna patterns
#define DBG_IQGEN_MAX_ANTENNA (8)
/*@TRACE*/
struct hci_dbg_iqgen_cfg_cmd
{
    /// Antenna switch/sample control
    // bit[0]: 0: up-sweep; 1: up-down sweep (internal switching mode)
    // bit[1]: 0: 1us intervals; 1: 2us intervals (internal switching mode)
    // bit[2]: 0: internal switching mode; 1: baseband switching mode
    uint8_t mode;
    /// Number of antenna patterns
    uint8_t nb_antenna;
    /// I/Q sample control
    struct
    {
        /// I-sample control
        /// 0: up-count from 0x01
        /// 1: down-count from 0xFF
        /// 2: Fixed value 0x00
        /// 3: PRBS pattern
        /// 4..0xFF: Fixed value
        uint8_t i;
        /// Q-sample control
        /// 0: up-count from 0x01
        /// 1: down-count from 0xFF
        /// 2: Fixed value 0x00
        /// 3: PRBS pattern
        /// 4..0xFF: Fixed value
        uint8_t q;
    } iq_ctrl[DBG_IQGEN_MAX_ANTENNA];
};
#endif //BLE_IQ_GEN

/// Used to force parameters used for an CIS channel
/*@TRACE*/
struct hci_dbg_iso_set_param_cmd
{
    /// Preferred sub event margin between two sub events (IFS + margin)
    uint32_t sub_evt_margin;
};

#if (BLE_ISO_MODE_0)
/// HCI VS Mic-Less Set command parameters
/*@TRACE*/
struct hci_vs_mic_less_set_cmd
{
    /// Handle that identify an ACL link that is requesting MIC-less mode
    uint16_t    conhdl;
};

/// HCI VS Setup AM0 Data Path command parameters
struct hci_vs_setup_am0_data_path_cmd
{
    /// Channel_Handle to be used to identify an Audio Mode 0 Channel (range 0x0000-0x0EFF)
    uint16_t am0_hdl;
    /// Data path direction
    /// 0x00       Input (Host to Controller)
    /// 0x01       Output (Controller to Host)
    /// Others     Reserved for future use
    uint8_t  data_path_direction;
    /// Data path ID
    /// 0x00       HCI
    /// 0x01-0xFE  Logical_Channel_Number. The meaning of the logical channel is vendor specific.
    /// 0xFF       Test Mode
    uint8_t  data_path_id;
    /// Controller delay in ms (Range: 0x000000 to 0x3D0900)
    uint32_t  ctrl_delay;
};

/// HCI VS Remove AM0 Data Path command parameters
struct hci_vs_remove_am0_data_path_cmd
{
    /// Channel_Handle to be used to identify an Audio Mode 0 Channel (range 0x0000-0x0EFF)
    uint16_t am0_hdl;
    /// Bit[0]: Input data path, Bit[1]: Output data path
    uint8_t  direction;
};

/// HCI VS Setup Audio Mode 0 channel command parameters
/*@TRACE*/
struct hci_vs_setup_am0_stream_cmd
{
    /// Handle that identify an ACL link between a Master and a Slave device that is requesting
    /// an Audio Mode 0 Channel (range 0x0000-0x0EFF)
    uint16_t    conhdl;
    /// The number of bits in each unit of data received from the Host over the data transport. (Range 0x01 - 0xFB)
    uint8_t     tx_size;
    /// The number of bits in each unit of data sent to the Host over the data transport. (Range 0x01 - 0xFB)
    uint8_t     rx_size;
};

/// Used to read the maximum size of the data portion of isochronous packets
/// no parameter on command, only need to specify returned parameters
struct hci_vs_setup_am0_stream_cmd_cmp_evt
{
    /// 0x00 - Request succeed ; 0x01-0xFF Failed reason
    uint8_t  status;
    /// Handle that identify an ACL link between a Master and a Slave device that is requesting
    /// an Audio Mode 0 Channel (range 0x0000-0x0EFF)
    uint16_t conhdl;
    /// Channel_Handle to be used to identify an Audio Mode 0 Channel (range 0x0000-0x0EFF)
    uint16_t am0_hdl;
};

/// Removes existing Audio Mode 0 channel command parameters
/*@TRACE*/
struct hci_vs_remove_am0_stream_cmd
{
    /// Channel_Handles used to identify Audio Mode 0 Channel (range 0x0000-0x0EFF)
    uint16_t am0_hdl;
};

/// HCI basic command complete event structure with AM0 Channel handle
/*@TRACE*/
struct hci_vs_basic_am0_cmd_cmp_evt
{
    /// status
    uint8_t     status;
    /// Channel_Handles used to identify Audio Mode 0 Channel (range 0x0000-0x0EFF)
    uint16_t    am0_hdl;
};
#endif // (BLE_ISO_MODE_0)


///HCI Debug HW Register Read command parameters - vendor specific
/*@TRACE*/
struct hci_dbg_ble_reg_rd_cmd
{
    /// register address
    uint16_t reg_addr;
};

///HCI Debug HW Register write command parameters - vendor specific
/*@TRACE*/
struct hci_dbg_ble_reg_wr_cmd
{
    /// register address
    uint16_t reg_addr;
    /// extra parameter
    uint16_t reserved;
    /// register value
    uint32_t reg_value;
};

///HCI Debug HW Register Read Complete event parameters - vendor specific
/*@TRACE*/
struct hci_dbg_ble_reg_rd_cmd_cmp_evt
{
    /// status
    uint8_t  status;
    /// register address
    uint16_t reg_addr;
    /// register value
    uint32_t reg_value;
};

///HCI Debug HW Register Write Complete event parameters - vendor specific
/*@TRACE*/
struct hci_dbg_ble_reg_wr_cmd_cmp_evt
{
    /// status
    uint8_t  status;
    /// register address
    uint16_t reg_addr;
};

///HCI Vendor Specific Set Preferred Slave Latency command parameters
/*@TRACE*/
struct hci_vs_set_pref_slave_latency_cmd
{
    /// Connection handle
    uint16_t conhdl;
    /// Preferred latency (in number of connection events)
    uint16_t latency;
};

///HCI Vendor Specific Set Preferred Slave event duration command parameters
/*@TRACE*/
struct hci_vs_set_pref_slave_evt_dur_cmd
{
    /// Connection handle
    uint16_t conhdl;
    /// Preferred event duration (N * 0.625 ms)
    uint16_t duration;
    /// Slave transmits a single packet per connection event (False/True)
    uint8_t single_tx;
};

///HCI Vendor Specific Set Maximum RX size and time command parameters
/*@TRACE*/
struct hci_vs_set_max_rx_size_and_time_cmd
{
    /// Connection handle
    uint16_t conhdl;
    /// Maximum RX size (in B)
    uint16_t rx_octets;
    /// Maximum RX time (in us)
    uint16_t rx_time;
};

///HCI Vendor Specific Set Priority Elevation command parameters
/*@TRACE*/
struct hci_vs_set_prio_elevation_cmd
{
    /// Connection handle
    uint16_t conhdl;
    /// Arbitration priority elevation (in units of increment)
    uint8_t elevation;
};

/// Send LLCP Packets
/*@TRACE*/
struct hci_dbg_send_llcp_cmd
{
    /// Connection handle
    uint16_t conhdl;
    ///buffer structure to return
    struct buffer_tag buf;
};

///HCI Debug LLC discard command parameters - vendor specific
/*@TRACE*/
struct hci_dbg_llcp_discard_cmd
{
    /// Handle pointing to the connection for which LLCP commands have to be discarded
    uint16_t conhdl;
    /// Flag indicating if the discarding has to be enabled or disabled
    uint8_t enable;
};

#if (BT_53)
///HCI_DBG_Set_Max_Enc_Key_Size command parameters
/*@TRACE*/
struct hci_dbg_set_max_enc_key_size_cmd
{
    /// Max Encryption Key Size
    uint8_t max_enc_key_size;
};
#endif // (BT_53)

#if (RW_DEBUG)
/// HCI DBG Meta Event indicating a SW assertion
/*@TRACE*/
struct hci_dbg_assert_evt
{
    ///DBG Subevent code
    uint8_t             subcode;
    /// Assert type (warning/error)
    //@trc_ref assert_type
    uint8_t             type;
    /// Line number
    uint32_t line;
    /// Param0
    uint32_t param0;
    /// Param1
    uint32_t param1;
    /// File name
    uint8_t file[__ARRAY_EMPTY];
};
#endif //(RW_DEBUG)

/// HCI LE Read PHY command
/*@TRACE*/
struct hci_le_rd_phy_cmd
{
    /// Connection Handle
    uint16_t            conhdl;
};

/// HCI LE Set Default PHY Command
/*@TRACE*/
struct hci_le_set_dft_phy_cmd
{
    /// Preferred PHYS selection
    uint8_t            all_phys;
    /// Preferred PHYS for TX
    uint8_t            tx_phys;
    /// Preferred PHYS for RX
    uint8_t            rx_phys;
};

/// HCI LE Set PHY Command
/*@TRACE*/
struct hci_le_set_phy_cmd
{
    /// Connection Handle
    uint16_t           conhdl;
    /// Preferred PHYS selection
    uint8_t            all_phys;
    /// Preferred PHYS for TX
    uint8_t            tx_phys;
    /// Preferred PHYS for RX
    uint8_t            rx_phys;
    /// PHY options
    uint16_t           phy_opt;
};

/// HCI LE Set Extended Advertising Parameters Command Complete Event
/*@TRACE*/
struct hci_le_set_ext_adv_param_cmd_cmp_evt
{
    /// Status
    uint8_t            status;
    /// Selected Tx power
    int8_t             sel_tx_pwr;
};

/// HCI LE Receiver Test v2 Command
/*@TRACE*/
struct hci_le_rx_test_v2_cmd
{
    /// RX channel, range: 0x00 to 0x27
    uint8_t     rx_channel;
    /// PHY (@enum le_phy_value)
    uint8_t     phy;
    /// Modulation index (0: standard | 1: stable)
    uint8_t     mod_idx;
};

/// HCI LE Transmitter Test v2 Command
/*@TRACE*/
struct hci_le_tx_test_v2_cmd
{
    /// TX channel, range: 0x00 to 0x27
    uint8_t     tx_channel;
    /// Length of test data in bytes, range: 0x00 to 0xFF
    uint8_t     test_data_len;
    /**
     * Packet payload
     * 0x00 PRBS9 sequence "11111111100000111101" (in transmission order) as described in [Vol 6] Part F, Section 4.1.5
     * 0x01 Repeated "11110000" (in transmission order) sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x02 Repeated "10101010" (in transmission order) sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x03 PRBS15 sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x04 Repeated "11111111" (in transmission order) sequence
     * 0x05 Repeated "00000000" (in transmission order) sequence
     * 0x06 Repeated "00001111" (in transmission order) sequence
     * 0x07 Repeated "01010101" (in transmission order) sequence
     * 0x08-0xFF Reserved for future use
     */
    uint8_t     pkt_payl;
    /// PHY (@enum le_phy_value)
    uint8_t     phy;
};

///HCI LE Set Advertising Set Random Address Command parameters structure
/*@TRACE*/
struct hci_le_set_adv_set_rand_addr_cmd
{
    /// Advertising handle
    uint8_t adv_hdl;
    /// Advertising random address
    struct bd_addr rand_addr;
};

///HCI LE Set Extended Advertising Parameters Command parameters structure
/*@TRACE*/
struct hci_le_set_ext_adv_param_cmd
{
    /// Advertising handle
    uint8_t adv_hdl;
    /// Advertising event properties
    uint16_t adv_evt_properties;
    /// Primary advertising minimum interval
    uint8_t prim_adv_intv_min[3];
    /// Primary advertising maximum interval
    uint8_t prim_adv_intv_max[3];
    /// Primary advertising channel map
    uint8_t prim_adv_ch_map;
    /// Own address type:  public=0 / random=1 / rpa_or_pub=2 / rpa_or_rnd=3
    uint8_t own_addr_type;
    /// Peer address type: public=0 / random=1
    uint8_t peer_addr_type;
    /// Peer Bluetooth device address
    struct bd_addr peer_addr;
    /// Advertising filter policy
    uint8_t adv_filt_policy;
    /// Advertising Tx power
    int8_t adv_tx_pwr;
    /// Primary advertising PHY
    uint8_t prim_adv_phy;
    /// Secondary advertising max skip
    uint8_t sec_adv_max_skip;
    /// Secondary advertising PHY
    uint8_t sec_adv_phy;
    /// Advertising SID
    uint8_t adv_sid;
    /// Scan request notification enable
    uint8_t scan_req_notif_en;
};

///HCI LE Set Extended Advertising Data Command parameters structure
/*@TRACE*/
struct hci_le_set_ext_adv_data_cmd
{
    /// Advertising handle
    uint8_t adv_hdl;
    /**
     *  Operation
     *  0x00 Intermediate fragment of fragmented extended advertising data
     *  0x01 First fragment of fragmented extended advertising data
     *  0x02 Last fragment of fragmented extended advertising data
     *  0x03 Complete extended advertising data
     *  0x04 [ID7300_r06] Unchanged data (just update the Advertising DID)
     *  All other values Reserved for future use
     */
    uint8_t operation;
    /**
     *  Fragment preference
     *  0x00 The Controller may fragment all Host advertising data
     *  0x01 The Controller should not fragment nor minimize fragmentation of Host advertising data
     *  All other values Reserved for future use
     */
    uint8_t frag_pref;
    /// Advertising Data Length (0-252 bytes)
    uint8_t data_len;
    /// Advertising data
    uint8_t data[__ARRAY_EMPTY];
};

///HCI LE Set Extended Scan Response Data Command parameters structure
/*@TRACE*/
struct hci_le_set_ext_scan_rsp_data_cmd
{
    /// Advertising handle
    uint8_t adv_hdl;
    /**
     *  Operation
     *  0x00 Intermediate fragment of fragmented extended advertising data
     *  0x01 First fragment of fragmented extended advertising data
     *  0x02 Last fragment of fragmented extended advertising data
     *  0x03 Complete extended advertising data
     *  0x04 [ID7300_r06] Unchanged data (just update the Advertising DID)
     *  All other values Reserved for future use
     */
    uint8_t operation;
    /**
     *  Fragment preference
     *  0x00 The Controller may fragment all Host advertising data
     *  0x01 The Controller should not fragment nor minimize fragmentation of Host advertising data
     *  All other values Reserved for future use
     */
    uint8_t frag_pref;
    /// Scan Response Data Length (0-252 bytes)
    uint8_t data_len;
    /// Advertising data
    uint8_t data[__ARRAY_EMPTY];
};

#if BLE_EMB_PRESENT || BLE_HOST_PRESENT

/// Advertising set enable parameters
/*@TRACE*/
struct hci_ext_adv_set_en_param
{
    /// Advertising handle
    uint8_t adv_hdl;
    /// Duration (N * 10 ms), 0x0000 No advertising duration. Advertising to continue until the Host disables it.
    uint16_t duration;
    /// Maximum number of extended advertising events
    uint8_t max_ext_adv_evt;
};

///HCI LE Set Extended Advertising Enable Command parameters structure
/*@TRACE*/
struct hci_le_set_ext_adv_en_cmd
{
    /// Enable
    uint8_t enable;
    /// Number of sets (1 - 0x3F)
    uint8_t nb_sets;
    /// Advertising Set enable parameters
    struct hci_ext_adv_set_en_param sets[BLE_ACTIVITY_MAX];
};
#endif //BLE_EMB_PRESENT || BLE_HOST_PRESENT

///HCI LE Read Maximum Advertising Data Length Command complete event
/*@TRACE*/
struct hci_le_rd_max_adv_data_len_cmd_cmp_evt
{
    /// Status
    uint8_t status;
    /// Maximum advertising data length
    uint16_t max_adv_data_len;
};

///HCI LE Remove Advertising Set Command parameters structure
/*@TRACE
 * hci_le_rmv_adv_set_cmd = hci_le_rem_adv_set_cmd*/
struct hci_le_rem_adv_set_cmd
{
    /// Advertising handle
    uint8_t adv_hdl;
};

///HCI LE Read Number of Supported Advertising Sets Command complete event
/*@TRACE*/
struct hci_le_rd_nb_supp_adv_sets_cmd_cmp_evt
{
    /// Status
    uint8_t status;
    /// Number of supported advertising sets
    uint8_t nb_supp_adv_sets;
};

///HCI LE Read Transmit Power Command complete event
/*@TRACE*/
struct hci_le_rd_tx_pwr_cmd_cmp_evt
{
    /// Status
    uint8_t status;
    /// Minimum TX Power
    int8_t min_tx_pwr;
    /// Maximum TX Power
    int8_t max_tx_pwr;
};

///HCI LE Read RF Path Compensation Command complete event
/*@TRACE*/
struct hci_le_rd_rf_path_comp_cmd_cmp_evt
{
    /// Status
    uint8_t status;
    /// RF TX Path Compensation
    int16_t tx_path_comp;
    /// RF RX Path Compensation
    int16_t rx_path_comp;
};

///HCI LE Write RF Path Compensation Command complete event
/*@TRACE*/
struct hci_le_wr_rf_path_comp_cmd
{
    /// RF TX Path Compensation
    int16_t tx_path_comp;
    /// RF RX Path Compensation
    int16_t rx_path_comp;
};

///HCI LE Set Periodic Advertising Parameters Command parameters structure
/*@TRACE*/
struct hci_le_set_per_adv_param_cmd
{
    /// Advertising handle
    uint8_t adv_hdl;
    /// Minimum advertising interval for periodic advertising (units of 1.25 ms)
    uint16_t adv_intv_min;
    /// Maximum advertising interval for periodic advertising (units of 1.25 ms)
    uint16_t adv_intv_max;
    /// Advertising properties
    uint16_t adv_prop;
};

///HCI LE Set Periodic Advertising Data Command parameters structure
/*@TRACE*/
struct hci_le_set_per_adv_data_cmd
{
    /// Advertising handle
    uint8_t adv_hdl;
    /**
     *  Operation
     *  0x00 Intermediate fragment of fragmented periodic advertising data
     *  0x01 First fragment of fragmented periodic advertising data
     *  0x02 Last fragment of fragmented periodic advertising data
     *  0x03 Complete periodic advertising data
     *  0x04 Unchanged data, just update advertising DID
     *  All other values Reserved for future use
     */
    uint8_t operation;
    /// Advertising Data Length (0-252 bytes)
    uint8_t data_len;
    /// Advertising data
    uint8_t data[__ARRAY_EMPTY];
};

///HCI LE Set Periodic Advertising Enable Command parameters structure
/*@TRACE*/
struct hci_le_set_per_adv_en_cmd
{
    /**
     *  Enable (@see enum per_adv_en)
     *  Bit number 0: Enable periodic advertising
     *  Bit number 1: Include ADI field in AUX_SYNC_IND PDUs (BT_53)
     *  All other bits: Reserved for future use
     */
    uint8_t enable;
    /// Advertising handle
    uint8_t adv_hdl;
};

#if (BT_53)
///HCI LE Set Data Related Address Changes Command parameters structure
/*@TRACE*/
struct hci_le_set_data_related_addr_chg_cmd
{
    /// Advertising handle
    uint8_t adv_hdl;
    /**
     *  Change reasons
     *  Bit number 0: Change the address whenever the advertising data changes
     *  Bit number 1: Change the address whenever the scan response data changes
     *  All other bits: Reserved for future use
     */
    uint8_t chg_reasons;
};

/// HCI LE Set Default Subrate Comamnd parameters structure
/*@TRACE*/
struct hci_le_set_dft_subrate_cmd
{
    /// Subrate min
    uint16_t subrate_min;
    /// Subrate max
    uint16_t subrate_max;
    /// Max latency (N x con intv)
    uint16_t max_latency;
    /// Continuation number (N x con evts)
    uint16_t cont_num;
    /// Supervision timeout (N x 10ms)
    uint16_t supv_to;
};

/// HCI LE Subrate Request command parameters structure
/*@TRACE*/
struct hci_le_subrate_req_cmd
{
    /// Connection handle
    uint16_t conhdl;
    /// Subrate min
    uint16_t subrate_min;
    /// Subrate max
    uint16_t subrate_max;
    /// Max latency (N x con intv)
    uint16_t max_latency;
    /// Continuation number (N x con evts)
    uint16_t cont_num;
    /// Supervision timeout (N x 10ms)
    uint16_t supv_to;
};

/// HCI LE Subrate Change event
/*@TRACE */
struct hci_le_subrate_chg_evt
{
    /// LE Subevent code
    uint8_t  subcode;
    /// Status
    uint8_t  status;
    /// Connection handle
    uint16_t conhdl;
    /// Subrate factor (N x con intv)
    uint16_t subrate_factor;
    /// Peripheral latency (N x con evts)
    uint16_t periph_latency;
    /// Continuation number (N x con evts)
    uint16_t cont_num;
    /// Supervision timeout (N x 10ms)
    uint16_t supv_to;
};

#endif // (BT_53)

///HCI LE Advertising Set Terminated event
/*@TRACE
 * hci_le_adv_set_terminated_evt = hci_le_adv_set_term_evt*/
struct hci_le_adv_set_term_evt
{
    /// LE Subevent code
    uint8_t  subcode;
    /// Status
    uint8_t  status;
    /// Advertising handle
    uint8_t  adv_hdl;
    /// Connection handle
    uint16_t conhdl;
    /// Num_Completed_Extended_Advertising_Events
    uint8_t  nb_cmp_ext_adv_evt;
};

///HCI LE Scan Request Received event
/*@TRACE*/
struct hci_le_scan_req_rcvd_evt
{
    /// LE Subevent code
    uint8_t  subcode;
    /// Advertising handle
    uint8_t  adv_hdl;
    /// Scanner address type:  public=0 / random=1 / rpa_or_pub=2 / rpa_or_rnd=3
    uint8_t scan_addr_type;
    /// Scanner address
    struct bd_addr scan_addr;
};

///HCI LE Read Periodic Advertiser List Size Command complete event
/*@TRACE*/
struct hci_le_rd_per_adv_list_size_cmd_cmp_evt
{
    ///Status
    uint8_t status;
    /// Periodic Advertiser List Size
    uint8_t size;
};

///HCI LE Channel Selection Algorithm event
/*@TRACE*/
struct hci_le_ch_sel_algo_evt
{
    /// LE Subevent code
    uint8_t  subcode;
    /// Connection handle
    uint16_t  conhdl;
    /// Channel selection algorithm
    uint8_t ch_sel_algo;
};

/// HCI LE read PHY Command complete event
/*@TRACE*/
struct hci_le_rd_phy_cmd_cmp_evt
{
    ///Status of received command
    uint8_t             status;
    /// Connection Handle
    uint16_t            conhdl;
    /// Current configured PHY for TX
    //@trc_ref le_phy_value
    uint8_t             tx_phy;
    /// Current configured PHY for RX
    //@trc_ref le_phy_value
    uint8_t             rx_phy;
};

/// HCI LE PHY Update Complete event
/*@TRACE*/
struct hci_le_phy_upd_cmp_evt
{
    ///LE Subevent code
    uint8_t             subcode;
    ///Status of received command
    uint8_t             status;
    ///Connection handle
    uint16_t            conhdl;
    ///TX phy chosen
    //@trc_ref le_phy_value
    uint8_t             tx_phy;
    ///RX phy chosen
    //@trc_ref le_phy_value
    uint8_t             rx_phy;
};

///HCI LE Connectionless IQ Report event
/*@TRACE*/
struct hci_le_conless_iq_report_evt
{
    /// LE Subevent code
    uint8_t  subcode;
    /// Sync handle
    uint16_t sync_hdl;
    /// Data channel index
    uint8_t  channel_idx;
    /// RSSI (in 0.1 dBm)
    int16_t rssi;
    /// RSSI antenna ID
    uint8_t rssi_antenna_id;
    /// CTE type (0: AOA | 1: AOD-1us | 2: AOD-2us)
    uint8_t cte_type;
    /// Slot durations (1: 1us | 2: 2us)
    uint8_t slot_dur;
    /// Packet status
    uint8_t pkt_status;
    /// paEventCounter
    uint16_t pa_evt_cnt;
    /// Sample_count
    uint8_t sample_cnt;
    /// I/Q sample
    struct
    {
        /// I sample
        int8_t i;
        /// Q sample
        int8_t q;
    } iq_sample[MAX_SAMPLE_CNT];
};

///HCI LE Connection IQ Report event
/*@TRACE*/
struct hci_le_con_iq_report_evt
{
    /// LE Subevent code
    uint8_t  subcode;
    /// Connection handle
    uint16_t conhdl;
    /// Rx PHY  (@see enum le_phy_value)
    uint8_t  rx_phy;
    /// Data channel index
    uint8_t  data_channel_idx;
    /// RSSI (in 0.1 dBm)
    int16_t rssi;
    /// RSSI antenna ID
    uint8_t rssi_antenna_id;
    /// CTE type (0: AOA | 1: AOD-1us | 2: AOD-2us)
    uint8_t cte_type;
    /// Slot durations (1: 1us | 2: 2us)
    uint8_t slot_dur;
    /// Packet status
    uint8_t pkt_status;
    /// connEventCounter
    uint16_t con_evt_cnt;
    /// Sample_count
    uint8_t sample_cnt;
    /// I/Q sample
    struct
    {
        /// I sample
        int8_t i;
        /// Q sample
        int8_t q;
    } iq_sample[MAX_SAMPLE_CNT];
};

///HCI LE CTE Request Failed event
/*@TRACE*/
struct hci_le_cte_req_failed_evt
{
    /// LE Subevent code
    uint8_t  subcode;
    /// Status
    uint8_t  status;
    /// Connection handle
    uint16_t  conhdl;
};

///Commands Opcodes: OGF(6b) | OCF(10b)
/* Some Abbreviation used in names:
 *  - LK   = Link Key
 *  - RD   = Read
 *  - WR   = Write
 *  - REM  = Remote
 *  - STG  = Settings
 *  - CON  = Connection
 *  - CHG  = Change
 *  - DFT  = Default
 *  - PER  = Periodic
 *  - CHAN = Channel
 *  - REQ  = Request
 */

/// Used to read the maximum size of the data portion of ACL data packets and isochronous data packets
/// no parameter on command, only need to specify returned parameters
/*@TRACE*/
struct hci_le_rd_buf_size_v2_cmd_cmp_evt
{
    /// 0x00 - Request succeed ; 0x01-0xFF Failed reason
    uint8_t  status;
    /// Maximum length (in octets) of the data portion of each HCI ACL (0x0000: No dedicated LE buffer, 0x0001-0x001A RFU, Range 0x001B-0xFFFF)
    uint16_t le_acl_data_packet_length;
    /// The total number of HCI ACL data packets that can be stored in the data buffers of the Controller (0x00: No dedicated LE buffer, Range 0x01-0xFF)
    uint8_t  total_num_le_acl_data_packets;
    /// The maximum length (in octets) of the data portion of each HCI ISO data packet (0x0000: No dedicated ISO buffer, Range 0x0001-0xFFFF)
    uint16_t iso_data_packet_length;
    /// The total number of HCI ISO data packets that can be stored in the ISO buffers of the Controller (0x00: No dedicated ISO buffer, Range 0x01-0xFF)
    uint8_t  total_num_iso_data_packets;
};

#if (BLE_CIS || BLE_BIS)
/// Command Complete event of HCI_LE_RD_ISO_TX_SYNC_CMD
/*@TRACE*/
struct hci_le_rd_iso_tx_sync_cmd_cmp_evt
{
    /// 0x00 - Command succeeded ; 0x01-0xFF Failed reason
    uint8_t  status;
    /// Connection handle of the CIS or BIS (12 bits meaningful - Range 0x0000-0x0EFF)
    uint16_t conhdl;
    /// The packet sequence number of an SDU (Range 0x0000-0xFFFF)
    uint16_t pkt_seq_nb;
    /// The time stamp, in microseconds, of the reference anchor point of a transmitted SDU (Range 0x00000000-0xFFFFFFFF)
    uint32_t time_stamp;
    /// The time offset, in microseconds, that is associated with a transmitted SDU (Range 0x000000-0xFFFFFF)
    uint32_t time_offset;
};
#endif // (BLE_CIS || BLE_BIS)

#if (BLE_CIS)
/// Description of CIS parameters
/*@TRACE*/
struct hci_le_cis_param
{
    /// Used to identify a Connected Isochronous Stream. (Range 0x00-0xEF)
    uint8_t  cis_id;
    /// Maximum size of an SDU in octets from the master Host (12 bits meaningful - Range 0x000-0xFFF).
    uint16_t max_sdu_m2s;
    /// Maximum size of an SDU in octets from the master Host (12 bits meaningful - Range 0x000-0xFFF).
    uint16_t max_sdu_s2m;
    /// Master to slave PHY, bit 0: 1Mbps, bit 1: 2Mbps, bit 2: LE-Coded
    uint8_t  phy_m2s;
    /// Slave to master PHY, bit 0: 1Mbps, bit 1: 2Mbps, bit 2: LE-Coded
    uint8_t  phy_s2m;
    /// Maximum number of times every CIS Data PDU should be retransmitted from the master to slave (range 0x00-0x0F)
    uint8_t  rtn_m2s;
    /// Maximum number of times every CIS Data PDU should be retransmitted from the slave to master (range 0x00-0x0F)
    uint8_t  rtn_s2m;
};

/// Used by a master Host to set the parameters of all Connected Isochronous Streams associated with a Connected
/// Isochronous Group in the Controller
/*@TRACE*/
struct hci_le_set_cig_params_cmd
{
    /// Used to identify the Connected Isochronous Group (range 0x00 to 0xEF).
    uint8_t  cig_id;
    /// Interval in microseconds of periodic SDUs master to slave (20 bits meaningful Range 0x000FF - 0xFFFFF).
    uint32_t sdu_interval_m2s;
    /// Interval in microseconds of periodic SDUs slave to master (20 bits meaningful Range 0x000FF - 0xFFFFF).
    uint32_t sdu_interval_s2m;
    /// Slave Clock Accuracy:
    /// 0x00: 251 ppm to 500 ppm,  0x01: 151 ppm to 250 ppm, 0x02: 101 ppm to 150 ppm, 0x03: 76 ppm to 100 ppm
    /// 0x04:  51 ppm to  75 ppm,  0x05:  31 ppm to  50 ppm, 0x06:  21 ppm to  30 ppm, 0x07:  0 ppm to  20 ppm
    uint8_t  sca;
    /// Scheduling method, 0: Sequential, 1: Interleaved
    uint8_t  packing;
    /// ISOAL Framing mode, 0: Unframed, 1: Framed
    uint8_t  framing;
    /// The maximum time in milliseconds for an SDU to be transported from the master Controller to slave Controller.
    /// (range 0x0005 to 0x0FA0)
    uint16_t max_trans_latency_m2s;
    /// The maximum time in milliseconds for an SDU to be transported from the slave Controller to master Controller.
    /// (range 0x0005 to 0x0FA0)
    uint16_t max_trans_latency_s2m;
    /// Total number of CISs in the CIG being added or modified. (Range 0x01-0x10)
    uint8_t  cis_count;
    /// CIS Parameters
    struct hci_le_cis_param params[0x10];
};

/// Command Complete event of HCI_LE_SET_CIG_PARAMS_CMD
/*@TRACE*/
struct hci_le_set_cig_params_cmd_cmp_evt
{
    /// 0x00 - The Broadcast Isochronous Group has been completed ; 0x01-0xFF Failed reason
    uint8_t  status;
    /// Used to identify the Connected Isochronous Group. (Range 0x00-0xEF)
    uint8_t  cig_id;
    /// Total number of CISs added or modified by this command (Range 0x01-0x10)
    uint8_t  cis_count;
    /// List of connection handles of CISs in the CIG. The connection handle list order is the same as the list order
    /// of the CISs in the command  (Range 0x0000-0x0EFF)
    uint16_t conhdl[0x10];
};

/// Description of CIS parameters
/*@TRACE*/
struct hci_le_cis_test_param
{
    /// Used to identify a Connected Isochronous Stream. (Range 0x00-0xEF)
    uint8_t  cis_id;
    /// Maximum number of subevents in each interval of Connected Isochronous Stream (Range 0x00-0x1F)
    uint8_t  nse;
    /// Maximum size of an SDU in octets from the master Host (12 bits meaningful - Range 0x000-0xFFF).
    uint16_t max_sdu_m2s;
    /// Maximum size of an SDU in octets from the master Host (12 bits meaningful - Range 0x000-0xFFF).
    uint16_t max_sdu_s2m;
    /// Maximum size of payload from master Link Layer to slave Link Layer (Range 0x01-0xFB).
    uint16_t  max_pdu_m2s;
    /// Maximum size of payload from slave Link Layer to master Link Layer (Range 0x01-0xFB).
    uint16_t  max_pdu_s2m;
    /// Master to slave PHY, bit 0: 1Mbps, bit 1: 2Mbps, bit 2: LE-Coded
    uint8_t  phy_m2s;
    /// Slave to master PHY, bit 0: 1Mbps, bit 1: 2Mbps, bit 2: LE-Coded
    uint8_t  phy_s2m;
    /// The burst number for master to slave transmission (Range 0x00-0x0F).
    /// 0x00 - No isochronous data from the master to the slave
    uint8_t  bn_m2s;
    /// The burst number for slave to master transmission (Range 0x00-0x0F).
    /// 0x00 - No isochronous data from the slave to the master
    uint8_t  bn_s2m;
};

/// Used by a master Host to set the parameters of all Connected Isochronous Streams associated with a Connected
/// Isochronous Group in the Controller
/*@TRACE*/
struct hci_le_set_cig_params_test_cmd
{
    /// Used to identify the Connected Isochronous Group. (Range 0x00-0xEF)
    uint8_t  cig_id;
    /// Interval in microseconds of periodic SDUs master to slave (20 bits meaningful Range 0x000FF - 0xFFFFF).
    uint32_t sdu_interval_m2s;
    /// Interval in microseconds of periodic SDUs slave to master (20 bits meaningful Range 0x000FF - 0xFFFFF).
    uint32_t sdu_interval_s2m;
    /// The maximum flush timeout for each payload from the master (Range 0x01-0x1F).
    uint8_t  ft_m2s;
    /// The maximum flush timeout for each payload from the slave (Range 0x01-0x1F).
    uint8_t  ft_s2m;
    /// The duration of the isochronous interval in multiples of 1.25 ms (Range 0x0004-0xC80).
    uint16_t iso_interval;
    /// Slave Clock Accuracy:
    /// 0x00: 251 ppm to 500 ppm,  0x01: 151 ppm to 250 ppm, 0x02: 101 ppm to 150 ppm, 0x03: 76 ppm to 100 ppm
    /// 0x04:  51 ppm to  75 ppm,  0x05:  31 ppm to  50 ppm, 0x06:  21 ppm to  30 ppm, 0x07:  0 ppm to  20 ppm
    uint8_t  sca;
    /// Scheduling method, 0: Sequential, 1: Interleaved
    uint8_t  packing;
    /// ISOAL Framing mode, 0: Unframed, 1: Framed
    uint8_t  framing;
    /// Total number of CISs in the CIG being added or modified. (Range 0x00-0x10)
    uint8_t  cis_count;
    /// CIS Parameters
    struct hci_le_cis_test_param params[0x10];
};

/// Command Complete event of HCI_LE_SET_CIG_PARAMS_TEST_CMD
/*@TRACE*/
struct hci_le_set_cig_params_test_cmd_cmp_evt
{
    /// 0x00 - The Broadcast Isochronous Group has been completed ; 0x01-0xFF Failed reason
    uint8_t  status;
    /// Used to identify the Connected Isochronous Group. (Range 0x00-0xEF)
    uint8_t  cig_id;
    /// Total number of CISs added or modified by this command (Range 0x00-0x10)
    uint8_t  cis_count;
    /// List of connection handles of CISs in the CIG. The connection handle list order is the same as the list order
    /// of the CISs in the command  (Range 0x0000-0x0EFF)
    uint16_t conhdl[0x10];
};

/// Create CIS parameter
/*@TRACE*/
struct hci_le_cis_create_param
{
    /// Connection handle of Connected Isochronous Stream (Range 0x0000-0x0EFF)
    uint16_t cis_conhdl;
    /// Connection handle of ACL Link (Range 0x0000-0x0EFF)
    uint16_t acl_conhdl;
};

/// Used by the master Host to establish one or more Connected Isochronous Streams using the connections identified
/// by the ACL_Connection_Handle
/*@TRACE*/
struct hci_le_create_cis_cmd
{
    /// Total number of CISs to be established. (Range 0x01-0x1F)
    uint8_t  cis_count;
    /// Create CIS Parameters
    struct hci_le_cis_create_param params[0x1F];
};

/// Removes all the Connected Isochronous Streams which have previously been set up in the Controller by using the
/// HCI_LE_Set_CIG_Parameters or the HCI_LE_Set_CIG_Parameters_Test command.
/*@TRACE*/
struct hci_le_remove_cig_cmd
{
    /// Used to identify the Connected Isochronous Group. (Range 0x00-0xEF)
    uint8_t  cig_id;
};

/// Command Complete event of HCI_LE_REMOVE_CIG_CMD
/*@TRACE*/
struct hci_le_remove_cig_cmd_cmp_evt
{
    /// 0x00 - The HCI_LE_Remove_CIG command succeeded ; 0x01-0xFF Failed reason
    uint8_t  status;
    /// Identifier of a CIG. (Range 0x00-0xEF)
    uint8_t  cig_id;
};

/// Used by the slave Host to inform the Controller to accept the request for the Connected Isochronous Stream that is
/// identified by the Connection_Handle
/*@TRACE*/
struct hci_le_accept_cis_req_cmd
{
    /// Connection handle of the Connected Isochronous Stream (Range: 0x0000-0x0EFF)
    uint16_t conhdl;
};

/// Used by the slave Host to reject the request for the Connected Isochronous Stream that is identified by the
/// Connection_Handle
/*@TRACE*/
struct hci_le_reject_cis_req_cmd
{
    /// Connection handle of the Connected Isochronous Stream (Range: 0x0000-0x0EFF)
    uint16_t conhdl;
    /// The reason for rejecting the Connected Isochronous Stream
    uint8_t  reason;
};
#endif // (BLE_CIS)

#if (BLE_BIS)
/// Used to transmit data PDUs of one or more Broadcast Isochronous Streams of the Broadcast Isochronous Group
/*@TRACE*/
struct hci_le_create_big_cmd
{
    /// BIG_Handle is used to identify the BIS Group. (Range 0x00-0xEF)
    uint8_t  big_hdl;
    /// Used to identify the Periodic Advertising set (Range 0x00-0xEF)
    uint8_t  adv_hdl;
    /// Total number of Broadcast Isochronous Streams in the Broadcast Isochronous Group. (Range 0x01-0x1F)
    uint8_t  num_bis;
    /// Interval of periodic SDUs in microseconds (20 bits meaningful - range 0x000FF-0xFFFFF).
    uint32_t sdu_interval;
    /// Maximum size of an SDU (12 bits meaningful Range 0x000 to 0xFFF).
    uint16_t max_sdu;
    /// The maximum time in milliseconds for a SDU to be transported (Range 0x0000-0x0FA0)
    uint16_t trans_latency;
    /// Number of times every BIS Data PDU should be retransmitted (Range 0x00-0x0F)
    uint8_t  rtn;
    /// PHY, bit 0: 1Mbps, bit 1: 2Mbps, bit 2: LE-Coded
    uint8_t  phy;
    /// Scheduling method, 0: Sequential, 1: Interleaved
    uint8_t  packing;
    /// ISOAL Framing mode, 0: Unframed, 1: Framed
    uint8_t  framing;
    /// 0: Unencrypted, 1: Encrypted
    uint8_t  encryption;
    /// Code used for deriving the session key for encrypting and decrypting payloads of BISs.
    uint8_t  broadcast_code[KEY_LEN];
};

/// Prepares the CL-Master Controller to set up a new BIG.
/*@TRACE*/
struct hci_le_create_big_test_cmd
{
    /// BIG_Handle is used to identify the BIS Group. (Range 0x00-0xEF)
    uint8_t  big_hdl;
    /// Used to identify the Periodic Advertising set (Range 0x00-0xEF)
    uint8_t  adv_hdl;
    /// Total number of Broadcast Isochronous Streams in the Broadcast Isochronous Group. (Range 0x01-0x1F)
    uint8_t  num_bis;
    /// Interval of periodic SDUs in microseconds (20 bits meaningful - range 0x000FF-0xFFFFF).
    uint32_t sdu_interval;
    /// Isochronous Interval in multiples of 1.25 ms (Range 0x0004-0xC80)
    uint16_t iso_interval;
    /// Total number of subevents in each interval of each BIS in the BIG (Range 0x01-0x1F)
    uint8_t  nse;
    /// Maximum size of an SDU (12 bits meaningful Range 0x000 to 0xFFF).
    uint16_t max_sdu;
    /// Maximum size of payload (Range 0x01-0xFB)
    uint16_t  max_pdu;
    /// PHY, bit 0: 1Mbps, bit 1: 2Mbps, bit 2: LE-Coded
    uint8_t  phy;
    /// Scheduling method, 0: Sequential, 1: Interleaved
    uint8_t  packing;
    /// ISOAL Framing mode, 0: Unframed, 1: Framed
    uint8_t  framing;
    /// Number of new payloads in each interval for each Broadcast Isochronous Stream (Burst Number) (Range 0x01-0x07)
    uint8_t  bn;
    /// Number of times the scheduled payload(s) should be transmitted (Range 0x01-0x0F)
    uint8_t  irc;
    /// Isochronous Interval spacing of payloads transmitted in the Pre_Transmission_Subevents (Range 0x00-0x0F)
    uint8_t  pto;
    /// 0: Unencrypted, 1: Encrypted
    uint8_t  encryption;
    /// Code used for encrypting or decrypting payloads of an encrypted Broadcast Isochronous Stream
    uint8_t  broadcast_code[KEY_LEN];
};

/// Terminate transmission of all Broadcast Isochronous Streams of a Broadcast Isochronous Group or to cancel
/// the process of creating a Broadcast Isochronous Group
/*@TRACE*/
struct hci_le_terminate_big_cmd
{
    /// BIG_Handle is used to identify the BIS Group. (Range 0x00-0xEF)
    uint8_t  big_hdl;
    /// Termination reason
    uint8_t  reason;
};

/// Used to synchronize and receive PDUs from one or more Broadcast Isochronous Streams within a Broadcast Isochronous
/// Group.
/*@TRACE*/
struct hci_le_big_create_sync_cmd
{
    /// BIG_Handle is used to identify the BIS Group. (Range 0x00-0xEF)
    uint8_t  big_hdl;
    /// Used to identify the periodic advertising set (Range 0x0000-0x0EFF)
    uint16_t sync_hdl;
    /// 0: Unencrypted, 1: Encrypted
    uint8_t  encryption;
    /// 128-bit code used for deriving the session key for decrypting payloads of encrypted BISs.
    uint8_t  broadcast_code[16];
    /// Maximum number of subevents that should be used to receive data payloads in each isochronous interval (0x00-0xFF)
    /// 0x00 - The Controller can  schedule reception of any number of subevents up to NSE
    uint8_t  mse;
    /// Synchronization timeout for a Broadcast Isochronous Group (step 10ms, range 0x000A-0x4000)
    uint16_t big_sync_timeout;
    /// Total number of BISs to synchronize. (Range 0x01-0x1F)
    uint8_t  num_bis;
    /// List of indices of BISs (bis_id Range 0x01-0x1F)
    uint8_t  bis_id[0x1F];
};

/// Stop synchronization with Broadcast Isochronous Streams or to cancel the process of synchronization to Broadcast Isochronous Streams
/*@TRACE*/
struct hci_le_big_terminate_sync_cmd
{
    /// BIG_Handle is used to identify the BIS Group. (Range 0x00-0xEF)
    uint8_t  big_hdl;
};

/// Command Complete event of HCI_LE_BIG_TERMINATE_SYNC_CMD
/*@TRACE*/
struct hci_le_big_terminate_sync_cmd_cmp_evt
{
    /// 0x00 - Command succeeded ; 0x01-0xFF Failed reason
    uint8_t  status;
    /// BIG_Handle is used to identify the BIS Group. (Range 0x00-0xEF)
    uint8_t  big_hdl;
};
#endif // (BLE_BIS)

/// Stop synchronization with Broadcast Isochronous Streams or to cancel the process of synchronization to Broadcast Isochronous Streams
/*@TRACE*/
struct hci_le_req_peer_sca_cmd
{
    /// Connection handle of the CIS or BIS (12 bits meaningful - Range 0x0000-0x0EFF)
    uint16_t conhdl;
};

#if (BLE_CIS || BLE_BIS)
/// The command is used to identify and create the isochronous data path between the Host and the Controller.
/// It can also be used to configure a codec for each data path.
/*@TRACE*/
struct hci_le_setup_iso_data_path_cmd
{
    /// Connection handle of a CIS or BIS. (Range 0x0000-0x0EFF)
    uint16_t conhdl;
    /// Data path direction
    /// 0x00       Input (Host to Controller)
    /// 0x01       Output (Controller to Host)
    /// Others     Reserved for future use
    uint8_t  data_path_direction;
    /// Data path ID
    /// 0x00       HCI
    /// 0x01-0xFE  Logical_Channel_Number. The meaning of the logical channel is vendor specific.
    /// 0xFF       Reserved for future use
    uint8_t  data_path_id;
    /// Codec ID (5 octets)
    uint8_t  codec_id[CODEC_ID_LEN];
    /// Controller delay in ms (Range: 0x000000 to 0x3D0900)
    uint32_t  ctrl_delay;
    /// Codec configuration length
    uint8_t  codec_cfg_len;
    /// Codec configuration
    uint8_t  codec_cfg[__ARRAY_EMPTY];
};

/// Command Complete event of HCI_LE_SETUP_ISO_DATA_PATH_CMD
/*@TRACE*/
struct hci_le_setup_iso_data_path_cmd_cmp_evt
{
    /// 0x00 - Command succeeded ; 0x01-0xFF Failed reason
    uint8_t  status;
    /// Connection handle of the CIS or BIS (12 bits meaningful - Range 0x0000-0x0EFF)
    uint16_t conhdl;
};

/// The command remove the data path between the Host and Controller and vice versa.
/*@TRACE*/
struct hci_le_remove_iso_data_path_cmd
{
    /// Connection handle of a CIS or BIS. (Range 0x0000-0x0EFF)
    uint16_t conhdl;
    /// Data path direction
    /// bit 0      Input (Host to Controller)
    /// bit 1      Output (Controller to Host)
    /// Others     Reserved for future use
    uint8_t  data_path_direction;
};

/// Command Complete event of HCI_LE_REMOVE_ISO_DATA_PATH_CMD
/*@TRACE*/
struct hci_le_remove_iso_data_path_cmd_cmp_evt
{
    /// 0x00 - Command succeeded ; 0x01-0xFF Failed reason
    uint8_t  status;
    /// Connection handle of the CIS or BIS (12 bits meaningful - Range 0x0000-0x0EFF)
    uint16_t conhdl;
};

/// Used to configure an established CIS or BIS specified by the Connection_Handle parameter, and transmit test payloads which are generated by the Controller.
/*@TRACE*/
struct hci_le_iso_tx_test_cmd
{
    /// Connection handle of the CIS or BIS (12 bits meaningful - Range 0x0000-0x0EFF)
    uint16_t conhdl;
    /// 0x00: Zero length payload 0x01: Variable length payload 0x02: Maximum length payload
    /// All other values: RFU
    uint8_t  payload_type;
};

/// Used to configure an established CIS or a synchronized BIG specified by the Connection_Handle parameter to receive payloads.
/*@TRACE*/
struct hci_le_iso_rx_test_cmd
{
    /// Connection handle of the CIS or BIS. (Range 0x0000-0x0EFF)
    uint16_t conhdl;
    /// 0x00: Zero length payload 0x01: Variable length payload 0x02: Maximum length payload
    /// All other values: RFU
    uint8_t  payload_type;
};

/// Used to read the test counters in the Controller which are configured in ISO Receive Test mode for a CIS or BIS specified by the Connection_Handle.
/*@TRACE*/
struct hci_le_iso_read_test_counters_cmd
{
    /// Connection handle of the CIS or BIS. (Range 0x0000-0x0EFF)
    uint16_t conhdl;
};

/// Command Complete event of HCI_LE_ISO_READ_TEST_COUNTERS_CMD
/*@TRACE*/
struct hci_le_iso_read_test_counters_cmd_cmp_evt
{
    /// 0x00 - Command succeeded ; 0x01-0xFF Failed reason
    uint8_t  status;
    /// Connection handle of the CIS or BIS (12 bits meaningful - Range 0x0000-0x0EFF)
    uint16_t conhdl;
    /// Number in the Received_Packet_Count (0xXXXXXXXX)
    uint32_t received_packet_count;
    /// Number in the Missed_Packet_Count (0xXXXXXXXX)
    uint32_t missed_packet_count;
    /// Number in the Failed_Packet_Count (0xXXXXXXXX)
    uint32_t failed_packet_count;
};

/// Used to terminate the ISO Transmit and/or Receive Test mode for a CIS or BIS specified by the Connection_Handle parameter but does not terminate the CIS or BIS.
/*@TRACE*/
struct hci_le_iso_test_end_cmd
{
    /// Connection handle of the CIS or BIS. (Range 0x0000-0x0EFF)
    uint16_t conhdl;
};

/// Command Complete event of HCI_LE_ISO_TEST_END_CMD
/*@TRACE*/
struct hci_le_iso_test_end_cmd_cmp_evt
{
    /// 0x00 - Command succeeded ; 0x01-0xFF Failed reason
    uint8_t  status;
    /// Connection handle of the CIS or BIS (12 bits meaningful - Range 0x0000-0x0EFF)
    uint16_t conhdl;
    /// Number in the Received_Packet_Count (0xXXXXXXXX)
    uint32_t received_packet_count;
    /// Number in the Missed_Packet_Count (0xXXXXXXXX)
    uint32_t missed_packet_count;
    /// Number in the Failed_Packet_Count (0xXXXXXXXX)
    uint32_t failed_packet_count;
};

/// Returns the values of various counters related to link quality that are associated with the isochronous stream specified by the Connection_Handle parameter.
/*@TRACE*/
struct hci_le_rd_iso_link_quality_cmd
{
    /// Connection handle of the CIS or BIS. (Range 0x0000-0x0EFF)
    uint16_t conhdl;
};
#endif // (BLE_CIS || BLE_BIS)

/// Used by the Host to set or clear a bit of the feature controlled by the Host in the Link Layer FeatureSet stored in the Controller.
struct hci_le_set_host_feature_cmd
{
    /// Bit position in the FeatureSet. (Range 0x00-0x3F)
    uint8_t bit_number;
    /// Bit value. (0x00: feature disabled 0x01: feature enabled All other values: RFU)
    uint8_t bit_value;
};

#if (BLE_CIS || BLE_BIS)
/// Command Complete event of HCI_LE_RD_ISO_LINK_QUALITY_CMD
/*@TRACE*/
struct hci_le_rd_iso_link_quality_cmd_cmp_evt
{
    /// 0x00 - Command succeeded ; 0x01-0xFF Failed reason
    uint8_t  status;
    /// Connection handle of the CIS or BIS (12 bits meaningful - Range 0x0000-0x0EFF)
    uint16_t conhdl;
    /// Value of the Tx_UnACKed_Packets counter (0xXXXXXXXX)
    uint32_t tx_unacked_packets;
    /// Value of the Tx_Flushed_Packets counter (0xXXXXXXXX)
    uint32_t tx_flushed_packets;
    /// Value of the Tx_Last_Subevent_Packets counter (0xXXXXXXXX)
    uint32_t tx_last_subevent_packets;
    /// Value of the Retransmitted_Packets counter (0xXXXXXXXX)
    uint32_t retransmitted_packets;
    /// Value of the CRC_Error_Packets counter (0xXXXXXXXX)
    uint32_t crc_error_packets;
    /// Value of the Rx_Unreceived_Packets counter (0xXXXXXXXX)
    uint32_t rx_unreceived_packets;
    /// Value of the Duplicate_Packets counter (0xXXXXXXXX)
    uint32_t duplicate_packets;
};
#endif // (BLE_CIS || BLE_BIS)

/**************************************************************************************
 **************                   HCI EVENT TRUCTURES                  ****************
 **************************************************************************************/

/// Basic LE event sutrcture
/*@TRACE*/
struct hci_le_basic_evt
{
    /// LE Subevent code
    uint8_t subcode;
};

#if (BLE_CIS)
/// Event indicates that the Connected Isochronous Stream with the Connection_Handle has been established.
/*@TRACE*/
struct hci_le_cis_established_evt
{
    ///LE Subevent code
    uint8_t  subcode;
    /// 0x00 - The Broadcast Isochronous Group has been completed ; 0x01-0xFF Failed reason
    uint8_t  status;
    /// Connection handle of the Connected Isochronous Stream (Range: 0x0000-0x0EFF)
    uint16_t conhdl;
    /// The CIG synchronization delay time in microseconds
    uint32_t cig_sync_delay;
    /// The CIS synchronization delay time in microseconds
    uint32_t cis_sync_delay;
    /// The maximum time, in microseconds, for transmission of SDUs of all CISes from master to slave
    /// (range 0x0000EA to 0x7FFFFF)
    uint32_t trans_latency_m2s;
    /// The maximum time, in microseconds, for transmission of SDUs of all CISes from slave to master
    /// (range 0x0000EA to 0x7FFFFF)
    uint32_t trans_latency_s2m;
    /// Master to slave PHY, 0x01: 1Mbps, 0x02: 2Mbps, 0x03: LE-Coded
    uint8_t  phy_m2s;
    /// Slave to master PHY, 0x01: 1Mbps, 0x02: 2Mbps, 0x03: LE-Coded
    uint8_t  phy_s2m;
    /// Maximum number of subevents in each isochronous event (Range: 0x01-0x1E)
    uint8_t  nse;
    /// The burst number for master to slave transmission (0x00: no isochronous data from the master to the slave, range 0x01-0x0F)
    uint8_t  bn_m2s;
    /// The burst number for slave to master transmission (0x00: no isochronous data from the slave to the master, range 0x01-0x0F)
    uint8_t  bn_s2m;
    /// The flush timeout, in multiples of the ISO_Interval, for each payload sent from the master to the slave (Range: 0x01-0x1F)
    uint8_t  ft_m2s;
    /// The flush timeout, in multiples of the ISO_Interval, for each payload sent from the slave to the master (Range: 0x01-0x1F)
    uint8_t  ft_s2m;
    /// Maximum size, in octets, of the payload from master to slave (Range: 0x00-0xFB)
    uint16_t  max_pdu_m2s;
    /// Maximum size, in octets, of the payload from slave to master (Range: 0x00-0xFB)
    uint16_t  max_pdu_s2m;
    /// ISO interval (1.25ms unit, range: 5ms to 4s)
    uint16_t iso_interval;
};

/// Event indicates that the Connected Isochronous Stream with the Connection_Handle has been established.
/*@TRACE*/
struct hci_le_cis_request_evt
{
    ///LE Subevent code
    uint8_t  subcode;
    /// Connection handle of the ACL (Range: 0x0000-0x0EFF)
    uint16_t acl_conhdl;
    /// Connection handle of the Connected Isochronous Stream (Range: 0x0000-0x0EFF)
    uint16_t cis_conhdl;
    /// Used to identify the Connected Isochronous Group. (Range 0x00-0xEF)
    uint8_t  cig_id;
    /// Used to identify a Connected Isochronous Stream. (Range 0x00-0xEF)
    uint8_t  cis_id;
};
#endif // (BLE_CIS)

#if (BLE_BIS)
/// Event Generated in the Isochronous Broadcaster when all the Broadcast Isochronous Streams in the Group are being
/// transmitted
/*@TRACE*/
struct hci_le_create_big_cmp_evt
{
    ///LE Subevent code
    uint8_t  subcode;
    /// 0x00 - The Broadcast Isochronous Group has been completed ; 0x01-0xFF Failed reason
    uint8_t  status;
    /// BIG_Handle is used to identify the BIS Group. (Range 0x00-0xEF)
    uint8_t  big_hdl;
    /// Transmission delay time in microseconds of all BISs in the BIG (in us range 0x0000EA-0x7FFFFF)
    uint32_t big_sync_delay;
    /// The maximum delay time, in microseconds, for transmission of SDUs of all BISes (in us range 0x0000EA-0x7FFFFF)
    uint32_t big_trans_latency;
    /// PHY used, bit 0: 1Mbps, bit 1: 2Mbps, bit 2: LE-Coded
    uint8_t  phy;
    /// The number of subevents in each BIS event in the BIG, range 0x01-0x1E
    uint8_t  nse;
    /// The number of new payloads in each BIS event, range 0x01-0x07
    uint8_t  bn;
    /// Offset used for pre-transmissions, range 0x00-0x0F
    uint8_t  pto;
    /// The number of times a payload is transmitted in a BIS event, range 0x01-0x0F
    uint8_t  irc;
    /// Maximum size of the payload in octets, range 0x00-0xFB
    uint16_t  max_pdu;
    /// ISO interval (1.25ms unit, range: 5ms to 4s)
    uint16_t iso_interval;
    /// Total number of BISs in the BIG (Range 0x01-0x1F)
    uint8_t  num_bis;
    /// The connection handles of the BISs (Range 0x0000-0x0EFF)
    uint16_t conhdl[0x1F];
};

/// Event is generated when transmissions of all the Broadcast Isochronous Streams in the Broadcast Isochronous Group
/// are terminated
/*@TRACE*/
struct hci_le_terminate_big_cmp_evt
{
    ///LE Subevent code
    uint8_t  subcode;
    /// BIG_Handle is used to identify the BIS Group. (Range 0x00-0xEF)
    uint8_t  big_hdl;
    /// Reason for termination (0xXX)
    uint8_t  reason;
};

/// Event is generated in a scanning device when the Controller has synchronized with the requested Broadcast
/// Isochronous Streams.
/*@TRACE*/
struct hci_le_big_sync_est_evt
{
    ///LE Subevent code
    uint8_t  subcode;
    /// 0x00 - The Broadcast Isochronous Group has been completed ; 0x01-0xFF Failed reason
    uint8_t  status;
    /// BIG_Handle is used to identify the BIS Group. (Range 0x00-0xEF)
    uint8_t  big_hdl;
    /// The maximum delay time, in microseconds, for transmission of SDUs of all BISes (in us range 0x0000EA-0x7FFFFF)
    uint32_t big_trans_latency;
    /// The number of subevents in each BIS event in the BIG, range 0x01-0x1E
    uint8_t  nse;
    /// The number of new payloads in each BIS event, range 0x01-0x07
    uint8_t  bn;
    /// Offset used for pre-transmissions, range 0x00-0x0F
    uint8_t  pto;
    /// The number of times a payload is transmitted in a BIS event, range 0x01-0x0F
    uint8_t  irc;
    /// Maximum size of the payload in octets, range 0x00-0xFB
    uint16_t  max_pdu;
    /// ISO interval (1.25ms unit, range: 5ms to 4s)
    uint16_t iso_interval;
    /// Total number of BISs in the BIG (Range 0x01-0x1F)
    uint8_t  num_bis;
    /// The connection handles of the BISs (Range 0x0000-0x0EFF)
    uint16_t conhdl[0x1F];
};

/// Event indicates that the Controller has not received any payload from any Broadcast Isochronous Stream within a
/// Broadcast Isochronous Group to which it was synchronized within the timeout period (BIS_Sync_Timeout).
/*@TRACE*/
struct hci_le_big_sync_lost_evt
{
    ///LE Subevent code
    uint8_t  subcode;
    /// BIG_Handle is used to identify the BIS Group. (Range 0x00-0xEF)
    uint8_t  big_hdl;
    /// Reason of synchronization to BIG termination
    uint8_t  reason;
};
#endif // (BLE_BIS)

/// Event indicates that the HCI_LE_Request_Peer_SCA command has been completed.
/*@TRACE*/
struct hci_le_req_peer_sca_cmp_evt
{
    ///LE Subevent code
    uint8_t  subcode;
    /// 0x00 - The Peer_Clock_Accuracy parameter is successfully received ; 0x01-0xFF Failed reason
    uint8_t  status;
    /// Connection handle of the ACL (12 bits meaningful Range 0x0000-0x0EFF)
    uint16_t conhdl;
    /// Peer clock accuracy (@see enum SCA)
    uint8_t  peer_clock_accuracy;
};

#if (BLE_BIS)
/// Event is generated in a scanning device when the Controller has received an advertising PDU that contained a BIGInfo field.
/*@TRACE*/
struct hci_le_big_info_adv_report_evt
{
    ///LE Subevent code
    uint8_t  subcode;
    /// Sync_Handle identifying the periodic advertising train (Range 0x0000-0x0EFF)
    uint16_t  sync_hdl;
    /// Value of the Num_BIS subfield of the BIGInfo field (Range 0x01-0x1F)
    uint8_t  num_bis;
    /// Value of the NSE subfield of the BIGInfo field (Range 0x01-0x1F)
    uint8_t  nse;
    /// Value of the ISO_Interval subfield of the BIGInfo field (0xXXXX)
    uint16_t  iso_interval;
    /// Value of the BN subfield of the BIGInfo field (Range 0x01-0x07)
    uint8_t  bn;
    /// Value of the PTO subfield of the BIGInfo field (Range 0x00-0x0F)
    uint8_t  pto;
    /// Value of the IRC subfield of the BIGInfo field (Range 0x01-0x0F)
    uint8_t  irc;
    /// Value of the Max_PDU subfield of the BIGInfo field (Range 0x0000-0x00FB)
    uint16_t  max_pdu;
    /// Value of the SDU_Interval subfield of the BIGInfo field (Range 0x0000FF-0x0FFFFF)
    uint8_t  sdu_interval[3];
    /// Value of the Max_SDU subfield of the BIGInfo field (Range 0x0000-0x0FFF)
    uint16_t max_sdu;
    /// Value of the PHY subfield of the BIGInfo field (0x01: 1M, 0x02: 2M, 0x03: Coded, All other values: RFU)
    uint8_t  phy;
    /// Value of the Framing subfield of the BIGInfo field (0x00: Unframed, 0x01: Framed, All other values: RFU)
    uint8_t  framing;
    /// Value of the Encryption subfield of the BIGInfo field (0x00: Unencrypted, 0x01: Encrypted, All other values: RFU)
    uint8_t  encryption;
};
#endif // (BLE_BIS)

#if (EAVESDROPPING_SUPPORT)
/******************************************************************************/
/* ----------------------   EAVESDROPPING COMMANDS     -----------------------*/
/******************************************************************************/

/// HCI ECI eavesdropping operation mode command parameters
struct hci_eci_set_operational_mode_cmd
{
    /// Eavesdropping operation mode (ED_OP_MODE_NE = 0x00, ED_OP_MODE_EDE = 0x01, ED_OP_MODE_ED = 0x02)
    uint8_t             mode;
    /// EDE BD address (Needed in ED master role for proper frequency hopping)
    struct bd_addr      ede_addr;
};

/// HCI ECI set power command parameters
struct hci_eci_set_power_cmd
{
    /// BD address to set
    struct bd_addr      addr;
    /// Power adjustment (PWR_ADJ_REQ_DEC_1_STEP = 0x00, PWR_ADJ_REQ_INC_1_STEP = 0x01, PWR_ADJ_REQ_INC_MAX = 0x02)
    uint8_t             pow_adj;
};

/// HCI ECI set channel classification command parameters
struct hci_eci_set_channel_classification_cmd
{
    /// BD address to set
    struct bd_addr      addr;
    /// AFH channel map (10 bytes, 79 usefull bits for every channel)
    struct bt_ch_map    ch_map;
};

/// HCI ECI capture master clock command parameters
struct hci_eci_capture_master_clock_cmd
{
    /// BD address to set
    struct bd_addr      addr;
    /// Capture mode (ED_CAPT_MODE_SW = 0x00, ED_CAPT_MODE_HW = 0x01)
    uint8_t             mode;
};

/// HCI ECI get AES CCM counter command parameters
struct hci_eci_get_aes_ccm_counter_cmd
{
    /// BD address to set
    struct bd_addr      addr;
    /// Connection type (ED_CON_TYPE_ACL = 0x00, ED_CON_TYPE_ESCO = 0x01)
    uint8_t             con_type;
};

/// HCI ECI get AES CCM counter command reply parameters
struct hci_eci_get_aes_ccm_counter_cmd_cmp_evt
{
    /// Status (CO_ERROR_NO_ERROR = 0x00, CO_ERROR_COMMAND_DISALLOWED = 0x0C, CO_ERROR_INVALID_HCI_PARAM = 0x12)
    uint8_t             status;
    /// Connection type (ED_CON_TYPE_ACL = 0x00, ED_CON_TYPE_ESCO = 0x01)
    uint8_t             con_type;
    /// Counter content
    uint8_t             cntr[5];
};

/// HCI ECI set eavesdropping state command parameters
struct hci_eci_set_eavesdropping_state_cmd
{
    /// BD address to set
    struct bd_addr      addr;
    /// Enabled or disabled (ED_DISABLED = 0x00, ED_ENABLED = 0x01)
    uint8_t             enabled;
    /// Connection type (ED_CON_TYPE_ACL = 0x00, ED_CON_TYPE_ESCO = 0x01)
    uint8_t             con_type;
};

/// HCI ECI set ACL connection command parameters
struct hci_eci_set_acl_connection_cmd
{
    /// BD address to set
    struct bd_addr      addr;
    /// ACL status (ED_STATUS_NOT_CONNECTED = 0x00, ED_STATUS_CONNECTED = 0x01)
    uint8_t             status;
    /// ACL role (MASTER_ROLE = 0x00, SLAVE_ROLE = 0x01)
    uint8_t             role;
    /// ACL LT address (0x01..0x07)
    uint8_t             lt_addr;
};

/// HCI ECI set AFH channel map command parameters
struct hci_eci_set_afh_channel_map_cmd
{
    /// BD address to set
    struct bd_addr      addr;
    /// AFH enabled (ED_DISABLED = 0x00, ED_ENABLED = 0x01)
    uint8_t             enabled;
    /// AFH time (BT master clk value at the time of switch seq, bits 27:1, shall be even)
    uint32_t            time;
    /// AFH channel map (10 bytes, 79 usefull bits for every channel)
    struct bt_ch_map    ch_map;
};

/// HCI ECI set encryption command parameters
struct hci_eci_set_encryption_cmd
{
    /// BD address to set
    struct bd_addr                 addr;
    /// Encryption type (ED_ENC_TYPE_NONE = 0x00, ED_ENC_TY0E_E0 = 0x01, ED_ENC_TYPE_AES = 0x02, ED_ENC_TYPE_PAUSED = 0x03)
    uint8_t                        type;
    /// Encryption key (Long term key, 16 bytes)
    struct ltk                     key;
    /// Encryption key size (1..16)
    uint8_t                        key_size;
    /// Encryption initialization vector (8 bytes)
    struct initialization_vector   iv;
};

/// HCI ECI set packet type table command parameters
struct hci_eci_set_packet_type_table_cmd
{
    /// BD address to set
    struct bd_addr      addr;
    /// Packet type table (PACKET_TABLE_1MBPS = 0x00, PACKET_TABLE_2_3MBPS = 0x01)
    uint8_t             table;
};

/// HCI ECI set max slot command parameters
struct hci_eci_set_max_slot_cmd
{
    /// BD address to set
    struct bd_addr      addr;
    /// Max slot (Max number of slots that phone may use to transmit: 1, 3 or 5)
    uint8_t             max_slot;
};

/// HCI ECI set SCO connection command parameters
struct hci_eci_set_sco_connection_cmd
{
    /// BD address to set
    struct bd_addr      addr;
    /// SCO status (ED_STATUS_NOT_CONNECTED = 0x00, ED_STATUS_CONNECTED = 0x01)
    uint8_t             status;
    /// SCO type (SCO_TYPE = 0x00, ESCO_TYPE = 0x02)
    uint8_t             type;
    /// SCO LT address (0x01..0x07)
    uint8_t             lt_addr;
    /// SCO timing flags (ED_SCO_TF_NO_TIMING_CHANGE = 0x00, ED_SCO_TF_TIMING_CHANGE = 0x01, ED_SCO_TF_USE_INIT1 = 0x00, ED_SCO_TF_USE_INIT2 = 0x02, ED_SCO_TF_ACC_WIN = 0x00, ED_SCO_TF_NO_ACC_WIN = 0x04)
    uint8_t             timing_flags;
    /// Synchronous link offset (in slots, even values only, less than TeSCO)
    uint8_t             d;
    /// Synchronous link interval (in slots, 4..254, even values only)
    uint8_t             t;
    /// Retransmission window size (in slots, eSCO only, 0..254)
    uint8_t             w;
    /// SCO packet type M (ESCO_PACKET_NULL = 0x00, ESCO_PACKET_EV3 = 0x07, ESCO_PACKET_EV4 = 0x0C, ESCO_PACKET_EV5 = 0x0D, ESCO_PACKET_EV3_2 = 0x26, ESCO_PACKET_EV5_2 = 0x2C, ESCO_PACKET_EV3_3 = 0x37, ESCO_PACKET_EV5_3 = 0x3D)
    uint8_t             type_m;
    /// SCO packet type S (ESCO_PACKET_NULL = 0x00, ESCO_PACKET_EV3 = 0x07, ESCO_PACKET_EV4 = 0x0C, ESCO_PACKET_EV5 = 0x0D, ESCO_PACKET_EV3_2 = 0x26, ESCO_PACKET_EV5_2 = 0x2C, ESCO_PACKET_EV3_3 = 0x37, ESCO_PACKET_EV5_3 = 0x3D)
    uint8_t             type_s;
    /// SCO packet length M (Depends on packet type. EV3: 1..30, EV4: 1..120, EV5: 1..180, 2-EV3: 1..60, 2-EV5: 1..360, 3-EV3: 1..90, 3-EV5: 1..540)
    uint16_t            len_m;
    /// SCO packet length S (Depends on packet type. EV3: 1..30, EV4: 1..120, EV5: 1..180, 2-EV3: 1..60, 2-EV5: 1..360, 3-EV3: 1..90, 3-EV5: 1..540)
    uint16_t            len_s;
    /// SCO air mode (MU_LAW_MODE = 0, A_LAW_MODE = 1, CVSD_MODE = 2, TRANS_MODE = 3)
    uint8_t             air_mode;
};

/// HCI ECI set min/max power command parameters
struct hci_eci_set_min_max_power_cmd
{
    /// BD address to set
    struct bd_addr      addr;
    /// Power level (ED_MIN_POWER = 0, ED_MAX_POWER = 1)
    uint8_t             level;
};

/// HCI ECI set master clock command parameters
struct hci_eci_set_master_clock_cmd
{
    /// BD address to set
    struct bd_addr      addr;
    /// Bluetooth clock value in slots
    uint32_t            clkn;
    /// Bluetooth fine timer counter in us
    uint16_t            fine_cnt;
    /// Bluetooth clock offset value in slots
    uint32_t            clk_off;
    /// Bluetooth fine timer counter offset value in us
    uint16_t            bit_off;
};

/// HCI ECI set AES CCM counter command parameters
struct hci_eci_set_aes_ccm_counter_cmd
{
    /// BD address to set
    struct bd_addr      addr;
    /// Connection type (ED_CON_TYPE_ACL = 0x00, ED_CON_TYPE_ESCO = 0x01)
    uint8_t             con_type;
    /// Day counter of an eSCO connection
    uint8_t             cntr[5];
};

struct hci_eci_set_clock_offsets_cmd
{
    /// BD address to set
    struct bd_addr      addr;
    /// Bluetooth clock offset value in slots
    uint32_t            clk_off;
    /// Bluetooth fine timer counter offset value in us
    uint16_t            bit_off;
};

/// HCI ECI set LRD eavesdropping command parameters
struct hci_eci_set_lrd_eavesdropping_cmd
{
    /// BD address to set
    struct bd_addr      addr;
    /// Enabled or disabled (ED_DISABLED = 0x00, ED_ENABLED = 0x01)
    uint8_t             enabled;
    /// Connection type (ED_CON_TYPE_ACL = 0x00, ED_CON_TYPE_ESCO = 0x01)
    uint8_t             con_type;
    /// Control Tx Prog (DIS = 0x00, EN = 0x01)
    uint8_t             control_tx_prog;
    /// Ignore SEQN Errors (DIS = 0x00, EN = 0x01)
    uint8_t             ignore_seqn_errors;
    /// L2CAP channel ID
    uint16_t            l2cap_cid;
    /// NULL filtering (DIS = 0x00, EN = 0x01)
    uint8_t             null_flt;
};

struct hci_eci_allow_tx_prog_cmd
{
    /// BD address to set
    struct bd_addr      addr;
};

struct hci_eci_set_tx_seqn_cmd
{
    /// BD address to set
    struct bd_addr      addr;
    /// Tx sequence number to set
    uint8_t             tx_seqn;
};

struct hci_eci_deliver_corr_acl_payl_cmd
{
    /// BD address to set
    struct bd_addr      addr;
    /// Enable or disable
    uint8_t             enable;
};

struct hci_eci_rx_stats_reporting_cmd
{
    /// BD address to set
    struct bd_addr      addr;
    /// Enable or disable
    uint8_t             enable;
    /// Connection type (ED_CON_TYPE_ACL = 0x00, ED_CON_TYPE_ESCO = 0x01)
    uint8_t             con_type;
    /// Reporting interval in slots (625 us)
    uint8_t             intv;
};

struct hci_eci_sample_esco_tx_seqn_cmd
{
    /// BD address to set
    struct bd_addr      bd_addr;
};

struct hci_eci_sync_esco_tx_seqn_cmd
{
    /// BD address to set
    struct bd_addr      bd_addr;
    /// eSCO Tx SEQN
    uint8_t             esco_tx_seqn;
    /// Master clock corresponding to the eSCO Tx SEQN
    uint32_t            clkn;
};


/******************************************************************************/
/* -----------------------   EAVESDROPPING EVENTS     ------------------------*/
/******************************************************************************/

/// HCI vendor specific ECI, ACL connection changed
struct hci_eci_acl_connection_changed_evt
{
    /// Vendor specific ED event subcode (0x80)
    uint8_t             subcode;
    /// BD address to set
    struct bd_addr      addr;
    /// ACL status (ED_STATUS_NOT_CONNECTED = 0x00, ED_STATUS_CONNECTED = 0x01)
    uint8_t             status;
    /// ACL role (MASTER_ROLE = 0x00, SLAVE_ROLE = 0x01)
    uint8_t             role;
    /// ACL LT address (0x01..0x07)
    uint8_t             lt_addr;
};

/// HCI vendor specific ECI, AFH channel map changed
struct hci_eci_afh_channel_map_changed_evt
{
    /// Vendor specific ED event subcode (0x81)
    uint8_t             subcode;
    /// BD address to set
    struct bd_addr      addr;
    /// AFH enabled (ED_DISABLED = 0x00, ED_ENABLED = 0x01)
    uint8_t             enabled;
    /// AFH time (BT master clk value at the time of switch seq, bits 27:1, shall be even)
    uint32_t            time;
    /// AFH channel map (10 bytes, 79 usefull bits for every channel)
    struct bt_ch_map    ch_map;
};

/// HCI vendor specific ECI, encryption changed
struct hci_eci_encryption_changed_evt
{
    /// Vendor specific ED event subcode (0x82)
    uint8_t                        subcode;
    /// BD address to set
    struct bd_addr                 addr;
    /// Encryption type (ED_ENC_TYPE_NONE = 0x00, ED_ENC_TY0E_E0 = 0x01, ED_ENC_TYPE_AES = 0x02, ED_ENC_TYPE_PAUSED = 0x03)
    uint8_t                        type;
    /// Encryption key (Long term key, 16 bytes)
    struct ltk                     key;
    /// Encryption key size (1..16)
    uint8_t                        key_size;
    /// Encryption initialization vector (8 bytes)
    struct initialization_vector   iv;
};

/// HCI vendor specific ECI, packet type table changed
struct hci_eci_packet_type_table_changed_evt
{
    /// Vendor specific ED event subcode (0x83)
    uint8_t             subcode;
    /// BD address to set
    struct bd_addr      addr;
    /// Packet type table (PACKET_TABLE_1MBPS = 0x00, PACKET_TABLE_2_3MBPS = 0x01)
    uint8_t             table;
};

/// HCI vendor specific ECI, max slot changed
struct hci_eci_max_slot_changed_evt
{
    /// Vendor specific ED event subcode (0x84)
    uint8_t             subcode;
    /// BD address to set
    struct bd_addr      addr;
    /// Max slot (Max number of slots that phone may use to transmit: 1, 3 or 5)
    uint8_t             max_slot;
};

/// HCI vendor specific ECI, SCO connection changed
struct hci_eci_sco_connection_changed_evt
{
    /// Vendor specific ED event subcode (0x85)
    uint8_t             subcode;
    /// BD address to set
    struct bd_addr      addr;
    /// SCO status (ED_STATUS_NOT_CONNECTED = 0x00, ED_STATUS_CONNECTED = 0x01)
    uint8_t             status;
    /// SCO type (SCO_TYPE = 0x00, ESCO_TYPE = 0x02)
    uint8_t             type;
    /// SCO LT address (0x01..0x07)
    uint8_t             lt_addr;
    /// SCO timing flags (ED_SCO_TF_NO_TIMING_CHANGE = 0x00, ED_SCO_TF_TIMING_CHANGE = 0x01, ED_SCO_TF_USE_INIT1 = 0x00, ED_SCO_TF_USE_INIT2 = 0x02, ED_SCO_TF_ACC_WIN = 0x00, ED_SCO_TF_NO_ACC_WIN = 0x04)
    uint8_t             timing_flags;
    /// Synchronous link offset (in slots, even values only, less than TeSCO)
    uint8_t             d;
    /// Synchronous link interval (in slots, 4..254, even values only)
    uint8_t             t;
    /// Retransmission window size (in slots, eSCO only, 0..254)
    uint8_t             w;
    /// SCO packet type M (ESCO_PACKET_NULL = 0x00, ESCO_PACKET_EV3 = 0x07, ESCO_PACKET_EV4 = 0x0C, ESCO_PACKET_EV5 = 0x0D, ESCO_PACKET_EV3_2 = 0x26, ESCO_PACKET_EV5_2 = 0x2C, ESCO_PACKET_EV3_3 = 0x37, ESCO_PACKET_EV5_3 = 0x3D)
    uint8_t             type_m;
    /// SCO packet type S (ESCO_PACKET_NULL = 0x00, ESCO_PACKET_EV3 = 0x07, ESCO_PACKET_EV4 = 0x0C, ESCO_PACKET_EV5 = 0x0D, ESCO_PACKET_EV3_2 = 0x26, ESCO_PACKET_EV5_2 = 0x2C, ESCO_PACKET_EV3_3 = 0x37, ESCO_PACKET_EV5_3 = 0x3D)
    uint8_t             type_s;
    /// SCO packet length M (Depends on packet type. EV3: 1..30, EV4: 1..120, EV5: 1..180, 2-EV3: 1..60, 2-EV5: 1..360, 3-EV3: 1..90, 3-EV5: 1..540)
    uint16_t            len_m;
    /// SCO packet length S (Depends on packet type. EV3: 1..30, EV4: 1..120, EV5: 1..180, 2-EV3: 1..60, 2-EV5: 1..360, 3-EV3: 1..90, 3-EV5: 1..540)
    uint16_t            len_s;
    /// SCO air mode (MU_LAW_MODE = 0, A_LAW_MODE = 1, CVSD_MODE = 2, TRANS_MODE = 3)
    uint8_t             air_mode;
};

/// HCI vendor specific ECI, min/max power reached
struct hci_eci_min_max_power_reached_evt
{
    /// Vendor specific ED event subcode (0x86)
    uint8_t             subcode;
    /// BD address to set
    struct bd_addr      addr;
    /// Power level (ED_MIN_POWER = 0, ED_MAX_POWER = 1,ED_INTERM_POWER = 2)
    uint8_t             level;
};

/// HCI vendor specific ECI, master clock captured
struct hci_eci_master_clock_captured_evt
{
    /// Vendor specific ED event subcode (0x87)
    uint8_t             subcode;
    /// BD address to set
    struct bd_addr      addr;
    /// Bluetooth clock value in slots
    uint32_t            clkn;
    /// Bluetooth fine timer counter in us
    uint16_t            fine_cnt;
    /// Bluetooth clock offset value in slots
    uint32_t            clk_off;
    /// Bluetooth fine timer counter offset value in us
    uint16_t            bit_off;
};

/// HCI vendor specific ECI, AES CCM Counter
struct hci_eci_aes_ccm_counter_evt
{
    /// Vendor specific ED event subcode (0x88)
    uint8_t             subcode;
    /// BD address to set
    struct bd_addr      addr;
    /// Connection type (ED_CON_TYPE_ACL = 0x00, ED_CON_TYPE_ESCO = 0x01)
    uint8_t             con_type;
    /// Counter, 5 bytes
    uint8_t             cntr[5];
    /// MIC ok (true or false)
    bool                mic_ok;
};

/// HCI vendor specific ECI, AES CCM Counter
struct hci_eci_eavesdropping_state_changed_evt
{
    /// Vendor specific ED event subcode (0x94)
    uint8_t             subcode;
    /// BD address to set
    struct bd_addr      addr;
    /// Enabled or disabled (ED_DISABLED = 0x00, ED_ENABLED = 0x01)
    uint8_t             enabled;
    /// Connection type (ED_CON_TYPE_ACL = 0x00, ED_CON_TYPE_ESCO = 0x01)
    uint8_t             con_type;
};

/// HCI vendor specific ECI, adjust power
struct hci_eci_adjust_power_evt
{
    /// Vendor specific ED event subcode (0x89)
    uint8_t             subcode;
    /// BD address to set
    struct bd_addr      addr;
    /// Power adjustment (PWR_ADJ_REQ_DEC_1_STEP = 0x00, PWR_ADJ_REQ_INC_1_STEP = 0x01, PWR_ADJ_REQ_INC_MAX = 0x02)
    uint8_t             adj;
};

/// HCI vendor specific ECI, channel classification
struct hci_eci_channel_classification_evt
{
    /// Vendor specific ED event subcode (0x8A)
    uint8_t             subcode;
    /// BD address to set
    struct bd_addr      addr;
    /// AFH channel map (10 bytes, 79 usefull bits for every channel)
    struct bt_ch_map    ch_map;
};

/// HCI vendor specific ECI, master clock adjusted
struct hci_eci_master_clock_adjusted_evt
{
    /// Vendor specific ED event subcode (0x8B)
    uint8_t             subcode;
    /// BD address to set
    struct bd_addr      addr;
};

/// HCI vendor specific ECI, Rx timeout
struct hci_eci_rx_timout_evt
{
    /// Vendor specific ED event subcode (0x8C)
    uint8_t             subcode;
    /// BD address to set
    struct bd_addr      addr;
};

/// HCI vendor specific ECI, invalid MIC
struct hci_eci_mic_invalid_evt
{
    /// Vendor specific ED event subcode (0x8D)
    uint8_t             subcode;
    /// BD address to set
    struct bd_addr      addr;
};

/// ECI clear capture master clock command complete event
struct hci_eci_clear_capture_master_clock_cmd_cmp_evt
{
    ///Status
    uint8_t status;
    ///Cleared
    uint8_t cleared;
};

/// HCI vendor specific ECI, Tx pending
struct hci_eci_tx_pending_evt
{
    /// Vendor specific ED event subcode (0x93)
    uint8_t             subcode;
    /// BD address to set
    struct bd_addr      addr;
    /// Tx pending (true or false)
    bool                tx_pending;
    /// Tx sequence number
    uint8_t             tx_seqn;
};

/// HCI vendor specific ECI, Rx stats
struct hci_eci_rx_stats_evt
{
    /// Vendor specific ED event subcode (0x95)
    uint8_t             subcode;
    /// BD address to set
    struct bd_addr      addr;
    /// Master clock at which the Rx statistics have been collected
    uint32_t            clkn;
    /// Total number of reception attempts
    uint8_t             rx_tot;
    /// Number of synchronized reception attempts
    uint8_t             rx_sync;
    /// Rx stats array
    uint8_t             rx_stats[HCI_ECI_RX_STATS_LEN_MAX];
};

/// HCI vendor specific ECI, eSCO Tx SEQN sampled
struct hci_eci_esco_tx_seqn_sampled_evt
{
    /// Vendor specific ED event subcode (0x98)
    uint8_t             subcode;
    /// BD address to set
    struct bd_addr      bd_addr;
    /// Status
    uint8_t             status;
    /// eSCO Tx SEQN
    uint8_t             esco_tx_seqn;
    /// Master clock at which the eSCO Tx SEQN has been sampled
    uint32_t            clkn;
};

/// HCI vendor specific ECI, eSCO Tx SEQN synced
struct hci_eci_esco_tx_seqn_synced_evt
{
    /// Vendor specific ED event subcode (0x99)
    uint8_t             subcode;
    /// BD address to set
    struct bd_addr      bd_addr;
    /// Status
    uint8_t             status;
};

/******************************************************************************/
/* ---------------------   VENDOR SPECIFIC COMMANDS     ----------------------*/
/******************************************************************************/

/// HCI VS set segmented inquiry scan command parameters
struct hci_vs_set_seg_inq_scan_cmd
{
    /// Enable/Disable
    uint8_t enable;

    /// Time offset defining the first scan window position, Clock % Interval = Offset (in 625us slots)
    uint16_t time_offset;

    /// Periodicity of scan segments (in 625us slots)
    uint16_t periodicity;

    /// Distance between scan segments (in 625us slots)
    uint16_t distance;

    /// Duration of each scan segment (in us)
    uint16_t duration;

    /// Number of segments in a scan window (maximum 16 supported)
    uint8_t segments;

    /// Phase offsets for each scan segment
    uint8_t phase_offsets[16];

    /// BD address of a master, if an active slave connection is present, the scan timings are based on this piconet
    struct bd_addr bd_addr;

    /// Microsecond offset
    int16_t us_offset;
};

/// HCI VS set segmented page scan command parameters
struct hci_vs_set_seg_page_scan_cmd
{
    /// Enable/Disable
    uint8_t enable;

    /// Time offset defining the first scan window position, Clock % Interval = Offset (in 625us slots)
    uint16_t time_offset;

    /// Periodicity of scan segments (in 625us slots)
    uint16_t periodicity;

    /// Distance between scan segments (in 625us slots)
    uint16_t distance;

    /// Duration of each scan segment (in us)
    uint16_t duration;

    /// Number of segments in a scan window (maximum 16 supported)
    uint8_t segments;

    /// Phase offsets for each scan segment
    uint8_t phase_offsets[16];

    /// BD address of a master, if an active slave connection is present, the scan timings are based on this piconet
    struct bd_addr bd_addr;

    /// Microsecond offset
    int16_t us_offset;
};

/// HCI VS Set Custom Advertising command parameters
struct hci_vs_set_cust_adv_cmd
{
    /// Advertising handle
    uint8_t adv_hdl;

    /// Enable/Disable
    uint8_t enable;

    /// Time offset defining the first scan window position, Clock % Interval = Offset (in 625us slots)
    uint16_t time_offset;

    /// Periodicity of advertising event, distance between event will be a multiple of this value (in 625us slots)
    /// It is intended that a advertising interval provided in HCI LE SET ADV PARAM command is a multiple of the
    /// periodicity
    uint16_t periodicity;

    /// Enable randomization
    /// 1: advDelay randomly chosen among (0ms, 2.5ms, 5ms, 7.5ms, 10ms)
    /// 0: advDelay fixed to 0
    uint8_t rand_en;

    /// BD Address - if there is an ACL connection with the device that has this address,
    /// advertising events must be aligned to the piconet clock
    struct bd_addr bd_addr;

    /// Microsecond offset
    int16_t us_offset;

    /// Indicates whether ADV_EXT_IND packets are grouped during custom advertising
    uint8_t group_ext_adv_ind;
};

/// HCI VS set custom ACL scheduling command parameters
struct hci_vs_set_cust_acl_sched_cmd
{
    /// BD address to set
    struct bd_addr      bd_addr;

    /// Enable/Disable
    uint8_t enable;

    /// Event duration for scheduling the ACL activity (in 625us slots)
    uint8_t evt_dur;
};

/// HCI VS set host max slot command parameters
struct hci_vs_set_host_max_slot_cmd
{
    /// Host max slot
    uint8_t host_max_slot;
};

/// HCI VS prevent deep sleep command parameters
struct hci_vs_prevent_deep_sleep_cmd
{
    ///Prevent/Not prevent
    uint8_t prevent;
};

enum coex_prio_bank_id
{
    /// BT nominal priorities
    COEX_BT_PRIO_NOM = 0,
    /// BT transmit priorities
    COEX_BT_PRIO_TX,
    /// BT receive priorities
    COEX_BT_PRIO_RX,
    /// BLE nominal priorities
    COEX_BLE_PRIO_NOM,
    /// BLE transmit priorities
    COEX_BLE_PRIO_TX,
    /// BLE receive priorities
    COEX_BLE_PRIO_RX,
    COEX_MAX
};

/// HCI VS set WLAN coex priorities command parameters
struct hci_vs_set_wlan_coex_priorities_cmd
{
    /// Priorities
    uint8_t priorities[COEX_MAX][24];
};

/// HCI VS set SCO link request delay command parameters
struct hci_vs_set_sco_link_request_delay_cmd
{
    /// Delay in ms
    uint16_t delay;
};

/// HCI VS set custom paging command parameters
struct hci_vs_set_cust_paging_cmd
{
    /// Enable/Disable
    uint8_t enable;
    /// Train nudging offset in slots (must be an even number, 0 means train nudging disabled)
    uint8_t train_nudging_offset;
    /// Minimum page timeout in slots
    uint16_t min_page_timeout;
};

/// HCI VS notify clock wrap command parameters
struct hci_vs_notify_clock_wrap_cmd
{
    /// Enable/Disable
    uint8_t enable;

    /// BD Address - if there is an ACL connection with the device that has this address,
    /// wrapping refers to the piconet clock
    struct bd_addr bd_addr;
};

/// HCI VS notify sniff events command parameters
struct hci_vs_notify_sniff_events_cmd
{
    /// Enable/Disable
    uint8_t enable;
};

/// HCI VS LE enable enhanced set data length command parameters
struct hci_vs_le_enable_enh_set_data_len_cmd
{
    /// Enable/Disable
    uint8_t enable;
};

/// HCI VS LE enhanced set data length command parameters
struct hci_vs_le_enh_set_data_len_cmd
{
    /// Connection handle
    uint16_t       conhdl;
    /// Preferred maximum number of Tx payload octet
    uint16_t       tx_octets;
    /// Preferred maximum number of Tx microseconds
    uint16_t       tx_time;
    /// Preferred maximum number of Rx payload octets
    uint16_t       rx_octets;
    /// Preferred maximum number of Rx microseconds
    uint16_t       rx_time;
};

/// HCI VS LE write suggested default data length command parameters
struct hci_vs_le_enh_wr_suggted_dft_data_len_cmd
{
    /// Preferred maximum number of Tx payload octets
    uint16_t       suggted_max_tx_octets;
    /// Preferred maximum number of Tx microseconds
    uint16_t       suggted_max_tx_time;
    /// Preferred maximum number of Rx payload octets
    uint16_t       suggted_max_rx_octets;
    /// Preferred maximum number of Rx microseconds
    uint16_t       suggted_max_rx_time;
};

/// HCI VS LE set custom transmitter test command parameters
struct hci_vs_le_set_cust_transmitter_test_cmd
{
    /// Enable/Disable
    uint8_t enable;
    /// Number of packets to be transmitted
    uint16_t tx_packets;
    /// Transmit power (in dBm)
    int8_t tx_power;
};

/// HCI VS handle MIC invalid command parameters
struct hci_vs_handle_mic_invalid_cmd
{
    /// BD address to set
    struct bd_addr bd_addr;

    /// Indicates whether the ECI_MIC_Invalid VS event is generated or not
    uint8_t gen_mic_invalid_event;

    /// Indicates whether the ECI_AES_CCM_Counter VS event is generated or not
    uint8_t gen_aes_ccm_counter_event;

    /// Indicates whether the Rx AES-CCM counter is increased after reception of an invalid MIC or not
    uint8_t increase_counter;

    /// Indicates whether special handling for invalid MIC is enabled or not
    uint8_t tolerate_invalid_mic;

    /// Number of invalid MIC receptions before the encryption key is refreshed
    uint8_t n_refr;

    /// Number of invalid MIC receptions before the link is disconnected
    uint8_t n_disc;
};

/// HCI VS Set Custom Advertising command parameters
struct hci_vs_le_set_manual_adv_cmd
{
    /// Advertising handle
    uint8_t adv_hdl;

    /// Enable/Disable
    uint8_t enable;

    /// Time offset
    uint16_t time_offset;

    /// Maximum random offset
    uint16_t max_rand_offset;
};

///HCI VS Set Segmented Scan command parameters
/*@TRACE*/
struct hci_vs_le_set_seg_scan_cmd
{
    /// Enable/Disable
    uint8_t         enable;

    /// BD address of the peer to align timings to
    struct bd_addr      bd_addr;

    ///Indicates the PHY(s) on which the scans shall be segmented (@see enum le_phy_mask)
    uint8_t         scan_phys;

    ///Paramaters for PHY(s)
    struct seg_scan_phy
    {
        /// Time offset (in slots)
        uint16_t        time_offset;
        /// Periodicity (in slots)
        uint16_t        periodicity;
        /// Duration of each segment (in usecs)
        uint16_t        duration;
        /// segmnets per scan (in number of segments)
        uint16_t        segments;
        /// Microsecond offset (in usecs)
        int16_t         us_offset;
    } phy[MAX_SCAN_PHYS];
};


/******************************************************************************/
/* ---------------------    VENDOR SPECIFIC EVENTS      ----------------------*/
/******************************************************************************/

/// HCI VS get deep sleep counter command complete event
struct hci_vs_get_deep_sleep_counter_cmd_cmp_evt
{
    /// Status
    uint8_t status;
    /// Deep sleep counter
    uint32_t deep_sleep_counter;
};

/// HCI VS SCO link request event
struct hci_vs_sco_link_request_evt
{
    /// Vendor specific event subcode
    uint8_t             subcode;
    /// BD address of the peer
    struct bd_addr      bd_addr;
};

/// HCI VS clock wrap event
struct hci_vs_clock_wrap_evt
{
    /// Vendor specific event subcode
    uint8_t             subcode;
    /// BD address of the connection of reference
    struct bd_addr      bd_addr;
};

/// HCI VS sniff request event
struct hci_vs_sniff_request_evt
{
    /// Vendor specific event subcode
    uint8_t             subcode;
    /// BD address of the peer
    struct bd_addr      bd_addr;
    /// Role of the local device
    uint8_t             role;
    /// Sniff interval
    uint16_t            sniff_interval;
    /// Sniff delay
    uint16_t            sniff_delay;
    /// Sniff attempt
    uint16_t            sniff_attempt;
    /// Sniff timeout
    uint16_t            sniff_timeout;
};

/// HCI VS sniff mode change event
struct hci_vs_sniff_mode_change_evt
{
    /// Vendor specific event subcode
    uint8_t             subcode;
    /// BD address of the peer
    struct bd_addr      bd_addr;
    /// Role of the local device
    uint8_t             role;
    /// Sniff mode
    uint8_t             sniff_mode;
    /// Sniff interval
    uint16_t            sniff_interval;
    /// Sniff delay
    uint16_t            sniff_delay;
    /// Sniff attempt
    uint16_t            sniff_attempt;
    /// Sniff timeout
    uint16_t            sniff_timeout;
};

/// HCI VS custom Tx test end event
struct hci_vs_cust_tx_test_end_evt
{
    /// Vendor specific event subcode
    uint8_t             subcode;
};
#endif // EAVESDROPPING_SUPPORT

struct hci_nvds_data_update_cmd_t
{
    uint8_t length;
    uint8_t data[__ARRAY_EMPTY];
};


struct hci_wlan_coex_enable_cmd_t
{
    uint8_t enable;
    uint8_t mode;
    uint8_t reserved0;
    uint8_t reserved1;
};

struct hci_wlan_coex_set_priority_cmd_t
{
    uint8_t threshold;
    uint32_t btm0;
    uint32_t btm1;
    uint32_t btm2;
    uint32_t blem0;
    uint32_t blem1;
};

struct hci_wlan_coex_set_filter_channel_cmd_t
{
    uint8_t channel[10];
};

struct hci_dbg_set_trc_cfg_cmd_t
{
    uint32_t cfg;
};

struct hci_dbg_reserved_32_cmd_t
{
    uint8_t reserved[4];
};

struct hci_dbg_reserved_64_cmd_t
{
    uint8_t reserved[8];
};

struct hci_dbg_reserved_128_cmd_t
{
    uint8_t reserved[16];
};


#if ((BLE_EMB_PRESENT && !HOST_PRESENT) || (!EMB_PRESENT && BLE_HOST_PRESENT))
///HCI VS LE Decrypt Command parameters structure
/*@TRACE*/
struct hci_vs_le_decrypt_cmd
{
    ///Long term key structure
    struct ltk     key;
    ///Pointer to buffer with encrypt data to decrypt - 16 bytes
    uint8_t        encrypt_data[16];
};
///HCI VS LE Decrypt Command complete event structure
/*@TRACE*/
struct hci_vs_le_decrypt_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t status;
    ///Decrypted  data to return to command source.
    uint8_t plain_data[ENC_DATA_LEN];
};
#endif // ((BLE_EMB_PRESENT && !HOST_PRESENT) || (!EMB_PRESENT && BLE_HOST_PRESENT))

#if (BLE_CH_SCAN_SUPPORT)
///HCI command complete event structure for Channel Scanning End
/*@TRACE*/
struct hci_vs_le_ch_scan_end_cmd_cmp_evt
{
    /// Status of the command reception
    uint8_t             status;
};

/// HCI LE Set Channel Scanning Command
struct hci_vs_le_ch_scan_cmd
{
    /// Scan window duration in us
    uint32_t win_duration;
    /// Channel Scan event in us
    uint32_t scan_duration;
    /// Channel Scan interval in Time = N*1.25ms
    uint16_t intv;
    /// Channel selection on which channel scan is performed (see: enum ch_scan_ch_sel)
    uint8_t ch_sel;
};
#endif // (BLE_CH_SCAN_SUPPORT)

#if (RW_DEBUG)
/// HCI LE Connection event trace Command
struct hci_dbg_le_con_evt_trace_cmd
{
    /// Connection handle
    uint16_t con_hdl;
    /// Number of events to check (1..255)
    uint8_t nb_events;
};

/// HCI LE Connection event trace Complete Event
struct hci_dbg_le_con_evt_trace_cmd_cmp_evt
{
    /// DBG Subevent code
    uint8_t  subcode;
    /// Connection handle
    uint16_t con_hdl;
    /// Number of events to trace
    uint8_t nb_events;
    /// Data trace
    uint16_t data[HCI_CON_MAX_TRACE_NB_EVENT];
};


struct hci_dbg_bt_rssi_notify_evt_t
{
    uint8_t subcode;
    int8_t ave_rssi[MAX_BT_ACL_LINK];
    int8_t chan_lvl[AFH_NB_CHANNEL_MAX];
};

#endif // (RW_DEBUG)

/// @} CO_BT
#endif // CO_HCI_H_
