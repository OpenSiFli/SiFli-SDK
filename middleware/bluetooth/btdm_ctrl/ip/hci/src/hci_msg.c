/**
 ****************************************************************************************
 *
 * @file hci_msg.c
 *
 * @brief HCI module source file.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup HCI
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"       // SW configuration

#if (HCI_PRESENT)

#include <string.h>          // string manipulation
#include "co_error.h"        // error definition
#include "co_utils.h"        // common utility definition
#include "co_list.h"         // list definition
#include "co_endian.h"       // 16bits in host format
#include "hci.h"             // hci definition
#include "hci_int.h"         // hci internal definition

#include "rom_config.h"

#if (AUDIO_SYNC_SUPPORT)
    #include "audio_sync_hci.h"  // AUSY support
#endif // (AUDIO_SYNC_SUPPORT)

#if (HOST_PRESENT)
    #include "hl_hci.h"         // HCI Host handler Local identifier
    #if (!EMB_PRESENT)
        #include "gap.h"            // GAP defines
    #endif // (!EMB_PRESENT)
#endif // (HOST_PRESENT)

/*
 * DEFINES
 ****************************************************************************************
 */
/// Macro to get OCF of a known command
#define OCF(cmd)        (HCI_OP2OCF(HCI_##cmd##_CMD_OPCODE))

/// Number of supported OGF
#define HCI_MSG_NB_OGF_SUPP   (8)

/**
 * Command Info Bit decoding
 *
 * bit | 15 14 | 13..10 | 9..0 |
 * def | RFU   | DST_LL | OCF  |
 */
enum hci_cmd_info_bf
{
    /// OCF value (10 bits)
    HCI_CMD_INFO_OCF_LSB        = 0,
    HCI_CMD_INFO_OCF_MASK       = 0x03FF,
    /// Controller Command destination
    HCI_CMD_INFO_DEST_LL_LSB    = 10,
    HCI_CMD_INFO_DEST_LL_MASK   = 0x3C00,
};

/**
 * Destination field decoding
 *
 * bit | 7..2 | 1 0    |
 * def | RFU  |  HL_TL |
 */
enum hci_evt_dest
{
    /// Destination HL transport layer
    HCI_EVT_DEST_HOST_TL_LSB  = 0,
    HCI_EVT_DEST_HOST_TL_MASK = 0x03,
};

#if (HCI_TL_SUPPORT)
/// Special Pack settings for HCI events
#define PK_SPE(pk_id)           ((uint32_t) (0xFFFFFF00L | (HCI_MSG_PK_##pk_id)))
/// retrieve Specific packer identifier
#define PK_SPE_ID(format_ptr)   ((uint8_t) (0x000000FFL & ((uint32_t) format_ptr)))
/// Get if specific packer function configured
#define IS_PK_SPE(format_ptr)   (((((uint32_t) format_ptr) & 0xFFFFFF00L) == (0xFFFFFF00L)) ? true : false)

#if(HOST_PRESENT)
/// Macro for building a command descriptor in split mode (with parameters packing/unpacking)
#define CMD(opcode, dest_ll, par_size_max, par_fmt, ret_fmt) \
    {OCF(opcode) | ((dest_ll) << HCI_CMD_INFO_DEST_LL_LSB), par_size_max, (const char*)par_fmt, (const char*)ret_fmt}
/// Macro for building an event descriptor in split mode (with parameters packing/unpacking)
#define EVT(code, dest_host, par_fmt) \
    {HCI_##code##_EVT_CODE, (dest_host << HCI_EVT_DEST_HOST_TL_LSB), HL_HCI_##code##_EVT, (const char*)par_fmt}
/// Macro for building an event descriptor in split mode (with parameters packing/unpacking)
#define LE_EVT(subcode, dest_host, par_fmt) \
    {HCI_##subcode##_EVT_SUBCODE, (dest_host << HCI_EVT_DEST_HOST_TL_LSB) , HL_HCI_##subcode##_EVT, (const char*)par_fmt}
/// Macro for building an event descriptor in split mode (with parameters packing/unpacking)
#define DBG_EVT(subcode, dest_host, par_fmt) \
    {HCI_##subcode##_EVT_SUBCODE, (dest_host << HCI_EVT_DEST_HOST_TL_LSB), HL_HCI_##subcode##_EVT, (const char*)par_fmt}

#else // !(HOST_PRESENT)
/// Macro for building a command descriptor in split mode (with parameters packing/unpacking)
#define CMD(opcode, dest_ll, par_size_max, par_fmt, ret_fmt) \
    {OCF(opcode) | ((dest_ll) << HCI_CMD_INFO_DEST_LL_LSB), par_size_max, (const char*)par_fmt, (const char*)ret_fmt}
/// Macro for building an event descriptor in split mode (with parameters packing/unpacking)
#define EVT(code, dest_host, par_fmt) \
    {HCI_##code##_EVT_CODE, (const char*)par_fmt}
/// Macro for building an event descriptor in split mode (with parameters packing/unpacking)
#define LE_EVT(subcode, dest_host, par_fmt) \
    {HCI_##subcode##_EVT_SUBCODE, (const char*)par_fmt}
/// Macro for building an event descriptor in split mode (with parameters packing/unpacking)
#define DBG_EVT(subcode, dest_host, par_fmt) \
    {HCI_##subcode##_EVT_SUBCODE, (const char*)par_fmt}
#endif // (HOST_PRESENT)
#else //(HCI_TL_SUPPORT)
#if(HOST_PRESENT)
/// Macro for building a command descriptor in full mode (without parameters packing/unpacking)
#define CMD(opcode, dest_ll, par_size_max, par_fmt, ret_fmt) \
    {OCF(opcode) | ((dest_ll) << HCI_CMD_INFO_DEST_LL_LSB)}
/// Macro for building an event descriptor in full mode (without parameters packing/unpacking)
#define EVT(code, dest_host, par_fmt) \
    {HCI_##code##_EVT_CODE, (dest_host << HCI_EVT_DEST_HOST_TL_LSB), HL_HCI_##code##_EVT}
/// Macro for building an event descriptor in full mode (without parameters packing/unpacking)
#define LE_EVT(subcode, dest_host, par_fmt) \
    {HCI_##subcode##_EVT_SUBCODE, (dest_host << HCI_EVT_DEST_HOST_TL_LSB), HL_HCI_##subcode##_EVT}
/// Macro for building an event descriptor in full mode (without parameters packing/unpacking)
#define DBG_EVT(subcode, dest_host, par_fmt) \
    {HCI_##subcode##_EVT_SUBCODE, (dest_host << HCI_EVT_DEST_HOST_TL_LSB), HL_HCI_##subcode##_EVT}
#else // !(HOST_PRESENT)
#error "Controller only without HCI TL not supported"
#endif // (HOST_PRESENT)
#endif //(HCI_TL_SUPPORT)


/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

/// HCI command descriptors root table index
enum hci_ogf_idx
{
    ///HCI Link Control Commands Group OGF code index
    HCI_OGF_IDX_LK_CNTL,
#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    ///HCI Link Policy Commands Group OGF code index
    HCI_OGF_IDX_LK_POL,
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)
    ///HCI Controller and Baseband Commands Group OGF code index
    HCI_OGF_IDX_CNTLR_BB,
    ///HCI Information Parameters Commands Group OGF code index
    HCI_OGF_IDX_INFO_PAR,
    ///HCI Status Commands Group OGF code index
    HCI_OGF_IDX_STAT_PAR,
#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    ///HCI Test Commands Group OGF code index
    HCI_OGF_IDX_TEST,
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)
    ///HCI Vendor Specific Group OGF code index
    HCI_OGF_IDX_VS,
#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
    ///HCI Low Energy Commands Group OGF code index
    HCI_OGF_IDX_LE_CNTLR,
#endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)

    HCI_OGF_IDX_NB,
    HCI_OGF_IDX_INVALID = 0xFF,
};

#if (HCI_TL_SUPPORT)
/// Specific packer identifier
enum hci_msg_pk_spe
{
#if(BT_EMB_PRESENT || BT_HOST_PRESENT)
    HCI_MSG_PK_SET_EVT_FILTER,
#if RW_MWS_COEX
    HCI_MSG_PK_GET_MWS_TRANSPORT_LAYER_CONFIG,
#endif //RW_MWS_COEX
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)
#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
    HCI_MSG_PK_LE_ADV_REPORT,
    HCI_MSG_PK_LE_EXT_ADV_REPORT,
    HCI_MSG_PK_LE_TX_TEST_V4,
#endif // (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
#if (RW_DEBUG)
    HCI_MSG_PK_DBG_ASSERT,
#endif //(RW_DEBUG)

    HCI_MSG_PK_NB,
};
#endif //(HCI_TL_SUPPORT)

/*
 * STRUCTURES DEFINITIONS
 ****************************************************************************************
 */

/// HCI pack/unpack function pointer type definition
typedef uint8_t (*hci_pkupk_func_t)(uint8_t *out, uint8_t *in, uint16_t *out_len, uint16_t in_len);

/// HCI command descriptor structure
struct hci_cmd_desc
{
    /// Information bit field that contains OCF and ll/hl destination
    uint16_t    info_bf;
#if (HCI_TL_SUPPORT)
    /// Maximum size of the parameters
    uint8_t     par_size_max;
    /// Parameters format string (or special unpacker)
    const char *par_fmt;
    /// Return parameters format string (or special unpacker)
    const char *ret_par_fmt;
#endif // (HCI_TL_SUPPORT)
};

/// HCI event descriptor structure
struct hci_evt_desc
{
    /// Event opcode
    uint8_t     code;
#if (HOST_PRESENT)
    /// Destination field (used to find the internal destination task)
    uint8_t     dest_field;
    /// Local identifier used to select the hci handler
    /// (@see enum hl_hci_le_evt_handler_lid or enum hl_hci_evt_handler_lid or @see enum hl_hci_vs_evt_handler_lid)
    uint8_t     host_lid;
#endif // (HOST_PRESENT)
#if (HCI_TL_SUPPORT)
    /// Parameters format string (or special unpacker)
    const char *par_fmt;
#endif //(HCI_TL_SUPPORT)
};

/// HCI command descriptor root table structure
struct hci_cmd_desc_tab_ref
{
    /// Number of commands supported in this group
    uint16_t nb_cmds;

    /// Command descriptor table
    const hci_cmd_desc_t *cmd_desc_tab;
};

/*
 * LOCAL FUNCTIONS DECLARATIONS
 ****************************************************************************************
 */

/*
 * If Transport layer is present, some commands/events parameters with special format (variable content) needs
 * specific function for packing or unpacking. The specific function may support packing only, unpacking only, or both.
 *
 * Function types:
 *   - pkupk => functions used to pack or unpack parameters (depending on the Host<->Controller direction supported)
 *   - upk   => functions used to unpack parameters
 *   - pk    => functions used to pack parameters
 *
 * The support of packing or unpacking depends on the Host<->Controller direction supported by each commands/event:
 *  - for commands supported in LE:
 *      - Split-Host configuration          -> command parameters are packed / return parameters are unpacked
 *      - Split-Emb or full configuration   -> command parameters are unpacked / return parameters are packed
 *  - for events supported in LE:
 *      - Split-Host configuration          -> event parameters are unpacked
 *      - Split-Emb or full configuration   -> event parameters are packed
 *  - for commands supported in BT only:
 *                                          -> command parameters are unpacked / return parameters are packed
 *  - for events supported in BT only:
 *                                          -> event parameters are packed
 */

#if (HCI_TL_SUPPORT)
    #if (BT_EMB_PRESENT)
        __STATIC uint8_t hci_set_evt_filter_cmd_upk(uint8_t *out, uint8_t *in, uint16_t *out_len, uint16_t in_len);
        #if RW_MWS_COEX
            __STATIC uint8_t hci_get_mws_transport_layer_config_cmd_cmp_evt_pk(uint8_t *out, uint8_t *in, uint16_t *out_len, uint16_t in_len);
        #endif //RW_MWS_COEX
    #elif (BT_HOST_PRESENT && BT_HOST_OTHER_STACK)
        uint8_t bts2_hci_set_evt_filter_cmd_upk(uint8_t *out, uint8_t *in, uint16_t *out_len, uint16_t in_len);
    #endif //(BT_EMB_PRESENT)
    #if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
        __STATIC uint8_t hci_le_adv_report_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t *out_len, uint16_t in_len);
        __STATIC uint8_t hci_le_ext_adv_report_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t *out_len, uint16_t in_len);
        __STATIC uint8_t hci_le_tx_test_v4_cmd_upk(uint8_t *out, uint8_t *in, uint16_t *out_len, uint16_t in_len);
    #endif // (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
    #if (RW_DEBUG)
        __STATIC uint8_t hci_dbg_assert_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t *out_len, uint16_t in_len);
    #endif //(RW_DEBUG)
#endif //(HCI_TL_SUPPORT)


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */


/// HCI command descriptors (OGF link control)
const hci_cmd_desc_t hci_cmd_desc_tab_lk_ctrl[] =
{
    // Note: all messages must be sorted in opcode ascending order
#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
#if (!BT_HOST_OTHER_STACK)
#ifdef CFG_INQ
    CMD(INQ,                         BT_MNG,          5,  "5B",       NULL),
    CMD(INQ_CANCEL,                  BT_MNG,          0,  "",         "B"),
    CMD(PER_INQ_MODE,                BT_MNG,          9,  "2H5B",     "B"),
    CMD(EXIT_PER_INQ_MODE,           BT_MNG,          0,  "",         "B"),
#endif //CFG_INQ
    CMD(CREATE_CON,                  BT_MNG,          13, "6BH2BHB",  NULL),
#else // BT_HOST_OTHER_STACK
#ifdef CFG_INQ
    CMD(INQ,                         BT_MNG,          5,  "D2B",       NULL),
    CMD(INQ_CANCEL,                  BT_MNG,          0,  "",         "B"),
    CMD(PER_INQ_MODE,                BT_MNG,          9,  "2HD2B",     "B"),
    CMD(EXIT_PER_INQ_MODE,           BT_MNG,          0,  "",         "B"),
#endif //CFG_INQ
    CMD(CREATE_CON,                  BT_MNG,          13, "TH2BHB",  NULL),
#endif // !BT_HOST_OTHER_STACK
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

#if (!BT_HOST_OTHER_STACK)
    CMD(DISCONNECT,                  CTRL,            3,  "HB",       NULL),
#else // BT_HOST_OTHER_STACK
    CMD(DISCONNECT,                  CTRL,            3,  "HB",       NULL),
#endif // !BT_HOST_OTHER_STACK

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
#if (!BT_HOST_OTHER_STACK)
    CMD(CREATE_CON_CANCEL,           BT_MNG,          6,  "6B",       "7B"),
    CMD(ACCEPT_CON_REQ,              BT_CTRL_BD_ADDR, 7,  "7B",       NULL),
    CMD(REJECT_CON_REQ,              BT_CTRL_BD_ADDR, 7,  "7B",       NULL),
#else
    CMD(CREATE_CON_CANCEL,           BT_MNG,          6,  "T",       "tT"),
    CMD(ACCEPT_CON_REQ,              BT_CTRL_BD_ADDR, 7,  "TB",       NULL),
    CMD(REJECT_CON_REQ,              BT_CTRL_BD_ADDR, 7,  "TB",       NULL),
#endif
#if (!BT_HOST_OTHER_STACK)
    CMD(LK_REQ_REPLY,                BT_CTRL_BD_ADDR, 22, "22B",      "7B"),
    CMD(LK_REQ_NEG_REPLY,            BT_CTRL_BD_ADDR, 6,  "6B",       "7B"),
    CMD(PIN_CODE_REQ_REPLY,          BT_CTRL_BD_ADDR, 23, "23B",      "7B"),
    CMD(PIN_CODE_REQ_NEG_REPLY,      BT_CTRL_BD_ADDR, 6,  "6B",       "7B"),
#else
    CMD(LK_REQ_REPLY,                BT_CTRL_BD_ADDR, 22, "T16B",      "tT"),
    CMD(LK_REQ_NEG_REPLY,            BT_CTRL_BD_ADDR, 6,  "T",       "tT"),
    CMD(PIN_CODE_REQ_REPLY,          BT_CTRL_BD_ADDR, 23, "T17B",      "tT"),
    CMD(PIN_CODE_REQ_NEG_REPLY,      BT_CTRL_BD_ADDR, 6,  "T",       "tT"),
#endif

    CMD(CHG_CON_PKT_TYPE,            BT_CTRL_CONHDL,  4,  "2H",       NULL),
    CMD(AUTH_REQ,                    BT_CTRL_CONHDL,  2,  "H",        NULL),
    CMD(SET_CON_ENC,                 BT_CTRL_CONHDL,  3,  "HB",       NULL),
#if (!BT_HOST_OTHER_STACK)
    CMD(CHG_CON_LK,                  BT_CTRL_CONHDL,  2,  "H",        NULL),
#else
    CMD(CHG_CON_LK,                  BT_CTRL_CONHDL,  2,  "Hz",        NULL),
#endif
#if (BCAST_ENC_SUPPORT)
    CMD(MASTER_LK,                   BT_BCST,         1,  "B",        NULL),
#endif // (BCAST_ENC_SUPPORT)
#if (!BT_HOST_OTHER_STACK)
    CMD(REM_NAME_REQ,                BT_MNG,          10, "8BH",      NULL),
    CMD(REM_NAME_REQ_CANCEL,         BT_MNG,          6,  "6B",       "7B"),
    CMD(RD_REM_SUPP_FEATS,           BT_CTRL_CONHDL,  2,  "H",        NULL),
    CMD(RD_REM_EXT_FEATS,            BT_CTRL_CONHDL,  3,  "HB",       NULL),
#else
    CMD(REM_NAME_REQ,                BT_MNG,          10, "T2BH",      NULL),
    CMD(REM_NAME_REQ_CANCEL,         BT_MNG,          6,  "T",       "tT"),
    CMD(RD_REM_SUPP_FEATS,           BT_CTRL_CONHDL,  2,  "Hz",        NULL),
    CMD(RD_REM_EXT_FEATS,            BT_CTRL_CONHDL,  3,  "HzB",       NULL),
#endif
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

#if (!BT_HOST_OTHER_STACK)
    CMD(RD_REM_VER_INFO,             CTRL,            2,  "H",        NULL),
#else // BT_HOST_OTHER_STACK
    CMD(RD_REM_VER_INFO,             CTRL,            2,  "Hz",        NULL),
#endif // !BT_HOST_OTHER_STACK

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
#if (!BT_HOST_OTHER_STACK)
    CMD(RD_CLK_OFF,                  BT_CTRL_CONHDL,  2,  "H",        NULL),
    CMD(RD_LMP_HDL,                  BT_CTRL_CONHDL,  2,  "H",        "BHBL"),
#else
    CMD(RD_CLK_OFF,                  BT_CTRL_CONHDL,  2,  "Hz",        NULL),
    CMD(RD_LMP_HDL,                  BT_CTRL_CONHDL,  2,  "H",        "tHBL"),
#endif
#if (!BT_HOST_OTHER_STACK)
#if (MAX_NB_SYNC > 0)
    CMD(SETUP_SYNC_CON,              BT_CTRL_CONHDL,  17, "H2L2HBH",  NULL),
    CMD(ACCEPT_SYNC_CON_REQ,         BT_CTRL_BD_ADDR, 21, "6B2L2HBH", NULL),
    CMD(REJECT_SYNC_CON_REQ,         BT_CTRL_BD_ADDR, 7,  "7B",       NULL),
#endif // CFG_SCO
    CMD(IO_CAP_REQ_REPLY,            BT_CTRL_BD_ADDR, 9,  "9B",       "7B"),
    CMD(USER_CFM_REQ_REPLY,          BT_CTRL_BD_ADDR, 6,  "6B",       "7B"),
    CMD(USER_CFM_REQ_NEG_REPLY,      BT_CTRL_BD_ADDR, 6,  "6B",       "7B"),
    CMD(USER_PASSKEY_REQ_REPLY,      BT_CTRL_BD_ADDR, 10, "6BL",      "7B"),
    CMD(USER_PASSKEY_REQ_NEG_REPLY,  BT_CTRL_BD_ADDR, 6,  "6B",       "7B"),
    CMD(REM_OOB_DATA_REQ_REPLY,      BT_CTRL_BD_ADDR, 38, "38B",      "7B"),
    CMD(REM_OOB_DATA_REQ_NEG_REPLY,  BT_CTRL_BD_ADDR, 6,  "6B",       "7B"),
    CMD(IO_CAP_REQ_NEG_REPLY,        BT_CTRL_BD_ADDR, 7,  "7B",       "7B"),
#if (MAX_NB_SYNC > 0)
    CMD(ENH_SETUP_SYNC_CON,          BT_CTRL_CONHDL,  59, "H2L10B2H2L10B2H8B2HB",  NULL),
    CMD(ENH_ACCEPT_SYNC_CON,         BT_CTRL_BD_ADDR, 63, "6B2L10B2H2L10B2H8B2HB", NULL),
#endif // CFG_SCO
#else
#if (MAX_NB_SYNC > 0)
    CMD(SETUP_SYNC_CON,              BT_CTRL_CONHDL,  17, "Hz2L2HBH",  NULL),
    CMD(ACCEPT_SYNC_CON_REQ,         BT_CTRL_BD_ADDR, 21, "T2L2HBH", NULL),
    CMD(REJECT_SYNC_CON_REQ,         BT_CTRL_BD_ADDR, 7,  "TB",       NULL),
#endif // CFG_SCO
    CMD(IO_CAP_REQ_REPLY,            BT_CTRL_BD_ADDR, 9,  "T3B",       "tT"),
    CMD(USER_CFM_REQ_REPLY,          BT_CTRL_BD_ADDR, 6,  "T",       "tT"),
    CMD(USER_CFM_REQ_NEG_REPLY,      BT_CTRL_BD_ADDR, 6,  "T",       "tT"),
    CMD(USER_PASSKEY_REQ_REPLY,      BT_CTRL_BD_ADDR, 10, "TL",      "tT"),
    CMD(USER_PASSKEY_REQ_NEG_REPLY,  BT_CTRL_BD_ADDR, 6,  "T",       "tT"),
    CMD(REM_OOB_DATA_REQ_REPLY,      BT_CTRL_BD_ADDR, 38, "T32B",      "tT"),
    CMD(REM_OOB_DATA_REQ_NEG_REPLY,  BT_CTRL_BD_ADDR, 6,  "T",       "tT"),
    CMD(IO_CAP_REQ_NEG_REPLY,        BT_CTRL_BD_ADDR, 7,  "TB",       "tT"),
#if (MAX_NB_SYNC > 0)
    CMD(ENH_SETUP_SYNC_CON,          BT_CTRL_CONHDL,  59, "Hz2L10B2H2L10B2H8B2HB",  NULL),
    CMD(ENH_ACCEPT_SYNC_CON,         BT_CTRL_BD_ADDR, 63, "T2L10B2H2L10B2H8B2HB", NULL),
#endif // CFG_SCO
#endif

#if (CSB_SUPPORT)
    CMD(TRUNC_PAGE,                  BT_MNG,          9,  "7BH",            NULL),
    CMD(TRUNC_PAGE_CAN,              BT_MNG,          6,  "6B",             "B6B"),
    CMD(SET_CON_SLV_BCST,            BT_BCST,         11, "3B4H",           "BBH"),
    CMD(SET_CON_SLV_BCST_REC,        BT_BCST,         34, "8BH2LH2BH10B",   "B6BB"),
    CMD(START_SYNC_TRAIN,            BT_BCST,         0,  "",               NULL),
    CMD(REC_SYNC_TRAIN,              BT_BCST,         12, "6B3H",           NULL),
#endif // (CSB_SUPPORT)

#if (!BT_HOST_OTHER_STACK)
    CMD(REM_OOB_EXT_DATA_REQ_REPLY,  BT_CTRL_BD_ADDR, 70, "70B", "7B"),
#else
    CMD(REM_OOB_EXT_DATA_REQ_REPLY,  BT_CTRL_BD_ADDR, 70, "70B", "tT"),
#endif
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)
};

///// HCI command descriptors (OGF link policy)
#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
const hci_cmd_desc_t hci_cmd_desc_tab_lk_pol[] =
{
    // Note: all messages must be sorted in opcode ascending order
#if (!BT_HOST_OTHER_STACK)
    CMD(SNIFF_MODE,                  BT_CTRL_CONHDL,  10, "5H",       NULL),
    CMD(EXIT_SNIFF_MODE,             BT_CTRL_CONHDL,  2,  "H",        NULL),
    CMD(QOS_SETUP,                   BT_CTRL_CONHDL,  20, "H2B4L",    NULL),
    CMD(ROLE_DISCOVERY,              BT_CTRL_CONHDL,  2,  "H",        "BHB"),
    CMD(SWITCH_ROLE,                 BT_CTRL_BD_ADDR, 7,  "7B",       NULL),
    CMD(RD_LINK_POL_STG,             BT_CTRL_CONHDL,  2,  "H",        "B2H"),
    CMD(WR_LINK_POL_STG,             BT_CTRL_CONHDL,  4,  "2H",       "BH"),
    CMD(RD_DFT_LINK_POL_STG,         BT_MNG,          0,  "",         "BH"),
    CMD(WR_DFT_LINK_POL_STG,         BT_MNG,          2,  "H",        "B"),
    CMD(FLOW_SPEC,                   BT_CTRL_CONHDL,  21, "H3B4L",    NULL),
    CMD(SNIFF_SUB,                   BT_CTRL_CONHDL,  8,  "4H",       "BH"),
#else // (!BT_HOST_OTHER_STACK)
    CMD(SNIFF_MODE,                  BT_CTRL_CONHDL,  10, "Hz4H",       NULL),
    CMD(EXIT_SNIFF_MODE,             BT_CTRL_CONHDL,  2,  "Hz",        NULL),
    CMD(QOS_SETUP,                   BT_CTRL_CONHDL,  20, "Hz2B4L",    NULL),
    CMD(ROLE_DISCOVERY,              BT_CTRL_CONHDL,  2,  "Hz",        "tHB"),
    CMD(SWITCH_ROLE,                 BT_CTRL_BD_ADDR, 7,  "TB",       NULL),
    CMD(RD_LINK_POL_STG,             BT_CTRL_CONHDL,  2,  "Hz",        "t2H"),
    CMD(WR_LINK_POL_STG,             BT_CTRL_CONHDL,  4,  "HzH",       "tH"),
    CMD(RD_DFT_LINK_POL_STG,         BT_MNG,          0,  "",         "tH"),
    CMD(WR_DFT_LINK_POL_STG,         BT_MNG,          2,  "H",        "t"),
    CMD(FLOW_SPEC,                   BT_CTRL_CONHDL,  21, "Hz3B4L",    NULL),
    CMD(SNIFF_SUB,                   BT_CTRL_CONHDL,  8,  "Hz3H",       "tH"),
#endif //(!BT_HOST_OTHER_STACK)
};
#endif// (BT_EMB_PRESENT || BT_HOST_PRESENT)

/// HCI command descriptors (OGF controller and baseband)
const hci_cmd_desc_t hci_cmd_desc_tab_ctrl_bb[] =
{
    // Note: all messages must be sorted in opcode ascending order
#if (!BT_HOST_OTHER_STACK) // for all desc
    CMD(SET_EVT_MASK,                  MNG,             8,   "8B",    "B"),
    CMD(RESET,                         MNG,             0,   "",      "B"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    CMD(SET_EVT_FILTER,                BT_MNG,          9,   PK_SPE(SET_EVT_FILTER), "B"),
    CMD(FLUSH,                         BT_CTRL_CONHDL,  2,   "H",                    "BH"),
    CMD(RD_PIN_TYPE,                   BT_MNG,          0,   "",                     "2B"),
    CMD(WR_PIN_TYPE,                   BT_MNG,          1,   "B",                    "B"),
    CMD(RD_STORED_LK,                  BT_MNG,          7,   "7B",                   "B2H"),
    CMD(WR_STORED_LK,                  BT_MNG,          243, "S122B",                "2B"),
    CMD(DEL_STORED_LK,                 BT_MNG,          7,   "7B",                   "BH"),
    CMD(WR_LOCAL_NAME,                 BT_MNG,          248, "248B",                 "B"),
    CMD(RD_LOCAL_NAME,                 BT_MNG,          0,   "",                     "249B"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    CMD(RD_CON_ACCEPT_TO,              MNG,             0,   "",                     "BH"),
    CMD(WR_CON_ACCEPT_TO,              MNG,             2,   "H",                    "B"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    CMD(RD_PAGE_TO,                    BT_MNG,          0,   "",                     "BH"),
    CMD(WR_PAGE_TO,                    BT_MNG,          2,   "H",                    "B"),
    CMD(RD_SCAN_EN,                    BT_MNG,          0,   "",                     "2B"),
    CMD(WR_SCAN_EN,                    BT_MNG,          1,   "B",                    "B"),
    CMD(RD_PAGE_SCAN_ACT,              BT_MNG,          0,   "",                     "B2H"),
    CMD(WR_PAGE_SCAN_ACT,              BT_MNG,          4,   "2H",                   "B"),
    CMD(RD_INQ_SCAN_ACT,               BT_MNG,          0,   "",                     "B2H"),
    CMD(WR_INQ_SCAN_ACT,               BT_MNG,          4,   "2H",                   "B"),
    CMD(RD_AUTH_EN,                    BT_MNG,          0,   "",                     "2B"),
    CMD(WR_AUTH_EN,                    BT_MNG,          1,   "B",                    "B"),
    CMD(RD_CLASS_OF_DEV,               BT_MNG,          0,   "",                     "4B"),
    CMD(WR_CLASS_OF_DEV,               BT_MNG,          3,   "3B",                   "B"),
#if (MAX_NB_SYNC > 0)
    CMD(RD_VOICE_STG,                  BT_MNG,          0,   "",      "BH"),
    CMD(WR_VOICE_STG,                  BT_MNG,          2,   "H",     "B"),
#endif // (MAX_NB_SYNC > 0)

    CMD(RD_AUTO_FLUSH_TO,              BT_CTRL_CONHDL,  2,   "H",     "B2H"),
    CMD(WR_AUTO_FLUSH_TO,              BT_CTRL_CONHDL,  4,   "2H",    "BH"),
    CMD(RD_NB_BDCST_RETX,              BT_BCST,         0,   "",      "2B"),
    CMD(WR_NB_BDCST_RETX,              BT_BCST,         1,   "B",     "B"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    CMD(RD_TX_PWR_LVL,                 CTRL,            3,   "HB",    "BHB"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    CMD(RD_SYNC_FLOW_CTRL_EN,          BT_MNG,          0,   "",      "2B"),
    CMD(WR_SYNC_FLOW_CTRL_EN,          BT_MNG,          1,   "B",     "B"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    CMD(SET_CTRL_TO_HOST_FLOW_CTRL,    MNG,             1,   "B",     "B"),
    CMD(HOST_BUF_SIZE,                 MNG,             7,   "HB2H",  "B"),
    CMD(HOST_NB_CMP_PKTS,              MNG,             30,  "S22H",  "B"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    CMD(RD_LINK_SUPV_TO,               BT_CTRL_CONHDL,  2,   "H",     "B2H"),
    CMD(WR_LINK_SUPV_TO,               BT_CTRL_CONHDL,  4,   "2H",    "BH"),
    CMD(RD_NB_SUPP_IAC,                BT_MNG,          0,   "",      "2B"),
    CMD(RD_CURR_IAC_LAP,               BT_MNG,          0,   "",      "5B"),
    CMD(WR_CURR_IAC_LAP,               BT_MNG,          253, "S13B",  "B"),
    CMD(SET_AFH_HOST_CH_CLASS,         BT_MNG,          10,  "10B",   "B"),
    CMD(RD_INQ_SCAN_TYPE,              BT_MNG,          0,   "",      "2B"),
    CMD(WR_INQ_SCAN_TYPE,              BT_MNG,          1,   "B",     "B"),
    CMD(RD_INQ_MODE,                   BT_MNG,          0,   "",      "2B"),
    CMD(WR_INQ_MODE,                   BT_MNG,          1,   "B",     "B"),
    CMD(RD_PAGE_SCAN_TYPE,             BT_MNG,          0,   "",      "2B"),
    CMD(WR_PAGE_SCAN_TYPE,             BT_MNG,          1,   "B",     "B"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)
    CMD(RD_AFH_CH_ASSESS_MODE,         MNG,             0,   "",      "2B"),
    CMD(WR_AFH_CH_ASSESS_MODE,         MNG,             1,   "B",     "B"),
#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    CMD(RD_EXT_INQ_RSP,                BT_MNG,          0,   "",      "242B"),
    CMD(WR_EXT_INQ_RSP,                BT_MNG,          241, "241B",  "B"),
    CMD(REFRESH_ENC_KEY,               BT_CTRL_CONHDL,  2,   "H",     NULL),
    CMD(RD_SP_MODE,                    BT_MNG,          0,   "",      "2B"),
    CMD(WR_SP_MODE,                    BT_MNG,          1,   "B",     "B"),
    CMD(RD_LOC_OOB_DATA,               BT_MNG,          0,   "",      "33B"),
    CMD(RD_INQ_RSP_TX_PWR_LVL,         BT_MNG,          0,   "",      "2B"),
    CMD(WR_INQ_TX_PWR_LVL,             BT_MNG,          1,   "B",     "B"),
    CMD(RD_DFT_ERR_DATA_REP,           BT_MNG,          0,   "",      "2B"),
    CMD(WR_DFT_ERR_DATA_REP,           BT_MNG,          1,   "B",     "B"),
    CMD(ENH_FLUSH,                     BT_CTRL_CONHDL,  3,   "HB",    NULL),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

#else // !BT_HOST_OTHER_STACK
    CMD(SET_EVT_MASK,                  MNG,             8,   "8B",    "t"),
    CMD(RESET,                         MNG,             0,   "",      "t"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    CMD(SET_EVT_FILTER,                BT_MNG,          9,   PK_SPE(SET_EVT_FILTER), "t"),
    CMD(FLUSH,                         BT_CTRL_CONHDL,  2,   "Hz",                    "tH"),
#ifndef CFG_HCI_SLIM
    CMD(RD_PIN_TYPE,                   BT_MNG,          0,   "",                     "tB"),
    CMD(WR_PIN_TYPE,                   BT_MNG,          1,   "B",                    "t"),
    CMD(RD_STORED_LK,                  BT_MNG,          7,   "TB",                   "t2H"),
    CMD(WR_STORED_LK,                  BT_MNG,          243, "S1T16B",                "tB"),
    CMD(DEL_STORED_LK,                 BT_MNG,          7,   "TB",                   "tH"),
#endif //CFG_HCI_SLIM
    CMD(WR_LOCAL_NAME,                 BT_MNG,          248, "248B",                 "t"),
    CMD(RD_LOCAL_NAME,                 BT_MNG,          0,   "",                     "t248B"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    CMD(RD_CON_ACCEPT_TO,              MNG,             0,   "",                     "tH"),
    CMD(WR_CON_ACCEPT_TO,              MNG,             2,   "H",                    "t"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    CMD(RD_PAGE_TO,                    BT_MNG,          0,   "",                     "tH"),
    CMD(WR_PAGE_TO,                    BT_MNG,          2,   "H",                    "t"),
    CMD(RD_SCAN_EN,                    BT_MNG,          0,   "",                     "tB"),
    CMD(WR_SCAN_EN,                    BT_MNG,          1,   "B",                    "t"),
    CMD(RD_PAGE_SCAN_ACT,              BT_MNG,          0,   "",                     "t2H"),
    CMD(WR_PAGE_SCAN_ACT,              BT_MNG,          4,   "2H",                   "t"),
    CMD(RD_INQ_SCAN_ACT,               BT_MNG,          0,   "",                     "t2H"),
    CMD(WR_INQ_SCAN_ACT,               BT_MNG,          4,   "2H",                   "t"),
    CMD(RD_AUTH_EN,                    BT_MNG,          0,   "",                     "tB"),
    CMD(WR_AUTH_EN,                    BT_MNG,          1,   "B",                    "t"),
    CMD(RD_CLASS_OF_DEV,               BT_MNG,          0,   "",                     "t3B"),
    CMD(WR_CLASS_OF_DEV,               BT_MNG,          3,   "D",                   "t"),

#if (MAX_NB_SYNC > 0)
    CMD(RD_VOICE_STG,                  BT_MNG,          0,   "",      "tH"),
    CMD(WR_VOICE_STG,                  BT_MNG,          2,   "H",     "t"),
#endif // (MAX_NB_SYNC > 0)

    CMD(RD_AUTO_FLUSH_TO,              BT_CTRL_CONHDL,  2,   "Hz",     "t2H"),
    CMD(WR_AUTO_FLUSH_TO,              BT_CTRL_CONHDL,  4,   "HzH",    "tH"),
#ifndef CFG_HCI_SLIM
    CMD(RD_NB_BDCST_RETX,              BT_BCST,         0,   "",      "tB"),
    CMD(WR_NB_BDCST_RETX,              BT_BCST,         1,   "B",     "t"),
#endif //CFG_HCI_SLIM
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    CMD(RD_TX_PWR_LVL,                 CTRL,            3,   "HzB",    "tHB"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
#if (MAX_NB_SYNC > 0)
    CMD(RD_SYNC_FLOW_CTRL_EN,          BT_MNG,          0,   "",      "tB"),
    CMD(WR_SYNC_FLOW_CTRL_EN,          BT_MNG,          1,   "B",     "t"),
#endif // (MAX_NB_SYNC > 0)
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

#ifndef CFG_HCI_SLIM
    CMD(SET_CTRL_TO_HOST_FLOW_CTRL,    MNG,             1,   "B",     "t"),
    CMD(HOST_BUF_SIZE,                 MNG,             7,   "HB2H",  "t"),
    CMD(HOST_NB_CMP_PKTS,              MNG,             30,  "S22H",  "t"),
#endif //!CFG_HCI_SLIM

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    CMD(RD_LINK_SUPV_TO,               BT_CTRL_CONHDL,  2,   "Hz",     "t2H"),
    CMD(WR_LINK_SUPV_TO,               BT_CTRL_CONHDL,  4,   "HzH",    "tH"),
#if defined(CFG_INQ) && !defined(CFG_HCI_SLIM)
    CMD(RD_NB_SUPP_IAC,                BT_MNG,          0,   "",      "tB"),
    CMD(RD_CURR_IAC_LAP,               BT_MNG,          0,   "",      "t4B"),
    CMD(WR_CURR_IAC_LAP,               BT_MNG,          253, "S13B",  "t"),
#endif //CFG_INQ && !CFG_HCI_SLIM
    CMD(SET_AFH_HOST_CH_CLASS,         BT_MNG,          10,  "10B",   "t"),
    CMD(RD_INQ_SCAN_TYPE,              BT_MNG,          0,   "",      "tB"),
    CMD(WR_INQ_SCAN_TYPE,              BT_MNG,          1,   "B",     "t"),
#ifdef CFG_INQ
    CMD(RD_INQ_MODE,                   BT_MNG,          0,   "",      "tB"),
    CMD(WR_INQ_MODE,                   BT_MNG,          1,   "B",     "t"),
#endif // CFG_INQ
    CMD(RD_PAGE_SCAN_TYPE,             BT_MNG,          0,   "",      "tB"),
    CMD(WR_PAGE_SCAN_TYPE,             BT_MNG,          1,   "B",     "t"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)
#ifndef CFG_HCI_SLIM
    CMD(RD_AFH_CH_ASSESS_MODE,         MNG,             0,   "",      "tB"),
    CMD(WR_AFH_CH_ASSESS_MODE,         MNG,             1,   "B",     "t"),
#endif //CFG_HCI_SLIM
#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    CMD(RD_EXT_INQ_RSP,                BT_MNG,          0,   "",      "t241B"),
    CMD(WR_EXT_INQ_RSP,                BT_MNG,          241, "241B",  "t"),
    CMD(REFRESH_ENC_KEY,               BT_CTRL_CONHDL,  2,   "H",     NULL),
    CMD(RD_SP_MODE,                    BT_MNG,          0,   "",      "tB"),
    CMD(WR_SP_MODE,                    BT_MNG,          1,   "B",     "t"),
    CMD(RD_LOC_OOB_DATA,               BT_MNG,          0,   "",      "t32B"),
    CMD(RD_INQ_RSP_TX_PWR_LVL,         BT_MNG,          0,   "",      "tB"),
    CMD(WR_INQ_TX_PWR_LVL,             BT_MNG,          1,   "B",     "t"),
    CMD(RD_DFT_ERR_DATA_REP,           BT_MNG,          0,   "",      "tB"),
    CMD(WR_DFT_ERR_DATA_REP,           BT_MNG,          1,   "B",     "t"),
    CMD(ENH_FLUSH,                     BT_CTRL_CONHDL,  3,   "HB",    NULL),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

#endif // !BT_HOST_OTHER_STACK

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
#if (!BT_HOST_OTHER_STACK)
    CMD(SEND_KEYPRESS_NOTIF,           BT_CTRL_BD_ADDR, 7,   "7B",    "7B"),
#else
    CMD(SEND_KEYPRESS_NOTIF,           BT_CTRL_BD_ADDR, 7,   "TB",    "tT"),
#endif
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

#if (!BT_HOST_OTHER_STACK)

    CMD(SET_EVT_MASK_PAGE_2,           MNG,             8,   "8B",    "B"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
#if (BT_PWR_CTRL)
    CMD(RD_ENH_TX_PWR_LVL,             BT_CTRL_CONHDL,  3,   "HB",    "BH3B"),
#endif // (BT_PWR_CTRL)
    CMD(RD_LE_HOST_SUPP,               BT_MNG,          0,   "",      "3B"),
    CMD(WR_LE_HOST_SUPP,               BT_MNG,          2,   "2B",    "B"),

#if (RW_MWS_COEX)
    CMD(SET_MWS_CHANNEL_PARAMS,        BT_MNG,          10,  "B4HB",   "B"),
    CMD(SET_EXTERNAL_FRAME_CONFIG,     BT_MNG,          255, "3HS2HB", "B"),
    CMD(SET_MWS_SIGNALING,             BT_MNG,          30,  "15H",    "B16H"),
    CMD(SET_MWS_TRANSPORT_LAYER,       BT_MNG,          9,   "B2L",    "B"),
    CMD(SET_MWS_SCAN_FREQ_TABLE,       BT_MNG,          255, "S22H",   "B"),
    CMD(SET_MWS_PATTERN_CONFIG,        BT_MNG,          255, "BS2HB",  "B"),
#endif // (RW_MWS_COEX)

#if (CSB_SUPPORT)
    CMD(SET_RES_LT_ADDR,               BT_BCST,         1,   "B",     "2B"),
    CMD(DEL_RES_LT_ADDR,               BT_BCST,         1,   "B",     "2B"),
    CMD(SET_CON_SLV_BCST_DATA,         BT_BCST,         255, "2BnB",  "2B"),
    CMD(RD_SYNC_TRAIN_PARAM,           BT_BCST,         0,   "",      "BHLB"),
    CMD(WR_SYNC_TRAIN_PARAM,           BT_BCST,         9,   "2HLB",  "BH"),
#endif // (CSB_SUPPORT)

    CMD(RD_SEC_CON_HOST_SUPP,          BT_MNG,          0,   "",      "2B"),
    CMD(WR_SEC_CON_HOST_SUPP,          BT_MNG,          1,   "B",     "B"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    CMD(RD_AUTH_PAYL_TO,               CTRL,            2,   "H",     "B2H"),
    CMD(WR_AUTH_PAYL_TO,               CTRL,            4,   "2H",    "BH"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    CMD(RD_LOC_OOB_EXT_DATA,           BT_MNG,          0,   "",      "65B"),
    CMD(RD_EXT_PAGE_TO,                BT_MNG,          0,   "",      "BH"),
    CMD(WR_EXT_PAGE_TO,                BT_MNG,          2,   "H",     "B"),
    CMD(RD_EXT_INQ_LEN,                BT_MNG,          0,   "",      "BH"),
    CMD(WR_EXT_INQ_LEN,                BT_MNG,          2,   "H",     "B"),
#endif// (BT_EMB_PRESENT || BT_HOST_PRESENT)

    CMD(SET_ECO_BASE_INTV,             MNG,             2,   "H",     "B"),
    CMD(CONFIG_DATA_PATH,              MNG,             255, "2BnB",  "B"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT) && (BT_53)
    CMD(SET_MIN_ENC_KEY_SIZE,          BT_MNG,          1,   "B",     "B"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT) && (BT_53)
#else // (!BT_HOST_OTHER_STACK)

    CMD(SET_EVT_MASK_PAGE_2,           MNG,             8,   "8B",    "t"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
#ifndef CFG_HCI_SLIM
#if (BT_PWR_CTRL)
    CMD(RD_ENH_TX_PWR_LVL,             BT_CTRL_CONHDL,  3,   "HB",    "tH3B"),
#endif // (BT_PWR_CTRL)
#endif // !CFG_HCI_SLIM
    CMD(RD_LE_HOST_SUPP,               BT_MNG,          0,   "",      "t2B"),
    CMD(WR_LE_HOST_SUPP,               BT_MNG,          2,   "2B",    "t"),

#ifndef CFG_HCI_SLIM
#if (RW_MWS_COEX)
    CMD(SET_MWS_CHANNEL_PARAMS,        BT_MNG,          10,  "B4HB",   "t"),
    CMD(SET_EXTERNAL_FRAME_CONFIG,     BT_MNG,          255, "3HS2HB", "t"),
    CMD(SET_MWS_SIGNALING,             BT_MNG,          30,  "15H",    "t16H"),
    CMD(SET_MWS_TRANSPORT_LAYER,       BT_MNG,          9,   "B2L",    "t"),
    CMD(SET_MWS_SCAN_FREQ_TABLE,       BT_MNG,          255, "S22H",   "t"),
    CMD(SET_MWS_PATTERN_CONFIG,        BT_MNG,          255, "BS2HB",  "t"),
#endif // (RW_MWS_COEX)

#if (CSB_SUPPORT)
    CMD(SET_RES_LT_ADDR,               BT_BCST,         1,   "B",     "tB"),
    CMD(DEL_RES_LT_ADDR,               BT_BCST,         1,   "B",     "tB"),
    CMD(SET_CON_SLV_BCST_DATA,         BT_BCST,         255, "2BnB",  "tB"),
    CMD(RD_SYNC_TRAIN_PARAM,           BT_BCST,         0,   "",      "tHLB"),
    CMD(WR_SYNC_TRAIN_PARAM,           BT_BCST,         9,   "2HLB",  "tH"),
#endif // (CSB_SUPPORT)

    CMD(RD_SEC_CON_HOST_SUPP,          BT_MNG,          0,   "",      "tB"),
    CMD(WR_SEC_CON_HOST_SUPP,          BT_MNG,          1,   "B",     "t"),
#endif //!CFG_HCI_SLIM
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

#ifndef CFG_HCI_SLIM
    CMD(RD_AUTH_PAYL_TO,               CTRL,            2,   "H",     "t2H"),
    CMD(WR_AUTH_PAYL_TO,               CTRL,            4,   "2H",    "tH"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    CMD(RD_LOC_OOB_EXT_DATA,           BT_MNG,          0,   "",      "t64B"),
    CMD(RD_EXT_PAGE_TO,                BT_MNG,          0,   "",      "tH"),
    CMD(WR_EXT_PAGE_TO,                BT_MNG,          2,   "H",     "t"),
    CMD(RD_EXT_INQ_LEN,                BT_MNG,          0,   "",      "tH"),
    CMD(WR_EXT_INQ_LEN,                BT_MNG,          2,   "H",     "t"),
#endif// (BT_EMB_PRESENT || BT_HOST_PRESENT)

    CMD(SET_ECO_BASE_INTV,             MNG,             2,   "H",     "t"),
    CMD(CONFIG_DATA_PATH,              MNG,             255, "2BnB",  "t"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT) && (BT_53)
    CMD(SET_MIN_ENC_KEY_SIZE,          BT_MNG,          1,   "B",     "t"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT) && (BT_53)
#endif // !CFG_HCI_SLIM

#endif // (!BT_HOST_OTHER_STACK)
};

/// HCI command descriptors (OGF informational parameters)
const hci_cmd_desc_t hci_cmd_desc_tab_info_par[] =
{
#if (!BT_HOST_OTHER_STACK)
    // Note: all messages must be sorted in opcode ascending order
    CMD(RD_LOCAL_VER_INFO,        MNG,    0, "",   "2BHB2H"),
    CMD(RD_LOCAL_SUPP_CMDS,       MNG,    0, "",   "65B"),
    CMD(RD_LOCAL_SUPP_FEATS,      MNG,    0, "",   "9B"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    CMD(RD_LOCAL_EXT_FEATS,       BT_MNG, 1, "B",  "11B"),
    CMD(RD_BUF_SIZE,              BT_MNG, 0, "",   "BHB2H"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)
#else // (!BT_HOST_OTHER_STACK)
    // Note: all messages must be sorted in opcode ascending order
    CMD(RD_LOCAL_VER_INFO,        MNG,    0, "",   "tBHB2H"),
    CMD(RD_LOCAL_SUPP_CMDS,       MNG,    0, "",   "t64B"),
    CMD(RD_LOCAL_SUPP_FEATS,      MNG,    0, "",   "t8B"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    CMD(RD_LOCAL_EXT_FEATS,       BT_MNG, 1, "B",  "t10B"),
    CMD(RD_BUF_SIZE,              BT_MNG, 0, "",   "tHB2H"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

#endif //(!BT_HOST_OTHER_STACK)

#if (!BT_HOST_OTHER_STACK)
    CMD(RD_BD_ADDR,               MNG,    0, "",   "7B"),
#else
    CMD(RD_BD_ADDR,               MNG,    0, "",   "tT"),
#endif

#if (!BT_HOST_OTHER_STACK)

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    CMD(RD_LOCAL_SUPP_CODECS,     BT_MNG, 0, "",   "3B"),
    CMD(RD_LOCAL_SP_OPT,          BT_MNG, 0, "",   "3B"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    CMD(RD_LOCAL_SUPP_CODECS_V2,  MNG,    0,   "",     "3B"),   // TODO: update when the number of codecs is available
    CMD(RD_LOCAL_SUPP_CODEC_CAP,  MNG,    7,   "7B",   "2BnB"), // TODO: update when the number of codecs is available
    CMD(RD_LOCAL_SUPP_CTRL_DELAY, MNG,    255, "7BnB", "7B"),
#else // (!BT_HOST_OTHER_STACK)

#ifndef CFG_HCI_SLIM
#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    CMD(RD_LOCAL_SUPP_CODECS,     BT_MNG, 0, "",   "t2B"),
    CMD(RD_LOCAL_SP_OPT,          BT_MNG, 0, "",   "t2B"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    CMD(RD_LOCAL_SUPP_CODECS_V2,  MNG,    0,   "",     "t2B"),   // TODO: update when the number of codecs is available
    CMD(RD_LOCAL_SUPP_CODEC_CAP,  MNG,    7,   "7B",   "tBnB"), // TODO: update when the number of codecs is available
    CMD(RD_LOCAL_SUPP_CTRL_DELAY, MNG,    255, "7BnB", "t6B"),
#endif //CFG_HCI_SLIM
#endif // (!BT_HOST_OTHER_STACK)
};

#if (BT_HOST_OTHER_STACK)
const hci_cmd_desc_t hci_cmd_desc_tab_info_par_dual_stack[] =
{
    CMD(RD_BD_ADDR,               MNG,    0, "",   "7B"),
};
#endif // BT_HOST_OTHER_STACK

/// HCI command descriptors (OGF status parameters)
const hci_cmd_desc_t hci_cmd_desc_tab_stat_par[] =
{
#if (!BT_HOST_OTHER_STACK)
    // Note: all messages must be sorted in opcode ascending order
#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    CMD(RD_FAIL_CONTACT_CNT,            BT_CTRL_CONHDL, 2, "H",  "B2H"),
    CMD(RST_FAIL_CONTACT_CNT,           BT_CTRL_CONHDL, 2, "H",  "BH"),
    CMD(RD_LINK_QUAL,                   BT_CTRL_CONHDL, 2, "H",  "BHB"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    CMD(RD_RSSI,                        CTRL,           2, "H",  "BHB"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    CMD(RD_AFH_CH_MAP,                  BT_CTRL_CONHDL, 2, "H",  "BH11B"),
    CMD(RD_CLK,                         BT_MNG, 3, "HB", "BHLH"),
    CMD(RD_ENC_KEY_SIZE,                BT_CTRL_CONHDL, 2, "H",  "BHB"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

#else // (!BT_HOST_OTHER_STACK)
    // Note: all messages must be sorted in opcode ascending order
#ifndef CFG_HCI_SLIM
#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    CMD(RD_FAIL_CONTACT_CNT,            BT_CTRL_CONHDL, 2, "Hz",  "t2H"),
    CMD(RST_FAIL_CONTACT_CNT,           BT_CTRL_CONHDL, 2, "Hz",  "tH"),
    CMD(RD_LINK_QUAL,                   BT_CTRL_CONHDL, 2, "Hz",  "tHB"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    CMD(RD_RSSI,                        CTRL,           2, "Hz",  "tHB"),
#endif // !CFG_HCI_SLIM

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    CMD(RD_AFH_CH_MAP,                  BT_CTRL_CONHDL, 2, "Hz",  "tH11B"),
#ifndef CFG_HCI_SLIM
    CMD(RD_CLK,                         BT_MNG, 3, "HzB", "tHLH"),
    CMD(RD_ENC_KEY_SIZE,                BT_CTRL_CONHDL, 2, "H",  "tHB"),
#endif // !CFG_HCI_SLIM
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)
#endif // (!BT_HOST_OTHER_STACK)

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
#if (RW_MWS_COEX)
    CMD(GET_MWS_TRANSPORT_LAYER_CONFIG, BT_MNG,         0, "",   PK_SPE(GET_MWS_TRANSPORT_LAYER_CONFIG)),
#endif // (RW_MWS_COEX)
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)
};

/// HCI command descriptors (OGF testing)
#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
const hci_cmd_desc_t hci_cmd_desc_tab_testing[] =
{
#if (!BT_HOST_OTHER_STACK)
    // Note: all messages must be sorted in opcode ascending order
    CMD(RD_LOOPBACK_MODE,     BT_MNG,         0, "",    "2B"),
    CMD(WR_LOOPBACK_MODE,     BT_MNG,         1, "B",   "B"),
    CMD(EN_DUT_MODE,          BT_MNG,         0, "",    "B"),
    CMD(WR_SP_DBG_MODE,       BT_MNG,         1, "B",   "B"),
    CMD(WR_SEC_CON_TEST_MODE, BT_CTRL_CONHDL, 4, "H2B", "BH"),
#else // (!BT_HOST_OTHER_STACK)
    CMD(RD_LOOPBACK_MODE,     BT_MNG,         0, "",    "tB"),
    CMD(WR_LOOPBACK_MODE,     BT_MNG,         1, "B",   "t"),
    CMD(EN_DUT_MODE,          BT_MNG,         0, "",    "t"),
    CMD(WR_SP_DBG_MODE,       BT_MNG,         1, "B",   "t"),
    CMD(WR_SEC_CON_TEST_MODE, BT_CTRL_CONHDL, 4, "H2B", "tH"),

#endif //(!BT_HOST_OTHER_STACK)
};
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
/// HCI command descriptors (OGF LE controller)
const hci_cmd_desc_t hci_cmd_desc_tab_le[] =
{
    // Note: all messages must be sorted in opcode ascending order
    CMD(LE_SET_EVT_MASK,                      BLE_MNG,  8,   "8B",                    "B"),
    CMD(LE_RD_BUF_SIZE,                       BLE_MNG,  0,   "",                      "BHB"),
    CMD(LE_RD_LOCAL_SUPP_FEATS,               BLE_MNG,  0,   "",                      "9B"),
    CMD(LE_SET_RAND_ADDR,                     BLE_MNG,  6,   "6B",                    "B"),

#ifndef CFG_HCI_SLIM
#if (HCI_TL_SUPPORT)
    CMD(LE_SET_ADV_PARAM,                     BLE_MNG,  15,  "HHBBB6BBB",             "B"),
#endif // (HCI_TL_SUPPORT)
#endif // CFG_HCI_SLIM

    CMD(LE_RD_ADV_CHNL_TX_PW,                 BLE_MNG,  0,   "",                      "2B"),

#ifndef CFG_HCI_SLIM
#if (HCI_TL_SUPPORT)
    CMD(LE_SET_ADV_DATA,                      BLE_MNG,  32,  "32B",                   "B"),
    CMD(LE_SET_SCAN_RSP_DATA,                 BLE_MNG,  32,  "32B",                   "B"),
    CMD(LE_SET_ADV_EN,                        BLE_MNG,  1,   "B",                     "B"),
    CMD(LE_SET_SCAN_PARAM,                    BLE_MNG,  7,   "B2H2B",                 "B"),
    CMD(LE_SET_SCAN_EN,                       BLE_MNG,  2,   "2B",                    "B"),
    CMD(LE_CREATE_CON,                        BLE_MNG,  25,  "2H9B6H",                NULL),
#endif // (HCI_TL_SUPPORT)
#endif //CFG_HCI_SLIM

#if (BLE_CENTRAL)
    CMD(LE_CREATE_CON_CANCEL,                 BLE_MNG,  0,   "",                      "B"),
#endif // (BLE_CENTRAL)
    CMD(LE_RD_WLST_SIZE,                      BLE_MNG,  0,   "",                      "2B"),
    CMD(LE_CLEAR_WLST,                        BLE_MNG,  0,   "",                      "B"),
    CMD(LE_ADD_DEV_TO_WLST,                   BLE_MNG,  7,   "7B",                    "B"),
    CMD(LE_RMV_DEV_FROM_WLST,                 BLE_MNG,  7,   "7B",                    "B"),
    CMD(LE_CON_UPDATE,                        BLE_CTRL, 14,  "7H",                    NULL),
#ifndef CFG_HCI_SLIM
    CMD(LE_SET_HOST_CH_CLASS,                 BLE_MNG,  5,   "5B",                    "B"),
    CMD(LE_RD_CH_MAP,                         BLE_CTRL, 2,   "H",                     "BH5B"),
#endif //CFG_HCI_SLIM
    CMD(LE_RD_REM_FEATS,                      BLE_CTRL, 2,   "H",                     NULL),
    CMD(LE_ENC,                               BLE_MNG,  32,  "32B",                   "17B"),
    CMD(LE_RAND,                              BLE_MNG,  0,   "",                      "9B"),
    CMD(LE_EN_ENC,                            BLE_CTRL, 28,  "H8BH16B",               NULL),
    CMD(LE_LTK_REQ_REPLY,                     BLE_CTRL, 18,  "H16B",                  "BH"),
    CMD(LE_LTK_REQ_NEG_REPLY,                 BLE_CTRL, 2,   "H",                     "BH"),
    CMD(LE_RD_SUPP_STATES,                    BLE_MNG,  0,   "",                      "9B"),

#if (HCI_TL_SUPPORT)
    CMD(LE_RX_TEST_V1,                        BLE_MNG,  1,   "B",                     "B"),
    CMD(LE_TX_TEST_V1,                        BLE_MNG,  3,   "BBB",                   "B"),
#endif //(HCI_TL_SUPPORT)

    CMD(LE_TEST_END,                          BLE_MNG,  0,   "",                      "BH"),
    CMD(LE_REM_CON_PARAM_REQ_REPLY,           BLE_CTRL, 14,  "7H",                    "BH"),
    CMD(LE_REM_CON_PARAM_REQ_NEG_REPLY,       BLE_CTRL, 3,   "HB",                    "BH"),
    CMD(LE_SET_DATA_LEN,                      BLE_CTRL, 6,   "3H",                    "BH"),
#ifndef CFG_HCI_SLIM
    CMD(LE_RD_SUGGTED_DFT_DATA_LEN,           BLE_MNG,  0,   "",                      "B2H"),
#endif //CFG_HCI_SLIM
    CMD(LE_WR_SUGGTED_DFT_DATA_LEN,           BLE_MNG,  4,   "2H",                    "B"),
    CMD(LE_RD_LOC_P256_PUB_KEY,               BLE_MNG,  0,   "",                      NULL),
    CMD(LE_GEN_DHKEY_V1,                      BLE_MNG,  64,  "64B",                   NULL),
    CMD(LE_ADD_DEV_TO_RSLV_LIST,              BLE_MNG,  39,  "39B",                   "B"),
    CMD(LE_RMV_DEV_FROM_RSLV_LIST,            BLE_MNG,  7,   "7B",                    "B"),
    CMD(LE_CLEAR_RSLV_LIST,                   BLE_MNG,  0,   "",                      "B"),
    CMD(LE_RD_RSLV_LIST_SIZE,                 BLE_MNG,  0,   "",                      "2B"),
    CMD(LE_RD_PEER_RSLV_ADDR,                 BLE_MNG,  7,   "7B",                    "7B"),
    CMD(LE_RD_LOC_RSLV_ADDR,                  BLE_MNG,  7,   "7B",                    "7B"),
    CMD(LE_SET_ADDR_RESOL_EN,                 BLE_MNG,  1,   "B",                     "B"),
    CMD(LE_SET_RSLV_PRIV_ADDR_TO,             BLE_MNG,  2,   "H",                     "B"),
    CMD(LE_RD_MAX_DATA_LEN,                   BLE_MNG,  0,   "",                      "B4H"),
    CMD(LE_RD_PHY,                            BLE_CTRL, 2,   "H",                     "BH2B"),
    CMD(LE_SET_DFT_PHY,                       BLE_MNG,  3,   "3B",                    "B"),
    CMD(LE_SET_PHY,                           BLE_CTRL, 7,   "H3BH",                  NULL),
    CMD(LE_RX_TEST_V2,                        BLE_MNG,  3,   "3B",                    "B"),
    CMD(LE_TX_TEST_V2,                        BLE_MNG,  4,   "4B",                    "B"),
    CMD(LE_SET_ADV_SET_RAND_ADDR,             BLE_MNG,  7,   "7B",                    "B"),
    CMD(LE_SET_EXT_ADV_PARAM,                 BLE_MNG,  25,  "BH22B",                 "2B"),
    CMD(LE_SET_EXT_ADV_DATA,                  BLE_MNG,  255, "3BnB",                  "B"),
    CMD(LE_SET_EXT_SCAN_RSP_DATA,             BLE_MNG,  255, "3BnB",                  "B"),
    CMD(LE_SET_EXT_ADV_EN,                    BLE_MNG,  42,  "BS2BHB",                "B"),
    CMD(LE_RD_MAX_ADV_DATA_LEN,               BLE_MNG,  0,   "",                      "BH"),
    CMD(LE_RD_NB_SUPP_ADV_SETS,               BLE_MNG,  0,   "",                      "2B"),
    CMD(LE_RMV_ADV_SET,                       BLE_MNG,  1,   "B",                     "B"),
    CMD(LE_CLEAR_ADV_SETS,                    BLE_MNG,  0,   "",                      "B"),
#ifndef CFG_HCI_SLIM
    CMD(LE_SET_PER_ADV_PARAM,                 BLE_MNG,  7,   "B3H",                   "B"),
    CMD(LE_SET_PER_ADV_DATA,                  BLE_MNG,  255, "2BnB",                  "B"),
    CMD(LE_SET_PER_ADV_EN,                    BLE_MNG,  2,   "2B",                    "B"),
#endif // CFG_HCI_SLIM

#if (BLE_OBSERVER)
    CMD(LE_SET_EXT_SCAN_PARAM,                BLE_MNG,  13,  "2Bs2B2H",               "B"),
    CMD(LE_SET_EXT_SCAN_EN,                   BLE_MNG,  6,   "2B2H",                  "B"),
#endif // BLE_OBSERVER
#if (BLE_CENTRAL)
    CMD(LE_EXT_CREATE_CON,                    BLE_MNG,  58,  "9Bs28H",                NULL),
#endif

#ifndef CFG_HCI_SLIM
    CMD(LE_PER_ADV_CREATE_SYNC,               BLE_MNG,  14,  "9B2HB",                 NULL),
    CMD(LE_PER_ADV_CREATE_SYNC_CANCEL,        BLE_MNG,  0,   "",                      "B"),
    CMD(LE_PER_ADV_TERM_SYNC,                 BLE_MNG,  2,   "H",                     "B"),
    CMD(LE_ADD_DEV_TO_PER_ADV_LIST,           BLE_MNG,  8,   "8B",                    "B"),
    CMD(LE_RMV_DEV_FROM_PER_ADV_LIST,         BLE_MNG,  8,   "8B",                    "B"),
    CMD(LE_CLEAR_PER_ADV_LIST,                BLE_MNG,  0,   "",                      "B"),
    CMD(LE_RD_PER_ADV_LIST_SIZE,              BLE_MNG,  0,   "",                      "2B"),
#endif // CFG_HCI_SLIM
#ifndef CFG_HCI_SLIM
    CMD(LE_RD_TX_PWR,                         BLE_MNG,  0,   "",                      "3B"),
    CMD(LE_RD_RF_PATH_COMP,                   BLE_MNG,  0,   "",                      "B2H"),
#endif // CFG_HCI_SLIM
    CMD(LE_WR_RF_PATH_COMP,                   BLE_MNG,  4,   "2H",                    "B"),
    CMD(LE_SET_PRIV_MODE,                     BLE_MNG,  8,   "8B",                    "B"),
#ifndef CFG_HCI_SLIM
    CMD(LE_RX_TEST_V3,                        BLE_MNG,  82,  "6BnB",                  "B"),
    CMD(LE_TX_TEST_V3,                        BLE_MNG,  82,  "6BnB",                  "B"),
#endif // CFG_HCI_SLIM

#if (BLE_CONLESS_CTE_TX)
    CMD(LE_SET_CONLESS_CTE_TX_PARAM,          BLE_MNG,  80,  "4BnB",                  "B"),
    CMD(LE_SET_CONLESS_CTE_TX_EN,             BLE_MNG,  2,   "2B",                    "B"),
#endif // (BLE_CONLESS_CTE_TX)

#if (BLE_CONLESS_CTE_RX)
    CMD(LE_SET_CONLESS_IQ_SAMPL_EN,           BLE_MNG,  81,  "H3BnB",                 "BH"),
#endif // (BLE_CONLESS_CTE_RX)

#if (BLE_CON_CTE_REQ)
    CMD(LE_SET_CON_CTE_RX_PARAM,              BLE_CTRL, 80,  "H2BnB",                 "BH"),
#endif // (BLE_CON_CTE_REQ)

#if (BLE_CON_CTE_RSP)
    CMD(LE_SET_CON_CTE_TX_PARAM,              BLE_CTRL, 79,  "HBnB",                  "BH"),
#endif // (BLE_CON_CTE_RSP)

#if (BLE_CON_CTE_REQ)
    CMD(LE_CON_CTE_REQ_EN,                    BLE_CTRL, 7,   "HBH2B",                 "BH"),
#endif // (BLE_CON_CTE_REQ)

#if BLE_CON_CTE_RSP
    CMD(LE_CON_CTE_RSP_EN,                    BLE_CTRL, 3,   "HB",                    "BH"),
#endif // BLE_CON_CTE_RSP

#if (BLE_AOD | BLE_AOA)
    CMD(LE_RD_ANTENNA_INF,                    BLE_MNG,  0,   "",                      "5B"),
#endif // (BLE_AOD | BLE_AOA)

#ifndef CFG_HCI_SLIM
    CMD(LE_SET_PER_ADV_REC_EN,                BLE_MNG,  3,   "HB",                    "B"),
#endif //CFG_HCI_SLIM

#if (BLE_PAST)
    CMD(LE_PER_ADV_SYNC_TRANSF,               BLE_CTRL, 6,   "3H",                    "BH"),
    CMD(LE_PER_ADV_SET_INFO_TRANSF,           BLE_CTRL, 5,   "2HB",                   "BH"),
    CMD(LE_SET_PER_ADV_SYNC_TRANSF_PARAM,     BLE_CTRL, 8,   "HB2HB",                 "BH"),
    CMD(LE_SET_DFT_PER_ADV_SYNC_TRANSF_PARAM, BLE_MNG,  6,   "B2HB",                  "B"),
#endif // (BLE_PAST)
    CMD(LE_GEN_DHKEY_V2,                      BLE_MNG,  65,  "65B",                   NULL),
#ifndef CFG_HCI_SLIM
    CMD(LE_MOD_SLEEP_CLK_ACC,                 BLE_MNG,  1,   "B",                     "B"),
#endif // CFG_HCI_SLIM

#if (BLE_CIS | BLE_BIS)
    /// Read buffer size v2
    CMD(LE_RD_BUF_SIZE_V2,                    BLE_ISO,  0,   "",                      "BHBHB"),
    CMD(LE_RD_ISO_TX_SYNC,                    BLE_ISO,  2,   "H",                     "B2HL3B"),

    /// CIS Specific commands
#if (BLE_CIS)
#if (BLE_CENTRAL)
    CMD(LE_SET_CIG_PARAMS,                    BLE_ISO,  255, "BDD3B2HS2B2H4B",        "2BnH"),
    CMD(LE_SET_CIG_PARAMS_TEST,               BLE_ISO,  255, "BDD2BH3BS22B4H4B",      "2BnH"),
    CMD(LE_CREATE_CIS,                        BLE_ISO,  125, "S22H",                  NULL),
    CMD(LE_REMOVE_CIG,                        BLE_ISO,  1,   "B",                     "2B"),
#endif // (BLE_CENTRAL)

#if (BLE_PERIPHERAL)
    CMD(LE_ACCEPT_CIS_REQ,                    BLE_ISO,  2,   "H",                     NULL),
    CMD(LE_REJECT_CIS_REQ,                    BLE_ISO,  3,   "HB",                    "BH"),
#endif // (BLE_PERIPHERAL)
#endif // (BLE_CIS)

    /// BIS Specific commands
#if (BLE_BIS)
#if (BLE_BROADCASTER)
    CMD(LE_CREATE_BIG,                        BLE_ISO,  31,  "3BD2H21B",              NULL),
    CMD(LE_CREATE_BIG_TEST,                   BLE_ISO,  36,  "3BDHB2H23B",            NULL),
    CMD(LE_TERMINATE_BIG,                     BLE_ISO,  2,   "BB",                    NULL),
#endif // (BLE_BROADCASTER)

#if (BLE_OBSERVER)
    CMD(LE_BIG_CREATE_SYNC,                   BLE_ISO,  55,  "BHB17BHnB",             NULL),
    CMD(LE_BIG_TERMINATE_SYNC,                BLE_ISO,  1,   "B",                     "2B"),
#endif // (BLE_OBSERVER)
#endif // (BLE_BIS)

    CMD(LE_REQ_PEER_SCA,                      BLE_CTRL, 2,   "H",                     NULL),
    /// ISO Datapath configuration specific commands
    CMD(LE_SETUP_ISO_DATA_PATH,               BLE_ISO,  255, "H10BnB",                "BH"),
    CMD(LE_REMOVE_ISO_DATA_PATH,              BLE_ISO,  3,   "HB",                    "BH"),
    /// ISO test commands
    CMD(LE_ISO_TX_TEST,                       BLE_ISO,  3,   "HB",                    "BH"),
    CMD(LE_ISO_RX_TEST,                       BLE_ISO,  3,   "HB",                    "BH"),
    CMD(LE_ISO_READ_TEST_COUNTERS,            BLE_ISO,  2,   "H",                     "BH3L"),
    CMD(LE_ISO_TEST_END,                      BLE_ISO,  2,   "H",                     "BH3L"),
    CMD(LE_SET_HOST_FEATURE,                  BLE_MNG,  2,   "2B",                    "B"),
    CMD(LE_RD_ISO_LINK_QUALITY,               BLE_ISO,  2,   "H",                     "BH7L"),
#endif // (BLE_CIS | BLE_BIS)

#if (BLE_PWR_CTRL)
    /// LE Power Control commands
    CMD(LE_ENH_RD_TX_PWR_LVL,                 BLE_CTRL, 3,   "HB",                    "BH3B"),
    CMD(LE_RD_REMOTE_TX_PWR_LVL,              BLE_CTRL, 3,   "HB",                    NULL),
    CMD(LE_SET_PATH_LOSS_REP_PARAM,           BLE_CTRL, 8,   "H4BH",                  "BH"),
    CMD(LE_SET_PATH_LOSS_REP_EN,              BLE_CTRL, 3,   "HB",                    "BH"),
    CMD(LE_SET_TX_POWER_REP_EN,               BLE_CTRL, 4,   "H2B",                   "BH"),
#endif // (BLE_PWR_CTRL)

#ifndef CFG_HCI_SLIM
    CMD(LE_TX_TEST_V4,                        BLE_MNG,  83,  PK_SPE(LE_TX_TEST_V4),   "B"),
#endif //CFG_HCI_SLIM

#if (BT_53)
    CMD(LE_SET_DATA_RELATED_ADDR_CHG,         BLE_MNG,  2,   "2B",                    "B"),
#if (BLE_CENTRAL)
    CMD(LE_SET_DFT_SUBRATE,                   BLE_MNG,  10,  "5H",                    "B"),
#endif // (BLE_CENTRAL)
    CMD(LE_SUBRATE_REQ,                       BLE_CTRL, 12,  "6H",                    NULL),
#endif // (BT_53)
};
#endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)

///// HCI command descriptors (OGF Vendor Specific)
const hci_cmd_desc_t hci_cmd_desc_tab_vs[] =
{
    // Note: all messages must be sorted in opcode ascending order
#if (RW_DEBUG && (!HOST_PRESENT || HCI_TL_SUPPORT))
    CMD(DBG_RD_MEM,                  DBG,             6, "L2B",      "BnB"),
    CMD(DBG_WR_MEM,                  DBG,           136, "LBnB",     "B"),
    CMD(DBG_DEL_PAR,                 DBG,             2, "H",        "B"),
    CMD(DBG_ID_FLASH,                DBG,             0, "",         "2B"),
    CMD(DBG_ER_FLASH,                DBG,             9, "B2L",      "B"),
    CMD(DBG_WR_FLASH,                DBG,           140, "BLnB",     "B"),
    CMD(DBG_RD_FLASH,                DBG,             6, "BLB",      "BnB"),
    CMD(DBG_RD_PAR,                  DBG,             2, "H",        "BnB"),
    CMD(DBG_WR_PAR,                  DBG,           132, "HnB",      "B"),
#endif //(RW_DEBUG && (!HOST_PRESENT || HCI_TL_SUPPORT))

#if (RW_DEBUG)
#if (RW_WLAN_COEX)
    CMD(DBG_WLAN_COEX,               DBG,             1, "B",        "B"),
#if (RW_WLAN_COEX_TEST)
    CMD(DBG_WLAN_COEXTST_SCEN,       DBG,             4, "L",        "B"),
#endif // (RW_BT_WLAN_COEX_TEST)
#endif // (RW_WLAN_COEX)

#if (BT_EMB_PRESENT)
    CMD(DBG_BT_SEND_LMP,             BT_CTRL_CONHDL,  131, "HnB",    "BH"),
    CMD(DBG_SET_LOCAL_CLOCK,    DBG,  4, "L",      "B"),
#endif // (BT_EMB_PRESENT)

    CMD(DBG_RD_KE_STATS,             DBG,               0, "",       "B5L"),
#endif // (RW_DEBUG)

    CMD(DBG_PLF_RESET,               DBG,               1, "B",      "B"),
#if(!HOST_PRESENT)
    CMD(DBG_RD_MEM_INFO,             DBG,               0, "",       "B3HL"),
#else // (HOST_PRESENT)
#ifndef CFG_HCI_SLIM
    CMD(DBG_RD_MEM_INFO,             DBG,               0, "",       "B4HL"),
#endif //CFG_HCI_SLIM
#endif // (!HOST_PRESENT)

#if (BLE_EMB_PRESENT)
#if (BLE_PERIPHERAL)
    CMD(VS_SET_PREF_SLAVE_LATENCY,   BLE_CTRL,          4, "HH",     "BH"),
    CMD(VS_SET_PREF_SLAVE_EVT_DUR,   BLE_CTRL,          5, "2HB",    "BH"),
#endif // (BLE_PERIPHERAL)

    CMD(VS_SET_MAX_RX_SIZE_AND_TIME, BLE_CTRL,          6, "3H",     "BH"),
    CMD(VS_SET_PRIO_ELEVATION,       BLE_CTRL,          3, "HB",     "BH"),
#if (RW_DEBUG)
    CMD(DBG_LE_CON_EVT_TRACE,        DBG,               3, "HB",     "B"),
#endif // (RW_DEBUG)
#if (BLE_CH_SCAN_SUPPORT)
    CMD(VS_LE_CH_SCAN,               BLE_MNG,          11, "2LHB",   "B"),
    CMD(VS_LE_CH_SCAN_END,           BLE_MNG,           0, NULL,     "B"),
#endif // (BLE_CH_SCAN_SUPPORT)
#if (RW_DEBUG)
    CMD(DBG_BLE_REG_RD,              BLE_MNG,           2, "H",      "BHL"),
    CMD(DBG_BLE_REG_WR,              BLE_MNG,           6, "HL",     "BH"),
    CMD(DBG_SEND_LLCP,               BLE_CTRL,        131, "HnB",    "BH"),
    CMD(DBG_LLCP_DISCARD,            BLE_CTRL,          3, "HB",     "B"),
    CMD(DBG_RF_REG_RD,               DBG,               2, "H",      "BHL"),
    CMD(DBG_RF_REG_WR,               DBG,               6, "HL",     "BH"),
#endif // (RW_DEBUG)
#endif // (BLE_EMB_PRESENT)

#if (BT_EMB_PRESENT && RW_DEBUG)
    CMD(DBG_BT_DISCARD_LMP_EN,       BT_CTRL_CONHDL,    3, "HB",     "BH"),
#endif // (BT_EMB_PRESENT && RW_DEBUG)

#if (RW_DEBUG)
#if (RW_MWS_COEX)
    CMD(DBG_MWS_COEX,                DBG,               1, "B",      "B"),

#if (RW_MWS_COEX_TEST)
    CMD(DBG_MWS_COEXTST_SCEN,        DBG,               4, "L",      "B"),
#endif // (RW_BT_MWS_COEX_TEST)

#endif // (RW_MWS_COEX)
#endif // (RW_DEBUG)

#if (BT_I2C_OVER_HCI)
    CMD(VS_I2C_READ,                 DBG,               3, "3B",     "BnB"),
    CMD(VS_I2C_WRITE,                DBG,             255, "2BnB",   "B"),
#endif // (BT_I2C_OVER_HCI)

#if (BT_READ_PICONET_CLOCK)
    CMD(VS_RD_PICONET_CLOCK,         BT_MNG,            4, "H2B",    "BHLHLH"),
#endif // (BT_READ_PICONET_CLOCK)

#if (BLE_EMB_PRESENT)
#if (BLE_ISO_MODE_0)
    CMD(VS_MIC_LESS_SET,              BLE_ISO,          2, "H",      "B"),
    CMD(VS_SETUP_AM0_DATA_PATH,       BLE_ISO,          8, "H2BL",   "BH"),
    CMD(VS_REMOVE_AM0_DATA_PATH,      BLE_ISO,          3, "HB",     "BH"),
    CMD(VS_SETUP_AM0_STREAM,          BLE_ISO,          4, "H2B",    "B2H"),
    CMD(VS_REMOVE_AM0_STREAM,         BLE_ISO,          2, "H",      "BH"),
#endif //(BLE_ISO_MODE_0)
#endif // (BLE_EMB_PRESENT)

#if (RW_DEBUG)
#if (BLE_CIS || BLE_BIS)
    CMD(DBG_ISO_SET_PARAM,            BLE_ISO,          4, "L",      "B"),
#endif // (BLE_CIS || BLE_BIS)
#endif // (RW_DEBUG)

#if (CRYPTO_UT)
    CMD(DBG_TEST_CRYPTO_FUNC,         BT_MNG,         136, "BnB",      "B"),
#endif // (CRYPTO_UT)

#if (RW_DEBUG && SCH_PLAN_UT)
    CMD(DBG_TEST_SCH_PLAN_SET,        DBG,             23, "4L3HB", "BH"),
    CMD(DBG_TEST_SCH_PLAN_REM,        DBG,             2, "H", "B"),
    CMD(DBG_TEST_SCH_PLAN_CHK,        DBG,             18, "3L3H", "B"),
    CMD(DBG_TEST_SCH_PLAN_REQ,        DBG,             23, "4L3HB", "B3L"),
#endif // (RW_DEBUG && SCH_PLAN_UT)

#if (BLE_IQ_GEN)
    CMD(DBG_IQGEN_CFG,                DBG,            18, "BS12B",     "B"),
#endif // (BLE_IQ_GEN)

#if (AUDIO_SYNC_SUPPORT)
#if(RW_DEBUG)
    CMD(VS_AUSY_CON_EVT_CNT_GET,      DBG,            2, AUSY_CON_EVT_CNT_GET_CMD_PK,  AUSY_CON_EVT_CNT_GET_CMP_EVT_PK),
    CMD(VS_AUSY_EVT_TX_TIME_GET,      DBG,            4, AUSY_EVT_TX_TIME_GET_CMD_PK,  AUSY_EVT_TX_TIME_GET_CMP_EVT_PK),
    CMD(VS_AUSY_LAST_RX_TIME_GET,     DBG,            2, AUSY_LAST_RX_TIME_GET_CMD_PK, AUSY_LAST_RX_TIME_GET_CMP_EVT_PK),
    CMD(VS_AUSY_CLOCK_SAMPLE_GET,     DBG,            0, AUSY_CLOCK_SAMPLE_GET_CMD_PK, AUSY_CLOCK_SAMPLE_GET_CMP_EVT_PK),
    CMD(VS_AUSY_CLOCK_CONVERT,        DBG,            6, AUSY_CLOCK_CONVERT_CMD_PK,    AUSY_CLOCK_CONVERT_CMP_EVT_PK),
#endif // (RW_DEBUG)
#if (BLE_CIS)
    CMD(VS_AUSY_CIS_EVT_CTRL,         DBG,            1, AUSY_CIS_EVT_CTRL_CMD_PK,     AUSY_CIS_EVT_CTRL_CMP_EVT_PK),
#endif // (BLE_CIS)
#if (BLE_BIS)
    CMD(VS_AUSY_BIS_EVT_CTRL,         DBG,            1, AUSY_BIS_EVT_CTRL_CMD_PK,     AUSY_BIS_EVT_CTRL_CMP_EVT_PK),
#endif // (BLE_BIS)
#endif // (AUDIO_SYNC_SUPPORT)
#if (BT_HCI_TEST_MODE)
    CMD(VS_RX_TEST,                   COMMON,         2,  "2B",      "B"),
    CMD(VS_TX_TEST,                   COMMON,         6,  "BH3B",    "B"),
    CMD(VS_TEST_END,                  COMMON,         0,   NULL,     "BH"),
#endif // (BT_HCI_TEST_MODE)

#if (BT_53)
    CMD(DBG_SET_MAX_ENC_KEY_SIZE,     BT_MNG,         1,   "B",       "B"),
#endif // (BT_53)

#if ((BLE_EMB_PRESENT && !HOST_PRESENT) || (!EMB_PRESENT && BLE_HOST_PRESENT))
    CMD(VS_LE_DECRYPT,                BLE_MNG,        32,  "32B",     "17B"),
#endif //  ((BLE_EMB_PRESENT && !HOST_PRESENT) || (!EMB_PRESENT && BLE_HOST_PRESENT))

#if (EAVESDROPPING_SUPPORT)
    CMD(ECI_SET_OPERATIONAL_MODE, ED,                7,  "B6B", "B"),
    CMD(ECI_SET_POWER, ED,                7,  "6BB", "B"),
    CMD(ECI_SET_CHANNEL_CLASSIFICATION, ED,                16, "6B10B", "B"),
    CMD(ECI_CAPTURE_MASTER_CLOCK, ED,                7,  "6BB", NULL),
    CMD(ECI_GET_AES_CCM_COUNTER, ED,                7,  "6BB", "BB5B"),
    CMD(ECI_SET_EAVESDROPPING_STATE, ED,                8,  "6BBB", "B"),
    CMD(ECI_SET_ACL_CONNECTION, ED,                9,  "6BBBB", "B"),
    CMD(ECI_SET_AFH_CHANNEL_MAP, ED,                21, "6BBL10B", "B"),
    CMD(ECI_SET_ENCRYPTION, ED,                32, "6BB16BB8B", "B"),
    CMD(ECI_SET_PACKET_TYPE_TABLE, ED,                7,  "6BB", "B"),
    CMD(ECI_SET_MAX_SLOT, ED,                7,  "6BB", "B"),
    CMD(ECI_SET_SCO_CONNECTION, ED,                20, "6BBBBBBBBBBHHB", "B"),
    CMD(ECI_SET_MIN_MAX_POWER, ED,                7,  "6BB", "B"),
    CMD(ECI_SET_MASTER_CLOCK, ED,                18, "6BLHLH", NULL),
    CMD(ECI_SET_AES_CCM_COUNTER, ED,                12, "6BB5B", "B"),
    CMD(ECI_SET_CLOCK_OFFSETS, ED,                12, "6BLH", "B"),
    CMD(ECI_CLEAR_CAPTURE_MASTER_CLOCK, ED,                0,  NULL, "BB"),
    CMD(ECI_SET_LRD_EAVESDROPPING, ED,                13, "6BBBBBHB", "B"),
    CMD(ECI_ALLOW_TX_PROG, ED,                6,  "6B", "B"),
    CMD(ECI_SET_TX_SEQN, ED,                7,  "6BB", "B"),
    CMD(ECI_DELIVER_CORR_ACL_PAYL, ED,                7,  "6BB", "B"),
    CMD(ECI_RX_STATS_REPORTING, ED,                9,  "6BBBB", "B"),
    CMD(ECI_SAMPLE_ESCO_TX_SEQN, ED,                6, "6B", "B"),
    CMD(ECI_SYNC_ESCO_TX_SEQN, ED,                11, "6BBL", "B"),
    CMD(VS_SET_SEG_INQ_SCAN, BT_MNG,            34, "BHHHHB16B6BH", "B"),
    CMD(VS_SET_SEG_PAGE_SCAN, BT_MNG,            34, "BHHHHB16B6BH", "B"),
    CMD(VS_SET_CUST_ADV, BLE_MNG,           16, "BBHHB6BHB", "B"),
    CMD(VS_SET_CUST_ACL_SCHED, BT_CTRL_BD_ADDR,   8,  "6BBB", "B6B"),
    CMD(VS_SET_HOST_MAX_SLOT, BT_MNG,            1,  "B", "B"),
    CMD(VS_GET_DEEP_SLEEP_COUNTER, DBG,          0,  NULL, "BL"),
    CMD(VS_PREVENT_DEEP_SLEEP, DBG,          1,  "B", "B"),
    CMD(VS_SET_WLAN_COEX_PRIORITIES, DBG,          144, "144B", "B"),
    CMD(VS_SET_SCO_LINK_REQUEST_DELAY, BT_MNG,          2,  "H", "B"),
    CMD(VS_SET_CUST_PAGING, BT_MNG,          4,  "BBH", "B"),
    CMD(VS_NOTIFY_CLOCK_WRAP, BT_MNG,          7,  "B6B", "B"),
    CMD(VS_NOTIFY_SNIFF_EVENTS, BT_MNG,          1,  "B", "B"),
    CMD(VS_LE_ENABLE_ENH_SET_DATA_LEN, BLE_MNG,          1,  "B", "B"),
    CMD(VS_LE_ENH_SET_DATA_LEN, BLE_CTRL,          10, "HHHHH", "BH"),
    CMD(VS_LE_ENH_WR_SUGG_DFT_DATA_LEN, BLE_MNG,          8,  "HHHH", "B"),
    CMD(VS_LE_SET_CUST_TRANSMITTER_TEST, BLE_MNG,           4,  "BHB", "B"),
    CMD(VS_HANDLE_MIC_INVALID, ED,                12, "6BBBBBBB", "B"),
    CMD(VS_LE_SET_MANUAL_ADV, BLE_MNG,           6,  "BBHH", "B"),
    CMD(VS_LE_SET_SEG_SCAN, BLE_MNG,           28, "B6Bs25H", "B"),
#endif // EAVESDROPPING_SUPPORT
    CMD(DBG_SET_TRC_CFG_EXT,               DBG,          4,     "L",       "B"),
    CMD(DBG_RESERVED1_EXT,             COMMON,          4,     "4B",      "B"),
    CMD(DBG_RESERVED2_EXT,             COMMON,          4,     "4B",      "B"),
    CMD(DBG_RESERVED6_EXT,             COMMON,          8,     "8B",      "B"),

#if (RW_DEBUG) && (RW_MWS_COEX)
    CMD(VS_EBQ_INIT_SAM_NEGO, BT_MNG,            1,  "B", "B"),
#endif // (RW_DEBUG) && (RW_MWS_COEX)


#if (NVDS_SUPPORT)
    CMD(NVDS_DATA_UPDATE,             COMMON,        255,  "nB",      "B"),
#endif
#ifndef CFG_HCI_SLIM
    CMD(WLAN_COEX_ENABLE,             COMMON,        4,    "4B",       "B"),
    CMD(WLAN_COEX_SET_PRIORITY,       COMMON,        21,   "B5L",      "B"),
    CMD(WLAN_COEX_SET_FILTER_CHANNEL, COMMON,        10,   "10B",     "B"),
#endif // CFG_HCI_SLIM
    CMD(DBG_SET_TRC_CFG,               DBG,          4,     "L",       "B"),

    CMD(DBG_RESERVED1,                 COMMON,          4,     "4B",      "B"),
    CMD(DBG_RESERVED2,                 COMMON,          4,     "4B",      "B"),
#ifndef CFG_HCI_SLIM
    CMD(DBG_RESERVED3,                 COMMON,          4,     "4B",      "B"),
    CMD(DBG_RESERVED4,                 COMMON,          4,     "4B",      "B"),
    CMD(DBG_RESERVED5,                 COMMON,          4,     "4B",      "B"),
#endif //CFG_HCI_SLIM
    CMD(DBG_RESERVED6,                 COMMON,          8,     "8B",      "B"),
#ifndef CFG_HCI_SLIM
    CMD(DBG_RESERVED7,                 COMMON,          8,     "8B",      "B"),
    CMD(DBG_RESERVED8,                 COMMON,          8,     "8B",      "B"),
    CMD(DBG_RESERVED9,                 COMMON,          8,     "8B",      "B"),
    CMD(DBG_RESERVED10,                COMMON,          8,     "8B",      "B"),
    CMD(DBG_RESERVED11,                COMMON,          16,    "16B",     "B"),
    CMD(DBG_RESERVED12,                COMMON,          16,    "16B",     "B"),
#endif //CFG_HCI_SLIM

};

const uint8_t hci_msg_ogf_to_idx[HCI_MSG_NB_OGF_SUPP] =
{
    ///HCI Link Control Commands Group OGF code index
    [0] = HCI_OGF_IDX_LK_CNTL,           // LK_CNTL_OGF
#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    ///HCI Link Policy Commands Group OGF code index
    [1] = HCI_OGF_IDX_LK_POL,            // LK_POL_OGF
#else
    [1] = HCI_OGF_IDX_INVALID,           // LK_POL_OGF
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)
    ///HCI Controller and Baseband Commands Group OGF code index
    [2] = HCI_OGF_IDX_CNTLR_BB,          // CNTLR_BB_OGF
    ///HCI Information Parameters Commands Group OGF code index
    [3] = HCI_OGF_IDX_INFO_PAR,          // INFO_PAR_OGF
    ///HCI Status Commands Group OGF code index
    [4] = HCI_OGF_IDX_STAT_PAR,          // STAT_PAR_OGF
    ///HCI Test Commands Group OGF code index
#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    [5] = HCI_OGF_IDX_TEST,              // TEST_OGF
#else
    [5] = HCI_OGF_IDX_INVALID,           // TEST_OGF
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)
    ///HCI Vendor Specific Group OGF code index
    [6] = HCI_OGF_IDX_VS,                // VS_OGF
    ///HCI Low Energy Commands Group OGF code index
#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
    [7] = HCI_OGF_IDX_LE_CNTLR,          // LE_CNTLR_OGF
#else
    [7] = HCI_OGF_IDX_INVALID,           // LE_CNTLR_OGF
#endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)
};

/// HCI command descriptors root table (classified by OGF)
const struct hci_cmd_desc_tab_ref hci_cmd_desc_root_tab[] =
{
    // Note: all messages must be sorted in opcode ascending order
    [HCI_OGF_IDX_LK_CNTL]     = {ARRAY_LEN(hci_cmd_desc_tab_lk_ctrl),  hci_cmd_desc_tab_lk_ctrl },

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    [HCI_OGF_IDX_LK_POL]      = {ARRAY_LEN(hci_cmd_desc_tab_lk_pol),   hci_cmd_desc_tab_lk_pol  },
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    [HCI_OGF_IDX_CNTLR_BB]    = {ARRAY_LEN(hci_cmd_desc_tab_ctrl_bb),  hci_cmd_desc_tab_ctrl_bb },
    [HCI_OGF_IDX_INFO_PAR]    = {ARRAY_LEN(hci_cmd_desc_tab_info_par), hci_cmd_desc_tab_info_par},
    [HCI_OGF_IDX_STAT_PAR]    = {ARRAY_LEN(hci_cmd_desc_tab_stat_par), hci_cmd_desc_tab_stat_par},

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    [HCI_OGF_IDX_TEST]        = {ARRAY_LEN(hci_cmd_desc_tab_testing),  hci_cmd_desc_tab_testing },
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    [HCI_OGF_IDX_VS]          = {ARRAY_LEN(hci_cmd_desc_tab_vs),       hci_cmd_desc_tab_vs      },

#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
    [HCI_OGF_IDX_LE_CNTLR]    = {ARRAY_LEN(hci_cmd_desc_tab_le),       hci_cmd_desc_tab_le      },
#endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)
};

/// HCI event descriptors table
const hci_evt_desc_t hci_evt_desc_tab[] =
{
#if (!BT_HOST_OTHER_STACK)
    // Note: all messages must be sorted in opcode ascending order
#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    EVT(INQ_CMP,                   HOST_TL_PK, "B"),
    EVT(INQ_RES,                   HOST_TL_PK, "13BH"),
    EVT(CON_CMP,                   HOST_TL_PK,  "BH8B"),
    EVT(CON_REQ,                   HOST_TL_PK,  "10B"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    EVT(DISC_CMP,                  HOST_UNDEF,  "BHB"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    EVT(AUTH_CMP,                  HOST_TL_PK,  "BH"),
    EVT(REM_NAME_REQ_CMP,          HOST_TL_PK,  "255B"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    EVT(ENC_CHG,                   HOST_UNDEF,  "BHB"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    EVT(CHG_CON_LK_CMP,            HOST_TL_PK,  "BH"),
#if BCAST_ENC_SUPPORT
    EVT(MASTER_LK_CMP,             HOST_TL_PK,  "BHB"),
#endif // BCAST_ENC_SUPPORT
    EVT(RD_REM_SUPP_FEATS_CMP,     HOST_TL_PK,  "BH8B"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    EVT(RD_REM_VER_INFO_CMP,       HOST_UNDEF,  "BHB2H"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    EVT(QOS_SETUP_CMP,             HOST_TL_PK,  "BH2B4L"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    EVT(HW_ERR,                    HOST_TL_PK,  "B"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    EVT(FLUSH_OCCURRED,            HOST_TL_PK,  "H"),
    EVT(ROLE_CHG,                  HOST_TL_PK,  "8B"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    EVT(NB_CMP_PKTS,               HOST_UNDEF,  "S22H"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    EVT(MODE_CHG,                  HOST_TL_PK,  "BHBH"),
    EVT(RETURN_LINK_KEYS,          HOST_TL_PK,  "23B"),
    EVT(PIN_CODE_REQ,              HOST_TL_PK,  "6B"),
    EVT(LK_REQ,                    HOST_TL_PK,  "6B"),
    EVT(LK_NOTIF,                  HOST_TL_PK,  "23B"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    EVT(DATA_BUF_OVFLW,            HOST_TL_PK,  "B"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    EVT(MAX_SLOT_CHG,              HOST_TL_PK,  "HB"),
    EVT(RD_CLK_OFF_CMP,            HOST_TL_PK,  "B2H"),
    EVT(CON_PKT_TYPE_CHG,          HOST_TL_PK,  "B2H"),
    EVT(QOS_VIOL,                  HOST_TL_PK,  "H"),
    EVT(PAGE_SCAN_REPET_MODE_CHG,  HOST_TL_PK,  "7B"),
    EVT(FLOW_SPEC_CMP,             HOST_TL_PK,  "BH3B4L"),
    EVT(INQ_RES_WITH_RSSI,         HOST_TL_PK,  "12BHB"),
    EVT(RD_REM_EXT_FEATS_CMP,      HOST_TL_PK,  "BH10B"),
    EVT(SYNC_CON_CMP,              HOST_TL_PK,  "BH9B2HB"),
    EVT(SYNC_CON_CHG,              HOST_TL_PK,  "BH2B2H"),
    EVT(SNIFF_SUB,                 HOST_TL_PK,  "B5H"),
    EVT(EXT_INQ_RES,               HOST_TL_PK, "12BH241B"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    EVT(ENC_KEY_REFRESH_CMP,       HOST_UNDEF,  "BH"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    EVT(IO_CAP_REQ,                HOST_TL_PK,  "6B"),
    EVT(IO_CAP_RSP,                HOST_TL_PK,  "9B"),
    EVT(USER_CFM_REQ,              HOST_TL_PK,  "6BL"),
    EVT(USER_PASSKEY_REQ,          HOST_TL_PK,  "6B"),
    EVT(REM_OOB_DATA_REQ,          HOST_TL_PK,  "6B"),
    EVT(SP_CMP,                    HOST_TL_PK,  "7B"),
    EVT(LINK_SUPV_TO_CHG,          HOST_TL_PK,  "2H"),
    EVT(ENH_FLUSH_CMP,             HOST_TL_PK,  "H"),
    EVT(USER_PASSKEY_NOTIF,        HOST_TL_PK,  "6BL"),
    EVT(KEYPRESS_NOTIF,            HOST_TL_PK,  "7B"),
    EVT(REM_HOST_SUPP_FEATS_NOTIF, HOST_TL_UPK, "14B"),
#if (CSB_SUPPORT)
    EVT(SYNC_TRAIN_CMP,            HOST_TL_PK,  "B"),
    EVT(SYNC_TRAIN_REC,            HOST_TL_PK,  "7BL11BLHB"),
    EVT(CON_SLV_BCST_REC,          HOST_TL_PK,  "7B2L2BnB"),
    EVT(CON_SLV_BCST_TO,           HOST_TL_PK,  "7B"),
    EVT(TRUNC_PAGE_CMP,            HOST_TL_PK,  "7B"),
    EVT(SLV_PAGE_RSP_TO,           HOST_TL_PK,  NULL),
    EVT(CON_SLV_BCST_CH_MAP_CHG,   HOST_TL_PK,  "10B"),
#endif // (CSB_SUPPORT)
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    EVT(AUTH_PAYL_TO_EXP,          HOST_UNDEF,  "H"),
#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    EVT(SAM_STATUS_CHANGE,         HOST_TL_PK,  "H6B"),
#endif// (BT_EMB_PRESENT || BT_HOST_PRESENT)

#if (BT_53)
    EVT(ENC_CHG_V2,                HOST_TL_PK,  "BH2B"),
#endif // (BT_53)

#else // !BT_HOST_OTHER_STACK

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
#ifdef CFG_INQ
    EVT(INQ_CMP,                   HOST_TL_PK, "B"),
    EVT(INQ_RES,                   HOST_TL_PK, "BT3BDH"),
#endif // CFG_INQ
    EVT(CON_CMP,                   HOST_TL_PK,  "BHT2B"),
    EVT(CON_REQ,                   HOST_TL_PK,  "TDB"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    EVT(DISC_CMP,                  HOST_UNDEF,  "BHB"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    EVT(AUTH_CMP,                  HOST_TL_PK,  "BH"),
    EVT(REM_NAME_REQ_CMP,          HOST_TL_PK,  "BT248B"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    EVT(ENC_CHG,                   HOST_UNDEF,  "BHB"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    EVT(CHG_CON_LK_CMP,            HOST_TL_PK,  "BH"),
    EVT(RD_REM_SUPP_FEATS_CMP,     HOST_TL_PK,  "BH8B"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    EVT(RD_REM_VER_INFO_CMP,       HOST_UNDEF,  "BHB2H"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    EVT(QOS_SETUP_CMP,             HOST_TL_PK,  "BH2B4L"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    EVT(HW_ERR,                    HOST_TL_PK,  "B"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    EVT(FLUSH_OCCURRED,            HOST_TL_PK,  "H"),
    EVT(ROLE_CHG,                  HOST_TL_PK,  "BTB"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    EVT(NB_CMP_PKTS,               HOST_UNDEF,  "S22H"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    EVT(MODE_CHG,                  HOST_TL_PK,  "BHBH"),
    EVT(RETURN_LINK_KEYS,          HOST_TL_PK,  "BT16B"),
    EVT(PIN_CODE_REQ,              HOST_TL_PK,  "T"),
    EVT(LK_REQ,                    HOST_TL_PK,  "T"),
    EVT(LK_NOTIF,                  HOST_TL_PK,  "T17B"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    EVT(DATA_BUF_OVFLW,            HOST_TL_PK,  "B"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    EVT(MAX_SLOT_CHG,              HOST_TL_PK,  "HB"),
    EVT(RD_CLK_OFF_CMP,            HOST_TL_PK,  "B2H"),
    EVT(CON_PKT_TYPE_CHG,          HOST_TL_PK,  "B2H"),
    EVT(QOS_VIOL,                  HOST_TL_PK,  "H"),
    EVT(PAGE_SCAN_REPET_MODE_CHG,  HOST_TL_PK,  "TB"),
    EVT(FLOW_SPEC_CMP,             HOST_TL_PK,  "BH3B4L"),
#ifdef CFG_INQ
    EVT(INQ_RES_WITH_RSSI,         HOST_TL_PK,  "BT2BDHB"),
#endif  //CFG_INQ
    EVT(RD_REM_EXT_FEATS_CMP,      HOST_TL_PK,  "BH10B"),
#if (MAX_NB_SYNC > 0)
    EVT(SYNC_CON_CMP,              HOST_TL_PK,  "BHT3B2HB"),
    EVT(SYNC_CON_CHG,              HOST_TL_PK,  "BH2B2H"),
#endif // (MAX_NB_SYNC > 0)
    EVT(SNIFF_SUB,                 HOST_TL_PK,  "B5H"),
#ifdef CFG_INQ
    EVT(EXT_INQ_RES,               HOST_TL_PK, "BT2BDH241B"),
#endif //CFG_INQ
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    EVT(ENC_KEY_REFRESH_CMP,       HOST_UNDEF,  "BH"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    EVT(IO_CAP_REQ,                HOST_TL_PK,  "T"),
    EVT(IO_CAP_RSP,                HOST_TL_PK,  "T3B"),
    EVT(USER_CFM_REQ,              HOST_TL_PK,  "TL"),
    EVT(USER_PASSKEY_REQ,          HOST_TL_PK,  "T"),
    EVT(REM_OOB_DATA_REQ,          HOST_TL_PK,  "T"),
    EVT(SP_CMP,                    HOST_TL_PK,  "BT"),
    EVT(LINK_SUPV_TO_CHG,          HOST_TL_PK,  "2H"),
    EVT(ENH_FLUSH_CMP,             HOST_TL_PK,  "H"),
    EVT(USER_PASSKEY_NOTIF,        HOST_TL_PK,  "TL"),
    EVT(KEYPRESS_NOTIF,            HOST_TL_PK,  "TB"),
    EVT(REM_HOST_SUPP_FEATS_NOTIF, HOST_TL_UPK, "T8B"),
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)

    EVT(AUTH_PAYL_TO_EXP,          HOST_UNDEF,  "H"),

#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
    EVT(SAM_STATUS_CHANGE,         HOST_TL_PK,  "H6B"),
#endif// (BT_EMB_PRESENT || BT_HOST_PRESENT)
#endif // !BT_HOST_OTHER_STACK
};

// Note: remove specific BLE Flag as soon as new debug event available on BT

#if (RW_DEBUG || AUDIO_SYNC_SUPPORT || EAVESDROPPING_SUPPORT)
/// HCI DBG event descriptors table
const hci_evt_desc_t hci_evt_dbg_desc_tab[] =
{
    // Note: all messages must be sorted in opcode ascending order
#if (RW_DEBUG)
    DBG_EVT(DBG_ASSERT,              HOST_TL_UPK,   PK_SPE(DBG_ASSERT)),
    DBG_EVT(DBG_LE_CON_EVT_TRACE,    HOST_NONE,   "BHnH"),
    DBG_EVT(DBG_BT_RSSI_NOTIFY,         HOST_TL_UPK, "B2B79B"),
#endif //(RW_DEBUG)
#if (AUDIO_SYNC_SUPPORT && BLE_CIS)
    DBG_EVT(VS_AUSY_CIS_ESTAB_PARAM, HOST_TL_PK,  AUSY_CIS_ESTAB_PARAM_EVT_PK),
#endif // (AUDIO_SYNC_SUPPORT && BLE_CIS)
#if (AUDIO_SYNC_SUPPORT && BLE_BIS)
    DBG_EVT(VS_AUSY_BIS_ESTAB_PARAM, HOST_TL_PK,  AUSY_BIS_ESTAB_PARAM_EVT_PK),
#endif // (AUDIO_SYNC_SUPPORT)
#if (EAVESDROPPING_SUPPORT)
    DBG_EVT(ECI_ACL_CONNECTION_CHANGED,        HOST_NONE, "B6BBBB"),
    DBG_EVT(ECI_AFH_CHANNEL_MAP_CHANGED,       HOST_NONE, "B6BBL10B"),
    DBG_EVT(ECI_ENCRYPTION_CHANGED,            HOST_NONE, "B6BB16BB8B"),
    DBG_EVT(ECI_PACKET_TYPE_TABLE_CHANGED,     HOST_NONE, "B6BB"),
    DBG_EVT(ECI_MAX_SLOT_CHANGED,              HOST_NONE, "B6BB"),
    DBG_EVT(ECI_SCO_CONNECTION_CHANGED,        HOST_NONE, "B6BBBBBBBBBBHHB"),
    DBG_EVT(ECI_MIN_MAX_POWER_REACHED,         HOST_NONE, "B6BB"),
    DBG_EVT(ECI_MASTER_CLOCK_CAPTURED,         HOST_NONE, "B6BLHLH"),
    DBG_EVT(ECI_AES_CCM_COUNTER,               HOST_NONE, "B6BB5BB"),
    DBG_EVT(ECI_ADJUST_POWER,                  HOST_NONE, "B6BB"),
    DBG_EVT(ECI_CHANNEL_CLASSIFICATION,        HOST_NONE, "B6B10B"),
    DBG_EVT(ECI_MASTER_CLOCK_ADJUSTED,         HOST_NONE, "B6B"),
    DBG_EVT(ECI_RX_TIMEOUT,                    HOST_NONE, "B6B"),
    DBG_EVT(ECI_MIC_INVALID,                   HOST_NONE, "B6B"),
    DBG_EVT(VS_SCO_LINK_REQUEST,               HOST_NONE, "B6B"),
    DBG_EVT(VS_CLOCK_WRAP,                     HOST_NONE, "B6B"),
    DBG_EVT(VS_SNIFF_REQUEST,                  HOST_NONE, "B6BBHHHH"),
    DBG_EVT(VS_SNIFF_MODE_CHANGE,              HOST_NONE, "B6BBBHHHH"),
    DBG_EVT(ECI_TX_PENDING,                    HOST_NONE, "B6BBB"),
    DBG_EVT(ECI_EAVESDROPPING_STATE_CHANGED,   HOST_NONE, "B6BBB"),
    DBG_EVT(ECI_RX_STATS,                      HOST_NONE, "B6BLBnB"),
    DBG_EVT(VS_CUST_TX_TEST_END,               HOST_NONE, "B"),
    DBG_EVT(ECI_ESCO_TX_SEQN_SAMPLED,          HOST_NONE, "B6BBBL"),
    DBG_EVT(ECI_ESCO_TX_SEQN_SYNCED,           HOST_NONE, "B6BB"),
#endif // EAVESDROPPING_SUPPORT
};
#endif // (RW_DEBUG || AUDIO_SYNC_SUPPORT || 0)


#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
/// HCI LE event descriptors table
const hci_evt_desc_t hci_evt_le_desc_tab[] =
{
    // Note: all messages must be sorted in opcode ascending order
    LE_EVT(LE_CON_CMP,                 HOST_TL_UPK,  "2BH8B3HB"),
    LE_EVT(LE_ADV_REPORT,              HOST_TL_UPK,  PK_SPE(LE_ADV_REPORT)),
    LE_EVT(LE_CON_UPDATE_CMP,          HOST_UNDEF,   "2B4H"),
    LE_EVT(LE_RD_REM_FEATS_CMP,        HOST_UNDEF,   "2BH8B"),
    LE_EVT(LE_LTK_REQUEST,             HOST_UNDEF,   "BH8BH"),
    LE_EVT(LE_REM_CON_PARAM_REQ,       HOST_UNDEF,    "B5H"),
    LE_EVT(LE_DATA_LEN_CHG,            HOST_UNDEF,    "B5H"),
    LE_EVT(LE_RD_LOC_P256_PUB_KEY_CMP, HOST_TL_UPK,   "66B"),
    LE_EVT(LE_GEN_DHKEY_CMP,           HOST_TL_UPK,   "34B"),
    LE_EVT(LE_ENH_CON_CMP,             HOST_TL_UPK,   "2BH20B3HB"),
    LE_EVT(LE_DIR_ADV_REP,             HOST_TL_UPK,   "BS116B"),
    LE_EVT(LE_PHY_UPD_CMP,             HOST_UNDEF,    "2BH2B"),
    LE_EVT(LE_EXT_ADV_REPORT,          HOST_TL_UPK,    PK_SPE(LE_EXT_ADV_REPORT)),
    LE_EVT(LE_PER_ADV_SYNC_EST,        HOST_TL_UPK,    "2BH9BHB"),
    LE_EVT(LE_PER_ADV_REPORT,          HOST_TL_UPK,    "BH4BnB"),
    LE_EVT(LE_PER_ADV_SYNC_LOST,       HOST_TL_UPK,    "BH"),
    LE_EVT(LE_SCAN_TIMEOUT,            HOST_TL_UPK,    "B"),
    LE_EVT(LE_ADV_SET_TERMINATED,      HOST_TL_UPK,    "3BHB"),
    LE_EVT(LE_SCAN_REQ_RCVD,           HOST_TL_UPK,    "9B"),
    LE_EVT(LE_CH_SEL_ALGO,             HOST_UNDEF,     "BHB"),

#if (BLE_CONLESS_CTE_RX)
    LE_EVT(LE_CONLESS_IQ_REPORT,         "BHBH4BHS12B"),
#endif // (BLE_CONLESS_CTE_RX)

#if (BLE_CON_CTE_REQ)
    LE_EVT(LE_CON_IQ_REPORT,             "BH2BH4BHS12B"),
    LE_EVT(LE_CTE_REQ_FAILED,            "2BH"),
#endif // (BLE_CON_CTE_REQ)

#if (BLE_PAST)
    LE_EVT(LE_PER_ADV_SYNC_TRANSF_REC, HOST_TL_UPK,  "2B3H9BHB"),
#endif //(BLE_PAST)
#if(BLE_CIS)
    LE_EVT(LE_CIS_ESTABLISHED,         HOST_UNDEF,   "2BHDDDD7B3H"),
#if (BLE_PERIPHERAL)
    LE_EVT(LE_CIS_REQUEST,             HOST_UNDEF,   "B2H2B"),
#endif // (BLE_PERIPHERAL)
#endif // (BLE_CIS)

#if(BLE_BIS)
#if (BLE_BROADCASTER)
    LE_EVT(LE_CREATE_BIG_CMP,          HOST_UNDEF,  "3BDD5B2HnH"),
    LE_EVT(LE_TERMINATE_BIG_CMP,       HOST_UNDEF,  "3B"),
#endif // (BLE_BROADCASTER)

#if (BLE_OBSERVER)
    LE_EVT(LE_BIG_SYNC_ESTABLISHED,    HOST_UNDEF,  "3BD4B2HnH"),
    LE_EVT(LE_BIG_SYNC_LOST,           HOST_UNDEF,  "3B"),
#endif // (BLE_OBSERVER)
#endif // (BLE_BIS)

    LE_EVT(LE_REQ_PEER_SCA_CMP,        HOST_UNDEF,   "2BHB"),

#if BLE_PWR_CTRL
    /// LE Power Control events
    LE_EVT(LE_PATH_LOSS_THRESHOLD,      HOST_UNDEF,  "BH2B"),
    LE_EVT(LE_TX_POWER_REPORTING,       HOST_UNDEF,  "2BH5B"),
#endif // BLE_PWR_CTRL

#if(BLE_BIS)
#if (BLE_BROADCASTER)
    LE_EVT(LE_BIG_INFO_ADV_REPORT,     HOST_TL_UPK,  "BH2BH3BH3BH3B"),
#endif // (BLE_OBSERVER)
#endif // (BLE_BIS)

#if (BT_53)
    LE_EVT(LE_SUBRATE_CHG,             HOST_UNDEF,  "2B5H"),
#endif // (BT_53)
};
#endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)

#if (HCI_TL_SUPPORT)
/// HCI LE event descriptors table
const hci_pkupk_func_t hci_pkupk_func_tab[HCI_MSG_PK_NB] =
{
#if (BT_EMB_PRESENT)
    [HCI_MSG_PK_SET_EVT_FILTER]                 = hci_set_evt_filter_cmd_upk,
#if RW_MWS_COEX
    [HCI_MSG_PK_GET_MWS_TRANSPORT_LAYER_CONFIG] = hci_get_mws_transport_layer_config_cmd_cmp_evt_pk,
#endif //RW_MWS_COEX
#elif (BT_HOST_PRESENT && BT_HOST_OTHER_STACK)
    [HCI_MSG_PK_SET_EVT_FILTER]                 = bts2_hci_set_evt_filter_cmd_upk,
#endif //(BT_EMB_PRESENT)
#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
    [HCI_MSG_PK_LE_ADV_REPORT]                  = hci_le_adv_report_evt_pkupk,
    [HCI_MSG_PK_LE_EXT_ADV_REPORT]              = hci_le_ext_adv_report_evt_pkupk,
    [HCI_MSG_PK_LE_TX_TEST_V4]                  = hci_le_tx_test_v4_cmd_upk,
#endif // (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
#if (RW_DEBUG)
    [HCI_MSG_PK_DBG_ASSERT]                     = hci_dbg_assert_evt_pkupk,
#endif //(RW_DEBUG)
};
#endif //(HCI_TL_SUPPORT)

/*
 * SPECIAL PACKER-UNPACKER DEFINITIONS
 ****************************************************************************************
 */

#if (HCI_TL_SUPPORT)
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
/**
****************************************************************************************
* @brief Apply a basic pack operation
*
* @param[inout] pp_in      Current input buffer position
* @param[inout] pp_out     Current output buffer position
* @param[in]    p_in_end   Input buffer end
* @param[in]    p_out_end  Output buffer end
* @param[in]    len        Number of bytes to copy
*
* @return status
*****************************************************************************************
*/
__STATIC uint8_t hci_pack_bytes(uint8_t **pp_in, uint8_t **pp_out, uint8_t *p_in_end, uint8_t *p_out_end, uint8_t len)
{
    uint8_t status = HCI_PACK_OK;

    // Check if enough space in input buffer to read
    if ((*pp_in + len) > p_in_end)
    {
        status = HCI_PACK_IN_BUF_OVFLW;
    }
    else
    {
        if (p_out_end != NULL)
        {
            // Check if enough space in out buffer to write
            if ((*pp_out + len) > p_out_end)
            {
                status = HCI_PACK_OUT_BUF_OVFLW;
            }

            // Copy BD Address
            memcpy(*pp_out, *pp_in, len);
        }
        *pp_in = *pp_in + len;
        *pp_out = *pp_out + len;
    }

    return (status);
}
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)

#if (BT_EMB_PRESENT)
/// Special packing/unpacking function for HCI Set Event Filter command
__STATIC uint8_t hci_set_evt_filter_cmd_upk(uint8_t *out, uint8_t *in, uint16_t *out_len, uint16_t in_len)
{
    struct hci_set_evt_filter_cmd temp_out;
    struct hci_set_evt_filter_cmd *cmd = (struct hci_set_evt_filter_cmd *) out;
    uint8_t *p_in = in;
    uint8_t *p_out;
    uint8_t *p_in_end = in + in_len;
    uint8_t *p_out_start;
    uint8_t *p_out_end;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if (in != NULL)
    {
        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            cmd = (struct hci_set_evt_filter_cmd *)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
        }
        else
        {
            cmd = (struct hci_set_evt_filter_cmd *)(&temp_out);
            p_out_start = (uint8_t *)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
        }

        do
        {
            // Filter Type
            p_out = &cmd->filter_type;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if (status != HCI_PACK_OK)
                break;

            // Check filter type
            switch (cmd->filter_type)
            {
            case INQUIRY_FILTER_TYPE:
            {
                // Filter Condition Type
                p_out = &cmd->filter.inq_res.cond_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if (status != HCI_PACK_OK)
                    break;

                // Check Filter Condition Type
                switch (cmd->filter.inq_res.cond_type)
                {
                case CLASS_FILTER_CONDITION_TYPE:
                {
                    // Class_of_Device
                    p_out = &cmd->filter.inq_res.cond.cond_1.class_of_dev.A[0];
                    status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, DEV_CLASS_LEN);
                    if (status != HCI_PACK_OK)
                        break;

                    // Class_of_Device_Mask
                    p_out = &cmd->filter.inq_res.cond.cond_1.class_of_dev_msk.A[0];
                    status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, DEV_CLASS_LEN);
                    if (status != HCI_PACK_OK)
                        break;
                }
                break;
                case BD_ADDR_FILTER_CONDITION_TYPE:
                {
                    // BD Address
                    p_out = &cmd->filter.inq_res.cond.cond_2.bd_addr.addr[0];
                    status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, BD_ADDR_LEN);
                    if (status != HCI_PACK_OK)
                        break;
                }
                break;
                default:
                {
                    // Nothing
                }
                break;
                }
            }
            break;
            case CONNECTION_FILTER_TYPE:
            {
                // Filter Condition Type
                p_out = &cmd->filter.con_set.cond_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if (status != HCI_PACK_OK)
                    break;

                // Check Filter Condition Type
                switch (cmd->filter.inq_res.cond_type)
                {
                case ALL_FILTER_CONDITION_TYPE:
                {
                    // Auto_Accept_Flag
                    p_out = &cmd->filter.con_set.cond.cond_0.auto_accept;
                    status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                    if (status != HCI_PACK_OK)
                        break;
                }
                break;
                case CLASS_FILTER_CONDITION_TYPE:
                {
                    // Class_of_Device
                    p_out = &cmd->filter.con_set.cond.cond_1.class_of_dev.A[0];
                    status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, DEV_CLASS_LEN);
                    if (status != HCI_PACK_OK)
                        break;

                    // Class_of_Device_Mask
                    p_out = &cmd->filter.con_set.cond.cond_1.class_of_dev_msk.A[0];
                    status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, DEV_CLASS_LEN);
                    if (status != HCI_PACK_OK)
                        break;

                    // Auto_Accept_Flag
                    p_out = &cmd->filter.con_set.cond.cond_1.auto_accept;
                    status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                    if (status != HCI_PACK_OK)
                        break;
                }
                break;
                case BD_ADDR_FILTER_CONDITION_TYPE:
                {
                    // BD Address
                    p_out = &cmd->filter.con_set.cond.cond_2.bd_addr.addr[0];
                    status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, BD_ADDR_LEN);
                    if (status != HCI_PACK_OK)
                        break;

                    // Auto_Accept_Flag
                    p_out = &cmd->filter.con_set.cond.cond_2.auto_accept;
                    status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                    if (status != HCI_PACK_OK)
                        break;
                }
                break;
                default:
                {
                    // Nothing
                }
                break;
                }
            }
            break;
            default:
            {
                // Nothing
            }
            break;
            }

        }
        while (0);

        *out_len = (uint16_t)(p_out - p_out_start);
    }
    else
    {
        // If no input data, size max is returned
        *out_len =  sizeof(struct hci_set_evt_filter_cmd);
    }

    return (status);
}

#if RW_MWS_COEX
/// Special packing/unpacking function for HCI Set MWS Pattern Configuration command complete event
__STATIC uint8_t hci_get_mws_transport_layer_config_cmd_cmp_evt_pk(uint8_t *out, uint8_t *in, uint16_t *out_len, uint16_t in_len)
{
    struct hci_get_mws_transport_layer_config_cmd_cmp_evt *evt = (struct hci_get_mws_transport_layer_config_cmd_cmp_evt *)(in);
    uint8_t *p_in = in;
    uint8_t *p_out = out;
    uint8_t *p_in_end = in + in_len;
    uint8_t *p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if (in != NULL)
    {
        do
        {
            uint8_t num_transports, num_baud_rates;
            struct mws_trans_rate *rates;

            /*The order of the return parameters in this HCI event packet is:
            Status
            Num_Transports
            Transport_Layer[0]
            Num_Baud_Rates[0]
            . . .
            Transport_Layer[n]
            Num_Baud_Rates[n]
            To_MWS_Baud_Rate[0]
            From_MWS_Baud_Rate[0]
            . . .
            To_MWS_Baud_Rate[m]
            From_MWS_Baud_Rate[m] */

            // Status
            p_in = &evt->status;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if (status != HCI_PACK_OK)
                break;

            // Data Length
            p_in = &evt->num_transports;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if (status != HCI_PACK_OK)
                break;

            num_transports = evt->num_transports;

            for (uint8_t i = 0; i < num_transports; i++)
            {
                // Data
                p_in = &evt->tran[i].layer_id;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if (status != HCI_PACK_OK)
                    break;

                // Num Baud Rates
                p_in = &evt->tran[i].num_baud_rates;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if (status != HCI_PACK_OK)
                    break;

                num_baud_rates = evt->tran[i].num_baud_rates;
                rates = evt->tran[i].rates;

                for (uint8_t j = 0; j < num_baud_rates; j++)
                {
                    // To MWS Rate
                    p_in = (uint8_t *) & (rates[j].to_mws_baud_rate);
                    status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 4);
                    if (status != HCI_PACK_OK)
                        break;

                    // From MWS Rate
                    p_in = (uint8_t *) & (rates[j].from_mws_baud_rate);
                    status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 4);
                    if (status != HCI_PACK_OK)
                        break;
                }
            }

        }
        while (0);

        *out_len = (uint16_t)(p_out - out);
    }
    else
    {
        *out_len = 0;
    }

    return (status);
}
#endif //RW_MWS_COEX
#endif //(BT_EMB_PRESENT)

#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
/// Special packing/unpacking function for HCI LE Advertising Report Event
__STATIC uint8_t hci_le_adv_report_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t *out_len, uint16_t in_len)
{
#if BLE_EMB_PRESENT

    /*
     * PACKING FUNCTION
     */
    struct hci_le_adv_report_evt temp_out;
    struct hci_le_adv_report_evt *s;
    uint8_t *p_in = in;
    uint8_t *p_out;
    uint8_t *p_in_end = in + in_len;
    uint8_t *p_out_start;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if (in != NULL)
    {
        uint8_t *p_out_end;

        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            s = (struct hci_le_adv_report_evt *)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
        }
        else
        {
            s = (struct hci_le_adv_report_evt *)(&temp_out);
            p_out_start = (uint8_t *)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
        }

        p_out = p_out_start;

        do
        {
            // Sub-code
            p_in = &s->subcode;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if (status != HCI_PACK_OK)
                break;

            // Number of reports
            p_in = &s->nb_reports;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if (status != HCI_PACK_OK)
                break;

            for (int i = 0; i < s->nb_reports; i++)
            {
                uint8_t data_len;

                // Event type
                p_in = &s->adv_rep[i].evt_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if (status != HCI_PACK_OK)
                    break;

                // Address type
                p_in = &s->adv_rep[i].adv_addr_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if (status != HCI_PACK_OK)
                    break;

                // BD Address
                p_in = &s->adv_rep[i].adv_addr.addr[0];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, BD_ADDR_LEN);
                if (status != HCI_PACK_OK)
                    break;

                // Data Length
                p_in = &s->adv_rep[i].data_len;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if (status != HCI_PACK_OK)
                    break;

                data_len = s->adv_rep[i].data_len;

                // ADV data
                p_in = &s->adv_rep[i].data[0];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, data_len);
                if (status != HCI_PACK_OK)
                    break;

                // RSSI
                p_in = (uint8_t *) &s->adv_rep[i].rssi;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if (status != HCI_PACK_OK)
                    break;
            }

        }
        while (0);

        *out_len = (uint16_t)(p_out - p_out_start);
    }
    else
    {
        *out_len = 0;
    }

    return (status);

#elif (BLE_HOST_PRESENT)

    /*
     * UNPACKING FUNCTION
     */

    // TODO unpack message as per compiler
    *out_len = in_len;
    if ((out != NULL) && (in != NULL))
    {
        // copy adv report
        memcpy(out, in, in_len);
    }

    return (HCI_PACK_OK);

#endif //BLE_EMB_PRESENT
}

/// Special packing/unpacking function for HCI LE Extended Advertising Report Event
__STATIC uint8_t hci_le_ext_adv_report_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t *out_len, uint16_t in_len)
{
#if BLE_EMB_PRESENT

    /*
     * PACKING FUNCTION
     */
    struct hci_le_ext_adv_report_evt temp_out;
    struct hci_le_ext_adv_report_evt *s;
    uint8_t *p_in = in;
    uint8_t *p_out;
    uint8_t *p_in_end = in + in_len;
    uint8_t *p_out_start;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if (in != NULL)
    {
        uint8_t *p_out_end;

        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            s = (struct hci_le_ext_adv_report_evt *)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
        }
        else
        {
            s = (struct hci_le_ext_adv_report_evt *)(&temp_out);
            p_out_start = (uint8_t *)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
        }

        p_out = p_out_start;

        do
        {
            // Sub-code
            p_in = &s->subcode;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if (status != HCI_PACK_OK)
                break;

            // Number of reports
            p_in = &s->nb_reports;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if (status != HCI_PACK_OK)
                break;

            for (int i = 0; i < s->nb_reports; i++)
            {
                uint8_t data_len;

                // Event type
                p_in = (uint8_t *) &s->adv_rep[i].evt_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
                if (status != HCI_PACK_OK)
                    break;

                // Adv Address type
                p_in = &s->adv_rep[i].adv_addr_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if (status != HCI_PACK_OK)
                    break;

                // Adv Address
                p_in = &s->adv_rep[i].adv_addr.addr[0];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, BD_ADDR_LEN);
                if (status != HCI_PACK_OK)
                    break;

                // Primary PHY
                p_in = &s->adv_rep[i].phy;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if (status != HCI_PACK_OK)
                    break;

                // Secondary PHY
                p_in = &s->adv_rep[i].phy2;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if (status != HCI_PACK_OK)
                    break;

                // Advertising SID
                p_in = &s->adv_rep[i].adv_sid;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if (status != HCI_PACK_OK)
                    break;

                // Tx Power
                p_in = &s->adv_rep[i].tx_power;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if (status != HCI_PACK_OK)
                    break;

                // RSSI
                p_in = (uint8_t *) &s->adv_rep[i].rssi;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if (status != HCI_PACK_OK)
                    break;

                // Periodic Advertising Interval
                p_in = (uint8_t *) &s->adv_rep[i].interval;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
                if (status != HCI_PACK_OK)
                    break;

                // Direct address type
                p_in = (uint8_t *) &s->adv_rep[i].dir_addr_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if (status != HCI_PACK_OK)
                    break;

                // Direct BD Address
                p_in = &s->adv_rep[i].dir_addr.addr[0];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, BD_ADDR_LEN);
                if (status != HCI_PACK_OK)
                    break;

                // Data Length
                p_in = &s->adv_rep[i].data_len;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if (status != HCI_PACK_OK)
                    break;

                data_len = s->adv_rep[i].data_len;

                // ADV data
                p_in = &s->adv_rep[i].data[0];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, data_len);
                if (status != HCI_PACK_OK)
                    break;
            }

        }
        while (0);

        *out_len = (uint16_t)(p_out - p_out_start);
    }
    else
    {
        *out_len = 0;
    }

    return (status);

#elif  (BLE_HOST_PRESENT)

    /*
     * UNPACKING FUNCTION
     */
    //TODO unpack message as per compiler
    *out_len = in_len;
    if ((out != NULL) && (in != NULL))
    {
        // copy adv report
        memcpy(out, in, in_len);
    }

    return (HCI_PACK_OK);

#endif //BLE_EMB_PRESENT
}

/// Special unpacking function for HCI LE Transmitter Test v4 command
__STATIC uint8_t hci_le_tx_test_v4_cmd_upk(uint8_t *out, uint8_t *in, uint16_t *out_len, uint16_t in_len)
{
    uint8_t status = HCI_PACK_OK;
#if (BLE_EMB_PRESENT)
    struct hci_le_tx_test_v4_cmd temp_out;
    struct hci_le_tx_test_v4_cmd *cmd = (struct hci_le_tx_test_v4_cmd *) out;
    uint8_t *p_in = in;
    uint8_t *p_out;
    uint8_t *p_in_end = in + in_len;
    uint8_t *p_out_start;
    uint8_t *p_out_end;

    // Check if there is input data to parse
    if (in != NULL)
    {
        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            cmd = (struct hci_le_tx_test_v4_cmd *)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
        }
        else
        {
            cmd = (struct hci_le_tx_test_v4_cmd *)(&temp_out);
            p_out_start = (uint8_t *)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
        }

        do
        {
            // TX_Channel
            p_out = &cmd->tx_channel;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if (status != HCI_PACK_OK)
                break;

            // Test_Data_Length
            p_out = &cmd->test_data_len;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if (status != HCI_PACK_OK)
                break;

            // Packet_Payload
            p_out = &cmd->pkt_payl;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if (status != HCI_PACK_OK)
                break;

            // PHY
            p_out = &cmd->phy;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if (status != HCI_PACK_OK)
                break;

            // CTE_Length
            p_out = &cmd->cte_len;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if (status != HCI_PACK_OK)
                break;

            // CTE_Type
            p_out = &cmd->cte_type;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if (status != HCI_PACK_OK)
                break;

            // Switching_Pattern_Length
            p_out = &cmd->switching_pattern_len;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if (status != HCI_PACK_OK)
                break;

            // Antenna_IDs[i]
            for (int i = 0; i < cmd->switching_pattern_len; i++)
            {
                p_out = &cmd->antenna_id[i];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if (status != HCI_PACK_OK)
                    break;
            }
            if (status != HCI_PACK_OK)
                break;

            // Transmit_Power_Level
            p_out = (uint8_t *)&cmd->tx_pwr_lvl;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if (status != HCI_PACK_OK)
                break;

        }
        while (0);

        *out_len = (uint16_t)(p_out - p_out_start);
    }
    else
    {
        // If no input data, size max is returned
        *out_len =  sizeof(struct hci_le_tx_test_v4_cmd);
    }
#else
    // TODO Implement
#endif // (BLE_EMB_PRESENT)

    return (status);
}
#endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)

#if (RW_DEBUG)
/// Special packing/unpacking function for HCI DBG assert error Event
__STATIC uint8_t hci_dbg_assert_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t *out_len, uint16_t in_len)
{
    uint8_t status = HCI_PACK_OK;
#if (EMB_PRESENT)
    struct hci_dbg_assert_evt *evt = (struct hci_dbg_assert_evt *)(in);
    uint8_t *p_in = in;
    uint8_t *p_out = out;
    uint8_t *p_in_end = in + in_len;
    uint8_t *p_out_end = out + *out_len;

    // Check if there is input data to parse
    if (in != NULL)
    {
        do
        {
            // Subcode
            p_in = &evt->subcode;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if (status != HCI_PACK_OK)
                break;

            // Type
            p_in = &evt->type;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if (status != HCI_PACK_OK)
                break;

            // Line
            p_in = (uint8_t *) &evt->line;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 4);
            if (status != HCI_PACK_OK)
                break;

            // Param0
            p_in = (uint8_t *) &evt->param0;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 4);
            if (status != HCI_PACK_OK)
                break;

            // Param1
            p_in = (uint8_t *) &evt->param1;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 4);
            if (status != HCI_PACK_OK)
                break;

            // File
            p_in = (uint8_t *) &evt->file;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, strlen((char *) evt->file));
            if (status != HCI_PACK_OK)
                break;
        }
        while (0);

        *out_len = (uint16_t)(p_out - out);
    }
    else
    {
        *out_len = 0;
    }

#elif  (HOST_PRESENT)

    /*
     * UNPACKING FUNCTION
     */
    //TODO unpack message as per compiler

    if ((out != NULL) && (in != NULL))
    {
        struct hci_dbg_assert_evt *evt = (struct hci_dbg_assert_evt *)(out);
        evt->subcode = *in;
        evt->type = *(in + 1);
        evt->line = co_read32(in + 2);
        evt->param0 = co_read32(in + 6);
        evt->param1 = co_read32(in + 10);
        memcpy(evt->file, in + 14, in_len - 14);
        *out_len = sizeof(struct hci_dbg_assert_evt) + in_len - 14;
        // copy adv report
        //memcpy(out, in, in_len);
    }
    else
        *out_len = 0;

    return (HCI_PACK_OK);

#endif // EMB_PRESENT

    return (status);
}
#endif //(RW_DEBUG)
#endif //(HCI_TL_SUPPORT)

#if (RW_DEBUG)
/**
 ****************************************************************************************
 * @brief Check if HCI command descriptor table is sorted in ascending order
 *
 * @param[in] p_desc_tab    Pointer to the table
 * @param[in] nb_cmds       Table length
 *
 * @return true for ascending order, false otherwise
 *****************************************************************************************
 */
__STATIC bool hci_cmd_desc_tab_order_check(const hci_cmd_desc_t *p_desc_tab, uint16_t nb_cmds)
{
    int16_t i;
    // Pointer to previous handler in the table
    const hci_cmd_desc_t *p_prev_handler;
    // Pointer to current handler in the table
    const hci_cmd_desc_t *p_cur_handler;
    // Check result
    bool result = 1;

    ASSERT_ERR(p_desc_tab != NULL);

    p_prev_handler = &(p_desc_tab[0]);
    p_cur_handler = &(p_desc_tab[1]);

    for (i = nb_cmds - 2; i >= 0; i--)
    {
        // ID of previous handler should small than ID of current handler
        if (GETF(p_prev_handler->info_bf, HCI_CMD_INFO_OCF) >= GETF(p_cur_handler->info_bf, HCI_CMD_INFO_OCF))
        {
            result = 0;
            break;
        }

        p_prev_handler++;
        p_cur_handler++;
    }

    return (result);
}

/**
 ****************************************************************************************
 * @brief Check if HCI event descriptor table is sorted in ascending order
 *
 * @param[in] p_desc_tab    Pointer to the table
 * @param[in] nb_cmds       Table length
 *
 * @return true for ascending order, false otherwise
 *****************************************************************************************
 */
__STATIC bool hci_evt_desc_tab_order_check(const hci_evt_desc_t *p_desc_tab, uint16_t nb_cmds)
{
    int16_t i;
    // Pointer to previous handler in the table
    const hci_evt_desc_t *p_prev_handler;
    // Pointer to current handler in the table
    const hci_evt_desc_t *p_cur_handler;
    // Check result
    bool result = 1;

    ASSERT_ERR(p_desc_tab != NULL);

    p_prev_handler = &(p_desc_tab[0]);
    p_cur_handler = &(p_desc_tab[1]);

    for (i = nb_cmds - 2; i >= 0; i--)
    {
        // ID of previous handler should small than ID of current handler
        if (p_prev_handler->code >= p_cur_handler->code)
        {
            result = 0;
            break;
        }

        p_prev_handler++;
        p_cur_handler++;
    }

    return (result);
}
#endif // (RW_DEBUG)


#if (HCI_TL_SUPPORT)

/**
 *****************************************************************************************
 * @brief Pack or unpack input data onto output buffer using "co_pack" format string or using
 *        a specific packer/unpacker funtion
 *
 * @param[in]  p_format    Parameters format or code of specific packer/unpacker
 * @param[out] p_out       Pointer to output Data Buffer
 * @param[in]  p_in        Pointer to input Data Buffer
 * @param[out] p_out_len   Pointer to oOutput size of packed data (in bytes)
 * @param[in]  in_len      Input buffer size (in bytes)
 * @param[in]  pack        True to pack data, False to unpack
 *
 * @return Status of packing/unpacking execution
 *****************************************************************************************
 */
__STATIC uint8_t hci_msg_pkupk(const char *p_format, uint8_t *p_out, uint8_t *p_in, uint16_t *p_out_len, uint16_t in_len,
                               bool pack)
{
    uint8_t status = CO_UTIL_PACK_ERROR;

    // Check if the generic packer can be used (basic fixed-length format)
    if (!IS_PK_SPE(p_format))
    {
        if (pack)
        {
            // Pack the returned parameters using the generic packer
            status = co_util_pack(p_out, p_in, p_out_len, in_len, p_format);
        }
        else
        {
            // UnPack the returned parameters using the generic packer
            status = co_util_unpack(p_out, p_in, p_out_len, in_len, p_format);
        }
    }
    else
    {
        uint8_t pk_id = PK_SPE_ID(p_format);
        if (pk_id < HCI_MSG_PK_NB)
        {
            hci_pkupk_func_t spe_pkuk_func = hci_pkupk_func_tab[pk_id];
            // Pack the return parameters using the special packer
            status = spe_pkuk_func(p_out, p_in, p_out_len, in_len);
        }
    }

    return (status);
}
#endif // (HCI_TL_SUPPORT)

/**
****************************************************************************************
* @brief Look for an event descriptor that could match with the specified event code
*
* @param[in]  code     event code
* @param[in]  p_tab    Pointer to the descriptor tab
* @param[in]  tab_len  Number of element in the tab
*
* @return     Pointer the event descriptor (NULL if not found)
*****************************************************************************************
*/
__STATIC const hci_evt_desc_t *hci_msg_evt_desc_search(uint8_t code, const hci_evt_desc_t *p_tab, uint16_t tab_len)
{
    // Table matched with the event code
    const hci_evt_desc_t *p_desc = NULL;
    // Left boundary index
    int16_t left_bond = 0;
    // Right boundary index
    int16_t right_bond = (int16_t)(tab_len - 1);

    // Get the descriptor table by binary searching the root table
    while (left_bond <= right_bond)
    {
        // Middle element index is the floor of (L + R)/2
        uint16_t mid = (left_bond + right_bond) / 2;

        // Index should not exceed table size
        ASSERT_ERR(mid <= tab_len - 1);

        // Update left boundary if middle element is smaller than target
        if (p_tab[mid].code < code)
        {
            left_bond = mid + 1;
        }
        // Update right boundary if middle element is larger than target
        else if (p_tab[mid].code > code)
        {
            right_bond = mid - 1;
        }
        else
        {
            ASSERT_ERR(&(p_tab[mid]));

            // Get the event descriptor pointer
            p_desc = &(p_tab[mid]);
            break;
        }
    }

    return (p_desc);

}

/*
 * MODULES INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (RW_DEBUG)
void hci_check_desc_tabs_order()
{
    // Sanity check to ensure all descriptor tables are sorted in opcode ascending order

    // Index of root table
    uint16_t root_index;
    // Root table
    const struct hci_cmd_desc_tab_ref *p_root_tab = hci_cmd_desc_root_tab;

    // Check all HCI command descriptor tables
    for (root_index = 0; root_index < ARRAY_LEN(hci_cmd_desc_root_tab); root_index++)
    {
        // Get each descriptor table
        const hci_cmd_desc_t *p_desc_tab = p_root_tab[root_index].cmd_desc_tab;
        // Number of commands in this table
        uint16_t nb_cmds = p_root_tab[root_index].nb_cmds;

        // Only check the order if the table is present
        if ((p_desc_tab != NULL) || (nb_cmds != 0))
        {
            ASSERT_ERR(hci_cmd_desc_tab_order_check(p_desc_tab, nb_cmds));
        }
    }

    // Check HCI event descriptors table
    ASSERT_ERR(hci_evt_desc_tab_order_check(hci_evt_desc_tab, ARRAY_LEN(hci_evt_desc_tab)));

#if (RW_DEBUG || AUDIO_SYNC_SUPPORT || EAVESDROPPING_SUPPORT)
    // Check HCI DBG event descriptors table
    ASSERT_ERR(hci_evt_desc_tab_order_check(hci_evt_dbg_desc_tab, ARRAY_LEN(hci_evt_dbg_desc_tab)));
#endif // (RW_DEBUG || AUDIO_SYNC_SUPPORT || 0)

#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
    // Check HCI LE event descriptors table
    ASSERT_ERR(hci_evt_desc_tab_order_check(hci_evt_le_desc_tab, ARRAY_LEN(hci_evt_le_desc_tab)));
#endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)
}
#endif // (RW_DEBUG)

#if (BT_HOST_OTHER_STACK)
const hci_cmd_desc_t *hci_msg_cmd_desc_get_for_dual_stack(uint16_t opcode)
{
    const hci_cmd_desc_t *p_tab = NULL;
    if (opcode == HCI_RD_BD_ADDR_CMD_OPCODE)
        p_tab = &hci_cmd_desc_tab_info_par_dual_stack[0];

    return p_tab;

}
#endif // BT_HOST_OTHER_STACK

const hci_cmd_desc_t *hci_msg_cmd_desc_get(uint16_t opcode)
{
    // Table matched with the OGF
    const hci_cmd_desc_t *p_tab = NULL;
    // Table match with the OCF
    const hci_cmd_desc_t *p_desc = NULL;
    // Number of commands in the OGF table
    uint16_t nb_cmds = 0;
    // OCF
    uint16_t ocf = HCI_OP2OCF(opcode);
    // OGF
    uint16_t ogf = HCI_OP2OGF(opcode);
    // OGF index in hci_msg_ogf_to_idx tab
    uint8_t ogf_index;
    // element index in hci_cmd_desc_root_tab tab
    uint8_t tab_index = HCI_OGF_IDX_INVALID;

    // Map OGF to its associated table index (Vendor Specific OGF is mapped at a specific table entry)
    ogf_index = (ogf - 1);

    // retrieve corresponding index in descriptor table
    if (ogf_index < HCI_MSG_NB_OGF_SUPP)
    {
        tab_index = hci_msg_ogf_to_idx[ogf_index];
    }
    else if (ogf == VS_OGF)
    {
        tab_index = HCI_OGF_IDX_VS;
    }

    if (tab_index != HCI_OGF_IDX_INVALID)
    {
        // Root table
        const struct hci_cmd_desc_tab_ref *p_root_tab = hci_cmd_desc_root_tab;

        ASSERT_ERR(p_root_tab[tab_index].cmd_desc_tab);

        // Get the command descriptors table information (size and pointer)
        p_tab = p_root_tab[tab_index].cmd_desc_tab;
        nb_cmds = p_root_tab[tab_index].nb_cmds;

        // Check if a table has been found for this OGF
        if (p_tab != NULL)
        {
            // Left boundary index
            int16_t left_bond = 0;
            // Right boundary index
            int16_t right_bond = (int16_t)(nb_cmds - 1);

            // Find the command descriptor associated to this OCF by binary searching
            while (left_bond <= right_bond)
            {
                // Middle element index is the floor of (L + R)/2
                uint16_t mid = (left_bond + right_bond) / 2;
                uint16_t mid_ocf;// = GETF(p_tab[mid].info_bf, HCI_CMD_INFO_OCF);

                // Index should not exceed table size
                ASSERT_ERR(mid <= nb_cmds - 1);

                mid_ocf = GETF(p_tab[mid].info_bf, HCI_CMD_INFO_OCF);

                // Update left boundary if middle element is smaller than target
                if (mid_ocf < ocf)
                {
                    left_bond = mid + 1;
                }
                // Update right boundary if middle element is larger than target
                else if (mid_ocf > ocf)
                {
                    right_bond = mid - 1;
                }
                else
                {
                    ASSERT_ERR(&(p_tab[mid]));

                    // Get the command descriptor pointer
                    p_desc = &(p_tab[mid]);
                    break;
                }
            }
        }
    }

    return (p_desc);
}

/**
****************************************************************************************
* @brief Look for an event descriptor that could match with the specified event code
*
* @param[in]  code   event code
*
* @return     Pointer the event descriptor (NULL if not found)
*****************************************************************************************
*/
const hci_evt_desc_t *hci_msg_evt_desc_get(uint8_t code)
{
    // Table matched with the event code
    const hci_evt_desc_t *p_desc = hci_msg_evt_desc_search(code, hci_evt_desc_tab, ARRAY_LEN(hci_evt_desc_tab));

    return (p_desc);
}

#if (RW_DEBUG || AUDIO_SYNC_SUPPORT || EAVESDROPPING_SUPPORT)
const hci_evt_desc_t *hci_msg_dbg_evt_desc_get(uint8_t subcode)
{
    const hci_evt_desc_t *p_desc = hci_msg_evt_desc_search(subcode, hci_evt_dbg_desc_tab, ARRAY_LEN(hci_evt_dbg_desc_tab));

    return (p_desc);
}
#endif // (RW_DEBUG || AUDIO_SYNC_SUPPORT || 0)


#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
const hci_evt_desc_t *hci_msg_le_evt_desc_get(uint8_t subcode)
{
    const hci_evt_desc_t *p_desc = hci_msg_evt_desc_search(subcode, hci_evt_le_desc_tab, ARRAY_LEN(hci_evt_le_desc_tab));

    return (p_desc);
}
#endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)


#if (HCI_TL_SUPPORT)

#if (EMB_PRESENT)
uint8_t hci_msg_cmd_get_max_param_size(const hci_cmd_desc_t *p_cmd)
{
    return (p_cmd->par_size_max);
}

#endif // (EMB_PRESENT)

#if(HOST_PRESENT)
bool hci_msg_cmd_status_exp(const hci_cmd_desc_t *p_cmd)
{
    return (p_cmd->ret_par_fmt == NULL);
}

#if (!EMB_PRESENT)
uint16_t hci_msg_evt_look_for_conhdl(const hci_evt_desc_t *p_evt_desc, const uint8_t *p_payload)
{
    uint8_t index = 0;
    uint16_t conhdl = GAP_INVALID_CONHDL;
    const char *p_format = p_evt_desc->par_fmt;
    bool finished = false;
    uint8_t num_b = 0;

    while (!finished && (*p_format != '\0'))
    {
        switch (*p_format)
        {
        case 's': // s2 --> array of structure starting with bit field used to compute array size
        case 'S': // S2 --> array of structure starting with byte that contains array size
        case 'B': // could be status
        {
            index++;
            if (num_b < 9)
            {
                index += num_b;
                num_b = 0;
            }
        }
        break;
        case 'H':
        {
            // Read connection handle
            conhdl = co_btohs(co_read16p(p_payload + index));
            finished = true;
        }
        break;
        default:
        {
            num_b = (uint8_t)(*p_format - '1'); // 2B  2H

        }
        break;
        }

        // Increment index
        p_format++;
    }

    return (conhdl);
}
#endif // (!EMB_PRESENT)

#endif //(HOST_PRESENT)

uint8_t hci_msg_cmd_pkupk(const hci_cmd_desc_t *p_cmd, uint8_t *p_out, uint8_t *p_in, uint16_t *p_out_len,
                          uint16_t in_len)
{
    uint8_t status = CO_UTIL_PACK_ERROR;

    if ((p_cmd != NULL) && (p_cmd->par_fmt != NULL))
    {
        bool pack;
        const char *p_format = p_cmd->par_fmt;
#if(EMB_PRESENT)
        pack = false;
#else
        pack = true;
#endif // (EMB_PRESENT)

        // perform packing/unpacking
        status = hci_msg_pkupk(p_format, p_out, p_in, p_out_len, in_len, pack);
    }

    return (status);
}
uint8_t hci_msg_cmd_cmp_pkupk(const hci_cmd_desc_t *p_cmd, uint8_t *p_out, uint8_t *p_in, uint16_t *p_out_len,
                              uint16_t in_len)
{
    uint8_t status = CO_UTIL_PACK_ERROR;

    if ((p_cmd != NULL) && (p_cmd->ret_par_fmt != NULL))
    {
        bool pack;
        const char *p_format = p_cmd->ret_par_fmt;
#if(EMB_PRESENT)
        pack = true;
#else
        pack = false;
#endif // (EMB_PRESENT)
        // perform packing/unpacking
        status = hci_msg_pkupk(p_format, p_out, p_in, p_out_len, in_len, pack);
    }

    return (status);
}

uint8_t hci_msg_evt_pkupk(const hci_evt_desc_t *p_evt, uint8_t *p_out, uint8_t *p_in, uint16_t *p_out_len,
                          uint16_t in_len)
{
    uint8_t status = CO_UTIL_PACK_ERROR;

    if (p_evt != NULL)
    {
        if (p_evt->par_fmt != NULL)
        {
            bool pack;
            const char *p_format = p_evt->par_fmt;
#if(EMB_PRESENT)
            pack = true;
#else
            pack = false;
#endif // (EMB_PRESENT)
            // perform packing/unpacking
            status = hci_msg_pkupk(p_format, p_out, p_in, p_out_len, in_len, pack);
        }
        else if (in_len == 0) // Nothing to pack
        {
            status = CO_UTIL_PACK_OK;
        }
    }

    return (status);
}


#if (EMB_PRESENT)
void hci_msg_cmd_reject_send(const hci_cmd_desc_t *p_cmd, uint16_t opcode, uint8_t error, uint8_t *p_payload)
{
    // Get size of the Command Complete Event message associated with the opcode
    uint16_t ret_par_len = 0;
    uint8_t status;

    if ((p_cmd == NULL) || (p_cmd->ret_par_fmt == NULL))
    {
        // Send a CS event with provided error code
        struct hci_cmd_stat_event *evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, 0, opcode, hci_cmd_stat_event);
        evt->status = error;
        hci_send_2_host(evt);
    }
    else
    {
        // Use Unpacker
        status = hci_msg_pkupk(p_cmd->ret_par_fmt, NULL, NULL, &ret_par_len, 0xFFFF, false);

        if (status == CO_UTIL_PACK_OK)
        {
            // Send a CC event with returned parameters (eventually copy received connection handle or BD address if needed)
            if (!memcmp(p_cmd->ret_par_fmt, "BH", 2))
            {
                uint16_t conhdl = co_btohs(co_read16p(p_payload));
                struct hci_basic_conhdl_cmd_cmp_evt *evt = (struct hci_basic_conhdl_cmd_cmp_evt *) ke_msg_alloc(HCI_CMD_CMP_EVENT, conhdl, opcode, ret_par_len);
                memset(evt, 0, ret_par_len);
                evt->status = error;
                evt->conhdl = conhdl;
                hci_send_2_host(evt);
            }
            else if (!memcmp(p_cmd->ret_par_fmt, "B6B", 3))
            {
                struct hci_basic_bd_addr_cmd_cmp_evt *evt = (struct hci_basic_bd_addr_cmd_cmp_evt *) ke_msg_alloc(HCI_CMD_CMP_EVENT, 0, opcode, ret_par_len);
                memset(evt, 0, ret_par_len);
                evt->status = error;
                memcpy(&evt->bd_addr, p_payload, 6);
                hci_send_2_host(evt);
            }
            else
            {
                struct hci_basic_cmd_cmp_evt *evt = (struct hci_basic_cmd_cmp_evt *) ke_msg_alloc(HCI_CMD_CMP_EVENT, 0, opcode, ret_par_len);
                memset(evt, 0, ret_par_len);
                evt->status = error;
                hci_send_2_host(evt);
            }
        }
        else
        {
            // Send a CC event with provided error code
            struct hci_basic_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_cmd_cmp_evt);
            evt->status = error;
            hci_send_2_host(evt);
        }
    }
}
#endif // (EMB_PRESENT)

#endif // (HCI_TL_SUPPORT)


#if (HOST_PRESENT)
uint8_t hci_msg_evt_get_hl_tl_dest(const hci_evt_desc_t *p_evt)
{
    return (GETF(p_evt->dest_field, HCI_EVT_DEST_HOST_TL));
}

uint8_t hci_msg_evt_host_lid_get(const hci_evt_desc_t *p_evt)
{
    return (p_evt->host_lid);
}
#endif // (HOST_PRESENT)


#if defined(HOST_PRESENT) || defined(EMB_PRESENT)
uint8_t hci_msg_cmd_ll_dest_get(const hci_cmd_desc_t *p_cmd)
{
    return (GETF(p_cmd->info_bf, HCI_CMD_INFO_DEST_LL));
}
#endif

#if (HCI_TL_SUPPORT)
const char *hci_msg_cmd_ret_para_get(const hci_cmd_desc_t *p_cmd)
{
    return p_cmd->ret_par_fmt;
}
#endif


#if (EMB_PRESENT)
uint16_t hci_msg_task_dest_compute(uint8_t ll_dest, uint16_t length, const uint8_t *p_data)
{
    uint16_t task_dest = TASK_NONE;

    switch (ll_dest)
    {
    case MNG:
#if BT_EMB_PRESENT
        if (rom_config_get_bt_controller_enabled() == 0)
        {
            // To adapt dynamic enable/disable BT.
            task_dest = TASK_LLM;
            break;
        }
    case BT_MNG:
    {
        task_dest = rom_config_get_bt_controller_enabled() == 1 ? TASK_LM : TASK_NONE;
    }
    break;
#endif //BT_EMB_PRESENT
#if(BLE_EMB_PRESENT)
    case BLE_MNG:
    {
        task_dest = TASK_LLM;
    }
    break;
#endif // (BLE_EMB_PRESENT)
#if BT_EMB_PRESENT
    case BT_BCST:
    {
        task_dest = rom_config_get_bt_controller_enabled() == 1 ? TASK_LB : TASK_NONE;
    }
    break;
#endif //BT_EMB_PRESENT
#if (BLE_EMB_PRESENT && BLE_ISO_PRESENT)
    case BLE_ISO:
    {
        task_dest = TASK_LLI;
    }
    break;
#endif //(BLE_EMB_PRESENT && BLE_ISO_PRESENT)
#if (EMB_PRESENT)
    case DBG:
    {
        task_dest = TASK_DBG;
    }
    break;
    case COMMON:
    {
        task_dest = TASK_COMMON;
    }
    break;
#endif // (EMB_PRESENT)
#if (EAVESDROPPING_SUPPORT)
    case ED:
    {
        task_dest = TASK_ED;
    }
    break;
#endif // (EAVESDROPPING_SUPPORT)
#if BT_EMB_PRESENT
    case BT_CTRL_BD_ADDR:
    {
        uint8_t link_id;
        if (rom_config_get_bt_controller_enabled() == 0)
            break;

        if (length < sizeof(struct bd_addr)) break;
        // Look for a BD address matching in the table
        for (link_id = 0 ; link_id < MAX_NB_ACTIVE_ACL ; link_id++)
        {
            const struct bd_addr *p_addr = (const struct bd_addr *) p_data;
            hci_bt_acl_con_t *p_bt_con = &(hci_env.bt_acl_con_tab[link_id]);
            // Check BT connection state and BD address (assuming BD address is located at payload 1st 6 bytes)
            if ((p_bt_con->state != HCI_BT_ACL_STATUS_NOT_ACTIVE)
                    && (!memcmp(p_addr, &(p_bt_con->bd_addr.addr), sizeof(struct bd_addr))))
            {
                // Build the destination task ID
                task_dest = KE_BUILD_ID(TASK_LC, link_id);
                break;
            }
        }
    }
    break;
#endif // BT_EMB_PRESENT

#if (HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)
    case CTRL:
#if (HCI_BLE_CON_SUPPORT)
    case BLE_CTRL:
#endif // (HCI_BLE_CON_SUPPORT)
#if BT_EMB_PRESENT
    case BT_CTRL_CONHDL:
#endif //BT_EMB_PRESENT
    {
        uint16_t conhdl;
        // Check if the parameters can contain a connection handle
        if (length < 2) break;
        // Retrieve connection handle from command parameters (expecting at payload 1st 2 bytes)
        conhdl = GETF(co_read16p(p_data), HCI_ACL_HDR_HDL);

#if (BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT))
#if(BLE_CIS)
        // check if Connection Handle is a CIS connection handle
        if ((conhdl >= BLE_CISHDL_MIN) && (conhdl < BLE_BISHDL_MIN))
        {
            task_dest = TASK_LLI;
        }
        else
#endif //(BLE_CIS)
            // Check if the connection handle corresponds to an active BLE link
            if ((conhdl <= BLE_CONHDL_MAX) && hci_env.ble_con_state[conhdl])
            {
                // Build the destination task ID
                task_dest = KE_BUILD_ID(TASK_LLC, conhdl);
            }
#endif //(BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT))

#if BT_EMB_PRESENT
        // Check if the connection handle corresponds to an active BT link (ACL or SCO)
        if (rom_config_get_bt_controller_enabled() == 1)
        {
            conhdl &= ~(BT_SYNC_CONHDL_MSK);
            if ((conhdl >= BT_ACL_CONHDL_MIN) && (conhdl <= BT_ACL_CONHDL_MAX))
            {
                if (hci_env.bt_acl_con_tab[(conhdl - BT_ACL_CONHDL_MIN)].state == HCI_BT_ACL_STATUS_BD_ADDR_CONHDL)
                {
                    // Build the destination task ID
                    task_dest = KE_BUILD_ID(TASK_LC, conhdl - BT_ACL_CONHDL_MIN);
                }
            }
        }
#endif //BT_EMB_PRESENT
    }
    break;
#endif //(HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)

    default:
    {
        ASSERT_ERR_FORCE(0); // Unknown LL destination
    }
    break;
    }

    return (task_dest);
}
#endif // (EMB_PRESENT)
#endif //(HCI_PRESENT)


/// @} HCI
