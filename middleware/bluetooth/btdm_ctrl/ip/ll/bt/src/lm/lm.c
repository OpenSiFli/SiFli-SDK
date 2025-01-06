/**
 ****************************************************************************************
 *
 * @file lm.c
 *
 * @brief Definition of the functions used by the link manager
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#include <string.h>
#include "co_bt.h"          // BT standard definitions
#include "co_utils.h"
#include "co_math.h"

#include "ke_mem.h"             // kernel memory
#include "ke_timer.h"
#include "ke_event.h"
#include "ecc_p256.h"       // Elliptic curve calculation P256
#include "ecc_p192.h"       // Elliptic curve calculation P192

#include "lm.h"          // link manager definitions
#include "lm_int.h"      // link manager internal definitions

#include "lc.h"          // link driver definitions
#include "ld.h"          // link driver definitions

#include "sha.h"              // sha-2 based functions

#if HCI_PRESENT
    #include "hci.h"            // host controller interface
#endif //HCI_PRESENT
#include "btdm_patch.h"

/*
 * DEFINES
 ****************************************************************************************
 */



/*
 * CONSTANTS DEFINITION
 *****************************************************************************************
 */




/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LM environment variable
struct lm_env_tag lm_env;

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/// P256 public key generated
__STATIC void lm_gen_p256_pub_key_completed(uint32_t metainfo, const ecc_p256_result_t *p_res)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LM_1_PATCH_TYPE, LM_GEN_P256_PUB_KEY_COMPLETED_1_FUN_BIT, metainfo, p_res);

    struct hci_rd_loc_oob_ext_data_cmd_cmp_evt *p_evt;
    lm_oob_data_t *p_oob_data = lm_env.p_oob_data;
    uint8_t Z = 0x00;

    // copy generated public key
    memcpy(p_oob_data->pub_key_256.x, p_res->key_res_x, PUB_KEY_256_LEN / 2);
    memcpy(p_oob_data->pub_key_256.y, p_res->key_res_y, PUB_KEY_256_LEN / 2);

    // Generate new random for P-256
    lm_generate_rand_16bytes((struct byte16 *) & (p_oob_data->rand_256));

    // allocate the complete event message
    p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_RD_LOC_OOB_EXT_DATA_CMD_OPCODE, hci_rd_loc_oob_ext_data_cmd_cmp_evt);
    p_evt->status = CO_ERROR_NO_ERROR;


    // Compute commitment data for P-192
    memcpy(&p_evt->oob_r_192, p_oob_data->rand_192.R, KEY_LEN);
    sha_256_f1(false, p_oob_data->pub_key_192.x, p_oob_data->pub_key_192.x,
               p_oob_data->rand_192.R, &Z, p_evt->oob_c_192.C);

    // Compute commitment data for P-256
    memcpy(&p_evt->oob_r_256, p_oob_data->rand_256.R, KEY_LEN);
    sha_256_f1(true, p_oob_data->pub_key_256.x, p_oob_data->pub_key_256.x,
               p_oob_data->rand_256.R, &Z, p_evt->oob_c_256.C);

    // send message
    hci_send_2_host(p_evt);
}

/// P192 public key generated
__STATIC void lm_gen_p192_pub_key_completed(uint32_t metainfo, uint8_t status, const ecc_p192_result_t *p_res)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LM_1_PATCH_TYPE, LM_GEN_P192_PUB_KEY_COMPLETED_1_FUN_BIT, metainfo, p_res);

    lm_oob_data_t *p_oob_data = lm_env.p_oob_data;

    memcpy(p_oob_data->pub_key_192.x, p_res->key_res_x, PUB_KEY_192_LEN / 2);
    memcpy(p_oob_data->pub_key_192.y, p_res->key_res_y, PUB_KEY_192_LEN / 2);


    // Generate new random for P-192
    lm_generate_rand_16bytes((struct byte16 *) & (p_oob_data->rand_192));


    if (p_oob_data->is_256_generated)
    {
        // Generate p256 key pair
        ecc_p256_gen_key_pair(lm_env.hci.sp_debug_mode, p_oob_data->priv_key_256.key, 0, lm_gen_p256_pub_key_completed);
    }
    else
    {
        uint8_t Z = 0x00;
        // allocate the complete event message
        struct hci_rd_loc_oob_data_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_RD_LOC_OOB_DATA_CMD_OPCODE,
                hci_rd_loc_oob_data_cmd_cmp_evt);
        p_evt->status = CO_ERROR_NO_ERROR;

        // Compute commitment data for P-192
        memcpy(&p_evt->oob_r, p_oob_data->rand_192.R, KEY_LEN);
        sha_256_f1(false, p_oob_data->pub_key_192.x, p_oob_data->pub_key_192.x,
                   p_oob_data->rand_192.R,  &Z, p_evt->oob_c.C);

        // send message
        hci_send_2_host(p_evt);
    }
}

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void lm_init(bool reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LM_1_PATCH_TYPE, LM_INIT_1_FUN_BIT, reset);

    uint8_t local_name[BD_NAME_SIZE + 1];
    uint8_t length;

    if (reset)
    {
        // Check if a name is present
        if (lm_env.local_name != NULL)
        {
            // Free current name buffer
            ke_free(lm_env.local_name);
            lm_env.local_name = NULL;
        }

        // Check if a HCI Create Connection command is stored
        if (lm_env.create_con_cmd != NULL)
        {
            ke_msg_free(ke_param2msg(lm_env.create_con_cmd));
        }

#if CSB_SUPPORT
        // Check if a HCI Truncated Page command is stored
        if (lm_env.trunc_page_cmd != NULL)
        {
            ke_msg_free(ke_param2msg(lm_env.trunc_page_cmd));
        }
#endif //CSB_SUPPORT

        // Check if a HCI Remote Name Request command is stored
        if (lm_env.rem_name_req_cmd != NULL)
        {
            ke_msg_free(ke_param2msg(lm_env.rem_name_req_cmd));
        }

        // Check if OOB data is present
        if (lm_env.p_oob_data != NULL)
        {
            ke_free(lm_env.p_oob_data);
        }

        // Clear AFH timer
        ke_timer_clear(LM_AFH_TO, TASK_LM);
    }

    memset(&lm_env, 0, sizeof(lm_env));

    // Initialize non-zero HCI configuration parameters
    lm_env.hci.page_to = PAGE_TO_DFT;
    lm_env.hci.inq_scan_intv = INQ_SCAN_INTV_DFT;
    lm_env.hci.inq_scan_win = INQ_SCAN_WIN_DFT;
    lm_env.hci.page_scan_intv = PAGE_SCAN_INTV_DFT;
    lm_env.hci.page_scan_win = PAGE_SCAN_WIN_DFT;
    lm_env.hci.page_scan_rep_mode = R1;
    lm_env.hci.inq_tx_pwr_lvl = INQ_TX_PWR_DBM_DFT;
    lm_env.hci.iac_lap.A[0] = GIAC_LAP_0;
    lm_env.hci.iac_lap.A[1] = GIAC_LAP_1;
    lm_env.hci.iac_lap.A[2] = GIAC_LAP_2;
#if (EAVESDROPPING_SUPPORT)
    lm_env.hci.host_max_slot = 5;
#endif // EAVESDROPPING_SUPPORT
    lm_env.sam_info.active_index = SAM_DISABLED;
#if (BT_53)
    lm_env.hci.min_enc_key_size = MIN_ENC_KEY_SIZE_DEFAULT;
#if (RW_DEBUG)
    lm_env.hci.max_enc_key_size = ENC_KEY_SIZE_MAX;
#endif // (RW_DEBUG)
#endif // (BT_53)

    // Initialize Host channel classification
    memset(&lm_env.afh.host_ch_class.map[0], 0xFF, BT_CH_MAP_LEN);

#if (EAVESDROPPING_SUPPORT)
    memset(&lm_env.afh.ed_ch_class.map[0], 0x00, BT_CH_CLASS_MAP_LEN);
#endif // EAVESDROPPING_SUPPORT

    // Local name)
    length = BD_NAME_SIZE;
    if (rwip_param.get(PARAM_ID_DEVICE_NAME, &length, &local_name[0]) == PARAM_OK)
    {
        // Allocate a buffer from Heap
        lm_env.local_name = ke_malloc_system(CO_ALIGN4_HI(length + 1), KE_MEM_ENV);

        if (lm_env.local_name != NULL)
        {
            // Copy name to buffer
            memcpy(&lm_env.local_name[0], &local_name[0], length);
            lm_env.local_name[length] = '\0';

            // Lib functions strlen() has been found to use 32-bit memory accesses, so can perform read-access a few bytes beyond end of string
            DBG_MEM_INIT(&lm_env.local_name[length], CO_ALIGN4_HI(length + 1) - length);
        }
    }

    // Number of link keys stored in Controller
    length = PARAM_LEN_BT_LINK_KEY_NB;
    if (rwip_param.get(PARAM_ID_BT_LINK_KEY_NB, &length, &lm_env.nb_stored_link_keys) != PARAM_OK)
    {
        lm_env.nb_stored_link_keys = 0;
    }

#if (MAX_NB_SYNC > 0)
    // Activity move configuration
    uint8_t act_move_cfg;
    length = PARAM_LEN_ACTIVITY_MOVE_CONFIG;
    if (rwip_param.get(PARAM_ID_ACTIVITY_MOVE_CONFIG, &length, &act_move_cfg) != PARAM_OK)
    {
        // If no value is set in persistent storage enable the feature by default
        lm_env.sco_move_en = true;
    }
    else
    {
        // Bit 1 denotes support for (e)SCO links
        lm_env.sco_move_en = (act_move_cfg & 0x02);
    }
#endif //(MAX_NB_SYNC > 0)

    // Create LM task
    ke_task_create(TASK_LM, &TASK_DESC_LM);

    ke_state_set(TASK_LM, LM_IDLE);
}

uint8_t lm_lt_addr_alloc(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LM_1_PATCH_TYPE, LM_LT_ADDR_ALLOC_1_FUN_BIT, uint8_t);

    uint8_t i = 0;

    for (i = LT_ADDR_MIN ; i <= LT_ADDR_MAX ; i++)
    {
        if (!(lm_env.lt_addr_bitmap & (1 << i)))
        {
            break;
        }
    }

    if (i > LT_ADDR_MAX)
    {
        i = 0;
    }
    else
    {
        lm_env.lt_addr_bitmap |= (1 << i);
    }

    return (i);
}

#if CSB_SUPPORT
bool lm_lt_addr_reserve(uint8_t lt_addr)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LM_1_PATCH_TYPE, LM_LT_ADDR_RESERVE_1_FUN_BIT, bool, lt_addr);

    bool reserved = false;

    if ((lt_addr >= LT_ADDR_MIN) && (lt_addr <= LT_ADDR_MAX) && !(lm_env.lt_addr_bitmap & (1 << lt_addr)))
    {
        lm_env.lt_addr_bitmap |= (1 << lt_addr);
        reserved = true;
    }

    return reserved;
}
#endif

void lm_lt_addr_free(uint8_t lt_addr)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LM_1_PATCH_TYPE, LM_LT_ADDR_FREE_1_FUN_BIT, lt_addr);

    if ((lt_addr >= LT_ADDR_MIN) && (lt_addr <= LT_ADDR_MAX))
    {
        lm_env.lt_addr_bitmap &= (~(1 << lt_addr));
    }
    else
    {
        ASSERT_INFO_FORCE(0, lt_addr, lm_env.lt_addr_bitmap);
    }
}

uint8_t lm_get_nb_acl(uint8_t acl_flag)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LM_1_PATCH_TYPE, LM_GET_NB_ACL_1_FUN_BIT, uint8_t, acl_flag);

    uint8_t slave_nb = 0, master_nb = 0, link_nb = 0;
    register uint8_t link_id = 0;

    while (link_id < MAX_NB_ACTIVE_ACL)
    {
        if (lm_env.con_info[link_id].state >= LM_CONNECTED)
        {
            if (lm_env.con_info[link_id].role == SLAVE_ROLE)
            {
                slave_nb++;
            }
            else
            {
                master_nb++;
            }
        }
        link_id++;
    }

    if (acl_flag & MASTER_FLAG)
    {
        link_nb = master_nb;
    }

    if (acl_flag & SLAVE_FLAG)
    {
        link_nb += slave_nb;
    }

    return (link_nb);
}

bool lm_role_switch_start(uint8_t link_id, uint8_t *lt_addr)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LM_2_PATCH_TYPE, LM_ROLE_SWITCH_START_2_FUN_BIT, bool, link_id, lt_addr);

    bool switch_allowed = true;

    // Slave->Master: allocate LT address
    if (lm_env.con_info[link_id].role == SLAVE_ROLE)
    {
        // Allocate LT address
        lm_env.con_info[link_id].lt_addr = lm_lt_addr_alloc();

        // Check if free LT address found
        if (lm_env.con_info[link_id].lt_addr == 0x00)
        {
            switch_allowed = false;
        }
        else
        {
            *lt_addr = lm_env.con_info[link_id].lt_addr;
        }
    }

    if (switch_allowed)
    {
        // Indicate the connection is under Role Switch
        lm_env.con_info[link_id].state = LM_SWITCH;
    }

    return (switch_allowed);
}

void lm_role_switch_finished(uint8_t link_id, bool success)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LM_2_PATCH_TYPE, LM_ROLE_SWITCH_FINISHED_2_FUN_BIT, link_id, success);
    if (success)
    {
        if (lm_env.con_info[link_id].role == MASTER_ROLE)
        {
            // Free LT Address
            lm_lt_addr_free(lm_env.con_info[link_id].lt_addr);
            lm_env.con_info[link_id].lt_addr = 0x00;
        }
        else
        {
            // Activate AFH timer if needed
            lm_afh_activate_timer();
        }

        // Invert connection role
        lm_env.con_info[link_id].role = !lm_env.con_info[link_id].role;
    }
    else if (lm_env.con_info[link_id].role == SLAVE_ROLE)
    {
        // Free LT Address
        lm_lt_addr_free(lm_env.con_info[link_id].lt_addr);
        lm_env.con_info[link_id].lt_addr = 0x00;
    }

    // Clear the connection state to connected
    lm_env.con_info[link_id].state = LM_CONNECTED;

#if PCA_SUPPORT && RW_BT_MWS_COEX
    if (lm_local_ext_fr_configured())
    {
        // Reconfigure target offset for MWS uplink/downlink based on new role
        ld_pca_update_target_offset(link_id);
        ld_pca_reporting_enable(true);
    }
#endif //PCA_SUPPORT && RW_BT_MWS_COEX
}

void lm_read_features(uint8_t page_nb, uint8_t *page_nb_max, struct features *feats)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LM_2_PATCH_TYPE, LM_READ_FEATURES_2_FUN_BIT, page_nb, page_nb_max, feats);
    // Maximum page number
    if (page_nb_max != NULL)
    {
        *page_nb_max = FEATURE_PAGE_MAX - 1;
    }

    if (page_nb < FEATURE_PAGE_MAX)
    {
        // Get features page
        memcpy(&feats->feats[0], &lm_local_supp_feats[page_nb][0], FEATS_LEN);

        // Add variable bits
        switch (page_nb)
        {
        case 0:
        {
            // No variable bit
        }
        break;
        case 1:
        {
            feats->feats[0] |= ((lm_env.hci.sp_mode              << B0_HOST_SSP_POS)  & B0_HOST_SSP_MSK);
            feats->feats[0] |= ((lm_env.hci.le_supported_host    << B0_HOST_LE_POS)  & B0_HOST_LE_MSK);
            feats->feats[0] |= ((lm_env.hci.simultaneous_le_host << B0_HOST_LE_BR_EDR_POS)  & B0_HOST_LE_BR_EDR_MSK);
            feats->feats[0] |= ((lm_env.hci.sec_con_host_supp    << B0_HOST_SECURE_CON_POS) & B0_HOST_SECURE_CON_MSK);
        }
        break;
        case 2:
        {
            // No variable bit
        }
        break;
        default:
        {
            ASSERT_INFO_FORCE(0, page_nb, (FEATURE_PAGE_MAX - 1));
        }
        break;
        }
    }
}

void lm_acl_disc(uint8_t link_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LM_1_PATCH_TYPE, LM_ACL_DISC_1_FUN_BIT, link_id);
    // Free link identifier
    lm_env.con_info[link_id].state = LM_FREE;

    if (lm_env.con_info[link_id].role == MASTER_ROLE)
    {
        // Free LT address
        lm_lt_addr_free(lm_env.con_info[link_id].lt_addr);
    }

    // Unregister linkID at HCI level
    hci_bt_acl_bdaddr_unregister(link_id);

    // Check if page scan is enabled
    if (lm_env.hci.scan_en & PAGE_SCAN_ENABLE)
    {
        // Try to start page scan (will be rejected by driver if already active)
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

bool lm_get_auth_en(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LM_1_PATCH_TYPE, LM_GET_AUTH_EN_1_FUN_BIT, bool);
    return (lm_env.hci.auth_en == AUTH_ENABLED);
}

bool lm_get_sp_en(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LM_1_PATCH_TYPE, LM_GET_SP_EN_1_FUN_BIT, bool);
    return (lm_env.hci.sp_mode == SP_MODE_EN);
}

bool lm_get_sec_con_host_supp(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LM_1_PATCH_TYPE, LM_GET_SEC_CON_HOST_SUPP_1_FUN_BIT, bool);
    return (lm_env.hci.sec_con_host_supp);
}

uint8_t LM_GetPINType(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LM_1_PATCH_TYPE, LM_GETPINTYPE_1_FUN_BIT, uint8_t);
    return (lm_env.hci.pin_type);
}

void LM_GetLocalNameSeg(struct name_vect *NameSeg, uint8_t NameOffset, uint8_t *NameLen)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LM_1_PATCH_TYPE, LM_GETLOCALNAMESEG_1_FUN_BIT, NameSeg, NameOffset, NameLen);
    uint8_t Index;

    *NameLen = (lm_env.local_name != NULL) ? strlen(lm_env.local_name) : 0;

    // Reset the name segment
    memset(&NameSeg->vect[0], 0x00, NAME_VECT_SIZE);

    // Fill data from stored local name
    for (Index = 0 ;
            (Index < NAME_VECT_SIZE) &&
            ((Index + NameOffset) < BD_NAME_SIZE) &&
            ((Index + NameOffset) < *NameLen);
            Index++)
    {
        NameSeg->vect[Index] = lm_env.local_name[Index + NameOffset];
    }
}

uint8_t lm_get_loopback_mode(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LM_1_PATCH_TYPE, LM_GET_LOOPBACK_MODE_1_FUN_BIT, uint8_t);
    return lm_env.hci.loopback_mode;
}

void lm_gen_local_oob_data(bool gen_oob_256)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LM_1_PATCH_TYPE, LM_GEN_LOCAL_OOB_DATA_1_FUN_BIT, gen_oob_256);
    lm_oob_data_t *p_oob_data = lm_env.p_oob_data;

    // allocate new oob data if no pointer already available
    if (p_oob_data == NULL)
    {
        p_oob_data = ke_malloc_system(sizeof(lm_oob_data_t), KE_MEM_KE_MSG);
    } // else overwrite existing OOB data

    lm_env.p_oob_data = p_oob_data;
    p_oob_data->is_256_generated = gen_oob_256;

    // Generate p192 key pair
    ecc_p192_gen_key_pair(lm_env.hci.sp_debug_mode, p_oob_data->priv_key_192.key, 0, lm_gen_p192_pub_key_completed);
}

lm_oob_data_t *lm_sp_get_local_oob_data(void)
{
    //FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LM_2_PATCH_TYPE, LM_SP_GET_LOCAL_OOB_DATA_2_FUN_BIT, uint32_t);
    return lm_env.p_oob_data;
}

void lm_sp_release_local_oob_data(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LM_2_PATCH_TYPE, LM_SP_RELEASE_LOCAL_OOB_DATA_2_FUN_BIT);
    // Check if OOB data is present
    if (lm_env.p_oob_data != NULL)
    {
        ke_free(lm_env.p_oob_data);
        lm_env.p_oob_data = NULL;
    }
}

uint8_t lm_sp_debug_mode_get(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LM_2_PATCH_TYPE, LM_SP_DEBUG_MODE_GET_2_FUN_BIT, uint8_t);
    return (lm_env.hci.sp_debug_mode);
}

void lm_generate_rand_16bytes(struct byte16 *number)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LM_1_PATCH_TYPE, LM_GENERATE_RAND_16BYTES_1_FUN_BIT, number);
    sha_256_hash_t hash;
    uint32_t message[7];

    // Fill buffer with random bytes
    message[1] = co_rand_word();  // 4:7
    message[2] = ld_read_clock(); // 8:11
    message[3] = co_rand_word();  // 12:15
    message[4] = co_rand_word();  // 16:19
    message[5] = co_rand_word();  // 20:23
    message[6] = co_rand_word();  // 24:27
    ld_bd_addr_get((struct bd_addr *)message); // 0:5

    /* Calculate the Hash value of it and return                                        */
    sha_256((uint8_t *) message, 7 * sizeof(uint32_t), &hash);
    memcpy(number, hash.value, sizeof(struct byte16));
}

bool lm_debug_key_compare_192(uint8_t *X, uint8_t *Y)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LM_1_PATCH_TYPE, LM_DEBUG_KEY_COMPARE_192_1_FUN_BIT, bool, X, Y);
    return ((memcmp(X, ecc_p192_get_debug_pub_key_x(), PUB_KEY_192_LEN / 2) == 0)
            && (memcmp(Y, ecc_p192_get_debug_pub_key_y(), PUB_KEY_192_LEN / 2) == 0));
}

bool lm_debug_key_compare_256(uint8_t *X, uint8_t *Y)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LM_1_PATCH_TYPE, LM_DEBUG_KEY_COMPARE_256_1_FUN_BIT, bool, X, Y);
    return ((memcmp(X, ecc_p256_get_debug_pub_key_x(), PUB_KEY_256_LEN / 2) == 0)
            && (memcmp(Y, ecc_p256_get_debug_pub_key_y(), PUB_KEY_256_LEN / 2) == 0));
}

bool lm_look_for_stored_link_key(struct bd_addr *p_bd_addr, struct ltk *Key)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LM_1_PATCH_TYPE, LM_LOOK_FOR_STORED_LINK_KEY_1_FUN_BIT, bool, p_bd_addr, Key);
    bool key_present = false;

    // Parse all entries in storage
    for (int i = 0; i < lm_env.nb_stored_link_keys ; i++)
    {
        uint8_t buffer[PARAM_LEN_BT_LINK_KEY];
        uint8_t length = PARAM_LEN_BT_LINK_KEY;

        // Fetch entry from storage
        if (rwip_param.get(PARAM_ID_BT_LINK_KEY_FIRST + i, &length, buffer) == PARAM_OK)
        {
            // Check BD address
            if (!memcmp(&p_bd_addr->addr[0], &buffer[0], BD_ADDR_LEN))
            {
                // Copy link key
                if (Key != NULL)
                    memcpy(&Key->ltk[0], &buffer[BD_ADDR_LEN], KEY_LEN);

                key_present = true;
                break;
            }
        }
    }

    return key_present;
}

bool lm_dut_mode_en_get(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LM_1_PATCH_TYPE, LM_DUT_MODE_EN_GET_1_FUN_BIT, bool);
    return lm_env.hci.dut_mode_en;
}

bool lm_sync_flow_ctrl_en_get(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LM_2_PATCH_TYPE, LM_SYNC_FLOW_CTRL_EN_GET_2_FUN_BIT, bool);
    return lm_env.hci.sync_flow_ctrl_en;
}

struct bt_ch_map *lm_afh_host_ch_class_get(void)
{
    //FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LM_1_PATCH_TYPE, LM_AFH_HOST_CH_CLASS_GET_1_FUN_BIT, uint32_t);
    return &lm_env.afh.host_ch_class;
}

#if (EAVESDROPPING_SUPPORT)

void lm_afh_ed_ch_class_set(struct bt_ch_map const *ch_class)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LM_1_PATCH_TYPE, LM_AFH_ED_CH_CLASS_SET_1_FUN_BIT, ch_class);
    memcpy(&lm_env.afh.ed_ch_class.map[0], &ch_class->map[0], BT_CH_CLASS_MAP_LEN);
}

struct bt_ch_map *lm_afh_ed_ch_class_get(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LM_1_PATCH_TYPE, LM_AFH_ED_CH_CLASS_GET_1_FUN_BIT, uint32_t);
    return &lm_env.afh.ed_ch_class;
}


struct bd_addr *lm_bd_addr_get(uint8_t link_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LM_1_PATCH_TYPE, LM_BD_ADDR_GET_1_FUN_BIT, uint32_t, link_id);
    return (&lm_env.con_info[link_id].bd_addr);
}

uint8_t lm_get_host_max_slot(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LM_1_PATCH_TYPE, LM_GET_HOST_MAX_SLOT_1_FUN_BIT, uint8_t);
    return (lm_env.hci.host_max_slot);
}

#endif // EAVESDROPPING_SUPPORT

void lm_afh_activate_timer(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LM_1_PATCH_TYPE, LM_AFH_ACTIVATE_TIMER_1_FUN_BIT);
    if (!lm_env.afh.active)
    {
        lm_env.afh.active = true;

        // Restart AFH timer
        ke_timer_set(LM_AFH_TO, TASK_LM, BT_AFH_UPDATE_PERIOD * 1000);
    }
}

bool lm_is_acl_con(uint8_t link_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LM_1_PATCH_TYPE, LM_IS_ACL_CON_1_FUN_BIT, bool, link_id);
    return (lm_env.con_info[link_id].state == LM_CONNECTED);
}

bool lm_is_acl_con_role(uint8_t link_id, uint8_t role)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LM_1_PATCH_TYPE, LM_IS_ACL_CON_ROLE_1_FUN_BIT, bool, link_id, role);
    return ((lm_env.con_info[link_id].state == LM_CONNECTED) && (lm_env.con_info[link_id].role == role));
}

uint8_t lm_page_scan_rep_mode_get(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LM_2_PATCH_TYPE, LM_PAGE_SCAN_REP_MODE_GET_2_FUN_BIT, uint8_t);
    return lm_env.hci.page_scan_rep_mode;
}

#if RW_BT_MWS_COEX
bool lm_local_ext_fr_configured(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LM_1_PATCH_TYPE, LM_LOCAL_EXT_FR_CONFIGURED_1_FUN_BIT, bool);
    return lm_env.local_ext_fr_config;
}
#endif //RW_BT_MWS_COEX

uint8_t lm_sam_submap_rx_slots_get(uint8_t t_sam_sm)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LM_2_PATCH_TYPE, LM_SAM_SUBMAP_RX_SLOTS_GET_2_FUN_BIT, uint8_t, t_sam_sm);
    uint8_t byte_idx;
    uint8_t bit_pos;

    uint8_t rx_slots = 0;

    for (int i = 0; i < t_sam_sm; i++)
    {
        byte_idx = i >> 2; // 4 sam slots per byte
        bit_pos = (i & 0x3) << 1; // 2 bits fields

        if (lm_env.sam_info.type0submap[byte_idx] & (SAM_SLOT_RX_AVAILABLE << bit_pos))
            rx_slots++;
    }

    return rx_slots;
}

uint8_t lm_sam_submap_tx_slots_get(uint8_t t_sam_sm)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LM_2_PATCH_TYPE, LM_SAM_SUBMAP_TX_SLOTS_GET_2_FUN_BIT, uint8_t, t_sam_sm);
    uint8_t byte_idx;
    uint8_t bit_pos;

    uint8_t tx_slots = 0;

    for (int i = 0; i < t_sam_sm; i++)
    {
        byte_idx = i >> 2; // 4 sam slots per byte
        bit_pos = (i & 0x3) << 1; // 2 bits fields

        if (lm_env.sam_info.type0submap[byte_idx] & (SAM_SLOT_TX_AVAILABLE << bit_pos))
            tx_slots++;
    }

    return tx_slots;
}

uint8_t *lm_sam_submap0_get(void)
{
    //FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LM_2_PATCH_TYPE, LM_SAM_SUBMAP0_GET_2_FUN_BIT, uint32_t);
    return &lm_env.sam_info.type0submap[0];
}

uint8_t *lm_sam_pattern_get(uint8_t pattern_idx)
{
    //FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LM_2_PATCH_TYPE, LM_SAM_PATTERN_GET_2_FUN_BIT, uint32_t, pattern_idx);
    return &lm_env.sam_info.pattern[pattern_idx].submaps.map[0];
}

#if (MAX_NB_SYNC > 0)
bool lm_sco_move_en(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LM_2_PATCH_TYPE, LM_SCO_MOVE_EN_2_FUN_BIT, bool);
    return lm_env.sco_move_en;
}
#endif //(MAX_NB_SYNC > 0)

#if (EAVESDROPPING_SUPPORT)
/// Find the link id from MAC address
uint8_t lm_find_link_id(struct bd_addr bd_addr)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LM_1_PATCH_TYPE, LM_FIND_LINK_ID_1_FUN_BIT, uint8_t, bd_addr);
    uint8_t link_id;

    // Find link identifier associated to BD Address
    for (link_id = 0 ; link_id < MAX_NB_ACTIVE_ACL ; link_id++)
    {
        // Check state
        if (((lm_env.con_info[link_id].state == LM_CONNECTED) || (lm_env.con_info[link_id].state == LM_SWITCH))
                && !memcmp(&lm_env.con_info[link_id].bd_addr.addr[0], &bd_addr.addr[0], BD_ADDR_LEN))
        {
            break;
        }
    }
    return link_id;
}
#endif // EAVESDROPPING_SUPPORT

void lm_ch_map_compute(struct bt_ch_map *ch_map)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LM_1_PATCH_TYPE, LM_CH_MAP_COMPUTE_1_FUN_BIT, ch_map);
    // Get Host channel classification
    struct bt_ch_map *host_ch_class = lm_afh_host_ch_class_get();

    // Get local channel map based on assessment
    rwip_ch_map_bt_get(ch_map);

    // Compute the channel map
    for (int i = 0 ; i < AFH_NB_CHANNEL_MAX ; i++)
    {
        /*
         * Each bit of the host channel classification corresponds to:
         *      0: bad channel
         *      1: unknown channel (treated as good)
         */

        uint8_t byte_idx = i >> 3;
        uint8_t bit_pos = i & 0x7;

        // Host indicates the channel as bad
        if (((host_ch_class->map[byte_idx] >> bit_pos) & 0x1) == 0x00)
        {
            ch_map->map[byte_idx] &= ~(1 << bit_pos);
            continue;
        }

#if (EAVESDROPPING_SUPPORT)
        /*
         * Decision made from slave classification report:
         *  - pair bad     => disable
         *  - pair unknown => enable
         *  - pair good    => enable
         */
        // Eavesdropper (ED) indicates the channel as bad
        // if (((lm_env.afh.ed_ch_class.map[byte_idx] >> bit_pos) & 0x1) == 0x00)
        if ((((lm_env.afh.ed_ch_class.map[byte_idx]) >> (bit_pos & 0x06)) & 0x3) == AFH_CH_CLASS_BAD)
        {
            ch_map->map[byte_idx] &= ~(1 << bit_pos);
            continue;
        }
#endif // EAVESDROPPING_SUPPORT
    }

#if PCA_SUPPORT
    // For all devices that may use coarse clock adjustment recovery mode, the RF channel indices used for
    //  the synchronization train shall be marked as unused in the AFH_channel_map for logical links LM 4.1.14.1
    ch_map->map[SYNC_TRAIN_CHANNEL_0 >> 3] &= ~(1 << (SYNC_TRAIN_CHANNEL_0 & 0x7));
    ch_map->map[SYNC_TRAIN_CHANNEL_1 >> 3] &= ~(1 << (SYNC_TRAIN_CHANNEL_1 & 0x7));
    ch_map->map[SYNC_TRAIN_CHANNEL_2 >> 3] &= ~(1 << (SYNC_TRAIN_CHANNEL_2 & 0x7));
#endif // PCA_SUPPORT
}

void lm_fill_ch_map(struct bt_ch_map *ch_map)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LM_1_PATCH_TYPE, LM_FILL_CH_MAP_1_FUN_BIT, ch_map);
    uint8_t nbchgood;

    // Count number of good channels
    nbchgood = co_nb_good_channels(ch_map);

    // Check if the map has a sufficient number of used channels
    if (nbchgood < AFH_NB_CHANNEL_MIN)
    {
        rwip_activate_channels_ch_map_bt((AFH_NB_CHANNEL_MIN - nbchgood), lm_afh_host_ch_class_get(), ch_map);
    }
}
/// @} LM
